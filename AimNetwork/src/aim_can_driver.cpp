#include "aim_can_driver.h"

#include <cstring>
#include <logger.h>

#if defined(ARDUINO_ARCH_STM32)

AimCanDriver::AimCanDriver(uint32_t baud, CAN_TypeDef* canbus)
    : _initialized(false),
      _canCore(baud, canbus) {
}

void AimCanDriver::getStm32Stats(AimStm32CanCore::Stats& stats) const {
  _canCore.getStats(stats);
}

void AimCanDriver::clearStm32Stats() {
  _canCore.clearStats();
}

#elif defined(ARDUINO_ARCH_ESP32)

AimCanDriver::AimCanDriver(uint32_t baud, int rxPin, int txPin)
    : _initialized(false),
      _canCore(baud, rxPin, txPin) {
}

void AimCanDriver::getEsp32Stats(AimEsp32CanCore::Stats& stats) const {
  _canCore.getStats(stats);
}

void AimCanDriver::clearEsp32Stats() {
  _canCore.clearStats();
}

#endif

bool AimCanDriver::packAimPkt(const aim::Pkt& aim_pkt, CanCoreFrame& can_msg) {
  AIM_ASSERT(static_cast<uint8_t>(aim_pkt.origin) <= aim::kNodeMax);
  AIM_ASSERT(static_cast<uint8_t>(aim_pkt.dest)   <= aim::kNodeMax);
  AIM_ASSERT(static_cast<uint8_t>(aim_pkt.type)   <= aim::kTypeMax);

  can_msg.id = (((static_cast<uint16_t>(aim_pkt.origin) & 0x07U) << 8) |
                ((static_cast<uint16_t>(aim_pkt.dest)   & 0x07U) << 5) |
                ((static_cast<uint16_t>(aim_pkt.type)   & 0x0FU))) & 0x07FFU;
  can_msg.dlc = static_cast<uint8_t>(sizeof(aim_pkt.data));
  if (can_msg.dlc != 8U) {
    return false;
  }

  (void)memcpy(can_msg.data, &aim_pkt.data, sizeof(aim_pkt.data));
  return true;
}

bool AimCanDriver::unpackAimPkt(const CanCoreFrame& can_msg, aim::Pkt& aim_pkt) {
  AIM_ASSERT((can_msg.id & 0xF800U) == 0U);
  AIM_ASSERT(can_msg.dlc <= 8U);

  aim_pkt.origin = static_cast<aim::Node>((can_msg.id >> 8) & 0x07U);
  aim_pkt.dest = static_cast<aim::Node>((can_msg.id >> 5) & 0x07U);
  aim_pkt.type = static_cast<aim::PacketType>(can_msg.id & 0x0FU);

  if (sizeof(aim_pkt.data) != can_msg.dlc) {
    return false;
  }

  (void)memcpy(&aim_pkt.data, can_msg.data, can_msg.dlc);
  return true;
}

void AimCanDriver::logFailure(bool isBeginFailure, uint16_t canId) const {
  (void)isBeginFailure;
  (void)canId;

#ifndef FLIGHT_BUILD
  const char* const op = isBeginFailure ? "begin" : "tx";

#if defined(ARDUINO_ARCH_STM32)
  AimStm32CanCore::Stats stats = {};
  _canCore.getStats(stats);
  LOG_ERROR(
      "AimCanDriver fail op=%s id=0x%03X beginErr=%lu txErr=%lu rxErr=%lu drops=%lu filtered=%lu busOff=%lu warn=%lu passive=%lu err=0x%08lX",
      op,
      static_cast<unsigned>(canId),
      0UL,
      static_cast<unsigned long>(stats.txHalErrors),
      static_cast<unsigned long>(stats.rxHalErrors),
      static_cast<unsigned long>(stats.txQueueDrops),
      0UL,
      static_cast<unsigned long>(stats.busOffEvents),
      static_cast<unsigned long>(stats.errorWarningEvents),
      static_cast<unsigned long>(stats.errorPassiveEvents),
      static_cast<unsigned long>(stats.lastHalError));
#elif defined(ARDUINO_ARCH_ESP32)
  AimEsp32CanCore::Stats stats = {};
  _canCore.getStats(stats);
  LOG_ERROR(
      "AimCanDriver fail op=%s id=0x%03X beginErr=%lu txErr=%lu rxErr=%lu drops=%lu filtered=%lu busOff=%lu warn=%lu passive=%lu err=0x%08lX",
      op,
      static_cast<unsigned>(canId),
      static_cast<unsigned long>(stats.beginErrors),
      static_cast<unsigned long>(stats.txErrors),
      static_cast<unsigned long>(stats.rxErrors),
      0UL,
      static_cast<unsigned long>(stats.filteredFrames),
      0UL,
      0UL,
      0UL,
      static_cast<unsigned long>(stats.lastError));
#endif

#endif // FLIGHT_BUILD
}

void AimCanDriver::begin(aim::Node acceptDest) {
  if (_initialized) {
    return;
  }

  if (!_canCore.setAcceptDest(static_cast<uint8_t>(acceptDest))) {
    LOG_ERROR("AimCanDriver begin failed: invalid destination id=%u", static_cast<unsigned>(acceptDest));
    _initialized = false;
    return;
  }

  _initialized = _canCore.begin();
  if (!_initialized) {
    logFailure(true);
  }
}

bool AimCanDriver::transmit(const aim::Pkt& pkt) {
  if (!_initialized) {
    LOG_ERROR("AimCanDriver transmit failed: driver not initialized");
    return false;
  }

  CanCoreFrame can_msg = {};
  if (!packAimPkt(pkt, can_msg)) {
    LOG_ERROR("AimCanDriver transmit failed: packAimPkt rejected frame");
    return false;
  }

  const bool sent = _canCore.transmit(can_msg);
  if (!sent) {
    logFailure(false, can_msg.id);
  }

  return sent;
}

bool AimCanDriver::receive(aim::Pkt& pkt) {
  if (!_initialized) {
    LOG_ERROR("AimCanDriver receive failed: driver not initialized");
    return false;
  }

  CanCoreFrame can_msg = {};
  if (!_canCore.receive(can_msg)) {
    return false;
  }

  if (!unpackAimPkt(can_msg, pkt)) {
    LOG_ERROR(
        "AimCanDriver receive failed: unpack rejected id=0x%03X dlc=%u",
        static_cast<unsigned>(can_msg.id),
        static_cast<unsigned>(can_msg.dlc));
    return false;
  }

  return true;
}
