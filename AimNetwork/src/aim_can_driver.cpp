#include "aim_can_driver.h"

#include <cstring>
#include <logger.h>

#if defined(ARDUINO_ARCH_STM32)

AimCanDriver::AimCanDriver(uint8_t origin, uint32_t baud, CAN_TypeDef* canbus)
    : _origin(origin),
      _initialized(false),
      _canCore(baud, canbus) {
}

void AimCanDriver::getStm32Stats(AimStm32CanCore::Stats& stats) const {
  _canCore.getStats(stats);
}

void AimCanDriver::clearStm32Stats() {
  _canCore.clearStats();
}

#elif defined(ARDUINO_ARCH_ESP32)

AimCanDriver::AimCanDriver(uint8_t origin, uint32_t baud, int rxPin, int txPin)
    : _origin(origin),
      _initialized(false),
      _canCore(baud, rxPin, txPin) {
}

void AimCanDriver::getEsp32Stats(AimEsp32CanCore::Stats& stats) const {
  _canCore.getStats(stats);
}

void AimCanDriver::clearEsp32Stats() {
  _canCore.clearStats();
}

#endif

bool AimCanDriver::packAimPkt(const aimPkt& aim_pkt, CanCoreFrame& can_msg) {
  AIM_ASSERT((aim_pkt.origin & 0xF8U) == 0U);
  AIM_ASSERT((aim_pkt.dest & 0xF8U) == 0U);
  AIM_ASSERT((aim_pkt.type & 0xF0U) == 0U);

  can_msg.id = (((aim_pkt.origin & 0x07U) << 8) |
                ((aim_pkt.dest   & 0x07U) << 5) |
                ((aim_pkt.type   & 0x0FU))) & 0x07FFU;
  can_msg.dlc = static_cast<uint8_t>(sizeof(aim_pkt.data));
  if (can_msg.dlc != 8U) {
    return false;
  }

  (void)memcpy(can_msg.data, &aim_pkt.data, sizeof(aim_pkt.data));
  return true;
}

bool AimCanDriver::unpackAimPkt(const CanCoreFrame& can_msg, aimPkt& aim_pkt) {
  AIM_ASSERT((can_msg.id & 0xF800U) == 0U);
  AIM_ASSERT(can_msg.dlc <= 8U);

  aim_pkt.origin = static_cast<uint8_t>((can_msg.id >> 8) & 0x07U);
  aim_pkt.dest = static_cast<uint8_t>((can_msg.id >> 5) & 0x07U);
  aim_pkt.type = static_cast<uint8_t>(can_msg.id & 0x0FU);

  if (sizeof(aim_pkt.data) != can_msg.dlc) {
    return false;
  }

  (void)memcpy(&aim_pkt.data, can_msg.data, can_msg.dlc);
  return true;
}

void AimCanDriver::logFailure(bool isBeginFailure, uint16_t canId) const {
  const char* const op = isBeginFailure ? "begin" : "tx";

#if defined(ARDUINO_ARCH_STM32)
  AimStm32CanCore::Stats stats = {};
  _canCore.getStats(stats);
  LOG_ERROR(
      "AimCanDriver fail op=%s org=%u id=0x%03X beginErr=%lu txErr=%lu rxErr=%lu drops=%lu filtered=%lu busOff=%lu warn=%lu passive=%lu err=0x%08lX",
      op,
      static_cast<unsigned>(_origin),
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
      "AimCanDriver fail op=%s org=%u id=0x%03X beginErr=%lu txErr=%lu rxErr=%lu drops=%lu filtered=%lu busOff=%lu warn=%lu passive=%lu err=0x%08lX",
      op,
      static_cast<unsigned>(_origin),
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
}

void AimCanDriver::begin() {
  if (_initialized) {
    return;
  }

  AIM_ASSERT((_origin & 0xF8U) == 0U);

  if (!_canCore.setAcceptDest(_origin)) {
    LOG_ERROR("AimCanDriver begin failed: invalid destination origin=%u", static_cast<unsigned>(_origin));
    _initialized = false;
    return;
  }

  _initialized = _canCore.begin();
  if (!_initialized) {
    logFailure(true);
  }
}

bool AimCanDriver::transmit(const uint8_t* buf, size_t len) {
  AIM_ASSERT((buf != nullptr) && (len == sizeof(aimPkt)));

  if ((buf == nullptr) || (len != sizeof(aimPkt))) {
    LOG_ERROR("AimCanDriver transmit failed: invalid buffer or size (%u)", static_cast<unsigned>(len));
    return false;
  }

  if (!_initialized) {
    LOG_ERROR("AimCanDriver transmit failed: driver not initialized");
    return false;
  }

  aimPkt pkt;
  (void)memcpy(&pkt, buf, sizeof(pkt));

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

bool AimCanDriver::receive(uint8_t* buf, size_t len) {
  AIM_ASSERT((buf != nullptr) && (len == sizeof(aimPkt)));

  if ((buf == nullptr) || (len != sizeof(aimPkt))) {
    LOG_ERROR("AimCanDriver receive failed: invalid buffer or size (%u)", static_cast<unsigned>(len));
    return false;
  }

  if (!_initialized) {
    LOG_ERROR("AimCanDriver receive failed: driver not initialized");
    return false;
  }

  CanCoreFrame can_msg = {};
  if (!_canCore.receive(can_msg)) {
    return false;
  }

  aimPkt pkt;
  if (!unpackAimPkt(can_msg, pkt)) {
    LOG_ERROR(
        "AimCanDriver receive failed: unpack rejected id=0x%03X dlc=%u",
        static_cast<unsigned>(can_msg.id),
        static_cast<unsigned>(can_msg.dlc));
    return false;
  }

  (void)memcpy(buf, &pkt, sizeof(pkt));
  return true;
}