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

bool AimCanDriver::packMsg(const aim::Msg& msg, CanCoreFrame& can_msg) {
  AIM_ASSERT(static_cast<uint8_t>(msg.source) != 0U);

  const uint8_t prio = aim::priorityFor(msg.cls, msg.subject);
  can_msg.id = aim::encodeId(prio, msg.cls, msg.subject, msg.source);
  can_msg.dlc = 8U;

  // Both MCUs are little-endian — straight copies, no byte swapping (spec).
  (void)memcpy(&can_msg.data[0], &msg.timestampMs, sizeof(msg.timestampMs));
  (void)memcpy(&can_msg.data[4], msg.b, sizeof(msg.b));
  return true;
}

bool AimCanDriver::unpackMsg(const CanCoreFrame& can_msg, aim::Msg& msg) {
  AIM_ASSERT((can_msg.id & ~aim::kExtIdMask) == 0U);

  if (can_msg.dlc != 8U) {
    return false;
  }

  uint8_t prio = 0U;
  if (!aim::decodeId(can_msg.id, msg, prio)) {
    return false;
  }

  (void)memcpy(&msg.timestampMs, &can_msg.data[0], sizeof(msg.timestampMs));
  (void)memcpy(msg.b, &can_msg.data[4], sizeof(msg.b));
  return true;
}

void AimCanDriver::logFailure(bool isBeginFailure, uint32_t canId) const {
  (void)isBeginFailure;
  (void)canId;

#ifndef FLIGHT_BUILD
  const char* const op = isBeginFailure ? "begin" : "tx";

#if defined(ARDUINO_ARCH_STM32)
  AimStm32CanCore::Stats stats = {};
  _canCore.getStats(stats);
  LOG_ERROR(
      "AimCanDriver fail op=%s id=0x%08lX beginErr=%lu txErr=%lu rxErr=%lu drops=%lu filtered=%lu busOff=%lu warn=%lu passive=%lu err=0x%08lX",
      op,
      static_cast<unsigned long>(canId),
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
      "AimCanDriver fail op=%s id=0x%08lX beginErr=%lu txErr=%lu rxErr=%lu drops=%lu filtered=%lu busOff=%lu warn=%lu passive=%lu err=0x%08lX",
      op,
      static_cast<unsigned long>(canId),
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

bool AimCanDriver::begin(uint16_t classAcceptMask) {
  if (_initialized) {
    return true;
  }

  if (!_canCore.setClassMask(classAcceptMask)) {
    LOG_ERROR("AimCanDriver begin failed: invalid class mask=0x%04X", static_cast<unsigned>(classAcceptMask));
    return false;
  }

  _initialized = _canCore.begin();
  if (!_initialized) {
    logFailure(true);
  }

  return _initialized;
}

bool AimCanDriver::transmit(const aim::Msg& msg) {
  if (!_initialized) {
    LOG_ERROR("AimCanDriver transmit failed: driver not initialized");
    return false;
  }

  CanCoreFrame can_msg = {};
  if (!packMsg(msg, can_msg)) {
    LOG_ERROR("AimCanDriver transmit failed: packMsg rejected frame");
    return false;
  }

  const bool sent = _canCore.transmit(can_msg);
  if (!sent) {
    logFailure(false, can_msg.id);
  }

  return sent;
}

bool AimCanDriver::receive(aim::Msg& msg) {
  if (!_initialized) {
    LOG_ERROR("AimCanDriver receive failed: driver not initialized");
    return false;
  }

  CanCoreFrame can_msg = {};
  if (!_canCore.receive(can_msg)) {
    return false;
  }

  if (!unpackMsg(can_msg, msg)) {
    LOG_ERROR(
        "AimCanDriver receive failed: unpack rejected id=0x%08lX dlc=%u",
        static_cast<unsigned long>(can_msg.id),
        static_cast<unsigned>(can_msg.dlc));
    return false;
  }

  return true;
}
