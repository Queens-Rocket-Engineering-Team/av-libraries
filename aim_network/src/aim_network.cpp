#include "aim_network.h"

#if defined(ARDUINO_ARCH_STM32)
  #include "aim_stm32_can_core.h"
#elif defined(ARDUINO_ARCH_ESP32)
  #include "aim_esp32_can_core.h"
#endif

#include "aim_safety.h"
#include <logger.h>
#include "aim_job.h"

namespace aim {
  static bool g_isHighDataRate = false;

  bool isHighDataRate() { return g_isHighDataRate; }
  void setHighDataRate(bool active) { g_isHighDataRate = active; }
}

AimNetwork::AimNetwork(AimCanHardware* hardware, aim::Source self)
    : _hw(hardware),
      _self(self),
      _timeOffset(0),
      _lastTxMs(0U),
      _lastSyncTimeMs(0xFFFFFFFFU) {
  AIM_ASSERT(static_cast<uint8_t>(self) != 0U);
  AIM_ASSERT(static_cast<uint8_t>(self) <= 0xEU);
}

bool AimNetwork::begin(uint16_t classAcceptMask) {
  if (_hw == nullptr) {
    LOG_ERROR("AimNetwork begin failed: hardware driver is null");
    return false;
  }

  if (!_hw->setClassMask(classAcceptMask)) {
    LOG_ERROR("AimNetwork begin failed: setClassMask rejected mask=0x%04X", static_cast<unsigned>(classAcceptMask));
    return false;
  }

  return _hw->begin();
}

bool AimNetwork::send(aim::Msg& m) {
  if (_hw == nullptr) {
    LOG_ERROR("AimNetwork send failed: hardware driver is null");
    return false;
  }

  m.source = _self;
  m.timestampMs = syncedMillis();

  if ((m.cls == aim::Class::Time) && (m.subject == aim::subject::TimeSync)) {
    (void)memcpy(m.b, &m.timestampMs, sizeof(m.timestampMs));
    m.offsetMs = 0;
    _lastSyncTimeMs = m.timestampMs;
  } else if (aim::isZeroTimestamp(m.cls, m.subject)) {
    m.offsetMs = 0;
  } else {
    if (_lastSyncTimeMs == 0xFFFFFFFFU) {
      m.offsetMs = 0xFFFFU;
    } else {
      uint32_t delta = m.timestampMs - _lastSyncTimeMs;
      if (delta > 65535U) {
        m.offsetMs = 0xFFFFU;
      } else {
        m.offsetMs = static_cast<uint16_t>(delta);
      }
    }
  }

  aim::Frame frame = {};
  if (!aim::packMsg(m, frame)) {
    return false;
  }

  const bool sent = _hw->transmit(frame);
  if (sent) {
    _lastTxMs = millis();  // local clock on purpose — synced time steps
    if ((m.cls == aim::Class::Event) && (m.subject == aim::subject::TelemetryMode)) {
      aim::setHighDataRate(m.b[0] == 1U);
    }
  } else {
    LOG_ERROR("AimNetwork send failed: CAN transmit returned false");
  }

  return sent;
}

bool AimNetwork::receive(aim::Msg& m) {
  if (_hw == nullptr) {
    LOG_ERROR("AimNetwork receive failed: hardware driver is null");
    return false;
  }

  aim::Frame frame = {};
  if (!_hw->receive(frame)) {
    return false;
  }

  if (!aim::unpackMsg(frame, m)) {
    return false;
  }

  // Only the TimeSync subject disciplines the clock; other (future) Time-class
  // subjects must not step every clock on the bus.
  if ((m.cls == aim::Class::Time) && (m.subject == aim::subject::TimeSync)) {
    uint32_t remoteTimeMs = 0U;
    (void)memcpy(&remoteTimeMs, m.b, sizeof(remoteTimeMs));
    syncTime(remoteTimeMs);
    _lastSyncTimeMs = remoteTimeMs;
    m.timestampMs = remoteTimeMs;
  } else {
    if ((m.cls == aim::Class::Event) && (m.subject == aim::subject::TelemetryMode)) {
      aim::setHighDataRate(m.b[0] == 1U);
    }

    if (aim::isZeroTimestamp(m.cls, m.subject)) {
      m.timestampMs = syncedMillis();
    } else {
      if (_lastSyncTimeMs == 0xFFFFFFFFU) {
        m.timestampMs = 0U;
      } else {
        m.timestampMs = _lastSyncTimeMs + m.offsetMs;
      }
    }
  }

  return true;
}

void AimNetwork::service(uint32_t nowMs, aim::NodeState state, uint16_t errorBits) {
  // Liveness = any valid frame; heartbeat only fills silence (spec: > T/2).
  // Local clock on purpose — synced time steps on resync and at midnight.
  if ((nowMs - _lastTxMs) <= (aim::kHeartbeatTxIntervalMs / 2U)) {
    return;
  }

  aim::Msg m = {};
  m.cls = aim::Class::Heartbeat;
  m.subject = aim::subject::Heartbeat;
  m.b[0] = static_cast<uint8_t>(state);
  m.b[1] = static_cast<uint8_t>(errorBits & 0xFFU);
  m.b[2] = static_cast<uint8_t>(errorBits >> 8U);
  m.b[3] = aim::kSchemaVersion;
  (void)send(m);
}

void AimNetwork::syncTime(uint32_t remoteMillis) {
  _timeOffset = static_cast<int32_t>(remoteMillis) - static_cast<int32_t>(millis());
}

uint32_t AimNetwork::syncedMillis() const {
  return millis() + _timeOffset;
}

void AimNetwork::setHighDataRate(bool active) {
  aim::setHighDataRate(active);
}
