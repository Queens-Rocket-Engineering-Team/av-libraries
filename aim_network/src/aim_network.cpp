#include "aim_network.h"

#include "aim_can_driver.h"
#include "aim_safety.h"
#include <logger.h>

AimNetwork::AimNetwork(AimCanDriver* hardware, aim::Source self)
    : _hw(hardware),
      _self(self),
      _timeOffset(0),
      _lastTxMs(0U) {
  AIM_ASSERT(static_cast<uint8_t>(self) != 0U);
  AIM_ASSERT(static_cast<uint8_t>(self) <= 0xEU);
}

bool AimNetwork::begin(uint16_t classAcceptMask) {
  if (_hw == nullptr) {
    LOG_ERROR("AimNetwork begin failed: hardware driver is null");
    return false;
  }

  return _hw->begin(classAcceptMask);
}

bool AimNetwork::send(aim::Msg& m) {
  if (_hw == nullptr) {
    LOG_ERROR("AimNetwork send failed: hardware driver is null");
    return false;
  }

  m.source = _self;
  m.timestampMs = syncedMillis();

  const bool sent = _hw->transmit(m);
  if (sent) {
    _lastTxMs = millis();  // local clock on purpose — synced time steps
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

  if (!_hw->receive(m)) {
    return false;
  }

  // Only the TimeSync subject disciplines the clock; other (future) Time-class
  // subjects must not step every clock on the bus.
  if ((m.cls == aim::Class::Time) && (m.subject == aim::subject::TimeSync)) {
    syncTime(m.timestampMs);
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
