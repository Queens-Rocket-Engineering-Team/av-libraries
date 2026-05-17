#include "aim_network.h"

#include "aim_can_driver.h"
#include <logger.h>

AimNetwork::AimNetwork(AimCanDriver* hardware, uint8_t origin)
    : _hw(hardware),
      _origin(origin),
      _timeOffset(0),
      _trackedOrigins(nullptr),
      _trackedCount(0U),
      _healthTable(nullptr),
      _heartbeatTimeoutMs(0U) {
}

void AimNetwork::begin() {
  if (_hw != nullptr) {
    _hw->begin();
  }
}

bool AimNetwork::sendPkt(const aimPkt& pkt) {
  if (_hw == nullptr) {
    LOG_ERROR("AimNetwork sendPkt failed: hardware driver is null");
    return false;
  }

  const bool sent = _hw->transmit(pkt);
  if (!sent) {
    LOG_ERROR("AimNetwork sendPkt failed: CAN transmit returned false");
  }
  return sent;
}

bool AimNetwork::sendPkt64(uint64_t data, uint8_t dest, uint8_t type) {
  if ((dest > AIM_DEST_ADDR_MAX) || (type > AIM_TYP_ADDR_MAX)) {
    LOG_ERROR("AimNetwork send failed: invalid dest/type (dest=0x%02X type=0x%02X)",
              static_cast<unsigned int>(dest),
              static_cast<unsigned int>(type));
    return false;
  }

  aimPkt pkt = {};
  pkt.origin = _origin;
  pkt.dest = dest;
  pkt.type = type;
  pkt.data = data;
  return sendPkt(pkt);
}

bool AimNetwork::sendPkt32(uint32_t ms, uint32_t payload, uint8_t dest, uint8_t type) {
  return sendPkt32Ex(0U, ms, payload, dest, type);
}

bool AimNetwork::sendPkt32Ex(uint32_t endpointId, uint32_t ms, uint32_t payload, uint8_t dest, uint8_t type) {
  if ((dest > AIM_DEST_ADDR_MAX) || (type > AIM_TYP_ADDR_MAX)) {
    LOG_ERROR("AimNetwork send failed: invalid dest/type (dest=0x%02X type=0x%02X)",
              static_cast<unsigned int>(dest),
              static_cast<unsigned int>(type));
    return false;
  }

  if (endpointId > AIM_PKT_TIMED_ENDPOINT_MAX) {
    LOG_ERROR("AimNetwork sendPkt32Ex failed: endpointId exceeds %u (endpointId=%u)",
              static_cast<unsigned int>(AIM_PKT_TIMED_ENDPOINT_MAX),
              static_cast<unsigned int>(endpointId));
    return false;
  }

  if (ms > AIM_PKT_TIMED_MILLIS_MAX) {
    LOG_ERROR("AimNetwork sendPkt32Ex failed: ms exceeds %lu (ms=%lu)",
              static_cast<unsigned long>(AIM_PKT_TIMED_MILLIS_MAX),
              static_cast<unsigned long>(ms));
    return false;
  }

  return sendPkt64(aimPkt::packDataEx(endpointId, ms, payload), dest, type);
}

bool AimNetwork::readPkt(aimPkt& pkt) {
  if (_hw == nullptr) {
    LOG_ERROR("AimNetwork readPkt failed: hardware driver is null");
    return false;
  }

  return _hw->receive(pkt);
}

void AimNetwork::syncTime(uint32_t remoteMillis) {
  _timeOffset = static_cast<int32_t>(remoteMillis) - static_cast<int32_t>(millis());
}

bool AimNetwork::configureHealthMonitor(const uint8_t* trackedOrigins,
                                        uint8_t trackedCount,
                                        AimNodeHealth* healthStorage,
                                        uint32_t heartbeatTimeoutMs,
                                        uint32_t nowMs) {
  if ((trackedOrigins == nullptr) ||
      (healthStorage == nullptr) ||
      (trackedCount == 0U) ||
      (heartbeatTimeoutMs == 0U)) {
    LOG_ERROR("AimNetwork configureHealthMonitor failed: invalid input arguments");
    return false;
  }

  for (uint8_t i = 0U; i < trackedCount; i++) {
    if (trackedOrigins[i] > AIM_ORG_ADDR_MAX) {
      LOG_ERROR("AimNetwork configureHealthMonitor failed: tracked origin out of range");
      return false;
    }
  }

  _trackedOrigins = trackedOrigins;
  _trackedCount = trackedCount;
  _healthTable = healthStorage;
  _heartbeatTimeoutMs = heartbeatTimeoutMs;

  for (uint8_t i = 0U; i < _trackedCount; i++) {
    AimNodeHealth& health = _healthTable[i];
    health.origin = _trackedOrigins[i];
    if (health.origin == _origin) {
      health.lastHeartbeatMs = nowMs;
      health.alive = true;
    } else {
      health.lastHeartbeatMs = 0U;
      health.alive = false;
    }
  }

  return true;
}

int16_t AimNetwork::findHealthIndex(uint8_t origin) const {
  if ((_trackedOrigins == nullptr) || (_trackedCount == 0U)) {
    return -1;
  }

  for (uint8_t i = 0U; i < _trackedCount; i++) {
    if (_trackedOrigins[i] == origin) {
      return static_cast<int16_t>(i);
    }
  }

  return -1;
}

void AimNetwork::updateHealthOnHeartbeat(uint8_t origin, uint32_t nowMs) {
  if ((_trackedOrigins == nullptr) || (_trackedCount == 0U) || (_healthTable == nullptr) || (_heartbeatTimeoutMs == 0U)) {
    return;
  }

  if (origin > AIM_ORG_ADDR_MAX) {
    return;
  }

  if (origin == _origin) {
    return;
  }

  const int16_t index = findHealthIndex(origin);
  if (index < 0) {
    return;
  }

  AimNodeHealth& health = _healthTable[index];
  health.lastHeartbeatMs = nowMs;
  health.alive = true;
}

void AimNetwork::evaluateHealth(uint32_t nowMs) {
  if ((_trackedOrigins == nullptr) || (_trackedCount == 0U) || (_healthTable == nullptr) || (_heartbeatTimeoutMs == 0U)) {
    return;
  }

  for (uint8_t i = 0U; i < _trackedCount; i++) {
    AimNodeHealth& health = _healthTable[i];
    if (health.origin == _origin) {
      health.lastHeartbeatMs = nowMs;
      health.alive = true;
      continue;
    }

    if (health.lastHeartbeatMs == 0U) {
      health.alive = false;
      continue;
    }

    health.alive = ((nowMs - health.lastHeartbeatMs) < _heartbeatTimeoutMs);
  }
}

const AimNodeHealth* AimNetwork::getHealthForOrigin(uint8_t origin) const {
  if ((_trackedOrigins == nullptr) || (_trackedCount == 0U) || (_healthTable == nullptr) || (_heartbeatTimeoutMs == 0U) || (origin > AIM_ORG_ADDR_MAX)) {
    return nullptr;
  }

  const int16_t index = findHealthIndex(origin);
  if (index < 0) {
    return nullptr;
  }

  return &_healthTable[index];
}

uint32_t AimNetwork::syncedMillis() const {
  return millis() + _timeOffset;
}

int32_t AimNetwork::getTimeOffset() const {
  return _timeOffset;
}
