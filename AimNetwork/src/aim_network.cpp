
#include "aim_network.h"

#include "aim_can_driver.h"
#include <logger.h>

AimNetwork::AimNetwork(AimCanDriver* hardware, aim::Node origin)
    : _hw(hardware),
      _origin(origin),
      _timeOffset(0) {}

void AimNetwork::begin() {
  if (_hw != nullptr) {
    _hw->begin(_origin);
  }
}

bool AimNetwork::sendPkt(aim::Pkt& pkt) {
  if (_hw == nullptr) {
    LOG_ERROR("AimNetwork sendPkt failed: hardware driver is null");
    return false;
  }

  pkt.origin = _origin;

  if (!pkt.validate()) {
    LOG_ERROR("AimNetwork sendPkt failed: invalid packet");
    return false;
  }

  const bool sent = _hw->transmit(pkt);
  if (!sent) {
    LOG_ERROR("AimNetwork sendPkt failed: CAN transmit returned false");
  }

  return sent;
}

bool AimNetwork::readPkt(aim::Pkt& pkt) {
  if (_hw == nullptr) {
    LOG_ERROR("AimNetwork readPkt failed: hardware driver is null");
    return false;
  }

  if (!_hw->receive(pkt)) {
    return false;
  }

  if (!pkt.validate()) {
    LOG_ERROR("AimNetwork readPkt failed: invalid packet");
    return false;
  }

  return true;
}

void AimNetwork::syncTime(uint32_t remoteMillis) {
  _timeOffset = static_cast<int32_t>(remoteMillis) - static_cast<int32_t>(millis());
}

uint32_t AimNetwork::syncedMillis() const {
  return millis() + _timeOffset;
}

