#include "aim_tx_policy.h"
#include "aim_safety.h"

uint8_t AimTxPolicy::addPeriodic(uint8_t endpoint, uint8_t dest, uint8_t type, uint32_t periodMs) {
    if (_periodicCount >= kMaxPeriodic) {
        return kInvalidSlot;
    }
    const uint8_t slot = _periodicCount++;
    _periodic[slot].endpoint   = endpoint;
    _periodic[slot].dest       = dest;
    _periodic[slot].type       = type;
    _periodic[slot].periodMs   = periodMs;
    _periodic[slot].lastSentMs = 0U;
    _periodic[slot].payload    = 0U;
    _periodic[slot].active     = true;
    return slot;
}

uint8_t AimTxPolicy::addLatched(uint8_t endpoint, uint8_t dest, uint8_t type, uint32_t refreshMs) {
    if (_latchedCount >= kMaxLatched) {
        return kInvalidSlot;
    }
    const uint8_t slot = _latchedCount++;
    _latched[slot].endpoint   = endpoint;
    _latched[slot].dest       = dest;
    _latched[slot].type       = type;
    _latched[slot].refreshMs  = refreshMs;
    _latched[slot].lastSentMs = 0U;
    _latched[slot].payload    = 0U;
    _latched[slot].dirty      = false;
    _latched[slot].active     = true;
    return slot;
}

bool AimTxPolicy::setPeriodic(uint8_t slot, uint32_t payload) {
    if (slot >= _periodicCount) {
        return false;
    }
    _periodic[slot].payload = payload;
    return true;
}

bool AimTxPolicy::setLatched(uint8_t slot, uint32_t payload) {
    if (slot >= _latchedCount) {
        return false;
    }
    if (_latched[slot].payload != payload) {
        _latched[slot].payload = payload;
        _latched[slot].dirty   = true;
    }
    return true;
}

void AimTxPolicy::service(AimNetwork& aim, uint32_t schedulerMs, uint32_t networkMs) {
    for (uint8_t i = 0U; i < _periodicCount; i++) {
        AimTxPeriodic& s = _periodic[i];
        if (!s.active) {
            continue;
        }
        if ((schedulerMs - s.lastSentMs) >= s.periodMs) {
            s.lastSentMs = schedulerMs;  // update unconditionally — drop on fail
            aimPkt pkt = {};
            pkt.dest = s.dest;
            pkt.type = s.type;
            if (pkt.packData(s.endpoint, networkMs, s.payload)) {
                (void)aim.sendPkt(pkt);
            }
        }
    }

    for (uint8_t i = 0U; i < _latchedCount; i++) {
        AimTxLatched& s = _latched[i];
        if (!s.active) {
            continue;
        }
        const bool refreshDue = (s.refreshMs > 0U) &&
                                ((schedulerMs - s.lastSentMs) >= s.refreshMs);
        if (s.dirty || refreshDue) {
            aimPkt pkt = {};
            pkt.dest = s.dest;
            pkt.type = s.type;
            if (pkt.packData(s.endpoint, networkMs, s.payload)) {
                if (aim.sendPkt(pkt)) {
                    s.dirty      = false;
                    s.lastSentMs = schedulerMs;
                }
                // on sendPkt failure: dirty stays set, retried next tick
            }
        }
    }
}
