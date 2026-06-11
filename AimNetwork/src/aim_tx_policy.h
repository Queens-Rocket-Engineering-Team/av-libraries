#pragma once
#include "aim_network.h"
#include <cstdint>

struct AimTxPeriodic {
    uint8_t  endpoint;
    uint8_t  dest;
    uint8_t  type;
    uint32_t periodMs;
    uint32_t lastSentMs;
    uint32_t payload;
    bool     active;
};

struct AimTxLatched {
    uint8_t  endpoint;
    uint8_t  dest;
    uint8_t  type;
    uint32_t refreshMs;   // 0 = dirty-only, no periodic refresh
    uint32_t lastSentMs;
    uint32_t payload;
    bool     dirty;
    bool     active;
};

class AimTxPolicy {
public:
    static constexpr uint8_t kMaxPeriodic = 8U;
    static constexpr uint8_t kMaxLatched  = 4U;
    static constexpr uint8_t kInvalidSlot = 0xFFU;

    // Call at init — returns slot index or kInvalidSlot if full
    uint8_t addPeriodic(uint8_t endpoint, uint8_t dest, uint8_t type, uint32_t periodMs);
    uint8_t addLatched (uint8_t endpoint, uint8_t dest, uint8_t type, uint32_t refreshMs);

    // Update latest value. Both return false for an out-of-range slot.
    // Periodic: payload is always replaced; latest value wins at next period.
    // Latched:  dirty is set iff the value differs from the stored payload.
    bool setPeriodic(uint8_t slot, uint32_t payload);
    bool setLatched (uint8_t slot, uint32_t payload);

    // Drive all eligible sends — call once per loop tick. Bounded:
    // at most (kMaxPeriodic + kMaxLatched) sendPkt attempts per call.
    //
    // Failure semantics (deliberate — do not "fix"):
    //   Periodic: lastSentMs is updated unconditionally. A failed send drops
    //   that sample; the next one goes out a full period later. Periodic data
    //   is fresh-by-definition, so retrying stale samples has no value.
    //   Latched:  on failure the dirty flag is left set, so the send is
    //   retried on the very next tick until it succeeds.
    void service(AimNetwork& aim, uint32_t schedulerMs, uint32_t networkMs);

private:
    AimTxPeriodic _periodic[kMaxPeriodic] = {};
    AimTxLatched  _latched [kMaxLatched]  = {};
    uint8_t _periodicCount = 0U;
    uint8_t _latchedCount  = 0U;
};
