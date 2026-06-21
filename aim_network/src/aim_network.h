// aim_network.h — AIM Network v0.6.x protocol layer.
// 29-bit extended CAN ID: prio[2] | class[4] | subject[8] | source[4] | reserved[11]=0.
// Payload: bytes 0–3 = uint32 LE ms-since-start-of-day timestamp, bytes 4–7 per class.

#ifndef AIM_NETWORK_H
#define AIM_NETWORK_H

#include <Arduino.h>
#include <cstdint>
#include <cstring>

#include "aim_catalog.h"

namespace aim {

static constexpr char kNetworkVersionString[] = "0.6.0";
static constexpr uint32_t kHeartbeatTxIntervalMs = 1000U;

// --- CAN ID layout ---

static constexpr uint8_t  kIdPrioShift    = 27U;
static constexpr uint8_t  kIdClassShift   = 23U;
static constexpr uint8_t  kIdSubjectShift = 15U;
static constexpr uint8_t  kIdSourceShift  = 11U;
static constexpr uint32_t kExtIdMask      = 0x1FFFFFFFU;

struct Msg {
  Class cls = Class::Debug;
  uint8_t subject = 0U;
  Source source = static_cast<Source>(0U);  // set by AimNetwork::send()
  uint32_t timestampMs = 0U;                // stamped by send() from syncedMillis()
  uint8_t b[4] = {};                        // bytes 4–7, layout per class:
                                            // CMD  b[0]=seq b[1]=desired_state
                                            // ACK  b[0]=seq b[1]=result
                                            // STATE b[0]=commanded b[1]=energized b[2]=hall
                                            // SENSOR i32 value (use accessors)
                                            // HEARTBEAT b[0]=node_state b[1..2]=error_bits LE
                                            //           b[3]=schema_version
                                            // EVENT b[0]=detail

  int32_t sensorValue() const {
    int32_t v;
    (void)memcpy(&v, b, sizeof(v));
    return v;
  }

  void setSensorValue(int32_t v) {
    (void)memcpy(b, &v, sizeof(v));
  }
};

static inline uint32_t encodeId(uint8_t prio, Class cls, uint8_t subj, Source src) {
  return (static_cast<uint32_t>(prio & 0x3U) << kIdPrioShift) |
         (static_cast<uint32_t>(static_cast<uint8_t>(cls) & 0xFU) << kIdClassShift) |
         (static_cast<uint32_t>(subj) << kIdSubjectShift) |
         (static_cast<uint32_t>(static_cast<uint8_t>(src) & 0xFU) << kIdSourceShift);
}

// Reserved bits (10:0) are masked off and ignored per protocol invariant.
// Returns false only on source == 0 (wiring-fault canary).
static inline bool decodeId(uint32_t id, Msg& m, uint8_t& prio) {
  const uint8_t src = static_cast<uint8_t>((id >> kIdSourceShift) & 0xFU);
  if (src == 0U) {
    return false;
  }

  prio      = static_cast<uint8_t>((id >> kIdPrioShift) & 0x3U);
  m.cls     = static_cast<Class>((id >> kIdClassShift) & 0xFU);
  m.subject = static_cast<uint8_t>((id >> kIdSubjectShift) & 0xFFU);
  m.source  = static_cast<Source>(src);
  return true;
}

static constexpr uint16_t classBit(Class cls) {
  return static_cast<uint16_t>(1U << static_cast<uint8_t>(cls));
}

}  // namespace aim


class AimCanDriver;

class AimNetwork {
public:
  AimNetwork(AimCanDriver* hardware, aim::Source self);

  // Returns false on driver/filter init failure — apps must check.
  bool begin(uint16_t classAcceptMask);

  // Sets source, timestamp (syncedMillis), and fixed priority, then transmits.
  bool send(aim::Msg& m);

  // Returns the next accepted frame. TIME/TimeSync frames sync the local
  // clock offset first, then are returned like any other frame.
  bool receive(aim::Msg& m);

  // Call periodically. Sends a HEARTBEAT only if this node has been silent
  // for more than half the heartbeat interval (any TX proves liveness).
  void service(uint32_t nowMs, aim::NodeState state, uint16_t errorBits);

  // Local time adjusted by the last TIME sync offset. Steps on resync and
  // backward at midnight — never use it for scheduling or liveness timing.
  uint32_t syncedMillis() const;

private:
  void syncTime(uint32_t remoteMillis);

  AimCanDriver* _hw;
  aim::Source _self;
  int32_t _timeOffset;
  uint32_t _lastTxMs;
};

#endif  // AIM_NETWORK_H
