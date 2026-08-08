// aim_network.h — AIM Network v0.7.0 protocol layer.
// 29-bit flat extended CAN ID: class[4] | subject[8] | source[4] | reserved[13]=0.
// Standard Payload: bytes 0-1 = uint16 relative offsetMs, bytes 2-7 per class.

#ifndef AIM_NETWORK_H
#define AIM_NETWORK_H

#include <Arduino.h>
#include <cstdint>
#include <cstring>

#include "aim_catalog.h"
#include "aim_can_frame.h"
#include "aim_safety.h"

// Conditional compilation mapping for the core hardware drivers.
#if defined(ARDUINO_ARCH_STM32)
  #include "aim_stm32_can_core.h"
  using AimCanHardware = AimStm32CanCore;
#elif defined(ARDUINO_ARCH_ESP32)
  #include "aim_esp32_can_core.h"
  using AimCanHardware = AimEsp32CanCore;
#else
  class AimCanHardware;
#endif

namespace aim {

static constexpr char kNetworkVersionString[] = "0.7.1";
static constexpr uint32_t kHeartbeatTxIntervalMs = 1000U;

// --- CAN ID layout ---

static constexpr uint8_t  kIdClassShift   = 25U;
static constexpr uint8_t  kIdSubjectShift = 17U;
static constexpr uint8_t  kIdSourceShift  = 13U;
static constexpr uint32_t kExtIdMask      = 0x1FFFFFFFU;

struct Msg {
  Class cls = Class::Debug;
  uint8_t subject = 0U;
  Source source = static_cast<Source>(0U);  // set by AimNetwork::send()
  uint32_t timestampMs = 0U;                // Memory-only timestamp (reconstructed on RX, syncedMillis on TX)
  uint16_t offsetMs = 0U;                   // 16-bit relative millisecond offset (carried on wire)
  uint8_t b[8] = {};                        // bytes 2–7 (or 0-7 if zero-timestamp), layout per class:
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

  void setGpsPosition(int32_t &lon, int32_t &lat) {
    (void)memcpy(b, &lon, sizeof(lon));
    (void)memcpy(b + sizeof(lon), &lat, sizeof(lat));
  }

  void getGpsPosition(int32_t &lon, int32_t &lat) const {
    (void)memcpy(&lon, b, sizeof(lon));
    (void)memcpy(&lat, b + sizeof(lon), sizeof(lat));
  }
};

static inline uint32_t encodeId(Class cls, uint8_t subj, Source src) {
  return (static_cast<uint32_t>(static_cast<uint8_t>(cls) & 0xFU) << kIdClassShift) |
         (static_cast<uint32_t>(subj) << kIdSubjectShift) |
         (static_cast<uint32_t>(static_cast<uint8_t>(src) & 0xFU) << kIdSourceShift);
}

// Reserved bits (12:0) are masked off and ignored per protocol invariant.
// Returns false only on source == 0 (wiring-fault canary).
static inline bool decodeId(uint32_t id, Msg& m) {
  const uint8_t src = static_cast<uint8_t>((id >> kIdSourceShift) & 0xFU);
  if (src == 0U) {
    return false;
  }

  m.cls     = static_cast<Class>((id >> kIdClassShift) & 0xFU);
  m.subject = static_cast<uint8_t>((id >> kIdSubjectShift) & 0xFFU);
  m.source  = static_cast<Source>(src);
  return true;
}

static inline bool packMsg(const Msg& msg, Frame& frame) {
  AIM_ASSERT(static_cast<uint8_t>(msg.source) != 0U);
  frame.id = encodeId(msg.cls, msg.subject, msg.source);
  frame.dlc = 8U;

  if (isZeroTimestamp(msg.cls, msg.subject)) {
    (void)memcpy(frame.data, msg.b, 8);
  } else {
    (void)memcpy(&frame.data[0], &msg.offsetMs, sizeof(msg.offsetMs));
    (void)memcpy(&frame.data[2], msg.b, 6);
  }
  return true;
}

static inline bool unpackMsg(const Frame& frame, Msg& msg) {
  AIM_ASSERT((frame.id & ~kExtIdMask) == 0U);
  if (frame.dlc != 8U) {
    return false;
  }
  if (!decodeId(frame.id, msg)) {
    return false;
  }
  if (isZeroTimestamp(msg.cls, msg.subject)) {
    (void)memcpy(msg.b, frame.data, 8);
    msg.offsetMs = 0;
  } else {
    (void)memcpy(&msg.offsetMs, &frame.data[0], sizeof(msg.offsetMs));
    (void)memcpy(msg.b, &frame.data[2], 6);
  }
  return true;
}

}  // namespace aim


class AimNetwork {
public:
  AimNetwork(AimCanHardware* hardware, aim::Source self);

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

  // Disciplines the local clock offset relative to remote master time (ms).
  void syncTime(uint32_t remoteMillis);

private:
  AimCanHardware* _hw;
  aim::Source _self;
  volatile int32_t _timeOffset;
  uint32_t _lastTxMs;
  uint32_t _lastSyncTimeMs;
};

#endif  // AIM_NETWORK_H
