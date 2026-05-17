#ifndef AIM_NETWORK_H
#define AIM_NETWORK_H

#include <Arduino.h>
#include <cstdint>
#include <cstddef>


// ── Module Addresses (3-bit, 0x0->0x7) ──────────────────────

#define AIM_ORG_NOORG 0x0
#define AIM_ORG_COMMS 0x1
#define AIM_ORG_UPROP 0x2
#define AIM_ORG_LPROP 0x3
#define AIM_ORG_ALT   0x4
#define AIM_ORG_GPS   0x5
#define AIM_ORG_PWR   0x6

#define AIM_DEST_COMMS     AIM_ORG_COMMS
#define AIM_DEST_UPROP     AIM_ORG_UPROP
#define AIM_DEST_LPROP     AIM_ORG_LPROP
#define AIM_DEST_ALT       AIM_ORG_ALT
#define AIM_DEST_BROADCAST 0x7


// ── Packet Types (4-bit, 0x0->0xF) ──────────────────────────

#define AIM_TYP_TIME      0x0
#define AIM_TYP_SENSOR    0x1
#define AIM_TYP_VALVE     0x2
#define AIM_TYP_GPS_LAT   0x3
#define AIM_TYP_GPS_LONG  0x4
#define AIM_TYP_ALT       0x5
#define AIM_TYP_LOWPW     0x6
#define AIM_TYP_HEARTBEAT 0x7
#define AIM_TYP_NODATA    0x8
#define AIM_TYP_UNDEFINED 0xF


// ── Bit-width constants (used by driver for packing) ────────

#define AIM_ORG_ADDR_SIZE  3
#define AIM_DEST_ADDR_SIZE 3
#define AIM_TYP_ADDR_SIZE  4

#define AIM_ORG_ADDR_MAX   ((1U << AIM_ORG_ADDR_SIZE) - 1U)
#define AIM_DEST_ADDR_MAX  ((1U << AIM_DEST_ADDR_SIZE) - 1U)
#define AIM_TYP_ADDR_MAX   ((1U << AIM_TYP_ADDR_SIZE) - 1U)


// ── Timed data layout constants (for downstream validation) ──

#define AIM_PKT_TIMED_ENDPOINT_BITS  5
#define AIM_PKT_TIMED_MILLIS_BITS    27
#define AIM_PKT_TIMED_PAYLOAD_BITS   32
#define AIM_PKT_RAW_PAYLOAD_BITS     64

#define AIM_PKT_TIMED_ENDPOINT_SHIFT (AIM_PKT_TIMED_MILLIS_BITS + AIM_PKT_TIMED_PAYLOAD_BITS)

#define AIM_PKT_TIMED_ENDPOINT_MAX   ((1U << AIM_PKT_TIMED_ENDPOINT_BITS) - 1U)
#define AIM_PKT_TIMED_MILLIS_MAX     ((1UL << AIM_PKT_TIMED_MILLIS_BITS) - 1UL)
#define AIM_PKT_TIMED_PAYLOAD_MAX    ((1ULL << AIM_PKT_TIMED_PAYLOAD_BITS) - 1ULL)

#define AIM_HEARTBEAT_TX_INTERVAL_DEFAULT_MS 5000U

#define AIM_NETWORK_VERSION_STRING "0.4.0"


// ── AIM Packet ───────────────────────────────────────────────
// Data field (timed format): upper 5 bits = endpoint ID, next 27 bits = timestamp (ms), lower 32 bits = payload
// Data field (raw format): entire 64 bits are payload

typedef struct aimPkt {
  uint8_t  padding : 5;
  uint8_t  origin  : AIM_ORG_ADDR_SIZE;
  uint8_t  dest    : AIM_DEST_ADDR_SIZE;
  uint8_t  type    : AIM_TYP_ADDR_SIZE;
  uint64_t data;

  uint8_t getEndpointId() const {
    return static_cast<uint8_t>((data >> AIM_PKT_TIMED_ENDPOINT_SHIFT) & (uint64_t)AIM_PKT_TIMED_ENDPOINT_MAX);
  }
  uint32_t getMillis()  const { return (uint32_t)((data >> AIM_PKT_TIMED_PAYLOAD_BITS) & (uint64_t)AIM_PKT_TIMED_MILLIS_MAX); }
  uint32_t getPayload() const { return (uint32_t)(data & (uint64_t)AIM_PKT_TIMED_PAYLOAD_MAX); }
  uint64_t getPayload64() const { return data; }

  static uint64_t packDataEx(uint8_t endpointId, uint32_t ms, uint32_t payload) {
    return (((uint64_t)endpointId & (uint64_t)AIM_PKT_TIMED_ENDPOINT_MAX) << AIM_PKT_TIMED_ENDPOINT_SHIFT) |
           (((uint64_t)ms & (uint64_t)AIM_PKT_TIMED_MILLIS_MAX) << AIM_PKT_TIMED_PAYLOAD_BITS) |
           ((uint64_t)payload & (uint64_t)AIM_PKT_TIMED_PAYLOAD_MAX);
  }

  static uint64_t packData(uint32_t ms, uint32_t payload) {
    return packDataEx(0U, ms, payload);
  }

} aimPkt;


// ── Debug utility ────────────────────────────────────────────
// Prints packet fields to any Stream (Serial, SoftwareSerial, etc.)

inline void aimPrintPkt(Stream& out, const aimPkt& pkt, const char* label = "") {
  if (label[0]) { out.print(label); out.print(" "); }
  out.print("org=0x"); out.print(pkt.origin, HEX);
  out.print(" dst=0x"); out.print(pkt.dest, HEX);
  out.print(" typ=0x"); out.print(pkt.type, HEX);
  out.print(" ep=0x");  out.print(pkt.getEndpointId(), HEX);
  out.print(" pay=0x"); out.print(pkt.getPayload(), HEX);
  out.print(" t=");     out.print(pkt.getMillis());
  out.println("ms");
}

class AimCanDriver;

struct AimNodeHealth {
  uint8_t origin;
  uint32_t lastHeartbeatMs;
  bool alive;
};


// ── AimNetwork ───────────────────────────────────────────────

class AimNetwork {
public:
  AimNetwork(AimCanDriver* hardware, uint8_t origin);

  void begin();

  // Send a pre-built packet
  bool sendPkt(const aimPkt& pkt);

  // Send with raw 64-bit data field
  bool sendPkt64(uint64_t data, uint8_t dest, uint8_t type);

  // Send with millis + payload (packs data for you)
  bool sendPkt32(uint32_t ms, uint32_t payload, uint8_t dest, uint8_t type);
  bool sendPkt32Ex(uint32_t endpointId, uint32_t ms, uint32_t payload, uint8_t dest, uint8_t type);

  bool readPkt(aimPkt& pkt);

  // Time sync — call when receiving a TIME packet
  void syncTime(uint32_t remoteMillis);

  // Optional node-health monitor.
  // Caller owns trackedOrigins/healthStorage memory and lifetime.
  // Initializes monitor state for tracked nodes at nowMs.
  bool configureHealthMonitor(const uint8_t* trackedOrigins,
                              uint8_t trackedCount,
                              AimNodeHealth* healthStorage,
                              uint32_t heartbeatTimeoutMs,
                              uint32_t nowMs);

  void updateHealthOnHeartbeat(uint8_t origin, uint32_t nowMs);
  void evaluateHealth(uint32_t nowMs);
  const AimNodeHealth* getHealthForOrigin(uint8_t origin) const;

  // Returns local time adjusted by the last sync offset
  uint32_t syncedMillis() const;

  int32_t getTimeOffset() const;

private:
  int16_t findHealthIndex(uint8_t origin) const;

  AimCanDriver* _hw;
  uint8_t _origin;
  int32_t _timeOffset;

  const uint8_t* _trackedOrigins;
  uint8_t _trackedCount;
  AimNodeHealth* _healthTable;
  uint32_t _heartbeatTimeoutMs;
};

#endif // AIM_NETWORK_H
