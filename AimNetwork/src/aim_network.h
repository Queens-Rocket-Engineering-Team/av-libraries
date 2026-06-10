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

#define AIM_TYPE_TIME      0x0
#define AIM_TYPE_SENSOR    0x1
#define AIM_TYPE_VALVE     0x2
#define AIM_TYPE_GPS_LAT   0x3
#define AIM_TYPE_GPS_LONG  0x4
#define AIM_TYPE_ALT       0x5
#define AIM_TYPE_LOWPW     0x6
#define AIM_TYPE_HEARTBEAT 0x7
#define AIM_TYPE_NODATA    0x8
#define AIM_TYPE_UNDEFINED 0xF


// ── Bit-width constants (used by driver for packing) ────────

#define AIM_ORG_ADDR_SIZE  3
#define AIM_DEST_ADDR_SIZE 3
#define AIM_TYPE_ADDR_SIZE  4

#define AIM_ORG_ADDR_MAX   ((1U << AIM_ORG_ADDR_SIZE) - 1U)
#define AIM_DEST_ADDR_MAX  ((1U << AIM_DEST_ADDR_SIZE) - 1U)
#define AIM_TYPE_ADDR_MAX   ((1U << AIM_TYPE_ADDR_SIZE) - 1U)


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

#define AIM_NETWORK_VERSION_STRING "0.4.2"


// ── AIM Packet ───────────────────────────────────────────────
// Data field (timed format): upper 5 bits = endpoint ID, next 27 bits = timestamp (ms), lower 32 bits = payload
// Data field (raw format): entire 64 bits are payload

typedef struct aimPkt {
  uint8_t  padding : 5;
  uint8_t  origin  : AIM_ORG_ADDR_SIZE;
  uint8_t  dest    : AIM_DEST_ADDR_SIZE;
  uint8_t  type    : AIM_TYPE_ADDR_SIZE;
  uint64_t data;

  bool packData(uint8_t endpointId, uint32_t ms, uint32_t payload) {
    if (endpointId > AIM_PKT_TIMED_ENDPOINT_MAX) {
      return false;
    }

    if (ms > AIM_PKT_TIMED_MILLIS_MAX) {
      return false;
    }

    data =
      (((uint64_t)endpointId) << AIM_PKT_TIMED_ENDPOINT_SHIFT) |
      (((uint64_t)ms) << AIM_PKT_TIMED_PAYLOAD_BITS) |
      ((uint64_t)payload);

    return true;
  }

  bool validate() const {
    return origin <= AIM_ORG_ADDR_MAX &&
           dest   <= AIM_DEST_ADDR_MAX &&
           type   <= AIM_TYPE_ADDR_MAX;
  }

  uint8_t getEndpointId() const {
    return (uint8_t)((data >> AIM_PKT_TIMED_ENDPOINT_SHIFT) & AIM_PKT_TIMED_ENDPOINT_MAX);
  }

  uint32_t getMillis() const {
    return (uint32_t)((data >> AIM_PKT_TIMED_PAYLOAD_BITS) & AIM_PKT_TIMED_MILLIS_MAX);
  }

  uint32_t getPayload() const {
    return (uint32_t)(data & AIM_PKT_TIMED_PAYLOAD_MAX);
  }

} aimPkt;

class AimCanDriver;

class AimNetwork {
public:
  AimNetwork(AimCanDriver* hardware, uint8_t origin);

  void begin();

  bool sendPkt(aimPkt& pkt);
  bool readPkt(aimPkt& pkt);

  // Time sync — call when receiving a TIME packet
  void syncTime(uint32_t remoteMillis);

  // Returns local time adjusted by the last sync offset
  uint32_t syncedMillis() const;

private:

  AimCanDriver* _hw;
  uint8_t _origin;
  int32_t _timeOffset;

  const uint8_t* _trackedOrigins;
  uint8_t _trackedCount;
};

#endif // AIM_NETWORK_H
