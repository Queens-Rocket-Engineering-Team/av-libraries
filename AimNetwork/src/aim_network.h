#ifndef AIM_NETWORK_H
#define AIM_NETWORK_H

#include <Arduino.h>
#include <cstdint>
#include <cstddef>

namespace aim {

// --- Packet field layout ---

static constexpr uint8_t kNodeBits = 3U;
static constexpr uint8_t kTypeBits = 4U;

static constexpr uint8_t kNodeMax =
    static_cast<uint8_t>((1U << kNodeBits) - 1U);

static constexpr uint8_t kTypeMax =
    static_cast<uint8_t>((1U << kTypeBits) - 1U);


// --- Timed packet data layout ---

static constexpr uint8_t kEndpointBits = 5U;
static constexpr uint8_t kMillisBits   = 27U;
static constexpr uint8_t kPayloadBits  = 32U;

static constexpr uint8_t kEndpointShift =
    static_cast<uint8_t>(kMillisBits + kPayloadBits);

static constexpr uint8_t kEndpointMax =
    static_cast<uint8_t>((1U << kEndpointBits) - 1U);

static constexpr uint32_t kMillisMax =
    static_cast<uint32_t>((1UL << kMillisBits) - 1UL);

static constexpr uint64_t kPayloadMask =
    (1ULL << kPayloadBits) - 1ULL;


// --- Defaults ---

static constexpr uint32_t kHeartbeatTxIntervalDefaultMs = 5000U;
static constexpr char kNetworkVersionString[] = "0.4.2";

using EndpointId = uint8_t;


enum class Node : uint8_t {
  NoOrg     = 0x0,
  Comms     = 0x1,
  UProp     = 0x2,
  LProp     = 0x3,
  Alt       = 0x4,
  Gps       = 0x5,
  Power     = 0x6,
  Broadcast = 0x7,
};

enum class PacketType : uint8_t {
  Time      = 0x0,
  Sensor    = 0x1,
  Valve     = 0x2,
  GpsLat    = 0x3,
  GpsLong   = 0x4,
  Altitude  = 0x5,
  LowPower  = 0x6,
  Heartbeat = 0x7,
  NoData    = 0x8,
  Undefined = 0xF,
};


// --- AIM Packet ---
// Timed data layout:
// upper 5 bits  = endpoint ID
// next 27 bits  = timestamp ms
// lower 32 bits = payload
struct Pkt {
  Node origin = Node::NoOrg;
  Node dest   = Node::NoOrg;
  PacketType type = PacketType::NoData;
  uint64_t data = 0U;

  bool packData(EndpointId endpointId, uint32_t ms, uint32_t payload) {
    if (endpointId > kEndpointMax) {
      return false;
    }

    if (ms > kMillisMax) {
      return false;
    }

    data =
        (static_cast<uint64_t>(endpointId) << kEndpointShift) |
        (static_cast<uint64_t>(ms) << kPayloadBits) |
        static_cast<uint64_t>(payload);

    return true;
  }

  bool validate() const {
    return static_cast<uint8_t>(origin) <= kNodeMax &&
           static_cast<uint8_t>(dest)   <= kNodeMax &&
           static_cast<uint8_t>(type)   <= kTypeMax;
  }

  EndpointId getEndpointId() const {
    return static_cast<EndpointId>(
        (data >> kEndpointShift) & kEndpointMax);
  }

  uint32_t getMillis() const {
    return static_cast<uint32_t>(
        (data >> kPayloadBits) & kMillisMax);
  }

  uint32_t getPayload() const {
    return static_cast<uint32_t>(data & kPayloadMask);
  }
};

}  // namespace aim


class AimCanDriver;

class AimNetwork {
public:
  AimNetwork(AimCanDriver* hardware, aim::Node origin);

  void begin();

  bool sendPkt(aim::Pkt& pkt);
  bool readPkt(aim::Pkt& pkt);

  // Time sync — call when receiving a TIME packet
  void syncTime(uint32_t remoteMillis);

  // Returns local time adjusted by the last sync offset
  uint32_t syncedMillis() const;

private:

  AimCanDriver* _hw;
  aim::Node _origin;
  int32_t _timeOffset;
};

#endif // AIM_NETWORK_H
