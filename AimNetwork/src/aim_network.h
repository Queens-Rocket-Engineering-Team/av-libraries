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
#define AIM_TYP_PT1       0x1
#define AIM_TYP_PT2       0x2
#define AIM_TYP_TC        0x4
#define AIM_TYP_VAL1      0x5
#define AIM_TYP_VAL2      0x6
#define AIM_TYP_GPS_LAT   0x7
#define AIM_TYP_GPS_LONG  0x8
#define AIM_TYP_ALT       0x9
#define AIM_TYP_LOWPW     0xA
#define AIM_TYP_WATCHDOG  0xB
#define AIM_TYP_NODATA    0xE
#define AIM_TYP_UNDEFINED 0xF


// ── Bit-width constants (used by driver for packing) ────────

#define AIM_ORG_ADDR_SIZE  3
#define AIM_DEST_ADDR_SIZE 3
#define AIM_TYP_ADDR_SIZE  4


// ── AIM Packet ───────────────────────────────────────────────
// Data field: upper 27 bits = timestamp (ms), lower 24 bits = payload

typedef struct aimPkt {
  uint8_t  padding : 5;
  uint8_t  origin  : AIM_ORG_ADDR_SIZE;
  uint8_t  dest    : AIM_DEST_ADDR_SIZE;
  uint8_t  type    : AIM_TYP_ADDR_SIZE;
  uint64_t data;

  uint32_t getMillis()  const { return (uint32_t)((data >> 24) & 0x7FFFFFF); }
  uint32_t getPayload() const { return (uint32_t)(data & 0xFFFFFF); }

  static uint64_t packData(uint32_t ms, uint32_t payload) {
    return (((uint64_t)ms & 0x7FFFFFFull) << 24) | ((uint64_t)payload & 0xFFFFFFull);
  }
} aimPkt;


// ── Debug utility ────────────────────────────────────────────
// Prints packet fields to any Stream (Serial, SoftwareSerial, etc.)

inline void aimPrintPkt(Stream& out, const aimPkt& pkt, const char* label = "") {
  if (label[0]) { out.print(label); out.print(" "); }
  out.print("org=0x"); out.print(pkt.origin, HEX);
  out.print(" dst=0x"); out.print(pkt.dest, HEX);
  out.print(" typ=0x"); out.print(pkt.type, HEX);
  out.print(" pay=0x"); out.print(pkt.getPayload(), HEX);
  out.print(" t=");     out.print(pkt.getMillis());
  out.println("ms");
}


// ── Hardware interface ───────────────────────────────────────

class AimTransceiver {
public:
  virtual bool transmit(const uint8_t* buf, size_t len) = 0;
  virtual bool receive(uint8_t* buf, size_t len) = 0;
  virtual void begin() = 0;
};


// ── AimNetwork ───────────────────────────────────────────────

class AimNetwork {
public:
  AimNetwork(AimTransceiver* hardware, uint8_t origin)
      : _hw(hardware), _origin(origin) {}

  void begin() { _hw->begin(); }

  // Send a pre-built packet (you can inspect it before sending)
  bool sendPkt(const aimPkt& pkt) {
    return _hw->transmit((const uint8_t*)&pkt, sizeof(aimPkt));
  }

  // Send with raw data field
  bool sendPkt(uint64_t data, uint8_t dest, uint8_t type) {
    aimPkt pkt;
    pkt.origin = _origin;
    pkt.dest   = dest;
    pkt.type   = type;
    pkt.data   = data;
    return sendPkt(pkt);
  }

  // Send with millis + payload (packs data for you)
  bool sendPkt(uint32_t ms, uint32_t payload, uint8_t dest, uint8_t type) {
    return sendPkt(aimPkt::packData(ms, payload), dest, type);
  }

  bool readPkt(aimPkt& pkt) {
    return _hw->receive((uint8_t*)&pkt, sizeof(aimPkt));
  }

private:
  AimTransceiver* _hw;
  uint8_t _origin;
};

#endif // AIM_NETWORK_H
