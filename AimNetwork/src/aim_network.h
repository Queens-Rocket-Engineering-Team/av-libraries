#ifndef AIM_NETWORK_H
#define AIM_NETWORK_H

#include <cstdint>
#include <cstddef>

// AIM Packet Origins (0x0->0x7)
// don't know if we can use 0x0 because of filtering
#define AIM_ORG_ADDR_SIZE 3
#define AIM_ORG_NOORG 0x0
#define AIM_ORG_COMMS 0x1
#define AIM_ORG_UPROP 0x2
#define AIM_ORG_LPROP 0x3
#define AIM_ORG_ALT   0x4
#define AIM_ORG_GPS   0x5
#define AIM_ORG_PWR   0x6

// AIM Packet Destionations (0x0->0x7)
#define AIM_DEST_ADDR_SIZE AIM_ORG_ADDR_SIZE
#define AIM_DEST_COMMS  AIM_ORG_COMMS
#define AIM_DEST_UPROP  AIM_ORG_UPROP
#define AIM_DEST_LPROP  AIM_ORG_LPROP
#define AIM_DEST_ALT    AIM_ORG_ALT
#define AIM_DEST_BROADCAST 0x7

// AIM Packet Type (0x0->0xF)
#define AIM_TYP_ADDR_SIZE 4
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
#define AIM_TYPE_WATCHDOG 0xB
#define AIM_TYP_NODATA    0xE
#define AIM_TYP_UNDEFINED 0xF


// AIM Packet (Avionics Interconnected Module)
typedef struct aimPkt {
  uint8_t padding : 5;
  uint8_t origin : AIM_ORG_ADDR_SIZE;
  uint8_t dest : AIM_DEST_ADDR_SIZE;
  uint8_t type : AIM_TYP_ADDR_SIZE;
  uint64_t data;

  uint32_t getDayMillis() const {
    return (uint32_t)((data >> 24) & 0x7FFFFFF);
  }
  uint32_t getData() const {
    return (uint32_t)(data & 0xFFFFFF);
  }
  // setData takes the 27-bit millis and 24-bit payload and packs them into a 64-bit integer
  static uint64_t setDataPkt(uint32_t millis, uint32_t payload) {
    // enforce bit widths: 27-bit millis, 24-bit payload
    uint64_t m = (uint64_t)millis & 0x7FFFFFFull;
    uint64_t p = (uint64_t)payload & 0xFFFFFFull;
    return (m << 24) | p;
  }

} aimPkt;


class AimTransceiver {
public:
    virtual bool transmit(const uint8_t* buf, size_t len) = 0;
    virtual bool receive(uint8_t* buf, size_t len) = 0;
    virtual void begin() = 0;
};

class AimNetwork {
public:
    // Pass the hardware driver in the constructor
    AimNetwork(AimTransceiver* hardware, uint8_t origin) 
        : _hw(hardware), _origin(origin) {}

  void begin() { _hw->begin(); }

  bool sendPkt(uint64_t data, uint8_t dest, uint8_t type) {
    aimPkt pkt;
    pkt.origin = _origin;
    pkt.dest = dest;
    pkt.type = type;
    // pack the fields from the 27/24-bit values into the 64‑bit container
    pkt.data = data;
    // Send the raw bytes without knowing if it's CAN or LoRa
    return _hw->transmit((const uint8_t*)&pkt, sizeof(aimPkt));
  }

  bool readPkt(aimPkt &pkt) {
    aimPkt rev_pkt;
    if (_hw->receive((uint8_t*)&rev_pkt, sizeof(aimPkt))) {
      pkt = rev_pkt;
      return true;
    }
    else return false;
  }

private:
    AimTransceiver* _hw;
    uint8_t _origin;
};

#endif // AIM_NETWORK_H

