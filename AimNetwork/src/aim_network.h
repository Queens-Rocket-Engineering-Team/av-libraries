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
#define AIM_TYP_NODATA    0xE
#define AIM_TYP_UNDEFINED 0xF

typedef struct dataPkt {
	uint32_t dayMilis; // time of day in miliseconds
	uint32_t data;
}dataPkt;

// AIM Packet (Avionics Interconnected Module)
typedef struct aimPkt {
  uint8_t padding : 5;
  uint8_t origin : AIM_ORG_ADDR_SIZE;
  uint8_t dest : AIM_DEST_ADDR_SIZE;
  uint8_t type : AIM_TYP_ADDR_SIZE;
  dataPkt data = {0, 0};
}aimPkt;

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

  bool sendPkt(dataPkt data, uint8_t dest, uint8_t type) {
    aimPkt pkt;
    pkt.origin = _origin;
    pkt.dest = dest;
    pkt.type = type;
    pkt.data = data;
    // Send the raw bytes without knowing if it's CAN or LoRa
    return _hw->transmit((const uint8_t*)&pkt, sizeof(aimPkt));
  }

  bool readPkt(dataPkt &aim_data, uint8_t &aim_origin, uint8_t &aim_type) {
    aimPkt pkt;
    if (_hw->receive((uint8_t*)&pkt, sizeof(aimPkt))) {
      aim_data = pkt.data;
      aim_origin = pkt.origin;
      aim_type = pkt.type;
      return true;
    }
    else return false;
  }

private:
    AimTransceiver* _hw;
    uint8_t _origin;
};

#endif // AIM_NETWORK_H

