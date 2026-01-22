#ifndef AIM_NETWORK_H
#define AIM_NETWORK_H
#include "STM32_CAN.h" // Version 1.1.2

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
};

class AimNetwork {
public:
  AimNetwork(uint8_t origin, uint32_t baud);
  void begin();

  bool sendPkt(dataPkt aim_data, uint8_t aim_dest, uint8_t aim_type);
  bool readPkt(dataPkt &aim_data, uint8_t &aim_origin, uint8_t &aim_type);

private:
  static STM32_CAN _canb;
  uint8_t _origin;
  uint32_t _baud;

  // AIM Packet (Avionics Interconnected Module)
  typedef struct aimPkt {
    uint8_t padding : 5;
    uint8_t origin : AIM_ORG_ADDR_SIZE;
    uint8_t dest : AIM_DEST_ADDR_SIZE;
    uint8_t type : AIM_TYP_ADDR_SIZE;
    dataPkt data = {0, 0};
  };

  bool packAimPkt(aimPkt aim_pkt, CAN_message_t &can_msg);
  bool unpackAimPkt(CAN_message_t can_msg, aimPkt &aim_pkt);
};

#endif // AIM_NETWORK_H

