#ifndef AIM_CAN_DRIVER_H
#define AIM_CAN_DRIVER_H

#include "aim_network.h"
#include "STM32_CAN.h" // Version 1.1.2

class AimCanDriver : public AimTransceiver{
public:
  AimCanDriver(uint8_t origin, uint32_t baud);
  void begin();
  bool transmit(const uint8_t* buf, size_t len) override;
  bool receive(uint8_t* buf, size_t len) override;
  

private:
  static STM32_CAN _canb;
  uint8_t _origin;
  uint32_t _baud;

  
  bool packAimPkt(aimPkt aim_pkt, CAN_message_t &can_msg);
  bool unpackAimPkt(CAN_message_t can_msg, aimPkt &aim_pkt);
};

#endif // AIM_CAN_DRIVER_H