#include "aim_can_driver.h"

// CAN1 ALT is PB8+PB9
STM32_CAN AimCanDriver::_canb(CAN1, ALT, RX_SIZE_16, TX_SIZE_16);

AimCanDriver::AimCanDriver(uint8_t origin, uint32_t baud) {
  _origin = origin;
  _baud = baud;
}
void AimCanDriver::begin() {
  _canb.begin();
  _canb.setBaudRate(_baud);
  // Filter for incoming dest==_origin
  const uint16_t id_dest   = (uint16_t)((_origin & 0x07) << 5);
  const uint16_t mask_dest = 0x0E0;
  _canb.setMBFilterProcessing(MB0, id_dest, mask_dest);

  // Broadcast filter
  const uint16_t id_broadcast  = (uint16_t)(AIM_DEST_BROADCAST << 5); 
  const uint16_t mask_broadcast = 0x0E0;
  _canb.setMBFilterProcessing(MB1, id_broadcast, mask_broadcast);
}

bool AimCanDriver::packAimPkt(const aimPkt &aim_pkt, CAN_message_t &can_msg) {
  // packing bits
  uint16_t id_packed =
      ((aim_pkt.origin     & 0x07) << 8)  | // 11 bits - 3 bits
      ((aim_pkt.dest       & 0x07) << 5)  | // 11 bits - 3 bits - 3 bits
      ((aim_pkt.type & 0x0F));  // bottom 4 bits

  // using standard 11 bit CAN header
  can_msg.id = id_packed & 0x07FF;
  can_msg.flags.extended = 0;
  can_msg.len = sizeof(aim_pkt.data);
  
  if (can_msg.len != 8) return false;

  if(!memcpy(can_msg.buf, &aim_pkt.data, sizeof(aim_pkt.data))){
    return false;
  }
  return true;
}

bool AimCanDriver::unpackAimPkt(CAN_message_t can_msg, aimPkt &aim_pkt) {
  // unpacking bits
  uint16_t id_packed = can_msg.id;
  aim_pkt.origin = (id_packed >> 8) & 0x07;
  aim_pkt.dest = (id_packed >> 5) & 0x07;
  aim_pkt.type = id_packed & 0x0F;

  if (sizeof(aim_pkt.data)!=can_msg.len) {
      return false;
  }

  if(!memcpy(&aim_pkt.data, can_msg.buf, can_msg.len)){
    return false;
  }
  return true;
}
bool AimCanDriver::transmit(const uint8_t* buf, size_t len) {
  // Sanity check: network should only send full AIM packets
  if (len != sizeof(aimPkt)) return false;

  // Reconstruct AIM packet from raw bytes
  aimPkt pkt;
  memcpy(&pkt, buf, sizeof(pkt));

  // Convert AIM packet → CAN frame
  CAN_message_t can_msg;
  if (!packAimPkt(pkt, can_msg)) return false;

  // Send on CAN bus
  return _canb.write(can_msg);
}
bool AimCanDriver::receive(uint8_t* buf, size_t len) {
  // Sanity check: network should only receive full AIM packets
  if (len != sizeof(aimPkt)) return false;

  CAN_message_t can_msg;
  if (!_canb.read(can_msg)) return false;

  // Convert CAN frame → AIM packet
  aimPkt pkt;
  if (!unpackAimPkt(can_msg, pkt)) return false;

  // Copy AIM packet to output buffer
  memcpy(buf, &pkt, sizeof(pkt));
  return true;
}

