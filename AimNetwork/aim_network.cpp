#include "aim_network.h"

// CAN1 ALT is PB8+PB9
STM32_CAN AimNetwork::_canb(CAN1, ALT, RX_SIZE_16, TX_SIZE_16);

AimNetwork::AimNetwork(uint8_t origin, uint32_t baud) {
  _origin = origin;
  _baud = baud;
}

void AimNetwork::begin() {
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

bool AimNetwork::packAimPkt(aimPkt aim_pkt, CAN_message_t &can_msg) {
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

bool AimNetwork::unpackAimPkt(CAN_message_t can_msg, aimPkt &aim_pkt) {
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

bool AimNetwork::sendPkt(dataPkt aim_data, uint8_t aim_dest, uint8_t aim_type){
  bool ret = true;
  aimPkt aim_pkt;
  aim_pkt.origin = _origin;
  aim_pkt.dest = aim_dest;
  aim_pkt.type = aim_type;
  aim_pkt.data = aim_data;

  CAN_message_t can_msg;
  ret = packAimPkt(aim_pkt, can_msg);
 
  ret &= _canb.write(can_msg);

  return ret;
}

bool AimNetwork::readPkt(dataPkt &aim_data, uint8_t &aim_origin, uint8_t &aim_type) {
  bool ret = false;
  aim_origin = AIM_ORG_NOORG;
  aim_type = AIM_TYP_NODATA;

  CAN_message_t can_msg;
  aimPkt aim_pkt;

  if(_canb.read(can_msg)) {
    ret = true;
    // not catastropic if unpack fails on one packet
    if(unpackAimPkt(can_msg, aim_pkt)) {
      aim_data = aim_pkt.data;
      aim_origin = aim_pkt.origin;
      aim_type = aim_pkt.type;
    }
  }
 
  return ret;
}
