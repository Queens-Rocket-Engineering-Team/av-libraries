#ifndef CAN_PACKET_H
#define CAN_PACKET_H

typedef struct dataPacket {
// setting bit fields for data&time. (distributing bits)
	uint32_t dayMilis; // time of day in miliseconds
	uint32_t data;
}dataPacket;

// AIM Packet (Avionics Interconnected Module)
typedef struct AIM_packet { // do we need typedef here? - Liam
  uint8_t unused : 5;
  uint8_t origin : 4;
  uint8_t dest : 4;
  uint8_t future_use : 3;
  dataPacket AIM_data;
} AIM_packet;

bool packAimPkt(AIM_packet AIM_pkt, CAN_message_t &CAN_msg) {

  // packing bits
  uint16_t idPacked =
      ((AIM_pkt.origin     & 0x0F) << 7)  |
      ((AIM_pkt.dest       & 0x0F) << 3)  |
      ((AIM_pkt.future_use & 0x07));

  CAN_msg.id = idPacked & 0x07FF;          // use packed ID from pkt.AIM_id
  CAN_msg.flags.extended = 0;              // standard 11-bit frame
  CAN_msg.len = sizeof(AIM_pkt.AIM_data);      // 8 bytes

  memcpy(CAN_msg.buf, &AIM_pkt.AIM_data, sizeof(AIM_pkt.AIM_data));
  return true;
}

bool unpackAimPkt(CAN_message_t CAN_msg, AIM_packet &AIM_pkt) {

  uint16_t idPacked = CAN_msg.id;
  AIM_pkt.origin = (idPacked >> 7) & 0x0F;
  AIM_pkt.dest = (idPacked >> 3) & 0x0F;
  AIM_pkt.future_use = idPacked & 0x07;

  if (sizeof(AIM_pkt.AIM_data)==CAN_msg.len) {
      memcpy(&AIM_pkt.AIM_data, CAN_msg.buf, CAN_msg.len); // copy all data from
  }
  else {
      return false; //didnt work
  }
  return true;//worked
} 
#endif // CAN_PACKET_H















