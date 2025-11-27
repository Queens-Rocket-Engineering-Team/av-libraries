
#ifndef CAN_PACKET_H
#define CAN_PACKET_H
 

struct id{
// setting bit fields for id. (distributing bits)
	uint8_t unused : 5;
  uint8_t origin : 4;
  uint8_t dest : 4;
  uint8_t future_use : 3;
}; // 11 out of 16 in use

struct dataPacket{
// setting bit fields for data&time. (distributing bits)
	uint32_t dayMilis; // time of day in miliseconds
	uint32_t data;
};

// AIM Packet (Avionics Interconnected Module)
struct AIM_packet{ // do we need typedef here? - Liam
  struct id AIM_id;
  struct dataPacket AIM_data;
};

#endif // CAN_PACKET_H