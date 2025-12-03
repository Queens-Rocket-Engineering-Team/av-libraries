#include "CAN_PACKET.h"
#include <cstring>
#include <iostream>
#include <cstdint>

void sendPacket() {
		AIM_packet pkt; // instance pkt = packet
		
		// assigning test values
    pkt.AIM_id.unused= 31;
    pkt.AIM_id.future_use = 0x2;
    pkt.AIM_id.origin = 0x2;
    pkt.AIM_id.dest = 0x3;

    // packing bits
    uint16_t idPacked =
        ((pkt.AIM_id.unused     & 0x1F) << 11) |
        ((pkt.AIM_id.origin     & 0x0F) << 7)  |
        ((pkt.AIM_id.dest       & 0x0F) << 3)  |
        ((pkt.AIM_id.future_use & 0x07));

    
    // assigning test values
    pkt.AIM_data.dayMilis = 23; 
    pkt.AIM_data.data = 123456;
    
    
    // printing values for id for testing in compiler
		   std::cout << "Packed CAN ID (binary): "
              << std::bitset<16>(idPacked)
              << std::endl;
              
		   std::cout << "Packed CAN ID (hex): 0x"
              << std::hex << idPacked
              << std::endl;
    //
   
		CAN_message_t CAN_msg{};
		CAN_msg.id = idPacked & 0x07FF;          // use packed ID from pkt.AIM_id
		CAN_msg.flags.extended = 0;              // standard 11-bit frame
		CAN_msg.len = sizeof(pkt.AIM_data);      // 8 bytes

		std::memcpy(CAN_msg.buf, &pkt.AIM_data, sizeof(pkt.AIM_data));

}