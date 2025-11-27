// haha oh SHIT (transmitter edition)

#include "STM32_CAN.h"
#include <SoftwareSerial.h>
#include "CAN_PACKET.h"
#include <bitset>
#include <iostream>
#include <cstdint>
#include <cstring>
#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
#define USB_RX_PIN PB10 // NOT CONNECTED
#define USB_TX_PIN PB11 // NOT CONNECTED
#define DB_LED_PIN PA15


#define USB_BAUD 9600 // FUCK IT 9600 BOYSSS


// FUCK hardware serial periperhals we doin SOFTWARE
SoftwareSerial usb(USB_RX_PIN, USB_TX_PIN); 
// CANBUS trash wahooo
STM32_CAN canb( CAN1, ALT );    //CAN1 ALT is PB8+PB9
static CAN_message_t CAN_msg;


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
   
		CAN_message_t CAN_msg{};
		CAN_msg.id = idPacked & 0x07FF;          // use packed ID from pkt.AIM_id
		CAN_msg.flags.extended = 0;              // standard 11-bit frame
		CAN_msg.len = sizeof(pkt.AIM_data);      // 8 bytes

		std::memcpy(CAN_msg.buf, &pkt.AIM_data, sizeof(pkt.AIM_data));
    //canb.write(CAN_msg);     //send

    //usb.println(CAN_msg.id);
    //usb.println(CAN_msg.len);
    for(int i = 0; i < CAN_msg.len; i++){
      usb.print("CAN Data index: ");
      usb.println(i+1);
      usb.print("0x");
      usb.println(CAN_msg.buf[i], HEX);
      usb.print("0b");
      usb.println(CAN_msg.buf[i], BIN);
    }

}
void canSend(){
  canb.write(CAN_msg);     //send
}//canSend()



void setup() {
  // setup canubs
  canb.begin(); //automatic retransmission can be done using arg "true"
  canb.setBaudRate(62500); //62.5kbps
  // setup usb serial
  usb.begin(USB_BAUD);

  pinMode(DB_LED_PIN, OUTPUT);
}//setup()

void loop() {
  // put your main code here, to run repeatedly:
  usb.println("Sending CAN packet");
  sendPacket();
  delay(1000);
  digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
}//loop()
