// Canbus transmitter and self test

#include "STM32_CAN.h"
#include <SoftwareSerial.h>
#include "CAN_PACKET.h"
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

AIM_packet AIM_pkt;

void canSend(){
  //canb.write(can_msg);     //send
}//canSend()


void setup() {
  // setup canubs
  canb.begin(); //automatic retransmission can be done using arg "true"
  canb.setBaudRate(62500); //62.5kbps
  // setup usb serial
  usb.begin(USB_BAUD);

  pinMode(DB_LED_PIN, OUTPUT);
  
  AIM_pkt.origin = 0x1;
  AIM_pkt.dest = 0x2;
  AIM_pkt.AIM_data.dayMilis = 0x1234;
  AIM_pkt.AIM_data.data = 0x123;

}//setup()

void loop() {
  // put your main code here, to run repeatedly:
  usb.println("Sending CAN packet");
  // sendPacket();
  usb.print("AIM origin: 0x");
  usb.println(AIM_pkt.origin, HEX);
  usb.print("AIM dest: 0x");
  usb.println(AIM_pkt.dest, HEX);
  usb.print("AIM data: 0x");
  usb.println(AIM_pkt.AIM_data.data, HEX);
  usb.print("AIM mili: 0x");
  usb.println(AIM_pkt.AIM_data.dayMilis, HEX);
  CAN_message_t can_msg1;
  packAimPkt(AIM_pkt, can_msg1);

  // Unpack code, can be used to test self
  /*
  AIM_packet AIM_pkt2;
  unpackAimPkt(can_msg1, AIM_pkt2);
  
  usb.print("AIM origin2: 0x");
  usb.println(AIM_pkt2.origin, HEX);
  usb.print("AIM dest2: 0x");
  usb.println(AIM_pkt2.dest, HEX);
  usb.print("AIM data2: 0x");
  usb.println(AIM_pkt2.AIM_data.data, HEX);
  usb.print("AIM mili2: 0x");
  usb.println(AIM_pkt2.AIM_data.dayMilis, HEX);

 
  */
  delay(1000);
  digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
  canb.write(can_msg1);
}//loop()