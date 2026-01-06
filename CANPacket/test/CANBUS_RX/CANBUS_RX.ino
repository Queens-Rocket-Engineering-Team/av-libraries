// Canbus Receiver

#include "STM32_CAN.h"
#include <SoftwareSerial.h>
#include "CAN_PACKET.h"
#include <cstdint>
#include <cstring>
#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
#define USB_RX_PIN PA10 // for GPS board
#define USB_TX_PIN PA9 // for GPS board
#define DB_LED_PIN PA15

#define USB_BAUD 9600


// FUCK hardware serial periperhals we doin SOFTWARE
SoftwareSerial usb(USB_RX_PIN, USB_TX_PIN); 
// CANBUS trash wahooo
STM32_CAN canb(CAN1,ALT);    //CAN1 ALT is PB8+PB9




void setup() {
  // setup canubs
  canb.begin(); //automatic retransmission can be done using arg "true"
  canb.setBaudRate(62500); //62.5kbps
  canb.setMBFilterProcessing(MB0, 0x008, 0x008); // 000 0111 1000 //000 1001 0 000
  // setup usb serial
  usb.begin(USB_BAUD);

  pinMode(DB_LED_PIN, OUTPUT);


}//setup()

void loop() {
  // put your main code here, to run repeatedly:
  CAN_message_t CAN_msg1;
  //usb.println("I'm alive RX");
  while(canb.read(CAN_msg1)){
    AIM_packet AIM_pkt2;
    unpackAimPkt(CAN_msg1, AIM_pkt2);
    usb.println("=========================");

    usb.print("AIM origin2: 0x");
    usb.println(AIM_pkt2.origin, HEX);
    usb.print("AIM dest2: 0x");
    usb.println(AIM_pkt2.dest, HEX);
    usb.print("AIM data2: 0x");
    usb.println(AIM_pkt2.AIM_data.data, HEX);
    usb.print("AIM mili2: 0x");
    usb.println(AIM_pkt2.AIM_data.dayMilis, HEX);

    digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
    /*
    while (canb.read(CAN_msg)) {
      usb.print(F("CAN ID = "));
      usb.println(CAN_msg.id, BIN);
      usb.print("CAN PAYLOAD = ");
      usb.print(CAN_msg.buf[0], HEX);
      usb.print(CAN_msg.buf[1], HEX);
      usb.print(CAN_msg.buf[2], HEX);
      usb.print(CAN_msg.buf[3], HEX);
      digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
    
    }//while
    */
  }
}//loop()
