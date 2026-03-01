// Canbus Receiver

#include <aim_can_driver.h>
#include <SoftwareSerial.h>
#include <cstdint>
#include <cstring>
#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
#define USB_RX_PIN PB10 // For com board
#define USB_TX_PIN PB11 // For com board
#define DB_LED_PIN PA15

#define USB_BAUD 9600

// FUCK hardware serial periperhals we doin SOFTWARE
SoftwareSerial usb(USB_RX_PIN, USB_TX_PIN); 

// AimNetwork instance (Comms Board Origin)
AimCanDriver canHw(AIM_ORG_GPS, 62500);
AimNetwork aimn(&canHw, AIM_ORG_GPS);
// no separate data packet type – we work directly with aimPkt

unsigned int packet_cnt=0;



void setup() {
  // setup AIM network
  aimn.begin();
  
  // setup usb serial
  usb.begin(USB_BAUD);

  pinMode(DB_LED_PIN, OUTPUT);

  usb.print("Size of aimPkt:");
  usb.println(sizeof(aimPkt));
}//setup()

aimPkt pkt;

void loop() {  
  if(aimn.readPkt(pkt)){
    usb.print("Received Packet #");
    usb.print(packet_cnt);
    usb.print(": origin=0x");
    usb.print(pkt.origin, HEX);
    usb.print(", type=0x");
    usb.print(pkt.type, HEX);
    usb.print(", data=0x");
    usb.print(pkt.getData(), HEX);
    usb.print(", =");
    usb.print(pkt.getDayMilis(),HEX);
    usb.println("ms");

    digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
    packet_cnt = ++packet_cnt;
  }
}//loop()
