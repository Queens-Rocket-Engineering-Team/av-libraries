// Canbus Receiver

#include <aim_network.h>
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
AimNetwork aimn(AIM_ORG_COMMS, 62500);

// Data containers
dataPkt recv_data;
uint8_t recv_origin;
uint8_t recv_type;

void setup() {
  // setup AIM network
  aimn.begin();
  
  // setup usb serial
  usb.begin(USB_BAUD);

  pinMode(DB_LED_PIN, OUTPUT);
}//setup()

void loop() {
  uint32_t num_recv = 0;
  
  if(aimn.readPkt(recv_data, recv_origin, recv_type)){
    usb.print("Received Packet #");
    usb.print(num_recv);
    usb.print(": origin=0x");
    usb.print(recv_origin, HEX);
    usb.print(", type=0x");
    usb.print(recv_type, HEX);
    usb.print(", data=0x");
    usb.print(recv_data.data, HEX);
    usb.print(", =");
    usb.print(recv_data.dayMilis);
    usb.println("ms");

    digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
    num_recv++;
  }
}//loop()
