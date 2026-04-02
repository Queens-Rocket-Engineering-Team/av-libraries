// AimNetwork CAN RX Example
// Receives and prints all incoming AIM packets.

#include <aim_network.h>
#include <aim_can_driver.h>
#include <SoftwareSerial.h>

#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
#define USB_RX_PIN PB10 // For com board
#define USB_TX_PIN PB11 // For com board
#define DB_LED_PIN PA15
#define USB_BAUD   9600

SoftwareSerial usb(USB_RX_PIN, USB_TX_PIN);

AimCanDriver canHw(AIM_ORG_COMMS, 500000, CAN_RX_PIN, CAN_TX_PIN);
AimNetwork   aim(&canHw, AIM_ORG_COMMS);

aimPkt rxPkt;
uint32_t rxCount = 0;


void setup() {
  usb.begin(USB_BAUD);
  pinMode(DB_LED_PIN, OUTPUT);
  aim.begin();

  usb.print("aimPkt size: ");
  usb.println(sizeof(aimPkt));
}


void loop() {
  while (aim.readPkt(rxPkt)) {
    usb.print("#");
    usb.print(rxCount);
    usb.print(" ");
    aimPrintPkt(usb, rxPkt, "RX");

    digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
    rxCount++;
  }
}
