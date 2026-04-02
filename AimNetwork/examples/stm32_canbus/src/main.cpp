// AimNetwork STM32 CAN Example
// Demonstrates sending and receiving packets on the AIM bus.
// Copy this folder as a starting point for new STM32 modules.

#include <Arduino.h>
#include <aim_network.h>
#include <aim_can_driver.h>
#include <SoftwareSerial.h>

#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
#define USB_RX_PIN PB10
#define USB_TX_PIN PB11
#define DB_LED_PIN PA15
#define USB_BAUD   9600

SoftwareSerial usb(USB_RX_PIN, USB_TX_PIN);

AimCanDriver canHw(AIM_ORG_GPS, 500000, CAN_RX_PIN, CAN_TX_PIN);
AimNetwork   aim(&canHw, AIM_ORG_GPS);

aimPkt rxPkt;

const uint32_t TX_INTERVAL = 1000;
uint32_t lastTx = 0;
uint32_t txCount = 0;


void setup() {
  usb.begin(USB_BAUD);
  pinMode(DB_LED_PIN, OUTPUT);
  aim.begin();
}


void loop() {

  // Receive
  while (aim.readPkt(rxPkt)) {
    aimPrintPkt(usb, rxPkt, "RX");

    switch (rxPkt.type) {
      case AIM_TYP_GPS_LAT:  break;
      case AIM_TYP_GPS_LONG: break;
      case AIM_TYP_ALT:      break;
      case AIM_TYP_LOWPW:    break;
      default:                break;
    }
  }

  // Transmit
  if (millis() - lastTx > TX_INTERVAL) {
    lastTx = millis();

    // Quick send
    aim.sendPkt(millis(), txCount, AIM_DEST_COMMS, AIM_TYP_GPS_LAT);

    // Explicit send — build, inspect, send
    aimPkt txPkt;
    txPkt.origin = AIM_ORG_GPS;
    txPkt.dest   = AIM_DEST_BROADCAST;
    txPkt.type   = AIM_TYP_TIME;
    txPkt.data   = aimPkt::packData(millis(), 0);
    aimPrintPkt(usb, txPkt, "TX");
    aim.sendPkt(txPkt);

    digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
    txCount++;
  }
}
