// AimNetwork STM32 CAN Example
// Demonstrates sending and receiving packets on the AIM bus.
// Copy this folder as a starting point for new STM32 modules.

#include <Arduino.h>
#include <aim_network.h>
#include <aim_can_driver.h>
#include <SoftwareSerial.h>

// CAN pins do not change between COMMS and GPS
#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
// COMMS pins for USB serial
// #define USB_RX_PIN PB10 
// #define USB_TX_PIN PB11 
// GPS pins for GPS serial 
#define USB_RX_PIN PA10
#define USB_TX_PIN PA9 



#define DB_LED_PIN PA15
#define USB_BAUD   9600

SoftwareSerial usb(USB_RX_PIN, USB_TX_PIN);

AimCanDriver canHw(AIM_ORG_GPS, 500000);
AimNetwork   aim(&canHw, AIM_ORG_GPS);

aimPkt rxPkt;

const uint32_t TX_INTERVAL = 100;
uint32_t lastTx = 0;
uint32_t txCount = 0;


void setup() {
  usb.begin(USB_BAUD);
  pinMode(DB_LED_PIN, OUTPUT);
  aim.begin();
  delay(1000);
  usb.println("USB ALIVE");
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
      case AIM_TYP_TIME:  
        aim.syncTime(rxPkt.getMillis());
        break;
      default:                break;
    }
  }

  // Transmit
  if (aim.syncedMillis() - lastTx > TX_INTERVAL) {
    lastTx = aim.syncedMillis();

    // Explicit send — build, inspect, send
    aimPkt txPkt;
    txPkt.origin = AIM_ORG_GPS;
    txPkt.dest   = AIM_DEST_BROADCAST;
    txPkt.type   = AIM_TYP_TIME;
    txPkt.data   = aimPkt::packData(aim.syncedMillis(), txCount);
    aimPrintPkt(usb, txPkt, "TX");
    aim.sendPkt(txPkt);
    usb.println("Packet sent");

    digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
    txCount++;
  }
}
