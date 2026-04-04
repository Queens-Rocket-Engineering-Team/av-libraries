// AimNetwork ESP32 CAN Example
// Demonstrates sending and receiving packets on the AIM bus.
// Copy this folder as a starting point for new ESP32 modules.
//
// ESP32 CAN support is currently in early stages and may not work all boards.

#include <Arduino.h>
#include <aim_network.h>
#include <aim_can_driver.h>

#define CAN_RX_PIN 0   // adjust to your board
#define CAN_TX_PIN 0   // adjust to your board
#define DB_LED_PIN 40

AimCanDriver canHw(AIM_ORG_UPROP, 500000, CAN_RX_PIN, CAN_TX_PIN);
AimNetwork   aim(&canHw, AIM_ORG_UPROP);

aimPkt rxPkt;

const uint32_t TX_INTERVAL = 1000;
uint32_t lastTx = 0;
uint32_t txCount = 0;


void setup() {
  Serial.begin(9600);
  pinMode(DB_LED_PIN, OUTPUT);
  aim.begin();
  delay(1000);
  Serial.println("ESP32 CAN ALIVE");
}


void loop() {
  Serial.println("ESP32 CAN ALIVE");
  // while (aim.readPkt(rxPkt)) { // currently blocking due to TWAI driver limitations; consider switching to non-blocking receive
  //   aimPrintPkt(Serial, rxPkt, "RX");
  // }

  if (aim.syncedMillis() - lastTx > TX_INTERVAL) {
    lastTx = aim.syncedMillis();
    aimPkt txPkt;
    Serial.println("Building packet...");
    txPkt.origin = AIM_ORG_GPS;
    txPkt.dest   = AIM_DEST_BROADCAST;
    txPkt.type   = AIM_TYP_TIME;
    Serial.println("packing packet...");
    txPkt.data   = aimPkt::packData(aim.syncedMillis(), txCount);
    Serial.println("sending packet...");
    aim.sendPkt(aim.syncedMillis(), txCount, AIM_DEST_COMMS, AIM_TYP_PT1);
    Serial.println("printing packet...");
    aimPrintPkt(Serial, txPkt, "TX");
    digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
    txCount++;
  }
}
