// AimNetwork ESP32 CAN Example
// Demonstrates sending and receiving packets on the AIM bus.
// Copy this folder as a starting point for new ESP32 modules.
//
// NOTE: ESP32 CAN driver is not yet implemented.
// This example will compile but CAN send/receive will return false.

#include <Arduino.h>
#include <aim_network.h>
#include <aim_can_driver.h>

#define CAN_RX_PIN 4   // adjust to your board
#define CAN_TX_PIN 5   // adjust to your board
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
}


void loop() {

  while (aim.readPkt(rxPkt)) {
    aimPrintPkt(Serial, rxPkt, "RX");
  }

  if (millis() - lastTx > TX_INTERVAL) {
    lastTx = millis();

    aim.sendPkt(millis(), txCount, AIM_DEST_COMMS, AIM_TYP_PT1);

    digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
    txCount++;
  }
}
