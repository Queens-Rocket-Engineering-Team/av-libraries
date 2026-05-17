#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_can_driver.h>
#include <aim_network.h>
#include <aim_safety.h>

// Node-level identity and interface configuration lives in this file.
#define NODE_ORIGIN AIM_ORG_UPROP
#define NODE_NAME "ESP32_CANBUS"

#define NODE_CAN_BAUD 500000U
#define NODE_CAN_RX_PIN 4
#define NODE_CAN_TX_PIN 5

#define NODE_SERIAL_BAUD 115200U

enum NodeState : uint8_t {
  INIT = 0U,
  OPERATIONAL = 1U,
  DEBUG_CONSOLE = 2U,
  FLASH_DUMP = 3U,
  FLASH_ERASE = 4U,
  SAFE_MODE = 5U,
  LOW_POWER = 6U,
  FAULT = 7U
};

// Add node-specific periodic behavior in nodeUpdate().
void nodeUpdate(uint32_t schedulerNowMs);

#endif  // NODE_H
