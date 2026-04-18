#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_can_driver.h>
#include <aim_network.h>
#include <aim_safety.h>

// Board-level identity and interface configuration lives in this file.
#define NODE_ORIGIN AIM_ORG_UPROP

#define NODE_CAN_BAUD 500000U

#define NODE_CAN_RX_PIN 4
#define NODE_CAN_TX_PIN 5

#define NODE_SERIAL_RX_PIN 16
#define NODE_SERIAL_TX_PIN 17
#define NODE_SERIAL_BAUD 38400U

// Board-level health monitor configuration.
#define NODE_ENABLE_HEALTH_MONITOR 1U
#define NODE_HEALTH_TIMEOUT_MS 750U

enum NodeState : uint8_t {
  INIT = 0U,
  OPERATIONAL = 1U,
  DEGRADED = 2U,
  SAFE_MODE = 3U,
  FAULT = 4U
};

void board_init(void);
// Add board-specific periodic behavior in board_update().
void board_update(NodeState state);

#endif  // NODE_H
