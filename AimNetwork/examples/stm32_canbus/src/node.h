#ifndef NODE_H
#define NODE_H

#include <Arduino.h>
#include <cstdint>

#include <aim_can_driver.h>
#include <aim_network.h>
#include <aim_safety.h>

// Board-level identity and interface configuration lives in this file.
#define NODE_ORIGIN AIM_ORG_ALT
#define NODE_NAME "STM32_CANBUS"

#define NODE_CAN_BAUD 500000U

#if defined(CAN1)
#define NODE_CAN_BUS CAN1
#else
#error "Define NODE_CAN_BUS for this STM32 target."
#endif

#define NODE_SERIAL_RX_PIN PA10
#define NODE_SERIAL_TX_PIN PA9

#define NODE_SERIAL_BAUD 38400U

// Flash debug storage configuration (SPI flash on PB12-PB15).
#define NODE_FLASH_CS_PIN PB12
#define NODE_FLASH_SCK_PIN PB13
#define NODE_FLASH_MISO_PIN PB14
#define NODE_FLASH_MOSI_PIN PB15

// Number of columns stored in each flash table row.
#define NODE_FLASH_TABLE_COLS 1U
// Interval, in rows, between origin/metadata refresh operations.
#define NODE_FLASH_ORIGIN_REFRESH_INT 64U
// Total size, in bytes, reserved for the flash table.
#define NODE_FLASH_TABLE_SIZE 65536U
// Flash table instance index used by this node.
#define NODE_FLASH_TABLE_NUM 0U
// Scratch/data buffer size, in bytes, allocated in STM32 MCU RAM for flash table operations.
#define NODE_MCU_BUFFER_SIZE 256U

enum NodeState : uint8_t {
  INIT = 0U,
  OPERATIONAL = 1U,
  DEBUG_CONSOLE = 2U,
  FLASH_DUMP = 3U,
  SAFE_MODE = 4U,
  LOW_POWER = 5U,
  FAULT = 6U
};

void board_init(void);
// Add board-specific periodic behavior in board_update(state).
// The current node state is provided so board logic can vary by system mode.
void board_update(NodeState state);

#endif  // NODE_H
