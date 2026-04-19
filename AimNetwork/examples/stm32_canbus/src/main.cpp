#include "node.h"
#include "console.h"

#include <IWatchdog.h>
#include <logger.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <flash_table.h>

static constexpr uint32_t kWatchdogTimeoutUs  = 2000000U;
static constexpr uint8_t  kMaxRxFramesPerLoop = 8U;

struct NodeSchedulerState {
  NodeState value = INIT;  // setup() always transitions to OPERATIONAL before loop runs
  uint32_t lastHeartbeatTxMs = 0U;
};

static AimCanDriver g_canHw(NODE_ORIGIN, NODE_CAN_BAUD, NODE_CAN_BUS);
static AimNetwork g_aim(&g_canHw, NODE_ORIGIN);
static SoftwareSerial g_serial(NODE_SERIAL_RX_PIN, NODE_SERIAL_TX_PIN);
static Logger g_log(g_serial, NODE_ORIGIN, LogLevel::INFO);
static uint32_t g_flashLastVals[NODE_FLASH_TABLE_COLS];
static uint8_t g_flashIoBuffer[NODE_MCU_BUFFER_SIZE];
static FlashTable g_flashTable(
  &SerialFlash,
  NODE_FLASH_TABLE_COLS,
  NODE_FLASH_ORIGIN_REFRESH_INT,
  NODE_FLASH_TABLE_SIZE,
  NODE_FLASH_TABLE_NUM,
  NODE_MCU_BUFFER_SIZE,
  g_flashLastVals,
  g_flashIoBuffer);
static NodeSchedulerState g_schedulerState = {};

void service_can_rx(uint32_t networkNowMs) {
  // Handle incoming bus messages and custom packet branches here.
  (void)networkNowMs;
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aimPkt pkt = {};
    if (!g_aim.readPkt(pkt)) {
      break;
    }

    if (pkt.type == AIM_TYP_TIME) {
      g_aim.syncTime(static_cast<uint32_t>(pkt.getPayload64()));
      LOG_DEBUG("Time sync received: networkNowMs=%u", g_aim.syncedMillis());
    }
  }
}

void service_can_tx(uint32_t networkNowMs) {
  // Add periodic transmit-side behavior in this service pattern.
  const uint32_t scheduleNowMs = millis();

  // TX SECTION 1: node heartbeat.
  if ((scheduleNowMs - g_schedulerState.lastHeartbeatTxMs) >= AIM_HEARTBEAT_TX_INTERVAL_DEFAULT_MS) {
    g_schedulerState.lastHeartbeatTxMs = scheduleNowMs;
    const uint32_t payload = static_cast<uint32_t>(g_schedulerState.value);
    if (!g_aim.sendPkt32(networkNowMs, payload, AIM_DEST_BROADCAST, AIM_TYP_HEARTBEAT)) {
      LOG_ERROR("Heartbeat TX failed");
    } else {
      LOG_DEBUG("Heartbeat TX ok");
    }
  }

  // TX SECTION 2: reserved for future periodic TX behavior.
  // TX SECTION 3: reserved for future periodic TX behavior.
}

void run_state_machine(uint32_t networkNowMs) {
  AIM_ASSERT(g_schedulerState.value <= FAULT);  // precondition: corrupted state → reset

  switch (g_schedulerState.value) {
    case OPERATIONAL: {
#ifndef FLIGHT_BUILD
      const ConsoleAction act = consoleCheckEntry();
      if (act == CONSOLE_ACTION_ENTER) {
        g_schedulerState.value = DEBUG_CONSOLE;
      }
#endif
      break;
    }

#ifndef FLIGHT_BUILD
    case DEBUG_CONSOLE: {
      const ConsoleAction act = consoleService(
          static_cast<uint8_t>(g_schedulerState.value), networkNowMs);
      if (act == CONSOLE_ACTION_EXIT) {
        g_schedulerState.value = OPERATIONAL;
      } else if (act == CONSOLE_ACTION_FLASH_DUMP) {
        g_schedulerState.value = FLASH_DUMP;
      }
      break;
    }

    case FLASH_DUMP: {
      const ConsoleAction act = consoleServiceFlashDump();
      if (act == CONSOLE_ACTION_DUMP_DONE) {
        g_schedulerState.value = DEBUG_CONSOLE;
      }
      break;
    }
#endif

    case SAFE_MODE:
    case LOW_POWER:
    case FAULT:
      break;

    default:
      AIM_ASSERT(false);  // unreachable — all valid states handled above
      break;
  }

  node_update();
  service_can_tx(networkNowMs);
}

void node_update(void) {
  // BOARD EXTENSION POINT: add recurring board logic here.
}

void setup(void) {
  AIM_ASSERT(NODE_ORIGIN <= AIM_ORG_ADDR_MAX);
  g_serial.begin(NODE_SERIAL_BAUD);
  g_logger = &g_log;
  LOG_INFO("Boot node origin=%u", static_cast<unsigned>(NODE_ORIGIN));
  IWatchdog.begin(kWatchdogTimeoutUs);
  LOG_INFO("Watchdog ready");

  g_aim.begin();
#ifndef FLIGHT_BUILD
  consoleInit(g_serial, g_aim, g_log, g_flashTable);
#endif

  // BOARD EXTENSION POINT: add one-time board setup here.
  SPI.setSCLK(NODE_FLASH_SCK_PIN);
  SPI.setMISO(NODE_FLASH_MISO_PIN);
  SPI.setMOSI(NODE_FLASH_MOSI_PIN);
  SPI.begin();
  if (SerialFlash.begin(NODE_FLASH_CS_PIN)) {
    g_flashTable.init(&g_serial);
  }
  if (g_flashTable.isReady()) {
    LOG_INFO("Flash ready");
  } else {
    LOG_WARN("Flash init failed");
  }

#ifndef FLIGHT_BUILD
  g_serial.println("Console ready. d=enter debug");
#endif
  g_schedulerState.lastHeartbeatTxMs = millis();
  g_schedulerState.value = OPERATIONAL;
}

void loop(void) {
  const uint32_t networkNowMs = g_aim.syncedMillis();
  // Main scheduler order: RX, state machine, watchdog.
  service_can_rx(networkNowMs);
  run_state_machine(networkNowMs);

  IWatchdog.reload();
}
