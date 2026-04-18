#include "node.h"

#include <IWatchdog.h>
#include <logger.h>
#include <SoftwareSerial.h>

static constexpr uint32_t kHeartbeatTxIntervalMs = AIM_HEARTBEAT_TX_INTERVAL_DEFAULT_MS;
static constexpr uint32_t kWatchdogTimeoutUs = 2000000U;
static constexpr uint8_t kMaxRxFramesPerLoop = 8U;

struct NodeSchedulerState {
  NodeState value = INIT;
  uint32_t lastHeartbeatTxMs = 0U;
};

static AimCanDriver g_canHw(NODE_ORIGIN, NODE_CAN_BAUD, NODE_CAN_BUS);
static AimNetwork g_aim(&g_canHw, NODE_ORIGIN);
static SoftwareSerial g_serial(NODE_SERIAL_RX_PIN, NODE_SERIAL_TX_PIN);
static Logger g_log(g_serial, NODE_ORIGIN, LogLevel::INFO);
static NodeSchedulerState g_schedulerState = {};

void service_can_rx(void) {
  // Handle incoming bus messages and custom packet branches here.
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

void service_tx(uint32_t networkNowMs) {
  // Add periodic transmit-side behavior in this service pattern.
  const uint32_t scheduleNowMs = millis();

  // TX SECTION 1: node heartbeat.
  if ((scheduleNowMs - g_schedulerState.lastHeartbeatTxMs) >= kHeartbeatTxIntervalMs) {
    g_schedulerState.lastHeartbeatTxMs = scheduleNowMs;
    const uint32_t payload = static_cast<uint32_t>(g_schedulerState.value);
    const bool heartbeatSent = g_aim.sendPkt32(networkNowMs, payload, AIM_DEST_BROADCAST, AIM_TYP_HEARTBEAT);
    if (!heartbeatSent) {
      LOG_ERROR("Heartbeat TX failed");
    } else {
      LOG_DEBUG("Heartbeat TX ok");
    }
  }

  // TX SECTION 2: reserved for future periodic TX behavior.
  // TX SECTION 3: reserved for future periodic TX behavior.
}

void run_state_machine(uint32_t networkNowMs) {
  if (g_schedulerState.value > FAULT) {
    g_schedulerState.value = FAULT;
  }

  AIM_ASSERT(g_schedulerState.value <= FAULT);
  if (g_schedulerState.value == INIT) {
    board_init();
    g_schedulerState.lastHeartbeatTxMs = millis();
    g_schedulerState.value = OPERATIONAL;
    LOG_INFO("State transition INIT -> OPERATIONAL");
    return;
  }

  board_update(g_schedulerState.value);
  service_tx(networkNowMs);
}

void board_init(void) {
  // BOARD EXTENSION POINT: add one-time board setup here.
  AIM_ASSERT(NODE_ORIGIN <= AIM_ORG_ADDR_MAX);
}

void board_update(NodeState state) {
  // BOARD EXTENSION POINT: add recurring board logic here.
  AIM_ASSERT(state <= FAULT);
  (void)state;
}

void setup(void) {
  AIM_ASSERT(NODE_ORIGIN <= AIM_ORG_ADDR_MAX);
  g_serial.begin(NODE_SERIAL_BAUD);
  g_logger = &g_log;
  LOG_INFO("Boot node origin=%u", static_cast<unsigned>(NODE_ORIGIN));
  IWatchdog.begin(kWatchdogTimeoutUs);
  LOG_INFO("Watchdog ready");
  g_aim.begin();

  g_schedulerState.value = INIT;
}

void loop(void) {
  AIM_ASSERT(g_schedulerState.value <= FAULT);

  const uint32_t networkNowMs = g_aim.syncedMillis();
  // Main scheduler order: RX, state machine, watchdog.
  service_can_rx();
  run_state_machine(networkNowMs);

  IWatchdog.reload();
}
