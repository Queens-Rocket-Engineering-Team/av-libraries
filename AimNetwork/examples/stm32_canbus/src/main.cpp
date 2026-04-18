#include "node.h"

#include <IWatchdog.h>
#include <logger.h>
#include <SoftwareSerial.h>

static constexpr uint32_t kHeartbeatTxIntervalMs = 250U;
static constexpr uint32_t kWatchdogTimeoutUs = 2000000U;
static constexpr uint8_t kMaxRxFramesPerLoop = 8U;

static_assert(kHeartbeatTxIntervalMs > 0U, "Heartbeat interval must be > 0");
static_assert(kWatchdogTimeoutUs > 0U, "Watchdog timeout must be > 0");
static_assert(kMaxRxFramesPerLoop > 0U, "RX frame budget must be > 0");

static AimCanDriver g_canHw(NODE_ORIGIN, NODE_CAN_BAUD, NODE_CAN_BUS);
static AimNetwork g_aim(&g_canHw, NODE_ORIGIN);
static NodeState g_nodeState = INIT;
static uint32_t g_lastHeartbeatTxMs = 0U;
static SoftwareSerial g_serial(NODE_SERIAL_RX_PIN, NODE_SERIAL_TX_PIN);
static Logger g_log(g_serial, NODE_ORIGIN, LogLevel::DEBUG);

void service_can_rx(void) {
  // Handle incoming bus messages and custom packet branches here.
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aimPkt pkt = {};
    if (!g_aim.readPkt(pkt)) {
      break;
    }

    if (pkt.type == AIM_TYP_TIME) {
      g_aim.syncTime(pkt.getMillis());
      LOG_DEBUG("Time sync received: networkNowMs=%u", g_aim.syncedMillis());
    }
  }
}

void service_tx(uint32_t networkNowMs) {
  // Add periodic transmit-side behavior in this service pattern.
  const uint32_t scheduleNowMs = millis();
  if ((scheduleNowMs - g_lastHeartbeatTxMs) < kHeartbeatTxIntervalMs) {
    return;
  }

  g_lastHeartbeatTxMs = scheduleNowMs;
  const uint32_t payload = static_cast<uint32_t>(g_nodeState) & 0xFFU;
  const bool heartbeatSent = g_aim.sendPkt(networkNowMs, payload, AIM_DEST_BROADCAST, AIM_TYP_HEARTBEAT);
  if (!heartbeatSent) {
    LOG_ERROR("Heartbeat TX failed");
  } else {
    LOG_DEBUG("Heartbeat TX ok");
  }

}

void run_state_machine(uint32_t networkNowMs) {
  if (g_nodeState > FAULT) {
    g_nodeState = FAULT;
  }

  AIM_ASSERT(g_nodeState <= FAULT);
  if (g_nodeState == INIT) {
    board_init();
    g_lastHeartbeatTxMs = millis();
    g_nodeState = OPERATIONAL;
    LOG_INFO("State transition INIT -> OPERATIONAL");
    return;
  }

  board_update(networkNowMs, g_nodeState);
}

void board_init(void) {
  AIM_ASSERT((NODE_ORIGIN & 0xF8U) == 0U);
  // BOARD EXTENSION POINT: add one-time board setup here.
}

void board_update(uint32_t networkNowMs, NodeState state) {
  AIM_ASSERT(state <= FAULT);
  (void)networkNowMs;
  (void)state;
  // BOARD EXTENSION POINT: add recurring board logic here.
}

void setup(void) {
  AIM_ASSERT((NODE_ORIGIN & 0xF8U) == 0U);
  g_serial.begin(NODE_SERIAL_BAUD);
  g_logger = &g_log;
  LOG_INFO("Boot node origin=%u", static_cast<unsigned>(NODE_ORIGIN));
  IWatchdog.begin(kWatchdogTimeoutUs);
  LOG_INFO("Watchdog ready");
  g_aim.begin();

  g_lastHeartbeatTxMs = millis();
  g_nodeState = INIT;
}

void loop(void) {
  AIM_ASSERT(g_nodeState <= FAULT);

  const uint32_t networkNowMs = g_aim.syncedMillis();
  // Main scheduler order: RX, TX/services, state machine, watchdog.
  service_can_rx();
  service_tx(networkNowMs);
  run_state_machine(networkNowMs);

  IWatchdog.reload();
}