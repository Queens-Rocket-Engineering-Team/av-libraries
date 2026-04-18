#include "node.h"

#include <esp_idf_version.h>
#include <esp_task_wdt.h>
#include <logger.h>
#include <SoftwareSerial.h>

static constexpr uint32_t kHeartbeatTxIntervalMs = AIM_HEARTBEAT_TX_INTERVAL_DEFAULT_MS;
static constexpr uint32_t kWatchdogTimeoutMs = 2000U;
static constexpr uint8_t kMaxRxFramesPerLoop = 8U;
static constexpr uint8_t kTrackedNodeCount = 6U;

struct NodeSchedulerState {
  NodeState value = INIT;
  uint32_t lastHeartbeatTxMs = 0U;
};

static const uint8_t kTrackedNodeOrigins[kTrackedNodeCount] = {
  AIM_ORG_COMMS,
  AIM_ORG_UPROP,
  AIM_ORG_LPROP,
  AIM_ORG_ALT,
  AIM_ORG_GPS,
  AIM_ORG_PWR
};

static AimCanDriver g_canHw(NODE_ORIGIN, NODE_CAN_BAUD, NODE_CAN_RX_PIN, NODE_CAN_TX_PIN);
static AimNetwork g_aim(&g_canHw, NODE_ORIGIN);
static AimNodeHealth g_nodeHealth[kTrackedNodeCount] = {};
static bool g_lastAliveSnapshot[kTrackedNodeCount] = {};
static NodeSchedulerState g_schedulerState = {};
static bool g_watchdogReady = false;
static SoftwareSerial g_serial(NODE_SERIAL_RX_PIN, NODE_SERIAL_TX_PIN);
static Logger g_log(g_serial, NODE_ORIGIN, LogLevel::INFO);

void init_watchdog(void) {
#if ESP_IDF_VERSION_MAJOR >= 5
  const esp_task_wdt_config_t config = {
    .timeout_ms = kWatchdogTimeoutMs,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  const esp_err_t initStatus = esp_task_wdt_init(&config);
#else
  const esp_err_t initStatus = esp_task_wdt_init(kWatchdogTimeoutMs / 1000U, true);
#endif

  const bool initOk = (initStatus == ESP_OK) || (initStatus == ESP_ERR_INVALID_STATE);
  const esp_err_t addStatus = esp_task_wdt_add(NULL);
  const bool addOk = (addStatus == ESP_OK) || (addStatus == ESP_ERR_INVALID_STATE);
  g_watchdogReady = initOk && addOk;
  if (!g_watchdogReady) {
    LOG_ERROR("Watchdog init failed (init=%d add=%d)", static_cast<int>(initStatus), static_cast<int>(addStatus));
    g_schedulerState.value = FAULT;
    return;
  }

  LOG_INFO("Watchdog ready");
}

void kick_watchdog(void) {
  if (!g_watchdogReady) {
    return;
  }

  const esp_err_t status = esp_task_wdt_reset();
  if ((status != ESP_OK) && (status != ESP_ERR_INVALID_STATE)) {
    LOG_ERROR("Watchdog reset failed (%d)", static_cast<int>(status));
    g_schedulerState.value = FAULT;
  }
}

void init_node_health(uint32_t networkNowMs) {
  if (NODE_ENABLE_HEALTH_MONITOR == 0U) {
    LOG_INFO("Node-health monitor disabled");
    return;
  }

  const bool configured = g_aim.configureHealthMonitor(
      kTrackedNodeOrigins,
      kTrackedNodeCount,
      g_nodeHealth,
      NODE_HEALTH_TIMEOUT_MS,
      networkNowMs);
  AIM_ASSERT(configured);

  for (uint8_t i = 0U; i < kTrackedNodeCount; i++) {
    g_lastAliveSnapshot[i] = g_nodeHealth[i].alive;
  }

  LOG_INFO("Node-health monitor enabled (%u tracked)", static_cast<unsigned>(kTrackedNodeCount));
}

void service_can_rx(uint32_t networkNowMs) {
  // Handle incoming bus messages and custom packet branches here.
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aimPkt pkt = {};
    if (!g_aim.readPkt(pkt)) {
      break;
    }

    if (pkt.type == AIM_TYP_TIME) {
      g_aim.syncTime(static_cast<uint32_t>(pkt.getPayload64()));
    }
    if (pkt.type == AIM_TYP_HEARTBEAT) {
      g_aim.updateHealthOnHeartbeat(pkt.origin, networkNowMs);
    }
  }
}

void service_node_health_monitor(uint32_t networkNowMs) {
  if (NODE_ENABLE_HEALTH_MONITOR == 0U) {
    return;
  }

  g_aim.evaluateHealth(networkNowMs);
  for (uint8_t i = 0U; i < kTrackedNodeCount; i++) {
    const AimNodeHealth& health = g_nodeHealth[i];
    const uint8_t origin = health.origin;
    if (origin == NODE_ORIGIN) {
      continue;
    }

    if (health.alive == g_lastAliveSnapshot[i]) {
      continue;
    }

    g_lastAliveSnapshot[i] = health.alive;
    if (health.alive) {
      LOG_INFO("Node %u is ALIVE", static_cast<unsigned>(origin));
    } else {
      LOG_WARN("Node %u is MISSING", static_cast<unsigned>(origin));
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

  service_node_health_monitor(networkNowMs);
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
  init_watchdog();
  g_aim.begin();

  const uint32_t networkNowMs = g_aim.syncedMillis();
  init_node_health(networkNowMs);
  if (g_schedulerState.value != FAULT) {
    g_schedulerState.value = INIT;
  }
}

void loop(void) {
  AIM_ASSERT(g_schedulerState.value <= FAULT);

  const uint32_t networkNowMs = g_aim.syncedMillis();
  // Main scheduler order: RX, state machine, watchdog.
  service_can_rx(networkNowMs);
  run_state_machine(networkNowMs);

  kick_watchdog();
}
