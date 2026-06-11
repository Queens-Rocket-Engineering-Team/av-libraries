#include "node.h"
#include "console.h"

#include <IWatchdog.h>
#include <logger.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <AimFileSystem.h>
#include <AimFlightRecorder.h>

static constexpr uint32_t kWatchdogTimeoutUs  = 2000000U;
static constexpr uint8_t  kMaxRxFramesPerLoop = 8U;

struct NodeSchedulerState {
  NodeState value = INIT;
  uint32_t lastHeartbeatTxMs = 0U;
};

static NodeSchedulerState g_schedulerState = {};

static AimCanDriver g_canHw(NODE_CAN_BAUD, NODE_CAN_BUS);
static AimNetwork g_aim(&g_canHw, NODE_ORIGIN);
static SoftwareSerial g_serial(NODE_SERIAL_RX_PIN, NODE_SERIAL_TX_PIN);
static Logger g_log(g_serial, static_cast<uint8_t>(NODE_ORIGIN), LogLevel::INFO);

static SerialFlashDriver g_flashHw(NODE_FLASH_CS_PIN);
static AimFileSystem g_fs(&g_flashHw);
static AimFlightRecorder g_flightRecorder(g_fs, NODE_FLASH_TABLE_COLS, 64, 65536U);

void serviceCanRx(uint32_t networkNowMs) {
  // Handle incoming bus messages and custom packet branches here.
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aim::Pkt pkt = {};
    if (!g_aim.readPkt(pkt)) {
      break;
    }

    if (pkt.type == aim::PacketType::Time) {
      g_aim.syncTime(pkt.getMillis());
      LOG_DEBUG("Time sync received: networkNowMs=%u", networkNowMs);
    }

    // Route node-specific packets to node.cpp for processing.
    (void)nodeHandleCanPacket(pkt, networkNowMs, g_aim);
  }
}

void serviceCanTx(uint32_t schedulerNowMs, uint32_t networkNowMs) {
  // Add periodic transmit-side behavior in this service pattern.

  // TX SECTION 1: node heartbeat.
  if ((schedulerNowMs - g_schedulerState.lastHeartbeatTxMs) >= aim::kHeartbeatTxIntervalDefaultMs) {
    g_schedulerState.lastHeartbeatTxMs = schedulerNowMs;
    const uint32_t payload = static_cast<uint32_t>(g_schedulerState.value);

    aim::Pkt pkt = {};
    pkt.dest = aim::Node::Broadcast;
    pkt.type = aim::PacketType::Heartbeat;
    pkt.packData(0U, networkNowMs, payload);

    if (!g_aim.sendPkt(pkt)) {
      LOG_ERROR("Heartbeat TX failed");
    } else {
      LOG_DEBUG("Heartbeat TX ok");
    }
  }

  // TX SECTION 2: reserved for future periodic TX behavior.
  // TX SECTION 3: reserved for future periodic TX behavior.
}

void runStateMachine(uint32_t schedulerNowMs, uint32_t networkNowMs) {
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
      } else if (act == CONSOLE_ACTION_FLASH_INFO) {
        // g_flightRecorder.commandInfo(&g_serial);
      } else if (act == CONSOLE_ACTION_FLASH_DUMP) {
        if (g_flightRecorder.startDump(&g_serial)) {
          g_schedulerState.value = FLASH_DUMP;
          g_serial.print("state=");
          g_serial.println(static_cast<unsigned>(FLASH_DUMP));
        }
      } else if (act == CONSOLE_ACTION_FLASH_ERASE) {
        g_fs.format();
        // g_schedulerState.value = FLASH_ERASE;
      }
      break;
    }

    case FLASH_DUMP: {
      if (g_serial.available() > 0) {
        const int c = g_serial.read();
        if (c == 'q' || c == 'Q') {
          g_flightRecorder.stopDump();
          g_serial.println("flash dump canceled");
          g_schedulerState.value = DEBUG_CONSOLE;
          g_serial.print("state=");
          g_serial.println(static_cast<unsigned>(DEBUG_CONSOLE));
          consoleResume();
          break;
        }
      }

      if (!g_flightRecorder.serviceDump(16U)) {
        g_serial.println("flash dump done");
        g_schedulerState.value = DEBUG_CONSOLE;
        g_serial.print("state=");
        g_serial.println(static_cast<unsigned>(DEBUG_CONSOLE));
        consoleResume();
      }
      break;
    }

    case FLASH_ERASE: {
      // AimFileSystem::format() is blocking in this version
      g_schedulerState.value = DEBUG_CONSOLE;
      consoleResume();
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

  nodeUpdate(schedulerNowMs);
  serviceCanTx(schedulerNowMs, networkNowMs);
}

void setup(void) {
  AIM_ASSERT(static_cast<uint8_t>(NODE_ORIGIN) <= aim::kNodeMax);
  g_serial.begin(NODE_SERIAL_BAUD);
  g_logger = &g_log;
  LOG_INFO("Boot node origin=%u", static_cast<unsigned>(NODE_ORIGIN));
  IWatchdog.begin(kWatchdogTimeoutUs);
  LOG_INFO("Watchdog ready");

  g_aim.begin();

#ifndef FLIGHT_BUILD
  consoleInit(g_serial, g_aim, g_canHw, g_log);
#endif

  // NODE EXTENSION POINT: add one-time node setup here.
  SPI.setSCLK(NODE_FLASH_SCK_PIN);
  SPI.setMISO(NODE_FLASH_MISO_PIN);
  SPI.setMOSI(NODE_FLASH_MOSI_PIN);
  SPI.begin();

  if (g_fs.begin()) {
    LOG_INFO("Filesystem ready");
    if (g_flightRecorder.begin()) {
      LOG_INFO("Flight recorder ready");
    } else {
      LOG_WARN("Flight recorder init failed");
    }
  } else {
    LOG_WARN("Filesystem init failed");
  }

#ifndef FLIGHT_BUILD
  g_serial.println("Console ready. d=enter debug");
#endif
  g_schedulerState.lastHeartbeatTxMs = millis();
  g_schedulerState.value = OPERATIONAL;
}

void loop(void) {
  const uint32_t schedulerNowMs = millis();
  const uint32_t networkNowMs = g_aim.syncedMillis();
  // Main scheduler order: RX, state machine, watchdog.
  serviceCanRx(networkNowMs);
  runStateMachine(schedulerNowMs, networkNowMs);

  IWatchdog.reload();
}
