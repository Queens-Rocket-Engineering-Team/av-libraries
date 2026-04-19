#include "node.h"

#include <IWatchdog.h>
#include <logger.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <flash_table.h>

static constexpr uint32_t kHeartbeatTxIntervalMs = AIM_HEARTBEAT_TX_INTERVAL_DEFAULT_MS;
static constexpr uint32_t kWatchdogTimeoutUs = 2000000U;
static constexpr uint8_t kMaxRxFramesPerLoop = 8U;
static constexpr uint16_t kFlashDumpLineBytes = 16U;
static constexpr uint32_t kFlashDumpMaxBytes = 512U;

struct NodeSchedulerState {
  NodeState value = INIT;
  uint32_t lastHeartbeatTxMs = 0U;
};

enum ConsoleMenu : uint8_t {
  CONSOLE_MENU_ROOT = 0U,
  CONSOLE_MENU_LOG_LEVEL = 1U,
  CONSOLE_MENU_FLASH = 2U
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
static ConsoleMenu g_consoleMenu = CONSOLE_MENU_ROOT;

void print_scheduler_state(void) {
  g_serial.print("state=");
  g_serial.println(static_cast<unsigned>(g_schedulerState.value));
}

void set_console_menu(ConsoleMenu menu) {
  g_consoleMenu = menu;
  switch (menu) {
    case CONSOLE_MENU_ROOT:
      g_serial.println("DEBUG: q exit | b back");
      g_serial.println("1 build | 2 version | 3 log | 4 status | 5 flash");
      break;
    case CONSOLE_MENU_LOG_LEVEL:
      g_serial.println("LOG: q exit | b back");
      g_serial.println("1 DEBUG | 2 INFO | 3 WARN | 4 ERROR");
      break;
    case CONSOLE_MENU_FLASH:
      g_serial.println("FLASH: q exit | b back");
      g_serial.println("1 info | 2 dump | 3 erase");
      break;
    default:
      g_consoleMenu = CONSOLE_MENU_ROOT;
      g_serial.println("DEBUG: q exit | b back");
      g_serial.println("1 build | 2 version | 3 log | 4 status | 5 flash");
      break;
  }
}

void print_console_status(void) {
  const uint32_t networkNowMs = g_aim.syncedMillis();
  g_serial.print("name=");
  g_serial.print(NODE_NAME);
  g_serial.print(" ");
  g_serial.print("state=");
  g_serial.print(static_cast<unsigned>(g_schedulerState.value));
  g_serial.print(" log=");
  g_serial.print(static_cast<unsigned>(g_log.level()));
  g_serial.print(" nowMs=");
  g_serial.print(static_cast<unsigned long>(networkNowMs));
  g_serial.print(" offset=");
  g_serial.println(static_cast<long>(g_aim.getTimeOffset()));
}

void finish_flash_dump(const char* status) {
  g_schedulerState.value = DEBUG_CONSOLE;
  g_serial.println(status);
  print_scheduler_state();
  set_console_menu(CONSOLE_MENU_FLASH);
}

int read_console_char(void) {
#ifndef FLIGHT_BUILD
  if (g_serial.available() <= 0) {
    return -1;
  }

  const int rxByte = g_serial.read();
  if (rxByte < 0) {
    return -1;
  }

  char c = static_cast<char>(rxByte);
  if ((c >= 'A') && (c <= 'Z')) {
    c = static_cast<char>(c + ('a' - 'A'));
  }

  if ((c == '\n') || (c == '\r') || (c == ' ') || (c == '\t')) {
    return -1;
  }

  return static_cast<int>(c);
#else
  return -1;
#endif
}

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

  const int c = read_console_char();

  switch (g_schedulerState.value) {
    case INIT:
      board_init();
      g_schedulerState.lastHeartbeatTxMs = millis();
      g_schedulerState.value = OPERATIONAL;
      LOG_INFO("State transition INIT -> OPERATIONAL");
      return;

    case FLASH_DUMP:
      if (c == 'q') {
        g_flashTable.cancelDump();
        finish_flash_dump("flash dump canceled");
        break;
      }

      {
      const FlashTableServiceResult dumpResult = g_flashTable.serviceDump(&g_serial, kFlashDumpLineBytes);
      if (dumpResult == FLASHTABLE_SERVICE_DONE) {
        finish_flash_dump("flash dump done");
      } else if (dumpResult == FLASHTABLE_SERVICE_ABORTED) {
        finish_flash_dump("flash dump aborted");
      } else if (dumpResult == FLASHTABLE_SERVICE_ERROR) {
        finish_flash_dump("flash dump error");
      } else if (dumpResult == FLASHTABLE_SERVICE_IDLE) {
        finish_flash_dump("flash dump idle");
      }
      break;
    }

    case OPERATIONAL:
      if (c == 'd') {
        g_schedulerState.value = DEBUG_CONSOLE;
        print_scheduler_state();
        set_console_menu(CONSOLE_MENU_ROOT);
      }
      break;

    case DEBUG_CONSOLE:
      {
      const bool eraseActive = (g_flashTable.state() == FLASHTABLE_STATE_ERASE);
      if (eraseActive) {
        if (g_consoleMenu != CONSOLE_MENU_FLASH) {
          set_console_menu(CONSOLE_MENU_FLASH);
        }
        g_flashTable.serviceErase();
      }

      if (c < 0) {
        break;
      }

      if (!eraseActive && (c == 'q')) {
        g_schedulerState.value = OPERATIONAL;
        g_consoleMenu = CONSOLE_MENU_ROOT;
        print_scheduler_state();
        break;
      }

      switch (g_consoleMenu) {
        case CONSOLE_MENU_ROOT:
          if (c == 'b') {
            set_console_menu(CONSOLE_MENU_ROOT);
            break;
          }

          switch (c) {
            case '1':
              g_serial.print("build ");
              g_serial.print(__DATE__);
              g_serial.print(" ");
              g_serial.println(__TIME__);
              break;
            case '2':
              g_serial.print("AimNetwork ");
              g_serial.println(AIM_NETWORK_VERSION_STRING);
              break;
            case '3':
              set_console_menu(CONSOLE_MENU_LOG_LEVEL);
              break;
            case '4':
              print_console_status();
              break;
            case '5':
              set_console_menu(CONSOLE_MENU_FLASH);
              break;
            default:
              break;
          }
          break;

        case CONSOLE_MENU_LOG_LEVEL:
          if (c == 'b') {
            set_console_menu(CONSOLE_MENU_ROOT);
            break;
          }

          if ((c >= '1') && (c <= '4')) {
            const LogLevel level = static_cast<LogLevel>(static_cast<uint8_t>(c - '1'));
            g_log.setLevel(level);
            g_serial.print("log=");
            g_serial.println(static_cast<unsigned>(level));
          }
          break;

        case CONSOLE_MENU_FLASH:
          if (eraseActive) {
            break;
          }

          if (c == 'b') {
            set_console_menu(CONSOLE_MENU_ROOT);
            break;
          }

          switch (c) {
            case '1':
              g_flashTable.commandInfo(&g_serial);
              break;
            case '2':
              if (g_flashTable.commandDump(&g_serial, kFlashDumpMaxBytes, nullptr, nullptr)) {
                g_schedulerState.value = FLASH_DUMP;
                print_scheduler_state();
              }
              break;
            case '3':
              g_flashTable.commandErase(&g_serial);
              break;
            default:
              break;
          }
          break;

        default:
          set_console_menu(CONSOLE_MENU_ROOT);
          break;
      }
      break;
      }

    case SAFE_MODE:
    case LOW_POWER:
    case FAULT:
      break;

    default:
      g_schedulerState.value = FAULT;
      break;
  }
  board_update();
  service_can_tx(networkNowMs);
}

void board_init(void) {
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
}

void board_update(void) {
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
  g_serial.println("Console ready. d=enter debug");

  g_schedulerState.value = INIT;
}

void loop(void) {
  const uint32_t networkNowMs = g_aim.syncedMillis();
  // Main scheduler order: RX, state machine, watchdog.
  service_can_rx(networkNowMs);
  run_state_machine(networkNowMs);

  IWatchdog.reload();
}
