#ifndef FLIGHT_BUILD

#include "aim_console.h"

// --- State ---

enum class ConsoleState : uint8_t {
  OFF,
  ROOT,
  FLS,           // renamed: STM32 CMSIS defines FLASH as a register-address macro
  ERASE_CONFIRM,
  DUMP,
};

static Stream*               s_serial    = nullptr;
static AimFileSystem*        s_fs        = nullptr;
static AimFlightRecorder*    s_recorder  = nullptr;
static const char*           s_boardName = nullptr;
static const AimConsoleHook* s_hooks     = nullptr;
static uint8_t               s_hookCount = 0U;
static ConsoleState          s_state     = ConsoleState::OFF;

// --- Prompt helpers ---

static void printRootPrompt(void) {
  s_serial->print("\r\nDBG [q:exit b:back] f:fls");
  for (uint8_t i = 0U; i < s_hookCount; ++i) {
    s_serial->printf("  %c:%s", s_hooks[i].key, s_hooks[i].label);
  }
  s_serial->print("\r\n");
}

static void printFlashPrompt(void) {
  s_serial->printf("\r\nDBG > FLS [q:exit b:back] 1:inf 2:dmp 3:ers 4:lst 5:clr  [%uB/%uB]\r\n",
                   static_cast<unsigned>(s_fs->getUsedSize()),
                   static_cast<unsigned>(s_fs->getTotalSize()));
}

static void printPrompt(void) {
  switch (s_state) {
    case ConsoleState::ROOT:          printRootPrompt();  break;
    case ConsoleState::FLS:         printFlashPrompt(); break;
    case ConsoleState::ERASE_CONFIRM:
      s_serial->print("\r\nDBG > FLS > ERS [q:exit b:back] 1:confirm\r\n");
      break;
    default: break;
  }
}

// --- Public API ---

bool aimConsoleInit(Stream&               serial,
                    AimFileSystem&        fs,
                    AimFlightRecorder&    recorder,
                    const char*           boardName,
                    const AimConsoleHook* hooks,
                    uint8_t               hookCount) {
  s_serial    = &serial;
  s_fs        = &fs;
  s_recorder  = &recorder;
  s_boardName = boardName;
  s_hooks     = hooks;
  s_hookCount = hookCount;
  s_state     = ConsoleState::OFF;
  return true;
}

bool aimConsoleIsActive(void) {
  return s_state != ConsoleState::OFF;
}

int aimConsoleWaitRead(Stream& out) {
  (void)out;
  if (s_serial == nullptr) { return -1; }
  const uint32_t start = millis();
  while (s_serial->available() == 0) {
    if (static_cast<uint32_t>(millis() - start) > 5000U) { return -1; }
  }
  return s_serial->read();
}

void aimConsoleService(void) {
  if (s_state == ConsoleState::OFF) {
    if (s_serial->available() > 0 && s_serial->read() == 'd') {
      s_state = ConsoleState::ROOT;
      printPrompt();
    }
    return;
  }

  if (s_state == ConsoleState::DUMP) {
    // Only 'q'/'b' abort. Peek so the host's dump-control bytes ('N'/'L') stay
    // in the buffer for serviceDump to consume — read()ing them here would
    // swallow the next-block requests and stall the transfer.
    if (s_serial->available() > 0) {
      const char c = static_cast<char>(s_serial->peek());
      if (c == 'q' || c == 'b') {
        s_serial->read();
        s_recorder->stopDump();
        s_state = (c == 'q') ? ConsoleState::OFF : ConsoleState::FLS;
        printPrompt();
        return;
      }
    }
    if (!s_recorder->serviceDump(32U)) {
      s_state = ConsoleState::FLS;
      printPrompt();
    }
    return;
  }

  if (s_serial->available() == 0) { return; }
  const char c = static_cast<char>(s_serial->read());

  if (c == 'q') {
    s_state = ConsoleState::OFF;
    return;
  }

  switch (s_state) {
    case ConsoleState::ROOT:
      if (c == 'b') {
        s_state = ConsoleState::OFF;
      } else if (c == 'f') {
        s_state = ConsoleState::FLS;
        printPrompt();
      } else {
        for (uint8_t i = 0U; i < s_hookCount; ++i) {
          if (c == s_hooks[i].key) {
            s_hooks[i].handler(*s_serial);
            printRootPrompt();
            break;
          }
        }
      }
      break;

    case ConsoleState::FLS:
      if (c == 'b') {
        s_state = ConsoleState::ROOT;
        printPrompt();
      } else if (c == '1') {
        s_serial->printf("ready=%d total=%u used=%u\r\n",
                         s_fs->isReady(),
                         static_cast<unsigned>(s_fs->getTotalSize()),
                         static_cast<unsigned>(s_fs->getUsedSize()));
        printFlashPrompt();
      } else if (c == '2') {
        if (s_recorder->startDump(s_serial, s_boardName)) {
          s_state = ConsoleState::DUMP;
        } else {
          s_serial->print("[ERR] dump failed to start\r\n");
          printFlashPrompt();
        }
      } else if (c == '3') {
        s_state = ConsoleState::ERASE_CONFIRM;
        printPrompt();
      } else if (c == '4') {
        s_serial->print("Stored flight logs:\r\n");
        uint16_t count = s_recorder->listLogs(s_serial);
        s_serial->printf("%u log(s) found\r\n", count);
        printFlashPrompt();
      } else if (c == '5') {
        uint16_t cleared = s_recorder->clearLogs();
        s_serial->printf("[OK] cleared %u log(s)\r\n", cleared);
        printFlashPrompt();
      } else if (c == 'i') {
        int idx = aimConsoleWaitRead(*s_serial);
        if (idx >= 0) {
          if (s_recorder->startDumpIndex(s_serial, s_boardName, static_cast<uint16_t>(idx))) {
            s_state = ConsoleState::DUMP;
          } else {
            s_serial->print("[ERR] dump index failed\r\n");
            printFlashPrompt();
          }
        }
      }
      break;

    case ConsoleState::ERASE_CONFIRM:
      if (c == 'b') {
        s_state = ConsoleState::FLS;
        printPrompt();
      } else if (c == '1') {
        s_serial->print("formatting...\r\n");
        if (s_recorder->isDumping()) { s_recorder->stopDump(); }
        s_recorder->closeLog();
        if (s_fs->format()) {
          s_fs->begin();
          s_recorder->begin();
          s_serial->print("[OK] erased\r\n");
        } else {
          s_serial->print("[ERR] erase failed\r\n");
        }
        s_state = ConsoleState::FLS;
        printPrompt();
      }
      break;

    default: break;
  }
}

#endif // FLIGHT_BUILD
