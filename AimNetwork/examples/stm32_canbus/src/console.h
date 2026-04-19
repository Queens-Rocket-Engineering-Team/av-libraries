#ifndef CONSOLE_H
#define CONSOLE_H

#ifndef FLIGHT_BUILD

#include <Arduino.h>
#include <cstdint>

class AimNetwork;
class Logger;
class FlashTable;

enum ConsoleAction : uint8_t {
  CONSOLE_ACTION_NONE       = 0U,
  CONSOLE_ACTION_ENTER      = 1U,
  CONSOLE_ACTION_EXIT       = 2U,
  CONSOLE_ACTION_FLASH_DUMP = 3U,
  CONSOLE_ACTION_DUMP_DONE  = 4U
};

void consoleInit(Stream& serial,
                 AimNetwork& aim,
                 Logger& log,
                 FlashTable& flash);

ConsoleAction consoleCheckEntry(void);
ConsoleAction consoleService(uint8_t currentState, uint32_t networkNowMs);
ConsoleAction consoleServiceFlashDump(void);

#endif // FLIGHT_BUILD

#endif  // CONSOLE_H
