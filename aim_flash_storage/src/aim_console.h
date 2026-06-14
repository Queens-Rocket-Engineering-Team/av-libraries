#ifndef AIM_CONSOLE_H
#define AIM_CONSOLE_H

#ifndef FLIGHT_BUILD

#include <Arduino.h>
#include "aim_file_system.h"
#include "aim_flight_recorder.h"

/**
 * @brief Application-supplied hook for a top-level console menu item.
 *
 * handler is called once when the key is pressed from the root menu. It
 * may write output to `out`, then returns. The library reprints the root
 * prompt automatically afterwards.
 *
 * All fields must point to static storage — the console does not copy them.
 */
struct AimConsoleHook {
  char        key;
  const char* label;
  void (*handler)(Stream& out);
};

/**
 * @brief Initialise the console.
 *
 * @param serial    Output/input stream (e.g. Serial).
 * @param fs        Filesystem instance for flash info and format.
 * @param recorder  Flight recorder instance for dump and erase.
 * @param boardName Board identity string shown in flash menu header.
 * @param hooks     Static array of application-supplied menu items (may be nullptr).
 * @param hookCount Number of entries in hooks[].
 */
bool aimConsoleInit(Stream&               serial,
                    AimFileSystem&        fs,
                    AimFlightRecorder&    recorder,
                    const char*           boardName,
                    const AimConsoleHook* hooks,
                    uint8_t               hookCount);

/**
 * @brief Returns true while the console is active (any menu visible).
 *
 * Use to gate telemetry writes:
 *   if (!aimConsoleIsActive()) { serviceLog(); }
 */
bool aimConsoleIsActive(void);

/**
 * @brief Drive the console state machine. Call every loop tick.
 */
void aimConsoleService(void);

#endif // FLIGHT_BUILD

#endif // AIM_CONSOLE_H
