#include "aim_safety.h"

static volatile const char* g_lastFaultFile = nullptr;
static volatile int g_lastFaultLine = 0;
static volatile uint32_t g_lastFaultCode = 0U;
static AimSafetyHook g_safetyHook = nullptr;

void aimSetSafetyHook(AimSafetyHook hook) {
  g_safetyHook = hook;
}

void aimGetLastSafetyFault(volatile const char*& file, int& line, uint32_t& code) {
  file = g_lastFaultFile;
  line = g_lastFaultLine;
  code = g_lastFaultCode;
}

[[noreturn]] void aimSafetyHalt(const char* file, int line, uint32_t code) {
  noInterrupts();

  g_lastFaultFile = file;
  g_lastFaultLine = line;
  g_lastFaultCode = code;

  if (g_safetyHook != nullptr) {
    g_safetyHook(file, line, code);
  }

#if defined(ARDUINO_ARCH_STM32)
  NVIC_SystemReset();
#elif defined(ARDUINO_ARCH_ESP32)
  ESP.restart();
#endif

  while (true) {
  }
}
