#include "aim_safety.h"

static volatile const char* s_lastFaultFile = nullptr;
static volatile int s_lastFaultLine = 0;
static volatile uint32_t s_lastFaultCode = 0U;
static AimSafetyHook s_safetyHook = nullptr;

void aimSetSafetyHook(AimSafetyHook hook) {
  s_safetyHook = hook;
}

void aimGetLastSafetyFault(const char*& file, int& line, uint32_t& code) {
  file = (const char*)s_lastFaultFile;
  line = s_lastFaultLine;
  code = s_lastFaultCode;
}

[[noreturn]] void aimSafetyHalt(const char* file, int line, uint32_t code) {
  noInterrupts();

  s_lastFaultFile = file;
  s_lastFaultLine = line;
  s_lastFaultCode = code;

  if (s_safetyHook != nullptr) {
    s_safetyHook(file, line, code);
  }

#if defined(ARDUINO_ARCH_STM32)
  NVIC_SystemReset();
#elif defined(ARDUINO_ARCH_ESP32)
  ESP.restart();
#endif

  while (true) {
  }
}
