#ifndef AIM_SAFETY_H
#define AIM_SAFETY_H

#include <Arduino.h>
#include <cstdint>

typedef void (*AimSafetyHook)(const char* file, int line, uint32_t code);

void aimSetSafetyHook(AimSafetyHook hook);
void aimGetLastSafetyFault(const char*& file, int& line, uint32_t& code);

[[noreturn]] void aimSafetyHalt(const char* file, int line, uint32_t code = 0U);

#ifndef AIM_ASSERT
#define AIM_ASSERT(cond) do { if (!(cond)) { aimSafetyHalt(__FILE__, __LINE__, 0U); } } while (0)
#endif

#endif  // AIM_SAFETY_H
