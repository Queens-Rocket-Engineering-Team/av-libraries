#pragma once

#include <stdint.h>

#include "aim_catalog.h"

namespace aim {

bool isHighDataRate();
void setHighDataRate(bool active);

struct Job {
  uint32_t idlePeriodMs;
  uint32_t activePeriodMs;
  uint32_t lastMs;

  constexpr Job(uint32_t idleMs, uint32_t activeMs) : idlePeriodMs(idleMs), activePeriodMs(activeMs), lastMs(0U) {}
  constexpr explicit Job(uint32_t fixedMs) : idlePeriodMs(fixedMs), activePeriodMs(fixedMs), lastMs(0U) {}

  uint32_t periodMs() const { return isHighDataRate() ? activePeriodMs : idlePeriodMs; }

  bool due(uint32_t nowMs) {
    if ((nowMs - lastMs) < periodMs()) {
      return false;
    }

    // Soft periodic job: skip missed runs rather than catching up.
    lastMs = nowMs;
    return true;
  }
};

}  // namespace aim
