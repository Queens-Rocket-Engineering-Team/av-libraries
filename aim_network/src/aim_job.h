#pragma once

#include <stdint.h>

namespace aim {

struct Job {
  uint32_t periodMs;
  uint32_t lastMs;
  constexpr explicit Job(uint32_t period) : periodMs(period), lastMs(0U) {}
  bool due(uint32_t nowMs) {
    if ((nowMs - lastMs) < periodMs) { return false; }
    // Soft periodic job: skip missed runs rather than catching up.
    lastMs = nowMs;
    return true;
  }
};

}  // namespace aim
