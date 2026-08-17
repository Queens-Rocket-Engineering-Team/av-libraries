#ifndef AIM_ESP32_CAN_CORE_H
#define AIM_ESP32_CAN_CORE_H

#if defined(ARDUINO_ARCH_ESP32)

#include "aim_can_frame.h"

#include <driver/twai.h>
#include <cstdint>

class AimEsp32CanCore {
public:
  struct Stats {
    uint32_t txFrames{0U};
    uint32_t rxFrames{0U};
    uint32_t txErrors{0U};
    uint32_t rxErrors{0U};
    uint32_t filteredFrames{0U};
    uint32_t beginErrors{0U};
    uint32_t busOffRecoveries{0U};
    uint32_t lastError{0U};
    uint32_t lastBusErrCount{0U};
    uint32_t lastTec{0U};
    uint32_t lastRec{0U};
  };

  AimEsp32CanCore(uint32_t baud, int rxPin, int txPin);

  // mask: OR of aim::classBit() values to accept. Hardware accepts broadly
  // (TWAI has one code/mask pair); unwanted classes are filtered in software.
  bool setClassMask(uint16_t mask);

  bool begin();
  bool transmit(const aim::Frame& frame);
  bool receive(aim::Frame& frame);

  void getStats(Stats& stats) const;
  void clearStats();

private:
  static constexpr uint32_t kRecoveryCooldownMs = 500U;

  bool validatePins() const;
  bool configureTiming(twai_timing_config_t& config) const;
  bool shouldAcceptId(uint32_t id) const;
  bool recoverBusOff();
  void captureTwaiCounters();

  uint16_t _classMask;
  uint32_t _baud;
  int _rxPin;
  int _txPin;
  bool _initialized;
  bool _driverInstalled;
  uint32_t _lastRecoveryMs;
  Stats _stats;
};

#endif  // ARDUINO_ARCH_ESP32

#endif  // AIM_ESP32_CAN_CORE_H
