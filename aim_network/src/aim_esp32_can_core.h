#ifndef AIM_ESP32_CAN_CORE_H
#define AIM_ESP32_CAN_CORE_H

#if defined(ARDUINO_ARCH_ESP32)

#include "aim_network.h"
#include "aim_safety.h"

#include "aim_can_frame.h"
#include <driver/twai.h>
#include <cstdint>

class AimEsp32CanCore {
public:
  using Frame = aim::Frame;

  struct Stats {
    uint32_t txFrames;
    uint32_t rxFrames;
    uint32_t txErrors;
    uint32_t rxErrors;
    uint32_t filteredFrames;
    uint32_t beginErrors;
    uint32_t lastError;
  };

  AimEsp32CanCore(uint32_t baud, int rxPin, int txPin);

  // mask: OR of aim::classBit() values to accept. Hardware accepts broadly
  // (TWAI has one code/mask pair); unwanted classes are filtered in software.
  bool setClassMask(uint16_t mask);

  bool begin();
  bool transmit(const Frame& frame);
  bool receive(Frame& frame);

  void getStats(Stats& stats) const;
  void clearStats();

private:
  bool validatePins() const;
  bool configureTiming(twai_timing_config_t& config) const;
  bool shouldAcceptId(uint32_t id) const;

  uint16_t _classMask;
  uint32_t _baud;
  int _rxPin;
  int _txPin;
  bool _initialized;
  bool _driverInstalled;
  Stats _stats;
};

#endif  // ARDUINO_ARCH_ESP32

#endif  // AIM_ESP32_CAN_CORE_H
