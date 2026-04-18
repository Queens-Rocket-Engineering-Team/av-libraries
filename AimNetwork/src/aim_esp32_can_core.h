#ifndef AIM_ESP32_CAN_CORE_H
#define AIM_ESP32_CAN_CORE_H

#if defined(ARDUINO_ARCH_ESP32)

#include "aim_network.h"
#include "aim_safety.h"

#include <driver/twai.h>
#include <cstdint>

class AimEsp32CanCore {
public:
  struct Frame {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
  };

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

  bool setAcceptDest(uint8_t dest);

  bool begin();
  bool transmit(const Frame& frame);
  bool receive(Frame& frame);

  void getStats(Stats& stats) const;
  void clearStats();

private:
  bool validatePins() const;
  bool configureTiming(twai_timing_config_t& config) const;
  bool shouldAcceptId(uint16_t id) const;

  uint8_t _acceptDest;
  uint32_t _baud;
  int _rxPin;
  int _txPin;
  bool _initialized;
  bool _driverInstalled;
  Stats _stats;
};

#endif  // ARDUINO_ARCH_ESP32

#endif  // AIM_ESP32_CAN_CORE_H
