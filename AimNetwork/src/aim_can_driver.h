// aim_can_driver.h — Platform-aware CAN driver for AimNetwork
// Supports STM32 (HAL core in AimStm32CanCore) and ESP32 (TWAI)

#ifndef AIM_CAN_DRIVER_H
#define AIM_CAN_DRIVER_H

#include "aim_network.h"

#if defined(ARDUINO_ARCH_STM32)
  #include "aim_stm32_can_core.h"
#elif defined(ARDUINO_ARCH_ESP32)
  #include "aim_esp32_can_core.h"
#else
  #error "AimCanDriver: Unsupported platform"
#endif


class AimCanDriver {
public:

#if defined(ARDUINO_ARCH_STM32)
  AimCanDriver(uint32_t baud, CAN_TypeDef* canbus);

  void getStm32Stats(AimStm32CanCore::Stats& stats) const;
  void clearStm32Stats();
#elif defined(ARDUINO_ARCH_ESP32)
  AimCanDriver(uint32_t baud, int rxPin = -1, int txPin = -1);

  void getEsp32Stats(AimEsp32CanCore::Stats& stats) const;
  void clearEsp32Stats();
#endif

  void begin(aim::Node acceptDest);
  bool transmit(const aim::Pkt& pkt);
  bool receive(aim::Pkt& pkt);

private:
  bool     _initialized;

#if defined(ARDUINO_ARCH_STM32)
  using CanCoreFrame = AimStm32CanCore::Frame;
  AimStm32CanCore _canCore;

#elif defined(ARDUINO_ARCH_ESP32)
  using CanCoreFrame = AimEsp32CanCore::Frame;
  AimEsp32CanCore _canCore;
#endif

  static bool packAimPkt(const aim::Pkt& aim_pkt, CanCoreFrame& can_msg);
  static bool unpackAimPkt(const CanCoreFrame& can_msg, aim::Pkt& aim_pkt);

  void logFailure(bool isBeginFailure, uint16_t canId = 0U) const;
};

#endif // AIM_CAN_DRIVER_H
