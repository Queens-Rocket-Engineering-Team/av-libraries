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

  // classAcceptMask: OR of aim::classBit() values this node receives.
  bool begin(uint16_t classAcceptMask);
  bool transmit(const aim::Msg& msg);
  bool receive(aim::Msg& msg);

private:
  bool     _initialized;

#if defined(ARDUINO_ARCH_STM32)
  using CanCoreFrame = AimStm32CanCore::Frame;
  AimStm32CanCore _canCore;

#elif defined(ARDUINO_ARCH_ESP32)
  using CanCoreFrame = AimEsp32CanCore::Frame;
  AimEsp32CanCore _canCore;
#endif

  static bool packMsg(const aim::Msg& msg, CanCoreFrame& can_msg);
  static bool unpackMsg(const CanCoreFrame& can_msg, aim::Msg& msg);

  void logFailure(bool isBeginFailure, uint32_t canId = 0U) const;
};

#endif // AIM_CAN_DRIVER_H
