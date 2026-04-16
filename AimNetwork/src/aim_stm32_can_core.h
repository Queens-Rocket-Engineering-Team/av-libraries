#ifndef AIM_STM32_CAN_CORE_H
#define AIM_STM32_CAN_CORE_H

#include "aim_network.h"

#if defined(ARDUINO_ARCH_STM32)

#include <Arduino.h>
#if defined(STM32F0xx)
#include <stm32f0xx_hal.h>
#elif defined(STM32F1xx)
#include <stm32f1xx_hal.h>
#elif defined(STM32F2xx)
#include <stm32f2xx_hal.h>
#elif defined(STM32F3xx)
#include <stm32f3xx_hal.h>
#elif defined(STM32F4xx)
#include <stm32f4xx_hal.h>
#elif defined(STM32F7xx)
#include <stm32f7xx_hal.h>
#elif defined(STM32G0xx)
#include <stm32g0xx_hal.h>
#elif defined(STM32G4xx)
#include <stm32g4xx_hal.h>
#elif defined(STM32H7xx)
#include <stm32h7xx_hal.h>
#elif defined(STM32L0xx)
#include <stm32l0xx_hal.h>
#elif defined(STM32L1xx)
#include <stm32l1xx_hal.h>
#elif defined(STM32L4xx)
#include <stm32l4xx_hal.h>
#else
#error "Unsupported STM32 HAL family for AimStm32CanCore"
#endif
#include <cstddef>
#include <cstdint>

#ifndef AIM_ASSERT
#define AIM_ASSERT(cond) do { if (!(cond)) { for (;;) { } } } while (0)
#endif

class AimStm32CanCore {
public:
  struct Frame {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
  };

  AimStm32CanCore(uint8_t origin, uint32_t baud);

  bool begin();
  bool transmit(const Frame& frame);
  bool receive(Frame& frame);

private:
  static constexpr uint8_t kTxQueueSize = 16U;
  static constexpr uint8_t kRxQueueSize = 16U;
  static constexpr uint8_t kMaxRxPollIterations = 8U;

  bool enqueueTx(const Frame& frame);
  bool dequeueRx(Frame& frame);
  bool pushRx(const Frame& frame);
  bool flushTxMailboxes();
  bool pollRx();
  bool configureFilter();
  bool configureTiming();

  uint8_t _origin;
  uint32_t _baud;
  bool _initialized;

  Frame _txQueue[kTxQueueSize];
  uint8_t _txHead;
  uint8_t _txTail;
  uint8_t _txCount;

  Frame _rxQueue[kRxQueueSize];
  uint8_t _rxHead;
  uint8_t _rxTail;
  uint8_t _rxCount;

  static CAN_HandleTypeDef _hcan;
};

#endif  // ARDUINO_ARCH_STM32

#endif  // AIM_STM32_CAN_CORE_H
