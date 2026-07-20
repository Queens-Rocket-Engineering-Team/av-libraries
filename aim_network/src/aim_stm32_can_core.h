#ifndef AIM_STM32_CAN_CORE_H
#define AIM_STM32_CAN_CORE_H

#include "aim_can_frame.h"
#include "aim_catalog.h"
#include "aim_safety.h"

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

#if defined(CAN1)
#define AIM_STM32_DEFAULT_CANBUS CAN1
#else
#define AIM_STM32_DEFAULT_CANBUS nullptr
#endif

class AimStm32CanCore {
public:
  struct Stats {
    uint32_t txFrames;
    uint32_t rxFrames;
    uint32_t txQueueDrops;
    uint32_t rxQueueDrops;
    uint32_t txHalErrors;
    uint32_t rxHalErrors;
    uint32_t busOffEvents;
    uint32_t errorWarningEvents;
    uint32_t errorPassiveEvents;
    uint32_t lastHalError;
    uint32_t lastEsr;
  };

  AimStm32CanCore(uint32_t baud,
                  CAN_TypeDef* canbus = AIM_STM32_DEFAULT_CANBUS);

  // mask: OR of aim::classBit() values to accept. One hardware filter bank
  // is configured per accepted class (bxCAN 32-bit IDMASK mode).
  bool setClassMask(uint16_t mask);

  bool begin();
  bool transmit(const aim::Frame& frame);
  bool receive(aim::Frame& frame);

  void getStats(Stats& stats) const;
  void clearStats();

  // These hooks are intended for HAL interrupt callbacks.
  void onRxInterrupt();
  void onTxInterrupt();
  void onErrorInterrupt();

private:
  static constexpr uint8_t kTxQueueSize = 16U;
  static constexpr uint8_t kRxQueueSize = 16U;
  static constexpr uint8_t kMaxRxPollIterations = 8U;

  bool enqueueTx(const aim::Frame& frame);
  bool dequeueRx(aim::Frame& frame);
  bool pushRx(const aim::Frame& frame);
  bool flushTxMailboxes();
  bool pollRx();
  bool configureFilter();
  bool configureTiming();
  void updateErrorTelemetry();

  static uint32_t enterCritical();
  static void exitCritical(uint32_t primask);

  uint16_t _classMask;
  uint32_t _baud;
  CAN_TypeDef* _canbus;
  bool _initialized;

  aim::Frame _txQueue[kTxQueueSize];
  uint8_t _txHead;
  uint8_t _txTail;
  uint8_t _txCount;

  aim::Frame _rxQueue[kRxQueueSize];
  uint8_t _rxHead;
  uint8_t _rxTail;
  uint8_t _rxCount;

  Stats _stats;
  uint32_t _lastErrorFlags;

  CAN_HandleTypeDef _hcan;
};

#undef AIM_STM32_DEFAULT_CANBUS

#endif  // ARDUINO_ARCH_STM32

#endif  // AIM_STM32_CAN_CORE_H
