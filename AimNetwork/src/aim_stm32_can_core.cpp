#include "aim_stm32_can_core.h"

#if defined(ARDUINO_ARCH_STM32)

#include <cstring>

namespace {
static constexpr uint16_t kStdIdMask = 0x07FFU;
// In 16-bit filter mode, StdId is left-shifted by 5 before compare.
// AIM destination bits [7:5] therefore map to filter bits [12:10].
static constexpr uint16_t kDestFieldFilterShift = 10U;
static constexpr uint16_t kDestFilterMask = static_cast<uint16_t>(0x07U << kDestFieldFilterShift);
}

CAN_HandleTypeDef AimStm32CanCore::_hcan = {};

AimStm32CanCore::AimStm32CanCore(uint8_t origin, uint32_t baud)
    : _origin(origin & 0x07U),
      _baud(baud),
      _initialized(false),
      _txHead(0U),
      _txTail(0U),
      _txCount(0U),
      _rxHead(0U),
      _rxTail(0U),
      _rxCount(0U) {
  static_assert(sizeof(Frame::data) == 8U, "CAN frame data must be 8 bytes");
  AIM_ASSERT(_origin <= 0x07U);
}

bool AimStm32CanCore::configureTiming() {
  _hcan.Init.Mode = CAN_MODE_NORMAL;
  _hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  _hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  _hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  _hcan.Init.TimeTriggeredMode = DISABLE;
  _hcan.Init.AutoBusOff = ENABLE;
  _hcan.Init.AutoWakeUp = DISABLE;
  _hcan.Init.AutoRetransmission = ENABLE;
  _hcan.Init.ReceiveFifoLocked = DISABLE;
  _hcan.Init.TransmitFifoPriority = ENABLE;

  if (_baud == 1000000U) {
    _hcan.Init.Prescaler = 3U;
  } else if (_baud == 500000U) {
    _hcan.Init.Prescaler = 6U;
  } else if (_baud == 250000U) {
    _hcan.Init.Prescaler = 12U;
  } else if (_baud == 125000U) {
    _hcan.Init.Prescaler = 24U;
  } else {
    return false;
  }

  return true;
}

bool AimStm32CanCore::configureFilter() {
  CAN_FilterTypeDef filter = {};
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
#if defined(CAN2)
  filter.SlaveStartFilterBank = 14U;
#endif

  const uint16_t originId = static_cast<uint16_t>((_origin & 0x07U) << kDestFieldFilterShift);
  const uint16_t broadcastId = static_cast<uint16_t>((AIM_DEST_BROADCAST & 0x07U) << kDestFieldFilterShift);
  const uint16_t mask = kDestFilterMask;

  filter.FilterIdHigh = originId;
  filter.FilterMaskIdHigh = mask;
  filter.FilterIdLow = broadcastId;
  filter.FilterMaskIdLow = mask;

  const HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(&_hcan, &filter);
  return (status == HAL_OK);
}

bool AimStm32CanCore::begin() {
  _hcan.Instance = CAN1;
  if (!configureTiming()) {
    return false;
  }

  const HAL_StatusTypeDef initStatus = HAL_CAN_Init(&_hcan);
  if (initStatus != HAL_OK) {
    return false;
  }

  const bool filterConfigured = configureFilter();
  if (!filterConfigured) {
    return false;
  }

  const HAL_StatusTypeDef startStatus = HAL_CAN_Start(&_hcan);
  if (startStatus != HAL_OK) {
    return false;
  }

  _initialized = true;
  return true;
}

bool AimStm32CanCore::enqueueTx(const Frame& frame) {
  if (_txCount >= kTxQueueSize) {
    return false;
  }

  _txQueue[_txHead] = frame;
  _txHead = static_cast<uint8_t>((_txHead + 1U) % kTxQueueSize);
  _txCount = static_cast<uint8_t>(_txCount + 1U);
  return true;
}

bool AimStm32CanCore::pushRx(const Frame& frame) {
  if (_rxCount >= kRxQueueSize) {
    return false;
  }

  _rxQueue[_rxHead] = frame;
  _rxHead = static_cast<uint8_t>((_rxHead + 1U) % kRxQueueSize);
  _rxCount = static_cast<uint8_t>(_rxCount + 1U);
  return true;
}

bool AimStm32CanCore::dequeueRx(Frame& frame) {
  if (_rxCount == 0U) {
    return false;
  }

  frame = _rxQueue[_rxTail];
  _rxTail = static_cast<uint8_t>((_rxTail + 1U) % kRxQueueSize);
  _rxCount = static_cast<uint8_t>(_rxCount - 1U);
  return true;
}

bool AimStm32CanCore::flushTxMailboxes() {
  uint8_t iterations = 0U;
  while ((_txCount > 0U) && (iterations < kTxQueueSize)) {
    const uint32_t freeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&_hcan);
    if (freeLevel == 0U) {
      break;
    }

    const Frame& frame = _txQueue[_txTail];
    CAN_TxHeaderTypeDef header = {};
    header.StdId = frame.id & kStdIdMask;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame.dlc;
    header.TransmitGlobalTime = DISABLE;

    if (frame.dlc != 8U) {
      return false;
    }

    uint8_t payload[8] = {};
    (void)memcpy(payload, frame.data, frame.dlc);
    uint32_t mailbox = 0U;
    const HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&_hcan, &header, payload, &mailbox);
    if (status != HAL_OK) {
      return false;
    }

    _txTail = static_cast<uint8_t>((_txTail + 1U) % kTxQueueSize);
    _txCount = static_cast<uint8_t>(_txCount - 1U);
    iterations = static_cast<uint8_t>(iterations + 1U);
  }

  return true;
}

bool AimStm32CanCore::pollRx() {
  uint8_t iterations = 0U;
  while (iterations < kMaxRxPollIterations) {
    const uint32_t pending = HAL_CAN_GetRxFifoFillLevel(&_hcan, CAN_RX_FIFO0);
    if (pending == 0U) {
      break;
    }

    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {};
    const HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(&_hcan, CAN_RX_FIFO0, &header, data);
    if (status != HAL_OK) {
      return false;
    }

    if ((header.IDE == CAN_ID_STD) && (header.RTR == CAN_RTR_DATA) && (header.DLC <= 8U)) {
      Frame frame = {};
      frame.id = static_cast<uint16_t>(header.StdId & kStdIdMask);
      frame.dlc = static_cast<uint8_t>(header.DLC);
      (void)memcpy(frame.data, data, frame.dlc);
      (void)pushRx(frame);
    }

    iterations = static_cast<uint8_t>(iterations + 1U);
  }

  return true;
}

bool AimStm32CanCore::transmit(const Frame& frame) {
  if (!_initialized) {
    return false;
  }
  if (frame.dlc != 8U) {
    return false;
  }

  const bool queued = enqueueTx(frame);
  if (!queued) {
    return false;
  }

  return flushTxMailboxes();
}

bool AimStm32CanCore::receive(Frame& frame) {
  if (!_initialized) {
    return false;
  }

  const bool polled = pollRx();
  if (!polled) {
    return false;
  }

  const bool rxAvailable = dequeueRx(frame);
  (void)flushTxMailboxes();
  return rxAvailable;
}

#if defined(STM32F1xx)
extern "C" void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
  if ((hcan == nullptr) || (hcan->Instance != CAN1)) {
    return;
  }

#if defined(__HAL_RCC_CAN1_CLK_ENABLE)
  __HAL_RCC_CAN1_CLK_ENABLE();
#endif
#if defined(__HAL_RCC_AFIO_CLK_ENABLE)
  __HAL_RCC_AFIO_CLK_ENABLE();
#endif
#if defined(__HAL_RCC_GPIOB_CLK_ENABLE)
  __HAL_RCC_GPIOB_CLK_ENABLE();
#endif
#if defined(__HAL_AFIO_REMAP_CAN1_2)
  __HAL_AFIO_REMAP_CAN1_2();
#endif

  GPIO_InitTypeDef gpioInit = {};
  gpioInit.Pin = GPIO_PIN_8;
  gpioInit.Mode = GPIO_MODE_INPUT;
  gpioInit.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpioInit);

  gpioInit.Pin = GPIO_PIN_9;
  gpioInit.Mode = GPIO_MODE_AF_PP;
  gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpioInit);
}
#endif

#endif  // ARDUINO_ARCH_STM32
