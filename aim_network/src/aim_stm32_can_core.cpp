#include "aim_stm32_can_core.h"
#include "aim_network.h"

#if defined(ARDUINO_ARCH_STM32)

#include <cstring>
#include <logger.h>

namespace {
// bxCAN 32-bit filter registers hold ExtId << 3 | IDE | RTR.
static constexpr uint8_t  kFilterExtIdShift = 3U;
static constexpr uint32_t kFilterIdeBit     = 0x4U;
static constexpr uint32_t kFilterRtrBit     = 0x2U;
// CAN1 owns banks 0..13; CAN2 owns 14..27 (SlaveStartFilterBank).
static constexpr uint8_t  kFilterBanksPerInstance = 14U;

struct TimingCandidate {
  uint8_t bs1;
  uint8_t bs2;
};

static constexpr uint16_t kSizeofTimingCandidate = 10;
static constexpr TimingCandidate kTimingCandidates[kSizeofTimingCandidate] = {
  {13U, 2U},
  {12U, 2U},
  {11U, 2U},
  {10U, 2U},
  {9U, 2U},
  {8U, 3U},
  {8U, 2U},
  {7U, 2U},
  {6U, 2U},
  {5U, 2U}
};

AimStm32CanCore* s_can1Owner = nullptr;
CAN_HandleTypeDef* s_can1Hcan = nullptr;
#if defined(CAN2)
AimStm32CanCore* s_can2Owner = nullptr;
CAN_HandleTypeDef* s_can2Hcan = nullptr;
#endif
#if defined(CAN3)
AimStm32CanCore* s_can3Owner = nullptr;
#endif

uint32_t toCanBs1(const uint8_t bs1) {
  switch (bs1) {
    case 1U: return CAN_BS1_1TQ;
    case 2U: return CAN_BS1_2TQ;
    case 3U: return CAN_BS1_3TQ;
    case 4U: return CAN_BS1_4TQ;
    case 5U: return CAN_BS1_5TQ;
    case 6U: return CAN_BS1_6TQ;
    case 7U: return CAN_BS1_7TQ;
    case 8U: return CAN_BS1_8TQ;
    case 9U: return CAN_BS1_9TQ;
    case 10U: return CAN_BS1_10TQ;
    case 11U: return CAN_BS1_11TQ;
    case 12U: return CAN_BS1_12TQ;
    case 13U: return CAN_BS1_13TQ;
    case 14U: return CAN_BS1_14TQ;
    case 15U: return CAN_BS1_15TQ;
    case 16U: return CAN_BS1_16TQ;
    default: return 0U;
  }
}

uint32_t toCanBs2(const uint8_t bs2) {
  switch (bs2) {
    case 1U: return CAN_BS2_1TQ;
    case 2U: return CAN_BS2_2TQ;
    case 3U: return CAN_BS2_3TQ;
    case 4U: return CAN_BS2_4TQ;
    case 5U: return CAN_BS2_5TQ;
    case 6U: return CAN_BS2_6TQ;
    case 7U: return CAN_BS2_7TQ;
    case 8U: return CAN_BS2_8TQ;
    default: return 0U;
  }
}

AimStm32CanCore* ownerFromInstance(CAN_TypeDef* instance) {
#if defined(CAN1)
  if (instance == CAN1) {
    return s_can1Owner;
  }
#endif
#if defined(CAN2)
  if (instance == CAN2) {
    return s_can2Owner;
  }
#endif
#if defined(CAN3)
  if (instance == CAN3) {
    return s_can3Owner;
  }
#endif
  return nullptr;
}

#if defined(STM32F1xx)
bool configureF1CanPins(CAN_TypeDef* canbus) {
  GPIO_InitTypeDef gpioInit = {};

  gpioInit.Mode = GPIO_MODE_INPUT;
  gpioInit.Pull = GPIO_PULLUP;

  if (canbus == CAN1) {
#if defined(__HAL_RCC_GPIOB_CLK_ENABLE)
    __HAL_RCC_GPIOB_CLK_ENABLE();
#endif
#if defined(__HAL_AFIO_REMAP_CAN1_2)
    __HAL_AFIO_REMAP_CAN1_2();
#endif
    gpioInit.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    gpioInit.Pin = GPIO_PIN_9;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpioInit);
    return true;
  }

#if defined(CAN2)
  if (canbus == CAN2) {
#if defined(__HAL_RCC_GPIOB_CLK_ENABLE)
    __HAL_RCC_GPIOB_CLK_ENABLE();
#endif
#if defined(__HAL_AFIO_REMAP_CAN2_DISABLE)
    __HAL_AFIO_REMAP_CAN2_DISABLE();
#endif

    gpioInit.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOB, &gpioInit);

    gpioInit.Pin = GPIO_PIN_13;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpioInit);
    return true;
  }
#endif

  return false;
}
#endif
}

AimStm32CanCore::AimStm32CanCore(uint32_t baud, CAN_TypeDef* canbus)
  : _classMask(0U),
      _baud(baud),
      _canbus(canbus),
      _initialized(false),
      _txHead(0U),
      _txTail(0U),
      _txCount(0U),
      _rxHead(0U),
      _rxTail(0U),
      _rxCount(0U),
      _stats{},
      _lastErrorFlags(0U),
      _hcan{} {
  static_assert(sizeof(aim::Frame::data) == 8U, "CAN frame data must be 8 bytes");
}

bool AimStm32CanCore::setClassMask(uint16_t mask) {
  if (_initialized) {
    return false;
  }

  if (mask == 0U) {
    return false;
  }

  // One hardware filter bank per accepted class.
  uint8_t banksNeeded = 0U;
  for (uint16_t bits = mask; bits != 0U; bits = static_cast<uint16_t>(bits & (bits - 1U))) {
    banksNeeded = static_cast<uint8_t>(banksNeeded + 1U);
  }
  if (banksNeeded > kFilterBanksPerInstance) {
    return false;
  }

  _classMask = mask;
  return true;
}

uint32_t AimStm32CanCore::enterCritical() {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void AimStm32CanCore::exitCritical(const uint32_t primask) {
  if ((primask & 0x1U) == 0U) {
    __enable_irq();
  }
}

void AimStm32CanCore::getStats(Stats& stats) const {
  const uint32_t primask = AimStm32CanCore::enterCritical();
  stats = _stats;
  AimStm32CanCore::exitCritical(primask);
}

void AimStm32CanCore::clearStats() {
  const uint32_t primask = AimStm32CanCore::enterCritical();
  _stats = {};
  _lastErrorFlags = 0U;
  AimStm32CanCore::exitCritical(primask);
}



bool AimStm32CanCore::configureTiming() {
  if (_canbus == nullptr) {
    LOG_ERROR("AimStm32CanCore configureTiming failed: CAN instance is null");
    return false;
  }

  _hcan.Init.Mode = CAN_MODE_NORMAL;
  _hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  _hcan.Init.TimeTriggeredMode = DISABLE;
  _hcan.Init.AutoBusOff = ENABLE;
  _hcan.Init.AutoWakeUp = DISABLE;
  _hcan.Init.AutoRetransmission = ENABLE;
  _hcan.Init.ReceiveFifoLocked = DISABLE;
  _hcan.Init.TransmitFifoPriority = ENABLE;

  const uint32_t pclk1Hz = HAL_RCC_GetPCLK1Freq();
  if ((pclk1Hz == 0U) || (_baud == 0U)) {
    LOG_ERROR(
        "AimStm32CanCore configureTiming failed: invalid clocks pclk1=%lu baud=%lu",
        static_cast<unsigned long>(pclk1Hz),
        static_cast<unsigned long>(_baud));
    return false;
  }

  for (size_t i = 0U; i < kSizeofTimingCandidate; i++) {
    const uint8_t bs1 = kTimingCandidates[i].bs1;
    const uint8_t bs2 = kTimingCandidates[i].bs2;
    const uint32_t totalTq = static_cast<uint32_t>(1U + bs1 + bs2);
    const uint32_t denominator = _baud * totalTq;

    if ((denominator == 0U) || ((pclk1Hz % denominator) != 0U)) {
      continue;
    }

    const uint32_t prescaler = pclk1Hz / denominator;
    if ((prescaler == 0U) || (prescaler > 1024U)) {
      continue;
    }

    const uint32_t bs1Reg = toCanBs1(bs1);
    const uint32_t bs2Reg = toCanBs2(bs2);
    if ((bs1Reg == 0U) || (bs2Reg == 0U)) {
      continue;
    }

    _hcan.Init.TimeSeg1 = bs1Reg;
    _hcan.Init.TimeSeg2 = bs2Reg;
    _hcan.Init.Prescaler = prescaler;
    return true;
  }

  LOG_ERROR(
      "AimStm32CanCore configureTiming failed: no timing candidate pclk1=%lu baud=%lu",
      static_cast<unsigned long>(pclk1Hz),
      static_cast<unsigned long>(_baud));
  return false;
}

bool AimStm32CanCore::configureFilter() {
  // One 32-bit IDMASK bank per accepted class: match the class field
  // (ID bits 26:23) and require an extended data frame. Reserved bits,
  // subject, and source are passed through (receivers ignore reserved).
  uint8_t bank = 0U;
#if defined(CAN2)
  if (_canbus == CAN2) {
    bank = kFilterBanksPerInstance;
  }
#endif

  for (uint8_t cls = 0U; cls < 16U; cls++) {
    if ((_classMask & (1U << cls)) == 0U) {
      continue;
    }

    const uint32_t id32 =
        (static_cast<uint32_t>(cls) << aim::kIdClassShift) << kFilterExtIdShift | kFilterIdeBit;
    const uint32_t mask32 =
        (static_cast<uint32_t>(0xFU) << aim::kIdClassShift) << kFilterExtIdShift |
        kFilterIdeBit | kFilterRtrBit;

    CAN_FilterTypeDef filter = {};
    filter.FilterBank = bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
#if defined(CAN2)
    filter.SlaveStartFilterBank = kFilterBanksPerInstance;
#endif
    filter.FilterIdHigh = static_cast<uint16_t>(id32 >> 16U);
    filter.FilterIdLow = static_cast<uint16_t>(id32 & 0xFFFFU);
    filter.FilterMaskIdHigh = static_cast<uint16_t>(mask32 >> 16U);
    filter.FilterMaskIdLow = static_cast<uint16_t>(mask32 & 0xFFFFU);

    if (HAL_CAN_ConfigFilter(&_hcan, &filter) != HAL_OK) {
      return false;
    }

    bank = static_cast<uint8_t>(bank + 1U);
  }

  return true;
}

bool AimStm32CanCore::begin() {
  AIM_ASSERT(_classMask != 0U);
  AIM_ASSERT(_canbus != nullptr);

  if (_initialized) {
    return true;
  }

  bool validBus = false;
  if (_canbus != nullptr) {
#if defined(CAN1)
    if (_canbus == CAN1) validBus = true;
#endif
#if defined(CAN2)
    if (_canbus == CAN2) validBus = true;
#endif
#if defined(CAN3)
    if (_canbus == CAN3) validBus = true;
#endif
  }

  if (!validBus) {
    LOG_ERROR("AimStm32CanCore begin failed: invalid CAN instance");
    return false;
  }

#if defined(CAN1)
  if (_canbus == CAN1) {
    s_can1Owner = this;
    s_can1Hcan = &_hcan;
  }
#endif
#if defined(CAN2)
  if (_canbus == CAN2) {
    s_can2Owner = this;
    s_can2Hcan = &_hcan;
  }
#endif
#if defined(CAN3)
  if (_canbus == CAN3) {
    s_can3Owner = this;
  }
#endif

  _hcan = {};
  _hcan.Instance = _canbus;
  if (!configureTiming()) {
    LOG_ERROR("AimStm32CanCore begin failed: timing configuration failed");
    return false;
  }

  const HAL_StatusTypeDef initStatus = HAL_CAN_Init(&_hcan);
  if (initStatus != HAL_OK) {
    LOG_ERROR(
        "AimStm32CanCore begin failed: HAL_CAN_Init status=%d hal=0x%08lX",
        static_cast<int>(initStatus),
        static_cast<unsigned long>(HAL_CAN_GetError(&_hcan)));
    return false;
  }

  const bool filterConfigured = configureFilter();
  if (!filterConfigured) {
    LOG_ERROR(
        "AimStm32CanCore begin failed: filter configuration failed hal=0x%08lX",
        static_cast<unsigned long>(HAL_CAN_GetError(&_hcan)));
    return false;
  }

  const HAL_StatusTypeDef startStatus = HAL_CAN_Start(&_hcan);
  if (startStatus != HAL_OK) {
    LOG_ERROR(
        "AimStm32CanCore begin failed: HAL_CAN_Start status=%d hal=0x%08lX",
        static_cast<int>(startStatus),
        static_cast<unsigned long>(HAL_CAN_GetError(&_hcan)));
    return false;
  }

  uint32_t notifications = CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY;
#if defined(CAN_IT_BUSOFF)
  notifications |= CAN_IT_BUSOFF;
#endif
#if defined(CAN_IT_ERROR_WARNING)
  notifications |= CAN_IT_ERROR_WARNING;
#endif
#if defined(CAN_IT_ERROR_PASSIVE)
  notifications |= CAN_IT_ERROR_PASSIVE;
#endif

  const HAL_StatusTypeDef notifStatus = HAL_CAN_ActivateNotification(&_hcan, notifications);
  if (notifStatus != HAL_OK) {
    LOG_ERROR(
        "AimStm32CanCore begin failed: notification activation status=%d hal=0x%08lX",
        static_cast<int>(notifStatus),
        static_cast<unsigned long>(HAL_CAN_GetError(&_hcan)));
    return false;
  }

  _initialized = true;
  return true;
}

bool AimStm32CanCore::enqueueTx(const aim::Frame& frame) {
  const uint32_t primask = enterCritical();
  if (_txCount >= kTxQueueSize) {
    _stats.txQueueDrops = _stats.txQueueDrops + 1U;
    exitCritical(primask);
    return false;
  }

  _txQueue[_txHead] = frame;
  _txHead = static_cast<uint8_t>((_txHead + 1U) % kTxQueueSize);
  _txCount = static_cast<uint8_t>(_txCount + 1U);
  exitCritical(primask);
  return true;
}

bool AimStm32CanCore::pushRx(const aim::Frame& frame) {
  const uint32_t primask = enterCritical();
  if (_rxCount >= kRxQueueSize) {
    _stats.rxQueueDrops = _stats.rxQueueDrops + 1U;
    exitCritical(primask);
    return false;
  }

  _rxQueue[_rxHead] = frame;
  _rxHead = static_cast<uint8_t>((_rxHead + 1U) % kRxQueueSize);
  _rxCount = static_cast<uint8_t>(_rxCount + 1U);
  _stats.rxFrames = _stats.rxFrames + 1U;
  exitCritical(primask);
  return true;
}

bool AimStm32CanCore::dequeueRx(aim::Frame& frame) {
  const uint32_t primask = enterCritical();
  if (_rxCount == 0U) {
    exitCritical(primask);
    return false;
  }

  frame = _rxQueue[_rxTail];
  _rxTail = static_cast<uint8_t>((_rxTail + 1U) % kRxQueueSize);
  _rxCount = static_cast<uint8_t>(_rxCount - 1U);
  exitCritical(primask);
  return true;
}

void AimStm32CanCore::updateErrorTelemetry() {
  if (_hcan.Instance != nullptr) {
    const uint32_t primask = enterCritical();
    _stats.lastEsr = _hcan.Instance->ESR;
    exitCritical(primask);
  }

  const uint32_t halError = HAL_CAN_GetError(&_hcan);
  if (halError == HAL_CAN_ERROR_NONE) {
    _lastErrorFlags = 0U;

    const uint32_t primask = enterCritical();
    _stats.lastHalError = 0U;
    exitCritical(primask);
    return;
  }

  const uint32_t newFlags = halError & (~_lastErrorFlags);
  _lastErrorFlags = halError;

  const uint32_t primask = enterCritical();
  _stats.lastHalError = halError;
#if defined(HAL_CAN_ERROR_BOF)
  if ((newFlags & HAL_CAN_ERROR_BOF) != 0U) {
    _stats.busOffEvents = _stats.busOffEvents + 1U;
  }
#endif
#if defined(HAL_CAN_ERROR_EWG)
  if ((newFlags & HAL_CAN_ERROR_EWG) != 0U) {
    _stats.errorWarningEvents = _stats.errorWarningEvents + 1U;
  }
#endif
#if defined(HAL_CAN_ERROR_EPV)
  if ((newFlags & HAL_CAN_ERROR_EPV) != 0U) {
    _stats.errorPassiveEvents = _stats.errorPassiveEvents + 1U;
  }
#endif
  exitCritical(primask);
}

bool AimStm32CanCore::flushTxMailboxes() {
  uint8_t iterations = 0U;
  while (iterations < kTxQueueSize) {
    const uint32_t primask = enterCritical();

    const uint32_t freeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&_hcan);
    if ((freeLevel == 0U) || (_txCount == 0U)) {
      exitCritical(primask);
      break;
    }

    aim::Frame frame = _txQueue[_txTail];
    _txTail = static_cast<uint8_t>((_txTail + 1U) % kTxQueueSize);
    _txCount = static_cast<uint8_t>(_txCount - 1U);

    if (frame.dlc != 8U) {
      _stats.txHalErrors = _stats.txHalErrors + 1U;
      exitCritical(primask);
      return false;
    }

    CAN_TxHeaderTypeDef header = {};
    header.StdId = 0U;
    header.ExtId = frame.id & aim::kExtIdMask;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame.dlc;
    header.TransmitGlobalTime = DISABLE;

    uint8_t payload[8] = {};
    (void)memcpy(payload, frame.data, frame.dlc);
    uint32_t mailbox = 0U;
    const HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&_hcan, &header, payload, &mailbox);

    if (status != HAL_OK) {
      _stats.txHalErrors = _stats.txHalErrors + 1U;
      exitCritical(primask);
      updateErrorTelemetry();
      return false;
    }

    _stats.txFrames = _stats.txFrames + 1U;
    exitCritical(primask);

    iterations = static_cast<uint8_t>(iterations + 1U);
  }

  updateErrorTelemetry();
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
      const uint32_t primask = enterCritical();
      _stats.rxHalErrors = _stats.rxHalErrors + 1U;
      exitCritical(primask);
      updateErrorTelemetry();
      return false;
    }

    if ((header.IDE == CAN_ID_EXT) && (header.RTR == CAN_RTR_DATA) && (header.DLC <= 8U)) {
      aim::Frame frame = {};
      frame.id = header.ExtId & aim::kExtIdMask;
      frame.dlc = static_cast<uint8_t>(header.DLC);
      (void)memcpy(frame.data, data, frame.dlc);
      (void)pushRx(frame);
    }

    iterations = static_cast<uint8_t>(iterations + 1U);
  }

  updateErrorTelemetry();
  return true;
}

bool AimStm32CanCore::transmit(const aim::Frame& frame) {
  AIM_ASSERT(frame.dlc <= 8U);

  if (!_initialized) {
    return false;
  }
  if (frame.dlc != 8U) {
    return false;
  }

  (void)flushTxMailboxes();

  const bool queued = enqueueTx(frame);
  if (!queued) {
    return false;
  }

  return flushTxMailboxes();
}

bool AimStm32CanCore::receive(aim::Frame& frame) {
  AIM_ASSERT(_canbus != nullptr);

  if (!_initialized) {
    return false;
  }

  (void)pollRx();

  const bool rxAvailable = dequeueRx(frame);
  (void)flushTxMailboxes();
  return rxAvailable;
}

void AimStm32CanCore::onRxInterrupt() {
  (void)pollRx();
}

void AimStm32CanCore::onTxInterrupt() {
  (void)flushTxMailboxes();
}

void AimStm32CanCore::onErrorInterrupt() {
  updateErrorTelemetry();
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (hcan == nullptr) {
    return;
  }

  AimStm32CanCore* const owner = ownerFromInstance(hcan->Instance);
  if (owner != nullptr) {
    owner->onRxInterrupt();
  }
}

extern "C" void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* hcan) {
  if (hcan == nullptr) {
    return;
  }

  AimStm32CanCore* const owner = ownerFromInstance(hcan->Instance);
  if (owner != nullptr) {
    owner->onTxInterrupt();
  }
}

extern "C" void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* hcan) {
  if (hcan == nullptr) {
    return;
  }

  AimStm32CanCore* const owner = ownerFromInstance(hcan->Instance);
  if (owner != nullptr) {
    owner->onTxInterrupt();
  }
}

extern "C" void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* hcan) {
  if (hcan == nullptr) {
    return;
  }

  AimStm32CanCore* const owner = ownerFromInstance(hcan->Instance);
  if (owner != nullptr) {
    owner->onTxInterrupt();
  }
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan) {
  if (hcan == nullptr) {
    return;
  }

  AimStm32CanCore* const owner = ownerFromInstance(hcan->Instance);
  if (owner != nullptr) {
    owner->onErrorInterrupt();
  }
}

#if defined(STM32F1xx)

#if defined(CAN1)
extern "C" void USB_HP_CAN1_TX_IRQHandler(void) {
  if (s_can1Hcan != nullptr) {
    HAL_CAN_IRQHandler(s_can1Hcan);
  }
}

extern "C" void USB_LP_CAN1_RX0_IRQHandler(void) {
  if (s_can1Hcan != nullptr) {
    HAL_CAN_IRQHandler(s_can1Hcan);
  }
}

extern "C" void CAN1_SCE_IRQHandler(void) {
  if (s_can1Hcan != nullptr) {
    HAL_CAN_IRQHandler(s_can1Hcan);
  }
}
#endif

#if defined(CAN2)
extern "C" void CAN2_TX_IRQHandler(void) {
  if (s_can2Hcan != nullptr) {
    HAL_CAN_IRQHandler(s_can2Hcan);
  }
}

extern "C" void CAN2_RX0_IRQHandler(void) {
  if (s_can2Hcan != nullptr) {
    HAL_CAN_IRQHandler(s_can2Hcan);
  }
}

extern "C" void CAN2_SCE_IRQHandler(void) {
  if (s_can2Hcan != nullptr) {
    HAL_CAN_IRQHandler(s_can2Hcan);
  }
}
#endif

extern "C" void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
  if ((hcan == nullptr) || (hcan->Instance == nullptr)) {
    return;
  }

#if defined(__HAL_RCC_CAN1_CLK_ENABLE)
  __HAL_RCC_CAN1_CLK_ENABLE();
#endif
#if defined(CAN2) && defined(__HAL_RCC_CAN2_CLK_ENABLE)
  if (hcan->Instance == CAN2) {
    __HAL_RCC_CAN2_CLK_ENABLE();
  }
#endif
#if defined(__HAL_RCC_AFIO_CLK_ENABLE)
  __HAL_RCC_AFIO_CLK_ENABLE();
#endif

  const bool configured = configureF1CanPins(hcan->Instance);
  AIM_ASSERT(configured);

#if defined(CAN1)
  if (hcan->Instance == CAN1) {
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  }
#endif
#if defined(CAN2)
  if (hcan->Instance == CAN2) {
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_SCE_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(CAN2_SCE_IRQn);
  }
#endif
}
#endif

#endif  // ARDUINO_ARCH_STM32
