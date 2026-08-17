#include "aim_esp32_can_core.h"
#include "aim_network.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <cstring>
#include <logger.h>

AimEsp32CanCore::AimEsp32CanCore(uint32_t baud, int rxPin, int txPin)
    : _classMask(0U),
      _baud(baud),
      _rxPin(rxPin),
      _txPin(txPin),
      _initialized(false),
      _driverInstalled(false),
      _lastRecoveryMs(0U),
      _stats{} {
}

bool AimEsp32CanCore::setClassMask(uint16_t mask) {
  if (_initialized) {
    return false;
  }

  if (mask == 0U) {
    return false;
  }

  _classMask = mask;
  return true;
}

bool AimEsp32CanCore::validatePins() const {
  return (_rxPin >= 0) && (_txPin >= 0) && (_rxPin != _txPin);
}

bool AimEsp32CanCore::configureTiming(twai_timing_config_t& config) const {
  if (_baud == 1000000U) {
    config = TWAI_TIMING_CONFIG_1MBITS();
    return true;
  }
  if (_baud == 500000U) {
    config = TWAI_TIMING_CONFIG_500KBITS();
    return true;
  }
  if (_baud == 250000U) {
    config = TWAI_TIMING_CONFIG_250KBITS();
    return true;
  }
  if (_baud == 125000U) {
    config = TWAI_TIMING_CONFIG_125KBITS();
    return true;
  }

  return false;
}

bool AimEsp32CanCore::begin() {
  AIM_ASSERT(_classMask != 0U);

  _initialized = false;

  if (_driverInstalled) {
    const esp_err_t stopStatus = twai_stop();
    if ((stopStatus != ESP_OK) && (stopStatus != ESP_ERR_INVALID_STATE)) {
      LOG_WARN(
          "AimEsp32CanCore begin: twai_stop status=%d",
          static_cast<int>(stopStatus));
    }

    const esp_err_t uninstallStatus = twai_driver_uninstall();
    if ((uninstallStatus != ESP_OK) && (uninstallStatus != ESP_ERR_INVALID_STATE)) {
      LOG_WARN(
          "AimEsp32CanCore begin: twai_driver_uninstall status=%d",
          static_cast<int>(uninstallStatus));
    }

    _driverInstalled = false;
  }

  if (!validatePins()) {
    _stats.beginErrors = _stats.beginErrors + 1U;
    _stats.lastError = static_cast<uint32_t>(ESP_ERR_INVALID_ARG);
    LOG_ERROR(
        "AimEsp32CanCore begin failed: invalid pins rx=%d tx=%d",
        _rxPin,
        _txPin);
    return false;
  }

  twai_general_config_t gConfig =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)_txPin, (gpio_num_t)_rxPin, TWAI_MODE_NORMAL);
  gConfig.tx_queue_len = 16;
  gConfig.rx_queue_len = 16;

  twai_timing_config_t tConfig;
  if (!configureTiming(tConfig)) {
    _stats.beginErrors = _stats.beginErrors + 1U;
    _stats.lastError = static_cast<uint32_t>(ESP_ERR_INVALID_ARG);
    LOG_ERROR(
        "AimEsp32CanCore begin failed: unsupported baud=%lu",
        static_cast<unsigned long>(_baud));
    return false;
  }

#if defined(TWAI_FILTER_CONFIG_ACCEPT_ALL)
  twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();
#else
  twai_filter_config_t fConfig = {
      .acceptance_code = 0U,
      .acceptance_mask = 0xFFFFFFFFU,
      .single_filter = true};
#endif

  esp_err_t status = twai_driver_install(&gConfig, &tConfig, &fConfig);
  if (status != ESP_OK) {
    _stats.beginErrors = _stats.beginErrors + 1U;
    _stats.lastError = static_cast<uint32_t>(status);
    LOG_ERROR(
        "AimEsp32CanCore begin failed: twai_driver_install status=%d rx=%d tx=%d baud=%lu",
        static_cast<int>(status),
        _rxPin,
        _txPin,
        static_cast<unsigned long>(_baud));
    return false;
  }

  _driverInstalled = true;

  status = twai_start();
  if (status != ESP_OK) {
    _stats.beginErrors = _stats.beginErrors + 1U;
    _stats.lastError = static_cast<uint32_t>(status);

    LOG_ERROR(
        "AimEsp32CanCore begin failed: twai_start status=%d",
        static_cast<int>(status));

    const esp_err_t uninstallStatus = twai_driver_uninstall();
    if ((uninstallStatus != ESP_OK) && (uninstallStatus != ESP_ERR_INVALID_STATE)) {
      LOG_WARN(
          "AimEsp32CanCore begin: twai_driver_uninstall after start fail status=%d",
          static_cast<int>(uninstallStatus));
    }

    _driverInstalled = false;
    return false;
  }

  const uint32_t alerts = TWAI_ALERT_ERR_PASS |
                          TWAI_ALERT_BUS_ERROR |
                          TWAI_ALERT_BUS_OFF |
                          TWAI_ALERT_BUS_RECOVERED |
                          TWAI_ALERT_TX_FAILED;
  status = twai_reconfigure_alerts(alerts, nullptr);
  if (status != ESP_OK) {
    LOG_WARN(
        "AimEsp32CanCore begin: twai_reconfigure_alerts status=%d",
        static_cast<int>(status));
  }

  _stats.lastError = static_cast<uint32_t>(ESP_OK);
  _initialized = true;
  return true;
}

void AimEsp32CanCore::captureTwaiCounters() {
  twai_status_info_t info = {};
  if (twai_get_status_info(&info) == ESP_OK) {
    _stats.lastBusErrCount = info.bus_error_count;
    _stats.lastTec = info.tx_error_counter;
    _stats.lastRec = info.rx_error_counter;
  }
}

bool AimEsp32CanCore::recoverBusOff() {
  const uint32_t nowMs = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
  if ((nowMs - _lastRecoveryMs) < kRecoveryCooldownMs) {
    return false;
  }
  _lastRecoveryMs = nowMs;
  _stats.busOffRecoveries = _stats.busOffRecoveries + 1U;

  LOG_WARN(
      "AimEsp32CanCore bus-off recovery #%lu",
      static_cast<unsigned long>(_stats.busOffRecoveries));

  return begin();
}

bool AimEsp32CanCore::transmit(const aim::Frame& frame) {
  AIM_ASSERT(frame.dlc <= 8U);

  if (!_initialized) {
    _stats.txErrors = _stats.txErrors + 1U;
    _stats.lastError = static_cast<uint32_t>(ESP_ERR_INVALID_STATE);
    LOG_ERROR("AimEsp32CanCore transmit failed: core not initialized");
    return false;
  }

  if (frame.dlc != 8U) {
    _stats.txErrors = _stats.txErrors + 1U;
    _stats.lastError = static_cast<uint32_t>(ESP_ERR_INVALID_SIZE);
    LOG_ERROR(
        "AimEsp32CanCore transmit failed: invalid dlc=%u",
        static_cast<unsigned>(frame.dlc));
    return false;
  }

  twai_message_t msg = {};
  msg.identifier = frame.id & aim::kExtIdMask;
  msg.extd = 1;
  msg.rtr = 0;
  msg.data_length_code = frame.dlc;
  (void)memcpy(msg.data, frame.data, frame.dlc);

  esp_err_t status = twai_transmit(&msg, 0);

  if (status == ESP_ERR_INVALID_STATE) {
    if (recoverBusOff()) {
      status = twai_transmit(&msg, 0);
    }
    if (status != ESP_OK) {
      captureTwaiCounters();
      _stats.txErrors = _stats.txErrors + 1U;
      _stats.lastError = static_cast<uint32_t>(status);
      return false;
    }
  }

  if (status != ESP_OK) {
    captureTwaiCounters();
    _stats.txErrors = _stats.txErrors + 1U;
    _stats.lastError = static_cast<uint32_t>(status);
    LOG_ERROR(
        "AimEsp32CanCore transmit failed: id=0x%08lX status=%d busErr=%lu tec=%lu rec=%lu",
        static_cast<unsigned long>(msg.identifier),
        static_cast<int>(status),
        static_cast<unsigned long>(_stats.lastBusErrCount),
        static_cast<unsigned long>(_stats.lastTec),
        static_cast<unsigned long>(_stats.lastRec));
    return false;
  }

  _stats.txFrames = _stats.txFrames + 1U;
  _stats.lastError = static_cast<uint32_t>(ESP_OK);
  return true;
}

bool AimEsp32CanCore::shouldAcceptId(const uint32_t id) const {
  // Reserved bits (10:0) ignored per protocol invariant; source==0 is the
  // wiring-fault canary and is dropped.
  const uint8_t cls = static_cast<uint8_t>((id >> aim::kIdClassShift) & 0xFU);
  const uint8_t src = static_cast<uint8_t>((id >> aim::kIdSourceShift) & 0xFU);
  return (src != 0U) && ((_classMask & (1U << cls)) != 0U);
}

bool AimEsp32CanCore::receive(aim::Frame& frame) {
  AIM_ASSERT(_classMask != 0U);

  if (!_initialized) {
    _stats.rxErrors = _stats.rxErrors + 1U;
    _stats.lastError = static_cast<uint32_t>(ESP_ERR_INVALID_STATE);
    LOG_ERROR("AimEsp32CanCore receive failed: core not initialized");
    return false;
  }

  twai_message_t msg = {};
  const esp_err_t status = twai_receive(&msg, 0);
  if (status == ESP_ERR_TIMEOUT) {
    return false;
  }
  if (status != ESP_OK) {
    _stats.rxErrors = _stats.rxErrors + 1U;
    _stats.lastError = static_cast<uint32_t>(status);
    LOG_ERROR(
        "AimEsp32CanCore receive failed: twai_receive status=%d",
        static_cast<int>(status));
    return false;
  }

  if ((msg.extd == 0U) || (msg.rtr != 0U)) {
    _stats.filteredFrames = _stats.filteredFrames + 1U;
    return false;
  }

  if (msg.data_length_code != 8U) {
    _stats.rxErrors = _stats.rxErrors + 1U;
    _stats.lastError = static_cast<uint32_t>(ESP_ERR_INVALID_SIZE);
    LOG_ERROR(
        "AimEsp32CanCore receive failed: invalid dlc=%u id=0x%08lX",
        static_cast<unsigned>(msg.data_length_code),
        static_cast<unsigned long>(msg.identifier & aim::kExtIdMask));
    return false;
  }

  const uint32_t id = msg.identifier & aim::kExtIdMask;
  if (!shouldAcceptId(id)) {
    _stats.filteredFrames = _stats.filteredFrames + 1U;
    return false;
  }

  frame.id = id;
  frame.dlc = msg.data_length_code;
  (void)memcpy(frame.data, msg.data, frame.dlc);

  _stats.rxFrames = _stats.rxFrames + 1U;
  _stats.lastError = static_cast<uint32_t>(ESP_OK);
  return true;
}

void AimEsp32CanCore::getStats(Stats& stats) const {
  stats = _stats;
}

void AimEsp32CanCore::clearStats() {
  _stats = {};
}

#endif  // ARDUINO_ARCH_ESP32
