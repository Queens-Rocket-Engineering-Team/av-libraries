// aim_can_driver.h — Platform-aware CAN driver for AimNetwork
// Supports STM32 (HAL core in AimStm32CanCore) and ESP32 (TWAI)

#ifndef AIM_CAN_DRIVER_H
#define AIM_CAN_DRIVER_H

#include "aim_network.h"

#if defined(ARDUINO_ARCH_STM32)
  #include "aim_stm32_can_core.h"
#elif defined(ARDUINO_ARCH_ESP32)
  #include <driver/twai.h>
#else
  #error "AimCanDriver: Unsupported platform"
#endif


class AimCanDriver : public AimTransceiver {
public:
  AimCanDriver(uint8_t origin, uint32_t baud, int rxPin = 0, int txPin = 0);

  void begin() override;
  bool transmit(const uint8_t* buf, size_t len) override;
  bool receive(uint8_t* buf, size_t len) override;

private:
  uint8_t  _origin;
  uint32_t _baud;
  int      _rxPin;
  int      _txPin;
  bool     _initialized;

#if defined(ARDUINO_ARCH_STM32)
  AimStm32CanCore _canCore;
  bool packAimPkt(const aimPkt& aim_pkt, AimStm32CanCore::Frame& can_msg);
  bool unpackAimPkt(const AimStm32CanCore::Frame& can_msg, aimPkt& aim_pkt);

#elif defined(ARDUINO_ARCH_ESP32)
  bool packAimPkt(const aimPkt& aim_pkt, twai_message_t& twai_msg);
  bool unpackAimPkt(const twai_message_t& twai_msg, aimPkt& aim_pkt);
#endif
};

#endif // AIM_CAN_DRIVER_H
