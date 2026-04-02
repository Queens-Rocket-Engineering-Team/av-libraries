// aim_can_driver.h — Platform-aware CAN driver for AimNetwork
// Supports STM32 (STM32_CAN) and ESP32 (TWAI)

#ifndef AIM_CAN_DRIVER_H
#define AIM_CAN_DRIVER_H

#include "aim_network.h"

#if defined(ARDUINO_ARCH_STM32)
  #include "STM32_CAN.h"  // v1.1.2
#elif defined(ARDUINO_ARCH_ESP32)
  #include <driver/twai.h>
#else
  #error "AimCanDriver: Unsupported platform"
#endif


class AimCanDriver : public AimTransceiver {
public:
  AimCanDriver(uint8_t origin, uint32_t baud, int rxPin, int txPin);

  void begin() override;
  bool transmit(const uint8_t* buf, size_t len) override;
  bool receive(uint8_t* buf, size_t len) override;

private:
  uint8_t  _origin;
  uint32_t _baud;
  int      _rxPin;
  int      _txPin;

#if defined(ARDUINO_ARCH_STM32)
  static STM32_CAN* _canb;
  bool packAimPkt(const aimPkt& aim_pkt, CAN_message_t& can_msg);
  bool unpackAimPkt(const CAN_message_t& can_msg, aimPkt& aim_pkt);

#elif defined(ARDUINO_ARCH_ESP32)
  bool _initialized;
  bool packAimPkt(const aimPkt& aim_pkt, twai_message_t& twai_msg);
  bool unpackAimPkt(const twai_message_t& twai_msg, aimPkt& aim_pkt);
#endif
};

#endif // AIM_CAN_DRIVER_H