#include "aim_can_driver.h"

AimCanDriver::AimCanDriver(uint8_t origin, uint32_t baud, int rxPin, int txPin) {
  _origin = origin;
  _baud   = baud;
  _rxPin  = rxPin;
  _txPin  = txPin;
  _initialized = false;
}

// ── STM32 ────────────────────────────────────────────────────
#if defined(ARDUINO_ARCH_STM32)

// Pin mapping: CAN1 ALT = PB8/PB9 (standard for STM32 QRET modules)
STM32_CAN AimCanDriver::_canb(CAN1, ALT, RX_SIZE_16, TX_SIZE_16);

void AimCanDriver::begin() {
  _canb.begin();
  _canb.setBaudRate(_baud);

  // Accept packets addressed to this module or broadcast
  _canb.setMBFilterProcessing(MB0, (uint16_t)((_origin & 0x07) << 5), 0x0E0);
  _canb.setMBFilterProcessing(MB1, (uint16_t)(AIM_DEST_BROADCAST << 5), 0x0E0);

  _initialized = true;
}

// AIM 11-bit CAN ID: [10:8]=origin, [7:5]=dest, [4:0]=type(4-bit)
bool AimCanDriver::packAimPkt(const aimPkt& aim_pkt, CAN_message_t& can_msg) {
  can_msg.id = (((aim_pkt.origin & 0x07) << 8) |
                ((aim_pkt.dest   & 0x07) << 5) |
                ((aim_pkt.type   & 0x0F))) & 0x07FF;
  can_msg.flags.extended = 0;
  can_msg.len = sizeof(aim_pkt.data);
  if (can_msg.len != 8) return false;

  memcpy(can_msg.buf, &aim_pkt.data, sizeof(aim_pkt.data));
  return true;
}

bool AimCanDriver::unpackAimPkt(const CAN_message_t& can_msg, aimPkt& aim_pkt) {
  aim_pkt.origin = (can_msg.id >> 8) & 0x07;
  aim_pkt.dest   = (can_msg.id >> 5) & 0x07;
  aim_pkt.type   =  can_msg.id       & 0x0F;

  if (sizeof(aim_pkt.data) != can_msg.len) return false;

  memcpy(&aim_pkt.data, can_msg.buf, can_msg.len);
  return true;
}

bool AimCanDriver::transmit(const uint8_t* buf, size_t len) {
  if (len != sizeof(aimPkt) || !_initialized) return false;

  aimPkt pkt;
  memcpy(&pkt, buf, sizeof(pkt));

  CAN_message_t can_msg;
  if (!packAimPkt(pkt, can_msg)) return false;

  return _canb.write(can_msg);
}

bool AimCanDriver::receive(uint8_t* buf, size_t len) {
  if (len != sizeof(aimPkt) || !_initialized) return false;

  CAN_message_t can_msg;
  if (!_canb.read(can_msg)) return false;

  aimPkt pkt;
  if (!unpackAimPkt(can_msg, pkt)) return false;

  memcpy(buf, &pkt, sizeof(pkt));
  return true;
}


// ── ESP32 ────────────────────────────────────────────────────
#elif defined(ARDUINO_ARCH_ESP32)

void AimCanDriver::begin() {
  _initialized = false;  // Reset in case begin() is called multiple times

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)_txPin, (gpio_num_t)_rxPin, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config;
  twai_filter_config_t f_config = {
    .acceptance_code = ((uint32_t)(_origin & 0x07) << 5) | ((uint32_t)AIM_DEST_BROADCAST << 5),
    .acceptance_mask = 0x0E0 | 0x0E0,  // Mask for dest bits [7:5]
    .single_filter = false  // Use dual filter
  };

  // Set baud rate based on _baud (assuming common rates; adjust as needed)
  if (_baud == 500000) {
    t_config = TWAI_TIMING_CONFIG_500KBITS();
  } else if (_baud == 250000) {
    t_config = TWAI_TIMING_CONFIG_250KBITS();
  } else if (_baud == 125000) {
    t_config = TWAI_TIMING_CONFIG_125KBITS();
  } else {
    // Default to 500kbps
    t_config = TWAI_TIMING_CONFIG_500KBITS();
  }

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    if (twai_start() == ESP_OK) {
      _initialized = true;
    }
  }
}

bool AimCanDriver::packAimPkt(const aimPkt& aim_pkt, twai_message_t& twai_msg) {
  twai_msg.identifier = (((aim_pkt.origin & 0x07) << 8) |
                        ((aim_pkt.dest   & 0x07) << 5) |
                        ((aim_pkt.type   & 0x0F))) & 0x07FF;
  twai_msg.extd = 0;  // Standard 11-bit ID
  twai_msg.data_length_code = sizeof(aim_pkt.data);
  if (twai_msg.data_length_code != 8) return false;

  memcpy(twai_msg.data, &aim_pkt.data, sizeof(aim_pkt.data));
  return true;
}

bool AimCanDriver::unpackAimPkt(const twai_message_t& twai_msg, aimPkt& aim_pkt) {
  aim_pkt.origin = (twai_msg.identifier >> 8) & 0x07;
  aim_pkt.dest   = (twai_msg.identifier >> 5) & 0x07;
  aim_pkt.type   =  twai_msg.identifier       & 0x0F;

  if (sizeof(aim_pkt.data) != twai_msg.data_length_code) return false;

  memcpy(&aim_pkt.data, twai_msg.data, twai_msg.data_length_code);
  return true;
}

bool AimCanDriver::transmit(const uint8_t* buf, size_t len) {
  if (len != sizeof(aimPkt) || !_initialized) return false;

  aimPkt pkt;
  memcpy(&pkt, buf, sizeof(pkt));

  twai_message_t twai_msg;
  if (!packAimPkt(pkt, twai_msg)) return false;

  return twai_transmit(&twai_msg, pdMS_TO_TICKS(1000)) == ESP_OK; // switch function to non-blocking

}

bool AimCanDriver::receive(uint8_t* buf, size_t len) {
  if (len != sizeof(aimPkt) || !_initialized) return false;

  twai_message_t twai_msg;
  if (twai_receive(&twai_msg, pdMS_TO_TICKS(1000)) != ESP_OK) return false; // switch function to non-blocking

  aimPkt pkt;
  if (!unpackAimPkt(twai_msg, pkt)) return false;

  memcpy(buf, &pkt, sizeof(pkt));
  return true;
}

#endif
