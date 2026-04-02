#include "aim_can_driver.h"


// ── STM32 ────────────────────────────────────────────────────
#if defined(ARDUINO_ARCH_STM32)

STM32_CAN* AimCanDriver::_canb = nullptr;

AimCanDriver::AimCanDriver(uint8_t origin, uint32_t baud, int rxPin, int txPin) {
  _origin = origin;
  _baud   = baud;
  _rxPin  = rxPin;
  _txPin  = txPin;
}

void AimCanDriver::begin() {
  // Pin mapping: CAN1 ALT = PB8/PB9 (standard for QRET modules)
  _canb = new STM32_CAN(CAN1, ALT, RX_SIZE_16, TX_SIZE_16);
  _canb->begin();
  _canb->setBaudRate(_baud);

  // Accept packets addressed to this module or broadcast
  _canb->setMBFilterProcessing(MB0, (uint16_t)((_origin & 0x07) << 5), 0x0E0);
  _canb->setMBFilterProcessing(MB1, (uint16_t)(AIM_DEST_BROADCAST << 5), 0x0E0);
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
  if (len != sizeof(aimPkt)) return false;

  aimPkt pkt;
  memcpy(&pkt, buf, sizeof(pkt));

  CAN_message_t can_msg;
  if (!packAimPkt(pkt, can_msg)) return false;

  return _canb->write(can_msg);
}

bool AimCanDriver::receive(uint8_t* buf, size_t len) {
  if (len != sizeof(aimPkt)) return false;

  CAN_message_t can_msg;
  if (!_canb->read(can_msg)) return false;

  aimPkt pkt;
  if (!unpackAimPkt(can_msg, pkt)) return false;

  memcpy(buf, &pkt, sizeof(pkt));
  return true;
}


// ── ESP32 ────────────────────────────────────────────────────
#elif defined(ARDUINO_ARCH_ESP32)

AimCanDriver::AimCanDriver(uint8_t origin, uint32_t baud, int rxPin, int txPin) {
  _origin      = origin;
  _baud        = baud;
  _rxPin       = rxPin;
  _txPin       = txPin;
  _initialized = false;
}

void AimCanDriver::begin() {
  // TODO: twai_driver_install() + twai_start() using _txPin, _rxPin, _baud
  _initialized = false;
}

bool AimCanDriver::packAimPkt(const aimPkt& aim_pkt, twai_message_t& twai_msg) {
  // TODO: same ID packing as STM32 → twai_msg.identifier
  (void)aim_pkt; (void)twai_msg;
  return false;
}

bool AimCanDriver::unpackAimPkt(const twai_message_t& twai_msg, aimPkt& aim_pkt) {
  (void)twai_msg; (void)aim_pkt;
  return false;
}

bool AimCanDriver::transmit(const uint8_t* buf, size_t len) {
  (void)buf; (void)len;
  return false;
}

bool AimCanDriver::receive(uint8_t* buf, size_t len) {
  (void)buf; (void)len;
  return false;
}

#endif
