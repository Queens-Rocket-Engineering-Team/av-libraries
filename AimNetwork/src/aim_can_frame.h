#ifndef AIM_CAN_FRAME_H
#define AIM_CAN_FRAME_H

#include <cstdint>

namespace aim {

/**
 * @brief Raw CAN frame format used by all AimNetwork drivers.
 */
struct Frame {
  uint16_t id;
  uint8_t dlc;
  uint8_t data[8];
};

} // namespace aim

#endif // AIM_CAN_FRAME_H
