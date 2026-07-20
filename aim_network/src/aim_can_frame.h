#ifndef AIM_CAN_FRAME_H
#define AIM_CAN_FRAME_H

#include <cstdint>

namespace aim {

/**
 * @brief Raw CAN frame format used by all AimNetwork drivers.
 * All AIM frames use 29-bit extended IDs; cores hard-code IDE.
 */
struct Frame {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
};

} // namespace aim

#endif // AIM_CAN_FRAME_H
