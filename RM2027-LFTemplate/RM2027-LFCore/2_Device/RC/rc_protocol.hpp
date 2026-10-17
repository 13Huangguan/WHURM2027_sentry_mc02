#ifndef RC_PROTOCOL_HPP
#define RC_PROTOCOL_HPP

#include "remote_control.h"

namespace rc_protocol {
// Decode a complete frame without HAL/DMA dependencies. Outputs are unchanged for invalid lengths.
bool decodeFrame(const uint8_t* buffer, uint16_t length, RcControl* control, SbusData* sbus);
} // namespace rc_protocol

#endif
