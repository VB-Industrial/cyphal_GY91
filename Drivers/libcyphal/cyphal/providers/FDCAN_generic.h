#pragma once

#include <cyphal/definitions.h>
#include <cstdint>

extern const uint32_t CanardFDCANLengthToDLC[65];
extern size_t fdcan_dlc_to_len(uint32_t);
