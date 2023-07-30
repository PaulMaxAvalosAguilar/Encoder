#pragma once

#include <inttypes.h>

void __ucHAL_Bluetooth_function_transmit(uint8_t *buffer, uint32_t nbytes);

#define BLUETOOTH_TRANSMIT(buffer, nbytes) __ucHAL_Bluetooth_function_transmit(buffer, nbytes)