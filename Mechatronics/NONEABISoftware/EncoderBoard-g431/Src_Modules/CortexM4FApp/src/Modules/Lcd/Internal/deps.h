#pragma once

#include <inttypes.h>

void __ucHAL_Display_function_transmit(uint8_t i2c_addr, uint8_t *buffer, uint32_t nbytes);

#define LCD_TRANSMIT(i2c_addr, buffer, nbytes) __ucHAL_Display_function_transmit(i2c_addr, buffer, nbytes)