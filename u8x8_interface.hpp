#include <stdlib.h>
#include "pico/stdlib.h"

#ifndef U8X8_INTERFACE_H
#define U8X8_INTERFACE_H

#define PIO_3WIRE_CLK_KHZ 12000
//#define U8G2_USE_SW_SPI



uint8_t __not_in_flash_func(u8x8_gpio_and_delay_pico)(u8x8_t *u8x8, uint8_t msg,uint8_t arg_int, void *arg_ptr);
uint8_t __not_in_flash_func(u8x8_byte_pio_3wire_spi)(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#ifdef U8G2_USE_SW_SPI
#define U8G2_BYTE_FN u8x8_byte_3wire_sw_spi

#else
#define U8G2_BYTE_FN u8x8_byte_pio_3wire_spi
#endif


#endif