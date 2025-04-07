#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <hw_def.h>
#include <u8g2.h>


// Handle GPIO initialization and control
uint8_t u8x8_gpio_and_delay_pico(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{

    switch (msg)
    {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:

        gpio_init(DISPLAY_RST_GPIO);
        gpio_init(DISPLAY_CS_GPIO);
        gpio_init(DISPLAY_SCL_GPIO);
        gpio_init(DISPLAY_SDA_GPIO);

        gpio_set_dir(DISPLAY_RST_GPIO, GPIO_OUT);
        gpio_set_dir(DISPLAY_CS_GPIO, GPIO_OUT);
        gpio_set_dir(DISPLAY_SCL_GPIO, GPIO_OUT);
        gpio_set_dir(DISPLAY_SDA_GPIO, GPIO_OUT);

        gpio_put(DISPLAY_RST_GPIO, 1);
        gpio_put(DISPLAY_CS_GPIO, 1);
        gpio_put(DISPLAY_SCL_GPIO, 0);
        gpio_put(DISPLAY_SDA_GPIO, 0);

        break;
    case U8X8_MSG_DELAY_NANO: // delay arg_int * 1 nano second
        sleep_us(arg_int);    // 1000 times slower, though generally fine in practice given rp2040 has no `sleep_ns()`
        break;
    case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
        sleep_us(arg_int);
        break;
    case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
        sleep_us(arg_int * 10);
        break;
    case U8X8_MSG_DELAY_MILLI: // delay arg_int * 1 milli second
        sleep_ms(arg_int);
        break;
    case U8X8_MSG_GPIO_CS: // CS (chip select) pin: Output level in arg_int
        gpio_put(DISPLAY_CS_GPIO, arg_int);
        break;
    case U8X8_MSG_GPIO_RESET:       // Reset pin: Output level in arg_int
        gpio_put(DISPLAY_RST_GPIO, arg_int); // printf("U8X8_MSG_GPIO_RESET %d\n", arg_int);
        break;
    case U8X8_MSG_GPIO_SPI_CLOCK:   // Clock pin: Output level in arg_int
        gpio_put(DISPLAY_SCL_GPIO, arg_int);
        break;        
    case U8X8_MSG_GPIO_SPI_DATA:    // Data pin: Output level in arg_int
        gpio_put(DISPLAY_SDA_GPIO, arg_int);
        break;         

    default:
        u8x8_SetGPIOResult(u8x8, 1); // default return value
        break;
    }
    return 1;
}