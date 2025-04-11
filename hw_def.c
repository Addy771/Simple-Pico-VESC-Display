#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include <hw_def.h>


void initialize_gpio(void)
{
    // Backlight control
    gpio_init(DISPLAY_BACKLIGHT_GPIO);
    gpio_set_dir(DISPLAY_BACKLIGHT_GPIO, GPIO_OUT);
    gpio_put(DISPLAY_BACKLIGHT_GPIO, 1);    // Start with backlight FET off

    // External Loads control
    gpio_init(EXT_LOAD_A_GPIO);
    gpio_init(EXT_LOAD_B_GPIO);
    gpio_set_dir(EXT_LOAD_A_GPIO, GPIO_OUT);
    gpio_set_dir(EXT_LOAD_B_GPIO, GPIO_OUT);
    // Start with external loads off
    gpio_put(EXT_LOAD_A_GPIO, 0);
    gpio_put(EXT_LOAD_B_GPIO, 0);

    // WS LED
    gpio_init(WS_LED_GPIO);
    gpio_set_dir(WS_LED_GPIO, GPIO_OUT);
    gpio_put(WS_LED_GPIO, 0);    
    
    // CAN GPIO
    gpio_set_function(CAN_TX_GPIO, GPIO_FUNC_UART);
    gpio_set_function(CAN_RX_GPIO, GPIO_FUNC_UART);

    // UART GPIO
    gpio_set_function(UART_TX_GPIO, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_GPIO, GPIO_FUNC_UART);

    //I2C GPIO
    i2c_init(i2c0, RTC_I2C_CLK);  
    gpio_set_function(RTC_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(RTC_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(RTC_SDA_GPIO);
    gpio_pull_up(RTC_SCL_GPIO);    

    // ADC inputs

    // Init debug GPIO
    gpio_init(DEBUG_GPIO);
    gpio_set_dir(DEBUG_GPIO, GPIO_OUT);
    gpio_put(DEBUG_GPIO, 1);

    // Enable pushbutton pullups
    gpio_pull_up(PB_LEFT_GPIO);
    gpio_pull_up(PB_RIGHT_GPIO);
    gpio_pull_up(PB_CENTER_GPIO);    


    /*
        Pico W Notes

        Onboard LED and PWM mode GPIO have been repurposed on Pico W and can't be used directly.
        These are now driven by GPIO on the CYW43439 WiFi/BT interface chip. 

        To use these GPIO requires different functions:
        cyw43_arch_gpio_put(wl_gpio, value)
        cyw43_arch_gpio_get(wl_gpio)

        which can only be used after the CYW43 architecture has been initialized:
        cyw43_arch_init()
    
    */

}