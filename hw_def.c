#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
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

    // I2C GPIO
    i2c_init(0, RTC_I2C_CLK);   // i2c0
    gpio_set_function(RTC_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(RTC_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(RTC_SDA_GPIO);
    gpio_pull_up(RTC_SCL_GPIO);    

    // ADC inputs


    // Set Pico DC/DC converter to PWM mode to reduce noise at light load
    // *******figure out the GPIO number without hardcoding
    //gpio_put(23, 1);

    // Init debug GPIO
    gpio_init(DEBUG_GPIO);
    gpio_set_dir(DEBUG_GPIO, GPIO_OUT);
    gpio_put(DEBUG_GPIO, 1);

    // Enable pushbutton pullups
    gpio_pull_up(PB_LEFT_GPIO);
    gpio_pull_up(PB_RIGHT_GPIO);
    gpio_pull_up(PB_CENTER_GPIO);    

    // Set the LED pin as an output
    // gpio_init(BUILTIN_LED_PIN);
    // gpio_set_dir(BUILTIN_LED_PIN, GPIO_OUT);   
    // gpio_put(BUILTIN_LED_PIN, 0); 

}