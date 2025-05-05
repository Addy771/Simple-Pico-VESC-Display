#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/rtc.h"
#include <hw_def.h>

#define DISPLAY_PWM_COUNT 1024

uint8_t rtc_connected;
uint8_t rtc_time_valid;

void set_backlight(uint8_t brightness)
{
    pwm_set_gpio_level(DISPLAY_BACKLIGHT_GPIO, brightness * (DISPLAY_PWM_COUNT / 256)); 
}

void initialize_gpio(void)
{
    // Backlight control
    gpio_init(DISPLAY_BACKLIGHT_GPIO);
    gpio_set_dir(DISPLAY_BACKLIGHT_GPIO, GPIO_OUT);
    gpio_set_function(DISPLAY_BACKLIGHT_GPIO, GPIO_FUNC_PWM);

    pwm_config pwm_cnf = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&pwm_cnf, (frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) / DISPLAY_PWM_COUNT) / (DISPLAY_BACKLIGHT_PWM_KHZ));
    pwm_config_set_output_polarity(&pwm_cnf, true, false);   // Invert output A because backlight control is active low
    pwm_config_set_wrap(&pwm_cnf, DISPLAY_PWM_COUNT-1); 
    pwm_set_gpio_level(DISPLAY_BACKLIGHT_GPIO, 0);    // Start with backlight FET off

    pwm_init(pwm_gpio_to_slice_num(DISPLAY_BACKLIGHT_GPIO), &pwm_cnf, true);

    // External Loads control
    gpio_init(EXT_LOAD_A_GPIO);
    gpio_init(EXT_LOAD_B_GPIO);
    gpio_set_dir(EXT_LOAD_A_GPIO, GPIO_OUT);
    gpio_set_dir(EXT_LOAD_B_GPIO, GPIO_OUT);
    // Start with external loads off
    gpio_put(EXT_LOAD_A_GPIO, 0);
    gpio_put(EXT_LOAD_B_GPIO, 0);

    // SD SPI
    // more settings are in hw_config.c
    gpio_init(SD_DETECT_GPIO);
    gpio_set_dir(SD_DETECT_GPIO, GPIO_IN);
    gpio_pull_up(SD_DETECT_GPIO);

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
    i2c_init(RTC_I2C_UNIT, RTC_I2C_CLK);  
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

    cyw43_arch_gpio_put(CYW43_SMPS_PS_GPIO, 1);     // Enable PWM mode on DC/DC to reduce noise

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


void init_external_rtc(void)
{
    uint8_t rx_byte;
    int32_t ret;
    uint8_t transmit_buf[2];

    rtc_init(); // on-chip RTC

    // Check if DS1307 is present
    ret = i2c_read_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &rx_byte, 1, false);

    if (ret != PICO_ERROR_GENERIC)
    {
        rtc_connected = 1;

        // Set 24hr mode
        // Read current hours value before modifying
        transmit_buf[0] = BCD_HOURS;
        i2c_write_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &transmit_buf, 1, false); // set register address
        i2c_read_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &rx_byte, 1, false);  

        rx_byte &= ~(1 << 6);   // Clear b6 to put in 24hr mode
        transmit_buf[0] = BCD_HOURS;
        transmit_buf[1] = rx_byte;
        i2c_write_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &transmit_buf, 2, false);    // write hours value back to rtc

        // Clear clock halt bit
        // Read current seconds value before modifying
        transmit_buf[0] = BCD_SECONDS;
        i2c_write_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &transmit_buf, 1, false); // set register address
        i2c_read_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &rx_byte, 1, false);          

        rx_byte &= ~(1 << 7);   // Clear b7 to disable clock halt
        transmit_buf[0] = BCD_SECONDS;
        transmit_buf[1] = rx_byte;
        i2c_write_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &transmit_buf, 2, false);    // write hours value back to rtc    
        
        // Check if the RTC's time value is valid
        transmit_buf[0] = RTC_SRAM_KEY;
        i2c_write_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &transmit_buf, 1, false); // set register address
        i2c_read_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &rx_byte, 1, false);      

        if (rx_byte == RTC_KEY_VALUE)
            rtc_time_valid = 1;
        else
            rtc_time_valid = 0;
    }
    else
    {
        rtc_connected = 0;
    }


}

// Write a time to the external RTC. Updates internal RTC as well.
void set_rtc_time(datetime_t current_time)
{
    uint8_t i2c_buf[8];

    rtc_set_datetime(&current_time);

    if (!rtc_connected)
        return;     // abort if external rtc isn't functional

    i2c_buf[0] = 0; // First byte sets register pointer

    // Pack time values into BCD format
    i2c_buf[BCD_SECONDS+1] = (current_time.sec / 10) << 4 | (current_time.sec % 10);
    i2c_buf[BCD_MINUTES+1] = (current_time.min / 10) << 4 | (current_time.min % 10);
    i2c_buf[BCD_HOURS+1] = (current_time.hour / 10) << 4 | (current_time.hour % 10);
    i2c_buf[BCD_DAYS+1] = (current_time.day / 10) << 4 | (current_time.day % 10);
    i2c_buf[WEEKDAY+1] = current_time.dotw + 1;   // Pico RTC days are 0 indexed, but ds1307 is 1 indexed
    i2c_buf[BCD_MONTHS+1] = (current_time.month / 10) << 4 | (current_time.month % 10);
    current_time.year -= 2000;  // Pico RTC stores full year value, but DS1307 only keeps 2 digits
    i2c_buf[BCD_YEARS+1] = (current_time.year / 10) << 4 | (current_time.year % 10);

    i2c_write_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &i2c_buf, 8, false); // Write time values to RTC

    // Write a pattern to RTC SRAM to indicate that the time is valid
    i2c_buf[0] = RTC_SRAM_KEY;
    i2c_buf[1] = RTC_KEY_VALUE;
    i2c_write_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &i2c_buf, 2, false);

    rtc_time_valid = 1;
}

// Copy time from external RTC to internal RTC
void get_rtc_time(datetime_t *rtc_time)
{
    uint8_t i2c_buf[7];

    if (!rtc_connected)
        return;     // abort if external rtc isn't functional

    i2c_buf[0] = 0;
    i2c_write_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &i2c_buf, 1, false); // set register pointer to 0 

    // Read all time value registers at once
    i2c_read_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &i2c_buf, 7, false);

    // Unpack BCD time values into datetime_t format
    (*rtc_time).sec = ((i2c_buf[BCD_SECONDS] & 0x70) >> 4) * 10 + (i2c_buf[BCD_SECONDS] & 0x0F);
    (*rtc_time).min = ((i2c_buf[BCD_MINUTES] & 0x70) >> 4) * 10 + (i2c_buf[BCD_MINUTES] & 0x0F);
    (*rtc_time).hour = ((i2c_buf[BCD_HOURS] & 0x30) >> 4) * 10 + (i2c_buf[BCD_HOURS] & 0x0F);
    (*rtc_time).day = ((i2c_buf[BCD_DAYS] & 0x30) >> 4) * 10 + (i2c_buf[BCD_DAYS] & 0x0F);    
    (*rtc_time).dotw = i2c_buf[WEEKDAY] - 1;  // Pico RTC days are 0 indexed, but ds1307 is 1 indexed
    (*rtc_time).month = ((i2c_buf[BCD_MONTHS] & 0x30) >> 4) * 10 + (i2c_buf[BCD_MONTHS] & 0x0F);  
    (*rtc_time).year = ((i2c_buf[BCD_YEARS] & 0xF0) >> 4) * 10 + (i2c_buf[BCD_YEARS] & 0x0F) + 2000;    // Pico RTC stores full year value, but DS1307 only keeps 2 digits

    // Set on-chip RTC to the same time
    rtc_set_datetime(rtc_time);
}

// Write 1 or more bytes to RTC's SRAM region
void set_rtc_sram(uint8_t sram_address, uint8_t *data, uint8_t len)
{
    uint8_t sram_buf[57];

    sram_buf[0] = sram_address;
    memcpy(sram_buf+1, data, len); 

    i2c_write_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &sram_buf, len + 1, false);    
}

// Read 1 or more bytes from RTC's SRAM region
void get_rtc_sram(uint8_t sram_address, uint8_t *data, uint8_t len)
{
    i2c_write_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, &sram_address, 1, false); // set register pointer
    
    i2c_read_blocking(RTC_I2C_UNIT, RTC_I2C_ADDR, data, 7, len);
}