#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"
#include "pico/sync.h"

#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"

#include "pico-oled/pico-oled.hpp"
#include "pico-oled/gfx_font.h"
#include "nv_flash.hpp"
#include "log.hpp"

#include "datatypes.h"
#include "packet.h"
#include "buffer.h"
#include "crc.h"

#include "util_enum.h"
#include <string.h>
#include "Button-debouncer/button_debounce.h"
#include "sd_card.h"
#include "ff.h"
#include "f_util.h"
#include "hw_config.h"
#include "rtc.h"
#include "hw_def.h"

#include "pico-oled/font/press_start_2p.h"
#include "pico-oled/font/too_simple.h"
#include "pico-oled/font/Retron2000.h"
#include "pico-oled/font/future_real.h"

/*  TODO:
- add a function in pico-led library to blank out the area where text would be
- improve analog gauge to have adjustable needle len
- add a horizontal/vertical analog gauge
- [x]add bigger font for main data (speed,power)
- fix future_real font
    - bottom pixels are cut off
    - number 3 is extra thick on the right
    - add dot, negative, positive and maybe other symbols or alphabet
- autocentering text function
- add better debug mode to be entered from display
- clean up old commented out code
- adjust rolling average for power/speed to be faster response
- [x] add odometer
- [x]save odometer to flash
- [x] add trip odometer
- [x] add whr/km for trip
- [x] add Display state button changes
- [x]add screensave state
    [x]- auto go into screen saver
    [x]- auto leave screen saver when speed changes or button pressed
- [x]whr/km
- [x]whr used
- [x] total whr used/charged/whr/km
- on default screen
    - [] Make temp bars just lines instead of hbars- or swap them with throt/regen bars better readability
- add settings page
    - reset nvflash
- add better soc lookup table for li ion battery

*/

// Comment out debug when not using
//#define DEBUG

// oled defines
#define OLED_HEIGHT 64
#define OLED_WIDTH 128
#define OLED_FRAMERATE 60

#define BOOTLOADER_BUTTON_TIME 0.5     // time in seconds for buttons to be pressed before entering bootloader mode
#define DISPLAY_RESET_BUTTON_TIME 2
#define ODOMETER_UPDATE_INTERVAL_MS 100 // time between odometer updates

#define BATT_TERMINAL_TOP_LEFT_X 4  // TODO: figure out a better way to do this
#define BATT_TERMINAL_TOP_LEFT_Y 15
#define BATT_TERMINAL_WIDTH 8
#define BATT_TERMINAL_HEIGHT 3

// Timer interval stuff
#define FAST_FLASH_INTERVAL_100MS 5   // 2 * 100ms = 200ms
#define NORMAL_FLASH_INTERVAL_100MS 10 // 5 * 100ms = 500ms
#define SLOW_FLASH_INTERVAL_100MS 20  // 10 * 100ms = 1000ms
//#define SCREENSAVER_TIMEOUT_100MS (5*10) // TESTING: 5 seconds
#define SCREENSAVER_TIMEOUT_100MS (5*60*10) // 5*60*10 * 100ms = 5 minutes
#define SCREENSAVER_MOVE_INTERVAL_100MS (5*10) // 5 seconds
//#define ODOMETER_FLASH_INTERVAL_100MS (10*10) // 10 seconds testing
#define ODOMETER_FLASH_INTERVAL_100MS (5*60*10) // 5 minutes
#define ODOMETER_CHANGED_AMOUNT 0.2         // Amount odometer must change before it can be saved to flash

// function prototypes
void draw_battery_icon();
void process_speed_calc();
void get_time_str(char * time_str);

void core1_entry();

uint32_t real_baudrate = 0;
uint8_t response_code = 0;
uint8_t vesc_connected = 0;

uint8_t get_values_response[100];

absolute_time_t prev_time_sample = get_absolute_time();


uint8_t display_normal_flashing_flag = false; // flag to indicate if text shold be shown during a normal flashing
uint8_t display_fast_flashing_flag = false; // flag to indicate if text should shown during a fast flashing
uint8_t display_slow_flashing_flag = false; // flag to indicate if text should shown during a slow flashing

float adc_v1, adc_v2;
float b_cur_avg = 0;
float b_volts_avg = 0;
float m_erpm_avg = 0;
float fps = 0;
float average_speed = 0;
float distance_travelled = 0;
float prev_kph = 0;
float trip_odometer = 0; // trip odometer resets every power cycle


int64_t time_between_odo_samples_us = 0;
uint8_t b_soc = 0;
uint8_t cells_in_series = 13;   // Hardcoded for now

uint8_t pb_left_state, pb_left_prev_state;
uint8_t pb_right_state, pb_right_prev_state;

auto_init_mutex(float_mutex); // mutex for doing float calculations only on 1 core at a time

pico_oled display(OLED_SSD1309, /*i2c_address=*/ 0x3C, /*screen_width=*/ 128, /*screen_height=*/ 64, /*reset_gpio=*/ 15);

Debounce debouncer;


// Core 0
int main()
{

    stdio_init_all();
    time_init();
    
    // Set Pico DC/DC converter to PWM mode to reduce noise at light load
    gpio_put(23, 1);

    // Init debug GPIO
    gpio_init(DEBUG_GPIO);
    gpio_set_dir(DEBUG_GPIO, GPIO_OUT);
    gpio_put(DEBUG_GPIO, 1);

    // Init i2c and configure it's GPIO pins
    //uint32_t i2c_clk = i2c_init(i2c_default, 400 * 1000);
    uint32_t i2c_clk = i2c_init(i2c_default, 1000 * 1000);  // 1120 kHz max (from testing on one display)
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Instantiate debouncer and configure GPIO
    gpio_pull_up(PB_LEFT_GPIO);
    gpio_pull_up(PB_RIGHT_GPIO);

    debouncer.debounce_gpio(PB_LEFT_GPIO);
    debouncer.set_debounce_time(PB_LEFT_GPIO, 20.0);    // 20ms debounce time
    debouncer.debounce_gpio(PB_RIGHT_GPIO);
    debouncer.set_debounce_time(PB_RIGHT_GPIO, 20.0);

    // Set initial values of button states
    pb_left_prev_state = pb_left_state = debouncer.read(PB_LEFT_GPIO);
    pb_right_prev_state = pb_right_state = debouncer.read(PB_RIGHT_GPIO);

    // Set any initial values from nvsettings
    data_pt.odometer = nv_settings.data.odometer;
    data_pt.total_watt_hours_used = nv_settings.data.total_watt_hours_used;
    data_pt.total_watt_hours_charged = nv_settings.data.total_watt_hours_charged;

    // oled init
    display.oled_init();
    display.set_brightness(nv_settings.data.disp_brightness);   // set brightness from saved flash settings

    // // Print out some system information
    // uint32_t f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);


    // for(;;)
    // {
    //     display.fill(0x1);    // 1/8th lit, 12.5%
    //     display.render();
    //     sleep_ms(5000);

    //     display.fill(0x9);    // 1/4th lit, 25%
    //     display.render();
    //     sleep_ms(5000);

    //     display.fill(0xAA);    // 1/2th lit, 50%
    //     display.render();
    //     sleep_ms(5000);

    //     display.fill(0xFF);    // all lit, 100%
    //     display.render();
    //     sleep_ms(5000);
    // }

    // // Show a page of debug values
    // display.set_cursor(0,0);
    // display.print("Pico SVD\n\n");

    // analog_gauge speedo(&display);
    // speedo.set_position(OLED_HEIGHT, 120);
    // speedo.set_scale(0, 100, 245, 295);
    // speedo.set_markers(5, 110, 16, 1);
    // speedo.set_value(25);
    // speedo.draw();

    // analog_gauge dial(&display);
    // dial.set_position(117, 53);
    // dial.set_scale(0, 60, 90, 450);
    // dial.set_markers(6, 10, 2, 1);
    // dial.set_value(10);
    // dial.draw();


    // display.render();

    // for(;;);
    // display.print("CLK_SYS:  ");
    // display.print_num("%d kHz\n", f_clk_sys);
    // display.print("Baudrate: ");
    // display.print_num("%d Bps\n", real_baudrate);
    // display.print("PRESS ANY BUTTON TO TEST SD\n");
    // display.render();

    // while(!debouncer.read(PB_LEFT_GPIO) && !debouncer.read(PB_RIGHT_GPIO));


    sleep_ms(2000); // can remove?

    // Start core 1
    multicore_launch_core1(core1_entry);


    // main() initilizing
    // Display Setup

    // Gauges setup
    #define MAX_KPH 60  // Hardcoded for now
    #define TEXT_UNDER_SPEEDO_Y_START 32

    analog_gauge speed_gauge(&display);
    speed_gauge.set_position(OLED_WIDTH/2 - 1, 120);
    speed_gauge.set_scale(0,MAX_KPH, 250, 290);
    speed_gauge.set_markers(6, 106, 12, 1);

    char temp_str[20];
    uint8_t temp_str_x, temp_str_y;

    #define ROLLING_AVG_RATIO 0.3

    absolute_time_t current_time_ms = get_absolute_time();
    absolute_time_t bootloader_timer_ms = current_time_ms;
    absolute_time_t next_frame_time = current_time_ms;
    absolute_time_t left_button_timer = current_time_ms;
    absolute_time_t next_fps_count = current_time_ms;
    absolute_time_t prev_loop_time = current_time_ms;

    // Loop Timers
    uint32_t loop_timer_100ms = 0; // 100ms timer for loop
    uint32_t screen_saver_timer_100ms = 0; // 100ms timer for screensaver

    // Flags
    uint8_t odometer_nvflash_flag = 0;  // Flag to indicate that the odometer needs to be saved to flash
    uint8_t odometer_changed_flag = 0;  // Flag to indicate that the odometer has changed more than ODOMETER_CHANGED_AMOUNT

    // LENGTH_USER_STATES is the number of user states, not including the non user states such as the screensaver
    // Add new states before LENGTH_USER_STATES
    enum state {DefaultState, AnalogSpeedState, DebugState, StatsState, LENGTH_USER_STATES, ScreensaverState};
    state display_state = DefaultState;
    state last_display_state = DefaultState;


    while(true)
    {
        // Limit framerate to OLED_FRAMERATE
        sleep_until(next_frame_time);
        current_time_ms = get_absolute_time();
        next_frame_time = delayed_by_ms(next_frame_time, 1000 / OLED_FRAMERATE);

        /////////////////////////////////////////////////////////////////
        // Update Loop Timer
        // if time between current time and last loop time is greater than 100ms, increment loop timer
        if (absolute_time_diff_us(prev_loop_time, current_time_ms) > 100000)
        {
            prev_loop_time = current_time_ms;
            loop_timer_100ms++;

            // q: how can I use loop_timer_100ms for 2 different timed operations?
            // a: use modulo operator to check if loop_timer_100ms is a multiple of 5
            //   if it is, do the thing
            //  if it isn't, don't do the thing
            // this way, you can have multiple timed operations in the same loop
            // and you can have them run at different intervals
            // and you can have them run at the same interval
            // and you can have them run at different intervals that are multiples of each other
            // and you can have them run at different intervals that are not multiples of each other
            // and you can have them run at the same interval that are multiples of each other
            // and you can have them run at the same interval that are not multiples of each other

            /////////////////////////////////////////////
            // Set things based on loop timer
            // use loop_timer_100ms to set the flashing flags
            // use loop timer to set 3 different flashing flags using modulo operator
            // the flag should be 50% on and 50% off
            if(loop_timer_100ms % FAST_FLASH_INTERVAL_100MS >= FAST_FLASH_INTERVAL_100MS/2)
                display_fast_flashing_flag = true;
            else
                display_fast_flashing_flag = false;

            if(loop_timer_100ms % NORMAL_FLASH_INTERVAL_100MS >= NORMAL_FLASH_INTERVAL_100MS/2)
                display_normal_flashing_flag = true;
            else
                display_normal_flashing_flag = false;

            if(loop_timer_100ms % SLOW_FLASH_INTERVAL_100MS >= SLOW_FLASH_INTERVAL_100MS/2)
                display_slow_flashing_flag = true;
            else
                display_slow_flashing_flag = false;

            // Enter screen saver mode if speed has been 0 for SCREENSAVER_TIMEOUT_100MS
            // We can reset the timer if the speed is not 0 and when timer is > screensaver timeout then we can enter screensaver mode
            if (data_pt.speed_kph == 0)
            {
                screen_saver_timer_100ms++;
            }
            else
            {
                screen_saver_timer_100ms = 0;
            }

            if(screen_saver_timer_100ms >= SCREENSAVER_TIMEOUT_100MS)
            {
                // only set last_display_state if we are not already in screensaver mode
                if(display_state != ScreensaverState)
                    last_display_state = display_state;
                display_state = ScreensaverState;
            }

            // Flash the odometer value every ODOMETER_FLASH_INTERVAL_100MS but also not every frame while loop_timer_100ms % ODOMETER_FLASH_INTERVAL_100MS == 0
            // this way, the odometer flashes every ODOMETER_FLASH_INTERVAL_100MS but it is not flashing every frame
            
            // Check if odometer has changed more than ODOMETER_CHANGED_AMOUNT
            mutex_enter_blocking(&float_mutex);
            if(data_pt.odometer - nv_settings.data.odometer >= ODOMETER_CHANGED_AMOUNT)
            {
                odometer_changed_flag = 1;
            }
            else
            {
                odometer_changed_flag = 0;
            }
            mutex_exit(&float_mutex);
            
            // Flash odometer value nvsettings when change is greater than ODOMETER_CHANGED_AMOUNT and ODOMETER_FLASH_INTERVAL_100MS has passed
            
            if(loop_timer_100ms % ODOMETER_FLASH_INTERVAL_100MS == 0 && odometer_changed_flag == 1 && odometer_nvflash_flag != 2 )
            {
                odometer_nvflash_flag = 1;
            }
            // else if speed is 0 and odometer has changed and odometer has not been flashed yet
            else if(data_pt.speed_kph == 0 && odometer_changed_flag == 1 && odometer_nvflash_flag != 2)
            {
                odometer_nvflash_flag = 1;
            }
            else
            {
                odometer_nvflash_flag = 0;
            }

            if(odometer_nvflash_flag == 1)
            {
                // Set flag to 2 so that it only does flashing once per loop timer and not every frame during the loop timer
                odometer_nvflash_flag = 2;
                // Flash odometer value nvsettings
                nv_settings.data.odometer = data_pt.odometer;
                // Also flash whr used/charged
                nv_settings.data.total_watt_hours_used = data_pt.total_watt_hours_used;
                nv_settings.data.total_watt_hours_charged = data_pt.total_watt_hours_charged;
                nv_settings.store_data();
            }

        } // End of loop timer stuff



        /////////////////////////////////////////////////////////////////
        // Button reads.
        // Go into bootloader when both buttons pressed for 3 seconds
        // check if both buttons are pressed. start timer
        // if button has been pressed again after set time, activate bootloader

        pb_right_prev_state = pb_right_state;
        pb_left_prev_state = pb_left_state;

        pb_right_state = debouncer.read(PB_RIGHT_GPIO);
        pb_left_state = debouncer.read(PB_LEFT_GPIO);


        // if either button is not pressed reset current time
        if (pb_left_state || pb_right_state)
        {
            //store current time
            bootloader_timer_ms = get_absolute_time();
        }

        // if current time - bootloader_timer_ms > 3 seconds, enter bootloader
        if (absolute_time_diff_us(bootloader_timer_ms,get_absolute_time()) > BOOTLOADER_BUTTON_TIME * 1E06)
        {
            // ENTER BOOTLOADER
            display.fill(0);
            display.set_cursor(0,OLED_HEIGHT/2);
            display.print("----ENTERING BOOTLOADER----");
            display.render();
            sleep_ms(100);
            reset_usb_boot(0,0);
        }

        // If left button has been pressed for DISPLAY_RESET_BUTTON_TIME
        // TODO: fix this to use correct timer
        if(absolute_time_diff_us(bootloader_timer_ms,get_absolute_time()) > DISPLAY_RESET_BUTTON_TIME * 1E06)
        {
            // Reset display
            left_button_timer = get_absolute_time();
            display.oled_init();
        }


        // If left PB was just pressed, change display state
        if (!pb_left_state && pb_left_prev_state)
        {
            // Change display state only not in ScreensaverState
            if(display_state != ScreensaverState)
            {
                // Wrap around display state
                display_state = (state)((display_state + LENGTH_USER_STATES - 1) % LENGTH_USER_STATES);
                // nv_settings.data.disp_brightness -= 16;
                // display.set_brightness(nv_settings.data.disp_brightness);
                // nv_settings.store_data();
            }
            else // if in screensaver state, reset screensaver timer and exit to last display state
            {
                display_state = last_display_state;
                screen_saver_timer_100ms = 0;
            }
        }

        // If right PB was just pressed, change display state
        if (!pb_right_state && pb_right_prev_state)
        {
            if(display_state != ScreensaverState)
            {
                display_state = (state)((display_state + 1) % LENGTH_USER_STATES);
                // nv_settings.data.disp_brightness += 16;
                // display.set_brightness(nv_settings.data.disp_brightness);
                // nv_settings.store_data();
            }
            else // if in screensaver state, reset screensaver timer and exit to last display state
            {
                display_state = last_display_state;
                screen_saver_timer_100ms = 0;
            }
        } // End of button reads

        // Exit screensaver mode if speed is not 0 and screensaverstate is on
        if (data_pt.speed_kph != 0 && display_state == ScreensaverState)
        {
            display_state = last_display_state;
            screen_saver_timer_100ms = 0;
        }


        // Other calculations
        // Speed calculation

        // Start display stuff
        display.fill(0);
        //display.set_font(press_start_2p); // reset to the small font
        display.set_font(too_simple); // reset to the small font

        if(odometer_nvflash_flag == 2) // If odometer is about to be flashed
            // Print out that the odometer is flashing centered in X
            display.print_centered_x("ODOMETER FLASHED",OLED_HEIGHT/2-1); // you shouldn't normally see this text, mostly for debug in case it flashes too often


        mutex_enter_blocking(&float_mutex);

        // Update rolling averages
        b_cur_avg = (1 - ROLLING_AVG_RATIO) * b_cur_avg + ROLLING_AVG_RATIO * data_pt.current_in;
        b_volts_avg = (1 - ROLLING_AVG_RATIO) * b_volts_avg + ROLLING_AVG_RATIO * data_pt.v_in;

        // for now hardcode 13S battery Full =54.6V empty = 39V (3.0/cell) (delta V = 15.6)
        b_soc = MIN(((b_volts_avg - 39.0) * 100 / 15.6), 100); // Get scale from min to max batt V

        #ifdef DEBUG

        // Set to Debug state
        //display_state = DebugState;

        //DEBUG: use buttons to change speed
        if(pb_left_state == 0)
            data_pt.rpm -= 600.0;
        if(pb_right_state == 0)
            data_pt.rpm += 600.0;
        
        // Process speed in debug because if vesc is not connected (because you are debugging), it will not process speed
        process_speed_calc();

        #else
        m_erpm_avg = (1 - ROLLING_AVG_RATIO) * m_erpm_avg + ROLLING_AVG_RATIO * abs(data_pt.rpm);
        #endif


        ////////////////////////////////////////////////////////////////////////////////////////////////
        switch (display_state)
        {
        case DefaultState:
            {
            // TODO:
            // minimalist display with big font and ez to read
            /*
            BATTERY ICON
            TEMP
            SPEED
            POWER
            */


            draw_battery_icon();

            // Draw small text stuff
            // // TODO: Display throttle intensity visually (ADC1) next to regen (ADC2)
            #define THROTTLE_BAR_WIDTH 5
            #define THROTTLE_BAR_HEIGHT 20


            display.set_cursor(OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH*2, 10);
            display.print("T\nH\nR\nO\nT");

            display.set_cursor(OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH + 1, 10);
            display.print("R\nE\nG\nE\nN");

            // Draw Throttle bar
            display.draw_vbar(data_pt.adc1_decoded*100, OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH*2, OLED_HEIGHT - 1 - THROTTLE_BAR_HEIGHT, OLED_WIDTH - 1-THROTTLE_BAR_WIDTH, OLED_HEIGHT - 1);
            // Draw Regen Bar
            display.draw_vbar(data_pt.adc2_decoded*100, OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH, OLED_HEIGHT - 1 - THROTTLE_BAR_HEIGHT, OLED_WIDTH - 1, OLED_HEIGHT - 1);


            // TODO: Display FET/MOTOR temperature and bar graph and change max temp to non hardcode
            
            #define TEMP_BAR_X 70       // x position of temp bar graph
            #define TEMP_BAR_HEIGHT 2   // height of temp bar graph
            #define FET_TEMP_Y 0        // y position of FET temp bar graph
            #define MOTOR_TEMP_Y (FET_TEMP_Y + TEMP_BAR_HEIGHT) // overlap one border pixel

            // Combine FET and Motor temp into one bar graph
            #define TEMP_TEXT_X 24
            display.set_cursor(TEMP_TEXT_X, 0);

            // Print out Fet and flash it if greater than 100C
            if(data_pt.temp_mos >= 100.0)
            {
                if(display_normal_flashing_flag)
                {
                    display.print_num("F: %2.0fC", data_pt.temp_mos);
                }
            }
            else
            {
                display.print_num("F: %2.0fC", data_pt.temp_mos);
            }

            // Print out Motor temp and flash it if greater than 100C
            if(data_pt.temp_motor >= 100.0)
            {
                if(display_normal_flashing_flag)
                {
                    display.print_num(" M: %3.0fC", data_pt.temp_motor);
                }
            }
            else
            {
                display.print_num(" M: %3.0fC", data_pt.temp_motor);
            }

            // Draw FET and Motor temp bar graphs
            // Flash the bar graph if equal to or greater than 100C
            if(data_pt.temp_mos >= 100.0)
            {
                if(display_normal_flashing_flag)
                {
                    display.draw_hbar(data_pt.temp_mos/110.0 * 100.0,0, TEMP_BAR_X,FET_TEMP_Y,110,TEMP_BAR_HEIGHT);
                }
            }
            else
            {
                display.draw_hbar(data_pt.temp_mos/110.0 * 100.0,0, TEMP_BAR_X,FET_TEMP_Y,110,TEMP_BAR_HEIGHT);
            }

            if(data_pt.temp_motor >= 100.0) // TESTING: always flash motor temp
            {
                if(display_normal_flashing_flag)
                {
                    display.draw_hbar(data_pt.temp_motor/110.0 * 100.0,0, TEMP_BAR_X,MOTOR_TEMP_Y,110,MOTOR_TEMP_Y + TEMP_BAR_HEIGHT);
                }
            }
            else
            {
                display.draw_hbar(data_pt.temp_motor/110.0 * 100.0,0, TEMP_BAR_X,MOTOR_TEMP_Y,110,MOTOR_TEMP_Y + TEMP_BAR_HEIGHT);
            }

            // Display odometer
            display.set_cursor(TEMP_TEXT_X, MOTOR_TEMP_Y + display.get_font_height());
            display.print_num("ODO: %2.1f km", data_pt.odometer);

            // Display trip odometer
            display.set_cursor(TEMP_TEXT_X, MOTOR_TEMP_Y + display.get_font_height()*2);
            display.print_num("TRIP: %2.1f km", trip_odometer);

            // Display whr/km next to trip odometer
            display.get_str_dimensions("TRIP: %2.1f km", &temp_str_x, &temp_str_y);
            display.set_cursor(TEMP_TEXT_X + temp_str_x, MOTOR_TEMP_Y + display.get_font_height()*2);
            display.print_num("%2.1f whr/km", data_pt.watt_hours/trip_odometer);
            


            // Draw big text stuff
            // KPH = ERPM / Pole Pairs * wheel diameter(mm)/1000000 * PI * 60 min/hour
            //
            // TODO: lots of temp stuff. Need to get big font for KPH and W letters

            display.set_font(future_real);

            sprintf(temp_str, "%.0f", data_pt.speed_kph);
            display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);



            // Display Speed text
            display.set_cursor(OLED_WIDTH/2 - 1 - (temp_str_x/2), 20);
            display.print(temp_str);

            // Display Power on next row
            sprintf(temp_str, "%.0f", b_cur_avg * b_volts_avg);
            display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);
            display.set_cursor(OLED_WIDTH/2 - 1 - (temp_str_x/2), 20 + display.get_font_height() + 1);
            display.print(temp_str);



           // draw temp
           // draw speed
           // draw power
           //

            break;
            }
        case AnalogSpeedState:
            {// Display battery voltage visually and numerically
            // TODO: Get max Batt V from vesc for auto ranging soc value

            #define BATT_TERMINAL_TOP_LEFT_X 4
            #define BATT_TERMINAL_TOP_LEFT_Y 15
            #define BATT_TERMINAL_WIDTH 8
            #define BATT_TERMINAL_HEIGHT 3

            display.draw_vbar(b_soc, 0, 18, 15, OLED_HEIGHT - 1); // Battery icon outline
            display.fill_rect(0, BATT_TERMINAL_TOP_LEFT_X, BATT_TERMINAL_TOP_LEFT_Y, BATT_TERMINAL_TOP_LEFT_X + BATT_TERMINAL_WIDTH, BATT_TERMINAL_TOP_LEFT_Y + BATT_TERMINAL_HEIGHT); // Draw block to represent battery terminal

            // Batt Voltage and Current text
            display.set_cursor(2, 0);
            display.print_num("%.1fV", b_volts_avg);
            display.set_cursor(2,display.get_font_height());
            display.print_num("%.0fA", b_cur_avg);

            // Display Speed on gauge

            speed_gauge.set_value(data_pt.speed_kph);
            speed_gauge.draw();
            sprintf(temp_str, "%.1f KPH", data_pt.speed_kph);
            display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);


            // Blank out underneath text (1 pixel bigger than text box)
            display.fill_rect(1, OLED_WIDTH/2 - 1 - (temp_str_x/2) - 1, TEXT_UNDER_SPEEDO_Y_START, OLED_WIDTH/2 - 1 + (temp_str_x/2), TEXT_UNDER_SPEEDO_Y_START + 2*temp_str_y+1);

            // Display Speed text
            display.set_cursor(OLED_WIDTH/2 - 1 - (temp_str_x/2), TEXT_UNDER_SPEEDO_Y_START);
            display.print(temp_str);

            // Display Power
            sprintf(temp_str, "%.0fW", b_cur_avg * b_volts_avg);
            display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);
            display.set_cursor(OLED_WIDTH/2 - 1 - (temp_str_x/2), TEXT_UNDER_SPEEDO_Y_START + display.get_font_height() + 1);
            display.print(temp_str);

            // Display throttle intensity visually (ADC1) next to regen (ADC2)
            #define THROTTLE_BAR_WIDTH 5
            #define THROTTLE_BAR_HEIGHT 20


            display.set_cursor(OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH*2, 10);
            display.print("T\nH\nR\nO\nT");

            display.set_cursor(OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH + 1, 10);
            display.print("R\nE\nG\nE\nN");

            // Draw Throttle bar
            display.draw_vbar(data_pt.adc1_decoded*100, OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH*2, OLED_HEIGHT - 1 - THROTTLE_BAR_HEIGHT, OLED_WIDTH - 1-THROTTLE_BAR_WIDTH, OLED_HEIGHT - 1);
            // Draw Regen Bar
            display.draw_vbar(data_pt.adc2_decoded*100, OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH, OLED_HEIGHT - 1 - THROTTLE_BAR_HEIGHT, OLED_WIDTH - 1, OLED_HEIGHT - 1);


            // TODO: Display FET/MOTOR temperature and bar graph and change max temp to non hardcode
            #define TEMP_BAR_X 70
            #define TEMP_BAR_HEIGHT 5
            #define FET_TEMP_Y 0
            #define MOTOR_TEMP_Y (FET_TEMP_Y + 5 + 1)

            display.set_cursor(24, 0);
            display.print_num("FETS: %2.0fC", data_pt.temp_mos);
            display.draw_hbar(data_pt.temp_mos/110.0 * 100.0,0, TEMP_BAR_X,0,110,TEMP_BAR_HEIGHT);


            display.set_cursor(24, MOTOR_TEMP_Y);
            display.print_num("MOT: %2.0fC", data_pt.temp_motor);
            display.draw_hbar(data_pt.temp_motor/110.0 * 100.0,0, TEMP_BAR_X,MOTOR_TEMP_Y,110,MOTOR_TEMP_Y + TEMP_BAR_HEIGHT);

            // TODO: fps counter
            // every second count how many frames
            // fps += 1;
            // if(absolute_time_diff_us(get_absolute_time(),next_fps_count) >= 0)
            // {
            //     // show fps in top right corner
            //     display.set_cursor(OLED_WIDTH - 10,0);
            //     display.print_num("%2.0", fps);
            //     delayed_by_ms(next_fps_count,1000); // set next fps count timer
            //     fps = 0; //
            // }
            break;
            }

        case DebugState:
            {// Do debug stuff


            //process_speed_calc();
            display.set_cursor(0,0);

            // Print DEBUG info
            display.print_num("KPH:%f\n",data_pt.speed_kph);
            display.print_num("ERPM:%f\n",data_pt.rpm);
            display.print_num("ODO:%f\n",data_pt.odometer);
            display.print_num("PREV_KPH:%f\n",prev_kph);
            display.print_num("AVG_SPEED:%f\n",average_speed);
            display.print_num("TIME_BTW_ODO_US:%d\n",(int32_t)time_between_odo_samples_us);
            display.print_num("DISTANCE_TRV:%f\n",distance_travelled);

            break;
            }
        case StatsState:
            {
            // TODO: stats stuff
            // show odometer, whr/km, whr used, whr charged,
            /* todo: stats
            - tot whr/km
            -average speed
            -max speed
            -battery IR
            
            */
            display.set_cursor(0,0);
            display.print_num("ODO:%.3f\n",data_pt.odometer);
            // add in total watt hours used and charged and whr/km
            display.print_num("WHR/KM:%.2f\n",(data_pt.total_watt_hours_used-data_pt.total_watt_hours_charged)/data_pt.odometer);
            display.print_num("WHR CHARGED:%.1f\n",data_pt.total_watt_hours_charged);

            // Print out money saved @ 13L/100km on car and 1.90/L hardcoded for now
            display.print_num("GAS COST:$%.2f\n", (float)(data_pt.odometer/(float)(100.0) * 13 * (float)(1.90)));

            // Trip stats
            display.print("TRIP TIME:");
            get_time_str(temp_str);
            display.print(temp_str);
            display.print("\n\r");

            display.print_num("TRIP ODO:%.3f\n",trip_odometer);
            display.print_num("TRIP WHR/KM:%.2f\n",data_pt.watt_hours/trip_odometer);
            display.print_num("TRIP WHR USED:%.1f\n",data_pt.watt_hours);
            display.print_num("TRIP WHR CHARGED:%.1f\n",data_pt.watt_hours_charged);

            break;
            }


        case ScreensaverState:
            {
            // Do screensaver stuff
            // Create a mode with minimal information that moves around to avoid burn in
            // show battery volatage and cell voltage with a bar graph
            // every SCREENSAVER_MOVE_INTERVAL_100MS - change positions of things

            
            display.set_font(future_real); // set to big font

            // Get height of both lines of text and bar graph
            #define BATT_BAR_WIDTH 60
            #define BATT_BAR_HEIGHT 20

            uint8_t bounding_box_height = BATT_BAR_HEIGHT;
            uint8_t bounding_box_width = BATT_BAR_WIDTH;

            // Display batt voltage
            sprintf(temp_str, "%.1f", data_pt.v_in);
            display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);
            bounding_box_height += temp_str_y;
            // set width to the max of the two
            bounding_box_width = (temp_str_x > bounding_box_width) ? temp_str_x : bounding_box_width;

            // Change to small font
            display.set_font(too_simple);
            // Calculate Cell voltage
            sprintf(temp_str, "%.2f", data_pt.v_in / cells_in_series);
            display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);
            bounding_box_height += temp_str_y;
            // set width to the max of the two
            bounding_box_width = (temp_str_x > bounding_box_width) ? temp_str_x : bounding_box_width;

    
            static uint8_t bounding_box_x = 0;
            static uint8_t bounding_box_y = 0;
            static uint8_t bounding_box_x_dir = 1; // 1 = right, 0 = left
            static uint8_t bounding_box_y_dir = 1; // 1 = down, 0 = up
            static uint8_t move_screensaver_flag = 0;


            // Reuse screen_saver_timer_100ms for the move interval timer
            // Move Bounding box around every SCREENSAVER_MOVE_INTERVAL_100MS
            // set move_screensaver_flag to 1 to move after the interval

            // Set the move_screensaver_flag only when the timer is at the interval but not every frame during that time
            
            // interval is up, set flag to move but only if it's not already set previously this interval
            if(screen_saver_timer_100ms % SCREENSAVER_MOVE_INTERVAL_100MS == 0 && move_screensaver_flag != 2)
            {
                move_screensaver_flag = 1;
            }
            else if(screen_saver_timer_100ms % SCREENSAVER_MOVE_INTERVAL_100MS != 0)
            {
                move_screensaver_flag = 0;
            }

            if(move_screensaver_flag == 1)
            {
                move_screensaver_flag = 2;
                // Move the bounding box around
                if(bounding_box_x_dir)
                {
                    bounding_box_x++;
                    if(bounding_box_x >= (OLED_WIDTH - 1 - bounding_box_width))
                    {
                        bounding_box_x_dir = 0;
                    }
                }
                else
                {
                    bounding_box_x--;
                    if(bounding_box_x <= 0)
                    {
                        bounding_box_x_dir = 1;
                    }
                }

                if(bounding_box_y_dir)
                {
                    bounding_box_y++;
                    if(bounding_box_y >= (OLED_HEIGHT - 1 - bounding_box_height))
                    {
                        bounding_box_y_dir = 0;
                    }
                }
                else
                {
                    bounding_box_y--;
                    if(bounding_box_y <= 0)
                    {
                        bounding_box_y_dir = 1;
                    }
                }
            }

            // Draw the bounding box
            display.draw_box(bounding_box_x, bounding_box_y, bounding_box_x + bounding_box_width, bounding_box_y + bounding_box_height);

            // Draw the batt and cell voltage text
            // TODO: center the text in the bounding box and do get_str_dimensions in a nicer way
            uint8_t bounding_box_text_y = bounding_box_y; // var for keeping text y cursor position

            display.set_font(future_real); // set to big font
            sprintf(temp_str, "%.1f", data_pt.v_in);
            display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);
            display.set_cursor(bounding_box_x + ((bounding_box_width - temp_str_x) / 2), bounding_box_text_y);
            display.print_num(temp_str, data_pt.v_in);
            bounding_box_text_y += temp_str_y; // move the text cursor down for next line

            // Change to small font
            display.set_font(too_simple);
            sprintf(temp_str, "%.2f CELL V", data_pt.v_in / cells_in_series);
            display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);

            display.set_cursor(bounding_box_x + ((bounding_box_width - temp_str_x) / 2), bounding_box_text_y);
            display.print_num(temp_str, data_pt.v_in / cells_in_series);
            bounding_box_text_y += temp_str_y;  // move the text cursor down for next line

            // Draw the bar graph after the lines of text
            display.draw_hbar(b_soc,0,bounding_box_x,bounding_box_text_y,bounding_box_x + BATT_BAR_WIDTH,bounding_box_text_y + BATT_BAR_HEIGHT);
            }
            break;
        }

        mutex_exit(&float_mutex);
        display.render();
    }

    // while(true)
    // {
    //     display.fill(0);

    //     display.set_cursor(0,0);


    //     mutex_enter_blocking(&float_mutex);


    //     // display.print_num(" VIN: %.1fV\n", data_pt.v_in);
    //     // display.print_num("TFET: %.1fC\n", data_pt.temp_mos);
    //     // display.print_num("TMOT: %.1fC\n", data_pt.temp_motor);
    //     // display.print_num("ERPM: %d\n", data_pt.rpm);
    //     //display.print_num("ADC1_RAW: %.1fV\n", adc_v1);
    //     //display.print_num("ADC1_DEC: %.3f\n", data_pt.adc1_decoded);
    //     //display.print_num("ADC2_RAW: %.1fV\n", adc_v2);
    //     //display.print_num("ADC2_DEC: %.3f\n", data_pt.adc2_decoded);

    //     display.draw_vbar(data_pt.adc1_decoded*100, 4, 20, 8, 60);

    //     display.draw_hbar(data_pt.adc1_decoded*100, 1, 20, 20, 60, 24);
    //     display.draw_hbar(data_pt.adc1_decoded*100, 0, 70, 20, 110, 24);

    //     display.print_num("BRIGHTNESS: %d\n", nv_settings.data.disp_brightness);
    //     display.print_num("FLASH PG=%d", nv_settings.page_id);
    //     display.print_num("    BLOCK=%d\n\r", nv_settings.block_id);
    //     display.print(vesc_connected ? "VESC CONNECTED" : "VESC NOT CONNECTED");

    //     // char hex_str[10];
    //     // uint8_t cols = 0;
    //     // for (uint8_t idx = 0; idx < 20; idx++)
    //     // {
    //     //     if (cols++ == 4)
    //     //     {
    //     //         cols = 0;
    //     //         display.print("\n");
    //     //     }

    //     //     sprintf(hex_str, "%.2X ", get_values_response[idx]);
    //     //     display.print(hex_str);
    //     // }

    //     mutex_exit(&float_mutex);

    //     // Process inputs
    //     pb_left_state = debouncer.read(PB_LEFT_GPIO);
    //     pb_right_state = debouncer.read(PB_RIGHT_GPIO);

    //     // Draw the state of the buttons
    //     if (pb_left_state)
    //         display.fill_rect(0, 80, 30, 100, 50);
    //     else
    //         display.draw_box(80, 30, 100, 50);

    //     if (pb_right_state)
    //         display.fill_rect(0, 105, 30, 125, 50);
    //     else
    //         display.draw_box(105, 30, 125, 50);


    //     pb_left_prev_state = pb_left_state;
    //     pb_right_prev_state = pb_right_state;


    //     draw_SD_status(118,0);

    //     display.render();

    // }
}

void draw_battery_icon()
{


    display.draw_vbar(b_soc, 0, 18, 15, OLED_HEIGHT - 1); // Battery icon outline
    display.fill_rect(0, BATT_TERMINAL_TOP_LEFT_X, BATT_TERMINAL_TOP_LEFT_Y, BATT_TERMINAL_TOP_LEFT_X + BATT_TERMINAL_WIDTH, BATT_TERMINAL_TOP_LEFT_Y + BATT_TERMINAL_HEIGHT); // Draw block to represent battery terminal

    // Batt Voltage and Current text
    display.set_cursor(2, 0);
    if(display_slow_flashing_flag)
    {
        // Display cell voltage (battery voltage / number of cells)
        display.print_num("%.1fV", b_volts_avg/cells_in_series);
    }
    else
    {
        // Print out battery voltage
        display.print_num("%.1fV", b_volts_avg);
    }

    display.set_cursor(2,display.get_font_height());
    display.print_num("%.0fA", b_cur_avg);
}


void uart0_write(uint8_t * src, size_t len)
{
    uart_write_blocking(uart0, src, len);
}


uint8_t receive_packet(PACKET_STATE_t *rx_packet)
{
    // Wait up to 10ms for the initial response
    if (uart_is_readable_within_us(uart0, 10000))
    {
        // Read each byte of the response until there is no more data sent for 30 bits worth of time
        while(uart_is_readable_within_us(uart0, (1e6*30)/UART_BAUDRATE))
        {
            packet_process_byte(uart_getc(uart0), rx_packet);
        }

        return 1;
    }
    else
    {
        return 0;
    }
}

void process_data(uint8_t *data, size_t len);

// Get absolute time from pico and convert to str containing hh:mm:ss for trip time
// Use Buffer of at least [15]
void get_time_str(char * time_str)
{
    // Get time from 
    uint64_t time_s = to_us_since_boot(get_absolute_time()) / 1000000;
    // Convert to hours mins seconds
    uint16_t hours = time_s/3600;
    uint8_t leftover_secs = time_s % 3600;
    uint8_t mins = leftover_secs / 60;
    uint8_t secs = leftover_secs % 60;
    // Store in time_str
    sprintf(time_str, "%02d:%02d:%02d", hours, mins, secs);

}


// Do the speed and odometer calculation here
void process_speed_calc()
{
    absolute_time_t current_time = get_absolute_time();
    time_between_odo_samples_us = absolute_time_diff_us(prev_time_sample,current_time);
    prev_time_sample = current_time; // update prev_time_sample to current time

    // calculate speed and odometer
    data_pt.speed_kph = float(data_pt.rpm/23.0 * 660/1000000 * 3.1415 * 60);
    // average speed * time travelled * (kph * us to km conversion)
    average_speed = abs((prev_kph + data_pt.speed_kph))/2;
    distance_travelled = average_speed * time_between_odo_samples_us/(3600); // distance travelled as mm
    data_pt.odometer += distance_travelled/1000000.0; // convert distance_travelled to km and add to odo
    // double check proper math is done and not truncated due to data types

    // Update total whr/km
    
    // Update trip odometer
    trip_odometer += distance_travelled/1000000.0;

    // Update prev_kph to current speed
    prev_kph = data_pt.speed_kph;
}


// Second core thread. Handles UART communication
void core1_entry()
{
    const uint BUILTIN_LED_PIN = PICO_DEFAULT_LED_PIN;
    // uint8_t LED_STATUS = 1;

    // Set the LED pin as an output
    gpio_init(BUILTIN_LED_PIN);
    gpio_set_dir(BUILTIN_LED_PIN, GPIO_OUT);

    // Set up debug output
    gpio_init(DEBUG_GPIO);
    gpio_set_dir(DEBUG_GPIO, GPIO_OUT);

    // Initialize UART0 and it's GPIO pins
    real_baudrate = uart_init(uart0, UART_BAUDRATE);
    gpio_set_function(UART_TX_GPIO, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_GPIO, GPIO_FUNC_UART);

    PACKET_STATE_t vesc_comm;
    uint8_t send_payload[50];
    int32_t send_pl_idx = 0;
    absolute_time_t next_sample, sample_time;

    /* Handle any config reads first */
    // Build Packet for config reads
    /* TODO: Get
        wheel diameter
        motor poles
        battery voltage
        battery AH
        gear ratio
        fet temp limit start/end
        motor temp limit start/end
    */

    next_sample = get_absolute_time();
    while (true)
    {
        mutex_enter_blocking(&flash_lock);  // Don't allow flash erase/write while running core1 code

        sleep_until(next_sample);
        next_sample = delayed_by_ms(next_sample, 1000 / LOG_SAMPLE_RATE);

        // Blink LED to show that communication is happening
        // gpio_put(LED_PIN, 1);
        // sleep_ms(50);
        // gpio_put(LED_PIN, 0);

        // Prepare COMM_GET_VALUES_SELECTIVE packet
        packet_init(uart0_write, process_data, &vesc_comm);
        memset(send_payload, 0, sizeof(send_payload));
        send_pl_idx = 0;

        // Payload: packet ID, value mask
        send_payload[send_pl_idx++] = COMM_GET_VALUES_SELECTIVE;
        static uint32_t get_values_mask = 0xFFFFFFFF;
        buffer_append_uint32(send_payload, get_values_mask, &send_pl_idx);

        packet_send_packet(send_payload, send_pl_idx, &vesc_comm);

        // Record time that data was sampled
        sample_time = get_absolute_time();

        // Reset packet that was sent so it can be reused
        packet_reset(&vesc_comm);


        // Wait for the response packet and process it
        vesc_connected = receive_packet(&vesc_comm);


        // Prepare COMM_GET_DECODED_ADC packet
        packet_init(uart0_write, process_data, &vesc_comm);
        memset(send_payload, 0, sizeof(send_payload));
        send_pl_idx = 0;

        // Payload: packet ID
        send_payload[send_pl_idx++] = COMM_GET_DECODED_ADC;
        packet_send_packet(send_payload, send_pl_idx, &vesc_comm);

        // Reset packet that was sent so it can be reused
        packet_reset(&vesc_comm);

        // Wait for the response packet and process it
        receive_packet(&vesc_comm);


        ///////////// Logging /////////////

        FRESULT result;

        // If SS == 1 an SD card is connected
        if (sd_status == SD_NOT_PRESENT)    // && gpio_get(SD_SS_GPIO) == 1)
        {
            DBG_PRINT("\033[2J\033[H" "FILESYSTEM INIT\n");
            gpio_put(DEBUG_GPIO, 0);
            result = init_filesystem();
            gpio_put(DEBUG_GPIO, 1);

            DBG_PRINT("init_filesystem() returned with %d: %s\n", result, FRESULT_str(result));
            sleep_ms(1);

            gpio_put(DEBUG_GPIO, 0);
            result = create_log_file();
            gpio_put(DEBUG_GPIO, 1);

            DBG_PRINT("create_log_file() returned with %d: %s\n", result, FRESULT_str(result));
            sleep_ms(1);

        }
        else if(sd_status == SD_PRESENT && vesc_connected)
        {
            gpio_put(DEBUG_GPIO, 0);
            result = append_data_pt();
            gpio_put(DEBUG_GPIO, 1);

            DBG_PRINT("append_data_pt() returned with %d: %s\n", result, FRESULT_str(result));
        }


        mutex_exit(&flash_lock);    // Release lock
    }
}


void process_data(uint8_t *data, size_t len)
{
    // Check packet ID and handle different packets accordingly ***
    int32_t idx = 0;
    uint8_t packet_id = data[idx++];

    switch (packet_id)
    {
        case COMM_GET_VALUES:
        case COMM_GET_VALUES_SELECTIVE:

            // First 4 bytes are get_values_selective 32b mask
            idx += 4;

            mutex_enter_blocking(&float_mutex);

            //memcpy(get_values_response, data+5, len-5);

            data_pt.ms_today = time_us_64() / 1000;

            // Unpack data from VESC response
            data_pt.temp_mos = buffer_get_float16(data, 1e1, &idx);
            data_pt.temp_motor = buffer_get_float16(data, 1e1, &idx);
            data_pt.current_motor = buffer_get_float32(data, 1e2, &idx);
            data_pt.current_in = buffer_get_float32(data, 1e2, &idx);
            data_pt.id = buffer_get_float32(data, 1e2, &idx);
            data_pt.iq = buffer_get_float32(data, 1e2, &idx);
            data_pt.duty_now = buffer_get_float16(data, 1e3, &idx);
            data_pt.rpm = buffer_get_float32(data, 1e0, &idx);
            data_pt.v_in = buffer_get_float16(data, 1e1, &idx);
            data_pt.amp_hours = buffer_get_float32(data, 1e4, &idx);
            data_pt.amp_hours_charged = buffer_get_float32(data, 1e4, &idx);
            data_pt.watt_hours = buffer_get_float32(data, 1e4, &idx);
            data_pt.watt_hours_charged = buffer_get_float32(data, 1e4, &idx);
            data_pt.tachometer = buffer_get_int32(data, &idx);
            data_pt.tachometer_abs = buffer_get_int32(data, &idx);
            data_pt.fault_code = (mc_fault_code)data[idx++];
            data_pt.position = buffer_get_float32(data, 1e6, &idx);
            data_pt.vesc_id = data[idx++];
            data_pt.temp_mos_1 = buffer_get_float16(data, 1e1, &idx);
            data_pt.temp_mos_2 = buffer_get_float16(data, 1e1, &idx);
            data_pt.temp_mos_3 = buffer_get_float16(data, 1e1, &idx);
            data_pt.vd = buffer_get_float32(data, 1e3, &idx);
            data_pt.vq = buffer_get_float32(data, 1e3, &idx);

            // Calculated values
            // Power
            data_pt.p_in = data_pt.current_in * data_pt.v_in;
            // Speed calculation
            process_speed_calc();

            //last byte is int8 status, but data_pt struct has no place for it

            mutex_exit(&float_mutex);
            break;

        case COMM_GET_DECODED_ADC:

            mutex_enter_blocking(&float_mutex);
            data_pt.adc1_decoded = buffer_get_float32(data, 1e6, &idx);
            adc_v1 = buffer_get_float32(data, 1e6, &idx);
            data_pt.adc2_decoded = buffer_get_float32(data, 1e6, &idx);
            adc_v2 = buffer_get_float32(data, 1e6, &idx);
            mutex_exit(&float_mutex);

            break;

    }

}




