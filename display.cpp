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

extern "C" 
{
#include "hw_def.h"
}

#include "pico-oled/font/press_start_2p.h"
#include "pico-oled/font/too_simple.h"
#include "pico-oled/font/Retron2000.h"
#include "pico-oled/font/future_real.h"

//#include "u8g2/cppsrc/U8g2lib.h"
#include <u8g2.h>
#include "u8x8_interface.hpp"

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
- add odometer
- whr/km
- whr used

- add screen saver


*/

// Comment out debug when not using
#define DEBUG

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

// function prototypes
void draw_battery_icon();

void core1_entry();


u8g2_t u8g2;


uint32_t real_baudrate = 0;
uint8_t response_code = 0;
uint8_t vesc_connected = 0;

uint8_t get_values_response[100];

float adc_v1, adc_v2;
float b_cur_avg = 0;
float b_volts_avg = 0;
float m_erpm_avg = 0;
float prev_kph_for_odometer = 0;
float fps = 0;
float average_speed = 0;
float distance_travelled = 0;
float prev_kph = 0;
int64_t time_us = 0;
uint8_t b_soc = 0;

uint8_t pb_left_state, pb_left_prev_state;
uint8_t pb_right_state, pb_right_prev_state;

auto_init_mutex(float_mutex);

pico_oled display(OLED_SSD1309, /*i2c_address=*/ 0x3C, /*screen_width=*/ 128, /*screen_height=*/ 64, /*reset_gpio=*/ 15); 

Debounce debouncer;




// Core 0
int main()
{

    stdio_init_all();
    time_init();    

    initialize_gpio();

    // Instantiate debouncer and configure GPIO

    debouncer.debounce_gpio(PB_LEFT_GPIO);
    debouncer.debounce_gpio(PB_RIGHT_GPIO);
    debouncer.debounce_gpio(PB_CENTER_GPIO);

    debouncer.set_debounce_time(PB_LEFT_GPIO, 20.0);    // 20ms debounce time
    debouncer.set_debounce_time(PB_RIGHT_GPIO, 20.0);
    debouncer.set_debounce_time(PB_CENTER_GPIO, 20.0);    


    // Set initial values of button states
    pb_left_prev_state = pb_left_state = debouncer.read(PB_LEFT_GPIO);
    pb_right_prev_state = pb_right_state = debouncer.read(PB_RIGHT_GPIO);
    

    // U8G2 init
    u8g2_Setup_st75256_jlx256128_f(&u8g2, U8G2_R0, u8x8_byte_3wire_sw_spi, u8x8_gpio_and_delay_pico);
    u8g2_InitDisplay(&u8g2);    // Init sequence, ends with display in sleep mode
    u8g2_SetPowerSave(&u8g2, 0);

    u8g2_ClearBuffer(&u8g2);
    u8g2_ClearDisplay(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_t0_11_te);
    u8g2_SetDrawColor(&u8g2, 1);

    // oled init
    // display.oled_init();
    // display.set_brightness(nv_settings.data.disp_brightness);   // set brightness from saved flash settings

    // display.set_font(too_simple);
    //display.set_font(press_start_2p);

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
    #define MAX_KPH 60
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
    absolute_time_t last_odometer_count = current_time_ms;
    absolute_time_t prev_time_sample = current_time_ms;

    int64_t time_between_odometer_check_ms = 0;

    float odometer = nv_settings.data.odometer; // todo: change to use data_pt.odometer when code is ready
    

   
    enum state {DefaultState, AnalogSpeedState,DebugState};
    state display_state = DefaultState;

    while(true)
    {
        // Limit framerate to OLED_FRAMERATE
        sleep_until(next_frame_time);
        current_time_ms = get_absolute_time();
        next_frame_time = delayed_by_ms(next_frame_time, 1000 / OLED_FRAMERATE);


        // TODO: Button reads. 
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
        

        // If left PB was just pressed, decrease brightness
        if (!pb_left_state && pb_left_prev_state)
        {
            nv_settings.data.disp_brightness -= 16;
            display.set_brightness(nv_settings.data.disp_brightness);
            nv_settings.store_data();
        }

        // If right PB was just pressed, increase brightness
        if (!pb_right_state && pb_right_prev_state)
        {
            nv_settings.data.disp_brightness += 16;
            display.set_brightness(nv_settings.data.disp_brightness);       
            nv_settings.store_data();                 
        }


        // Other calculations 
        // Speed calculation
        // core 1 should be doing speed calculations
        // // KPH = ERPM / Pole Pairs * wheel diameter(mm)/1000000 * PI * 60 min/hour
        // kph = float(m_erpm_avg/23 * 660/1000000 * 3.1415 * 60);

        // // Odometer calculation
        // // Distance = avg Speed * time
        // static float average_speed = 0;
        // static float time_hours = 0;
        // static float distance_travelled = 0;
        // // if time has been at least ODOMETER_UPDATE_INTERVAL_MS, calculate distance traveled and add to odometer
        // time_between_odometer_check_ms = absolute_time_diff_us(last_odometer_count,current_time_ms)/1000;

        // if (time_between_odometer_check_ms >= ODOMETER_UPDATE_INTERVAL_MS)
        // {
        //     average_speed = abs((kph + prev_kph_for_odometer))/2;
        //     time_hours = time_between_odometer_check_ms/3600.0/1000.0;// convert kph * ms to km (odo)
        //     distance_travelled = average_speed * time_hours;
        //     odometer += distance_travelled;
        //     prev_kph_for_odometer = kph;
        //     last_odometer_count = current_time_ms;
        // }
        
        

    
        // Start display stuff
        display.fill(0);

        mutex_enter_blocking(&float_mutex);

        // Update rolling averages
        b_cur_avg = (1 - ROLLING_AVG_RATIO) * b_cur_avg + ROLLING_AVG_RATIO * data_pt.current_in;
        b_volts_avg = (1 - ROLLING_AVG_RATIO) * b_volts_avg + ROLLING_AVG_RATIO * data_pt.v_in;

        // for now hardcode 13S battery Full =54.6V empty = 39V (3.0/cell) (delta V = 15.6)
        b_soc = MIN(((b_volts_avg - 39.0) * 100 / 15.6), 100); // Get scale from min to max batt V

        #ifdef DEBUG

        // Set to Debug state
        display_state = DebugState;

        //DEBUG: use buttons to change speed
        if(pb_left_state == 0)
            data_pt.rpm -= 600.0;   
        if(pb_right_state == 0)
            data_pt.rpm += 600.0;
        #else
        m_erpm_avg = (1 - ROLLING_AVG_RATIO) * m_erpm_avg + ROLLING_AVG_RATIO * abs(data_pt.rpm);
        #endif


        ////////////////////////////////////////////////////////////////////////////////////////////////
        switch (display_state)
        {
        case DefaultState:
            {// TODO:
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

            // Display odometer
            display.set_cursor(24, MOTOR_TEMP_Y + TEMP_BAR_HEIGHT + 1);
            display.print_num("ODO: %2.3f km", data_pt.odometer);
            



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

            display.set_font(too_simple); // reset to the small font
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


            // Display watt-hours charged numerically
            // TODO Watt hours used instead
            // display.fill_rect(1, 18, 52, 117, OLED_HEIGHT - 1);  // Blank out area where text will draw
            // display.set_cursor(19, 54);
            // display.print_num("WATT-HRS GENERATED: %.1f", data_pt.watt_hours_charged);

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
            break;}

        case DebugState:
            {// Do debug stuff

            // do debug data here instead of getting from vesc
            data_pt.speed_kph = float(data_pt.rpm/23.0 * 660/1000000 * 3.1415 * 60);
            // average speed * time travelled * (kph * us to km conversion)
            average_speed = (prev_kph + data_pt.speed_kph)/2;
            time_us = absolute_time_diff_us(prev_time_sample,get_absolute_time());
            distance_travelled = (double)(average_speed * time_us/(3600)); // distance travelled as mm
            data_pt.odometer = (double)(data_pt.odometer +  distance_travelled/1000000.0); // convert distance_travelled to km and add to odo
            // double check proper math is done and not truncated due to data types

            prev_kph = data_pt.speed_kph;
            prev_time_sample = get_absolute_time();

            display.set_cursor(0,0);
            char debug_string[200];
            sprintf(debug_string,"odometer:%f\n\
            kph:%f\n\
            erpm:%f\n\
            prev_kph:%f\n\
            average_speed:%f\n\
            time_us:%d\n\
            distance_trv:%f\n\
            ",data_pt.odometer,data_pt.speed_kph,data_pt.rpm,prev_kph,average_speed,time_us,distance_travelled);
            display.print(debug_string);

        default:
            break;}
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
    display.print_num("%.1fV", b_volts_avg);
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


// Second core thread. Handles UART communication
void core1_entry()
{
    const uint BUILTIN_LED_PIN = PICO_DEFAULT_LED_PIN;
    // uint8_t LED_STATUS = 1;




    // Initialize UART0 
    real_baudrate = uart_init(uart0, UART_BAUDRATE);

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
    static float prev_kph = 0;
    static absolute_time_t prev_time_sample = get_absolute_time();


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
            // TODO: change hardcode wheel diameter and get from vesc
            // KPH = ERPM / Pole Pairs * wheel diameter(mm)/1000000 * PI * 60 min/hour

            data_pt.speed_kph = float(data_pt.rpm/23.0 * 660/1000000 * 3.1415 * 60);
            
            // do debug data here instead of getting from vesc
            data_pt.speed_kph = float(data_pt.rpm/23.0 * 660/1000000 * 3.1415 * 60);
            // average speed * time travelled * (kph * us to km conversion)
            average_speed = (prev_kph + data_pt.speed_kph)/2;
            time_us = absolute_time_diff_us(prev_time_sample,get_absolute_time());
            distance_travelled = (double)(average_speed * time_us/(3600)); // distance travelled as mm
            data_pt.odometer = (double)(data_pt.odometer +  distance_travelled/1000000.0); // convert distance_travelled to km and add to odo
            // double check proper math is done and not truncated due to data types

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




