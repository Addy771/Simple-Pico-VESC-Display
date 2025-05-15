#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "pico/cyw43_arch.h"

#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/spi.h"
#include "hardware/rtc.h"
#include "hardware/irq.h"

#include "nv_flash.hpp"
#include "log.hpp"

#include "datatypes.h"
#include "packet.h"
#include "buffer.h"
#include "crc.h"

#include "util_enum.h"
#include <string.h>
#include "sd_card.h"
#include "ff.h"
#include "f_util.h"
#include "hw_config.h"
#include "rtc.h"
#include "circular_buf.hpp"

extern "C" 
{
#include "hw_def.h"
#include "can2040/src/can2040.h"
}

#include <u8g2.h>
#include "u8x8_interface.hpp"
#include "spi_tx_9bit.pio.h"

#include "page_ctrl.hpp"

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


//#define BOOTLOADER_BUTTON_TIME 0.5     // time in seconds for buttons to be pressed before entering bootloader mode
#define DISPLAY_RESET_BUTTON_TIME 2
#define ODOMETER_UPDATE_INTERVAL_MS 100 // time between odometer updates


// function prototypes
void draw_battery_icon();

void core1_entry();
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg);




static struct can2040 cbus;
volatile circular_buf<can2040_msg, 10> can_msg_buf;

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


absolute_time_t current_time_ms = get_absolute_time();
absolute_time_t bootloader_timer_ms = current_time_ms;
absolute_time_t next_frame_time = current_time_ms;
absolute_time_t left_button_timer = current_time_ms;
absolute_time_t next_fps_count = current_time_ms;
absolute_time_t last_odometer_count = current_time_ms;
absolute_time_t prev_time_sample = current_time_ms;
absolute_time_t buff_write_time;

page_controller page_ctrl;
log_data_t *data_pt;
mutex_t *flash_mutex, *float_mutex;

// Core 0
int main()
{
    stdio_init_all();
    time_init();  
    cyw43_arch_init();  

    initialize_gpio();
    init_external_rtc();


    // U8G2 init
    set_backlight(120);
    u8g2_Setup_st75256_jlx256128_f(&page_ctrl.u8g2, U8G2_R0, U8G2_BYTE_FN, u8x8_gpio_and_delay_pico);
    u8g2_InitDisplay(&page_ctrl.u8g2);    // Init sequence, ends with display in sleep mode
    u8g2_SetContrast(&page_ctrl.u8g2, 170); 
    u8g2_SetPowerSave(&page_ctrl.u8g2, 0);

    u8g2_ClearBuffer(&page_ctrl.u8g2);
    u8g2_ClearDisplay(&page_ctrl.u8g2);
    //u8g2_SetFont(&page_ctrl.u8g2, u8g2_font_t0_11_te);
    u8g2_SetFont(&page_ctrl.u8g2, u8g2_font_10x20_tf);
    u8g2_SetDrawColor(&page_ctrl.u8g2, 1);  

  
    DBG_PRINT("Core 0 init complete.\n");

    // Set up some pointers that core1 needs to access
    data_pt = &page_ctrl.esc_data; 
    flash_mutex = &page_ctrl.flash_mutex;
    float_mutex = &page_ctrl.float_mutex;
    multicore_launch_core1(core1_entry);

    next_frame_time = get_absolute_time();
    while(1)
    {
        sleep_until(next_frame_time);
        next_frame_time = delayed_by_ms(next_frame_time, 1000 / 20);

        page_ctrl.update();
        page_ctrl.draw_page();


    }





    // main() initilizing
    // Display Setup

    // Gauges setup
    // #define MAX_KPH 60
    // #define TEXT_UNDER_SPEEDO_Y_START 32


    // analog_gauge speed_gauge(&display);
    // speed_gauge.set_position(OLED_WIDTH/2 - 1, 120);
    // speed_gauge.set_scale(0,MAX_KPH, 250, 290);
    // speed_gauge.set_markers(6, 106, 12, 1);

    // char temp_str[20];
    // uint8_t temp_str_x, temp_str_y;

    // #define ROLLING_AVG_RATIO 0.3

    // int64_t time_between_odometer_check_ms = 0;

    // float odometer = nv_settings.data.odometer; // todo: change to use data_pt.odometer when code is ready
    

   
    // enum state {DefaultState, AnalogSpeedState,DebugState};
    // state display_state = DefaultState;

    
    // while(true)
    // {
    //     // Limit framerate to OLED_FRAMERATE
    //     sleep_until(next_frame_time);
    //     current_time_ms = get_absolute_time();
    //     next_frame_time = delayed_by_ms(next_frame_time, 1000 / OLED_FRAMERATE);


    //     // TODO: Button reads. 
    //     // Go into bootloader when both buttons pressed for 3 seconds
    //     // check if both buttons are pressed. start timer 
    //     // if button has been pressed again after set time, activate bootloader

    //     pb_right_prev_state = pb_right_state;
    //     pb_left_prev_state = pb_left_state;

    //     pb_right_state = debouncer.read(PB_RIGHT_GPIO);
    //     pb_left_state = debouncer.read(PB_LEFT_GPIO);
        
    
    //     // if either button is not pressed reset current time
    //     if (pb_left_state || pb_right_state)
    //     {
    //         //store current time
    //         bootloader_timer_ms = get_absolute_time();
    //     }

    //     // // if current time - bootloader_timer_ms > 3 seconds, enter bootloader
    //     // if (absolute_time_diff_us(bootloader_timer_ms,get_absolute_time()) > BOOTLOADER_BUTTON_TIME * 1E06)
    //     // {
    //     //     // ENTER BOOTLOADER
    //     //     display.fill(0);
    //     //     display.set_cursor(0,OLED_HEIGHT/2);
    //     //     display.print("----ENTERING BOOTLOADER----");
    //     //     display.render();
    //     //     sleep_ms(100);
    //     //     reset_usb_boot(0,0);
    //     // }

    //     // If left button has been pressed for DISPLAY_RESET_BUTTON_TIME
    //     // TODO: fix this to use correct timer
    //     // if(absolute_time_diff_us(bootloader_timer_ms,get_absolute_time()) > DISPLAY_RESET_BUTTON_TIME * 1E06)
    //     // {
    //     //     // Reset display
    //     //     left_button_timer = get_absolute_time();
    //     //     display.oled_init();
    //     // }
        

    //     // If left PB was just pressed, decrease brightness
    //     // if (!pb_left_state && pb_left_prev_state)
    //     // {
    //     //     nv_settings.data.disp_brightness -= 16;
    //     //     display.set_brightness(nv_settings.data.disp_brightness);
    //     //     nv_settings.store_data();
    //     // }

    //     // // If right PB was just pressed, increase brightness
    //     // if (!pb_right_state && pb_right_prev_state)
    //     // {
    //     //     nv_settings.data.disp_brightness += 16;
    //     //     display.set_brightness(nv_settings.data.disp_brightness);       
    //     //     nv_settings.store_data();                 
    //     // }


    //     // Other calculations 
    //     // Speed calculation
    //     // core 1 should be doing speed calculations
    //     // // KPH = ERPM / Pole Pairs * wheel diameter(mm)/1000000 * PI * 60 min/hour
    //     // kph = float(m_erpm_avg/23 * 660/1000000 * 3.1415 * 60);

    //     // // Odometer calculation
    //     // // Distance = avg Speed * time
    //     // static float average_speed = 0;
    //     // static float time_hours = 0;
    //     // static float distance_travelled = 0;
    //     // // if time has been at least ODOMETER_UPDATE_INTERVAL_MS, calculate distance traveled and add to odometer
    //     // time_between_odometer_check_ms = absolute_time_diff_us(last_odometer_count,current_time_ms)/1000;

    //     // if (time_between_odometer_check_ms >= ODOMETER_UPDATE_INTERVAL_MS)
    //     // {
    //     //     average_speed = abs((kph + prev_kph_for_odometer))/2;
    //     //     time_hours = time_between_odometer_check_ms/3600.0/1000.0;// convert kph * ms to km (odo)
    //     //     distance_travelled = average_speed * time_hours;
    //     //     odometer += distance_travelled;
    //     //     prev_kph_for_odometer = kph;
    //     //     last_odometer_count = current_time_ms;
    //     // }
        
        

    
    //     // Start display stuff
    //     display.fill(0);

    //     mutex_enter_blocking(&float_mutex);

    //     // Update rolling averages
    //     b_cur_avg = (1 - ROLLING_AVG_RATIO) * b_cur_avg + ROLLING_AVG_RATIO * data_pt.current_in;
    //     b_volts_avg = (1 - ROLLING_AVG_RATIO) * b_volts_avg + ROLLING_AVG_RATIO * data_pt.v_in;

    //     // for now hardcode 13S battery Full =54.6V empty = 39V (3.0/cell) (delta V = 15.6)
    //     b_soc = MIN(((b_volts_avg - 39.0) * 100 / 15.6), 100); // Get scale from min to max batt V

    //     // #ifdef DEBUG

    //     // // Set to Debug state
    //     // display_state = DebugState;

    //     // //DEBUG: use buttons to change speed
    //     // if(pb_left_state == 0)
    //     //     data_pt.rpm -= 600.0;   
    //     // if(pb_right_state == 0)
    //     //     data_pt.rpm += 600.0;
    //     // #else
    //     // m_erpm_avg = (1 - ROLLING_AVG_RATIO) * m_erpm_avg + ROLLING_AVG_RATIO * abs(data_pt.rpm);
    //     // #endif


    //     ////////////////////////////////////////////////////////////////////////////////////////////////
    //     switch (display_state)
    //     {
    //     case DefaultState:
    //         {// TODO:
    //         // minimalist display with big font and ez to read
    //         /*
    //         BATTERY ICON
    //         TEMP
    //         SPEED
    //         POWER
    //         */
    //         draw_battery_icon();

    //         // Draw small text stuff
    //         // // TODO: Display throttle intensity visually (ADC1) next to regen (ADC2)
    //         #define THROTTLE_BAR_WIDTH 5
    //         #define THROTTLE_BAR_HEIGHT 20


    //         display.set_cursor(OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH*2, 10);
    //         display.print("T\nH\nR\nO\nT");

    //         display.set_cursor(OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH + 1, 10);
    //         display.print("R\nE\nG\nE\nN");

    //         // Draw Throttle bar
    //         display.draw_vbar(data_pt.adc1_decoded*100, OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH*2, OLED_HEIGHT - 1 - THROTTLE_BAR_HEIGHT, OLED_WIDTH - 1-THROTTLE_BAR_WIDTH, OLED_HEIGHT - 1);
    //         // Draw Regen Bar
    //         display.draw_vbar(data_pt.adc2_decoded*100, OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH, OLED_HEIGHT - 1 - THROTTLE_BAR_HEIGHT, OLED_WIDTH - 1, OLED_HEIGHT - 1);
            

    //         // TODO: Display FET/MOTOR temperature and bar graph and change max temp to non hardcode
    //         #define TEMP_BAR_X 70
    //         #define TEMP_BAR_HEIGHT 5
    //         #define FET_TEMP_Y 0
    //         #define MOTOR_TEMP_Y (FET_TEMP_Y + 5 + 1)
        
    //         display.set_cursor(24, 0);
    //         display.print_num("FETS: %2.0fC", data_pt.temp_mos);
    //         display.draw_hbar(data_pt.temp_mos/110.0 * 100.0,0, TEMP_BAR_X,0,110,TEMP_BAR_HEIGHT);
            

    //         display.set_cursor(24, MOTOR_TEMP_Y);
    //         display.print_num("MOT: %2.0fC", data_pt.temp_motor);
    //         display.draw_hbar(data_pt.temp_motor/110.0 * 100.0,0, TEMP_BAR_X,MOTOR_TEMP_Y,110,MOTOR_TEMP_Y + TEMP_BAR_HEIGHT);

    //         // Display odometer
    //         display.set_cursor(24, MOTOR_TEMP_Y + TEMP_BAR_HEIGHT + 1);
    //         display.print_num("ODO: %2.3f km", data_pt.odometer);
            



    //         // Draw big text stuff
    //         // KPH = ERPM / Pole Pairs * wheel diameter(mm)/1000000 * PI * 60 min/hour
    //         // 
    //         // TODO: lots of temp stuff. Need to get big font for KPH and W letters
            
    //         display.set_font(future_real);
           
    //         sprintf(temp_str, "%.0f", data_pt.speed_kph);
    //         display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);



    //         // Display Speed text
    //         display.set_cursor(OLED_WIDTH/2 - 1 - (temp_str_x/2), 20);
    //         display.print(temp_str);

    //         // Display Power on next row
    //         sprintf(temp_str, "%.0f", b_cur_avg * b_volts_avg);
    //         display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);      
    //         display.set_cursor(OLED_WIDTH/2 - 1 - (temp_str_x/2), 20 + display.get_font_height() + 1);       
    //         display.print(temp_str);

            
           
    //        // draw temp
    //        // draw speed
    //        // draw power
    //        //

    //         display.set_font(too_simple); // reset to the small font
    //         break;
    //         }
    //     case AnalogSpeedState:
    //         {// Display battery voltage visually and numerically
    //         // TODO: Get max Batt V from vesc for auto ranging soc value
            
    //         #define BATT_TERMINAL_TOP_LEFT_X 4
    //         #define BATT_TERMINAL_TOP_LEFT_Y 15
    //         #define BATT_TERMINAL_WIDTH 8
    //         #define BATT_TERMINAL_HEIGHT 3

    //         display.draw_vbar(b_soc, 0, 18, 15, OLED_HEIGHT - 1); // Battery icon outline
    //         display.fill_rect(0, BATT_TERMINAL_TOP_LEFT_X, BATT_TERMINAL_TOP_LEFT_Y, BATT_TERMINAL_TOP_LEFT_X + BATT_TERMINAL_WIDTH, BATT_TERMINAL_TOP_LEFT_Y + BATT_TERMINAL_HEIGHT); // Draw block to represent battery terminal

    //         // Batt Voltage and Current text
    //         display.set_cursor(2, 0);
    //         display.print_num("%.1fV", b_volts_avg);
    //         display.set_cursor(2,display.get_font_height());
    //         display.print_num("%.0fA", b_cur_avg);

    //         // Display Speed on gauge

    //         speed_gauge.set_value(data_pt.speed_kph);
    //         speed_gauge.draw();
    //         sprintf(temp_str, "%.1f KPH", data_pt.speed_kph);
    //         display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);


    //         // Blank out underneath text (1 pixel bigger than text box)
    //         display.fill_rect(1, OLED_WIDTH/2 - 1 - (temp_str_x/2) - 1, TEXT_UNDER_SPEEDO_Y_START, OLED_WIDTH/2 - 1 + (temp_str_x/2), TEXT_UNDER_SPEEDO_Y_START + 2*temp_str_y+1);

    //         // Display Speed text
    //         display.set_cursor(OLED_WIDTH/2 - 1 - (temp_str_x/2), TEXT_UNDER_SPEEDO_Y_START);
    //         display.print(temp_str);

    //         // Display Power
    //         sprintf(temp_str, "%.0fW", b_cur_avg * b_volts_avg);
    //         display.get_str_dimensions(temp_str, &temp_str_x, &temp_str_y);      
    //         display.set_cursor(OLED_WIDTH/2 - 1 - (temp_str_x/2), TEXT_UNDER_SPEEDO_Y_START + display.get_font_height() + 1);       
    //         display.print(temp_str);


    //         // Display watt-hours charged numerically
    //         // TODO Watt hours used instead
    //         // display.fill_rect(1, 18, 52, 117, OLED_HEIGHT - 1);  // Blank out area where text will draw
    //         // display.set_cursor(19, 54);
    //         // display.print_num("WATT-HRS GENERATED: %.1f", data_pt.watt_hours_charged);

    //         // // TODO: Display throttle intensity visually (ADC1) next to regen (ADC2)
    //         #define THROTTLE_BAR_WIDTH 5
    //         #define THROTTLE_BAR_HEIGHT 20


    //         display.set_cursor(OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH*2, 10);
    //         display.print("T\nH\nR\nO\nT");

    //         display.set_cursor(OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH + 1, 10);
    //         display.print("R\nE\nG\nE\nN");

    //         // Draw Throttle bar
    //         display.draw_vbar(data_pt.adc1_decoded*100, OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH*2, OLED_HEIGHT - 1 - THROTTLE_BAR_HEIGHT, OLED_WIDTH - 1-THROTTLE_BAR_WIDTH, OLED_HEIGHT - 1);
    //         // Draw Regen Bar
    //         display.draw_vbar(data_pt.adc2_decoded*100, OLED_WIDTH - 1 - THROTTLE_BAR_WIDTH, OLED_HEIGHT - 1 - THROTTLE_BAR_HEIGHT, OLED_WIDTH - 1, OLED_HEIGHT - 1);
            

    //         // TODO: Display FET/MOTOR temperature and bar graph and change max temp to non hardcode
    //         #define TEMP_BAR_X 70
    //         #define TEMP_BAR_HEIGHT 5
    //         #define FET_TEMP_Y 0
    //         #define MOTOR_TEMP_Y (FET_TEMP_Y + 5 + 1)
        
    //         display.set_cursor(24, 0);
    //         display.print_num("FETS: %2.0fC", data_pt.temp_mos);
    //         display.draw_hbar(data_pt.temp_mos/110.0 * 100.0,0, TEMP_BAR_X,0,110,TEMP_BAR_HEIGHT);
            

    //         display.set_cursor(24, MOTOR_TEMP_Y);
    //         display.print_num("MOT: %2.0fC", data_pt.temp_motor);
    //         display.draw_hbar(data_pt.temp_motor/110.0 * 100.0,0, TEMP_BAR_X,MOTOR_TEMP_Y,110,MOTOR_TEMP_Y + TEMP_BAR_HEIGHT);

    //         // TODO: fps counter
    //         // every second count how many frames
    //         // fps += 1;
    //         // if(absolute_time_diff_us(get_absolute_time(),next_fps_count) >= 0)
    //         // {
    //         //     // show fps in top right corner
    //         //     display.set_cursor(OLED_WIDTH - 10,0);
    //         //     display.print_num("%2.0", fps);
    //         //     delayed_by_ms(next_fps_count,1000); // set next fps count timer
    //         //     fps = 0; // 
    //         // }
    //         break;}

    //     case DebugState:
    //         {// Do debug stuff

    //         // do debug data here instead of getting from vesc
    //         data_pt.speed_kph = float(data_pt.rpm/23.0 * 660/1000000 * 3.1415 * 60);
    //         // average speed * time travelled * (kph * us to km conversion)
    //         average_speed = (prev_kph + data_pt.speed_kph)/2;
    //         time_us = absolute_time_diff_us(prev_time_sample,get_absolute_time());
    //         distance_travelled = (double)(average_speed * time_us/(3600)); // distance travelled as mm
    //         data_pt.odometer = (double)(data_pt.odometer +  distance_travelled/1000000.0); // convert distance_travelled to km and add to odo
    //         // double check proper math is done and not truncated due to data types

    //         prev_kph = data_pt.speed_kph;
    //         prev_time_sample = get_absolute_time();

    //         display.set_cursor(0,0);
    //         char debug_string[200];
    //         sprintf(debug_string,"odometer:%f\n\
    //         kph:%f\n\
    //         erpm:%f\n\
    //         prev_kph:%f\n\
    //         average_speed:%f\n\
    //         time_us:%d\n\
    //         distance_trv:%f\n\
    //         ",data_pt.odometer,data_pt.speed_kph,data_pt.rpm,prev_kph,average_speed,time_us,distance_travelled);
    //         display.print(debug_string);

    //     default:
    //         break;}
    //     }

        
        

    //     mutex_exit(&float_mutex);
    //     display.render();
    // }
}


void uart0_write(uint8_t * src, size_t len)
{
    uart_write_blocking(uart0, src, len);
    //DBG_PRINT("UART0 write, size: %d\n", len);
}


uint8_t receive_packet(PACKET_STATE_t *rx_packet)
{
    uint bytes_read = 0;

    // Wait up to 20ms for the initial response
    if (uart_is_readable_within_us(uart0, 10000))
    {
        // Read each byte of the response until there is no more data sent for 30 bits worth of time
        while(uart_is_readable_within_us(uart0, (1e6*30)/UART_BAUDRATE))
        {
            packet_process_byte(uart_getc(uart0), rx_packet);
            bytes_read++;
        }
    }

    //DBG_PRINT("receive_packet: %d bytes read\n", bytes_read);

    if (bytes_read > 0)
        return 1;
    else
        return 0;
}

void process_data(uint8_t *data, size_t len);


static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}


// Second core thread. Handles UART/CAN communication and SD card logging
void core1_entry()
{
    DBG_PRINT("Core 1 launched.\n");

    // Initialize UART0 
    real_baudrate = uart_init(uart0, UART_BAUDRATE);

    // Initialize can2040
    can2040_setup(&cbus, CAN_PIO_UNIT_NUM);
    can2040_callback_config(&cbus, can2040_cb);

    irq_set_exclusive_handler(CAN_PIO_IRQn, PIOx_IRQHandler);
    irq_set_priority(CAN_PIO_IRQn, 1);
    irq_set_enabled(CAN_PIO_IRQn, true);
    can2040_start(&cbus, frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS)*1000, CAN_BAUDRATE, CAN_RX_GPIO, CAN_TX_GPIO);

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

    // CAN test
    can2040_msg cur_msg;
    uint8_t VESC_ID;
    uint8_t CMD_ID;
    uint8_t status_flags = 0;
    uint32_t idx;
    absolute_time_t last_complete_status = get_absolute_time();
 
    while (true)
    {
        if (!can_msg_buf.is_empty())
        {
            cur_msg = can_msg_buf.pop();

            // MS bytes should be 0x8000 for VESC messages
            if ((cur_msg.id & 0xFFFF0000) == 0x80000000)
            {
                VESC_ID = cur_msg.id & 0xFF;        // low byte of ID is VESC unit ID
                CMD_ID = (cur_msg.id & 0xFF00) >> 8;  // second byte of ID is command type
                idx = 0;

                DBG_PRINT("CAN_ID=%08X VESC ID=0x%02X CMD=%d\n", cur_msg.id, VESC_ID, CMD_ID);

                mutex_enter_blocking(float_mutex);
                switch (CMD_ID)
                {
                    case CAN_PACKET_STATUS:
                        status_flags |= 1 << 1;
                        data_pt->rpm = buffer_get_int32(cur_msg.data, &idx);
                        data_pt->current_motor = buffer_get_int16(cur_msg.data, &idx) * 10;
                        data_pt->duty_now = buffer_get_int16(cur_msg.data, &idx) * 1000;
                        break;

                    case CAN_PACKET_STATUS_2:
                        status_flags |= 1 << 2;
                        data_pt->amp_hours = buffer_get_int32(cur_msg.data, &idx) * 10000;
                        data_pt->amp_hours_charged = buffer_get_int32(cur_msg.data, &idx) * 10000;
                        break;                 

                    case CAN_PACKET_STATUS_3:
                        status_flags |= 1 << 3;
                        data_pt->watt_hours = buffer_get_int32(cur_msg.data, &idx) * 10000;
                        data_pt->watt_hours_charged = buffer_get_int32(cur_msg.data, &idx) * 10000;
                        break;       
                        
                    case CAN_PACKET_STATUS_4:
                        status_flags |= 1 << 4;
                        data_pt->temp_mos = buffer_get_int16(cur_msg.data, &idx) * 10;
                        data_pt->temp_motor = buffer_get_int16(cur_msg.data, &idx) * 10;
                        data_pt->current_in = buffer_get_int16(cur_msg.data, &idx) * 10;
                        data_pt->position = buffer_get_int16(cur_msg.data, &idx) * 50;     
                        break;        
                        
                    case CAN_PACKET_STATUS_5:
                        status_flags |= 1 << 5;
                        data_pt->tachometer = buffer_get_int32(cur_msg.data, &idx) * 6;
                        data_pt->v_in = buffer_get_int16(cur_msg.data, &idx) * 10;
                        break;    

                    case CAN_PACKET_STATUS_6:
                        status_flags |= 1 << 6;
                        // need space in struct for ADC readings

                        break;                                                    
                }
                mutex_exit(float_mutex);

                // If B1-B6 are set, a full set of status messages was received
                if (status_flags == 0b1111110)
                {
                    DBG_PRINT("Full status received, %dms since last\n", get_absolute_time() - last_complete_status);
                    last_complete_status = get_absolute_time();
                    status_flags = 0;
                }
            }
            else
            {
                // non VESC message
                DBG_PRINT("Rx CAN msg: ID 0x%08X, %d data bytes\n", cur_msg.id, cur_msg.dlc);
            }
        }
        
    }

    next_sample = get_absolute_time();
    while (true)
    {
        mutex_enter_blocking(flash_mutex);  // Don't allow flash erase/write while running core1 code

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

        // If SD_DETECT_GPIO == 0 an SD card is connected
        if (sd_status == SD_NOT_PRESENT && gpio_get(SD_DETECT_GPIO) == 0)
        {
            DBG_PRINT(/*"\033[2J\033[H"*/ "FILESYSTEM INIT\n");
            result = init_filesystem();

            DBG_PRINT("init_filesystem(): %s\n", FRESULT_str(result));
            sleep_ms(1);

            result = create_log_file();    
            
            DBG_PRINT("create_log_file(): %s\n", FRESULT_str(result));
            sleep_ms(1);

        }
        else if(sd_status == SD_PRESENT && vesc_connected)
        {
            result = append_data_pt(data_pt);       

            DBG_PRINT("append_data_pt(): %s\n", FRESULT_str(result));            
        }

        //DBG_PRINT("sd_status: %d, vesc_connected:%d\r", sd_status, vesc_connected);


        mutex_exit(flash_mutex);    // Release lock
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

            mutex_enter_blocking(float_mutex);

            //memcpy(get_values_response, data+5, len-5);

            data_pt->ms_today = time_us_64() / 1000;

            // Unpack data from VESC response
            data_pt->temp_mos = buffer_get_float16(data, 1e1, &idx);
            data_pt->temp_motor = buffer_get_float16(data, 1e1, &idx);
            data_pt->current_motor = buffer_get_float32(data, 1e2, &idx);
            data_pt->current_in = buffer_get_float32(data, 1e2, &idx);
            data_pt->id = buffer_get_float32(data, 1e2, &idx);
            data_pt->iq = buffer_get_float32(data, 1e2, &idx);
            data_pt->duty_now = buffer_get_float16(data, 1e3, &idx);
            data_pt->rpm = buffer_get_float32(data, 1e0, &idx);
            data_pt->v_in = buffer_get_float16(data, 1e1, &idx);
            data_pt->amp_hours = buffer_get_float32(data, 1e4, &idx);
            data_pt->amp_hours_charged = buffer_get_float32(data, 1e4, &idx);
            data_pt->watt_hours = buffer_get_float32(data, 1e4, &idx);
            data_pt->watt_hours_charged = buffer_get_float32(data, 1e4, &idx);
            data_pt->tachometer = buffer_get_int32(data, &idx);
            data_pt->tachometer_abs = buffer_get_int32(data, &idx);
            data_pt->fault_code = (mc_fault_code)data[idx++];
            data_pt->position = buffer_get_float32(data, 1e6, &idx);
            data_pt->vesc_id = data[idx++];
            data_pt->temp_mos_1 = buffer_get_float16(data, 1e1, &idx);
            data_pt->temp_mos_2 = buffer_get_float16(data, 1e1, &idx);
            data_pt->temp_mos_3 = buffer_get_float16(data, 1e1, &idx);
            data_pt->vd = buffer_get_float32(data, 1e3, &idx);
            data_pt->vq = buffer_get_float32(data, 1e3, &idx);

            // Calculated values
            // Power
            data_pt->p_in = data_pt->current_in * data_pt->v_in;
            // Speed calculation
            // TODO: change hardcode wheel diameter and get from vesc
            // KPH = ERPM / Pole Pairs * wheel diameter(mm)/1000000 * PI * 60 min/hour

            
            // do debug data here instead of getting from vesc
            data_pt->speed_kph = float(data_pt->rpm/23.0 * 660/1000000 * 3.1415 * 60);
            // average speed * time travelled * (kph * us to km conversion)
            average_speed = (prev_kph + data_pt->speed_kph)/2;
            time_us = absolute_time_diff_us(prev_time_sample,get_absolute_time());
            distance_travelled = (double)(average_speed * time_us/(3600)); // distance travelled as mm
            data_pt->odometer = (double)(data_pt->odometer +  distance_travelled/1000000.0); // convert distance_travelled to km and add to odo
            // double check proper math is done and not truncated due to data types

            //last byte is int8 status, but data_pt struct has no place for it

            mutex_exit(float_mutex);
            break;

        case COMM_GET_DECODED_ADC:

            mutex_enter_blocking(float_mutex);
            data_pt->adc1_decoded = buffer_get_float32(data, 1e6, &idx);
            adc_v1 = buffer_get_float32(data, 1e6, &idx);
            data_pt->adc2_decoded = buffer_get_float32(data, 1e6, &idx);
            adc_v2 = buffer_get_float32(data, 1e6, &idx);
            mutex_exit(float_mutex);

            break;

    }

}


// Callback to handle CAN messages
static void __not_in_flash_func(can2040_cb)(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    switch(notify)
    {
        case CAN2040_NOTIFY_RX:
            // DBG_PRINT("Rx'd\n");
            can_msg_buf.push(*msg);
            break;

        case CAN2040_NOTIFY_TX:
            // DBG_PRINT("Tx'd\n");
            break;

        case CAN2040_NOTIFY_ERROR:
            DBG_PRINT("ERR\n");
            break;
    }
}
