#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "pico/cyw43_arch.h"
#include "pico/rand.h"
#include "pico/stdlib.h"
#include "pico/util/datetime.h"

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
#include "hardware/watchdog.h"
#include "hardware/exception.h"

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
#include "crash.h"
}

#include <u8g2.h>
#include "u8x8_interface.hpp"
#include "spi_tx_9bit.pio.h"

#include "page_ctrl.hpp"

// Comment out debug when not using
#define DEBUG

#define DISPLAY_RESET_BUTTON_TIME 2
#define ODOMETER_UPDATE_INTERVAL_MS 100 // time between odometer updates


// function prototypes
void core1_entry();
static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg);
void process_data(uint8_t *data, size_t len);

typedef struct
{
    uint8_t msg[6];
    uint8_t length;
} comm_msg;

static struct can2040 cbus;
volatile circular_buf<can2040_msg, 20> can_rx_buf;
circular_buf<can2040_msg, 10> can_tx_buf;
circular_buf<comm_msg, 10> comm_request_buf;
uint8_t vesc_can_ids[VESC_CAN_ID_MAX];
uint8_t vesc_id_count = 0;
uint8_t display_can_id = 0;
uint8_t awaiting_response = 0;

uint32_t real_baudrate = 0;
uint8_t response_code = 0;
volatile uint8_t vesc_connected = 0;
uint8_t do_logging = 0;

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
volatile absolute_time_t core1_last_loop = current_time_ms;

page_controller page_ctrl;
log_data_t *data_pt;
mutex_t *float_mutex;


// Core 0
int main()
{
    // Check if reset was caused by watchdog or fault
    if (watchdog_caused_reboot())
        crash_info.watchdog_trip = 1;
    else
        crash_info.watchdog_trip = 0;

    if (crash_info.magic == 0xDEADBEEF && (crash_info.reason != 0 || crash_info.watchdog_trip))
    {
        // Copy the previous crash data to a different location so it doesn't get overwritten
        //previous_crash = crash_info;
        memcpy(&previous_crash, (const void *)&crash_info, sizeof(crash_info));
    }
    else
    {
        // Normal boot, initialize crash data
        // crash_info struct doesn't get zeroed, magic number proves it's not random values
        crash_info.magic = 0xDEADBEEF;
        crash_info.reason = 0;
        crash_info.watchdog_trip = 0;
        crash_info.fault_caused_reset = 0;
        crash_info.core0_state = C0_ENTRY;
        crash_info.core1_state = C1_OFF;

        previous_crash = {};    // clear previous_crash
    }

    // Register exception handlers for core 0
    exception_set_exclusive_handler(HARDFAULT_EXCEPTION, hardfault_handler);
    exception_set_exclusive_handler(NMI_EXCEPTION, nmi_handler);

    stdio_init_all();
    time_init();  
    cyw43_arch_init();  

    initialize_gpio();
    init_external_rtc();

    get_rand_32();  // First call of this takes ~1ms to generate the seed

    watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);    // 1 = pause on debug

    // U8G2 init
    //set_backlight(120);
    u8g2_Setup_st75256_jlx256128_f(&page_ctrl.u8g2, U8G2_R0, U8G2_BYTE_FN, u8x8_gpio_and_delay_pico);
    u8g2_InitDisplay(&page_ctrl.u8g2);    // Init sequence, ends with display in sleep mode
    u8g2_SetContrast(&page_ctrl.u8g2, 170); 

    // COG-F206 init sequence

    // EXT=0, Sleep out, EXT=1, autoread disable
    u8g2_SendF(&page_ctrl.u8g2, "ccccd", 0x30, 0x94, 0x31, 0xd7, 0x9f);

    // Analog set, OSC freq. adjust, 6kHz, bias=1/12, grey level
    u8g2_SendF(&page_ctrl.u8g2, "cdddc", 0x32, 0x00, 0x01, 0x02, 0x20);
    u8g2_SendF(&page_ctrl.u8g2, "ccddd", 0x31, 0xf2, 0x1e, 0x28, 0x32);

    u8g2_SendF(&page_ctrl.u8g2, "ddddd", 0x01, 0x03, 0x05, 0x07, 0x09);
    u8g2_SendF(&page_ctrl.u8g2, "ddddd", 0x0b, 0x0d, 0x10, 0x11, 0x13);
    u8g2_SendF(&page_ctrl.u8g2, "dddddd", 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f);

    // EXT=0, Page addr, column addr
    u8g2_SendF(&page_ctrl.u8g2, "ccddcdd", 0x30, 0x75, 0x00, 0x14, 0x15, 0x00, 0xff);

    // Duty=128, Nline=off
    u8g2_SendF(&page_ctrl.u8g2, "cddd", 0xca, 0x00, 0x7f, 0x20);

    // Monochrome mode
    u8g2_SendF(&page_ctrl.u8g2, "cd", 0xf0, 0x10);

    // EV control, VOP = 15.0V
    u8g2_SendF(&page_ctrl.u8g2, "cdd", 0x81, 0x06, 0x05);
    
    // Power, VB, VF, VR on
    u8g2_SendF(&page_ctrl.u8g2, "cd", 0x20, 0x0b);

    sleep_us(100);
    u8g2_SetPowerSave(&page_ctrl.u8g2, 0);        


    u8g2_ClearBuffer(&page_ctrl.u8g2);
    u8g2_ClearDisplay(&page_ctrl.u8g2);
    //u8g2_SetFont(&page_ctrl.u8g2, u8g2_font_t0_11_te);
    u8g2_SetFont(&page_ctrl.u8g2, u8g2_font_10x20_tf);
    u8g2_SetDrawColor(&page_ctrl.u8g2, 1);  

  
    DBG_PRINT("Core 0 init complete.\n");

    // Set up some pointers that core1 needs to access
    data_pt = &page_ctrl.esc_data; 
    float_mutex = &page_ctrl.float_mutex;
    multicore_launch_core1(core1_entry);

    datetime_t sram_time;
    datetime_t current_time;
    uint32_t time_past_s;
    uint16_t time_diff_day;
    uint8_t time_diff_hour;
    uint8_t time_diff_min;

    #define SECONDS_IN_MIN  60
    #define SECONDS_IN_HOUR 3600
    #define SECONDS_IN_DAY  86400    


    // Get time that had been stored in RTC SRAM
    if (rtc_connected)
    {
        get_rtc_sram(offsetof(rtc_sram_map_t, LAST_TIME_SEEN), (uint8_t *)&sram_time, sizeof(sram_time));
        rtc_get_datetime(&current_time);

        // Compute the time difference
        time_past_s = datetime_to_sec(&current_time) - datetime_to_sec(&sram_time);
        time_diff_day = time_past_s / SECONDS_IN_DAY;
        time_past_s = time_past_s % SECONDS_IN_DAY;

        time_diff_hour = time_past_s / SECONDS_IN_HOUR;
        time_past_s = time_past_s % SECONDS_IN_HOUR;

        time_diff_min = time_past_s / SECONDS_IN_MIN;

        DBG_PRINT(
            "System was last running on:\n %4d/%02d/%02d, at %02d:%02d:%02d\n (%d days, %d hours and %d minutes ago)\n", \
            sram_time.year, sram_time.month, sram_time.day, \
            sram_time.hour, sram_time.min, sram_time.sec, \
            time_diff_day, time_diff_hour, time_diff_min                    
        );
    }
    crash_info.core0_state = C0_INIT_DONE;

    next_frame_time = get_absolute_time();
    while(1)
    {
        // Only update watchdog if core1 is not stalled
        if (time_us_64() - core1_last_loop < (WATCHDOG_TIMEOUT_MS) * 1000)
            watchdog_update();

        crash_info.core0_state = C0_WAIT_FOR_FRAME;
        sleep_until(next_frame_time);
        next_frame_time = delayed_by_ms(next_frame_time, 1000 / 20);

        crash_info.core0_state = C0_PAGE_UPDATE;
        page_ctrl.update();

        crash_info.core0_state = C0_PAGE_DRAW;
        page_ctrl.draw_page();

        // Use idle time between frames to store current time to RTC SRAM
        rtc_get_datetime(&sram_time);
        set_rtc_sram_nonblocking(offsetof(rtc_sram_map_t, LAST_TIME_SEEN), (const uint8_t *)&sram_time, sizeof(sram_time));
    }
   





    //     // If left button has been pressed for DISPLAY_RESET_BUTTON_TIME
    //     // TODO: fix this to use correct timer
    //     // if(absolute_time_diff_us(bootloader_timer_ms,get_absolute_time()) > DISPLAY_RESET_BUTTON_TIME * 1E06)
    //     // {
    //     //     // Reset display
    //     //     left_button_timer = get_absolute_time();
    //     //     display.oled_init();
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
    /*
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
    */
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


uint8_t receive_packet_uart(PACKET_STATE_t *rx_packet)
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

    //DBG_PRINT("receive_packet_uart: %d bytes read\n", bytes_read);

    if (bytes_read > 0)
        return 1;
    else
        return 0;
}


void receive_packet_can(can2040_msg *rx_msg, uint8_t can_command_id)
{
    static uint8_t rx_buffer[PACKET_MAX_PL_LEN];
    uint16_t buffer_len;
    uint8_t vesc_id, first_empty_id;

    switch (can_command_id)
    {
        case CAN_PACKET_FILL_RX_BUFFER:
            // Only process this packet if it was sent to the display specifically
            if ((rx_msg->id & 0xFF) == display_can_id)
            {
                //DBG_PRINT("Filling RX buffer at offset 0x%02X\n", rx_msg->data[0]);
                // Copy the 7 data bytes into the rx buffer
                for (uint buf_offset = 0; buf_offset < 7; buf_offset++)
                {
                    rx_buffer[rx_msg->data[0] + buf_offset] = rx_msg->data[1+buf_offset];
                }
            }
            break;

        case CAN_PACKET_PROCESS_RX_BUFFER:
            // Only process this packet if it was sent to the display specifically
            if ((rx_msg->id & 0xFF) == display_can_id)
            {        
                buffer_len = (rx_msg->data[2] << 8) | rx_msg->data[3];
                //DBG_PRINT("Processing RX buf of length %d\n", buffer_len);
                process_data(rx_buffer, buffer_len);
                awaiting_response = 0;
            }
            break;


        case CAN_PACKET_PROCESS_SHORT_BUFFER:
            // Only process this packet if it was sent to the display specifically
            if ((rx_msg->id & 0xFF) == display_can_id)
            {
                buffer_len = rx_msg->dlc - 2;
                memcpy(rx_buffer, rx_msg+2, buffer_len);
                process_data(rx_buffer, buffer_len);
                awaiting_response = 0;
            }
            break;

        //A VESC device replied to a CAN_PACKET_PING
        case CAN_PACKET_PONG:
            awaiting_response = 0;
            vesc_id = rx_msg->data[0];
            //DBG_PRINT("Ping reply from ADDR: 0x%02X, HW_TYPE: %d\n", vesc_id, rx_msg->data[1]);
            
            // Only process pong packets from ESCs
            if (rx_msg->data[1] == HW_TYPE_VESC)
            {
                first_empty_id = 0xFF;
                for (uint8_t idn = 0; idn < VESC_CAN_ID_MAX; idn++)
                {
                    if (vesc_can_ids[idn] == 0 && first_empty_id == 0xFF)
                    {
                        first_empty_id = idn;
                    }
                    // If the ID is in the list already, abort
                    if (vesc_can_ids[idn] == vesc_id)
                        return;
                }
                if (first_empty_id != 0xFF)
                {
                    vesc_can_ids[first_empty_id] = vesc_id;
                    vesc_id_count++;
                    DBG_PRINT("VESC ID 0x%02X added.\n", vesc_id);
                }
            }
            break;


        // case CAN_PACKET_NOTIFY_BOOT:
        //     // Add this VESC's ID if it's not known already
        //     vesc_id = rx_msg->id & 0xFF;
        //     first_empty_id = 0xFF;
        //     for (uint8_t idn = 0; idn < VESC_CAN_ID_MAX; idn++)
        //     {
        //         if (vesc_can_ids[idn] == 0 && first_empty_id == 0xFF)
        //         {
        //             first_empty_id = idn;
        //         }
        //         // If the ID is in the list already, abort
        //         if (vesc_can_ids[idn] == vesc_id)
        //             return;
        //     }
        //     if (first_empty_id != 0xFF)
        //     {
        //         vesc_can_ids[first_empty_id] = vesc_id;
        //         vesc_id_count++;
        //         DBG_PRINT("VESC ID 0x%02X added.\n", vesc_id);
        //     }

        //     break;

        default:
            //DBG_PRINT("Unhandled packet type.\n");
            break;
    }

}


static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}


// Second core thread. Handles UART/CAN communication and SD card logging
void core1_entry()
{
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

    // UART related
    PACKET_STATE_t vesc_comm;
    uint8_t send_payload[50];

    // CAN related
    can2040_msg cur_msg;
    uint8_t cmd_id;
    uint8_t temp_id;
    //page_ctrl.nv_settings.data.flags |= COMM_USE_CAN;    ///// debug override
    absolute_time_t msg_send_time;
    absolute_time_t sd_last_attempt;
    absolute_time_t next_sample;  
    absolute_time_t comm_retry; 
    absolute_time_t next_ping_time; 
    FRESULT result;
    char time_str[25];
    datetime_t current_time;
    comm_msg request_msg;
    uint8_t can_ping_addr;
    uint8_t can_scan_active = 0;

    page_ctrl.nv_settings.alt_core_init();     // Allows core 0 to stop this core while doing flash operations

    // Register exception handlers for core 1
    exception_set_exclusive_handler(HARDFAULT_EXCEPTION, hardfault_handler);
    exception_set_exclusive_handler(NMI_EXCEPTION, nmi_handler);    

    DBG_PRINT("Core 1 launched.\n");
    crash_info.core1_state = C1_ENTRY;

    if (rtc_time_valid)
    {
        rtc_get_datetime(&current_time);
        time_to_str(&current_time, time_str, 0);
        DBG_PRINT("RTC time: %s\n", time_str);
    }


    memset(vesc_can_ids, 0, VESC_CAN_ID_MAX); // Clear array of VESC CAN IDs

    // Initialize UART0 
    real_baudrate = uart_init(uart0, UART_BAUDRATE);

    // Initialize can2040
    can2040_setup(&cbus, CAN_PIO_UNIT_NUM);
    can2040_callback_config(&cbus, can2040_cb);

    irq_set_exclusive_handler(CAN_PIO_IRQn, PIOx_IRQHandler);
    irq_set_priority(CAN_PIO_IRQn, 1);
    irq_set_enabled(CAN_PIO_IRQn, true);
    can2040_start(&cbus, frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS)*1000, CAN_BAUDRATE, CAN_RX_GPIO, CAN_TX_GPIO);

    // Apply user CAN IDs if they've been set
    if (page_ctrl.nv_settings.data.user_vesc_can_id)
    {
        vesc_can_ids[0] = page_ctrl.nv_settings.data.user_vesc_can_id;
        vesc_id_count++;
    }

    if (page_ctrl.nv_settings.data.user_disp_can_id)
    {
        display_can_id = page_ctrl.nv_settings.data.user_disp_can_id;
    }    


    next_sample = get_absolute_time();    
    sd_last_attempt = get_absolute_time();
    comm_retry = delayed_by_ms(get_absolute_time(), 10*COMM_MSG_TIMEOUT_MS);
    next_ping_time = delayed_by_ms(get_absolute_time(), 2000);  // Start ping scanning 2 seconds after boot

    //DBG_PRINT("Comm. interface init complete.\n");
    crash_info.core1_state = C1_INIT_DONE;
    while (true)
    {
        core1_last_loop = get_absolute_time();      

        crash_info.core1_state = C1_GENERATE_REQUESTS;
        if (!config_received && comm_retry < get_absolute_time())
        {
            // Generate config request messages
            memset(request_msg.msg, 0, 6);     
            request_msg.msg[0] = COMM_GET_MCCONF_TEMP;
            request_msg.length = 1;
            comm_request_buf.push(request_msg);            
            comm_retry = delayed_by_ms(get_absolute_time(), 10*COMM_MSG_TIMEOUT_MS);
        }

        // Add request messages to queue if it's time to get updated data from ESC
        if (next_sample < get_absolute_time())
        {
            next_sample = delayed_by_ms(next_sample, 1000 / LOG_SAMPLE_RATE);        

            memset(request_msg.msg, 0, 6); 

            // Main data polling
            request_msg.msg[0] = COMM_GET_VALUES;
            request_msg.length = 1;
            comm_request_buf.push(request_msg);

            // Throttle + regen input values
            request_msg.msg[0] = COMM_GET_DECODED_ADC;
            request_msg.length = 1;
            comm_request_buf.push(request_msg);

            // Battery level
            int32_t idx = 1;
            request_msg.msg[0] = COMM_GET_VALUES_SETUP_SELECTIVE;
             // battery_level (1 << 8)
            buffer_append_uint32(request_msg.msg, (1 << 8), &idx);
            request_msg.length = 5;
            comm_request_buf.push(request_msg);
        }

        crash_info.core1_state = C1_COMM_RECEIVE;
        // Handle CAN messages if CAN mode is enabled
        if (page_ctrl.nv_settings.data.flags & COMM_USE_CAN)
        {
            if (!can_rx_buf.is_empty())
            {
                crash_info.core1_state = C1_PROCESS_CAN_MSG;
                cur_msg = can_rx_buf.pop();

                // MS bytes should be 0x8000 for VESC messages
                if ((cur_msg.id & 0xFFFF0000) == 0x80000000)
                {
                    //vesc_id = cur_msg.id & 0xFF;        // low byte of ID is VESC unit ID
                    cmd_id = (cur_msg.id & 0xFF00) >> 8;  // second byte of ID is command type

                    //DBG_PRINT("CAN MSG! ID=%08X\n", cur_msg.id);
                    //DBG_PRINT("CAN_ID=%08X VESC ID=0x%02X CMD=%d DAT=%dB\n", cur_msg.id, vesc_id, cmd_id, cur_msg.dlc);

                    // If VESC ID is not known, use the ID from the first VESC message we see
                    // if (vesc_id_count == 0)
                    // {
                    //     vesc_can_ids[0] = vesc_id;
                    //     vesc_id_count++;
                    // }

                    //DBG_PRINT("VESC=0x%02X CMD=%d DATA(%d)=%08X%08X\n", vesc_id, cmd_id, cur_msg.dlc, cur_msg.data32[1], cur_msg.data32[0]);

                    receive_packet_can(&cur_msg, cmd_id);
                }
                else
                {
                    // non VESC message
                    DBG_PRINT("Rx CAN msg: ID 0x%08X, %d data bytes\n", cur_msg.id, cur_msg.dlc);
                }
            }

            // Need some method to send next packet type after first is received
            // and to run log data saving after all desired packets were received

            // If it's time to poll ESC and an ESC ID is known, send out requests for data
            //if (vesc_id_count != 0 && (next_sample + (100000 / LOG_SAMPLE_RATE)) < get_absolute_time())
            //if(vesc_id_count != 0 && !comm_request_buf.is_empty())


            // Decide on an ID for the display if one isn't known already  
            while (!display_can_id)
            {
                temp_id = (uint8_t) get_rand_32();

                for (uint8_t idn = 0; idn < VESC_CAN_ID_MAX; idn++)
                {
                    // If the ID matches an existing one
                    if (temp_id == vesc_can_ids[idn])
                    {
                        temp_id = 0;
                    }
                }

                // If the temp ID didn't get cleared, we can use it
                if (temp_id && temp_id != 0xFF)
                {
                    display_can_id = temp_id;
                    DBG_PRINT("Display CAN ID selected: 0x%02X\n", display_can_id);
                }
            }

            // // If no VESCs are known send out a ping to the broadcast address 0xFF
            // if (vesc_id_count == 0 && next_ping_time < get_absolute_time())
            // {
            //     DBG_PRINT("Initiating CAN bus scan on connected ESCs.\n");
            //     // cur_msg.id = (CAN_PACKET_PING << 8) | 0xFF | CAN2040_ID_EFF;
            //     // cur_msg.dlc = 0;    // no data for ping
            //     // can_tx_buf.push(cur_msg);

                
            //     // Use CAN short buffer to send a UART command COMM_PING_CAN
            //     cur_msg.id = (CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | 0xFF | CAN2040_ID_EFF;
            //     cur_msg.data[0] = display_can_id;
            //     cur_msg.data[1] = 0;
            //     cur_msg.data[2] = COMM_PING_CAN;
            //     cur_msg.dlc = 3;
            //     can_tx_buf.push(cur_msg);
            //     next_ping_time = delayed_by_ms(get_absolute_time(), 1000*COMM_MSG_TIMEOUT_MS);
            // }   
            
            // If no VESCs are known, initiate scan to ping all addresses
            if (vesc_id_count == 0 && !can_scan_active && next_ping_time < get_absolute_time())
            {
                can_scan_active = 1;
                can_ping_addr = 1;  // Skip address 0
                DBG_PRINT("Initiating CAN bus probe.\n");
            }
            
            if (can_scan_active && next_ping_time < get_absolute_time())
            {
                if (!can_tx_buf.is_full())
                {
                    cur_msg.id = (CAN_PACKET_PING << 8) | can_ping_addr++ | CAN2040_ID_EFF;
                    cur_msg.dlc = 0;    // no data for ping
                    can_tx_buf.push(cur_msg);       
                    next_ping_time = delayed_by_ms(get_absolute_time(), 10);
                    awaiting_response = 0;  // Don't wait, most pings won't get a response
                }

                // Reached end of address range
                if (can_ping_addr == 255)
                {
                    can_scan_active = 0;
                    next_ping_time = delayed_by_ms(get_absolute_time(), 5*1000);    // Don't scan again for 5 seconds
                }
            }
            
            // Process comm requests into CAN request messages
            while (!comm_request_buf.is_empty())
            {
                request_msg = comm_request_buf.pop();

                // Abort if VESC CAN ID is not known yet
                if (vesc_can_ids[0] == 0)
                    break;
                cur_msg.id = (CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | vesc_can_ids[0] | CAN2040_ID_EFF;
                cur_msg.data[0] = display_can_id;
                cur_msg.data[1] = 0;         
                memcpy(&cur_msg.data[2], request_msg.msg, request_msg.length);
                cur_msg.dlc = 2 + request_msg.length;
                can_tx_buf.push(cur_msg);
            }

            crash_info.core1_state = C1_TX_CAN_MSG;
            // Transmit messages in the queue when possible
            if (!awaiting_response && !can_tx_buf.is_empty() && can2040_check_transmit(&cbus))
            {
                crash_info.core1_state = C1_TX_CAN_START;
                cur_msg = can_tx_buf.pop();

                can2040_transmit(&cbus, &cur_msg);
                msg_send_time = get_absolute_time();    // Record time sent                 
                awaiting_response = 1;
                crash_info.core1_state = C1_TX_CAN_SENT;
            }

            // If we're waiting for a reply and it has taken too long
            if (awaiting_response && (msg_send_time + 1000 * COMM_MSG_TIMEOUT_MS) < get_absolute_time())
            {
                vesc_connected = 0;
                awaiting_response = 0;
            }
        }
        else
        {
            // UART mode; CAN is not being used, keep msg buffer clear
            can_rx_buf.pop();

            // Send out comm request messages and process the responses
            while (!comm_request_buf.is_empty())
            {
                crash_info.core1_state = C1_TX_UART_MSG;
                request_msg = comm_request_buf.pop();

                packet_init(uart0_write, process_data, &vesc_comm);
                memset(send_payload, 0, sizeof(send_payload));
                memcpy(send_payload, request_msg.msg, request_msg.length);

                packet_send_packet(send_payload, request_msg.length, &vesc_comm);

                // Reset packet that was sent so it can be reused
                packet_reset(&vesc_comm);    

                crash_info.core1_state = C1_PROCESS_UART_MSG;
                // Wait for the response packet and process it
                if (receive_packet_uart(&vesc_comm) == 0)
                {
                    // No response, consider vesc connection lost
                    vesc_connected = 0;
                } 
            }
        }


        ///////////// Logging /////////////
        // CAN gets disabled during SD i/o so CAN interrupts don't affect log writes

        // Compute speed in kph
        crash_info.core1_state = C1_LOG_PREP_DATA;
        mutex_enter_blocking(float_mutex);        
        raw_speed_kph = calc_speed_kph(page_ctrl.esc_data.rpm);
        page_ctrl.esc_data.speed_kph = raw_speed_kph;    
        mutex_exit(float_mutex);    

        crash_info.core1_state = C1_LOG_SD_CHECK;
        // Check if SD card was removed
        if ((sd_status == SD_PRESENT || sd_status == SD_ERROR) 
#ifndef SD_CARD_POLLING
            && gpio_get(SD_DETECT_GPIO)
#endif    
        )
        {
            sd_status = SD_NOT_PRESENT;
        }

        // If SD_DETECT_GPIO == 0 an SD card is connected
        else if (sd_status == SD_NOT_PRESENT 
#ifndef SD_CARD_POLLING            
            && gpio_get(SD_DETECT_GPIO) == 0
#else
            // Attempt to mount SD card periodically (10M microseconds = 10s)
            && (sd_last_attempt + 10e6) < get_absolute_time()
#endif
            
        )
        {
            sd_last_attempt = get_absolute_time();
            DBG_PRINT(/*"\033[2J\033[H"*/ "FILESYSTEM INIT\n");
            can2040_stop(&cbus);            
            result = init_filesystem();

            DBG_PRINT("init_filesystem(): %s\n", FRESULT_str(result));
            sleep_ms(1);

            result = create_log_file();    
            can2040_start(&cbus, frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS)*1000, CAN_BAUDRATE, CAN_RX_GPIO, CAN_TX_GPIO);           
            
            DBG_PRINT("create_log_file(): %s\n", FRESULT_str(result));
            sleep_ms(1);

        }
        // Try to save data to log, even if there was an error previously
        else if(do_logging && vesc_connected && (sd_status == SD_PRESENT || sd_status == SD_ERROR))
        {
            crash_info.core1_state = C1_LOG_SAVE_DATA;
            do_logging = 0;
            can2040_stop(&cbus);
            result = append_data_pt(data_pt);       
            can2040_start(&cbus, frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS)*1000, CAN_BAUDRATE, CAN_RX_GPIO, CAN_TX_GPIO);

            if (result != FR_OK)
                DBG_PRINT("append_data_pt(): %s\n", FRESULT_str(result));            
        } 
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
        case COMM_GET_VALUES_SELECTIVE:
            // First 4 bytes are get_values_selective 32b mask
            idx += 4;
        case COMM_GET_VALUES:            
            vesc_connected = 1; 
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

            
            // // do debug data here instead of getting from vesc
            // data_pt->speed_kph = float(data_pt->rpm/23.0 * 660/1000000 * 3.1415 * 60);
            // // average speed * time travelled * (kph * us to km conversion)
            // average_speed = (prev_kph + data_pt->speed_kph)/2;
            // time_us = absolute_time_diff_us(prev_time_sample,get_absolute_time());
            // distance_travelled = (double)(average_speed * time_us/(3600)); // distance travelled as mm
            // data_pt->odometer = (double)(data_pt->odometer +  distance_travelled/1000000.0); // convert distance_travelled to km and add to odo
            // // double check proper math is done and not truncated due to data types

            //last byte is int8 status, but data_pt struct has no place for it

            mutex_exit(float_mutex);
            break;

        case COMM_GET_DECODED_ADC:
            vesc_connected = 1; 
            mutex_enter_blocking(float_mutex);
            data_pt->adc1_decoded = buffer_get_float32(data, 1e6, &idx);
            adc_v1 = buffer_get_float32(data, 1e6, &idx);
            data_pt->adc2_decoded = buffer_get_float32(data, 1e6, &idx);
            adc_v2 = buffer_get_float32(data, 1e6, &idx);
            mutex_exit(float_mutex);

            break;

        case COMM_GET_MCCONF_TEMP:
            vesc_connected = 1;         
            config_received = 1;
            /*

            All float32 unless specified
            0:  current_min_scale
            4:  current_max_scale
            8:  min_erpm
            12: max_erpm
            16: min_duty
            20: max_duty
             24: watt_min
             28: watt_max
            32: in_current_min
            36: in_current_max
             40: motor_poles (uint8)
             41: gear_ratio
             45: wheel_diameter
            */
            mutex_enter_blocking(float_mutex);  

            idx = 24+1;
            watt_min = buffer_get_float32_auto(data, &idx);
            watt_max = buffer_get_float32_auto(data, &idx);

            idx = 40+1;  
            motor_poles = data[idx++];

            gear_ratio = buffer_get_float32_auto(data, &idx);
            wheel_diameter = buffer_get_float32_auto(data, &idx);
            DBG_PRINT("Received ESC motor config:\n");
            DBG_PRINT("%d poles, %.2f gear ratio, \n%.2fmm wheel diameter\n",
                motor_poles, gear_ratio, wheel_diameter*1000);
            DBG_PRINT("watt_min: %.0f, watt_max: %.0f\n", watt_min, watt_max);
            mutex_exit(float_mutex);            
            break;

        case COMM_GET_VALUES_SETUP_SELECTIVE:
            vesc_connected = 1;

            // Selection mask
            idx += 4;
            mutex_enter_blocking(float_mutex);   
            
            // Scale battery level to be within range of 0-100.0
            data_pt->battery_level = std::max(0.0f, buffer_get_float16(data, 1e1, &idx));   
            if (data_pt->battery_level > 100.0f)
                data_pt->battery_level = 100.0f;

            //DBG_PRINT("battery_level=%f\n", data_pt->battery_level);
            mutex_exit(float_mutex);  
            // This is the last request packet response, so now we can save the data to the log file
            do_logging = 1;            
            break;

        /*
        COMM_PING_CAN:
        B0: COMM_PING_CAN
        Bn: device ID
        */
        // case COMM_PING_CAN:
        //     DBG_PRINT("PING_CAN: ");
        //     for (; idx < len; idx++)
        //         DBG_PRINT("%02X", data[idx]);
        //     DBG_PRINT("\n");
        //     break;

    }

}


// Callback to handle CAN messages
static void __not_in_flash_func(can2040_cb)(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    switch(notify)
    {
        case CAN2040_NOTIFY_RX:
            // DBG_PRINT("Rx'd\n");
            can_rx_buf.push(*msg);
            break;

        case CAN2040_NOTIFY_TX:
            // DBG_PRINT("Tx'd\n");
            break;

        case CAN2040_NOTIFY_ERROR:
            DBG_PRINT("CAN error.\n");
            break;
    }
}

