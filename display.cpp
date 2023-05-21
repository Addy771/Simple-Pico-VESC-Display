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

void core1_entry();

uint32_t real_baudrate = 0;
uint8_t response_code = 0;
uint8_t vesc_connected = 0;

uint8_t get_values_response[100];

float adc_v1, adc_v2;

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

    // Set Pico DC/DC converter to PWM mode to reduce noise at light load
    gpio_put(23, 1);

    // Init debug GPIO
    gpio_init(DEBUG_GPIO);
    gpio_set_dir(DEBUG_GPIO, GPIO_OUT);
    gpio_put(DEBUG_GPIO, 1);

    // Init i2c and configure it's GPIO pins
    //uint32_t i2c_clk = i2c_init(i2c_default, 400 * 1000);
    uint32_t i2c_clk = i2c_init(i2c_default, 1000 * 1000);  // 1120 kHz max
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Instantiate debouncer and configure GPIO
    // gpio_pull_up(PB_LEFT_GPIO);
    // gpio_pull_up(PB_RIGHT_GPIO);

    debouncer.debounce_gpio(PB_LEFT_GPIO);
    debouncer.set_debounce_time(PB_LEFT_GPIO, 20.0);    // 20ms debounce time
    debouncer.debounce_gpio(PB_RIGHT_GPIO);
    debouncer.set_debounce_time(PB_RIGHT_GPIO, 20.0);

    // Set initial values of button states
    pb_left_prev_state = pb_left_state = debouncer.read(PB_LEFT_GPIO);
    pb_right_prev_state = pb_right_state = debouncer.read(PB_RIGHT_GPIO);
    
    display.oled_init();

    display.set_font(too_simple);
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
    // speedo.set_position(63, 120);
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

    sleep_ms(2000);    

    // Start core 1
    multicore_launch_core1(core1_entry);


    // Exercise bike display page
    analog_gauge batt_current(&display);
    batt_current.set_position(63, 120);
    batt_current.set_scale(0, 10, 250, 290);
    batt_current.set_markers(5, 106, 12, 1);

    char b_cur_str[20];
    uint8_t b_cur_txt_x, b_cur_txt_y, b_soc;

    #define ROLLING_AVG_RATIO 0.3

    float b_cur_avg = 0;
    float b_volts_avg = 0;
    float m_erpm_avg = 0;

    while(true)
    {
        display.fill(0);

        mutex_enter_blocking(&float_mutex);

        // Update rolling averages
        b_cur_avg = (1 - ROLLING_AVG_RATIO) * b_cur_avg + ROLLING_AVG_RATIO * abs(data_pt.current_in);
        b_volts_avg = (1 - ROLLING_AVG_RATIO) * b_volts_avg + ROLLING_AVG_RATIO * data_pt.v_in;
        m_erpm_avg = (1 - ROLLING_AVG_RATIO) * m_erpm_avg + ROLLING_AVG_RATIO * abs(data_pt.rpm);

        // Display battery voltage visually and numerically
        // 12V SLA, 0% SOC = 9V, 100% SOC = 12.6. 12.6 - 9.0 = 3.6V range
        b_soc = MIN(((b_volts_avg - 9.0) * 100 / 3.6), 100);
        display.draw_vbar(b_soc, 0, 12, 15, 63);
        display.fill_rect(0, 4, 9, 11, 11); // Draw block to represent battery terminal

        display.set_cursor(2, 0);
        display.print_num("%.1fV", b_volts_avg);

        // Display battery current visually and numerically
        batt_current.set_value(b_cur_avg);
        batt_current.draw();
        sprintf(b_cur_str, "%.1fA", b_cur_avg);
        display.get_str_dimensions(b_cur_str, &b_cur_txt_x, &b_cur_txt_y);


        // Blank out underneath text
        display.fill_rect(1, 63 - (b_cur_txt_x/2), 32, 63 + (b_cur_txt_x/2), 32 + 2*b_cur_txt_y);
        display.set_cursor(63 - (b_cur_txt_x/2), 32);
        display.print(b_cur_str);        

        // Display human input power
        sprintf(b_cur_str, "%.1fW", b_cur_avg * b_volts_avg);      
        display.set_cursor(63 - (b_cur_txt_x/2), 40);       
        display.print(b_cur_str);             


        // Display watt-hours charged numerically
        display.fill_rect(1, 18, 52, 117, 63);  // Blank out area where text will draw
        display.set_cursor(19, 54);
        display.print_num("WATT-HRS GENERATED: %.1f", data_pt.watt_hours_charged);

        // Display regen intensity visually (ADC2)
        display.set_cursor(123, 10);
        display.print("R\nE\nG\nE\nN");
        display.draw_vbar(data_pt.adc2_decoded*100, 121, 42, 127, 63);


        // Display FET temperature
        display.set_cursor(24, 0);
        display.print_num("FETS: %2.0fC", data_pt.temp_mos);

        // Display RPM
        // Convert to pedal cadence 
        // SK3 6374 has 14 pole pairs (unconfirmed)
        // RPM = ERPM / Pole Pairs
        // Motor to pedal reduction ratio is 21.4452:1
        display.set_cursor(64, 0);
        display.print_num("PEDALS: %.0fRPM", float(m_erpm_avg / (21.4452 * 14)));

        mutex_exit(&float_mutex);
        display.render();
    }

    while(true)
    {
        display.fill(0);

        display.set_cursor(0,0);

        
        mutex_enter_blocking(&float_mutex);


        // display.print_num(" VIN: %.1fV\n", data_pt.v_in);
        // display.print_num("TFET: %.1fC\n", data_pt.temp_mos);
        // display.print_num("TMOT: %.1fC\n", data_pt.temp_motor);        
        // display.print_num("ERPM: %d\n", data_pt.rpm);  
        //display.print_num("ADC1_RAW: %.1fV\n", adc_v1);
        //display.print_num("ADC1_DEC: %.3f\n", data_pt.adc1_decoded);
        //display.print_num("ADC2_RAW: %.1fV\n", adc_v2);
        //display.print_num("ADC2_DEC: %.3f\n", data_pt.adc2_decoded);

        display.draw_vbar(data_pt.adc1_decoded*100, 4, 20, 8, 60);

        display.draw_hbar(data_pt.adc1_decoded*100, 1, 20, 20, 60, 24);
        display.draw_hbar(data_pt.adc1_decoded*100, 0, 70, 20, 110, 24);        

        display.print_num("BRIGHTNESS: %d\n", nv_settings.data.disp_brightness);
        display.print_num("FLASH PG=%d", nv_settings.page_id);
        display.print_num("    BLOCK=%d\n\r", nv_settings.block_id);
        display.print(vesc_connected ? "VESC CONNECTED" : "VESC NOT CONNECTED");

        // char hex_str[10];
        // uint8_t cols = 0;
        // for (uint8_t idx = 0; idx < 20; idx++)
        // {
        //     if (cols++ == 4)
        //     {
        //         cols = 0;
        //         display.print("\n");
        //     }

        //     sprintf(hex_str, "%.2X ", get_values_response[idx]);
        //     display.print(hex_str);            
        // }        

        mutex_exit(&float_mutex);

        // Process inputs
        pb_left_state = debouncer.read(PB_LEFT_GPIO);
        pb_right_state = debouncer.read(PB_RIGHT_GPIO);

        // Draw the state of the buttons
        if (pb_left_state)
            display.fill_rect(0, 80, 30, 100, 50);
        else
            display.draw_box(80, 30, 100, 50);

        if (pb_right_state)
            display.fill_rect(0, 105, 30, 125, 50);
        else
            display.draw_box(105, 30, 125, 50);

        // If left PB was just pressed
        if (pb_left_state && !pb_left_prev_state)
        {
            nv_settings.data.disp_brightness -= 16;
            display.set_brightness(nv_settings.data.disp_brightness);
            nv_settings.store_data();
        }

        // If right PB was just pressed
        if (pb_right_state && !pb_right_prev_state)
        {
            nv_settings.data.disp_brightness += 16;
            display.set_brightness(nv_settings.data.disp_brightness);       
            nv_settings.store_data();                 
        }

        pb_left_prev_state = pb_left_state;
        pb_right_prev_state = pb_right_state;


        draw_SD_status(118,0);

        display.render();

    }
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




