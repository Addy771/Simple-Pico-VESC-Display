/**
 *
 */

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
#include "pico-oled/font/press_start_2p.h"
#include "pico-oled/font/too_simple.h"
#include "nv_flash.hpp"

#include "datatypes.h"
#include "packet.h"
#include "buffer.h"
#include "crc.h"

#include "util_enum.h"
#include <string.h>
#include "Button-debouncer/button_debounce.h"

#define DISPLAY_I2C_ADDR _u(0x3C)
#define DISPLAY_WIDTH _u(128)
#define DISPLAY_HEIGHT _u(64)

#define UART_BAUDRATE 115200
#define UART_TX_GPIO 0          // UART0 TX default pin is GP0 (Pico pin 1)
#define UART_RX_GPIO 1          // UART0 RX default pin is GP1 (Pico pin 2)
#define DEBUG_GPIO 22           // Timing measurement debug output
#define PB_LEFT_GPIO 21         // Left-side pushbutton
#define PB_RIGHT_GPIO 20        // Right-side pushbutton

#define log_sample_rate 10      // How often to sample data from VESC (in Hz)

void core1_entry();

uint32_t real_baudrate = 0;
uint8_t response_code = 0;

uint8_t get_values_response[100];

mc_values rt_data;
float adc_v1, adc_v2;
float adc_decoded1, adc_decoded2;

uint8_t pb_left_state, pb_left_prev_state;
uint8_t pb_right_state, pb_right_prev_state;

auto_init_mutex(float_mutex);
auto_init_mutex(flash_lock);

nv_flash_storage nv_settings(&flash_lock);


// Core 0
int main()
{

    //stdio_init_all();

    // Init i2c and configure it's GPIO pins
    //uint32_t i2c_clk = i2c_init(i2c_default, 400 * 1000);
    uint32_t i2c_clk = i2c_init(i2c_default, 1000 * 1000);  // 1120 kHz max
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Instantiate debouncer and configure GPIO
    Debounce debouncer;
    // gpio_pull_up(PB_LEFT_GPIO);
    // gpio_pull_up(PB_RIGHT_GPIO);

    debouncer.debounce_gpio(PB_LEFT_GPIO);
    debouncer.set_debounce_time(PB_LEFT_GPIO, 20.0);    // 20ms debounce time

    debouncer.debounce_gpio(PB_RIGHT_GPIO);
    debouncer.set_debounce_time(PB_RIGHT_GPIO, 20.0);

    // Set initial values of button states
    pb_left_prev_state = pb_left_state = debouncer.read(PB_LEFT_GPIO);
    pb_right_prev_state = pb_right_state = debouncer.read(PB_RIGHT_GPIO);
    
    // Instantiate display and initialize it
    pico_oled display(OLED_SSD1309, DISPLAY_I2C_ADDR, DISPLAY_WIDTH, DISPLAY_HEIGHT, /*reset_gpio=*/ 15); 
    display.oled_init();

    display.set_font(too_simple);
    //display.set_font(press_start_2p);

    // Print out some system information
    uint32_t f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);



    // Show a page of debug values
    display.set_cursor(0,0);
    display.print("Pico SVD\n");
    // display.print("CLK_SYS:  ");
    // display.print_num("%d kHz\n", f_clk_sys);
    // display.print("Baudrate: ");
    // display.print_num("%d Bps\n", real_baudrate);
    

    display.render();
    sleep_ms(2000);    

    // uint8_t box_w = 4;
    // uint8_t box_h = 4;

    // display.fill_rect(0, 5, 20, 8, 23);
    // display.draw_fast_hline(10, 20, 20);
    // display.draw_fast_hline(10, 20, 23);

    // display.draw_box(5, 30, 8, 33);

    // display.render();

    // for(;;);


    // Start core 1
    multicore_launch_core1(core1_entry);

    while(true)
    {
        display.fill(0);

        display.set_cursor(0,0);

        
        mutex_enter_blocking(&float_mutex);


        // display.print_num(" VIN: %.1fV\n", rt_data.v_in);
        // display.print_num("TFET: %.1fC\n", rt_data.temp_mos);
        // display.print_num("TMOT: %.1fC\n", rt_data.temp_motor);        
        // display.print_num("ERPM: %d\n", rt_data.rpm);  
        //display.print_num("ADC1_RAW: %.1fV\n", adc_v1);
        //display.print_num("ADC1_DEC: %.3f\n", adc_decoded1);
        //display.print_num("ADC2_RAW: %.1fV\n", adc_v2);
        //display.print_num("ADC2_DEC: %.3f\n", adc_decoded2);

        display.draw_vbar(adc_decoded1*100, 4, 20, 8, 60);

        display.draw_hbar(adc_decoded1*100, 1, 20, 20, 60, 24);
        display.draw_hbar(adc_decoded1*100, 0, 70, 20, 110, 24);        

        display.print_num("BRIGHTNESS: %d\n", nv_settings.data.disp_brightness);
        display.print_num("FLASH PG=%d", nv_settings.page_id);
        display.print_num("    BLOCK=%d", nv_settings.block_id);

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

        response_code = 1;
    }
    else
    {
        response_code = 0;
    }

    return response_code;
}

void process_data(uint8_t *data, size_t len);


// Second core thread. Handles UART communication
void core1_entry()
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    // uint8_t LED_STATUS = 1;

    // Set the LED pin as an output
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

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
        next_sample = delayed_by_ms(next_sample, 1000 / log_sample_rate);

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
        receive_packet(&vesc_comm);

        
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

            // Unpack data from VESC response
            rt_data.temp_mos = buffer_get_float16(data, 1e1, &idx);
            rt_data.temp_motor = buffer_get_float16(data, 1e1, &idx);
            rt_data.current_motor = buffer_get_float32(data, 1e2, &idx);
            rt_data.current_in = buffer_get_float32(data, 1e2, &idx);
            rt_data.id = buffer_get_float32(data, 1e2, &idx);
            rt_data.iq = buffer_get_float32(data, 1e2, &idx);
            rt_data.duty_now = buffer_get_float16(data, 1e3, &idx);
            rt_data.rpm = buffer_get_float32(data, 1e0, &idx);
            rt_data.v_in = buffer_get_float16(data, 1e1, &idx);
            rt_data.amp_hours = buffer_get_float32(data, 1e4, &idx);
            rt_data.amp_hours_charged = buffer_get_float32(data, 1e4, &idx);
            rt_data.watt_hours = buffer_get_float32(data, 1e4, &idx);
            rt_data.watt_hours_charged = buffer_get_float32(data, 1e4, &idx);
            rt_data.tachometer = buffer_get_int32(data, &idx);
            rt_data.tachometer_abs = buffer_get_int32(data, &idx);
            rt_data.fault_code = data[idx++];
            rt_data.position = buffer_get_float32(data, 1e6, &idx);
            rt_data.vesc_id = data[idx++];
            rt_data.temp_mos_1 = buffer_get_float16(data, 1e1, &idx);
            rt_data.temp_mos_2 = buffer_get_float16(data, 1e1, &idx);
            rt_data.temp_mos_3 = buffer_get_float16(data, 1e1, &idx);
            rt_data.vd = buffer_get_float32(data, 1e3, &idx);
            rt_data.vq = buffer_get_float32(data, 1e3, &idx);

            //last byte is int8 status, but rt_data struct has no place for it

            mutex_exit(&float_mutex);
            break;

        case COMM_GET_DECODED_ADC:

            mutex_enter_blocking(&float_mutex);
            adc_decoded1 = buffer_get_float32(data, 1e6, &idx);
            adc_v1 = buffer_get_float32(data, 1e6, &idx);
            adc_decoded2 = buffer_get_float32(data, 1e6, &idx);
            adc_v2 = buffer_get_float32(data, 1e6, &idx);
            mutex_exit(&float_mutex);

            break;

    }

}




