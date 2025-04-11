#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include <hw_def.h>
#include <u8g2.h>
#include "spi_tx_9bit.pio.h"
#include "u8x8_interface.hpp"

PIO pio_3wire;
uint pio_3wire_sm;
uint pio_3wire_offset;

// Handle GPIO initialization and control
uint8_t __not_in_flash_func(u8x8_gpio_and_delay_pico)(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static uint32_t f_clk_sys_khz;
    static uint8_t delay_div;
    pio_sm_config cnf;

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

        // Check system clock
        f_clk_sys_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
        delay_div = f_clk_sys_khz / (125000 / 24);   // factor to divide arg_int by, minimum delay of busy wait loop in ns

        // Check if pio0 has an sm available
        pio_3wire = pio0;
        pio_3wire_sm = pio_claim_unused_sm(pio_3wire, false);
        if (pio_3wire_sm == -1 || !pio_can_add_program(pio_3wire, &spi_tx_9bit_program))
        {
            // Couldn't put the program on pio0, try pio1
            pio_3wire = pio1;
            pio_3wire_sm = pio_claim_unused_sm(pio_3wire, false);
        }

        // If there's a free sm
        if (pio_3wire_sm != -1)
        {
            // Try to find an offset to put the program
            for (pio_3wire_offset = 0; pio_3wire_offset < 31; pio_3wire_offset++)
            {
                if (pio_can_add_program_at_offset(pio_3wire, &spi_tx_9bit_program, pio_3wire_offset))
                {
                    pio_add_program_at_offset(pio_3wire, &spi_tx_9bit_program, pio_3wire_offset);
                    break;
                }
            }
        }

        // Don't init PIO if bitbanging GPIO is happening
        #ifndef U8G2_USE_SW_SPI        
        pio_gpio_init(pio_3wire, DISPLAY_SDA_GPIO);
        pio_gpio_init(pio_3wire, DISPLAY_SCL_GPIO);
        pio_sm_set_consecutive_pindirs(pio_3wire, pio_3wire_sm, DISPLAY_SDA_GPIO, 1, true);
        pio_sm_set_consecutive_pindirs(pio_3wire, pio_3wire_sm, DISPLAY_SCL_GPIO, 1, true);
        cnf = spi_tx_9bit_program_get_default_config(pio_3wire_offset);
        sm_config_set_sideset_pins(&cnf, DISPLAY_SCL_GPIO);
        sm_config_set_out_pins(&cnf, DISPLAY_SDA_GPIO, 1);

        // Only TX fifo is needed since we will only be transmitting
        sm_config_set_fifo_join(&cnf, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&cnf, (float)f_clk_sys_khz / (2.0*PIO_3WIRE_CLK_KHZ));

        // Shift left (MSB first), autopull enabled, autopull after 9 bits have been transmitted
        sm_config_set_out_shift(&cnf, false, true, 9);

        pio_sm_init(pio_3wire, pio_3wire_sm, pio_3wire_offset, &cnf);
        pio_sm_set_enabled(pio_3wire, pio_3wire_sm, true);
        #endif

        break;

    case U8X8_MSG_DELAY_NANO: // delay arg_int * 1 nano second
        // Can't do delays as short as a nanosecond, pico clock defaults to 125MHz, 8ns per cycle
        arg_int /= delay_div;
        while(arg_int--);
        // sleep_us(arg_int);
        break;

    case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
        sleep_us(arg_int/10);
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


// Communication handler to provide to U8G2 library, using PIO for HW 3wire SPI
uint8_t __not_in_flash_func(u8x8_byte_pio_3wire_spi)(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    uint8_t i;
    uint8_t *data;
    uint8_t takeover_edge = u8x8_GetSPIClockPhase(u8x8);
    uint8_t not_takeover_edge = 1 - takeover_edge;
    uint32_t b;
    uint32_t sm_stall_mask;
    static uint8_t last_dc;

    switch (msg)
    {
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            while (arg_int > 0)
            {
                b = *data;

                // Set b8 if required and shift left so that b8 ends up as MSB b31
                if (last_dc != 0)
                    b |= 256;
                b <<= 23;

                data++;
                arg_int--;

                // Wait until there's room and add the data to the PIO's FIFO
                pio_sm_put_blocking(pio_3wire, pio_3wire_sm, b);
                
            }
            break;

        case U8X8_MSG_BYTE_INIT:
            u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
            // Normally, SCK level is set here, but the PIO starts with SCK low so this shouldn't be needed
            break;

        case U8X8_MSG_BYTE_SET_DC:
            last_dc = arg_int;
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
            u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->post_chip_enable_wait_ns, NULL);
            break;

        case U8X8_MSG_BYTE_END_TRANSFER:
            // Wait for PIO to finish transmitting
            sm_stall_mask = 1u << (pio_3wire_sm + PIO_FDEBUG_TXSTALL_LSB);
            pio_3wire->fdebug = sm_stall_mask;
            while (!(pio_3wire->fdebug & sm_stall_mask));

            u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->pre_chip_disable_wait_ns, NULL);
            u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
            break;
        
        default:
            return 0;
    }
    return 1;
}



