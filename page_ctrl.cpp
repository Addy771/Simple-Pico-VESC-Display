#include <u8g2.h>
#include "page_ctrl.hpp"
#include "hw_def.h"
#include "pico/cyw43_arch.h"
#include "pico/bootrom.h"
#include "pico/sync.h"
#include "nv_flash.hpp"

page_controller::page_controller(void)
{
    mutex_init(&float_mutex);   // Only one core can use float ROM functions at a time
    mutex_init(&flash_mutex);   // Other core must halt when flash is being written
    
    // Button debouncing init
    btn_gpio[PB_LEFT] = PB_LEFT_GPIO;
    btn_gpio[PB_RIGHT] = PB_RIGHT_GPIO;
    btn_gpio[PB_CONFIRM] = PB_CENTER_GPIO;

    for (uint8_t i = 0; i < BUTTON_COUNT; i++)
    {
        btn_lockouts[i] = get_absolute_time();  // Initialize lockout timestamps with current time
        btn_state[i] = gpio_get(btn_gpio[i]);
    }

    // External load control init
    load_gpio[LOAD_A] = EXT_LOAD_A_GPIO;
    load_gpio[LOAD_B] = EXT_LOAD_B_GPIO;

    // Read load mode settings from flash
    // Init flash storage system
    nv_settings.init(&flash_mutex);
    load_mode[LOAD_A] = nv_settings.data.load_modes & 0x0F;         // Lower nibble has A channel mode
    load_mode[LOAD_B] = (nv_settings.data.load_modes & 0xF0) >> 4;  // Upper nibble has B channel mode

    // Add page functions to list
    page_fn[0] = (page_draw_fn) &page_controller::page_main_draw;
    page_fn[1] = (page_draw_fn) &page_controller::page_log_draw;
    page_idx = 0;

    // Log system init
    u8log_Init(&u8log, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buf);
    mutex_init(&log_mutex);
}


// Handle common tasks per frame that are needed by page drawing functions
void page_controller::update(void)
{
    uint8_t btn_new_state[BUTTON_COUNT];
    absolute_time_t btn_poll_time;

    // Handle load output tasks
    update_load_outputs();

    // Process button inputs
    btn_poll_time = get_absolute_time();    
    for (uint8_t i = 0; i < BUTTON_COUNT; i++)
    {
        btn_new_state[i] = gpio_get(btn_gpio[i]);
        btn_pressed[i] = 0; // Clear button presses before detecting them 
        
        // If state changed and button isn't locked out
        if (btn_new_state[i] != btn_state[i] && absolute_time_diff_us(btn_poll_time, btn_lockouts[i]) > BUTTON_LOCK_TIME_MS*1000)
        {
            // Indicate button was pressed
            if (!btn_new_state[i] && btn_state[i])
                btn_pressed[i] = 1;
            
            btn_state[i] = btn_new_state[i];
            btn_lockouts[i] = btn_poll_time;
        }
    }      
    
    // Check for bootloader button combo
    if 
    (
        !btn_state[PB_LEFT] 
        && !btn_state[PB_RIGHT]
        && absolute_time_diff_us(btn_poll_time, btn_lockouts[PB_LEFT]) > BOOTLOADER_BUTTON_TIME_MS*1000
        && absolute_time_diff_us(btn_poll_time, btn_lockouts[PB_RIGHT]) > BOOTLOADER_BUTTON_TIME_MS*1000
    )
    {
        // Inform user that bootloader mode is being entered
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_10x20_tf);    
        u8g2_DrawStr(&u8g2, 10, 64, "Entering Bootloader Mode");
        u8g2_SendBuffer(&u8g2);
        sleep_ms(1000);

        // Put up something on the screen to indicate bootloader mode
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_10x20_tf);    
        u8g2_DrawStr(&u8g2, 10, 64, "Bootloader Active");
        u8g2_SendBuffer(&u8g2);        

        reset_usb_boot(0,0);
    }

    // Switch pages
}


// Update load GPIO as necessary
void page_controller::update_load_outputs(void)
{
    for (uint8_t i = 0; i < EXT_LOAD_COUNT; i++)
    {
        switch (load_mode[i])
        {
            case LOAD_MODE_NONE:            
                gpio_put(load_gpio[i], 0);
                break;

            case LOAD_MODE_ALWAYS_ON:     
                gpio_put(load_gpio[i], 1);
                break;      

            case LOAD_MODE_ALWAYS_ON_BLINK:      
            case LOAD_MODE_AUTO_LIGHT_AT_DUSK:   
            case LOAD_MODE_BRAKE_LIGHT:  
                break;        
        }

    }
}


// Wrapper for external access
void page_controller::log_write_string(char *log_str)
{
    mutex_enter_blocking(&log_mutex);
    u8log_WriteString(&u8log, log_str); 
    mutex_exit(&log_mutex);
}


// Call the draw function for the current page
void page_controller::draw_page(void)
{
    page_fn[page_idx]();
}


// 
void page_controller::page_main_draw(void)
{
    u8g2_ClearBuffer(&u8g2);

    u8g2_SetFont(&u8g2, u8g2_font_fub42_tn);    
    u8g2_DrawStr(&u8g2, 10, 50, "120");

    u8g2_SetFont(&u8g2, u8g2_font_logisoso38_tn);
    u8g2_DrawStr(&u8g2, 130, 50, "120");
    
    u8g2_SetFont(&u8g2, u8g2_font_fub35_tn);
    u8g2_DrawStr(&u8g2, 10, 110, "120");

    u8g2_SendBuffer(&u8g2);
}


//
void page_controller::page_log_draw(void)
{
    // Draw common frame
    // Populate button descriptions

    u8g2_DrawLog(&u8g2, 0, 12, &u8log);    // Draw log text area
}