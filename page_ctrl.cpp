#include <u8g2.h>
#include "page_ctrl.hpp"
#include "hw_def.h"
#include "pico/cyw43_arch.h"


page_controller::page_controller(void)
{
    // Button debouncing init
    btn_gpio[PB_LEFT] = PB_LEFT_GPIO;
    btn_gpio[PB_RIGHT] = PB_RIGHT_GPIO;
    btn_gpio[PB_CONFIRM] = PB_CENTER_GPIO;

    for (uint8_t i = 0; i < BUTTON_COUNT; i++)
    {
        btn_lockouts[i] = get_absolute_time();  // Initialize lockout timestamps with current time
        btn_state[i] = gpio_get(btn_gpio[i]);
    }

    // Add page functions to list
    page_fn[0] = (page_draw_fn) &page_controller::page_main_draw;
    page_idx = 0;
}


// Handle common tasks per frame that are needed by page drawing functions
void page_controller::update(void)
{
    uint8_t btn_new_state[BUTTON_COUNT];
    absolute_time_t btn_poll_time;

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

    // Switch pages
}

// Call the draw function for the current page
void page_controller::draw_page(void)
{
    page_fn[page_idx]();
}

// 
void page_controller::page_main_draw(void)
{
    cyw43_arch_gpio_put(CYW43_LED_GPIO, 0);   

    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_10x20_tf);    
    u8g2_DrawStr(&u8g2, 10, 64, "Main Page");
    u8g2_SendBuffer(&u8g2);

    cyw43_arch_gpio_put(CYW43_LED_GPIO, 1);  
}