#include <u8g2.h>
#include "page_ctrl.hpp"
#include "hw_def.h"
#include "pico/cyw43_arch.h"
#include "pico/bootrom.h"
#include "pico/sync.h"
#include "nv_flash.hpp"
#include "log.hpp"
#include <cstdarg>
#include <math.h>

#include "bitmap/sd_ok.xbm"
#include "bitmap/sd_err.xbm"
#include "bitmap/sd_none.xbm"
#include "bitmap/batt_frame.xbm"
#include "bitmap/batt_unknown.xbm"
#include "bitmap/esc_connected.xbm"
#include "bitmap/esc_disconnected.xbm"


volatile extern uint8_t vesc_connected;


// Scale value and add SI prefixes to keep length of number printout short
void format_with_si(float value, char *out_buf, size_t buf_len, const char *unit)
{
    static const char* si_prefixes[] = {"u", "m", "", "k", "M"};
    int8_t exp = 0;

    // multiply or divide value as necessary until it's between 1.0 and 1000.0
    while (fabs(value) >= 1000.0 && exp < 2)
    {
        value /= 1000.0;
        exp++;
    }
    while (fabs(value) <= 1.0 && exp > -2)
    {
        value *= 1000.0;
        exp--;
    }

    snprintf(out_buf, buf_len, "%.1f%s%s", value, si_prefixes[exp + 2], unit);
}

// Compute speed using data and config values from ESC
inline float page_controller::calc_speed_kph()
{
    if (motor_poles <= 0 || wheel_diameter <= 0.0f || config_received == 0) return 0.0f;

    uint8_t pole_pairs = motor_poles / 2;
    float circumference = wheel_diameter * M_PI;
    float mech_rpm = static_cast<float>(esc_data.rpm) / pole_pairs;
    float mech_rps = mech_rpm / 60.0f;
    float speed_m_per_s = mech_rps * circumference;
    float speed_kph = speed_m_per_s * 3.6f; // speed_m_per_s * 3600 seconds / 1000m
    return speed_kph;

}

page_controller::page_controller(void)
{
    absolute_time_t btn_init;
    mutex_init(&float_mutex);   // Only one core can use float ROM functions at a time
    mutex_init(&flash_mutex);   // Other core must halt when flash is being written
    
    // Button debouncing init
    btn_gpio[PB_LEFT] = PB_LEFT_GPIO;
    btn_gpio[PB_RIGHT] = PB_RIGHT_GPIO;
    btn_gpio[PB_CONFIRM] = PB_CENTER_GPIO;
    btn_init = get_absolute_time();

    for (uint8_t i = 0; i < BUTTON_COUNT; i++)
    {
        btn_lockouts[i] = btn_init;  // Initialize lockout timestamps with current time
        btn_state[i] = gpio_get(btn_gpio[i]);
        btn_held[i] = 0;
        btn_pressed[i] = 0;
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

    // Initialize config with reasonable values
    config_received = 0;
    motor_poles = 46;
    wheel_diameter = 0.66;

    esc_data.battery_level = 0.0;

    // Set ratio of weighted moving averages
    v_in_smoothed.set_ratio(0.7);
    speed_smoothed.set_ratio(0.3);
}


// Handle common tasks per frame that are needed by page drawing functions
void page_controller::update(void)
{
    uint8_t btn_new_state[BUTTON_COUNT];
    absolute_time_t btn_poll_time;
    uint btn_time_diff;
    float raw_speed_kph;


    // Handle load output tasks
    update_load_outputs();

    // Process button inputs
    btn_poll_time = get_absolute_time();    
    for (uint8_t i = 0; i < BUTTON_COUNT; i++)
    {
        btn_time_diff = absolute_time_diff_us(btn_lockouts[i], btn_poll_time);
        btn_new_state[i] = gpio_get(btn_gpio[i]);
        btn_pressed[i] = 0; // Clear button presses before detecting them 
        
        // If state changed and button isn't locked out
        if ((btn_new_state[i] != btn_state[i]) && btn_time_diff > BUTTON_LOCK_TIME_MS*1000)
        {
            // Indicate button was pressed
            if (!btn_new_state[i] && btn_state[i])
                btn_pressed[i] = 1;
            
            btn_lockouts[i] = btn_poll_time;
            btn_time_diff = 0;  // Reset time difference since button was just pressed
        }
        btn_state[i] = btn_new_state[i];       
        
        // Check if button was held
        if (btn_state[i] == 0 && btn_time_diff > BUTTON_LONG_PRESS_MS*1000)
            btn_held[i] = 1;
        else
            btn_held[i] = 0;
    }      
    
    // Check for bootloader button combo
    if 
    (
        !btn_state[PB_LEFT] 
        && !btn_state[PB_RIGHT]
        && absolute_time_diff_us(btn_lockouts[PB_LEFT], btn_poll_time) > BOOTLOADER_BUTTON_TIME_MS*1000
        && absolute_time_diff_us(btn_lockouts[PB_RIGHT], btn_poll_time) > BOOTLOADER_BUTTON_TIME_MS*1000
    )
    {
        // Inform user that bootloader mode is being entered
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_10x20_tf);    
        draw_string(10, 64, "Entering Bootloader Mode");
        u8g2_SendBuffer(&u8g2);
        sleep_ms(1000);

        // Put up something on the screen to indicate bootloader mode
        u8g2_ClearBuffer(&u8g2);
        u8g2_SetFont(&u8g2, u8g2_font_10x20_tf);    
        u8g2_DrawStr(&u8g2, 10, 64, "Bootloader Active");
        u8g2_SendBuffer(&u8g2);   
        sleep_ms(200);     

        reset_usb_boot(0,0);
    }

    // Switch pages
    // If left PB is held and right PB is not, decrement page index if possible
    if (page_idx > 0 && btn_held[PB_LEFT] && btn_state[PB_RIGHT])
        page_idx--;
    else if (page_idx < PAGE_COUNT-1 && btn_held[PB_RIGHT] && btn_state[PB_LEFT])
        page_idx++;

    // Common calculations
    mutex_enter_blocking(&float_mutex);

    v_in_smoothed.update(esc_data.v_in);

    raw_speed_kph = calc_speed_kph();

    speed_smoothed.update(raw_speed_kph);
    //speed_smoothed.update(120.0);


    //     // Update rolling averages
    //     b_cur_avg = (1 - ROLLING_AVG_RATIO) * b_cur_avg + ROLLING_AVG_RATIO * data_pt.current_in;
    //     b_volts_avg = (1 - ROLLING_AVG_RATIO) * b_volts_avg + ROLLING_AVG_RATIO * data_pt.v_in;

    //     // for now hardcode 13S battery Full =54.6V empty = 39V (3.0/cell) (delta V = 15.6)
    //     b_soc = MIN(((b_volts_avg - 39.0) * 100 / 15.6), 100); // Get scale from min to max batt V
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
    mutex_exit(&float_mutex);     
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


// Draw a string with sprintf 
void page_controller::draw_string(uint16_t x_coord, uint16_t y_coord, const char* format, ...)
{
    char string_buf[100];
    va_list args;
    va_start(args, format);

    vsnprintf(string_buf, sizeof(string_buf), format, args);
    u8g2_DrawStr(&u8g2, x_coord, y_coord, string_buf);
}


// Draw the appropriate icon representing the status of the SD card
void page_controller::draw_SD_icon(uint16_t x_coord, uint16_t y_coord)
{
    switch (sd_status)
    {
        case SD_PRESENT:
            u8g2_DrawXBMP(&u8g2, x_coord, y_coord, sd_ok_width, sd_ok_height, sd_ok_bits);
            break;

        case SD_NOT_PRESENT:
            u8g2_DrawXBMP(&u8g2, x_coord, y_coord, sd_none_width, sd_none_height, sd_none_bits);
            break;

        case SD_ERROR:
            u8g2_DrawXBMP(&u8g2, x_coord, y_coord, sd_err_width, sd_err_height, sd_err_bits);
            break;

    }    
}


// Draw the appropriate icon representing the status of the connection to the ESC
void page_controller::draw_ESC_icon(uint16_t x_coord, uint16_t y_coord)
{
    if (vesc_connected)
    {
        u8g2_DrawXBMP(&u8g2, x_coord, y_coord, esc_connected_width, esc_connected_height, esc_connected_bits);
    }
    else
    {
        u8g2_DrawXBMP(&u8g2, x_coord, y_coord, esc_disconnected_width, esc_disconnected_height, esc_disconnected_bits);
    }
}


// Status overlay, covers top area of screen
void page_controller::draw_overlay_status(void)
{
    datetime_t rtc_time;
    // Border line
    u8g2_DrawHLine(&u8g2, 0, STATUS_HEIGHT - 1, u8g2_GetDisplayWidth(&u8g2));

    u8g2_SetFont(&u8g2, u8g2_font_t0_11_te);

    // VESC connected?
    // RTC connected?
    if (rtc_connected)
    {
        rtc_get_datetime(&rtc_time);
        draw_string(104, 9, "%02d:%02d:%02d", rtc_time.hour, rtc_time.min, rtc_time.sec);
    }
    else
    {
        draw_string(104, 9, "no clock");
    }

    // SD card working?
    draw_SD_icon(170, 0);


    // Wifi status?

    // Battery level/voltage
    if (vesc_connected)
    {
        u8g2_DrawXBMP(&u8g2, 0, 0, batt_frame_width, batt_frame_height, batt_frame_bits);
        // Fill bar according to battery level

        mutex_enter_blocking(&float_mutex);
        draw_string(30, 9, "%3.1fV", v_in_smoothed.get_value());
        mutex_exit(&float_mutex);
    }
    else
    {
        u8g2_DrawXBMP(&u8g2, 0, 0, batt_unknown_width, batt_unknown_height, batt_unknown_bits);
    }

}


//  Draw main page speed UI. Includes bar and text speed displays
void page_controller::draw_speed_bar(uint8_t x, uint8_t y, float speed)
{
    static const uint bar_max_speed = nv_settings.data.speed_bar_max;
    static const uint8_t bar_width = 120;   // 136
    static const uint8_t bar_height = 8;    // Height at the low end of the bar
    static const uint8_t axis_label_height = 14;
    static const uint8_t axis_tick_height = 5;
    static const float bar_x_2_term = 0.002; // x^2 term of parabola shape

    uint8_t bar_marker_spacing = bar_width / 4;
    uint8_t bar_marker_value = bar_max_speed / 4;
    uint8_t bar_max_height;
    uint8_t bar_filled_cnt;
    uint8_t bar_line_h;
    char axis_value[10];
    uint8_t axis_text_y;

    mutex_enter_blocking(&float_mutex);
    bar_max_height = bar_height + bar_x_2_term * bar_width*bar_width;

    // Draw main speed value text
    u8g2_SetFont(&u8g2, u8g2_font_logisoso38_tn);
    if (speed < 10)
        draw_string(x+22, y-55, "%1.2f", speed);
    else if (speed < 100)
        draw_string(x+22, y-55, "%2.1f", speed);
    else
        draw_string(x+22, y-55, "%3.0f", speed);

    bar_filled_cnt = (bar_width * speed) / bar_max_speed;

    // Fill bar with vertical lines according to speed value
    for (uint8_t bar_x = 0; bar_x < bar_filled_cnt; bar_x += 2)
    {
        bar_line_h = bar_height + bar_x*bar_x * bar_x_2_term;
        u8g2_DrawVLine(&u8g2, x+bar_x, y-axis_label_height-bar_line_h, bar_line_h);
    }    
    
    mutex_exit(&float_mutex);

    // Draw speed bar frame lines
    u8g2_DrawVLine(&u8g2, x, y-axis_label_height-bar_height, bar_height);    // Left edge
    u8g2_DrawHLine(&u8g2, x, y-axis_label_height, bar_width);     // Bottom edge

    u8g2_DrawVLine(&u8g2, x+bar_width, y-axis_label_height-bar_max_height, bar_max_height);  // Right edge

    // Draw axis marker lines and label texts
    u8g2_SetFont(&u8g2, u8g2_font_logisoso16_tr);
    draw_string(x+15, y-axis_label_height-18, "kph");   // Speed unit

    u8g2_SetFont(&u8g2, u8g2_font_t0_11_te);
    axis_text_y = y - axis_label_height + axis_tick_height + 2 + u8g2_GetAscent(&u8g2);

    // Left edge marker
    u8g2_DrawVLine(&u8g2, x, y-axis_label_height, axis_tick_height);    
    sprintf(axis_value, "%d", 0);
    draw_string(x-u8g2_GetStrWidth(&u8g2, axis_value)/2, axis_text_y, axis_value);    

    // 1/4 marker
    u8g2_DrawVLine(&u8g2, x+bar_marker_spacing, y-axis_label_height, axis_tick_height); 
    sprintf(axis_value, "%d", bar_marker_value);
    draw_string(x+bar_marker_spacing-u8g2_GetStrWidth(&u8g2, axis_value)/2, axis_text_y, axis_value);

    // 1/2 marker
    u8g2_DrawVLine(&u8g2, x+2*bar_marker_spacing, y-axis_label_height, axis_tick_height); 
    sprintf(axis_value, "%d", 2*bar_marker_value);
    draw_string(x+2*bar_marker_spacing-u8g2_GetStrWidth(&u8g2, axis_value)/2, axis_text_y, axis_value);    

    // 3/4 marker
    u8g2_DrawVLine(&u8g2, x+3*bar_marker_spacing, y-axis_label_height, axis_tick_height); 
    sprintf(axis_value, "%d", 3*bar_marker_value);
    draw_string(x+3*bar_marker_spacing-u8g2_GetStrWidth(&u8g2, axis_value)/2, axis_text_y, axis_value);    

    // Right edge marker
    u8g2_DrawVLine(&u8g2, x+bar_width, y-axis_label_height, axis_tick_height); 
    sprintf(axis_value, "%d", bar_max_speed);
    draw_string(x+bar_width-u8g2_GetStrWidth(&u8g2, axis_value)/2, axis_text_y, axis_value);    

}


// 
void page_controller::page_main_draw(void)
{
    static char print_buf[30];
    datetime_t rtc_time;

    u8g2_ClearBuffer(&u8g2);

    draw_speed_bar(67, 112, speed_smoothed.get_value());

    const uint8_t side_bar_y = 27;

    // Temperature bars
    static bar_graph FET_temp(&u8g2, 210, side_bar_y, 16, 46, 0, 120, RIGHT_TO_LEFT);
    static bar_graph MTR_temp(&u8g2, 210, side_bar_y+19, 16, 46, 0, 120, RIGHT_TO_LEFT);

    FET_temp.set_font(u8g2_font_t0_11_te);
    MTR_temp.set_font(u8g2_font_t0_11_te);
    
    // Temperature labels
    u8g2_SetFont(&u8g2, u8g2_font_helvR08_tf);
    draw_string(210, side_bar_y-19, "Temps \xB0" "C");
    draw_string(190, side_bar_y-4, "FET");
    draw_string(188, side_bar_y+15, "MTR");

    FET_temp.draw(esc_data.temp_mos);
    MTR_temp.draw(esc_data.temp_motor);

    // Motor current bar
    static bar_graph MTR_current(&u8g2, 0, side_bar_y, 16, 60, 0,  120, LEFT_TO_RIGHT);
    MTR_current.set_font(u8g2_font_t0_11_te);

    // Motor current label
    u8g2_SetFont(&u8g2, u8g2_font_helvR08_tf);
    draw_string(0, side_bar_y-19, "Motor Amps");

    MTR_current.draw(esc_data.current_motor);

    // Battery level bar
    static bar_graph batt_level(&u8g2, 0, side_bar_y+30, 16, 60, 0, 100, LEFT_TO_RIGHT);
    batt_level.set_font(u8g2_font_t0_11_te);

    // Battery level label
    u8g2_SetFont(&u8g2, u8g2_font_helvR08_tf);    
    draw_string(0, side_bar_y+11, "Battery (%%)");

    batt_level.draw(esc_data.battery_level);
    u8g2_DrawFrame(&u8g2, 59, side_bar_y+17, 5, 10);    // Button-tip to make bar look like a battery 

    // Power bar
    const uint8_t pwr_bar_h = 12;

    static bar_graph pwr_bar_neg(&u8g2, 0, 127, pwr_bar_h, 128, 0, 5000, RIGHT_TO_LEFT);
    static bar_graph pwr_bar_pos(&u8g2, 127, 127, pwr_bar_h, 127, 0, 5000, LEFT_TO_RIGHT);

    if (esc_data.p_in >= 0)
    {
        pwr_bar_neg.draw(0);
        pwr_bar_pos.draw(esc_data.p_in);
    }
    else
    {
        pwr_bar_neg.draw(fabs(esc_data.p_in));
        pwr_bar_pos.draw(0);        
    }


    // Text values
    // Battery power
    u8g2_SetFont(&u8g2, u8g2_font_helvB10_tf);   

    //format_with_si(esc_data.v_in, print_buf, sizeof(print_buf), "V");
    format_with_si(v_in_smoothed.get_value(), print_buf, sizeof(print_buf), "V");
    draw_string(0, side_bar_y+46, print_buf);

    format_with_si(esc_data.current_in, print_buf, sizeof(print_buf), "A");
    draw_string(0, side_bar_y+64, print_buf);

    format_with_si(esc_data.p_in, print_buf, sizeof(print_buf), "W");
    draw_string(0, side_bar_y+82, print_buf);

    // Odometer related
    uint8_t odo_start_x = 195;
    u8g2_SetFont(&u8g2, u8g2_font_helvR08_tf);    
    // draw_string(odo_start_x, side_bar_y+30, "Trip A | B");
    // draw_string(odo_start_x, side_bar_y+44, "1337  1337");
    draw_string(odo_start_x, side_bar_y+30, "Trip A: 1337");
    draw_string(odo_start_x, side_bar_y+44, "Trip B: 1337");    
    draw_string(odo_start_x, side_bar_y+58, "Odo: %.1f", esc_data.odometer);
    draw_string(odo_start_x+10, side_bar_y+72, "(km)");

    // Status / Error icons
    draw_SD_icon(70, 1);
    draw_ESC_icon(87, 0);
    // Wifi?
    // Load outputs?

    if (rtc_connected)
    {
        rtc_get_datetime(&rtc_time);
        u8g2_SetFont(&u8g2, u8g2_font_t0_11_te);      
        draw_string(113, 9, "%02d:%02d", rtc_time.hour, rtc_time.min);
    }


    u8g2_SendBuffer(&u8g2);
}


// Log page, shows console text
void page_controller::page_log_draw(void)
{
    u8g2_ClearBuffer(&u8g2);
    draw_overlay_status();
    // Populate button descriptions

    // Set font for log text
    u8g2_SetFont(&u8g2, u8g2_font_t0_11_te);

    mutex_enter_blocking(&log_mutex);
    u8g2_DrawLog(&u8g2, 0, 24, &u8log);    // Draw log text area
    mutex_exit(&log_mutex);

    u8g2_SendBuffer(&u8g2);
}