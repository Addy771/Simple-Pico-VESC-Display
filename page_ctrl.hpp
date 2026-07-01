#include <u8g2.h>
#include <mui_u8g2.h>
#include <mui.h>
#include "pico/stdlib.h"
#include "hw_def.h"
#include "pico/sync.h"
#include "nv_flash.hpp"
#include "log.hpp"
#include <cstdarg>
#include <cstring>
#include <cstdio>
#include <math.h>

#ifndef PAGE_CTRL_H
#define PAGE_CTRL_H

#define PAGE_COUNT 4
#define BUTTON_COUNT 3  // left, right buttons and center as confirm
#define EXT_LOAD_COUNT 2    // channel A, B

#define BUTTON_LOCK_TIME_MS 50  // After a button is pressed, it won't be checked again until this much time has passed. 

#define U8LOG_WIDTH 42
#define U8LOG_HEIGHT 11
#define STATUS_HEIGHT 12

typedef void (*page_draw_fn)();
extern float raw_speed_kph;
extern uint8_t motor_poles;
extern float gear_ratio;
extern float wheel_diameter;
extern uint8_t config_received;

typedef enum
{
    PB_LEFT = 0,
    PB_RIGHT = 1,
    PB_CONFIRM = 2
} btn_map;

typedef enum
{
    LOAD_A,
    LOAD_B
} ext_load_map;


template<typename TYPE>
class moving_avg
{
    private:
        TYPE data_val;
        float new_ratio;
        float prev_ratio;

    public:
        moving_avg() : data_val(), prev_ratio(0.5), new_ratio(0.5) {}

        void set_ratio(float ratio)
        {
            new_ratio = ratio;
            prev_ratio = 1 - ratio;
        }

        void update(TYPE new_val)
        {
            data_val = new_ratio * new_val + prev_ratio * data_val;
        }

        TYPE get_value(void) const
        {
            return data_val;
        }

};


typedef enum
{
    LEFT_TO_RIGHT,
    RIGHT_TO_LEFT,
    BOTTOM_TO_TOP,
    TOP_TO_BOTTOM
} bar_orientation;

#define BAR_UNITS_BUF 10

class bar_graph
{
    private:
        u8g2_t *u8g2;  
        uint x;
        uint y;
        uint width;
        uint height;
        int min_val;
        int max_val;
        uint8_t orientation;
        char unit_str[BAR_UNITS_BUF];
        u8g2_t *value_font;

    public:       
        bar_graph(u8g2_t *u8, uint origin_x, uint origin_y, uint bar_width, uint bar_height, int min_value, int max_value, bar_orientation bar_type) : 
        u8g2(u8), 
        x(origin_x), 
        y(origin_y), 
        width(bar_width), 
        height(bar_height), 
        min_val(min_value), 
        max_val(max_value), 
        orientation(bar_type),
        value_font(NULL) 
        {}

        void draw(int bar_value)
        {
            int filled_px = std::min((height - 4) * bar_value / (max_val - min_val), height - 4);
            uint bar_center_x;
            uint bar_center_y;
            char text_buf[BAR_UNITS_BUF+10];

            if (value_font != NULL)
                u8g2_SetFont(u8g2, (uint8_t *)value_font);
            u8g2_SetFontMode(u8g2, 1);      // Transparent mode

            // Draw outline using a frame
            // Draw filled portion using a solid box
            // Bar origins are the lower left corners
            switch (orientation)
            {
                case LEFT_TO_RIGHT:
                    u8g2_DrawFrame(u8g2, x, y-width, height, width);
                    u8g2_DrawBox(u8g2, x+2, y+2-width, filled_px, width-4);
                    if (value_font != NULL)
                    {
                        u8g2_SetDrawColor(u8g2, 2);     // XOR mode
                        bar_center_x = x + height/2;
                        bar_center_y = y - width/2;
                        snprintf(text_buf, sizeof(text_buf), "%d", bar_value);

                        u8g2_DrawStr(u8g2, bar_center_x-u8g2_GetStrWidth(u8g2, text_buf)/2, bar_center_y+u8g2_GetAscent(u8g2)/2, text_buf);
                    }     
                    break;

                case RIGHT_TO_LEFT:
                    u8g2_DrawFrame(u8g2, x, y-width, height, width);
                    u8g2_DrawBox(u8g2, x+2+(height-4-filled_px), y+2-width, filled_px, width-4);

                    if (value_font != NULL)
                    {
                        u8g2_SetDrawColor(u8g2, 2);     // XOR mode
                        bar_center_x = x + height/2;
                        bar_center_y = y - width/2;
                        snprintf(text_buf, sizeof(text_buf), "%d", bar_value);

                        u8g2_DrawStr(u8g2, bar_center_x-u8g2_GetStrWidth(u8g2, text_buf)/2, bar_center_y+u8g2_GetAscent(u8g2)/2, text_buf);
                    }                    
                    break;   
                    
                case BOTTOM_TO_TOP:
                    u8g2_DrawFrame(u8g2, x, y-height, width, height);
                    u8g2_DrawBox(u8g2, x+2, y-2-filled_px, width-4, filled_px);

                    if (value_font != NULL)
                    {
                        u8g2_SetDrawColor(u8g2, 2);     // XOR mode
                        bar_center_x = x + width/2;
                        bar_center_y = y - height/2;
                        snprintf(text_buf, sizeof(text_buf), "%d", bar_value);

                        u8g2_DrawStr(u8g2, bar_center_x-u8g2_GetStrWidth(u8g2, text_buf)/2, bar_center_y+u8g2_GetAscent(u8g2)/2, text_buf);
                    }
                    break;                

                case TOP_TO_BOTTOM:
                    u8g2_DrawFrame(u8g2, x, y-height, width, height);
                    u8g2_DrawBox(u8g2, x+2, y-height+2, width-4, filled_px);

                    if (value_font != NULL)
                    {
                        u8g2_SetDrawColor(u8g2, 2);     // XOR mode
                        bar_center_x = x + width/2;
                        bar_center_y = y - height/2;

                        snprintf(text_buf, sizeof(text_buf), "%d", bar_value);
                        u8g2_DrawStr(u8g2, bar_center_x-u8g2_GetStrWidth(u8g2, text_buf)/2, bar_center_y+u8g2_GetAscent(u8g2)/2, text_buf);
                    }                    
                    break;                        
            }

            u8g2_SetDrawColor(u8g2, 1);     // Normal mode
            u8g2_SetFontMode(u8g2, 0);      // Solid mode
        }


        // Set the string to use for the unit display
        // The font must be a transparent type (name ends in _tx)        
        void set_font(const uint8_t *u8_font)
        {
            value_font = (u8g2_t *) u8_font;
        }
};


class page_controller
{
    private:
        mutex_t log_mutex;

    public:
        u8g2_t u8g2;
        u8log_t u8log;
        mui_t mui;         
        uint8_t page_idx;
        page_draw_fn page_fn[PAGE_COUNT];
        absolute_time_t btn_lockouts[BUTTON_COUNT];
        uint8_t btn_state[BUTTON_COUNT];
        uint8_t btn_gpio[BUTTON_COUNT];
        uint8_t btn_pressed[BUTTON_COUNT];
        uint8_t btn_held[BUTTON_COUNT];
        uint8_t load_mode[EXT_LOAD_COUNT];
        uint8_t load_gpio[EXT_LOAD_COUNT];
        uint8_t u8log_buf[U8LOG_WIDTH*U8LOG_HEIGHT];
        mutex_t float_mutex;
        mutex_t flash_mutex;
        nv_flash_storage nv_settings;
        log_data_t esc_data;
        moving_avg<float> v_in_smoothed;
        moving_avg<float> speed_smoothed;
        float wh_used;
        float session_distance;
        uint8_t skip_frames;
        uint8_t new_page;
        uint8_t btn_retrigger;


        page_controller(void);
        void update(void);
        void draw_page(void);
        void draw_string(uint16_t x_coord, uint16_t y_coord, const char* format, ...);
        void draw_string_lines(uint16_t x_coord, uint16_t y_coord, const char* full_string);        
        void draw_SD_icon(uint16_t x_coord, uint16_t y_coord);
        void draw_ESC_icon(uint16_t x_coord, uint16_t y_coord);
        void draw_overlay_status(void);
        void draw_speed_bar(uint8_t x, uint8_t y, float speed);
        void update_load_outputs(void);
        void log_write_string(char *log_str);

        // page functions
        void page_main_draw(void);
        void page_log_draw(void);
        void page_cfg_draw(void);
        void page_details_draw(void);
};


void format_with_si(float value, char *out_buf, size_t buf_len, const char *unit);
float calc_speed_kph(float erpm);

// Config update functions
void speed_max_store(void);
void backlight_bright_store(void);
void load_A_store(void);
void load_B_store(void);
void backlight_mode_store(void);
void datetime_store(void);
void day_start_store(void);
void night_start_store(void);
void cfg_reset(void);

#endif