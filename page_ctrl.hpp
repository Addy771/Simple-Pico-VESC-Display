#include <u8g2.h>
#include "pico/stdlib.h"
#include "hw_def.h"
#include "pico/sync.h"
#include "nv_flash.hpp"
#include "log.hpp"
#include <cstdarg>

#ifndef PAGE_CTRL_H
#define PAGE_CTRL_H

#define PAGE_COUNT 2
#define BUTTON_COUNT 3  // left, right buttons and center as confirm
#define EXT_LOAD_COUNT 2    // channel A, B

#define BUTTON_LOCK_TIME_MS 50  // After a button is pressed, it won't be checked again until this much time has passed. 

#define U8LOG_WIDTH 42
#define U8LOG_HEIGHT 11
#define STATUS_HEIGHT 11

typedef void (*page_draw_fn)();

typedef enum
{
    PB_LEFT,
    PB_RIGHT,
    PB_CONFIRM
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


class page_controller
{
    private:
        mutex_t log_mutex;

    public:
        u8g2_t u8g2;
        u8log_t u8log;
        uint8_t page_idx;
        page_draw_fn page_fn[PAGE_COUNT];
        absolute_time_t btn_lockouts[BUTTON_COUNT];
        uint8_t btn_state[BUTTON_COUNT];
        uint8_t btn_gpio[BUTTON_COUNT];
        uint8_t btn_pressed[BUTTON_COUNT];
        uint8_t load_mode[EXT_LOAD_COUNT];
        uint8_t load_gpio[EXT_LOAD_COUNT];
        uint8_t u8log_buf[U8LOG_WIDTH*U8LOG_HEIGHT];
        mutex_t float_mutex;
        mutex_t flash_mutex;
        nv_flash_storage nv_settings;
        log_data_t esc_data;
        uint8_t motor_poles;
        float gear_ratio;
        float wheel_diameter;
        uint8_t config_received;
        moving_avg<float> v_in_smoothed;
        moving_avg<float> speed_smoothed;


        page_controller(void);
        void update(void);
        void draw_page(void);
        void draw_string(uint16_t x_coord, uint16_t y_coord, const char* format, ...);
        void draw_overlay_status(void);
        void draw_speed_bar(uint8_t x, uint8_t y, float speed);
        void update_load_outputs(void);
        void log_write_string(char *log_str);

        // page functions
        void page_main_draw(void);
        void page_log_draw(void);
};

#endif