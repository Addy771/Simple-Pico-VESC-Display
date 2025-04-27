#include <u8g2.h>
#include "pico/stdlib.h"
#include "hw_def.h"

#ifndef PAGE_CTRL_H
#define PAGE_CTRL_H

#define PAGE_COUNT 1
#define BUTTON_COUNT 3  // left, right buttons and center as confirm

#define BUTTON_LOCK_TIME_MS 50  // After a button is pressed, it won't be checked again until this much time has passed. 

typedef void (*page_draw_fn)();

typedef enum
{
    PB_LEFT,
    PB_RIGHT,
    PB_CONFIRM
} btn_map;

class page_controller
{
    public:
        u8g2_t u8g2;
        u8log_t u8log;
        uint8_t page_idx;
        page_draw_fn page_fn[PAGE_COUNT];
        absolute_time_t btn_lockouts[BUTTON_COUNT];
        uint8_t btn_state[BUTTON_COUNT];
        uint8_t btn_gpio[BUTTON_COUNT];
        uint8_t btn_pressed[BUTTON_COUNT];

        page_controller(void);
        void update(void);
        void draw_page(void);

        // page functions
        void page_main_draw(void);
};


#endif