#ifndef USER_CFG_H
#define USER_CFG_H

#include <stdint.h>

// Indicates what type of editor is needed to modify the config value
typedef enum
{
    CFG_NUMBER,
    CFG_LIST,
    CFG_DATE_EDIT,
    CFG_TIME_EDIT,
    CFG_BOOL
} user_config_type;


// States for config page state machine
typedef enum
{
    CFG_SCREEN_MAIN,
    CFG_SCREEN_EDIT_NUMBER,
    CFG_SCREEN_EDIT_LIST,
    CFG_SCREEN_EDIT_DATE,
    CFG_SCREEN_EDIT_TIME,
    CFG_SCREEN_EDIT_BOOL
} config_screen;


// Define which index of the config table is which parameter
typedef enum
{
    CFG_SPEED_SCALE = 0,
    CFG_BACKLIGHT_BRIGHTNESS = 1,
    CFG_BACKLIGHT_MODE = 2,
    CFG_DATE = 3,
    CFG_TIME = 4,
    CFG_LOAD_A = 5,
    CFG_LOAD_B = 6,
    CFG_ESC_COMM = 7,
    CFG_ESC_CAN_ID = 8,
    CFG_DISP_CAN_ID = 9,
    CFG_DAY_START = 10,
    CFG_NIGHT_START = 11,
    CFG_DISP_RESET_SETTINGS = 12    // must be last item
} config_option_idx;


// Single configuration option
typedef struct
{
    const char *display_name;
    const char *description;
    const user_config_type cfg_type;
    void *value;
    uint8_t min_val;
    uint8_t max_val;
    const char **list_items;
    uint8_t list_count;
    void (*store)(void);

} user_config_setting;


extern user_config_setting config_table[];
extern const uint8_t config_table_size;

extern const uint8_t max_speed_list_values[];
#endif