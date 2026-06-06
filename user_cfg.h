#ifndef USER_CFG_H
#define USER_CFG_H

#include <stdint.h>

// Indicates what type of editor is needed to modify the config value
typedef enum
{
    CFG_NUMBER,
    CFG_LIST,
    CFG_DATETIME
} user_config_type;


// States for config page state machine
typedef enum
{
    CFG_SCREEN_MAIN,
    CFG_SCREEN_EDIT_NUMBER,
    CFG_SCREEN_EDIT_LIST,
    CFG_SCREEN_EDIT_DATETIME
} config_screen;


// Define which index of the config table is which parameter
typedef enum
{
    CFG_SPEED_SCALE = 0,
    CFG_BACKLIGHT_BRIGHTNESS = 1
} config_option_idx;


// Single configuration option
typedef struct
{
    const char *display_name;
    const user_config_type cfg_type;
    void *value;
    uint8_t min_val;
    uint8_t max_val;
    const char **list_items;
    uint8_t list_count;

} user_config_setting;


extern user_config_setting config_table[];

extern const uint8_t config_table_size;

#endif