

#include "user_cfg.h"

// Configuration values
uint8_t speedo_max;
uint8_t backlight_brightness;
uint8_t backlight_mode;
uint8_t current_datetime;
uint8_t load_A_mode, load_B_mode;
uint8_t esc_comm_mode;
uint8_t esc_can_override;
uint8_t disp_can_override;

// Configuration options
user_config_setting config_table[] =
{
    {
        "Speedometer Scale",
        CFG_LIST,
        &speedo_max,
        0,
        0,
        (const char *[]){"30", "60", "120", "180"},
        4
    },

    {
        "Backlight Brightness",
        CFG_NUMBER,
        &backlight_brightness,
        1,
        10
    },

    {
        "Backlight Mode",
        CFG_LIST,
        &backlight_mode,
        0,
        0,
        (const char *[]){"Off", "On", "Nighttime"},
        3
    },

    {
        "Date / Time",
        CFG_DATETIME,
        &current_datetime,
        0,
        0
    },

    {
        "Load Channel A Mode",
        CFG_LIST,
        &load_A_mode,
        0,
        0,
        (const char *[]){"Disabled", "Always On", "Always On - Blinking", "Brake Activated", "Nighttime", "Daytime"},
        6
    },

    {
        "Load Channel B Mode",
        CFG_LIST,
        &load_B_mode,
        0,
        0,
        (const char *[]){"Disabled", "Always On", "Always On - Blinking", "Brake Activated", "Nighttime", "Daytime"},
        6
    },
    
    {
        "ESC Communication Mode",
        CFG_LIST,
        &esc_comm_mode,
        0,
        0,
        (const char*[]){"CAN", "UART"},
        2
    },

    {
        "ESC CAN ID override",
        CFG_NUMBER,
        &esc_can_override,
        0,
        255
    },

    {
        "Display CAN ID override",
        CFG_NUMBER,
        &disp_can_override,
        0,
        255
    }


};

const uint8_t config_table_size = sizeof(config_table) / sizeof(user_config_setting);