

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
uint8_t cfg_reset_bool = 0;

// Configuration options
user_config_setting config_table[] =
{
    {
        "Speedometer Scale",
        "Maximum speed shown on \nthe speed bar graph.",
        CFG_LIST,
        &speedo_max,
        0,
        0,
        (const char *[]){"30", "60", "120", "180"},
        4
    },

    {
        "Backlight Brightness",
        "Range: 1-10. ",
        CFG_NUMBER,
        &backlight_brightness,
        1,
        10
    },

    {
        "Backlight Mode",
        "LCD screen backlight \nbehavior.",
        CFG_LIST,
        &backlight_mode,
        0,
        0,
        (const char *[]){"Off", "On", "Nighttime"},
        3
    },

    {
        "Date",
        "Current date, used for log files.",
        CFG_DATE_EDIT,
        &current_datetime,
        0,
        0
    },

    {
        "Time",
        "Time of day, used for log files and time based behavior.",
        CFG_TIME_EDIT,
        &current_datetime,
        0,
        0
    },    

    {
        "Load Channel A Mode",
        "Load output behavior. \nTime-based modes require \nRTC time to be set, \notherwise they will keep the \noutput on permanently.",
        CFG_LIST,
        &load_A_mode,
        0,
        0,
        (const char *[]){"Disabled", "Always On", "Always On - Blinking", "Nighttime", "Daytime", "Brake Activated"},
        6
    },

    {
        "Load Channel B Mode",
        "Load output behavior. \nTime-based modes require \nRTC time to be set, \notherwise they will keep the \noutput on permanently.",
        CFG_LIST,
        &load_B_mode,
        0,
        0,
        (const char *[]){"Disabled", "Always On", "Always On - Blinking", "Nighttime", "Daytime", "Brake Activated"},
        6
    },
    
    {
        "ESC Communication Mode",
        "Interface that the display will \nuse to communicate with \nthe ESC.",
        CFG_LIST,
        &esc_comm_mode,
        0,
        0,
        (const char*[]){"CAN", "UART"},
        2
    },

    {
        "ESC CAN ID override",
        "Range: 0x01 to 0xFE. If the display connected to \nthe wrong ESC on the CAN bus, it can be \nmanually set here. Set to 0 to disable.",
        CFG_NUMBER,
        &esc_can_override,
        0,
        254,
        0,
        1   // use list count field to indicate that the value should display as hex
    },

    {
        "Display CAN ID override",
        "If the display's CAN ID is conflicting with another \ndevice, it can be manually set here.",
        CFG_NUMBER,
        &disp_can_override,
        0,
        255,
        0,
        1   // use list count field to indicate that the value should display as hex
    },

    {
        "Daytime Transition",
        "Start of daytime period. Seconds are ignored",
        CFG_TIME_EDIT,
        0,
        0,
        0
    },   

    {
        "Nighttime Transition",
        "Start of nighttime period. Seconds are ignored",
        CFG_TIME_EDIT,
        0,
        0,
        0
    },       

    {
        "Restore default settings",
        "Reset settings and date / time to default values.",
        CFG_BOOL,
        &cfg_reset_bool
    }


};

const uint8_t config_table_size = sizeof(config_table) / sizeof(user_config_setting);

const uint8_t max_speed_list_values[] = {30, 60, 90, 120};