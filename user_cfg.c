

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
        "Maximum speed shown on the speed bar graph.",
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
        "LCD screen backlight behavior.",
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
        CFG_DATE,
        &current_datetime,
        0,
        0
    },

    {
        "Time",
        "Time of day, used for log files and time based behavior.",
        CFG_TIME,
        &current_datetime,
        0,
        0
    },    

    {
        "Load Channel A Mode",
        "Load channel output behavior. Time-based modes require RTC time to be set, otherwise they will keep the output on permanently.",
        CFG_LIST,
        &load_A_mode,
        0,
        0,
        (const char *[]){"Disabled", "Always On", "Always On - Blinking", "Brake Activated", "Nighttime", "Daytime"},
        6
    },

    {
        "Load Channel B Mode",
        "Load channel output behavior. Time-based modes require RTC time to be set, otherwise they will keep the output on permanently.",
        CFG_LIST,
        &load_B_mode,
        0,
        0,
        (const char *[]){"Disabled", "Always On", "Always On - Blinking", "Brake Activated", "Nighttime", "Daytime"},
        6
    },
    
    {
        "ESC Communication Mode",
        "Interface that the display will use to communicate with the ESC.",
        CFG_LIST,
        &esc_comm_mode,
        0,
        0,
        (const char*[]){"CAN", "UART"},
        2
    },

    {
        "ESC CAN ID override",
        "Range: 0x00 to 0xFF. If the display connected to the wrong ESC on the CAN bus, it can be manually set here.",
        CFG_NUMBER,
        &esc_can_override,
        0,
        255,
        0,
        1   // use list count field to indicate that the value should display as hex
    },

    {
        "Display CAN ID override",
        "If the display's CAN ID is conflicting with another device, it can be manually set here",
        CFG_NUMBER,
        &disp_can_override,
        0,
        255,
        0,
        1   // use list count field to indicate that the value should display as hex
    }


};

const uint8_t config_table_size = sizeof(config_table) / sizeof(user_config_setting);