
#ifndef LOG_H
#define LOG_H

#include "datatypes.h"
#include "ff.h"


#define MIN_FREE_MB 100     // If the SD has less free space than this, the user should be warned
#define LOG_PREFIX "log_"   // Numbered log filename prefix
#define MAX_ESCS 4          // Maximum number of ESCs to request data from (CAN mode only)

// #define DBG_PRINT printf
#define DBG_PRINT print_and_log

void print_and_log(const char* format, ...);

typedef enum 
{
    SD_NOT_PRESENT,
    SD_PRESENT,
    SD_ERROR
} sd_state;

typedef struct
{
    // calculated values
    float p_in;
    float speed_kph;
    float odometer;

    // values from vesc
    int ms_today;       // Time of day in ms
    float adc1_decoded;
    float adc2_decoded;

    float v_in;
    float temp_mos;
    float temp_mos_1;
    float temp_mos_2;
    float temp_mos_3;
    float temp_motor;
    float current_motor;
    float current_in;
    float id;
    float iq;
    float duty_now;
    float rpm;
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    float battery_level;
    int tachometer;
    int tachometer_abs;
    float position;
    int vesc_id;
    float vd;
    float vq;
    mc_fault_code fault_code;
} log_data_t;


// Log data fields are defined with an X macro to generate the necessary structures
#define COMMON_LOG_FIELDS(X)    \
    X(float, p_in)    \
    X(float, speed_kph)    \
    X(float, odometer)    


#define ESC_LOG_FIELDS(X)    \
    X(int32_t, ms_today)    \
    X(float, v_in)  \
    X(float, temp_mos)    \
    X(float, temp_mos_1)    \
    X(float, temp_mos_2)    \
    X(float, temp_mos_3)    \
    X(float, temp_motor)    \
    X(float, current_motor)    \
    X(float, current_in)    \
    X(float, id)    \
    X(float, iq)    \
    X(float, duty_now )    \
    X(float, rpm)    \
    X(float, amp_hours_spent)    \
    X(float, amp_hours_charged)    \
    X(float, watt_hours_spent)    \
    X(float, watt_hours_charged)    \
    X(float, battery_level)    \
    X(int32_t, tachometer)    \
    X(int32_t, tachometer_abs)    \
    X(float, position )    \
    X(int32_t, vesc_id)    \
    X(float, vd)    \
    X(float, vq)    \
    X(mc_fault_code, fault_code)


#define DECLARE_FIELD(type, name) type name;


typedef struct
{
    COMMON_LOG_FIELDS(DECLARE_FIELD)
} common_log_data_t;


typedef struct 
{
    ESC_LOG_FIELDS(DECLARE_FIELD)
} esc_log_data_t;


typedef struct
{
    common_log_data_t common;
    esc_log_data_t esc[MAX_ESCS];
} log_data_combined_t;

#undef DECLARE_FIELD

extern volatile log_data_combined_t log_data;
extern uint8_t sd_status;

FRESULT init_filesystem();
FRESULT create_log_file();
FRESULT append_data_pt(log_data_t *data);
void print_and_log(const char *format, ...);
void generate_csv_head(FIL *log_file, uint8_t esc_count);
void write_data_row(FIL *log_file, uint8_t esc_count);
#endif