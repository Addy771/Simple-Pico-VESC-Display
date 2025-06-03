
#ifndef LOG_H
#define LOG_H

#include "datatypes.h"
#include "ff.h"


#define MIN_FREE_MB 100     // If the SD has less free space than this, the user should be warned
#define LOG_PREFIX "log_"   // Numbered log filename prefix

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
    int tachometer;
    int tachometer_abs;
    float position;
    int vesc_id;
    float vd;
    float vq;
    mc_fault_code fault_code;
} log_data_t;

extern uint8_t sd_status;

FRESULT init_filesystem();
FRESULT create_log_file();
FRESULT append_data_pt(log_data_t *data);
void print_and_log(const char* format, ...);

#endif