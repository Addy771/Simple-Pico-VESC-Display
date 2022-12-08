#ifndef UTIL_ENUM_H_
#define UTIL_ENUM_H_


typedef enum 
{
	TEMP_FET_FILTERED               = 0,
    TEMP_MOTOR_FILTERED             = 1,
    READ_RESET_AVG_MOTOR_CURRENT    = 2,
    READ_RESET_AVG_INPUT_CURRENT    = 3,
    READ_RESET_AVG_ID               = 4,
    READ_RESET_AVG_IQ               = 5,
    GET_DUTY_CYCLE_NOW              = 6,
    GET_RPM                         = 7,
    GET_INPUT_VOLTAGE_FILTERED      = 8,
    GET_AMP_HOURS                   = 9,
    GET_AMP_HOURS_CHARGED           = 10,
    GET_WATT_HOURS                  = 11,
    GET_WATT_HOURS_CHARGED          = 12,
    GET_TACHOMETER_VALUE            = 13,
    GET_TACHOMETER_ABS_VALUE        = 14,
    GET_FAULT                       = 15,
    GET_PID_POS_NOW                 = 16,
    CONTROLLER_ID                   = 17,
    TEMP_MOS_123                    = 18,
    READ_RESET_AVG_VQ               = 19,
    READ_RESET_AVG_VD               = 20,
    STATUS                          = 21
} comm_value_mask;


typedef enum
{
    type_int8 = 0,
    type_int16,
    type_int32,
    type_float16,
    type_float32
} comm_value_type;


const comm_value_type COMM_VALUE_TYPES[] = 
{
    type_float16,   // 0
    type_float16,   // 1
    type_float32,   // 2
    type_float32,   // 3
    type_float32,   // 4
    type_float32,   // 5
    type_float16,   // 6
    type_float32,   // 7
    type_float16,   // 8
    type_float32,   // 9 
    type_float32,   // 10
    type_float32,   // 11
    type_float32,   // 12
    type_int32,     // 13
    type_int32,     // 14
    type_int8,      // 15
    type_float32,   // 16
    type_int8,      // 17
    type_float16,   // 18
    type_float32,   // 19
    type_float32,   // 20
    type_int8,      // 21


};

#endif /* UTIL_ENUM_H_ */