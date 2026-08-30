/*
crash.h
store information preserved between resets to debug crashes
*/

#ifndef CRASH_H
#define CRASH_H

#include "pico/stdlib.h"

typedef enum
{
    C0_ENTRY,
    C0_INIT_DONE,
    C0_WAIT_FOR_FRAME,
    C0_PAGE_UPDATE,
    C0_PAGE_DRAW
} core0_status;


// Translate state number into a string label
static inline const char* core0_state_str(core0_status state)
{
    switch(state)
    {
        case C0_ENTRY:              return "C0_ENTRY";
        case C0_INIT_DONE:          return "C0_INIT_DONE";
        case C0_WAIT_FOR_FRAME:     return "C0_WAIT_FOR_FRAME";
        case C0_PAGE_UPDATE:        return "C0_PAGE_UPDATE";
        case C0_PAGE_DRAW:          return "C0_PAGE_DRAW";
        default:                    return "Unknown";
    }
}


typedef enum
{
    C1_OFF,
    C1_ENTRY,
    C1_INIT_DONE,
    C1_PROCESS_CAN_MSG,
    C1_TX_CAN_MSG,
    C1_TX_CAN_START,
    C1_TX_CAN_SENT,
    C1_PROCESS_UART_MSG,
    C1_TX_UART_MSG,
    C1_LOG_PREP_DATA,
    C1_LOG_SAVE_DATA
} core1_status;


// Translate state number into a string label
static inline const char* core1_state_str(core1_status state)
{
    switch(state)
    {
        case C1_OFF:                return "C1_OFF";
        case C1_ENTRY:              return "C1_ENTRY";
        case C1_INIT_DONE:          return "C1_INIT_DONE";
        case C1_PROCESS_CAN_MSG:    return "C1_PROCESS_CAN_MSG";
        case C1_TX_CAN_MSG:         return "C1_TX_CAN_MSG";
        case C1_TX_CAN_START:       return "C1_TX_CAN_START";
        case C1_TX_CAN_SENT:        return "C1_TX_CAN_SENT";
        case C1_PROCESS_UART_MSG:   return "C1_PROCESS_UART_MSG";
        case C1_TX_UART_MSG:        return "C1_TX_UART_MSG";
        case C1_LOG_PREP_DATA:      return "C1_LOG_PREP_DATA";
        case C1_LOG_SAVE_DATA:      return "C1_LOG_SAVE_DATA";
        default:                    return "Unknown";
    }
}


typedef struct
{
    uint32_t magic;
    uint8_t reason;
    uint8_t watchdog_trip;
    core0_status core0_state;
    core1_status core1_state;
} crash_info_t;

extern volatile crash_info_t crash_info;
extern volatile crash_info_t previous_crash;


#endif