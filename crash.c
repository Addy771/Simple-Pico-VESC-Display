/*
crash.c
store information preserved between resets to debug crashes
*/

#include "crash.h"
#include "hardware/watchdog.h"

// Store crash data in the uninitialized section so it isn't cleared on reset
volatile crash_info_t crash_info __attribute__((section(".uninitialized_data")));
volatile crash_info_t previous_crash;


