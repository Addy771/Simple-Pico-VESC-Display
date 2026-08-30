/*
crash.c
store information preserved between resets to debug crashes
*/

#include "crash.h"
#include "hardware/watchdog.h"
#include "pico/platform.h"

// Store crash data in the uninitialized section so it isn't cleared on reset
volatile crash_info_t crash_info __attribute__((section(".uninitialized_data")));
volatile crash_info_t previous_crash;


void nmi_handler(void)
{
    exception_handler(NMI_EXCEPTION);
}

void hardfault_handler(void)
{
    exception_handler(HARDFAULT_EXCEPTION);
}

void exception_handler(int8_t exception)
{
    crash_info.reason = exception;
    crash_info.core = get_core_num();
    crash_info.fault_caused_reset = 1;

    watchdog_reboot(0, 0, 1);   // reboot in 1ms

    while(1);
}