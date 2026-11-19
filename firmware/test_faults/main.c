/* test_faults: misbehaviours the host must detect, selected by button 0/1 at boot.
 *   button 1 pressed: SysTick ISR calls a blocking function   -> host fault
 *   button 0 pressed: main loop spins without any HAL call    -> watchdog
 *   neither:          fw_main returns                         -> host "halted" */
#include "mmhal/hal.h"

static void bad_isr(void) {
    hal_delay_ms(1);
}

void fw_main(void) {
    if (hal_button_pressed(1)) {
        hal_set_systick_handler(bad_isr);
        for (;;) hal_delay_ms(10);
    }
    if (hal_button_pressed(0)) {
        static volatile uint32_t g_spin =
            0; /* static volatile: the loop cannot be optimised away */
        for (;;) g_spin = g_spin + 1u;
    }
    hal_log("returning from fw_main");
}
