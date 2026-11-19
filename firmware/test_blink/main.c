/* test_blink: LED 0 toggles every 500 ms; SysTick ISR counts ticks.
 * Host tests: exactly 20 toggles in [0 s, 10 s); ISR runs 1000x per second. */
#include "mmhal/hal.h"

static volatile uint32_t g_isr_ticks = 0;

static void systick(void) {
    g_isr_ticks++;
    if (g_isr_ticks == 1000u) hal_log("isr ticks=1000 at t=%lu ms", (unsigned long)hal_millis());
}

void fw_main(void) {
    bool on = false;
    hal_set_systick_handler(systick);
    for (;;) {
        on = !on;
        hal_led_set(0, on);
        hal_delay_ms(500);
    }
}
