/* test_globals: proves reset reloads firmware with fresh globals.
 * Logs "boot N" with a global boot counter; after a host reset N must be 1 again. */
#include "mmhal/hal.h"

static int g_boots = 0;
static int g_initialised = 7; /* .data, not .bss */

void fw_main(void) {
    g_boots++;
    hal_log("boot %d init %d", g_boots, g_initialised);
    g_initialised = 99;
    for (;;) hal_delay_ms(100);
}
