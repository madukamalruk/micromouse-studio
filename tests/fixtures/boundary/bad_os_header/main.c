/* Fixture: MUST be rejected — OS API inside firmware. */
#include <windows.h>

#include "mmhal/hal.h"

int fixture_bad(void) { return 0; }
