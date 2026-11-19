/* Fixture: a firmware file that respects the HAL boundary. */
#include <math.h>
#include <stdint.h>

#include "mmhal/hal.h"
#include "util.h"

int fixture_good(void) { return util_answer() + (int)sqrt(4.0); }
