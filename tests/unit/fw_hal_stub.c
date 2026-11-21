/* HAL stand-in for the planner unit tests: firmware/common/maze.c and plan.c are compiled on the
 * host without a simulator, so the few annotation calls they make go nowhere. */
#include <stdbool.h>
#include <stdint.h>

#include "mmhal/hal.h"

void hal_annotate_cell_color(uint8_t x, uint8_t y, char mms_color) {
    (void)x;
    (void)y;
    (void)mms_color;
}
void hal_annotate_cell_text(uint8_t x, uint8_t y, const char* text) {
    (void)x;
    (void)y;
    (void)text;
}
void hal_annotate_wall(uint8_t x, uint8_t y, char dir, bool present) {
    (void)x;
    (void)y;
    (void)dir;
    (void)present;
}
