#ifndef DEMO_MOUSE_H
#define DEMO_MOUSE_H

#include <stdbool.h>
#include <stdint.h>

#include "maze.h"

/* Where the mouse believes it is. */
typedef struct {
    uint8_t x, y;
    dir_t heading;
} mouse_state_t;

extern mouse_state_t g_mouse;

void mouse_init(void); /* start cell (0, 0) facing north, stopped at the centre */

/* Explores towards the target with flood fill, mapping walls; ends stopped at the centre of
 * the first target cell reached. Returns false if the target is unreachable. */
bool mouse_search(target_t target);
/* Speed run from the start to the goal over walls known to be open. */
bool mouse_speed_run(void);
/* Turns in place to face `d` (from a stop). */
void mouse_face(dir_t d);
void mouse_blink(int times);

#endif
