#ifndef FWC_MOUSE_H
#define FWC_MOUSE_H

#include <stdbool.h>
#include <stdint.h>

#include "maze.h"
#include "solver.h"

/* Where the mouse believes it is. */
typedef struct {
    uint8_t x, y;
    dir_t heading;
} mouse_state_t;

extern mouse_state_t g_mouse;

void mouse_init(void); /* start cell (0, 0) facing north, stopped at the centre */

/* Explores towards the target one decision per cell, asking the solver where to go next and
 * mapping walls on the way; ends stopped at the centre of the first target cell reached.
 * Returns false if the solver finds no way (DIR_NONE). */
bool mouse_search(const solver_t* solver, target_t target);
/* Speed run from the start to the goal over walls known to be open, along the solver's own
 * idea of the best path (FLOOD_KNOWN). */
bool mouse_speed_run(const solver_t* solver);
/* Turns in place to face `d` (from a stop). */
void mouse_face(dir_t d);
void mouse_blink(int times);

#endif
