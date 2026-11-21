#ifndef FWC_SOLVER_H
#define FWC_SOLVER_H

#include <stdint.h>

#include "maze.h"

/* A micromouse solver = a decision rule on top of the shared motion stack. Everything else
 * (calibration, motion, wall sensing, the search -> return -> speed-run mission, annotations
 * of walls and visited cells) is common. A solver supplies:
 *
 *   init   once, after maze_init()
 *   next   absolute direction to leave cell (x, y) in, standing there facing `heading`
 *          (the cell was just entered; its walls are in the map). DIR_NONE = no way.
 *          FLOOD_SEARCH: unknown walls are assumed open (the mouse is still exploring).
 *          FLOOD_KNOWN:  only walls known to be open (speed run); must be a pure function of
 *                        its arguments and the map, it is also used to look ahead.
 *          The search loop calls it exactly once per cell entered, so FLOOD_SEARCH decisions
 *          may keep state (Tremaux marks) and publish their own cell annotations.
 */
typedef struct {
    const char* name;        /* log prefix = firmware name, e.g. "demo_dijkstra" */
    const char* description; /* one line, logged at boot */
    void (*init)(void);
    dir_t (*next)(target_t target, flood_mode_t mode, uint8_t x, uint8_t y, dir_t heading);
} solver_t;

/* The whole mission: calibrate gyro -> search to the goal -> blink -> search back to the
 * start -> speed run -> blink. Never returns. */
void solver_main(const solver_t* solver);

#endif /* FWC_SOLVER_H */
