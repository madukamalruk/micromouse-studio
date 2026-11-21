#ifndef FWC_PLAN_H
#define FWC_PLAN_H

#include <stdbool.h>
#include <stdint.h>

#include "maze.h"

/* Path planning shared by the demo solvers. Pure C11, static memory only (about 25 KB of RAM).
 *
 * Cost model for the turn-weighted planners (Dijkstra, A*): the mouse stops to turn, so a
 * 90 degree turn costs 5 and a cell costs 2, i.e. a turn is worth 2.5 cells; a U-turn is two
 * turns. The graph is over states (cell, heading). */
#define COST_CELL 2u
#define COST_TURN 5u
#define COST_INF 0xFFFFu
#define PLAN_MAX_STEPS 512u

unsigned plan_turn_cost(dir_t from, dir_t to); /* 0, 5 or 10 */

/* ---- Cell-count flood (BFS) ---------------------------------------------------------------
 * Next direction downhill from (x, y); DIR_NONE when the target is unreachable. Cached
 * against the map version. annotate: show the distances as cell text. */
dir_t plan_flood_next(target_t t, flood_mode_t m, uint8_t x, uint8_t y, dir_t heading,
                      bool annotate);

/* ---- Turn-weighted Dijkstra field ----------------------------------------------------------
 * Reverse Dijkstra from the target over (cell, heading) states: cost to get there from every
 * state. Rebuilt only when the map, target or mode changed (returns true when rebuilt). */
bool plan_field_build(target_t t, flood_mode_t m);
uint16_t plan_field_cost(uint8_t x, uint8_t y, dir_t heading); /* COST_INF if unreachable */
uint16_t plan_field_cell_cost(uint8_t x, uint8_t y);           /* best over headings */
dir_t plan_field_next(uint8_t x, uint8_t y, dir_t heading, flood_mode_t m);

/* ---- A* ------------------------------------------------------------------------------------
 * Forward search from one state to the nearest target cell. Heuristic: COST_CELL * Manhattan
 * distance to the target cell block, plus COST_TURN when both axes are still open (at least
 * one turn is unavoidable); admissible, so the path is optimal. */
typedef struct {
    uint16_t len;  /* steps */
    uint16_t cost; /* total cost of the path */
    uint8_t dir[PLAN_MAX_STEPS];
    uint8_t sx, sy, sh; /* state the path starts from */
} plan_path_t;

bool plan_astar(target_t t, flood_mode_t m, uint8_t x, uint8_t y, dir_t heading, plan_path_t* out);

#endif /* FWC_PLAN_H */
