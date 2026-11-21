#ifndef FWC_MAZE_H
#define FWC_MAZE_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

/* Directions: absolute headings, clockwise. */
typedef enum { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3, DIR_NONE = 4 } dir_t;

/* Which cells the flood starts from. */
typedef enum { TARGET_GOAL, TARGET_START } target_t;

/* Search: unknown walls are assumed open (optimistic). Run: only walls known to be open. */
typedef enum { FLOOD_SEARCH, FLOOD_KNOWN } flood_mode_t;

#define FLOOD_UNREACHABLE 0xFFFFu

static inline dir_t dir_right(dir_t d) {
    return (dir_t)((d + 1) & 3);
}
static inline dir_t dir_left(dir_t d) {
    return (dir_t)((d + 3) & 3);
}
static inline dir_t dir_behind(dir_t d) {
    return (dir_t)((d + 2) & 3);
}

/* Map version: changes whenever a known wall changes. Planners cache against it. */
uint16_t maze_version(void);
bool maze_in_bounds(int x, int y);
/* No known wall on side d of (x, y); FLOOD_KNOWN additionally requires the wall to be
 * known to be absent. */
bool maze_passable(uint8_t x, uint8_t y, dir_t d, flood_mode_t mode);

void maze_init(void);
void maze_set_wall(uint8_t x, uint8_t y, dir_t d, bool present); /* both sides, marks known */
bool maze_wall(uint8_t x, uint8_t y, dir_t d);                   /* known present */
bool maze_known(uint8_t x, uint8_t y, dir_t d);
bool maze_is_target(uint8_t x, uint8_t y, target_t t);
void maze_neighbour(uint8_t x, uint8_t y, dir_t d, uint8_t* nx, uint8_t* ny);

/* Breadth-first flood from the target cells; returns the distance of (x, y) in cells. */
uint16_t maze_flood(target_t t, flood_mode_t mode, uint8_t x, uint8_t y);
uint16_t maze_distance(uint8_t x, uint8_t y);
/* Best direction out of (x, y) after maze_flood(); prefers ahead, then left/right, then back. */
dir_t maze_best_dir(uint8_t x, uint8_t y, dir_t heading, flood_mode_t mode);
/* Writes the flood distances into the simulator overlay (no-op on hardware). */
void maze_annotate_distances(void);

#endif
