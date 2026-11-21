/*
 * demo_floodfill - classic micromouse demo in pure C11 on mmhal only.
 *
 *   calibrate gyro -> search to the centre (flood fill, mapping walls) -> blink
 *   -> search back to the start -> speed run over the known shortest path -> blink.
 *
 * Algorithm: modified flood fill. A breadth-first flood from the target gives every cell its
 * distance in cells (unknown walls are assumed open while exploring); the mouse always steps
 * downhill, preferring straight ahead. The numbers in the cells are that distance map.
 *
 * Motion, sensing and the mission live in firmware/common (shared by all demo_* solvers):
 * 1 kHz control loop, trapezoidal profiles, wall-following steering, in-place turns, 6 V
 * motor cap with a 20 ms reversal coast.
 */
#include "common/plan.h"
#include "common/solver.h"
#include "mmhal/hal.h"

static dir_t next(target_t target, flood_mode_t mode, uint8_t x, uint8_t y, dir_t heading) {
    return plan_flood_next(target, mode, x, y, heading, true);
}

static const solver_t g_solver = {
    "demo_floodfill",
    "Flood-Fill: BFS cell count to the target, always step downhill (numbers = cells to go)",
    0,
    next,
};

void fw_main(void) {
    solver_main(&g_solver);
}
