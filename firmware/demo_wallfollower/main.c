/*
 * demo_wallfollower - left-hand wall follower, pure C11 on mmhal only.
 *
 * The classic educational baseline: keep the left hand on the wall. Turn left if you can,
 * else go straight, else turn right, else turn round. No map is consulted and nothing is
 * remembered except where the mouse has been (to notice a loop).
 *
 * It only works when the goal touches the outer wall. If the goal is a free-standing island
 * (as in many contest mazes) the mouse circles the same walls for ever; the firmware notices
 * the repeated state (cell + heading) and stops with an ERROR instead of driving in circles.
 * After the goal the way back and the speed run use the shortest path over the cells it saw,
 * which is usually far from the shortest in the maze: compare it with the other solvers.
 *
 * While following the wall, the number in a cell is the step on which it was first entered;
 * cells entered again are orange.
 */
#include "common/plan.h"
#include "common/solver.h"
#include "mmhal/hal.h"

static uint8_t g_seen[MAZE_SIZE][MAZE_SIZE];   /* bit h: been here heading h */
static uint16_t g_first[MAZE_SIZE][MAZE_SIZE]; /* step of first entry, 0 = never */
static unsigned g_steps, g_revisits;

static void init(void) {
    for (int x = 0; x < MAZE_SIZE; ++x)
        for (int y = 0; y < MAZE_SIZE; ++y) {
            g_seen[x][y] = 0;
            g_first[x][y] = 0;
        }
    g_steps = 0;
    g_revisits = 0;
}

static void put_number(uint8_t x, uint8_t y, unsigned v) {
    char tmp[6], buf[8];
    unsigned n = 0;
    do {
        tmp[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v > 0 && n < sizeof tmp);
    for (unsigned i = 0; i < n; ++i) buf[i] = tmp[n - 1 - i];
    buf[n] = '\0';
    hal_annotate_cell_text(x, y, buf);
}

static dir_t follow(uint8_t x, uint8_t y, dir_t heading) {
    const uint8_t bit = (uint8_t)(1u << heading);
    if (g_seen[x][y] & bit) {
        hal_log("ERROR left-hand rule is looping: the goal is not attached to the outer wall");
        return DIR_NONE;
    }
    g_seen[x][y] |= bit;
    ++g_steps;
    if (g_first[x][y] == 0) {
        g_first[x][y] = (uint16_t)g_steps;
        put_number(x, y, g_steps);
    } else {
        ++g_revisits;
        hal_annotate_cell_color(x, y, 'O');
    }
    hal_telemetry("revisits", (float)g_revisits);
    const dir_t order[4] = {dir_left(heading), heading, dir_right(heading), dir_behind(heading)};
    for (int i = 0; i < 4; ++i)
        if (maze_passable(x, y, order[i], FLOOD_SEARCH)) return order[i];
    return DIR_NONE;
}

static dir_t next(target_t target, flood_mode_t mode, uint8_t x, uint8_t y, dir_t heading) {
    if (target == TARGET_GOAL && mode == FLOOD_SEARCH) return follow(x, y, heading);
    return plan_flood_next(target, FLOOD_KNOWN, x, y, heading, true);
}

static const solver_t g_solver = {
    "demo_wallfollower",
    "Left-Hand Wall Follower: turn left if you can (numbers = step, orange = revisited)",
    init,
    next,
};

void fw_main(void) {
    solver_main(&g_solver);
}
