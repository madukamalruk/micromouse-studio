/*
 * demo_tremaux - Tremaux's algorithm, pure C11 on mmhal only.
 *
 * A graph search that needs no distance map, only chalk marks on the passages:
 *   - mark the passage you leave by, and mark the passage you arrive by (marks sit at each
 *     end of a passage, at most two);
 *   - arriving at a cell you have already been in by a passage that is new to it: turn back;
 *   - otherwise leave by the passage with the fewest marks (never a twice-marked one unless
 *     it is the only way), preferring straight ahead, then left, then right, then back.
 * It copes with loops and always finds the goal, but not by the shortest route. Once the goal
 * is found the mouse knows a route; the way back and the speed run use the shortest path over
 * the known cells (BFS), with the cell-distance numbers shown.
 *
 * While exploring, the number in a cell is how many times the mouse has been in it; dead ends
 * are marked red.
 */
#include "common/plan.h"
#include "common/solver.h"
#include "mmhal/hal.h"

static uint8_t g_marks[MAZE_SIZE][MAZE_SIZE][4]; /* marks at this cell's end of each passage */
static uint8_t g_visits[MAZE_SIZE][MAZE_SIZE];
static bool g_have_last;
static uint8_t g_last_x, g_last_y;
static unsigned g_total_marks, g_backtracks;

static void init(void) {
    for (int x = 0; x < MAZE_SIZE; ++x)
        for (int y = 0; y < MAZE_SIZE; ++y) {
            g_visits[x][y] = 0;
            for (int d = 0; d < 4; ++d) g_marks[x][y][d] = 0;
        }
    g_have_last = false;
    g_total_marks = 0;
    g_backtracks = 0;
}

static void put_visits(uint8_t x, uint8_t y) {
    char buf[4];
    unsigned v = g_visits[x][y];
    if (v > 99) v = 99;
    buf[0] = v >= 10 ? (char)('0' + v / 10u) : (char)('0' + v);
    buf[1] = v >= 10 ? (char)('0' + v % 10u) : '\0';
    buf[2] = '\0';
    hal_annotate_cell_text(x, y, buf);
}

static unsigned open_ends(uint8_t x, uint8_t y) {
    unsigned n = 0;
    for (int d = 0; d < 4; ++d)
        if (maze_passable(x, y, (dir_t)d, FLOOD_SEARCH)) ++n;
    return n;
}

static dir_t explore(uint8_t x, uint8_t y, dir_t heading) {
    const dir_t order[4] = {heading, dir_left(heading), dir_right(heading), dir_behind(heading)};
    const dir_t e_in = dir_behind(heading); /* the passage we came in by */
    bool turn_back = false;

    if (g_have_last && (x != g_last_x || y != g_last_y)) {
        unsigned before = 0;
        for (int d = 0; d < 4; ++d) before += g_marks[x][y][d];
        const bool arrival_was_new = g_marks[x][y][e_in] == 0;
        ++g_marks[x][y][e_in]; /* arrival mark */
        ++g_total_marks;
        /* Old cell, new passage: treat it as a dead end. */
        turn_back = before > 0 && arrival_was_new;
    }
    if (g_visits[x][y] < 255) ++g_visits[x][y];
    put_visits(x, y);
    if (open_ends(x, y) == 1 && (x != 0 || y != 0))
        hal_annotate_cell_color(x, y, 'R'); /* dead end */

    dir_t best = DIR_NONE;
    if (turn_back && maze_passable(x, y, e_in, FLOOD_SEARCH)) {
        best = e_in;
        ++g_backtracks;
    } else {
        unsigned best_marks = 3;
        for (int i = 0; i < 4; ++i) { /* strict <: ties keep ahead > left > right > back */
            const dir_t d = order[i];
            if (!maze_passable(x, y, d, FLOOD_SEARCH)) continue;
            if (g_marks[x][y][d] < best_marks) {
                best_marks = g_marks[x][y][d];
                best = d;
            }
        }
        if (best == e_in) ++g_backtracks;
    }
    if (best != DIR_NONE) {
        ++g_marks[x][y][best]; /* leaving mark */
        ++g_total_marks;
    }
    g_have_last = true;
    g_last_x = x;
    g_last_y = y;
    hal_telemetry("marks", (float)g_total_marks);
    hal_telemetry("backtracks", (float)g_backtracks);
    return best;
}

static dir_t next(target_t target, flood_mode_t mode, uint8_t x, uint8_t y, dir_t heading) {
    /* Tremaux only to find the goal; afterwards the known map gives the shortest way. */
    if (target == TARGET_GOAL && mode == FLOOD_SEARCH) return explore(x, y, heading);
    return plan_flood_next(target, FLOOD_KNOWN, x, y, heading, true);
}

static const solver_t g_solver = {
    "demo_tremaux",
    "Tremaux: chalk marks on passages (numbers = visits, red = dead end), then shortest way back",
    init,
    next,
};

void fw_main(void) {
    solver_main(&g_solver);
}
