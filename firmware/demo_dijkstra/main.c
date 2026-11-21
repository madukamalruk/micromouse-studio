/*
 * demo_dijkstra - turn-weighted Dijkstra, pure C11 on mmhal only.
 *
 * Plain flood fill counts cells; this one counts time. The mouse stops to turn, so a 90 degree
 * turn is priced at 2.5 cells (see plan.h: cell 2, turn 5). A reverse Dijkstra from the target
 * over (cell, heading) states gives the cost of getting there from every cell, and the mouse
 * steps to the neighbour with the lowest total. Straighter routes win even when they are
 * longer in cells, which is what makes the speed run faster on twisty mazes.
 *
 * The numbers in the cells are that cost map (2 per cell + 5 per 90 degree turn).
 */
#include "common/plan.h"
#include "common/solver.h"
#include "mmhal/hal.h"

static unsigned g_rebuilds;
static uint16_t g_shown[MAZE_SIZE][MAZE_SIZE];

static void init(void) {
    for (int x = 0; x < MAZE_SIZE; ++x)
        for (int y = 0; y < MAZE_SIZE; ++y) g_shown[x][y] = 0xFFFEu;
}

static void annotate_field(void) {
    char buf[8];
    for (int x = 0; x < MAZE_SIZE; ++x)
        for (int y = 0; y < MAZE_SIZE; ++y) {
            const uint16_t c = plan_field_cell_cost((uint8_t)x, (uint8_t)y);
            if (c == g_shown[x][y]) continue;
            g_shown[x][y] = c;
            if (c == COST_INF) {
                buf[0] = '\0';
            } else {
                /* tiny itoa: costs stay below 10000 */
                unsigned v = c, n = 0;
                char tmp[6];
                do {
                    tmp[n++] = (char)('0' + v % 10u);
                    v /= 10u;
                } while (v > 0 && n < sizeof tmp);
                for (unsigned i = 0; i < n; ++i) buf[i] = tmp[n - 1 - i];
                buf[n] = '\0';
            }
            hal_annotate_cell_text((uint8_t)x, (uint8_t)y, buf);
        }
}

static dir_t next(target_t target, flood_mode_t mode, uint8_t x, uint8_t y, dir_t heading) {
    if (plan_field_build(target, mode)) {
        annotate_field();
        hal_telemetry("rebuilds", (float)++g_rebuilds);
    }
    if (plan_field_cost(x, y, heading) == COST_INF) return DIR_NONE;
    return plan_field_next(x, y, heading, mode);
}

static const solver_t g_solver = {
    "demo_dijkstra",
    "Turn-Weighted Dijkstra: lowest time cost to the target, 90-degree turns cost 2.5 cells",
    init,
    next,
};

void fw_main(void) {
    solver_main(&g_solver);
}
