/*
 * demo_astar - dynamic A* with lazy replanning, pure C11 on mmhal only.
 *
 * One path at a time instead of a whole map: A* searches from the mouse to the nearest target
 * cell with the same turn-weighted costs as demo_dijkstra, guided by an admissible heuristic
 * (2 per cell of Manhattan distance, plus one turn when both axes are still open).
 *
 * While exploring, unknown walls are assumed open, so a newly discovered wall can only make
 * *other* routes worse. The plan therefore stays optimal until a wall lands on the planned
 * path itself; only then does the mouse replan. The numbers in the cells are the cost still
 * to pay along the current plan, and only the planned path carries numbers.
 */
#include "common/plan.h"
#include "common/solver.h"
#include "mmhal/hal.h"

static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {1, 0, -1, 0};

static plan_path_t g_path;
static bool g_have_path;
static uint16_t g_version;
static target_t g_target;
static flood_mode_t g_mode;
static unsigned g_replans;
static uint16_t g_shown[MAZE_SIZE][MAZE_SIZE];
static uint16_t g_want[MAZE_SIZE][MAZE_SIZE];

static void init(void) {
    g_have_path = false;
    for (int x = 0; x < MAZE_SIZE; ++x)
        for (int y = 0; y < MAZE_SIZE; ++y) g_shown[x][y] = 0xFFFEu;
}

/* Index of the step whose start state is (x, y, heading), or -1. */
static int find_step(uint8_t x, uint8_t y, dir_t heading) {
    int px = g_path.sx, py = g_path.sy;
    unsigned ph = g_path.sh;
    for (unsigned i = 0; i <= g_path.len; ++i) {
        if (px == x && py == y && ph == (unsigned)heading) return (int)i;
        if (i == g_path.len) break;
        px += DX[g_path.dir[i]];
        py += DY[g_path.dir[i]];
        ph = g_path.dir[i];
    }
    return -1;
}

/* Are all remaining edges of the plan from step `from` still passable? */
static bool path_clear(unsigned from, flood_mode_t m) {
    int px = g_path.sx, py = g_path.sy;
    for (unsigned i = 0; i < g_path.len; ++i) {
        if (i >= from && !maze_passable((uint8_t)px, (uint8_t)py, (dir_t)g_path.dir[i], m))
            return false;
        px += DX[g_path.dir[i]];
        py += DY[g_path.dir[i]];
    }
    return true;
}

/* Remaining cost along the plan from step `from`, written as cell text. */
static void annotate_path(unsigned from) {
    for (int x = 0; x < MAZE_SIZE; ++x)
        for (int y = 0; y < MAZE_SIZE; ++y) g_want[x][y] = COST_INF;
    /* Cells in order, then costs back to front. */
    int px = g_path.sx, py = g_path.sy;
    unsigned ph = g_path.sh;
    static int xs[PLAN_MAX_STEPS + 1], ys[PLAN_MAX_STEPS + 1];
    static unsigned step_cost[PLAN_MAX_STEPS + 1];
    for (unsigned i = 0; i < g_path.len; ++i) {
        xs[i] = px;
        ys[i] = py;
        step_cost[i] = plan_turn_cost((dir_t)ph, (dir_t)g_path.dir[i]) + COST_CELL;
        px += DX[g_path.dir[i]];
        py += DY[g_path.dir[i]];
        ph = g_path.dir[i];
    }
    xs[g_path.len] = px;
    ys[g_path.len] = py;
    unsigned rem = 0;
    g_want[px][py] = 0;
    for (unsigned i = g_path.len; i-- > from;) {
        rem += step_cost[i];
        g_want[xs[i]][ys[i]] = (uint16_t)rem;
    }
    char buf[8];
    for (int x = 0; x < MAZE_SIZE; ++x)
        for (int y = 0; y < MAZE_SIZE; ++y) {
            if (g_want[x][y] == g_shown[x][y]) continue;
            g_shown[x][y] = g_want[x][y];
            if (g_want[x][y] == COST_INF) {
                buf[0] = '\0';
            } else {
                unsigned v = g_want[x][y], n = 0;
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
    int step = -1;
    if (g_have_path && g_target == target && g_mode == mode) {
        step = find_step(x, y, heading);
        if (step >= 0 && g_version != maze_version()) {
            /* New walls: keep the plan only if none of them sits on what is left of it. */
            if (path_clear((unsigned)step, mode))
                g_version = maze_version();
            else
                step = -1;
        }
    }
    if (step < 0) {
        g_have_path = plan_astar(target, mode, x, y, heading, &g_path);
        if (!g_have_path) return DIR_NONE;
        g_target = target;
        g_mode = mode;
        g_version = maze_version();
        step = 0;
        hal_telemetry("replans", (float)++g_replans);
        hal_telemetry("plan_cost", (float)g_path.cost);
    }
    if ((unsigned)step >= g_path.len) return DIR_NONE; /* already at the target */
    if (mode == FLOOD_SEARCH) annotate_path((unsigned)step);
    return (dir_t)g_path.dir[step];
}

static const solver_t g_solver = {
    "demo_astar",
    "Dynamic A*: plan one optimal turn-weighted path, replan only when a wall blocks it",
    init,
    next,
};

void fw_main(void) {
    solver_main(&g_solver);
}
