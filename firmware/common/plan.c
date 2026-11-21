#include "plan.h"

#define NSTATES ((unsigned)MAZE_SIZE * MAZE_SIZE * 4u)

static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {1, 0, -1, 0};

static unsigned sidx(unsigned x, unsigned y, unsigned h) {
    return (y * MAZE_SIZE + x) * 4u + h;
}

unsigned plan_turn_cost(dir_t from, dir_t to) {
    const unsigned diff = ((unsigned)to - (unsigned)from) & 3u;
    return COST_TURN * (diff == 0 ? 0u : diff == 2 ? 2u : 1u);
}

/* ---- Binary min-heap with lazy deletion (shared by Dijkstra and A*) ---------------------- */

typedef struct {
    uint16_t key, state;
} heap_item_t;

#define HEAP_CAP 4096u
static heap_item_t g_heap[HEAP_CAP];
static unsigned g_heap_n;

static void heap_push(uint16_t key, uint16_t state) {
    if (g_heap_n >= HEAP_CAP) return; /* never overflows the array; planning stays safe */
    unsigned i = g_heap_n++;
    while (i > 0) {
        const unsigned p = (i - 1) / 2;
        if (g_heap[p].key <= key) break;
        g_heap[i] = g_heap[p];
        i = p;
    }
    g_heap[i].key = key;
    g_heap[i].state = state;
}

static bool heap_pop(heap_item_t* out) {
    if (g_heap_n == 0) return false;
    *out = g_heap[0];
    const heap_item_t last = g_heap[--g_heap_n];
    if (g_heap_n == 0) return true;
    unsigned i = 0;
    for (;;) {
        unsigned c = 2 * i + 1;
        if (c >= g_heap_n) break;
        if (c + 1 < g_heap_n && g_heap[c + 1].key < g_heap[c].key) ++c;
        if (g_heap[c].key >= last.key) break;
        g_heap[i] = g_heap[c];
        i = c;
    }
    g_heap[i] = last;
    return true;
}

/* ---- Cell-count flood ------------------------------------------------------------------- */

dir_t plan_flood_next(target_t t, flood_mode_t m, uint8_t x, uint8_t y, dir_t heading,
                      bool annotate) {
    static bool valid, annotated;
    static uint16_t version;
    static target_t ct;
    static flood_mode_t cm;
    if (!valid || version != maze_version() || ct != t || cm != m) {
        maze_flood(t, m, x, y);
        valid = true;
        annotated = false;
        version = maze_version();
        ct = t;
        cm = m;
    }
    if (annotate && !annotated) {
        maze_annotate_distances();
        annotated = true;
    }
    if (maze_distance(x, y) == FLOOD_UNREACHABLE) return DIR_NONE;
    return maze_best_dir(x, y, heading, m);
}

/* ---- Turn-weighted Dijkstra field ------------------------------------------------------- */

static uint16_t g_field[NSTATES];

bool plan_field_build(target_t t, flood_mode_t m) {
    static bool valid;
    static uint16_t version;
    static target_t ct;
    static flood_mode_t cm;
    if (valid && version == maze_version() && ct == t && cm == m) return false;
    valid = true;
    version = maze_version();
    ct = t;
    cm = m;

    for (unsigned s = 0; s < NSTATES; ++s) g_field[s] = COST_INF;
    g_heap_n = 0;
    for (unsigned x = 0; x < MAZE_SIZE; ++x)
        for (unsigned y = 0; y < MAZE_SIZE; ++y)
            if (maze_is_target((uint8_t)x, (uint8_t)y, t))
                for (unsigned h = 0; h < 4; ++h) {
                    g_field[sidx(x, y, h)] = 0;
                    heap_push(0, (uint16_t)sidx(x, y, h));
                }
    heap_item_t it;
    while (heap_pop(&it)) {
        const unsigned s = it.state;
        if (it.key > g_field[s]) continue; /* stale */
        const unsigned d = s & 3u, cell = s >> 2;
        const int x = (int)(cell % MAZE_SIZE), y = (int)(cell / MAZE_SIZE);
        const int px = x - DX[d], py = y - DY[d]; /* the cell we moved in from, heading d */
        if (!maze_in_bounds(px, py) || !maze_passable((uint8_t)px, (uint8_t)py, (dir_t)d, m))
            continue;
        for (unsigned h = 0; h < 4; ++h) {
            const unsigned nc = it.key + plan_turn_cost((dir_t)h, (dir_t)d) + COST_CELL;
            const unsigned ps = sidx((unsigned)px, (unsigned)py, h);
            if (nc < g_field[ps]) {
                g_field[ps] = (uint16_t)nc;
                heap_push((uint16_t)nc, (uint16_t)ps);
            }
        }
    }
    return true;
}

uint16_t plan_field_cost(uint8_t x, uint8_t y, dir_t heading) {
    return g_field[sidx(x, y, (unsigned)heading)];
}

uint16_t plan_field_cell_cost(uint8_t x, uint8_t y) {
    uint16_t best = COST_INF;
    for (unsigned h = 0; h < 4; ++h)
        if (g_field[sidx(x, y, h)] < best) best = g_field[sidx(x, y, h)];
    return best;
}

dir_t plan_field_next(uint8_t x, uint8_t y, dir_t heading, flood_mode_t m) {
    const dir_t order[4] = {heading, dir_left(heading), dir_right(heading), dir_behind(heading)};
    dir_t best = DIR_NONE;
    unsigned best_c = COST_INF;
    for (int i = 0; i < 4; ++i) {
        const dir_t d = order[i];
        if (!maze_passable(x, y, d, m)) continue;
        const int nx = x + DX[d], ny = y + DY[d];
        if (!maze_in_bounds(nx, ny)) continue;
        const unsigned c = g_field[sidx((unsigned)nx, (unsigned)ny, (unsigned)d)];
        if (c == COST_INF) continue;
        const unsigned total = c + plan_turn_cost(heading, d) + COST_CELL;
        if (total < best_c) { /* strict: ties keep ahead > left > right > back */
            best_c = total;
            best = d;
        }
    }
    return best;
}

/* ---- A* --------------------------------------------------------------------------------- */

static uint16_t g_g[NSTATES];
static uint16_t g_parent[NSTATES];

static unsigned heuristic(int x, int y, target_t t) {
    int dx, dy;
    if (t == TARGET_START) {
        dx = x;
        dy = y;
    } else {
        const int c = MAZE_SIZE / 2;
        dx = x < c - 1 ? (c - 1) - x : (x > c ? x - c : 0);
        dy = y < c - 1 ? (c - 1) - y : (y > c ? y - c : 0);
    }
    return COST_CELL * (unsigned)(dx + dy) + ((dx > 0 && dy > 0) ? COST_TURN : 0u);
}

bool plan_astar(target_t t, flood_mode_t m, uint8_t x, uint8_t y, dir_t heading, plan_path_t* out) {
    for (unsigned s = 0; s < NSTATES; ++s) g_g[s] = COST_INF;
    g_heap_n = 0;
    const unsigned start = sidx(x, y, (unsigned)heading);
    g_g[start] = 0;
    g_parent[start] = (uint16_t)start;
    heap_push((uint16_t)heuristic(x, y, t), (uint16_t)start);
    out->len = 0;
    out->cost = 0;
    out->sx = x;
    out->sy = y;
    out->sh = (uint8_t)heading;

    heap_item_t it;
    while (heap_pop(&it)) {
        const unsigned s = it.state;
        const int cx = (int)((s >> 2) % MAZE_SIZE), cy = (int)((s >> 2) / MAZE_SIZE);
        const unsigned hd = s & 3u;
        if (it.key > g_g[s] + heuristic(cx, cy, t)) continue; /* stale */
        if (maze_is_target((uint8_t)cx, (uint8_t)cy, t)) {
            /* Walk the parents back to the start state. */
            unsigned n = 0, cur = s;
            while (cur != start && n < PLAN_MAX_STEPS) {
                out->dir[n++] = (uint8_t)(cur & 3u);
                cur = g_parent[cur];
            }
            if (cur != start) return false;
            for (unsigned i = 0; i < n / 2; ++i) { /* reverse in place */
                const uint8_t tmp = out->dir[i];
                out->dir[i] = out->dir[n - 1 - i];
                out->dir[n - 1 - i] = tmp;
            }
            out->len = (uint16_t)n;
            out->cost = g_g[s];
            return true;
        }
        for (unsigned d = 0; d < 4; ++d) {
            if (!maze_passable((uint8_t)cx, (uint8_t)cy, (dir_t)d, m)) continue;
            const int nx = cx + DX[d], ny = cy + DY[d];
            if (!maze_in_bounds(nx, ny)) continue;
            const unsigned ng = g_g[s] + plan_turn_cost((dir_t)hd, (dir_t)d) + COST_CELL;
            const unsigned ns = sidx((unsigned)nx, (unsigned)ny, d);
            if (ng <
                g_g[ns]) { /* re-opening allowed: the heuristic is admissible, not consistent */
                g_g[ns] = (uint16_t)ng;
                g_parent[ns] = (uint16_t)s;
                heap_push((uint16_t)(ng + heuristic(nx, ny, t)), (uint16_t)ns);
            }
        }
    }
    return false;
}
