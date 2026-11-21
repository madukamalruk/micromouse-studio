#include "maze.h"

#include <stdio.h>

#include "mmhal/hal.h"

static uint8_t g_walls[MAZE_SIZE][MAZE_SIZE]; /* bit d = wall known present */
static uint8_t g_known[MAZE_SIZE][MAZE_SIZE]; /* bit d = wall state known */
static uint16_t g_dist[MAZE_SIZE][MAZE_SIZE];
static uint16_t g_shown[MAZE_SIZE][MAZE_SIZE]; /* last value sent to the overlay */
static uint16_t g_version;                     /* bumps whenever a known wall changes */

static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {1, 0, -1, 0};
static const char DCHAR[4] = {'n', 'e', 's', 'w'};

static bool in_maze(int x, int y) {
    return x >= 0 && y >= 0 && x < MAZE_SIZE && y < MAZE_SIZE;
}

bool maze_in_bounds(int x, int y) {
    return in_maze(x, y);
}

uint16_t maze_version(void) {
    return g_version;
}

static void set_one(int x, int y, dir_t d, bool present) {
    const uint8_t bit = (uint8_t)(1u << d);
    const uint8_t before_w = g_walls[x][y], before_k = g_known[x][y];
    if (present)
        g_walls[x][y] |= bit;
    else
        g_walls[x][y] &= (uint8_t)~bit;
    g_known[x][y] |= bit;
    if (before_w != g_walls[x][y] || before_k != g_known[x][y]) ++g_version;
}

void maze_init(void) {
    ++g_version; /* monotonic: a reset map never looks like an older version to a cache */
    for (int x = 0; x < MAZE_SIZE; ++x)
        for (int y = 0; y < MAZE_SIZE; ++y) {
            g_walls[x][y] = 0;
            g_known[x][y] = 0;
            g_dist[x][y] = FLOOD_UNREACHABLE;
            g_shown[x][y] = 0xFFFEu;
        }
    for (int i = 0; i < MAZE_SIZE; ++i) {
        set_one(i, 0, SOUTH, true);
        set_one(i, MAZE_SIZE - 1, NORTH, true);
        set_one(0, i, WEST, true);
        set_one(MAZE_SIZE - 1, i, EAST, true);
    }
    /* Contest rule: the start cell is walled on three sides, open to the north. */
    maze_set_wall(0, 0, EAST, true);
    maze_set_wall(0, 0, NORTH, false);
}

void maze_neighbour(uint8_t x, uint8_t y, dir_t d, uint8_t* nx, uint8_t* ny) {
    *nx = (uint8_t)(x + DX[d]);
    *ny = (uint8_t)(y + DY[d]);
}

void maze_set_wall(uint8_t x, uint8_t y, dir_t d, bool present) {
    if (!in_maze(x, y)) return;
    const bool changed = !maze_known(x, y, d) || maze_wall(x, y, d) != present;
    set_one(x, y, d, present);
    const int nx = x + DX[d], ny = y + DY[d];
    if (in_maze(nx, ny)) set_one(nx, ny, dir_behind(d), present);
    if (changed) hal_annotate_wall(x, y, DCHAR[d], present);
}

bool maze_wall(uint8_t x, uint8_t y, dir_t d) {
    return (g_walls[x][y] >> d) & 1u;
}
bool maze_known(uint8_t x, uint8_t y, dir_t d) {
    return (g_known[x][y] >> d) & 1u;
}

bool maze_is_target(uint8_t x, uint8_t y, target_t t) {
    if (t == TARGET_START) return x == 0 && y == 0;
    const int c = MAZE_SIZE / 2;
    return (x == c - 1 || x == c) && (y == c - 1 || y == c);
}

bool maze_passable(uint8_t x, uint8_t y, dir_t d, flood_mode_t mode) {
    if (maze_wall(x, y, d)) return false;
    return mode == FLOOD_SEARCH || maze_known(x, y, d);
}

static bool passable(int x, int y, dir_t d, flood_mode_t mode) {
    return maze_passable((uint8_t)x, (uint8_t)y, d, mode);
}

uint16_t maze_flood(target_t t, flood_mode_t mode, uint8_t x, uint8_t y) {
    static uint8_t queue[MAZE_SIZE * MAZE_SIZE];
    unsigned head = 0, tail = 0;
    for (int i = 0; i < MAZE_SIZE; ++i)
        for (int j = 0; j < MAZE_SIZE; ++j) {
            g_dist[i][j] = FLOOD_UNREACHABLE;
            if (maze_is_target((uint8_t)i, (uint8_t)j, t)) {
                g_dist[i][j] = 0;
                queue[tail++] = (uint8_t)(i * MAZE_SIZE + j);
            }
        }
    while (head < tail) {
        const int cx = queue[head] / MAZE_SIZE, cy = queue[head] % MAZE_SIZE;
        ++head;
        for (int d = 0; d < 4; ++d) {
            if (!passable(cx, cy, (dir_t)d, mode)) continue;
            const int nx = cx + DX[d], ny = cy + DY[d];
            if (!in_maze(nx, ny) || g_dist[nx][ny] != FLOOD_UNREACHABLE) continue;
            g_dist[nx][ny] = (uint16_t)(g_dist[cx][cy] + 1);
            queue[tail++] = (uint8_t)(nx * MAZE_SIZE + ny);
        }
    }
    return g_dist[x][y];
}

uint16_t maze_distance(uint8_t x, uint8_t y) {
    return g_dist[x][y];
}

dir_t maze_best_dir(uint8_t x, uint8_t y, dir_t heading, flood_mode_t mode) {
    const dir_t order[4] = {heading, dir_left(heading), dir_right(heading), dir_behind(heading)};
    dir_t best = DIR_NONE;
    uint16_t best_d = g_dist[x][y];
    for (int i = 0; i < 4; ++i) {
        const dir_t d = order[i];
        if (!passable(x, y, d, mode)) continue;
        const int nx = x + DX[d], ny = y + DY[d];
        if (!in_maze(nx, ny)) continue;
        if (g_dist[nx][ny] < best_d) {
            best_d = g_dist[nx][ny];
            best = d;
        }
    }
    return best;
}

void maze_annotate_distances(void) {
    char buf[8];
    for (int x = 0; x < MAZE_SIZE; ++x)
        for (int y = 0; y < MAZE_SIZE; ++y) {
            if (g_dist[x][y] == g_shown[x][y]) continue;
            g_shown[x][y] = g_dist[x][y];
            if (g_dist[x][y] == FLOOD_UNREACHABLE)
                buf[0] = '\0';
            else
                snprintf(buf, sizeof buf, "%u", (unsigned)g_dist[x][y]);
            hal_annotate_cell_text((uint8_t)x, (uint8_t)y, buf);
        }
}
