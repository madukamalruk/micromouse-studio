// The path planners of firmware/common (plan.c: flood fill, turn-weighted Dijkstra field, A*),
// compiled for the host and checked against independent reference implementations on generated
// mazes with full wall knowledge.
#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <queue>
#include <tuple>
#include <vector>

#include "maze_gen.hpp"
#include "sim/core/maze.hpp"

extern "C" {
#include "firmware/common/maze.h"
#include "firmware/common/plan.h"
}

using mmsim::core::Dir;
using mmsim::core::Maze;

namespace {

constexpr int N = 16;
const int DX[4] = {0, 1, 0, -1};
const int DY[4] = {1, 0, -1, 0};

// Teach the firmware's map every wall of `m` (what the mouse knows after a perfect search).
void load_known(const Maze& m) {
    maze_init();
    for (int x = 0; x < N; ++x)
        for (int y = 0; y < N; ++y)
            for (int d = 0; d < 4; ++d)
                maze_set_wall(static_cast<uint8_t>(x), static_cast<uint8_t>(y),
                              static_cast<dir_t>(d), m.has_wall(x, y, static_cast<Dir>(d)));
}

bool is_goal(int x, int y) {
    return (x == 7 || x == 8) && (y == 7 || y == 8);
}

unsigned turn_cost(int from, int to) {
    const int diff = (to - from + 4) & 3;
    return COST_TURN * (diff == 0 ? 0u : diff == 2 ? 2u : 1u);
}

// Reference: plain forward Dijkstra over (x, y, heading) with std::priority_queue.
unsigned ref_cost(const Maze& m, int sx, int sy, int sh, bool to_start = false) {
    using Item = std::tuple<unsigned, int, int, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> pq;
    std::vector<unsigned> best(N * N * 4, 0xFFFFFFFFu);
    auto idx = [](int x, int y, int h) { return (y * N + x) * 4 + h; };
    best[idx(sx, sy, sh)] = 0;
    pq.push({0, sx, sy, sh});
    while (!pq.empty()) {
        const auto [c, x, y, h] = pq.top();
        pq.pop();
        if (c > best[idx(x, y, h)]) continue;
        if (to_start ? (x == 0 && y == 0) : is_goal(x, y)) return c;
        for (int d = 0; d < 4; ++d) {
            if (m.has_wall(x, y, static_cast<Dir>(d))) continue;
            const int nx = x + DX[d], ny = y + DY[d];
            if (nx < 0 || ny < 0 || nx >= N || ny >= N) continue;
            const unsigned nc = c + turn_cost(h, d) + COST_CELL;
            if (nc < best[idx(nx, ny, d)]) {
                best[idx(nx, ny, d)] = nc;
                pq.push({nc, nx, ny, d});
            }
        }
    }
    return 0xFFFFFFFFu;
}

// Reference BFS length in cells from (sx, sy) to the goal block.
int ref_cells(const Maze& m, int sx, int sy) {
    std::vector<int> dist(N * N, -1);
    std::queue<std::pair<int, int>> q;
    dist[sy * N + sx] = 0;
    q.push({sx, sy});
    while (!q.empty()) {
        const auto [x, y] = q.front();
        q.pop();
        if (is_goal(x, y)) return dist[y * N + x];
        for (int d = 0; d < 4; ++d) {
            if (m.has_wall(x, y, static_cast<Dir>(d))) continue;
            const int nx = x + DX[d], ny = y + DY[d];
            if (nx < 0 || ny < 0 || nx >= N || ny >= N || dist[ny * N + nx] >= 0) continue;
            dist[ny * N + nx] = dist[y * N + x] + 1;
            q.push({nx, ny});
        }
    }
    return -1;
}

}  // namespace

TEST_SUITE("planners") {
    TEST_CASE("turn cost: straight 0, quarter turn 5, U-turn 10") {
        CHECK(plan_turn_cost(NORTH, NORTH) == 0);
        CHECK(plan_turn_cost(NORTH, EAST) == COST_TURN);
        CHECK(plan_turn_cost(NORTH, WEST) == COST_TURN);
        CHECK(plan_turn_cost(EAST, WEST) == 2 * COST_TURN);
    }

    TEST_CASE("flood fill: steps downhill along a shortest path in cells") {
        for (std::uint64_t seed = 1; seed <= 12; ++seed) {
            CAPTURE(seed);
            const Maze m = test_maze::generate(seed);
            load_known(m);
            uint8_t x = 0, y = 0;
            dir_t h = NORTH;
            int steps = 0;
            while (!is_goal(x, y) && steps < 300) {
                const dir_t d = plan_flood_next(TARGET_GOAL, FLOOD_KNOWN, x, y, h, false);
                REQUIRE(d != DIR_NONE);
                maze_neighbour(x, y, d, &x, &y);
                h = d;
                ++steps;
            }
            CHECK(steps == ref_cells(m, 0, 0));
        }
    }

    TEST_CASE("dijkstra field and A* both equal the reference optimum") {
        int tested = 0;
        for (std::uint64_t seed = 1; seed <= 15; ++seed) {
            const Maze m = test_maze::generate(seed, N, seed % 3 == 0 ? 60 : 20);
            load_known(m);
            plan_field_build(TARGET_GOAL, FLOOD_KNOWN);
            // Several start states per maze: the start cell and some others, all headings.
            const int starts[][2] = {{0, 0}, {3, 5}, {12, 2}, {1, 14}, {10, 10}, {15, 15}};
            for (const auto& s : starts)
                for (int h = 0; h < 4; ++h) {
                    const int sx = s[0], sy = s[1];
                    if (is_goal(sx, sy)) continue;
                    CAPTURE(seed);
                    CAPTURE(sx);
                    CAPTURE(sy);
                    CAPTURE(h);
                    const unsigned want = ref_cost(m, sx, sy, h);
                    REQUIRE(want != 0xFFFFFFFFu);
                    CHECK(plan_field_cost(static_cast<uint8_t>(sx), static_cast<uint8_t>(sy),
                                          static_cast<dir_t>(h)) == want);
                    static plan_path_t path;
                    REQUIRE(plan_astar(TARGET_GOAL, FLOOD_KNOWN, static_cast<uint8_t>(sx),
                                       static_cast<uint8_t>(sy), static_cast<dir_t>(h), &path));
                    CHECK(path.cost == want);
                    // The path itself: legal, ends in the goal, and costs what it claims.
                    int x = sx, y = sy, hd = h;
                    unsigned c = 0;
                    for (unsigned i = 0; i < path.len; ++i) {
                        const int d = path.dir[i];
                        REQUIRE_FALSE(m.has_wall(x, y, static_cast<Dir>(d)));
                        c += turn_cost(hd, d) + COST_CELL;
                        x += DX[d];
                        y += DY[d];
                        hd = d;
                    }
                    CHECK(is_goal(x, y));
                    CHECK(c == path.cost);
                    // Following the field's own steps also lands on the optimum.
                    x = sx;
                    y = sy;
                    hd = h;
                    c = 0;
                    for (int guard = 0; !is_goal(x, y) && guard < 600; ++guard) {
                        const dir_t d =
                            plan_field_next(static_cast<uint8_t>(x), static_cast<uint8_t>(y),
                                            static_cast<dir_t>(hd), FLOOD_KNOWN);
                        REQUIRE(d != DIR_NONE);
                        c += turn_cost(hd, d) + COST_CELL;
                        x += DX[d];
                        y += DY[d];
                        hd = d;
                    }
                    CHECK(c == want);
                    ++tested;
                }
        }
        CHECK(tested > 300);
    }

    TEST_CASE("A* and Dijkstra also reach the start cell (the way back)") {
        for (std::uint64_t seed = 1; seed <= 8; ++seed) {
            const Maze m = test_maze::generate(seed);
            load_known(m);
            plan_field_build(TARGET_START, FLOOD_KNOWN);
            const unsigned want = ref_cost(m, 7, 7, 0, /*to_start=*/true);
            CHECK(plan_field_cost(7, 7, NORTH) == want);
            static plan_path_t path;
            REQUIRE(plan_astar(TARGET_START, FLOOD_KNOWN, 7, 7, NORTH, &path));
            CHECK(path.cost == want);
        }
    }

    TEST_CASE("unknown walls: search mode is optimistic, known mode refuses to cross them") {
        maze_init();  // only the outer walls and the start cell are known
        CHECK(plan_field_build(TARGET_GOAL, FLOOD_SEARCH));
        CHECK(plan_field_cost(0, 0, NORTH) != COST_INF);
        CHECK_FALSE(plan_field_build(TARGET_GOAL, FLOOD_SEARCH));  // cached
        plan_field_build(TARGET_GOAL, FLOOD_KNOWN);
        CHECK(plan_field_cost(0, 0, NORTH) == COST_INF);  // nothing known to be open
        static plan_path_t path;
        CHECK(plan_astar(TARGET_GOAL, FLOOD_SEARCH, 0, 0, NORTH, &path));
        CHECK_FALSE(plan_astar(TARGET_GOAL, FLOOD_KNOWN, 0, 0, NORTH, &path));
        // A new wall invalidates the cache.
        plan_field_build(TARGET_GOAL, FLOOD_SEARCH);
        maze_set_wall(0, 1, NORTH, true);
        CHECK(plan_field_build(TARGET_GOAL, FLOOD_SEARCH));
    }

    TEST_CASE("a sealed goal is unreachable for every planner") {
        maze_init();
        for (int x = 7; x <= 8; ++x) {
            maze_set_wall(static_cast<uint8_t>(x), 8, NORTH, true);
            maze_set_wall(static_cast<uint8_t>(x), 7, SOUTH, true);
        }
        for (int y = 7; y <= 8; ++y) {
            maze_set_wall(7, static_cast<uint8_t>(y), WEST, true);
            maze_set_wall(8, static_cast<uint8_t>(y), EAST, true);
        }
        plan_field_build(TARGET_GOAL, FLOOD_SEARCH);
        CHECK(plan_field_cost(0, 0, NORTH) == COST_INF);
        static plan_path_t path;
        CHECK_FALSE(plan_astar(TARGET_GOAL, FLOOD_SEARCH, 0, 0, NORTH, &path));
        CHECK(plan_flood_next(TARGET_GOAL, FLOOD_SEARCH, 0, 0, NORTH, false) == DIR_NONE);
    }
}
