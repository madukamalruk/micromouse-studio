#pragma once
// Deterministic classic-style 16x16 test mazes (no external files needed): DFS perfect maze,
// start (0,0) facing north with walls W/E/S, centre 2x2 goal opened into one room, plus a few
// removed walls so there are loops (like real contest mazes).
#include <utility>
#include <vector>

#include "sim/core/maze.hpp"
#include "sim/core/rng.hpp"

namespace test_maze {

inline mmsim::core::Maze generate(std::uint64_t seed, int n = 16, int extra_openings = 20) {
    using namespace mmsim::core;
    Maze m(n, n);
    for (int x = 0; x < n; ++x)
        for (int y = 0; y < n; ++y)
            for (Dir d : {Dir::N, Dir::E, Dir::S, Dir::W}) m.set_wall(x, y, d, true);
    Rng rng(derive_seed(seed, "maze_gen"));
    std::vector<char> seen(static_cast<std::size_t>(n * n), 0);
    auto at = [&](int x, int y) -> char& { return seen[static_cast<std::size_t>(y * n + x)]; };
    const int dx[4] = {0, 1, 0, -1}, dy[4] = {1, 0, -1, 0};
    // Start cell opens only to the north.
    at(0, 0) = 1;
    m.set_wall(0, 0, Dir::N, false);
    std::vector<std::pair<int, int>> stack{{0, 1}};
    at(0, 1) = 1;
    while (!stack.empty()) {
        const auto [x, y] = stack.back();
        int opts[4], k = 0;
        for (int d = 0; d < 4; ++d) {
            const int nx = x + dx[d], ny = y + dy[d];
            if (m.in_bounds(nx, ny) && !at(nx, ny)) opts[k++] = d;
        }
        if (k == 0) {
            stack.pop_back();
            continue;
        }
        const int d = opts[rng.next() % static_cast<std::uint64_t>(k)];
        m.set_wall(x, y, static_cast<Dir>(d), false);
        at(x + dx[d], y + dy[d]) = 1;
        stack.push_back({x + dx[d], y + dy[d]});
    }
    // Centre room.
    const int c = n / 2;
    m.set_wall(c - 1, c - 1, Dir::N, false);
    m.set_wall(c - 1, c - 1, Dir::E, false);
    m.set_wall(c, c, Dir::S, false);
    m.set_wall(c, c, Dir::W, false);
    // Loops: knock out random interior walls (never touching the start cell).
    for (int i = 0; i < extra_openings; ++i) {
        const int x = 1 + static_cast<int>(rng.next() % static_cast<std::uint64_t>(n - 2));
        const int y = 1 + static_cast<int>(rng.next() % static_cast<std::uint64_t>(n - 2));
        m.set_wall(x, y, static_cast<Dir>(rng.next() % 4), false);
    }
    m.set_start({0, 0}, Dir::N);
    m.set_goals({{c - 1, c - 1}, {c - 1, c}, {c, c - 1}, {c, c}});
    return m;
}

}  // namespace test_maze
