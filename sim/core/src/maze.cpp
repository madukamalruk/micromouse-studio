#include "sim/core/maze.hpp"

#include <algorithm>
#include <stdexcept>

namespace mmsim::core {

Maze::Maze(int width, int height) : width_(width), height_(height) {
    if (width <= 0 || height <= 0 || width > 255 || height > 255)
        throw std::invalid_argument("maze dimensions must be 1..255");
    walls_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
}

void Maze::set_wall(int x, int y, Dir d, bool present) {
    if (!in_bounds(x, y)) return;
    auto apply = [present](std::uint8_t& w, Dir dir) {
        if (present)
            w = static_cast<std::uint8_t>(w | wall_bit(dir));
        else
            w = static_cast<std::uint8_t>(w & ~wall_bit(dir));
    };
    apply(walls_[index(x, y)], d);
    const int nx = x + dx(d), ny = y + dy(d);
    if (in_bounds(nx, ny)) apply(walls_[index(nx, ny)], opposite(d));
}

void Maze::add_boundary() {
    for (int x = 0; x < width_; ++x) {
        set_wall(x, 0, Dir::S, true);
        set_wall(x, height_ - 1, Dir::N, true);
    }
    for (int y = 0; y < height_; ++y) {
        set_wall(0, y, Dir::W, true);
        set_wall(width_ - 1, y, Dir::E, true);
    }
}

bool Maze::is_enclosed() const {
    for (int x = 0; x < width_; ++x)
        if (!has_wall(x, 0, Dir::S) || !has_wall(x, height_ - 1, Dir::N)) return false;
    for (int y = 0; y < height_; ++y)
        if (!has_wall(0, y, Dir::W) || !has_wall(width_ - 1, y, Dir::E)) return false;
    return true;
}

int Maze::wall_count() const {
    int n = 0;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            // Count N and E of every cell, plus S of the bottom row and W of the left column.
            if (has_wall(x, y, Dir::N)) ++n;
            if (has_wall(x, y, Dir::E)) ++n;
            if (y == 0 && has_wall(x, y, Dir::S)) ++n;
            if (x == 0 && has_wall(x, y, Dir::W)) ++n;
        }
    }
    return n;
}

bool Maze::is_goal(Cell c) const {
    return std::find(goals_.begin(), goals_.end(), c) != goals_.end();
}

void Maze::infer_start_heading() {
    int open = 0;
    Dir open_dir = Dir::N;
    for (Dir d : kAllDirs) {
        if (!has_wall(start_, d)) {
            ++open;
            open_dir = d;
        }
    }
    start_heading_ = (open == 1) ? open_dir : Dir::N;
}

void Maze::set_goals(std::vector<Cell> goals) {
    std::sort(goals.begin(), goals.end(),
              [](Cell a, Cell b) { return a.y != b.y ? a.y < b.y : a.x < b.x; });
    goals.erase(std::unique(goals.begin(), goals.end()), goals.end());
    goals_ = std::move(goals);
}

void Maze::set_default_goals() {
    std::vector<Cell> g;
    const int x0 = (width_ - 1) / 2, x1 = width_ / 2;
    const int y0 = (height_ - 1) / 2, y1 = height_ / 2;
    for (int x = x0; x <= x1; ++x)
        for (int y = y0; y <= y1; ++y) g.push_back({x, y});
    set_goals(std::move(g));
}

std::uint64_t Maze::hash() const {
    std::uint64_t h = 0xcbf29ce484222325ULL;
    auto mix = [&h](std::uint8_t v) {
        h ^= v;
        h *= 0x100000001b3ULL;
    };
    mix(static_cast<std::uint8_t>(width_));
    mix(static_cast<std::uint8_t>(height_));
    for (std::uint8_t w : walls_) mix(w);
    return h;
}

}  // namespace mmsim::core
