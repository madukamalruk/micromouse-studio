#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "sim/core/maze.hpp"

namespace mmsim::core {

/// BFS step distance from every cell to the nearest goal cell.
class FloodMap {
public:
    static constexpr int kUnreachable = -1;
    FloodMap(int width, int height) : width_(width), dist_(static_cast<std::size_t>(width * height), kUnreachable) {}
    int at(int x, int y) const { return dist_[static_cast<std::size_t>(y * width_ + x)]; }
    void set(int x, int y, int d) { dist_[static_cast<std::size_t>(y * width_ + x)] = d; }

private:
    int width_;
    std::vector<int> dist_;
};

FloodMap flood_distances(const Maze& m);

/// mms-style validity + "official contest maze" checks. Reported, never auto-fixed.
struct ValidationReport {
    bool enclosed = false;
    bool solvable = false;                   // a goal is reachable from start
    std::vector<Cell> unreachable;           // cells not reachable from start
    int start_wall_count = 0;                // official: exactly 3
    int goal_entrances = 0;                  // openings from goal region to outside; official: 1
    bool hollow_centre_post = false;         // no walls between goal cells
    bool all_posts_have_walls = false;       // every post except interior goal posts touches a wall
    bool left_wall_follower_solves = false;  // official: false
    bool right_wall_follower_solves = false; // official: false

    bool is_official() const;
    std::vector<std::string> issues() const;  // human-readable list of failed official rules
};

ValidationReport validate(const Maze& m);

}  // namespace mmsim::core
