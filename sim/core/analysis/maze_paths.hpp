#pragma once
// Cell-level route planning for the maze inspector / path preview (Slice 9.3, 9.5).
// Ported from Neomouse V2 dashboard_analysis.py (shortest_path, distance_field,
// potential_field); cross-checked on tests/fixtures/analysis/maze_paths.json.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "sim/core/maze.hpp"

namespace mmsim::core::analysis {

/// Wall bits per cell N=1 E=2 S=4 W=8 (same as core::Maze). `known` empty = all known.
struct WallGrid {
    int width = 16;
    int height = 16;
    std::vector<std::uint8_t> walls;  // y * width + x
    std::vector<std::uint8_t> known;  // same layout, or empty
    std::vector<Cell> goals;          // default: 2x2 centre box
    Cell start{0, 0};

    std::uint8_t wall(int x, int y) const { return walls[static_cast<std::size_t>(y * width + x)]; }
    bool has_known() const { return !known.empty(); }
    std::uint8_t kn(int x, int y) const {
        return known.empty() ? 15 : known[static_cast<std::size_t>(y * width + x)];
    }
    bool in(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
    bool is_goal(int x, int y) const;

    static WallGrid from_maze(const Maze& m);
    /// Centre goal box of a width x height maze (four cells, or one/two for odd sizes).
    static std::vector<Cell> centre_goals(int width, int height);
};

struct RouteStep {
    int x = 0, y = 0;
    int heading = 0;  // 0 N, 1 E, 2 S, 3 W (arrival heading)
};

struct ShortestPath {
    std::vector<Cell> cells;
    int moves = 0;
    int turns = 0;
    std::string basis;  // "known open walls only" / "unknown sides assumed open"
    bool optimistic = false;
};

/// Cheapest route start -> goal: 1 per cell + turn_cost per 90 deg (Dijkstra over cell+heading).
std::optional<ShortestPath> shortest_path(const WallGrid& g, double turn_cost = 0.5);

struct DistanceField {
    std::vector<int> steps;    // -1 = no route
    std::vector<int> heading;  // -1 = none (goal / no route)
    std::vector<Cell> dead_ends;
    int unreachable = 0;
    bool optimistic = false;
};
/// Breadth-first cell count to the goal box.
DistanceField distance_field(const WallGrid& g, bool optimistic = true);

struct PotentialField {
    std::vector<double> cost;  // < 0 = no route; goal cells 0
    std::vector<int> heading;  // -1 = none
    double turn_cost = 0.5;
};
/// Turn-weighted Dijkstra cost to the goal box (unknown sides open), rounded to 0.01.
PotentialField potential_field(const WallGrid& g, double turn_cost = 0.5);

/// Side open test used by distance/potential fields (checks both cells' knowledge).
bool side_open(const WallGrid& g, int x, int y, int d, bool optimistic);

}  // namespace mmsim::core::analysis
