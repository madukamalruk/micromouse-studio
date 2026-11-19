#pragma once

#include <vector>

#include "sim/core/maze.hpp"

namespace mmsim::core {

/// Physical maze dimensions in metres. World origin = centre of the south-west post;
/// post (i,j) sits at (i*cell, j*cell). Defaults: classic 16x16 contest maze.
struct MazeDims {
    double cell = 0.180;  // post-to-post pitch
    double wall = 0.012;  // wall/post thickness
};

struct Vec2d {
    double x = 0.0;
    double y = 0.0;
};

/// Axis-aligned rectangle, x0 <= x1, y0 <= y1 (metres).
struct Rect {
    double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
};

/// One distinct wall segment between two posts; (cell, dir) is a canonical owner.
struct WallRect {
    Rect rect;
    Cell cell;
    Dir dir;
};

std::vector<Rect> post_rects(const Maze& m, const MazeDims& d = {});
std::vector<WallRect> wall_rects(const Maze& m, const MazeDims& d = {});
Rect maze_bounds(const Maze& m, const MazeDims& d = {});
Vec2d cell_center(Cell c, const MazeDims& d = {});
/// Wall rectangle for (cell, dir) whether or not the wall exists (used for "missing wall" probes).
Rect wall_slot(Cell c, Dir dir, const MazeDims& d = {});

}  // namespace mmsim::core
