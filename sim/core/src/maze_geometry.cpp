#include "sim/core/maze_geometry.hpp"

namespace mmsim::core {

std::vector<Rect> post_rects(const Maze& m, const MazeDims& d) {
    std::vector<Rect> out;
    const double h = d.wall / 2.0;
    out.reserve(static_cast<std::size_t>((m.width() + 1) * (m.height() + 1)));
    for (int j = 0; j <= m.height(); ++j)
        for (int i = 0; i <= m.width(); ++i) {
            const double cx = i * d.cell, cy = j * d.cell;
            out.push_back({cx - h, cy - h, cx + h, cy + h});
        }
    return out;
}

Rect wall_slot(Cell c, Dir dir, const MazeDims& d) {
    const double h = d.wall / 2.0;
    const double x0 = c.x * d.cell, y0 = c.y * d.cell;
    const double x1 = x0 + d.cell, y1 = y0 + d.cell;
    switch (dir) {
        case Dir::N: return {x0 + h, y1 - h, x1 - h, y1 + h};
        case Dir::S: return {x0 + h, y0 - h, x1 - h, y0 + h};
        case Dir::E: return {x1 - h, y0 + h, x1 + h, y1 - h};
        case Dir::W: return {x0 - h, y0 + h, x0 + h, y1 - h};
    }
    return {};
}

std::vector<WallRect> wall_rects(const Maze& m, const MazeDims& d) {
    std::vector<WallRect> out;
    for (int y = 0; y < m.height(); ++y)
        for (int x = 0; x < m.width(); ++x) {
            const Cell c{x, y};
            if (m.has_wall(c, Dir::N)) out.push_back({wall_slot(c, Dir::N, d), c, Dir::N});
            if (m.has_wall(c, Dir::E)) out.push_back({wall_slot(c, Dir::E, d), c, Dir::E});
            if (y == 0 && m.has_wall(c, Dir::S)) out.push_back({wall_slot(c, Dir::S, d), c, Dir::S});
            if (x == 0 && m.has_wall(c, Dir::W)) out.push_back({wall_slot(c, Dir::W, d), c, Dir::W});
        }
    return out;
}

Rect maze_bounds(const Maze& m, const MazeDims& d) {
    const double h = d.wall / 2.0;
    return {-h, -h, m.width() * d.cell + h, m.height() * d.cell + h};
}

Vec2d cell_center(Cell c, const MazeDims& d) { return {(c.x + 0.5) * d.cell, (c.y + 0.5) * d.cell}; }

}  // namespace mmsim::core
