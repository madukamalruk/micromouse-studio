#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "sim/app/camera.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/maze_geometry.hpp"
#include "sim/core/maze_validate.hpp"

struct ImDrawList;

namespace mmsim::app {

/// RGBA8 colours, ImGui IM_COL32 byte order (R in the low byte).
struct Rgba {
    std::uint8_t r, g, b, a;
    std::uint32_t packed() const {
        return std::uint32_t(r) | (std::uint32_t(g) << 8) | (std::uint32_t(b) << 16) | (std::uint32_t(a) << 24);
    }
};

namespace colors {
inline constexpr Rgba kBackground{24, 26, 31, 255};
inline constexpr Rgba kFloor{36, 39, 46, 255};
inline constexpr Rgba kGrid{48, 52, 61, 255};
inline constexpr Rgba kWall{214, 69, 65, 255};     // real wall (mms-style red)
inline constexpr Rgba kPost{205, 209, 217, 255};
inline constexpr Rgba kStart{40, 92, 64, 255};
inline constexpr Rgba kGoal{110, 86, 30, 255};
inline constexpr Rgba kText{150, 156, 168, 255};
}  // namespace colors

struct MazeViewOptions {
    bool show_grid = true;
    bool show_distances = true;
    bool show_coords = true;
    bool show_start_goal = true;
};

/// Draws the maze (floor, start/goal tint, grid, distances, walls, posts, coordinates).
void draw_maze(ImDrawList* dl, const Camera& cam, const core::Maze& maze, const core::FloodMap* flood,
               const core::MazeDims& dims, const MazeViewOptions& opt);

/// CPU-side image, RGBA8, row-major, top-left origin.
struct Image {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> rgba;
    Rgba at(int x, int y) const;
};

/// Pixel-level check that the rendered image matches the maze: every present wall's centre is
/// wall-coloured, every missing wall slot is not, every post centre is post-coloured.
struct RenderCheck {
    int checked = 0;
    std::vector<std::string> failures;
    bool ok() const { return checked > 0 && failures.empty(); }
};
RenderCheck check_maze_pixels(const Image& img, const Camera& cam, const core::Maze& maze,
                              const core::MazeDims& dims);

}  // namespace mmsim::app
