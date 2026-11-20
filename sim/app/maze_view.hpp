#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "sim/app/camera.hpp"
#include "sim/app/palette.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/maze_geometry.hpp"
#include "sim/core/maze_validate.hpp"
#include "sim/host/annotations.hpp"

struct ImDrawList;

namespace mmsim::app {

namespace colors {
// Maze rendering (DESIGN.md "Maze rendering"); all derived from the tokens in palette.hpp.
inline constexpr Rgba kBackground = tok::canvas;
inline constexpr Rgba kFloor{17, 21, 27,
                             255};             // a lift above the canvas: the maze reads as a board
inline constexpr Rgba kGrid{27, 32, 42, 255};  // hairline at ~50 % over the floor
inline constexpr Rgba kWall{220, 227, 236, 255};  // real wall: light, crisp against the floor
inline constexpr Rgba kPost = tok::text;
inline constexpr Rgba kStart = mix(kFloor, tok::accent, 0.16f);
inline constexpr Rgba kGoal = mix(kFloor, tok::good, 0.20f);
inline constexpr Rgba kText{96, 106, 122, 255};  // BFS distances / coordinates: quiet
inline constexpr Rgba kFwText = tok::text;
inline constexpr Rgba kKnownWall = tok::accent;  // firmware map agrees with the maze
inline constexpr Rgba kWrongWall = tok::warn;    // firmware map disagrees
}  // namespace colors

struct MazeViewOptions {
    bool show_grid = true;
    bool show_distances = true;
    bool show_coords = true;
    bool show_start_goal = true;
    bool show_true_walls = true;   // the real maze walls
    bool show_fw_colors = true;    // hal_annotate_cell_color fills
    bool show_fw_text = true;      // hal_annotate_cell_text (replaces the idle BFS distances)
    bool show_known_walls = true;  // hal_annotate_wall map (mismatches highlighted)
};

/// Draws the maze (floor, start/goal tint, firmware cell colours, grid, distances or firmware
/// text, walls, firmware wall map, posts, coordinates). `ann` may be null (no firmware).
void draw_maze(ImDrawList* dl, const Camera& cam, const core::Maze& maze,
               const core::FloodMap* flood, const core::MazeDims& dims, const MazeViewOptions& opt,
               const host::Annotations* ann = nullptr);

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
