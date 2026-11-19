#include "sim/app/maze_view.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace mmsim::app {

using core::Cell;
using core::Dir;
using core::Rect;

namespace {

ImVec2 to_im(Vec2 v) { return {static_cast<float>(v.x), static_cast<float>(v.y)}; }

/// Screen-space rectangle (min/max corners) for a world rectangle.
void screen_rect(const Camera& cam, const Rect& r, ImVec2& lo, ImVec2& hi) {
    const Vec2 a = cam.world_to_screen({r.x0, r.y1});  // top-left in screen space
    const Vec2 b = cam.world_to_screen({r.x1, r.y0});
    lo = to_im(a);
    hi = to_im(b);
}

void fill(ImDrawList* dl, const Camera& cam, const Rect& r, Rgba c) {
    ImVec2 lo, hi;
    screen_rect(cam, r, lo, hi);
    dl->AddRectFilled(lo, hi, c.packed());
}

void centred_text(ImDrawList* dl, ImVec2 centre, float size, Rgba c, const char* s) {
    ImGui::PushFont(nullptr, size);
    const ImVec2 ts = ImGui::CalcTextSize(s);
    ImGui::PopFont();
    dl->AddText(ImGui::GetFont(), size, {centre.x - ts.x / 2.0f, centre.y - ts.y / 2.0f}, c.packed(), s);
}

bool near(Rgba a, Rgba b, int tol) {
    return std::abs(a.r - b.r) <= tol && std::abs(a.g - b.g) <= tol && std::abs(a.b - b.b) <= tol;
}

}  // namespace

void draw_maze(ImDrawList* dl, const Camera& cam, const core::Maze& maze, const core::FloodMap* flood,
               const core::MazeDims& dims, const MazeViewOptions& opt) {
    fill(dl, cam, core::maze_bounds(maze, dims), colors::kFloor);

    const double cell_px = dims.cell * cam.pixels_per_metre();
    if (opt.show_start_goal) {
        for (Cell g : maze.goals())
            fill(dl, cam, {g.x * dims.cell, g.y * dims.cell, (g.x + 1) * dims.cell, (g.y + 1) * dims.cell},
                 colors::kGoal);
        const Cell s = maze.start();
        fill(dl, cam, {s.x * dims.cell, s.y * dims.cell, (s.x + 1) * dims.cell, (s.y + 1) * dims.cell},
             colors::kStart);
    }

    if (opt.show_grid && cell_px >= 12.0) {
        for (int i = 1; i < maze.width(); ++i)
            dl->AddLine(to_im(cam.world_to_screen({i * dims.cell, 0.0})),
                        to_im(cam.world_to_screen({i * dims.cell, maze.height() * dims.cell})),
                        colors::kGrid.packed());
        for (int j = 1; j < maze.height(); ++j)
            dl->AddLine(to_im(cam.world_to_screen({0.0, j * dims.cell})),
                        to_im(cam.world_to_screen({maze.width() * dims.cell, j * dims.cell})),
                        colors::kGrid.packed());
    }

    if (opt.show_distances && flood && cell_px >= 18.0) {
        const float size = static_cast<float>(std::clamp(cell_px * 0.32, 8.0, 28.0));
        char buf[16];
        for (int y = 0; y < maze.height(); ++y)
            for (int x = 0; x < maze.width(); ++x) {
                const int d = flood->at(x, y);
                std::snprintf(buf, sizeof buf, "%s", d < 0 ? "-" : std::to_string(d).c_str());
                const auto c = core::cell_center({x, y}, dims);
                centred_text(dl, to_im(cam.world_to_screen({c.x, c.y})), size, colors::kText, buf);
            }
    }

    for (const auto& w : core::wall_rects(maze, dims)) fill(dl, cam, w.rect, colors::kWall);
    for (const auto& p : core::post_rects(maze, dims)) fill(dl, cam, p, colors::kPost);

    if (opt.show_coords && cell_px >= 14.0) {
        const float size = static_cast<float>(std::clamp(cell_px * 0.28, 8.0, 18.0));
        char buf[8];
        for (int x = 0; x < maze.width(); ++x) {
            std::snprintf(buf, sizeof buf, "%d", x);
            centred_text(dl, to_im(cam.world_to_screen({(x + 0.5) * dims.cell, -0.45 * dims.cell})), size,
                         colors::kText, buf);
        }
        for (int y = 0; y < maze.height(); ++y) {
            std::snprintf(buf, sizeof buf, "%d", y);
            centred_text(dl, to_im(cam.world_to_screen({-0.45 * dims.cell, (y + 0.5) * dims.cell})), size,
                         colors::kText, buf);
        }
    }
}

Rgba Image::at(int x, int y) const {
    const auto i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)) * 4;
    return {rgba[i], rgba[i + 1], rgba[i + 2], rgba[i + 3]};
}

RenderCheck check_maze_pixels(const Image& img, const Camera& cam, const core::Maze& maze,
                              const core::MazeDims& dims) {
    RenderCheck rc;
    const double wall_px = dims.wall * cam.pixels_per_metre();
    if (wall_px < 2.0) {
        rc.failures.push_back("walls are " + std::to_string(wall_px) + " px thick; need >= 2 px to verify");
        return rc;
    }
    // Samples a strip of pixels across the rectangle's thin axis, through its centre. A present
    // feature needs >= 1 pixel of its colour in the strip; an absent wall needs none. (Sampling a
    // strip instead of one pixel tolerates rasterizers that leave a seam on a triangle diagonal,
    // e.g. SDL's software renderer, without accepting a missing wall.)
    auto probe = [&](const Rect& r, bool want_present, Rgba expect, const std::string& what) {
        const Vec2 a = cam.world_to_screen({r.x0, r.y1});
        const Vec2 b = cam.world_to_screen({r.x1, r.y0});
        const int cx = static_cast<int>(std::floor((a.x + b.x) / 2.0));
        const int cy = static_cast<int>(std::floor((a.y + b.y) / 2.0));
        const bool vertical_strip = (b.x - a.x) >= (b.y - a.y);  // wide rect -> sample a column
        const int lo = static_cast<int>(std::floor(vertical_strip ? a.y : a.x));
        const int hi = static_cast<int>(std::ceil(vertical_strip ? b.y : b.x)) - 1;
        if (cx < 0 || cy < 0 || cx >= img.width || cy >= img.height || lo < 0 ||
            hi >= (vertical_strip ? img.height : img.width)) {
            rc.failures.push_back(what + ": off-screen");
            return;
        }
        ++rc.checked;
        int hits = 0;
        for (int k = lo; k <= hi; ++k)
            if (near(vertical_strip ? img.at(cx, k) : img.at(k, cy), expect, 8)) ++hits;
        if ((hits > 0) != want_present) {
            const Rgba got = img.at(cx, cy);
            char buf[200];
            std::snprintf(buf, sizeof buf, "%s at px(%d,%d): %d/%d strip pixels match rgb(%d,%d,%d) (%s); centre rgb(%d,%d,%d)",
                          what.c_str(), cx, cy, hits, hi - lo + 1, expect.r, expect.g, expect.b,
                          want_present ? "expected >= 1" : "expected 0", got.r, got.g, got.b);
            rc.failures.push_back(buf);
        }
    };
    for (int y = 0; y < maze.height(); ++y)
        for (int x = 0; x < maze.width(); ++x)
            for (Dir d : {Dir::N, Dir::E}) {
                const bool present = maze.has_wall(x, y, d);
                probe(core::wall_slot({x, y}, d, dims), present, colors::kWall,
                      "wall (" + std::to_string(x) + "," + std::to_string(y) + ")" + (d == Dir::N ? "N" : "E"));
            }
    for (const auto& p : core::post_rects(maze, dims)) probe(p, true, colors::kPost, "post");
    return rc;
}

}  // namespace mmsim::app
