// The maze canvas shared by Cockpit (live), Maze Studio (overlays, 2.5D, cell picking) and
// Telemetry & Compare (ghost replay of the archived run B).
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "sim/app/panels.hpp"
#include "sim/app/theme.hpp"
#include "sim/app/widgets.hpp"

namespace mmsim::app {

namespace an = core::analysis;

namespace {

constexpr double kPi = 3.14159265358979323846;

ImVec2 to_im(Vec2 v) {
    return {static_cast<float>(v.x), static_cast<float>(v.y)};
}

Rgba heat_colour(float f) {
    // Sequential, dark-bg safe: deep teal -> accent -> warn -> bad.
    f = std::clamp(f, 0.0f, 1.0f);
    const Rgba a{20, 64, 84, 255};
    if (f < 0.4f) return mix(a, tok::accent, f / 0.4f);
    if (f < 0.75f) return mix(tok::accent, tok::warn, (f - 0.4f) / 0.35f);
    return mix(tok::warn, tok::bad, (f - 0.75f) / 0.25f);
}

/// Value 0..1 of the selected heat layer in cell (x, y); <0 = nothing to show.
float heat_value(AppState& st, int x, int y) {
    if (st.heat == HeatLayer::Off) return -1.0f;
    if (st.heat == HeatLayer::Potential) {
        const auto& p = studio_plan(st);
        if (!st.maze || p.potential.cost.empty() || p.potential_max <= 0.0) return -1.0f;
        const double c = p.potential.cost[static_cast<std::size_t>(y * st.maze->width() + x)];
        if (c < 0) return -1.0f;
        return static_cast<float>(1.0 - c / p.potential_max);  // hot = close to the goal
    }
    const auto& h = heat_cache(st);
    const an::HeatCell* c = heat_at(h, x, y);
    if (!c) return -1.0f;
    if (st.heat == HeatLayer::PeakSpeed)
        return h.v_max > 0 ? static_cast<float>(c->v_max / h.v_max) : -1.0f;
    const double limit = st.mu * an::kGmms2;  // grip use: lateral demand vs mu g
    return static_cast<float>(std::hypot(c->a_lat_max, c->a_long_max) / limit);
}

void draw_heat(AppState& st, ImDrawList* dl, const Camera& cam) {
    if (!st.maze || st.heat == HeatLayer::Off) return;
    const auto& m = *st.maze;
    const double c = st.dims.cell, inset = st.dims.wall * 0.9;
    for (int y = 0; y < m.height(); ++y)
        for (int x = 0; x < m.width(); ++x) {
            const float f = heat_value(st, x, y);
            if (f < 0) continue;
            const ImVec2 a = to_im(cam.world_to_screen({x * c + inset, (y + 1) * c - inset}));
            const ImVec2 b = to_im(cam.world_to_screen({(x + 1) * c - inset, y * c + inset}));
            dl->AddRectFilled(a, b, theme::u32(heat_colour(f), 0.55f), 2.0f);
        }
}

/// Edge midpoint between consecutive route cells (the polyline cuts corners, so staircase
/// chains read as diagonals).
Vec2 route_point(const std::vector<core::Cell>& cells, std::size_t i, double c) {
    if (i == 0 || i + 1 >= cells.size() + 1) {
        const auto& k = cells[std::min(i, cells.size() - 1)];
        return {(k.x + 0.5) * c, (k.y + 0.5) * c};
    }
    const auto& p = cells[i - 1];
    const auto& q = cells[i];
    return {((p.x + q.x) * 0.5 + 0.5) * c, ((p.y + q.y) * 0.5 + 0.5) * c};
}

void draw_route(AppState& st, ImDrawList* dl, const Camera& cam) {
    if (!st.show_route) return;
    const auto& plan = studio_plan(st);
    if (!plan.path || plan.path->cells.size() < 2) return;
    const auto& cells = plan.path->cells;
    const double c = st.dims.cell;
    const float w = std::max(2.0f, static_cast<float>(cam.pixels_per_metre() * c * 0.06));
    // Points: start centre, every edge midpoint, goal centre.
    std::vector<Vec2> pts;
    pts.push_back({(cells.front().x + 0.5) * c, (cells.front().y + 0.5) * c});
    for (std::size_t i = 1; i < cells.size(); ++i) pts.push_back(route_point(cells, i, c));
    pts.push_back({(cells.back().x + 0.5) * c, (cells.back().y + 0.5) * c});
    // Segment k (pts[k] -> pts[k+1]) lies in cell k; diagonal if both moves around it are.
    for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
        bool diag = false;
        if (k >= 1 && k < plan.diagonal_move.size() + 1)
            diag = plan.diagonal_move[k - 1] &&
                   (k < plan.diagonal_move.size() ? plan.diagonal_move[k] : false);
        const Rgba col = diag ? tok::warn : tok::accent;
        dl->AddLine(to_im(cam.world_to_screen(pts[k])), to_im(cam.world_to_screen(pts[k + 1])),
                    theme::u32(col, 0.9f), w);
    }
    for (const auto& p : {pts.front(), pts.back()})
        dl->AddCircleFilled(to_im(cam.world_to_screen(p)), w * 1.6f, theme::u32(tok::text));
}

void draw_cell_marker(const AppState& st, ImDrawList* dl, const Camera& cam, core::Cell k, Rgba col,
                      float thick) {
    const double c = st.dims.cell;
    const ImVec2 a = to_im(cam.world_to_screen({k.x * c, (k.y + 1) * c}));
    const ImVec2 b = to_im(cam.world_to_screen({(k.x + 1) * c, k.y * c}));
    dl->AddRect(a, b, theme::u32(col), 3.0f, 0, thick);
}

// --- 2.5D isometric view (Studio) -------------------------------------------------------------
struct Iso {
    ImVec2 origin;
    float s = 10.0f;  // pixels per metre
    ImVec2 at(double x, double y, double z) const {
        const float cx = static_cast<float>((x - y) * 0.8660254);
        const float cy = static_cast<float>((x + y) * 0.5);
        return {origin.x + cx * s, origin.y - cy * s - static_cast<float>(z) * s};
    }
};

void draw_iso(AppState& st, ImDrawList* dl, ImVec2 p0, ImVec2 size) {
    if (!st.maze) return;
    const auto& m = *st.maze;
    const double c = st.dims.cell, t = st.dims.wall * 0.5, h = 0.05;  // 50 mm walls
    const double W = m.width() * c, H = m.height() * c;
    // Fit: projected extents x in [-H*cos30, W*cos30], y in [0, (W+H)/2 + h].
    const double ex = (W + H) * 0.8660254, ey = (W + H) * 0.5 + h;
    Iso iso;
    iso.s = static_cast<float>(std::min(size.x / ex, size.y / ey) * 0.92);
    iso.origin = {p0.x + size.x * 0.5f - static_cast<float>((W - H) * 0.8660254 * 0.5) * iso.s,
                  p0.y + size.y * 0.5f + static_cast<float>(ey * 0.5) * iso.s};
    auto quad = [&](ImVec2 a, ImVec2 b, ImVec2 cc, ImVec2 d, ImU32 col) {
        dl->AddQuadFilled(a, b, cc, d, col);
    };
    // Floor cells (with heat), far (high x+y) first.
    for (int s = m.width() + m.height() - 2; s >= 0; --s)
        for (int x = 0; x < m.width(); ++x) {
            const int y = s - x;
            if (y < 0 || y >= m.height()) continue;
            const float f = heat_value(st, x, y);
            Rgba col = colors::kFloor;
            if (m.is_goal({x, y})) col = colors::kGoal;
            if (x == m.start().x && y == m.start().y) col = colors::kStart;
            if (f >= 0) col = mix(col, heat_colour(f), 0.55f);
            quad(iso.at(x * c, y * c, 0), iso.at((x + 1) * c, y * c, 0),
                 iso.at((x + 1) * c, (y + 1) * c, 0), iso.at(x * c, (y + 1) * c, 0),
                 theme::u32(col));
        }
    // Route on the floor.
    const auto& plan = studio_plan(st);
    if (st.show_route && plan.path && plan.path->cells.size() >= 2) {
        const auto& cells = plan.path->cells;
        for (std::size_t i = 0; i + 1 < cells.size(); ++i) {
            const bool diag = i < plan.diagonal_move.size() && plan.diagonal_move[i];
            dl->AddLine(iso.at((cells[i].x + 0.5) * c, (cells[i].y + 0.5) * c, 0.001),
                        iso.at((cells[i + 1].x + 0.5) * c, (cells[i + 1].y + 0.5) * c, 0.001),
                        theme::u32(diag ? tok::warn : tok::accent, 0.95f), 2.5f);
        }
    }
    // Walls: south and west edges of every cell + the outer north/east, painter's order.
    struct Wall {
        double x0, y0, x1, y1;
        double depth;
    };
    std::vector<Wall> walls;
    for (int y = 0; y < m.height(); ++y)
        for (int x = 0; x < m.width(); ++x) {
            if (m.has_wall(x, y, core::Dir::S))
                walls.push_back({x * c, y * c - t, (x + 1) * c, y * c + t, 0});
            if (m.has_wall(x, y, core::Dir::W))
                walls.push_back({x * c - t, y * c, x * c + t, (y + 1) * c, 0});
            if (y == m.height() - 1 && m.has_wall(x, y, core::Dir::N))
                walls.push_back({x * c, H - t, (x + 1) * c, H + t, 0});
            if (x == m.width() - 1 && m.has_wall(x, y, core::Dir::E))
                walls.push_back({W - t, y * c, W + t, (y + 1) * c, 0});
        }
    for (auto& w : walls) w.depth = w.x0 + w.x1 + w.y0 + w.y1;
    std::sort(walls.begin(), walls.end(),
              [](const Wall& a, const Wall& b) { return a.depth > b.depth; });
    const ImU32 top = theme::u32(colors::kWall),
                left = theme::u32(mix(colors::kWall, tok::canvas, 0.45f)),
                right = theme::u32(mix(colors::kWall, tok::canvas, 0.30f));
    for (const auto& w : walls) {
        // Visible faces from the viewer (looking towards +x +y): the y0 face and the x0 face.
        quad(iso.at(w.x0, w.y0, 0), iso.at(w.x1, w.y0, 0), iso.at(w.x1, w.y0, h),
             iso.at(w.x0, w.y0, h), right);
        quad(iso.at(w.x0, w.y0, 0), iso.at(w.x0, w.y1, 0), iso.at(w.x0, w.y1, h),
             iso.at(w.x0, w.y0, h), left);
        quad(iso.at(w.x0, w.y0, h), iso.at(w.x1, w.y0, h), iso.at(w.x1, w.y1, h),
             iso.at(w.x0, w.y1, h), top);
    }
    // Robot marker.
    if (st.session) {
        const auto& p = st.session->world().drive().pose;
        const ImVec2 q = iso.at(p.x, p.y, 0.02);
        dl->AddCircleFilled(q, std::max(4.0f, iso.s * 0.03f), theme::u32(tok::accent));
        dl->AddCircle(q, std::max(4.0f, iso.s * 0.03f) + 2.0f, theme::u32(tok::text), 0, 1.5f);
    }
}

/// Small floating chip row inside the canvas (top-right): Fit, and the 2D / 2.5D switch.
void canvas_chips(AppState& st, ImVec2 origin, ImVec2 size, CanvasKind kind, int ws) {
    const ImGuiStyle& s = ImGui::GetStyle();
    float w = ui::button_width("Fit");
    if (kind == CanvasKind::Studio)
        w += ui::button_width("2D") + ui::button_width("2.5D") + s.ItemSpacing.x * 2;
    ImGui::SetCursorScreenPos({origin.x + size.x - w - 12.0f, origin.y + 8.0f});
    if (kind == CanvasKind::Studio) {
        if (ui::pill_toggle("2D", !st.iso, "Top-down view")) st.iso = false;
        ImGui::SameLine();
        if (ui::pill_toggle("2.5D", st.iso, "Isometric view (walls extruded)")) st.iso = true;
        ImGui::SameLine();
    }
    if (ui::pill_toggle("Fit", false, "Fit the maze (F)")) {
        st.need_fit[ws] = true;
        st.user_moved[ws] = false;
    }
}

void draw_ghost_b(AppState& st, ImDrawList* dl, const Camera& cam) {
    if (!st.run_b || !st.session) return;
    const auto& r = *st.run_b;
    const auto *t = r.column("t"), *x = r.column("sim.x"), *y = r.column("sim.y"),
               *th = r.column("sim.theta");
    if (!t || !x || !y || !th || t->empty()) return;
    const double now =
        st.tv.scrub ? st.tv.cursor_t : static_cast<double>(st.session->time_us()) * 1e-6;
    // Trace of B (thin, warm) and the ghost at the same simulated time.
    const std::size_t stride = std::max<std::size_t>(1, t->size() / 3000);
    for (std::size_t i = stride; i < t->size(); i += stride)
        dl->AddLine(to_im(cam.world_to_screen({(*x)[i - stride], (*y)[i - stride]})),
                    to_im(cam.world_to_screen({(*x)[i], (*y)[i]})), theme::u32(tok::warn, 0.55f),
                    1.5f);
    auto it = std::upper_bound(t->begin(), t->end(), static_cast<float>(now));
    const std::size_t i = it == t->begin() ? 0 : static_cast<std::size_t>(it - t->begin() - 1);
    draw_ghost(dl, cam, st.session->world().config(), {(*x)[i], (*y)[i], (*th)[i] * kPi / 180.0},
               tok::warn);
}

}  // namespace

void panel_maze(AppState& st, const char* window, CanvasKind kind) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    const bool open = ImGui::Begin(
        window, nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    if (!open) {
        ImGui::End();
        return;
    }
    const int ws = static_cast<int>(st.workspace);
    Camera& cam = st.cams[ws];
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.x = std::max(avail.x, 50.0f);
    avail.y = std::max(avail.y, 50.0f);
    ImGui::InvisibleButton("canvas", avail,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonMiddle |
                               ImGuiButtonFlags_MouseButtonRight);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    const bool clicked =
        ImGui::IsItemDeactivated() && ImGui::GetIO().MouseDragMaxDistanceSqr[0] < 16.0f;
    ImGuiIO& io = ImGui::GetIO();
    const bool iso = kind == CanvasKind::Studio && st.iso;

    // The chip row (Fit, 2D/2.5D, legends) owns the top strip: the maze is fitted below it.
    const float chips_h = st.maze ? ImGui::GetFrameHeight() + 14.0f : 0.0f;
    cam.set_viewport({origin.x, origin.y + chips_h}, {avail.x, std::max(20.0f, avail.y - chips_h)});
    if (st.maze && (st.need_fit[ws] || !st.user_moved[ws])) {
        const auto b = core::maze_bounds(*st.maze, st.dims);
        const double pad = st.dims.cell * 0.9;  // room for coordinate labels
        cam.fit({b.x0 - pad, b.y0 - pad}, {b.x1 + pad * 0.3, b.y1 + pad * 0.3}, 0.02);
        st.need_fit[ws] = false;
    }
    if (!iso) {
        if (active && (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f)) {
            cam.pan_pixels({io.MouseDelta.x, io.MouseDelta.y});
            st.user_moved[ws] = true;
        }
        if (hovered && io.MouseWheel != 0.0f) {
            cam.zoom_at({io.MousePos.x, io.MousePos.y}, std::pow(1.15, io.MouseWheel));
            st.user_moved[ws] = true;
        }
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(origin, {origin.x + avail.x, origin.y + avail.y}, true);
    dl->AddRectFilled(origin, {origin.x + avail.x, origin.y + avail.y},
                      colors::kBackground.packed());
    if (!st.maze) {
        const char* msg = "Pick a maze from the maze menu in the top bar.";
        const ImVec2 ts = ImGui::CalcTextSize(msg);
        dl->AddText({origin.x + (avail.x - ts.x) * 0.5f, origin.y + (avail.y - ts.y) * 0.5f},
                    theme::u32(tok::muted), msg);
    } else if (iso) {
        draw_iso(st, dl, origin, avail);
    } else {
        MazeViewOptions vo = st.view;
        if (kind == CanvasKind::Studio && st.heat != HeatLayer::Off)
            vo.show_fw_colors = false;  // heat owns the fills
        draw_maze(dl, cam, *st.maze, st.flood ? &*st.flood : nullptr, st.dims, vo,
                  st.session ? &st.session->annotations() : nullptr);
        if (kind == CanvasKind::Studio) {
            draw_heat(st, dl, cam);
            draw_route(st, dl, cam);
        }
        if (st.session) {
            RobotViewOptions ro = st.robot_view;
            if (kind == CanvasKind::Studio) ro.show_beams = false;
            draw_robot(dl, cam, st.session->world(), st.trace, ro);
            if (st.tv.scrub) {
                const auto& r = st.session->recorder();
                const auto *x = r.channel("sim.x"), *y = r.channel("sim.y"),
                           *th = r.channel("sim.theta");
                if (x && y && th) {
                    const double t = st.tv.cursor_t;
                    const core::Pose g{x->value_at(t), y->value_at(t),
                                       th->value_at(t) * kPi / 180.0};
                    if (!std::isnan(g.x)) draw_ghost(dl, cam, st.session->world().config(), g);
                }
            }
            if (kind == CanvasKind::Ghost) draw_ghost_b(st, dl, cam);
        }
        if (kind == CanvasKind::Studio) {
            // Cell picking.
            st.hover.reset();
            if (hovered) {
                const Vec2 wpos = cam.screen_to_world({io.MousePos.x, io.MousePos.y});
                const core::Cell k{static_cast<int>(std::floor(wpos.x / st.dims.cell)),
                                   static_cast<int>(std::floor(wpos.y / st.dims.cell))};
                if (k.x >= 0 && k.y >= 0 && k.x < st.maze->width() && k.y < st.maze->height()) {
                    st.hover = k;
                    if (clicked) st.inspect = k;
                }
            }
            if (st.hover) draw_cell_marker(st, dl, cam, *st.hover, tok::muted, 1.5f);
            if (st.inspect) draw_cell_marker(st, dl, cam, *st.inspect, tok::accent, 2.5f);
        }
    }
    dl->PopClipRect();

    // Legends / overlays inside the canvas.
    if (st.maze) {
        canvas_chips(st, origin, avail, kind, ws);
        if (kind == CanvasKind::Ghost) {
            ImGui::SetCursorScreenPos({origin.x + 12.0f, origin.y + 8.0f});
            ui::pill(st.run_b ? "Ghost: archived run B (amber)"
                              : "No run B: save or pick one in Run compare",
                     st.run_b ? ui::Tone::Warn : ui::Tone::Neutral, false);
        } else if (kind == CanvasKind::Studio && st.show_route && !iso) {
            ImGui::SetCursorScreenPos({origin.x + 12.0f, origin.y + 8.0f});
            ui::pill("Orthogonal", ui::Tone::Active, false);
            ImGui::SameLine();
            ui::pill("Diagonal", ui::Tone::Warn, false);
        }
    }
    ImGui::End();
}

}  // namespace mmsim::app
