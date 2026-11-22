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

// --- 2.5D isometric view (Cockpit & Studio) --------------------------------------------------
struct Iso {
    ImVec2 origin;
    float s = 10.0f;  // pixels per metre
    ImVec2 at(double x, double y, double z) const {
        const float cx = static_cast<float>((x - y) * 0.8660254);
        const float cy = static_cast<float>((x + y) * 0.5);
        return {origin.x + cx * s, origin.y - cy * s - static_cast<float>(z) * s};
    }
};

void draw_iso(AppState& st, ImDrawList* dl, ImVec2 p0, ImVec2 size, int ws) {
    if (!st.maze) return;
    const auto& m = *st.maze;
    const double c = st.dims.cell, t = st.dims.wall * 0.5, h = 0.05;  // 50 mm walls
    const double W = m.width() * c, H = m.height() * c;
    const double ex = (W + H) * 0.8660254, ey = (W + H) * 0.5 + h;

    const float zoom = ws >= 0 && ws < ui::kWorkspaceCount ? st.iso_zoom[ws] : 1.0f;
    const ImVec2 pan = ws >= 0 && ws < ui::kWorkspaceCount
                           ? ImVec2{static_cast<float>(st.iso_pan[ws].x),
                                    static_cast<float>(st.iso_pan[ws].y)}
                           : ImVec2{0.0f, 0.0f};

    Iso iso;
    iso.s = static_cast<float>(std::min(size.x / ex, size.y / ey) * 0.92) * zoom;
    iso.origin = {p0.x + size.x * 0.5f - static_cast<float>((W - H) * 0.8660254 * 0.5) * iso.s + pan.x,
                  p0.y + size.y * 0.5f + static_cast<float>(ey * 0.5) * iso.s + pan.y};

    auto quad = [&](ImVec2 a, ImVec2 b, ImVec2 cc, ImVec2 d, ImU32 col) {
        dl->AddQuadFilled(a, b, cc, d, col);
    };

    // 1. Floor cells (with heat), far (high x+y) to near (low x+y).
    for (int s = m.width() + m.height() - 2; s >= 0; --s) {
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
    }

    // 2. Route on the floor.
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

    // 3. Path trace on the floor.
    if (st.robot_view.show_trace && !st.trace.points().empty()) {
        const auto& pts = st.trace.points();
        const std::size_t stride = std::max<std::size_t>(1, pts.size() / 2000);
        for (std::size_t i = stride; i < pts.size(); i += stride) {
            dl->AddLine(iso.at(pts[i - stride].x, pts[i - stride].y, 0.001),
                        iso.at(pts[i].x, pts[i].y, 0.001),
                        theme::u32(tok::accent, 0.65f), 1.8f);
        }
    }

    // 4. Unified 3D Objects: Walls, Posts, and Robot.
    enum class ObjType { Wall, Post, Robot };
    struct IsoObj {
        ObjType type;
        double depth;
        double x0, y0, x1, y1;
        double px, py;
    };
    std::vector<IsoObj> objs;
    objs.reserve(m.width() * m.height() * 2 + (m.width() + 1) * (m.height() + 1) + 4);

    // Walls
    for (int y = 0; y < m.height(); ++y) {
        for (int x = 0; x < m.width(); ++x) {
            if (m.has_wall(x, y, core::Dir::S)) {
                IsoObj o{};
                o.type = ObjType::Wall;
                o.x0 = x * c; o.y0 = y * c - t; o.x1 = (x + 1) * c; o.y1 = y * c + t;
                o.depth = (o.x0 + o.x1 + o.y0 + o.y1) * 0.5;
                objs.push_back(o);
            }
            if (m.has_wall(x, y, core::Dir::W)) {
                IsoObj o{};
                o.type = ObjType::Wall;
                o.x0 = x * c - t; o.y0 = y * c; o.x1 = x * c + t; o.y1 = (y + 1) * c;
                o.depth = (o.x0 + o.x1 + o.y0 + o.y1) * 0.5;
                objs.push_back(o);
            }
            if (y == m.height() - 1 && m.has_wall(x, y, core::Dir::N)) {
                IsoObj o{};
                o.type = ObjType::Wall;
                o.x0 = x * c; o.y0 = H - t; o.x1 = (x + 1) * c; o.y1 = H + t;
                o.depth = (o.x0 + o.x1 + o.y0 + o.y1) * 0.5;
                objs.push_back(o);
            }
            if (x == m.width() - 1 && m.has_wall(x, y, core::Dir::E)) {
                IsoObj o{};
                o.type = ObjType::Wall;
                o.x0 = W - t; o.y0 = y * c; o.x1 = W + t; o.y1 = (y + 1) * c;
                o.depth = (o.x0 + o.x1 + o.y0 + o.y1) * 0.5;
                objs.push_back(o);
            }
        }
    }

    // Corner posts (pillars) at grid intersections where walls meet
    for (int py = 0; py <= m.height(); ++py) {
        for (int px = 0; px <= m.width(); ++px) {
            IsoObj o{};
            o.type = ObjType::Post;
            o.px = px * c; o.py = py * c;
            o.depth = o.px + o.py;
            objs.push_back(o);
        }
    }

    // Robot in 3D
    if (st.session) {
        const auto& p = st.session->world().drive().pose;
        IsoObj o{};
        o.type = ObjType::Robot;
        o.depth = p.x + p.y;
        objs.push_back(o);
    }

    // Sort descending by depth: farthest (large depth) first, nearest (small depth) last
    std::sort(objs.begin(), objs.end(), [](const IsoObj& a, const IsoObj& b) {
        if (std::abs(a.depth - b.depth) > 1e-6) return a.depth > b.depth;
        return static_cast<int>(a.type) < static_cast<int>(b.type);
    });

    const ImU32 top = theme::u32(colors::kWall),
                left = theme::u32(mix(colors::kWall, tok::canvas, 0.45f)),
                right = theme::u32(mix(colors::kWall, tok::canvas, 0.30f));
    const ImU32 post_top = theme::u32(colors::kPost),
                post_left = theme::u32(mix(colors::kPost, tok::canvas, 0.45f)),
                post_right = theme::u32(mix(colors::kPost, tok::canvas, 0.30f));

    for (const auto& o : objs) {
        if (o.type == ObjType::Wall) {
            quad(iso.at(o.x0, o.y0, 0), iso.at(o.x1, o.y0, 0), iso.at(o.x1, o.y0, h),
                 iso.at(o.x0, o.y0, h), right);
            quad(iso.at(o.x0, o.y0, 0), iso.at(o.x0, o.y1, 0), iso.at(o.x0, o.y1, h),
                 iso.at(o.x0, o.y0, h), left);
            quad(iso.at(o.x0, o.y0, h), iso.at(o.x1, o.y0, h), iso.at(o.x1, o.y1, h),
                 iso.at(o.x0, o.y1, h), top);
        } else if (o.type == ObjType::Post) {
            const double px = o.px, py = o.py;
            quad(iso.at(px - t, py - t, 0), iso.at(px + t, py - t, 0), iso.at(px + t, py - t, h),
                 iso.at(px - t, py - t, h), post_right);
            quad(iso.at(px - t, py - t, 0), iso.at(px - t, py + t, 0), iso.at(px - t, py + t, h),
                 iso.at(px - t, py - t, h), post_left);
            quad(iso.at(px - t, py - t, h), iso.at(px + t, py - t, h), iso.at(px + t, py + t, h),
                 iso.at(px - t, py + t, h), post_top);
        } else if (o.type == ObjType::Robot && st.session) {
            const auto& p = st.session->world().drive().pose;
            const double L = st.robot ? st.robot->body.length : 0.080;
            const double W = st.robot ? st.robot->body.width : 0.065;
            const double x_off = st.robot ? st.robot->body.offset_x : 0.010;
            const double hr = 0.022; // 22 mm chassis height
            const double ct = std::cos(p.theta), sth = std::sin(p.theta);
            auto rrot = [&](double lx, double ly, double lz) -> ImVec2 {
                const double wx = p.x + lx * ct - ly * sth;
                const double wy = p.y + lx * sth + ly * ct;
                return iso.at(wx, wy, lz);
            };

            // 3D Directional Chassis Wedge
            const double fx = L * 0.50 + x_off, rx = -L * 0.45 + x_off, hy = W * 0.50;
            const ImVec2 b0 = rrot(fx, 0, 0.003), b1 = rrot(fx * 0.4, hy, 0.003),
                         b2 = rrot(rx, hy, 0.003), b3 = rrot(rx, -hy, 0.003),
                         b4 = rrot(fx * 0.4, -hy, 0.003);
            const ImVec2 t0 = rrot(fx, 0, hr), t1 = rrot(fx * 0.4, hy, hr),
                         t2 = rrot(rx, hy, hr), t3 = rrot(rx, -hy, hr),
                         t4 = rrot(fx * 0.4, -hy, hr);

            // Shadow on floor
            dl->AddQuadFilled(b1, b2, b3, b4, theme::u32(tok::canvas, 0.6f));

            // Extruded Side Panels
            const ImU32 rside1 = theme::u32(mix(tok::accent, tok::canvas, 0.35f));
            const ImU32 rside2 = theme::u32(mix(tok::accent, tok::canvas, 0.50f));
            quad(b3, b2, t2, t3, rside2); // Rear face
            quad(b2, b1, t1, t2, rside1); // Left face
            quad(b4, b3, t3, t4, rside2); // Right face
            quad(b1, b0, t0, t1, rside1); // Front-left face
            quad(b0, b4, t4, t0, rside2); // Front-right face

            // Top Face (Vivid Accent)
            const ImU32 rtop = theme::u32(tok::accent);
            dl->AddQuadFilled(t1, t2, t3, t4, rtop);
            dl->AddTriangleFilled(t0, t1, t4, rtop);
            dl->AddQuad(t1, t2, t3, t4, theme::u32(tok::text), 0, 1.2f);
            dl->AddTriangle(t0, t1, t4, theme::u32(tok::text), 1.2f);

            // Distinct Heading Arrow on Top
            const ImVec2 a_tip = rrot(fx * 0.85, 0, hr + 0.001);
            const ImVec2 a_l = rrot(x_off + 0.005, hy * 0.40, hr + 0.001);
            const ImVec2 a_r = rrot(x_off + 0.005, -hy * 0.40, hr + 0.001);
            dl->AddTriangleFilled(a_tip, a_l, a_r, theme::u32(tok::text, 0.95f));

            // Left & Right Wheels (Dark rubber with rim)
            const double wy = hy * 0.95, wx0 = x_off - L * 0.15, wx1 = x_off + L * 0.15, wh = 0.024;
            const ImU32 wcol = theme::u32(tok::panel);
            quad(rrot(wx0, wy, 0), rrot(wx1, wy, 0), rrot(wx1, wy, wh), rrot(wx0, wy, wh), wcol);
            quad(rrot(wx0, -wy, 0), rrot(wx1, -wy, 0), rrot(wx1, -wy, wh), rrot(wx0, -wy, wh), wcol);

            // Distance Sensor Beams in 3D (if enabled)
            if (st.robot_view.show_beams) {
                const auto& wld = st.session->world();
                const auto& ss = wld.sensors();
                for (std::size_t ch = 0; ch < ss.size(); ++ch) {
                    if (!wld.emitter(static_cast<int>(ch))) continue;
                    const auto& sc = ss.config(ch);
                    const auto mm = ss.distance_mm(ch);
                    const double dist = (mm == core::kDistanceNone) ? sc.range_max : (mm * 1e-3);
                    const double sa = (p.theta * 180.0 / 3.14159265358979323846 + sc.angle_deg) * (3.14159265358979323846 / 180.0);
                    const ImVec2 src = rrot(sc.x, sc.y, 0.012);
                    const ImVec2 dst = iso.at(p.x + sc.x * ct - sc.y * sth + std::cos(sa) * dist,
                                              p.y + sc.x * sth + sc.y * ct + std::sin(sa) * dist, 0.012);
                    dl->AddLine(src, dst, theme::u32(tok::accent, 0.45f), 1.2f);
                    dl->AddCircleFilled(dst, 2.5f, theme::u32(mm == core::kDistanceNone ? tok::muted : tok::warn));
                }
            }
        }
    }
}

/// Small floating chip row inside the canvas (top-right): Fit, and the 2D / 2.5D switch.
void canvas_chips(AppState& st, ImVec2 origin, ImVec2 size, CanvasKind kind, int ws) {
    const ImGuiStyle& s = ImGui::GetStyle();
    const bool can_iso = (kind == CanvasKind::Cockpit || kind == CanvasKind::Studio);
    float w = ui::button_width("Fit");
    if (can_iso)
        w += ui::button_width("2D") + ui::button_width("2.5D") + s.ItemSpacing.x * 2;
    ImGui::SetCursorScreenPos({origin.x + size.x - w - 12.0f, origin.y + 8.0f});
    if (can_iso) {
        if (ui::pill_toggle("2D", !st.iso, "Top-down 2D view")) st.iso = false;
        ImGui::SameLine();
        if (ui::pill_toggle("2.5D", st.iso, "Isometric 3D view (walls extruded, I to toggle)")) st.iso = true;
        ImGui::SameLine();
    }
    if (ui::pill_toggle("Fit", false, "Fit the maze (F)")) {
        st.need_fit[ws] = true;
        st.user_moved[ws] = false;
        if (ws >= 0 && ws < ui::kWorkspaceCount) {
            st.iso_pan[ws] = {0.0f, 0.0f};
            st.iso_zoom[ws] = 1.0f;
        }
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
    const bool can_iso = (kind == CanvasKind::Cockpit || kind == CanvasKind::Studio);
    const bool iso = can_iso && st.iso;

    // The chip row (Fit, 2D/2.5D, legends) owns the top strip: the maze is fitted below it.
    const float chips_h = st.maze ? ImGui::GetFrameHeight() + 14.0f : 0.0f;
    cam.set_viewport({origin.x, origin.y + chips_h}, {avail.x, std::max(20.0f, avail.y - chips_h)});
    if (st.maze && (st.need_fit[ws] || !st.user_moved[ws])) {
        const auto b = core::maze_bounds(*st.maze, st.dims);
        const double pad = st.dims.cell * 0.9;  // room for coordinate labels
        cam.fit({b.x0 - pad, b.y0 - pad}, {b.x1 + pad * 0.3, b.y1 + pad * 0.3}, 0.02);
        st.need_fit[ws] = false;
    }
    if (iso) {
        if (active && (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f)) {
            if (ws >= 0 && ws < ui::kWorkspaceCount) {
                st.iso_pan[ws].x += io.MouseDelta.x;
                st.iso_pan[ws].y += io.MouseDelta.y;
            }
        }
        if (hovered && io.MouseWheel != 0.0f) {
            if (ws >= 0 && ws < ui::kWorkspaceCount) {
                st.iso_zoom[ws] = std::clamp(
                    st.iso_zoom[ws] * static_cast<float>(std::pow(1.15, io.MouseWheel)), 0.2f, 10.0f);
            }
        }
    } else {
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
        draw_iso(st, dl, origin, avail, ws);
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
            ui::help(Help::GhostReplay);
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
