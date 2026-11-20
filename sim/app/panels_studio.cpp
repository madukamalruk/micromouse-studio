// Maze Studio: overlays (heatmaps, route preview), trajectory deck, cell inspector.
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "sim/app/panels.hpp"
#include "sim/app/theme.hpp"
#include "sim/app/widgets.hpp"

namespace mmsim::app {

namespace an = core::analysis;

const StudioPlan& studio_plan(AppState& st) {
    static StudioPlan plan;
    if (!st.maze) {
        plan = {};
        return plan;
    }
    std::uint64_t key = st.maze->hash();
    auto mixk = [&key](double v) {
        key = key * 1099511628211ull ^ static_cast<std::uint64_t>(std::llround(v * 1000.0));
    };
    mixk(st.run_mms);
    mixk(st.accel_mms2);
    mixk(st.mu);
    mixk(st.diagonals ? 1 : 0);
    mixk(st.dims.cell);
    if (key == plan.key && key != 0) return plan;
    plan = {};
    plan.key = key;
    const auto g = an::WallGrid::from_maze(*st.maze);
    plan.path = an::shortest_path(g);
    plan.potential = an::potential_field(g);
    for (double c : plan.potential.cost) plan.potential_max = std::max(plan.potential_max, c);
    if (plan.path && plan.path->cells.size() >= 2) {
        an::TrajectoryOptions o;
        o.run_mms = st.run_mms;
        o.accel_mms2 = st.accel_mms2;
        o.mu = st.mu;
        o.diagonal = st.diagonals;
        o.cell_mm = st.dims.cell * 1000.0;
        plan.traj = an::compile_trajectory(plan.path->cells, o);
        const auto& cells = plan.path->cells;
        const std::size_t moves = cells.size() - 1;
        std::vector<int> dir(moves);
        for (std::size_t i = 0; i < moves; ++i) {
            const int dx = cells[i + 1].x - cells[i].x, dy = cells[i + 1].y - cells[i].y;
            dir[i] = dy > 0 ? 0 : dx > 0 ? 1 : dy < 0 ? 2 : 3;
        }
        plan.diagonal_move.assign(moves, false);
        if (st.diagonals)
            for (std::size_t i = 1; i + 1 < moves; ++i)
                if (dir[i] != dir[i - 1] && dir[i + 1] != dir[i]) plan.diagonal_move[i] = true;
    }
    return plan;
}

const an::Heatmap& heat_cache(AppState& st) {
    static an::Heatmap heat;
    static const host::Session* sess = nullptr;
    static std::int64_t t_us = -1;
    static double wall = -1.0;
    const double now = ImGui::GetTime();
    const std::int64_t sim_t = st.session ? st.session->time_us() : -1;
    const bool changed = st.session.get() != sess || sim_t != t_us;
    if (changed && (st.session.get() != sess || st.scripted || now - wall > 0.25 || sim_t < t_us)) {
        heat = motion_heat(st);
        sess = st.session.get();
        t_us = sim_t;
        wall = now;
    }
    return heat;
}

const an::HeatCell* heat_at(const an::Heatmap& h, int x, int y) {
    for (const auto& c : h.cells)
        if (c.x == x && c.y == y) return &c;
    return nullptr;
}

namespace {

void gradient_legend(const char* lo, const char* hi) {
    const float w = ImGui::GetContentRegionAvail().x, h = 10.0f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const int steps = 24;
    for (int i = 0; i < steps; ++i) {
        const float f0 = static_cast<float>(i) / steps, f1 = static_cast<float>(i + 1) / steps;
        const Rgba a{20, 64, 84, 255};
        auto col = [&](float f) {
            if (f < 0.4f) return mix(a, tok::accent, f / 0.4f);
            if (f < 0.75f) return mix(tok::accent, tok::warn, (f - 0.4f) / 0.35f);
            return mix(tok::warn, tok::bad, (f - 0.75f) / 0.25f);
        };
        dl->AddRectFilled({p.x + w * f0, p.y}, {p.x + w * f1 + 0.5f, p.y + h}, theme::u32(col(f0)));
    }
    ImGui::Dummy({w, h});
    ImGui::TextDisabled("%s", lo);
    const float tw = ImGui::CalcTextSize(hi).x;
    ImGui::SameLine(std::max(ImGui::GetCursorPosX(), ImGui::GetWindowContentRegionMax().x - tw));
    ImGui::TextDisabled("%s", hi);
}

}  // namespace

void panel_overlays(AppState& st) {
    if (!ImGui::Begin(win::kOverlays)) {
        ImGui::End();
        return;
    }
    ui::section("Heatmap");
    struct L {
        HeatLayer l;
        const char* name;
        const char* tip;
    };
    const L layers[] = {
        {HeatLayer::Off, "Off", "No heatmap"},
        {HeatLayer::PeakSpeed, "Peak speed", "Fastest speed recorded in each cell"},
        {HeatLayer::GripUse, "Grip use", "Peak acceleration / (mu g): 100 % = tyres at the limit"},
        {HeatLayer::Potential, "Turn-weighted potential",
         "Cost to the goal with turns weighted 0.5 cell: hot = close"}};
    bool first = true;
    for (const auto& l : layers) {
        if (!first) ui::flow(ui::button_width(l.name));
        first = false;
        if (ui::pill_toggle(l.name, st.heat == l.l, l.tip)) st.heat = l.l;
    }
    if (st.heat != HeatLayer::Off) {
        const auto& h = heat_cache(st);
        char hi[48];
        if (st.heat == HeatLayer::PeakSpeed) {
            std::snprintf(hi, sizeof hi, "%.2f m/s", h.v_max * 1e-3);
            gradient_legend("0", hi);
        } else if (st.heat == HeatLayer::GripUse) {
            gradient_legend("0 %", "100 % of mu g");
        } else {
            std::snprintf(hi, sizeof hi, "goal");
            gradient_legend("far", hi);
        }
        if (st.heat != HeatLayer::Potential && h.cells.empty())
            ui::hint("Run the firmware to collect motion data for this layer.");
    }
    ui::section("Route preview");
    ImGui::Checkbox("Show planned route", &st.show_route);
    ui::hint("Shortest route on the true maze. Blue = orthogonal moves, amber = diagonal chains.");
    ui::section("Layers");
    ImGui::Checkbox("Grid", &st.view.show_grid);
    ImGui::Checkbox("Distances to goal", &st.view.show_distances);
    ImGui::Checkbox("Coordinates", &st.view.show_coords);
    ImGui::Checkbox("Known walls (firmware)", &st.view.show_known_walls);
    ImGui::Checkbox("Firmware cell colours", &st.view.show_fw_colors);
    ImGui::Checkbox("Firmware cell text", &st.view.show_fw_text);
    ImGui::Checkbox("Path trace", &st.robot_view.show_trace);
    ImGui::Checkbox("Crash markers", &st.robot_view.show_crashes);
    ImGui::End();
}

void panel_trajectory(AppState& st) {
    if (!ImGui::Begin(win::kTrajectory)) {
        ImGui::End();
        return;
    }
    if (!st.maze) {
        ui::hint("Load a maze to plan a speed run.");
        ImGui::End();
        return;
    }
    const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    ImGui::SetNextItemWidth(half);
    ImGui::DragFloat("##run", &st.run_mms, 5.0f, 200.0f, 3000.0f, "run %.0f mm/s",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-1);
    ImGui::DragFloat("##acc", &st.accel_mms2, 25.0f, 500.0f, 15000.0f, "accel %.0f mm/s2",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::SetNextItemWidth(half);
    ImGui::DragFloat("##mu", &st.mu, 0.005f, 0.3f, 1.5f, "tyre mu %.2f",
                     ImGuiSliderFlags_AlwaysClamp);
    ImGui::SameLine();
    ImGui::Checkbox("Diagonals", &st.diagonals);
    const auto& plan = studio_plan(st);
    if (!plan.path) {
        ui::pill("No route to the goal", ui::Tone::Bad);
        ImGui::End();
        return;
    }
    const auto& t = plan.traj;
    if (!t.ok()) {
        ui::pill(t.error.c_str(), ui::Tone::Bad);
        ImGui::End();
        return;
    }
    if (ImGui::BeginTable("sum", 4, ImGuiTableFlags_SizingStretchSame)) {
        auto stat = [](const char* label, const char* v, Rgba c = tok::text) {
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", label);
            ImGui::PushFont(theme::fonts().mono, theme::kTitleSize);
            ImGui::TextColored(theme::v4(c), "%s", v);
            ImGui::PopFont();
        };
        char b[4][32];
        std::snprintf(b[0], 32, "%.2f s", t.total_s);
        std::snprintf(b[1], 32, "%.2f m", t.total_mm * 1e-3);
        std::snprintf(b[2], 32, "%d / %d", t.moves, t.diagonals);
        std::snprintf(b[3], 32, "%.2f g", t.peak_g);
        stat("Run time", b[0]);
        stat("Distance", b[1]);
        stat("Moves / diag", b[2]);
        stat("Peak lateral", b[3],
             t.peak_g > st.mu          ? tok::bad
             : t.peak_g > 0.85 * st.mu ? tok::warn
                                       : tok::good);
        ImGui::EndTable();
    }
    ImGui::TextDisabled("Corner limit %.0f mm/s at r = %.0f mm (mu g)", t.limit_mms,
                        an::kCornerRmm);
    const ImGuiTableFlags tf = ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                               ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_BordersInnerH;
    if (ImGui::BeginTable("deck", 6, tf, {0, ImGui::GetContentRegionAvail().y})) {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 28.0f);
        ImGui::TableSetupColumn("Move");
        ImGui::TableSetupColumn("Length");
        ImGui::TableSetupColumn("Speed");
        ImGui::TableSetupColumn("Time");
        ImGui::TableSetupColumn("g");
        ImGui::TableHeadersRow();
        for (const auto& p : t.primitives) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ui::mono("%d", p.seq);
            ImGui::TableNextColumn();
            const Rgba c = p.kind == "diagonal" ? tok::warn
                           : p.kind == "curve"  ? tok::text
                                                : tok::accent;
            if (p.kind == "curve")
                ImGui::TextColored(theme::v4(c), "%c %d deg", p.dir, p.angle_deg);
            else
                ImGui::TextColored(theme::v4(c), "%s", p.kind.c_str());
            ImGui::TableNextColumn();
            ui::mono("%.0f mm", p.length_mm);
            ImGui::TableNextColumn();
            ui::mono("%.0f", p.v_mms);
            ImGui::TableNextColumn();
            ui::mono("%d ms", p.ms);
            ImGui::TableNextColumn();
            if (p.kind == "curve")
                ui::mono_colored(p.g > st.mu ? tok::bad : tok::text, "%.2f", p.g);
            else
                ImGui::TextDisabled("-");
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

void panel_inspector(AppState& st) {
    if (!ImGui::Begin(win::kInspector)) {
        ImGui::End();
        return;
    }
    if (!st.maze) {
        ui::hint("Load a maze first.");
        ImGui::End();
        return;
    }
    auto k = st.inspect ? st.inspect : st.hover;
    const bool following = !k && st.session;
    if (following) k = cell_of(st.session->world().drive().pose, st.dims);
    if (!k || k->x < 0 || k->y < 0 || k->x >= st.maze->width() || k->y >= st.maze->height()) {
        ui::hint("Click a cell in the maze to inspect it.");
        ImGui::End();
        return;
    }
    const auto& m = *st.maze;
    const int x = k->x, y = k->y;
    ImGui::PushFont(theme::fonts().mono, theme::kTitleSize);
    ImGui::Text("(%d, %d)", x, y);
    ImGui::PopFont();
    ImGui::SameLine();
    if (m.is_goal(*k))
        ui::pill("goal", ui::Tone::Good, false);
    else if (x == m.start().x && y == m.start().y)
        ui::pill("start", ui::Tone::Active, false);
    char walls[16];
    std::snprintf(walls, sizeof walls, "%s%s%s%s", m.has_wall(x, y, core::Dir::N) ? "N " : "",
                  m.has_wall(x, y, core::Dir::E) ? "E " : "",
                  m.has_wall(x, y, core::Dir::S) ? "S " : "",
                  m.has_wall(x, y, core::Dir::W) ? "W" : "");
    ui::value_row("Walls", "%s", walls[0] ? walls : "none");
    if (st.flood) ui::value_row("Steps to goal (BFS)", "%d", st.flood->at(x, y));
    const auto& plan = studio_plan(st);
    if (!plan.potential.cost.empty()) {
        const double c = plan.potential.cost[static_cast<std::size_t>(y * m.width() + x)];
        if (c >= 0)
            ui::value_row("Turn-weighted cost", "%.1f", c);
        else
            ui::value_row("Turn-weighted cost", "no route");
    }
    bool on_route = false;
    if (plan.path)
        for (const auto& c : plan.path->cells) on_route = on_route || (c.x == x && c.y == y);
    ui::value_row("On planned route", "%s", on_route ? "yes" : "no");
    const auto& h = heat_cache(st);
    if (const auto* hc = heat_at(h, x, y)) {
        ui::value_row("Peak speed", "%.2f m/s", hc->v_max * 1e-3);
        ui::value_row("Peak lateral accel", "%.2f g", hc->a_lat_max / an::kGmms2);
        ui::value_row("Grip use", "%.0f %%",
                      100.0 * std::hypot(hc->a_lat_max, hc->a_long_max) / (st.mu * an::kGmms2));
    } else {
        ui::value_row("Visited", "no");
    }
    // Closest approach of the axle centre to a wall face in this cell (from the trace).
    double best = 1e9;
    const double c = st.dims.cell, t = st.dims.wall * 0.5;
    for (const auto& p : st.trace.points()) {
        if (static_cast<int>(std::floor(p.x / c)) != x ||
            static_cast<int>(std::floor(p.y / c)) != y)
            continue;
        if (m.has_wall(x, y, core::Dir::S)) best = std::min(best, p.y - (y * c + t));
        if (m.has_wall(x, y, core::Dir::N)) best = std::min(best, (y + 1) * c - t - p.y);
        if (m.has_wall(x, y, core::Dir::W)) best = std::min(best, p.x - (x * c + t));
        if (m.has_wall(x, y, core::Dir::E)) best = std::min(best, (x + 1) * c - t - p.x);
    }
    if (best < 1e8) ui::value_row("Min wall clearance", "%.0f mm", best * 1000.0);
    if (st.session) {
        const auto& ann = st.session->annotations();
        if (auto it = ann.text.find({x, y}); it != ann.text.end())
            ui::value_row("Firmware text", "%s", it->second.c_str());
        if (auto it = ann.color.find({x, y}); it != ann.color.end())
            ui::value_row("Firmware colour", "%c", it->second);
    }
    if (following) ui::hint("Showing the mouse's cell. Click any cell to pin it.");
    if (st.inspect && ImGui::Button("Clear selection")) st.inspect.reset();
    ImGui::End();
}

}  // namespace mmsim::app
