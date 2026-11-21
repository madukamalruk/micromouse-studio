#include "sim/app/telemetry_view.hpp"

#include <imgui.h>
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "sim/app/theme.hpp"
#include "sim/app/widgets.hpp"

namespace mmsim::app {

void visible_range(const host::TelemetryChannel& c, double x0, double x1, std::size_t& i0,
                   std::size_t& i1) {
    const std::size_t a = c.lower_bound(x0);
    const std::size_t b = c.lower_bound(x1);
    i0 = a > 0 ? a - 1 : 0;
    i1 = std::min(c.size(), b + 1);
}

namespace {

constexpr std::size_t kMaxPointsPerSeries = 2000;  // min/max buckets -> <= 4000 vertices

struct PlotSpec {
    const char* title;
    const char* unit;
    std::vector<std::string> channels;  // missing channels are skipped
    std::string prefix;                 // or: every channel starting with prefix
    Help help;                          // (?) in the plot's corner
};

/// Min/max decimation of the visible samples: spikes survive however far you zoom out.
void decimate(const host::TelemetryChannel& c, std::size_t i0, std::size_t i1,
              std::vector<ImPlotPoint>& out) {
    out.clear();
    const std::size_t n = i1 > i0 ? i1 - i0 : 0;
    if (n == 0) return;
    if (n <= 2 * kMaxPointsPerSeries) {
        for (std::size_t i = i0; i < i1; ++i) out.push_back({c.t[i], c.v[i]});
        return;
    }
    const std::size_t per = (n + kMaxPointsPerSeries - 1) / kMaxPointsPerSeries;
    for (std::size_t b = i0; b < i1; b += per) {
        const std::size_t e = std::min(i1, b + per);
        std::size_t lo = b, hi = b;
        bool any = false;
        for (std::size_t i = b; i < e; ++i) {
            if (std::isnan(c.v[i])) continue;
            if (!any || c.v[i] < c.v[lo]) lo = i;
            if (!any || c.v[i] > c.v[hi]) hi = i;
            any = true;
        }
        if (!any) {
            out.push_back({c.t[b], NAN});
            continue;
        }
        const std::size_t f = std::min(lo, hi), s = std::max(lo, hi);
        out.push_back({c.t[f], c.v[f]});
        if (s != f) out.push_back({c.t[s], c.v[s]});
    }
}

ImPlotPoint point_getter(int idx, void* data) {
    return static_cast<const std::vector<ImPlotPoint>*>(data)->at(static_cast<std::size_t>(idx));
}

}  // namespace

TelemetryActions draw_telemetry(TelemetryView& v, const host::TelemetryRecorder& rec, double now_s,
                                bool running) {
    TelemetryActions act;
    if (running) {
        v.follow = true;
        v.scrub = false;
    }
    const host::TelemetryChannel* clock = rec.channel("sim.v");
    const double t_first = clock && clock->size() ? clock->t[0] : 0.0;
    const double t_last = clock && clock->size() ? clock->t.back() : now_s;

    // --- toolbar (wraps instead of clipping: DESIGN.md "zero clipped labels") -------------
    const char* group_names[] = {"Motion", "Control", "Power", "Sensors"};
    for (int i = 0; i < 4; ++i) {
        if (i) ui::flow(ui::button_width(group_names[i]));
        const unsigned bit = 1u << i;
        if (ui::pill_toggle(group_names[i], (v.groups & bit) != 0)) v.groups ^= bit;
    }
    ui::help(Help::TelemetryGroups);
    {
        const float w = ImGui::GetFontSize() * 9.0f;
        ui::flow(w);
        ImGui::SetNextItemWidth(w);
        float win = static_cast<float>(v.window_s);
        if (ImGui::DragFloat("##window", &win, 0.05f, 0.5f, 60.0f, "window %.1f s",
                             ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic))
            v.window_s = win;
        ui::help(Help::TelemetryWindow, true);
    }
    ui::flow(ImGui::GetFrameHeight() + ImGui::CalcTextSize("Follow").x + 8.0f);
    if (ImGui::Checkbox("Follow", &v.follow) && v.follow) v.scrub = false;
    ui::help(Help::TelemetryFollow, true);
    ui::flow(ui::button_width("Export view CSV"));
    if (ImGui::Button("Export view CSV")) act.export_view = true;
    ui::flow(ui::button_width("Export all CSV"));
    if (ImGui::Button("Export all CSV")) act.export_all = true;
    ui::flow(ui::button_width("Channels..."));
    if (ImGui::Button("Channels...")) ImGui::OpenPopup("channels");
    ui::help(Help::TelemetryExport, true);
    if (ImGui::BeginPopup("channels")) {
        ImGui::TextDisabled("Custom plot channels");
        for (const auto& c : rec.channels()) {
            bool on = v.custom.count(c->name) > 0;
            if (ImGui::Checkbox(c->name.c_str(), &on)) {
                if (on)
                    v.custom.insert(c->name);
                else
                    v.custom.erase(c->name);
            }
        }
        ImGui::EndPopup();
    }
    if (rec.dropped_channels() > 0) {
        char b[96];
        std::snprintf(b, sizeof b, "channel limit (%zu): %zu dropped",
                      host::TelemetryRecorder::kMaxChannels, rec.dropped_channels());
        ui::flow(ImGui::CalcTextSize(b).x);
        ImGui::TextColored(theme::v4(tok::warn), "%s", b);
    }

    // Timeline: scrub the recorded window (paused only).
    ImGui::BeginDisabled(running || !clock || clock->size() == 0);
    double cur = v.scrub ? v.cursor_t : t_last;
    char rec_text[48];
    std::snprintf(rec_text, sizeof rec_text, "%.1f s recorded", t_last - t_first);
    const ImGuiStyle& st = ImGui::GetStyle();
    const float tail = ui::button_width("Rewind sim to cursor") + ui::button_width("Live") +
                       ImGui::CalcTextSize(rec_text).x + st.ItemSpacing.x * 3;
    const float avail = ImGui::GetContentRegionAvail().x;
    ImGui::SetNextItemWidth(avail - tail > 160.0f ? avail - tail : -1.0f);
    if (ImGui::SliderScalar("##timeline", ImGuiDataType_Double, &cur, &t_first, &t_last,
                            "t = %.3f s")) {
        v.scrub = true;
        v.cursor_t = cur;
        v.follow = false;
        // Keep the cursor in view while scrubbing.
        if (cur < v.view_t0 || cur > v.view_t1) {
            const double half = (v.view_t1 - v.view_t0) / 2.0;
            v.view_t0 = cur - half;
            v.view_t1 = cur + half;
            v.jump = true;
        }
    }
    ImGui::BeginDisabled(!v.scrub);
    ui::flow(ui::button_width("Rewind sim to cursor"));
    if (ImGui::Button("Rewind sim to cursor")) act.rewind = true;
    ui::flow(ui::button_width("Live"));
    if (ImGui::Button("Live")) {
        act.live = true;
        v.scrub = false;
        v.follow = true;
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ui::flow(ImGui::CalcTextSize(rec_text).x + ImGui::CalcTextSize("(?)").x + st.ItemSpacing.x);
    ImGui::TextDisabled("%s", rec_text);
    ui::help(Help::TelemetryTimeline);

    // --- plots -----------------------------------------------------------------------------
    std::vector<PlotSpec> plots;
    if (v.groups & 1u) {
        plots.push_back({"Linear velocity", "mm/s", {"v_set", "v", "sim.v"}, "", Help::PlotLinear});
        plots.push_back({"Angular velocity",
                         "deg/s",
                         {"w_set", "w", "sim.w", "sim.gyro_z"},
                         "",
                         Help::PlotAngular});
    }
    if (v.groups & 2u)
        plots.push_back(
            {"Steering / PID error", "", {"steer", "fwd_err", "rot_err"}, "", Help::PlotSteering});
    if (v.groups & 4u) {
        plots.push_back(
            {"Motor voltage", "V", {"sim.motor_l", "sim.motor_r"}, "", Help::PlotMotorVoltage});
        plots.push_back({"Motor current", "A", {"sim.i_l", "sim.i_r"}, "", Help::PlotMotorCurrent});
        plots.push_back({"Battery", "V", {"sim.v_bat"}, "", Help::PlotBattery});
    }
    if (v.groups & 8u) plots.push_back({"Sensor distance", "mm", {}, "sim.d.", Help::PlotSensors});
    if (!v.custom.empty())
        plots.push_back({"Custom", "", {v.custom.begin(), v.custom.end()}, "", Help::PlotCustom});

    double x0 = v.view_t0, x1 = v.view_t1;
    if (v.follow) {
        x1 = std::max(now_s, v.window_s * 0.02);
        x0 = x1 - v.window_s;
    }
    const bool force_x = v.follow || v.jump;  // follow, or the timeline moved the view
    v.jump = false;
    v.plotted.clear();
    static std::vector<ImPlotPoint> pts;

    const ImVec2 size = ImGui::GetContentRegionAvail();
    const int rows = static_cast<int>(plots.size());
    if (rows == 0) {
        ImGui::TextDisabled("Pick a plot group above.");
        return act;
    }
    const float row_h = 96.0f;
    ImGui::BeginChild("##plots", {0, 0}, ImGuiChildFlags_None);
    if (ImPlot::BeginSubplots("##telemetry", rows, 1,
                              {-1, std::max(ImGui::GetContentRegionAvail().y, row_h * rows)},
                              ImPlotSubplotFlags_LinkAllX | ImPlotSubplotFlags_NoTitle)) {
        bool first = true;
        for (const auto& p : plots) {
            if (!ImPlot::BeginPlot(p.title, {-1, 0}, ImPlotFlags_NoMenus | ImPlotFlags_NoTitle))
                continue;
            ImPlot::SetupAxes(nullptr, p.unit,
                              ImPlotAxisFlags_NoLabel | (first ? 0 : ImPlotAxisFlags_NoTickLabels),
                              v.follow ? ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit : 0);
            ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
            if (force_x) ImPlot::SetupAxisLimits(ImAxis_X1, x0, x1, ImPlotCond_Always);
            ImPlot::SetupFinish();
            const ImPlotRect lim = ImPlot::GetPlotLimits();
            if (first) {
                v.view_t0 = lim.X.Min;
                v.view_t1 = lim.X.Max;
            }
            std::vector<const host::TelemetryChannel*> series;
            for (const auto& n : p.channels)
                if (const auto* c = rec.channel(n)) series.push_back(c);
            if (!p.prefix.empty())
                for (const auto& c : rec.channels())
                    if (c->name.rfind(p.prefix, 0) == 0) series.push_back(c.get());
            for (const auto* c : series) {
                std::size_t i0 = 0, i1 = 0;
                visible_range(*c, lim.X.Min, lim.X.Max, i0, i1);
                decimate(*c, i0, i1, pts);
                ImPlotSpec spec;
                spec.Flags = ImPlotLineFlags_SkipNaN;
                spec.LineWeight = 1.5f;
                const std::string label =
                    p.prefix.empty() ? c->name : c->name.substr(p.prefix.size());
                ImPlot::PlotLineG(label.c_str(), point_getter, &pts, static_cast<int>(pts.size()),
                                  spec);
                v.plotted.push_back(c->name);
            }
            if (v.scrub) {
                if (ImPlot::DragLineX(0, &v.cursor_t, theme::v4(tok::warn), 1.5f))
                    v.cursor_t = std::clamp(v.cursor_t, t_first, t_last);
                if (first) ImPlot::TagX(v.cursor_t, theme::v4(tok::warn), "%.3f", v.cursor_t);
            }
            // Clicking a paused plot puts the scrub cursor there.
            if (!running && ImPlot::IsPlotHovered() && ImGui::IsMouseDoubleClicked(0)) {
                v.scrub = true;
                v.follow = false;
                v.cursor_t = std::clamp(ImPlot::GetPlotMousePos().x, t_first, t_last);
            }
            const ImVec2 pp = ImPlot::GetPlotPos(), ps = ImPlot::GetPlotSize();
            ImPlot::EndPlot();
            // Plot title + (?) in the top-right corner (the legend sits top-left).
            {
                const float tw =
                    ImGui::CalcTextSize(p.title).x + ImGui::CalcTextSize("(?)").x + 10.0f;
                const float x = pp.x + ps.x - tw - 6.0f;
                ImGui::GetWindowDrawList()->AddRectFilled(
                    {x - 4.0f, pp.y + 1.0f},
                    {x + tw + 2.0f, pp.y + 5.0f + ImGui::GetTextLineHeight()},
                    theme::u32(tok::canvas, 0.75f), 3.0f);
                ImGui::GetWindowDrawList()->AddText({x, pp.y + 3.0f}, theme::u32(tok::muted),
                                                    p.title);
                ui::help_hotspot(x + ImGui::CalcTextSize(p.title).x + 8.0f, pp.y + 3.0f, p.help);
            }
            first = false;
        }
        ImPlot::EndSubplots();
    }
    ImGui::EndChild();
    (void)size;
    // Free zoom/pan when paused: once the user touches the axes, stop following.
    if (!running && v.follow && ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) &&
        (ImGui::GetIO().MouseWheel != 0.0f || ImGui::IsMouseDragging(0)))
        v.follow = false;
    std::sort(v.plotted.begin(), v.plotted.end());
    v.plotted.erase(std::unique(v.plotted.begin(), v.plotted.end()), v.plotted.end());
    return act;
}

}  // namespace mmsim::app
