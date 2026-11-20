#include "sim/app/telemetry_view.hpp"

#include <imgui.h>
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

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

    // --- toolbar ---------------------------------------------------------------------------
    ImGui::SetNextItemWidth(120);
    float win = static_cast<float>(v.window_s);
    if (ImGui::SliderFloat("window", &win, 0.5f, 60.0f, "%.1f s", ImGuiSliderFlags_Logarithmic))
        v.window_s = win;
    ImGui::SameLine();
    if (ImGui::Checkbox("Follow", &v.follow) && v.follow) v.scrub = false;
    ImGui::SameLine();
    if (ImGui::Button("Export view CSV")) act.export_view = true;
    ImGui::SameLine();
    if (ImGui::Button("Export all CSV")) act.export_all = true;
    ImGui::SameLine();
    if (ImGui::Button("Channels...")) ImGui::OpenPopup("channels");
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
        ImGui::SameLine();
        ImGui::TextColored({1.0f, 0.6f, 0.3f, 1.0f}, "channel limit (%zu): %zu dropped",
                           host::TelemetryRecorder::kMaxChannels, rec.dropped_channels());
    }

    // Timeline: scrub the recorded window (paused only).
    ImGui::BeginDisabled(running || !clock || clock->size() == 0);
    double cur = v.scrub ? v.cursor_t : t_last;
    ImGui::SetNextItemWidth(-ImGui::GetFontSize() * 22.0f);
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
    ImGui::SameLine();
    ImGui::BeginDisabled(!v.scrub);
    if (ImGui::Button("Rewind sim to cursor")) act.rewind = true;
    ImGui::SameLine();
    if (ImGui::Button("Live")) {
        act.live = true;
        v.scrub = false;
        v.follow = true;
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("%.1f s recorded", t_last - t_first);

    // --- plots -----------------------------------------------------------------------------
    std::vector<PlotSpec> plots = {
        {"Linear velocity", "mm/s", {"v_set", "v", "sim.v"}, ""},
        {"Angular velocity", "deg/s", {"w_set", "w", "sim.w", "sim.gyro_z"}, ""},
        {"Steering / PID error", "", {"steer", "fwd_err", "rot_err"}, ""},
        {"Motor voltage", "V", {"sim.motor_l", "sim.motor_r"}, ""},
        {"Sensor distance", "mm", {}, "sim.d."},
    };
    if (!v.custom.empty()) plots.push_back({"Custom", "", {v.custom.begin(), v.custom.end()}, ""});

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
    if (ImPlot::BeginSubplots("##telemetry", rows, 1, {-1, std::max(size.y, 120.0f * rows)},
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
                if (ImPlot::DragLineX(0, &v.cursor_t, {1.0f, 0.85f, 0.3f, 1.0f}, 1.5f))
                    v.cursor_t = std::clamp(v.cursor_t, t_first, t_last);
                if (first) ImPlot::TagX(v.cursor_t, {1.0f, 0.85f, 0.3f, 1.0f}, "%.3f", v.cursor_t);
            }
            // Clicking a paused plot puts the scrub cursor there.
            if (!running && ImPlot::IsPlotHovered() && ImGui::IsMouseDoubleClicked(0)) {
                v.scrub = true;
                v.follow = false;
                v.cursor_t = std::clamp(ImPlot::GetPlotMousePos().x, t_first, t_last);
            }
            ImPlot::EndPlot();
            first = false;
        }
        ImPlot::EndSubplots();
    }
    // Free zoom/pan when paused: once the user touches the axes, stop following.
    if (!running && v.follow && ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) &&
        (ImGui::GetIO().MouseWheel != 0.0f || ImGui::IsMouseDragging(0)))
        v.follow = false;
    std::sort(v.plotted.begin(), v.plotted.end());
    v.plotted.erase(std::unique(v.plotted.begin(), v.plotted.end()), v.plotted.end());
    return act;
}

}  // namespace mmsim::app
