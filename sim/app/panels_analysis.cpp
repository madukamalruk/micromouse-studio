// Telemetry & Compare: waveforms, run archive A vs B (common-distance dt(s), v(s)), traction.
#include <imgui.h>
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "sim/app/panels.hpp"
#include "sim/app/theme.hpp"
#include "sim/app/widgets.hpp"

namespace mmsim::app {

namespace fs = std::filesystem;
namespace an = core::analysis;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;

an::RunSeries series_of_current(const AppState& st) {
    if (!st.session) return {};
    const auto& rec = st.session->recorder();
    const auto *v = rec.channel("sim.v"), *g = rec.channel("sim.gyro_z");
    if (!v || !g) return {};
    std::vector<double> t_ms, vv, gg;
    for (std::size_t i = 0; i < v->size(); ++i) {
        t_ms.push_back(v->t[i] * 1000.0);
        vv.push_back(v->v[i]);
        gg.push_back(g->value_at(v->t[i]));
    }
    return an::run_series(t_ms, vv, gg);
}

an::RunSeries series_of_archive(const an::ArchivedRun& r) {
    const auto *t = r.column("t"), *v = r.column("sim.v"), *g = r.column("sim.gyro_z");
    if (!t || !v || !g) return {};
    std::vector<double> t_ms, vv, gg;
    for (std::size_t i = 0; i < t->size(); ++i) {
        t_ms.push_back((*t)[i] * 1000.0);
        vv.push_back((*v)[i]);
        gg.push_back((*g)[i]);
    }
    return an::run_series(t_ms, vv, gg);
}

std::string run_label(const fs::path& p) {
    return p.filename().string();
}

}  // namespace

GripSamples grip_samples(const AppState& st, double mu) {
    GripSamples g;
    if (!st.session) return g;
    const auto& rec = st.session->recorder();
    const auto *v = rec.channel("sim.v"), *w = rec.channel("sim.w");
    if (!v || !w || v->size() < 3) return g;
    const std::size_t stride = std::max<std::size_t>(1, v->size() / 4000);
    for (std::size_t i = stride; i < v->size(); i += stride) {
        const double dt = v->t[i] - v->t[i - stride];
        if (dt <= 0) continue;
        const double al = (v->v[i] - v->v[i - stride]) * 1e-3 / dt / kG;
        const double ac = v->v[i] * 1e-3 * w->value_at(v->t[i]) * kPi / 180.0 / kG;
        g.a_long_g.push_back(al);
        g.a_lat_g.push_back(ac);
        g.peak_ratio = std::max(g.peak_ratio, std::hypot(al, ac) / mu);
    }
    return g;
}

void panel_telemetry(AppState& st) {
    if (!ImGui::Begin(win::kTelemetry)) {
        ImGui::End();
        return;
    }
    if (!st.session) {
        ui::hint("No session. Load a maze and a firmware.");
        ImGui::End();
        return;
    }
    const auto& rec = st.session->recorder();
    const double now = static_cast<double>(st.session->time_us()) * 1e-6;
    const TelemetryActions a = draw_telemetry(st.tv, rec, now, st.running);
    if (a.export_view || a.export_all) {
        const fs::path out = timestamped(st.root / "screenshots", "telemetry", ".csv");
        std::size_t rows = 0;
        const bool ok =
            a.export_all ? export_csv(rec, out, -1e300, 1e300, {}, rows)
                         : export_csv(rec, out, st.tv.view_t0, st.tv.view_t1, st.tv.plotted, rows);
        toast(st,
              ok ? "Saved " + out.filename().string() + " (" + std::to_string(rows) + " rows)"
                 : "Could not write " + out.string(),
              ok ? ui::Tone::Good : ui::Tone::Bad);
    }
    if (a.rewind) {
        set_running(st, false);
        rewind_to(st, st.tv.cursor_t);
        st.tv.scrub = false;
        st.tv.follow = true;
    }
    ImGui::End();
}

void panel_compare(AppState& st) {
    if (!ImGui::Begin(win::kCompare)) {
        ImGui::End();
        return;
    }
    // Archive controls.
    if (ui::action_button("Save current run", ui::Tone::Active, !st.session)) {
        std::string err;
        const fs::path p = save_current_run(st, err);
        toast(st, p.empty() ? "Save failed: " + err : "Saved run " + p.filename().string(),
              p.empty() ? ui::Tone::Bad : ui::Tone::Good);
        if (!p.empty() && !st.run_b) load_compare(st, st.archive.size() > 1 ? 1 : 0);
    }
    ui::flow(ui::button_width("Refresh"));
    if (ImGui::Button("Refresh")) refresh_archive(st);
    const std::string current_b =
        st.run_b ? run_label(st.run_b_path)
                 : std::string(st.archive.empty() ? "no archived runs" : "pick run B");
    const float cw = std::min(ImGui::GetContentRegionAvail().x, 340.0f);
    ui::flow(cw);
    ImGui::SetNextItemWidth(cw);
    if (ImGui::BeginCombo("##runb", ("B: " + current_b).c_str())) {
        for (int i = 0; i < static_cast<int>(st.archive.size()); ++i)
            if (ImGui::Selectable(run_label(st.archive[static_cast<std::size_t>(i)]).c_str(),
                                  st.run_b && i == st.compare_b))
                load_compare(st, i);
        ImGui::EndCombo();
    }
    if (!st.compare_status.empty())
        ImGui::TextColored(theme::v4(tok::bad), "%s", st.compare_status.c_str());
    if (!st.run_b) {
        ui::hint(
            "A = the live run. B = an archived run (runs/ folder). Save a run, change the seed, "
            "firmware or config, run again, then compare: dt(s) shows where time is won or lost "
            "at the same distance.");
        ImGui::End();
        return;
    }
    const auto A = series_of_current(st);
    const auto B = series_of_archive(*st.run_b);
    if (!A.error.empty() || !B.error.empty()) {
        ui::hint(!A.error.empty() ? ("Run A: " + A.error).c_str() : ("Run B: " + B.error).c_str());
        ImGui::End();
        return;
    }
    const auto cmp = an::compare_runs(A, B);
    if (!cmp.ok()) {
        ui::hint(cmp.error.c_str());
        ImGui::End();
        return;
    }
    // Summary chips.
    char b[96];
    std::snprintf(b, sizeof b, "Common %.2f m", cmp.common_mm * 1e-3);
    ui::pill(b, ui::Tone::Neutral, false);
    // dt = t_B - t_A at the same distance: > 0 means the live run A is ahead.
    std::snprintf(b, sizeof b,
                  cmp.delta_t_final_s >= 0 ? "A ahead by %.3f s" : "A behind by %.3f s",
                  std::abs(cmp.delta_t_final_s));
    ui::flow(ui::pill_width(b));
    ui::pill(b, cmp.delta_t_final_s >= 0 ? ui::Tone::Good : ui::Tone::Bad, true);
    ui::flow(ui::pill_width(cmp.verdict.c_str()));
    ui::pill(cmp.verdict.c_str(), ui::Tone::Active, false);
    const auto meta = [&](const char* k) {
        auto it = st.run_b->meta.find(k);
        return it == st.run_b->meta.end() ? std::string("-") : it->second;
    };
    ImGui::TextDisabled("B: %s, %s, seed %s, %s", meta("maze").c_str(), meta("source").c_str(),
                        meta("seed").c_str(), meta("outcome").c_str());

    // dt(s) and v(s) on common distance; the scrub cursor maps through run A's s(t).
    const float h = std::max(
        110.0f, (ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y) * 0.5f);
    double cursor_mm = -1.0;
    if (st.tv.scrub) cursor_mm = an::interp_at(A.t, A.s, st.tv.cursor_t);
    if (ImPlot::BeginPlot("##dt", {-1, h}, ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxes("distance (mm)", "t_B - t_A (s)", ImPlotAxisFlags_AutoFit,
                          ImPlotAxisFlags_AutoFit);
        ImPlot::SetupLegend(ImPlotLocation_NorthWest);
        ImPlotSpec spec;
        spec.LineColor = theme::v4(tok::accent);
        spec.FillColor = theme::v4(tok::accent, 0.18f);
        ImPlot::PlotShaded("dt(s): > 0 = A ahead", cmp.grid_mm.data(), cmp.delta_t_s.data(),
                           static_cast<int>(cmp.grid_mm.size()), 0.0, spec);
        ImPlot::PlotLine("dt(s): > 0 = A ahead", cmp.grid_mm.data(), cmp.delta_t_s.data(),
                         static_cast<int>(cmp.grid_mm.size()), spec);
        if (cursor_mm >= 0 && ImPlot::DragLineX(1, &cursor_mm, theme::v4(tok::warn), 1.5f)) {
            st.tv.cursor_t = an::interp_at(A.s, A.t, cursor_mm);
            st.tv.scrub = true;
        }
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("##vs", {-1, h}, ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxes("distance (mm)", "speed (mm/s)", ImPlotAxisFlags_AutoFit,
                          ImPlotAxisFlags_AutoFit);
        ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_Horizontal);
        ImPlotSpec sa;
        sa.LineColor = theme::v4(tok::accent);
        ImPlot::PlotLine("A (live)", cmp.grid_mm.data(), cmp.v_a.data(),
                         static_cast<int>(cmp.grid_mm.size()), sa);
        ImPlotSpec sb;
        sb.LineColor = theme::v4(tok::warn);
        ImPlot::PlotLine("B (archive)", cmp.grid_mm.data(), cmp.v_b.data(),
                         static_cast<int>(cmp.grid_mm.size()), sb);
        if (cursor_mm >= 0 && ImPlot::DragLineX(2, &cursor_mm, theme::v4(tok::warn), 1.5f)) {
            st.tv.cursor_t = an::interp_at(A.s, A.t, cursor_mm);
            st.tv.scrub = true;
        }
        ImPlot::EndPlot();
    }
    ImGui::End();
}

void panel_traction(AppState& st) {
    if (!ImGui::Begin(win::kTraction)) {
        ImGui::End();
        return;
    }
    const double mu = st.mu;
    const auto g = grip_samples(st, mu);
    char b[64];
    std::snprintf(b, sizeof b, "Peak %.0f %% of mu g", g.peak_ratio * 100.0);
    ui::pill(b, g.peak_ratio > 1.0    ? ui::Tone::Bad
                : g.peak_ratio > 0.85 ? ui::Tone::Warn
                                      : ui::Tone::Good);
    ImGui::SameLine();
    ImGui::TextDisabled("mu %.2f (Studio deck)", mu);
    const ImVec2 av = ImGui::GetContentRegionAvail();
    const float side = std::max(120.0f, std::min(av.x, av.y));
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, (av.x - side) * 0.5f));
    if (ImPlot::BeginPlot("##traction", {side, side}, ImPlotFlags_NoMenus | ImPlotFlags_Equal)) {
        const double lim = std::max(mu * 1.25, 0.2);
        ImPlot::SetupAxes("a_c lateral (g)", "a_long (g)");
        ImPlot::SetupAxesLimits(-lim, lim, -lim, lim, ImPlotCond_Always);
        ImPlot::SetupLegend(ImPlotLocation_NorthEast);
        // Friction circle.
        static double cx[129], cy[129];
        for (int i = 0; i <= 128; ++i) {
            cx[i] = mu * std::cos(2 * kPi * i / 128);
            cy[i] = mu * std::sin(2 * kPi * i / 128);
        }
        ImPlotSpec sc;
        sc.LineColor = theme::v4(tok::bad);
        sc.LineWeight = 1.5f;
        ImPlot::PlotLine("mu g limit", cx, cy, 129, sc);
        ImPlotSpec sp;
        sp.Marker = ImPlotMarker_Circle;
        sp.MarkerSize = 1.6f;
        sp.MarkerFillColor = theme::v4(tok::accent, 0.55f);
        sp.MarkerLineColor = theme::v4(tok::accent, 0.0f);
        if (!g.a_lat_g.empty())
            ImPlot::PlotScatter("samples", g.a_lat_g.data(), g.a_long_g.data(),
                                static_cast<int>(g.a_lat_g.size()), sp);
        ImPlot::EndPlot();
    }
    if (g.a_lat_g.empty())
        ui::hint(
            "Run the firmware: every sample's demand lands here. Outside the circle = sliding.");
    ImGui::End();
}

}  // namespace mmsim::app
