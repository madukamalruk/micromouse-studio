// Tuning & Reports: sensor mounts, battery sag + thermal, step response, IMU noise, reports,
// run record.
#include <imgui.h>
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "sim/app/panels.hpp"
#include "sim/app/theme.hpp"
#include "sim/app/widgets.hpp"
#include "sim/core/analysis/metrics.hpp"

namespace mmsim::app {

namespace fs = std::filesystem;
namespace an = core::analysis;

namespace {
constexpr double kPi = 3.14159265358979323846;

void load_mounts(AppState& st) {
    if (st.mounts.loaded || !st.robot) return;
    const auto& s = st.robot->sensors;
    if (s.size() >= 4) {
        st.mounts.side_deg = static_cast<float>(std::abs(s[0].angle_deg));
        st.mounts.front_deg = static_cast<float>(std::abs(s[1].angle_deg));
        st.mounts.rays = s[0].rays;
        st.mounts.beam_deg = static_cast<float>(s[0].fov_deg);
    }
    st.mounts.loaded = true;
}

/// Top-down preview of the robot outline and the four beams with the edited angles.
void mount_preview(const AppState& st, float h) {
    const float w = ImGui::GetContentRegionAvail().x;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, {p.x + w, p.y + h}, theme::u32(tok::canvas), 6.0f);
    dl->PushClipRect(p, {p.x + w, p.y + h}, true);
    const ImVec2 c{p.x + w * 0.5f, p.y + h * 0.66f};
    if (st.robot) {
        const auto& b = st.robot->body;
        const float ppm = h * 0.42f / static_cast<float>(std::max(b.length, 0.02));  // px per metre
        const float L = static_cast<float>(b.length) * ppm, W = static_cast<float>(b.width) * ppm;
        const float off = static_cast<float>(b.offset_x) * ppm;
        dl->AddRect({c.x - W * 0.5f, c.y - L * 0.5f - off}, {c.x + W * 0.5f, c.y + L * 0.5f - off},
                    theme::u32(tok::text, 0.8f), 4.0f, 0, 1.5f);
        for (std::size_t i = 0; i < st.robot->sensors.size() && i < 4; ++i) {
            const auto& sc = st.robot->sensors[i];
            const double sign = sc.angle_deg >= 0 ? 1.0 : -1.0;
            const double ang = (i == 0 || i == 3 ? st.mounts.side_deg : st.mounts.front_deg) * sign;
            const ImVec2 o{c.x - static_cast<float>(sc.y) * ppm,
                           c.y - static_cast<float>(sc.x) * ppm};
            const double len = h * 0.55;
            const double half = st.mounts.beam_deg * 0.5;
            for (double a : {ang - half, ang, ang + half}) {
                const double r = a * kPi / 180.0;
                const ImVec2 e{o.x - static_cast<float>(std::sin(r) * len),
                               o.y - static_cast<float>(std::cos(r) * len)};
                dl->AddLine(o, e, theme::u32({244, 114, 182, 255}, a == ang ? 0.9f : 0.35f),
                            a == ang ? 1.5f : 1.0f);
            }
            dl->AddCircleFilled(o, 3.0f, theme::u32(tok::accent));
        }
    }
    dl->PopClipRect();
    ImGui::Dummy({w, h});
}

/// Width for a full-width widget that leaves room for a "(?)" after it.
float width_for_help() {
    return -ImGui::CalcTextSize("(?)").x - ImGui::GetStyle().ItemSpacing.x * 2.0f;
}

/// A (?) hotspot just after the label of a labeled_bar row that is about to be drawn.
void bar_label_help(const char* label, Help h) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float ty = p.y + (ImGui::GetFrameHeight() - ImGui::GetTextLineHeight()) * 0.5f;
    ui::help_hotspot(p.x + ImGui::CalcTextSize(label).x + 5.0f, ty, h);
}

}  // namespace

void panel_mounts(AppState& st) {
    if (!ImGui::Begin(win::kMounts)) {
        ImGui::End();
        return;
    }
    if (!st.robot) {
        ui::hint("Robot config not loaded.");
        ImGui::End();
        return;
    }
    load_mounts(st);
    if (st.robot->sensors.size() < 4) {
        ui::hint(
            "This config has fewer than four distance sensors; mounts are edited in its JSON.");
        ImGui::End();
        return;
    }
    mount_preview(st, std::clamp(ImGui::GetContentRegionAvail().y * 0.36f, 96.0f, 200.0f));
    ImGui::SetNextItemWidth(width_for_help());
    ImGui::DragFloat("##front", &st.mounts.front_deg, 0.1f, 0.0f, 30.0f, "Front toe-out %.1f deg",
                     ImGuiSliderFlags_AlwaysClamp);
    ui::help(Help::MountFront, true);
    ImGui::SetNextItemWidth(width_for_help());
    ImGui::DragFloat("##side", &st.mounts.side_deg, 0.2f, 20.0f, 90.0f, "Side / diagonal %.1f deg",
                     ImGuiSliderFlags_AlwaysClamp);
    ui::help(Help::MountSide, true);
    ImGui::SetNextItemWidth(width_for_help());
    ImGui::DragInt("##rays", &st.mounts.rays, 0.05f, 1, 15, "Ray count %d",
                   ImGuiSliderFlags_AlwaysClamp);
    ui::help(Help::MountRays, true);
    ImGui::SetNextItemWidth(width_for_help());
    ImGui::DragFloat("##beam", &st.mounts.beam_deg, 0.1f, 0.0f, 40.0f, "Beam width %.1f deg",
                     ImGuiSliderFlags_AlwaysClamp);
    ui::help(Help::MountBeam, true);
    if (ui::action_button("Apply & restart run", ui::Tone::Active)) {
        auto& s = st.robot->sensors;
        for (std::size_t i = 0; i < 4; ++i) {
            const double sign = s[i].angle_deg >= 0 ? 1.0 : -1.0;
            s[i].angle_deg = sign * (i == 0 || i == 3 ? st.mounts.side_deg : st.mounts.front_deg);
            s[i].rays = st.mounts.rays;
            s[i].fov_deg = st.mounts.beam_deg;
        }
        make_session(st);
        toast(st, "Sensor mounts applied; run restarted", ui::Tone::Good);
    }
    ui::help(Help::MountApply, true);
    ui::flow(ui::button_width("Revert"));
    if (ImGui::Button("Revert")) st.mounts.loaded = false;
    ui::hint(
        "Changing a mount restarts the run so the result stays reproducible (seed unchanged).");
    ImGui::End();
}

void panel_battery(AppState& st) {
    if (!ImGui::Begin(win::kBattery)) {
        ImGui::End();
        return;
    }
    if (!st.session) {
        ui::hint("No session.");
        ImGui::End();
        return;
    }
    const auto& w = st.session->world();
    const auto& bc = w.config().battery;
    const double vb = w.battery_voltage(), soc = w.state_of_charge();
    const double cells = bc.cells, vfull = 4.2 * cells, vempty = 3.0 * cells;
    if (!bc.model)
        ui::pill("Ideal supply (battery.model off)", ui::Tone::Neutral, false);
    else
        ui::pill("Battery model on: sag drives the motors", ui::Tone::Active, false);
    const double margin = vb - bc.reset_v;
    const float lw = ImGui::CalcTextSize("U18 LDO 3.3 V").x + 16.0f;
    char b[64];
    std::snprintf(b, sizeof b, "%.2f V  %.0f %%", vb, soc * 100.0);
    bar_label_help("Pack", Help::BatteryPack);
    ui::labeled_bar("Pack", lw, "##vbat", static_cast<float>((vb - vempty) / (vfull - vempty)), b,
                    margin < 0.3   ? tok::bad
                    : margin < 0.8 ? tok::warn
                                   : tok::good);
    std::snprintf(b, sizeof b, "%.2f A", std::abs(w.motor_current(0)));
    bar_label_help("Motor L", Help::BatteryMotorAmps);
    ui::labeled_bar("Motor L", lw, "##il", static_cast<float>(std::abs(w.motor_current(0)) / 2.6),
                    b, tok::accent);
    std::snprintf(b, sizeof b, "%.2f A", std::abs(w.motor_current(1)));
    ui::labeled_bar("Motor R", lw, "##ir", static_cast<float>(std::abs(w.motor_current(1)) / 2.6),
                    b, tok::accent);
    ui::value_row(Help::BrownoutMargin, "Brownout margin", "%+.2f V over %.1f V", margin,
                  bc.reset_v);
    ui::value_row(Help::MotorCap, "Motor cap", "%.1f V = %.0f %% duty",
                  w.config().motor.max_voltage,
                  100.0 * w.config().motor.max_voltage / std::max(0.1, vb));
    ui::value_row(Help::PackCurrent, "Pack current", "%.2f A", w.battery_current());
    ui::value_row(Help::ChargeUsed, "Charge used", "%.1f of %.0f mAh", w.charge_used_mah(),
                  bc.capacity_mah);

    ui::section("Regulator junctions (estimate)", Help::Junctions);
    struct J {
        const char* name;
        const char* id;
        double t;
    };
    const J js[] = {{"U17 buck 5 V", "##u17", st.thermal.u17()},
                    {"U18 LDO 3.3 V", "##u18", st.thermal.u18()},
                    {"U13 driver", "##u13", st.thermal.u13()}};
    for (const auto& j : js) {
        std::snprintf(b, sizeof b, "%.1f C", j.t);
        ui::labeled_bar(j.name, lw, j.id, static_cast<float>((j.t - 25.0) / (125.0 - 25.0)), b,
                        j.t > 105  ? tok::bad
                        : j.t > 85 ? tok::warn
                                   : tok::good);
    }
    // Mission prediction (Python reference model, golden-tested).
    static an::BatteryRun pred;
    static bool have = false;
    if (!have) {
        pred = an::battery_run(an::mission_profile(), bc.capacity_mah, bc.r_int_ohm, bc.soc0);
        have = true;
    }
    ui::value_row(Help::MissionMin, "3-run mission min", "%.2f V (%+.2f V margin)", pred.min_v,
                  pred.margin_to_6v0);
    ImGui::End();
}

void panel_step(AppState& st) {
    if (!ImGui::Begin(win::kStep)) {
        ImGui::End();
        return;
    }
    if (!st.session) {
        ui::hint("No session.");
        ImGui::End();
        return;
    }
    const auto& rec = st.session->recorder();
    static float window_s = 0.6f;
    const float cw = std::min(ImGui::GetContentRegionAvail().x * 0.5f, 200.0f);
    ImGui::SetNextItemWidth(cw);
    if (ImGui::BeginCombo("##stepch", st.step_channel.c_str())) {
        for (const auto& c : rec.channels())
            if (ImGui::Selectable(c->name.c_str(), c->name == st.step_channel))
                st.step_channel = c->name;
        ImGui::EndCombo();
    }
    ui::flow(160.0f);
    ImGui::SetNextItemWidth(160.0f);
    ImGui::DragFloat("##stepwin", &window_s, 0.01f, 0.1f, 10.0f, "window %.2f s",
                     ImGuiSliderFlags_AlwaysClamp);
    ui::help(Help::StepSource, true);
    // First step in the recording, then measured inside [start, start + window]: a speed
    // profile's first acceleration is the step, not the whole run.
    const Series all = series(rec, st.step_channel.c_str());
    const auto first = an::step_metrics(all.t, all.v);
    Series s;
    an::StepMetrics m = first;
    if (first.ok()) {
        const double t0 = first.t_start_s - 0.05, t1 = first.t_start_s + window_s;
        for (std::size_t i = 0; i < all.t.size(); ++i)
            if (all.t[i] >= t0 && all.t[i] <= t1) {
                s.t.push_back(all.t[i]);
                s.v.push_back(all.v[i]);
            }
        // A speed profile ramps back down: stop at the end of the first plateau (within 2 % of
        // the window's peak) so the measured step is the acceleration, not the deceleration.
        if (!s.v.empty()) {
            double peak = 0.0;
            for (double v : s.v) peak = std::max(peak, std::abs(v));
            std::size_t i = 0;
            while (i < s.v.size() && std::abs(s.v[i]) < 0.98 * peak) ++i;
            while (i < s.v.size() && std::abs(s.v[i]) >= 0.98 * peak) ++i;
            s.t.resize(i);
            s.v.resize(i);
        }
        m = an::step_metrics(s.t, s.v);
    }
    if (!m.ok()) {
        ui::hint(m.error.c_str());
    } else if (ImGui::BeginTable("stepm", 3, ImGuiTableFlags_SizingStretchSame)) {
        auto stat = [](const char* l, Help h, const char* f, double v, Rgba c) {
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", l);
            ui::help(h);
            ImGui::PushFont(theme::fonts().mono, theme::kTitleSize);
            char b[32];
            std::snprintf(b, sizeof b, f, v);
            ImGui::TextColored(theme::v4(c), "%s", b);
            ImGui::PopFont();
        };
        stat("Rise", Help::StepRise, "%.3f s", m.rise_time_s, tok::text);
        stat("Settling", Help::StepSettling, "%.3f s", m.settling_time_s, tok::text);
        stat("Overshoot", Help::StepOvershoot, "%.1f %%", m.overshoot_pct,
             m.overshoot_pct > 20  ? tok::bad
             : m.overshoot_pct > 5 ? tok::warn
                                   : tok::good);
        ImGui::EndTable();
    }
    if (!s.t.empty() && ImPlot::BeginPlot("##step", {-1, -1}, ImPlotFlags_NoMenus)) {
        ImPlot::SetupAxes("time (s)", nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        ImPlot::SetupLegend(ImPlotLocation_NorthWest);
        ImPlotSpec sp;
        sp.LineColor = theme::v4(tok::accent);
        if (m.ok()) {
            const double band = std::abs(m.final_value - m.baseline) * 0.02;
            const double xs[2] = {s.t.front(), s.t.back()};
            const double hi[2] = {m.final_value + band, m.final_value + band},
                         lo[2] = {m.final_value - band, m.final_value - band};
            ImPlotSpec sb;
            sb.FillColor = theme::v4(tok::good, 0.20f);
            ImPlot::PlotShaded("2 % band", xs, lo, hi, 2, sb);
        }
        ImPlot::PlotLine(st.step_channel.c_str(), s.t.data(), s.v.data(),
                         static_cast<int>(s.t.size()), sp);
        if (m.ok()) {
            double t10 = m.t10_s, t90 = m.t90_s;
            ImPlot::DragLineX(10, &t10, theme::v4(tok::muted), 1.0f, ImPlotDragToolFlags_NoInputs);
            ImPlot::DragLineX(11, &t90, theme::v4(tok::muted), 1.0f, ImPlotDragToolFlags_NoInputs);
            // Labels inside the plot (axis tags would cover the tick labels).
            const double span = m.final_value - m.baseline;
            ImPlot::Annotation(m.t10_s, m.baseline + 0.2 * span, theme::v4(tok::raised),
                               {-22.0f, 0.0f}, true, "10%%");
            ImPlot::Annotation(m.t90_s, m.baseline + 0.6 * span, theme::v4(tok::raised),
                               {22.0f, 0.0f}, true, "90%%");
        }
        ImPlot::EndPlot();
    }
    ImGui::End();
}

void panel_imu(AppState& st) {
    if (!ImGui::Begin(win::kImu)) {
        ImGui::End();
        return;
    }
    if (!st.session) {
        ui::hint("No session.");
        ImGui::End();
        return;
    }
    const auto& rec = st.session->recorder();
    // Stationary samples only (|v| < 1 mm/s and |w| < 0.5 deg/s): noise, not motion.
    const auto *g = rec.channel("sim.gyro_z"), *v = rec.channel("sim.v"), *w = rec.channel("sim.w");
    std::vector<double> still;
    if (g && v && w)
        for (std::size_t i = 0; i < g->size(); ++i) {
            const double t = g->t[i];
            if (std::abs(v->value_at(t)) < 1.0 && std::abs(w->value_at(t)) < 0.5)
                still.push_back(g->v[i]);
        }
    const auto s = an::imu_stats(still);
    ui::value_row(Help::ImuSamples, "Stationary samples", "%d", s.n);
    if (s.mean) {
        ui::value_row(Help::ImuBias, "Bias (mean)", "%+.3f deg/s", *s.mean);
        ui::value_row(Help::ImuNoise, "Noise (1 sigma)", "%.3f deg/s", *s.stddev);
        ui::value_row(Help::ImuPeak, "Peak to peak", "%.3f deg/s", *s.p2p);
        ui::value_row(Help::ImuConfigured, "Configured bias", "%+.3f deg/s",
                      st.session->world().imu_model().gyro_bias_dps());
        if (ImPlot::BeginPlot("##imuh", {-1, -1}, ImPlotFlags_NoMenus | ImPlotFlags_NoLegend)) {
            ImPlot::SetupAxes("gyro z (deg/s)", "samples", ImPlotAxisFlags_AutoFit,
                              ImPlotAxisFlags_AutoFit);
            ImPlotSpec sp;
            sp.FillColor = theme::v4(tok::accent, 0.7f);
            sp.LineColor = theme::v4(tok::accent);
            ImPlot::PlotHistogram("gyro", still.data(), static_cast<int>(still.size()), 40, 1.0,
                                  ImPlotRange(), sp);
            const ImVec2 pp = ImPlot::GetPlotPos(), ps = ImPlot::GetPlotSize();
            ImPlot::EndPlot();
            ui::help_hotspot(pp.x + ps.x - 26.0f, pp.y + 4.0f, Help::ImuHistogram);
        }
    } else {
        ui::hint(
            "Needs the robot standing still for a moment (e.g. before GO, or while paused at the "
            "start).");
    }
    ImGui::End();
}

void panel_reports(AppState& st) {
    if (!ImGui::Begin(win::kReports)) {
        ImGui::End();
        return;
    }
    ui::hint(
        "Offline, printable engineering report (HTML): checks, metrics, plots and the event log.");
    ui::help(Help::ReportsExport);
    const float bw = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    if (ui::action_button("Save HTML report", ui::Tone::Active, !st.session, bw))
        st.actions.trigger("report");
    ImGui::SameLine();
    if (ui::action_button("MATLAB CSV bundle", ui::Tone::Neutral, !st.session, bw))
        st.actions.trigger("matlab");
    if (ui::action_button("All telemetry CSV", ui::Tone::Neutral, !st.session, bw))
        st.actions.trigger("csv");
    ImGui::SameLine();
    if (ui::action_button("Screenshot (F12)", ui::Tone::Neutral, false, bw))
        st.actions.trigger("screenshot");
    if (!st.report_status.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(theme::v4(tok::muted), "%s", st.report_status.c_str());
        ImGui::PopTextWrapPos();
    }
    if (!st.session) {
        ImGui::End();
        return;
    }
    // Live preview of the report's checks (same code path as the HTML).
    static an::ReportInput in;
    static double last = -1.0;
    if (ImGui::GetTime() - last > 0.5 || st.scripted) {
        in = report_input(st);
        last = ImGui::GetTime();
    }
    const std::string overall = an::overall_verdict(in.checks);
    ui::section("Checks", Help::ReportChecks);
    ImGui::TextDisabled("Overall");
    ImGui::SameLine();
    ui::pill(overall.c_str(), overall == "PASS"   ? ui::Tone::Good
                              : overall == "FAIL" ? ui::Tone::Bad
                                                  : ui::Tone::Neutral);
    if (ImGui::BeginTable("checks", 3, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("check", ImGuiTableColumnFlags_WidthStretch, 1.3f);
        ImGui::TableSetupColumn("verdict", ImGuiTableColumnFlags_WidthFixed,
                                ImGui::CalcTextSize("NOT MEASURED").x + 10.0f);
        ImGui::TableSetupColumn("measured", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        for (const auto& c : in.checks) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            {
                const std::string shown = ui::ellipsize(c.name, ImGui::GetContentRegionAvail().x);
                ImGui::TextUnformatted(shown.c_str());
                if (shown != c.name && ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", c.name.c_str());
            }
            ImGui::TableNextColumn();
            const ui::Tone t = c.verdict == "PASS"   ? ui::Tone::Good
                               : c.verdict == "FAIL" ? ui::Tone::Bad
                                                     : ui::Tone::Neutral;
            ImGui::TextColored(theme::v4(t == ui::Tone::Neutral ? tok::muted : ui::tone_color(t)),
                               "%s", c.verdict.c_str());
            ImGui::TableNextColumn();
            {
                const std::string shown =
                    ui::ellipsize(c.measured, ImGui::GetContentRegionAvail().x);
                ui::mono_colored(tok::muted, "%s", shown.c_str());
                if (shown != c.measured && ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", c.measured.c_str());
            }
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

void panel_record(AppState& st) {
    if (!ImGui::Begin(win::kRecord)) {
        ImGui::End();
        return;
    }
    if (st.maze) {
        const auto& m = *st.maze;
        ui::section("Maze");
        ui::value_row("File", "%s", st.maze_path.filename().string().c_str());
        ui::value_row("Size", "%d x %d", m.width(), m.height());
        if (st.flood)
            ui::value_row("Start to goal", "%d cells", st.flood->at(m.start().x, m.start().y));
        ui::value_row("Official rules", "%s", st.report.is_official() ? "yes" : "no");
    }
    if (st.session) {
        const auto& s = *st.session;
        ui::section("Replay record", Help::RunRecord);
        ui::value_row("Maze hash", "%016llx",
                      static_cast<unsigned long long>(s.world().maze().hash()));
        ui::value_row("Config", "%s", short_hash(st.config_hash).c_str());
        ui::value_row("Firmware", "%s", short_hash(s.firmware_hash()).c_str());
        ui::value_row("Seed", "%llu", static_cast<unsigned long long>(s.world().seed()));
        ui::value_row("Trajectory", "%s", short_hash(s.trajectory_hash()).c_str());
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", s.trajectory_hash().c_str());
        ui::hint("Same record + same time = same trajectory hash.");
    }
    ImGui::End();
}

}  // namespace mmsim::app
