// v0.2.0 panels: Attitude compass and Move & Firmware Details (Cockpit), live Robot config
// editor and Race timer (Tuning & Reports).
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "sim/app/attitude.hpp"
#include "sim/app/panels.hpp"
#include "sim/app/theme.hpp"
#include "sim/app/widgets.hpp"
#include "sim/core/sha256.hpp"

namespace mmsim::app {

namespace fs = std::filesystem;

namespace {

constexpr const char* kDeg = "\xC2\xB0";  // UTF-8 degree sign

ImVec2 at(const ImVec2& c, double bearing_deg_v, float r) {
    const double a = bearing_deg_v * kPiD / 180.0;
    return {c.x + static_cast<float>(std::sin(a)) * r, c.y - static_cast<float>(std::cos(a)) * r};
}

/// Crisp circular compass: rose ticks, N/E/S/W, heading needle and (optional) target tick.
void draw_compass(ImDrawList* dl, const ImVec2& c, float R, double bearing, const double* target) {
    dl->AddCircleFilled(c, R, theme::u32(tok::canvas), 72);
    dl->AddCircle(c, R, theme::u32(tok::hairline), 72, 1.5f);
    for (int d = 0; d < 360; d += 10) {
        const bool cardinal = d % 90 == 0, mid = d % 30 == 0;
        const float len = cardinal ? 0.20f : mid ? 0.13f : 0.07f;
        dl->AddLine(at(c, d, R * (1.0f - len)), at(c, d, R * 0.97f),
                    theme::u32(cardinal ? tok::text : tok::muted, cardinal ? 1.0f
                                                                  : mid    ? 0.8f
                                                                           : 0.45f),
                    cardinal ? 2.0f : 1.0f);
    }
    static const char* const names[4] = {"N", "E", "S", "W"};
    for (int i = 0; i < 4; ++i) {
        const ImVec2 p = at(c, i * 90.0, R * 0.66f);
        const ImVec2 ts = ImGui::CalcTextSize(names[i]);
        dl->AddText({p.x - ts.x * 0.5f, p.y - ts.y * 0.5f},
                    theme::u32(i == 0 ? tok::accent : tok::text, i == 0 ? 1.0f : 0.85f), names[i]);
    }
    if (target) {
        const ImVec2 tip = at(c, *target, R * 0.80f);
        const ImVec2 a = at(c, *target - 6.0, R * 1.0f), b = at(c, *target + 6.0, R * 1.0f);
        dl->AddTriangleFilled(tip, a, b, theme::u32(tok::warn));
    }
    // Heading needle: a kite with the tip toward the bearing, a muted tail behind the hub.
    const ImVec2 tip = at(c, bearing, R * 0.74f);
    const ImVec2 left = at(c, bearing - 90.0, R * 0.09f), right = at(c, bearing + 90.0, R * 0.09f);
    const ImVec2 tail = at(c, bearing + 180.0, R * 0.22f);
    dl->AddTriangleFilled(tip, left, right, theme::u32(tok::accent));
    dl->AddTriangleFilled(tail, left, right, theme::u32(tok::muted, 0.7f));
    dl->AddCircleFilled(c, R * 0.065f, theme::u32(tok::canvas));
    dl->AddCircle(c, R * 0.065f, theme::u32(tok::accent), 24, 1.5f);
}

}  // namespace

void panel_attitude(AppState& st) {
    if (!ImGui::Begin(win::kAttitude)) {
        ImGui::End();
        return;
    }
    if (!st.session) {
        ui::hint("No session. Load a maze and press GO.");
        ImGui::End();
        return;
    }
    const auto& w = st.session->world();
    const double theta = w.drive().pose.theta;
    const double bearing = bearing_deg(theta);
    const auto target_v = st.session->telemetry_value("hdg_target");
    double target_b = 0.0;
    const bool have_target = target_v.has_value();
    if (have_target) target_b = wrap_deg360(static_cast<double>(*target_v));
    const double gyro = w.imu().gyro_dps[2];

    const ImVec2 av = ImGui::GetContentRegionAvail();
    const bool side_by_side = av.x >= av.y * 1.35f;
    const float dia =
        std::clamp(side_by_side ? av.y - 8.0f : std::min(av.x - 8.0f, av.y * 0.62f), 90.0f, 190.0f);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 c{p0.x + dia * 0.5f + 2.0f, p0.y + dia * 0.5f + 2.0f};
    draw_compass(ImGui::GetWindowDrawList(), c, dia * 0.5f - 2.0f, bearing,
                 have_target ? &target_b : nullptr);
    ImGui::Dummy({dia + 4.0f, dia + 4.0f});

    if (side_by_side) ImGui::SetCursorScreenPos({p0.x + dia + 18.0f, p0.y + 2.0f});
    ImGui::BeginGroup();
    ImGui::TextDisabled("Heading");
    ui::help(Help::Attitude);
    ImGui::PushFont(theme::fonts().mono, theme::kTitleSize);
    ImGui::Text("%05.1f%s %s", bearing, kDeg, rose8(bearing));
    ImGui::PopFont();
    if (have_target) {
        ImGui::TextDisabled("Target");
        ImGui::SameLine();
        ui::mono_colored(tok::warn, "%05.1f%s %s", target_b, kDeg, rose8(target_b));
    } else {
        ImGui::TextDisabled("Target: not reported");
    }
    ImGui::TextDisabled("Gyro Z");
    ui::help(Help::GyroRate);
    ui::mono("%+.1f deg/s", gyro);
    ImGui::EndGroup();
    ImGui::End();
}

void panel_details(AppState& st) {
    if (!ImGui::Begin(win::kDetails, &st.details_open)) {
        ImGui::End();
        return;
    }
    // Toolbar: filter pills, search, auto-scroll, clear, copy. Wraps instead of clipping.
    const char* names[] = {"All", "Moves", "Firmware", "Errors"};
    for (int i = 0; i < 4; ++i) {
        if (i) ui::flow(ui::button_width(names[i]));
        if (ui::pill_toggle(names[i], static_cast<int>(st.details_filter) == i))
            st.details_filter = static_cast<DetailFilter>(i);
    }
    ui::help(Help::DetailsLog);
    const float sw = std::max(110.0f, ImGui::GetFontSize() * 9.0f);
    ui::flow(sw);
    ImGui::SetNextItemWidth(sw);
    ImGui::InputTextWithHint("##dsearch", "Search...", st.details_search, sizeof st.details_search);
    ui::flow(ImGui::GetFrameHeight() + ImGui::CalcTextSize("Auto-scroll").x + 8.0f);
    ImGui::Checkbox("Auto-scroll", &st.details_autoscroll);
    ui::flow(ui::button_width("Clear"));
    if (ImGui::Button("Clear")) clear_details(st);
    ui::flow(ui::button_width("Copy log"));
    if (ImGui::Button("Copy log")) {
        const std::string text = st.details.to_text(st.details_filter, st.details_search);
        ImGui::SetClipboardText(text.c_str());
        const auto lines = std::count(text.begin(), text.end(), '\n');
        toast(st, "Copied " + std::to_string(lines) + " log lines", ui::Tone::Good);
    }

    std::vector<const DetailEntry*> rows;
    rows.reserve(st.details.entries().size());
    for (const auto& e : st.details.entries())
        if (detail_matches(e, st.details_filter, st.details_search)) rows.push_back(&e);

    ImGui::BeginChild("##drows", {0, 0}, ImGuiChildFlags_None);
    if (rows.empty()) {
        ui::hint(!st.session                    ? "No session."
                 : st.details.entries().empty() ? "No details yet. Press GO."
                                                : "Nothing matches the filter or search.");
    }
    ImGuiListClipper clip;
    clip.Begin(static_cast<int>(rows.size()), ImGui::GetTextLineHeightWithSpacing());
    while (clip.Step()) {
        for (int i = clip.DisplayStart; i < clip.DisplayEnd; ++i) {
            const DetailEntry& e = *rows[static_cast<std::size_t>(i)];
            const Rgba col = (e.flags & kDetError)       ? tok::bad
                             : (e.flags & kDetMilestone) ? tok::good
                             : (e.flags & kDetMove)      ? tok::accent
                                                         : tok::muted;
            ImGui::PushID(i);
            const ImVec2 p = ImGui::GetCursorScreenPos();
            if (ImGui::Selectable("##d", false, ImGuiSelectableFlags_SpanAllColumns,
                                  {0, ImGui::GetTextLineHeight()}) &&
                st.session) {
                set_running(st, false);
                st.tv.scrub = true;
                st.tv.jump = true;
                st.tv.cursor_t = e.t_s;
            }
            const bool hovered = ImGui::IsItemHovered();
            ImGui::SetCursorScreenPos(p);
            ImGui::GetWindowDrawList()->AddCircleFilled(
                {p.x + 5.0f, p.y + ImGui::GetTextLineHeight() * 0.5f}, 3.5f, theme::u32(col));
            ImGui::SetCursorScreenPos({p.x + 16.0f, p.y});
            ui::mono_colored(tok::muted, "%9.3f", e.t_s);
            ImGui::SameLine();
            ui::mono_colored(col, "%s", detail_tag(e));
            ImGui::SameLine();
            const float room = ImGui::GetContentRegionAvail().x - 4.0f;
            const std::string shown = ui::ellipsize(e.text, room);
            ImGui::TextColored(
                theme::v4((e.flags & kDetFirmware) && !(e.flags & kDetError) ? tok::text : col),
                "%s", shown.c_str());
            if (hovered && shown != e.text) ImGui::SetTooltip("%s", e.text.c_str());
            ImGui::PopID();
        }
    }
    // Follow the newest line; scrolling up by hand turns following off.
    static std::size_t last_rows = 0;
    if (st.details_autoscroll && rows.size() != last_rows) ImGui::SetScrollHereY(1.0f);
    if (st.details_autoscroll && ImGui::IsWindowHovered() && ImGui::GetIO().MouseWheel > 0.0f)
        st.details_autoscroll = false;
    last_rows = rows.size();
    ImGui::EndChild();
    ImGui::End();
}

// --- Robot config editor -----------------------------------------------------------------------
namespace {

bool cfg_field(const char* label, Help h, double* v, double speed, double lo, double hi,
               const char* fmt) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ui::help(h, true);
    ImGui::TableNextColumn();
    ImGui::PushID(label);
    ImGui::SetNextItemWidth(-1);
    const bool changed =
        ImGui::DragScalar("##v", ImGuiDataType_Double, v, static_cast<float>(speed), &lo, &hi, fmt,
                          ImGuiSliderFlags_AlwaysClamp);
    ImGui::PopID();
    return changed;
}

bool cfg_toggle(const char* label, Help h, bool* v, const char* text) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(label);
    ui::help(h, true);
    ImGui::TableNextColumn();
    ImGui::PushID(label);
    const bool changed = ImGui::Checkbox(text, v);
    ImGui::PopID();
    return changed;
}

}  // namespace

void panel_config(AppState& st) {
    if (!ImGui::Begin(win::kConfig)) {
        ImGui::End();
        return;
    }
    if (!st.robot) {
        ui::hint("Robot config not loaded.");
        ImGui::End();
        return;
    }
    const fs::path copy_dir = st.user_dir / "config" / "robots";
    if (!st.cfg_loaded) {
        st.cfg_edit = edit_from_config(*st.robot);
        st.cfg_files = scan_robot_configs(st);
        const std::string base =
            st.config_path.empty() ? st.robot->name : st.config_path.stem().string();
        const std::string stem = unique_copy_stem(copy_dir, base);
        std::snprintf(st.cfg_name, sizeof st.cfg_name, "%s", stem.c_str());
        st.cfg_loaded = true;
    }
    ConfigEdit& e = st.cfg_edit;

    // Which config is loaded (switch = load that file, restart the run).
    ImGui::TextDisabled("Config");
    ui::help(Help::CfgFile);
    ImGui::SetNextItemWidth(-1);
    const std::string cur =
        st.config_path.filename().string() + (st.config_edited ? " (edited)" : "");
    if (ImGui::BeginCombo("##cfgfile", cur.c_str())) {
        for (const auto& f : st.cfg_files) {
            std::error_code ec;
            const bool sel = !st.config_edited && fs::equivalent(f, st.config_path, ec);
            const bool mine =
                f.parent_path() != (st.root / "config" / "robots") && st.user_dir != st.root;
            const std::string label = f.filename().string() + (mine ? "  (your copy)" : "");
            if (ImGui::Selectable(label.c_str(), sel)) {
                const std::string err = use_robot_config(st, f);
                st.config_edited = false;
                st.cfg_status = err.empty() ? "Loaded " + f.filename().string() : err;
                toast(st, st.cfg_status, err.empty() ? ui::Tone::Good : ui::Tone::Bad);
            }
        }
        ImGui::EndCombo();
    }

    if (ImGui::BeginTable("cfgt", 2, ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("k", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("v", ImGuiTableColumnFlags_WidthFixed,
                                std::max(110.0f, ImGui::GetFontSize() * 8.0f));
        cfg_field("Mass", Help::CfgMass, &e.mass_g, 0.5, 10.0, 1000.0, "%.0f g");
        cfg_field("Wheel radius", Help::CfgWheel, &e.wheel_radius_mm, 0.05, 5.0, 60.0, "%.2f mm");
        cfg_field("Track width", Help::CfgTrack, &e.track_width_mm, 0.1, 20.0, 200.0, "%.1f mm");
        cfg_field("Motor max voltage", Help::CfgVoltage, &e.max_voltage, 0.05, 0.5, 6.0, "%.2f V");
        cfg_toggle("Tyre slip model", Help::CfgSlip, &e.slip_enabled,
                   e.slip_enabled ? "On" : "Off");
        ImGui::BeginDisabled(!e.slip_enabled);
        cfg_field("Tyre mu", Help::CfgMu, &e.tyre_mu, 0.01, 0.2, 2.0, "%.2f");
        ImGui::EndDisabled();
        cfg_toggle("Battery model", Help::CfgBatteryModel, &e.battery_model,
                   e.battery_model ? "Sag on" : "Ideal supply");
        ImGui::BeginDisabled(!e.battery_model);
        cfg_field("Battery R_int", Help::CfgRint, &e.r_int_ohm, 0.005, 0.0, 2.0, "%.3f ohm");
        ImGui::EndDisabled();
        if (e.has_sensors) {
            cfg_field("Sensor range", Help::CfgSensorRange, &e.sensor_range_mm, 1.0, 20.0, 600.0,
                      "%.0f mm");
            cfg_field("Sensor noise (fixed)", Help::CfgSensorNoise, &e.noise_sigma, 0.1, 0.0, 200.0,
                      "%.1f");
            cfg_field("Sensor noise (relative)", Help::CfgSensorNoise, &e.noise_rel_pct, 0.05, 0.0,
                      50.0, "%.2f %%");
        }
        ImGui::EndTable();
    }

    const auto issues = validate_edit(e);
    const bool blocked = has_errors(issues);
    for (const auto& i : issues) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(theme::v4(i.level == IssueLevel::Error ? tok::bad : tok::warn), "%s",
                           i.text.c_str());
        ImGui::PopTextWrapPos();
    }

    if (ui::action_button("Apply & Restart", ui::Tone::Active, blocked)) {
        apply_edit(e, *st.robot);
        st.config_edited = true;
        st.config_hash = core::sha256_hex(core::robot_config_to_json(*st.robot));
        st.mounts.loaded = false;
        st.cfg_loaded = false;
        make_session(st);
        st.cfg_status = "Applied; run restarted";
        toast(st, "Robot config applied; run restarted", ui::Tone::Good);
    }
    ui::help(Help::CfgApply, true);
    ui::flow(ui::button_width("Save copy..."));
    static std::string popup_err;
    if (ImGui::Button("Save copy...")) {
        popup_err.clear();
        ImGui::OpenPopup("Save robot copy");
    }
    ui::help(Help::CfgSaveCopy, true);
    ui::flow(ui::button_width("Revert"));
    if (ImGui::Button("Revert")) st.cfg_loaded = false;
    if (!st.cfg_status.empty()) ui::hint(st.cfg_status.c_str());

    if (ImGui::BeginPopupModal("Save robot copy", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ui::hint("Saved as a new file. An existing file is never overwritten.");
        ImGui::TextDisabled("Folder");
        ui::mono("%s", copy_dir.string().c_str());
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 14.0f);
        ImGui::InputText("##cfgname", st.cfg_name, sizeof st.cfg_name);
        ImGui::SameLine();
        ImGui::TextDisabled(".json");
        if (!popup_err.empty()) ImGui::TextColored(theme::v4(tok::bad), "%s", popup_err.c_str());
        if (ui::action_button("Save", ui::Tone::Active, blocked || st.cfg_name[0] == '\0')) {
            core::RobotConfig c = *st.robot;
            apply_edit(e, c);
            fs::path out;
            if (save_config_copy(c, copy_dir, st.cfg_name, out, popup_err)) {
                st.cfg_files = scan_robot_configs(st);
                st.cfg_status = "Saved " + out.string();
                toast(st, "Saved " + out.filename().string(), ui::Tone::Good);
                std::snprintf(st.cfg_name, sizeof st.cfg_name, "%s",
                              unique_copy_stem(copy_dir, out.stem().string()).c_str());
                ImGui::CloseCurrentPopup();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::End();
}

// --- Race timer --------------------------------------------------------------------------------
void panel_timer(AppState& st) {
    if (!ImGui::Begin(win::kTimer)) {
        ImGui::End();
        return;
    }
    RaceTimer& t = st.timer;
    const bool locked = t.state() == TimerState::Running || t.state() == TimerState::Paused;
    const double left = t.remaining_s();
    const Rgba col = t.state() == TimerState::Expired || left <= 10.0 ? tok::bad
                     : left <= 30.0                                   ? tok::warn
                                                                      : tok::text;
    ImGui::PushFont(theme::fonts().mono, 30.0f);
    ImGui::TextColored(theme::v4(col), "%s", RaceTimer::format(left).c_str());
    ImGui::PopFont();
    ImGui::SameLine();
    const char* state = t.state() == TimerState::Running   ? "running"
                        : t.state() == TimerState::Paused  ? "paused"
                        : t.state() == TimerState::Expired ? "time up"
                                                           : "ready";
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("%s", state);
    ui::bar("##timerbar", static_cast<float>(t.duration_s() > 0 ? left / t.duration_s() : 0.0), "",
            col.packed() == tok::text.packed() ? tok::accent : col, 8.0f);

    ImGui::BeginDisabled(locked);
    for (int i = 0; i < 3; ++i) {
        char lbl[16];
        std::snprintf(lbl, sizeof lbl, "%d min", RaceTimer::kPresetMinutes[i]);
        if (i) ui::flow(ui::button_width(lbl));
        if (ui::pill_toggle(lbl, t.preset_minutes() == RaceTimer::kPresetMinutes[i]))
            t.set_preset_minutes(RaceTimer::kPresetMinutes[i]);
    }
    ImGui::EndDisabled();
    ui::help(Help::TimerPreset);
    ImGui::BeginDisabled(locked);
    if (ui::pill_toggle("Real time", t.basis() == TimerBasis::Real)) t.set_basis(TimerBasis::Real);
    ui::flow(ui::button_width("Sim time"));
    if (ui::pill_toggle("Sim time", t.basis() == TimerBasis::Sim)) t.set_basis(TimerBasis::Sim);
    ImGui::EndDisabled();
    ui::help(Help::TimerBasis);

    const bool running = t.state() == TimerState::Running;
    if (ui::action_button(
            running ? "Pause" : (t.state() == TimerState::Paused ? "Resume" : "Start"),
            running ? ui::Tone::Neutral : ui::Tone::Good)) {
        if (running) {
            t.pause();
        } else {
            t.start();
            st.timer_note.clear();
        }
    }
    ui::flow(ui::button_width("Reset"));
    if (ImGui::Button("Reset")) {
        t.reset();
        st.timer_note.clear();
    }
    ui::help(Help::TimerControls, true);
    ImGui::Checkbox("Start with GO", &st.timer_with_go);
    ui::flow(ImGui::GetFrameHeight() + ImGui::CalcTextSize("Tick sounds").x + 8.0f);
    ImGui::Checkbox("Tick sounds", &st.timer_audio);
    ui::help(Help::TimerAudio, true);
    if (!st.timer_note.empty())
        ImGui::TextColored(theme::v4(tok::warn), "%s", st.timer_note.c_str());

    ui::section("Attempts", Help::AttemptTable);
    const auto& rows = st.attempts.rows();
    if (rows.empty()) {
        ui::hint("No attempts yet. A run is logged when its speed run finishes.");
    } else if (ImGui::BeginTable("attempts", 5,
                                 ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp |
                                     ImGuiTableFlags_BordersInnerH)) {
        ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 26.0f);
        ImGui::TableSetupColumn("Search");
        ImGui::TableSetupColumn("Speed run");
        ImGui::TableSetupColumn("Crashes", ImGuiTableColumnFlags_WidthFixed,
                                ImGui::CalcTextSize("Crashes").x + 10.0f);
        ImGui::TableSetupColumn("Best");
        ImGui::TableHeadersRow();
        const int best_n = st.attempts.best_number();
        auto secs = [](double s) {
            char b[24];
            if (s < 0.0)
                std::snprintf(b, sizeof b, "-");
            else
                std::snprintf(b, sizeof b, "%.2f s", s);
            return std::string(b);
        };
        for (const auto& r : rows) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ui::mono("%d", r.number);
            ImGui::TableNextColumn();
            ui::mono("%s", secs(r.search_s).c_str());
            ImGui::TableNextColumn();
            if (r.finished)
                ui::mono_colored(r.number == best_n ? tok::good : tok::text, "%s",
                                 secs(r.speed_run_s).c_str());
            else
                ImGui::TextDisabled("%s", r.note.empty() ? "-" : r.note.c_str());
            ImGui::TableNextColumn();
            ui::mono_colored(r.crashes > 0 ? tok::warn : tok::muted, "%d", r.crashes);
            ImGui::TableNextColumn();
            ui::mono_colored(tok::muted, "%s", secs(r.best_so_far_s).c_str());
        }
        ImGui::EndTable();
    }
    if (const auto best = st.attempts.best_speed_run())
        ui::value_row("Best speed run", "%.2f s (attempt %d)", *best, st.attempts.best_number());
    ImGui::BeginDisabled(rows.empty());
    if (ImGui::Button("Copy table")) {
        ImGui::SetClipboardText(st.attempts.to_text().c_str());
        toast(st, "Attempt table copied", ui::Tone::Good);
    }
    ui::flow(ui::button_width("Clear attempts"));
    if (ImGui::Button("Clear attempts")) st.attempts.clear();
    ImGui::EndDisabled();
    ImGui::End();
}

}  // namespace mmsim::app
