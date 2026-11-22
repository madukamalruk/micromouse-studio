// Cockpit: virtual robot faceplate, live sensor bars, event log.
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "sim/app/panels.hpp"
#include "sim/app/theme.hpp"
#include "sim/app/widgets.hpp"

namespace mmsim::app {

namespace {

const char* heading_name(double theta) {
    static const char* n[] = {"E", "N", "W", "S"};
    const double q = std::fmod(std::fmod(theta / (3.14159265358979 / 2.0), 4.0) + 4.0 + 0.5, 4.0);
    return n[static_cast<int>(q) % 4];
}

/// OLED-style status glass: three mono lines on a near-black panel.
void oled(const AppState& st, const ui::MissionStatus& ms) {
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::PushFont(theme::fonts().mono, theme::kOledSize);
    const float lh = ImGui::GetTextLineHeight();
    const float h = lh * 3.0f + 20.0f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, {p.x + w, p.y + h}, theme::u32(tok::oled), 6.0f);
    dl->AddRect(p, {p.x + w, p.y + h}, theme::u32(tok::hairline), 6.0f);
    char l1[64], l2[64], l3[64];
    std::string mode = st.mms_solver.empty() ? "FW" : "MMS";
    std::snprintf(l1, sizeof l1, "%-4s %s", mode.c_str(), ms.text.c_str());
    if (st.session) {
        const auto& d = st.session->world().drive();
        const auto c = cell_of(d.pose, st.dims);
        std::snprintf(l2, sizeof l2, "CELL %2d,%-2d  %s", c.x, c.y, heading_name(d.pose.theta));
        std::snprintf(l3, sizeof l3, "%s  %4.2fm/s",
                      ui::format_sim_time(st.session->time_us()).c_str(), d.v);
    } else {
        std::snprintf(l2, sizeof l2, "CELL --,--");
        std::snprintf(l3, sizeof l3, "00:00.000");
    }
    const ImU32 col = theme::u32(tok::oled_text);
    dl->AddText({p.x + 12.0f, p.y + 10.0f}, col, l1);
    dl->AddText({p.x + 12.0f, p.y + 10.0f + lh}, theme::u32(tok::oled_text, 0.8f), l2);
    dl->AddText({p.x + 12.0f, p.y + 10.0f + lh * 2}, theme::u32(tok::oled_text, 0.8f), l3);
    ImGui::PopFont();
    ImGui::Dummy({w, h});
}

}  // namespace

void panel_faceplate(AppState& st) {
    if (!ImGui::Begin(win::kFaceplate)) {
        ImGui::End();
        return;
    }
    const bool beginner = st.mode == ui::Mode::Beginner;
    const auto ms = current_status(st);
    oled(st, ms);
    ui::hint(ms.detail.c_str());
    if (!beginner) ui::help(Help::OledDisplay);

    // LEDs + the two buttons on one row (hold a button = pressed).
    if (beginner)
        ui::section("Lights & buttons");
    else
        ui::section("LEDs & buttons", Help::Leds);
    const Rgba led_col[] = {tok::good, tok::accent, tok::warn, tok::bad};
    const float fh = ImGui::GetFrameHeight();
    const float r = fh * 0.22f;
    const float y0 = ImGui::GetCursorPosY();
    for (int i = 0; i < 4; ++i) {
        if (i) ImGui::SameLine(0, 2.0f);
        ImGui::SetCursorPosY(y0 + fh * 0.5f - r * 1.8f);
        char id[16];
        std::snprintf(id, sizeof id, "##led%d", i);
        ui::led(id, st.session && st.session->led(i), led_col[i], r);
    }
    ImGui::SameLine(0, 12.0f);
    ImGui::SetCursorPosY(y0);
    const float b2 = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    for (int i = 0; i < 2; ++i) {
        if (i) ImGui::SameLine();
        char label[24];
        std::snprintf(label, sizeof label, "Button %d", i);
        ImGui::Button(label, {b2, 0});
        const bool held = ImGui::IsItemActive();
        if (st.session && held != st.held_buttons[i])
            st.session->set_button(i, held || st.boot_buttons[i]);
        st.held_buttons[i] = held;
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal))
            ImGui::SetTooltip("Hold to press button %d on the robot", i);
    }
    if (!beginner) {
        bool changed = ImGui::Checkbox("Hold B0 at boot", &st.boot_buttons[0]);
        ui::flow(ImGui::GetFrameHeight() + ImGui::CalcTextSize("Hold B1 at boot").x + 8.0f);
        changed |= ImGui::Checkbox("Hold B1 at boot", &st.boot_buttons[1]);
        ui::help(Help::Buttons, true);
        if (changed) make_session(st);
    }

    // PWM duty bars (signed, from the centre).
    if (beginner)
        ui::section("Motor power");
    else
        ui::section("Motor PWM duty", Help::MotorPwm);
    if (st.session) {
        const auto& w = st.session->world();
        const double vb = std::max(0.1, w.battery_voltage());
        const double cap = core::effective_max_voltage(w.config().motor);
        const float lw = ImGui::CalcTextSize("R_PWM").x + 14.0f;
        for (int side = 0; side < 2; ++side) {
            const double v = w.motor_voltage(side);
            const float duty = static_cast<float>(v / vb);
            char txt[48];
            std::snprintf(txt, sizeof txt, "%+4.0f %%  %+5.2f V", duty * 100.0, v);
            const bool at_cap = std::abs(v) >= cap - 1e-3;
            ui::labeled_bar(
                beginner ? (side == 0 ? "Left" : "Right") : (side == 0 ? "L_PWM" : "R_PWM"), lw,
                side == 0 ? "##pwm_l" : "##pwm_r", duty, txt, at_cap ? tok::warn : tok::accent,
                true);
        }
        if (!beginner) {
            ImGui::TextDisabled("Cap %.1f V = %.0f %% of the %.2f V pack", cap, 100.0 * cap / vb,
                                vb);
            ui::help(Help::MotorCap);
        }
    } else {
        ui::hint("No session.");
    }

    // Transport (mirrors the top bar).
    ui::section("Control");
    const float bw =
        (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 2) / 3.0f;
    const bool have = st.session && st.session->state() != host::FwState::LoadError;
    if (ui::action_button("GO", ui::Tone::Good, !have || st.running, bw))
        st.actions.trigger("go");
    ImGui::SameLine();
    if (ui::action_button("STOP", ui::Tone::Bad, !st.running, bw)) st.actions.trigger("stop");
    ImGui::SameLine();
    if (ui::action_button("RESET", ui::Tone::Neutral, !st.maze, bw)) st.actions.trigger("reset");
    if (beginner && have && !st.running && st.session->time_us() == 0)
        ui::hint("Press GO (or Space) to start. Use 2D / 2.5D at top-right.");
    ImGui::End();
}

void panel_sensors(AppState& st) {
    if (!ImGui::Begin(win::kSensors)) {
        ImGui::End();
        return;
    }
    if (!st.session) {
        ui::hint("No session. Load a maze.");
        ImGui::End();
        return;
    }
    const bool beginner = st.mode == ui::Mode::Beginner;
    const auto& w = st.session->world();
    const auto& ss = w.sensors();
    if (!beginner) ui::section("Distance sensors", Help::SensorBars);
    static const char* plain[] = {"Left", "Front L", "Front R", "Right"};
    float name_w = 0.0f;
    for (std::size_t ch = 0; ch < ss.size(); ++ch)
        name_w = std::max(
            name_w,
            ImGui::CalcTextSize(beginner && ch < 4 ? plain[ch] : ss.config(ch).name.c_str()).x);
    ImGui::PushFont(theme::fonts().mono, 0.0f);
    const float val_w = ImGui::CalcTextSize("no wall").x;
    ImGui::PopFont();
    const float fh = ImGui::GetFrameHeight();
    for (std::size_t ch = 0; ch < ss.size(); ++ch) {
        const auto& cfg = ss.config(ch);
        const bool ir = cfg.type == core::SensorType::IR;
        const auto mm = ss.distance_mm(ch);
        const bool none = mm == core::kDistanceNone;
        const double range_mm = cfg.range_max * 1000.0;
        const float f =
            ui::sensor_bar(ir, ss.raw(ch), cfg.ir_adc_max, none ? range_mm : mm, range_mm);
        // Row: name | bar (raw + normalised inside) | distance.
        const ImVec2 row = ImGui::GetCursorScreenPos();
        const float x0 = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMin().x;
        const float x1 = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const float ty = row.y + (fh - ImGui::GetTextLineHeight()) * 0.5f;
        dl->AddText({x0, ty}, theme::u32(tok::text),
                    beginner && ch < 4 ? plain[ch] : cfg.name.c_str());
        char right[32];
        if (none)
            std::snprintf(right, sizeof right, "no wall");
        else
            std::snprintf(right, sizeof right, "%u mm", static_cast<unsigned>(mm));
        ImGui::PushFont(theme::fonts().mono, 0.0f);
        const float tw = ImGui::CalcTextSize(right).x;
        dl->AddText({x1 - tw, ty}, theme::u32(none ? tok::muted : tok::text), right);
        ImGui::PopFont();
        char txt[48];
        if (beginner)
            std::snprintf(txt, sizeof txt, "%.0f %%", f * 100.0);
        else
            std::snprintf(txt, sizeof txt, "raw %4u  %.2f%s", static_cast<unsigned>(ss.raw(ch)), f,
                          w.emitter(static_cast<int>(ch)) ? "" : " off");
        char id[24];
        std::snprintf(id, sizeof id, "##sens%zu", ch);
        ImGui::SetCursorScreenPos({x0 + name_w + 12.0f, row.y + fh * 0.1f});
        const float bar_w = std::max(28.0f, x1 - (x0 + name_w + 12.0f) - val_w - 12.0f);
        ImGui::PushItemWidth(bar_w);
        ui::bar(id, f, txt, f > 0.75f ? tok::warn : tok::accent, fh * 0.8f, false, bar_w);
        ImGui::PopItemWidth();
        // Row pitch = frame height + 4 px, so four sensors fit a 720 px tall window.
        ImGui::SetCursorScreenPos({x0, row.y});
        ImGui::Dummy({1.0f, fh + 4.0f - ImGui::GetStyle().ItemSpacing.y});
    }
    if (ss.size() == 0) ui::hint("This robot config has no distance sensors.");
    if (!beginner) {
        ui::section("IMU & encoders", Help::ImuEncoders);
        ui::value_row("Gyro z", "%+7.2f deg/s", w.imu().gyro_dps[2]);
        ui::value_row("Accel x / y", "%+6.2f %+6.2f m/s2", w.imu().accel_mps2[0],
                      w.imu().accel_mps2[1]);
        ui::value_row("Encoders L / R", "%ld %ld", static_cast<long>(w.encoder_count(0)),
                      static_cast<long>(w.encoder_count(1)));
    }
    ImGui::End();
}

void panel_events(AppState& st) {
    if (!ImGui::Begin(win::kEvents)) {
        ImGui::End();
        return;
    }
    static int filter = 0;  // 0 all, 1 milestones, 2 problems
    const bool beginner = st.mode == ui::Mode::Beginner;
    if (beginner) filter = filter == 0 ? 1 : filter;
    const char* names[] = {"All", "Milestones", "Problems"};
    for (int i = beginner ? 1 : 0; i < 3; ++i) {
        if (i > (beginner ? 1 : 0)) ImGui::SameLine();
        if (ui::pill_toggle(names[i], filter == i)) filter = i;
    }
    ui::help(Help::Events);
    if (!beginner && !st.details_open) {
        ui::flow(ui::button_width("Open details log"));
        if (ImGui::Button("Open details log")) st.actions.trigger("details");
    }
    struct Row {
        double t;
        ui::EventKind kind;
        std::string text;
    };
    std::vector<Row> rows;
    if (st.session) {
        for (const auto& l : st.session->logs())
            rows.push_back(
                {static_cast<double>(l.time_us) * 1e-6, ui::classify_log(l.text), l.text});
        for (const auto& c : st.session->world().crashes()) {
            char b[64];
            std::snprintf(b, sizeof b, "Wall hit at %.2f m/s", c.impact_speed);
            rows.push_back({static_cast<double>(c.time_us) * 1e-6, ui::EventKind::Error, b});
        }
        std::stable_sort(rows.begin(), rows.end(),
                         [](const Row& a, const Row& b) { return a.t < b.t; });
    }
    if (!st.sim_error.empty()) rows.push_back({-1.0, ui::EventKind::Error, st.sim_error});
    ImGui::BeginChild("rows", {0, 0}, ImGuiChildFlags_None);
    int shown = 0;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const auto& r = rows[i];
        if (filter == 1 && r.kind == ui::EventKind::Info) continue;
        if (filter == 2 && r.kind != ui::EventKind::Error) continue;
        ++shown;
        const Rgba c = r.kind == ui::EventKind::Error       ? tok::bad
                       : r.kind == ui::EventKind::Milestone ? tok::good
                                                            : tok::muted;
        ImGui::PushID(static_cast<int>(i));
        const ImVec2 p = ImGui::GetCursorScreenPos();
        if (ImGui::Selectable("##ev", false, ImGuiSelectableFlags_SpanAllColumns,
                              {0, ImGui::GetTextLineHeight()}) &&
            r.t >= 0 && st.session) {
            set_running(st, false);
            st.tv.scrub = true;
            st.tv.jump = true;
            st.tv.cursor_t = r.t;
        }
        const bool row_hovered = ImGui::IsItemHovered();
        ImGui::SetCursorScreenPos(p);
        ImGui::GetWindowDrawList()->AddCircleFilled(
            {p.x + 5.0f, p.y + ImGui::GetTextLineHeight() * 0.5f}, 3.5f, theme::u32(c));
        ImGui::SetCursorScreenPos({p.x + 16.0f, p.y});
        if (r.t >= 0)
            ui::mono_colored(tok::muted, "%9.3f", r.t);
        else
            ui::mono_colored(tok::muted, "%9s", "--");
        ImGui::SameLine();
        // Long lines end in "..." instead of being cut off; the full text is in the tooltip.
        std::string shown_text = r.text;
        const float room = ImGui::GetContentRegionAvail().x - 4.0f;
        const bool cut = ImGui::CalcTextSize(shown_text.c_str()).x > room;
        if (cut) {
            while (!shown_text.empty() &&
                   ImGui::CalcTextSize((shown_text + "...").c_str()).x > room)
                shown_text.pop_back();
            shown_text += "...";
        }
        ImGui::TextColored(theme::v4(r.kind == ui::EventKind::Info ? tok::text : c), "%s",
                           shown_text.c_str());
        if (cut && row_hovered) ImGui::SetTooltip("%s", r.text.c_str());
        ImGui::PopID();
    }
    if (shown == 0) ui::hint(st.session ? "No events yet. Press GO." : "No session.");
    if (st.running && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4)
        ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();
    ImGui::End();
}

}  // namespace mmsim::app
