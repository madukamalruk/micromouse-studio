#include "sim/app/ui_logic.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace mmsim::app::ui {

namespace {
std::string lower(std::string_view s) {
    std::string out(s);
    for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return out;
}
bool contains(std::string_view hay, std::string_view needle) {
    return lower(hay).find(lower(needle)) != std::string::npos;
}
}  // namespace

std::optional<Workspace> parse_workspace(std::string_view s) {
    const std::string l = lower(s);
    if (l == "cockpit" || l == "1") return Workspace::Cockpit;
    if (l == "studio" || l == "maze" || l == "maze-studio" || l == "2") return Workspace::Studio;
    if (l == "telemetry" || l == "compare" || l == "3") return Workspace::Telemetry;
    if (l == "tuning" || l == "reports" || l == "4") return Workspace::Tuning;
    return std::nullopt;
}

const char* workspace_id(Workspace w) {
    switch (w) {
        case Workspace::Cockpit:
            return "cockpit";
        case Workspace::Studio:
            return "studio";
        case Workspace::Telemetry:
            return "telemetry";
        case Workspace::Tuning:
            return "tuning";
    }
    return "cockpit";
}

const char* workspace_title(Workspace w) {
    switch (w) {
        case Workspace::Cockpit:
            return "Cockpit";
        case Workspace::Studio:
            return "Maze Studio";
        case Workspace::Telemetry:
            return "Telemetry & Compare";
        case Workspace::Tuning:
            return "Tuning & Reports";
    }
    return "Cockpit";
}

std::optional<Mode> parse_mode(std::string_view s) {
    const std::string l = lower(s);
    if (l == "beginner" || l == "simple") return Mode::Beginner;
    if (l == "engineer" || l == "expert") return Mode::Engineer;
    return std::nullopt;
}

const char* mode_id(Mode m) {
    return m == Mode::Beginner ? "beginner" : "engineer";
}

EventKind classify_log(std::string_view line) {
    if (contains(line, "error") || contains(line, "fault") || contains(line, "unreachable"))
        return EventKind::Error;
    if (contains(line, "goal reached") || contains(line, "start reached") ||
        contains(line, "speed run"))
        return EventKind::Milestone;
    return EventKind::Info;
}

MissionStatus mission_status(const MissionInput& in) {
    if (!in.have_maze) return {"Pick a maze", Tone::Neutral, "Choose a maze from the maze menu."};
    if (in.load_error)
        return {"Firmware error", Tone::Bad, "The firmware could not be loaded. See the log."};
    if (in.faulted)
        return {"Crashed", Tone::Bad, "The firmware stopped responding. Press RESET to retry."};
    if (!in.have_source)
        return {"Ready", Tone::Neutral, "No firmware selected. Pick one to drive the mouse."};

    // Latest phase announced by the firmware.
    enum class Phase { None, Explore, Goal, Back, SpeedRun, Done, Error } phase = Phase::None;
    for (const auto& l : in.logs) {
        if (contains(l, "speed run done"))
            phase = Phase::Done;
        else if (contains(l, "speed run start"))
            phase = Phase::SpeedRun;
        else if (contains(l, "start reached"))
            phase = Phase::Back;
        else if (contains(l, "goal reached"))
            phase = Phase::Goal;
        else if (classify_log(l) == EventKind::Error)
            phase = Phase::Error;
        else if (phase == Phase::None)
            phase = Phase::Explore;
    }
    const bool recent_crash =
        in.last_crash_us >= 0 && in.time_us - in.last_crash_us <= 750000 && in.time_us > 0;
    if (recent_crash) return {"Crashed", Tone::Bad, "The mouse hit a wall."};
    switch (phase) {
        case Phase::Error:
            return {"Stuck", Tone::Bad, "The firmware reported an error. See the event log."};
        case Phase::Done:
            return {"Run complete", Tone::Good, "Speed run finished. Compare it in Telemetry."};
        case Phase::SpeedRun:
            return {"Speed run", Tone::Active, "Racing the best known route."};
        case Phase::Back:
            return {in.running ? "Returning" : "Back at start", Tone::Active,
                    "Exploration done; back at the start cell."};
        case Phase::Goal:
            return {"Goal reached!", Tone::Good, "The mouse found the centre."};
        case Phase::Explore:
        case Phase::None:
            break;
    }
    if (in.running)
        return {"Exploring maze", Tone::Active, "Mapping walls and searching for the goal."};
    if (in.time_us > 0) return {"Paused", Tone::Warn, "Press GO (Space) to continue."};
    return {"Ready", Tone::Neutral, "Press GO (Space) to start."};
}

int fit_count(const std::vector<float>& widths, float avail, float spacing) {
    float used = 0.0f;
    int n = 0;
    for (float w : widths) {
        const float need = used + (n > 0 ? spacing : 0.0f) + w;
        if (need > avail) break;
        used = need;
        ++n;
    }
    return n;
}

FirmwareInfo firmware_info(std::string_view stem) {
    struct Known {
        const char *stem, *label, *short_label, *blurb;
    };
    static const Known kKnown[] = {
        {"demo_floodfill", "Flood-Fill", "Flood-Fill",
         "Counts cells to the goal and always steps downhill. The classic."},
        {"demo_dijkstra", "Turn-Weighted Dijkstra", "Dijkstra",
         "Prices every 90 degree turn, so it prefers straighter, faster routes."},
        {"demo_astar", "Dynamic A*", "A*",
         "Plans one optimal path and replans only when a newly found wall blocks it."},
        {"demo_tremaux",
         "Tr\xc3\xa9"
         "maux",
         "Tr\xc3\xa9"
         "maux",
         "Marks passages with chalk, copes with loops, then returns the short way."},
        {"demo_wallfollower", "Wall Follower", "Wall Follower",
         "Left hand on the wall. The baseline: fails when the goal is an island."},
    };
    FirmwareInfo fi;
    int i = 0;
    for (const Known& k : kKnown) {
        if (stem == k.stem) {
            fi.label = k.label;
            fi.short_label = k.short_label;
            fi.blurb = k.blurb;
            fi.group = 0;
            fi.order = i;
            return fi;
        }
        ++i;
    }
    fi.label = fi.short_label = std::string(stem);
    if (stem.rfind("test_", 0) == 0) {
        fi.group = 2;
        fi.blurb = "Development fixture, not a solver.";
    } else if (stem.rfind("demo_", 0) == 0) {
        fi.group = 0;
        fi.blurb = "Demo firmware.";
    } else {
        fi.group = 1;
        fi.blurb = "Custom firmware.";
    }
    return fi;
}

std::vector<std::size_t> firmware_menu_order(const std::vector<std::string>& stems) {
    std::vector<std::size_t> idx(stems.size());
    for (std::size_t i = 0; i < idx.size(); ++i) idx[i] = i;
    std::stable_sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) {
        const FirmwareInfo fa = firmware_info(stems[a]), fb = firmware_info(stems[b]);
        const int ga = fa.group == 1 ? 0 : fa.group, gb = fb.group == 1 ? 0 : fb.group;
        if (ga != gb) return ga < gb;  // solvers and custom firmware share the top group
        if (fa.group != fb.group) return fa.group < fb.group;
        if (fa.order != fb.order) return fa.order < fb.order;
        return stems[a] < stems[b];
    });
    return idx;
}

std::optional<std::size_t> match_firmware(const std::vector<std::string>& stems,
                                          std::string_view query) {
    auto norm = [](std::string_view s) {
        std::string o;
        for (const char c : s)
            if (c != ' ' && c != '-' && c != '_')
                o += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return o;
    };
    const std::string q = norm(query);
    if (q.empty()) return std::nullopt;
    for (std::size_t i = 0; i < stems.size(); ++i)
        if (stems[i] == query) return i;
    for (const std::size_t i : firmware_menu_order(stems)) {
        const FirmwareInfo fi = firmware_info(stems[i]);
        if (norm(stems[i]).find(q) != std::string::npos ||
            norm(fi.label).find(q) != std::string::npos ||
            norm(fi.short_label).find(q) != std::string::npos)
            return i;
    }
    return std::nullopt;
}

WindowPlacement fit_window(int want_w, int want_h, const ScreenRect& usable, const FrameBorders& b,
                           int min_w, int min_h) {
    WindowPlacement p;
    const int max_w = std::max(1, usable.w - b.left - b.right);
    const int max_h = std::max(1, usable.h - b.top - b.bottom);
    p.min_w = std::min(min_w, max_w);
    p.min_h = std::min(min_h, max_h);
    p.maximize = want_w > max_w || want_h > max_h;
    p.client.w = std::clamp(want_w, p.min_w, max_w);
    p.client.h = std::clamp(want_h, p.min_h, max_h);
    p.client.x = usable.x + b.left + (max_w - p.client.w) / 2;
    p.client.y = usable.y + b.top + (max_h - p.client.h) / 2;
    return p;
}

std::string format_sim_time(std::int64_t time_us) {
    if (time_us < 0) time_us = 0;
    const std::int64_t ms = time_us / 1000;
    const long long minutes = static_cast<long long>(ms / 60000);
    const int sec = static_cast<int>((ms / 1000) % 60);
    const int milli = static_cast<int>(ms % 1000);
    char b[32];
    std::snprintf(b, sizeof b, "%02lld:%02d.%03d", minutes, sec, milli);
    return b;
}

const std::vector<SpeedPreset>& speed_presets() {
    static const std::vector<SpeedPreset> p = {
        {"1x", 1.0}, {"2x", 2.0}, {"5x", 5.0}, {"Fast", 0.0}};
    return p;
}

int speed_preset_index(double speed) {
    const auto& p = speed_presets();
    for (std::size_t i = 0; i < p.size(); ++i)
        if ((p[i].speed <= 0.0 && speed <= 0.0) ||
            (p[i].speed > 0.0 && std::abs(p[i].speed - speed) < 1e-9))
            return static_cast<int>(i);
    return -1;
}

float sensor_bar(bool is_ir, double raw, double adc_max, double mm, double range_mm) {
    double f = 0.0;
    if (is_ir)
        f = adc_max > 0.0 ? raw / adc_max : 0.0;
    else
        f = (range_mm > 0.0 && mm >= 0.0) ? 1.0 - mm / range_mm : 0.0;
    return static_cast<float>(std::clamp(f, 0.0, 1.0));
}

}  // namespace mmsim::app::ui
