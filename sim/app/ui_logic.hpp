#pragma once
// Pure UI decisions (no ImGui): workspaces, audience mode, the plain-English mission status,
// top-bar fitting and small formatters. Unit-tested under the "ui" label.

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mmsim::app::ui {

enum class Tone { Neutral, Active, Good, Warn, Bad };

enum class Workspace { Cockpit = 0, Studio = 1, Telemetry = 2, Tuning = 3 };
inline constexpr int kWorkspaceCount = 4;

/// "cockpit" | "studio" | "telemetry" | "tuning" (also "1".."4", "maze", "compare",
/// "reports"); case-insensitive. nullopt if unknown.
std::optional<Workspace> parse_workspace(std::string_view s);
const char* workspace_id(Workspace w);     // "cockpit"
const char* workspace_title(Workspace w);  // "Cockpit", "Maze Studio", ...

enum class Mode { Beginner, Engineer };
std::optional<Mode> parse_mode(std::string_view s);
const char* mode_id(Mode m);

/// Everything the status pill needs to know, gathered by the app each frame.
struct MissionInput {
    bool have_maze = false;
    bool have_source = false;  // firmware DLL or mms solver selected
    bool load_error = false;   // DLL / solver failed to load
    bool faulted = false;      // firmware fault, hang or solver crash
    bool running = false;
    std::int64_t time_us = 0;
    std::int64_t last_crash_us = -1;  // most recent wall contact, -1 = none
    std::vector<std::string> logs;    // firmware log lines, oldest first
};

struct MissionStatus {
    std::string text;  // "Ready", "Exploring maze", "Goal reached!", "Speed run", "Crashed"
    Tone tone = Tone::Neutral;
    std::string detail;  // one short sentence for the tooltip / faceplate
};

/// Plain-English status. Phases come from the demo firmware's log protocol ("goal reached",
/// "start reached", "speed run start", "speed run done", "ERROR ..."); a wall hit in the last
/// 0.75 s of simulated time shows "Crashed".
MissionStatus mission_status(const MissionInput& in);

/// How many of `widths` (left to right) fit in `avail` with `spacing` between items.
int fit_count(const std::vector<float>& widths, float avail, float spacing);

/// Window placement in physical pixels. `usable` is the display's work area (no taskbar),
/// `border_*` the native frame around the client area (titlebar = top).
struct ScreenRect {
    int x = 0, y = 0, w = 0, h = 0;
};
struct FrameBorders {
    int top = 36, left = 8, bottom = 8, right = 8;  // Windows 11 fallback
};
struct WindowPlacement {
    ScreenRect client;      // client-area rect to set (position = client origin)
    bool maximize = false;  // the request did not fit: start maximized
    int min_w = 0, min_h = 0;
};
/// Fit a requested client size (already DPI-scaled) inside the usable area so the whole
/// frame, titlebar included, is on screen; centred; min size `min_w x min_h`, never larger
/// than what fits.
WindowPlacement fit_window(int want_w, int want_h, const ScreenRect& usable, const FrameBorders& b,
                           int min_w, int min_h);

/// "00:12.345" (minutes:seconds.millis); hours are folded into minutes.
std::string format_sim_time(std::int64_t time_us);

/// Speed pills: 1x, 2x, 5x, Fast (0 = as fast as possible).
struct SpeedPreset {
    const char* label;
    double speed;
};
const std::vector<SpeedPreset>& speed_presets();
/// Index of the preset matching `speed`, -1 if none (a speed set from the console).
int speed_preset_index(double speed);

/// Sensor reading as a 0..1 bar: IR = raw / adc_max (bright = close), ToF = 1 - mm / range.
float sensor_bar(bool is_ir, double raw, double adc_max, double mm, double range_mm);

/// A firmware log line classified for the event list.
enum class EventKind { Info, Milestone, Error };
EventKind classify_log(std::string_view line);

}  // namespace mmsim::app::ui
