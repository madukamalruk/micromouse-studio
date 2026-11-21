#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace mmsim::app {

/// GUI session state restored at the next start (Slice 8, PRD F17). Stored as JSON in
/// `<root>/mmsim.json` next to ImGui's `imgui.ini` (both gitignored). Command-line arguments
/// win over saved values; scripted runs (--frames/--exit) neither read nor write the file.
struct AppSettings {
    std::filesystem::path maze;
    std::filesystem::path config;
    std::filesystem::path firmware;
    std::filesystem::path mms_solver;
    bool mms_sensor_walls = false;
    std::uint64_t seed = 1;
    double speed = 1.0;  // 0 = max
    bool boot_buttons[2] = {false, false};
    // Slice 9.10 UI: workspace (cockpit/studio/telemetry/tuning), mode (beginner/engineer), audio.
    std::string workspace = "cockpit";
    std::string mode = "engineer";
    bool muted = false;
    // v0.2.0: layouts changed; a saved session from an older version rebuilds them once.
    int layout_version = 0;
    bool timer_audio = true;
    bool details_open = true;
    int timer_minutes = 5;
};
inline constexpr int kLayoutVersion = 2;

/// Missing file -> defaults, true. Unreadable/corrupt file -> defaults, false + error (the
/// GUI starts anyway). Unknown keys are ignored; wrong types fall back to the default.
bool load_settings(const std::filesystem::path& file, AppSettings& out, std::string& error);
/// Writes atomically (temp file + rename). false + error on failure.
bool save_settings(const std::filesystem::path& file, const AppSettings& s, std::string& error);

}  // namespace mmsim::app
