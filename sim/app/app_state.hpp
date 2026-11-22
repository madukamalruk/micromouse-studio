#pragma once
// GUI application state and the operations every panel shares (load a maze, (re)create the
// session, advance / rewind, export). Split out of main.cpp in Slice 9.10.

#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "sim/app/actions.hpp"
#include "sim/app/camera.hpp"
#include "sim/app/config_edit.hpp"
#include "sim/app/details_log.hpp"
#include "sim/app/maze_view.hpp"
#include "sim/app/race_timer.hpp"
#include "sim/app/robot_view.hpp"
#include "sim/app/telemetry_view.hpp"
#include "sim/app/ui_logic.hpp"
#include "sim/core/analysis/battery.hpp"
#include "sim/core/analysis/report.hpp"
#include "sim/core/analysis/runs.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/maze_geometry.hpp"
#include "sim/core/maze_validate.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/host/mms_bridge.hpp"
#include "sim/host/pacer.hpp"
#include "sim/host/session.hpp"

namespace mmsim::app {

struct MazeGroup {
    std::string label;
    std::vector<std::filesystem::path> files;
};

enum class HeatLayer { Off, PeakSpeed, GripUse, Potential };

struct Toast {
    std::string text;
    ui::Tone tone = ui::Tone::Neutral;
    double age_s = 0.0;
};

/// Sensor-mount editor (Tuning): edits a copy, "Apply" rebuilds the session (honest restart).
struct MountEdit {
    bool loaded = false;
    float front_deg = 0.0f;  // front pair toe-out (+/-)
    float side_deg = 45.0f;  // side/diagonal pair angle from forward (+/-)
    int rays = 5;
    float beam_deg = 10.0f;
};

/// Cursors into the session's logs/crashes/recorder so the details log only ingests new data.
struct DetailsSync {
    const host::Session* session = nullptr;
    std::size_t logs = 0, crashes = 0;
    double last_t = -1.0;
    // Run milestones read from the firmware log (for the attempt table).
    double goal_t = -1.0, speed0_t = -1.0, speed1_t = -1.0;
};

struct AppState {
    std::filesystem::path root;      // read-only data: mazes/, config/, external/
    std::filesystem::path user_dir;  // writable state: runs/, reports/, screenshots/, configs
    std::vector<MazeGroup> library;
    char filter[64] = "";

    std::optional<core::Maze> maze;
    std::filesystem::path maze_path;
    std::string load_error;
    std::optional<core::FloodMap> flood;
    core::ValidationReport report;
    core::MazeDims dims;

    Camera cams[ui::kWorkspaceCount];
    bool need_fit[ui::kWorkspaceCount] = {true, true, true, true};
    bool user_moved[ui::kWorkspaceCount] = {false, false, false, false};
    MazeViewOptions view;
    bool want_screenshot = false;
    std::string status;

    // Simulation
    std::optional<core::RobotConfig> robot;
    std::string robot_error;
    std::filesystem::path firmware_dir;
    std::vector<std::filesystem::path> firmwares;
    std::filesystem::path firmware;    // selected DLL
    std::filesystem::path mms_solver;  // mms solver exe (replaces the firmware when set)
    host::MmsOptions mms_opt;
    std::shared_ptr<host::Session> session;
    bool running = false;
    bool boot_buttons[2] = {false, false};
    bool held_buttons[2] = {false, false};  // faceplate buttons pressed right now
    std::string sim_error;                  // load error, hang, ...
    bool hung = false;
    PathTrace trace;
    RobotViewOptions robot_view;
    host::Pacer pacer;
    double ratio = 0.0;
    double window_wall = 0.0;
    long long window_sim_ms = 0;
    std::uint64_t seed = 1;
    TelemetryView tv;
    std::filesystem::path config_path;
    std::string config_hash;

    // Slice 9.10 UI
    ui::Workspace workspace = ui::Workspace::Cockpit;
    ui::Mode mode = ui::Mode::Engineer;
    bool scripted = false;  // --frames: no motion, no audio, deterministic layout
    bool muted = false;
    bool console_open = false;
    bool help_open = false;
    std::vector<std::pair<std::string, ui::Tone>> console_lines;
    std::deque<Toast> toasts;
    ActionRegistry actions;
    // 1 = build the default layout on first visit if none was restored, 2 = rebuild now.
    int layout_state[ui::kWorkspaceCount] = {1, 1, 1, 1};

    // View modes
    HeatLayer heat = HeatLayer::PeakSpeed;
    bool iso = false;
    Vec2 iso_pan[ui::kWorkspaceCount] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
    float iso_zoom[ui::kWorkspaceCount] = {1.0f, 1.0f, 1.0f, 1.0f};
    bool show_route = true;
    std::optional<core::Cell> inspect;  // selected cell
    std::optional<core::Cell> hover;
    float run_mms = 800.0f, accel_mms2 = 3500.0f, mu = 0.8f;
    bool diagonals = true;

    // Telemetry & Compare
    std::vector<std::filesystem::path> archive;  // runs/ folders, newest first
    int compare_b = 0;                           // index into archive
    std::optional<core::analysis::ArchivedRun> run_b;
    std::filesystem::path run_b_path;
    std::string compare_status;

    // Tuning & Reports
    MountEdit mounts;
    std::string step_channel = "sim.v";
    std::string report_status;
    core::analysis::ThermalEstimator thermal;
    std::int64_t thermal_t_us = -1;

    // Event watchers (audio + toasts)
    std::size_t seen_logs = 0;
    std::size_t seen_crashes = 0;

    // v0.2.0: Move & Firmware Details log (Cockpit, Engineer)
    DetailsLog details;
    MoveTracker tracker;
    DetailsSync dsync;
    DetailFilter details_filter = DetailFilter::All;
    char details_search[64] = "";
    bool details_autoscroll = true;

    // v0.2.0: Robot config editor (Tuning)
    ConfigEdit cfg_edit;
    bool cfg_loaded = false;
    std::string cfg_status;
    char cfg_name[64] = "";
    std::vector<std::filesystem::path> cfg_files;  // robot configs found (data + user folders)

    // v0.2.0: race timer + attempts (Tuning)
    RaceTimer timer;
    AttemptTable attempts;
    bool timer_with_go = true;
    bool timer_audio = true;
    bool details_open = true;       // the Details tab (close x on the tab, reopen from Events)
    bool config_edited = false;     // the live config differs from its file (Apply & Restart)
    bool attempt_recorded = false;  // the current run already has its row
    std::string timer_note;         // "Run finished at 1:23.4", "Time is up"

    Camera& cam() { return cams[static_cast<int>(workspace)]; }
};

// --- operations (app_ops.cpp) -----------------------------------------------------------------
std::filesystem::path find_root_from(const std::vector<std::filesystem::path>& starts);
std::vector<MazeGroup> scan_library(const std::filesystem::path& root,
                                    const std::filesystem::path& user_dir = {});
std::filesystem::path find_firmware_dir(const std::filesystem::path& root,
                                        const std::filesystem::path& exe_dir);
std::vector<std::filesystem::path> scan_firmware(const std::filesystem::path& dir);
std::string file_sha256(const std::filesystem::path& p);
std::string short_hash(const std::string& h);
core::Cell cell_of(const core::Pose& p, const core::MazeDims& d);

void make_session(AppState& st);
long long advance(AppState& st, long long ms, double budget_s = 0.0, bool until_cell = false);
void rewind_to(AppState& st, double t_s);
void rebuild_trace(AppState& st);
void set_running(AppState& st, bool on);
void load_maze(AppState& st, const std::filesystem::path& p);
void select_firmware(AppState& st, const std::filesystem::path& dll);  // empty = none
bool export_csv(const host::TelemetryRecorder& rec, const std::filesystem::path& out, double t0,
                double t1, const std::vector<std::string>& channels, std::size_t& rows);
std::filesystem::path timestamped(const std::filesystem::path& dir, const char* prefix,
                                  const char* ext);
std::string now_stamp();  // "2026-11-20 14:03:12"

/// v0.2.0 run diary: ingests new recorder samples, firmware/solver log lines and crashes.
void sync_details(AppState& st);
void clear_details(AppState& st);  // "Clear" button: empties the list, keeps the cursors
/// Race timer + attempt table, once per frame. Returns the audio cue to play (if any).
TimerCue update_race(AppState& st, double dt_real_s);
void record_attempt(AppState& st, const char* note);  // adds a row for the current run (if any)
/// Robot configs under <root>/config/robots and <user_dir>/config/robots, sorted.
std::vector<std::filesystem::path> scan_robot_configs(const AppState& st);
/// Loads a robot config as the active one and restarts the run. Returns the error ("" = ok).
std::string use_robot_config(AppState& st, const std::filesystem::path& path);

void toast(AppState& st, std::string text, ui::Tone tone = ui::Tone::Neutral);
const char* source_label(const AppState& st);  // firmware stem / solver name / "No firmware"
ui::MissionStatus current_status(const AppState& st);

// Recorder helpers ("sim.*" channels, see Session).
struct Series {
    std::vector<double> t, v;
};
Series series(const host::TelemetryRecorder& rec, const char* channel, double t0 = -1e300,
              double t1 = 1e300);

// Run archive (runs/<id>/run.json + telemetry.f32), Slice 9.5.
core::analysis::ArchivedRun archive_current(const AppState& st);
std::filesystem::path save_current_run(AppState& st, std::string& err);
void refresh_archive(AppState& st);
bool load_compare(AppState& st, int index);

// Engineering report (HTML) and MATLAB CSV bundle, Slice 9.8.
core::analysis::ReportInput report_input(const AppState& st);  // checks, metrics, plots, events
bool write_report(const AppState& st, const std::filesystem::path& out, std::string& err);
bool write_matlab_bundle(const AppState& st, const std::filesystem::path& dir, std::string& err);

// Studio maths on the current maze / recording.
core::analysis::Heatmap motion_heat(const AppState& st);

}  // namespace mmsim::app
