#pragma once
// Workspace panels (Slice 9.10). Each draws one dockable window; window IDs are unique per
// workspace ("Maze###maze_cockpit") so every workspace keeps its own layout.

#include <cstdint>
#include <optional>
#include <vector>

#include "sim/app/app_state.hpp"
#include "sim/core/analysis/maze_paths.hpp"
#include "sim/core/analysis/runs.hpp"
#include "sim/core/analysis/trajectory.hpp"

namespace mmsim::app {

enum class CanvasKind { Cockpit, Studio, Ghost };

// Window names (titles + stable ### ids), shared with the DockBuilder layouts in main.cpp.
namespace win {
inline constexpr const char* kMazeCockpit = "Maze###maze_cockpit";
inline constexpr const char* kMazeStudio = "Maze###maze_studio";
inline constexpr const char* kMazeGhost = "Ghost replay###maze_ghost";
inline constexpr const char* kFaceplate = "Robot###faceplate";
inline constexpr const char* kSensors = "Sensors###sensors";
inline constexpr const char* kEvents = "Events###events";
inline constexpr const char* kOverlays = "Overlays###overlays";
inline constexpr const char* kTrajectory = "Trajectory deck###trajectory";
inline constexpr const char* kInspector = "Cell inspector###inspector";
inline constexpr const char* kTelemetry = "Telemetry###telemetry";
inline constexpr const char* kCompare = "Run compare###compare";
inline constexpr const char* kTraction = "Traction circle###traction";
inline constexpr const char* kMounts = "Sensor mounts###mounts";
inline constexpr const char* kBattery = "Battery & thermal###battery";
inline constexpr const char* kStep = "Step response###step";
inline constexpr const char* kImu = "IMU noise###imu";
inline constexpr const char* kReports = "Reports###reports";
inline constexpr const char* kRecord = "Run record###record";
inline constexpr const char* kAttitude = "Attitude###attitude";
inline constexpr const char* kDetails = "Move & Firmware Details###details";
inline constexpr const char* kConfig = "Robot config###config";
inline constexpr const char* kTimer = "Race timer###timer";
}  // namespace win

void panel_maze(AppState& st, const char* window, CanvasKind kind);

void panel_faceplate(AppState& st);
void panel_sensors(AppState& st);
void panel_events(AppState& st);

void panel_overlays(AppState& st);
void panel_trajectory(AppState& st);
void panel_inspector(AppState& st);

void panel_telemetry(AppState& st);
void panel_compare(AppState& st);
void panel_traction(AppState& st);

void panel_mounts(AppState& st);
void panel_battery(AppState& st);
void panel_step(AppState& st);
void panel_imu(AppState& st);
void panel_reports(AppState& st);
void panel_record(AppState& st);

// v0.2.0 (panels_extra.cpp)
void panel_attitude(AppState& st);  // Cockpit: gyro attitude compass
void panel_details(AppState& st);   // Cockpit: Move & Firmware Details log
void panel_config(AppState& st);    // Tuning: live robot config editor
void panel_timer(AppState& st);     // Tuning: race timer + attempt table

/// Planned route on the true maze (Studio): shortest path, compiled trajectory, potential
/// field and which moves belong to diagonal (staircase) chains. Cached per maze + options.
struct StudioPlan {
    std::uint64_t key = 0;
    std::optional<core::analysis::ShortestPath> path;
    core::analysis::Trajectory traj;
    core::analysis::PotentialField potential;
    double potential_max = 0.0;
    std::vector<bool> diagonal_move;  // per move (cells[i] -> cells[i+1])
};
const StudioPlan& studio_plan(AppState& st);
/// Motion heatmap of the current recording, recomputed at most ~4x per second.
const core::analysis::Heatmap& heat_cache(AppState& st);
const core::analysis::HeatCell* heat_at(const core::analysis::Heatmap& h, int x, int y);

/// a_long / a_lat samples (g) of the current recording for the traction circle.
struct GripSamples {
    std::vector<double> a_long_g, a_lat_g;
    double peak_ratio = 0.0;  // max |a| / (mu g)
};
GripSamples grip_samples(const AppState& st, double mu);

}  // namespace mmsim::app
