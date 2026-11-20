#pragma once
// Runs on a common distance axis, spatial heatmaps and the local run archive (Slice 9.2,
// 9.3). compare_runs / run_series / motion_heatmap are ports of dashboard_analysis.py.

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace mmsim::core::analysis {

/// Speed trace from the first movement above 20 mm/s to the last.
struct RunSeries {
    std::string error;
    std::vector<double> t, v, s, gyro;  // s, mm/s, mm, deg/s
};
/// t_ms, speed (mm/s) and gyro per sample (one log segment).
RunSeries run_series(const std::vector<double>& t_ms, const std::vector<double>& v_mms,
                     const std::vector<double>& gyro_dps);

struct RunSummary {
    double duration_s = 0, distance_mm = 0, mean_mms = 0, peak_mms = 0;
};
struct Swing {
    double at_mm = 0, delta_s = 0, window_mm = 0;
};
struct Comparison {
    std::string error;
    RunSummary a, b;
    double common_mm = 0, delta_t_final_s = 0;
    std::vector<double> grid_mm, delta_t_s, delta_v_mms, v_a, v_b;
    std::vector<double> trace_a_t, trace_a_v, trace_b_t, trace_b_v;
    Swing best_gain, worst_loss;
    std::string verdict;
    bool ok() const { return error.empty(); }
};
/// B minus A on a distance grid (negative dt = B ahead).
Comparison compare_runs(const RunSeries& a, const RunSeries& b, double step_mm = 10.0,
                        std::size_t max_points = 400, double window_mm = 100.0);

/// Linear interpolation over non-decreasing xs (clamped), like dashboard_analysis._at.
double interp_at(const std::vector<double>& xs, const std::vector<double>& ys, double x);

/// One sample for the heatmap: position (mm), |speed| (mm/s), |a_long|, |a_lat| (mm/s^2).
struct MotionSample {
    double x_mm = 0, y_mm = 0, v = 0, a_long = 0, a_lat = 0;
};
struct HeatCell {
    int x = 0, y = 0;
    double v_max = 0, a_long_max = 0, a_lat_max = 0;
    int samples = 0;
};
struct Heatmap {
    std::string error;
    std::vector<HeatCell> cells;  // sorted by (y, x); values rounded to 0.1
    double v_max = 0, a_long_max = 0, a_lat_max = 0;
};
/// Per-cell maxima of samples (cell = floor(pos / cell_mm)).
Heatmap bin_heatmap(const std::vector<MotionSample>& samples, double cell_mm = 180.0);

/// Neomouse WHEELS export row (dead-reckoned heatmap, cross-check with Python).
struct WheelRow {
    int segment = 0;
    double t_ms = 0, wheel_l_cps = 0, wheel_r_cps = 0, gyro_dps = 0;
};
Heatmap motion_heatmap(const std::vector<WheelRow>& rows, double counts_per_mm,
                       double cell_mm = 180.0, double tau_s = 0.02);

// ---------------------------------------------------------------- run archive (runs/)
/// A finished run: record fields + equally sampled telemetry columns (float32).
struct ArchivedRun {
    std::string id;                           // folder name
    std::map<std::string, std::string> meta;  // maze, source, seed, outcome, trajectory_hash ...
    std::map<std::string, double> numbers;    // sim_s, search_s, speed_run_s, distance_m, crashes
    std::vector<std::string> channels;        // first channel = time (s)
    std::vector<std::vector<float>> data;     // data[channel][sample]
    const std::vector<float>* column(const std::string& name) const;
};
/// Writes runs/<id>/run.json + telemetry.f32 (little-endian float32, channel-major).
/// Returns the folder, or empty + err.
std::filesystem::path save_run(const std::filesystem::path& root, ArchivedRun run,
                               std::string& err);
std::optional<ArchivedRun> load_run(const std::filesystem::path& dir, std::string& err);
/// Run folders under root, newest id first (ids start with a sortable timestamp).
std::vector<std::filesystem::path> list_runs(const std::filesystem::path& root);
/// "20261120-153012_uk1987f_ab12cd34" from a timestamp, maze name and trajectory hash.
std::string make_run_id(const std::string& timestamp, const std::string& maze,
                        const std::string& hash);

}  // namespace mmsim::core::analysis
