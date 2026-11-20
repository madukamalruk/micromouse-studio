#pragma once
// Scalar engineering metrics (Slice 9.4, 9.7, 9.8): step response, IMU noise, cornering
// envelope, battery health, fault explainer, telemetry events. Ports of
// Neomouse V2 dashboard_analysis.py; cross-checked on tests/fixtures/analysis/*.json.

#include <optional>
#include <string>
#include <vector>

namespace mmsim::core::analysis {

struct StepMetrics {
    std::string error;  // non-empty: no step found (reason)
    double rise_time_s = 0, settling_time_s = 0, overshoot_pct = 0;
    double final_value = 0, baseline = 0, peak = 0;
    double t10_s = 0, t90_s = 0, t_start_s = 0;
    int samples = 0;
    bool ok() const { return error.empty(); }
};
/// Rise 10-90 %, settling (2 % band) and overshoot of the first step in y(t).
StepMetrics step_metrics(const std::vector<double>& t_s, const std::vector<double>& y,
                         double min_step = 100.0);
/// Steady-state error |setpoint - final| for a known setpoint (0 if no step).
double steady_state_error(const StepMetrics& m, double setpoint);

struct ImuStats {
    int n = 0;
    std::optional<double> mean, stddev, p2p;  // unset for n < 2
};
ImuStats imu_stats(const std::vector<double>& samples);

struct Cornering {
    std::string error;
    double v_mms = 0, radius_mm = 0, mu = 0;
    double a_lat_mms2 = 0, a_lat_g = 0, limit_mms = 0, limit_g = 0, ratio = 0;
    std::string verdict;  // WITHIN TRACTION / NEAR LIMIT / EXCEEDS TRACTION
    bool within = false;  // ratio <= 1
};
Cornering cornering(double v_mms, double radius_mm = 90.0, double mu = 0.8);

struct BatteryHealth {
    std::string error;
    double rint_mohm = 0, peak_a = 0, sag_mv = 0, predicted_min_mv = 0, margin_mv = 0;
    bool healthy = false, brownout_risk = false;
    std::string badge;
};
constexpr double kMcuResetMv = 6000.0;
constexpr double kRintHealthyMohm = 150.0;
/// R_int = (V_rest - V_load) / I; margin of the predicted pack minimum at peak_a over reset_mv.
BatteryHealth battery_health(double rest_mv, double load_mv, double current_a,
                             std::optional<double> peak_a = std::nullopt,
                             double reset_mv = kMcuResetMv);

struct FaultInfo {
    int code = 0;
    std::string name, meaning, action;
};
/// Neomouse V2 firmware fault codes (safety.h fault_t), for Phase 2 logs.
FaultInfo neomouse_fault_info(int code);
/// Simulator-side faults and host errors (firmware fault, hang, crash, load error ...).
/// `kind` is the host outcome / fault keyword; unknown kinds get a generic card.
FaultInfo sim_fault_info(const std::string& kind, const std::string& detail = {});

/// One decoded log record (Neomouse export layout).
struct LogRow {
    int segment = 0;
    double t_ms = 0;
    int state = 0;
    std::string state_name;
    int fault = 0;
    std::string fault_name;
    bool bias_valid = false, slip = false, degraded = false;
};
struct TelemetryEvent {
    double t_s = 0;
    int segment = 0;
    std::string kind;  // segment / state / fault / bias_valid / slip / degraded (sim: log, led,
                       // crash, phase)
    std::string text;
};
std::vector<TelemetryEvent> telemetry_events(const std::vector<LogRow>& rows,
                                             std::size_t limit = 500);

}  // namespace mmsim::core::analysis
