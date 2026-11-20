#pragma once
// Battery sag, mission energy and thermal margin (Slice 9.1). Port of Neomouse V2
// tools/thermal_battery_sim.py; the live pack model in World uses the same OCV table.

#include <utility>
#include <vector>

namespace mmsim::core::analysis {

constexpr double kGms2 = 9.81;

/// Open-circuit voltage of one generic LiPo cell at state of charge soc (clamped 0..1).
double lipo_ocv(double soc);

/// Electronics loads and motor currents of the Neomouse mission profile (A, placeholders).
struct Loads {
    double i_3v3_a = 0.060;
    double i_5v_own_a = 0.045;
    double motor_explore_a = 0.55;
    double motor_run_a = 1.6;
    double motor_peak_a = 2.6;
    double electronics() const { return i_3v3_a + i_5v_own_a; }
};

/// (t, motor current) samples: exploration, then `runs` speed runs with idle between.
std::vector<std::pair<double, double>> mission_profile(double explore_s = 180.0,
                                                       double run_s = 12.0, int runs = 3,
                                                       double idle_between_s = 20.0,
                                                       double dt = 0.05);

struct BatteryRow {
    double t, soc, v, i;
};
struct BatteryRun {
    std::vector<BatteryRow> rows;
    double end_soc = 0, min_v = 0, mah_used = 0, margin_to_6v0 = 0;
};
/// 2S pack: V = 2 OCV(SoC) - R_int I, Coulomb counting.
BatteryRun battery_run(const std::vector<std::pair<double, double>>& profile,
                       double cap_mah = 450.0, double r_int = 0.15, double soc0 = 1.0);

/// Dissipation (W) in the 5 V LDO (U17), 3V3 LDO (U18) and DRV8833 (U13).
struct Powers {
    double u17 = 0, u18 = 0, u13 = 0;
};
Powers powers(double v_bat, double i_motor_total);

struct ThermalPart {
    double theta_ja;  // K/W
    double tau_s;     // first-order time constant
};
struct Junctions {
    Powers p_full;
    double steady_u17 = 0, steady_u18 = 0, steady_u13 = 0;  // degC at 8.4 V, motor_run_a
    double peak_u17 = 0, peak_u18 = 0, peak_u13 = 0;        // transient max over the mission
};
/// u17_v2 = true: V2 board copper (theta 52 K/W) instead of RC3.2 (80 K/W).
Junctions junction_temps(const BatteryRun& b, double t_amb = 30.0, bool u17_v2 = true);

/// Live first-order junction estimate driven by the simulated pack (GUI gauge).
class ThermalEstimator {
  public:
    explicit ThermalEstimator(double t_amb = 30.0) { reset(t_amb); }
    void reset(double t_amb);
    void update(double dt, double v_bat, double i_motor_total);
    double u17() const { return tj_[0]; }
    double u18() const { return tj_[1]; }
    double u13() const { return tj_[2]; }
    double peak_max() const;  // hottest part's peak so far

  private:
    double amb_ = 30.0;
    double tj_[3]{};
    double peak_[3]{};
};

struct AxleLoads {
    double front_n, rear_n;
};
struct Chassis {
    double mass_kg = 0.105, wheelbase_mm = 35.35, cg_from_front_mm = 16.8, cg_height_mm = 11.0;
};
AxleLoads axle_loads(double a_mms2, const Chassis& ch = {});
struct PitchLimits {
    double brake_rear_lift_ms2, accel_front_lift_ms2;
};
PitchLimits pitch_limits(const Chassis& ch = {});

}  // namespace mmsim::core::analysis
