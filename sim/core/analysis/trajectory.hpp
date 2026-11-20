#pragma once
// Motion-primitive deck of a cell route (Slice 9.5): STRAIGHT / CURVE / DIAGONAL with a
// trapezoidal speed plan. Port of dashboard_analysis.compile_trajectory (an estimate for
// the path preview, not a firmware trajectory compiler).

#include <string>
#include <vector>

#include "sim/core/maze.hpp"

namespace mmsim::core::analysis {

constexpr double kGmms2 = 9806.65;   // standard gravity, mm/s^2
constexpr double kTyreMu = 0.8;      // assumed tyre / floor friction
constexpr double kCornerRmm = 90.0;  // smooth 90 deg corner through a 180 mm cell

struct Primitive {
    std::string kind;  // "straight", "curve", "diagonal"
    double length_mm = 0.0;
    double v_mms = 0.0;  // curve: arc speed; straight/diagonal: peak speed
    int ms = 0;
    int angle_deg = 0;  // curve only
    char dir = 0;       // curve only: 'L' / 'R'
    double radius_mm = 0.0;
    double g = 0.0;  // curve only
    int seq = 0;
    std::string text;
};

struct TrajectoryOptions {
    double run_mms = 800.0;
    double accel_mms2 = 3500.0;
    double radius_mm = kCornerRmm;
    double mu = kTyreMu;
    bool diagonal = true;
    double cell_mm = 180.0;
};

struct Trajectory {
    std::string error;  // non-empty: invalid input (other fields unset)
    std::vector<Primitive> primitives;
    double total_mm = 0.0;
    double total_s = 0.0;
    double peak_g = 0.0;
    double limit_mms = 0.0;
    int moves = 0;
    int diagonals = 0;
    bool ok() const { return error.empty(); }
};

/// cells: route with the start first; consecutive cells must be neighbours, no U-turns.
Trajectory compile_trajectory(const std::vector<Cell>& cells, const TrajectoryOptions& o = {});

/// Seconds and peak speed for `length` mm from v0 to v1 (accel a both ways, <= vmax).
std::pair<double, double> straight_time(double length, double v0, double v1, double vmax, double a);

/// Python-compatible rounding: round half to even on the scaled value.
double round_half_even(double v, int digits);

}  // namespace mmsim::core::analysis
