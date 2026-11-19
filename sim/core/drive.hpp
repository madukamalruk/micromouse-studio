#pragma once

#include "sim/core/robot_config.hpp"

namespace mmsim::core {

/// Planar pose: axle centre (m), heading (rad, 0 = +x/east, +pi/2 = north, CCW positive).
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

/// Exact constant-(v, w) arc integration. Straight lines and pure pivots are exact too.
Pose integrate_pose(const Pose& p, double v, double w, double dt);

/// Index 0 = left wheel, 1 = right wheel. Wheel speed/angle: + = robot moves forward.
struct DriveState {
    Pose pose;
    double wheel_speed[2] = {0.0, 0.0};  // rad/s
    double wheel_angle[2] = {0.0, 0.0};  // rad, accumulated (encoder source)
    // Body motion. Slip off: derived from the wheels after every step. Slip on: own states.
    double v = 0.0;  // forward ground speed, m/s
    double w = 0.0;  // yaw rate, rad/s
    double beta =
        0.0;  // slip angle: heading minus velocity direction, rad (+ = nose inside a left turn)
};

/// Differential drive with two DC gear motors, wheel-speed coordinates (Lagrangian):
///   M w' = A V - D w + f_coulomb,   M = [[a, c], [c, a]]
///   a = m r^2/4 + Iz r^2/b^2 + J,  c = m r^2/4 - Iz r^2/b^2,  J = J_wheel + J_rotor G^2
///   A = eta G Kt / R,              D = eta G^2 (Kt Ke / R + b_v)
/// Integrated implicitly (stable for any dt) in the decoupled sum/difference modes, so
/// symmetric inputs give exactly straight lines and exactly in-place pivots.
/// Optional linear tyre slip model (D7): see step_slip().
class DriveModel {
  public:
    explicit DriveModel(const RobotConfig& c);

    /// Advances wheels and pose by dt with terminal voltages vl, vr (already clamped).
    void step(DriveState& s, double vl, double vr, double dt) const;

    /// Clamps to +-max_voltage; every clamp is counted (overvoltage flag for the UI).
    double clamp_voltage(double v);
    int overvoltage_events() const { return overvoltage_events_; }
    void reset_counters() { overvoltage_events_ = 0; }

    double forward_speed(const DriveState& s) const;  // m/s
    double yaw_rate(const DriveState& s) const;       // rad/s, + = CCW
    bool slip_enabled() const { return slip_.enabled; }
    /// Wheel rolling speed minus its contact patch ground speed (m/s); + = wheelspin.
    double slip_velocity(const DriveState& s, int side) const;
    /// Tyre force limit per wheel, N (mu * m * g / 2).
    double traction_limit() const { return mu_n_; }

  private:
    double r_, b_;
    double a_, c_;    // mass matrix entries
    double A_, D_;    // drive gain, damping (wheel side)
    double coulomb_;  // dry friction torque at the wheel
    double vmax_;
    int overvoltage_events_ = 0;
    // Slip model (D7)
    void step_slip(DriveState& s, const double tq[2], double dt) const;
    SlipConfig slip_;
    double m_, iz_, j_;    // body mass, yaw inertia, wheel+rotor inertia (wheel side)
    double mu_n_ = 0.0;    // traction limit per wheel
    double slip_k_ = 0.0;  // slip angle per lateral acceleration, rad per m/s^2
};

}  // namespace mmsim::core
