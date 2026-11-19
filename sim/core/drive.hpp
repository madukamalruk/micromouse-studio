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
};

/// Differential drive with two DC gear motors, wheel-speed coordinates (Lagrangian):
///   M w' = A V - D w + f_coulomb,   M = [[a, c], [c, a]]
///   a = m r^2/4 + Iz r^2/b^2 + J,  c = m r^2/4 - Iz r^2/b^2,  J = J_wheel + J_rotor G^2
///   A = eta G Kt / R,              D = eta G^2 (Kt Ke / R + b_v)
/// Integrated implicitly (stable for any dt) in the decoupled sum/difference modes, so
/// symmetric inputs give exactly straight lines and exactly in-place pivots.
/// No wheel slip (Slice 3); slip/noise models arrive in Slice 4.
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

  private:
    double r_, b_;
    double a_, c_;    // mass matrix entries
    double A_, D_;    // drive gain, damping (wheel side)
    double coulomb_;  // dry friction torque at the wheel
    double vmax_;
    int overvoltage_events_ = 0;
};

}  // namespace mmsim::core
