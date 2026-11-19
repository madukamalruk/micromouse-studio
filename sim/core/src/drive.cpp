#include "sim/core/drive.hpp"

#include <cmath>

namespace mmsim::core {

Pose integrate_pose(const Pose& p, double v, double w, double dt) {
    Pose q = p;
    const double dth = w * dt;
    if (std::abs(dth) < 1e-9) {
        // Second-order midpoint is exact to ~dth^2 * v dt / 24, far below double noise here.
        const double mid = p.theta + 0.5 * dth;
        q.x = p.x + v * dt * std::cos(mid);
        q.y = p.y + v * dt * std::sin(mid);
    } else {
        const double R = v / w;
        q.x = p.x + R * (std::sin(p.theta + dth) - std::sin(p.theta));
        q.y = p.y - R * (std::cos(p.theta + dth) - std::cos(p.theta));
    }
    q.theta = p.theta + dth;
    return q;
}

DriveModel::DriveModel(const RobotConfig& c)
    : r_(c.wheel_radius), b_(c.track_width), vmax_(c.motor.max_voltage) {
    const MotorConfig& m = c.motor;
    const double G = m.gear_ratio;
    const double J = c.wheel_inertia + m.rotor_inertia * G * G;
    const double lin = c.mass * r_ * r_ / 4.0;
    const double rot = c.yaw_inertia * r_ * r_ / (b_ * b_);
    a_ = lin + rot + J;
    c_ = lin - rot;
    A_ = m.efficiency * G * m.kt / m.resistance;
    D_ = m.efficiency * G * G * (m.kt * m.ke / m.resistance + m.viscous);
    coulomb_ = G * m.coulomb;
}

double DriveModel::clamp_voltage(double v) {
    if (v > vmax_) {
        ++overvoltage_events_;
        return vmax_;
    }
    if (v < -vmax_) {
        ++overvoltage_events_;
        return -vmax_;
    }
    return v;
}

void DriveModel::step(DriveState& s, double vl, double vr, double dt) const {
    const double V[2] = {vl, vr};
    double f[2];
    for (int i = 0; i < 2; ++i) {
        const double w = s.wheel_speed[i];
        if (w != 0.0) {
            f[i] = w > 0.0 ? -coulomb_ : coulomb_;
        } else {
            const double drive = A_ * V[i];
            f[i] = std::abs(drive) <= coulomb_ ? -drive : (drive > 0.0 ? -coulomb_ : coulomb_);
        }
    }
    const double tq[2] = {A_ * V[0] + f[0], A_ * V[1] + f[1]};

    // Decoupled modes: sum = wl + wr (inertia a + c), diff = wr - wl (inertia a - c).
    const double ms = a_ + c_, md = a_ - c_;
    const double sum0 = s.wheel_speed[0] + s.wheel_speed[1];
    const double dif0 = s.wheel_speed[1] - s.wheel_speed[0];
    const double sum1 = (ms * sum0 + dt * (tq[0] + tq[1])) / (ms + dt * D_);
    const double dif1 = (md * dif0 + dt * (tq[1] - tq[0])) / (md + dt * D_);
    double w1[2] = {0.5 * (sum1 - dif1), 0.5 * (sum1 + dif1)};

    // Dry friction can stop a wheel but never reverse it within a step.
    for (int i = 0; i < 2; ++i) {
        const double w0 = s.wheel_speed[i];
        if (w0 != 0.0 && w1[i] * w0 < 0.0 && std::abs(A_ * V[i]) <= coulomb_) w1[i] = 0.0;
    }

    s.wheel_speed[0] = w1[0];
    s.wheel_speed[1] = w1[1];
    s.wheel_angle[0] += w1[0] * dt;
    s.wheel_angle[1] += w1[1] * dt;
    s.pose = integrate_pose(s.pose, forward_speed(s), yaw_rate(s), dt);
}

double DriveModel::forward_speed(const DriveState& s) const {
    return r_ * 0.5 * (s.wheel_speed[0] + s.wheel_speed[1]);
}

double DriveModel::yaw_rate(const DriveState& s) const {
    return r_ * (s.wheel_speed[1] - s.wheel_speed[0]) / b_;
}

}  // namespace mmsim::core
