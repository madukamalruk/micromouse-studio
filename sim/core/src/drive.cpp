#include "sim/core/drive.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace mmsim::core {

namespace {

constexpr double kGravity = 9.80665;

// Solves A x = b (4x4) by Gaussian elimination with partial pivoting.
void solve4(double A[4][4], double b[4], double x[4]) {
    for (int c = 0; c < 4; ++c) {
        int p = c;
        for (int r = c + 1; r < 4; ++r)
            if (std::abs(A[r][c]) > std::abs(A[p][c])) p = r;
        if (p != c) {
            for (int k = 0; k < 4; ++k) std::swap(A[c][k], A[p][k]);
            std::swap(b[c], b[p]);
        }
        for (int r = c + 1; r < 4; ++r) {
            const double f = A[r][c] / A[c][c];
            if (f == 0.0) continue;
            for (int k = c; k < 4; ++k) A[r][k] -= f * A[c][k];
            b[r] -= f * b[c];
        }
    }
    for (int r = 3; r >= 0; --r) {
        double acc = b[r];
        for (int k = r + 1; k < 4; ++k) acc -= A[r][k] * x[k];
        x[r] = acc / A[r][r];
    }
}

}  // namespace

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
    : r_(c.wheel_radius), b_(c.track_width), vmax_(effective_max_voltage(c.motor)) {
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
    slip_ = c.slip;
    m_ = c.mass;
    iz_ = c.yaw_inertia;
    j_ = J;
    mu_n_ = c.slip.mu * c.mass * kGravity / 2.0;
    slip_k_ = c.slip.slip_gradient_deg_per_g * std::numbers::pi / 180.0 / kGravity;
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
    const double w0[2] = {s.wheel_speed[0], s.wheel_speed[1]};

    if (slip_.enabled) {
        step_slip(s, tq, dt);
        // Dry friction can stop a wheel but never reverse it within a step.
        for (int i = 0; i < 2; ++i)
            if (w0[i] != 0.0 && s.wheel_speed[i] * w0[i] < 0.0 && std::abs(A_ * V[i]) <= coulomb_)
                s.wheel_speed[i] = 0.0;
        return;
    }

    // Decoupled modes: sum = wl + wr (inertia a + c), diff = wr - wl (inertia a - c).
    const double ms = a_ + c_, md = a_ - c_;
    const double sum0 = w0[0] + w0[1];
    const double dif0 = w0[1] - w0[0];
    const double sum1 = (ms * sum0 + dt * (tq[0] + tq[1])) / (ms + dt * D_);
    const double dif1 = (md * dif0 + dt * (tq[1] - tq[0])) / (md + dt * D_);
    double w1[2] = {0.5 * (sum1 - dif1), 0.5 * (sum1 + dif1)};

    for (int i = 0; i < 2; ++i)
        if (w0[i] != 0.0 && w1[i] * w0[i] < 0.0 && std::abs(A_ * V[i]) <= coulomb_) w1[i] = 0.0;

    s.wheel_speed[0] = w1[0];
    s.wheel_speed[1] = w1[1];
    s.wheel_angle[0] += w1[0] * dt;
    s.wheel_angle[1] += w1[1] * dt;
    s.v = r_ * 0.5 * (w1[0] + w1[1]);
    s.w = r_ * (w1[1] - w1[0]) / b_;
    s.beta = 0.0;
    s.pose = integrate_pose(s.pose, s.v, s.w, dt);
}

// Tyre model (D7). States: wheel speeds, body forward speed v, yaw rate w.
//   F_i = C (r w_i - u_i),  u_L = v - w b/2,  u_R = v + w b/2,  |F_i| <= mu m g / 2
//   J w_i' = tq_i - D w_i - r F_i,   m v' = F_L + F_R,   Iz w' = (F_R - F_L) b / 2
// Unsaturated: backward Euler in decoupled sum/difference modes (exact symmetry).
// A saturated wheel's force is the constant limit; the remaining system is solved 4x4.
void DriveModel::step_slip(DriveState& s, const double tq[2], double dt) const {
    const double C = slip_.long_stiffness, r = r_, b = b_, J = j_;
    const double jw = J / dt + D_;
    double wl, wr, v1, w1;
    {
        const double om0 = 0.5 * (s.wheel_speed[0] + s.wheel_speed[1]);
        const double de0 = 0.5 * (s.wheel_speed[1] - s.wheel_speed[0]);
        const double tbar = 0.5 * (tq[0] + tq[1]), tdif = 0.5 * (tq[1] - tq[0]);
        // Symmetric mode [Omega, v].
        double a11 = jw + C * r * r, a12 = -C * r, a21 = -2.0 * C * r, a22 = m_ / dt + 2.0 * C;
        double b1 = J / dt * om0 + tbar, b2 = m_ / dt * s.v;
        double det = a11 * a22 - a12 * a21;
        const double om1 = (b1 * a22 - a12 * b2) / det;
        v1 = (a11 * b2 - a21 * b1) / det;
        // Antisymmetric mode [Delta, w].
        a12 = -C * r * b / 2.0;
        a21 = -C * b * r;
        a22 = iz_ / dt + C * b * b / 2.0;
        b1 = J / dt * de0 + tdif;
        b2 = iz_ / dt * s.w;
        det = a11 * a22 - a12 * a21;
        const double de1 = (b1 * a22 - a12 * b2) / det;
        w1 = (a11 * b2 - a21 * b1) / det;
        wl = om1 - de1;
        wr = om1 + de1;
    }

    bool sat[2] = {false, false};
    double k[2] = {0.0, 0.0};
    for (int iter = 0; iter < 3; ++iter) {
        const double ws[2] = {wl, wr};
        bool changed = false;
        for (int i = 0; i < 2; ++i) {
            const double side = i == 0 ? 1.0 : -1.0;
            const double F = C * (r * ws[i] - v1 + side * w1 * b / 2.0);
            if (!sat[i] && std::abs(F) > mu_n_) {
                sat[i] = true;
                k[i] = F > 0.0 ? mu_n_ : -mu_n_;
                changed = true;
            }
        }
        if (!changed) break;
        // x = [wl, wr, v, w]; F_i = g_i . x + k_i  (g_i = 0 when saturated, k_i = 0 otherwise).
        double g[2][4] = {{C * r, 0.0, -C, C * b / 2.0}, {0.0, C * r, -C, -C * b / 2.0}};
        for (int i = 0; i < 2; ++i)
            if (sat[i])
                for (double& e : g[i]) e = 0.0;
        double A[4][4] = {};
        double rhs[4];
        for (int i = 0; i < 2; ++i) {
            A[i][i] = jw;
            for (int c = 0; c < 4; ++c) A[i][c] += r * g[i][c];
            rhs[i] = J / dt * s.wheel_speed[i] + tq[i] - r * k[i];
        }
        for (int c = 0; c < 4; ++c) {
            A[2][c] = -(g[0][c] + g[1][c]);
            A[3][c] = -(g[1][c] - g[0][c]) * b / 2.0;
        }
        A[2][2] += m_ / dt;
        A[3][3] += iz_ / dt;
        rhs[2] = m_ / dt * s.v + k[0] + k[1];
        rhs[3] = iz_ / dt * s.w + (k[1] - k[0]) * b / 2.0;
        double x[4] = {};
        solve4(A, rhs, x);
        wl = x[0];
        wr = x[1];
        v1 = x[2];
        w1 = x[3];
    }

    s.wheel_speed[0] = wl;
    s.wheel_speed[1] = wr;
    s.wheel_angle[0] += wl * dt;
    s.wheel_angle[1] += wr * dt;

    // Lateral: velocity direction phi = theta - beta with beta = k * a_lat (slip gradient);
    // the path may not curve faster than mu*g allows, so at the limit the robot skids wide.
    const double th0 = s.pose.theta, th1 = th0 + w1 * dt;
    const double phi0 = th0 - s.beta;
    const double amax = slip_.mu * kGravity;
    const double a_lat = std::clamp(v1 * w1, -amax, amax);
    double dphi = (th1 - slip_k_ * a_lat) - phi0;
    if (std::abs(v1) > 1e-9) {
        const double lim = amax * dt / std::abs(v1);
        dphi = std::clamp(dphi, -lim, lim);
    }
    const double phi1 = phi0 + dphi;
    const double mid = 0.5 * (phi0 + phi1);
    s.pose.x += v1 * dt * std::cos(mid);
    s.pose.y += v1 * dt * std::sin(mid);
    s.pose.theta = th1;
    s.beta = th1 - phi1;
    s.v = v1;
    s.w = w1;
}

double DriveModel::slip_velocity(const DriveState& s, int side) const {
    side &= 1;
    const double u = side == 0 ? s.v - s.w * b_ / 2.0 : s.v + s.w * b_ / 2.0;
    return r_ * s.wheel_speed[side] - u;
}

double DriveModel::forward_speed(const DriveState& s) const {
    if (slip_.enabled) return s.v;
    return r_ * 0.5 * (s.wheel_speed[0] + s.wheel_speed[1]);
}

double DriveModel::yaw_rate(const DriveState& s) const {
    if (slip_.enabled) return s.w;
    return r_ * (s.wheel_speed[1] - s.wheel_speed[0]) / b_;
}

}  // namespace mmsim::core
