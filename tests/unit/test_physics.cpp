// Differential-drive kinematics + motor dynamics (Slice 3, PRD F6).
// Expected values are closed-form solutions written independently of the model code.
#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

#include "sim/core/drive.hpp"
#include "sim/core/robot_config.hpp"

using namespace mmsim::core;
using std::numbers::pi;

namespace {

// Frictionless, 100 %-efficient robot so the analytic first-order solutions are exact.
RobotConfig ideal_robot() {
    RobotConfig c;
    c.mass = 0.1;
    c.yaw_inertia = 1.4e-4;
    c.track_width = 0.072;
    c.wheel_radius = 0.016;
    c.wheel_inertia = 2.0e-7;
    c.motor.resistance = 10.0;
    c.motor.kt = 0.00229;
    c.motor.ke = 0.00229;
    c.motor.gear_ratio = 30.0;
    c.motor.rotor_inertia = 1.0e-8;
    c.motor.viscous = 0.0;
    c.motor.coulomb = 0.0;
    c.motor.efficiency = 1.0;
    c.motor.max_voltage = 100.0;
    c.motor.allow_unsafe_voltage = true;
    c.battery_voltage = 7.4;
    return c;
}

// Independent derivation (Lagrangian, wheel-speed coordinates):
//   per-wheel generalised inertia for symmetric motion  Js = m r^2/2 + J
//   ... for antisymmetric (spin) motion                Ja = 2 Iz r^2 / b^2 + J
//   J = J_wheel + J_rotor G^2, damping D = G^2 Kt Ke / R, drive gain A = G Kt / R
struct Analytic {
    double J, Js, Ja, D, A;
    explicit Analytic(const RobotConfig& c) {
        const double r = c.wheel_radius, b = c.track_width, G = c.motor.gear_ratio;
        J = c.wheel_inertia + c.motor.rotor_inertia * G * G;
        Js = c.mass * r * r / 2.0 + J;
        Ja = 2.0 * c.yaw_inertia * r * r / (b * b) + J;
        D = G * G * c.motor.kt * c.motor.ke / c.motor.resistance;
        A = G * c.motor.kt / c.motor.resistance;
    }
};

constexpr double kDt = 0.00025;  // physics substep

void run(DriveModel& m, DriveState& s, double vl, double vr, double seconds) {
    const long n = std::lround(seconds / kDt);
    for (long i = 0; i < n; ++i) m.step(s, vl, vr, kDt);
}

}  // namespace

TEST_SUITE("physics") {
    TEST_CASE("kinematics: exact arc integration - straight, circle, pivot") {
        Pose p{0, 0, 0.3};
        Pose q = integrate_pose(p, 1.0, 0.0, 2.0);  // straight 2 m along heading 0.3 rad
        CHECK(q.x == doctest::Approx(2.0 * std::cos(0.3)).epsilon(1e-12));
        CHECK(q.y == doctest::Approx(2.0 * std::sin(0.3)).epsilon(1e-12));
        CHECK(q.theta == 0.3);

        // v = 0.5 m/s, w = 2 rad/s -> R = 0.25 m; a full revolution returns to start.
        Pose c{0, 0, 0};
        const double T = 2.0 * pi / 2.0;
        const int steps = 4000;
        double max_r_err = 0.0;
        for (int i = 0; i < steps; ++i) {
            c = integrate_pose(c, 0.5, 2.0, T / steps);
            const double r = std::hypot(c.x - 0.0, c.y - 0.25);  // centre at (0, R) for left turn
            max_r_err = std::max(max_r_err, std::abs(r - 0.25) / 0.25);
        }
        CHECK(max_r_err < 1e-9);
        CHECK(std::hypot(c.x, c.y) < 1e-9);

        Pose piv = integrate_pose({1, 2, 0}, 0.0, 3.0, 0.5);  // pure rotation
        CHECK(piv.x == 1.0);
        CHECK(piv.y == 2.0);
        CHECK(piv.theta == doctest::Approx(1.5));
    }

    TEST_CASE("dynamics: straight-line step response matches first-order analytic") {
        const RobotConfig cfg = ideal_robot();
        const Analytic an(cfg);
        DriveModel model(cfg);
        DriveState s;
        const double V = 3.0;
        const double tau = an.Js / an.D;
        const double w_ss = an.A * V / an.D;  // = V / (G Ke)
        CHECK(w_ss == doctest::Approx(V / (cfg.motor.gear_ratio * cfg.motor.ke)));

        // Time to reach 63.2 % of the final wheel speed.
        double t = 0.0, t63 = -1.0;
        while (t < 10.0 * tau) {
            model.step(s, V, V, kDt);
            t += kDt;
            if (t63 < 0 && s.wheel_speed[0] >= (1.0 - std::exp(-1.0)) * w_ss) t63 = t;
        }
        INFO("tau analytic = " << tau << " s, measured = " << t63 << " s");
        CHECK(t63 == doctest::Approx(tau).epsilon(0.02));
        CHECK(s.wheel_speed[0] == doctest::Approx(w_ss).epsilon(1e-3));
        CHECK(s.wheel_speed[1] == doctest::Approx(w_ss).epsilon(1e-3));
    }

    TEST_CASE("dynamics: straight-line distance and zero heading drift over 10 m") {
        const RobotConfig cfg = ideal_robot();
        const Analytic an(cfg);
        DriveModel model(cfg);
        DriveState s;
        s.pose = {0, 0, pi / 2};  // facing north
        const double V = 4.0;
        const double tau = an.Js / an.D, v_ss = cfg.wheel_radius * an.A * V / an.D;
        const double T = 1.0;
        run(model, s, V, V, T);
        const double expect = v_ss * (T - tau * (1.0 - std::exp(-T / tau)));
        INFO("distance analytic " << expect << " m, simulated " << s.pose.y << " m");
        CHECK(s.pose.y == doctest::Approx(expect).epsilon(0.005));
        CHECK(std::abs(s.pose.x) < 1e-12);

        while (s.pose.y < 10.0) model.step(s, V, V, kDt);
        CHECK(std::abs(s.pose.theta - pi / 2) < 1e-9);
        CHECK(std::abs(s.pose.x) < 1e-9);
    }

    TEST_CASE("dynamics: pure in-place turn keeps position and reaches analytic yaw rate") {
        const RobotConfig cfg = ideal_robot();
        const Analytic an(cfg);
        DriveModel model(cfg);
        DriveState s;
        s.pose = {0.5, 0.25, 0.0};
        const double V = 2.0;
        const double tau = an.Ja / an.D;
        run(model, s, -V, V, 12.0 * tau);
        const double w_wheel = an.A * V / an.D;
        const double yaw_rate = 2.0 * cfg.wheel_radius * w_wheel / cfg.track_width;
        CHECK(s.pose.x == 0.5);
        CHECK(s.pose.y == 0.25);
        CHECK(model.yaw_rate(s) == doctest::Approx(yaw_rate).epsilon(1e-3));
        CHECK(model.forward_speed(s) == doctest::Approx(0.0).epsilon(1e-12));
    }

    TEST_CASE("dynamics: unequal voltages settle on the analytic turning radius") {
        const RobotConfig cfg = ideal_robot();
        const Analytic an(cfg);
        DriveModel model(cfg);
        DriveState s;
        const double VL = 2.0, VR = 3.0;
        run(model, s, VL, VR, 2.0);
        // Linear model: steady wheel speeds are A V / D each (no coupling at steady state).
        const double wl = an.A * VL / an.D, wr = an.A * VR / an.D;
        const double R = cfg.track_width / 2.0 * (wr + wl) / (wr - wl);
        const double v = model.forward_speed(s), w = model.yaw_rate(s);
        CHECK(v / w == doctest::Approx(R).epsilon(1e-3));
    }

    TEST_CASE("motor: voltage clamped to max_voltage and flagged; stiction holds small voltage") {
        RobotConfig cfg = ideal_robot();
        cfg.motor.max_voltage = 6.0;
        cfg.motor.coulomb = 2.0e-4;  // at the motor shaft
        DriveModel model(cfg);
        DriveState s;
        CHECK(model.clamp_voltage(7.4) == 6.0);
        CHECK(model.clamp_voltage(-9.0) == -6.0);
        CHECK(model.overvoltage_events() == 2);
        CHECK(model.clamp_voltage(5.0) == 5.0);

        // Drive torque A*V below the Coulomb torque at the wheel -> the robot must not creep.
        const double coulomb_wheel = cfg.motor.gear_ratio * cfg.motor.coulomb;  // efficiency 1
        const double a = cfg.motor.gear_ratio * cfg.motor.kt / cfg.motor.resistance;
        const double v_small = 0.5 * coulomb_wheel / a;
        run(model, s, v_small, v_small, 1.0);
        CHECK(s.wheel_speed[0] == 0.0);
        CHECK(s.pose.x == 0.0);

        run(model, s, 3.0, 3.0, 0.5);  // well above stiction: moves
        CHECK(s.pose.x > 0.1);
        run(model, s, 0.0, 0.0, 2.0);  // coasts to a full stop, never reverses
        CHECK(s.wheel_speed[0] == 0.0);
        CHECK(s.wheel_speed[1] == 0.0);
    }

    TEST_CASE("dynamics: wheel angles integrate wheel speed (encoder source)") {
        const RobotConfig cfg = ideal_robot();
        DriveModel model(cfg);
        DriveState s;
        run(model, s, 3.0, 3.0, 1.0);
        const double dist = s.pose.x;
        CHECK(s.wheel_angle[0] * cfg.wheel_radius == doctest::Approx(dist).epsilon(1e-9));
        CHECK(s.wheel_angle[1] * cfg.wheel_radius == doctest::Approx(dist).epsilon(1e-9));
    }

}  // TEST_SUITE
