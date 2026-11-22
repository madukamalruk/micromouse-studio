// Linear tyre slip model (Slice 4.7, D7): longitudinal slip velocity with a friction cap,
// lateral slip angle proportional to lateral acceleration (slip gradient), skid limit.
#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

#include "sim/core/drive.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/core/world.hpp"
#include "test_paths.hpp"

using namespace mmsim::core;
using std::numbers::pi;

namespace {

constexpr double kG = 9.80665;
constexpr double kDt = 0.00025;

RobotConfig slip_robot(double mu = 1.0) {
    auto r = load_robot_config(test_paths::source_dir() / "config/robots/generic_classic.json");
    REQUIRE(r.ok());
    RobotConfig c = *r.config;
    c.slip.enabled = true;
    c.slip.mu = mu;
    c.motor.max_voltage = 100.0;  // allow hard steps in tests
    c.motor.allow_unsafe_voltage = true;
    c.sensors.clear();
    c.imu = ImuConfig{};
    return c;
}

// Large open maze, start in the middle: no walls near the robot.
Maze open_floor() {
    Maze m(16, 16);
    m.set_start({8, 8}, Dir::N);
    m.set_goals({{0, 0}});
    return m;
}

void run(DriveModel& m, DriveState& s, double vl, double vr, double seconds) {
    const long n = std::lround(seconds / kDt);
    for (long i = 0; i < n; ++i) m.step(s, vl, vr, kDt);
}

}  // namespace

TEST_SUITE("physics") {
    TEST_CASE("slip: symmetric drive stays exactly straight; steady state has no slip") {
        const RobotConfig c = slip_robot();
        DriveModel m(c);
        REQUIRE(m.slip_enabled());
        DriveState s;
        run(m, s, 3.0, 3.0, 2.0);  // heading 0: sin(0) == 0 exactly, so any drift would show
        CHECK(s.pose.y == 0.0);
        CHECK(s.pose.theta == 0.0);
        CHECK(s.w == 0.0);
        CHECK(s.beta == 0.0);
        CHECK(std::abs(m.slip_velocity(s, 0)) < 1e-6);
        // Same steady speed as the rigid model: tyres carry no force at constant speed.
        RobotConfig rigid = c;
        rigid.slip.enabled = false;
        DriveModel mr(rigid);
        DriveState sr;
        run(mr, sr, 3.0, 3.0, 2.0);
        CHECK(s.v == doctest::Approx(mr.forward_speed(sr)).epsilon(1e-4));
    }

    TEST_CASE("slip: pure spin keeps the axle centre exactly in place") {
        const RobotConfig c = slip_robot();
        DriveModel m(c);
        DriveState s;
        s.pose = {0.5, 0.25, 0.0};
        run(m, s, -2.0, 2.0, 1.0);
        CHECK(s.pose.x == 0.5);
        CHECK(s.pose.y == 0.25);
        CHECK(s.v == 0.0);
        CHECK(s.w > 1.0);
    }

    TEST_CASE("slip: acceleration is capped at mu*g and the wheels spin up (odometry over-reads)") {
        const double mu = 0.2;
        const RobotConfig c = slip_robot(mu);
        DriveModel m(c);
        DriveState s;
        double max_acc = 0.0;
        for (int i = 0; i < 2000; ++i) {  // 0.5 s at 6 V
            const double v0 = s.v;
            m.step(s, 6.0, 6.0, kDt);
            max_acc = std::max(max_acc, (s.v - v0) / kDt);
        }
        CHECK(max_acc <= mu * kG * (1.0 + 1e-6));
        CHECK(max_acc >= mu * kG * 0.99);  // and it actually reaches the limit
        CHECK(m.slip_velocity(s, 0) > 0.05);
        const double odo = s.wheel_angle[0] * c.wheel_radius;
        CHECK(odo > 1.05 * s.pose.x);
        // With good grip the same step does not saturate.
        DriveModel g(slip_robot(1.5));
        DriveState sg;
        double peak = 0.0;
        for (int i = 0; i < 400; ++i) {
            const double v0 = sg.v;
            g.step(sg, 3.0, 3.0, kDt);
            peak = std::max(peak, (sg.v - v0) / kDt);
        }
        CHECK(peak < 1.5 * kG * 0.9);
    }

    TEST_CASE("slip: slip angle = gradient x lateral acceleration in a steady turn") {
        const RobotConfig c = slip_robot();
        DriveModel m(c);
        DriveState s;
        run(m, s, 2.0, 3.0, 1.5);
        const double a_lat = s.v * s.w;
        const double k = c.slip.slip_gradient_deg_per_g * pi / 180.0 / kG;
        CHECK(a_lat > 1.0);
        CHECK(s.beta == doctest::Approx(k * a_lat).epsilon(0.01));
        CHECK(s.beta > 0.0);  // left turn: nose points inside the turn
        // Direction of travel = heading - beta.
        const Pose p0 = s.pose;
        m.step(s, 2.0, 3.0, kDt);
        const double travel = std::atan2(s.pose.y - p0.y, s.pose.x - p0.x);
        const double expect = 0.5 * (p0.theta + s.pose.theta) - s.beta;
        CHECK(std::remainder(travel - expect, 2 * pi) == doctest::Approx(0.0).epsilon(1e-6));
    }

    TEST_CASE("slip: lateral acceleration never exceeds mu*g - the robot skids wide") {
        const double mu = 0.3;
        RobotConfig c = slip_robot(mu);
        World w(open_floor(), c);
        // Fast straight, then a hard pivot command at speed.
        w.set_motor_duty(0, 0.6);
        w.set_motor_duty(1, 0.6);
        for (int i = 0; i < 300; ++i) w.step_tick();
        w.set_motor_duty(0, -0.2);
        w.set_motor_duty(1, 0.8);
        double max_lat = 0.0;
        for (int i = 0; i < 120; ++i) {  // stop before the skid carries it into a post
            w.step_tick();
            max_lat = std::max(max_lat, std::abs(w.true_accel_body().y));
        }
        REQUIRE(w.crashes().empty());
        CHECK(max_lat <= mu * kG * 1.02);
        CHECK(std::abs(w.drive().beta) > c.slip.slip_gradient_deg_per_g * pi / 180.0 * mu);
    }

    TEST_CASE(
        "slip: pinned against a wall the wheels keep spinning (encoders count, pose does not)") {
        Maze m(1, 6);
        m.add_boundary();
        m.set_start({0, 0}, Dir::N);
        m.set_default_goals();
        World w(m, slip_robot(1.0));
        w.set_motor_duty(0, 0.5);
        w.set_motor_duty(1, 0.5);
        for (int i = 0; i < 3000; ++i) w.step_tick();
        REQUIRE(w.crashes().size() == 1);
        const double y = w.drive().pose.y;
        const auto e0 = w.encoder_count(0);
        for (int i = 0; i < 500; ++i) w.step_tick();
        CHECK(w.drive().pose.y == doctest::Approx(y).epsilon(1e-6));
        CHECK(w.encoder_count(0) > e0 + 100);
        CHECK(w.in_contact());
    }

}  // TEST_SUITE
