// OBB-vs-AABB collision, time-of-impact clamping, rebound (Slice 3, PRD F7).
#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

#include "sim/core/collision.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/maze_geometry.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/core/world.hpp"
#include "test_paths.hpp"

using namespace mmsim::core;
using std::numbers::pi;

namespace {

RobotConfig robot() {
    auto r = load_robot_config(test_paths::source_dir() / "config/robots/generic_classic.json");
    REQUIRE(r.ok());
    RobotConfig c = *r.config;
    c.slip.enabled = false;  // rigid contact: these tests pin exact no-slip behaviour
    return c;
}

// 1 x 6 corridor, start (0,0) facing north.
Maze corridor() {
    Maze m(1, 6);
    m.add_boundary();
    m.set_start({0, 0}, Dir::N);
    m.set_default_goals();
    return m;
}

// Distance from the robot's front face to the north corridor wall's inner face.
double front_gap(const World& w) {
    const auto& cfg = w.config();
    const Obb box = body_obb(w.drive().pose, cfg.body);
    double max_y = -1e9;
    for (const auto& c : obb_corners(box)) max_y = std::max(max_y, c.y);
    const double wall_face = 6 * MazeDims{}.cell - MazeDims{}.wall / 2.0;
    return wall_face - max_y;
}

// Moves the robot (facing north) so its front face is `gap` metres from the north wall.
// With 0 V on the motors the back-EMF brakes hard (tau ~ 50 ms), so impact tests start close.
void place_before_north_wall(World& w, double gap) {
    auto& s = w.drive_mut();
    const double wall_face = 6 * MazeDims{}.cell - MazeDims{}.wall / 2.0;
    const auto& b = w.config().body;
    s.pose.y = wall_face - gap - (b.offset_x + b.length / 2.0);
}

}  // namespace

TEST_SUITE("physics") {
    TEST_CASE("collision: OBB vs AABB separating-axis test and minimum translation") {
        const Rect wall{0.0, 0.0, 1.0, 0.012};
        Obb box{{0.5, 0.05}, 0.04, 0.03, 0.0};  // bottom at y=0.02, clear of the wall
        CHECK_FALSE(obb_vs_aabb(box, wall).hit);

        box.c.y = 0.035;  // bottom at y=0.005 -> 7 mm overlap
        Contact c = obb_vs_aabb(box, wall);
        REQUIRE(c.hit);
        CHECK(c.depth == doctest::Approx(0.007));
        CHECK(c.normal.x == doctest::Approx(0.0));
        CHECK(c.normal.y == doctest::Approx(1.0));  // pushes the box up, out of the wall

        // Rotated 45 deg: corner pokes into the wall only when close enough.
        Obb diamond{{0.5, 0.012 + 0.04 * std::sqrt(2.0) + 0.001}, 0.04, 0.04, pi / 4};
        CHECK_FALSE(obb_vs_aabb(diamond, wall).hit);
        diamond.c.y -= 0.003;
        CHECK(obb_vs_aabb(diamond, wall).hit);
    }

    TEST_CASE("collision: maze obstacle grid finds walls and posts near the robot") {
        Maze m = corridor();
        CollisionWorld cw(m, MazeDims{});
        const Obb in_middle{{0.09, 0.09 + 0.18 * 2}, 0.04, 0.03, pi / 2};
        CHECK_FALSE(cw.query(in_middle).hit);
        const Obb touching_west{{0.040, 0.45}, 0.04, 0.03, 0.0};  // left edge at 0.000 < 0.006
        CHECK(cw.query(touching_west).hit);
    }

    TEST_CASE(
        "world: open-loop drive into the end wall stops with <= 0.5 mm penetration and one crash") {
        World w(corridor(), robot());
        REQUIRE(w.drive().pose.theta == doctest::Approx(pi / 2));
        w.set_motor_duty(0, 0.5);
        w.set_motor_duty(1, 0.5);
        for (int i = 0; i < 4000; ++i) w.step_tick();  // 4 s
        REQUIRE(w.crashes().size() == 1);
        const CrashEvent& e = w.crashes()[0];
        CHECK(e.penetration <= 0.0005);
        CHECK(e.normal.y == doctest::Approx(-1.0));  // wall pushes robot back south
        CHECK(e.impact_speed > 0.3);
        CHECK(front_gap(w) >= -0.0005);
        CHECK(front_gap(w) < 0.001);
        CHECK(w.drive().pose.x == doctest::Approx(0.09).epsilon(1e-6));  // drove straight
        CHECK(std::abs(w.drive().wheel_speed[0]) < 1e-9);  // clamped: pinned at the wall
        CHECK(w.in_contact());
    }

    TEST_CASE("world: no tunnelling at 5 m/s and 20 m/s") {
        for (double speed : {5.0, 20.0}) {
            CAPTURE(speed);
            World w(corridor(), robot());
            place_before_north_wall(w, 0.020);
            auto& s = w.drive_mut();
            s.wheel_speed[0] = s.wheel_speed[1] = speed / w.config().wheel_radius;
            for (int i = 0; i < 1000 && w.crashes().empty(); ++i) w.step_tick();
            REQUIRE(w.crashes().size() == 1);
            CHECK(w.crashes()[0].impact_speed > 0.6 * speed);
            CHECK(w.crashes()[0].penetration <= 0.0005);
            CHECK(front_gap(w) >= -0.0005);
            CHECK(w.drive().pose.y < 6 * 0.18);  // still inside the maze
        }
    }

    TEST_CASE("world: restitution reverses wheel speeds (rebound)") {
        RobotConfig cfg = robot();
        cfg.collision_restitution = 0.5;
        cfg.motor.coulomb = 0.0;
        cfg.motor.viscous = 0.0;
        World w(corridor(), cfg);
        place_before_north_wall(w, 0.020);
        auto& s = w.drive_mut();
        const double w0 = 1.0 / cfg.wheel_radius;  // 1 m/s coasting, motors at 0 V
        s.wheel_speed[0] = s.wheel_speed[1] = w0;
        for (int i = 0; i < 3000 && w.crashes().empty(); ++i) w.step_tick();
        REQUIRE(w.crashes().size() == 1);
        REQUIRE(w.drive().wheel_speed[0] < 0.0);  // rebound applied in the impact substep
        w.step_tick();
        CHECK(w.drive().wheel_speed[0] < 0.0);  // now moving away from the wall
        CHECK(std::abs(w.drive().wheel_speed[0]) < 0.5 * w0 + 1e-6);
        CHECK_FALSE(w.in_contact());
    }

    TEST_CASE("world: reset returns to the start pose and clears events") {
        World w(corridor(), robot());
        w.set_motor_duty(0, 0.5);
        w.set_motor_duty(1, 0.5);
        for (int i = 0; i < 4000; ++i) w.step_tick();
        REQUIRE_FALSE(w.crashes().empty());
        w.reset();
        CHECK(w.time_us() == 0);
        CHECK(w.crashes().empty());
        CHECK(w.drive().pose.x == doctest::Approx(0.09));
        CHECK(w.drive().pose.y == doctest::Approx(0.09));
        CHECK(w.encoder_count(0) == 0);
    }

    TEST_CASE("world: ideal encoders and gyro follow the wheels") {
        World w(corridor(), robot());
        auto& s = w.drive_mut();
        s.wheel_angle[0] = 2.0 * pi;  // exactly one wheel revolution
        const double counts_per_wheel_rev = w.config().encoder_counts_per_wheel_rev();
        CHECK(w.encoder_count(0) == static_cast<int>(counts_per_wheel_rev));
        s.wheel_speed[0] = -1.0;
        s.wheel_speed[1] = 1.0;  // spin CCW
        CHECK(w.gyro_z_dps() > 0.0);
        CHECK(w.gyro_z_dps() ==
              doctest::Approx(2.0 * w.config().wheel_radius / w.config().track_width * 180.0 / pi));
    }

}  // TEST_SUITE
