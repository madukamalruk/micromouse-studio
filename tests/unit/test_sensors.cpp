// Distance sensors (raycast, IR/ToF models, noise, rate/latency), encoders, IMU, PRNG
// (Slice 4, PRD F8). Expected values come from plain geometry written in the test.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include "sim/core/collision.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/maze_geometry.hpp"
#include "sim/core/rng.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/core/sensors.hpp"
#include "sim/core/world.hpp"
#include "test_paths.hpp"

using namespace mmsim::core;
using std::numbers::pi;

namespace {

constexpr double kCell = 0.180, kHalfWall = 0.006;

// One closed 180 mm cell: inner faces at 0.006 and 0.174 on both axes.
Maze box() {
    Maze m(1, 1);
    m.add_boundary();
    m.set_start({0, 0}, Dir::N);
    m.set_goals({{0, 0}});
    return m;
}

Maze corridor() {
    Maze m(1, 6);
    m.add_boundary();
    m.set_start({0, 0}, Dir::N);
    m.set_default_goals();
    return m;
}

RobotConfig generic() {
    auto r = load_robot_config(test_paths::source_dir() / "config/robots/generic_classic.json");
    REQUIRE(r.ok());
    return *r.config;
}

// Generic robot with an ideal (noise-free, 1 kHz, no latency) sensor layout and no slip.
RobotConfig ideal(const char* preset, SensorType type) {
    RobotConfig c = generic();
    c.slip.enabled = false;
    c.imu = ImuConfig{};
    c.sensors = *sensor_layout_preset(preset, type, c.body);
    for (auto& s : c.sensors) {
        s.noise_sigma = s.noise_rel = 0.0;
        s.rate_hz = 1000.0;
        s.latency_ms = 0.0;
        s.range_min = 0.0;
    }
    return c;
}

// Distance from p along unit d to the first inner face of the box.
double box_distance(Vec2d p, Vec2d d) {
    double t = 1e9;
    if (d.x > 1e-12) t = std::min(t, (kCell - kHalfWall - p.x) / d.x);
    if (d.x < -1e-12) t = std::min(t, (kHalfWall - p.x) / d.x);
    if (d.y > 1e-12) t = std::min(t, (kCell - kHalfWall - p.y) / d.y);
    if (d.y < -1e-12) t = std::min(t, (kHalfWall - p.y) / d.y);
    return t;
}

// World sensor origin/direction computed independently of the sensor code.
void mount(const SensorConfig& s, const Pose& r, Vec2d& o, Vec2d& d) {
    const double c = std::cos(r.theta), sn = std::sin(r.theta);
    o = {r.x + s.x * c - s.y * sn, r.y + s.x * sn + s.y * c};
    const double a = r.theta + s.angle_deg * pi / 180.0;
    d = {std::cos(a), std::sin(a)};
}

}  // namespace

TEST_SUITE("sensors") {
    TEST_CASE("prng: xoshiro256** is deterministic, streams are independent, normal() is N(0,1)") {
        Rng a(42), b(42), c(43);
        for (int i = 0; i < 100; ++i) {
            const auto x = a.next();
            CHECK(x == b.next());
            (void)c.next();
        }
        CHECK(Rng(42).next() != Rng(43).next());
        CHECK(derive_seed(1, "gyro") != derive_seed(1, "ir0"));
        CHECK(derive_seed(1, "gyro") == derive_seed(1, "gyro"));
        CHECK(derive_seed(1, "gyro") != derive_seed(2, "gyro"));

        Rng n(7);
        double sum = 0, sq = 0;
        const int N = 200000;
        for (int i = 0; i < N; ++i) {
            const double x = n.normal();
            sum += x;
            sq += x * x;
        }
        const double mean = sum / N, sd = std::sqrt(sq / N - mean * mean);
        CHECK(std::abs(mean) < 0.01);
        CHECK(sd == doctest::Approx(1.0).epsilon(0.01));
        for (int i = 0; i < 1000; ++i) {
            const double u = n.uniform();
            CHECK((u >= 0.0 && u < 1.0));
        }
    }

    TEST_CASE("raycast: exact distances to walls and posts (+-0.1 mm)") {
        CollisionWorld cw(box(), MazeDims{});
        // Straight up from the centre: north face at 0.174.
        auto h = cw.raycast({0.09, 0.09}, {0.0, 1.0}, 1.0);
        REQUIRE(h.hit);
        CHECK(h.t == doctest::Approx(0.084).epsilon(1e-9));
        CHECK(h.normal.y == doctest::Approx(-1.0));
        // 30 degrees: hits the east face x = 0.174 at t = 0.084 / cos 30.
        const double a = pi / 6;
        h = cw.raycast({0.09, 0.09}, {std::cos(a), std::sin(a)}, 1.0);
        REQUIRE(h.hit);
        CHECK(std::abs(h.t - 0.084 / std::cos(a)) < 1e-4);
        // Out of range -> no hit.
        CHECK_FALSE(cw.raycast({0.09, 0.09}, {0.0, 1.0}, 0.05).hit);
        // Aimed exactly at the NE post's inner corner region: the post (not a gap) stops it.
        const Vec2d o{0.09, 0.09};
        const double t_post = std::hypot(0.174 - 0.09, 0.174 - 0.09);
        h = cw.raycast(o, {std::sqrt(0.5), std::sqrt(0.5)}, 1.0);
        REQUIRE(h.hit);
        CHECK(std::abs(h.t - t_post) < 1e-4);
        // Single rectangle, origin inside -> t = 0.
        CHECK(ray_vs_aabb({0.5, 0.5}, {1, 0}, 1.0, Rect{0, 0, 1, 1}).t == 0.0);
    }

    TEST_CASE("sensors: distance accuracy at the cell centre (ToF, front + side layout)") {
        World w(box(), ideal("front_side", SensorType::ToF));
        REQUIRE(w.sensor_count() == 4);
        w.step_tick();
        const Pose p = w.drive().pose;
        CHECK(p.x == doctest::Approx(0.09));
        CHECK(p.y == doctest::Approx(0.09));
        for (std::size_t ch = 0; ch < 4; ++ch) {
            CAPTURE(ch);
            Vec2d o, d;
            mount(w.sensors().config(ch), p, o, d);
            const double expect = box_distance(o, d);
            CHECK(std::abs(w.sensors().true_distance(ch) - expect) < 1e-4);
            // Perpendicular walls: the cone minimum is the centre ray; 1 mm quantisation.
            CHECK(std::abs(w.sensor_distance_mm(static_cast<int>(ch)) - expect * 1000.0) <=
                  0.5 + 1e-9);
            CHECK(w.sensor_raw(static_cast<int>(ch)) == w.sensor_distance_mm(static_cast<int>(ch)));
        }
        // Front sensors see the north wall: 174 mm - (90 mm + mount x).
        const double front_mount = w.sensors().config(1).x;
        CHECK(w.sensor_distance_mm(1) == std::lround((0.174 - 0.09 - front_mount) * 1000.0));
    }

    TEST_CASE("sensors: 45-degree diagonals hit the analytic wall point") {
        World w(box(), ideal("front_diagonal", SensorType::ToF));
        w.step_tick();
        for (std::size_t ch : {std::size_t{0}, std::size_t{3}}) {
            CAPTURE(ch);
            Vec2d o, d;
            mount(w.sensors().config(ch), w.drive().pose, o, d);
            CHECK(std::abs(w.sensors().true_distance(ch) - box_distance(o, d)) < 1e-4);
            const auto& beam = w.sensors().beam(ch);
            REQUIRE(beam.size() == static_cast<std::size_t>(w.sensors().config(ch).rays));
            CHECK(std::all_of(beam.begin(), beam.end(), [](const BeamRay& r) { return r.hit; }));
        }
    }

    TEST_CASE("sensors: field of view - an off-axis wall inside the cone is seen") {
        // Facing a long corridor: the narrow centre ray sees nothing in range, a wide cone sees
        // the side wall.
        RobotConfig narrow = ideal("front_side", SensorType::ToF);
        for (auto& s : narrow.sensors) {
            s.fov_deg = 0.0;
            s.rays = 1;
            s.range_max = 0.3;
        }
        RobotConfig wide = narrow;
        for (auto& s : wide.sensors) {
            s.fov_deg = 60.0;
            s.rays = 13;
        }
        World a(corridor(), narrow), b(corridor(), wide);
        a.step_tick();
        b.step_tick();
        CHECK(a.sensor_distance_mm(1) == kDistanceNone);  // centre ray: 1 m to the far wall
        CHECK(b.sensor_distance_mm(1) != kDistanceNone);  // edge rays hit the side walls
        CHECK(b.sensor_distance_mm(1) < 300);
    }

    TEST_CASE("sensors: IR reflectance model - monotonic, emitter, calibration inverse") {
        SensorConfig s = default_sensor(SensorType::IR);
        // Calibration inverse of the on-axis model: adc = ambient + gain / (d + d0)^2.
        for (double d : {0.02, 0.05, 0.1, 0.2}) {
            const double adc = s.ir_ambient + s.ir_gain / ((d + s.ir_d0) * (d + s.ir_d0));
            CHECK(std::abs(ir_adc_to_mm(s, adc) - d * 1000.0) <= 1.0);
        }
        CHECK(ir_adc_to_mm(s, s.ir_ambient) == kDistanceNone);

        RobotConfig c = ideal("front_side", SensorType::IR);
        World w(box(), c);
        w.step_tick();
        const auto on = w.sensor_raw(1);
        // Off-axis rays travel further and hit at an angle: IR reads within 3 % of the truth.
        const double truth = w.sensors().true_distance(1) * 1000.0;
        CHECK(std::abs(w.sensor_distance_mm(1) - truth) <= 0.03 * truth + 1.0);
        w.set_emitter(1, false);
        w.step_tick();
        CHECK(w.sensor_raw(1) == static_cast<std::uint16_t>(c.sensors[1].ir_ambient));
        CHECK(w.sensor_distance_mm(1) == kDistanceNone);
        CHECK(on > w.sensor_raw(1));

        // Closer wall -> larger ADC.
        World near(box(), c);
        near.drive_mut().pose.y += 0.02;
        near.step_tick();
        CHECK(near.sensor_raw(1) > on);
    }

    TEST_CASE("sensors: Gaussian noise + quantisation statistics over 10k samples") {
        RobotConfig c = ideal("front_side", SensorType::ToF);
        c.sensors[1].noise_sigma = 2.0;  // mm
        c.sensors[1].quant = 1.0;
        World w(box(), c, MazeDims{}, 1234);
        const int N = 10000;
        double sum = 0, sq = 0;
        w.step_tick();
        const double truth = w.sensors().true_distance(1) * 1000.0;
        for (int i = 0; i < N; ++i) {
            w.step_tick();
            const double x = w.sensor_raw(1);
            CHECK(x == std::round(x));
            sum += x;
            sq += x * x;
        }
        const double mean = sum / N, sd = std::sqrt(sq / N - mean * mean);
        const double expect_sd = std::sqrt(2.0 * 2.0 + 1.0 / 12.0);  // noise + quantisation
        CHECK(std::abs(mean - truth) < 0.1);
        CHECK(sd == doctest::Approx(expect_sd).epsilon(0.02));
    }

    TEST_CASE("sensors: same seed reproduces the stream, another seed does not") {
        RobotConfig c = ideal("front_side", SensorType::ToF);
        for (auto& s : c.sensors) s.noise_sigma = 3.0;
        auto stream = [&](std::uint64_t seed) {
            World w(box(), c, MazeDims{}, seed);
            std::vector<int> v;
            for (int i = 0; i < 200; ++i) {
                w.step_tick();
                v.push_back(w.sensor_raw(0) * 7 + w.sensor_raw(2));
            }
            return v;
        };
        CHECK(stream(5) == stream(5));
        CHECK(stream(5) != stream(6));
        World w(box(), c, MazeDims{}, 5);
        for (int i = 0; i < 50; ++i) w.step_tick();
        const auto before = w.sensor_raw(0);
        w.reset();  // reset re-seeds: the run repeats exactly
        for (int i = 0; i < 50; ++i) w.step_tick();
        CHECK(w.sensor_raw(0) == before);
    }

    TEST_CASE("sensors: sample rate and latency are honoured") {
        RobotConfig c = ideal("front_side", SensorType::ToF);
        c.sensors[1].rate_hz = 50.0;     // a new sample every 20 ms
        c.sensors[1].latency_ms = 30.0;  // delivered 30 ms after it was taken
        World w(box(), c);
        CHECK(w.sensor_raw(1) == 0);  // nothing delivered before the first sample arrives
        for (int i = 0; i < 100; ++i) w.step_tick();
        const auto far = w.sensor_distance_mm(1);
        // Step change: move the robot 20 mm closer to the north wall at t = 100 ms.
        w.drive_mut().pose.y += 0.020;
        int changed_at = -1;
        for (int i = 1; i <= 100 && changed_at < 0; ++i) {
            w.step_tick();
            if (w.sensor_distance_mm(1) != far) changed_at = i;
        }
        // Next sample at t = 120 ms (20 ms after the move), delivered 30 ms later = 50 ms.
        CHECK(changed_at >= 30);
        CHECK(changed_at <= 50);
        CHECK(w.sensor_distance_mm(1) == far - 20);
    }

    TEST_CASE("encoders: counts equal wheel rotation x CPR; inversion; floor quantisation") {
        RobotConfig c = ideal("front_side", SensorType::ToF);
        World w(corridor(), c);
        const double cpr = c.encoder.counts_per_motor_rev * c.motor.gear_ratio;  // 28 * 30 = 840
        CHECK(cpr == 840.0);
        auto& s = w.drive_mut();
        s.wheel_angle[0] = 3.0 * 2.0 * pi;  // 3 revolutions
        s.wheel_angle[1] = -2.0 * pi;
        CHECK(w.encoder_count(0) == 2520);
        CHECK(w.encoder_count(1) == -840);
        s.wheel_angle[0] = 10.6 * 2.0 * pi / cpr;  // 10.6 counts -> 10 (an edge not yet reached)
        s.wheel_angle[1] = -0.3 * 2.0 * pi / cpr;  // -0.3 counts -> -1
        CHECK(w.encoder_count(0) == 10);
        CHECK(w.encoder_count(1) == -1);

        c.encoder.invert_right = true;
        World inv(corridor(), c);
        inv.drive_mut().wheel_angle[1] = 2.0 * 2.0 * pi;
        CHECK(inv.encoder_count(1) == -1680);

        // Driving (no slip): counts track distance / (2 pi r) * CPR.
        World d(corridor(), ideal("front_side", SensorType::ToF));
        d.set_motor_duty(0, 0.4);
        d.set_motor_duty(1, 0.4);
        for (int i = 0; i < 600; ++i) d.step_tick();
        const double dist = d.drive().pose.y - 0.09;
        const double expect = dist / (2.0 * pi * c.wheel_radius) * cpr;
        CHECK(std::abs(d.encoder_count(0) - expect) <= 1.0);
    }

    TEST_CASE("imu: integrated gyro follows the true heading; bias shows as drift") {
        RobotConfig c = ideal("front_side", SensorType::ToF);
        c.imu.gyro_noise_dps = 0.1;
        c.imu.gyro_quant_dps = 0.061;
        World w(corridor(), c, MazeDims{}, 99);
        w.set_motor_duty(0, -0.3);
        w.set_motor_duty(1, 0.3);
        const double theta0 = w.drive().pose.theta;
        double integ = 0.0;
        for (int i = 0; i < 2000; ++i) {
            w.step_tick();
            integ += w.imu().gyro_dps[2] * 1e-3;
        }
        const double truth = (w.drive().pose.theta - theta0) * 180.0 / pi;
        CHECK(truth > 360.0);
        // Sample-and-hold integration of the end-of-tick rate: error ~ rate step at start + noise.
        CHECK(std::abs(integ - truth) < 0.5);

        c.imu.gyro_bias_dps = 1.0;
        World b(corridor(), c, MazeDims{}, 99);
        double drift = 0.0;
        for (int i = 0; i < 2000; ++i) {
            b.step_tick();
            drift += b.imu().gyro_dps[2] * 1e-3;
        }
        CHECK(drift == doctest::Approx(2.0).epsilon(0.1));  // standing still: 1 dps * 2 s
        CHECK(std::abs(b.imu().gyro_dps[0]) < 0.5);
    }

    TEST_CASE("imu: gyro saturates at its range; accelerometer sees gravity, thrust and turns") {
        RobotConfig c = ideal("front_side", SensorType::ToF);
        c.imu.gyro_range_dps = 100.0;
        World w(corridor(), c);
        w.set_motor_duty(0, -0.5);
        w.set_motor_duty(1, 0.5);
        for (int i = 0; i < 500; ++i) w.step_tick();
        CHECK(w.gyro_z_dps() > 100.0);
        CHECK(w.imu().gyro_dps[2] == doctest::Approx(100.0));
        CHECK(w.imu().accel_mps2[2] == doctest::Approx(9.80665));

        World a(corridor(), ideal("front_side", SensorType::ToF));
        a.set_motor_duty(0, 0.5);
        a.set_motor_duty(1, 0.5);
        double v0 = a.model().forward_speed(a.drive());
        a.step_tick();
        double v1 = a.model().forward_speed(a.drive());
        CHECK(a.imu().accel_mps2[0] == doctest::Approx((v1 - v0) / 1e-3).epsilon(0.02));
        CHECK(std::abs(a.imu().accel_mps2[1]) < 1e-9);

        // Steady turn: lateral acceleration = v * w toward the centre (+y for a left turn).
        Maze open(16,
                  16);  // no walls; start mid-floor so the 180 mm-radius circle clears the posts
        open.set_start({8, 8}, Dir::N);
        open.set_goals({{0, 0}});
        World t(open, ideal("front_side", SensorType::ToF));
        t.set_motor_duty(0, 0.2);
        t.set_motor_duty(1, 0.3);
        for (int i = 0; i < 1500; ++i) t.step_tick();
        const double v = t.model().forward_speed(t.drive()), wz = t.model().yaw_rate(t.drive());
        CHECK(t.imu().accel_mps2[1] == doctest::Approx(v * wz).epsilon(0.01));
    }

}  // TEST_SUITE
