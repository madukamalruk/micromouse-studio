// Slice 6: deterministic replay (trajectory hash, rewind/scrub), annotation state, stepping.
#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <string>

#include "maze_gen.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/host/annotations.hpp"
#include "sim/host/session.hpp"
#include "test_paths.hpp"

using namespace mmsim;
using namespace mmsim::core;
using host::Session;

namespace {

std::filesystem::path fw(const char* name) {
    return std::filesystem::path(MMSIM_FIRMWARE_DIR) / (std::string(name) + ".dll");
}

RobotConfig robot() {
    auto r = load_robot_config(test_paths::source_dir() / "config/robots/generic_classic.json");
    REQUIRE(r.ok());
    return *r.config;
}

Maze corridor() {
    Maze m(1, 6);
    m.add_boundary();
    m.set_start({0, 0}, Dir::N);
    m.set_default_goals();
    return m;
}

}  // namespace

TEST_SUITE("replay") {
    TEST_CASE("replay: same inputs give the same trajectory hash; another seed differs") {
        const Maze m = test_maze::generate(4);
        auto run = [&](std::uint64_t seed) {
            Session s(m, robot(), {}, seed);
            REQUIRE(s.load(fw("demo_floodfill")));
            s.run_ms(4000);
            return s.trajectory_hash();
        };
        const std::string a = run(1), b = run(1), c = run(2);
        CHECK(a.size() == 64);
        CHECK(a == b);
        CHECK(a != c);
    }

    TEST_CASE("replay: rewind_to(t) reproduces the exact state at t (scrub is deterministic)") {
        Session s(test_maze::generate(4), robot(), {}, 3);
        REQUIRE(s.load(fw("demo_floodfill")));
        s.run_ms(3000);
        const Pose p3 = s.world().drive().pose;
        const std::string h3 = s.trajectory_hash();
        const auto e3 = s.world().encoder_count(0);
        s.run_ms(2500);
        CHECK(s.time_us() == 5500000);
        s.rewind_to(3000000);
        CHECK(s.time_us() == 3000000);
        CHECK(s.world().drive().pose.x == p3.x);
        CHECK(s.world().drive().pose.y == p3.y);
        CHECK(s.world().drive().pose.theta == p3.theta);
        CHECK(s.world().encoder_count(0) == e3);
        CHECK(s.trajectory_hash() == h3);
        // The recorder was rebuilt for the replayed span too.
        const auto* v = s.recorder().channel("sim.v");
        REQUIRE(v);
        CHECK(v->size() == 3000);
    }

    TEST_CASE("replay: built-in channels are sampled every tick, firmware channels as sent") {
        Session s(test_maze::generate(4), robot());
        REQUIRE(s.load(fw("demo_floodfill")));
        s.run_ms(1000);
        for (const char* name : {"sim.v", "sim.w", "sim.x", "sim.y", "sim.theta", "sim.motor_l",
                                 "sim.motor_r", "sim.gyro_z", "sim.enc_l", "sim.enc_r"}) {
            CAPTURE(name);
            const auto* c = s.recorder().channel(name);
            REQUIRE(c);
            CHECK(c->size() == 1000);
            CHECK(c->t.back() == doctest::Approx(1.0));
        }
        CHECK(s.recorder().channel("sim.d.front_left"));
        const auto* vset = s.recorder().channel("v_set");  // firmware, every 10 ms
        REQUIRE(vset);
        CHECK(vset->size() >= 60);
        CHECK(vset->size() <= 101);
        CHECK(s.telemetry_value("v_set").has_value());
    }
}

TEST_SUITE("controls") {
    TEST_CASE("controls: run_ms advances exactly N ticks; zero = paused") {
        Session s(corridor(), robot());
        REQUIRE(s.load(fw("test_blink")));
        s.run_ms(0);
        CHECK(s.time_us() == 0);
        s.run_ms(1);
        CHECK(s.time_us() == 1000);
        s.run_ms(10);
        CHECK(s.time_us() == 11000);
        s.run_ms(100);
        CHECK(s.time_us() == 111000);
    }

    TEST_CASE("annotations: colours, text, walls - validated, last write wins, clear") {
        host::Annotations a(1, 6);
        a.set_color(0, 2, 'G');
        a.set_color(0, 2, 'B');  // last write wins
        a.set_color(0, 3, 'z');  // not an mms colour: ignored
        a.set_color(5, 0, 'R');  // outside the maze: ignored
        CHECK(a.color.size() == 1);
        CHECK(a.color.at({0, 2}) == 'B');

        a.set_text(0, 1, "12345678901234");  // clipped to 10 chars
        CHECK(a.text.at({0, 1}) == "1234567890");
        a.set_text(0, 1, "");  // empty text removes it
        CHECK(a.text.empty());

        // Walls are stored once per segment: (0,1) north == (0,2) south.
        a.set_wall(0, 1, 'n', true);
        a.set_wall(0, 2, 's', false);
        CHECK(a.walls.size() == 1);
        CHECK(a.wall(0, 1, Dir::N) == std::optional<bool>(false));
        CHECK(a.wall(0, 2, Dir::S) == std::optional<bool>(false));
        CHECK_FALSE(a.wall(0, 3, Dir::N).has_value());
        a.set_wall(0, 0, 'x', true);  // bad direction: ignored
        a.set_wall(0, 9, 'n', true);  // outside: ignored
        CHECK(a.walls.size() == 1);
        // Repeating the same reports never grows the map (bounded memory).
        for (int i = 0; i < 1000; ++i) a.set_wall(0, 1, 'n', i % 2);
        CHECK(a.walls.size() == 1);

        std::uint32_t rgb = 0;
        CHECK(host::mms_color_rgb('G', rgb));
        CHECK_FALSE(host::mms_color_rgb('z', rgb));

        a.clear();
        CHECK(a.color.empty());
        CHECK(a.walls.empty());
    }

    TEST_CASE("annotations: the demo firmware's map agrees with the maze") {
        const Maze m = test_maze::generate(4);
        Session s(m, robot());
        REQUIRE(s.load(fw("demo_floodfill")));
        s.run_ms(15000);
        int checked = 0;
        for (int x = 0; x < m.width(); ++x)
            for (int y = 0; y < m.height(); ++y)
                for (Dir d : {Dir::N, Dir::E, Dir::S, Dir::W})
                    if (auto k = s.annotations().wall(x, y, d)) {
                        ++checked;
                        CHECK(*k == m.has_wall(x, y, d));
                    }
        CHECK(checked > 40);
        CHECK(s.annotations().color.size() > 10);  // visited cells
        CHECK(s.annotations().text.size() > 50);   // flood values
    }
}
