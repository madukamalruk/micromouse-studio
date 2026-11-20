// Slice 7: mms stdio compatibility mode — protocol, stats/score, physics motion, process.
#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "maze_gen.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/maze_io.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/core/world.hpp"
#include "sim/host/annotations.hpp"
#include "sim/host/mms_bridge.hpp"
#include "sim/host/mms_protocol.hpp"
#include "sim/host/session.hpp"
#include "sim/platform/process.hpp"
#include "test_paths.hpp"

using namespace mmsim;
using namespace mmsim::core;
using host::MmsBridge;
using host::MmsStats;
using host::SemiPose;

constexpr double kPi = 3.14159265358979323846;

namespace {

RobotConfig robot() {
    auto r = load_robot_config(test_paths::source_dir() / "config/robots/generic_classic.json");
    REQUIRE(r.ok());
    return *r.config;
}

/// 1 x 6 corridor, start (0,0) facing north, goal (0,5).
Maze corridor() {
    Maze m(1, 6);
    m.add_boundary();
    m.set_start({0, 0}, Dir::N);
    m.set_goals({{0, 5}});
    return m;
}

/// 3 x 3: start (0,0) N, goal (2,2). Wall east of (0,0) and north of (1,1).
Maze small() {
    Maze m(3, 3);
    m.add_boundary();
    m.set_wall(0, 0, Dir::E, true);
    m.set_wall(1, 1, Dir::N, true);
    m.set_start({0, 0}, Dir::N);
    m.set_goals({{2, 2}});
    return m;
}

struct Rig {
    World world;
    host::Annotations ann;
    MmsBridge bridge;
    explicit Rig(const Maze& m, host::MmsOptions o = {})
        : world(m, robot()), ann(m.width(), m.height()), bridge(world, ann, o) {}

    /// Sends one line and runs ticks until a response arrives (or max_ticks); "" = none.
    std::string cmd(const std::string& line, int max_ticks = 20000) {
        bridge.feed_line(line);
        for (int i = 0; i < max_ticks; ++i) {
            bridge.tick();
            auto r = bridge.take_responses();
            if (!r.empty()) {
                REQUIRE(r.size() == 1);
                return r[0];
            }
            if (bridge.idle()) return "";
        }
        return "<timeout>";
    }
};

std::filesystem::path solver_exe() {
    return std::filesystem::path(MMSIM_TEST_SOLVER);
}

}  // namespace

TEST_SUITE("mms") {
    TEST_CASE("mms stats: effective distance, runs, penalty, score (hand-checked)") {
        MmsStats s;
        s.reset_all();
        CHECK(s.get("score") == "2000");
        CHECK(s.get("best-run-turns") == "-1");  // no finished run yet
        CHECK(s.get("best-run-distance") == "-1");
        CHECK(s.get("total-distance") == "0");
        CHECK_FALSE(s.has("nonsense"));

        // Leave the start: run starts. moveForward 1 = 2 half-steps (eff 2), moveForward 3 =
        // 6 half-steps (eff 6/2 + 1 = 4), one turn.
        s.start_run();
        s.add_distance(2);
        s.add_distance(6);
        s.add_turn();
        CHECK(s.get("total-distance") == "8");
        CHECK(s.get("total-effective-distance") == "6");
        CHECK(s.get("current-run-turns") == "1");
        s.finish_run();  // entered the goal
        CHECK(s.get("best-run-turns") == "1");
        CHECK(s.get("best-run-effective-distance") == "6");
        // score = best turns 1 + best eff 6 + 0.1 * (total turns 1 + total eff 6) = 7.7
        CHECK(s.get("score") == "7.7");

        // Reset by the user: 15-point penalty on the next run's effective distance.
        s.penalize_for_reset();
        s.end_unfinished_run();
        s.start_run();
        CHECK(s.get("current-run-effective-distance") == "15");
        CHECK(s.get("total-effective-distance") == "21");
        s.add_distance(1);  // a half step: eff 1
        s.finish_run();     // 0 turns + 16 eff = 16 > 7 -> best unchanged
        CHECK(s.get("best-run-effective-distance") == "6");
        // 1 + 6 + 0.1 * (1 + 22) = 9.3
        CHECK(s.get("score") == "9.3");
    }

    TEST_CASE("mms wall test: half-steps ahead, edges, diagonals (mms isWall rules)") {
        const Maze m = small();
        // Start cell centre (1,1) facing N: no north wall of (0,0).
        CHECK_FALSE(host::mms_is_wall(m, {1, 1, 0}, 0));
        // East of (0,0) is a wall.
        CHECK(host::mms_is_wall(m, {1, 1, 2}, 0));
        // West = boundary.
        CHECK(host::mms_is_wall(m, {1, 1, 6}, 0));
        // Facing north, 2 half-steps ahead = the north side of (0,1): open; 4 = north of (0,2)
        // = boundary.
        CHECK_FALSE(host::mms_is_wall(m, {1, 1, 0}, 2));
        CHECK(host::mms_is_wall(m, {1, 1, 0}, 4));
        // Diagonal from a cell centre aims at a post.
        CHECK(host::mms_is_wall(m, {1, 1, 1}, 0));
        // On the horizontal edge between (1,0) and (1,1) (semi (3,2)) facing NE: the wall is
        // (1,1) east? -> no wall there; facing NW: (1,1) west = no wall.
        CHECK_FALSE(host::mms_is_wall(m, {3, 2, 1}, 0));
        // On the vertical edge (2,1) facing NE: north wall of (1,0)? no.
        CHECK_FALSE(host::mms_is_wall(m, {2, 1, 1}, 0));
        // On the horizontal edge (3,2) facing E: a post.
        CHECK(host::mms_is_wall(m, {3, 2, 2}, 0));
        // Facing the centre of a cell from an edge: never a wall.
        CHECK_FALSE(host::mms_is_wall(m, {3, 2, 0}, 0));
    }

    TEST_CASE("mms parse: integers like QString::toInt") {
        bool ok = false;
        CHECK(host::mms_to_int("42", ok) == 42);
        CHECK(ok);
        CHECK(host::mms_to_int("+3", ok) == 3);
        CHECK(host::mms_to_int("-7", ok) == -7);
        CHECK(host::mms_to_int("3.5", ok) == 0);
        CHECK_FALSE(ok);
        CHECK(host::mms_to_int("", ok) == 0);
        CHECK_FALSE(ok);
        CHECK(host::mms_to_int("x1", ok) == 0);
        CHECK_FALSE(ok);
    }

    TEST_CASE("mms bridge: queries, maze size, invalid commands get no response") {
        Rig r(small());
        CHECK(r.cmd("mazeWidth") == "3");
        CHECK(r.cmd("mazeHeight") == "3");
        CHECK(r.cmd("wallFront") == "false");
        CHECK(r.cmd("wallRight") == "true");  // east of (0,0)
        CHECK(r.cmd("wallLeft") == "true");   // boundary
        CHECK(r.cmd("wallBack") == "true");
        CHECK(r.cmd("wallFront 5") == "true");    // 5 half-steps away = north boundary
        CHECK(r.cmd("wallFront 3") == "false");   // north side of (0,1)
        CHECK(r.cmd("wallFront x") == "false");   // toInt fails -> 0 -> same as 1
        CHECK(r.cmd("wallFrontLeft") == "true");  // aims at a post
        CHECK(r.cmd("wallFrontRight") == "true");
        CHECK(r.cmd("wallBackLeft") == "true");
        CHECK(r.cmd("wallBackRight") == "true");
        CHECK(r.cmd("wasReset") == "false");
        // Trailing space (mms-cpp sends "moveForward ") and CRLF are fine.
        CHECK(r.cmd("mazeWidth \r") == "3");
        const int before = r.bridge.invalid_count();
        for (const char* bad :
             {"", "fly", "mazeWidth 3", "wallFront 1 2", "getStat", "getStat bogus", "turnLeft 2",
              "setColor 0 0", "setColor 0 0 z", "setWall 0 0 q", "setWall a 0 n", "setColorX 0 0 r",
              "clearAllColor 1", "ackReset now"}) {
            CAPTURE(bad);
            CHECK(r.cmd(bad) == "");
        }
        CHECK(r.bridge.invalid_count() == before + 14);
        CHECK(r.cmd("mazeWidth") == "3");  // still in sync after the bad lines
    }

    TEST_CASE("mms bridge: annotations - colour, text, walls (both sides), clears") {
        Rig r(small());
        r.cmd("setColor 1 2 G");
        r.cmd("setColor 9 9 G");  // outside: ignored
        CHECK(r.ann.color.at({1, 2}) == 'G');
        CHECK(r.ann.color.size() == 1);
        r.cmd("setText 0 1 hello world");  // spaces allowed
        CHECK(r.ann.text.at({0, 1}) == "hello worl");
        r.cmd(
            "setText 2 0 a\x01"
            "b");  // non-printable -> '?'
        CHECK(r.ann.text.at({2, 0}) == "a?b");
        r.cmd("setWall 1 1 n");
        CHECK(r.ann.wall(1, 2, Dir::S) == std::optional<bool>(true));
        r.cmd("clearWall 1 2 s");
        CHECK_FALSE(r.ann.wall(1, 1, Dir::N).has_value());
        r.cmd("clearColor 1 2");
        CHECK(r.ann.color.empty());
        r.cmd("setColor 0 0 r");
        r.cmd("setColor 1 0 b");
        r.cmd("clearAllColor");
        CHECK(r.ann.color.empty());
        r.cmd("clearText 0 1");
        CHECK(r.ann.text.size() == 1);
        r.cmd("clearAllText");
        CHECK(r.ann.text.empty());
    }

    TEST_CASE("mms bridge: moveForward / turns drive the physics to the target pose") {
        Rig r(corridor());
        const double cell = r.world.dims().cell;
        CHECK(r.cmd("moveForward 2") == "ack");
        CHECK(r.bridge.pose().x == 1);
        CHECK(r.bridge.pose().y == 5);  // centre of (0,2)
        const auto& p = r.world.drive().pose;
        CHECK(std::abs(p.x - 0.5 * cell) < 0.003);
        CHECK(std::abs(p.y - 2.5 * cell) < 0.003);
        CHECK(std::abs(std::remainder(p.theta - kPi / 2, 2 * kPi)) < 0.02);
        CHECK(r.world.crashes().empty());
        CHECK(r.cmd("moveForwardHalf") == "ack");
        CHECK(r.bridge.pose().y == 6);
        CHECK(r.cmd("moveForwardHalf 1") == "ack");
        CHECK(r.bridge.pose().y == 7);
        CHECK(r.cmd("turnLeft") == "ack");
        CHECK(r.bridge.pose().dir == 6);  // west
        CHECK(std::abs(std::remainder(r.world.drive().pose.theta - kPi, 2 * kPi)) < 0.02);
        CHECK(r.cmd("turnRight90") == "ack");
        CHECK(r.cmd("turnRight45") == "ack");
        CHECK(r.bridge.pose().dir == 1);  // NE
        CHECK(r.cmd("turnLeft45") == "ack");
        CHECK(r.cmd("turnLeft90") == "ack");
        CHECK(r.cmd("turnRight") == "ack");
        CHECK(r.bridge.pose().dir == 0);
        CHECK(r.cmd("getStat total-distance") == "6");  // 4 + 1 + 1 half-steps
        CHECK(r.cmd("getStat total-turns") == "6");
        CHECK(r.world.crashes().empty());
    }

    TEST_CASE("mms bridge: crash - blocked at once, and doomed after some cells") {
        Rig r(corridor());
        CHECK(r.cmd("moveForward 0") == "crash");    // N < 1
        CHECK(r.cmd("moveForward abc") == "crash");  // toInt fails -> 0
        CHECK(r.cmd("turnLeft") == "ack");
        CHECK(r.cmd("moveForward") == "crash");  // west wall right there
        CHECK(r.bridge.pose().x == 1);
        CHECK(r.bridge.pose().y == 1);
        CHECK_FALSE(r.world.crashes().empty());  // it really touched the wall
        CHECK(r.cmd("turnRight") == "ack");
        // 10 cells asked, 5 possible: drives 5, hits the wall, answers crash, ends at (0,5).
        const auto crashes = r.world.crashes().size();
        CHECK(r.cmd("moveForward 10", 60000) == "crash");
        CHECK(r.bridge.pose().y == 11);
        CHECK(r.world.crashes().size() > crashes);
        const double cell = r.world.dims().cell;
        CHECK(std::abs(r.world.drive().pose.y - 5.5 * cell) < 0.003);
        CHECK(r.cmd("getStat total-distance") == "20");  // mms counts the requested 20
        // Run finished on entering the goal (0,5).
        CHECK(r.cmd("getStat best-run-turns") != "-1");
    }

    TEST_CASE("mms bridge: wasReset / ackReset return to start with a 15-point penalty") {
        Rig r(corridor());
        CHECK(r.cmd("moveForward 2") == "ack");
        r.bridge.press_reset();
        CHECK(r.cmd("wasReset") == "true");
        CHECK(r.cmd("ackReset") == "ack");
        CHECK(r.cmd("wasReset") == "false");
        CHECK(r.bridge.pose().y == 1);
        const double cell = r.world.dims().cell;
        CHECK(std::abs(r.world.drive().pose.y - 0.5 * cell) < 1e-9);
        CHECK(r.cmd("moveForward 5", 60000) == "ack");  // to the goal
        // run: 15 penalty + 10 half-steps (eff 10/2+1 = 6) = 21, 0 turns
        CHECK(r.cmd("getStat best-run-effective-distance") == "21");
        CHECK(r.cmd("getStat best-run-distance") == "10");
        // total eff = 3 (4 half-steps: 4/2+1) + 15 + 6 = 24 -> score = 0 + 21 + 2.4 = 23.4
        CHECK(r.cmd("getStat score") == "23.4");
    }

    TEST_CASE("mms bridge: annotation lines apply immediately, even during a move") {
        Rig r(corridor());
        r.bridge.feed_line("moveForward 3");
        for (int i = 0; i < 50; ++i) r.bridge.tick();
        CHECK_FALSE(r.bridge.idle());
        r.bridge.feed_line("setColor 0 3 y");
        CHECK(r.ann.color.at({0, 3}) == 'y');
        std::vector<std::string> got;
        for (int i = 0; i < 20000 && got.empty(); ++i) {
            r.bridge.tick();
            got = r.bridge.take_responses();
        }
        CHECK(got == std::vector<std::string>{"ack"});
    }

    TEST_CASE("mms bridge: sensor-based wall queries agree with the truth at cell centres") {
        host::MmsOptions o;
        o.walls = host::MmsWallSource::Sensors;
        Rig r(small(), o);
        for (int i = 0; i < 50; ++i) r.bridge.tick();  // let the sensors sample
        CHECK(r.cmd("wallFront") == "false");
        CHECK(r.cmd("wallRight") == "true");
        CHECK(r.cmd("wallLeft") == "true");
        CHECK(r.cmd("moveForward") == "ack");
        for (int i = 0; i < 50; ++i) r.bridge.tick();
        CHECK(r.cmd("wallRight") == "false");  // (0,1) east open
        CHECK(r.cmd("wallLeft") == "true");
        CHECK(r.cmd("wallFront") == "false");
        CHECK(r.cmd("moveForward") == "ack");
        for (int i = 0; i < 50; ++i) r.bridge.tick();
        CHECK(r.cmd("wallFront") == "true");  // north boundary
    }

    TEST_CASE("process: pipes, stderr, exit code, kill, bad exe") {
        platform::ChildProcess p;
        std::string err;
        REQUIRE(p.start(solver_exe(), {"--echo"}, {}, err));
        CHECK(p.running());
        CHECK(p.write("hello\n"));
        auto l = p.read_line(std::chrono::milliseconds(5000));
        REQUIRE(l.has_value());
        CHECK(*l == "hello");
        CHECK(p.write("quit\n"));
        CHECK_FALSE(p.read_line(std::chrono::milliseconds(5000)).has_value());  // EOF
        CHECK(p.wait(std::chrono::milliseconds(5000)));
        CHECK(p.exit_code() == std::optional<int>(3));
        bool saw = false;
        for (const auto& s : p.take_stderr_lines()) saw = saw || s == "echo: bye";
        CHECK(saw);

        platform::ChildProcess hang;
        REQUIRE(hang.start(solver_exe(), {"--hang"}, {}, err));
        CHECK_FALSE(hang.read_line(std::chrono::milliseconds(50)).has_value());
        CHECK(hang.running());
        hang.kill();
        CHECK_FALSE(hang.running());

        platform::ChildProcess none;
        CHECK_FALSE(none.start("Z:/does/not/exist.exe", {}, {}, err));
        CHECK_FALSE(err.empty());
    }

    TEST_CASE("mms session: tiny flood-fill solver reaches the goal on the physics mouse") {
        const Maze m = test_maze::generate(5);
        auto run = [&]() {
            host::Session s(m, robot());
            REQUIRE(s.load_mms(solver_exe(), {}));
            for (int i = 0; i < 400000 && s.mms_running(); ++i) s.step_tick();
            CHECK_FALSE(s.mms_running());
            if (s.mms_exit_code() != std::optional<int>(0))
                for (const auto& l : s.logs()) MESSAGE(l.text);
            if (s.mms_exit_code() != std::optional<int>(0)) {
                for (const auto& h : s.mms()->history()) MESSAGE(h);
                MESSAGE(s.mms()->crash_count());
                MESSAGE(s.time_us());
            }
            CHECK(s.mms_exit_code() == std::optional<int>(0));
            CHECK(s.mms()->crash_count() == 0);
            CHECK(s.mms()->invalid_count() == 0);
            const auto p = s.mms()->pose();
            CHECK(m.is_goal({p.x / 2, p.y / 2}));
            CHECK(s.annotations().text.size() > 50);   // flood values
            CHECK(s.annotations().color.size() > 10);  // visited
            CHECK(s.world().crashes().empty());
            bool done = false;
            for (const auto& l : s.logs())
                done = done || l.text.find("solver: done") != std::string::npos;
            CHECK(done);
            return std::make_pair(s.time_us(), s.trajectory_hash());
        };
        const auto a = run();
        const auto b = run();
        CHECK(a.first > 0);
        CHECK(a == b);  // lockstep: the solver's thinking time costs no sim time
    }

#ifdef MMSIM_MMS_CPP_EXE
    TEST_CASE("mms session: upstream mms-cpp example runs unchanged") {
        const Maze m = test_maze::generate(6);
        host::Session s(m, robot());
        REQUIRE(s.load_mms(MMSIM_MMS_CPP_EXE, {}));
        s.run_ms(60000);
        CHECK(s.mms_running());  // left-wall follower never stops; it aborts on "crash"
        CHECK(s.annotations().color.at({0, 0}) == 'G');
        CHECK(s.annotations().text.at({0, 0}) == "abc");
        CHECK(s.mms()->crash_count() == 0);
        CHECK(s.mms()->moves() > 20);
        CHECK(s.world().crashes().empty());
    }
#endif
}
