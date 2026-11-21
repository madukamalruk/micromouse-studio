// Integration: every demo solver (firmware/demo_*, pure C11 on mmhal + firmware/common) searches
// a 16x16 maze to the centre, returns to the start and does a speed run - with sensor noise,
// tyre slip and the 6 V motor cap - without touching a wall. The wall follower is the odd one
// out: it only solves mazes whose goal touches the outer wall.
#include <doctest/doctest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <initializer_list>
#include <map>
#include <optional>
#include <string>
#include <tuple>

#include "maze_gen.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/maze_geometry.hpp"
#include "sim/core/maze_io.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/host/harness.hpp"
#include "sim/host/session.hpp"
#include "test_paths.hpp"

using namespace mmsim;
using namespace mmsim::core;

namespace {

std::filesystem::path dll(const std::string& fw) {
    return std::filesystem::path(MMSIM_FIRMWARE_DIR) / (fw + ".dll");
}

RobotConfig robot() {
    auto r = load_robot_config(test_paths::source_dir() / "config/robots/generic_classic.json");
    REQUIRE(r.ok());
    return *r.config;
}

std::int64_t log_time_ms(const host::Session& s, const std::string& needle) {
    for (const auto& l : s.logs())
        if (l.text.find(needle) != std::string::npos) return l.time_us / 1000;
    return -1;
}

Cell cell_of(const Pose& p, const MazeDims& d) {
    return {static_cast<int>(std::floor(p.x / d.cell)), static_cast<int>(std::floor(p.y / d.cell))};
}

void dump_logs(const host::Session& s) {
    for (const auto& l : s.logs()) {
        const std::string line = std::to_string(l.time_us / 1000) + " ms  " + l.text;
        MESSAGE(line);
    }
}

// Runs search -> return -> speed run and checks the whole mission.
void solve(const std::string& fw, const Maze& maze, const char* name, std::uint64_t seed = 1,
           std::int64_t budget_ms = 300000) {
    host::Session s(maze, robot(), {}, seed);
    REQUIRE_MESSAGE(s.load(dll(fw)), s.error());
    host::RunLimits lim;
    lim.max_sim_ms = budget_ms;  // whole mission (search + return + speed run)
    lim.done = [](const host::Session& ss) {
        return host::has_log(ss, "speed run done") || host::has_log(ss, "ERROR");
    };
    const host::RunResult r = host::run_headless(s, lim);

    const std::int64_t t_goal = log_time_ms(s, "goal reached");
    const std::int64_t t_home = log_time_ms(s, "start reached");
    const std::int64_t t_run = log_time_ms(s, "speed run done");
    const std::int64_t t_run_start = log_time_ms(s, "speed run start");
    std::printf(
        "[%s] %-10s seed %llu: %s, search %.1f s, return %.1f s, speed run %.1f s, "
        "total %.1f s sim in %.2f s wall, crashes %zu\n",
        fw.c_str(), name, static_cast<unsigned long long>(seed), host::to_string(r.outcome),
        static_cast<double>(t_goal) / 1000.0, static_cast<double>(t_home - t_goal) / 1000.0,
        static_cast<double>(t_run - t_run_start) / 1000.0, static_cast<double>(r.sim_ms) / 1000.0,
        r.wall_s, r.crashes);
    if (r.outcome != host::RunResult::Outcome::Done || host::has_log(s, "ERROR")) dump_logs(s);

    CHECK(r.outcome == host::RunResult::Outcome::Done);
    CHECK_FALSE(host::has_log(s, "ERROR"));
    CHECK(r.crashes == 0);
    CHECK(t_goal > 0);
    CHECK(t_home > t_goal);
    CHECK(t_run > t_home);
    // The speed run (known shortest path, faster profile) beats the exploring search.
    CHECK(t_run - t_run_start < t_goal);
    // Ends in a goal cell.
    CHECK(maze.is_goal(cell_of(s.world().drive().pose, s.world().dims())));
    // Every wall the firmware mapped (its annotations, last word per wall) matches the maze.
    int mapped = 0, wrong = 0;
    for (int x = 0; x < maze.width(); ++x)
        for (int y = 0; y < maze.height(); ++y)
            for (Dir d : {Dir::N, Dir::E, Dir::S, Dir::W})
                if (const auto k = s.annotations().wall(x, y, d)) {
                    ++mapped;
                    if (maze.has_wall(x, y, d) != *k) ++wrong;
                }
    CHECK(mapped > 40);
    CHECK(wrong == 0);
    // Mission goals reported with LEDs.
    CHECK(s.led_events().size() >= 8);
}

void generated(const std::string& fw, std::initializer_list<std::uint64_t> seeds,
               int extra_openings = 20) {
    for (std::uint64_t seed : seeds) {
        CAPTURE(seed);
        solve(fw, test_maze::generate(seed, 16, extra_openings), "generated", seed);
    }
}

std::optional<Maze> uk(const char* file) {
    const auto p = test_paths::mazefiles_dir() / file;
    if (!std::filesystem::exists(p)) {
        MESSAGE("SKIPPED " << file << ": run tools/fetch_mazes.ps1 to fetch mazefiles");
        return std::nullopt;
    }
    auto m = load_maze_file(p);
    REQUIRE_MESSAGE(m.ok(), m.error.to_string());
    return *m.maze;
}

void uk1987(const std::string& fw, std::initializer_list<std::uint64_t> seeds) {
    const auto m = uk("classic/uk1987f.txt");
    if (!m) return;
    for (std::uint64_t seed : seeds) {  // noise seeds
        CAPTURE(seed);
        solve(fw, *m, "uk1987f", seed);  // < 5 min simulated
    }
}

}  // namespace

TEST_SUITE("integration") {
    // --- demo_floodfill: the reference solver, tested the hardest ---------------------------
    TEST_CASE("demo_floodfill: generated 16x16 mazes (3 seeds)") {
        generated("demo_floodfill", {1, 2, 3});
    }
    TEST_CASE("demo_floodfill: UK 1987 final (classic/uk1987f.txt)") {
        uk1987("demo_floodfill", {1, 2, 3});
    }
    TEST_CASE("demo_floodfill: other classic mazes") {
        for (const char* f :
             {"classic/uk1988f.txt", "classic/uk2000f.txt", "classic/uk2017f.txt"}) {
            const auto m = uk(f);
            if (!m) continue;
            CAPTURE(f);
            solve("demo_floodfill", *m, f + 8, 1, 600000);  // longer mazes: 10 min budget
        }
    }

    // --- The other solvers that must solve every maze crash-free ------------------------------
    TEST_CASE("demo_dijkstra: generated 16x16 mazes (3 seeds)") {
        generated("demo_dijkstra", {1, 2, 3});
    }
    TEST_CASE("demo_dijkstra: UK 1987 final") {
        uk1987("demo_dijkstra", {1, 2});
    }

    TEST_CASE("demo_astar: generated 16x16 mazes (3 seeds)") {
        generated("demo_astar", {1, 2, 3});
    }
    TEST_CASE("demo_astar: UK 1987 final") {
        uk1987("demo_astar", {1, 2});
    }

    TEST_CASE("demo_tremaux: generated 16x16 mazes (3 seeds)") {
        generated("demo_tremaux", {1, 2, 3});
    }
    TEST_CASE("demo_tremaux: UK 1987 final") {
        uk1987("demo_tremaux", {1});
    }

    // --- Wall follower: works when the goal touches the outer wall, says so when it cannot ---
    TEST_CASE("demo_wallfollower: perfect mazes (goal attached to the walls)") {
        generated("demo_wallfollower", {1, 2, 3}, /*extra_openings=*/0);
    }
    TEST_CASE("demo_wallfollower: UK 1987 goal island -> loop detected, no crash") {
        const auto m = uk("classic/uk1987f.txt");
        if (!m) return;
        host::Session s(*m, robot(), {}, 1);
        REQUIRE_MESSAGE(s.load(dll("demo_wallfollower")), s.error());
        host::RunLimits lim;
        lim.max_sim_ms = 300000;
        lim.done = [](const host::Session& ss) {
            return host::has_log(ss, "speed run done") || host::has_log(ss, "ERROR");
        };
        const host::RunResult r = host::run_headless(s, lim);
        CHECK(r.outcome == host::RunResult::Outcome::Done);
        CHECK(r.crashes == 0);
        CHECK(host::has_log(s, "left-hand rule is looping"));
        CHECK_FALSE(host::has_log(s, "goal reached"));
    }

    // --- Every solver is deterministic: same seed = identical run ---------------------------
    TEST_CASE("demo solvers: same seed = identical run") {
        const Maze m = test_maze::generate(5);
        for (const char* fw : {"demo_floodfill", "demo_dijkstra", "demo_astar", "demo_tremaux",
                               "demo_wallfollower"}) {
            CAPTURE(fw);
            auto once = [&](std::uint64_t seed) {
                host::Session s(m, robot(), {}, seed);
                REQUIRE(s.load(dll(fw)));
                host::RunLimits lim;
                lim.max_sim_ms = 20000;
                host::run_headless(s, lim);
                return s.world().drive().pose;
            };
            const Pose a = once(9), b = once(9);
            CHECK(a.x == b.x);
            CHECK(a.y == b.y);
            CHECK(a.theta == b.theta);
        }
    }

    // --- They all publish what they think: cell text, colours, telemetry ---------------------
    TEST_CASE("demo solvers: annotations and telemetry show how each one thinks") {
        const Maze m = test_maze::generate(4);
        struct Case {
            const char* fw;
            const char* channel;  // solver-specific telemetry
        };
        for (const Case c : {Case{"demo_floodfill", "cells"}, Case{"demo_dijkstra", "rebuilds"},
                             Case{"demo_astar", "replans"}, Case{"demo_tremaux", "marks"},
                             Case{"demo_wallfollower", "revisits"}}) {
            CAPTURE(c.fw);
            host::Session s(m, robot(), {}, 1);
            REQUIRE(s.load(dll(c.fw)));
            host::RunLimits lim;
            lim.max_sim_ms = 25000;
            host::run_headless(s, lim);
            CHECK(s.annotations().text.size() > 0);
            CHECK(s.annotations().color.size() > 0);
            CHECK(s.telemetry_value(c.channel).has_value());
            CHECK(s.telemetry_value("cells").has_value());
            CHECK(s.telemetry_value("phase").has_value());
            CHECK(host::has_log(s, "algorithm:"));
        }
    }
}
