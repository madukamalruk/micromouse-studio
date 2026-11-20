// Slice 5 integration: the demo firmware (firmware/demo_floodfill, pure C11 + mmhal) searches a
// 16x16 maze to the centre, returns to the start and does a speed run - with sensor noise,
// tyre slip and the 6 V motor cap - without touching a wall.
#include <doctest/doctest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
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

std::filesystem::path demo_dll() {
    return std::filesystem::path(MMSIM_FIRMWARE_DIR) / "demo_floodfill.dll";
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

// Runs search -> return -> speed run and checks the whole mission.
void solve(const Maze& maze, const char* name, std::uint64_t seed = 1,
           std::int64_t budget_ms = 300000) {
    host::Session s(maze, robot(), {}, seed);
    REQUIRE_MESSAGE(s.load(demo_dll()), s.error());
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
        "[demo] %-10s seed %llu: %s, search %.1f s, return %.1f s, speed run %.1f s, "
        "total %.1f s sim in %.2f s wall, crashes %zu\n",
        name, static_cast<unsigned long long>(seed), host::to_string(r.outcome),
        static_cast<double>(t_goal) / 1000.0, static_cast<double>(t_home - t_goal) / 1000.0,
        static_cast<double>(t_run - t_run_start) / 1000.0, static_cast<double>(r.sim_ms) / 1000.0,
        r.wall_s, r.crashes);
    if (r.outcome != host::RunResult::Outcome::Done || host::has_log(s, "ERROR"))
        for (const auto& l : s.logs()) {
            const std::string line = std::to_string(l.time_us / 1000) + " ms  " + l.text;
            MESSAGE(line);
        }

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
    std::map<std::tuple<int, int, char>, bool> mapped;
    for (const auto& w : s.annotations().walls) mapped[{w.x, w.y, w.dir}] = w.present;
    int wrong = 0;
    for (const auto& [k, present] : mapped) {
        const auto [x, y, dc] = k;
        const Dir d = dc == 'n' ? Dir::N : dc == 'e' ? Dir::E : dc == 's' ? Dir::S : Dir::W;
        if (maze.has_wall(x, y, d) != present) ++wrong;
    }
    CHECK(mapped.size() > 40);
    CHECK(wrong == 0);
    // Mission goals reported with LEDs.
    CHECK(s.led_events().size() >= 8);
}

}  // namespace

TEST_SUITE("integration") {
    TEST_CASE("demo_floodfill: generated 16x16 mazes (3 seeds)") {
        for (std::uint64_t seed : {1ull, 2ull, 3ull}) {
            CAPTURE(seed);
            solve(test_maze::generate(seed), "generated", seed);
        }
    }

    TEST_CASE("demo_floodfill: UK 1987 final (classic/uk1987f.txt)") {
        const auto p = test_paths::mazefiles_dir() / "classic/uk1987f.txt";
        if (!std::filesystem::exists(p)) {
            MESSAGE("SKIPPED: run tools/fetch_mazes.ps1 to fetch mazefiles");
            return;
        }
        auto m = load_maze_file(p);
        REQUIRE_MESSAGE(m.ok(), m.error.to_string());
        for (std::uint64_t seed : {1ull, 2ull, 3ull}) {  // noise seeds
            CAPTURE(seed);
            solve(*m.maze, "uk1987f", seed);  // < 5 min simulated
        }
    }

    TEST_CASE("demo_floodfill: other classic mazes") {
        for (const char* f :
             {"classic/uk1988f.txt", "classic/uk2000f.txt", "classic/uk2017f.txt"}) {
            const auto p = test_paths::mazefiles_dir() / f;
            if (!std::filesystem::exists(p)) continue;
            CAPTURE(f);
            auto m = load_maze_file(p);
            REQUIRE_MESSAGE(m.ok(), m.error.to_string());
            solve(*m.maze, f + 8, 1, 600000);  // longer mazes: 10 min budget
        }
    }

    TEST_CASE("demo_floodfill: same seed = identical run") {
        const Maze m = test_maze::generate(5);
        auto once = [&](std::uint64_t seed) {
            host::Session s(m, robot(), {}, seed);
            REQUIRE(s.load(demo_dll()));
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
