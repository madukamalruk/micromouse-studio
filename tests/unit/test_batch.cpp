// Slice 8: mmsim-cli batch runner — globs, arguments, parallel determinism, reports, failures.
#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "maze_gen.hpp"
#include "sim/cli/cli_args.hpp"
#include "sim/core/maze_io.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/host/batch.hpp"
#include "test_paths.hpp"

using namespace mmsim;
namespace fs = std::filesystem;

namespace {

core::RobotConfig robot() {
    auto r =
        core::load_robot_config(test_paths::source_dir() / "config/robots/generic_classic.json");
    REQUIRE(r.ok());
    return *r.config;
}

/// Writes generated 16x16 mazes (seeds) as text maps into a fresh temp folder.
std::vector<fs::path> generated_mazes(const std::string& tag, std::initializer_list<int> seeds) {
    const fs::path dir = fs::temp_directory_path() / ("mmsim_batch_" + tag);
    fs::remove_all(dir);
    fs::create_directories(dir);
    std::vector<fs::path> out;
    for (int s : seeds) {
        const fs::path p = dir / ("gen" + std::to_string(s) + ".txt");
        std::ofstream(p, std::ios::binary) << core::write_text_map(test_maze::generate(s));
        out.push_back(p);
    }
    return out;
}

fs::path fw_dir() {
    return fs::path(MMSIM_FIRMWARE_DIR);
}

}  // namespace

TEST_SUITE("batch") {
    TEST_CASE("glob: name wildcards, case-insensitive, sorted, unmatched reported") {
        CHECK(host::glob_match("*.txt", "uk1987f.txt"));
        CHECK(host::glob_match("UK*.TXT", "uk1987f.txt"));
        CHECK(host::glob_match("uk198?f.txt", "uk1987f.txt"));
        CHECK_FALSE(host::glob_match("*.num", "uk1987f.txt"));
        CHECK_FALSE(host::glob_match("uk?.txt", "uk1987f.txt"));
        CHECK(host::glob_match("*", ""));

        const std::string dir = (test_paths::source_dir() / "mazes/test").generic_string();
        std::vector<std::string> missing;
        const auto files = host::expand_maze_globs(
            {dir + "/*.txt", dir + "/tiny4x4.txt", dir + "/*.nothing", dir + "/nope.txt"},
            &missing);
        REQUIRE(files.size() == 2);  // corridor + tiny4x4, the duplicate merged
        CHECK(files[0].filename() == "corridor.txt");
        CHECK(files[1].filename() == "tiny4x4.txt");
        CHECK(missing.size() == 2);
    }

    TEST_CASE("cli arguments: values, multi-pattern --mazes, errors") {
        cli::CliArgs a;
        std::string err;
        REQUIRE(cli::parse_cli(
            {"--firmware", "demo", "--mazes", "a/*.txt", "b.txt", "--jobs", "8", "--json",
             "out.json", "--max-sim-time", "2min", "--seed", "7", "--csv", "o.csv"},
            a, err));
        CHECK(a.firmware == "demo");
        CHECK(a.mazes == std::vector<std::string>{"a/*.txt", "b.txt"});
        CHECK(a.jobs == 8);
        CHECK(a.max_sim_ms == 120000);
        CHECK(a.seed == 7);
        CHECK(a.json == "out.json");
        CHECK(a.csv == "o.csv");
        CHECK(cli::parse_duration_ms("250ms") == 250);
        CHECK(cli::parse_duration_ms("1.5") == 1500);
        CHECK_FALSE(cli::parse_duration_ms("5h").has_value());
        CHECK_FALSE(cli::parse_duration_ms("-1s").has_value());

        CHECK_FALSE(cli::parse_cli({"--mazes", "x.txt"}, a, err));  // no firmware/solver
        CHECK_FALSE(cli::parse_cli({"--firmware", "d", "--mms-solver", "s.exe", "--mazes", "x"}, a,
                                   err));                          // both
        CHECK_FALSE(cli::parse_cli({"--firmware", "d"}, a, err));  // no mazes
        CHECK_FALSE(cli::parse_cli({"--firmware", "d", "--mazes", "x", "--jobs", "0"}, a, err));
        CHECK_FALSE(cli::parse_cli({"--firmware", "d", "--mazes", "x", "--bogus"}, a, err));
        CHECK(err.find("--bogus") != std::string::npos);
        REQUIRE(cli::parse_cli({"--help"}, a, err));
        CHECK(a.help);
    }

    TEST_CASE("firmware names resolve by exact name or unique prefix") {
        std::string err;
        auto p = host::resolve_firmware("demo_flood", fw_dir(), err);  // unique prefix
        REQUIRE(p.has_value());
        CHECK(p->filename() == "demo_floodfill.dll");
        p = host::resolve_firmware("dijkstra", fw_dir(), err);  // short solver name
        REQUIRE(p.has_value());
        CHECK(p->filename() == "demo_dijkstra.dll");
        CHECK_FALSE(host::resolve_firmware("demo", fw_dir(), err).has_value());  // five solvers
        CHECK(err.find("ambiguous") != std::string::npos);
        CHECK(host::resolve_firmware("test_blink", fw_dir(), err).has_value());
        CHECK_FALSE(host::resolve_firmware("test_", fw_dir(), err).has_value());  // ambiguous
        CHECK(err.find("ambiguous") != std::string::npos);
        CHECK_FALSE(host::resolve_firmware("nosuch", fw_dir(), err).has_value());
    }

    TEST_CASE("batch: mms solver on 3 mazes, parallel = serial, JSON/CSV reports") {
        host::BatchOptions o;
        o.mms_solver = MMSIM_TEST_SOLVER;
        o.robot = robot();
        o.mazes = generated_mazes("mms", {5, 6, 7});
        o.max_sim_ms = 400000;
        o.jobs = 1;
        const host::BatchReport serial = host::run_batch(o);
        o.jobs = 3;
        const host::BatchReport par = host::run_batch(o);
        REQUIRE(par.runs.size() == 3);
        for (std::size_t i = 0; i < 3; ++i) {
            CAPTURE(par.runs[i].name);
            CHECK(par.runs[i].solved);
            CHECK(par.runs[i].crashes == 0);
            CHECK(par.runs[i].outcome == "solver exited");
            CHECK(par.runs[i].error.empty());
            CHECK(par.runs[i].mms_score != "2000");
            CHECK(par.runs[i].distance_m > 0.5);
            // Order kept and runs independent of the thread they ran on.
            CHECK(par.runs[i].name == serial.runs[i].name);
            CHECK(par.runs[i].trajectory_hash == serial.runs[i].trajectory_hash);
            CHECK(par.runs[i].sim_s == serial.runs[i].sim_s);
        }
        CHECK(par.failed() == 0);
        CHECK(par.solved() == 3);

        std::ostringstream js;
        host::write_json(js, par, o);
        const auto j = nlohmann::json::parse(js.str());
        CHECK(j["summary"]["mazes"] == 3);
        CHECK(j["summary"]["solved"] == 3);
        CHECK(j["jobs"] == 3);
        CHECK(j["runs"][0]["name"] == "gen5");
        CHECK(j["runs"][0]["search_s"].is_null());  // firmware-only marker

        std::ostringstream csv;
        host::write_csv(csv, par);
        int lines = 0;
        for (char c : csv.str()) lines += c == '\n';
        CHECK(lines == 4);  // header + 3
        std::ostringstream table;
        host::write_table(table, par);
        CHECK(table.str().find("3/3 solved, 0 failed") != std::string::npos);
    }

    TEST_CASE("batch: demo firmware solves; time limit and load errors are failures") {
        host::BatchOptions o;
        std::string err;
        o.firmware = *host::resolve_firmware("floodfill", fw_dir(), err);
        o.robot = robot();
        const auto mazes = generated_mazes("fw", {3});
        o.mazes = {mazes[0], "Z:/no/such/maze.txt"};
        o.jobs = 2;
        const host::BatchReport r = host::run_batch(o);
        REQUIRE(r.runs.size() == 2);
        const auto& ok = r.runs[0];
        CHECK(ok.solved);
        CHECK(ok.outcome == "done");
        CHECK(ok.crashes == 0);
        CHECK(ok.search_s > 0);
        CHECK(ok.return_s > 0);
        CHECK(ok.speed_run_s > 0);
        CHECK_FALSE(ok.failed());
        CHECK(r.runs[1].outcome == "load error");
        CHECK(r.runs[1].failed());
        CHECK(r.failed() == 1);

        // A firmware that never finishes the mission hits the limit: timeout, failed.
        o.firmware = *host::resolve_firmware("test_blink", fw_dir(), err);
        o.mazes = {mazes[0]};
        o.max_sim_ms = 1500;
        const host::BatchReport t = host::run_batch(o);
        CHECK(t.runs[0].timeout);
        CHECK(t.runs[0].failed());
        CHECK(t.runs[0].sim_s == doctest::Approx(1.5));
    }
}
