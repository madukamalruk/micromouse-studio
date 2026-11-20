#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "sim/core/robot_config.hpp"
#include "sim/host/mms_bridge.hpp"

namespace mmsim::host {

/// What one headless batch (mmsim-cli, Slice 8) runs.
struct BatchOptions {
    std::filesystem::path firmware;    // firmware DLL, or
    std::filesystem::path mms_solver;  // mms-protocol solver exe (exactly one is set)
    MmsOptions mms;
    std::vector<std::filesystem::path> mazes;
    core::RobotConfig robot;
    std::filesystem::path robot_path;  // for the report
    std::uint64_t seed = 1;
    std::int64_t max_sim_ms = 300000;
    int jobs = 1;  // parallel worlds (threads); each world has its own DLL copy / solver process
};

struct MazeRunResult {
    std::filesystem::path maze;
    std::string name;     // file stem
    std::string error;    // maze/firmware load error, firmware fault, "ERROR" log line
    std::string outcome;  // done | time limit | faulted | halted | solver exited | load error
    bool solved = false;  // firmware: speed run done without ERROR; mms: reached the goal
    bool timeout = false;
    std::size_t crashes = 0;  // firmware: physical wall contacts; mms: "crash" replies
    double sim_s = 0;
    double wall_s = 0;
    double search_s = -1, return_s = -1, speed_run_s = -1;  // firmware log markers (-1 = none)
    double distance_m = 0;                                  // path length driven
    std::string mms_score;                                  // mms mode: score as mms prints it
    int mms_moves = 0;
    std::string trajectory_hash;

    bool failed() const { return !solved || timeout || crashes > 0 || !error.empty(); }
};

struct BatchReport {
    std::vector<MazeRunResult> runs;  // in the order of BatchOptions::mazes
    double wall_s = 0;                // elapsed for the whole batch
    int jobs = 1;
    double sim_s() const;
    double run_wall_s() const;  // sum of per-run wall times (= CPU time of the worlds)
    int solved() const;
    int failed() const;
};

/// Expands each pattern: a plain path is taken as is; '*' and '?' are matched in the file
/// name part only (e.g. "external/mazefiles/classic/*.txt"). Sorted, duplicates removed.
/// Patterns matching nothing are reported in `unmatched`.
std::vector<std::filesystem::path> expand_maze_globs(const std::vector<std::string>& patterns,
                                                     std::vector<std::string>* unmatched = nullptr);
bool glob_match(std::string_view pattern, std::string_view name);

/// Runs one maze (deterministic for a given maze, robot, seed and firmware/solver).
MazeRunResult run_one(const BatchOptions& o, const std::filesystem::path& maze);
/// Runs every maze on `jobs` worker threads; results keep the input order.
BatchReport run_batch(const BatchOptions& o);

void write_table(std::ostream& out, const BatchReport& r);
void write_json(std::ostream& out, const BatchReport& r, const BatchOptions& o);
void write_csv(std::ostream& out, const BatchReport& r);

/// Firmware argument: a .dll path, or NAME looked up in `dir` (exact NAME.dll, else the only
/// DLL whose name starts with NAME, so "demo" finds demo_floodfill.dll).
std::optional<std::filesystem::path> resolve_firmware(const std::string& arg,
                                                      const std::filesystem::path& dir,
                                                      std::string& error);

}  // namespace mmsim::host
