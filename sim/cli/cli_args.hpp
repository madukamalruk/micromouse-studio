#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mmsim::cli {

struct CliArgs {
    std::string firmware;    // NAME or .dll
    std::string mms_solver;  // exe
    bool mms_sensor_walls = false;
    std::vector<std::string> mazes;  // paths or globs (name part)
    std::string config;              // robot JSON (default: config/robots/generic_classic.json)
    std::uint64_t seed = 1;
    std::int64_t max_sim_ms = 300000;
    int jobs = 1;
    std::string json, csv;  // report files
    std::string root;       // data root (default: auto-detect)
    std::string firmware_dir;
    bool bench = false;  // physics-only tick benchmark
    bool quiet = false;  // no per-maze table
    bool help = false;
};

/// Parses argv[1..]. false + error on bad input. `--mazes` takes every following argument up
/// to the next one starting with "--".
bool parse_cli(const std::vector<std::string>& args, CliArgs& out, std::string& error);

/// "300", "300s", "250ms", "5min" -> milliseconds; nullopt if malformed or negative.
std::optional<std::int64_t> parse_duration_ms(const std::string& s);

const char* usage();

}  // namespace mmsim::cli
