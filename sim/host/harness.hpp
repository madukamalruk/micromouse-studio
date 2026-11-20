#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "sim/host/session.hpp"

namespace mmsim::host {

/// Limits for one headless run (tests now, mmsim-cli in Slice 8).
struct RunLimits {
    std::int64_t max_sim_ms = 300000;  // simulated-time budget
    /// Stop early when this returns true (checked every `check_every_ms` of sim time).
    std::function<bool(const Session&)> done;
    std::int64_t check_every_ms = 10;
};

struct RunResult {
    enum class Outcome { Done, TimeLimit, Faulted, Halted };
    Outcome outcome = Outcome::TimeLimit;
    std::int64_t sim_ms = 0;
    double wall_s = 0.0;
    std::size_t crashes = 0;
    std::string error;  // firmware fault text, if any
};

const char* to_string(RunResult::Outcome o);

/// Runs an already-loaded session as fast as possible until `done`, a fault, a halt, or the
/// time limit. Deterministic: the result depends only on maze, robot, seed and firmware.
RunResult run_headless(Session& s, const RunLimits& limits);

/// True if any firmware log line contains `needle`.
bool has_log(const Session& s, const std::string& needle);

}  // namespace mmsim::host
