#include "sim/host/harness.hpp"

#include <algorithm>
#include <chrono>

namespace mmsim::host {

const char* to_string(RunResult::Outcome o) {
    switch (o) {
        case RunResult::Outcome::Done:
            return "done";
        case RunResult::Outcome::TimeLimit:
            return "time limit";
        case RunResult::Outcome::Faulted:
            return "faulted";
        case RunResult::Outcome::Halted:
            return "halted";
    }
    return "?";
}

RunResult run_headless(Session& s, const RunLimits& limits) {
    RunResult r;
    const auto t0 = std::chrono::steady_clock::now();
    const std::int64_t start_ms = s.time_us() / 1000;
    const std::int64_t every = std::max<std::int64_t>(1, limits.check_every_ms);
    for (std::int64_t i = 1; i <= limits.max_sim_ms; ++i) {
        s.step_tick();
        if (s.state() == FwState::Faulted) {
            r.outcome = RunResult::Outcome::Faulted;
            r.error = s.error();
            break;
        }
        if (s.state() == FwState::Halted) {
            r.outcome = RunResult::Outcome::Halted;
            break;
        }
        if (limits.done && i % every == 0 && limits.done(s)) {
            r.outcome = RunResult::Outcome::Done;
            break;
        }
    }
    r.sim_ms = s.time_us() / 1000 - start_ms;
    r.wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    r.crashes = s.world().crashes().size();
    return r;
}

bool has_log(const Session& s, const std::string& needle) {
    return std::any_of(s.logs().begin(), s.logs().end(),
                       [&](const LogLine& l) { return l.text.find(needle) != std::string::npos; });
}

}  // namespace mmsim::host
