#include "sim/cli/cli_args.hpp"

#include <cmath>
#include <cstdlib>

namespace mmsim::cli {

std::optional<std::int64_t> parse_duration_ms(const std::string& s) {
    if (s.empty()) return std::nullopt;
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || !std::isfinite(v) || v < 0) return std::nullopt;
    const std::string unit = end;
    double ms = 0;
    if (unit.empty() || unit == "s")
        ms = v * 1000.0;
    else if (unit == "ms")
        ms = v;
    else if (unit == "min")
        ms = v * 60000.0;
    else
        return std::nullopt;
    return static_cast<std::int64_t>(std::llround(ms));
}

bool parse_cli(const std::vector<std::string>& a, CliArgs& o, std::string& err) {
    o = CliArgs{};
    for (std::size_t i = 0; i < a.size(); ++i) {
        const std::string& s = a[i];
        auto value = [&](std::string& dst) {
            if (i + 1 >= a.size()) {
                err = s + " needs a value";
                return false;
            }
            dst = a[++i];
            return true;
        };
        std::string v;
        if (s == "--help" || s == "-h") {
            o.help = true;
        } else if (s == "--firmware") {
            if (!value(o.firmware)) return false;
        } else if (s == "--mms-solver") {
            if (!value(o.mms_solver)) return false;
        } else if (s == "--mms-walls") {
            if (!value(v)) return false;
            if (v != "truth" && v != "sensors") {
                err = "--mms-walls must be truth or sensors";
                return false;
            }
            o.mms_sensor_walls = v == "sensors";
        } else if (s == "--mazes") {
            while (i + 1 < a.size() && a[i + 1].rfind("--", 0) != 0) o.mazes.push_back(a[++i]);
            if (o.mazes.empty()) {
                err = "--mazes needs at least one path or pattern";
                return false;
            }
        } else if (s == "--config") {
            if (!value(o.config)) return false;
        } else if (s == "--root") {
            if (!value(o.root)) return false;
        } else if (s == "--firmware-dir") {
            if (!value(o.firmware_dir)) return false;
        } else if (s == "--json") {
            if (!value(o.json)) return false;
        } else if (s == "--csv") {
            if (!value(o.csv)) return false;
        } else if (s == "--seed") {
            if (!value(v)) return false;
            char* end = nullptr;
            o.seed = std::strtoull(v.c_str(), &end, 0);
            if (end == v.c_str() || *end) {
                err = "bad --seed: " + v;
                return false;
            }
        } else if (s == "--max-sim-time") {
            if (!value(v)) return false;
            auto d = parse_duration_ms(v);
            if (!d || *d <= 0) {
                err = "bad --max-sim-time: " + v + " (e.g. 300s, 5min, 250ms)";
                return false;
            }
            o.max_sim_ms = *d;
        } else if (s == "--jobs" || s == "-j") {
            if (!value(v)) return false;
            char* end = nullptr;
            const long n = std::strtol(v.c_str(), &end, 10);
            if (end == v.c_str() || *end || n < 1 || n > 256) {
                err = "bad --jobs: " + v + " (1..256)";
                return false;
            }
            o.jobs = static_cast<int>(n);
        } else if (s == "--bench") {
            o.bench = true;
        } else if (s == "--quiet") {
            o.quiet = true;
        } else {
            err = "unknown argument: " + s;
            return false;
        }
    }
    if (o.help) return true;
    if (o.firmware.empty() == o.mms_solver.empty()) {
        err = "give exactly one of --firmware or --mms-solver";
        return false;
    }
    if (o.mazes.empty()) {
        err = "--mazes is required";
        return false;
    }
    return true;
}

const char* usage() {
    return "mmsim-cli — headless batch runner (max speed, no window)\n"
           "  --firmware NAME|DLL     firmware (NAME = firmware_bin/NAME.dll or a unique prefix)\n"
           "  --mms-solver EXE        or an mms-protocol solver (stdin/stdout)\n"
           "  --mms-walls truth|sensors  mms wall queries (default truth)\n"
           "  --mazes PATH|GLOB...    maze files; '*'/'?' in the file name (quote them)\n"
           "  --config FILE           robot JSON (default config/robots/generic_classic.json)\n"
           "  --seed N                noise seed (default 1)\n"
           "  --max-sim-time T        per-maze simulated-time limit (default 300s)\n"
           "  --jobs N                parallel worlds (default 1)\n"
           "  --json FILE / --csv FILE  write the report\n"
           "  --bench                 also measure the physics-only tick time\n"
           "  --quiet                 summary only (no per-maze table)\n"
           "  --root DIR / --firmware-dir DIR  override auto-detection\n"
           "Exit code: 0 all solved crash-free, 1 any crash/timeout/failure, 2 usage error.\n";
}

}  // namespace mmsim::cli
