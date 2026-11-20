// mmsim-cli — headless batch runner (Slice 8): a firmware DLL or an mms solver across many
// mazes at maximum speed, in parallel worlds; table + JSON/CSV report; non-zero exit code on any
// crash, timeout or failure.
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "sim/cli/cli_args.hpp"
#include "sim/core/maze_io.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/core/version.hpp"
#include "sim/core/world.hpp"
#include "sim/host/batch.hpp"

namespace fs = std::filesystem;
using namespace mmsim;

namespace {

fs::path find_root(const fs::path& exe_dir) {
    for (fs::path start : {exe_dir, fs::current_path()}) {
        fs::path p = start;
        for (int i = 0; i < 8 && !p.empty(); ++i) {
            std::error_code ec;
            if (fs::is_directory(p / "config/robots", ec)) return p;
            if (p == p.parent_path()) break;
            p = p.parent_path();
        }
    }
    return fs::current_path();
}

fs::path find_firmware_dir(const fs::path& exe_dir, const fs::path& root) {
    for (const fs::path& p : {exe_dir / "../../firmware_bin", root / "build/release/firmware_bin",
                              root / "build/dev/firmware_bin"}) {
        std::error_code ec;
        if (fs::is_directory(p, ec)) return fs::weakly_canonical(p, ec);
    }
    return {};
}

/// Physics only (World::step_tick, sensors and IMU included, no firmware): spins in place so
/// the robot never pins against a wall.
double bench_physics_us(const core::Maze& m, const core::RobotConfig& cfg) {
    core::World w(m, cfg);
    w.set_motor_duty(0, -0.3);
    w.set_motor_duty(1, 0.3);
    for (int i = 0; i < 2000; ++i) w.step_tick();  // warm up
    constexpr int kTicks = 200000;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kTicks; ++i) w.step_tick();
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return s * 1e6 / kTicks;
}

bool write_file(const std::string& path, const std::function<void(std::ostream&)>& fn) {
    std::error_code ec;
    const fs::path p(path);
    if (p.has_parent_path()) fs::create_directories(p.parent_path(), ec);
    std::ofstream f(p, std::ios::binary);
    if (!f) return false;
    fn(f);
    return static_cast<bool>(f);
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);
    cli::CliArgs a;
    std::string err;
    if (!cli::parse_cli(args, a, err)) {
        std::fprintf(stderr, "error: %s\n\n%s", err.c_str(), cli::usage());
        return 2;
    }
    if (a.help) {
        std::printf("%s", cli::usage());
        return 0;
    }
    const fs::path exe_dir = fs::absolute(fs::path(argv[0])).parent_path();
    const fs::path root = a.root.empty() ? find_root(exe_dir) : fs::path(a.root);

    host::BatchOptions o;
    o.seed = a.seed;
    o.max_sim_ms = a.max_sim_ms;
    o.jobs = a.jobs;
    o.robot_path = a.config.empty() ? root / "config/robots/generic_classic.json" : a.config;
    auto rc = core::load_robot_config(o.robot_path);
    if (!rc.ok()) {
        std::fprintf(stderr, "error: %s: %s\n", o.robot_path.string().c_str(), rc.error.c_str());
        return 2;
    }
    o.robot = *rc.config;
    if (!a.mms_solver.empty()) {
        o.mms_solver = fs::absolute(a.mms_solver);
        if (a.mms_sensor_walls) o.mms.walls = host::MmsWallSource::Sensors;
    } else {
        const fs::path dir =
            a.firmware_dir.empty() ? find_firmware_dir(exe_dir, root) : fs::path(a.firmware_dir);
        auto fw = host::resolve_firmware(a.firmware, dir, err);
        if (!fw) {
            std::fprintf(stderr, "error: %s\n", err.c_str());
            return 2;
        }
        o.firmware = *fw;
    }
    std::vector<std::string> unmatched;
    o.mazes = host::expand_maze_globs(a.mazes, &unmatched);
    for (const auto& u : unmatched)
        std::fprintf(stderr, "warning: no maze matches %s\n", u.c_str());
    if (o.mazes.empty()) {
        std::fprintf(stderr, "error: no maze files\n");
        return 2;
    }

    std::printf("mmsim-cli %s: %s on %zu maze(s), robot %s, seed %llu, limit %.0f s, %d job(s)\n",
                std::string(core::version()).c_str(),
                (o.mms_solver.empty() ? o.firmware : o.mms_solver).filename().string().c_str(),
                o.mazes.size(), o.robot.name.c_str(), static_cast<unsigned long long>(o.seed),
                static_cast<double>(o.max_sim_ms) / 1000.0, o.jobs);
    std::fflush(stdout);

    const host::BatchReport rep = host::run_batch(o);
    if (a.quiet) {
        std::ostringstream tmp;
        host::write_table(tmp, rep);
        const std::string t = tmp.str();
        std::cout << t.substr(t.find("\n\n") + 1);  // summary + failures
    } else {
        host::write_table(std::cout, rep);
    }
    const double sim = rep.sim_s(), cpu = rep.run_wall_s();
    if (sim > 0)
        std::printf("full tick (physics + firmware/solver): %.2f us per simulated ms\n",
                    cpu * 1e6 / (sim * 1000.0));
    if (a.bench) {
        auto lm = core::load_maze_file(o.mazes.front());
        if (lm.ok())
            std::printf("physics-only tick (World::step_tick, 4 substeps + sensors): %.2f us\n",
                        bench_physics_us(*lm.maze, o.robot));
    }
    int code = rep.failed() > 0 ? 1 : 0;
    if (!a.json.empty()) {
        if (write_file(a.json, [&](std::ostream& f) { host::write_json(f, rep, o); }))
            std::printf("json: %s\n", a.json.c_str());
        else {
            std::fprintf(stderr, "error: could not write %s\n", a.json.c_str());
            code = 2;
        }
    }
    if (!a.csv.empty()) {
        if (write_file(a.csv, [&](std::ostream& f) { host::write_csv(f, rep); }))
            std::printf("csv: %s\n", a.csv.c_str());
        else {
            std::fprintf(stderr, "error: could not write %s\n", a.csv.c_str());
            code = 2;
        }
    }
    return code;
}
