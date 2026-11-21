#include "sim/host/batch.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <nlohmann/json.hpp>
#include <set>
#include <thread>

#include "sim/core/maze_io.hpp"
#include "sim/host/session.hpp"

namespace mmsim::host {

namespace fs = std::filesystem;

// --- Globs -------------------------------------------------------------------------------

bool glob_match(std::string_view p, std::string_view s) {
    std::size_t pi = 0, si = 0, star = std::string_view::npos, mark = 0;
    auto lower = [](char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c + 32) : c; };
    while (si < s.size()) {
        if (pi < p.size() && (p[pi] == '?' || lower(p[pi]) == lower(s[si]))) {
            ++pi;
            ++si;
        } else if (pi < p.size() && p[pi] == '*') {
            star = pi++;
            mark = si;
        } else if (star != std::string_view::npos) {
            pi = star + 1;
            si = ++mark;
        } else {
            return false;
        }
    }
    while (pi < p.size() && p[pi] == '*') ++pi;
    return pi == p.size();
}

std::vector<fs::path> expand_maze_globs(const std::vector<std::string>& patterns,
                                        std::vector<std::string>* unmatched) {
    std::set<fs::path> out;
    for (const std::string& pat : patterns) {
        const fs::path p(pat);
        const std::string name = p.filename().string();
        std::size_t found = 0;
        if (name.find_first_of("*?") == std::string::npos) {
            std::error_code ec;
            if (fs::is_regular_file(p, ec)) {
                out.insert(p.lexically_normal());
                ++found;
            }
        } else {
            const fs::path dir = p.has_parent_path() ? p.parent_path() : fs::path(".");
            std::error_code ec;
            if (fs::is_directory(dir, ec))
                for (const auto& e : fs::directory_iterator(dir, ec))
                    if (e.is_regular_file(ec) && glob_match(name, e.path().filename().string())) {
                        out.insert(e.path().lexically_normal());
                        ++found;
                    }
        }
        if (found == 0 && unmatched) unmatched->push_back(pat);
    }
    return {out.begin(), out.end()};
}

std::optional<fs::path> resolve_firmware(const std::string& arg, const fs::path& dir,
                                         std::string& error) {
    const fs::path p(arg);
    std::error_code ec;
    if (p.extension() == ".dll") {
        if (fs::is_regular_file(p, ec)) return p;
        error = "firmware not found: " + arg;
        return std::nullopt;
    }
    if (fs::is_regular_file(dir / (arg + ".dll"), ec)) return dir / (arg + ".dll");
    // Short names for the demo solvers: "dijkstra" -> demo_dijkstra.dll.
    if (fs::is_regular_file(dir / ("demo_" + arg + ".dll"), ec))
        return dir / ("demo_" + arg + ".dll");
    std::vector<fs::path> hits;
    if (fs::is_directory(dir, ec))
        for (const auto& e : fs::directory_iterator(dir, ec)) {
            const std::string stem = e.path().stem().string();
            if (e.path().extension() == ".dll" && stem.rfind(arg, 0) == 0) hits.push_back(e.path());
        }
    if (hits.size() == 1) return hits[0];
    error = hits.empty() ? "no firmware named '" + arg + "' in " + dir.string()
                         : "firmware name '" + arg + "' is ambiguous in " + dir.string();
    return std::nullopt;
}

// --- Runs --------------------------------------------------------------------------------

namespace {

double log_time_s(const Session& s, const char* needle) {
    for (const auto& l : s.logs())
        if (l.text.find(needle) != std::string::npos) return static_cast<double>(l.time_us) * 1e-6;
    return -1;
}

}  // namespace

MazeRunResult run_one(const BatchOptions& o, const fs::path& maze_path) {
    MazeRunResult r;
    r.maze = maze_path;
    r.name = maze_path.stem().string();
    const auto t0 = std::chrono::steady_clock::now();
    auto finish = [&]() -> MazeRunResult {
        r.wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return r;
    };
    auto lm = core::load_maze_file(maze_path);
    if (!lm.ok()) {
        r.outcome = "load error";
        r.error = lm.error.to_string();
        return finish();
    }
    Session s(*lm.maze, o.robot, {}, o.seed);
    const bool mms = !o.mms_solver.empty();
    if (mms ? !s.load_mms(o.mms_solver, o.mms) : !s.load(o.firmware)) {
        r.outcome = "load error";
        r.error = s.error();
        return finish();
    }

    std::size_t logs_seen = 0;
    bool done = false, error_logged = false;
    r.outcome = "time limit";
    core::Pose last = s.world().drive().pose;
    for (std::int64_t ms = 0; ms < o.max_sim_ms && !done; ++ms) {
        s.step_tick();
        const core::Pose& p = s.world().drive().pose;
        r.distance_m += std::hypot(p.x - last.x, p.y - last.y);
        last = p;
        if (mms) {
            if (!s.mms_running()) {
                r.outcome = "solver exited";
                done = true;
            }
            continue;
        }
        if (s.state() == FwState::Faulted) {
            r.outcome = "faulted";
            r.error = s.error();
            break;
        }
        if (s.state() == FwState::Halted) {
            r.outcome = "halted";
            break;
        }
        // New log lines only (the log is bounded, so restart the scan if it was trimmed).
        const auto& logs = s.logs();
        if (logs_seen > logs.size()) logs_seen = 0;
        for (; logs_seen < logs.size(); ++logs_seen) {
            const std::string& t = logs[logs_seen].text;
            if (t.find("ERROR") != std::string::npos) {
                error_logged = true;
                if (r.error.empty()) r.error = t;
                done = true;
            }
            if (t.find("speed run done") != std::string::npos) done = true;
        }
        if (done) r.outcome = "done";
    }

    r.sim_s = static_cast<double>(s.time_us()) * 1e-6;
    r.trajectory_hash = s.trajectory_hash();
    if (mms) {
        const MmsBridge& b = *s.mms();
        r.mms_score = b.stats().get("score");
        r.mms_moves = b.moves();
        r.solved = b.stats().get("best-run-turns") != "-1";
        r.crashes = static_cast<std::size_t>(b.crash_count());
        if (b.invalid_count() > 0 && r.error.empty())
            r.error = std::to_string(b.invalid_count()) + " invalid mms commands";
        if (r.outcome == "solver exited") {
            if (auto c = s.mms_exit_code(); c && *c != 0 && r.error.empty())
                r.error = "solver exit code " + std::to_string(*c);
        }
    } else {
        r.crashes = s.world().crashes().size();
        r.search_s = log_time_s(s, "goal reached");
        const double home = log_time_s(s, "start reached");
        if (home >= 0 && r.search_s >= 0) r.return_s = home - r.search_s;
        const double a = log_time_s(s, "speed run start"), b = log_time_s(s, "speed run done");
        if (a >= 0 && b >= 0) r.speed_run_s = b - a;
        r.solved = b >= 0 && !error_logged;
    }
    r.timeout = r.outcome == "time limit" && !r.solved;
    return finish();
}

BatchReport run_batch(const BatchOptions& o) {
    BatchReport rep;
    rep.jobs = std::max(1, o.jobs);
    rep.runs.resize(o.mazes.size());
    const auto t0 = std::chrono::steady_clock::now();
    std::atomic<std::size_t> next{0};
    auto worker = [&]() {
        for (std::size_t i = next++; i < o.mazes.size(); i = next++)
            rep.runs[i] = run_one(o, o.mazes[i]);
    };
    const int n = static_cast<int>(std::min<std::size_t>(rep.jobs, o.mazes.size()));
    std::vector<std::thread> pool;
    for (int i = 1; i < n; ++i) pool.emplace_back(worker);
    worker();
    for (auto& t : pool) t.join();
    rep.wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return rep;
}

double BatchReport::sim_s() const {
    double t = 0;
    for (const auto& r : runs) t += r.sim_s;
    return t;
}
double BatchReport::run_wall_s() const {
    double t = 0;
    for (const auto& r : runs) t += r.wall_s;
    return t;
}
int BatchReport::solved() const {
    return static_cast<int>(
        std::count_if(runs.begin(), runs.end(), [](auto& r) { return r.solved; }));
}
int BatchReport::failed() const {
    return static_cast<int>(
        std::count_if(runs.begin(), runs.end(), [](auto& r) { return r.failed(); }));
}

// --- Reports -----------------------------------------------------------------------------

namespace {

std::string fmt(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}
std::string secs(double v) {
    return v < 0 ? "-" : fmt("%.1f", v);
}

}  // namespace

void write_table(std::ostream& out, const BatchReport& r) {
    char line[256];
    std::snprintf(line, sizeof line, "%-28s %-13s %6s %7s %7s %7s %8s %7s %5s %7s\n", "maze",
                  "outcome", "solved", "search", "return", "speed", "sim s", "wall s", "crash",
                  "score");
    out << line;
    for (const auto& x : r.runs) {
        std::snprintf(line, sizeof line, "%-28.28s %-13.13s %6s %7s %7s %7s %8.1f %7.2f %5zu %7s\n",
                      x.name.c_str(), x.outcome.c_str(), x.solved ? "yes" : "NO",
                      secs(x.search_s).c_str(), secs(x.return_s).c_str(),
                      secs(x.speed_run_s).c_str(), x.sim_s, x.wall_s, x.crashes,
                      x.mms_score.empty() ? "-" : x.mms_score.c_str());
        out << line;
    }
    const double cpu = r.run_wall_s(), sim = r.sim_s();
    std::snprintf(line, sizeof line,
                  "\n%d/%zu solved, %d failed | sim %.1f s in %.1f s wall (%d jobs) | "
                  "per world %.1fx real time, aggregate %.1fx\n",
                  r.solved(), r.runs.size(), r.failed(), sim, r.wall_s, r.jobs,
                  cpu > 0 ? sim / cpu : 0.0, r.wall_s > 0 ? sim / r.wall_s : 0.0);
    out << line;
    for (const auto& x : r.runs)
        if (x.failed())
            out << "  FAILED " << x.name << ": " << x.outcome
                << (x.crashes ? ", " + std::to_string(x.crashes) + " crash(es)" : "")
                << (x.error.empty() ? "" : ", " + x.error) << "\n";
}

void write_json(std::ostream& out, const BatchReport& r, const BatchOptions& o) {
    using nlohmann::json;
    json j;
    j["tool"] = "mmsim-cli";
    j["firmware"] = o.firmware.empty() ? json(nullptr) : json(o.firmware.filename().string());
    j["mms_solver"] = o.mms_solver.empty() ? json(nullptr) : json(o.mms_solver.filename().string());
    j["mms_walls"] = o.mms.walls == MmsWallSource::Sensors ? "sensors" : "truth";
    j["robot"] = o.robot.name;
    j["seed"] = o.seed;
    j["max_sim_s"] = static_cast<double>(o.max_sim_ms) / 1000.0;
    j["jobs"] = r.jobs;
    j["summary"] = {{"mazes", r.runs.size()},
                    {"solved", r.solved()},
                    {"failed", r.failed()},
                    {"sim_s", r.sim_s()},
                    {"wall_s", r.wall_s},
                    {"per_world_speed", r.run_wall_s() > 0 ? r.sim_s() / r.run_wall_s() : 0.0},
                    {"aggregate_speed", r.wall_s > 0 ? r.sim_s() / r.wall_s : 0.0}};
    json runs = json::array();
    auto opt = [](double v) { return v < 0 ? json(nullptr) : json(v); };
    for (const auto& x : r.runs) {
        runs.push_back({{"maze", x.maze.generic_string()},
                        {"name", x.name},
                        {"outcome", x.outcome},
                        {"solved", x.solved},
                        {"timeout", x.timeout},
                        {"failed", x.failed()},
                        {"crashes", x.crashes},
                        {"error", x.error},
                        {"sim_s", x.sim_s},
                        {"wall_s", x.wall_s},
                        {"search_s", opt(x.search_s)},
                        {"return_s", opt(x.return_s)},
                        {"speed_run_s", opt(x.speed_run_s)},
                        {"distance_m", x.distance_m},
                        {"mms_score", x.mms_score.empty() ? json(nullptr) : json(x.mms_score)},
                        {"mms_moves", x.mms_moves},
                        {"trajectory_sha256", x.trajectory_hash}});
    }
    j["runs"] = std::move(runs);
    out << j.dump(2) << "\n";
}

void write_csv(std::ostream& out, const BatchReport& r) {
    auto q = [](const std::string& s) {
        std::string o = "\"";
        for (char c : s) o += c == '"' ? std::string("\"\"") : std::string(1, c);
        return o + "\"";
    };
    auto num = [](double v) { return v < 0 ? std::string() : fmt("%.3f", v); };
    out << "maze,outcome,solved,timeout,crashes,sim_s,wall_s,search_s,return_s,speed_run_s,"
           "distance_m,mms_score,mms_moves,error,trajectory_sha256\n";
    for (const auto& x : r.runs)
        out << q(x.maze.generic_string()) << ',' << q(x.outcome) << ',' << (x.solved ? 1 : 0) << ','
            << (x.timeout ? 1 : 0) << ',' << x.crashes << ',' << fmt("%.3f", x.sim_s) << ','
            << fmt("%.3f", x.wall_s) << ',' << num(x.search_s) << ',' << num(x.return_s) << ','
            << num(x.speed_run_s) << ',' << fmt("%.3f", x.distance_m) << ',' << x.mms_score << ','
            << x.mms_moves << ',' << q(x.error) << ',' << x.trajectory_hash << '\n';
}

}  // namespace mmsim::host
