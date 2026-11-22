#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iterator>
#include <sstream>

#include "sim/app/app_state.hpp"
#include "sim/app/attitude.hpp"
#include "sim/app/user_paths.hpp"
#include "sim/core/analysis/metrics.hpp"
#include "sim/core/analysis/report.hpp"
#include "sim/core/maze_io.hpp"
#include "sim/core/sha256.hpp"
#include "sim/core/version.hpp"

namespace mmsim::app {

namespace fs = std::filesystem;
namespace an = core::analysis;

fs::path find_root_from(const std::vector<fs::path>& starts) {
    return find_data_root(starts);
}

std::vector<MazeGroup> scan_library(const fs::path& root) {
    std::vector<MazeGroup> groups;
    const std::pair<const char*, fs::path> dirs[] = {
        {"Test mazes", root / "mazes/test"},
        {"Classic 16x16 (mazefiles)", root / "external/mazefiles/classic"},
        {"Half-size (mazefiles)", root / "external/mazefiles/halfsize"},
        {"Training (mazefiles)", root / "external/mazefiles/training"},
    };
    for (const auto& [label, dir] : dirs) {
        MazeGroup g{label, {}};
        std::error_code ec;
        if (fs::is_directory(dir, ec))
            for (const auto& e : fs::directory_iterator(dir, ec)) {
                const auto ext = e.path().extension().string();
                if (ext == ".txt" || ext == ".num" || ext == ".maz") g.files.push_back(e.path());
            }
        std::sort(g.files.begin(), g.files.end());
        groups.push_back(std::move(g));
    }
    return groups;
}

fs::path find_firmware_dir(const fs::path& root, const fs::path& exe_dir) {
    std::vector<fs::path> c;
    if (!exe_dir.empty()) {
        c.push_back(exe_dir / "firmware_bin");  // portable package: next to the exe
        c.push_back(exe_dir / "../../firmware_bin");
    }
    c.push_back(root / "build/dev/firmware_bin");
    c.push_back(root / "build/release/firmware_bin");
    for (const auto& p : c) {
        std::error_code ec;
        if (fs::is_directory(p, ec)) return fs::weakly_canonical(p, ec);
    }
    return {};
}

std::vector<fs::path> scan_firmware(const fs::path& dir) {
    std::vector<fs::path> out;
    std::error_code ec;
    if (!dir.empty() && fs::is_directory(dir, ec))
        for (const auto& e : fs::directory_iterator(dir, ec))
            if (e.path().extension() == ".dll") out.push_back(e.path());
    std::sort(out.begin(), out.end());
    return out;
}

std::string file_sha256(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    if (!in) return {};
    const std::string bytes((std::istreambuf_iterator<char>(in)), {});
    return core::sha256_hex(bytes);
}

std::string short_hash(const std::string& h) {
    return h.size() > 16 ? h.substr(0, 16) : h;
}

core::Cell cell_of(const core::Pose& p, const core::MazeDims& d) {
    return {static_cast<int>(std::floor(p.x / d.cell)), static_cast<int>(std::floor(p.y / d.cell))};
}

void rebuild_trace(AppState& st) {
    st.trace.clear();
    if (!st.session) return;
    const auto& r = st.session->recorder();
    const auto *x = r.channel("sim.x"), *y = r.channel("sim.y"), *v = r.channel("sim.v");
    if (!x || !y || !v) return;
    const std::size_t n = std::min({x->size(), y->size(), v->size()});
    for (std::size_t i = 0; i < n; i += 5)
        st.trace.add({x->v[i], y->v[i], 0.0}, static_cast<double>(v->v[i]) * 1e-3);
}

void make_session(AppState& st) {
    record_attempt(st, "restarted");  // a run that reached the goal but never finished
    st.session.reset();
    st.details.clear();
    st.tracker.reset();
    st.dsync = {};
    st.attempt_recorded = false;
    st.trace.clear();
    st.running = false;
    st.hung = false;
    st.sim_error.clear();
    st.seen_logs = 0;
    st.seen_crashes = 0;
    st.thermal.reset(30.0);
    st.thermal_t_us = -1;
    if (!st.maze || !st.robot) return;
    st.session = std::make_shared<host::Session>(*st.maze, *st.robot, st.dims, st.seed);
    st.session->set_button(0, st.boot_buttons[0]);
    st.session->set_button(1, st.boot_buttons[1]);
    if (!st.mms_solver.empty()) {
        // The GUI steps on a guarded worker (2 s hang detection): keep solver waits short.
        st.session->set_mms_think_timeout(std::chrono::milliseconds(500));
        if (!st.session->load_mms(st.mms_solver, st.mms_opt)) st.sim_error = st.session->error();
    } else if (!st.firmware.empty() && !st.session->load(st.firmware)) {
        st.sim_error = st.session->error();
    }
    st.trace.add(st.session->world().drive().pose, 0.0);
    st.tv.scrub = false;
    st.tv.follow = true;
}

long long advance(AppState& st, long long ms, double budget_s, bool until_cell) {
    if (!st.session || (ms <= 0 && budget_s <= 0.0)) return 0;
    struct Sample {
        core::Pose pose;
        double v;
    };
    auto pts = std::make_shared<std::vector<Sample>>();
    auto ran = std::make_shared<long long>(0);
    const core::MazeDims dims = st.dims;
    const auto r = host::run_guarded(
        st.session,
        [ms, budget_s, pts, ran, until_cell, dims](host::Session& s) {
            const auto t0 = std::chrono::steady_clock::now();
            const long long limit = budget_s > 0.0 ? 10000000 : ms;
            const core::Cell c0 = cell_of(s.world().drive().pose, dims);
            for (long long i = 0; i < limit; ++i) {
                s.step_tick();
                ++*ran;
                const auto& d = s.world().drive();
                if (s.time_us() % 5000 == 0) pts->push_back({d.pose, d.v});
                if (until_cell && !(cell_of(d.pose, dims) == c0)) break;
                if (budget_s > 0.0 && i % 16 == 15 &&
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() >=
                        budget_s)
                    break;
            }
        },
        std::chrono::milliseconds(2000 + std::max(0ll, ms) +
                                  static_cast<long long>(budget_s * 1000.0)));
    if (r == host::GuardResult::Faulted) {
        st.sim_error = "Simulation step failed: " + host::guard_last_error() +
                       ". Session abandoned; press RESET to reload.";
        st.session.reset();
        st.running = false;
        st.hung = true;
        return 0;
    }
    if (r == host::GuardResult::Hung) {
        st.sim_error =
            "Firmware hung: no HAL call for 2 s (endless loop without "
            "hal_delay_ms/hal_wait_tick?). "
            "Session abandoned; press RESET to reload." +
            (host::guard_leaked_threads() > 1
                 ? " Warning: " + std::to_string(host::guard_leaked_threads()) +
                       " hung firmware threads are still burning CPU; restart the app."
                 : std::string());
        st.session.reset();
        st.running = false;
        st.hung = true;
        return 0;
    }
    for (const auto& p : *pts) st.trace.add(p.pose, p.v);
    st.trace.add(st.session->world().drive().pose, st.session->world().drive().v);
    // Thermal estimate (Tuning): integrate over the simulated time that just ran.
    const auto& w = st.session->world();
    const std::int64_t now = st.session->time_us();
    if (st.thermal_t_us >= 0 && now > st.thermal_t_us)
        st.thermal.update(static_cast<double>(now - st.thermal_t_us) * 1e-6, w.battery_voltage(),
                          std::abs(w.motor_current(0)) + std::abs(w.motor_current(1)));
    st.thermal_t_us = now;
    return *ran;
}

void rewind_to(AppState& st, double t_s) {
    if (!st.session) return;
    const auto t_us = static_cast<std::int64_t>(std::llround(t_s * 1000.0)) * 1000;
    const auto r = host::run_guarded(
        st.session, [t_us](host::Session& s) { s.rewind_to(t_us); },
        std::chrono::milliseconds(5000 + t_us / 1000));
    if (r != host::GuardResult::Completed) {
        st.sim_error = r == host::GuardResult::Faulted
                           ? "Rewind failed: " + host::guard_last_error() + ". Press RESET."
                           : "Firmware hung during rewind; session abandoned. Press RESET.";
        st.session.reset();
        st.hung = true;
        return;
    }
    rebuild_trace(st);
    st.thermal.reset(30.0);
    st.thermal_t_us = st.session->time_us();
    st.seen_logs = st.session->logs().size();
    st.seen_crashes = st.session->world().crashes().size();
}

void set_running(AppState& st, bool on) {
    st.running = on && st.session != nullptr;
    if (st.timer_with_go) {
        if (st.running &&
            (st.timer.state() == TimerState::Idle || st.timer.state() == TimerState::Paused)) {
            st.timer.start();
            st.timer_note.clear();
        } else if (!on && st.timer.state() == TimerState::Running) {
            st.timer.pause();
        }
    }
    st.pacer.reset();
    st.window_wall = 0.0;
    st.window_sim_ms = 0;
    if (st.running) st.tv.scrub = false;
}

void load_maze(AppState& st, const fs::path& p) {
    auto r = core::load_maze_file(p);
    st.maze_path = p;
    if (!r.ok()) {
        st.load_error = r.error.to_string();
        return;
    }
    st.load_error.clear();
    st.maze = std::move(*r.maze);
    st.flood = core::flood_distances(*st.maze);
    st.report = core::validate(*st.maze);
    for (int i = 0; i < ui::kWorkspaceCount; ++i) {
        st.need_fit[i] = true;
        st.user_moved[i] = false;
    }
    st.inspect.reset();
    make_session(st);
}

void select_firmware(AppState& st, const fs::path& dll) {
    st.mms_solver.clear();
    st.firmware = dll;
    make_session(st);
}

bool export_csv(const host::TelemetryRecorder& rec, const fs::path& out, double t0, double t1,
                const std::vector<std::string>& channels, std::size_t& rows) {
    std::error_code ec;
    if (out.has_parent_path()) fs::create_directories(out.parent_path(), ec);
    std::ofstream f(out, std::ios::binary);
    if (!f) return false;
    rows = host::write_csv(rec, f, t0, t1, channels);
    return static_cast<bool>(f);
}

fs::path timestamped(const fs::path& dir, const char* prefix, const char* ext) {
    std::time_t t = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof buf, "-%Y%m%d-%H%M%S", std::localtime(&t));
    return dir / (std::string(prefix) + buf + ext);
}

std::string now_stamp() {
    std::time_t t = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof buf, "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    return buf;
}

void toast(AppState& st, std::string text, ui::Tone tone) {
    if (text.empty()) return;
    st.toasts.push_back({std::move(text), tone, 0.0});
    while (st.toasts.size() > 4) st.toasts.pop_front();
}

const char* source_label(const AppState& st) {
    static std::string s;
    if (!st.mms_solver.empty())
        s = st.mms_solver.stem().string() + " (mms)";
    else if (!st.firmware.empty())
        s = ui::firmware_info(st.firmware.stem().string()).label;
    else
        s = "No firmware";
    return s.c_str();
}

ui::MissionStatus current_status(const AppState& st) {
    ui::MissionInput in;
    in.have_maze = st.maze.has_value();
    in.have_source = !st.firmware.empty() || !st.mms_solver.empty();
    in.running = st.running;
    in.faulted = st.hung;
    if (st.session) {
        const auto state = st.session->state();
        in.load_error = state == host::FwState::LoadError ||
                        (!st.sim_error.empty() && !st.hung && st.session->time_us() == 0);
        in.faulted = in.faulted || state == host::FwState::Faulted;
        if (st.session->mms() && !st.session->mms_running() && st.session->time_us() > 0) {
            const auto code = st.session->mms_exit_code();
            if (code && *code != 0) in.faulted = true;
        }
        in.time_us = st.session->time_us();
        const auto& c = st.session->world().crashes();
        if (!c.empty()) in.last_crash_us = c.back().time_us;
        for (const auto& l : st.session->logs()) in.logs.push_back(l.text);
    } else if (in.have_maze && in.have_source && !st.sim_error.empty()) {
        in.faulted = true;
    }
    return ui::mission_status(in);
}

Series series(const host::TelemetryRecorder& rec, const char* channel, double t0, double t1) {
    Series s;
    const auto* c = rec.channel(channel);
    if (!c) return s;
    for (std::size_t i = c->lower_bound(t0); i < c->size() && c->t[i] <= t1; ++i) {
        s.t.push_back(c->t[i]);
        s.v.push_back(c->v[i]);
    }
    return s;
}

// --- run archive ------------------------------------------------------------------------------
namespace {
const char* const kArchiveChannels[] = {"sim.x",     "sim.y",       "sim.theta",  "sim.v",
                                        "sim.w",     "sim.gyro_z",  "sim.i_l",    "sim.i_r",
                                        "sim.v_bat", "sim.motor_l", "sim.motor_r"};

double log_time(const host::Session& s, const char* needle) {
    for (const auto& l : s.logs())
        if (l.text.find(needle) != std::string::npos) return static_cast<double>(l.time_us) * 1e-6;
    return -1.0;
}
}  // namespace

an::ArchivedRun archive_current(const AppState& st) {
    an::ArchivedRun run;
    if (!st.session) return run;
    const auto& s = *st.session;
    const auto& rec = s.recorder();
    const auto* base = rec.channel("sim.v");
    const std::string maze = st.maze_path.stem().string();
    std::time_t now = std::time(nullptr);
    char ts[32];
    std::strftime(ts, sizeof ts, "%Y%m%d-%H%M%S", std::localtime(&now));
    run.id = an::make_run_id(ts, maze, s.trajectory_hash());
    run.meta["maze"] = maze;
    run.meta["source"] = source_label(st);
    run.meta["seed"] = std::to_string(st.seed);
    run.meta["outcome"] = current_status(st).text;
    run.meta["trajectory_hash"] = s.trajectory_hash();
    run.meta["firmware_hash"] = s.firmware_hash();
    run.meta["config_hash"] = st.config_hash;
    run.meta["mmsim"] = std::string(core::version());
    run.numbers["sim_s"] = static_cast<double>(s.time_us()) * 1e-6;
    run.numbers["crashes"] = static_cast<double>(s.world().crashes().size());
    const double goal = log_time(s, "goal reached"), sr0 = log_time(s, "speed run start"),
                 sr1 = log_time(s, "speed run done");
    if (goal >= 0) run.numbers["search_s"] = goal;
    if (sr0 >= 0 && sr1 > sr0) run.numbers["speed_run_s"] = sr1 - sr0;
    run.channels.push_back("t");
    for (const char* c : kArchiveChannels)
        if (rec.channel(c)) run.channels.push_back(c);
    run.data.resize(run.channels.size());
    double dist = 0.0, last_t = 0.0;
    if (base)
        for (std::size_t i = 0; i < base->size(); ++i) {
            const double t = base->t[i];
            run.data[0].push_back(static_cast<float>(t));
            for (std::size_t k = 1; k < run.channels.size(); ++k)
                run.data[k].push_back(rec.channel(run.channels[k])->value_at(t));
            if (i > 0) dist += std::abs(static_cast<double>(base->v[i])) * 1e-3 * (t - last_t);
            last_t = t;
        }
    run.numbers["distance_m"] = dist;
    return run;
}

fs::path save_current_run(AppState& st, std::string& err) {
    if (!st.session) {
        err = "no run to save";
        return {};
    }
    const fs::path out = an::save_run(st.user_dir / "runs", archive_current(st), err);
    refresh_archive(st);
    return out;
}

void refresh_archive(AppState& st) {
    st.archive = an::list_runs(st.user_dir / "runs");
    if (st.compare_b >= static_cast<int>(st.archive.size())) st.compare_b = 0;
}

bool load_compare(AppState& st, int index) {
    st.run_b.reset();
    st.run_b_path.clear();
    if (index < 0 || index >= static_cast<int>(st.archive.size())) return false;
    std::string err;
    auto r = an::load_run(st.archive[static_cast<std::size_t>(index)], err);
    if (!r) {
        st.compare_status = err;
        return false;
    }
    st.compare_b = index;
    st.run_b = std::move(*r);
    st.run_b_path = st.archive[static_cast<std::size_t>(index)];
    st.compare_status.clear();
    return true;
}

// --- motion heatmap ---------------------------------------------------------------------------
an::Heatmap motion_heat(const AppState& st) {
    if (!st.session) return {};
    const auto& rec = st.session->recorder();
    const auto *x = rec.channel("sim.x"), *y = rec.channel("sim.y"), *v = rec.channel("sim.v"),
               *w = rec.channel("sim.w");
    if (!x || !y || !v || !w || v->size() < 3) return {};
    std::vector<an::MotionSample> samples;
    const std::size_t n = v->size();
    const std::size_t stride = std::max<std::size_t>(1, n / 20000);
    for (std::size_t i = stride; i < n; i += stride) {
        const double t = v->t[i], dt = t - v->t[i - stride];
        if (dt <= 0) continue;
        const double vel = v->v[i];  // mm/s
        an::MotionSample m;
        m.x_mm = x->value_at(t) * 1000.0;
        m.y_mm = y->value_at(t) * 1000.0;
        m.v = std::abs(vel);
        m.a_long = std::abs((vel - v->v[i - stride]) / dt);
        m.a_lat = std::abs(vel * w->value_at(t) * 3.14159265358979323846 / 180.0);
        samples.push_back(m);
    }
    return an::bin_heatmap(samples, st.dims.cell * 1000.0);
}

// --- report + MATLAB bundle -------------------------------------------------------------------
namespace {
double max_abs(const Series& s) {
    double m = 0;
    for (double v : s.v) m = std::max(m, std::abs(v));
    return m;
}
std::string fmt(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}
an::ReportPlot plot_of(const Series& s, const char* title, const char* y, const char* colour,
                       double scale = 1.0) {
    an::ReportPlot p{title, "time (s)", y, {}};
    an::ReportSeries rs{title, colour, {}, {}};
    const std::size_t stride = std::max<std::size_t>(1, s.t.size() / 600);
    for (std::size_t i = 0; i < s.t.size(); i += stride) {
        rs.x.push_back(s.t[i]);
        rs.y.push_back(s.v[i] * scale);
    }
    p.series.push_back(std::move(rs));
    return p;
}
}  // namespace

an::ReportInput report_input(const AppState& st) {
    an::ReportInput in;
    if (!st.session) return in;
    const auto& s = *st.session;
    const auto& w = s.world();
    const auto& rec = s.recorder();
    char maze_hash[32];
    std::snprintf(maze_hash, sizeof maze_hash, "%016llx",
                  static_cast<unsigned long long>(w.maze().hash()));
    in.title = "mmsim engineering report - " + st.maze_path.stem().string();
    in.generated = now_stamp();
    in.run = {{"Maze", st.maze_path.filename().string()},
              {"Source", source_label(st)},
              {"Robot config", st.config_path.filename().string()},
              {"Seed", std::to_string(st.seed)},
              {"Simulated time", ui::format_sim_time(s.time_us())},
              {"Outcome", current_status(st).text},
              {"Maze hash", maze_hash},
              {"Firmware hash", s.firmware_hash()},
              {"Trajectory hash", s.trajectory_hash()},
              {"mmsim", std::string(core::version())}};

    const Series v = series(rec, "sim.v"), vb = series(rec, "sim.v_bat"),
                 ml = series(rec, "sim.motor_l"), mr = series(rec, "sim.motor_r"),
                 wz = series(rec, "sim.w"), il = series(rec, "sim.i_l"),
                 gz = series(rec, "sim.gyro_z");
    const bool has_fw = !st.firmware.empty() || !st.mms_solver.empty();
    bool goal = false, errored = !st.sim_error.empty() || !w.crashes().empty();
    for (const auto& l : s.logs()) {
        if (l.text.find("goal reached") != std::string::npos) goal = true;
        if (l.text.find("ERROR") != std::string::npos) errored = true;
    }
    // A run that is simply not there yet is "not measured", not a failure.
    in.checks.push_back({"Goal reached",
                         (!has_fw || (!goal && !errored)) ? "NOT MEASURED"
                         : goal                           ? "PASS"
                                                          : "FAIL",
                         goal ? "yes" : (errored ? "no" : "not yet"), "centre found"});
    in.checks.push_back({"No wall crashes", w.crashes().empty() ? "PASS" : "FAIL",
                         std::to_string(w.crashes().size()), "0"});
    const double cap = core::effective_max_voltage(w.config().motor);
    const double vmax = std::max(max_abs(ml), max_abs(mr));
    in.checks.push_back({"Motor voltage cap", vmax <= cap + 1e-6 ? "PASS" : "FAIL",
                         fmt("%.2f V", vmax), fmt("<= %.1f V", cap)});
    double vmin = 1e9;
    for (double x : vb.v) vmin = std::min(vmin, x);
    const double reset_v = w.config().battery.reset_v;
    if (vb.v.empty())
        in.checks.push_back({"Brownout margin", "NOT MEASURED", "-", fmt("> %.1f V", reset_v)});
    else
        in.checks.push_back({"Brownout margin", vmin > reset_v ? "PASS" : "FAIL",
                             fmt("%.2f V min", vmin), fmt("> %.1f V", reset_v)});
    double peak_lat_g = 0;
    for (std::size_t i = 0; i < v.t.size(); ++i)
        peak_lat_g =
            std::max(peak_lat_g, std::abs(v.v[i] * 1e-3 * (i < wz.v.size() ? wz.v[i] : 0.0) *
                                          3.14159265358979323846 / 180.0) /
                                     9.80665);
    const double mu = st.mu;
    in.checks.push_back({"Traction circle", peak_lat_g <= mu ? "PASS" : "FAIL",
                         fmt("%.2f g peak a_c", peak_lat_g), fmt("<= %.2f g (mu g)", mu)});

    std::vector<double> ts(v.t.begin(), v.t.end()), ys(v.v.begin(), v.v.end());
    const auto sm = an::step_metrics(ts, ys);
    in.metrics.push_back({"Peak speed", fmt("%.0f mm/s", max_abs(v))});
    if (sm.ok()) {
        in.metrics.push_back({"Speed step rise 10-90 %", fmt("%.3f s", sm.rise_time_s)});
        in.metrics.push_back({"Speed step settling 2 %", fmt("%.3f s", sm.settling_time_s)});
        in.metrics.push_back({"Speed step overshoot", fmt("%.1f %%", sm.overshoot_pct)});
    } else {
        in.metrics.push_back({"Speed step", sm.error});
    }
    const auto imu = an::imu_stats(gz.v);
    if (imu.stddev)
        in.metrics.push_back({"Gyro z noise (1 sigma)", fmt("%.3f deg/s", *imu.stddev)});
    in.metrics.push_back(
        {"Peak junction temperature (est.)", fmt("%.1f C", st.thermal.peak_max())});
    in.metrics.push_back({"Charge used", fmt("%.1f mAh", w.charge_used_mah())});

    in.plots.push_back(plot_of(v, "Forward speed", "mm/s", "#38BDF8"));
    if (!vb.v.empty()) in.plots.push_back(plot_of(vb, "Battery voltage", "V", "#F59E0B"));
    if (!il.v.empty()) in.plots.push_back(plot_of(il, "Left motor current", "A", "#22C55E"));
    for (const auto& l : s.logs())
        in.events.push_back(fmt("%9.3f s  ", static_cast<double>(l.time_us) * 1e-6) + l.text);
    for (const auto& c : w.crashes())
        in.events.push_back(fmt("%9.3f s  ", static_cast<double>(c.time_us) * 1e-6) +
                            "wall contact at " + fmt("%.2f m/s", c.impact_speed));
    in.notes =
        "Generated offline by mmsim. Same maze + config + firmware + seed reproduces the same "
        "trajectory hash.";
    return in;
}

bool write_report(const AppState& st, const fs::path& out, std::string& err) {
    if (!st.session) {
        err = "no session: load a maze first";
        return false;
    }
    const std::string html = an::build_report(report_input(st));
    std::error_code ec;
    if (out.has_parent_path()) fs::create_directories(out.parent_path(), ec);
    std::ofstream f(out, std::ios::binary);
    f << html;
    if (!f) {
        err = "cannot write " + out.string();
        return false;
    }
    return true;
}

bool write_matlab_bundle(const AppState& st, const fs::path& dir, std::string& err) {
    if (!st.session) {
        err = "no session: load a maze first";
        return false;
    }
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        err = "cannot create " + dir.string();
        return false;
    }
    const auto& rec = st.session->recorder();
    const Series el = series(rec, "sim.enc_l"), er = series(rec, "sim.enc_r");
    const Series v = series(rec, "sim.v");
    std::vector<double> t_ms, wl, wr;
    std::vector<std::pair<std::string, std::vector<double>>> extra = {
        {"v_mms", {}}, {"gyro_dps", {}}, {"i_l_a", {}}, {"i_r_a", {}}, {"v_bat", {}}};
    const auto *gz = rec.channel("sim.gyro_z"), *il = rec.channel("sim.i_l"),
               *ir = rec.channel("sim.i_r"), *vb = rec.channel("sim.v_bat");
    const std::size_t n = std::min(el.t.size(), er.t.size());
    for (std::size_t i = 1; i < n; ++i) {
        const double dt = el.t[i] - el.t[i - 1];
        if (dt <= 0) continue;
        const double t = el.t[i];
        t_ms.push_back(t * 1000.0);
        wl.push_back((el.v[i] - el.v[i - 1]) / dt);
        wr.push_back((er.v[i] - er.v[i - 1]) / dt);
        extra[0].second.push_back(i < v.v.size() ? v.v[i] : 0.0);
        extra[1].second.push_back(gz ? gz->value_at(t) : 0.0);
        extra[2].second.push_back(il ? il->value_at(t) : 0.0);
        extra[3].second.push_back(ir ? ir->value_at(t) : 0.0);
        extra[4].second.push_back(vb ? vb->value_at(t) : 0.0);
    }
    {
        std::ofstream f(dir / "wheel_log.csv", std::ios::binary);
        if (!an::write_matlab_csv(f, t_ms, wl, wr, extra) || !f) {
            err = "cannot write wheel_log.csv";
            return false;
        }
    }
    std::size_t rows = 0;
    if (!export_csv(rec, dir / "telemetry.csv", -1e300, 1e300, {}, rows)) {
        err = "cannot write telemetry.csv";
        return false;
    }
    std::ofstream readme(dir / "README.txt", std::ios::binary);
    readme << "mmsim MATLAB bundle (" << now_stamp() << ")\n"
           << "wheel_log.csv : t_ms, segment, wheel_l_cps, wheel_r_cps, v_mms, gyro_dps, i_l_a, "
              "i_r_a, v_bat\n"
           << "                (encoder counts/s; the Neomouse neomouse_pid_fit.m reads the "
              "first four columns)\n"
           << "telemetry.csv : every recorded channel, one row per sample time\n"
           << "MATLAB        : T = readtable('wheel_log.csv'); plot(T.t_ms, T.wheel_l_cps)\n";
    return static_cast<bool>(readme);
}

}  // namespace mmsim::app
