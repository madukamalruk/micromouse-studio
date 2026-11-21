// v0.2.0 app operations: the run diary (Move & Firmware Details), the race timer + attempt
// table, and robot config loading. Split from app_ops.cpp to keep both files readable.
#include <algorithm>
#include <cctype>
#include <cmath>

#include "sim/app/app_state.hpp"
#include "sim/app/attitude.hpp"
#include "sim/core/robot_config.hpp"

namespace mmsim::app {

namespace fs = std::filesystem;

namespace {
bool has_ci(const std::string& hay, const char* needle) {
    std::string h = hay;
    std::transform(h.begin(), h.end(), h.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return h.find(needle) != std::string::npos;
}
}  // namespace

void sync_details(AppState& st) {
    auto& d = st.dsync;
    if (!st.session) {
        d = {};
        return;
    }
    const host::Session& s = *st.session;
    const double now_s = static_cast<double>(s.time_us()) * 1e-6;
    if (d.session != &s || now_s + 1e-9 < d.last_t || s.logs().size() < d.logs) {
        // New run, or a rewind (the run replays from t = 0): start the diary again.
        st.details.clear();
        st.tracker.reset();
        d = {};
        d.session = &s;
    }
    std::vector<DetailEntry> batch;

    // Motion events from the recorded pose (1 kHz, so no cell is ever skipped).
    const auto& rec = s.recorder();
    const auto *cx = rec.channel("sim.x"), *cy = rec.channel("sim.y"),
               *cth = rec.channel("sim.theta");
    if (cx && cy && cth) {
        const std::size_t n = std::min({cx->size(), cy->size(), cth->size()});
        std::size_t i = st.tracker.started() ? cx->lower_bound(st.tracker.last_t() + 1e-7) : 0;
        for (; i < n; ++i) {
            st.tracker.feed({cx->t[i], static_cast<double>(cx->v[i]), static_cast<double>(cy->v[i]),
                             static_cast<double>(cth->v[i]) * kPiD / 180.0},
                            batch);
        }
    }

    // Raw firmware / solver lines (and the run milestones for the attempt table).
    const auto& logs = s.logs();
    for (std::size_t i = d.logs; i < logs.size(); ++i) {
        const double t = static_cast<double>(logs[i].time_us) * 1e-6;
        const std::string& text = logs[i].text;
        st.details.ingest_log(t, text, batch);
        if (d.goal_t < 0 && has_ci(text, "goal reached")) d.goal_t = t;
        if (d.speed0_t < 0 && has_ci(text, "speed run start")) d.speed0_t = t;
        if (d.speed1_t < 0 && has_ci(text, "speed run done")) d.speed1_t = t;
    }
    d.logs = logs.size();

    // Wall crashes.
    const auto& crashes = s.world().crashes();
    for (std::size_t i = d.crashes; i < crashes.size(); ++i) {
        const double t = static_cast<double>(crashes[i].time_us) * 1e-6;
        const auto c = st.tracker.cell_at(t);
        batch.push_back(DetailsLog::crash_entry(t, crashes[i].impact_speed, c.first, c.second));
    }
    d.crashes = crashes.size();
    d.last_t = std::max(d.last_t, now_s);

    std::stable_sort(batch.begin(), batch.end(),
                     [](const DetailEntry& a, const DetailEntry& b) { return a.t_s < b.t_s; });
    for (auto& e : batch) st.details.add(std::move(e));
}

void clear_details(AppState& st) {
    st.details.clear();
}

void record_attempt(AppState& st, const char* note) {
    if (!st.session || st.attempt_recorded) return;
    sync_details(st);
    const auto& d = st.dsync;
    if (d.goal_t < 0 && d.speed0_t < 0) return;  // nothing worth a row yet
    Attempt a;
    a.search_s = d.goal_t;
    a.crashes = static_cast<int>(st.session->world().crashes().size());
    a.note = note;
    if (d.speed0_t >= 0 && d.speed1_t > d.speed0_t) {
        a.speed_run_s = d.speed1_t - d.speed0_t;
        a.finished = true;
    }
    st.attempts.add(std::move(a));
    st.attempt_recorded = true;
}

TimerCue update_race(AppState& st, double dt_real_s) {
    sync_details(st);
    const double sim = st.session ? static_cast<double>(st.session->time_us()) * 1e-6 : -1.0;
    st.timer.update(dt_real_s, sim);
    if (st.session && !st.attempt_recorded && st.dsync.speed1_t >= 0.0 &&
        st.dsync.speed0_t >= 0.0) {
        record_attempt(st, "");
        if (st.timer.state() == TimerState::Running) {
            st.timer.pause();
            st.timer_note = "Run finished at " + RaceTimer::format(st.timer.elapsed_s());
        }
    }
    const TimerCue cue = st.timer.take_cue();
    if (cue == TimerCue::TimeUp) {
        if (st.session && !st.attempt_recorded) {
            // Time ran out: log what the run had reached, even if that is nothing.
            Attempt a;
            a.search_s = st.dsync.goal_t;
            a.crashes = static_cast<int>(st.session->world().crashes().size());
            a.note = "time up";
            st.attempts.add(std::move(a));
            st.attempt_recorded = true;
        }
        if (st.running) set_running(st, false);
        st.timer_note = "Time is up";
        toast(st, "Time is up", ui::Tone::Warn);
    }
    return cue;
}

std::vector<fs::path> scan_robot_configs(const AppState& st) {
    std::vector<fs::path> out;
    for (const fs::path& base : {st.root, st.user_dir}) {
        std::error_code ec;
        const fs::path dir = base / "config" / "robots";
        if (!fs::is_directory(dir, ec)) continue;
        for (const auto& e : fs::directory_iterator(dir, ec))
            if (e.path().extension() == ".json") out.push_back(e.path());
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end(),
                          [](const fs::path& a, const fs::path& b) {
                              std::error_code ec;
                              return fs::equivalent(a, b, ec);
                          }),
              out.end());
    return out;
}

std::string use_robot_config(AppState& st, const fs::path& path) {
    auto r = core::load_robot_config(path);
    if (!r.ok()) return r.error;
    st.robot = *r.config;
    st.robot_error.clear();
    st.config_path = path;
    st.config_hash = file_sha256(path);
    st.cfg_loaded = false;
    st.mounts.loaded = false;
    make_session(st);
    return {};
}

}  // namespace mmsim::app
