#include "sim/app/race_timer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace mmsim::app {

void RaceTimer::set_preset_minutes(int m) {
    if (state_ == TimerState::Running || state_ == TimerState::Paused) return;
    minutes_ = std::clamp(m, 1, 30);
    reset();
}

void RaceTimer::set_basis(TimerBasis b) {
    if (state_ == TimerState::Running || state_ == TimerState::Paused) return;
    basis_ = b;
}

void RaceTimer::start() {
    if (state_ == TimerState::Running) return;
    if (state_ == TimerState::Idle || state_ == TimerState::Expired) {
        elapsed_ = 0.0;
        last_whole_left_ = 1 << 30;
        pending_ = TimerCue::None;
    }
    last_sim_ = -1.0;
    state_ = TimerState::Running;
}

void RaceTimer::pause() {
    if (state_ == TimerState::Running) state_ = TimerState::Paused;
}

void RaceTimer::reset() {
    state_ = TimerState::Idle;
    elapsed_ = 0.0;
    last_sim_ = -1.0;
    last_whole_left_ = 1 << 30;
    pending_ = TimerCue::None;
}

void RaceTimer::update(double dt_real_s, double sim_now_s) {
    if (state_ != TimerState::Running) {
        last_sim_ = sim_now_s;
        return;
    }
    if (basis_ == TimerBasis::Real) {
        elapsed_ += std::max(0.0, dt_real_s);
    } else {
        // Only simulated time that actually passed counts (a reset or rewind goes backwards).
        if (last_sim_ >= 0.0 && sim_now_s > last_sim_) elapsed_ += sim_now_s - last_sim_;
    }
    last_sim_ = sim_now_s;
    const double left = remaining_s();
    if (left <= 0.0) {
        elapsed_ = duration_s();
        state_ = TimerState::Expired;
        pending_ = TimerCue::TimeUp;
        return;
    }
    const int whole = static_cast<int>(std::ceil(left));
    if (whole < last_whole_left_) {
        if (whole <= 10 && last_whole_left_ != (1 << 30)) pending_ = TimerCue::Tick;
        last_whole_left_ = whole;
    }
}

TimerCue RaceTimer::take_cue() {
    const TimerCue c = pending_;
    pending_ = TimerCue::None;
    return c;
}

std::string RaceTimer::format(double seconds) {
    if (seconds < 0.0) seconds = 0.0;
    const int tenths = static_cast<int>(std::floor(seconds * 10.0 + 1e-9));
    char b[32];
    std::snprintf(b, sizeof b, "%d:%02d.%d", tenths / 600, (tenths / 10) % 60, tenths % 10);
    return b;
}

const Attempt& AttemptTable::add(Attempt a) {
    a.number = static_cast<int>(rows_.size()) + 1;
    double best = -1.0;
    for (const auto& r : rows_)
        if (r.finished && r.speed_run_s >= 0.0 && (best < 0.0 || r.speed_run_s < best))
            best = r.speed_run_s;
    if (a.finished && a.speed_run_s >= 0.0 && (best < 0.0 || a.speed_run_s < best))
        best = a.speed_run_s;
    a.best_so_far_s = best;
    rows_.push_back(std::move(a));
    return rows_.back();
}

std::optional<double> AttemptTable::best_speed_run() const {
    std::optional<double> best;
    for (const auto& r : rows_)
        if (r.finished && r.speed_run_s >= 0.0 && (!best || r.speed_run_s < *best))
            best = r.speed_run_s;
    return best;
}

int AttemptTable::best_number() const {
    const auto b = best_speed_run();
    if (!b) return 0;
    for (const auto& r : rows_)
        if (r.finished && r.speed_run_s == *b) return r.number;
    return 0;
}

std::string AttemptTable::to_text() const {
    auto t = [](double s) {
        char b[24];
        if (s < 0.0)
            std::snprintf(b, sizeof b, "-");
        else
            std::snprintf(b, sizeof b, "%.3f", s);
        return std::string(b);
    };
    std::string out = "attempt\tsearch_s\tspeed_run_s\tcrashes\tbest_s\tnote\n";
    for (const auto& r : rows_) {
        out += std::to_string(r.number) + "\t" + t(r.search_s) + "\t" + t(r.speed_run_s) + "\t" +
               std::to_string(r.crashes) + "\t" + t(r.best_so_far_s) + "\t" + r.note + "\n";
    }
    return out;
}

}  // namespace mmsim::app
