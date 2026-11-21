#pragma once
// Competition race timer and the session attempt table (v0.2.0). No GUI dependencies.
//
// The timer is a countdown (3, 5 or 7 minutes, the official run budgets) that counts either real
// time, like a stopwatch beside the course, or simulated time, which follows the pacing speed.
// The attempt table logs one row per run: search time, speed-run time, crashes, best so far.

#include <optional>
#include <string>
#include <vector>

namespace mmsim::app {

enum class TimerBasis { Real, Sim };
enum class TimerState { Idle, Running, Paused, Expired };
enum class TimerCue { None, Tick, TimeUp };

class RaceTimer {
  public:
    static constexpr int kPresetMinutes[3] = {3, 5, 7};

    void set_preset_minutes(int m);  // only while Idle or Expired
    int preset_minutes() const { return minutes_; }
    void set_basis(TimerBasis b);  // only while Idle or Expired
    TimerBasis basis() const { return basis_; }

    void start();  // Idle/Expired: restart from the full time; Paused: resume
    void pause();
    void reset();

    /// Call once per frame. `sim_now_s` is the session's simulated time (negative = no session).
    void update(double dt_real_s, double sim_now_s);

    TimerState state() const { return state_; }
    double duration_s() const { return minutes_ * 60.0; }
    double elapsed_s() const { return elapsed_; }
    double remaining_s() const { return duration_s() > elapsed_ ? duration_s() - elapsed_ : 0.0; }
    /// A countdown tick (each whole second of the last ten) or the final time-up; consumed.
    TimerCue take_cue();

    /// "4:59.3" style text for the big readout.
    static std::string format(double seconds);

  private:
    int minutes_ = 5;
    TimerBasis basis_ = TimerBasis::Real;
    TimerState state_ = TimerState::Idle;
    double elapsed_ = 0.0;
    double last_sim_ = -1.0;
    int last_whole_left_ = 1 << 30;
    TimerCue pending_ = TimerCue::None;
};

struct Attempt {
    int number = 0;
    double search_s = -1.0;     // goal reached; < 0 = never
    double speed_run_s = -1.0;  // speed run start -> done; < 0 = not finished
    int crashes = 0;
    bool finished = false;  // the speed run completed
    double best_so_far_s = -1.0;
    std::string note;  // "", "time up", "restarted"
};

class AttemptTable {
  public:
    /// Adds a row (number and best-so-far are filled in).
    const Attempt& add(Attempt a);
    void clear() { rows_.clear(); }
    const std::vector<Attempt>& rows() const { return rows_; }
    /// Fastest finished speed run, nullopt if none finished.
    std::optional<double> best_speed_run() const;
    int best_number() const;  // attempt number of the best speed run, 0 if none
    std::string to_text() const;

  private:
    std::vector<Attempt> rows_;
};

}  // namespace mmsim::app
