#pragma once

#include <cstdint>

namespace mmsim::host {

/// Converts wall-clock frame time into simulated milliseconds for the GUI.
///
/// Real-time modes (1x, 2x, 5x, 10x) keep a fractional accumulator so simulated time tracks
/// `speed * wall time` exactly over any number of frames (no per-frame rounding drift). If the
/// simulation cannot keep up, the per-frame work is capped (no spiral of death), the backlog
/// is dropped and `behind()` reports it. Max mode runs as much as fits in a wall-time budget
/// per frame; the caller measures that (see `max_budget_s()`).
class Pacer {
  public:
    static constexpr double kMax = 0.0;  // speed value meaning "as fast as possible"

    void set_speed(double speed);  // 1, 2, 5, 10, ... or kMax
    double speed() const { return speed_; }
    bool is_max() const { return speed_ <= 0.0; }

    /// Simulated ms to run for a frame that took `wall_dt_s` seconds. Real-time modes only.
    std::int64_t ticks_for_frame(double wall_dt_s);
    /// Clears the accumulator (call on pause/resume so a pause is not "caught up").
    void reset();

    /// True if the last frame had to drop backlog (simulation slower than requested speed).
    bool behind() const { return behind_; }
    /// Largest wall-clock gap one frame may catch up (seconds); longer gaps are dropped.
    static constexpr double kMaxCatchUpS = 0.1;
    /// Wall-time budget per frame in Max mode (seconds).
    static constexpr double max_budget_s() { return 0.012; }

  private:
    double speed_ = 1.0;
    double acc_ms_ = 0.0;
    bool behind_ = false;
};

}  // namespace mmsim::host
