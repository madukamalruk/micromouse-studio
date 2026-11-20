#pragma once

#include <limits>
#include <string>
#include <string_view>

#include "sim/core/maze.hpp"

namespace mmsim::host {

/// mms "semi" position: cell (x, y) has its centre at (2x+1, 2y+1); even coordinates are cell
/// edges. dir 0..7 = N, NE, E, SE, S, SW, W, NW (clockwise, 45 degrees each).
struct SemiPose {
    int x = 1, y = 1, dir = 0;
    friend bool operator==(const SemiPose&, const SemiPose&) = default;
};

/// Unit step of one half-cell in semi coordinates for dir 0..7.
void mms_step(int dir, int& dx, int& dy);

/// mms Maze::isWall(position, direction, halfStepsAhead): checks the start, then each of the
/// next halfStepsAhead half-steps. From a cell centre an ordinal direction aims at a post
/// (true); from a cell edge the diagonal looks at the wall beside it (see mms Model.cpp).
bool mms_is_wall(const core::Maze& m, SemiPose p, int half_steps_ahead);

/// QString::toInt(&ok) for base 10: optional sign, digits only; 0 and ok = false otherwise.
int mms_to_int(std::string_view s, bool& ok);

/// mms run statistics and score (mms Stats.cpp). Distances are in half-steps.
class MmsStats {
  public:
    static constexpr float kResetPenalty = 15.0f;

    void reset_all();
    void start_run();
    void finish_run();
    void end_unfinished_run();
    void penalize_for_reset();
    void add_distance(int half_steps);
    void add_turn();

    bool run_started() const { return started_; }
    bool has(std::string_view name) const;
    /// Value as mms prints it; "-1" while a field is empty (e.g. best run before a finish).
    std::string get(std::string_view name) const;
    float score() const;

  private:
    static float effective(int half_steps) {
        return half_steps > 2 ? static_cast<float>(half_steps) / 2.0f + 1.0f
                              : static_cast<float>(half_steps);
    }
    int total_dist_ = 0, total_turns_ = 0;
    float total_eff_ = 0;
    int cur_dist_ = 0, cur_turns_ = 0;
    float cur_eff_ = 0;
    bool cur_valid_ = false;  // a run has been started at least once
    int best_dist_ = 0, best_turns_ = 0;
    float best_eff_ = 0;
    bool solved_ = false;
    bool started_ = false;
    float penalty_ = 0;
};

}  // namespace mmsim::host
