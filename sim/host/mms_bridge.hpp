#pragma once

#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "sim/core/world.hpp"
#include "sim/host/annotations.hpp"
#include "sim/host/mms_protocol.hpp"

namespace mmsim::host {

enum class MmsWallSource {
    Truth,    // wall queries answered from the maze (mms behaviour)
    Sensors,  // from the simulated distance sensors where the layout can see the wall
};

struct MmsOptions {
    MmsWallSource walls = MmsWallSource::Truth;
    double v_max = 0.6;            // m/s, straight moves
    double accel = 3.0;            // m/s^2
    double w_max = 6.0;            // rad/s, turns
    double alpha = 40.0;           // rad/s^2
    double push_speed = 0.15;      // m/s, driving into the wall on a doomed move
    double front_wall_mm = 120.0;  // sensor mode: front sensors closer than this = wall
    double side_wall_mm = 140.0;   // sensor mode: side/diagonal closer than this = wall
};

/// The mms text API on top of the physics World (Slice 7). Lines from the solver go in with
/// feed_line(); annotation lines (setWall, setColor, setText, clear*) apply at once, even
/// while the mouse moves; every other command is queued and executed in order. Queries answer
/// without simulated time; moves and turns are driven on the real diff-drive physics by a
/// closed-loop controller on the true pose and answer "ack" / "crash" when they finish.
class MmsBridge {
  public:
    MmsBridge(core::World& world, Annotations& ann, MmsOptions opt = {});

    void feed_line(std::string_view line);
    /// Executes queued commands until a motion starts or the queue is empty (no sim time).
    void process();
    /// process() + one physics tick (motion controller or idle) + process() after a motion
    /// finished.
    void tick();
    std::vector<std::string> take_responses();

    bool idle() const { return !motion_ && queue_.empty(); }
    bool moving() const { return motion_.has_value(); }
    /// The mms "Reset" button: wasReset answers true until the solver sends ackReset.
    void press_reset() { was_reset_ = true; }

    SemiPose pose() const { return pose_; }
    int invalid_count() const { return invalid_; }
    int crash_count() const { return crashes_; }
    int moves() const { return moves_; }
    int sensor_fallbacks() const { return fallbacks_; }
    const MmsStats& stats() const { return stats_; }
    const MmsOptions& options() const { return opt_; }
    /// Last commands executed (newest last), for the GUI.
    const std::deque<std::string>& history() const { return history_; }

  private:
    struct Motion {
        enum Kind { Straight, Turn } kind = Straight;
        // Straight: start point, heading, length. Turn: start heading, signed angle.
        double x0 = 0, y0 = 0, th0 = 0, len = 0, ang = 0;
        double s_ref = 0, v_ref = 0;  // profile (m or rad)
        bool profile_done = false;
        int settle_ticks = 0;
        // Doomed straight: after the profile, push into the wall until contact.
        bool doomed = false, pushing = false;
        double push_s = 0;
        std::size_t crashes_at_start = 0;
        SemiPose target;
        bool reply_crash = false;
        bool latch_sides = false;  // sensor mode: sample diagonals on the way in
    };

    void handle_annotation(const std::vector<std::string_view>& tok, std::string_view line);
    void execute(const std::string& line);
    void respond(std::string r) { out_.push_back(std::move(r)); }
    void invalid(std::string_view line);
    bool wall_query(int rel_dir, int half_steps);
    std::optional<bool> sensor_wall(int rel_dir, int half_steps);
    void start_move(int half_steps);
    void start_turn(int eighths);  // + = clockwise (right)
    void control();
    void finish_motion();
    void after_movement();
    void teleport(const SemiPose& p);
    void drive_wheels(double v, double w, double a_ff);
    core::Pose world_pose(const SemiPose& p) const;
    bool at_start(const SemiPose& p) const;

    core::World& world_;
    Annotations& ann_;
    MmsOptions opt_;
    std::deque<std::string> queue_;
    std::vector<std::string> out_;
    std::optional<Motion> motion_;
    SemiPose pose_;
    MmsStats stats_;
    bool was_reset_ = false;
    int invalid_ = 0, crashes_ = 0, moves_ = 0, fallbacks_ = 0;
    std::deque<std::string> history_;
    // Sensor mode: side walls seen on the way into pose_ (cell + heading), -1 = unknown.
    SemiPose latched_at_{-1, -1, -1};
    int latched_left_ = -1, latched_right_ = -1;
    double latch_left_ = 0, latch_right_ = 0;  // min reading during the latch window (mm)
};

}  // namespace mmsim::host
