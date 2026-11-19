#pragma once

#include <cstdint>
#include <vector>

#include "sim/core/collision.hpp"
#include "sim/core/drive.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/maze_geometry.hpp"
#include "sim/core/robot_config.hpp"

namespace mmsim::core {

/// Raised once when the body starts touching a wall/post (not on every pinned step).
struct CrashEvent {
    std::int64_t time_us = 0;
    Pose pose;
    Vec2d normal;             // direction the obstacle pushes the robot
    double impact_speed = 0;  // m/s, |forward speed| just before the response
    double penetration = 0;   // m, depth at the first overlapping sample (TOI bracket)
};

/// Physical world: maze + one robot. Pure, deterministic, no threads, no OS calls.
/// One tick = 1 ms = 4 substeps of 250 us (PRD physics table).
class World {
  public:
    static constexpr std::int64_t kTickUs = 1000;
    static constexpr int kSubsteps = 4;

    World(Maze maze, RobotConfig cfg, MazeDims dims = {});

    const Maze& maze() const { return maze_; }
    const MazeDims& dims() const { return dims_; }
    const RobotConfig& config() const { return cfg_; }
    const DriveModel& model() const { return model_; }
    const DriveState& drive() const { return s_; }
    DriveState& drive_mut() { return s_; }

    /// duty in [-1, 1] of battery voltage; the result is clamped to motor.max_voltage.
    void set_motor_duty(int side, double duty);
    double motor_voltage(int side) const { return volts_[side & 1]; }
    int overvoltage_events() const { return model_.overvoltage_events(); }

    void step_tick();
    void reset();
    std::int64_t time_us() const { return time_us_; }

    const std::vector<CrashEvent>& crashes() const { return crashes_; }
    bool in_contact() const { return in_contact_; }

    /// Ideal sensors (noise models arrive in Slice 4).
    std::int32_t encoder_count(int side) const;
    double gyro_z_dps() const;

    Pose start_pose() const;

  private:
    void substep(double dt);

    Maze maze_;
    RobotConfig cfg_;
    MazeDims dims_;
    DriveModel model_;
    CollisionWorld collide_;
    DriveState s_;
    double volts_[2] = {0.0, 0.0};
    std::int64_t time_us_ = 0;
    std::vector<CrashEvent> crashes_;
    bool in_contact_ = false;
    double body_radius_ = 0.0;  // farthest body corner from the axle centre
};

}  // namespace mmsim::core
