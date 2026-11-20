#pragma once

#include <cstdint>
#include <vector>

#include "sim/core/collision.hpp"
#include "sim/core/drive.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/maze_geometry.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/core/sensors.hpp"

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

    /// seed drives every noise source; same seed + same inputs = identical run.
    World(Maze maze, RobotConfig cfg, MazeDims dims = {}, std::uint64_t seed = 1);
    std::uint64_t seed() const { return seed_; }

    const Maze& maze() const { return maze_; }
    const MazeDims& dims() const { return dims_; }
    const RobotConfig& config() const { return cfg_; }
    const DriveModel& model() const { return model_; }
    const CollisionWorld& collision() const { return collide_; }
    const DriveState& drive() const { return s_; }
    DriveState& drive_mut() { return s_; }

    /// duty in [-1, 1] of battery voltage; the result is clamped to motor.max_voltage.
    void set_motor_duty(int side, double duty);
    double motor_voltage(int side) const { return volts_[side & 1]; }
    /// Armature current (A): (V - Ke * G * wheel_speed) / R; + = driving forward.
    double motor_current(int side) const { return amps_[side & 1]; }
    /// Pack terminal voltage (V); constant battery_voltage unless battery.model.
    double battery_voltage() const { return v_bat_; }
    /// Current drawn from the pack (A): electronics + sum(duty * I_motor), clamped >= 0.
    double battery_current() const { return i_bat_; }
    double state_of_charge() const { return soc_; }
    double charge_used_mah() const;
    /// Stall-current helper (also used by tests): armature current at voltage v.
    static double armature_current(const RobotConfig& c, double v, double wheel_speed);
    int overvoltage_events() const { return model_.overvoltage_events(); }

    void step_tick();
    void reset();
    std::int64_t time_us() const { return time_us_; }

    const std::vector<CrashEvent>& crashes() const { return crashes_; }
    bool in_contact() const { return in_contact_; }

    /// Encoder counts (quantised wheel angle, wiring inversion applied; no other noise).
    std::int32_t encoder_count(int side) const;
    /// TRUE yaw rate in deg/s (ground truth; firmware sees imu()).
    double gyro_z_dps() const;
    /// TRUE body-frame acceleration averaged over the last tick (m/s^2).
    Vec2d true_accel_body() const { return accel_body_; }
    /// Measured IMU sample of the last tick (noise, bias, quantisation).
    const ImuReading& imu() const { return imu_.reading(); }
    const Imu& imu_model() const { return imu_; }

    /// Distance sensors (delivered readings, see SensorSuite).
    const SensorSuite& sensors() const { return sensors_; }
    std::size_t sensor_count() const { return sensors_.size(); }
    std::uint16_t sensor_raw(int ch) const;
    std::uint16_t sensor_distance_mm(int ch) const;
    void set_emitter(int ch, bool on);
    bool emitter(int ch) const;

    Pose start_pose() const;

  private:
    void substep(double dt);
    void accumulate_accel(const DriveState& prev);

    Maze maze_;
    RobotConfig cfg_;
    MazeDims dims_;
    DriveModel model_;
    CollisionWorld collide_;
    DriveState s_;
    double volts_[2] = {0.0, 0.0};
    double duty_[2] = {0.0, 0.0};
    double amps_[2] = {0.0, 0.0};
    double v_bat_ = 0.0;
    double i_bat_ = 0.0;
    double soc_ = 1.0;
    void update_battery(double dt);
    std::int64_t time_us_ = 0;
    std::vector<CrashEvent> crashes_;
    bool in_contact_ = false;
    double body_radius_ = 0.0;  // farthest body corner from the axle centre
    std::uint64_t seed_ = 1;
    SensorSuite sensors_;
    std::vector<bool> emitters_;
    Imu imu_;
    Vec2d accel_body_;
    Vec2d accel_sum_;  // world-frame velocity change accumulated over the tick
};

}  // namespace mmsim::core
