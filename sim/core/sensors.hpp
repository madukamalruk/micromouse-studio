#pragma once

#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

#include "sim/core/collision.hpp"
#include "sim/core/drive.hpp"
#include "sim/core/maze_geometry.hpp"
#include "sim/core/rng.hpp"
#include "sim/core/robot_config.hpp"

namespace mmsim::core {

inline constexpr std::uint16_t kDistanceNone = 0xFFFF;  // == HAL_DISTANCE_NONE

/// One ray of a sensor beam at the last sample (for the GUI overlay).
struct BeamRay {
    Vec2d from, to;
    double weight = 1.0;
    bool hit = false;
};

/// Sensor origin and centre-ray direction in the world for a robot pose.
void sensor_world_pose(const SensorConfig& s, const Pose& robot, Vec2d& origin, Vec2d& dir);

/// IR inverse model used as "board calibration": ADC -> mm (kDistanceNone if no target).
std::uint16_t ir_adc_to_mm(const SensorConfig& s, double adc);

/// All distance sensors of one robot. Each samples at its own rate on 1 ms tick boundaries,
/// draws its noise from its own seeded stream at sample time (so readings do not depend on
/// how often firmware reads them), and delivers the sample after its latency.
class SensorSuite {
  public:
    SensorSuite() = default;
    SensorSuite(std::vector<SensorConfig> cfg, std::uint64_t world_seed);
    void reset(std::uint64_t world_seed);

    /// Call once per 1 ms tick, after physics. emitter_on has size() entries.
    void update(std::int64_t now_us, const Pose& robot, const CollisionWorld& world,
                const std::vector<bool>& emitter_on);

    std::size_t size() const { return cfg_.size(); }
    const SensorConfig& config(std::size_t ch) const { return cfg_[ch]; }
    std::uint16_t raw(std::size_t ch) const;          // delivered raw reading (0 if none yet)
    std::uint16_t distance_mm(std::size_t ch) const;  // delivered distance
    /// Noise-free centre-ray range at the last sample, metres (infinity if no hit in range).
    double true_distance(std::size_t ch) const;
    const std::vector<BeamRay>& beam(std::size_t ch) const { return ch_[ch].beam; }

  private:
    struct Sample {
        std::int64_t ready_us;
        std::uint16_t raw, mm;
    };
    struct Channel {
        Rng rng;
        std::deque<Sample> pending;
        std::uint16_t raw = 0, mm = kDistanceNone;
        double true_d = std::numeric_limits<double>::infinity();
        std::vector<BeamRay> beam;
        std::int64_t next_sample_us = 0;
    };
    void sample(std::size_t ch, std::int64_t now_us, const Pose& robot, const CollisionWorld& w,
                bool emitter);

    std::vector<SensorConfig> cfg_;
    std::vector<Channel> ch_;
};

/// IMU reading: body-frame rates (deg/s) and accelerations (m/s^2), x fwd, y left, z up.
struct ImuReading {
    double gyro_dps[3] = {0, 0, 0};
    double accel_mps2[3] = {0, 0, 9.80665};
};

/// Gyro + accelerometer with white noise, bias + bias random walk, quantisation and range.
class Imu {
  public:
    Imu() = default;
    Imu(ImuConfig cfg, std::uint64_t world_seed);
    void reset(std::uint64_t world_seed);
    /// One sample (every 1 ms tick): true yaw rate (deg/s) and true body-frame acceleration.
    void update(double dt, double yaw_rate_dps, Vec2d accel_body);
    const ImuReading& reading() const { return r_; }
    double gyro_bias_dps() const { return bias_; }

  private:
    ImuConfig cfg_;
    Rng gyro_rng_[3], accel_rng_[3], walk_rng_;
    double bias_ = 0.0;
    ImuReading r_;
};

}  // namespace mmsim::core
