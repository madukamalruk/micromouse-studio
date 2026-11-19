#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mmsim::core {

/// DC gear-motor (one per wheel). SI units; torques at the motor shaft.
struct MotorConfig {
    double resistance = 10.0;     // ohm
    double kt = 0.00229;          // N*m/A
    double ke = 0.00229;          // V*s/rad
    double gear_ratio = 30.0;     // motor revs per wheel rev
    double rotor_inertia = 1e-8;  // kg*m^2
    double viscous = 1e-8;        // N*m*s/rad
    double coulomb = 2e-5;        // N*m (dry friction, also stiction threshold)
    double efficiency = 0.85;     // gearbox, (0, 1]
    double max_voltage = 6.0;     // V; N20 motors on 2S are capped at 6 V (hardware rule)
};

/// Rectangular body footprint for collisions; offset_x = centre ahead of the wheel axle.
struct BodyConfig {
    double length = 0.095;
    double width = 0.075;
    double offset_x = 0.010;
};

/// Quadrature encoder on the motor shaft. Counts = floor(wheel angle * cpr * G / 2pi).
/// invert_* models the wiring (as on a real board, firmware must know about it).
struct EncoderConfig {
    double counts_per_motor_rev = 28;  // edges per motor revolution (x4 decoding included)
    bool invert_left = false;
    bool invert_right = false;
};

/// Yaw gyro + planar accelerometer, sampled every 1 ms tick.
struct ImuConfig {
    double gyro_noise_dps = 0.0;     // white noise sigma per sample
    double gyro_bias_dps = 0.0;      // initial bias
    double gyro_bias_walk = 0.0;     // bias random walk, dps / sqrt(s)
    double gyro_quant_dps = 0.0;     // LSB size (0 = none)
    double gyro_range_dps = 2000.0;  // saturation
    double accel_noise_mps2 = 0.0;
    double accel_quant_mps2 = 0.0;
    double accel_range_mps2 = 156.9;  // 16 g
};

/// Tyre model (D7). Disabled = rigid no-slip contact (Slice 3 behaviour, exact).
/// Enabled: longitudinal force = stiffness * slip velocity, capped at mu * N per wheel;
/// lateral: body heading leads the velocity vector by slip angle = gradient * a_lat
/// (slip-gradient idea), lateral acceleration capped at mu * g (skid).
struct SlipConfig {
    bool enabled = false;
    double mu = 1.0;                       // tyre/floor friction coefficient
    double long_stiffness = 20.0;          // N per (m/s) of slip velocity, per wheel
    double slip_gradient_deg_per_g = 1.5;  // slip angle per g of lateral acceleration
};

enum class SensorType { IR, ToF };

/// One distance sensor (PRD F8). Pose in the robot frame: x forward, y left, from the axle
/// centre. The beam is `rays` rays over `fov_deg`, weighted edge_weight^((a/half_fov)^2).
///   ToF: reading = nearest hit in the cone, mm.
///   IR:  ADC = ambient + emitter * weighted mean of gain * cos(incidence)^p / (d + d0)^2.
/// Noise (raw units: mm or ADC counts): sigma + rel * reading, then quantised to `quant`.
struct SensorConfig {
    std::string name = "sensor";
    SensorType type = SensorType::IR;
    double x = 0.0, y = 0.0, angle_deg = 0.0;
    double fov_deg = 0.0;
    int rays = 1;
    double edge_weight = 0.135;
    double range_min = 0.0, range_max = 0.25;  // m
    double rate_hz = 1000.0;
    double latency_ms = 0.0;
    double noise_sigma = 0.0;
    double noise_rel = 0.0;
    double quant = 1.0;
    // IR reflectance model
    double ir_gain = 7.2;  // ADC * m^2
    double ir_d0 = 0.01;   // m
    double ir_ambient = 40.0;
    double ir_adc_max = 4095.0;
    double ir_incidence_exp = 1.0;
};

/// Physical robot description loaded from config/robots/*.json (PRD F9).
struct RobotConfig {
    std::string name = "unnamed";
    std::string description;
    double mass = 0.1;            // kg
    double yaw_inertia = 1.4e-4;  // kg*m^2 about the axle centre
    double track_width = 0.072;   // m, wheel contact separation
    double wheel_radius = 0.016;  // m
    double wheel_inertia = 2e-7;  // kg*m^2 per wheel
    BodyConfig body;
    MotorConfig motor;
    double battery_voltage = 7.4;  // V
    EncoderConfig encoder;
    ImuConfig imu;
    SlipConfig slip;
    std::vector<SensorConfig> sensors;
    double collision_restitution = 0.0;  // 0 = stop at the wall, 1 = elastic bounce

    double encoder_counts_per_wheel_rev() const {
        return encoder.counts_per_motor_rev * motor.gear_ratio;
    }
};

/// Built-in 4-sensor layouts. Channel order: 0 = left (side/diagonal), 1 = front-left,
/// 2 = front-right, 3 = right. Names: "front_side", "front_diagonal".
std::optional<std::vector<SensorConfig>> sensor_layout_preset(std::string_view preset,
                                                              SensorType type,
                                                              const BodyConfig& body);
SensorConfig default_sensor(SensorType type);

struct RobotConfigResult {
    std::optional<RobotConfig> config;
    std::string error;  // names the offending key
    bool ok() const { return config.has_value(); }
};

RobotConfigResult parse_robot_config(std::string_view json_text);
RobotConfigResult load_robot_config(const std::filesystem::path& path);
std::string robot_config_to_json(const RobotConfig& c);

}  // namespace mmsim::core
