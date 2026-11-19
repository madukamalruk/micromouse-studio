#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

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
    double encoder_counts_per_motor_rev = 28;
    double collision_restitution = 0.0;  // 0 = stop at the wall, 1 = elastic bounce

    double encoder_counts_per_wheel_rev() const {
        return encoder_counts_per_motor_rev * motor.gear_ratio;
    }
};

struct RobotConfigResult {
    std::optional<RobotConfig> config;
    std::string error;  // names the offending key
    bool ok() const { return config.has_value(); }
};

RobotConfigResult parse_robot_config(std::string_view json_text);
RobotConfigResult load_robot_config(const std::filesystem::path& path);
std::string robot_config_to_json(const RobotConfig& c);

}  // namespace mmsim::core
