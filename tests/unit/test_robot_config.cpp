// Robot config JSON loader (Slice 3, PRD F9).
#include <doctest/doctest.h>

#include <string>

#include "sim/core/robot_config.hpp"
#include "test_paths.hpp"

using namespace mmsim::core;

TEST_SUITE("physics") {
    TEST_CASE("config: generic_classic.json loads with expected values") {
        auto r = load_robot_config(test_paths::source_dir() / "config/robots/generic_classic.json");
        INFO(r.error);
        REQUIRE(r.ok());
        const RobotConfig& c = *r.config;
        CHECK(c.name == "generic_classic");
        CHECK(c.mass == doctest::Approx(0.100));
        CHECK(c.track_width == doctest::Approx(0.072));
        CHECK(c.wheel_radius == doctest::Approx(0.016));
        CHECK(c.motor.gear_ratio == doctest::Approx(30.0));
        CHECK(c.motor.max_voltage == doctest::Approx(6.0));
        CHECK(c.battery_voltage == doctest::Approx(7.4));
        CHECK(c.body.length == doctest::Approx(0.095));
        CHECK(c.encoder.counts_per_motor_rev == doctest::Approx(28.0));
        CHECK(c.slip.enabled);
        REQUIRE(c.sensors.size() == 4);
        CHECK(c.sensors[0].name == "diag_left");
        CHECK(c.sensors[0].angle_deg == doctest::Approx(45.0));
        CHECK(c.sensors[1].type == SensorType::IR);
        CHECK(c.imu.gyro_quant_dps == doctest::Approx(0.061));
    }

    TEST_CASE("config: errors name the offending key") {
        auto missing = parse_robot_config(R"({"name":"x","mass_kg":0.1})");
        CHECK_FALSE(missing.ok());
        CHECK(missing.error.find("yaw_inertia_kg_m2") != std::string::npos);

        auto r = load_robot_config(test_paths::source_dir() / "config/robots/generic_classic.json");
        REQUIRE(r.ok());
        std::string text = R"({"name":"x","mass_kg":"heavy"})";
        auto wrong_type = parse_robot_config(text);
        CHECK_FALSE(wrong_type.ok());
        CHECK(wrong_type.error.find("mass_kg") != std::string::npos);

        auto bad_json = parse_robot_config("{ not json");
        CHECK_FALSE(bad_json.ok());
    }

    TEST_CASE("config: sensor layout presets, explicit lists and keyed sensor errors") {
        const std::string base =
            R"("name":"x","mass_kg":0.1,"yaw_inertia_kg_m2":1e-4,"track_width_m":0.07,
        "wheel":{"radius_m":0.016,"inertia_kg_m2":0},"body":{"length_m":0.09,"width_m":0.07,"offset_x_m":0},
        "motor":{"resistance_ohm":10,"kt_nm_per_a":0.002,"ke_v_s_per_rad":0.002,"gear_ratio":30,
        "rotor_inertia_kg_m2":0,"viscous_nm_s_per_rad":0,"coulomb_nm":0,"efficiency":1,"max_voltage_v":6},
        "battery_voltage_v":7.4,"encoder":{"counts_per_motor_rev":12},"collision_restitution":0)";
        auto side = parse_robot_config("{" + base +
                                       R"(,"sensor_layout":{"preset":"front_side","type":"tof"}})");
        INFO(side.error);
        REQUIRE(side.ok());
        REQUIRE(side.config->sensors.size() == 4);
        CHECK(side.config->sensors[0].angle_deg == doctest::Approx(90.0));
        CHECK(side.config->sensors[3].angle_deg == doctest::Approx(-90.0));
        CHECK(side.config->sensors[1].type == SensorType::ToF);
        CHECK_FALSE(side.config->slip.enabled);  // optional sections default off

        auto bad_preset =
            parse_robot_config("{" + base + R"(,"sensor_layout":{"preset":"octopus"}})");
        CHECK(bad_preset.error.find("sensor_layout.preset") != std::string::npos);

        auto list = parse_robot_config(
            "{" + base +
            R"(,"sensors":[{"type":"ir","angle_deg":10},{"type":"tof","fov_deg":-1}]})");
        CHECK_FALSE(list.ok());
        CHECK(list.error.find("sensors[1].fov_deg") != std::string::npos);

        auto bad_flag = parse_robot_config("{" + base + R"(,"slip":{"enabled":"yes"}})");
        CHECK(bad_flag.error.find("slip.enabled") != std::string::npos);
    }

    TEST_CASE("config: physically invalid values are rejected") {
        auto r = load_robot_config(test_paths::source_dir() / "config/robots/generic_classic.json");
        REQUIRE(r.ok());
        std::string text = robot_config_to_json(*r.config);
        auto again = parse_robot_config(text);
        REQUIRE(again.ok());  // round trip

        auto pos = text.find("\"mass_kg\"");
        REQUIRE(pos != std::string::npos);
        RobotConfig neg = *r.config;
        neg.mass = -1.0;
        auto bad = parse_robot_config(robot_config_to_json(neg));
        CHECK_FALSE(bad.ok());
        CHECK(bad.error.find("mass_kg") != std::string::npos);
    }

}  // TEST_SUITE
