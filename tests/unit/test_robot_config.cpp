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
        CHECK(c.encoder_counts_per_motor_rev == doctest::Approx(28.0));
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
