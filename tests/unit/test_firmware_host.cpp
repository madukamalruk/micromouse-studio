// Firmware DLL host: lockstep timing, SysTick ISR, reset/reload, faults (Slice 3, PRD F2/F3).
// Drives the real test firmwares built from firmware/<name>/main.c + hal_sim.c.
#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

#include "sim/core/maze.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/host/session.hpp"
#include "test_paths.hpp"

using namespace mmsim;
using namespace mmsim::core;
using host::FwState;
using host::Session;

namespace {

std::filesystem::path fw(const char* name) {
    return std::filesystem::path(MMSIM_FIRMWARE_DIR) / (std::string(name) + ".dll");
}

RobotConfig robot() {
    auto r = load_robot_config(test_paths::source_dir() / "config/robots/generic_classic.json");
    REQUIRE(r.ok());
    return *r.config;
}

Maze corridor() {
    Maze m(1, 6);
    m.add_boundary();
    m.set_start({0, 0}, Dir::N);
    m.set_default_goals();
    return m;
}

bool has_log(const Session& s, const std::string& needle) {
    return std::any_of(s.logs().begin(), s.logs().end(), [&](const host::LogLine& l) {
        return l.text.find(needle) != std::string::npos;
    });
}

}  // namespace

TEST_SUITE("host") {
    TEST_CASE("host: missing or non-firmware DLL is rejected with a message") {
        Session s(corridor(), robot());
        CHECK_FALSE(s.load(fw("does_not_exist")));
        CHECK(s.state() == FwState::LoadError);
        CHECK_FALSE(s.error().empty());
    }

    TEST_CASE("host: blink - 20 LED toggles in [0, 10 s) and SysTick runs at 1 kHz") {
        Session s(corridor(), robot());
        REQUIRE_MESSAGE(s.load(fw("test_blink")), s.error());
        CHECK(s.state() == FwState::Ready);
        s.run_ms(10000);
        CHECK(s.state() == FwState::Running);
        CHECK(s.time_us() == 10'000'000);
        const auto& ev = s.led_events();
        const auto in_window = std::count_if(ev.begin(), ev.end(), [](const host::LedEvent& e) {
            return e.led == 0 && e.time_us < 10'000'000;
        });
        CHECK(in_window == 20);
        REQUIRE(ev.size() >= 2);
        CHECK(ev[0].time_us == 0);
        CHECK(ev[1].time_us == 500'000);
        CHECK(has_log(s, "isr ticks=1000 at t=1000 ms"));
    }

    TEST_CASE("host: reset unloads and reloads the DLL - globals start fresh") {
        Session s(corridor(), robot());
        REQUIRE_MESSAGE(s.load(fw("test_globals")), s.error());
        s.run_ms(50);
        CHECK(has_log(s, "boot 1 init 7"));
        s.reset();
        CHECK(s.time_us() == 0);
        CHECK(s.logs().empty());
        s.run_ms(50);
        CHECK(has_log(s, "boot 1 init 7"));
        CHECK_FALSE(has_log(s, "boot 2"));
    }

    TEST_CASE("host: two sessions with the same firmware do not share globals") {
        Session a(corridor(), robot());
        Session b(corridor(), robot());
        REQUIRE(a.load(fw("test_globals")));
        REQUIRE(b.load(fw("test_globals")));
        a.run_ms(5);
        b.run_ms(5);
        CHECK(has_log(a, "boot 1 init 7"));
        CHECK(has_log(b, "boot 1 init 7"));
    }

    TEST_CASE("host: fw_main returning puts the firmware in Halted") {
        Session s(corridor(), robot());
        REQUIRE(s.load(fw("test_faults")));
        s.run_ms(5);
        CHECK(s.state() == FwState::Halted);
        CHECK(has_log(s, "returning from fw_main"));
    }

    TEST_CASE("host: blocking HAL call inside the SysTick ISR is a fault") {
        Session s(corridor(), robot());
        s.set_button(1, true);
        REQUIRE(s.load(fw("test_faults")));
        s.run_ms(5);
        CHECK(s.state() == FwState::Faulted);
        CHECK(s.error().find("ISR") != std::string::npos);
        const auto t = s.time_us();
        s.run_ms(5);  // physics still advances, firmware stays stopped
        CHECK(s.time_us() == t + 5000);
        CHECK(s.state() == FwState::Faulted);
    }

    TEST_CASE("host: open-loop firmware drives into the wall - one crash, encoders and telemetry") {
        Session s(corridor(), robot());
        REQUIRE(s.load(fw("test_open_loop")));
        s.run_ms(4000);
        CHECK(s.state() == FwState::Running);
        REQUIRE(s.world().crashes().size() == 1);
        CHECK(s.world().crashes()[0].penetration <= 0.0005);
        CHECK(s.world().encoder_count(0) > 1000);
        CHECK(s.telemetry_value("enc_left").has_value());
        CHECK(*s.telemetry_value("enc_left") ==
              doctest::Approx(s.world().encoder_count(0)).epsilon(0.05));
        CHECK(has_log(s, "battery 7.40 V"));
    }

    TEST_CASE("host: spin-turn firmware rotates in place with positive gyro") {
        Session s(corridor(), robot());
        REQUIRE(s.load(fw("test_spin_turn")));
        const auto start = s.world().drive().pose;
        s.run_ms(2000);
        const auto p = s.world().drive().pose;
        CHECK(std::hypot(p.x - start.x, p.y - start.y) < 1e-9);
        CHECK(s.world().gyro_z_dps() > 90.0);
        CHECK(s.world().crashes().empty());
        CHECK(has_log(s, "t=1000 ms gyro="));
    }

    TEST_CASE("host: lockstep runs are bit-for-bit deterministic") {
        auto run = [] {
            Session s(corridor(), robot());
            REQUIRE(s.load(fw("test_open_loop")));
            s.run_ms(1500);
            return s.world().drive().pose;
        };
        const auto a = run(), b = run();
        CHECK(a.x == b.x);
        CHECK(a.y == b.y);
        CHECK(a.theta == b.theta);
    }

}  // TEST_SUITE

// Separate suite + separate ctest entry: a hung firmware leaves a spinning detached thread
// behind until the test process exits.
TEST_SUITE("watchdog") {
    TEST_CASE("watchdog: firmware spinning without HAL calls is reported Hung") {
        auto s = std::make_shared<Session>(corridor(), robot());
        s->set_button(0, true);
        REQUIRE(s->load(fw("test_faults")));
        const auto t0 = std::chrono::steady_clock::now();
        const auto r = host::run_guarded(s, 1000, std::chrono::milliseconds(500));
        const auto waited = std::chrono::steady_clock::now() - t0;
        CHECK(r == host::GuardResult::Hung);
        CHECK(waited < std::chrono::seconds(3));
    }

    TEST_CASE("watchdog: healthy firmware completes under the guard") {
        auto s = std::make_shared<Session>(corridor(), robot());
        REQUIRE(s->load(fw("test_blink")));
        CHECK(host::run_guarded(s, 2000, std::chrono::milliseconds(5000)) ==
              host::GuardResult::Completed);
        CHECK(s->time_us() == 2'000'000);
    }

}  // TEST_SUITE
