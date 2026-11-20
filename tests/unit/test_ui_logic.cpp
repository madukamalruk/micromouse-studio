// Slice 9.10: pure UI logic (workspaces, mission status, action dispatcher).
#include <doctest/doctest.h>

#include "sim/app/actions.hpp"
#include "sim/app/ui_logic.hpp"

using namespace mmsim::app;
using namespace mmsim::app::ui;

TEST_SUITE("ui") {
    TEST_CASE("ui: workspaces parse by name, alias and number key") {
        CHECK(parse_workspace("cockpit") == Workspace::Cockpit);
        CHECK(parse_workspace("Studio") == Workspace::Studio);
        CHECK(parse_workspace("3") == Workspace::Telemetry);
        CHECK(parse_workspace("reports") == Workspace::Tuning);
        CHECK_FALSE(parse_workspace("kitchen").has_value());
        for (int i = 0; i < kWorkspaceCount; ++i) {
            const auto w = static_cast<Workspace>(i);
            CHECK(parse_workspace(workspace_id(w)) == w);
        }
        CHECK(std::string(workspace_title(Workspace::Telemetry)) == "Telemetry & Compare");
        CHECK(parse_mode("BEGINNER") == Mode::Beginner);
        CHECK(parse_mode(mode_id(Mode::Engineer)) == Mode::Engineer);
        CHECK_FALSE(parse_mode("wizard").has_value());
    }

    TEST_CASE("ui: mission status speaks plain English through the demo phases") {
        MissionInput in;
        CHECK(mission_status(in).text == "Pick a maze");
        in.have_maze = true;
        CHECK(mission_status(in).text == "Ready");
        CHECK(mission_status(in).detail.find("No firmware") != std::string::npos);
        in.have_source = true;
        CHECK(mission_status(in).text == "Ready");
        in.running = true;
        in.time_us = 1000;
        in.logs = {"demo_floodfill: 16x16 maze"};
        CHECK(mission_status(in).text == "Exploring maze");
        CHECK(mission_status(in).tone == Tone::Active);
        in.logs.push_back("goal reached t=12.3");
        CHECK(mission_status(in).text == "Goal reached!");
        CHECK(mission_status(in).tone == Tone::Good);
        in.logs.push_back("start reached t=25.0");
        CHECK(mission_status(in).text == "Returning");
        in.logs.push_back("speed run start");
        CHECK(mission_status(in).text == "Speed run");
        in.logs.push_back("speed run done t=31.2");
        CHECK(mission_status(in).text == "Run complete");
        in.running = false;
        in.logs = {"demo_floodfill", "ERROR target unreachable"};
        CHECK(mission_status(in).text == "Stuck");
        CHECK(mission_status(in).tone == Tone::Bad);
    }

    TEST_CASE("ui: crashes, faults and pause") {
        MissionInput in;
        in.have_maze = in.have_source = true;
        in.time_us = 2'000'000;
        CHECK(mission_status(in).text == "Paused");
        in.running = true;
        in.last_crash_us = 1'500'000;
        CHECK(mission_status(in).text == "Crashed");
        in.time_us = 3'000'000;  // crash was 1.5 s ago: back to the phase
        CHECK(mission_status(in).text == "Exploring maze");
        in.faulted = true;
        CHECK(mission_status(in).text == "Crashed");
        in.load_error = true;
        CHECK(mission_status(in).text == "Firmware error");
    }

    TEST_CASE("ui: log classification for the event list") {
        CHECK(classify_log("goal reached t=1") == EventKind::Milestone);
        CHECK(classify_log("speed run start") == EventKind::Milestone);
        CHECK(classify_log("ERROR target unreachable") == EventKind::Error);
        CHECK(classify_log("cell 3,4") == EventKind::Info);
    }

    TEST_CASE("ui: top bar fitting, time format, speed presets, sensor bars") {
        CHECK(fit_count({100, 100, 100}, 320, 8) == 3);
        CHECK(fit_count({100, 100, 100}, 300, 8) == 2);
        CHECK(fit_count({100}, 50, 8) == 0);
        CHECK(fit_count({}, 50, 8) == 0);
        CHECK(format_sim_time(0) == "00:00.000");
        CHECK(format_sim_time(83'456'000) == "01:23.456");
        CHECK(format_sim_time(-5) == "00:00.000");
        CHECK(speed_presets().size() == 4);
        CHECK(speed_preset_index(5.0) == 2);
        CHECK(speed_preset_index(0.0) == 3);
        CHECK(speed_preset_index(64.0) == -1);
        CHECK(sensor_bar(true, 2047.5, 4095, 0, 0) == doctest::Approx(0.5));
        CHECK(sensor_bar(false, 0, 0, 50, 200) == doctest::Approx(0.75));
        CHECK(sensor_bar(false, 0, 0, 500, 200) == doctest::Approx(0.0));
        CHECK(sensor_bar(true, 9000, 4095, 0, 0) == doctest::Approx(1.0));
    }

    TEST_CASE("ui: one dispatcher for shortcuts and console") {
        ActionRegistry r;
        int go = 0;
        std::vector<std::string> got;
        r.add({"go", "Space", "Run", "Start or stop", "", [&](const auto&) {
                   ++go;
                   return std::string("running");
               }});
        r.add({"speed", "+/-", "Run", "Set speed", "<1|2|5|fast>", [&](const auto& a) {
                   got = a;
                   return a.empty() ? std::string("usage: speed <x>") : "speed " + a[0];
               }});
        r.add({"screenshot", "F12", "View", "Save PNG", "", nullptr});
        CHECK(r.trigger("go") == "running");
        CHECK(r.dispatch("  GO  ") == "running");
        CHECK(go == 2);
        CHECK(r.dispatch("speed 5") == "speed 5");
        CHECK(r.dispatch("speed \"two words\" x") == "speed two words");
        CHECK(got.size() == 2);
        CHECK(r.dispatch("speed") == "usage: speed <x>");
        CHECK(r.dispatch("fly").find("Unknown command 'fly'") == 0);
        CHECK(r.dispatch("   ").empty());
        CHECK(r.dispatch("screenshot").empty());  // no handler: no crash

        CHECK(r.complete("s") == std::vector<std::string>{"screenshot", "speed"});
        CHECK(r.complete_line("s") == "s");
        CHECK(r.complete_line("sp") == "speed ");
        CHECK(r.complete_line("g") == "go ");
        CHECK(r.complete_line("speed 5") == "speed 5");
        CHECK(r.complete_line("zz") == "zz");
    }

    TEST_CASE("ui: console history walks back and forward") {
        ActionRegistry r;
        CHECK(r.history_step(-1).empty());
        r.remember("go");
        r.remember("speed 5");
        r.remember("speed 5");  // duplicate of the last entry
        r.remember("");
        CHECK(r.history().size() == 2);
        CHECK(r.history_step(-1) == "speed 5");
        CHECK(r.history_step(-1) == "go");
        CHECK(r.history_step(-1) == "go");  // clamps at the oldest
        CHECK(r.history_step(+1) == "speed 5");
        CHECK(r.history_step(+1).empty());  // past the newest: empty line
        CHECK(r.history_step(+1).empty());
    }
}
