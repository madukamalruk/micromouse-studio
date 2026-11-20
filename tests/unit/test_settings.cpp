// Slice 8: GUI session persistence (mmsim.json).
#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>

#include "sim/app/settings.hpp"

namespace fs = std::filesystem;
using mmsim::app::AppSettings;

namespace {
fs::path fresh_dir(const char* tag) {
    const fs::path d = fs::temp_directory_path() / tag;
    fs::remove_all(d);
    fs::create_directories(d);
    return d;
}
}  // namespace

TEST_SUITE("app") {
    TEST_CASE("settings: save/load round trip, atomic write") {
        const fs::path dir = fresh_dir("mmsim_settings_rt");
        AppSettings s;
        s.maze = "D:/mazes/uk1987f.txt";
        s.config = "config/robots/generic_classic.json";
        s.firmware = "build/dev/firmware_bin/demo_floodfill.dll";
        s.mms_solver = "";
        s.mms_sensor_walls = true;
        s.seed = 42;
        s.speed = 0.0;  // max
        s.boot_buttons[1] = true;
        s.workspace = "studio";
        s.mode = "beginner";
        s.muted = true;
        std::string err;
        REQUIRE(mmsim::app::save_settings(dir / "mmsim.json", s, err));
        CHECK_FALSE(fs::exists(dir / "mmsim.json.tmp"));
        AppSettings r;
        REQUIRE(mmsim::app::load_settings(dir / "mmsim.json", r, err));
        CHECK(r.maze == s.maze);
        CHECK(r.config == s.config);
        CHECK(r.firmware == s.firmware);
        CHECK(r.mms_solver.empty());
        CHECK(r.mms_sensor_walls);
        CHECK(r.seed == 42);
        CHECK(r.speed == 0.0);
        CHECK_FALSE(r.boot_buttons[0]);
        CHECK(r.boot_buttons[1]);
        CHECK(r.workspace == "studio");
        CHECK(r.mode == "beginner");
        CHECK(r.muted);
    }

    TEST_CASE("settings: missing file = defaults; corrupt file = defaults + error") {
        const fs::path dir = fresh_dir("mmsim_settings_bad");
        AppSettings r;
        std::string err;
        CHECK(mmsim::app::load_settings(dir / "none.json", r, err));
        CHECK(err.empty());
        CHECK(r.seed == 1);
        CHECK(r.speed == 1.0);

        std::ofstream(dir / "bad.json") << "{ not json";
        r.seed = 99;
        CHECK_FALSE(mmsim::app::load_settings(dir / "bad.json", r, err));
        CHECK_FALSE(err.empty());
        CHECK(r.seed == 1);  // reset to defaults

        std::ofstream(dir / "types.json") << R"({"seed": "x", "speed": -3, "maze": 5,
            "firmware": "fw.dll", "future_key": [1, 2]})";
        CHECK(mmsim::app::load_settings(dir / "types.json", r, err));
        CHECK(r.seed == 1);     // wrong type -> default
        CHECK(r.speed == 1.0);  // negative -> default
        CHECK(r.maze.empty());  // wrong type -> default
        CHECK(r.firmware == "fw.dll");
    }
}
