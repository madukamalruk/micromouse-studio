// v0.2.0 pure-logic modules: user data paths, move tracker + details log, race timer + attempt
// table, robot config editor, help catalog, compass maths. All GUI-free.
#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include "sim/app/attitude.hpp"
#include "sim/app/config_edit.hpp"
#include "sim/app/details_log.hpp"
#include "sim/app/help_text.hpp"
#include "sim/app/race_timer.hpp"
#include "sim/app/user_paths.hpp"
#include "sim/core/robot_config.hpp"

using namespace mmsim::app;
namespace fs = std::filesystem;
namespace core = mmsim::core;

namespace {

fs::path scratch(const char* name) {
    const fs::path p = fs::temp_directory_path() / "mmsim_v02_tests" / name;
    std::error_code ec;
    fs::remove_all(p, ec);
    fs::create_directories(p);
    return p;
}

void touch(const fs::path& p) {
    fs::create_directories(p.parent_path());
    std::ofstream(p) << "x";
}

// A fake program folder: <dir>/mazes exists so find_data_root accepts it.
fs::path fake_data_dir(const char* name) {
    const fs::path d = scratch(name);
    fs::create_directories(d / "mazes");
    return d;
}

bool always(const fs::path&) {
    return true;
}
bool never(const fs::path&) {
    return false;
}

}  // namespace

TEST_SUITE("ui") {
    // --- user paths ------------------------------------------------------------------------
    TEST_CASE("paths: explicit --root keeps data and state together") {
        PathInputs in;
        in.root_arg = fs::path("X:/somewhere");
        in.local_appdata = "C:/Users/u/AppData/Local";
        const auto d = resolve_data_paths(in, never);
        CHECK(d.kind == InstallKind::Explicit);
        CHECK(d.root == d.user_dir);
    }

    TEST_CASE("paths: portable flag and marker write next to the data") {
        const fs::path root = fake_data_dir("portable");
        PathInputs in;
        in.exe_dir = root;
        in.cwd = root;
        in.local_appdata = "C:/Users/u/AppData/Local";
        in.portable_flag = true;
        auto d = resolve_data_paths(in, never);
        CHECK(d.kind == InstallKind::Portable);
        CHECK(d.user_dir == d.root);
        in.portable_flag = false;
        touch(root / kPortableMarker);
        d = resolve_data_paths(in, never);
        CHECK(d.kind == InstallKind::Portable);
    }

    TEST_CASE("paths: a source checkout stays in the repo") {
        const fs::path root = fake_data_dir("devrepo");
        touch(root / "CMakeLists.txt");
        fs::create_directories(root / "firmware");
        PathInputs in;
        in.exe_dir = root / "build" / "dev";
        in.cwd = root;
        in.local_appdata = "C:/Users/u/AppData/Local";
        const auto d = resolve_data_paths(in, never);
        CHECK(d.kind == InstallKind::DevRepo);
        CHECK(d.user_dir == d.root);
    }

    TEST_CASE("paths: the installer marker sends user data to LOCALAPPDATA") {
        const fs::path root = fake_data_dir("installed");
        touch(root / kInstalledMarker);
        PathInputs in;
        in.exe_dir = root;
        in.cwd = root;
        in.local_appdata = scratch("appdata");
        const auto d = resolve_data_paths(in, always);  // even when the folder is writable
        CHECK(d.kind == InstallKind::Installed);
        CHECK(d.user_dir == in.local_appdata / "MMSim");
        CHECK(d.root == find_data_root({root}));
    }

    TEST_CASE("paths: loose writable folder stays plain, read-only falls back to LOCALAPPDATA") {
        const fs::path root = fake_data_dir("loose");
        PathInputs in;
        in.exe_dir = root;
        in.cwd = root;
        in.local_appdata = scratch("appdata2");
        auto d = resolve_data_paths(in, always);
        CHECK(d.kind == InstallKind::Plain);
        CHECK(d.user_dir == d.root);
        d = resolve_data_paths(in, never);
        CHECK(d.kind == InstallKind::ReadOnly);
        CHECK(d.user_dir == in.local_appdata / "MMSim");
        // No LOCALAPPDATA at all: never invent a path, keep state with the data.
        in.local_appdata.clear();
        d = resolve_data_paths(in, never);
        CHECK(d.user_dir == d.root);
    }

    TEST_CASE("paths: dir_writable probes and cleans up") {
        const fs::path d = scratch("probe");
        CHECK(dir_writable(d));
        CHECK(fs::is_empty(d));
        CHECK_FALSE(dir_writable(d / "does" / "not" / "exist"));
    }

    // --- compass maths ---------------------------------------------------------------------
    TEST_CASE("attitude: bearing is clockwise from north") {
        CHECK(bearing_deg(kPiD / 2.0) == doctest::Approx(0.0));     // +y = north
        CHECK(bearing_deg(0.0) == doctest::Approx(90.0));           // +x = east
        CHECK(bearing_deg(-kPiD / 2.0) == doctest::Approx(180.0));  // -y = south
        CHECK(bearing_deg(kPiD) == doctest::Approx(270.0));         // -x = west
        CHECK(theta_of_bearing(bearing_deg(0.3)) == doctest::Approx(0.3));
        CHECK(wrap_deg360(-90.0) == doctest::Approx(270.0));
        CHECK(wrap_deg180(270.0) == doctest::Approx(-90.0));
        CHECK(quarter_of_bearing(89.0) == 1);
        CHECK(quarter_of_bearing(359.0) == 0);
        CHECK(std::string(rose8(44.0)) == "NE");
        CHECK(std::string(rose8(359.0)) == "N");
    }

    // --- move tracker ----------------------------------------------------------------------
    namespace {
    struct Sim1k {
        MoveTracker tr;
        std::vector<DetailEntry> out;
        double t = 0.0, x = 0.09, y = 0.09, th = kPiD / 2.0;  // centre of cell (0,0) facing north
        void feed() {
            tr.feed({t, x, y, th}, out);
            t += 0.001;
        }
        void drive(double dist, double v = 0.5) {  // along the current heading
            const int n = static_cast<int>(dist / v / 0.001);
            for (int i = 0; i < n; ++i) {
                x += std::cos(th) * v * 0.001;
                y += std::sin(th) * v * 0.001;
                feed();
            }
        }
        void turn(double rad) {  // CCW positive, 90 deg in ~0.3 s
            const int n = 300;
            for (int i = 0; i < n; ++i) {
                th += rad / n;
                feed();
            }
        }
        void rest(double s) {
            for (int i = 0; i < static_cast<int>(s / 0.001); ++i) feed();
        }
        bool has(const std::string& needle) const {
            for (const auto& e : out)
                if (e.text.find(needle) != std::string::npos) return true;
            return false;
        }
    };
    }  // namespace

    TEST_CASE("moves: cells, a straight run and turns are reported") {
        Sim1k s;
        s.rest(0.05);
        s.drive(0.54);  // three cells north
        s.rest(0.3);
        s.turn(-kPiD / 2.0);  // clockwise = right turn
        s.rest(0.05);
        s.turn(kPiD / 2.0);  // back to north = left turn
        s.rest(0.05);
        CHECK(s.has("Start (0,0) facing N"));
        CHECK(s.has("Cell (0,1) N"));
        CHECK(s.has("Cell (0,3) N"));
        CHECK(s.has("Straight 3 cells to (0,3)"));
        CHECK(s.has("Turn Right 90\xC2\xB0 at (0,3)"));
        CHECK(s.has("Turn Left 90\xC2\xB0 at (0,3)"));
        CHECK(s.tr.cell_at(0.0) == std::pair<int, int>{0, 0});
        CHECK(s.tr.cell_at(1e9) == std::pair<int, int>{0, 3});
    }

    TEST_CASE("moves: a half turn is one U-turn and wobble is not a turn") {
        Sim1k s;
        s.rest(0.05);
        s.turn(kPiD);
        s.rest(0.05);
        CHECK(s.has("U-turn 180\xC2\xB0"));
        Sim1k w;
        w.rest(0.05);
        for (int i = 0; i < 400; ++i) {  // +-3 degrees of steering wobble
            w.th = kPiD / 2.0 + 0.05 * std::sin(i * 0.05);
            w.feed();
        }
        w.rest(0.05);
        CHECK_FALSE(w.has("Turn"));
    }

    TEST_CASE("moves: reset starts the diary again") {
        Sim1k s;
        s.rest(0.01);
        CHECK(s.tr.started());
        s.tr.reset();
        CHECK_FALSE(s.tr.started());
        s.out.clear();
        s.feed();
        CHECK(s.has("Start (0,0)"));
    }

    // --- details log -----------------------------------------------------------------------
    TEST_CASE("details: raw lines, milestones, crashes and the filter pills") {
        DetailsLog log;
        std::vector<DetailEntry> batch;
        log.ingest_log(1.0, "[fw] picking route", batch);
        log.ingest_log(2.0, "GOAL REACHED in 12.4 s", batch);
        log.ingest_log(3.0, "ERROR: sensor timeout", batch);
        log.ingest_log(4.0, "Speed run start", batch);
        batch.push_back(DetailsLog::crash_entry(5.0, 0.62, 3, 4));
        for (auto& e : batch) log.add(e);

        auto count = [&](DetailFilter f, const char* q = "") {
            int n = 0;
            for (const auto& e : log.entries())
                if (detail_matches(e, f, q)) ++n;
            return n;
        };
        CHECK(count(DetailFilter::All) == static_cast<int>(batch.size()));
        CHECK(count(DetailFilter::Firmware) == 4);     // every raw line, none of the derived ones
        CHECK(count(DetailFilter::Errors) == 2);       // the ERROR line and the crash
        CHECK(count(DetailFilter::Moves) == 3);        // goal, speed run, crash
        CHECK(count(DetailFilter::All, "GOAL") == 2);  // case-insensitive, raw + milestone
        CHECK(count(DetailFilter::All, "m/s") == 1);
        CHECK(count(DetailFilter::All, "nothing like this") == 0);

        const std::string text = log.to_text(DetailFilter::Errors, "");
        CHECK(text.find("sensor timeout") != std::string::npos);
        CHECK(text.find("Wall crash at 0.62 m/s near (3,4)") != std::string::npos);
        CHECK(text.find("picking route") == std::string::npos);
        CHECK(std::string(detail_tag({0, kDetError, ""})) == "ERR ");

        log.clear();
        CHECK(log.entries().empty());
    }

    TEST_CASE("details: the log is bounded") {
        DetailsLog log;
        for (std::size_t i = 0; i < DetailsLog::kMax + 100; ++i)
            log.add({static_cast<double>(i), kDetFirmware, "x"});
        CHECK(log.entries().size() == DetailsLog::kMax);
        CHECK(log.entries().front().t_s == doctest::Approx(100.0));
    }

    // --- race timer + attempts -------------------------------------------------------------
    TEST_CASE("timer: counts down in real time and signals ticks and time-up") {
        RaceTimer t;
        t.set_preset_minutes(3);
        CHECK(t.remaining_s() == doctest::Approx(180.0));
        CHECK(t.state() == TimerState::Idle);
        t.update(5.0, -1.0);  // idle: nothing moves
        CHECK(t.remaining_s() == doctest::Approx(180.0));
        t.start();
        t.update(60.0, -1.0);
        CHECK(t.remaining_s() == doctest::Approx(120.0));
        t.pause();
        t.update(30.0, -1.0);
        CHECK(t.remaining_s() == doctest::Approx(120.0));
        t.start();
        CHECK(t.state() == TimerState::Running);
        t.update(111.5, -1.0);
        CHECK(t.take_cue() == TimerCue::Tick);  // inside the last ten seconds
        t.update(9.0, -1.0);
        CHECK(t.state() == TimerState::Expired);
        CHECK(t.remaining_s() == doctest::Approx(0.0));
        CHECK(t.take_cue() == TimerCue::TimeUp);
        CHECK(t.take_cue() == TimerCue::None);  // consumed
        t.reset();
        CHECK(t.state() == TimerState::Idle);
        CHECK(t.remaining_s() == doctest::Approx(180.0));
    }

    TEST_CASE("timer: sim basis follows simulated time, not wall time") {
        RaceTimer t;
        t.set_basis(TimerBasis::Sim);
        t.start();
        t.update(0.016, 10.0);
        t.update(0.016, 40.0);  // 30 s of sim in one frame (fast-forward)
        CHECK(t.elapsed_s() == doctest::Approx(30.0).epsilon(0.05));
        t.set_preset_minutes(7);  // locked while running
        CHECK(t.preset_minutes() == 5);
        CHECK(RaceTimer::format(272.34) == "4:32.3");
        CHECK(RaceTimer::format(0.0) == "0:00.0");
    }

    TEST_CASE("attempts: numbering, best-so-far and the copy text") {
        AttemptTable a;
        CHECK_FALSE(a.best_speed_run().has_value());
        Attempt r1;
        r1.search_s = 40.0;
        r1.speed_run_s = 12.5;
        r1.finished = true;
        Attempt r2;
        r2.search_s = 35.0;
        r2.crashes = 2;  // never finished its speed run
        Attempt r3;
        r3.search_s = 30.0;
        r3.speed_run_s = 9.75;
        r3.finished = true;
        a.add(r1);
        a.add(r2);
        a.add(r3);
        CHECK(a.rows().size() == 3);
        CHECK(a.rows()[0].number == 1);
        CHECK(a.rows()[2].number == 3);
        CHECK(a.rows()[0].best_so_far_s == doctest::Approx(12.5));
        CHECK(a.rows()[1].best_so_far_s == doctest::Approx(12.5));
        CHECK(a.rows()[2].best_so_far_s == doctest::Approx(9.75));
        CHECK(*a.best_speed_run() == doctest::Approx(9.75));
        CHECK(a.best_number() == 3);
        CHECK_FALSE(a.to_text().empty());
        a.clear();
        CHECK(a.rows().empty());
        CHECK(a.best_number() == 0);
    }

    // --- robot config editor ---------------------------------------------------------------
    namespace {
    core::RobotConfig generic() {
        auto r = core::load_robot_config(fs::path(MMSIM_SOURCE_DIR) / "config" / "robots" /
                                         "generic_classic.json");
        REQUIRE_MESSAGE(r.ok(), r.error);
        return *r.config;
    }
    }  // namespace

    TEST_CASE("config edit: round trip keeps every untouched value exactly") {
        const core::RobotConfig c0 = generic();
        core::RobotConfig c = c0;
        const ConfigEdit e = edit_from_config(c0);
        CHECK(e.mass_g == doctest::Approx(c0.mass * 1000.0));
        CHECK(e.wheel_radius_mm == doctest::Approx(c0.wheel_radius * 1000.0));
        apply_edit(e, c);
        CHECK(core::robot_config_to_json(c) == core::robot_config_to_json(c0));
    }

    TEST_CASE("config edit: only the changed fields change") {
        const core::RobotConfig c0 = generic();
        core::RobotConfig c = c0;
        ConfigEdit e = edit_from_config(c0);
        e.mass_g = 120.0;
        e.tyre_mu = 0.75;
        e.slip_enabled = true;
        apply_edit(e, c);
        CHECK(c.mass == doctest::Approx(0.12));
        CHECK(c.slip.mu == doctest::Approx(0.75));
        CHECK(c.slip.enabled);
        CHECK(c.track_width == doctest::Approx(c0.track_width));
        CHECK(c.motor.max_voltage == doctest::Approx(c0.motor.max_voltage));
    }

    TEST_CASE("config edit: the 6 V motor rule warns, nonsense is an error") {
        ConfigEdit e = edit_from_config(generic());
        CHECK(validate_edit(e).empty());
        e.max_voltage = 7.4;
        auto issues = validate_edit(e);
        REQUIRE(issues.size() == 1);
        CHECK(issues[0].level == IssueLevel::Warn);
        CHECK(issues[0].text.find("6.0") != std::string::npos);
        CHECK_FALSE(has_errors(issues));
        e.mass_g = -5.0;
        e.wheel_radius_mm = 0.0;
        issues = validate_edit(e);
        CHECK(has_errors(issues));
    }

    TEST_CASE("config edit: save copy never overwrites") {
        const fs::path dir = scratch("cfgcopy");
        core::RobotConfig c = generic();
        const std::string stem = unique_copy_stem(dir, "generic_classic");
        CHECK(stem == "generic_classic_edited");
        fs::path out;
        std::string err;
        REQUIRE(save_config_copy(c, dir, stem, out, err));
        CHECK(fs::exists(out));
        CHECK(unique_copy_stem(dir, "generic_classic") == "generic_classic_edited2");
        CHECK_FALSE(save_config_copy(c, dir, stem, out, err));  // same name again
        CHECK_FALSE(err.empty());
        CHECK_FALSE(save_config_copy(c, dir, "../evil", out, err));
        CHECK_FALSE(save_config_copy(c, dir, "bad:name", out, err));
        // The copy loads back as the same robot.
        auto back = core::load_robot_config(out);
        REQUIRE(back.ok());
        CHECK(back.config->mass == doctest::Approx(c.mass));
        // The shipped file is untouched by all of this.
        CHECK(core::load_robot_config(fs::path(MMSIM_SOURCE_DIR) / "config" / "robots" /
                                      "generic_classic.json")
                  .ok());
    }

    // --- help catalog ----------------------------------------------------------------------
    TEST_CASE("help: every id has a short plain-English text") {
        CHECK(kHelpCount > 60);
        for (std::size_t i = 0; i < kHelpCount; ++i) {
            const auto h = static_cast<Help>(i);
            const std::string t = help_text(h);
            INFO(help_name(h));
            CHECK(t.size() >= 30);
            CHECK(t.size() <= 330);  // 1-2 sentences
            CHECK(t.back() == '.');
            for (unsigned char ch : t) CHECK(ch < 0x80);  // ASCII only: renders in every font
        }
        CHECK(std::string(help_name(Help::Attitude)) == "Attitude");
    }
}
