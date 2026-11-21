// Maze model, parsers and writers (Slice 2, PRD F1).
#include <doctest/doctest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "sim/core/maze.hpp"
#include "sim/core/maze_io.hpp"
#include "test_paths.hpp"

using namespace mmsim::core;
namespace fs = std::filesystem;

namespace {

Maze must_load(const fs::path& p) {
    auto r = load_maze_file(p);
    INFO("loading " << p.string() << ": " << r.error.to_string());
    REQUIRE(r.ok());
    return *r.maze;
}

std::vector<fs::path> txt_files(const fs::path& dir) {
    std::vector<fs::path> out;
    if (!fs::exists(dir)) return out;
    for (const auto& e : fs::directory_iterator(dir))
        if (e.path().extension() == ".txt") out.push_back(e.path());
    std::sort(out.begin(), out.end());
    return out;
}

}  // namespace

TEST_SUITE("maze") {
    TEST_CASE("wall bits match the classic .maz encoding") {
        CHECK(wall_bit(Dir::N) == 0x01);
        CHECK(wall_bit(Dir::E) == 0x02);
        CHECK(wall_bit(Dir::S) == 0x04);
        CHECK(wall_bit(Dir::W) == 0x08);
        CHECK(opposite(Dir::N) == Dir::S);
        CHECK(opposite(Dir::E) == Dir::W);
    }

    TEST_CASE("set_wall keeps shared walls consistent and is bounded") {
        Maze m(3, 2);
        CHECK(m.width() == 3);
        CHECK(m.height() == 2);
        CHECK(m.walls(1, 1) == 0);

        m.set_wall(1, 0, Dir::E, true);  // shared with (2,0) west
        CHECK(m.has_wall(1, 0, Dir::E));
        CHECK(m.has_wall(2, 0, Dir::W));
        m.set_wall(2, 0, Dir::W, false);
        CHECK_FALSE(m.has_wall(1, 0, Dir::E));

        m.set_wall(0, 1, Dir::N, true);  // boundary: no neighbour, must not crash
        CHECK(m.walls(0, 1) == wall_bit(Dir::N));

        m.add_boundary();
        CHECK(m.is_enclosed());
        CHECK(m.walls(0, 0) == (wall_bit(Dir::S) | wall_bit(Dir::W)));
    }

    TEST_CASE("text map: tiny 4x4 hand-checked walls, start, goal") {
        Maze m = must_load(test_paths::source_dir() / "mazes/test/tiny4x4.txt");
        CHECK(m.width() == 4);
        CHECK(m.height() == 4);
        CHECK(m.walls(0, 0) == (wall_bit(Dir::S) | wall_bit(Dir::W)));                     // 12
        CHECK(m.walls(3, 3) == (wall_bit(Dir::N) | wall_bit(Dir::E)));                     // 3
        CHECK(m.walls(1, 3) == (wall_bit(Dir::N) | wall_bit(Dir::E) | wall_bit(Dir::S)));  // 7
        CHECK(m.walls(1, 1) == (wall_bit(Dir::S) | wall_bit(Dir::W)));                     // 12
        CHECK(m.start() == Cell{0, 0});
        CHECK(m.start_heading() == Dir::N);
        REQUIRE(m.goals().size() == 1);
        CHECK(m.goals()[0] == Cell{3, 3});
        CHECK(m.hash() == 0x6f8b2a2096b585b1ULL);  // independent Python cross-check
    }

    TEST_CASE("text map: '+' posts parse identically to 'o' posts") {
        Maze a = must_load(test_paths::source_dir() / "mazes/test/tiny4x4.txt");
        Maze b = must_load(test_paths::source_dir() / "tests/fixtures/maze/tiny4x4_plus_posts.txt");
        CHECK(a == b);
    }

    TEST_CASE("text map: CRLF line endings and trailing blank lines are tolerated") {
        std::string t = "o---o---o\r\n| S     |\r\no---o---o\r\n\r\n\r\n";
        auto r = parse_text_map(t);
        REQUIRE(r.ok());
        CHECK(r.maze->width() == 2);
        CHECK(r.maze->height() == 1);
    }

    TEST_CASE("text map: corridor has one opening so heading points through it") {
        Maze m = must_load(test_paths::source_dir() / "mazes/test/corridor.txt");
        CHECK(m.width() == 1);
        CHECK(m.height() == 6);
        CHECK(m.walls(0, 0) == (wall_bit(Dir::E) | wall_bit(Dir::S) | wall_bit(Dir::W)));
        CHECK(m.walls(0, 5) == (wall_bit(Dir::N) | wall_bit(Dir::E) | wall_bit(Dir::W)));
        CHECK(m.start_heading() == Dir::N);
    }

    TEST_CASE("default goal: centre 2x2 for even sizes when no G marker") {
        std::string t =
            "o---o---o---o---o\n"
            "|               |\n"
            "o   o   o   o   o\n"
            "|               |\n"
            "o   o   o   o   o\n"
            "|               |\n"
            "o   o   o   o   o\n"
            "|               |\n"
            "o---o---o---o---o\n";
        auto r = parse_text_map(t);
        REQUIRE(r.ok());
        auto g = r.maze->goals();
        REQUIRE(g.size() == 4);
        CHECK(r.maze->is_goal({1, 1}));
        CHECK(r.maze->is_goal({1, 2}));
        CHECK(r.maze->is_goal({2, 1}));
        CHECK(r.maze->is_goal({2, 2}));
    }

    TEST_CASE("text map: malformed files are rejected with line numbers") {
        const fs::path fx = test_paths::source_dir() / "tests/fixtures/maze";

        auto even = load_maze_file(fx / "bad_even_rows.txt");
        CHECK_FALSE(even.ok());
        CHECK(even.error.message.find("odd") != std::string::npos);

        auto width = load_maze_file(fx / "bad_width.txt");
        CHECK_FALSE(width.ok());
        CHECK(width.error.message.find("width") != std::string::npos);

        auto open = load_maze_file(fx / "not_enclosed.txt");
        CHECK_FALSE(open.ok());
        CHECK(open.error.message.find("enclosed") != std::string::npos);

        CHECK_FALSE(parse_text_map("").ok());
    }

    TEST_CASE("num format: N E S W columns, mms README example") {
        Maze m = must_load(test_paths::source_dir() / "tests/fixtures/maze/tiny3x2.num");
        CHECK(m.width() == 3);
        CHECK(m.height() == 2);
        CHECK(m.walls(0, 0) == (wall_bit(Dir::E) | wall_bit(Dir::S) | wall_bit(Dir::W)));
        CHECK(m.walls(2, 1) == (wall_bit(Dir::N) | wall_bit(Dir::E) | wall_bit(Dir::W)));
        CHECK(m.walls(1, 1) == (wall_bit(Dir::N) | wall_bit(Dir::E)));
    }

    TEST_CASE("num format: inconsistent shared wall and bad tokens are rejected") {
        const fs::path fx = test_paths::source_dir() / "tests/fixtures/maze";
        auto inc = load_maze_file(fx / "inconsistent.num");
        CHECK_FALSE(inc.ok());
        CHECK(inc.error.message.find("inconsistent") != std::string::npos);

        auto bad = load_maze_file(fx / "bad_line.num");
        CHECK_FALSE(bad.ok());
        CHECK(bad.error.line == 2);
    }

    TEST_CASE("maz binary: column-major bytes, round trip, size checks") {
        Maze m = must_load(test_paths::source_dir() / "mazes/test/tiny4x4.txt");
        std::vector<std::uint8_t> bytes = write_maz(m);
        REQUIRE(bytes.size() == 16);
        CHECK(bytes[0 * 4 + 0] == m.walls(0, 0));  // index = x*H + y
        CHECK(bytes[1 * 4 + 3] == m.walls(1, 3));
        CHECK(bytes[3 * 4 + 3] == m.walls(3, 3));

        auto back = parse_maz(bytes);
        REQUIRE(back.ok());
        CHECK(back.maze->hash() == m.hash());

        std::vector<std::uint8_t> bad(100, 0x0F);  // 100 = 10x10 square but...
        bad[0] = 0x00;                             // ...(0,0) not enclosed on S/W
        CHECK_FALSE(parse_maz(bad).ok());
        CHECK_FALSE(parse_maz(std::vector<std::uint8_t>(99, 0x0F)).ok());  // not square
    }

    TEST_CASE("writers: text and num round-trip") {
        Maze m = must_load(test_paths::source_dir() / "mazes/test/tiny4x4.txt");
        auto t = parse_text_map(write_text_map(m));
        REQUIRE(t.ok());
        CHECK(*t.maze == m);
        auto n = parse_num(write_num(m));
        REQUIRE(n.ok());
        CHECK(n.maze->hash() == m.hash());
    }

    TEST_CASE("UK 1987 final (classic/uk1987f.txt): golden values") {
        const fs::path p = test_paths::mazefiles_dir() / "classic/uk1987f.txt";
        if (!fs::exists(p)) {
            MESSAGE("SKIPPED: run tools/fetch_mazes.ps1 to fetch mazefiles");
            return;
        }
        Maze m = must_load(p);
        CHECK(m.width() == 16);
        CHECK(m.height() == 16);
        CHECK(m.start() == Cell{0, 0});
        CHECK(m.start_heading() == Dir::N);
        CHECK(m.walls(0, 0) == 14);
        // Python: 466 wall bits over all cells = 2*internal + 64 boundary -> 201 internal + 64 =
        // 265
        CHECK(m.wall_count() == 265);
        CHECK(m.goals().size() == 4);
        for (Cell c : {Cell{7, 7}, Cell{7, 8}, Cell{8, 7}, Cell{8, 8}}) CHECK(m.is_goal(c));
        CHECK(m.hash() == 0xbf204a6be93ccd88ULL);  // independent Python cross-check
    }

    TEST_CASE("every classic maze is 16x16 and round-trips through .maz/.num/.txt") {
        auto files = txt_files(test_paths::mazefiles_dir() / "classic");
        if (files.empty()) {
            MESSAGE("SKIPPED: run tools/fetch_mazes.ps1 to fetch mazefiles");
            return;
        }
        CHECK(files.size() == 525);
        for (const auto& f : files) {
            CAPTURE(f.filename().string());
            Maze m = must_load(f);
            CHECK(m.width() == 16);
            CHECK(m.height() == 16);
            CHECK(m.goals().size() >= 1);
            auto maz = parse_maz(write_maz(m));
            REQUIRE(maz.ok());
            CHECK(maz.maze->hash() == m.hash());
            auto num = parse_num(write_num(m));
            REQUIRE(num.ok());
            CHECK(num.maze->hash() == m.hash());
            auto txt = parse_text_map(write_text_map(m));
            REQUIRE(txt.ok());
            CHECK(*txt.maze == m);
        }
    }

    TEST_CASE("halfsize mazes load; the 32x32 ones are 32x32") {
        auto files = txt_files(test_paths::mazefiles_dir() / "halfsize");
        if (files.empty()) {
            MESSAGE("SKIPPED: run tools/fetch_mazes.ps1 to fetch mazefiles");
            return;
        }
        int n32 = 0;
        for (const auto& f : files) {
            CAPTURE(f.filename().string());
            Maze m = must_load(f);
            CHECK(m.width() == m.height());
            if (m.width() == 32) ++n32;
            auto maz = parse_maz(write_maz(m));
            REQUIRE(maz.ok());
            CHECK(maz.maze->hash() == m.hash());
        }
        CHECK(n32 == 33);
        Maze j = must_load(test_paths::mazefiles_dir() / "halfsize/japan2010hef.txt");
        CHECK(j.width() == 32);
        CHECK(j.hash() == 0x9d26cc55d8f1f15aULL);  // independent Python cross-check
    }

    TEST_CASE("training mazes load") {
        auto files = txt_files(test_paths::mazefiles_dir() / "training");
        if (files.empty()) {
            MESSAGE("SKIPPED: run tools/fetch_mazes.ps1 to fetch mazefiles");
            return;
        }
        for (const auto& f : files) {
            CAPTURE(f.filename().string());
            must_load(f);
        }
    }

}  // TEST_SUITE
