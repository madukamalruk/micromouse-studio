// Maze validation report + flood distances (Slice 2, PRD F2).
#include <doctest/doctest.h>

#include <filesystem>

#include "sim/core/maze.hpp"
#include "sim/core/maze_io.hpp"
#include "sim/core/maze_validate.hpp"
#include "test_paths.hpp"

using namespace mmsim::core;

namespace {

// 4x4 open box with only the boundary: classic start (0,0), centre goal 2x2.
Maze open_box() {
    Maze m(4, 4);
    m.add_boundary();
    m.set_default_goals();
    return m;
}

}  // namespace

TEST_SUITE("maze") {

TEST_CASE("flood distances: open box Manhattan distance to nearest goal") {
    Maze m = open_box();
    auto d = flood_distances(m);
    CHECK(d.at(1, 1) == 0);
    CHECK(d.at(2, 2) == 0);
    CHECK(d.at(0, 0) == 2);
    CHECK(d.at(3, 0) == 2);
    CHECK(d.at(0, 3) == 2);
}

TEST_CASE("flood distances: unreachable cells are marked") {
    Maze m = open_box();
    // wall off cell (3,3) completely
    m.set_wall(3, 3, Dir::S, true);
    m.set_wall(3, 3, Dir::W, true);
    auto d = flood_distances(m);
    CHECK(d.at(3, 3) == FloodMap::kUnreachable);
    auto r = validate(m);
    REQUIRE(r.unreachable.size() == 1);
    CHECK(r.unreachable[0] == Cell{3, 3});
}

TEST_CASE("validate: open box facts") {
    Maze m = open_box();
    auto r = validate(m);
    CHECK(r.enclosed);
    CHECK(r.unreachable.empty());
    CHECK(r.start_wall_count == 2);           // only S and W boundary walls
    CHECK(r.goal_entrances == 8);             // 2x2 centre open on all sides
    CHECK(r.hollow_centre_post);              // no walls inside the 2x2 goal
    CHECK_FALSE(r.all_posts_have_walls);      // inner posts are bare
    CHECK(r.solvable);
    CHECK(r.left_wall_follower_solves == false);  // hugs the boundary, never enters centre
    CHECK_FALSE(r.is_official());
}

TEST_CASE("validate: start-wall count and single goal entrance") {
    Maze m = open_box();
    m.set_wall(0, 0, Dir::E, true);  // classic 3-wall start
    // close the 2x2 goal except one entrance on the west of (1,1)
    for (int x : {1, 2}) {
        m.set_wall(x, 1, Dir::S, true);
        m.set_wall(x, 2, Dir::N, true);
    }
    m.set_wall(2, 1, Dir::E, true);
    m.set_wall(2, 2, Dir::E, true);
    m.set_wall(1, 2, Dir::W, true);
    auto r = validate(m);
    CHECK(r.start_wall_count == 3);
    CHECK(r.goal_entrances == 1);
    CHECK(r.hollow_centre_post);
    CHECK(r.solvable);
}

TEST_CASE("validate: UK 1987 final is solvable with a 3-wall start") {
    auto p = test_paths::mazefiles_dir() / "classic/uk1987f.txt";
    if (!std::filesystem::exists(p)) {
        MESSAGE("SKIPPED: run tools/fetch_mazes.ps1 to fetch mazefiles");
        return;
    }
    auto lr = load_maze_file(p);
    REQUIRE(lr.ok());
    auto r = validate(*lr.maze);
    CHECK(r.enclosed);
    CHECK(r.solvable);
    CHECK(r.start_wall_count == 3);
    CHECK(r.goal_entrances == 1);
    CHECK(r.hollow_centre_post);
    CHECK(r.unreachable.empty());
    CHECK(flood_distances(*lr.maze).at(0, 0) > 0);
}

}  // TEST_SUITE
