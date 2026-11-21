// Physical maze geometry in metres (shared by renderer now, physics/raycast in Slices 3-4).
#include <doctest/doctest.h>

#include "sim/core/maze.hpp"
#include "sim/core/maze_geometry.hpp"

using namespace mmsim::core;

TEST_SUITE("maze") {
    TEST_CASE("geometry: classic dimensions, posts and wall rectangles") {
        Maze m(2, 1);
        m.add_boundary();
        MazeDims d;  // classic: 180 mm pitch, 12 mm walls
        CHECK(d.cell == doctest::Approx(0.180));
        CHECK(d.wall == doctest::Approx(0.012));

        auto posts = post_rects(m, d);
        CHECK(posts.size() == 3 * 2);
        CHECK(posts[0].x0 == doctest::Approx(-0.006));
        CHECK(posts[0].x1 == doctest::Approx(0.006));

        auto walls = wall_rects(m, d);
        CHECK(static_cast<int>(walls.size()) == m.wall_count());  // 6 boundary segments
        // South wall of cell (0,0): between posts (0,0) and (1,0), centred on y = 0.
        bool found = false;
        for (const auto& w : walls) {
            if (w.cell == Cell{0, 0} && w.dir == Dir::S) {
                found = true;
                CHECK(w.rect.x0 == doctest::Approx(0.006));
                CHECK(w.rect.x1 == doctest::Approx(0.174));
                CHECK(w.rect.y0 == doctest::Approx(-0.006));
                CHECK(w.rect.y1 == doctest::Approx(0.006));
            }
        }
        CHECK(found);

        Rect b = maze_bounds(m, d);
        CHECK(b.x0 == doctest::Approx(-0.006));
        CHECK(b.x1 == doctest::Approx(0.366));
        CHECK(b.y1 == doctest::Approx(0.186));
        CHECK(cell_center({1, 0}, d).x == doctest::Approx(0.270));
        CHECK(cell_center({1, 0}, d).y == doctest::Approx(0.090));
    }

    TEST_CASE("geometry: internal walls are emitted once") {
        Maze m(3, 3);
        m.add_boundary();
        m.set_wall(1, 1, Dir::E, true);
        m.set_wall(1, 1, Dir::N, true);
        CHECK(static_cast<int>(wall_rects(m, {}).size()) == m.wall_count());
        CHECK(m.wall_count() == 12 + 2);
    }

}  // TEST_SUITE
