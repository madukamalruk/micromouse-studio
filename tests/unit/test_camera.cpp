// 2-D camera: world metres (y up) <-> screen pixels (y down). Slice 2, PRD F3.
#include <doctest/doctest.h>

#include "sim/app/camera.hpp"

using mmsim::app::Camera;
using mmsim::app::Vec2;

TEST_SUITE("app") {

TEST_CASE("camera: fit centres the bounds and preserves aspect") {
    Camera c;
    c.set_viewport({100, 50}, {400, 200});  // origin (100,50), size 400x200 px
    c.fit({0.0, 0.0}, {2.0, 1.0}, 0.0);      // 2 m x 1 m world box, no margin
    CHECK(c.pixels_per_metre() == doctest::Approx(200.0));
    Vec2 sw = c.world_to_screen({0.0, 0.0});
    Vec2 ne = c.world_to_screen({2.0, 1.0});
    CHECK(sw.x == doctest::Approx(100.0));
    CHECK(sw.y == doctest::Approx(250.0));  // y flipped: world bottom = screen bottom
    CHECK(ne.x == doctest::Approx(500.0));
    CHECK(ne.y == doctest::Approx(50.0));
}

TEST_CASE("camera: margin shrinks scale; tall box limited by height") {
    Camera c;
    c.set_viewport({0, 0}, {400, 400});
    c.fit({0.0, 0.0}, {1.0, 2.0}, 0.1);  // 10 % margin each side
    CHECK(c.pixels_per_metre() == doctest::Approx(400.0 * 0.8 / 2.0));
    Vec2 mid = c.world_to_screen({0.5, 1.0});
    CHECK(mid.x == doctest::Approx(200.0));
    CHECK(mid.y == doctest::Approx(200.0));
}

TEST_CASE("camera: screen_to_world inverts world_to_screen") {
    Camera c;
    c.set_viewport({10, 20}, {640, 480});
    c.fit({-0.1, -0.1}, {2.9, 2.9}, 0.05);
    Vec2 w{1.234, 0.567};
    Vec2 back = c.screen_to_world(c.world_to_screen(w));
    CHECK(back.x == doctest::Approx(w.x));
    CHECK(back.y == doctest::Approx(w.y));
}

TEST_CASE("camera: zoom_at keeps the point under the cursor fixed") {
    Camera c;
    c.set_viewport({0, 0}, {800, 600});
    c.fit({0.0, 0.0}, {3.0, 3.0}, 0.0);
    Vec2 cursor{250.0, 333.0};
    Vec2 before = c.screen_to_world(cursor);
    c.zoom_at(cursor, 2.5);
    Vec2 after = c.screen_to_world(cursor);
    CHECK(after.x == doctest::Approx(before.x));
    CHECK(after.y == doctest::Approx(before.y));
    c.zoom_at(cursor, 1e9);  // clamped, never infinite
    CHECK(c.pixels_per_metre() <= Camera::kMaxPixelsPerMetre);
}

TEST_CASE("camera: pan moves the world with the mouse") {
    Camera c;
    c.set_viewport({0, 0}, {800, 600});
    c.fit({0.0, 0.0}, {3.0, 3.0}, 0.0);
    Vec2 p = c.world_to_screen({1.0, 1.0});
    c.pan_pixels({30.0, -12.0});
    Vec2 q = c.world_to_screen({1.0, 1.0});
    CHECK(q.x == doctest::Approx(p.x + 30.0));
    CHECK(q.y == doctest::Approx(p.y - 12.0));
}

}  // TEST_SUITE
