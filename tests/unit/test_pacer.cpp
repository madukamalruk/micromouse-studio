// GUI real-time pacing (Slice 5.3): simulated time tracks speed x wall time exactly.
#include <doctest/doctest.h>

#include <cstdint>

#include "sim/core/rng.hpp"
#include "sim/host/pacer.hpp"

using mmsim::host::Pacer;

TEST_SUITE("pacing") {
    TEST_CASE("pacer: 60 s at 1x of 60 Hz frames gives 60 000 ms (+-1 %)") {
        Pacer p;
        p.set_speed(1.0);
        std::int64_t sim = 0;
        for (int i = 0; i < 3600; ++i) sim += p.ticks_for_frame(1.0 / 60.0);
        CHECK(sim >= 59400);
        CHECK(sim <= 60600);
        CHECK(sim >= 59999);  // in fact exact up to the fractional remainder
        CHECK_FALSE(p.behind());
    }

    TEST_CASE("pacer: 2x / 5x / 10x with jittery frame times") {
        for (double speed : {2.0, 5.0, 10.0}) {
            CAPTURE(speed);
            Pacer p;
            p.set_speed(speed);
            mmsim::core::Rng rng(42);
            double wall = 0.0;
            std::int64_t sim = 0;
            while (wall < 30.0) {
                const double dt = 0.004 + 0.03 * rng.uniform();  // 4..34 ms frames
                wall += dt;
                sim += p.ticks_for_frame(dt);
            }
            const double expect = wall * 1000.0 * speed;
            CHECK(static_cast<double>(sim) == doctest::Approx(expect).epsilon(0.001));
        }
    }

    TEST_CASE("pacer: a long stall is not caught up and is reported as behind") {
        Pacer p;
        p.set_speed(1.0);
        CHECK(p.ticks_for_frame(2.0) == 100);  // capped at kMaxCatchUpS
        CHECK(p.behind());
        CHECK(p.ticks_for_frame(0.01) == 10);
        CHECK_FALSE(p.behind());
    }

    TEST_CASE("pacer: max mode is driven by the caller's time budget") {
        Pacer p;
        p.set_speed(Pacer::kMax);
        CHECK(p.is_max());
        CHECK(p.ticks_for_frame(0.016) == 0);
        CHECK(Pacer::max_budget_s() > 0.0);
    }
}
