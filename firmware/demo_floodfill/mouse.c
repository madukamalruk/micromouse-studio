#include "mouse.h"

#include "config.h"
#include "control.h"
#include "mmhal/hal.h"
#include "sensors.h"

mouse_state_t g_mouse;

static float absf(float v) {
    return v < 0.0f ? -v : v;
}

/* ---- Primitive moves (blocking; the ISR does the work) --------------------------------- */

static void wait_profile(const profile_t* p) {
    while (!profile_done(p)) hal_wait_tick();
}

static void forward(float mm, float top, float final, float accel) {
    profile_start(&g_fwd, mm, top, final, accel);
    wait_profile(&g_fwd);
}

/* Lets the position loops pull the remaining error to (almost) zero. */
static void settle(unsigned max_ms) {
    for (unsigned i = 0; i < max_ms; ++i) {
        if (absf(control_fwd_error()) < 0.5f && absf(control_rot_error()) < 0.5f &&
            absf(control_speed()) < 5.0f)
            break;
        hal_wait_tick();
    }
}

static void turn(float degrees) {
    control_steering(false);
    profile_start(&g_rot, degrees, TURN_SPEED, 0.0f, TURN_ACCEL);
    wait_profile(&g_rot);
    settle(80);
}

/* With a wall ahead, square up to it and set the distance from the two front sensors. */
static void align_front(void) {
    if (g_sensors.front > FRONT_NEAR_MM) return;
    for (int i = 0; i < 300; ++i) {
        const float dist_err = g_sensors.front - FRONT_ALIGN_MM;            /* + = too far back */
        const float ang_err = g_sensors.front_left - g_sensors.front_right; /* + = nose left */
        if (absf(dist_err) < 0.5f && absf(ang_err) < 0.4f && i > 10) break;
        float v = 6.0f * dist_err, w = -40.0f * ang_err;
        if (v > 100.0f) v = 100.0f;
        if (v < -100.0f) v = -100.0f;
        if (w > 90.0f) w = 90.0f;
        if (w < -90.0f) w = -90.0f;
        profile_set_speed(&g_fwd, v);
        profile_set_speed(&g_rot, w);
        hal_wait_tick();
    }
    profile_set_speed(&g_fwd, 0.0f);
    profile_set_speed(&g_rot, 0.0f);
    for (int i = 0; i < 15; ++i) hal_wait_tick();
    control_clear_errors(); /* the wall is the reference now */
}

static void stop_at_centre(float speed, float accel) {
    forward(HALF_CELL_MM, speed, 0.0f, accel);
    control_steering(false);
    settle(60);
    align_front();
}

/* Turn in place to a new heading (from a stop). */
void mouse_face(dir_t d) {
    const int diff = ((int)d - (int)g_mouse.heading + 4) & 3; /* clockwise quarter turns */
    if (diff == 1) turn(-90.0f);
    if (diff == 2) {
        turn(-90.0f); /* two quarters, so the side walls can be used in between */
        align_front();
        turn(-90.0f);
    }
    if (diff == 3) turn(90.0f);
    g_mouse.heading = d;
}

void mouse_blink(int times) {
    for (int i = 0; i < times; ++i) {
        for (uint8_t l = 0; l < 4; ++l) hal_led_set(l, (i & 1) == 0);
        hal_delay_ms(120);
    }
    for (uint8_t l = 0; l < 4; ++l) hal_led_set(l, false);
}

void mouse_init(void) {
    g_mouse.x = 0;
    g_mouse.y = 0;
    g_mouse.heading = NORTH;
}

/* Called at the boundary of the cell just entered: walls left/front/right of it. */
static void read_walls(void) {
    const uint8_t x = g_mouse.x, y = g_mouse.y;
    const dir_t h = g_mouse.heading;
    maze_set_wall(x, y, h, sensors_wall_front_at_boundary());
    maze_set_wall(x, y, dir_left(h), sensors_wall_left());
    maze_set_wall(x, y, dir_right(h), sensors_wall_right());
    maze_set_wall(x, y, dir_behind(h), false); /* we just came through it */
}

static void enter_next_cell(void) {
    maze_neighbour(g_mouse.x, g_mouse.y, g_mouse.heading, &g_mouse.x, &g_mouse.y);
}

/* ---- Search ------------------------------------------------------------------------------ */

bool mouse_search(target_t target) {
    bool moving = false; /* false: stopped at a cell centre; true: at a cell boundary */
    unsigned cells = 0;
    while (!maze_is_target(g_mouse.x, g_mouse.y, target)) {
        if (maze_flood(target, FLOOD_SEARCH, g_mouse.x, g_mouse.y) == FLOOD_UNREACHABLE) {
            if (moving) stop_at_centre(SEARCH_SPEED, SEARCH_ACCEL);
            hal_log("ERROR target unreachable from (%u,%u)", g_mouse.x, g_mouse.y);
            return false;
        }
        maze_annotate_distances();
        const dir_t d = maze_best_dir(g_mouse.x, g_mouse.y, g_mouse.heading, FLOOD_SEARCH);
        if (!moving) {
            mouse_face(d);
            control_steering(true);
            forward(HALF_CELL_MM, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL);
        } else if (d == g_mouse.heading) {
            forward(CELL_MM, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL);
        } else {
            stop_at_centre(SEARCH_SPEED, SEARCH_ACCEL);
            mouse_face(d);
            control_steering(true);
            forward(HALF_CELL_MM, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCEL);
        }
        moving = true;
        enter_next_cell();
        read_walls();
        ++cells;
        hal_annotate_cell_color(g_mouse.x, g_mouse.y, 'B');
    }
    stop_at_centre(SEARCH_SPEED, SEARCH_ACCEL);
    maze_flood(target, FLOOD_SEARCH, g_mouse.x, g_mouse.y);
    maze_annotate_distances();
    hal_log("search reached (%u,%u) after %u cells", g_mouse.x, g_mouse.y, cells);
    return true;
}

/* ---- Speed run -------------------------------------------------------------------------- */

bool mouse_speed_run(void) {
    if (maze_flood(TARGET_GOAL, FLOOD_KNOWN, g_mouse.x, g_mouse.y) == FLOOD_UNREACHABLE) {
        hal_log("ERROR no known path to the goal");
        return false;
    }
    maze_annotate_distances();
    while (!maze_is_target(g_mouse.x, g_mouse.y, TARGET_GOAL)) {
        const dir_t d = maze_best_dir(g_mouse.x, g_mouse.y, g_mouse.heading, FLOOD_KNOWN);
        if (d == DIR_NONE) {
            hal_log("ERROR speed run lost at (%u,%u)", g_mouse.x, g_mouse.y);
            return false;
        }
        /* Count straight cells in this direction along the path. */
        unsigned n = 0;
        uint8_t x = g_mouse.x, y = g_mouse.y;
        for (;;) {
            if (maze_is_target(x, y, TARGET_GOAL) && n > 0) break;
            if (maze_best_dir(x, y, d, FLOOD_KNOWN) != d) break;
            maze_neighbour(x, y, d, &x, &y);
            hal_annotate_cell_color(x, y, 'G');
            ++n;
        }
        mouse_face(d);
        control_steering(true);
        forward(CELL_MM * (float)n, RUN_SPEED, 0.0f, RUN_ACCEL);
        control_steering(false);
        settle(60);
        align_front();
        g_mouse.x = x;
        g_mouse.y = y;
    }
    return true;
}
