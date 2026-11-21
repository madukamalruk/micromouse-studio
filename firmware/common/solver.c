#include "solver.h"

#include "control.h"
#include "mmhal/hal.h"
#include "mouse.h"

static void finish(const char* why) {
    control_enable(false);
    hal_log("ERROR %s", why);
    for (;;) hal_delay_ms(1000);
}

void solver_main(const solver_t* solver) {
    hal_log("%s: %u sensors, battery %.2f V", solver->name, (unsigned)hal_sensor_count(),
            (double)hal_battery_voltage());
    hal_log("algorithm: %s", solver->description);
    control_init();
    maze_init();
    mouse_init();
    if (solver->init) solver->init();
    control_calibrate_gyro(300);
    control_enable(true);

    const uint32_t t0 = hal_millis();
    hal_telemetry("phase", 0.0f);
    if (!mouse_search(solver, TARGET_GOAL)) finish("search failed");
    hal_log("goal reached t=%lu ms", (unsigned long)(hal_millis() - t0));
    mouse_blink(8);

    hal_telemetry("phase", 1.0f);
    if (!mouse_search(solver, TARGET_START)) finish("return failed");
    mouse_face(NORTH);
    hal_log("start reached t=%lu ms", (unsigned long)(hal_millis() - t0));
    hal_delay_ms(300);

    const uint32_t t_run = hal_millis();
    hal_telemetry("phase", 2.0f);
    hal_log("speed run start");
    if (!mouse_speed_run(solver)) finish("speed run failed");
    hal_log("speed run done t=%lu ms time=%lu ms", (unsigned long)(hal_millis() - t0),
            (unsigned long)(hal_millis() - t_run));
    mouse_blink(8);

    control_enable(false);
    for (;;) hal_delay_ms(1000);
}
