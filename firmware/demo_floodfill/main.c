/*
 * demo_floodfill — classic micromouse demo in pure C11 on mmhal only (Slice 5).
 *
 *   calibrate gyro -> search to the centre (flood fill, mapping walls) -> blink
 *   -> search back to the start -> speed run over the known shortest path -> blink.
 *
 * Motion: 1 kHz control loop (control.c) with trapezoidal profiles, encoder odometry, gyro
 * heading, wall-following steering from the diagonal sensors and front-wall alignment at
 * stops. In-place 90/180 degree turns. Motor safety in motors.c (6 V cap, reversal coast).
 */
#include "control.h"
#include "maze.h"
#include "mmhal/hal.h"
#include "mouse.h"

static void finish(const char* why) {
    control_enable(false);
    hal_log("ERROR %s", why);
    for (;;) hal_delay_ms(1000);
}

void fw_main(void) {
    hal_log("demo_floodfill: %u sensors, battery %.2f V", (unsigned)hal_sensor_count(),
            (double)hal_battery_voltage());
    control_init();
    maze_init();
    mouse_init();
    control_calibrate_gyro(300);
    control_enable(true);

    const uint32_t t0 = hal_millis();
    if (!mouse_search(TARGET_GOAL)) finish("search failed");
    hal_log("goal reached t=%lu ms", (unsigned long)(hal_millis() - t0));
    mouse_blink(8);

    if (!mouse_search(TARGET_START)) finish("return failed");
    mouse_face(NORTH);
    hal_log("start reached t=%lu ms", (unsigned long)(hal_millis() - t0));
    hal_delay_ms(300);

    const uint32_t t_run = hal_millis();
    hal_log("speed run start");
    if (!mouse_speed_run()) finish("speed run failed");
    hal_log("speed run done t=%lu ms time=%lu ms", (unsigned long)(hal_millis() - t0),
            (unsigned long)(hal_millis() - t_run));
    mouse_blink(8);

    control_enable(false);
    for (;;) hal_delay_ms(1000);
}
