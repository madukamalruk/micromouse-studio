#include "sensors.h"

#include <stdint.h>

#include "config.h"
#include "mmhal/hal.h"

sensor_view_t g_sensors;

static float read_mm(uint8_t ch) {
    const uint16_t d = hal_get_distance(ch);
    return d == HAL_DISTANCE_NONE ? SENSOR_FAR_MM : (float)d;
}

void sensors_update(void) {
    g_sensors.left = read_mm(SENSOR_LEFT);
    g_sensors.right = read_mm(SENSOR_RIGHT);
    g_sensors.front_left = read_mm(SENSOR_FRONT_LEFT);
    g_sensors.front_right = read_mm(SENSOR_FRONT_RIGHT);
    g_sensors.front = 0.5f * (g_sensors.front_left + g_sensors.front_right);
}

bool sensors_wall_left(void) {
    return g_sensors.left < SIDE_WALL_MM;
}
bool sensors_wall_right(void) {
    return g_sensors.right < SIDE_WALL_MM;
}
bool sensors_wall_front_at_boundary(void) {
    return g_sensors.front < FRONT_WALL_BOUNDARY_MM;
}
bool sensors_front_near(void) {
    return g_sensors.front < FRONT_NEAR_MM;
}

/* Diagonal error: both walls -> left - right; one wall -> twice its offset from nominal.
 * Closer to the left wall gives a shorter left reading -> negative -> steer right. */
float sensors_steering_error(void) {
    const bool l = sensors_wall_left(), r = sensors_wall_right();
    if (l && r) return g_sensors.left - g_sensors.right;
    if (l) return 2.0f * (g_sensors.left - DIAG_NOMINAL_MM);
    if (r) return 2.0f * (DIAG_NOMINAL_MM - g_sensors.right);
    return 0.0f;
}
