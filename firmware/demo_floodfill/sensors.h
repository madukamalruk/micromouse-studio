#ifndef DEMO_SENSORS_H
#define DEMO_SENSORS_H

#include <stdbool.h>

/* Distance sensors, sampled once per tick in the ISR. Missing readings are reported as a
 * large distance (SENSOR_FAR_MM) so comparisons stay simple. */
#define SENSOR_FAR_MM 999.0f

typedef struct {
    volatile float left, right;             /* diagonals, mm */
    volatile float front_left, front_right; /* mm */
    volatile float front;                   /* mean of the two front sensors */
} sensor_view_t;

extern sensor_view_t g_sensors;

void sensors_update(void);          /* ISR */
float sensors_steering_error(void); /* mm; + = robot too far right of centre -> steer left */
bool sensors_wall_left(void);
bool sensors_wall_right(void);
bool sensors_wall_front_at_boundary(void);
bool sensors_front_near(void);

#endif
