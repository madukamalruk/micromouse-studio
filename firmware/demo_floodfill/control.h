#ifndef DEMO_CONTROL_H
#define DEMO_CONTROL_H

#include <stdbool.h>

#include "profile.h"

/* 1 kHz control loop (SysTick ISR): odometry from encoders + gyro, forward and rotation
 * profiles, PD position loops with motor-model feed-forward, wall-following steering. */
extern profile_t g_fwd; /* mm */
extern profile_t g_rot; /* degrees, + = CCW (left) */

void control_init(void);        /* installs the ISR */
void control_enable(bool on);   /* off = 0 V, errors cleared */
void control_steering(bool on); /* wall following on straights */
void control_clear_errors(void);
float control_fwd_error(void);            /* mm */
float control_rot_error(void);            /* degrees */
float control_speed(void);                /* measured forward speed, mm/s */
float control_distance(void);             /* odometer, mm */
void control_calibrate_gyro(unsigned ms); /* robot must be still; blocking */

#endif
