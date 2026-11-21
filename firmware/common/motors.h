#ifndef FWC_MOTORS_H
#define FWC_MOTORS_H

/* Sets motor terminal voltages (+ = forward). Enforces the 6 V cap (duty = V / V_battery)
 * and >= 20 ms of coasting before any change of direction. Call once per 1 ms tick. */
void motors_set_volts(float left, float right);
void motors_stop(void);
void motors_reset(void);

#endif
