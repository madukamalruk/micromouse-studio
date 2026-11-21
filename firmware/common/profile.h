#ifndef FWC_PROFILE_H
#define FWC_PROFILE_H

#include <stdbool.h>

/* Trapezoidal motion profile (forward in mm, rotation in degrees), stepped in the 1 ms ISR.
 * After it finishes it keeps moving at final_speed until the next profile starts, so
 * consecutive moves chain without stopping. */
typedef enum { PROFILE_IDLE, PROFILE_RUNNING, PROFILE_DONE } profile_state_t;

typedef struct {
    volatile profile_state_t state;
    volatile float position; /* progress along this move (always >= 0) */
    volatile float speed;    /* signed output speed */
    volatile float accel;    /* signed output acceleration of the last step */
    float distance, top_speed, final_speed, rate, sign;
} profile_t;

void profile_reset(profile_t* p);
void profile_start(profile_t* p, float distance, float top_speed, float final_speed, float accel);
void profile_set_speed(profile_t* p, float speed); /* direct speed command (IDLE state) */
void profile_update(profile_t* p, float dt);
bool profile_done(const profile_t* p);

#endif
