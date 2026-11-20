#include "profile.h"

#define MIN_CREEP 0.04f /* fraction of top speed kept until a stopping move reaches its end */

static float absf(float v) {
    return v < 0.0f ? -v : v;
}

void profile_reset(profile_t* p) {
    p->state = PROFILE_IDLE;
    p->position = 0.0f;
    p->speed = 0.0f;
    p->accel = 0.0f;
    p->distance = p->top_speed = p->final_speed = p->rate = 0.0f;
    p->sign = 1.0f;
}

void profile_start(profile_t* p, float distance, float top_speed, float final_speed, float accel) {
    const float sign = distance < 0.0f ? -1.0f : 1.0f;
    float carried = p->speed * sign; /* current speed along the new direction */
    if (carried < 0.0f) carried = 0.0f;
    p->sign = sign;
    p->distance = absf(distance);
    p->top_speed = absf(top_speed);
    p->final_speed = absf(final_speed);
    p->rate = absf(accel);
    p->position = 0.0f;
    p->speed = carried * sign;
    p->state = p->distance > 0.0f ? PROFILE_RUNNING : PROFILE_DONE;
    if (p->state == PROFILE_DONE) p->speed = p->final_speed * sign;
}

void profile_set_speed(profile_t* p, float speed) {
    p->state = PROFILE_IDLE;
    p->accel = 0.0f;
    p->speed = speed;
}

void profile_update(profile_t* p, float dt) {
    if (p->state != PROFILE_RUNNING) {
        p->accel = 0.0f;
        return;
    }
    float v = p->speed * p->sign;
    const float remaining = p->distance - p->position;
    const float braking = (v * v - p->final_speed * p->final_speed) / (2.0f * p->rate);
    const float target = remaining <= braking ? p->final_speed : p->top_speed;
    const float v0 = v;
    if (v < target) {
        v += p->rate * dt;
        if (v > target) v = target;
    } else if (v > target) {
        v -= p->rate * dt;
        if (v < target) v = target;
    }
    const float creep = MIN_CREEP * p->top_speed;
    if (remaining > 0.0f && v < creep) v = creep; /* never stall short of the end */
    p->position += v * dt;
    p->accel = (v - v0) / dt * p->sign;
    if (p->position >= p->distance) {
        p->position = p->distance;
        v = p->final_speed;
        p->state = PROFILE_DONE;
    }
    p->speed = v * p->sign;
}

bool profile_done(const profile_t* p) {
    return p->state != PROFILE_RUNNING;
}
