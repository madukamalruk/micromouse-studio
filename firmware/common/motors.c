#include "motors.h"

#include <stdint.h>

#include "config.h"
#include "mmhal/hal.h"

typedef struct {
    int8_t dir;       /* direction of the last voltage applied: -1, 0, +1 */
    uint16_t idle_ms; /* consecutive ticks at 0 V */
    uint16_t coast_ms;
} motor_t;

static motor_t g_motor[2];

static float clampf(float v, float lim) {
    return v > lim ? lim : (v < -lim ? -lim : v);
}

/* Returns the voltage actually allowed this tick. */
static float guard(motor_t* m, float volts) {
    int8_t want = 0;
    if (volts > MOTOR_DEADBAND_VOLTS) want = 1;
    if (volts < -MOTOR_DEADBAND_VOLTS) want = -1;

    if (m->coast_ms > 0) { /* reversal in progress: hold 0 V */
        --m->coast_ms;
        ++m->idle_ms;
        if (m->coast_ms == 0) m->dir = 0;
        return 0.0f;
    }
    if (want == 0) {
        if (m->idle_ms < MOTOR_REVERSAL_COAST_MS) ++m->idle_ms;
        if (m->idle_ms >= MOTOR_REVERSAL_COAST_MS) m->dir = 0; /* rested long enough */
        return 0.0f;
    }
    if (m->dir != 0 && want != m->dir) {
        /* Change of direction: coast for whatever is left of the 20 ms. */
        const uint16_t left = (uint16_t)(MOTOR_REVERSAL_COAST_MS - m->idle_ms);
        m->coast_ms = (uint16_t)(left > 0 ? left - 1 : 0);
        ++m->idle_ms;
        if (m->coast_ms == 0) m->dir = 0;
        return 0.0f;
    }
    m->dir = want;
    m->idle_ms = 0;
    return volts;
}

void motors_set_volts(float left, float right) {
    float vbat = hal_battery_voltage();
    if (vbat < 1.0f) vbat = 1.0f;
    const float lim = vbat < MOTOR_MAX_VOLTS ? vbat : MOTOR_MAX_VOLTS;
    const float vl = guard(&g_motor[0], clampf(left, lim));
    const float vr = guard(&g_motor[1], clampf(right, lim));
    hal_motor_set_duty(HAL_LEFT, vl / vbat);
    hal_motor_set_duty(HAL_RIGHT, vr / vbat);
}

void motors_stop(void) {
    motors_set_volts(0.0f, 0.0f);
}

void motors_reset(void) {
    for (int i = 0; i < 2; ++i) {
        g_motor[i].dir = 0;
        g_motor[i].idle_ms = MOTOR_REVERSAL_COAST_MS;
        g_motor[i].coast_ms = 0;
    }
    hal_motor_set_duty(HAL_LEFT, 0.0f);
    hal_motor_set_duty(HAL_RIGHT, 0.0f);
}
