#include "motors.h"

#include <stdint.h>

#include "config.h"
#include "mmhal/hal.h"

typedef struct {
    int8_t dir;             /* direction of the last voltage applied: -1, 0, +1 */
    uint8_t zeroed;         /* 1 once 0 V has been commanded after driving in `dir` */
    uint32_t zero_since_ms; /* hal_millis() when that 0 V started */
} motor_t;

static motor_t g_motor[2];

static float clampf(float v, float lim) {
    return v > lim ? lim : (v < -lim ? -lim : v);
}

/* Returns the voltage actually allowed now. Time based (hal_millis), not call counted: a
 * reversal is held at 0 V until hal_millis() - zero_since_ms >= MOTOR_REVERSAL_COAST_MS,
 * whatever the call rate. The coast starts at the first 0 V (or at the first reversal attempt
 * if the caller flips the sign directly). */
static float guard(motor_t* m, float volts, uint32_t now_ms) {
    int8_t want = 0;
    if (volts > MOTOR_DEADBAND_VOLTS) want = 1;
    if (volts < -MOTOR_DEADBAND_VOLTS) want = -1;

    if (want == 0) {
        if (m->dir != 0 && !m->zeroed) {
            m->zeroed = 1;
            m->zero_since_ms = now_ms;
        }
        return 0.0f;
    }
    if (m->dir != 0 && want != m->dir) {
        if (!m->zeroed) {
            m->zeroed = 1;
            m->zero_since_ms = now_ms;
        }
        if ((uint32_t)(now_ms - m->zero_since_ms) < (uint32_t)MOTOR_REVERSAL_COAST_MS)
            return 0.0f; /* still coasting */
    }
    m->dir = want;
    m->zeroed = 0;
    return volts;
}

void motors_set_volts(float left, float right) {
    float vbat = hal_battery_voltage();
    if (vbat < 1.0f) vbat = 1.0f;
    /* Never above the 6 V motor cap, and never above 95 % duty even when the pack sags below
     * 6 V (100 % PWM is forbidden). */
    const float duty_lim = 0.95f * vbat;
    const float lim = duty_lim < MOTOR_MAX_VOLTS ? duty_lim : MOTOR_MAX_VOLTS;
    const uint32_t now = hal_millis();
    const float vl = guard(&g_motor[0], clampf(left, lim), now);
    const float vr = guard(&g_motor[1], clampf(right, lim), now);
    hal_motor_set_duty(HAL_LEFT, vl / vbat);
    hal_motor_set_duty(HAL_RIGHT, vr / vbat);
}

void motors_stop(void) {
    motors_set_volts(0.0f, 0.0f);
}

void motors_reset(void) {
    for (int i = 0; i < 2; ++i) {
        g_motor[i].dir = 0;
        g_motor[i].zeroed = 0;
        g_motor[i].zero_since_ms = 0;
    }
    hal_motor_set_duty(HAL_LEFT, 0.0f);
    hal_motor_set_duty(HAL_RIGHT, 0.0f);
}
