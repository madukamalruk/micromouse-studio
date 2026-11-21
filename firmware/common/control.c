#include "control.h"

#include <stdint.h>

#include "config.h"
#include "mmhal/hal.h"
#include "motors.h"
#include "sensors.h"

#define DT 0.001f

profile_t g_fwd;
profile_t g_rot;

static volatile bool g_enabled;
static volatile bool g_steering;
static volatile float g_fwd_err, g_fwd_err_prev;
static volatile float g_rot_err, g_rot_err_prev;
static volatile float g_speed, g_distance;
static volatile float g_gyro_bias;
static volatile float g_steer_dps;
static int32_t g_enc_l, g_enc_r;
static uint32_t g_ticks;

static float sgn(float v) {
    return v > 0.0f ? 1.0f : (v < 0.0f ? -1.0f : 0.0f);
}

static float wheel_ff(float v, float a) {
    return KV_WHEEL * v + KA_WHEEL * a + KS_WHEEL * sgn(v);
}

static void control_isr(void) {
    /* Odometry */
    int32_t l, r;
    hal_get_encoders(&l, &r);
    const int32_t dl = (int32_t)((uint32_t)l - (uint32_t)g_enc_l);
    const int32_t dr = (int32_t)((uint32_t)r - (uint32_t)g_enc_r);
    g_enc_l = l;
    g_enc_r = r;
    const float fwd_change = 0.5f * (float)(dl + dr) * MM_PER_COUNT;
    const float rot_change = (hal_gyro_z_dps() - g_gyro_bias) * DT;
    g_distance += fwd_change;
    g_speed = fwd_change / DT;

    sensors_update();
    profile_update(&g_fwd, DT);
    profile_update(&g_rot, DT);

    if (!g_enabled) {
        motors_stop();
        return;
    }

    /* Wall following: only on straights at speed, and not when a front wall is so close
     * that the diagonals see it. */
    float steer = 0.0f;
    if (g_steering && g_fwd.speed > STEER_MIN_SPEED && !sensors_front_near()) {
        steer = STEER_KP * sensors_steering_error();
        if (steer > STEER_MAX_DPS) steer = STEER_MAX_DPS;
        if (steer < -STEER_MAX_DPS) steer = -STEER_MAX_DPS;
    }
    g_steer_dps = steer;

    g_fwd_err += g_fwd.speed * DT - fwd_change;
    g_rot_err += (g_rot.speed + steer) * DT - rot_change;
    const float fwd_out = FWD_KP * g_fwd_err + FWD_KD * (g_fwd_err - g_fwd_err_prev);
    const float rot_out = ROT_KP * g_rot_err + ROT_KD * (g_rot_err - g_rot_err_prev);
    g_fwd_err_prev = g_fwd_err;
    g_rot_err_prev = g_rot_err;

    /* Feed-forward per wheel from the set-point speeds/accelerations. */
    const float half_track = 0.5f * TRACK_MM * DEG_TO_RAD; /* mm of wheel travel per degree */
    const float vw = (g_rot.speed + steer) * half_track, aw = g_rot.accel * half_track;
    const float v = g_fwd.speed, a = g_fwd.accel;
    const float left = wheel_ff(v - vw, a - aw) + fwd_out - rot_out;
    const float right = wheel_ff(v + vw, a + aw) + fwd_out + rot_out;
    motors_set_volts(left, right);

    if (++g_ticks % 10 == 0) {
        hal_telemetry("v_set", g_fwd.speed);
        hal_telemetry("v", g_speed);
        hal_telemetry("w_set", g_rot.speed + steer);
        hal_telemetry("w", rot_change / DT);
        hal_telemetry("steer", steer);
        hal_telemetry("fwd_err", g_fwd_err);
        hal_telemetry("rot_err", g_rot_err);
    }
}

void control_init(void) {
    profile_reset(&g_fwd);
    profile_reset(&g_rot);
    motors_reset();
    hal_get_encoders(&g_enc_l, &g_enc_r);
    g_enabled = false;
    g_steering = false;
    control_clear_errors();
    hal_set_systick_handler(control_isr);
}

void control_clear_errors(void) {
    g_fwd_err = g_fwd_err_prev = 0.0f;
    g_rot_err = g_rot_err_prev = 0.0f;
}

void control_enable(bool on) {
    control_clear_errors();
    g_enabled = on;
}

void control_steering(bool on) {
    g_steering = on;
}
float control_fwd_error(void) {
    return g_fwd_err;
}
float control_rot_error(void) {
    return g_rot_err;
}
float control_speed(void) {
    return g_speed;
}
float control_distance(void) {
    return g_distance;
}

void control_calibrate_gyro(unsigned ms) {
    float sum = 0.0f;
    for (unsigned i = 0; i < ms; ++i) {
        hal_wait_tick();
        sum += hal_gyro_z_dps();
    }
    g_gyro_bias = ms ? sum / (float)ms : 0.0f;
}
