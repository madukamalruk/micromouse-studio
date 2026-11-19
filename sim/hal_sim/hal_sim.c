/*
 * Simulator implementation of mmhal/hal.h. Linked (static) into every firmware DLL.
 * Each hal_* call forwards to the host's function table bound via mmsim_fw_bind().
 */
#include <stdarg.h>
#include <stdio.h>

#include "mmhal/hal.h"
#include "sim/hal_sim/host_api.h"

#if defined(_WIN32)
#define MMSIM_EXPORT __declspec(dllexport)
#else
#define MMSIM_EXPORT __attribute__((visibility("default")))
#endif

static const mmsim_host_api* g_api = NULL;

MMSIM_EXPORT uint32_t mmsim_fw_abi(void) {
    return ((uint32_t)MMSIM_HOST_ABI << 16) | (uint32_t)MMHAL_VERSION;
}
MMSIM_EXPORT void mmsim_fw_bind(const mmsim_host_api* api) {
    g_api = api;
}
MMSIM_EXPORT void mmsim_fw_main(void) {
    fw_main();
}

#define CTX g_api->ctx

uint32_t hal_millis(void) {
    return g_api->millis(CTX);
}
uint32_t hal_micros(void) {
    return g_api->micros(CTX);
}
void hal_delay_ms(uint32_t ms) {
    g_api->delay_ms(CTX, ms);
}
void hal_wait_tick(void) {
    g_api->wait_tick(CTX);
}
void hal_set_systick_handler(void (*isr)(void)) {
    g_api->set_systick(CTX, isr);
}

void hal_motor_set_duty(hal_side_t side, float duty) {
    g_api->motor_set_duty(CTX, (int)side, duty);
}
float hal_battery_voltage(void) {
    return g_api->battery_voltage(CTX);
}
int32_t hal_encoder_count(hal_side_t side) {
    return g_api->encoder_count(CTX, (int)side);
}
float hal_gyro_z_dps(void) {
    return g_api->gyro_z_dps(CTX);
}

uint8_t hal_sensor_count(void) {
    return g_api->sensor_count(CTX);
}
uint16_t hal_sensor_read(uint8_t ch) {
    return g_api->sensor_read(CTX, ch);
}
void hal_emitter_set(uint8_t ch, bool on) {
    g_api->emitter_set(CTX, ch, on);
}

void hal_led_set(uint8_t led, bool on) {
    g_api->led_set(CTX, led, on);
}
bool hal_button_pressed(uint8_t btn) {
    return g_api->button_pressed(CTX, btn);
}

void hal_log(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    g_api->log(CTX, buf);
}

void hal_telemetry(const char* channel, float value) {
    g_api->telemetry(CTX, channel, value);
}
void hal_annotate_cell_color(uint8_t x, uint8_t y, char c) {
    g_api->annotate_cell_color(CTX, x, y, c);
}
void hal_annotate_cell_text(uint8_t x, uint8_t y, const char* t) {
    g_api->annotate_cell_text(CTX, x, y, t);
}
void hal_annotate_wall(uint8_t x, uint8_t y, char d, bool p) {
    g_api->annotate_wall(CTX, x, y, d, p);
}
void hal_annotate_clear(void) {
    g_api->annotate_clear(CTX);
}

/* ---- HAL v2 ------------------------------------------------------------------- */
void hal_get_encoders(int32_t* left, int32_t* right) {
    if (left) *left = g_api->encoder_count(CTX, HAL_LEFT);
    if (right) *right = g_api->encoder_count(CTX, HAL_RIGHT);
}

void hal_get_gyro(hal_gyro_t* out) {
    float g[3], a[3];
    if (!out) return;
    g_api->imu(CTX, g, a);
    out->x_dps = g[0];
    out->y_dps = g[1];
    out->z_dps = g[2];
}

void hal_get_accel(hal_accel_t* out) {
    float g[3], a[3];
    if (!out) return;
    g_api->imu(CTX, g, a);
    out->x_mps2 = a[0];
    out->y_mps2 = a[1];
    out->z_mps2 = a[2];
}

uint16_t hal_get_distance(uint8_t ch) {
    return g_api->sensor_distance(CTX, ch);
}
