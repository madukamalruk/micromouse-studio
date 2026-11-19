/*
 * Simulator <-> firmware-DLL binary interface. SIMULATOR-SIDE ONLY: firmware never
 * includes this. hal_sim.c (linked into every firmware DLL) implements mmhal/hal.h by
 * forwarding to this table, which the host hands over via mmsim_fw_bind().
 */
#ifndef MMSIM_HOST_API_H
#define MMSIM_HOST_API_H

#include <stdbool.h>
#include <stdint.h>

#define MMSIM_HOST_ABI 2

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mmsim_host_api {
    uint32_t abi; /* MMSIM_HOST_ABI */
    void* ctx;    /* passed back as first argument */

    uint32_t (*millis)(void* ctx);
    uint32_t (*micros)(void* ctx);
    void (*delay_ms)(void* ctx, uint32_t ms);
    void (*wait_tick)(void* ctx);
    void (*set_systick)(void* ctx, void (*isr)(void));

    void (*motor_set_duty)(void* ctx, int side, float duty);
    float (*battery_voltage)(void* ctx);
    int32_t (*encoder_count)(void* ctx, int side);
    float (*gyro_z_dps)(void* ctx);

    void (*imu)(void* ctx, float gyro_dps[3], float accel_mps2[3]);

    uint8_t (*sensor_count)(void* ctx);
    uint16_t (*sensor_read)(void* ctx, uint8_t ch);
    uint16_t (*sensor_distance)(void* ctx, uint8_t ch);
    void (*emitter_set)(void* ctx, uint8_t ch, bool on);

    void (*led_set)(void* ctx, uint8_t led, bool on);
    bool (*button_pressed)(void* ctx, uint8_t btn);
    void (*log)(void* ctx, const char* line);

    void (*telemetry)(void* ctx, const char* channel, float value);
    void (*annotate_cell_color)(void* ctx, uint8_t x, uint8_t y, char color);
    void (*annotate_cell_text)(void* ctx, uint8_t x, uint8_t y, const char* text);
    void (*annotate_wall)(void* ctx, uint8_t x, uint8_t y, char dir, bool present);
    void (*annotate_clear)(void* ctx);
} mmsim_host_api;

/* Exported by every firmware DLL (implemented in hal_sim.c). */
typedef uint32_t (*mmsim_fw_abi_fn)(void); /* (MMSIM_HOST_ABI << 16) | MMHAL_VERSION */
typedef void (*mmsim_fw_bind_fn)(const mmsim_host_api* api);
typedef void (*mmsim_fw_main_fn)(void); /* calls fw_main() */

#ifdef __cplusplus
}
#endif

#endif /* MMSIM_HOST_API_H */
