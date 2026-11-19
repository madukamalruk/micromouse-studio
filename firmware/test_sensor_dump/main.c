/* test_sensor_dump: exercises HAL v2 sensing. Standing still, it streams every distance
 * channel, both encoders and the IMU as telemetry at 100 Hz and logs a summary at 10 Hz. */
#include "mmhal/hal.h"

#define MAX_CH 8

static const char* const k_dist[MAX_CH] = {"dist0", "dist1", "dist2", "dist3",
                                           "dist4", "dist5", "dist6", "dist7"};
static const char* const k_raw[MAX_CH] = {"raw0", "raw1", "raw2", "raw3",
                                          "raw4", "raw5", "raw6", "raw7"};

void fw_main(void) {
    const uint8_t n = hal_sensor_count() < MAX_CH ? hal_sensor_count() : MAX_CH;
    uint32_t loops = 0;
    hal_log("sensors %u", (unsigned)n);
    for (;;) {
        int32_t left = 0, right = 0;
        hal_gyro_t g;
        hal_accel_t a;
        uint16_t d[MAX_CH] = {0};
        hal_get_encoders(&left, &right);
        hal_get_gyro(&g);
        hal_get_accel(&a);
        for (uint8_t ch = 0; ch < n; ++ch) {
            d[ch] = hal_get_distance(ch);
            hal_telemetry(k_dist[ch], (float)d[ch]);
            hal_telemetry(k_raw[ch], (float)hal_sensor_read(ch));
        }
        hal_telemetry("enc_left", (float)left);
        hal_telemetry("enc_right", (float)right);
        hal_telemetry("gyro_z", g.z_dps);
        hal_telemetry("accel_z", a.z_mps2);
        if (loops++ % 10u == 0u)
            hal_log("t=%lu d=%u,%u,%u,%u enc=%ld,%ld gz=%.2f az=%.2f", (unsigned long)hal_millis(),
                    (unsigned)d[0], (unsigned)d[1], (unsigned)d[2], (unsigned)d[3], (long)left,
                    (long)right, (double)g.z_dps, (double)a.z_mps2);
        hal_delay_ms(10);
    }
}
