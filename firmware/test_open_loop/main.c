/* test_open_loop: equal fixed duty on both motors -> drives straight until it hits a wall.
 * Uses no sensors on purpose (Slice 3 crash/collision demo). */
#include "mmhal/hal.h"

#define DUTY 0.5f

void fw_main(void) {
    hal_log("open loop: duty %.2f, battery %.2f V", (double)DUTY, (double)hal_battery_voltage());
    hal_motor_set_duty(HAL_LEFT, DUTY);
    hal_motor_set_duty(HAL_RIGHT, DUTY);
    for (;;) {
        hal_telemetry("enc_left", (float)hal_encoder_count(HAL_LEFT));
        hal_telemetry("enc_right", (float)hal_encoder_count(HAL_RIGHT));
        hal_delay_ms(10);
    }
}
