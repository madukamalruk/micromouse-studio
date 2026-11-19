/* test_spin_turn: opposite duty -> turns in place; reports gyro and encoders each second. */
#include "mmhal/hal.h"

void fw_main(void) {
    hal_motor_set_duty(HAL_LEFT, -0.3f);
    hal_motor_set_duty(HAL_RIGHT, 0.3f);
    for (;;) {
        hal_delay_ms(1000);
        hal_log("t=%lu ms gyro=%.1f dps encL=%ld encR=%ld", (unsigned long)hal_millis(),
                (double)hal_gyro_z_dps(), (long)hal_encoder_count(HAL_LEFT),
                (long)hal_encoder_count(HAL_RIGHT));
    }
}
