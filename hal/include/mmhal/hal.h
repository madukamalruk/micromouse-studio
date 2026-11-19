/*
 * mmhal/hal.h — the ONLY contract between micromouse firmware and the world.
 *
 * Firmware includes this header and the C standard library, nothing else.
 * The simulator implements it in sim/hal_sim; real hardware implements it in a
 * board backend (Phase 2: hal_stm32.c for Neomouse V2). Neither side knows the
 * other exists.
 *
 * Rules: C11 only, no simulator types, compiles for Cortex-M4.
 *
 * Execution model (identical intent on MCU and in the simulator):
 *   - fw_main() is the firmware entry point (the body of main() on the MCU).
 *   - The handler set with hal_set_systick_handler() runs every 1 ms (SysTick ISR).
 *   - Blocking calls (hal_delay_ms, hal_wait_tick) are where the main loop yields.
 *     In the simulator the main loop advances ONLY at these calls, so a loop that
 *     never calls a blocking HAL function hangs the simulation (watchdog reports it).
 *   - Time does not advance while firmware code runs. A busy-wait on hal_millis()/
 *     hal_micros() therefore implicitly yields one tick every 256 reads.
 *   - Calling a blocking function from the SysTick ISR is a fault (the host stops the
 *     firmware and reports it).
 *   - In the simulator the ISR never preempts the main loop mid-instruction
 *     (lockstep scheduling). Code must still protect shared data as on hardware.
 */
#ifndef MMHAL_HAL_H
#define MMHAL_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MMHAL_VERSION 1

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { HAL_LEFT = 0, HAL_RIGHT = 1 } hal_side_t;

/* ---- Lifecycle (implemented BY the firmware) -------------------------------- */
void fw_main(void);

/* ---- Time --------------------------------------------------------------------- */
uint32_t hal_millis(void);                       /* ms since boot */
uint32_t hal_micros(void);                       /* us since boot */
void hal_delay_ms(uint32_t ms);                  /* block >= ms; 0 returns at once. Not from ISR. */
void hal_wait_tick(void);                        /* block until the next 1 ms tick. Not from ISR. */
void hal_set_systick_handler(void (*isr)(void)); /* NULL disables; called every 1 ms */

/* ---- Actuators ---------------------------------------------------------------- */
void hal_motor_set_duty(hal_side_t side, float duty); /* -1..+1 of battery voltage; + = forward */
float hal_battery_voltage(void);                      /* volts */

/* ---- Proprioception ----------------------------------------------------------- */
int32_t hal_encoder_count(hal_side_t side); /* quadrature counts, + = wheel forward, wraps */
float hal_gyro_z_dps(void);                 /* yaw rate, deg/s, + = counter-clockwise (left turn) */

/* ---- Exteroception ------------------------------------------------------------ */
uint8_t hal_sensor_count(void);
uint16_t hal_sensor_read(uint8_t ch); /* raw: mm for ToF, ADC counts for IR */
void hal_emitter_set(uint8_t ch, bool on);

/* ---- User interface ----------------------------------------------------------- */
void hal_led_set(uint8_t led, bool on);
bool hal_button_pressed(uint8_t btn);
void hal_log(const char* fmt, ...); /* printf-style; serial on hardware */

/* ---- Debug / visualisation (no-ops or serial on hardware) ---------------------- */
void hal_telemetry(const char* channel, float value);
void hal_annotate_cell_color(uint8_t x, uint8_t y, char mms_color);   /* mms colour chars */
void hal_annotate_cell_text(uint8_t x, uint8_t y, const char* text);  /* <= 10 chars */
void hal_annotate_wall(uint8_t x, uint8_t y, char dir, bool present); /* dir: n e s w */
void hal_annotate_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* MMHAL_HAL_H */
