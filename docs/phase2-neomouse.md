# Phase 2 hand-off: Neomouse V2 (STM32F405)

Phase 1 ships a generic classic mouse. Phase 2 adds the Neomouse V2 as **data plus one new HAL
backend**: no simulator core changes are planned. This note lists what to add and where.

## 1. `config/robots/neomouse_v2.json`

Copy `generic_classic.json` and replace every value with a measured one. Keys (SI units):

| Group | Keys | How to measure |
|---|---|---|
| Body | `mass_kg`, `yaw_inertia_kg_m2`, `body.length_m`, `body.width_m`, `body.offset_x_m` (axle to body centre) | Scale; pendulum or CAD for inertia; calipers. |
| Wheels | `wheel.radius_m`, `wheel.inertia_kg_m2`, `track_width_m` | Roll 10 turns and measure distance; contact-patch centres. |
| Motors (N20) | `motor.resistance_ohm`, `kt_nm_per_a`, `ke_v_s_per_rad`, `gear_ratio`, `rotor_inertia_kg_m2`, `viscous_nm_s_per_rad`, `coulomb_nm`, `efficiency` | R with the rotor locked; Ke from no-load speed vs voltage; friction from the start-up voltage. Fit with the telemetry CSV (`neomouse_pid_fit.m`, Slice 9.7). |
| Motor cap | `motor.max_voltage_v` = **6.0** | Hard limit for N20 on 2S LiPo; the World clamps to it. |
| Battery | `battery_voltage_v` (7.4 nominal 2S); OCV curve, capacity and `R_int` arrive with Slice 9.1 | Bench discharge test. |
| Encoders | `encoder.counts_per_motor_rev`, `invert_left`, `invert_right` | Count edges for one motor turn; check the sign by pushing the robot forward. |
| IMU | `imu.gyro_noise_dps`, `gyro_bias_dps`, `gyro_bias_walk_dps_per_sqrt_s`, `gyro_quant_dps`, `gyro_range_dps`, accel equivalents | 60 s stationary log: mean = bias, sigma = noise, Allan deviation for the walk. |
| Slip | `slip.enabled`, `mu`, `long_stiffness_n_per_mps`, `slip_gradient_deg_per_g` | Max cornering speed on a known radius; tyre data. |
| Sensors | explicit `sensors` array instead of `sensor_layout.preset` (see below) | Mount positions from CAD. |

Validation: `mmsim.exe --config config/robots/neomouse_v2.json` reports the first bad key by
name. Then run `mmsim-cli --config config/robots/neomouse_v2.json ...` on the classic set and
compare speeds and turn times with the real robot.

## 2. Sensors: VL53L0X slot

Each entry of `sensors` takes `name`, `type` (`"ir"` or `"tof"`), pose `x_m`, `y_m`,
`angle_deg` (robot frame: x forward, y left, from the axle centre), beam `fov_deg` and `rays`,
`range_min_m`, `range_max_m`, `rate_hz`, `latency_ms`, `noise_sigma`, `noise_rel`, `quant`.
Keep the HAL channel order (0 left, 1 front-left, 2 front-right, 3 right) so firmware does not
change.

The current `tof` model returns the nearest hit in the cone in mm. That covers a first
Neomouse config (VL53L0X: about 25 deg FOV, `rate_hz` ~30 for the 33 ms default timing budget,
`latency_ms` ~33, range up to ~1.2 m). A dedicated VL53L0X model adds, as a new sensor `type`
in `sim/core/sensors` + `robot_config` (one new branch, no other changes):

- timing budget -> sample period and latency (`timing_budget_ms`);
- range status (valid / sigma fail / signal fail / out of range) -> `HAL_DISTANCE_NONE`;
- distance-dependent noise and the measured max range on maze walls (white vs black tops);
- optional crosstalk/offset calibration terms.

Each extra field needs a unit test against values measured on the robot.

## 3. `hal_stm32.c` (in the Neomouse repo, not here)

Implement every function in `hal/include/mmhal/hal.h` on the STM32F405. Suggested mapping (adapt
to the board's CubeMX pin/timer assignment):

| HAL | STM32F405 |
|---|---|
| `fw_main` | Called from `main()` after HAL/CubeMX init. |
| `hal_millis`, `hal_micros` | `HAL_GetTick()`; DWT cycle counter / 168 for us. |
| `hal_delay_ms`, `hal_wait_tick` | Wait on the SysTick tick counter (`__WFI()` between ticks). |
| `hal_set_systick_handler` | Store the pointer; call it from `HAL_SYSTICK_Callback` (1 kHz). |
| `hal_motor_set_duty` | TIMx PWM compare (DRV8833-style H-bridge): clamp duty to `6.0 V / V_battery`, enforce >= 20 ms coast on reversal, never 100 % PWM. |
| `hal_battery_voltage` | ADC on the divider, filtered. |
| `hal_encoder_count`, `hal_get_encoders` | TIM2/TIM5 (32-bit) in encoder mode: `(int32_t)TIMx->CNT`. |
| `hal_gyro_z_dps`, `hal_get_gyro`, `hal_get_accel` | IMU over SPI, scaled to deg/s and m/s^2 (body frame x forward, y left, z up). |
| `hal_sensor_count`, `hal_sensor_read`, `hal_get_distance` | VL53L0X over I2C (continuous mode, cached last range) and/or IR ADC with the emitter pulse synchronised in the SysTick ISR; `HAL_DISTANCE_NONE` when out of range. |
| `hal_emitter_set` | Emitter GPIO enable. |
| `hal_led_set`, `hal_button_pressed` | GPIO. |
| `hal_log` | `vsnprintf` into a buffer, UART DMA (non-blocking). |
| `hal_telemetry`, `hal_annotate_*` | Optional binary frames over UART, or empty functions. |

Rules: the backend may use CubeMX/LL headers; firmware files never see them. Run
`cmake --build --preset dev --target arm-check` before every hardware build so firmware stays
portable.

## 4. Checklist

- [ ] `neomouse_v2.json` with measured values; `ctest` passes with it loaded in a config test.
- [ ] Demo / Neomouse firmware solves the classic set in `mmsim-cli` with the Neomouse config.
- [ ] Optional VL53L0X sensor type + tests.
- [ ] `hal_stm32.c` in the Neomouse repo; the same firmware folder builds in STM32CubeIDE.
- [ ] Compare sim vs robot: straight-line speed profile, 90 deg turn, sensor readings at fixed
      distances; feed differences back into the JSON.

## Battery block (Slice 9.1)
Turn on the pack model once the Neomouse pack is measured (R_int from a rest/load voltage pair, see the Tuning & Reports battery card):
```json
"battery": { "model": true, "cells": 2, "capacity_mah": 450, "r_int_ohm": 0.15,
             "soc0": 1.0, "electronics_a": 0.105, "reset_v": 6.0 }
```
With the model on, `hal_battery_voltage()` returns the sagging terminal voltage, so firmware that scales duty by `6.0 / V_bat` keeps the N20 motors at or below 6.0 V as the pack discharges.

