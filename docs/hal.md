# HAL reference (`hal/include/mmhal/hal.h`, MMHAL_VERSION 2)

The HAL is the **only** contract between firmware and the world. Firmware includes
`<mmhal/hal.h>` and the C standard library, nothing else (checked by
`tools/check_hal_boundary.py`). The simulator implements it in `sim/hal_sim/hal_sim.c`; a
board implements it in its own backend (Phase 2: `hal_stm32.c`).

## Execution model

| Topic | Rule |
|---|---|
| Entry | `void fw_main(void)` is implemented by the firmware (the body of `main()` on the MCU). Returning from it halts the firmware. |
| Tick | The world advances in 1 ms ticks (4 physics substeps of 250 us). |
| Yielding | Time only advances while the main loop is blocked in `hal_delay_ms` or `hal_wait_tick`. Code between those calls takes zero simulated time. |
| Busy waits | Polling `hal_millis()`/`hal_micros()` yields one tick every 256 reads, so `while (hal_millis() < t) {}` still works. |
| Hang | A loop that never blocks stops the simulation; the watchdog reports it after 2 s of wall time and the session must be reset. |
| SysTick | `hal_set_systick_handler(isr)` runs `isr` once per tick, before the main loop resumes. It never preempts the main loop mid-statement, but protect shared data as you would on hardware. |
| ISR rules | Calling a blocking function from the ISR is a fault: the firmware is stopped and the error is shown. |
| Determinism | Same maze + robot config + seed + firmware = bit-identical run. |

## Functions

### Time
| Function | Meaning |
|---|---|
| `uint32_t hal_millis(void)` | ms since boot (wraps after 49.7 days). |
| `uint32_t hal_micros(void)` | us since boot. |
| `void hal_delay_ms(uint32_t ms)` | Blocks at least `ms`; `0` returns at once. Not from the ISR. |
| `void hal_wait_tick(void)` | Blocks until the next 1 ms tick. Not from the ISR. |
| `void hal_set_systick_handler(void (*isr)(void))` | 1 kHz handler; `NULL` disables. |

### Actuators
| Function | Meaning |
|---|---|
| `void hal_motor_set_duty(hal_side_t side, float duty)` | `-1..+1` of battery voltage, `+` = forward. The terminal voltage is clamped to the config's `motor.max_voltage` (6.0 V for N20 motors on 2S) and over-voltage requests are counted. Keep duty <= `max_voltage / battery` and coast >= 20 ms before reversing on real hardware. |
| `float hal_battery_voltage(void)` | Volts (constant in the current model). |

### Proprioception
| Function | Meaning |
|---|---|
| `int32_t hal_encoder_count(hal_side_t side)` | Counts; `+` = wheel forward unless the config inverts a channel; wraps. Resolution = `encoder.counts_per_motor_rev * gear_ratio` per wheel turn. |
| `void hal_get_encoders(int32_t* l, int32_t* r)` | Both channels at the same instant. |
| `float hal_gyro_z_dps(void)` | Yaw rate, deg/s, `+` = counter-clockwise. Measured: noise, bias, bias walk, quantisation. |
| `void hal_get_gyro(hal_gyro_t*)` / `void hal_get_accel(hal_accel_t*)` | Body frame: x forward, y left, z up; accel includes gravity (+9.81 on z). |

### Exteroception
| Function | Meaning |
|---|---|
| `uint8_t hal_sensor_count(void)` | Number of distance channels. |
| `uint16_t hal_sensor_read(uint8_t ch)` | Raw: mm for ToF, ADC counts for IR. |
| `uint16_t hal_get_distance(uint8_t ch)` | mm (IR: the board's inverse calibration applied); `HAL_DISTANCE_NONE` (0xFFFF) = nothing in range. |
| `void hal_emitter_set(uint8_t ch, bool on)` | IR emitter; on by default (the board pulses them). Off = ambient only. |

Channel order on 4-sensor boards: **0 left** (side or diagonal), **1 front-left**,
**2 front-right**, **3 right**. Sensors have their own rate and latency from the config, so a
reading can be up to one sample period old.

### User interface
| Function | Meaning |
|---|---|
| `void hal_led_set(uint8_t led, bool on)` | LEDs 0..7 (shown in the Run panel, logged with time). |
| `bool hal_button_pressed(uint8_t btn)` | Buttons 0..7 (GUI "Button N at boot" check boxes). |
| `void hal_log(const char* fmt, ...)` | printf-style line; Log panel in the simulator, serial on hardware. |

### Debug and visualisation (no-ops or serial on hardware)
| Function | Meaning |
|---|---|
| `void hal_telemetry(const char* channel, float value)` | Named channel, plotted live and exported to CSV. |
| `void hal_annotate_cell_color(uint8_t x, uint8_t y, char c)` | mms colour chars `kbacgorwyBACGORWY`; others ignored. |
| `void hal_annotate_cell_text(uint8_t x, uint8_t y, const char* t)` | <= 10 chars shown in the cell; empty removes it. |
| `void hal_annotate_wall(uint8_t x, uint8_t y, char dir, bool present)` | The firmware's wall belief (`n e s w`), drawn against the truth. |
| `void hal_annotate_clear(void)` | Clears colours, text and believed walls. |

Cells outside the maze are ignored, so a buggy firmware cannot grow the simulator's memory.

## Changing the contract

Any change to `hal.h` bumps `MMHAL_VERSION`, gets an entry in `docs/decisions.md`, and updates
both `sim/hal_sim/hal_sim.c` and every hardware backend. The header may only use
`stdint.h`, `stdbool.h` and `stddef.h`.
