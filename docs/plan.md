# MMSim — Implementation plan

_Status: **Approved 2026-11-20** · Last updated: 2026-11-20 · PRD: [prd.md](prd.md) · Rules: [../AGENTS.md](../AGENTS.md)_

Every slice ends runnable and tested end-to-end. Tick a box only after its verification command has been run and its output shown. Sizes: S ≈ ½ day, M ≈ 1–2 days, L ≈ 3+ days (agent-assisted).

All commands below are PowerShell from `D:\Projects\micromouse-sim`, after:
```powershell
$env:Path = "D:\msys64\ucrt64\bin;" + $env:Path   # MSYS2 first, so the old C:\MinGW gcc 6.3 is never picked up
```

---

## Architecture at a glance

```
                ┌────────────────────── firmware_<name>.dll ─────────────────────┐
                │  firmware/*.c  (pure C11)  ──calls──►  mmhal/hal.h             │
                │                                         │ implemented by       │
                │                          sim/hal_sim (static lib: forwards to  │
                │                          a vtable handed in by the host)       │
                └──────────────── exports: fw_bind(vtable), fw_main() ───────────┘
                                                  ▲
 sim/host:  FirmwareHost (LoadLibrary copy → bind → run fw_main on fw thread, lockstep)
            MmsBridge    (child process + pipes, mms text API → motion controller)
                                                  ▲
 sim/core:  World { Maze, Robot{Motors, DiffDrive, Body}, Sensors[], Encoders, Gyro,
                    Clock(1 kHz tick, 4×0.25 ms physics substeps), RNG per source,
                    Collision, Annotations, TelemetryBus, TrajectoryLog }
                                                  ▲
 sim/app (SDL2 + ImGui + ImPlot GUI)     sim/cli (mmsim-cli headless batch runner)
```

**Lockstep scheduling (deterministic):** the sim thread owns time. Each 1 ms tick: (1) run physics substeps, (2) sample sensors due this tick, (3) call the firmware's registered systick ISR on the sim thread, (4) wake the firmware main thread if its wait condition (`hal_delay_ms` deadline, `hal_wait_tick`) is met and block until it calls the next blocking HAL function. The main thread therefore never runs concurrently with the ISR → no data races, bit-identical runs. *Limitation (documented):* true ISR preemption races are not reproduced.

**Reset/hot-reload:** the host copies `firmware_x.dll` to a temp name, loads it, binds the vtable, starts `fw_main`. Reset = stop thread at a HAL call, `FreeLibrary`, reload → all firmware globals return to initial values; a rebuilt DLL is picked up on reset.

---

## Slice 1 — Toolchain, repo, build system, headless test runner, smoke test (M)
**Goal:** one command builds and one command tests; the HAL boundary check exists from day one.

- [x] 1.1 Install toolchain (**asks you first** — installs software): MSYS2 installer (hash-checked via `winget download`) run unattended with `--root D:/msys64`, then in UCRT64: `pacman -S --needed mingw-w64-ucrt-x86_64-{gcc,cmake,ninja,gdb,SDL2,clang-tools-extra,python}`.
- [x] 1.2 `git init`; `.gitignore` (`build/`, `external/`, `.env*`, `!.env.example`, `.claude/settings.local.json`, `*.dll` outside build, imgui.ini); `.env.example` (comment only — project needs no secrets); `.clang-format`; `.editorconfig`. *`LICENSE` still pending your choice.*
- [x] 1.3 `CMakeLists.txt`, `CMakePresets.json` (`dev` = Debug+Ninja, `release`), `cmake/deps.cmake` (FetchContent pinned: doctest, nlohmann/json; ImGui/ImPlot added in Slice 2), `cmake/warnings.cmake`.
- [x] 1.4 Skeleton targets: `mmhal` (INTERFACE, `hal/include`), `simcore` (empty lib + `version()`), `tests/unit` (doctest) with one smoke test.
- [x] 1.5 `tools/check_hal_boundary.py` — scans `firmware/**/*.{c,h}` includes against the allow-list; registered as a ctest; plus a negative fixture proving it fails on `#include "sim/..."`.
- [x] 1.6 `.github/workflows/ci.yml` (windows-latest + `msys2/setup-msys2`, configure/build/ctest) — dormant until a remote exists; `docs/decisions.md` seeded with D1 (stack), D2 (DLL firmware + lockstep).

**Verify**
```powershell
gcc --version            # expect GCC 14+ from D:\msys64\ucrt64\bin
cmake --preset dev; cmake --build --preset dev; ctest --preset dev --output-on-failure
git check-ignore -v .env .env.local .claude/settings.local.json build/x   # each must be ignored
git check-ignore -v .env.example                                          # must NOT be ignored
```
Expected: `100% tests passed` (smoke + boundary + boundary-negative). First commit after secrets scan of staged diff.

---

## Slice 2 — Maze loader (`.txt` map / `.num` / `.maz`) + 2-D renderer + camera (M)
**Goal:** open `uk1987f.txt` and see it.

- [x] 2.1 `tools/fetch_mazes.ps1` → shallow-clones `micromouseonline/mazefiles` @ `070f716d2379` into `external/mazefiles` (gitignored). Our own tiny mazes in `mazes/test/` (committed).
- [x] 2.2 `sim/core/maze`: `Maze` (W×H, per-cell NESW bits, start, start heading, goal set), `parse_text_map` (any non-space = wall, `+`/`o` posts, S/G markers, `\r\n` tolerated), `parse_num`, `parse_maz`, `write_text_map`, `write_maz`; errors carry line/col.
- [x] 2.3 `sim/core/maze_validate`: enclosed, consistent, rectangular + "official" report (unreachable cells, 3-wall start, single centre entrance, hollow centre post, wall-follower-solvable via simulated left/right follower).
- [x] 2.4 Tests: golden hash for `uk1987f.txt`; round-trip text→maz→text for every `classic/*.txt` (skipped with a clear message if mazes not fetched); malformed fixtures; each validation rule.
- [x] 2.5 `sim/app`: SDL2 window + ImGui (docking) + ImPlot fetched & pinned; `MazeRenderer` (ImGui draw list on a dockable "Maze" viewport): posts, walls (true walls dim, firmware-known solid — mms convention), coordinates, distance-to-goal numbers when idle; camera pan (drag) / zoom (wheel) / `F` fit; maze picker (file list of `mazes/` + `external/mazefiles`); `F12` screenshot.
- [x] 2.6 CLI flags for testability: `mmsim --maze <file> --screenshot <png> --frames N --exit`.

**Verify**
```powershell
powershell -File tools/fetch_mazes.ps1
cmake --build --preset dev; ctest --preset dev -L maze --output-on-failure
build/dev/sim/app/mmsim.exe --maze external/mazefiles/classic/uk1987f.txt --screenshot build/uk1987.png --frames 5 --exit
```
Expected: maze tests pass (incl. all classic round-trips); `build/uk1987.png` shows the UK 1987 maze (checked visually and attached in chat).

---

## Slice 3 — Firmware HAL + firmware host + differential-drive physics + collision (L)
**Goal:** a C firmware DLL drives a physically simulated mouse into a wall and the sim reports the crash.

- [ ] 3.1 `hal/include/mmhal/hal.h` v0.1 (C11). Draft API:
  ```c
  #define MMHAL_VERSION 1
  /* lifecycle — implemented by firmware */
  void fw_main(void);                                   /* never returns on hardware */
  /* time */
  uint32_t hal_millis(void);  uint32_t hal_micros(void);
  void hal_delay_ms(uint32_t ms);                       /* blocking; yields to sim */
  void hal_wait_tick(void);                             /* block until next 1 kHz tick */
  void hal_set_systick_handler(void (*isr)(void));      /* called every 1 ms */
  /* actuators */
  void  hal_motor_set_duty(hal_side_t side, float duty);   /* -1..+1, × battery V */
  float hal_battery_voltage(void);
  /* proprioception */
  int32_t hal_encoder_count(hal_side_t side);
  float   hal_gyro_z_dps(void);
  /* exteroception */
  uint8_t  hal_sensor_count(void);
  uint16_t hal_sensor_read(uint8_t ch);                 /* raw units per sensor type (mm or ADC) */
  void     hal_emitter_set(uint8_t ch, bool on);
  /* UI */
  void hal_led_set(uint8_t led, bool on);  bool hal_button_pressed(uint8_t btn);
  void hal_log(const char *fmt, ...);
  /* debug/vis — no-ops or serial on hardware */
  void hal_telemetry(const char *channel, float value);
  void hal_annotate_cell_color(uint8_t x, uint8_t y, char mms_color);
  void hal_annotate_cell_text(uint8_t x, uint8_t y, const char *text);
  void hal_annotate_wall(uint8_t x, uint8_t y, char dir, bool present);
  void hal_annotate_clear(void);
  ```
- [ ] 3.2 `sim/hal_sim`: vtable struct + `fw_bind()` export; static lib linked into each firmware DLL.
- [ ] 3.3 `sim/platform`: DLL load/unload (copy-on-load), thread + condition-variable primitives wrapper.
- [ ] 3.4 `sim/host/FirmwareHost`: lockstep scheduler, ISR dispatch, reset/reload, watchdog (no HAL call for 2 s wall time → stop + error).
- [ ] 3.5 `sim/core`: `Clock`, `DcMotor` (R, L ignored v1, Kt, Ke, gear ratio, rotor+wheel inertia, viscous+Coulomb friction, `max_voltage` warning), `DiffDrive` (mass, yaw inertia, wheel radius, track; semi-implicit Euler at 0.25 ms; no-slip default, optional lateral-slip clamp), `Body` polygon, `Collision` (body vs wall/post AABBs, swept test against tunnelling → stop + `CrashEvent`).
- [ ] 3.6 `config/robots/generic_classic.json` + `RobotConfig` loader with keyed errors.
- [ ] 3.7 Test firmwares: `firmware/test_blink` (LED every 500 ms), `firmware/test_open_loop` (fixed duty → drive forward until crash).
- [ ] 3.8 GUI: mouse body + wheels drawn at continuous pose; Run/Reset buttons (minimal; full controls in Slice 6).
- [ ] 3.9 `arm-check` target: `arm-none-eabi-gcc -std=c11 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fsyntax-only -Wall -Werror -Ihal/include` on every firmware.

**Verify**
```powershell
cmake --build --preset dev; ctest --preset dev -L "physics|host|boundary" --output-on-failure
cmake --build --preset dev --target arm-check
build/dev/sim/app/mmsim.exe --maze mazes/test/corridor.txt --firmware test_open_loop --screenshot build/crash.png --run-for 3s --exit
```
Expected tests: straight-line heading drift < 1e-9 rad/10 m; circle radius within 0.1 %; motor τ within 2 %; blink = exactly 20 toggles in 10 s sim at 1×/10×/max; open-loop run ends with one `CrashEvent`, penetration ≤ 0.5 mm, no tunnelling at 5 m/s; reset restores firmware globals; watchdog fires on a deliberately non-yielding firmware fixture.

---

## Slice 4 — Raycast distance sensors, encoders, gyro, noise (M)
**Goal:** firmware sees the world only through realistic sensor data.

- [ ] 4.1 `sim/core/raycast`: ray vs wall/post rectangles (exact), optional cone = N rays → min.
- [ ] 4.2 `ISensorModel` + `ToFModel` (mm, range limit, σ, quantisation, rate, latency) + `IRReflectanceModel` (ADC = f(d, angle), emitter on/off, ambient, saturation). Selected by `"type"` in JSON.
- [ ] 4.3 `Encoder` (CPR × gear, integer counts from accumulated wheel angle, optional noise/missed counts), `Gyro` (σ, bias, bias drift).
- [ ] 4.4 `Rng` per source seeded from world seed (`splitmix64`/`xoshiro256**`), documented derivation.
- [ ] 4.5 GUI overlay: sensor rays + hit points (toggle).
- [ ] 4.6 Test firmware `firmware/test_sensor_dump` (logs all channels at 100 Hz).

**Verify**
```powershell
cmake --build --preset dev; ctest --preset dev -L sensors --output-on-failure
build/dev/sim/app/mmsim.exe --maze mazes/test/corridor.txt --firmware test_sensor_dump --screenshot build/rays.png --run-for 1s --exit
```
Expected: raycast ±0.1 mm vs analytic; noise mean/σ within 2 % over 10 k samples; encoder counts exact for known wheel rotations; latency/rate honoured (step change appears after configured delay); same seed ⇒ same sample stream.

---

## Slice 5 — Demo firmware: flood fill + smooth turns, real time on UK 1987 (L)
**Goal:** the headline demo — the same C files that would go onto the robot solve a classic maze in the sim.

- [ ] 5.1 `firmware/demo/` (C11, split like real firmware): `motion.c` (trapezoidal profiles for forward + rotation), `control.c` (1 kHz ISR: encoder/gyro odometry, PD velocity + ω loops, wall-centring from side sensors), `sensors.c` (wall detection thresholds, front-wall alignment), `maze.c` + `floodfill.c` (search to goal, return to start, compute fastest path), `moves.c` (cell-straight, smooth 90° turns as arc/clothoid-ish profiles, in-place 180°), `main.c` (`fw_main`: calibrate → search → return → speed run; LED/button states).
- [ ] 5.2 Firmware uses `hal_annotate_*` for known walls/flood values and `hal_telemetry` for v/ω setpoints & errors (visible later in Slice 6).
- [ ] 5.3 Real-time pacing in the GUI: 1× wall-clock tracking, "behind real time" indicator.
- [ ] 5.4 Internal headless harness in tests (`run_world(fw, maze, seed, limits)`) — reused by the CLI in Slice 8.
- [ ] 5.5 Integration tests: UK 1987 (seed 1) search+return+speed run, zero crashes, < 5 min sim time; plus 3 of our small test mazes.

**Verify**
```powershell
cmake --build --preset dev; ctest --preset dev -L integration --output-on-failure
cmake --build --preset dev --target arm-check     # demo firmware is ARM-clean
python tools/check_hal_boundary.py firmware/demo  # explicit boundary report
build/dev/sim/app/mmsim.exe --maze external/mazefiles/classic/uk1987f.txt --firmware demo   # manual: watch at 1×
```
Expected: integration tests print search/return/speed-run times and pass; real-time test: 60 s at 1× within ±1 %; manual run screenshot/GIF attached in chat.

---

## Slice 6 — Exceeding mms: telemetry, controls, trace, annotations, replay (M)
- [ ] 6.1 `TelemetryBus`: built-in channels (pose, v, ω, wheel speeds, motor V/I, battery, sensors, encoder counts) + firmware `hal_telemetry` channels; ring buffer (60 s @ 1 kHz).
- [ ] 6.2 ImPlot panels (dockable): v/ω actual vs setpoint, wheel speeds, motor voltages, PID errors, sensor channels; channel picker; pause-follows-sim; CSV export of visible window.
- [ ] 6.3 Path trace (colour by speed) + clear on reset; overlay toggles.
- [ ] 6.4 Controls: play/pause, step 1 tick / 10 ms / until next cell, speed 0.1×–64× + max, reset, reload firmware (hot-reload), maze/config/firmware selectors; keyboard shortcuts.
- [ ] 6.5 Full mms colour palette + cell text (≤10 chars) + known-wall rendering; idle distance numbers.
- [ ] 6.6 Deterministic replay: run record (maze hash, config hash, firmware DLL hash, seed) + trajectory SHA-256.

**Verify**
```powershell
cmake --build --preset dev; ctest --preset dev -L "telemetry|replay|controls" --output-on-failure
build/dev/sim/app/mmsim.exe --maze external/mazefiles/classic/uk1987f.txt --firmware demo --run-for 20s --export-csv build/t.csv --screenshot build/telemetry.png --exit
```
Expected: CSV of a 10 s window has 10 000 rows at 1 kHz; pause advances 0 ticks; step advances exactly N; same inputs ⇒ identical hash, different seed ⇒ different hash; screenshot shows plots + trace.

---

## Slice 7 — mms stdio compatibility mode (M)
- [ ] 7.1 `sim/platform/process`: spawn child with stdin/stdout/stderr pipes (Win32 `CreateProcess`), line reader, kill on reset.
- [ ] 7.2 `sim/host/MmsBridge`: parser + dispatcher for every command in the reference brief; responses identical to mms (`ack`/`crash`/`true`/`false`/ints/floats; unknown commands ignored).
- [ ] 7.3 Motion executor: built-in profiled controller (shares math with demo firmware's design but lives in the sim, since the solver isn't firmware) executes `moveForward[Half] N`, 90°/45° turns on the physics; crash ⇒ `crash`.
- [ ] 7.4 Wall queries: ground truth (default, mms-identical) or sensor-based (option); `wasReset`/`ackReset` + 15-pt penalty; stats & score formula.
- [ ] 7.5 Tests: command-level unit tests (every command, malformed args, score maths by hand); integration with a tiny test solver exe built in `tests/` and with upstream `mms-cpp` fetched by `tools/fetch_mms_cpp.ps1` (not committed).

**Verify**
```powershell
cmake --build --preset dev; ctest --preset dev -L mms --output-on-failure
build/dev/sim/app/mmsim.exe --maze external/mazefiles/classic/uk1987f.txt --mms-solver external/mms-cpp/build/main.exe
```
Expected: all mms tests pass; manual run shows the solver's colours/text/walls on the physics mouse.

---

## Slice 8 — Headless batch runner `mmsim-cli` + polish (M)
- [ ] 8.1 `sim/cli`: `--firmware NAME|--mms-solver EXE`, `--mazes GLOB...`, `--config`, `--seed`, `--max-sim-time`, `--jobs N` (parallel worlds), `--json/--csv out`; table to stdout; exit 1 on any crash/timeout.
- [ ] 8.2 Session save (last maze/config/firmware, ImGui layout).
- [ ] 8.3 `README.md` (build, run, write-your-own-firmware guide, HAL reference), `docs/hal.md`, Phase-2 hand-off notes (`docs/phase2-neomouse.md`: what to add — `neomouse_v2.json`, VL53L0X model, `hal_stm32.c` mapping).
- [ ] 8.4 Performance pass: physics tick ≤ 20 µs; headless ≥ 50× real time for demo.

**Verify**
```powershell
cmake --preset release; cmake --build --preset release; ctest --preset release --output-on-failure
build/release/sim/cli/mmsim-cli.exe --firmware demo --mazes "external/mazefiles/classic/*.txt" --jobs 8 --json build/classic.json
```
Expected: report for all classic mazes; ≥ 90 % solved crash-free (failures listed); speed factor printed ≥ 50×.

---

## Candidate project skills (create only after the pattern repeats twice)
- `add-sensor-model` — new `ISensorModel` + JSON type + tests.
- `add-firmware` — new firmware folder + CMake DLL target + arm-check + boundary test.
- `add-mms-command` — parser + response + unit test.

## Risks & open questions
- **Toolchain conflict:** old `C:\MinGW` (GCC 6.3) is on PATH → presets pin compilers to `D:\msys64\ucrt64\bin` explicitly; verified in Slice 1.
- **DLL + CRT:** firmware DLLs must not share CRT state wrongly → link `-static-libgcc`, no C++ in firmware, all host↔fw traffic via the vtable.
- **MCU idioms that don't yield** (`while(!flag);` spin-waits) hang the lockstep → rule 5 in AGENTS.md + watchdog; Phase 2 firmware may need `hal_wait_*` in a few spots.
- **ISR preemption not reproduced** (lockstep) — race bugs won't show in sim; documented limitation. Possible later "jittered preemption" mode.
- **Physics fidelity** — generic constants won't match Neomouse until Phase 2 measurements; plan a calibration step there (step-response + encoder logs from real robot via `ee-robotics-automation`).
- **Maze-file quirks** — mazefiles notes possible errors/duplicates; the loader reports, doesn't silently fix.
- **ImGui docking branch churn** — pin exact commit; upgrade deliberately.
- **Open (from PRD §13):** MSYS2 location; OK with generic classic mouse defaults; future GitHub remote?

## Next up
Slice 1 done (2026-11-20). Slice 2 done (2026-11-20): maze model + `.txt`/`.num`/`.maz` parsers/writers, validator, flood distances, geometry, SDL2+ImGui renderer with camera; 13/13 ctest green incl. GPU pixel checks on tiny4x4, UK 1987 and a 32x32. Nothing pushed.
Next: **Slice 3** — `mmhal/hal.h` v1, hal_sim + DLL firmware host (lockstep), DC motor + diff-drive physics, collision, `arm-check` target.
