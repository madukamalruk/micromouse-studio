# MMSim — real-time Software-in-the-Loop Micromouse Simulator — PRD

_Status: **Approved 2026-11-20** · Last updated: 2026-11-20_

Reference brief: [`docs/reference/mms-analysis.md`](reference/mms-analysis.md).

## 1. Problem
`mackorone/mms` lets you test *maze-solving logic* but not a *micromouse*: it teleports a polygon between half-cell positions, answers wall queries from ground truth, and has no notion of time, motors, encoders, sensors or control loops. Bugs that kill real runs — PID tuning, sensor thresholds, profile timing, turn geometry, encoder/gyro fusion — are invisible until the robot is on the table. Peter Harrison (micromouseonline) demonstrated the alternative: the *same mouse code* builds either for the robot or inside a real-time simulator, and neither side knows about the other. No open, Windows-friendly tool does this well today.

## 2. Users
- **Primary:** Maduka — building and tuning the Neomouse V2 (STM32F405) firmware; wants to develop algorithms *and* control code off-robot.
- **Secondary:** other micromouse builders / students (incl. existing mms users who want their stdio solvers to keep working).

## 3. Goals (v1 = project Phase 1)
- **G1 — SIL execution:** unmodified C firmware written against a small HAL (`mmhal/hal.h`) runs inside the simulator, with a 1 kHz "systick ISR" and a background main loop, exactly as it would on an MCU.
- **G2 — Continuous physics:** 2-D differential-drive dynamics with DC-motor model, fixed-timestep integration, wall collisions, simulated encoders, gyro and raycast distance sensors with configurable noise.
- **G3 — Real time & deterministic:** runs at 1× wall-clock (or faster/slower/paused/single-step); same firmware + maze + robot config + seed ⇒ bit-identical run.
- **G4 — Exceeds mms:** live telemetry plots, path trace + sensor-ray overlay, cell colour/text/wall annotations, headless batch runner, plus an **mms stdio compatibility mode** so existing mms solvers run unchanged on top of the physics.
- **G5 — Phase-2-ready:** robot geometry, motors, encoders and sensors come from a config file and pluggable sensor models, so the Neomouse V2 can be added later as data + a new HAL backend without touching the simulator core.

## 4. Non-goals (v1)
- **Neomouse V2 specifics** (exact STM32F405 pin/timer mapping, VL53L0X driver behaviour, measured motor constants, real IR emitter/receiver curves, STM32 `hal_stm32.c` backend) → **Phase 2**. v1 ships a *generic classic mouse* config and a documented slot where Neomouse plugs in.
- Instruction-level MCU emulation (QEMU/Renode), register-level peripheral simulation, RTOS simulation.
- 3-D rendering; full rigid-body physics engine (Box2D etc.); tyre-slip models beyond a simple optional lateral-slip clamp.
- Maze editor GUI (load files only; editing = text editor). Half-size (32×32) mazes load and run but get no special tuning.
- Networking, multi-mouse races, cloud anything, macOS/Linux packaging (code stays portable; only Windows 11 is built/tested).
- Flashing hardware from the simulator.

## 5. Features & acceptance criteria
| ID | Feature | Priority | Acceptance criteria (testable) |
|----|---------|----------|--------------------------------|
| F1 | **Maze loading** — mms/mazefiles text map (`+`/`o` posts), `.num`, `.maz` binary; S/G markers; default 2×2 centre goal for 16×16 | Must | Given `uk1987f.txt`, when loaded, then 16×16, start (0,0) facing north, goal = {(7,7),(7,8),(8,7),(8,8)}, and wall set equals a checked-in golden hash. Round-trip text→`.maz`→text is lossless for all 16×16 files in `mazefiles/classic`. Malformed files (non-rectangular, unenclosed, inconsistent shared walls) are rejected with a line-numbered error. |
| F2 | **Maze validation report** — mms "official" checks | Should | For each file the loader reports: unreachable cells, start-wall count, centre-entrance count, hollow centre post, wall-follower solvable. Unit tests cover each rule with a tiny hand-made maze. |
| F3 | **2-D renderer** — maze, posts, walls (real vs firmware-known), mouse body + wheels, pan/zoom camera, grid coords | Must | Window opens at 60 FPS (vsync) with a 16×16 maze; mouse/scroll pans/zooms; "fit maze" key; screenshot (F12) saves PNG. Manual check + smoke test that one frame renders headless-hidden without GL errors. |
| F4 | **Firmware HAL** — C11 header `mmhal/hal.h`: time, systick registration, motor PWM, battery V, encoders, gyro, distance sensors/emitters, LEDs/buttons, log, debug annotations & telemetry | Must | Demo firmware compiles with **both** host GCC (sim) and `arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -fsyntax-only` against the same header. Firmware sources include nothing but `mmhal/*.h` and the C standard library (enforced by CMake include paths **and** a boundary-check test). |
| F5 | **Firmware host** — loads firmware as a DLL, runs `fw_main()` on its own thread in lockstep with sim time; ISR callback at 1 kHz; reset = unload/reload (clean globals); hot-reload after rebuild | Must | Test firmware toggling an LED every 500 ms produces exactly 20 toggles in 10.000 s sim time at 1×, 10× and max speed. Reset restores all firmware globals to initial values. A firmware that never yields is detected (watchdog: no HAL call for 2 s wall time) and stopped with an error, not a hang. |
| F6 | **Differential-drive physics** — fixed dt = 0.25 ms physics substeps, 1 kHz control tick; DC motor (R, Kt, Ke, gear ratio, inertia, friction), wheel radius, track width, mass; semi-implicit integration | Must | Analytic tests: constant equal wheel speeds → straight line, heading drift < 1e-9 rad over 10 m; constant different speeds → circle radius within 0.1 % of `v/ω`; step voltage → motor speed time-constant within 2 % of `τ = J·R/(Kt·Ke)` model. |
| F7 | **Collision** — mouse body (rectangle + optional rounded nose) vs wall/post rectangles (12 mm walls, 180 mm pitch — configurable) | Must | Driving straight into a wall stops the mouse with body/wall penetration ≤ 0.5 mm, raises a `crash` event (logged + UI flash), and the run is marked crashed. No tunnelling at 5 m/s. |
| F8 | **Sensors** — N configurable raycast distance sensors (mount x/y/angle, range, cone or single ray, noise σ, quantisation, update rate, latency); two output models: `ToF` (mm) and `IR reflectance` (ADC counts ∝ f(d)); gyro z (noise, bias); encoders (CPR, quantisation, optional noise) | Must | Raycast vs known geometry within ±0.1 mm (noise off). With σ set, 10 k samples have mean/σ within 2 % of config. Encoder counts = `round(θ_wheel·CPR/2π)` exactly, noise off. Sensors read from firmware see values sampled at their configured rate & latency. |
| F9 | **Robot config file** — JSON: geometry, mass/inertia, motors, gearbox, wheels, encoders, gyro, sensors[], battery | Must | `config/robots/generic_classic.json` loads; schema errors name the offending key. A second config with different sensor layout runs the same firmware without recompiling the simulator. |
| F10 | **Demo firmware** — pure C11: wall-following sensor calibration, flood-fill search to goal and back, then fastest run with trapezoidal profiles, PD wall-centring, smooth (curved) 90° turns | Must | On `uk1987f.txt`, headless, seed 1: reaches goal in search run, returns, completes speed run, **zero crashes**, finishes in < 5 min sim time; result line printed and asserted by an integration test. Same firmware passes the ARM syntax-only build (F4). |
| F11 | **Sim controls** — play / pause / single-step (1 tick, 10 ms, 1 cell), speed 0.1×–64× + "max", reset, reload firmware, choose maze/config/firmware | Must | At 1× over 60 s, sim time tracks wall clock within ±1 %. Pause freezes sim time exactly (no ticks advance). Step advances exactly the requested ticks. |
| F12 | **Live telemetry** — ImPlot panels: v/ω (actual vs setpoint), wheel speeds, motor voltages, PID errors, sensor channels, battery; firmware can publish named channels via `hal_telemetry()`; rolling window + CSV export | Must | Firmware-published channel appears in a plot within 1 frame; CSV export of a 10 s window has 10 000 rows at 1 kHz with correct timestamps. |
| F13 | **Overlays & annotations** — path trace (colour by speed), sensor rays with hit points, firmware-set cell colours/text/known walls (mms colour set) | Must | Annotation calls from firmware render within 1 frame; toggles per overlay; trace persists across a run and clears on reset. |
| F14 | **mms stdio compatibility mode** — spawn an mms solver exe, speak the full mms API (§2 of reference brief incl. stats/score, reset), execute moves with a built-in motion controller on the physics | Must | The upstream `mms-cpp` example algorithm (left-wall follower) runs unchanged and its `setColor/setText/setWall` output appears; command-level unit tests cover every command and the `ackReset`/score rules against values computed by hand. Wall queries answer from ground truth by default (mms-identical) or from sensors (option). |
| F15 | **Headless batch runner** `mmsim-cli` — run a firmware (or mms solver) across a list/glob of mazes at max speed; per-maze result (solved, search time, speed-run time, distance, crashes, mms score) as table + JSON/CSV | Must | `mmsim-cli --firmware demo --mazes mazes/classic/*.txt --json out.json` completes; exit code non-zero if any crash; ≥ 50× real time for the demo on one core. |
| F16 | **Deterministic replay** — seeded RNG per noise source; run log (inputs + seed) replays bit-identically | Should | Two runs with same inputs ⇒ identical SHA-256 of trajectory log; different seed ⇒ different hash. |
| F17 | **Session save** — remember last maze/config/firmware/layout (imgui.ini + `mmsim.json`) | Could | Restart restores last selections. |

## 6. User flows
1. **SIL development (main):** edit firmware C → `cmake --build` (rebuilds firmware DLL) → sim hot-reloads → press Play → watch mouse + plots → pause/step at the problem → tweak → repeat. Later the same `.c` files go into the STM32CubeIDE project with `hal_stm32.c` (Phase 2).
2. **Algorithm regression:** `mmsim-cli` over all classic mazes → JSON report → compare with previous run.
3. **mms user:** choose "mms solver" mode → point at their solver exe → run on physics with the familiar API.

## 7. Constraints
- **Platform:** Windows 11 x64 desktop. Code kept portable (no Win32 outside `sim/platform/`).
- **Toolchain:** MSYS2 UCRT64 — GCC (C11/C++20), CMake, Ninja, SDL2 from pacman; Dear ImGui (docking branch) + ImPlot fetched at pinned versions. ARM syntax check uses STM32CubeIDE's bundled `arm-none-eabi-gcc` 14.3.
- **Disk:** project on `D:\Projects\micromouse-sim`; MSYS2 at **`D:\msys64`** (~1.5–2 GB) to conserve C: space.
- **Deadline / budget:** none stated; all deps free & permissively licensed (zlib/MIT).
- **Performance:** 60 FPS UI with 16×16 maze + 4 plots; ≥ 50× real time headless; physics tick ≤ 20 µs on this machine.
- **Offline:** fully offline after first dependency fetch.

## 8. Physics & timing accuracy requirements
| Quantity | Requirement |
|---|---|
| Physics step | fixed 0.25 ms (4 substeps per 1 ms control tick); never variable with frame rate |
| Control tick (firmware ISR) | exactly every 1.000 ms sim time; jitter 0 in sim time |
| Real-time tracking at 1× | ±1 % over 60 s; if host can't keep up, sim slows (no skipped ticks) and UI shows "behind real time" |
| Kinematics | straight-line heading error < 1e-9 rad / 10 m; arc radius error < 0.1 % |
| Motor model | first-order step response τ within 2 % of analytic |
| Raycast | ±0.1 mm vs analytic geometry (noise off) |
| Encoders | exact quantisation, no drift (integer counts from accumulated wheel angle) |
| Determinism | bit-identical across runs on same build + inputs + seed |
| Units | SI internally (m, s, rad, V, A); firmware-facing values in the units the HAL documents |

## 9. Data, auth & security
- Data stored: local config/maze/log files only. No accounts, no network, no secrets.
- Firmware DLLs and mms solver exes are **user code executed locally** — the sim only loads from paths the user selects; mms solvers run as child processes with pipes only.

## 10. Integrations
- `micromouseonline/mazefiles` (fetched locally, no licence → not committed).
- `mackorone/mms` solver templates (e.g. `mms-cpp`) for compatibility testing (fetched, not committed).
- STM32CubeIDE ARM GCC for the firmware portability check.

## 11. Phase-2 boundary (designed now, implemented later)
- `hal/include/mmhal/hal.h` is the **only** contract. Phase 2 adds `hal_stm32.c` in the Neomouse repo — zero simulator changes.
- `config/robots/neomouse_v2.json` added in Phase 2 (geometry, measured motor constants, 6 V motor cap, VL53L0X timing ~33 ms/range, IR curves).
- Sensor models behind `ISensorModel` (`ToF`, `IRReflectance`); a `VL53L0X` model (timing budget, range status, max range, crosstalk) can be added as a new class + JSON `type`.
- Motor config carries `max_voltage`; sim logs a warning when commanded voltage exceeds it (mirrors the N20 ≤ 6.0 V rule).

## 12. Success metrics
- Demo firmware completes UK 1987 search + speed run crash-free in the GUI at 1× and headless.
- An unmodified mms solver runs in compatibility mode.
- Batch run over all `classic/` mazes produces a report; ≥ 90 % solved by demo firmware without crash (failures listed, not hidden).
- All tests green via one `ctest` command.

## 13. Open questions (resolved 2026-11-20)
- [x] MSYS2 install location → **`D:\msys64`**; build tools, CMake and Ninja run from there.
- [x] Generic mouse defaults → **approved**: classic mouse (≈ 100 g, 32 mm wheels, 72 mm track, 4 IR + gyro) for Phase 1; Neomouse V2 parameters injected in Phase 2.
- [x] GitHub remote → **yes, later**; keep the CI workflow ready. Scan diffs and always ask before any `git push`.
