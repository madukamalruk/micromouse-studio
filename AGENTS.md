# AGENTS.md — MMSim (real-time SIL micromouse simulator)

Read `docs/prd.md` (what/why) and `docs/plan.md` (current slice, checkboxes) before working. Decisions that change stack, architecture or scope go in `docs/decisions.md` (date · decision · why · alternatives) — update this file if a rule changes.

## Stack
- Windows 11 x64, **MSYS2 UCRT64** shell/toolchain: GCC (C11 for firmware, C++20 for sim), CMake ≥ 3.28, Ninja, SDL2 (pacman).
- Dear ImGui (docking branch) + ImPlot + nlohmann/json + doctest — pinned tags via CMake `FetchContent` (versions recorded in `cmake/deps.cmake`).
- Firmware portability check: STM32CubeIDE `arm-none-eabi-gcc` 14.3 (path in `cmake/arm-check.cmake`, overridable with `ARM_GCC`).

## Commands (run from repo root in an MSYS2 UCRT64 shell, or PowerShell with `D:\msys64\ucrt64\bin` first on PATH)
```
cmake --preset dev                      # configure (Debug, Ninja, build/dev)
cmake --build --preset dev              # build sim, CLI, firmware DLLs, tests
ctest --preset dev                      # all tests (unit + boundary + integration)
ctest --preset dev -L unit              # fast unit tests only
cmake --build --preset dev --target arm-check   # firmware compiles for Cortex-M4 (syntax-only)
build/dev/sim/app/mmsim.exe             # GUI
build/dev/sim/cli/mmsim-cli.exe --help  # headless runner
cmake --preset release && cmake --build --preset release   # optimised build
```
Format: `clang-format -i` (config `.clang-format`, Google-based, 100 cols). Warnings: `-Wall -Wextra -Wpedantic -Werror` on our targets (not on third-party).

## Layout
```
hal/include/mmhal/      hal.h — THE firmware↔world contract (C11, no sim types)
firmware/<name>/        pure C11 firmware; builds to a DLL for the sim, to .o for ARM check
sim/core/               maze, geometry, physics, motor, sensors, encoders, world, clock — no UI, no OS
sim/hal_sim/            implementation of hal.h for the sim (static lib linked into each firmware DLL; forwards to a vtable from the host)
sim/host/               firmware DLL loader, lockstep thread scheduler, mms stdio bridge
sim/platform/           the only place Win32 / process / DLL APIs are allowed
sim/app/                SDL2 + ImGui + ImPlot GUI
sim/cli/                mmsim-cli headless batch runner
config/robots/          robot JSON configs (generic_classic.json; neomouse_v2.json = Phase 2)
mazes/                  our own small test mazes (committed); external/mazefiles = fetched, gitignored
tests/                  doctest unit + integration tests, golden files
tools/                  check_hal_boundary.py, fetch_mazes.ps1, fetch_mms_cpp.ps1
docs/                   prd.md, plan.md, decisions.md, hal.md, phase2-neomouse.md, reference/
```

## HAL boundary rules (strict — a test enforces them)
1. **Firmware never includes simulator headers.** Files under `firmware/` may include only `mmhal/*.h`, C standard headers, and headers in their own firmware folder. No C++ in firmware. No OS APIs.
2. **The simulator never alters firmware logic.** No `#ifdef SIMULATOR` in firmware, no patching firmware state, no peeking at firmware globals. The sim only sees the firmware through HAL calls and the exported `fw_main` / ISR registration.
3. `hal.h` contains only C types (`stdint.h`, `stdbool.h`, `stddef.h`) — nothing that cannot compile on STM32.
4. Debug/visualisation HAL calls (`hal_annotate_*`, `hal_telemetry`) must be cheap no-ops or serial output on hardware; firmware must behave identically if they do nothing.
5. Firmware must yield: its main loop calls a blocking HAL function (`hal_delay_ms`, `hal_wait_tick`, …) regularly; busy loops without HAL calls are a bug (the host watchdog will stop them).
6. Changing `hal.h` = contract change: bump `MMHAL_VERSION`, note it in `docs/decisions.md`, update both the sim backend and (Phase 2) the STM32 backend.
7. `sim/core` must not depend on SDL/ImGui/OS headers; it is unit-tested headless.
8. Win32 APIs (`windows.h`: fibers, `LoadLibrary`) only in `sim/platform`. Firmware runs on a fiber driven by `sim/host/FirmwareHost`; never call a blocking HAL function from the SysTick ISR (host reports a fault).

## Simulation rules
- SI units inside `sim/core`. Fixed physics dt (0.25 ms) and 1 kHz control tick — never tie physics to frame rate.
- All randomness from seeded per-source RNGs owned by the world; no `rand()`, no time-based seeds in core.
- Robot/sensor parameters come from config JSON, never hard-coded in core (defaults live in the JSON).

## Workflow
- One plan slice at a time; write/adjust a failing test first, then make it pass.
- Before saying a slice/step is done: run its verification commands from `docs/plan.md` and show the output; then tick the checkbox.
- Commit after each green step (small, clear messages). **Never push without asking.**

## Safety & git hygiene
- `.gitignore` must cover `.env*` (keep `!.env.example`), `.claude/settings.local.json`, `build/`, `external/`. Verify with `git check-ignore -v` before the first commit.
- Scan the staged diff for secrets before every commit; stop and report if found. No secrets are expected in this project.
- Don't commit third-party maze files (`micromouseonline/mazefiles` has no licence) — fetch them with `tools/fetch_mazes.ps1`.
- Hardware habit carried into configs: motor `max_voltage` is respected; N20 on 2S LiPo ≤ 6.0 V (Phase 2 config).
