# MMSim — real-time SIL micromouse simulator

MMSim runs **unchanged micromouse firmware** (pure C11 written against one small HAL header) on
a physics model of the robot, in lockstep, on Windows. The same `.c` files later go onto the
STM32 with a hardware HAL backend. It can also drive **mms-protocol solvers** (the
[mackorone/mms](https://github.com/mackorone/mms) text API) on the same physics.

- 2-D maze view (real vs firmware-known walls, cell colours/text, sensor beams, speed trace)
- Diff-drive physics: DC motors with a 6 V cap, wheel/rotor inertia, tyre slip, continuous
  collision with crash events; IR/ToF sensors, encoders and IMU with seeded noise
- Firmware as a DLL on a fiber (lockstep 1 kHz tick, SysTick ISR, watchdog, hot reload)
- Live ImPlot telemetry, CSV export, deterministic replay (trajectory SHA-256), timeline scrub
- `mmsim-cli`: headless batch runs over hundreds of mazes in parallel, JSON/CSV reports

## Requirements

- Windows 10/11, [MSYS2](https://www.msys2.org) at `D:\msys64` (UCRT64) with
  `pacman -S mingw-w64-ucrt-x86_64-{gcc,cmake,ninja,SDL2,clang-tools-extra} git python`
- Optional: STM32CubeIDE's `arm-none-eabi-gcc` for the Cortex-M4 portability check (`arm-check`)

The build uses `cmake/toolchain-msys2-ucrt64.cmake`; nothing is added to the system `PATH`.

## Build and test

```powershell
cmake --preset dev        # Debug (use "release" for speed)
cmake --build --preset dev
ctest --preset dev
```

Optional third-party data (gitignored, fetched on demand):

```powershell
tools/fetch_mazes.ps1      # micromouseonline/mazefiles -> external/mazefiles (no licence: never committed)
tools/fetch_mms_cpp.ps1    # mackorone/mms-cpp (MIT) -> external/mms-cpp, built by the tests
```

## Run

GUI (the maze library lists `mazes/test` and the fetched mazefiles):

```powershell
build/dev/sim/app/mmsim.exe --maze external/mazefiles/classic/uk1987f.txt --firmware demo_floodfill
```

Keys: Space play/pause, R reset, Ctrl+R rebuild-reload, N run until the next cell, Right step
10 ms (Shift 100 ms), +/- speed (0.1x-64x, Max), F12 screenshot. The last maze, robot config,
firmware/solver, seed, speed and window layout are restored at the next start (`mmsim.json`
and `imgui.ini` in the data root, gitignored; `--no-session` disables it). `mmsim.exe --help`
lists every option (`--run-for`, `--seed`, `--export-csv`, `--speed`, `--screenshot`, ...).

mms solver instead of firmware (any exe speaking the mms stdin/stdout API):

```powershell
build/dev/sim/app/mmsim.exe --maze external/mazefiles/classic/uk1987f.txt --mms-solver build/dev/tests/mms_cpp_example.exe
```

`--mms-walls sensors` answers wall queries from the simulated sensors instead of the maze.

Headless batch:

```powershell
build/release/sim/cli/mmsim-cli.exe --firmware demo --mazes "external/mazefiles/classic/*.txt" --jobs 8 --json build/classic.json
```

The exit code is 0 only if every maze was solved without a crash or timeout. See
`mmsim-cli --help`.

## Write your own firmware

1. Create `firmware/<name>/main.c` (more `.c`/`.h` files in the same folder are picked up).
2. Include only `<mmhal/hal.h>` and the C standard library; implement `void fw_main(void)`.
3. Build: `cmake --build --preset dev` produces `build/dev/firmware_bin/<name>.dll`; pick it
   in the GUI's Run panel or pass `--firmware <name>`.
4. `cmake --build --preset dev --target arm-check` compiles it for Cortex-M4 as well.

```c
#include <mmhal/hal.h>

void fw_main(void) {
    hal_log("hello from %u sensors", (unsigned)hal_sensor_count());
    for (;;) {
        const uint16_t front = hal_get_distance(1);
        const float duty = (front != HAL_DISTANCE_NONE && front < 60) ? 0.0f : 0.25f;
        hal_motor_set_duty(HAL_LEFT, duty);
        hal_motor_set_duty(HAL_RIGHT, duty);
        hal_wait_tick(); /* the main loop must block, or the simulation cannot advance */
    }
}
```

The full contract (units, channel order, timing, what blocks) is in [docs/hal.md](docs/hal.md).
`firmware/demo_floodfill` is a complete example: calibration, flood-fill search, return and a
profiled speed run. The rules the boundary checker enforces are in [AGENTS.md](AGENTS.md).

## Layout

```
hal/include/mmhal/hal.h   the only firmware <-> world contract
firmware/                 firmware folders (each -> <name>.dll)
sim/core/                 maze, physics, sensors, world (pure C++20, no OS calls)
sim/host/                 session, firmware host, mms bridge, telemetry, batch runner
sim/platform/             the only Win32 code (fibers, DLLs, child processes)
sim/app/  sim/cli/        GUI (SDL2 + Dear ImGui + ImPlot) and mmsim-cli
config/robots/            robot JSON configs
docs/                     prd.md, plan.md, decisions.md, hal.md, phase2-neomouse.md
```

Phase 2 (Neomouse V2 on STM32F405): [docs/phase2-neomouse.md](docs/phase2-neomouse.md).
