# Decision log

Format: date · decision · why · alternatives considered.

## D1 — 2026-11-20 · Stack: C++20 sim + C11 firmware, SDL2 + Dear ImGui + ImPlot, MSYS2 UCRT64 at `D:\msys64`
- **Why:** native C/C++ links firmware directly; ImPlot gives fast live plots; GCC matches the arm-none-eabi-gcc used for STM32; MSYS2 pacman provides GCC/CMake/Ninja/SDL2 in one place; D: chosen to save C: space.
- **Installed (2026-11-20):** GCC 16.2.0, CMake 4.5.0, Ninja 1.13.2, GDB 18.1, SDL2 2.32.10, clang-format 23.1.5, Python 3.14.10 (all MSYS2 UCRT64).
- **PATH policy:** persistent system/user PATH is not modified; `cmake/toolchain-msys2-ucrt64.cmake` pins compilers/Ninja under `MSYS2_ROOT` (default `D:/msys64`), so legacy `C:\MinGW` GCC 6.3 is never used.
- **Alternatives:** Qt 6 (heavy, LGPL, slower iteration); raylib (weak UI/plots); Web UI + C++ backend (two codebases); MSVC Build Tools (different compiler family from STM32 GCC).

## D2 — 2026-11-20 · Firmware runs as a DLL in lockstep with sim time
- **Why:** unload/reload gives clean firmware globals on reset and hot-reload after rebuild; lockstep (ISR on sim thread, main loop resumes only at blocking HAL calls) makes runs bit-deterministic.
- **Trade-off:** real ISR-preemption races are not reproduced (documented limitation).
- **Alternatives:** static link into the sim exe (no clean reset of globals); free-running threads (non-deterministic); MCU emulation via QEMU/Renode (out of scope, much slower iteration).

## D3 — 2026-11-20 · Dependencies pinned by archive hash via FetchContent
- doctest 2.5.3, nlohmann/json 3.12.0 (`cmake/deps.cmake`, SHA-256 checked). ImGui (docking) + ImPlot to be added in Slice 2 the same way.
- **Why:** reproducible builds without vendoring; CI and local fetch identical sources.

## D4 — 2026-11-20 · Generic classic mouse for Phase 1; GitHub remote later
- Generic ≈100 g mouse, 4 IR + gyro until Neomouse V2 parameters arrive (Phase 2).
- Repo will be pushed to GitHub later; CI workflow (`.github/workflows/ci.yml`) is ready. Always scan diffs and ask before any `git push`.

## D5 — 2026-11-20 · GUI deps, renderer, visual verification, static runtime
- **Deps (Slice 2):** Dear ImGui 1.93.0-docking, ImPlot 1.1, stb @ `6b5799b` (stb_image_write), SHA-256 pinned; SDL2 2.32.10 from MSYS2. Backend: SDL2 + SDL_Renderer (D3D11), per the ImGui `example_sdl2_sdlrenderer2` of the same version.
- **Visual test = real renderer.** `mmsim --verify-render` pixel-checks every wall slot (present/absent) and post against the maze on the accelerated renderer. SDL's `--software` rasterizer leaves a 1-px seam on the diagonal of thin wall quads and drops some ~3-px post squares, so it is diagnostic only. Probes sample a strip across the wall thickness (robust to seams, still fails on any missing/extra wall).
- **Static C/C++ runtime:** all exes/DLLs link `-static-libgcc -static-libstdc++` + static winpthread. Reason: `C:\MinGW\bin` (GCC 6.3) is on PATH and would otherwise supply an ABI-incompatible `libstdc++-6.dll`. Only `SDL2.dll` is copied next to `mmsim.exe`.
- **Geometry lives in `sim/core/maze_geometry`** (metres; origin = centre of SW post; 180 mm pitch, 12 mm walls) so physics/raycast reuse exactly what is drawn.
- **Amended in Slice 3:** shared libraries (= firmware DLLs, pure C) link only `-static-libgcc`; whole-archive winpthread stays exe-only because a DLL that is `FreeLibrary`'d on every reset should not carry pthread TLS callbacks.
- **Maze text parser follows mms semantics:** any non-space char at a wall position is a wall (`+` or `o` posts); `.maz` = N*N bytes column-major, low nibble only.

## D6 — 2026-11-20 · Firmware main runs on a Win32 fiber; physics model and collision choices (Slice 3)
- **Fiber, not thread.** `fw_main` runs on a fiber (`sim/platform/fiber`); `FirmwareHost` switches to it only at t=0 and when a blocking HAL call's deadline is reached; the SysTick ISR runs on the caller's stack after each 1 ms physics tick. Same lockstep semantics as D2 with a ~1 µs switch instead of 5–20 µs thread hand-offs (needed for the ≥ 50× headless target). Reset = delete the suspended fiber + `FreeLibrary` + load a fresh private copy of the DLL (copy-on-load also allows rebuilding while the sim runs and gives parallel sessions separate globals).
- **Faults:** blocking call inside the ISR → `Faulted` (motors zeroed); `fw_main` returning → `Halted`; a busy-wait polling `hal_millis/micros` yields a tick every 256 reads. A loop with no HAL calls at all cannot be pre-empted: `run_guarded` runs the session on a worker thread and, after a wall-clock timeout (GUI: 2 s), reports `Hung` and abandons that thread + session.
- **Drive model:** DC motor (R, Kt, Ke, gear, rotor/wheel inertia, viscous + Coulomb friction with stiction, efficiency) coupled through the rigid-body mass matrix in wheel-speed coordinates; integrated implicitly in the decoupled sum/difference modes at 0.25 ms, so symmetric inputs give exactly straight lines and exact pivots; pose by exact arc integration. Verified against closed-form solutions (τ within 0.3 %, arc radius 1e-9, zero heading drift over 10 m). No wheel slip yet (Slice 4). Terminal voltage = duty × battery, clamped to `motor.max_voltage` (6 V for N20 on 2S) with a counted warning.
- **Collision:** body = oriented rectangle (exactly what is drawn) vs wall/post AABBs (SAT + minimum translation), uniform-grid broadphase, swept sampling ≤ half a wall thickness per sample + 12-step bisection to the last free pose; response zeroes or reverses (×restitution) wheel speeds; one `CrashEvent` per contact begin. No tunnelling at 20 m/s; penetration ≈ 2e-5 mm at 0.8 m/s.
- **arm-check** compiles each firmware to an object with `-O2 -Werror -Wdouble-promotion` for Cortex-M4F (stricter than the planned `-fsyntax-only`, which skips codegen warnings).
- **Alternatives:** OS threads + condition variables (planned in 3.3; too slow per tick); setjmp/longjmp stacks (non-portable stack switching); explicit RK4 (needs smaller steps for the stiff motor pole, no exact symmetry); polygon body (rectangle suffices until a real chassis outline arrives in Phase 2).
