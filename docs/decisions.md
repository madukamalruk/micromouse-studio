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
