# firmware/

One folder per firmware (`firmware/<name>/`), pure C11. Each folder with a `main.c` builds to
`<name>.dll` for the simulator and every `.c` must also compile for Cortex-M4F (`arm-check`,
`-O2 -Werror -Wdouble-promotion`).

Allowed includes: `<mmhal/...>`, C standard headers, headers in the same folder, and
`"common/<file>.h"`. Enforced by `tools/check_hal_boundary.py` (ctest label `boundary`). See
AGENTS.md, "HAL boundary rules".

## The solver suite

All five `demo_*` firmwares share one motion, control and sensor stack in `common/` (1 kHz ISR
odometry, trapezoidal profiles, PD plus feed-forward, wall centring, the 6.0 V motor cap and the
20 ms reversal coast). A solver is only a `solver_t` (name, description, `init`, `next`) plus
a two-line `fw_main`; `solver_main` runs the mission: calibrate, search to the goal, return,
speed run.

| Folder | Menu label | Idea |
|---|---|---|
| `demo_floodfill` | Flood-Fill | Breadth-first distance map, step downhill. |
| `demo_dijkstra` | Turn-Weighted Dijkstra | Cost 2 per cell plus 5 per 90 degree turn, over (cell, heading) states. |
| `demo_astar` | Dynamic A* | One optimal plan, replanned only when a new wall blocks it. |
| `demo_tremaux` | Trémaux | Chalk marks on passages; copes with loops. Then the shortest way back. |
| `demo_wallfollower` | Wall Follower | Left hand on the wall. Fails (and says so) when the goal is an island. |

`common/plan.c` holds the planners, `common/maze.c` the map (with a version counter used as a
cache key). Cell text is at most 10 characters; colours use the mms palette
(`kbacgorwyBACGORWY`). Each solver also publishes telemetry (`cells`, `phase`, and its own
channels such as `rebuilds`, `replans`, `marks`, `revisits`).

Required log strings (the tests and the UI read them): `algorithm:`, `goal reached`,
`start reached`, `speed run start`, `speed run done`, `ERROR ...`.

## Adding a solver

1. Create `firmware/demo_<name>/main.c` with a `solver_t` and
   `void fw_main(void) { solver_main(&g_solver); }`.
2. `next(target, mode, x, y, heading)` returns the direction to leave the cell. In
   `FLOOD_SEARCH` it may keep state (called once per cell entered). In `FLOOD_KNOWN` it must be
   pure: the speed run looks ahead with it.
3. Add a label in `firmware_info()` (`sim/app/ui_logic.cpp`) and a test case in
   `tests/unit/test_solvers.cpp`. CMake, arm-check and the boundary check pick it up.

`test_*` folders are development fixtures (fault injection and the like). The GUI lists them
under "Test fixtures" in Engineer mode only.
