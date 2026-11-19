# Reference analysis: `mackorone/mms` and `micromouseonline/mazefiles`

Studied 2026-11-20 from shallow clones (not vendored into this repo).

| Repo | Commit | License |
|---|---|---|
| mackorone/mms | `4ed1df48c522` (2025-09-05) | MIT |
| micromouseonline/mazefiles | `070f716d2379` | **none stated** → do not copy files into this repo; fetch locally (see plan Slice 2) |

## 1. mms architecture in one paragraph

Qt 5/6 + OpenGL desktop app (~4.8 kLOC C++11, `src/`). The user configures a **build command** and a **run command**; mms spawns the algorithm as a child `QProcess`, reads newline-terminated commands from its **stdout**, and writes responses to its **stdin** (`Window::dispatchCommand`). The mouse is a polygon that is *teleported/interpolated* between discrete **semi-positions** (half-cell grid, 8 headings) — a `QTimer` advances "movement progress" at a rate set by a speed slider (`Window::updateMouseProgress`). There is no physics, time model, motor, encoder or sensor simulation: wall queries are answered from ground-truth maze data.

## 2. Mouse API (stdin/stdout, text, one command per line)

| Command | Response |
|---|---|
| `mazeWidth`, `mazeHeight` | int |
| `wallFront [N]`, `wallRight [N]`, `wallLeft [N]`, `wallBack [N]` (N = half-steps, default 1) | `true`/`false` |
| `moveForward [N]` (full cells), `moveForwardHalf [N]` | `ack` or `crash` (N<1 or blocked) |
| `turnRight`/`turnRight90`, `turnLeft`/`turnLeft90`, `turnRight45`, `turnLeft45` | `ack` |
| `setWall X Y D`, `clearWall X Y D` (D ∈ n e s w) | none |
| `setColor X Y C`, `clearColor X Y`, `clearAllColor` | none |
| `setText X Y TEXT` (≤10 printable ASCII), `clearText X Y`, `clearAllText` | none |
| `wasReset` | `true`/`false` |
| `ackReset` | `ack` (mouse returned to start) |
| `getStat NAME` | int/float, `-1` if none |

Colors: `k` black, `b` blue, `a` gray, `c` cyan, `g` green, `o` orange, `r` red, `w` white, `y` yellow, `B` dark blue, `C` dark cyan, `A` dark gray, `G` dark green, `R` dark red, `Y` dark yellow. Invalid commands are silently ignored.

Stats: `total-distance`, `total-turns`, `best-run-distance`, `best-run-turns`, `current-run-distance`, `current-run-turns`, `total-effective-distance`, `best-run-effective-distance`, `current-run-effective-distance`, `score`.
Effective distance: in `moveForward(N)`, cells beyond the 2nd count 0.5. `ackReset` adds a 15-point penalty to the next run.
`score = best_run_turns + best_run_eff_dist + 0.1 × (total_turns + total_eff_dist)`; 2000 if goal never reached.

## 3. Maze formats

**Map (text) format** — cell = 5 chars wide × 3 lines tall (the 4-char `+---` pitch with shared posts). *Any non-space character* at a wall position counts as a wall, so mms reads both `+---+` and the mazefiles `o---o` style. mazefiles adds `S` (start) and `G` (goal) markers in cell centres; files use `\n` endings and end with a newline.

**Num format** — one line per cell: `X Y N E S W` (0/1 per wall). *(The README's E/S labels are swapped in prose; the column order is N E S W.)*

**`.maz` binary** — *not supported by mms.* Classic 256-byte format (one byte per cell, column-major from (0,0), bits N=0x01 E=0x02 S=0x04 W=0x08). We add it.

Validation in mms (`Maze::isValid`): nonempty, rectangular, enclosed, consistent (shared walls agree). "Official" checks (reported, not enforced): no unreachable cells, exactly-3-wall start, single centre entrance, hollow centre post, every non-centre post has a wall, not wall-follower solvable.

mazefiles collections: `classic/` 16×16 (goal 7,7 area; some smaller active areas), `halfsize/` 32×32 (single `G` goal cell), `training/` small test mazes. **UK 1987 final = `classic/uk1987f.txt`** (start bottom-left, 2×2 goal centre).

## 4. Rendering rules worth keeping

- Origin (0,0) bottom-left; x east, y north.
- Unknown-but-real walls drawn dimmed (dark red); firmware-declared walls drawn solid (`setWall`).
- Idle state shows distance-to-centre numbers in cells.
- Cell colour + up to 10 chars of text per cell; mouse polygon + wheel polygon.

## 5. What mms lacks (our differentiators)

1. **No continuous physics** — motion is interpolated between discrete semi-positions; no acceleration, inertia, slip, collisions with wall geometry.
2. **No actuator/sensor models** — no motors, voltage, encoders, gyro, ToF/IR raycasts, noise.
3. **No firmware-level execution** — the algorithm talks a text protocol; your real `main()`, control ISR, PID loops and sensor processing are never exercised.
4. **No sim time** — speed slider changes animation rate, not a deterministic time base; nothing is reproducible tick-for-tick.
5. **No telemetry** — no plots of velocity/ω/PID error/sensors, no trajectory trace.
6. **No step/pause-by-tick, no seeded determinism, no headless batch runner** (its own TODO lists a CLI batch runner as a wish, issue #35).
7. Curve turns / diagonal / accelerated moves listed as TODO only.
8. No unit tests (listed in its TODO).

## 6. The micromouseonline concept (target behaviour)

Peter Harrison's real-time sim: *the same mouse source* builds either for the real robot or inside the simulator framework, and neither side knows about the other. The simulator provides the robot's world (time, motors, encoders, sensors); the mouse code only talks to a hardware abstraction. That is our core architecture: **firmware ↔ `mmhal/hal.h` ↔ {sim HAL | STM32 HAL}**.
