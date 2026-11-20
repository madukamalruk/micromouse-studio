# mmsim design system (Slice 9.10)

Mode: **Operate**. Desktop tool (SDL2 + Dear ImGui + ImPlot), 1440x900 reference window.
Two audiences: a non-technical viewer who must be able to run and watch a mouse in
10 seconds, and a robotics engineer who tunes, compares and reports. Progressive
disclosure: the Beginner workspace hides nothing an engineer needs, it just starts simple.

Not this: ImGui default blue/grey, the Neomouse V2 web dashboard (olive/lime), neon
"gamer" glow, gradients, glassmorphism, or decorative motion.

## Tokens (`sim/app/theme.hpp`, single source; never hard-code a colour elsewhere)

| Token | Hex | Use |
|---|---|---|
| `canvas` | `#0E1116` | window background, maze floor |
| `panel` | `#151921` | docked windows |
| `raised` | `#1C222D` | controls, hover, inputs |
| `pressed` | `#232B38` | active/pressed controls |
| `hairline` | `#262D3A` | 1 px borders, separators, grid |
| `text` | `#F1F5F9` | primary text, walls |
| `muted` | `#8B96A8` | labels, units, secondary text |
| `accent` | `#38BDF8` | active control, focus, ortho path, selection |
| `accent_dim` | `#38BDF8` @ 22 % | accent fills behind text |
| `good` | `#22C55E` | goal, success, "Goal reached!" |
| `warn` | `#F59E0B` | warnings, diagonal / arc path segments |
| `bad` | `#EF4444` | crash, fault, STOP |

Contrast: `text` on `panel` 15.6:1, `muted` on `panel` 6.0:1, `accent` on `canvas` 8.6:1.
Status colours are never the only signal: every status pill also carries words.

## Type

- UI: **Roboto Medium** (Apache-2.0), 15 px regular, 13 px small labels, 18 px section titles.
- Numbers: **Cousine** (SIL OFL 1.1), 15 px. Every live number (time, speed,
  sensor mm, PWM %) is drawn in mono so digits never shift at 60 FPS. Fixed field
  widths (`%7.2f`), units in `muted`.
- Both fonts come from Dear ImGui's `misc/fonts` and are copied next to the exe at build time; attribution in README (D13).

## Space and shape

- 4 px base grid. Panel padding 12, item spacing 8, inner item spacing 6.
- Rounding: controls 6, panels 8, pills 999 (full), maze 0.
- Borders: 1 px `hairline`; no shadows except the modal scrim (canvas @ 70 %).
- Every button sizes to its label (`CalcTextSize + 2*padding`); no fixed widths that can
  clip. Rows that may not fit wrap to the next line instead of truncating.

## Layout

- **Top bar** (44 px, always visible): maze pill · source (firmware / mms) · status pill ·
  GO / STOP / RESET · speed pills `1x 2x 5x Fast` · sim time · workspace tabs ·
  Beginner/Engineer switch.
- **Maze viewport** gets >= 55 % of the width in Cockpit and Maze Studio.
- Workspaces (keys `1`-`4`, rebuilt with the DockBuilder in one frame):
  1. **Cockpit** (default): maze · robot faceplate · sensor bars · event log.
  2. **Maze Studio**: maze (2D / 2.5D iso) · heatmap + path preview controls ·
     trajectory deck · cell inspector.
  3. **Telemetry & Compare**: plots · run archive A/B with ghost · `dt(s)` / `v(s)` ·
     traction circle.
  4. **Tuning & Reports**: sensor mounts · battery & thermal · step metrics · IMU noise ·
     report / CSV export.
- Beginner mode = Cockpit only, plain-English labels, no raw channel names.

## Status language (plain English first, codes second)

`Ready` · `Exploring maze` · `Returning to start` · `Speed run` · `Goal reached!` ·
`Paused` · `Crashed` · `Firmware fault` · `Solver finished`.
Pill colour: accent (running), good (goal/finished), warn (paused), bad (crash/fault),
muted (ready).

## Motion

- Keyboard and button actions take effect in the same frame. No transition on state.
- Only gauges, bars and toasts ease: 160 ms ease-out (`1 - (1-t)^3`), toasts slide 8 px.
- Nothing animates in scripted / screenshot runs (deterministic captures).

## Maze rendering

Floor `canvas`, posts and walls `text`, unknown walls `hairline`, grid `hairline`
@ 50 %, goal cells `good` @ 18 %, ortho path `accent`, diagonal/arc `warn`, crash
marker `bad`, speed trace: `accent` (slow) to `warn` (fast). Heatmaps use a
sequential `canvas -> accent -> text` ramp with a legend.

## Audio

Two short synthesized cues (goal: rising two-tone; crash: low thud), off when muted
and always off in scripted / headless runs.
