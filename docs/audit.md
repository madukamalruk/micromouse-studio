# Functional and UI/UX audit (Slice 10.2, 2026-11-21)

Method: scripted renders of every workspace and mode at 1280x720 (`--size 1280x720 --screenshot`),
headless runs with `mmsim-cli` and `tools/benchmark.py`, report and CSV inspection, session
files, and the ctest suite (34 tests). Result: **Pass** = checked and fine, **Fixed** = a real
issue found and corrected in this slice, **Known limit** = real, documented, not changed.

| # | Area | Result | Notes |
|---|---|---|---|
| 1 | Beginner mode (Cockpit only, plain-English labels) | Pass | "Lights & buttons", "Motor power", Left/Right sensors, Milestones/Problems only. |
| 2 | Cockpit: faceplate, sensor bars, events, live maze | Pass | All five solvers draw their own cell text and colours on the maze. |
| 3 | Maze Studio: 2D, 2.5D, heatmaps, trajectory deck, cell inspector | Pass | Peak-speed heatmap, route overlay, deck table and inspector render; inspector shows firmware text (cost, step, visits). |
| 4 | Telemetry & Compare: plots, Ghost replay (run A vs archived run B), dt(s), traction circle | Pass | Ghost of an archived Flood-Fill run against a live Dynamic A* run renders; `dt` sign documented in D13. |
| 5 | Tuning & Reports: sensor mounts, battery and thermal, IMU noise, step response | Pass | Live-apply sliders restart the run (documented on screen). |
| 6 | HTML engineering report | Pass | 3 inline SVGs, all parse as XML; no `http(s)` references, no `<script src>`/stylesheet links; no local paths in the file; "Source" shows the solver label. |
| 7 | Report "Goal reached" while a run is still going | **Fixed** | Showed FAIL (and an overall FAIL) for a run that was merely not finished. Now "NOT MEASURED / not yet"; FAIL only after an error, crash or stuck firmware. |
| 8 | Telemetry CSV export | Pass | 60 001 rows for 100 s (the recorder keeps the last 60 s at 1 kHz), header `t_s,sim.v,...`. MATLAB bundle covered by `unit.analysis`. |
| 9 | Firmware menu | **Fixed** | New: human labels, five solvers first, test fixtures under a heading (Engineer only), tooltip with blurb and DLL name. The top bar uses the short name when the long one does not fit. |
| 10 | Non-ASCII label "Trémaux" | **Fixed** | Came out as "TrÃ©maux" (double-encoded in the source). Now written with C++ hex escapes. |
| 11 | Long event lines | **Fixed** | Rows were cut off at the panel edge mid-word. They now end in an ellipsis and show the full text in a tooltip. |
| 12 | Console `fw` command | **Fixed** | Substring match on the file name only, first hit alphabetically. Now an exact stem, else the first match in menu order on stem, label or short label (`fw dijkstra`, `fw flood-fill`, `fw a*`). Unit-tested. |
| 13 | Wall follower on an island goal (UK 1987) | Pass | Status pill "Stuck", plain-English message, red ERROR event, revisited cells orange. 0 crashes. |
| 14 | `mmsim-cli` and `mms` mode across solvers | Pass | All five run through `--firmware NAME` or a short name (`--firmware dijkstra`); `unit.mms` exercises the stdio protocol. JSON, CSV and exit codes unchanged. |
| 15 | Session save and restore (`mmsim.json`, `imgui.ini`) | Pass | Written by an interactive run (not by scripted `--frames` runs, by design); restores firmware, maze, mode, workspace, speed, seed. A saved path that no longer exists is skipped without an error. |
| 16 | Window fit and dark titlebar | Pass | `ui::fit_window` unit-tested at 100/150/200 % and oversize requests (D14). Native titlebar colour set through `platform::apply_dark_titlebar`; not re-checked by eye in this slice. |
| 17 | GO, Stop, Reset, 1x/2x/5x/Fast, Step, hot reload, backtick console, `?` sheet, audio toggle | Pass (by test) | One `ActionRegistry` serves buttons, shortcuts and console; dispatch, history and the shortcut table are unit-tested (`unit.ui`), reload and fault paths in `unit.host`, pacing in `unit.pacer`. |
| 18 | Layout at 1280x720 | Pass | No clipped labels in the top bar or panels; narrow tiers drop the brand and Beginner toggle first. |
| 19 | Mouse-driven popups (firmware and maze combos, ImGui popups) | Known limit | Not driven with synthetic input here; the menu logic is in `ui_logic` and unit-tested, the rendered states were inspected through scripted screenshots. |
| 20 | Wall Follower on mazes with a free-standing goal | Known limit | By design: it stops with an error after one loop (see D15). |
| 21 | Solver parity | Known limit | Flood-Fill, Dijkstra and A* find the same speed-run route on the benchmark mazes; the difference shows on the way back (D15). Trémaux searches far longer on loop-rich mazes. |
| 22 | Windows only | Known limit | The platform layer (titlebar) and the mms child-process code are Win32; other platforms are not built. |
