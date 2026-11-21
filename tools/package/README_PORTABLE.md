# MMSim (portable Windows build)

A real-time software-in-the-loop micromouse simulator. Your robot firmware is plain C, built as
a DLL against a small hardware layer; the simulator runs it in lockstep with the physics, so a
run is repeatable bit for bit.

No installer and no extra runtime: unzip anywhere and start it. Windows 10 or 11, 64-bit.

## Start

Double-click **`Launch_MMSim.bat`**. It opens a bundled maze with the **Flood-Fill** solver.
Press **GO** (or Space). Pick another solver in the *Firmware* menu and another maze in the
*Maze* menu. Press **?** for the shortcut sheet, **`** (backtick) for the command console.

Beginner mode shows one simple screen; Engineer mode adds four workspaces (keys 1 to 4).

## Solvers included (`firmware_bin/`)

| Menu name | What it does |
|---|---|
| Flood-Fill | Classic distance map, always steps downhill. |
| Turn-Weighted Dijkstra | Prices turns, prefers straighter routes. |
| Dynamic A* | One optimal plan, replanned only when a new wall blocks it. |
| Trémaux | Chalk marks on passages; copes with loops. |
| Wall Follower | Left hand on the wall. Fails when the goal is an island, and says so. |

## Command line

```bat
mmsim-cli.exe --firmware dijkstra --mazes "mazes\test\*.txt" --csv results.csv
mmsim.exe --maze mazes\test\gen16_loops_a.txt --firmware astar
```

`mmsim-cli.exe --help` and `mmsim.exe --help` list every option.

## More mazes

This package only carries a few original test mazes (`mazes/test/`). The large classic maze
collection is not redistributed here: get it with `tools/fetch_mazes.ps1` from the source
repository (https://github.com/madukamalruk/micromouse-studio) and read the licence notes of that
collection first. Unpack it so that `external/mazefiles/classic/` sits next to `mmsim.exe`.

## Your own firmware

Write C against `<mmhal/hal.h>` (see `docs/hal.md` in the repository), build a DLL, and put it
in `firmware_bin/`. It then shows up in the Firmware menu (Ctrl+R rescans).

## Files

`mmsim.exe` (window), `mmsim-cli.exe` (headless batch runner), `SDL2.dll`, `fonts/`,
`firmware_bin/`, `config/robots/generic_classic.json`, `mazes/test/`. The app writes
`mmsim.json`, `imgui.ini`, `runs/` and `reports/` next to itself; delete them any time.

## Licence

MIT, see `LICENSE`. Third-party components and fonts: `THIRD_PARTY_NOTICES.md` and `licenses/`.
