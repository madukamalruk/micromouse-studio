#!/usr/bin/env python3
"""Run every demo solver over a set of mazes with mmsim-cli and print a Markdown table.

  python tools/benchmark.py [--cli PATH] [--mazes GLOB ...] [--seed N] [--jobs N]

By default it uses build/release (or build/dev) and mazes/test/gen16_*.txt, plus
external/mazefiles/classic/uk1987f.txt when you have fetched the mazefiles
(tools/fetch_mazes.ps1). Times are simulated seconds on the generic_classic robot:
search = start -> goal, return = goal -> start, run = the final speed run.
"""
import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOLVERS = [
    ("demo_floodfill", "Flood-Fill"),
    ("demo_dijkstra", "Turn-Weighted Dijkstra"),
    ("demo_astar", "Dynamic A*"),
    ("demo_tremaux", "Trémaux"),
    ("demo_wallfollower", "Wall Follower"),
]


def find_cli(explicit):
    if explicit:
        return Path(explicit).resolve()
    for preset in ("release", "dev"):
        for rel in ("sim/cli/mmsim-cli.exe", "sim/cli/mmsim-cli"):
            p = ROOT / "build" / preset / rel
            if p.exists():
                return p
    sys.exit("mmsim-cli not found: build first (cmake --build --preset release) or pass --cli")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cli")
    ap.add_argument("--mazes", nargs="*")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--max-sim-time", default="600")
    a = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    cli = find_cli(a.cli)
    mazes = a.mazes or [str(ROOT / "mazes/test/gen16_*.txt")]
    uk = ROOT / "external/mazefiles/classic/uk1987f.txt"
    if not a.mazes and uk.exists():
        mazes.append(str(uk))

    results = {}  # (solver, maze) -> run dict
    names = []
    for stem, _ in SOLVERS:
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "r.json"
            cmd = [str(cli), "--firmware", stem, "--mazes", *mazes, "--seed", str(a.seed),
                   "--jobs", str(a.jobs), "--max-sim-time", a.max_sim_time, "--quiet",
                   "--json", str(out)]
            proc = subprocess.run(cmd, capture_output=True, text=True)
            if not out.exists():
                sys.exit(f"{stem}: mmsim-cli wrote no report (exit {proc.returncode}): "
                         f"{proc.stdout[-600:]} {proc.stderr[-600:]}")
            for r in json.loads(out.read_text(encoding="utf-8"))["runs"]:
                results[(stem, r["name"])] = r
                if r["name"] not in names:
                    names.append(r["name"])

    names.sort(key=lambda n: (n.startswith("uk"), n))
    print("| Maze | Algorithm | Result | Search | Return | Speed run | Crashes |")
    print("|---|---|---|---:|---:|---:|---:|")
    for n in names:
        for stem, label in SOLVERS:
            r = results.get((stem, n))
            if r is None:
                continue
            if r["solved"]:
                print(f"| {n} | {label} | solved | {r['search_s']:.1f} s | {r['return_s']:.1f} s "
                      f"| **{r['speed_run_s']:.1f} s** | {r['crashes']} |")
            else:
                why = (r["error"] or r["outcome"]).replace("|", "/")
                if len(why) > 48:
                    why = why[:45] + "..."
                print(f"| {n} | {label} | fails: {why} | - | - | - | {r['crashes']} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
