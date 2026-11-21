#!/usr/bin/env python3
"""Generate the small, original 16x16 test mazes that ship in mazes/test/ (MIT, like the rest
of this repository). Deterministic: the same seed always writes the same file.

  python tools/gen_mazes.py            # rewrites mazes/test/gen16_*.txt

Layout: the classic text format (o---o / | |), start S at the bottom-left cell, goal = the
central 2x2 block, opened into one room. 'perfect' mazes are a spanning tree (every wall
touches the outer wall, so even a left-hand wall follower solves them); 'loops' mazes also
knock out extra walls, like real contest mazes (the wall follower can circle in those).
"""
import random
import sys
from pathlib import Path

N = 16
DX = {"N": 0, "E": 1, "S": 0, "W": -1}
DY = {"N": 1, "E": 0, "S": -1, "W": 0}
OPP = {"N": "S", "S": "N", "E": "W", "W": "E"}


def carve(seed: int, extra_openings: int):
    rng = random.Random(seed)
    open_ = set()  # frozenset({(x, y), (nx, ny)}) = passage between two cells

    def link(a, b):
        open_.add(frozenset((a, b)))

    seen = {(0, 0)}
    stack = [(0, 0)]
    while stack:
        x, y = stack[-1]
        nbrs = []
        for d in "NESW":
            nx, ny = x + DX[d], y + DY[d]
            if (x, y) == (0, 0) and d == "E":
                continue  # classic start cell: walls on W, E and S, the way out is north
            if 0 <= nx < N and 0 <= ny < N and (nx, ny) not in seen:
                nbrs.append((nx, ny))
        if not nbrs:
            stack.pop()
            continue
        n = rng.choice(nbrs)
        link((x, y), n)
        seen.add(n)
        stack.append(n)
    # Goal room: the centre 2x2 block is one open room.
    block = [(7, 7), (8, 7), (7, 8), (8, 8)]
    for a in block:
        for b in block:
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1:
                link(a, b)
    # Extra openings (loops), never touching the start cell's side walls.
    tries = 0
    done = 0
    while done < extra_openings and tries < 5000:
        tries += 1
        x, y = rng.randrange(N), rng.randrange(N)
        d = rng.choice("NE")
        nx, ny = x + DX[d], y + DY[d]
        if not (0 <= nx < N and 0 <= ny < N):
            continue
        e = frozenset(((x, y), (nx, ny)))
        if e in open_ or (x, y) == (0, 0) and d == "E":
            continue
        open_.add(e)
        done += 1
    return open_


def render(open_):
    rows = []
    for r in range(2 * N + 1):
        rows.append([" "] * (4 * N + 1))
    for r in range(0, 2 * N + 1, 2):
        for c in range(0, 4 * N + 1, 4):
            rows[r][c] = "o"
    for x in range(N):
        for y in range(N):
            r0 = 2 * (N - 1 - y)
            c0 = 4 * x
            # north wall
            if y == N - 1 or frozenset(((x, y), (x, y + 1))) not in open_:
                rows[r0][c0 + 1 : c0 + 4] = list("---")
            # south wall
            if y == 0 or frozenset(((x, y), (x, y - 1))) not in open_:
                rows[r0 + 2][c0 + 1 : c0 + 4] = list("---")
            # west wall
            if x == 0 or frozenset(((x, y), (x - 1, y))) not in open_:
                rows[r0 + 1][c0] = "|"
            # east wall
            if x == N - 1 or frozenset(((x, y), (x + 1, y))) not in open_:
                rows[r0 + 1][c0 + 4] = "|"
    rows[2 * (N - 1) + 1][2] = "S"
    return "\n".join("".join(r).rstrip() for r in rows) + "\n"


def main():
    out = Path(__file__).resolve().parent.parent / "mazes" / "test"
    out.mkdir(parents=True, exist_ok=True)
    specs = [
        ("gen16_perfect_a", 11, 0),
        ("gen16_perfect_b", 23, 0),
        ("gen16_loops_a", 31, 20),
        ("gen16_loops_b", 47, 45),
    ]
    for name, seed, extra in specs:
        # Rows are rstripped, so keep the trailing wall characters intact: they are never blank.
        (out / f"{name}.txt").write_text(render(carve(seed, extra)), encoding="utf-8", newline="\n")
        print("wrote", out / f"{name}.txt")
    return 0


if __name__ == "__main__":
    sys.exit(main())
