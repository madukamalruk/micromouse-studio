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
    return carve_sized(N, seed, extra_openings, straight_bias=0.0)


def carve_sized(n: int, seed: int, extra_openings: int, straight_bias: float = 0.0):
    """Deterministic maze generator supporting arbitrary even dimensions >= 4."""
    rng = random.Random(seed)
    open_ = set()  # frozenset({(x, y), (nx, ny)}) = passage between two cells

    def link(a, b):
        open_.add(frozenset((a, b)))

    seen = {(0, 0)}
    stack = [((0, 0), None)]
    while stack:
        (x, y), last_d = stack[-1]
        nbrs = []
        for d in "NESW":
            nx, ny = x + DX[d], y + DY[d]
            if (x, y) == (0, 0) and d == "E":
                continue  # classic start cell: walls on W, E and S, the way out is north
            if 0 <= nx < n and 0 <= ny < n and (nx, ny) not in seen:
                nbrs.append((d, (nx, ny)))
        if not nbrs:
            stack.pop()
            continue
        chosen = None
        if straight_bias > 0.0 and last_d is not None and rng.random() < straight_bias:
            for d, pt in nbrs:
                if d == last_d:
                    chosen = (d, pt)
                    break
        if chosen is None:
            chosen = rng.choice(nbrs)
        d, nxt = chosen
        link((x, y), nxt)
        seen.add(nxt)
        stack.append((nxt, d))
    # Goal room: the centre 2x2 block is one open room.
    gx, gy = n // 2 - 1, n // 2 - 1
    block = [(gx, gy), (gx + 1, gy), (gx, gy + 1), (gx + 1, gy + 1)]
    for a in block:
        for b in block:
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1:
                link(a, b)
    # Extra openings (loops), never touching the start cell's side walls.
    tries = 0
    done = 0
    while done < extra_openings and tries < 10000:
        tries += 1
        x, y = rng.randrange(n), rng.randrange(n)
        d = rng.choice("NE")
        nx, ny = x + DX[d], y + DY[d]
        if not (0 <= nx < n and 0 <= ny < n):
            continue
        e = frozenset(((x, y), (nx, ny)))
        if e in open_ or (x, y) == (0, 0) and d == "E":
            continue
        open_.add(e)
        done += 1
    return open_


def carve_boxed_training(box_w: int, box_h: int, seed: int, extra_openings: int, straight_bias: float = 0.25):
    """Carve a smaller [0..box_w) x [0..box_h) arena inside a 16x16 grid enclosing (7,7)-(8,8)."""
    assert box_w >= 9 and box_h >= 9 and box_w <= N and box_h <= N
    rng = random.Random(seed)
    open_ = set()

    def link(a, b):
        open_.add(frozenset((a, b)))

    def in_box(x, y):
        return 0 <= x < box_w and 0 <= y < box_h

    seen = {(0, 0)}
    stack = [((0, 0), None)]
    while stack:
        (x, y), last_d = stack[-1]
        nbrs = []
        for d in "NESW":
            nx, ny = x + DX[d], y + DY[d]
            if (x, y) == (0, 0) and d == "E":
                continue
            if in_box(nx, ny) and (nx, ny) not in seen:
                nbrs.append((d, (nx, ny)))
        if not nbrs:
            stack.pop()
            continue
        chosen = None
        if straight_bias > 0.0 and last_d is not None and rng.random() < straight_bias:
            for d, pt in nbrs:
                if d == last_d:
                    chosen = (d, pt)
                    break
        if chosen is None:
            chosen = rng.choice(nbrs)
        d, nxt = chosen
        link((x, y), nxt)
        seen.add(nxt)
        stack.append((nxt, d))

    # Goal room: classic centre 2x2 block (7,7)-(8,8), enclosed inside the training box.
    block = [(7, 7), (8, 7), (7, 8), (8, 8)]
    for a in block:
        for b in block:
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1:
                link(a, b)

    # Extra loops strictly inside the active training box.
    tries = 0
    done = 0
    while done < extra_openings and tries < 5000:
        tries += 1
        x, y = rng.randrange(box_w), rng.randrange(box_h)
        d = rng.choice("NE")
        nx, ny = x + DX[d], y + DY[d]
        if not in_box(nx, ny):
            continue
        e = frozenset(((x, y), (nx, ny)))
        if e in open_ or (x, y) == (0, 0) and d == "E":
            continue
        open_.add(e)
        done += 1

    # Leave the unused outer region outside the sealed box open (matches upstream training style).
    for x in range(N):
        for y in range(N):
            if in_box(x, y):
                continue
            for d in "NE":
                nx, ny = x + DX[d], y + DY[d]
                if 0 <= nx < N and 0 <= ny < N and not in_box(nx, ny):
                    link((x, y), (nx, ny))
    return open_


def render(open_, n: int = N):
    rows = []
    for r in range(2 * n + 1):
        rows.append([" "] * (4 * n + 1))
    for r in range(0, 2 * n + 1, 2):
        for c in range(0, 4 * n + 1, 4):
            rows[r][c] = "o"
    for x in range(n):
        for y in range(n):
            r0 = 2 * (n - 1 - y)
            c0 = 4 * x
            # north wall
            if y == n - 1 or frozenset(((x, y), (x, y + 1))) not in open_:
                rows[r0][c0 + 1 : c0 + 4] = list("---")
            # south wall
            if y == 0 or frozenset(((x, y), (x, y - 1))) not in open_:
                rows[r0 + 2][c0 + 1 : c0 + 4] = list("---")
            # west wall
            if x == 0 or frozenset(((x, y), (x - 1, y))) not in open_:
                rows[r0 + 1][c0] = "|"
            # east wall
            if x == n - 1 or frozenset(((x, y), (x + 1, y))) not in open_:
                rows[r0 + 1][c0 + 4] = "|"
    rows[2 * (n - 1) + 1][2] = "S"
    return "\n".join("".join(r).rstrip() for r in rows) + "\n"


def main():
    root = Path(__file__).resolve().parent.parent / "mazes"
    test_dir = root / "test"
    test_dir.mkdir(parents=True, exist_ok=True)
    test_specs = [
        ("gen16_perfect_a", 11, 0),
        ("gen16_perfect_b", 23, 0),
        ("gen16_loops_a", 31, 20),
        ("gen16_loops_b", 47, 45),
    ]
    for name, seed, extra in test_specs:
        # Rows are rstripped, so keep the trailing wall characters intact: they are never blank.
        (test_dir / f"{name}.txt").write_text(render(carve(seed, extra), N), encoding="utf-8", newline="\n")
        print("wrote", test_dir / f"{name}.txt")

    # Built-in MIT-licensed library: Classic 16x16, Training, Half-size 32x32
    library_specs = [
        # Classic 16x16 (16 mazes)
        ("classic", "classic_tournament_01", 16, 101, 28, 0.25),
        ("classic", "classic_tournament_02", 16, 102, 34, 0.30),
        ("classic", "classic_tournament_03", 16, 103, 40, 0.20),
        ("classic", "classic_tournament_04", 16, 104, 48, 0.35),
        ("classic", "classic_tournament_05", 16, 105, 55, 0.15),
        ("classic", "classic_tournament_06", 16, 106, 32, 0.40),
        ("classic", "classic_tournament_07", 16, 107, 42, 0.25),
        ("classic", "classic_tournament_08", 16, 108, 60, 0.30),
        ("classic", "classic_perfect_01", 16, 201, 0, 0.20),
        ("classic", "classic_perfect_02", 16, 202, 0, 0.35),
        ("classic", "classic_perfect_03", 16, 203, 0, 0.0),
        ("classic", "classic_perfect_04", 16, 204, 0, 0.45),
        ("classic", "classic_speed_01", 16, 301, 35, 0.70),
        ("classic", "classic_speed_02", 16, 302, 42, 0.75),
        ("classic", "classic_speed_03", 16, 303, 50, 0.80),
        ("classic", "classic_speed_04", 16, 304, 45, 0.65),
        # Half-size 32x32 (4 mazes)
        ("halfsize", "halfsize_32x32_01", 32, 501, 140, 0.35),
        ("halfsize", "halfsize_32x32_02", 32, 502, 180, 0.50),
        ("halfsize", "halfsize_32x32_03", 32, 503, 220, 0.70),
        ("halfsize", "halfsize_32x32_04", 32, 504, 0, 0.30),
    ]
    for folder, name, dim, seed, extra, bias in library_specs:
        d = root / folder
        d.mkdir(parents=True, exist_ok=True)
        txt = render(carve_sized(dim, seed, extra, bias), dim)
        (d / f"{name}.txt").write_text(txt, encoding="utf-8", newline="\n")
        print("wrote", d / f"{name}.txt")

    # Training (6 16x16 boxed arenas enclosing (0,0) and goal (7,7)-(8,8))
    training_specs = [
        ("train_9x9_basic", 9, 9, 411, 0, 0.20),
        ("train_9x9_loops", 9, 9, 412, 12, 0.30),
        ("train_9x9_speed", 9, 9, 413, 16, 0.70),
        ("train_10x10_basic", 10, 10, 414, 0, 0.25),
        ("train_10x10_loops", 10, 10, 415, 18, 0.35),
        ("train_12x12_loops", 12, 12, 416, 26, 0.40),
    ]
    train_dir = root / "training"
    train_dir.mkdir(parents=True, exist_ok=True)
    for name, bw, bh, seed, extra, bias in training_specs:
        txt = render(carve_boxed_training(bw, bh, seed, extra, bias), N)
        (train_dir / f"{name}.txt").write_text(txt, encoding="utf-8", newline="\n")
        print("wrote", train_dir / f"{name}.txt")
    return 0


if __name__ == "__main__":
    sys.exit(main())
