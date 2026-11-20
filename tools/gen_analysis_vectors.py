"""Generates tests/fixtures/analysis/*.json: shared golden vectors for sim/core/analysis.

The reference maths is the Neomouse V2 Mission Control code (read-only, MIT):
    tools/dashboard_analysis.py, tools/thermal_battery_sim.py
Run once (outputs are committed):
    python -I tools/gen_analysis_vectors.py "<path to Micromouse-NeomouseV2/tools>"
Mazes are generated here (own seeded generator), so no mazefiles content is copied.
"""
import json
import math
import os
import random
import sys

if len(sys.argv) != 2:
    sys.exit(__doc__)
sys.path.insert(0, os.path.abspath(sys.argv[1]))
import dashboard_analysis as da  # noqa: E402
import thermal_battery_sim as tb  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tests", "fixtures", "analysis")
os.makedirs(OUT, exist_ok=True)


def dump(name, obj):
    with open(os.path.join(OUT, name), "w", encoding="utf-8", newline="\n") as f:
        json.dump(obj, f, indent=1, sort_keys=True)
        f.write("\n")
    print("wrote", name)


def err(fn, *a, **k):
    try:
        return fn(*a, **k)
    except ValueError as e:
        return {"raises": str(e)}


# ------------------------------------------------------------------ mazes (16x16, N=1 E=2 S=4 W=8)
def gen_maze(seed, braid):
    rnd = random.Random(seed)
    w = [[15] * 16 for _ in range(16)]
    seen = [[False] * 16 for _ in range(16)]

    def knock(x, y, d):
        nx, ny = x + da.DXY[d][0], y + da.DXY[d][1]
        w[y][x] &= ~da.BITS[d]
        w[ny][nx] &= ~da.BITS[(d + 2) % 4]

    stack = [(0, 0)]
    seen[0][0] = True
    while stack:
        x, y = stack[-1]
        opts = [d for d in range(4) if 0 <= x + da.DXY[d][0] < 16 and 0 <= y + da.DXY[d][1] < 16
                and not seen[y + da.DXY[d][1]][x + da.DXY[d][0]]]
        if not opts:
            stack.pop()
            continue
        d = rnd.choice(opts)
        knock(x, y, d)
        nx, ny = x + da.DXY[d][0], y + da.DXY[d][1]
        seen[ny][nx] = True
        stack.append((nx, ny))
    # open the goal box interior, then braid (remove random interior walls)
    knock(7, 7, 0); knock(7, 7, 1); knock(8, 8, 2); knock(8, 8, 3)
    for _ in range(braid):
        x, y, d = rnd.randrange(16), rnd.randrange(16), rnd.randrange(4)
        nx, ny = x + da.DXY[d][0], y + da.DXY[d][1]
        if 0 <= nx < 16 and 0 <= ny < 16:
            knock(x, y, d)
    return w


def known_mask(seed, frac):
    rnd = random.Random(seed)
    k = [[0] * 16 for _ in range(16)]
    for y in range(16):
        for x in range(16):
            for d in range(4):
                if rnd.random() < frac:
                    k[y][x] |= da.BITS[d]
    return k


mazes = []
for seed, braid, frac in ((11, 0, None), (12, 40, 0.6), (13, 90, 0.85)):
    walls = gen_maze(seed, braid)
    known = None if frac is None else known_mask(seed + 100, frac)
    sp = da.shortest_path(walls, known)
    sp_tc0 = da.shortest_path(walls, known, turn_cost=0.0)
    pf = da.potential_field(walls, known)
    df = da.distance_field(walls, known)
    df.pop("potential")
    traj = {}
    if sp:
        for diag in (True, False):
            traj["diag" if diag else "ortho"] = err(da.compile_trajectory, sp["cells"], diagonal=diag)
        traj["fast"] = err(da.compile_trajectory, sp["cells"], run_mms=2000.0, accel_mms2=8000.0, radius_mm=60.0, mu=1.1)
    mazes.append({"seed": seed, "walls": walls, "known": known, "shortest_path": sp, "shortest_path_tc0": sp_tc0,
                  "potential_field": pf, "distance_field": df, "trajectory": traj})

routes = {
    "zigzag": [[0, 0], [0, 1], [1, 1], [1, 2], [2, 2], [2, 3], [3, 3], [3, 4], [3, 5], [3, 6]],
    "l_turn": [[0, 0], [0, 1], [0, 2], [0, 3], [1, 3], [2, 3], [3, 3]],
    "straight": [[0, 0], [0, 1], [0, 2], [0, 3], [0, 4]],
    "s_bend": [[0, 0], [0, 1], [1, 1], [2, 1], [2, 2], [2, 3]],
}
route_out = {k: {"cells": v, "diag": err(da.compile_trajectory, v), "ortho": err(da.compile_trajectory, v, diagonal=False)} for k, v in routes.items()}
route_out["uturn"] = {"cells": [[0, 0], [0, 1], [0, 0]], "diag": err(da.compile_trajectory, [[0, 0], [0, 1], [0, 0]])}
route_out["jump"] = {"cells": [[0, 0], [0, 2]], "diag": err(da.compile_trajectory, [[0, 0], [0, 2]])}
dump("maze_paths.json", {"mazes": mazes, "routes": route_out})

# ------------------------------------------------------------------ step response
def first_order(tau, amp, t0, n=400, dt=0.002, noise=0.0, seed=1):
    rnd = random.Random(seed)
    t = [i * dt for i in range(n)]
    y = [(amp * (1 - math.exp(-(ti - t0) / tau)) if ti >= t0 else 0.0) + rnd.gauss(0, noise) for ti in t]
    return t, y


def second_order(wn, zeta, amp, t0, n=500, dt=0.002, fall_at=None):
    t = [i * dt for i in range(n)]
    wd = wn * math.sqrt(1 - zeta * zeta)
    phi = math.acos(zeta)
    y = []
    for ti in t:
        if ti < t0:
            y.append(0.0)
            continue
        tt = ti - t0
        v = amp * (1 - math.exp(-zeta * wn * tt) / math.sqrt(1 - zeta * zeta) * math.sin(wd * tt + phi))
        if fall_at is not None and ti >= fall_at:
            v = 0.0
        y.append(v)
    return t, y


steps = []
for name, (t, y) in {
    "first_order": first_order(0.05, 2000.0, 0.1),
    "first_order_noisy": first_order(0.03, 1500.0, 0.05, noise=8.0),
    "second_order_underdamped": second_order(40.0, 0.3, 1800.0, 0.08),
    "second_order_damped": second_order(30.0, 0.8, 1200.0, 0.1),
    "negative": tuple(map(list, (first_order(0.04, 1000.0, 0.1)[0], [-v for v in first_order(0.04, 1000.0, 0.1)[1]]))),
    "falls_back": second_order(40.0, 0.5, 1500.0, 0.05, fall_at=0.7),
    "too_small": first_order(0.05, 50.0, 0.1),
    "few": ([0.0, 0.1], [0.0, 1.0]),
}.items():
    steps.append({"name": name, "t": t, "y": y, "out": da.step_metrics(t, y)})
dump("step_metrics.json", {"cases": steps})

# ------------------------------------------------------------------ scalar helpers
rnd = random.Random(5)
imu = [[], [0.3], [0.1, -0.1, 0.2, 0.05], [rnd.gauss(0.2, 0.15) for _ in range(500)]]
corner = [{"args": a, "out": err(da.cornering, *a)} for a in
          ((0.0, 90.0, 0.8), (500.0, 90.0, 0.8), (750.0, 90.0, 0.8), (840.0, 90.0, 0.8), (900.0, 90.0, 0.8),
           (1200.0, 60.0, 1.2), (-1.0, 90.0, 0.8), (500.0, 0.0, 0.8))]
health = [{"args": a, "out": err(da.battery_health, *a)} for a in
          ((8200, 7900, 2.0, None, 6000), (8000, 7400, 2.0, 3.0, 6000), (7600, 7500, 1.0, 2.6, 6500), (8000, 8100, 1.0, None, 6000), (8000, 7800, 0.0, None, 6000))]
dump("scalars.json", {
    "imu_stats": [{"samples": s, "out": da.imu_stats(s)} for s in imu],
    "cornering": corner,
    "battery_health": health,
    "fault_info": [da.fault_info(c) for c in list(range(12)) + [42]],
})

# ------------------------------------------------------------------ battery / thermal / chassis
prof = tb.mission_profile()
br = tb.battery_run(prof)
jt = tb.junction_temps(br)
small = tb.mission_profile(explore_s=6.0, run_s=2.0, runs=1, idle_between_s=1.0, dt=0.05)
brs = tb.battery_run(small, cap_mah=300.0, r_int=0.2, soc0=0.9)
dump("battery.json", {
    "ocv": [[s, tb.ocv(s)] for s in (-0.1, 0.0, 0.025, 0.05, 0.15, 0.3, 0.5, 0.7, 0.9, 1.0, 1.2)],
    "mission": {"n": len(prof), "end_soc": br["end_soc"], "min_v": br["min_v"], "mah_used": br["mah_used"], "margin_to_6v0": br["margin_to_6v0"]},
    "junction": jt,
    "small_profile": [list(p) for p in small],
    "small_run": {"rows": [list(r) for r in brs["rows"]], "end_soc": brs["end_soc"], "min_v": brs["min_v"], "mah_used": brs["mah_used"]},
    "loads": tb.LOADS,
    "powers": [{"v": v, "i": i, "out": tb.powers(v, i)} for v, i in ((8.4, 1.6), (7.4, 0.0), (6.5, 2.6))],
    "axle_loads": [{"a": a, "out": tb.axle_loads(a)} for a in (0.0, 5000.0, -8000.0)],
    "pitch_limits": tb.pitch_limits(),
    "chassis": tb.CHASSIS,
})

# ------------------------------------------------------------------ runs, heatmap, events
def run_rows(profile_v, seg=1, dt_ms=2, gyro=None, jitter=0.0, seed=3):
    rnd = random.Random(seed)
    rows = []
    for k, v in enumerate(profile_v):
        g = gyro(k) if gyro else 0.0
        rows.append({"segment": seg, "t_ms": k * dt_ms, "wheel_l_mms": v + rnd.gauss(0, jitter), "wheel_r_mms": v + rnd.gauss(0, jitter),
                     "wheel_l_cps": (v - g) * 4.0, "wheel_r_cps": (v + g) * 4.0, "gyro_dps": g})
    return rows


def trapezoid(vmax, acc, length_mm, dt=0.002, idle=40):
    out = [0.0] * idle
    v, s = 0.0, 0.0
    while s < length_mm:
        brake = v * v / (2 * acc)
        if length_mm - s <= brake:
            v = max(0.0, v - acc * dt)
            if v <= 1e-6:
                break
        else:
            v = min(vmax, v + acc * dt)
        s += v * dt
        out.append(v)
    return out + [0.0] * idle


ra = run_rows(trapezoid(800.0, 3000.0, 2000.0))
rb = run_rows(trapezoid(950.0, 4000.0, 2100.0), jitter=3.0)
cmp_ab = da.compare_runs(ra, rb)
cmp_aa = da.compare_runs(ra, ra)
cmp_short = da.compare_runs(run_rows(trapezoid(300.0, 3000.0, 15.0)), ra)
dump("compare_runs.json", {"a": ra, "b": rb, "ab": cmp_ab, "aa": cmp_aa, "short": cmp_short,
                           "series_a": da.run_series(ra)})

curve = run_rows(trapezoid(600.0, 2500.0, 1500.0), gyro=lambda k: 120.0 * math.sin(k / 60.0))
for r in curve[:30]:
    r["gyro_dps"] += 0.4        # stationary bias the heatmap removes
dump("heatmap.json", {"rows": curve, "counts_per_mm": 4.0, "out": da.motion_heatmap(curve, 4.0)})

ev_rows = []
states = ["ARMED_IDLE"] * 5 + ["RUN"] * 6 + ["FAULT"] * 3
for k, st in enumerate(states):
    ev_rows.append({"segment": 0 if k < 9 else 1, "t_ms": k * 100, "state": da.STATES.index(st), "state_name": st,
                    "fault": 8 if st == "FAULT" else 0, "fault_name": "WHEEL STALL" if st == "FAULT" else "NONE",
                    "bias_valid": k >= 2, "slip": k in (7, 8), "degraded": False})
dump("events.json", {"rows": ev_rows, "out": da.telemetry_events(ev_rows), "limited": da.telemetry_events(ev_rows, limit=3)})
