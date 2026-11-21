#!/usr/bin/env python3
"""Generates the MMSim application icon (pure Python, no dependencies).

Obsidian tile (#0E1116) with a Precision Cyan (#38BDF8) route that looks like a maze run and an
"M". Writes:
  assets/mmsim.ico        16, 32, 48 and 256 px (PNG-compressed entries)
  assets/mmsim.png        256 px preview
  sim/app/icon_png.inc    64 px PNG as a C array for SDL_SetWindowIcon

Run from the repository root:  python tools/gen_icon.py
"""
import math
import os
import struct
import zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

OBSIDIAN = (14, 17, 22)
PANEL = (28, 34, 45)
HAIRLINE = (38, 45, 58)
CYAN = (56, 189, 248)
CYAN_LIGHT = (186, 230, 253)

# The route: a stepped "M" in unit coordinates (0..1), like a mouse running a maze.
ROUTE = [(0.22, 0.78), (0.22, 0.26), (0.50, 0.56), (0.78, 0.26), (0.78, 0.78)]
SS = 4  # supersampling per axis


def sd_round_rect(px, py, cx, cy, hw, hh, r):
    qx = abs(px - cx) - (hw - r)
    qy = abs(py - cy) - (hh - r)
    return math.hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0) - r


def sd_segment(px, py, ax, ay, bx, by):
    pax, pay = px - ax, py - ay
    bax, bay = bx - ax, by - ay
    h = max(0.0, min(1.0, (pax * bax + pay * bay) / (bax * bax + bay * bay)))
    return math.hypot(pax - bax * h, pay - bay * h)


def blend(dst, src, a):
    return tuple(d + (s - d) * a for d, s in zip(dst, src))


def render(size):
    """RGBA bytes for one icon size."""
    out = bytearray()
    stroke = max(0.055 * size, 1.4 if size <= 16 else 1.0)  # half-width of the route
    pad = 0.04 * size
    half = size / 2.0
    radius = 0.22 * size
    pts = [(x * size, y * size) for x, y in ROUTE]
    end = pts[-1]
    for y in range(size):
        for x in range(size):
            acc = [0.0, 0.0, 0.0, 0.0]
            for sy in range(SS):
                for sx in range(SS):
                    px = x + (sx + 0.5) / SS
                    py = y + (sy + 0.5) / SS
                    d_tile = sd_round_rect(px, py, half, half, half - pad, half - pad, radius)
                    if d_tile > 0.5 / SS:
                        continue  # transparent corner
                    col = OBSIDIAN
                    # soft top-left light
                    t = max(0.0, 1.0 - math.hypot(px - 0.3 * size, py - 0.2 * size) / (0.9 * size))
                    col = blend(col, PANEL, 0.55 * t)
                    # hairline rim
                    if d_tile > -max(1.0, 0.025 * size):
                        col = blend(col, HAIRLINE, 0.9)
                    # faint maze grid on larger sizes
                    if size >= 48:
                        step = size / 4.0
                        gx = abs(((px - pad) % step) - step / 2.0)
                        gy = abs(((py - pad) % step) - step / 2.0)
                        if min(step / 2.0 - gx, step / 2.0 - gy) < 0.006 * size:
                            col = blend(col, HAIRLINE, 0.45)
                    # route
                    d_route = min(
                        sd_segment(px, py, a[0], a[1], b[0], b[1]) for a, b in zip(pts, pts[1:])
                    )
                    cov = max(0.0, min(1.0, (stroke - d_route) * SS * 0.5 + 0.5))
                    if cov > 0.0:
                        col = blend(col, CYAN, cov)
                    # the mouse: a bright dot at the end of the run
                    d_dot = math.hypot(px - end[0], py - end[1])
                    cov_dot = max(0.0, min(1.0, (stroke * 1.45 - d_dot) * SS * 0.5 + 0.5))
                    if cov_dot > 0.0:
                        col = blend(col, CYAN_LIGHT, cov_dot)
                    acc[0] += col[0]
                    acc[1] += col[1]
                    acc[2] += col[2]
                    acc[3] += 1.0
            n = SS * SS
            a = acc[3] / n
            if acc[3] > 0:
                out += bytes((int(acc[0] / acc[3] + 0.5), int(acc[1] / acc[3] + 0.5),
                              int(acc[2] / acc[3] + 0.5), int(a * 255 + 0.5)))
            else:
                out += b"\x00\x00\x00\x00"
    return bytes(out)


def png(size, rgba):
    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + rgba[y * size * 4:(y + 1) * size * 4] for y in range(size))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def ico(entries):
    """entries: [(size, png_bytes)] -> .ico bytes (PNG-compressed images)."""
    head = struct.pack("<HHH", 0, 1, len(entries))
    offset = 6 + 16 * len(entries)
    directory, blobs = b"", b""
    for size, data in entries:
        directory += struct.pack("<BBBBHHII", size % 256, size % 256, 0, 0, 1, 32, len(data), offset)
        blobs += data
        offset += len(data)
    return head + directory + blobs


def main():
    sizes = [16, 32, 48, 256]
    pngs = {s: png(s, render(s)) for s in sizes}
    assets = os.path.join(ROOT, "assets")
    os.makedirs(assets, exist_ok=True)
    with open(os.path.join(assets, "mmsim.ico"), "wb") as f:
        f.write(ico([(s, pngs[s]) for s in sizes]))
    with open(os.path.join(assets, "mmsim.png"), "wb") as f:
        f.write(pngs[256])
    p64 = png(64, render(64))
    lines = ["// Generated by tools/gen_icon.py: the 64 px window icon (PNG). Do not edit.",
             "static const unsigned char kIconPng64[] = {"]
    for i in range(0, len(p64), 16):
        lines.append("    " + ", ".join("0x%02x" % b for b in p64[i:i + 16]) + ",")
    lines.append("};")
    with open(os.path.join(ROOT, "sim", "app", "icon_png.inc"), "w", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print("wrote assets/mmsim.ico (%s), assets/mmsim.png, sim/app/icon_png.inc (%d bytes)"
          % (", ".join(str(s) for s in sizes), len(p64)))


if __name__ == "__main__":
    main()
