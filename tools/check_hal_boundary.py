#!/usr/bin/env python3
"""Enforce the firmware <-> simulator HAL boundary (AGENTS.md rules 1-3).

Firmware mode (default): every file under the given directories must be C
(.c/.h only), and may include only
  * <mmhal/...>            the HAL contract
  * C standard headers     (C11 freestanding + hosted set)
  * "local.h"              headers inside the same firmware folder (no "..")
HAL mode (--hal): headers may include only C standard headers and other mmhal headers.

Exit 0 = clean, 1 = violations, 2 = usage error. Missing directory = clean (nothing to check).
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

C_STD_HEADERS = {
    "assert.h", "complex.h", "ctype.h", "errno.h", "fenv.h", "float.h", "inttypes.h",
    "iso646.h", "limits.h", "locale.h", "math.h", "setjmp.h", "signal.h", "stdalign.h",
    "stdarg.h", "stdatomic.h", "stdbool.h", "stddef.h", "stdint.h", "stdio.h", "stdlib.h",
    "stdnoreturn.h", "string.h", "tgmath.h", "threads.h", "time.h", "uchar.h", "wchar.h",
    "wctype.h",
}
C_EXTS = {".c", ".h"}
SOURCE_EXTS = {".c", ".h", ".cc", ".cpp", ".cxx", ".hh", ".hpp", ".hxx", ".inl", ".ipp"}
INCLUDE_RE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]')


def check_file(path: Path, root: Path, hal_mode: bool) -> list[str]:
    errors: list[str] = []
    rel = path.relative_to(root.parent)
    if path.suffix.lower() not in C_EXTS:
        return [f"{rel}: C++/non-C source not allowed here (firmware is pure C11)"]
    for lineno, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        m = INCLUDE_RE.match(line)
        if not m:
            continue
        kind, name = m.groups()
        name = name.strip()
        where = f"{rel}:{lineno}: #include {kind}{name}{'>' if kind == '<' else chr(34)}"
        if name in C_STD_HEADERS or name.startswith("mmhal/"):
            continue
        if not hal_mode and kind == '"':
            target = (path.parent / name).resolve()
            if ".." not in Path(name).parts and target.is_relative_to(path.parent.resolve()):
                if target.exists():
                    continue
                errors.append(f"{where}  -> not a header in this firmware folder; "
                              "not allowed across the HAL boundary")
                continue
        errors.append(f"{where}  -> not allowed across the HAL boundary")
    return errors


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+", type=Path)
    ap.add_argument("--hal", action="store_true", help="check HAL headers instead of firmware")
    args = ap.parse_args()

    errors: list[str] = []
    checked = 0
    for d in args.dirs:
        if not d.exists():
            print(f"[boundary] {d}: does not exist yet — nothing to check")
            continue
        if not d.is_dir():
            print(f"[boundary] {d}: not a directory", file=sys.stderr)
            return 2
        for f in sorted(p for p in d.rglob("*") if p.is_file() and p.suffix.lower() in SOURCE_EXTS):
            checked += 1
            errors.extend(check_file(f, d, args.hal))

    for e in errors:
        print(f"[boundary] VIOLATION {e}")
    print(f"[boundary] {checked} file(s) checked, {len(errors)} violation(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
