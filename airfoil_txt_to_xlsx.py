#!/usr/bin/env python3
"""
airfoil_txt_to_xlsx.py

Converts an airfoil coordinate TXT file (two columns: x y) into an XLSX file,
scaled to a requested chord length in mm, and optionally reordered into a
single continuous closed-ish loop suitable for Inventor spline work.

Typical input format (like NACA0012.txt):
- Upper surface block: x from 0..1, y positive
- Blank line
- Lower surface block: x from 0..1, y negative

Outputs:
- XLSX with columns: X_mm, Y_mm

Usage examples:
  python airfoil_txt_to_xlsx.py ./airfoils/NACA0012.txt --chord-mm 100 --out NACA0012_100mm.xlsx
  python airfoil_txt_to_xlsx.py ./airfoils/NACA0012.txt --chord-mm 150 --template ./airfoils.xlsx --sheet Points
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import List, Tuple, Optional

from openpyxl import Workbook, load_workbook


Point = Tuple[float, float]


def parse_airfoil_txt(path: Path) -> Tuple[List[Point], List[Point]]:
    """
    Parse airfoil txt with 2 columns (x y). Supports:
    - Two blocks separated by blank line (upper, lower)
    - Or single block (treated as one list; lower will be empty)
    Ignores non-numeric lines safely.
    """
    text = path.read_text(encoding="utf-8", errors="ignore").splitlines()

    blocks: List[List[Point]] = [[]]
    for line in text:
        s = line.strip()
        if not s:
            # start a new block only if current block already has points
            if blocks[-1]:
                blocks.append([])
            continue

        # Split by whitespace or comma, allow things like "-.0042603"
        parts = re.split(r"[,\s]+", s)
        if len(parts) < 2:
            continue

        try:
            x = float(parts[0])
            y = float(parts[1])
        except ValueError:
            continue

        blocks[-1].append((x, y))

    # Remove empty blocks
    blocks = [b for b in blocks if b]

    if not blocks:
        raise ValueError(f"No numeric point data found in: {path}")

    if len(blocks) == 1:
        upper = blocks[0]
        lower: List[Point] = []
    else:
        upper = blocks[0]
        lower = blocks[1]

    return upper, lower


def chord_scale_factor(upper: List[Point], lower: List[Point], chord_mm: float) -> float:
    """
    Determine scale factor to reach chord_mm.

    If x is normalized (max around 1.0), we scale by chord_mm / max_x.
    If x is already in mm (max around 1000 etc.), still scale by chord_mm / max_x.
    This makes the script robust to either case.
    """
    xs = [p[0] for p in upper] + [p[0] for p in lower]
    max_x = max(xs)
    min_x = min(xs)
    current_chord = max_x - min_x
    if current_chord <= 0:
        raise ValueError("Invalid chord (max_x - min_x <= 0). Check input data.")

    return chord_mm / current_chord


def shift_to_le_zero(points: List[Point]) -> List[Point]:
    """
    Shift x so that leading edge is at x=0 (min x becomes 0).
    """
    min_x = min(x for x, _ in points)
    return [(x - min_x, y) for x, y in points]


def make_continuous_loop(upper: List[Point], lower: List[Point]) -> List[Point]:
    """
    Create a single ordered point list:
      TE -> upper -> LE -> lower -> TE

    Given typical data:
      upper: LE(0) -> TE(1)
      lower: LE(0) -> TE(1)

    We want:
      upper reversed: TE -> LE
      then lower forward: LE -> TE (skip first LE to avoid duplicate)
    """
    if not lower:
        # If only one block, return as-is (user can decide)
        return upper

    upper_rev = list(reversed(upper))
    # Avoid duplicating the LE point if both start at LE
    lower_fwd = lower[1:] if lower and _is_same_point(lower[0], upper[0]) else lower
    return upper_rev + lower_fwd


def _is_same_point(a: Point, b: Point, eps: float = 1e-9) -> bool:
    return (abs(a[0] - b[0]) < eps) and (abs(a[1] - b[1]) < eps)


def scale_points(points: List[Point], scale: float) -> List[Point]:
    return [(x * scale, y * scale) for x, y in points]


def write_xlsx(
    points_mm: List[Point],
    out_path: Path,
    template_path: Optional[Path] = None,
    sheet_name: str = "Points",
    start_row: int = 1,
    start_col: int = 1,
    write_header: bool = True,
) -> None:
    """
    Write points to XLSX.

    If template_path is provided, it loads that workbook and writes into sheet_name.
    Otherwise creates a new workbook.
    """
    if template_path:
        wb = load_workbook(template_path)
        ws = wb[sheet_name] if sheet_name in wb.sheetnames else wb.create_sheet(sheet_name)
    else:
        wb = Workbook()
        ws = wb.active
        ws.title = sheet_name

    r = start_row
    c = start_col

    if write_header:
        ws.cell(row=r, column=c, value="X_mm")
        ws.cell(row=r, column=c + 1, value="Y_mm")
        r += 1

    for x, y in points_mm:
        ws.cell(row=r, column=c, value=float(x))
        ws.cell(row=r, column=c + 1, value=float(y))
        r += 1

    wb.save(out_path)


def main() -> None:
    ap = argparse.ArgumentParser(description="Convert airfoil TXT points to scaled XLSX for Inventor.")
    ap.add_argument("txt_path", type=Path, help="Input .txt file with airfoil points (x y).")
    ap.add_argument("--chord-mm", type=float, required=True, help="Desired chord length in mm (e.g. 100).")
    ap.add_argument("--out", type=Path, default=None, help="Output .xlsx path (default: <input>_<chord>mm.xlsx).")

    ap.add_argument("--template", type=Path, default=None, help="Optional template .xlsx to write into.")
    ap.add_argument("--sheet", type=str, default="Points", help="Sheet name to write into (default: Points).")
    ap.add_argument("--start-row", type=int, default=1, help="Start row for writing (default: 1).")
    ap.add_argument("--start-col", type=int, default=1, help="Start col for writing (default: 1).")
    ap.add_argument("--no-header", action="store_true", help="Do not write X_mm/Y_mm header row.")

    ap.add_argument("--no-loop", action="store_true",
                    help="Do not reorder into a continuous loop; keep original block order.")
    ap.add_argument("--shift-le-to-zero", action="store_true",
                    help="Shift x so leading edge becomes x=0 before scaling (recommended if your data isn't already).")

    args = ap.parse_args()

    upper, lower = parse_airfoil_txt(args.txt_path)

    # Choose ordering
    if args.no_loop:
        combined = upper + lower  # simple concatenate
    else:
        combined = make_continuous_loop(upper, lower)

    if args.shift_le_to_zero:
        combined = shift_to_le_zero(combined)

    # Scale to requested chord
    scale = chord_scale_factor(upper, lower, args.chord_mm)
    combined_mm = scale_points(combined, scale)

    # Output path
    if args.out is None:
        stem = args.txt_path.stem
        args.out = args.txt_path.with_name(f"{stem}_{int(round(args.chord_mm))}mm.xlsx")

    write_xlsx(
        points_mm=combined_mm,
        out_path=args.out,
        template_path=args.template,
        sheet_name=args.sheet,
        start_row=args.start_row,
        start_col=args.start_col,
        write_header=not args.no_header,
    )

    print(f"Saved: {args.out}")
    print(f"Scale factor applied: {scale:.6g} (to reach chord {args.chord_mm} mm)")


if __name__ == "__main__":
    main()
