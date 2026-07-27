# naca_to_dxf.py
"""
Generate NACA 4-digit airfoil coordinates and write a DXF (R12) outline.

- No external dependencies.
- Produces a single closed POLYLINE in mm units (you can scale in Inventor if needed).
- Works for symmetric (e.g., 0006) and cambered (e.g., 2410).

Usage examples:
  python naca_to_dxf.py --naca 0012 --chord_mm 200 --n 201 --out NACA0012_200mm.dxf
  python naca_to_dxf.py --naca 2410 --chord_mm 250 --n 301 --out NACA2410_250mm.dxf
  python naca_to_dxf.py --batch 0006 2410 --chord_mm 200 --n 241 --out_dir ./dxf
"""

from __future__ import annotations
import argparse
import math
import os
from typing import List, Tuple


Point = Tuple[float, float]


def _cosine_spacing(n: int) -> List[float]:
    """
    Cosine spacing from 0..1 with clustering near LE/TE.
    Returns x list length n, from 0 -> 1.
    """
    if n < 3:
        raise ValueError("n must be >= 3")
    xs = []
    for i in range(n):
        beta = math.pi * i / (n - 1)
        x = 0.5 * (1.0 - math.cos(beta))
        xs.append(x)
    return xs


def naca4_coordinates(naca: str, n: int = 201) -> List[Point]:
    """
    Returns a CLOSED outline in normalized chord units:
      - Starts at TE on upper surface, goes to LE, then back to TE on lower surface.
      - No duplicate last point; caller can close polyline.

    naca: '0006', '2410', etc.
    n: number of points per surface INCLUDING LE and TE (typical 201..401).
    """
    naca = naca.strip()
    if len(naca) != 4 or not naca.isdigit():
        raise ValueError("NACA must be 4 digits, e.g. '0006' or '2410'")

    m = int(naca[0]) / 100.0      # max camber
    p = int(naca[1]) / 10.0       # location of max camber
    t = int(naca[2:]) / 100.0     # thickness ratio

    # Standard NACA thickness distribution coefficients (closed TE often uses -0.1036;
    # -0.1015 yields ~finite TE thickness. For CAD, -0.1015 is commonly used.
    a0, a1, a2, a3, a4 = 0.2969, -0.1260, -0.3516, 0.2843, -0.1015

    xs = _cosine_spacing(n)

    upper: List[Point] = []
    lower: List[Point] = []

    for x in xs:
        # Thickness distribution
        yt = 5.0 * t * (
            a0 * math.sqrt(max(x, 0.0))
            + a1 * x
            + a2 * x**2
            + a3 * x**3
            + a4 * x**4
        )

        # Camber line yc and slope dyc/dx
        if m == 0.0 or p == 0.0:
            yc = 0.0
            dyc_dx = 0.0
        else:
            if x < p:
                yc = (m / (p**2)) * (2.0 * p * x - x**2)
                dyc_dx = (2.0 * m / (p**2)) * (p - x)
            else:
                yc = (m / ((1.0 - p) ** 2)) * ((1.0 - 2.0 * p) + 2.0 * p * x - x**2)
                dyc_dx = (2.0 * m / ((1.0 - p) ** 2)) * (p - x)

        theta = math.atan(dyc_dx)

        # Upper surface
        xu = x - yt * math.sin(theta)
        yu = yc + yt * math.cos(theta)

        # Lower surface
        xl = x + yt * math.sin(theta)
        yl = yc - yt * math.cos(theta)

        upper.append((xu, yu))
        lower.append((xl, yl))

    # Build closed outline: start at TE upper -> LE -> TE lower
    # upper is from LE..TE because xs goes 0..1, so reverse it for TE..LE.
    outline = list(reversed(upper)) + lower[1:-1]  # avoid duplicating LE and TE twice
    return outline


def scale_points(points: List[Point], chord_mm: float) -> List[Point]:
    """Scale normalized-chord points to mm."""
    return [(x * chord_mm, y * chord_mm) for (x, y) in points]


def write_dxf_r12_polyline(path: str, points_mm: List[Point], layer: str = "AIRFOIL") -> None:
    """
    Writes a minimal AutoCAD R12 DXF with one closed POLYLINE.
    Many CAD packages (including Inventor) import this cleanly.
    """
    if len(points_mm) < 3:
        raise ValueError("Need at least 3 points")

    # Ensure directory exists
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

    def dxf(*pairs) -> str:
        # pairs as (code, value, code, value, ...)
        out = []
        for code, value in pairs:
            out.append(str(code))
            out.append(str(value))
        return "\n".join(out) + "\n"

    with open(path, "w", newline="\n") as f:
        # HEADER + TABLES (minimal)
        f.write("0\nSECTION\n2\nHEADER\n0\nENDSEC\n")
        f.write("0\nSECTION\n2\nTABLES\n0\nENDSEC\n")
        f.write("0\nSECTION\n2\nBLOCKS\n0\nENDSEC\n")

        # ENTITIES
        f.write("0\nSECTION\n2\nENTITIES\n")

        # POLYLINE entity
        f.write(dxf(
            (0, "POLYLINE"),
            (8, layer),     # layer
            (66, 1),        # vertices follow
            (70, 1),        # closed polyline
            (10, 0), (20, 0), (30, 0),
        ))

        # VERTEX entities
        for (x, y) in points_mm:
            f.write(dxf(
                (0, "VERTEX"),
                (8, layer),
                (10, f"{x:.6f}"),
                (20, f"{y:.6f}"),
                (30, "0.0"),
            ))

        # SEQEND
        f.write(dxf((0, "SEQEND"), (8, layer)))

        # END ENTITIES
        f.write("0\nENDSEC\n0\nEOF\n")


def generate_one(naca: str, chord_mm: float, n: int, out_path: str) -> None:
    pts = naca4_coordinates(naca=naca, n=n)
    pts_mm = scale_points(pts, chord_mm=chord_mm)
    write_dxf_r12_polyline(out_path, pts_mm, layer=f"NACA{naca}")
    print(f"Wrote: {out_path}  (NACA {naca}, chord={chord_mm} mm, points={len(pts_mm)})")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--naca", type=str, default=None, help="NACA 4-digit code, e.g. 0006 or 2410")
    ap.add_argument("--batch", nargs="+", default=None, help="Multiple NACA codes, e.g. --batch 0006 2410")
    ap.add_argument("--chord_mm", type=float, default=200.0, help="Chord length in mm")
    ap.add_argument("--n", type=int, default=241, help="Points per surface (>=3). Typical 201..401")
    ap.add_argument("--out", type=str, default=None, help="Output DXF path (single mode)")
    ap.add_argument("--out_dir", type=str, default="dxf_out", help="Output directory (batch mode)")
    args = ap.parse_args()

    if args.batch:
        os.makedirs(args.out_dir, exist_ok=True)
        for code in args.batch:
            code = code.strip()
            out_path = os.path.join(args.out_dir, f"NACA{code}_{args.chord_mm:g}mm.dxf")
            generate_one(code, args.chord_mm, args.n, out_path)
        return

    if not args.naca:
        raise SystemExit("Provide --naca 0006 (or use --batch ...)")

    out_path = args.out or f"NACA{args.naca}_{args.chord_mm:g}mm.dxf"
    generate_one(args.naca, args.chord_mm, args.n, out_path)


if __name__ == "__main__":
    main()
