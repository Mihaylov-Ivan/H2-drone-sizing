from __future__ import annotations

"""Automated AVL geometry export, batch execution and derivative import.

AVL is used as an inviscid lifting-surface stability/control derivative model.
It does not replace the existing stall, viscous drag, propeller, transition or
thermal models.  The integration is deliberately self-contained so the normal
model can fall back to the existing screening derivatives when AVL is absent
and the user allows fallback.
"""

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Mapping, Optional
import hashlib
import json
import os
import re
import shutil
import subprocess
import tempfile


_CACHE: Dict[str, Dict[str, Any]] = {}


def _f(x: Any) -> float:
    return float(x)


def _naca_digits(code: Any, default: str = "0012") -> str:
    digits = "".join(ch for ch in str(code) if ch.isdigit())
    if len(digits) >= 4:
        return digits[-4:]
    return default


def _trapezoid_mac(root: float, taper: float) -> float:
    return (2.0 / 3.0) * root * (1.0 + taper + taper * taper) / (1.0 + taper)


def _surface_z(cfg: Mapping[str, Any], name: str, default: float) -> float:
    mm = cfg.get("manufacturing_mass_model") or {}
    return float((mm.get(name) or {}).get("z_m", default))


def _boom_y(cfg: Mapping[str, Any]) -> float:
    geom = (((cfg.get("geometry_mass") or {}).get("geometry_model") or {}).get("geometry") or {})
    ys = geom.get("boom_center_y_m")
    if isinstance(ys, (list, tuple)) and len(ys) >= 2:
        return max(abs(float(ys[0])), abs(float(ys[1])))
    return abs(float((((cfg.get("aero_propulsion") or {}).get("lift_layout") or {}).get("y_abs_m", 0.374))))


def _section(
    xle: float,
    yle: float,
    zle: float,
    chord: float,
    ainc_deg: float,
    *,
    naca: str,
    control: Optional[tuple[str, float, float, tuple[float, float, float], float]] = None,
) -> str:
    lines = [
        "SECTION",
        "# Xle Yle Zle Chord Ainc",
        f"{xle:.8f} {yle:.8f} {zle:.8f} {chord:.8f} {ainc_deg:.6f}",
        "NACA",
        naca,
    ]
    if control is not None:
        name, gain, hinge, vec, sgn_dup = control
        lines += [
            "CONTROL",
            f"{name} {gain:.6f} {hinge:.6f} {vec[0]:.6f} {vec[1]:.6f} {vec[2]:.6f} {sgn_dup:.6f}",
        ]
    return "\n".join(lines)


def render_avl_geometry(cfg: Mapping[str, Any], mass_result: Any) -> str:
    """Render the current aircraft lifting-surface geometry as an AVL file."""
    d = cfg["design"]
    avl_cfg = cfg.get("avl_derivatives") or {}

    S = _f(d["wing_area_m2"])
    b = _f(d["wing_span_m"])
    cref = _f(d["wing_mac_m"])
    cg = mass_result.takeoff.cg_m
    xref, yref, zref = _f(cg[0]), _f(cg[1]), _f(cg[2])

    wing_root = _f(d["main_wing_root_chord_m"])
    wing_taper = _f(d["main_wing_taper_ratio"])
    wing_tip = wing_root * wing_taper
    wing_mac = _f(d["wing_mac_m"])
    wing_half = b / 2.0
    wing_z = _surface_z(cfg, "main_wing", 0.087)
    wing_xle_root = _f(d["wing_ac_x_m"]) - 0.25 * wing_mac
    sweep_deg = _f(d.get("main_wing_sweep_deg", 0.0))
    from math import radians, tan
    wing_xle_tip = wing_xle_root + tan(radians(sweep_deg)) * wing_half
    wing_twist_root = _f(d.get("main_wing_twist_root_deg", 0.0))
    wing_twist_tip = _f(d.get("main_wing_twist_tip_deg", 0.0))
    wing_naca = _naca_digits(d.get("main_wing_naca_code"), "6415")

    ail_start = float(avl_cfg.get("aileron_start_semispan_fraction", 0.60))
    ail_hinge = float(avl_cfg.get("aileron_hinge_fraction", 0.70))
    ail_start = min(0.90, max(0.20, ail_start))
    y_ail = ail_start * wing_half
    frac = y_ail / max(wing_half, 1e-9)
    chord_ail = wing_root + frac * (wing_tip - wing_root)
    xle_ail = wing_xle_root + frac * (wing_xle_tip - wing_xle_root)
    twist_ail = wing_twist_root + frac * (wing_twist_tip - wing_twist_root)
    # d1 = aileron. SgnDup=-1 gives opposite deflection on the mirrored half.
    aileron = ("aileron", 1.0, ail_hinge, (0.0, 1.0, 0.0), -1.0)

    ht_span = _f(d["horizontal_tail_span_m"])
    ht_root = _f(d["horizontal_tail_root_chord_m"])
    ht_taper = _f(d["horizontal_tail_taper_ratio"])
    ht_tip = ht_root * ht_taper
    ht_mac = _f(d.get("horizontal_tail_mac_m", _trapezoid_mac(ht_root, ht_taper)))
    ht_half = ht_span / 2.0
    ht_z = _surface_z(cfg, "horizontal_tail", wing_z)
    ht_xle = _f(d["horizontal_tail_ac_x_m"]) - 0.25 * ht_mac
    ht_naca = _naca_digits(d.get("horizontal_tail_naca_code"), "0012")
    ht_twist_root = _f(d.get("horizontal_tail_twist_root_deg", 0.0))
    ht_twist_tip = _f(d.get("horizontal_tail_twist_tip_deg", 0.0))
    elev_hinge = float(avl_cfg.get("elevator_hinge_fraction", 0.70))
    # d2 = elevator.
    elevator = ("elevator", 1.0, elev_hinge, (0.0, 1.0, 0.0), 1.0)

    vt_h = _f(d["vertical_tail_height_m"])
    vt_root = _f(d["vertical_tail_root_chord_m"])
    vt_taper = _f(d["vertical_tail_taper_ratio"])
    vt_tip = vt_root * vt_taper
    vt_mac = _trapezoid_mac(vt_root, vt_taper)
    vt_xle = _f(d["vertical_tail_ac_x_m"]) - 0.25 * vt_mac
    vt_y = _boom_y(cfg)
    vt_z0 = ht_z
    vt_naca = _naca_digits(d.get("vertical_tail_naca_code"), "0012")
    rud_hinge = float(avl_cfg.get("rudder_hinge_fraction", 0.70))
    # d3 = rudder. Both explicit fins use the same control variable.
    rudder = ("rudder", 1.0, rud_hinge, (0.0, 0.0, 1.0), 1.0)

    nch = int(avl_cfg.get("n_chordwise", 8))
    nspan_w = int(avl_cfg.get("n_spanwise_wing", 16))
    nspan_t = int(avl_cfg.get("n_spanwise_tail", 8))

    lines = [
        "H2AIR automated AVL model",
        "0.0",
        "0 0 0.0",
        f"{S:.8f} {cref:.8f} {b:.8f}",
        f"{xref:.8f} {yref:.8f} {zref:.8f}",
        "0.0",
        "",
        "SURFACE",
        "MainWing",
        f"{nch} 1.0 {nspan_w} 1.0",
        "YDUPLICATE",
        "0.0",
        _section(wing_xle_root, 0.0, wing_z, wing_root, wing_twist_root, naca=wing_naca),
        _section(xle_ail, y_ail, wing_z, chord_ail, twist_ail, naca=wing_naca, control=aileron),
        _section(wing_xle_tip, wing_half, wing_z, wing_tip, wing_twist_tip, naca=wing_naca, control=aileron),
        "",
        "SURFACE",
        "HorizontalTail",
        f"{max(6,nch-2)} 1.0 {nspan_t} 1.0",
        "YDUPLICATE",
        "0.0",
        _section(ht_xle, 0.0, ht_z, ht_root, ht_twist_root, naca=ht_naca, control=elevator),
        _section(ht_xle, ht_half, ht_z, ht_tip, ht_twist_tip, naca=ht_naca, control=elevator),
        "",
    ]

    for side, y in (("LeftVerticalTail", -vt_y), ("RightVerticalTail", vt_y)):
        lines += [
            "SURFACE",
            side,
            f"{max(6,nch-2)} 1.0 {nspan_t} 1.0",
            _section(vt_xle, y, vt_z0, vt_root, 0.0, naca=vt_naca, control=rudder),
            _section(vt_xle, y, vt_z0 + vt_h, vt_tip, 0.0, naca=vt_naca, control=rudder),
            "",
        ]

    return "\n".join(lines).rstrip() + "\n"


def _parse_avl_output(text: str) -> Dict[str, float]:
    vals: Dict[str, float] = {}
    pat = re.compile(r"\b([A-Za-z][A-Za-z0-9]*)\s*=\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][-+]?\d+)?)")
    for key, raw in pat.findall(text):
        try:
            vals[key] = float(raw)
        except ValueError:
            pass
    return vals


def _normalize_controls(vals: Mapping[str, float]) -> Dict[str, float]:
    """Map AVL d1/d2/d3 to the H2AIR aileron/elevator/rudder conventions."""
    out: Dict[str, float] = {}

    # Stability/state derivatives: AVL stability-axis normalization matches the
    # H2AIR q*b/(2V), q*c/(2V) convention.
    direct = {
        "CLa": "CL_alpha_per_rad",
        "CYb": "CY_beta",
        "CYp": "CY_p",
        "CYr": "CY_r",
        "Clb": "Cl_beta",
        "Clp": "Cl_p",
        "Clr": "Cl_r",
        "Cmq": "Cm_q",
        "Cnb": "Cn_beta",
        "Cnp": "Cn_p",
        "Cnr": "Cn_r",
    }
    for src, dst in direct.items():
        if src in vals:
            out[dst] = float(vals[src])

    # Controls are defined in this exporter as d1 aileron, d2 elevator, d3 rudder.
    # Normalize sign to the existing H2AIR controller convention:
    # +aileron -> +Cl, +elevator -> -Cm, +rudder -> -Cn.
    ail_sign = 1.0
    if vals.get("Cld1") is not None and float(vals["Cld1"]) < 0.0:
        ail_sign = -1.0
    elev_sign = 1.0
    if vals.get("Cmd2") is not None and float(vals["Cmd2"]) > 0.0:
        elev_sign = -1.0
    rud_sign = 1.0
    if vals.get("Cnd3") is not None and float(vals["Cnd3"]) > 0.0:
        rud_sign = -1.0

    control_map = {
        "Cld1": ("Cl_da", ail_sign),
        "Cnd1": ("Cn_da", ail_sign),
        "Cmd2": ("Cm_de", elev_sign),
        "CYd3": ("CY_dr", rud_sign),
        "Cld3": ("Cl_dr", rud_sign),
        "Cnd3": ("Cn_dr", rud_sign),
    }
    for src, (dst, sign) in control_map.items():
        if src in vals:
            out[dst] = sign * float(vals[src])

    if "Xnp" in vals:
        out["neutral_point_station_x_m"] = float(vals["Xnp"])
    if "Cma" in vals:
        out["Cm_alpha_per_rad"] = float(vals["Cma"])
    return out


def _cache_key(geometry: str, executable: str, avl_cfg: Mapping[str, Any]) -> str:
    payload = {
        "geometry": geometry,
        "executable": executable,
        "settings": {
            k: avl_cfg.get(k)
            for k in (
                "aileron_start_semispan_fraction", "aileron_hinge_fraction",
                "elevator_hinge_fraction", "rudder_hinge_fraction",
                "n_chordwise", "n_spanwise_wing", "n_spanwise_tail",
            )
        },
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True, default=str).encode()).hexdigest()


def find_avl_executable(avl_cfg: Mapping[str, Any]) -> Optional[str]:
    requested = avl_cfg.get("executable") or os.environ.get("AVL_EXECUTABLE")
    if requested:
        p = shutil.which(str(requested)) or (str(requested) if Path(str(requested)).exists() else None)
        if p:
            return str(Path(p).resolve())
    p = shutil.which("avl")
    return str(Path(p).resolve()) if p else None


def evaluate_avl_derivatives(cfg: Mapping[str, Any], mass_result: Any) -> Dict[str, Any]:
    """Run AVL for the current geometry and return imported stability derivatives."""
    avl_cfg = cfg.get("avl_derivatives") or {}
    mode = str(avl_cfg.get("mode", "approximate")).lower().strip()
    if mode not in {"avl", "approximate"}:
        raise ValueError("avl_derivatives.mode must be 'AVL' or 'approximate'")
    if mode != "avl":
        return {"status": "NOT_USED", "mode": mode, "reason": "approximate derivative mode selected"}

    exe = find_avl_executable(avl_cfg)
    required = bool(avl_cfg.get("required", False))
    if exe is None:
        msg = "AVL executable not found. Install AVL or set AVL_EXECUTABLE / avl_derivatives.executable."
        if required:
            raise RuntimeError(msg)
        return {"status": "FALLBACK", "mode": "avl", "reason": msg}

    geometry = render_avl_geometry(cfg, mass_result)
    key = _cache_key(geometry, exe, avl_cfg)
    if bool(avl_cfg.get("cache", True)) and key in _CACHE:
        out = deepcopy(_CACHE[key])
        out["cache_hit"] = True
        return out

    timeout_s = float(avl_cfg.get("timeout_s", 12.0))
    with tempfile.TemporaryDirectory(prefix="h2air_avl_") as td:
        td_path = Path(td)
        geom_path = td_path / "h2air.avl"
        deriv_path = td_path / "stability.txt"
        geom_path.write_text(geometry, encoding="utf-8")

        commands = "\n".join([
            "OPER",
            "A A 0.0",
            "X",
            "ST",
            deriv_path.name,
            "",
            "QUIT",
            "",
        ])
        proc = subprocess.run(
            [exe, geom_path.name],
            input=commands,
            text=True,
            capture_output=True,
            cwd=td,
            timeout=timeout_s,
            check=False,
        )
        text = deriv_path.read_text(encoding="utf-8", errors="ignore") if deriv_path.exists() else proc.stdout
        vals = _parse_avl_output(text)
        mapped = _normalize_controls(vals)

        required_keys = {
            "CL_alpha_per_rad", "Cm_alpha_per_rad", "neutral_point_station_x_m",
            "Cm_de", "Cn_beta", "Cn_dr", "Cl_da",
        }
        missing = sorted(required_keys - set(mapped))
        status = "PASS" if proc.returncode == 0 and not missing else "FAIL"
        result: Dict[str, Any] = {
            "status": status,
            "mode": "avl",
            "executable": exe,
            "returncode": int(proc.returncode),
            "cache_hit": False,
            "geometry_sha256": hashlib.sha256(geometry.encode()).hexdigest(),
            "derivatives": mapped,
            "raw_selected": {k: vals[k] for k in sorted(vals) if k in {
                "CLa","Cma","Xnp","CYb","CYp","CYr","Clb","Clp","Clr",
                "Cmq","Cnb","Cnp","Cnr","Cld1","Cnd1","Cmd2","CYd3","Cld3","Cnd3"
            }},
            "missing_required": missing,
            "stdout_tail": proc.stdout[-3000:],
            "stderr_tail": proc.stderr[-2000:],
            "fidelity": "AVL vortex-lattice lifting-surfaces only; fuselage/boom viscous and separated-flow effects are not represented in these derivatives.",
        }
        if status != "PASS" and required:
            raise RuntimeError(
                f"AVL derivative evaluation failed (returncode={proc.returncode}, missing={missing}). "
                f"stderr={proc.stderr[-500:]}"
            )

    if bool(avl_cfg.get("cache", True)) and result["status"] == "PASS":
        _CACHE[key] = deepcopy(result)
    return result
