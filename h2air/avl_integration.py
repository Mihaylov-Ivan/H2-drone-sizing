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
from math import pi


_CACHE: Dict[str, Dict[str, Any]] = {}
_WING_CLA_CACHE: Dict[str, float] = {}


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


def render_avl_geometry(cfg: Mapping[str, Any], mass_result: Any, *, wing_only: bool = False) -> str:
    """Render the current aircraft lifting-surface geometry as an AVL file.

    wing_only=True is used for main-wing gust/stall lift slope so whole-aircraft
    tail lift is not incorrectly charged against the main-wing CLmax margin.
    """
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
    ]
    if wing_only:
        return "\n".join(lines).rstrip() + "\n"

    lines += [
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
    # First assignment wins. Later summary lines such as
    # "Clb Cnr / Clr Cnb = ..." must not overwrite the true Cnb derivative.
    pat = re.compile(r"\b([A-Za-z][A-Za-z0-9]*)\s*=\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][-+]?\d+)?)")
    for key, raw in pat.findall(text):
        if key in vals:
            continue
        try:
            vals[key] = float(raw)
        except ValueError:
            pass
    return vals


def _avl_get(vals: Mapping[str, float], *names: str) -> Optional[float]:
    """Return the first named AVL token present in a parsed ST dump."""
    for name in names:
        if name in vals:
            return float(vals[name])
    return None


def _control_token_names(prefix: str, index: int) -> tuple[str, ...]:
    """Accept both `Cld1` and MIT AVL 3.5x `Cld01` control-derivative labels."""
    return (f"{prefix}{index}", f"{prefix}{index:02d}")


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
    cld1 = _avl_get(vals, *_control_token_names("Cld", 1))
    cmd2 = _avl_get(vals, *_control_token_names("Cmd", 2))
    cnd3 = _avl_get(vals, *_control_token_names("Cnd", 3))
    ail_sign = -1.0 if cld1 is not None and cld1 < 0.0 else 1.0
    elev_sign = -1.0 if cmd2 is not None and cmd2 > 0.0 else 1.0
    rud_sign = -1.0 if cnd3 is not None and cnd3 > 0.0 else 1.0

    control_map = (
        ("Cl_da", ail_sign, _control_token_names("Cld", 1)),
        ("Cn_da", ail_sign, _control_token_names("Cnd", 1)),
        ("Cm_de", elev_sign, _control_token_names("Cmd", 2)),
        ("CY_dr", rud_sign, _control_token_names("CYd", 3)),
        ("Cl_dr", rud_sign, _control_token_names("Cld", 3)),
        ("Cn_dr", rud_sign, _control_token_names("Cnd", 3)),
    )
    # AVL's control variables/printed control sensitivities are per degree of
    # DELCON. H2AIR's 6-DOF uses control derivatives per radian, so convert.
    per_degree_to_per_radian = 180.0 / pi
    for dst, sign, names in control_map:
        raw = _avl_get(vals, *names)
        if raw is not None:
            out[dst] = sign * raw * per_degree_to_per_radian

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


def _wing_cache_key(cfg: Mapping[str, Any], executable: str) -> str:
    d = cfg["design"]
    avl_cfg = cfg.get("avl_derivatives") or {}
    payload = {
        "executable": executable,
        "span": d.get("wing_span_m"),
        "area": d.get("wing_area_m2"),
        "root": d.get("main_wing_root_chord_m"),
        "taper": d.get("main_wing_taper_ratio"),
        "mac": d.get("wing_mac_m"),
        "ac_x": d.get("wing_ac_x_m"),
        "twist_root": d.get("main_wing_twist_root_deg"),
        "twist_tip": d.get("main_wing_twist_tip_deg"),
        "sweep": d.get("main_wing_sweep_deg"),
        "naca": d.get("main_wing_naca_code"),
        "n_chordwise": avl_cfg.get("n_chordwise"),
        "n_spanwise_wing": avl_cfg.get("n_spanwise_wing"),
        "aileron_start": avl_cfg.get("aileron_start_semispan_fraction"),
        "aileron_hinge": avl_cfg.get("aileron_hinge_fraction"),
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True, default=str).encode()).hexdigest()

def find_avl_executable(avl_cfg: Mapping[str, Any]) -> Optional[str]:
    requested = avl_cfg.get("executable") or os.environ.get("AVL_EXECUTABLE")
    if requested:
        p = shutil.which(str(requested)) or (str(requested) if Path(str(requested)).exists() else None)
        if p:
            return str(Path(p).resolve())
    # Prefer a repo-local Windows/Linux binary if present (tools/bin/avl[.exe]).
    repo_bin = Path(__file__).resolve().parents[1] / "tools" / "bin"
    for name in ("avl.exe", "avl"):
        candidate = repo_bin / name
        if candidate.exists():
            return str(candidate.resolve())
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

        rho = float(cfg["sixdof_control"]["aerodynamics"].get("rho_kg_m3", 1.225))
        speed = float(cfg["design"]["cruise_speed_mps"])
        area = float(cfg["design"]["wing_area_m2"])
        weight = float(mass_result.takeoff.total_mass_kg) * 9.80665
        q = 0.5 * rho * speed * speed
        cl_target = weight / max(q * area, 1e-9)

        # Solve the actual cruise lifting condition and trim d2 (elevator) to
        # zero pitching moment before requesting the derivative matrix. This
        # keeps the linearized derivatives and trim reference internally
        # consistent for a cambered wing and changing tail geometry.
        commands = "\n".join([
            "OPER",
            f"A C {cl_target:.8f}",
            "D2 PM 0.0",
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

        # Main-wing-only CLa is used by the main-wing CLmax gust/stall screen.
        # Whole-aircraft CLa remains the correct value for rigid-body dynamics.
        wing_key = _wing_cache_key(cfg, exe)
        wing_cache_hit = False
        if bool(avl_cfg.get("cache", True)) and wing_key in _WING_CLA_CACHE:
            wing_cla = float(_WING_CLA_CACHE[wing_key])
            wing_cache_hit = True
        else:
            wing_geom = render_avl_geometry(cfg, mass_result, wing_only=True)
            wing_geom_path = td_path / "h2air_wing_only.avl"
            wing_deriv_path = td_path / "wing_stability.txt"
            wing_geom_path.write_text(wing_geom, encoding="utf-8")
            wing_commands = "\n".join([
                "OPER",
                "A A 0.0",
                "X",
                "ST",
                wing_deriv_path.name,
                "",
                "QUIT",
                "",
            ])
            wing_proc = subprocess.run(
                [exe, wing_geom_path.name],
                input=wing_commands, text=True, capture_output=True,
                cwd=td, timeout=timeout_s, check=False,
            )
            wing_text = wing_deriv_path.read_text(encoding="utf-8", errors="ignore") if wing_deriv_path.exists() else wing_proc.stdout
            wing_vals = _parse_avl_output(wing_text)
            wing_cla = float(wing_vals["CLa"]) if "CLa" in wing_vals else float("nan")
            if wing_proc.returncode == 0 and wing_cla > 0.0 and bool(avl_cfg.get("cache", True)):
                _WING_CLA_CACHE[wing_key] = wing_cla
        if wing_cla > 0.0:
            mapped["wing_CL_alpha_per_rad"] = wing_cla

        # Normalize the trim elevator sign using the same convention as Cm_de.
        cmd2 = _avl_get(vals, *_control_token_names("Cmd", 2))
        elev_sign = -1.0 if cmd2 is not None and cmd2 > 0.0 else 1.0
        trim = {
            "target_CL": float(cl_target),
            "alpha_deg": float(vals["Alpha"]) if "Alpha" in vals else None,
            "elevator_deg": elev_sign * float(vals["elevator"]) if "elevator" in vals else None,
            "Cmtot": float(vals["Cmtot"]) if "Cmtot" in vals else None,
        }

        required_keys = {
            "CL_alpha_per_rad", "wing_CL_alpha_per_rad", "Cm_alpha_per_rad", "neutral_point_station_x_m",
            "Cm_de", "Cn_beta", "Cn_dr", "Cl_da",
        }
        missing = sorted(required_keys - set(mapped))
        if trim["alpha_deg"] is None:
            missing.append("trim_alpha_deg")
        if trim["elevator_deg"] is None:
            missing.append("trim_elevator_deg")
        status = "PASS" if proc.returncode == 0 and not missing else "FAIL"
        result: Dict[str, Any] = {
            "status": status,
            "mode": "avl",
            "executable": exe,
            "returncode": int(proc.returncode),
            "cache_hit": False,
            "geometry_sha256": hashlib.sha256(geometry.encode()).hexdigest(),
            "derivatives": mapped,
            "trim": trim,
            "wing_CL_alpha_per_rad": mapped.get("wing_CL_alpha_per_rad"),
            "wing_cla_cache_hit": wing_cache_hit,
            "raw_selected": {k: vals[k] for k in sorted(vals) if k in {
                "CLa","Cma","Xnp","CYb","CYp","CYr","Clb","Clp","Clr",
                "Cmq","Cnb","Cnp","Cnr",
                "Cld1","Cld01","Cnd1","Cnd01","Cmd2","Cmd02",
                "CYd3","CYd03","Cld3","Cld03","Cnd3","Cnd03",
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
