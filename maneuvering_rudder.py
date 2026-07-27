# maneuvering_rudder.py
from __future__ import annotations

import math
from typing import Dict, List, Tuple, Optional, Any

from constants import ATMOSPHERIC
from design_inputs import DesignInputs


# ============================================================
# Small utilities
# ============================================================
def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _lerp(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
    if abs(x1 - x0) < 1e-12:
        return y0
    t = (x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)


def _piecewise_linear(x: float, pts: List[Tuple[float, float]]) -> float:
    pts = sorted(pts, key=lambda p: p[0])
    if x <= pts[0][0]:
        return pts[0][1]
    if x >= pts[-1][0]:
        return pts[-1][1]
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        if x0 <= x <= x1:
            return _lerp(x, x0, x1, y0, y1)
    return pts[-1][1]


def _get_attr(obj: Any, name: str, default: float = 0.0) -> float:
    """Read either obj.name or obj[name] safely."""
    if obj is None:
        return float(default)
    if isinstance(obj, dict):
        v = obj.get(name, default)
        try:
            return float(v)
        except Exception:
            return float(default)
    try:
        return float(getattr(obj, name))
    except Exception:
        return float(default)


def _control_surface_effective_span_fraction(start: float, end: float) -> Dict[str, float]:
    """
    Computes geometric span fraction and an *effective* span fraction that accounts for
    spanwise lift distribution (tip segments contribute differently than root segments).

    start/end are normalized on SPAN: 0=root, 1=tip.

    Uses an elliptical-ish weighting w(x)=sqrt(1-x^2), integrated analytically.
    Effective fraction = integral(w) over [start,end] / integral(w) over [0,1].

    Returns:
      geom_frac: end-start
      eff_frac:  location-weighted effectiveness fraction (0..1)
    """
    s = _clamp(float(start), 0.0, 1.0)
    e = _clamp(float(end),   0.0, 1.0)
    if e <= s:
        return {"geom_frac": 0.0, "eff_frac": 0.0}

    # Antiderivative of sqrt(1-x^2):
    # F(x) = 0.5*( x*sqrt(1-x^2) + asin(x) )
    def F(x: float) -> float:
        x = _clamp(x, 0.0, 1.0)
        return 0.5 * (x * math.sqrt(max(0.0, 1.0 - x * x)) + math.asin(x))

    total = F(1.0) - F(0.0)  # = pi/4
    seg = F(e) - F(s)

    eff = seg / max(1e-12, total)
    geom = e - s
    return {"geom_frac": geom, "eff_frac": eff}


def _y_mac_trapezoid_single_surface(span_m: float, taper_ratio: float) -> float:
    """
    Mean-aerodynamic-chord spanwise station for a *single* trapezoid surface
    (root at y=0, tip at y=span). For a half-wing, this is:
      y_MAC = (b_half/3) * (1 + 2*lam) / (1 + lam)

    For a vertical tail, span_m should be the full height H (root->tip).
    """
    H = float(span_m)
    lam = float(taper_ratio)
    if H <= 0.0:
        return 0.0
    return (H / 3.0) * (1.0 + 2.0 * lam) / max(1e-12, (1.0 + lam))


# ============================================================
# Rudder effectiveness tau_R
# (Same chart concept as your elevator tau_e)
# You can override via inputs.vertical_tail_rudder_tau_override > 0.
# ============================================================
def rudder_tau_r(cR_over_cV: float, inputs: DesignInputs) -> float:
    tau_override = float(getattr(inputs, "vertical_tail_rudder_tau_override", -1.0))
    if tau_override > 0.0:
        return tau_override

    x = _clamp(float(cR_over_cV), 0.05, 0.8)

    pts = [
        (0.10, 0.20),
        (0.20, 0.35),
        (0.30, 0.48),
        (0.40, 0.60),
        (0.50, 0.68),
        (0.60, 0.75),
        (0.70, 0.80),
        (0.80, 0.84),
    ]
    return _piecewise_linear(x, pts)


# ============================================================
# STRUCTURAL LIMITS (VERTICAL TAIL)
# Mirrors your structures.calculate_surface_structure sizing relations:
#   M_cap ≈ sigma_cf_allow * A_spar * h
#   T_cap ≈ 2 * A_cell * tau_allow * t_skin
#
# Demand link for rudder maneuvers:
#   N_req  (yaw moment)  ->  Y_req = N_req / l_v   (tail sideforce)
#   M_req  ≈ (|Y_req|/2) * y_MAC_single
#
# Note:
# - This is strength-based (no stiffness/rigidity yet).
# - Torsion check uses an aerodynamic CM proxy (optional) because yaw-moment
#   requirements don’t uniquely define torsion without a shear-center model.
# ============================================================
def _vt_structural_capacity(
    *,
    vertical_tail_structure: Any,
    vt_c_root_m: float,
    vt_max_thickness_fraction: float,
    rudder_chord_ratio: float,
    inputs: DesignInputs,
) -> Dict[str, float]:
    spar_area_root = _get_attr(vertical_tail_structure, "spar_area_root_m2", 0.0)
    t_skin_root = _get_attr(vertical_tail_structure, "t_skin_root_m", 0.0)

    h = float(vt_c_root_m) * float(vt_max_thickness_fraction)
    h = max(h, 1e-9)

    # Bending capacity (CF)
    cf_sf = float(getattr(inputs, "carbon_fiber_sf", 1.0))  # optional; default 1.0
    sigma_allow_cf = float(inputs.carbon_fiber_max_bending_stress_MPa) * 1e6 / max(1.0, cf_sf)
    M_bend_cap = sigma_allow_cf * spar_area_root * h

    # Torsion capacity (skin, Bredt-Batho; same cell model as your structures.py)
    c_cell_root = float(vt_c_root_m) * float(rudder_chord_ratio)
    c_cell_root = max(c_cell_root, 1e-9)
    A_cell_root = 2.0 * c_cell_root * h / 3.0

    tau_allow = float(inputs.filament_shear_strength_MPa) * 1e6 / max(1.0, float(inputs.printed_parts_sf))
    T_cap = 2.0 * A_cell_root * tau_allow * t_skin_root

    return {
        "h_m": float(h),
        "spar_area_root_m2": float(spar_area_root),
        "t_skin_root_m": float(t_skin_root),
        "M_bend_cap_Nm": float(M_bend_cap),
        "T_cap_Nm": float(T_cap),
    }


def _vt_structural_check_from_yaw_moment(
    *,
    N_req_Nm: float,
    V_mps: float,
    wing_S_m2: float,
    wing_span_m: float,
    l_v_m: float,
    vertical_tail_S_m2: float,
    vt_span_m: float,
    vt_taper_ratio: float,
    vt_c_root_m: float,
    vt_c_MGC_m: float,
    vt_max_thickness_fraction: float,
    vertical_tail_structure: Any,
    inputs: DesignInputs,
    # rudder geometry for torsion-capacity cell model
    rudder_chord_ratio: float,
    # torsion proxy: if you have something better, pass it in
    vt_CM_abs_for_torsion: Optional[float] = None,
) -> Dict[str, float]:
    if vertical_tail_structure is None:
        return {"structural_check_used": 0.0}

    cap = _vt_structural_capacity(
        vertical_tail_structure=vertical_tail_structure,
        vt_c_root_m=vt_c_root_m,
        vt_max_thickness_fraction=vt_max_thickness_fraction,
        rudder_chord_ratio=rudder_chord_ratio,
        inputs=inputs,
    )

    # Convert required yaw moment to required VT sideforce
    if abs(float(l_v_m)) < 1e-9:
        return {"structural_check_used": 1.0, "ok_struct": 0.0, "reason": "l_v_too_small_for_struct_check"}

    Y_req = float(N_req_Nm) / float(l_v_m)  # [N] (sideforce on VT)

    # Root bending moment demand (single surface)
    y_mac = _y_mac_trapezoid_single_surface(span_m=vt_span_m, taper_ratio=vt_taper_ratio)
    M_req = (abs(Y_req) / 2.0) * y_mac

    # Torsion demand (proxy, same form as in structures.py: 0.5*q*S*c*|CM|)
    q = 0.5 * ATMOSPHERIC.rho * float(V_mps) ** 2
    if vt_CM_abs_for_torsion is None:
        # fallback: allow a user-defined CM; else use a conservative small constant
        vt_CM_abs_for_torsion = abs(float(getattr(inputs, "vertical_tail_CM_max", getattr(inputs, "tail_CM_max", 0.05))))
    T_req = 0.5 * q * float(vertical_tail_S_m2) * float(vt_c_MGC_m) * abs(float(vt_CM_abs_for_torsion))

    M_cap = float(cap["M_bend_cap_Nm"])
    T_cap = float(cap["T_cap_Nm"])

    margin_b = M_cap / max(1e-9, M_req)
    margin_t = T_cap / max(1e-9, T_req)

    ok_b = 1.0 if (M_req <= M_cap * 1.000) else 0.0
    ok_t = 1.0 if (T_req <= T_cap * 1.000) else 0.0
    ok = 1.0 if (ok_b > 0.5 and ok_t > 0.5) else 0.0

    # Also compute the max yaw moment allowed by bending capacity (useful to "constrain results")
    # M_req = (|Y|/2)*y_mac, Y = N/l_v => M_req = (|N|/(2*l_v))*y_mac
    # => |N|max_bend = 2*l_v*M_cap / y_mac
    N_max_bend = 0.0
    if y_mac > 1e-12:
        N_max_bend = 2.0 * abs(float(l_v_m)) * M_cap / y_mac

    return {
        "structural_check_used": 1.0,
        "Y_req_N": float(Y_req),
        "y_MAC_m": float(y_mac),
        "M_req_Nm": float(M_req),
        "T_req_Nm": float(T_req),
        "M_cap_Nm": float(M_cap),
        "T_cap_Nm": float(T_cap),
        "margin_bending": float(margin_b),
        "margin_torsion": float(margin_t),
        "ok_bending": float(ok_b),
        "ok_torsion": float(ok_t),
        "ok_struct": float(ok),
        "N_max_bend_Nm": float(N_max_bend),
        # echoes
        "spar_area_root_m2": float(cap["spar_area_root_m2"]),
        "t_skin_root_m": float(cap["t_skin_root_m"]),
        "h_m": float(cap["h_m"]),
        "vt_CM_abs_for_torsion": float(vt_CM_abs_for_torsion),
    }


# ============================================================
# Core derivatives for the vertical tail + rudder
# ============================================================
def rudder_control_derivatives(
    *,
    wing_S_m2: float,
    wing_span_m: float,
    vertical_tail_S_m2: float,
    vertical_tail_AR: float,
    x_cg_m: float,
    x_ac_vertical_tail_m: float,
    inputs: DesignInputs,
    S_v_effective_fraction: float = 1.0,
    eta_v_maneuver: Optional[float] = None,
) -> Dict[str, float]:
    S = float(wing_S_m2)
    b = float(wing_span_m)
    Sv = float(vertical_tail_S_m2)
    ARv = float(vertical_tail_AR)

    if S <= 0.0 or b <= 0.0 or Sv <= 0.0 or ARv <= 0.0:
        raise ValueError("Invalid reference/VT geometry for rudder derivatives.")

    l_v = float(x_ac_vertical_tail_m) - float(x_cg_m)
    if l_v <= 1e-9:
        raise ValueError("Vertical tail moment arm too small or negative (x_ac_vt must be aft of CG).")

    a2d = 2.0 * math.pi
    e_v = float(getattr(inputs, "tail_e_oswald", 0.8))
    a_v = a2d / (1.0 + a2d / (math.pi * max(1e-9, e_v) * ARv))

    cR_cV = float(getattr(inputs, "vertical_tail_rudder_chord_ratio", 0.30))
    tau_r = rudder_tau_r(cR_cV, inputs)
    seg = _control_surface_effective_span_fraction(
        getattr(inputs, "vertical_tail_rudder_span_start", 0.0),
        getattr(inputs, "vertical_tail_rudder_span_end", 1.0),
    )
    bR_bV_geom = seg["geom_frac"]
    bR_bV_eff = seg["eff_frac"]

    if eta_v_maneuver is None:
        eta_v_maneuver = float(getattr(inputs, "vertical_tail_eta_v", 1.0))

    dsigma_dbeta = float(getattr(inputs, "vertical_tail_dsigma_dbeta", 0.0))
    one_minus = (1.0 - dsigma_dbeta)

    S_v_eff = Sv * max(0.0, float(S_v_effective_fraction))
    V_Ve = (l_v * S_v_eff) / (b * S)

    Cn_deltaR = -a_v * float(eta_v_maneuver) * V_Ve * tau_r * bR_bV_eff
    CY_deltaR = +a_v * float(eta_v_maneuver) * (S_v_eff / S) * tau_r * bR_bV_eff
    CY_beta_v = -a_v * float(eta_v_maneuver) * (Sv / S) * one_minus

    return {
        "a_v_per_rad": a_v,
        "l_v_m": l_v,
        "cR_over_cV": cR_cV,
        "tau_r": tau_r,
        "bR_over_bV_geom": bR_bV_geom,
        "bR_over_bV_eff": bR_bV_eff,
        "eta_v": float(eta_v_maneuver),
        "dsigma_dbeta": dsigma_dbeta,
        "S_v_eff_m2": S_v_eff,
        "V_Ve": V_Ve,
        "Cn_deltaR_per_rad": Cn_deltaR,
        "CY_deltaR_per_rad": CY_deltaR,
        "CY_beta_v_per_rad": CY_beta_v,
    }


# ============================================================
# Rudder authority at a speed (NOW with OPTIONAL structural limiting)
# ============================================================
def rudder_max_yaw_moment(
    *,
    V_mps: float,
    wing_S_m2: float,
    wing_span_m: float,
    Cn_deltaR_per_rad: float,
    inputs: DesignInputs,
    # ----- OPTIONAL: pass from rudder_control_derivatives() so we can compute bending-limited N_max
    l_v_m: Optional[float] = None,
    # ----- OPTIONAL STRUCTURE / GEOMETRY (enables structural limiting)
    vertical_tail_structure: Optional[Any] = None,
    vertical_tail_S_m2: Optional[float] = None,
    vt_span_m: Optional[float] = None,
    vt_taper_ratio: Optional[float] = None,
    vt_c_root_m: Optional[float] = None,
    vt_c_MGC_m: Optional[float] = None,
    vt_max_thickness_fraction: Optional[float] = None,
    vt_CM_abs_for_torsion: Optional[float] = None,
) -> Dict[str, float]:
    q = 0.5 * ATMOSPHERIC.rho * float(V_mps) ** 2
    S = float(wing_S_m2)
    b = float(wing_span_m)

    dR_left = float(getattr(inputs, "vertical_tail_rudder_max_left_deg", -25.0))
    dR_right = float(getattr(inputs, "vertical_tail_rudder_max_right_deg", 25.0))
    dR_max_abs_deg = max(abs(dR_left), abs(dR_right))
    dR_max_abs_rad = math.radians(dR_max_abs_deg)

    # Aerodynamic yaw-moment limit
    N_aero_max = q * S * b * abs(float(Cn_deltaR_per_rad)) * dR_max_abs_rad

    out: Dict[str, Any] = {
        "q_Pa": float(q),
        "deltaR_max_abs_deg": float(dR_max_abs_deg),
        "deltaR_max_abs_rad": float(dR_max_abs_rad),
        "Cn_max_abs": float(abs(float(Cn_deltaR_per_rad)) * dR_max_abs_rad),
        "N_max_abs_Nm_aero": float(N_aero_max),
        "N_max_abs_Nm": float(N_aero_max),
        "limited_by_structure": 0.0,
        "structural_check_used": 0.0,
    }

    # Structural limiting (bending-driven) if we have enough info
    if (
        vertical_tail_structure is not None
        and l_v_m is not None
        and vertical_tail_S_m2 is not None
        and vt_span_m is not None
        and vt_taper_ratio is not None
        and vt_c_root_m is not None
        and vt_c_MGC_m is not None
        and vt_max_thickness_fraction is not None
    ):
        rudder_chord_ratio = float(getattr(inputs, "vertical_tail_rudder_chord_ratio", 0.30))

        struct = _vt_structural_check_from_yaw_moment(
            N_req_Nm=float(N_aero_max),
            V_mps=float(V_mps),
            wing_S_m2=float(wing_S_m2),
            wing_span_m=float(wing_span_m),
            l_v_m=float(l_v_m),
            vertical_tail_S_m2=float(vertical_tail_S_m2),
            vt_span_m=float(vt_span_m),
            vt_taper_ratio=float(vt_taper_ratio),
            vt_c_root_m=float(vt_c_root_m),
            vt_c_MGC_m=float(vt_c_MGC_m),
            vt_max_thickness_fraction=float(vt_max_thickness_fraction),
            vertical_tail_structure=vertical_tail_structure,
            inputs=inputs,
            rudder_chord_ratio=rudder_chord_ratio,
            vt_CM_abs_for_torsion=vt_CM_abs_for_torsion,
        )

        out.update({k: float(v) for k, v in struct.items()})

        # Apply bending-based limit to yaw moment
        N_max_bend = float(struct.get("N_max_bend_Nm", 0.0))
        if N_max_bend > 0.0:
            N_allowed = min(float(N_aero_max), N_max_bend)
            out["N_max_abs_Nm"] = float(N_allowed)
            if N_allowed < float(N_aero_max) * 0.999:
                out["limited_by_structure"] = 1.0

    return out


# ============================================================
# Asymmetric thrust check (NOW includes OPTIONAL structural check)
# ============================================================
def rudder_asymmetric_thrust(
    *,
    V_mps: float,
    wing_S_m2: float,
    wing_span_m: float,
    Cn_deltaR_per_rad: float,
    y_engine_m: float,
    thrust_asym_N: float,
    inputs: DesignInputs,
    # ----- OPTIONAL: geometry/arm for structural check
    l_v_m: Optional[float] = None,
    vertical_tail_structure: Optional[Any] = None,
    vertical_tail_S_m2: Optional[float] = None,
    vt_span_m: Optional[float] = None,
    vt_taper_ratio: Optional[float] = None,
    vt_c_root_m: Optional[float] = None,
    vt_c_MGC_m: Optional[float] = None,
    vt_max_thickness_fraction: Optional[float] = None,
    vt_CM_abs_for_torsion: Optional[float] = None,
) -> Dict[str, float]:
    q = 0.5 * ATMOSPHERIC.rho * float(V_mps) ** 2
    S = float(wing_S_m2)
    b = float(wing_span_m)

    if q <= 1e-9 or S <= 0.0 or b <= 0.0:
        raise ValueError("Invalid q/S/b in asymmetric thrust check.")

    if abs(float(Cn_deltaR_per_rad)) < 1e-12:
        raise ValueError("Cn_deltaR is ~0; rudder has no yaw authority in model.")

    # Required yaw moment from thrust asymmetry
    N_req = float(thrust_asym_N) * float(y_engine_m)

    # Required deflection
    deltaR_req_rad = N_req / (q * S * b * float(Cn_deltaR_per_rad))
    deltaR_req_deg = math.degrees(deltaR_req_rad)

    # Limits
    dR_left = float(getattr(inputs, "vertical_tail_rudder_max_left_deg", -25.0))
    dR_right = float(getattr(inputs, "vertical_tail_rudder_max_right_deg", 25.0))

    # Max thrust asymmetry that can be trimmed at this V:
    dR_max_abs_rad = math.radians(max(abs(dR_left), abs(dR_right)))
    N_max_abs = q * S * b * abs(float(Cn_deltaR_per_rad)) * dR_max_abs_rad
    T_asym_max = N_max_abs / max(1e-9, abs(float(y_engine_m)))

    feasible = (dR_left <= deltaR_req_deg <= dR_right)
    reason = "" if feasible else "rudder_deflection_limit"

    out: Dict[str, Any] = {
        "q_Pa": float(q),
        "y_engine_m": float(y_engine_m),
        "thrust_asym_N": float(thrust_asym_N),
        "N_req_Nm": float(N_req),
        "deltaR_req_deg": float(deltaR_req_deg),
        "deltaR_limits_deg": (float(dR_left), float(dR_right)),
        "T_asym_max_N": float(T_asym_max),
        "feasible": 1.0 if feasible else 0.0,
        "reason": reason,
        "structural_check_used": 0.0,
        "ok_struct": 1.0,
    }

    # OPTIONAL structural feasibility check
    if (
        vertical_tail_structure is not None
        and l_v_m is not None
        and vertical_tail_S_m2 is not None
        and vt_span_m is not None
        and vt_taper_ratio is not None
        and vt_c_root_m is not None
        and vt_c_MGC_m is not None
        and vt_max_thickness_fraction is not None
    ):
        rudder_chord_ratio = float(getattr(inputs, "vertical_tail_rudder_chord_ratio", 0.30))
        struct = _vt_structural_check_from_yaw_moment(
            N_req_Nm=float(N_req),
            V_mps=float(V_mps),
            wing_S_m2=float(wing_S_m2),
            wing_span_m=float(wing_span_m),
            l_v_m=float(l_v_m),
            vertical_tail_S_m2=float(vertical_tail_S_m2),
            vt_span_m=float(vt_span_m),
            vt_taper_ratio=float(vt_taper_ratio),
            vt_c_root_m=float(vt_c_root_m),
            vt_c_MGC_m=float(vt_c_MGC_m),
            vt_max_thickness_fraction=float(vt_max_thickness_fraction),
            vertical_tail_structure=vertical_tail_structure,
            inputs=inputs,
            rudder_chord_ratio=rudder_chord_ratio,
            vt_CM_abs_for_torsion=vt_CM_abs_for_torsion,
        )
        out.update({k: float(v) for k, v in struct.items()})

        if struct.get("ok_struct", 1.0) < 0.5:
            out["feasible"] = 0.0
            out["reason"] = "vertical_tail_structure_limit_bending" if struct.get("ok_bending", 1.0) < 0.5 else "vertical_tail_structure_limit_torsion"

    return out


# ============================================================
# Spin recovery capability (NOW includes OPTIONAL structural check)
# ============================================================
def estimate_Izz_simple(
    *,
    mass_kg: float,
    wing_span_m: float,
    fuselage_length_m: float,
) -> float:
    m = float(mass_kg)
    b = float(wing_span_m)
    L = float(fuselage_length_m)
    return (1.0 / 12.0) * m * (b * b + L * L)


def rudder_spin_recovery_capability(
    *,
    V_spin_mps: float,
    wing_S_m2: float,
    wing_span_m: float,
    Cn_deltaR_per_rad_spin: float,
    inputs: DesignInputs,
    mass_kg: float,
    fuselage_length_m: float,
    spin_rate_rev_per_s: float = 1.0,
    t_recover_s: float = 2.0,
    Izz_kgm2: Optional[float] = None,
    # ----- OPTIONAL: geometry/arm for structural check
    l_v_m: Optional[float] = None,
    vertical_tail_structure: Optional[Any] = None,
    vertical_tail_S_m2: Optional[float] = None,
    vt_span_m: Optional[float] = None,
    vt_taper_ratio: Optional[float] = None,
    vt_c_root_m: Optional[float] = None,
    vt_c_MGC_m: Optional[float] = None,
    vt_max_thickness_fraction: Optional[float] = None,
    vt_CM_abs_for_torsion: Optional[float] = None,
) -> Dict[str, float]:
    if Izz_kgm2 is None:
        Izz_kgm2 = estimate_Izz_simple(mass_kg=mass_kg, wing_span_m=wing_span_m, fuselage_length_m=fuselage_length_m)

    q = 0.5 * ATMOSPHERIC.rho * float(V_spin_mps) ** 2
    S = float(wing_S_m2)
    b = float(wing_span_m)

    if abs(float(Cn_deltaR_per_rad_spin)) < 1e-12:
        raise ValueError("Cn_deltaR_spin is ~0; no modeled spin-recovery authority.")

    r0 = 2.0 * math.pi * float(spin_rate_rev_per_s)  # rad/s
    t = max(1e-6, float(t_recover_s))
    rdot_req = r0 / t

    N_req = float(Izz_kgm2) * float(rdot_req)

    deltaR_req_rad = N_req / (q * S * b * float(Cn_deltaR_per_rad_spin))
    deltaR_req_deg = math.degrees(deltaR_req_rad)

    dR_left = float(getattr(inputs, "vertical_tail_rudder_max_left_deg", -25.0))
    dR_right = float(getattr(inputs, "vertical_tail_rudder_max_right_deg", 25.0))
    feasible = (dR_left <= deltaR_req_deg <= dR_right)
    reason = "" if feasible else "rudder_deflection_limit"

    dR_max_abs_rad = math.radians(max(abs(dR_left), abs(dR_right)))
    N_max_abs = q * S * b * abs(float(Cn_deltaR_per_rad_spin)) * dR_max_abs_rad
    rdot_max = N_max_abs / max(1e-9, float(Izz_kgm2))
    t_stop_best = r0 / max(1e-9, rdot_max)

    out: Dict[str, Any] = {
        "V_spin_mps": float(V_spin_mps),
        "q_Pa": float(q),
        "mass_kg": float(mass_kg),
        "Izz_kgm2": float(Izz_kgm2),
        "spin_rate_rev_per_s": float(spin_rate_rev_per_s),
        "r0_radps": float(r0),
        "t_recover_s": float(t_recover_s),
        "rdot_req_radps2": float(rdot_req),
        "N_req_Nm": float(N_req),
        "deltaR_req_deg": float(deltaR_req_deg),
        "deltaR_limits_deg": (float(dR_left), float(dR_right)),
        "rdot_max_radps2": float(rdot_max),
        "t_stop_best_s": float(t_stop_best),
        "feasible": 1.0 if feasible else 0.0,
        "reason": reason,
        "structural_check_used": 0.0,
        "ok_struct": 1.0,
    }

    # OPTIONAL structural feasibility check
    if (
        vertical_tail_structure is not None
        and l_v_m is not None
        and vertical_tail_S_m2 is not None
        and vt_span_m is not None
        and vt_taper_ratio is not None
        and vt_c_root_m is not None
        and vt_c_MGC_m is not None
        and vt_max_thickness_fraction is not None
    ):
        rudder_chord_ratio = float(getattr(inputs, "vertical_tail_rudder_chord_ratio", 0.30))
        struct = _vt_structural_check_from_yaw_moment(
            N_req_Nm=float(N_req),
            V_mps=float(V_spin_mps),
            wing_S_m2=float(wing_S_m2),
            wing_span_m=float(wing_span_m),
            l_v_m=float(l_v_m),
            vertical_tail_S_m2=float(vertical_tail_S_m2),
            vt_span_m=float(vt_span_m),
            vt_taper_ratio=float(vt_taper_ratio),
            vt_c_root_m=float(vt_c_root_m),
            vt_c_MGC_m=float(vt_c_MGC_m),
            vt_max_thickness_fraction=float(vt_max_thickness_fraction),
            vertical_tail_structure=vertical_tail_structure,
            inputs=inputs,
            rudder_chord_ratio=rudder_chord_ratio,
            vt_CM_abs_for_torsion=vt_CM_abs_for_torsion,
        )
        out.update({k: float(v) for k, v in struct.items()})
        if struct.get("ok_struct", 1.0) < 0.5:
            out["feasible"] = 0.0
            out["reason"] = "vertical_tail_structure_limit_bending" if struct.get("ok_bending", 1.0) < 0.5 else "vertical_tail_structure_limit_torsion"

    return out


# ============================================================
# Coordinated-turn “what can rudder do?” (unchanged except
# authority call can be structurally limited now)
# ============================================================
def rudder_coordinated_turn_characteristics(
    *,
    V_mps: float,
    wing_S_m2: float,
    wing_span_m: float,
    Cn_deltaR_per_rad: float,
    Cn_beta_per_rad: Optional[float],
    inputs: DesignInputs,
    # pass-through optionals to enable structural limiting in rudder_max_yaw_moment
    l_v_m: Optional[float] = None,
    vertical_tail_structure: Optional[Any] = None,
    vertical_tail_S_m2: Optional[float] = None,
    vt_span_m: Optional[float] = None,
    vt_taper_ratio: Optional[float] = None,
    vt_c_root_m: Optional[float] = None,
    vt_c_MGC_m: Optional[float] = None,
    vt_max_thickness_fraction: Optional[float] = None,
    vt_CM_abs_for_torsion: Optional[float] = None,
) -> Dict[str, float]:
    auth = rudder_max_yaw_moment(
        V_mps=V_mps,
        wing_S_m2=wing_S_m2,
        wing_span_m=wing_span_m,
        Cn_deltaR_per_rad=Cn_deltaR_per_rad,
        inputs=inputs,
        l_v_m=l_v_m,
        vertical_tail_structure=vertical_tail_structure,
        vertical_tail_S_m2=vertical_tail_S_m2,
        vt_span_m=vt_span_m,
        vt_taper_ratio=vt_taper_ratio,
        vt_c_root_m=vt_c_root_m,
        vt_c_MGC_m=vt_c_MGC_m,
        vt_max_thickness_fraction=vt_max_thickness_fraction,
        vt_CM_abs_for_torsion=vt_CM_abs_for_torsion,
    )

    dR_left = float(getattr(inputs, "vertical_tail_rudder_max_left_deg", -25.0))
    dR_right = float(getattr(inputs, "vertical_tail_rudder_max_right_deg", 25.0))
    dR_max_abs_rad = math.radians(max(abs(dR_left), abs(dR_right)))

    out: Dict[str, Any] = dict(auth)

    if Cn_beta_per_rad is None or abs(float(Cn_beta_per_rad)) < 1e-12:
        out.update({
            "Cn_beta_per_rad": float("nan"),
            "beta_trim_max_abs_deg": float("nan"),
            "note": "Cn_beta not provided (or ~0); cannot estimate beta-hold capability.",
        })
        return out

    beta_max_rad = abs(float(Cn_deltaR_per_rad)) * dR_max_abs_rad / abs(float(Cn_beta_per_rad))
    beta_max_deg = math.degrees(beta_max_rad)

    out.update({
        "Cn_beta_per_rad": float(Cn_beta_per_rad),
        "beta_trim_max_abs_deg": float(beta_max_deg),
        "note": "beta_trim_max_abs_deg = max sideslip magnitude you can trim (yaw moment = 0) using max rudder.",
    })
    return out


# ============================================================
# Pretty printer
# ============================================================
def print_rudder_report(title: str, d: Dict[str, Any]) -> None:
    print("\n" + "=" * 72)
    print(title)
    print("=" * 72)
    for k in sorted(d.keys()):
        v = d[k]
        if isinstance(v, float):
            if abs(v) >= 1e4 or (abs(v) > 0 and abs(v) < 1e-3):
                s = f"{v:.6g}"
            else:
                s = f"{v:.6f}"
            print(f"{k:32s}: {s}")
        else:
            print(f"{k:32s}: {v}")


if __name__ == "__main__":
    print(
        "This module is meant to be imported and called from your main script after sizing.\n"
        "Suggested usage from script.py:\n"
        "  deriv = rudder_control_derivatives(...)\n"
        "  auth  = rudder_max_yaw_moment(..., l_v_m=deriv['l_v_m'], vertical_tail_structure=results.vtail.structure, ...)\n"
        "  asym  = rudder_asymmetric_thrust(...)\n"
        "  spin  = rudder_spin_recovery_capability(...)\n"
        "  coord = rudder_coordinated_turn_characteristics(...)\n"
    )
