# maneuvering.py
from __future__ import annotations

import math
from typing import Dict, List, Tuple, Optional, Any

from constants import ATMOSPHERIC
from design_inputs import DesignInputs
from airfoil_naca2410 import interpolate_by_alpha as interpolate_naca2410
from airfoil_naca0012 import interpolate_by_alpha as interpolate_naca0012


# ============================================================
# Helpers
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

    start/end are normalized on HALF-span: 0=root, 1=tip.

    Uses elliptical-ish weighting w(x)=sqrt(1-x^2).
    Effective fraction = integral(w) over [start,end] / integral(w) over [0,1].

    Returns:
      geom_frac: end-start
      eff_frac:  location-weighted effectiveness fraction (0..1)
    """
    s = _clamp(float(start), 0.0, 1.0)
    e = _clamp(float(end), 0.0, 1.0)
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


def _y_mac_trapezoid(span_m: float, taper_ratio: float) -> float:
    """
    y_MAC from centerline for a trapezoid planform:
      y_MAC = (b/6) * (1 + 2*lam) / (1 + lam)
    """
    b = float(span_m)
    lam = float(taper_ratio)
    if b <= 0.0:
        return 0.0
    return (b / 6.0) * (1.0 + 2.0 * lam) / max(1e-12, (1.0 + lam))


# ============================================================
# Elevator effectiveness tau_e
# ============================================================
def elevator_tau_e(ce_over_ch: float, inputs: DesignInputs) -> float:
    """
    Approximates typical plain-elevator effectiveness tau_e(ce/ch).
    If you want exactness, set inputs.tail_elevator_tau_override > 0.
    """
    if getattr(inputs, "tail_elevator_tau_override", -1.0) and inputs.tail_elevator_tau_override > 0.0:
        return float(inputs.tail_elevator_tau_override)

    x = _clamp(float(ce_over_ch), 0.05, 0.8)

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
# Tail stall angle reduction due to elevator deflection
# (Table 12.19 encoded for bilinear interpolation)
# ============================================================
_CE_GRID = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
_DE_GRID = [0, 5, 10, 15, 20, 25, 30]
_DA_TABLE = {
    0:  [0, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
    5:  [0, 0.3, 0.5, 1.1, 1.6, 2.2, 2.7, 3.3, 3.9, 4.4, 5],
    10: [0, 0.6, 1.0, 2.1, 3.2, 4.4, 5.5, 6.6, 7.7, 8.9, 10],
    15: [0, 0.9, 1.5, 3.2, 4.9, 6.5, 8.2, 9.9, 11.6, 13.3, 15],
    20: [0, 1.2, 2.0, 4.2, 6.5, 8.7, 11.0, 13.2, 15.5, 17.7, 20],
    25: [0, 1.6, 2.5, 5.3, 8.1, 11.0, 13.7, 16.5, 19.4, 22.2, 25],
    30: [0, 1.9, 3.0, 6.4, 9.7, 13.1, 16.5, 19.9, 23.2, 26.6, 30],
}


def tail_stall_reduction_deg(delta_e_deg_abs: float, ce_over_ch: float) -> float:
    de = _clamp(float(delta_e_deg_abs), 0.0, 30.0)
    ce = _clamp(float(ce_over_ch), 0.0, 1.0)

    # Find bounding de rows
    for i in range(len(_DE_GRID) - 1):
        de0 = _DE_GRID[i]
        de1 = _DE_GRID[i + 1]
        if de0 <= de <= de1:
            break
    else:
        de0 = _DE_GRID[-2]
        de1 = _DE_GRID[-1]

    # Find bounding ce cols
    for j in range(len(_CE_GRID) - 1):
        ce0 = _CE_GRID[j]
        ce1 = _CE_GRID[j + 1]
        if ce0 <= ce <= ce1:
            break
    else:
        ce0 = _CE_GRID[-2]
        ce1 = _CE_GRID[-1]

    t = (de - de0) / (de1 - de0) if de1 != de0 else 0.0
    u = (ce - ce0) / (ce1 - ce0) if ce1 != ce0 else 0.0

    z00 = _DA_TABLE[de0][j]
    z01 = _DA_TABLE[de0][j + 1]
    z10 = _DA_TABLE[de1][j]
    z11 = _DA_TABLE[de1][j + 1]

    z0 = z00 + u * (z01 - z00)
    z1 = z10 + u * (z11 - z10)
    return z0 + t * (z1 - z0)


# ============================================================
# STRUCTURAL LIMIT CHECKS (HORIZONTAL TAIL)
#   - Strength-like checks using the ALREADY-SIZED spar_area_root and t_skin_root
#   - Mirrors your structures.calculate_surface_structure sizing relations:
#       M_cap ≈ sigma_cf_allow * A_spar * h
#       T_cap ≈ 2 * A_cell * tau_allow * t_skin
#   - Demand during maneuver:
#       M_req ≈ (n * |L_tail_req| / 2) * y_MAC
#       T_req ≈ 0.5 * q * S * c_MGC * |CM_tail(alpha_equiv)|
# ============================================================
def _tail_structural_capacity(
    *,
    tail_structure: Any,
    tail_c_root_m: float,
    tail_taper_ratio: float,
    tail_max_thickness_fraction: float,
    elevator_chord_ratio: float,
    inputs: DesignInputs,
) -> Dict[str, float]:
    spar_area_root = _get_attr(tail_structure, "spar_area_root_m2", 0.0)
    t_skin_root = _get_attr(tail_structure, "t_skin_root_m", 0.0)

    # Airfoil thickness used by your structures sizing
    h = float(tail_c_root_m) * float(tail_max_thickness_fraction)
    h = max(h, 1e-9)

    # Bending capacity
    cf_sf = inputs.carbon_fiber_safety_factor
    sigma_allow_cf = float(inputs.carbon_fiber_max_bending_stress_MPa) * 1e6 / max(1.0, cf_sf)
    M_bend_cap = sigma_allow_cf * spar_area_root * h

    # Torsion capacity (same cell area model as structures.py)
    c_cell_root = float(tail_c_root_m) * float(elevator_chord_ratio)
    c_cell_root = max(c_cell_root, 1e-9)
    A_cell_root = 2.0 * c_cell_root * h / 3.0

    tau_allow = float(inputs.filament_shear_strength_MPa) * 1e6 / max(1.0, float(inputs.printed_parts_sf))
    T_cap = 2.0 * A_cell_root * tau_allow * t_skin_root

    return {
        "h_m": h,
        "spar_area_root_m2": float(spar_area_root),
        "t_skin_root_m": float(t_skin_root),
        "M_bend_cap_Nm": float(M_bend_cap),
        "T_cap_Nm": float(T_cap),
    }


def _tail_structural_check(
    *,
    tail_structure: Any,
    n_for_check: float,
    L_tail_req_N: float,
    V_mps: float,
    tail_span_m: float,
    tail_S_m2: float,
    tail_c_root_m: float,
    tail_taper_ratio: float,
    tail_c_MGC_m: float,
    tail_max_thickness_fraction: float,
    tail_CM_at_condition: float,
    inputs: DesignInputs,
    elevator_chord_ratio: float,
) -> Dict[str, float]:
    if tail_structure is None:
        return {"structural_check_used": 0.0}

    cap = _tail_structural_capacity(
        tail_structure=tail_structure,
        tail_c_root_m=tail_c_root_m,
        tail_taper_ratio=tail_taper_ratio,
        tail_max_thickness_fraction=tail_max_thickness_fraction,
        elevator_chord_ratio=elevator_chord_ratio,
        inputs=inputs,
    )

    # Bending demand (mirror structures.py, but replace inputs.n_ult with current n)
    y_mac = _y_mac_trapezoid(span_m=tail_span_m, taper_ratio=tail_taper_ratio)
    shear = float(n_for_check) * abs(float(L_tail_req_N)) / 2.0
    M_req = shear * y_mac

    # Torsion demand (use maneuver dynamic pressure; capacity already sized on dive_speed in structures.py)
    q = 0.5 * ATMOSPHERIC.rho * float(V_mps) ** 2
    T_req = 0.5 * q * float(tail_S_m2) * float(tail_c_MGC_m) * abs(float(tail_CM_at_condition))

    M_cap = float(cap["M_bend_cap_Nm"])
    T_cap = float(cap["T_cap_Nm"])

    margin_b = M_cap / max(1e-9, M_req)
    margin_t = T_cap / max(1e-9, T_req)

    ok_b = 1.0 if (M_req <= M_cap * 1.000) else 0.0
    ok_t = 1.0 if (T_req <= T_cap * 1.000) else 0.0
    ok = 1.0 if (ok_b > 0.5 and ok_t > 0.5) else 0.0

    return {
        "structural_check_used": 1.0,
        "M_req_Nm": float(M_req),
        "T_req_Nm": float(T_req),
        "M_cap_Nm": float(M_cap),
        "T_cap_Nm": float(T_cap),
        "margin_bending": float(margin_b),
        "margin_torsion": float(margin_t),
        "ok_bending": float(ok_b),
        "ok_torsion": float(ok_t),
        "ok_struct": float(ok),
        "y_MAC_m": float(y_mac),
        "tail_CM_used": float(tail_CM_at_condition),
        # echo sized values for debugging
        "spar_area_root_m2": float(cap["spar_area_root_m2"]),
        "t_skin_root_m": float(cap["t_skin_root_m"]),
        "h_m": float(cap["h_m"]),
    }


# ============================================================
# Elevator control derivatives (maneuver-only)
# ============================================================
def elevator_control_derivatives(
    *,
    wing_S_m2: float,
    wing_MAC_m: float,
    tail_S_m2: float,
    tail_AR: float,
    x_cg_m: float,
    x_ac_tail_m: float,
    inputs: DesignInputs,
) -> Dict[str, float]:
    CL_alpha_2D = 2.0 * math.pi
    CL_alpha_h = CL_alpha_2D / (1.0 + CL_alpha_2D / (math.pi * float(inputs.tail_e_oswald) * float(tail_AR)))

    l_h = float(x_ac_tail_m) - float(x_cg_m)
    if abs(l_h) < 1e-9:
        raise ValueError("Tail moment arm too small for elevator derivative calculation.")

    V_H = (float(tail_S_m2) * l_h) / (float(wing_S_m2) * float(wing_MAC_m))

    ce_ch = float(inputs.tail_elevator_chord_ratio)
    tau_e = elevator_tau_e(ce_ch, inputs)

    seg = _control_surface_effective_span_fraction(
        float(getattr(inputs, "tail_elevator_span_start", 0.0)),
        float(getattr(inputs, "tail_elevator_span_end", 1.0)),
    )
    bE_bh_geom = seg["geom_frac"]
    bE_bh_eff = seg["eff_frac"]

    eta_h = float(getattr(inputs, "tail_eta_h_maneuver", 1.0))

    Cm_deltaE = -CL_alpha_h * eta_h * V_H * bE_bh_eff * tau_e
    CL_deltaE = CL_alpha_h * eta_h * (float(tail_S_m2) / float(wing_S_m2)) * bE_bh_eff * tau_e
    CLh_deltaE = CL_alpha_h * tau_e

    return {
        "CL_alpha_h_per_rad": float(CL_alpha_h),
        "tau_e": float(tau_e),
        "bE_over_bh_geom": float(bE_bh_geom),
        "bE_over_bh_eff": float(bE_bh_eff),
        "eta_h": float(eta_h),
        "l_h_m": float(l_h),
        "V_H": float(V_H),
        "Cm_deltaE_per_rad": float(Cm_deltaE),
        "CL_deltaE_per_rad": float(CL_deltaE),
        "CLh_deltaE_per_rad": float(CLh_deltaE),
    }


# ============================================================
# Maneuver feasibility at target load factor n (steady trimmed pull/turn)
#   - Solves wing alpha for total lift = n*W
#   - Then checks elevator trim, tail CLmax, tail stall-with-elevator
#   - OPTIONAL: structural check for horizontal tail (bending + torsion)
# ============================================================
def elevator_feasible_for_load_factor(
    *,
    n: float,
    V_mps: float,
    W0_N: float,
    wing_S_m2: float,
    wing_MAC_m: float,
    wing_AR: float,
    wing_CL_alpha_per_rad: float,
    wing_CL_max: float,
    x_ac_wing_m: float,
    x_cg_m: float,
    tail_S_m2: float,
    tail_MAC_m: float,
    tail_AR: float,
    tail_CL_max: float,
    tail_incidence_deg: float,
    x_ac_tail_m: float,
    inputs: DesignInputs,
    # -------- OPTIONAL STRUCTURAL DATA (if provided, we enforce it) ----------
    tail_structure: Optional[Any] = None,
    tail_span_m: Optional[float] = None,
    tail_c_root_m: Optional[float] = None,
    tail_taper_ratio: Optional[float] = None,
    tail_c_MGC_m: Optional[float] = None,
    tail_max_thickness_fraction: Optional[float] = None,
) -> Dict[str, float]:
    q = 0.5 * ATMOSPHERIC.rho * float(V_mps) ** 2
    if q <= 0.0:
        raise ValueError("Invalid dynamic pressure (check speed).")

    alpha_L0_w_deg = float(inputs.NACA2410_zero_lift_angle_of_attack_deg)

    arm = float(x_ac_tail_m) - float(x_cg_m)
    if abs(arm) < 1e-9:
        raise ValueError("Tail moment arm too small or zero.")

    L_target = float(n) * float(W0_N)

    # Upper alpha bound based on CLmax (linear estimate)
    alpha_stall_w_deg = alpha_L0_w_deg + math.degrees(float(wing_CL_max) / max(1e-9, float(wing_CL_alpha_per_rad)))
    alpha_lo = -5.0
    alpha_hi = min(25.0, alpha_stall_w_deg)

    def _lift_residual(alpha_w_deg: float) -> float:
        CL_w = float(wing_CL_alpha_per_rad) * math.radians(alpha_w_deg - alpha_L0_w_deg)
        if CL_w > float(wing_CL_max) * 1.001:
            return -1e9

        L_w = q * float(wing_S_m2) * CL_w

        CM_w = float(interpolate_naca2410(alpha_w_deg)["CM"])
        M_ac_w = CM_w * q * float(wing_S_m2) * float(wing_MAC_m)

        M_cg_w = M_ac_w + L_w * math.cos(math.radians(alpha_w_deg)) * (float(x_cg_m) - float(x_ac_wing_m))

        L_tail_req = M_cg_w / arm
        L_total = L_w + L_tail_req
        return L_total - L_target

    f_lo = _lift_residual(alpha_lo)
    f_hi = _lift_residual(alpha_hi)

    if f_hi < 0.0:
        return {"feasible": 0.0, "reason": "insufficient_lift_at_wing_CLmax", "n": float(n)}

    # Bisection for alpha
    if f_lo > 0.0:
        alpha_sol = alpha_lo
    else:
        a, b = alpha_lo, alpha_hi
        fa, fb = f_lo, f_hi
        alpha_sol = b
        for _ in range(60):
            m = 0.5 * (a + b)
            fm = _lift_residual(m)
            if abs(fm) < 1e-3:
                alpha_sol = m
                break
            if fm > 0.0:
                b, fb = m, fm
            else:
                a, fa = m, fm
            alpha_sol = 0.5 * (a + b)

    # Recompute at solution
    alpha_w_deg = float(alpha_sol)
    CL_w = float(wing_CL_alpha_per_rad) * math.radians(alpha_w_deg - alpha_L0_w_deg)
    L_w = q * float(wing_S_m2) * CL_w

    CM_w = float(interpolate_naca2410(alpha_w_deg)["CM"])
    M_ac_w = CM_w * q * float(wing_S_m2) * float(wing_MAC_m)
    M_cg_w = M_ac_w + L_w * math.cos(math.radians(alpha_w_deg)) * (float(x_cg_m) - float(x_ac_wing_m))
    L_tail_req = M_cg_w / arm

    # Tail CL required
    eta_h = float(getattr(inputs, "tail_eta_h_maneuver", 1.0))
    CL_tail_req = L_tail_req / (q * float(tail_S_m2) * max(1e-9, eta_h))

    # Tail lift curve slope (3D)
    CL_alpha_2D = 2.0 * math.pi
    CL_alpha_h = CL_alpha_2D / (1.0 + CL_alpha_2D / (math.pi * float(inputs.tail_e_oswald) * float(tail_AR)))

    # Downwash model
    downwash = float(inputs.tail_downwash_offset_deg) + float(inputs.tail_downwash_k) * alpha_w_deg
    downwash = _clamp(downwash, float(inputs.tail_downwash_min_deg), float(inputs.tail_downwash_max_deg))

    alpha_L0_tail = float(inputs.NACA0012_zero_lift_angle_of_attack_deg)
    alpha_tail_geom = alpha_w_deg - downwash

    # Elevator effectiveness (tau_e * effective span fraction)
    tau_e = elevator_tau_e(float(inputs.tail_elevator_chord_ratio), inputs)
    seg = _control_surface_effective_span_fraction(
        float(getattr(inputs, "tail_elevator_span_start", 0.0)),
        float(getattr(inputs, "tail_elevator_span_end", 1.0)),
    )
    tau_eff = tau_e * seg["eff_frac"]

    if abs(tau_eff) < 1e-9:
        return {"feasible": 0.0, "reason": "tau_eff_too_small", "n": float(n)}

    # Solve delta_e for required tail CL (linear)
    required_alpha_term_deg = alpha_L0_tail + math.degrees(CL_tail_req / max(1e-9, CL_alpha_h))
    delta_e_deg = (required_alpha_term_deg - (alpha_tail_geom + float(tail_incidence_deg))) / tau_eff

    # Deflection limits
    de_up = float(getattr(inputs, "tail_elevator_max_up_deg", -25.0))
    de_dn = float(getattr(inputs, "tail_elevator_max_down_deg", 20.0))
    if not (de_up <= delta_e_deg <= de_dn):
        return {
            "feasible": 0.0,
            "reason": "elevator_deflection_limit",
            "n": float(n),
            "delta_e_deg": float(delta_e_deg),
            "delta_e_limits": (de_up, de_dn),
        }

    # Tail CLmax check
    if abs(CL_tail_req) > float(tail_CL_max) * 1.001:
        return {
            "feasible": 0.0,
            "reason": "tail_CLmax_limit",
            "n": float(n),
            "CL_tail_req": float(CL_tail_req),
            "CL_tail_max": float(tail_CL_max),
        }

    # Tail stall margin with elevator deflection
    alpha_stall_tail = alpha_L0_tail + math.degrees(float(tail_CL_max) / max(1e-9, CL_alpha_h))
    red = tail_stall_reduction_deg(abs(delta_e_deg), float(inputs.tail_elevator_chord_ratio))
    alpha_stall_eff = alpha_stall_tail - red

    alpha_equiv = alpha_tail_geom + float(tail_incidence_deg) + tau_eff * float(delta_e_deg)
    if abs(alpha_equiv - alpha_L0_tail) > abs(alpha_stall_eff - alpha_L0_tail) * 1.000:
        return {
            "feasible": 0.0,
            "reason": "tail_stall_with_elevator",
            "n": float(n),
            "alpha_equiv_deg": float(alpha_equiv),
            "alpha_stall_eff_deg": float(alpha_stall_eff),
            "stall_reduction_deg": float(red),
        }

    # ============================================================
    # OPTIONAL: STRUCTURAL CHECK (horizontal tail)
    # ============================================================
    struct_info: Dict[str, float] = {"structural_check_used": 0.0}
    if tail_structure is not None:
        b_tail = float(tail_span_m) if tail_span_m is not None else 0.0
        c_root = float(tail_c_root_m) if tail_c_root_m is not None else float(getattr(inputs, "tail_c_root", 0.0))
        lam = float(tail_taper_ratio) if tail_taper_ratio is not None else float(getattr(inputs, "tail_taper_ratio", 1.0))

        # c_MGC: best is to pass it in; else compute from S/b if possible; else use trapezoid mean chord
        if tail_c_MGC_m is not None and float(tail_c_MGC_m) > 0.0:
            c_mgc = float(tail_c_MGC_m)
        else:
            if b_tail > 0.0 and float(tail_S_m2) > 0.0:
                c_mgc = float(tail_S_m2) / b_tail
            else:
                c_mgc = c_root * (1.0 + lam) / 2.0

        tfrac = float(tail_max_thickness_fraction) if tail_max_thickness_fraction is not None else 0.0

        # Only run the check if we have enough geometric info
        if (b_tail > 0.0) and (c_root > 0.0) and (c_mgc > 0.0) and (tfrac > 0.0):
            CM_tail = float(interpolate_naca0012(alpha_equiv)["CM"])

            struct_info = _tail_structural_check(
                tail_structure=tail_structure,
                n_for_check=float(n),
                L_tail_req_N=float(L_tail_req),
                V_mps=float(V_mps),
                tail_span_m=float(b_tail),
                tail_S_m2=float(tail_S_m2),
                tail_c_root_m=float(c_root),
                tail_taper_ratio=float(lam),
                tail_c_MGC_m=float(c_mgc),
                tail_max_thickness_fraction=float(tfrac),
                tail_CM_at_condition=float(CM_tail),
                inputs=inputs,
                elevator_chord_ratio=float(inputs.tail_elevator_chord_ratio),
            )

            if struct_info.get("ok_struct", 1.0) < 0.5:
                reason = "tail_structure_limit_bending" if struct_info.get("ok_bending", 1.0) < 0.5 else "tail_structure_limit_torsion"
                return {
                    "feasible": 0.0,
                    "reason": reason,
                    "n": float(n),
                    "V_mps": float(V_mps),
                    "alpha_w_deg": float(alpha_w_deg),
                    "CL_w": float(CL_w),
                    "L_w_N": float(L_w),
                    "M_ac_w_Nm": float(M_ac_w),
                    "M_cg_w_Nm": float(M_cg_w),
                    "L_tail_req_N": float(L_tail_req),
                    "CL_tail_req": float(CL_tail_req),
                    "delta_e_deg": float(delta_e_deg),
                    "downwash_deg": float(downwash),
                    "alpha_tail_geom_deg": float(alpha_tail_geom),
                    "alpha_equiv_tail_deg": float(alpha_equiv),
                    "alpha_stall_eff_tail_deg": float(alpha_stall_eff),
                    # structure detail
                    **{k: float(v) for k, v in struct_info.items()},
                }

    # Feasible (include structural echoes if computed)
    out = {
        "feasible": 1.0,
        "n": float(n),
        "V_mps": float(V_mps),
        "alpha_w_deg": float(alpha_w_deg),
        "CL_w": float(CL_w),
        "L_w_N": float(L_w),
        "M_ac_w_Nm": float(M_ac_w),
        "M_cg_w_Nm": float(M_cg_w),
        "L_tail_req_N": float(L_tail_req),
        "CL_tail_req": float(CL_tail_req),
        "delta_e_deg": float(delta_e_deg),
        "downwash_deg": float(downwash),
        "alpha_tail_geom_deg": float(alpha_tail_geom),
        "alpha_equiv_tail_deg": float(alpha_equiv),
        "alpha_stall_eff_tail_deg": float(alpha_stall_eff),
    }
    # attach structural results (or structural_check_used=0.0)
    out.update({k: float(v) for k, v in struct_info.items()})
    return out


def max_load_factor_from_elevator(
    *,
    V_mps: float,
    W0_N: float,
    wing_S_m2: float,
    wing_MAC_m: float,
    wing_AR: float,
    wing_CL_alpha_per_rad: float,
    wing_CL_max: float,
    x_ac_wing_m: float,
    x_cg_m: float,
    tail_S_m2: float,
    tail_MAC_m: float,
    tail_AR: float,
    tail_CL_max: float,
    tail_incidence_deg: float,
    x_ac_tail_m: float,
    inputs: DesignInputs,
    n_hi: Optional[float] = None,
    # -------- OPTIONAL STRUCTURAL DATA (if provided, we enforce it) ----------
    tail_structure: Optional[Any] = None,
    tail_span_m: Optional[float] = None,
    tail_c_root_m: Optional[float] = None,
    tail_taper_ratio: Optional[float] = None,
    tail_c_MGC_m: Optional[float] = None,
    tail_max_thickness_fraction: Optional[float] = None,
) -> Dict[str, float]:
    # Search up to your ultimate factor (or user override)
    n_upper = float(n_hi) if n_hi is not None else float(inputs.n_ult)

    # If n=1 is not feasible, return early
    base = elevator_feasible_for_load_factor(
        n=1.0,
        V_mps=V_mps,
        W0_N=W0_N,
        wing_S_m2=wing_S_m2,
        wing_MAC_m=wing_MAC_m,
        wing_AR=wing_AR,
        wing_CL_alpha_per_rad=wing_CL_alpha_per_rad,
        wing_CL_max=wing_CL_max,
        x_ac_wing_m=x_ac_wing_m,
        x_cg_m=x_cg_m,
        tail_S_m2=tail_S_m2,
        tail_MAC_m=tail_MAC_m,
        tail_AR=tail_AR,
        tail_CL_max=tail_CL_max,
        tail_incidence_deg=tail_incidence_deg,
        x_ac_tail_m=x_ac_tail_m,
        inputs=inputs,
        tail_structure=tail_structure,
        tail_span_m=tail_span_m,
        tail_c_root_m=tail_c_root_m,
        tail_taper_ratio=tail_taper_ratio,
        tail_c_MGC_m=tail_c_MGC_m,
        tail_max_thickness_fraction=tail_max_thickness_fraction,
    )
    if base.get("feasible", 0.0) < 0.5:
        base["n_max"] = 0.0
        return base

    lo, hi = 1.0, n_upper
    best = dict(base)

    for _ in range(40):
        mid = 0.5 * (lo + hi)
        r = elevator_feasible_for_load_factor(
            n=mid,
            V_mps=V_mps,
            W0_N=W0_N,
            wing_S_m2=wing_S_m2,
            wing_MAC_m=wing_MAC_m,
            wing_AR=wing_AR,
            wing_CL_alpha_per_rad=wing_CL_alpha_per_rad,
            wing_CL_max=wing_CL_max,
            x_ac_wing_m=x_ac_wing_m,
            x_cg_m=x_cg_m,
            tail_S_m2=tail_S_m2,
            tail_MAC_m=tail_MAC_m,
            tail_AR=tail_AR,
            tail_CL_max=tail_CL_max,
            tail_incidence_deg=tail_incidence_deg,
            x_ac_tail_m=x_ac_tail_m,
            inputs=inputs,
            tail_structure=tail_structure,
            tail_span_m=tail_span_m,
            tail_c_root_m=tail_c_root_m,
            tail_taper_ratio=tail_taper_ratio,
            tail_c_MGC_m=tail_c_MGC_m,
            tail_max_thickness_fraction=tail_max_thickness_fraction,
        )
        if r.get("feasible", 0.0) > 0.5:
            best = r
            lo = mid
        else:
            hi = mid

    n_max = float(best.get("n", lo))

    # Equivalent max bank angle for level turn: n = 1/cos(phi)
    phi_max_deg = 0.0
    if n_max > 1.0:
        phi_max_deg = math.degrees(math.acos(1.0 / n_max))

    best["n_max"] = float(n_max)
    best["phi_max_deg_level_turn"] = float(phi_max_deg)
    return best
