"""
Aerodynamic calculations for the twin-boom lift-plus-cruise VTOL.

Step 5 scope:
- Preserves the existing main-wing geometry and NACA 2410 polar model.
- Retains NACA 0012 for horizontal and vertical tail calculations.
- Adds a configuration-level cruise-drag build-up for the central pod, two booms,
  stopped lift propellers, motor nacelles/mounts, cooling flow, and interference.
- Treats vertical-tail geometry as one fin; total twin-fin area is accounted for
  using inputs.n_vertical_tail_surfaces.

The component drag values are conceptual estimates and are deliberately exposed
through DesignInputs so they can later be replaced with CFD, wind-tunnel, CAD,
or flight-test data without changing the model structure.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

from constants import ATMOSPHERIC
from design_inputs import DesignInputs
from airfoil_naca2410 import interpolate_by_alpha as interpolate_naca2410
from airfoil_naca0012 import interpolate_by_alpha as interpolate_naca0012


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _finite_wing_lift_slope(a0_per_rad: float, AR: float, e: float) -> float:
    """Prandtl finite-wing lift-curve slope [1/rad]."""
    return a0_per_rad / (1.0 + a0_per_rad / (math.pi * max(e, 1e-6) * max(AR, 1e-6)))


def _polar_lift_slope_per_rad(interpolator, alpha0_deg: float) -> float:
    """Estimate local 2-D lift slope from the supplied polar around alpha0."""
    p_lo = interpolator(alpha0_deg - 1.0)
    p_hi = interpolator(alpha0_deg + 1.0)
    slope_per_deg = (float(p_hi["CL"]) - float(p_lo["CL"])) / 2.0
    return max(1e-6, slope_per_deg * 180.0 / math.pi)


def _alpha_for_required_cl(
    CL_required: float,
    alpha_zero_lift_deg: float,
    CL_alpha_3D_per_rad: float,
) -> float:
    return float(alpha_zero_lift_deg) + math.degrees(float(CL_required) / max(CL_alpha_3D_per_rad, 1e-9))


def _profile_and_induced_drag(
    CL: float,
    alpha_deg: float,
    AR: float,
    e: float,
    interpolator,
) -> Dict[str, float]:
    polar = interpolator(alpha_deg)
    CD_profile = max(0.0, float(polar["CD"]))
    CD_induced = float(CL) ** 2 / (math.pi * max(e, 1e-6) * max(AR, 1e-6))
    return {
        "CD_profile": CD_profile,
        "CD_induced": CD_induced,
        "CD_total": CD_profile + CD_induced,
        "CM": float(polar["CM"]),
        "CM_max": max(abs(float(polar.get("CM_max", polar["CM"]))), abs(float(polar["CM"]))),
        "polar": polar,
    }


def wing_aerodynamic_coeffs(
    L_wing_N: float,
    L_tail_N: float,
    S: float,
    MAC: float,
    AR: float,
    inputs: DesignInputs,
) -> Dict[str, float]:
    """Main-wing aerodynamics using the existing NACA 2410 polar.

    The main-wing planform inputs are intentionally unchanged in Step 5. The
    configuration drag of the pod, booms and stopped VTOL propellers is added
    separately by configuration_cruise_drag().
    """
    del L_tail_N  # retained for API compatibility

    rho = ATMOSPHERIC.rho
    V = float(inputs.speed)
    q = 0.5 * rho * V**2
    CL_required = float(L_wing_N) / max(q * float(S), 1e-9)

    alpha0 = float(inputs.NACA2410_zero_lift_angle_of_attack_deg)
    a0 = _polar_lift_slope_per_rad(interpolate_naca2410, alpha0)
    a3d = _finite_wing_lift_slope(a0, float(AR), float(inputs.e_oswald))
    alpha = _alpha_for_required_cl(CL_required, alpha0, a3d)

    drag = _profile_and_induced_drag(
        CL=CL_required,
        alpha_deg=alpha,
        AR=AR,
        e=float(inputs.e_oswald),
        interpolator=interpolate_naca2410,
    )

    polar_at_alpha0 = interpolate_naca2410(alpha0)
    CL_max_2d = float(drag["polar"].get("CL_max", 1.1))
    CL_max = CL_max_2d * float(getattr(inputs, "wing_CLmax_3D_factor", 0.95))
    V_stall = math.sqrt(
        2.0 * max(float(L_wing_N), 0.0)
        / max(rho * float(S) * max(CL_max, 1e-6), 1e-9)
    )

    Re = rho * V * float(MAC) / max(ATMOSPHERIC.mu, 1e-12)
    L_D = CL_required / max(drag["CD_total"], 1e-9)

    return {
        "CL_cruise": CL_required,
        "CL_max": CL_max,
        "CD0": max(0.0, float(polar_at_alpha0["CD"])),
        "CD_cruise": drag["CD_total"],
        "CD_profile": drag["CD_profile"],
        "CD_induced": drag["CD_induced"],
        "CD0_fuselage": 0.0,
        "CM_cruise": drag["CM"],
        "CM_max": drag["CM_max"],
        "alpha_cruise": alpha,
        "L_over_D_cruise": L_D,
        "V_stall": V_stall,
        "Re_cruise": Re,
        "max_thickness_fraction_of_chord": float(
            drag["polar"].get("max_thickness_fraction_of_chord", 0.10)
        ),
        "CL_alpha_3D_per_rad": a3d,
    }


def _elevator_tau(ce_over_ch: float, inputs: DesignInputs) -> float:
    override = float(getattr(inputs, "tail_elevator_tau_override", -1.0))
    if override > 0.0:
        return override
    x = _clamp(float(ce_over_ch), 0.05, 0.80)
    points = [
        (0.10, 0.20), (0.20, 0.35), (0.30, 0.48), (0.40, 0.60),
        (0.50, 0.68), (0.60, 0.75), (0.70, 0.80), (0.80, 0.84),
    ]
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        if x0 <= x <= x1:
            t = (x - x0) / max(x1 - x0, 1e-9)
            return y0 + t * (y1 - y0)
    return points[0][1] if x < points[0][0] else points[-1][1]


def tail_aerodynamic_coeffs(
    L_required_N: float,
    S: float,
    MAC: float,
    AR: float,
    alpha_body_deg: float,
    downwash_deg: float,
    inputs: DesignInputs,
) -> Dict[str, Any]:
    """Horizontal-tail trim and drag model using the existing NACA 0012 polar."""
    rho = ATMOSPHERIC.rho
    V = float(inputs.speed)
    q = 0.5 * rho * V**2
    CL_req = float(L_required_N) / max(q * float(S), 1e-9)

    alpha0 = float(inputs.NACA0012_zero_lift_angle_of_attack_deg)
    a0 = _polar_lift_slope_per_rad(interpolate_naca0012, alpha0)
    a3d = _finite_wing_lift_slope(a0, float(AR), float(inputs.tail_e_oswald))
    alpha_req = _alpha_for_required_cl(CL_req, alpha0, a3d)

    alpha_clean = float(alpha_body_deg) - float(downwash_deg)
    incidence_required = alpha_req - alpha_clean
    incidence_limit = abs(float(inputs.tail_incidence_limit_deg))
    incidence = _clamp(incidence_required, -incidence_limit, incidence_limit)

    tau_e = _elevator_tau(float(inputs.tail_elevator_chord_ratio), inputs)
    remaining_alpha = alpha_req - (alpha_clean + incidence)
    delta_e = remaining_alpha / max(tau_e, 1e-9)
    elevator_limit = abs(float(inputs.tail_elevator_limit_deg))
    delta_e_limited = _clamp(delta_e, -elevator_limit, elevator_limit)

    alpha_equiv = alpha_clean + incidence + tau_e * delta_e_limited
    CL_no_elev = a3d * math.radians(alpha_clean + incidence - alpha0)
    dCL_required = CL_req - CL_no_elev

    drag = _profile_and_induced_drag(
        CL=CL_req,
        alpha_deg=alpha_equiv,
        AR=AR,
        e=float(inputs.tail_e_oswald),
        interpolator=interpolate_naca0012,
    )
    control_drag = float(getattr(inputs, "control_surface_drag_k", 0.015)) * math.radians(delta_e_limited) ** 2
    CD_total = drag["CD_total"] + control_drag

    polar = drag["polar"]
    CL_max_2d = float(polar.get("CL_max", 1.1))
    CL_max = CL_max_2d * float(getattr(inputs, "tail_CLmax_3D_factor", 0.90))
    V_stall = math.sqrt(
        2.0 * abs(float(L_required_N))
        / max(rho * float(S) * max(CL_max, 1e-6), 1e-9)
    ) if abs(float(L_required_N)) > 1e-9 else 0.0

    Re = rho * V * float(MAC) / max(ATMOSPHERIC.mu, 1e-12)
    feasible = (
        abs(delta_e) <= elevator_limit + 1e-9
        and abs(incidence_required) <= incidence_limit + abs(tau_e * elevator_limit) + 1e-9
        and abs(alpha_equiv) <= float(inputs.tail_effective_alpha_limit_deg) + 1e-9
    )

    elevator_area = float(S) * float(inputs.tail_elevator_chord_ratio)
    elevator_chord = float(MAC) * float(inputs.tail_elevator_chord_ratio)
    Ch_delta = float(getattr(inputs, "elevator_Ch_delta_per_rad", 0.03))
    M_h = q * elevator_area * elevator_chord * Ch_delta * abs(math.radians(float(inputs.tail_elevator_max_deflection_deg)))
    T_servo = M_h * float(inputs.elevator_horn_radius_m) / max(
        float(inputs.servo_efficiency_eta) * float(inputs.servo_arm_radius_m), 1e-9
    )

    failure_reason = ""
    if not feasible:
        failure_reason = (
            f"Required incidence/elevator combination exceeds limits: incidence_required="
            f"{incidence_required:.2f} deg, elevator_required={delta_e:.2f} deg, "
            f"alpha_effective={alpha_equiv:.2f} deg."
        )

    return {
        "feasible": feasible,
        "failure_reason": failure_reason,
        "CL_cruise": CL_req,
        "CL_req": CL_req,
        "CL_no_elev": CL_no_elev,
        "dCL_required": dCL_required,
        "CL_max": CL_max,
        "CD0": float(interpolate_naca0012(alpha0)["CD"]),
        "CD_cruise": CD_total,
        "CD_profile": drag["CD_profile"] + control_drag,
        "CD_induced": drag["CD_induced"],
        "CM_cruise": drag["CM"],
        "CM_max": drag["CM_max"],
        "alpha_body_deg": float(alpha_body_deg),
        "downwash_deg": float(downwash_deg),
        "tail_incidence_deg": incidence,
        "delta_elev_deg": delta_e_limited,
        "alpha_equiv_deg": alpha_equiv,
        "elevator_chord_ratio": float(inputs.tail_elevator_chord_ratio),
        "tau_e": tau_e,
        "L_over_D_cruise": CL_req / max(CD_total, 1e-9),
        "V_stall": V_stall,
        "Re_cruise": Re,
        "max_thickness_fraction_of_chord": float(polar.get("max_thickness_fraction_of_chord", 0.12)),
        "CL_alpha_3D_per_rad": a3d,
        "elevator_hinge_moment_M_h_Nm": M_h,
        "elevator_area_S_e_m2": elevator_area,
        "elevator_chord_c_e_m": elevator_chord,
        "servo_torque_T_servo_Nm": T_servo,
        "servo_torque_with_margin_Nm": T_servo * float(inputs.servo_torque_margin),
    }


def vertical_tail_aerodynamic_coeffs(
    S: float,
    MAC: float,
    AR: float,
    inputs: DesignInputs,
) -> Dict[str, float]:
    """Zero-sideslip aerodynamic properties for one vertical fin."""
    del S
    rho = ATMOSPHERIC.rho
    V = float(inputs.speed)
    alpha0 = float(inputs.NACA0012_zero_lift_angle_of_attack_deg)
    a0 = _polar_lift_slope_per_rad(interpolate_naca0012, alpha0)
    a3d = _finite_wing_lift_slope(a0, float(AR), float(inputs.tail_e_oswald))
    polar = interpolate_naca0012(0.0)
    CL_max = float(polar.get("CL_max", 1.1)) * float(getattr(inputs, "tail_CLmax_3D_factor", 0.90))
    Re = rho * V * float(MAC) / max(ATMOSPHERIC.mu, 1e-12)
    return {
        "CL_cruise": 0.0,
        "CL_max": CL_max,
        "CD0": float(polar["CD"]),
        "CD_cruise": float(polar["CD"]),
        "CD0_fuselage": 0.0,
        "CM_cruise": float(polar["CM"]),
        "CM_max": max(abs(float(polar.get("CM_max", polar["CM"]))), abs(float(polar["CM"]))),
        "alpha_cruise": 0.0,
        "L_over_D_cruise": 0.0,
        "V_stall": 0.0,
        "Re_cruise": Re,
        "max_thickness_fraction_of_chord": float(polar.get("max_thickness_fraction_of_chord", 0.12)),
        "CL_alpha_3D_per_rad": a3d,
    }


def effective_lifting_surface_wetted_area_m2(
    inputs: DesignInputs,
    hardcode_attr: str,
    planform_area_m2: float,
) -> float:
    if bool(getattr(inputs, "use_hardcoded_values", False)):
        value = getattr(inputs, hardcode_attr, None)
        if value is not None:
            return float(value)
    return 2.0 * float(planform_area_m2)


def _skin_friction_coefficient(Re: float, laminar_fraction: float = 0.0) -> float:
    Re = max(float(Re), 1.0e3)
    Cf_lam = 1.328 / math.sqrt(Re)
    Cf_turb = 0.455 / max(math.log10(Re) ** 2.58, 1e-9)
    f_lam = _clamp(float(laminar_fraction), 0.0, 1.0)
    return f_lam * Cf_lam + (1.0 - f_lam) * Cf_turb


def _body_form_factor(length_m: float, equivalent_diameter_m: float) -> float:
    fineness = max(float(length_m) / max(float(equivalent_diameter_m), 1e-6), 1.0)
    return 1.0 + 60.0 / fineness**3 + fineness / 400.0


def _lift_prop_combo(propulsion_selection: Optional[Dict[str, Any]], inputs: DesignInputs) -> Dict[str, Any]:
    if propulsion_selection:
        lift = propulsion_selection.get("lift", {})
        combo = lift.get("combo")
        if combo:
            return combo
    return dict(getattr(inputs, "hardcode_lift_propulsion", {}) or {})


def _lift_prop_drag_area_each_m2(
    propulsion_selection: Optional[Dict[str, Any]],
    inputs: DesignInputs,
) -> float:
    direct = getattr(inputs, "lift_prop_equivalent_drag_area_each_m2", None)
    if direct is not None:
        return max(0.0, float(direct))

    combo = _lift_prop_combo(propulsion_selection, inputs)
    diameter_in = float(combo.get("prop_diameter_in", 15.0))
    radius_m = 0.5 * diameter_in * 0.0254
    n_blades = max(1, int(getattr(inputs, "lift_prop_blade_count", 2)))
    chord_ratio = max(0.005, float(getattr(inputs, "lift_prop_mean_blade_chord_ratio", 0.08)))
    blade_planform_area = n_blades * radius_m * (chord_ratio * 2.0 * radius_m)

    state = str(getattr(inputs, "lift_prop_cruise_state", "stopped_aligned")).strip().lower()
    factors = {
        "folded": 0.05,
        "stopped_aligned": 0.15,
        "stopped_random": 0.50,
        "windmilling": 1.00,
    }
    factor = factors.get(state, float(getattr(inputs, "lift_prop_drag_area_factor", 0.15)))
    return blade_planform_area * max(0.0, factor)


def configuration_cruise_drag(
    *,
    wing_geometry: Any,
    wing_aerodynamics: Any,
    inputs: DesignInputs,
    tail_geometry: Optional[Any] = None,
    tail_aerodynamics: Optional[Any] = None,
    vertical_tail_geometry: Optional[Any] = None,
    vertical_tail_aerodynamics: Optional[Any] = None,
    propulsion_selection: Optional[Dict[str, Any]] = None,
) -> Dict[str, float]:
    """Build up total cruise drag for the high-wing twin-boom VTOL.

    Surface aerodynamic objects contain their own profile and induced drag. The
    remaining components use wetted-area skin-friction or equivalent drag area.
    """
    rho = ATMOSPHERIC.rho
    mu = ATMOSPHERIC.mu
    V = float(inputs.speed)
    q = 0.5 * rho * V**2
    S_ref = max(float(wing_geometry.area_m2), 1e-9)

    D_wing = q * S_ref * float(wing_aerodynamics.CD_cruise)
    wing_induced_CD = float(wing_aerodynamics.CL_cruise) ** 2 / (
        math.pi * max(float(inputs.e_oswald), 1e-6) * max(float(wing_geometry.AR), 1e-6)
    )
    D_induced_wing = q * S_ref * wing_induced_CD

    D_tail = 0.0
    D_induced_tail = 0.0
    if tail_geometry is not None and tail_aerodynamics is not None:
        S_tail = float(tail_geometry.area_m2)
        D_tail = q * S_tail * float(tail_aerodynamics.CD_cruise)
        tail_induced_CD = float(tail_aerodynamics.CL_cruise) ** 2 / (
            math.pi * max(float(inputs.tail_e_oswald), 1e-6) * max(float(tail_geometry.AR), 1e-6)
        )
        D_induced_tail = q * S_tail * tail_induced_CD

    D_vertical_tails = 0.0
    if vertical_tail_geometry is not None and vertical_tail_aerodynamics is not None:
        n_fins = max(1, int(inputs.n_vertical_tail_surfaces))
        D_vertical_tails = (
            q * n_fins * float(vertical_tail_geometry.area_m2) * float(vertical_tail_aerodynamics.CD_cruise)
        )

    roughness_mult = max(0.1, float(getattr(inputs, "printed_surface_drag_multiplier", 1.15)))
    laminar_fraction = float(getattr(inputs, "body_laminar_fraction", 0.05))

    # Rounded central pod.
    L_pod = float(inputs.central_pod_length_m)
    w_pod = float(inputs.central_pod_width_m)
    h_pod = float(inputs.central_pod_height_m)
    Swet_pod = 0.82 * 2.0 * (L_pod * w_pod + L_pod * h_pod + w_pod * h_pod)
    if bool(getattr(inputs, "use_hardcoded_values", False)):
        hard = getattr(inputs, "hardcode_central_pod_wetted_area_m2", None)
        if hard is not None:
            Swet_pod = float(hard)
    d_eq_pod = math.sqrt(max(4.0 * w_pod * h_pod / math.pi, 1e-9))
    Re_pod = rho * V * L_pod / max(mu, 1e-12)
    Cf_pod = _skin_friction_coefficient(Re_pod, laminar_fraction) * roughness_mult
    FF_pod = _body_form_factor(L_pod, d_eq_pod) * float(getattr(inputs, "central_pod_cross_section_form_factor", 1.10))
    Q_pod = float(getattr(inputs, "central_pod_interference_factor", 1.05))
    D_pod = q * Cf_pod * FF_pod * Q_pod * Swet_pod

    # Two circular booms aligned with the flow.
    n_booms = max(1, int(inputs.n_booms))
    L_boom = float(inputs.boom_length_m)
    D_boom_outer = float(inputs.boom_outer_diameter_m)
    Swet_booms = n_booms * math.pi * D_boom_outer * L_boom
    Re_boom = rho * V * L_boom / max(mu, 1e-12)
    Cf_boom = _skin_friction_coefficient(Re_boom, laminar_fraction) * roughness_mult
    FF_boom = _body_form_factor(L_boom, D_boom_outer)
    Q_boom = float(getattr(inputs, "boom_interference_factor", 1.10))
    D_booms = q * Cf_boom * FF_boom * Q_boom * Swet_booms

    # Lift-motor nacelles, support pylons and stopped rotor blades.
    n_lift = max(0, int(inputs.n_lift_propellers))
    nacelle_d = float(getattr(inputs, "lift_motor_nacelle_diameter_m", 0.055))
    nacelle_Cd = float(getattr(inputs, "lift_motor_nacelle_CD", 0.80))
    nacelle_frontal_each = math.pi * (0.5 * nacelle_d) ** 2
    D_lift_nacelles = q * n_lift * nacelle_Cd * nacelle_frontal_each

    support_CdA_each = max(0.0, float(getattr(inputs, "lift_motor_support_CdA_each_m2", 0.0010)))
    D_lift_supports = q * n_lift * support_CdA_each

    prop_drag_area_each = _lift_prop_drag_area_each_m2(propulsion_selection, inputs)
    prop_blade_Cd = float(getattr(inputs, "lift_prop_blade_section_CD", 1.10))
    D_stopped_lift_props = q * n_lift * prop_blade_Cd * prop_drag_area_each

    # Cooling, openings, antennae and local junction losses are entered as equivalent CdA.
    cooling_CdA = max(0.0, float(getattr(inputs, "fuel_cell_cooling_CdA_m2", 0.0025)))
    misc_CdA = max(0.0, float(getattr(inputs, "miscellaneous_CdA_m2", 0.0015)))
    D_cooling = q * cooling_CdA
    D_misc = q * misc_CdA

    junction_CD0 = max(0.0, float(getattr(inputs, "configuration_junction_CD0", 0.0040)))
    D_junction = q * S_ref * junction_CD0

    D_total = (
        D_wing + D_tail + D_vertical_tails + D_pod + D_booms
        + D_lift_nacelles + D_lift_supports + D_stopped_lift_props
        + D_cooling + D_misc + D_junction
    )
    D_induced_total = D_induced_wing + D_induced_tail
    D_parasite_total = max(0.0, D_total - D_induced_total)

    return {
        "dynamic_pressure_Pa": q,
        "wing_N": D_wing,
        "horizontal_tail_N": D_tail,
        "vertical_tails_N": D_vertical_tails,
        "central_pod_N": D_pod,
        "twin_booms_N": D_booms,
        "lift_motor_nacelles_N": D_lift_nacelles,
        "lift_motor_supports_N": D_lift_supports,
        "stopped_lift_propellers_N": D_stopped_lift_props,
        "fuel_cell_cooling_N": D_cooling,
        "miscellaneous_N": D_misc,
        "junction_interference_N": D_junction,
        "induced_total_N": D_induced_total,
        "parasite_total_N": D_parasite_total,
        "total_N": D_total,
        "CD0_aircraft": D_parasite_total / max(q * S_ref, 1e-9),
        "CD_total_ref": D_total / max(q * S_ref, 1e-9),
        "pod_wetted_area_m2": Swet_pod,
        "booms_wetted_area_m2": Swet_booms,
        "lift_prop_drag_area_each_m2": prop_drag_area_each,
        "number_of_vertical_fins": float(max(1, int(inputs.n_vertical_tail_surfaces))),
    }
