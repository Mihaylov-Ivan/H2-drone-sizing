"""
Aerodynamic coefficient calculations for wing and tail surfaces.
"""

import math
from typing import Dict

from design_inputs import DesignInputs
from constants import ATMOSPHERIC
from geometry import lifting_surface_wetted_area_estimated_m2
from airfoil_naca2410 import interpolate_by_alpha as interpolate_naca2410
from airfoil_naca0012 import interpolate_by_alpha as interpolate_naca0012


def _profile_drag_wetted_scale(
    inputs: DesignInputs,
    hardcode_attr: str,
    planform_S_m2: float,
) -> float:
    """
    Scale factor for airfoil profile CD when hardcoded wetted area is set (else 1.0).
    Baseline wetted area = 2 × planform (upper + lower).
    """
    S_wet_est = lifting_surface_wetted_area_estimated_m2(planform_S_m2)
    if S_wet_est <= 1e-18:
        return 1.0
    if getattr(inputs, "use_hardcoded_values", False):
        hc = getattr(inputs, hardcode_attr, None)
        if hc is not None:
            return float(hc) / S_wet_est
    return 1.0


def effective_lifting_surface_wetted_area_m2(
    inputs: DesignInputs,
    hardcode_attr: str,
    planform_S_m2: float,
) -> float:
    """
    Wetted area [m²] used for lifting-surface profile-drag scaling: hardcoded value if
    enabled, otherwise 2 × planform area.
    """
    S_wet_est = lifting_surface_wetted_area_estimated_m2(planform_S_m2)
    if getattr(inputs, "use_hardcoded_values", False):
        hc = getattr(inputs, hardcode_attr, None)
        if hc is not None:
            return float(hc)
    return S_wet_est


def wing_aerodynamic_coeffs(
    L_wing: float,
    L_tail: float,
    S: float,
    MAC: float,
    AR: float,
    inputs: DesignInputs,
) -> Dict[str, float]:
    """
    Calculate wing aerodynamic coefficients at cruise.
    
    Cruciform assumptions:
    - Only the horizontal wing carries vertical lift.
    # VERTICAL MAIN WING REMOVED FOR NOW
    # - The vertical wing contributes drag only (no vertical lift).
    - S is the area of the horizontal wing and is the reference area.
    
    Args:
        L_wing: Aircraft weight [N]
        L_tail: Tail lift at cruise [N] (positive up, negative for downforce)
        S: Wing area [m²]
        MAC: Mean aerodynamic chord [m]
        AR: Aspect ratio
        inputs: Design inputs
    
    Returns:
        Dictionary of aerodynamic coefficients and properties
    """
    V = inputs.speed
    q = 0.5 * ATMOSPHERIC.rho * V**2
    
    # Aircraft lift coefficient referenced to horizontal wing area S
    CL = L_wing / (q * S)
    
    # 3D finite-wing lift-curve slope
    CL_alpha_2D = 2.0 * math.pi  # ~6.283 per rad
    CL_alpha_3D = CL_alpha_2D / (1.0 + CL_alpha_2D / (math.pi * inputs.e_oswald * AR))
    
    alpha_L0_wing_deg = float(inputs.NACA2410_zero_lift_angle_of_attack_deg)  # for NACA2410
    alpha_cruise_rad = CL / CL_alpha_3D
    alpha_cruise_deg = alpha_L0_wing_deg + math.degrees(alpha_cruise_rad)
    
    # Airfoil data at cruise AoA and at 0 deg (NACA2410 for horizontal wing)
    airfoil_data = interpolate_naca2410(alpha_cruise_deg)
    airfoil_zero = interpolate_naca2410(inputs.NACA2410_zero_lift_angle_of_attack_deg)
    
    # 2D -> 3D CLmax
    CLmax_2D = airfoil_data["CL_max_pos"]  # global 2D CLmax from polar
    CL_max_3D = CLmax_2D * (AR / (AR + 2.0))  # simple finite-wing correction
    
    # Zero-lift CD of horizontal wing at 0 deg
    CD0_wing = airfoil_zero["CD"]

    # Horizontal wing profile drag at cruise AoA
    CD_profile_h = airfoil_data["CD"]

    wet_scale_w = _profile_drag_wetted_scale(inputs, "hardcode_wing_wetted_area_m2", S)
    CD0_wing *= wet_scale_w
    CD_profile_h *= wet_scale_w

    # Induced drag (horizontal wing is the only lifting surface)
    CD_induced_h = CL**2 / (math.pi * AR * inputs.e_oswald)
    
    # Fuselage parasite drag
    CD0_fuselage = fuselage_parasite_drag(inputs, S, V)

    # Total horizontal wing CD at cruise, referenced to S (horizontal wing area)
    # VERTICAL MAIN WING REMOVED FOR NOW
    # Note: Vertical wing drag will be added separately in script.py
    CD_total = CD_profile_h + CD_induced_h + CD0_fuselage
    
    # Aircraft L/D using same reference area S
    L_over_D = CL / CD_total
    
    # Stall speed using 3D CLmax
    V_stall = math.sqrt(2.0 * L_wing / (ATMOSPHERIC.rho * S * CL_max_3D))
    
    # Reynolds number at cruise (based on MAC)
    Re = ATMOSPHERIC.rho * V * MAC / ATMOSPHERIC.mu
    
    return {
        "CL_cruise": CL,
        "CL_max": CL_max_3D,  # 3D CLmax actually used for stall
        "CL_alpha_3D_per_rad": CL_alpha_3D,
        "CD0": CD0_wing,  # zero-lift CD of horizontal wing, ref S
        "CD0_fuselage": CD0_fuselage,  # fuselage parasite drag
        "CD_cruise": CD_total,  # horizontal wing CD at cruise, ref S
        # VERTICAL MAIN WING REMOVED FOR NOW (vertical wing drag added separately)
        "CM_cruise": airfoil_data["CM"],
        "CM_max": airfoil_data["CM_max"],
        "alpha_cruise": alpha_cruise_deg,
        "L_over_D_cruise": L_over_D,
        "V_stall": V_stall,
        "Re_cruise": Re,
        "max_thickness_fraction_of_chord": airfoil_data["max_thickness_fraction_of_chord"],
    }

def tail_aerodynamic_coeffs(
    L_required_N: float,
    S: float,
    MAC: float,
    AR: float,
    alpha_body_deg: float,
    downwash_deg: float,
    inputs: DesignInputs,
) -> Dict[str, float]:
    """
    Tail trim at cruise using fixed/limited built-in incidence and elevator for residual trim.

    S and AR come from geometry.tail_geometry (horizontal tail span is fixed when
    use_hardcoded_values and hardcode_tail_span are set).

    Logic:
    - Built-in tail incidence is limited by tail_incidence_limit_deg
    - Elevator provides the remaining trim authority
    - Preferred cruise elevator limit is reported but not fatal
    - Hard failure only if:
        * elevator exceeds physical max up/down limits, or
        * effective tail angle exceeds allowed/stall limits
    """
    V = inputs.speed
    q = 0.5 * ATMOSPHERIC.rho * V**2

    eta_tail = inputs.tail_efficiency_factor
    S_eff = S

    CL_req = L_required_N / (q * S_eff * eta_tail)

    # finite-tail lift-curve slope
    CL_alpha_2D = 2.0 * math.pi
    CL_alpha_3D = CL_alpha_2D / (1.0 + CL_alpha_2D / (math.pi * inputs.tail_e_oswald * AR))

    alpha_L0_deg = inputs.NACA0012_zero_lift_angle_of_attack_deg
    alpha_tail_geom_deg = alpha_body_deg - downwash_deg

    # elevator effectiveness tau_e
    elevator_chord_ratio = inputs.tail_elevator_chord_ratio
    elevator_span_fraction = inputs.tail_elevator_span_end - inputs.tail_elevator_span_start

    if inputs.tail_elevator_tau_override > 0.0:
        tau_e = float(inputs.tail_elevator_tau_override)
    else:
        # simple but tunable first-order estimate
        eta_f = max(0.05, 1.0 - 0.5 * elevator_chord_ratio)
        tau_e = elevator_chord_ratio * eta_f * elevator_span_fraction

    # ------------------------------------------------------------------
    # Choose built-in incidence first, within the allowed hard limit
    # ------------------------------------------------------------------
    inc_limit = abs(float(inputs.tail_incidence_limit_deg))

    # incidence that would achieve target elevator at cruise
    target_delta_elev_deg = float(inputs.tail_target_elevator_deg)
    required_total_deg = math.degrees(CL_req / CL_alpha_3D)
    tail_incidence_ideal_deg = (
        required_total_deg
        - (alpha_tail_geom_deg - alpha_L0_deg + tau_e * target_delta_elev_deg)
    )

    # clamp to allowed mounting limit
    tail_incidence_deg = max(-inc_limit, min(inc_limit, tail_incidence_ideal_deg))

    # ------------------------------------------------------------------
    # Now solve required elevator with that incidence fixed
    # ------------------------------------------------------------------
    alpha_no_elev_deg = alpha_tail_geom_deg + tail_incidence_deg - alpha_L0_deg
    CL_no_elev = CL_alpha_3D * math.radians(alpha_no_elev_deg)

    dCL_required = CL_req - CL_no_elev
    d_alpha_deg = math.degrees(dCL_required / CL_alpha_3D)

    if abs(tau_e) > 1e-9:
        delta_elev_deg = d_alpha_deg / tau_e
    else:
        delta_elev_deg = 0.0

    # effective tail angle used for 2D polar sampling / stall check
    alpha_equiv_deg = alpha_tail_geom_deg + tail_incidence_deg + tau_e * delta_elev_deg

    # airfoil polar
    airfoil_data = interpolate_naca0012(alpha_equiv_deg)
    airfoil_zero = interpolate_naca0012(inputs.NACA0012_zero_lift_angle_of_attack_deg)

    CLmax_2D = airfoil_data["CL_max_pos"]
    CL_max_3D = CLmax_2D * (AR / (AR + 2.0))

    CD0 = airfoil_zero["CD"]
    CD_profile = airfoil_data["CD"]
    wet_scale_t = _profile_drag_wetted_scale(inputs, "hardcode_tail_wetted_area_m2", S_eff)
    CD0 *= wet_scale_t
    CD_profile *= wet_scale_t
    CD_induced = (CL_req**2) / (math.pi * AR * inputs.tail_e_oswald)

    # empirical elevator drag
    S_elevator = elevator_chord_ratio * S_eff * elevator_span_fraction
    CD0_elevator = control_surface_drag_delta(
        delta_deg=abs(delta_elev_deg),
        S_control_m2=S_elevator,
        S_ref_m2=S_eff,
        k=inputs.control_surface_drag_k,
    )
    CD_total = CD_profile + CD_induced + CD0_elevator

    Re = ATMOSPHERIC.rho * V * MAC / ATMOSPHERIC.mu

    # ------------------------------------------------------------------
    # Hard feasibility checks
    # ------------------------------------------------------------------
    elevator_hard_ok = (
        inputs.tail_elevator_max_up_deg <= delta_elev_deg <= inputs.tail_elevator_max_down_deg
    )

    # polar range guard
    alpha_polar_min = -14.0
    alpha_polar_max = 14.0

    alpha_limit = abs(float(inputs.tail_effective_alpha_limit_deg))
    alpha_allowed_ok = abs(alpha_equiv_deg) <= alpha_limit

    alpha_stall_ok = (
        alpha_equiv_deg >= alpha_polar_min + inputs.tail_stall_margin_deg
        and alpha_equiv_deg <= alpha_polar_max - inputs.tail_stall_margin_deg
    )

    feasible = elevator_hard_ok and alpha_allowed_ok and alpha_stall_ok

    failure_reason = None
    if not elevator_hard_ok:
        failure_reason = (
            f"Required cruise elevator {delta_elev_deg:.1f} deg exceeds hard limits "
            f"[{inputs.tail_elevator_max_up_deg:.1f}, {inputs.tail_elevator_max_down_deg:.1f}] deg."
        )
    elif not alpha_allowed_ok:
        failure_reason = (
            f"Effective tail angle {alpha_equiv_deg:.1f} deg exceeds allowed limit "
            f"±{alpha_limit:.1f} deg."
        )
    elif not alpha_stall_ok:
        failure_reason = (
            f"Effective tail angle {alpha_equiv_deg:.1f} deg is too close to/outside polar range "
            f"with margin {inputs.tail_stall_margin_deg:.1f} deg."
        )

    # hinge moment estimate
    S_e = S_elevator
    c_e = elevator_chord_ratio * MAC
    delta_max_rad = math.radians(abs(inputs.tail_elevator_max_deflection_deg))
    Ch = inputs.elevator_Ch_delta_per_rad * delta_max_rad
    q_max = 0.5 * ATMOSPHERIC.rho * inputs.dive_speed**2
    M_h_Nm = q_max * S_e * c_e * Ch

    r_servo = inputs.servo_arm_radius_m
    r_horn = inputs.elevator_horn_radius_m
    eta = inputs.servo_efficiency_eta
    T_servo_Nm = (M_h_Nm * r_horn) / (eta * r_servo) if r_servo > 1e-9 else 0.0
    T_servo_with_margin_Nm = T_servo_Nm * inputs.servo_torque_margin

    return {
        "tail_incidence_ideal_deg": tail_incidence_ideal_deg,
        "tail_incidence_deg": tail_incidence_deg,
        "delta_elev_deg": delta_elev_deg,
        "CL_req": CL_req,
        "CL_alpha_3D_per_rad": CL_alpha_3D,
        "CL_no_elev": CL_no_elev,
        "dCL_required": dCL_required,
        "alpha_body_deg": alpha_body_deg,
        "downwash_deg": downwash_deg,
        "alpha_tail_geom_deg": alpha_tail_geom_deg,
        "alpha_equiv_deg": alpha_equiv_deg,
        "elevator_chord_ratio": elevator_chord_ratio,
        "tau_e": tau_e,
        "CL_cruise": CL_req,
        "CL_max": CL_max_3D,
        "CD0": CD0,
        "CD_cruise": CD_total,
        "CM_cruise": airfoil_data["CM"],
        "CM_max": airfoil_data["CM_max"],
        "L_over_D_cruise": (CL_req / CD_total) if abs(CD_total) > 1e-12 else float("inf"),
        "V_stall": math.sqrt(2.0 * abs(L_required_N) / (ATMOSPHERIC.rho * S_eff * max(1e-6, abs(CL_max_3D)))),
        "Re_cruise": Re,
        "max_thickness_fraction_of_chord": airfoil_data["max_thickness_fraction_of_chord"],
        "preferred_cruise_elevator_ok": abs(delta_elev_deg) <= abs(inputs.tail_elevator_limit_deg),
        "elevator_hard_ok": elevator_hard_ok,
        "alpha_allowed_ok": alpha_allowed_ok,
        "alpha_stall_ok": alpha_stall_ok,
        "feasible": feasible,
        "failure_reason": failure_reason,
        "elevator_hinge_moment_M_h_Nm": M_h_Nm,
        "elevator_area_S_e_m2": S_e,
        "elevator_chord_c_e_m": c_e,
        "servo_torque_T_servo_Nm": T_servo_Nm,
        "servo_torque_with_margin_Nm": T_servo_with_margin_Nm,
    }

def fuselage_parasite_drag(inputs: DesignInputs, S_ref_m2: float, V_mps: float) -> float:
    """
    Estimate fuselage zero-lift drag coefficient using Scholz (HAW Hamburg) method.

    C_D0_fuse = C_f * F_fuse * (S_wet_fuse / S_ref)

    Args:
        inputs: DesignInputs (with fuselage length & diameter)
        S_ref_m2: reference wing area [m²]
        V_mps: cruise speed [m/s]

    Returns:
        C_D0_fuse: parasite drag coefficient contribution of fuselage
    """

    L_D = inputs.fuselage_length / inputs.fuselage_OD

    # Wetted area (approximate circular cylinder)
    S_wet = inputs.fuselage_wetted_area

    # Reynolds number based on length
    Re = ATMOSPHERIC.rho * V_mps * inputs.fuselage_length / ATMOSPHERIC.mu

    # Turbulent flat plate Cf
    C_f = 0.455 / (math.log10(Re) ** 2.58)

    # Form factor (Scholz / Torenbeek)
    F_fuse = 1 + 60.0 / (L_D ** 3) + 0.0025 * L_D

    # Fuselage parasite drag coefficient
    C_D0_fuse = C_f * F_fuse * (S_wet / S_ref_m2)

    return C_D0_fuse


# VERTICAL MAIN WING REMOVED FOR NOW
# def vertical_wing_aerodynamic_coeffs(
#     S: float,
#     MAC: float,
#     AR: float,
#     inputs: DesignInputs,
# ) -> Dict[str, float]:
#     """
#     Calculate vertical wing aerodynamic coefficients at cruise.
#     
#     Vertical wings are assumed to operate at ~0 deg angle of attack (no lift),
#     contributing only profile drag. Uses NACA0012 airfoil.
#     
#     Args:
#         S: Vertical wing area [m²] (same as horizontal wing for now)
#         MAC: Mean aerodynamic chord [m]
#         AR: Aspect ratio
#         inputs: Design inputs
#     
#     Returns:
#         Dictionary of aerodynamic coefficients and properties
#     """
#     V = inputs.speed
#     q = 0.5 * ATMOSPHERIC.rho * V**2
#     
#     # Vertical wing operates at ~0 deg AoA (no lift in cruise)
#     alpha_vertical_deg = 0.0
#     
#     # Use NACA0012 airfoil for vertical wing
#     airfoil_data = interpolate_naca0012(alpha_vertical_deg)
#     airfoil_zero = interpolate_naca0012(inputs.NACA0012_zero_lift_angle_of_attack_deg)  # Zero lift at 0 deg for symmetric airfoil
#     
#     # No lift on vertical wing in cruise
#     CL = 0.0
#     
#     # 2D -> 3D CLmax (for reference, though not used for lift)
#     CLmax_2D = airfoil_data["CL_max_pos"]
#     CL_max_3D = CLmax_2D * (AR / (AR + 2.0))
#     
#     # Profile drag at 0 deg AoA
#     CD_profile = airfoil_data["CD"]
#     
#     # No induced drag (no lift)
#     CD_induced = 0.0
#     
#     # Total drag coefficient
#     CD_total = CD_profile + CD_induced
#     
#     # L/D is undefined (no lift), set to 0 or inf
#     L_over_D = 0.0
#     
#     # Stall speed (not meaningful for vertical wing, but calculate for completeness)
#     # Use a small reference lift for calculation
#     # Validate inputs before square root
#     if S <= 0:
#         raise ValueError(
#             f"Invalid vertical wing area: S = {S:.6f} m² (must be positive). "
#             f"This indicates the horizontal wing area is negative, which occurs when "
#             f"L_wing = W0_N - L_tail becomes negative (tail lift exceeds aircraft weight)."
#         )
#     
#     denominator = ATMOSPHERIC.rho * S * max(1e-6, abs(CL_max_3D))
#     if denominator <= 0:
#         raise ValueError(
#             f"Invalid denominator for V_stall calculation: rho*S*CL_max = {denominator:.6f}. "
#             f"rho = {ATMOSPHERIC.rho}, S = {S:.6f}, CL_max_3D = {CL_max_3D:.6f}"
#         )
#     
#     V_stall = math.sqrt(2.0 * 1.0 / denominator)
#     
#     # Reynolds number at cruise (based on MAC)
#     Re = ATMOSPHERIC.rho * V * MAC / ATMOSPHERIC.mu
#     
#     return {
#         "CL_cruise": CL,
#         "CL_max": CL_max_3D,
#         "CD0": airfoil_zero["CD"],
#         "CD0_fuselage": 0.0,  # No fuselage drag contribution here
#         "CD_cruise": CD_total,
#         "CM_cruise": airfoil_data["CM"],
#         "CM_max": airfoil_data["CM_max"],
#         "alpha_cruise": alpha_vertical_deg,
#         "L_over_D_cruise": L_over_D,
#         "V_stall": V_stall,
#         "Re_cruise": Re,
#         "max_thickness_fraction_of_chord": airfoil_data["max_thickness_fraction_of_chord"],
#     }


def vertical_tail_aerodynamic_coeffs(
    S: float,
    MAC: float,
    AR: float,
    inputs: DesignInputs,
) -> Dict[str, float]:
    """
    Calculate vertical tail aerodynamic coefficients at cruise.
    
    Vertical tail is assumed to operate at ~0 deg angle of attack (no lift in cruise),
    contributing only profile drag. Uses NACA0012 airfoil.
    S and AR come from geometry.vertical_tail_geometry (span may be fixed by
    use_hardcoded_values + hardcode_vertical_tail_span).
    
    Args:
        S: Vertical tail area [m²] (same as horizontal tail for now)
        MAC: Mean aerodynamic chord [m]
        AR: Aspect ratio
        inputs: Design inputs
    
    Returns:
        Dictionary of aerodynamic coefficients and properties
    """
    V = inputs.speed
    q = 0.5 * ATMOSPHERIC.rho * V**2
    
    # Vertical tail operates at ~0 deg AoA (no lift in cruise)
    alpha_vertical_deg = 0.0
    
    # Use NACA0012 airfoil for vertical tail
    airfoil_data = interpolate_naca0012(alpha_vertical_deg)
    airfoil_zero = interpolate_naca0012(inputs.NACA0012_zero_lift_angle_of_attack_deg)  # Zero lift at 0 deg for symmetric airfoil

    # No lift on vertical tail in cruise
    CL = 0.0

    CL_alpha_2D = 2.0 * math.pi  # per rad
    CL_alpha_3D = CL_alpha_2D / (1.0 + CL_alpha_2D / (math.pi * inputs.tail_e_oswald * AR))
    
    # 2D -> 3D CLmax (for reference, though not used for lift)
    CLmax_2D = airfoil_data["CL_max_pos"]
    CL_max_3D = CLmax_2D * (AR / (AR + 2.0))
    
    # Profile drag at 0 deg AoA
    CD_profile = airfoil_data["CD"]
    wet_scale_vt = _profile_drag_wetted_scale(inputs, "hardcode_vertical_tail_wetted_area_m2", S)
    CD_profile *= wet_scale_vt
    CD0_vertical = airfoil_zero["CD"] * wet_scale_vt

    # No induced drag (no lift)
    CD_induced = 0.0

    # Rudder model (for control surface drag calculation)
    rudder_chord_ratio = inputs.vertical_tail_rudder_chord_ratio
    rudder_span_fraction = inputs.vertical_tail_rudder_span_end - inputs.vertical_tail_rudder_span_start
    # At cruise with no sideslip, rudder deflection is 0 deg (sizing logic, not maneuvering)
    delta_rudder_deg = 0.0
    
    # Control surface drag
    # Rudder area accounts for chord ratio and span fraction
    S_rudder = rudder_chord_ratio * S * rudder_span_fraction
    CD0_rudder = control_surface_drag_delta(
        delta_deg=abs(delta_rudder_deg),
        S_control_m2=S_rudder,
        S_ref_m2=S,
        k=inputs.control_surface_drag_k,
    )
    
    # Total drag coefficient
    CD_total = CD_profile + CD_induced + CD0_rudder
    
    # L/D is undefined (no lift), set to 0
    L_over_D = 0.0
    
    # Stall speed (not meaningful for vertical tail, but calculate for completeness)
    V_stall = math.sqrt(2.0 * 1.0 / (ATMOSPHERIC.rho * S * max(1e-6, abs(CL_max_3D))))
    
    # Reynolds number at cruise (based on MAC)
    Re = ATMOSPHERIC.rho * V * MAC / ATMOSPHERIC.mu
    
    return {
        "CL_cruise": CL,
        "CL_max": CL_max_3D,
        "CL_alpha_3D_per_rad": CL_alpha_3D,
        "CD0": CD0_vertical,
        "CD0_fuselage": 0.0,  # No fuselage drag contribution here
        "CD0_rudder": CD0_rudder,
        "CD_cruise": CD_total,
        "CM_cruise": airfoil_data["CM"],
        "CM_max": airfoil_data["CM_max"],
        "alpha_cruise": alpha_vertical_deg,
        "L_over_D_cruise": L_over_D,
        "V_stall": V_stall,
        "Re_cruise": Re,
        "max_thickness_fraction_of_chord": airfoil_data["max_thickness_fraction_of_chord"],
        "rudder_chord_ratio": rudder_chord_ratio,
        "rudder_span_fraction": rudder_span_fraction,
        "delta_rudder_deg": delta_rudder_deg,
    }

def control_surface_drag_delta(
    delta_deg: float,
    S_control_m2: float,
    S_ref_m2: float,
    k: float = 0.015,
) -> float:
    """
    Empirical profile drag increment due to control surface deflection.
    ΔCD0 ≈ k * (S_ctrl / S_ref) * δ^2
    """
    delta_rad = math.radians(delta_deg)
    return k * (S_control_m2 / S_ref_m2) * delta_rad**2
