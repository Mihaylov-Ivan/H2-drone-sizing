"""Aero/propulsion aerodynamics and propulsion screening for H2AIR-4+1.

The model is deliberately split into two evidence levels:

1. ``calibrated_reference``: total parasite/induced drag is calibrated to the
   latest accessible H2AIR model result at 20 m/s. This preserves the current
   aircraft model while allowing speed, mass and lift-prop cruise state to vary.
2. ``engineering_assumption``: generic propeller coefficient maps are used only
   when measured/supplier prop maps are not provided. These results are useful
   for sizing and candidate ranking, but are not hardware release evidence.

Coordinate convention follows Phase 3: x from nose positive aft, y right,
z positive upward.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from math import pi, sqrt
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

G = 9.80665
INCH_TO_M = 0.0254
EPS = 1e-12


@dataclass(frozen=True)
class CruiseDragResult:
    mass_kg: float
    speed_mps: float
    lift_prop_diameter_in: float
    lift_prop_state: str
    dynamic_pressure_Pa: float
    non_lift_prop_parasite_drag_N: float
    lift_prop_drag_N: float
    induced_drag_N: float
    total_drag_N: float
    lift_to_drag: float
    propulsive_power_W: float
    cruise_propulsion_electrical_W: float
    auxiliary_power_W: float
    total_cruise_bus_power_W: float
    lift_prop_CdA_total_m2: float
    non_lift_prop_parasite_CdA_m2: float
    induced_drag_factor_k: float
    wing_area_m2: float
    wing_span_m: float
    wing_aspect_ratio: float

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class LiftSizingResult:
    mass_kg: float
    prop_diameter_in: float
    total_design_thrust_N: float
    design_thrust_per_motor_N: float
    total_disk_area_m2: float
    disk_loading_N_m2: float
    induced_velocity_hover_mps: float
    total_electrical_climb_power_W: float
    electrical_climb_power_per_motor_W: float
    total_design_power_with_margin_W: float
    design_power_per_motor_with_margin_W: float
    placeholder_thrust_pass: Optional[bool]
    placeholder_power_pass: Optional[bool]
    five_kW_screen_pass: bool
    conditional_hybrid_peak_screen_pass: bool

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class LiftLayoutResult:
    prop_diameter_in: float
    required_center_spacing_m: float
    current_center_spacing_m: float
    current_edge_clearance_m: float
    current_layout_pass: bool
    proposed_front_x_m: float
    proposed_rear_x_m: float
    proposed_edge_clearance_m: float
    proposed_layout_pass: bool
    front_shift_mm: float
    rear_shift_mm: float
    pod_lateral_clearance_m: float
    pod_lateral_clearance_pass: bool

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PusherOperatingPoint:
    prop_diameter_in: float
    speed_mps: float
    required_thrust_N: float
    rpm: float
    advance_ratio_J: float
    Ct: float
    Cp: float
    thrust_N: float
    shaft_power_W: float
    propulsive_efficiency: float
    electrical_power_W: float
    helical_tip_mach: float
    feasible: bool
    evidence_status: str

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class MissionEnergyScreen:
    prop_diameter_in: float
    lift_prop_state: str
    cruise_duration_min: float
    fixed_stage_energy_Wh: float
    cruise_bus_power_W: float
    cruise_energy_Wh: float
    total_bus_energy_Wh: float
    hover_power_W: float
    climb_power_W: float

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _linear_interp(points: Sequence[Tuple[float, float]], x: float) -> float:
    if not points:
        raise ValueError("Interpolation points cannot be empty")
    pts = sorted((float(a), float(b)) for a, b in points)
    if x <= pts[0][0]:
        return pts[0][1]
    if x >= pts[-1][0]:
        return pts[-1][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if x <= x1:
            f = (x - x0) / max(x1 - x0, EPS)
            return y0 + f * (y1 - y0)
    return pts[-1][1]


def _prop_coefficients(prop_map: Sequence[Mapping[str, float]], J: float) -> Tuple[float, float]:
    ct = _linear_interp([(p["J"], p["Ct"]) for p in prop_map], J)
    cp = _linear_interp([(p["J"], p["Cp"]) for p in prop_map], J)
    return max(0.0, ct), max(EPS, cp)


def calibrated_drag_constants(cfg: Mapping[str, Any]) -> Dict[str, float]:
    """Return parasite CdA and induced factor that reproduce the current baseline.

    The reference report gives total parasite drag and lift-prop drag separately.
    We subtract lift-prop drag from parasite drag so that Aero/propulsion can change prop
    diameter/state without changing the rest of the aircraft calibration.
    """
    atm = cfg["atmosphere"]
    ac = cfg["aircraft"]
    ref = cfg["reference_calibration"]
    rho = float(atm["rho_kg_m3"])
    V = float(ref["reference_speed_mps"])
    q = 0.5 * rho * V * V
    D_other_parasite = float(ref["reference_parasite_drag_N"]) - float(ref["reference_stopped_lift_prop_drag_N"])
    CdA_other = D_other_parasite / max(q, EPS)

    W = float(ref["reference_mass_kg"]) * G
    S = float(ref.get("reference_wing_area_m2", ac["wing_area_m2"]))
    D_i = float(ref["reference_induced_drag_N"])
    # D_i = k * W^2 / (q*S)
    k = D_i * q * S / max(W * W, EPS)
    b_ref = float(ref.get("reference_wing_span_m", ac["wing_span_m"]))
    AR = b_ref ** 2 / S
    e_equiv = 1.0 / (pi * AR * max(k, EPS))
    return {
        "non_lift_prop_parasite_CdA_m2": CdA_other,
        "induced_drag_factor_k": k,
        "equivalent_oswald_efficiency": e_equiv,
        "reference_wing_profile_drag_N": max(0.0, float(ref.get("reference_main_wing_drag_N", D_i)) - D_i),
    }


def lift_prop_drag_CdA_each_m2(cfg: Mapping[str, Any], diameter_in: float, state: str) -> float:
    ls = cfg["lift_system"]
    D = float(diameter_in) * INCH_TO_M
    R = 0.5 * D
    n_blades = max(1, int(ls["blade_count"]))
    chord_ratio = float(ls["mean_blade_chord_ratio"])
    blade_planform_area = n_blades * R * (chord_ratio * 2.0 * R)
    factors = ls["cruise_state_projected_area_factors"]
    if state not in factors:
        raise ValueError(f"Unsupported lift-prop cruise state {state!r}")
    projected_area = blade_planform_area * float(factors[state])
    return projected_area * float(ls["blade_section_CD"])


def cruise_drag(
    cfg: Mapping[str, Any],
    *,
    mass_kg: float,
    speed_mps: float,
    lift_prop_diameter_in: float,
    lift_prop_state: str,
    wing_area_m2: Optional[float] = None,
    wing_span_m: Optional[float] = None,
) -> CruiseDragResult:
    """Calibrated aircraft drag with optional wing-area sensitivity.

    At the reference wing area this reproduces the accessible H2AIR drag result.
    When area changes, the reference wing-profile component scales with q*S and
    induced drag scales with 1/(q*S). The remainder of the airframe parasite
    CdA is held fixed. This is a Aero/propulsion sizing surrogate, not a CFD replacement.
    """
    atm = cfg["atmosphere"]
    ac = cfg["aircraft"]
    ref = cfg["reference_calibration"]
    rho = float(atm["rho_kg_m3"])
    q = 0.5 * rho * speed_mps * speed_mps
    constants = calibrated_drag_constants(cfg)
    S_ref = float(ref.get("reference_wing_area_m2", ac["wing_area_m2"]))
    b_ref = float(ref.get("reference_wing_span_m", ac["wing_span_m"]))
    S = S_ref if wing_area_m2 is None else float(wing_area_m2)
    if S <= 0.0:
        raise ValueError("wing_area_m2 must be > 0")
    AR_ref = b_ref * b_ref / S_ref
    b = sqrt(AR_ref * S) if wing_span_m is None else float(wing_span_m)
    AR = b * b / S

    # Split the calibrated non-lift parasite term into a reference wing-profile
    # term and fixed aircraft CdA. The source report gives main-wing total drag
    # and total induced drag, so assigning all calibrated induced drag to the
    # wing is a documented screening approximation.
    q_ref = 0.5 * rho * float(ref["reference_speed_mps"]) ** 2
    D_wing_profile_ref = constants["reference_wing_profile_drag_N"]
    wing_profile_CD = D_wing_profile_ref / max(q_ref * S_ref, EPS)
    fixed_other_CdA = max(0.0, constants["non_lift_prop_parasite_CdA_m2"] - D_wing_profile_ref / max(q_ref, EPS))
    D_other = q * fixed_other_CdA + q * S * wing_profile_CD

    n_lift = int(cfg["lift_system"]["number_of_propellers"])
    CdA_prop_each = lift_prop_drag_CdA_each_m2(cfg, lift_prop_diameter_in, lift_prop_state)
    CdA_props = n_lift * CdA_prop_each
    D_props = q * CdA_props

    # Preserve equivalent Oswald efficiency when the screened span/area changes.
    e = constants["equivalent_oswald_efficiency"]
    k = 1.0 / (pi * max(AR, EPS) * max(e, EPS))
    W = mass_kg * G
    D_induced = k * W * W / max(q * S, EPS)
    D_total = D_other + D_props + D_induced
    eta = float(ac["cruise_propulsive_efficiency_reference"])
    P_propulsive = D_total * speed_mps
    P_cruise = P_propulsive / max(eta, EPS)
    aux = float(ac.get("continuous_auxiliary_power_W", 0.0))
    return CruiseDragResult(
        mass_kg=mass_kg,
        speed_mps=speed_mps,
        lift_prop_diameter_in=lift_prop_diameter_in,
        lift_prop_state=lift_prop_state,
        dynamic_pressure_Pa=q,
        non_lift_prop_parasite_drag_N=D_other,
        lift_prop_drag_N=D_props,
        induced_drag_N=D_induced,
        total_drag_N=D_total,
        lift_to_drag=W / max(D_total, EPS),
        propulsive_power_W=P_propulsive,
        cruise_propulsion_electrical_W=P_cruise,
        auxiliary_power_W=aux,
        total_cruise_bus_power_W=P_cruise + aux,
        lift_prop_CdA_total_m2=CdA_props,
        non_lift_prop_parasite_CdA_m2=(D_other / max(q, EPS)),
        induced_drag_factor_k=k,
        wing_area_m2=S,
        wing_span_m=b,
        wing_aspect_ratio=AR,
    )


def stall_speed_mps(cfg: Mapping[str, Any], mass_kg: float, wing_area_m2: Optional[float] = None) -> float:
    """Stall-speed screen calibrated to the source model's reported stall speed.

    The source model's wing lift is not exactly aircraft weight because the tail
    carries trim load. Scaling from its reported 14.75 m/s preserves that trim
    convention better than recomputing from W/(qSCLmax) with total weight.
    """
    ac = cfg["aircraft"]
    ref = cfg["reference_calibration"]
    S_ref = float(ref.get("reference_wing_area_m2", ac["wing_area_m2"]))
    S = S_ref if wing_area_m2 is None else float(wing_area_m2)
    if S <= 0.0:
        raise ValueError("wing_area_m2 must be > 0")
    return float(ref["reference_wing_stall_speed_mps"]) * sqrt(
        (float(mass_kg) / float(ref["reference_mass_kg"])) * (S_ref / S)
    )


def minimum_wing_area_for_stall_margin(
    cfg: Mapping[str, Any], *, mass_kg: float, speed_mps: float, stall_ratio: Optional[float] = None
) -> Dict[str, float]:
    """Minimum wing-area screen for V / Vstall >= requested ratio."""
    ref = cfg["reference_calibration"]
    ws = cfg.get("wing_sizing", {})
    ratio = float(stall_ratio if stall_ratio is not None else ws.get("transition_stall_ratio", cfg["aircraft"]["minimum_transition_stall_ratio"]))
    target_vstall = float(speed_mps) / max(ratio, EPS)
    S_ref = float(ref.get("reference_wing_area_m2", cfg["aircraft"]["wing_area_m2"]))
    S_req = S_ref * (float(mass_kg) / float(ref["reference_mass_kg"])) * (float(ref["reference_wing_stall_speed_mps"]) / max(target_vstall, EPS)) ** 2
    b_ref = float(ref.get("reference_wing_span_m", cfg["aircraft"]["wing_span_m"]))
    AR_ref = b_ref ** 2 / S_ref
    b_req = sqrt(AR_ref * S_req)
    return {
        "mass_kg": float(mass_kg),
        "speed_mps": float(speed_mps),
        "stall_ratio": ratio,
        "target_max_stall_speed_mps": target_vstall,
        "minimum_wing_area_m2": S_req,
        "screened_span_preserving_AR_m": b_req,
        "reference_AR": AR_ref,
        "current_area_pass": S_ref + EPS >= S_req,
        "current_stall_speed_mps": stall_speed_mps(cfg, float(mass_kg), S_ref),
    }


def _disk_area_total(diameter_in: float, n: int) -> float:
    D = diameter_in * INCH_TO_M
    return n * pi * (0.5 * D) ** 2


def _axial_climb_power_ideal_W(T_N: float, rho: float, A_total: float, climb_speed_mps: float) -> Tuple[float, float]:
    vh = sqrt(max(T_N, EPS) / max(2.0 * rho * A_total, EPS))
    P = T_N * (0.5 * climb_speed_mps + sqrt((0.5 * climb_speed_mps) ** 2 + vh * vh))
    return P, vh


def lift_system_sizing(cfg: Mapping[str, Any], *, mass_kg: float, prop_diameter_in: float) -> LiftSizingResult:
    ls = cfg["lift_system"]
    rho = float(cfg["atmosphere"]["rho_kg_m3"])
    n = max(1, int(ls["number_of_propellers"]))
    a = float(ls["design_vertical_acceleration_mps2"])
    thrust_margin = float(ls["thrust_margin"])
    T = mass_kg * (G + a) * thrust_margin
    A = _disk_area_total(prop_diameter_in, n)
    P_ideal, vh = _axial_climb_power_ideal_W(T, rho, A, float(ls["vertical_climb_speed_mps"]))
    P_elec = P_ideal / max(float(ls["effective_hover_climb_efficiency"]), EPS)
    P_margin = P_elec * float(ls["power_margin"])

    max_T = ls.get("current_placeholder_max_thrust_per_motor_N")
    max_P = ls.get("current_placeholder_max_power_per_motor_W")
    thrust_pass = None if max_T is None else float(max_T) >= T / n
    power_pass = None if max_P is None else float(max_P) >= P_margin / n
    return LiftSizingResult(
        mass_kg=mass_kg,
        prop_diameter_in=prop_diameter_in,
        total_design_thrust_N=T,
        design_thrust_per_motor_N=T / n,
        total_disk_area_m2=A,
        disk_loading_N_m2=T / max(A, EPS),
        induced_velocity_hover_mps=vh,
        total_electrical_climb_power_W=P_elec,
        electrical_climb_power_per_motor_W=P_elec / n,
        total_design_power_with_margin_W=P_margin,
        design_power_per_motor_with_margin_W=P_margin / n,
        placeholder_thrust_pass=thrust_pass,
        placeholder_power_pass=power_pass,
        five_kW_screen_pass=P_margin <= float(ls["application_peak_screening_W"]),
        conditional_hybrid_peak_screen_pass=P_margin <= float(ls.get("supplier_conditional_hybrid_peak_W", float("inf"))),
    )


def lift_layout(cfg: Mapping[str, Any], prop_diameter_in: float) -> LiftLayoutResult:
    lay = cfg["lift_layout"]
    D = prop_diameter_in * INCH_TO_M
    clear = float(lay["required_disk_edge_clearance_m"])
    xf = float(lay["current_front_x_m"])
    xr = float(lay["current_rear_x_m"])
    current_spacing = xr - xf
    current_edge = current_spacing - D
    required_spacing = D + clear

    mid = float(lay["common_midpoint_x_m"])
    spacing = max(current_spacing, required_spacing)
    proposed_front = mid - spacing / 2.0
    proposed_rear = mid + spacing / 2.0
    proposed_edge = spacing - D
    x_bounds_ok = proposed_front >= float(lay["minimum_front_x_m"]) - EPS and proposed_rear <= float(lay["maximum_rear_x_m"]) + EPS

    y = abs(float(lay["y_abs_m"]))
    pod_half = 0.5 * float(lay["central_pod_width_m"])
    lateral_gap = y - 0.5 * D - pod_half
    pod_pass = lateral_gap >= float(lay["required_disk_to_pod_clearance_m"]) - EPS
    return LiftLayoutResult(
        prop_diameter_in=prop_diameter_in,
        required_center_spacing_m=required_spacing,
        current_center_spacing_m=current_spacing,
        current_edge_clearance_m=current_edge,
        current_layout_pass=current_edge >= clear - EPS,
        proposed_front_x_m=proposed_front,
        proposed_rear_x_m=proposed_rear,
        proposed_edge_clearance_m=proposed_edge,
        proposed_layout_pass=(proposed_edge >= clear - EPS and x_bounds_ok and pod_pass),
        front_shift_mm=(proposed_front - xf) * 1000.0,
        rear_shift_mm=(proposed_rear - xr) * 1000.0,
        pod_lateral_clearance_m=lateral_gap,
        pod_lateral_clearance_pass=pod_pass,
    )


def pusher_operating_point(
    cfg: Mapping[str, Any],
    *,
    prop_diameter_in: float,
    speed_mps: float,
    required_aircraft_thrust_N: float,
    measured_map: Optional[Sequence[Mapping[str, float]]] = None,
) -> PusherOperatingPoint:
    ps = cfg["pusher_screen"]
    prop_map = measured_map if measured_map is not None else ps["generic_prop_map"]
    status = "measured" if measured_map is not None else str(ps["generic_prop_map_status"])
    D = prop_diameter_in * INCH_TO_M
    rho = float(cfg["atmosphere"]["rho_kg_m3"])
    a = float(cfg["atmosphere"]["speed_of_sound_mps"])
    eta_drive = float(ps["motor_efficiency_assumption"]) * float(ps["esc_efficiency_assumption"])
    best: Optional[PusherOperatingPoint] = None
    required = required_aircraft_thrust_N * float(ps["thrust_margin"])

    rpm = float(ps["rpm_min"])
    while rpm <= float(ps["rpm_max"]) + EPS:
        n = rpm / 60.0
        J = speed_mps / max(n * D, EPS)
        Ct, Cp = _prop_coefficients(prop_map, J)
        thrust = Ct * rho * n * n * D**4
        shaft = Cp * rho * n**3 * D**5
        eta_prop = thrust * speed_mps / max(shaft, EPS)
        tip_mach = sqrt((pi * D * n) ** 2 + speed_mps**2) / max(a, EPS)
        feasible = thrust >= required - EPS and tip_mach <= float(ps["maximum_tip_mach"]) + EPS and eta_prop > 0.0
        op = PusherOperatingPoint(
            prop_diameter_in=prop_diameter_in,
            speed_mps=speed_mps,
            required_thrust_N=required,
            rpm=rpm,
            advance_ratio_J=J,
            Ct=Ct,
            Cp=Cp,
            thrust_N=thrust,
            shaft_power_W=shaft,
            propulsive_efficiency=eta_prop,
            electrical_power_W=shaft / max(eta_drive, EPS),
            helical_tip_mach=tip_mach,
            feasible=feasible,
            evidence_status=status,
        )
        if feasible and (best is None or op.electrical_power_W < best.electrical_power_W):
            best = op
        rpm += float(ps["rpm_step"])

    if best is not None:
        return best
    return PusherOperatingPoint(
        prop_diameter_in=prop_diameter_in,
        speed_mps=speed_mps,
        required_thrust_N=required,
        rpm=0.0,
        advance_ratio_J=0.0,
        Ct=0.0,
        Cp=0.0,
        thrust_N=0.0,
        shaft_power_W=0.0,
        propulsive_efficiency=0.0,
        electrical_power_W=0.0,
        helical_tip_mach=0.0,
        feasible=False,
        evidence_status=status,
    )


def _operational_hover_power_W(cfg: Mapping[str, Any], mass_kg: float, prop_diameter_in: float) -> float:
    ls = cfg["lift_system"]
    rho = float(cfg["atmosphere"]["rho_kg_m3"])
    A = _disk_area_total(prop_diameter_in, int(ls["number_of_propellers"]))
    T = mass_kg * G
    Pideal, _ = _axial_climb_power_ideal_W(T, rho, A, 0.0)
    return Pideal / max(float(ls["effective_hover_climb_efficiency"]), EPS) + float(cfg["aircraft"]["continuous_auxiliary_power_W"])


def _operational_climb_power_W(cfg: Mapping[str, Any], mass_kg: float, prop_diameter_in: float) -> float:
    ls = cfg["lift_system"]
    rho = float(cfg["atmosphere"]["rho_kg_m3"])
    A = _disk_area_total(prop_diameter_in, int(ls["number_of_propellers"]))
    T = mass_kg * (G + float(ls["design_vertical_acceleration_mps2"]))
    Pideal, _ = _axial_climb_power_ideal_W(T, rho, A, float(ls["vertical_climb_speed_mps"]))
    return Pideal / max(float(ls["effective_hover_climb_efficiency"]), EPS) + float(cfg["aircraft"]["continuous_auxiliary_power_W"])


def mission_energy_screen(cfg: Mapping[str, Any], *, mass_kg: float, prop_diameter_in: float, lift_prop_state: str) -> MissionEnergyScreen:
    ms = cfg["mission_screen"]
    cruise = cruise_drag(
        cfg,
        mass_kg=mass_kg,
        speed_mps=float(cfg["aircraft"]["cruise_speed_mps"]),
        lift_prop_diameter_in=prop_diameter_in,
        lift_prop_state=lift_prop_state,
    )
    hover = _operational_hover_power_W(cfg, mass_kg, prop_diameter_in)
    climb = _operational_climb_power_W(cfg, mass_kg, prop_diameter_in)
    fixed_Wh = 0.0
    for s in ms["fixed_stages"]:
        mode = s["mode"]
        if mode == "fixed":
            P = float(s["power_W"])
        elif mode == "hover":
            P = hover
        elif mode == "climb":
            P = climb
        elif mode == "hover_factor":
            P = hover * float(s.get("factor", 1.0))
        else:
            raise ValueError(f"Unknown mission-screen stage mode {mode!r}")
        fixed_Wh += P * float(s["duration_s"]) / 3600.0
    fixed_duration_s = sum(max(0.0, float(s["duration_s"])) for s in ms["fixed_stages"])
    if "target_total_mission_duration_min" in ms:
        cruise_duration_min = max(0.0, float(ms["target_total_mission_duration_min"]) - fixed_duration_s / 60.0)
    else:
        cruise_duration_min = float(ms.get("cruise_duration_min", 0.0))
    cruise_h = cruise_duration_min / 60.0
    cruise_Wh = cruise.total_cruise_bus_power_W * cruise_h
    return MissionEnergyScreen(
        prop_diameter_in=prop_diameter_in,
        lift_prop_state=lift_prop_state,
        cruise_duration_min=cruise_duration_min,
        fixed_stage_energy_Wh=fixed_Wh,
        cruise_bus_power_W=cruise.total_cruise_bus_power_W,
        cruise_energy_Wh=cruise_Wh,
        total_bus_energy_Wh=fixed_Wh + cruise_Wh,
        hover_power_W=hover,
        climb_power_W=climb,
    )


def maximum_mass_supported_by_placeholder_thrust(cfg: Mapping[str, Any]) -> Optional[float]:
    ls = cfg["lift_system"]
    max_T = ls.get("current_placeholder_max_thrust_per_motor_N")
    if max_T is None:
        return None
    n = int(ls["number_of_propellers"])
    denom = (G + float(ls["design_vertical_acceleration_mps2"])) * float(ls["thrust_margin"])
    return n * float(max_T) / max(denom, EPS)
