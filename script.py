# ================================
# script.py  (FULL UPDATED FILE)
# ================================
"""
Conceptual sizing script following the SAKR-2 SUAV design procedure
(Zakaria et al., "Design and Production of Small Tailless UAV")
with Raymer-style weight & performance logic.

Workflow:
1) Mission + payload fraction  -> MTOW (W0)
2) Choose design point (W/S, P/W)
3) Wing sizing: S, b, chords, MAC
4) Aerodynamics at cruise: CL, CD, L/D, Re, stall speed
5) Propulsion: required cruise power, installed motor power
6) Energy: battery sizing, endurance estimate
"""

import math
import os
import sys
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import asdict

from design_inputs import DesignInputs
from constants import ATMOSPHERIC, DESIGN
from geometry import wing_geometry, tail_geometry, vertical_tail_geometry
from aerodynamics import (
    wing_aerodynamic_coeffs,
    tail_aerodynamic_coeffs,
    # vertical_wing_aerodynamic_coeffs,
    vertical_tail_aerodynamic_coeffs,
    effective_lifting_surface_wetted_area_m2,
    configuration_cruise_drag,
)
from structures import (
    calculate_wing_structure,
    calculate_tail_structure,
    calculate_central_pod_structure,
    calculate_twin_boom_structure,
)
from propulsion_system import (
    select_propulsion_system,
    calculate_component_masses,
    calculate_component_mass_breakdown,
    convert_to_results,
)
from maneuvering import elevator_control_derivatives, max_load_factor_from_elevator
from maneuvering_rudder import (
    rudder_control_derivatives,
    rudder_asymmetric_thrust,
    rudder_spin_recovery_capability,
    rudder_coordinated_turn_characteristics,
    rudder_max_yaw_moment,
)
from convergence import check_convergence, plot_mass_convergence, plot_tail_trim_convergence
from results import (
    DesignResults,
    AircraftResults,
    WingResults,
    TailResults,
    # VerticalWingResults,
    VerticalTailResults,
    FuselageResults,
    BoomResults,
    Geometry,
    Aerodynamics,
    Structure,
    AvionicsResults,
    ManeuverResults,
    RudderResults,
    MassPropertiesResults,
    DragBreakdownResults,
    build_cg_locations,
)
from reporting import print_all_results
from avionics import estimate_avionics_and_electrics
from inventor_param_export import export_inventor_parameters


def _get_hardcoded_value(
    inputs: DesignInputs,
    value_attr: str,
    calculated_value: float,
) -> float:
    """
    Helper function to get hardcoded value if master toggle is enabled and value is not None, otherwise return calculated value.
    
    Args:
        inputs: DesignInputs instance
        value_attr: Attribute name for the hardcoded value (e.g., 'hardcode_wing_weight_kg')
        calculated_value: The calculated value to use if hardcoding is disabled or value is None
    
    Returns:
        Hardcoded value if use_hardcoded_values is True and value is not None, otherwise calculated_value
    """
    if getattr(inputs, 'use_hardcoded_values', False):
        hardcoded = getattr(inputs, value_attr, None)
        if hardcoded is not None:
            return float(hardcoded)
    return calculated_value


class _Tee:
    """Write to multiple file-like objects (e.g. stdout and a file)."""

    def __init__(self, *files):
        self.files = files

    def write(self, obj):
        for f in self.files:
            f.write(obj)
            f.flush()

    def flush(self):
        for f in self.files:
            f.flush()


# ============================================================
# Tail extension: single source-of-truth helper
# ============================================================
def _apply_tail_span_extension_and_recompute(
    *,
    S_tail_nominal_or_scaled: float,
    c_root_tail: float,
    taper_ratio: float,
    span_extension_m: float,
) -> Tuple[float, float, float, float]:
    """
    Returns (S_tail_total, b_tail_total, AR_tail_total, y_MAC_tail_total)
    where the extension is ALWAYS baked into the geometry.

    IMPORTANT: This treats the extension as *physical span increase*.
    We therefore recompute area from trapezoid rule using extended span.
    """
    lam = float(taper_ratio)
    c_root = float(c_root_tail)
    c_tip = c_root * lam

    # "Base" span from area sizing (no extension)
    S_base = float(S_tail_nominal_or_scaled)
    if S_base <= 0.0:
        raise ValueError(f"Tail area must be > 0, got {S_base}")

    b_base = 2.0 * S_base / (c_root * (1.0 + lam))
    b_total = b_base + max(0.0, float(span_extension_m))

    # Physical area with extension (consistent trapezoid)
    S_total = b_total * (c_root + c_tip) / 2.0

    AR_total = (b_total ** 2) / S_total if S_total > 0 else 0.0
    y_MAC_total = (b_total / 6.0) * (1.0 + 2.0 * lam) / (1.0 + lam)

    return S_total, b_total, AR_total, y_MAC_total


def _mass_from_specific_weight(length_m: float, specific_weight_kg_per_m: float) -> float:
    """Mass [kg] from linear specific weight [kg/m]."""
    return max(0.0, float(length_m)) * max(0.0, float(specific_weight_kg_per_m))


def _fuselage_mass_from_specific_weights(inputs: DesignInputs) -> Tuple[float, float, float, float, float, float]:
    """
    Fuselage total and segment masses from segment-specific linear factors [kg/m].
    Returns (total, nose, section_C, section_B, section_A, tail).
    """
    wn = _mass_from_specific_weight(
        float(inputs.fuselage_nose_length),
        float(getattr(inputs, "fuselage_nose_specific_weight_kg_per_m", 0.0)),
    )
    wc = _mass_from_specific_weight(
        float(inputs.fuselage_body_section_C_length),
        float(getattr(inputs, "fuselage_C_specific_weight_kg_per_m", 0.0)),
    )
    wb = _mass_from_specific_weight(
        float(inputs.fuselage_body_section_B_length),
        float(getattr(inputs, "fuselage_B_specific_weight_kg_per_m", 0.0)),
    )
    wa = _mass_from_specific_weight(
        float(inputs.fuselage_body_section_A_length),
        float(getattr(inputs, "fuselage_A_specific_weight_kg_per_m", 0.0)),
    )
    wt = _mass_from_specific_weight(
        float(inputs.fuselage_tail_length),
        float(getattr(inputs, "fuselage_tail_specific_weight_kg_per_m", 0.0)),
    )
    w_total = wn + wc + wb + wa + wt
    return w_total, wn, wc, wb, wa, wt


def _fuselage_total_and_segments(
    inputs: DesignInputs,
    W_fuselage_calc: float,
) -> Tuple[float, float, float, float, float, float]:
    """Compatibility wrapper for the new central-pod mass model.

    The returned segment fields are retained because ``FuselageResults`` and the
    CAD/export path still expect them.  The complete pod mass is placed in the
    central wing/equipment bay (section B); CG accounting uses the single
    ``cg_x_central_pod`` location rather than these legacy segment locations.
    """
    w_total = max(0.0, float(W_fuselage_calc))
    return w_total, 0.0, 0.0, w_total, 0.0, 0.0


def _calculate_mass_properties(
    W_wings: float,
    W_nose: float,
    W_body_section_C: float,
    W_body_section_B: float,
    W_body_section_A: float,
    W_fuselage_tail: float,
    W_booms: float,
    W_tails: float,
    W_vertical_tails: float,
    component_masses: Dict[str, float],
    avionics_selection: Any,
    inputs: DesignInputs,
) -> Dict[str, Any]:
    """Build a detailed take-off/end-of-mission longitudinal mass table."""
    tube_wing_secondary = tube_wing_primary = 0.0
    tube_tail_secondary = tube_tail_primary = 0.0
    tube_vertical_tail_secondary = tube_vertical_tail_primary = 0.0
    longeron_add = 0.0

    if bool(getattr(inputs, "use_hardcoded_values", False)):
        tube_wing_secondary = float(getattr(inputs, "hardcode_wing_secondary_spar_weight_kg", 0.0) or 0.0)
        tube_wing_primary = float(getattr(inputs, "hardcode_wing_spar_weight_kg", 0.0) or 0.0)
        tube_tail_primary = float(getattr(inputs, "hardcode_tail_wing_spar_weight_kg", 0.0) or 0.0)
        tube_tail_secondary = float(getattr(inputs, "hardcode_elevator_rod_weight_kg", 0.0) or 0.0)
        tube_vertical_tail_primary = float(getattr(inputs, "hardcode_vertical_tail_wing_spar_weight_kg", 0.0) or 0.0)
        tube_vertical_tail_secondary = float(getattr(inputs, "hardcode_rudder_rod_weight_kg", 0.0) or 0.0)
        longeron_add = 0.0  # obsolete single-fuselage longerons are not used on the twin-boom pod

    c_MGC_wing = float(inputs.c_root) * (1.0 + float(inputs.taper_ratio)) / 2.0
    c_MGC_tail = float(inputs.tail_c_root) * (1.0 + float(inputs.tail_taper_ratio)) / 2.0
    wing_le_x = float(inputs.cg_x_wing) - 0.5 * c_MGC_wing
    tail_le_x = float(inputs.cg_x_tail) - 0.5 * c_MGC_tail
    vertical_tail_le_x = float(inputs.cg_x_vertical_tail) - 0.5 * c_MGC_tail

    x_primary_wing = wing_le_x + 0.3 * c_MGC_wing
    x_secondary_wing = wing_le_x + 0.7 * c_MGC_wing
    x_primary_tail = tail_le_x + 0.3 * c_MGC_tail
    x_secondary_tail = tail_le_x + 0.7 * c_MGC_tail
    x_primary_vertical_tail = vertical_tail_le_x + 0.3 * c_MGC_tail
    x_secondary_vertical_tail = vertical_tail_le_x + 0.7 * c_MGC_tail

    lift_mount_unit = (
        float(getattr(inputs, "lift_motor_mount_mass_each_kg", 0.0))
        * float(getattr(inputs, "lift_motor_mount_reinforcement_factor", 1.0))
    )
    lift_xs = [float(x) for x in inputs.lift_prop_x_positions_m]
    x_mid = 0.5 * (min(lift_xs) + max(lift_xs)) if lift_xs else 0.0
    n_mount_front = sum(1 for x in lift_xs if x <= x_mid)
    n_mount_rear = len(lift_xs) - n_mount_front
    W_mount_front = n_mount_front * lift_mount_unit
    W_mount_rear = n_mount_rear * lift_mount_unit
    wing_other = max(0.0, W_wings - tube_wing_primary - tube_wing_secondary - W_mount_front - W_mount_rear)
    tail_other = max(0.0, W_tails - tube_tail_primary - tube_tail_secondary)
    vertical_tail_other = max(0.0, W_vertical_tails - tube_vertical_tail_primary - tube_vertical_tail_secondary)

    items: Dict[str, Dict[str, float]] = {}

    def add(name: str, mass_takeoff: float, x_m: float, mass_end: Optional[float] = None) -> None:
        m0 = max(0.0, float(mass_takeoff))
        m1 = m0 if mass_end is None else max(0.0, float(mass_end))
        x = float(x_m)
        items[name] = {
            "mass_takeoff_kg": m0,
            "mass_end_kg": m1,
            "x_m": x,
            "moment_takeoff_kgm": m0 * x,
            "moment_end_kgm": m1 * x,
        }

    # Airframe and payload.
    add("wing_structure", wing_other, inputs.cg_x_wing)
    add("wing_primary_spar", tube_wing_primary, x_primary_wing)
    add("wing_secondary_spar", tube_wing_secondary, x_secondary_wing)
    add("lift_motor_mounts_front", W_mount_front, inputs.cg_x_lift_propulsion_front)
    add("lift_motor_mounts_rear", W_mount_rear, inputs.cg_x_lift_propulsion_rear)
    W_central_pod = W_nose + W_body_section_C + W_body_section_B + W_body_section_A + W_fuselage_tail + longeron_add
    add("central_pod_structure", W_central_pod, inputs.cg_x_central_pod)
    add("twin_boom_structure", W_booms, inputs.cg_x_booms)
    add("horizontal_tail_structure", tail_other, inputs.cg_x_tail)
    add("horizontal_tail_spar", tube_tail_primary, x_primary_tail)
    add("elevator_hinge_rod", tube_tail_secondary, x_secondary_tail)
    add("vertical_tail_structure", vertical_tail_other, inputs.cg_x_vertical_tail)
    add("vertical_tail_spar", tube_vertical_tail_primary, x_primary_vertical_tail)
    add("rudder_hinge_rod", tube_vertical_tail_secondary, x_secondary_vertical_tail)
    add("payload", inputs.payload_mass, inputs.cg_x_payload)

    # Hybrid energy system, each at its own position.
    add("buffer_battery", component_masses.get("buffer_battery_kg", 0.0), inputs.cg_x_battery)
    add("fuel_cell_system", component_masses.get("fuel_cell_system_kg", 0.0), inputs.cg_x_fuel_cell)
    add("hydrogen_tank", component_masses.get("hydrogen_tank_kg", 0.0), inputs.cg_x_hydrogen_tank)
    add("hydrogen_regulator", component_masses.get("hydrogen_regulator_kg", 0.0), inputs.cg_x_hydrogen_regulator)
    add(
        "stored_hydrogen",
        component_masses.get("hydrogen_takeoff_kg", 0.0),
        inputs.cg_x_hydrogen_tank,
        component_masses.get("hydrogen_end_kg", 0.0),
    )

    # Propulsion groups at actual front/rear/pusher stations.
    add("lift_propulsion_front", component_masses.get("lift_front_propulsion_kg", 0.0), inputs.cg_x_lift_propulsion_front)
    add("lift_propulsion_rear", component_masses.get("lift_rear_propulsion_kg", 0.0), inputs.cg_x_lift_propulsion_rear)
    add("cruise_propulsion", component_masses.get("cruise_propulsion_kg", 0.0), inputs.cg_x_cruise_propulsion)

    # ESCs are separated from the rest of avionics.
    add("lift_ESC_front", component_masses.get("lift_front_esc_kg", 0.0), inputs.cg_x_lift_esc_front)
    add("lift_ESC_rear", component_masses.get("lift_rear_esc_kg", 0.0), inputs.cg_x_lift_esc_rear)
    add("cruise_ESC", component_masses.get("cruise_esc_kg", 0.0), inputs.cg_x_cruise_esc)

    scale = float(component_masses.get("avionics_scale_factor", 1.0))
    add("autopilot", avionics_selection.autopilot_mass_kg * scale, inputs.cg_x_autopilot)
    add("gps", avionics_selection.gps_mass_kg * scale, inputs.cg_x_gps)
    add("airspeed_sensor", avionics_selection.airspeed_mass_kg * scale, inputs.cg_x_airspeed)
    add("rc_receiver", avionics_selection.rc_rx_mass_kg * scale, inputs.cg_x_rx)
    add("telemetry", avionics_selection.telemetry_mass_kg * scale, inputs.cg_x_tel)
    add("power_module", avionics_selection.power_module_mass_kg * scale, inputs.cg_x_power_module)
    add("BEC", avionics_selection.bec_mass_kg * scale, inputs.cg_x_bec)
    add("autopilot_powerbrick", avionics_selection.autopilot_powerbrick_mass_kg * scale, inputs.cg_x_autopilot_powerbrick)
    add("servos", avionics_selection.servos_mass_kg * scale, inputs.cg_x_servos)
    add("misc_electronics", avionics_selection.misc_electronics_mass_kg * scale, inputs.cg_x_misc)
    add("camera", avionics_selection.camera_mass_kg * scale, inputs.cg_x_camera)

    listed_non_esc = sum(
        items[k]["mass_takeoff_kg"] for k in (
            "autopilot", "gps", "airspeed_sensor", "rc_receiver", "telemetry",
            "power_module", "BEC", "autopilot_powerbrick", "servos",
            "misc_electronics", "camera",
        )
    )
    residual = max(0.0, float(component_masses.get("avionics_non_esc_kg", 0.0)) - listed_non_esc)
    add("avionics_unallocated_residual", residual, inputs.cg_x_misc)

    takeoff_mass = sum(v["mass_takeoff_kg"] for v in items.values())
    end_mass = sum(v["mass_end_kg"] for v in items.values())
    takeoff_moment = sum(v["moment_takeoff_kgm"] for v in items.values())
    end_moment = sum(v["moment_end_kgm"] for v in items.values())
    x_takeoff = takeoff_moment / max(takeoff_mass, 1e-12)
    x_end = end_moment / max(end_mass, 1e-12)

    return {
        "items": items,
        "takeoff_mass_kg": takeoff_mass,
        "end_mass_kg": end_mass,
        "takeoff_moment_kgm": takeoff_moment,
        "end_moment_kgm": end_moment,
        "x_cg_takeoff_m": x_takeoff,
        "x_cg_end_m": x_end,
        "cg_shift_m": x_end - x_takeoff,
        "hydrogen_takeoff_kg": float(component_masses.get("hydrogen_takeoff_kg", 0.0)),
        "hydrogen_end_kg": float(component_masses.get("hydrogen_end_kg", 0.0)),
    }


def _calculate_cg_position(
    W_wings: float,
    W_nose: float,
    W_body_section_C: float,
    W_body_section_B: float,
    W_body_section_A: float,
    W_fuselage_tail: float,
    W_booms: float,
    W_tails: float,
    W_vertical_tails: float,
    component_masses: Dict[str, float],
    avionics_selection: Any,
    inputs: DesignInputs,
) -> float:
    """Backward-compatible wrapper returning take-off CG."""
    return float(_calculate_mass_properties(
        W_wings,
        W_nose,
        W_body_section_C,
        W_body_section_B,
        W_body_section_A,
        W_fuselage_tail,
        W_booms,
        W_tails,
        W_vertical_tails,
        component_masses,
        avionics_selection,
        inputs,
    )["x_cg_takeoff_m"])


def _calculate_wing_system(
    W0_N: float,
    L_tail: float,
    inputs: DesignInputs,
    propulsion_selection: Optional[Dict[str, Any]] = None,
) -> Tuple[Geometry, Aerodynamics, Structure, float, float, float, Dict[str, float]]:
    """
    Calculate complete wing system (geometry, aerodynamics, structure).
    """
    L_wing = W0_N - L_tail

    if L_wing < 0:
        raise ValueError(
            f"Configuration invalid: Tail lift ({L_tail:.2f} N)  exceeds aircraft weight ({W0_N:.2f} N). "
            f"This results in negative wing lift ({L_wing:.2f} N) and negative wing area. "
            f"Possible causes: CG too far forward, wing moment too large, or tail moment arm too short. "
            f"Check CG positions in design_inputs.py and ensure tail can provide sufficient moment arm."
        )

    q = 0.5 * ATMOSPHERIC.rho * inputs.speed**2

    # Geometry (from wing_loading, or fixed span when use_hardcoded_values + hardcode_wing_span; see geometry.wing_geometry).
    S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC = wing_geometry(L_wing, inputs)

    if S <= 0:
        raise ValueError(
            f"Configuration invalid: Calculated wing area is non-positive ({S:.6f} m²). "
            f"This occurs when L_wing ({L_wing:.2f} N) is negative or zero. "
            f"W0_N: {W0_N:.2f} N, L_tail: {L_tail:.2f} N"
        )

    # Aerodynamics
    aero_wing = wing_aerodynamic_coeffs(L_wing, L_tail, S, MAC, AR, inputs)

    if aero_wing['V_stall'] > 0.75 * inputs.speed:
        raise ValueError(
            f"Configuration is invalid due to the large stall speed. "
            f"Stall speed ({aero_wing['V_stall']:.2f} m/s) exceeds 0.75 * cruise speed ({0.75 * inputs.speed:.2f} m/s)."
        )

    # Structure
    wing_structure, _, wing_spar_results = calculate_wing_structure(
        L_wing=L_wing,
        S=S,
        c_root=c_root,
        taper_ratio=inputs.taper_ratio,
        b=b,
        c_MGC=c_MGC,
        y_MAC=y_MAC,
        max_thickness_fraction=aero_wing['max_thickness_fraction_of_chord'],
        CM_max=aero_wing['CM_max'],
        inputs=inputs,
        propulsion_selection=propulsion_selection,
    )
    # The structural calculation is now the default mass model.  The aggregate
    # hardcode remains available for later CAD-measured mass replacement.
    W_wings_calc = 2.0 * float(wing_structure.W_total_kg)
    W_wings_total = _get_hardcoded_value(inputs, 'hardcode_wing_weight_kg', W_wings_calc)
    if abs(W_wings_total - W_wings_calc) > 1e-12:
        wing_structure.W_total_kg = W_wings_total / 2.0

    # Wing AC position
    wing_le_x = inputs.cg_x_wing - c_MGC / 2.0
    x_ac_wing = wing_le_x + ac_on_MAC

    M_ac_wing = aero_wing['CM_cruise'] * q * S * MAC
    M_cg_wing = M_ac_wing

    geometry = Geometry(
        area_m2=S,
        span_m=b,
        c_root_m=c_root,
        c_tip_m=c_tip,
        MAC_m=MAC,
        y_MAC_from_centerline_m=y_MAC,
        ac_on_MAC_m=ac_on_MAC,
        AR=AR,
        c_MGC_m=c_MGC,
    )

    aerodynamics = Aerodynamics(
        CL_cruise=aero_wing['CL_cruise'],
        CL_max=aero_wing['CL_max'],
        CD0=aero_wing['CD0'],
        CD_cruise=aero_wing['CD_cruise'],
        CD0_fuselage=aero_wing['CD0_fuselage'],
        CM_cruise=aero_wing['CM_cruise'],
        CM_max=aero_wing['CM_max'],
        alpha_cruise_deg=aero_wing['alpha_cruise'],
        L_over_D_cruise=aero_wing['L_over_D_cruise'],
        V_stall=aero_wing['V_stall'],
        Re_cruise=aero_wing['Re_cruise'],
        max_thickness_fraction_of_chord=aero_wing['max_thickness_fraction_of_chord'],
        CL_alpha_3D_per_rad=aero_wing["CL_alpha_3D_per_rad"],
    )

    return geometry, aerodynamics, wing_structure, x_ac_wing, M_cg_wing, W_wings_total, wing_spar_results

def _calculate_tail_system(
    x_cg: float,
    wing_MAC: float,
    wing_S: float,
    wing_alpha_cruise: float,
    wing_M_cg: float,
    wing_CD_cruise: float,
    inputs: DesignInputs,
    propulsion_selection: Optional[Dict[str, Any]] = None,
) -> Tuple[Geometry, Aerodynamics, Structure, float, float, Dict[str, Any], bool, Dict[str, float], Dict[str, float]]:

    # Downwash (you already switched to wing-alpha based)
    downwash_deg = inputs.tail_downwash_offset_deg + inputs.tail_downwash_k * wing_alpha_cruise
    downwash_deg = max(inputs.tail_downwash_min_deg, min(inputs.tail_downwash_max_deg, downwash_deg))

    q = 0.5 * ATMOSPHERIC.rho * inputs.speed**2

    # Base tail geometry: volume ratio, or fixed span when use_hardcoded_values + hardcode_tail_span (see geometry.tail_geometry).
    S_tail_nom, b_tail_nom, c_root_tail, c_tip_tail, MAC_tail, y_MAC_tail, AR_tail_nom, c_MGC_tail, ac_on_MAC_tail = tail_geometry(
        x_cg, wing_MAC, wing_S, inputs
    )

    x_ac_tail = inputs.cg_x_tail - 0.5 * c_root_tail + ac_on_MAC_tail

    # Required tail lift for trim (initial)
    arm = (x_ac_tail - x_cg)
    if abs(arm) < 1e-9:
        raise ValueError("Tail moment arm too small or zero (x_ac_tail == x_cg).")

    L_tail = wing_M_cg / arm

    # --- Prop clearance check
    # The legacy tailsitter mounted propellers adjacent to the tail. The new aircraft has
    # four wing/boom-mounted lift propellers and one central aft pusher, so tail-span
    # sizing must not be driven by this old clearance rule.
    if bool(getattr(inputs, "tail_mounted_propulsion", True)):
        prop_R = _get_prop_radius_m(propulsion_selection)
        clearance_met = _prop_clearance_met(
            prop_radius_m=prop_R,
            y_fraction=float(inputs.prop_y_position_fraction),
            span_m=float(b_tail_nom),
            clearance_m=float(getattr(inputs, "prop_fuselage_clearance_m", 0.02)),
        )
    else:
        prop_R = 0.0
        clearance_met = True

    # if not clearance_met:
    #     raise ValueError(f"Prop clearance not met. Try increasing tail volume ratio.\n"
    #     f"  half_b_tail_nom = {b_tail_nom/2:.2f} m < {(inputs.prop_fuselage_clearance_m + prop_R) / inputs.prop_y_position_fraction:.2f} m\n"
    #     f"  prop_R = {prop_R:.2f} m\n"
    #     f"  inputs.prop_fuselage_clearance_m = {inputs.prop_fuselage_clearance_m:.2f} m\n"
    #     f"  inputs.prop_y_position_fraction = {inputs.prop_y_position_fraction:.2f}\n"
    # )

    # Treat inputs.tail_span_extension_m as minimum baseline extension the user wants
    ext_min_user = float(getattr(inputs, "tail_span_extension_m", 0.0) or 0.0)

    # Iterate tail area scaling until incidence + elevator constraints satisfied
    aero_tail = None

    # lam = float(inputs.tail_taper_ratio)
    aero_try = tail_aerodynamic_coeffs(
        L_required_N=L_tail,
        S=S_tail_nom,
        MAC=MAC_tail,
        AR=AR_tail_nom,
        alpha_body_deg=wing_alpha_cruise,
        downwash_deg=downwash_deg,
        inputs=inputs,
    )

    # if not aero_try["feasible"]:
    #     raise ValueError(
    #         "Tail trim not achievable with current geometry.\n"
    #         f"{aero_try['failure_reason']}\n"
    #         "Try: increase tail volume ratio, move CG aft, move tail further back, "
    #         "reduce wing Cm, or relax tail effective-alpha demand."
    #     )

    aero_tail = aero_try
    
    # Structure
    tail_structure, _, elevator_rod_results, tail_spar_results = calculate_tail_structure(
        L_tail=abs(L_tail),
        S=S_tail_nom,
        c_root=c_root_tail,
        taper_ratio=inputs.tail_taper_ratio,
        b=b_tail_nom,
        c_MGC=c_MGC_tail,
        y_MAC=y_MAC_tail,
        max_thickness_fraction=aero_tail["max_thickness_fraction_of_chord"],
        CM_max=aero_tail["CM_max"],
        inputs=inputs,
        propulsion_selection=propulsion_selection,
    )
    W_tails_calc = 2.0 * float(tail_structure.W_total_kg)
    W_tails_total = _get_hardcoded_value(inputs, 'hardcode_tail_weight_kg', W_tails_calc)
    if abs(W_tails_total - W_tails_calc) > 1e-12:
        tail_structure.W_total_kg = W_tails_total / 2.0

    geometry = Geometry(
        area_m2=S_tail_nom,
        span_m=b_tail_nom,
        c_root_m=c_root_tail,
        c_tip_m=c_tip_tail,
        MAC_m=MAC_tail,
        y_MAC_from_centerline_m=y_MAC_tail,
        ac_on_MAC_m=ac_on_MAC_tail,
        AR=AR_tail_nom,
        c_MGC_m=c_MGC_tail,
    )

    aerodynamics = Aerodynamics(
        CL_cruise=aero_tail["CL_cruise"],
        CL_max=aero_tail["CL_max"],
        CD0=aero_tail["CD0"],
        CD_cruise=aero_tail["CD_cruise"],
        CD0_fuselage=0.0,
        CM_cruise=aero_tail["CM_cruise"],
        CM_max=aero_tail["CM_max"],
        alpha_cruise_deg=aero_tail["alpha_equiv_deg"],
        L_over_D_cruise=aero_tail["L_over_D_cruise"],
        V_stall=aero_tail["V_stall"],
        Re_cruise=aero_tail["Re_cruise"],
        max_thickness_fraction_of_chord=aero_tail["max_thickness_fraction_of_chord"],
        CL_alpha_3D_per_rad=aero_tail["CL_alpha_3D_per_rad"],
    )

    aero_tail = dict(aero_tail)
    aero_tail["tail_span_total_m"] = float(b_tail_nom)
    aero_tail["tail_area_total_m2"] = float(S_tail_nom)
    aero_tail["prop_radius_m"] = float(prop_R)

    return geometry, aerodynamics, tail_structure, x_ac_tail, L_tail, aero_tail, clearance_met, elevator_rod_results, tail_spar_results

# VERTICAL MAIN WING REMOVED FOR NOW
# def _calculate_vertical_wing_system(
#     wing_geometry: Geometry,
#     inputs: DesignInputs,
# ) -> Tuple[Geometry, Aerodynamics, Structure, float, float]:
#     """
#     Vertical wing is identical in size to horizontal wing but uses NACA0012 airfoil.
#     """
#     S = wing_geometry.area_m2
#     b = wing_geometry.span_m
#     c_root = wing_geometry.c_root_m
#     c_tip = wing_geometry.c_tip_m
#     MAC = wing_geometry.MAC_m
#     y_MAC = wing_geometry.y_MAC_from_centerline_m
#     AR = wing_geometry.AR
#     c_MGC = wing_geometry.c_MGC_m
#     ac_on_MAC = wing_geometry.ac_on_MAC_m
# 
#     aero_vertical_wing = vertical_wing_aerodynamic_coeffs(S, MAC, AR, inputs)
# 
#     L_vertical_wing = 0.0
#     vertical_wing_structure, W_vertical_wings = calculate_wing_structure(
#         L_wing=L_vertical_wing,
#         S=S,
#         c_root=c_root,
#         taper_ratio=inputs.taper_ratio,
#         b=b,
#         c_MGC=c_MGC,
#         y_MAC=y_MAC,
#         max_thickness_fraction=aero_vertical_wing['max_thickness_fraction_of_chord'],
#         CM_max=aero_vertical_wing['CM_max'],
#         inputs=inputs,
#     )
# 
#     wing_le_x = inputs.cg_x_wing - c_MGC / 2.0
#     x_ac_vertical_wing = wing_le_x + ac_on_MAC
# 
#     geometry = Geometry(
#         area_m2=S,
#         span_m=b,
#         c_root_m=c_root,
#         c_tip_m=c_tip,
#         MAC_m=MAC,
#         y_MAC_from_centerline_m=y_MAC,
#         ac_on_MAC_m=ac_on_MAC,
#         AR=AR,
#         c_MGC_m=c_MGC,
#     )
# 
#     aerodynamics = Aerodynamics(
#         CL_cruise=aero_vertical_wing['CL_cruise'],
#         CL_max=aero_vertical_wing['CL_max'],
#         CD0=aero_vertical_wing['CD0'],
#         CD_cruise=aero_vertical_wing['CD_cruise'],
#         CD0_fuselage=aero_vertical_wing['CD0_fuselage'],
#         CM_cruise=aero_vertical_wing['CM_cruise'],
#         CM_max=aero_vertical_wing['CM_max'],
#         alpha_cruise_deg=aero_vertical_wing['alpha_cruise'],
#         L_over_D_cruise=aero_vertical_wing['L_over_D_cruise'],
#         V_stall=aero_vertical_wing['V_stall'],
#         Re_cruise=aero_vertical_wing['Re_cruise'],
#         max_thickness_fraction_of_chord=aero_vertical_wing['max_thickness_fraction_of_chord'],
#     )
# 
#     return geometry, aerodynamics, vertical_wing_structure, x_ac_vertical_wing, W_vertical_wings


def _calculate_vertical_tail_system(
    x_cg: float,
    wing_geometry_obj: Geometry,
    inputs: DesignInputs,
) -> Tuple[Geometry, Aerodynamics, Structure, float, float, Dict[str, float], Dict[str, float]]:
    """
    Vertical tail is sized from vertical_tail_volume_ratio (Vv), or fixed by
    hardcode_vertical_tail_span when hardcoded overrides are enabled.
    Directional stability derivative Cn_beta is computed and returned for reporting.
    """
    # 1) Geometry from Vv
    Sv, b_v, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC = vertical_tail_geometry(
        x_cg=x_cg,
        b_wing=wing_geometry_obj.span_m,
        S_wing=wing_geometry_obj.area_m2,
        inputs=inputs,
    )

    # 2) Aero (still “no lift at cruise” in your current simplified model)
    aero_vertical_tail = vertical_tail_aerodynamic_coeffs(Sv, MAC, AR, inputs)

    # 3) Structure (no required sideforce in this simplified cruise point)
    L_vertical_tail = 0.0
    vertical_tail_structure, _, _, vertical_tail_spar_results = calculate_tail_structure(
        L_tail=abs(L_vertical_tail),
        S=Sv,
        c_root=c_root,
        taper_ratio=inputs.tail_taper_ratio,
        b=b_v,
        c_MGC=c_MGC,
        y_MAC=y_MAC,
        max_thickness_fraction=aero_vertical_tail["max_thickness_fraction_of_chord"],
        CM_max=aero_vertical_tail["CM_max"],
        inputs=inputs,
        propulsion_selection=None,
        skin_root_thickness_hardcode_attr="hardcode_vertical_tail_skin_root_thickness_m",
        spar_primary_weight_hardcode_attr="hardcode_vertical_tail_wing_spar_weight_kg",
        spar_secondary_weight_hardcode_attr="hardcode_rudder_rod_weight_kg",
    )
    W_vertical_tails_calc = 2.0 * float(vertical_tail_structure.W_total_kg)
    W_vertical_tails_total = _get_hardcoded_value(inputs, 'hardcode_vertical_tail_weight_kg', W_vertical_tails_calc)
    if abs(W_vertical_tails_total - W_vertical_tails_calc) > 1e-12:
        vertical_tail_structure.W_total_kg = W_vertical_tails_total / 2.0

    x_ac_vertical_tail = inputs.cg_x_vertical_tail - 0.5 * c_root + ac_on_MAC

    # 4) Directional stability derivative
    dir_stab = _compute_directional_stability(
        x_cg_m=x_cg,
        x_ac_vertical_tail_m=x_ac_vertical_tail,
        wing_span_m=wing_geometry_obj.span_m,
        wing_S_m2=wing_geometry_obj.area_m2,
        vertical_tail_S_m2=float(inputs.n_vertical_tail_surfaces) * Sv,
        a_v_per_rad=aero_vertical_tail["CL_alpha_3D_per_rad"],
        inputs=inputs,
    )

    geometry = Geometry(
        area_m2=Sv,
        span_m=b_v,
        c_root_m=c_root,
        c_tip_m=c_tip,
        MAC_m=MAC,
        y_MAC_from_centerline_m=y_MAC,
        ac_on_MAC_m=ac_on_MAC,
        AR=AR,
        c_MGC_m=c_MGC,
    )

    aerodynamics = Aerodynamics(
        CL_cruise=aero_vertical_tail["CL_cruise"],
        CL_max=aero_vertical_tail["CL_max"],
        CD0=aero_vertical_tail["CD0"],
        CD_cruise=aero_vertical_tail["CD_cruise"],
        CD0_fuselage=aero_vertical_tail["CD0_fuselage"],
        CM_cruise=aero_vertical_tail["CM_cruise"],
        CM_max=aero_vertical_tail["CM_max"],
        alpha_cruise_deg=aero_vertical_tail["alpha_cruise"],
        L_over_D_cruise=aero_vertical_tail["L_over_D_cruise"],
        V_stall=aero_vertical_tail["V_stall"],
        Re_cruise=aero_vertical_tail["Re_cruise"],
        max_thickness_fraction_of_chord=aero_vertical_tail["max_thickness_fraction_of_chord"],
        CL_alpha_3D_per_rad=aero_vertical_tail["CL_alpha_3D_per_rad"],
    )

    return geometry, aerodynamics, vertical_tail_structure, x_ac_vertical_tail, W_vertical_tails_total, dir_stab, vertical_tail_spar_results


def _wing_downwash_at_tail_deg(
    *,
    L_wing_N: float,
    wing_S_m2: float,
    wing_AR: float,
    inputs: DesignInputs,
) -> float:
    """
    Simple wing->tail downwash model (guide-consistent):
    epsilon_deg = k * CL_wing  (default)  OR  epsilon_deg = k * alpha_wing_deg

    This is intentionally simple and stable for iteration.
    """
    V = inputs.speed
    q = 0.5 * ATMOSPHERIC.rho * V**2

    eps = inputs.tail_downwash_offset_deg

    k = inputs.tail_downwash_k

    if wing_S_m2 > 0 and q > 0:
        CL_w = L_wing_N / (q * wing_S_m2)
    else:
        CL_w = 0.0

    eps += k * CL_w

    eps_min = inputs.tail_downwash_min_deg
    eps_max = inputs.tail_downwash_max_deg
    eps = max(eps_min, min(eps_max, eps))
    return eps

def _get_prop_radius_m(propulsion_selection: Optional[Dict[str, Any]]) -> float:
    """
    Extract prop radius [m] from propulsion_selection["combo"].
    Tries common keys. Returns 0 if unknown.
    """
    if propulsion_selection is None:
        return 0.0

    combo = propulsion_selection.get("combo", {}) if isinstance(propulsion_selection, dict) else {}
    if not isinstance(combo, dict):
        return 0.0

    # Try meters first
    for k in ("prop_diameter_m", "prop_diam_m", "diameter_m"):
        if k in combo and combo[k] is not None:
            d = float(combo[k])
            return max(0.0, 0.5 * d)

    # Try inches
    for k in ("prop_diameter_in", "prop_diameter_inch", "prop_diameter_inches", "diameter_in"):
        if k in combo and combo[k] is not None:
            d_in = float(combo[k])
            d_m = d_in * 0.0254
            return max(0.0, 0.5 * d_m)

    return 0.0

def _prop_clearance_met(
    *,
    prop_radius_m: float,
    y_fraction: float,
    clearance_m: float,
    span_m: float,
) -> float:
    """
    Required TOTAL span b such that prop disk clears:
      1) fuselage: y_abs - R >= fuselage_OR + clearance
      2) tip:      y_abs + R <= (b/2) - clearance
    where y_abs = y_fraction * (b/2), y_fraction in (0..1)
    """
    R = max(0.0, float(prop_radius_m))

    clearance_met = span_m*y_fraction/2 - R >= clearance_m

    return clearance_met

def _compute_static_margin(
    *,
    x_cg_m: float,
    x_ac_wing_m: float,
    x_ac_tail_m: float,
    wing_MAC_m: float,
    wing_S_m2: float,
    tail_S_m2: float,
    a_w_per_rad: float,
    a_t_per_rad: float,
    eta_tail: float,
    inputs: DesignInputs,
    alpha_wing_deg: float,
) -> Dict[str, float]:
    """
    Computes longitudinal neutral point and static margin (dimensionless and %MAC)
    using a conventional linear approximation:
      x_np = x_ac_w + Vh * eta * (a_t/a_w) * (1 - dε/dα) * MAC
      SM = (x_np - x_cg)/MAC

    dε/dα is taken from your downwash model:
      ε = offset + k * alpha_wing_deg, clamped.
    If clamped, derivative is treated as 0.
    """
    MAC = float(wing_MAC_m)
    if MAC <= 0.0 or wing_S_m2 <= 0.0:
        return {"x_np_m": float("nan"), "SM": float("nan"), "SM_percent_MAC": float("nan")}

    l_t = float(x_ac_tail_m) - float(x_ac_wing_m)  # wing AC -> tail AC
    Vh = (float(tail_S_m2) * l_t) / (float(wing_S_m2) * MAC)

    # Downwash derivative from your model (deg/deg ~= rad/rad)
    eps_raw = float(inputs.tail_downwash_offset_deg) + float(inputs.tail_downwash_k) * float(alpha_wing_deg)
    eps_clamped = max(float(inputs.tail_downwash_min_deg), min(float(inputs.tail_downwash_max_deg), eps_raw))

    if abs(eps_clamped - eps_raw) > 1e-9:
        d_eps_d_alpha = 0.0
    else:
        d_eps_d_alpha = float(inputs.tail_downwash_k)

    # Protect against divide-by-zero
    if abs(a_w_per_rad) < 1e-12:
        return {"x_np_m": float("nan"), "SM": float("nan"), "SM_percent_MAC": float("nan")}

    x_np = float(x_ac_wing_m) + Vh * float(eta_tail) * (float(a_t_per_rad) / float(a_w_per_rad)) * (1.0 - d_eps_d_alpha) * MAC
    SM = (x_np - float(x_cg_m)) / MAC

    return {
        "x_np_m": x_np,
        "SM": SM,
        "SM_percent_MAC": 100.0 * SM,
        "Vh": Vh,
        "d_eps_d_alpha": d_eps_d_alpha,
        "l_t_m": l_t,
    }

def _compute_directional_stability(
    *,
    x_cg_m: float,
    x_ac_vertical_tail_m: float,
    wing_span_m: float,
    wing_S_m2: float,
    vertical_tail_S_m2: float,
    a_v_per_rad: float,
    inputs: DesignInputs,
) -> Dict[str, float]:
    """
    Eq. 6.73-style approximation:
      Cn_beta ≈ Kf1 * a_v * (1 - dσ/dβ) * η_v * (l_vt * Sv)/(b * S)
            = Kf1 * a_v * (1 - dσ/dβ) * η_v * Vv

    Returns Cn_beta (per rad), Vv, and l_vt.
    """
    b = float(wing_span_m)
    S = float(wing_S_m2)
    Sv = float(vertical_tail_S_m2)

    if b <= 0.0 or S <= 0.0:
        return {"Cn_beta": float("nan"), "Vv": float("nan"), "l_vt_m": float("nan")}

    l_vt = float(x_ac_vertical_tail_m) - float(x_cg_m)  # CG -> VT AC (requested assumption)
    if l_vt <= 1e-9:
        return {"Cn_beta": float("nan"), "Vv": float("nan"), "l_vt_m": l_vt}

    Vv = (Sv * l_vt) / (b * S)

    Kf1 = float(getattr(inputs, "vertical_tail_Kf1", 0.75))
    eta_v = float(getattr(inputs, "vertical_tail_eta_v", 1.0))
    dsigma_dbeta = float(getattr(inputs, "vertical_tail_dsigma_dbeta", 0.0))

    Cn_beta = Kf1 * float(a_v_per_rad) * (1.0 - dsigma_dbeta) * eta_v * Vv

    return {"Cn_beta": Cn_beta, "Vv": Vv, "l_vt_m": l_vt}

def _assemble_results(
    W0_mass: float,
    W0_N: float,
    L_wing: float,
    x_cg: float,
    drag_cruise: float,
    cruise_power: float,
    cruise_power_elec: float,
    drag_breakdown: Dict[str, float],
    wing_geometry: Geometry,
    wing_aerodynamics: Aerodynamics,
    wing_structure: Structure,
    x_ac_wing: float,
    wing_moment: float,
    wing_spar_results: Dict[str, float],
    tail_geometry: Geometry,
    tail_aerodynamics: Aerodynamics,
    tail_structure: Structure,
    x_ac_tail: float,
    L_tail: float,
    tail_aero_dict: Dict[str, Any],
    elevator_rod_results: Dict[str, float],
    tail_spar_results: Dict[str, float],
    # vertical_wing_geometry: Geometry,
    # vertical_wing_aerodynamics: Aerodynamics,
    # vertical_wing_structure: Structure,
    # x_ac_vertical_wing: float,
    vertical_tail_geometry: Geometry,
    vertical_tail_aerodynamics: Aerodynamics,
    vertical_tail_structure: Structure,
    x_ac_vertical_tail: float,
    vertical_tail_spar_results: Dict[str, float],
    fuselage_t_shell: float,
    W_fuselage: float,
    W_fuselage_nose: float,
    W_fuselage_section_C: float,
    W_fuselage_section_B: float,
    W_fuselage_section_A: float,
    W_fuselage_tail: float,
    W_booms: float,
    boom_details: Dict[str, float],
    battery_results: Optional[Any],
    propulsion_results: Optional[Any],
    avionics_results: AvionicsResults,
    mass_properties: Dict[str, Any],
    inputs: DesignInputs,
    fuselage_details: Optional[Dict[str, float]] = None,
) -> DesignResults:

    wing_wetted_m2 = effective_lifting_surface_wetted_area_m2(
        inputs, "hardcode_wing_wetted_area_m2", wing_geometry.area_m2
    )
    tail_wetted_m2 = effective_lifting_surface_wetted_area_m2(
        inputs, "hardcode_tail_wetted_area_m2", tail_geometry.area_m2
    )
    vertical_tail_wetted_m2 = effective_lifting_surface_wetted_area_m2(
        inputs,
        "hardcode_vertical_tail_wetted_area_m2",
        float(inputs.n_vertical_tail_surfaces) * vertical_tail_geometry.area_m2,
    )
    fuselage_wetted_m2 = float(inputs.fuselage_wetted_area)

    wing_results = WingResults(
        geometry=wing_geometry,
        aerodynamics=wing_aerodynamics,
        structure=wing_structure,
        x_ac_wing_m=x_ac_wing,
        moment_at_cg_Nm=wing_moment,
        wing_spar_length_m=wing_spar_results.get("L_spar_m", 0.0),
        wing_spar_M_max_Nm=wing_spar_results.get("M_max_Nm", 0.0),
        wing_spar_sigma_allow_MPa=wing_spar_results.get("sigma_allow_MPa", 0.0),
        wing_spar_sigma_root_MPa=wing_spar_results.get("sigma_root_MPa", 0.0),
        wing_spar_OD_root_m=wing_spar_results.get("OD_root_m", 0.0),
        wing_spar_ID_root_m=wing_spar_results.get("ID_root_m", 0.0),
        wing_spar_OD_tip_m=wing_spar_results.get("OD_tip_m", 0.0),
        wing_spar_ID_tip_m=wing_spar_results.get("ID_tip_m", 0.0),
        wing_spar_A_root_mm2=wing_spar_results.get("A_root_mm2", 0.0),
        wing_spar_A_tip_mm2=wing_spar_results.get("A_tip_mm2", 0.0),
        wing_spar_I_root_mm4=wing_spar_results.get("I_root_mm4", 0.0),
        wing_spar_I_tip_mm4=wing_spar_results.get("I_tip_mm4", 0.0),
        wing_spar_mass_kg=wing_spar_results.get("W_spar_kg", 0.0),
        wing_spar_N_ribs=wing_spar_results.get("N_ribs", 0.0),
        wing_spar_rib_contact_L_m=wing_spar_results.get("rib_contact_L_m", 0.0),
        wing_spar_bearing_allow_MPa=wing_spar_results.get("bearing_allow_MPa", 0.0),
        wing_spar_bearing_stress_MPa=wing_spar_results.get("bearing_stress_MPa", 0.0),
        wing_spar_bearing_ok=wing_spar_results.get("bearing_ok", 0.0),
        wing_stiffener_N=wing_spar_results.get("N_stiffeners", 0.0),
        wing_stiffener_spacing_m=wing_spar_results.get("stiffener_spacing_m", 0.0),
        wing_stiffener_thickness_m=wing_spar_results.get("stiffener_thickness_m", 0.0),
        wing_stiffener_height_m=wing_spar_results.get("stiffener_height_m", 0.0),
        wing_stiffener_area_one_mm2=wing_spar_results.get("stiffener_area_one_mm2", 0.0),
        wing_stiffener_I_one_mm4=wing_spar_results.get("stiffener_I_one_mm4", 0.0),
        wing_stiffener_tau_skin_MPa=wing_spar_results.get("stiffener_tau_skin_MPa", 0.0),
        wing_stiffener_tau_cr_panel_MPa=wing_spar_results.get("stiffener_tau_cr_panel_MPa", 0.0),
        wing_stiffener_q_shear_N_per_m=wing_spar_results.get("stiffener_q_shear_N_per_m", 0.0),
        wing_stiffener_aspect_ratio=wing_spar_results.get("stiffener_aspect_ratio", 0.0),
        wing_stiffener_mass_kg=wing_spar_results.get("W_stiffeners_kg", 0.0),
        wing_stiffener_panel_ok=wing_spar_results.get("stiffener_panel_ok", 0.0),
        wing_spar_tip_deflection_m=wing_spar_results.get("tip_deflection_m", 0.0),
        wing_spar_tip_deflection_mm=wing_spar_results.get("tip_deflection_mm", 0.0),
        wing_spar_root_slope_rad=wing_spar_results.get("root_slope_rad", 0.0),
        wing_spar_root_slope_deg=wing_spar_results.get("root_slope_deg", 0.0),
        wetted_area_m2=wing_wetted_m2,
        lift_mount_mass_total_kg=wing_spar_results.get("lift_mount_mass_total_kg", 0.0),
        lift_thrust_per_motor_structural_N=wing_spar_results.get("lift_thrust_per_motor_structural_N", 0.0),
        lift_weight_root_moment_Nm_half=wing_spar_results.get("lift_weight_root_moment_Nm_half", 0.0),
        lift_thrust_root_moment_Nm_half=wing_spar_results.get("lift_thrust_root_moment_Nm_half", 0.0),
        lift_shaft_torque_Nm_half=wing_spar_results.get("lift_shaft_torque_Nm_half", 0.0),
        lift_offset_torsion_Nm_half=wing_spar_results.get("lift_offset_torsion_Nm_half", 0.0),
    )

    tail_results = TailResults(
        geometry=tail_geometry,
        aerodynamics=tail_aerodynamics,
        structure=tail_structure,
        x_ac_tail_m=x_ac_tail,
        lift_N=L_tail,
        tail_incidence_deg=tail_aero_dict["tail_incidence_deg"],
        cruise_elevator_deg=tail_aero_dict["delta_elev_deg"],
        CL_req=tail_aero_dict["CL_req"],
        CL_no_elev=tail_aero_dict["CL_no_elev"],
        dCL_required=tail_aero_dict["dCL_required"],
        elevator_chord_ratio=tail_aero_dict["elevator_chord_ratio"],
        eta_f=tail_aero_dict.get("tau_e", 0.0),
        elevator_deflection_deg=tail_aero_dict["delta_elev_deg"],
        alpha_body_deg=tail_aero_dict["alpha_body_deg"],
        alpha_equiv_deg=tail_aero_dict["alpha_equiv_deg"],
        elevator_rod_span_frac=elevator_rod_results.get("span_frac", 0.0),
        elevator_rod_length_m=elevator_rod_results.get("L_rod_m", 0.0),
        elevator_rod_F_elev_N=elevator_rod_results.get("F_elev_N", 0.0),
        elevator_rod_M_max_Nm=elevator_rod_results.get("M_max_rod_Nm", 0.0),
        elevator_rod_sigma_allow_MPa=elevator_rod_results.get("sigma_allow_MPa", 0.0),
        elevator_rod_sigma_rod_MPa=elevator_rod_results.get("sigma_rod_MPa", 0.0),
        elevator_rod_OD_m=elevator_rod_results.get("OD_m", 0.0),
        elevator_rod_ID_m=elevator_rod_results.get("ID_m", 0.0),
        elevator_rod_area_mm2=elevator_rod_results.get("A_rod_mm2", 0.0),
        elevator_rod_I_mm4=elevator_rod_results.get("I_rod_mm4", 0.0),
        elevator_rod_mass_kg=elevator_rod_results.get("W_rod_kg", 0.0),
        elevator_rod_N_support=elevator_rod_results.get("N_support", 0.0),
        elevator_rod_contact_L_m=elevator_rod_results.get("rod_contact_L_m", 0.0),
        elevator_rod_bearing_allow_MPa=elevator_rod_results.get("rod_bearing_allow_MPa", 0.0),
        elevator_hinge_moment_M_h_Nm=tail_aero_dict.get("elevator_hinge_moment_M_h_Nm", 0.0),
        elevator_area_S_e_m2=tail_aero_dict.get("elevator_area_S_e_m2", 0.0),
        elevator_chord_c_e_m=tail_aero_dict.get("elevator_chord_c_e_m", 0.0),
        servo_torque_T_servo_Nm=tail_aero_dict.get("servo_torque_T_servo_Nm", 0.0),
        servo_torque_with_margin_Nm=tail_aero_dict.get("servo_torque_with_margin_Nm", 0.0),
        tail_spar_length_m=tail_spar_results.get("L_spar_m", 0.0),
        tail_spar_M_max_Nm=tail_spar_results.get("M_max_Nm", 0.0),
        tail_spar_sigma_allow_MPa=tail_spar_results.get("sigma_allow_MPa", 0.0),
        tail_spar_sigma_root_MPa=tail_spar_results.get("sigma_root_MPa", 0.0),
        tail_spar_OD_root_m=tail_spar_results.get("OD_root_m", 0.0),
        tail_spar_ID_root_m=tail_spar_results.get("ID_root_m", 0.0),
        tail_spar_OD_tip_m=tail_spar_results.get("OD_tip_m", 0.0),
        tail_spar_ID_tip_m=tail_spar_results.get("ID_tip_m", 0.0),
        tail_spar_A_root_mm2=tail_spar_results.get("A_root_mm2", 0.0),
        tail_spar_A_tip_mm2=tail_spar_results.get("A_tip_mm2", 0.0),
        tail_spar_I_root_mm4=tail_spar_results.get("I_root_mm4", 0.0),
        tail_spar_I_tip_mm4=tail_spar_results.get("I_tip_mm4", 0.0),
        tail_spar_mass_kg=tail_spar_results.get("W_spar_kg", 0.0),
        tail_spar_N_ribs=tail_spar_results.get("N_ribs", 0.0),
        tail_spar_rib_contact_L_m=tail_spar_results.get("rib_contact_L_m", 0.0),
        tail_spar_bearing_allow_MPa=tail_spar_results.get("bearing_allow_MPa", 0.0),
        tail_spar_bearing_stress_MPa=tail_spar_results.get("bearing_stress_MPa", 0.0),
        tail_spar_bearing_ok=tail_spar_results.get("bearing_ok", 0.0),
        tail_stiffener_N=tail_spar_results.get("N_stiffeners", 0.0),
        tail_stiffener_spacing_m=tail_spar_results.get("stiffener_spacing_m", 0.0),
        tail_stiffener_thickness_m=tail_spar_results.get("stiffener_thickness_m", 0.0),
        tail_stiffener_height_m=tail_spar_results.get("stiffener_height_m", 0.0),
        tail_stiffener_area_one_mm2=tail_spar_results.get("stiffener_area_one_mm2", 0.0),
        tail_stiffener_I_one_mm4=tail_spar_results.get("stiffener_I_one_mm4", 0.0),
        tail_stiffener_tau_skin_MPa=tail_spar_results.get("stiffener_tau_skin_MPa", 0.0),
        tail_stiffener_tau_cr_panel_MPa=tail_spar_results.get("stiffener_tau_cr_panel_MPa", 0.0),
        tail_stiffener_q_shear_N_per_m=tail_spar_results.get("stiffener_q_shear_N_per_m", 0.0),
        tail_stiffener_aspect_ratio=tail_spar_results.get("stiffener_aspect_ratio", 0.0),
        tail_stiffener_mass_kg=tail_spar_results.get("W_stiffeners_kg", 0.0),
        tail_stiffener_panel_ok=tail_spar_results.get("stiffener_panel_ok", 0.0),
        tail_spar_tip_deflection_m=tail_spar_results.get("tip_deflection_m", 0.0),
        tail_spar_tip_deflection_mm=tail_spar_results.get("tip_deflection_mm", 0.0),
        tail_spar_root_slope_rad=tail_spar_results.get("root_slope_rad", 0.0),
        tail_spar_root_slope_deg=tail_spar_results.get("root_slope_deg", 0.0),
        wetted_area_m2=tail_wetted_m2,
    )

    # VERTICAL MAIN WING REMOVED FOR NOW
    # vertical_wing_results = VerticalWingResults(
    #     geometry=vertical_wing_geometry,
    #     aerodynamics=vertical_wing_aerodynamics,
    #     structure=vertical_wing_structure,
    #     x_ac_wing_m=x_ac_vertical_wing,
    # )

    vertical_tail_results = VerticalTailResults(
        geometry=vertical_tail_geometry,
        aerodynamics=vertical_tail_aerodynamics,
        structure=vertical_tail_structure,
        x_ac_tail_m=x_ac_vertical_tail,
        vertical_tail_spar_length_m=vertical_tail_spar_results.get("L_spar_m", 0.0),
        vertical_tail_spar_M_max_Nm=vertical_tail_spar_results.get("M_max_Nm", 0.0),
        vertical_tail_spar_sigma_allow_MPa=vertical_tail_spar_results.get("sigma_allow_MPa", 0.0),
        vertical_tail_spar_sigma_root_MPa=vertical_tail_spar_results.get("sigma_root_MPa", 0.0),
        vertical_tail_spar_OD_root_m=vertical_tail_spar_results.get("OD_root_m", 0.0),
        vertical_tail_spar_ID_root_m=vertical_tail_spar_results.get("ID_root_m", 0.0),
        vertical_tail_spar_OD_tip_m=vertical_tail_spar_results.get("OD_tip_m", 0.0),
        vertical_tail_spar_ID_tip_m=vertical_tail_spar_results.get("ID_tip_m", 0.0),
        vertical_tail_spar_A_root_mm2=vertical_tail_spar_results.get("A_root_mm2", 0.0),
        vertical_tail_spar_A_tip_mm2=vertical_tail_spar_results.get("A_tip_mm2", 0.0),
        vertical_tail_spar_I_root_mm4=vertical_tail_spar_results.get("I_root_mm4", 0.0),
        vertical_tail_spar_I_tip_mm4=vertical_tail_spar_results.get("I_tip_mm4", 0.0),
        vertical_tail_spar_mass_kg=vertical_tail_spar_results.get("W_spar_kg", 0.0),
        vertical_tail_spar_N_ribs=vertical_tail_spar_results.get("N_ribs", 0.0),
        vertical_tail_spar_rib_contact_L_m=vertical_tail_spar_results.get("rib_contact_L_m", 0.0),
        vertical_tail_spar_bearing_allow_MPa=vertical_tail_spar_results.get("bearing_allow_MPa", 0.0),
        vertical_tail_spar_bearing_stress_MPa=vertical_tail_spar_results.get("bearing_stress_MPa", 0.0),
        vertical_tail_spar_bearing_ok=vertical_tail_spar_results.get("bearing_ok", 0.0),
        vertical_tail_stiffener_N=vertical_tail_spar_results.get("N_stiffeners", 0.0),
        vertical_tail_stiffener_spacing_m=vertical_tail_spar_results.get("stiffener_spacing_m", 0.0),
        vertical_tail_stiffener_thickness_m=vertical_tail_spar_results.get("stiffener_thickness_m", 0.0),
        vertical_tail_stiffener_height_m=vertical_tail_spar_results.get("stiffener_height_m", 0.0),
        vertical_tail_stiffener_area_one_mm2=vertical_tail_spar_results.get("stiffener_area_one_mm2", 0.0),
        vertical_tail_stiffener_I_one_mm4=vertical_tail_spar_results.get("stiffener_I_one_mm4", 0.0),
        vertical_tail_stiffener_tau_skin_MPa=vertical_tail_spar_results.get("stiffener_tau_skin_MPa", 0.0),
        vertical_tail_stiffener_tau_cr_panel_MPa=vertical_tail_spar_results.get("stiffener_tau_cr_panel_MPa", 0.0),
        vertical_tail_stiffener_q_shear_N_per_m=vertical_tail_spar_results.get("stiffener_q_shear_N_per_m", 0.0),
        vertical_tail_stiffener_aspect_ratio=vertical_tail_spar_results.get("stiffener_aspect_ratio", 0.0),
        vertical_tail_stiffener_mass_kg=vertical_tail_spar_results.get("W_stiffeners_kg", 0.0),
        vertical_tail_stiffener_panel_ok=vertical_tail_spar_results.get("stiffener_panel_ok", 0.0),
        vertical_tail_spar_tip_deflection_m=vertical_tail_spar_results.get("tip_deflection_m", 0.0),
        vertical_tail_spar_tip_deflection_mm=vertical_tail_spar_results.get("tip_deflection_mm", 0.0),
        vertical_tail_spar_root_slope_rad=vertical_tail_spar_results.get("root_slope_rad", 0.0),
        vertical_tail_spar_root_slope_deg=vertical_tail_spar_results.get("root_slope_deg", 0.0),
        wetted_area_m2=vertical_tail_wetted_m2,
    )

    fuselage_nose_longeron_boss_height_m = (
        inputs.fuselage_OD / 2.0 - fuselage_t_shell - inputs.payload_length / 2.0
    )
    fd = fuselage_details or {}
    fuselage_results = FuselageResults(
        t_shell_m=fuselage_t_shell,
        W_fuselage_kg=W_fuselage,
        W_fuselage_nose_kg=W_fuselage_nose,
        W_fuselage_section_C_kg=W_fuselage_section_C,
        W_fuselage_section_B_kg=W_fuselage_section_B,
        W_fuselage_section_A_kg=W_fuselage_section_A,
        W_fuselage_tail_kg=W_fuselage_tail,
        wetted_area_m2=fuselage_wetted_m2,
        fuselage_nose_longeron_boss_height_m=fuselage_nose_longeron_boss_height_m,
        W_shell_kg=fd.get("W_shell_kg", 0.0),
        W_frames_total_kg=fd.get("W_frames_total_kg", 0.0),
        W_stringers_total_kg=fd.get("W_stringers_total_kg", 0.0),
        W_longerons_total_kg=fd.get("W_longerons_total_kg", 0.0),
        T_design_Nm=fd.get("T_design_Nm", 0.0),
        N_frames=fd.get("N_frames", 0.0),
        frame_spacing_m=fd.get("frame_spacing_m", 0.0),
        frame_annulus_area_mm2=fd.get("frame_annulus_area_mm2", 0.0),
        N_stringers=fd.get("N_stringers", 0.0),
        stringer_spacing_m=fd.get("stringer_spacing_m", 0.0),
        stringer_thickness_m=fd.get("stringer_thickness_m", 0.0),
        stringer_height_m=fd.get("stringer_height_m", 0.0),
        stringer_area_one_mm2=fd.get("stringer_area_one_mm2", 0.0),
        N_longerons=fd.get("N_longerons", 0.0),
        longeron_OD_m=fd.get("longeron_OD_m", 0.0),
        longeron_ID_m=fd.get("longeron_ID_m", 0.0),
        longeron_A_one_mm2=fd.get("longeron_A_one_mm2", 0.0),
        sigma_x_MPa=fd.get("sigma_x_MPa", 0.0),
        tau_xy_MPa=fd.get("tau_xy_MPa", 0.0),
        sigma_cr_MPa=fd.get("sigma_cr_MPa", 0.0),
        tau_cr_MPa=fd.get("tau_cr_MPa", 0.0),
        q_torsion_N_per_m=fd.get("q_torsion_N_per_m", 0.0),
        panel_aspect_ratio=fd.get("panel_aspect_ratio", 0.0),
        panel_interaction_index=fd.get("panel_interaction_index", 0.0),
        panel_comp_ok=fd.get("panel_comp_ok", 0.0),
        panel_shear_ok=fd.get("panel_shear_ok", 0.0),
        panel_interaction_ok=fd.get("panel_interaction_ok", 0.0),
    )

    bd = boom_details or {}
    boom_results = BoomResults(
        total_mass_kg=float(W_booms),
        tube_mass_kg=bd.get("W_tubes_kg", 0.0),
        mount_mass_kg=bd.get("W_mounts_kg", 0.0),
        tail_crossmember_mass_kg=bd.get("W_tail_crossmember_kg", 0.0),
        outer_diameter_m=bd.get("OD_m", float(inputs.boom_outer_diameter_m)),
        inner_diameter_m=bd.get("ID_m", float(inputs.boom_inner_diameter_m)),
        length_m=float(inputs.boom_length_m),
        bending_moment_cruise_Nm_per_boom=bd.get("M_cruise_Nm_per_boom", 0.0),
        bending_moment_vtol_Nm_per_boom=bd.get("M_vtol_Nm_per_boom", 0.0),
        bending_moment_design_Nm_per_boom=bd.get("M_design_Nm_per_boom", 0.0),
        pusher_axial_load_N_per_boom=bd.get("pusher_axial_N_per_boom", 0.0),
        bending_stress_MPa=bd.get("sigma_bending_MPa", 0.0),
        axial_stress_MPa=bd.get("sigma_axial_MPa", 0.0),
        combined_stress_MPa=bd.get("sigma_combined_MPa", 0.0),
        allowable_stress_MPa=bd.get("sigma_allow_MPa", 0.0),
        strength_ok=bool(bd.get("strength_ok", 0.0) > 0.5),
        tip_deflection_m=bd.get("tip_deflection_m", 0.0),
        tip_deflection_mm=bd.get("tip_deflection_mm", 0.0),
    )

    sm = _compute_static_margin(
        x_cg_m=x_cg,
        x_ac_wing_m=x_ac_wing,
        x_ac_tail_m=x_ac_tail,
        wing_MAC_m=wing_geometry.MAC_m,
        wing_S_m2=wing_geometry.area_m2,
        tail_S_m2=tail_geometry.area_m2,
        a_w_per_rad=wing_aerodynamics.CL_alpha_3D_per_rad,
        a_t_per_rad=tail_aerodynamics.CL_alpha_3D_per_rad,
        eta_tail=inputs.tail_efficiency_factor,
        inputs=inputs,  # see note below
        alpha_wing_deg=wing_aerodynamics.alpha_cruise_deg,
    )

    dir_stab = _compute_directional_stability(
        x_cg_m=x_cg,
        x_ac_vertical_tail_m=x_ac_vertical_tail,
        wing_span_m=wing_geometry.span_m,
        wing_S_m2=wing_geometry.area_m2,
        vertical_tail_S_m2=float(inputs.n_vertical_tail_surfaces) * vertical_tail_geometry.area_m2,
        a_v_per_rad=vertical_tail_aerodynamics.CL_alpha_3D_per_rad,
        inputs=inputs,
    )

    aircraft_results = AircraftResults(
        W0_mass_kg=W0_mass,
        W0_weight_N=W0_N,
        L_wing_N=L_wing,
        x_cg_m=x_cg,
        drag_cruise_N=drag_cruise,
        cruise_power_W=cruise_power,
        cruise_power_elec_W=cruise_power_elec,
        x_np_m=sm["x_np_m"],
        static_margin=sm["SM"],
        static_margin_percent_MAC=sm["SM_percent_MAC"],
        V_h=sm["Vh"],
        Cn_beta=dir_stab["Cn_beta"],
        V_v=dir_stab["Vv"],

    )

    mass_properties_results = MassPropertiesResults(**mass_properties)
    drag_breakdown_results = DragBreakdownResults(**drag_breakdown)

    cg_locations = build_cg_locations(
        inputs,
        x_cg_m=x_cg,
        x_ac_wing_m=x_ac_wing,
        x_ac_tail_m=x_ac_tail,
        x_ac_vertical_tail_m=x_ac_vertical_tail,
        x_np_m=sm["x_np_m"],
    )

    return DesignResults(
        aircraft=aircraft_results,
        wing=wing_results,
        tail=tail_results,
        # VERTICAL MAIN WING REMOVED FOR NOW
        # vertical_wing=vertical_wing_results,
        vertical_tail=vertical_tail_results,
        fuselage=fuselage_results,
        booms=boom_results,
        battery=battery_results,
        propulsion=propulsion_results,
        avionics=avionics_results,
        drag_breakdown=drag_breakdown_results,
        cg_locations=cg_locations,
        mass_properties=mass_properties_results,
        inputs=asdict(inputs),
    )


def run_design(inputs: DesignInputs, plot_convergence: bool = False) -> DesignResults:
    """
    Main design iteration loop with an INNER trim-consistency loop:
    wing sizing <-> CG <-> wing moment about CG <-> tail sizing.
    """
    # Initial guesses
    W0_mass = inputs.payload_mass / inputs.payload_fraction_hist

    W_tails = 0.0
    W_vertical_tails = 0.0
    W_booms = 0.0
    boom_details: Dict[str, float] = {}
    L_tail = 0.0

    S_tail = 0.0
    aero_tail_dict = {"CD_cruise": 0.0, "CM_cruise": 0.0}
    drag_breakdown: Dict[str, float] = {}
    if getattr(inputs, "use_hardcoded_values", False) and getattr(inputs, "hardcode_tail_span", None) is not None:
        tail_span_for_props_m = float(inputs.hardcode_tail_span)
    else:
        tail_span_for_props_m = 1.0

    battery_selection = None
    propulsion_selection = None
    avionics_selection = None

    mass_history: List[float] = [W0_mass]
    iteration_numbers: List[int] = [0]

    # Outer loop: converges MTOW / masses
    clearance_met = True  # Initialize to True, will be updated in each iteration
    prop_R_final = 0.0  # Store prop radius for error message
    b_tail_nom_final = 0.0  # Store tail span for error message
    for i in range(DESIGN.max_iterations):
        W0_N = W0_mass * ATMOSPHERIC.g
        q = 0.5 * ATMOSPHERIC.rho * inputs.speed**2

        # ---------------------------------------------------------------------
        # INNER LOOP: converge (L_tail, wing sizing, CG, wing_M_cg, tail sizing)
        # ---------------------------------------------------------------------
        L_tail_inner = float(L_tail)  # start from last outer iter value

        # We keep propulsion/battery/avionics selections updated once per OUTER iter,
        # but we *allow* tail geometry/span to change and we use the latest span
        # for clearance checks next outer iter.
        #
        # The key fix is: within this outer iteration we no longer compute wing once
        # using stale L_tail and only afterwards compute tail. We iterate until consistent.

        # Store the "latest" solved objects from the inner loop for use after it finishes
        wing_geom = wing_aero = wing_struct = None
        # VERTICAL MAIN WING REMOVED FOR NOW
        # vertical_wing_geom = vertical_wing_aero = vertical_wing_struct = None
        fuselage_t_shell = None
        fuselage_details: Dict[str, float] = {}
        boom_details = {}
        tail_geom = tail_aero = tail_struct = None
        vertical_tail_geom = vertical_tail_aero = vertical_tail_struct = None
        x_ac_wing = None
        x_ac_tail = None
        x_cg = None
        wing_M_cg = None
        elevator_rod_results = {}
        wing_spar_results = {}
        tail_spar_results = {}
        vertical_tail_spar_results = {}

        # First: do a quick wing pass so we can estimate fuselage thickness for fuselage_ID
        wing_geom, wing_aero, wing_struct, x_ac_wing, wing_M_cg, W_wings, wing_spar_results = _calculate_wing_system(
            W0_N, L_tail_inner, inputs, propulsion_selection
        )
        # vertical_wing_geom, vertical_wing_aero, vertical_wing_struct, x_ac_vertical_wing, W_vertical_wings = _calculate_vertical_wing_system(
        #     wing_geom, inputs
        # )
        fuselage_t_shell, W_fuselage_calc, fuselage_details = calculate_central_pod_structure(
            wing_struct.bending_moment_Nm,
            inputs,
        )
        W_fuselage, W_fn, W_fc, W_fb, W_fa, W_ft = _fuselage_total_and_segments(inputs, W_fuselage_calc)
        W_booms, boom_details = calculate_twin_boom_structure(
            L_tail_inner, W_tails, W_vertical_tails, propulsion_selection, inputs
        )
        fuselage_ID = min(inputs.central_pod_width_m, inputs.central_pod_height_m) - 2 * fuselage_t_shell

        # Configuration-level drag build-up. The main-wing definition remains
        # unchanged; this adds the pod, booms, inactive lift units and cooling drag.
        drag_breakdown = configuration_cruise_drag(
            wing_geometry=wing_geom,
            wing_aerodynamics=wing_aero,
            inputs=inputs,
            propulsion_selection=propulsion_selection,
        )
        drag_cruise = float(drag_breakdown["total_N"])
        cruise_power = drag_cruise * inputs.speed
        cruise_power_elec = cruise_power / inputs.cruise_propeller_efficiency

        # Select propulsion/battery/avionics based on current aero estimate
        battery_selection, propulsion_selection, avionics_selection = select_propulsion_system(
            drag_cruise,
            cruise_power,
            cruise_power_elec,
            fuselage_ID,
            inputs,
            battery_selection,
            W0_mass_kg=W0_mass,
            V_stall_mps=wing_aero.V_stall,
            tail_span_m=tail_span_for_props_m,
            wing_S_m2=wing_geom.area_m2,
            wing_AR=wing_geom.AR,
            wing_CD0=drag_breakdown["CD0_aircraft"],
            wing_CD_cruise=drag_breakdown["CD_total_ref"],
            S_tail_m2=S_tail,
            tail_CD_cruise=aero_tail_dict.get("CD_cruise", 0.0),
        )

        component_masses = calculate_component_mass_breakdown(
            battery_selection,
            propulsion_selection,
            avionics_selection,
            inputs,
        )
        W_battery = float(component_masses["energy_system_total_kg"])
        W_propulsion = float(component_masses["propulsion_total_kg"])
        W_avionics = float(component_masses["avionics_total_kg"])

        # Now the actual inner trim loop
        # Track convergence for plotting
        L_tail_inner_history: List[float] = [L_tail_inner]
        inner_iteration_numbers: List[int] = [0]
        
        for j in range(DESIGN.trim_inner_max_iter):
            # 1) Wing system with current L_tail guess
            wing_geom, wing_aero, wing_struct, x_ac_wing, wing_M_cg_tmp, W_wings, wing_spar_results = _calculate_wing_system(
                W0_N, L_tail_inner, inputs, propulsion_selection
            )

            # 2) Vertical wing & fuselage (depend on wing sizing)
            # vertical_wing_geom, vertical_wing_aero, vertical_wing_struct, x_ac_vertical_wing, W_vertical_wings = _calculate_vertical_wing_system(
            #     wing_geom, inputs
            # )

            # VERTICAL MAIN WING REMOVED FOR NOW
            W_vertical_wings = 0.0

            fuselage_t_shell, W_fuselage_calc, fuselage_details = calculate_central_pod_structure(
                wing_struct.bending_moment_Nm,
                inputs,
            )
            W_fuselage, W_fn, W_fc, W_fb, W_fa, W_ft = _fuselage_total_and_segments(inputs, W_fuselage_calc)
            W_booms, boom_details = calculate_twin_boom_structure(
                L_tail_inner, W_tails, W_vertical_tails, propulsion_selection, inputs
            )

            # 3) Need CG -> depends on tail masses, but tail masses depend on tail sizing.
            # We start with current tail mass estimates (W_tails, W_vertical_tails),
            # compute CG, then compute wing moment about CG, then resize tail, update masses,
            # recompute CG next inner iteration until L_tail stabilizes.

            # CG using current mass guesses (tail masses are from previous inner pass)
            # x_cg = _calculate_cg_position(
            #     W_wings, W_vertical_wings, W_fuselage, W_tails, W_vertical_tails,
            #     W_battery, W_propulsion, W_avionics,
            #     avionics_selection, inputs, W0_mass
            # )

            # VERTICAL MAIN WING REMOVED FOR NOW
            x_cg = _calculate_cg_position(
                W_wings,
                W_fn, W_fc, W_fb, W_fa, W_ft,
                W_booms,
                W_tails, W_vertical_tails,
                component_masses,
                avionics_selection, inputs
            )

            # 4) Update wing pitching moment about CG using actual CG
            M_ac_wing = wing_aero.CM_cruise * q * wing_geom.area_m2 * wing_geom.MAC_m
            L_wing = W0_N - L_tail_inner
            wing_M_cg = (
                M_ac_wing
                + L_wing * math.cos(wing_aero.alpha_cruise_deg * math.pi / 180.0) * (x_cg - x_ac_wing)
            )

            # 5) Tail sizing (trim) based on updated wing_M_cg and CG
            tail_geom, tail_aero, tail_struct, x_ac_tail, L_tail_new, aero_tail_dict, clearance_met_inner, elevator_rod_results, tail_spar_results = _calculate_tail_system(
                x_cg,
                wing_geom.MAC_m,
                wing_geom.area_m2,
                wing_aero.alpha_cruise_deg,
                wing_M_cg,
                wing_aero.CD_cruise,
                inputs,
                propulsion_selection,
            )

            # Update outer loop clearance_met with latest value from inner loop
            clearance_met = clearance_met_inner

            # Update tail span for clearance checks next pass / next outer iter
            tail_span_for_props_m = float(tail_geom.span_m)
            # Store values for error message if clearance not met
            prop_R_final = aero_tail_dict.get("prop_radius_m", 0.0)
            b_tail_nom_final = float(tail_geom.span_m)

            # Update tail masses from the newly sized tail
            S_tail = tail_geom.area_m2
            W_tails = tail_struct.W_total_kg * 2  # both tails; already from specific weight model (or hardcoded override)

            vertical_tail_geom, vertical_tail_aero, vertical_tail_struct, x_ac_vertical_tail, W_vertical_tails, dir_stab_dict, vertical_tail_spar_results = _calculate_vertical_tail_system(
                x_cg, wing_geom, inputs
            )
            W_booms, boom_details = calculate_twin_boom_structure(
                L_tail_new, W_tails, W_vertical_tails, propulsion_selection, inputs
            )

            # Update propulsion using the complete twin-boom configuration drag.
            drag_breakdown_inner = configuration_cruise_drag(
                wing_geometry=wing_geom,
                wing_aerodynamics=wing_aero,
                tail_geometry=tail_geom,
                tail_aerodynamics=tail_aero,
                vertical_tail_geometry=vertical_tail_geom,
                vertical_tail_aerodynamics=vertical_tail_aero,
                inputs=inputs,
                propulsion_selection=propulsion_selection,
            )
            drag_cruise_inner = float(drag_breakdown_inner["total_N"])
            cruise_power_inner = drag_cruise_inner * inputs.speed
            cruise_power_elec_inner = cruise_power_inner / inputs.cruise_propeller_efficiency

            fuselage_ID = min(inputs.central_pod_width_m, inputs.central_pod_height_m) - 2 * fuselage_t_shell

            battery_selection, propulsion_selection, avionics_selection = select_propulsion_system(
                drag_cruise_inner,
                cruise_power_inner,
                cruise_power_elec_inner,
                fuselage_ID,
                inputs,
                battery_selection,
                W0_mass_kg=W0_mass,
                V_stall_mps=wing_aero.V_stall,
                tail_span_m=tail_span_for_props_m,
                wing_S_m2=wing_geom.area_m2,
                wing_AR=wing_geom.AR,
                wing_CD0=drag_breakdown_inner["CD0_aircraft"],
                wing_CD_cruise=drag_breakdown_inner["CD_total_ref"],
                S_tail_m2=S_tail,
                tail_CD_cruise=aero_tail_dict.get("CD_cruise", 0.0),
            )

            component_masses = calculate_component_mass_breakdown(
                battery_selection,
                propulsion_selection,
                avionics_selection,
                inputs,
            )
            W_battery = float(component_masses["energy_system_total_kg"])
            W_propulsion = float(component_masses["propulsion_total_kg"])
            W_avionics = float(component_masses["avionics_total_kg"])
            drag_breakdown = drag_breakdown_inner

            # Convergence check on L_tail
            dL = abs(L_tail_new - L_tail_inner)

            # Relaxation to avoid oscillation / runaway in sensitive regimes
            relax = DESIGN.trim_inner_relax
            L_tail_inner = (1.0 - relax) * L_tail_inner + relax * L_tail_new
            
            # Track convergence history
            L_tail_inner_history.append(L_tail_inner)
            inner_iteration_numbers.append(j + 1)

            if dL < DESIGN.trim_inner_tol_N:
                # Inner trim loop converged
                L_tail = float(L_tail_inner)
                break

            # If we hit max inner iterations, keep best estimate
            L_tail = float(L_tail_inner)
        
        # Plot inner loop convergence if requested
        if plot_convergence and len(L_tail_inner_history) > 1:
            try:
                plot_tail_trim_convergence(inner_iteration_numbers, L_tail_inner_history, i)
            except Exception as e:
                # Don't fail if plotting fails
                print(f"Warning: Could not plot tail trim convergence: {e}")

        # ---------------------------------------------------------------------
        # End of INNER LOOP: now we have consistent wing<->CG<->tail for this W0
        # ---------------------------------------------------------------------

        # With final aero (including both fins), recompute complete cruise drag.
        drag_breakdown = configuration_cruise_drag(
            wing_geometry=wing_geom,
            wing_aerodynamics=wing_aero,
            tail_geometry=tail_geom,
            tail_aerodynamics=tail_aero,
            vertical_tail_geometry=vertical_tail_geom,
            vertical_tail_aerodynamics=vertical_tail_aero,
            inputs=inputs,
            propulsion_selection=propulsion_selection,
        )
        drag_cruise = float(drag_breakdown["total_N"])
        cruise_power = drag_cruise * inputs.speed
        cruise_power_elec = cruise_power / inputs.cruise_propeller_efficiency

        # Update propulsion selection (optional but recommended) now that tail is consistent
        fuselage_ID = min(inputs.central_pod_width_m, inputs.central_pod_height_m) - 2 * fuselage_t_shell
        battery_selection, propulsion_selection, avionics_selection = select_propulsion_system(
            drag_cruise,
            cruise_power,
            cruise_power_elec,
            fuselage_ID,
            inputs,
            battery_selection,
            W0_mass_kg=W0_mass,
            V_stall_mps=wing_aero.V_stall,
            tail_span_m=tail_span_for_props_m,
            wing_S_m2=wing_geom.area_m2,
            wing_AR=wing_geom.AR,
            wing_CD0=drag_breakdown["CD0_aircraft"],
            wing_CD_cruise=drag_breakdown["CD_total_ref"],
            S_tail_m2=S_tail,
            tail_CD_cruise=aero_tail_dict.get("CD_cruise", 0.0),
        )
        component_masses = calculate_component_mass_breakdown(
            battery_selection,
            propulsion_selection,
            avionics_selection,
            inputs,
        )
        W_battery = float(component_masses["energy_system_total_kg"])
        W_propulsion = float(component_masses["propulsion_total_kg"])
        W_avionics = float(component_masses["avionics_total_kg"])

        # Re-run the structural masses with the final lift/cruise selections for
        # this outer iteration so concentrated motor loads are not one pass stale.
        wing_geom, wing_aero, wing_struct, x_ac_wing, wing_M_cg_tmp, W_wings, wing_spar_results = _calculate_wing_system(
            W0_N, L_tail, inputs, propulsion_selection
        )
        fuselage_t_shell, W_fuselage_calc, fuselage_details = calculate_central_pod_structure(
            wing_struct.bending_moment_Nm, inputs
        )
        W_fuselage, W_fn, W_fc, W_fb, W_fa, W_ft = _fuselage_total_and_segments(inputs, W_fuselage_calc)
        W_booms, boom_details = calculate_twin_boom_structure(
            L_tail, W_tails, W_vertical_tails, propulsion_selection, inputs
        )

        # Recompute take-off/end-of-mission mass properties with the final selections.
        mass_properties_current = _calculate_mass_properties(
            W_wings,
            W_fn, W_fc, W_fb, W_fa, W_ft,
            W_booms,
            W_tails, W_vertical_tails,
            component_masses,
            avionics_selection,
            inputs,
        )
        x_cg = float(mass_properties_current["x_cg_takeoff_m"])
        W0_mass_new = float(mass_properties_current["takeoff_mass_kg"])

        mass_history.append(W0_mass_new)
        iteration_numbers.append(i + 1)

        # Convergence on MTOW
        if abs(W0_mass_new - W0_mass) / max(W0_mass, 1e-6) < DESIGN.convergence_threshold:
            W0_mass = W0_mass_new
            break

        W0_mass = W0_mass_new

    # Check the legacy tail-mounted-prop clearance only for configurations that use it.
    if bool(getattr(inputs, "tail_mounted_propulsion", True)) and not clearance_met:
        raise ValueError(f"Prop clearance not met. Try increasing tail volume ratio.\n"
            f"  half_b_tail_nom = {b_tail_nom_final/2:.2f} m < {(getattr(inputs, 'prop_fuselage_clearance_m', 0.02) + prop_R_final) / inputs.prop_y_position_fraction:.2f} m\n"
            f"  prop_R = {prop_R_final:.2f} m\n"
            f"  inputs.prop_fuselage_clearance_m = {getattr(inputs, 'prop_fuselage_clearance_m', 0.02):.2f} m\n"
            f"  inputs.prop_y_position_fraction = {inputs.prop_y_position_fraction:.2f}\n"
        )

    # Build final DesignResults using the latest solved state.
    mass_properties_final = _calculate_mass_properties(
        W_wings,
        W_fn, W_fc, W_fb, W_fa, W_ft,
        W_booms,
        W_tails, W_vertical_tails,
        component_masses,
        avionics_selection,
        inputs,
    )
    W0_mass = float(mass_properties_final["takeoff_mass_kg"])
    x_cg = float(mass_properties_final["x_cg_takeoff_m"])
    W0_N = W0_mass * ATMOSPHERIC.g
    L_wing = W0_N - L_tail
    
    # VERTICAL MAIN WING REMOVED FOR NOW
    # vertical_wing variables not used but may be referenced
    # vertical_wing_geom = None
    # vertical_wing_aero = None
    # vertical_wing_struct = None
    # x_ac_vertical_wing = 0.0
    
    # Convert selections to results objects
    battery_results, propulsion_results, avionics_results = convert_to_results(
        battery_selection,
        propulsion_selection,
        avionics_selection,
    )
    
    # Create inputs dictionary
    inputs_dict = asdict(inputs)
    
    # Assemble results using the helper function
    results = _assemble_results(
        W0_mass=W0_mass,
        W0_N=W0_N,
        L_wing=L_wing,
        x_cg=x_cg,
        drag_cruise=drag_cruise,
        cruise_power=cruise_power,
        cruise_power_elec=cruise_power_elec,
        drag_breakdown=drag_breakdown,
        wing_geometry=wing_geom,
        wing_aerodynamics=wing_aero,
        wing_structure=wing_struct,
        x_ac_wing=x_ac_wing,
        wing_moment=wing_M_cg,
        wing_spar_results=wing_spar_results,
        tail_geometry=tail_geom,
        tail_aerodynamics=tail_aero,
        tail_structure=tail_struct,
        x_ac_tail=x_ac_tail,
        L_tail=L_tail,
        tail_aero_dict=aero_tail_dict,
        elevator_rod_results=elevator_rod_results,
        tail_spar_results=tail_spar_results,
        # VERTICAL MAIN WING REMOVED FOR NOW
        # vertical_wing_geometry=vertical_wing_geom,
        # vertical_wing_aerodynamics=vertical_wing_aero,
        # vertical_wing_structure=vertical_wing_struct,
        # x_ac_vertical_wing=x_ac_vertical_wing,
        vertical_tail_geometry=vertical_tail_geom,
        vertical_tail_aerodynamics=vertical_tail_aero,
        vertical_tail_structure=vertical_tail_struct,
        x_ac_vertical_tail=x_ac_vertical_tail,
        vertical_tail_spar_results=vertical_tail_spar_results,
        fuselage_t_shell=fuselage_t_shell,
        W_fuselage=W_fuselage,
        W_fuselage_nose=W_fn,
        W_fuselage_section_C=W_fc,
        W_fuselage_section_B=W_fb,
        W_fuselage_section_A=W_fa,
        W_fuselage_tail=W_ft,
        W_booms=W_booms,
        boom_details=boom_details,
        battery_results=battery_results,
        propulsion_results=propulsion_results,
        avionics_results=avionics_results,
        mass_properties=mass_properties_final,
        inputs=inputs,
        fuselage_details=fuselage_details,
    )

    if plot_convergence:
        try:
            import matplotlib.pyplot as plt
            plt.figure()
            plt.plot(iteration_numbers, mass_history, marker="o")
            plt.xlabel("Iteration")
            plt.ylabel("MTOW mass [kg]")
            plt.title("Convergence history")
            plt.grid(True)
            plt.show()
        except Exception:
            pass

    return results


if __name__ == "__main__":
    inputs = DesignInputs()
    results = run_design(inputs)

    if inputs.export_inventor_xml:
        xml_path = inputs.inventor_xml_path
        if os.path.isdir(xml_path):
            base = os.path.join(xml_path, "parameters")
        else:
            base = os.path.splitext(xml_path)[0]

        paths = export_inventor_parameters(
            results,
            base,
            write_xml=True,
            write_xlsx=True,
        )

    # Maneuver point (choose a low-speed case; elevator is least effective there) :contentReference[oaicite:11]{index=11}
    V_man = 30.0  # m/s (example)

    W0_N = results.aircraft.W0_weight_N

    # Derivatives (maneuver-only)
    derivs = elevator_control_derivatives(
        wing_S_m2=results.wing.geometry.area_m2,
        wing_MAC_m=results.wing.geometry.MAC_m,
        tail_S_m2=results.tail.geometry.area_m2,
        tail_AR=results.tail.geometry.AR,
        x_cg_m=results.aircraft.x_cg_m,
        x_ac_tail_m=results.tail.x_ac_tail_m,
        inputs=inputs,
    )

    # Max steady load factor limited by elevator
    lim = max_load_factor_from_elevator(
        V_mps=V_man,
        W0_N=W0_N,
        wing_S_m2=results.wing.geometry.area_m2,
        wing_MAC_m=results.wing.geometry.MAC_m,
        wing_AR=results.wing.geometry.AR,
        wing_CL_alpha_per_rad=results.wing.aerodynamics.CL_alpha_3D_per_rad,
        wing_CL_max=results.wing.aerodynamics.CL_max,
        x_ac_wing_m=results.wing.x_ac_wing_m,
        x_cg_m=results.aircraft.x_cg_m,
        tail_S_m2=results.tail.geometry.area_m2,
        tail_MAC_m=results.tail.geometry.MAC_m,
        tail_AR=results.tail.geometry.AR,
        tail_CL_max=results.tail.aerodynamics.CL_max,
        tail_incidence_deg=results.tail.tail_incidence_deg,
        x_ac_tail_m=results.tail.x_ac_tail_m,
        inputs=inputs,
        n_hi=inputs.n_ult,
        tail_structure=results.tail.structure,
        tail_span_m=results.tail.geometry.span_m,
        tail_c_root_m=results.tail.geometry.c_root_m,
        tail_taper_ratio=inputs.tail_taper_ratio,
        tail_c_MGC_m=results.tail.geometry.c_MGC_m,
        tail_max_thickness_fraction=results.tail.aerodynamics.max_thickness_fraction_of_chord,
    )

    # Store maneuver results
    maneuver_results = ManeuverResults(
        V_maneuver_mps=V_man,
        CL_alpha_h_per_rad=derivs["CL_alpha_h_per_rad"],
        tau_e=derivs["tau_e"],
        bE_over_bh_geom=derivs["bE_over_bh_geom"],
        bE_over_bh_eff=derivs["bE_over_bh_eff"],
        eta_h=derivs["eta_h"],
        l_h_m=derivs["l_h_m"],
        V_H=derivs["V_H"],
        Cm_deltaE_per_rad=derivs["Cm_deltaE_per_rad"],
        CL_deltaE_per_rad=derivs["CL_deltaE_per_rad"],
        CLh_deltaE_per_rad=derivs["CLh_deltaE_per_rad"],
        n_max=lim.get("n_max", 0.0),
        feasible=lim.get("feasible", 0.0) > 0.5,
        alpha_w_deg=lim.get("alpha_w_deg"),
        CL_w=lim.get("CL_w"),
        L_w_N=lim.get("L_w_N"),
        M_ac_w_Nm=lim.get("M_ac_w_Nm"),
        M_cg_w_Nm=lim.get("M_cg_w_Nm"),
        L_tail_req_N=lim.get("L_tail_req_N"),
        CL_tail_req=lim.get("CL_tail_req"),
        delta_e_deg=lim.get("delta_e_deg"),
        downwash_deg=lim.get("downwash_deg"),
        alpha_tail_geom_deg=lim.get("alpha_tail_geom_deg"),
        alpha_stall_eff_tail_deg=lim.get("alpha_stall_eff_tail_deg"),
        phi_max_deg_level_turn=lim.get("phi_max_deg_level_turn"),
        reason=lim.get("reason"),
    )
    results.maneuver = maneuver_results

    # Choose a "worst" speed for rudder authority (usually low speed)
    V_rud = 40.0  # m/s (example)

    # 1) Rudder derivatives (clean flow)
    rud_deriv = rudder_control_derivatives(
        wing_S_m2=results.wing.geometry.area_m2,
        wing_span_m=results.wing.geometry.span_m,
        vertical_tail_S_m2=float(inputs.n_vertical_tail_surfaces) * results.vertical_tail.geometry.area_m2,
        vertical_tail_AR=results.vertical_tail.geometry.AR,
        x_cg_m=results.aircraft.x_cg_m,
        x_ac_vertical_tail_m=results.vertical_tail.x_ac_tail_m,
        inputs=inputs,
        S_v_effective_fraction=1.0,
        eta_v_maneuver=getattr(inputs, "vertical_tail_eta_v_maneuver", getattr(inputs, "vertical_tail_eta_v", 1.0)),
    )

    auth = rudder_max_yaw_moment(
        V_mps=V_rud,
        wing_S_m2=results.wing.geometry.area_m2,
        wing_span_m=results.wing.geometry.span_m,
        Cn_deltaR_per_rad=rud_deriv["Cn_deltaR_per_rad"],
        inputs=inputs,
    )

    vt_struct = results.vertical_tail.structure
    vt_geom   = results.vertical_tail.geometry
    vt_aero   = results.vertical_tail.aerodynamics

    vt_struct_args = dict(
        l_v_m=rud_deriv["l_v_m"],
        vertical_tail_structure=vt_struct,
        vertical_tail_S_m2=float(inputs.n_vertical_tail_surfaces) * vt_geom.area_m2,
        vt_span_m=vt_geom.span_m,
        vt_c_root_m=vt_geom.c_root_m,
        vt_taper_ratio=getattr(inputs, "tail_taper_ratio", 1.0),
        vt_max_thickness_fraction=vt_aero.max_thickness_fraction_of_chord,
    )

    # 2) Asymmetric thrust test (pick a lateral engine arm; adapt to your layout)
    # Example: use tail half-span * prop_y_position_fraction
    y_engine = inputs.prop_y_position_fraction * (results.tail.geometry.span_m / 2.0) + inputs.fuselage_OD/2

    # Example thrust asymmetry: one motor out at "some" thrust (you can choose max thrust if you want worst-case)
    Tmax_per_motor = results.propulsion.max_thrust_per_motor_N if results.propulsion else 0.0
    T_asym = 0.7 * Tmax_per_motor  # example

    asym = rudder_asymmetric_thrust(
        V_mps=V_rud,
        wing_S_m2=results.wing.geometry.area_m2,
        wing_span_m=results.wing.geometry.span_m,
        Cn_deltaR_per_rad=rud_deriv["Cn_deltaR_per_rad"],
        y_engine_m=y_engine,
        thrust_asym_N=T_asym,
        inputs=inputs,
        **vt_struct_args,
    )

    # 3) Spin recovery test (use an "effective VT fraction" for spin wake immersion)
    Sfrac_spin = float(getattr(inputs, "vertical_tail_effective_area_fraction_spin", 0.6))
    eta_spin = float(getattr(inputs, "vertical_tail_eta_v_spin", 0.6))

    rud_deriv_spin = rudder_control_derivatives(
        wing_S_m2=results.wing.geometry.area_m2,
        wing_span_m=results.wing.geometry.span_m,
        vertical_tail_S_m2=float(inputs.n_vertical_tail_surfaces) * results.vertical_tail.geometry.area_m2,
        vertical_tail_AR=results.vertical_tail.geometry.AR,
        x_cg_m=results.aircraft.x_cg_m,
        x_ac_vertical_tail_m=results.vertical_tail.x_ac_tail_m,
        inputs=inputs,
        S_v_effective_fraction=Sfrac_spin,
        eta_v_maneuver=eta_spin,
    )

    spin = rudder_spin_recovery_capability(
        V_spin_mps=results.wing.aerodynamics.V_stall * 1.1,  # example "near-stall"
        wing_S_m2=results.wing.geometry.area_m2,
        wing_span_m=results.wing.geometry.span_m,
        Cn_deltaR_per_rad_spin=rud_deriv_spin["Cn_deltaR_per_rad"],
        inputs=inputs,
        mass_kg=results.aircraft.W0_mass_kg,
        fuselage_length_m=inputs.fuselage_length,
        spin_rate_rev_per_s=float(getattr(inputs, "spin_rate_rev_per_s", 1.0)),
        t_recover_s=float(getattr(inputs, "spin_recovery_time_s", 2.0)),
        **vt_struct_args,
    )

    # 4) Coordinated-turn characteristics (needs Cn_beta from your main model)
    coord = rudder_coordinated_turn_characteristics(
        V_mps=V_rud,
        wing_S_m2=results.wing.geometry.area_m2,
        wing_span_m=results.wing.geometry.span_m,
        Cn_deltaR_per_rad=rud_deriv["Cn_deltaR_per_rad"],
        Cn_beta_per_rad=results.aircraft.Cn_beta,  # from your directional stability calc
        inputs=inputs,
        **vt_struct_args,
    )

    # Store rudder results in structured format
    rudder_results = RudderResults(
        V_rudder_mps=V_rud,
        # Rudder derivatives
        a_v_per_rad=rud_deriv["a_v_per_rad"],
        l_v_m=rud_deriv["l_v_m"],
        cR_over_cV=rud_deriv["cR_over_cV"],
        tau_r=rud_deriv["tau_r"],
        bR_over_bV_geom=rud_deriv["bR_over_bV_geom"],
        bR_over_bV_eff=rud_deriv["bR_over_bV_eff"],
        eta_v=rud_deriv["eta_v"],
        dsigma_dbeta=rud_deriv["dsigma_dbeta"],
        S_v_eff_m2=rud_deriv["S_v_eff_m2"],
        V_Ve=rud_deriv["V_Ve"],
        Cn_deltaR_per_rad=rud_deriv["Cn_deltaR_per_rad"],
        CY_deltaR_per_rad=rud_deriv["CY_deltaR_per_rad"],
        CY_beta_v_per_rad=rud_deriv["CY_beta_v_per_rad"],
        # Asymmetric thrust
        y_engine_m=asym["y_engine_m"],
        thrust_asym_N=asym["thrust_asym_N"],
        N_req_asym_Nm=asym["N_req_Nm"],
        deltaR_req_asym_deg=asym["deltaR_req_deg"],
        T_asym_max_N=asym["T_asym_max_N"],
        feasible_asym=bool(asym["feasible"]),
        reason_asym=asym.get("reason", None),
        # Spin recovery
        V_spin_mps=spin["V_spin_mps"],
        mass_kg=spin["mass_kg"],
        Izz_kgm2=spin["Izz_kgm2"],
        spin_rate_rev_per_s=spin["spin_rate_rev_per_s"],
        r0_radps=spin["r0_radps"],
        t_recover_s=spin["t_recover_s"],
        rdot_req_radps2=spin["rdot_req_radps2"],
        N_req_spin_Nm=spin["N_req_Nm"],
        deltaR_req_spin_deg=spin["deltaR_req_deg"],
        rdot_max_radps2=spin["rdot_max_radps2"],
        t_stop_best_s=spin["t_stop_best_s"],
        feasible_spin=bool(spin["feasible"]),
        reason_spin=spin.get("reason", None),
        # Coordinated turn
        Cn_beta_per_rad=coord.get("Cn_beta_per_rad"),
        beta_trim_max_abs_deg=coord.get("beta_trim_max_abs_deg"),
        N_max_abs_Nm=coord.get("N_max_abs_Nm"),
        deltaR_max_abs_deg=coord.get("deltaR_max_abs_deg"),
    )
    results.rudder = rudder_results

    with open("results.txt", "w", encoding="utf-8") as f:
        tee = _Tee(sys.stdout, f)
        old_stdout = sys.stdout
        sys.stdout = tee
        try:
            print_all_results(results, inputs)
        finally:
            sys.stdout = old_stdout

    results_dict = results.to_dict()