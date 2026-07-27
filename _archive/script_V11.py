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
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import asdict

from design_inputs import DesignInputs
from constants import ATMOSPHERIC, DESIGN
from geometry import wing_geometry, tail_geometry
from aerodynamics import (
    wing_aerodynamic_coeffs,
    tail_aerodynamic_coeffs,
    vertical_wing_aerodynamic_coeffs,
    vertical_tail_aerodynamic_coeffs,
)
from structures import calculate_wing_structure, calculate_tail_structure, calculate_fuselage_structure
from propulsion_system import (
    select_propulsion_system,
    calculate_component_masses,
    convert_to_results,
)
from convergence import check_convergence, plot_mass_convergence
from results import (
    DesignResults,
    AircraftResults,
    WingResults,
    TailResults,
    VerticalWingResults,
    VerticalTailResults,
    FuselageResults,
    Geometry,
    Aerodynamics,
    Structure,
    AvionicsResults,
)
from reporting import print_all_results
from avionics import estimate_avionics_and_electrics
from inventor_param_export import export_inventor_parameters


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


def _calculate_cg_position(
    W_wings: float,
    W_vertical_wings: float,
    W_fuselage: float,
    W_tails: float,
    W_vertical_tails: float,
    W_battery: float,
    W_propulsion: float,
    W_avionics: float,
    avionics_selection: Any,
    inputs: DesignInputs,
    W0_mass: float,
) -> float:
    """
    Calculate center of gravity position.
    """
    num = (
        W_wings * inputs.cg_x_wing +
        W_vertical_wings * inputs.cg_x_wing +  # Vertical wings at same x-position as horizontal
        W_fuselage * inputs.cg_x_fuselage +
        W_tails * inputs.cg_x_tail +
        W_vertical_tails * inputs.cg_x_tail +  # Vertical tails at same x-position as horizontal
        W_battery * inputs.cg_x_battery +
        W_propulsion * inputs.cg_x_propulsion +
        inputs.payload_mass * inputs.cg_x_payload +
        avionics_selection.autopilot_mass_kg * inputs.cg_x_autopilot +
        avionics_selection.gps_mass_kg * inputs.cg_x_gps +
        avionics_selection.airspeed_mass_kg * inputs.cg_x_airspeed +
        avionics_selection.rc_rx_mass_kg * inputs.cg_x_rx +
        avionics_selection.telemetry_mass_kg * inputs.cg_x_tel +
        avionics_selection.power_module_mass_kg * inputs.cg_x_power_module +
        avionics_selection.esc_mass_total_kg * inputs.cg_x_esc +
        avionics_selection.servos_mass_kg * inputs.cg_x_servos +
        avionics_selection.misc_electronics_mass_kg * inputs.cg_x_misc
    )
    return num / W0_mass


def _calculate_wing_system(
    W0_N: float,
    L_tail: float,
    inputs: DesignInputs,
) -> Tuple[Geometry, Aerodynamics, Structure, float, float, float]:
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

    # Geometry
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
    wing_structure, W_wings = calculate_wing_structure(
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
    )

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
    )

    return geometry, aerodynamics, wing_structure, x_ac_wing, M_cg_wing, W_wings


def _calculate_tail_system(
    x_cg: float,
    wing_MAC: float,
    wing_S: float,
    wing_alpha_cruise: float,
    wing_M_cg: float,
    wing_CD_cruise: float,
    inputs: DesignInputs,
    propulsion_selection: Optional[Dict[str, Any]] = None,
) -> Tuple[Geometry, Aerodynamics, Structure, float, float, Dict[str, Any]]:
    """
    Calculate complete tail system with trim iteration.

    FIX (FOREVER):
      - The tail span extension is treated as a *physical span increase*.
      - After trim scaling, we recompute (S, b, AR, yMAC) with the extension baked in.
      - This ensures structure, aero, clearance, printing, AND DoE all use the same physical span.
    """
    q = 0.5 * ATMOSPHERIC.rho * inputs.speed**2

    # Initial tail geometry (nominal)
    S_tail_nom, b_tail_nom, c_root_tail, c_tip_tail, MAC_tail, y_MAC_tail, AR_tail_nom, c_MGC_tail, ac_on_MAC_tail = tail_geometry(
        x_cg, wing_MAC, wing_S, inputs
    )

    x_ac_tail = inputs.cg_x_tail - 0.5 * c_root_tail + ac_on_MAC_tail

    # Required tail lift for trim (initial)
    L_tail = wing_M_cg / (x_ac_tail - x_cg)

    # Iterate tail area scaling until incidence + elevator constraints satisfied
    tail_area_scale = 1.0
    aero_tail = None

    while tail_area_scale <= inputs.tail_area_scale_max + 1e-9:
        aero_try = tail_aerodynamic_coeffs(
            L_required_N=L_tail,
            S=S_tail_nom,
            MAC=MAC_tail,
            AR=AR_tail_nom,  # use nominal AR for this trim scan (keeps behavior stable)
            alpha_body_deg=wing_alpha_cruise,
            inputs=inputs,
            tail_area_scale=tail_area_scale,
            downwash_deg=0.0,
        )

        inc = aero_try["tail_incidence_deg"]
        de = aero_try["delta_elev_deg"]

        if abs(inc) <= inputs.tail_incidence_limit_deg and abs(de) <= inputs.tail_elevator_limit_deg:
            aero_tail = aero_try
            break

        tail_area_scale += inputs.tail_area_scale_step

    if aero_tail is None:
        raise ValueError(
            "Trim constraints not achievable with current geometry.\n"
            f"Even with tail area scaled up to {inputs.tail_area_scale_max:.2f}x nominal, "
            f"could not satisfy: |incidence| <= {inputs.tail_incidence_limit_deg:.1f} deg "
            f"and |elevator| <= {inputs.tail_elevator_limit_deg:.1f} deg at cruise.\n"
            "Try: increase tail volume ratio, move CG aft, move tail further back, "
            "reduce wing Cm, or relax limits."
        )

    # Use the scaled tail area
    S_tail_scaled = float(S_tail_nom) * float(aero_tail["tail_area_scale"])

    # --- SINGLE SOURCE OF TRUTH: bake extension into physical geometry ---
    ext = float(getattr(inputs, "tail_span_extension_m", 0.0) or 0.0)
    S_tail, b_tail, AR_tail, y_MAC_tail = _apply_tail_span_extension_and_recompute(
        S_tail_nominal_or_scaled=S_tail_scaled,
        c_root_tail=float(c_root_tail),
        taper_ratio=float(inputs.tail_taper_ratio),
        span_extension_m=ext,
    )

    # keep c_MGC consistent with the physical geometry
    c_MGC_tail = S_tail / b_tail

    # Tail moment coefficient contribution at cruise
    M_ac_tail = aero_tail['CM_cruise'] * q * S_tail * MAC_tail

    # refine tail lift (one-step update)
    L_tail_new = (wing_M_cg + M_ac_tail) / (x_ac_tail - x_cg)
    if abs(L_tail_new - L_tail) / max(abs(L_tail), 1e-6) > 0.1:
        L_tail = 0.5 * (L_tail + L_tail_new)
    else:
        L_tail = L_tail_new

    # Structure uses physical geometry
    tail_structure, W_tails = calculate_tail_structure(
        L_tail=abs(L_tail),
        S=S_tail,
        c_root=c_root_tail,
        taper_ratio=inputs.tail_taper_ratio,
        b=b_tail,
        c_MGC=c_MGC_tail,
        y_MAC=y_MAC_tail,
        max_thickness_fraction=aero_tail['max_thickness_fraction_of_chord'],
        CM_max=aero_tail['CM_max'],
        inputs=inputs,
        propulsion_selection=propulsion_selection,
    )

    geometry = Geometry(
        area_m2=S_tail,
        span_m=b_tail,
        c_root_m=c_root_tail,
        c_tip_m=c_tip_tail,
        MAC_m=MAC_tail,
        y_MAC_from_centerline_m=y_MAC_tail,
        ac_on_MAC_m=ac_on_MAC_tail,
        AR=AR_tail,
        c_MGC_m=c_MGC_tail,
    )

    aerodynamics = Aerodynamics(
        CL_cruise=aero_tail['CL_cruise'],
        CL_max=aero_tail['CL_max'],
        CD0=aero_tail['CD0'],
        CD_cruise=aero_tail['CD_cruise'],
        CD0_fuselage=0.0,
        CM_cruise=aero_tail['CM_cruise'],
        CM_max=aero_tail['CM_max'],
        alpha_cruise_deg=aero_tail['alpha_equiv_deg'],
        L_over_D_cruise=aero_tail['L_over_D_cruise'],
        V_stall=aero_tail['V_stall'],
        Re_cruise=aero_tail['Re_cruise'],
        max_thickness_fraction_of_chord=aero_tail['max_thickness_fraction_of_chord'],
    )

    # Add explicit debug keys so downstream exporters can always find them
    aero_tail = dict(aero_tail)
    aero_tail["tail_span_extension_m"] = ext
    aero_tail["tail_span_total_m"] = float(b_tail)
    aero_tail["tail_span_base_m"] = float(b_tail) - ext
    aero_tail["tail_area_total_m2"] = float(S_tail)

    return geometry, aerodynamics, tail_structure, x_ac_tail, L_tail, aero_tail


def _calculate_vertical_wing_system(
    wing_geometry: Geometry,
    inputs: DesignInputs,
) -> Tuple[Geometry, Aerodynamics, Structure, float, float]:
    """
    Vertical wing is identical in size to horizontal wing but uses NACA0006 airfoil.
    """
    S = wing_geometry.area_m2
    b = wing_geometry.span_m
    c_root = wing_geometry.c_root_m
    c_tip = wing_geometry.c_tip_m
    MAC = wing_geometry.MAC_m
    y_MAC = wing_geometry.y_MAC_from_centerline_m
    AR = wing_geometry.AR
    c_MGC = wing_geometry.c_MGC_m
    ac_on_MAC = wing_geometry.ac_on_MAC_m

    aero_vertical_wing = vertical_wing_aerodynamic_coeffs(S, MAC, AR, inputs)

    L_vertical_wing = 0.0
    vertical_wing_structure, W_vertical_wings = calculate_wing_structure(
        L_wing=L_vertical_wing,
        S=S,
        c_root=c_root,
        taper_ratio=inputs.taper_ratio,
        b=b,
        c_MGC=c_MGC,
        y_MAC=y_MAC,
        max_thickness_fraction=aero_vertical_wing['max_thickness_fraction_of_chord'],
        CM_max=aero_vertical_wing['CM_max'],
        inputs=inputs,
    )

    wing_le_x = inputs.cg_x_wing - c_MGC / 2.0
    x_ac_vertical_wing = wing_le_x + ac_on_MAC

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
        CL_cruise=aero_vertical_wing['CL_cruise'],
        CL_max=aero_vertical_wing['CL_max'],
        CD0=aero_vertical_wing['CD0'],
        CD_cruise=aero_vertical_wing['CD_cruise'],
        CD0_fuselage=aero_vertical_wing['CD0_fuselage'],
        CM_cruise=aero_vertical_wing['CM_cruise'],
        CM_max=aero_vertical_wing['CM_max'],
        alpha_cruise_deg=aero_vertical_wing['alpha_cruise'],
        L_over_D_cruise=aero_vertical_wing['L_over_D_cruise'],
        V_stall=aero_vertical_wing['V_stall'],
        Re_cruise=aero_vertical_wing['Re_cruise'],
        max_thickness_fraction_of_chord=aero_vertical_wing['max_thickness_fraction_of_chord'],
    )

    return geometry, aerodynamics, vertical_wing_structure, x_ac_vertical_wing, W_vertical_wings


def _calculate_vertical_tail_system(
    tail_geometry: Geometry,
    inputs: DesignInputs,
) -> Tuple[Geometry, Aerodynamics, Structure, float, float]:
    """
    Vertical tail is identical in size to horizontal tail but uses NACA0006 airfoil.

    FIX (FOREVER):
      - We DO NOT recompute span from area here.
      - We clone the *physical* geometry from the (already-correct) horizontal tail geometry.
      - So if horizontal tail span includes extension, vertical tail span includes it too.
    """
    S = tail_geometry.area_m2
    b = tail_geometry.span_m
    c_root = tail_geometry.c_root_m
    c_tip = tail_geometry.c_tip_m
    MAC = tail_geometry.MAC_m
    y_MAC = tail_geometry.y_MAC_from_centerline_m
    AR = tail_geometry.AR
    c_MGC = tail_geometry.c_MGC_m
    ac_on_MAC = tail_geometry.ac_on_MAC_m

    aero_vertical_tail = vertical_tail_aerodynamic_coeffs(S, MAC, AR, inputs)

    L_vertical_tail = 0.0
    vertical_tail_structure, W_vertical_tails = calculate_tail_structure(
        L_tail=abs(L_vertical_tail),
        S=S,
        c_root=c_root,
        taper_ratio=inputs.tail_taper_ratio,
        b=b,
        c_MGC=c_MGC,
        y_MAC=y_MAC,
        max_thickness_fraction=aero_vertical_tail['max_thickness_fraction_of_chord'],
        CM_max=aero_vertical_tail['CM_max'],
        inputs=inputs,
        propulsion_selection=None,
    )

    x_ac_vertical_tail = inputs.cg_x_tail - 0.5 * c_root + ac_on_MAC

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
        CL_cruise=aero_vertical_tail['CL_cruise'],
        CL_max=aero_vertical_tail['CL_max'],
        CD0=aero_vertical_tail['CD0'],
        CD_cruise=aero_vertical_tail['CD_cruise'],
        CD0_fuselage=aero_vertical_tail['CD0_fuselage'],
        CM_cruise=aero_vertical_tail['CM_cruise'],
        CM_max=aero_vertical_tail['CM_max'],
        alpha_cruise_deg=aero_vertical_tail['alpha_cruise'],
        L_over_D_cruise=aero_vertical_tail['L_over_D_cruise'],
        V_stall=aero_vertical_tail['V_stall'],
        Re_cruise=aero_vertical_tail['Re_cruise'],
        max_thickness_fraction_of_chord=aero_vertical_tail['max_thickness_fraction_of_chord'],
    )

    return geometry, aerodynamics, vertical_tail_structure, x_ac_vertical_tail, W_vertical_tails


def _assemble_results(
    W0_mass: float,
    W0_N: float,
    L_wing: float,
    x_cg: float,
    drag_cruise: float,
    cruise_power: float,
    cruise_power_elec: float,
    wing_geometry: Geometry,
    wing_aerodynamics: Aerodynamics,
    wing_structure: Structure,
    x_ac_wing: float,
    wing_moment: float,
    tail_geometry: Geometry,
    tail_aerodynamics: Aerodynamics,
    tail_structure: Structure,
    x_ac_tail: float,
    L_tail: float,
    tail_aero_dict: Dict[str, Any],
    vertical_wing_geometry: Geometry,
    vertical_wing_aerodynamics: Aerodynamics,
    vertical_wing_structure: Structure,
    x_ac_vertical_wing: float,
    vertical_tail_geometry: Geometry,
    vertical_tail_aerodynamics: Aerodynamics,
    vertical_tail_structure: Structure,
    x_ac_vertical_tail: float,
    fuselage_t_shell: float,
    W_fuselage: float,
    battery_results: Optional[Any],
    propulsion_results: Optional[Any],
    avionics_results: AvionicsResults,
    inputs_dict: Dict[str, Any],
) -> DesignResults:

    wing_results = WingResults(
        geometry=wing_geometry,
        aerodynamics=wing_aerodynamics,
        structure=wing_structure,
        x_ac_wing_m=x_ac_wing,
        moment_at_cg_Nm=wing_moment,
    )

    tail_results = TailResults(
        geometry=tail_geometry,
        aerodynamics=tail_aerodynamics,
        structure=tail_structure,
        x_ac_tail_m=x_ac_tail,
        lift_N=L_tail,
        tail_area_scale=tail_aero_dict["tail_area_scale"],
        tail_incidence_deg=tail_aero_dict["tail_incidence_deg"],
        cruise_elevator_deg=tail_aero_dict["delta_elev_deg"],
        CL_req=tail_aero_dict["CL_req"],
        CL_no_elev=tail_aero_dict["CL_no_elev"],
        dCL_required=tail_aero_dict["dCL_required"],
        elevator_chord_ratio=tail_aero_dict["elevator_chord_ratio"],
        eta_f=tail_aero_dict["eta_f"],
        elevator_deflection_deg=tail_aero_dict["delta_elev_deg"],
        alpha_body_deg=tail_aero_dict["alpha_body_deg"],
        alpha_equiv_deg=tail_aero_dict["alpha_equiv_deg"],
    )

    vertical_wing_results = VerticalWingResults(
        geometry=vertical_wing_geometry,
        aerodynamics=vertical_wing_aerodynamics,
        structure=vertical_wing_structure,
        x_ac_wing_m=x_ac_vertical_wing,
    )

    vertical_tail_results = VerticalTailResults(
        geometry=vertical_tail_geometry,
        aerodynamics=vertical_tail_aerodynamics,
        structure=vertical_tail_structure,
        x_ac_tail_m=x_ac_vertical_tail,
    )

    fuselage_results = FuselageResults(
        t_shell_m=fuselage_t_shell,
        W_fuselage_kg=W_fuselage,
    )

    aircraft_results = AircraftResults(
        W0_mass_kg=W0_mass,
        W0_weight_N=W0_N,
        L_wing_N=L_wing,
        x_cg_m=x_cg,
        drag_cruise_N=drag_cruise,
        cruise_power_W=cruise_power,
        cruise_power_elec_W=cruise_power_elec,
    )

    return DesignResults(
        aircraft=aircraft_results,
        wing=wing_results,
        tail=tail_results,
        vertical_wing=vertical_wing_results,
        vertical_tail=vertical_tail_results,
        fuselage=fuselage_results,
        battery=battery_results,
        propulsion=propulsion_results,
        avionics=avionics_results,
        inputs=inputs_dict,
    )


def run_design(inputs: DesignInputs, plot_convergence: bool = False) -> DesignResults:
    """
    Main design iteration loop.
    """
    W0_mass = inputs.payload_mass / inputs.payload_fraction_hist
    W_tails = 0.0
    W_vertical_tails = 0.0
    L_tail = 0.0
    S_tail = 0.0
    aero_tail_dict = {'CD_cruise': 0.0, 'CM_cruise': 0.0}
    tail_span_for_props_m = 1.0

    battery_selection = None
    propulsion_selection = None
    avionics_selection = None

    mass_history: List[float] = [W0_mass]
    iteration_numbers: List[int] = [0]

    for i in range(DESIGN.max_iterations):
        W0_N = W0_mass * ATMOSPHERIC.g
        q = 0.5 * ATMOSPHERIC.rho * inputs.speed**2

        wing_geom, wing_aero, wing_struct, x_ac_wing, wing_M_cg, W_wings = _calculate_wing_system(
            W0_N, L_tail, inputs
        )

        vertical_wing_geom, vertical_wing_aero, vertical_wing_struct, x_ac_vertical_wing, W_vertical_wings = _calculate_vertical_wing_system(
            wing_geom, inputs
        )

        fuselage_t_shell, W_fuselage = calculate_fuselage_structure(
            wing_struct.bending_moment_Nm,
            inputs,
        )

        vertical_wing_CD_contribution = vertical_wing_aero.CD_cruise * (vertical_wing_geom.area_m2 / wing_geom.area_m2)

        # NOTE: S_tail/aero_tail_dict are from previous iteration initially
        drag_cruise = q * wing_geom.area_m2 * (wing_aero.CD_cruise + vertical_wing_CD_contribution) + q * S_tail * aero_tail_dict['CD_cruise']
        cruise_power = drag_cruise * inputs.speed
        cruise_power_elec = cruise_power / inputs.propeller_efficiency

        fuselage_ID = inputs.fuselage_OD - 2 * fuselage_t_shell

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
            wing_CD0=wing_aero.CD0,
        )

        # Update tail extension requirement from propulsion clearance
        if propulsion_selection is not None:
            req_ext = float(propulsion_selection.get("requires_tail_span_extension_m", 0.0) or 0.0)
            if req_ext > float(getattr(inputs, "tail_span_extension_m", 0.0) or 0.0) + 1e-9:
                inputs.tail_span_extension_m = req_ext
                print(
                    f"[TAIL EXTENSION] Added tail span extension = {req_ext:.3f} m "
                    f"(applied equally to horizontal + vertical tail) to meet prop clearance."
                )

        if avionics_selection is not None and propulsion_selection is not None:
            batt_voltage = battery_selection["battery"]["voltage_V"]

            power_per_motor_for_esc_W = propulsion_selection["esc_sizing_power_per_motor_W"]
            esc_cont_current_A = power_per_motor_for_esc_W / (batt_voltage * inputs.esc_efficiency)

            avionics_selection = estimate_avionics_and_electrics(
                W0_mass_kg=W0_mass,
                n_motors=inputs.n_propellers,
                esc_cont_current_A=esc_cont_current_A,
                autopilot_class=inputs.autopilot_class,
                n_servos=inputs.n_servos,
            )

        W_battery, W_propulsion, W_avionics = calculate_component_masses(
            battery_selection,
            propulsion_selection,
            avionics_selection,
            inputs,
        )

        W0_mass_new = (
            inputs.payload_mass + W_battery + W_propulsion +
            W_fuselage + W_wings + W_vertical_wings + W_tails + W_vertical_tails + W_avionics
        )

        mass_history.append(W0_mass_new)
        iteration_numbers.append(i + 1)

        if i > 0:
            converged, _ = check_convergence(W0_mass_new, W0_mass)
            if converged:
                W0_mass = W0_mass_new
                break

        W0_mass = W0_mass_new

        # Clearance check: Wing TE to Tail LE
        wing_le_x = inputs.cg_x_wing - 0.5 * float(wing_geom.c_MGC_m)
        wing_te_x = wing_le_x + float(wing_geom.c_root_m)

        tail_le_x = inputs.cg_x_tail - 0.5 * inputs.tail_c_root
        gap_m = tail_le_x - wing_te_x

        min_gap_m = max(
            0.20 * float(wing_geom.c_root_m),
            0.50 * float(inputs.tail_c_root),
            0.03
        )

        if gap_m <= 0.0:
            raise ValueError(
                f"Invalid geometry: Tail leading edge is not behind wing trailing edge.\n"
                f"  wing_le_x = {wing_le_x:.3f} m\n"
                f"  wing_te_x = {wing_te_x:.3f} m\n"
                f"  tail_le_x = {tail_le_x:.3f} m\n"
                f"  gap = {gap_m:.3f} m (must be > 0)"
            )

        if gap_m < min_gap_m:
            raise ValueError(
                f"Invalid geometry: insufficient wing-to-tail clearance.\n"
                f"  Required gap >= {min_gap_m:.3f} m\n"
                f"  Actual gap   = {gap_m:.3f} m\n"
                f"  (Computed from wing TE @ {wing_te_x:.3f} m and tail LE @ {tail_le_x:.3f} m)\n"
                f"Try moving tail aft (increase tail_le_x / fuselage_tail_length) or moving wing forward."
            )

        x_cg = _calculate_cg_position(
            W_wings, W_vertical_wings, W_fuselage, W_tails, W_vertical_tails, W_battery, W_propulsion, W_avionics,
            avionics_selection, inputs, W0_mass
        )

        # Update wing moment with actual CG position
        M_ac_wing = wing_aero.CM_cruise * q * wing_geom.area_m2 * wing_geom.MAC_m
        L_wing = W0_N - L_tail
        wing_M_cg = (
            M_ac_wing +
            L_wing * math.cos(wing_aero.alpha_cruise_deg * math.pi / 180.0) * (x_cg - x_ac_wing)
        )

        # Tail system (this now ALWAYS returns physical span including extension)
        tail_geom, tail_aero, tail_struct, x_ac_tail, L_tail, aero_tail_dict = _calculate_tail_system(
            x_cg,
            wing_geom.MAC_m,
            wing_geom.area_m2,
            wing_aero.alpha_cruise_deg,
            wing_M_cg,
            wing_aero.CD_cruise,
            inputs,
            propulsion_selection,
        )

        # Use physical span for next iteration clearance check in propulsion sizing
        tail_span_for_props_m = float(tail_geom.span_m)

        S_tail = tail_geom.area_m2
        W_tails = tail_struct.W_total_kg * 2

        vertical_tail_geom, vertical_tail_aero, vertical_tail_struct, x_ac_vertical_tail, W_vertical_tails = _calculate_vertical_tail_system(
            tail_geom, inputs
        )

        vertical_tail_CD_contribution = vertical_tail_aero.CD_cruise * (vertical_tail_geom.area_m2 / wing_geom.area_m2)
        drag_cruise = (
            q * wing_geom.area_m2 * (wing_aero.CD_cruise + vertical_wing_CD_contribution)
            + q * S_tail * tail_aero.CD_cruise
            + q * wing_geom.area_m2 * vertical_tail_CD_contribution
        )

        W0_mass_new = (
            inputs.payload_mass + W_battery + W_propulsion +
            W_fuselage + W_wings + W_tails + W_vertical_wings + W_vertical_tails + W_avionics
        )

    if plot_convergence and len(mass_history) > 1:
        plot_mass_convergence(iteration_numbers, mass_history)

    battery_results, propulsion_results, avionics_results = convert_to_results(
        battery_selection,
        propulsion_selection,
        avionics_selection,
    )

    inputs_dict = asdict(inputs)

    # Attach envelope / mission profile
    if battery_selection is not None:
        inputs_dict["mission_profile"] = battery_selection.get("mission_profile")
        inputs_dict["required_mission_energy_Wh"] = battery_selection.get("required_mission_energy_Wh")
        inputs_dict["battery_usable_energy_Wh"] = battery_selection.get("usable_energy_Wh")

    if propulsion_selection is not None:
        inputs_dict["esc_sizing_power_per_motor_W"] = propulsion_selection.get("esc_sizing_power_per_motor_W")
        inputs_dict["esc_cont_current_A_per_motor"] = propulsion_selection.get("esc_cont_current_A_per_motor")
        inputs_dict["envelope_peak_thrust_per_motor_N"] = propulsion_selection.get("envelope_peak_thrust_per_motor_N")
        inputs_dict["envelope_peak_power_per_motor_W"] = propulsion_selection.get("envelope_peak_power_per_motor_W")

    # Explicit tail span outputs (so printing + DoE can always extract)
    ext = float(getattr(inputs, "tail_span_extension_m", 0.0) or 0.0)
    inputs_dict["tail_span_extension_m"] = ext
    inputs_dict["tail_span_total_m"] = float(tail_geom.span_m)
    inputs_dict["tail_span_base_m"] = float(tail_geom.span_m) - ext
    inputs_dict["tail_area_total_m2"] = float(tail_geom.area_m2)

    # Recalculate vertical wings and tails for final results
    vertical_wing_geom, vertical_wing_aero, vertical_wing_struct, x_ac_vertical_wing, W_vertical_wings = _calculate_vertical_wing_system(
        wing_geom, inputs
    )
    vertical_tail_geom, vertical_tail_aero, vertical_tail_struct, x_ac_vertical_tail, W_vertical_tails = _calculate_vertical_tail_system(
        tail_geom, inputs
    )

    results = _assemble_results(
        W0_mass, W0_N, W0_N - L_tail, x_cg,
        drag_cruise, cruise_power, cruise_power_elec,
        wing_geom, wing_aero, wing_struct, x_ac_wing, wing_M_cg,
        tail_geom, tail_aero, tail_struct, x_ac_tail, L_tail, aero_tail_dict,
        vertical_wing_geom, vertical_wing_aero, vertical_wing_struct, x_ac_vertical_wing,
        vertical_tail_geom, vertical_tail_aero, vertical_tail_struct, x_ac_vertical_tail,
        fuselage_t_shell, W_fuselage,
        battery_results, propulsion_results, avionics_results,
        inputs_dict,
    )

    return results


if __name__ == "__main__":
    inputs = DesignInputs()
    results = run_design(inputs)

    if getattr(inputs, "export_inventor_xml", False):
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

    print_all_results(results, inputs)
    results_dict = results.to_dict()
