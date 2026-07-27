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
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import asdict

from design_inputs import DesignInputs
from constants import ATMOSPHERIC, DESIGN
from geometry import wing_geometry, tail_geometry
from aerodynamics import wing_aerodynamic_coeffs, tail_aerodynamic_coeffs
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
    FuselageResults,
    Geometry,
    Aerodynamics,
    Structure,
    AvionicsResults,
)
from reporting import print_all_results
from avionics import estimate_avionics_and_electrics

def _calculate_cg_position(
    W_wings: float,
    W_fuselage: float,
    W_tails: float,
    W_battery: float,
    W_propulsion: float,
    W_avionics: float,
    avionics_selection: Any,
    inputs: DesignInputs,
    W0_mass: float,
) -> float:
    """
    Calculate center of gravity position.
    
    Args:
        W_wings: Total wing mass [kg]
        W_fuselage: Fuselage mass [kg]
        W_tails: Total tail mass [kg]
        W_battery: Battery mass [kg]
        W_propulsion: Propulsion mass [kg]
        W_avionics: Total avionics mass [kg]
        avionics_selection: Avionics selection object
        inputs: Design inputs
        W0_mass: Total aircraft mass [kg]
    
    Returns:
        CG x-position [m]
    """
    num = (
        W_wings * inputs.cg_x_wing +
        W_fuselage * inputs.cg_x_fuselage +
        W_tails * inputs.cg_x_tail +
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
) -> Tuple[Geometry, Aerodynamics, Structure, float, float]:
    """
    Calculate complete wing system (geometry, aerodynamics, structure).
    
    Args:
        W0_N: Aircraft weight [N]
        L_tail: Tail lift [N]
        inputs: Design inputs
    
    Returns:
        Tuple of (geometry, aerodynamics, structure, x_ac_wing, moment_at_cg)
    """
    L_wing = W0_N - L_tail
    q = 0.5 * ATMOSPHERIC.rho * inputs.speed**2
    
    # Geometry
    S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC = wing_geometry(L_wing, inputs)
    
    # Aerodynamics
    aero_wing = wing_aerodynamic_coeffs(L_wing, L_tail, S, MAC, AR, inputs)
    
    # Validate stall speed
    if aero_wing['V_stall'] > 0.75 * inputs.speed:
        raise ValueError(
            f"Configuration is invalid due to the large stall speed. "
            f"Stall speed ({aero_wing['V_stall']:.2f} m/s) exceeds 0.75 * cruise speed ({0.75 * inputs.speed:.2f} m/s)."
        )
    
    # Structure
    wing_structure, W_wings = calculate_wing_structure(
        L_wing=L_wing,
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
    
    # Wing moment at AC (will be updated with actual CG later)
    M_ac_wing = aero_wing['CM_cruise'] * q * S * MAC
    M_cg_wing = M_ac_wing  # Initial estimate, will be updated
    
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
) -> Tuple[Geometry, Aerodynamics, Structure, float, float, Dict[str, Any]]:
    """
    Calculate complete tail system with trim iteration.
    
    Args:
        x_cg: Center of gravity x-position [m]
        wing_MAC: Wing MAC [m]
        wing_S: Wing area [m²]
        wing_alpha_cruise: Wing cruise angle of attack [deg]
        wing_M_cg: Wing moment at CG [N·m]
        wing_CD_cruise: Wing cruise drag coefficient
        inputs: Design inputs
    
    Returns:
        Tuple of (geometry, aerodynamics, structure, x_ac_tail, L_tail, aero_tail_dict)
    """
    q = 0.5 * ATMOSPHERIC.rho * inputs.speed**2
    
    # Initial tail geometry (nominal)
    S_tail_nom, b_tail, c_root_tail, c_tip_tail, MAC_tail, y_MAC_tail, AR_tail, c_MGC_tail, ac_on_MAC_tail = tail_geometry(
        x_cg, wing_MAC, wing_S, inputs
    )
    
    x_ac_tail = inputs.tail_le_x + ac_on_MAC_tail
    
    # Calculate required tail lift for trim (iterative)
    # Start with initial estimate
    L_tail = wing_M_cg / (x_ac_tail - x_cg)
    
    # Iterate tail area scaling until incidence + elevator constraints satisfied
    tail_area_scale = 1.0
    aero_tail = None
    
    while tail_area_scale <= inputs.tail_area_scale_max + 1e-9:
        aero_try = tail_aerodynamic_coeffs(
            L_required_N=L_tail,
            S=S_tail_nom,
            MAC=MAC_tail,
            AR=AR_tail,
            alpha_body_deg=wing_alpha_cruise,
            inputs=inputs,
            tail_area_scale=tail_area_scale,
            downwash_deg=0.0,
        )
        
        inc = aero_try["tail_incidence_deg"]
        de = aero_try["delta_elev_deg"]
        
        # Hard constraints
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
    
    # Use the scaled tail area for structural sizing and drag
    S_tail = S_tail_nom * aero_tail["tail_area_scale"]
    
    # Update tail geometry with scaled area
    lam_t = inputs.tail_taper_ratio
    b_tail = 2.0 * S_tail / (inputs.tail_c_root * (1.0 + lam_t))
    AR_tail = b_tail**2 / S_tail
    c_MGC_tail = S_tail / b_tail
    
    # Recalculate tail moment with actual aero
    M_ac_tail = aero_tail['CM_cruise'] * q * S_tail * MAC_tail
    
    # Recalculate tail lift with updated moments (iterative refinement)
    L_tail_new = (wing_M_cg + M_ac_tail) / (x_ac_tail - x_cg)
    # Use average for stability
    L_tail = 0.5 * (L_tail + L_tail_new) if abs(L_tail_new - L_tail) / max(abs(L_tail), 1e-6) > 0.1 else L_tail_new
    
    # Structure
    tail_structure, W_tails = calculate_tail_structure(
        L_tail=abs(L_tail),
        c_root=c_root_tail,
        taper_ratio=inputs.tail_taper_ratio,
        b=b_tail,
        c_MGC=c_MGC_tail,
        y_MAC=y_MAC_tail,
        max_thickness_fraction=aero_tail['max_thickness_fraction_of_chord'],
        CM_max=aero_tail['CM_max'],
        inputs=inputs,
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
        CM_cruise=aero_tail['CM_cruise'],
        CM_max=aero_tail['CM_max'],
        alpha_cruise_deg=aero_tail['alpha_equiv_deg'],
        L_over_D_cruise=aero_tail['L_over_D_cruise'],
        V_stall=aero_tail['V_stall'],
        Re_cruise=aero_tail['Re_cruise'],
        max_thickness_fraction_of_chord=aero_tail['max_thickness_fraction_of_chord'],
    )
    
    return geometry, aerodynamics, tail_structure, x_ac_tail, L_tail, aero_tail


def _assemble_results(
    W0_mass: float,
    W0_N: float,
    L_wing: float,
    x_cg: float,
    drag_cruise: float,
    drag_cruise_old: float,
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
    fuselage_t_shell: float,
    W_fuselage: float,
    battery_results: Optional[Any],
    propulsion_results: Optional[Any],
    avionics_results: AvionicsResults,
    inputs: DesignInputs,
) -> DesignResults:
    """Assemble all results into structured DesignResults."""
    
    # Wing moment already includes CG correction
    wing_moment_updated = wing_moment
    
    wing_results = WingResults(
        geometry=wing_geometry,
        aerodynamics=wing_aerodynamics,
        structure=wing_structure,
        x_ac_wing_m=x_ac_wing,
        moment_at_cg_Nm=wing_moment_updated,
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
        drag_cruise_old_N=drag_cruise_old,
        cruise_power_W=cruise_power,
        cruise_power_elec_W=cruise_power_elec,
    )
    
    return DesignResults(
        aircraft=aircraft_results,
        wing=wing_results,
        tail=tail_results,
        fuselage=fuselage_results,
        battery=battery_results,
        propulsion=propulsion_results,
        avionics=avionics_results,
        inputs=asdict(inputs),
    )


def run_design(inputs: DesignInputs, plot_convergence: bool = True) -> DesignResults:
    """
    Main design iteration loop.
    
    Args:
        inputs: Design inputs
        plot_convergence: Whether to plot convergence history
    
    Returns:
        DesignResults with complete design
    """
    # Initialize
    W0_mass = inputs.payload_mass / inputs.payload_fraction_hist
    W_tails = 0.0
    L_tail = 0.0
    S_tail = 0.0
    aero_tail_dict = {'CD_cruise': 0.0, 'CM_cruise': 0.0}
    
    battery_selection = None
    propulsion_selection = None
    avionics_selection = None
    
    # Track convergence
    mass_history: List[float] = [W0_mass]
    iteration_numbers: List[int] = [0]
    
    for i in range(DESIGN.max_iterations):
        W0_N = W0_mass * ATMOSPHERIC.g
        q = 0.5 * ATMOSPHERIC.rho * inputs.speed**2
        
        # Wing system
        wing_geom, wing_aero, wing_struct, x_ac_wing, wing_M_cg, W_wings = _calculate_wing_system(
            W0_N, L_tail, inputs
        )
        
        # Fuselage
        fuselage_t_shell, W_fuselage = calculate_fuselage_structure(
            wing_struct.bending_moment_Nm,
            inputs,
        )
        
        # Propulsion system
        drag_cruise = q * wing_geom.area_m2 * wing_aero.CD_cruise + q * S_tail * aero_tail_dict['CD_cruise']
        drag_cruise_old = (W0_N - L_tail) / wing_aero.L_over_D_cruise
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
        )
        
        # Update avionics with actual mass
        if avionics_selection is not None and propulsion_selection is not None:
            batt_voltage = battery_selection["battery"]["voltage_V"]
            esc_cont_current_A = (
                propulsion_selection["required_power_per_motor_W"] /
                (batt_voltage * inputs.esc_efficiency)
            )
            avionics_selection = estimate_avionics_and_electrics(
                W0_mass_kg=W0_mass,
                n_motors=inputs.n_propellers,
                esc_cont_current_A=esc_cont_current_A,
                autopilot_class=inputs.autopilot_class,
                n_servos=inputs.n_servos,
            )
        
        # Component masses
        W_battery, W_propulsion, W_avionics = calculate_component_masses(
            battery_selection,
            propulsion_selection,
            avionics_selection,
            inputs,
        )
        
        # New mass estimate
        W0_mass_new = (
            inputs.payload_mass + W_battery + W_propulsion +
            W_fuselage + W_wings + W_tails + W_avionics
        )
        
        # Track convergence
        mass_history.append(W0_mass_new)
        iteration_numbers.append(i + 1)
        
        # Check convergence
        if i > 0:
            converged, _ = check_convergence(W0_mass_new, W0_mass)
            if converged:
                W0_mass = W0_mass_new
                break
        
        W0_mass = W0_mass_new
        
        # Calculate CG
        x_cg = _calculate_cg_position(
            W_wings, W_fuselage, W_tails, W_battery, W_propulsion, W_avionics,
            avionics_selection, inputs, W0_mass
        )
        
        # Update wing moment with actual CG position
        q = 0.5 * ATMOSPHERIC.rho * inputs.speed**2
        M_ac_wing = wing_aero.CM_cruise * q * wing_geom.area_m2 * wing_geom.MAC_m
        L_wing = W0_N - L_tail
        wing_M_cg = (
            M_ac_wing +
            L_wing * math.cos(wing_aero.alpha_cruise_deg * math.pi / 180.0) * (x_cg - x_ac_wing)
        )
        
        # Tail system (requires updated CG and wing properties)
        tail_geom, tail_aero, tail_struct, x_ac_tail, L_tail, aero_tail_dict = _calculate_tail_system(
            x_cg,
            wing_geom.MAC_m,
            wing_geom.area_m2,
            wing_aero.alpha_cruise_deg,
            wing_M_cg,
            wing_aero.CD_cruise,
            inputs,
        )
        
        S_tail = tail_geom.area_m2
        W_tails = tail_struct.W_total_kg * 2  # Both cruciform surfaces
    
    # Plot convergence
    if plot_convergence and len(mass_history) > 1:
        plot_mass_convergence(iteration_numbers, mass_history)
    
    # Convert selections to results
    battery_results, propulsion_results, avionics_results = convert_to_results(
        battery_selection,
        propulsion_selection,
        avionics_selection,
    )
    
    # Assemble final results
    results = _assemble_results(
        W0_mass, W0_N, W0_N - L_tail, x_cg,
        drag_cruise, drag_cruise_old, cruise_power, cruise_power_elec,
        wing_geom, wing_aero, wing_struct, x_ac_wing, wing_M_cg,
        tail_geom, tail_aero, tail_struct, x_ac_tail, L_tail, aero_tail_dict,
        fuselage_t_shell, W_fuselage,
        battery_results, propulsion_results, avionics_results,
        inputs,
    )
    
    return results


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    inputs = DesignInputs()
    results = run_design(inputs)
    
    # Print all results
    print_all_results(results, inputs)
    
    # Also provide dictionary format for backward compatibility
    results_dict = results.to_dict()
