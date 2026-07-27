"""
Conceptual sizing script following the SAKR-2 SUAV design procedure
(Zakaria et al., "Design and Production of Small Tailless UAV") :contentReference[oaicite:5]{index=5}
with Raymer-style weight & performance logic. :contentReference[oaicite:6]{index=6}

Workflow:
1) Mission + payload fraction  -> MTOW (W0)
2) Choose design point (W/S, P/W)
3) Wing sizing: S, b, chords, MAC
4) Aerodynamics at cruise: CL, CD, L/D, Re, stall speed
5) Propulsion: required cruise power, installed motor power
6) Energy: battery sizing, endurance estimate
"""

import math
from dataclasses import dataclass, asdict
from airfoil_naca0006 import interpolate_by_alpha
from batteries import get_battery_database
from design_inputs import DesignInputs
from helper_function import select_battery, select_propulsion
from propulsion import get_propulsion_database
from avionics import estimate_avionics_and_electrics

# ============================================================
# CONSTANTS
# ============================================================

g = 9.81             # gravitational acceleration [m/s²]
rho = 1.2            # air density at ~150 m altitude [kg/m³]
mu = 1.8e-5          # dynamic viscosity of air [kg/(m·s)] ~ ISA sea-level

max_iterations = 100 # max number of iterations to find the MTOW

# ============================================================
# SIZING FUNCTIONS
# ============================================================

def wing_geometry(W0_N: float, inputs: DesignInputs):
    """
    Step 3: Wing span, chords, MAC, AR from c_root, taper_ratio, and wing_loading.

    Given:
        c_root (design input)
        taper_ratio λ = c_tip / c_root
        wing_loading W/S
    
    Calculate:
        S = W0_N / (W/S)
        b from trapezoidal wing area: S = b * c_root * (1 + λ) / 2
        c_tip = λ * c_root
        AR = b² / S

    Mean aerodynamic chord (standard trapezoidal wing formula):
        MAC = 2/3 * c_root * (1 + λ + λ²) / (1 + λ)

    Spanwise position of MAC from centerline (for reference):
        y_MAC = b/6 * (1 + 2λ)/(1 + λ)

    Returns:
        S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, x_ac
    """
    lam = inputs.taper_ratio
    c_root = inputs.c_root

    # Calculate wing area from wing loading
    S = W0_N / inputs.wing_loading

    # Calculate span from trapezoidal wing area formula: S = b * (c_root + c_tip) / 2
    # Rearranging: b = 2*S / (c_root + c_tip) = 2*S / (c_root * (1 + λ))
    b = 2.0 * S / (c_root * (1.0 + lam))

    # Calculate tip chord
    c_tip = lam * c_root

    # Calculate aspect ratio
    AR = b**2 / S

    # Calculate mean aerodynamic chord
    MAC = (2.0 / 3.0) * c_root * (1.0 + lam + lam**2) / (1.0 + lam)
    
    # Spanwise position of MAC from centerline
    y_MAC = (b / 6.0) * (1.0 + 2.0 * lam) / (1.0 + lam)
    
    # Mean geometric chord
    c_MGC = S / b
    
    # Aerodynamic center position
    x_ac = 0.25 * y_MAC

    return S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, x_ac


def tail_geometry(W0_N: float, inputs: DesignInputs):
    """
    Tail geometry calculations from c_root, taper_ratio, and tail_wing_loading.

    Given:
        tail_c_root (design input)
        tail_taper_ratio λ = c_tip / c_root
        tail_wing_loading W/S
    
    Calculate:
        S = W0_N / (W/S)
        b from trapezoidal tail area: S = b * c_root * (1 + λ) / 2
        c_tip = λ * c_root
        AR = b² / S

    Mean aerodynamic chord (standard trapezoidal wing formula):
        MAC = 2/3 * c_root * (1 + λ + λ²) / (1 + λ)

    Spanwise position of MAC from centerline (for reference):
        y_MAC = b/6 * (1 + 2λ)/(1 + λ)

    Returns:
        S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, x_ac
    """
    lam = inputs.tail_taper_ratio
    c_root = inputs.tail_c_root

    # Calculate tail area from tail wing loading
    S = W0_N / inputs.tail_wing_loading

    # Calculate span from trapezoidal tail area formula: S = b * (c_root + c_tip) / 2
    # Rearranging: b = 2*S / (c_root + c_tip) = 2*S / (c_root * (1 + λ))
    b = 2.0 * S / (c_root * (1.0 + lam))

    # Calculate tip chord
    c_tip = lam * c_root

    # Calculate aspect ratio
    AR = b**2 / S

    # Calculate mean aerodynamic chord
    MAC = (2.0 / 3.0) * c_root * (1.0 + lam + lam**2) / (1.0 + lam)
    
    # Spanwise position of MAC from centerline
    y_MAC = (b / 6.0) * (1.0 + 2.0 * lam) / (1.0 + lam)
    
    # Mean geometric chord
    c_MGC = S / b
    
    # Aerodynamic center position
    x_ac = 0.25 * y_MAC

    return S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, x_ac


def aerodynamic_coeffs(W0_N: float, S: float, MAC: float, AR: float, inputs: DesignInputs):
    """
    Step 4: CL, CD, L/D at cruise, stall speed, Reynolds number.

    Level-flight lift coefficient (from W = L = q S CL):
        CL_cruise = W / (0.5 * rho * V_cruise² * S)

    Induced drag:
        CDi = CL² / (π * AR * e)

    Total drag:
        CD_total = CD0 + CDi

    Stall speed (using CL_max):
        V_stall = sqrt( 2W / (rho * S * CL_max) )

    Reynolds number at cruise:
        Re = rho * V_cruise * MAC / mu
    """
    W = W0_N
    V = inputs.speed
    q = 0.5 * rho * V**2

    CL = W / (q * S)

    # Slender-wing 3D lift curve slope
    CL_alpha_3D = 0.5 * math.pi * AR
    alpha_cruise_rad = CL / CL_alpha_3D
    alpha_cruise_deg = alpha_cruise_rad * 180.0 / math.pi

    airfoil_data = interpolate_by_alpha(alpha_cruise_deg)
    CLmax_2D = airfoil_data['CL_max']
    CL_max = CLmax_2D * (AR / (AR + 2.0))

    airfoil_data_zero_angle = interpolate_by_alpha(0.0)
    CD0 = airfoil_data_zero_angle['CD']
    
    CD_profile = airfoil_data['CD']
    CD_induced = CL**2 / (math.pi * AR * inputs.e_oswald)
    
    CD_total = CD_profile + CD_induced
    
    L_over_D = CL / (CD_total*2) # two wings forming a cruciform

    V_stall = math.sqrt(2.0 * W / (rho * S * CL_max))
    Re = rho * V * MAC / mu

    return {
        "CL_cruise": CL,
        "CL_max": airfoil_data['CL_max'],
        "CD0": CD0,
        "CD_cruise": CD_total,
        "CM_cruise": airfoil_data['CM'],
        "CM_max": airfoil_data['CM_max'],
        "alpha_cruise": alpha_cruise_deg,
        "L_over_D_cruise": L_over_D,
        "V_stall": V_stall,
        "Re_cruise": Re,
        "max_thickness_fraction_of_chord": airfoil_data['max_thickness_fraction_of_chord'],
    }


def tail_aerodynamic_coeffs(W0_N: float, S: float, MAC: float, AR: float, inputs: DesignInputs):
    """
    Tail aerodynamic coefficients (same as wing but using tail parameters).
    """
    W = W0_N
    V = inputs.speed
    q = 0.5 * rho * V**2

    CL = W / (q * S)

    # Slender-wing 3D lift curve slope
    CL_alpha_3D = 0.5 * math.pi * AR
    alpha_cruise_rad = CL / CL_alpha_3D
    alpha_cruise_deg = alpha_cruise_rad * 180.0 / math.pi

    airfoil_data = interpolate_by_alpha(alpha_cruise_deg)
    CLmax_2D = airfoil_data['CL_max']
    CL_max = CLmax_2D * (AR / (AR + 2.0))

    airfoil_data_zero_angle = interpolate_by_alpha(0.0)
    CD0 = airfoil_data_zero_angle['CD']
    
    CD_profile = airfoil_data['CD']
    CD_induced = CL**2 / (math.pi * AR * inputs.tail_e_oswald)
    
    CD_total = CD_profile + CD_induced
    
    L_over_D = CL / CD_total*2 # two tail surfaces forming a cruciform

    V_stall = math.sqrt(2.0 * W / (rho * S * CL_max))
    Re = rho * V * MAC / mu

    return {
        "CL_cruise": CL,
        "CL_max": airfoil_data['CL_max'],
        "CD0": CD0,
        "CD_cruise": CD_total,
        "CM_cruise": airfoil_data['CM'],
        "CM_max": airfoil_data['CM_max'],
        "alpha_cruise": alpha_cruise_deg,
        "L_over_D_cruise": L_over_D,
        "V_stall": V_stall,
        "Re_cruise": Re,
        "max_thickness_fraction_of_chord": airfoil_data['max_thickness_fraction_of_chord'],
    }

# ============================================================
# MASTER DRIVER
# ============================================================

def run_design(inputs: DesignInputs):
    # Step 1: MTOW from payload fraction
    W0_mass = inputs.payload_mass / inputs.payload_fraction_hist

    battery_selection = None
    propulsion_selection = None

    for i in range(max_iterations):
        W0_N = W0_mass * g

        # Step 2: Wing estimations
        S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, x_ac = wing_geometry(W0_N, inputs)

        aero = aerodynamic_coeffs(W0_N, S, MAC, AR, inputs)
        
        # Validate stall speed
        if aero['V_stall'] > 0.5 * inputs.speed:
            raise ValueError(
                f"Configuration is invalid due to the large stall speed. "
                f"Stall speed ({aero['V_stall']:.2f} m/s) exceeds 0.5 * cruise speed ({0.5 * inputs.speed:.2f} m/s)."
            )

        wing_shear = inputs.n_ult*W0_N/2
        wing_bending_moment = wing_shear*y_MAC
        wing_torsion = 0.25*rho*inputs.dive_speed**2*S*c_MGC*aero['CM_max']

        h = c_root*aero['max_thickness_fraction_of_chord']
        c_cell_root = c_root*inputs.wing_flap_span_ratio
        A_cell_root = 2*c_cell_root*h/3
        A_cell_tip = inputs.taper_ratio**2*A_cell_root
        t_skin_root = max(wing_torsion*inputs.printed_parts_sf/(2*A_cell_root*inputs.filament_shear_strength_MPa*1e6), 0.000508)
        t_skin_tip = max(t_skin_root*0.15, 0.000508)
        
        s_cell_root = math.sqrt((h/2)**2 + 4*c_cell_root**2) + (h**2/(16*c_cell_root)) * math.asinh(4*c_cell_root/h)
        s_cell_tip = inputs.taper_ratio*s_cell_root

        W_skin = inputs.filament_density * b * (t_skin_root + t_skin_tip) * s_cell_root * (1 + inputs.taper_ratio) / 8

        t_web_root = max(3*inputs.n_ult*W0_N*inputs.printed_parts_sf/(4*h*inputs.filament_shear_strength_MPa*1e6), 0.000508)
        t_web_tip = max(t_web_root*0.15, 0.000508)

        W_web = inputs.filament_density * b * (t_web_root + t_web_tip) * h * (1 + inputs.taper_ratio) / 8

        F_bend = wing_bending_moment/h
        spar_area_root = max(F_bend/(inputs.carbon_fiber_max_bending_stress_MPa*1e6), 6.4516e-6)
        spar_area_tip = max(spar_area_root*0.05, 6.4516e-6)

        W_spars = inputs.carbon_fiber_density*(spar_area_root + spar_area_tip)*b/2

        N_ribs = math.ceil(b/c_MGC)+1
        W_ribs = N_ribs*inputs.filament_density*A_cell_root*(1+inputs.taper_ratio**2)*(t_skin_root+t_skin_tip)/4

        W_wing = W_skin + W_web + W_spars + W_ribs # one of two cruciform wings
        W_wings = 2*W_wing # two wings forming a cruciform

        # Step 2b: Tail estimations
        S_tail, b_tail, c_root_tail, c_tip_tail, MAC_tail, y_MAC_tail, AR_tail, c_MGC_tail, x_ac_tail = tail_geometry(W0_N, inputs)

        aero_tail = tail_aerodynamic_coeffs(W0_N, S_tail, MAC_tail, AR_tail, inputs)

        tail_shear = inputs.n_ult*W0_N/2
        tail_bending_moment = tail_shear*y_MAC_tail
        tail_torsion = 0.25*rho*inputs.dive_speed**2*S_tail*c_MGC_tail*aero_tail['CM_max']

        h_tail = c_root_tail*aero_tail['max_thickness_fraction_of_chord']
        c_cell_root_tail = c_root_tail*inputs.tail_flap_width_ratio
        A_cell_root_tail = 2*c_cell_root_tail*h_tail/3
        A_cell_tip_tail = inputs.tail_taper_ratio**2*A_cell_root_tail
        t_skin_root_tail = max(tail_torsion*inputs.printed_parts_sf/(2*A_cell_root_tail*inputs.filament_shear_strength_MPa*1e6), 0.000508)
        t_skin_tip_tail = max(t_skin_root_tail*0.15, 0.000508)
        
        s_cell_root_tail = math.sqrt((h_tail/2)**2 + 4*c_cell_root_tail**2) + (h_tail**2/(16*c_cell_root_tail)) * math.asinh(4*c_cell_root_tail/h_tail)
        s_cell_tip_tail = inputs.tail_taper_ratio*s_cell_root_tail

        W_skin_tail = inputs.filament_density * b_tail * (t_skin_root_tail + t_skin_tip_tail) * s_cell_root_tail * (1 + inputs.tail_taper_ratio) / 8

        t_web_root_tail = max(3*inputs.n_ult*W0_N*inputs.printed_parts_sf/(4*h_tail*inputs.filament_shear_strength_MPa*1e6), 0.000508)
        t_web_tip_tail = max(t_web_root_tail*0.15, 0.000508)

        W_web_tail = inputs.filament_density * b_tail * (t_web_root_tail + t_web_tip_tail) * h_tail * (1 + inputs.tail_taper_ratio) / 8

        F_bend_tail = tail_bending_moment/h_tail
        spar_area_root_tail = max(F_bend_tail/(inputs.carbon_fiber_max_bending_stress_MPa*1e6), 6.4516e-6)
        spar_area_tip_tail = max(spar_area_root_tail*0.05, 6.4516e-6)

        W_spars_tail = inputs.carbon_fiber_density*(spar_area_root_tail + spar_area_tip_tail)*b_tail/2

        N_ribs_tail = math.ceil(b_tail/c_MGC_tail)+1
        W_ribs_tail = N_ribs_tail*inputs.filament_density*A_cell_root_tail*(1+inputs.tail_taper_ratio**2)*(t_skin_root_tail+t_skin_tip_tail)/4

        W_tail = W_skin_tail + W_web_tail + W_spars_tail + W_ribs_tail # one of two cruciform tail surfaces
        W_tails = 2*W_tail # two tail surfaces forming a cruciform

        # Step 3: Fuselage estimations - monocoque fuselage
        # Assume max fuselage bending moment occurs at wing–fuselage junction
        fuselage_bending_moment = wing_bending_moment  # [N·m], simple approximation

        # Axial bending stress resultant in cylindrical shell: NxB = M * r / Is
        fuselage_NxB = abs(
            fuselage_bending_moment*inputs.fuselage_OR/inputs.fuselage_shell_area_moment_of_inertia
        )  # [N/m]

        # Allowable axial bending stress in filament (with safety factor)
        fuselage_sigma_bend_allow = inputs.filament_max_bending_stress_MPa * 1e6 / inputs.printed_parts_sf  # [Pa]

        # Required shell thickness from bending (NxB = sigma * t  ->  t = NxB / sigma)
        fuselage_t_shell_bend = fuselage_NxB / fuselage_sigma_bend_allow  # [m]

        # Enforce minimum printable gage
        fuselage_t_shell = max(fuselage_t_shell_bend, 0.000508)  # [m]

        # Fuselage shell surface area (cylinder approximation)
        fuselage_shell_area = inputs.fuselage_cs_perimeter * inputs.fuselage_length

        # Fuselage shell mass (load-bearing structure only)
        W_fuselage_shell = (
            inputs.filament_density * fuselage_t_shell * fuselage_shell_area
        )  # [kg]

        W_fuselage_total = inputs.fuselage_nonopt_factor * W_fuselage_shell  # [kg]

        # Step 4: Propulsion estimations
        # dive_power = (0.5*aero['CD0']*rho*inputs.dive_speed**3*S + 2*W0_N**2*math.cos(inputs.dive_angle_rad)**2/(math.pi*b**2*inputs.e_oswald*rho*inputs.dive_speed) - W0_N*inputs.dive_speed*math.sin(inputs.dive_angle_rad)) / inputs.propeller_efficiency
        drag_cruise = W0_N/aero['L_over_D_cruise']
        cruise_power = drag_cruise*inputs.speed

        # Electrical power required from battery (account for prop efficiency)
        cruise_power_elec = cruise_power / inputs.propeller_efficiency

        fuselage_ID = inputs.fuselage_OD - 2*fuselage_t_shell

        # Battery selection for this cruise power and desired endurance
        battery_db = get_battery_database()
        battery_selection = select_battery(
            cruise_power_elec,
            inputs.cruise_endurance_min,
            battery_db,
            fuselage_ID
        )

        propulsion_db = get_propulsion_database()
        propulsion_selection = select_propulsion(
            drag_cruise,
            cruise_power_elec,
            propulsion_db,
            n_propellers=inputs.n_propellers,
            thrust_margin=inputs.propulsion_margin,
            power_margin=inputs.propulsion_power_margin,
            battery_voltage_V=(
                battery_selection["battery"]["voltage_V"]
                if battery_selection is not None else None
            )
        )

        batt_voltage = battery_selection["battery"]["voltage_V"]
        esc_cont_current_A = propulsion_selection["required_power_per_motor_W"] / (batt_voltage * inputs.esc_efficiency)

        avionics_selection = estimate_avionics_and_electrics(
            W0_mass_kg=W0_mass,
            n_motors=inputs.n_propellers,
            esc_cont_current_A=esc_cont_current_A,
            autopilot_class=inputs.autopilot_class,
            n_servos=inputs.n_servos,
        )

        W_battery = battery_selection["battery"]["mass_kg"]
        W_propulsion = (propulsion_selection["combo"]["motor_mass_kg"] + propulsion_selection["combo"]["prop_mass_kg"]) * inputs.n_propellers
        W_avionics = avionics_selection.total_avionics_electrics_mass_kg

        W0_mass = inputs.payload_mass+ W_battery + W_propulsion + W_fuselage_total + W_wings + W_tails + W_avionics

        # Calculate CG position using weighted average of all component masses and positions
        num = (
            W_wings * inputs.cg_x_wing +
            W_fuselage_total * inputs.cg_x_fuselage +
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

        x_cg = num / W0_mass

    # Collect results
    results = {
        "inputs": asdict(inputs),
        "W0_mass_kg": W0_mass,
        "W0_weight_N": W0_N,
        "wing_area_S_m2": S,
        "span_b_m": b,
        "AR": AR,
        "c_root_m": c_root,
        "c_tip_m": c_tip,
        "MAC_m": MAC,
        "y_MAC_from_centerline_m": y_MAC,
        "x_ac_from_centerline_m": x_ac,
        "CL_cruise": aero['CL_cruise'],
        "CL_max": aero['CL_max'],
        "CD0": aero['CD0'],
        "CD_cruise": aero['CD_cruise'],
        "CM_cruise": aero['CM_cruise'],
        "CM_max": aero['CM_max'],
        "alpha_cruise": aero['alpha_cruise'],
        "L_over_D_cruise": aero['L_over_D_cruise'],
        "V_stall": aero['V_stall'],
        "Re_cruise": aero['Re_cruise'],
        'wing_bending_moment_Nm': wing_bending_moment,
        'wing_torsion_Nm': wing_torsion,
        'h_m': h,
        't_skin_root_m': t_skin_root,
        't_skin_tip_m': t_skin_tip,
        'W_skin_kg': W_skin,
        't_web_root_m': t_web_root,
        't_web_tip_m': t_web_tip,
        'W_web_kg': W_web,
        'spar_area_root_m2': spar_area_root,
        'spar_area_tip_m2': spar_area_tip,
        'W_spars_kg': W_spars,
        'N_ribs': N_ribs,
        'W_ribs_kg': W_ribs,
        'W_wings_kg': W_wings,
        'tail_area_S_m2': S_tail,
        'tail_span_b_m': b_tail,
        'tail_AR': AR_tail,
        'tail_c_root_m': c_root_tail,
        'tail_c_tip_m': c_tip_tail,
        'tail_MAC_m': MAC_tail,
        'tail_y_MAC_from_centerline_m': y_MAC_tail,
        'tail_x_ac_from_centerline_m': x_ac_tail,
        'tail_CL_cruise': aero_tail['CL_cruise'],
        'tail_CL_max': aero_tail['CL_max'],
        'tail_CD0': aero_tail['CD0'],
        'tail_CD_cruise': aero_tail['CD_cruise'],
        'tail_CM_cruise': aero_tail['CM_cruise'],
        'tail_CM_max': aero_tail['CM_max'],
        'tail_alpha_cruise': aero_tail['alpha_cruise'],
        'tail_L_over_D_cruise': aero_tail['L_over_D_cruise'],
        'tail_V_stall': aero_tail['V_stall'],
        'tail_Re_cruise': aero_tail['Re_cruise'],
        'tail_bending_moment_Nm': tail_bending_moment,
        'tail_torsion_Nm': tail_torsion,
        'tail_h_m': h_tail,
        'tail_t_skin_root_m': t_skin_root_tail,
        'tail_t_skin_tip_m': t_skin_tip_tail,
        'tail_W_skin_kg': W_skin_tail,
        'tail_t_web_root_m': t_web_root_tail,
        'tail_t_web_tip_m': t_web_tip_tail,
        'tail_W_web_kg': W_web_tail,
        'tail_spar_area_root_m2': spar_area_root_tail,
        'tail_spar_area_tip_m2': spar_area_tip_tail,
        'tail_W_spars_kg': W_spars_tail,
        'tail_N_ribs': N_ribs_tail,
        'tail_W_ribs_kg': W_ribs_tail,
        'tail_W_tails_kg': W_tails,
        'fuselage_t_shell_m': fuselage_t_shell,
        'W_fuselage_kg': W_fuselage_total,
        'drag_cruise_N': drag_cruise,
        'cruise_power_W': cruise_power,
        'propulsion_name': propulsion_selection["combo"]["name"],
        'cruise_power_W': cruise_power,
        'cruise_power_elec_W': cruise_power_elec,
        "autopilot_name": avionics_selection.autopilot_name,
        "autopilot_mass_kg": avionics_selection.autopilot_mass_kg,
        "gps_mass_kg": avionics_selection.gps_mass_kg,
        "airspeed_mass_kg": avionics_selection.airspeed_mass_kg,
        "rc_rx_mass_kg": avionics_selection.rc_rx_mass_kg,
        "telemetry_mass_kg": avionics_selection.telemetry_mass_kg,
        "power_module_mass_kg": avionics_selection.power_module_mass_kg,
        "servos_mass_kg": avionics_selection.servos_mass_kg,
        "esc_mass_total_kg": avionics_selection.esc_mass_total_kg,
        "misc_electronics_mass_kg": avionics_selection.misc_electronics_mass_kg,
        "avionics_electrics_total_kg": avionics_selection.total_avionics_electrics_mass_kg,
        "x_cg_m": x_cg,
    }

    if battery_selection is not None:
        batt = battery_selection["battery"]
        results.update({
            "battery_name": batt["name"],
            "battery_voltage_V": batt["voltage_V"],
            "battery_capacity_mAh": batt["capacity_mAh"],
            "battery_mass_kg": batt["mass_kg"],
            "battery_length_m": batt["length_m"],
            "battery_width_m": batt["width_m"],
            "battery_height_m": batt["height_m"],
            "battery_usable_energy_Wh": battery_selection["usable_energy_Wh"],
            "battery_endurance_min_at_cruise": battery_selection["endurance_min"],
            "battery_C_at_cruise": battery_selection["C_at_cruise"],
            "battery_meets_endurance_req": battery_selection["meets_endurance"],
        })

    # Add propulsion selection to results (if any)
    if propulsion_selection is not None:
        combo = propulsion_selection["combo"]

        n_prop = propulsion_selection["n_propellers"]
        results.update({
            "n_propellers": n_prop,
            "propulsion_name": combo["name"],
            "prop_diameter_in": combo["prop_diameter_in"],
            "prop_pitch_in": combo["prop_pitch_in"],
            "motor_kv": combo["motor_kv"],
            "motor_mass_kg": combo["motor_mass_kg"],
            "prop_mass_kg": combo["prop_mass_kg"],
            "total_motor_mass_kg": combo["motor_mass_kg"] * n_prop,
            "total_prop_mass_kg": combo["prop_mass_kg"] * n_prop,
            "total_propulsion_mass_kg": (combo["motor_mass_kg"] + combo["prop_mass_kg"]) * n_prop,
            "max_thrust_per_motor_N": combo["max_thrust_N"],
            "max_power_per_motor_W": combo["max_power_W"],
            "required_thrust_per_motor_N": propulsion_selection["required_thrust_per_motor_N"],
            "required_power_per_motor_W": propulsion_selection["required_power_per_motor_W"],
            "propulsion_meets_thrust_req": propulsion_selection["meets_thrust"],
            "propulsion_meets_power_req": propulsion_selection["meets_power"],
        })

        if "est_current_A_per_motor_at_cruise" in propulsion_selection:
            results["est_current_A_per_motor_at_cruise"] = (
                propulsion_selection["est_current_A_per_motor_at_cruise"]
            )

    return results

# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    inputs = DesignInputs()
    results = run_design(inputs)

    # Pretty-print summary
    print("=== CONCEPTUAL SUAV DESIGN SUMMARY ===")
    print(f"MTOW mass           : {results['W0_mass_kg']:.3f} kg")
    print(f"MTOW weight         : {results['W0_weight_N']:.1f} N")
    print(f"payload mass        : {inputs.payload_mass:.3f} kg")
    print()
    print("=== WING ===")
    print(f"Wing area S         : {results['wing_area_S_m2']:.3f} m²")
    print(f"Wing span b         : {results['span_b_m']:.3f} m")
    print(f"Aspect ratio AR     : {results['AR']:.3f}")
    print(f"Root chord c_root   : {results['c_root_m']:.3f} m")
    print(f"Tip chord  c_tip    : {results['c_tip_m']:.3f} m")
    print(f"MAC                 : {results['MAC_m']:.3f} m")
    print(f"MAC spanwise y_MAC  : {results['y_MAC_from_centerline_m']:.3f} m")
    print(f"MAC spanwise x_ac  : {results['x_ac_from_centerline_m']:.3f} m")
    print()
    print(f"CL_cruise           : {results['CL_cruise']:.3f}")
    print(f"CL_max              : {results['CL_max']:.3f}")
    print(f"CD0                 : {results['CD0']:.4f}")
    print(f"CD_cruise           : {results['CD_cruise']:.4f}")
    print(f"CM_cruise           : {results['CM_cruise']:.4f}")
    print(f"alpha_cruise        : {results['alpha_cruise']:.3f} deg")
    print(f"L/D at cruise       : {results['L_over_D_cruise']:.1f}")
    print(f"Stall speed V_stall : {results['V_stall']:.2f} m/s")
    print(f"Re_cruise           : {results['Re_cruise']:.2e}")
    print()
    print(f"Wing bending moment : {results['wing_bending_moment_Nm']:.3f} Nm")
    print(f"Wing torsion        : {results['wing_torsion_Nm']:.3f} Nm")
    print(f"h_m                 : {results['h_m']*1000:.3f} mm")
    print(f"t_skin_root_m       : {results['t_skin_root_m']*1000:.3f} mm")
    print(f"t_skin_tip_m        : {results['t_skin_tip_m']*1000:.3f} mm")
    print(f"W_skin_kg           : {results['W_skin_kg']:.3f} kg")
    print(f"t_web_root_m        : {results['t_web_root_m']*1000:.3f} mm")
    print(f"t_web_tip_m         : {results['t_web_tip_m']*1000:.3f} mm")
    print(f"W_web_kg            : {results['W_web_kg']:.3f} kg")
    print(f"spar_area_root_m2   : {results['spar_area_root_m2']*10**6:.3f} mm²")
    print(f"spar_area_tip_m2    : {results['spar_area_tip_m2']*10**6:.3f} mm²")
    print(f"W_spars_kg          : {results['W_spars_kg']:.3f} kg")
    print(f"N_ribs              : {results['N_ribs']}")
    print(f"W_ribs_kg           : {results['W_ribs_kg']:.3f} kg")
    print(f"W_wings_kg          : {results['W_wings_kg']:.3f} kg")
    print()
    print("=== TAILPLANE ===")
    print(f"Tail area S         : {results['tail_area_S_m2']:.3f} m²")
    print(f"Tail span b         : {results['tail_span_b_m']:.3f} m")
    print(f"Aspect ratio AR     : {results['tail_AR']:.3f}")
    print(f"Root chord c_root   : {results['tail_c_root_m']:.3f} m")
    print(f"Tip chord  c_tip    : {results['tail_c_tip_m']:.3f} m")
    print(f"MAC                 : {results['tail_MAC_m']:.3f} m")
    print(f"MAC spanwise y_MAC  : {results['tail_y_MAC_from_centerline_m']:.3f} m")
    print(f"MAC spanwise x_ac  : {results['tail_x_ac_from_centerline_m']:.3f} m")
    print()
    print(f"CL_cruise           : {results['tail_CL_cruise']:.3f}")
    print(f"CL_max              : {results['tail_CL_max']:.3f}")
    print(f"CD0                 : {results['tail_CD0']:.4f}")
    print(f"CD_cruise           : {results['tail_CD_cruise']:.4f}")
    print(f"CM_cruise           : {results['tail_CM_cruise']:.4f}")
    print(f"alpha_cruise        : {results['tail_alpha_cruise']:.3f} deg")
    print(f"L/D at cruise       : {results['tail_L_over_D_cruise']:.1f}")
    print(f"Stall speed V_stall : {results['tail_V_stall']:.2f} m/s")
    print(f"Re_cruise           : {results['tail_Re_cruise']:.2e}")
    print()
    print(f"Tail bending moment : {results['tail_bending_moment_Nm']:.3f} Nm")
    print(f"Tail torsion        : {results['tail_torsion_Nm']:.3f} Nm")
    print(f"h_m                 : {results['tail_h_m']*1000:.3f} mm")
    print(f"t_skin_root_m       : {results['tail_t_skin_root_m']*1000:.3f} mm")
    print(f"t_skin_tip_m        : {results['tail_t_skin_tip_m']*1000:.3f} mm")
    print(f"W_skin_kg           : {results['tail_W_skin_kg']:.3f} kg")
    print(f"t_web_root_m        : {results['tail_t_web_root_m']*1000:.3f} mm")
    print(f"t_web_tip_m         : {results['tail_t_web_tip_m']*1000:.3f} mm")
    print(f"W_web_kg            : {results['tail_W_web_kg']:.3f} kg")
    print(f"spar_area_root_m2   : {results['tail_spar_area_root_m2']*10**6:.3f} mm²")
    print(f"spar_area_tip_m2    : {results['tail_spar_area_tip_m2']*10**6:.3f} mm²")
    print(f"W_spars_kg          : {results['tail_W_spars_kg']:.3f} kg")
    print(f"N_ribs              : {results['tail_N_ribs']}")
    print(f"W_ribs_kg           : {results['tail_W_ribs_kg']:.3f} kg")
    print(f"W_tails_kg          : {results['tail_W_tails_kg']:.3f} kg")
    print()
    print("=== FUSELAGE ===")
    print(f"fuselage_t_shell_m  : {results['fuselage_t_shell_m']*1000:.3f} mm")
    print(f"W_fuselage_kg       : {results['W_fuselage_kg']:.3f} kg")
    print()
    print("=== PROPULSION ===")
    print(f"drag_cruise_N       : {results['drag_cruise_N']:.3f} N")
    print(f"cruise_power_W      : {results['cruise_power_W']:.3f} W")
    print(f"cruise_power_elec_W : {results['cruise_power_elec_W']:.3f} W")
    if 'n_propellers' in results:
        print(f"n_propellers        : {results['n_propellers']}")
        print(f"motor+prop combo    : {results['propulsion_name']}")
        print(f"prop_diameter       : {results['prop_diameter_in']:.1f} in")
        print(f"prop_pitch          : {results['prop_pitch_in']:.1f} in")
        print(f"motor_kv            : {results['motor_kv']:.0f} rpm/V")
        print(f"motor_mass_kg       : {results['motor_mass_kg']:.3f} kg")
        print(f"prop_mass_kg        : {results['prop_mass_kg']:.3f} kg")
        print(f"total_motor_mass    : {results['total_motor_mass_kg']:.3f} kg")
        print(f"total_prop_mass     : {results['total_prop_mass_kg']:.3f} kg")
        print(f"total_propulsion_mass: {results['total_propulsion_mass_kg']:.3f} kg")
        print(f"max_thrust_per_motor: {results['max_thrust_per_motor_N']:.2f} N")
        print(f"req_thrust_per_motor: {results['required_thrust_per_motor_N']:.2f} N")
        print(f"max_power_per_motor : {results['max_power_per_motor_W']:.1f} W")
        print(f"req_power_per_motor : {results['required_power_per_motor_W']:.1f} W")
        if 'est_current_A_per_motor_at_cruise' in results:
            print(f"I_cruise_per_motor  : {results['est_current_A_per_motor_at_cruise']:.2f} A")
        print(f"meets thrust req?   : {results['propulsion_meets_thrust_req']}")
        print(f"meets power req?    : {results['propulsion_meets_power_req']}")
    print()
    print("=== BATTERY SELECTION ===")
    print(f"battery_name        : {results['battery_name']}")
    print(f"battery_voltage_V   : {results['battery_voltage_V']:.1f} V")
    print(f"battery_capacity    : {results['battery_capacity_mAh']:.0f} mAh")
    print(f"battery_mass_kg     : {results['battery_mass_kg']:.3f} kg")
    print(f"battery_endurance   : {results['battery_endurance_min_at_cruise']:.1f} min")
    print(f"battery_C_at_cruise : {results['battery_C_at_cruise']:.2f} C")
    print(f"battery_length_m    : {results['battery_length_m']:.3f} m")
    print(f"battery_width_m     : {results['battery_width_m']:.3f} m")
    print(f"battery_height_m    : {results['battery_height_m']:.3f} m")
    print(f"meets endurance req?: {results['battery_meets_endurance_req']}")
    print()
    print("=== AVIONICS / ELECTRONICS ===")
    print(f"Autopilot          : {results['autopilot_name']}")
    print(f"m_autopilot_kg     : {results['autopilot_mass_kg']:.3f} kg")
    print(f"m_GPS_kg           : {results['gps_mass_kg']:.3f} kg")
    print(f"m_airspeed_kg      : {results['airspeed_mass_kg']:.3f} kg")
    print(f"m_RC_rx_kg         : {results['rc_rx_mass_kg']:.3f} kg")
    print(f"m_telemetry_kg     : {results['telemetry_mass_kg']:.3f} kg")
    print(f"m_power_module_kg  : {results['power_module_mass_kg']:.3f} kg")
    print(f"m_servos_kg        : {results['servos_mass_kg']:.3f} kg")
    print(f"m_ESCs_total_kg    : {results['esc_mass_total_kg']:.3f} kg")
    print(f"m_misc_elec_kg     : {results['misc_electronics_mass_kg']:.3f} kg")
    print(f"TOTAL avionics+elec: {results['avionics_electrics_total_kg']:.3f} kg")
    print()
    print(f"fuselage_length    : {inputs.fuselage_length:.3f} m")
    print(f"x_cg_m             : {results['x_cg_m']:.3f} m")
    print(f"x_cg_m/fuselage_length: {results['x_cg_m']/inputs.fuselage_length:.3f}")