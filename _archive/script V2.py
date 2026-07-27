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
from airfoil_naca0006 import interpolate_by_CL

# ============================================================
# 0. CONSTANTS
# ============================================================

g = 9.81             # gravitational acceleration [m/s²]
rho = 1.2            # air density at ~150 m altitude [kg/m³]
mu = 1.8e-5          # dynamic viscosity of air [kg/(m·s)] ~ ISA sea-level

max_iterations = 100 # max number of iterations to find the MTOW

# ============================================================
# 1. DESIGN INPUTS (tuned to SAKR-2 style SUAV) :contentReference[oaicite:7]{index=7}
# ============================================================

@dataclass
class DesignInputs:
    # --- Mission / payload ---
    payload_mass: float = 5      # [kg] 
    # Historical payload weight fraction from Table 5 (~0.33) :contentReference[oaicite:8]{index=8}
    payload_fraction_hist: float = 0.33  # W_payload / W0

    n_ult: float = 2.5        # [-] max load factor in 45° banked turn

    airfoil_type: str = "NACA0006"

    AR: float = 1.0                # [-] aspect ratio
    taper_ratio: float = 0.5       # λ = c_tip / c_root
    wing_flap_span_ratio: float = 5/10       # [-] ratio of wing flap span to wing span
    # CM: float = 0.0054               # [-] pitching moment coefficient
    # CD: float = 0.02247               # [-] total drag coeff
    # e_oswald: float = 0.8          # [-] Oswald efficiency factor
    
    wing_loading: float = 100.0     # [N/m²] design W/S
    # CL_max: float = 0.4405            # [-] max 3D CL (no flaps)
    # CD0: float = 0.035             # [-] parasite drag coeff

    speed: float = 30.0         # [m/s] cruise speed
    dive_speed: float = speed*1.5     # [m/s] dive speed

    fuselage_length: float = 1.2         # [m] length of fuselage
    fuselage_OD: float = 0.25         # [m] outer diameter of fuselage
    fuselage_OR: float = fuselage_OD/2         # [m] outer radius of fuselage
    fuselage_cs_area: float = math.pi*fuselage_OR**2         # [m²] cross-sectional area of fuselage
    fuselage_cs_perimeter: float = 2*math.pi*fuselage_OR         # [m] cross-sectional perimeter of fuselage
    fuselage_shell_area_moment_of_inertia: float = math.pi*fuselage_OR**3         # [m4] area moment of inertia of fuselage shell per unit thickness

    printed_parts_sf: float = 4.0        # [-] safety factor for 3D printed parts
    filament_density: float = 1220.0         # [kg/m³] density of filament
    filament_tensile_strength_MPa: float = 55.0         # [MPa] strength of filament
    filament_max_bending_stress_MPa: float = 77.0         # [MPa] bending strength of filament
    filament_shear_strength_MPa: float = filament_tensile_strength_MPa/math.sqrt(3)         # [MPa] shear strength of filament

    carbon_fiber_density: float = 1800.0         # [kg/m³] density of carbon fiber
    carbon_fiber_max_bending_stress_MPa: float = 1500.0         # [MPa] bending strength of carbon fiber

# ============================================================
# 2. SIZING FUNCTIONS
# ============================================================

def wing_geometry(S: float, inputs: DesignInputs):
    """
    Step 3: Wing span, chords, MAC.

    Aspect ratio:
        AR = b² / S  -> b = sqrt(AR * S)

    Root and tip chord with taper ratio λ:
        c_root = 2*S / [b*(1 + λ)]
        c_tip  = λ * c_root

    Mean aerodynamic chord (standard trapezoidal wing formula):
        MAC = 2/3 * c_root * (1 + λ + λ²) / (1 + λ)

    Spanwise position of MAC from centerline (for reference):
        y_MAC = b/6 * (1 + 2λ)/(1 + λ)

    Returns:
        b, c_root, c_tip, MAC, y_MAC
    """
    AR = inputs.AR
    lam = inputs.taper_ratio

    b = math.sqrt(AR * S)
    c_root = 2.0 * S / (b * (1.0 + lam))
    c_tip = lam * c_root
    MAC = (2.0 / 3.0) * c_root * (1.0 + lam + lam**2) / (1.0 + lam)
    y_MAC = (b / 6.0) * (1.0 + 2.0 * lam) / (1.0 + lam)
    c_MGC = S/b

    return b, c_root, c_tip, MAC, y_MAC, c_MGC


def aerodynamic_coeffs(W0_N: float, S: float, MAC: float, inputs: DesignInputs):
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

    airfoil_data = interpolate_by_CL(CL)

    # CDi = CL**2 / (math.pi * inputs.AR * inputs.e_oswald)
    # CD = inputs.CD0 + CDi
    L_over_D = CL / airfoil_data['CD']

    V_stall = math.sqrt(2.0 * W / (rho * S * airfoil_data['CL_max']))
    Re = rho * V * MAC / mu

    return {
        "CL_cruise": CL,
        "CL_max": airfoil_data['CL_max'],
        "CD_cruise": airfoil_data['CD'],
        "CM_cruise": airfoil_data['CM'],
        "CM_max": airfoil_data['CM_max'],
        "alpha_cruise": airfoil_data['alpha'],
        "L_over_D_cruise": L_over_D,
        "V_stall": V_stall,
        "Re_cruise": Re,
        "max_thickness_fraction_of_chord": airfoil_data['max_thickness_fraction_of_chord'],
    }

# ============================================================
# 3. MASTER DRIVER
# ============================================================

def run_design(inputs: DesignInputs):
    # Step 1: MTOW from payload fraction
    W0_mass = inputs.payload_mass / inputs.payload_fraction_hist

    for i in range(max_iterations):
        W0_N = W0_mass * g

        # Step 2: Wing estimations

        S = W0_N / inputs.wing_loading
        b, c_root, c_tip, MAC, y_MAC, c_MGC = wing_geometry(S, inputs)

        aero = aerodynamic_coeffs(W0_N, S, MAC, inputs)

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

        # Optional non-optimum factor to approximate total fuselage (structure + fittings)
        fuselage_nonopt_factor = 1.0  # set to ~2.57 for big transports if desired

        W_fuselage_total = fuselage_nonopt_factor * W_fuselage_shell  # [kg]


    # Collect results
    results = {
        "inputs": asdict(inputs),
        "W0_mass_kg": W0_mass,
        "W0_weight_N": W0_N,
        "wing_area_S_m2": S,
        "span_b_m": b,
        "c_root_m": c_root,
        "c_tip_m": c_tip,
        "MAC_m": MAC,
        "y_MAC_from_centerline_m": y_MAC,
        "CL_cruise": aero['CL_cruise'],
        "CL_max": aero['CL_max'],
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
        'fuselage_t_shell_m': fuselage_t_shell,
        'W_fuselage_kg': W_fuselage_total,
    }

    return results

if __name__ == "__main__":
    inputs = DesignInputs()
    results = run_design(inputs)

    # Pretty-print summary
    print("=== CONCEPTUAL SUAV DESIGN SUMMARY ===")
    print(f"MTOW mass           : {results['W0_mass_kg']:.3f} kg")
    print(f"MTOW weight         : {results['W0_weight_N']:.1f} N")
    print()
    print(f"Wing area S         : {results['wing_area_S_m2']:.3f} m²")
    print(f"Wing span b         : {results['span_b_m']:.3f} m")
    print(f"Root chord c_root   : {results['c_root_m']:.3f} m")
    print(f"Tip chord  c_tip    : {results['c_tip_m']:.3f} m")
    print(f"MAC                 : {results['MAC_m']:.3f} m")
    print(f"MAC spanwise y_MAC  : {results['y_MAC_from_centerline_m']:.3f} m")
    print()
    print(f"CL_cruise           : {results['CL_cruise']:.3f}")
    print(f"CL_max              : {results['CL_max']:.3f}")
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
    print(f"fuselage_t_shell_m  : {results['fuselage_t_shell_m']*1000:.3f} mm")
    print(f"W_fuselage_kg       : {results['W_fuselage_kg']:.3f} kg")