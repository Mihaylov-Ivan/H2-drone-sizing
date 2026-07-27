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

    # --- Speeds & load factors ---
    V_cruise: float = 20.0         # [m/s] cruise speed
    # V_loiter: float = 12.0         # [m/s] loiter (not strictly used below)
    n_max_turn: float = 2.5        # [-] max load factor in 45° banked turn

    # --- Wing loading & power loading (design point) ---
    # W/S chosen from constraint diagram (approx SAKR-2 scale) :contentReference[oaicite:9]{index=9}
    wing_loading: float = 65.0     # [N/m²] design W/S
    # Installed power-to-weight ratio (shaft power / weight)
    # 150 W/kg ≈ (150 / (g)) W/N ≈ 15.3 W/N
    power_to_weight_installed: float = 150.0  # [W/kg] installed motor rating

    # --- Wing geometry choices (from paper / historical data) :contentReference[oaicite:10]{index=10}
    AR: float = 3.0                # [-] aspect ratio
    taper_ratio: float = 0.5       # λ = c_tip / c_root
    sweep_LE_deg: float = 27.0     # [deg] leading-edge sweep (informational)
    CL_max: float = 1.2            # [-] max 3D CL (no flaps)
    CD0: float = 0.035             # [-] parasite drag coeff
    e_oswald: float = 0.8          # [-] Oswald efficiency factor

    # --- Stability target (static margin) :contentReference[oaicite:11]{index=11}
    static_margin: float = 0.20    # 20% of MAC (fairly stable flying wing)

    # --- Propulsion & battery ---
    prop_eff: float = 0.8          # propeller efficiency η_prop
    elec_eff: float = 0.9          # electrical efficiency (ESC + wiring)
    battery_voltage: float = 14.8  # [V] 4S LiPo nominal (Thunder Power 4S4600) :contentReference[oaicite:12]{index=12}
    battery_capacity_Ah: float = 4.6  # [Ah] capacity
    # Safety factor on required cruise power when picking motor rating
    motor_power_margin: float = 1.3


# ============================================================
# 2. HIGH-LEVEL SIZING FUNCTIONS
# ============================================================

def estimate_mtow(inputs: DesignInputs) -> float:
    """
    Step 1: Estimate MTOW (W0_mass) from payload mass and historical payload fraction.

    Eq. (7) style:
        W0_mass = W_payload_mass / (W_payload / W0)_hist

    Returns:
        W0_mass [kg]
    """
    return inputs.payload_mass / inputs.payload_fraction_hist

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

    return b, c_root, c_tip, MAC, y_MAC


def aerodynamic_coeffs(W0_mass: float, S: float, MAC: float, inputs: DesignInputs):
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
    W = W0_mass * g
    V = inputs.V_cruise

    q = 0.5 * rho * V**2
    CL = W / (q * S)
    CDi = CL**2 / (math.pi * inputs.AR * inputs.e_oswald)
    CD = inputs.CD0 + CDi
    L_over_D = CL / CD

    V_stall = math.sqrt(2.0 * W / (rho * S * inputs.CL_max))
    Re = rho * V * MAC / mu

    return {
        "CL_cruise": CL,
        "CDi_cruise": CDi,
        "CD_cruise": CD,
        "L_over_D_cruise": L_over_D,
        "V_stall": V_stall,
        "Re_cruise": Re,
    }


def stability_positions(MAC: float, inputs: DesignInputs):
    """
    Step 5: Simple 1D longitudinal stability layout on MAC.

    For a tailless wing, the aerodynamic center (AC) is ~25% MAC. :contentReference[oaicite:13]{index=13}
    We place CG ahead of AC by 'static_margin' * MAC:

        x_ac  = 0.25 * MAC
        x_cg  = x_ac - SM * MAC

    Return:
        x_ac_from_LE, x_cg_from_LE  [m]
    """
    x_ac = 0.25 * MAC
    x_cg = x_ac - inputs.static_margin * MAC
    return x_ac, x_cg


def propulsion_sizing(W0_mass: float, S: float, aero: dict, inputs: DesignInputs):
    """
    Step 6: Required and installed power.

    Option A: use installed power loading (P_installed / m0) as in design point:
        P_installed = (P/W)_installed [W/kg] * m0 [kg]

    Option B (here combined): compute *required* cruise shaft power from drag
    and check it against installed power.

    Level-flight drag at cruise:
        D = q * S * CD

    Required shaft power (propeller):
        P_req_cruise = D * V / (η_prop * η_elec)

    Return:
        P_req_cruise, P_installed, power_margins
    """
    V = inputs.V_cruise
    q = 0.5 * rho * V**2
    CD = aero["CD_cruise"]

    # Required power for level cruise [W]
    D = q * S * CD                     # [N]
    P_req_cruise = D * V / (inputs.prop_eff * inputs.elec_eff)

    # Installed motor rating from power-to-weight choice [W]
    P_installed_nom = inputs.power_to_weight_installed * W0_mass  # W/kg * kg = W

    # Add design margin (e.g. 1.3x)
    P_installed = inputs.motor_power_margin * P_installed_nom

    power_margin = P_installed / P_req_cruise

    return {
        "P_req_cruise_W": P_req_cruise,
        "P_installed_W": P_installed,
        "P_installed_nom_W": P_installed_nom,
        "power_margin_factor": power_margin,
    }


def battery_endurance(power: dict, inputs: DesignInputs):
    """
    Step 7: Battery endurance at cruise.

    Battery energy (ideal):
        E_bat = U * C_Ah * 3600  [J]

    Usable energy (account for depth of discharge & efficiencies):
        E_usable = E_bat * η_elec  (we've already applied η_elec in P_req_cruise,
                                   so here we just keep it simple and use 0.9 overall)

    Endurance:
        t = E_usable / P_req_cruise   [s]
    """
    U = inputs.battery_voltage
    C_Ah = inputs.battery_capacity_Ah

    E_bat = U * C_Ah * 3600.0  # [J]
    # Assume ~90% usable (DOD and misc losses)
    E_usable = 0.9 * E_bat

    P_req = power["P_req_cruise_W"]
    endurance_s = E_usable / P_req
    endurance_min = endurance_s / 60.0

    return {
        "E_bat_J": E_bat,
        "E_usable_J": E_usable,
        "endurance_s": endurance_s,
        "endurance_min": endurance_min,
    }


# ============================================================
# 3. MASTER DRIVER
# ============================================================

def run_design(inputs: DesignInputs):
    # Step 1: MTOW from payload fraction
    W0_mass_guess = estimate_mtow(inputs)
    # W0_N = W0_mass * g

    for i in range(max_iterations):
        W0_mass = W0_mass_guess
        W0_N = W0_mass * g



        # Step 2: Wing area from wing loading
        S = W0_N / inputs.wing_loading

        # Step 3: Wing geometry
        b, c_root, c_tip, MAC, y_MAC = wing_geometry(S, inputs)

        # Step 4: Aerodynamics at cruise & stall
        aero = aerodynamic_coeffs(W0_mass, S, MAC, inputs)

        # Step 5: Stability (AC & CG along MAC)
        x_ac, x_cg = stability_positions(MAC, inputs)

        # Step 6: Propulsion sizing
        power = propulsion_sizing(W0_mass, S, aero, inputs)

        # Step 7: Battery endurance estimate
        endurance = battery_endurance(power, inputs)

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
        "x_ac_from_LE_m": x_ac,
        "x_cg_from_LE_m": x_cg,
        "aero": aero,
        "power": power,
        "endurance": endurance,
    }

    return results


# ============================================================
# 4. EXAMPLE RUN (SAKR-2-LIKE BASELINE)
# ============================================================

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
    print(f"CL_cruise           : {results['aero']['CL_cruise']:.3f}")
    print(f"CD_cruise           : {results['aero']['CD_cruise']:.4f}")
    print(f"L/D at cruise       : {results['aero']['L_over_D_cruise']:.1f}")
    print(f"Stall speed V_stall : {results['aero']['V_stall']:.2f} m/s")
    print(f"Re_cruise           : {results['aero']['Re_cruise']:.2e}")
    print()
    print(f"x_ac_from_LE_m     : {results['x_ac_from_LE_m']:.3f} m")
    print(f"x_cg_from_LE_m     : {results['x_cg_from_LE_m']:.3f} m")
    print()
    print(f"Required P_cruise   : {results['power']['P_req_cruise_W']:.1f} W")
    print(f"Installed P_nominal : {results['power']['P_installed_nom_W']:.1f} W")
    print(f"Installed P (with margin) : {results['power']['P_installed_W']:.1f} W")
    print(f"Power margin (inst/req)   : {results['power']['power_margin_factor']:.2f} x")
    print()
    print(f"Battery energy (ideal)    : {results['endurance']['E_bat_J'] / 3600:.1f} Wh")
    print(f"Endurance (cruise est.)   : {results['endurance']['endurance_min']:.1f} min")
