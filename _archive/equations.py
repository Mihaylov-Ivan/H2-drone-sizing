import math

# ============================================================
# 0. CONSTANTS
# ============================================================

g = 9.81  # gravitational acceleration [m/s²]

# ============================================================
# 1. GEOMETRY & AERODYNAMICS
#   (Sections 2.1, 2.3 and Appendix A.1–A.3)
# ============================================================

rho = 1.112             # air density at operating altitude ρ [kg/m³]
S = ...               # wing planform area S [m²]
b = ...               # wingspan b [m]
A = b**2 / S          # aspect ratio A = b²/S [-]
e_oswald = ...        # Oswald efficiency factor e [-]

C_D0 = ...            # zero-lift drag coefficient C_D0 [-]
C_L = ...             # current lift coefficient C_L [-]
dCL_dalpha = ...      # lift-curve slope dC_L/dα [1/rad]

# Total drag coefficient using 2-term polar (Eq. A.13)
# C_D = C_D0 + C_L² / (π A e)
C_D = C_D0 + C_L**2 / (math.pi * A * e_oswald)

# ============================================================
# 2. WEIGHTS & LOADINGS
# ============================================================

W = ...                   # aircraft weight W (≈ MTOM * g) [N]
wing_loading = W / S      # wing loading W/S [N/m²]

# ============================================================
# 3. PROPULSION & ENERGY
#   (Endurance & terminal dive, Eqs. (2), (3), A.7, A.8)
# ============================================================

eta_prop = ...        # propeller efficiency η_prop at current V [-]
eta_prop_Vmax = ...   # propeller efficiency at V_max [-]
eta_elec = ...        # electrical system efficiency η_elec [-]

P_br = ...            # shaft (brake) power P_br [W]
P_br_max = ...        # max shaft power P_br,max [W]

U_bat = ...           # usable battery energy U_bat [J]
dUbat_dt = ...        # time derivative dU_bat/dt [J/s] (negative during discharge)

# Power available from shaft power (Eq. A.7)
P_a = eta_prop * P_br  # P_a = η_prop * P_br

# Brake power from battery discharge (Eq. A.8)
P_br_from_battery = -eta_elec * dUbat_dt  # P_br = -η_elec dU_bat/dt


# ============================================================
# 4. LOITERING ENDURANCE – ELECTRIC (Eq. (2) / A.18)
#   Ebattery for constant-altitude, constant-α loiter
# ============================================================

def battery_endurance(W, wing_loading, rho, A, e_oswald, C_D0,
                      eta_prop, eta_elec, U_bat):
    """
    Loitering endurance for a battery-electric LM (Eq. (2) / A.18).

    All inputs in SI units, wing_loading in N/m².
    Returns endurance E_battery in seconds.
    """

    factor = math.sqrt(
        (27.0 / 512.0)
        * (1.0 / wing_loading)
        * (1.0 / W**2)
        * (rho * math.pi * A * e_oswald) ** 3
        / C_D0
    )

    E_battery = eta_elec * eta_prop * factor * U_bat
    return E_battery


# ============================================================
# 5. TERMINAL ATTACK DIVE MAX SPEED (Eq. (3) / A.24)
#   Solve residual = 0 for V_max
# ============================================================

def terminal_dive_residual(V, W, rho, S, b, e_oswald, C_D0,
                           P_br_max, eta_prop_V, gamma):
    """
    Residual of Eq. (3): should be ≈ 0 at the terminal dive V_max.

    V      : candidate airspeed [m/s]
    gamma  : (negative) descent flight-path angle γ [rad]
    """

    # power available from engine-propeller combination
    term_power_avail = P_br_max * eta_prop_V

    # zero-lift drag power: C_D0 * 0.5 * ρ * V³ * S
    term_zero_lift = C_D0 * 0.5 * rho * V**3 * S

    # induced drag power: 2 W² cos²γ / (π b² e ρ) * 1/V
    term_induced = 2.0 * W**2 * math.cos(gamma)**2 / (math.pi * b**2 * e_oswald * rho) * (1.0 / V)

    # rate of change of potential energy: W V sinγ
    term_potential = W * V * math.sin(gamma)

    # Eq. (3): P_br,max η_prop(V) - zero-lift - induced + potential = 0
    return term_power_avail - term_zero_lift - term_induced + term_potential


# ============================================================
# 6. GUST RESPONSE (Eq. (4), A.25–A.27)
# ============================================================

U_gust = ...          # vertical gust velocity U [m/s]
V = ...               # current airspeed V [m/s]

# Exact gust-induced angle-of-attack change (Eq. A.26)
delta_alpha_exact = math.atan(U_gust / V)  # Δα [rad]

# Gust-induced lift increment (Eq. A.25)
delta_L = dCL_dalpha * delta_alpha_exact * 0.5 * rho * (V**2 + U_gust**2) * S  # [N]

# Load-factor increment from ΔL
delta_n_from_deltaL = delta_L / W  # Δn [-]

# Compact analytic gust-load factor (Eq. (4) / A.27) with K≈1
K_gust = 1.0
delta_n = (
    K_gust * dCL_dalpha * rho * U_gust * V / wing_loading
)


# ============================================================
# 7. TURNING / CLIMBING FLIGHT (Appendix A.4)
# ============================================================

T = ...               # thrust magnitude T [N]
alpha_T = ...         # thrust angle of attack α_T [rad]
L = ...               # lift force L [N]
gamma = ...           # flight-path angle γ [rad]
mu = ...              # bank angle μ [rad]
R = ...               # turn radius R [m]
chi_dot = ...         # heading rate χ̇ [rad/s]

# Kinematics (Eq. A.31 and A.32)
V_from_turn_rate = chi_dot * R   # V = χ̇ R
h_dot = V * math.sin(gamma)      # climb/descent rate \dot{h} = V sinγ [m/s]

def normal_force_balance_residual(T, alpha_T, L, W, gamma, mu, V, R, g=g):
    """
    Normal-force equilibrium in a coordinated turn (vertical plane component),
    based on the equation shown in your image
    (derived from Appendix A.4 combined with Eq. (A.29)).

    Returns residual that should be ≈0 when forces balance.
    """
    # -T sinα_T - L + W cosγ cosμ - (W/g) * V²/R * cos²γ * sinμ = 0
    term1 = -T * math.sin(alpha_T)
    term2 = -L
    term3 = W * math.cos(gamma) * math.cos(mu)
    term4 = -(W / g) * (V**2 / R) * (math.cos(gamma) ** 2) * math.sin(mu)

    return term1 + term2 + term3 + term4
