import math

# ============================================================
# 0. CONSTANTS
# ============================================================

g = 9.81  # gravitational acceleration [m/s²]
rho = 1.112             # air density at operating altitude ρ [kg/m³]


# ============================================================
# 0A. GENERIC HELPERS
# ============================================================

def dyn_pressure(rho, V):
    """Dynamic pressure q = 0.5 * rho * V^2 [Pa]."""
    return 0.5 * rho * V**2


# ============================================================
# 1. GEOMETRY & AERODYNAMICS
#   (Base reference + X-wing specific geometry)
# ============================================================

# --- Global reference geometry (for forces, drag, endurance) ---

S = ...                 # reference area S_ref [m²] (vertical-lift reference)
b = ...                 # reference span b [m] (used in induced power, etc.)
A = b**2 / S            # aspect ratio based on S_ref and b [-]

e_oswald = ...          # Oswald efficiency factor e [-], ~0.6–0.8
C_D0 = ...              # zero-lift drag coefficient C_D0 [-]

# ------------------------------------------------------------
# 1A. X-WING GEOMETRY & SIZING (slender-wing theory)
# ------------------------------------------------------------

# Basic X-wing parameters
n_wings = 4                   # number of wings in the X-wing
delta_deg = 45.0              # roll angle of each wing from "cruciform" [deg]
delta = math.radians(delta_deg)

# Per-wing geometry (physical, not projected)
S_w = ...                     # area of ONE physical wing [m²]
b_w = ...                     # physical semispan of ONE wing (root to tip) [m]
A_w = b_w**2 / S_w            # aspect ratio of one slender wing [-]

# Optional: if you want S (reference area) to be derived from 4 wings at angle delta:
# S = 4.0 * S_w * math.cos(delta)

# --- Slender wing lift-curve slope (Jones / Spreiter) ---

def CL_alpha_slender(A_w):
    """
    Lift-curve slope dC_L/dα for a single slender low-AR wing [1/rad]:
    C_Lα ≈ (π/2) * A_w
    """
    return 0.5 * math.pi * A_w


def CL_alpha_xwing(A_w, delta_rad):
    """
    Effective vertical lift-curve slope for an X-wing whose wings are rolled
    by delta about the body axis. Vertical component is reduced by cos(delta).
    """
    return CL_alpha_slender(A_w) * math.cos(delta_rad)


def CL_xwing(alpha_rad, A_w, delta_rad):
    """
    Vertical lift coefficient C_L(α) for the X-wing (small α, radians).
    """
    return CL_alpha_xwing(A_w, delta_rad) * alpha_rad


# Use the X-wing slope as your dC_L/dα for gusts, etc.
dCL_dalpha = CL_alpha_xwing(A_w, delta)

# ============================================================
# 1B. LIFT & DRAG (including induced drag)
# ============================================================

# Effective aspect ratio for induced drag:
# - If only 2 "horizontal" wings lift: A_eff ≈ b_w^2 / (2 * S_w)
# - If all 4 wings at 45° project into lift direction: A_eff ≈ b_w^2 * cos(delta) / S_w
# A_eff = b_w**2 * math.cos(delta) / S_w   # choose this for full X-wing contribution
A_eff = b_w**2 / S_w   # choose this for only 2 horizontal wings contribution

C_L = ...    # current lift coefficient C_L [-] at the condition of interest

def induced_drag_coeff(C_L, A_eff, e):
    """C_Di = C_L^2 / (π A_eff e)."""
    return C_L**2 / (math.pi * A_eff * e)


# Total drag coefficient using 2-term polar (induced + parasite)
C_Di = induced_drag_coeff(C_L, A_eff, e_oswald)
C_D = C_D0 + C_Di


# ============================================================
# 1C. WING SIZING EQUATIONS
# ============================================================

def S_eff_from_stall(W, rho, Vs, CL_max):
    """
    Effective lifting area required from stall constraint:
    S_eff = 2W / (rho * Vs^2 * C_L_max)
    """
    return 2.0 * W / (rho * Vs**2 * CL_max)


def S_w_from_S_eff(S_eff, use_all_four=True, delta_rad=0.0):
    """
    Convert effective area S_eff to per-wing area S_w.

    - If only 2 "horizontal" wings counted: S_w = S_eff / 2
    - If all 4 wings counted with projection cos(delta):
        S_w = S_eff / (4 * cos(delta))
    """
    if use_all_four:
        return S_eff / (4.0 * math.cos(delta_rad))
    else:
        return S_eff / 2.0


def S_ref_from_cruise(W, rho, V_c, C_L_c):
    """
    Reference area from cruise condition:
    S_ref = W / (q_c * C_L_c), q_c = 0.5 * rho * V_c^2
    """
    q_c = dyn_pressure(rho, V_c)
    return W / (q_c * C_L_c)


def CL_maneuver(n_max, W, rho, V_man, S_ref):
    """
    Required C_L at maneuver condition:
    C_L,man = n_max * W / (q_man * S_ref)
    """
    q_man = dyn_pressure(rho, V_man)
    return n_max * W / (q_man * S_ref)


# ============================================================
# 1D. PARASITE DRAG (skin friction + form factor)
# ============================================================

def Cf_turbulent(Re_L):
    """Flat-plate turbulent skin-friction coefficient."""
    return 0.074 / (Re_L ** 0.2)


def Cf_laminar(Re_L):
    """Flat-plate laminar skin-friction coefficient."""
    return 1.328 / math.sqrt(Re_L)


def CD0_component(Cf, FF, S_wet_i, S_ref):
    """Component-level parasite drag contribution."""
    return Cf * FF * (S_wet_i / S_ref)


def CD0_total(components, S_ref):
    """
    Sum parasite drag from components.

    components: list of dicts with keys {"Cf", "FF", "S_wet"}
    """
    CD0_sum = 0.0
    for comp in components:
        CD0_sum += CD0_component(
            comp["Cf"], comp["FF"], comp["S_wet"], S_ref
        )
    return CD0_sum


# ============================================================
# 1E. MOMENTS, CENTER OF PRESSURE & STABILITY
# ============================================================

# Triangular slender wing center of pressure ~ 2/3 of mean aerodynamic chord
cp_triangular = 2.0 / 3.0

c_bar = ...           # mean aerodynamic chord of reference wing set [m]
x_ac = ...            # aerodynamic center position (from reference) [m]
x_cg = ...            # center of gravity position [m]

def static_margin(x_ac, x_cg, c_bar):
    """
    Static margin SM = (x_ac - x_cg) / c_bar.
    Positive SM -> statically stable.
    """
    return (x_ac - x_cg) / c_bar


def Cm_from_CL(CL, x_ac, x_cg, c_bar):
    """
    Pitching moment coefficient about CG using linearized relation:
    C_m ≈ C_L * (x_ac - x_cg) / c_bar
    """
    return CL * (x_ac - x_cg) / c_bar


SM = static_margin(x_ac, x_cg, c_bar)
C_m = Cm_from_CL(C_L, x_ac, x_cg, c_bar)


# ============================================================
# 2. WEIGHTS & LOADINGS
# ============================================================

m = ...                   # aircraft mass [kg]
W = m * g                 # weight W [N]
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
    term_induced = (
        2.0 * W**2 * math.cos(gamma)**2
        / (math.pi * b**2 * e_oswald * rho)
        * (1.0 / V)
    )

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
