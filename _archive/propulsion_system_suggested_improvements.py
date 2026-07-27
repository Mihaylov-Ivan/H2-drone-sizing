"""
propulsion_system.py

Propulsion system selection and sizing (cruise + tailsitter VTOL mission envelope).

NEW mission profile rules (as requested):
- ONLY mark a propulsion combo infeasible if it cannot:
    (1) Hover (for the commanded hover time)
    (2) Climb to the required VTOL climb altitude (time is estimated, NOT constrained)
    (3) Transition (Stage 3) using differential thrust for rotation, with:
          - NO fixed transition time (time is estimated from available power/thrust)
          - per-motor peak electrical power limited to combo max_power_W
          - rotation torque checked (combo invalid if torque cannot be achieved)
          - drag included
          - NO wing lift included during transition
          - altitude loss estimated; combo invalid if it loses more altitude than available climb height
    (4) Accelerate from handoff speed to cruise speed using available power (time estimated, NOT constrained)

- No extra stage constraints that artificially inflate required peak thrust/power.
"""

from __future__ import annotations

from typing import Optional, Dict, Any, Tuple, List
import math

from design_inputs import DesignInputs
from constants import ATMOSPHERIC
from batteries import get_battery_database
from propulsion import get_propulsion_database
from helper_function import select_battery, select_propulsion
from avionics import estimate_avionics_and_electrics
from results import BatteryResults, PropulsionResults, AvionicsResults


# =========================
# Small utilities
# =========================

def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _disk_area_one(prop_diameter_in: float) -> float:
    d_m = float(prop_diameter_in) * 0.0254
    return math.pi * (0.5 * d_m) ** 2


def _disk_area_total(prop_diameter_in: float, n_prop: int) -> float:
    return float(n_prop) * _disk_area_one(prop_diameter_in)


# =========================
# Momentum theory (simple)
# =========================

def _power_hover_induced_W(T_N: float, rho: float, A_total: float) -> float:
    # Ideal induced power in hover (actuator disk)
    T = max(float(T_N), 0.0)
    A = max(float(A_total), 1e-12)
    return (T ** 1.5) / math.sqrt(2.0 * rho * A)


def _power_axial_climb_W(T_N: float, rho: float, A_total: float, Vc_mps: float) -> float:
    # Ideal actuator disk power in axial climb: P = T*(0.5Vc + sqrt((0.5Vc)^2 + v_h^2))
    T = max(float(T_N), 0.0)
    A = max(float(A_total), 1e-12)
    Vc = max(float(Vc_mps), 0.0)
    v_h = math.sqrt(max(T, 1e-12) / (2.0 * rho * A))
    return T * (0.5 * Vc + math.sqrt((0.5 * Vc) ** 2 + v_h ** 2))


# =========================
# Drag models
# =========================

def _drag_parasite_only_N(V_mps: float, S_m2: float, CD0: float) -> float:
    # During transition we assume wing lift ineffective; use parasite-only drag
    V = float(V_mps)
    q = 0.5 * ATMOSPHERIC.rho * V * V
    if q <= 1e-12 or S_m2 <= 1e-12:
        return 0.0
    return q * float(S_m2) * max(0.0, float(CD0))


def _drag_parabolic_N(
    L_ref_N: float,
    V_mps: float,
    S_m2: float,
    AR: float,
    CD0: float,
    e_oswald: float,
    CL_cap: float = 1.6,
) -> float:
    """
    D = q S (CD0 + k CL^2), with CL from L_ref (typically weight).
    Used for post-handoff acceleration where wing is assumed effective.
    """
    V = float(V_mps)
    q = 0.5 * ATMOSPHERIC.rho * V * V
    if q <= 1e-12 or S_m2 <= 1e-12:
        return 0.0

    CL = float(L_ref_N) / (q * float(S_m2))
    CL = _clamp(CL, -CL_cap, CL_cap)

    k = 1.0 / (math.pi * max(float(e_oswald), 1e-12) * max(float(AR), 1e-12))
    CD = max(0.0, float(CD0) + k * CL * CL)
    return q * float(S_m2) * CD


# =========================
# VTOL/transition per-motor power model (conservative)
# =========================

def _motor_power_transition_elec_W(
    *,
    T_motor_N: float,
    theta_rad: float,
    V_mps: float,
    rho: float,
    A_one: float,
    eta_vtol: float,
) -> float:
    """
    Conservative: induced (hover-like) + useful (T_forward*V), divided by eta_vtol.
    No attempt to model advance ratio / prop efficiency variation in transition.
    """
    Tm = max(0.0, float(T_motor_N))
    A = max(float(A_one), 1e-12)

    P_ind = (Tm ** 1.5) / math.sqrt(2.0 * rho * A)
    P_useful = max(0.0, (Tm * math.sin(theta_rad)) * max(0.0, float(V_mps)))
    P_elec = (P_ind + P_useful) / max(float(eta_vtol), 1e-12)
    return P_elec


def _solve_T_for_power_limit(
    *,
    Pmax_elec_W: float,
    theta_rad: float,
    V_mps: float,
    rho: float,
    A_one: float,
    eta_vtol: float,
    Tmax_N: float,
) -> float:
    """
    Find max T_motor such that motor power <= Pmax (binary search).
    Monotonic enough for this simple model.
    """
    Pmax = float(Pmax_elec_W)
    Tmax = max(0.0, float(Tmax_N))
    if Pmax <= 1e-9 or Tmax <= 1e-12:
        return 0.0

    def P(T: float) -> float:
        return _motor_power_transition_elec_W(
            T_motor_N=T,
            theta_rad=theta_rad,
            V_mps=V_mps,
            rho=rho,
            A_one=A_one,
            eta_vtol=eta_vtol,
        )

    # If even Tmax is under power limit, return Tmax
    if P(Tmax) <= Pmax:
        return Tmax

    lo, hi = 0.0, Tmax
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if P(mid) <= Pmax:
            lo = mid
        else:
            hi = mid
    return lo


# =========================
# Rotation / inertia helpers
# =========================

def _estimate_I_axis(inputs: DesignInputs, mass_kg: float) -> float:
    # Simple: I ≈ m*(k*L)^2
    L = float(inputs.fuselage_length)
    k = float(inputs.transition_Izz_k_frac) * L
    return float(mass_kg) * k * k


def _transition_arm_m(inputs: DesignInputs, tail_span_m: float) -> float:
    # Arm from CG to each thrust line (assumes symmetric pair)
    y_prop = 0.5 * float(inputs.fuselage_OD) + float(inputs.prop_y_position_fraction) * (0.5 * float(tail_span_m))
    arm = y_prop - float(getattr(inputs, "cg_y", 0.0))
    return abs(arm)


# =========================
# Stage estimators
# =========================

def _hover_requirements(
    *,
    inputs: DesignInputs,
    mass_kg: float,
    prop_diameter_in: float,
) -> Dict[str, float]:
    """
    Hover requirement used for feasibility:
      T_req_total = (W + m*a_hover) * hover_thrust_margin
      P_req_total = induced(T_req_total)/eta_vtol
    """
    m = float(mass_kg)
    g = ATMOSPHERIC.g
    rho = ATMOSPHERIC.rho

    W = m * g
    a = float(inputs.hover_accel_mps2)

    T_req_total = (W + m * a) * float(inputs.hover_thrust_margin)
    T_req_per = T_req_total / max(int(inputs.n_propellers), 1)

    A_total = _disk_area_total(prop_diameter_in, int(inputs.n_propellers))
    P_ideal = _power_hover_induced_W(T_req_total, rho, A_total)
    P_elec_total = P_ideal / max(float(inputs.hover_eta), 1e-12)
    P_elec_per = P_elec_total / max(int(inputs.n_propellers), 1)

    return {
        "W_N": W,
        "T_req_total_N": T_req_total,
        "T_req_per_motor_N": T_req_per,
        "P_req_total_W": P_elec_total,
        "P_req_per_motor_W": P_elec_per,
    }


def _estimate_vertical_climb_time(
    *,
    inputs: DesignInputs,
    mass_kg: float,
    prop_diameter_in: float,
    T_total_N: float,
    Pmax_total_elec_W: float,
    height_m: float,
) -> Dict[str, float]:
    """
    Estimate climb time (no time constraint). Uses max power at fixed thrust T_total_N.
    If max power only equals hover power (Vc=0), climb is impossible (no altitude gain).
    """
    h = max(0.0, float(height_m))
    if h <= 0.0:
        return {"Vc_mps": 0.0, "time_s": 0.0, "P_used_W": 0.0}

    rho = ATMOSPHERIC.rho
    A_total = _disk_area_total(prop_diameter_in, int(inputs.n_propellers))
    eta = max(float(inputs.hover_eta), 1e-12)
    Pmax = max(0.0, float(Pmax_total_elec_W))

    def P_elec(Vc: float) -> float:
        return _power_axial_climb_W(T_total_N, rho, A_total, Vc) / eta

    P0 = P_elec(0.0)
    if Pmax <= P0 * (1.0 + 1e-6):
        # no positive climb rate achievable at that thrust (with this power limit)
        return {"Vc_mps": 0.0, "time_s": float("inf"), "P_used_W": P0}

    # Solve max Vc such that P_elec(Vc) <= Pmax
    lo, hi = 0.0, 50.0  # m/s upper bound for numerical search
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if P_elec(mid) <= Pmax:
            lo = mid
        else:
            hi = mid

    Vc = max(0.0, lo)
    if Vc <= 1e-6:
        return {"Vc_mps": 0.0, "time_s": float("inf"), "P_used_W": P0}

    t = h / Vc
    # Conservative assumption: we climb at ~max power (close to limit)
    return {"Vc_mps": Vc, "time_s": t, "P_used_W": min(Pmax, P_elec(Vc))}


def _simulate_transition_stage3(
    *,
    inputs: DesignInputs,
    mass_kg: float,
    W_N: float,
    prop_diameter_in: float,
    Tmax_per_motor_N: float,
    Pmax_per_motor_W: float,
    tail_span_m: float,
    V_handoff_mps: float,
    wing_S_m2: float,
    wing_CD0: float,
) -> Dict[str, Any]:
    """
    Simultaneous rotation + acceleration with:
      - per-motor power never exceeding Pmax_per_motor_W (thrust cap solved each step)
      - differential thrust used for rotation torque (2-motor pair)
      - drag included (parasite-only)
      - NO wing lift
      - stops when BOTH: theta=90deg and V>=V_handoff

    Returns timing + altitude loss + trace (lightweight).
    """
    n_prop = int(inputs.n_propellers)
    if n_prop < 2:
        raise ValueError("TRANSITION: requires >=2 propellers for differential thrust.")

    if tail_span_m is None or float(tail_span_m) <= 1e-9:
        raise ValueError("TRANSITION: tail_span_m must be provided and > 0.")

    m = float(mass_kg)
    rho = ATMOSPHERIC.rho
    eta_vtol = max(float(inputs.hover_eta), 1e-12)
    A_one = _disk_area_one(prop_diameter_in)

    theta_total = 0.5 * math.pi
    sf = max(float(inputs.transition_torque_sf), 1e-6)
    Izz = _estimate_I_axis(inputs, m)
    arm = _transition_arm_m(inputs, float(tail_span_m))

    if arm <= 1e-6:
        raise ValueError("TRANSITION: thrust moment arm ~0 (increase tail span / prop_y_position_fraction).")
    if Izz <= 1e-12:
        raise ValueError("TRANSITION: inertia estimate too small / invalid.")

    # State
    t = 0.0
    theta = 0.0
    omega = 0.0

    V = 0.0
    vz = 0.0
    z = 0.0  # altitude change (positive up). Negative => loss.

    # Track when each requirement is met
    t_theta_done: Optional[float] = None
    t_V_done: Optional[float] = None

    # Accounting
    E_Wh = 0.0
    P_peak = 0.0
    T_peak_total = 0.0
    worst_vert_def_N = 0.0
    min_moment_margin = float("inf")

    # Integration settings (no "transition_time_s"; this is adaptive by simulation)
    dt = 0.05
    t_max = 300.0  # safety cap to avoid infinite loops

    # A small trace for debugging/printing
    trace: List[Dict[str, Any]] = []
    trace_stride = 5  # store every Nth step

    step = 0
    while t < t_max:
        # Check completion
        if t_theta_done is None and theta >= theta_total - 1e-3:
            t_theta_done = t
            theta = theta_total
            omega = 0.0

        if t_V_done is None and V >= 0.999 * V_handoff_mps:
            t_V_done = t

        if (t_theta_done is not None) and (t_V_done is not None):
            break

        # Per-motor thrust cap from power limit at current (theta, V)
        T_cap = _solve_T_for_power_limit(
            Pmax_elec_W=Pmax_per_motor_W,
            theta_rad=theta,
            V_mps=V,
            rho=rho,
            A_one=A_one,
            eta_vtol=eta_vtol,
            Tmax_N=Tmax_per_motor_N,
        )
        if T_cap <= 1e-9:
            raise ValueError("TRANSITION: available thrust collapses to ~0 under power limit.")

        # Max moment from 2-motor differential using one motor reduced down to 0
        dT_max = T_cap  # high = T_cap, low = 0
        M_max = dT_max * arm
        alpha_max = M_max / (Izz * sf)

        if alpha_max <= 1e-9:
            raise ValueError(
                f"TRANSITION: insufficient rotation authority (alpha_max={alpha_max:.3e} rad/s²)."
            )

        # Time-optimal bang-bang-like control (accelerate then decelerate)
        theta_rem = max(0.0, theta_total - theta)
        stop_angle = (omega * omega) / (2.0 * alpha_max) if alpha_max > 0 else float("inf")
        alpha_cmd = -alpha_max if stop_angle >= theta_rem else +alpha_max

        # Convert alpha_cmd to required moment and differential thrust
        M_req = abs(Izz * alpha_cmd) * sf
        dT_need = M_req / arm

        # Feasibility: if needed differential > available, rotation torque cannot be achieved
        if dT_need > dT_max * (1.0 + 1e-6):
            raise ValueError(
                f"TRANSITION: torque infeasible (dT_need={dT_need:.3f}N > dT_max={dT_max:.3f}N)."
            )

        # Apply differential by reducing one motor: total thrust reduces by dT_need
        T_low = max(0.0, T_cap - dT_need)
        T_high = T_cap
        T_total = (n_prop - 2) * T_cap + T_high + T_low  # equals n_prop*T_cap - dT_need

        # Actual achieved moment/alpha
        dT_applied = T_high - T_low
        M_avail = dT_applied * arm
        alpha = (M_avail / (Izz * sf)) * (1.0 if alpha_cmd >= 0 else -1.0)

        moment_margin = (M_avail / max(M_req, 1e-12))
        min_moment_margin = min(min_moment_margin, moment_margin)

        # Drag (parasite-only)
        D = _drag_parasite_only_N(V_mps=V, S_m2=wing_S_m2, CD0=wing_CD0)

        # Translational dynamics (NO wing lift)
        ax = (T_total * math.sin(theta) - D) / m
        az = (T_total * math.cos(theta) - W_N) / m

        # Integrate
        V = max(0.0, V + ax * dt)
        vz = vz + az * dt
        z = z + vz * dt

        # Altitude deficit tracking
        vert_support = T_total * math.cos(theta)
        vert_def = max(0.0, W_N - vert_support)
        worst_vert_def_N = max(worst_vert_def_N, vert_def)

        # Power: compute per-motor powers at (pre-step) conditions (conservative)
        P_high = _motor_power_transition_elec_W(
            T_motor_N=T_high, theta_rad=theta, V_mps=V, rho=rho, A_one=A_one, eta_vtol=eta_vtol
        )
        P_low = _motor_power_transition_elec_W(
            T_motor_N=T_low, theta_rad=theta, V_mps=V, rho=rho, A_one=A_one, eta_vtol=eta_vtol
        )
        P_total = (n_prop - 1) * P_high + P_low

        # Hard enforce (numerical guard)
        if P_high > Pmax_per_motor_W * (1.0 + 2e-3):
            raise ValueError(
                f"TRANSITION: per-motor power exceeded (P_high={P_high:.1f}W > Pmax={Pmax_per_motor_W:.1f}W)."
            )

        E_Wh += (P_total * dt) / 3600.0
        P_peak = max(P_peak, P_total)
        T_peak_total = max(T_peak_total, T_total)

        # Rotation integrate
        omega = omega + alpha * dt
        theta = theta + omega * dt
        theta = _clamp(theta, 0.0, theta_total)

        if step % trace_stride == 0:
            trace.append({
                "t_s": t,
                "theta_deg": theta * 180.0 / math.pi,
                "V_mps": V,
                "ax_mps2": ax,
                "az_mps2": az,
                "altitude_change_m": z,
                "T_total_N": T_total,
                "dT_applied_N": dT_applied,
                "M_req_Nm": M_req,
                "M_avail_Nm": M_avail,
                "moment_margin": moment_margin,
                "D_N": D,
                "P_total_W": P_total,
                "P_per_motor_high_W": P_high,
            })

        t += dt
        step += 1

    if (t_theta_done is None) or (t_V_done is None):
        raise ValueError(
            f"TRANSITION: did not complete within {t_max:.0f}s "
            f"(theta_done={t_theta_done is not None}, V_done={t_V_done is not None}, "
            f"theta={theta*180/math.pi:.1f}deg, V={V:.2f}m/s, target={V_handoff_mps:.2f}m/s)."
        )

    alt_loss = max(0.0, -z)
    return {
        "time_s": float(max(t_theta_done, t_V_done)),
        "time_to_rotate_s": float(t_theta_done),
        "time_to_speed_s": float(t_V_done),
        "V_end_mps": float(V),
        "altitude_loss_m": float(alt_loss),
        "worst_vertical_deficit_N": float(worst_vert_def_N),
        "min_moment_margin": float(min_moment_margin),
        "peak_power_total_W": float(P_peak),
        "peak_thrust_total_N": float(T_peak_total),
        "energy_Wh": float(E_Wh),
        "trace": trace,
        "Izz_kgm2": float(Izz),
        "arm_m": float(arm),
    }


def _estimate_accel_time_to_cruise(
    *,
    inputs: DesignInputs,
    mass_kg: float,
    W_N: float,
    Tmax_per_motor_N: float,
    Pmax_per_motor_W: float,
    V_start_mps: float,
    V_target_mps: float,
    wing_S_m2: float,
    wing_AR: float,
    wing_CD0: float,
) -> Dict[str, float]:
    """
    Power-limited acceleration from V_start -> V_target (wing effective; includes induced drag).
    No fixed time input; estimate dt = dV/a(V). Infeasible if net accel <= 0 at any point.
    """
    V1 = float(V_start_mps)
    V2 = float(V_target_mps)
    if V2 <= V1 + 1e-9:
        return {"time_s": 0.0, "energy_Wh": 0.0, "peak_power_W": 0.0, "peak_thrust_N": 0.0}

    m = float(mass_kg)
    n_prop = int(inputs.n_propellers)
    eta_prop = max(float(inputs.propeller_efficiency), 1e-12)

    Pmax_total = max(0.0, float(Pmax_per_motor_W) * n_prop)
    Tmax_total = max(0.0, float(Tmax_per_motor_N) * n_prop)

    N = 200
    dV = (V2 - V1) / N

    t_sum = 0.0
    E_Wh = 0.0
    P_peak = 0.0
    T_peak = 0.0

    for i in range(N):
        V = V1 + (i + 0.5) * dV
        V_eff = max(V, 0.5)

        D = _drag_parabolic_N(
            L_ref_N=W_N,
            V_mps=V_eff,
            S_m2=wing_S_m2,
            AR=wing_AR,
            CD0=wing_CD0,
            e_oswald=inputs.e_oswald,
        )

        # Available forward thrust from power (and thrust cap)
        T_from_power = (eta_prop * Pmax_total) / V_eff
        T_avail = min(Tmax_total, T_from_power)

        a = (T_avail - D) / m
        if a <= 1e-6:
            raise ValueError(
                f"ACCEL: cannot reach cruise speed (net accel <= 0 at V≈{V_eff:.1f} m/s). "
                f"T_avail={T_avail:.1f}N, D={D:.1f}N"
            )

        dt = dV / a
        t_sum += dt

        # Electrical power used to produce T_avail at this V (simple prop-eff model)
        P_elec = (T_avail * V_eff) / eta_prop
        P_elec = min(P_elec, Pmax_total)

        E_Wh += (P_elec * dt) / 3600.0
        P_peak = max(P_peak, P_elec)
        T_peak = max(T_peak, T_avail)

    return {"time_s": t_sum, "energy_Wh": E_Wh, "peak_power_W": P_peak, "peak_thrust_N": T_peak}


# =========================
# Mission profile (feasibility + timing)
# =========================

def estimate_tailsitter_mission_profile(
    inputs: DesignInputs,
    W0_mass_kg: float,
    prop_diameter_in: float,
    cruise_power_elec_W: float,
    V_stall_mps: float,
    wing_S_m2: float,
    wing_AR: float,
    wing_CD0: float,
    tail_span_m: float,
    usable_energy_Wh: Optional[float] = None,
    max_thrust_per_motor_N: Optional[float] = None,
    max_power_per_motor_W: Optional[float] = None,
) -> Dict[str, Any]:
    """
    Stages:
      1) Hover (fixed hover time)
      2) Vertical climb to height (estimate time from available power; no fixed time)
      3) Transition (estimate time from available power + differential thrust torque; drag included; no lift)
      4) Accelerate handoff -> cruise (estimate time from available power)
      5) Cruise (battery-limited max time if usable_energy_Wh provided)

    Feasibility gates (hard fails):
      - cannot hover (thrust or power)
      - cannot climb to altitude (no positive climb rate)
      - cannot transition (torque or cannot reach handoff speed; or altitude loss > climb height)
      - cannot accelerate to cruise
    """
    m = float(W0_mass_kg)
    g = ATMOSPHERIC.g
    W_N = m * g

    n_prop = int(inputs.n_propellers)
    if n_prop <= 0:
        raise ValueError("n_propellers must be >= 1.")

    if max_thrust_per_motor_N is None or max_power_per_motor_W is None:
        raise ValueError("Mission profile requires max_thrust_per_motor_N and max_power_per_motor_W.")

    Tmax = float(max_thrust_per_motor_N)
    Pmax = float(max_power_per_motor_W)

    V_handoff = max(float(inputs.transition_min_speed_mps), float(inputs.transition_stall_margin) * float(V_stall_mps))
    V_cruise = float(inputs.speed)

    stages: List[Dict[str, Any]] = []

    peak_elec_power_W = 0.0
    peak_elec_power_per_motor_W = 0.0
    peak_thrust_total_N = 0.0
    peak_thrust_per_motor_N = 0.0

    def _push(stage: str, time_s: float, avgP_W: float, peakP_W: float, T_total_N: float, notes: str = "", extra: Optional[Dict[str, Any]] = None) -> None:
        nonlocal peak_elec_power_W, peak_elec_power_per_motor_W, peak_thrust_total_N, peak_thrust_per_motor_N
        time_s = max(0.0, float(time_s))
        avgP_W = max(0.0, float(avgP_W))
        peakP_W = max(0.0, float(peakP_W))
        T_total_N = max(0.0, float(T_total_N))

        energy_Wh = (avgP_W * time_s) / 3600.0 if time_s > 0 else 0.0

        d = {
            "stage": stage,
            "time_s": time_s,
            "time_min": time_s / 60.0,
            "avg_elec_power_W": avgP_W,
            "peak_elec_power_W": peakP_W,
            "energy_Wh": energy_Wh,
            "thrust_total_N": T_total_N,
            "thrust_per_motor_N": (T_total_N / n_prop) if n_prop > 0 else 0.0,
            "notes": notes,
        }
        if extra:
            d.update(extra)
        stages.append(d)

        peak_elec_power_W = max(peak_elec_power_W, peakP_W)
        peak_elec_power_per_motor_W = max(peak_elec_power_per_motor_W, peakP_W / n_prop if n_prop > 0 else 0.0)
        peak_thrust_total_N = max(peak_thrust_total_N, T_total_N)
        peak_thrust_per_motor_N = max(peak_thrust_per_motor_N, T_total_N / n_prop if n_prop > 0 else 0.0)

    # ------------------------
    # Stage 1: Hover (fixed time)
    # ------------------------
    hover = _hover_requirements(inputs=inputs, mass_kg=m, prop_diameter_in=prop_diameter_in)

    T_hover_req_per = hover["T_req_per_motor_N"]
    P_hover_req_per = hover["P_req_per_motor_W"]

    if Tmax < T_hover_req_per:
        raise ValueError(
            f"HOVER: insufficient thrust (Tmax={Tmax:.2f}N < required={T_hover_req_per:.2f}N per motor)."
        )
    if Pmax < P_hover_req_per:
        raise ValueError(
            f"HOVER: insufficient power (Pmax={Pmax:.1f}W < required={P_hover_req_per:.1f}W per motor)."
        )

    t1 = float(inputs.vtol_takeoff_hover_time_s)
    P1_total = hover["P_req_total_W"]
    T1_total = hover["T_req_total_N"]
    _push(
        "VTOL hover",
        t1,
        P1_total,
        P1_total,
        T1_total,
        notes=f"hover_req_per_motor: T={T_hover_req_per:.2f}N, P={P_hover_req_per:.0f}W",
    )

    # ------------------------
    # Stage 2: Vertical climb (estimate time, no time constraint)
    # ------------------------
    h = float(inputs.vtol_climb_height_m)
    Pmax_total = Pmax * n_prop

    climb = _estimate_vertical_climb_time(
        inputs=inputs,
        mass_kg=m,
        prop_diameter_in=prop_diameter_in,
        T_total_N=T1_total,                 # climb at hover-required thrust level
        Pmax_total_elec_W=Pmax_total,       # limited by available max electrical power
        height_m=h,
    )

    if h > 0.0 and (not math.isfinite(climb["time_s"]) or climb["Vc_mps"] <= 1e-6):
        raise ValueError(
            f"CLIMB: cannot reach {h:.1f}m (no positive climb rate at max power). "
            f"P_hover≈{climb['P_used_W']:.0f}W, Pmax_total={Pmax_total:.0f}W"
        )

    t2 = 0.0 if h <= 0.0 else float(climb["time_s"])
    P2 = 0.0 if h <= 0.0 else float(climb["P_used_W"])
    _push(
        "VTOL climb to altitude",
        t2 if math.isfinite(t2) else 0.0,
        P2,
        P2,
        T1_total,
        notes=("disabled" if h <= 0.0 else f"h={h:.1f}m, Vc≈{climb['Vc_mps']:.2f}m/s"),
        extra={"climb_rate_mps": float(climb["Vc_mps"]), "climb_height_m": h},
    )

    # ------------------------
    # Stage 3: Transition (estimate time from available power + torque)
    # ------------------------
    if float(getattr(inputs, "vtol_enabled", True)) and V_handoff > 0.0:
        tr = _simulate_transition_stage3(
            inputs=inputs,
            mass_kg=m,
            W_N=W_N,
            prop_diameter_in=prop_diameter_in,
            Tmax_per_motor_N=Tmax,
            Pmax_per_motor_W=Pmax,
            tail_span_m=float(tail_span_m),
            V_handoff_mps=V_handoff,
            wing_S_m2=float(wing_S_m2),
            wing_CD0=float(wing_CD0),
        )

        # Ground-hit feasibility: must not lose more altitude than you climbed
        alt_loss = float(tr["altitude_loss_m"])
        if h > 0.0 and alt_loss > h * 0.999:
            raise ValueError(
                f"TRANSITION: altitude loss too large (loss≈{alt_loss:.2f}m > available climb height {h:.2f}m)."
            )

        t3 = float(tr["time_s"])
        P3_peak = float(tr["peak_power_total_W"])
        P3_avg = float(tr["energy_Wh"]) * 3600.0 / max(t3, 1e-9)
        _push(
            "Transition to handoff",
            t3,
            P3_avg,
            P3_peak,
            float(tr["peak_thrust_total_N"]),
            notes=(
                f"Vhandoff={V_handoff:.1f}m/s, "
                f"t_rot={tr['time_to_rotate_s']:.2f}s, t_V={tr['time_to_speed_s']:.2f}s, "
                f"alt_loss≈{alt_loss:.2f}m, min_moment_margin≈{tr['min_moment_margin']:.2f}x"
            ),
            extra={
                "handoff_speed_mps": V_handoff,
                "time_to_rotate_s": float(tr["time_to_rotate_s"]),
                "time_to_speed_s": float(tr["time_to_speed_s"]),
                "altitude_loss_m": alt_loss,
                "worst_vertical_deficit_N": float(tr["worst_vertical_deficit_N"]),
                "min_moment_margin": float(tr["min_moment_margin"]),
                "transition_trace": tr["trace"],
                "Izz_kgm2": float(tr["Izz_kgm2"]),
                "arm_m": float(tr["arm_m"]),
            },
        )
    else:
        _push("Transition to handoff", 0.0, 0.0, 0.0, 0.0, notes="disabled")

    # ------------------------
    # Stage 4: Accel to cruise (estimate time from available power)
    # ------------------------
    if V_cruise > V_handoff + 1e-9:
        accel = _estimate_accel_time_to_cruise(
            inputs=inputs,
            mass_kg=m,
            W_N=W_N,
            Tmax_per_motor_N=Tmax,
            Pmax_per_motor_W=Pmax,
            V_start_mps=V_handoff,
            V_target_mps=V_cruise,
            wing_S_m2=float(wing_S_m2),
            wing_AR=float(wing_AR),
            wing_CD0=float(wing_CD0),
        )
        t4 = float(accel["time_s"])
        P4_peak = float(accel["peak_power_W"])
        P4_avg = float(accel["energy_Wh"]) * 3600.0 / max(t4, 1e-9)
        _push(
            "Accel handoff → cruise",
            t4,
            P4_avg,
            P4_peak,
            float(accel["peak_thrust_N"]),
            notes=f"{V_handoff:.1f}→{V_cruise:.1f} m/s",
        )
    else:
        _push("Accel handoff → cruise", 0.0, 0.0, 0.0, 0.0, notes="disabled")

    # ------------------------
    # Stage 5: Cruise (battery-limited if usable_energy_Wh provided)
    # ------------------------
    energy_precruise_Wh = sum(s["energy_Wh"] for s in stages)
    cruise_time_s = float(inputs.cruise_endurance_min) * 60.0
    cruise_time_source = "requested"

    energy_remaining_for_cruise_Wh = None
    battery_deficit_Wh = None

    if usable_energy_Wh is not None:
        energy_remaining_for_cruise_Wh = float(usable_energy_Wh) - energy_precruise_Wh
        if cruise_power_elec_W > 1e-9:
            cruise_time_s = max(0.0, energy_remaining_for_cruise_Wh) * 3600.0 / float(cruise_power_elec_W)
        else:
            cruise_time_s = 0.0
        cruise_time_source = "battery-limited"
        battery_deficit_Wh = max(0.0, -energy_remaining_for_cruise_Wh)

    P5 = max(0.0, float(cruise_power_elec_W))
    _push("Cruise", cruise_time_s, P5, P5, W_N, notes=cruise_time_source)

    total_time_s = sum(s["time_s"] for s in stages)
    total_energy_Wh = sum(s["energy_Wh"] for s in stages)

    return {
        "stages": stages,
        "handoff_speed_mps": V_handoff,
        "cruise_speed_mps": V_cruise,

        "total_time_s": total_time_s,
        "total_time_min": total_time_s / 60.0,
        "total_energy_Wh": total_energy_Wh,

        "peak_elec_power_W": peak_elec_power_W,
        "peak_elec_power_per_motor_W": peak_elec_power_per_motor_W,
        "peak_thrust_total_N": peak_thrust_total_N,
        "peak_thrust_per_motor_N": peak_thrust_per_motor_N,

        "usable_energy_Wh": usable_energy_Wh,
        "energy_precruise_Wh": energy_precruise_Wh,
        "energy_remaining_for_cruise_Wh": energy_remaining_for_cruise_Wh,
        "battery_deficit_Wh": battery_deficit_Wh,

        "cruise_time_requested_min": float(inputs.cruise_endurance_min),
        "cruise_time_max_min": cruise_time_s / 60.0,
        "cruise_time_source": cruise_time_source,
    }


# =========================
# Prop-fuselage clearance (unchanged)
# =========================

def _check_prop_fuselage_clearance(
    prop_diameter_in: float,
    prop_y_position_fraction: float,
    wing_span_m: float,
    fuselage_OR_m: float,
    min_clearance_m: float,
) -> Tuple[bool, float]:
    prop_y_position_m = float(prop_y_position_fraction) * (float(wing_span_m) / 2.0)
    prop_radius_m = (float(prop_diameter_in) * 0.0254) / 2.0
    actual_clearance_m = prop_y_position_m - prop_radius_m - float(fuselage_OR_m)
    meets_requirement = actual_clearance_m >= float(min_clearance_m)
    return meets_requirement, actual_clearance_m


def _required_span_extension_for_clearance(
    prop_diameter_in: float,
    prop_y_position_fraction: float,
    current_span_m: float,
    fuselage_OR_m: float,
    min_clearance_m: float,
) -> float:
    prop_radius_m = (float(prop_diameter_in) * 0.0254) / 2.0
    required_y_m = prop_radius_m + float(fuselage_OR_m) + float(min_clearance_m)
    if float(prop_y_position_fraction) <= 1e-9:
        return float("inf")
    required_span_m = 2.0 * required_y_m / float(prop_y_position_fraction)
    return max(0.0, required_span_m - float(current_span_m))


# =========================
# Envelope filter (ONLY hard feasibility)
# =========================

def _filter_propulsion_db_for_envelope(
    propulsion_db: List[Dict[str, Any]],
    inputs: DesignInputs,
    W0_mass_kg: float,
    cruise_power_elec_W: float,
    V_stall_mps: float,
    wing_S_m2: float,
    wing_AR: float,
    wing_CD0: float,
    tail_span_m: float,
) -> List[Dict[str, Any]]:
    filtered: List[Dict[str, Any]] = []

    failures: List[str] = []
    n_checked = 0

    for combo in propulsion_db:
        if not all(k in combo for k in ("prop_diameter_in", "max_thrust_N", "max_power_W")):
            continue

        n_checked += 1

        prop_d_in = float(combo["prop_diameter_in"])
        Tmax = float(combo["max_thrust_N"])
        Pmax = float(combo["max_power_W"])
        name = combo.get("name", f"{prop_d_in:.1f}in")

        try:
            profile = estimate_tailsitter_mission_profile(
                inputs=inputs,
                W0_mass_kg=float(W0_mass_kg),
                prop_diameter_in=prop_d_in,
                cruise_power_elec_W=float(cruise_power_elec_W),
                V_stall_mps=float(V_stall_mps),
                wing_S_m2=float(wing_S_m2),
                wing_AR=float(wing_AR),
                wing_CD0=float(wing_CD0),
                tail_span_m=float(tail_span_m),
                usable_energy_Wh=None,  # envelope filter doesn't need battery energy
                max_thrust_per_motor_N=Tmax,
                max_power_per_motor_W=Pmax,
            )
        except ValueError as e:
            failures.append(f"  [{name}] {str(e)}")
            continue

        combo2 = dict(combo)
        combo2["_mission_profile_preview"] = profile
        filtered.append(combo2)

    if not filtered:
        msg = [
            "No propulsion combo satisfies the required feasibility envelope.\n",
            f"Checked {n_checked} combos, 0 feasible.\n",
        ]
        if failures:
            msg.append("FAILURE SUMMARY:\n")
            msg.extend(failures[:60])
            if len(failures) > 60:
                msg.append(f"\n  ... and {len(failures) - 60} more\n")
        msg.append("\nFix: bigger props / stronger motors (Tmax/Pmax), more propellers, reduce MTOW, reduce drag, or increase climb height.")
        raise ValueError("".join(msg))

    return filtered


# =========================
# Selection logic (API unchanged)
# =========================

def select_propulsion_system(
    drag_cruise_N: float,
    cruise_power_W: float,
    cruise_power_elec_W: float,
    fuselage_ID: float,
    inputs: DesignInputs,
    battery_selection: Optional[Dict[str, Any]] = None,
    W0_mass_kg: float = 0.0,
    V_stall_mps: Optional[float] = None,
    wing_S_m2: Optional[float] = None,
    wing_AR: Optional[float] = None,
    tail_span_m: Optional[float] = None,
    wing_CD0: Optional[float] = None,
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:

    if W0_mass_kg <= 0.0:
        raise ValueError("W0_mass_kg must be > 0 for mission envelope sizing.")
    if V_stall_mps is None or wing_S_m2 is None or wing_AR is None or wing_CD0 is None:
        raise ValueError("Must pass V_stall_mps, wing_S_m2, wing_AR, wing_CD0 into select_propulsion_system().")
    if tail_span_m is None:
        raise ValueError("Must pass tail_span_m into select_propulsion_system() for transition torque sizing.")

    # Battery selection (as before)
    if battery_selection is None:
        battery_db = get_battery_database()
        battery_selection = select_battery(
            cruise_power_elec_W,
            inputs.cruise_endurance_min,
            battery_db,
            fuselage_ID
        )

    # Propulsion DB + feasibility filter (NEW)
    propulsion_db = get_propulsion_database()
    propulsion_db = _filter_propulsion_db_for_envelope(
        propulsion_db=propulsion_db,
        inputs=inputs,
        W0_mass_kg=float(W0_mass_kg),
        cruise_power_elec_W=float(cruise_power_elec_W),
        V_stall_mps=float(V_stall_mps),
        wing_S_m2=float(wing_S_m2),
        wing_AR=float(wing_AR),
        wing_CD0=float(wing_CD0),
        tail_span_m=float(tail_span_m),
    )

    # Cruise-point selection (as before)
    propulsion_selection = select_propulsion(
        drag_cruise_N,
        cruise_power_elec_W,
        propulsion_db,
        n_propellers=inputs.n_propellers,
        thrust_margin=inputs.propulsion_margin,
        power_margin=inputs.propulsion_power_margin,
        battery_voltage_V=battery_selection["battery"]["voltage_V"],
    )

    if propulsion_selection is None:
        return battery_selection, None, None

    combo = propulsion_selection["combo"]
    prop_d_in = float(combo["prop_diameter_in"])

    # Clearance check (as before)
    span_for_clearance_m = float(tail_span_m) if (tail_span_m is not None) else 0.0
    meets_clearance, actual_clearance = _check_prop_fuselage_clearance(
        prop_diameter_in=prop_d_in,
        prop_y_position_fraction=inputs.prop_y_position_fraction,
        wing_span_m=span_for_clearance_m,
        fuselage_OR_m=inputs.fuselage_OR,
        min_clearance_m=inputs.prop_fuselage_clearance_min_m,
    )
    if not meets_clearance:
        ext_m = _required_span_extension_for_clearance(
            prop_diameter_in=prop_d_in,
            prop_y_position_fraction=inputs.prop_y_position_fraction,
            current_span_m=span_for_clearance_m,
            fuselage_OR_m=inputs.fuselage_OR,
            min_clearance_m=inputs.prop_fuselage_clearance_min_m,
        )
        propulsion_selection["requires_tail_span_extension_m"] = ext_m
        propulsion_selection["prop_fuselage_clearance_actual_m"] = float(actual_clearance)
        propulsion_selection["prop_fuselage_clearance_required_m"] = float(inputs.prop_fuselage_clearance_min_m)

    # Mission profile for selected combo (battery-aware cruise time)
    usable_Wh = float(battery_selection["usable_energy_Wh"])
    profile = estimate_tailsitter_mission_profile(
        inputs=inputs,
        W0_mass_kg=float(W0_mass_kg),
        prop_diameter_in=prop_d_in,
        cruise_power_elec_W=float(cruise_power_elec_W),
        V_stall_mps=float(V_stall_mps),
        wing_S_m2=float(wing_S_m2),
        wing_AR=float(wing_AR),
        wing_CD0=float(wing_CD0),
        tail_span_m=float(tail_span_m),
        usable_energy_Wh=usable_Wh,
        max_thrust_per_motor_N=float(combo["max_thrust_N"]),
        max_power_per_motor_W=float(combo["max_power_W"]),
    )

    battery_selection["mission_profile"] = profile
    propulsion_selection["mission_profile"] = profile

    # Avionics selection (ESC sizing)
    batt_voltage = battery_selection["battery"]["voltage_V"]
    cruise_per_motor_W = propulsion_selection["required_power_per_motor_W"]
    peak_per_motor_W = profile["peak_elec_power_per_motor_W"]

    esc_sizing_W = max(cruise_per_motor_W, peak_per_motor_W)
    esc_cont_current_A = esc_sizing_W / (batt_voltage * max(float(inputs.esc_efficiency), 1e-12))

    avionics_selection = estimate_avionics_and_electrics(
        W0_mass_kg=0.0,
        n_motors=inputs.n_propellers,
        esc_cont_current_A=esc_cont_current_A,
        autopilot_class=inputs.autopilot_class,
        n_servos=inputs.n_servos,
    )

    propulsion_selection["esc_sizing_power_per_motor_W"] = float(esc_sizing_W)
    propulsion_selection["esc_cont_current_A_per_motor"] = float(esc_cont_current_A)

    return battery_selection, propulsion_selection, avionics_selection


# =========================
# Mass & conversion to results (unchanged)
# =========================

def calculate_component_masses(
    battery_selection: Optional[Dict[str, Any]],
    propulsion_selection: Optional[Dict[str, Any]],
    avionics_selection: Optional[Any],
    inputs: DesignInputs,
) -> Tuple[float, float, float]:

    W_battery = battery_selection["battery"]["mass_kg"] if battery_selection is not None else 0.0

    W_propulsion = 0.0
    if propulsion_selection is not None:
        combo = propulsion_selection["combo"]
        W_propulsion = (combo["motor_mass_kg"] + combo["prop_mass_kg"]) * inputs.n_propellers

    W_avionics = avionics_selection.total_avionics_electrics_mass_kg if avionics_selection is not None else 0.0
    return W_battery, W_propulsion, W_avionics


def convert_to_results(
    battery_selection: Optional[Dict[str, Any]],
    propulsion_selection: Optional[Dict[str, Any]],
    avionics_selection: Optional[Any],
) -> Tuple[Optional[BatteryResults], Optional[PropulsionResults], Optional[AvionicsResults]]:

    battery_results = None
    propulsion_results = None
    avionics_results = None

    if battery_selection is not None:
        batt = battery_selection["battery"]
        battery_results = BatteryResults(
            name=batt["name"],
            voltage_V=batt["voltage_V"],
            capacity_mAh=batt["capacity_mAh"],
            mass_kg=batt["mass_kg"],
            length_m=batt["length_m"],
            width_m=batt["width_m"],
            height_m=batt["height_m"],
            usable_energy_Wh=battery_selection["usable_energy_Wh"],
            endurance_min_at_cruise=battery_selection["endurance_min"],
            C_at_cruise=battery_selection["C_at_cruise"],
            meets_endurance_req=battery_selection["meets_endurance"],
        )
        if "mission_profile" in battery_selection:
            battery_results.mission_profile = battery_selection["mission_profile"]

    if propulsion_selection is not None:
        combo = propulsion_selection["combo"]
        n_prop = propulsion_selection["n_propellers"]

        propulsion_results = PropulsionResults(
            n_propellers=n_prop,
            name=combo["name"],
            prop_diameter_in=combo["prop_diameter_in"],
            prop_pitch_in=combo["prop_pitch_in"],
            motor_kv=combo["motor_kv"],
            motor_mass_kg=combo["motor_mass_kg"],
            prop_mass_kg=combo["prop_mass_kg"],
            total_motor_mass_kg=combo["motor_mass_kg"] * n_prop,
            total_prop_mass_kg=combo["prop_mass_kg"] * n_prop,
            total_propulsion_mass_kg=(combo["motor_mass_kg"] + combo["prop_mass_kg"]) * n_prop,
            max_thrust_per_motor_N=combo["max_thrust_N"],
            required_thrust_per_motor_N=propulsion_selection["required_thrust_per_motor_N"],
            max_power_per_motor_W=combo["max_power_W"],
            required_power_per_motor_W=propulsion_selection["required_power_per_motor_W"],
            meets_thrust_req=propulsion_selection["meets_thrust"],
            meets_power_req=propulsion_selection["meets_power"],
            est_current_A_per_motor_at_cruise=propulsion_selection.get("est_current_A_per_motor_at_cruise"),
        )

        if "mission_profile" in propulsion_selection:
            propulsion_results.mission_profile = propulsion_selection["mission_profile"]
            propulsion_results.esc_sizing_power_per_motor_W = propulsion_selection.get("esc_sizing_power_per_motor_W", None)
            propulsion_results.esc_cont_current_A_per_motor = propulsion_selection.get("esc_cont_current_A_per_motor", None)

    if avionics_selection is not None:
        avionics_results = AvionicsResults(
            autopilot_name=avionics_selection.autopilot_name,
            autopilot_mass_kg=avionics_selection.autopilot_mass_kg,
            gps_mass_kg=avionics_selection.gps_mass_kg,
            airspeed_mass_kg=avionics_selection.airspeed_mass_kg,
            rc_rx_mass_kg=avionics_selection.rc_rx_mass_kg,
            telemetry_mass_kg=avionics_selection.telemetry_mass_kg,
            power_module_mass_kg=avionics_selection.power_module_mass_kg,
            servos_mass_kg=avionics_selection.servos_mass_kg,
            esc_mass_total_kg=avionics_selection.esc_mass_total_kg,
            misc_electronics_mass_kg=avionics_selection.misc_electronics_mass_kg,
            total_avionics_electrics_mass_kg=avionics_selection.total_avionics_electrics_mass_kg,
        )

    return battery_results, propulsion_results, avionics_results
