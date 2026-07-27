"""
propulsion_system.py

Propulsion system selection and sizing (cruise + tailsitter VTOL mission envelope).

Updates:
- Mission profile cruise time is limited by remaining usable battery energy:
    cruise_time = max(0, (usable_Wh - energy_precruise_Wh) / cruise_power_elec_W) * 3600
- Mission profile always attached to BatteryResults + PropulsionResults for printing.
- No getattr/max(getattr...) guards; uses inputs.variable directly.
"""

from typing import Optional, Dict, Any, Tuple, List
import math

from design_inputs import DesignInputs
from constants import ATMOSPHERIC
from batteries import get_battery_database
from propulsion import get_propulsion_database
from helper_function import select_battery, select_propulsion, rank_battery_candidates
from avionics import estimate_avionics_and_electrics
from results import BatteryResults, PropulsionResults, AvionicsResults


# ============================================================
# Momentum theory / aero approximations
# ============================================================

def _disk_area_total(prop_diameter_in: float, n_propellers: int) -> float:
    d_m = prop_diameter_in * 0.0254
    A_one = math.pi * (0.5 * d_m) ** 2
    return n_propellers * A_one


def _power_hover_induced_W(T_N: float, rho: float, A_total: float) -> float:
    return (T_N ** 1.5) / math.sqrt(2.0 * rho * max(A_total, 1e-9))


def _power_axial_climb_W(T_N: float, rho: float, A_total: float, Vc_mps: float) -> float:
    v_h = math.sqrt(max(T_N, 1e-9) / (2.0 * rho * max(A_total, 1e-9)))
    return T_N * (0.5 * Vc_mps + math.sqrt((0.5 * Vc_mps) ** 2 + v_h ** 2))


def _drag_parabolic_N(W_N: float, V_mps: float, S_m2: float, AR: float, CD0: float, e_oswald: float) -> float:
    q = 0.5 * ATMOSPHERIC.rho * V_mps ** 2
    if q <= 1e-9:
        return 0.0
    CL = W_N / (q * S_m2)
    k = 1.0 / (math.pi * max(e_oswald, 1e-9) * max(AR, 1e-9))
    CD = CD0 + k * CL ** 2
    return q * S_m2 * CD


# ============================================================
# Hover requirement (simple guardrail)
# ============================================================

def _hover_requirements_for_prop(
    W0_mass_kg: float,
    prop_diameter_in: float,
    n_propellers: int,
    inputs: DesignInputs,
) -> Dict[str, float]:
    rho = ATMOSPHERIC.rho
    g = ATMOSPHERIC.g

    W_N = W0_mass_kg * g
    T_total_req_N = (W_N + W0_mass_kg * inputs.hover_accel_mps2) * inputs.hover_thrust_margin
    T_per_motor_req_N = T_total_req_N / n_propellers

    A_total = _disk_area_total(prop_diameter_in, n_propellers)
    P_ideal_W = _power_hover_induced_W(T_total_req_N, rho, A_total)
    P_elec_total_W = P_ideal_W / inputs.hover_eta
    P_elec_per_motor_W = P_elec_total_W / n_propellers

    return {
        "T_total_req_N": T_total_req_N,
        "T_per_motor_req_N": T_per_motor_req_N,
        "P_hover_ideal_W": P_ideal_W,
        "P_hover_elec_total_W": P_elec_total_W,
        "P_hover_elec_per_motor_W": P_elec_per_motor_W,
    }


# ============================================================
# Mission profile (energy + time per stage + peaks)
# ============================================================

def estimate_tailsitter_mission_profile(
    inputs: DesignInputs,
    W0_mass_kg: float,
    prop_diameter_in: float,
    cruise_power_elec_W: float,
    V_stall_mps: float,
    wing_S_m2: float,
    wing_AR: float,
    wing_CD0: float,
    wing_CD_cruise: float,
    S_tail_m2: float = 0.0,
    tail_CD_cruise: float = 0.0,
    usable_energy_Wh: Optional[float] = None,   # <-- NEW
) -> Dict[str, Any]:
    """
    Stages:
      1) VTOL takeoff hover
      2) Vertical climb to height (accelerate + capped climb)
      3) Transition to safe handoff speed (>= stall margin)
      4) Accelerate to cruise speed
      5) Cruise (NOW: computed from remaining battery energy)

    If usable_energy_Wh is provided:
      - cruise time becomes the MAX possible with remaining energy.
    If usable_energy_Wh is None:
      - cruise time uses inputs.cruise_endurance_min (legacy behavior)
    """
    rho = ATMOSPHERIC.rho
    g = ATMOSPHERIC.g

    m = W0_mass_kg
    W_N = m * g
    n_prop = inputs.n_propellers
    A_total = _disk_area_total(prop_diameter_in, n_prop)

    eta_vtol = inputs.hover_eta
    eta_prop = inputs.propeller_efficiency

    V_handoff = max(inputs.transition_min_speed_mps, inputs.transition_stall_margin * V_stall_mps)
    V_cruise = inputs.speed

    stages: List[Dict[str, Any]] = []

    peak_elec_power_W = 0.0
    peak_elec_power_per_motor_W = 0.0
    peak_thrust_total_N = 0.0
    peak_thrust_per_motor_N = 0.0

    def push(stage: str, time_s: float, avgP_W: float, peakP_W: float, T_total: float, notes: str = ""):
        nonlocal peak_elec_power_W, peak_elec_power_per_motor_W, peak_thrust_total_N, peak_thrust_per_motor_N
        energy_Wh = avgP_W * time_s / 3600.0

        stages.append({
            "stage": stage,
            "time_s": time_s,
            "time_min": time_s / 60.0,
            "avg_elec_power_W": avgP_W,
            "peak_elec_power_W": peakP_W,
            "energy_Wh": energy_Wh,
            "thrust_total_N": T_total,
            "thrust_per_motor_N": T_total / n_prop,
            "notes": notes,
        })

        peak_elec_power_W = max(peak_elec_power_W, peakP_W)
        peak_elec_power_per_motor_W = max(peak_elec_power_per_motor_W, peakP_W / n_prop)
        peak_thrust_total_N = max(peak_thrust_total_N, T_total)
        peak_thrust_per_motor_N = max(peak_thrust_per_motor_N, T_total / n_prop)

    # ------------------------
    # Stage 1–3: VTOL-only stages (skip if vtol_enabled == False)
    # ------------------------
    if inputs.vtol_enabled:
        # ------------------------
        # Stage 1: VTOL takeoff hover
        # ------------------------
        t1 = inputs.vtol_takeoff_hover_time_s
        T1 = W_N * inputs.hover_thrust_margin
        P1 = _power_hover_induced_W(T1, rho, A_total) / eta_vtol
        push("VTOL takeoff / hover", t1, P1, P1, T1)

        # ------------------------
        # Stage 2: Vertical climb
        # ------------------------
        h = inputs.vtol_climb_height_m
        a_up = inputs.vtol_climb_accel_mps2
        vmax = inputs.vtol_climb_vmax_mps

        if h > 0.0 and a_up > 0.0 and vmax > 0.0:
            t_acc = vmax / a_up
            h_acc = 0.5 * a_up * t_acc ** 2

            if h <= h_acc:
                t2 = math.sqrt(2.0 * h / a_up)
                Vc_end = a_up * t2
                Vc_avg = 0.5 * Vc_end
                T2 = m * (g + a_up) * inputs.hover_thrust_margin
                P2_avg = _power_axial_climb_W(T2, rho, A_total, Vc_avg) / eta_vtol
                P2_peak = _power_axial_climb_W(T2, rho, A_total, Vc_end) / eta_vtol
                push("Vertical climb (accelerating)", t2, P2_avg, P2_peak, T2, notes=f"h={h:.1f}m")
            else:
                h2 = h - h_acc
                t_const = h2 / vmax

                T2a = m * (g + a_up) * inputs.hover_thrust_margin
                P2a_avg = _power_axial_climb_W(T2a, rho, A_total, 0.5 * vmax) / eta_vtol
                P2a_peak = _power_axial_climb_W(T2a, rho, A_total, vmax) / eta_vtol
                E2a = P2a_avg * t_acc / 3600.0

                T2b = W_N * inputs.hover_thrust_margin
                P2b = _power_axial_climb_W(T2b, rho, A_total, vmax) / eta_vtol
                E2b = P2b * t_const / 3600.0

                t2 = t_acc + t_const
                E2 = E2a + E2b
                P2_avg = (E2 * 3600.0) / t2
                P2_peak = max(P2a_peak, P2b)
                T2_peak = max(T2a, T2b)
                push("Vertical climb to height", t2, P2_avg, P2_peak, T2_peak, notes=f"h={h:.1f}m")
        elif h > 0.0 and vmax > 0.0:
            t2 = h / vmax
            T2 = W_N * inputs.hover_thrust_margin
            P2 = _power_axial_climb_W(T2, rho, A_total, vmax) / eta_vtol
            push("Vertical climb to height", t2, P2, P2, T2, notes=f"h={h:.1f}m")
        else:
            push("Vertical climb to height", 0.0, 0.0, 0.0, 0.0, notes="disabled")

        # ------------------------
        # Stage 3: Transition to handoff speed
        # ------------------------
        t3 = inputs.transition_time_s
        a_x = V_handoff / t3

        T3 = math.sqrt(W_N ** 2 + (m * a_x) ** 2) * inputs.hover_thrust_margin
        P3_ind = _power_hover_induced_W(T3, rho, A_total)
        P3_trans = (m * a_x) * (0.5 * V_handoff)
        P3 = (P3_ind + P3_trans) / eta_vtol
        push("Transition to handoff speed", t3, P3, P3, T3, notes=f"handoff={V_handoff:.1f} m/s")
    else:
        # Explicitly keep stages present but disabled (for consistent reporting)
        push("VTOL takeoff / hover", 0.0, 0.0, 0.0, 0.0, notes="vtol disabled")
        push("Vertical climb to height", 0.0, 0.0, 0.0, 0.0, notes="vtol disabled")
        push("Transition to handoff speed", 0.0, 0.0, 0.0, 0.0, notes="vtol disabled")

    # ------------------------
    # Stage 4: Accelerate to cruise speed
    # a4 = (V_cruise - V_handoff) / t4
    # ------------------------
    t4 = inputs.accel_to_cruise_time_s
    a4 = 0.0  # acceleration handoff -> cruise [m/s^2]; set below if stage 4 active
    if V_cruise > V_handoff and t4 > 0.0:
        a4 = (V_cruise - V_handoff) / t4
        N = inputs.mission_profile_steps

        E4 = 0.0
        P4_peak = 0.0
        T4_peak = 0.0

        for k in range(N):
            frac = k / (N - 1)
            V = V_handoff + frac * (V_cruise - V_handoff)

            q = 0.5 * rho * V ** 2
            D = q * (wing_S_m2 * wing_CD_cruise + S_tail_m2 * tail_CD_cruise) if q > 1e-9 else 0.0

            T_req = (D + m * a4) * inputs.propulsion_margin
            P_mech = T_req * V
            P_elec = P_mech / eta_prop 

            dt = t4 / (N - 1)
            w = 0.5 if (k == 0 or k == N - 1) else 1.0
            E4 += (P_elec * dt / 3600.0) * w

            P4_peak = max(P4_peak, P_elec)
            T4_peak = max(T4_peak, T_req)

        P4_avg = (E4 * 3600.0) / t4
        push("Accelerate to cruise speed", t4, P4_avg, P4_peak, T4_peak, notes=f"{V_handoff:.1f}→{V_cruise:.1f} m/s")
    else:
        push("Accelerate to cruise speed", 0.0, 0.0, 0.0, 0.0, notes="disabled")

    # ------------------------
    # Cruise energy accounting (NEW)
    # ------------------------
    energy_precruise_Wh = sum(s["energy_Wh"] for s in stages)

    cruise_time_s = inputs.cruise_endurance_min * 60.0
    cruise_time_source = "requested"

    energy_remaining_for_cruise_Wh = None
    battery_deficit_Wh = None

    if usable_energy_Wh is not None:
        energy_remaining_for_cruise_Wh = usable_energy_Wh - energy_precruise_Wh

        if cruise_power_elec_W > 1e-9:
            cruise_time_s = max(0.0, energy_remaining_for_cruise_Wh) * 3600.0 / cruise_power_elec_W
        else:
            cruise_time_s = 0.0

        cruise_time_source = "battery-limited"
        battery_deficit_Wh = max(0.0, -energy_remaining_for_cruise_Wh)

    # Stage 5: Cruise (NOW: battery-limited max time if usable_energy_Wh provided)
    # In level flight thrust = drag. T = P_mech/V = (P_elec * eta_prop) / V_cruise.
    P5 = max(0.0, cruise_power_elec_W)
    T5 = (cruise_power_elec_W * eta_prop / V_cruise) if (V_cruise > 1e-9 and cruise_power_elec_W > 1e-9) else 0.0
    push("Cruise", cruise_time_s, P5, P5, T5, notes=cruise_time_source)

    total_time_s = sum(s["time_s"] for s in stages)
    total_energy_Wh = sum(s["energy_Wh"] for s in stages)

    # Also compute "if requested cruise was fully flown" energy:
    requested_cruise_Wh = (cruise_power_elec_W * (inputs.cruise_endurance_min * 60.0) / 3600.0) if cruise_power_elec_W > 0 else 0.0
    mission_energy_if_requested_Wh = energy_precruise_Wh + requested_cruise_Wh

    return {
        "stages": stages,
        "handoff_speed_mps": V_handoff,
        "accel_to_cruise_a4_mps2": a4,

        "total_time_s": total_time_s,
        "total_time_min": total_time_s / 60.0,
        "total_energy_Wh": total_energy_Wh,

        "peak_elec_power_W": peak_elec_power_W,
        "peak_elec_power_per_motor_W": peak_elec_power_per_motor_W,
        "peak_thrust_total_N": peak_thrust_total_N,
        "peak_thrust_per_motor_N": peak_thrust_per_motor_N,

        # NEW battery accounting fields
        "usable_energy_Wh": usable_energy_Wh,
        "energy_precruise_Wh": energy_precruise_Wh,
        "energy_remaining_for_cruise_Wh": energy_remaining_for_cruise_Wh,
        "battery_deficit_Wh": battery_deficit_Wh,

        "cruise_time_requested_min": inputs.cruise_endurance_min,
        "cruise_time_max_min": cruise_time_s / 60.0,
        "cruise_time_source": cruise_time_source,

        "mission_energy_if_requested_Wh": mission_energy_if_requested_Wh,
    }


# ============================================================
# Prop-fuselage clearance check
# ============================================================

def _check_prop_fuselage_clearance(
    prop_diameter_in: float,
    prop_y_position_fraction: float,
    wing_span_m: float,
    fuselage_OR_m: float,
    min_clearance_m: float,
) -> Tuple[bool, float]:
    """
    Check if prop meets minimum clearance requirement from fuselage.
    
    Args:
        prop_diameter_in: Propeller diameter [in]
        prop_y_position_fraction: Fraction of half-wing span (0 = centerline, 1 = tip)
        wing_span_m: Total wing span [m]
        fuselage_OR_m: Fuselage outer radius [m]
        min_clearance_m: Minimum required clearance [m]
    
    Returns:
        Tuple of (meets_requirement, actual_clearance_m)
    """
    # Convert fraction to absolute position from centerline
    # wing_span_m is total span, so half-span from centerline to tip is wing_span_m / 2
    prop_y_position_m = prop_y_position_fraction * (wing_span_m / 2.0)
    
    prop_radius_m = (prop_diameter_in * 0.0254) / 2.0
    # Clearance is distance from prop tip (closest to fuselage) to fuselage surface
    actual_clearance_m = prop_y_position_m - prop_radius_m - fuselage_OR_m
    meets_requirement = actual_clearance_m >= min_clearance_m
    return meets_requirement, actual_clearance_m

def _required_span_extension_for_clearance(
    prop_diameter_in: float,
    prop_y_position_fraction: float,
    current_span_m: float,
    fuselage_OR_m: float,
    min_clearance_m: float,
) -> float:
    """
    Returns the REQUIRED additional TOTAL span (m) to meet clearance.
    Extension is applied symmetrically via span, so span increases by +extension_m.
    """
    prop_radius_m = (prop_diameter_in * 0.0254) / 2.0
    required_y_m = prop_radius_m + fuselage_OR_m + min_clearance_m
    required_span_m = 2.0 * required_y_m / prop_y_position_fraction

    return max(0.0, required_span_m - current_span_m)

# ============================================================
# Envelope filter (power/thrust peaks)
# ============================================================

def _filter_propulsion_db_for_envelope(
    propulsion_db: List[Dict[str, Any]],
    inputs: DesignInputs,
    W0_mass_kg: float,
    cruise_power_elec_W: float,
    V_stall_mps: float,
    wing_S_m2: float,
    wing_AR: float,
    wing_CD0: float,
    wing_CD_cruise: float,
    S_tail_m2: float = 0.0,
    tail_CD_cruise: float = 0.0,
) -> List[Dict[str, Any]]:
    filtered = []

    for combo in propulsion_db:
        if "prop_diameter_in" not in combo or "max_thrust_N" not in combo or "max_power_W" not in combo:
            continue

        prop_d_in = float(combo["prop_diameter_in"])
        max_T = float(combo["max_thrust_N"])
        max_P = float(combo["max_power_W"])

        if inputs.vtol_enabled:
            hover_req = _hover_requirements_for_prop(W0_mass_kg, prop_d_in, inputs.n_propellers, inputs)
            if max_T < hover_req["T_per_motor_req_N"]:
                continue
            if max_P < hover_req["P_hover_elec_per_motor_W"] * inputs.hover_power_margin:
                continue

        profile = estimate_tailsitter_mission_profile(
            inputs=inputs,
            W0_mass_kg=W0_mass_kg,
            prop_diameter_in=prop_d_in,
            cruise_power_elec_W=cruise_power_elec_W,
            V_stall_mps=V_stall_mps,
            wing_S_m2=wing_S_m2,
            wing_AR=wing_AR,
            wing_CD0=wing_CD0,
            wing_CD_cruise=wing_CD_cruise,
            S_tail_m2=S_tail_m2,
            tail_CD_cruise=tail_CD_cruise,
            usable_energy_Wh=None,  # envelope peaks don't need battery energy
        )

        peak_T = profile["peak_thrust_per_motor_N"]
        power_margin = inputs.hover_power_margin if inputs.vtol_enabled else inputs.propulsion_power_margin
        peak_P = profile["peak_elec_power_per_motor_W"] * power_margin

        if max_T >= peak_T and max_P >= peak_P:
            combo2 = dict(combo)
            combo2["_mission_profile_preview"] = profile
            filtered.append(combo2)

    return filtered


# ============================================================
# Selection logic
# ============================================================

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
    wing_CD_cruise: Optional[float] = None,
    S_tail_m2: float = 0.0,
    tail_CD_cruise: float = 0.0,
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:

    if W0_mass_kg <= 0.0:
        raise ValueError("W0_mass_kg must be > 0 for mission envelope sizing.")
    if V_stall_mps is None or wing_S_m2 is None or wing_AR is None or wing_CD0 is None:
        raise ValueError("Must pass V_stall_mps, wing_S_m2, wing_AR, wing_CD0 into select_propulsion_system().")
    if wing_CD_cruise is None:
        wing_CD_cruise = wing_CD0  # fallback for script-style drag

    # ---------------------------------------------------------
    # 0a) If use_hardcoded_values with hardcode_battery and hardcode_propulsion, use them and skip DB selection
    # ---------------------------------------------------------
    use_hardcoded = inputs.use_hardcoded_values
    hardcode_battery = inputs.hardcode_battery
    hardcode_propulsion = inputs.hardcode_propulsion

    if use_hardcoded and hardcode_battery is not None and hardcode_propulsion is not None:
        batt = hardcode_battery
        combo = hardcode_propulsion
        n_prop = int(inputs.n_propellers)
        Vb = float(batt["voltage_V"])
        capacity_Ah = float(batt["capacity_mAh"]) / 1000.0
        usable_energy_frac = 0.9
        usable_Wh = Vb * capacity_Ah * usable_energy_frac
        endurance_h = usable_Wh / float(cruise_power_elec_W) if cruise_power_elec_W > 1e-9 else 0.0
        endurance_min = endurance_h * 60.0
        C_at_cruise = float(cruise_power_elec_W) / (Vb * capacity_Ah) if (Vb * capacity_Ah) > 1e-9 else 0.0
        req_endurance_h = float(inputs.cruise_endurance_min) / 60.0
        meets_endurance = endurance_h >= req_endurance_h

        battery_selection = {
            "battery": batt,
            "usable_energy_Wh": usable_Wh,
            "endurance_min": endurance_min,
            "endurance_h": endurance_h,
            "C_at_cruise": C_at_cruise,
            "meets_endurance": meets_endurance,
        }

        required_thrust_total_N = float(drag_cruise_N) * float(inputs.propulsion_margin)
        required_thrust_per_motor_N = required_thrust_total_N / n_prop
        required_power_total_W = float(cruise_power_elec_W) * float(inputs.propulsion_power_margin)
        required_power_per_motor_W = required_power_total_W / n_prop
        max_T = float(combo["max_thrust_N"])
        max_P = float(combo["max_power_W"])

        propulsion_selection = {
            "combo": combo,
            "combo_name": combo.get("name", ""),
            "n_propellers": n_prop,
            "required_thrust_per_motor_N": required_thrust_per_motor_N,
            "required_power_per_motor_W": required_power_per_motor_W,
            "meets_thrust": max_T >= required_thrust_per_motor_N,
            "meets_power": max_P >= required_power_per_motor_W,
            "est_current_A_per_motor_at_cruise": required_power_per_motor_W / max(Vb, 1e-9),
        }

        span_for_clearance_m = float(tail_span_m) if (tail_span_m is not None) else 0.0
        prop_d_in = float(combo["prop_diameter_in"])

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

        profile = estimate_tailsitter_mission_profile(
            inputs=inputs,
            W0_mass_kg=W0_mass_kg,
            prop_diameter_in=prop_d_in,
            cruise_power_elec_W=cruise_power_elec_W,
            V_stall_mps=float(V_stall_mps),
            wing_S_m2=float(wing_S_m2),
            wing_AR=float(wing_AR),
            wing_CD0=float(wing_CD0),
            wing_CD_cruise=float(wing_CD_cruise),
            S_tail_m2=float(S_tail_m2),
            tail_CD_cruise=float(tail_CD_cruise),
            usable_energy_Wh=usable_Wh,
        )
        battery_selection["mission_profile"] = profile
        propulsion_selection["mission_profile"] = profile

        power_margin = inputs.hover_power_margin if inputs.vtol_enabled else inputs.propulsion_power_margin
        esc_sizing_W = max(required_power_per_motor_W, profile["peak_elec_power_per_motor_W"] * power_margin)
        esc_cont_current_A = esc_sizing_W / (Vb * inputs.esc_efficiency)
        avionics_selection = estimate_avionics_and_electrics(
            W0_mass_kg=0.0,
            n_motors=inputs.n_propellers,
            esc_cont_current_A=esc_cont_current_A,
            autopilot_class=inputs.autopilot_class,
            n_servos=inputs.n_servos,
            inputs=inputs,
        )
        propulsion_selection["esc_sizing_power_per_motor_W"] = esc_sizing_W
        propulsion_selection["esc_cont_current_A_per_motor"] = esc_cont_current_A

        return battery_selection, propulsion_selection, avionics_selection

    # ---------------------------------------------------------
    # 0) Build propulsion DB filtered for envelope peaks
    # ---------------------------------------------------------
    propulsion_db = get_propulsion_database()
    propulsion_db = _filter_propulsion_db_for_envelope(
        propulsion_db=propulsion_db,
        inputs=inputs,
        W0_mass_kg=W0_mass_kg,
        cruise_power_elec_W=cruise_power_elec_W,
        V_stall_mps=float(V_stall_mps),
        wing_S_m2=float(wing_S_m2),
        wing_AR=float(wing_AR),
        wing_CD0=float(wing_CD0),
        wing_CD_cruise=float(wing_CD_cruise),
        S_tail_m2=float(S_tail_m2),
        tail_CD_cruise=float(tail_CD_cruise),
    )

    if not propulsion_db:
        raise ValueError(
            "No propulsion combo satisfies the full flight envelope constraints.\n"
            "Try: bigger props, stronger motors, more propellers, reduce MTOW, "
            "reduce climb/transition aggressiveness, or reduce cruise speed."
        )

    # ---------------------------------------------------------
    # 1) Rank batteries by your priority:
    #    1) meet endurance
    #    2) smallest voltage
    #    3) smallest weight
    #    4) smallest size
    # ---------------------------------------------------------
    battery_db = get_battery_database()

    # Use the same defaults you used before (discharge margin + usable fraction)
    discharge_margin = 0.7
    usable_energy_frac = 0.9

    battery_candidates = rank_battery_candidates(
        required_power_W=cruise_power_elec_W,
        required_endurance_min=inputs.cruise_endurance_min,
        battery_db=battery_db,
        fuselage_ID=fuselage_ID,
        discharge_margin=discharge_margin,
        usable_energy_frac=usable_energy_frac,
    )

    if not battery_candidates:
        raise ValueError(
            "No battery fits fuselage and power constraints (with margin).\n"
            "Try: larger fuselage ID, lower cruise power, or add batteries to DB."
        )

    # Priority #1 is strict: must meet endurance time
    if not any(b["meets_endurance"] for b in battery_candidates):
        raise ValueError(
            "No battery can meet the requested endurance at current cruise electrical power.\n"
            "Reduce cruise_endurance_min, reduce cruise power, or add higher-capacity packs."
        )

    # ---------------------------------------------------------
    # 2) Joint selection loop (this enforces the full priority chain):
    #    For each battery (already sorted), pick the smallest prop diameter
    #    that meets thrust+power (and voltage window).
    # ---------------------------------------------------------
    chosen_batt = None
    chosen_prop = None

    for bsel in battery_candidates:
        Vb = float(bsel["battery"]["voltage_V"])

        prop_sel = select_propulsion(
            drag_cruise_N=drag_cruise_N,
            cruise_power_elec_W=cruise_power_elec_W,
            propulsion_db=propulsion_db,  # select_propulsion sorts by diameter internally
            n_propellers=inputs.n_propellers,
            thrust_margin=inputs.propulsion_margin,
            power_margin=inputs.propulsion_power_margin,
            battery_voltage_V=Vb,
        )

        if prop_sel is None:
            # This battery voltage window can't find a feasible combo
            continue

        chosen_batt = bsel
        chosen_prop = prop_sel
        break

    if chosen_batt is None or chosen_prop is None:
        raise ValueError(
            "Batteries that meet endurance exist, but none are compatible with any propulsion combo\n"
            "after voltage window + thrust/power constraints.\n"
            "Try: add propulsion combos for that voltage, allow higher voltage packs, or reduce margins."
        )

    battery_selection = chosen_batt
    propulsion_selection = chosen_prop

    # ---------------------------------------------------------
    # 3) Clearance check (your existing logic)
    # ---------------------------------------------------------
    combo = propulsion_selection["combo"]
    prop_d_in = float(combo["prop_diameter_in"])

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

    # ---------------------------------------------------------
    # 4) Mission profile (battery-aware) + attach to results dicts
    # ---------------------------------------------------------
    usable_Wh = float(battery_selection["usable_energy_Wh"])

    profile = estimate_tailsitter_mission_profile(
        inputs=inputs,
        W0_mass_kg=W0_mass_kg,
        prop_diameter_in=prop_d_in,
        cruise_power_elec_W=cruise_power_elec_W,
        V_stall_mps=float(V_stall_mps),
        wing_S_m2=float(wing_S_m2),
        wing_AR=float(wing_AR),
        wing_CD0=float(wing_CD0),
        wing_CD_cruise=float(wing_CD_cruise),
        S_tail_m2=float(S_tail_m2),
        tail_CD_cruise=float(tail_CD_cruise),
        usable_energy_Wh=usable_Wh,
    )

    battery_selection["mission_profile"] = profile
    propulsion_selection["mission_profile"] = profile

    # ---------------------------------------------------------
    # 5) Avionics / ESC sizing (your existing logic)
    # ---------------------------------------------------------
    batt_voltage = float(battery_selection["battery"]["voltage_V"])
    cruise_per_motor_W = float(propulsion_selection["required_power_per_motor_W"])
    peak_per_motor_W = float(profile["peak_elec_power_per_motor_W"])

    power_margin = inputs.hover_power_margin if inputs.vtol_enabled else inputs.propulsion_power_margin
    esc_sizing_W = max(cruise_per_motor_W, peak_per_motor_W * power_margin)
    esc_cont_current_A = esc_sizing_W / (batt_voltage * inputs.esc_efficiency)

    avionics_selection = estimate_avionics_and_electrics(
        W0_mass_kg=0.0,
        n_motors=inputs.n_propellers,
        esc_cont_current_A=esc_cont_current_A,
        autopilot_class=inputs.autopilot_class,
        n_servos=inputs.n_servos,
        inputs=inputs,
    )

    propulsion_selection["esc_sizing_power_per_motor_W"] = esc_sizing_W
    propulsion_selection["esc_cont_current_A_per_motor"] = esc_cont_current_A

    return battery_selection, propulsion_selection, avionics_selection

# ============================================================
# Mass & conversion to results
# ============================================================

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
            bec_mass_kg=avionics_selection.bec_mass_kg,
            autopilot_powerbrick_mass_kg=avionics_selection.autopilot_powerbrick_mass_kg,
            servos_mass_kg=avionics_selection.servos_mass_kg,
            esc_mass_total_kg=avionics_selection.esc_mass_total_kg,
            misc_electronics_mass_kg=avionics_selection.misc_electronics_mass_kg,
            camera_mass_kg=avionics_selection.camera_mass_kg,
            total_avionics_electrics_mass_kg=avionics_selection.total_avionics_electrics_mass_kg,
        )

    return battery_results, propulsion_results, avionics_results
