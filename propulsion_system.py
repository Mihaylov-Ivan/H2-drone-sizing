"""
Hybrid propulsion and mission model for a high-wing twin-boom lift-plus-cruise VTOL.

The public function names and return structure are kept compatible with script.py:
- select_propulsion_system(...)
- calculate_component_masses(...)
- convert_to_results(...)

Major changes from the tailsitter model:
- four dedicated vertical-lift motor/propeller units;
- one dedicated rear cruise motor/propeller;
- PEM fuel-cell + hydrogen storage + high-power buffer battery;
- separate lift/cruise thrust, power and ESC checks;
- mission energy is allocated between fuel cell and battery stage by stage;
- endurance is limited by both hydrogen and the usable battery SOC window.
"""

from __future__ import annotations

from typing import Optional, Dict, Any, Tuple, List
import math

from design_inputs import DesignInputs
from constants import ATMOSPHERIC
from batteries import get_battery_database
from propulsion import get_propulsion_database
from avionics import estimate_avionics_and_electrics
from results import BatteryResults, PropulsionResults, AvionicsResults


# ============================================================
# Basic rotor / propeller approximations
# ============================================================

def _disk_area_total(prop_diameter_in: float, n_propellers: int) -> float:
    d_m = float(prop_diameter_in) * 0.0254
    return max(0, int(n_propellers)) * math.pi * (0.5 * d_m) ** 2


def _power_hover_induced_W(T_N: float, rho: float, A_total: float) -> float:
    return max(T_N, 0.0) ** 1.5 / math.sqrt(2.0 * rho * max(A_total, 1e-9))


def _power_axial_climb_W(T_N: float, rho: float, A_total: float, Vc_mps: float) -> float:
    v_h = math.sqrt(max(T_N, 1e-9) / (2.0 * rho * max(A_total, 1e-9)))
    return T_N * (0.5 * Vc_mps + math.sqrt((0.5 * Vc_mps) ** 2 + v_h**2))


def _drag_parabolic_N(
    W_N: float,
    V_mps: float,
    S_m2: float,
    AR: float,
    CD0: float,
    e_oswald: float,
) -> float:
    q = 0.5 * ATMOSPHERIC.rho * max(V_mps, 0.0) ** 2
    if q <= 1e-9 or S_m2 <= 0.0:
        return 0.0
    CL = W_N / (q * S_m2)
    k = 1.0 / (math.pi * max(e_oswald, 1e-9) * max(AR, 1e-9))
    return q * S_m2 * (CD0 + k * CL**2)


def _lift_requirements_for_prop(
    W0_mass_kg: float,
    prop_diameter_in: float,
    n_lift_propellers: int,
    inputs: DesignInputs,
) -> Dict[str, float]:
    """Peak lift-system requirement, using vertical-climb acceleration as the design case."""
    n_lift = max(1, int(n_lift_propellers))
    W_N = W0_mass_kg * ATMOSPHERIC.g
    a_peak = max(float(inputs.hover_accel_mps2), float(inputs.vtol_climb_accel_mps2), 0.0)
    thrust_margin = float(getattr(inputs, "lift_thrust_margin", inputs.hover_thrust_margin))
    power_margin = float(getattr(inputs, "lift_power_margin", inputs.hover_power_margin))
    eta_lift = float(getattr(inputs, "lift_propeller_efficiency", inputs.hover_eta))

    T_total_req_N = W0_mass_kg * (ATMOSPHERIC.g + a_peak) * thrust_margin
    A_total = _disk_area_total(prop_diameter_in, n_lift)
    P_ideal_W = _power_axial_climb_W(
        T_total_req_N,
        ATMOSPHERIC.rho,
        A_total,
        max(float(inputs.vtol_climb_vmax_mps), 0.0),
    )
    P_total_req_W = P_ideal_W / max(eta_lift, 1e-9)

    return {
        "T_total_req_N": T_total_req_N,
        "T_per_motor_req_N": T_total_req_N / n_lift,
        "P_total_req_W": P_total_req_W,
        "P_per_motor_req_W": P_total_req_W / n_lift,
        "P_per_motor_with_margin_W": P_total_req_W / n_lift * power_margin,
        "disk_area_total_m2": A_total,
        "disk_loading_N_m2": T_total_req_N / max(A_total, 1e-9),
    }


# ============================================================
# Component selection helpers
# ============================================================

def _voltage_compatible(combo: Dict[str, Any], voltage_V: float) -> bool:
    return float(combo.get("voltage_min_V", -1e9)) <= voltage_V <= float(combo.get("voltage_max_V", 1e9))


def _select_buffer_battery(inputs: DesignInputs, fuselage_ID: float) -> Dict[str, Any]:
    """Use the provisional input battery, otherwise choose a near-bus-voltage pack."""
    if getattr(inputs, "hardcode_buffer_battery", None) is not None:
        return dict(inputs.hardcode_buffer_battery)

    target_V = float(inputs.dc_bus_nominal_voltage_V)
    candidates: List[Dict[str, Any]] = []
    for batt in get_battery_database():
        width = float(batt["width_m"])
        height = float(batt["height_m"])
        if fuselage_ID > 0.0 and math.hypot(width, height) > fuselage_ID:
            continue
        voltage_error = abs(float(batt["voltage_V"]) - target_V)
        candidates.append((voltage_error, float(batt["mass_kg"]), -float(batt["max_discharge_C"]), batt))

    if not candidates:
        raise ValueError("No buffer-battery candidate fits the fuselage envelope.")
    candidates.sort(key=lambda item: item[:3])
    return dict(candidates[0][3])


def _select_lift_combo(
    inputs: DesignInputs,
    voltage_V: float,
    W0_mass_kg: float,
) -> Tuple[Dict[str, Any], Dict[str, float]]:
    if getattr(inputs, "hardcode_lift_propulsion", None) is not None:
        combo = dict(inputs.hardcode_lift_propulsion)
        req = _lift_requirements_for_prop(W0_mass_kg, combo["prop_diameter_in"], inputs.n_lift_propellers, inputs)
        return combo, req

    feasible: List[Tuple[float, float, Dict[str, Any], Dict[str, float]]] = []
    for combo in get_propulsion_database():
        if not _voltage_compatible(combo, voltage_V):
            continue
        req = _lift_requirements_for_prop(W0_mass_kg, combo["prop_diameter_in"], inputs.n_lift_propellers, inputs)
        if float(combo["max_thrust_N"]) < req["T_per_motor_req_N"]:
            continue
        if float(combo["max_power_W"]) < req["P_per_motor_with_margin_W"]:
            continue
        feasible.append((float(combo["prop_diameter_in"]), float(combo["motor_mass_kg"]), dict(combo), req))

    if not feasible:
        raise ValueError("No lift motor/propeller combination satisfies VTOL thrust and power requirements.")
    feasible.sort(key=lambda item: (item[0], item[1]))
    _, _, combo, req = feasible[0]
    return combo, req


def _select_cruise_combo(
    inputs: DesignInputs,
    voltage_V: float,
    drag_cruise_N: float,
    cruise_power_elec_W: float,
) -> Tuple[Dict[str, Any], Dict[str, float]]:
    n_cruise = max(1, int(inputs.n_cruise_propellers))
    thrust_margin = float(getattr(inputs, "cruise_thrust_margin", inputs.propulsion_margin))
    power_margin = float(getattr(inputs, "cruise_power_margin", inputs.propulsion_power_margin))

    req = {
        "T_total_req_N": float(drag_cruise_N) * thrust_margin,
        "T_per_motor_req_N": float(drag_cruise_N) * thrust_margin / n_cruise,
        "P_total_req_W": float(cruise_power_elec_W) * power_margin,
        "P_per_motor_req_W": float(cruise_power_elec_W) * power_margin / n_cruise,
    }

    if getattr(inputs, "hardcode_cruise_propulsion", None) is not None:
        return dict(inputs.hardcode_cruise_propulsion), req

    feasible: List[Tuple[float, float, Dict[str, Any]]] = []
    for combo in get_propulsion_database():
        if not _voltage_compatible(combo, voltage_V):
            continue
        if float(combo["max_thrust_N"]) < req["T_per_motor_req_N"]:
            continue
        if float(combo["max_power_W"]) < req["P_per_motor_req_W"]:
            continue
        feasible.append((float(combo["prop_diameter_in"]), float(combo["motor_mass_kg"]), dict(combo)))

    if not feasible:
        raise ValueError("No cruise motor/propeller combination satisfies cruise thrust and power requirements.")
    feasible.sort(key=lambda item: (item[0], item[1]))
    return feasible[0][2], req


# ============================================================
# Lift-plus-cruise mission and hybrid energy allocation
# ============================================================

def estimate_lift_plus_cruise_hybrid_mission_profile(
    inputs: DesignInputs,
    W0_mass_kg: float,
    lift_prop_diameter_in: float,
    cruise_prop_diameter_in: float,
    cruise_power_elec_W: float,
    drag_cruise_N: float,
    V_stall_mps: float,
    wing_S_m2: float,
    wing_AR: float,
    wing_CD0: float,
    wing_CD_cruise: float,
    battery: Dict[str, Any],
) -> Dict[str, Any]:
    """
    Complete take-off-to-landing mission using separate lift and cruise propulsors.

    The fixed stages are retained around a variable-duration cruise stage. A stage-level
    power allocator draws from the fuel cell first and the battery second. The battery is
    never allowed below buffer_battery_min_soc. This avoids treating the buffer pack as
    the primary endurance energy source.
    """
    del cruise_prop_diameter_in, wing_CD_cruise  # retained for interface/reporting compatibility

    rho = ATMOSPHERIC.rho
    g = ATMOSPHERIC.g
    m = float(W0_mass_kg)
    W_N = m * g
    n_lift = max(1, int(inputs.n_lift_propellers))
    n_cruise = max(1, int(inputs.n_cruise_propellers))
    A_lift = _disk_area_total(lift_prop_diameter_in, n_lift)
    eta_lift = max(float(getattr(inputs, "lift_propeller_efficiency", inputs.hover_eta)), 1e-9)
    eta_cruise = max(float(getattr(inputs, "cruise_propeller_efficiency", inputs.propeller_efficiency)), 1e-9)
    aux_W = max(0.0, float(getattr(inputs, "continuous_auxiliary_power_W", 35.0)))

    V_handoff = max(float(inputs.transition_min_speed_mps), float(inputs.transition_stall_margin) * V_stall_mps)
    V_cruise = float(inputs.speed)

    def make_stage(
        name: str,
        time_s: float,
        lift_avg_W: float = 0.0,
        lift_peak_W: float = 0.0,
        cruise_avg_W: float = 0.0,
        cruise_peak_W: float = 0.0,
        lift_thrust_N: float = 0.0,
        cruise_thrust_N: float = 0.0,
        notes: str = "",
    ) -> Dict[str, Any]:
        avg_total = max(0.0, lift_avg_W) + max(0.0, cruise_avg_W) + aux_W
        peak_total = max(0.0, lift_peak_W) + max(0.0, cruise_peak_W) + aux_W
        return {
            "stage": name,
            "time_s": max(0.0, float(time_s)),
            "time_min": max(0.0, float(time_s)) / 60.0,
            "lift_power_avg_W": max(0.0, lift_avg_W),
            "lift_power_peak_W": max(0.0, lift_peak_W),
            "cruise_power_avg_W": max(0.0, cruise_avg_W),
            "cruise_power_peak_W": max(0.0, cruise_peak_W),
            "auxiliary_power_W": aux_W,
            "avg_elec_power_W": avg_total,
            "peak_elec_power_W": peak_total,
            "lift_thrust_total_N": max(0.0, lift_thrust_N),
            "cruise_thrust_total_N": max(0.0, cruise_thrust_N),
            "thrust_total_N": math.hypot(max(0.0, lift_thrust_N), max(0.0, cruise_thrust_N)),
            "thrust_per_motor_N": max(
                max(0.0, lift_thrust_N) / n_lift,
                max(0.0, cruise_thrust_N) / n_cruise,
            ),
            "notes": notes,
        }

    pre: List[Dict[str, Any]] = []
    post: List[Dict[str, Any]] = []

    # 1) Hover / take-off
    T_hover = W_N * float(inputs.lift_thrust_margin)
    P_hover = _power_hover_induced_W(T_hover, rho, A_lift) / eta_lift
    pre.append(make_stage(
        "VTOL takeoff / hover",
        inputs.vtol_takeoff_hover_time_s,
        lift_avg_W=P_hover,
        lift_peak_W=P_hover,
        lift_thrust_N=T_hover,
    ))

    # 2) Vertical climb
    h = max(0.0, float(inputs.vtol_climb_height_m))
    vmax = max(0.0, float(inputs.vtol_climb_vmax_mps))
    a_up = max(0.0, float(inputs.vtol_climb_accel_mps2))
    t_climb = h / vmax if vmax > 1e-9 else 0.0
    T_climb = m * (g + a_up) * float(inputs.lift_thrust_margin)
    P_climb_avg = _power_axial_climb_W(T_climb, rho, A_lift, 0.75 * vmax) / eta_lift if t_climb > 0 else 0.0
    P_climb_peak = _power_axial_climb_W(T_climb, rho, A_lift, vmax) / eta_lift if t_climb > 0 else 0.0
    pre.append(make_stage(
        "Vertical climb",
        t_climb,
        lift_avg_W=P_climb_avg,
        lift_peak_W=P_climb_peak,
        lift_thrust_N=T_climb,
        notes=f"h={h:.1f}m",
    ))

    # 3) Transition: lift rotors unload while pusher accelerates aircraft.
    t_trans = max(float(inputs.transition_time_s), 1e-6)
    a_x = V_handoff / t_trans
    avg_lift_fraction = float(getattr(inputs, "transition_average_lift_fraction", 0.55))
    avg_lift_fraction = max(0.0, min(1.0, avg_lift_fraction))
    T_lift_trans_avg = W_N * avg_lift_fraction * float(inputs.lift_thrust_margin)
    T_lift_trans_peak = W_N * float(inputs.lift_thrust_margin)
    P_lift_trans_avg = _power_hover_induced_W(T_lift_trans_avg, rho, A_lift) / eta_lift
    P_lift_trans_peak = _power_hover_induced_W(T_lift_trans_peak, rho, A_lift) / eta_lift
    D_handoff = _drag_parabolic_N(W_N, V_handoff, wing_S_m2, wing_AR, wing_CD0, inputs.e_oswald)
    F_cruise_trans_avg = m * a_x + 0.5 * D_handoff
    F_cruise_trans_peak = m * a_x + D_handoff
    P_cruise_trans_avg = F_cruise_trans_avg * (0.5 * V_handoff) / eta_cruise
    P_cruise_trans_peak = F_cruise_trans_peak * V_handoff / eta_cruise
    pre.append(make_stage(
        "Transition to wing-borne flight",
        t_trans,
        lift_avg_W=P_lift_trans_avg,
        lift_peak_W=P_lift_trans_peak,
        cruise_avg_W=P_cruise_trans_avg,
        cruise_peak_W=P_cruise_trans_peak,
        lift_thrust_N=T_lift_trans_avg,
        cruise_thrust_N=F_cruise_trans_avg,
        notes=f"handoff={V_handoff:.1f}m/s",
    ))

    # 4) Wing-borne acceleration to cruise
    t_acc = max(float(inputs.accel_to_cruise_time_s), 0.0)
    if V_cruise > V_handoff and t_acc > 1e-9:
        a4 = (V_cruise - V_handoff) / t_acc
        V_avg = 0.5 * (V_handoff + V_cruise)
        D_avg = _drag_parabolic_N(W_N, V_avg, wing_S_m2, wing_AR, wing_CD0, inputs.e_oswald)
        F_avg = D_avg + m * a4
        P_avg = F_avg * V_avg / eta_cruise
        D_peak = max(float(drag_cruise_N), D_avg)
        F_peak = D_peak + m * a4
        P_peak = F_peak * V_cruise / eta_cruise
        pre.append(make_stage(
            "Accelerate to cruise",
            t_acc,
            cruise_avg_W=P_avg,
            cruise_peak_W=P_peak,
            cruise_thrust_N=F_avg,
        ))
    else:
        a4 = 0.0
        pre.append(make_stage("Accelerate to cruise", 0.0, notes="not required"))

    # Reverse transition and landing reserve.
    include_landing = bool(getattr(inputs, "include_landing_sequence", True))
    if include_landing:
        post.append(make_stage(
            "Transition to hover",
            float(getattr(inputs, "transition_to_hover_time_s", inputs.transition_time_s)),
            lift_avg_W=P_lift_trans_avg,
            lift_peak_W=P_lift_trans_peak,
            cruise_avg_W=0.55 * P_cruise_trans_avg,
            cruise_peak_W=P_cruise_trans_peak,
            lift_thrust_N=T_lift_trans_avg,
            cruise_thrust_N=0.55 * F_cruise_trans_avg,
        ))

        descent_h = max(0.0, float(getattr(inputs, "vtol_descent_height_m", inputs.vtol_climb_height_m)))
        descent_v = max(0.0, float(getattr(inputs, "vtol_descent_vmax_mps", 2.0)))
        descent_t = descent_h / descent_v if descent_v > 1e-9 else 0.0
        descent_power_factor = max(0.4, float(getattr(inputs, "vtol_descent_power_factor", 0.75)))
        T_descent = W_N * float(inputs.lift_thrust_margin)
        P_descent = P_hover * descent_power_factor
        post.append(make_stage(
            "Vertical descent",
            descent_t,
            lift_avg_W=P_descent,
            lift_peak_W=P_hover,
            lift_thrust_N=T_descent,
            notes=f"h={descent_h:.1f}m",
        ))
        post.append(make_stage(
            "Landing hover",
            float(getattr(inputs, "vtol_landing_hover_time_s", inputs.vtol_takeoff_hover_time_s)),
            lift_avg_W=P_hover,
            lift_peak_W=P_hover,
            lift_thrust_N=T_hover,
        ))

    cruise_load_W = max(0.0, float(cruise_power_elec_W)) + aux_W
    cruise_stage_template = make_stage(
        "Cruise",
        0.0,
        cruise_avg_W=max(0.0, float(cruise_power_elec_W)),
        cruise_peak_W=max(0.0, float(cruise_power_elec_W)),
        cruise_thrust_N=max(0.0, float(drag_cruise_N)),
    )

    # Energy-store definitions
    batt_V = float(battery["voltage_V"])
    batt_Ah = float(battery["capacity_mAh"]) / 1000.0
    batt_nominal_Wh = batt_V * batt_Ah
    initial_soc = max(0.0, min(1.0, float(getattr(inputs, "buffer_battery_initial_soc", 1.0))))
    min_soc = max(0.0, min(initial_soc, float(inputs.buffer_battery_min_soc)))
    usable_fraction_cap = max(0.0, min(1.0, float(inputs.buffer_battery_usable_fraction)))
    batt_available_Wh = min(batt_nominal_Wh * (initial_soc - min_soc), batt_nominal_Wh * usable_fraction_cap)
    batt_max_power_W = batt_V * batt_Ah * float(battery["max_discharge_C"]) * float(getattr(inputs, "buffer_battery_discharge_margin", 0.80))

    fc_enabled = bool(inputs.fuel_cell_enabled and inputs.hydrogen_storage_enabled)
    fc_max_W = (
        float(inputs.fuel_cell_rated_net_power_W)
        * float(inputs.fuel_cell_max_continuous_fraction)
        * float(inputs.fuel_cell_dc_dc_efficiency)
        * float(inputs.power_management_efficiency)
        if fc_enabled else 0.0
    )
    fc_min_W = float(inputs.fuel_cell_rated_net_power_W) * float(inputs.fuel_cell_min_stable_fraction) if fc_enabled else 0.0
    fc_energy_initial_Wh = float(inputs.fuel_cell_usable_electrical_energy_Wh) if fc_enabled else 0.0
    fc_ready = bool(getattr(inputs, "fuel_cell_ready_at_takeoff", True))

    def simulate(cruise_time_s: float, keep_records: bool = False) -> Dict[str, Any]:
        batt_remaining = batt_available_Wh
        fc_remaining = fc_energy_initial_Wh
        feasible = True
        records: List[Dict[str, Any]] = []
        max_batt_power = 0.0
        max_batt_C = 0.0
        max_total_power = 0.0
        max_lift_per_motor = 0.0
        max_cruise_per_motor = 0.0
        max_thrust_total = 0.0
        max_thrust_per_motor = 0.0
        total_load_energy = 0.0
        total_fc_energy = 0.0
        total_batt_energy = 0.0

        cruise_stage = dict(cruise_stage_template)
        cruise_stage["time_s"] = max(0.0, cruise_time_s)
        cruise_stage["time_min"] = max(0.0, cruise_time_s) / 60.0
        stages = [dict(s) for s in pre] + [cruise_stage] + [dict(s) for s in post]

        for idx, stage in enumerate(stages):
            dt = float(stage["time_s"])
            load_avg = float(stage["avg_elec_power_W"])
            load_peak = float(stage["peak_elec_power_W"])
            load_energy = load_avg * dt / 3600.0
            total_load_energy += load_energy

            fc_power = 0.0
            if fc_ready and fc_remaining > 1e-12 and dt > 0.0 and load_avg >= fc_min_W:
                fc_energy_limited_power = fc_remaining * 3600.0 / dt
                fc_power = min(load_avg, fc_max_W, fc_energy_limited_power)
            fc_energy = fc_power * dt / 3600.0
            batt_power = max(0.0, load_avg - fc_power)
            batt_energy = batt_power * dt / 3600.0

            # Peak current is conservatively based on the stage-average FC contribution.
            batt_peak_power = max(0.0, load_peak - fc_power)
            max_batt_power = max(max_batt_power, batt_peak_power)
            max_batt_C = max(max_batt_C, batt_peak_power / max(batt_V * batt_Ah, 1e-9))
            max_total_power = max(max_total_power, load_peak)
            max_lift_per_motor = max(max_lift_per_motor, float(stage["lift_power_peak_W"]) / n_lift)
            max_cruise_per_motor = max(max_cruise_per_motor, float(stage["cruise_power_peak_W"]) / n_cruise)
            max_thrust_total = max(max_thrust_total, float(stage["thrust_total_N"]))
            max_thrust_per_motor = max(max_thrust_per_motor, float(stage["thrust_per_motor_N"]))

            if batt_peak_power > batt_max_power_W + 1e-9:
                feasible = False
                stage_reason = "battery peak-power limit"
            elif batt_energy > batt_remaining + 1e-9:
                feasible = False
                stage_reason = "battery SOC limit"
            else:
                stage_reason = ""

            fc_remaining = max(0.0, fc_remaining - fc_energy)
            batt_remaining = max(0.0, batt_remaining - batt_energy)
            total_fc_energy += fc_energy
            total_batt_energy += batt_energy

            stage["energy_Wh"] = load_energy
            stage["fuel_cell_power_W"] = fc_power
            stage["fuel_cell_energy_Wh"] = fc_energy
            stage["battery_power_W"] = batt_power
            stage["battery_peak_power_W"] = batt_peak_power
            stage["battery_energy_Wh"] = batt_energy
            stage["battery_soc_end"] = min_soc + batt_remaining / max(batt_nominal_Wh, 1e-9)
            stage["hydrogen_electrical_energy_remaining_Wh"] = fc_remaining
            if stage_reason:
                stage["notes"] = (stage.get("notes", "") + " " + stage_reason).strip()
            if keep_records:
                records.append(stage)

        final_soc = min_soc + batt_remaining / max(batt_nominal_Wh, 1e-9)
        h2_used_fraction = total_fc_energy / max(fc_energy_initial_Wh, 1e-9) if fc_energy_initial_Wh > 0 else 0.0
        h2_used_kg = h2_used_fraction * float(inputs.hydrogen_usable_mass_kg)
        return {
            "feasible": feasible,
            "stages": records,
            "battery_remaining_Wh": batt_remaining,
            "battery_energy_used_Wh": total_batt_energy,
            "battery_final_soc": max(min_soc, min(1.0, final_soc)),
            "fuel_cell_energy_remaining_Wh": fc_remaining,
            "fuel_cell_energy_used_Wh": total_fc_energy,
            "hydrogen_used_kg": h2_used_kg,
            "total_load_energy_Wh": total_load_energy,
            "max_battery_power_W": max_batt_power,
            "max_battery_C": max_batt_C,
            "peak_elec_power_W": max_total_power,
            "peak_lift_power_per_motor_W": max_lift_per_motor,
            "peak_cruise_power_per_motor_W": max_cruise_per_motor,
            "peak_elec_power_per_motor_W": max(max_lift_per_motor, max_cruise_per_motor),
            "peak_thrust_total_N": max_thrust_total,
            "peak_thrust_per_motor_N": max_thrust_per_motor,
        }

    requested_cruise_s = max(0.0, float(inputs.cruise_endurance_min) * 60.0)
    requested_sim = simulate(requested_cruise_s, keep_records=False)

    # Maximum feasible cruise duration, while preserving all post-cruise VTOL stages.
    total_store_Wh = batt_available_Wh + fc_energy_initial_Wh
    high = min(12.0 * 3600.0, 1.20 * total_store_Wh * 3600.0 / max(cruise_load_W, 1e-9) + 600.0)
    low = 0.0
    if not simulate(0.0, keep_records=False)["feasible"]:
        max_cruise_s = 0.0
    else:
        for _ in range(55):
            mid = 0.5 * (low + high)
            if simulate(mid, keep_records=False)["feasible"]:
                low = mid
            else:
                high = mid
        max_cruise_s = low

    actual_cruise_s = requested_cruise_s if requested_sim["feasible"] else max_cruise_s
    final = simulate(actual_cruise_s, keep_records=True)
    stages = final["stages"]

    pre_energy = sum(s["energy_Wh"] for s in stages if s["stage"] not in {"Cruise", "Transition to hover", "Vertical descent", "Landing hover"})
    post_energy = sum(s["energy_Wh"] for s in stages if s["stage"] in {"Transition to hover", "Vertical descent", "Landing hover"})
    total_time_s = sum(s["time_s"] for s in stages)
    cruise_source = "requested target feasible" if requested_sim["feasible"] else "hybrid-energy-limited"

    return {
        "architecture": "PEM fuel-cell + buffer battery; separate lift and cruise propulsion",
        "stages": stages,
        "handoff_speed_mps": V_handoff,
        "accel_to_cruise_a4_mps2": a4,
        "total_time_s": total_time_s,
        "total_time_min": total_time_s / 60.0,
        "total_energy_Wh": final["total_load_energy_Wh"],
        "peak_elec_power_W": final["peak_elec_power_W"],
        "peak_elec_power_per_motor_W": final["peak_elec_power_per_motor_W"],
        "peak_lift_power_per_motor_W": final["peak_lift_power_per_motor_W"],
        "peak_cruise_power_per_motor_W": final["peak_cruise_power_per_motor_W"],
        "peak_thrust_total_N": final["peak_thrust_total_N"],
        "peak_thrust_per_motor_N": final["peak_thrust_per_motor_N"],
        "usable_energy_Wh": batt_available_Wh + fc_energy_initial_Wh,
        "battery_usable_energy_Wh": batt_available_Wh,
        "fuel_cell_usable_energy_Wh": fc_energy_initial_Wh,
        "energy_precruise_Wh": pre_energy,
        "energy_postcruise_Wh": post_energy,
        "energy_remaining_for_cruise_Wh": max(0.0, batt_available_Wh + fc_energy_initial_Wh - pre_energy - post_energy),
        "battery_deficit_Wh": 0.0 if final["feasible"] else max(0.0, -final["battery_remaining_Wh"]),
        "cruise_time_requested_min": requested_cruise_s / 60.0,
        "cruise_time_max_min": max_cruise_s / 60.0,
        "cruise_time_flown_min": actual_cruise_s / 60.0,
        "cruise_time_source": cruise_source,
        "mission_energy_if_requested_Wh": requested_sim["total_load_energy_Wh"],
        "meets_requested_endurance": bool(requested_sim["feasible"]),
        "battery_nominal_energy_Wh": batt_nominal_Wh,
        "battery_initial_soc": initial_soc,
        "battery_min_soc": min_soc,
        "battery_final_soc": final["battery_final_soc"],
        "battery_energy_used_Wh": final["battery_energy_used_Wh"],
        "battery_peak_power_W": final["max_battery_power_W"],
        "battery_peak_C": final["max_battery_C"],
        "battery_max_power_W": batt_max_power_W,
        "fuel_cell_max_bus_power_W": fc_max_W,
        "fuel_cell_energy_used_Wh": final["fuel_cell_energy_used_Wh"],
        "hydrogen_used_kg": final["hydrogen_used_kg"],
        "hydrogen_remaining_kg": max(0.0, float(inputs.hydrogen_usable_mass_kg) - final["hydrogen_used_kg"]),
    }


# Backwards-compatible name retained for any external call sites.
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
    usable_energy_Wh: Optional[float] = None,
) -> Dict[str, Any]:
    del S_tail_m2, tail_CD_cruise, usable_energy_Wh
    batt = dict(inputs.hardcode_buffer_battery or inputs.hardcode_battery)
    drag = cruise_power_elec_W * float(inputs.cruise_propeller_efficiency) / max(float(inputs.speed), 1e-9)
    return estimate_lift_plus_cruise_hybrid_mission_profile(
        inputs=inputs,
        W0_mass_kg=W0_mass_kg,
        lift_prop_diameter_in=prop_diameter_in,
        cruise_prop_diameter_in=prop_diameter_in,
        cruise_power_elec_W=cruise_power_elec_W,
        drag_cruise_N=drag,
        V_stall_mps=V_stall_mps,
        wing_S_m2=wing_S_m2,
        wing_AR=wing_AR,
        wing_CD0=wing_CD0,
        wing_CD_cruise=wing_CD_cruise,
        battery=batt,
    )


# ============================================================
# Main selection function, same signature as the original model
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
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]], Optional[Any]]:
    del cruise_power_W, battery_selection, tail_span_m, S_tail_m2, tail_CD_cruise

    if W0_mass_kg <= 0.0:
        raise ValueError("W0_mass_kg must be > 0 for lift-plus-cruise propulsion sizing.")
    if V_stall_mps is None or wing_S_m2 is None or wing_AR is None or wing_CD0 is None:
        raise ValueError("V_stall_mps, wing_S_m2, wing_AR and wing_CD0 are required.")
    if wing_CD_cruise is None:
        wing_CD_cruise = wing_CD0

    battery = _select_buffer_battery(inputs, fuselage_ID)
    bus_voltage = float(battery["voltage_V"])
    lift_combo, lift_req = _select_lift_combo(inputs, bus_voltage, W0_mass_kg)
    cruise_combo, cruise_req = _select_cruise_combo(inputs, bus_voltage, drag_cruise_N, cruise_power_elec_W)

    if not _voltage_compatible(lift_combo, bus_voltage):
        raise ValueError(f"Lift propulsion is not compatible with the {bus_voltage:.1f} V buffer battery.")
    if not _voltage_compatible(cruise_combo, bus_voltage):
        raise ValueError(f"Cruise propulsion is not compatible with the {bus_voltage:.1f} V buffer battery.")

    lift_meets_thrust = float(lift_combo["max_thrust_N"]) >= lift_req["T_per_motor_req_N"]
    lift_meets_power = float(lift_combo["max_power_W"]) >= lift_req["P_per_motor_with_margin_W"]
    cruise_meets_thrust = float(cruise_combo["max_thrust_N"]) >= cruise_req["T_per_motor_req_N"]
    cruise_meets_power = float(cruise_combo["max_power_W"]) >= cruise_req["P_per_motor_req_W"]

    if not (lift_meets_thrust and lift_meets_power):
        raise ValueError(
            "Selected lift propulsion is undersized: "
            f"required {lift_req['T_per_motor_req_N']:.1f} N and "
            f"{lift_req['P_per_motor_with_margin_W']:.0f} W per motor."
        )
    if not (cruise_meets_thrust and cruise_meets_power):
        raise ValueError(
            "Selected cruise propulsion is undersized: "
            f"required {cruise_req['T_per_motor_req_N']:.1f} N and "
            f"{cruise_req['P_per_motor_req_W']:.0f} W per motor."
        )

    profile = estimate_lift_plus_cruise_hybrid_mission_profile(
        inputs=inputs,
        W0_mass_kg=W0_mass_kg,
        lift_prop_diameter_in=float(lift_combo["prop_diameter_in"]),
        cruise_prop_diameter_in=float(cruise_combo["prop_diameter_in"]),
        cruise_power_elec_W=float(cruise_power_elec_W),
        drag_cruise_N=float(drag_cruise_N),
        V_stall_mps=float(V_stall_mps),
        wing_S_m2=float(wing_S_m2),
        wing_AR=float(wing_AR),
        wing_CD0=float(wing_CD0),
        wing_CD_cruise=float(wing_CD_cruise),
        battery=battery,
    )

    capacity_Ah = float(battery["capacity_mAh"]) / 1000.0
    nominal_Wh = bus_voltage * capacity_Ah
    battery_selection_out = {
        "battery": battery,
        "nominal_energy_Wh": nominal_Wh,
        "usable_energy_Wh": profile["battery_usable_energy_Wh"],
        "endurance_min": profile["cruise_time_max_min"],
        "endurance_h": profile["cruise_time_max_min"] / 60.0,
        "C_at_cruise": max(0.0, cruise_power_elec_W - profile["fuel_cell_max_bus_power_W"]) / max(bus_voltage * capacity_Ah, 1e-9),
        "meets_endurance": profile["meets_requested_endurance"],
        "mission_profile": profile,
        "fuel_cell": {
            "name": inputs.fuel_cell_name,
            "rated_net_power_W": inputs.fuel_cell_rated_net_power_W,
            "max_bus_power_W": profile["fuel_cell_max_bus_power_W"],
            "system_mass_kg": inputs.fuel_cell_system_mass_kg,
            "usable_electrical_energy_Wh": inputs.fuel_cell_usable_electrical_energy_Wh,
        },
        "hydrogen_storage": {
            "name": inputs.hydrogen_storage_name,
            "pressure_bar": inputs.hydrogen_storage_pressure_bar,
            "nominal_hydrogen_mass_kg": inputs.hydrogen_mass_nominal_kg,
            "usable_hydrogen_mass_kg": inputs.hydrogen_usable_mass_kg,
            "tank_mass_kg": inputs.hydrogen_tank_mass_kg,
            "regulator_and_valves_mass_kg": inputs.hydrogen_regulator_and_valves_mass_kg,
        },
    }

    lift_esc_W = max(lift_req["P_per_motor_with_margin_W"], profile["peak_lift_power_per_motor_W"] * float(inputs.lift_power_margin))
    cruise_esc_W = max(cruise_req["P_per_motor_req_W"], profile["peak_cruise_power_per_motor_W"] * float(inputs.cruise_power_margin))
    lift_esc_A = lift_esc_W / max(bus_voltage * inputs.esc_efficiency, 1e-9)
    cruise_esc_A = cruise_esc_W / max(bus_voltage * inputs.esc_efficiency, 1e-9)

    propulsion_selection = {
        "architecture": "separate_lift_and_cruise",
        "lift": {
            "combo": lift_combo,
            "n_propellers": int(inputs.n_lift_propellers),
            "required_thrust_per_motor_N": lift_req["T_per_motor_req_N"],
            "required_power_per_motor_W": lift_req["P_per_motor_with_margin_W"],
            "meets_thrust": lift_meets_thrust,
            "meets_power": lift_meets_power,
            "esc_sizing_power_per_motor_W": lift_esc_W,
            "esc_cont_current_A_per_motor": lift_esc_A,
        },
        "cruise": {
            "combo": cruise_combo,
            "n_propellers": int(inputs.n_cruise_propellers),
            "required_thrust_per_motor_N": cruise_req["T_per_motor_req_N"],
            "required_power_per_motor_W": cruise_req["P_per_motor_req_W"],
            "meets_thrust": cruise_meets_thrust,
            "meets_power": cruise_meets_power,
            "esc_sizing_power_per_motor_W": cruise_esc_W,
            "esc_cont_current_A_per_motor": cruise_esc_A,
            "est_current_A_per_motor_at_cruise": cruise_power_elec_W / max(bus_voltage * inputs.esc_efficiency * inputs.n_cruise_propellers, 1e-9),
        },
        "mission_profile": profile,
        # Legacy aliases intentionally represent the cruise propulsor. They keep reporting
        # and other modules import-compatible until those modules are updated.
        "combo": cruise_combo,
        "combo_name": cruise_combo.get("name", ""),
        "n_propellers": int(inputs.n_cruise_propellers),
        "required_thrust_per_motor_N": cruise_req["T_per_motor_req_N"],
        "required_power_per_motor_W": cruise_req["P_per_motor_req_W"],
        "meets_thrust": cruise_meets_thrust,
        "meets_power": cruise_meets_power,
        "est_current_A_per_motor_at_cruise": cruise_power_elec_W / max(bus_voltage * inputs.esc_efficiency * inputs.n_cruise_propellers, 1e-9),
        "esc_sizing_power_per_motor_W": cruise_esc_W,
        "esc_cont_current_A_per_motor": cruise_esc_A,
    }

    avionics_selection = estimate_avionics_and_electrics(
        W0_mass_kg=W0_mass_kg,
        n_motors=int(inputs.n_motors),
        esc_cont_current_A=max(lift_esc_A, cruise_esc_A),
        autopilot_class=inputs.autopilot_class,
        n_servos=inputs.n_servos,
        inputs=inputs,
        n_lift_motors=int(inputs.n_lift_propellers),
        n_cruise_motors=int(inputs.n_cruise_propellers),
        lift_esc_cont_current_A=lift_esc_A,
        cruise_esc_cont_current_A=cruise_esc_A,
    )

    return battery_selection_out, propulsion_selection, avionics_selection


# ============================================================
# Mass aggregation and conversion to result dataclasses
# ============================================================

# ============================================================
# Detailed component mass breakdown for longitudinal CG
# ============================================================

def _split_lift_propulsor_counts(inputs: DesignInputs) -> Tuple[int, int]:
    """Return the number of front and rear lift propulsors from their x coordinates."""
    n = int(inputs.n_lift_propellers)
    xs = [float(x) for x in inputs.lift_prop_x_positions_m]
    if n <= 1:
        return n, 0
    xmin, xmax = min(xs), max(xs)
    if abs(xmax - xmin) < 1e-9:
        n_front = n // 2
    else:
        split_x = 0.5 * (xmin + xmax)
        n_front = sum(1 for x in xs if x <= split_x)
        if n_front <= 0 or n_front >= n:
            n_front = n // 2
    return n_front, n - n_front


def _scale_mass_group(
    values: Dict[str, float],
    keys: Tuple[str, ...],
    target_total_kg: Optional[float],
) -> float:
    """Scale a group proportionally to a legacy aggregate hardcode and return the factor."""
    if target_total_kg is None:
        return 1.0
    target = max(0.0, float(target_total_kg))
    current = sum(max(0.0, float(values.get(k, 0.0))) for k in keys)
    if current <= 1e-12:
        if keys:
            values[keys[0]] = target
        return 1.0
    factor = target / current
    for key in keys:
        values[key] = max(0.0, float(values.get(key, 0.0))) * factor
    return factor


def calculate_component_mass_breakdown(
    battery_selection: Optional[Dict[str, Any]],
    propulsion_selection: Optional[Dict[str, Any]],
    avionics_selection: Optional[Any],
    inputs: DesignInputs,
) -> Dict[str, float]:
    """
    Detailed mass buckets used by the longitudinal CG model.

    The legacy aggregate hardcodes remain supported. When enabled, the relevant
    detailed masses are scaled proportionally so total mass and CG accounting stay consistent.
    """
    out: Dict[str, float] = {
        "buffer_battery_kg": 0.0,
        "fuel_cell_system_kg": 0.0,
        "hydrogen_tank_kg": 0.0,
        "hydrogen_regulator_kg": 0.0,
        "hydrogen_takeoff_kg": 0.0,
        "hydrogen_end_kg": 0.0,
        "lift_front_propulsion_kg": 0.0,
        "lift_rear_propulsion_kg": 0.0,
        "cruise_propulsion_kg": 0.0,
        "lift_front_esc_kg": 0.0,
        "lift_rear_esc_kg": 0.0,
        "cruise_esc_kg": 0.0,
        "avionics_non_esc_kg": 0.0,
        "n_lift_front": 0.0,
        "n_lift_rear": 0.0,
        "avionics_scale_factor": 1.0,
    }

    if battery_selection is not None:
        out["buffer_battery_kg"] = float(battery_selection["battery"]["mass_kg"])
        out["fuel_cell_system_kg"] = float(inputs.fuel_cell_system_mass_kg)
        out["hydrogen_tank_kg"] = float(inputs.hydrogen_tank_mass_kg)
        out["hydrogen_regulator_kg"] = float(inputs.hydrogen_regulator_and_valves_mass_kg)
        h2_takeoff = float(inputs.hydrogen_mass_nominal_kg)
        profile = battery_selection.get("mission_profile", {})
        h2_used = max(0.0, float(profile.get("hydrogen_used_kg", 0.0)))
        out["hydrogen_takeoff_kg"] = h2_takeoff
        out["hydrogen_end_kg"] = max(0.0, h2_takeoff - h2_used)

    if propulsion_selection is not None:
        lift = propulsion_selection["lift"]
        cruise = propulsion_selection["cruise"]
        lift_combo = lift["combo"]
        cruise_combo = cruise["combo"]
        n_front, n_rear = _split_lift_propulsor_counts(inputs)
        lift_unit_mass = float(lift_combo["motor_mass_kg"]) + float(lift_combo["prop_mass_kg"])
        cruise_unit_mass = float(cruise_combo["motor_mass_kg"]) + float(cruise_combo["prop_mass_kg"])
        out["n_lift_front"] = float(n_front)
        out["n_lift_rear"] = float(n_rear)
        out["lift_front_propulsion_kg"] = n_front * lift_unit_mass
        out["lift_rear_propulsion_kg"] = n_rear * lift_unit_mass
        out["cruise_propulsion_kg"] = int(cruise["n_propellers"]) * cruise_unit_mass

    if avionics_selection is not None:
        n_front = int(out["n_lift_front"])
        n_rear = int(out["n_lift_rear"])
        n_lift = max(1, n_front + n_rear)
        lift_esc_total = float(getattr(avionics_selection, "lift_esc_mass_total_kg", 0.0))
        cruise_esc_total = float(getattr(avionics_selection, "cruise_esc_mass_total_kg", 0.0))
        out["lift_front_esc_kg"] = lift_esc_total * n_front / n_lift
        out["lift_rear_esc_kg"] = lift_esc_total * n_rear / n_lift
        out["cruise_esc_kg"] = cruise_esc_total
        out["avionics_non_esc_kg"] = max(
            0.0,
            float(avionics_selection.total_avionics_electrics_mass_kg)
            - lift_esc_total
            - cruise_esc_total,
        )

    energy_keys = (
        "buffer_battery_kg",
        "fuel_cell_system_kg",
        "hydrogen_tank_kg",
        "hydrogen_regulator_kg",
        "hydrogen_takeoff_kg",
    )
    propulsion_keys = (
        "lift_front_propulsion_kg",
        "lift_rear_propulsion_kg",
        "cruise_propulsion_kg",
    )
    avionics_keys = (
        "avionics_non_esc_kg",
        "lift_front_esc_kg",
        "lift_rear_esc_kg",
        "cruise_esc_kg",
    )

    if bool(getattr(inputs, "use_hardcoded_values", False)):
        energy_factor = _scale_mass_group(
            out, energy_keys, getattr(inputs, "hardcode_battery_weight_kg", None)
        )
        _scale_mass_group(
            out, propulsion_keys, getattr(inputs, "hardcode_propulsion_weight_kg", None)
        )
        avionics_factor = _scale_mass_group(
            out, avionics_keys, getattr(inputs, "hardcode_avionics_weight_kg", None)
        )
        out["avionics_scale_factor"] = avionics_factor
        # Preserve the mission fuel fraction after an aggregate energy-system mass override.
        nominal_unscaled = float(inputs.hydrogen_mass_nominal_kg)
        remaining_fraction = (
            out["hydrogen_end_kg"] / max(nominal_unscaled, 1e-12)
            if nominal_unscaled > 0.0 else 0.0
        )
        out["hydrogen_end_kg"] = out["hydrogen_takeoff_kg"] * max(0.0, min(1.0, remaining_fraction))
    else:
        out["avionics_scale_factor"] = 1.0

    out["energy_system_total_kg"] = sum(out[k] for k in energy_keys)
    out["propulsion_total_kg"] = sum(out[k] for k in propulsion_keys)
    out["avionics_total_kg"] = sum(out[k] for k in avionics_keys)
    return out


def calculate_component_masses(
    battery_selection: Optional[Dict[str, Any]],
    propulsion_selection: Optional[Dict[str, Any]],
    avionics_selection: Optional[Any],
    inputs: DesignInputs,
) -> Tuple[float, float, float]:
    """Backward-compatible aggregate totals derived from the detailed mass breakdown."""
    breakdown = calculate_component_mass_breakdown(
        battery_selection,
        propulsion_selection,
        avionics_selection,
        inputs,
    )
    return (
        float(breakdown["energy_system_total_kg"]),
        float(breakdown["propulsion_total_kg"]),
        float(breakdown["avionics_total_kg"]),
    )


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
        profile = battery_selection["mission_profile"]
        fc = battery_selection["fuel_cell"]
        h2 = battery_selection["hydrogen_storage"]
        total_energy_system_mass = (
            float(batt["mass_kg"])
            + float(fc["system_mass_kg"])
            + float(h2["tank_mass_kg"])
            + float(h2["regulator_and_valves_mass_kg"])
            + float(h2["nominal_hydrogen_mass_kg"])
        )
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
            nominal_energy_Wh=battery_selection["nominal_energy_Wh"],
            initial_soc=profile["battery_initial_soc"],
            minimum_soc=profile["battery_min_soc"],
            final_soc=profile["battery_final_soc"],
            peak_power_W=profile["battery_peak_power_W"],
            peak_C=profile["battery_peak_C"],
            fuel_cell_name=fc["name"],
            fuel_cell_rated_power_W=fc["rated_net_power_W"],
            fuel_cell_max_bus_power_W=fc["max_bus_power_W"],
            fuel_cell_system_mass_kg=fc["system_mass_kg"],
            fuel_cell_usable_energy_Wh=fc["usable_electrical_energy_Wh"],
            hydrogen_storage_name=h2["name"],
            hydrogen_pressure_bar=h2["pressure_bar"],
            hydrogen_nominal_mass_kg=h2["nominal_hydrogen_mass_kg"],
            hydrogen_usable_mass_kg=h2["usable_hydrogen_mass_kg"],
            hydrogen_used_kg=profile["hydrogen_used_kg"],
            hydrogen_tank_mass_kg=h2["tank_mass_kg"],
            hydrogen_regulator_mass_kg=h2["regulator_and_valves_mass_kg"],
            total_energy_system_mass_kg=total_energy_system_mass,
        )
        battery_results.mission_profile = profile

    if propulsion_selection is not None:
        lift = propulsion_selection["lift"]
        cruise = propulsion_selection["cruise"]
        lc = lift["combo"]
        cc = cruise["combo"]
        n_lift = int(lift["n_propellers"])
        n_cruise = int(cruise["n_propellers"])
        total_motor_mass = n_lift * float(lc["motor_mass_kg"]) + n_cruise * float(cc["motor_mass_kg"])
        total_prop_mass = n_lift * float(lc["prop_mass_kg"]) + n_cruise * float(cc["prop_mass_kg"])

        propulsion_results = PropulsionResults(
            # Legacy primary fields represent cruise propulsion.
            n_propellers=n_cruise,
            name=cc["name"],
            prop_diameter_in=cc["prop_diameter_in"],
            prop_pitch_in=cc["prop_pitch_in"],
            motor_kv=cc["motor_kv"],
            motor_mass_kg=cc["motor_mass_kg"],
            prop_mass_kg=cc["prop_mass_kg"],
            total_motor_mass_kg=total_motor_mass,
            total_prop_mass_kg=total_prop_mass,
            total_propulsion_mass_kg=total_motor_mass + total_prop_mass,
            max_thrust_per_motor_N=cc["max_thrust_N"],
            required_thrust_per_motor_N=cruise["required_thrust_per_motor_N"],
            max_power_per_motor_W=cc["max_power_W"],
            required_power_per_motor_W=cruise["required_power_per_motor_W"],
            meets_thrust_req=cruise["meets_thrust"],
            meets_power_req=cruise["meets_power"],
            est_current_A_per_motor_at_cruise=cruise.get("est_current_A_per_motor_at_cruise"),
            architecture="separate_lift_and_cruise",
            n_lift_propellers=n_lift,
            lift_name=lc["name"],
            lift_prop_diameter_in=lc["prop_diameter_in"],
            lift_prop_pitch_in=lc["prop_pitch_in"],
            lift_motor_kv=lc["motor_kv"],
            lift_motor_mass_kg=lc["motor_mass_kg"],
            lift_prop_mass_kg=lc["prop_mass_kg"],
            lift_max_thrust_per_motor_N=lc["max_thrust_N"],
            lift_required_thrust_per_motor_N=lift["required_thrust_per_motor_N"],
            lift_max_power_per_motor_W=lc["max_power_W"],
            lift_required_power_per_motor_W=lift["required_power_per_motor_W"],
            lift_meets_thrust_req=lift["meets_thrust"],
            lift_meets_power_req=lift["meets_power"],
            lift_esc_sizing_power_per_motor_W=lift["esc_sizing_power_per_motor_W"],
            lift_esc_cont_current_A_per_motor=lift["esc_cont_current_A_per_motor"],
            n_cruise_propellers=n_cruise,
            cruise_name=cc["name"],
            cruise_prop_diameter_in=cc["prop_diameter_in"],
            cruise_prop_pitch_in=cc["prop_pitch_in"],
            cruise_motor_kv=cc["motor_kv"],
            cruise_motor_mass_kg=cc["motor_mass_kg"],
            cruise_prop_mass_kg=cc["prop_mass_kg"],
            cruise_max_thrust_per_motor_N=cc["max_thrust_N"],
            cruise_required_thrust_per_motor_N=cruise["required_thrust_per_motor_N"],
            cruise_max_power_per_motor_W=cc["max_power_W"],
            cruise_required_power_per_motor_W=cruise["required_power_per_motor_W"],
            cruise_meets_thrust_req=cruise["meets_thrust"],
            cruise_meets_power_req=cruise["meets_power"],
            cruise_esc_sizing_power_per_motor_W=cruise["esc_sizing_power_per_motor_W"],
            cruise_esc_cont_current_A_per_motor=cruise["esc_cont_current_A_per_motor"],
        )
        propulsion_results.mission_profile = propulsion_selection["mission_profile"]
        propulsion_results.esc_sizing_power_per_motor_W = cruise["esc_sizing_power_per_motor_W"]
        propulsion_results.esc_cont_current_A_per_motor = cruise["esc_cont_current_A_per_motor"]

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
            lift_esc_mass_total_kg=getattr(avionics_selection, "lift_esc_mass_total_kg", 0.0),
            cruise_esc_mass_total_kg=getattr(avionics_selection, "cruise_esc_mass_total_kg", 0.0),
            lift_esc_cont_current_A=getattr(avionics_selection, "lift_esc_cont_current_A", 0.0),
            cruise_esc_cont_current_A=getattr(avionics_selection, "cruise_esc_cont_current_A", 0.0),
        )

    return battery_results, propulsion_results, avionics_results
