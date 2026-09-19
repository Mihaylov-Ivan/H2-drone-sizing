"""Mission/transition complete mission construction and Phase-2 EMS coupling."""
from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

from .config import load_hybrid_power_config
from .hybrid_power import HybridPowerSystem, PowerDemandPoint
from .aero_propulsion import _operational_climb_power_W, _operational_hover_power_W, cruise_drag
from .transition import TransitionResult, simulate_forward_transition, simulate_reverse_transition


def _append_constant(profile: List[PowerDemandPoint], t: float, duration_s: float, power_W: float, stage: str) -> float:
    duration = max(0.0, float(duration_s))
    if duration <= 0.0:
        return t
    if not profile or abs(profile[-1].time_s - t) > 1e-9:
        profile.append(PowerDemandPoint(t, float(power_W), stage))
    else:
        profile[-1] = PowerDemandPoint(t, float(power_W), stage)
    t2 = t + duration
    profile.append(PowerDemandPoint(t2, float(power_W), stage))
    return t2


def _append_dynamic(
    profile: List[PowerDemandPoint],
    t: float,
    result: TransitionResult,
    stage_prefix: str,
    sample_period_s: float,
) -> float:
    samples = result.samples
    if not samples:
        return t
    next_keep = samples[0].time_s
    kept = []
    for s in samples:
        if s.time_s + 1e-9 >= next_keep:
            kept.append(s)
            next_keep += sample_period_s
    if kept[-1].time_s < samples[-1].time_s - 1e-9:
        kept.append(samples[-1])
    t0_local = kept[0].time_s
    for s in kept:
        tt = t + (s.time_s - t0_local)
        stage = f"{stage_prefix} | {s.lift_prop_drag_state}"
        if profile and abs(profile[-1].time_s - tt) <= 1e-9:
            profile[-1] = PowerDemandPoint(tt, s.total_bus_power_W, stage)
        else:
            profile.append(PowerDemandPoint(tt, s.total_bus_power_W, stage))
    return t + result.duration_s


def build_phase5_mission_profile(
    cfg4: Mapping[str, Any],
    cfg5: Mapping[str, Any],
    *,
    mass_kg: Optional[float] = None,
    wing_area_m2: Optional[float] = None,
    wing_span_m: Optional[float] = None,
    lift_prop_diameter_in: Optional[float] = None,
    pusher_prop_diameter_in: Optional[float] = None,
    available_lift_thrust_per_motor_N: Optional[float] = None,
) -> Dict[str, Any]:
    ref = cfg5["reference_case"]
    mission = cfg5["mission"]
    mass = float(ref["mass_kg"] if mass_kg is None else mass_kg)
    S = float(ref["wing_area_m2"] if wing_area_m2 is None else wing_area_m2)
    b = float(ref["wing_span_m"] if wing_span_m is None else wing_span_m)
    Dlift = float(ref["lift_prop_diameter_in"] if lift_prop_diameter_in is None else lift_prop_diameter_in)
    Dpush = float(ref["pusher_prop_diameter_in"] if pusher_prop_diameter_in is None else pusher_prop_diameter_in)
    Tmax_each = float(ref["lift_thrust_available_per_motor_N"] if available_lift_thrust_per_motor_N is None else available_lift_thrust_per_motor_N)

    fwd = simulate_forward_transition(
        cfg4, cfg5, mass_kg=mass, wing_area_m2=S, wing_span_m=b,
        lift_prop_diameter_in=Dlift, pusher_prop_diameter_in=Dpush,
        available_lift_thrust_per_motor_N=Tmax_each,
    )
    rev = simulate_reverse_transition(
        cfg4, cfg5, mass_kg=mass, wing_area_m2=S, wing_span_m=b,
        lift_prop_diameter_in=Dlift, pusher_prop_diameter_in=Dpush,
        available_lift_thrust_per_motor_N=Tmax_each,
    )

    hover_W = _operational_hover_power_W(cfg4, mass, Dlift)
    climb_W = _operational_climb_power_W(cfg4, mass, Dlift)
    cruise = cruise_drag(
        cfg4,
        mass_kg=mass,
        speed_mps=float(ref["cruise_speed_mps"]),
        lift_prop_diameter_in=Dlift,
        lift_prop_state=str(ref["cruise_lift_prop_state"]),
        wing_area_m2=S,
        wing_span_m=b,
    )

    # Wing-borne climb: power from climb rate + cruise bus (screening energy model).
    G = 9.80665
    roc = float(mission.get("wingborne_climb_rate_mps", 2.0))
    climb_height = float(mission.get("wingborne_climb_height_m", 40.0))
    climb_eta = float(
        mission.get(
            "wingborne_climb_propulsive_efficiency",
            cfg4.get("aircraft", {}).get("cruise_propulsive_efficiency_reference", 0.62),
        )
    )
    wingborne_duration = float(mission.get("wingborne_climb_duration_s", climb_height / max(roc, 1e-6)))
    if mission.get("wingborne_climb_height_m") is not None and mission.get("wingborne_climb_duration_s") is None:
        wingborne_duration = climb_height / max(roc, 1e-6)
    wingborne_W = mass * G * roc / max(climb_eta, 1e-6) + float(cruise.total_cruise_bus_power_W)

    fixed_no_cruise_s = (
        float(mission["preflight_duration_s"])
        + float(mission["vtol_takeoff_duration_s"])
        + float(mission["vertical_climb_duration_s"])
        + fwd.duration_s
        + wingborne_duration
        + rev.duration_s
        + float(mission["vertical_descent_duration_s"])
        + float(mission["landing_hover_duration_s"])
    )
    target_s = float(mission["target_total_duration_min"]) * 60.0
    cruise_duration_s = max(0.0, target_s - fixed_no_cruise_s)
    if cruise_duration_s < float(mission["minimum_cruise_duration_s"]):
        raise ValueError("Mission timing leaves less than configured minimum cruise duration")

    profile: List[PowerDemandPoint] = []
    t = 0.0
    t = _append_constant(profile, t, mission["preflight_duration_s"], mission["preflight_power_W"], "Preflight / ready")
    t = _append_constant(profile, t, mission["vtol_takeoff_duration_s"], hover_W, "VTOL takeoff")
    t = _append_constant(profile, t, mission["vertical_climb_duration_s"], climb_W, "Vertical climb")
    t = _append_dynamic(profile, t, fwd, "Forward transition", float(mission["dynamic_transition_sample_period_s"]))
    t = _append_constant(profile, t, wingborne_duration, wingborne_W, "Wing-borne climb")
    t = _append_constant(profile, t, cruise_duration_s, cruise.total_cruise_bus_power_W, "Cruise")
    t = _append_dynamic(profile, t, rev, "Reverse transition", float(mission["dynamic_transition_sample_period_s"]))
    t = _append_constant(
        profile, t, mission["vertical_descent_duration_s"],
        hover_W * float(mission["vertical_descent_hover_power_factor"]), "Vertical descent"
    )
    t = _append_constant(profile, t, mission["landing_hover_duration_s"], hover_W, "Landing hover")

    # Ensure exact final endpoint at the target duration; tiny sample-grid differences
    # are acceptable but are normalized here for repeatable mission accounting.
    if profile:
        profile[-1] = PowerDemandPoint(target_s, profile[-1].bus_power_W, profile[-1].stage)

    stage_energy_Wh: Dict[str, float] = {}
    for a, bpt in zip(profile, profile[1:]):
        name = a.stage.split(" | ")[0]
        stage_energy_Wh[name] = stage_energy_Wh.get(name, 0.0) + a.bus_power_W * (bpt.time_s - a.time_s) / 3600.0

    return {
        "profile": profile,
        "forward_transition": fwd,
        "reverse_transition": rev,
        "hover_bus_power_W": hover_W,
        "vertical_climb_bus_power_W": climb_W,
        "cruise_bus_power_W": cruise.total_cruise_bus_power_W,
        "cruise_duration_s": cruise_duration_s,
        "stage_energy_Wh": stage_energy_Wh,
        "total_profile_energy_Wh": sum(stage_energy_Wh.values()),
        "total_duration_s": target_s,
    }


def run_phase5_through_phase2_ems(
    cfg4: Mapping[str, Any],
    cfg5: Mapping[str, Any],
    *,
    mass_kg: Optional[float] = None,
    wing_area_m2: Optional[float] = None,
    wing_span_m: Optional[float] = None,
    lift_prop_diameter_in: Optional[float] = None,
    pusher_prop_diameter_in: Optional[float] = None,
    available_lift_thrust_per_motor_N: Optional[float] = None,
    phase2_config_path: Path | str = Path("config/phase2_ie_soar_800_reference.json"),
    capture_ems_steps: bool = False,
) -> Dict[str, Any]:
    mission = build_phase5_mission_profile(
        cfg4, cfg5,
        mass_kg=mass_kg, wing_area_m2=wing_area_m2, wing_span_m=wing_span_m,
        lift_prop_diameter_in=lift_prop_diameter_in, pusher_prop_diameter_in=pusher_prop_diameter_in,
        available_lift_thrust_per_motor_N=available_lift_thrust_per_motor_N,
    )
    cfg2 = load_hybrid_power_config(Path(phase2_config_path))
    ems = HybridPowerSystem(cfg2).simulate(mission["profile"], capture_steps=capture_ems_steps, max_step_s=0.5)
    return {
        "mission": {
            "total_duration_s": mission["total_duration_s"],
            "cruise_duration_s": mission["cruise_duration_s"],
            "hover_bus_power_W": mission["hover_bus_power_W"],
            "vertical_climb_bus_power_W": mission["vertical_climb_bus_power_W"],
            "cruise_bus_power_W": mission["cruise_bus_power_W"],
            "stage_energy_Wh": mission["stage_energy_Wh"],
            "total_profile_energy_Wh": mission["total_profile_energy_Wh"],
        },
        "forward_transition": mission["forward_transition"].to_dict(include_samples=False),
        "reverse_transition": mission["reverse_transition"].to_dict(include_samples=False),
        "ems": {
            "feasible": ems.feasible,
            "failure_reasons": ems.failure_reasons,
            "warnings": ems.warnings,
            "gates": [g.__dict__ for g in ems.gates],
            "summary": ems.summary,
        },
    }

# Clean integrated API aliases; legacy development names retained for compatibility.
build_mission_profile = build_phase5_mission_profile
run_mission_through_ems = run_phase5_through_phase2_ems
