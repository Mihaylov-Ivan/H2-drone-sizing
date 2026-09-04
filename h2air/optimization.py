from __future__ import annotations

"""Multidisciplinary screening optimization for H2AIR.

The optimizer intentionally uses a two-level strategy:
1) a fast, physics-connected screen for many candidates (mass/packaging,
   aero/propulsion, time-domain point-mass transition/mission, structures,
   CG/static margin and rotor pitch balance), then
2) the expensive integrated EMS + 6-DOF pipeline for the best candidates.

This avoids pretending that hundreds of generic 6-DOF runs add fidelity while
still requiring the final selected candidate to pass the full integrated model.
"""

from copy import deepcopy
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple
import csv, json, math
import numpy as np
from scipy.stats import qmc

from .design_coupling import design_overrides, resolve_design_couplings
from .pipeline import _components_with_power, _mass_result, _sync_design, _apply_structural_feedback
from .aero_propulsion import lift_layout, cruise_drag, lift_system_sizing, minimum_wing_area_for_stall_margin
from .mission import build_mission_profile
from .structures import build_structural_result

G = 9.80665

LEGACY_DEFAULT_SPACE: Dict[str, Tuple[float, float]] = {
    "wing_area_m2": (0.80, 0.90),
    "wing_span_m": (3.15, 3.60),
    "wing_ac_x_m": (0.34, 0.43),
    "horizontal_tail_area_m2": (0.13, 0.20),
    "lift_row_center_x_m": (0.42, 0.50),
    "battery_x_m": (0.18, 0.62),
    "battery_z_m": (0.060, 0.064),
    "fuel_cell_x_m": (0.140, 0.190),
    "fuel_cell_z_m": (-0.029, -0.025),
    "hydrogen_cylinder_x_m": (0.505, 0.554),
    "hydrogen_cylinder_z_m": (-0.029, -0.025),
}



def default_space_for_config(cfg: Mapping[str, Any]) -> Dict[str, Tuple[float,float]]:
    d=cfg["design"]
    if str(d.get("surface_geometry_mode", "legacy_area_span")) != "root_taper_span":
        return dict(LEGACY_DEFAULT_SPACE)
    opt_space = cfg.get("optimizer_settings", {}).get("design_space", {})
    defaults = {
        "wing_ac_x_m": (0.30, 0.48),
        "horizontal_tail_ac_x_m": (0.92, 1.15),
        "vertical_tail_ac_x_m": (0.92, 1.15),
        "lift_row_center_x_m": (0.35, 0.52),
        "battery_x_m": (0.12, 0.65),
        "battery_z_m": (0.02, 0.065),
        "fuel_cell_x_m": (0.14, 0.40),
        "fuel_cell_z_m": (-0.035, 0.01),
        "hydrogen_cylinder_x_m": (0.25, 0.55),
        "hydrogen_cylinder_z_m": (-0.035, 0.01),
    }
    space: Dict[str, Tuple[float,float]] = {
        k: tuple(float(x) for x in opt_space.get(k, v)) for k,v in defaults.items()
    }
    if str(d.get("main_wing_span_mode","auto")).lower()=="auto":
        space["main_wing_span_m"] = tuple(float(x) for x in d["main_wing_span_bounds_m"])
    if str(d.get("horizontal_tail_span_mode","auto")).lower()=="auto":
        space["horizontal_tail_span_m"] = tuple(float(x) for x in d["horizontal_tail_span_bounds_m"])
    if str(d.get("vertical_tail_span_mode","auto")).lower()=="auto":
        space["vertical_tail_height_m"] = tuple(float(x) for x in d["vertical_tail_span_bounds_m"])

    # Constrain the lift-row midpoint to the physically feasible range for the
    # selected propeller diameter and the user's fixed fuselage envelope. This
    # prevents an otherwise good LHS search from wasting nearly all candidates
    # on rotor rows that cannot satisfy the required disk-edge clearance.
    lay=cfg["aero_propulsion"]["lift_layout"]
    D=float(d["lift_propeller_diameter_in"])*0.0254
    sep=D+float(lay["required_disk_edge_clearance_m"])
    physical_lo=float(lay["minimum_front_x_m"])+0.5*sep
    physical_hi=float(lay["maximum_rear_x_m"])-0.5*sep
    if physical_lo > physical_hi:
        raise ValueError(f"Selected lift propeller {float(d['lift_propeller_diameter_in']):.1f} in cannot fit within the fixed longitudinal lift-station envelope")
    if "lift_row_center_x_m" in space:
        lo,hi=space["lift_row_center_x_m"]
        lo=max(float(lo),physical_lo); hi=min(float(hi),physical_hi)
        if lo>hi:
            raise ValueError("Optimizer lift-row bounds do not overlap the physically feasible propeller-clearance interval")
        space["lift_row_center_x_m"]=(lo,hi)
    return space

@dataclass
class Candidate:
    index: int
    values: Dict[str, float]
    score: float
    feasible: bool
    metrics: Dict[str, Any]
    violations: List[str]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _zero_pitch_pair(total_thrust_N: float, xf: float, xr: float, cg: float) -> Tuple[float, float]:
    af, ar = xf - cg, xr - cg
    if not (af < 0 < ar):
        return float("inf"), float("inf")
    front_each = total_thrust_N * ar / (2.0 * (ar - af))
    rear_each = total_thrust_N * (-af) / (2.0 * (ar - af))
    return front_each, rear_each


def screening_evaluate(raw_cfg: Mapping[str, Any], values: Mapping[str, float], *, index: int = 0) -> Candidate:
    cfg = design_overrides(raw_cfg, values)
    d = cfg["design"]
    comps = _components_with_power(cfg)
    mass0 = _mass_result(cfg, comps)
    aero_cfg, trans_cfg, six_cfg, struct_cfg = _sync_design(cfg, mass0)

    D = float(d["lift_propeller_diameter_in"])
    V = float(d["cruise_speed_mps"])
    state = str(d["cruise_lift_propeller_state"])
    layout = lift_layout(aero_cfg, D)
    drag = cruise_drag(
        aero_cfg, mass_kg=mass0.takeoff.total_mass_kg, speed_mps=V,
        lift_prop_diameter_in=D, lift_prop_state=state,
        wing_area_m2=float(d["wing_area_m2"]), wing_span_m=float(d["wing_span_m"]),
    )
    lift15 = lift_system_sizing(aero_cfg, mass_kg=float(d["mtow_ceiling_kg"]), prop_diameter_in=D)
    wing15 = minimum_wing_area_for_stall_margin(aero_cfg, mass_kg=float(d["mtow_ceiling_kg"]), speed_mps=V)

    rows = [layout.proposed_front_x_m, layout.proposed_rear_x_m]
    y = float(aero_cfg["lift_layout"]["y_abs_m"])
    mission = build_mission_profile(
        aero_cfg, trans_cfg,
        mass_kg=mass0.takeoff.total_mass_kg,
        wing_area_m2=float(d["wing_area_m2"]), wing_span_m=float(d["wing_span_m"]),
        available_lift_thrust_per_motor_N=float(d["lift_motor_available_thrust_per_motor_N"]),
    )

    # Feed current candidate into the actual structural screening model.
    struct_cfg["aircraft"].update({
        "reference_mass_kg": mass0.takeoff.total_mass_kg,
        "cg_station_x_m": mass0.takeoff.cg_m[0],
        "design_mass_ceiling_kg": float(d["mtow_ceiling_kg"]),
        "reference_wing_span_m": float(d["wing_span_m"]),
        "design_wing_span_m": float(d["wing_span_m"]),
        "horizontal_tail_area_m2": float(d["horizontal_tail_area_m2"]),
    })
    struct_cfg["rotor_system"].update({
        "row_x_m": rows,
        "motor_y_m": y,
        "phase4_equal_share_design_thrust_N": lift15.design_thrust_per_motor_N,
        "candidate_motor_max_thrust_N": float(d["lift_motor_structural_limit_thrust_per_motor_N"]),
    })
    # 1-g zero-pitch hover allocation for structural reference.
    f1, r1 = _zero_pitch_pair(mass0.takeoff.total_mass_kg * G, rows[0], rows[1], mass0.takeoff.cg_m[0])
    struct_cfg["rotor_system"]["phase6_reference_hover_thrusts_N"] = [f1, f1, r1, r1]
    structure = build_structural_result(struct_cfg)
    comps_post = _apply_structural_feedback(cfg, comps, structure)
    mass_post = _mass_result(cfg, comps_post)

    np_x = float(d["neutral_point_station_x_m"])
    mac = float(d["wing_mac_m"])
    sm = (np_x - mass_post.takeoff.cg_m[0]) / mac
    total15 = 4.0 * float(lift15.design_thrust_per_motor_N)
    front15, rear15 = _zero_pitch_pair(total15, rows[0], rows[1], mass_post.takeoff.cg_m[0])
    max_motor15 = max(front15, rear15)

    req = cfg["power_system"]["requirements"]
    h2 = cfg["power_system"]["hydrogen"]
    energy_target_Wh = float(h2["reference_net_energy_Wh"]) * (1.0 - float(h2["mission_energy_reserve_fraction"]))
    structural_fail = any(
        g["status"] == "FAIL"
        for g in structure["gates"]
        if "current_" not in g["name"] and "legacy_" not in g["name"]
    )

    metrics: Dict[str, Any] = {
        "pre_structure_mass_kg": mass0.takeoff.total_mass_kg,
        "post_structure_mass_kg": mass_post.takeoff.total_mass_kg,
        "cg_x_m": mass_post.takeoff.cg_m[0],
        "neutral_point_x_m": np_x,
        "static_margin": sm,
        "packaging_fail_count": mass0.summary["packaging_fail_count"],
        "layout_pass": bool(layout.proposed_layout_pass),
        "lift_rows_x_m": rows,
        "cruise_bus_power_W": drag.total_cruise_bus_power_W,
        "cruise_drag_N": drag.total_drag_N,
        "wing_required_15kg_m2": wing15["minimum_wing_area_m2"],
        "mission_energy_Wh": mission["total_profile_energy_Wh"],
        "energy_target_Wh": energy_target_Wh,
        "forward_transition_success": bool(mission["forward_transition"].success),
        "reverse_transition_success": bool(mission["reverse_transition"].success),
        "forward_transition_duration_s": mission["forward_transition"].duration_s,
        "reverse_transition_duration_s": mission["reverse_transition"].duration_s,
        "front_motor_15kg_N": front15,
        "rear_motor_15kg_N": rear15,
        "max_motor_15kg_N": max_motor15,
        "available_motor_thrust_N": float(d["lift_motor_available_thrust_per_motor_N"]),
        "structural_screen_fail": bool(structural_fail),
        "wing_spar_od_mm": structure["summary"]["recommended_wing_spar_od_mm"],
        "boom_od_mm": structure["summary"]["recommended_boom_od_mm"],
    }

    violations: List[str] = []
    penalty = 0.0
    def add_violation(name: str, amount: float, weight: float) -> None:
        nonlocal penalty
        violations.append(name)
        penalty += weight * max(float(amount), 0.0) ** 2

    ceiling = float(d["mtow_ceiling_kg"])
    if mass_post.takeoff.total_mass_kg > ceiling:
        add_violation("mass_ceiling", mass_post.takeoff.total_mass_kg - ceiling, 250.0)
    if mass0.summary["packaging_fail_count"]:
        add_violation("packaging", float(mass0.summary["packaging_fail_count"]), 300.0)
    if not layout.proposed_layout_pass:
        add_violation("lift_prop_clearance", 1.0, 300.0)
    if float(d["wing_area_m2"]) + 1e-9 < float(wing15["minimum_wing_area_m2"]):
        add_violation("15kg_stall_margin", (float(wing15["minimum_wing_area_m2"])-float(d["wing_area_m2"]))/0.05, 100.0)
    if sm < 0.05:
        add_violation("static_margin_low", (0.05-sm)/0.05, 400.0)
    if sm > 0.15:
        add_violation("static_margin_high", (sm-0.15)/0.05, 40.0)
    if drag.total_cruise_bus_power_W > float(req["representative_cruise_power_gate_W"]):
        add_violation("cruise_power_gate", (drag.total_cruise_bus_power_W-float(req["representative_cruise_power_gate_W"]))/100.0, 100.0)
    if mission["total_profile_energy_Wh"] > energy_target_Wh:
        add_violation("120min_h2_energy_target", (mission["total_profile_energy_Wh"]-energy_target_Wh)/100.0, 80.0)
    if not mission["forward_transition"].success:
        add_violation("forward_transition", 1.0, 300.0)
    if not mission["reverse_transition"].success:
        add_violation("reverse_transition", 1.0, 300.0)
    if max_motor15 > float(d["lift_motor_available_thrust_per_motor_N"]):
        add_violation("15kg_zero_pitch_motor_thrust", (max_motor15-float(d["lift_motor_available_thrust_per_motor_N"]))/5.0, 200.0)
    if structural_fail:
        add_violation("structural_screen", 1.0, 300.0)

    feasible = not violations
    # Mass is primary; power/energy and SM-centering are secondary objectives.
    objective = (
        mass_post.takeoff.total_mass_kg
        + 0.0020 * max(drag.total_cruise_bus_power_W - 300.0, 0.0)
        + 0.0008 * max(mission["total_profile_energy_Wh"] - 650.0, 0.0)
        + 2.0 * abs(sm - 0.08)
    )
    score = objective + penalty
    return Candidate(index=index, values={k: float(v) for k,v in values.items()}, score=float(score), feasible=feasible, metrics=metrics, violations=violations)


def _lhs_points(space: Mapping[str, Tuple[float,float]], n: int, seed: int) -> Iterable[Dict[str,float]]:
    names = list(space)
    sampler = qmc.LatinHypercube(d=len(names), seed=seed)
    unit = sampler.random(n=n)
    lo = np.array([space[k][0] for k in names], dtype=float)
    hi = np.array([space[k][1] for k in names], dtype=float)
    pts = qmc.scale(unit, lo, hi)
    for row in pts:
        yield {k: float(v) for k,v in zip(names,row)}


def optimize_design(
    cfg: Mapping[str, Any], *, sample_count: int = 48, seed: int = 42,
    design_space: Mapping[str, Tuple[float,float]] | None = None,
    local_refinement_count: int = 16,
) -> Dict[str, Any]:
    space = dict(design_space or default_space_for_config(cfg))
    candidates: List[Candidate] = []
    # Always include the current resolved design as a deterministic seed. This
    # keeps the LHS+local-refinement algorithm but guarantees that a known-good
    # user design is not accidentally omitted by random space filling.
    current_values = {k: float(cfg["design"][k]) for k in space}
    candidates.append(screening_evaluate(cfg, current_values, index=0))
    for i, values in enumerate(_lhs_points(space, sample_count, seed), start=1):
        candidates.append(screening_evaluate(cfg, values, index=i))

    # Deterministic local jitter around the best screen candidates.
    rng = np.random.default_rng(seed + 1)
    base = sorted(candidates, key=lambda c: c.score)[:min(4, len(candidates))]
    names = list(space)
    widths = {k: space[k][1]-space[k][0] for k in names}
    for j in range(local_refinement_count):
        parent = base[j % len(base)]
        vals = dict(parent.values)
        for k in names:
            vals[k] = float(np.clip(vals[k] + rng.normal(0.0, 0.08*widths[k]), space[k][0], space[k][1]))
        candidates.append(screening_evaluate(cfg, vals, index=len(candidates)))

    ranked = sorted(candidates, key=lambda c: (not c.feasible, c.score))
    best = ranked[0]
    return {
        "method": "Latin-hypercube screening + deterministic local refinement",
        "fidelity": "physics-connected screening; final candidate requires full integrated EMS + 6DOF verification",
        "sample_count": len(candidates),
        "seed": seed,
        "design_space": {k:list(v) for k,v in space.items()},
        "feasible_count": sum(c.feasible for c in candidates),
        "best": best.to_dict(),
        "ranked": [c.to_dict() for c in ranked],
    }


def write_optimization_outputs(result: Mapping[str, Any], out_dir: str | Path) -> None:
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    (out/"optimization_results.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    ranked = result["ranked"]
    if ranked:
        rows=[]
        for c in ranked:
            row={"rank":len(rows)+1,"index":c["index"],"feasible":c["feasible"],"score":c["score"],"violations":";".join(c["violations"])}
            row.update(c["values"]); row.update(c["metrics"])
            # Flatten row locations for CSV.
            if isinstance(row.get("lift_rows_x_m"), list): row["lift_rows_x_m"]="/".join(f"{x:.5f}" for x in row["lift_rows_x_m"])
            rows.append(row)
        keys=[]
        for r in rows:
            for k in r:
                if k not in keys: keys.append(k)
        with (out/"optimization_candidates.csv").open("w",newline="",encoding="utf-8") as f:
            w=csv.DictWriter(f,fieldnames=keys); w.writeheader(); w.writerows(rows)


def optimized_config(cfg: Mapping[str, Any], optimization_result: Mapping[str, Any]) -> Dict[str, Any]:
    return design_overrides(cfg, optimization_result["best"]["values"])

def derive_margin_variant(cfg:Mapping[str,Any],optimization_result:Mapping[str,Any],*,stall_area_factor:float=1.109)->Dict[str,Any]:
    """Derive deterministic margin-biased candidate while preserving v2 selection logic."""
    best=optimization_result["best"]
    vals=dict(best["values"])
    req=float(best["metrics"]["wing_required_15kg_m2"])
    d=cfg["design"]
    space=default_space_for_config(cfg)

    if str(d.get("surface_geometry_mode","legacy_area_span"))=="root_taper_span":
        cr=float(d["main_wing_root_chord_m"]); lam=float(d["main_wing_taper_ratio"])
        if "main_wing_span_m" in space:
            target_area=stall_area_factor*req
            target_span=2.0*target_area/(cr*(1.0+lam))
            lo,hi=space["main_wing_span_m"]
            vals["main_wing_span_m"]=float(np.clip(round(max(vals.get("main_wing_span_m",d["main_wing_span_m"]),target_span),3),lo,hi))
        if "horizontal_tail_span_m" in space:
            loh,hih=space["horizontal_tail_span_m"]
            # Add modest tail margin without blindly maxing the tail span, which
            # can over-stabilize the actual 6-DOF transition.
            vals["horizontal_tail_span_m"]=float(np.clip(vals.get("horizontal_tail_span_m",d["horizontal_tail_span_m"])*1.05,loh,hih))
        loac,hiac=space["wing_ac_x_m"]
        vals["wing_ac_x_m"]=float(np.clip(round(best["values"]["wing_ac_x_m"]+0.003,3),loac,hiac))
    else:
        loS,hiS=LEGACY_DEFAULT_SPACE["wing_area_m2"]
        target=float(np.clip(round(max(vals["wing_area_m2"],stall_area_factor*req),2),loS,hiS))
        ar=vals["wing_span_m"]**2/vals["wing_area_m2"]
        vals["wing_area_m2"]=target
        vals["wing_span_m"]=float(np.sqrt(ar*target))
        vals["horizontal_tail_area_m2"]=float(LEGACY_DEFAULT_SPACE["horizontal_tail_area_m2"][1])
        vals["wing_ac_x_m"]=float(np.clip(round(best["values"]["wing_ac_x_m"]+0.005,3),*LEGACY_DEFAULT_SPACE["wing_ac_x_m"]))

    primary=screening_evaluate(cfg,vals,index=0)
    candidates=[primary]
    if not primary.feasible:
        loac,hiac=space.get("wing_ac_x_m",LEGACY_DEFAULT_SPACE["wing_ac_x_m"])
        for dx in (-0.005,0.005,0.010):
            v=dict(vals)
            v["wing_ac_x_m"]=float(np.clip(vals["wing_ac_x_m"]+dx,loac,hiac))
            candidates.append(screening_evaluate(cfg,v,index=len(candidates)))
    ranked=sorted(candidates,key=lambda c:(not c.feasible,c.score))
    return {
        "method":"deterministic stall-area margin + tail/AC margin (v3 span-derived geometry aware)",
        "stall_area_factor":stall_area_factor,
        "best":ranked[0].to_dict(),
        "candidates":[c.to_dict() for c in ranked],
    }


def select_recommended_candidate(
    nominal_verified_summary: Mapping[str, Any],
    robust_verified_summary: Mapping[str, Any],
    nominal_uncertainty_summary: Mapping[str, Any],
    robust_uncertainty_summary: Mapping[str, Any],
) -> str:
    """Return the accepted design-study candidate name.

    The margin-biased candidate is selected only when it passes the full
    integrated pipeline and its uncertainty-screen pass probability is not
    worse than the nominal mass optimum.  Otherwise the nominal candidate is
    retained (provided it is the better available verified option).
    """
    rpass = robust_verified_summary.get("design_status") == "SCREENING_PASS"
    npass = nominal_verified_summary.get("design_status") == "SCREENING_PASS"
    rp = float(robust_uncertainty_summary.get("screening_pass_probability", 0.0))
    np_ = float(nominal_uncertainty_summary.get("screening_pass_probability", 0.0))
    if rpass and (not npass or rp >= np_):
        return "robust_margin_biased"
    return "nominal_mass_optimized"
