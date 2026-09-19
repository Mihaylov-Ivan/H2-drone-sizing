from __future__ import annotations
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence
import csv, hashlib, json, platform
import numpy as np

from .design_coupling import apply_high_wing_spar_stack, resolve_design_couplings, design_overrides
from .config import hybrid_config_from_mapping
from .hybrid_power import HybridPowerSystem, reference_sources
from .mass_properties import component_from_mapping, build_mass_geometry_result, apply_growth_margins
from .manufacturing_mass import apply_manufacturing_mass_model, apply_structural_mass_updates, check_manufacturing_ledger_consistency
from .landing_legs import sync_landing_legs_drag
from .aero_propulsion import cruise_drag, lift_layout, lift_system_sizing, minimum_wing_area_for_stall_margin
from .transition import simulate_forward_transition, simulate_reverse_transition
from .mission import build_mission_profile
from .sixdof import Model, simulate_tracking, simulate_cruise_gust, summarize_samples
from .control_gates import evaluate_case as evaluate_control_case
from .structures import build_structural_result
from .report import write_markdown_report
from .aircraft_spec import write_aircraft_specification
from .cad_export import write_cad_exports
from .fuel_cell_cooling import analyze_fuel_cell_cooling
from .flight_envelope import evaluate_vn_envelope, required_area_for_positive_manoeuvre

G=9.80665

def _sha(obj: Any)->str:
    return hashlib.sha256(json.dumps(obj,sort_keys=True,separators=(",",":"),default=str).encode()).hexdigest()

def recommended_structure_screen_pass(structure_gates: Sequence[Mapping[str, Any]]) -> bool:
    """True when recommended design-member screens pass.

    Audit reference/current/legacy sections and the legacy partial-feedback mass
    gate are excluded; closed MTOW is checked separately via mass_ceiling.
    """
    for g in structure_gates:
        n = str(g["name"])
        if g["status"] == "NOT_EVALUATED":
            continue
        if n == "mass_ceiling_after_partial_structural_feedback":
            continue
        if "current_" in n or "legacy_" in n or n.startswith("reference_"):
            continue
        if g["status"] == "FAIL":
            return False
    return True

def _sync_design(cfg: Mapping[str,Any], mass_result=None):
    d=cfg["design"]
    a=deepcopy(cfg["aero_propulsion"]); t=deepcopy(cfg["transition_mission"]); s6=deepcopy(cfg["sixdof_control"]); st=deepcopy(cfg["structures"])
    mass=float(mass_result.takeoff.total_mass_kg) if mass_result is not None else float(a["aircraft"]["phase3_screening_mtow_kg"])
    a["aircraft"].update({
        "phase3_screening_mtow_kg":mass,"mtow_ceiling_kg":float(d["mtow_ceiling_kg"]),
        "wing_area_m2":float(d["wing_area_m2"]),"wing_span_m":float(d["wing_span_m"]),"wing_MAC_m":float(d["wing_mac_m"]),
        "wing_CLmax":float(d["wing_CLmax"]),"cruise_speed_mps":float(d["cruise_speed_mps"]),
        "transition_CLmax_increment":float(d.get("transition_CLmax_increment",0.0)),
        "pusher_propeller_diameter_in":float(d.get("pusher_propeller_diameter_in",18.0)),
    })
    main_af=cfg.get("airfoil_model",{}).get("main_wing",{})
    if main_af.get("polar_rows"):
        a["aircraft"]["wing_polar_rows"] = main_af["polar_rows"]
        a["aircraft"]["wing_polar_reynolds_number"] = main_af.get("reynolds_number")
    a["lift_system"]["supplier_conditional_hybrid_peak_W"]=float(cfg["power_system"]["requirements"]["supplier_conditional_hybrid_peak_W"])
    t["reference_case"].update({
        "mass_kg":mass,"wing_area_m2":float(d["wing_area_m2"]),"wing_span_m":float(d["wing_span_m"]),
        "lift_prop_diameter_in":float(d["lift_propeller_diameter_in"]),"pusher_prop_diameter_in":float(d["pusher_propeller_diameter_in"]),
        "cruise_lift_prop_state":str(d["cruise_lift_propeller_state"]),"lift_thrust_available_per_motor_N":float(d["lift_motor_available_thrust_per_motor_N"]),
        "cruise_speed_mps":float(d["cruise_speed_mps"]),
    })
    t["propulsion"]["pusher_max_electrical_power_W"]=float(d["pusher_max_electrical_power_W"])
    t["mission"]["target_total_duration_min"]=float(d["mission_duration_min"])
    if mass_result is not None:
        mp=mass_result.takeoff
        s6["aircraft"].update({
            "mass_kg":mp.total_mass_kg,"wing_area_m2":float(d["wing_area_m2"]),"wing_span_m":float(d["wing_span_m"]),"wing_mac_m":float(d["wing_mac_m"]),
            "cg_station_x_m":mp.cg_m[0],"cg_z_up_m":mp.cg_m[2],"neutral_point_station_x_m":float(d["neutral_point_station_x_m"]),
            "horizontal_tail_area_m2":float(d.get("horizontal_tail_area_m2",s6["aircraft"].get("horizontal_tail_area_m2",0.1276))),
            "horizontal_tail_ac_station_x_m":float(d.get("horizontal_tail_ac_x_m",s6["aircraft"].get("horizontal_tail_ac_station_x_m",1.05))),
            "inertia_tensor_kgm2":[[mp.Ixx_kgm2,-mp.Ixy_kgm2,-mp.Ixz_kgm2],[-mp.Ixy_kgm2,mp.Iyy_kgm2,-mp.Iyz_kgm2],[-mp.Ixz_kgm2,-mp.Iyz_kgm2,mp.Izz_kgm2]],
        })
    s6["aerodynamics"]["CL_max"]=float(d["wing_CLmax"])
    t["aerodynamics"]["CL_max"]=float(d["wing_CLmax"])
    t["aerodynamics"]["transition_CLmax_increment"]=float(d.get("transition_CLmax_increment",0.0))
    a["aircraft"]["transition_CLmax_increment"]=float(d.get("transition_CLmax_increment",0.0))
    s6["lift_system"].update({"max_thrust_per_motor_N":float(d["lift_motor_available_thrust_per_motor_N"]),"propeller_diameter_in":float(d["lift_propeller_diameter_in"])})
    s6["pusher"].update({
        "propeller_diameter_in": float(d["pusher_propeller_diameter_in"]),
        "max_electrical_power_W": float(d["pusher_max_electrical_power_W"]),
        "motor_efficiency_assumption": float(a["pusher_screen"]["motor_efficiency_assumption"]),
        "esc_efficiency_assumption": float(a["pusher_screen"]["esc_efficiency_assumption"]),
    })
    return a,t,s6,st

def _components_with_power(cfg:Mapping[str,Any]):
    raw=deepcopy(cfg["geometry_mass"]["component_ledger"])
    comps=[component_from_mapping(x) for x in raw["components"]]
    p=cfg["power_system"]; fc=p["fuel_cell"]; h=p["hydrogen"]; batt=p["battery"]
    by={c.name:c for c in comps}
    if "fuel_cell_system" in by and fc.get("module_mass_kg") is not None:
        # Prefer packaging split (stack + control box) when present; else whole module.
        stack_m = fc.get("stack_mass_kg")
        box_m = fc.get("control_box_mass_kg")
        if stack_m is not None and box_m is not None:
            c = by["fuel_cell_system"]
            c.mass_takeoff_kg = c.mass_end_kg = float(stack_m)
            c.status = "supplier_confirmed"
            c.source = "FCPM stack mass (share of module_mass_kg)"
            if "fuel_cell_control_box" in by:
                cb = by["fuel_cell_control_box"]
                cb.mass_takeoff_kg = cb.mass_end_kg = float(box_m)
                cb.status = "supplier_confirmed"
                cb.source = "FCPM control-box mass (share of module_mass_kg)"
        else:
            c = by["fuel_cell_system"]
            c.mass_takeoff_kg = c.mass_end_kg = float(fc["module_mass_kg"])
            c.status = "supplier_confirmed"
            c.source = "Integrated power-system config"
    if "hydrogen_regulator" in by and h.get("regulator_mass_kg") is not None:
        c=by["hydrogen_regulator"]; c.mass_takeoff_kg=c.mass_end_kg=float(h["regulator_mass_kg"]); c.status="supplier_confirmed"; c.source="Integrated power-system config"
    if "stored_hydrogen" in by and h.get("nominal_h2_mass_kg") is not None:
        c=by["stored_hydrogen"]; c.mass_takeoff_kg=float(h["nominal_h2_mass_kg"]); c.mass_end_kg=float(h["nominal_h2_mass_kg"]); c.status="preliminary_target"; c.source="Integrated power-system config"
    if "buffer_battery" in by and batt.get("mass_kg") is not None:
        c=by["buffer_battery"]; c.mass_takeoff_kg=c.mass_end_kg=float(batt["mass_kg"]); c.status="supplier_confirmed"; c.source="Integrated power-system config"
    if "hydrogen_cylinder" in by and h.get("cylinder_mass_kg") is not None:
        c=by["hydrogen_cylinder"]; c.mass_takeoff_kg=c.mass_end_kg=float(h["cylinder_mass_kg"]); c.status="supplier_confirmed"; c.source="Integrated power-system config"
        dims = dict(c.dimensions_m or {})
        if h.get("cylinder_length_m") is not None:
            dims["length"] = float(h["cylinder_length_m"])
        if h.get("cylinder_diameter_m") is not None:
            dims["diameter"] = float(h["cylinder_diameter_m"])
        if dims:
            c.dimensions_m = dims
    comps, manufacturing_ledger = apply_manufacturing_mass_model(cfg, comps)
    if isinstance(cfg, dict):
        cfg.setdefault("_runtime", {})["manufacturing_mass_ledger"] = manufacturing_ledger
        if manufacturing_ledger:
            ok, delta, msg = check_manufacturing_ledger_consistency(comps, manufacturing_ledger)
            cfg["_runtime"]["manufacturing_ledger_consistency"] = {"ok": ok, "delta_kg": delta, "message": msg}
    comps = apply_growth_margins(comps)
    return comps

def _mass_result(cfg, comps):
    gm=cfg["geometry_mass"]["geometry_model"]
    return build_mass_geometry_result(components=comps,geometry=gm["geometry"],requirements=gm["requirements"],packaging=gm["packaging"],model_version="H2AIR-integrated-mass-properties")

def _apply_structural_feedback(cfg, comps, structure):
    # v3 detailed manufacturing ledger: structural-linear members scale with
    # selected structural section area, while hardcoded masses stay locked.
    if cfg.get("manufacturing_mass_model", {}).get("enabled", False):
        return apply_structural_mass_updates(cfg, comps, structure)
    fb=structure["mass_feedback"]
    out=deepcopy(comps); by={c.name:c for c in out}
    if "wing_structure" in by:
        by["wing_structure"].mass_takeoff_kg += float(fb["wing_spar_mass_delta_kg"]); by["wing_structure"].mass_end_kg=by["wing_structure"].mass_takeoff_kg
    if "boom_tube_left" in by and "boom_tube_right" in by:
        each=float(fb["geometry_reconciled_candidate_boom_tube_mass_kg"])/2
        for n in ("boom_tube_left","boom_tube_right"): by[n].mass_takeoff_kg=by[n].mass_end_kg=each
    if "horizontal_tail_structure" in by:
        by["horizontal_tail_structure"].mass_takeoff_kg += float(fb["horizontal_tail_spar_mass_delta_kg"]); by["horizontal_tail_structure"].mass_end_kg=by["horizontal_tail_structure"].mass_takeoff_kg
    for n in ("vertical_tail_left","vertical_tail_right"):
        if n in by:
            by[n].mass_takeoff_kg += float(fb["vertical_tail_spar_mass_delta_kg"])/2; by[n].mass_end_kg=by[n].mass_takeoff_kg
    return out

def _structural_section_consistency(cfg, structure, tol_m: float = 1e-9):
    """Check that the sections verified by Structures are the ones in the mass/CAD rules."""
    mm=cfg.get("manufacturing_mass_model",{})
    checks=structure.get("structural_checks",{})
    pairs=[
        ("main_wing_primary_spar", (mm.get("main_wing") or {}).get("primary_spar") or {}, ((checks.get("wing") or {}).get("recommended_screen") or {}).get("section") or {}),
        ("horizontal_tail_primary_spar", (mm.get("horizontal_tail") or {}).get("primary_spar") or {}, (((checks.get("tails") or {}).get("horizontal_recommended_screen") or {}).get("section") or {})),
        ("vertical_tail_primary_spar", (mm.get("vertical_tail") or {}).get("primary_spar") or {}, (((checks.get("tails") or {}).get("vertical_recommended_screen") or {}).get("section") or {})),
        ("boom_tubes", mm.get("boom_tubes") or {}, ((checks.get("booms") or {}).get("recommended_screen") or {}).get("section") or {}),
    ]
    land_rule=((mm.get("landing_legs") or {}).get("tube") or {})
    land=((checks.get("landing_legs") or {}).get("recommended_screen") or {}).get("section") or {}
    if land:
        pairs.append(("landing_legs",land_rule,land))
    mismatches=[]
    for name, rule, sec in pairs:
        rod=rule.get("design_tube_od_m"); rid=rule.get("design_tube_id_m")
        sod=sec.get("od_m"); sid=sec.get("id_m")
        if None in (rod,rid,sod,sid):
            mismatches.append(f"{name}: unresolved OD/ID")
            continue
        if abs(float(rod)-float(sod))>tol_m or abs(float(rid)-float(sid))>tol_m:
            mismatches.append(
                f"{name}: mass/CAD {1000*float(rod):.2f}/{1000*float(rid):.2f} mm "
                f"vs verified {1000*float(sod):.2f}/{1000*float(sid):.2f} mm"
            )
    return not mismatches, ("All verified primary sections match mass/CAD geometry." if not mismatches else "; ".join(mismatches))


def _zero_pitch_pair(total:float,xf:float,xr:float,cg:float):
    af,ar=xf-cg,xr-cg
    if not (af<0<ar): return float('nan'),float('nan')
    return total*ar/(2*(ar-af)), total*(-af)/(2*(ar-af))

def _structural_closure(cfg, base_comps, *, max_iterations=8, mass_tol_kg=2e-4, cg_tol_m=2e-5):
    """Close structure mass/CG against the real structural sizing model.

    Structural feedback is always applied to the original unreinforced ledger,
    preventing repeated iterations from double-counting spar/tube deltas.
    """
    d=cfg["design"]
    comps=deepcopy(base_comps); history=[]; structure=None
    for it in range(1,max_iterations+1):
        m=_mass_result(cfg,comps); a,_,_,st=_sync_design(cfg,m)
        layout=lift_layout(a,float(d["lift_propeller_diameter_in"])); rows=[layout.proposed_front_x_m,layout.proposed_rear_x_m]
        lift15=lift_system_sizing(a,mass_kg=float(d["mtow_ceiling_kg"]),prop_diameter_in=float(d["lift_propeller_diameter_in"]))
        # Structural loads retain the 15 kg certification/screening mass, but
        # the member geometry is the ACTUAL current wing. The previous code
        # mixed a 15 kg stall-derived surrogate span into structural geometry.
        st["aircraft"].update({"reference_mass_kg":m.takeoff.total_mass_kg,"cg_station_x_m":m.takeoff.cg_m[0],"design_mass_ceiling_kg":float(d["mtow_ceiling_kg"]),"reference_wing_span_m":float(d["wing_span_m"]),"design_wing_span_m":float(d["wing_span_m"]),"horizontal_tail_area_m2":float(d.get("horizontal_tail_area_m2",st["aircraft"].get("horizontal_tail_area_m2",0.1276)))})
        y=float(a["lift_layout"]["y_abs_m"])
        st["rotor_system"].update({"row_x_m":rows,"motor_y_m":y,"phase4_equal_share_design_thrust_N":lift15.design_thrust_per_motor_N,"candidate_motor_max_thrust_N":float(d["lift_motor_structural_limit_thrust_per_motor_N"])})
        f,r=_zero_pitch_pair(m.takeoff.total_mass_kg*G,rows[0],rows[1],m.takeoff.cg_m[0]); st["rotor_system"]["phase6_reference_hover_thrusts_N"]=[f,f,r,r]
        structure=build_structural_result(st)
        nxt=_apply_structural_feedback(cfg,base_comps,structure); mn=_mass_result(cfg,nxt)
        history.append({"iteration":it,"input_mass_kg":m.takeoff.total_mass_kg,"output_mass_kg":mn.takeoff.total_mass_kg,"input_cg_x_m":m.takeoff.cg_m[0],"output_cg_x_m":mn.takeoff.cg_m[0],"mass_residual_kg":abs(mn.takeoff.total_mass_kg-m.takeoff.total_mass_kg),"cg_residual_m":abs(mn.takeoff.cg_m[0]-m.takeoff.cg_m[0])})
        comps=nxt
        if history[-1]["mass_residual_kg"]<=mass_tol_kg and history[-1]["cg_residual_m"]<=cg_tol_m: break
    return comps,_mass_result(cfg,comps),structure,history

def _main_wing_candidate_metrics(
    base_cfg: Mapping[str, Any],
    *,
    span_m: float,
    root_chord_m: float,
    taper_ratio: float,
    sizing_mass_kg: float,
) -> Dict[str, Any]:
    """Evaluate one coupled main-wing planform at a fixed outer-iteration mass.

    Stall/transition and operational manoeuvre-stall sizing requirements are
    computed from sizing_mass_kg (the current M2 fixed-point mass), not the
    15 kg design ceiling. Structural load cases still use the configured
    15 kg ceiling on the actual candidate geometry.
    """
    values = {
        "main_wing_span_m": float(span_m),
        "main_wing_root_chord_m": float(root_chord_m),
        "main_wing_taper_ratio": float(taper_ratio),
    }
    cfg = design_overrides(base_cfg, values)
    d = cfg["design"]
    base_comps = _components_with_power(cfg)
    comps, mass, structure, closure = _structural_closure(cfg, base_comps)
    aero_cfg, _, six_cfg, _ = _sync_design(cfg, mass)

    V = float(d["cruise_speed_mps"])
    D = float(d["lift_propeller_diameter_in"])
    state = str(d["cruise_lift_propeller_state"])
    drag = cruise_drag(
        aero_cfg,
        mass_kg=mass.takeoff.total_mass_kg,
        speed_mps=V,
        lift_prop_diameter_in=D,
        lift_prop_state=state,
        wing_area_m2=float(d["wing_area_m2"]),
        wing_span_m=float(d["wing_span_m"]),
    )
    req = minimum_wing_area_for_stall_margin(
        aero_cfg, mass_kg=float(sizing_mass_kg), speed_mps=V
    )
    margin = float(d.get("main_wing_auto_area_margin_factor", 1.10))
    stall_required_area = float(req["minimum_wing_area_m2"]) * margin
    manoeuvre_required_area = required_area_for_positive_manoeuvre(
        mass_kg=float(sizing_mass_kg),
        rho_kg_m3=float(aero_cfg["atmosphere"]["rho_kg_m3"]),
        speed_mps=V,
        cl_max=float(d["wing_CLmax"]),
        load_factor=float(d.get("operational_positive_manoeuvre_load_factor_g", 1.5)),
        max_cl_utilization=float(d.get("maximum_manoeuvre_clmax_utilization", 0.90)),
    )
    required_area = max(stall_required_area, manoeuvre_required_area)
    area = float(d["wing_area_m2"])
    tip = float(d["main_wing_root_chord_m"]) * float(d["main_wing_taper_ratio"])
    ar = float(d["wing_span_m"]) ** 2 / max(area, 1e-9)
    sm = (float(d["neutral_point_station_x_m"]) - mass.takeoff.cg_m[0]) / max(float(d["wing_mac_m"]), 1e-9)

    structural_pass = recommended_structure_screen_pass(structure["gates"])
    packaging_pass = int(mass.summary.get("packaging_fail_count", 0)) == 0
    min_sm = float((six_cfg.get("gates") or {}).get("minimum_static_margin", 0.05))
    ceiling = float(d["mtow_ceiling_kg"])
    min_tip = float(d.get("main_wing_minimum_tip_chord_m", 0.15))

    violations = []
    if area + 1e-9 < required_area:
        violations.append("current_iteration_stall_area")
    if tip + 1e-9 < min_tip:
        violations.append("minimum_tip_chord")
    if mass.takeoff.total_mass_kg > ceiling + 1e-9:
        violations.append("mass_ceiling")
    if sm < min_sm:
        violations.append("loaded_static_margin")
    if not structural_pass:
        violations.append("recommended_structure")
    if not packaging_pass:
        violations.append("packaging")

    target_sm = float((cfg.get("optimizer_settings") or {}).get(
        "target_static_margin", d.get("stability_screen_static_margin", 0.12)
    ))
    objective = (
        float(mass.takeoff.total_mass_kg)
        + 0.0020 * max(float(drag.total_cruise_bus_power_W) - 300.0, 0.0)
        + 2.0 * abs(sm - target_sm)
    )
    penalty = 1000.0 * len(violations)
    if area < required_area:
        penalty += 500.0 * ((required_area - area) / max(required_area, 1e-9)) ** 2
    if mass.takeoff.total_mass_kg > ceiling:
        penalty += 500.0 * (mass.takeoff.total_mass_kg - ceiling) ** 2

    return {
        "values": values,
        "score": float(objective + penalty),
        "objective": float(objective),
        "feasible": not violations,
        "violations": violations,
        "sizing_mass_kg": float(sizing_mass_kg),
        "required_area_with_margin_m2": required_area,
        "stall_required_area_with_margin_m2": stall_required_area,
        "manoeuvre_required_area_m2": manoeuvre_required_area,
        "governing_area_constraint": "manoeuvre" if manoeuvre_required_area >= stall_required_area else "stall_transition",
        "wing_area_m2": area,
        "aspect_ratio": ar,
        "tip_chord_m": tip,
        "closed_mass_kg": float(mass.takeoff.total_mass_kg),
        "cg_x_m": float(mass.takeoff.cg_m[0]),
        "static_margin": float(sm),
        "cruise_drag_N": float(drag.total_drag_N),
        "cruise_bus_power_W": float(drag.total_cruise_bus_power_W),
        "wing_spar_od_mm": float(structure["summary"]["recommended_wing_spar_od_mm"]),
        "wing_spar_id_mm": float(structure["summary"]["recommended_wing_spar_id_mm"]),
        "structural_closure_iterations": len(closure),
    }


def _auto_size_main_wing(cfg: Mapping[str, Any]) -> tuple[Dict[str, Any], Dict[str, Any]]:
    """Jointly optimize span, root chord and taper in an outer mass fixed point."""
    clean = deepcopy(dict(cfg))
    d0 = clean["design"]
    if not bool(d0.get("auto_size_main_wing", False)):
        return clean, {"enabled": False, "mode": "hardcoded"}

    seed_cfg = resolve_design_couplings(clean)
    seed_base = _components_with_power(seed_cfg)
    _, seed_mass, _, _ = _structural_closure(seed_cfg, seed_base)
    sizing_mass = float(seed_mass.takeoff.total_mass_kg)

    span_lo, span_hi = [float(x) for x in d0["main_wing_span_bounds_m"]]
    chord_lo, chord_hi = [float(x) for x in d0["main_wing_root_chord_bounds_m"]]
    taper_lo, taper_hi = [float(x) for x in d0["main_wing_taper_ratio_bounds"]]
    n_global = max(int(d0.get("main_wing_autosize_samples", 28)), 8)
    n_local = max(int(d0.get("main_wing_autosize_local_samples", 12)), 0)
    max_outer = max(int(d0.get("main_wing_autosize_max_iterations", 4)), 1)
    min_tip = float(d0.get("main_wing_minimum_tip_chord_m", 0.15))

    # Reuse the same normalized search cloud at every outer M2 iteration.
    # Only the live-M2 area floor moves. This avoids apparent geometry
    # "non-convergence" caused solely by drawing a different random cloud.
    history = []
    best = None
    previous_values = None

    def required_area_for_mass(mass_kg: float) -> float:
        probe_a,_,_,_ = _sync_design(clean)
        req0 = minimum_wing_area_for_stall_margin(
            probe_a, mass_kg=mass_kg, speed_mps=float(d0["cruise_speed_mps"])
        )
        return float(req0["minimum_wing_area_m2"]) * float(d0.get("main_wing_auto_area_margin_factor", 1.10))

    for outer in range(1, max_outer + 1):
        rng = np.random.default_rng(20260919)
        Sreq = required_area_for_mass(sizing_mass)
        candidate_tuples = []
        current_span = float((best or {}).get("values", {}).get("main_wing_span_m", d0["main_wing_span_m"]))
        current_chord = float((best or {}).get("values", {}).get("main_wing_root_chord_m", d0["main_wing_root_chord_m"]))
        current_taper = float((best or {}).get("values", {}).get("main_wing_taper_ratio", d0["main_wing_taper_ratio"]))
        if span_lo <= current_span <= span_hi and chord_lo <= current_chord <= chord_hi and taper_lo <= current_taper <= taper_hi:
            candidate_tuples.append((current_span, current_chord, current_taper))

        def sampled_tuple(span: float, taper: float, fraction: float):
            cr_req_area = 2.0 * Sreq / max(span * (1.0 + taper), 1e-9)
            cr_req_tip = min_tip / max(taper, 1e-9)
            lo = max(chord_lo, cr_req_area, cr_req_tip)
            if lo > chord_hi:
                return None
            cr = lo + float(fraction) * (chord_hi - lo)
            return float(span), float(cr), float(taper)

        for _ in range(n_global):
            span = float(rng.uniform(span_lo, span_hi))
            taper = float(rng.uniform(taper_lo, taper_hi))
            vals = sampled_tuple(span, taper, float(rng.random()))
            if vals is not None:
                candidate_tuples.append(vals)

        evaluated = []
        seen = set()
        for span, cr, taper in candidate_tuples:
            key = (round(span, 6), round(cr, 6), round(taper, 6))
            if key in seen:
                continue
            seen.add(key)
            evaluated.append(_main_wing_candidate_metrics(
                clean, span_m=span, root_chord_m=cr, taper_ratio=taper,
                sizing_mass_kg=sizing_mass,
            ))

        ranked0 = sorted(evaluated, key=lambda x: (not x["feasible"], x["score"]))
        parents = ranked0[:min(4, len(ranked0))]
        widths = (span_hi-span_lo, chord_hi-chord_lo, taper_hi-taper_lo)
        for j in range(n_local):
            if not parents:
                break
            parent = parents[j % len(parents)]["values"]
            span = float(np.clip(parent["main_wing_span_m"] + rng.normal(0.0, 0.04*widths[0]), span_lo, span_hi))
            taper = float(np.clip(parent["main_wing_taper_ratio"] + rng.normal(0.0, 0.04*widths[2]), taper_lo, taper_hi))
            vals = sampled_tuple(span, taper, float(np.clip(0.25 + rng.normal(0.0, 0.20), 0.0, 1.0)))
            if vals is None:
                continue
            span, cr, taper = vals
            evaluated.append(_main_wing_candidate_metrics(
                clean, span_m=span, root_chord_m=cr, taper_ratio=taper,
                sizing_mass_kg=sizing_mass,
            ))

        if not evaluated:
            raise RuntimeError("AutoSize Main Wing found no candidate inside the configured planform bounds")

        ranked = sorted(evaluated, key=lambda x: (not x["feasible"], x["score"]))
        best = ranked[0]
        vals = best["values"]
        new_mass = float(best["closed_mass_kg"])
        geom_delta = 0.0
        if previous_values is not None:
            geom_delta = max(
                abs(float(vals["main_wing_span_m"]) - float(previous_values["main_wing_span_m"])),
                abs(float(vals["main_wing_root_chord_m"]) - float(previous_values["main_wing_root_chord_m"])),
                abs(float(vals["main_wing_taper_ratio"]) - float(previous_values["main_wing_taper_ratio"])),
            )
        history.append({
            "iteration": outer,
            "input_M2_kg": sizing_mass,
            "selected_M2_kg": new_mass,
            "mass_residual_kg": abs(new_mass - sizing_mass),
            "geometry_residual": geom_delta,
            "selected": deepcopy(best),
            "evaluated_candidates": len(evaluated),
            "feasible_candidates": sum(1 for x in evaluated if x["feasible"]),
        })
        if previous_values is not None and abs(new_mass - sizing_mass) <= 0.005 and geom_delta <= 0.002:
            sizing_mass = new_mass
            break
        sizing_mass = new_mass
        previous_values = deepcopy(vals)

    if best is None:
        raise RuntimeError("AutoSize Main Wing failed to produce a selected planform")

    selected_cfg = design_overrides(clean, best["values"])
    selected_cfg["design"]["main_wing_autosize_sizing_mass_kg"] = sizing_mass
    result = {
        "enabled": True,
        "mode": "coupled_span_root_chord_taper",
        "mass_basis": "current_iteration_M2",
        "mtow_ceiling_role": "hard_constraint_and_structural_load_case_only",
        "bounds": {
            "span_m": [span_lo, span_hi],
            "root_chord_m": [chord_lo, chord_hi],
            "taper_ratio": [taper_lo, taper_hi],
        },
        "seed_mass_kg": float(seed_mass.takeoff.total_mass_kg),
        "selected": deepcopy(best),
        "history": history,
        "converged": bool(
            history
            and history[-1]["mass_residual_kg"] <= 0.005
            and history[-1]["geometry_residual"] <= 0.002
        ),
    }
    return selected_cfg, result


def _sample_rows(samples): return [s.to_dict() for s in samples]

def _csv(path:Path,rows:Sequence[Mapping[str,Any]]):
    if not rows:return
    keys=[]
    for r in rows:
        for k in r:
            if k not in keys:keys.append(k)
    with path.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)

def run_model(raw_cfg:Mapping[str,Any], output_dir:str|Path, *, write_histories:bool=True)->Dict[str,Any]:
    cfg=resolve_design_couplings(raw_cfg)
    out=Path(output_dir); out.mkdir(parents=True,exist_ok=True)

    # Optional user-triggered coupled main-wing planform sizing.
    main_wing_autosizing = {"enabled": False, "mode": "hardcoded"}
    if bool(cfg["design"].get("auto_size_main_wing", False)):
        cfg, main_wing_autosizing = _auto_size_main_wing(cfg)
    d=cfg["design"]

    # 1. Base ledger and actual structural mass/CG fixed-point closure.
    base_comps=_components_with_power(cfg); mass_pre=_mass_result(cfg,base_comps)
    comps,mass_final,structure,closure_history=_structural_closure(cfg,base_comps)

    # Structural closure writes the selected primary tube OD/ID back into the
    # manufacturing rules. Rebuild once for its ledger side effect so exported
    # manufacturing_mass_ledger.csv describes the same resolved structure as
    # the final component ledger, rather than the pre-sizing reference tubes.
    _components_with_power(cfg)

    aero_cfg,trans_cfg,six_cfg,struct_cfg=_sync_design(cfg,mass_final)

    # Fuel-cell cooling architecture is evaluated on the resolved package before
    # aero/mission so its side openings contribute to cruise drag.  Hover/ground
    # cooling never credits ram air.
    cooling=analyze_fuel_cell_cooling(cfg,cruise_speed_mps=float(d["cruise_speed_mps"]))
    aero_cfg["aircraft"]["cooling_openings_CdA_m2"]=cooling.cooling_openings_CdA_m2
    landing_cda=sync_landing_legs_drag(cfg)
    aero_cfg["aircraft"]["landing_legs_CdA_m2"]=landing_cda
    if isinstance(cfg,dict):
        cfg.setdefault("_runtime",{})["fuel_cell_cooling"]=cooling.to_dict()

    # 2. Aero/propulsion on the structurally closed aircraft.
    D=float(d["lift_propeller_diameter_in"]); state=str(d["cruise_lift_propeller_state"]); V=float(d["cruise_speed_mps"])
    layout=lift_layout(aero_cfg,D); rows_x=[layout.proposed_front_x_m,layout.proposed_rear_x_m]
    drag=cruise_drag(aero_cfg,mass_kg=mass_final.takeoff.total_mass_kg,speed_mps=V,lift_prop_diameter_in=D,lift_prop_state=state,wing_area_m2=float(d["wing_area_m2"]),wing_span_m=float(d["wing_span_m"]))
    lift_ref=lift_system_sizing(aero_cfg,mass_kg=mass_final.takeoff.total_mass_kg,prop_diameter_in=D)
    lift_ceiling=lift_system_sizing(aero_cfg,mass_kg=float(d["mtow_ceiling_kg"]),prop_diameter_in=D)
    wing_ceiling=minimum_wing_area_for_stall_margin(aero_cfg,mass_kg=float(d["mtow_ceiling_kg"]),speed_mps=V)
    y=float(aero_cfg["lift_layout"]["y_abs_m"]);z=float(aero_cfg["lift_layout"]["z_m"])
    six_cfg["aircraft"]["lift_motor_stations_m"]=[[rows_x[0],-y,z],[rows_x[0],y,z],[rows_x[1],-y,z],[rows_x[1],y,z]]

    # 3. Mission and dynamic hybrid EMS on structurally closed mass.
    mission=build_mission_profile(aero_cfg,trans_cfg,mass_kg=mass_final.takeoff.total_mass_kg,wing_area_m2=float(d["wing_area_m2"]),wing_span_m=float(d["wing_span_m"]),available_lift_thrust_per_motor_N=float(d["lift_motor_available_thrust_per_motor_N"]))
    power_cfg=hybrid_config_from_mapping(cfg["power_system"]); ems=HybridPowerSystem(power_cfg).simulate(mission["profile"],capture_steps=True,max_step_s=0.5)
    h2_end=float(ems.summary.get("h2_remaining_physical_kg",cfg["power_system"]["hydrogen"]["nominal_h2_mass_kg"]))
    for c in comps:
        if c.name=="stored_hydrogen":c.mass_end_kg=h2_end
    mass_final=_mass_result(cfg,comps); aero_cfg,trans_cfg,six_cfg,struct_cfg=_sync_design(cfg,mass_final)
    six_cfg["aircraft"]["lift_motor_stations_m"]=[[rows_x[0],-y,z],[rows_x[0],y,z],[rows_x[1],-y,z],[rows_x[1],y,z]]

    # 4. Nonlinear 6-DOF on the actual closed layout plus explicit stability reference screen.
    # Keep the historical lateral gust as controlled_gust for backward-compatible
    # control gates, and add vertical/combined inspection gusts for stall margin.
    fwd_rows=[x.to_dict() for x in mission["forward_transition"].samples]; rev_rows=[x.to_dict() for x in mission["reverse_transition"].samples]
    current_model=Model(six_cfg); screen_model=Model(six_cfg,static_margin_override=float(d["stability_screen_static_margin"]))
    gust_vectors=six_cfg["simulation"].get("inspection_gust_cases_ned_mps",{"lateral":six_cfg["simulation"]["wind_gust_ned_mps"]})
    control_cases={};control_hist={};control_gust_hist={}
    for label,m in (("current_layout",current_model),("stability_screen",screen_model)):
        trim=m.trim_cruise();fwd=simulate_tracking(m,fwd_rows,final_speed_override=V,max_duration_s=30.0);rev=simulate_tracking(m,rev_rows,final_speed_override=None,max_duration_s=25.0)
        gust_hist={};gust_summaries={}
        for gust_name,gust_vec in gust_vectors.items():
            gs=simulate_cruise_gust(m,speed_mps=V,stabilized=True,gust_ned_mps=gust_vec)
            gust_hist[str(gust_name)]=gs
            gust_summaries[str(gust_name)]=summarize_samples(gs,m)
        gust=gust_hist.get("lateral",next(iter(gust_hist.values())))
        case={"static_margin":m.static_margin,"trim":trim,"forward_transition":summarize_samples(fwd,m),"reverse_transition":summarize_samples(rev,m),"controlled_gust":summarize_samples(gust,m),"inspection_gust_cases":gust_summaries}
        all_gust_samples=[s for rows in gust_hist.values() for s in rows]
        hi_alpha=[abs(x.alpha_deg) for x in (fwd+rev+all_gust_samples) if x.airspeed_mps>=10.0]
        case["linear_aero_validity"]={"maximum_abs_alpha_above_10mps_deg":max(hi_alpha) if hi_alpha else 0.0,"recommended_linear_limit_deg":15.0,"status":"PASS" if (not hi_alpha or max(hi_alpha)<=15.0) else "WARNING","note":"6-DOF coefficient model uses linear aerodynamic derivatives outside the CL clamp; high-alpha results require nonlinear AVL/CFD/wind-tunnel/flight correlation."}
        case["gate_evaluation"]=evaluate_control_case(six_cfg,case,label=label);control_cases[label]=case;control_hist[label]=(fwd,rev,gust);control_gust_hist[label]=gust_hist

    # 5. Final structure report using the closed mass and actual rotor allocator.
    # Use the actual selected wing geometry. 15 kg remains the structural load
    # mass, but must not silently enlarge the beam length via a stall surrogate.
    design_span=float(d["wing_span_m"])
    struct_cfg["aircraft"].update({"reference_mass_kg":mass_final.takeoff.total_mass_kg,"cg_station_x_m":mass_final.takeoff.cg_m[0],"design_mass_ceiling_kg":float(d["mtow_ceiling_kg"]),"reference_wing_span_m":float(d["wing_span_m"]),"design_wing_span_m":design_span,"horizontal_tail_area_m2":float(d.get("horizontal_tail_area_m2",struct_cfg["aircraft"].get("horizontal_tail_area_m2",0.1276)))})
    struct_cfg["rotor_system"].update({"row_x_m":rows_x,"motor_y_m":y,"phase4_equal_share_design_thrust_N":lift_ceiling.design_thrust_per_motor_N,"candidate_motor_max_thrust_N":float(d["lift_motor_structural_limit_thrust_per_motor_N"])})
    hover_thrusts,_=current_model.rotor_allocate(current_model.m*G,np.zeros(3));struct_cfg["rotor_system"]["phase6_reference_hover_thrusts_N"]=hover_thrusts.tolist();structure=build_structural_result(struct_cfg)

    # Re-seat the wing under the recommended primary spar so CAD/viewer do not
    # draw the structural spar through the top longeron sleeve.
    rec_spar_od=float(structure["structural_checks"]["wing"]["recommended_screen"]["section"]["od_m"])
    apply_high_wing_spar_stack(cfg,spar_od_m=rec_spar_od)
    structural_sections_match, structural_sections_note = _structural_section_consistency(cfg, structure)

    total_design=4*float(lift_ceiling.design_thrust_per_motor_N);front_each,rear_each=_zero_pitch_pair(total_design,rows_x[0],rows_x[1],mass_final.takeoff.cg_m[0])
    post_static_margin=(float(d["neutral_point_station_x_m"])-mass_final.takeoff.cg_m[0])/float(d["wing_mac_m"])
    rear_recovery=float(d.get("rear_thrust_recovery_factor", 1.0))
    if str(d.get("lift_prop_mount_side", "under")).lower() == "under" and rear_recovery < 1.0 + 1e-12:
        rear_recovery = 1.15
        d["rear_thrust_recovery_factor"] = rear_recovery
    avail_thrust=float(d["lift_motor_available_thrust_per_motor_N"])
    rear_avail=avail_thrust*rear_recovery

    # Payload-off CG / static-margin gate.
    payload_off_comps=deepcopy(comps)
    for c in payload_off_comps:
        if c.name=="payload" or c.group=="payload":
            c.mass_takeoff_kg=0.0
            c.mass_end_kg=0.0
    mass_payload_off=_mass_result(cfg,payload_off_comps)
    sm_payload_off=(float(d["neutral_point_station_x_m"])-mass_payload_off.takeoff.cg_m[0])/float(d["wing_mac_m"])
    min_sm=float(six_cfg["gates"]["minimum_static_margin"])

    # Defaults preserve legacy/internal configs that predate the user-facing
    # inspection flight-envelope block.
    flight_envelope=evaluate_vn_envelope(
        mass_kg=mass_final.takeoff.total_mass_kg,
        rho_kg_m3=float(aero_cfg["atmosphere"]["rho_kg_m3"]),
        wing_area_m2=float(d["wing_area_m2"]),
        speed_mps=V,
        cl_max=float(d["wing_CLmax"]),
        cl_min=float(d.get("wing_CLmin", six_cfg["aerodynamics"].get("stall_screen_CLmin", six_cfg["aerodynamics"].get("CL_min", -0.6)))),
        positive_limit_g=float(d.get("operational_positive_manoeuvre_load_factor_g", 1.5)),
        negative_limit_g=float(d.get("operational_negative_manoeuvre_load_factor_g", -0.5)),
        max_cl_utilization=float(d.get("maximum_manoeuvre_clmax_utilization", 0.90)),
        maximum_bank_deg=float(d.get("operational_maximum_bank_deg", 45.0)),
    )
    current_gust_cases=control_cases["current_layout"]["inspection_gust_cases"]
    max_gust_cl_util=max(float(x["maximum_clmax_utilization"]) for x in current_gust_cases.values())
    gust_cl_limit=float(d.get("maximum_gust_clmax_utilization", 0.95))

    gates=[]
    def gate(name,status,note):gates.append({"name":name,"status":status,"note":note})
    gate("mass_ceiling","PASS" if mass_final.takeoff.total_mass_kg<=float(d["mtow_ceiling_kg"]) else "FAIL",f"Structurally closed mass {mass_final.takeoff.total_mass_kg:.3f} kg vs {float(d['mtow_ceiling_kg']):.1f} kg ceiling.")
    gate("packaging","PASS" if mass_final.summary["packaging_fail_count"]==0 else "FAIL",f"{mass_final.summary['packaging_fail_count']} packaging collision/containment failures remain.")
    gate("fuel_cell_cooling_airflow_screen","PASS" if cooling.airflow_screen_pass else "FAIL",f"Estimated fan/duct operating flow {cooling.estimated_operating_airflow_cfm:.1f} CFM vs {cooling.required_airflow_cfm:.1f} CFM required by the configured thermal screen.")
    gate("fuel_cell_cooling_geometry","PASS" if cooling.overall_geometry_pass else "FAIL",f"Inlet/exhaust panel + plenum geometry screen; inlet/exhaust side gaps {1000*cooling.inlet_side_plenum_gap_m:.1f}/{1000*cooling.exhaust_side_plenum_gap_m:.1f} mm vs {1000*cooling.required_inlet_side_plenum_gap_m:.1f}/{1000*cooling.required_exhaust_side_plenum_gap_m:.1f} mm required.")
    gate("fuel_cell_cooling_fan_curve","NOT_EVALUATED","IE-SOAR fan/system airflow and allowable external pressure-drop data are not present in the supplied documentation; current airflow capacity is an engineering assumption to be replaced by supplier or bench data.")
    gate("lift_prop_clearance","PASS" if layout.proposed_layout_pass else "FAIL","Collision-free lift-row geometry screen.")
    gate("15kg_stall_margin","PASS" if float(d["wing_area_m2"])+1e-9>=float(wing_ceiling["minimum_wing_area_m2"]) else "FAIL",f"Wing area {float(d['wing_area_m2']):.3f} m² vs minimum {float(wing_ceiling['minimum_wing_area_m2']):.3f} m² screen.")
    gate("inspection_bank_load","PASS" if flight_envelope["bank_within_positive_limit"] else "FAIL",f"{float(d.get('operational_maximum_bank_deg',45.0)):.0f} deg bank requires {flight_envelope['bank_load_factor_g']:.2f} g vs +{flight_envelope['positive_operational_limit_g']:.2f} g operational limit.")
    gate("positive_manoeuvre_stall_margin","PASS" if flight_envelope["positive_manoeuvre_stall_pass"] else "FAIL",f"+{flight_envelope['positive_operational_limit_g']:.2f} g at {V:.1f} m/s uses {100*flight_envelope['positive_manoeuvre_cl_utilization']:.1f}% of CLmax; limit {100*flight_envelope['maximum_cl_utilization']:.0f}%. Minimum speed with margin {flight_envelope['minimum_speed_for_positive_limit_with_margin_mps']:.2f} m/s.")
    gate("negative_manoeuvre_stall_margin","PASS" if flight_envelope["negative_manoeuvre_stall_pass"] else "FAIL",f"{flight_envelope['negative_operational_limit_g']:.2f} g at {V:.1f} m/s uses {100*flight_envelope['negative_manoeuvre_cl_utilization']:.1f}% of |CLmin|; limit {100*flight_envelope['maximum_cl_utilization']:.0f}%.")
    gate("inspection_gust_stall_margin","PASS" if max_gust_cl_util<=gust_cl_limit+1e-9 else "FAIL",f"Worst 6-DOF inspection gust reaches {100*max_gust_cl_util:.1f}% of conservative stall-screen CLmax vs {100*gust_cl_limit:.0f}% limit across lateral, vertical +/- and combined cases.")
    gate("cruise_power","PASS" if drag.total_cruise_bus_power_W<=float(cfg["power_system"]["requirements"]["representative_cruise_power_gate_W"]) else "FAIL",f"{drag.total_cruise_bus_power_W:.1f} W vs configured gate.")
    gate("mission_ems","PASS" if ems.feasible else "FAIL","Dynamic mission profile through hybrid H2/battery EMS.")
    gate("transition","PASS" if mission["forward_transition"].success and mission["reverse_transition"].success else "FAIL","Forward and reverse point-mass transition completion on closed mass.")
    gate("current_longitudinal_stability","PASS" if current_model.static_margin>=float(six_cfg["gates"]["minimum_static_margin"]) else "FAIL",f"Closed-aircraft static margin {100*current_model.static_margin:.2f}% MAC.")
    gate("payload_off_static_margin","PASS" if sm_payload_off>=min_sm else "FAIL",f"Payload-off static margin {100*sm_payload_off:.2f}% MAC (CG x={mass_payload_off.takeoff.cg_m[0]:.4f} m) vs minimum {100*min_sm:.1f}% MAC.")
    gate("current_layout_6dof","PASS" if control_cases["current_layout"]["gate_evaluation"]["pass"] else "FAIL","Actual closed geometry/control 6-DOF gate set.")
    gate("sixdof_linear_aero_validity",control_cases["current_layout"]["linear_aero_validity"]["status"],control_cases["current_layout"]["linear_aero_validity"]["note"]+f" Maximum |alpha| above 10 m/s = {control_cases["current_layout"]["linear_aero_validity"]["maximum_abs_alpha_above_10mps_deg"]:.1f} deg.")
    gate("stability_screen_6dof","PASS" if control_cases["stability_screen"]["gate_evaluation"]["pass"] else "FAIL",f"Explicit +{100*float(d['stability_screen_static_margin']):.0f}% SM reference screen.")
    gate("15kg_zero_pitch_motor_thrust","PASS" if front_each<=avail_thrust+1e-9 and rear_each<=rear_avail+1e-9 else "FAIL",f"Front/rear requirement {front_each:.1f}/{rear_each:.1f} N vs available {avail_thrust:.1f}/{rear_avail:.1f} N (rear recovery x{rear_recovery:.2f}).")
    gate("recommended_structure_screen","PASS" if recommended_structure_screen_pass(structure["gates"]) else "FAIL","Recommended structural member screens. Audit reference/current/legacy sections and the legacy partial-feedback mass gate are excluded because integrated-v2 already closes structural mass explicitly and checks the actual closed MTOW separately.")
    gate("structural_mass_geometry_consistency","PASS" if structural_sections_match else "FAIL",structural_sections_note)
    gate("motor_mount","NOT_EVALUATED","Controlled mount CAD, fasteners and material allowables are still required.")
    selected_battery=cfg.get("hardware_selection_result",{}).get("selected_battery",{})
    battery_known = selected_battery.get("mass_kg") is not None and all(selected_battery.get(k) is not None for k in ("length_m","width_m","height_m"))
    gate("selected_battery_database_record","PASS" if battery_known else "NOT_EVALUATED",
         "Selected battery mass/envelope are present in the controlled hardware database; final release still requires supplier/measured validation." if battery_known else "Selected battery mass/envelope are incomplete.")
    cylinder_known = cfg["power_system"]["hydrogen"].get("cylinder_mass_kg") is not None
    gate("certified_hydrogen_cylinder_mass","PASS" if cylinder_known else "NOT_EVALUATED",
         "Certified cylinder mass is present." if cylinder_known else "Selected certified 3.0 L / 300 bar cylinder mass remains open and is not released.")
    hard=[g for g in gates if g["status"]=="FAIL"]
    status="SCREENING_PASS" if not hard else ("DESIGN_REQUIRES_GEOMETRY_UPDATE" if any(g["name"] in {"packaging","fuel_cell_cooling_geometry","current_longitudinal_stability","payload_off_static_margin","15kg_zero_pitch_motor_thrust","15kg_stall_margin","positive_manoeuvre_stall_margin","negative_manoeuvre_stall_margin","inspection_gust_stall_margin"} for g in hard) else "SCREENING_FAIL")

    usable_h2_Wh=float(ems.summary.get("fc_operational_energy_remaining_Wh", 0.0) or 0.0)
    # Prefer total operational usable energy from hydrogen config when remaining is mid-mission.
    try:
        usable_h2_Wh=float(power_cfg.hydrogen.operational_net_energy_Wh)
    except Exception:
        pass
    cruise_bus_W=float(drag.total_cruise_bus_power_W)
    endurance_at_cruise_h=(usable_h2_Wh/cruise_bus_W) if cruise_bus_W>1e-9 else float("nan")

    result={"model":cfg["model"],"config_hash":_sha(cfg),"summary":{"design_status":status,"reference_mass_kg":mass_pre.takeoff.total_mass_kg,"post_structure_mass_kg":mass_final.takeoff.total_mass_kg,"post_structure_cg_x_m":mass_final.takeoff.cg_m[0],"mass_margin_to_15kg_kg":float(d["mtow_ceiling_kg"])-mass_final.takeoff.total_mass_kg,"current_static_margin":current_model.static_margin,"payload_off_static_margin":sm_payload_off,"post_structure_static_margin_if_np_fixed":post_static_margin,"positive_operational_load_factor_g":flight_envelope["positive_operational_limit_g"],"negative_operational_load_factor_g":flight_envelope["negative_operational_limit_g"],"maximum_bank_deg":flight_envelope["maximum_bank_deg"],"positive_manoeuvre_cl_utilization":flight_envelope["positive_manoeuvre_cl_utilization"],"negative_manoeuvre_cl_utilization":flight_envelope["negative_manoeuvre_cl_utilization"],"worst_gust_cl_utilization":max_gust_cl_util,"cruise_drag_N":drag.total_drag_N,"cruise_bus_power_W":drag.total_cruise_bus_power_W,"endurance_at_cruise_h":endurance_at_cruise_h,"usable_h2_Wh":usable_h2_Wh,"landing_legs_CdA_m2":drag.landing_legs_CdA_m2,"landing_legs_drag_N":drag.landing_legs_drag_N,"forward_transition_duration_s":mission["forward_transition"].duration_s,"reverse_transition_duration_s":mission["reverse_transition"].duration_s,"mission_energy_Wh":mission["total_profile_energy_Wh"],"h2_used_kg":ems.summary.get("h2_used_kg"),"final_battery_soc":ems.summary.get("final_battery_soc"),"wing_spar_od_mm":structure["summary"]["recommended_wing_spar_od_mm"],"wing_spar_id_mm":structure["summary"]["recommended_wing_spar_id_mm"],"horizontal_tail_spar_od_mm":structure["summary"]["horizontal_tail_recommended_spar_od_mm"],"horizontal_tail_spar_id_mm":structure["summary"]["horizontal_tail_recommended_spar_id_mm"],"vertical_tail_spar_od_mm":structure["summary"]["vertical_tail_recommended_spar_od_mm"],"vertical_tail_spar_id_mm":structure["summary"]["vertical_tail_recommended_spar_id_mm"],"boom_od_mm":structure["summary"]["recommended_boom_od_mm"],"boom_id_mm":structure["summary"]["recommended_boom_id_mm"],"motor_mount_limit_thrust_N":structure["summary"]["required_motor_mount_limit_thrust_N"],"landing_leg_od_mm":structure["summary"].get("recommended_landing_leg_od_mm"),"landing_leg_id_mm":structure["summary"].get("recommended_landing_leg_id_mm"),"post_structure_15kg_zero_pitch_front_motor_N":front_each,"post_structure_15kg_zero_pitch_rear_motor_N":rear_each,"rear_thrust_recovery_factor":rear_recovery,"fuel_cell_cooling_required_airflow_cfm":cooling.required_airflow_cfm,"fuel_cell_cooling_estimated_airflow_cfm":cooling.estimated_operating_airflow_cfm,"fuel_cell_cooling_airflow_margin":cooling.airflow_margin_ratio,"fuel_cell_cooling_geometry_pass":cooling.overall_geometry_pass,"fuel_cell_cooling_drag_N":cooling.cooling_drag_at_cruise_N,"structural_closure_iterations":len(closure_history)},"mass_properties":{"pre_structure":mass_pre.to_dict(),"post_structure":mass_final.to_dict()},"coupled_closure":{"converged":bool(closure_history and closure_history[-1]["mass_residual_kg"]<=2e-4 and closure_history[-1]["cg_residual_m"]<=2e-5),"history":closure_history},"aero_propulsion":{"drag":drag.to_dict(),"layout":layout.to_dict(),"lift_reference":lift_ref.to_dict(),"lift_15kg":lift_ceiling.to_dict(),"wing_15kg":wing_ceiling},"mission":{"forward_transition":mission["forward_transition"].to_dict(include_samples=False),"reverse_transition":mission["reverse_transition"].to_dict(include_samples=False),"stage_energy_Wh":mission["stage_energy_Wh"],"total_profile_energy_Wh":mission["total_profile_energy_Wh"],"cruise_bus_power_W":mission["cruise_bus_power_W"]},"hybrid_power":{"feasible":ems.feasible,"failure_reasons":ems.failure_reasons,"warnings":ems.warnings,"summary":ems.summary,"gates":[x.__dict__ for x in ems.gates],"source_ledger":{k:v.__dict__ for k,v in reference_sources().items()}},"sixdof_control":control_cases,"structures":structure,"hardware_selection":cfg.get("hardware_selection_result"),"airfoil_model":cfg.get("airfoil_model"),"fuel_cell_cooling":cooling.to_dict(),"flight_envelope":flight_envelope,"main_wing_autosizing":main_wing_autosizing,"gates":gates,"fidelity_note":"Integrated-v4 closure/optimization is screening fidelity. Airfoil polars are used directly where supplied; missing airfoils retain explicit screening fallbacks. Selected motor/prop/battery data remain subject to supplier/bench validation and pusher forward-flight correlation.","run_environment":{"python":platform.python_version(),"platform":platform.platform()}}

    # Top-level: summary + configs. Supporting artifacts live under details/.
    details = out / "details"
    details.mkdir(parents=True, exist_ok=True)
    write_markdown_report(out / "SUMMARY.md", result, cfg=cfg)
    write_aircraft_specification(out / "aircraft_specification.json", cfg, result)
    resolved = {k: v for k, v in cfg.items() if k != "_runtime"}
    (out / "resolved_configuration.json").write_text(json.dumps(resolved, indent=2, default=str) + "\n", encoding="utf-8")
    if cfg.get("user_input_snapshot") is not None:
        from .user_inputs import write_user_inputs_json
        write_user_inputs_json(out / "user_inputs.json", cfg["user_input_snapshot"])

    (details / "model_results.json").write_text(json.dumps(result, indent=2, default=str) + "\n", encoding="utf-8")
    rows = []
    for c in comps:
        rows.append({
            "name": c.name, "group": c.group,
            "mass_takeoff_kg": c.mass_takeoff_kg, "mass_end_kg": c.mass_end_kg,
            "x_m": c.position_m[0], "y_m": c.position_m[1], "z_m": c.position_m[2],
            "status": c.status, "source": c.source,
        })
    _csv(details / "component_ledger.csv", rows)
    _csv(details / "mission_profile.csv", [{"time_s": p.time_s, "bus_power_W": p.bus_power_W, "stage": p.stage} for p in mission["profile"]])
    _csv(details / "structure_load_cases.csv", structure["load_cases"])
    _csv(details / "closure_history.csv", closure_history)
    if cfg.get("_runtime", {}).get("manufacturing_mass_ledger"):
        _csv(details / "manufacturing_mass_ledger.csv", cfg["_runtime"]["manufacturing_mass_ledger"])
    if cfg.get("user_input_snapshot") is not None:
        from .user_inputs import input_classification
        (details / "INPUT_CLASSIFICATION.json").write_text(json.dumps(input_classification(cfg), indent=2) + "\n", encoding="utf-8")
        auto_values = {
            "geometry": cfg.get("derived_geometry_v3", {}),
            "closed_mtow_kg": mass_final.takeoff.total_mass_kg,
            "closed_cg_m": list(mass_final.takeoff.cg_m),
            "neutral_point_x_m": float(d["neutral_point_station_x_m"]),
            "static_margin": current_model.static_margin,
            "cruise_bus_power_W": drag.total_cruise_bus_power_W,
            "mission_energy_Wh": mission["total_profile_energy_Wh"],
            "h2_used_kg": ems.summary.get("h2_used_kg"),
            "final_battery_soc": ems.summary.get("final_battery_soc"),
            "main_wing_autosizing": main_wing_autosizing,
            "main_wing_span_m": float(d["wing_span_m"]),
            "main_wing_root_chord_m": float(d["main_wing_root_chord_m"]),
            "main_wing_taper_ratio": float(d["main_wing_taper_ratio"]),
            "main_wing_area_m2": float(d["wing_area_m2"]),
            "main_wing_mac_m": float(d["wing_mac_m"]),
            "flight_envelope": flight_envelope,
            "worst_gust_cl_utilization": max_gust_cl_util,
            "wing_spar_od_mm": structure["summary"]["recommended_wing_spar_od_mm"],
            "boom_od_mm": structure["summary"]["recommended_boom_od_mm"],
            "front_motor_15kg_N": front_each,
            "rear_motor_15kg_N": rear_each,
            "fuel_cell_cooling_required_airflow_cfm": cooling.required_airflow_cfm,
            "fuel_cell_cooling_estimated_airflow_cfm": cooling.estimated_operating_airflow_cfm,
            "fuel_cell_cooling_min_fuselage_width_m": cooling.minimum_fuselage_width_for_plenums_m,
            "fuel_cell_cooling_openings_CdA_m2": cooling.cooling_openings_CdA_m2,
            "landing_legs_CdA_m2": drag.landing_legs_CdA_m2,
            "landing_legs_drag_N": drag.landing_legs_drag_N,
        }
        (details / "AUTO_CALCULATED_VALUES.json").write_text(json.dumps(auto_values, indent=2) + "\n", encoding="utf-8")
    if cfg.get("hardware_selection_result") is not None:
        (details / "HARDWARE_SELECTION.json").write_text(json.dumps(cfg["hardware_selection_result"], indent=2, default=str) + "\n", encoding="utf-8")
    (details / "FUEL_CELL_COOLING.json").write_text(json.dumps(cooling.to_dict(), indent=2, default=str) + "\n", encoding="utf-8")
    write_cad_exports(details, cfg, structure)
    if write_histories:
        _csv(details / "transition_forward.csv", [x.to_dict() for x in mission["forward_transition"].samples])
        _csv(details / "transition_reverse.csv", [x.to_dict() for x in mission["reverse_transition"].samples])
        if ems.steps:
            _csv(details / "hybrid_power_history.csv", [x.__dict__ for x in ems.steps])
        for label, (f, r, g) in control_hist.items():
            _csv(details / f"sixdof_forward_{label}.csv", _sample_rows(f))
            _csv(details / f"sixdof_reverse_{label}.csv", _sample_rows(r))
            _csv(details / f"sixdof_gust_{label}.csv", _sample_rows(g))
            for gust_name, rows in control_gust_hist.get(label, {}).items():
                _csv(details / f"sixdof_gust_{gust_name}_{label}.csv", _sample_rows(rows))
    detail_files = sorted(p.name for p in details.iterdir() if p.is_file())
    top_files = sorted(p.name for p in out.iterdir() if p.is_file())
    manifest = {
        "model_version": cfg["model"]["version"],
        "config_hash": result["config_hash"],
        "layout": {"top_level": top_files, "details": detail_files},
        "outputs": top_files + [f"details/{n}" for n in detail_files],
    }
    (details / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return result
