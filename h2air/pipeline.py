from __future__ import annotations
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence
import csv, hashlib, json, platform
import numpy as np

from .design_coupling import resolve_design_couplings
from .config import hybrid_config_from_mapping
from .hybrid_power import HybridPowerSystem, reference_sources
from .mass_properties import component_from_mapping, build_mass_geometry_result
from .manufacturing_mass import apply_manufacturing_mass_model, apply_structural_mass_updates
from .aero_propulsion import cruise_drag, lift_layout, lift_system_sizing, minimum_wing_area_for_stall_margin
from .transition import simulate_forward_transition, simulate_reverse_transition
from .mission import build_mission_profile
from .sixdof import Model, simulate_tracking, simulate_cruise_gust, summarize_samples
from .control_gates import evaluate_case as evaluate_control_case
from .structures import build_structural_result
from .report import write_markdown_report
from .cad_export import write_cad_exports
from .fuel_cell_cooling import analyze_fuel_cell_cooling

G=9.80665

def _sha(obj: Any)->str:
    return hashlib.sha256(json.dumps(obj,sort_keys=True,separators=(",",":"),default=str).encode()).hexdigest()

def _sync_design(cfg: Mapping[str,Any], mass_result=None):
    d=cfg["design"]
    a=deepcopy(cfg["aero_propulsion"]); t=deepcopy(cfg["transition_mission"]); s6=deepcopy(cfg["sixdof_control"]); st=deepcopy(cfg["structures"])
    mass=float(mass_result.takeoff.total_mass_kg) if mass_result is not None else float(a["aircraft"]["phase3_screening_mtow_kg"])
    a["aircraft"].update({
        "phase3_screening_mtow_kg":mass,"mtow_ceiling_kg":float(d["mtow_ceiling_kg"]),
        "wing_area_m2":float(d["wing_area_m2"]),"wing_span_m":float(d["wing_span_m"]),"wing_MAC_m":float(d["wing_mac_m"]),
        "wing_CLmax":float(d["wing_CLmax"]),"cruise_speed_mps":float(d["cruise_speed_mps"]),
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
    s6["lift_system"].update({"max_thrust_per_motor_N":float(d["lift_motor_available_thrust_per_motor_N"]),"propeller_diameter_in":float(d["lift_propeller_diameter_in"])})
    s6["pusher"].update({"propeller_diameter_in":float(d["pusher_propeller_diameter_in"]),"max_electrical_power_W":float(d["pusher_max_electrical_power_W"])})
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
        wing15=minimum_wing_area_for_stall_margin(a,mass_kg=float(d["mtow_ceiling_kg"]),speed_mps=float(d["cruise_speed_mps"]))
        st["aircraft"].update({"reference_mass_kg":m.takeoff.total_mass_kg,"cg_station_x_m":m.takeoff.cg_m[0],"design_mass_ceiling_kg":float(d["mtow_ceiling_kg"]),"reference_wing_span_m":float(d["wing_span_m"]),"design_wing_span_m":max(float(d["wing_span_m"]),float(wing15["screened_span_preserving_AR_m"])),"horizontal_tail_area_m2":float(d.get("horizontal_tail_area_m2",st["aircraft"].get("horizontal_tail_area_m2",0.1276)))})
        y=float(a["lift_layout"]["y_abs_m"])
        st["rotor_system"].update({"row_x_m":rows,"motor_y_m":y,"phase4_equal_share_design_thrust_N":lift15.design_thrust_per_motor_N,"candidate_motor_max_thrust_N":float(d["lift_motor_structural_limit_thrust_per_motor_N"])})
        f,r=_zero_pitch_pair(m.takeoff.total_mass_kg*G,rows[0],rows[1],m.takeoff.cg_m[0]); st["rotor_system"]["phase6_reference_hover_thrusts_N"]=[f,f,r,r]
        structure=build_structural_result(st)
        nxt=_apply_structural_feedback(cfg,base_comps,structure); mn=_mass_result(cfg,nxt)
        history.append({"iteration":it,"input_mass_kg":m.takeoff.total_mass_kg,"output_mass_kg":mn.takeoff.total_mass_kg,"input_cg_x_m":m.takeoff.cg_m[0],"output_cg_x_m":mn.takeoff.cg_m[0],"mass_residual_kg":abs(mn.takeoff.total_mass_kg-m.takeoff.total_mass_kg),"cg_residual_m":abs(mn.takeoff.cg_m[0]-m.takeoff.cg_m[0])})
        comps=nxt
        if history[-1]["mass_residual_kg"]<=mass_tol_kg and history[-1]["cg_residual_m"]<=cg_tol_m: break
    return comps,_mass_result(cfg,comps),structure,history

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
    d=cfg["design"]

    # 1. Base ledger and actual structural mass/CG fixed-point closure.
    base_comps=_components_with_power(cfg); mass_pre=_mass_result(cfg,base_comps)
    comps,mass_final,structure,closure_history=_structural_closure(cfg,base_comps)
    aero_cfg,trans_cfg,six_cfg,struct_cfg=_sync_design(cfg,mass_final)

    # Fuel-cell cooling architecture is evaluated on the resolved package before
    # aero/mission so its side openings contribute to cruise drag.  Hover/ground
    # cooling never credits ram air.
    cooling=analyze_fuel_cell_cooling(cfg,cruise_speed_mps=float(d["cruise_speed_mps"]))
    aero_cfg["aircraft"]["cooling_openings_CdA_m2"]=cooling.cooling_openings_CdA_m2
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
    fwd_rows=[x.to_dict() for x in mission["forward_transition"].samples]; rev_rows=[x.to_dict() for x in mission["reverse_transition"].samples]
    current_model=Model(six_cfg); screen_model=Model(six_cfg,static_margin_override=float(d["stability_screen_static_margin"]))
    control_cases={};control_hist={}
    for label,m in (("current_layout",current_model),("stability_screen",screen_model)):
        trim=m.trim_cruise();fwd=simulate_tracking(m,fwd_rows,final_speed_override=V,max_duration_s=30.0);rev=simulate_tracking(m,rev_rows,final_speed_override=None,max_duration_s=25.0);gust=simulate_cruise_gust(m,speed_mps=V,stabilized=True)
        case={"static_margin":m.static_margin,"trim":trim,"forward_transition":summarize_samples(fwd,m),"reverse_transition":summarize_samples(rev,m),"controlled_gust":summarize_samples(gust,m)}
        hi_alpha=[abs(x.alpha_deg) for x in (fwd+rev+gust) if x.airspeed_mps>=10.0]
        case["linear_aero_validity"]={"maximum_abs_alpha_above_10mps_deg":max(hi_alpha) if hi_alpha else 0.0,"recommended_linear_limit_deg":15.0,"status":"PASS" if (not hi_alpha or max(hi_alpha)<=15.0) else "WARNING","note":"6-DOF coefficient model uses linear aerodynamic derivatives outside the CL clamp; high-alpha results require nonlinear AVL/CFD/wind-tunnel/flight correlation."}
        case["gate_evaluation"]=evaluate_control_case(six_cfg,case,label=label);control_cases[label]=case;control_hist[label]=(fwd,rev,gust)

    # 5. Final structure report using the closed mass and actual rotor allocator.
    design_span=max(float(d["wing_span_m"]),float(wing_ceiling["screened_span_preserving_AR_m"]))
    struct_cfg["aircraft"].update({"reference_mass_kg":mass_final.takeoff.total_mass_kg,"cg_station_x_m":mass_final.takeoff.cg_m[0],"design_mass_ceiling_kg":float(d["mtow_ceiling_kg"]),"reference_wing_span_m":float(d["wing_span_m"]),"design_wing_span_m":design_span,"horizontal_tail_area_m2":float(d.get("horizontal_tail_area_m2",struct_cfg["aircraft"].get("horizontal_tail_area_m2",0.1276)))})
    struct_cfg["rotor_system"].update({"row_x_m":rows_x,"motor_y_m":y,"phase4_equal_share_design_thrust_N":lift_ceiling.design_thrust_per_motor_N,"candidate_motor_max_thrust_N":float(d["lift_motor_structural_limit_thrust_per_motor_N"])})
    hover_thrusts,_=current_model.rotor_allocate(current_model.m*G,np.zeros(3));struct_cfg["rotor_system"]["phase6_reference_hover_thrusts_N"]=hover_thrusts.tolist();structure=build_structural_result(struct_cfg)

    total_design=4*float(lift_ceiling.design_thrust_per_motor_N);front_each,rear_each=_zero_pitch_pair(total_design,rows_x[0],rows_x[1],mass_final.takeoff.cg_m[0])
    post_static_margin=(float(d["neutral_point_station_x_m"])-mass_final.takeoff.cg_m[0])/float(d["wing_mac_m"])

    gates=[]
    def gate(name,status,note):gates.append({"name":name,"status":status,"note":note})
    gate("mass_ceiling","PASS" if mass_final.takeoff.total_mass_kg<=float(d["mtow_ceiling_kg"]) else "FAIL",f"Structurally closed mass {mass_final.takeoff.total_mass_kg:.3f} kg vs {float(d['mtow_ceiling_kg']):.1f} kg ceiling.")
    gate("packaging","PASS" if mass_final.summary["packaging_fail_count"]==0 else "FAIL",f"{mass_final.summary['packaging_fail_count']} packaging collision/containment failures remain.")
    gate("fuel_cell_cooling_airflow_screen","PASS" if cooling.airflow_screen_pass else "FAIL",f"Estimated fan/duct operating flow {cooling.estimated_operating_airflow_cfm:.1f} CFM vs {cooling.required_airflow_cfm:.1f} CFM required by the configured thermal screen.")
    gate("fuel_cell_cooling_geometry","PASS" if cooling.overall_geometry_pass else "FAIL",f"Inlet/exhaust panel + plenum geometry screen; inlet/exhaust side gaps {1000*cooling.inlet_side_plenum_gap_m:.1f}/{1000*cooling.exhaust_side_plenum_gap_m:.1f} mm vs {1000*cooling.required_inlet_side_plenum_gap_m:.1f}/{1000*cooling.required_exhaust_side_plenum_gap_m:.1f} mm required.")
    gate("fuel_cell_cooling_fan_curve","NOT_EVALUATED","IE-SOAR fan/system airflow and allowable external pressure-drop data are not present in the supplied documentation; current airflow capacity is an engineering assumption to be replaced by supplier or bench data.")
    gate("lift_prop_clearance","PASS" if layout.proposed_layout_pass else "FAIL","Collision-free lift-row geometry screen.")
    gate("15kg_stall_margin","PASS" if float(d["wing_area_m2"])+1e-9>=float(wing_ceiling["minimum_wing_area_m2"]) else "FAIL",f"Wing area {float(d['wing_area_m2']):.3f} m² vs minimum {float(wing_ceiling['minimum_wing_area_m2']):.3f} m² screen.")
    gate("cruise_power","PASS" if drag.total_cruise_bus_power_W<=float(cfg["power_system"]["requirements"]["representative_cruise_power_gate_W"]) else "FAIL",f"{drag.total_cruise_bus_power_W:.1f} W vs configured gate.")
    gate("mission_ems","PASS" if ems.feasible else "FAIL","Dynamic mission profile through hybrid H2/battery EMS.")
    gate("transition","PASS" if mission["forward_transition"].success and mission["reverse_transition"].success else "FAIL","Forward and reverse point-mass transition completion on closed mass.")
    gate("current_longitudinal_stability","PASS" if current_model.static_margin>=float(six_cfg["gates"]["minimum_static_margin"]) else "FAIL",f"Closed-aircraft static margin {100*current_model.static_margin:.2f}% MAC.")
    gate("current_layout_6dof","PASS" if control_cases["current_layout"]["gate_evaluation"]["pass"] else "FAIL","Actual closed geometry/control 6-DOF gate set.")
    gate("sixdof_linear_aero_validity",control_cases["current_layout"]["linear_aero_validity"]["status"],control_cases["current_layout"]["linear_aero_validity"]["note"]+f" Maximum |alpha| above 10 m/s = {control_cases["current_layout"]["linear_aero_validity"]["maximum_abs_alpha_above_10mps_deg"]:.1f} deg.")
    gate("stability_screen_6dof","PASS" if control_cases["stability_screen"]["gate_evaluation"]["pass"] else "FAIL",f"Explicit +{100*float(d['stability_screen_static_margin']):.0f}% SM reference screen.")
    gate("15kg_zero_pitch_motor_thrust","PASS" if max(front_each,rear_each)<=float(d["lift_motor_available_thrust_per_motor_N"]) else "FAIL",f"Front/rear requirement {front_each:.1f}/{rear_each:.1f} N vs {float(d['lift_motor_available_thrust_per_motor_N']):.1f} N available screen.")
    gate("recommended_structure_screen","PASS" if all(g["status"]!="FAIL" for g in structure["gates"] if "current_" not in g["name"] and "legacy_" not in g["name"] and g["name"]!="mass_ceiling_after_partial_structural_feedback" and g["status"]!="NOT_EVALUATED") else "FAIL","Recommended structural member screens. The legacy partial-feedback mass gate is excluded because integrated-v2 already closes structural mass explicitly and checks the actual closed MTOW separately.")
    gate("motor_mount","NOT_EVALUATED","Controlled mount CAD, fasteners and material allowables are still required.")
    selected_battery=cfg.get("hardware_selection_result",{}).get("selected_battery",{})
    battery_known = selected_battery.get("mass_kg") is not None and all(selected_battery.get(k) is not None for k in ("length_m","width_m","height_m"))
    gate("selected_battery_database_record","PASS" if battery_known else "NOT_EVALUATED",
         "Selected battery mass/envelope are present in the controlled hardware database; final release still requires supplier/measured validation." if battery_known else "Selected battery mass/envelope are incomplete.")
    cylinder_known = cfg["power_system"]["hydrogen"].get("cylinder_mass_kg") is not None
    gate("certified_hydrogen_cylinder_mass","PASS" if cylinder_known else "NOT_EVALUATED",
         "Certified cylinder mass is present." if cylinder_known else "Selected certified 3.0 L / 300 bar cylinder mass remains open and is not released.")
    hard=[g for g in gates if g["status"]=="FAIL"]
    status="SCREENING_PASS" if not hard else ("DESIGN_REQUIRES_GEOMETRY_UPDATE" if any(g["name"] in {"packaging","fuel_cell_cooling_geometry","current_longitudinal_stability","15kg_zero_pitch_motor_thrust","15kg_stall_margin"} for g in hard) else "SCREENING_FAIL")

    result={"model":cfg["model"],"config_hash":_sha(cfg),"summary":{"design_status":status,"reference_mass_kg":mass_pre.takeoff.total_mass_kg,"post_structure_mass_kg":mass_final.takeoff.total_mass_kg,"post_structure_cg_x_m":mass_final.takeoff.cg_m[0],"mass_margin_to_15kg_kg":float(d["mtow_ceiling_kg"])-mass_final.takeoff.total_mass_kg,"current_static_margin":current_model.static_margin,"post_structure_static_margin_if_np_fixed":post_static_margin,"cruise_drag_N":drag.total_drag_N,"cruise_bus_power_W":drag.total_cruise_bus_power_W,"forward_transition_duration_s":mission["forward_transition"].duration_s,"reverse_transition_duration_s":mission["reverse_transition"].duration_s,"mission_energy_Wh":mission["total_profile_energy_Wh"],"h2_used_kg":ems.summary.get("h2_used_kg"),"final_battery_soc":ems.summary.get("final_battery_soc"),"wing_spar_od_mm":structure["summary"]["recommended_wing_spar_od_mm"],"wing_spar_id_mm":structure["summary"]["recommended_wing_spar_id_mm"],"boom_od_mm":structure["summary"]["recommended_boom_od_mm"],"boom_id_mm":structure["summary"]["recommended_boom_id_mm"],"motor_mount_limit_thrust_N":structure["summary"]["required_motor_mount_limit_thrust_N"],"post_structure_15kg_zero_pitch_front_motor_N":front_each,"post_structure_15kg_zero_pitch_rear_motor_N":rear_each,"fuel_cell_cooling_required_airflow_cfm":cooling.required_airflow_cfm,"fuel_cell_cooling_estimated_airflow_cfm":cooling.estimated_operating_airflow_cfm,"fuel_cell_cooling_airflow_margin":cooling.airflow_margin_ratio,"fuel_cell_cooling_geometry_pass":cooling.overall_geometry_pass,"fuel_cell_cooling_drag_N":cooling.cooling_drag_at_cruise_N,"structural_closure_iterations":len(closure_history)},"mass_properties":{"pre_structure":mass_pre.to_dict(),"post_structure":mass_final.to_dict()},"coupled_closure":{"converged":bool(closure_history and closure_history[-1]["mass_residual_kg"]<=2e-4 and closure_history[-1]["cg_residual_m"]<=2e-5),"history":closure_history},"aero_propulsion":{"drag":drag.to_dict(),"layout":layout.to_dict(),"lift_reference":lift_ref.to_dict(),"lift_15kg":lift_ceiling.to_dict(),"wing_15kg":wing_ceiling},"mission":{"forward_transition":mission["forward_transition"].to_dict(include_samples=False),"reverse_transition":mission["reverse_transition"].to_dict(include_samples=False),"stage_energy_Wh":mission["stage_energy_Wh"],"total_profile_energy_Wh":mission["total_profile_energy_Wh"],"cruise_bus_power_W":mission["cruise_bus_power_W"]},"hybrid_power":{"feasible":ems.feasible,"failure_reasons":ems.failure_reasons,"warnings":ems.warnings,"summary":ems.summary,"gates":[x.__dict__ for x in ems.gates],"source_ledger":{k:v.__dict__ for k,v in reference_sources().items()}},"sixdof_control":control_cases,"structures":structure,"hardware_selection":cfg.get("hardware_selection_result"),"airfoil_model":cfg.get("airfoil_model"),"fuel_cell_cooling":cooling.to_dict(),"gates":gates,"fidelity_note":"Integrated-v4 closure/optimization is screening fidelity. Airfoil polars are used directly where supplied; missing airfoils retain explicit screening fallbacks. Selected motor/prop/battery data remain subject to supplier/bench validation and pusher forward-flight correlation.","run_environment":{"python":platform.python_version(),"platform":platform.platform()}}

    # Top-level: summary + configs. Supporting artifacts live under details/.
    details = out / "details"
    details.mkdir(parents=True, exist_ok=True)
    write_markdown_report(out / "SUMMARY.md", result, cfg=cfg)
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
            "wing_spar_od_mm": structure["summary"]["recommended_wing_spar_od_mm"],
            "boom_od_mm": structure["summary"]["recommended_boom_od_mm"],
            "front_motor_15kg_N": front_each,
            "rear_motor_15kg_N": rear_each,
            "fuel_cell_cooling_required_airflow_cfm": cooling.required_airflow_cfm,
            "fuel_cell_cooling_estimated_airflow_cfm": cooling.estimated_operating_airflow_cfm,
            "fuel_cell_cooling_min_fuselage_width_m": cooling.minimum_fuselage_width_for_plenums_m,
            "fuel_cell_cooling_openings_CdA_m2": cooling.cooling_openings_CdA_m2,
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
