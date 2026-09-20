from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Optional


def _f(v: Any, n: int = 3) -> str:
    try:
        return f"{float(v):.{n}f}"
    except Exception:
        return str(v)


def _pct(v: Any, n: int = 1) -> str:
    try:
        return f"{100.0 * float(v):.{n}f}%"
    except Exception:
        return str(v)


def _get(m: Mapping[str, Any] | None, *keys: str, default: Any = None) -> Any:
    cur: Any = m
    for k in keys:
        if not isinstance(cur, Mapping) or k not in cur:
            return default
        cur = cur[k]
    return cur


def _tip_chord(root: float, taper: float) -> float:
    return float(root) * float(taper)


def write_markdown_report(
    path: str | Path,
    r: Mapping[str, Any],
    *,
    cfg: Optional[Mapping[str, Any]] = None,
) -> None:
    """Write a human-readable aircraft picture + run status summary."""
    s = r["summary"]
    gates = r["gates"]
    d = (cfg or {}).get("design", {}) if cfg else {}
    mm = (cfg or {}).get("manufacturing_mass_model", {}) if cfg else {}
    power = (cfg or {}).get("power_system", {}) if cfg else {}
    geom = _get(cfg, "geometry_mass", "geometry_model", "geometry", default={}) or {}
    pkg_env = _get(cfg, "geometry_mass", "geometry_model", "packaging", "envelopes", "central_pod", default={}) or {}
    lay_cfg = _get(cfg, "aero_propulsion", "lift_layout", default={}) or {}
    layout = _get(r, "aero_propulsion", "layout", default={}) or {}
    hs = r.get("hardware_selection") or {}
    lift = hs.get("selected_lift_combo") or {}
    pusher = hs.get("selected_pusher_combo") or {}
    batt = hs.get("selected_battery") or {}
    mp = _get(r, "mass_properties", "post_structure", default={}) or {}
    groups = mp.get("mass_groups_takeoff_kg") or {}
    cg = _get(mp, "takeoff", "cg_m", default=[None, None, None]) or [None, None, None]
    airfoils = _get(r, "airfoil_model", default={}) or _get(cfg, "airfoil_model", default={}) or {}

    # Geometry essentials
    b = float(d.get("wing_span_m", 0.0) or 0.0)
    S = float(d.get("wing_area_m2", 0.0) or 0.0)
    mac = float(d.get("wing_mac_m", 0.0) or 0.0)
    cr = float(d.get("main_wing_root_chord_m", 0.0) or 0.0)
    lam = float(d.get("main_wing_taper_ratio", 0.0) or 0.0)
    ar = (b * b / S) if S > 1e-12 else 0.0
    bh = float(d.get("horizontal_tail_span_m", 0.0) or 0.0)
    Sh = float(d.get("horizontal_tail_area_m2", 0.0) or 0.0)
    chr_ = float(d.get("horizontal_tail_root_chord_m", 0.0) or 0.0)
    lamh = float(d.get("horizontal_tail_taper_ratio", 0.0) or 0.0)
    hv = float(d.get("vertical_tail_height_m", 0.0) or 0.0)
    Sv = float(d.get("vertical_tail_area_total_m2", 0.0) or 0.0)
    cvr = float(d.get("vertical_tail_root_chord_m", 0.0) or 0.0)
    lamv = float(d.get("vertical_tail_taper_ratio", 0.0) or 0.0)
    ht_ar = (bh * bh / Sh) if Sh > 1e-12 else 0.0
    nfin = max(int(d.get("vertical_tail_count", 2) or 2), 1)
    sv_each = Sv / nfin if nfin else 0.0
    vt_ar = (hv * hv / sv_each) if sv_each > 1e-12 else 0.0
    Lf = float(d.get("fuselage_length_m", 0.0) or 0.0)
    Df = float(d.get("fuselage_max_diameter_m", 0.0) or 0.0)
    boom_y = geom.get("boom_center_y_m") or [-0.35, 0.35]
    boom_span = abs(float(boom_y[1]) - float(boom_y[0])) if isinstance(boom_y, (list, tuple)) and len(boom_y) >= 2 else None
    boom_x0 = geom.get("boom_x_start_m")
    boom_x1 = geom.get("boom_x_end_m")
    y_lift = float(lay_cfg.get("y_abs_m", 0.48) or 0.48)
    wing_z = float(_get(mm, "main_wing", "z_m", default=0.10) or 0.10)
    ht_z = float(_get(mm, "horizontal_tail", "z_m", default=0.03) or 0.03)
    np_x = float(d.get("neutral_point_station_x_m", 0.0) or 0.0)
    pusher_xyz = _get(cfg, "sixdof_control", "aircraft", "pusher_station_m", default=[None, None, None]) or [None, None, None]

    lines = [
        "# H2AIR Integrated Model — Run Summary",
        "",
        f"**Model status:** {s['design_status']}",
        "",
        "## Aircraft at a glance",
        "",
        (
            f"A **{_f(s['post_structure_mass_kg'])} kg** hydrogen hybrid VTOL / fixed-wing UAV "
            f"with a **{_f(b, 2)} m** main wing, **{_f(Lf, 2)} m x {_f(Df * 1000, 0)} mm** central pod, "
            f"twin booms, **4x lift props** and a rear pusher. "
            f"Design cruise **{_f(d.get('cruise_speed_mps', 0), 1)} m/s** for "
            f"**{_f(d.get('mission_duration_min', 0), 0)} min** with **{_f(d.get('payload_mass_kg', 0), 1)} kg** payload."
        ),
        "",
        "## Configuration / look",
        "",
        "### Fuselage / pod",
        f"- Length x outer size: **{_f(Lf, 3)} m x {_f(Df, 3)} m** ({_f(Df * 1000, 0)} mm)"
        + (
            f" ({d.get('fuselage_cross_section', 'square_filleted')}"
            + (
                f", fillet {_f(1000 * float(d.get('fuselage_fillet_radius_m') or 0), 0)} mm"
                if d.get("fuselage_cross_section", "square_filleted") != "circular"
                else ""
            )
            + ")"
        )
        + (
            f" (auto side {_f(1000 * float(d.get('fuselage_max_diameter_m') or 0), 0)} mm, uniform sections)"
            if d.get("fuselage_diameter_mode") == "auto"
            else ""
        )
        + (
            f"; midbody auto lengths: "
            + ", ".join(
                f"{k} {_f(1000 * float(v), 0)} mm"
                for k, v in (d.get("fuselage_section_lengths_m") or {}).items()
                if str(k).startswith("center")
            )
            + (
                f"; component axial {_f(1000 * float(d.get('component_axial_clearance_m') or d.get('energy_bay_axial_clearance_m') or 0), 0)} mm"
                if d.get("component_axial_clearance_m") is not None or d.get("energy_bay_axial_clearance_m") is not None
                else ""
            )
            if d.get("fuselage_midbody_length_mode") == "auto" and d.get("fuselage_section_lengths_m")
            else ""
        ),
        (
            f"- Overall reference length (nose-to-tail layout): "
            f"**{_f(_get(cfg, 'user_input_snapshot', 'fuselage', 'overall_reference_length_m', default=None), 2)} m**"
            if _get(cfg, "user_input_snapshot", "fuselage", "overall_reference_length_m", default=None) is not None
            else ""
        ),
        "",
        "### Main wing",
        (
            f"- AutoSize Main Wing: **ON** — coupled span/root-chord/taper optimization using current iteration M2 "
            f"(final sizing mass **{_f(_get(r, 'main_wing_autosizing', 'selected', 'sizing_mass_kg', default=None), 3)} kg**)"
            if bool(_get(r, "main_wing_autosizing", "enabled", default=False))
            else "- AutoSize Main Wing: **OFF** — hardcoded span/root chord/taper"
        ),
        f"- Span / area / AR: **{_f(b, 3)} m / {_f(S, 3)} m² / {_f(ar, 2)}**",
        f"- Root / tip / MAC: **{_f(cr, 3)} / {_f(_tip_chord(cr, lam), 3)} / {_f(mac, 3)} m** (taper {_f(lam, 2)})",
        f"- Airfoil: **{d.get('main_wing_naca_code', _get(airfoils, 'main_wing', 'code', default='—'))}**"
        + (f", CLmax screen **{_f(d.get('wing_CLmax'), 3)}**" if d.get("wing_CLmax") is not None else ""),
        f"- Aerodynamic centre: **x = {_f(d.get('wing_ac_x_m'), 4)} m**",
        f"- Twist root to tip: **{_f(d.get('main_wing_twist_root_deg', 0), 1)} deg to {_f(d.get('main_wing_twist_tip_deg', 0), 1)} deg**; "
        f"sweep **{_f(d.get('main_wing_sweep_deg', 0), 1)} deg**; dihedral **{_f(d.get('main_wing_dihedral_deg', 0), 1)} deg**",
        f"- Vertical station (structure CG height): **z = {_f(wing_z, 3)} m**",
        "",
        "### Empennage",
        (
            f"- AutoSize Horizontal Tail: **ON** — performance-constrained span/root-chord/taper optimization "
            f"(AR **{_f(ht_ar, 2)}**, resulting Vh **{_f(_get(r,'tail_planform_autosizing','selected','horizontal_tail_volume_coefficient',default=None),3)}**; "
            f"reference **{_f(_get(r,'tail_planform_autosizing','constraints','horizontal_tail_reference_volume_coefficient',default=None),3)}**)"
            if bool(_get(r, "tail_planform_autosizing", "horizontal_tail", default=False))
            else "- AutoSize Horizontal Tail: **OFF** — hardcoded span/root chord/taper"
        ),
        (
            f"- AutoSize Vertical Tail: **ON** — performance-constrained height/root-chord/taper optimization "
            f"(per-fin AR **{_f(vt_ar, 2)}**, resulting Vv **{_f(_get(r,'tail_planform_autosizing','selected','vertical_tail_volume_coefficient',default=None),4)}**; "
            f"reference **{_f(_get(r,'tail_planform_autosizing','constraints','vertical_tail_reference_volume_coefficient',default=None),4)}**)"
            if bool(_get(r, "tail_planform_autosizing", "vertical_tail", default=False))
            else "- AutoSize Vertical Tail: **OFF** — hardcoded height/root chord/taper"
        ),
        (
            f"- Tail control sizing safety factor: **{_f(_get(r,'tail_planform_autosizing','control_safety_factor',default=None),2)}x**; "
            f"Vh/Vv are **outputs/calibration references, not hard minimums**"
            if bool(_get(r, "tail_planform_autosizing", "enabled", default=False))
            else ""
        ),
        f"- Horizontal tail: span **{_f(bh, 3)} m**, area **{_f(Sh, 3)} m²**, "
        f"root/tip **{_f(chr_, 3)}/{_f(_tip_chord(chr_, lamh), 3)} m**, "
        f"**{d.get('horizontal_tail_naca_code', '—')}**, AC **x = {_f(d.get('horizontal_tail_ac_x_m'), 4)} m**, "
        f"z = **{_f(ht_z, 3)} m**",
        f"- Vertical tails: **twin fins**, height **{_f(hv, 3)} m** each, total area **{_f(Sv, 3)} m²**, "
        f"root/tip **{_f(cvr, 3)}/{_f(_tip_chord(cvr, lamv), 3)} m**, "
        f"**{d.get('vertical_tail_naca_code', '—')}**, AC **x = {_f(d.get('vertical_tail_ac_x_m'), 4)} m**",
        "",
        "### Aerodynamic stability/control derivatives",
        f"- Derivative model: **{str(_get(r,'aerodynamic_derivatives','status',default='screening')).replace('_',' ')}** "
        f"(mode **{_get(r,'aerodynamic_derivatives','mode',default='approximate')}**)",
        (
            f"- AVL: **{_get(r,'aerodynamic_derivatives','AVL','status',default='—')}**, "
            f"neutral point **x = {_f(_get(r,'aerodynamic_derivatives','AVL','derivatives','neutral_point_station_x_m',default=None),4)} m**, "
            f"CLa **{_f(_get(r,'aerodynamic_derivatives','active','CL_alpha_per_rad',default=None),3)} /rad**, "
            f"Cmde **{_f(_get(r,'aerodynamic_derivatives','active','Cm_de',default=None),3)} /rad**, "
            f"Cnb **{_f(_get(r,'aerodynamic_derivatives','active','Cn_beta',default=None),3)} /rad**, "
            f"Cndr **{_f(_get(r,'aerodynamic_derivatives','active','Cn_dr',default=None),3)} /rad**"
            if _get(r,'aerodynamic_derivatives','mode',default='approximate') == 'avl'
            else "- AVL not selected; internal screening derivatives active"
        ),
        "- AVL scope: inviscid lifting-surface stability/control derivatives; stall, viscous drag, propeller interaction and separated flow remain separate model/screening calculations.",
        "",
        "### Booms & lift layout",
        (
            f"- Twin boom spacing (centre-to-centre): **{_f(boom_span, 3)} m**"
            if boom_span is not None else "- Twin boom layout"
        ),
        (
            f"- Boom longitudinal extent: **x = {_f(boom_x0, 3)} to {_f(boom_x1, 3)} m**"
            if boom_x0 is not None and boom_x1 is not None else ""
        ),
        f"- Lift-row midpoint: **x = {_f(d.get('lift_row_center_x_m'), 4)} m**; "
        f"front/rear rotor rows at **{_f(layout.get('proposed_front_x_m'), 3)} / {_f(layout.get('proposed_rear_x_m'), 3)} m**",
        f"- Rotor lateral station: **|y| = {_f(y_lift, 3)} m** (boom-mounted through wing); "
        f"motor z = **{_f(lay_cfg.get('z_m'), 3)} m**; "
        f"prop diameter **{_f(layout.get('prop_diameter_in', d.get('lift_propeller_diameter_in')), 1)} in**; "
        f"edge clearance **{_f(1000 * float(layout.get('proposed_edge_clearance_m', 0) or 0), 0)} mm**",
        f"- Cruise lift-prop state: **{d.get('cruise_lift_propeller_state', '—')}**",
        f"- Cruise pusher: fuselage-aft mount at **x = {_f(pusher_xyz[0], 3)} m**",
        f"- Empennage waterline: HT z = **{_f(ht_z, 3)} m** (aligned with main wing z = **{_f(wing_z, 3)} m**)",
        f"- Wing/fuselage junction: fuse top z = **{_f(geom.get('fuselage_top_z_m'), 3)} m**; "
        f"wing drop from flush = **{_f(1000*float(geom.get('wing_drop_from_flush_m') or 0), 1)} mm**; "
        f"longerons **top/bottom** (wing uses top boss only)",
        "",
        "### Energy / avionics packaging (stations)",
        f"- Energy bay order: **{d.get('energy_bay_order', '—')}**"
        + (
            f" (mode {d.get('energy_bay_order_mode', '—')})"
            if d.get("energy_bay_order_mode") is not None else ""
        ),
        (
            f"- Longeron boss cabin clear: **{_f(1000 * float(_get(cfg, 'coupling', 'packaging_layout', 'longeron_boss_clearance_m', default=0) or 0), 0)} mm**; "
            f"Z lifts (mm): "
            + ", ".join(
                f"{k} {_f(1000 * float(v), 1)}"
                for k, v in ((_get(cfg, 'coupling', 'packaging_layout', 'z_lifts_m', default={}) or {}).items())
            )
            if _get(cfg, "coupling", "packaging_layout", "z_lifts_m", default=None) is not None
            else ""
        ),
        f"- Fuel cell: **x = {_f(d.get('fuel_cell_x_m'), 4)} m**, z = **{_f(d.get('fuel_cell_z_m'), 4)} m**"
        + (
            f" ({_f(d.get('fuel_cell_stack_mass_kg'), 3)} kg)"
            if d.get("fuel_cell_stack_mass_kg") is not None else ""
        ),
        (
            f"- FC control box ({d.get('packaging_control_box_site', '—')}): "
            f"**x,y,z = ({_f(d.get('fuel_cell_control_box_x_m'), 4)}, "
            f"{_f(d.get('fuel_cell_control_box_y_m', 0), 4)}, "
            f"{_f(d.get('fuel_cell_control_box_z_m'), 4)}) m**"
            + (
                f" ({_f(d.get('fuel_cell_control_box_mass_kg'), 3)} kg)"
                if d.get("fuel_cell_control_box_mass_kg") is not None else ""
            )
            if d.get("fuel_cell_control_box_x_m") is not None else ""
        ),
        (
            f"- Buffer battery ({d.get('packaging_battery_site', '—')}): "
            f"**x,y,z = ({_f(d.get('battery_x_m'), 4)}, "
            f"{_f(d.get('battery_y_m', 0), 4)}, "
            f"{_f(d.get('battery_z_m'), 4)}) m**"
        ),
        f"- H2 cylinder (+ stored H2): **x = {_f(d.get('hydrogen_cylinder_x_m'), 4)} m**, z = **{_f(d.get('hydrogen_cylinder_z_m'), 4)} m**",
        (
            f"- Payload / Cube Orange: **x,z = ({_f(d.get('payload_x_m'), 4)}, {_f(d.get('payload_z_m'), 4)}) / "
            f"({_f(d.get('cube_orange_x_m'), 4)}, {_f(d.get('cube_orange_z_m'), 4)}) m** "
            f"(nose aft flush x={_f(d.get('nose_section_end_x_m'), 4)} m)"
            if d.get("cube_orange_x_m") is not None
            else ""
        ),
        (
            f"- PDBs: **({_f(d.get('pdb_x_m'), 4)}, {_f(d.get('pdb_y_m', 0), 4)}, {_f(d.get('pdb_z_m'), 4)})**"
            + (
                f" and **({_f(d.get('pdb2_x_m'), 4)}, {_f(d.get('pdb2_y_m', 0), 4)}, {_f(d.get('pdb2_z_m'), 4)})** m"
                if d.get("pdb2_x_m") is not None
                else " m"
            )
            if d.get("pdb_x_m") is not None
            else ""
        ),
        f"- Fuselage wall keepout: **{_f(1000 * float(pkg_env.get('wall_clearance_m') or 0), 1)} mm** "
        f"(circular cross-section; packaging gate fails on radial protrusion)",
        "",
        "## Mass & balance",
        "",
        f"- Closed MTOW: **{_f(s['post_structure_mass_kg'])} kg** "
        f"(pre-structure **{_f(s['reference_mass_kg'])} kg**; margin to ceiling **{_f(s['mass_margin_to_15kg_kg'])} kg**)",
        f"- CG: **({_f(cg[0], 4)}, {_f(cg[1], 4)}, {_f(cg[2], 4)}) m**",
        f"- Neutral point / static margin: **x_NP = {_f(np_x, 4)} m**, **{_f(100 * s['current_static_margin'], 2)}% MAC**",
        "",
        "### Mass breakdown (takeoff)",
    ]

    # Prefer a readable order for mass groups
    order = [
        "payload", "wing_structure", "fuselage_structure", "tail_structure", "landing_gear",
        "energy", "propulsion", "avionics", "airframe",
    ]
    seen = set()
    for name in order + sorted(groups):
        if name in seen or name not in groups:
            continue
        seen.add(name)
        lines.append(f"- {name.replace('_', ' ')}: **{_f(groups[name])} kg**")

    lines += [
        "",
        "## Propulsion & power",
        "",
        f"- Lift set: **{lift.get('name', '—')}** "
        f"({_f(lift.get('prop_diameter_in', d.get('lift_propeller_diameter_in')), 1)} in, "
        f"max thrust **{_f(lift.get('max_thrust_N'), 1)} N/motor**)",
        f"- Pusher set: **{pusher.get('name', '—')}** "
        f"({_f(pusher.get('prop_diameter_in', d.get('pusher_propeller_diameter_in')), 1)} in, "
        f"max thrust **{_f(pusher.get('max_thrust_N'), 1)} N**)",
        f"- Battery: **{batt.get('name', d.get('selected_battery_name', '—'))}** "
        f"({_f(batt.get('mass_kg'), 3)} kg, **{_f(batt.get('energy_Wh'), 1)} Wh**, "
        f"{_f(batt.get('capacity_Ah'), 1)} Ah)",
        f"- Fuel cell: **{_f(_get(power, 'fuel_cell', 'rated_net_power_W'), 0)} W** net, "
        f"module **{_f(_get(power, 'fuel_cell', 'module_mass_kg'), 2)} kg**"
        + (
            f" (stack {_f(_get(power, 'fuel_cell', 'stack_mass_kg'), 3)} kg + "
            f"control {_f(_get(power, 'fuel_cell', 'control_box_mass_kg'), 3)} kg)"
            if _get(power, "fuel_cell", "stack_mass_kg", default=None) is not None
            else ""
        ),
        f"- Hydrogen store: **{_f(_get(power, 'hydrogen', 'water_volume_L'), 1)} L** / "
        f"**{_f(_get(power, 'hydrogen', 'pressure_bar'), 0)} bar**, "
        f"nominal H2 **{_f(1000 * float(_get(power, 'hydrogen', 'nominal_h2_mass_kg', default=0) or 0), 0)} g**"
        + (
            f", cylinder **{_f(_get(power, 'hydrogen', 'cylinder_mass_kg'), 2)} kg**"
            if _get(power, "hydrogen", "cylinder_mass_kg") is not None
            else " (cylinder mass not certified in model)"
        ),
        f"- 15 kg zero-pitch motor need: front/rear **{_f(s['post_structure_15kg_zero_pitch_front_motor_N'], 1)} / "
        f"{_f(s['post_structure_15kg_zero_pitch_rear_motor_N'], 1)} N**",
        "",
        "### Fuel-cell cooling integration",
        f"- Architecture: **{_get(r, 'fuel_cell_cooling', 'architecture', default='—')}**, "
        f"fan direction **{_get(r, 'fuel_cell_cooling', 'fan_flow_direction', default='—')}**",
        f"- Inlet / exhaust sides: **{_get(r, 'fuel_cell_cooling', 'inlet_side', default='—')} / "
        f"{_get(r, 'fuel_cell_cooling', 'exhaust_side', default='—')}**",
        f"- Thermal screen: **{_f(_get(r, 'fuel_cell_cooling', 'waste_heat_design_W'), 0)} W** design waste heat, "
        f"required airflow **{_f(_get(r, 'fuel_cell_cooling', 'required_airflow_cfm'), 1)} CFM**",
        f"- Fan/duct screening flow: **{_f(_get(r, 'fuel_cell_cooling', 'estimated_operating_airflow_cfm'), 1)} CFM** "
        f"(margin **{_f(_get(r, 'fuel_cell_cooling', 'airflow_margin_ratio'), 2)}x**) — engineering assumption until fan data are confirmed",
        f"- Inlet/exhaust free area: **{_f(1e4*float(_get(r, 'fuel_cell_cooling', 'inlet_free_area_m2', default=0) or 0), 1)} / "
        f"{_f(1e4*float(_get(r, 'fuel_cell_cooling', 'exhaust_free_area_m2', default=0) or 0), 1)} cm²**",
        f"- Inlet/exhaust inner side gap: **{_f(1000*float(_get(r, 'fuel_cell_cooling', 'inlet_side_plenum_gap_m', default=0) or 0), 1)} / "
        f"{_f(1000*float(_get(r, 'fuel_cell_cooling', 'exhaust_side_plenum_gap_m', default=0) or 0), 1)} mm**; minimum fuselage width for configured gaps **{_f(1000*float(_get(r, 'fuel_cell_cooling', 'minimum_fuselage_width_for_plenums_m', default=0) or 0), 1)} mm**",
        f"- Cooling-opening cruise drag allowance: **{_f(_get(r, 'fuel_cell_cooling', 'cooling_drag_at_cruise_N'), 3)} N**",
        "",
        "## Performance / mission",
        "",
        f"- Cruise drag / bus power: **{_f(s['cruise_drag_N'], 2)} N / {_f(s['cruise_bus_power_W'], 1)} W** "
        f"(lift props **{d.get('cruise_lift_propeller_state', '—')}**)",
        f"- Inspection manoeuvre envelope: **+{_f(_get(r,'flight_envelope','positive_operational_limit_g'),2)} / {_f(_get(r,'flight_envelope','negative_operational_limit_g'),2)} g**, "
        f"maximum planned bank **{_f(_get(r,'flight_envelope','maximum_bank_deg'),0)} deg** ({_f(_get(r,'flight_envelope','bank_load_factor_g'),2)} g)",
        f"- Accelerated-stall margin at cruise: +g case uses **{_pct(_get(r,'flight_envelope','positive_manoeuvre_cl_utilization'),1)} of CLmax**, "
        f"-g case **{_pct(_get(r,'flight_envelope','negative_manoeuvre_cl_utilization'),1)} of |CLmin|**",
        f"- Gust stall screen: sharp-edge vertical case uses **{_pct(s.get('vertical_gust_cl_utilization'),1)} of CLmax** "
        f"(no alleviation credit); 6-DOF gust cases peak at **{_pct(s.get('sixdof_worst_gust_cl_utilization'),1)}**; "
        f"configured lateral/vertical gusts **{_f(d.get('inspection_lateral_gust_mps'),1)} / ±{_f(d.get('inspection_vertical_gust_mps'),1)} m/s** for **{_f(d.get('inspection_gust_duration_s'),1)} s**",
        (
            f"- Tail authority: **{'PASS' if bool(_get(r,'tail_control_authority','pass',default=False)) else 'FAIL'}** "
            f"with **{_f(_get(r,'tail_control_authority','safety_factor',default=None),2)}x** safety factor; "
            f"trim elevator **{_f(_get(r,'tail_control_authority','trim_elevator_deg',default=None),1)} deg**, "
            f"worst vertical-gust elevator **{_f(_get(r,'tail_control_authority','worst_abs_elevator_deg',default=None),1)} deg**, "
            f"lateral-gust rudder **{_f(_get(r,'tail_control_authority','required_rudder_deg',default=None),1)} deg**"
            if _get(r,'tail_control_authority',default=None) is not None else ""
        ),
        (
            f"- Endurance at cruise (usable H₂ / cruise bus): **{_f(s.get('endurance_at_cruise_h'), 2)} h** "
            f"({_f(s.get('usable_h2_Wh'), 0)} Wh usable)"
            if s.get("endurance_at_cruise_h") is not None else None
        ),
        (
            f"- Landing-leg parasite: **{_f(s.get('landing_legs_drag_N'), 3)} N** "
            f"(CdA **{_f(1e4 * float(s.get('landing_legs_CdA_m2') or 0), 2)} cm²**, sleeve cylinder screen)"
            if s.get('landing_legs_CdA_m2') else None
        ),
        f"- Forward / reverse transition: **{_f(s['forward_transition_duration_s'], 2)} s / "
        f"{_f(s['reverse_transition_duration_s'], 2)} s**",
        f"- Mission energy ({_f(d.get('mission_duration_min', 120), 0)} min): **{_f(s['mission_energy_Wh'], 1)} Wh**",
        f"- H2 used / final battery SOC: **{_f(1000 * float(s.get('h2_used_kg') or 0), 1)} g / "
        f"{_pct(s.get('final_battery_soc'), 1)}**",
        "",
        "## Structures (recommended screen)",
        "",
        f"- Wing spar: **{_f(s['wing_spar_od_mm'], 2)} / {_f(s['wing_spar_id_mm'], 2)} mm OD/ID**",
        f"- Horizontal-tail spar: **{_f(s.get('horizontal_tail_spar_od_mm'), 2)} / {_f(s.get('horizontal_tail_spar_id_mm'), 2)} mm OD/ID**",
        f"- Vertical-tail spars: **{_f(s.get('vertical_tail_spar_od_mm'), 2)} / {_f(s.get('vertical_tail_spar_id_mm'), 2)} mm OD/ID**",
        f"- Boom tube: **{_f(s['boom_od_mm'], 2)} / {_f(s['boom_id_mm'], 2)} mm OD/ID**",
        (
            f"- Landing legs (3g): **{_f(s.get('landing_leg_od_mm'), 2)} / {_f(s.get('landing_leg_id_mm'), 2)} mm OD/ID** "
            f"(CFRP tube + 2 mm printed sleeve; nose + 2 wing)"
            if s.get('landing_leg_od_mm') is not None else None
        ),
        f"- Motor-mount limit thrust: **{_f(s['motor_mount_limit_thrust_N'], 1)} N/motor**",
        "",
        "## Integrated gates",
        "",
    ]

    # Drop empty boom line if we added ""
    lines = [ln for ln in lines if ln is not None]

    for g in gates:
        lines.append(f"- **{g['status']}** — {g['name']}: {g.get('note', '')}")

    lines += [
        "",
        "## Fidelity boundary",
        "",
        str(r.get("fidelity_note", "")),
        "",
    ]

    # Clean accidental blank-only duplicates from optional fields
    cleaned: list[str] = []
    for ln in lines:
        if ln == "" and cleaned and cleaned[-1] == "":
            continue
        cleaned.append(ln)

    Path(path).write_text("\n".join(cleaned), encoding="utf-8")
