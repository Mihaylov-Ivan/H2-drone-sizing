"""Closed-aircraft specification export for a model run.

``aircraft_specification.json`` is the single post-run source of truth for UAV
geometry, mass, performance, hardware and gates. Prefer it over
``resolved_configuration.json``, which also retains template/calibration seeds.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Mapping, Optional


def _get(m: Mapping[str, Any] | None, *keys: str, default: Any = None) -> Any:
    cur: Any = m
    for k in keys:
        if not isinstance(cur, Mapping) or k not in cur:
            return default
        cur = cur[k]
    return cur


def _f(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _tip(root: Any, taper: Any) -> Optional[float]:
    r, t = _f(root), _f(taper)
    if r is None or t is None:
        return None
    return r * t


def build_aircraft_specification(
    cfg: Mapping[str, Any],
    result: Mapping[str, Any],
) -> Dict[str, Any]:
    """Assemble closed-aircraft specs from the run result + resolved design."""
    s = result["summary"]
    d = cfg.get("design") or {}
    mm = cfg.get("manufacturing_mass_model") or {}
    geom = _get(cfg, "geometry_mass", "geometry_model", "geometry", default={}) or {}
    power = cfg.get("power_system") or {}
    h2 = power.get("hydrogen") or {}
    fc = power.get("fuel_cell") or {}
    batt_cfg = power.get("battery") or {}
    hs = result.get("hardware_selection") or cfg.get("hardware_selection_result") or {}
    mp = _get(result, "mass_properties", "post_structure", default={}) or {}
    groups = mp.get("mass_groups_takeoff_kg") or {}
    takeoff = mp.get("takeoff") or {}
    cg = list(takeoff.get("cg_m") or s.get("post_structure_cg_x_m") and [s["post_structure_cg_x_m"], 0.0, 0.0] or [])
    layout = _get(result, "aero_propulsion", "layout", default={}) or {}
    drag = _get(result, "aero_propulsion", "drag", default={}) or {}
    airfoils = result.get("airfoil_model") or cfg.get("airfoil_model") or {}
    cooling = result.get("fuel_cell_cooling") or {}
    ems = _get(result, "hybrid_power", "summary", default={}) or {}
    struct_sum = _get(result, "structures", "summary", default={}) or {}
    ledger = list(_get(cfg, "_runtime", "manufacturing_mass_ledger", default=[]) or [])
    derived = cfg.get("derived_geometry_v3") or {}
    lay_cfg = _get(cfg, "aero_propulsion", "lift_layout", default={}) or {}
    pkg = cfg.get("packaging_layout") or {}

    b = _f(d.get("wing_span_m")) or 0.0
    S = _f(d.get("wing_area_m2")) or 0.0
    ar = (b * b / S) if S > 1e-12 else None

    boom_y = geom.get("boom_center_y_m") or [-0.35, 0.35]
    boom_span = None
    if isinstance(boom_y, (list, tuple)) and len(boom_y) >= 2:
        boom_span = abs(float(boom_y[1]) - float(boom_y[0]))

    def _shell_rule(group: str) -> Dict[str, Any]:
        rule = _get(mm, group, "shell", default={}) or {}
        return {
            "mass_mode": rule.get("mass_mode"),
            "specific_mass_kg_per_m": _f(rule.get("specific_mass_kg_per_m")),
            "source": rule.get("source"),
        }

    lift = hs.get("selected_lift_combo") or {}
    pusher = hs.get("selected_pusher_combo") or {}
    batt = hs.get("selected_battery") or {}

    return {
        "schema": "H2AIR-aircraft-specification-v1",
        "description": (
            "Closed-aircraft source of truth after the latest model run. "
            "Use this file for UAV specs; resolved_configuration.json also contains "
            "template/calibration seeds that are not closed-run values."
        ),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "run": {
            "design_status": s.get("design_status"),
            "model": (cfg.get("model") or {}),
            "config_hash": result.get("config_hash"),
            "fidelity_note": result.get("fidelity_note"),
        },
        "mission": {
            "payload_mass_kg": _f(d.get("payload_mass_kg")),
            "cruise_speed_mps": _f(d.get("cruise_speed_mps")),
            "cruise_speed_kph": (_f(d.get("cruise_speed_mps")) or 0.0) * 3.6,
            "mission_duration_min": _f(d.get("mission_duration_min")),
            "mtow_target_low_kg": _f(d.get("mtow_target_low_kg")),
            "mtow_target_high_kg": _f(d.get("mtow_target_high_kg")),
            "mtow_ceiling_kg": _f(d.get("mtow_ceiling_kg")),
            "target_static_margin": _f(
                _get(cfg, "optimizer_settings", "target_static_margin", default=d.get("stability_screen_static_margin"))
            ),
        },
        "flight_envelope": result.get("flight_envelope") or {},
        "tail_autosizing": result.get("tail_planform_autosizing") or {},
        "tail_control_authority": result.get("tail_control_authority") or {},
        "aerodynamic_derivatives": result.get("aerodynamic_derivatives") or {},
        "geometry": {
            "fuselage": {
                "length_m": _f(d.get("fuselage_length_m")),
                "max_diameter_m": _f(d.get("fuselage_max_diameter_m")),
                "width_m": _f(d.get("fuselage_width_m")),
                "height_m": _f(d.get("fuselage_height_m")),
                "cross_section": d.get("fuselage_cross_section"),
                "fillet_radius_m": _f(d.get("fuselage_fillet_radius_m")),
                "diameter_mode": d.get("fuselage_diameter_mode"),
                "midbody_length_mode": d.get("fuselage_midbody_length_mode"),
                "section_lengths_m": d.get("fuselage_section_lengths_m"),
                "overall_reference_length_m": _f(
                    _get(cfg, "user_input_snapshot", "fuselage", "overall_reference_length_m", default=None)
                ),
            },
            "main_wing": {
                "span_m": b,
                "area_m2": S,
                "aspect_ratio": ar,
                "mac_m": _f(d.get("wing_mac_m")),
                "root_chord_m": _f(d.get("main_wing_root_chord_m")),
                "tip_chord_m": _tip(d.get("main_wing_root_chord_m"), d.get("main_wing_taper_ratio")),
                "taper_ratio": _f(d.get("main_wing_taper_ratio")),
                "ac_x_m": _f(d.get("wing_ac_x_m")),
                "z_m": _f(_get(mm, "main_wing", "z_m", default=None)),
                "CLmax_screen": _f(d.get("wing_CLmax")),
                "naca_code": d.get("main_wing_naca_code") or _get(airfoils, "main_wing", "code"),
                "twist_root_deg": _f(d.get("main_wing_twist_root_deg")),
                "twist_tip_deg": _f(d.get("main_wing_twist_tip_deg")),
                "sweep_deg": _f(d.get("main_wing_sweep_deg")),
                "dihedral_deg": _f(d.get("main_wing_dihedral_deg")),
            },
            "horizontal_tail": {
                "span_m": _f(d.get("horizontal_tail_span_m")),
                "area_m2": _f(d.get("horizontal_tail_area_m2")),
                "mac_m": _f(d.get("horizontal_tail_mac_m")),
                "root_chord_m": _f(d.get("horizontal_tail_root_chord_m")),
                "tip_chord_m": _tip(d.get("horizontal_tail_root_chord_m"), d.get("horizontal_tail_taper_ratio")),
                "taper_ratio": _f(d.get("horizontal_tail_taper_ratio")),
                "ac_x_m": _f(d.get("horizontal_tail_ac_x_m")),
                "z_m": _f(_get(mm, "horizontal_tail", "z_m", default=None)),
                "naca_code": d.get("horizontal_tail_naca_code") or _get(airfoils, "horizontal_tail", "code"),
            },
            "vertical_tail": {
                "count": int(d.get("vertical_tail_count") or _get(mm, "vertical_tail", "count", default=2) or 2),
                "height_m": _f(d.get("vertical_tail_height_m")),
                "area_total_m2": _f(d.get("vertical_tail_area_total_m2")),
                "root_chord_m": _f(d.get("vertical_tail_root_chord_m")),
                "tip_chord_m": _tip(d.get("vertical_tail_root_chord_m"), d.get("vertical_tail_taper_ratio")),
                "taper_ratio": _f(d.get("vertical_tail_taper_ratio")),
                "ac_x_m": _f(d.get("vertical_tail_ac_x_m")),
                "naca_code": d.get("vertical_tail_naca_code") or _get(airfoils, "vertical_tail", "code"),
            },
            "control_surfaces": {
                "elevator_span_fraction": _f(d.get("elevator_span_fraction")),
                "elevator_total_span_m": _f(float(d.get("horizontal_tail_span_m") or 0.0) * float(d.get("elevator_span_fraction") or 0.0)),
                "elevator_chord_fraction": _f(d.get("elevator_chord_fraction")),
                "elevator_hinge_x_over_c": _f(1.0-float(d.get("elevator_chord_fraction") or 0.0)),
                "rudder_height_fraction": _f(d.get("rudder_height_fraction")),
                "rudder_height_m": _f(float(d.get("vertical_tail_height_m") or 0.0) * float(d.get("rudder_height_fraction") or 0.0)),
                "rudder_chord_fraction": _f(d.get("rudder_chord_fraction")),
                "rudder_hinge_x_over_c": _f(1.0-float(d.get("rudder_chord_fraction") or 0.0)),
            },
            "booms_and_lift": {
                "boom_spacing_m": boom_span,
                "boom_x_start_m": _f(geom.get("boom_x_start_m")),
                "boom_x_end_m": _f(geom.get("boom_x_end_m")),
                "boom_z_m": _f(geom.get("boom_z_m")),
                "boom_outer_diameter_m": _f(geom.get("boom_outer_diameter_m") or d.get("boom_outer_diameter_m")),
                "lift_row_center_x_m": _f(d.get("lift_row_center_x_m")),
                "front_rotor_x_m": _f(layout.get("proposed_front_x_m")),
                "rear_rotor_x_m": _f(layout.get("proposed_rear_x_m")),
                "rotor_y_abs_m": _f(lay_cfg.get("y_abs_m")),
                "motor_z_m": _f(lay_cfg.get("z_m")),
                "lift_prop_diameter_in": _f(layout.get("prop_diameter_in") or d.get("lift_propeller_diameter_in")),
                "disk_edge_clearance_m": _f(layout.get("proposed_edge_clearance_m")),
                "cruise_lift_propeller_state": d.get("cruise_lift_propeller_state"),
                "pusher_station_m": list(
                    _get(cfg, "sixdof_control", "aircraft", "pusher_station_m", default=[]) or []
                ),
            },
            "airfoils": airfoils,
            "derived_geometry_v3": derived,
        },
        "mass_balance": {
            "pre_structure_mtow_kg": _f(s.get("reference_mass_kg")),
            "closed_mtow_kg": _f(s.get("post_structure_mass_kg")),
            "mass_margin_to_ceiling_kg": _f(s.get("mass_margin_to_15kg_kg")),
            "cg_m": {
                "x": _f(cg[0]) if len(cg) > 0 else _f(s.get("post_structure_cg_x_m")),
                "y": _f(cg[1]) if len(cg) > 1 else 0.0,
                "z": _f(cg[2]) if len(cg) > 2 else None,
            },
            "neutral_point_x_m": _f(d.get("neutral_point_station_x_m")),
            "static_margin": _f(s.get("current_static_margin")),
            "mass_groups_takeoff_kg": {k: _f(v) for k, v in groups.items()},
            "components_takeoff_kg": {
                str(c.get("name")): _f(c.get("mass_takeoff_kg"))
                for c in (mp.get("components") or [])
                if isinstance(c, Mapping) and c.get("name")
            },
        },
        "manufacturing_mass": {
            "enabled": bool(mm.get("enabled", False)),
            "printed_shells": {
                "fuselage_sections": [
                    {
                        "name": sec.get("name"),
                        "length_m": _f(sec.get("length_m")),
                        "mass": sec.get("mass"),
                    }
                    for sec in (mm.get("fuselage_sections") or [])
                ],
                "main_wing_shell": _shell_rule("main_wing"),
                "horizontal_tail_shell": _shell_rule("horizontal_tail"),
                "vertical_tail_shell": _shell_rule("vertical_tail"),
            },
            "ledger": ledger,
        },
        "packaging": {
            "enabled": bool(pkg.get("enabled", False)),
            "energy_bay_order": d.get("energy_bay_order") or pkg.get("energy_bay_order"),
            "energy_bay_order_mode": pkg.get("energy_bay_order_mode"),
            "stations_m": {
                "fuel_cell": {"x": _f(d.get("fuel_cell_x_m")), "z": _f(d.get("fuel_cell_z_m"))},
                "hydrogen_cylinder": {
                    "x": _f(d.get("hydrogen_cylinder_x_m")),
                    "z": _f(d.get("hydrogen_cylinder_z_m")),
                },
                "battery": {"x": _f(d.get("battery_x_m")), "z": _f(d.get("battery_z_m"))},
                "lift_row_center_x_m": _f(d.get("lift_row_center_x_m")),
            },
            "clearances_m": pkg.get("clearances_m"),
        },
        "propulsion": {
            "lift": {
                "combo_name": lift.get("name") or lift.get("combo_name"),
                "prop_diameter_in": _f(lift.get("prop_diameter_in") or d.get("lift_propeller_diameter_in")),
                "max_thrust_per_motor_N": _f(
                    lift.get("max_thrust_N") or d.get("lift_motor_available_thrust_per_motor_N")
                ),
                "structural_limit_thrust_per_motor_N": _f(
                    d.get("lift_motor_structural_limit_thrust_per_motor_N")
                ),
                "zero_pitch_15kg_front_motor_N": _f(s.get("post_structure_15kg_zero_pitch_front_motor_N")),
                "zero_pitch_15kg_rear_motor_N": _f(s.get("post_structure_15kg_zero_pitch_rear_motor_N")),
                "record": lift or None,
            },
            "pusher": {
                "combo_name": pusher.get("name") or pusher.get("combo_name"),
                "prop_diameter_in": _f(pusher.get("prop_diameter_in") or d.get("pusher_propeller_diameter_in")),
                "max_electrical_power_W": _f(
                    pusher.get("max_electrical_power_W") or d.get("pusher_max_electrical_power_W")
                ),
                "record": pusher or None,
            },
        },
        "energy": {
            "fuel_cell": {
                "net_power_W": _f(fc.get("net_power_W") or fc.get("rated_net_power_W")),
                "module_mass_kg": _f(fc.get("module_mass_kg")),
                "record": {k: fc.get(k) for k in fc if k in (
                    "name", "net_power_W", "rated_net_power_W", "module_mass_kg", "status", "source"
                )} or None,
            },
            "hydrogen": {
                "storage_name": h2.get("storage_name"),
                "pressure_bar": _f(h2.get("pressure_bar")),
                "water_volume_L": _f(h2.get("water_volume_L")),
                "nominal_h2_mass_kg": _f(h2.get("nominal_h2_mass_kg")),
                "cylinder_mass_kg": _f(h2.get("cylinder_mass_kg")),
                "regulator_mass_kg": _f(h2.get("regulator_mass_kg")),
                "reference_net_energy_Wh": _f(h2.get("reference_net_energy_Wh")),
                "mission_energy_reserve_fraction": _f(h2.get("mission_energy_reserve_fraction")),
                "usable_mission_energy_Wh": (
                    (_f(h2.get("reference_net_energy_Wh")) or 0.0)
                    * (1.0 - (_f(h2.get("mission_energy_reserve_fraction")) or 0.0))
                ),
                "cylinder_length_m": _f(h2.get("cylinder_length_m")),
                "cylinder_diameter_m": _f(h2.get("cylinder_diameter_m")),
            },
            "battery": {
                "name": batt.get("name") or batt_cfg.get("name"),
                "mass_kg": _f(batt.get("mass_kg") or batt_cfg.get("mass_kg")),
                "capacity_Ah": _f(batt.get("capacity_Ah") or batt_cfg.get("capacity_Ah")),
                "energy_Wh": _f(batt.get("energy_Wh") or batt_cfg.get("energy_Wh")),
                "record": batt or None,
            },
            "mission_ems": {
                "feasible": _get(result, "hybrid_power", "feasible", default=None),
                "bus_energy_demand_Wh": _f(ems.get("bus_energy_demand_Wh") or s.get("mission_energy_Wh")),
                "fc_net_energy_used_Wh": _f(ems.get("fc_net_energy_used_Wh")),
                "battery_net_energy_out_Wh": _f(ems.get("battery_net_energy_out_Wh")),
                "h2_used_kg": _f(ems.get("h2_used_kg") or s.get("h2_used_kg")),
                "h2_remaining_physical_kg": _f(ems.get("h2_remaining_physical_kg")),
                "final_battery_soc": _f(ems.get("final_battery_soc") or s.get("final_battery_soc")),
                "warnings": list(_get(result, "hybrid_power", "warnings", default=[]) or []),
                "failure_reasons": list(_get(result, "hybrid_power", "failure_reasons", default=[]) or []),
            },
        },
        "performance": {
            "cruise_drag_N": _f(s.get("cruise_drag_N") or drag.get("total_drag_N")),
            "cruise_bus_power_W": _f(s.get("cruise_bus_power_W")),
            "landing_legs_CdA_m2": _f(s.get("landing_legs_CdA_m2")),
            "landing_legs_drag_N": _f(s.get("landing_legs_drag_N")),
            "forward_transition_duration_s": _f(s.get("forward_transition_duration_s")),
            "reverse_transition_duration_s": _f(s.get("reverse_transition_duration_s")),
            "mission_energy_Wh": _f(s.get("mission_energy_Wh")),
            "drag_breakdown": drag or None,
            "mission_stage_energy_Wh": _get(result, "mission", "stage_energy_Wh", default=None),
        },
        "structures": {
            "wing_spar_od_mm": _f(s.get("wing_spar_od_mm") or struct_sum.get("recommended_wing_spar_od_mm")),
            "wing_spar_id_mm": _f(s.get("wing_spar_id_mm") or struct_sum.get("recommended_wing_spar_id_mm")),
            "boom_od_mm": _f(s.get("boom_od_mm") or struct_sum.get("recommended_boom_od_mm")),
            "boom_id_mm": _f(s.get("boom_id_mm") or struct_sum.get("recommended_boom_id_mm")),
            "horizontal_tail_spar_od_mm": _f(struct_sum.get("horizontal_tail_recommended_spar_od_mm")),
            "vertical_tail_spar_od_mm": _f(struct_sum.get("vertical_tail_recommended_spar_od_mm")),
            "landing_leg_od_mm": _f(s.get("landing_leg_od_mm") or struct_sum.get("recommended_landing_leg_od_mm")),
            "landing_leg_id_mm": _f(s.get("landing_leg_id_mm") or struct_sum.get("recommended_landing_leg_id_mm")),
            "motor_mount_limit_thrust_N": _f(
                s.get("motor_mount_limit_thrust_N") or struct_sum.get("required_motor_mount_limit_thrust_N")
            ),
            "carbon_fiber_tubes": cfg.get("carbon_fiber_tubes"),
            "structural_closure_iterations": s.get("structural_closure_iterations"),
        },
        "fuel_cell_cooling": {
            "required_airflow_cfm": _f(s.get("fuel_cell_cooling_required_airflow_cfm")),
            "estimated_airflow_cfm": _f(s.get("fuel_cell_cooling_estimated_airflow_cfm")),
            "airflow_margin": _f(s.get("fuel_cell_cooling_airflow_margin")),
            "geometry_pass": s.get("fuel_cell_cooling_geometry_pass"),
            "cooling_drag_N": _f(s.get("fuel_cell_cooling_drag_N")),
            "detail": cooling or None,
        },
        "gates": list(result.get("gates") or []),
        "hardware_selection": hs or None,
    }


def write_aircraft_specification(
    path: str | Path,
    cfg: Mapping[str, Any],
    result: Mapping[str, Any],
) -> Dict[str, Any]:
    spec = build_aircraft_specification(cfg, result)
    path = Path(path)
    path.write_text(json.dumps(spec, indent=2, default=str) + "\n", encoding="utf-8")
    return spec
