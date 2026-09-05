from __future__ import annotations

"""User-facing input layer for H2AIR Integrated v3.

Normal users edit only ``config/user_inputs.json``.  This module converts those
controlled inputs into the detailed internal configuration consumed by the
existing physics modules.  Derived values are written to the output directory;
they are not intended to be hand-edited.
"""

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Mapping
import json
from math import sqrt

from .airfoils import resolve_airfoil_entry
from .hardware_selection import select_hardware, apply_hardware_selection

G = 9.80665
RHO0 = 1.225


def trapezoid_area(span_m: float, root_chord_m: float, taper_ratio: float) -> float:
    return float(span_m * root_chord_m * (1.0 + taper_ratio) / 2.0)


def trapezoid_mac(root_chord_m: float, taper_ratio: float) -> float:
    lam = float(taper_ratio)
    return float((2.0 / 3.0) * root_chord_m * (1.0 + lam + lam * lam) / (1.0 + lam))


def _surface_span(surface: Mapping[str, Any], *, auto_value: float) -> float:
    mode = str(surface.get("span_mode", "auto")).lower()
    if mode == "hardcoded":
        v = surface.get("hardcoded_span_m")
        if v is None or float(v) <= 0:
            raise ValueError("hardcoded span_mode requires hardcoded_span_m > 0")
        return float(v)
    if mode != "auto":
        raise ValueError(f"Unsupported span_mode {mode!r}; use 'auto' or 'hardcoded'")
    lo, hi = [float(x) for x in surface["span_bounds_m"]]
    return float(min(max(auto_value, lo), hi))


def _resolve_surface_airfoil(surface: Mapping[str, Any], airfoil_db: Mapping[str, Any], *, root: Path) -> Dict[str, Any]:
    entry = resolve_airfoil_entry(str(surface["naca_code"]), airfoil_db, root=root)
    if surface.get("CLmax_override") is not None:
        entry["CLmax_used"] = float(surface["CLmax_override"])
        entry["CLmax_source"] = "user_override"
    elif entry.get("polar_rows"):
        entry["CLmax_used"] = float(entry["polar_summary"]["CL_max"])
        entry["CLmax_source"] = "polar_file"
    elif entry.get("CLmax_screen") is not None:
        entry["CLmax_used"] = float(entry["CLmax_screen"])
        entry["CLmax_source"] = "screening_fallback"
    else:
        raise ValueError(f"Airfoil {entry['code']} has neither polar data nor CLmax_override/CLmax_screen")
    return entry


def _load_json(path: Path) -> Dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def build_internal_config(user: Mapping[str, Any], *, root: Path) -> Dict[str, Any]:
    """Build the detailed internal config from the v3 user-facing schema."""
    template_path = root / "config" / "internal_template.json"
    if not template_path.exists():
        raise FileNotFoundError(f"Internal model template not found: {template_path}")
    cfg = _load_json(template_path)
    u = deepcopy(dict(user))

    if str(u.get("schema", "")) not in {"H2AIR-user-inputs-v3", "H2AIR-user-inputs-v4"}:
        raise ValueError("Expected schema='H2AIR-user-inputs-v4' (v3 remains backwards compatible)")

    # -------- Mission / top-level requirements --------
    mission = u["mission"]
    d = cfg["design"]
    d.update({
        "payload_mass_kg": float(mission["payload_mass_kg"]),
        "mtow_target_low_kg": float(mission["mtow_target_low_kg"]),
        "mtow_target_high_kg": float(mission["mtow_target_high_kg"]),
        "mtow_ceiling_kg": float(mission["mtow_ceiling_kg"]),
        "cruise_speed_mps": float(mission["cruise_speed_mps"]),
        "mission_duration_min": float(mission["mission_duration_min"]),
        "stability_screen_static_margin": float(mission["target_static_margin"]),
    })

    # -------- Fuselage geometry (nose/tail lengths fixed; midbody/diameters may auto in packaging) --------
    fus = u["fuselage"]
    sections = fus["sections"]
    total_len = sum(float(x["length_m"]) for x in sections)
    if total_len <= 0:
        raise ValueError("Fuselage section lengths must sum to > 0")
    d["fuselage_diameter_mode"] = str(fus.get("diameter_mode", "hardcoded"))
    d["fuselage_midbody_length_mode"] = str(fus.get("midbody_length_mode", "hardcoded"))
    d["fuselage_minimum_diameter_m"] = float(fus.get("minimum_diameter_m", 0.12))
    d["fuselage_diameter_margin_m"] = float(fus.get("diameter_margin_m", 0.0))
    max_d = max(float(x["diameter_m"]) for x in sections)
    if str(fus.get("diameter_mode", "hardcoded")).lower() == "auto":
        max_d = max(max_d, float(fus.get("minimum_diameter_m", 0.12)))
    d["fuselage_length_m"] = total_len
    d["fuselage_max_diameter_m"] = max_d
    cfg["geometry_mass"]["geometry_model"]["geometry"]["central_pod_outer"] = {
        "center_m": [total_len / 2.0, 0.0, 0.0],
        "dimensions_m": [total_len, max_d, max_d],
    }
    env = cfg["geometry_mass"]["geometry_model"]["packaging"]["envelopes"]["central_pod"]
    # Standard wall keepout matches CAD component_keepout_clearance (default 3 mm).
    keepout_mm = float(
        u.get("cad_export", {}).get("tolerances_mm", {}).get("component_keepout_clearance", 3.0)
    )
    wall_clearance_m = keepout_mm / 1000.0
    env.update({
        "center_m": [total_len / 2.0, 0.0, 0.0],
        "dimensions_m": [total_len, max_d, max_d],
        "wall_clearance_m": wall_clearance_m,
        "note": (
            "Circular fuselage outer envelope with CAD wall keepout; AABB screening alone "
            "does not prove radial fit inside the pod shell."
        ),
    })
    pkg = cfg["geometry_mass"]["geometry_model"]["packaging"]
    pkg["fuselage_cross_section"] = "circular"
    pkg["component_clearance_m"] = wall_clearance_m
    cfg["geometry_mass"]["geometry_model"]["geometry"]["overall_reference_length_m"] = float(fus.get("overall_reference_length_m", max(total_len, 1.15)))
    # Lift-motor station bounds follow the user-fixed fuselage longitudinal envelope.
    # This replaces the legacy Phase-4 hidden 0.18/0.76 m limits; the fuselage is
    # never enlarged to make a selected propeller fit. A layout outside these
    # limits therefore fails rather than resizing the pod.
    cfg["aero_propulsion"]["lift_layout"].update({
        "minimum_front_x_m": 0.0,
        "maximum_rear_x_m": total_len,
        "central_pod_width_m": max_d,
    })

    # -------- Airfoils and lifting-surface geometry --------
    airfoil_db = _load_json(root / "config" / "airfoil_database.json")
    surf = u["lifting_surfaces"]
    main = surf["main_wing"]
    htail = surf["horizontal_tail"]
    vtail = surf["vertical_tail"]

    main_airfoil = _resolve_surface_airfoil(main, airfoil_db, root=root)
    htail_airfoil = _resolve_surface_airfoil(htail, airfoil_db, root=root)
    vtail_airfoil = _resolve_surface_airfoil(vtail, airfoil_db, root=root)
    main_clmax = float(main_airfoil["CLmax_used"])
    d["wing_CLmax"] = main_clmax
    cfg["aero_propulsion"]["aircraft"]["stall_model"] = "direct_CLmax"
    d["main_wing_root_chord_m"] = float(main["root_chord_m"])
    d["main_wing_taper_ratio"] = float(main["taper_ratio"])
    d["main_wing_naca_code"] = str(main["naca_code"])
    cfg["airfoil_model"] = {
        "main_wing": main_airfoil,
        "horizontal_tail": htail_airfoil,
        "vertical_tail": vtail_airfoil,
    }
    if main_airfoil.get("polar_rows"):
        ps = main_airfoil["polar_summary"]
        cfg["transition_mission"]["aerodynamics"]["polar_rows"] = main_airfoil["polar_rows"]
        cfg["transition_mission"]["aerodynamics"]["zero_lift_alpha_deg"] = float(ps["zero_lift_alpha_deg"])
        cfg["transition_mission"]["aerodynamics"]["CL_max"] = float(ps["CL_max"])
        cfg["transition_mission"]["aerodynamics"]["CL_min"] = float(ps["CL_min"])
        cfg["sixdof_control"]["aerodynamics"]["polar_rows"] = main_airfoil["polar_rows"]
        cfg["sixdof_control"]["aerodynamics"]["zero_lift_alpha_deg"] = float(ps["zero_lift_alpha_deg"])
        cfg["sixdof_control"]["aerodynamics"]["CL_max"] = float(ps["CL_max"])
        cfg["sixdof_control"]["aerodynamics"]["CL_min"] = float(ps["CL_min"])
        cfg["sixdof_control"]["aerodynamics"]["Cm0"] = float(ps.get("CM_at_zero_lift",0.0))
    d["main_wing_twist_root_deg"] = float(main.get("twist_root_deg", 0.0))
    d["main_wing_twist_tip_deg"] = float(main.get("twist_tip_deg", 0.0))
    d["main_wing_sweep_deg"] = float(main.get("sweep_deg", 0.0))
    d["main_wing_dihedral_deg"] = float(main.get("dihedral_deg", 0.0))
    d["surface_geometry_mode"] = "root_taper_span"
    d["main_wing_span_mode"] = str(main.get("span_mode", "auto"))
    d["main_wing_span_bounds_m"] = [float(x) for x in main["span_bounds_m"]]

    # Normal-run AUTO sizing starts from the minimum span that gives the configured
    # stall-speed ratio at the 15 kg design ceiling, then applies the requested
    # robust area factor. Full --design-study is still the MDO optimizer.
    V = float(mission["cruise_speed_mps"])
    stall_ratio = float(main.get("minimum_transition_speed_over_stall", 1.30))
    Vstall_max = V / stall_ratio
    Sreq = 2.0 * float(mission["mtow_ceiling_kg"]) * G / (RHO0 * Vstall_max * Vstall_max * main_clmax)
    Sauto = Sreq * float(main.get("auto_area_margin_factor", 1.10))
    bauto = 2.0 * Sauto / (float(main["root_chord_m"]) * (1.0 + float(main["taper_ratio"])))
    bmain = _surface_span(main, auto_value=bauto)
    Smain = trapezoid_area(bmain, float(main["root_chord_m"]), float(main["taper_ratio"]))
    d["main_wing_span_m"] = bmain
    d["wing_span_m"] = bmain
    d["wing_area_m2"] = Smain
    d["wing_mac_m"] = trapezoid_mac(float(main["root_chord_m"]), float(main["taper_ratio"]))

    d["wing_ac_x_m"] = float(main["aerodynamic_center_x_m"])

    # Horizontal tail: AUTO uses a user-defined tail-volume target as a robust
    # starting size. The MDO optimizer can then vary span within the bounds.
    d["horizontal_tail_root_chord_m"] = float(htail["root_chord_m"])
    d["horizontal_tail_taper_ratio"] = float(htail["taper_ratio"])
    d["horizontal_tail_naca_code"] = str(htail["naca_code"])
    d["horizontal_tail_span_mode"] = str(htail.get("span_mode", "auto"))
    d["horizontal_tail_span_bounds_m"] = [float(x) for x in htail["span_bounds_m"]]
    d["horizontal_tail_ac_x_m"] = float(htail["aerodynamic_center_x_m"])
    tail_arm = max(float(htail["aerodynamic_center_x_m"]) - float(main["aerodynamic_center_x_m"]), 0.05)
    Vh = float(htail.get("target_tail_volume_coefficient", 0.50))
    Sh_auto = Vh * Smain * d["wing_mac_m"] / tail_arm
    bh_auto = 2.0 * Sh_auto / (float(htail["root_chord_m"]) * (1.0 + float(htail["taper_ratio"])))
    bh = _surface_span(htail, auto_value=bh_auto)
    Sh = trapezoid_area(bh, float(htail["root_chord_m"]), float(htail["taper_ratio"]))
    d["horizontal_tail_span_m"] = bh
    d["horizontal_tail_area_m2"] = Sh
    d["horizontal_tail_mac_m"] = trapezoid_mac(float(htail["root_chord_m"]), float(htail["taper_ratio"]))

    # Vertical-tail AUTO sizing uses a two-fin vertical-tail-volume screen.
    d["vertical_tail_root_chord_m"] = float(vtail["root_chord_m"])
    d["vertical_tail_taper_ratio"] = float(vtail["taper_ratio"])
    d["vertical_tail_naca_code"] = str(vtail["naca_code"])
    d["vertical_tail_span_mode"] = str(vtail.get("span_mode", "auto"))
    d["vertical_tail_span_bounds_m"] = [float(x) for x in vtail["span_bounds_m"]]
    d["vertical_tail_ac_x_m"] = float(vtail["aerodynamic_center_x_m"])
    nfin = int(vtail.get("count", 2))
    lv = max(float(vtail["aerodynamic_center_x_m"]) - float(main["aerodynamic_center_x_m"]), 0.05)
    Vv = float(vtail.get("target_vertical_tail_volume_coefficient", 0.035))
    Sv_total_auto = Vv * Smain * bmain / lv
    h_auto = 2.0 * (Sv_total_auto / nfin) / (float(vtail["root_chord_m"]) * (1.0 + float(vtail["taper_ratio"])))
    hv = _surface_span(vtail, auto_value=h_auto)
    Sv_each = trapezoid_area(hv, float(vtail["root_chord_m"]), float(vtail["taper_ratio"]))
    d["vertical_tail_height_m"] = hv
    d["vertical_tail_area_each_m2"] = Sv_each
    d["vertical_tail_area_total_m2"] = Sv_each * nfin

    # -------- Propulsion and placement user inputs --------
    prop = u["propulsion"]
    d.update({
        "lift_propeller_diameter_in": float(prop.get("lift_propeller_diameter_in", d["lift_propeller_diameter_in"])),
        "cruise_lift_propeller_state": str(prop.get("cruise_lift_propeller_state", d.get("cruise_lift_propeller_state","folded"))),
        "lift_motor_available_thrust_per_motor_N": float(prop.get("lift_motor_available_thrust_per_motor_N", d["lift_motor_available_thrust_per_motor_N"])),
        "lift_motor_structural_limit_thrust_per_motor_N": float(prop.get("lift_motor_structural_limit_thrust_per_motor_N", d["lift_motor_structural_limit_thrust_per_motor_N"])),
        "pusher_propeller_diameter_in": float(prop.get("pusher_propeller_diameter_in", d["pusher_propeller_diameter_in"])),
        "pusher_max_electrical_power_W": float(prop.get("pusher_max_electrical_power_W", d["pusher_max_electrical_power_W"])),
    })
    pos = u["component_positions"]
    for key in (
        "lift_row_center_x_m", "battery_x_m", "battery_z_m", "fuel_cell_x_m",
        "fuel_cell_z_m", "hydrogen_cylinder_x_m", "hydrogen_cylinder_z_m",
    ):
        d[key] = float(pos[key])

    # -------- Rule-based packaging layout (clearances + PDB / Cube Orange) --------
    if u.get("packaging_layout"):
        cfg["packaging_layout"] = deepcopy(u["packaging_layout"])
        pl = cfg["packaging_layout"]
        order = str(pl.get("energy_bay_order", "fuel_cell_forward")).lower().strip()
        if order not in {"fuel_cell_forward", "hydrogen_forward"}:
            raise ValueError(
                "packaging_layout.energy_bay_order must be 'fuel_cell_forward' or 'hydrogen_forward'"
            )
        pl["energy_bay_order"] = order
        mode = str(pl.get("energy_bay_order_mode", "hardcoded")).lower().strip()
        if mode not in {"hardcoded", "auto"}:
            raise ValueError("packaging_layout.energy_bay_order_mode must be 'hardcoded' or 'auto'")
        pl["energy_bay_order_mode"] = mode
        d["energy_bay_order"] = order
        d["energy_bay_order_mode"] = mode
        d["energy_bay_order_code"] = 0.0 if order == "fuel_cell_forward" else 1.0

        clear = pl.setdefault("clearances_m", {})
        boss_clear = float(clear.get("longeron_boss_clearance_m", 0.01))
        if boss_clear < 0:
            raise ValueError("packaging_layout.clearances_m.longeron_boss_clearance_m must be >= 0")
        clear["longeron_boss_clearance_m"] = boss_clear
        d["longeron_boss_clearance_m"] = boss_clear
        # Prefer component_axial_m; migrate legacy energy_bay_axial_m.
        if "component_axial_m" in clear:
            axial = float(clear["component_axial_m"])
        elif "energy_bay_axial_m" in clear:
            axial = float(clear["energy_bay_axial_m"])
        else:
            axial = float(clear.get("component_m", 0.003))
        if axial < 0:
            raise ValueError("packaging_layout.clearances_m.component_axial_m must be >= 0")
        clear["component_axial_m"] = axial
        clear.pop("energy_bay_axial_m", None)
        d["component_axial_clearance_m"] = axial
        d["energy_bay_axial_clearance_m"] = axial

    # -------- Selected energy-system hardware --------
    hw = u.get("hardware", {})
    if "fuel_cell" in hw:
        for k, v in hw["fuel_cell"].items():
            cfg["power_system"]["fuel_cell"][k] = v
    if "battery" in hw:
        for k, v in hw["battery"].items():
            cfg["power_system"]["battery"][k] = v
    if "hydrogen" in hw:
        for k, v in hw["hydrogen"].items():
            cfg["power_system"]["hydrogen"][k] = v

    # -------- Discrete motor/prop/battery database selection --------
    if u.get("hardware_selection"):
        selection = select_hardware(u, root=root)
        apply_hardware_selection(cfg, selection)

    # -------- Manufacturing / measured linear-mass model --------
    cfg["optimizer_settings"] = deepcopy(u.get("optimizer", {}))
    cfg["manufacturing_mass_model"] = deepcopy(u["manufacturing_mass_model"])
    cfg["manufacturing_mass_model"]["fuselage_total_length_m"] = total_len
    cfg["manufacturing_mass_model"]["fuselage_sections"] = deepcopy(sections)

    # Store geometry definitions and user/derived classification for reporting.
    cfg["user_input_snapshot"] = deepcopy(u)
    cfg["derived_geometry_v3"] = {
        "fuselage_total_length_m": total_len,
        "fuselage_max_diameter_m": max_d,
        "main_wing_span_m": bmain,
        "main_wing_area_m2": Smain,
        "main_wing_mac_m": d["wing_mac_m"],
        "horizontal_tail_span_m": bh,
        "horizontal_tail_area_m2": Sh,
        "vertical_tail_height_m": hv,
        "vertical_tail_area_total_m2": Sv_each * nfin,
        "main_wing_CLmax_screen": main_clmax,
        "airfoils": {
            "main_wing": {"code": main_airfoil["code"], "mode": main_airfoil["aero_source_mode"], "CLmax_used": main_clmax, "Re": main_airfoil.get("reynolds_number")},
            "horizontal_tail": {"code": htail_airfoil["code"], "mode": htail_airfoil["aero_source_mode"], "Re": htail_airfoil.get("reynolds_number")},
            "vertical_tail": {"code": vtail_airfoil["code"], "mode": vtail_airfoil["aero_source_mode"], "Re": vtail_airfoil.get("reynolds_number")},
        },
    }
    cfg["model"].update({
        "version": "H2AIR-integrated-v4.0",
        "description": "Single-entry H2AIR model with separated user inputs, automatic lifting-surface sizing, measured linear-mass manufacturing model, polar-based airfoils, CAD export and discrete hardware selection.",
    })
    return cfg


def load_user_input_config(path: str | Path) -> Dict[str, Any]:
    path = Path(path)
    user = _load_json(path)
    # Resolve internal resources from the installed/repository package, not from
    # wherever the user happens to copy their input JSON.
    root = Path(__file__).resolve().parents[1]
    return build_internal_config(user, root=root)


def _round(x: float, nd: int = 6) -> float:
    return float(round(float(x), nd))


def user_inputs_from_design_values(
    base_user: Mapping[str, Any],
    values: Mapping[str, float],
    *,
    notes: str | None = None,
) -> Dict[str, Any]:
    """Return a user_inputs.json-compatible dict with optimizer design values applied.

    Spans present in ``values`` are locked to ``span_mode='hardcoded'`` so a later
    normal run reproduces the selected sweep geometry instead of re-autosizing.
    """
    u = deepcopy(dict(base_user))
    vals = {str(k): float(v) for k, v in values.items()}
    surf = u.setdefault("lifting_surfaces", {})
    main = surf.setdefault("main_wing", {})
    htail = surf.setdefault("horizontal_tail", {})
    vtail = surf.setdefault("vertical_tail", {})
    pos = u.setdefault("component_positions", {})

    if "wing_ac_x_m" in vals:
        main["aerodynamic_center_x_m"] = _round(vals["wing_ac_x_m"])
    if "horizontal_tail_ac_x_m" in vals:
        htail["aerodynamic_center_x_m"] = _round(vals["horizontal_tail_ac_x_m"])
    if "vertical_tail_ac_x_m" in vals:
        vtail["aerodynamic_center_x_m"] = _round(vals["vertical_tail_ac_x_m"])

    if "main_wing_span_m" in vals:
        main["span_mode"] = "hardcoded"
        main["hardcoded_span_m"] = _round(vals["main_wing_span_m"])
    if "horizontal_tail_span_m" in vals:
        htail["span_mode"] = "hardcoded"
        htail["hardcoded_span_m"] = _round(vals["horizontal_tail_span_m"])
    if "vertical_tail_height_m" in vals:
        vtail["span_mode"] = "hardcoded"
        vtail["hardcoded_span_m"] = _round(vals["vertical_tail_height_m"])

    for key in (
        "lift_row_center_x_m",
        "battery_x_m", "battery_z_m",
        "fuel_cell_x_m", "fuel_cell_z_m",
        "hydrogen_cylinder_x_m", "hydrogen_cylinder_z_m",
    ):
        if key in vals:
            pos[key] = _round(vals[key])

    if "energy_bay_order_code" in vals:
        pl = u.setdefault("packaging_layout", {})
        order = "hydrogen_forward" if float(vals["energy_bay_order_code"]) >= 0.5 else "fuel_cell_forward"
        pl["energy_bay_order"] = order
        pl["energy_bay_order_mode"] = "hardcoded"

    if notes is not None:
        u["notes"] = str(notes)
    elif not str(u.get("notes", "")).strip():
        u["notes"] = "Exported from design-study selection; spans hardcoded to reproduce the chosen layout."
    return u


def write_user_inputs_json(path: str | Path, user: Mapping[str, Any]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(user, indent=2) + "\n", encoding="utf-8")
    return path


def input_classification(cfg: Mapping[str, Any]) -> Dict[str, Any]:
    """Return a concise machine-readable user-input vs derived classification."""
    u = cfg.get("user_input_snapshot", {})
    return {
        "user_input_file_schema": u.get("schema"),
        "user_editable_groups": [
            "mission", "fuselage", "lifting_surfaces", "propulsion",
            "component_positions", "hardware", "hardware_selection", "manufacturing_mass_model", "cad_export", "optimizer",
            "packaging_layout",
        ],
        "auto_calculated_examples": [
            "fuselage total length from section lengths",
            "main wing span in AUTO mode and wing area/MAC",
            "airfoil CL/CD/CM interpolation from a supplied polar when available",
            "AUTO motor/prop/battery database selection when enabled",
            "clean CAD parameter export (geometry/clearance/hardware only)",
            "horizontal-tail span/area in AUTO mode",
            "vertical-tail height/area in AUTO mode",
            "component masses in LINEAR mode from kg/m x length",
            "CG and inertia tensor",
            "neutral point and static margin",
            "aerodynamic drag/cruise power",
            "VTOL thrust/power requirements",
            "mission energy, hydrogen use and battery SOC",
            "structural sizing feedback and converged MTOW",
        ],
        "hardcode_rule": "Any span/mass entry with mode='hardcoded' is treated as fixed and is excluded from automatic sizing for that quantity.",
    }
