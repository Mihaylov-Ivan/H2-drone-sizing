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
from .fuel_cell_cooling import cooling_opening_cda_from_user_inputs

G = 9.80665
RHO0 = 1.225

# structural_linear members calibrate kg/m at a reference tube OD (not the
# recommended/layout design size). Prefer reference_tube_od_m; migrate legacy
# tube_od_m on load so both naming styles work.
_STRUCTURAL_LINEAR_TUBE_RULES = (
    ("main_wing", "primary_spar"),
    ("horizontal_tail", "primary_spar"),
    ("vertical_tail", "primary_spar"),
)


def _normalize_manufacturing_mass_model(mm_in: Mapping[str, Any]) -> Dict[str, Any]:
    mm = deepcopy(mm_in)
    for group, member in _STRUCTURAL_LINEAR_TUBE_RULES:
        rule = (mm.get(group) or {}).get(member)
        if not isinstance(rule, dict):
            continue
        if rule.get("reference_tube_od_m") is None and rule.get("tube_od_m") is not None:
            rule["reference_tube_od_m"] = rule.pop("tube_od_m")
        elif rule.get("reference_tube_od_m") is not None and "tube_od_m" in rule:
            rule.pop("tube_od_m", None)
    boom = mm.get("boom_tubes")
    if isinstance(boom, dict):
        if boom.get("reference_tube_od_m") is None and boom.get("tube_od_m") is not None:
            boom["reference_tube_od_m"] = boom.pop("tube_od_m")
        elif boom.get("reference_tube_od_m") is not None and "tube_od_m" in boom:
            boom.pop("tube_od_m", None)
    legs = mm.get("landing_legs")
    if isinstance(legs, dict):
        tube = legs.get("tube")
        if isinstance(tube, dict):
            if tube.get("reference_tube_od_m") is None and tube.get("tube_od_m") is not None:
                tube["reference_tube_od_m"] = tube.pop("tube_od_m")
            elif tube.get("reference_tube_od_m") is not None and "tube_od_m" in tube:
                tube.pop("tube_od_m", None)
    return mm


def ceil_even_mm(value_mm: float) -> int:
    """Round *up* to the next even millimetre (off-the-shelf 2 mm OD/ID steps)."""
    from math import ceil
    v = float(value_mm)
    if v <= 0:
        raise ValueError("tube dimension mm must be > 0")
    return int(ceil(v / 2.0) * 2)


def _tube_mm_pair(spec: Mapping[str, Any], *, require_id: bool, require_even_id: bool = True) -> tuple[float, float | None]:
    od = float(spec["od_mm"])
    if od <= 0:
        raise ValueError("carbon_fiber_tubes od_mm must be > 0")
    if od != float(ceil_even_mm(od)):
        raise ValueError(f"carbon_fiber_tubes od_mm must be an even integer mm, got {od}")
    id_mm = spec.get("id_mm")
    if id_mm is None:
        if require_id:
            raise ValueError("carbon_fiber_tubes primary/boom entries require id_mm")
        return od * 1e-3, None
    idv = float(id_mm)
    if idv <= 0 or idv >= od:
        raise ValueError(f"carbon_fiber_tubes id_mm must satisfy 0 < id < od, got od={od} id={idv}")
    if require_even_id and idv != float(ceil_even_mm(idv)):
        raise ValueError(f"carbon_fiber_tubes id_mm must be an even integer mm, got {idv}")
    return od * 1e-3, idv * 1e-3


def apply_carbon_fiber_tubes(cfg: Dict[str, Any], tubes: Mapping[str, Any]) -> None:
    """Lock structural/layout CFRP tube sections from user COTS hardcodes."""
    mode = str(tubes.get("section_mode", "auto")).lower().strip()
    if mode not in {"hardcoded", "auto"}:
        raise ValueError("carbon_fiber_tubes.section_mode must be 'hardcoded' or 'auto'")
    cfg["carbon_fiber_tubes"] = deepcopy(tubes)
    if mode != "hardcoded":
        # Auto mode means the structural solver owns all primary load-carrying
        # tube sections.  Clear any stale hardcoded/design values so a config
        # that was previously run in hardcoded mode cannot silently pin a
        # section during the coupled mass/CG closure.
        st = cfg["structures"]["structure"]
        st["recommended_sections_mode"] = "auto"
        for key in (
            "hardcoded_wing_spar_od_m", "hardcoded_wing_spar_id_m",
            "hardcoded_boom_od_m", "hardcoded_boom_id_m",
            "hardcoded_horizontal_tail_spar_od_m", "hardcoded_horizontal_tail_spar_id_m",
            "hardcoded_vertical_tail_spar_od_m", "hardcoded_vertical_tail_spar_id_m",
            "hardcoded_landing_leg_od_m", "hardcoded_landing_leg_id_m",
            "stepped_wing_primary_spar",
        ):
            st.pop(key, None)

        mm = cfg["manufacturing_mass_model"]
        for rule in (
            mm["main_wing"]["primary_spar"],
            mm["horizontal_tail"]["primary_spar"],
            mm["vertical_tail"]["primary_spar"],
            mm["boom_tubes"],
            (mm.get("landing_legs") or {}).get("tube", {}),
        ):
            if not isinstance(rule, dict):
                continue
            rule.pop("design_tube_od_m", None)
            rule.pop("design_tube_id_m", None)

        # A stepped primary spar is only accepted when its sections are
        # explicitly controlled.  Auto sizing therefore starts from one
        # structurally consistent full-span primary member; a future stepped
        # optimization must verify each segment/load envelope explicitly.
        ps = mm["main_wing"]["primary_spar"]
        ps.pop("stepped", None)
        ps.pop("inboard", None)
        ps.pop("outboard", None)
        ps.pop("joiner_span_fraction", None)
        ps.pop("joiner_mass_kg_each", None)

        # Auto ownership applies only to primary load-carrying members.
        # Secondary spars and fuselage longerons remain explicit user geometry.
        secondary_rules = (
            ("main_wing_secondary_spar", mm["main_wing"]["secondary_spar"]),
            ("horizontal_tail_secondary_spar", mm["horizontal_tail"]["secondary_spar"]),
            ("vertical_tail_secondary_spar", mm["vertical_tail"]["secondary_spar"]),
        )
        for name, rule in secondary_rules:
            if tubes.get(name) is not None:
                od, _ = _tube_mm_pair(tubes[name], require_id=False)
                rule["tube_od_m"] = od
        if tubes.get("fuselage_longerons") is not None:
            long_spec = tubes["fuselage_longerons"]
            long_od, _ = _tube_mm_pair(long_spec, require_id=False)
            mm["fuselage_longerons"]["tube_od_m"] = long_od
            if long_spec.get("boss_od_mm") is not None:
                boss = float(long_spec["boss_od_mm"])
                if boss <= float(long_spec["od_mm"]):
                    raise ValueError("fuselage_longerons boss_od_mm must exceed tube od_mm")
                mm["fuselage_longerons"]["boss_od_m"] = boss * 1e-3
        return

    wing_spec = tubes["main_wing_primary_spar"]
    stepped = bool(wing_spec.get("stepped"))
    # Root bending screen uses the inboard section; flat od/id is accepted as inboard.
    root_spec = wing_spec.get("inboard") if stepped and isinstance(wing_spec.get("inboard"), Mapping) else wing_spec
    if stepped and not isinstance(wing_spec.get("inboard"), Mapping) and wing_spec.get("od_mm") is not None:
        root_spec = wing_spec
    wing_od, wing_id = _tube_mm_pair(root_spec, require_id=True, require_even_id=not stepped)
    boom_od, boom_id = _tube_mm_pair(tubes["boom_tubes"], require_id=True)
    ht_od, ht_id = _tube_mm_pair(tubes["horizontal_tail_primary_spar"], require_id=True)
    vt_od, vt_id = _tube_mm_pair(tubes["vertical_tail_primary_spar"], require_id=True)
    assert wing_id is not None and boom_id is not None and ht_id is not None and vt_id is not None

    st = cfg["structures"]["structure"]
    st["recommended_sections_mode"] = "hardcoded"
    st["hardcoded_wing_spar_od_m"] = wing_od
    st["hardcoded_wing_spar_id_m"] = wing_id
    st["hardcoded_boom_od_m"] = boom_od
    st["hardcoded_boom_id_m"] = boom_id
    st["hardcoded_horizontal_tail_spar_od_m"] = ht_od
    st["hardcoded_horizontal_tail_spar_id_m"] = ht_id
    st["hardcoded_vertical_tail_spar_od_m"] = vt_od
    st["hardcoded_vertical_tail_spar_id_m"] = vt_id

    # Layout / packaging use design boom OD (not the 35 mm audit reference).
    geom = cfg["geometry_mass"]["geometry_model"]["geometry"]
    geom["boom_outer_diameter_m"] = boom_od
    cfg["design"]["boom_outer_diameter_m"] = boom_od

    mm = cfg["manufacturing_mass_model"]
    # Primary spars: keep reference_* as mass-calibration sections; CAD/layout OD
    # comes from structures hardcoded recommended + design coupling.
    ps = mm["main_wing"]["primary_spar"]
    ps["design_tube_od_m"] = wing_od
    ps["design_tube_id_m"] = wing_id
    if stepped:
        inb_spec = wing_spec.get("inboard") if isinstance(wing_spec.get("inboard"), Mapping) else root_spec
        out_spec = wing_spec.get("outboard")
        if not isinstance(out_spec, Mapping):
            raise ValueError("stepped main_wing_primary_spar requires outboard {od_mm,id_mm}")
        in_od, in_id = _tube_mm_pair(inb_spec, require_id=True, require_even_id=False)
        out_od, out_id = _tube_mm_pair(out_spec, require_id=True, require_even_id=False)
        assert in_id is not None and out_id is not None
        ps["stepped"] = True
        ps["joiner_span_fraction"] = float(wing_spec.get("joiner_span_fraction", 0.45))
        ps["joiner_mass_kg_each"] = float(wing_spec.get("joiner_mass_kg_each", 0.06))
        ps["inboard"] = {"design_tube_od_m": in_od, "design_tube_id_m": in_id, "od_mm": float(inb_spec["od_mm"]), "id_mm": float(inb_spec["id_mm"])}
        ps["outboard"] = {"design_tube_od_m": out_od, "design_tube_id_m": out_id, "od_mm": float(out_spec["od_mm"]), "id_mm": float(out_spec["id_mm"])}
        st["stepped_wing_primary_spar"] = {
            "enabled": True,
            "joiner_span_fraction": ps["joiner_span_fraction"],
            "inboard_od_m": in_od,
            "inboard_id_m": in_id,
            "outboard_od_m": out_od,
            "outboard_id_m": out_id,
            "joiner_mass_kg_each": ps["joiner_mass_kg_each"],
        }
        # Flat root OD/ID for structures screen / backward compat.
        if wing_spec.get("od_mm") is None:
            tubes = dict(tubes)
            cfg["carbon_fiber_tubes"]["main_wing_primary_spar"] = {
                **deepcopy(wing_spec),
                "od_mm": float(inb_spec["od_mm"]),
                "id_mm": float(inb_spec["id_mm"]),
            }
    else:
        ps.pop("stepped", None)
        st.pop("stepped_wing_primary_spar", None)
    mm["horizontal_tail"]["primary_spar"]["design_tube_od_m"] = ht_od
    mm["horizontal_tail"]["primary_spar"]["design_tube_id_m"] = ht_id
    mm["vertical_tail"]["primary_spar"]["design_tube_od_m"] = vt_od
    mm["vertical_tail"]["primary_spar"]["design_tube_id_m"] = vt_id
    mm["boom_tubes"]["design_tube_od_m"] = boom_od
    mm["boom_tubes"]["design_tube_id_m"] = boom_id

    wsec_od, _ = _tube_mm_pair(tubes["main_wing_secondary_spar"], require_id=False)
    hsec_od, _ = _tube_mm_pair(tubes["horizontal_tail_secondary_spar"], require_id=False)
    vsec_od, _ = _tube_mm_pair(tubes["vertical_tail_secondary_spar"], require_id=False)
    mm["main_wing"]["secondary_spar"]["tube_od_m"] = wsec_od
    mm["horizontal_tail"]["secondary_spar"]["tube_od_m"] = hsec_od
    mm["vertical_tail"]["secondary_spar"]["tube_od_m"] = vsec_od

    long_spec = tubes["fuselage_longerons"]
    long_od, _ = _tube_mm_pair(long_spec, require_id=False)
    mm["fuselage_longerons"]["tube_od_m"] = long_od
    if long_spec.get("boss_od_mm") is not None:
        boss = float(long_spec["boss_od_mm"])
        if boss <= long_spec["od_mm"]:
            raise ValueError("fuselage_longerons boss_od_mm must exceed tube od_mm")
        mm["fuselage_longerons"]["boss_od_m"] = boss * 1e-3

    # Optional COTS landing-leg tube lock (independent of wing/boom hardcodes).
    if tubes.get("landing_legs") is not None:
        land_od, land_id = _tube_mm_pair(tubes["landing_legs"], require_id=True)
        assert land_id is not None
        st["hardcoded_landing_leg_od_m"] = land_od
        st["hardcoded_landing_leg_id_m"] = land_id
        if "landing_legs" in mm and isinstance(mm["landing_legs"], dict):
            tube = mm["landing_legs"].setdefault("tube", {})
            tube["design_tube_od_m"] = land_od
            tube["design_tube_id_m"] = land_id


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

    # -------- Inspection-aircraft operational flight envelope --------
    fe = u.get("flight_envelope") or {}
    pos_g = float(fe.get("positive_manoeuvre_load_factor_g", 1.5))
    neg_g = float(fe.get("negative_manoeuvre_load_factor_g", -0.5))
    max_bank = float(fe.get("maximum_bank_deg", 45.0))
    cl_util = float(fe.get("maximum_manoeuvre_clmax_utilization", 0.90))
    gust_cl_util = float(fe.get("maximum_gust_clmax_utilization", 0.95))
    lateral_gust = float(fe.get("lateral_gust_mps", 5.0))
    vertical_gust = float(fe.get("vertical_gust_mps", 2.0))
    gust_duration = float(fe.get("gust_duration_s", 2.0))
    structural_g = float(fe.get("structural_design_positive_load_factor_g", 3.0))
    if pos_g <= 1.0:
        raise ValueError("flight_envelope positive_manoeuvre_load_factor_g must be > 1")
    if neg_g >= 0.0:
        raise ValueError("flight_envelope negative_manoeuvre_load_factor_g must be < 0")
    if not (0.0 <= max_bank < 90.0):
        raise ValueError("flight_envelope maximum_bank_deg must be in [0,90)")
    if not (0.0 < cl_util <= 1.0 and 0.0 < gust_cl_util <= 1.0):
        raise ValueError("flight_envelope CL utilization limits must be in (0,1]")
    if lateral_gust < 0.0 or vertical_gust < 0.0 or gust_duration <= 0.0:
        raise ValueError("flight_envelope gust speeds must be >=0 and duration >0")
    if structural_g < pos_g:
        raise ValueError("structural_design_positive_load_factor_g must be >= operational positive load factor")

    d.update({
        "operational_positive_manoeuvre_load_factor_g": pos_g,
        "operational_negative_manoeuvre_load_factor_g": neg_g,
        "operational_maximum_bank_deg": max_bank,
        "maximum_manoeuvre_clmax_utilization": cl_util,
        "maximum_gust_clmax_utilization": gust_cl_util,
        "inspection_lateral_gust_mps": lateral_gust,
        "inspection_vertical_gust_mps": vertical_gust,
        "inspection_gust_duration_s": gust_duration,
        "structural_design_positive_load_factor_g": structural_g,
    })
    cfg["structures"]["load_cases"]["maneuver_load_factor"] = structural_g
    cfg["sixdof_control"]["simulation"]["wind_gust_duration_s"] = gust_duration
    cfg["sixdof_control"]["simulation"]["wind_gust_ned_mps"] = [0.0, lateral_gust, 0.0]
    cfg["sixdof_control"]["simulation"]["inspection_gust_cases_ned_mps"] = {
        "lateral": [0.0, lateral_gust, 0.0],
        "vertical_up": [0.0, 0.0, -vertical_gust],
        "vertical_down": [0.0, 0.0, vertical_gust],
        "combined_lateral_up": [0.0, lateral_gust, -vertical_gust],
    }
    cfg["sixdof_control"]["gates"]["maximum_gust_cl_utilization_fraction"] = gust_cl_util

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
    xs = str(fus.get("cross_section", "rectangular_filleted")).lower().strip()
    if xs in {"rectangular", "rounded_rectangle", "rect_filleted", "filleted_rectangle"}:
        xs = "rectangular_filleted"
    if xs in {"square", "square_fillet", "rounded_square", "filleted_square"}:
        xs = "square_filleted"
    if xs not in {"circular", "square_filleted", "rectangular_filleted"}:
        raise ValueError(
            "fuselage.cross_section must be 'rectangular_filleted', 'square_filleted', or 'circular'"
        )
    d["fuselage_cross_section"] = xs
    max_d = max(float(x["diameter_m"]) for x in sections)
    if str(fus.get("diameter_mode", "hardcoded")).lower() == "auto":
        max_d = max(max_d, float(fus.get("minimum_diameter_m", 0.12)))
    # Optional explicit width/height seeds (else use diameter_m as both).
    fuse_w = float(fus.get("width_m", max_d))
    fuse_h = float(fus.get("height_m", max_d))
    if xs == "square_filleted":
        fuse_w = fuse_h = max(fuse_w, fuse_h)
    elif xs == "circular":
        fuse_w = fuse_h = max_d
    fillet = fus.get("fillet_radius_m")
    if fillet is None:
        fillet = min(0.020, 0.35 * 0.5 * min(fuse_w, fuse_h))
    else:
        fillet = float(fillet)
    d["fuselage_fillet_radius_m"] = float(fillet)
    d["fuselage_length_m"] = total_len
    d["fuselage_width_m"] = fuse_w
    d["fuselage_height_m"] = fuse_h
    d["fuselage_max_diameter_m"] = max(fuse_w, fuse_h)
    cfg["geometry_mass"]["geometry_model"]["geometry"]["central_pod_outer"] = {
        "center_m": [total_len / 2.0, 0.0, 0.0],
        "dimensions_m": [total_len, fuse_w, fuse_h],
    }
    env = cfg["geometry_mass"]["geometry_model"]["packaging"]["envelopes"]["central_pod"]
    # Prefer packaging fuselage_wall_m; else CAD component_keepout_clearance (mm).
    pl_clear = ((u.get("packaging_layout") or {}).get("clearances_m") or {})
    if "fuselage_wall_m" in pl_clear:
        wall_clearance_m = float(pl_clear["fuselage_wall_m"])
    else:
        keepout_mm = float(
            u.get("cad_export", {}).get("tolerances_mm", {}).get("component_keepout_clearance", 3.0)
        )
        wall_clearance_m = keepout_mm / 1000.0
    if wall_clearance_m < 0:
        raise ValueError("packaging_layout.clearances_m.fuselage_wall_m must be >= 0")
    env.update({
        "center_m": [total_len / 2.0, 0.0, 0.0],
        "dimensions_m": [total_len, fuse_w, fuse_h],
        "wall_clearance_m": wall_clearance_m,
        "note": (
            "Filleted rectangular fuselage outer envelope with wall keepout; "
            "nose/tail fair from that base to circular tips in the viewer/CAD."
            if xs != "circular"
            else (
                "Circular fuselage outer envelope with wall keepout; AABB screening alone "
                "does not prove radial fit inside the pod shell."
            )
        ),
    })
    pkg = cfg["geometry_mass"]["geometry_model"]["packaging"]
    pkg["fuselage_cross_section"] = xs
    pkg["fuselage_fillet_radius_m"] = float(fillet)
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
    if main_airfoil.get("polar_rows"):
        main_clmin = float(main_airfoil["polar_summary"]["CL_min"])
    else:
        main_clmin = float(cfg["sixdof_control"]["aerodynamics"].get("CL_min", -0.6))
    if main_clmin >= 0.0:
        raise ValueError("main-wing CL_min must be negative for the negative-g flight-envelope screen")
    d["wing_CLmax"] = main_clmax
    d["wing_CLmin"] = main_clmin
    cfg["sixdof_control"]["aerodynamics"]["stall_screen_CLmax"] = main_clmax
    cfg["sixdof_control"]["aerodynamics"]["stall_screen_CLmin"] = main_clmin
    d["transition_CLmax_increment"] = float(main.get("transition_CLmax_increment", u.get("transition_CLmax_increment", 0.08)))
    cfg["aero_propulsion"]["aircraft"]["stall_model"] = "direct_CLmax"
    cfg["aero_propulsion"]["aircraft"]["transition_CLmax_increment"] = d["transition_CLmax_increment"]
    cfg["transition_mission"]["aerodynamics"]["transition_CLmax_increment"] = d["transition_CLmax_increment"]
    cfg["transition_mission"]["aerodynamics"]["CL_max"] = main_clmax
    d["main_wing_root_chord_m"] = float(main["root_chord_m"])
    d["main_wing_taper_ratio"] = float(main["taper_ratio"])
    d["main_wing_naca_code"] = str(main["naca_code"])
    # Preserve the airfoil thickness ratio when root chord changes during
    # AutoSize Main Wing.  For a standard 4-digit NACA code the final two
    # digits are t/c; an explicit user thickness_ratio overrides that parser.
    if main.get("thickness_ratio") is not None:
        thickness_ratio = float(main["thickness_ratio"])
    else:
        code_digits = "".join(ch for ch in str(main["naca_code"]) if ch.isdigit())
        thickness_ratio = (float(code_digits[-2:]) / 100.0) if len(code_digits) >= 4 else 0.15
    if not (0.05 <= thickness_ratio <= 0.30):
        raise ValueError("main_wing thickness_ratio must be between 0.05 and 0.30")
    d["main_wing_thickness_ratio"] = thickness_ratio
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

    # User-facing trigger.  When it is absent we retain backwards compatibility
    # with legacy span_mode='auto' files.  When present it is authoritative:
    # False = use the hardcoded span/root chord/taper exactly;
    # True  = the coupled pipeline optimizes span + root chord + taper together.
    auto_raw = main.get("AutoSize Main Wing", main.get("auto_size_main_wing"))
    if auto_raw is None:
        auto_size_main_wing = str(main.get("span_mode", "hardcoded")).lower() == "auto"
    elif isinstance(auto_raw, bool):
        auto_size_main_wing = auto_raw
    else:
        raise ValueError("main_wing 'AutoSize Main Wing' must be true or false")
    d["auto_size_main_wing"] = bool(auto_size_main_wing)
    d["main_wing_span_mode"] = "auto" if auto_size_main_wing else "hardcoded"

    span_bounds = [float(x) for x in main.get("span_bounds_m", [3.15, 3.70])]
    chord_bounds = [float(x) for x in main.get("root_chord_bounds_m", [0.26, 0.36])]
    taper_bounds = [float(x) for x in main.get("taper_ratio_bounds", [0.45, 0.75])]
    if len(span_bounds) != 2 or not (0 < span_bounds[0] < span_bounds[1]):
        raise ValueError("main_wing span_bounds_m must be [positive_min, larger_max]")
    if len(chord_bounds) != 2 or not (0 < chord_bounds[0] < chord_bounds[1]):
        raise ValueError("main_wing root_chord_bounds_m must be [positive_min, larger_max]")
    if len(taper_bounds) != 2 or not (0 < taper_bounds[0] < taper_bounds[1] <= 1.0):
        raise ValueError("main_wing taper_ratio_bounds must satisfy 0 < min < max <= 1")
    d["main_wing_span_bounds_m"] = span_bounds
    d["main_wing_root_chord_bounds_m"] = chord_bounds
    d["main_wing_taper_ratio_bounds"] = taper_bounds
    d["main_wing_minimum_tip_chord_m"] = float(main.get("minimum_tip_chord_m", 0.15))
    d["main_wing_minimum_transition_speed_over_stall"] = float(main.get("minimum_transition_speed_over_stall", 1.30))
    d["main_wing_auto_area_margin_factor"] = float(main.get("auto_area_margin_factor", 1.10))
    d["main_wing_autosize_samples"] = int(main.get("autosize_samples", 28))
    d["main_wing_autosize_local_samples"] = int(main.get("autosize_local_samples", 12))
    d["main_wing_autosize_max_iterations"] = int(main.get("autosize_max_iterations", 4))
    if d["main_wing_auto_area_margin_factor"] < 1.0:
        raise ValueError("main_wing auto_area_margin_factor must be >= 1.0")
    if d["main_wing_minimum_transition_speed_over_stall"] <= 1.0:
        raise ValueError("main_wing minimum_transition_speed_over_stall must be > 1.0")

    hardcoded_span = main.get("hardcoded_span_m")
    if hardcoded_span is None:
        if auto_size_main_wing:
            hardcoded_span = 0.5 * (span_bounds[0] + span_bounds[1])
        else:
            raise ValueError("AutoSize Main Wing=false requires hardcoded_span_m")
    bmain = float(hardcoded_span)
    if auto_size_main_wing:
        bmain = min(max(bmain, span_bounds[0]), span_bounds[1])
    elif bmain <= 0:
        raise ValueError("hardcoded_span_m must be > 0")
    Smain = trapezoid_area(bmain, float(main["root_chord_m"]), float(main["taper_ratio"]))
    d["main_wing_span_m"] = bmain
    d["wing_span_m"] = bmain
    d["wing_area_m2"] = Smain
    d["wing_mac_m"] = trapezoid_mac(float(main["root_chord_m"]), float(main["taper_ratio"]))

    d["wing_ac_x_m"] = float(main["aerodynamic_center_x_m"])

    # Horizontal/vertical tail-volume coefficients are calibration references
    # for the screening aero model, not hard sizing minimums.  The dedicated
    # tail optimizer chooses geometry from static/control/gust/structural constraints.
    tail_auto_cfg = u.get("tail_autosizing") or {}
    tail_control_sf = float(tail_auto_cfg.get("control_safety_factor", 1.10))
    if not (1.0 <= tail_control_sf <= 1.30):
        raise ValueError("tail_autosizing control_safety_factor must be between 1.0 and 1.30")
    d["tail_control_safety_factor"] = tail_control_sf
    d["tail_dynamic_validation_candidates"] = int(tail_auto_cfg.get("dynamic_validation_candidates", 12))
    if d["tail_dynamic_validation_candidates"] < 1:
        raise ValueError("tail_autosizing dynamic_validation_candidates must be >= 1")
    ht_auto_raw = htail.get("AutoSize Horizontal Tail", htail.get("auto_size_horizontal_tail"))
    if ht_auto_raw is None:
        auto_size_ht = str(htail.get("span_mode", "auto")).lower() == "auto"
    elif isinstance(ht_auto_raw, bool):
        auto_size_ht = ht_auto_raw
    else:
        raise ValueError("horizontal_tail 'AutoSize Horizontal Tail' must be true or false")
    d["auto_size_horizontal_tail"] = bool(auto_size_ht)
    d["horizontal_tail_span_mode"] = "auto" if auto_size_ht else "hardcoded"
    d["horizontal_tail_root_chord_m"] = float(htail["root_chord_m"])
    d["horizontal_tail_taper_ratio"] = float(htail["taper_ratio"])
    d["horizontal_tail_naca_code"] = str(htail["naca_code"])
    d["horizontal_tail_span_bounds_m"] = [float(x) for x in htail["span_bounds_m"]]
    d["horizontal_tail_root_chord_bounds_m"] = [float(x) for x in htail.get("root_chord_bounds_m", [0.16, 0.32])]
    d["horizontal_tail_taper_ratio_bounds"] = [float(x) for x in htail.get("taper_ratio_bounds", [0.45, 0.85])]
    d["horizontal_tail_aspect_ratio_bounds"] = [float(x) for x in htail.get("aspect_ratio_bounds", [3.5, 6.5])]
    d["horizontal_tail_minimum_tip_chord_m"] = float(htail.get("minimum_tip_chord_m", 0.10))
    d["horizontal_tail_preferred_aspect_ratio"] = float(htail.get("preferred_aspect_ratio", 4.5))
    d["horizontal_tail_preferred_taper_ratio"] = float(htail.get("preferred_taper_ratio", 0.60))
    d["horizontal_tail_profile_cd0_screen"] = float(htail.get("profile_cd0_screen", 0.010))
    d["horizontal_tail_autosize_samples"] = int(htail.get("autosize_samples", 36))
    d["horizontal_tail_autosize_local_samples"] = int(htail.get("autosize_local_samples", 12))
    d["horizontal_tail_ac_x_m"] = float(htail["aerodynamic_center_x_m"])
    tail_arm = max(float(htail["aerodynamic_center_x_m"]) - float(main["aerodynamic_center_x_m"]), 0.05)
    Vh = float(htail.get("target_tail_volume_coefficient", 0.50))
    d["horizontal_tail_target_volume_coefficient"] = Vh
    d["horizontal_tail_reference_volume_coefficient"] = float(htail.get("reference_tail_volume_coefficient", Vh))
    Sh_auto = Vh * Smain * d["wing_mac_m"] / tail_arm
    bh_auto = 2.0 * Sh_auto / (float(htail["root_chord_m"]) * (1.0 + float(htail["taper_ratio"])))
    bh = _surface_span(htail, auto_value=bh_auto) if auto_size_ht else float(htail["hardcoded_span_m"])
    Sh = trapezoid_area(bh, float(htail["root_chord_m"]), float(htail["taper_ratio"]))
    d["horizontal_tail_span_m"] = bh
    d["horizontal_tail_area_m2"] = Sh
    d["horizontal_tail_mac_m"] = trapezoid_mac(float(htail["root_chord_m"]), float(htail["taper_ratio"]))
    d["horizontal_tail_thickness_ratio"] = float(htail.get("thickness_ratio", 0.12))

    # Vertical tail: same pattern for the two-fin vertical-tail-volume floor.
    vt_auto_raw = vtail.get("AutoSize Vertical Tail", vtail.get("auto_size_vertical_tail"))
    if vt_auto_raw is None:
        auto_size_vt = str(vtail.get("span_mode", "auto")).lower() == "auto"
    elif isinstance(vt_auto_raw, bool):
        auto_size_vt = vt_auto_raw
    else:
        raise ValueError("vertical_tail 'AutoSize Vertical Tail' must be true or false")
    d["auto_size_vertical_tail"] = bool(auto_size_vt)
    d["vertical_tail_span_mode"] = "auto" if auto_size_vt else "hardcoded"
    d["vertical_tail_root_chord_m"] = float(vtail["root_chord_m"])
    d["vertical_tail_taper_ratio"] = float(vtail["taper_ratio"])
    d["vertical_tail_naca_code"] = str(vtail["naca_code"])
    d["vertical_tail_span_bounds_m"] = [float(x) for x in vtail["span_bounds_m"]]
    d["vertical_tail_root_chord_bounds_m"] = [float(x) for x in vtail.get("root_chord_bounds_m", [0.12, 0.26])]
    d["vertical_tail_taper_ratio_bounds"] = [float(x) for x in vtail.get("taper_ratio_bounds", [0.40, 0.80])]
    d["vertical_tail_aspect_ratio_bounds"] = [float(x) for x in vtail.get("aspect_ratio_bounds", [2.0, 4.5])]
    d["vertical_tail_minimum_tip_chord_m"] = float(vtail.get("minimum_tip_chord_m", 0.075))
    d["vertical_tail_preferred_aspect_ratio"] = float(vtail.get("preferred_aspect_ratio", 2.5))
    d["vertical_tail_preferred_taper_ratio"] = float(vtail.get("preferred_taper_ratio", 0.60))
    d["vertical_tail_profile_cd0_screen"] = float(vtail.get("profile_cd0_screen", 0.010))
    d["vertical_tail_autosize_samples"] = int(vtail.get("autosize_samples", 36))
    d["vertical_tail_autosize_local_samples"] = int(vtail.get("autosize_local_samples", 12))
    d["vertical_tail_ac_x_m"] = float(vtail["aerodynamic_center_x_m"])
    nfin = int(vtail.get("count", 2))
    d["vertical_tail_count"] = nfin
    lv = max(float(vtail["aerodynamic_center_x_m"]) - float(main["aerodynamic_center_x_m"]), 0.05)
    Vv = float(vtail.get("target_vertical_tail_volume_coefficient", 0.035))
    d["vertical_tail_target_volume_coefficient"] = Vv
    d["vertical_tail_reference_volume_coefficient"] = float(vtail.get("reference_vertical_tail_volume_coefficient", Vv))
    Sv_total_auto = Vv * Smain * bmain / lv
    h_auto = 2.0 * (Sv_total_auto / nfin) / (float(vtail["root_chord_m"]) * (1.0 + float(vtail["taper_ratio"])))
    hv = _surface_span(vtail, auto_value=h_auto) if auto_size_vt else float(vtail["hardcoded_span_m"])
    Sv_each = trapezoid_area(hv, float(vtail["root_chord_m"]), float(vtail["taper_ratio"]))
    d["vertical_tail_height_m"] = hv
    d["vertical_tail_area_each_m2"] = Sv_each
    d["vertical_tail_area_total_m2"] = Sv_each * nfin
    d["vertical_tail_thickness_ratio"] = float(vtail.get("thickness_ratio", 0.12))

    for label, bounds in (
        ("horizontal_tail span", d["horizontal_tail_span_bounds_m"]),
        ("horizontal_tail root chord", d["horizontal_tail_root_chord_bounds_m"]),
        ("horizontal_tail taper", d["horizontal_tail_taper_ratio_bounds"]),
        ("horizontal_tail aspect ratio", d["horizontal_tail_aspect_ratio_bounds"]),
        ("vertical_tail height", d["vertical_tail_span_bounds_m"]),
        ("vertical_tail root chord", d["vertical_tail_root_chord_bounds_m"]),
        ("vertical_tail taper", d["vertical_tail_taper_ratio_bounds"]),
        ("vertical_tail aspect ratio", d["vertical_tail_aspect_ratio_bounds"]),
    ):
        if len(bounds) != 2 or not (float(bounds[0]) < float(bounds[1])):
            raise ValueError(f"{label} bounds must be [min, max] with min < max")

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
    # Optional cruise lift-prop CdA split (blade + motor fairings) → lift_system.
    ls = cfg["aero_propulsion"]["lift_system"]
    if prop.get("blade_CdA_total_m2") is not None:
        ls["blade_CdA_total_m2"] = float(prop["blade_CdA_total_m2"])
    if prop.get("motor_mount_CdA_total_m2") is not None:
        ls["motor_mount_CdA_total_m2"] = float(prop["motor_mount_CdA_total_m2"])
    if prop.get("motor_mount_CdA_each_m2") is not None:
        ls["motor_mount_CdA_each_m2"] = float(prop["motor_mount_CdA_each_m2"])
    # Optional ledger mass overrides (dedupe joiners / harness vs misc).
    for name, mass in (u.get("component_mass_overrides") or {}).items():
        if name in {"note", "notes"} or mass is None or isinstance(mass, (dict, list)):
            continue
        for c in cfg["geometry_mass"]["component_ledger"]["components"]:
            if c.get("name") == name:
                c["mass_takeoff_kg"] = float(mass)
                c["source"] = f"user_inputs.component_mass_overrides.{name}"
                break
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

        cb_bat = str(pl.get("cb_battery_order", "battery_forward")).lower().strip()
        if cb_bat not in {"battery_forward", "control_box_forward"}:
            raise ValueError(
                "packaging_layout.cb_battery_order must be 'battery_forward' or 'control_box_forward'"
            )
        pl["cb_battery_order"] = cb_bat
        d["cb_battery_order"] = cb_bat

        clear = pl.setdefault("clearances_m", {})
        wall_clear = float(clear.get("fuselage_wall_m", env.get("wall_clearance_m", 0.01)))
        if wall_clear < 0:
            raise ValueError("packaging_layout.clearances_m.fuselage_wall_m must be >= 0")
        clear["fuselage_wall_m"] = wall_clear
        d["fuselage_wall_clearance_m"] = wall_clear
        env["wall_clearance_m"] = wall_clear
        pkg["component_clearance_m"] = wall_clear
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
        h = hw["hydrogen"]
        for k, v in h.items():
            cfg["power_system"]["hydrogen"][k] = v
        # Apply supplier envelope + mass into the packaging component ledger.
        L = h.get("cylinder_length_m")
        Dia = h.get("cylinder_diameter_m")
        m_cyl = h.get("cylinder_mass_kg")
        for c in cfg["geometry_mass"]["component_ledger"]["components"]:
            if c.get("name") != "hydrogen_cylinder":
                continue
            dims = c.setdefault("dimensions_m", {})
            if L is not None:
                dims["length"] = float(L)
            if Dia is not None:
                dims["diameter"] = float(Dia)
            if m_cyl is not None:
                c["mass_takeoff_kg"] = float(m_cyl)
            c["shape"] = "cylinder_x"
            c["status"] = "supplier_confirmed"
            c["source"] = str(h.get("storage_name") or "hardware.hydrogen")
            c["note"] = (
                f"Envelope from user_inputs hardware.hydrogen "
                f"({1000 * float(dims.get('length', 0)):.0f} x {1000 * float(dims.get('diameter', 0)):.0f} mm); "
                f"mass {float(m_cyl) if m_cyl is not None else c.get('mass_takeoff_kg')} kg."
            )
            break

    # Direct-bus architecture: never model pack/bus voltage above the FC ceiling.
    from .hybrid_power import enforce_bus_voltage_ceiling_on_mapping
    enforce_bus_voltage_ceiling_on_mapping(cfg["power_system"])

    # -------- Discrete motor/prop/battery database selection --------
    if u.get("hardware_selection"):
        # Keep hardware screening criteria aligned with the clamped FC bus ceiling.
        u_hw = deepcopy(u)
        hs = u_hw.setdefault("hardware_selection", {})
        crit = hs.setdefault("auto_criteria", {})
        fc_max = float(cfg["power_system"]["fuel_cell"]["output_voltage_max_V"])
        series = int(crit.get("series_cells", cfg["power_system"]["battery"]["series_cells"]))
        crit["bus_voltage_ceiling_V"] = min(float(crit.get("bus_voltage_ceiling_V", fc_max)), fc_max)
        crit["maximum_cell_voltage_V"] = min(
            float(crit.get("maximum_cell_voltage_V", fc_max / max(series, 1))),
            fc_max / max(series, 1),
        )
        selection = select_hardware(u_hw, root=root)
        apply_hardware_selection(cfg, selection)

    # -------- IE-SOAR fuel-cell cooling integration --------
    # The integration assumption is that the built-in fans suck ambient air
    # through the fuel-cell stack and discharge through the fan-side fuselage
    # opening.  The detailed cooling calculation runs after packaging/coupling,
    # but the opening drag allowance is available here so optimization sees it.
    cooling = deepcopy(u.get("fuel_cell_cooling", {}))
    if cooling:
        if str(cooling.get("fan_flow_direction", "suction_through_stack_to_fan_side")) != "suction_through_stack_to_fan_side":
            raise ValueError("fuel_cell_cooling.fan_flow_direction must be 'suction_through_stack_to_fan_side' for the current H2AIR architecture")
        fan_side = str(cooling.get("fan_side", "right")).lower()
        inlet_side = str(cooling.get("inlet_side", "left")).lower()
        exhaust_side = str(cooling.get("exhaust_side", fan_side)).lower()
        if fan_side not in {"left", "right"} or inlet_side not in {"left", "right"} or exhaust_side not in {"left", "right"}:
            raise ValueError("fuel_cell_cooling fan/inlet/exhaust sides must be 'left' or 'right'")
        if inlet_side == exhaust_side:
            raise ValueError("fuel_cell_cooling inlet_side and exhaust_side must be opposite for the cross-fuselage architecture")
        cooling["fan_side"] = fan_side
        cooling["inlet_side"] = inlet_side
        cooling["exhaust_side"] = exhaust_side
        cfg["fuel_cell_cooling"] = cooling
        cooling_cda = cooling_opening_cda_from_user_inputs(cooling)
        cfg["aero_propulsion"]["aircraft"]["cooling_openings_CdA_m2"] = cooling_cda
        d["fuel_cell_cooling_openings_CdA_m2"] = cooling_cda

    # -------- Manufacturing / measured linear-mass model --------
    cfg["optimizer_settings"] = deepcopy(u.get("optimizer", {}))
    cfg["manufacturing_mass_model"] = _normalize_manufacturing_mass_model(u["manufacturing_mass_model"])
    cfg["manufacturing_mass_model"]["fuselage_total_length_m"] = total_len
    cfg["manufacturing_mass_model"]["fuselage_sections"] = deepcopy(sections)
    if u.get("carbon_fiber_tubes"):
        apply_carbon_fiber_tubes(cfg, u["carbon_fiber_tubes"])

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
        "version": "H2AIR-integrated-v4.1",
        "description": "Single-entry H2AIR model with separated user inputs, automatic lifting-surface sizing, measured linear-mass manufacturing model, polar-based airfoils, CAD export, discrete hardware selection and IE-SOAR cross-fuselage cooling integration.",
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
        main["AutoSize Main Wing"] = False
        main["span_mode"] = "hardcoded"
        main["hardcoded_span_m"] = _round(vals["main_wing_span_m"])
    if "main_wing_root_chord_m" in vals:
        main["root_chord_m"] = _round(vals["main_wing_root_chord_m"])
    if "main_wing_taper_ratio" in vals:
        main["taper_ratio"] = _round(vals["main_wing_taper_ratio"])
    if "horizontal_tail_span_m" in vals:
        htail["AutoSize Horizontal Tail"] = False
        htail["span_mode"] = "hardcoded"
        htail["hardcoded_span_m"] = _round(vals["horizontal_tail_span_m"])
    if "horizontal_tail_root_chord_m" in vals:
        htail["root_chord_m"] = _round(vals["horizontal_tail_root_chord_m"])
    if "horizontal_tail_taper_ratio" in vals:
        htail["taper_ratio"] = _round(vals["horizontal_tail_taper_ratio"])
    if "vertical_tail_height_m" in vals:
        vtail["AutoSize Vertical Tail"] = False
        vtail["span_mode"] = "hardcoded"
        vtail["hardcoded_span_m"] = _round(vals["vertical_tail_height_m"])
    if "vertical_tail_root_chord_m" in vals:
        vtail["root_chord_m"] = _round(vals["vertical_tail_root_chord_m"])
    if "vertical_tail_taper_ratio" in vals:
        vtail["taper_ratio"] = _round(vals["vertical_tail_taper_ratio"])

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
            "mission", "fuselage", "lifting_surfaces", "propulsion", "component_mass_overrides",
            "component_positions", "hardware", "hardware_selection", "manufacturing_mass_model", "carbon_fiber_tubes", "cad_export", "optimizer",
            "packaging_layout", "fuel_cell_cooling",
        ],
        "auto_calculated_examples": [
            "fuselage total length from section lengths",
            "main wing span in AUTO mode and wing area/MAC",
            "airfoil CL/CD/CM interpolation from a supplied polar when available",
            "AUTO motor/prop/battery database selection when enabled",
            "clean CAD parameter export (geometry/clearance/hardware/cooling only)",
            "fuel-cell waste heat, required airflow, duct pressure loss and cooling-airflow margin",
            "horizontal-tail span/root-chord/taper/area from AutoSize Horizontal Tail",
            "vertical-tail height/root-chord/taper/area from AutoSize Vertical Tail",
            "component masses in LINEAR mode from kg/m x length",
            "CG and inertia tensor",
            "neutral point and static margin",
            "aerodynamic drag/cruise power",
            "VTOL thrust/power requirements",
            "mission energy, hydrogen use and battery SOC",
            "structural sizing feedback and converged MTOW",
            "landing-leg CFRP tubes sized for 3g landing with 2 mm printed sleeves",
            "COTS carbon-tube OD/ID when carbon_fiber_tubes.section_mode=hardcoded",
        ],
        "hardcode_rule": "Any span/mass entry with mode='hardcoded' is treated as fixed and is excluded from automatic sizing for that quantity. carbon_fiber_tubes.section_mode='hardcoded' locks recommended structural tube OD/ID to even-mm COTS sizes. Optional carbon_fiber_tubes.landing_legs locks landing-leg tubes; otherwise they are auto-sized from the 3g landing screen and rounded to even-mm COTS.",
    }
