from __future__ import annotations

"""Landing-leg geometry for the H2AIR screening model.

Layout: one nose leg under the fuselage + two wing legs under the primary-spar /
boom junction. Each leg is a CFRP tube inside a constant-thickness 3D-printed sleeve.
"""

from copy import deepcopy
from math import pi
from typing import Any, Dict, Mapping, MutableMapping, Optional

INCH_TO_M = 0.0254
EPS = 1e-12


def landing_legs_enabled(cfg: Mapping[str, Any]) -> bool:
    mm = cfg.get("manufacturing_mass_model") or {}
    legs = mm.get("landing_legs") or {}
    return bool(legs.get("enabled", False))


def sleeve_od_m(tube_od_m: float, sleeve_wall_m: float) -> float:
    return float(tube_od_m) + 2.0 * float(sleeve_wall_m)


def sleeve_annulus_area_m2(tube_od_m: float, sleeve_wall_m: float) -> float:
    od = sleeve_od_m(tube_od_m, sleeve_wall_m)
    return 0.25 * pi * (od**2 - float(tube_od_m) ** 2)


def sleeve_mass_kg(
    *,
    tube_od_m: float,
    sleeve_wall_m: float,
    length_m: float,
    density_kg_m3: float,
    count: int = 1,
) -> float:
    return sleeve_annulus_area_m2(tube_od_m, sleeve_wall_m) * float(length_m) * float(density_kg_m3) * int(count)


def landing_legs_cda_m2(cfg: Mapping[str, Any]) -> float:
    """Screening cruise CdA of exposed landing-leg sleeves.

    Prefer an explicit ``drag_CdA_m2`` override (streamlined fairings). Otherwise
    each vertical leg is treated as a circular cylinder with frontal area
    ``sleeve_OD * exposed_length`` and a user Cd screen.
    """
    legs_cfg = (cfg.get("manufacturing_mass_model") or {}).get("landing_legs") or {}
    if legs_cfg.get("drag_CdA_m2") is not None:
        return max(0.0, float(legs_cfg["drag_CdA_m2"]))
    layout = resolve_landing_leg_layout(cfg)
    if layout is None:
        return 0.0
    cd = float(legs_cfg.get("drag_coefficient_screen", 1.0))
    if cd < 0.0:
        raise ValueError("landing_legs.drag_coefficient_screen must be >= 0")
    sleeve_od = float(layout.get("sleeve_od_m") or sleeve_od_m(float(layout["tube_od_m"]), float(layout["sleeve_wall_m"])))
    cda = 0.0
    for leg in layout["legs"]:
        cda += cd * sleeve_od * float(leg["length_m"])
    return max(0.0, cda)


def sync_landing_legs_drag(cfg: MutableMapping[str, Any]) -> float:
    """Write current landing-leg CdA into aero/design so cruise/mission see it."""
    cda = landing_legs_cda_m2(cfg)
    cfg.setdefault("aero_propulsion", {}).setdefault("aircraft", {})["landing_legs_CdA_m2"] = cda
    cfg.setdefault("design", {})["landing_legs_CdA_m2"] = cda
    legs_cfg = cfg.setdefault("manufacturing_mass_model", {}).setdefault("landing_legs", {})
    if isinstance(legs_cfg, dict):
        resolved = legs_cfg.get("resolved")
        if isinstance(resolved, dict):
            resolved["drag_CdA_m2"] = cda
            resolved["drag_coefficient_screen"] = float(legs_cfg.get("drag_coefficient_screen", 1.0))
    geom = cfg.get("geometry_mass", {}).get("geometry_model", {}).get("geometry")
    if isinstance(geom, dict) and isinstance(geom.get("landing_legs"), dict):
        geom["landing_legs"]["drag_CdA_m2"] = cda
    return cda


def resolve_landing_leg_layout(cfg: Mapping[str, Any]) -> Optional[Dict[str, Any]]:
    """Compute nose + wing landing-leg stations from the closed geometry.

    Returns ``None`` when landing legs are disabled. Lengths are positive when the
    mount sits above the ground plane (z positive up).
    """
    if not landing_legs_enabled(cfg):
        return None

    d = cfg["design"]
    gm = cfg["geometry_mass"]["geometry_model"]["geometry"]
    mm = cfg["manufacturing_mass_model"]
    legs = mm["landing_legs"]
    mw = mm.get("main_wing") or {}

    fuse_h = float(d.get("fuselage_height_m", d.get("fuselage_max_diameter_m", 0.18)))
    fuse_bottom = -0.5 * fuse_h
    wing_lower = float(gm.get("wing_lower_z_m", float(mw.get("z_m", 0.1)) - 0.5 * float(mw.get("thickness_m", 0.028))))

    # Nose station: mid-nose fuselage section.
    sections = mm.get("fuselage_sections") or []
    nose = next((s for s in sections if str(s.get("name", "")).lower() == "nose"), None)
    if nose is not None:
        nose_L = float(nose["length_m"])
        nose_x = 0.5 * nose_L
    else:
        nose_L = float(legs.get("nose_length_fallback_m", 0.14))
        nose_x = 0.5 * nose_L

    # Wing stations: primary-spar chord fraction at boom |y| (load into wing–boom junction).
    mac = float(d["wing_mac_m"])
    xac = float(d["wing_ac_x_m"])
    le = xac - 0.25 * mac
    spar_frac = float((mw.get("primary_spar") or {}).get("chord_fraction", 0.25))
    wing_x = le + spar_frac * mac
    boom_ys = gm.get("boom_center_y_m") or [-0.35, 0.35]
    y_abs = abs(float(boom_ys[1] if len(boom_ys) > 1 else boom_ys[0]))

    # Ground plane clears the lowest lift-prop tip by the configured margin.
    prop_dia_m = float(d.get("lift_propeller_diameter_in", 22.0)) * INCH_TO_M
    centers = gm.get("lift_prop_centers_m") or []
    if centers:
        motor_z = min(float(p[2]) for p in centers if isinstance(p, (list, tuple)) and len(p) >= 3)
    else:
        motor_z = float(cfg.get("aero_propulsion", {}).get("lift_layout", {}).get("z_m", 0.12))
    prop_tip_z = motor_z - 0.5 * prop_dia_m
    prop_clear = float(legs.get("prop_ground_clearance_m", 0.020))
    if legs.get("ground_z_m") is not None:
        ground_z = float(legs["ground_z_m"])
    else:
        ground_z = prop_tip_z - prop_clear

    # Optional extra drop so short nose legs remain printable / attachable.
    min_len = float(legs.get("minimum_leg_length_m", 0.060))
    nose_mount_z = float(legs.get("nose_mount_z_m", fuse_bottom))
    wing_mount_z = float(legs.get("wing_mount_z_m", wing_lower))
    nose_len = max(nose_mount_z - ground_z, min_len)
    wing_len = max(wing_mount_z - ground_z, min_len)
    # Keep a common ground plane: if min length forced a longer nose, drop ground.
    ground_z = min(nose_mount_z - nose_len, wing_mount_z - wing_len)
    nose_len = nose_mount_z - ground_z
    wing_len = wing_mount_z - ground_z

    sleeve_wall = float(legs.get("sleeve_wall_m", 0.002))
    tube_rule = legs.get("tube") or {}
    tube_od = float(
        tube_rule.get("design_tube_od_m")
        or tube_rule.get("reference_tube_od_m")
        or tube_rule.get("tube_od_m")
        or 0.012
    )

    nose_cg_z = 0.5 * (nose_mount_z + ground_z)
    wing_cg_z = 0.5 * (wing_mount_z + ground_z)

    return {
        "enabled": True,
        "count": 3,
        "layout": "nose_plus_wing",
        "landing_load_factor": float(legs.get("landing_load_factor", 3.0)),
        "side_load_fraction": float(legs.get("side_load_fraction", 0.25)),
        "sleeve_wall_m": sleeve_wall,
        "printed_density_kg_m3": float(legs.get("printed_density_kg_m3", 1200.0)),
        "ground_z_m": ground_z,
        "prop_tip_z_m": prop_tip_z,
        "prop_ground_clearance_m": prop_clear,
        "sizing_length_m": max(nose_len, wing_len),
        "legs": [
            {
                "name": "nose",
                "x_m": nose_x,
                "y_m": 0.0,
                "mount_z_m": nose_mount_z,
                "foot_z_m": ground_z,
                "length_m": nose_len,
                "cg_z_m": nose_cg_z,
            },
            {
                "name": "wing_left",
                "x_m": wing_x,
                "y_m": -y_abs,
                "mount_z_m": wing_mount_z,
                "foot_z_m": ground_z,
                "length_m": wing_len,
                "cg_z_m": wing_cg_z,
            },
            {
                "name": "wing_right",
                "x_m": wing_x,
                "y_m": y_abs,
                "mount_z_m": wing_mount_z,
                "foot_z_m": ground_z,
                "length_m": wing_len,
                "cg_z_m": wing_cg_z,
            },
        ],
        "tube_od_m": tube_od,
        "sleeve_od_m": sleeve_od_m(tube_od, sleeve_wall),
    }


def apply_landing_leg_layout(cfg: MutableMapping[str, Any]) -> Optional[Dict[str, Any]]:
    """Resolve layout and write stations into manufacturing / structures / geometry."""
    layout = resolve_landing_leg_layout(cfg)
    mm = cfg.setdefault("manufacturing_mass_model", {})
    legs_cfg = mm.setdefault("landing_legs", {})
    geom = cfg["geometry_mass"]["geometry_model"]["geometry"]
    st = cfg.setdefault("structures", {}).setdefault("structure", {})
    lc = cfg.setdefault("structures", {}).setdefault("load_cases", {})

    if layout is None:
        legs_cfg["resolved"] = {"enabled": False}
        geom.pop("landing_legs", None)
        sync_landing_legs_drag(cfg)
        return None

    legs_cfg["resolved"] = deepcopy(layout)
    geom["landing_legs"] = deepcopy(layout)
    geom["landing_ground_z_m"] = layout["ground_z_m"]

    st["landing_leg_enabled"] = True
    st["landing_leg_length_m"] = float(layout["sizing_length_m"])
    st["landing_leg_count"] = int(layout["count"])
    st["landing_leg_side_load_fraction"] = float(layout["side_load_fraction"])
    st["landing_leg_deflection_limit_fraction"] = float(
        legs_cfg.get("deflection_limit_fraction", st.get("landing_leg_deflection_limit_fraction", 0.05))
    )
    st["landing_leg_slope_limit_deg"] = float(
        legs_cfg.get("slope_limit_deg", st.get("landing_leg_slope_limit_deg", 3.0))
    )
    st["landing_leg_id_over_od"] = float(
        (legs_cfg.get("tube") or {}).get("id_over_od", st.get("landing_leg_id_over_od", 0.75))
    )
    lc["landing_load_factor"] = float(layout["landing_load_factor"])
    sync_landing_legs_drag(cfg)
    return layout
