from __future__ import annotations

"""Landing-leg geometry for the H2AIR screening model.

Layout: one nose leg under the fuselage + two wing legs under the primary-spar /
boom junction. Each leg is a CFRP tube inside a constant-thickness 3D-printed sleeve.
"""

from copy import deepcopy
from math import atan, pi, sin
from typing import Any, Dict, Mapping, MutableMapping, Optional

INCH_TO_M = 0.0254
EPS = 1e-12


def landing_legs_enabled(cfg: Mapping[str, Any]) -> bool:
    mm = cfg.get("manufacturing_mass_model") or {}
    legs = mm.get("landing_legs") or {}
    return bool(legs.get("enabled", False))


def horizontal_prop_blade_half_height_m(
    diameter_m: float,
    pitch_m: float,
    *,
    chord_fraction_of_diameter: float = 0.055,
    minimum_half_height_m: float = 0.003,
) -> float:
    """Vertical half-thickness of a *horizontal* lift-prop disk.

    The disk lies in XY (spin about Z). Diameter sets the planform radius only.
    Geometric pitch tilts the blade chord, so the airfoil projects a small height
    ±h about the disk plane: ``h ≈ 0.5 * chord * sin(φ)`` with helix angle ``φ``
    evaluated at 75% radius.
    """
    D = max(float(diameter_m), EPS)
    P = max(0.0, float(pitch_m))
    chord = max(EPS, float(chord_fraction_of_diameter) * D)
    r75 = 0.75 * 0.5 * D
    phi = atan(P / max(2.0 * pi * r75, EPS)) if P > 0.0 else 0.0
    half = 0.5 * chord * sin(phi)
    return max(float(minimum_half_height_m), half)


def lift_prop_vertical_extents_m(cfg: Mapping[str, Any]) -> Dict[str, float]:
    """Disk-plane z and blade/motor low/high z for horizontal lift props."""
    d = cfg["design"]
    gm = ((cfg.get("geometry_mass") or {}).get("geometry_model") or {}).get("geometry") or {}
    legs = ((cfg.get("manufacturing_mass_model") or {}).get("landing_legs") or {})
    hw = (cfg.get("hardware_selection_result") or {}).get("selected_lift_combo") or {}

    dia_m = float(d.get("lift_propeller_diameter_in", 22.0)) * INCH_TO_M
    pitch_in = float(hw.get("prop_pitch_in") or d.get("lift_propeller_pitch_in") or 0.0)
    pitch_m = pitch_in * INCH_TO_M
    chord_frac = float(legs.get("blade_chord_fraction_of_diameter", 0.055))
    blade_half = horizontal_prop_blade_half_height_m(
        dia_m,
        pitch_m,
        chord_fraction_of_diameter=chord_frac,
        minimum_half_height_m=float(legs.get("blade_minimum_half_height_m", 0.003)),
    )

    centers = gm.get("lift_prop_centers_m") or []
    if centers:
        motor_z = min(float(p[2]) for p in centers if isinstance(p, (list, tuple)) and len(p) >= 3)
    else:
        motor_z = float(cfg.get("aero_propulsion", {}).get("lift_layout", {}).get("z_m", 0.12))

    motor_len = float(hw.get("motor_length_m") or 0.0)
    if motor_len <= 0.0:
        motor_len = float(legs.get("lift_motor_body_length_fallback_m", 0.027))
    # Motor body is modelled along Z, centred on the disk plane.
    motor_half = 0.5 * motor_len
    half_down = max(blade_half, motor_half)
    half_up = max(blade_half, motor_half)
    return {
        "disk_z_m": motor_z,
        "diameter_m": dia_m,
        "pitch_m": pitch_m,
        "blade_half_height_m": blade_half,
        "motor_half_height_m": motor_half,
        "low_z_m": motor_z - half_down,
        "high_z_m": motor_z + half_up,
    }


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
    if not landing_legs_enabled(cfg):
        return 0.0
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

    # Ground clears the lowest of: fuselage belly, horizontal-prop blade band
    # (pitch thickness — NOT diameter), and lift-motor body. Diameter only sets
    # the planform disk; it must not be treated as a vertical tip drop.
    extents = lift_prop_vertical_extents_m(cfg)
    prop_low_z = float(extents["low_z_m"])
    prop_tip_z = prop_low_z  # retained key name: lowest prop/motor z
    prop_clear = float(legs.get("prop_ground_clearance_m", 0.020))
    lowest_airframe_z = min(fuse_bottom, prop_low_z)
    if legs.get("ground_z_m") is not None:
        ground_z = float(legs["ground_z_m"])
    else:
        ground_z = lowest_airframe_z - prop_clear

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
        "prop_disk_z_m": float(extents["disk_z_m"]),
        "prop_blade_half_height_m": float(extents["blade_half_height_m"]),
        "prop_high_z_m": float(extents["high_z_m"]),
        "prop_ground_clearance_m": prop_clear,
        "lowest_airframe_z_m": lowest_airframe_z,
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
        geom.pop("landing_ground_z_m", None)
        st["landing_leg_enabled"] = False
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
