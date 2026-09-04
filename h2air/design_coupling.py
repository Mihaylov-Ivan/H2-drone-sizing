from __future__ import annotations

"""Cross-subsystem design-variable resolver for the integrated H2AIR model.

The integrated model has many legacy subsystem sections because the physics were
built incrementally.  This module makes aircraft-level design variables the
source of truth and propagates them into those subsystem records before a run.

The neutral-point update is a *calibrated tail-volume screening relation*.
It is anchored exactly to the accessible baseline geometry and NP, so it does
not claim CFD/AVL fidelity.  It exists to let the optimizer move wing/tail
geometry without incorrectly holding the neutral point fixed.
"""

from copy import deepcopy
from math import sqrt
from typing import Any, Dict, Mapping, MutableMapping, Sequence

INCH_TO_M = 0.0254

# Accessible integrated-v1 reference state.  These are calibration anchors,
# not new hardware claims.
REF = {
    "wing_area_m2": 0.634,
    "wing_span_m": 2.92,
    "wing_mac_m": 0.223,
    "wing_ac_x_m": 0.377,
    "neutral_point_x_m": 0.420,
    "horizontal_tail_area_m2": 0.1276,
    "horizontal_tail_ac_x_m": 1.05,
    "wing_structure_mass_kg": 2.6153,
    "wing_structure_x_m": 0.43,
    "horizontal_tail_structure_mass_kg": 0.1305,
    "horizontal_tail_semispan_m": 0.4309,
    "lift_row_center_x_m": 0.50,
    "battery_position_m": (0.39, 0.0, 0.0),
    "fuel_cell_position_m": (0.57, 0.0, 0.0),
    "hydrogen_cylinder_position_m": (0.34, 0.0, 0.0),
}

# Calibrate the compact tail-volume NP relation to reproduce x_NP=0.420 m.
_TAIL_FACTOR = (
    (REF["neutral_point_x_m"] - REF["wing_ac_x_m"])
    / ((REF["horizontal_tail_area_m2"] / REF["wing_area_m2"])
       * (REF["horizontal_tail_ac_x_m"] - REF["wing_ac_x_m"]))
)
_MAC_SHAPE_FACTOR = REF["wing_mac_m"] / (REF["wing_area_m2"] / REF["wing_span_m"])
_WING_STRUCTURE_X_OFFSET = REF["wing_structure_x_m"] - REF["wing_ac_x_m"]


def _components(cfg: MutableMapping[str, Any]):
    return cfg["geometry_mass"]["component_ledger"]["components"]


def _by_name(cfg: MutableMapping[str, Any]) -> Dict[str, MutableMapping[str, Any]]:
    return {x["name"]: x for x in _components(cfg)}


def _set_position(item: MutableMapping[str, Any], x: float | None = None,
                  y: float | None = None, z: float | None = None) -> None:
    p = list(item.get("position_m", [0.0, 0.0, 0.0]))
    if x is not None: p[0] = float(x)
    if y is not None: p[1] = float(y)
    if z is not None: p[2] = float(z)
    item["position_m"] = p


def calibrated_neutral_point(
    *, wing_ac_x_m: float, wing_area_m2: float,
    horizontal_tail_area_m2: float, horizontal_tail_ac_x_m: float,
) -> float:
    """Return screening neutral point from a baseline-calibrated tail-volume law."""
    tail_arm = max(horizontal_tail_ac_x_m - wing_ac_x_m, 1e-6)
    return float(
        wing_ac_x_m
        + _TAIL_FACTOR * (horizontal_tail_area_m2 / wing_area_m2) * tail_arm
    )


def resolve_design_couplings(raw_cfg: Mapping[str, Any]) -> Dict[str, Any]:
    """Return a deep-copied config with aircraft-level design variables propagated.

    Existing baseline configs remain backwards compatible: missing new design
    variables are populated from the integrated-v1 reference state.
    """
    cfg: Dict[str, Any] = deepcopy(dict(raw_cfg))
    d = cfg["design"]

    # New top-level design variables with baseline-compatible defaults.
    d.setdefault("wing_ac_x_m", REF["wing_ac_x_m"])
    d.setdefault("horizontal_tail_area_m2", REF["horizontal_tail_area_m2"])
    d.setdefault("horizontal_tail_ac_x_m", REF["horizontal_tail_ac_x_m"])
    d.setdefault("lift_row_center_x_m", REF["lift_row_center_x_m"])
    d.setdefault("battery_x_m", REF["battery_position_m"][0])
    d.setdefault("battery_z_m", REF["battery_position_m"][2])
    d.setdefault("fuel_cell_x_m", REF["fuel_cell_position_m"][0])
    d.setdefault("fuel_cell_z_m", REF["fuel_cell_position_m"][2])
    d.setdefault("hydrogen_cylinder_x_m", REF["hydrogen_cylinder_position_m"][0])
    d.setdefault("hydrogen_cylinder_z_m", REF["hydrogen_cylinder_position_m"][2])
    d.setdefault("neutral_point_model", "calibrated_tail_volume")

    # v3 root/taper/span geometry derives planform values from the user-defined
    # chords. Legacy v2 configs retain the calibrated MAC relation.
    if str(d.get("surface_geometry_mode", "legacy_area_span")) == "root_taper_span":
        b = float(d["main_wing_span_m"])
        cr = float(d["main_wing_root_chord_m"]); lam = float(d["main_wing_taper_ratio"])
        S = b * cr * (1.0 + lam) / 2.0
        mac = (2.0/3.0) * cr * (1.0 + lam + lam*lam) / (1.0 + lam)
        d["wing_span_m"] = b; d["wing_area_m2"] = S; d["wing_mac_m"] = mac
        bh = float(d["horizontal_tail_span_m"]); chr_ = float(d["horizontal_tail_root_chord_m"]); lamh = float(d["horizontal_tail_taper_ratio"])
        d["horizontal_tail_area_m2"] = bh * chr_ * (1.0 + lamh) / 2.0
        d["horizontal_tail_mac_m"] = (2.0/3.0) * chr_ * (1.0 + lamh + lamh*lamh) / (1.0 + lamh)
        hv = float(d["vertical_tail_height_m"]); cvr=float(d["vertical_tail_root_chord_m"]); lamv=float(d["vertical_tail_taper_ratio"])
        d["vertical_tail_area_each_m2"] = hv*cvr*(1.0+lamv)/2.0
        d["vertical_tail_area_total_m2"] = 2.0*d["vertical_tail_area_each_m2"]
    else:
        S = float(d["wing_area_m2"]); b = float(d["wing_span_m"])
        if S <= 0 or b <= 0:
            raise ValueError("wing area/span must be positive")
        mac = _MAC_SHAPE_FACTOR * S / b
        d["wing_mac_m"] = mac
    wing_ac = float(d["wing_ac_x_m"])
    Sh = float(d["horizontal_tail_area_m2"])
    xh = float(d["horizontal_tail_ac_x_m"])
    if str(d.get("neutral_point_model")) == "calibrated_tail_volume":
        d["neutral_point_station_x_m"] = calibrated_neutral_point(
            wing_ac_x_m=wing_ac, wing_area_m2=S,
            horizontal_tail_area_m2=Sh, horizontal_tail_ac_x_m=xh,
        )

    # Propagate wing/tail geometry to the mass/geometry model.
    gm = cfg["geometry_mass"]["geometry_model"]
    geom = gm["geometry"]
    geom.update({
        "wing_area_m2": S,
        "wing_span_m": b,
        "wing_MAC_m": mac,
        "wing_mean_geometric_chord_m": S / b,
        "wing_ac_x_m": wing_ac,
        "reference_neutral_point_x_m": float(d["neutral_point_station_x_m"]),
    })
    if str(d.get("surface_geometry_mode", "")) == "root_taper_span":
        geom["horizontal_tail_span_m"] = float(d["horizontal_tail_span_m"])
        geom["vertical_fin_height_m"] = float(d["vertical_tail_height_m"])
        # Preserve the user's fixed fuselage/pod geometry. No component-fit driven pod sizing is performed.
        if "fuselage_length_m" in d:
            geom["central_pod_outer"]["center_m"] = [float(d["fuselage_length_m"])/2.0, 0.0, 0.0]
            geom["central_pod_outer"]["dimensions_m"] = [float(d["fuselage_length_m"]), float(d["fuselage_max_diameter_m"]), float(d["fuselage_max_diameter_m"])]

    # Propagate to aero/transition/six-DOF records; pipeline will still refresh mass.
    ap = cfg["aero_propulsion"]["aircraft"]
    ap.update({"wing_area_m2": S, "wing_span_m": b, "wing_MAC_m": mac})
    tm = cfg["transition_mission"]["reference_case"]
    tm.update({"wing_area_m2": S, "wing_span_m": b})
    s6a = cfg["sixdof_control"]["aircraft"]
    s6a.update({
        "wing_area_m2": S, "wing_span_m": b, "wing_mac_m": mac,
        "neutral_point_station_x_m": float(d["neutral_point_station_x_m"]),
        "horizontal_tail_area_m2": Sh,
        "horizontal_tail_ac_station_x_m": xh,
    })
    cfg["structures"]["aircraft"].update({
        "reference_wing_span_m": b,
        "design_wing_span_m": b,
        "horizontal_tail_area_m2": Sh,
    })
    cfg["structures"]["structure"]["design_wing_semispan_m"] = 0.5*b
    cfg["structures"]["structure"]["wing_structure_station_x_m"] = wing_ac + _WING_STRUCTURE_X_OFFSET

    # Simple explicit geometric mass scaling for the pre-sized shell/secondary
    # structure.  Primary spar sizing is still calculated by structures.py.
    # Linear area scaling is intentionally conservative/simple and evidence-tagged.
    by = _by_name(cfg)
    if "wing_structure" in by:
        w = by["wing_structure"]
        w["mass_takeoff_kg"] = REF["wing_structure_mass_kg"] * S / REF["wing_area_m2"]
        w["mass_end_kg"] = w["mass_takeoff_kg"]
        _set_position(w, x=wing_ac + _WING_STRUCTURE_X_OFFSET)
        dims = dict(w.get("dimensions_m", {}))
        dims.update({"length_x": mac, "width_y": b})
        w["dimensions_m"] = dims
        w["status"] = "engineering_assumption"
        w["note"] = (str(w.get("note", "")) +
                     " Integrated-v2: baseline wing-structure lump scales linearly with wing area; primary spar is independently sized.").strip()
    if "horizontal_tail_structure" in by:
        h = by["horizontal_tail_structure"]
        h["mass_takeoff_kg"] = REF["horizontal_tail_structure_mass_kg"] * Sh / REF["horizontal_tail_area_m2"]
        h["mass_end_kg"] = h["mass_takeoff_kg"]
        _set_position(h, x=xh)
        h["status"] = "engineering_assumption"

    # Tail spar span scales with sqrt(area) at constant tail AR for screening.
    if str(d.get("surface_geometry_mode", "")) == "root_taper_span":
        cfg["structures"]["structure"]["horizontal_tail_semispan_m"] = 0.5*float(d["horizontal_tail_span_m"])
        cfg["structures"]["structure"]["vertical_tail_height_m"] = float(d["vertical_tail_height_m"])
    else:
        cfg["structures"]["structure"]["horizontal_tail_semispan_m"] = (
            REF["horizontal_tail_semispan_m"] * sqrt(Sh / REF["horizontal_tail_area_m2"])
        )
    cfg["structures"]["structure"]["tail_structure_station_x_m"] = xh

    # Energy-system packaging positions.
    if "buffer_battery" in by:
        _set_position(by["buffer_battery"], x=float(d["battery_x_m"]), z=float(d["battery_z_m"]))
    if "fuel_cell_system" in by:
        _set_position(by["fuel_cell_system"], x=float(d["fuel_cell_x_m"]), z=float(d["fuel_cell_z_m"]))
    if "hydrogen_cylinder" in by:
        _set_position(by["hydrogen_cylinder"], x=float(d["hydrogen_cylinder_x_m"]), z=float(d["hydrogen_cylinder_z_m"]))
    if "stored_hydrogen" in by:
        _set_position(by["stored_hydrogen"], x=float(d["hydrogen_cylinder_x_m"]), z=float(d["hydrogen_cylinder_z_m"]))
    if "hydrogen_regulator" in by:
        # Keep regulator near FC unless a future selected hardware layout overrides it.
        _set_position(by["hydrogen_regulator"], x=float(d["fuel_cell_x_m"]) - 0.01, z=float(d["fuel_cell_z_m"]))

    # Lift rotor rows: enforce propeller edge-clearance geometry about a design centre.
    lay = cfg["aero_propulsion"]["lift_layout"]
    Dm = float(d["lift_propeller_diameter_in"]) * INCH_TO_M
    separation = Dm + float(lay["required_disk_edge_clearance_m"])
    center = float(d["lift_row_center_x_m"])
    xf, xr = center - separation / 2.0, center + separation / 2.0
    lay.update({"current_front_x_m": xf, "current_rear_x_m": xr, "common_midpoint_x_m": center})
    y = abs(float(lay["y_abs_m"])); z = float(lay["z_m"])
    centers = [[xf, -y, z], [xf, y, z], [xr, -y, z], [xr, y, z]]
    geom["lift_prop_centers_m"] = centers
    geom["lift_prop_diameter_m"] = Dm
    for idx, p in enumerate(centers, start=1):
        for prefix in ("lift_motor_mount_", "lift_propulsion_", "lift_ESC_"):
            name = f"{prefix}{idx}"
            if name in by:
                by[name]["position_m"] = list(p)
    cfg["sixdof_control"]["aircraft"]["lift_motor_stations_m"] = centers
    cfg["structures"]["rotor_system"]["row_x_m"] = [xf, xr]

    # Update pusher design power/diameter where those are user-level variables.
    cfg["transition_mission"]["propulsion"]["pusher_max_electrical_power_W"] = float(d["pusher_max_electrical_power_W"])
    cfg["sixdof_control"]["pusher"]["max_electrical_power_W"] = float(d["pusher_max_electrical_power_W"])
    cfg["sixdof_control"]["pusher"]["propeller_diameter_in"] = float(d["pusher_propeller_diameter_in"])

    cfg.setdefault("coupling", {})["resolution"] = {
        "method": "integrated_design_variable_resolver_v2",
        "neutral_point_method": str(d.get("neutral_point_model")),
        "neutral_point_calibration_factor": _TAIL_FACTOR,
        "wing_mac_shape_factor": _MAC_SHAPE_FACTOR,
        "lift_row_separation_m": separation,
        "resolved_lift_rows_x_m": [xf, xr],
        "evidence_note": (
            "Neutral-point and geometry mass scaling are screening relations calibrated to the accessible baseline. "
            "Replace with AVL/CFD/CAD/test correlations before design freeze."
        ),
    }
    return cfg


def design_overrides(cfg: Mapping[str, Any], values: Mapping[str, float]) -> Dict[str, Any]:
    out = deepcopy(dict(cfg))
    for k, v in values.items():
        if k not in out["design"]:
            # New optimizer fields are allowed if resolver recognizes them.
            if k not in {
                "wing_ac_x_m", "horizontal_tail_area_m2", "horizontal_tail_ac_x_m",
                "main_wing_span_m", "horizontal_tail_span_m", "vertical_tail_height_m",
                "lift_row_center_x_m", "battery_x_m", "battery_z_m", "fuel_cell_x_m",
                "fuel_cell_z_m", "hydrogen_cylinder_x_m", "hydrogen_cylinder_z_m",
            }:
                raise KeyError(f"Unknown design override: {k}")
        out["design"][k] = float(v)
    return resolve_design_couplings(out)
