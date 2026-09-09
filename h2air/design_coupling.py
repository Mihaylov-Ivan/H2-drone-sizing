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

from .packaging_layout import apply_packaging_layout

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


def apply_high_wing_spar_stack(
    cfg: MutableMapping[str, Any],
    *,
    spar_od_m: float | None = None,
) -> float:
    """Place wing/boom/HT under the top longeron sleeve for a given primary-spar OD.

    The primary spar sits just below the full-length printed longeron sleeve
    (``boss_od_m``) with ``spar_stack_gap_m``.  Returns the spar OD used.
    """
    d = cfg["design"]
    geom = cfg["geometry_mass"]["geometry_model"]["geometry"]
    lay = cfg["aero_propulsion"]["lift_layout"]
    mm_model = cfg.setdefault("manufacturing_mass_model", {})
    mw = mm_model.setdefault("main_wing", {})
    ht_mm = mm_model.setdefault("horizontal_tail", {})
    longr = mm_model.setdefault("fuselage_longerons", {})

    pod_h = float(d.get("fuselage_height_m", d.get("fuselage_max_diameter_m", 0.18)))
    fuse_top = 0.5 * pod_h
    wing_th = float(mw.get("thickness_m", 0.028))
    boom_od = float(geom.get("boom_outer_diameter_m", 0.035))

    long_od = float(longr.get("tube_od_m", 0.0065))
    long_r = 0.5 * long_od
    boss_od = float(longr.get("boss_od_m", max(2.0 * long_od + 0.004, long_od + 0.006)))
    boss_r = 0.5 * boss_od
    stack_gap = float(longr.get("spar_stack_gap_m", 0.001))
    r_axis = max(fuse_top - long_r, 0.5 * fuse_top)
    z_longeron_top = r_axis
    z_longeron_bot = -r_axis
    boss_bottom = z_longeron_top - boss_r

    if spar_od_m is None:
        # Pre-structures seating uses the mass-calibration reference OD only.
        # After structures, callers pass the recommended screen OD explicitly.
        ps = mw.get("primary_spar") or {}
        spar_od = float(ps.get("reference_tube_od_m", ps.get("tube_od_m", 0.013)))
        st = cfg.get("structures", {}).get("structure", {})
        ref = st.get("reference_wing_spar_od_m", st.get("current_wing_spar_od_m"))
        if ref is not None:
            spar_od = max(spar_od, float(ref))
    else:
        spar_od = max(float(spar_od_m), 1e-6)
    spar_r = 0.5 * spar_od

    wing_z_flush = fuse_top - 0.5 * wing_th
    wing_z = min(wing_z_flush, boss_bottom - stack_gap - spar_r)
    wing_upper = wing_z + 0.5 * wing_th
    wing_lower = wing_z - 0.5 * wing_th

    mw["z_m"] = wing_z
    d["main_wing_z_m"] = wing_z
    ht_mm["z_m"] = wing_z
    d["horizontal_tail_z_m"] = wing_z

    longr["layout"] = "top_bottom"
    longr["boss_od_m"] = boss_od
    longr["spar_stack_gap_m"] = stack_gap
    longr["stations_yz_m"] = [[0.0, z_longeron_top], [0.0, z_longeron_bot]]

    geom["fuselage_top_z_m"] = fuse_top
    geom["fuselage_longeron_top_z_m"] = z_longeron_top
    geom["fuselage_longeron_bottom_z_m"] = z_longeron_bot
    geom["fuselage_longeron_z_m"] = z_longeron_top
    geom["fuselage_longeron_y_abs_m"] = 0.0
    geom["fuselage_longeron_boss_od_m"] = boss_od
    geom["wing_primary_spar_layout_od_m"] = spar_od
    geom["wing_upper_z_m"] = wing_upper
    geom["wing_lower_z_m"] = wing_lower
    geom["wing_z_flush_m"] = wing_z_flush
    geom["wing_drop_from_flush_m"] = max(0.0, wing_z_flush - wing_z)
    geom["boom_z_m"] = wing_z

    # Keep lift motors / prop disk clear of the (possibly lowered) wing.
    clear_wing = float(lay.get("required_disk_edge_clearance_m", lay.get("required_disk_to_pod_clearance_m", 0.02)))
    hw = cfg.get("hardware_selection_result") or {}
    lift_hw = hw.get("selected_lift_combo") or {}
    motor_len = float(lift_hw.get("motor_length_m") or 0.0)
    if motor_len <= 0.0:
        motor_len = 0.03
    z_disk = wing_upper + clear_wing
    z_on_boom = wing_z + 0.5 * boom_od + 0.5 * motor_len
    z_motor = max(z_disk - 0.5 * motor_len, z_on_boom)
    lay["z_m"] = z_motor
    geom["lift_prop_disk_z_m"] = z_motor + 0.5 * motor_len
    geom["lift_prop_wing_clearance_m"] = float(geom["lift_prop_disk_z_m"]) - wing_upper

    # Refresh already-placed lift-motor stations if present.
    centers = geom.get("lift_prop_centers_m")
    if isinstance(centers, list) and centers:
        new_centers = []
        for p in centers:
            if isinstance(p, (list, tuple)) and len(p) >= 3:
                new_centers.append([float(p[0]), float(p[1]), z_motor])
            else:
                new_centers.append(p)
        geom["lift_prop_centers_m"] = new_centers
        by = _by_name(cfg)
        for idx, p in enumerate(new_centers, start=1):
            if not (isinstance(p, list) and len(p) >= 3):
                continue
            for prefix in ("lift_motor_mount_", "lift_propulsion_", "lift_ESC_"):
                name = f"{prefix}{idx}"
                if name in by:
                    by[name]["position_m"] = list(p)
        if "sixdof_control" in cfg and "aircraft" in cfg["sixdof_control"]:
            cfg["sixdof_control"]["aircraft"]["lift_motor_stations_m"] = new_centers

    return spar_od


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
    d.setdefault("vertical_tail_ac_x_m", REF["horizontal_tail_ac_x_m"])
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
        # Fuselage length is user-fixed; diameters may later auto-size via packaging_layout.
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

    # Rule-based internal packaging (may auto-size fuselage section ODs). Run
    # before boom/wing junction so pod diameter drives clearance and longerons.
    apply_packaging_layout(cfg)
    d = cfg["design"]

    # ------------------------------------------------------------------
    # Layout architecture (high-wing twin-boom):
    #   • boom tubes pass through the main wing at wing z
    #   • HT is high-mounted on the same z-plane as the main wing
    #   • lift motors sit on the boom tubes above the wing with disk clearance
    #   • boom spacing widens if disk-to-pod clearance requires it
    #   • HT span grows with boom spacing when fins sit on the boom tips
    #   • cruise pusher mounts on the fuselage aft bulkhead
    # ------------------------------------------------------------------
    lay = cfg["aero_propulsion"]["lift_layout"]
    mm_model = cfg.setdefault("manufacturing_mass_model", {})
    mw = mm_model.setdefault("main_wing", {})
    ht_mm = mm_model.setdefault("horizontal_tail", {})
    longr = mm_model.setdefault("fuselage_longerons", {})

    Dm = float(d["lift_propeller_diameter_in"]) * INCH_TO_M
    pod_w = float(d.get("fuselage_width_m", d.get("fuselage_max_diameter_m", lay.get("central_pod_width_m", 0.18))))
    pod_h = float(d.get("fuselage_height_m", d.get("fuselage_max_diameter_m", pod_w)))
    pod_half = 0.5 * pod_w
    fuse_top = 0.5 * pod_h
    clear_pod = float(lay.get("required_disk_to_pod_clearance_m", 0.02))
    clear_wing = float(lay.get("required_disk_edge_clearance_m", clear_pod))
    y_min = pod_half + 0.5 * Dm + clear_pod

    boom_ys = [float(v) for v in geom.get("boom_center_y_m", [-0.35, 0.35])]
    y_boom_in = max(abs(boom_ys[0]), abs(boom_ys[1])) if boom_ys else 0.35
    y = max(y_boom_in, y_min)
    boom_widened = y > y_boom_in + 1e-9
    geom["boom_center_y_m"] = [-y, y]
    lay["y_abs_m"] = y
    lay["central_pod_width_m"] = pod_w

    boom_od = float(geom.get("boom_outer_diameter_m", 0.035))
    wing_th = float(mw.get("thickness_m", 0.028))

    apply_high_wing_spar_stack(cfg)

    hw = cfg.get("hardware_selection_result") or {}
    lift_hw = hw.get("selected_lift_combo") or {}
    motor_len = float(lift_hw.get("motor_length_m") or 0.0)
    if motor_len <= 0.0:
        motor_len = 0.03

    wing_z = float(mw["z_m"])
    wing_upper = float(geom["wing_upper_z_m"])
    boom_z = float(geom["boom_z_m"])
    wing_z_flush = float(geom["wing_z_flush_m"])
    z_longeron_top = float(geom["fuselage_longeron_top_z_m"])
    z_longeron_bot = float(geom["fuselage_longeron_bottom_z_m"])
    boss_od = float(geom["fuselage_longeron_boss_od_m"])

    # Prop disk above wing upper surface + clearance; motor stack on the boom.
    z_disk = wing_upper + clear_wing
    z_on_boom = boom_z + 0.5 * boom_od + 0.5 * motor_len
    z = max(z_disk - 0.5 * motor_len, z_on_boom)
    lay["z_m"] = z
    geom["lift_prop_disk_z_m"] = z + 0.5 * motor_len
    geom["lift_prop_wing_clearance_m"] = float(geom["lift_prop_disk_z_m"]) - wing_upper

    # Twin-fin / HT layout: horizontal-tail span must reach the boom stations.
    ht_span_in = float(d.get("horizontal_tail_span_m", 2.0 * y))
    ht_widened = False
    if ht_span_in + 1e-9 < 2.0 * y:
        ht_widened = True
        d["horizontal_tail_span_m"] = 2.0 * y
        if str(d.get("surface_geometry_mode", "")) == "root_taper_span":
            chr_ = float(d["horizontal_tail_root_chord_m"])
            lamh = float(d["horizontal_tail_taper_ratio"])
            Sh = float(d["horizontal_tail_span_m"]) * chr_ * (1.0 + lamh) / 2.0
            d["horizontal_tail_area_m2"] = Sh
            d["horizontal_tail_mac_m"] = (2.0 / 3.0) * chr_ * (1.0 + lamh + lamh * lamh) / (1.0 + lamh)
            if str(d.get("neutral_point_model")) == "calibrated_tail_volume":
                d["neutral_point_station_x_m"] = calibrated_neutral_point(
                    wing_ac_x_m=wing_ac, wing_area_m2=S,
                    horizontal_tail_area_m2=Sh, horizontal_tail_ac_x_m=xh,
                )
            geom["reference_neutral_point_x_m"] = float(d["neutral_point_station_x_m"])
            geom["horizontal_tail_span_m"] = float(d["horizontal_tail_span_m"])
            s6a.update({
                "neutral_point_station_x_m": float(d["neutral_point_station_x_m"]),
                "horizontal_tail_area_m2": Sh,
            })
            cfg["structures"]["aircraft"]["horizontal_tail_area_m2"] = Sh
            cfg["structures"]["structure"]["horizontal_tail_semispan_m"] = 0.5 * float(d["horizontal_tail_span_m"])
            if "horizontal_tail_structure" in by:
                h = by["horizontal_tail_structure"]
                h["mass_takeoff_kg"] = REF["horizontal_tail_structure_mass_kg"] * Sh / REF["horizontal_tail_area_m2"]
                h["mass_end_kg"] = h["mass_takeoff_kg"]

    # Lift rotor rows: enforce propeller edge-clearance geometry about a design centre.
    separation = Dm + float(lay["required_disk_edge_clearance_m"])
    center = float(d["lift_row_center_x_m"])
    xf, xr = center - separation / 2.0, center + separation / 2.0
    lay.update({"current_front_x_m": xf, "current_rear_x_m": xr, "common_midpoint_x_m": center})
    centers = [[xf, -y, z], [xf, y, z], [xr, -y, z], [xr, y, z]]
    geom["lift_prop_centers_m"] = centers
    geom["lift_prop_diameter_m"] = Dm
    geom["lift_prop_required_edge_clearance_m"] = float(lay["required_disk_edge_clearance_m"])
    geom["lift_prop_to_pod_required_clearance_m"] = clear_pod
    for idx, p in enumerate(centers, start=1):
        for prefix in ("lift_motor_mount_", "lift_propulsion_", "lift_ESC_"):
            name = f"{prefix}{idx}"
            if name in by:
                by[name]["position_m"] = list(p)
    cfg["sixdof_control"]["aircraft"]["lift_motor_stations_m"] = centers
    cfg["structures"]["rotor_system"]["row_x_m"] = [xf, xr]
    cfg["structures"]["rotor_system"]["motor_y_m"] = y

    # Boom tubes carry front/rear lift motors and the empennage (through the wing).
    boom_x0 = min(float(geom.get("boom_x_start_m", xf)), xf)
    boom_x1 = max(float(geom.get("boom_x_end_m", xr)), float(d.get("horizontal_tail_ac_x_m", xr)), xr)
    geom["boom_x_start_m"] = boom_x0
    geom["boom_x_end_m"] = boom_x1

    # Cruise pusher: mount on fuselage aft bulkhead with standard axial clearance.
    fuse_len = float(d.get("fuselage_length_m", 0.78))
    push_hw = hw.get("selected_pusher_combo") or {}
    p_len = float(push_hw.get("motor_length_m") or 0.034)
    axial_clear = float(geom.get("pusher_required_axial_clearance_m", 0.02))
    px = fuse_len + axial_clear + 0.5 * p_len
    pusher_station = [px, 0.0, 0.0]
    s6a["pusher_station_m"] = pusher_station
    if isinstance(geom.get("pusher"), dict):
        geom["pusher"]["center_m"] = list(pusher_station)
        geom["pusher"]["diameter_m"] = float(d["pusher_propeller_diameter_in"]) * INCH_TO_M
    if "cruise_propulsion" in by:
        _set_position(by["cruise_propulsion"], x=px, y=0.0, z=0.0)

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
        "high_wing_boom_through_wing": True,
        "boom_mounted_lift_motors": True,
        "boom_y_abs_m": y,
        "boom_y_input_m": y_boom_in,
        "boom_z_m": boom_z,
        "boom_widened_for_pod_clearance": boom_widened,
        "main_wing_z_m": wing_z,
        "main_wing_z_flush_m": wing_z_flush,
        "main_wing_drop_from_flush_m": float(geom["wing_drop_from_flush_m"]),
        "wing_primary_spar_layout_od_m": float(geom.get("wing_primary_spar_layout_od_m", 0.0)),
        "fuselage_top_z_m": fuse_top,
        "fuselage_longeron_layout": "top_bottom",
        "fuselage_longeron_top_z_m": z_longeron_top,
        "fuselage_longeron_bottom_z_m": z_longeron_bot,
        "fuselage_longeron_z_m": z_longeron_top,
        "fuselage_longeron_boss_od_m": boss_od,
        "lift_motor_z_m": z,
        "lift_prop_disk_z_m": float(geom["lift_prop_disk_z_m"]),
        "lift_prop_wing_clearance_m": float(geom["lift_prop_wing_clearance_m"]),
        "horizontal_tail_z_m": wing_z,
        "horizontal_tail_span_widened_to_boom": ht_widened,
        "pusher_fuselage_mount": True,
        "pusher_station_m": pusher_station,
        "pusher_fuselage_axial_clearance_m": axial_clear,
        "evidence_note": (
            "Neutral-point and geometry mass scaling are screening relations calibrated to the accessible baseline. "
            "High-wing layout: fuselage longerons are top+bottom on the centreline; wing Z is auto-set so the "
            "TOP longeron boss sits just above the mid-wing primary spar (wing top flush-or-below the fuselage crown); "
            "twin booms pass through the wing; HT shares the wing waterline; lift motors sit on the boom tubes with "
            "prop-disk clearance above the wing; boom/HT spacing widens only if disk-to-pod clearance requires it. "
            "Cruise pusher mounts on the fuselage aft bulkhead. "
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
                "vertical_tail_ac_x_m",
                "main_wing_span_m", "horizontal_tail_span_m", "vertical_tail_height_m",
                "lift_row_center_x_m", "battery_x_m", "battery_z_m", "fuel_cell_x_m",
                "fuel_cell_z_m", "hydrogen_cylinder_x_m", "hydrogen_cylinder_z_m",
                "energy_bay_order_code",
            }:
                raise KeyError(f"Unknown design override: {k}")
        out["design"][k] = float(v)
    # Discrete energy-bay order (0=FC forward, 1=H2 forward) for packaging + sweeps.
    if "energy_bay_order_code" in values or "energy_bay_order_code" in out["design"]:
        code = float(out["design"].get("energy_bay_order_code", 0.0))
        order = "hydrogen_forward" if code >= 0.5 else "fuel_cell_forward"
        out["design"]["energy_bay_order"] = order
        out["design"]["energy_bay_order_code"] = 1.0 if order == "hydrogen_forward" else 0.0
        pl = out.setdefault("packaging_layout", {})
        if isinstance(pl, dict):
            pl["energy_bay_order"] = order
    return resolve_design_couplings(out)
