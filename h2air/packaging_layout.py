"""Rule-based internal packaging layout for the central fuselage pod.

Positions the H₂ cylinder, nose-bay payload/Cube, fuel cell, control box, battery,
and dual Holybro PDBs from ``packaging_layout``. Control-box and battery seats are
chosen to minimise fuselage diameter. Raises ``ValueError`` when rules cannot be met.
"""
from __future__ import annotations

from math import hypot
from typing import Any, Dict, List, Mapping, MutableMapping, Optional, Sequence, Tuple

EPS = 1e-9

# Placed envelope: name, x, y, length, width, height, z
Placed = Tuple[str, float, float, float, float, float, float]


def _f(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except (TypeError, ValueError):
        return float(default)


def _box_dims(raw: Mapping[str, Any], *, fallback: Tuple[float, float, float]) -> Tuple[float, float, float]:
    d = raw.get("dimensions_m") or {}
    if "length" in d and "diameter" in d and "length_x" not in d:
        L, D = _f(d.get("length")), _f(d.get("diameter"))
        return L, D, D
    lx = d.get("length_x", d.get("length"))
    wy = d.get("width_y", d.get("width"))
    hz = d.get("height_z", d.get("height"))
    return (
        _f(lx, fallback[0]) if lx is not None and _f(lx) > 0 else fallback[0],
        _f(wy, fallback[1]) if wy is not None and _f(wy) > 0 else fallback[1],
        _f(hz, fallback[2]) if hz is not None and _f(hz) > 0 else fallback[2],
    )


def _cyl_dims(raw: Mapping[str, Any]) -> Tuple[float, float]:
    d = raw.get("dimensions_m") or {}
    L, D = _f(d.get("length")), _f(d.get("diameter"))
    if L <= 0 or D <= 0:
        raise ValueError("hydrogen_cylinder needs positive length and diameter for packaging layout")
    return L, D


def _max_yz_radius(y0: float, z0: float, wy: float, hz: float) -> float:
    hy, hh = 0.5 * wy, 0.5 * hz
    return max(hypot(y0 + sy * hy, z0 + sz * hh) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0))


def _require_circular_fit(
    name: str,
    *,
    y0: float,
    z0: float,
    wy: float,
    hz: float,
    allowed_r: float,
) -> None:
    r = _max_yz_radius(y0, z0, wy, hz)
    if r > allowed_r + EPS:
        raise ValueError(
            f"Packaging layout: {name} protrudes past fuselage wall keepout "
            f"(radial extent {1000 * r:.1f} mm > allowed {1000 * allowed_r:.1f} mm)."
        )


def _od_from_placed(placed: Sequence[Placed], *, wall_m: float, margin_m: float = 0.0) -> float:
    if not placed:
        return 0.0
    r = max(_max_yz_radius(float(y), float(z), float(wy), float(hz)) for _n, _x, y, _lx, wy, hz, z in placed)
    return 2.0 * (r + float(wall_m) + float(margin_m))


def _boxes_collide(
    a: Placed,
    b: Placed,
    *,
    clearance: float = 0.0,
) -> bool:
    """True on positive AABB penetration. Flush faces (zero overlap) are allowed."""
    del clearance  # seats already insert component gaps; flush mates share a face
    _n1, x1, y1, l1, w1, h1, z1 = a
    _n2, x2, y2, l2, w2, h2, z2 = b
    ox = min(x1 + 0.5 * l1, x2 + 0.5 * l2) - max(x1 - 0.5 * l1, x2 - 0.5 * l2)
    oy = min(y1 + 0.5 * w1, y2 + 0.5 * w2) - max(y1 - 0.5 * w1, y2 - 0.5 * w2)
    oz = min(z1 + 0.5 * h1, z2 + 0.5 * h2) - max(z1 - 0.5 * h1, z2 - 0.5 * h2)
    return ox > EPS and oy > EPS and oz > EPS


def _set_ledger_box(
    by: MutableMapping[str, Any],
    name: str,
    *,
    pos: Sequence[float],
    dims: Tuple[float, float, float],
    status: str,
    source: str,
    note: str,
    collide: bool = True,
    mass_kg: float | None = None,
    group: str = "energy",
    components: list | None = None,
) -> None:
    if name not in by:
        if components is None:
            return
        entry = {
            "name": name,
            "group": group,
            "mass_takeoff_kg": float(mass_kg or 0.0),
            "mass_end_kg": float(mass_kg or 0.0),
            "position_m": [float(pos[0]), float(pos[1]), float(pos[2])],
            "shape": "box",
            "dimensions_m": {
                "length_x": float(dims[0]),
                "width_y": float(dims[1]),
                "height_z": float(dims[2]),
            },
            "package_zone": "central_pod",
            "include_in_collision_check": collide,
            "status": status,
            "source": source,
            "note": note,
        }
        components.append(entry)
        by[name] = entry
        return
    c = by[name]
    c["position_m"] = [float(pos[0]), float(pos[1]), float(pos[2])]
    c["shape"] = "box"
    c["dimensions_m"] = {
        "length_x": float(dims[0]),
        "width_y": float(dims[1]),
        "height_z": float(dims[2]),
    }
    c["package_zone"] = "central_pod"
    c["include_in_collision_check"] = collide
    c["status"] = status
    c["source"] = source
    c["note"] = note
    if mass_kg is not None:
        c["mass_takeoff_kg"] = float(mass_kg)
        c["mass_end_kg"] = float(mass_kg)


def _fuselage_sections(cfg: Mapping[str, Any]) -> list:
    return list(
        cfg.get("manufacturing_mass_model", {}).get("fuselage_sections")
        or (cfg.get("user_input_snapshot") or {}).get("fuselage", {}).get("sections")
        or []
    )


def _section_x_ranges(sections: Sequence[Mapping[str, Any]]) -> list[tuple[str, float, float]]:
    out: list[tuple[str, float, float]] = []
    x = 0.0
    for sec in sections:
        L = _f(sec.get("length_m"))
        name = str(sec.get("name", f"section_{len(out)}"))
        out.append((name, x, x + L))
        x += L
    return out


def _apply_uniform_fuse_od(cfg: MutableMapping[str, Any], od: float) -> None:
    """Write a uniform fuselage OD into design + section records (placement frame)."""
    od = float(od)
    d = cfg["design"]
    d["fuselage_max_diameter_m"] = od
    fuse_len = _f(d.get("fuselage_length_m"))
    geom = cfg["geometry_mass"]["geometry_model"]["geometry"]
    geom["central_pod_outer"] = {
        "center_m": [fuse_len / 2.0, 0.0, 0.0],
        "dimensions_m": [fuse_len, od, od],
    }
    env = cfg["geometry_mass"]["geometry_model"]["packaging"]["envelopes"]["central_pod"]
    env["center_m"] = [fuse_len / 2.0, 0.0, 0.0]
    env["dimensions_m"] = [fuse_len, od, od]
    cfg["aero_propulsion"]["lift_layout"]["central_pod_width_m"] = od
    for sec in _fuselage_sections(cfg):
        sec["diameter_m"] = od
    snap = cfg.get("user_input_snapshot")
    if isinstance(snap, dict) and isinstance(snap.get("fuselage"), dict):
        for sec in snap["fuselage"].get("sections") or []:
            sec["diameter_m"] = od


def _commit_section_ods(
    cfg: MutableMapping[str, Any],
    section_ods: Mapping[str, float],
) -> None:
    """Persist per-section ODs; design max OD is the envelope maximum."""
    max_od = max(float(v) for v in section_ods.values()) if section_ods else _f(cfg["design"].get("fuselage_max_diameter_m"))
    d = cfg["design"]
    fuse_len = _f(d.get("fuselage_length_m"))
    d["fuselage_max_diameter_m"] = max_od
    d["fuselage_section_diameters_m"] = {k: float(v) for k, v in section_ods.items()}
    geom = cfg["geometry_mass"]["geometry_model"]["geometry"]
    geom["central_pod_outer"] = {
        "center_m": [fuse_len / 2.0, 0.0, 0.0],
        "dimensions_m": [fuse_len, max_od, max_od],
    }
    env = cfg["geometry_mass"]["geometry_model"]["packaging"]["envelopes"]["central_pod"]
    env["center_m"] = [fuse_len / 2.0, 0.0, 0.0]
    env["dimensions_m"] = [fuse_len, max_od, max_od]
    cfg["aero_propulsion"]["lift_layout"]["central_pod_width_m"] = max_od
    for sec in _fuselage_sections(cfg):
        name = str(sec.get("name", ""))
        if name in section_ods:
            sec["diameter_m"] = float(section_ods[name])
    snap = cfg.get("user_input_snapshot")
    if isinstance(snap, dict) and isinstance(snap.get("fuselage"), dict):
        for sec in snap["fuselage"].get("sections") or []:
            name = str(sec.get("name", ""))
            if name in section_ods:
                sec["diameter_m"] = float(section_ods[name])


def _section_ods_from_placed(
    placed: Sequence[Placed],
    sections: Sequence[Mapping[str, Any]],
    *,
    wall_m: float,
    margin_m: float = 0.0,
    min_od_m: float = 0.12,
) -> Dict[str, float]:
    """Required outer diameter per fuselage section from component YZ radial extent + wall keepout."""
    ranges = _section_x_ranges(sections)
    needs = {name: float(min_od_m) for name, _a, _b in ranges}
    for _name, x, y, lx, wy, hz, z in placed:
        r = _max_yz_radius(float(y), float(z), float(wy), float(hz))
        need = 2.0 * (r + float(wall_m) + float(margin_m))
        x0, x1 = float(x) - 0.5 * float(lx), float(x) + 0.5 * float(lx)
        for sec_name, a, b in ranges:
            if x1 <= a + EPS or x0 >= b - EPS:
                continue
            needs[sec_name] = max(needs[sec_name], need)
    return needs


def apply_packaging_layout(cfg: MutableMapping[str, Any]) -> Dict[str, Any]:
    """Resolve internal stations from ``packaging_layout`` rules; raise on violation.

    When ``fuselage.diameter_mode`` is ``auto``, section outer diameters are iterated
    from packed component radial extents + wall keepout (+ optional margin).
    """
    layout = cfg.get("packaging_layout") or {}
    if not bool(layout.get("enabled", False)):
        return {"enabled": False}

    fus_snap = (cfg.get("user_input_snapshot") or {}).get("fuselage") or {}
    auto = str(fus_snap.get("diameter_mode", "hardcoded")).lower() == "auto"
    if not auto:
        return _packaging_layout_core(cfg)

    wall = _f(
        cfg.get("geometry_mass", {})
        .get("geometry_model", {})
        .get("packaging", {})
        .get("envelopes", {})
        .get("central_pod", {})
        .get("wall_clearance_m"),
        0.003,
    )
    margin = _f(fus_snap.get("diameter_margin_m"), 0.0)
    min_od = _f(fus_snap.get("minimum_diameter_m"), 0.12)
    od = max(_f(cfg["design"].get("fuselage_max_diameter_m"), min_od), min_od)
    last_err: Exception | None = None
    summary: Dict[str, Any] = {}
    for _ in range(32):
        _apply_uniform_fuse_od(cfg, od)
        try:
            summary = _packaging_layout_core(cfg)
        except ValueError as e:
            last_err = e
            msg = str(e)
            if "Packaging layout" in msg and any(
                k in msg
                for k in (
                    "protrudes",
                    "bay needs",
                    "top bay",
                    "exceeds",
                    "headroom",
                    "FCPM top",
                    "Increase fuselage diameter",
                    "does not fit",
                    "no room",
                    "collides",
                )
            ):
                od = max(od + 0.010, od * 1.04)
                continue
            raise
        placed_raw = summary.get("placed_envelopes") or []
        placed_tuples: List[Placed] = [
            (
                p["name"],
                float(p["x"]),
                float(p.get("y", 0.0)),
                float(p["length"]),
                float(p["width"]),
                float(p["height"]),
                float(p["z"]),
            )
            for p in placed_raw
        ]
        sections = _fuselage_sections(cfg)
        section_ods = _section_ods_from_placed(
            placed_tuples, sections, wall_m=wall, margin_m=margin, min_od_m=min_od
        )
        need = max(section_ods.values()) if section_ods else od
        if need > od + 5e-5:
            od = need
            continue
        if section_ods:
            driver = max(section_ods, key=section_ods.get)
            section_ods[driver] = max(float(section_ods[driver]), float(od))
        _commit_section_ods(cfg, section_ods)
        summary = _packaging_layout_core(cfg)
        summary["fuselage_diameter_mode"] = "auto"
        summary["fuselage_section_ods_m"] = dict(section_ods)
        summary["fuselage_od_m"] = float(cfg["design"]["fuselage_max_diameter_m"])
        cfg.setdefault("coupling", {})["packaging_layout"] = summary
        return summary
    raise ValueError(
        "Packaging layout: fuselage auto-diameter did not converge"
        + (f" ({last_err})" if last_err else "")
    )


def _seat_on_fc(
    *,
    site: str,
    name: str,
    L: float,
    W: float,
    H: float,
    x_fc: float,
    z_fc: float,
    fc_L: float,
    fc_W: float,
    fc_H: float,
    stack_v: float,
    comp_c: float,
    side_sign: float,
) -> Placed:
    """Place a box on/ beside the fuel cell. ``site`` is fc_top or fc_side."""
    fc_top = z_fc + 0.5 * fc_H
    if site == "fc_top":
        y = 0.0 if abs(side_sign) < EPS else side_sign * (0.5 * W + 0.5 * comp_c)
        return (name, x_fc, y, L, W, H, fc_top + stack_v + 0.5 * H)
    if site == "fc_side":
        y = side_sign * (0.5 * fc_W + comp_c + 0.5 * W)
        return (name, x_fc, y, L, W, H, z_fc)
    raise ValueError(f"Unknown FC seat {site}")


def _seat_on_h2(
    *,
    name: str,
    L: float,
    W: float,
    H: float,
    x_cyl_fwd: float,
    cyl_top: float,
    stack_v: float,
    y: float = 0.0,
) -> Placed:
    """Front face flush with H₂ cylinder forward face, sitting on the cylinder top."""
    x = x_cyl_fwd + 0.5 * L
    z = cyl_top + stack_v + 0.5 * H
    return (name, x, y, L, W, H, z)


def _place_pdbs_on_cylinder(
    *,
    pdb_L: float,
    pdb_W: float,
    pdb_H: float,
    x_cyl: float,
    x_cyl_fwd: float,
    x_cyl_aft: float,
    cyl_top: float,
    pdb_around: float,
    comp_c: float,
    occupants: Sequence[Placed],
) -> List[Placed]:
    """Two PDBs on the cylinder top shelf — never stacked on CB/battery."""
    z = cyl_top + pdb_around + 0.5 * pdb_H
    gap = max(comp_c, 0.5 * pdb_around)

    def _ok(cands: Sequence[Placed]) -> bool:
        all_boxes = list(occupants) + list(cands)
        for i in range(len(all_boxes)):
            for j in range(i + 1, len(all_boxes)):
                if _boxes_collide(all_boxes[i], all_boxes[j], clearance=comp_c):
                    return False
            lo, hi = all_boxes[i][1] - 0.5 * all_boxes[i][3], all_boxes[i][1] + 0.5 * all_boxes[i][3]
            if lo < x_cyl_fwd - EPS or hi > x_cyl_aft + EPS:
                return False
        return True

    # Prefer side-by-side at cylinder centre.
    y_off = 0.5 * pdb_W + 0.5 * gap
    side_by_side = [
        ("power_module", x_cyl, +y_off, pdb_L, pdb_W, pdb_H, z),
        ("power_module_2", x_cyl, -y_off, pdb_L, pdb_W, pdb_H, z),
    ]
    if _ok(side_by_side):
        return side_by_side

    # Aft of the forward-most occupant on the cylinder.
    aft_of = x_cyl_fwd
    for _n, x, _y, lx, _w, _h, _z in occupants:
        aft_of = max(aft_of, x + 0.5 * lx)
    x_pdb = aft_of + comp_c + 0.5 * pdb_L
    aft_pair = [
        ("power_module", x_pdb, +y_off, pdb_L, pdb_W, pdb_H, z),
        ("power_module_2", x_pdb, -y_off, pdb_L, pdb_W, pdb_H, z),
    ]
    if _ok(aft_pair):
        return aft_pair

    # Single-file aft on centreline if sides are blocked.
    x1 = aft_of + comp_c + 0.5 * pdb_L
    x2 = x1 + 0.5 * pdb_L + comp_c + 0.5 * pdb_L
    tandem = [
        ("power_module", x1, 0.0, pdb_L, pdb_W, pdb_H, z),
        ("power_module_2", x2, 0.0, pdb_L, pdb_W, pdb_H, z),
    ]
    if _ok(tandem):
        return tandem

    raise ValueError(
        "Packaging layout: cannot place two PDBs on the H₂ cylinder top without stacking "
        "on the control box/battery or leaving the cylinder bay. Increase fuselage diameter "
        "or lengthen the cylinder bay."
    )


def _candidate_plans() -> List[Dict[str, Any]]:
    """Enumerate CB/battery seats. Prefer split-to-H2 on OD ties."""
    plans: List[Dict[str, Any]] = []
    # Split layouts first in tie-break order (lower rank preferred when OD equal).
    for cb_site, bat_site, rank in [
        ("fc_top", "h2_top", 0),
        ("fc_side", "h2_top", 1),
        ("h2_top", "fc_top", 2),
        ("h2_top", "fc_side", 3),
        ("fc_top", "fc_side", 4),
        ("fc_side", "fc_top", 5),
        ("fc_top", "fc_top", 6),
        ("fc_side", "fc_side", 7),
        ("h2_top", "h2_top", 8),
    ]:
        plans.append({"cb": cb_site, "bat": bat_site, "tie_rank": rank})
    return plans


def _packaging_layout_core(cfg: MutableMapping[str, Any]) -> Dict[str, Any]:
    """One packaging placement pass at the current fuselage max OD (min-OD seat search)."""
    layout = cfg.get("packaging_layout") or {}
    if not bool(layout.get("enabled", False)):
        return {"enabled": False}

    d = cfg["design"]
    clear = layout.get("clearances_m") or {}
    hw = layout.get("hardware") or {}
    pdb = hw.get("holybro_pdb_300a") or {}
    cube = hw.get("cube_orange") or {}

    bottom_c = _f(clear.get("fuselage_bottom_m"), 0.020)
    aft_beyond_motor = _f(clear.get("aft_tail_beyond_motor_m"), 0.020)
    pdb_around = _f(clear.get("pdb_all_around_m"), 0.020)
    comp_c = _f(clear.get("component_m"), 0.003)
    stack_v = _f(clear.get("bottom_to_top_m"), comp_c)
    cube_front = _f(clear.get("cube_front_connector_m"), 0.050)
    min_cyl_top = _f(clear.get("cylinder_to_top_inner_m"), 0.030)
    wall = _f(
        cfg.get("geometry_mass", {})
        .get("geometry_model", {})
        .get("packaging", {})
        .get("envelopes", {})
        .get("central_pod", {})
        .get("wall_clearance_m"),
        0.003,
    )

    fuse_len = _f(d.get("fuselage_length_m"))
    fuse_od = _f(d.get("fuselage_max_diameter_m"))
    if fuse_len <= 0 or fuse_od <= 0:
        raise ValueError("Packaging layout requires positive fuselage length and diameter")
    R = 0.5 * fuse_od
    z_bot = -R + wall
    z_top = R - wall
    allowed_r = R - wall

    hw_sel = cfg.get("hardware_selection_result") or {}
    push = hw_sel.get("selected_pusher_combo") or {}
    motor_len = _f(push.get("motor_length_m"), 0.034)
    if motor_len <= 0:
        motor_len = 0.034

    comps = cfg["geometry_mass"]["component_ledger"]["components"]
    by = {c["name"]: c for c in comps}

    for name, env in (layout.get("envelopes") or {}).items():
        if name not in by or not isinstance(env, Mapping):
            continue
        c = by[name]
        shape = str(env.get("shape") or c.get("shape") or "box")
        if shape.startswith("cylinder"):
            c["shape"] = shape
            c["dimensions_m"] = {
                "length": _f(env.get("length_m", (c.get("dimensions_m") or {}).get("length"))),
                "diameter": _f(env.get("diameter_m", (c.get("dimensions_m") or {}).get("diameter"))),
            }
        else:
            c["shape"] = "box"
            prev = c.get("dimensions_m") or {}
            c["dimensions_m"] = {
                "length_x": _f(env.get("length_m", prev.get("length_x")), 0.1),
                "width_y": _f(env.get("width_m", prev.get("width_y")), 0.1),
                "height_z": _f(env.get("height_m", prev.get("height_z")), 0.1),
            }
        if env.get("note"):
            c["note"] = str(env["note"])
        if env.get("status"):
            c["status"] = str(env["status"])
        if env.get("source"):
            c["source"] = str(env["source"])

    if "hydrogen_cylinder" not in by:
        raise ValueError("Packaging layout requires hydrogen_cylinder in the component ledger")
    L_cyl, D_cyl = _cyl_dims(by["hydrogen_cylinder"])
    aft_keepout = motor_len + aft_beyond_motor
    x_aft_limit = fuse_len - aft_keepout

    order_raw = str(
        d.get("energy_bay_order")
        or layout.get("energy_bay_order")
        or "fuel_cell_forward"
    ).lower().strip()
    if "energy_bay_order_code" in d:
        order_raw = "hydrogen_forward" if _f(d.get("energy_bay_order_code")) >= 0.5 else "fuel_cell_forward"
    if order_raw not in {"fuel_cell_forward", "hydrogen_forward"}:
        raise ValueError(
            "Packaging layout: energy_bay_order must be 'fuel_cell_forward' or 'hydrogen_forward'"
        )
    energy_bay_order = order_raw

    z_cyl = z_bot + bottom_c + 0.5 * D_cyl
    cyl_top = z_cyl + 0.5 * D_cyl
    top_gap = z_top - cyl_top

    pdb_L = _f(pdb.get("length_m"), 0.080)
    pdb_W = _f(pdb.get("width_m"), 0.060)
    pdb_H = _f(pdb.get("height_m"), 0.0235)
    pdb_count = max(1, int(_f(pdb.get("count"), 2)))
    pdb_need = pdb_H + 2.0 * pdb_around
    top_need = max(min_cyl_top, pdb_need)
    if top_gap + EPS < top_need:
        raise ValueError(
            f"Packaging layout: need {1000 * top_need:.1f} mm between H₂ cylinder top and fuselage "
            f"top inner wall (max of cylinder_to_top_inner={1000 * min_cyl_top:.1f} mm and "
            f"PDB {1000 * pdb_H:.1f} mm + 2×{1000 * pdb_around:.0f} mm clearances), "
            f"but only {1000 * top_gap:.1f} mm is available at OD={1000 * fuse_od:.0f} mm with "
            f"bottom clearance {1000 * bottom_c:.0f} mm. Increase fuselage diameter or reduce envelopes."
        )
    _require_circular_fit("hydrogen_cylinder", y0=0.0, z0=z_cyl, wy=D_cyl, hz=D_cyl, allowed_r=allowed_r)

    fuse_half = 0.55 * fuse_od
    fc_hw = hw.get("fuel_cell_stack") or {}
    cb_hw = hw.get("fuel_cell_control_box") or {}
    fc_L = _f(fc_hw.get("length_m"), 0.2305)
    fc_W = _f(fc_hw.get("width_m"), 0.1084)
    fc_H = _f(fc_hw.get("height_m"), 0.1198)
    cb_L = _f(cb_hw.get("length_m"), 0.2022)
    cb_W = _f(cb_hw.get("width_m"), 0.107)
    cb_H = _f(cb_hw.get("height_m"), 0.04515)

    module_mass = _f(
        (cfg.get("power_system") or {}).get("fuel_cell", {}).get("module_mass_kg"),
        _f(fc_hw.get("mass_kg"), 0.0) + _f(cb_hw.get("mass_kg"), 0.0),
    )
    if module_mass <= 0:
        module_mass = 1.45
    m_fc_in = fc_hw.get("mass_kg")
    m_cb_in = cb_hw.get("mass_kg")
    if m_fc_in is not None and m_cb_in is not None:
        fc_mass = _f(m_fc_in)
        cb_mass = _f(m_cb_in)
    else:
        v_fc = max(fc_L * fc_W * fc_H, EPS)
        v_cb = max(cb_L * cb_W * cb_H, EPS)
        fc_mass = module_mass * v_fc / (v_fc + v_cb)
        cb_mass = module_mass - fc_mass
    if abs((fc_mass + cb_mass) - module_mass) > 1e-6:
        raise ValueError(
            f"Packaging layout: fuel_cell_stack ({fc_mass:.4f} kg) + control_box ({cb_mass:.4f} kg) "
            f"must equal module_mass_kg ({module_mass:.4f} kg)."
        )

    bat_fb = (0.16, 0.05, 0.05)
    if "buffer_battery" in by:
        bat_L, bat_W, bat_H = _box_dims(by["buffer_battery"], fallback=bat_fb)
    else:
        bat_L, bat_W, bat_H = bat_fb
    if "payload" in by:
        pay_L, pay_W, pay_H = _box_dims(
            by["payload"],
            fallback=(0.053, min(0.08, fuse_half), min(0.08, fuse_half)),
        )
    else:
        pay_L = pay_W = pay_H = min(0.08, fuse_half)

    cube_L = _f(cube.get("length_m"), 0.0945)
    cube_W = _f(cube.get("width_m"), 0.0443)
    cube_H = _f(cube.get("height_m"), 0.031)

    sections = _fuselage_sections(cfg)
    nose_len = 0.0
    for sec in sections:
        if str(sec.get("name", "")).lower() == "nose":
            nose_len = _f(sec.get("length_m"))
            break
    if nose_len <= 0 and sections:
        nose_len = _f(sections[0].get("length_m"))
    if nose_len <= 0:
        raise ValueError("Packaging layout: fuselage nose section length is required")

    # ---- Nose bay: payload + Cube, aft faces flush with nose-section end ----
    x_pay_aft = nose_len
    x_pay = x_pay_aft - 0.5 * pay_L
    x_pay_fwd = x_pay - 0.5 * pay_L
    if x_pay_fwd < wall - EPS:
        raise ValueError(
            f"Packaging layout: payload length {1000 * pay_L:.1f} mm does not fit with its aft face "
            f"flush at the nose-section end x={1000 * nose_len:.1f} mm."
        )
    z_pay = z_bot + bottom_c + 0.5 * pay_H
    pay_top = z_pay + 0.5 * pay_H
    _require_circular_fit("payload", y0=0.0, z0=z_pay, wy=pay_W, hz=pay_H, allowed_r=allowed_r)

    x_cube_aft = nose_len
    x_cube = x_cube_aft - 0.5 * cube_L
    cube_fwd = x_cube - 0.5 * cube_L
    if cube_fwd < wall - EPS:
        raise ValueError(
            f"Packaging layout: Cube Orange length {1000 * cube_L:.1f} mm does not fit in the nose "
            f"with aft face flush at x={1000 * nose_len:.1f} mm."
        )
    # Prefer the requested connector bay; truncate to the free nose length if needed.
    connector_start = max(wall, cube_fwd - cube_front)
    connector_len = cube_fwd - connector_start
    z_cube = pay_top + stack_v + 0.5 * cube_H
    if z_cube + 0.5 * cube_H + comp_c > z_top + EPS:
        raise ValueError(
            "Packaging layout: Cube Orange over payload exceeds the fuselage top inner wall. "
            "Increase fuselage diameter or reduce envelopes."
        )
    _require_circular_fit("cube_orange", y0=0.0, z0=z_cube, wy=cube_W, hz=cube_H, allowed_r=allowed_r)

    # ---- FC / H₂ longitudinal order (nose-break forward item, aft item to tail keepout) ----
    if energy_bay_order == "fuel_cell_forward":
        x_fc_fwd = nose_len
        x_fc = x_fc_fwd + 0.5 * fc_L
        x_fc_aft = x_fc + 0.5 * fc_L
        x_cyl_aft = x_aft_limit
        x_cyl = x_cyl_aft - 0.5 * L_cyl
        x_cyl_fwd = x_cyl - 0.5 * L_cyl
        if x_cyl_fwd < wall - EPS:
            raise ValueError(
                f"Packaging layout: H₂ cylinder cannot sit aft with motor+{1000 * aft_beyond_motor:.0f} mm "
                f"tail keepout — cylinder length {1000 * L_cyl:.0f} mm does not fit."
            )
        if x_fc_aft + comp_c > x_cyl_fwd + EPS:
            raise ValueError(
                "Packaging layout: fuel cell (front flush with nose end) collides with the H₂ cylinder "
                "forward keepout. Lengthen the fuselage or shorten envelopes."
            )
    else:
        x_cyl_fwd = nose_len
        x_cyl = x_cyl_fwd + 0.5 * L_cyl
        x_cyl_aft = x_cyl + 0.5 * L_cyl
        x_fc_aft = x_aft_limit
        x_fc = x_fc_aft - 0.5 * fc_L
        x_fc_fwd = x_fc - 0.5 * fc_L
        if x_fc_fwd < wall - EPS:
            raise ValueError(
                f"Packaging layout: fuel cell cannot sit aft with motor+{1000 * aft_beyond_motor:.0f} mm "
                f"tail keepout — FC length {1000 * fc_L:.0f} mm does not fit."
            )
        if x_cyl_aft + comp_c > x_fc_fwd + EPS:
            raise ValueError(
                "Packaging layout: H₂ cylinder (front flush with nose end) collides with the fuel cell "
                "forward keepout. Lengthen the fuselage or shorten envelopes."
            )

    z_fc = z_bot + bottom_c + 0.5 * fc_H
    fc_top = z_fc + 0.5 * fc_H
    if fc_top + stack_v > z_top + EPS:
        raise ValueError(
            "Packaging layout: fuel cell height exceeds fuselage top inner wall with bottom clearance. "
            "Increase fuselage diameter."
        )
    _require_circular_fit("fuel_cell_system", y0=0.0, z0=z_fc, wy=fc_W, hz=fc_H, allowed_r=allowed_r)

    base: List[Placed] = [
        ("payload", x_pay, 0.0, pay_L, pay_W, pay_H, z_pay),
        ("autopilot", x_cube, 0.0, cube_L, cube_W, cube_H, z_cube),
        ("fuel_cell_system", x_fc, 0.0, fc_L, fc_W, fc_H, z_fc),
        ("hydrogen_cylinder", x_cyl, 0.0, L_cyl, D_cyl, D_cyl, z_cyl),
    ]

    best: Optional[Dict[str, Any]] = None
    errors: List[str] = []

    for plan in _candidate_plans():
        try:
            cb_site = str(plan["cb"])
            bat_site = str(plan["bat"])
            # Side signs: opposite sides when both beside/on-top of FC; centred when alone on top.
            if cb_site == "fc_side" and bat_site == "fc_side":
                cb_sign, bat_sign = 1.0, -1.0
            elif cb_site == "fc_top" and bat_site == "fc_top":
                cb_sign, bat_sign = 1.0, -1.0
            elif cb_site == "fc_top":
                cb_sign, bat_sign = 0.0, -1.0
            elif bat_site == "fc_top":
                cb_sign, bat_sign = 1.0, 0.0
            else:
                cb_sign, bat_sign = 1.0, -1.0

            if cb_site == "h2_top":
                if cb_L > L_cyl + EPS:
                    raise ValueError("control box longer than H₂ cylinder")
                p_cb = _seat_on_h2(
                    name="fuel_cell_control_box",
                    L=cb_L,
                    W=cb_W,
                    H=cb_H,
                    x_cyl_fwd=x_cyl_fwd,
                    cyl_top=cyl_top,
                    stack_v=stack_v,
                )
                if p_cb[1] + 0.5 * cb_L > x_cyl_aft + EPS:
                    raise ValueError("control box overhangs H₂ cylinder aft")
            else:
                if cb_L > fc_L + EPS and cb_site == "fc_top":
                    raise ValueError("control box longer than fuel cell for top seat")
                p_cb = _seat_on_fc(
                    site=cb_site,
                    name="fuel_cell_control_box",
                    L=cb_L,
                    W=cb_W,
                    H=cb_H,
                    x_fc=x_fc,
                    z_fc=z_fc,
                    fc_L=fc_L,
                    fc_W=fc_W,
                    fc_H=fc_H,
                    stack_v=stack_v,
                    comp_c=comp_c,
                    side_sign=cb_sign,
                )

            if bat_site == "h2_top":
                # If CB already owns the cylinder front, offset battery laterally.
                y_bat = 0.0
                if cb_site == "h2_top":
                    y_bat = 0.5 * cb_W + comp_c + 0.5 * bat_W
                if bat_L > L_cyl + EPS:
                    raise ValueError("battery longer than H₂ cylinder")
                p_bat = _seat_on_h2(
                    name="buffer_battery",
                    L=bat_L,
                    W=bat_W,
                    H=bat_H,
                    x_cyl_fwd=x_cyl_fwd,
                    cyl_top=cyl_top,
                    stack_v=stack_v,
                    y=y_bat,
                )
                if p_bat[1] + 0.5 * bat_L > x_cyl_aft + EPS:
                    raise ValueError("battery overhangs H₂ cylinder aft")
            else:
                if bat_L > fc_L + EPS and bat_site == "fc_top":
                    raise ValueError("battery longer than fuel cell for top seat")
                p_bat = _seat_on_fc(
                    site=bat_site,
                    name="buffer_battery",
                    L=bat_L,
                    W=bat_W,
                    H=bat_H,
                    x_fc=x_fc,
                    z_fc=z_fc,
                    fc_L=fc_L,
                    fc_W=fc_W,
                    fc_H=fc_H,
                    stack_v=stack_v,
                    comp_c=comp_c,
                    side_sign=bat_sign,
                )

            h2_occupants = [p for p in (p_cb, p_bat) if p[0] in {"fuel_cell_control_box", "buffer_battery"} and (
                (p[0] == "fuel_cell_control_box" and cb_site == "h2_top")
                or (p[0] == "buffer_battery" and bat_site == "h2_top")
            )]
            pdbs = _place_pdbs_on_cylinder(
                pdb_L=pdb_L,
                pdb_W=pdb_W,
                pdb_H=pdb_H,
                x_cyl=x_cyl,
                x_cyl_fwd=x_cyl_fwd,
                x_cyl_aft=x_cyl_aft,
                cyl_top=cyl_top,
                pdb_around=pdb_around,
                comp_c=comp_c,
                occupants=h2_occupants,
            )
            if pdb_count < 2:
                pdbs = pdbs[:1]

            placed = list(base) + [p_cb, p_bat] + list(pdbs)

            # Circular fit + top wall + pairwise clearance.
            for n, _x, y, _lx, wy, hz, z in placed:
                _require_circular_fit(n, y0=y, z0=z, wy=wy, hz=hz, allowed_r=allowed_r)
                if z + 0.5 * hz + (pdb_around if n.startswith("power_module") else comp_c) > z_top + EPS:
                    raise ValueError(f"{n} exceeds fuselage top inner wall")

            for i in range(len(placed)):
                for j in range(i + 1, len(placed)):
                    if _boxes_collide(placed[i], placed[j], clearance=comp_c):
                        raise ValueError(f"{placed[i][0]} collides with {placed[j][0]}")

            # Cube forward connector bay at cube height.
            bay = (connector_start, cube_fwd)
            for n, x, y, lx, wy, hz, z in placed:
                if n == "autopilot":
                    continue
                lo, hi = x - 0.5 * lx, x + 0.5 * lx
                if hi > bay[0] + EPS and lo < bay[1] - EPS:
                    if abs(z - z_cube) < 0.5 * (hz + cube_H) - EPS and abs(y) < 0.5 * (wy + cube_W) + comp_c:
                        raise ValueError(f"{n} intrudes into Cube Orange connector bay")

            od_need = _od_from_placed(placed, wall_m=wall)
            if od_need > fuse_od + 5e-5:
                raise ValueError(
                    f"layout needs OD {1000 * od_need:.1f} mm > current {1000 * fuse_od:.1f} mm"
                )

            score = (od_need, int(plan["tie_rank"]))
            if best is None or score < best["score"]:
                best = {
                    "score": score,
                    "placed": placed,
                    "cb_site": cb_site,
                    "bat_site": bat_site,
                    "p_cb": p_cb,
                    "p_bat": p_bat,
                    "pdbs": pdbs,
                    "od_need": od_need,
                }
        except ValueError as e:
            errors.append(f"{plan['cb']}+{plan['bat']}: {e}")
            continue

    if best is None:
        detail = "; ".join(errors[:4])
        raise ValueError(
            "Packaging layout: no control-box/battery seat combination fits the current fuselage. "
            f"Increase fuselage diameter or revise envelopes. Tried: {detail}"
        )

    p_cb = best["p_cb"]
    p_bat = best["p_bat"]
    pdbs = best["pdbs"]
    placed: List[Placed] = best["placed"]
    x_cb, y_cb, z_cb = p_cb[1], p_cb[2], p_cb[6]
    x_bat, y_bat, z_bat = p_bat[1], p_bat[2], p_bat[6]

    # ---- Write design + ledger ----
    d["hydrogen_cylinder_x_m"] = x_cyl
    d["hydrogen_cylinder_z_m"] = z_cyl
    d["fuel_cell_x_m"] = x_fc
    d["fuel_cell_z_m"] = z_fc
    d["fuel_cell_control_box_x_m"] = x_cb
    d["fuel_cell_control_box_y_m"] = y_cb
    d["fuel_cell_control_box_z_m"] = z_cb
    d["fuel_cell_stack_mass_kg"] = fc_mass
    d["fuel_cell_control_box_mass_kg"] = cb_mass
    d["battery_x_m"] = x_bat
    d["battery_y_m"] = y_bat
    d["battery_z_m"] = z_bat
    d["cube_orange_x_m"] = x_cube
    d["cube_orange_z_m"] = z_cube
    d["payload_x_m"] = x_pay
    d["payload_z_m"] = z_pay
    d["nose_section_end_x_m"] = nose_len
    d["energy_bay_order"] = energy_bay_order
    d["energy_bay_order_code"] = 1.0 if energy_bay_order == "hydrogen_forward" else 0.0
    d["packaging_control_box_site"] = best["cb_site"]
    d["packaging_battery_site"] = best["bat_site"]
    if isinstance(layout, dict):
        layout["energy_bay_order"] = energy_bay_order

    for i, p in enumerate(pdbs):
        key = "pdb" if i == 0 else f"pdb{i + 1}"
        d[f"{key}_x_m"] = p[1]
        d[f"{key}_y_m"] = p[2]
        d[f"{key}_z_m"] = p[6]

    def _pos(c: MutableMapping[str, Any], x: float, y: float, z: float) -> None:
        p = list(c.get("position_m") or [0.0, 0.0, 0.0])
        while len(p) < 3:
            p.append(0.0)
        p[0], p[1], p[2] = float(x), float(y), float(z)
        c["position_m"] = p

    _pos(by["hydrogen_cylinder"], x_cyl, 0.0, z_cyl)
    if "stored_hydrogen" in by:
        _pos(by["stored_hydrogen"], x_cyl, 0.0, z_cyl)

    comps_list = cfg["geometry_mass"]["component_ledger"]["components"]
    _set_ledger_box(
        by,
        "fuel_cell_system",
        pos=(x_fc, 0.0, z_fc),
        dims=(fc_L, fc_W, fc_H),
        status=str(fc_hw.get("status", "supplier_confirmed")),
        source=str(fc_hw.get("source", "FCPM datasheet fuel-cell envelope")),
        note=(
            "FCPM fuel-cell section; "
            + (
                "forward face flush with nose-section end."
                if energy_bay_order == "fuel_cell_forward"
                else "aft as possible with pusher-motor keepout."
            )
        ),        mass_kg=fc_mass,
        components=comps_list,
    )
    _set_ledger_box(
        by,
        "fuel_cell_control_box",
        pos=(x_cb, y_cb, z_cb),
        dims=(cb_L, cb_W, cb_H),
        status=str(cb_hw.get("status", "supplier_confirmed")),
        source=str(cb_hw.get("source", "FCPM datasheet control-box envelope")),
        note=f"FCPM control box seat={best['cb_site']} (min-diameter search).",
        mass_kg=cb_mass,
        components=comps_list,
    )
    cfg.setdefault("power_system", {}).setdefault("fuel_cell", {})["module_mass_kg"] = module_mass
    cfg["power_system"]["fuel_cell"]["stack_mass_kg"] = fc_mass
    cfg["power_system"]["fuel_cell"]["control_box_mass_kg"] = cb_mass

    if "buffer_battery" in by:
        _pos(by["buffer_battery"], x_bat, y_bat, z_bat)
        by["buffer_battery"]["include_in_collision_check"] = True
        by["buffer_battery"]["note"] = f"Buffer battery seat={best['bat_site']} (min-diameter search)."
    if "hydrogen_regulator" in by:
        _pos(by["hydrogen_regulator"], x_fc - 0.01, 0.0, z_fc)
    if "payload" in by:
        _pos(by["payload"], x_pay, 0.0, z_pay)
        dims = by["payload"].setdefault("dimensions_m", {})
        dims["length_x"] = pay_L
        dims["width_y"] = pay_W
        dims["height_z"] = pay_H
        by["payload"]["include_in_collision_check"] = True
        by["payload"]["note"] = "Payload in nose; aft face flush with nose-section end; floor clearance."

    pdb_mass = pdb.get("mass_kg")
    for i, p in enumerate(pdbs):
        name = "power_module" if i == 0 else f"power_module_{i + 1}"
        _set_ledger_box(
            by,
            name,
            pos=(p[1], p[2], p[6]),
            dims=(pdb_L, pdb_W, pdb_H),
            status=str(pdb.get("status", "supplier_confirmed")),
            source=str(pdb.get("source", "Holybro PDB 300A datasheet (user packaging_layout)")),
            note="Holybro PDB 300A on H₂ cylinder top (not stacked on CB/battery).",
            mass_kg=float(pdb_mass) if pdb_mass is not None else None,
            components=comps_list,
        )

    cube_mass = cube.get("mass_kg")
    _set_ledger_box(
        by,
        "autopilot",
        pos=(x_cube, 0.0, z_cube),
        dims=(cube_L, cube_W, cube_H),
        status=str(cube.get("status", "supplier_confirmed")),
        source=str(cube.get("source", "CubePilot Cube Orange + carrier (user packaging_layout)")),
        note=(
            f"Cube Orange over payload in nose; aft flush with nose end; "
            f"{1000 * cube_front:.0f} mm forward connector keepout."
        ),
        mass_kg=float(cube_mass) if cube_mass is not None else None,
        components=comps_list,
    )

    summary = {
        "enabled": True,
        "energy_bay_order": energy_bay_order,
        "fuselage_length_m": fuse_len,
        "fuselage_od_m": fuse_od,
        "wall_clearance_m": wall,
        "pusher_motor_length_m": motor_len,
        "aft_keepout_m": aft_keepout,
        "cylinder_top_gap_m": top_gap,
        "cylinder_top_gap_required_m": top_need,
        "bottom_to_top_clearance_m": stack_v,
        "nose_section_end_x_m": nose_len,
        "fuel_cell_module_mass_kg": module_mass,
        "fuel_cell_stack_mass_kg": fc_mass,
        "fuel_cell_control_box_mass_kg": cb_mass,
        "control_box_site": best["cb_site"],
        "battery_site": best["bat_site"],
        "layout_od_need_m": best["od_need"],
        "cube_front_connector_bay_m": [connector_start, cube_fwd],
        "cube_front_connector_available_m": connector_len,
        "cube_front_connector_requested_m": cube_front,
        "placed_envelopes": [
            {
                "name": n,
                "x": float(x),
                "y": float(y),
                "length": float(lx),
                "width": float(wy),
                "height": float(hz),
                "z": float(z),
                "radial_m": _max_yz_radius(float(y), float(z), float(wy), float(hz)),
            }
            for n, x, y, lx, wy, hz, z in placed
        ],
        "stations_m": {
            "hydrogen_cylinder": [x_cyl, 0.0, z_cyl],
            "fuel_cell_system": [x_fc, 0.0, z_fc],
            "fuel_cell_control_box": [x_cb, y_cb, z_cb],
            "buffer_battery": [x_bat, y_bat, z_bat],
            "autopilot_cube_orange": [x_cube, 0.0, z_cube],
            "payload": [x_pay, 0.0, z_pay],
            **{p[0]: [p[1], p[2], p[6]] for p in pdbs},
        },
    }
    cfg.setdefault("coupling", {})["packaging_layout"] = summary
    return summary
