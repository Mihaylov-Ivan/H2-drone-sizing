"""Rule-based internal packaging layout for the central fuselage pod.

Positions the H₂ cylinder, fuel-cell stack (battery / Cube Orange), and Holybro PDB
from explicit clearance and hardware envelopes in ``packaging_layout``. Raises
``ValueError`` when the fixed fuselage cannot satisfy the rules.
"""
from __future__ import annotations

from math import hypot
from typing import Any, Dict, Mapping, MutableMapping, Sequence, Tuple

EPS = 1e-9


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
    placed: Sequence[tuple],
    sections: Sequence[Mapping[str, Any]],
    *,
    wall_m: float,
    margin_m: float = 0.0,
    min_od_m: float = 0.12,
) -> Dict[str, float]:
    """Required outer diameter per fuselage section from component YZ radial extent + wall keepout."""
    ranges = _section_x_ranges(sections)
    needs = {name: float(min_od_m) for name, _a, _b in ranges}
    for _name, x, lx, wy, hz, z in placed:
        r = _max_yz_radius(0.0, float(z), float(wy), float(hz))
        need = 2.0 * (r + float(wall_m) + float(margin_m))
        x0, x1 = float(x) - 0.5 * float(lx), float(x) + 0.5 * float(lx)
        for sec_name, a, b in ranges:
            if x1 <= a + EPS or x0 >= b - EPS:
                continue
            needs[sec_name] = max(needs[sec_name], need)
    # Sections with no overlapping components stay at minimum_diameter_m.
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
    for _ in range(24):
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
                )
            ):
                od = max(od + 0.010, od * 1.04)
                continue
            raise
        placed = summary.get("placed_envelopes") or []
        placed_tuples = [
            (p["name"], p["x"], p["length"], p["width"], p["height"], p["z"])
            for p in placed
        ]
        sections = _fuselage_sections(cfg)
        section_ods = _section_ods_from_placed(
            placed_tuples, sections, wall_m=wall, margin_m=margin, min_od_m=min_od
        )
        need = max(section_ods.values()) if section_ods else od
        if need > od + 5e-5:
            # Radial extent demands a larger shell than the current placement frame.
            od = need
            continue
        # Placement succeeded at `od`. Do not shrink (avoids headroom oscillation);
        # lift the driving section up to the proven OD so max matches the frame.
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


def _packaging_layout_core(cfg: MutableMapping[str, Any]) -> Dict[str, Any]:
    """One packaging placement pass at the current fuselage max OD."""
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

    # Optional packaging envelopes override ledger placeholders (e.g. fuel-cell box).
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

    # ---- H₂ cylinder: as aft as possible, bottom clearance, top bay for PDB ----
    if "hydrogen_cylinder" not in by:
        raise ValueError("Packaging layout requires hydrogen_cylinder in the component ledger")
    L_cyl, D_cyl = _cyl_dims(by["hydrogen_cylinder"])
    aft_keepout = motor_len + aft_beyond_motor
    x_cyl_aft = fuse_len - aft_keepout
    x_cyl = x_cyl_aft - 0.5 * L_cyl
    x_cyl_fwd = x_cyl - 0.5 * L_cyl
    if x_cyl_fwd < wall - EPS:
        raise ValueError(
            f"Packaging layout: H₂ cylinder cannot sit aft with motor+{1000 * aft_beyond_motor:.0f} mm "
            f"tail keepout — cylinder length {1000 * L_cyl:.0f} mm does not fit in "
            f"{1000 * fuse_len:.0f} mm fuselage (forward face would be at x={1000 * x_cyl_fwd:.1f} mm)."
        )
    z_cyl = z_bot + bottom_c + 0.5 * D_cyl
    cyl_top = z_cyl + 0.5 * D_cyl
    top_gap = z_top - cyl_top

    pdb_L = _f(pdb.get("length_m"), 0.080)
    pdb_W = _f(pdb.get("width_m"), 0.060)
    pdb_H = _f(pdb.get("height_m"), 0.0235)
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

    z_pdb = None  # set after control-box placement over the cylinder

    # ---- Fuel cell (forward/bottom) + control box aft over H₂ cylinder ----
    if "fuel_cell_system" not in by and "fuel_cell_stack" not in (hw or {}):
        raise ValueError("Packaging layout requires fuel_cell_system / fuel_cell_stack")
    fuse_half = 0.55 * fuse_od

    fc_hw = hw.get("fuel_cell_stack") or {}
    cb_hw = hw.get("fuel_cell_control_box") or {}
    fc_L = _f(fc_hw.get("length_m"), 0.2305)
    fc_W = _f(fc_hw.get("width_m"), 0.1084)
    fc_H = _f(fc_hw.get("height_m"), 0.1198)
    cb_L = _f(cb_hw.get("length_m"), 0.2022)
    cb_W = _f(cb_hw.get("width_m"), 0.107)
    cb_H = _f(cb_hw.get("height_m"), 0.04515)

    # Module mass (supplier total) split across stack + control box.
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

    # Nose-section aft face (payload aft flush here).
    sections = (
        cfg.get("manufacturing_mass_model", {}).get("fuselage_sections")
        or (cfg.get("user_input_snapshot") or {}).get("fuselage", {}).get("sections")
        or []
    )
    nose_len = 0.0
    for sec in sections:
        if str(sec.get("name", "")).lower() == "nose":
            nose_len = _f(sec.get("length_m"))
            break
    if nose_len <= 0 and sections:
        nose_len = _f(sections[0].get("length_m"))
    if nose_len <= 0:
        raise ValueError("Packaging layout: fuselage nose section length is required")

    x_pay_aft = nose_len
    x_pay_fwd = x_pay_aft - pay_L
    x_pay = x_pay_aft - 0.5 * pay_L
    if x_pay_fwd < wall - EPS:
        raise ValueError(
            f"Packaging layout: payload length {1000 * pay_L:.1f} mm does not fit with its aft face "
            f"flush at the nose-section end x={1000 * nose_len:.1f} mm."
        )

    # Fuel cell as far aft as possible toward the cylinder, while still under the payload.
    x_fc_aft_limit = x_cyl_fwd - comp_c
    x_fc_aft_cover = x_pay_fwd + fc_L
    if x_fc_aft_limit + EPS < x_pay_aft:
        raise ValueError(
            "Packaging layout: H₂ cylinder keepout leaves no room for a fuel cell that reaches "
            "under the payload at the nose-section break. Lengthen the fuselage or shorten envelopes."
        )
    x_fc_aft = min(x_fc_aft_limit, x_fc_aft_cover)
    if x_fc_aft + EPS < x_pay_aft:
        raise ValueError(
            "Packaging layout: fuel cell is too short to sit under the payload with the payload aft "
            f"flush at the nose-section end (need FC length ≥ {1000 * pay_L:.1f} mm)."
        )
    x_fc = x_fc_aft - 0.5 * fc_L
    x_fc_fwd = x_fc - 0.5 * fc_L
    if x_fc_fwd > x_pay_fwd + EPS or x_fc_aft + EPS < x_pay_aft:
        raise ValueError(
            "Packaging layout: fuel cell does not fully underlie the payload at the nose-section station."
        )

    z_fc = z_bot + bottom_c + 0.5 * fc_H
    fc_top = z_fc + 0.5 * fc_H
    headroom = z_top - fc_top

    # Upper deck on the fuel cell (payload / battery / Cube) — control box is no longer here.
    col_payload = stack_v + pay_H + comp_c
    col_bat_cube = stack_v + bat_H + stack_v + cube_H + comp_c
    reserved = max(col_payload, col_bat_cube)
    if headroom + EPS < reserved:
        raise ValueError(
            f"Packaging layout: fuel-cell top bay needs {1000 * reserved:.1f} mm "
            f"(bottom-to-top clearance + payload/battery/Cube columns) but only "
            f"{1000 * headroom:.1f} mm remains above the fuel cell. Increase fuselage diameter "
            f"or reduce stack heights."
        )

    _require_circular_fit("fuel_cell_system", y0=0.0, z0=z_fc, wy=fc_W, hz=fc_H, allowed_r=allowed_r)

    # Control box: aft of fuel cell, over the H₂ cylinder.
    x_cb_min = x_fc_aft + comp_c + 0.5 * cb_L
    x_cb_max = x_cyl_aft - 0.5 * cb_L
    x_cb_pref = x_cyl  # prefer cylinder centre
    if x_cb_max + EPS < x_cb_min:
        raise ValueError(
            "Packaging layout: no longitudinal room for the FCPM control box aft of the fuel cell "
            "and over the H₂ cylinder. Lengthen the fuselage or shorten envelopes."
        )
    x_cb = min(max(x_cb_pref, x_cb_min), x_cb_max)
    x_cb_fwd = x_cb - 0.5 * cb_L
    x_cb_aft = x_cb + 0.5 * cb_L
    if x_cb_fwd + EPS < x_cyl_fwd or x_cb_aft > x_cyl_aft + EPS:
        raise ValueError(
            "Packaging layout: FCPM control box cannot stay fully over the H₂ cylinder while "
            "remaining aft of the fuel cell."
        )

    z_cb = cyl_top + stack_v + 0.5 * cb_H
    cb_top = z_cb + 0.5 * cb_H
    _require_circular_fit("fuel_cell_control_box", y0=0.0, z0=z_cb, wy=cb_W, hz=cb_H, allowed_r=allowed_r)

    # PDB stacked above the control box in the cylinder bay.
    z_pdb = cb_top + pdb_around + 0.5 * pdb_H
    pdb_top = z_pdb + 0.5 * pdb_H
    if pdb_top + pdb_around > z_top + EPS:
        raise ValueError(
            "Packaging layout: PDB above the control box exceeds the fuselage top inner wall. "
            "Increase fuselage diameter or reduce cylinder-bay stack heights."
        )
    # Re-check cylinder top bay includes control box + PDB.
    cyl_bay_need = stack_v + cb_H + pdb_around + pdb_H + pdb_around
    cyl_bay_have = z_top - cyl_top
    if cyl_bay_have + EPS < max(top_need, cyl_bay_need):
        raise ValueError(
            f"Packaging layout: cylinder top bay needs {1000 * max(top_need, cyl_bay_need):.1f} mm "
            f"for control box + PDB clearances, but only {1000 * cyl_bay_have:.1f} mm is available."
        )
    _require_circular_fit("holybro_pdb_300a", y0=0.0, z0=z_pdb, wy=pdb_W, hz=pdb_H, allowed_r=allowed_r)

    z_pay = fc_top + stack_v + 0.5 * pay_H
    _require_circular_fit("payload", y0=0.0, z0=z_pay, wy=pay_W, hz=pay_H, allowed_r=allowed_r)

    # Battery on fuel-cell deck, aft of payload.
    x_bat_fwd_min = x_pay_aft + comp_c
    x_bat_aft_max = x_fc_aft
    if x_bat_aft_max - x_bat_fwd_min + EPS < bat_L:
        raise ValueError(
            "Packaging layout: no room on the fuel cell aft of the payload for the buffer battery "
            f"({1000 * bat_L:.1f} mm) with {1000 * comp_c:.0f} mm clearance."
        )
    x_bat_aft = x_bat_aft_max
    x_bat = x_bat_aft - 0.5 * bat_L
    x_bat_fwd = x_bat - 0.5 * bat_L
    if x_bat_fwd + EPS < x_bat_fwd_min:
        x_bat_fwd = x_bat_fwd_min
        x_bat = x_bat_fwd + 0.5 * bat_L
        x_bat_aft = x_bat + 0.5 * bat_L
    z_bat = fc_top + stack_v + 0.5 * bat_H
    _require_circular_fit("buffer_battery", y0=0.0, z0=z_bat, wy=bat_W, hz=bat_H, allowed_r=allowed_r)

    bat_top = z_bat + 0.5 * bat_H
    z_cube = bat_top + stack_v + 0.5 * cube_H
    _require_circular_fit("cube_orange", y0=0.0, z0=z_cube, wy=cube_W, hz=cube_H, allowed_r=allowed_r)
    if z_cube + 0.5 * cube_H + comp_c > z_top + EPS:
        raise ValueError("Packaging layout: Cube Orange + top clearance exceeds fuselage top inner wall")

    x_cube = x_bat
    cube_fwd = x_cube - 0.5 * cube_L
    connector_start = cube_fwd - cube_front
    if connector_start < wall - EPS:
        x_cube = wall + cube_front + 0.5 * cube_L
        cube_fwd = x_cube - 0.5 * cube_L
        connector_start = cube_fwd - cube_front
    if x_cube + 0.5 * cube_L > x_fc_aft + EPS:
        raise ValueError(
            f"Packaging layout: Cube Orange with {1000 * cube_front:.0f} mm forward connector bay "
            f"does not fit over the fuel-cell length."
        )
    if connector_start + EPS < wall:
        raise ValueError(
            f"Packaging layout: Cube Orange needs {1000 * cube_front:.0f} mm free in front for "
            f"connectors, but the bay would start at x={1000 * connector_start:.1f} mm."
        )
    if connector_start < x_pay_aft - EPS and cube_fwd > x_pay_fwd + EPS:
        if abs(z_cube - z_pay) < 0.5 * (cube_H + pay_H) + stack_v - EPS:
            x_cube_min = x_pay_aft + cube_front + 0.5 * cube_L
            if x_cube_min + 0.5 * cube_L > x_fc_aft + EPS:
                raise ValueError(
                    f"Packaging layout: Cube Orange {1000 * cube_front:.0f} mm connector bay conflicts "
                    f"with payload at the nose-section break."
                )
            x_cube = max(x_cube, x_cube_min)
            cube_fwd = x_cube - 0.5 * cube_L
            connector_start = cube_fwd - cube_front

    # ---- Write design + ledger ----
    d["hydrogen_cylinder_x_m"] = x_cyl
    d["hydrogen_cylinder_z_m"] = z_cyl
    d["fuel_cell_x_m"] = x_fc
    d["fuel_cell_z_m"] = z_fc
    d["fuel_cell_control_box_x_m"] = x_cb
    d["fuel_cell_control_box_z_m"] = z_cb
    d["fuel_cell_stack_mass_kg"] = fc_mass
    d["fuel_cell_control_box_mass_kg"] = cb_mass
    d["battery_x_m"] = x_bat
    d["battery_z_m"] = z_bat
    d["cube_orange_x_m"] = x_cube
    d["cube_orange_z_m"] = z_cube
    d["pdb_x_m"] = x_cyl
    d["pdb_z_m"] = z_pdb
    d["payload_x_m"] = x_pay
    d["payload_z_m"] = z_pay
    d["nose_section_end_x_m"] = nose_len

    def _pos(c: MutableMapping[str, Any], x: float, z: float) -> None:
        p = list(c.get("position_m") or [0.0, 0.0, 0.0])
        while len(p) < 3:
            p.append(0.0)
        p[0], p[1], p[2] = float(x), 0.0, float(z)
        c["position_m"] = p

    _pos(by["hydrogen_cylinder"], x_cyl, z_cyl)
    if "stored_hydrogen" in by:
        _pos(by["stored_hydrogen"], x_cyl, z_cyl)

    comps_list = cfg["geometry_mass"]["component_ledger"]["components"]
    _set_ledger_box(
        by,
        "fuel_cell_system",
        pos=(x_fc, 0.0, z_fc),
        dims=(fc_L, fc_W, fc_H),
        status=str(fc_hw.get("status", "supplier_confirmed")),
        source=str(fc_hw.get("source", "FCPM datasheet fuel-cell envelope")),
        note="FCPM fuel-cell section (bottom of stack).",
        mass_kg=fc_mass,
        components=comps_list,
    )
    _set_ledger_box(
        by,
        "fuel_cell_control_box",
        pos=(x_cb, 0.0, z_cb),
        dims=(cb_L, cb_W, cb_H),
        status=str(cb_hw.get("status", "supplier_confirmed")),
        source=str(cb_hw.get("source", "FCPM datasheet control-box envelope")),
        note="FCPM control box aft of fuel cell, over H₂ cylinder; PDB stacks above it.",
        mass_kg=cb_mass,
        components=comps_list,
    )
    # Keep power-system module mass as the sum for EMS accounting.
    cfg.setdefault("power_system", {}).setdefault("fuel_cell", {})["module_mass_kg"] = module_mass
    cfg["power_system"]["fuel_cell"]["stack_mass_kg"] = fc_mass
    cfg["power_system"]["fuel_cell"]["control_box_mass_kg"] = cb_mass

    if "buffer_battery" in by:
        _pos(by["buffer_battery"], x_bat, z_bat)
        by["buffer_battery"]["include_in_collision_check"] = True
    if "hydrogen_regulator" in by:
        _pos(by["hydrogen_regulator"], x_fc - 0.01, z_fc)
    if "payload" in by:
        _pos(by["payload"], x_pay, z_pay)
        dims = by["payload"].setdefault("dimensions_m", {})
        if dims.get("width_y") is None or _f(dims.get("width_y")) <= 0:
            dims["width_y"] = pay_W
        if dims.get("height_z") is None or _f(dims.get("height_z")) <= 0:
            dims["height_z"] = pay_H
        by["payload"]["include_in_collision_check"] = True

    pdb_mass = pdb.get("mass_kg")
    _set_ledger_box(
        by,
        "power_module",
        pos=(x_cyl, 0.0, z_pdb),
        dims=(pdb_L, pdb_W, pdb_H),
        status=str(pdb.get("status", "supplier_confirmed")),
        source=str(pdb.get("source", "Holybro PDB 300A datasheet (user packaging_layout)")),
        note="Holybro Power Distribution Board 300A above H₂ cylinder with user clearances.",
    )
    if pdb_mass is not None and "power_module" in by:
        by["power_module"]["mass_takeoff_kg"] = float(pdb_mass)
        by["power_module"]["mass_end_kg"] = float(pdb_mass)

    cube_mass = cube.get("mass_kg")
    _set_ledger_box(
        by,
        "autopilot",
        pos=(x_cube, 0.0, z_cube),
        dims=(cube_L, cube_W, cube_H),
        status=str(cube.get("status", "supplier_confirmed")),
        source=str(cube.get("source", "CubePilot Cube Orange + carrier (user packaging_layout)")),
        note=(
            f"Cube Orange over fuel-cell/battery stack; "
            f"{1000 * cube_front:.0f} mm forward connector keepout."
        ),
    )
    if cube_mass is not None and "autopilot" in by:
        by["autopilot"]["mass_takeoff_kg"] = float(cube_mass)
        by["autopilot"]["mass_end_kg"] = float(cube_mass)

    placed = [
        ("fuel_cell_system", x_fc, fc_L, fc_W, fc_H, z_fc),
        ("fuel_cell_control_box", x_cb, cb_L, cb_W, cb_H, z_cb),
        ("buffer_battery", x_bat, bat_L, bat_W, bat_H, z_bat),
        ("autopilot", x_cube, cube_L, cube_W, cube_H, z_cube),
        ("hydrogen_cylinder", x_cyl, L_cyl, D_cyl, D_cyl, z_cyl),
        ("power_module", x_cyl, pdb_L, pdb_W, pdb_H, z_pdb),
        ("payload", x_pay, pay_L, pay_W, pay_H, z_pay),
    ]
    bay = (connector_start, cube_fwd)
    for name, x, lx, wy, hz, z in placed:
        if name == "autopilot":
            continue
        lo, hi = x - 0.5 * lx, x + 0.5 * lx
        if hi > bay[0] + EPS and lo < bay[1] - EPS:
            if abs(z - z_cube) < 0.5 * (hz + cube_H) - EPS:
                raise ValueError(
                    f"Packaging layout: {name} intrudes into the Cube Orange "
                    f"{1000 * cube_front:.0f} mm forward connector bay."
                )

    for i in range(len(placed)):
        n1, x1, l1, w1, h1, z1 = placed[i]
        for j in range(i + 1, len(placed)):
            n2, x2, l2, w2, h2, z2 = placed[j]
            ox = min(x1 + 0.5 * l1, x2 + 0.5 * l2) - max(x1 - 0.5 * l1, x2 - 0.5 * l2)
            oy = min(0.5 * w1, 0.5 * w2) - max(-0.5 * w1, -0.5 * w2)
            oz = min(z1 + 0.5 * h1, z2 + 0.5 * h2) - max(z1 - 0.5 * h1, z2 - 0.5 * h2)
            if ox > -comp_c + EPS and oy > -comp_c + EPS and oz > -comp_c + EPS:
                raise ValueError(
                    f"Packaging layout: {n1} collides with {n2} "
                    f"(overlaps/gaps m: {ox:+.4f}, {oy:+.4f}, {oz:+.4f}; "
                    f"need {1000 * comp_c:.0f} mm component clearance)."
                )

    summary = {
        "enabled": True,
        "fuselage_length_m": fuse_len,
        "fuselage_od_m": fuse_od,
        "wall_clearance_m": wall,
        "pusher_motor_length_m": motor_len,
        "aft_keepout_m": aft_keepout,
        "cylinder_top_gap_m": top_gap,
        "cylinder_top_gap_required_m": top_need,
        "fuel_cell_headroom_m": headroom,
        "fuel_cell_headroom_required_m": reserved,
        "bottom_to_top_clearance_m": stack_v,
        "nose_section_end_x_m": nose_len,
        "fuel_cell_module_mass_kg": module_mass,
        "fuel_cell_stack_mass_kg": fc_mass,
        "fuel_cell_control_box_mass_kg": cb_mass,
        "cube_front_connector_bay_m": [connector_start, cube_fwd],
        "placed_envelopes": [
            {
                "name": n,
                "x": float(x),
                "length": float(lx),
                "width": float(wy),
                "height": float(hz),
                "z": float(z),
                "radial_m": _max_yz_radius(0.0, float(z), float(wy), float(hz)),
            }
            for n, x, lx, wy, hz, z in placed
        ],
        "stations_m": {
            "hydrogen_cylinder": [x_cyl, 0.0, z_cyl],
            "fuel_cell_system": [x_fc, 0.0, z_fc],
            "fuel_cell_control_box": [x_cb, 0.0, z_cb],
            "buffer_battery": [x_bat, 0.0, z_bat],
            "autopilot_cube_orange": [x_cube, 0.0, z_cube],
            "power_module_pdb": [x_cyl, 0.0, z_pdb],
            "payload": [x_pay, 0.0, z_pay],
        },
    }
    cfg.setdefault("coupling", {})["packaging_layout"] = summary
    return summary
