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


def _longeron_cabin_band(
    cfg: Mapping[str, Any],
    *,
    fuse_od_m: float,
    boss_clearance_m: float,
) -> Dict[str, float]:
    """Cabin Z band between bottom/top longeron bosses for the current fuselage OD."""
    R = 0.5 * float(fuse_od_m)
    longr = (cfg.get("manufacturing_mass_model") or {}).get("fuselage_longerons") or {}
    long_od = _f(longr.get("tube_od_m"), 0.0065)
    boss_od = _f(longr.get("boss_od_m"), max(2.0 * long_od + 0.004, long_od + 0.006))
    long_r = 0.5 * long_od
    boss_r = 0.5 * boss_od
    r_axis = max(R - long_r, 0.5 * R)
    z_long_top = r_axis
    z_long_bot = -r_axis
    clear = max(0.0, float(boss_clearance_m))
    # Component faces must stay clear of the boss extremities toward the cabin.
    z_floor = z_long_bot + boss_r + clear  # min component bottom face
    z_ceil = z_long_top - boss_r - clear  # max component top face
    if z_ceil <= z_floor + EPS:
        raise ValueError(
            "Packaging layout: longeron boss clearances leave no cabin height at the current "
            f"fuselage OD ({1000 * fuse_od_m:.0f} mm). Increase diameter or reduce boss/clearance."
        )
    return {
        "long_od_m": long_od,
        "boss_od_m": boss_od,
        "z_longeron_top_m": z_long_top,
        "z_longeron_bottom_m": z_long_bot,
        "z_floor_m": z_floor,
        "z_ceil_m": z_ceil,
        "boss_clearance_m": clear,
    }


def _lift_placed(placed: Sequence[Placed], groups: Mapping[str, str], lifts: Mapping[str, float]) -> List[Placed]:
    out: List[Placed] = []
    for n, x, y, lx, wy, hz, z in placed:
        g = groups.get(n, "")
        dz = float(lifts.get(g, 0.0)) if g else 0.0
        out.append((n, x, y, lx, wy, hz, z + dz))
    return out


def _pack_z_valid(
    placed: Sequence[Placed],
    *,
    z_floor: float,
    z_ceil: float,
    allowed_r: float,
    wall_m: float,
    comp_c: float,
) -> bool:
    del wall_m
    for n, _x, y, _lx, wy, hz, z in placed:
        if (z - 0.5 * hz) < z_floor - EPS:
            return False
        if (z + 0.5 * hz) > z_ceil + EPS:
            return False
        if _max_yz_radius(y, z, wy, hz) > allowed_r + EPS:
            return False
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            if _boxes_collide(placed[i], placed[j], clearance=comp_c):
                return False
    return True


def _optimize_pack_z(
    placed: Sequence[Placed],
    groups: Mapping[str, str],
    *,
    z_floor: float,
    z_ceil: float,
    allowed_r: float,
    wall_m: float,
    fuse_od: float,
    comp_c: float,
) -> Tuple[List[Placed], Dict[str, float], float]:
    """Raise independent Z groups to minimise required fuselage OD (fit current OD if possible)."""
    group_names = sorted({g for g in groups.values() if g})
    lifts = {g: 0.0 for g in group_names}

    def _max_lift_for(g: str, cur: Mapping[str, float]) -> float:
        trial = dict(cur)
        trial[g] = 0.0
        base = _lift_placed(placed, groups, trial)
        cap = float("inf")
        for n, _x, _y, _lx, _wy, hz, z in base:
            if groups.get(n) != g:
                continue
            cap = min(cap, z_ceil - (z + 0.5 * hz))
        if cap == float("inf") or cap < 0:
            return 0.0
        return float(cap)

    def _score(lifted: Sequence[Placed]) -> float:
        return _od_from_placed(lifted, wall_m=wall_m)

    # Prefer layouts that fit the current shell; among those (or all), minimise OD need.
    def _key(lifted: Sequence[Placed]) -> Tuple[float, float, float]:
        need = _score(lifted)
        fit_pen = 0.0 if need <= fuse_od + 5e-5 else 1.0
        lift_sum = sum(lifts.values())
        return (fit_pen, need, lift_sum)

    best_lifts = dict(lifts)
    best_placed = _lift_placed(placed, groups, best_lifts)
    if not _pack_z_valid(
        best_placed, z_floor=z_floor, z_ceil=z_ceil, allowed_r=allowed_r, wall_m=wall_m, comp_c=comp_c
    ):
        # Still search lifts; floor placement may already violate circular fit at this OD.
        best_key = (1.0, 1e9, 0.0)
    else:
        best_key = _key(best_placed)

    for _pass in range(4):
        improved = False
        for g in group_names:
            max_dz = _max_lift_for(g, lifts)
            if max_dz <= EPS:
                continue
            local_best = lifts[g]
            local_key = best_key
            steps = 24
            for i in range(steps + 1):
                dz = max_dz * (i / steps)
                lifts[g] = dz
                lifted = _lift_placed(placed, groups, lifts)
                if not _pack_z_valid(
                    lifted, z_floor=z_floor, z_ceil=z_ceil, allowed_r=allowed_r, wall_m=wall_m, comp_c=comp_c
                ):
                    continue
                k = _key(lifted)
                if k < local_key:
                    local_key = k
                    local_best = dz
                    improved = True
            lifts[g] = local_best
            best_key = local_key
            best_lifts = dict(lifts)
            best_placed = _lift_placed(placed, groups, best_lifts)
        if not improved:
            break

    if not _pack_z_valid(
        best_placed, z_floor=z_floor, z_ceil=z_ceil, allowed_r=allowed_r, wall_m=wall_m, comp_c=comp_c
    ):
        raise ValueError("Z-lift search could not fit packaging in the cabin/longeron band")
    return best_placed, best_lifts, _score(best_placed)


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


def _component_axial_m(clear: Mapping[str, Any]) -> float:
    """Longitudinal clearance between non-nose components (legacy key accepted)."""
    if "component_axial_m" in clear:
        return _f(clear.get("component_axial_m"))
    if "energy_bay_axial_m" in clear:
        return _f(clear.get("energy_bay_axial_m"))
    return _f(clear.get("component_m"), 0.003)


def _fwd_aft(p: Placed) -> Tuple[float, float]:
    return float(p[1]) - 0.5 * float(p[3]), float(p[1]) + 0.5 * float(p[3])


def _yz_overlap(a: Placed, b: Placed) -> bool:
    _n1, _x1, y1, _l1, w1, h1, z1 = a
    _n2, _x2, y2, _l2, w2, h2, z2 = b
    oy = min(y1 + 0.5 * w1, y2 + 0.5 * w2) - max(y1 - 0.5 * w1, y2 - 0.5 * w2)
    oz = min(z1 + 0.5 * h1, z2 + 0.5 * h2) - max(z1 - 0.5 * h1, z2 - 0.5 * h2)
    return oy > EPS and oz > EPS


_NOSE_NAMES = frozenset({"payload", "autopilot"})


def _axial_gap_violations(
    placed: Sequence[Placed],
    *,
    axial: float,
    exempt: Mapping[str, Any] | None = None,
) -> List[str]:
    """Pairs with YZ overlap must keep at least ``axial`` clear in X (nose exempt)."""
    skip = _NOSE_NAMES if exempt is None else frozenset(exempt)
    bad: List[str] = []
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            a, b = placed[i], placed[j]
            if a[0] in skip or b[0] in skip:
                continue
            if not _yz_overlap(a, b):
                continue
            a0, a1 = _fwd_aft(a)
            b0, b1 = _fwd_aft(b)
            gap = max(b0 - a1, a0 - b1)
            if gap + EPS < float(axial):
                bad.append(f"{a[0]}/{b[0]} axial gap {1000 * gap:.1f} mm < {1000 * axial:.0f} mm")
    return bad


def _pack_top_forward_x(
    *,
    name: str,
    L: float,
    W: float,
    H: float,
    y: float,
    z: float,
    host_fwd: float,
    host_aft: float,
    blockers: Sequence[Placed],
    axial: float,
    allow_forward_of_host: bool,
) -> Placed:
    """Seat a top-bay box as far forward as axial clearance / host support allow.

    Nose stays fixed separately. Aft-host tops may overhang forward of the host
    front face into the inter-bay gap, while keeping positive X-overlap on the host.
    """
    if L > (host_aft - host_fwd) + EPS and not allow_forward_of_host:
        raise ValueError(f"{name} longer than its host bay")

    # Lower bound on forward face from YZ-overlapping blockers.
    x_fwd_lo = -1e9
    probe = (name, 0.0, y, L, W, H, z)
    for b in blockers:
        if b[0] in _NOSE_NAMES:
            continue
        if not _yz_overlap(probe, b):
            continue
        _bf, ba = _fwd_aft(b)
        x_fwd_lo = max(x_fwd_lo, ba + float(axial))

    if allow_forward_of_host:
        # Keep positive overlap with the host X-span (may overhang the front face).
        x_fwd_min_support = host_fwd - L + 1e-6
        x_fwd = max(x_fwd_lo, x_fwd_min_support)
    else:
        x_fwd = max(x_fwd_lo, host_fwd)

    x_aft = x_fwd + L
    if x_aft > host_aft + EPS:
        # Push aft to fit on host; re-check blockers.
        x_fwd = host_aft - L
        x_aft = host_aft
        if x_fwd + EPS < (x_fwd_lo if allow_forward_of_host else max(x_fwd_lo, host_fwd)):
            raise ValueError(f"{name} cannot satisfy component_axial_m and host bay length")
        probe = (name, x_fwd + 0.5 * L, y, L, W, H, z)
        for b in blockers:
            if b[0] in _NOSE_NAMES:
                continue
            if not _yz_overlap(probe, b):
                continue
            _bf, ba = _fwd_aft(b)
            if x_fwd + EPS < ba + float(axial):
                raise ValueError(f"{name} cannot clear {b[0]} with component_axial_m on host")

    # Final support check for overhanging seats.
    if allow_forward_of_host and x_aft <= host_fwd + EPS:
        raise ValueError(f"{name} would leave its host bay entirely")
    if (not allow_forward_of_host) and x_fwd + EPS < host_fwd:
        raise ValueError(f"{name} cannot sit forward of its host front face")

    return (name, x_fwd + 0.5 * L, y, L, W, H, z)


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
    section_ods: Dict[str, float] = {}
    for sec in _fuselage_sections(cfg):
        sec["diameter_m"] = od
        name = str(sec.get("name", ""))
        if name:
            section_ods[name] = od
    d["fuselage_section_diameters_m"] = dict(section_ods)
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


def _commit_fuselage_length(cfg: MutableMapping[str, Any], fuse_len: float) -> None:
    """Update design + envelopes + lift bounds for a new fuselage total length."""
    fuse_len = float(fuse_len)
    d = cfg["design"]
    od = _f(d.get("fuselage_max_diameter_m"))
    d["fuselage_length_m"] = fuse_len
    geom = cfg["geometry_mass"]["geometry_model"]["geometry"]
    geom["central_pod_outer"] = {
        "center_m": [fuse_len / 2.0, 0.0, 0.0],
        "dimensions_m": [fuse_len, od, od],
    }
    env = cfg["geometry_mass"]["geometry_model"]["packaging"]["envelopes"]["central_pod"]
    env["center_m"] = [fuse_len / 2.0, 0.0, 0.0]
    env["dimensions_m"] = [fuse_len, od, od]
    lay = cfg["aero_propulsion"]["lift_layout"]
    lay["maximum_rear_x_m"] = fuse_len
    mm = cfg.get("manufacturing_mass_model")
    if isinstance(mm, dict):
        mm["fuselage_total_length_m"] = fuse_len
    derived = cfg.get("derived_geometry_v3")
    if isinstance(derived, dict):
        derived["fuselage_total_length_m"] = fuse_len


def _commit_section_lengths(cfg: MutableMapping[str, Any], lengths: Mapping[str, float]) -> None:
    """Write per-section lengths into manufacturing + user snapshot records."""
    for sec in _fuselage_sections(cfg):
        name = str(sec.get("name", ""))
        if name in lengths:
            sec["length_m"] = float(lengths[name])
    snap = cfg.get("user_input_snapshot")
    if isinstance(snap, dict) and isinstance(snap.get("fuselage"), dict):
        for sec in snap["fuselage"].get("sections") or []:
            name = str(sec.get("name", ""))
            if name in lengths:
                sec["length_m"] = float(lengths[name])
    total = sum(_f(s.get("length_m")) for s in _fuselage_sections(cfg))
    _commit_fuselage_length(cfg, total)
    cfg["design"]["fuselage_section_lengths_m"] = {
        str(s.get("name", f"s{i}")): _f(s.get("length_m"))
        for i, s in enumerate(_fuselage_sections(cfg))
    }


def _autosize_midbody_lengths(cfg: MutableMapping[str, Any]) -> Dict[str, Any]:
    """Size center_a/center_b so FC–H₂ facing gap equals component_axial_m.

    Nose and tail section lengths stay user-fixed. Returns a small summary dict
    (empty when midbody_length_mode is not auto).
    """
    fus_snap = (cfg.get("user_input_snapshot") or {}).get("fuselage") or {}
    mode = str(
        fus_snap.get("midbody_length_mode")
        or cfg["design"].get("fuselage_midbody_length_mode")
        or "hardcoded"
    ).lower()
    if mode != "auto":
        return {}

    layout = cfg.get("packaging_layout") or {}
    clear = layout.get("clearances_m") or {}
    hw = layout.get("hardware") or {}
    axial = _component_axial_m(clear)
    if axial < 0:
        raise ValueError("Packaging layout: component_axial_m must be >= 0")

    sections = _fuselage_sections(cfg)
    by_name = {str(s.get("name", "")).lower(): s for s in sections}
    if "nose" not in by_name or "tail" not in by_name or "center_a" not in by_name or "center_b" not in by_name:
        raise ValueError(
            "Packaging layout: midbody_length_mode auto requires fuselage sections "
            "named nose, center_a, center_b, and tail."
        )
    nose_len = _f(by_name["nose"].get("length_m"))
    tail_len = _f(by_name["tail"].get("length_m"))
    if nose_len <= 0 or tail_len <= 0:
        raise ValueError("Packaging layout: nose and tail section lengths must be > 0")

    fc_hw = hw.get("fuel_cell_stack") or {}
    fc_L = _f(fc_hw.get("length_m"), 0.2305)
    comps = {c["name"]: c for c in cfg["geometry_mass"]["component_ledger"]["components"]}
    if "hydrogen_cylinder" not in comps:
        raise ValueError("Packaging layout: hydrogen_cylinder required for midbody auto-length")
    L_cyl, _D = _cyl_dims(comps["hydrogen_cylinder"])

    d = cfg["design"]
    order = str(d.get("energy_bay_order") or layout.get("energy_bay_order") or "fuel_cell_forward").lower()
    if "energy_bay_order_code" in d:
        order = "hydrogen_forward" if _f(d.get("energy_bay_order_code")) >= 0.5 else "fuel_cell_forward"
    if order == "hydrogen_forward":
        L_fwd, L_aft = L_cyl, fc_L
    else:
        L_fwd, L_aft = fc_L, L_cyl

    aft_beyond = _f(clear.get("aft_tail_beyond_motor_m"), 0.020)
    push = (cfg.get("hardware_selection_result") or {}).get("selected_pusher_combo") or {}
    motor_len = _f(push.get("motor_length_m"), 0.034)
    if motor_len <= 0:
        motor_len = 0.034
    aft_keepout = motor_len + aft_beyond

    # Total length so facing gap == axial clearance with forward flush at nose end
    # and aft item hard against the pusher keepout.
    fuse_len = nose_len + L_fwd + axial + L_aft + aft_keepout
    mid = fuse_len - nose_len - tail_len
    # Split A/B at mid-gap between the two energy items.
    center_a = L_fwd + 0.5 * axial
    center_b = mid - center_a
    if center_a <= EPS or center_b <= EPS:
        raise ValueError(
            f"Packaging layout: midbody auto-length produced non-positive center sections "
            f"(center_a={1000 * center_a:.1f} mm, center_b={1000 * center_b:.1f} mm). "
            f"Check nose/tail lengths vs energy envelopes and component_axial_m."
        )

    _commit_section_lengths(
        cfg,
        {
            "nose": nose_len,
            "center_a": center_a,
            "center_b": center_b,
            "tail": tail_len,
        },
    )
    d["fuselage_midbody_length_mode"] = "auto"
    d["component_axial_clearance_m"] = axial
    d["energy_bay_axial_clearance_m"] = axial  # legacy alias
    summary = {
        "midbody_length_mode": "auto",
        "component_axial_clearance_m": axial,
        "energy_bay_axial_clearance_m": axial,
        "fuselage_length_m": fuse_len,
        "center_a_length_m": center_a,
        "center_b_length_m": center_b,
        "forward_energy_length_m": L_fwd,
        "aft_energy_length_m": L_aft,
        "aft_keepout_m": aft_keepout,
        "energy_bay_order": order,
    }
    cfg.setdefault("coupling", {})["fuselage_midbody"] = summary
    return summary


def apply_packaging_layout(cfg: MutableMapping[str, Any]) -> Dict[str, Any]:
    """Resolve internal stations from ``packaging_layout`` rules; raise on violation.

    When ``fuselage.diameter_mode`` is ``auto``, fuselage outer diameter is iterated
    from the maximum packed-component radial extent + wall keepout (+ optional margin)
    and applied uniformly to nose, center, and tail sections.
    When ``fuselage.midbody_length_mode`` is ``auto``, center_a/center_b lengths are
    sized so the FC–H₂ facing gap equals ``component_axial_m``.
    """
    layout = cfg.get("packaging_layout") or {}
    if not bool(layout.get("enabled", False)):
        return {"enabled": False}

    midbody_info = _autosize_midbody_lengths(cfg)

    fus_snap = (cfg.get("user_input_snapshot") or {}).get("fuselage") or {}
    auto = str(fus_snap.get("diameter_mode", "hardcoded")).lower() == "auto"
    if not auto:
        summary = _packaging_layout_core(cfg)
        if midbody_info:
            summary["fuselage_midbody"] = midbody_info
            cfg.setdefault("coupling", {})["packaging_layout"] = summary
        return summary

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
        section_needs = _section_ods_from_placed(
            placed_tuples, sections, wall_m=wall, margin_m=margin, min_od_m=min_od
        )
        need = max(section_needs.values()) if section_needs else od
        if need > od + 5e-5:
            od = need
            continue
        # Keep one constant diameter for every fuselage section.
        od = max(float(od), float(need))
        _apply_uniform_fuse_od(cfg, od)
        summary = _packaging_layout_core(cfg)
        section_ods = dict(cfg["design"].get("fuselage_section_diameters_m") or {})
        summary["fuselage_diameter_mode"] = "auto"
        summary["fuselage_section_ods_m"] = section_ods
        summary["fuselage_section_od_needs_m"] = dict(section_needs)
        summary["fuselage_od_m"] = float(cfg["design"]["fuselage_max_diameter_m"])
        if midbody_info:
            summary["fuselage_midbody"] = midbody_info
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
    x_override: float | None = None,
) -> Placed:
    """Place a box on/ beside the fuel cell. ``site`` is fc_top or fc_side.

    On ``fc_top``, default X is host-front flush; callers may pass ``x_override``.
    """
    fc_top = z_fc + 0.5 * fc_H
    x_fc_fwd = x_fc - 0.5 * fc_L
    if site == "fc_top":
        y = 0.0 if abs(side_sign) < EPS else side_sign * (0.5 * W + 0.5 * comp_c)
        x = float(x_override) if x_override is not None else (x_fc_fwd + 0.5 * L)
        return (name, x, y, L, W, H, fc_top + stack_v + 0.5 * H)
    if site == "fc_side":
        y = side_sign * (0.5 * fc_W + comp_c + 0.5 * W)
        x = float(x_override) if x_override is not None else x_fc
        return (name, x, y, L, W, H, z_fc)
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
    x_override: float | None = None,
) -> Placed:
    """Default: front face flush with H₂ forward face; ``x_override`` sets centre X."""
    x = float(x_override) if x_override is not None else (x_cyl_fwd + 0.5 * L)
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
    axial: float,
    occupants: Sequence[Placed],
) -> List[Placed]:
    """Two PDBs on the cylinder top shelf — never stacked on CB/battery.

    Packed as far forward as ``component_axial_m`` against occupants allows.
    """
    z = cyl_top + pdb_around + 0.5 * pdb_H
    gap = max(comp_c, 0.5 * pdb_around, 0.5 * axial)

    def _ok(cands: Sequence[Placed]) -> bool:
        all_boxes = list(occupants) + list(cands)
        for i in range(len(all_boxes)):
            for j in range(i + 1, len(all_boxes)):
                if _boxes_collide(all_boxes[i], all_boxes[j], clearance=comp_c):
                    return False
        if _axial_gap_violations(all_boxes, axial=axial):
            return False
        for c in cands:
            lo, hi = _fwd_aft(c)
            if lo < x_cyl_fwd - EPS or hi > x_cyl_aft + EPS:
                return False
        return True

    # Prefer side-by-side, shifted as far forward as axial clearance allows.
    y_off = 0.5 * pdb_W + 0.5 * gap
    aft_of = x_cyl_fwd
    for occ in occupants:
        if _yz_overlap(
            ("power_module", x_cyl, y_off, pdb_L, pdb_W, pdb_H, z),
            occ,
        ) or _yz_overlap(
            ("power_module", x_cyl, -y_off, pdb_L, pdb_W, pdb_H, z),
            occ,
        ):
            aft_of = max(aft_of, _fwd_aft(occ)[1] + axial)
    # Also clear each other when tandem; for side-by-side same x is fine (no Y overlap issue - they don't overlap Y).
    x_side = aft_of + 0.5 * pdb_L
    # Keep on cylinder.
    x_side = min(x_side, x_cyl_aft - 0.5 * pdb_L)
    x_side = max(x_side, x_cyl_fwd + 0.5 * pdb_L)
    side_by_side = [
        ("power_module", x_side, +y_off, pdb_L, pdb_W, pdb_H, z),
        ("power_module_2", x_side, -y_off, pdb_L, pdb_W, pdb_H, z),
    ]
    if _ok(side_by_side):
        return side_by_side

    # Fallback: try host-centre side-by-side (legacy).
    side_by_side_c = [
        ("power_module", x_cyl, +y_off, pdb_L, pdb_W, pdb_H, z),
        ("power_module_2", x_cyl, -y_off, pdb_L, pdb_W, pdb_H, z),
    ]
    if _ok(side_by_side_c):
        return side_by_side_c

    # Aft of the forward-most occupant on the cylinder.
    aft_of = x_cyl_fwd
    for _n, x, _y, lx, _w, _h, _z in occupants:
        aft_of = max(aft_of, x + 0.5 * lx)
    x_pdb = aft_of + axial + 0.5 * pdb_L
    aft_pair = [
        ("power_module", x_pdb, +y_off, pdb_L, pdb_W, pdb_H, z),
        ("power_module_2", x_pdb, -y_off, pdb_L, pdb_W, pdb_H, z),
    ]
    if _ok(aft_pair):
        return aft_pair

    # Single-file aft on centreline if sides are blocked.
    x1 = aft_of + axial + 0.5 * pdb_L
    x2 = x1 + 0.5 * pdb_L + axial + 0.5 * pdb_L
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
    boss_clear = _f(
        d.get("longeron_boss_clearance_m"),
        _f(clear.get("longeron_boss_clearance_m"), 0.010),
    )
    aft_beyond_motor = _f(clear.get("aft_tail_beyond_motor_m"), 0.020)
    pdb_around = _f(clear.get("pdb_all_around_m"), 0.020)
    comp_c = _f(clear.get("component_m"), 0.003)
    axial_c = _component_axial_m(clear)
    if axial_c < 0:
        raise ValueError("Packaging layout: component_axial_m must be >= 0")
    stack_v = _f(clear.get("bottom_to_top_m"), comp_c)
    cube_front = _f(clear.get("cube_front_connector_m"), 0.050)
    nose_raise = _f(clear.get("nose_bay_raise_m"), 0.0)
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
    cabin = _longeron_cabin_band(cfg, fuse_od_m=fuse_od, boss_clearance_m=boss_clear)
    # Floor sits above the bottom longeron boss (+ optional legacy skin bottom clear).
    z_floor = max(float(cabin["z_floor_m"]), z_bot + bottom_c)
    z_ceil = float(cabin["z_ceil_m"])

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

    z_cyl = z_floor + 0.5 * D_cyl
    cyl_top = z_cyl + 0.5 * D_cyl
    top_gap = z_ceil - cyl_top

    pdb_L = _f(pdb.get("length_m"), 0.080)
    pdb_W = _f(pdb.get("width_m"), 0.060)
    pdb_H = _f(pdb.get("height_m"), 0.0235)
    pdb_count = max(1, int(_f(pdb.get("count"), 2)))
    pdb_need = pdb_H + 2.0 * pdb_around
    top_need = max(min_cyl_top, pdb_need)
    if top_gap + EPS < top_need:
        raise ValueError(
            f"Packaging layout: need {1000 * top_need:.1f} mm between H₂ cylinder top and top "
            f"longeron-boss cabin limit (max of cylinder_to_top_inner={1000 * min_cyl_top:.1f} mm and "
            f"PDB {1000 * pdb_H:.1f} mm + 2×{1000 * pdb_around:.0f} mm clearances), "
            f"but only {1000 * top_gap:.1f} mm is available at OD={1000 * fuse_od:.0f} mm with "
            f"bottom boss clearance {1000 * boss_clear:.0f} mm. Increase fuselage diameter or reduce envelopes."
        )

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

    # ---- Nose bay: payload + Cube, aft faces 1 cm forward of nose-section end ----
    nose_aft_ref = nose_len - 0.01
    if nose_aft_ref <= wall + EPS:
        raise ValueError(
            "Packaging layout: nose section is too short to place payload/Cube 10 mm forward "
            f"of the nose-section end (nose length {1000 * nose_len:.1f} mm)."
        )
    x_pay_aft = nose_aft_ref
    x_pay = x_pay_aft - 0.5 * pay_L
    x_pay_fwd = x_pay - 0.5 * pay_L
    if x_pay_fwd < wall - EPS:
        raise ValueError(
            f"Packaging layout: payload length {1000 * pay_L:.1f} mm does not fit with its aft face "
            f"10 mm forward of the nose-section end (aft at x={1000 * nose_aft_ref:.1f} mm)."
        )
    z_pay = z_floor + nose_raise + 0.5 * pay_H
    pay_top = z_pay + 0.5 * pay_H

    x_cube_aft = nose_aft_ref
    x_cube = x_cube_aft - 0.5 * cube_L
    cube_fwd = x_cube - 0.5 * cube_L
    if cube_fwd < wall - EPS:
        raise ValueError(
            f"Packaging layout: Cube Orange length {1000 * cube_L:.1f} mm does not fit in the nose "
            f"with aft face 10 mm forward of the nose end (aft at x={1000 * nose_aft_ref:.1f} mm)."
        )
    # Prefer the requested connector bay; truncate to the free nose length if needed.
    connector_start = max(wall, cube_fwd - cube_front)
    connector_len = cube_fwd - connector_start
    z_cube = pay_top + stack_v + 0.5 * cube_H
    if z_cube + 0.5 * cube_H > z_ceil + EPS:
        raise ValueError(
            "Packaging layout: Cube Orange over payload exceeds the top longeron-boss cabin limit. "
            "Increase fuselage diameter, reduce envelopes, or lower nose_bay_raise_m."
        )

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
        if x_fc_aft + axial_c > x_cyl_fwd + EPS:
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
        if x_cyl_aft + axial_c > x_fc_fwd + EPS:
            raise ValueError(
                "Packaging layout: H₂ cylinder (front flush with nose end) collides with the fuel cell "
                "forward keepout. Lengthen the fuselage or shorten envelopes."
            )

    z_fc = z_floor + 0.5 * fc_H
    fc_top = z_fc + 0.5 * fc_H
    if fc_top + stack_v > z_ceil + EPS:
        raise ValueError(
            "Packaging layout: fuel cell height exceeds the top longeron-boss cabin limit with "
            "bottom boss clearance. Increase fuselage diameter."
        )

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

            # Forward energy host tops stay host-front flush; aft-host tops may
            # overhang forward into the bay gap, packed against component_axial_m.
            fwd_host_is_fc = energy_bay_order == "fuel_cell_forward"
            fc_is_aft_host = not fwd_host_is_fc
            h2_is_aft_host = fwd_host_is_fc

            # Build seat specs, place forward-host top first, then aft-host top.
            seats: List[Dict[str, Any]] = []
            seats.append(
                {
                    "name": "fuel_cell_control_box",
                    "site": cb_site,
                    "L": cb_L,
                    "W": cb_W,
                    "H": cb_H,
                    "sign": cb_sign,
                    "on_fc": cb_site.startswith("fc"),
                    "is_top": cb_site.endswith("top") or cb_site == "h2_top",
                }
            )
            seats.append(
                {
                    "name": "buffer_battery",
                    "site": bat_site,
                    "L": bat_L,
                    "W": bat_W,
                    "H": bat_H,
                    "sign": bat_sign,
                    "on_fc": bat_site.startswith("fc"),
                    "is_top": bat_site.endswith("top") or bat_site == "h2_top",
                }
            )

            def _host_rank(s: Dict[str, Any]) -> Tuple[int, int]:
                # Forward host first; tops before sides on same host.
                on_fwd = (s["on_fc"] and fwd_host_is_fc) or ((not s["on_fc"]) and (not fwd_host_is_fc))
                return (0 if on_fwd else 1, 0 if s["is_top"] else 1)

            seats.sort(key=_host_rank)

            placed_seats: Dict[str, Placed] = {}
            blockers: List[Placed] = []

            for s in seats:
                site = str(s["site"])
                if site == "h2_top":
                    if float(s["L"]) > L_cyl + EPS:
                        raise ValueError(f"{s['name']} longer than H₂ cylinder")
                    y = 0.0
                    if s["name"] == "buffer_battery" and cb_site == "h2_top":
                        y = 0.5 * cb_W + comp_c + 0.5 * bat_W
                    elif s["name"] == "fuel_cell_control_box" and bat_site == "h2_top" and "buffer_battery" in placed_seats:
                        y = -(0.5 * bat_W + comp_c + 0.5 * cb_W)
                    z = cyl_top + stack_v + 0.5 * float(s["H"])
                    p = _pack_top_forward_x(
                        name=str(s["name"]),
                        L=float(s["L"]),
                        W=float(s["W"]),
                        H=float(s["H"]),
                        y=y,
                        z=z,
                        host_fwd=x_cyl_fwd,
                        host_aft=x_cyl_aft,
                        blockers=blockers,
                        axial=axial_c,
                        allow_forward_of_host=h2_is_aft_host,
                    )
                    if p[1] + 0.5 * float(s["L"]) > x_cyl_aft + EPS:
                        raise ValueError(f"{s['name']} overhangs H₂ cylinder aft")
                elif site == "fc_top":
                    if float(s["L"]) > fc_L + EPS:
                        raise ValueError(f"{s['name']} longer than fuel cell for top seat")
                    y = 0.0 if abs(float(s["sign"])) < EPS else float(s["sign"]) * (
                        0.5 * float(s["W"]) + 0.5 * comp_c
                    )
                    z = fc_top + stack_v + 0.5 * float(s["H"])
                    p = _pack_top_forward_x(
                        name=str(s["name"]),
                        L=float(s["L"]),
                        W=float(s["W"]),
                        H=float(s["H"]),
                        y=y,
                        z=z,
                        host_fwd=x_fc_fwd,
                        host_aft=x_fc_aft,
                        blockers=blockers,
                        axial=axial_c,
                        allow_forward_of_host=fc_is_aft_host,
                    )
                else:
                    # fc_side — keep X centred on FC (not a second-level top shelf).
                    p = _seat_on_fc(
                        site=site,
                        name=str(s["name"]),
                        L=float(s["L"]),
                        W=float(s["W"]),
                        H=float(s["H"]),
                        x_fc=x_fc,
                        z_fc=z_fc,
                        fc_L=fc_L,
                        fc_W=fc_W,
                        fc_H=fc_H,
                        stack_v=stack_v,
                        comp_c=comp_c,
                        side_sign=float(s["sign"]),
                    )
                placed_seats[str(s["name"])] = p
                blockers.append(p)

            p_cb = placed_seats["fuel_cell_control_box"]
            p_bat = placed_seats["buffer_battery"]

            h2_occupants = [
                p
                for p in (p_cb, p_bat)
                if (p[0] == "fuel_cell_control_box" and cb_site == "h2_top")
                or (p[0] == "buffer_battery" and bat_site == "h2_top")
            ]
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
                axial=axial_c,
                occupants=h2_occupants,
            )
            if pdb_count < 2:
                pdbs = pdbs[:1]

            placed = list(base) + [p_cb, p_bat] + list(pdbs)
            axial_bad = _axial_gap_violations(placed, axial=axial_c)
            if axial_bad:
                raise ValueError("axial clearance: " + "; ".join(axial_bad[:2]))

            # Z groups for min-OD lift search (raise stacks toward the centreline).
            z_groups: Dict[str, str] = {
                "payload": "nose",
                "autopilot": "nose",
                "fuel_cell_system": "fc",
                "hydrogen_cylinder": "h2",
            }
            if cb_site.startswith("fc"):
                z_groups["fuel_cell_control_box"] = "fc"
            else:
                z_groups["fuel_cell_control_box"] = "h2"
            if bat_site.startswith("fc"):
                z_groups["buffer_battery"] = "fc"
            else:
                z_groups["buffer_battery"] = "h2"
            for p in pdbs:
                z_groups[p[0]] = "h2"

            placed, z_lifts, od_need = _optimize_pack_z(
                placed,
                z_groups,
                z_floor=z_floor,
                z_ceil=z_ceil,
                allowed_r=allowed_r,
                wall_m=wall,
                fuse_od=fuse_od,
                comp_c=comp_c,
            )
            axial_bad = _axial_gap_violations(placed, axial=axial_c)
            if axial_bad:
                raise ValueError("axial clearance after Z-lift: " + "; ".join(axial_bad[:2]))
            by_p = {p[0]: p for p in placed}
            p_cb = by_p["fuel_cell_control_box"]
            p_bat = by_p["buffer_battery"]
            pdbs = [by_p[p[0]] for p in pdbs]

            # Cube forward connector bay at cube height (after Z lift).
            z_cube_now = by_p["autopilot"][6]
            bay = (connector_start, cube_fwd)
            for n, x, y, lx, wy, hz, z in placed:
                if n == "autopilot":
                    continue
                lo, hi = x - 0.5 * lx, x + 0.5 * lx
                if hi > bay[0] + EPS and lo < bay[1] - EPS:
                    if abs(z - z_cube_now) < 0.5 * (hz + cube_H) - EPS and abs(y) < 0.5 * (wy + cube_W) + comp_c:
                        raise ValueError(f"{n} intrudes into Cube Orange connector bay")

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
                    "z_lifts_m": dict(z_lifts),
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
    by_p = {p[0]: p for p in placed}
    x_cb, y_cb, z_cb = p_cb[1], p_cb[2], p_cb[6]
    x_bat, y_bat, z_bat = p_bat[1], p_bat[2], p_bat[6]
    x_cyl, z_cyl = by_p["hydrogen_cylinder"][1], by_p["hydrogen_cylinder"][6]
    x_fc, z_fc = by_p["fuel_cell_system"][1], by_p["fuel_cell_system"][6]
    x_pay, z_pay = by_p["payload"][1], by_p["payload"][6]
    x_cube, z_cube = by_p["autopilot"][1], by_p["autopilot"][6]

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
    d["component_axial_clearance_m"] = axial_c
    d["energy_bay_axial_clearance_m"] = axial_c
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
        by["payload"]["note"] = "Payload in nose; aft face 10 mm forward of nose-section end; floor clearance."

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
            f"Cube Orange over payload in nose; aft 10 mm forward of nose end; "
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
        "energy_bay_axial_gap_m": (
            (x_cyl_fwd - x_fc_aft) if energy_bay_order == "fuel_cell_forward" else (x_fc_fwd - x_cyl_aft)
        ),
        "component_axial_clearance_m": axial_c,
        "energy_bay_axial_clearance_m": axial_c,        "longeron_boss_clearance_m": boss_clear,
        "cabin_z_floor_m": z_floor,
        "cabin_z_ceil_m": z_ceil,
        "z_lifts_m": best.get("z_lifts_m") or {},
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
