"""Phase-3 3-D mass properties and packaging model for H2AIR-4+1.

Coordinate frame used by the design/packaging model
---------------------------------------------------
* x: from aircraft nose, positive aft
* y: aircraft centreline, positive right (legacy input uses left negative/right positive)
* z: positive upward

This is deliberately a *design geometry* frame.  A later 6-DOF phase may use a
body/NED convention; conversion between the two must be explicit.

Evidence discipline
-------------------
Every component carries a status/source.  Missing dimensions are never treated
as a successful packaging check.  Legacy placeholder dimensions/masses may be
used to exercise the model, but the output preserves their status so that a
screening PASS cannot be mistaken for hardware release evidence.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from math import hypot, sqrt
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

EPS = 1e-12


class EvidenceStatus(str, Enum):
    MEASURED = "measured"
    SUPPLIER_CONFIRMED = "supplier_confirmed"
    DERIVED = "calculated"
    ENGINEERING_ASSUMPTION = "engineering_assumption"
    PRELIMINARY_TARGET = "preliminary_target"
    LEGACY_PLACEHOLDER = "legacy_placeholder"
    OPEN = "open"


class CheckStatus(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    WARNING = "WARNING"
    NOT_EVALUATED = "NOT_EVALUATED"


class Shape(str, Enum):
    POINT = "point"
    BOX = "box"
    CYLINDER_X = "cylinder_x"
    CYLINDER_Y = "cylinder_y"
    CYLINDER_Z = "cylinder_z"


@dataclass
class Component:
    name: str
    group: str
    mass_takeoff_kg: float
    position_m: Tuple[float, float, float]
    mass_end_kg: Optional[float] = None
    shape: str = Shape.POINT.value
    dimensions_m: Dict[str, Optional[float]] = field(default_factory=dict)
    package_zone: Optional[str] = None
    status: str = EvidenceStatus.OPEN.value
    source: str = ""
    note: str = ""
    include_in_collision_check: bool = True

    def validate(self) -> List[str]:
        issues: List[str] = []
        if self.mass_takeoff_kg < 0:
            issues.append(f"{self.name}: mass_takeoff_kg cannot be negative")
        if self.mass_end_kg is not None and self.mass_end_kg < 0:
            issues.append(f"{self.name}: mass_end_kg cannot be negative")
        if len(self.position_m) != 3:
            issues.append(f"{self.name}: position_m must have 3 coordinates")
        try:
            Shape(self.shape)
        except ValueError:
            issues.append(f"{self.name}: unsupported shape {self.shape!r}")
        for key, value in self.dimensions_m.items():
            if value is not None and value <= 0:
                issues.append(f"{self.name}: dimension {key} must be > 0 when supplied")
        return issues

    @property
    def end_mass_kg(self) -> float:
        return self.mass_takeoff_kg if self.mass_end_kg is None else self.mass_end_kg

    def known_box_dimensions(self) -> Optional[Tuple[float, float, float]]:
        """Return axis-aligned x/y/z envelope dimensions or None when incomplete."""
        shape = Shape(self.shape)
        d = self.dimensions_m
        if shape == Shape.BOX:
            vals = (d.get("length_x"), d.get("width_y"), d.get("height_z"))
            if all(v is not None and v > 0 for v in vals):
                return tuple(float(v) for v in vals)  # type: ignore[return-value]
            return None
        if shape == Shape.CYLINDER_X:
            L, D = d.get("length"), d.get("diameter")
            if L is not None and D is not None and L > 0 and D > 0:
                return float(L), float(D), float(D)
            return None
        if shape == Shape.CYLINDER_Y:
            L, D = d.get("length"), d.get("diameter")
            if L is not None and D is not None and L > 0 and D > 0:
                return float(D), float(L), float(D)
            return None
        if shape == Shape.CYLINDER_Z:
            L, D = d.get("length"), d.get("diameter")
            if L is not None and D is not None and L > 0 and D > 0:
                return float(D), float(D), float(L)
            return None
        return None

    def centroid_inertia(self, mass_kg: float) -> Tuple[float, float, float]:
        """Principal centroid inertias aligned to model axes (kg m²)."""
        m = max(0.0, float(mass_kg))
        shape = Shape(self.shape)
        d = self.dimensions_m
        if shape == Shape.POINT or m <= 0:
            return 0.0, 0.0, 0.0
        if shape == Shape.BOX:
            box = self.known_box_dimensions()
            if box is None:
                return 0.0, 0.0, 0.0
            lx, ly, lz = box
            return (
                m * (ly * ly + lz * lz) / 12.0,
                m * (lx * lx + lz * lz) / 12.0,
                m * (lx * lx + ly * ly) / 12.0,
            )
        L = d.get("length")
        D = d.get("diameter")
        if L is None or D is None:
            return 0.0, 0.0, 0.0
        L, r = float(L), 0.5 * float(D)
        iaxis = 0.5 * m * r * r
        itrans = m * (3.0 * r * r + L * L) / 12.0
        if shape == Shape.CYLINDER_X:
            return iaxis, itrans, itrans
        if shape == Shape.CYLINDER_Y:
            return itrans, iaxis, itrans
        return itrans, itrans, iaxis


@dataclass
class BoxEnvelope:
    name: str
    center_m: Tuple[float, float, float]
    dimensions_m: Tuple[float, float, float]
    clearance_m: float = 0.0
    status: str = EvidenceStatus.DERIVED.value
    note: str = ""

    @property
    def bounds(self) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
        out = []
        for c, d in zip(self.center_m, self.dimensions_m):
            half = max(0.0, 0.5 * float(d) - float(self.clearance_m))
            out.append((float(c) - half, float(c) + half))
        return out[0], out[1], out[2]


@dataclass
class Check:
    name: str
    status: str
    value: Any = None
    limit: Any = None
    message: str = ""
    affected: List[str] = field(default_factory=list)


@dataclass
class MassProperties:
    scenario: str
    total_mass_kg: float
    cg_m: Tuple[float, float, float]
    Ixx_kgm2: float
    Iyy_kgm2: float
    Izz_kgm2: float
    Ixy_kgm2: float
    Ixz_kgm2: float
    Iyz_kgm2: float
    inertia_tensor_kgm2: List[List[float]]
    positive_definite_screen: bool

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class Phase3Result:
    model_version: str
    coordinate_frame: Dict[str, str]
    components: List[Dict[str, Any]]
    mass_groups_takeoff_kg: Dict[str, float]
    mass_groups_end_kg: Dict[str, float]
    takeoff: MassProperties
    end_of_mission: MassProperties
    hydrogen_cg_sweep: List[Dict[str, Any]]
    packaging_checks: List[Check]
    propeller_clearance_checks: List[Check]
    design_gates: List[Check]
    unknown_or_placeholder_items: List[Dict[str, Any]]
    summary: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        out = asdict(self)
        out["takeoff"] = self.takeoff.to_dict()
        out["end_of_mission"] = self.end_of_mission.to_dict()
        return out


def component_from_mapping(raw: Mapping[str, Any]) -> Component:
    pos = raw.get("position_m", [0.0, 0.0, 0.0])
    c = Component(
        name=str(raw["name"]),
        group=str(raw.get("group", "other")),
        mass_takeoff_kg=float(raw.get("mass_takeoff_kg", 0.0)),
        mass_end_kg=None if raw.get("mass_end_kg") is None else float(raw["mass_end_kg"]),
        position_m=(float(pos[0]), float(pos[1]), float(pos[2])),
        shape=str(raw.get("shape", Shape.POINT.value)),
        dimensions_m=dict(raw.get("dimensions_m", {})),
        package_zone=raw.get("package_zone"),
        status=str(raw.get("status", EvidenceStatus.OPEN.value)),
        source=str(raw.get("source", "")),
        note=str(raw.get("note", "")),
        include_in_collision_check=bool(raw.get("include_in_collision_check", True)),
    )
    issues = c.validate()
    if issues:
        raise ValueError("; ".join(issues))
    return c


def _mass_for(c: Component, scenario: str) -> float:
    if scenario == "takeoff":
        return c.mass_takeoff_kg
    if scenario == "end":
        return c.end_mass_kg
    raise ValueError("scenario must be 'takeoff' or 'end'")


def calculate_mass_properties(components: Sequence[Component], scenario: str) -> MassProperties:
    masses = [_mass_for(c, scenario) for c in components]
    total = sum(masses)
    if total <= EPS:
        raise ValueError("Total mass must be positive")
    cg = tuple(
        sum(m * c.position_m[i] for c, m in zip(components, masses)) / total
        for i in range(3)
    )

    Ixx = Iyy = Izz = 0.0
    Ixy = Ixz = Iyz = 0.0  # conventional products: integral(xy dm), etc.
    for c, m in zip(components, masses):
        if m <= 0:
            continue
        cx, cy, cz = c.position_m
        dx, dy, dz = cx - cg[0], cy - cg[1], cz - cg[2]
        ixx0, iyy0, izz0 = c.centroid_inertia(m)
        Ixx += ixx0 + m * (dy * dy + dz * dz)
        Iyy += iyy0 + m * (dx * dx + dz * dz)
        Izz += izz0 + m * (dx * dx + dy * dy)
        Ixy += m * dx * dy
        Ixz += m * dx * dz
        Iyz += m * dy * dz

    tensor = [
        [Ixx, -Ixy, -Ixz],
        [-Ixy, Iyy, -Iyz],
        [-Ixz, -Iyz, Izz],
    ]
    # Sylvester criterion for a symmetric 3x3 tensor.
    d1 = Ixx
    d2 = Ixx * Iyy - Ixy * Ixy
    det = (
        Ixx * (Iyy * Izz - Iyz * Iyz)
        - (-Ixy) * ((-Ixy) * Izz - (-Iyz) * (-Ixz))
        + (-Ixz) * ((-Ixy) * (-Iyz) - Iyy * (-Ixz))
    )
    pd = d1 > 0 and d2 > 0 and det > 0
    return MassProperties(
        scenario=scenario,
        total_mass_kg=total,
        cg_m=(float(cg[0]), float(cg[1]), float(cg[2])),
        Ixx_kgm2=Ixx,
        Iyy_kgm2=Iyy,
        Izz_kgm2=Izz,
        Ixy_kgm2=Ixy,
        Ixz_kgm2=Ixz,
        Iyz_kgm2=Iyz,
        inertia_tensor_kgm2=tensor,
        positive_definite_screen=pd,
    )


def group_masses(components: Sequence[Component], scenario: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for c in components:
        out[c.group] = out.get(c.group, 0.0) + _mass_for(c, scenario)
    return dict(sorted(out.items()))


def _bounds_for_component(c: Component) -> Optional[Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]]:
    dims = c.known_box_dimensions()
    if dims is None:
        return None
    return tuple((p - d / 2.0, p + d / 2.0) for p, d in zip(c.position_m, dims))  # type: ignore[return-value]


def _box_contains(outer: BoxEnvelope, inner_bounds, clearance_m: float = 0.0) -> bool:
    ob = outer.bounds
    for (ilo, ihi), (olo, ohi) in zip(inner_bounds, ob):
        if ilo < olo + clearance_m - EPS or ihi > ohi - clearance_m + EPS:
            return False
    return True


def _aabb_overlap(a, b, clearance_m: float = 0.0) -> Tuple[bool, Tuple[float, float, float]]:
    overlaps = []
    for (alo, ahi), (blo, bhi) in zip(a, b):
        overlap = min(ahi, bhi) - max(alo, blo)
        overlaps.append(overlap)
    # Hard volumetric intersection (positive overlap on all axes).
    if all(o > EPS for o in overlaps):
        return True, (float(overlaps[0]), float(overlaps[1]), float(overlaps[2]))
    # Clearance gaps: ignore exact flush-face mates (overlap ~ 0 on one axis) so
    # nose-break packaging (payload|FC, Cube|FC) can share a face.
    if clearance_m > 0.0:
        close = all(o > -float(clearance_m) + EPS for o in overlaps)
        flush_axes = sum(1 for o in overlaps if abs(o) <= EPS)
        if close and flush_axes == 0:
            return True, (float(overlaps[0]), float(overlaps[1]), float(overlaps[2]))
    return False, (float(overlaps[0]), float(overlaps[1]), float(overlaps[2]))


def _max_yz_radius_from_axis(
    c: Component,
    *,
    axis_y: float = 0.0,
    axis_z: float = 0.0,
) -> Optional[float]:
    """Farthest YZ radial extent of the component envelope from a fuselage centreline."""
    shape = Shape(c.shape)
    y0, z0 = float(c.position_m[1]), float(c.position_m[2])
    if shape == Shape.CYLINDER_X:
        D = c.dimensions_m.get("diameter")
        if D is None or float(D) <= 0:
            return None
        return hypot(y0 - axis_y, z0 - axis_z) + 0.5 * float(D)
    dims = c.known_box_dimensions()
    if dims is None:
        return None
    _lx, ly, lz = dims
    hy, hz = 0.5 * float(ly), 0.5 * float(lz)
    return max(
        hypot(y0 + sy * hy - axis_y, z0 + sz * hz - axis_z)
        for sy in (-1.0, 1.0)
        for sz in (-1.0, 1.0)
    )


def _circular_pod_radius_m(env: BoxEnvelope) -> float:
    """Treat packaging envelope Y/Z extents as the circular fuselage outer diameter."""
    return 0.5 * min(float(env.dimensions_m[1]), float(env.dimensions_m[2]))


def evaluate_packaging(
    components: Sequence[Component],
    envelopes: Mapping[str, BoxEnvelope],
    *,
    component_clearance_m: float = 0.003,
    cross_section: str = "circular",
) -> List[Check]:
    checks: List[Check] = []
    by_zone: Dict[str, List[Component]] = {}
    for c in components:
        if c.package_zone:
            by_zone.setdefault(c.package_zone, []).append(c)

    for zone_name, items in by_zone.items():
        env = envelopes.get(zone_name)
        if env is None:
            checks.append(Check(
                name=f"zone_exists:{zone_name}", status=CheckStatus.FAIL.value,
                message="Component refers to an undefined packaging zone.",
                affected=[c.name for c in items],
            ))
            continue
        unknown: List[str] = []
        failed: List[str] = []
        for c in items:
            b = _bounds_for_component(c)
            if b is None:
                unknown.append(c.name)
                continue
            if not _box_contains(env, b):
                failed.append(c.name)
        if failed:
            checks.append(Check(
                name=f"fit_inside:{zone_name}", status=CheckStatus.FAIL.value,
                message="One or more known component envelopes extend outside the packaging zone AABB.",
                affected=failed,
            ))
        else:
            checks.append(Check(
                name=f"fit_inside:{zone_name}",
                status=(CheckStatus.NOT_EVALUATED.value if unknown else CheckStatus.PASS.value),
                message=("All known AABB envelopes fit, but some components have incomplete dimensions."
                         if unknown else "All evaluated component AABB envelopes fit inside the zone."),
                affected=unknown,
            ))

        if str(cross_section).lower() == "circular":
            wall_clear = float(env.clearance_m)
            outer_r = _circular_pod_radius_m(env)
            allowed_r = outer_r - wall_clear
            axis_y, axis_z = float(env.center_m[1]), float(env.center_m[2])
            radial_failed: List[str] = []
            radial_unknown: List[str] = []
            worst_protrusion_m = 0.0
            for c in items:
                r_ext = _max_yz_radius_from_axis(c, axis_y=axis_y, axis_z=axis_z)
                if r_ext is None:
                    radial_unknown.append(c.name)
                    continue
                protrusion = r_ext - allowed_r
                if protrusion > EPS:
                    radial_failed.append(
                        f"{c.name} (radial extent {1000.0 * r_ext:.1f} mm > "
                        f"allowed {1000.0 * allowed_r:.1f} mm; "
                        f"protrudes {1000.0 * protrusion:.1f} mm past wall keepout)"
                    )
                    worst_protrusion_m = max(worst_protrusion_m, protrusion)
            if radial_failed:
                checks.append(Check(
                    name=f"circular_wall_clearance:{zone_name}",
                    status=CheckStatus.FAIL.value,
                    value=worst_protrusion_m,
                    limit=wall_clear,
                    message=(
                        f"Component envelope(s) violate the circular fuselage wall keepout "
                        f"({1000.0 * wall_clear:.1f} mm clearance inside OD "
                        f"{1000.0 * 2.0 * outer_r:.1f} mm)."
                    ),
                    affected=radial_failed,
                ))
            else:
                checks.append(Check(
                    name=f"circular_wall_clearance:{zone_name}",
                    status=(CheckStatus.NOT_EVALUATED.value if radial_unknown else CheckStatus.PASS.value),
                    value=0.0,
                    limit=wall_clear,
                    message=(
                        "Known envelopes keep the required wall clearance, but some components "
                        "have incomplete dimensions."
                        if radial_unknown
                        else (
                            f"All evaluated envelopes stay inside the circular fuselage with "
                            f"{1000.0 * wall_clear:.1f} mm wall keepout."
                        )
                    ),
                    affected=radial_unknown,
                ))

        colliding: List[str] = []
        eval_count = 0
        for i in range(len(items)):
            a = items[i]
            if not a.include_in_collision_check:
                continue
            ab = _bounds_for_component(a)
            if ab is None:
                continue
            for j in range(i + 1, len(items)):
                bcomp = items[j]
                if not bcomp.include_in_collision_check:
                    continue
                bb = _bounds_for_component(bcomp)
                if bb is None:
                    continue
                eval_count += 1
                collision, overlap = _aabb_overlap(ab, bb, component_clearance_m)
                if collision:
                    colliding.append(
                        f"{a.name} <-> {bcomp.name} (axis overlaps/gaps m: "
                        f"{overlap[0]:+.4f}, {overlap[1]:+.4f}, {overlap[2]:+.4f})"
                    )
        if colliding:
            checks.append(Check(
                name=f"collision_free:{zone_name}", status=CheckStatus.FAIL.value,
                value=len(colliding), limit=0,
                message="Current mass-station placements are not a collision-free hardware layout.",
                affected=colliding,
            ))
        elif eval_count == 0:
            checks.append(Check(
                name=f"collision_free:{zone_name}", status=CheckStatus.NOT_EVALUATED.value,
                message="No component pair had enough dimensional data for collision evaluation.",
            ))
        else:
            checks.append(Check(
                name=f"collision_free:{zone_name}", status=CheckStatus.PASS.value,
                value=0, limit=0, message="No collision found among evaluated component envelopes.",
            ))
    return checks


def evaluate_lift_propeller_clearance(
    centers_m: Sequence[Tuple[float, float, float]],
    diameter_m: float,
    *,
    required_edge_clearance_m: float = 0.020,
) -> List[Check]:
    """Coplanar horizontal lift-disk pair check based on x-y centre spacing."""
    checks: List[Check] = []
    if diameter_m <= 0 or len(centers_m) < 2:
        return [Check("lift_propeller_pair_clearance", CheckStatus.NOT_EVALUATED.value,
                      message="Lift propeller diameter/centres unavailable.")]
    min_edge = float("inf")
    min_pair = None
    for i in range(len(centers_m)):
        for j in range(i + 1, len(centers_m)):
            dx = centers_m[i][0] - centers_m[j][0]
            dy = centers_m[i][1] - centers_m[j][1]
            dz = centers_m[i][2] - centers_m[j][2]
            # If rotor planes are materially separated in z, a simple coplanar disk check is not decisive.
            if abs(dz) > 0.05 * diameter_m:
                continue
            centre_distance = sqrt(dx * dx + dy * dy)
            edge = centre_distance - diameter_m
            if edge < min_edge:
                min_edge = edge
                min_pair = (i + 1, j + 1, centre_distance)
    if min_pair is None:
        return [Check("lift_propeller_pair_clearance", CheckStatus.NOT_EVALUATED.value,
                      message="No approximately coplanar lift-propeller pairs were found.")]
    status = CheckStatus.PASS.value if min_edge >= required_edge_clearance_m else CheckStatus.FAIL.value
    checks.append(Check(
        name="lift_propeller_pair_clearance",
        status=status,
        value=min_edge,
        limit=required_edge_clearance_m,
        message=(f"Minimum lift-disk edge clearance is {min_edge*1000:.1f} mm; "
                 f"governing pair M{min_pair[0]}/M{min_pair[1]}."),
        affected=[f"M{min_pair[0]}", f"M{min_pair[1]}"],
    ))
    max_diam_for_clearance = max(0.0, min_pair[2] - required_edge_clearance_m)
    checks.append(Check(
        name="lift_propeller_max_diameter_for_current_centres",
        status=CheckStatus.WARNING.value,
        value=max_diam_for_clearance,
        limit=diameter_m,
        message=(f"At the current lift-motor centres and {required_edge_clearance_m*1000:.0f} mm edge "
                 f"clearance, the governing equal propeller diameter is <= {max_diam_for_clearance:.4f} m "
                 f"({max_diam_for_clearance/0.0254:.2f} in)."),
    ))
    return checks


def evaluate_lift_disk_to_pod(
    centers_m: Sequence[Tuple[float, float, float]],
    diameter_m: float,
    pod_outer: BoxEnvelope,
    *,
    required_clearance_m: float = 0.020,
) -> Check:
    """Horizontal rotor disk versus pod top-view rectangle."""
    if diameter_m <= 0:
        return Check("lift_propeller_to_pod", CheckStatus.NOT_EVALUATED.value)
    r = diameter_m / 2.0
    (xmin, xmax), (ymin, ymax), _ = pod_outer.bounds
    min_gap = float("inf")
    for x, y, _z in centers_m:
        nx = min(max(x, xmin), xmax)
        ny = min(max(y, ymin), ymax)
        dist = sqrt((x - nx) ** 2 + (y - ny) ** 2)
        min_gap = min(min_gap, dist - r)
    return Check(
        "lift_propeller_to_pod",
        CheckStatus.PASS.value if min_gap >= required_clearance_m else CheckStatus.FAIL.value,
        value=min_gap, limit=required_clearance_m,
        message=f"Minimum top-view lift-disk to pod clearance: {min_gap*1000:.1f} mm.",
    )


def evaluate_pusher_clearances(
    *,
    prop_center_m: Tuple[float, float, float],
    diameter_m: float,
    boom_y_m: Sequence[float],
    boom_radius_m: Optional[float],
    tail_le_x_m: Optional[float],
    required_radial_clearance_m: float = 0.020,
    required_axial_clearance_m: float = 0.020,
) -> List[Check]:
    checks: List[Check] = []
    if diameter_m <= 0:
        return [Check("pusher_clearance", CheckStatus.NOT_EVALUATED.value)]
    r = diameter_m / 2.0
    if boom_y_m and boom_radius_m is not None:
        radial_gap = min(abs(y - prop_center_m[1]) - r - boom_radius_m for y in boom_y_m)
        checks.append(Check(
            "pusher_disk_to_boom_lateral_clearance",
            CheckStatus.PASS.value if radial_gap >= required_radial_clearance_m else CheckStatus.FAIL.value,
            value=radial_gap, limit=required_radial_clearance_m,
            message=f"Minimum pusher disk-edge to boom-envelope lateral clearance: {radial_gap*1000:.1f} mm.",
        ))
    else:
        checks.append(Check("pusher_disk_to_boom_lateral_clearance", CheckStatus.NOT_EVALUATED.value,
                            message="Boom location/radius unavailable."))
    if tail_le_x_m is not None:
        axial = tail_le_x_m - prop_center_m[0]
        checks.append(Check(
            "pusher_plane_to_tail_le_axial_clearance",
            CheckStatus.PASS.value if axial >= required_axial_clearance_m else CheckStatus.FAIL.value,
            value=axial, limit=required_axial_clearance_m,
            message=f"Pusher disk-plane to horizontal-tail leading-edge axial separation: {axial*1000:.1f} mm.",
        ))
    else:
        checks.append(Check("pusher_plane_to_tail_le_axial_clearance", CheckStatus.NOT_EVALUATED.value))
    return checks


def hydrogen_fraction_sweep(
    components: Sequence[Component],
    *,
    fractions: Sequence[float],
    hydrogen_component_name: str = "stored_hydrogen",
) -> List[Dict[str, Any]]:
    original = None
    for c in components:
        if c.name == hydrogen_component_name:
            original = c
            break
    if original is None:
        return []
    out: List[Dict[str, Any]] = []
    for f in fractions:
        f = max(0.0, min(1.0, float(f)))
        copied = [Component(**asdict(c)) for c in components]
        for c in copied:
            if c.name == hydrogen_component_name:
                c.mass_takeoff_kg = original.mass_takeoff_kg * f
                c.mass_end_kg = c.mass_takeoff_kg
        mp = calculate_mass_properties(copied, "takeoff")
        out.append({
            "h2_fraction": f,
            "h2_mass_kg": original.mass_takeoff_kg * f,
            "total_mass_kg": mp.total_mass_kg,
            "cg_m": list(mp.cg_m),
            "Ixx_kgm2": mp.Ixx_kgm2,
            "Iyy_kgm2": mp.Iyy_kgm2,
            "Izz_kgm2": mp.Izz_kgm2,
        })
    return out


def static_margin_reference_check(
    cg_x_m: float,
    *,
    neutral_point_x_m: Optional[float],
    wing_MAC_m: Optional[float],
    minimum_static_margin: float = 0.05,
) -> Check:
    if neutral_point_x_m is None or wing_MAC_m is None or wing_MAC_m <= 0:
        return Check("reference_static_margin", CheckStatus.NOT_EVALUATED.value,
                     message="Reference neutral point or MAC unavailable; recompute in the stability phase.")
    sm = (neutral_point_x_m - cg_x_m) / wing_MAC_m
    max_cg_for_limit = neutral_point_x_m - minimum_static_margin * wing_MAC_m
    forward_shift_needed = max(0.0, cg_x_m - max_cg_for_limit)
    return Check(
        name="reference_static_margin",
        status=CheckStatus.PASS.value if sm >= minimum_static_margin else CheckStatus.FAIL.value,
        value=sm,
        limit=minimum_static_margin,
        message=("Cross-check only: uses the inherited neutral point from the current aero/stability model. "
                 f"At that fixed neutral point, reaching {minimum_static_margin*100:.1f}% MAC would require "
                 f"CG x <= {max_cg_for_limit:.4f} m (about {forward_shift_needed*1000:.1f} mm forward from this ledger). "
                 "Aero/propulsion/6 must recompute the neutral point after geometry/CG updates."),
    )


def build_phase3_result(
    *,
    components: Sequence[Component],
    geometry: Mapping[str, Any],
    requirements: Mapping[str, Any],
    packaging: Mapping[str, Any],
    model_version: str = "H2AIR-phase3-mass-geometry-1.0",
) -> Phase3Result:
    takeoff = calculate_mass_properties(components, "takeoff")
    end = calculate_mass_properties(components, "end")
    groups_to = group_masses(components, "takeoff")
    groups_end = group_masses(components, "end")

    envelopes: Dict[str, BoxEnvelope] = {}
    for name, raw in packaging.get("envelopes", {}).items():
        envelopes[name] = BoxEnvelope(
            name=name,
            center_m=tuple(float(v) for v in raw["center_m"]),
            dimensions_m=tuple(float(v) for v in raw["dimensions_m"]),
            clearance_m=float(raw.get("wall_clearance_m", 0.0)),
            status=str(raw.get("status", EvidenceStatus.DERIVED.value)),
            note=str(raw.get("note", "")),
        )
    packaging_checks = evaluate_packaging(
        components, envelopes,
        component_clearance_m=float(packaging.get("component_clearance_m", 0.003)),
        cross_section=str(packaging.get("fuselage_cross_section", "circular")),
    )

    lift_centers = [tuple(map(float, p)) for p in geometry.get("lift_prop_centers_m", [])]
    lift_d = float(geometry.get("lift_prop_diameter_m", 0.0) or 0.0)
    prop_checks = evaluate_lift_propeller_clearance(
        lift_centers, lift_d,
        required_edge_clearance_m=float(geometry.get("lift_prop_required_edge_clearance_m", 0.020)),
    )
    pod_outer_raw = geometry.get("central_pod_outer")
    if pod_outer_raw:
        pod_outer = BoxEnvelope(
            "central_pod_outer",
            tuple(float(v) for v in pod_outer_raw["center_m"]),
            tuple(float(v) for v in pod_outer_raw["dimensions_m"]),
        )
        prop_checks.append(evaluate_lift_disk_to_pod(
            lift_centers, lift_d, pod_outer,
            required_clearance_m=float(geometry.get("lift_prop_to_pod_required_clearance_m", 0.020)),
        ))
    pusher = geometry.get("pusher", {})
    if pusher:
        prop_checks.extend(evaluate_pusher_clearances(
            prop_center_m=tuple(float(v) for v in pusher.get("center_m", [0, 0, 0])),
            diameter_m=float(pusher.get("diameter_m", 0.0)),
            boom_y_m=[float(v) for v in geometry.get("boom_center_y_m", [])],
            boom_radius_m=(None if geometry.get("boom_outer_diameter_m") is None
                           else 0.5 * float(geometry["boom_outer_diameter_m"])),
            tail_le_x_m=(None if geometry.get("horizontal_tail_le_x_m") is None
                         else float(geometry["horizontal_tail_le_x_m"])),
            required_radial_clearance_m=float(geometry.get("pusher_required_radial_clearance_m", 0.020)),
            required_axial_clearance_m=float(geometry.get("pusher_required_axial_clearance_m", 0.020)),
        ))

    # A geometrically clear placeholder is not a release-quality PASS.
    lift_d_status = str(geometry.get("lift_prop_diameter_status", "open"))
    pusher_d_status = str(pusher.get("diameter_status", "open")) if pusher else "open"
    for check in prop_checks:
        if check.status != CheckStatus.PASS.value:
            continue
        is_pusher = check.name.startswith("pusher_")
        status = pusher_d_status if is_pusher else lift_d_status
        if status not in {EvidenceStatus.SUPPLIER_CONFIRMED.value, EvidenceStatus.MEASURED.value}:
            check.status = CheckStatus.WARNING.value
            check.message += " Screening only because the propeller diameter is not yet supplier/measured final hardware."

    gates: List[Check] = []
    ceiling = float(requirements.get("mtow_ceiling_kg", 15.0))
    warning = float(requirements.get("mtow_warning_kg", 14.0))
    gates.append(Check(
        "mtow_ceiling_numeric",
        CheckStatus.PASS.value if takeoff.total_mass_kg <= ceiling else CheckStatus.FAIL.value,
        value=takeoff.total_mass_kg, limit=ceiling,
        message="Numerical ledger check; hardware-completeness gate is separate.",
    ))
    if takeoff.total_mass_kg > warning and takeoff.total_mass_kg <= ceiling:
        gates.append(Check("mtow_growth_margin", CheckStatus.WARNING.value,
                           value=ceiling - takeoff.total_mass_kg, limit=ceiling-warning,
                           message="Forecast MTOW is above the 14 kg design-review trigger."))
    else:
        gates.append(Check("mtow_growth_margin", CheckStatus.PASS.value,
                           value=ceiling - takeoff.total_mass_kg,
                           message="Numerical mass margin to the 15 kg ceiling."))

    uncertain = [
        {"name": c.name, "status": c.status, "source": c.source, "note": c.note}
        for c in components
        if c.status in {EvidenceStatus.OPEN.value, EvidenceStatus.LEGACY_PLACEHOLDER.value,
                        EvidenceStatus.ENGINEERING_ASSUMPTION.value}
    ]
    critical_unknown_names = set(packaging.get("critical_mass_items", ["buffer_battery", "hydrogen_cylinder"]))
    critical_unknown = [c.name for c in components if c.name in critical_unknown_names and
                        c.status in {EvidenceStatus.OPEN.value, EvidenceStatus.LEGACY_PLACEHOLDER.value,
                                     EvidenceStatus.ENGINEERING_ASSUMPTION.value}]
    gates.append(Check(
        "hardware_mass_completeness",
        CheckStatus.NOT_EVALUATED.value if critical_unknown else CheckStatus.PASS.value,
        value=len(critical_unknown), limit=0,
        message=("Final MTOW cannot be released until selected battery and cylinder masses are supplier/measured values."
                 if critical_unknown else "Critical hardware masses are sufficiently sourced."),
        affected=critical_unknown,
    ))
    gates.append(static_margin_reference_check(
        takeoff.cg_m[0],
        neutral_point_x_m=(None if geometry.get("reference_neutral_point_x_m") is None
                           else float(geometry["reference_neutral_point_x_m"])),
        wing_MAC_m=(None if geometry.get("wing_MAC_m") is None else float(geometry["wing_MAC_m"])),
        minimum_static_margin=float(requirements.get("minimum_reference_static_margin", 0.05)),
    ))
    gates.append(Check(
        "lateral_cg_symmetry",
        CheckStatus.PASS.value if abs(takeoff.cg_m[1]) <= float(requirements.get("max_abs_cg_y_m", 0.005)) else CheckStatus.FAIL.value,
        value=takeoff.cg_m[1], limit=float(requirements.get("max_abs_cg_y_m", 0.005)),
        message="3-D BOM lateral-CG symmetry check.",
    ))
    gates.append(Check(
        "inertia_tensor_positive_definite",
        CheckStatus.PASS.value if takeoff.positive_definite_screen else CheckStatus.FAIL.value,
        value=takeoff.positive_definite_screen,
        message="Basic physical consistency screen for the assembled inertia tensor.",
    ))

    h2_sweep = hydrogen_fraction_sweep(
        components,
        fractions=geometry.get("hydrogen_mass_fraction_sweep", [1.0, 0.75, 0.5, 0.25, 0.0]),
    )
    payload_mass = groups_to.get("payload", 0.0)
    h2_mass = next((c.mass_takeoff_kg for c in components if c.name == "stored_hydrogen"), 0.0)
    empty_mass = takeoff.total_mass_kg - payload_mass - h2_mass
    summary = {
        "takeoff_mass_kg": takeoff.total_mass_kg,
        "empty_mass_excluding_payload_and_h2_kg": empty_mass,
        "payload_mass_kg": payload_mass,
        "payload_fraction": payload_mass / max(takeoff.total_mass_kg, EPS),
        "takeoff_cg_m": list(takeoff.cg_m),
        "end_cg_m": list(end.cg_m),
        "mission_cg_shift_mm": [(end.cg_m[i]-takeoff.cg_m[i])*1000.0 for i in range(3)],
        "mass_margin_to_15kg_kg": 15.0 - takeoff.total_mass_kg,
        "Ixx_kgm2": takeoff.Ixx_kgm2,
        "Iyy_kgm2": takeoff.Iyy_kgm2,
        "Izz_kgm2": takeoff.Izz_kgm2,
        "product_inertia_Ixy_kgm2": takeoff.Ixy_kgm2,
        "product_inertia_Ixz_kgm2": takeoff.Ixz_kgm2,
        "product_inertia_Iyz_kgm2": takeoff.Iyz_kgm2,
        "packaging_fail_count": sum(c.status == CheckStatus.FAIL.value for c in packaging_checks),
        "propeller_clearance_fail_count": sum(c.status == CheckStatus.FAIL.value for c in prop_checks),
        "design_gate_fail_count": sum(c.status == CheckStatus.FAIL.value for c in gates),
        "critical_mass_complete": not bool(critical_unknown),
    }

    return Phase3Result(
        model_version=model_version,
        coordinate_frame={
            "x": "nose -> aft",
            "y": "centreline -> right positive (left negative)",
            "z": "up positive",
            "note": "design/packaging frame; future 6-DOF frame conversion must be explicit",
        },
        components=[asdict(c) for c in components],
        mass_groups_takeoff_kg=groups_to,
        mass_groups_end_kg=groups_end,
        takeoff=takeoff,
        end_of_mission=end,
        hydrogen_cg_sweep=h2_sweep,
        packaging_checks=packaging_checks,
        propeller_clearance_checks=prop_checks,
        design_gates=gates,
        unknown_or_placeholder_items=uncertain,
        summary=summary,
    )

# Clean integrated API alias; legacy development name retained for compatibility.
build_mass_geometry_result = build_phase3_result
