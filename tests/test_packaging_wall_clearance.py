from __future__ import annotations

from h2air.mass_properties import (
    BoxEnvelope,
    CheckStatus,
    Component,
    Shape,
    evaluate_packaging,
)


def _box(name: str, *, pos, dims, zone="central_pod") -> Component:
    return Component(
        name=name,
        group="energy",
        mass_takeoff_kg=1.0,
        position_m=tuple(float(v) for v in pos),
        shape=Shape.BOX.value,
        dimensions_m={
            "length_x": float(dims[0]),
            "width_y": float(dims[1]),
            "height_z": float(dims[2]),
        },
        package_zone=zone,
        include_in_collision_check=False,
    )


def test_circular_wall_clearance_flags_box_that_fits_aabb():
    """160x120 mm box fits a 180 mm AABB but corners stick out of the circle."""
    env = BoxEnvelope(
        name="central_pod",
        center_m=(0.39, 0.0, 0.0),
        dimensions_m=(0.78, 0.18, 0.18),
        clearance_m=0.003,
    )
    fc = _box("fuel_cell_system", pos=(0.16, 0.0, -0.025), dims=(0.26, 0.16, 0.12))
    checks = evaluate_packaging([fc], {"central_pod": env}, cross_section="circular")
    by = {c.name: c for c in checks}
    assert by["fit_inside:central_pod"].status == CheckStatus.PASS.value
    radial = by["circular_wall_clearance:central_pod"]
    assert radial.status == CheckStatus.FAIL.value
    assert any("fuel_cell_system" in a for a in radial.affected)


def test_circular_wall_clearance_passes_with_keepout():
    env = BoxEnvelope(
        name="central_pod",
        center_m=(0.39, 0.0, 0.0),
        dimensions_m=(0.78, 0.18, 0.18),
        clearance_m=0.003,
    )
    # 50 mm square on centreline: corner radius ~35.4 mm << 87 mm allowed.
    bat = _box("buffer_battery", pos=(0.35, 0.0, 0.0), dims=(0.16, 0.05, 0.05))
    checks = evaluate_packaging([bat], {"central_pod": env}, cross_section="circular")
    by = {c.name: c for c in checks}
    assert by["circular_wall_clearance:central_pod"].status == CheckStatus.PASS.value
