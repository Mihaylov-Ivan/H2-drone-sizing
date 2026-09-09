from __future__ import annotations

"""Fuel-cell cooling architecture screening for H2AIR.

This module models only the *airframe integration around* the IE-SOAR cooling
fans.  It assumes, per the current project integration decision, that the two
fuel-cell fans draw ambient air through the stack and discharge through the fan
face.  Supplier fan-flow / fan-curve data are not yet available, therefore fan
capacity is an explicit engineering assumption in user_inputs.json.

The screening model deliberately does not credit forward-flight ram air when it
checks hover/ground cooling.  Cruise ram pressure is reported only as a possible
bonus.  Pressure loss is represented by a simple lumped K model and intersected
with an assumed linear external fan-pressure curve.
"""

from dataclasses import asdict, dataclass
from math import atan, degrees
from typing import Any, Dict, Mapping, Optional

CFM_PER_M3_S = 2118.880003
EPS = 1e-12


def _component(cfg: Mapping[str, Any], name: str) -> Optional[Mapping[str, Any]]:
    for c in cfg["geometry_mass"]["component_ledger"]["components"]:
        if c.get("name") == name:
            return c
    return None


def _slot_geometry(block: Mapping[str, Any]) -> Dict[str, float]:
    """Return gross and free side-panel area from slot geometry.

    slot_length_x is the long dimension of each slot; slot_width_z and
    slot_gap_z stack the slots vertically on the fuselage side wall.
    """
    n = max(1, int(block.get("slot_count", 1)))
    L = float(block.get("slot_length_x_m", 0.0))
    w = float(block.get("slot_width_z_m", 0.0))
    gap = max(0.0, float(block.get("slot_gap_z_m", 0.0)))
    free = n * L * w
    panel_h = n * w + max(0, n - 1) * gap
    gross = L * panel_h
    return {
        "slot_count": n,
        "slot_length_x_m": L,
        "slot_width_z_m": w,
        "slot_gap_z_m": gap,
        "panel_length_x_m": L,
        "panel_height_z_m": panel_h,
        "gross_area_m2": gross,
        "free_area_m2": free,
        "open_fraction": free / max(gross, EPS),
    }


def cooling_opening_cda_from_user_inputs(user_cooling: Mapping[str, Any]) -> float:
    """Screening cruise CdA of the cooling openings.

    This is intentionally a geometry-level drag allowance, not a CFD result.
    """
    if not user_cooling or not bool(user_cooling.get("enabled", False)):
        return 0.0
    inlet = user_cooling.get("inlet", {}) or {}
    exhaust = user_cooling.get("exhaust", {}) or {}
    ai = _slot_geometry(inlet)["gross_area_m2"]
    ao = _slot_geometry(exhaust)["gross_area_m2"]
    cdi = float(inlet.get("external_drag_coefficient_screen", 0.03))
    cdo = float(exhaust.get("external_drag_coefficient_screen", 0.04))
    return max(0.0, ai * cdi + ao * cdo)


@dataclass(frozen=True)
class FuelCellCoolingResult:
    enabled: bool
    architecture: str
    fan_flow_direction: str
    fan_side: str
    inlet_side: str
    exhaust_side: str
    fuel_cell_net_power_W: float
    efficiency_screen: float
    waste_heat_nominal_W: float
    waste_heat_design_W: float
    allowable_air_temperature_rise_C: float
    required_air_mass_flow_kg_s: float
    required_air_volume_flow_m3_s: float
    required_airflow_cfm: float
    assumed_module_zero_external_pressure_airflow_cfm: float
    assumed_external_shutoff_pressure_Pa: float
    inlet_free_area_m2: float
    exhaust_free_area_m2: float
    minimum_flow_area_m2: float
    recommended_minimum_free_area_m2: float
    inlet_velocity_at_required_flow_mps: float
    exhaust_velocity_at_required_flow_mps: float
    total_external_loss_coefficient_K: float
    estimated_external_pressure_drop_at_required_flow_Pa: float
    estimated_operating_airflow_m3_s: float
    estimated_operating_airflow_cfm: float
    airflow_margin_ratio: float
    airflow_screen_pass: bool
    fuselage_width_m: float
    fuselage_wall_keepout_m: float
    fuel_cell_width_m: float
    inlet_side_plenum_gap_m: float
    exhaust_side_plenum_gap_m: float
    required_inlet_side_plenum_gap_m: float
    required_exhaust_side_plenum_gap_m: float
    minimum_fuselage_width_for_plenums_m: float
    plenum_geometry_pass: bool
    inlet_panel_fits_fc_face: bool
    exhaust_panel_fits_fc_face: bool
    side_opening_geometry_pass: bool
    overall_geometry_pass: bool
    cooling_openings_CdA_m2: float
    cooling_drag_at_cruise_N: float
    estimated_cruise_ram_pressure_Pa: float
    credited_cruise_ram_pressure_Pa: float
    inlet_geometry: Dict[str, Any]
    exhaust_geometry: Dict[str, Any]
    fan_geometry: Dict[str, Any]
    evidence_status: str
    warning: str

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _system_pressure_drop_Pa(rho: float, q_m3s: float, area_m2: float, K: float) -> float:
    if q_m3s <= 0 or area_m2 <= 0 or K <= 0:
        return 0.0
    v = q_m3s / area_m2
    return K * 0.5 * rho * v * v


def _fan_available_pressure_Pa(q_m3s: float, q0_m3s: float, shutoff_Pa: float) -> float:
    if q0_m3s <= 0 or shutoff_Pa <= 0:
        return 0.0
    return max(0.0, shutoff_Pa * (1.0 - q_m3s / q0_m3s))


def _solve_operating_flow(rho: float, area_m2: float, K: float, q0_m3s: float, shutoff_Pa: float) -> float:
    """Solve intersection of a linear assumed fan curve and quadratic system curve."""
    if q0_m3s <= 0 or area_m2 <= 0:
        return 0.0
    lo, hi = 0.0, q0_m3s
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        avail = _fan_available_pressure_Pa(mid, q0_m3s, shutoff_Pa)
        loss = _system_pressure_drop_Pa(rho, mid, area_m2, K)
        if avail >= loss:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def analyze_fuel_cell_cooling(cfg: Mapping[str, Any], *, cruise_speed_mps: Optional[float] = None) -> FuelCellCoolingResult:
    snap = (cfg.get("user_input_snapshot") or {}).get("fuel_cell_cooling") or {}
    enabled = bool(snap.get("enabled", False))
    arch = str(snap.get("architecture", "cross_fuselage"))
    direction = str(snap.get("fan_flow_direction", "suction_through_stack_to_fan_side"))
    fan_side = str(snap.get("fan_side", "right")).lower()
    inlet_side = str(snap.get("inlet_side", "left")).lower()
    exhaust_side = str(snap.get("exhaust_side", fan_side)).lower()

    thermal = snap.get("thermal_screen", {}) or {}
    fans = snap.get("fans", {}) or {}
    inlet = snap.get("inlet", {}) or {}
    exhaust = snap.get("exhaust", {}) or {}
    plenum = snap.get("plenum", {}) or {}
    loss = snap.get("external_pressure_loss_coefficients", {}) or {}

    fc_net = float(thermal.get("fuel_cell_net_power_W", cfg.get("power_system", {}).get("fuel_cell", {}).get("rated_net_power_W", 800.0)))
    eta = float(thermal.get("fuel_cell_efficiency_screen", 0.50))
    if not (0 < eta < 1):
        raise ValueError("fuel_cell_cooling.thermal_screen.fuel_cell_efficiency_screen must be in (0,1)")
    margin = max(1.0, float(thermal.get("waste_heat_margin_factor", 1.10)))
    dT = max(1e-6, float(thermal.get("allowable_air_temperature_rise_C", 15.0)))
    rho = float(thermal.get("air_density_kg_m3", cfg.get("atmosphere", {}).get("rho_kg_m3", 1.225)))
    cp = max(1.0, float(thermal.get("air_specific_heat_J_kgK", 1005.0)))
    waste = fc_net * (1.0 / eta - 1.0)
    waste_design = waste * margin
    mdot = waste_design / (cp * dT)
    q_req = mdot / max(rho, EPS)

    ig = _slot_geometry(inlet)
    eg = _slot_geometry(exhaust)
    a_min = min(ig["free_area_m2"], eg["free_area_m2"])
    vmax = max(0.1, float(snap.get("target_max_opening_air_velocity_mps", 5.0)))
    a_rec = q_req / vmax

    K = sum(max(0.0, float(v)) for v in loss.values())
    q0_cfm = float(fans.get("assumed_module_airflow_no_external_duct_cfm", 150.0))
    q0 = q0_cfm / CFM_PER_M3_S
    shut = float(fans.get("assumed_external_shutoff_pressure_Pa", 90.0))
    q_oper = _solve_operating_flow(rho, a_min, K, q0, shut) if enabled else 0.0

    inlet_v = q_req / max(ig["free_area_m2"], EPS)
    exhaust_v = q_req / max(eg["free_area_m2"], EPS)
    dp_req = _system_pressure_drop_Pa(rho, q_req, a_min, K)
    flow_margin = q_oper / max(q_req, EPS)
    airflow_pass = bool(enabled and flow_margin >= 1.0)

    d = cfg.get("design", {})
    fuse_w = float(d.get("fuselage_width_m", d.get("fuselage_max_diameter_m", 0.0)))
    wall = float(d.get("fuselage_wall_clearance_m", (cfg.get("geometry_mass", {}).get("geometry_model", {}).get("packaging", {}).get("envelopes", {}).get("central_pod", {}) or {}).get("wall_clearance_m", 0.0)))
    fc = _component(cfg, "fuel_cell_system") or {}
    dims = fc.get("dimensions_m", {}) or {}
    fc_w = float(dims.get("width_y", 0.1084))
    fc_L = float(dims.get("length_x", 0.2305))
    fc_H = float(dims.get("height_z", 0.1198))
    fc_pos = fc.get("position_m", [0.0, 0.0, 0.0])
    y_fc = float(fc_pos[1] if len(fc_pos) > 1 else 0.0)

    inner_half = max(0.0, fuse_w / 2.0 - wall)
    right_gap = inner_half - (y_fc + fc_w / 2.0)
    left_gap = (y_fc - fc_w / 2.0) - (-inner_half)
    side_gaps = {"left": left_gap, "right": right_gap}
    gap_in = side_gaps.get(inlet_side, left_gap)
    gap_out = side_gaps.get(exhaust_side, right_gap)
    req_in_gap = max(0.0, float(plenum.get("minimum_inlet_side_gap_m", 0.004)))
    req_out_gap = max(0.0, float(plenum.get("minimum_fan_side_shroud_gap_m", 0.010)))
    min_fuse_w = fc_w + 2.0 * wall + req_in_gap + req_out_gap
    plenum_pass = gap_in + 1e-12 >= req_in_gap and gap_out + 1e-12 >= req_out_gap

    inlet_fits = ig["panel_length_x_m"] <= fc_L + 1e-12 and ig["panel_height_z_m"] <= fc_H + 1e-12
    exhaust_fits = eg["panel_length_x_m"] <= fc_L + 1e-12 and eg["panel_height_z_m"] <= fc_H + 1e-12
    opening_pass = inlet_fits and exhaust_fits and ig["free_area_m2"] + 1e-12 >= a_rec and eg["free_area_m2"] + 1e-12 >= a_rec
    geom_pass = plenum_pass and opening_pass

    cooling_cda = cooling_opening_cda_from_user_inputs(snap)
    V = float(cruise_speed_mps if cruise_speed_mps is not None else d.get("cruise_speed_mps", 20.0))
    dyn = 0.5 * rho * V * V
    drag = dyn * cooling_cda
    recovery = max(0.0, min(1.0, float(inlet.get("cruise_pressure_recovery_fraction_screen", 0.15))))
    ram = dyn
    credited_ram = 0.0  # deliberately zero for the hover/ground design gate

    nfan = max(1, int(fans.get("count", 2)))
    fan_d = float(fans.get("diameter_m", 0.080))
    fan_spacing = float(fans.get("center_spacing_m", 0.086))
    fan_geom = {
        "count": nfan,
        "diameter_m": fan_d,
        "center_spacing_m": fan_spacing,
        "total_span_x_m": fan_d + max(0, nfan - 1) * fan_spacing,
        "assumed_module_airflow_no_external_duct_cfm": q0_cfm,
        "assumed_external_shutoff_pressure_Pa": shut,
        "fan_data_status": str(fans.get("status", "engineering_assumption")),
    }

    inlet_geom = dict(ig)
    inlet_geom.update({
        "type": str(inlet.get("type", "naca_style_flush_side_inlet")),
        "side": inlet_side,
        "center_x_m": float(fc_pos[0] if len(fc_pos) > 0 else 0.0) + float(inlet.get("center_x_offset_from_fc_m", 0.0)),
        "center_z_m": float(fc_pos[2] if len(fc_pos) > 2 else 0.0) + float(inlet.get("center_z_offset_from_fc_m", 0.0)),
        "recess_depth_y_m": float(inlet.get("recess_depth_y_m", 0.008)),
        "ramp_length_x_m": float(inlet.get("ramp_length_x_m", ig["panel_length_x_m"])),
        "ramp_angle_deg": degrees(atan(float(inlet.get("recess_depth_y_m", 0.008)) / max(float(inlet.get("ramp_length_x_m", ig["panel_length_x_m"])), EPS))),
        "sidewall_angle_deg": float(inlet.get("sidewall_angle_deg", 7.0)),
        "lip_radius_m": float(inlet.get("lip_radius_m", 0.002)),
        "cruise_pressure_recovery_fraction_screen": recovery,
    })
    exhaust_geom = dict(eg)
    exhaust_geom.update({
        "type": str(exhaust.get("type", "flush_fan_exhaust_grille")),
        "side": exhaust_side,
        "center_x_m": float(fc_pos[0] if len(fc_pos) > 0 else 0.0) + float(exhaust.get("center_x_offset_from_fc_m", 0.0)),
        "center_z_m": float(fc_pos[2] if len(fc_pos) > 2 else 0.0) + float(exhaust.get("center_z_offset_from_fc_m", 0.0)),
        "fan_shroud_depth_y_m": float(exhaust.get("fan_shroud_depth_y_m", req_out_gap)),
        "louvre_angle_deg": float(exhaust.get("louvre_angle_deg", 15.0)),
        "lip_radius_m": float(exhaust.get("lip_radius_m", 0.002)),
    })

    warning = (
        "Supplier fan airflow/static-pressure data are not available in the supplied IE-SOAR documents; "
        "airflow PASS/FAIL therefore uses explicit engineering-assumption fan capacity and must be replaced by supplier or bench data."
    )

    return FuelCellCoolingResult(
        enabled=enabled,
        architecture=arch,
        fan_flow_direction=direction,
        fan_side=fan_side,
        inlet_side=inlet_side,
        exhaust_side=exhaust_side,
        fuel_cell_net_power_W=fc_net,
        efficiency_screen=eta,
        waste_heat_nominal_W=waste,
        waste_heat_design_W=waste_design,
        allowable_air_temperature_rise_C=dT,
        required_air_mass_flow_kg_s=mdot,
        required_air_volume_flow_m3_s=q_req,
        required_airflow_cfm=q_req * CFM_PER_M3_S,
        assumed_module_zero_external_pressure_airflow_cfm=q0_cfm,
        assumed_external_shutoff_pressure_Pa=shut,
        inlet_free_area_m2=ig["free_area_m2"],
        exhaust_free_area_m2=eg["free_area_m2"],
        minimum_flow_area_m2=a_min,
        recommended_minimum_free_area_m2=a_rec,
        inlet_velocity_at_required_flow_mps=inlet_v,
        exhaust_velocity_at_required_flow_mps=exhaust_v,
        total_external_loss_coefficient_K=K,
        estimated_external_pressure_drop_at_required_flow_Pa=dp_req,
        estimated_operating_airflow_m3_s=q_oper,
        estimated_operating_airflow_cfm=q_oper * CFM_PER_M3_S,
        airflow_margin_ratio=flow_margin,
        airflow_screen_pass=airflow_pass,
        fuselage_width_m=fuse_w,
        fuselage_wall_keepout_m=wall,
        fuel_cell_width_m=fc_w,
        inlet_side_plenum_gap_m=gap_in,
        exhaust_side_plenum_gap_m=gap_out,
        required_inlet_side_plenum_gap_m=req_in_gap,
        required_exhaust_side_plenum_gap_m=req_out_gap,
        minimum_fuselage_width_for_plenums_m=min_fuse_w,
        plenum_geometry_pass=plenum_pass,
        inlet_panel_fits_fc_face=inlet_fits,
        exhaust_panel_fits_fc_face=exhaust_fits,
        side_opening_geometry_pass=opening_pass,
        overall_geometry_pass=geom_pass,
        cooling_openings_CdA_m2=cooling_cda,
        cooling_drag_at_cruise_N=drag,
        estimated_cruise_ram_pressure_Pa=ram,
        credited_cruise_ram_pressure_Pa=credited_ram,
        inlet_geometry=inlet_geom,
        exhaust_geometry=exhaust_geom,
        fan_geometry=fan_geom,
        evidence_status="engineering_assumption",
        warning=warning,
    )
