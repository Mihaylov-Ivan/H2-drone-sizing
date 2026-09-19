from __future__ import annotations

"""Flight-envelope utilities for the H2AIR inspection aircraft.

The operational V-n / stall screens in this module are deliberately separate
from the conservative structural load cases.  They are screening-fidelity
engineering checks, not certification evidence.
"""

from math import cos, radians, sqrt
from pathlib import Path
from typing import Any, Dict, Mapping
import csv
import json

from .aero_propulsion import cruise_drag, stall_speed_mps, lift_system_sizing

G = 9.80665
EPS = 1e-12


def direct_stall_speed_mps(*, mass_kg: float, rho_kg_m3: float, wing_area_m2: float, cl_abs: float) -> float:
    if mass_kg <= 0 or rho_kg_m3 <= 0 or wing_area_m2 <= 0 or cl_abs <= 0:
        raise ValueError("stall-speed inputs must be positive")
    return sqrt(2.0 * mass_kg * G / (rho_kg_m3 * wing_area_m2 * cl_abs))


def required_area_for_positive_manoeuvre(
    *,
    mass_kg: float,
    rho_kg_m3: float,
    speed_mps: float,
    cl_max: float,
    load_factor: float,
    max_cl_utilization: float,
) -> float:
    """Wing area needed to carry +n without using more than the allowed CLmax."""
    if speed_mps <= 0 or load_factor <= 0:
        raise ValueError("speed and positive load factor must be > 0")
    if not (0.0 < max_cl_utilization <= 1.0):
        raise ValueError("max_cl_utilization must be in (0,1]")
    q = 0.5 * rho_kg_m3 * speed_mps * speed_mps
    return load_factor * mass_kg * G / max(q * cl_max * max_cl_utilization, EPS)


def required_area_for_vertical_gust(
    *,
    mass_kg: float,
    rho_kg_m3: float,
    speed_mps: float,
    cl_max: float,
    cl_alpha_per_rad: float,
    vertical_gust_mps: float,
    max_cl_utilization: float,
) -> float:
    """Conservative sharp-edge vertical-gust wing-area requirement.

    Uses ΔCL = a * U/V with no gust-alleviation credit.  This intentionally
    screens the instantaneous aerodynamic load separately from the slower
    closed-loop 6-DOF controller response.
    """
    if speed_mps <= 0 or mass_kg <= 0 or rho_kg_m3 <= 0 or cl_max <= 0 or cl_alpha_per_rad <= 0:
        raise ValueError("vertical-gust sizing inputs must be positive")
    if vertical_gust_mps < 0:
        raise ValueError("vertical_gust_mps must be >= 0")
    if not (0.0 < max_cl_utilization <= 1.0):
        raise ValueError("max_cl_utilization must be in (0,1]")
    q = 0.5 * rho_kg_m3 * speed_mps * speed_mps
    gust_pressure_term = 0.5 * rho_kg_m3 * speed_mps * cl_alpha_per_rad * vertical_gust_mps
    available_per_area = max_cl_utilization * q * cl_max - gust_pressure_term
    if available_per_area <= EPS:
        raise ValueError("configured vertical gust leaves no positive CL margin at this speed")
    return mass_kg * G / available_per_area


def evaluate_vertical_gust_stall(
    *,
    mass_kg: float,
    rho_kg_m3: float,
    wing_area_m2: float,
    speed_mps: float,
    cl_max: float,
    cl_alpha_per_rad: float,
    vertical_gust_mps: float,
    max_cl_utilization: float,
) -> Dict[str, Any]:
    """Conservative no-alleviation sharp-edge upward-gust stall screen."""
    W = mass_kg * G
    q = 0.5 * rho_kg_m3 * speed_mps * speed_mps
    delta_cl = cl_alpha_per_rad * vertical_gust_mps / max(speed_mps, EPS)
    delta_lift_N = q * wing_area_m2 * delta_cl
    delta_n = delta_lift_N / max(W, EPS)
    n_gust = 1.0 + delta_n
    n_stall = q * wing_area_m2 * cl_max / max(W, EPS)
    utilization = n_gust / max(n_stall, EPS)
    required_area = required_area_for_vertical_gust(
        mass_kg=mass_kg,
        rho_kg_m3=rho_kg_m3,
        speed_mps=speed_mps,
        cl_max=cl_max,
        cl_alpha_per_rad=cl_alpha_per_rad,
        vertical_gust_mps=vertical_gust_mps,
        max_cl_utilization=max_cl_utilization,
    )
    return {
        "model": "sharp_edge_no_alleviation",
        "mass_kg": float(mass_kg),
        "rho_kg_m3": float(rho_kg_m3),
        "speed_mps": float(speed_mps),
        "wing_area_m2": float(wing_area_m2),
        "cl_max": float(cl_max),
        "cl_alpha_per_rad": float(cl_alpha_per_rad),
        "vertical_gust_mps": float(vertical_gust_mps),
        "delta_cl": float(delta_cl),
        "incremental_load_factor_g": float(delta_n),
        "total_positive_load_factor_g": float(n_gust),
        "positive_stall_load_factor_g": float(n_stall),
        "clmax_utilization": float(utilization),
        "maximum_cl_utilization": float(max_cl_utilization),
        "required_wing_area_m2": float(required_area),
        "pass": bool(utilization <= max_cl_utilization + 1e-12),
        "note": "Sharp-edge vertical-gust screen with no gust-alleviation credit; conservative screening assumption, not a certification gust analysis.",
    }


def evaluate_vn_envelope(
    *,
    mass_kg: float,
    rho_kg_m3: float,
    wing_area_m2: float,
    speed_mps: float,
    cl_max: float,
    cl_min: float,
    positive_limit_g: float,
    negative_limit_g: float,
    max_cl_utilization: float,
    maximum_bank_deg: float,
) -> Dict[str, Any]:
    """Compact accelerated-stall / V-n screen at the mission cruise speed."""
    if positive_limit_g <= 1.0:
        raise ValueError("positive_limit_g must be > 1")
    if negative_limit_g >= 0.0:
        raise ValueError("negative_limit_g must be < 0")
    if cl_min >= 0.0:
        raise ValueError("cl_min must be negative for the negative-g screen")
    if not (0.0 < max_cl_utilization <= 1.0):
        raise ValueError("max_cl_utilization must be in (0,1]")
    if not (0.0 <= maximum_bank_deg < 90.0):
        raise ValueError("maximum_bank_deg must be in [0,90)")

    W = mass_kg * G
    q = 0.5 * rho_kg_m3 * speed_mps * speed_mps
    n_pos_stall = q * wing_area_m2 * cl_max / max(W, EPS)
    n_neg_stall = q * wing_area_m2 * cl_min / max(W, EPS)
    pos_util = positive_limit_g / max(n_pos_stall, EPS)
    neg_util = abs(negative_limit_g) / max(abs(n_neg_stall), EPS)
    bank_n = 1.0 / max(cos(radians(maximum_bank_deg)), EPS)

    vs_pos = direct_stall_speed_mps(
        mass_kg=mass_kg, rho_kg_m3=rho_kg_m3,
        wing_area_m2=wing_area_m2, cl_abs=cl_max,
    )
    vs_neg = direct_stall_speed_mps(
        mass_kg=mass_kg, rho_kg_m3=rho_kg_m3,
        wing_area_m2=wing_area_m2, cl_abs=abs(cl_min),
    )

    return {
        "mass_kg": float(mass_kg),
        "rho_kg_m3": float(rho_kg_m3),
        "speed_mps": float(speed_mps),
        "wing_area_m2": float(wing_area_m2),
        "cl_max": float(cl_max),
        "cl_min": float(cl_min),
        "maximum_bank_deg": float(maximum_bank_deg),
        "bank_load_factor_g": float(bank_n),
        "positive_operational_limit_g": float(positive_limit_g),
        "negative_operational_limit_g": float(negative_limit_g),
        "maximum_cl_utilization": float(max_cl_utilization),
        "positive_1g_stall_speed_mps": float(vs_pos),
        "negative_1g_stall_speed_mps": float(vs_neg),
        "positive_stall_load_factor_at_speed_g": float(n_pos_stall),
        "negative_stall_load_factor_at_speed_g": float(n_neg_stall),
        "positive_manoeuvre_cl_utilization": float(pos_util),
        "negative_manoeuvre_cl_utilization": float(neg_util),
        "minimum_speed_for_positive_limit_with_margin_mps": float(vs_pos * sqrt(positive_limit_g / max_cl_utilization)),
        "minimum_speed_for_negative_limit_with_margin_mps": float(vs_neg * sqrt(abs(negative_limit_g) / max_cl_utilization)),
        "bank_within_positive_limit": bool(bank_n <= positive_limit_g + 1e-12),
        "positive_manoeuvre_stall_pass": bool(pos_util <= max_cl_utilization + 1e-12),
        "negative_manoeuvre_stall_pass": bool(neg_util <= max_cl_utilization + 1e-12),
    }


def build_flight_envelope(cfg:Mapping[str,Any],*,v_min:float=10.0,v_max:float=35.0,dv:float=1.0)->Dict[str,Any]:
    # Local import avoids a circular dependency: pipeline imports the pure V-n
    # helpers above, while this optional report helper uses pipeline closure.
    from .pipeline import _components_with_power, _structural_closure, _sync_design

    base=_components_with_power(cfg);_,mass,_,_= _structural_closure(cfg,base);a,_,_,_=_sync_design(cfg,mass);d=cfg['design'];vs=stall_speed_mps(a,mass.takeoff.total_mass_kg,float(d['wing_area_m2']))
    rows=[];v=v_min
    gate=float(cfg['power_system']['requirements']['representative_cruise_power_gate_W'])
    while v<=v_max+1e-9:
        dr=cruise_drag(a,mass_kg=mass.takeoff.total_mass_kg,speed_mps=v,lift_prop_diameter_in=float(d['lift_propeller_diameter_in']),lift_prop_state=str(d['cruise_lift_propeller_state']),wing_area_m2=float(d['wing_area_m2']),wing_span_m=float(d['wing_span_m']))
        ratio=v/max(vs,1e-9);cls='VALID' if ratio>=1.30 and dr.total_cruise_bus_power_W<=gate else ('STALL_MARGIN_FAIL' if ratio<1.30 else 'POWER_GATE_FAIL')
        rows.append({'speed_mps':v,'stall_speed_mps':vs,'V_over_Vstall':ratio,'drag_N':dr.total_drag_N,'bus_power_W':dr.total_cruise_bus_power_W,'classification':cls});v+=dv
    lift=lift_system_sizing(a,mass_kg=mass.takeoff.total_mass_kg,prop_diameter_in=float(d['lift_propeller_diameter_in']))
    return {'mass_kg':mass.takeoff.total_mass_kg,'hover':{'speed_mps':0.0,'design_thrust_per_motor_N':lift.design_thrust_per_motor_N,'design_total_electrical_power_W':lift.total_design_power_with_margin_W},'rows':rows}


def write_flight_envelope(result:Mapping[str,Any],out_dir:str|Path)->None:
    out=Path(out_dir);out.mkdir(parents=True,exist_ok=True);(out/'flight_envelope.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    rows=result['rows']
    if rows:
        with (out/'flight_envelope.csv').open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
