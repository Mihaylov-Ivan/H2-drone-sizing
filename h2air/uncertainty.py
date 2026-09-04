from __future__ import annotations

"""Uncertainty propagation for H2AIR screening design studies.

Distributions are deliberately explicit engineering assumptions.  They are not
claimed statistical descriptions of supplier hardware.  Replace the default
uncertainty register with measured/supplier distributions as evidence matures.
"""

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence
import csv, json
import numpy as np
from scipy.stats import qmc, norm

from .optimization import screening_evaluate

DEFAULT_UNCERTAINTY = {
    "non_lift_parasite_drag_factor": {"mean":1.0,"sigma":0.08,"min":0.80,"max":1.20,"status":"engineering_assumption"},
    "cruise_propulsive_efficiency_factor": {"mean":1.0,"sigma":0.05,"min":0.85,"max":1.12,"status":"engineering_assumption"},
    "reference_stall_speed_factor": {"mean":1.0,"sigma":0.03,"min":0.92,"max":1.08,"status":"engineering_assumption"},
    "battery_mass_factor": {"mean":1.0,"sigma":0.12,"min":0.75,"max":1.30,"status":"engineering_assumption_placeholder_hardware"},
    "cylinder_mass_factor": {"mean":1.0,"sigma":0.12,"min":0.75,"max":1.30,"status":"engineering_assumption_placeholder_hardware"},
    "fuel_cell_mass_factor": {"mean":1.0,"sigma":0.04,"min":0.90,"max":1.10,"status":"supplier_reference_tolerance_screen"},
    "h2_net_energy_factor": {"mean":1.0,"sigma":0.05,"min":0.88,"max":1.12,"status":"engineering_assumption"},
}


def _clipped_normal(u: float, spec: Mapping[str,float]) -> float:
    z=norm.ppf(min(max(u,1e-9),1-1e-9))
    return float(np.clip(float(spec["mean"])+float(spec["sigma"])*z,float(spec["min"]),float(spec["max"])))


def _perturb(base: Mapping[str,Any], factors: Mapping[str,float]) -> Dict[str,Any]:
    c=deepcopy(dict(base))
    ref=c["aero_propulsion"]["reference_calibration"]
    # Scale the non-lift-prop calibrated parasite remainder while leaving the
    # lift-prop state model independent.
    stopped=float(ref["reference_stopped_lift_prop_drag_N"])
    other=float(ref["reference_parasite_drag_N"])-stopped
    fd=float(factors["non_lift_parasite_drag_factor"])
    ref["reference_parasite_drag_N"]=stopped+fd*other
    # Wing profile drag is small but kept consistent with the same factor.
    induced=float(ref["reference_induced_drag_N"]); main=float(ref["reference_main_wing_drag_N"])
    ref["reference_main_wing_drag_N"]=induced+fd*max(main-induced,0.0)
    c["aero_propulsion"]["aircraft"]["cruise_propulsive_efficiency_reference"] *= float(factors["cruise_propulsive_efficiency_factor"])
    ref["reference_wing_stall_speed_mps"] *= float(factors["reference_stall_speed_factor"])

    b=c["power_system"]["battery"]
    baseline_batt=float(b["mass_kg"] if b.get("mass_kg") is not None else 0.75)
    b["mass_kg"]=baseline_batt*float(factors["battery_mass_factor"])
    h=c["power_system"]["hydrogen"]
    baseline_cyl=float(h["cylinder_mass_kg"] if h.get("cylinder_mass_kg") is not None else 1.10)
    h["cylinder_mass_kg"]=baseline_cyl*float(factors["cylinder_mass_factor"])
    fc=c["power_system"]["fuel_cell"]
    fc["module_mass_kg"]=float(fc["module_mass_kg"])*float(factors["fuel_cell_mass_factor"])
    h["reference_net_energy_Wh"]=float(h["reference_net_energy_Wh"])*float(factors["h2_net_energy_factor"])
    return c


def run_uncertainty(base_cfg: Mapping[str,Any], design_values: Mapping[str,float], *, samples:int=32, seed:int=77, register:Mapping[str,Any]|None=None)->Dict[str,Any]:
    reg=dict(register or DEFAULT_UNCERTAINTY); names=list(reg)
    sampler=qmc.LatinHypercube(d=len(names),seed=seed); U=sampler.random(samples)
    rows=[]
    for i,urow in enumerate(U):
        factors={name:_clipped_normal(float(u),reg[name]) for name,u in zip(names,urow)}
        cfg=_perturb(base_cfg,factors)
        c=screening_evaluate(cfg,design_values,index=i)
        row={"sample":i,"feasible":c.feasible,"score":c.score,**factors,**c.metrics,"violations":";".join(c.violations)}
        rows.append(row)
    def pct(key,p):
        vals=np.array([float(r[key]) for r in rows if np.isfinite(float(r[key]))]); return float(np.percentile(vals,p))
    summary={
        "samples":samples,"seed":seed,"screening_pass_probability":sum(bool(r["feasible"]) for r in rows)/max(samples,1),
        "post_structure_mass_kg":{"p05":pct("post_structure_mass_kg",5),"p50":pct("post_structure_mass_kg",50),"p95":pct("post_structure_mass_kg",95)},
        "static_margin":{"p05":pct("static_margin",5),"p50":pct("static_margin",50),"p95":pct("static_margin",95)},
        "cruise_bus_power_W":{"p05":pct("cruise_bus_power_W",5),"p50":pct("cruise_bus_power_W",50),"p95":pct("cruise_bus_power_W",95)},
        "mission_energy_Wh":{"p05":pct("mission_energy_Wh",5),"p50":pct("mission_energy_Wh",50),"p95":pct("mission_energy_Wh",95)},
        "max_motor_15kg_N":{"p05":pct("max_motor_15kg_N",5),"p50":pct("max_motor_15kg_N",50),"p95":pct("max_motor_15kg_N",95)},
        "failure_counts":{},
    }
    for r in rows:
        for v in str(r["violations"]).split(";"):
            if v:summary["failure_counts"][v]=summary["failure_counts"].get(v,0)+1
    return {"method":"Latin-hypercube propagation","uncertainty_register":reg,"summary":summary,"samples":rows}


def write_uncertainty_outputs(result:Mapping[str,Any],out_dir:str|Path)->None:
    out=Path(out_dir);out.mkdir(parents=True,exist_ok=True)
    (out/"uncertainty_results.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    rows=result["samples"]
    if rows:
        keys=[]
        for r in rows:
            for k in r:
                if k not in keys:keys.append(k)
        with (out/"uncertainty_samples.csv").open("w",newline="",encoding="utf-8") as f:
            w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
