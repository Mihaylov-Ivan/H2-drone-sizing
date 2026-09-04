from __future__ import annotations
"""Model-to-test correlation ledger and weighted scale-factor fitter.

This module never manufactures test data.  Empty or unsupported datasets return
NOT_EVALUATED.  The generated CSV template is intended for thrust-stand, cruise
power, mass/CG and transition-test measurements as they become available.
"""
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping
import csv, json, math

SUPPORTED={"cruise_bus_power_W":"aero_power_scale","hover_power_W":"lift_power_scale","transition_duration_s":"transition_time_scale","post_structure_mass_kg":"structural_mass_scale"}

def load_measurements(path:str|Path):
    p=Path(path)
    if not p.exists():return []
    with p.open(newline="",encoding="utf-8") as f:return list(csv.DictReader(f))

def fit_scale_factors(rows:Iterable[Mapping[str,Any]])->Dict[str,Any]:
    groups={}
    for r in rows:
        q=str(r.get("quantity",""));
        if q not in SUPPORTED:continue
        try:m=float(r["measured"]);p=float(r["predicted"]);sigma=float(r.get("sigma") or 1.0)
        except (TypeError,ValueError):continue
        if p<=0 or sigma<=0:continue
        groups.setdefault(SUPPORTED[q],[]).append((m/p,1.0/(sigma*sigma)))
    factors={}
    for name,vals in groups.items():
        sw=sum(w for _,w in vals);factors[name]={"factor":sum(v*w for v,w in vals)/sw,"n":len(vals),"status":"measured_correlation"}
    return {"status":"CALIBRATED" if factors else "NOT_EVALUATED","factors":factors,"note":"Scale factors are empirical correlations only; inspect residuals and test applicability before using outside the measured envelope."}

def write_template(path:str|Path)->None:
    p=Path(path);p.parent.mkdir(parents=True,exist_ok=True)
    fields=["quantity","measured","predicted","sigma","unit","test_id","configuration_hash","source_note"]
    with p.open("w",newline="",encoding="utf-8") as f:csv.DictWriter(f,fieldnames=fields).writeheader()
