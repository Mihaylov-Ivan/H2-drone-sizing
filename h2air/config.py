from __future__ import annotations
from copy import deepcopy
from dataclasses import fields
from pathlib import Path
from typing import Any, Dict, Mapping, Type, TypeVar
import json

from .hybrid_power import BatteryConfig, EMSConfig, FuelCellConfig, HydrogenConfig, HybridPowerConfig, RequirementConfig
from .design_coupling import resolve_design_couplings
from .user_inputs import load_user_input_config

T=TypeVar("T")

def _strict_dataclass(cls: Type[T], raw: Mapping[str, Any]) -> T:
    valid={f.name for f in fields(cls)}
    unknown=sorted(set(raw)-valid)
    if unknown: raise ValueError(f"Unknown {cls.__name__} key(s): {', '.join(unknown)}")
    return cls(**dict(raw))

def hybrid_config_from_mapping(raw: Mapping[str,Any]) -> HybridPowerConfig:
    allowed={"fuel_cell","hydrogen","battery","ems","requirements","model_version","metadata"}
    unknown=sorted(set(raw)-allowed)
    if unknown: raise ValueError("Unknown hybrid-power config key(s): "+", ".join(unknown))
    # Hydrogen may carry datasheet metadata (model, thread, …); keep only HydrogenConfig fields.
    h_raw = dict(raw.get("hydrogen", {}) or {})
    h_valid = {f.name for f in fields(HydrogenConfig)}
    h_cfg = {k: v for k, v in h_raw.items() if k in h_valid}
    return HybridPowerConfig(
        fuel_cell=_strict_dataclass(FuelCellConfig,raw.get("fuel_cell",{})),
        hydrogen=_strict_dataclass(HydrogenConfig,h_cfg),
        battery=_strict_dataclass(BatteryConfig,raw.get("battery",{})),
        ems=_strict_dataclass(EMSConfig,raw.get("ems",{})),
        requirements=_strict_dataclass(RequirementConfig,raw.get("requirements",{})),
        model_version=str(raw.get("model_version","H2AIR-integrated-power")),
    )

def load_hybrid_power_config(path: str|Path) -> HybridPowerConfig:
    return hybrid_config_from_mapping(json.loads(Path(path).read_text(encoding="utf-8")))

def load_model_config(path: str|Path) -> Dict[str,Any]:
    path=Path(path)
    raw=json.loads(path.read_text(encoding="utf-8"))
    if raw.get("schema") in {"H2AIR-user-inputs-v3", "H2AIR-user-inputs-v4"}:
        cfg=load_user_input_config(path)
    else:
        cfg=raw
    required={"model","design","power_system","geometry_mass","aero_propulsion","transition_mission","sixdof_control","structures"}
    missing=sorted(required-set(cfg))
    if missing: raise ValueError("Integrated config missing: "+", ".join(missing))
    d=cfg["design"]
    for key in ("payload_mass_kg","mtow_ceiling_kg","cruise_speed_mps","wing_area_m2","wing_span_m","wing_mac_m","wing_CLmax","lift_propeller_diameter_in"):
        if float(d[key])<=0: raise ValueError(f"design.{key} must be > 0")
    return resolve_design_couplings(cfg)

def clone_section(cfg: Mapping[str,Any], name: str) -> Dict[str,Any]:
    return deepcopy(cfg[name])
