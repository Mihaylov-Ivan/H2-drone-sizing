from __future__ import annotations

import itertools
import json
import math
import os
import sys
import traceback
import importlib
from dataclasses import fields, is_dataclass, asdict, MISSING
from datetime import datetime
from io import BytesIO
from typing import Any, Dict, List, Optional, Tuple, Union, get_args, get_origin

import numpy as np
import pandas as pd
import streamlit as st
import matplotlib.pyplot as plt

import design_inputs
import script


# ============================================================
# App configuration
# ============================================================
APP_TITLE = "UAV DoE / Optimisation App"
DEFAULT_EXPORTED_CONFIG_NAME = "uav_doe_config"
DEFAULT_EXPORTED_RESULTS_NAME = "uav_doe_results"

# Inputs that are usually derived inside the model or from other primary inputs.
# Hidden by default, but you can show them from the UI.
DERIVED_INPUT_FIELDS = {
    "dive_speed",
    "battery_length",
    "battery_width",
    "battery_height",
    "fuselage_OR",
    "fuselage_cs_area",
    "fuselage_cs_perimeter",
    "fuselage_shell_area_moment_of_inertia",
    "fuselage_wetted_area",
    "fuselage_body_length",
}

# These get refreshed from the main driving geometry if enabled.
LINKED_REFRESH_TARGETS = {
    "fuselage_longeron_diameter",
    "fuselage_body_section_A_length",
    "fuselage_body_section_B_length",
    "fuselage_body_section_C_length",
    "fuselage_longeron_lenght",
    "fuselage_B_front_bulkhead_location",
    "fuselage_B_rear_bulkhead_location",
}

PROJECT_MODULES_TO_RELOAD = [
    "constants",
    "design_inputs",
    "geometry",
    "aerodynamics",
    "structures",
    "avionics",
    "propulsion_system",
    "convergence",
    "results",
    "reporting",
    "script",
]

DEFAULT_CONSTRAINTS_JSON = json.dumps(
    [
        {
            "metric": "aircraft.static_margin_percent_MAC",
            "op": ">=",
            "value": 5.0,
        },
        {
            "metric": "wing.geometry.span_m",
            "op": "<=",
            "value": 2.5,
        },
    ],
    indent=2,
)

DEFAULT_LINKED_REFRESH_JSON = json.dumps(
    {
        "enabled": True,
        "notes": "Keeps a few DesignInputs fields consistent when core geometry inputs are changed.",
        "targets": sorted(LINKED_REFRESH_TARGETS),
    },
    indent=2,
)

PLOT_DPI = 120


# ============================================================
# Generic helpers
# ============================================================
def normalize_annotation(tp: Any) -> Any:
    origin = get_origin(tp)
    if origin is Union:
        args = [a for a in get_args(tp) if a is not type(None)]
        if len(args) == 1:
            return args[0]
    return tp


def is_bool_type(tp: Any) -> bool:
    return normalize_annotation(tp) is bool


def is_int_type(tp: Any) -> bool:
    return normalize_annotation(tp) is int


def is_float_type(tp: Any) -> bool:
    return normalize_annotation(tp) is float


def is_str_type(tp: Any) -> bool:
    return normalize_annotation(tp) is str


def is_dict_like_type(tp: Any) -> bool:
    tp = normalize_annotation(tp)
    origin = get_origin(tp)
    return tp is dict or origin is dict


def is_list_like_type(tp: Any) -> bool:
    tp = normalize_annotation(tp)
    origin = get_origin(tp)
    return tp is list or origin is list or tp is tuple or origin is tuple


def is_scalar_annotation(tp: Any) -> bool:
    return is_bool_type(tp) or is_int_type(tp) or is_float_type(tp) or is_str_type(tp)


def is_scalar_value(v: Any) -> bool:
    return isinstance(v, (str, int, float, bool, np.integer, np.floating, np.bool_)) or v is None


def is_numeric_value(v: Any) -> bool:
    return isinstance(v, (int, float, np.integer, np.floating)) and not isinstance(v, bool)


def default_value_for_field(f) -> Any:
    if f.default is not MISSING:
        return f.default
    if f.default_factory is not MISSING:  # type: ignore[attr-defined]
        return f.default_factory()  # type: ignore[misc]
    return None


def to_pretty_json(value: Any) -> str:
    return json.dumps(value, indent=2, ensure_ascii=False)


def parse_json_or_raise(text: str, label: str) -> Any:
    try:
        return json.loads(text)
    except Exception as e:
        raise ValueError(f"Invalid JSON for {label}: {e}") from e


def parse_comma_list(text: str) -> List[str]:
    return [x.strip() for x in text.split(",") if x.strip()]


def infer_group(name: str) -> str:
    prefixes = [
        "payload",
        "wing",
        "tail",
        "vertical_tail",
        "fuselage",
        "battery",
        "propulsion",
        "prop",
        "motor",
        "servo",
        "elevator",
        "rudder",
        "cg",
        "vtol",
        "transition",
        "mission",
        "autopilot",
        "esc",
        "gps",
        "airspeed",
        "rx",
        "tel",
        "bolt",
        "threaded_insert",
        "clamp",
        "hardcode",
        "filament",
        "carbon_fiber",
        "printed",
        "snap_fit",
    ]
    for p in prefixes:
        if name.startswith(p):
            return p
    return name.split("_")[0]


# ============================================================
# DesignInputs reflection
# ============================================================
def get_design_field_specs(include_derived: bool = False) -> List[Dict[str, Any]]:
    specs: List[Dict[str, Any]] = []
    annotations = getattr(design_inputs.DesignInputs, "__annotations__", {})

    base_instance = design_inputs.DesignInputs()
    for f in fields(design_inputs.DesignInputs):
        if not f.init:
            continue
        if (not include_derived) and f.name in DERIVED_INPUT_FIELDS:
            continue

        tp = annotations.get(f.name, Any)
        value = getattr(base_instance, f.name)
        specs.append(
            {
                "name": f.name,
                "type": tp,
                "default": value,
                "group": infer_group(f.name),
                "is_scalar": is_scalar_annotation(tp) or is_scalar_value(value),
                "is_numeric": is_int_type(tp) or is_float_type(tp) or is_numeric_value(value),
                "is_bool": is_bool_type(tp) or isinstance(value, bool),
                "is_json": is_dict_like_type(tp) or is_list_like_type(tp) or isinstance(value, (dict, list, tuple)),
                "is_string": is_str_type(tp) or isinstance(value, str),
            }
        )
    specs.sort(key=lambda d: (d["group"], d["name"]))
    return specs


def get_base_input_values(specs: List[Dict[str, Any]]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for spec in specs:
        name = spec["name"]
        widget_key = f"base__{name}"
        default = spec["default"]

        if widget_key not in st.session_state:
            out[name] = default
            continue

        raw = st.session_state[widget_key]
        tp = spec["type"]
        if spec["is_bool"]:
            out[name] = bool(raw)
        elif is_int_type(tp):
            out[name] = int(raw)
        elif is_float_type(tp):
            out[name] = float(raw)
        elif spec["is_json"]:
            out[name] = parse_json_or_raise(raw, name)
        elif spec["is_string"]:
            out[name] = str(raw)
        else:
            out[name] = raw
    return out


def get_varyable_specs(specs: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    varyable: List[Dict[str, Any]] = []
    for spec in specs:
        if spec["is_json"]:
            continue
        varyable.append(spec)
    return varyable


# ============================================================
# Model/result flattening
# ============================================================
def make_json_safe(obj: Any) -> Any:
    if is_dataclass(obj):
        return make_json_safe(asdict(obj))
    if hasattr(obj, "to_dict") and callable(getattr(obj, "to_dict")):
        try:
            return make_json_safe(obj.to_dict())
        except Exception:
            pass
    if isinstance(obj, dict):
        return {str(k): make_json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [make_json_safe(v) for v in obj]
    if isinstance(obj, np.generic):
        return obj.item()
    return obj


def flatten_any(obj: Any, prefix: str = "") -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    obj = make_json_safe(obj)

    if isinstance(obj, dict):
        for k, v in obj.items():
            key = f"{prefix}.{k}" if prefix else str(k)
            out.update(flatten_any(v, key))
        return out

    if isinstance(obj, list):
        # Lists are kept only if scalar and short; otherwise skipped from metric catalogue.
        if all(is_scalar_value(v) for v in obj) and len(obj) <= 20:
            out[prefix] = json.dumps(obj)
        return out

    if prefix:
        out[prefix] = obj
    return out


def excel_scalar(v: Any) -> bool:
    return v is None or isinstance(v, (str, int, float, bool, np.integer, np.floating, np.bool_))


def metric_catalog_from_probe(result_obj: Any) -> List[str]:
    flat = flatten_any(result_obj)
    keys = [k for k, v in flat.items() if excel_scalar(v)]
    keys.sort()
    return keys


# ============================================================
# Optional linked input refresh
# ============================================================
def refresh_linked_inputs(inp: Any, locked_fields: set[str]) -> Any:
    def unlocked(name: str) -> bool:
        return name not in locked_fields

    # Keep this intentionally small and transparent.
    try:
        if unlocked("fuselage_longeron_diameter"):
            inp.fuselage_longeron_diameter = 0.008 + float(inp.transition_fit_clearance)

        body_mid = float(inp.fuselage_length) - float(inp.fuselage_nose_length) - float(inp.fuselage_tail_length)
        if unlocked("fuselage_body_section_B_length"):
            inp.fuselage_body_section_B_length = float(inp.c_root) * 1.1

        b_len = float(inp.fuselage_body_section_B_length)
        rem = body_mid - b_len

        if unlocked("fuselage_body_section_A_length"):
            inp.fuselage_body_section_A_length = rem * 2.25 / 3.0 - 0.03
        if unlocked("fuselage_body_section_C_length"):
            inp.fuselage_body_section_C_length = rem * 0.75 / 3.0 + 0.03

        if unlocked("fuselage_longeron_lenght"):
            inp.fuselage_longeron_lenght = (
                float(inp.fuselage_nose_longeron_length)
                + float(inp.fuselage_tail_longeron_length)
                + float(inp.fuselage_body_section_A_length)
                + float(inp.fuselage_body_section_B_length)
                + float(inp.fuselage_body_section_C_length)
            )

        if unlocked("fuselage_B_front_bulkhead_location"):
            inp.fuselage_B_front_bulkhead_location = float(inp.fuselage_body_section_B_length) / 4.0
        if unlocked("fuselage_B_rear_bulkhead_location"):
            inp.fuselage_B_rear_bulkhead_location = float(inp.fuselage_body_section_B_length) / 3.2

        # Re-run post-init to refresh derived quantities and CG defaults.
        if hasattr(inp, "__post_init__"):
            inp.__post_init__()
    except Exception:
        # Never hard-fail here; the main model run will still validate the configuration.
        pass
    return inp


# ============================================================
# Constraints
# ============================================================
def parse_constraints_json(raw: str) -> List[Dict[str, Any]]:
    if not raw.strip():
        return []
    data = parse_json_or_raise(raw, "constraints")
    if not isinstance(data, list):
        raise ValueError("Constraints JSON must be a list of objects.")

    cleaned: List[Dict[str, Any]] = []
    valid_ops = {">=", "<=", ">", "<", "==", "between"}
    for i, item in enumerate(data, start=1):
        if not isinstance(item, dict):
            raise ValueError(f"Constraint #{i} must be an object.")
        metric = item.get("metric")
        op = item.get("op")
        if not metric or op not in valid_ops:
            raise ValueError(f"Constraint #{i} must include metric and valid op {sorted(valid_ops)}.")
        if op == "between":
            if "min" not in item or "max" not in item:
                raise ValueError(f"Constraint #{i} with op='between' must contain min and max.")
            cleaned.append({"metric": str(metric), "op": op, "min": float(item["min"]), "max": float(item["max"])})
        else:
            if "value" not in item:
                raise ValueError(f"Constraint #{i} must contain value.")
            cleaned.append({"metric": str(metric), "op": op, "value": float(item["value"])})
    return cleaned


def evaluate_constraints(flat: Dict[str, Any], constraints: List[Dict[str, Any]]) -> Tuple[bool, List[str]]:
    messages: List[str] = []
    feasible = True

    for c in constraints:
        metric = c["metric"]
        if metric not in flat:
            feasible = False
            messages.append(f"missing metric: {metric}")
            continue

        try:
            x = float(flat[metric])
        except Exception:
            feasible = False
            messages.append(f"non-numeric metric: {metric}")
            continue

        op = c["op"]
        ok = True
        if op == ">=":
            ok = x >= float(c["value"])
            if not ok:
                messages.append(f"{metric}={x:.6g} < {float(c['value']):.6g}")
        elif op == "<=":
            ok = x <= float(c["value"])
            if not ok:
                messages.append(f"{metric}={x:.6g} > {float(c['value']):.6g}")
        elif op == ">":
            ok = x > float(c["value"])
            if not ok:
                messages.append(f"{metric}={x:.6g} <= {float(c['value']):.6g}")
        elif op == "<":
            ok = x < float(c["value"])
            if not ok:
                messages.append(f"{metric}={x:.6g} >= {float(c['value']):.6g}")
        elif op == "==":
            ok = math.isclose(x, float(c["value"]), rel_tol=1e-9, abs_tol=1e-9)
            if not ok:
                messages.append(f"{metric}={x:.6g} != {float(c['value']):.6g}")
        elif op == "between":
            lo = float(c["min"])
            hi = float(c["max"])
            ok = lo <= x <= hi
            if not ok:
                messages.append(f"{metric}={x:.6g} not in [{lo:.6g}, {hi:.6g}]")
        else:
            ok = False
            messages.append(f"unsupported operator for {metric}: {op}")

        feasible = feasible and ok

    return feasible, messages


# ============================================================
# Config IO
# ============================================================
def build_config_dict(
    base_inputs: Dict[str, Any],
    vary: Dict[str, Dict[str, Any]],
    objective_metric: str,
    objective_direction: str,
    constraints: List[Dict[str, Any]],
    include_derived_inputs: bool,
    auto_refresh_linked_inputs: bool,
    reload_modules_before_run: bool,
) -> Dict[str, Any]:
    return {
        "schema_version": 1,
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "base_inputs": base_inputs,
        "vary": vary,
        "objective": {
            "metric": objective_metric,
            "direction": objective_direction,
        },
        "constraints": constraints,
        "options": {
            "include_derived_inputs": include_derived_inputs,
            "auto_refresh_linked_inputs": auto_refresh_linked_inputs,
            "reload_modules_before_run": reload_modules_before_run,
        },
    }


def apply_config_to_session(cfg: Dict[str, Any], editable_specs: List[Dict[str, Any]], varyable_specs: List[Dict[str, Any]]) -> None:
    base_inputs = cfg.get("base_inputs", {}) or {}
    vary = cfg.get("vary", {}) or {}
    objective = cfg.get("objective", {}) or {}
    constraints = cfg.get("constraints", []) or []
    options = cfg.get("options", {}) or {}

    st.session_state["objective_metric"] = objective.get("metric", st.session_state.get("objective_metric", ""))
    st.session_state["objective_direction"] = objective.get("direction", st.session_state.get("objective_direction", "min"))
    st.session_state["constraints_json"] = json.dumps(constraints, indent=2, ensure_ascii=False)
    st.session_state["include_derived_inputs"] = bool(options.get("include_derived_inputs", False))
    st.session_state["auto_refresh_linked_inputs"] = bool(options.get("auto_refresh_linked_inputs", True))
    st.session_state["reload_modules_before_run"] = bool(options.get("reload_modules_before_run", True))

    editable_names = {s["name"] for s in editable_specs}
    for name, value in base_inputs.items():
        if name not in editable_names:
            continue
        spec = next(s for s in editable_specs if s["name"] == name)
        key = f"base__{name}"
        if spec["is_json"]:
            st.session_state[key] = json.dumps(value, indent=2, ensure_ascii=False)
        else:
            st.session_state[key] = value

    vary_names = {s["name"] for s in varyable_specs}
    selected = [name for name in vary.keys() if name in vary_names]
    st.session_state["vary_selected_params"] = selected

    for name, spec_cfg in vary.items():
        if name not in vary_names:
            continue
        mode = spec_cfg.get("mode", "list")
        st.session_state[f"vary_mode__{name}"] = mode
        if mode == "list":
            vals = spec_cfg.get("values", [])
            st.session_state[f"vary_list__{name}"] = ", ".join(str(v) for v in vals)
        elif mode in {"range", "range_points"}:
            st.session_state[f"vary_mode__{name}"] = "range_points"
            st.session_state[f"vary_min__{name}"] = spec_cfg.get("min")
            st.session_state[f"vary_max__{name}"] = spec_cfg.get("max")
            st.session_state[f"vary_points__{name}"] = int(spec_cfg.get("points", 5))
        elif mode == "range_step":
            st.session_state[f"vary_min__{name}"] = spec_cfg.get("min")
            st.session_state[f"vary_max__{name}"] = spec_cfg.get("max")
            st.session_state[f"vary_step__{name}"] = spec_cfg.get("step")


# ============================================================
# Reload/project helpers
# ============================================================
def reload_project_modules(module_names: Optional[List[str]] = None) -> None:
    importlib.invalidate_caches()
    names = module_names or PROJECT_MODULES_TO_RELOAD
    for name in names:
        try:
            if name in sys.modules:
                importlib.reload(sys.modules[name])
            else:
                importlib.import_module(name)
        except Exception:
            continue


def refresh_import_handles() -> None:
    globals()["design_inputs"] = importlib.import_module("design_inputs")
    globals()["script"] = importlib.import_module("script")


# ============================================================
# Variation parsing
# ============================================================
def parse_value_list_for_spec(text: str, spec: Dict[str, Any]) -> List[Any]:
    vals: List[Any] = []
    for raw in parse_comma_list(text):
        if spec["is_bool"]:
            low = raw.lower()
            if low not in {"true", "false"}:
                raise ValueError(f"{spec['name']}: use true/false for bool lists")
            vals.append(low == "true")
        elif is_int_type(spec["type"]):
            vals.append(int(round(float(raw))))
        elif is_float_type(spec["type"]) or spec["is_numeric"]:
            vals.append(float(raw))
        else:
            vals.append(raw)
    if not vals:
        raise ValueError(f"{spec['name']}: list cannot be empty")
    return vals


def build_range_from_step(spec: Dict[str, Any], lo: Any, hi: Any, step: Any) -> List[Any]:
    name = spec["name"]
    try:
        lo_f = float(lo)
        hi_f = float(hi)
        step_f = float(step)
    except Exception as e:
        raise ValueError(f"{name}: min/max/step must be numeric") from e

    if step_f <= 0.0:
        raise ValueError(f"{name}: step must be > 0")

    if hi_f < lo_f:
        raise ValueError(f"{name}: max must be >= min")

    vals: List[Any] = []

    if is_int_type(spec["type"]):
        lo_i = int(round(lo_f))
        hi_i = int(round(hi_f))
        step_i = int(round(step_f))
        if step_i <= 0:
            raise ValueError(f"{name}: integer step must be >= 1")
        vals = list(range(lo_i, hi_i + 1, step_i))
        if not vals:
            raise ValueError(f"{name}: range produced no values")
        if vals[-1] != hi_i:
            vals.append(hi_i)
        return vals

    # Float-safe inclusive generation.
    tol = max(1e-12, abs(step_f) * 1e-9)
    x = lo_f
    while x <= hi_f + tol:
        vals.append(float(x))
        x += step_f

    if not vals:
        raise ValueError(f"{name}: range produced no values")

    if abs(vals[-1] - hi_f) > tol:
        vals.append(float(hi_f))

    # Remove near-duplicates from floating point accumulation.
    deduped: List[Any] = []
    for v in vals:
        if not deduped or abs(float(v) - float(deduped[-1])) > tol:
            deduped.append(float(v))
    return deduped


def build_param_grid(vary_specs: List[Dict[str, Any]]) -> Dict[str, List[Any]]:
    grid: Dict[str, List[Any]] = {}
    for spec in vary_specs:
        name = spec["name"]
        mode = st.session_state.get(f"vary_mode__{name}", "list")
        if mode == "list":
            text = st.session_state.get(f"vary_list__{name}", "")
            grid[name] = parse_value_list_for_spec(text, spec)
        elif mode == "range_points":
            lo = st.session_state.get(f"vary_min__{name}")
            hi = st.session_state.get(f"vary_max__{name}")
            n = int(st.session_state.get(f"vary_points__{name}", 5))
            if lo is None or hi is None:
                raise ValueError(f"{name}: range bounds are missing")
            if n < 1:
                raise ValueError(f"{name}: number of points must be >= 1")
            vals = np.linspace(float(lo), float(hi), n).tolist()
            if is_int_type(spec["type"]):
                vals = [int(round(v)) for v in vals]
                vals = sorted(set(vals))
            grid[name] = vals
        elif mode == "range_step":
            lo = st.session_state.get(f"vary_min__{name}")
            hi = st.session_state.get(f"vary_max__{name}")
            step = st.session_state.get(f"vary_step__{name}")
            if lo is None or hi is None or step is None:
                raise ValueError(f"{name}: min/max/step are required")
            grid[name] = build_range_from_step(spec, lo, hi, step)
        else:
            raise ValueError(f"Unsupported vary mode for {name}: {mode}")
    return grid


# ============================================================
# Probe run for metric discovery
# ============================================================
@st.cache_data(show_spinner=False)
def probe_metric_catalog_cached(include_derived: bool) -> Tuple[List[str], str]:
    # Probe with a clean default run.
    inp = design_inputs.DesignInputs()
    res = script.run_design(inp)
    flat = flatten_any(res)
    metrics = sorted(k for k, v in flat.items() if excel_scalar(v))

    preferred = [
        "aircraft.W0_mass_kg",
        "W0_mass_kg",
        "aircraft.static_margin_percent_MAC",
        "wing.geometry.span_m",
    ]
    default_metric = next((m for m in preferred if m in metrics), metrics[0] if metrics else "")
    return metrics, default_metric


# ============================================================
# DoE runner
# ============================================================
def run_single_case(
    base_inputs: Dict[str, Any],
    overrides: Dict[str, Any],
    locked_fields: set[str],
    auto_refresh_linked_inputs: bool,
) -> Tuple[Any, Dict[str, Any]]:
    merged = dict(base_inputs)
    merged.update(overrides)
    inp = design_inputs.DesignInputs(**merged)
    if auto_refresh_linked_inputs:
        inp = refresh_linked_inputs(inp, locked_fields=locked_fields)
    res = script.run_design(inp)
    flat = flatten_any(res)
    return inp, flat


def run_doe(
    base_inputs: Dict[str, Any],
    param_grid: Dict[str, List[Any]],
    objective_metric: str,
    direction: str,
    constraints: List[Dict[str, Any]],
    auto_refresh_linked_inputs: bool,
    reload_modules_before_run: bool,
) -> Tuple[pd.DataFrame, pd.DataFrame, Dict[str, Any]]:
    if reload_modules_before_run:
        reload_project_modules()
        refresh_import_handles()

    params = list(param_grid.keys())
    combos = list(itertools.product(*[param_grid[p] for p in params])) if params else [()]
    locked_fields = set(base_inputs.keys()) if not auto_refresh_linked_inputs else set(params)

    successes: List[Dict[str, Any]] = []
    failures: List[Dict[str, Any]] = []

    total = len(combos)
    prog = st.progress(0.0)
    status = st.empty()

    for i, combo in enumerate(combos, start=1):
        overrides = {p: v for p, v in zip(params, combo)}
        status.write(f"Running {i}/{total} ...")

        try:
            _, flat = run_single_case(
                base_inputs=base_inputs,
                overrides=overrides,
                locked_fields=locked_fields,
                auto_refresh_linked_inputs=auto_refresh_linked_inputs,
            )

            if objective_metric not in flat:
                raise KeyError(f"Objective metric '{objective_metric}' not found in results.")

            try:
                objective_value = float(flat[objective_metric])
            except Exception as e:
                raise ValueError(f"Objective metric '{objective_metric}' is not numeric: {flat.get(objective_metric)!r}") from e

            feasible, constraint_messages = evaluate_constraints(flat, constraints)

            row: Dict[str, Any] = {p: overrides.get(p, base_inputs.get(p)) for p in params}
            row["status"] = "OK"
            row["feasible"] = feasible
            row["constraint_messages"] = " | ".join(constraint_messages)
            row["objective_metric"] = objective_metric
            row["objective_direction"] = direction
            row["objective"] = objective_value

            for k, v in flat.items():
                if excel_scalar(v):
                    row[k] = v

            successes.append(row)
        except Exception as e:
            fail_row = {p: overrides.get(p, base_inputs.get(p)) for p in params}
            fail_row["status"] = "FAILED"
            fail_row["error"] = f"{type(e).__name__}: {e}"
            fail_row["traceback"] = traceback.format_exc()[:32000]
            failures.append(fail_row)

        prog.progress(i / max(total, 1))

    status.empty()

    df_ok = pd.DataFrame(successes)
    df_bad = pd.DataFrame(failures)

    if not df_ok.empty:
        ascending = direction.lower() == "min"
        # Feasible runs always rank ahead of infeasible runs.
        df_ok["_feasible_rank"] = (~df_ok["feasible"]).astype(int)
        df_ok = df_ok.sort_values(["_feasible_rank", "objective"], ascending=[True, ascending]).drop(columns=["_feasible_rank"])
        df_ok = df_ok.reset_index(drop=True)

    meta = {
        "n_total": int(total),
        "n_success": int(len(df_ok)),
        "n_failed": int(len(df_bad)),
        "n_feasible": int(df_ok["feasible"].sum()) if not df_ok.empty and "feasible" in df_ok.columns else 0,
    }
    return df_ok, df_bad, meta


# ============================================================
# Analysis/export helpers
# ============================================================
def compute_sensitivity(df_ok: pd.DataFrame, params: List[str], objective_col: str = "objective") -> pd.DataFrame:
    if df_ok.empty:
        return pd.DataFrame(columns=["param", "pearson_r", "spearman_rho", "std_slope", "abs_std_slope", "n"])

    y = pd.to_numeric(df_ok[objective_col], errors="coerce").astype(float).values
    y_std = float(np.nanstd(y)) if float(np.nanstd(y)) > 0 else 1.0

    rows = []
    for p in params:
        if p not in df_ok.columns:
            continue
        x = pd.to_numeric(df_ok[p], errors="coerce").astype(float).values
        mask = np.isfinite(x) & np.isfinite(y)
        if int(mask.sum()) < 3:
            continue

        xx = x[mask]
        yy = y[mask]

        pearson = float(pd.Series(xx).corr(pd.Series(yy), method="pearson"))
        spearman = float(pd.Series(xx).corr(pd.Series(yy), method="spearman"))
        x_std = float(np.std(xx)) if float(np.std(xx)) > 0 else 1.0
        slope = float(np.polyfit(xx, yy, 1)[0]) if len(xx) >= 2 else 0.0
        std_slope = float(slope * (x_std / y_std))

        rows.append(
            {
                "param": p,
                "pearson_r": pearson,
                "spearman_rho": spearman,
                "std_slope": std_slope,
                "abs_std_slope": abs(std_slope),
                "n": int(mask.sum()),
            }
        )

    df = pd.DataFrame(rows)
    if not df.empty:
        df = df.sort_values("abs_std_slope", ascending=False).reset_index(drop=True)
    return df


def dataframe_to_excel_bytes(df_ok: pd.DataFrame, df_bad: pd.DataFrame, df_sens: pd.DataFrame, config: Dict[str, Any]) -> bytes:
    bio = BytesIO()
    with pd.ExcelWriter(bio, engine="openpyxl") as writer:
        df_ok.to_excel(writer, sheet_name="success", index=False)
        df_bad.to_excel(writer, sheet_name="failed", index=False)
        df_sens.to_excel(writer, sheet_name="sensitivity", index=False)
        pd.DataFrame([{"config_json": json.dumps(config, indent=2, ensure_ascii=False)}]).to_excel(writer, sheet_name="config", index=False)
    return bio.getvalue()


def plot_1d(df: pd.DataFrame, x: str) -> None:
    d = df.sort_values(x)
    fig = plt.figure(figsize=(6, 3), dpi=PLOT_DPI)
    ax = fig.add_subplot(111)
    ax.plot(d[x], d["objective"], marker="o", markersize=4)
    ax.set_xlabel(x)
    ax.set_ylabel("objective")
    ax.grid(True)
    fig.tight_layout()
    st.pyplot(fig, use_container_width=True)
    plt.close(fig)


def plot_2d(df: pd.DataFrame, x: str, y: str) -> None:
    fig = plt.figure(figsize=(6, 4), dpi=PLOT_DPI)
    ax = fig.add_subplot(111)
    sc = ax.scatter(df[x], df[y], c=df["objective"], s=40)
    ax.set_xlabel(x)
    ax.set_ylabel(y)
    ax.grid(True)
    fig.colorbar(sc, ax=ax, label="objective")
    fig.tight_layout()
    st.pyplot(fig, use_container_width=True)
    plt.close(fig)


# ============================================================
# UI
# ============================================================
st.set_page_config(page_title=APP_TITLE, layout="wide")
st.title(APP_TITLE)
st.caption("Auto-discovers DesignInputs fields and result metrics from your UAV sizing model.")

# Apply uploaded config on a fresh rerun *before* widget-bound keys are instantiated.
_pending_loaded_cfg = st.session_state.pop("_pending_loaded_cfg", None)
if _pending_loaded_cfg is not None:
    try:
        _cfg = _pending_loaded_cfg if isinstance(_pending_loaded_cfg, dict) else json.loads(str(_pending_loaded_cfg))
        _include_derived_from_cfg = bool(((_cfg.get("options", {}) or {}).get("include_derived_inputs", False)))
        st.session_state["include_derived_inputs"] = _include_derived_from_cfg
        _editable_specs_for_cfg = get_design_field_specs(include_derived=_include_derived_from_cfg)
        _varyable_specs_for_cfg = get_varyable_specs(_editable_specs_for_cfg)
        apply_config_to_session(_cfg, _editable_specs_for_cfg, _varyable_specs_for_cfg)
        st.session_state["_loaded_cfg_success"] = True
    except Exception as e:
        st.session_state["_loaded_cfg_error"] = f"{type(e).__name__}: {e}"


with st.sidebar:
    st.header("Model / metric discovery")
    include_derived_inputs = st.checkbox(
        "Show derived input fields",
        value=st.session_state.get("include_derived_inputs", False),
        key="include_derived_inputs",
        help="Hidden by default: fields like dive_speed, fuselage_OR, battery dimensions, etc.",
    )

editable_specs = get_design_field_specs(include_derived=include_derived_inputs)
varyable_specs = get_varyable_specs(editable_specs)

try:
    metric_catalog, default_metric = probe_metric_catalog_cached(include_derived_inputs)
except Exception as e:
    metric_catalog = []
    default_metric = ""
    st.error(f"Could not probe model outputs from a default run: {type(e).__name__}: {e}")

if "objective_metric" not in st.session_state:
    st.session_state["objective_metric"] = default_metric
if "objective_direction" not in st.session_state:
    st.session_state["objective_direction"] = "min"
if "constraints_json" not in st.session_state:
    st.session_state["constraints_json"] = DEFAULT_CONSTRAINTS_JSON
if "auto_refresh_linked_inputs" not in st.session_state:
    st.session_state["auto_refresh_linked_inputs"] = True
if "reload_modules_before_run" not in st.session_state:
    st.session_state["reload_modules_before_run"] = True
if "vary_selected_params" not in st.session_state:
    st.session_state["vary_selected_params"] = []
if "doe_results" not in st.session_state:
    st.session_state["doe_results"] = None

with st.sidebar:
    st.divider()
    st.header("Load / save config")
    uploaded_cfg = st.file_uploader("Upload DoE config (.json)", type=["json"])
    if uploaded_cfg is not None:
        try:
            cfg = json.loads(uploaded_cfg.getvalue().decode("utf-8-sig"))
            st.session_state["_pending_loaded_cfg"] = cfg
            st.session_state["_pending_loaded_cfg_name"] = uploaded_cfg.name
            st.rerun()
        except Exception as e:
            st.error(f"Could not load config: {type(e).__name__}: {e}")

    if st.session_state.pop("_loaded_cfg_success", False):
        loaded_name = st.session_state.pop("_pending_loaded_cfg_name", "uploaded config")
        st.success(f"Loaded config: {loaded_name}")
    loaded_err = st.session_state.pop("_loaded_cfg_error", None)
    if loaded_err:
        st.error(f"Could not load config: {loaded_err}")

    st.divider()
    st.header("Objective")
    if metric_catalog:
        objective_metric = st.selectbox(
            "Objective metric",
            options=metric_catalog,
            index=metric_catalog.index(st.session_state.get("objective_metric", default_metric)) if st.session_state.get("objective_metric", default_metric) in metric_catalog else 0,
            key="objective_metric",
        )
    else:
        objective_metric = st.text_input("Objective metric", value=st.session_state.get("objective_metric", default_metric), key="objective_metric")

    objective_direction = st.radio("Direction", ["min", "max"], horizontal=True, key="objective_direction")

    st.divider()
    st.header("Run options")
    auto_refresh_linked_inputs = st.checkbox(
        "Auto-refresh linked geometry inputs",
        value=st.session_state.get("auto_refresh_linked_inputs", True),
        key="auto_refresh_linked_inputs",
        help="Recomputes a few section lengths / linked dimensions from the primary geometry inputs before each run.",
    )
    reload_modules_before_run = st.checkbox(
        "Reload project modules before run",
        value=st.session_state.get("reload_modules_before_run", True),
        key="reload_modules_before_run",
    )

    st.text_area(
        "Linked refresh info",
        value=DEFAULT_LINKED_REFRESH_JSON,
        height=150,
        disabled=True,
    )

    st.divider()
    run_button = st.button("Run DoE", type="primary")


# ============================================================
# Base inputs editor
# ============================================================
st.subheader("1) Base design inputs")
col1, col2 = st.columns([2, 1])
with col1:
    search_text = st.text_input("Filter inputs by name", value="")
with col2:
    groups = sorted({s["group"] for s in editable_specs})
    selected_groups = st.multiselect("Groups", options=groups, default=groups)

filtered_specs = [
    s for s in editable_specs
    if s["group"] in selected_groups and (search_text.strip().lower() in s["name"].lower())
]

if not filtered_specs:
    st.info("No inputs match the current filter.")
else:
    group_map: Dict[str, List[Dict[str, Any]]] = {}
    for spec in filtered_specs:
        group_map.setdefault(spec["group"], []).append(spec)

    for group, specs_in_group in group_map.items():
        with st.expander(group, expanded=False):
            left, right = st.columns(2)
            for idx, spec in enumerate(specs_in_group):
                target_col = left if idx % 2 == 0 else right
                name = spec["name"]
                default = spec["default"]
                key = f"base__{name}"
                tp = spec["type"]
                with target_col:
                    if spec["is_bool"]:
                        st.checkbox(name, value=bool(st.session_state.get(key, default)), key=key)
                    elif is_int_type(tp):
                        st.number_input(name, value=int(st.session_state.get(key, int(default) if default is not None else 0)), step=1, key=key)
                    elif is_float_type(tp) or (spec["is_numeric"] and not spec["is_bool"]):
                        st.number_input(name, value=float(st.session_state.get(key, float(default) if default is not None else 0.0)), key=key, format="%.10g")
                    elif spec["is_json"]:
                        st.text_area(name, value=st.session_state.get(key, to_pretty_json(default)), key=key, height=160)
                    else:
                        st.text_input(name, value=str(st.session_state.get(key, default)), key=key)


# ============================================================
# Varying parameters editor
# ============================================================
st.subheader("2) Parameters to vary in the DoE")
selected_vary_params = st.multiselect(
    "Choose parameters to sweep",
    options=[s["name"] for s in varyable_specs],
    default=st.session_state.get("vary_selected_params", []),
    key="vary_selected_params",
)

if selected_vary_params:
    for name in selected_vary_params:
        spec = next(s for s in varyable_specs if s["name"] == name)
        default_val = st.session_state.get(f"base__{name}", spec["default"])
        with st.expander(name, expanded=True):
            if spec["is_bool"]:
                st.session_state[f"vary_mode__{name}"] = "list"
                st.text_input(
                    "Values (true/false, comma-separated)",
                    value=st.session_state.get(f"vary_list__{name}", "true, false" if bool(default_val) else "false, true"),
                    key=f"vary_list__{name}",
                )
            elif spec["is_numeric"]:
                mode = st.radio(
                    "Mode",
                    ["list", "range_step", "range_points"],
                    horizontal=True,
                    key=f"vary_mode__{name}",
                    format_func=lambda m: {
                        "list": "List",
                        "range_step": "Min / Max / Step",
                        "range_points": "Min / Max / Points",
                    }[m],
                )
                if mode == "list":
                    st.text_input(
                        "Values (comma-separated)",
                        value=st.session_state.get(f"vary_list__{name}", str(default_val)),
                        key=f"vary_list__{name}",
                    )
                else:
                    base_num = float(default_val) if default_val is not None else 0.0
                    lo_default = base_num * 0.9 if base_num != 0 else 0.0
                    hi_default = base_num * 1.1 if base_num != 0 else 1.0
                    default_step = 1.0 if is_int_type(spec["type"]) else max(abs(hi_default - lo_default) / 4.0, 0.01 if base_num == 0 else abs(base_num) * 0.05)
                    c1, c2, c3 = st.columns(3)
                    with c1:
                        st.number_input("min", value=float(st.session_state.get(f"vary_min__{name}", lo_default)), key=f"vary_min__{name}", format="%.10g")
                    with c2:
                        st.number_input("max", value=float(st.session_state.get(f"vary_max__{name}", hi_default)), key=f"vary_max__{name}", format="%.10g")
                    with c3:
                        if mode == "range_step":
                            st.number_input(
                                "step",
                                min_value=1 if is_int_type(spec["type"]) else None,
                                value=int(st.session_state.get(f"vary_step__{name}", max(1, int(round(default_step))))) if is_int_type(spec["type"]) else float(st.session_state.get(f"vary_step__{name}", default_step)),
                                step=1 if is_int_type(spec["type"]) else None,
                                key=f"vary_step__{name}",
                                format=None if is_int_type(spec["type"]) else "%.10g",
                            )
                        else:
                            st.number_input("points", min_value=1, max_value=500, value=int(st.session_state.get(f"vary_points__{name}", 5)), step=1, key=f"vary_points__{name}")
            else:
                st.session_state[f"vary_mode__{name}"] = "list"
                st.text_input(
                    "Values (comma-separated)",
                    value=st.session_state.get(f"vary_list__{name}", str(default_val)),
                    key=f"vary_list__{name}",
                )
else:
    st.info("No varying parameters selected. The app will still run one case with the base inputs.")


# ============================================================
# Constraints editor
# ============================================================
st.subheader("3) Constraints")
st.caption("Use any discovered result metric here. Example: static margin >= 5 and wing span <= 2.5 m.")
if metric_catalog:
    with st.expander("Available metrics from the model probe", expanded=False):
        st.code("\n".join(metric_catalog), language="text")

st.text_area(
    "Constraints JSON",
    key="constraints_json",
    height=220,
)


# ============================================================
# Build config preview / downloads
# ============================================================
config_name = st.text_input("Config filename stem", value=DEFAULT_EXPORTED_CONFIG_NAME)
results_name = st.text_input("Results filename stem", value=DEFAULT_EXPORTED_RESULTS_NAME)

config_error = None
base_inputs_current: Dict[str, Any] = {}
constraints_current: List[Dict[str, Any]] = []
param_grid_current: Dict[str, List[Any]] = {}

try:
    base_inputs_current = get_base_input_values(editable_specs)
    constraints_current = parse_constraints_json(st.session_state.get("constraints_json", ""))
    selected_specs = [s for s in varyable_specs if s["name"] in selected_vary_params]
    param_grid_current = build_param_grid(selected_specs)
except Exception as e:
    config_error = f"{type(e).__name__}: {e}"
    st.error(f"Configuration error: {config_error}")

if param_grid_current:
    n_runs = int(np.prod([len(v) for v in param_grid_current.values()]))
else:
    n_runs = 1
st.info(f"Planned runs: {n_runs}")
if n_runs > 1500:
    st.warning("This is a large full-factorial sweep. Reduce points or number of parameters if it becomes slow.")

if not config_error:
    cfg_preview = build_config_dict(
        base_inputs=base_inputs_current,
        vary={
            name: (
                {"mode": "range_points", "min": st.session_state.get(f"vary_min__{name}"), "max": st.session_state.get(f"vary_max__{name}"), "points": int(st.session_state.get(f"vary_points__{name}", 5))}
                if st.session_state.get(f"vary_mode__{name}", "list") == "range_points"
                else (
                    {"mode": "range_step", "min": st.session_state.get(f"vary_min__{name}"), "max": st.session_state.get(f"vary_max__{name}"), "step": st.session_state.get(f"vary_step__{name}")}
                    if st.session_state.get(f"vary_mode__{name}", "list") == "range_step"
                    else {"mode": "list", "values": param_grid_current.get(name, [])}
                )
            )
            for name in selected_vary_params
        },
        objective_metric=objective_metric,
        objective_direction=objective_direction,
        constraints=constraints_current,
        include_derived_inputs=include_derived_inputs,
        auto_refresh_linked_inputs=auto_refresh_linked_inputs,
        reload_modules_before_run=reload_modules_before_run,
    )

    st.download_button(
        "Download current config as JSON",
        data=json.dumps(cfg_preview, indent=2, ensure_ascii=False).encode("utf-8"),
        file_name=f"{config_name}.json",
        mime="application/json",
    )


# ============================================================
# Run DoE
# ============================================================
if run_button and not config_error:
    with st.spinner("Running DoE..."):
        df_ok, df_bad, meta = run_doe(
            base_inputs=base_inputs_current,
            param_grid=param_grid_current,
            objective_metric=objective_metric,
            direction=objective_direction,
            constraints=constraints_current,
            auto_refresh_linked_inputs=auto_refresh_linked_inputs,
            reload_modules_before_run=reload_modules_before_run,
        )
        df_sens = compute_sensitivity(df_ok[df_ok.get("feasible", True) == True] if not df_ok.empty else df_ok, list(param_grid_current.keys()))

    st.session_state["doe_results"] = {
        "df_ok": df_ok,
        "df_bad": df_bad,
        "df_sens": df_sens,
        "meta": meta,
        "config": cfg_preview,
        "params": list(param_grid_current.keys()),
    }


# ============================================================
# Display results
# ============================================================
respack = st.session_state.get("doe_results")
if respack is None:
    st.info("Set the base inputs, choose the parameters to vary, define the objective/constraints, then click Run DoE.")
else:
    df_ok = respack["df_ok"]
    df_bad = respack["df_bad"]
    df_sens = respack["df_sens"]
    meta = respack["meta"]
    cfg = respack["config"]
    params = respack["params"]

    st.subheader("4) Results")
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Total runs", meta.get("n_total", 0))
    c2.metric("Successful", meta.get("n_success", 0))
    c3.metric("Failed", meta.get("n_failed", 0))
    c4.metric("Feasible", meta.get("n_feasible", 0))

    if not df_ok.empty:
        feasible_df = df_ok[df_ok["feasible"] == True] if "feasible" in df_ok.columns else df_ok
        best_df = feasible_df if not feasible_df.empty else df_ok
        best = best_df.iloc[0]

        if not feasible_df.empty:
            st.success(f"Best feasible objective: {float(best['objective']):.6g}")
        else:
            st.warning("No feasible run satisfied all constraints. Showing best available objective among all successful runs.")
            st.write(f"Best available objective: {float(best['objective']):.6g}")

        best_cols = [p for p in params if p in best.index]
        if best_cols:
            st.write("**Best run decision variables**")
            st.dataframe(pd.DataFrame([{ "parameter": p, "value": best[p]} for p in best_cols]), use_container_width=True)

        tabs = st.tabs(["Top runs", "Plots", "Sensitivity", "All successful runs", "Failures"])

        with tabs[0]:
            show_cols = [c for c in ["feasible", "objective", *params, objective_metric, "constraint_messages"] if c in df_ok.columns]
            st.dataframe(df_ok[show_cols].head(30), use_container_width=True)

        with tabs[1]:
            numeric_params = [p for p in params if p in df_ok.columns and pd.api.types.is_numeric_dtype(df_ok[p])]
            if len(numeric_params) == 1:
                plot_1d(df_ok, numeric_params[0])
            elif len(numeric_params) >= 2:
                x = st.selectbox("X axis", options=numeric_params, key="plot_x")
                y_opts = [p for p in numeric_params if p != x]
                y = st.selectbox("Y axis", options=y_opts, key="plot_y")
                plot_2d(df_ok, x, y)
            else:
                st.info("No numeric swept parameters available for plotting.")

        with tabs[2]:
            if df_sens.empty:
                st.info("Not enough feasible numeric runs to compute sensitivities.")
            else:
                st.dataframe(df_sens, use_container_width=True)

        with tabs[3]:
            st.dataframe(df_ok, use_container_width=True)

        with tabs[4]:
            st.dataframe(df_bad, use_container_width=True)

        excel_bytes = dataframe_to_excel_bytes(df_ok, df_bad, df_sens, cfg)
        st.download_button(
            "Download results as Excel",
            data=excel_bytes,
            file_name=f"{results_name}.xlsx",
            mime="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
        )
    else:
        st.error("No successful runs were produced.")
        st.dataframe(df_bad, use_container_width=True)
