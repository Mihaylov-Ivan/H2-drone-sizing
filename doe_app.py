# doe_app.py
from __future__ import annotations

import itertools
import json
import os
import sys
import traceback
import importlib
from datetime import datetime
from io import BytesIO
from typing import Any, Dict, List, Tuple, Optional, get_origin, get_args

import numpy as np
import pandas as pd
import streamlit as st
import matplotlib.pyplot as plt

import design_inputs
import script


# ============================================================
# HARD-CODED DROPDOWN OPTIONS (EDIT THESE LISTS)
# ============================================================

OBJECTIVE_OPTIONS: List[Tuple[str, str]] = [
    ("MTOW (W0_mass_kg)", "W0_mass_kg"),
    ("cruise electric power (cruise_power_elec_W)", "cruise_power_elec_W"),
    ("cruise drag (drag_cruise_N)", "drag_cruise_N"),
    ("cruise L/D (L_over_D_cruise)", "L_over_D_cruise"),
    ("stall speed (V_stall)", "V_stall"),
    ("mission-limited cruise time (mission_profile.cruise_time_max_min)", "mission_profile.cruise_time_max_min"),
    ("mission total energy (mission_profile.total_energy_Wh)", "mission_profile.total_energy_Wh"),
    ("mission peak electrical power (mission_profile.peak_elec_power_W)", "mission_profile.peak_elec_power_W"),
]

VARY_PARAM_OPTIONS: List[Tuple[str, str]] = [
    ("Payload mass [kg]", "payload_mass"),
    ("Cruise endurance [min]", "cruise_endurance_min"),
    ("Cruise speed [m/s]", "speed"),
    ("Wing loading W/S [N/m²]", "wing_loading"),
    ("Ultimate load factor n_ult [-]", "n_ult"),

    ("Wing root chord c_root [m]", "c_root"),
    ("Wing taper ratio [-]", "taper_ratio"),
    ("Wing sweep angle [deg]", "sweep_angle_deg"),
    ("Oswald efficiency e (wing)", "e_oswald"),
    ("Plain flap chord ratio (wing)", "wing_plain_flap_chord_ratio"),
    ("Control surface drag factor k", "control_surface_drag_k"),

    ("Tail root chord [m]", "tail_c_root"),
    ("Tail taper ratio [-]", "tail_taper_ratio"),
    ("Oswald efficiency e (tail)", "tail_e_oswald"),
    ("Tail sweep angle [deg]", "tail_sweep_angle_deg"),
    ("Elevator chord ratio (tail)", "tail_elevator_chord_ratio"),

    ("Fuselage length [m]", "fuselage_length"),
    ("Fuselage outer diameter OD [m]", "fuselage_OD"),
    ("Fuselage nose length [m]", "fuselage_nose_length"),
    ("Fuselage non-opt factor [-]", "fuselage_nonopt_factor"),

    ("Prop Y position fraction [-]", "prop_y_position_fraction"),
    ("Prop X position fraction [-]", "prop_x_position_fraction"),
    ("Min prop-fuselage clearance [m]", "prop_fuselage_clearance_min_m"),

    ("VTOL takeoff hover time [s]", "vtol_takeoff_hover_time_s"),
    ("VTOL climb height [m]", "vtol_climb_height_m"),
    ("Transition time [s]", "transition_time_s"),
    ("Accel-to-cruise time [s]", "accel_to_cruise_time_s"),

    # --------------------------------------------------------
    # CG locations
    # --------------------------------------------------------
    ("CG x: wing [m]", "cg_x_wing"),
    ("CG x: tail [m]", "cg_x_tail"),
    ("CG x: propulsion [m]", "cg_x_propulsion"),
    ("CG x: battery [m]", "cg_x_battery"),
    ("CG x: autopilot [m]", "cg_x_autopilot"),
    ("CG x: power module [m]", "cg_x_power_module"),
    ("CG x: ESC [m]", "cg_x_esc"),
    ("CG x: GPS [m]", "cg_x_gps"),
    ("CG x: airspeed sensor [m]", "cg_x_airspeed"),
    ("CG x: RX [m]", "cg_x_rx"),
    ("CG x: telemetry [m]", "cg_x_tel"),
    ("CG x: servos [m]", "cg_x_servos"),
    ("CG x: misc [m]", "cg_x_misc"),
    ("CG x: payload [m]", "cg_x_payload"),
]

DEFAULT_OBJECTIVE_KEY = "W0_mass_kg"
DEFAULT_VARY_PARAMS = ["wing_loading", "speed"]

OBJECTIVE_KEY_TO_LABEL = {key: label for (label, key) in OBJECTIVE_OPTIONS}
OBJECTIVE_KEYS = [key for (_, key) in OBJECTIVE_OPTIONS]

PARAM_LABEL_TO_NAME = {label: name for (label, name) in VARY_PARAM_OPTIONS}
PARAM_NAME_TO_LABEL = {name: label for (label, name) in VARY_PARAM_OPTIONS}
PARAM_LABELS = [label for (label, _) in VARY_PARAM_OPTIONS]


# ============================================================
# Plot sizing + text sizing
# ============================================================
PLOT_FIGSIZE_1D = (4.5, 2.2)
PLOT_FIGSIZE_2D = (4.8, 2.6)
PLOT_FIGSIZE_HIST = (4.5, 2.0)
PLOT_DPI = 120

FONT_LABEL = 6
FONT_TICK = 6
FONT_CBAR = 6


# ============================================================
# Type helpers (FIX: decide float/int from annotations, not base_val)
# ============================================================
def _normalize_annot(t: Any) -> Any:
    """
    Reduce Optional[T]/Union[T,None] -> T, keep plain types as-is.
    """
    origin = get_origin(t)
    if origin is None:
        return t
    if origin is Optional:
        args = [a for a in get_args(t) if a is not type(None)]
        return args[0] if args else t
    if origin is list or origin is dict or origin is tuple:
        return t
    if origin is type(Optional[int]).__origin__ or origin is type(Optional[float]).__origin__:
        args = [a for a in get_args(t) if a is not type(None)]
        return args[0] if args else t
    return t


def wants_int(param_name: str) -> bool:
    ann = getattr(design_inputs.DesignInputs, "__annotations__", {}).get(param_name, None)
    ann = _normalize_annot(ann)
    return ann is int


def wants_float(param_name: str) -> bool:
    ann = getattr(design_inputs.DesignInputs, "__annotations__", {}).get(param_name, None)
    ann = _normalize_annot(ann)
    return ann is float


def is_numeric(v: Any) -> bool:
    return isinstance(v, (int, float, np.number)) and not isinstance(v, bool)


def _is_excel_scalar(v: Any) -> bool:
    return (
        v is None
        or isinstance(v, (int, float, bool, str, np.integer, np.floating, np.bool_))
    )


# ============================================================
# Utilities
# ============================================================
def flatten_results(res: Any) -> Dict[str, Any]:
    """
    Flatten DesignResults -> dict of mostly scalars.

    - Uses results.to_dict() as authoritative.
    - Promotes inputs.mission_profile.* to top-level keys
    """
    flat = res.to_dict() if hasattr(res, "to_dict") else dict(res)

    inputs_blob = flat.get("inputs", {})
    if isinstance(inputs_blob, dict):
        mp = inputs_blob.get("mission_profile", None)
        if isinstance(mp, dict):
            for k, v in mp.items():
                if _is_excel_scalar(v):
                    flat[f"mission_profile.{k}"] = v

        for k in ("tail_span_total_m", "tail_span_extension_m", "tail_area_total_m2"):
            if k in inputs_blob and _is_excel_scalar(inputs_blob.get(k)):
                flat[k] = inputs_blob.get(k)

    return flat


def metric_from_flat(flat: Dict[str, Any], key: str) -> float:
    if key in flat and flat[key] is not None:
        return float(flat[key])
    raise KeyError(f"Objective '{key}' not found in result keys for this run.")


def validate_simulation_postrun(inp: "design_inputs.DesignInputs", flat: Dict[str, Any]) -> None:
    issues: List[str] = []

    try:
        if float(getattr(inp, "fuselage_length")) <= 0:
            issues.append(f"fuselage_length must be > 0 (got {getattr(inp,'fuselage_length')})")
    except Exception:
        pass

    try:
        V_stall = float(flat.get("V_stall", float("nan")))
    except Exception:
        V_stall = float("nan")
    try:
        V_cruise = float(getattr(inp, "speed"))
    except Exception:
        V_cruise = float("nan")

    if np.isfinite(V_stall) and np.isfinite(V_cruise) and V_cruise > 0:
        if V_stall > 0.75 * V_cruise:
            issues.append(
                f"Stall speed ({V_stall:.2f} m/s) exceeds 0.75 * cruise speed ({0.75 * V_cruise:.2f} m/s)"
            )

    if issues:
        raise ValueError("Configuration invalid: " + "; ".join(issues))


def parse_values_list_raw(s: str) -> List[Any]:
    """
    Raw parsing: bool/int/float/string.
    """
    out: List[Any] = []
    for raw in s.split(","):
        raw = raw.strip()
        if not raw:
            continue
        low = raw.lower()
        if low in {"true", "false"}:
            out.append(low == "true")
            continue
        if raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit()):
            out.append(int(raw))
            continue
        try:
            out.append(float(raw))
            continue
        except ValueError:
            out.append(raw)
    return out


def coerce_values_to_param_type(param_name: str, vals: List[Any]) -> List[Any]:
    """
    FIX: keep float parameters as float even if user types '5' (parsed as int).
    """
    if wants_int(param_name):
        out: List[Any] = []
        for v in vals:
            if isinstance(v, bool):
                out.append(bool(v))
            elif isinstance(v, (int, np.integer)):
                out.append(int(v))
            elif isinstance(v, (float, np.floating)):
                out.append(int(round(float(v))))
            else:
                out.append(v)
        return out

    # default numeric => float
    if wants_float(param_name) or True:
        out2: List[Any] = []
        for v in vals:
            if isinstance(v, bool):
                out2.append(bool(v))
            elif isinstance(v, (int, np.integer, float, np.floating)):
                out2.append(float(v))
            else:
                out2.append(v)
        return out2


def values_to_csv_string(vals: List[Any]) -> str:
    def fmt(v: Any) -> str:
        if isinstance(v, bool):
            return "true" if v else "false"
        if isinstance(v, (int, np.integer)):
            return str(int(v))
        if isinstance(v, (float, np.floating)):
            return f"{float(v):.8g}"
        return str(v)

    return ",".join(fmt(v) for v in vals)


def make_range_values(lo: float, hi: float, n: int, *, force_int: bool) -> List[Any]:
    if n <= 1:
        v = float(lo)
        return [int(round(v))] if force_int else [float(v)]
    vals = [float(x) for x in np.linspace(lo, hi, n)]
    if force_int:
        return [int(round(v)) for v in vals]
    return vals


def detect_linspace(vals: List[Any]) -> Optional[Tuple[float, float, int]]:
    try:
        arr = np.array([float(v) for v in vals], dtype=float)
    except Exception:
        return None
    if len(arr) < 2 or not np.all(np.isfinite(arr)):
        return None
    diffs = np.diff(arr)
    if np.allclose(diffs, diffs[0], rtol=1e-6, atol=1e-9):
        return float(arr.min()), float(arr.max()), int(len(arr))
    return None


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
            dict(
                param=p,
                pearson_r=pearson,
                spearman_rho=spearman,
                std_slope=std_slope,
                abs_std_slope=abs(std_slope),
                n=int(mask.sum()),
            )
        )

    df = pd.DataFrame(rows)
    if not df.empty:
        df = df.sort_values("abs_std_slope", ascending=False).reset_index(drop=True)
        df["param_label"] = df["param"].map(lambda n: PARAM_NAME_TO_LABEL.get(n, n))
        df = df[["param_label", "param", "pearson_r", "spearman_rho", "std_slope", "abs_std_slope", "n"]]
    return df


def dataframe_to_excel_bytes(df_ok: pd.DataFrame, df_bad: pd.DataFrame, df_sens: pd.DataFrame, config: Dict[str, Any]) -> bytes:
    bio = BytesIO()
    with pd.ExcelWriter(bio, engine="openpyxl") as writer:
        df_ok.to_excel(writer, sheet_name="success", index=False)
        df_bad.to_excel(writer, sheet_name="failed", index=False)
        df_sens.to_excel(writer, sheet_name="sensitivity", index=False)
        pd.DataFrame([{"config_json": json.dumps(config, indent=2)}]).to_excel(writer, sheet_name="config", index=False)
    return bio.getvalue()


def save_to_xlsx(df_ok: pd.DataFrame, df_bad: pd.DataFrame, df_sens: pd.DataFrame, config: Dict[str, Any], folder: str, exp_name: str) -> str:
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"{exp_name}.xlsx")
    with pd.ExcelWriter(path, engine="openpyxl") as writer:
        df_ok.to_excel(writer, sheet_name="success", index=False)
        df_bad.to_excel(writer, sheet_name="failed", index=False)
        df_sens.to_excel(writer, sheet_name="sensitivity", index=False)
        pd.DataFrame([{"config_json": json.dumps(config, indent=2)}]).to_excel(writer, sheet_name="config", index=False)
    return path


# ============================================================
# JSON CONFIG IO (LOAD + SAVE)
# ============================================================
def build_config(objective_key: str, direction: str, param_grid: Dict[str, List[Any]]) -> Dict[str, Any]:
    return {
        "objective": objective_key,
        "direction": direction,
        "param_grid": param_grid,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "schema_version": 3,
    }


def load_config_json_bytes(file_bytes: bytes) -> Dict[str, Any]:
    txt = file_bytes.decode("utf-8-sig")
    cfg = json.loads(txt)
    if not isinstance(cfg, dict):
        raise ValueError("Config JSON must be an object/dict.")
    if "objective" not in cfg or "direction" not in cfg or "param_grid" not in cfg:
        raise ValueError("Missing required keys: objective, direction, param_grid.")
    if not isinstance(cfg["param_grid"], dict):
        raise ValueError("'param_grid' must be a dict.")
    return cfg


def config_to_json_bytes(cfg: Dict[str, Any]) -> bytes:
    txt = json.dumps(cfg, indent=2, sort_keys=True, ensure_ascii=False)
    return txt.encode("utf-8")


def save_config_to_disk(cfg: Dict[str, Any], folder: str, filename_no_ext: str) -> str:
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"{filename_no_ext}.json")
    with open(path, "wb") as f:
        f.write(config_to_json_bytes(cfg))
    return path


def safe_default_objective_key(loaded: Optional[str]) -> str:
    if OBJECTIVE_KEYS:
        if loaded in OBJECTIVE_KEYS:
            return loaded
        if DEFAULT_OBJECTIVE_KEY in OBJECTIVE_KEYS:
            return DEFAULT_OBJECTIVE_KEY
        return OBJECTIVE_KEYS[0]
    return DEFAULT_OBJECTIVE_KEY


def prime_sidebar_from_loaded_config(cfg: Dict[str, Any]) -> None:
    obj = safe_default_objective_key(cfg.get("objective", None))
    dirn = cfg.get("direction", "min")
    dirn = dirn if dirn in ("min", "max") else "min"

    st.session_state["objective_key_widget"] = obj
    st.session_state["direction_widget"] = dirn

    pg = cfg.get("param_grid", {})
    if not isinstance(pg, dict):
        return

    names = [p for p in pg.keys() if p in PARAM_NAME_TO_LABEL]
    st.session_state["vary_params_widget"] = [PARAM_NAME_TO_LABEL[p] for p in names]

    base = design_inputs.DesignInputs()

    for p, vals in pg.items():
        if p not in PARAM_NAME_TO_LABEL or not isinstance(vals, list) or len(vals) == 0:
            continue

        base_val = getattr(base, p, None)
        # Coerce for correct typing (float stays float)
        vals_typed = coerce_values_to_param_type(p, vals)

        lin = detect_linspace(vals_typed) if is_numeric(base_val) else None

        if isinstance(base_val, bool):
            st.session_state[f"{p}_bool"] = [bool(v) for v in vals_typed if isinstance(v, bool)]
        elif wants_int(p):
            if lin is not None:
                lo, hi, n = lin
                st.session_state[f"{p}_mode_int"] = "Range"
                st.session_state[f"{p}_lo_int"] = int(round(lo))
                st.session_state[f"{p}_hi_int"] = int(round(hi))
                st.session_state[f"{p}_n_int"] = int(n)
            else:
                st.session_state[f"{p}_mode_int"] = "List"
                st.session_state[f"{p}_list_int"] = values_to_csv_string(vals_typed)
        else:
            # float-like
            if lin is not None:
                lo, hi, n = lin
                st.session_state[f"{p}_mode_float"] = "Range"
                st.session_state[f"{p}_lo_float"] = float(lo)
                st.session_state[f"{p}_hi_float"] = float(hi)
                st.session_state[f"{p}_n_float"] = int(n)
            else:
                st.session_state[f"{p}_mode_float"] = "List"
                st.session_state[f"{p}_list_float"] = values_to_csv_string(vals_typed)


# ============================================================
# DoE runner (manual-run equivalent)
# ============================================================

PROJECT_MODULES_TO_RELOAD: List[str] = [
    "constants",
    "design_inputs",
    "geometry",
    "aerodynamics",
    "structures",
    "avionics",
    "propulsion",
    "batteries",
    "propulsion_system",
    "convergence",
    "results",
    "reporting",
    "script",
]


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


def run_doe(
    param_grid: Dict[str, List[Any]],
    objective_key: str,
    direction: str,
    *,
    reload_modules: bool = True,
    store_tracebacks: bool = True,
) -> Tuple[pd.DataFrame, pd.DataFrame, Dict[str, Any]]:
    if reload_modules:
        reload_project_modules()
        globals()["design_inputs"] = importlib.import_module("design_inputs")
        globals()["script"] = importlib.import_module("script")

    params = list(param_grid.keys())
    combos = list(itertools.product(*[param_grid[p] for p in params])) if params else [()]

    successes: List[Dict[str, Any]] = []
    failures: List[Dict[str, Any]] = []

    total = len(combos)
    prog = st.progress(0.0)
    status = st.empty()

    for i, combo in enumerate(combos, start=1):
        status.write(f"Running {i}/{total} ...")
        overrides = {p: v for p, v in zip(params, combo)}

        try:
            inp = design_inputs.DesignInputs(**overrides)
            res = script.run_design(inp)
            flat = flatten_results(res)

            validate_simulation_postrun(inp, flat)
            obj = metric_from_flat(flat, objective_key)

            if not np.isfinite(float(obj)):
                raise ValueError(f"Objective '{objective_key}' is non-finite: {obj}")

            row: Dict[str, Any] = {}
            for p in params:
                row[p] = getattr(inp, p, overrides.get(p, None))

            row["status"] = "OK"
            row["objective"] = float(obj)
            row["objective_name"] = objective_key

            for k in [
                "W0_mass_kg",
                "drag_cruise_N",
                "cruise_power_elec_W",
                "L_over_D_cruise",
                "V_stall",
                "mission_profile.cruise_time_max_min",
                "mission_profile.total_energy_Wh",
                "mission_profile.peak_elec_power_W",
            ]:
                if k in flat and flat[k] is not None:
                    row[k] = flat[k]

            for k, v in flat.items():
                if not _is_excel_scalar(v):
                    continue
                col = f"res.{k}"
                if col not in row:
                    row[col] = v

            successes.append(row)

        except Exception as e:
            fail_row: Dict[str, Any] = dict(overrides)
            fail_row["status"] = "FAILED"
            fail_row["error"] = f"{type(e).__name__}: {e}"
            if store_tracebacks:
                tb = traceback.format_exc()
                fail_row["traceback"] = tb[:32000]
            failures.append(fail_row)

        prog.progress(i / total)

    status.empty()

    df_ok = pd.DataFrame(successes)
    df_bad = pd.DataFrame(failures)

    if not df_ok.empty:
        ascending = (direction.lower() == "min")
        df_ok = df_ok.sort_values("objective", ascending=ascending).reset_index(drop=True)

    meta = dict(n_total=int(total), n_success=int(len(df_ok)), n_failed=int(len(df_bad)))
    return df_ok, df_bad, meta


# ============================================================
# Plot helpers
# ============================================================
def plot_objective_1d(df: pd.DataFrame, x: str) -> None:
    d = df.sort_values(x)
    fig = plt.figure(figsize=PLOT_FIGSIZE_1D, dpi=PLOT_DPI)
    ax = fig.add_subplot(111)
    ax.plot(d[x], d["objective"], marker="o", markersize=4)
    ax.set_xlabel(PARAM_NAME_TO_LABEL.get(x, x), fontsize=FONT_LABEL)
    ax.set_ylabel(OBJECTIVE_KEY_TO_LABEL.get(d["objective_name"].iloc[0], d["objective_name"].iloc[0]), fontsize=FONT_LABEL)
    ax.tick_params(axis="both", labelsize=FONT_TICK)
    ax.grid(True)
    fig.tight_layout()
    st.pyplot(fig, use_container_width=False)
    plt.close(fig)


def plot_objective_2d(df: pd.DataFrame, x: str, y: str) -> None:
    fig = plt.figure(figsize=PLOT_FIGSIZE_2D, dpi=PLOT_DPI)
    ax = fig.add_subplot(111)
    sc = ax.scatter(df[x], df[y], c=df["objective"], s=40)
    ax.set_xlabel(PARAM_NAME_TO_LABEL.get(x, x), fontsize=FONT_LABEL)
    ax.set_ylabel(PARAM_NAME_TO_LABEL.get(y, y), fontsize=FONT_LABEL)
    ax.tick_params(axis="both", labelsize=FONT_TICK)
    ax.grid(True)
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label(OBJECTIVE_KEY_TO_LABEL.get(df["objective_name"].iloc[0], df["objective_name"].iloc[0]), fontsize=FONT_CBAR)
    cbar.ax.tick_params(labelsize=FONT_TICK)
    fig.tight_layout()
    st.pyplot(fig, use_container_width=False)
    plt.close(fig)


def plot_objective_hist(df: pd.DataFrame) -> None:
    fig = plt.figure(figsize=PLOT_FIGSIZE_HIST, dpi=PLOT_DPI)
    ax = fig.add_subplot(111)
    ax.hist(df["objective"].values, bins=min(30, max(5, len(df) // 4)))
    ax.set_xlabel(OBJECTIVE_KEY_TO_LABEL.get(df["objective_name"].iloc[0], df["objective_name"].iloc[0]), fontsize=FONT_LABEL)
    ax.set_ylabel("count", fontsize=FONT_LABEL)
    ax.tick_params(axis="both", labelsize=FONT_TICK)
    ax.grid(True)
    fig.tight_layout()
    st.pyplot(fig, use_container_width=False)
    plt.close(fig)


# ============================================================
# Streamlit app
# ============================================================
st.set_page_config(page_title="Drone DoE", layout="wide")
st.title("Drone Design of Experiments (DoE)")

if "doe_results" not in st.session_state:
    st.session_state.doe_results = None
if "plot_x" not in st.session_state:
    st.session_state.plot_x = None
if "plot_y" not in st.session_state:
    st.session_state.plot_y = None
if "last_loaded_cfg_name" not in st.session_state:
    st.session_state.last_loaded_cfg_name = None
if "last_loaded_cfg_size" not in st.session_state:
    st.session_state.last_loaded_cfg_size = None
if "last_saved_config_path" not in st.session_state:
    st.session_state.last_saved_config_path = None


with st.sidebar:
    st.header("Load config (JSON)")

    if st.button("Reset app state"):
        st.session_state.clear()
        st.rerun()

    uploaded_cfg = st.file_uploader("Upload config (.json)", type=["json"], key="cfg_uploader")

    if uploaded_cfg is not None:
        cfg_bytes = uploaded_cfg.getvalue()
        cfg_name = uploaded_cfg.name
        cfg_size = len(cfg_bytes) if cfg_bytes is not None else 0

        already_loaded = (
            st.session_state.last_loaded_cfg_name == cfg_name
            and st.session_state.last_loaded_cfg_size == cfg_size
        )

        if (not already_loaded) and cfg_bytes and cfg_size > 0:
            try:
                cfg = load_config_json_bytes(cfg_bytes)
                prime_sidebar_from_loaded_config(cfg)

                st.session_state.last_loaded_cfg_name = cfg_name
                st.session_state.last_loaded_cfg_size = cfg_size

                st.success(f"Loaded config: {cfg_name}")
                st.rerun()
            except Exception as e:
                st.error(f"Failed to load config: {type(e).__name__}: {e}")
        elif already_loaded:
            st.caption(f"Config loaded: {cfg_name}")
        elif cfg_size == 0:
            st.warning("Uploaded file appears empty (0 bytes). Re-upload it.")

    st.divider()
    st.header("DoE setup")

    if not OBJECTIVE_KEYS:
        st.error("OBJECTIVE_OPTIONS is empty. Add at least one objective.")
        st.stop()

    loaded_obj = st.session_state.get("objective_key_widget", None)
    obj_key = safe_default_objective_key(loaded_obj)
    obj_idx = OBJECTIVE_KEYS.index(obj_key) if obj_key in OBJECTIVE_KEYS else 0

    objective_key = st.selectbox(
        "Objective metric",
        options=OBJECTIVE_KEYS,
        index=obj_idx,
        format_func=lambda k: OBJECTIVE_KEY_TO_LABEL.get(k, k),
        key="objective_key_widget",
    )

    direction = st.radio(
        "Direction",
        ["min", "max"],
        horizontal=True,
        index=(0 if st.session_state.get("direction_widget", "min") == "min" else 1),
        key="direction_widget",
    )

    st.divider()

    default_labels = [PARAM_NAME_TO_LABEL[p] for p in DEFAULT_VARY_PARAMS if p in PARAM_NAME_TO_LABEL]
    selected_param_labels = st.multiselect(
        "Parameters to vary",
        options=PARAM_LABELS,
        default=st.session_state.get("vary_params_widget", default_labels),
        key="vary_params_widget",
    )

    param_grid: Dict[str, List[Any]] = {}
    base = design_inputs.DesignInputs()

    for lbl in selected_param_labels:
        p = PARAM_LABEL_TO_NAME[lbl]
        base_val = getattr(base, p)

        with st.expander(f"{lbl}  ({p})   base={base_val}", expanded=True):
            # bool
            if isinstance(base_val, bool):
                default_bools = st.session_state.get(f"{p}_bool", [base_val])
                vals = st.multiselect(f"Values for {p}", options=[True, False], default=default_bools, key=f"{p}_bool")
                param_grid[p] = vals if vals else [base_val]
                continue

            # numeric decision from annotations
            if is_numeric(base_val) or wants_int(p) or wants_float(p):
                is_int_param = wants_int(p)

                # Int params: keep int UI
                if is_int_param:
                    mode = st.radio("Mode", ["Range", "List"], horizontal=True, key=f"{p}_mode_int")
                    if mode == "Range":
                        lo0 = int(round(float(base_val) * 0.9))
                        hi0 = int(round(float(base_val) * 1.1))
                        if hi0 == lo0:
                            hi0 = lo0 + 1
                        lo = st.number_input("min", value=int(st.session_state.get(f"{p}_lo_int", lo0)), step=1, key=f"{p}_lo_int")
                        hi = st.number_input("max", value=int(st.session_state.get(f"{p}_hi_int", hi0)), step=1, key=f"{p}_hi_int")
                        n = st.number_input("points", min_value=1, max_value=200, value=int(st.session_state.get(f"{p}_n_int", 5)), step=1, key=f"{p}_n_int")
                        vals = make_range_values(float(lo), float(hi), int(n), force_int=True)
                        param_grid[p] = vals
                    else:
                        s0 = str(st.session_state.get(f"{p}_list_int", str(int(round(float(base_val))))))
                        s = st.text_input("Values (comma-separated)", value=s0, key=f"{p}_list_int")
                        raw_vals = parse_values_list_raw(s)
                        typed_vals = coerce_values_to_param_type(p, raw_vals)
                        param_grid[p] = typed_vals if typed_vals else [int(round(float(base_val)))]
                    continue

                # Float params: ALWAYS float UI (this is your requested fix)
                mode = st.radio("Mode", ["Range", "List"], horizontal=True, key=f"{p}_mode_float")
                if mode == "Range":
                    lo0 = float(base_val) * 0.9
                    hi0 = float(base_val) * 1.1
                    lo = st.number_input("min", value=float(st.session_state.get(f"{p}_lo_float", lo0)), key=f"{p}_lo_float")
                    hi = st.number_input("max", value=float(st.session_state.get(f"{p}_hi_float", hi0)), key=f"{p}_hi_float")
                    n = st.number_input("points", min_value=1, max_value=200, value=int(st.session_state.get(f"{p}_n_float", 5)), step=1, key=f"{p}_n_float")
                    vals = make_range_values(float(lo), float(hi), int(n), force_int=False)
                    param_grid[p] = coerce_values_to_param_type(p, vals)
                else:
                    s0 = str(st.session_state.get(f"{p}_list_float", str(float(base_val))))
                    s = st.text_input("Values (comma-separated)", value=s0, key=f"{p}_list_float")
                    raw_vals = parse_values_list_raw(s)
                    typed_vals = coerce_values_to_param_type(p, raw_vals)
                    param_grid[p] = typed_vals if typed_vals else [float(base_val)]
                continue

            # categorical / string
            s0 = str(st.session_state.get(f"{p}_cat", str(base_val)))
            s = st.text_input("Values (comma-separated)", value=s0, key=f"{p}_cat")
            raw_vals = parse_values_list_raw(s)
            param_grid[p] = raw_vals if raw_vals else [base_val]

    n_runs = 1
    for v in param_grid.values():
        n_runs *= max(1, len(v))
    st.info(f"Total runs: {n_runs}")
    if n_runs > 800:
        st.warning("Large sweep. Reduce points/params for faster iteration.")

    st.divider()

    st.header("Config save / export")

    cfg_default = datetime.now().strftime("cfg_%Y%m%d_%H%M%S")
    cfg_name = st.text_input("Config name (no extension)", value=cfg_default, key="cfg_name")

    live_cfg_for_download = build_config(objective_key, direction, param_grid)
    st.download_button(
        "Download current config as JSON",
        data=config_to_json_bytes(live_cfg_for_download),
        file_name=f"{cfg_name}.json",
        mime="application/json",
        help="Saves objective + direction + param grid. You can upload it back later.",
    )

    save_config_enabled = st.checkbox("Also save config to disk (.json)", value=False, key="save_config_enabled")
    config_folder = st.text_input("Config folder", value="doe_configs", key="config_folder")

    st.divider()

    save_results_enabled = st.checkbox("Save results to disk (.xlsx)", value=False, key="save_results_enabled")
    results_folder = st.text_input("Results folder", value="doe_out", key="results_folder")
    exp_default = datetime.now().strftime("exp_%Y%m%d_%H%M%S")
    exp_name = st.text_input("Experiment name", value=exp_default, key="exp_name")

    st.divider()
    reload_modules = st.checkbox(
        "Reload sizing code before run",
        value=True,
        help="Reloads project modules once before the sweep (behaves like a fresh `python script.py`).",
        key="reload_modules",
    )
    store_tracebacks = st.checkbox(
        "Store tracebacks for failed runs",
        value=True,
        help="Adds a 'traceback' column to the failed-runs table.",
        key="store_tracebacks",
    )

    colA, colB = st.columns(2)
    with colA:
        apply_button = st.button("Apply settings (no run)")
    with colB:
        run_button = st.button("Run DoE", type="primary")


current_config_sig = json.dumps({"objective": objective_key, "direction": direction, "param_grid": param_grid}, sort_keys=True)

if apply_button:
    st.toast("Settings applied.", icon="✅")

if run_button:
    config = build_config(objective_key, direction, param_grid)

    saved_cfg_path = None
    if bool(st.session_state.get("save_config_enabled", False)):
        try:
            saved_cfg_path = save_config_to_disk(
                config,
                folder=str(st.session_state.get("config_folder", "doe_configs")),
                filename_no_ext=str(st.session_state.get("cfg_name", "doe_config")),
            )
            st.session_state.last_saved_config_path = saved_cfg_path
        except Exception as e:
            st.warning(f"Could not save config to disk: {type(e).__name__}: {e}")

    with st.spinner("Running DoE..."):
        if reload_modules:
            reload_project_modules()
            globals()["design_inputs"] = importlib.import_module("design_inputs")
            globals()["script"] = importlib.import_module("script")

        df_ok, df_bad, meta = run_doe(
            param_grid,
            objective_key,
            direction,
            reload_modules=False,  # already reloaded above
            store_tracebacks=bool(st.session_state.get("store_tracebacks", True)),
        )
        df_sens = compute_sensitivity(df_ok, list(param_grid.keys()))

    save_path = None
    if save_results_enabled:
        save_path = save_to_xlsx(df_ok, df_bad, df_sens, config, results_folder, exp_name)

    params_used = list(param_grid.keys())
    st.session_state.plot_x = params_used[0] if len(params_used) >= 1 else None
    st.session_state.plot_y = params_used[1] if len(params_used) >= 2 else None

    st.session_state.doe_results = {
        "df_ok": df_ok,
        "df_bad": df_bad,
        "df_sens": df_sens,
        "meta": meta,
        "config": config,
        "config_sig": current_config_sig,
        "exp_name": exp_name,
        "save_path": save_path,
        "params_used": params_used,
        "saved_cfg_path": saved_cfg_path,
    }


# ============================================================
# Main display
# ============================================================
respack = st.session_state.doe_results

if respack is None:
    st.info("Load a JSON config (sidebar) or set parameters and click **Run DoE**.")
else:
    df_ok = respack.get("df_ok", pd.DataFrame())
    df_bad = respack.get("df_bad", pd.DataFrame())
    df_sens = respack.get("df_sens", pd.DataFrame())
    meta = respack.get("meta", {"n_total": 0, "n_success": 0, "n_failed": 0})
    config = respack.get("config", {"objective": DEFAULT_OBJECTIVE_KEY, "direction": "min", "param_grid": {}})
    exp_name_last = respack.get("exp_name", "experiment")

    if respack.get("config_sig") is not None and respack.get("config_sig") != current_config_sig:
        st.warning("Settings changed since the displayed experiment. Click **Run DoE** to generate new results.")

    st.subheader("Run summary")
    c1, c2, c3 = st.columns(3)
    c1.metric("Total runs", meta.get("n_total", 0))
    c2.metric("Success", meta.get("n_success", 0))
    c3.metric("Failed", meta.get("n_failed", 0))

    if respack.get("saved_cfg_path"):
        st.info(f"Config saved to: {respack['saved_cfg_path']}")
    elif st.session_state.get("last_saved_config_path"):
        st.caption(f"Last saved config: {st.session_state.last_saved_config_path}")

    if df_ok.empty and not df_bad.empty:
        st.error("No successful runs. See failures below.")
        st.dataframe(df_bad, use_container_width=True)
        st.stop()

    if df_ok.empty:
        st.info("No results to show yet.")
        st.stop()

    params_used = [p for p in respack.get("params_used", []) if p in df_ok.columns]

    best = df_ok.iloc[0]
    obj_key = config.get("objective", DEFAULT_OBJECTIVE_KEY)
    obj_label = OBJECTIVE_KEY_TO_LABEL.get(obj_key, obj_key)
    st.success(f"Best objective ({obj_label}, {config.get('direction','min')}): {float(best['objective']):.4g}")

    st.write("**Best configuration (varied params):**")
    best_cfg = pd.DataFrame(
        [{"param_label": PARAM_NAME_TO_LABEL.get(p, p), "param": p, "value": best.get(p, None)} for p in params_used]
    )
    st.dataframe(best_cfg, use_container_width=True)

    st.write("**Top 20 results:**")
    st.dataframe(df_ok.head(20), use_container_width=True)

    tab1, tab2, tab3, tab4 = st.tabs(["Plots", "Sensitivity", "All results", "Failures"])

    with tab1:
        st.write("**Objective plots**")
        if len(params_used) == 0:
            st.info("No varied parameters in this experiment.")
            plot_objective_hist(df_ok)
        elif len(params_used) == 1:
            plot_objective_1d(df_ok, params_used[0])
            plot_objective_hist(df_ok)
        else:
            if st.session_state.plot_x not in params_used:
                st.session_state.plot_x = params_used[0]

            x = st.selectbox("X axis", params_used, key="plot_x", format_func=lambda n: PARAM_NAME_TO_LABEL.get(n, n))
            y_options = [p for p in params_used if p != x]
            if not y_options:
                plot_objective_hist(df_ok)
            else:
                if st.session_state.plot_y not in y_options:
                    st.session_state.plot_y = y_options[0]

                y = st.selectbox("Y axis", y_options, key="plot_y", format_func=lambda n: PARAM_NAME_TO_LABEL.get(n, n))
                try:
                    pd.to_numeric(df_ok[x])
                    pd.to_numeric(df_ok[y])
                    plot_objective_2d(df_ok, x, y)
                except Exception:
                    st.warning("Selected X/Y must be numeric for 2D scatter. Pick numeric params.")
                plot_objective_hist(df_ok)

    with tab2:
        st.write("**Sensitivity (numeric params only)**")
        if df_sens is None or df_sens.empty:
            df_sens_local = compute_sensitivity(df_ok, params_used)
            if df_sens_local.empty:
                st.info("No numeric sensitivity could be computed.")
            else:
                st.dataframe(df_sens_local, use_container_width=True)
        else:
            st.dataframe(df_sens, use_container_width=True)

    with tab3:
        st.write("**All successful runs:**")
        st.dataframe(df_ok, use_container_width=True)

    with tab4:
        st.write("**Failed runs:**")
        st.dataframe(df_bad, use_container_width=True)

    export_bytes = dataframe_to_excel_bytes(df_ok, df_bad, df_sens if df_sens is not None else pd.DataFrame(), config)
    st.download_button(
        "Download displayed results as Excel",
        data=export_bytes,
        file_name=f"{exp_name_last}.xlsx",
        mime="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
    )

    if respack.get("save_path"):
        st.info(f"Saved to: {respack['save_path']}")
