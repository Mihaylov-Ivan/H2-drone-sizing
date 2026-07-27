"""
doe_study.py

Design of Experiments / parameter studies for the drone sizing model.

This is a small, expandable runner around `script.run_design(inputs)`.

Features (v1)
-------------
- Choose which `DesignInputs` parameters to vary.
- Define each parameter as either:
    1) numeric range:  name:low:high:n
    2) explicit list:  name=val1,val2,val3
- Choose an objective metric and direction (min or max).
- Run all grid combinations, collect results, rank, and plot.

Examples
--------
Minimize MTOW while sweeping wing loading and cruise speed:

    python doe_study.py \
      --objective W0_mass_kg --direction min \
      --param wing_loading:250:450:5 \
      --param speed:40:60:5

Maximize max cruise time (limited by usable battery energy) while sweeping payload and speed:

    python doe_study.py \
      --objective mission_profile.cruise_time_max_min --direction max \
      --param payload_mass:2:6:5 \
      --param speed:35:55:5
"""

from __future__ import annotations

import argparse
import itertools
import json
import os
from dataclasses import asdict
from typing import Any, Dict, List, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from design_inputs import DesignInputs
from script import run_design


def _is_int_field(name: str) -> bool:
    """Quick heuristic for int-like inputs (avoid float casting for these)."""
    return name.startswith("n_") or name.endswith("_steps")


def _linspace(low: float, high: float, n: int) -> List[float]:
    if n <= 1:
        return [float(low)]
    return [float(x) for x in np.linspace(low, high, n)]


def parse_param_spec(token: str) -> Tuple[str, List[Any]]:
    """Parse a param spec from CLI.

    Supported formats:
      - name:low:high:n
      - name=val1,val2,val3

    Values are parsed as bool/int/float when possible, otherwise kept as strings.
    """
    token = token.strip()
    if "=" in token:
        name, values_s = token.split("=", 1)
        name = name.strip()
        values: List[Any] = []
        for part in values_s.split(","):
            part = part.strip()
            if not part:
                continue

            pl = part.lower()
            if pl in {"true", "false"}:
                values.append(pl == "true")
                continue

            if part.isdigit() or (part.startswith("-") and part[1:].isdigit()):
                values.append(int(part))
                continue

            try:
                values.append(float(part))
                continue
            except ValueError:
                values.append(part)  # category/string

        if not values:
            raise ValueError(f"Empty values list for parameter '{name}'.")
        return name, values

    parts = token.split(":")
    if len(parts) != 4:
        raise ValueError(
            f"Bad param spec '{token}'. Use name:low:high:n or name=val1,val2,..."
        )
    name = parts[0].strip()
    low = float(parts[1])
    high = float(parts[2])
    n = int(parts[3])
    return name, _linspace(low, high, n)


def refresh_derived_fields(inputs: DesignInputs) -> None:
    """Keep derived fields consistent after modifying base inputs."""
    # Works if you applied the DesignInputs.__post_init__ patch.
    if hasattr(inputs, "__post_init__"):
        inputs.__post_init__()


def flatten_results(results: Any) -> Dict[str, Any]:
    """Flatten DesignResults into a dict of scalars for tabulation/plotting."""
    d = results.to_dict()

    if getattr(results, "battery", None) is not None:
        d["battery.name"] = results.battery.name
        d["battery.usable_energy_Wh"] = results.battery.usable_energy_Wh
        d["battery.meets_endurance_req"] = bool(results.battery.meets_endurance_req)

    if getattr(results, "propulsion", None) is not None:
        d["propulsion.name"] = results.propulsion.name
        d["propulsion.prop_diameter_in"] = results.propulsion.prop_diameter_in
        # convenience: total thrust
        try:
            d["propulsion.max_thrust_total_N"] = (
                float(results.propulsion.max_thrust_per_motor_N)
                * float(results.propulsion.n_propellers)
            )
        except Exception:
            pass

    # Mission profile is attached into results.inputs for printing/reporting
    if getattr(results, "inputs", None) and isinstance(results.inputs, dict):
        mp = results.inputs.get("mission_profile")
        if isinstance(mp, dict):
            for k, v in mp.items():
                if isinstance(v, (int, float, bool)) or v is None:
                    d[f"mission_profile.{k}"] = v

    return d


def get_metric(flat: Dict[str, Any], metric: str) -> float:
    if metric in flat and flat[metric] is not None:
        return float(flat[metric])
    raise KeyError(
        f"Metric '{metric}' not found. "
        f"Try: W0_mass_kg, L_over_D_cruise, V_stall, cruise_power_elec_W, "
        f"mission_profile.cruise_time_max_min, mission_profile.total_energy_Wh, ..."
    )


def run_grid_study(
    base_inputs: DesignInputs,
    param_grid: Dict[str, List[Any]],
    objective: str,
    direction: str,
    top_n: int = 10,
    outdir: str = "doe_out",
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    os.makedirs(outdir, exist_ok=True)

    names = list(param_grid.keys())
    values_lists = [param_grid[n] for n in names]
    combos = list(itertools.product(*values_lists)) if values_lists else [()]

    successes: List[Dict[str, Any]] = []
    failures: List[Dict[str, Any]] = []

    total = len(combos)
    for i, combo in enumerate(combos, start=1):
        inputs = DesignInputs(**asdict(base_inputs))
        for n, v in zip(names, combo):
            if _is_int_field(n) and isinstance(v, float) and float(v).is_integer():
                v = int(v)
            setattr(inputs, n, v)

        refresh_derived_fields(inputs)

        try:
            results = run_design(inputs)
            flat = flatten_results(results)
            obj = get_metric(flat, objective)

            row = {k: getattr(inputs, k) for k in names}
            row.update({"objective": obj, "objective_name": objective})

            # common “headline” outputs
            for k in [
                "W0_mass_kg",
                "drag_cruise_N",
                "cruise_power_elec_W",
                "L_over_D_cruise",
                "V_stall",
                "mission_profile.cruise_time_max_min",
                "mission_profile.total_energy_Wh",
            ]:
                if k in flat:
                    row[k] = flat[k]

            for k in ["battery.name", "propulsion.name"]:
                if k in flat:
                    row[k] = flat[k]

            successes.append(row)

        except Exception as e:
            failures.append({
                **{k: getattr(inputs, k) for k in names},
                "error": f"{type(e).__name__}: {e}",
            })

        if i % max(1, total // 10) == 0 or i == total:
            print(f"Progress: {i}/{total}")

    df_ok = pd.DataFrame(successes)
    df_bad = pd.DataFrame(failures)

    if not df_ok.empty:
        ascending = direction.lower() == "min"
        df_ok = df_ok.sort_values("objective", ascending=ascending).reset_index(drop=True)

        print("\n=== BEST CONFIGURATION ===")
        best = df_ok.iloc[0].to_dict()
        print(f"objective ({objective}, {direction}) = {best['objective']:.4g}")
        for n in names:
            print(f"  {n} = {best[n]}")

        print("\nTop results:")
        print(df_ok.head(top_n).to_string(index=False))

        excel_path = os.path.join(outdir, "doe_results.xlsx")
        with pd.ExcelWriter(excel_path, engine="openpyxl") as writer:
            df_ok.to_excel(writer, sheet_name="success", index=False)
            df_bad.to_excel(writer, sheet_name="failed", index=False)

            ws = writer.book["success"]
            ws.freeze_panes = "A2"
            ws.auto_filter.ref = ws.dimensions

        print(f"Saved Excel results to: {excel_path}")
    with open(os.path.join(outdir, "study_config.json"), "w", encoding="utf-8") as f:
        json.dump(
            {"objective": objective, "direction": direction, "param_grid": param_grid},
            f,
            indent=2,
        )

    return df_ok, df_bad


def plot_study(df_ok: pd.DataFrame, param_names: List[str], outdir: str) -> None:
    if df_ok.empty:
        print("No successful runs to plot.")
        return

    if len(param_names) == 1:
        x = param_names[0]
        dfp = df_ok.sort_values(x)
        plt.figure()
        plt.plot(dfp[x], dfp["objective"], marker="o")
        plt.xlabel(x)
        plt.ylabel(dfp["objective_name"].iloc[0])
        plt.title("Objective vs parameter")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"plot_{x}.png"), dpi=150)
        plt.show()
        return

    if len(param_names) == 2:
        x, y = param_names
        plt.figure()
        sc = plt.scatter(df_ok[x], df_ok[y], c=df_ok["objective"], marker="o")
        plt.xlabel(x)
        plt.ylabel(y)
        plt.title("Objective over 2-parameter sweep")
        plt.colorbar(sc, label=df_ok["objective_name"].iloc[0])
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(outdir, f"plot_{x}_vs_{y}.png"), dpi=150)
        plt.show()
        return

    plt.figure()
    plt.hist(df_ok["objective"].values, bins=min(30, max(5, len(df_ok) // 5)))
    plt.xlabel(df_ok["objective_name"].iloc[0])
    plt.ylabel("count")
    plt.title("Objective distribution")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, "plot_objective_hist.png"), dpi=150)
    plt.show()


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="DoE / parameter study runner")
    p.add_argument("--objective", default="W0_mass_kg")
    p.add_argument("--direction", choices=["min", "max"], default="min")
    p.add_argument("--param", action="append", default=[])
    p.add_argument("--outdir", default="doe_out")
    p.add_argument("--top", type=int, default=10)
    return p


def main() -> None:
    args = build_arg_parser().parse_args()

    base_inputs = DesignInputs()
    refresh_derived_fields(base_inputs)

    grid: Dict[str, List[Any]] = {}
    for token in args.param:
        name, values = parse_param_spec(token)
        grid[name] = values

    print("Parameters to sweep:")
    if not grid:
        print("  (none) -> single baseline run")
    for k, v in grid.items():
        print(f"  {k}: {len(v)} values")

    df_ok, df_bad = run_grid_study(
        base_inputs=base_inputs,
        param_grid=grid,
        objective=args.objective,
        direction=args.direction,
        top_n=args.top,
        outdir=args.outdir,
    )

    print(f"\nSaved: {os.path.join(args.outdir, 'doe_results.xlsx')}")

    plot_study(df_ok, list(grid.keys()), args.outdir)


if __name__ == "__main__":
    main()
