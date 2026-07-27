# UAV DoE App

This Streamlit app auto-discovers:
- editable inputs from `DesignInputs`
- output metrics from `script.run_design(...)`

## What it fixes compared with the old version
- No hard-coded input dropdown list
- No hard-coded objective list
- Any discovered model metric can be used as the objective
- Constraints can be applied on any discovered metric
- Feasible designs are ranked ahead of infeasible ones
- Exports config as JSON and results as Excel

## Run

Place `doe_app_ready.py` in the same folder as your UAV model modules, then run:

```bash
streamlit run doe_app_ready.py
```

## Typical workflow

1. Set your base `DesignInputs` values in the app.
2. Select the parameters to vary.
3. Choose one of these for each swept parameter:
   - `list`
   - `min / max / step`
   - `min / max / points`
4. Pick an objective metric, for example:
   - `aircraft.W0_mass_kg`
   - `aircraft.static_margin_percent_MAC`
   - `wing.geometry.span_m`
5. Add constraints in JSON, for example:

```json
[
  {"metric": "aircraft.static_margin_percent_MAC", "op": ">=", "value": 5},
  {"metric": "wing.geometry.span_m", "op": "<=", "value": 2.4}
]
```

6. Run the DoE.

## Notes about linked inputs

Your current `DesignInputs` contains some values that were originally defined from other inputs at class-definition time, for example:
- `fuselage_body_section_A_length`
- `fuselage_body_section_B_length`
- `fuselage_body_section_C_length`
- `fuselage_longeron_diameter`

The app therefore includes **Auto-refresh linked geometry inputs**. When enabled, it recomputes a small set of those linked fields before each run so that changing `c_root`, `fuselage_length`, `transition_fit_clearance`, etc. does not leave stale geometry behind.

If you want to manually control those linked fields yourself, disable that option.

## Constraint operators
- `>=`
- `<=`
- `>`
- `<`
- `==`
- `between`

Example of `between`:

```json
[
  {"metric": "aircraft.static_margin_percent_MAC", "op": "between", "min": 5, "max": 15}
]
```

## Recommendation

For optimisation goals like:
- minimum MTOW
- minimum static margin above a safe limit
- maximum span under a packaging cap

use:
- one metric as the objective
- the rest as constraints

Example:
- objective: `aircraft.W0_mass_kg`, direction `min`
- constraints:
  - static margin `>= 5`
  - wing span `<= 2.3`
