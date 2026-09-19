# Outputs layout

Each model run writes a clean top-level folder. Supporting files live under `details/`.

## Normal run (`python run_model.py --output outputs/my_design`)

```
outputs/my_design/
  SUMMARY.md                     # human-readable run summary
  aircraft_specification.json    # closed-aircraft source of truth (all UAV specs)
  user_inputs.json               # user file used for the run
  resolved_configuration.json    # full internal model parameters (incl. template seeds)
  details/                       # ledgers, histories, CAD, gates detail, …
```

Prefer ``aircraft_specification.json`` for geometry/mass/performance/hardware after a run.
``resolved_configuration.json`` also keeps calibration/template fields that are not closed-run values.
## Design study (`python run_model.py --design-study --output outputs/design_sweep`)

```
outputs/design_sweep/
  DESIGN_STUDY_SUMMARY.json      # selection + scores
  aircraft_specification.json    # closed-aircraft specs for the selected design
  user_inputs.json               # COPY over config/user_inputs.json to adopt the chosen design
  resolved_configuration.json    # full internal config of the selected design
  details/                       # optimization candidates, verified runs, uncertainty, …
```

Adopt a sweep result:

```powershell
Copy-Item outputs\design_sweep\user_inputs.json config\user_inputs.json
python run_model.py --output outputs/my_design --no-histories
```
