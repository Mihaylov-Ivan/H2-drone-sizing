# Outputs layout

Each model run writes a clean top-level folder. Supporting files live under `details/`.

## Normal run (`python run_model.py --output outputs/my_design`)

```
outputs/my_design/
  SUMMARY.md                     # read this first
  user_inputs.json               # user file used for the run
  resolved_configuration.json    # full internal model parameters
  details/                       # ledgers, histories, CAD, gates detail, …
```

## Design study (`python run_model.py --design-study --output outputs/design_sweep`)

```
outputs/design_sweep/
  DESIGN_STUDY_SUMMARY.json      # selection + scores
  user_inputs.json               # COPY over config/user_inputs.json to adopt the chosen design
  resolved_configuration.json    # full internal config of the selected design
  details/                       # optimization candidates, verified runs, uncertainty, …
```

Adopt a sweep result:

```powershell
Copy-Item outputs\design_sweep\user_inputs.json config\user_inputs.json
python run_model.py --output outputs/my_design --no-histories
```
