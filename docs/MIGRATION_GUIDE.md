# Migration from cumulative Phase packages

## Before
Normal development used `run_phase2.py`, `run_phase3.py`, ..., `run_phase7.py` and separate phase configuration files.

## Now
Use only:

```bash
python run_model.py
```

The main editable case is `config/baseline.json`. The top-level `design` section is the aircraft-level source of truth. The integrated resolver propagates those settings to power, mass properties, aero/propulsion, mission/transition, 6-DOF and structures.

Results are written to `outputs/latest/`.

Historical phase reports are kept under `docs/development_history/` for traceability only. They are not part of the normal execution path.
