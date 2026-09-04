# H2AIR Integrated Model v2.0 — Multidisciplinary Design Study

This is the Phase-8 development of the single-entry H2AIR model. Runtime code is organized by physics, **not by development phase**.

## Normal aircraft run

```bash
python run_model.py
```

Use another configuration:

```bash
python run_model.py --config config/robust_screening_case.json --output outputs/my_case
```

## Full multidisciplinary design study

```bash
python run_model.py --design-study
```

That command performs:

`design variables -> cross-subsystem resolver -> screening MDO search -> full EMS/6-DOF/structure verification -> uncertainty propagation -> local sensitivity -> flight-envelope export -> calibration template`

Use `--samples`, `--uncertainty-samples` and `--seed` to change study size/reproducibility.

## Primary configurations

- `config/baseline.json` — inherited integrated baseline; now correctly exposes its closed-mass geometry/stability shortcomings.
- `config/optimized_screening_case.json` — nominal mass/energy-optimized screening candidate from the Phase-8 search.
- `config/robust_screening_case.json` — recommended margin-biased candidate after uncertainty review.
- `config/design_space.json` — current screening optimization bounds.
- `config/uncertainty_register.json` — explicit uncertainty assumptions.
- `config/calibration_measurements_template.csv` — empty template for real test data; no measurements are invented.

## Calculation chain

`config -> design-variable coupling -> BOM/packaging -> structural fixed-point closure -> aero/propulsion -> transition/mission -> H2/battery EMS -> actual-layout 6-DOF/control -> final structures -> gates/report`

Structural mass is now closed **before** mission, EMS and 6-DOF verification, so the aircraft is no longer evaluated dynamically at an unrealistically light pre-reinforcement mass.

## Current recommended screening case

The robust screening case uses approximately 0.890 m² wing area, 3.415 m span, 0.200 m² horizontal-tail area and a lift-row midpoint near 0.442 m. The full integrated run returns `SCREENING_PASS`, but this is not a design-freeze release. Selected battery/cylinder mass, exact component envelopes, motor-mount FEA/allowables, real propeller maps and aero/control correlation are still required.

See `docs/DESIGN_STUDY.md` and `docs/MODEL_LIMITATIONS.md`.
