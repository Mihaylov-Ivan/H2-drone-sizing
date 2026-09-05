# H2AIR Integrated Model v2.0 — Multidisciplinary Design Study

This is the Phase-8 development of the single-entry H2AIR model. Runtime code is organized by physics, **not by development phase**.

## Normal aircraft run

```bash
python run_model.py --output outputs/my_design --no-histories
```

Each run folder keeps only the essentials at the top level:

- `SUMMARY.md` — status, MTOW, gates
- `user_inputs.json` — the user file used for this run
- `resolved_configuration.json` — full internal model parameters

Everything else (ledgers, histories, CAD params, manifests) is under `details/`.

## Exterior geometry viewer

Interactive 3-D web view of the outside architecture (fuselage, wing, twin booms, tails, motors/props) driven by `cad_parameters.json`:

```bash
python viewer/serve.py outputs/my_design
```

Opens a local page with orbit controls and layer toggles. Pass any run folder (or a `cad_parameters.json` path). Re-run the model after pulling CAD export updates to pick up wing/tail vertical stations in the JSON.

## Full multidisciplinary design study

```bash
python run_model.py --design-study --output outputs/design_sweep
```

Study top level:

- `DESIGN_STUDY_SUMMARY.json` — selection, scores, uncertainty
- `user_inputs.json` — **copy-ready** file; replace `config/user_inputs.json` with this to adopt the chosen design
- `resolved_configuration.json` — full internal config for the selected design

Supporting search/verification artifacts are under `details/`.

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
