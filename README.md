# H2AIR Integrated Model v4.1

Single-entry H2AIR hydrogen-hybrid VTOL/fixed-wing design model. Runtime code is organized by physics rather than development phase.

## Normal run

```bash
python run_model.py --config config/user_inputs.json --output outputs/my_design --no-histories
```

Top-level run files:

- `SUMMARY.md` — human-readable status, geometry, mass, cooling, mission and gates;
- `user_inputs.json` — controlled user file used for the run;
- `resolved_configuration.json` — full resolved internal configuration.

Supporting outputs are in `details/`, including `model_results.json`, component/mass ledgers, hardware selection, airfoil/CAD exports and `FUEL_CELL_COOLING.json`.

## Fuel-cell cooling (v4.1)

The IE-SOAR integration assumes the built-in fans **suck air through the stack**. Default air path:

`left NACA-style flush inlet -> fuel-cell stack -> built-in fans -> right direct exhaust grille`

The cooling screen estimates waste heat, required airflow, external duct/grille pressure loss and the fan/system operating point. Hover/ground cooling receives no ram-air credit. Supplier fan-flow/pressure data are not yet available, so fan capacity remains an explicit engineering assumption and the fan-curve evidence gate is `NOT_EVALUATED`.

Cooling geometry is user-editable under `fuel_cell_cooling` in `config/user_inputs.json` and is exported to CAD. See `docs/FUEL_CELL_COOLING.md`.

## CAD workflow

Run output contains:

```text
outputs/my_design/details/
    cad_parameters.json
    params.xml
    params.xlsx
```

The cooling export includes inlet/exhaust side, panel/slot geometry, NACA-style recess/ramp, fan locations, shroud depth, plenum gaps and minimum fuselage width.

## 3-D viewer

```bash
python viewer/serve.py outputs/my_design
```

The viewer draws the cross-fuselage cooling architecture on the fuselage sides: NACA-style inlet ramp and slots, exhaust louvre grille, fan discs/shroud and plenum-gap volumes, all driven by the `fc_cooling_*` CAD parameters.

## Concept images (optional)

After a sizing run, you can ask OpenAI to generate concept renders from the closed aircraft specs (isometric, top/side/perspective, fuselage cutaway, cooling airflow):

```bash
pip install -r requirements-concept-images.txt
set OPENAI_API_KEY=sk-...
python run_model.py --config config/user_inputs.json --output outputs/my_design --no-histories --concept-images
```

Images and prompts land under `outputs/my_design/details/concept_images/`. With `--design-study --concept-images`, images are generated once for the selected verified design folder only. Override the key or model with `--openai-api-key` / `--concept-image-model` (default `gpt-image-1`). Core sizing does not require the `openai` package.

## Design study

```bash
python run_model.py --design-study --output outputs/design_sweep
```

The existing LHS + local-refinement + integrated verification workflow is retained. Cooling-opening drag is included in the aerodynamic screening when cooling is enabled.

## Validation

Run:

```bash
python run_selftests.py
pytest -q
```

This remains a screening/preliminary-design model. Real fan curves, propeller maps, battery limits, motor-mount release analysis and nonlinear aero/test correlation are still required before design freeze.
