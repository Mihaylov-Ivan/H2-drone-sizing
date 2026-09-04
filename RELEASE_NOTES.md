# H2AIR Integrated Model v2.0 — Release Notes

This release completes the multidisciplinary development through the optimization/uncertainty stage and keeps runtime architecture organized by physics rather than development phase.

Key release capabilities:

- one `run_model.py` entry point;
- controlled baseline, nominal-optimum and robust-screening configs;
- 3-D component ledger, packaging, CG and inertia;
- structural fixed-point mass/CG closure;
- calibrated screening aerodynamics and lift/cruise propulsion models;
- dynamic H2 / fuel-cell / battery EMS;
- time-domain forward/reverse transition and mission model;
- nonlinear rigid-body 6-DOF/control screening;
- structural load cases, strength/stiffness sizing and feedback;
- multidisciplinary candidate search;
- deterministic robust-margin selector;
- uncertainty propagation, sensitivity and flight-envelope exports;
- calibration measurement template;
- source/provenance and model-limit documentation.

Frozen recommended screening case:

- MTOW 13.620 kg after structural closure;
- +8.55% MAC static margin;
- 362.4 W cruise bus power at 20 m/s;
- 755.3 Wh 120-minute mission energy;
- 49.3 g H2 consumption;
- 89.7% final battery SOC;
- 62.9 / 55.9 N front/rear 15 kg zero-pitch motor requirement;
- full integrated status `SCREENING_PASS`.

Using the same 32-sample uncertainty set (seed 79), the nominal optimum passes 68.75% of screening samples and the robust case passes 93.75%. These are engineering-screen probabilities, not certified reliability probabilities.

Validation at release freeze:

- pytest: 9/9 PASS
- integrated self-tests: 9/9 PASS

The package remains pre-design-freeze. Selected battery/cylinder data, exact IE-SOAR interfaces, propeller maps, nonlinear aero/control evidence, detailed composite/mount FEA and test correlation remain required.
