# H2AIR Multidisciplinary Design Study — Integrated v2.0

## Purpose

Integrated v2 closes structural mass/CG before evaluating aero, transition, hybrid energy and nonlinear 6-DOF behavior. Design variables propagate through one cross-subsystem resolver rather than through separate phase scripts.

The design-study command uses two fidelities:

1. physics-connected screening for many candidate geometries;
2. full integrated EMS + nonlinear 6-DOF + structural verification for selected candidates.

It then derives a deterministic stall-margin-biased candidate and compares nominal versus robust uncertainty propagation before selecting the recommended case.

## Reference deterministic optimization

The frozen 60-candidate search found 7 screening-feasible candidates. The nominal mass/energy optimum is stored in `config/optimized_screening_case.json` and closes at 13.325 kg, +8.87% static margin, 360.8 W cruise bus power and 751.3 Wh for the 120-minute mission.

## Automatic margin-biased candidate

The robust selector:

- increases wing area to an engineering-friendly margin above the 15 kg stall-area requirement;
- preserves the nominal wing aspect ratio;
- uses the upper screened horizontal-tail-area bound;
- shifts the wing aerodynamic-centre station by approximately 5 mm to retain longitudinal margin;
- re-runs the normal multidisciplinary screen;
- requires full integrated verification;
- compares uncertainty pass probability against the nominal optimum.

The resulting frozen reference is `config/robust_screening_case.json`:

- wing area: 0.890 m²
- wing span: 3.4153 m
- wing aerodynamic-centre station: 0.4050 m
- horizontal-tail area: 0.200 m²
- lift-row midpoint: 0.44231 m

Full verification returns `SCREENING_PASS`, 13.620 kg MTOW, +8.55% static margin, 362.4 W cruise bus power and 755.3 Wh mission energy.

## Uncertainty comparison

For the frozen release, both candidates use the same 32-sample Latin-hypercube set and seed 79:

- nominal optimum: 68.75% screening pass; 10 failures, all `15kg_stall_margin`;
- robust candidate: 93.75% screening pass; 2 failures, both `15kg_stall_margin`.

These distributions are explicit engineering assumptions, not measured supplier statistics or certified reliability probabilities.

## Sensitivity priorities

The local screen identifies:

- wing AC station as the strongest static-margin variable;
- lift-row midpoint as the dominant 15 kg front/rear motor-balance variable;
- wing area/span as dominant cruise-power and mission-energy geometry variables;
- component longitudinal stations as important CG/stability variables.

These are the highest-value dimensions to control and validate in CAD/test correlation.

## Remaining release-quality gaps

The optimizer only knows the current component envelopes and engineering models. Exact IE-SOAR interfaces, certified cylinder dimensions/mass, selected battery geometry/current/thermal data, installation/service clearances, H2 safety separation and ventilation, real propeller maps, nonlinear aerodynamic/control derivatives, detailed composite laminate properties, motor-mount FEA and test correlation must supersede current screening assumptions before design freeze.
