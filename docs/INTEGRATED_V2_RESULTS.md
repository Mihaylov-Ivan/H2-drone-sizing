# H2AIR Integrated v2.0 — Frozen Reference Results

## Recommended screening configuration

The recommended configuration is `config/robust_screening_case.json`.

Full integrated verification:

- status: `SCREENING_PASS`
- reference/pre-structural mass: 12.208 kg
- structurally closed MTOW: 13.620 kg
- mass margin to 15 kg: 1.380 kg
- closed CG x: 0.4281 m
- static margin: +8.55% MAC
- cruise bus power at 20 m/s: 362.4 W
- 120-minute mission energy: 755.3 Wh
- H2 used: 49.3 g
- final battery SOC: 89.7%
- forward transition duration: 16.26 s
- reverse transition duration: 19.96 s
- 15 kg zero-pitch front/rear motor requirement: 62.9 / 55.9 N per motor
- recommended wing spar screen: 22.5 / 16.875 mm OD/ID
- recommended boom screen: 22.25 / 18.25 mm OD/ID
- actual-layout nonlinear 6-DOF: PASS

## Nominal optimum comparison

The deterministic 60-candidate reference search found 7 screening-feasible candidates. The nominal best candidate closes at:

- MTOW: 13.325 kg
- static margin: +8.87% MAC
- cruise bus power: 360.8 W
- 120-minute mission energy: 751.3 Wh
- H2 used: 49.1 g
- final battery SOC: 90.1%
- maximum 15 kg zero-pitch motor requirement: 63.7 N
- full integrated status: `SCREENING_PASS`

The nominal optimum is lighter, but it sits too close to the 15 kg stall-area screen.

## Apples-to-apples uncertainty comparison

Both candidates were propagated through the same 32-sample Latin-hypercube set with seed 79.

### Nominal optimum

- screening pass probability: 68.75% (22/32)
- failures: 10 × `15kg_stall_margin`

### Recommended robust case

- screening pass probability: 93.75% (30/32)
- failures: 2 × `15kg_stall_margin`
- P95 MTOW: ~13.87 kg
- P95 cruise bus power: ~383.8 W
- P95 mission energy: ~791.0 Wh
- P95 maximum 15 kg motor requirement: ~63.46 N

These probabilities quantify only the uncertainty assumptions currently encoded in the model. They are not certification or reliability probabilities.

## Why the robust case is selected

The robust case gives up roughly 0.295 kg of nominal mass margin to gain materially more stall-area robustness while retaining:

- positive static margin,
- collision-free screening packaging,
- 15 kg lift-motor pitch-balance feasibility,
- cruise-power margin,
- 120-minute energy margin,
- nonlinear 6-DOF control feasibility,
- structural screening feasibility.

It is therefore the recommended geometry for the next CAD/supplier/test iteration, but not a design-freeze release.
