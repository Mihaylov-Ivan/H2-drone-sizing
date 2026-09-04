# v4 Fresh Optimization Check

A fresh multidisciplinary screening search was run after introducing the v4 12S AUTO hardware selection and manufacturing/airfoil/CAD updates.

Configuration:

- 32 Latin-hypercube samples;
- deterministic current-design seed;
- 8 local-refinement candidates;
- 41 total screened candidates;
- seed 44.

Two candidates were screening-feasible.

## Screening rank 1

- screening score: 14.0969;
- wing span: 3.4792 m;
- horizontal-tail span: 0.9115 m;
- vertical-tail height: 0.5199 m;
- wing AC x: 0.3524 m;
- lift-row center x: 0.4208 m;
- screening static margin: 7.40% MAC.

Full integrated verification:

- closed MTOW: 13.858 kg;
- cruise bus power: 375.4 W;
- 120-minute mission energy: 776.0 Wh;
- current-layout static margin: 7.40% MAC;
- reverse hover-capture speed: 2.82 m/s — PASS;
- reverse maximum pitch-tracking error: 5.59 deg versus 5.0 deg limit — FAIL.

## Screening rank 2 / current AUTO geometry

Full integrated verification:

- closed MTOW: 14.053 kg;
- cruise bus power: 374.7 W;
- 120-minute mission energy: 775.1 Wh;
- static margin: 10.57% MAC;
- reverse hover-capture speed: 4.39 m/s versus 3.0 m/s limit — FAIL;
- reverse maximum pitch-tracking error: 6.98 deg versus 5.0 deg limit — FAIL.

## Interpretation

The fresh search confirms that the updated 12S hardware/mass architecture remains feasible in mass, cruise power, energy, static stability and normal forward-transition screening, but the current reverse-transition control/dynamics model is the limiting integrated gate. The package preserves this failure rather than relaxing the gate merely to produce a nominal PASS.
