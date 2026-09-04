# H2AIR Phase 7 — Results

## Executive result

Phase 7 shows that the current structural model is not ready to freeze at the 15 kg design ceiling. The dominant new finding is **stiffness**, not merely ultimate stress: the previous strength-oriented wing/boom sizing is too flexible once the Phase-4/6 design envelope is applied.

## Controlled load cases

The reference matrix includes:

- Phase-6 trimmed hover with unequal front/rear rotor thrust;
- 15 kg zero-pitch VTOL design thrust redistribution;
- 15 kg / 3 g symmetric wing-borne manoeuvre;
- 50 m/s horizontal-tail download screen;
- 50 m/s twin-fin side-force screen;
- conservative tail + rear-rotor boom envelope;
- pusher axial load;
- local 80 N motor-mount load;
- one-motor-out surviving-motor structural envelope.

### Governing wing case

For a 15 kg aircraft, 3.0 load factor and 3.285 m screened wing span:

**Wing root bending moment ≈ 181.21 N·m per side.**

The old 13.29 / 9.97 mm OD/ID tube gives approximately:

- bending stress: **1150.8 MPa** versus 1000 MPa screening allowable — FAIL;
- equivalent-UDL tip deflection: **389.4 mm** versus 49.3 mm limit — FAIL;
- equivalent root/tip-slope screen: **18.1°** versus 3° limit — FAIL.

The first 0.25 mm-step candidate satisfying the current constraints is:

**22.50 / 16.88 mm OD/ID**

with approximately 237 MPa bending stress, 47.4 mm equivalent tip deflection and 2.20° slope.

This is a sizing screen, not a purchase specification. The live dual-spar/skin model and eventual FEA must replace the one-tube equivalent model during the refactor.

## Lift-system structural coupling

The Phase-6 CG/rotor geometry means the 15 kg design-thrust total cannot be shared equally while maintaining zero pitch moment. The structural design load becomes approximately:

- front motor: **71.92 N** each;
- rear motor: **46.84 N** each.

Phase 7 additionally defines an **80 N/motor local structural screening limit** for mounts and surviving-motor degraded cases. With the present 50 mm mount lever assumption, the mount must react about **4.0 N·m** local bending. Mount stress is intentionally NOT_EVALUATED until CAD/fasteners/material orientation are defined.

## Boom structure

The governing conservative boom envelope is approximately **92.92 N·m/root boom**, combining the high-speed tail envelope with rear lift-rotor bending, plus 30 N axial pusher load per boom.

- Legacy strength-only auto-size 12.3 / 10.9 mm: ~1328 MPa combined stress and ~103.9 mm deflection — FAIL.
- Existing design-input geometry 35 / 31 mm: ~1.58 mm deflection — PASS as geometry, but its physical mass is inconsistent with the old reported 24.2 g total boom-tube mass.
- First 2 mm-wall screening candidate satisfying the current stiffness limit: **20 / 16 mm**, ~9.65 mm deflection — PASS.

The supplier/CAD tube mass must be reconciled before design freeze.

## Tail spars

The old tail structure was largely sized around cruise loads. Phase 7 applies explicit high-speed design envelopes. Under the deliberately conservative 50 m/s screens:

- horizontal tail root moment ≈ **21.05 N·m**;
- vertical fin root moment ≈ **23.40 N·m** per equivalent fin screen.

The current minimum spar reference fails both strength and stiffness. First screening replacements are:

- horizontal tail: **8.75 / 6.56 mm OD/ID**;
- vertical tail: **9.50 / 7.13 mm OD/ID**.

These results depend strongly on the configured |CL|=1.0 horizontal-tail and |CY|=0.8 vertical-tail assumptions. They must be replaced by a true CFD/6-DOF load envelope during the integrated refactor/optimization stage.

## Central pod

Scaling the current live pod-shell stress result from the old ~136.2 N·m wing moment to the Phase-7 ~181.2 N·m moment gives:

- bending stress ≈ **1.33 MPa** vs 14.5 MPa allowable;
- shear stress ≈ **0.71 MPa** vs 4.62 MPa allowable.

This remains a PASS screen, while the existing `structures.py` shell/frame calculation remains the source model to retain during refactor.

## Partial mass / CG feedback

Only spar/tube corrections are fed back here; skins/ribs/webs/mounts are not rescaled.

- Phase-3 reference mass: **11.0777 kg**
- screened wing-spar mass increase: **+0.8799 kg**
- boom-tube reconciliation increase: **+0.2852 kg**
- horizontal-tail spar increase: **+0.0831 kg**
- vertical-tail spar increase: **+0.0609 kg**
- partial updated mass: **12.3869 kg**
- remaining margin to 15 kg: **2.6131 kg**
- CG moves from x=0.4496 m to **x≈0.4614 m**, about **11.8 mm aft**

That aft shift is important because Phase 6 already showed the longitudinal stability problem. Structural reinforcement therefore strengthens the case for the upcoming integrated geometry/CG/stability refactor rather than freezing the present component stations.

## Interpretation

The five Phase-7 gate failures are expected and useful: they correspond to the **old/current structural references** under the expanded design envelope, not to the recommended screening candidates. The recommended wing, boom and tail candidates pass their current beam screens; motor mounts remain NOT_EVALUATED.
