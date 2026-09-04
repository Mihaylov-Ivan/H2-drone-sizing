# H2AIR Phase 7 — Engineering Audit

## What is now covered

Phase 7 adds a traceable structural load-case matrix, separate strength/stiffness checks, explicit 15 kg / 3 g wing sizing, Phase-6 unequal rotor-load transfer, boom tail/rotor/pusher envelopes, high-speed tail-spar screens, local motor-mount requirements, one-motor-out local loads, central-pod stress scaling and partial structural mass/CG feedback.

## Key corrections to earlier model behavior

1. **Strength-only sizing is no longer accepted.** The old wing and boom sections can be rejected for flexibility even if they approach a nominal stress allowable.
2. **15 kg is used as the structural design mass ceiling** for the main envelope instead of the ~11 kg reference mass only.
3. **Rotor loads are not assumed symmetric front/rear.** Phase-6 zero-pitch allocation is used.
4. **Tail spars are no longer accepted based on cruise-only loading.** Explicit high-speed load envelopes are applied.
5. **Unknown motor-mount geometry stays NOT_EVALUATED.** No fabricated pass/fail stress result is generated.
6. **Structural mass is fed back to CG.** The partial reinforcement screen moves CG aft, which must be resolved with stability and packaging during the refactor.

## Open evidence / engineering items before hardware freeze

- selected CFRP tube supplier data: OD/ID, measured mass per metre, longitudinal modulus, transverse/shear data and strength allowables;
- laminate/layup definition where custom composite parts are used;
- wing spar joint/carry-through design and insert/bearing loads;
- lift-motor mount CAD, bolt pattern, inserts, printed/composite orientation, bearing/pull-through and fatigue;
- actual tail aerodynamic/control load envelope from CFD/AVL + 6-DOF;
- detailed spanwise wing aerodynamic load distribution rather than equivalent-UDL root-moment conversion;
- torsional rigidity of the complete wing closed section and control-surface alignment under load;
- boom-tail joints and pusher load path;
- landing/ground-handling/proof loads;
- modal/vibration/propeller excitation analysis;
- local buckling, crippling, adhesive and fastener checks;
- static proof testing and correlation to the model.

## Important inconsistency retained as a hard warning

The design inputs describe 35 / 31 mm booms, while an older auto-size/report represented the boom tubes as about 12.3 / 10.9 mm and only ~24 g total tube mass. Those cannot all describe the same physical component. Phase 7 therefore reports the 35 / 31 mm geometry separately from the legacy mass and does not silently merge them.

## Refactor note

This is the final phase-oriented development package requested before refactor. The next package should consolidate the live model into functional modules (`geometry`, `mass_properties`, `aerodynamics`, `propulsion`, `hybrid_power`, `mission`, `transition`, `stability`, `sixdof`, `structures`, `convergence`, `reporting`) with one input object, one run entry point and one results object. Phase documents should move under development history rather than remain part of normal execution.
