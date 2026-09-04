# Phase 7 Changelog

- Added `phase7_structures.py` structural load-case and beam/tube screening engine.
- Added `phase7_config.py` and controlled `config/phase7_structure_reference.json`.
- Added assumption/evidence register for structural quantities.
- Added Phase-6 unequal front/rear lift-motor load transfer to structural cases.
- Added 15 kg / 3 g wing-borne manoeuvre sizing.
- Added separate strength, deflection and slope gates.
- Added boom tail + rotor + pusher envelope and stiffness sizing.
- Added high-speed horizontal/vertical tail spar screens.
- Added central-pod scaled audit check.
- Added motor-mount and one-motor-out local load requirements while preserving NOT_EVALUATED status for unknown mount geometry.
- Added partial structural mass/CG feedback.
- Added Phase-7 CSV/JSON outputs and 15 new tests.
- Cumulative regression suite: **69 / 69 PASS**.
