# Phase 3 Changelog

## Added

- `phase3_mass_geometry.py`
  - 3-D component ledger
  - take-off/end mass properties
  - full inertia tensor and products of inertia
  - H2 depletion sweep
  - packaging envelopes and collision detection
  - lift/pusher propeller clearance gates
  - MTOW, lateral-CG, inertia and reference-static-margin gates
- `phase3_config.py`
  - Phase-2-to-Phase-3 hardware mass reconciliation
  - no silent replacement of unknown battery/cylinder mass
- `phase3_closure.py`
  - generic relaxed fixed-point closure with convergence history
- `run_phase3.py`
- `phase3_layout_sweep.py`
- `config/phase3_geometry_reference.json`
- `config/phase3_component_ledger.json`
- `tests/test_phase3_mass_geometry.py`
- `run_phase3_selftests.py`
- Phase-3 audit/results/capability/integration documentation.

## Important behavioral change

Mass stations are no longer treated as if they prove a physical arrangement. Components with known envelopes are now checked for physical overlap, and missing dimensions produce `NOT_EVALUATED`, not `PASS`.
