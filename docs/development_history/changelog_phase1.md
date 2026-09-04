# CHANGELOG — Phase 1 Foundation

## Added

- Strict JSON configuration override loader.
- Rejection of unknown/derived compatibility override fields.
- Architecture, geometry, energy-state and derived-value validation.
- Explicit warnings for present H2 hardware placeholders and programme mismatch.
- Input provenance schema and starter provenance register.
- Full configuration SHA-256 hash.
- Source-file SHA-256 manifest.
- Governed result JSON archive linked to the same configuration hash.
- Current programme target file kept separate from executable hardware overrides.
- Post-run MTOW and cruise-power gates.
- Phase-1 regression/unit tests.
- Capability matrix and audit.

## Deliberately not changed

- Existing aerodynamic equations.
- Existing structural equations.
- Existing MTOW/trim convergence physics.
- Existing fuel-cell/H2 values.
- Existing motor/propeller databases.
- Existing CAD geometry.

Those remain unchanged in Phase 1 so the current baseline can be reproduced before physics refactors begin.

## Next: Phase 2

Rebuild the hybrid power architecture around accepted IE-SOAR/cylinder/battery data: 48 V-class bus, source/load power flow, battery internal resistance and voltage sag, FC dynamic limits, net-efficiency map, auxiliaries, DC/DC, thermal/derating, SOC recovery, H2 real-gas inventory/residual pressure and fault/reserve states.
