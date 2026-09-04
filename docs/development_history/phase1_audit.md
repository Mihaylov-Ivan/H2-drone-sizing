# Phase 1 Audit — Current H2AIR-4+1 Model

## Executive conclusion

The current H2 model is **not an empty or primitive starting point**. It already contains major pieces of the correct 4+1 aircraft: a central pod + twin-boom architecture, four lift propulsors and one rear pusher, detailed wing/tail structure, configuration drag build-up, conventional static stability/control calculations, an H2/fuel-cell/buffer-battery mission allocator, mass/CG accounting and CAD parameter export.

The largest immediate weakness is therefore **model governance and architectural consistency**, not lack of equations. Several modules still carry compatibility assumptions from the older aircraft model, and current fuel-cell/storage/bus values are explicitly provisional. The present convergence implementation mainly closes MTOW/tail trim rather than the complete mass-CG-power-energy-geometry fixed point required by the capability specification.

## Existing strengths to preserve

1. Explicit H2AIR-4+1 topology: 4 lift + 1 cruise, one horizontal tailplane, twin fins and twin booms.
2. Explicit lift motor x/y/z locations and rotation directions.
3. Component-level cruise drag build-up including pod, booms, lift nacelles/supports, stopped lift propellers, cooling and junction drag.
4. Stopped/folded/windmilling lift-prop cruise-state model.
5. Dedicated vertical-lift and rear-pusher propulsion selection paths.
6. Hybrid stage-level energy allocation using the fuel cell as base source and the battery for excess/peak demand.
7. H2, tank/regulator, fuel-cell, battery, propulsion, avionics and structural masses included in the aircraft mass model.
8. Takeoff/end-of-mission mass accounting and hydrogen depletion treatment.
9. Wing, tail, central-pod and twin-boom structural calculations including CFRP tube checks.
10. Longitudinal static margin, directional stability, elevator/rudder manoeuvre screening and asymmetric-thrust checks.
11. Inventor/CAD parameter export from the model.

## Phase-1 critical findings

### P1-01 — Legacy geometry compatibility remains in the source of truth

`fuselage_length`, `fuselage_OD`, legacy circular body properties, `n_propellers`, legacy single-battery and single-propulsion aliases are still present. This is acceptable temporarily but dangerous if a downstream module interprets an equivalent value as the real rounded-rectangular pod.

**Action:** do not delete them in Phase 1. Tag them as compatibility fields and migrate every consumer together in Phase 3.

### P1-02 — Current H2 hardware values are not aligned with the newer programme configuration

Current model values include an explicitly named placeholder 800 W fuel-cell model, a 300 bar / 120 g H2 placeholder storage system and a 22.2 V bus. The newer H2AIR-4+1 application describes a preferred IE-SOAR 800 package with certified 3.0 L / 350 bar storage and a 48 V-class / preliminary 12S buffer architecture.

**Action:** flag the mismatch now; do not silently overwrite. Phase 2 will rebuild the accepted supplier component records and hybrid bus model.

### P1-03 — One endurance field mixes target and pass/fail meaning

The current model uses `cruise_endurance_min = 90 min`. The newer programme has two different requirements: **>60 min minimum project success** and **>=120 min target**.

**Action:** Phase 2/5 should introduce minimum, target and stretch endurance gates rather than one ambiguous number.

### P1-04 — Convergence is not yet fully coupled

The H2 `convergence.py` is primarily a mass relative-change checker and plotting utility. `script.run_design` does have an outer MTOW loop and inner trim-consistency loop, which is useful, but this is not yet the full fixed-point convergence required across MTOW, CG/inertias, propulsion selection, H2/battery sizing, structural geometry, cruise power, transition energy and packaging.

**Action:** replace with an explicit convergence-state object in Phase 3/5, with residuals and convergence history for every coupled state.

### P1-05 — Propulsion databases remain screening-quality

The generic propulsion database uses approximate static maximum thrust/power records. Current selection logic checks static max thrust and power. This is insufficient for final lift thermal sizing and especially for the cruise pusher, which must be selected from forward-speed efficiency/thrust data.

**Action:** Phase 4 introduces data-quality-labelled measured/supplier maps and interpolation bounds.

### P1-06 — Transition is still a screening model

The mission model uses a stall-margin handoff speed but also a prescribed transition time and average lift fraction. It is useful for preliminary energy sizing, but it is not yet a force-balanced dynamic transition model.

**Action:** Phase 5 replaces the fixed average lift fraction with time-integrated wing lift, rotor unloading and pitch/altitude/control constraints; Phase 6 then validates it in H2-specific 6-DOF.

### P1-07 — Output traceability is incomplete

The existing H2 script writes a text report and exposes a flat `results.to_dict()`, but a run is not yet guaranteed to carry a project version, configuration hash, source hashes, provenance status and validation gates.

**Action:** implemented by this Phase-1 overlay.

### P1-08 — Machine-specific CAD output path

The current Inventor XML path is a local Windows path.

**Action:** use repository-relative or CLI/configured output roots. The Phase-1 wrapper archives source/config hashes independently of CAD location.

## Phase 1 acceptance criteria

Phase 1 is complete when the merged repository can:

- instantiate current default `DesignInputs` without changing physics;
- reject unknown JSON override keys;
- validate architecture, geometry, energy-state bounds and derived-value consistency;
- clearly flag placeholder FC/H2/bus values;
- run the existing H2 sizing model through `run_phase1.py`;
- produce a configuration SHA-256 and source-file SHA-256 manifest;
- save governed JSON results tied to the same hash;
- pass the Phase-1 governance unit tests;
- preserve the original H2 baseline output to numerical tolerance.

The final bullet requires the actual source package to be mounted/executed; the present attachment runtime did not expose the uploaded ZIP bytes, so this overlay has been syntax/unit tested independently but cannot yet perform the repository-level regression run here.
