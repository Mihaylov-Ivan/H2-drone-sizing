# Phase 3 Engineering Audit

## Scope

Phase 3 addresses the model capabilities that must exist before detailed aero/propulsion optimisation can be trusted: physical mass stations, 3-D CG/inertia, packaging envelopes, propeller geometric clearance, fuel-depletion mass movement and closure infrastructure.

## Preserved from the current model

The current H2AIR model already has a strong itemized longitudinal mass ledger, twin-boom/central-pod geometry, lift and pusher stations, hydrogen burn CG movement, wing/tail structural masses and a conventional-tail stability model. Phase 3 preserves those values as the reference rather than rebuilding the aircraft from unrelated assumptions.

## Corrected modeling weakness: CG station != packaging proof

The prior model could assign multiple large components to longitudinal stations and produce a valid CG even when their physical envelopes occupied the same volume. Phase 3 separates:

1. mass-property station;
2. physical component envelope;
3. packaging zone;
4. collision result;
5. evidence status.

This prevents a numerically good CG from implying that the aircraft can actually be assembled.

## Corrected modeling weakness: 1-D inertia is insufficient for VTOL dynamics

The 4+1 aircraft has large spanwise masses: four lift propulsors, ESCs, motor mounts, twin booms and twin fins. Phase 3 therefore calculates full x/y/z CG and the complete symmetric inertia tensor instead of deriving only a longitudinal CG.

## New hard design flags

- current 18-inch lift-prop placeholder overlaps its same-side front/rear disk;
- current major hardware stations cause pod-envelope collisions;
- current inherited neutral point + updated CG gives a negative static-margin cross-check;
- final MTOW remains incomplete until selected battery and cylinder masses are inserted.

## Known limitations intentionally retained

- selected 12S battery mass/envelope is open;
- selected 3.0 L / 350 bar cylinder mass/envelope is open;
- exact IE-SOAR integration envelope is not confirmed in the current source set;
- many avionics/service/ducting dimensions are absent;
- boom vertical station and vertical-fin root z are preliminary for inertia;
- package checker uses axis-aligned screening envelopes and does not model tapered pod skins, frames, brackets, wiring bend radii, H2 ventilation zones, thermal spacing or maintenance access;
- propeller disks are rigid geometric envelopes and do not include blade flex or dynamic deflection;
- physical coupled closure is adapter-ready but not connected to the inaccessible original source-tree runtime in this package.

## Engineering consequence

The aircraft is still viable as a design programme, but the current geometry should **not** be frozen for CAD/manufacture. Phase 4 should jointly re-close wing/motor positions, actual lift-prop diameter, stopped/folding prop drag, pusher performance and longitudinal stability before detailed structural/CAD release.
