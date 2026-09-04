# Phase 5 Engineering Audit

## What was replaced

Phase 4 represented forward and reverse transition as fixed-duration hover-power multipliers. Phase 5 removes those fixed transition power assumptions from the detailed mission path and computes transition duration and power from the aircraft state.

## New physics/control layers

- horizontal and vertical point-mass equations of motion
- wing angle of attack, CL limit, lift and induced drag through transition
- Phase-4 calibrated parasite drag retained
- powered lift-prop forward-drag blending toward folded state
- pusher Ct/Cp/J/RPM operating point during acceleration
- pusher electrical-power saturation
- lift and pusher first-order response lags
- altitude-hold lift-thrust command and saturation
- automatic lift-motor unloading as wing lift grows
- pitch schedule with rate and response limits
- reverse-transition lift spool-up as wing lift decays
- handover/capture criteria
- altitude-loss, sink-rate, stall-margin, energy and power gates
- dynamic power history passed to the Phase-2 EMS

## Important evidence boundaries

The following remain assumptions and must not be treated as verified aircraft performance:

- CL-alpha and zero-lift angle
- post-stall behavior
- powered-transition lift-prop drag
- pusher generic Ct/Cp map
- pusher 1.2 kW screen
- motor time constants
- transition controller gains/pitch schedule
- 55 N current lift placeholder
- 65 N/motor 15 kg design case

## What Phase 5 does not model

Phase 5 does not solve pitch/roll/yaw rotational dynamics. Pitch is a commanded/rate-limited kinematic state. Therefore the model cannot yet prove that the elevator/tail, CG/static margin and propulsion moments can actually produce the requested attitude history. That is intentionally deferred to Phase 6 rather than hidden inside an assumed pitch response.

No hardware is released from Phase 5. Battery current and thermal limits remain NOT_EVALUATED until the selected pack is known, and pusher/lift hardware requires measured maps.

## Regression

All Phase 1–5 automated tests pass: **44 / 44**.
