# Phase 8 changelog

- Refactored structural feedback into a fixed-point closure before dynamic verification.
- Corrected wing stall/drag calibration so reference wing area/span remain fixed while candidate geometry changes.
- Added cross-subsystem propagation of wing area/span/MAC/AC, tail area, energy-system package stations and lift-row midpoint.
- Added baseline-calibrated tail-volume neutral-point screening relation.
- Added MDO screening search and final high-fidelity candidate verification.
- Added uncertainty, sensitivity, flight-envelope and calibration interfaces.
- Added nominal optimized and robust screening configuration files.
- Added post-closure static-margin, actual-layout 6-DOF, 15 kg zero-pitch motor, stall-area and structural gates.
- Added 6-DOF linear-aerodynamics applicability screen.
