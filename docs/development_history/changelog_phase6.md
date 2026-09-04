# Phase 6 Changelog

Added a screening-fidelity nonlinear rigid-body 6-DOF model, nonlinear cruise trim, aerodynamic/control derivatives, full lift-motor moment allocation, actuator dynamics, forward/reverse transition trajectory tracking, gust testing and control/stability gates.

Key model correction: Phase 5 treated the 65 N/motor candidate using aircraft-total lift margins. Phase 6 now accounts for the lift-rotor centroid being aft of CG. At the Phase-4 15 kg design total thrust (237.52 N), zero-pitch allocation requires about 71.92 N on each front motor and 46.84 N on each rear motor. Therefore the 65 N candidate does not close the 15 kg design-margin case with the current rotor/CG geometry.
