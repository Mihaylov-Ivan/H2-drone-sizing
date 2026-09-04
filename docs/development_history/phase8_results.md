# Phase 8 development result

Phase 8 was implemented inside the integrated runtime architecture rather than as `phase8_*.py` calculation files.

Added capabilities:

- aircraft-level design-variable resolver;
- neutral-point update through a baseline-calibrated tail-volume screening relation;
- structural mass/CG fixed-point closure before mission/EMS/6-DOF;
- multidisciplinary screening optimizer;
- full verification of selected candidates with EMS + nonlinear 6-DOF;
- uncertainty propagation;
- local sensitivity analysis;
- fixed-wing/hover flight-envelope export;
- test-correlation ledger/template with no invented measurements;
- explicit linear-aero validity screen above 10 m/s.

The nominal optimization found a 13.325 kg fully verified screening-pass design. A more conservative 13.620 kg robust screening case is recommended because it materially improves the uncertain 15 kg stall-margin robustness with a small cruise-energy penalty.
