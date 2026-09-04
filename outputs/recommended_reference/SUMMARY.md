# H2AIR Integrated Model — Run Summary

**Model status:** SCREENING_PASS

## Aircraft state

- Pre-structure screening MTOW: **12.208 kg**
- Post-structure screening MTOW: **13.620 kg**
- Post-structure CG x: **0.4281 m**
- Remaining mass margin to 15 kg: **1.380 kg**
- Current-layout static margin: **8.55% MAC**

## Performance / mission

- Folded-prop cruise bus power: **362.4 W**
- Forward transition: **16.26 s**
- Reverse transition: **19.96 s**
- 120-min mission energy: **755.3 Wh**
- H₂ used: **49.3 g**
- Final battery SOC: **89.7%**

## Structural screen

- Recommended wing spar: **22.50/16.88 mm OD/ID**
- Recommended boom: **22.25/18.25 mm OD/ID**
- Motor-mount limit thrust: **80.0 N/motor**

## Integrated gates

- **PASS** — mass_ceiling: Structurally closed mass 13.620 kg vs 15.0 kg ceiling.
- **PASS** — packaging: 0 packaging collision/containment failures remain.
- **PASS** — lift_prop_clearance: Collision-free lift-row geometry screen.
- **PASS** — 15kg_stall_margin: Wing area 0.890 m² vs minimum 0.802 m² screen.
- **PASS** — cruise_power: 362.4 W vs configured gate.
- **PASS** — mission_ems: Dynamic mission profile through hybrid H2/battery EMS.
- **PASS** — transition: Forward and reverse point-mass transition completion on closed mass.
- **PASS** — current_longitudinal_stability: Closed-aircraft static margin 8.55% MAC.
- **PASS** — current_layout_6dof: Actual closed geometry/control 6-DOF gate set.
- **PASS** — sixdof_linear_aero_validity: 6-DOF coefficient model uses linear aerodynamic derivatives outside the CL clamp; high-alpha results require nonlinear AVL/CFD/wind-tunnel/flight correlation. Maximum |alpha| above 10 m/s = 8.0 deg.
- **PASS** — stability_screen_6dof: Explicit +8% SM reference screen.
- **PASS** — 15kg_zero_pitch_motor_thrust: Front/rear requirement 62.9/55.9 N vs 65.0 N available screen.
- **PASS** — recommended_structure_screen: Recommended structural member screens. The legacy partial-feedback mass gate is excluded because integrated-v2 already closes structural mass explicitly and checks the actual closed MTOW separately.
- **NOT_EVALUATED** — motor_mount: Controlled mount CAD, fasteners and material allowables are still required.
- **NOT_EVALUATED** — selected_battery_and_cylinder_mass: Battery and certified cylinder masses remain placeholders where supplier values are absent.

## Fidelity boundary

Integrated-v2 closure/optimization is screening fidelity. Generic prop maps, aerodynamic/control derivatives, CFRP equivalent properties and placeholder battery/cylinder hardware still require supplier/CFD/FEA/test evidence.
