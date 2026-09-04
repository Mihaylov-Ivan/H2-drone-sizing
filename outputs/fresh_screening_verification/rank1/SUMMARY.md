# H2AIR Integrated Model — Run Summary

**Model status:** SCREENING_FAIL

## Aircraft state

- Pre-structure screening MTOW: **13.157 kg**
- Post-structure screening MTOW: **13.858 kg**
- Post-structure CG x: **0.3740 m**
- Remaining mass margin to 15 kg: **1.142 kg**
- Current-layout static margin: **7.40% MAC**

## Performance / mission

- Folded-prop cruise bus power: **375.4 W**
- Forward transition: **17.46 s**
- Reverse transition: **19.04 s**
- 120-min mission energy: **776.0 Wh**
- H₂ used: **51.0 g**
- Final battery SOC: **92.5%**

## Structural screen

- Recommended wing spar: **23.00/17.25 mm OD/ID**
- Recommended boom: **21.25/17.25 mm OD/ID**
- Motor-mount limit thrust: **79.6 N/motor**

## Integrated gates

- **PASS** — mass_ceiling: Structurally closed mass 13.858 kg vs 15.0 kg ceiling.
- **PASS** — packaging: 0 packaging collision/containment failures remain.
- **PASS** — lift_prop_clearance: Collision-free lift-row geometry screen.
- **PASS** — 15kg_stall_margin: Wing area 0.907 m² vs minimum 0.874 m² screen.
- **PASS** — cruise_power: 375.4 W vs configured gate.
- **PASS** — mission_ems: Dynamic mission profile through hybrid H2/battery EMS.
- **PASS** — transition: Forward and reverse point-mass transition completion on closed mass.
- **PASS** — current_longitudinal_stability: Closed-aircraft static margin 7.40% MAC.
- **FAIL** — current_layout_6dof: Actual closed geometry/control 6-DOF gate set.
- **PASS** — sixdof_linear_aero_validity: 6-DOF coefficient model uses linear aerodynamic derivatives outside the CL clamp; high-alpha results require nonlinear AVL/CFD/wind-tunnel/flight correlation. Maximum |alpha| above 10 m/s = 8.1 deg.
- **FAIL** — stability_screen_6dof: Explicit +8% SM reference screen.
- **PASS** — 15kg_zero_pitch_motor_thrust: Front/rear requirement 69.0/49.8 N vs 71.0 N available screen.
- **PASS** — recommended_structure_screen: Recommended structural member screens. The legacy partial-feedback mass gate is excluded because integrated-v2 already closes structural mass explicitly and checks the actual closed MTOW separately.
- **NOT_EVALUATED** — motor_mount: Controlled mount CAD, fasteners and material allowables are still required.
- **PASS** — selected_battery_database_record: Selected battery mass/envelope are present in the controlled hardware database; final release still requires supplier/measured validation.
- **NOT_EVALUATED** — certified_hydrogen_cylinder_mass: Selected certified 3.0 L / 350 bar cylinder mass remains open and is not released.

## Fidelity boundary

Integrated-v4 closure/optimization is screening fidelity. Airfoil polars are used directly where supplied; missing airfoils retain explicit screening fallbacks. Selected motor/prop/battery data remain subject to supplier/bench validation and pusher forward-flight correlation.
