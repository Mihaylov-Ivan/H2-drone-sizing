# H2AIR Integrated Model — Run Summary

**Model status:** DESIGN_REQUIRES_GEOMETRY_UPDATE

## Aircraft state

- Pre-structure screening MTOW: **11.078 kg**
- Post-structure screening MTOW: **12.387 kg**
- Post-structure CG x: **0.4614 m**
- Remaining mass margin to 15 kg: **2.613 kg**
- Current-layout static margin: **-18.58% MAC**

## Performance / mission

- Folded-prop cruise bus power: **356.0 W**
- Forward transition: **30.00 s**
- Reverse transition: **16.68 s**
- 120-min mission energy: **739.9 Wh**
- H₂ used: **48.5 g**
- Final battery SOC: **91.4%**

## Structural screen

- Recommended wing spar: **22.50/16.88 mm OD/ID**
- Recommended boom: **20.00/16.00 mm OD/ID**
- Motor-mount limit thrust: **80.0 N/motor**

## Integrated gates

- **PASS** — mass_ceiling: Structurally closed mass 12.387 kg vs 15.0 kg ceiling.
- **FAIL** — packaging: 1 packaging collision/containment failures remain.
- **PASS** — lift_prop_clearance: Collision-free lift-row geometry screen.
- **FAIL** — 15kg_stall_margin: Wing area 0.634 m² vs minimum 0.802 m² screen.
- **PASS** — cruise_power: 356.0 W vs configured gate.
- **PASS** — mission_ems: Dynamic mission profile through hybrid H2/battery EMS.
- **FAIL** — transition: Forward and reverse point-mass transition completion on closed mass.
- **FAIL** — current_longitudinal_stability: Closed-aircraft static margin -18.58% MAC.
- **FAIL** — current_layout_6dof: Actual closed geometry/control 6-DOF gate set.
- **WARNING** — sixdof_linear_aero_validity: 6-DOF coefficient model uses linear aerodynamic derivatives outside the CL clamp; high-alpha results require nonlinear AVL/CFD/wind-tunnel/flight correlation. Maximum |alpha| above 10 m/s = 61.3 deg.
- **FAIL** — stability_screen_6dof: Explicit +8% SM reference screen.
- **FAIL** — 15kg_zero_pitch_motor_thrust: Front/rear requirement 69.0/49.8 N vs 65.0 N available screen.
- **PASS** — recommended_structure_screen: Recommended structural member screens. The legacy partial-feedback mass gate is excluded because integrated-v2 already closes structural mass explicitly and checks the actual closed MTOW separately.
- **NOT_EVALUATED** — motor_mount: Controlled mount CAD, fasteners and material allowables are still required.
- **NOT_EVALUATED** — selected_battery_and_cylinder_mass: Battery and certified cylinder masses remain placeholders where supplier values are absent.

## Fidelity boundary

Integrated-v2 closure/optimization is screening fidelity. Generic prop maps, aerodynamic/control derivatives, CFRP equivalent properties and placeholder battery/cylinder hardware still require supplier/CFD/FEA/test evidence.
