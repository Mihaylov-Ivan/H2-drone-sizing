# H2AIR Integrated Model — Run Summary

**Model status:** SCREENING_FAIL

## Aircraft at a glance

A **14.145 kg** hydrogen hybrid VTOL / fixed-wing UAV with a **3.37 m** main wing, **0.78 m x 180 mm** central pod, twin booms, **4x lift props** and a rear pusher. Design cruise **20.0 m/s** for **120 min** with **1.2 kg** payload.

## Configuration / look

### Fuselage / pod
- Length x max diameter: **0.780 m x 0.180 m** (180 mm)
- Overall reference length (nose-to-tail layout): **1.15 m**

### Main wing
- Span / area / AR: **3.374 m / 0.879 m² / 12.95**
- Root / tip / MAC: **0.326 / 0.195 / 0.266 m** (taper 0.60)
- Airfoil: **NACA4412**, CLmax screen **1.185**
- Aerodynamic centre: **x = 0.3976 m**
- Twist root to tip: **0.0 deg to -2.0 deg**; sweep **0.0 deg**; dihedral **0.0 deg**
- Vertical station (structure CG height): **z = 0.100 m**

### Empennage
- Horizontal tail: span **0.869 m**, area **0.161 m²**, root/tip **0.218/0.153 m**, **NACA0012**, AC **x = 1.0051 m**, z = **0.030 m**
- Vertical tails: **twin fins**, height **0.300 m** each, total area **0.071 m²**, root/tip **0.148/0.089 m**, **NACA0012**, AC **x = 1.0923 m**

### Booms & lift layout
- Twin boom spacing (centre-to-centre): **0.700 m**
- Boom longitudinal extent: **x = 0.340 to 1.100 m**
- Lift-row midpoint: **x = 0.4574 m**; front/rear rotor rows at **0.168 / 0.747 m**
- Rotor lateral station: **|y| = 0.480 m**; prop diameter **22.0 in**; edge clearance **20 mm**
- Cruise lift-prop state: **folded**

### Energy / avionics packaging (stations)
- Fuel cell: **x = 0.1490 m**, z = **-0.0255 m**
- Buffer battery: **x = 0.3203 m**, z = **0.0631 m**
- H2 cylinder (+ stored H2): **x = 0.5186 m**, z = **-0.0283 m**

## Mass & balance

- Closed MTOW: **14.145 kg** (pre-structure **13.516 kg**; margin to ceiling **0.855 kg**)
- CG: **(0.4015, 0.0000, 0.0426) m**
- Neutral point / static margin: **x_NP = 0.4329 m**, **11.79% MAC**

### Mass breakdown (takeoff)
- payload: **1.200 kg**
- wing structure: **4.200 kg**
- fuselage structure: **0.996 kg**
- tail structure: **0.288 kg**
- energy: **5.081 kg**
- propulsion: **1.327 kg**
- avionics: **0.586 kg**
- airframe: **0.467 kg**

## Propulsion & power

- Lift set: **Tmotor_U8Lite_150kV_P22x6p6_12S** (22.0 in, max thrust **71.0 N/motor**)
- Pusher set: **Tmotor_MN501S_240kV_P15x5_12S** (15.0 in, max thrust **45.7 N**)
- Battery: **SeriesPair_12S_Lipo_Battery_Pack_5500mAh/22.2V_30/60C_177.6Wh** (1.634 kg, **244.2 Wh**, 5.5 Ah)
- Fuel cell: **800 W** net, module **1.45 kg**
- Hydrogen store: **3.0 L** / **300 bar**, nominal H2 **62 g**, cylinder **1.62 kg**
- 15 kg zero-pitch motor need: front/rear **70.9 / 47.9 N**

## Performance / mission

- Cruise drag / bus power: **12.45 N / 380.7 W** (lift props **folded**)
- Forward / reverse transition: **18.94 s / 18.58 s**
- Mission energy (120 min): **787.9 Wh**
- H2 used / final battery SOC: **46.2 g / 92.0%**

## Structures (recommended screen)

- Wing spar: **22.75 / 17.06 mm OD/ID**
- Boom tube: **21.00 / 17.00 mm OD/ID**
- Motor-mount limit thrust: **79.6 N/motor**

## Integrated gates

- **PASS** — mass_ceiling: Structurally closed mass 14.145 kg vs 15.0 kg ceiling.
- **PASS** — packaging: 0 packaging collision/containment failures remain.
- **PASS** — lift_prop_clearance: Collision-free lift-row geometry screen.
- **PASS** — 15kg_stall_margin: Wing area 0.879 m² vs minimum 0.874 m² screen.
- **PASS** — cruise_power: 380.7 W vs configured gate.
- **PASS** — mission_ems: Dynamic mission profile through hybrid H2/battery EMS.
- **PASS** — transition: Forward and reverse point-mass transition completion on closed mass.
- **PASS** — current_longitudinal_stability: Closed-aircraft static margin 11.79% MAC.
- **FAIL** — current_layout_6dof: Actual closed geometry/control 6-DOF gate set.
- **PASS** — sixdof_linear_aero_validity: 6-DOF coefficient model uses linear aerodynamic derivatives outside the CL clamp; high-alpha results require nonlinear AVL/CFD/wind-tunnel/flight correlation. Maximum |alpha| above 10 m/s = 6.4 deg.
- **FAIL** — stability_screen_6dof: Explicit +12% SM reference screen.
- **PASS** — 15kg_zero_pitch_motor_thrust: Front/rear requirement 70.9/47.9 N vs 71.0 N available screen.
- **PASS** — recommended_structure_screen: Recommended structural member screens. The legacy partial-feedback mass gate is excluded because integrated-v2 already closes structural mass explicitly and checks the actual closed MTOW separately.
- **NOT_EVALUATED** — motor_mount: Controlled mount CAD, fasteners and material allowables are still required.
- **PASS** — selected_battery_database_record: Selected battery mass/envelope are present in the controlled hardware database; final release still requires supplier/measured validation.
- **PASS** — certified_hydrogen_cylinder_mass: Certified cylinder mass is present.

## Fidelity boundary

Integrated-v4 closure/optimization is screening fidelity. Airfoil polars are used directly where supplied; missing airfoils retain explicit screening fallbacks. Selected motor/prop/battery data remain subject to supplier/bench validation and pusher forward-flight correlation.
