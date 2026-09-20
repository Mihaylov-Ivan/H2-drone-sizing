# H2AIR Integrated Model — Run Summary

**Model status:** SCREENING_FAIL

## Aircraft at a glance

A **14.003 kg** hydrogen hybrid VTOL / fixed-wing UAV with a **3.17 m** main wing, **0.88 m x 228 mm** central pod, twin booms, **4x lift props** and a rear pusher. Design cruise **18.0 m/s** for **120 min** with **1.2 kg** payload.

## Configuration / look

### Fuselage / pod
- Length x outer size: **0.879 m x 0.228 m** (228 mm) (rectangular_filleted, fillet 20 mm) (auto side 228 mm, uniform sections); midbody auto lengths: center_a 236 mm, center_b 364 mm; component axial 10 mm
- Overall reference length (nose-to-tail layout): **1.25 m**

### Main wing
- AutoSize Main Wing: **ON** — coupled span/root-chord/taper optimization using current iteration M2 (final sizing mass **14.269 kg**)
- Span / area / AR: **3.171 m / 0.977 m² / 10.29**
- Root / tip / MAC: **0.361 / 0.255 / 0.311 m** (taper 0.71)
- Airfoil: **NACA6415**, CLmax screen **1.370**
- Aerodynamic centre: **x = 0.4557 m**
- Twist root to tip: **0.0 deg to -2.0 deg**; sweep **0.0 deg**; dihedral **0.0 deg**
- Vertical station (structure CG height): **z = 0.087 m**

### Empennage
- AutoSize Horizontal Tail: **ON** — performance-constrained span/root-chord/taper optimization (AR **4.54**, resulting Vh **0.264**; reference **0.440**)
- AutoSize Vertical Tail: **ON** — performance-constrained height/root-chord/taper optimization (per-fin AR **3.25**, resulting Vv **0.0236**; reference **0.0255**)
- Tail control sizing safety factor: **1.10x**; Vh/Vv are **outputs/calibration references, not hard minimums**
- Horizontal tail: span **0.782 m**, area **0.135 m²**, root/tip **0.234/0.111 m**, **NACA0012**, AC **x = 1.0502 m**, z = **0.087 m**
- Vertical tails: **twin fins**, height **0.426 m** each, total area **0.112 m²**, root/tip **0.159/0.103 m**, **NACA0012**, AC **x = 1.1103 m**
- Elevator: **90.5% tail span**, **29.9% local chord** (hinge x/c **0.701**) — **auto-sized with AVL**
- Rudders: **84.5% fin height**, **26.1% local chord** (hinge x/c **0.739**) — **auto-sized with AVL**

### Aerodynamic stability/control derivatives
- Derivative model: **AVL automated geometry derived** (mode **avl**)
- AVL: **PASS**, neutral point **x = 0.4922 m**, CLa **5.195 /rad**, Cmde **-0.681 /rad**, Cnb **0.078 /rad**, Cndr **-0.045 /rad**
- AVL scope: inviscid lifting-surface stability/control derivatives; stall, viscous drag, propeller interaction and separated flow remain separate model/screening calculations.

### Booms & lift layout
- Twin boom spacing (centre-to-centre): **0.748 m**
- Boom longitudinal extent: **x = 0.111 to 1.100 m**
- Lift-row midpoint: **x = 0.4007 m**; front/rear rotor rows at **0.111 / 0.690 m**
- Rotor lateral station: **|y| = 0.374 m** (boom-mounted through wing); motor z = **0.053 m**; prop diameter **22.0 in**; edge clearance **20 mm**
- Cruise lift-prop state: **stopped_aligned**
- Cruise pusher: fuselage-aft mount at **x = 0.916 m**
- Empennage waterline: HT z = **0.087 m** (aligned with main wing z = **0.087 m**)
- Wing/fuselage junction: fuse top z = **0.114 m**; wing drop from flush = **0.0 mm**; longerons **top/bottom** (wing uses top boss only)

### Energy / avionics packaging (stations)
- Energy bay order: **fuel_cell_forward** (mode hardcoded)
- Longeron boss cabin clear: **10 mm**; Z lifts (mm): fc 0.0, h2 0.0, nose 0.0, top 0.0
- Fuel cell: **x = 0.2553 m**, z = **-0.0331 m** (1.093 kg)
- FC control box (h2_top): **x,y,z = (0.4141, 0.0000, 0.0594) m** (0.357 kg)
- Buffer battery (fc_top): **x,y,z = (0.2215, 0.0000, 0.0623) m**
- H2 cylinder (+ stored H2): **x = 0.5930 m**, z = **-0.0355 m**
- Payload / Cube Orange: **x,z = (0.1035, -0.0330) / (0.0828, 0.0325) m** (nose aft flush x=0.1400 m)
- PDBs: **(0.5652, 0.0000, 0.0585)** and **(0.6552, 0.0000, 0.0585)** m
- Fuselage wall keepout: **10.0 mm** (rectangular_filleted cross-section; enforced by the packaging containment/collision screen)

## Mass & balance

- Closed MTOW: **14.003 kg** (pre-structure **13.573 kg**; margin to ceiling **0.997 kg**)
- CG: **(0.4435, 0.0000, 0.0308) m**
- Neutral point / static margin: **x_NP = 0.4922 m**, **15.66% MAC**

### Mass breakdown (takeoff)
- payload: **1.200 kg**
- wing structure: **2.616 kg**
- fuselage structure: **0.993 kg**
- tail structure: **0.812 kg**
- energy: **5.341 kg**
- propulsion: **1.340 kg**
- avionics: **1.210 kg**
- airframe: **0.490 kg**

## Propulsion & power

- Lift set: **Tmotor_U8Lite_150kV_P22x6p6_12S** (22.0 in, max thrust **71.0 N/motor**)
- Pusher set: **Tmotor_MN501S_240kV_P15x5_12S** (15.0 in, max thrust **45.7 N**)
- Battery: **SeriesPair_12S_Lipo_Battery_Pack_5500mAh/22.2V_30/60C_177.6Wh** (1.634 kg, **244.2 Wh**, 5.5 Ah)
- Fuel cell: **800 W** net, module **1.45 kg** (stack 1.093 kg + control 0.357 kg)
- Hydrogen store: **3.0 L** / **300 bar**, nominal H2 **62 g**, cylinder **1.62 kg**
- 15 kg zero-pitch motor need: front/rear **50.6 / 68.1 N**

### Fuel-cell cooling integration
- Architecture: **cross_fuselage**, fan direction **suction_through_stack_to_fan_side**
- Inlet / exhaust sides: **left / right**
- Thermal screen: **880 W** design waste heat, required airflow **101.0 CFM**
- Fan/duct screening flow: **118.2 CFM** (margin **1.17x**) — engineering assumption until fan data are confirmed
- Inlet/exhaust free area: **122.4 / 140.4 cm²**
- Inlet/exhaust inner side gap: **10.3 / 10.3 mm**; minimum fuselage width for configured gaps **142.4 mm**
- Cooling-opening cruise drag allowance: **0.265 N**

## Performance / mission

- Cruise drag / bus power: **13.42 N / 432.6 W** (lift props **stopped_aligned**)
- Inspection manoeuvre envelope: **+1.50 / -0.50 g**, maximum planned bank **45 deg** (1.41 g)
- Accelerated-stall margin at cruise: +g case uses **79.1% of CLmax**, -g case **49.2% of |CLmin|**
- Gust stall screen: sharp-edge vertical case uses **92.9% of CLmax** (no alleviation credit); 6-DOF gust cases peak at **59.0%**; configured lateral/vertical gusts **5.0 / ±2.0 m/s** for **2.0 s**
- Tail authority: **PASS** with **1.10x** safety factor; trim elevator **-10.6 deg**, worst vertical-gust elevator **18.2 deg**, directional Cnβ **0.078 /rad**, Cnr **-0.044 /rad**
- Endurance at cruise (usable H₂ / cruise bus): **2.02 h** (876 Wh usable)
- Forward / reverse transition: **15.08 s / 20.28 s**
- Mission energy (120 min): **887.1 Wh**
- H2 used / final battery SOC: **52.3 g / 92.8%**

## Structures (recommended screen)

- Wing spar: **28.00 / 26.00 mm OD/ID**
- Horizontal-tail spar: **10.00 / 8.00 mm OD/ID**
- Vertical-tail spars: **10.00 / 8.00 mm OD/ID**
- Boom tube: **26.00 / 24.00 mm OD/ID**
- Motor-mount limit thrust: **79.6 N/motor**

## Integrated gates

- **PASS** — mass_ceiling: Structurally closed mass 14.003 kg vs 15.0 kg ceiling.
- **PASS** — packaging: 0 packaging collision/containment failures remain.
- **PASS** — fuel_cell_cooling_airflow_screen: Estimated fan/duct operating flow 118.2 CFM vs 101.0 CFM required by the configured thermal screen.
- **PASS** — fuel_cell_cooling_geometry: Inlet/exhaust panel + plenum geometry screen; inlet/exhaust side gaps 10.3/10.3 mm vs 4.0/10.0 mm required.
- **NOT_EVALUATED** — fuel_cell_cooling_fan_curve: IE-SOAR fan/system airflow and allowable external pressure-drop data are not present in the supplied documentation; current airflow capacity is an engineering assumption to be replaced by supplier or bench data.
- **PASS** — lift_prop_clearance: Collision-free lift-row geometry screen.
- **PASS** — 15kg_stall_margin: Wing area 0.977 m² vs minimum 0.882 m² screen.
- **PASS** — inspection_bank_load: 45 deg bank requires 1.41 g vs +1.50 g operational limit.
- **PASS** — positive_manoeuvre_stall_margin: +1.50 g at 18.0 m/s uses 79.1% of CLmax; limit 90%. Minimum speed with margin 16.88 m/s.
- **PASS** — negative_manoeuvre_stall_margin: -0.50 g at 18.0 m/s uses 49.2% of |CLmin|; limit 90%.
- **PASS** — inspection_gust_stall_margin: Conservative sharp-edge +2.0 m/s vertical gust reaches 92.9% of CLmax (no alleviation credit); 6-DOF gust trajectories peak at 59.0%; hard limit 95%.
- **PASS** — cruise_power: 432.6 W vs configured gate.
- **PASS** — mission_ems: Dynamic mission profile through hybrid H2/battery EMS.
- **PASS** — transition: Forward and reverse point-mass transition completion on closed mass.
- **PASS** — AVL_derivatives: Automated AVL stability/control derivative evaluation: PASS; cache_hit=True.
- **PASS** — current_longitudinal_stability: Closed-aircraft static margin 15.66% MAC.
- **PASS** — tail_control_authority: Tail trim/elevator/rudder reserve with 1.10x safety factor; failed checks: none.
- **PASS** — payload_off_static_margin: Payload-off static margin 5.42% MAC (CG x=0.4753 m) vs minimum 5.0% MAC.
- **FAIL** — current_layout_6dof: Actual closed geometry/control 6-DOF gate set.
- **PASS** — sixdof_linear_aero_validity: 6-DOF coefficient model uses linear aerodynamic derivatives outside the CL clamp; high-alpha results require nonlinear AVL/CFD/wind-tunnel/flight correlation. Maximum |alpha| above 10 m/s = 9.3 deg.
- **FAIL** — stability_screen_6dof: Explicit +12% SM reference screen.
- **PASS** — 15kg_zero_pitch_motor_thrust: Front/rear requirement 50.6/68.1 N vs available 71.0/81.7 N (rear recovery x1.15).
- **PASS** — recommended_structure_screen: Recommended structural member screens. Audit reference/current/legacy sections and the legacy partial-feedback mass gate are excluded because integrated-v2 already closes structural mass explicitly and checks the actual closed MTOW separately.
- **PASS** — structural_mass_geometry_consistency: All verified primary sections match mass/CAD geometry.
- **NOT_EVALUATED** — motor_mount: Controlled mount CAD, fasteners and material allowables are still required.
- **PASS** — selected_battery_database_record: Selected battery mass/envelope are present in the controlled hardware database; final release still requires supplier/measured validation.
- **PASS** — certified_hydrogen_cylinder_mass: Certified cylinder mass is present.

## Fidelity boundary

Integrated-v4 closure/optimization is screening fidelity. Airfoil polars are used directly where supplied; missing airfoils retain explicit screening fallbacks. Selected motor/prop/battery data remain subject to supplier/bench validation and pusher forward-flight correlation.
