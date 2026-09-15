# H2AIR Integrated Model — Run Summary

**Model status:** SCREENING_FAIL

## Aircraft at a glance

A **14.308 kg** hydrogen hybrid VTOL / fixed-wing UAV with a **3.37 m** main wing, **0.86 m x 221 mm** central pod, twin booms, **4x lift props** and a rear pusher. Design cruise **20.0 m/s** for **120 min** with **1.2 kg** payload.

## Configuration / look

### Fuselage / pod
- Length x outer size: **0.859 m x 0.221 m** (221 mm) (rectangular_filleted, fillet 20 mm) (auto side 221 mm, uniform sections); midbody auto lengths: center_a 236 mm, center_b 344 mm; component axial 10 mm
- Overall reference length (nose-to-tail layout): **1.25 m**

### Main wing
- Span / area / AR: **3.374 m / 0.879 m² / 12.95**
- Root / tip / MAC: **0.326 / 0.195 / 0.266 m** (taper 0.60)
- Airfoil: **NACA4412**, CLmax screen **1.185**
- Aerodynamic centre: **x = 0.3868 m**
- Twist root to tip: **0.0 deg to -2.0 deg**; sweep **0.0 deg**; dihedral **0.0 deg**
- Vertical station (structure CG height): **z = 0.089 m**

### Empennage
- Horizontal tail: span **0.869 m**, area **0.161 m²**, root/tip **0.218/0.153 m**, **NACA0012**, AC **x = 1.0489 m**, z = **0.089 m**
- Vertical tails: **twin fins**, height **0.300 m** each, total area **0.071 m²**, root/tip **0.148/0.089 m**, **NACA0012**, AC **x = 1.0216 m**

### Booms & lift layout
- Twin boom spacing (centre-to-centre): **0.748 m**
- Boom longitudinal extent: **x = 0.096 to 1.100 m**
- Lift-row midpoint: **x = 0.3859 m**; front/rear rotor rows at **0.096 / 0.675 m**
- Rotor lateral station: **|y| = 0.374 m** (boom-mounted through wing); motor z = **0.120 m**; prop diameter **22.0 in**; edge clearance **20 mm**
- Cruise lift-prop state: **windmilling**
- Cruise pusher: fuselage-aft mount at **x = 0.896 m**
- Empennage waterline: HT z = **0.089 m** (aligned with main wing z = **0.089 m**)
- Wing/fuselage junction: fuse top z = **0.111 m**; wing drop from flush = **7.9 mm**; longerons **top/bottom** (wing uses top boss only)

### Energy / avionics packaging (stations)
- Energy bay order: **fuel_cell_forward** (mode auto)
- Longeron boss cabin clear: **10 mm**; Z lifts (mm): fc 0.0, h2 0.0, nose 0.0, top 0.0
- Fuel cell: **x = 0.2553 m**, z = **-0.0313 m** (1.093 kg)
- FC control box (h2_top): **x,y,z = (0.4141, 0.0000, 0.0612) m** (0.357 kg)
- Buffer battery (fc_top): **x,y,z = (0.2215, 0.0000, 0.0641) m**
- H2 cylinder (+ stored H2): **x = 0.5930 m**, z = **-0.0337 m**
- Payload / Cube Orange: **x,z = (0.1035, -0.0312) / (0.0828, 0.0343) m** (nose aft flush x=0.1400 m)
- PDBs: **(0.5652, 0.0000, 0.0604)** and **(0.6552, 0.0000, 0.0604)** m
- Fuselage wall keepout: **10.0 mm** (circular cross-section; packaging gate fails on radial protrusion)

## Mass & balance

- Closed MTOW: **14.308 kg** (pre-structure **13.683 kg**; margin to ceiling **0.692 kg**)
- CG: **(0.4025, 0.0000, 0.0427) m**
- Neutral point / static margin: **x_NP = 0.4253 m**, **8.56% MAC**

### Mass breakdown (takeoff)
- payload: **1.200 kg**
- wing structure: **4.200 kg**
- fuselage structure: **1.097 kg**
- tail structure: **0.293 kg**
- energy: **5.081 kg**
- propulsion: **1.327 kg**
- avionics: **0.643 kg**
- airframe: **0.467 kg**

## Propulsion & power

- Lift set: **Tmotor_U8Lite_150kV_P22x6p6_12S** (22.0 in, max thrust **71.0 N/motor**)
- Pusher set: **Tmotor_MN501S_240kV_P15x5_12S** (15.0 in, max thrust **45.7 N**)
- Battery: **SeriesPair_12S_Lipo_Battery_Pack_5500mAh/22.2V_30/60C_177.6Wh** (1.634 kg, **244.2 Wh**, 5.5 Ah)
- Fuel cell: **800 W** net, module **1.45 kg** (stack 1.093 kg + control 0.357 kg)
- Hydrogen store: **3.0 L** / **300 bar**, nominal H2 **62 g**, cylinder **1.62 kg**
- 15 kg zero-pitch motor need: front/rear **56.0 / 62.8 N**

### Fuel-cell cooling integration
- Architecture: **cross_fuselage**, fan direction **suction_through_stack_to_fan_side**
- Inlet / exhaust sides: **left / right**
- Thermal screen: **880 W** design waste heat, required airflow **101.0 CFM**
- Fan/duct screening flow: **118.2 CFM** (margin **1.17x**) — engineering assumption until fan data are confirmed
- Inlet/exhaust free area: **122.4 / 140.4 cm²**
- Inlet/exhaust inner side gap: **10.3 / 10.3 mm**; minimum fuselage width for configured gaps **142.4 mm**
- Cooling-opening cruise drag allowance: **0.327 N**

## Performance / mission

- Cruise drag / bus power: **37.89 N / 1087.4 W** (lift props **windmilling**)
- Forward / reverse transition: **19.44 s / 18.48 s**
- Mission energy (120 min): **2175.8 Wh**
- H2 used / final battery SOC: **52.6 g / 20.0%**

## Structures (recommended screen)

- Wing spar: **22.75 / 17.06 mm OD/ID**
- Boom tube: **21.25 / 17.25 mm OD/ID**
- Motor-mount limit thrust: **79.6 N/motor**

## Integrated gates

- **PASS** — mass_ceiling: Structurally closed mass 14.308 kg vs 15.0 kg ceiling.
- **PASS** — packaging: 0 packaging collision/containment failures remain.
- **PASS** — fuel_cell_cooling_airflow_screen: Estimated fan/duct operating flow 118.2 CFM vs 101.0 CFM required by the configured thermal screen.
- **PASS** — fuel_cell_cooling_geometry: Inlet/exhaust panel + plenum geometry screen; inlet/exhaust side gaps 10.3/10.3 mm vs 4.0/10.0 mm required.
- **NOT_EVALUATED** — fuel_cell_cooling_fan_curve: IE-SOAR fan/system airflow and allowable external pressure-drop data are not present in the supplied documentation; current airflow capacity is an engineering assumption to be replaced by supplier or bench data.
- **PASS** — lift_prop_clearance: Collision-free lift-row geometry screen.
- **PASS** — 15kg_stall_margin: Wing area 0.879 m² vs minimum 0.874 m² screen.
- **FAIL** — cruise_power: 1087.4 W vs configured gate.
- **FAIL** — mission_ems: Dynamic mission profile through hybrid H2/battery EMS.
- **PASS** — transition: Forward and reverse point-mass transition completion on closed mass.
- **PASS** — current_longitudinal_stability: Closed-aircraft static margin 8.56% MAC.
- **PASS** — current_layout_6dof: Actual closed geometry/control 6-DOF gate set.
- **PASS** — sixdof_linear_aero_validity: 6-DOF coefficient model uses linear aerodynamic derivatives outside the CL clamp; high-alpha results require nonlinear AVL/CFD/wind-tunnel/flight correlation. Maximum |alpha| above 10 m/s = 8.3 deg.
- **PASS** — stability_screen_6dof: Explicit +12% SM reference screen.
- **PASS** — 15kg_zero_pitch_motor_thrust: Front/rear requirement 56.0/62.8 N vs 71.0 N available screen.
- **PASS** — recommended_structure_screen: Recommended structural member screens. Audit reference/current/legacy sections and the legacy partial-feedback mass gate are excluded because integrated-v2 already closes structural mass explicitly and checks the actual closed MTOW separately.
- **NOT_EVALUATED** — motor_mount: Controlled mount CAD, fasteners and material allowables are still required.
- **PASS** — selected_battery_database_record: Selected battery mass/envelope are present in the controlled hardware database; final release still requires supplier/measured validation.
- **PASS** — certified_hydrogen_cylinder_mass: Certified cylinder mass is present.

## Fidelity boundary

Integrated-v4 closure/optimization is screening fidelity. Airfoil polars are used directly where supplied; missing airfoils retain explicit screening fallbacks. Selected motor/prop/battery data remain subject to supplier/bench validation and pusher forward-flight correlation.
