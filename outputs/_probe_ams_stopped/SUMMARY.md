# H2AIR Integrated Model — Run Summary

**Model status:** DESIGN_REQUIRES_GEOMETRY_UPDATE

## Aircraft at a glance

A **18.816 kg** hydrogen hybrid VTOL / fixed-wing UAV with a **4.58 m** main wing, **0.96 m x 269 mm** central pod, twin booms, **4x lift props** and a rear pusher. Design cruise **20.0 m/s** for **240 min** with **1.2 kg** payload.

## Configuration / look

### Fuselage / pod
- Length x outer size: **0.958 m x 0.269 m** (269 mm) (rectangular_filleted, fillet 20 mm) (auto side 269 mm, uniform sections); midbody auto lengths: center_a 236 mm, center_b 443 mm; component axial 10 mm
- Overall reference length (nose-to-tail layout): **1.45 m**

### Main wing
- Span / area / AR: **4.577 m / 1.193 m² / 17.56**
- Root / tip / MAC: **0.326 / 0.195 / 0.266 m** (taper 0.60)
- Airfoil: **NACA4412**, CLmax screen **1.185**
- Aerodynamic centre: **x = 0.5200 m**
- Twist root to tip: **0.0 deg to -2.0 deg**; sweep **0.0 deg**; dihedral **0.0 deg**
- Vertical station (structure CG height): **z = 0.108 m**

### Empennage
- Horizontal tail: span **0.966 m**, area **0.179 m²**, root/tip **0.218/0.153 m**, **NACA0012**, AC **x = 1.3000 m**, z = **0.108 m**
- Vertical tails: **twin fins**, height **0.650 m** each, total area **0.154 m²**, root/tip **0.148/0.089 m**, **NACA0012**, AC **x = 1.3500 m**

### Booms & lift layout
- Twin boom spacing (centre-to-centre): **0.793 m**
- Boom longitudinal extent: **x = 0.186 to 1.300 m**
- Lift-row midpoint: **x = 0.4757 m**; front/rear rotor rows at **0.186 / 0.765 m**
- Rotor lateral station: **|y| = 0.396 m** (boom-mounted through wing); motor z = **0.139 m**; prop diameter **22.0 in**; edge clearance **20 mm**
- Cruise lift-prop state: **stopped_aligned**
- Cruise pusher: fuselage-aft mount at **x = 0.995 m**
- Empennage waterline: HT z = **0.108 m** (aligned with main wing z = **0.108 m**)
- Wing/fuselage junction: fuse top z = **0.134 m**; wing drop from flush = **12.0 mm**; longerons **top/bottom** (wing uses top boss only)

### Energy / avionics packaging (stations)
- Energy bay order: **fuel_cell_forward** (mode auto)
- Longeron boss cabin clear: **10 mm**; Z lifts (mm): fc 0.0, h2 0.0, nose 0.0, top 0.0
- Fuel cell: **x = 0.2553 m**, z = **-0.0535 m** (1.093 kg)
- FC control box (h2_top): **x,y,z = (0.4141, 0.0000, 0.0792) m** (0.357 kg)
- Buffer battery (fc_top): **x,y,z = (0.2215, 0.0000, 0.0821) m**
- H2 cylinder (+ stored H2): **x = 0.6425 m**, z = **-0.0334 m**
- Payload / Cube Orange: **x,z = (0.1035, -0.0534) / (0.0828, 0.0121) m** (nose aft flush x=0.1400 m)
- PDBs: **(0.5652, 0.0000, 0.0784)** and **(0.6552, 0.0000, 0.0784)** m
- Fuselage wall keepout: **10.0 mm** (circular cross-section; packaging gate fails on radial protrusion)

## Mass & balance

- Closed MTOW: **18.816 kg** (pre-structure **17.343 kg**; margin to ceiling **0.184 kg**)
- CG: **(0.5370, 0.0000, 0.0539) m**
- Neutral point / static margin: **x_NP = 0.5572 m**, **7.59% MAC**

### Mass breakdown (takeoff)
- payload: **1.200 kg**
- wing structure: **4.879 kg**
- fuselage structure: **1.511 kg**
- tail structure: **1.286 kg**
- landing gear: **0.064 kg**
- energy: **7.439 kg**
- propulsion: **1.327 kg**
- avionics: **0.643 kg**
- airframe: **0.467 kg**

## Propulsion & power

- Lift set: **Tmotor_U8Lite_150kV_P22x6p6_12S** (22.0 in, max thrust **71.0 N/motor**)
- Pusher set: **Tmotor_MN501S_240kV_P15x5_12S** (15.0 in, max thrust **45.7 N**)
- Battery: **SeriesPair_12S_Lipo_Battery_Pack_5500mAh/22.2V_30/60C_177.6Wh** (1.634 kg, **244.2 Wh**, 5.5 Ah)
- Fuel cell: **800 W** net, module **1.45 kg** (stack 1.093 kg + control 0.357 kg)
- Hydrogen store: **6.8 L** / **300 bar**, nominal H2 **140 g**, cylinder **3.90 kg**
- 15 kg zero-pitch motor need: front/rear **59.3 / 91.1 N**

### Fuel-cell cooling integration
- Architecture: **cross_fuselage**, fan direction **suction_through_stack_to_fan_side**
- Inlet / exhaust sides: **left / right**
- Thermal screen: **880 W** design waste heat, required airflow **101.0 CFM**
- Fan/duct screening flow: **118.2 CFM** (margin **1.17x**) — engineering assumption until fan data are confirmed
- Inlet/exhaust free area: **122.4 / 140.4 cm²**
- Inlet/exhaust inner side gap: **32.8 / 32.8 mm**; minimum fuselage width for configured gaps **142.4 mm**
- Cooling-opening cruise drag allowance: **0.327 N**

## Performance / mission

- Cruise drag / bus power: **16.91 N / 504.6 W** (lift props **stopped_aligned**)
- Landing-leg parasite: **0.939 N** (CdA **39.14 cm²**, sleeve cylinder screen)
- Forward / reverse transition: **24.72 s / 20.94 s**
- Mission energy (240 min): **2060.5 Wh**
- H2 used / final battery SOC: **119.3 g / 69.2%**

## Structures (recommended screen)

- Wing spar: **28.00 / 21.00 mm OD/ID**
- Boom tube: **22.75 / 18.75 mm OD/ID**
- Landing legs (3g): **8.00 / 6.00 mm OD/ID** (CFRP tube + 2 mm printed sleeve; nose + 2 wing)
- Motor-mount limit thrust: **91.1 N/motor**

## Integrated gates

- **PASS** — mass_ceiling: Structurally closed mass 18.816 kg vs 19.0 kg ceiling.
- **PASS** — packaging: 0 packaging collision/containment failures remain.
- **PASS** — fuel_cell_cooling_airflow_screen: Estimated fan/duct operating flow 118.2 CFM vs 101.0 CFM required by the configured thermal screen.
- **PASS** — fuel_cell_cooling_geometry: Inlet/exhaust panel + plenum geometry screen; inlet/exhaust side gaps 32.8/32.8 mm vs 4.0/10.0 mm required.
- **NOT_EVALUATED** — fuel_cell_cooling_fan_curve: IE-SOAR fan/system airflow and allowable external pressure-drop data are not present in the supplied documentation; current airflow capacity is an engineering assumption to be replaced by supplier or bench data.
- **PASS** — lift_prop_clearance: Collision-free lift-row geometry screen.
- **PASS** — 15kg_stall_margin: Wing area 1.193 m² vs minimum 1.107 m² screen.
- **PASS** — cruise_power: 504.6 W vs configured gate.
- **PASS** — mission_ems: Dynamic mission profile through hybrid H2/battery EMS.
- **PASS** — transition: Forward and reverse point-mass transition completion on closed mass.
- **PASS** — current_longitudinal_stability: Closed-aircraft static margin 7.59% MAC.
- **PASS** — current_layout_6dof: Actual closed geometry/control 6-DOF gate set.
- **PASS** — sixdof_linear_aero_validity: 6-DOF coefficient model uses linear aerodynamic derivatives outside the CL clamp; high-alpha results require nonlinear AVL/CFD/wind-tunnel/flight correlation. Maximum |alpha| above 10 m/s = 8.9 deg.
- **PASS** — stability_screen_6dof: Explicit +12% SM reference screen.
- **FAIL** — 15kg_zero_pitch_motor_thrust: Front/rear requirement 59.3/91.1 N vs 71.0 N available screen.
- **PASS** — recommended_structure_screen: Recommended structural member screens. Audit reference/current/legacy sections and the legacy partial-feedback mass gate are excluded because integrated-v2 already closes structural mass explicitly and checks the actual closed MTOW separately.
- **NOT_EVALUATED** — motor_mount: Controlled mount CAD, fasteners and material allowables are still required.
- **PASS** — selected_battery_database_record: Selected battery mass/envelope are present in the controlled hardware database; final release still requires supplier/measured validation.
- **PASS** — certified_hydrogen_cylinder_mass: Certified cylinder mass is present.

## Fidelity boundary

Integrated-v4 closure/optimization is screening fidelity. Airfoil polars are used directly where supplied; missing airfoils retain explicit screening fallbacks. Selected motor/prop/battery data remain subject to supplier/bench validation and pusher forward-flight correlation.
