# H2AIR v3 user inputs

Normal users should edit **`config/user_inputs.json` only**.

## Inspection flight envelope

The optional top-level `flight_envelope` block controls the non-aerobatic route-following manoeuvre/gust screen.

- `maximum_bank_deg`: planned mission bank limit. The default inspection profile uses **45 deg** (about **1.41 g** in a level coordinated turn).
- `positive_manoeuvre_load_factor_g`: operational accelerated-stall screen; default **+1.5 g**.
- `negative_manoeuvre_load_factor_g`: unloading/gust screen; default **-0.5 g**. This is not an aerobatic capability target.
- `maximum_manoeuvre_clmax_utilization`: maximum allowed fraction of the conservative wing CLmax during the manoeuvre screen; default **0.90**.
- `lateral_gust_mps` / `vertical_gust_mps`: disturbance cases. Defaults are **5 m/s lateral** and **±2 m/s vertical**, including a combined lateral+upward 6-DOF case.
- The upward vertical gust also drives a conservative **sharp-edge, no-alleviation** stall screen. AutoSize Main Wing must satisfy this instantaneous aerodynamic screen as well as the slower closed-loop 6-DOF disturbance cases.
- `maximum_gust_clmax_utilization`: hard CLmax-utilization limit for the gust stall screen; default **0.95**.
- `structural_design_positive_load_factor_g`: structural wing-bending screen, kept at **+3.0 g** to provide structural margin above the operational +1.5 g envelope.

AutoSize Main Wing includes the +1.5 g accelerated-stall requirement using the current M2 iteration mass. The 15 kg ceiling remains a separate mass/stall robustness check and the +3 g structural case remains based on the 15 kg ceiling.

## Modes

- Main wing: `"AutoSize Main Wing": true` jointly optimizes **span + root chord + taper ratio** inside the normal integrated run. The stall/transition area calculation uses the **current structurally closed iteration mass M2**, not the fixed 15 kg ceiling. The 15 kg value remains the hard final mass constraint and the structural design/load-case mass.
- Main wing: `"AutoSize Main Wing": false` uses `hardcoded_span_m`, `root_chord_m` and `taper_ratio` exactly.
- Main-wing search bounds are `span_bounds_m`, `root_chord_bounds_m` and `taper_ratio_bounds`; `minimum_tip_chord_m` prevents an impractically narrow tip. `auto_area_margin_factor` is applied to the live-M2 stall/transition area requirement.
- Horizontal tail: `"AutoSize Horizontal Tail": true` jointly optimizes **span + root chord + taper ratio** after the main-wing pass from actual aircraft requirements: loaded static margin, cruise trim, fixed-wing gust response, structural closure, mass, packaging, aspect ratio and minimum tip chord.
- Vertical tail: `"AutoSize Vertical Tail": true` jointly optimizes **fin height + root chord + taper ratio** from directional stability/control response, fixed-wing gust response, structural closure, mass and geometric bounds.
- `target_tail_volume_coefficient` (HT) and `target_vertical_tail_volume_coefficient` (VT) are **calibration references** for the screening aerodynamic derivatives. They are not hard minimum tail sizes; the resulting Vh and Vv are model outputs.
- Tail geometry scales screening derivatives: elevator authority and pitch damping with HT effectiveness; directional stability, yaw damping and rudder authority with VT effectiveness. These remain calibrated engineering assumptions until AVL/CFD/wind-tunnel/flight-test derivatives are available.
- `tail_autosizing.control_safety_factor` defaults to **1.10**. During optimization the normal trim and gust/control limits are divided by this factor, leaving about 10% control headroom.
- Tail gust validation is run in fixed-wing mode with the **lift rotors disabled**, so stopped/aligned VTOL propellers cannot hide insufficient aerodynamic tail authority.
- Setting either tail autosize trigger to `false` uses that surface's `hardcoded_span_m`, `root_chord_m` and `taper_ratio` exactly.
- Tail search bounds are controlled by `span_bounds_m`, `root_chord_bounds_m`, `taper_ratio_bounds`, `aspect_ratio_bounds` and `minimum_tip_chord_m`. `preferred_aspect_ratio` and `preferred_taper_ratio` are weak secondary regularizers only; structurally closed MTOW is the primary objective.
- The tail optimizer uses a transparent low-lift profile-drag proxy only as a secondary tie-breaker because the current cruise-drag model does not yet resolve HT/VT profile drag independently. Final 6-DOF, tail-control-authority and structural gates run on the selected geometry.
- `mass_mode: "linear"` — mass = `specific_mass_kg_per_m × geometric length × count`.
- `mass_mode: "structural_linear"` — same measured kg/m basis, but the primary structural member mass is scaled if the structure solver selects a different tube section. Enter the calibration OD as `reference_tube_od_m` (not the recommended design OD). Legacy `tube_od_m` on these members is still accepted and renamed on load.
- `mass_mode: "hardcoded"` — `hardcoded_mass_kg` is fixed and structural/geometry sizing does not change that member's mass.
- Primary wing/HT/VT spars and boom tubes: `reference_tube_od_m` is the mass-scaling / audit reference section only. CAD and layout use the structures **recommended** OD. Secondary spars and fuselage longerons keep `tube_od_m` as real geometry.

## User-defined versus calculated

User-defined examples: payload, mission duration, fuselage section lengths/diameters, main-wing/HT/VT autosize triggers and bounds, hardcoded/seed root chords and taper ratios, NACA codes, twist/sweep/dihedral, propeller sizes, hardware data (including H₂ cylinder length/diameter/mass from `hardware.hydrogen`), component positions, kg/m values and hardcode switches.

## Hydrogen cylinder

`hardware.hydrogen` is the source of truth for the selected tank (AMS 3 CF by default): `cylinder_length_m`, `cylinder_diameter_m`, `cylinder_mass_kg`, `water_volume_L`, `pressure_bar`, `nominal_h2_mass_kg`, `reference_net_energy_Wh`, etc. Those length/diameter values are written into the `hydrogen_cylinder` packaging envelope before layout/sizing runs.

Auto-calculated examples: fuselage total length, main-wing/HT/VT optimized span/chord/taper in AUTO mode, wing/tail areas and MACs, detailed structural component masses from kg/m, CG/inertias, neutral point/static margin, drag/cruise power, VTOL requirements, transitions, mission energy, H2 use, battery SOC, primary structural sizing and converged MTOW.

The complete resolved/internal configuration is written after each run as `resolved_configuration.json`. Do not edit that file and feed it back as your normal working input unless you intentionally want an expert/internal run.

## Fuselage rule

- Nose and tail **lengths** are always user-fixed.
- `midbody_length_mode: "hardcoded"` — `center_a` / `center_b` `length_m` values are used as given.
- `midbody_length_mode: "auto"` — `center_a` / `center_b` are sized so the FC–H₂ facing gap equals `packaging_layout.clearances_m.component_axial_m`. Section break is at mid-gap. Seed midbody lengths are only an initial guess.
- `diameter_mode: "hardcoded"` — section `diameter_m` values are used as given; packaging fails if components do not fit.
- `cross_section: "rectangular_filleted"` (default) — rounded-rectangle midbody; **width (Y)** and **height (Z)** sized independently from packed AABB + wall keepout. `square_filleted` forces W=H. `"circular"` keeps the legacy tube. Nose/tail fair from that base to circular tips.
- `fillet_radius_m` — corner fillet on the rectangular/square outer profile.

## Packaging layout (when enabled)

- `energy_bay_order`: `fuel_cell_forward` (FC at nose break, H₂ aft) or `hydrogen_forward` (H₂ at nose break, FC aft).
- `energy_bay_order_mode`: `hardcoded` uses `energy_bay_order`; `auto` sweeps both in the design-study optimizer via `energy_bay_order_code` (0 / 1).
- `cb_battery_order`: `battery_forward` or `control_box_forward` — which of the buffer battery / FCPM control box sits more forward in X. They are always packed **tandem in X** (never side-by-side); the preferred item is on/near the forward energy host, the other behind it with at least `component_axial_m` clearance.
- Payload in the nose, aft face 10 mm forward of the nose-section end, as low as bottom clearance allows, plus optional `nose_bay_raise_m` (raises payload and Cube together).
- Cube Orange on top of the payload with the same aft station; connector keepout forward of the Cube.
- Forward energy item flush with the nose-section end; aft energy item as aft as possible with pusher keepout.
- `fuselage_wall_m` — keepout between component outer faces and the fuselage inner wall in Y (and for circular, radial); default 10 mm. Sets `wall_clearance_m` used for auto width/OD sizing.
- Component Z is auto-placed above the bottom longeron boss with `packaging_layout.clearances_m.longeron_boss_clearance_m` (default 10 mm) to both top and bottom bosses; independent bay groups are lifted to minimise fuselage diameter.
- Control box and battery sit on a shared energy-bay top shelf in X tandem (`cb_battery_order`). The forward top item is flush with the forward energy host front face; the second is placed `component_axial_m` behind it. Site labels (`fc_top` / `h2_top`) are overlap tags only. PDBs pack on the remaining top-shelf length.
- `component_axial_m` is the required X clearance between non-nose components that overlap in YZ (also sets the FC–H₂ gap in midbody auto).
- Two PDBs sit on the H₂ cylinder top shelf (not stacked on CB/battery); tandem in X is preferred when that shrinks the required fuselage section.

## Airfoil note

The NACA code is now a controlled user input. For the main wing, `CLmax_override` takes precedence when supplied. If it is `null`, the model uses the screening value in `airfoil_database.json`. Those database values are conceptual assumptions and should ultimately be replaced by XFOIL/CFD/wind-tunnel/flight-test polars at the actual Reynolds number.

## Fuel-cell cooling inputs (v4.1)

Edit `fuel_cell_cooling` to control the IE-SOAR cooling installation.

The default architecture assumes the fuel-cell fans suck air through the stack, with a left-side NACA-style inlet and right-side fan exhaust. The most useful user-editable drawing parameters are:

- `inlet.slot_count`, `slot_length_x_m`, `slot_width_z_m`, `slot_gap_z_m`;
- `inlet.recess_depth_y_m`, `ramp_length_x_m`, `sidewall_angle_deg`, `lip_radius_m`;
- corresponding exhaust slot geometry;
- `exhaust.fan_shroud_depth_y_m` and `louvre_angle_deg`;
- fan diameter / spacing;
- minimum inlet-side and exhaust-side plenum gaps;
- CAD edge distance, grille corner radius and minimum web between slots.

The fan-flow and external pressure values are **screening assumptions** until supplier or bench data are available. Do not treat the airflow PASS as supplier validation.
