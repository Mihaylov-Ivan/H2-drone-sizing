# H2AIR v3 user inputs

Normal users should edit **`config/user_inputs.json` only**.

## Modes

- `span_mode: "auto"` — the lifting-surface span is derived automatically for a normal run and is available to the design-study optimizer.
- `span_mode: "hardcoded"` — `hardcoded_span_m` is used exactly and the optimizer does not vary that span.
- `mass_mode: "linear"` — mass = `specific_mass_kg_per_m × geometric length × count`.
- `mass_mode: "structural_linear"` — same measured kg/m basis, but the primary structural member mass is scaled if the structure solver selects a different tube section. Enter the calibration OD as `reference_tube_od_m` (not the recommended design OD). Legacy `tube_od_m` on these members is still accepted and renamed on load.
- `mass_mode: "hardcoded"` — `hardcoded_mass_kg` is fixed and structural/geometry sizing does not change that member's mass.
- Primary wing/HT/VT spars and boom tubes: `reference_tube_od_m` is the mass-scaling / audit reference section only. CAD and layout use the structures **recommended** OD. Secondary spars and fuselage longerons keep `tube_od_m` as real geometry.

## User-defined versus calculated

User-defined examples: payload, mission duration, fuselage section lengths/diameters, root chords, taper ratios, NACA codes, twist/sweep/dihedral, propeller sizes, hardware data (including H₂ cylinder length/diameter/mass from `hardware.hydrogen`), component positions, kg/m values and hardcode switches.

## Hydrogen cylinder

`hardware.hydrogen` is the source of truth for the selected tank (AMS 3 CF by default): `cylinder_length_m`, `cylinder_diameter_m`, `cylinder_mass_kg`, `water_volume_L`, `pressure_bar`, `nominal_h2_mass_kg`, `reference_net_energy_Wh`, etc. Those length/diameter values are written into the `hydrogen_cylinder` packaging envelope before layout/sizing runs.

Auto-calculated examples: fuselage total length, lifting-surface spans in AUTO mode, wing/tail areas and MACs, detailed structural component masses from kg/m, CG/inertias, neutral point/static margin, drag/cruise power, VTOL requirements, transitions, mission energy, H2 use, battery SOC, primary structural sizing and converged MTOW.

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
