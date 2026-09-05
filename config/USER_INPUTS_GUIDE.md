# H2AIR v3 user inputs

Normal users should edit **`config/user_inputs.json` only**.

## Modes

- `span_mode: "auto"` — the lifting-surface span is derived automatically for a normal run and is available to the design-study optimizer.
- `span_mode: "hardcoded"` — `hardcoded_span_m` is used exactly and the optimizer does not vary that span.
- `mass_mode: "linear"` — mass = `specific_mass_kg_per_m × geometric length × count`.
- `mass_mode: "structural_linear"` — same measured kg/m basis, but the primary structural member mass is scaled if the structure solver selects a different tube section.
- `mass_mode: "hardcoded"` — `hardcoded_mass_kg` is fixed and structural/geometry sizing does not change that member's mass.

## User-defined versus calculated

User-defined examples: payload, mission duration, fuselage section lengths/diameters, root chords, taper ratios, NACA codes, twist/sweep/dihedral, propeller sizes, hardware data, component positions, kg/m values and hardcode switches.

Auto-calculated examples: fuselage total length, lifting-surface spans in AUTO mode, wing/tail areas and MACs, detailed structural component masses from kg/m, CG/inertias, neutral point/static margin, drag/cruise power, VTOL requirements, transitions, mission energy, H2 use, battery SOC, primary structural sizing and converged MTOW.

The complete resolved/internal configuration is written after each run as `resolved_configuration.json`. Do not edit that file and feed it back as your normal working input unless you intentionally want an expert/internal run.

## Fuselage rule

- Section **lengths** are always user-fixed.
- `diameter_mode: "hardcoded"` — section `diameter_m` values are used as given; packaging fails if components do not fit.
- `diameter_mode: "auto"` — each section OD is sized from packed component radial extents + wall keepout (+ optional `diameter_margin_m`), not below `minimum_diameter_m`. Seed `diameter_m` values are only an initial guess.

## Packaging layout (when enabled)

- `energy_bay_order`: `fuel_cell_forward` (FC at nose break, H₂ aft) or `hydrogen_forward` (H₂ at nose break, FC aft).
- `energy_bay_order_mode`: `hardcoded` uses `energy_bay_order`; `auto` sweeps both in the design-study optimizer via `energy_bay_order_code` (0 / 1).
- Payload in the nose, aft face flush with the nose-section end, as low as bottom clearance allows.
- Cube Orange on top of the payload with the same aft flush; connector keepout forward of the Cube.
- Forward energy item flush with the nose-section end; aft energy item as aft as possible with pusher keepout.
- Control box and battery seats (`fc_top` / `fc_side` / `h2_top`) are searched to minimise fuselage diameter (split to H₂ preferred on OD ties).
- Two PDBs sit on the H₂ cylinder top shelf (not stacked on CB/battery).

## Airfoil note

The NACA code is now a controlled user input. For the main wing, `CLmax_override` takes precedence when supplied. If it is `null`, the model uses the screening value in `airfoil_database.json`. Those database values are conceptual assumptions and should ultimately be replaced by XFOIL/CFD/wind-tunnel/flight-test polars at the actual Reynolds number.
