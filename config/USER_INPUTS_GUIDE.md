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

The fuselage is **never enlarged to fit components**. Its section lengths and diameters come from the user input. If a known component envelope does not fit inside the resulting pod envelope, the packaging gate fails.

## Airfoil note

The NACA code is now a controlled user input. For the main wing, `CLmax_override` takes precedence when supplied. If it is `null`, the model uses the screening value in `airfoil_database.json`. Those database values are conceptual assumptions and should ultimately be replaced by XFOIL/CFD/wind-tunnel/flight-test polars at the actual Reynolds number.
