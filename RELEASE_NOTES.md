# H2AIR Integrated Model v4.1 — Release Notes

This package is the user's latest H2AIR model with the IE-SOAR cooling integration added without replacing the existing layout, airfoil, CAD, hardware-selection, mission, 6-DOF or structural workflows.

## v4.1 additions

- Dedicated `h2air/fuel_cell_cooling.py` screening model.
- Assumed IE-SOAR fan direction: built-in fans suck air through the fuel-cell stack and discharge on the fan side.
- Cross-fuselage architecture: left NACA-style flush inlet, stack, right fan-side exhaust by default.
- Hover/ground cooling is sized with no ram-air credit.
- Waste-heat and required-airflow estimate from net FC power, efficiency screen, air properties and allowable air temperature rise.
- External duct/grille pressure-loss model and fan/system-curve operating-point estimate.
- Explicit fan-curve provenance: supplier fan data remain `NOT_EVALUATED`; current capacity is an engineering assumption.
- Inlet/exhaust slot geometry, NACA-style ramp geometry, plenum gaps and fan geometry are user-editable.
- Cooling-opening drag allowance feeds the cruise/mission model.
- Cooling airflow and cooling-geometry gates added to the integrated aircraft gate set.
- 42+ cooling drawing parameters exported to Inventor XML/XLSX/JSON.
- Viewer can display the cooling inlet and exhaust slot pattern.
- `details/FUEL_CELL_COOLING.json` written for every user-input run.

## Current default cooling screen

The configured 800 W / 50% efficiency / 1.10 heat margin / 15 C air-rise case requires about 101 CFM. The assumed module/external fan curve and current grille areas predict about 118 CFM, so the *assumption-based airflow* screen passes.

The current 135 mm fuselage width provides only about 3.3 mm inner side gap around the fuel-cell module; the configured 4 mm inlet-side and 10 mm fan-side gaps require about 142.4 mm fuselage width. Therefore the cooling geometry screen correctly fails until the installation width/shroud is revised.

See `docs/FUEL_CELL_COOLING.md`.
