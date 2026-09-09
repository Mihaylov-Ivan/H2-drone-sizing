# IE-SOAR 800 W fuel-cell cooling integration

## Architecture implemented

H2AIR assumes the two built-in IE-SOAR fuel-cell fans **suck ambient air through the fuel-cell stack** and discharge through the fan face. The airframe integration therefore uses a short cross-fuselage path:

`left-side NACA-style flush inlet -> fuel-cell stack -> built-in fans -> right-side direct exhaust grille`

The inlet and exhaust sides are user-controlled, but the default is left inlet / right fan-side exhaust. The hover/ground cooling gate deliberately gives **zero credit to ram air**. Cruise pressure recovery is reported only as a possible bonus.

## User inputs

All cooling inputs are under `fuel_cell_cooling` in `config/user_inputs.json`.

The initial drawing values are deliberately explicit and easy to change:

- fan count, diameter and centre spacing;
- assumed no-external-duct module airflow and external pressure capability;
- fuel-cell efficiency screening value and allowable air temperature rise;
- inlet slot count, slot length, slot width and slot gap;
- NACA-style ramp/recess depth, ramp length, sidewall angle and lip radius;
- exhaust slot geometry, shroud depth and louvre angle;
- minimum inlet-side and fan-side plenum gaps;
- cooling grille CAD tolerances / edge distances.

## Thermal / airflow screen

The model estimates fuel-cell waste heat from:

`Q_heat = P_net * (1/eta - 1) * heat_margin`

and required cooling airflow from:

`Qdot_air = Q_heat / (rho * cp * deltaT_air)`

The current default 800 W / 50% / 1.10 heat-margin / 15 C air-rise screen requires about 101 CFM.

The external inlet and exhaust pressure loss is represented by a lumped K coefficient. The operating point is the intersection of:

- an assumed linear external fan-pressure curve; and
- the quadratic external duct/grille pressure-loss curve.

**Important:** Intelligent Energy fan airflow/static-pressure data were not present in the supplied brochures. The default fan curve is therefore an engineering assumption. The model explicitly exports the cooling fan-curve gate as `NOT_EVALUATED` until supplier or bench data replace the assumption.

## CAD output

The CAD exporter now writes drawing-driving cooling parameters, including:

- inlet/exhaust sides and centres;
- NACA-style inlet panel dimensions;
- inlet slot count/length/width/gap;
- recess depth, ramp length/angle, sidewall angle and lip radius;
- exhaust panel and slot dimensions;
- fan shroud depth and louvre angle;
- fan diameter, spacing and centres;
- actual and minimum side plenum gaps;
- minimum fuselage width required for the configured cooling gaps;
- grille corner radius, slot web and section-joint edge distance.

No thermal power/stress quantities are included in the CAD parameter bundle.

## Current design implication

The present 135 mm fuselage width gives only about 3.3 mm inner air gap on each side of the 108.4 mm-wide fuel-cell envelope after the configured 10 mm wall keepout. The initial cooling architecture asks for 4 mm on the inlet side and 10 mm on the fan/exhaust side, so the **cooling geometry gate fails** and reports a preliminary minimum fuselage width of about 142.4 mm.

This is intentional: the model does not hide a cooling integration conflict. The user can widen the fuselage or reduce the gap requirement after a real shroud/bench test.

## Model limitations / next evidence

Before design freeze, replace the assumed fan curve with one of:

1. Intelligent Energy module airflow / allowable external pressure-drop data; or
2. a bench measurement with the actual inlet/outlet mock-up.

Recommended bench measurements are inlet temperature, exhaust temperature, air velocity/flow, fuel-cell net output and any available stack temperature telemetry.
