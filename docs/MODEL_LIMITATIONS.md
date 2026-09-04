# Model limitations — Integrated v2.0

The model is a multidisciplinary engineering screening and development tool, not certification evidence.

The following items remain open or assumption-limited:

- selected 12S battery mass, dimensions, current limits, resistance and thermal data;
- exact certified 3.0 L / 350 bar cylinder mass/envelope and installation hardware;
- controlled motor-mount CAD, fasteners, laminate/material allowables and local FEA;
- measured 12S lift motor/prop thrust-power maps and forward-flight pusher map;
- nonlinear aerodynamic/control derivatives from AVL/CFD/wind tunnel/flight test;
- transition aerodynamics outside the linear-derivative validity region;
- exact IE-SOAR efficiency/transient/thermal map beyond the currently documented reference basis;
- real packaging service access, cooling airflow, H2 safety separation, plumbing bend radii and maintenance clearances;
- manufacturing tolerances, environmental temperature/altitude effects and actuator reliability;
- uncertainty distributions: current defaults are engineering assumptions and must be replaced with evidence-backed distributions.

`SCREENING_PASS` therefore means that no currently evaluated hard model gate fails. It does **not** mean flight release, airworthiness approval or hardware qualification.
