# Twin-boom VTOL conversion — Step 2

Replace the corresponding project files with the seven `.py` files in this folder.

This step implements:

- four dedicated lift motor/propeller units;
- one dedicated rear cruise motor/propeller;
- separate lift and cruise thrust/power validation;
- separate lift and cruise ESC sizing;
- PEM fuel-cell, hydrogen storage and buffer-battery mass accounting;
- stage-by-stage fuel-cell/battery power allocation;
- complete take-off, climb, transition, cruise, reverse transition, descent and landing energy reserve;
- hybrid-energy-limited maximum cruise endurance;
- removal of the old tailsitter tail-propeller clearance and tail propulsion-load assumptions.

The current MTOW loop includes the complete hybrid energy-system mass, but it still places that mass at the legacy single battery CG. Individual fuel-cell, tank, regulator, battery, lift-motor and cruise-motor CG positions will be integrated in the next step.
