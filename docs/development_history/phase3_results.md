# Phase 3 Reference Results

## Screening mass ledger

After reconciling the current H2AIR itemized baseline with the Phase-2 IE-SOAR / 3.0 L reference values:

- take-off mass: **11.0777 kg**
- end-of-representative-mission mass: **11.0210 kg**
- numerical margin to 15 kg ceiling: **3.9223 kg**
- payload: **1.200 kg** (10.83% of screening MTOW)
- airframe group: **4.3063 kg**
- energy group: **3.6850 kg**
- propulsion group: **1.3000 kg**
- avionics/electrics group: **0.5864 kg**

This is **not a released MTOW** because the selected 12S battery mass and selected 3.0 L / 350 bar cylinder mass are not yet available. Their old model values are retained only to keep the ledger numerically closed.

## 3-D mass properties

Take-off CG:

- x = **0.44961 m**
- y = **0.00000 m**
- z = **0.03951 m**

Representative mission-end CG:

- x = **0.45018 m**
- y = **0.00000 m**
- z = **0.03972 m**

CG movement over the Phase-2 representative mission:

- dx = **+0.56 mm aft**
- dy = **0.00 mm**
- dz = **+0.20 mm**

Take-off inertia screen:

- Ixx = **2.2781 kg m^2**
- Iyy = **0.5490 kg m^2**
- Izz = **2.7551 kg m^2**
- Ixy = **0.0000 kg m^2**
- Ixz = **0.01843 kg m^2**
- Iyz = **0.0000 kg m^2**

The inertia tensor passes the positive-definite consistency screen. Some z-locations and hardware envelopes are preliminary, therefore these are design-screening inertias, not flight-controller identification values.

## Major design findings

### 1. Current component mass stations are not a valid packaging layout

Using the available legacy hardware envelopes at the current x stations, the model detects three overlaps:

- battery <-> fuel-cell module
- battery <-> H2 cylinder
- fuel-cell module <-> H2 cylinder

Each of the known major envelopes can fit inside the 0.78 x 0.18 x 0.18 m pod individually, but the current mass-station coordinates do not form a collision-free arrangement. Several other components also lack dimensions, so packaging cannot yet be released.

### 2. Current 18-inch lift-prop placeholder is geometrically incompatible with current motor centres

The governing same-side front/rear motor centre spacing is **0.380 m**.

- 18-inch propeller diameter = 0.4572 m
- resulting disk overlap = **77.2 mm**
- required design edge clearance = 20 mm
- maximum equal prop diameter at the current centres and 20 mm clearance = **14.17 in**
- to keep 18-inch props with 20 mm clearance, governing centre spacing must increase to **0.4772 m**, i.e. by **97.2 mm** relative to the present 0.380 m spacing.

This is a geometry gate, not a motor-selection conclusion. Phase 4 should jointly choose prop diameter and motor locations from the real thrust requirement.

### 3. Longitudinal stability cross-check remains a critical open issue

The current accessible aircraft result reports an inherited neutral point near x = 0.420 m. With the Phase-3 screening CG of x = 0.4496 m and MAC = 0.223 m, the fixed-neutral-point cross-check gives:

- static margin = **-13.28% MAC**

At that same fixed neutral point, a 5% MAC minimum would require CG x <= **0.4088 m**, approximately **40.8 mm forward** of the Phase-3 screening CG.

Do not solve this by adding ballast yet. The neutral point and tail sizing must be recomputed after Phase-4 aerodynamic and layout changes; moving wing/tail geometry changes the neutral point as well as the CG.

### 4. Hydrogen depletion is not currently the dominant CG problem

Sweeping the modeled H2 inventory from 70 g to zero shifts x-CG only about **0.70 mm aft**. The dominant longitudinal balance is therefore the placement of the kilogram-class airframe, FC, battery, cylinder, payload and propulsion components rather than hydrogen consumption itself.

## Closure

The fixed-point closure engine is implemented with relaxation, independent tolerances and iteration history. The included example converges in 9 iterations, but is explicitly a **ledger-reconciliation plumbing test**, not a physical aircraft closure. The physical evaluator must be connected to the original live structure/aero/propulsion model; no arbitrary mass or power exponents were introduced in Phase 3.
