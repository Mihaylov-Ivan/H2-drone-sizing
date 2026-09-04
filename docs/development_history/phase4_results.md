# Phase 4 Reference Results — Aerodynamics + Lift/Cruise Propulsion

## Scope and evidence level

Phase 4 closes the most important aerodynamic/propulsion trade around the Phase-3 aircraft: wing/stall screening, inactive lift-propeller cruise drag, lift-propeller diameter versus motor spacing, 15 kg VTOL sizing, pusher operating-point screening and re-coupling of Phase-4 loads into the Phase-2 dynamic FC/battery EMS.

The source-model baseline is preserved by calibration. Lift-prop cruise-state factors and the generic pusher map are **engineering assumptions**, not hardware evidence. Real motor/propeller selection remains blocked until 12S thrust-stand/manufacturer maps are supplied.

## Baseline reproduction

At the source-model reference point (10.893 kg, 20 m/s, 18 in stopped-random lift props), Phase 4 reproduces:

- source total drag: 18.905 N
- Phase-4 calibrated total drag: 18.9045 N
- source cruise propulsion electrical power: 525.147 W
- Phase-4 calibrated cruise propulsion electrical power: 525.125 W

The small difference is rounding in the reported source values.

## Lift-prop cruise-state consequence at Phase-3 screening mass

For 11.0777 kg, 20 m/s and 18 in lift propellers:

| Lift-prop cruise state | Lift-prop drag | Total drag | Cruise propulsion electrical | Total cruise bus incl. 35 W aux |
|---|---:|---:|---:|---:|
| folded | 0.883 N | 11.028 N | 306.3 W | 341.3 W |
| stopped_aligned | 2.649 N | 12.794 N | 355.4 W | 390.4 W |
| stopped_random | 8.830 N | 18.974 N | 527.1 W | 562.1 W |
| windmilling | 17.659 N | 27.804 N | 772.3 W | 807.3 W |

Under the current source-model drag law, moving from stopped-random to folded reduces modeled cruise drag by **7.95 N** and modeled total cruise bus demand by **220.7 W**. Because the underlying projected-area factors are conceptual, this is a design-direction result, not a promised flight performance number.

## Dynamic Phase-2 EMS re-check for a 120 min modeled cycle

The Phase-4 stage powers were fed into the Phase-2 time-resolved IE-SOAR/H2/battery model. The fixed VTOL/transition stages remain preliminary; Phase 5 will replace them with transition dynamics.

| 18 in state | Bus energy | H2 used | Final battery SOC | FC operational energy remaining | Result |
|---|---:|---:|---:|---:|---|
| folded | 704.8 Wh | 46.45 g | 93.3% | 193.9 Wh | PASS |
| stopped_aligned | 801.4 Wh | 52.95 g | 93.3% | 97.3 Wh | PASS |
| stopped_random | 1139.5 Wh | 59.50 g* | 20.0% | 0 Wh | FAIL |

\*The stopped-random case reaches the protected operational H2 reserve and then depletes the battery to its minimum in the screen. Planned battery endurance extension is disabled, as required by the Phase-2 EMS policy.

This makes reliable folding or low-drag indexing of the lift propellers a high-priority architecture item.

## 15 kg VTOL ceiling screen

The existing lift-sizing equation uses maximum take-off mass, 0.75 m/s² vertical acceleration, 1.50 thrust margin, 3 m/s climb speed, 0.62 effective lift efficiency and 1.15 power margin.

At 15 kg the required design thrust is:

- total: 237.52 N
- per lift motor: **59.38 N**

The current placeholder combination is rated 55 N/motor, so it **fails the 15 kg thrust gate**. Under the same model equation, 55 N/motor supports only **13.893 kg** maximum mass.

15 kg design peak power screen:

| Lift prop diameter | Total design peak | Per motor | 5 kW application screen | Conditional 7.5 kW hybrid screen |
|---|---:|---:|---|---|
| 14 in | 7.646 kW | 1.912 kW | FAIL | FAIL |
| 15 in | 7.185 kW | 1.796 kW | FAIL | PASS* |
| 16 in | 6.781 kW | 1.695 kW | FAIL | PASS* |
| 17 in | 6.426 kW | 1.606 kW | FAIL | PASS* |
| 18 in | **6.110 kW** | **1.527 kW** | FAIL | PASS* |

\*Conditional only: the supplier-reported hybrid peak depends on the selected battery. Battery current, voltage sag and thermal limits remain unverified.

The 18 in diameter is therefore the strongest current design direction for peak-power reduction, but the placeholder motor itself is still insufficient in thrust.

## Lift-motor geometry

Current front/rear lift-motor centres are 0.31 / 0.69 m, only 0.380 m apart. An 18 in prop with 20 mm edge clearance needs 0.4772 m.

A symmetric re-spacing about x=0.50 m gives:

- front x = **0.2614 m**
- rear x = **0.7386 m**
- each motor row moves 48.6 mm away from the midpoint
- required 20 mm disk-edge clearance is recovered in the Phase-4 geometric screen

This is a screening layout only. Wing spar, boom, CG, wiring and CAD interfaces must be re-closed before release.

## Wing/stall screen

Phase 4 scales stall speed from the source-model reported 14.75 m/s baseline, which preserves the source model's tail-trim convention. With the current 0.634 m² wing, the 11.0777 kg Phase-3 mass gives 14.87 m/s stall and still clears the 1.30 V/Vstall handoff screen at 20 m/s. The current wing becomes marginal above roughly the 12 kg region.

Minimum wing areas to make 20 m/s equal to 1.30 Vstall, preserving the current aspect ratio only for screening:

| Mass | Min wing area | Screened span | Current 0.634 m² passes? |
|---|---:|---:|---|
| 12 kg | 0.642 m² | 2.938 m | No (slightly) |
| 14 kg | **0.749 m²** | **3.174 m** | No |
| 15 kg | **0.8025 m²** | **3.285 m** | No |

At the 15 kg minimum-area screen, 18 in folded lift props give about 377 W total cruise-bus demand at 20 m/s; stopped-aligned gives about 427 W. Structural mass, CG, tail sizing and packaging are not yet fed back into this wing-area sensitivity, so these are not released geometry values.

## Pusher screen

A generic Ct/Cp map was added so the pusher is evaluated around cruise advance ratio rather than static thrust only. For the 11.0777 kg / 20 m/s / 18 in folded-lift-prop aircraft, the calibrated drag is 11.03 N and the sizing thrust with 1.30 margin is 14.34 N.

The generic 14–18 in pusher sweep finds feasible conceptual points. The 18 in case is approximately 3800 rpm, 14.44 N, 360 W shaft, 404 W electrical at the sizing-margin point and helical tip Mach 0.27. These values are **not** manufacturer or thrust-stand data; no pusher hardware should be frozen from them.

## Phase-4 design direction

The strongest current direction is:

**retain a relatively large (~18 in) lift disk, use folding lift propellers, re-space the front/rear lift motor stations, resize the wing for the actual 12–15 kg converged mass/stall gate, and select new 12S lift hardware capable of at least the 15 kg thrust requirement.**

The key open hardware evidence is a real 12S motor/prop/ESC thrust-power-current-temperature map and a real pusher efficiency map at the intended cruise operating point.
