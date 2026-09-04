# Phase 2 preliminary results — power-system screen

These results are **not an aircraft endurance prediction**. They use the current application-reference energy system and a fixed preliminary electrical load cycle so the hybrid power architecture can be screened before the aerodynamic and propulsion models are recoupled.

## Reference basis

- IE-SOAR 800 continuous class.
- 1.45 kg module reference mass.
- 24–48 V output reference.
- 315 g regulator.
- certified 3.0 L / 350 bar storage direction.
- ~0.070 kg H2 preliminary inventory.
- supplier/application reference: 2.6 h at 400 W -> 1.04 kWh net reference energy.
- preliminary 15% mission-energy reserve -> 0.884 kWh normal operational FC/H2 budget.
- preliminary 12S 5 Ah buffer -> 44.4 V nominal / 222 Wh.
- normal cruise is **not deliberately extended on battery** after the protected H2 budget is reached; battery remains a transient and landing/go-around reserve.

## Representative 111.85 min load-cycle run

Load cycle: 30 s ready, 10 s 5 kW VTOL, 10 s 4.2 kW climb, 8 s 5 kW transition, 20 s 1.2 kW acceleration/climb, 110 min at 450 W cruise, then 5 kW reverse transition, descent and landing.

Result:

- feasible in the Phase-2 energy model;
- peak aircraft bus load: 5.00 kW;
- peak FC bus output: 0.80 kW;
- peak buffer-battery power: ~4.82 kW;
- calculated peak battery current: ~101.8 A using the explicit **engineering-assumption** 30 mΩ pack resistance;
- minimum calculated pack voltage: ~44.65 V with the preliminary OCV/sag model;
- final SOC: ~71.9%;
- H2 used: ~56.7 g;
- protected normal-operation H2 budget is not exceeded.

Battery current and thermal limits remain `NOT_EVALUATED` because the selected pack's verified limits are not in the current source set.

## Conservative maximum mission screen

Using the same fixed non-cruise stages and prohibiting planned battery endurance extension after the protected H2 budget:

| Battery | Cruise bus power | Max total mission | >60 min minimum | >=120 min target |
|---|---:|---:|---:|---:|
| 12S 4 Ah | 400 W | ~132.8 min | PASS | PASS |
| 12S 4 Ah | 450 W | ~118.3 min | PASS | FAIL / marginal |
| 12S 4 Ah | 550 W | ~97.1 min | PASS | FAIL |
| 12S 5 Ah | 400 W | ~132.8 min | PASS | PASS |
| 12S 5 Ah | 450 W | ~118.3 min | PASS | FAIL / marginal |
| 12S 5 Ah | 550 W | ~97.1 min | PASS | FAIL |

The 4 Ah and 5 Ah endurance results are essentially identical because the conservative policy does not use the battery as the primary endurance store; the battery's real importance is peak power, voltage sag, thermal capability and landing/go-around reserve.

Under this particular preliminary load cycle, the maximum representative cruise-bus power that closes a 120 min total mission is approximately **443 W**. This is a useful design target for the next aero/propulsion phases, not a guaranteed aircraft performance number.

## Design consequence

The current application target of 400–450 W cruise is correctly placed, but 450 W is already close to the 120 min target on the present conservative energy basis. Phase 4 therefore needs to demonstrate that the converged aircraft can cruise near the low end of that band, or Phase 2 must be updated with better measured usable H2 energy / reserve data before claiming >=120 min margin.
