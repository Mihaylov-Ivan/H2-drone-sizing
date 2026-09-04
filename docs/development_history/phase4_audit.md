# Phase 4 Engineering Audit

## What was corrected

1. **Propeller selection is no longer treated as "smallest static-thrust-feasible prop wins."** Phase 4 exposes the mission trade between disk diameter, 15 kg VTOL power, lift-prop cruise drag and geometric spacing.
2. **The current drag model is calibrated before being modified.** At the reference condition it reproduces the source result, so improvements are attributable to the Phase-4 variables instead of an unrelated new drag model.
3. **Lift-prop cruise state is treated as an architecture decision.** Folded, indexed/aligned, random-stopped and windmilling states produce separate drag/energy outcomes.
4. **The 15 kg ceiling is a hard propulsion sizing screen.** The current 55 N placeholder no longer receives an implicit pass merely because it works near the current ~11 kg screening mass.
5. **Wing area is screened against the transition stall-margin requirement at target/ceiling masses.** The source-model stall result is used as the calibration anchor.
6. **Pusher selection is evaluated at advance ratio.** The generic map is clearly labelled an engineering assumption and can be replaced by measured Ct/Cp or thrust-power data without changing the interface.
7. **Phase-4 aerodynamic loads are fed back into the Phase-2 EMS.** This makes the endurance consequence of lift-prop cruise drag visible in H2 reserve and battery SOC.

## Still not released / not validated

- no real 12S lift motor/propeller thrust stand data;
- no verified lift-motor/ESC thermal map;
- no selected battery current/thermal limits;
- no measured stopped/folded lift-prop cruise drag;
- no measured pusher map;
- wing-area changes are not yet coupled back into the inaccessible original structure/tail/CG runtime;
- transition is still a stage-level energy approximation until Phase 5;
- Phase-3 pod hardware collisions remain unresolved;
- Phase-3 static-margin issue must be recalculated after wing/tail/CG geometry changes.

## Release consequence

Phase 4 is suitable for **architecture and test planning**, not production release. The next physical decisions should be supported by thrust-stand data, actual folding-prop hardware behavior and a re-run of the full coupled source-tree model with the new wing/motor layout.
