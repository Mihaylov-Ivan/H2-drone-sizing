# Phase 6 Engineering Audit

## Evidence boundary
Phase 6 is deliberately a screening model. Mass/CG/inertia and the Phase-5 reference trajectories come from the existing cumulative model. The current neutral point/CG combination gives a calculated static margin of about -13.28%. Most dynamic stability derivatives, control-surface effectiveness, actuator rates, prop reaction torque and the pusher map are engineering assumptions and are tagged accordingly.

## Critical finding 1 — current longitudinal layout remains unacceptable
The current CG station is 0.4496 m and the inherited neutral point is 0.420 m with MAC 0.223 m, giving SM = -13.28%. A nonlinear cruise trim exists at 20 m/s (about 6.27 deg alpha and +4.94 deg elevator in the screening sign convention), but trim existence is not stability. In the Phase-6 nonlinear dynamic screen the current geometry diverges in pitch during forward/reverse transition and during controlled cruise before the scheduled gust is even applied. This is a design FAIL, not a controller-tuning PASS.

If the neutral point stayed at 0.420 m, +5% static margin would require CG <= 0.40885 m (40.76 mm forward of the current CG), while +8% would require CG <= 0.40216 m (47.45 mm forward). The preferred solution should jointly move CG and/or move the neutral point aft by tail/wing geometry; do not assume ballast is required.

## Critical finding 2 — lift-rotor centroid / CG mismatch changes motor sizing
The Phase-4 respaced 18-inch motor rows are centered at x=0.500 m while the Phase-3 CG is x=0.4496 m. The lift resultant therefore sits 50.39 mm aft of CG.

At 11.08 kg trimmed hover, zero pitch moment requires approximately:
- front-left/right: 32.89 N each
- rear-left/right: 21.42 N each
- front/rear per-motor ratio: 1.535

The unequal loading raises ideal/effective hover power by about 1.7% versus the same total thrust shared equally, before motor/ESC efficiency differences.

For the Phase-4 15 kg design total thrust requirement of 237.52 N, exact zero-pitch allocation requires about 71.92 N on each front motor and 46.84 N on each rear motor. Consequently four nominal 65 N motors cannot simultaneously provide the Phase-4 total design thrust and zero pitch moment with the current geometry. This supersedes the less-coupled Phase-5 65 N design screen.

Preferred architecture action: align the lift-rotor centroid much closer to the final CG. With the current 477.2 mm row separation, centering the rows on the present CG would put them near x=0.2110 and 0.6882 m. If CG is moved forward for stability, the motor-row center must be reconsidered again.

## Candidate stable-layout screen
A deliberately hypothetical +8% static-margin case was run without claiming how the geometry achieves it. This candidate closes nonlinear trim with about -0.88 deg elevator, tracks the forward transition with max pitch error about 3.60 deg, reaches 20.45 m/s with no material altitude loss, and tracks reverse transition to 2.16 m/s with about 1.96 m altitude loss.

Under a 5 m/s lateral step gust, the same candidate keeps roll to about 2.16 deg, yaw to about 12.85 deg and altitude loss below 0.1 m with maximum surface utilization about 32%. These results are useful controller/authority screens, but remain derivative-dependent.

## Release blockers before flight-control freeze
1. Recompute neutral point and stability derivatives from the final Phase-4/Phase-7 geometry.
2. Select a positive static-margin geometry; do not rely on active software stabilization of the current -13% margin design.
3. Resolve lift rotor centroid versus CG and re-size lift motors using per-motor allocation, not total thrust/4.
4. Replace assumed control-surface derivatives with actual geometry/AVL/CFD values.
5. Replace servo travel/rate assumptions with selected servo/linkage data.
6. Replace prop reaction torque and pusher map with bench/supplier data.
7. Correlate the model to hover/transition/forward-flight logs before increasing flight envelope.
