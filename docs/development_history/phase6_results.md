# Phase 6 Results — 6-DOF Stability and Control

## Current geometry
- Static margin: **-13.28% MAC — FAIL**.
- 20 m/s nonlinear trim exists: alpha ~**6.27 deg**, elevator ~**+4.94 deg**, pusher thrust ~**9.12 N**.
- Forward 6-DOF transition screen diverges in pitch: max tracking error ~**54.0 deg**, final speed only ~**14.8 m/s** before the attitude abort.
- Reverse transition similarly diverges.
- Current-layout controlled cruise case diverges before the scheduled crosswind gust, so active control is not accepted as a substitute for positive static stability in this design phase.

## +8% static-margin design candidate
This is a hypothetical stability-corrected geometry used to test whether the remaining assumed control authority is adequate.

- Static margin: **+8.0% MAC**.
- Cruise trim: alpha **6.27 deg**, elevator **-0.88 deg**, pusher thrust **9.12 N**.
- Forward transition: **30.0 s simulation**, final airspeed **20.45 m/s**, final stall ratio **1.331**, max pitch tracking error **3.60 deg**, negligible altitude loss, max rotor utilization **50.7%**, max surface utilization **3.3%**.
- Reverse transition: final speed **2.16 m/s**, altitude loss **1.96 m**, max pitch tracking error **4.66 deg**, max rotor utilization **50.3%**.
- 5 m/s lateral gust with controller: max roll **2.16 deg**, max yaw **12.85 deg**, altitude loss **0.088 m**, max surface utilization **32.0%**, max rotor utilization **14.8%**.
- The Phase-6 gate set passes **15/15** for this candidate, subject to the assumption register.

## Lift-motor allocation result
The current respaced 18-inch rotor rows are centered **50.39 mm aft of CG**. Therefore equal lift thrust produces a nose-down pitch moment.

At current 11.08 kg hover, zero-pitch allocation is:
- front motors: **32.89 N each**
- rear motors: **21.42 N each**
- front/rear per-motor ratio: **1.535**

At the Phase-4 15 kg design total lift requirement (**237.52 N**), zero-pitch allocation requires:
- front motors: **71.92 N each**
- rear motors: **46.84 N each**

Thus the Phase-5 **65 N/motor design candidate is insufficient for the 15 kg / 1.5-thrust-margin design case with the present CG/rotor layout**. Either shift the rotor centroid toward the final CG, reduce the required design load through justified criteria, or select higher-thrust front/all motors with additional control margin.

## Stability geometry implication
Keeping the inherited neutral point fixed at x=0.420 m would require:
- +5% SM: CG x <= **0.40885 m**, a **40.76 mm** forward shift.
- +8% SM: CG x <= **0.40216 m**, a **47.45 mm** forward shift.

This is not a recommendation to add ballast. Moving the neutral point aft through tail/wing geometry may be the better mass-efficient solution. Phase 7/8 should solve stability, rotor centroid, mass/CG and structure together.
