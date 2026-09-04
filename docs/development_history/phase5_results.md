# Phase 5 Reference Results — Dynamic Transition and Mission

## Scope and fidelity

Phase 5 replaces the Phase-4 fixed 8 s transition/reverse-transition power blocks with a time-domain 2-D point-mass simulation. It resolves horizontal/vertical velocity and position, pitch scheduling, angle of attack, wing lift/drag, lift-motor unloading/spool-up, pusher thrust/RPM/power, lift-prop transition drag, motor response lag, altitude-hold demand, transition energy and abort gates. The resulting power history is passed directly to the Phase-2 fuel-cell/battery EMS.

This is **not yet the Phase-6 6-DOF flight-control model**. Rotational equations of motion, control-surface allocation, actuator dynamics, propwash moments, gust/wind response and ArduPilot controller behavior are not release-validated here.

## Reference aircraft screen

Reference basis carried forward from Phases 3–4:

- mass: 11.0777 kg screening value
- wing: 0.634 m², 2.92 m span
- lift props: 4 × 18 in
- cruise lift-prop state: folded
- current lift-thrust placeholder: 55 N/motor
- pusher: 18 in generic Ct/Cp map; no selected-hardware claim
- cruise: 20 m/s

### Forward transition

The time-domain reference transition reaches the wing-borne handover gate successfully:

| Metric | Result |
|---|---:|
| transition time | **19.34 s** |
| horizontal distance | **220.0 m** |
| minimum altitude loss | **0.00 m** |
| peak sink rate | <0.05 m/s |
| handover airspeed | **19.64 m/s** |
| handover V/Vstall | **1.320** |
| final wing lift / weight | **0.914** |
| final lift-rotor thrust / weight | **0.072** |
| maximum transition bus power | **1.774 kW** |
| maximum pusher electrical power | **0.743 kW** |
| transition energy | **6.87 Wh** |
| maximum |alpha| | **6.11 deg** |

The lift rotors are therefore largely unloaded before handover rather than being switched off at an arbitrary time.

### Reverse transition

| Metric | Result |
|---|---:|
| reverse-transition time | **17.20 s** |
| horizontal distance | **174.6 m** |
| maximum altitude loss | **1.75 m** |
| final airspeed | **2.44 m/s** |
| final lift-rotor thrust / weight | **0.993** |
| maximum bus power | **1.474 kW** |
| reverse-transition energy | **4.18 Wh** |

The modeled vehicle decelerates with aerodynamic drag and nose-up attitude while the vertical lift system progressively assumes the aircraft weight. This is a point-mass screening strategy only; Phase 6 must prove pitch/control authority and moments.

## 120-minute mission with dynamic transitions

The Stage-4 transition blocks have been replaced by the actual power histories above. For the same 120-minute mission target:

- total bus energy: **704.97 Wh**
- cruise duration: **7068.46 s / 117.81 min**
- cruise bus power: **341.32 W**
- vertical-climb bus power: **1.967 kW**
- peak mission bus power: **1.967 kW**
- peak battery contribution: **1.360 kW**
- modeled peak battery current: **27.43 A** (using the Phase-2 assumed pack resistance/OCV model)
- minimum/final SOC: **93.78%**
- H2 consumed: **46.53 g**
- physical H2 remaining: **23.47 g**
- protected operational FC energy remaining: **192.77 Wh**
- Phase-2 EMS result: **PASS**

This is an important consistency check: Phase 4's simpler folded-prop 120-minute energy screen was about 704.8 Wh, while the new dynamic transition mission is 704.97 Wh. The new dynamics therefore add fidelity without creating an unexplained energy discontinuity.

## 15 kg design-ceiling transition screen

A second case was run at the 15 kg ceiling using the Phase-4 minimum stall-margin wing screen (0.8025 m² / 3.285 m) and a **65 N/motor conceptual lift capability**. The 65 N value is an engineering target, not selected hardware.

### Forward

- transition: **27.24 s**
- distance: **377.1 m**
- altitude loss: **0.00 m**
- handover V/Vstall: **1.301**
- maximum bus power: **2.775 kW**
- maximum pusher power: **0.966 kW**
- transition energy: **11.74 Wh**

### Reverse

- reverse transition: **18.16 s**
- distance: **186.4 m**
- altitude loss: **2.06 m**
- maximum bus power: **2.304 kW**
- reverse-transition energy: **7.00 Wh**

### 120-minute EMS screen at 15 kg

- total mission bus energy: **792.67 Wh**
- cruise bus power: **377.46 W**
- peak bus power: **3.007 kW**
- peak battery contribution: **2.207 kW**
- modeled peak battery current: **45.69 A**
- final SOC: **86.64%**
- H2 consumed: **51.37 g**
- Phase-2 EMS: **PASS**

The same 15 kg dynamics can still hover/transition using the old 55 N placeholder because 4×55 N exceeds weight. However, **55 N/motor fails the Phase-4 design-thrust-margin requirement of 59.38 N/motor**, so it remains a design-gate FAIL. Dynamic survival is not a substitute for thrust margin.

## Pusher requirement exposed by transition

The forward-transition sensitivity sweep gives:

| Pusher electrical limit | Result | Final speed / handover |
|---:|---|---:|
| 400 W | FAIL | 5.00 m/s |
| 500 W | FAIL | 7.15 m/s |
| 600 W | FAIL | 13.44 m/s |
| 650 W | FAIL | 16.11 m/s |
| **700 W** | **PASS** | 19.67 m/s, 25.24 s |
| **750 W+** | **PASS** | 19.64 m/s, 19.34 s |

Within the present generic-prop/controller assumptions, the reference aircraft therefore needs roughly a **700 W-class minimum transition capability**, while ~750 W avoids the long saturated transition. This is not a motor purchase specification until the pusher Ct/Cp map is replaced with measured or supplier data.

## Active lift-prop drag sensitivity

Changing the uncertain powered-transition rotor-drag factor from 0.3 to 1.0 of the Phase-4 windmilling reference changes maximum pusher demand from ~667 W to ~796 W and transition energy from ~6.66 to ~7.03 Wh. Transition still closes in all four cases with the 1.2 kW pusher screen.

This confirms that transition prop drag matters to pusher sizing, even though it is not a large fraction of total mission energy.

## Key engineering conclusion

Phase 5 supports the current 4+1 architecture at screening level, but it changes the hardware priorities:

1. The cruise pusher must not be sized only for ~300–400 W cruise. It needs roughly **0.75–1.0 kW transient capability** if it is also responsible for a practical transition to 20 m/s.
2. The lift system must retain the Phase-4 **15 kg design thrust margin** even though normal transition/hover power is lower than the 6.11 kW worst-case design-power screen.
3. Reverse transition needs **~2 m altitude allowance** in the present point-mass screen; this should be treated as an initial envelope-expansion parameter, not a guaranteed flight-test value.
4. Folding/indexing logic must be coordinated with lift-thrust unloading; folding is only permitted after low lift-thrust fraction plus a delay in the Phase-5 model.
5. The next major uncertainty is no longer mission energy. It is **attitude/control authority and rotational dynamics through transition**, which is the Phase-6 task.
