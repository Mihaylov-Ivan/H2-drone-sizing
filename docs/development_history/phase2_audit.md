# Phase 2 audit — hybrid hydrogen-electric system

## What was wrong with the previous allocator

The current `propulsion_system.py` already separates lift and cruise propulsion and generates sensible mission stages, but its energy allocator reduces the hybrid system to two storage buckets:

- a fixed maximum FC bus power;
- one precomputed FC electrical-energy store;
- battery energy above minimum SOC.

The approach is useful for first sizing but misses several build-critical effects:

1. Fuel-cell ramp rate is defined in inputs but is not actually resolved time-step by time-step in the allocator.
2. Battery power is checked mainly from nominal V × Ah × C-rate; voltage sag is not solved.
3. FC/battery SOC recovery is not a supervisory state machine.
4. A constant FC efficiency converts the entire H2 inventory to one electrical-energy number.
5. Battery current/voltage/thermal limits cannot remain traceably unknown — they need `NOT_EVALUATED` states until real data arrive.
6. The old source uses a 22.2 V / 6S buffer whereas the current application architecture is 48 V-class / preliminary 12S.
7. The old storage is explicitly a 300 bar / 120 g placeholder whereas the current procurement direction is certified 3.0 L / 350 bar and ~70 g preliminary inventory.

## Phase 2 changes

- 12S / 44.4 V nominal reference bus (48 V class).
- IE-SOAR 800 reference: 800 W continuous, 1.45 kg module, 24–48 V output.
- 315 g regulator and 3.0 L / 350 bar storage reference.
- Supplier/reference energy method: 2.6 h × 400 W = 1.04 kWh; 15% mission reserve leaves 0.884 kWh operational basis.
- Time-resolved FC ramping.
- EMS states: peak support, load follow, SOC recovery, battery-only, safe landing.
- Battery current and terminal-voltage solution with configurable OCV and internal resistance.
- SOC integration with charge/discharge efficiencies.
- Optional battery thermal model.
- H2 consumption integrated with time; direct measured FC efficiency map can replace the reference-energy method later.
- Conditional 7.5 kW hybrid-peak gate and 5 kW application screening warning.
- Protected hydrogen energy reserve and landing battery SOC reserve.
- Unknown battery current/thermal limits are `NOT_EVALUATED` rather than silently passed.
- Fault-injection hook for FC loss.

## Critical evidence boundary

The application supports the 800 W continuous rating, 1.45 kg module mass, 315 g regulator, 24–48 V output, conditional 7.5 kW hybrid peak, 3.0 L/350 bar package and 2.6 h at 400 W reference. It also gives ~70 g H2 and 0.88 kWh after a preliminary 15% reserve as preliminary sizing values.

It does **not** yet support an exact cylinder mass, detailed FC efficiency map, dynamic ramp map, battery internal resistance, battery current limits or battery thermal properties. Phase 2 therefore models these as open/configurable inputs and never promotes them to supplier-confirmed facts.
