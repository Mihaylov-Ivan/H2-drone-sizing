# Changelog — Phase 2

## Added

- `phase2_hybrid_power.py`: time-resolved FC/H2/battery/EMS model.
- `phase2_config.py`: strict JSON configuration loader.
- `phase2_mission_adapter.py`: adapter for existing H2 mission stage dictionaries.
- `run_phase2.py`: standalone load-cycle runner.
- IE-SOAR 800 application-reference config and explicit assumption register.
- Representative 450 W cruise / 5 kW VTOL load cycle.
- Electrical, SOC, H2 reserve and conditional hybrid-peak gates.
- FC fault injection.
- Unit/self tests.

## Preserved

All Phase-1 governance files are included. No Phase-1 traceability or validation capability was removed.

## Not yet changed

- aerodynamic cruise-power derivation;
- motor/propeller maps;
- mass/CG geometry closure;
- transition force balance;
- 6-DOF;
- structural model.

Those remain subsequent phases so changes are independently testable.
