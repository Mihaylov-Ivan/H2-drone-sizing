from copy import deepcopy

from h2air.hybrid_power import (
    BatteryConfig,
    FuelCellConfig,
    HybridPowerConfig,
    HydrogenConfig,
    _battery_current_voltage_for_power,
    enforce_bus_voltage_ceiling_on_mapping,
)
from h2air.hardware_selection import select_hardware
from h2air.user_inputs import load_user_input_config
from pathlib import Path


def test_pack_ocv_never_exceeds_48v():
    batt = BatteryConfig()
    assert batt.maximum_pack_voltage_V <= 48.0 + 1e-9
    assert batt.pack_ocv_V(1.0) <= 48.0 + 1e-9


def test_enforce_clamps_legacy_4p2_curve():
    ps = {
        "fuel_cell": {"output_voltage_max_V": 48.0},
        "battery": {
            "series_cells": 12,
            "maximum_charge_cell_voltage_V": 4.2,
            "ocv_curve_cell_V": [[0.0, 3.45], [1.0, 4.2]],
        },
    }
    enforce_bus_voltage_ceiling_on_mapping(ps)
    assert ps["battery"]["maximum_charge_cell_voltage_V"] == 4.0
    assert ps["battery"]["ocv_curve_cell_V"][-1][1] == 4.0
    assert ps["aircraft_bus_voltage_ceiling_V"] == 48.0


def test_charge_voltage_limited_at_48v():
    batt = BatteryConfig(maximum_charge_cell_voltage_V=4.0)
    # Request large charge power at full SOC OCV.
    P, I, V, warns = _battery_current_voltage_for_power(batt, 1.0, -2000.0)
    assert V <= 48.0 + 1e-9
    assert batt.pack_ocv_V(1.0) <= 48.0 + 1e-9


def test_hybrid_config_rejects_pack_above_fc_max():
    cfg = HybridPowerConfig(
        fuel_cell=FuelCellConfig(output_voltage_max_V=48.0),
        hydrogen=HydrogenConfig(),
        battery=BatteryConfig(maximum_charge_cell_voltage_V=4.2),
    )
    # Bypass constructor clamp by forcing invalid battery after init fields
    cfg.battery.maximum_charge_cell_voltage_V = 4.2
    cfg.battery.ocv_curve_cell_V = [(0.0, 3.45), (1.0, 4.2)]
    errs = cfg.validate()
    assert any("exceeds fuel-cell" in e or "OCV curve peaks" in e for e in errs)


def test_hardware_selection_bus_max_capped_at_48():
    user = {
        "mission": {"mtow_ceiling_kg": 15.0},
        "hardware": {"fuel_cell": {"output_voltage_max_V": 48.0}},
        "hardware_selection": {
            "mode": "hardcoded",
            "hardcoded": {
                "lift_combo_name": "Tmotor_U8Lite_150kV_P22x6p6_12S",
                "pusher_combo_name": "Tmotor_MN501S_240kV_P15x5_12S",
                "battery_name": "SeriesPair_12S_Lipo_Battery_Pack_5500mAh/22.2V_30/60C_177.6Wh",
            },
            "auto_criteria": {
                "series_cells": 12,
                "nominal_cell_voltage_V": 3.7,
                # Legacy 4.2 V/cell would imply 50.4 V; must be clamped by FC ceiling.
                "maximum_cell_voltage_V": 4.2,
                "number_of_lift_motors": 4,
                "lift_total_thrust_factor_at_mtow_ceiling": 1.5,
                "lift_climb_thrust_factor": 1.08,
                "minimum_pusher_power_W": 750.0,
                "minimum_battery_capacity_Ah": 5.0,
                "battery_current_margin": 1.05,
                "minimum_lift_prop_diameter_in": 18.0,
                "maximum_lift_prop_diameter_in": 24.0,
            },
        },
    }
    sel = select_hardware(user, root=Path("."))
    assert abs(sel["bus_max_voltage_V"] - 48.0) < 1e-9
    assert sel["bus_max_voltage_V"] < 50.4 - 1e-9
    assert abs(sel["bus_voltage_ceiling_V"] - 48.0) < 1e-9


def test_user_config_power_system_clamped():
    cfg = load_user_input_config(Path("config/user_inputs.json"))
    batt = cfg["power_system"]["battery"]
    assert float(batt["maximum_charge_cell_voltage_V"]) <= 4.0 + 1e-12
    assert max(float(v) for _, v in batt["ocv_curve_cell_V"]) <= 4.0 + 1e-12
    assert float(cfg["power_system"]["aircraft_bus_voltage_ceiling_V"]) == 48.0
    if cfg.get("hardware_selection_result"):
        assert float(cfg["hardware_selection_result"]["bus_max_voltage_V"]) <= 48.0 + 1e-9
