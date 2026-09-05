from __future__ import annotations

import json
from pathlib import Path

import pytest

from h2air.config import load_model_config
from h2air.design_coupling import resolve_design_couplings
from h2air.user_inputs import load_user_input_config

ROOT = Path(__file__).resolve().parents[1]


def test_packaging_layout_resolves_on_current_user_inputs():
    c = load_model_config(ROOT / "config" / "user_inputs.json")
    d = c["design"]
    pl = c["coupling"]["packaging_layout"]
    assert pl["enabled"] is True
    assert d["hydrogen_cylinder_x_m"] > d["fuel_cell_x_m"]
    assert d["fuel_cell_control_box_x_m"] > d["fuel_cell_x_m"]
    assert d["fuel_cell_control_box_z_m"] > d["hydrogen_cylinder_z_m"]
    assert d["battery_z_m"] > d["fuel_cell_z_m"]
    assert d["payload_z_m"] > d["fuel_cell_z_m"]
    assert abs(float(d["fuel_cell_stack_mass_kg"]) + float(d["fuel_cell_control_box_mass_kg"]) - 1.45) < 1e-6
    nose = float(pl["nose_section_end_x_m"])
    pay = next(
        x for x in c["geometry_mass"]["component_ledger"]["components"] if x["name"] == "payload"
    )
    pay_L = float(pay["dimensions_m"]["length_x"])
    assert abs((d["payload_x_m"] + 0.5 * pay_L) - nose) < 1e-9
    if str(d.get("fuselage_diameter_mode", "")).lower() == "auto":
        sec_ods = d.get("fuselage_section_diameters_m") or {}
        assert sec_ods
        assert float(d["fuselage_max_diameter_m"]) == pytest.approx(max(sec_ods.values()), abs=1e-9)
        assert float(d["fuselage_max_diameter_m"]) >= float(d.get("fuselage_minimum_diameter_m", 0.12)) - 1e-9


def test_packaging_layout_errors_when_pod_too_small(tmp_path):
    u = json.loads((ROOT / "config" / "user_inputs.json").read_text(encoding="utf-8"))
    u["fuselage"]["diameter_mode"] = "hardcoded"
    for sec in u["fuselage"]["sections"]:
        sec["diameter_m"] = 0.12
    p = tmp_path / "user_inputs.json"
    p.write_text(json.dumps(u, indent=2) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="Packaging layout"):
        load_model_config(p)


def test_fcpm_part_masses_must_sum_to_module(tmp_path):
    raw = load_user_input_config(ROOT / "config" / "user_inputs.json")
    raw["packaging_layout"]["hardware"]["fuel_cell_stack"]["mass_kg"] = 1.0
    raw["packaging_layout"]["hardware"]["fuel_cell_control_box"]["mass_kg"] = 0.2
    with pytest.raises(ValueError, match="must equal module_mass_kg"):
        resolve_design_couplings(raw)
