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
    nose = float(pl["nose_section_end_x_m"])
    order = str(d.get("energy_bay_order", pl.get("energy_bay_order", "fuel_cell_forward")))

    pay = next(x for x in c["geometry_mass"]["component_ledger"]["components"] if x["name"] == "payload")
    pay_L = float(pay["dimensions_m"]["length_x"])
    assert abs((d["payload_x_m"] + 0.5 * pay_L) - nose) < 1e-9

    cube = next(x for x in c["geometry_mass"]["component_ledger"]["components"] if x["name"] == "autopilot")
    cube_L = float(cube["dimensions_m"]["length_x"])
    assert abs((d["cube_orange_x_m"] + 0.5 * cube_L) - nose) < 1e-9
    assert d["cube_orange_z_m"] > d["payload_z_m"]

    fc = next(x for x in c["geometry_mass"]["component_ledger"]["components"] if x["name"] == "fuel_cell_system")
    fc_L = float(fc["dimensions_m"]["length_x"])
    cyl = next(x for x in c["geometry_mass"]["component_ledger"]["components"] if x["name"] == "hydrogen_cylinder")
    L_cyl = float((cyl.get("dimensions_m") or {}).get("length") or 0.0)

    if order == "fuel_cell_forward":
        assert abs((d["fuel_cell_x_m"] - 0.5 * fc_L) - nose) < 1e-9
        assert d["hydrogen_cylinder_x_m"] > d["fuel_cell_x_m"]
    else:
        assert abs((d["hydrogen_cylinder_x_m"] - 0.5 * L_cyl) - nose) < 1e-9
        assert d["fuel_cell_x_m"] > d["hydrogen_cylinder_x_m"]

    assert abs(float(d["fuel_cell_stack_mass_kg"]) + float(d["fuel_cell_control_box_mass_kg"]) - 1.45) < 1e-6
    assert pl["control_box_site"] in {"fc_top", "fc_side", "h2_top"}
    assert pl["battery_site"] in {"fc_top", "fc_side", "h2_top"}
    assert d.get("pdb_x_m") is not None and d.get("pdb2_x_m") is not None
    assert "power_module" in {x["name"] for x in c["geometry_mass"]["component_ledger"]["components"]}
    assert "power_module_2" in {x["name"] for x in c["geometry_mass"]["component_ledger"]["components"]}

    if str(d.get("fuselage_diameter_mode", "")).lower() == "auto":
        sec_ods = d.get("fuselage_section_diameters_m") or {}
        assert sec_ods
        assert float(d["fuselage_max_diameter_m"]) == pytest.approx(max(sec_ods.values()), abs=1e-9)


def test_packaging_hydrogen_forward_order(tmp_path):
    u = json.loads((ROOT / "config" / "user_inputs.json").read_text(encoding="utf-8"))
    u["packaging_layout"]["energy_bay_order"] = "hydrogen_forward"
    u["packaging_layout"]["energy_bay_order_mode"] = "hardcoded"
    p = tmp_path / "user_inputs.json"
    p.write_text(json.dumps(u, indent=2) + "\n", encoding="utf-8")
    c = load_model_config(p)
    d = c["design"]
    nose = float(d["nose_section_end_x_m"])
    cyl = next(x for x in c["geometry_mass"]["component_ledger"]["components"] if x["name"] == "hydrogen_cylinder")
    L_cyl = float(cyl["dimensions_m"]["length"])
    assert d["energy_bay_order"] == "hydrogen_forward"
    assert abs((d["hydrogen_cylinder_x_m"] - 0.5 * L_cyl) - nose) < 1e-9
    assert d["fuel_cell_x_m"] > d["hydrogen_cylinder_x_m"]


def test_energy_bay_order_in_optimizer_space_when_auto():
    from h2air.optimization import default_space_for_config

    c = load_model_config(ROOT / "config" / "user_inputs.json")
    if str(c["design"].get("energy_bay_order_mode", "")).lower() == "auto":
        space = default_space_for_config(c)
        assert "energy_bay_order_code" in space
        assert space["energy_bay_order_code"] == (0.0, 1.0)


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
