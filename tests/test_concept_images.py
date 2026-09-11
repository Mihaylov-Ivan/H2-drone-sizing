"""Offline tests for H2AIR concept-image prompt builders."""
from __future__ import annotations

import pytest

from h2air.concept_images import (
    VIEW_STEMS,
    build_aircraft_brief,
    build_view_prompts,
    generate_concept_images,
)


def _minimal_cfg_result():
    cfg = {
        "design": {
            "wing_span_m": 3.37,
            "wing_area_m2": 0.879,
            "wing_mac_m": 0.266,
            "main_wing_root_chord_m": 0.326,
            "main_wing_taper_ratio": 0.60,
            "main_wing_naca_code": "NACA4412",
            "main_wing_sweep_deg": 0.0,
            "main_wing_dihedral_deg": 0.0,
            "fuselage_length_m": 0.86,
            "fuselage_width_m": 0.221,
            "fuselage_height_m": 0.221,
            "fuselage_max_diameter_m": 0.221,
            "fuselage_cross_section": "rectangular_filleted",
            "fuselage_fillet_radius_m": 0.02,
            "horizontal_tail_span_m": 0.87,
            "vertical_tail_height_m": 0.30,
            "cruise_speed_mps": 20.0,
            "payload_mass_kg": 1.2,
            "energy_bay_order": "fuel_cell_forward",
            "fuel_cell_x_m": 0.255,
            "fuel_cell_z_m": -0.031,
            "battery_x_m": 0.222,
            "battery_z_m": 0.064,
            "hydrogen_cylinder_x_m": 0.593,
            "hydrogen_cylinder_z_m": -0.034,
            "payload_x_m": 0.104,
            "cruise_lift_propeller_state": "folded",
            "lift_propeller_diameter_in": 22.0,
        },
        "geometry_mass": {
            "geometry_model": {
                "geometry": {
                    "boom_center_y_m": [-0.374, 0.374],
                    "boom_x_start_m": 0.096,
                    "boom_x_end_m": 1.100,
                }
            }
        },
        "aero_propulsion": {"lift_layout": {"y_abs_m": 0.374, "z_m": 0.12}},
        "manufacturing_mass_model": {
            "main_wing": {"z_m": 0.089},
            "horizontal_tail": {"z_m": 0.089},
        },
        "sixdof_control": {"aircraft": {"pusher_station_m": [0.896, 0.0, 0.0]}},
    }
    result = {
        "summary": {"post_structure_mass_kg": 14.3},
        "aero_propulsion": {
            "layout": {
                "prop_diameter_in": 22.0,
                "proposed_front_x_m": 0.096,
                "proposed_rear_x_m": 0.675,
            }
        },
        "fuel_cell_cooling": {
            "architecture": "cross_fuselage",
            "fan_flow_direction": "suction_through_stack_to_fan_side",
            "inlet_side": "left",
            "exhaust_side": "right",
            "waste_heat_design_W": 880.0,
            "required_airflow_cfm": 101.0,
        },
    }
    return cfg, result


def test_brief_includes_key_dimensions():
    cfg, result = _minimal_cfg_result()
    brief = build_aircraft_brief(cfg, result)
    assert "3.370" in brief or "3.37" in brief
    assert "0.860" in brief or "0.86" in brief
    assert "twin-boom" in brief.lower() or "Twin-boom" in brief
    assert "left" in brief and "right" in brief
    assert "NACA-style" in brief or "naca" in brief.lower()
    assert "fuel cell" in brief.lower()
    assert "H2 cylinder" in brief or "hydrogen" in brief.lower()


def test_view_prompts_cover_all_stems():
    cfg, result = _minimal_cfg_result()
    brief = build_aircraft_brief(cfg, result)
    prompts = build_view_prompts(brief)
    assert set(prompts.keys()) == set(VIEW_STEMS)
    for stem, text in prompts.items():
        assert brief in text
        assert "hydrogen-hybrid" in text.lower() or "Hydrogen-hybrid" in brief
        assert len(text) > 200
    assert "isometric" in prompts["isometric_clean"].lower()
    assert "orthographic" in prompts["top_concept"].lower()
    assert "orthographic" in prompts["side_concept"].lower()
    assert "perspective" in prompts["perspective_concept"].lower()
    assert "cutaway" in prompts["cutaway_internal"].lower()
    assert "airflow" in prompts["cooling_airflow"].lower()
    assert "left" in prompts["cooling_airflow"] and "right" in prompts["cooling_airflow"]


def test_generate_requires_api_key(tmp_path, monkeypatch):
    cfg, result = _minimal_cfg_result()
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)

    class _FakeOpenAI:
        def __init__(self, *args, **kwargs):
            raise AssertionError("should not construct client without a key")

    import h2air.concept_images as mod

    monkeypatch.setattr(mod, "_require_openai", lambda: _FakeOpenAI)
    with pytest.raises(RuntimeError, match="OPENAI_API_KEY"):
        generate_concept_images(cfg, result, tmp_path, api_key=None)


def test_generate_rejects_unknown_view(tmp_path, monkeypatch):
    cfg, result = _minimal_cfg_result()
    with pytest.raises(ValueError, match="Unknown concept view"):
        generate_concept_images(cfg, result, tmp_path, api_key="sk-test", views=["not_a_view"])
