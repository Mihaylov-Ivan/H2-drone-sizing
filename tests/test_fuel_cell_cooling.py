from pathlib import Path
import json

from h2air.config import load_model_config
from h2air.fuel_cell_cooling import analyze_fuel_cell_cooling
from h2air.pipeline import run_model

ROOT=Path(__file__).resolve().parents[1]


def test_cooling_screen_uses_cross_fuselage_suction_architecture():
    c=load_model_config(ROOT/'config'/'user_inputs.json')
    r=analyze_fuel_cell_cooling(c)
    assert r.enabled
    assert r.fan_flow_direction=='suction_through_stack_to_fan_side'
    assert r.inlet_side != r.exhaust_side
    assert r.required_airflow_cfm > 0
    assert r.estimated_operating_airflow_cfm > 0
    assert r.cooling_openings_CdA_m2 > 0


def test_cooling_geometry_detects_current_side_plenum_shortfall():
    c=load_model_config(ROOT/'config'/'user_inputs.json')
    r=analyze_fuel_cell_cooling(c)
    # The present 135 mm-wide pod has only ~3.3 mm inner side gaps around the
    # 108.4 mm FC.  Cooling integration should expose, not hide, this issue.
    assert not r.plenum_geometry_pass
    assert r.minimum_fuselage_width_for_plenums_m > r.fuselage_width_m


def test_cooling_cad_parameters_and_gate_written(tmp_path):
    c=load_model_config(ROOT/'config'/'user_inputs.json')
    r=run_model(c,tmp_path/'run',write_histories=False)
    details=tmp_path/'run'/'details'
    cool=json.loads((details/'FUEL_CELL_COOLING.json').read_text())
    cad=json.loads((details/'cad_parameters.json').read_text())
    names={p['name'] for p in cad['parameters']}
    assert cool['airflow_screen_pass']
    assert 'fc_cooling_inlet_ramp_length' in names
    assert 'fc_cooling_inlet_slot_gap' in names
    assert 'fc_cooling_exhaust_shroud_depth' in names
    assert 'fc_cooling_min_fuselage_width' in names
    gates={g['name']:g['status'] for g in r['gates']}
    assert gates['fuel_cell_cooling_airflow_screen']=='PASS'
    assert gates['fuel_cell_cooling_geometry']=='FAIL'
    assert gates['fuel_cell_cooling_fan_curve']=='NOT_EVALUATED'
