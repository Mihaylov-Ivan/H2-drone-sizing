from __future__ import annotations
import json
from pathlib import Path

from h2air.config import load_model_config
from h2air.optimization import default_space_for_config
from h2air.pipeline import _components_with_power, _mass_result, _structural_closure, run_model

ROOT=Path(__file__).resolve().parents[1]


def _write(tmp_path, obj):
    p=tmp_path/'user_inputs.json';p.write_text(json.dumps(obj,indent=2)+'\n');return p


def test_v3_user_file_resolves_auto_geometry():
    c=load_model_config(ROOT/'config'/'user_inputs.json')
    assert c['model']['version']=='H2AIR-integrated-v4.0'
    assert c['design']['surface_geometry_mode']=='root_taper_span'
    assert c['design']['main_wing_naca_code']=='NACA4412'
    assert 3.15 <= c['design']['wing_span_m'] <= 3.60
    assert c['design']['wing_area_m2'] > 0
    assert c['design']['horizontal_tail_area_m2'] > 0
    assert c['design']['vertical_tail_area_total_m2'] > 0


def test_hardcoded_span_is_fixed_and_removed_from_optimizer(tmp_path):
    u=json.loads((ROOT/'config'/'user_inputs.json').read_text())
    u['lifting_surfaces']['main_wing']['span_mode']='hardcoded'
    u['lifting_surfaces']['main_wing']['hardcoded_span_m']=3.333
    c=load_model_config(_write(tmp_path,u))
    assert abs(c['design']['wing_span_m']-3.333)<1e-12
    assert 'main_wing_span_m' not in default_space_for_config(c)


def test_linear_mass_uses_kg_per_m():
    c=load_model_config(ROOT/'config'/'user_inputs.json')
    comps=_components_with_power(c);by={x.name:x for x in comps}
    mm=c['manufacturing_mass_model']
    L=c['design']['fuselage_length_m']
    expect=float(mm['fuselage_longerons']['specific_mass_kg_per_m'])*L*2
    assert abs(by['fuselage_longerons'].mass_takeoff_kg-expect)<1e-10
    b=c['design']['wing_span_m']
    expect_shell=float(mm['main_wing']['shell']['specific_mass_kg_per_m'])*b
    assert abs(by['main_wing_shell'].mass_takeoff_kg-expect_shell)<1e-10


def test_hardcoded_primary_spar_mass_survives_structural_closure(tmp_path):
    u=json.loads((ROOT/'config'/'user_inputs.json').read_text())
    r=u['manufacturing_mass_model']['main_wing']['primary_spar']
    r['mass_mode']='hardcoded';r['hardcoded_mass_kg']=0.321
    c=load_model_config(_write(tmp_path,u))
    base=_components_with_power(c)
    closed,_,_,_=_structural_closure(c,base,max_iterations=3)
    by={x.name:x for x in closed}
    assert abs(by['main_wing_primary_spar'].mass_takeoff_kg-0.321)<1e-10


def test_fixed_fuselage_does_not_autogrow_to_fit(tmp_path):
    u=json.loads((ROOT/'config'/'user_inputs.json').read_text())
    for sec in u['fuselage']['sections']:
        sec['diameter_m']=0.12
    c=load_model_config(_write(tmp_path,u))
    m=_mass_result(c,_components_with_power(c))
    assert m.summary['packaging_fail_count'] > 0
    assert abs(c['design']['fuselage_max_diameter_m']-0.12)<1e-12


def test_user_inputs_from_design_values_locks_spans(tmp_path):
    from h2air.user_inputs import user_inputs_from_design_values
    base=json.loads((ROOT/'config'/'user_inputs.json').read_text())
    out=user_inputs_from_design_values(base,{
        'wing_ac_x_m':0.401,'main_wing_span_m':3.37,
        'horizontal_tail_ac_x_m':1.04,'horizontal_tail_span_m':0.87,
        'vertical_tail_ac_x_m':1.02,'vertical_tail_height_m':0.3,
        'lift_row_center_x_m':0.43,'battery_x_m':0.38,'fuel_cell_x_m':0.15,
        'hydrogen_cylinder_x_m':0.53,
    })
    assert out['lifting_surfaces']['main_wing']['span_mode']=='hardcoded'
    assert abs(out['lifting_surfaces']['main_wing']['hardcoded_span_m']-3.37)<1e-9
    assert abs(out['lifting_surfaces']['main_wing']['aerodynamic_center_x_m']-0.401)<1e-9
    assert abs(out['component_positions']['battery_x_m']-0.38)<1e-9
    path=tmp_path/'u.json'
    path.write_text(json.dumps(out))
    c=load_model_config(path)
    assert abs(c['design']['wing_span_m']-3.37)<1e-9
    assert abs(c['design']['wing_ac_x_m']-0.401)<1e-9


def test_v4_default_user_design_full_pipeline_runs(tmp_path):
    c=load_model_config(ROOT/'config'/'user_inputs.json')
    r=run_model(c,tmp_path/'run',write_histories=False)
    assert r['coupled_closure']['converged']
    assert (tmp_path/'run'/'SUMMARY.md').exists()
    assert (tmp_path/'run'/'resolved_configuration.json').exists()
    assert (tmp_path/'run'/'user_inputs.json').exists()
    assert (tmp_path/'run'/'details'/'AUTO_CALCULATED_VALUES.json').exists()
    assert (tmp_path/'run'/'details'/'manufacturing_mass_ledger.csv').exists()
    assert (tmp_path/'run'/'details'/'cad_parameters.xml').exists()
    assert (tmp_path/'run'/'details'/'params.xml').exists()
    assert (tmp_path/'run'/'details'/'params.xlsx').exists()
    assert (tmp_path/'run'/'details'/'HARDWARE_SELECTION.json').exists()
