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
    assert c['model']['version']=='H2AIR-integrated-v4.1'
    assert c['design']['surface_geometry_mode']=='root_taper_span'
    assert c['design']['main_wing_naca_code']=='NACA6415'
    assert abs(float(c['design']['wing_CLmax'])-1.37)<1e-9
    assert float(c['design']['mtow_ceiling_kg'])==15.0
    assert c['design']['wing_area_m2'] > 0
    assert c['design']['horizontal_tail_area_m2'] > 0
    assert c['design']['vertical_tail_area_total_m2'] > 0
    assert float(c['design']['wing_span_m']) > 0



def test_inspection_flight_envelope_user_inputs_resolve():
    c=load_model_config(ROOT/'config'/'user_inputs.json')
    d=c['design']
    assert d['operational_positive_manoeuvre_load_factor_g']==1.5
    assert d['operational_negative_manoeuvre_load_factor_g']==-0.5
    assert d['operational_maximum_bank_deg']==45.0
    assert d['maximum_manoeuvre_clmax_utilization']==0.90
    assert d['maximum_gust_clmax_utilization']==0.95
    assert c['structures']['load_cases']['maneuver_load_factor']==3.0
    gusts=c['sixdof_control']['simulation']['inspection_gust_cases_ned_mps']
    assert gusts['lateral']==[0.0,5.0,0.0]
    assert gusts['vertical_up']==[0.0,0.0,-2.0]
    assert gusts['vertical_down']==[0.0,0.0,2.0]


def test_default_primary_structure_is_solver_owned():
    c=load_model_config(ROOT/'config'/'user_inputs.json')
    st=c['structures']['structure']
    assert st['recommended_sections_mode']=='auto'
    ps=c['manufacturing_mass_model']['main_wing']['primary_spar']
    assert not ps.get('stepped', False)
    assert ps.get('design_tube_od_m') is None
    assert c['manufacturing_mass_model']['boom_tubes'].get('design_tube_od_m') is None

def test_hardcoded_span_is_fixed_and_removed_from_optimizer(tmp_path):
    u=json.loads((ROOT/'config'/'user_inputs.json').read_text())
    u['lifting_surfaces']['main_wing']['AutoSize Main Wing']=False
    u['lifting_surfaces']['main_wing']['span_mode']='hardcoded'
    u['lifting_surfaces']['main_wing']['hardcoded_span_m']=3.333
    u['lifting_surfaces']['main_wing']['root_chord_m']=0.307
    u['lifting_surfaces']['main_wing']['taper_ratio']=0.59
    c=load_model_config(_write(tmp_path,u))
    assert c['design']['auto_size_main_wing'] is False
    assert abs(c['design']['wing_span_m']-3.333)<1e-12
    assert abs(c['design']['main_wing_root_chord_m']-0.307)<1e-12
    assert abs(c['design']['main_wing_taper_ratio']-0.59)<1e-12
    assert 'main_wing_span_m' not in default_space_for_config(c)


def test_auto_size_main_wing_trigger_and_bounds():
    c=load_model_config(ROOT/'config'/'user_inputs.json')
    d=c['design']
    assert d['auto_size_main_wing'] is True
    assert d['main_wing_span_mode']=='auto'
    assert d['main_wing_span_bounds_m']==[3.15,3.7]
    assert d['main_wing_root_chord_bounds_m']==[0.27,0.35]
    assert d['main_wing_taper_ratio_bounds']==[0.45,0.75]
    assert 'main_wing_span_m' not in default_space_for_config(c)


def test_linear_mass_uses_kg_per_m():
    c=load_model_config(ROOT/'config'/'user_inputs.json')
    comps=_components_with_power(c);by={x.name:x for x in comps}
    mm=c['manufacturing_mass_model']
    L=c['design']['fuselage_length_m']
    # Growth margins (1.05 for engineering_assumption) apply after manufacturing mass.
    expect=float(mm['fuselage_longerons']['specific_mass_kg_per_m'])*L*2*1.05
    mlong=by['fuselage_longeron_top'].mass_takeoff_kg+by['fuselage_longeron_bottom'].mass_takeoff_kg
    assert abs(mlong-expect)<1e-10
    b=c['design']['wing_span_m']
    expect_shell=float(mm['main_wing']['shell']['specific_mass_kg_per_m'])*b*1.05
    assert abs(by['main_wing_shell'].mass_takeoff_kg-expect_shell)<1e-10


def test_hardcoded_primary_spar_mass_survives_structural_closure(tmp_path):
    u=json.loads((ROOT/'config'/'user_inputs.json').read_text())
    # Force non-stepped single spar so the hardcoded aggregate component exists.
    u['carbon_fiber_tubes']['main_wing_primary_spar']={'od_mm':24,'id_mm':18}
    r=u['manufacturing_mass_model']['main_wing']['primary_spar']
    r['mass_mode']='hardcoded';r['hardcoded_mass_kg']=0.321
    r.pop('stepped', None)
    c=load_model_config(_write(tmp_path,u))
    base=_components_with_power(c)
    closed,_,_,_=_structural_closure(c,base,max_iterations=3)
    by={x.name:x for x in closed}
    # Growth margin (1.05) applies to engineering_assumption/calculated; hardcoded status may still grow.
    assert 'main_wing_primary_spar' in by
    assert by['main_wing_primary_spar'].mass_takeoff_kg > 0.3


def test_fixed_fuselage_does_not_autogrow_to_fit(tmp_path):
    import pytest
    u=json.loads((ROOT/'config'/'user_inputs.json').read_text())
    # Explicitly exercise HARD-CODED fuselage mode.  When the requested body
    # is physically too small, resolution must fail rather than silently grow
    # the fuselage.
    u['fuselage']['diameter_mode']='hardcoded'
    u['fuselage']['midbody_length_mode']='hardcoded'
    for sec in u['fuselage']['sections']:
        sec['diameter_m']=0.12
    with pytest.raises(ValueError, match='Packaging layout'):
        load_model_config(_write(tmp_path,u))
    assert all(abs(sec['diameter_m']-0.12)<1e-12 for sec in u['fuselage']['sections'])


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
    assert out['lifting_surfaces']['main_wing']['AutoSize Main Wing'] is False
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
    assert (tmp_path/'run'/'aircraft_specification.json').exists()
    assert (tmp_path/'run'/'resolved_configuration.json').exists()
    spec = json.loads((tmp_path/'run'/'aircraft_specification.json').read_text(encoding='utf-8'))
    assert spec['schema'] == 'H2AIR-aircraft-specification-v1'
    assert spec['run']['design_status'] == r['summary']['design_status']
    assert abs(spec['mass_balance']['closed_mtow_kg'] - r['summary']['post_structure_mass_kg']) < 1e-9
    assert abs(spec['geometry']['main_wing']['span_m'] - r['main_wing_autosizing']['selected']['values']['main_wing_span_m']) < 1e-6
    assert 'components_takeoff_kg' in spec['mass_balance']
    assert 'gates' in spec
    assert (tmp_path/'run'/'user_inputs.json').exists()
    assert (tmp_path/'run'/'details'/'AUTO_CALCULATED_VALUES.json').exists()
    assert (tmp_path/'run'/'details'/'manufacturing_mass_ledger.csv').exists()
    assert (tmp_path/'run'/'details'/'cad_parameters.xml').exists()
    assert (tmp_path/'run'/'details'/'params.xml').exists()
    assert (tmp_path/'run'/'details'/'params.xlsx').exists()
    assert (tmp_path/'run'/'details'/'HARDWARE_SELECTION.json').exists()
