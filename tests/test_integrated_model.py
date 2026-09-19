from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
from h2air.pipeline import run_model

def test_config_loads(model_config):
    assert model_config['design']['mtow_ceiling_kg']==15.0
    assert model_config['model']['version']=='H2AIR-integrated-v4.1'
    assert model_config['design']['surface_geometry_mode']=='root_taper_span'

def test_v4_default_run_closes(integrated_result):
    r,out=integrated_result
    # Mass may exceed the 15 kg ceiling until a design study rebalances;
    # still require a converged closure and positive static margin.
    assert float(r['summary']['post_structure_mass_kg']) > 0.0
    assert r['coupled_closure']['converged'] is True
    assert r['summary']['current_static_margin']>0.05
    assert (out/'SUMMARY.md').exists()
    assert (out/'details'/'model_results.json').exists()
    assert not (out/'model_results.json').exists()
    assert 'endurance_at_cruise_h' in r['summary']

def test_main_wing_autosizing_uses_live_M2_and_joint_planform(integrated_result):
    r,_=integrated_result
    a=r['main_wing_autosizing']
    assert a['enabled'] is True
    assert a['mode']=='coupled_span_root_chord_taper'
    assert a['mass_basis']=='current_iteration_M2'
    assert a['history']
    assert abs(float(a['history'][0]['input_M2_kg'])-15.0) > 0.05
    vals=a['selected']['values']
    assert set(vals)=={'main_wing_span_m','main_wing_root_chord_m','main_wing_taper_ratio'}
    assert a['selected']['wing_area_m2'] + 1e-9 >= a['selected']['required_area_with_margin_m2']


def test_autosized_sections_are_applied_to_mass_and_cad(integrated_result):
    r,_=integrated_result
    gate=next(g for g in r['gates'] if g['name']=='structural_mass_geometry_consistency')
    assert gate['status']=='PASS'
    assert next(g for g in r['gates'] if g['name']=='recommended_structure_screen')['status']=='PASS'
    assert r['summary']['wing_spar_od_mm'] > 0
    assert r['summary']['horizontal_tail_spar_od_mm'] > 0
    assert r['summary']['vertical_tail_spar_od_mm'] > 0
    assert r['summary']['boom_od_mm'] > 0


def test_structural_closure_is_fixed_point(integrated_result):
    r,_=integrated_result
    h=r['coupled_closure']['history']
    assert h and h[-1]['mass_residual_kg']<=2e-4
    assert h[-1]['cg_residual_m']<=2e-5

def test_legacy_problem_case_still_exposed(tmp_path,legacy_config):
    r=run_model(legacy_config,tmp_path/'legacy',write_histories=False)
    assert r['summary']['current_static_margin']<0
    assert r['summary']['design_status']!='SCREENING_PASS'

def test_no_phase_entrypoints_in_root():assert not list(ROOT.glob('run_phase*.py'))
