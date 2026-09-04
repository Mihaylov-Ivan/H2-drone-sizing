from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
from h2air.pipeline import run_model

def test_config_loads(model_config):
    assert model_config['design']['mtow_ceiling_kg']==15.0
    assert model_config['model']['version']=='H2AIR-integrated-v4.0'
    assert model_config['design']['surface_geometry_mode']=='root_taper_span'

def test_v4_default_run_closes(integrated_result):
    r,out=integrated_result
    assert r['summary']['post_structure_mass_kg']<15.0
    assert r['coupled_closure']['converged'] is True
    assert r['summary']['current_static_margin']>0.05
    assert (out/'model_results.json').exists() and (out/'SUMMARY.md').exists()

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
