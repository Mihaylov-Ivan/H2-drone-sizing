from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def test_config_loads(model_config):
    assert model_config['design']['mtow_ceiling_kg']==15.0
    assert model_config['design']['neutral_point_model']=='calibrated_tail_volume'

def test_baseline_run_exposes_real_closure_failures(integrated_result):
    r,out=integrated_result
    assert 10.0<r['summary']['reference_mass_kg']<13.0
    assert r['summary']['post_structure_mass_kg']>r['summary']['reference_mass_kg']
    assert r['summary']['post_structure_mass_kg']<15.0
    assert r['coupled_closure']['converged'] is True
    assert r['summary']['current_static_margin']<0
    assert r['summary']['design_status']!='SCREENING_PASS'
    assert (out/'model_results.json').exists() and (out/'SUMMARY.md').exists()

def test_structural_closure_is_fixed_point(integrated_result):
    r,_=integrated_result
    h=r['coupled_closure']['history']
    assert h and h[-1]['mass_residual_kg']<=2e-4
    assert h[-1]['cg_residual_m']<=2e-5

def test_no_phase_entrypoints_in_root():assert not list(ROOT.glob('run_phase*.py'))
