from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
from h2air.optimization import screening_evaluate
from h2air.flight_envelope import build_flight_envelope
from h2air.calibration import fit_scale_factors

def test_selected_design_is_screening_feasible(model_config,optimized_config):
    keys=('wing_area_m2','wing_span_m','wing_ac_x_m','horizontal_tail_area_m2','lift_row_center_x_m','battery_x_m','battery_z_m','fuel_cell_x_m','fuel_cell_z_m','hydrogen_cylinder_x_m','hydrogen_cylinder_z_m')
    vals={k:optimized_config['design'][k] for k in keys}
    c=screening_evaluate(model_config,vals)
    assert c.feasible

def test_flight_envelope_contains_hover_and_fixed_wing(optimized_config):
    e=build_flight_envelope(optimized_config,v_min=15,v_max=25,dv=5)
    assert e['hover']['speed_mps']==0.0
    assert len(e['rows'])==3

def test_empty_calibration_does_not_invent_evidence():
    assert fit_scale_factors([])['status']=='NOT_EVALUATED'

def test_margin_variant_is_feasible_and_adds_stall_margin(model_config):
    import json
    from h2air.optimization import derive_margin_variant
    opt=json.loads((ROOT/'outputs'/'optimization'/'optimization_results.json').read_text())
    r=derive_margin_variant(model_config,opt)
    b=r['best']
    assert b['feasible']
    assert b['values']['wing_area_m2'] >= 1.10*b['metrics']['wing_required_15kg_m2']
    assert b['metrics']['static_margin'] >= 0.05

def test_recommended_selector_prefers_verified_more_robust_case():
    from h2air.optimization import select_recommended_candidate
    n={'design_status':'SCREENING_PASS'}
    r={'design_status':'SCREENING_PASS'}
    nu={'screening_pass_probability':2/3}
    ru={'screening_pass_probability':0.9375}
    assert select_recommended_candidate(n,r,nu,ru)=='robust_margin_biased'
    assert select_recommended_candidate(n,{'design_status':'FAIL'},nu,ru)=='nominal_mass_optimized'
