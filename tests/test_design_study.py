from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
from h2air.optimization import screening_evaluate, derive_margin_variant, select_recommended_candidate
from h2air.flight_envelope import build_flight_envelope
from h2air.calibration import fit_scale_factors

def _current_values(cfg):
    from h2air.optimization import default_space_for_config
    return {k:cfg['design'][k] for k in default_space_for_config(cfg)}

def test_selected_design_screening_evaluator_is_consistent(model_config):
    c=screening_evaluate(model_config,_current_values(model_config))
    # v4.1 intentionally exposes geometry/structure/cooling shortcomings in
    # the current user design rather than forcing a false feasible result.
    assert isinstance(c.feasible, bool)
    assert isinstance(c.violations, list)
    if not c.feasible:
        assert c.violations

def test_flight_envelope_contains_hover_and_fixed_wing(optimized_config):
    e=build_flight_envelope(optimized_config,v_min=15,v_max=25,dv=5)
    assert e['hover']['speed_mps']==0.0
    assert len(e['rows'])==3

def test_empty_calibration_does_not_invent_evidence():
    assert fit_scale_factors([])['status']=='NOT_EVALUATED'

def test_margin_variant_is_well_formed_and_targets_positive_stability(model_config):
    # Margin derivation must remain well formed even if the present design is
    # infeasible for an unrelated packaging/structural/cooling gate.
    c=screening_evaluate(model_config,_current_values(model_config))
    opt={'best':c.to_dict(),'ranked':[c.to_dict()]}
    r=derive_margin_variant(model_config,opt)
    b=r['best']
    assert 'feasible' in b and 'metrics' in b
    assert b['metrics']['static_margin'] >= 0.05

def test_recommended_selector_prefers_verified_more_robust_case():
    n={'design_status':'SCREENING_PASS'}
    r={'design_status':'SCREENING_PASS'}
    nu={'screening_pass_probability':2/3}
    ru={'screening_pass_probability':0.9375}
    assert select_recommended_candidate(n,r,nu,ru)=='robust_margin_biased'
    assert select_recommended_candidate(n,{'design_status':'FAIL'},nu,ru)=='nominal_mass_optimized'
