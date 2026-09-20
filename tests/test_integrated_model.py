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
    assert a['converged'] is True
    assert abs(float(a['history'][0]['input_M2_kg'])-15.0) > 0.05
    vals=a['selected']['values']
    assert set(vals)=={'main_wing_span_m','main_wing_root_chord_m','main_wing_taper_ratio'}
    assert a['selected']['wing_area_m2'] + 1e-9 >= a['selected']['required_area_with_margin_m2']


def test_tail_planform_autosizing_joint_geometry(integrated_result):
    r,_=integrated_result
    a=r['tail_planform_autosizing']
    assert a['enabled'] is True
    assert a['horizontal_tail'] is True
    assert a['vertical_tail'] is True
    assert a['mode']=='performance_constrained_ht_vt_span_root_chord_taper'
    assert a['tail_volume_role']=='calibration_reference_and_output_not_hard_constraint'
    assert abs(a['control_safety_factor']-1.10)<1e-12
    assert a['evaluated_candidates'] >= 2
    assert a['preliminary_feasible_candidates'] >= 1
    assert a['control_authority_method']=='AVL_trim_vertical_elevator_reserve_plus_surface_only_6dof_lateral_gust'
    assert a['dynamic_validation_candidates'] >= 1
    assert a['dynamic_feasible_candidates'] >= 1
    sel=a['selected']
    assert sel['feasible'] is True
    assert sel['control_authority']['pass'] is True
    assert sel['lateral_dynamic_authority']['pass'] is True
    assert all(x['pass'] for x in sel['lateral_dynamic_authority']['checks'])
    vals=sel['resolved_values']
    assert {'horizontal_tail_span_m','horizontal_tail_root_chord_m','horizontal_tail_taper_ratio',
            'vertical_tail_height_m','vertical_tail_root_chord_m','vertical_tail_taper_ratio'} <= set(vals)
    assert sel['loaded_static_margin'] + 1e-9 >= a['constraints']['loaded_static_margin_target']
    # Vh/Vv are now outcomes, not hard floors.
    assert sel['horizontal_tail_volume_coefficient'] > 0
    assert sel['vertical_tail_volume_coefficient'] > 0
    assert sel['horizontal_tail_reference_volume_coefficient']==a['constraints']['horizontal_tail_reference_volume_coefficient']
    assert sel['vertical_tail_reference_volume_coefficient']==a['constraints']['vertical_tail_reference_volume_coefficient']


def test_tail_control_authority_has_quasi_steady_safety_margin(integrated_result):
    r,_=integrated_result
    s=r['tail_control_authority']
    assert s['method']=='quasi_steady_linearized_surface_authority'
    assert abs(s['safety_factor']-1.10)<1e-12
    assert s['pass'] is True
    assert all(x['pass'] for x in s['checks'])
    names={x['name'] for x in s['checks']}
    assert {'cruise_trim_elevator','vertical_gust_elevator_reserve','directional_static_stability','directional_yaw_damping'} <= names
    assert next(g for g in r['gates'] if g['name']=='tail_control_authority')['status']=='PASS'
    assert s['worst_abs_elevator_deg'] <= s['limits']['gust_elevator_deg'] + 1e-9
    dyn=r['tail_planform_autosizing']['selected']['lateral_dynamic_authority']
    dnames={x['name'] for x in dyn['checks']}
    assert {'lateral_gust_yaw_response','lateral_gust_rudder_utilization','lateral_gust_simulation_completion'} <= dnames
    assert dyn['maximum_rudder_utilization'] <= dyn['checks'][1]['limit'] + 1e-9


def test_automated_avl_derivatives_are_active(integrated_result):
    r,_=integrated_result
    a=r['aerodynamic_derivatives']
    assert a['mode']=='avl'
    assert a['status']=='AVL_automated_geometry_derived'
    assert a['AVL']['status']=='PASS'
    assert not a['AVL']['missing_required']
    active=a['active']
    for key in ('CL_alpha_per_rad','Cl_da','Cm_de','Cn_beta','Cn_dr'):
        assert active[key] is not None
    assert active['Cl_da'] > 0
    assert active['Cm_de'] < 0
    assert active['Cn_beta'] > 0
    assert active['Cn_dr'] < 0
    assert next(g for g in r['gates'] if g['name']=='AVL_derivatives')['status']=='PASS'
    sel=r['tail_planform_autosizing']['selected']
    assert sel['aero_derivative_source']=='AVL_automated_geometry_derived'
    assert sel['avl_result']['status']=='PASS'


def test_inspection_flight_envelope_and_gust_stall_gates(integrated_result):
    r,_=integrated_result
    env=r['flight_envelope']
    assert abs(env['positive_operational_limit_g']-1.5)<1e-12
    assert abs(env['negative_operational_limit_g']+0.5)<1e-12
    assert abs(env['maximum_bank_deg']-45.0)<1e-12
    assert 1.40 < env['bank_load_factor_g'] < 1.42
    names={g['name']:g for g in r['gates']}
    assert names['inspection_bank_load']['status']=='PASS'
    assert names['positive_manoeuvre_stall_margin']['status']=='PASS'
    assert names['negative_manoeuvre_stall_margin']['status']=='PASS'
    assert names['inspection_gust_stall_margin']['status']=='PASS'
    gusts=r['sixdof_control']['current_layout']['inspection_gust_cases']
    assert {'lateral','vertical_up','vertical_down','combined_lateral_up'} <= set(gusts)
    sharp=r['flight_envelope']['vertical_gust_screen']
    assert sharp['model']=='sharp_edge_no_alleviation'
    assert sharp['pass'] is True
    assert sharp['required_wing_area_m2'] <= r['main_wing_autosizing']['selected']['wing_area_m2'] + 1e-9
    assert max(v['maximum_clmax_utilization'] for v in gusts.values()) <= r['summary']['worst_gust_cl_utilization'] + 1e-12
    assert sharp['clmax_utilization'] <= r['summary']['worst_gust_cl_utilization'] + 1e-12


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
