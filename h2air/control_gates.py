from __future__ import annotations
from math import sqrt
from typing import Any, Dict, Mapping, List
G=9.80665

def evaluate_case(cfg:Mapping[str,Any], case:Mapping[str,Any], *, label:str)->Dict[str,Any]:
    g=cfg['gates']; a=cfg['aircraft']; ad=cfg['aerodynamics']; f=case['forward_transition']; r=case['reverse_transition']; gust=case['controlled_gust']; trim=case['trim']
    vs=sqrt(2*float(a['mass_kg'])*G/(float(ad['rho_kg_m3'])*float(a['wing_area_m2'])*float(ad['CL_max'])))
    stall_ratio=float(f['final_speed_mps'])/vs
    checks=[]
    def add(name,ok,value,limit,note=''): checks.append({'name':name,'status':'PASS' if ok else 'FAIL','value':value,'limit':limit,'note':note})
    add('static_margin',float(case['static_margin'])>=float(g['minimum_static_margin']),case['static_margin'],g['minimum_static_margin'])
    add('cruise_trim_solver',bool(trim['success']),trim['residual_norm'],'trim residual near zero')
    add('cruise_trim_elevator',abs(float(trim['elevator_deg']))<=float(g['maximum_abs_elevator_trim_deg']),trim['elevator_deg'],f"±{g['maximum_abs_elevator_trim_deg']} deg")
    add('forward_handover_stall_ratio',stall_ratio>=1.30,stall_ratio,1.30)
    add('forward_pitch_tracking',float(f['maximum_abs_pitch_tracking_error_deg'])<=float(g['maximum_transition_pitch_tracking_error_deg']),f['maximum_abs_pitch_tracking_error_deg'],g['maximum_transition_pitch_tracking_error_deg'])
    add('forward_roll_control',float(f['maximum_abs_roll_deg'])<=float(g['maximum_transition_roll_deg']),f['maximum_abs_roll_deg'],g['maximum_transition_roll_deg'])
    add('forward_altitude_loss',float(f['altitude_loss_m'])<=float(g['maximum_transition_altitude_loss_m']),f['altitude_loss_m'],g['maximum_transition_altitude_loss_m'])
    add('forward_surface_headroom',float(f['maximum_surface_utilization'])<=float(g['maximum_surface_utilization_fraction']),f['maximum_surface_utilization'],g['maximum_surface_utilization_fraction'])
    add('forward_rotor_headroom',float(f['maximum_rotor_utilization'])<=float(g['maximum_rotor_utilization_fraction']),f['maximum_rotor_utilization'],g['maximum_rotor_utilization_fraction'])
    add('reverse_hover_capture_speed',float(r['final_speed_mps'])<=3.0,r['final_speed_mps'],3.0)
    add('reverse_pitch_tracking',float(r['maximum_abs_pitch_tracking_error_deg'])<=float(g['maximum_transition_pitch_tracking_error_deg']),r['maximum_abs_pitch_tracking_error_deg'],g['maximum_transition_pitch_tracking_error_deg'])
    add('reverse_altitude_loss',float(r['altitude_loss_m'])<=float(g['maximum_transition_altitude_loss_m']),r['altitude_loss_m'],g['maximum_transition_altitude_loss_m'])
    add('gust_roll',float(gust['maximum_abs_roll_deg'])<=float(g['maximum_cruise_gust_roll_deg']),gust['maximum_abs_roll_deg'],g['maximum_cruise_gust_roll_deg'])
    add('gust_yaw',float(gust['maximum_abs_yaw_deg'])<=float(g['maximum_cruise_gust_yaw_deg']),gust['maximum_abs_yaw_deg'],g['maximum_cruise_gust_yaw_deg'])
    add('gust_altitude_loss',float(gust['altitude_loss_m'])<=float(g['maximum_cruise_gust_altitude_loss_m']),gust['altitude_loss_m'],g['maximum_cruise_gust_altitude_loss_m'])
    return {'label':label,'stall_speed_mps':vs,'forward_final_stall_ratio':stall_ratio,'checks':checks,'pass':all(x['status']=='PASS' for x in checks)}
