from pathlib import Path
import json
from h2air.config import load_model_config
from h2air.airfoils import interpolate_polar
from h2air.hardware_selection import select_hardware
from h2air.pipeline import run_model
ROOT=Path(__file__).resolve().parents[1]

def test_naca0012_user_polar_is_used():
    c=load_model_config(ROOT/'config/user_inputs.json')
    af=c['airfoil_model']['horizontal_tail']
    assert af['code']=='NACA0012'
    assert af['aero_source_mode']=='polar_file'
    r=interpolate_polar(af['polar_rows'],0.0)
    assert abs(r['CL'])<1e-12
    assert abs(r['CD']-0.01020)<1e-8
    assert abs(af['polar_summary']['CL_max']-1.1101)<1e-6

def test_auto_hardware_selection_returns_12s_compatible_set():
    u=json.loads((ROOT/'config/user_inputs.json').read_text())
    s=select_hardware(u,root=ROOT)
    assert s['mode']=='auto'
    assert s['selected_lift_combo']['voltage_max_V']>=50.4-1e-9
    assert s['selected_pusher_combo']['voltage_max_V']>=50.4-1e-9
    assert s['selected_battery']['series_cells']==12
    assert s['selected_lift_combo']['max_thrust_N']>=s['requirements']['lift_thrust_per_motor_N']

def test_hardcoded_hardware_mode_preserves_requested_combo(tmp_path):
    u=json.loads((ROOT/'config/user_inputs.json').read_text())
    u['hardware_selection']['mode']='hardcoded'
    p=tmp_path/'u.json';p.write_text(json.dumps(u))
    c=load_model_config(p)
    sel=c['hardware_selection_result']
    h=u['hardware_selection']['hardcoded']
    assert sel['selected_lift_combo']['name']==h['lift_combo_name']
    assert sel['selected_pusher_combo']['name']==h['pusher_combo_name']
    assert sel['selected_battery']['name']==h['battery_name']

def test_cad_export_is_geometry_only(integrated_result):
    _,out=integrated_result
    d=json.loads((out/'details'/'cad_parameters.json').read_text())
    names={x['name'] for x in d['parameters']}
    for needed in ('wing_span','wing_z','wing_thickness','horizontal_tail_z','vertical_tail_thickness','wing_primary_spar_OD','boom_OD','boom_spacing','lift_motor_front_x','pusher_prop_diameter','general_print_tolerance','bolt_M4_clearance_hole_OD'):
        assert needed in names
    forbidden=('mass','weight','power','stress','mpa','energy','thrust','soc','current')
    assert not [n for n in names if any(x in n.lower() for x in forbidden)]
    assert (out/'details'/'params.xml').exists()
    assert (out/'details'/'params.xlsx').exists()
