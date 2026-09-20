from __future__ import annotations

"""Clean CAD parameter export for H2AIR.

Only geometry, component envelopes/locations, clearances/tolerances and selected
fastener sizes are exported. Masses, powers, stresses and performance metrics
are intentionally excluded so the XML/XLSX remain useful as a drawing/CAD driver.
"""

from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple
import json, math
import xml.etree.ElementTree as ET

try:
    from openpyxl import Workbook
except ImportError:  # pragma: no cover
    Workbook = None  # type: ignore[misc, assignment]


def _mm(x: float) -> float: return 1000.0*float(x)

def _tip(root: float, taper: float) -> float: return float(root)*float(taper)

def _param(name: str, value: Any, unit: str='mm', comment: str='H2AIR CAD export') -> Dict[str,Any]:
    return {'name':name,'value':value,'unit':unit,'comment':comment}

def _find_comp(cfg: Mapping[str,Any], name: str) -> Mapping[str,Any]|None:
    for c in cfg['geometry_mass']['component_ledger']['components']:
        if c.get('name')==name: return c
    return None

def build_cad_parameters(cfg: Mapping[str,Any], structure: Mapping[str,Any]) -> Dict[str,Any]:
    d=cfg['design']; u=cfg.get('user_input_snapshot',{}); cad=u.get('cad_export',{})
    gm=cfg['geometry_mass']['geometry_model']['geometry']; mm=cfg.get('manufacturing_mass_model',{})
    hw=cfg.get('hardware_selection_result',{})
    out=[]
    # Fuselage sections (lengths user-fixed; diameters may be packaging-auto).
    fuse_secs = mm.get('fuselage_sections') or u.get('fuselage',{}).get('sections',[])
    x=0.0
    for sec in fuse_secs:
        n=str(sec['name']).lower()
        L=float(sec['length_m']); dia=float(sec['diameter_m'])
        out += [_param(f'fuselage_{n}_length',_mm(L)),_param(f'fuselage_{n}_diameter',_mm(dia)),_param(f'fuselage_{n}_start_x',_mm(x)),_param(f'fuselage_{n}_end_x',_mm(x+L))]
        x += L
    out += [
        _param('fuselage_total_length', _mm(d['fuselage_length_m'])),
        _param('fuselage_max_diameter', _mm(d['fuselage_max_diameter_m'])),
        _param('fuselage_width', _mm(float(d.get('fuselage_width_m', d['fuselage_max_diameter_m'])))),
        _param('fuselage_height', _mm(float(d.get('fuselage_height_m', d['fuselage_max_diameter_m'])))),
        _param('fuselage_cross_section', str(d.get('fuselage_cross_section', 'rectangular_filleted')), 'String'),
        _param('fuselage_fillet_radius', _mm(float(d.get('fuselage_fillet_radius_m', 0.0)))),
    ]

    # Main wing.
    cr=float(d['main_wing_root_chord_m']); lam=float(d['main_wing_taper_ratio']); mac=float(d['wing_mac_m']); b=float(d['wing_span_m']); xac=float(d['wing_ac_x_m']); le=xac-0.25*mac
    mw=mm.get('main_wing',{})
    out += [
        _param('wing_span',_mm(b)),_param('wing_half_span',_mm(b/2)),_param('wing_root_chord',_mm(cr)),_param('wing_tip_chord',_mm(_tip(cr,lam))),_param('wing_mac',_mm(mac)),
        _param('wing_le_x',_mm(le)),_param('wing_ac_x',_mm(xac)),_param('wing_z',_mm(float(mw.get('z_m',0.1)))),_param('wing_thickness',_mm(float(mw.get('thickness_m',0.028)))),
        _param('wing_taper_ratio',lam,'unitless'),_param('wing_sweep',float(d.get('main_wing_sweep_deg',0)),'deg'),
        _param('wing_dihedral',float(d.get('main_wing_dihedral_deg',0)),'deg'),_param('wing_twist_root',float(d.get('main_wing_twist_root_deg',0)),'deg'),_param('wing_twist_tip',float(d.get('main_wing_twist_tip_deg',0)),'deg'),
        _param('wing_NACA',str(d.get('main_wing_naca_code','')),'String')]
    wsec=structure['structural_checks']['wing']['recommended_screen']['section']
    out += [_param('wing_primary_spar_OD',_mm(wsec['od_m'])),_param('wing_primary_spar_ID',_mm(wsec['id_m']))]
    if mw:
        s2=mw['secondary_spar']; out += [_param('wing_primary_spar_chord_fraction',float(mw['primary_spar'].get('chord_fraction',0.25)),'unitless'),_param('wing_secondary_spar_chord_fraction',float(s2.get('chord_fraction',0.65)),'unitless'),_param('wing_secondary_spar_OD',_mm(s2.get('tube_od_m',0.0)))]
        ps=mw.get('primary_spar',{})
        layout_od=float(gm.get('wing_primary_spar_layout_od_m', ps.get('design_tube_od_m', ps.get('reference_tube_od_m', ps.get('tube_od_m', wsec['od_m'])))))
        out.append(_param('wing_primary_spar_layout_OD',_mm(layout_od)))
        step=(cfg.get('structures') or {}).get('structure',{}).get('stepped_wing_primary_spar') or {}
        if bool(ps.get('stepped')) or bool(step.get('enabled')):
            inb=ps.get('inboard') or {}
            outb=ps.get('outboard') or {}
            in_od=float(inb.get('design_tube_od_m', step.get('inboard_od_m', layout_od)))
            in_id=float(inb.get('design_tube_id_m', step.get('inboard_id_m', 0.75*in_od)))
            out_od=float(outb.get('design_tube_od_m', step.get('outboard_od_m', 0.016)))
            out_id=float(outb.get('design_tube_id_m', step.get('outboard_id_m', 0.75*out_od)))
            frac=float(ps.get('joiner_span_fraction', step.get('joiner_span_fraction', 0.45)))
            out += [
                _param('wing_spar_inboard_OD',_mm(in_od)),
                _param('wing_spar_inboard_ID',_mm(in_id)),
                _param('wing_spar_outboard_OD',_mm(out_od)),
                _param('wing_spar_outboard_ID',_mm(out_id)),
                _param('wing_spar_joiner_station_y',_mm(0.5*frac*b)),
                _param('wing_spar_joiner_span_fraction',frac,'unitless'),
            ]
    longr=mm.get('fuselage_longerons',{})
    if longr:
        out += [
            _param('fuselage_longeron_OD',_mm(float(longr.get('tube_od_m',0.0065)))),
            _param('fuselage_longeron_boss_OD',_mm(float(longr.get('boss_od_m',0.016)))),
            _param('fuselage_longeron_top_z',_mm(float(gm.get('fuselage_longeron_top_z_m',gm.get('fuselage_longeron_z_m',0.0))))),
            _param('fuselage_longeron_bottom_z',_mm(float(gm.get('fuselage_longeron_bottom_z_m',0.0)))),
            _param('fuselage_top_z',_mm(float(gm.get('fuselage_top_z_m',0.0)))),
            _param('wing_drop_from_flush',_mm(float(gm.get('wing_drop_from_flush_m',0.0)))),
        ]

    # Horizontal tail.
    hr=float(d['horizontal_tail_root_chord_m']); hl=float(d['horizontal_tail_taper_ratio']); hm=float(d['horizontal_tail_mac_m']); hb=float(d['horizontal_tail_span_m']); hx=float(d['horizontal_tail_ac_x_m']); hle=hx-0.25*hm
    ht=mm.get('horizontal_tail',{})
    out += [
        _param('horizontal_tail_span',_mm(hb)),_param('horizontal_tail_half_span',_mm(hb/2)),_param('horizontal_tail_root_chord',_mm(hr)),_param('horizontal_tail_tip_chord',_mm(_tip(hr,hl))),_param('horizontal_tail_mac',_mm(hm)),
        _param('horizontal_tail_le_x',_mm(hle)),_param('horizontal_tail_ac_x',_mm(hx)),_param('horizontal_tail_z',_mm(float(ht.get('z_m',0.03)))),_param('horizontal_tail_thickness',_mm(float(ht.get('thickness_m',0.022)))),
        _param('horizontal_tail_taper_ratio',hl,'unitless'),_param('horizontal_tail_NACA',str(d.get('horizontal_tail_naca_code','')),'String'),
        _param('elevator_span_fraction',float(d.get('elevator_span_fraction',1.0)),'unitless'),
        _param('elevator_total_span',_mm(hb*float(d.get('elevator_span_fraction',1.0)))),
        _param('elevator_chord_fraction',float(d.get('elevator_chord_fraction',0.30)),'unitless'),
        _param('elevator_hinge_x_over_c',1.0-float(d.get('elevator_chord_fraction',0.30)),'unitless')]
    hsec=structure['structural_checks']['tails']['horizontal_recommended_screen']['section']
    out += [_param('horizontal_tail_primary_spar_OD',_mm(hsec['od_m'])),_param('horizontal_tail_primary_spar_ID',_mm(hsec['id_m']))]
    if ht:
        out += [_param('horizontal_tail_secondary_spar_OD',_mm(ht['secondary_spar'].get('tube_od_m',0.0)))]

    # Vertical tails.
    vr=float(d['vertical_tail_root_chord_m']); vl=float(d['vertical_tail_taper_ratio']); vh=float(d['vertical_tail_height_m']); vx=float(d.get('vertical_tail_ac_x_m',d['horizontal_tail_ac_x_m'])); vm=(2/3)*vr*(1+vl+vl*vl)/(1+vl); vle=vx-0.25*vm
    vt=mm.get('vertical_tail',{})
    out += [
        _param('vertical_tail_count',int(vt.get('count',2)),'unitless'),_param('vertical_tail_height',_mm(vh)),_param('vertical_tail_root_chord',_mm(vr)),_param('vertical_tail_tip_chord',_mm(_tip(vr,vl))),
        _param('vertical_tail_le_x',_mm(vle)),_param('vertical_tail_ac_x',_mm(vx)),_param('vertical_tail_thickness',_mm(float(vt.get('thickness_m',0.012)))),
        _param('vertical_tail_taper_ratio',vl,'unitless'),_param('vertical_tail_NACA',str(d.get('vertical_tail_naca_code','')),'String'),
        _param('rudder_height_fraction',float(d.get('rudder_height_fraction',1.0)),'unitless'),
        _param('rudder_height',_mm(vh*float(d.get('rudder_height_fraction',1.0)))),
        _param('rudder_chord_fraction',float(d.get('rudder_chord_fraction',0.30)),'unitless'),
        _param('rudder_hinge_x_over_c',1.0-float(d.get('rudder_chord_fraction',0.30)),'unitless')]
    vsec=structure['structural_checks']['tails']['vertical_recommended_screen']['section']
    out += [_param('vertical_tail_primary_spar_OD',_mm(vsec['od_m'])),_param('vertical_tail_primary_spar_ID',_mm(vsec['id_m']))]
    if vt:
        out += [_param('vertical_tail_secondary_spar_OD',_mm(vt['secondary_spar'].get('tube_od_m',0.0)))]

    # Twin booms.
    bs=structure['structural_checks']['booms']['recommended_screen']['section']; bx0=float(gm['boom_x_start_m']); bx1=float(gm['boom_x_end_m']); ys=gm['boom_center_y_m']
    out += [_param('boom_length',_mm(bx1-bx0)),_param('boom_start_x',_mm(bx0)),_param('boom_end_x',_mm(bx1)),_param('boom_left_y',_mm(ys[0])),_param('boom_right_y',_mm(ys[1])),_param('boom_spacing',_mm(abs(float(ys[1])-float(ys[0])))),_param('boom_z',_mm(float(gm.get('boom_z_m',0.0)))),_param('boom_OD',_mm(bs['od_m'])),_param('boom_ID',_mm(bs['id_m']))]

    # Landing legs (nose + wing left/right): geometry only.
    land = (structure.get('structural_checks') or {}).get('landing_legs') or {}
    layout = gm.get('landing_legs') or (mm.get('landing_legs') or {}).get('resolved') or {}
    if land.get('enabled') and layout.get('enabled'):
        sec = land['recommended_screen']['section']
        tube_od = float(sec['od_m']); tube_id = float(sec['id_m'])
        wall = float(layout.get('sleeve_wall_m', (mm.get('landing_legs') or {}).get('sleeve_wall_m', 0.002)))
        sleeve_od = tube_od + 2.0 * wall
        out += [
            _param('landing_legs_enabled', 1, 'unitless'),
            _param('landing_ground_z', _mm(float(layout['ground_z_m']))),
            _param('landing_leg_tube_OD', _mm(tube_od)),
            _param('landing_leg_tube_ID', _mm(tube_id)),
            _param('landing_leg_sleeve_OD', _mm(sleeve_od)),
            _param('landing_leg_sleeve_wall', _mm(wall)),
            _param('landing_leg_sizing_length', _mm(float(layout.get('sizing_length_m', land.get('length_m', 0.0))))),
        ]
        for leg in layout.get('legs', []):
            prefix = f"landing_leg_{leg['name']}"
            out += [
                _param(f'{prefix}_x', _mm(float(leg['x_m']))),
                _param(f'{prefix}_y', _mm(float(leg['y_m']))),
                _param(f'{prefix}_mount_z', _mm(float(leg['mount_z_m']))),
                _param(f'{prefix}_foot_z', _mm(float(leg['foot_z_m']))),
                _param(f'{prefix}_length', _mm(float(leg['length_m']))),
            ]
    else:
        out.append(_param('landing_legs_enabled', 0, 'unitless'))

    # Lift rows and selected hardware.
    centers=gm['lift_prop_centers_m']; front=float(centers[0][0]); rear=float(centers[2][0]); yabs=abs(float(centers[0][1])); z=float(centers[0][2])
    lift=hw.get('selected_lift_combo',{}); push=hw.get('selected_pusher_combo',{}); bat=hw.get('selected_battery',{})
    out += [_param('lift_motor_front_x',_mm(front)),_param('lift_motor_rear_x',_mm(rear)),_param('lift_motor_y_abs',_mm(yabs)),_param('lift_motor_z',_mm(z)),_param('lift_prop_diameter',float(d['lift_propeller_diameter_in'])*25.4),_param('lift_prop_pitch',float(lift.get('prop_pitch_in',0))*25.4),_param('lift_motor_name',str(lift.get('name','')),'String')]
    if lift.get('motor_diameter_m'): out += [_param('lift_motor_OD',_mm(lift['motor_diameter_m'])),_param('lift_motor_body_length',_mm(lift['motor_length_m']))]
    pusher_station=cfg['sixdof_control']['aircraft']['pusher_station_m']
    out += [_param('pusher_motor_x',_mm(pusher_station[0])),_param('pusher_motor_y',_mm(pusher_station[1])),_param('pusher_motor_z',_mm(pusher_station[2])),_param('pusher_prop_diameter',float(d['pusher_propeller_diameter_in'])*25.4),_param('pusher_prop_pitch',float(push.get('prop_pitch_in',0))*25.4),_param('pusher_motor_name',str(push.get('name','')),'String')]
    if push.get('motor_diameter_m'): out += [_param('pusher_motor_OD',_mm(push['motor_diameter_m'])),_param('pusher_motor_body_length',_mm(push['motor_length_m']))]

    # Major component envelopes and stations (no masses).
    fuse_dia = float(d.get('fuselage_max_diameter_m', 0.18))
    for name, prefix in [
        ('payload', 'payload'),
        ('buffer_battery', 'battery'),
        ('fuel_cell_system', 'fuel_cell'),
        ('fuel_cell_control_box', 'fuel_cell_control_box'),
        ('hydrogen_cylinder', 'hydrogen_cylinder'),
        ('autopilot', 'cube_orange'),
        ('power_module', 'pdb'),
        ('power_module_2', 'pdb2'),
    ]:
        c = _find_comp(cfg, name)
        if not c:
            continue
        pos = c.get('position_m', [0, 0, 0])
        dims = dict(c.get('dimensions_m', {}) or {})
        out += [
            _param(f'{prefix}_x', _mm(pos[0])),
            _param(f'{prefix}_y', _mm(pos[1] if len(pos) > 1 else 0.0)),
            _param(f'{prefix}_z', _mm(pos[2] if len(pos) > 2 else 0.0)),
        ]
        if 'diameter' in dims and 'length' in dims:
            out += [
                _param(f'{prefix}_length', _mm(float(dims['length']))),
                _param(f'{prefix}_diameter', _mm(float(dims['diameter']))),
            ]
            continue
        L = float(dims.get('length_x') or dims.get('length') or 0.0)
        W = dims.get('width_y')
        H = dims.get('height_z')
        # Screening fallbacks when only length is controlled (e.g. payload).
        if W is None or float(W or 0) <= 0:
            W = min(0.08, 0.55 * fuse_dia)
        if H is None or float(H or 0) <= 0:
            H = min(0.08, 0.55 * fuse_dia)
        out += [
            _param(f'{prefix}_length', _mm(L)),
            _param(f'{prefix}_width', _mm(float(W))),
            _param(f'{prefix}_height', _mm(float(H))),
        ]

    # IE-SOAR cross-fuselage cooling geometry.  Performance quantities are
    # intentionally not exported; only drawing-driving geometry and layout.
    cool=(cfg.get('_runtime',{}) or {}).get('fuel_cell_cooling') or {}
    cool_user=(u.get('fuel_cell_cooling') or {})
    if cool and cool.get('enabled'):
        ci=cool.get('inlet_geometry',{}); co=cool.get('exhaust_geometry',{}); cf=cool.get('fan_geometry',{})
        fuse_w=float(cool.get('fuselage_width_m',d.get('fuselage_width_m',0.0)))
        side_sign={'left':-1.0,'right':1.0}
        inlet_y=side_sign.get(str(cool.get('inlet_side','left')).lower(),-1.0)*0.5*fuse_w
        exhaust_y=side_sign.get(str(cool.get('exhaust_side','right')).lower(),1.0)*0.5*fuse_w
        out += [
            _param('fc_cooling_architecture',str(cool.get('architecture','cross_fuselage')),'String'),
            _param('fc_cooling_fan_flow_direction',str(cool.get('fan_flow_direction','suction_through_stack_to_fan_side')),'String'),
            _param('fc_cooling_inlet_side',str(cool.get('inlet_side','left')),'String'),
            _param('fc_cooling_exhaust_side',str(cool.get('exhaust_side','right')),'String'),
            _param('fc_cooling_inlet_center_x',_mm(float(ci.get('center_x_m',0.0)))),
            _param('fc_cooling_inlet_center_y',_mm(inlet_y)),
            _param('fc_cooling_inlet_center_z',_mm(float(ci.get('center_z_m',0.0)))),
            _param('fc_cooling_inlet_panel_length',_mm(float(ci.get('panel_length_x_m',0.0)))),
            _param('fc_cooling_inlet_panel_height',_mm(float(ci.get('panel_height_z_m',0.0)))),
            _param('fc_cooling_inlet_slot_count',int(ci.get('slot_count',0)),'unitless'),
            _param('fc_cooling_inlet_slot_length',_mm(float(ci.get('slot_length_x_m',0.0)))),
            _param('fc_cooling_inlet_slot_width',_mm(float(ci.get('slot_width_z_m',0.0)))),
            _param('fc_cooling_inlet_slot_gap',_mm(float(ci.get('slot_gap_z_m',0.0)))),
            _param('fc_cooling_inlet_recess_depth',_mm(float(ci.get('recess_depth_y_m',0.0)))),
            _param('fc_cooling_inlet_ramp_length',_mm(float(ci.get('ramp_length_x_m',0.0)))),
            _param('fc_cooling_inlet_ramp_angle',float(ci.get('ramp_angle_deg',0.0)),'deg'),
            _param('fc_cooling_inlet_sidewall_angle',float(ci.get('sidewall_angle_deg',0.0)),'deg'),
            _param('fc_cooling_inlet_lip_radius',_mm(float(ci.get('lip_radius_m',0.0)))),
            _param('fc_cooling_exhaust_center_x',_mm(float(co.get('center_x_m',0.0)))),
            _param('fc_cooling_exhaust_center_y',_mm(exhaust_y)),
            _param('fc_cooling_exhaust_center_z',_mm(float(co.get('center_z_m',0.0)))),
            _param('fc_cooling_exhaust_panel_length',_mm(float(co.get('panel_length_x_m',0.0)))),
            _param('fc_cooling_exhaust_panel_height',_mm(float(co.get('panel_height_z_m',0.0)))),
            _param('fc_cooling_exhaust_slot_count',int(co.get('slot_count',0)),'unitless'),
            _param('fc_cooling_exhaust_slot_length',_mm(float(co.get('slot_length_x_m',0.0)))),
            _param('fc_cooling_exhaust_slot_width',_mm(float(co.get('slot_width_z_m',0.0)))),
            _param('fc_cooling_exhaust_slot_gap',_mm(float(co.get('slot_gap_z_m',0.0)))),
            _param('fc_cooling_exhaust_shroud_depth',_mm(float(co.get('fan_shroud_depth_y_m',0.0)))),
            _param('fc_cooling_exhaust_louvre_angle',float(co.get('louvre_angle_deg',0.0)),'deg'),
            _param('fc_cooling_exhaust_lip_radius',_mm(float(co.get('lip_radius_m',0.0)))),
            _param('fc_cooling_fan_count',int(cf.get('count',0)),'unitless'),
            _param('fc_cooling_fan_diameter',_mm(float(cf.get('diameter_m',0.0)))),
            _param('fc_cooling_fan_center_spacing',_mm(float(cf.get('center_spacing_m',0.0)))),
            _param('fc_cooling_fan_1_center_x',_mm(float(co.get('center_x_m',0.0))-0.5*float(cf.get('center_spacing_m',0.0)))),
            _param('fc_cooling_fan_2_center_x',_mm(float(co.get('center_x_m',0.0))+0.5*float(cf.get('center_spacing_m',0.0)))),
            _param('fc_cooling_fan_center_y',_mm(exhaust_y)),
            _param('fc_cooling_fan_center_z',_mm(float(co.get('center_z_m',0.0)))),
            _param('fc_cooling_inlet_plenum_gap_actual',_mm(float(cool.get('inlet_side_plenum_gap_m',0.0)))),
            _param('fc_cooling_exhaust_plenum_gap_actual',_mm(float(cool.get('exhaust_side_plenum_gap_m',0.0)))),
            _param('fc_cooling_inlet_plenum_gap_min',_mm(float(cool.get('required_inlet_side_plenum_gap_m',0.0)))),
            _param('fc_cooling_exhaust_plenum_gap_min',_mm(float(cool.get('required_exhaust_side_plenum_gap_m',0.0)))),
            _param('fc_cooling_min_fuselage_width',_mm(float(cool.get('minimum_fuselage_width_for_plenums_m',0.0)))),
        ]
        ccad=cool_user.get('cad',{})
        if ccad:
            out += [
                _param('fc_cooling_section_joint_edge_distance',_mm(float(ccad.get('minimum_edge_distance_from_fuselage_section_joint_m',0.012)))),
                _param('fc_cooling_grille_corner_radius',_mm(float(ccad.get('grille_corner_radius_m',0.004)))),
                _param('fc_cooling_min_web_between_slots',_mm(float(ccad.get('minimum_web_between_slots_m',0.003)))),
                _param('fc_cooling_exhaust_grille_fastener',str(ccad.get('removable_exhaust_grille_fastener','M3')),'String'),
            ]

    # Drawing/assembly parameters deliberately controlled by the user.
    tol=cad.get('tolerances_mm',{})
    for k,v in tol.items(): out.append(_param(k,float(v),'mm','User CAD tolerance/clearance'))
    fast=cad.get('fasteners',{})
    for label,val in fast.items():
        unit='String' if isinstance(val,str) else 'mm'
        out.append(_param(label,val,unit,'User CAD fastener parameter'))

    # Rib counts derived only for drawing generation from target spacing.
    rib=cad.get('rib_target_spacing_m',{})
    if rib:
        out += [_param('wing_rib_count',max(2,math.ceil(b/float(rib.get('main_wing',0.25)))+1),'unitless'),_param('horizontal_tail_rib_count',max(2,math.ceil(hb/float(rib.get('horizontal_tail',0.15)))+1),'unitless'),_param('vertical_tail_rib_count_each',max(2,math.ceil(vh/float(rib.get('vertical_tail',0.12)))+1),'unitless')]

    return {'schema':'H2AIR-CAD-parameters-v1','model_version':cfg['model']['version'],'parameters':out,'note':'Geometry/CAD only: masses, powers, stresses and performance outputs are intentionally excluded.'}


def _xml_value_text(value: Any, unit: str) -> str:
    if unit == 'mm':
        return f'{float(value):.3f} mm'
    if unit == 'deg':
        return f'{float(value):.3f} deg'
    return str(value)


def write_params_xml(path: Path, parameters: Sequence[Mapping[str, Any]]) -> None:
    """Inventor-style ParamWithValueList XML matching parameters.xml."""
    root = ET.Element('ParamWithValueList', {
        'xmlns:xsd': 'http://www.w3.org/2001/XMLSchema',
        'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
    })
    ET.SubElement(root, 'version').text = '20080502'
    pts = ET.SubElement(root, 'parameterTypes')
    for name, code in [('Inventor', '0'), ('String', '1'), ('Boolean', '2')]:
        pt = ET.SubElement(pts, 'ParamType')
        ET.SubElement(pt, 'typeName').text = name
        ET.SubElement(pt, 'typeCode').text = code
    pars = ET.SubElement(root, 'parameters')
    for p in parameters:
        node = ET.SubElement(pars, 'ParamWithValue')
        ET.SubElement(node, 'name').text = p['name']
        ET.SubElement(node, 'typeCode').text = p['unit']
        ET.SubElement(node, 'value').text = _xml_value_text(p['value'], p['unit'])
        ET.SubElement(node, 'comment').text = p['comment']
        ET.SubElement(node, 'isKey').text = 'false'
    tree = ET.ElementTree(root)
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)


def write_params_xlsx(path: Path, parameters: Sequence[Mapping[str, Any]]) -> None:
    """Excel table matching parameters.xlsx: Name / Value / Unit / Comment."""
    if Workbook is None:
        raise ImportError('openpyxl is required to write params.xlsx CAD exports')
    wb = Workbook()
    ws = wb.active
    ws.title = 'Parameters'
    ws.append(['Name', 'Value', 'Unit', 'Comment'])
    for p in parameters:
        val = p['value']
        if p['unit'] in ('mm', 'deg', 'unitless') and not isinstance(val, str):
            try:
                val = float(val)
            except (TypeError, ValueError):
                pass
        ws.append([p['name'], val, p['unit'], p['comment']])
    ws.column_dimensions['A'].width = 45.0
    ws.column_dimensions['B'].width = 18.0
    ws.column_dimensions['C'].width = 10.0
    ws.column_dimensions['D'].width = 45.0
    path.parent.mkdir(parents=True, exist_ok=True)
    wb.save(path)


def _write_params_bundle(directory: Path, data: Mapping[str, Any]) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    params = data['parameters']
    write_params_xml(directory / 'params.xml', params)
    write_params_xlsx(directory / 'params.xlsx', params)


def write_cad_exports(outdir: str | Path, cfg: Mapping[str, Any], structure: Mapping[str, Any]) -> None:
    # CAD export is a v4 user-workflow feature. Legacy/internal regression
    # configurations have no controlled CAD input block and are intentionally
    # skipped rather than guessing drawing parameters.
    u = cfg.get('user_input_snapshot')
    if not u or not u.get('cad_export', {}).get('enabled', False):
        return
    outdir = Path(outdir)
    data = build_cad_parameters(cfg, structure)
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / 'cad_parameters.json').write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')
    # Run-folder copies for reproducibility / tests.
    _write_params_bundle(outdir, data)
    # Also keep the legacy XML filename used by older run consumers.
    write_params_xml(outdir / 'cad_parameters.xml', data['parameters'])

    export_dir = u.get('cad_export', {}).get('export_directory')
    if export_dir:
        _write_params_bundle(Path(export_dir), data)
