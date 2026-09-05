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
    # Fuselage sections, fixed by user.
    x=0.0
    for sec in u.get('fuselage',{}).get('sections',[]):
        n=str(sec['name']).lower()
        L=float(sec['length_m']); dia=float(sec['diameter_m'])
        out += [_param(f'fuselage_{n}_length',_mm(L)),_param(f'fuselage_{n}_diameter',_mm(dia)),_param(f'fuselage_{n}_start_x',_mm(x)),_param(f'fuselage_{n}_end_x',_mm(x+L))]
        x += L
    out += [_param('fuselage_total_length',_mm(d['fuselage_length_m'])),_param('fuselage_max_diameter',_mm(d['fuselage_max_diameter_m']))]

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
        _param('horizontal_tail_taper_ratio',hl,'unitless'),_param('horizontal_tail_NACA',str(d.get('horizontal_tail_naca_code','')),'String')]
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
        _param('vertical_tail_taper_ratio',vl,'unitless'),_param('vertical_tail_NACA',str(d.get('vertical_tail_naca_code','')),'String')]
    vsec=structure['structural_checks']['tails']['vertical_recommended_screen']['section']
    out += [_param('vertical_tail_primary_spar_OD',_mm(vsec['od_m'])),_param('vertical_tail_primary_spar_ID',_mm(vsec['id_m']))]
    if vt:
        out += [_param('vertical_tail_secondary_spar_OD',_mm(vt['secondary_spar'].get('tube_od_m',0.0)))]

    # Twin booms.
    bs=structure['structural_checks']['booms']['recommended_screen']['section']; bx0=float(gm['boom_x_start_m']); bx1=float(gm['boom_x_end_m']); ys=gm['boom_center_y_m']
    out += [_param('boom_length',_mm(bx1-bx0)),_param('boom_start_x',_mm(bx0)),_param('boom_end_x',_mm(bx1)),_param('boom_left_y',_mm(ys[0])),_param('boom_right_y',_mm(ys[1])),_param('boom_spacing',_mm(abs(float(ys[1])-float(ys[0])))),_param('boom_z',_mm(float(gm.get('boom_z_m',0.0)))),_param('boom_OD',_mm(bs['od_m'])),_param('boom_ID',_mm(bs['id_m']))]

    # Lift rows and selected hardware.
    centers=gm['lift_prop_centers_m']; front=float(centers[0][0]); rear=float(centers[2][0]); yabs=abs(float(centers[0][1])); z=float(centers[0][2])
    lift=hw.get('selected_lift_combo',{}); push=hw.get('selected_pusher_combo',{}); bat=hw.get('selected_battery',{})
    out += [_param('lift_motor_front_x',_mm(front)),_param('lift_motor_rear_x',_mm(rear)),_param('lift_motor_y_abs',_mm(yabs)),_param('lift_motor_z',_mm(z)),_param('lift_prop_diameter',float(d['lift_propeller_diameter_in'])*25.4),_param('lift_prop_pitch',float(lift.get('prop_pitch_in',0))*25.4),_param('lift_motor_name',str(lift.get('name','')),'String')]
    if lift.get('motor_diameter_m'): out += [_param('lift_motor_OD',_mm(lift['motor_diameter_m'])),_param('lift_motor_body_length',_mm(lift['motor_length_m']))]
    pusher_station=cfg['sixdof_control']['aircraft']['pusher_station_m']
    out += [_param('pusher_motor_x',_mm(pusher_station[0])),_param('pusher_motor_y',_mm(pusher_station[1])),_param('pusher_motor_z',_mm(pusher_station[2])),_param('pusher_prop_diameter',float(d['pusher_propeller_diameter_in'])*25.4),_param('pusher_prop_pitch',float(push.get('prop_pitch_in',0))*25.4),_param('pusher_motor_name',str(push.get('name','')),'String')]
    if push.get('motor_diameter_m'): out += [_param('pusher_motor_OD',_mm(push['motor_diameter_m'])),_param('pusher_motor_body_length',_mm(push['motor_length_m']))]

    # Major component envelopes and stations (no masses).
    for name,prefix in [('buffer_battery','battery'),('fuel_cell_system','fuel_cell'),('hydrogen_cylinder','hydrogen_cylinder')]:
        c=_find_comp(cfg,name)
        if not c: continue
        pos=c.get('position_m',[0,0,0]); dims=c.get('dimensions_m',{})
        out += [_param(f'{prefix}_x',_mm(pos[0])),_param(f'{prefix}_y',_mm(pos[1])),_param(f'{prefix}_z',_mm(pos[2]))]
        if 'length_x' in dims:
            out += [_param(f'{prefix}_length',_mm(dims['length_x'])),_param(f'{prefix}_width',_mm(dims['width_y'])),_param(f'{prefix}_height',_mm(dims['height_z']))]
        elif 'length' in dims:
            out += [_param(f'{prefix}_length',_mm(dims['length'])),_param(f'{prefix}_diameter',_mm(dims['diameter']))]

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
