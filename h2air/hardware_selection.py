from __future__ import annotations

"""Discrete motor/prop/battery database selection for H2AIR.

The selector is deliberately conservative and transparent. It screens hardware
against voltage, 15 kg lift-thrust, pusher transient power, battery energy/current
and simple propeller-geometry bounds. Feasible items are scored primarily on
installed mass, with smaller penalties for poor power/thrust efficiency and low
margin. It is a screening selector, not a substitute for measured prop maps.
"""

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple
import json, math

G=9.80665


def load_hardware_database(root: Path) -> Dict[str,Any]:
    return json.loads((root/'data/hardware/hardware_database.json').read_text(encoding='utf-8'))


def _role_ok(rec: Mapping[str,Any], role: str) -> bool:
    r=str(rec.get('role','both')).lower()
    return r in ('both',role)


def _find(items: Sequence[Mapping[str,Any]], name: str) -> Dict[str,Any]:
    for x in items:
        if x['name']==name: return deepcopy(dict(x))
    raise ValueError(f"Hardware database item not found: {name}")


def _compatible_voltage(rec: Mapping[str,Any], bus_nominal_V: float, bus_max_V: float) -> bool:
    return float(rec['voltage_min_V']) <= bus_nominal_V+1e-9 and float(rec['voltage_max_V']) >= bus_max_V-1e-9


def select_hardware(user: Mapping[str,Any], *, root: Path) -> Dict[str,Any]:
    db=load_hardware_database(root)
    hs=deepcopy(dict(user.get('hardware_selection',{})))
    mode=str(hs.get('mode','auto')).lower()
    hard=hs.get('hardcoded',{})
    crit=hs.get('auto_criteria',{})
    nominal_cell=float(crit.get('nominal_cell_voltage_V',3.7))
    max_cell=float(crit.get('maximum_cell_voltage_V',4.0))
    s=int(crit.get('series_cells',12))
    # Direct-bus architecture is limited by the fuel-cell max (IE-SOAR 48 V).
    # Prefer explicit criteria, else FC rating from the user file, else 48 V.
    fc_max = None
    hw = user.get('hardware') or {}
    if isinstance(hw, Mapping) and isinstance(hw.get('fuel_cell'), Mapping):
        fc_max = hw['fuel_cell'].get('output_voltage_max_V')
    if fc_max is None and isinstance(user.get('power_system'), Mapping):
        fc = user['power_system'].get('fuel_cell') or {}
        if isinstance(fc, Mapping):
            fc_max = fc.get('output_voltage_max_V')
    configured_ceiling = crit.get('bus_voltage_ceiling_V')
    bus_ceiling = float(configured_ceiling if configured_ceiling is not None else (fc_max if fc_max is not None else 48.0))
    if fc_max is not None:
        bus_ceiling = min(bus_ceiling, float(fc_max))
    # Never screen above the architecture bus ceiling (not 12S×4.2 = 50.4 V).
    max_cell = min(max_cell, bus_ceiling / float(s))
    bus_nom=s*nominal_cell
    bus_max=min(s*max_cell, bus_ceiling)
    if bus_nom > bus_ceiling + 1e-9:
        raise ValueError(
            f"Battery nominal bus {bus_nom:.1f} V exceeds aircraft bus ceiling {bus_ceiling:.1f} V"
        )
    if bus_max > bus_ceiling + 1e-9:
        bus_max = bus_ceiling
    n_lift=int(crit.get('number_of_lift_motors',4))
    thrust_factor=float(crit.get('lift_total_thrust_factor_at_mtow_ceiling',1.5))
    climb_factor=float(crit.get('lift_climb_thrust_factor',1.08))
    mtow=float(user['mission']['mtow_ceiling_kg'])
    required_lift=mtow*G*thrust_factor*climb_factor/n_lift
    pusher_req=float(crit.get('minimum_pusher_power_W',750.0))
    min_capacity=float(crit.get('minimum_battery_capacity_Ah',5.0))
    current_margin=float(crit.get('battery_current_margin',1.05))
    min_prop=float(crit.get('minimum_lift_prop_diameter_in',18.0)); max_prop=float(crit.get('maximum_lift_prop_diameter_in',24.0))

    propulsion=db['propulsion_combos']; batteries=db['batteries']
    if mode=='hardcoded':
        lift=_find(propulsion,hard['lift_combo_name']); push=_find(propulsion,hard['pusher_combo_name']); bat=_find(batteries,hard['battery_name'])
    elif mode=='auto':
        lift_c=[]
        for r0 in propulsion:
            r=dict(r0)
            if not _role_ok(r,'lift') or not _compatible_voltage(r,bus_nom,bus_max): continue
            if float(r['max_thrust_N'])+1e-9<required_lift: continue
            if not (min_prop<=float(r['prop_diameter_in'])<=max_prop): continue
            m=float(r['motor_mass_kg'])+float(r['prop_mass_kg'])
            eff=float(r['max_thrust_N'])/max(float(r['max_power_W']),1e-9)
            margin=float(r['max_thrust_N'])/required_lift
            # Lower is better. Installed mass dominates; smaller prop and useful margin break ties.
            score=4*n_lift*m + 0.20/eff + 0.05*float(r['prop_diameter_in']) - 0.25*min(margin,1.5)
            lift_c.append((score,r))
        if not lift_c:
            raise ValueError(f"No AUTO lift motor/prop combo meets {required_lift:.1f} N/motor, {bus_nom:.1f} V nominal and {min_prop:.0f}-{max_prop:.0f} in prop bounds")
        lift=min(lift_c,key=lambda x:x[0])[1]

        push_c=[]
        for r0 in propulsion:
            r=dict(r0)
            if not _role_ok(r,'pusher') or not _compatible_voltage(r,bus_nom,bus_max): continue
            if float(r['max_power_W'])+1e-9<pusher_req: continue
            m=float(r['motor_mass_kg'])+float(r['prop_mass_kg'])
            margin=float(r['max_power_W'])/pusher_req
            score=5*m + 0.00015*float(r['max_power_W']) - 0.1*min(margin,1.5) + 0.01*float(r['prop_diameter_in'])
            push_c.append((score,r))
        if not push_c: raise ValueError(f"No AUTO pusher combo meets {pusher_req:.0f} W at 12S-class voltage")
        push=min(push_c,key=lambda x:x[0])[1]

        total_peak=float(lift['max_power_W'])*n_lift+float(push['max_power_W'])
        required_current=current_margin*total_peak/bus_nom
        bat_c=[]
        for b0 in batteries:
            b=dict(b0)
            if int(b.get('series_cells',round(float(b['voltage_V'])/nominal_cell)))!=s: continue
            if float(b.get('capacity_Ah',float(b['capacity_mAh'])/1000.0))+1e-9<min_capacity: continue
            curr=float(b.get('continuous_discharge_current_A',0.0) or 0.0)
            if curr+1e-9<required_current: continue
            # Prefer low mass, then smaller volume and modest surplus capacity.
            vol=float(b['length_m'])*float(b['width_m'])*float(b['height_m'])
            score=8*float(b['mass_kg'])+1000*vol+0.05*float(b['capacity_Ah'])
            bat_c.append((score,b))
        if not bat_c:
            raise ValueError(f"No AUTO 12S battery meets >= {min_capacity:.1f} Ah and >= {required_current:.0f} A continuous screening demand")
        bat=min(bat_c,key=lambda x:x[0])[1]
    else:
        raise ValueError("hardware_selection.mode must be 'auto' or 'hardcoded'")

    total_peak=float(lift['max_power_W'])*n_lift+float(push['max_power_W'])
    required_current=current_margin*total_peak/bus_nom
    return {
        'mode':mode,'bus_nominal_voltage_V':bus_nom,'bus_max_voltage_V':bus_max,
        'bus_voltage_ceiling_V':bus_ceiling,
        'requirements':{'lift_thrust_per_motor_N':required_lift,'minimum_pusher_power_W':pusher_req,'minimum_battery_capacity_Ah':min_capacity,'battery_screen_current_A':required_current},
        'selected_lift_combo':lift,'selected_pusher_combo':push,'selected_battery':bat,
        'notes':[
            'AUTO selection is a database screening step. Final propulsion release requires measured thrust/power/thermal data at the selected voltage and propeller.',
            'Battery current screen conservatively assumes simultaneous maximum lift + pusher database power.',
            f'Aircraft bus ceiling is {bus_ceiling:.1f} V (fuel-cell max); hardware bus_max is capped accordingly (not 12S×4.2=50.4 V).',
        ],
    }


def apply_hardware_selection(cfg: Dict[str,Any], selection: Mapping[str,Any]) -> None:
    d=cfg['design']; lift=selection['selected_lift_combo']; push=selection['selected_pusher_combo']; bat=selection['selected_battery']
    d['lift_propeller_diameter_in']=float(lift['prop_diameter_in'])
    d['lift_motor_available_thrust_per_motor_N']=float(lift['max_thrust_N'])
    d['lift_motor_structural_limit_thrust_per_motor_N']=max(float(lift['max_thrust_N'])*1.12,float(lift['max_thrust_N'])+5.0)
    d['pusher_propeller_diameter_in']=float(push['prop_diameter_in'])
    d['pusher_max_electrical_power_W']=float(push['max_power_W'])
    d['selected_lift_combo_name']=lift['name']; d['selected_pusher_combo_name']=push['name']; d['selected_battery_name']=bat['name']

    bcfg=cfg['power_system']['battery']
    bcfg.update({'name':bat['name'],'series_cells':int(bat.get('series_cells',12)),'capacity_Ah':float(bat['capacity_Ah']),'mass_kg':float(bat['mass_kg']),'continuous_discharge_current_A':float(bat.get('continuous_discharge_current_A')) if bat.get('continuous_discharge_current_A') is not None else None})

    comps=cfg['geometry_mass']['component_ledger']['components']; by={x['name']:x for x in comps}
    for i in range(1,5):
        c=by.get(f'lift_propulsion_{i}')
        if c:
            c['mass_takeoff_kg']=float(lift['motor_mass_kg'])+float(lift['prop_mass_kg']); c['mass_end_kg']=c['mass_takeoff_kg']
            c['status']=lift.get('status','database'); c['source']=lift.get('source','hardware database')
            if lift.get('motor_diameter_m') and lift.get('motor_length_m'):
                c['shape']='cylinder_z'; c['dimensions_m']={'length':float(lift['motor_length_m']),'diameter':float(lift['motor_diameter_m'])}
    c=by.get('cruise_propulsion')
    if c:
        c['mass_takeoff_kg']=float(push['motor_mass_kg'])+float(push['prop_mass_kg']); c['mass_end_kg']=c['mass_takeoff_kg']; c['status']=push.get('status','database'); c['source']=push.get('source','hardware database')
        if push.get('motor_diameter_m') and push.get('motor_length_m'):
            c['shape']='cylinder_x'; c['dimensions_m']={'length':float(push['motor_length_m']),'diameter':float(push['motor_diameter_m'])}
    c=by.get('buffer_battery')
    if c:
        c['mass_takeoff_kg']=float(bat['mass_kg']); c['mass_end_kg']=c['mass_takeoff_kg']; c['status']=bat.get('status','database'); c['source']=bat.get('source','hardware database')
        c['shape']='box'; c['dimensions_m']={'length_x':float(bat['length_m']),'width_y':float(bat['width_m']),'height_z':float(bat['height_m'])}
    cfg['hardware_selection_result']=deepcopy(dict(selection))
