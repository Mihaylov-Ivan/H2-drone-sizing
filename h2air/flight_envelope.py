from __future__ import annotations
from pathlib import Path
from typing import Any, Dict, Mapping
import csv, json
from .pipeline import _components_with_power,_structural_closure,_sync_design
from .aero_propulsion import cruise_drag,stall_speed_mps,lift_system_sizing


def build_flight_envelope(cfg:Mapping[str,Any],*,v_min:float=10.0,v_max:float=35.0,dv:float=1.0)->Dict[str,Any]:
    base=_components_with_power(cfg);_,mass,_,_= _structural_closure(cfg,base);a,_,_,_=_sync_design(cfg,mass);d=cfg['design'];vs=stall_speed_mps(a,mass.takeoff.total_mass_kg,float(d['wing_area_m2']))
    rows=[];v=v_min
    gate=float(cfg['power_system']['requirements']['representative_cruise_power_gate_W'])
    while v<=v_max+1e-9:
        dr=cruise_drag(a,mass_kg=mass.takeoff.total_mass_kg,speed_mps=v,lift_prop_diameter_in=float(d['lift_propeller_diameter_in']),lift_prop_state=str(d['cruise_lift_propeller_state']),wing_area_m2=float(d['wing_area_m2']),wing_span_m=float(d['wing_span_m']))
        ratio=v/max(vs,1e-9);cls='VALID' if ratio>=1.30 and dr.total_cruise_bus_power_W<=gate else ('STALL_MARGIN_FAIL' if ratio<1.30 else 'POWER_GATE_FAIL')
        rows.append({'speed_mps':v,'stall_speed_mps':vs,'V_over_Vstall':ratio,'drag_N':dr.total_drag_N,'bus_power_W':dr.total_cruise_bus_power_W,'classification':cls});v+=dv
    lift=lift_system_sizing(a,mass_kg=mass.takeoff.total_mass_kg,prop_diameter_in=float(d['lift_propeller_diameter_in']))
    return {'mass_kg':mass.takeoff.total_mass_kg,'hover':{'speed_mps':0.0,'design_thrust_per_motor_N':lift.design_thrust_per_motor_N,'design_total_electrical_power_W':lift.total_design_power_with_margin_W},'rows':rows}

def write_flight_envelope(result:Mapping[str,Any],out_dir:str|Path)->None:
    out=Path(out_dir);out.mkdir(parents=True,exist_ok=True);(out/'flight_envelope.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    rows=result['rows'];
    if rows:
        with (out/'flight_envelope.csv').open('w',newline='',encoding='utf-8') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
