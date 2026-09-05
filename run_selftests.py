from pathlib import Path
import tempfile,traceback,json
from h2air.config import load_model_config
from h2air.pipeline import run_model,_components_with_power
from h2air.optimization import screening_evaluate,default_space_for_config
from h2air.airfoils import interpolate_polar
ROOT=Path(__file__).resolve().parent

def main():
    checks=[]
    try:
        c=load_model_config(ROOT/'config/user_inputs.json')
        checks.append(('v4_user_config',c['model']['version']=='H2AIR-integrated-v4.0',''))
        checks.append(('auto_main_span',c['design']['main_wing_span_mode']=='auto' and 'main_wing_span_m' in default_space_for_config(c),''))
        checks.append(('naca0012_polar_loaded',bool(c['airfoil_model']['horizontal_tail'].get('polar_rows')),''))
        row=interpolate_polar(c['airfoil_model']['horizontal_tail']['polar_rows'],0.0)
        checks.append(('naca0012_zero_alpha',abs(row['CL'])<1e-8 and abs(row['CD']-0.01020)<1e-6,''))
        sel=c.get('hardware_selection_result',{})
        checks.append(('hardware_auto_selection',sel.get('mode')=='auto' and bool(sel.get('selected_lift_combo')) and bool(sel.get('selected_pusher_combo')) and bool(sel.get('selected_battery')),''))
        comps=_components_with_power(c);by={x.name:x for x in comps}
        expected=float(c['manufacturing_mass_model']['fuselage_longerons']['specific_mass_kg_per_m'])*float(c['design']['fuselage_length_m'])*2
        mlong=by['fuselage_longeron_top'].mass_takeoff_kg+by['fuselage_longeron_bottom'].mass_takeoff_kg
        checks.append(('linear_mass',abs(mlong-expected)<1e-10,''))
        vals={k:c['design'][k] for k in default_space_for_config(c)}
        screen=screening_evaluate(c,vals)
        checks.append(('current_design_screening_feasible',screen.feasible,','.join(screen.violations)))
        with tempfile.TemporaryDirectory() as td:
            out=Path(td)
            r=run_model(c,out,write_histories=False)
            checks += [
              ('structural_closure',r['coupled_closure']['converged'],''),
              ('known_component_packaging',next(g for g in r['gates'] if g['name']=='packaging')['status']=='PASS',''),
              ('lift_prop_clearance',next(g for g in r['gates'] if g['name']=='lift_prop_clearance')['status']=='PASS',''),
              ('cad_json_written',(out/'cad_parameters.json').exists(),''),
              ('cad_xml_written',(out/'cad_parameters.xml').exists(),''),
              ('hardware_selection_written',(out/'HARDWARE_SELECTION.json').exists(),''),
            ]
            cad=json.loads((out/'cad_parameters.json').read_text())
            names=[p['name'].lower() for p in cad['parameters']]
            forbidden=('mass','weight','power','stress','mpa','energy','thrust','soc')
            checks.append(('cad_geometry_only',not any(any(f in n for f in forbidden) for n in names),''))
    except Exception:
        traceback.print_exc();return 1
    failed=[x for x in checks if not x[1]]
    for n,ok,msg in checks:print(('PASS' if ok else 'FAIL'),n,msg)
    print(f'{len(checks)-len(failed)}/{len(checks)} self-tests PASS')
    return 1 if failed else 0
if __name__=='__main__':raise SystemExit(main())
