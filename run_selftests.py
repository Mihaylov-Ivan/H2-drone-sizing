from pathlib import Path
import tempfile,traceback,json
from h2air.config import load_model_config
from h2air.pipeline import run_model
from h2air.optimization import screening_evaluate
ROOT=Path(__file__).resolve().parent

def main():
    checks=[]
    try:
        c=load_model_config(ROOT/'config/baseline.json');checks.append(('config',True,''))
        with tempfile.TemporaryDirectory() as td:
            r=run_model(c,Path(td),write_histories=False)
            checks += [
              ('baseline_reference_mass',10<r['summary']['reference_mass_kg']<13,''),
              ('structural_closure',r['coupled_closure']['converged'],''),
              ('baseline_geometry_problem_exposed',r['summary']['design_status']!='SCREENING_PASS',''),
              ('baseline_instability_detected',r['summary']['current_static_margin']<0,''),
              ('ems_executes',isinstance(r['hybrid_power']['feasible'],bool),''),
            ]
        oc=load_model_config(ROOT/'config/optimized_screening_case.json')
        keys=('wing_area_m2','wing_span_m','wing_ac_x_m','horizontal_tail_area_m2','lift_row_center_x_m','battery_x_m','battery_z_m','fuel_cell_x_m','fuel_cell_z_m','hydrogen_cylinder_x_m','hydrogen_cylinder_z_m')
        sc=screening_evaluate(c,{k:oc['design'][k] for k in keys});checks.append(('optimized_screening_candidate',sc.feasible,''))
        vr=json.load(open(ROOT/'outputs'/'nominal_reference'/'model_results.json'))
        checks += [('optimized_full_status',vr['summary']['design_status']=='SCREENING_PASS',''),('optimized_6dof',vr['sixdof_control']['current_layout']['gate_evaluation']['pass'],'')]
    except Exception:
        traceback.print_exc();return 1
    failed=[x for x in checks if not x[1]]
    for n,ok,msg in checks:print(('PASS' if ok else 'FAIL'),n,msg)
    print(f'{len(checks)-len(failed)}/{len(checks)} self-tests PASS')
    return 1 if failed else 0
if __name__=='__main__':raise SystemExit(main())
