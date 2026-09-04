from __future__ import annotations
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence
import csv, json
from .optimization import screening_evaluate, default_space_for_config

OUTPUTS=("post_structure_mass_kg","static_margin","cruise_bus_power_W","mission_energy_Wh","max_motor_15kg_N")

def local_sensitivity(cfg:Mapping[str,Any],design_values:Mapping[str,float],*,fraction_of_range:float=0.03,space:Mapping[str,Sequence[float]]|None=None)->Dict[str,Any]:
    sp=dict(space or default_space_for_config(cfg)); base=screening_evaluate(cfg,design_values,index=0); rows=[]
    for name,(lo,hi) in sp.items():
        x=float(design_values[name]); dx=fraction_of_range*(float(hi)-float(lo));
        xm=max(float(lo),x-dx); xp=min(float(hi),x+dx)
        vm=dict(design_values);vp=dict(design_values);vm[name]=xm;vp[name]=xp
        cm=screening_evaluate(cfg,vm);cp=screening_evaluate(cfg,vp)
        row={"variable":name,"baseline":x,"minus":xm,"plus":xp}
        for k in OUTPUTS:
            deriv=(float(cp.metrics[k])-float(cm.metrics[k]))/max(xp-xm,1e-12)
            row[f"d_{k}_d_x"]=deriv
            denom=max(abs(float(base.metrics[k])),1e-12)
            row[f"normalized_{k}"]=deriv*x/denom if abs(x)>1e-12 else deriv*(hi-lo)/denom
        rows.append(row)
    return {"method":"central finite difference around optimized screening point","fraction_of_range":fraction_of_range,"baseline_metrics":base.metrics,"rows":rows}

def write_sensitivity_outputs(result:Mapping[str,Any],out_dir:str|Path)->None:
    out=Path(out_dir);out.mkdir(parents=True,exist_ok=True);(out/"sensitivity_results.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    rows=result["rows"]
    if rows:
        keys=list(rows[0]);
        with (out/"sensitivity.csv").open("w",newline="",encoding="utf-8") as f:
            w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
