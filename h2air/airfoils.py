from __future__ import annotations

"""Data-driven airfoil polar support for H2AIR.

Airfoil polars are ordinary CSV files with columns alpha, CL, CD and CM.
Additional XFOIL columns are preserved but are not required by the flight model.
The lookup intentionally clamps outside the supplied alpha range; extrapolation
past the source data is not silently invented.
"""

from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence
import csv


def _interp(x: float, x0: float, y0: float, x1: float, y1: float) -> float:
    if abs(x1-x0) < 1e-12:
        return float(y0)
    t=(x-x0)/(x1-x0)
    return float(y0+t*(y1-y0))


def load_polar_csv(path: str|Path) -> List[Dict[str,float]]:
    p=Path(path)
    rows=[]
    with p.open('r',encoding='utf-8-sig',newline='') as f:
        for raw in csv.DictReader(f):
            row={}
            for k,v in raw.items():
                if v is None or str(v).strip()=="":
                    continue
                row[k]=float(v)
            if not {'alpha','CL','CD'}.issubset(row):
                raise ValueError(f"Airfoil polar {p} requires alpha, CL and CD columns")
            row.setdefault('CM',0.0)
            rows.append(row)
    rows.sort(key=lambda r:r['alpha'])
    if len(rows)<2:
        raise ValueError(f"Airfoil polar {p} needs at least two rows")
    return rows


def interpolate_polar(rows: Sequence[Mapping[str,float]], alpha_deg: float) -> Dict[str,float]:
    a=float(alpha_deg)
    if a<=float(rows[0]['alpha']):
        out=dict(rows[0]); out['alpha_requested']=a; out['clamped']=True; return out
    if a>=float(rows[-1]['alpha']):
        out=dict(rows[-1]); out['alpha_requested']=a; out['clamped']=True; return out
    for lo,hi in zip(rows,rows[1:]):
        a0=float(lo['alpha']); a1=float(hi['alpha'])
        if a0<=a<=a1:
            keys=set(lo)|set(hi)
            out={'alpha':a,'alpha_requested':a,'clamped':False}
            for k in keys:
                if k=='alpha': continue
                if k in lo and k in hi:
                    out[k]=_interp(a,a0,float(lo[k]),a1,float(hi[k]))
            return out
    raise RuntimeError('airfoil interpolation failed')


def polar_summary(rows: Sequence[Mapping[str,float]]) -> Dict[str,float]:
    clmax=max(rows,key=lambda r:float(r['CL']))
    clmin=min(rows,key=lambda r:float(r['CL']))
    # zero-lift alpha from nearest sign crossing; fall back to minimum |CL| row.
    alpha0=None
    for lo,hi in zip(rows,rows[1:]):
        c0=float(lo['CL']); c1=float(hi['CL'])
        if c0==0:
            alpha0=float(lo['alpha']); break
        if c0*c1<=0 and c1!=c0:
            alpha0=_interp(0.0,c0,float(lo['alpha']),c1,float(hi['alpha'])); break
    if alpha0 is None:
        alpha0=float(min(rows,key=lambda r:abs(float(r['CL'])))['alpha'])
    z=interpolate_polar(rows,alpha0)
    return {
        'alpha_min_deg':float(rows[0]['alpha']), 'alpha_max_deg':float(rows[-1]['alpha']),
        'CL_max':float(clmax['CL']), 'alpha_at_CL_max_deg':float(clmax['alpha']),
        'CL_min':float(clmin['CL']), 'alpha_at_CL_min_deg':float(clmin['alpha']),
        'zero_lift_alpha_deg':float(alpha0), 'CD_at_zero_lift':float(z['CD']),
        'CM_at_zero_lift':float(z.get('CM',0.0)),
    }



def interpolate_polar_by_cl(rows: Sequence[Mapping[str,float]], cl_req: float, *, prefer_positive_alpha: bool=True) -> Dict[str,float]:
    """Interpolate a polar by CL on the pre-stall branch where possible."""
    target=float(cl_req)
    candidates=[]
    for lo,hi in zip(rows,rows[1:]):
        c0=float(lo['CL']); c1=float(hi['CL'])
        if (c0-target)*(c1-target)<=0 and abs(c1-c0)>1e-12:
            a=_interp(target,c0,float(lo['alpha']),c1,float(hi['alpha']))
            if prefer_positive_alpha and target>=0 and a < -1e-9:
                continue
            candidates.append(interpolate_polar(rows,a))
    if candidates:
        # Choose the lowest-drag branch if CL is multi-valued near stall.
        return min(candidates,key=lambda r:float(r['CD']))
    # Outside available CL range: return nearest CL row and mark clamped.
    r=dict(min(rows,key=lambda r:abs(float(r['CL'])-target)))
    r['CL_requested']=target; r['clamped']=True
    return r

def resolve_airfoil_entry(code: str, airfoil_db: Mapping[str,Any], *, root: Path) -> Dict[str,Any]:
    key=str(code).upper().replace(' ','')
    if key not in airfoil_db:
        raise ValueError(f"Airfoil {key!r} not found in airfoil database")
    e=dict(airfoil_db[key])
    e['code']=key
    polar_file=e.get('polar_file')
    if polar_file:
        path=root/str(polar_file)
        rows=load_polar_csv(path)
        e['polar_rows']=rows
        e['polar_summary']=polar_summary(rows)
        e['polar_path']=str(path)
        e['aero_source_mode']='polar_file'
    else:
        e['polar_rows']=None
        e['aero_source_mode']='screening_fallback'
    return e
