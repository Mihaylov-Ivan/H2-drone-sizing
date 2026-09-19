from __future__ import annotations

"""Measured linear-mass and hardcode mass model for H2AIR v3.

The user can weigh printed/tube parts and enter kg/m.  The model derives mass
from actual geometric length.  Any rule can instead be locked with
``mass_mode='hardcoded'`` and ``hardcoded_mass_kg``.

Carbon tubes may use ``mass_mode='carbon_density'`` (or ``structural_linear`` with
design OD/ID present) and ρ=1575 kg/m³ by default.
"""

from copy import deepcopy
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .mass_properties import Component, Shape
from .landing_legs import (
    apply_landing_leg_layout,
    resolve_landing_leg_layout,
    sleeve_mass_kg,
    sleeve_od_m,
)
from .structures import carbon_tube_mass_kg

DEFAULT_CFRP_DENSITY_KG_M3 = 1575.0
_PI = 3.141592653589793


def _tube_od_m(rule: Mapping[str, Any], default: float = 0.0) -> float:
    """OD used for ledger geometry.

    Prefer ``design_tube_od_m`` (COTS/hardcoded design section), then
    ``reference_tube_od_m`` (kg/m calibration), then legacy ``tube_od_m``.
    """
    if rule.get("design_tube_od_m") is not None:
        return float(rule["design_tube_od_m"])
    if rule.get("reference_tube_od_m") is not None:
        return float(rule["reference_tube_od_m"])
    if rule.get("tube_od_m") is not None:
        return float(rule["tube_od_m"])
    return float(default)


def _tube_id_m(rule: Mapping[str, Any], od_m: Optional[float] = None) -> Optional[float]:
    if rule.get("design_tube_id_m") is not None:
        return float(rule["design_tube_id_m"])
    if rule.get("tube_id_m") is not None:
        return float(rule["tube_id_m"])
    if rule.get("reference_tube_id_m") is not None:
        return float(rule["reference_tube_id_m"])
    od = float(od_m) if od_m is not None else (
        float(rule["design_tube_od_m"]) if rule.get("design_tube_od_m") is not None else None
    )
    if od is not None and rule.get("id_over_od") is not None:
        return float(od) * float(rule["id_over_od"])
    return None


def _cfrp_density(cfg: Mapping[str, Any], rule: Mapping[str, Any]) -> float:
    if rule.get("density_kg_m3") is not None:
        return float(rule["density_kg_m3"])
    mats = (cfg.get("structures") or {}).get("materials") or {}
    cfrp = mats.get("cfrp_tube") or {}
    if cfrp.get("density_kg_m3") is not None:
        return float(cfrp["density_kg_m3"])
    return DEFAULT_CFRP_DENSITY_KG_M3


def _mass(
    rule: Mapping[str, Any],
    length_m: float,
    count: int = 1,
    *,
    cfg: Mapping[str, Any] | None = None,
    area_m2: float | None = None,
) -> float:
    mode = str(rule.get("mass_mode", "linear")).lower()
    if mode == "hardcoded":
        v = rule.get("hardcoded_mass_kg")
        if v is None or float(v) < 0:
            raise ValueError("hardcoded mass_mode requires hardcoded_mass_kg >= 0")
        return float(v)
    if mode in {"areal", "planform", "planform_areal"}:
        if area_m2 is None:
            raise ValueError(f"{mode} mass_mode requires area_m2")
        kgm2 = float(rule.get("specific_mass_kg_per_m2", rule.get("specific_mass_kg_per_m", 0.0)))
        if kgm2 < 0:
            raise ValueError("specific_mass_kg_per_m2 cannot be negative")
        return kgm2 * float(area_m2) * int(count)
    if mode == "carbon_density":
        od = _tube_od_m(rule)
        id_m = _tube_id_m(rule, od)
        if od <= 0 or id_m is None:
            raise ValueError("carbon_density mass_mode requires design_tube_od_m and design_tube_id_m (or od/id)")
        dens = _cfrp_density(cfg or {}, rule)
        return carbon_tube_mass_kg(od, float(id_m), float(length_m), int(count), dens)
    if mode not in {"linear", "structural_linear"}:
        raise ValueError(f"Unsupported mass_mode {mode!r}")
    # structural_linear: prefer density recompute when design OD/ID are known.
    if mode == "structural_linear":
        od = None
        if rule.get("design_tube_od_m") is not None:
            od = float(rule["design_tube_od_m"])
        id_m = _tube_id_m(rule, od)
        if od is not None and id_m is not None and od > id_m >= 0:
            dens = _cfrp_density(cfg or {}, rule)
            return carbon_tube_mass_kg(od, float(id_m), float(length_m), int(count), dens)
    kgm = float(rule.get("specific_mass_kg_per_m", 0.0))
    if kgm < 0:
        raise ValueError("specific_mass_kg_per_m cannot be negative")
    return kgm * float(length_m) * int(count)


def _status(rule: Mapping[str, Any]) -> str:
    return str(rule.get("status", "engineering_assumption"))


def _source(rule: Mapping[str, Any]) -> str:
    return str(rule.get("source", "User manufacturing mass model"))


def _add_box(out, *, name, group, mass, pos, dims, rule, package_zone=None, collide=False):
    out.append(Component(
        name=name, group=group, mass_takeoff_kg=mass, mass_end_kg=mass,
        position_m=tuple(float(x) for x in pos), shape=Shape.BOX.value,
        dimensions_m={"length_x": float(dims[0]), "width_y": float(dims[1]), "height_z": float(dims[2])},
        package_zone=package_zone, status=_status(rule), source=_source(rule),
        note=f"v3 manufacturing mass mode={rule.get('mass_mode','linear')}; "
             f"specific_mass_kg_per_m={rule.get('specific_mass_kg_per_m')}; "
             f"specific_mass_kg_per_m2={rule.get('specific_mass_kg_per_m2')}",
        include_in_collision_check=collide,
    ))


def _add_tube(out, *, name, group, mass, pos, length, diameter, axis, rule):
    shape = {"x": Shape.CYLINDER_X.value, "y": Shape.CYLINDER_Y.value, "z": Shape.CYLINDER_Z.value}[axis]
    out.append(Component(
        name=name, group=group, mass_takeoff_kg=mass, mass_end_kg=mass,
        position_m=tuple(float(x) for x in pos), shape=shape,
        dimensions_m={"length": float(length), "diameter": float(max(diameter, 1e-4))},
        status=_status(rule), source=_source(rule),
        note=f"v3 tube/member mass mode={rule.get('mass_mode','linear')}; specific_mass_kg_per_m={rule.get('specific_mass_kg_per_m')}",
        include_in_collision_check=False,
    ))


def _growth_factor_for_status(status: str) -> float:
    st = str(status or "").lower().strip()
    if st in {"supplier_confirmed", "measured", "database", "certified", "user_supplied_representative"} or "supplier" in st or "certified" in st or "measured" in st:
        return 1.01
    return 1.05


def _set_component_mass(by: Dict[str, Component], name: str, mass: float, *, diameter: float | None = None, length: float | None = None, apply_growth: bool = False) -> None:
    if name not in by:
        return
    m = float(mass)
    if apply_growth:
        m *= _growth_factor_for_status(by[name].status)
    by[name].mass_takeoff_kg = m
    by[name].mass_end_kg = m
    dims = dict(by[name].dimensions_m or {})
    if diameter is not None:
        dims["diameter"] = float(diameter)
    if length is not None:
        dims["length"] = float(length)
    if dims:
        by[name].dimensions_m = dims


def check_manufacturing_ledger_consistency(
    components: Sequence[Component],
    ledger: Sequence[Mapping[str, Any]],
    tol_kg: float = 0.001,
) -> Tuple[bool, float, str]:
    """Compare ledger mass_kg totals vs matching component takeoff masses."""
    if not ledger:
        return True, 0.0, "No manufacturing ledger entries"
    ledger_sum = sum(float(e.get("mass_kg", 0.0) or 0.0) for e in ledger)
    names: List[str] = []
    for e in ledger:
        n = str(e.get("name", ""))
        # Ledger aggregates may map to one or more component names.
        aliases = e.get("component_names")
        if isinstance(aliases, (list, tuple)) and aliases:
            names.extend(str(x) for x in aliases)
        else:
            names.append(n)
    by = {c.name: c for c in components}
    comp_sum = 0.0
    matched = 0
    for n in names:
        if n in by:
            comp_sum += float(by[n].mass_takeoff_kg)
            matched += 1
        else:
            # Aggregate ledger rows (e.g. boom_tubes) — sum prefix matches.
            prefix_hits = [c for c in components if c.name == n or c.name.startswith(n.rstrip("s") + "_") or c.name.startswith(n + "_")]
            if n.endswith("s") and not prefix_hits:
                stem = n[:-1]
                prefix_hits = [c for c in components if c.name.startswith(stem)]
            for c in prefix_hits:
                comp_sum += float(c.mass_takeoff_kg)
                matched += 1
    # Prefer explicit component_names when present; otherwise compare ledger sum to
    # sum of components whose names appear in ledger entries (direct or split).
    delta = abs(ledger_sum - comp_sum) if matched else 0.0
    # Direct path: sum ledger mass vs sum of all manufacturing-group structural masses is noisy.
    # Use per-entry reconciliation when component_names are provided; else ledger vs named hits.
    ok = delta <= float(tol_kg)
    msg = (
        f"Manufacturing ledger sum {ledger_sum:.4f} kg vs matched components {comp_sum:.4f} kg "
        f"(delta {delta:.4f} kg, tol {tol_kg:.4f} kg)"
    )
    return ok, delta, msg


def _add_stepped_wing_primary_spar(
    out: list,
    ledger: list,
    *,
    cfg: Mapping[str, Any],
    rule: Mapping[str, Any],
    span_m: float,
    spar_x: float,
    wing_z: float,
) -> None:
    """Split full-span primary spar into inboard/outboard tubes + two joiners."""
    frac = float(rule.get("joiner_span_fraction", 0.45))
    frac = min(max(frac, 0.05), 0.95)
    inb = rule.get("inboard") or {}
    outb = rule.get("outboard") or {}
    in_od = float(inb.get("design_tube_od_m", rule.get("design_tube_od_m", 0.028)))
    in_id = float(inb.get("design_tube_id_m", rule.get("design_tube_id_m", in_od * 0.75)))
    out_od = float(outb.get("design_tube_od_m", 0.016))
    out_id = float(outb.get("design_tube_id_m", out_od * 0.75))
    dens = _cfrp_density(cfg, rule)
    L_in = frac * float(span_m)
    L_out = (1.0 - frac) * float(span_m)
    m_in = carbon_tube_mass_kg(in_od, in_id, L_in, 1, dens)
    m_out = carbon_tube_mass_kg(out_od, out_id, L_out, 1, dens)
    joiner_each = float(rule.get("joiner_mass_kg_each", 0.06))
    joiner_rule = {
        "mass_mode": "hardcoded",
        "hardcoded_mass_kg": joiner_each,
        "status": _status(rule),
        "source": _source(rule),
    }
    y_join = 0.5 * frac * float(span_m)
    _add_tube(
        out, name="main_wing_primary_spar_inboard", group="wing_structure", mass=m_in,
        pos=[spar_x, 0.0, wing_z], length=L_in, diameter=in_od, axis="y", rule=rule,
    )
    _add_tube(
        out, name="main_wing_primary_spar_outboard", group="wing_structure", mass=m_out,
        pos=[spar_x, 0.0, wing_z], length=L_out, diameter=out_od, axis="y", rule=rule,
    )
    _add_box(
        out, name="main_wing_spar_joiner_left", group="wing_structure", mass=joiner_each,
        pos=[spar_x, -y_join, wing_z], dims=[max(in_od, 0.02), 0.04, max(in_od, 0.02)], rule=joiner_rule,
    )
    _add_box(
        out, name="main_wing_spar_joiner_right", group="wing_structure", mass=joiner_each,
        pos=[spar_x, y_join, wing_z], dims=[max(in_od, 0.02), 0.04, max(in_od, 0.02)], rule=joiner_rule,
    )
    total = m_in + m_out + 2.0 * joiner_each
    ledger.append({
        "name": "main_wing_primary_spar",
        "length_m": float(span_m),
        "count": 1,
        "mass_mode": "carbon_density_stepped",
        "specific_mass_kg_per_m": None,
        "mass_kg": total,
        "hardcoded": False,
        "component_names": [
            "main_wing_primary_spar_inboard",
            "main_wing_primary_spar_outboard",
            "main_wing_spar_joiner_left",
            "main_wing_spar_joiner_right",
        ],
        "joiner_span_fraction": frac,
        "inboard_od_m": in_od,
        "outboard_od_m": out_od,
    })


def apply_manufacturing_mass_model(cfg: Mapping[str, Any], components: Sequence[Component]) -> Tuple[list[Component], list[Dict[str, Any]]]:
    mm = cfg.get("manufacturing_mass_model")
    if not mm or not bool(mm.get("enabled", False)):
        return deepcopy(list(components)), []

    d = cfg["design"]
    gm = cfg["geometry_mass"]["geometry_model"]["geometry"]
    out = [deepcopy(c) for c in components]

    # Remove old aggregate structural masses that are replaced by the detailed
    # manufacturing ledger. Other hardware remains untouched.
    replaced = {
        "central_pod_structure", "wing_structure", "horizontal_tail_structure",
        "vertical_tail_left", "vertical_tail_right", "boom_tube_left", "boom_tube_right",
        "main_wing_primary_spar", "main_wing_primary_spar_inboard", "main_wing_primary_spar_outboard",
        "main_wing_spar_joiner_left", "main_wing_spar_joiner_right",
    }
    out = [c for c in out if c.name not in replaced]
    ledger: list[Dict[str, Any]] = []

    # Fuselage printed sections.
    x0 = 0.0
    max_d = float(d["fuselage_max_diameter_m"])
    fuse_w = float(d.get("fuselage_width_m", max_d))
    fuse_h = float(d.get("fuselage_height_m", max_d))
    for sec in mm["fuselage_sections"]:
        L = float(sec["length_m"])
        dia = float(sec["diameter_m"])
        w = float(sec.get("width_m", fuse_w if fuse_w > 0 else dia))
        h = float(sec.get("height_m", fuse_h if fuse_h > 0 else dia))
        rule = sec["mass"]
        m = _mass(rule, L, cfg=cfg)
        name = f"fuselage_{sec['name']}_shell"
        _add_box(out, name=name, group="fuselage_structure", mass=m,
                 pos=[x0 + L/2, 0, 0], dims=[L, w, h], rule=rule, collide=False)
        ledger.append({"name":name,"length_m":L,"count":1,"mass_mode":rule.get("mass_mode"),"specific_mass_kg_per_m":rule.get("specific_mass_kg_per_m"),"mass_kg":m,"hardcoded":rule.get("mass_mode")=="hardcoded","component_names":[name]})
        x0 += L

    # Two full-length CFRP fuselage longerons: top and bottom centreline.
    longr = mm["fuselage_longerons"]
    Lfus = float(mm["fuselage_total_length_m"]); nlong = int(longr.get("count",2)); mlong = _mass(longr,Lfus,nlong,cfg=cfg)
    Rfuse = 0.5 * float(d.get("fuselage_height_m", d.get("fuselage_max_diameter_m", 0.18)))
    long_od = float(longr.get("tube_od_m", 0.006))
    stations = longr.get("stations_yz_m")
    if not stations:
        r_axis = max(Rfuse - 0.5 * long_od, 0.5 * Rfuse)
        stations = [[0.0, r_axis], [0.0, -r_axis]]
    labels = ("top", "bottom")
    long_names = []
    for i, (y_l, z_l) in enumerate(stations[:nlong]):
        label = labels[i] if i < len(labels) else f"{i}"
        name = f"fuselage_longeron_{label}"
        long_names.append(name)
        _add_tube(out,name=name,group="fuselage_structure",mass=mlong/max(nlong,1),
                  pos=[Lfus/2,float(y_l),float(z_l)],length=Lfus,diameter=long_od,axis="x",rule=longr)
    ledger.append({"name":"fuselage_longerons","length_m":Lfus,"count":nlong,"mass_mode":longr.get("mass_mode"),"specific_mass_kg_per_m":longr.get("specific_mass_kg_per_m"),"mass_kg":mlong,"hardcoded":longr.get("mass_mode")=="hardcoded","component_names":long_names})

    # Main wing shell and two spanwise spars.
    mw = mm["main_wing"]
    b=float(d["wing_span_m"]); mac=float(d["wing_mac_m"]); xac=float(d["wing_ac_x_m"])
    S=float(d["wing_area_m2"])
    le=xac-0.25*mac
    shell_x=le+float(mw.get("shell_cg_chord_fraction",0.45))*mac
    mshell=_mass(mw["shell"],b,cfg=cfg,area_m2=S)
    _add_box(out,name="main_wing_shell",group="wing_structure",mass=mshell,pos=[shell_x,0,float(mw.get("z_m",0.10))],dims=[mac,b,float(mw.get("thickness_m",0.028))],rule=mw["shell"],collide=False)
    ledger.append({"name":"main_wing_shell","length_m":b,"area_m2":S,"count":1,"mass_mode":mw["shell"].get("mass_mode"),"specific_mass_kg_per_m":mw["shell"].get("specific_mass_kg_per_m"),"specific_mass_kg_per_m2":mw["shell"].get("specific_mass_kg_per_m2"),"mass_kg":mshell,"hardcoded":mw["shell"].get("mass_mode")=="hardcoded","component_names":["main_wing_shell"]})
    for label in ("primary_spar","secondary_spar"):
        rule=mw[label]; frac=float(rule.get("chord_fraction",0.25 if label=="primary_spar" else 0.65)); x=le+frac*mac
        if label == "primary_spar" and bool(rule.get("stepped")) and str(rule.get("mass_mode","linear")).lower() != "hardcoded":
            _add_stepped_wing_primary_spar(
                out, ledger, cfg=cfg, rule=rule, span_m=b, spar_x=x, wing_z=float(mw.get("z_m", 0.10)),
            )
            continue
        m=_mass(rule,b,cfg=cfg)
        _add_tube(out,name=f"main_wing_{label}",group="wing_structure",mass=m,pos=[x,0,float(mw.get("z_m",0.10))],length=b,diameter=_tube_od_m(rule,0.02),axis="y",rule=rule)
        ledger.append({"name":f"main_wing_{label}","length_m":b,"count":1,"mass_mode":rule.get("mass_mode"),"specific_mass_kg_per_m":rule.get("specific_mass_kg_per_m"),"mass_kg":m,"hardcoded":rule.get("mass_mode")=="hardcoded","component_names":[f"main_wing_{label}"]})

    # Horizontal tail shell and two spars.
    ht=mm["horizontal_tail"]; bh=float(d["horizontal_tail_span_m"]); hmac=float(d["horizontal_tail_mac_m"]); xh=float(d["horizontal_tail_ac_x_m"]); zh=float(ht.get("z_m",0.03))
    Sh=float(d.get("horizontal_tail_area_m2", bh * hmac))
    mh=_mass(ht["shell"],bh,cfg=cfg,area_m2=Sh)
    _add_box(out,name="horizontal_tail_shell",group="tail_structure",mass=mh,pos=[xh,0,zh],dims=[hmac,bh,float(ht.get("thickness_m",0.022))],rule=ht["shell"],collide=False)
    ledger.append({"name":"horizontal_tail_shell","length_m":bh,"area_m2":Sh,"count":1,"mass_mode":ht["shell"].get("mass_mode"),"specific_mass_kg_per_m":ht["shell"].get("specific_mass_kg_per_m"),"specific_mass_kg_per_m2":ht["shell"].get("specific_mass_kg_per_m2"),"mass_kg":mh,"hardcoded":ht["shell"].get("mass_mode")=="hardcoded","component_names":["horizontal_tail_shell"]})
    hle=xh-0.25*hmac
    for label in ("primary_spar","secondary_spar"):
        rule=ht[label]; frac=float(rule.get("chord_fraction",0.25 if label=="primary_spar" else 0.65)); m=_mass(rule,bh,cfg=cfg)
        _add_tube(out,name=f"horizontal_tail_{label}",group="tail_structure",mass=m,pos=[hle+frac*hmac,0,zh],length=bh,diameter=_tube_od_m(rule,0.009),axis="y",rule=rule)
        ledger.append({"name":f"horizontal_tail_{label}","length_m":bh,"count":1,"mass_mode":rule.get("mass_mode"),"specific_mass_kg_per_m":rule.get("specific_mass_kg_per_m"),"mass_kg":m,"hardcoded":rule.get("mass_mode")=="hardcoded","component_names":[f"horizontal_tail_{label}"]})

    # Two vertical fins, each with two CFRP spars (roots on the boom waterline).
    vt=mm["vertical_tail"]; hv=float(d["vertical_tail_height_m"]); nfin=int(vt.get("count",2)); yabs=abs(float(gm.get("boom_center_y_m",[-0.35,0.35])[1])); xv=float(d.get("vertical_tail_ac_x_m",d["horizontal_tail_ac_x_m"])); vroot=float(d["vertical_tail_root_chord_m"])
    boom_z=float(gm.get("boom_z_m",0.0))
    Sv=float(d.get("vertical_tail_area_total_m2", nfin * hv * vroot * (1.0 + float(d.get("vertical_tail_taper_ratio", 0.6))) / 2.0))
    mvs=_mass(vt["shell"],hv,1,cfg=cfg,area_m2=Sv)
    vt_shell_names=[]
    for side,y in (("left",-yabs),("right",yabs)):
        name=f"vertical_tail_{side}_shell"; vt_shell_names.append(name)
        _add_box(out,name=name,group="tail_structure",mass=mvs/nfin,pos=[xv,y,boom_z+hv/2],dims=[vroot,float(vt.get("thickness_m",0.012)),hv],rule=vt["shell"],collide=False)
    ledger.append({"name":"vertical_tail_shells","length_m":hv,"area_m2":Sv,"count":nfin,"mass_mode":vt["shell"].get("mass_mode"),"specific_mass_kg_per_m":vt["shell"].get("specific_mass_kg_per_m"),"specific_mass_kg_per_m2":vt["shell"].get("specific_mass_kg_per_m2"),"mass_kg":mvs,"hardcoded":vt["shell"].get("mass_mode")=="hardcoded","component_names":vt_shell_names})
    for label in ("primary_spar","secondary_spar"):
        rule=vt[label]; mt=_mass(rule,hv,nfin,cfg=cfg)
        vt_names=[]
        for side,y in (("left",-yabs),("right",yabs)):
            name=f"vertical_tail_{side}_{label}"; vt_names.append(name)
            _add_tube(out,name=name,group="tail_structure",mass=mt/nfin,pos=[xv,y,boom_z+hv/2],length=hv,diameter=_tube_od_m(rule,0.009),axis="z",rule=rule)
        ledger.append({"name":f"vertical_tail_{label}s","length_m":hv,"count":nfin,"mass_mode":rule.get("mass_mode"),"specific_mass_kg_per_m":rule.get("specific_mass_kg_per_m"),"mass_kg":mt,"hardcoded":rule.get("mass_mode")=="hardcoded","component_names":vt_names})

    # Twin booms (through the wing structural station).
    br=mm["boom_tubes"]; Lb=float(gm["boom_x_end_m"])-float(gm["boom_x_start_m"]); mb=_mass(br,Lb,2,cfg=cfg); xmid=(float(gm["boom_x_start_m"])+float(gm["boom_x_end_m"]))/2
    boom_names=[]
    for side,y in (("left",-yabs),("right",yabs)):
        name=f"boom_tube_{side}"; boom_names.append(name)
        _add_tube(out,name=name,group="tail_structure",mass=mb/2,pos=[xmid,y,boom_z],length=Lb,diameter=_tube_od_m(br,float(gm.get("boom_outer_diameter_m",0.035))),axis="x",rule=br)
    ledger.append({"name":"boom_tubes","length_m":Lb,"count":2,"mass_mode":br.get("mass_mode"),"specific_mass_kg_per_m":br.get("specific_mass_kg_per_m"),"mass_kg":mb,"hardcoded":br.get("mass_mode")=="hardcoded","component_names":boom_names})

    # Landing legs: CFRP tubes inside 2 mm printed sleeves (nose + two wing).
    apply_landing_leg_layout(cfg)
    layout = resolve_landing_leg_layout(cfg)
    if layout:
        legs_cfg = mm["landing_legs"]
        tube_rule = legs_cfg.get("tube") or {}
        sleeve_wall = float(layout["sleeve_wall_m"])
        dens = float(layout["printed_density_kg_m3"])
        tube_od = _tube_od_m(tube_rule, float(layout["tube_od_m"]))
        sleeve_od = sleeve_od_m(tube_od, sleeve_wall)
        sleeve_rule = {
            "mass_mode": "linear",
            "status": legs_cfg.get("status", "engineering_assumption"),
            "source": legs_cfg.get("source", "Printed landing-leg sleeve around CFRP tube"),
            "specific_mass_kg_per_m": dens * (_PI / 4.0) * (sleeve_od**2 - tube_od**2),
        }
        mt_total = 0.0
        ms_total = 0.0
        tube_names = []
        sleeve_names = []
        for leg in layout["legs"]:
            L = float(leg["length_m"])
            mt = _mass(tube_rule, L, cfg=cfg)
            ms = sleeve_mass_kg(
                tube_od_m=tube_od, sleeve_wall_m=sleeve_wall, length_m=L, density_kg_m3=dens, count=1,
            )
            mt_total += mt
            ms_total += ms
            pos = [float(leg["x_m"]), float(leg["y_m"]), float(leg["cg_z_m"])]
            tname = f"landing_leg_{leg['name']}_tube"
            sname = f"landing_leg_{leg['name']}_sleeve"
            tube_names.append(tname)
            sleeve_names.append(sname)
            _add_tube(
                out, name=tname, group="landing_gear", mass=mt,
                pos=pos, length=L, diameter=tube_od, axis="z", rule=tube_rule,
            )
            _add_tube(
                out, name=sname, group="landing_gear", mass=ms,
                pos=pos, length=L, diameter=sleeve_od, axis="z", rule=sleeve_rule,
            )
        ledger.append({
            "name": "landing_leg_tubes", "length_m": float(layout["sizing_length_m"]), "count": 3,
            "mass_mode": tube_rule.get("mass_mode"), "specific_mass_kg_per_m": tube_rule.get("specific_mass_kg_per_m"),
            "mass_kg": mt_total, "hardcoded": tube_rule.get("mass_mode") == "hardcoded",
            "component_names": tube_names,
        })
        ledger.append({
            "name": "landing_leg_sleeves", "length_m": float(layout["sizing_length_m"]), "count": 3,
            "mass_mode": "printed_annulus", "specific_mass_kg_per_m": sleeve_rule["specific_mass_kg_per_m"],
            "mass_kg": ms_total, "hardcoded": False, "component_names": sleeve_names,
        })

        # Minimum landing-gear mass floor as a fraction of MTOW ceiling.
        floor_frac = float(legs_cfg.get("minimum_mass_fraction_of_mtow", 0.0) or 0.0)
        if floor_frac > 0.0:
            floor_kg = floor_frac * float(d["mtow_ceiling_kg"])
            gear_mass = mt_total + ms_total
            shortfall = floor_kg - gear_mass
            if shortfall > 1e-9:
                margin_rule = {
                    "mass_mode": "hardcoded",
                    "hardcoded_mass_kg": shortfall,
                    "status": legs_cfg.get("status", "engineering_assumption"),
                    "source": "Landing-gear minimum mass floor vs mtow_ceiling",
                }
                xac = float(d["wing_ac_x_m"])
                _add_box(
                    out, name="landing_gear_margin", group="landing_gear", mass=shortfall,
                    pos=[xac, 0.0, float(mw.get("z_m", 0.10)) - 0.15],
                    dims=[0.05, 0.05, 0.05], rule=margin_rule,
                )
                ledger.append({
                    "name": "landing_gear_margin", "length_m": 0.0, "count": 1,
                    "mass_mode": "hardcoded", "specific_mass_kg_per_m": None,
                    "mass_kg": shortfall, "hardcoded": True,
                    "component_names": ["landing_gear_margin"],
                    "note": f"Floor {floor_frac:.0%} of mtow_ceiling ({floor_kg:.3f} kg)",
                })

    return out, ledger


def apply_structural_mass_updates(cfg: Mapping[str, Any], components: Sequence[Component], structure: Mapping[str, Any]) -> list[Component]:
    """Recompute structural tube masses from design OD/ID × length × ρ=1575.

    Hardcoded mass_mode entries remain fixed. Printed sleeves follow the sized tube OD.
    """
    mm=cfg.get("manufacturing_mass_model",{})
    if not mm or not bool(mm.get("enabled",False)):
        return deepcopy(list(components))
    out=deepcopy(list(components)); by={c.name:c for c in out}
    d=cfg["design"]
    gm=cfg["geometry_mass"]["geometry_model"]["geometry"]

    def skip_hardcoded(rule: Mapping[str, Any]) -> bool:
        return str(rule.get("mass_mode", "linear")).lower() == "hardcoded"

    # Wing primary spar (stepped or single).
    mw = mm.get("main_wing") or {}
    ps = mw.get("primary_spar") or {}
    b = float(d["wing_span_m"])
    if bool(ps.get("stepped")) and not skip_hardcoded(ps):
        frac = float(ps.get("joiner_span_fraction", 0.45))
        frac = min(max(frac, 0.05), 0.95)
        inb = ps.get("inboard") or {}
        outb = ps.get("outboard") or {}
        rec = structure["structural_checks"]["wing"]["recommended_screen"]["section"]
        # Root screen uses inboard; keep inboard OD/ID from rule (or recommended if larger).
        in_od = float(inb.get("design_tube_od_m", ps.get("design_tube_od_m", rec["od_m"])))
        in_id = float(inb.get("design_tube_id_m", ps.get("design_tube_id_m", rec["id_m"])))
        out_od = float(outb.get("design_tube_od_m", 0.016))
        out_id = float(outb.get("design_tube_id_m", out_od * 0.75))
        dens = _cfrp_density(cfg, ps)
        L_in = frac * b
        L_out = (1.0 - frac) * b
        _set_component_mass(by, "main_wing_primary_spar_inboard", carbon_tube_mass_kg(in_od, in_id, L_in, 1, dens), diameter=in_od, length=L_in, apply_growth=True)
        _set_component_mass(by, "main_wing_primary_spar_outboard", carbon_tube_mass_kg(out_od, out_id, L_out, 1, dens), diameter=out_od, length=L_out, apply_growth=True)
        # Joiners stay at configured mass (already grown if present from initial ledger).
        joiner = float(ps.get("joiner_mass_kg_each", 0.06)) * _growth_factor_for_status(ps.get("status", "engineering_assumption"))
        for jn in ("main_wing_spar_joiner_left", "main_wing_spar_joiner_right"):
            if jn in by:
                _set_component_mass(by, jn, joiner)
        ps["design_tube_od_m"] = in_od
        ps["design_tube_id_m"] = in_id
    elif not skip_hardcoded(ps):
        rec = structure["structural_checks"]["wing"]["recommended_screen"]["section"]
        od = float(rec["od_m"]); id_m = float(rec["id_m"])
        ps["design_tube_od_m"] = od
        ps["design_tube_id_m"] = id_m
        dens = _cfrp_density(cfg, ps)
        _set_component_mass(by, "main_wing_primary_spar", carbon_tube_mass_kg(od, id_m, b, 1, dens), diameter=od, length=b, apply_growth=True)

    # Horizontal / vertical primary spars.
    ht = mm.get("horizontal_tail") or {}
    hps = ht.get("primary_spar") or {}
    bh = float(d["horizontal_tail_span_m"])
    if not skip_hardcoded(hps):
        hrec = structure["structural_checks"]["tails"]["horizontal_recommended_screen"]["section"]
        hod, hid = float(hrec["od_m"]), float(hrec["id_m"])
        hps["design_tube_od_m"] = hod
        hps["design_tube_id_m"] = hid
        dens = _cfrp_density(cfg, hps)
        _set_component_mass(by, "horizontal_tail_primary_spar", carbon_tube_mass_kg(hod, hid, bh, 1, dens), diameter=hod, length=bh, apply_growth=True)

    vt = mm.get("vertical_tail") or {}
    vps = vt.get("primary_spar") or {}
    hv = float(d["vertical_tail_height_m"])
    if not skip_hardcoded(vps):
        vrec = structure["structural_checks"]["tails"]["vertical_recommended_screen"]["section"]
        vod, vid = float(vrec["od_m"]), float(vrec["id_m"])
        vps["design_tube_od_m"] = vod
        vps["design_tube_id_m"] = vid
        dens = _cfrp_density(cfg, vps)
        for n in ("vertical_tail_left_primary_spar", "vertical_tail_right_primary_spar"):
            _set_component_mass(by, n, carbon_tube_mass_kg(vod, vid, hv, 1, dens), diameter=vod, length=hv, apply_growth=True)

    # Booms.
    br = mm.get("boom_tubes") or {}
    Lb = float(gm["boom_x_end_m"]) - float(gm["boom_x_start_m"])
    if not skip_hardcoded(br):
        brec = structure["structural_checks"]["booms"]["recommended_screen"]["section"]
        bod, bid = float(brec["od_m"]), float(brec["id_m"])
        br["design_tube_od_m"] = bod
        br["design_tube_id_m"] = bid
        dens = _cfrp_density(cfg, br)
        for n in ("boom_tube_left", "boom_tube_right"):
            _set_component_mass(by, n, carbon_tube_mass_kg(bod, bid, Lb, 1, dens), diameter=bod, length=Lb, apply_growth=True)

    # Landing-leg tubes + sleeves.
    legs_cfg = mm.get("landing_legs") or {}
    land = structure.get("structural_checks", {}).get("landing_legs") or {}
    if legs_cfg.get("enabled") and land.get("enabled"):
        tube_rule = legs_cfg.get("tube") or {}
        if not skip_hardcoded(tube_rule):
            lsec = land["recommended_screen"]["section"]
            lod = float(lsec["od_m"]); lid = float(lsec["id_m"])
            tube_rule["design_tube_od_m"] = lod
            tube_rule["design_tube_id_m"] = lid
            dens = _cfrp_density(cfg, tube_rule)
            layout = resolve_landing_leg_layout(cfg)
            if layout:
                wall = float(layout["sleeve_wall_m"])
                print_dens = float(layout["printed_density_kg_m3"])
                sod = sleeve_od_m(lod, wall)
                by_layout = {leg["name"]: leg for leg in layout["legs"]}
                for name, leg_key in (
                    ("landing_leg_nose_tube", "nose"),
                    ("landing_leg_wing_left_tube", "wing_left"),
                    ("landing_leg_wing_right_tube", "wing_right"),
                ):
                    leg = by_layout.get(leg_key)
                    if not leg:
                        continue
                    L = float(leg["length_m"])
                    _set_component_mass(
                        by, name, carbon_tube_mass_kg(lod, lid, L, 1, dens),
                        diameter=lod, length=L, apply_growth=True,
                    )
                for name, leg_key in (
                    ("landing_leg_nose_sleeve", "nose"),
                    ("landing_leg_wing_left_sleeve", "wing_left"),
                    ("landing_leg_wing_right_sleeve", "wing_right"),
                ):
                    leg = by_layout.get(leg_key)
                    if not leg or name not in by:
                        continue
                    L = float(leg["length_m"])
                    _set_component_mass(
                        by, name,
                        sleeve_mass_kg(tube_od_m=lod, sleeve_wall_m=wall, length_m=L, density_kg_m3=print_dens, count=1),
                        diameter=sod, length=L, apply_growth=True,
                    )
                layout["tube_od_m"] = lod
                layout["tube_id_m"] = lid
                layout["sleeve_od_m"] = sod
                legs_cfg["resolved"] = layout
                cfg["geometry_mass"]["geometry_model"]["geometry"]["landing_legs"] = layout
                from .landing_legs import sync_landing_legs_drag
                sync_landing_legs_drag(cfg)

                # Re-apply landing-gear mass floor after structural resize.
                floor_frac = float(legs_cfg.get("minimum_mass_fraction_of_mtow", 0.0) or 0.0)
                if floor_frac > 0.0:
                    floor_kg = floor_frac * float(d["mtow_ceiling_kg"])
                    gear = sum(c.mass_takeoff_kg for c in out if c.group == "landing_gear" and c.name != "landing_gear_margin")
                    shortfall = floor_kg - gear
                    if "landing_gear_margin" in by:
                        _set_component_mass(by, "landing_gear_margin", max(0.0, shortfall))
                    elif shortfall > 1e-9:
                        out.append(Component(
                            name="landing_gear_margin", group="landing_gear",
                            mass_takeoff_kg=shortfall, mass_end_kg=shortfall,
                            position_m=(float(d["wing_ac_x_m"]), 0.0, float(mw.get("z_m", 0.10)) - 0.15),
                            shape=Shape.BOX.value,
                            dimensions_m={"length_x": 0.05, "width_y": 0.05, "height_z": 0.05},
                            status=str(legs_cfg.get("status", "engineering_assumption")),
                            source="Landing-gear minimum mass floor vs mtow_ceiling",
                            include_in_collision_check=False,
                        ))
                        by = {c.name: c for c in out}

    return out
