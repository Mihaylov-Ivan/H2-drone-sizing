from __future__ import annotations

"""Measured linear-mass and hardcode mass model for H2AIR v3.

The user can weigh printed/tube parts and enter kg/m.  The model derives mass
from actual geometric length.  Any rule can instead be locked with
``mass_mode='hardcoded'`` and ``hardcoded_mass_kg``.
"""

from copy import deepcopy
from typing import Any, Dict, Mapping, Sequence, Tuple

from .mass_properties import Component, Shape
from .landing_legs import (
    apply_landing_leg_layout,
    resolve_landing_leg_layout,
    sleeve_mass_kg,
    sleeve_od_m,
)


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


def _mass(rule: Mapping[str, Any], length_m: float, count: int = 1) -> float:
    mode = str(rule.get("mass_mode", "linear")).lower()
    if mode == "hardcoded":
        v = rule.get("hardcoded_mass_kg")
        if v is None or float(v) < 0:
            raise ValueError("hardcoded mass_mode requires hardcoded_mass_kg >= 0")
        return float(v)
    if mode not in {"linear", "structural_linear"}:
        raise ValueError(f"Unsupported mass_mode {mode!r}")
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
        note=f"v3 manufacturing mass mode={rule.get('mass_mode','linear')}; specific_mass_kg_per_m={rule.get('specific_mass_kg_per_m')}",
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
        m = _mass(rule, L)
        name = f"fuselage_{sec['name']}_shell"
        _add_box(out, name=name, group="fuselage_structure", mass=m,
                 pos=[x0 + L/2, 0, 0], dims=[L, w, h], rule=rule, collide=False)
        ledger.append({"name":name,"length_m":L,"count":1,"mass_mode":rule.get("mass_mode"),"specific_mass_kg_per_m":rule.get("specific_mass_kg_per_m"),"mass_kg":m,"hardcoded":rule.get("mass_mode")=="hardcoded"})
        x0 += L

    # Two full-length CFRP fuselage longerons: top and bottom centreline.
    longr = mm["fuselage_longerons"]
    Lfus = float(mm["fuselage_total_length_m"]); nlong = int(longr.get("count",2)); mlong = _mass(longr,Lfus,nlong)
    Rfuse = 0.5 * float(d.get("fuselage_height_m", d.get("fuselage_max_diameter_m", 0.18)))
    long_od = float(longr.get("tube_od_m", 0.006))
    stations = longr.get("stations_yz_m")
    if not stations:
        r_axis = max(Rfuse - 0.5 * long_od, 0.5 * Rfuse)
        stations = [[0.0, r_axis], [0.0, -r_axis]]
    labels = ("top", "bottom")
    for i, (y_l, z_l) in enumerate(stations[:nlong]):
        label = labels[i] if i < len(labels) else f"{i}"
        _add_tube(out,name=f"fuselage_longeron_{label}",group="fuselage_structure",mass=mlong/max(nlong,1),
                  pos=[Lfus/2,float(y_l),float(z_l)],length=Lfus,diameter=long_od,axis="x",rule=longr)
    ledger.append({"name":"fuselage_longerons","length_m":Lfus,"count":nlong,"mass_mode":longr.get("mass_mode"),"specific_mass_kg_per_m":longr.get("specific_mass_kg_per_m"),"mass_kg":mlong,"hardcoded":longr.get("mass_mode")=="hardcoded"})

    # Main wing shell and two spanwise spars.
    mw = mm["main_wing"]
    b=float(d["wing_span_m"]); mac=float(d["wing_mac_m"]); xac=float(d["wing_ac_x_m"])
    le=xac-0.25*mac
    shell_x=le+float(mw.get("shell_cg_chord_fraction",0.45))*mac
    mshell=_mass(mw["shell"],b); _add_box(out,name="main_wing_shell",group="wing_structure",mass=mshell,pos=[shell_x,0,float(mw.get("z_m",0.10))],dims=[mac,b,float(mw.get("thickness_m",0.028))],rule=mw["shell"],collide=False)
    ledger.append({"name":"main_wing_shell","length_m":b,"count":1,"mass_mode":mw["shell"].get("mass_mode"),"specific_mass_kg_per_m":mw["shell"].get("specific_mass_kg_per_m"),"mass_kg":mshell,"hardcoded":mw["shell"].get("mass_mode")=="hardcoded"})
    for label in ("primary_spar","secondary_spar"):
        rule=mw[label]; frac=float(rule.get("chord_fraction",0.25 if label=="primary_spar" else 0.65)); x=le+frac*mac
        m=_mass(rule,b)
        _add_tube(out,name=f"main_wing_{label}",group="wing_structure",mass=m,pos=[x,0,float(mw.get("z_m",0.10))],length=b,diameter=_tube_od_m(rule,0.02),axis="y",rule=rule)
        ledger.append({"name":f"main_wing_{label}","length_m":b,"count":1,"mass_mode":rule.get("mass_mode"),"specific_mass_kg_per_m":rule.get("specific_mass_kg_per_m"),"mass_kg":m,"hardcoded":rule.get("mass_mode")=="hardcoded"})

    # Horizontal tail shell and two spars.
    ht=mm["horizontal_tail"]; bh=float(d["horizontal_tail_span_m"]); hmac=float(d["horizontal_tail_mac_m"]); xh=float(d["horizontal_tail_ac_x_m"]); zh=float(ht.get("z_m",0.03))
    mh=_mass(ht["shell"],bh); _add_box(out,name="horizontal_tail_shell",group="tail_structure",mass=mh,pos=[xh,0,zh],dims=[hmac,bh,float(ht.get("thickness_m",0.022))],rule=ht["shell"],collide=False)
    ledger.append({"name":"horizontal_tail_shell","length_m":bh,"count":1,"mass_mode":ht["shell"].get("mass_mode"),"specific_mass_kg_per_m":ht["shell"].get("specific_mass_kg_per_m"),"mass_kg":mh,"hardcoded":ht["shell"].get("mass_mode")=="hardcoded"})
    hle=xh-0.25*hmac
    for label in ("primary_spar","secondary_spar"):
        rule=ht[label]; frac=float(rule.get("chord_fraction",0.25 if label=="primary_spar" else 0.65)); m=_mass(rule,bh)
        _add_tube(out,name=f"horizontal_tail_{label}",group="tail_structure",mass=m,pos=[hle+frac*hmac,0,zh],length=bh,diameter=_tube_od_m(rule,0.009),axis="y",rule=rule)
        ledger.append({"name":f"horizontal_tail_{label}","length_m":bh,"count":1,"mass_mode":rule.get("mass_mode"),"specific_mass_kg_per_m":rule.get("specific_mass_kg_per_m"),"mass_kg":m,"hardcoded":rule.get("mass_mode")=="hardcoded"})

    # Two vertical fins, each with two CFRP spars (roots on the boom waterline).
    vt=mm["vertical_tail"]; hv=float(d["vertical_tail_height_m"]); nfin=int(vt.get("count",2)); yabs=abs(float(gm.get("boom_center_y_m",[-0.35,0.35])[1])); xv=float(d.get("vertical_tail_ac_x_m",d["horizontal_tail_ac_x_m"])); vroot=float(d["vertical_tail_root_chord_m"])
    boom_z=float(gm.get("boom_z_m",0.0))
    mvs=_mass(vt["shell"],hv,nfin)
    for side,y in (("left",-yabs),("right",yabs)):
        _add_box(out,name=f"vertical_tail_{side}_shell",group="tail_structure",mass=mvs/nfin,pos=[xv,y,boom_z+hv/2],dims=[vroot,float(vt.get("thickness_m",0.012)),hv],rule=vt["shell"],collide=False)
    ledger.append({"name":"vertical_tail_shells","length_m":hv,"count":nfin,"mass_mode":vt["shell"].get("mass_mode"),"specific_mass_kg_per_m":vt["shell"].get("specific_mass_kg_per_m"),"mass_kg":mvs,"hardcoded":vt["shell"].get("mass_mode")=="hardcoded"})
    for label in ("primary_spar","secondary_spar"):
        rule=vt[label]; mt=_mass(rule,hv,nfin)
        for side,y in (("left",-yabs),("right",yabs)):
            _add_tube(out,name=f"vertical_tail_{side}_{label}",group="tail_structure",mass=mt/nfin,pos=[xv,y,boom_z+hv/2],length=hv,diameter=_tube_od_m(rule,0.009),axis="z",rule=rule)
        ledger.append({"name":f"vertical_tail_{label}s","length_m":hv,"count":nfin,"mass_mode":rule.get("mass_mode"),"specific_mass_kg_per_m":rule.get("specific_mass_kg_per_m"),"mass_kg":mt,"hardcoded":rule.get("mass_mode")=="hardcoded"})

    # Twin booms (through the wing structural station).
    br=mm["boom_tubes"]; Lb=float(gm["boom_x_end_m"])-float(gm["boom_x_start_m"]); mb=_mass(br,Lb,2); xmid=(float(gm["boom_x_start_m"])+float(gm["boom_x_end_m"]))/2
    for side,y in (("left",-yabs),("right",yabs)):
        _add_tube(out,name=f"boom_tube_{side}",group="tail_structure",mass=mb/2,pos=[xmid,y,boom_z],length=Lb,diameter=_tube_od_m(br,float(gm.get("boom_outer_diameter_m",0.035))),axis="x",rule=br)
    ledger.append({"name":"boom_tubes","length_m":Lb,"count":2,"mass_mode":br.get("mass_mode"),"specific_mass_kg_per_m":br.get("specific_mass_kg_per_m"),"mass_kg":mb,"hardcoded":br.get("mass_mode")=="hardcoded"})

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
            "specific_mass_kg_per_m": dens * (3.141592653589793 / 4.0) * (sleeve_od**2 - tube_od**2),
        }
        mt_total = 0.0
        ms_total = 0.0
        for leg in layout["legs"]:
            L = float(leg["length_m"])
            mt = _mass(tube_rule, L)
            ms = sleeve_mass_kg(
                tube_od_m=tube_od, sleeve_wall_m=sleeve_wall, length_m=L, density_kg_m3=dens, count=1,
            )
            mt_total += mt
            ms_total += ms
            pos = [float(leg["x_m"]), float(leg["y_m"]), float(leg["cg_z_m"])]
            _add_tube(
                out, name=f"landing_leg_{leg['name']}_tube", group="landing_gear", mass=mt,
                pos=pos, length=L, diameter=tube_od, axis="z", rule=tube_rule,
            )
            _add_tube(
                out, name=f"landing_leg_{leg['name']}_sleeve", group="landing_gear", mass=ms,
                pos=pos, length=L, diameter=sleeve_od, axis="z", rule=sleeve_rule,
            )
        ledger.append({
            "name": "landing_leg_tubes", "length_m": float(layout["sizing_length_m"]), "count": 3,
            "mass_mode": tube_rule.get("mass_mode"), "specific_mass_kg_per_m": tube_rule.get("specific_mass_kg_per_m"),
            "mass_kg": mt_total, "hardcoded": tube_rule.get("mass_mode") == "hardcoded",
        })
        ledger.append({
            "name": "landing_leg_sleeves", "length_m": float(layout["sizing_length_m"]), "count": 3,
            "mass_mode": "printed_annulus", "specific_mass_kg_per_m": sleeve_rule["specific_mass_kg_per_m"],
            "mass_kg": ms_total, "hardcoded": False,
        })

    return out, ledger


def apply_structural_mass_updates(cfg: Mapping[str, Any], components: Sequence[Component], structure: Mapping[str, Any]) -> list[Component]:
    """Update only structural-linear members; hardcoded masses remain fixed."""
    mm=cfg.get("manufacturing_mass_model",{})
    if not mm or not bool(mm.get("enabled",False)):
        return deepcopy(list(components))
    out=deepcopy(list(components)); by={c.name:c for c in out}

    def scaled(name: str, rule: Mapping[str,Any], area_new: float, area_ref: float):
        if name not in by or str(rule.get("mass_mode","linear")).lower()=="hardcoded": return
        if str(rule.get("mass_mode","linear")).lower()!="structural_linear": return
        by[name].mass_takeoff_kg *= area_new/max(area_ref,1e-12); by[name].mass_end_kg=by[name].mass_takeoff_kg

    # Primary wing spar scales vs the reference section that kg/m was calibrated to.
    # Prefer structures.reference_wing_spar_*; legacy current_wing_spar_* still accepted.
    st=cfg["structures"]["structure"]
    _cur_od=st.get("reference_wing_spar_od_m", st.get("current_wing_spar_od_m"))
    _cur_id=st.get("reference_wing_spar_id_m", st.get("current_wing_spar_id_m"))
    if _cur_od is None or _cur_id is None:
        raise KeyError("structures.structure requires reference_wing_spar_od_m/id_m (or legacy current_wing_spar_*)")
    cur_od=float(_cur_od); cur_id=float(_cur_id)
    aref=3.141592653589793/4*(cur_od**2-cur_id**2)
    rec=structure["structural_checks"]["wing"]["recommended_screen"]["section"]
    anew=3.141592653589793/4*(float(rec["od_m"])**2-float(rec["id_m"])**2)
    scaled("main_wing_primary_spar",mm["main_wing"]["primary_spar"],anew,aref)

    # Horizontal and vertical primary spars use the model's configured tail reference area.
    href=float(st["tail_spar_reference_area_mm2"])*1e-6
    hrec=structure["structural_checks"]["tails"]["horizontal_recommended_screen"]["section"]
    hare=3.141592653589793/4*(float(hrec["od_m"])**2-float(hrec["id_m"])**2)
    scaled("horizontal_tail_primary_spar",mm["horizontal_tail"]["primary_spar"],hare,href)
    vrec=structure["structural_checks"]["tails"]["vertical_recommended_screen"]["section"]
    vare=3.141592653589793/4*(float(vrec["od_m"])**2-float(vrec["id_m"])**2)
    for n in ("vertical_tail_left_primary_spar","vertical_tail_right_primary_spar"):
        scaled(n,mm["vertical_tail"]["primary_spar"],vare,href)

    # Booms: scale the entered kg/m against the configured reference tube area.
    bref_od=float(st["reference_boom_od_m"]); bref_id=float(st["reference_boom_id_m"])
    bref=3.141592653589793/4*(bref_od**2-bref_id**2)
    brec=structure["structural_checks"]["booms"]["recommended_screen"]["section"]
    bare=3.141592653589793/4*(float(brec["od_m"])**2-float(brec["id_m"])**2)
    for n in ("boom_tube_left","boom_tube_right"):
        scaled(n,mm["boom_tubes"],bare,bref)

    # Landing-leg tubes: scale vs the kg/m calibration OD; sleeves follow design OD.
    legs_cfg = mm.get("landing_legs") or {}
    land = structure.get("structural_checks", {}).get("landing_legs") or {}
    if legs_cfg.get("enabled") and land.get("enabled"):
        tube_rule = legs_cfg.get("tube") or {}
        ref_od = float(tube_rule.get("reference_tube_od_m") or tube_rule.get("tube_od_m") or 0.012)
        # Reference wall uses the same id/od ratio as the design screen when ID unknown.
        id_ratio = float(tube_rule.get("id_over_od", 0.75))
        ref_id = float(tube_rule.get("reference_tube_id_m") or (ref_od * id_ratio))
        lref = 3.141592653589793 / 4.0 * (ref_od**2 - ref_id**2)
        lsec = land["recommended_screen"]["section"]
        lod = float(lsec["od_m"]); lid = float(lsec["id_m"])
        lare = 3.141592653589793 / 4.0 * (lod**2 - lid**2)
        tube_rule["design_tube_od_m"] = lod
        tube_rule["design_tube_id_m"] = lid
        for name in ("landing_leg_nose_tube", "landing_leg_wing_left_tube", "landing_leg_wing_right_tube"):
            scaled(name, tube_rule, lare, lref)
        # Refresh sleeve diameters/masses for the sized tube (2 mm wall annulus).
        layout = resolve_landing_leg_layout(cfg)
        if layout:
            wall = float(layout["sleeve_wall_m"])
            dens = float(layout["printed_density_kg_m3"])
            sod = sleeve_od_m(lod, wall)
            by_layout = {leg["name"]: leg for leg in layout["legs"]}
            for name, leg_key in (
                ("landing_leg_nose_sleeve", "nose"),
                ("landing_leg_wing_left_sleeve", "wing_left"),
                ("landing_leg_wing_right_sleeve", "wing_right"),
            ):
                if name not in by:
                    continue
                leg = by_layout[leg_key]
                by[name].mass_takeoff_kg = sleeve_mass_kg(
                    tube_od_m=lod, sleeve_wall_m=wall, length_m=float(leg["length_m"]),
                    density_kg_m3=dens, count=1,
                )
                by[name].mass_end_kg = by[name].mass_takeoff_kg
                by[name].dimensions_m = {"length": float(leg["length_m"]), "diameter": float(sod)}
            # Keep resolved layout OD in sync for CAD/viewer.
            layout["tube_od_m"] = lod
            layout["tube_id_m"] = lid
            layout["sleeve_od_m"] = sod
            legs_cfg["resolved"] = layout
            cfg["geometry_mass"]["geometry_model"]["geometry"]["landing_legs"] = layout
            from .landing_legs import sync_landing_legs_drag
            sync_landing_legs_drag(cfg)
    return out
