"""H2AIR Structures structural/load-case closure.

This module is intentionally screening-fidelity. It consumes the geometry and
control-allocation results established in Phases 3-6 and turns them into a
controlled structural load-case matrix. Strength and stiffness are kept
separate: a member can be strong enough and still be rejected for excessive
flexibility.

Coordinate convention for stations is the Phase-3 design frame:
  x positive aft from nose, y positive right, z positive upward.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from math import pi, sqrt, degrees
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

G = 9.80665
EPS = 1e-12


@dataclass(frozen=True)
class TubeSection:
    od_m: float
    id_m: float

    def validate(self) -> None:
        if self.od_m <= 0 or self.id_m < 0 or self.id_m >= self.od_m:
            raise ValueError('Tube requires OD > ID >= 0')

    @property
    def area_m2(self) -> float:
        self.validate()
        return pi / 4.0 * (self.od_m**2 - self.id_m**2)

    @property
    def I_m4(self) -> float:
        self.validate()
        return pi / 64.0 * (self.od_m**4 - self.id_m**4)

    @property
    def J_m4(self) -> float:
        self.validate()
        return pi / 32.0 * (self.od_m**4 - self.id_m**4)


@dataclass(frozen=True)
class Material:
    name: str
    E_Pa: float
    G_Pa: float
    density_kg_m3: float
    bending_allow_Pa: float
    shear_allow_Pa: float
    status: str = 'engineering_assumption'


@dataclass(frozen=True)
class LoadCase:
    name: str
    category: str
    mass_kg: float
    load_factor: float = 1.0
    wing_root_bending_Nm: float = 0.0
    wing_torsion_Nm: float = 0.0
    boom_root_bending_Nm: float = 0.0
    boom_axial_N: float = 0.0
    max_motor_thrust_N: float = 0.0
    max_motor_mount_bending_Nm: float = 0.0
    tail_force_N: float = 0.0
    note: str = ''

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class MemberCheck:
    member: str
    case: str
    bending_stress_MPa: float
    axial_stress_MPa: float
    combined_stress_MPa: float
    allowable_stress_MPa: float
    stress_utilization: float
    tip_deflection_mm: float
    deflection_limit_mm: float
    deflection_utilization: float
    root_slope_deg: float
    slope_limit_deg: float
    slope_utilization: float
    strength_pass: bool
    stiffness_pass: bool
    overall_pass: bool
    note: str = ''

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def bending_stress_Pa(moment_Nm: float, section: TubeSection) -> float:
    return abs(moment_Nm) * (section.od_m / 2.0) / max(section.I_m4, EPS)


def axial_stress_Pa(axial_N: float, section: TubeSection) -> float:
    return abs(axial_N) / max(section.area_m2, EPS)


def cantilever_udl_equivalent_deflection_m(moment_root_Nm: float, length_m: float, E_Pa: float, section: TubeSection) -> float:
    """Equivalent UDL cantilever tip deflection from root moment.

    For UDL: M_root = w L^2 / 2 and delta_tip = w L^4 / (8 EI),
    therefore delta = M_root L^2 / (4 EI).
    """
    return abs(moment_root_Nm) * length_m**2 / max(4.0 * E_Pa * section.I_m4, EPS)


def cantilever_udl_equivalent_root_slope_rad(moment_root_Nm: float, length_m: float, E_Pa: float, section: TubeSection) -> float:
    # For UDL: theta_tip = w L^3/(6EI) = M_root L/(3EI).
    return abs(moment_root_Nm) * length_m / max(3.0 * E_Pa * section.I_m4, EPS)


def torsion_shear_Pa(torque_Nm: float, section: TubeSection) -> float:
    return abs(torque_Nm) * (section.od_m / 2.0) / max(section.J_m4, EPS)


def torsion_twist_rad(torque_Nm: float, length_m: float, G_Pa: float, section: TubeSection) -> float:
    return abs(torque_Nm) * length_m / max(G_Pa * section.J_m4, EPS)


def check_tube_member(
    *, member: str, case: str, section: TubeSection, material: Material,
    length_m: float, bending_Nm: float, axial_N: float = 0.0,
    deflection_limit_fraction: float = 0.03, slope_limit_deg: float = 3.0,
    note: str = '',
) -> MemberCheck:
    sb = bending_stress_Pa(bending_Nm, section)
    sa = axial_stress_Pa(axial_N, section)
    # Conservative scalar interaction for screening; not laminate failure theory.
    sc = sb + sa
    d = cantilever_udl_equivalent_deflection_m(bending_Nm, length_m, material.E_Pa, section)
    slope = degrees(cantilever_udl_equivalent_root_slope_rad(bending_Nm, length_m, material.E_Pa, section))
    dlim = max(1e-6, deflection_limit_fraction * length_m)
    su = sc / max(material.bending_allow_Pa, EPS)
    du = d / dlim
    ru = slope / max(slope_limit_deg, EPS)
    strength = su <= 1.0
    stiffness = du <= 1.0 and ru <= 1.0
    return MemberCheck(
        member=member, case=case,
        bending_stress_MPa=sb/1e6, axial_stress_MPa=sa/1e6,
        combined_stress_MPa=sc/1e6, allowable_stress_MPa=material.bending_allow_Pa/1e6,
        stress_utilization=su, tip_deflection_mm=d*1e3, deflection_limit_mm=dlim*1e3,
        deflection_utilization=du, root_slope_deg=slope, slope_limit_deg=slope_limit_deg,
        slope_utilization=ru, strength_pass=strength, stiffness_pass=stiffness,
        overall_pass=strength and stiffness, note=note,
    )


def size_tube_for_bending_and_stiffness(
    *, moment_Nm: float, length_m: float, material: Material, id_over_od: float,
    deflection_limit_fraction: float, slope_limit_deg: float,
    min_od_m: float = 0.008, max_od_m: float = 0.050, step_m: float = 0.00025,
) -> Tuple[TubeSection, MemberCheck]:
    if not (0.0 <= id_over_od < 1.0):
        raise ValueError('id_over_od must be in [0,1)')
    od = min_od_m
    while od <= max_od_m + 1e-12:
        sec = TubeSection(od, od * id_over_od)
        chk = check_tube_member(
            member='candidate', case='sizing', section=sec, material=material,
            length_m=length_m, bending_Nm=moment_Nm,
            deflection_limit_fraction=deflection_limit_fraction,
            slope_limit_deg=slope_limit_deg,
        )
        if chk.overall_pass:
            return sec, chk
        od += step_m
    raise RuntimeError('No tube satisfies constraints in requested OD range')


def tube_mass_kg(section: TubeSection, length_m: float, density_kg_m3: float, count: int = 1) -> float:
    return section.area_m2 * length_m * density_kg_m3 * count


def wing_maneuver_root_moment_Nm(mass_kg: float, load_factor: float, span_m: float) -> float:
    """Per-side root bending approximation for symmetric wing lift.

    Treats each half wing as carrying half the total lift with its resultant at
    half of the semi-span: M = n W b / 8. Appropriate as a controlled sizing
    approximation, not a substitute for spanwise aeroelastic load integration.
    """
    return mass_kg * G * load_factor * span_m / 8.0


def rotor_side_root_moment_Nm(thrusts_N: Sequence[float], motor_y_m: Sequence[float], side: str) -> float:
    sign = -1 if side == 'left' else 1
    return sum(abs(float(t) * float(y)) for t, y in zip(thrusts_N, motor_y_m) if (y < 0 if sign < 0 else y > 0))


def rotor_wing_torsion_Nm(thrusts_N: Sequence[float], motor_x_m: Sequence[float], spar_x_m: float, side_indices: Sequence[int], shaft_torque_Nm: float = 0.0) -> float:
    t = sum(abs(float(thrusts_N[i]) * (float(motor_x_m[i]) - spar_x_m)) for i in side_indices)
    t += abs(shaft_torque_Nm) * len(side_indices)
    return t


def zero_pitch_pair_thrust(total_thrust_N: float, x_front_station_m: float, x_rear_station_m: float, cg_station_m: float) -> Tuple[float, float]:
    """Return per-motor front/rear thrust for two front + two rear motors.

    Enforces total thrust and zero pitch moment about CG.
    """
    af = cg_station_m - x_front_station_m
    ar = x_rear_station_m - cg_station_m
    if af <= 0 or ar <= 0:
        raise ValueError('CG must lie between front and rear rotor rows for this allocator')
    # 2Tf + 2Tr = T; Tf*af = Tr*ar
    tf = total_thrust_N * ar / (2.0 * (af + ar))
    tr = total_thrust_N * af / (2.0 * (af + ar))
    return tf, tr


def build_load_cases(cfg: Mapping[str, Any]) -> List[LoadCase]:
    a = cfg['aircraft']; s = cfg['structure']; lc = cfg['load_cases']; rotor = cfg['rotor_system']
    mass_ref = float(a['reference_mass_kg']); mass_design = float(a['design_mass_ceiling_kg'])
    span_design = float(a['design_wing_span_m']); b_ref = float(a['reference_wing_span_m'])
    x_front, x_rear = map(float, rotor['row_x_m'])
    cg = float(a['cg_station_x_m'])
    total_design = 4.0 * float(rotor['phase4_equal_share_design_thrust_N'])
    tf, tr = zero_pitch_pair_thrust(total_design, x_front, x_rear, cg)
    y = abs(float(rotor['motor_y_m']))
    spar_x = float(s['wing_primary_spar_x_m'])
    shaft_q = float(rotor['screening_shaft_reaction_torque_Nm'])
    thrusts = [tf, tf, tr, tr]
    xs = [x_front, x_front, x_rear, x_rear]
    sideidx = [0,2]
    vtol_root = (tf + tr) * y
    vtol_torsion = rotor_wing_torsion_Nm(thrusts, xs, spar_x, sideidx, shaft_q)
    man = wing_maneuver_root_moment_Nm(mass_design, float(lc['maneuver_load_factor']), span_design)

    # Tail/boom screening at dive speed. Coefficients are explicit assumptions in config.
    rho=float(lc['rho_kg_m3']); vd=float(lc['dive_speed_mps'])
    q=0.5*rho*vd*vd
    hforce=q*float(a['horizontal_tail_area_m2'])*float(lc['horizontal_tail_design_CL_abs'])
    sideforce=q*float(a['vertical_tail_total_area_m2'])*float(lc['vertical_tail_design_CY_abs'])
    boomL=float(s['boom_length_m'])
    boom_tail_bend=max(hforce,sideforce)/2.0*boomL
    rear_motor_bend=tr*max(0.0,x_rear-float(s['boom_x_start_m']))
    boom_design=boom_tail_bend + rear_motor_bend
    pusher_axial=float(lc['pusher_design_thrust_N'])/2.0

    # 6-DOF/control reference hover thrust allocation from stored result, not reconstructed.
    hover=list(map(float, rotor['phase6_reference_hover_thrusts_N']))
    hover_root=(hover[0]+hover[2])*y
    hover_tors=rotor_wing_torsion_Nm(hover,xs,spar_x,sideidx,shaft_q*0.6)

    return [
        LoadCase('reference_trimmed_hover','VTOL',mass_ref,1.0,wing_root_bending_Nm=hover_root,wing_torsion_Nm=hover_tors,max_motor_thrust_N=max(hover),max_motor_mount_bending_Nm=max(hover)*float(s['motor_mount_lever_m']),note='6-DOF/control unequal rotor allocation at reference mass.'),
        LoadCase('15kg_zero_pitch_design_VTOL','VTOL',mass_design,1.0,wing_root_bending_Nm=vtol_root,wing_torsion_Nm=vtol_torsion,boom_root_bending_Nm=rear_motor_bend,max_motor_thrust_N=max(tf,tr),max_motor_mount_bending_Nm=max(tf,tr)*float(s['motor_mount_lever_m']),note='Aero/propulsion total design thrust redistributed to zero pitch moment using 6-DOF/control CG/rotor geometry.'),
        LoadCase('15kg_3g_wingborne_manoeuvre','wingborne',mass_design,float(lc['maneuver_load_factor']),wing_root_bending_Nm=man,wing_torsion_Nm=float(lc['wing_aero_torsion_Nm']),tail_force_N=hforce,note='Symmetric wing-borne structural manoeuvre screen.'),
        LoadCase('dive_tail_download','tail/boom',mass_design,1.0,boom_root_bending_Nm=boom_tail_bend,tail_force_N=hforce,note='Horizontal-tail absolute CL design screen at configured dive speed.'),
        LoadCase('dive_crosswind_fin_load','tail/boom',mass_design,1.0,boom_root_bending_Nm=boom_tail_bend,tail_force_N=sideforce,note='Twin-fin side-force structural screen at configured dive speed.'),
        LoadCase('combined_boom_tail_plus_rear_rotor','tail/boom',mass_design,1.0,boom_root_bending_Nm=boom_design,boom_axial_N=pusher_axial,max_motor_thrust_N=tr,note='Conservative envelope combining boom tail load with rear lift-rotor bending; simultaneous occurrence is a sizing envelope, not a trajectory claim.'),
        LoadCase('pusher_axial_design','propulsion',mass_design,1.0,boom_axial_N=pusher_axial,note='Pusher design thrust divided across two booms.'),
        LoadCase('single_motor_mount_limit','local',mass_design,1.0,max_motor_thrust_N=float(rotor['candidate_motor_max_thrust_N']),max_motor_mount_bending_Nm=float(rotor['candidate_motor_max_thrust_N'])*float(s['motor_mount_lever_m']),note='Local motor-mount load requirement. Geometry/material absent, so stress remains NOT_EVALUATED.'),
        LoadCase('one_motor_out_remaining_motor_peak','degraded',mass_design,1.0,max_motor_thrust_N=float(rotor['candidate_motor_max_thrust_N']),max_motor_mount_bending_Nm=float(rotor['candidate_motor_max_thrust_N'])*float(s['motor_mount_lever_m']),note='Structural local envelope for a surviving motor during motor-out recovery; flight controllability remains a 6-DOF/control/8 problem.'),
    ]


def evaluate_structure(cfg: Mapping[str, Any], cases: Sequence[LoadCase]) -> Dict[str, Any]:
    s=cfg['structure']; matcfg=cfg['materials']['cfrp_tube']
    E=float(matcfg['E_Pa']); nu=float(matcfg['poisson']); Gm=E/(2*(1+nu))
    mat=Material(matcfg['name'],E,Gm,float(matcfg['density_kg_m3']),float(matcfg['bending_allow_MPa'])*1e6,float(matcfg['shear_allow_MPa'])*1e6,matcfg.get('status','engineering_assumption'))

    _wing_od=s.get('reference_wing_spar_od_m', s.get('current_wing_spar_od_m'))
    _wing_id=s.get('reference_wing_spar_id_m', s.get('current_wing_spar_id_m'))
    if _wing_od is None or _wing_id is None:
        raise KeyError('structure requires reference_wing_spar_od_m/id_m (or legacy current_wing_spar_*)')
    wing_reference=TubeSection(float(_wing_od), float(_wing_id))
    boom_strength=TubeSection(float(s['legacy_strength_only_boom_od_m']),float(s['legacy_strength_only_boom_id_m']))
    boom_reference=TubeSection(float(s['reference_boom_od_m']),float(s['reference_boom_id_m']))

    wing_case=max(cases,key=lambda c:c.wing_root_bending_Nm)
    boom_case=max(cases,key=lambda c:c.boom_root_bending_Nm)
    wing_reference_chk=check_tube_member(member='wing_primary_spar_reference',case=wing_case.name,section=wing_reference,material=mat,length_m=float(s['design_wing_semispan_m']),bending_Nm=wing_case.wing_root_bending_Nm,deflection_limit_fraction=float(s['wing_tip_deflection_limit_fraction']),slope_limit_deg=float(s['wing_root_slope_limit_deg']))
    wing_rec,wing_rec_chk=size_tube_for_bending_and_stiffness(moment_Nm=wing_case.wing_root_bending_Nm,length_m=float(s['design_wing_semispan_m']),material=mat,id_over_od=float(s['wing_spar_id_over_od']),deflection_limit_fraction=float(s['wing_tip_deflection_limit_fraction']),slope_limit_deg=float(s['wing_root_slope_limit_deg']),min_od_m=0.010,max_od_m=0.040,step_m=0.00025)
    boom_strength_chk=check_tube_member(member='boom_legacy_strength_only',case=boom_case.name,section=boom_strength,material=mat,length_m=float(s['boom_length_m']),bending_Nm=boom_case.boom_root_bending_Nm,axial_N=boom_case.boom_axial_N,deflection_limit_fraction=float(s['boom_tip_deflection_limit_fraction']),slope_limit_deg=float(s['boom_root_slope_limit_deg']))
    boom_reference_chk=check_tube_member(member='boom_reference_geometry',case=boom_case.name,section=boom_reference,material=mat,length_m=float(s['boom_length_m']),bending_Nm=boom_case.boom_root_bending_Nm,axial_N=boom_case.boom_axial_N,deflection_limit_fraction=float(s['boom_tip_deflection_limit_fraction']),slope_limit_deg=float(s['boom_root_slope_limit_deg']))
    # Tail spars: current code only sized these from the cruise tail load.
    # Structures applies the configured design tail-force envelopes so high-speed
    # control/gust loads cannot silently inherit a cruise-only spar.
    hcase=max(cases,key=lambda c:c.tail_force_N if c.name=='dive_tail_download' else -1.0)
    vcase=max(cases,key=lambda c:c.tail_force_N if c.name=='dive_crosswind_fin_load' else -1.0)
    hL=float(s['horizontal_tail_semispan_m']); vL=float(s['vertical_tail_height_m'])
    hM=hcase.tail_force_N/2.0*(hL/2.0)
    vM=vcase.tail_force_N/2.0*(vL/2.0)
    tail_current=TubeSection(float(s['current_tail_spar_reference_od_m']),float(s['current_tail_spar_reference_id_m']))
    hcur=check_tube_member(member='horizontal_tail_spar_current_reference',case=hcase.name,section=tail_current,material=mat,length_m=hL,bending_Nm=hM,deflection_limit_fraction=float(s['tail_deflection_limit_fraction']),slope_limit_deg=float(s['tail_root_slope_limit_deg']))
    vcur=check_tube_member(member='vertical_tail_spar_current_reference',case=vcase.name,section=tail_current,material=mat,length_m=vL,bending_Nm=vM,deflection_limit_fraction=float(s['tail_deflection_limit_fraction']),slope_limit_deg=float(s['tail_root_slope_limit_deg']))
    hrec,hrec_chk=size_tube_for_bending_and_stiffness(moment_Nm=hM,length_m=hL,material=mat,id_over_od=float(s['wing_spar_id_over_od']),deflection_limit_fraction=float(s['tail_deflection_limit_fraction']),slope_limit_deg=float(s['tail_root_slope_limit_deg']),min_od_m=0.004,max_od_m=0.025,step_m=0.00025)
    vrec,vrec_chk=size_tube_for_bending_and_stiffness(moment_Nm=vM,length_m=vL,material=mat,id_over_od=float(s['wing_spar_id_over_od']),deflection_limit_fraction=float(s['tail_deflection_limit_fraction']),slope_limit_deg=float(s['tail_root_slope_limit_deg']),min_od_m=0.004,max_od_m=0.025,step_m=0.00025)

    # Central pod: preserve the live structures.py shell model as authority, but
    # scale its reported stress to the larger Structures governing wing moment.
    pod_scale=wing_case.wing_root_bending_Nm/max(float(s['pod_reference_bending_moment_Nm']),EPS)
    pod_bending=float(s['pod_reference_bending_stress_MPa'])*pod_scale
    pod_shear=float(s['pod_reference_shear_stress_MPa'])*pod_scale
    pod={'scaled_bending_stress_MPa':pod_bending,'scaled_shear_stress_MPa':pod_shear,'bending_allow_MPa':float(s['pod_printed_bending_allow_MPa']),'shear_allow_MPa':float(s['pod_printed_shear_allow_MPa']),'bending_pass':pod_bending<=float(s['pod_printed_bending_allow_MPa']),'shear_pass':pod_shear<=float(s['pod_printed_shear_allow_MPa']),'status':'screen_scaled_from_current_live_structures.py_result'}

    # Alternative boom preserving 2 mm wall, sized for strength+stiffness.
    boom_candidate=None; boom_candidate_chk=None
    wall=float(s['boom_candidate_wall_m'])
    od=float(s['boom_candidate_min_od_m'])
    while od<=float(s['boom_candidate_max_od_m'])+EPS:
        sec=TubeSection(od,max(0.001,od-2*wall))
        chk=check_tube_member(member='boom_stiffness_sized_candidate',case=boom_case.name,section=sec,material=mat,length_m=float(s['boom_length_m']),bending_Nm=boom_case.boom_root_bending_Nm,axial_N=boom_case.boom_axial_N,deflection_limit_fraction=float(s['boom_tip_deflection_limit_fraction']),slope_limit_deg=float(s['boom_root_slope_limit_deg']))
        if chk.overall_pass:
            boom_candidate,boom_candidate_chk=sec,chk;break
        od+=float(s['boom_candidate_step_m'])
    if boom_candidate is None or boom_candidate_chk is None:
        raise RuntimeError('No boom candidate met configured structural constraints')

    # Wing torsion is reported separately because the real closed-cell skin/dual-spar
    # torsional stiffness should be retained from structures.py rather than pretending
    # one circular spar carries all wing torsion.
    wing_torsion_case=max(cases,key=lambda c:c.wing_torsion_Nm)

    # Local motor mount remains unevaluated until mount geometry is controlled.
    local=max(cases,key=lambda c:c.max_motor_mount_bending_Nm)

    return {
        'material': asdict(mat),
        'governing_cases': {'wing_bending':wing_case.name,'wing_torsion':wing_torsion_case.name,'boom':boom_case.name,'motor_mount':local.name},
        'wing': {
            # 'current' kept as alias of 'reference' for older report consumers.
            'reference': {'section':asdict(wing_reference),'check':wing_reference_chk.to_dict()},
            'current': {'section':asdict(wing_reference),'check':wing_reference_chk.to_dict()},
            'recommended_screen': {'section':asdict(wing_rec),'check':wing_rec_chk.to_dict()},
            'governing_root_bending_Nm':wing_case.wing_root_bending_Nm,
            'governing_torsion_Nm':wing_torsion_case.wing_torsion_Nm,
            'torsional_skin_status':'NOT_EVALUATED_IN_PHASE7_WRAPPER_USE_EXISTING_structures.py_closed_cell_check_and_future_CFD/aeroelastic_loads',
        },
        'tails': {
            'horizontal_current_reference': {'section':asdict(tail_current),'check':hcur.to_dict(),'root_moment_Nm':hM},
            'horizontal_recommended_screen': {'section':asdict(hrec),'check':hrec_chk.to_dict(),'root_moment_Nm':hM},
            'vertical_current_reference': {'section':asdict(tail_current),'check':vcur.to_dict(),'root_moment_Nm':vM},
            'vertical_recommended_screen': {'section':asdict(vrec),'check':vrec_chk.to_dict(),'root_moment_Nm':vM},
        },
        'central_pod': pod,
        'booms': {
            'legacy_strength_only': {'section':asdict(boom_strength),'check':boom_strength_chk.to_dict()},
            'reference_35x31_geometry': {'section':asdict(boom_reference),'check':boom_reference_chk.to_dict()},
            'recommended_screen': {'section':asdict(boom_candidate),'check':boom_candidate_chk.to_dict()},
            'governing_root_bending_Nm':boom_case.boom_root_bending_Nm,
            'governing_axial_N':boom_case.boom_axial_N,
        },
        'motor_mount': {
            'status':'NOT_EVALUATED',
            'required_limit_thrust_N':local.max_motor_thrust_N,
            'required_limit_bending_Nm':local.max_motor_mount_bending_Nm,
            'reason':'Mount section geometry, fastener pattern, laminate/printed material orientation and bearing allowables are not controlled in the current model.',
        },
    }


def structural_mass_feedback(cfg: Mapping[str, Any], structural: Mapping[str, Any]) -> Dict[str, Any]:
    """Conservative bookkeeping feedback into Phase-3 mass/CG.

    Only two corrections are applied here:
    1) wing spar mass scales with cross-sectional area from the current root tube;
    2) boom tube mass is reconciled geometrically from candidate tube area and CFRP density.

    No shell/skin/rib rescaling is invented. The future refactor should rerun the live
    structure functions and then the Phase-3 component ledger inside one closure loop.
    """
    a=cfg['aircraft']; s=cfg['structure']; mat=structural['material']
    m0=float(a['reference_mass_kg']); x0=float(a['cg_station_x_m'])
    _ref_od=s.get('reference_wing_spar_od_m', s.get('current_wing_spar_od_m'))
    _ref_id=s.get('reference_wing_spar_id_m', s.get('current_wing_spar_id_m'))
    if _ref_od is None or _ref_id is None:
        raise KeyError('structure requires reference_wing_spar_od_m/id_m (or legacy current_wing_spar_*)')
    cur=TubeSection(float(_ref_od), float(_ref_id))
    rec=TubeSection(**structural['wing']['recommended_screen']['section'])
    old_spar_mass=float(s['current_total_wing_spar_mass_kg'])
    rec_spar_mass=old_spar_mass*rec.area_m2/cur.area_m2
    delta_wing=rec_spar_mass-old_spar_mass

    boom=TubeSection(**structural['booms']['recommended_screen']['section'])
    rec_boom_tube_mass=tube_mass_kg(boom,float(s['boom_length_m']),float(mat['density_kg_m3']),count=int(s['n_booms']))
    old_boom_tube_mass=float(s['legacy_reported_boom_tube_mass_total_kg'])
    delta_boom=rec_boom_tube_mass-old_boom_tube_mass

    # Tail spar deltas use the current model's reported minimum spar area as the
    # mass-scaling reference. This avoids pretending the full tail shell/rib mass
    # scales with spar diameter.
    href_area=float(s['tail_spar_reference_area_mm2'])*1e-6
    hsec=TubeSection(**structural['tails']['horizontal_recommended_screen']['section'])
    vsec=TubeSection(**structural['tails']['vertical_recommended_screen']['section'])
    old_h=float(s['current_horizontal_tail_spar_mass_total_kg']); old_v=float(s['current_vertical_tail_spar_mass_total_kg'])
    new_h=old_h*hsec.area_m2/max(href_area,EPS); new_v=old_v*vsec.area_m2/max(href_area,EPS)
    delta_h=new_h-old_h; delta_v=new_v-old_v

    m1=m0+delta_wing+delta_boom+delta_h+delta_v
    xwing=float(s['wing_structure_station_x_m']); xboom=float(s['boom_mass_station_x_m']); xtail=float(s['tail_structure_station_x_m'])
    x1=(m0*x0+delta_wing*xwing+delta_boom*xboom+(delta_h+delta_v)*xtail)/m1
    return {
        'reference_mass_kg':m0,'reference_cg_x_m':x0,
        'reference_wing_spar_mass_kg':old_spar_mass,'current_wing_spar_mass_kg':old_spar_mass,
        'recommended_screen_wing_spar_mass_kg':rec_spar_mass,'wing_spar_mass_delta_kg':delta_wing,
        'legacy_reported_boom_tube_mass_kg':old_boom_tube_mass,'geometry_reconciled_candidate_boom_tube_mass_kg':rec_boom_tube_mass,'boom_tube_mass_delta_kg':delta_boom,
        'current_horizontal_tail_spar_mass_kg':old_h,'recommended_horizontal_tail_spar_mass_kg':new_h,'horizontal_tail_spar_mass_delta_kg':delta_h,
        'current_vertical_tail_spar_mass_kg':old_v,'recommended_vertical_tail_spar_mass_kg':new_v,'vertical_tail_spar_mass_delta_kg':delta_v,
        'screening_updated_mass_kg':m1,'screening_updated_cg_x_m':x1,'screening_cg_shift_mm':(x1-x0)*1000,
        'mass_margin_to_15kg_kg':float(a['design_mass_ceiling_kg'])-m1,
        'warning':'This is a partial structural-mass feedback only. It does not rescale wing skins/ribs/webs, mounts, pod or tails and is not the final coupled MTOW closure.',
    }


def evaluate_gates(cfg: Mapping[str, Any], structural: Mapping[str, Any], feedback: Mapping[str, Any]) -> List[Dict[str, Any]]:
    gates=[]
    def add(name,status,value,limit,note=''):
        gates.append({'name':name,'status':status,'value':value,'limit':limit,'note':note})
    wc=structural['wing'].get('reference', structural['wing']['current'])['check']; wr=structural['wing']['recommended_screen']['check']
    bl=structural['booms']['legacy_strength_only']['check']; br=structural['booms']['recommended_screen']['check']; bref=structural['booms']['reference_35x31_geometry']['check']
    add('reference_wing_spar_strength','PASS' if wc['strength_pass'] else 'FAIL',wc['stress_utilization'],'<=1.0','Mass-calibration / audit reference section — not the recommended design spar.')
    add('reference_wing_spar_stiffness','PASS' if wc['stiffness_pass'] else 'FAIL',max(wc['deflection_utilization'],wc['slope_utilization']),'<=1.0','Reference spar is an audit/mass-scaling section, not accepted when stiffness fails.')
    add('recommended_wing_spar_screen','PASS' if wr['overall_pass'] else 'FAIL',max(wr['stress_utilization'],wr['deflection_utilization'],wr['slope_utilization']),'<=1.0')
    add('legacy_strength_only_boom_stiffness','PASS' if bl['stiffness_pass'] else 'FAIL',max(bl['deflection_utilization'],bl['slope_utilization']),'<=1.0')
    add('reference_35x31_boom_screen','PASS' if bref['overall_pass'] else 'FAIL',max(bref['stress_utilization'],bref['deflection_utilization'],bref['slope_utilization']),'<=1.0','Geometry passes the screen, but its mass conflicts with the old reported boom-tube mass and needs supplier/CAD reconciliation.')
    add('recommended_boom_screen','PASS' if br['overall_pass'] else 'FAIL',max(br['stress_utilization'],br['deflection_utilization'],br['slope_utilization']),'<=1.0')
    for label,key in [('horizontal_tail_current','horizontal_current_reference'),('vertical_tail_current','vertical_current_reference')]:
        c=structural['tails'][key]['check']; add(label+'_screen','PASS' if c['overall_pass'] else 'FAIL',max(c['stress_utilization'],c['deflection_utilization'],c['slope_utilization']),'<=1.0','Current tail spars were not sized for this high-speed load envelope.')
    for label,key in [('horizontal_tail_recommended','horizontal_recommended_screen'),('vertical_tail_recommended','vertical_recommended_screen')]:
        c=structural['tails'][key]['check']; add(label+'_screen','PASS' if c['overall_pass'] else 'FAIL',max(c['stress_utilization'],c['deflection_utilization'],c['slope_utilization']),'<=1.0')
    pod=structural['central_pod']; add('central_pod_scaled_strength','PASS' if pod['bending_pass'] and pod['shear_pass'] else 'FAIL',max(pod['scaled_bending_stress_MPa']/pod['bending_allow_MPa'],pod['scaled_shear_stress_MPa']/pod['shear_allow_MPa']),'<=1.0','Scaled audit screen; live pod shell/frame model remains authority.')
    add('motor_mount_strength','NOT_EVALUATED',structural['motor_mount']['required_limit_bending_Nm'],'controlled mount geometry + material allowables required')
    add('mass_ceiling_after_partial_structural_feedback','PASS' if feedback['screening_updated_mass_kg']<=float(cfg['aircraft']['design_mass_ceiling_kg']) else 'FAIL',feedback['screening_updated_mass_kg'],float(cfg['aircraft']['design_mass_ceiling_kg']))
    return gates


def build_phase7_result(cfg: Mapping[str, Any]) -> Dict[str, Any]:
    cases=build_load_cases(cfg)
    structural=evaluate_structure(cfg,cases)
    feedback=structural_mass_feedback(cfg,structural)
    gates=evaluate_gates(cfg,structural,feedback)
    return {
        'model_version':cfg['model_version'],
        'metadata':cfg['metadata'],
        'load_cases':[c.to_dict() for c in cases],
        'structural_checks':structural,
        'mass_feedback':feedback,
        'gates':gates,
        'summary':{
            'governing_wing_root_bending_Nm':structural['wing']['governing_root_bending_Nm'],
            'recommended_wing_spar_od_mm':structural['wing']['recommended_screen']['section']['od_m']*1000,
            'recommended_wing_spar_id_mm':structural['wing']['recommended_screen']['section']['id_m']*1000,
            'current_wing_tip_deflection_mm':structural['wing']['current']['check']['tip_deflection_mm'],
            'recommended_wing_tip_deflection_mm':structural['wing']['recommended_screen']['check']['tip_deflection_mm'],
            'governing_boom_root_bending_Nm':structural['booms']['governing_root_bending_Nm'],
            'recommended_boom_od_mm':structural['booms']['recommended_screen']['section']['od_m']*1000,
            'recommended_boom_id_mm':structural['booms']['recommended_screen']['section']['id_m']*1000,
            'reference_35x31_boom_tip_deflection_mm':structural['booms']['reference_35x31_geometry']['check']['tip_deflection_mm'],
            'legacy_strength_only_boom_tip_deflection_mm':structural['booms']['legacy_strength_only']['check']['tip_deflection_mm'],
            'horizontal_tail_recommended_spar_od_mm':structural['tails']['horizontal_recommended_screen']['section']['od_m']*1000,
            'vertical_tail_recommended_spar_od_mm':structural['tails']['vertical_recommended_screen']['section']['od_m']*1000,
            'required_motor_mount_limit_thrust_N':structural['motor_mount']['required_limit_thrust_N'],
            'required_motor_mount_limit_bending_Nm':structural['motor_mount']['required_limit_bending_Nm'],
            'partial_structural_feedback_mass_kg':feedback['screening_updated_mass_kg'],
            'partial_structural_feedback_cg_x_m':feedback['screening_updated_cg_x_m'],
            'mass_margin_to_15kg_kg':feedback['mass_margin_to_15kg_kg'],
            'gate_fail_count':sum(g['status']=='FAIL' for g in gates),
            'not_evaluated_count':sum(g['status']=='NOT_EVALUATED' for g in gates),
        },
    }

# Clean integrated API alias; legacy development name retained for compatibility.
build_structural_result = build_phase7_result
