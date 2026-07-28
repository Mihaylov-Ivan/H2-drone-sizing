"""
Structured results dataclasses for design output.
"""

from dataclasses import dataclass, field
from typing import Optional, Dict, Any, TYPE_CHECKING

if TYPE_CHECKING:
    from design_inputs import DesignInputs


@dataclass
class Geometry:
    """Geometric properties of a lifting surface."""
    area_m2: float
    span_m: float
    c_root_m: float
    c_tip_m: float
    MAC_m: float
    y_MAC_from_centerline_m: float
    ac_on_MAC_m: float
    AR: float
    c_MGC_m: float


@dataclass
class Aerodynamics:
    """Aerodynamic coefficients and properties."""
    CL_cruise: float
    CL_max: float
    CD0: float
    CD_cruise: float
    CD0_fuselage: float
    CM_cruise: float
    CM_max: float
    alpha_cruise_deg: float
    L_over_D_cruise: float
    V_stall: float
    Re_cruise: float
    max_thickness_fraction_of_chord: float
    CL_alpha_3D_per_rad: float = 0.0

@dataclass
class Structure:
    """Structural properties and loads."""
    bending_moment_Nm: float
    bending_moment_type: str  # "cruise" or "vtol" - indicates which moment dominates
    torsion_Nm: float
    h_m: float
    t_skin_root_m: float
    t_skin_tip_m: float
    W_skin_kg: float
    t_web_root_m: float
    t_web_tip_m: float
    W_web_kg: float
    spar_area_root_m2: float
    spar_area_tip_m2: float
    W_spars_kg: float
    N_ribs: int
    rib_spacing_m: float
    W_ribs_kg: float
    W_total_kg: float


@dataclass
class WingResults:
    """Complete wing system results."""
    geometry: Geometry
    aerodynamics: Aerodynamics
    structure: Structure
    x_ac_wing_m: float
    moment_at_cg_Nm: float
    # Wing spar tube sizing results
    wing_spar_length_m: float = 0.0
    wing_spar_M_max_Nm: float = 0.0
    wing_spar_sigma_allow_MPa: float = 0.0
    wing_spar_sigma_root_MPa: float = 0.0
    wing_spar_OD_root_m: float = 0.0
    wing_spar_ID_root_m: float = 0.0
    wing_spar_OD_tip_m: float = 0.0
    wing_spar_ID_tip_m: float = 0.0
    wing_spar_A_root_mm2: float = 0.0
    wing_spar_A_tip_mm2: float = 0.0
    wing_spar_I_root_mm4: float = 0.0
    wing_spar_I_tip_mm4: float = 0.0
    wing_spar_mass_kg: float = 0.0
    wing_spar_N_ribs: float = 0.0
    wing_spar_rib_contact_L_m: float = 0.0
    wing_spar_bearing_allow_MPa: float = 0.0
    wing_spar_bearing_stress_MPa: float = 0.0
    wing_spar_bearing_ok: float = 0.0
    # Wing skin stiffeners (torsion / shear buckling)
    wing_stiffener_N: float = 0.0
    wing_stiffener_spacing_m: float = 0.0
    wing_stiffener_thickness_m: float = 0.0
    wing_stiffener_height_m: float = 0.0
    wing_stiffener_area_one_mm2: float = 0.0
    wing_stiffener_I_one_mm4: float = 0.0
    wing_stiffener_tau_skin_MPa: float = 0.0
    wing_stiffener_tau_cr_panel_MPa: float = 0.0
    wing_stiffener_q_shear_N_per_m: float = 0.0
    wing_stiffener_aspect_ratio: float = 0.0
    wing_stiffener_mass_kg: float = 0.0
    wing_stiffener_panel_ok: float = 0.0
    # Wing spar deflection results (simple cantilever estimate)
    wing_spar_tip_deflection_m: float = 0.0
    wing_spar_tip_deflection_mm: float = 0.0
    wing_spar_root_slope_rad: float = 0.0
    wing_spar_root_slope_deg: float = 0.0
    wetted_area_m2: float = 0.0  # wetted area [m²] used in cruise drag (2×S est. or hardcoded)
    # Concentrated lift-propulsor structural loads (one half-wing moments).
    lift_mount_mass_total_kg: float = 0.0
    lift_thrust_per_motor_structural_N: float = 0.0
    lift_weight_root_moment_Nm_half: float = 0.0
    lift_thrust_root_moment_Nm_half: float = 0.0
    lift_shaft_torque_Nm_half: float = 0.0
    lift_offset_torsion_Nm_half: float = 0.0


@dataclass
class TailResults:
    """Complete tail system results."""
    geometry: Geometry
    aerodynamics: Aerodynamics
    structure: Structure
    x_ac_tail_m: float
    lift_N: float
    tail_incidence_deg: float
    cruise_elevator_deg: float
    CL_req: float
    CL_no_elev: float
    dCL_required: float
    elevator_chord_ratio: float
    eta_f: float
    elevator_deflection_deg: float
    alpha_body_deg: float
    alpha_equiv_deg: float
    # Elevator hinge rod sizing results
    elevator_rod_span_frac: float = 0.0
    elevator_rod_length_m: float = 0.0
    elevator_rod_F_elev_N: float = 0.0
    elevator_rod_M_max_Nm: float = 0.0
    elevator_rod_sigma_allow_MPa: float = 0.0
    elevator_rod_sigma_rod_MPa: float = 0.0
    elevator_rod_OD_m: float = 0.0
    elevator_rod_ID_m: float = 0.0
    elevator_rod_area_mm2: float = 0.0
    elevator_rod_I_mm4: float = 0.0
    elevator_rod_mass_kg: float = 0.0
    elevator_rod_N_support: float = 0.0
    elevator_rod_contact_L_m: float = 0.0
    elevator_rod_bearing_allow_MPa: float = 0.0
    # Elevator hinge moment (at max deflection)
    elevator_hinge_moment_M_h_Nm: float = 0.0
    elevator_area_S_e_m2: float = 0.0
    elevator_chord_c_e_m: float = 0.0
    servo_torque_T_servo_Nm: float = 0.0
    servo_torque_with_margin_Nm: float = 0.0
    # Tail spar tube sizing results
    tail_spar_length_m: float = 0.0
    tail_spar_M_max_Nm: float = 0.0
    tail_spar_sigma_allow_MPa: float = 0.0
    tail_spar_sigma_root_MPa: float = 0.0
    tail_spar_OD_root_m: float = 0.0
    tail_spar_ID_root_m: float = 0.0
    tail_spar_OD_tip_m: float = 0.0
    tail_spar_ID_tip_m: float = 0.0
    tail_spar_A_root_mm2: float = 0.0
    tail_spar_A_tip_mm2: float = 0.0
    tail_spar_I_root_mm4: float = 0.0
    tail_spar_I_tip_mm4: float = 0.0
    tail_spar_mass_kg: float = 0.0
    tail_spar_N_ribs: float = 0.0
    tail_spar_rib_contact_L_m: float = 0.0
    tail_spar_bearing_allow_MPa: float = 0.0
    tail_spar_bearing_stress_MPa: float = 0.0
    tail_spar_bearing_ok: float = 0.0
    # Tail skin stiffeners (torsion / shear buckling)
    tail_stiffener_N: float = 0.0
    tail_stiffener_spacing_m: float = 0.0
    tail_stiffener_thickness_m: float = 0.0
    tail_stiffener_height_m: float = 0.0
    tail_stiffener_area_one_mm2: float = 0.0
    tail_stiffener_I_one_mm4: float = 0.0
    tail_stiffener_tau_skin_MPa: float = 0.0
    tail_stiffener_tau_cr_panel_MPa: float = 0.0
    tail_stiffener_q_shear_N_per_m: float = 0.0
    tail_stiffener_aspect_ratio: float = 0.0
    tail_stiffener_mass_kg: float = 0.0
    tail_stiffener_panel_ok: float = 0.0
    # Tail spar deflection results (simple cantilever estimate)
    tail_spar_tip_deflection_m: float = 0.0
    tail_spar_tip_deflection_mm: float = 0.0
    tail_spar_root_slope_rad: float = 0.0
    tail_spar_root_slope_deg: float = 0.0
    wetted_area_m2: float = 0.0  # complete horizontal tailplane wetted area [m²]


@dataclass
class VerticalWingResults:
    """Complete vertical wing system results."""
    geometry: Geometry
    aerodynamics: Aerodynamics
    structure: Structure
    x_ac_wing_m: float


@dataclass
class VerticalTailResults:
    """Complete vertical tail system results."""
    geometry: Geometry
    aerodynamics: Aerodynamics
    structure: Structure
    x_ac_tail_m: float
    # Vertical tail spar tube sizing results
    vertical_tail_spar_length_m: float = 0.0
    vertical_tail_spar_M_max_Nm: float = 0.0
    vertical_tail_spar_sigma_allow_MPa: float = 0.0
    vertical_tail_spar_sigma_root_MPa: float = 0.0
    vertical_tail_spar_OD_root_m: float = 0.0
    vertical_tail_spar_ID_root_m: float = 0.0
    vertical_tail_spar_OD_tip_m: float = 0.0
    vertical_tail_spar_ID_tip_m: float = 0.0
    vertical_tail_spar_A_root_mm2: float = 0.0
    vertical_tail_spar_A_tip_mm2: float = 0.0
    vertical_tail_spar_I_root_mm4: float = 0.0
    vertical_tail_spar_I_tip_mm4: float = 0.0
    vertical_tail_spar_mass_kg: float = 0.0
    vertical_tail_spar_N_ribs: float = 0.0
    vertical_tail_spar_rib_contact_L_m: float = 0.0
    vertical_tail_spar_bearing_allow_MPa: float = 0.0
    vertical_tail_spar_bearing_stress_MPa: float = 0.0
    vertical_tail_spar_bearing_ok: float = 0.0
    # Vertical tail skin stiffeners (torsion / shear buckling)
    vertical_tail_stiffener_N: float = 0.0
    vertical_tail_stiffener_spacing_m: float = 0.0
    vertical_tail_stiffener_thickness_m: float = 0.0
    vertical_tail_stiffener_height_m: float = 0.0
    vertical_tail_stiffener_area_one_mm2: float = 0.0
    vertical_tail_stiffener_I_one_mm4: float = 0.0
    vertical_tail_stiffener_tau_skin_MPa: float = 0.0
    vertical_tail_stiffener_tau_cr_panel_MPa: float = 0.0
    vertical_tail_stiffener_q_shear_N_per_m: float = 0.0
    vertical_tail_stiffener_aspect_ratio: float = 0.0
    vertical_tail_stiffener_mass_kg: float = 0.0
    vertical_tail_stiffener_panel_ok: float = 0.0
    # Vertical tail spar deflection results (simple cantilever estimate)
    vertical_tail_spar_tip_deflection_m: float = 0.0
    vertical_tail_spar_tip_deflection_mm: float = 0.0
    vertical_tail_spar_root_slope_rad: float = 0.0
    vertical_tail_spar_root_slope_deg: float = 0.0
    wetted_area_m2: float = 0.0  # total wetted area of both vertical fins [m²]


@dataclass
class FuselageResults:
    """Fuselage structure results."""
    t_shell_m: float
    W_fuselage_kg: float
    fuselage_nose_longeron_boss_height_m: float  # OD/2 - skin_thickness - payload_length/2 [m]
    W_fuselage_nose_kg: float = 0.0
    W_fuselage_section_C_kg: float = 0.0
    W_fuselage_section_B_kg: float = 0.0
    W_fuselage_section_A_kg: float = 0.0
    W_fuselage_tail_kg: float = 0.0
    wetted_area_m2: float = 0.0  # fuselage wetted area used in parasite drag [m²]
    # Mass breakdown (from fuselage structure estimation)
    W_shell_kg: float = 0.0
    W_frames_total_kg: float = 0.0
    W_stringers_total_kg: float = 0.0
    W_longerons_total_kg: float = 0.0
    T_design_Nm: float = 0.0
    # Frames
    N_frames: float = 0.0
    frame_spacing_m: float = 0.0
    frame_annulus_area_mm2: float = 0.0
    # Stringers
    N_stringers: float = 0.0
    stringer_spacing_m: float = 0.0
    stringer_thickness_m: float = 0.0
    stringer_height_m: float = 0.0
    stringer_area_one_mm2: float = 0.0
    # Longerons
    N_longerons: float = 0.0
    longeron_OD_m: float = 0.0
    longeron_ID_m: float = 0.0
    longeron_A_one_mm2: float = 0.0
    # Panel check (compression + shear buckling)
    sigma_x_MPa: float = 0.0
    tau_xy_MPa: float = 0.0
    sigma_cr_MPa: float = 0.0
    tau_cr_MPa: float = 0.0
    q_torsion_N_per_m: float = 0.0
    panel_aspect_ratio: float = 0.0
    panel_interaction_index: float = 0.0
    panel_comp_ok: float = 0.0
    panel_shear_ok: float = 0.0
    panel_interaction_ok: float = 0.0


@dataclass
class BoomResults:
    """Twin-boom primary structure and load-check results."""
    total_mass_kg: float
    tube_mass_kg: float
    mount_mass_kg: float
    tail_crossmember_mass_kg: float
    outer_diameter_m: float
    inner_diameter_m: float
    length_m: float
    bending_moment_cruise_Nm_per_boom: float
    bending_moment_vtol_Nm_per_boom: float
    bending_moment_design_Nm_per_boom: float
    pusher_axial_load_N_per_boom: float
    bending_stress_MPa: float
    axial_stress_MPa: float
    combined_stress_MPa: float
    allowable_stress_MPa: float
    strength_ok: bool
    tip_deflection_m: float
    tip_deflection_mm: float



@dataclass
class BatteryResults:
    """Hybrid energy-system results; legacy fields still describe the buffer battery."""
    name: str
    voltage_V: float
    capacity_mAh: float
    mass_kg: float
    length_m: float
    width_m: float
    height_m: float
    usable_energy_Wh: float
    endurance_min_at_cruise: float
    C_at_cruise: float
    meets_endurance_req: bool
    nominal_energy_Wh: float = 0.0
    initial_soc: float = 1.0
    minimum_soc: float = 0.0
    final_soc: float = 0.0
    peak_power_W: float = 0.0
    peak_C: float = 0.0
    fuel_cell_name: str = ""
    fuel_cell_rated_power_W: float = 0.0
    fuel_cell_max_bus_power_W: float = 0.0
    fuel_cell_system_mass_kg: float = 0.0
    fuel_cell_usable_energy_Wh: float = 0.0
    hydrogen_storage_name: str = ""
    hydrogen_pressure_bar: float = 0.0
    hydrogen_nominal_mass_kg: float = 0.0
    hydrogen_usable_mass_kg: float = 0.0
    hydrogen_used_kg: float = 0.0
    hydrogen_tank_mass_kg: float = 0.0
    hydrogen_regulator_mass_kg: float = 0.0
    total_energy_system_mass_kg: float = 0.0


@dataclass
class PropulsionResults:
    """Separate lift and cruise propulsion results with legacy cruise aliases."""
    n_propellers: int
    name: str
    prop_diameter_in: float
    prop_pitch_in: float
    motor_kv: float
    motor_mass_kg: float
    prop_mass_kg: float
    total_motor_mass_kg: float
    total_prop_mass_kg: float
    total_propulsion_mass_kg: float
    max_thrust_per_motor_N: float
    required_thrust_per_motor_N: float
    max_power_per_motor_W: float
    required_power_per_motor_W: float
    meets_thrust_req: bool
    meets_power_req: bool
    est_current_A_per_motor_at_cruise: Optional[float] = None
    architecture: str = "legacy_single_propulsion"
    n_lift_propellers: int = 0
    lift_name: str = ""
    lift_prop_diameter_in: float = 0.0
    lift_prop_pitch_in: float = 0.0
    lift_motor_kv: float = 0.0
    lift_motor_mass_kg: float = 0.0
    lift_prop_mass_kg: float = 0.0
    lift_max_thrust_per_motor_N: float = 0.0
    lift_required_thrust_per_motor_N: float = 0.0
    lift_max_power_per_motor_W: float = 0.0
    lift_required_power_per_motor_W: float = 0.0
    lift_meets_thrust_req: bool = False
    lift_meets_power_req: bool = False
    lift_esc_sizing_power_per_motor_W: float = 0.0
    lift_esc_cont_current_A_per_motor: float = 0.0
    n_cruise_propellers: int = 0
    cruise_name: str = ""
    cruise_prop_diameter_in: float = 0.0
    cruise_prop_pitch_in: float = 0.0
    cruise_motor_kv: float = 0.0
    cruise_motor_mass_kg: float = 0.0
    cruise_prop_mass_kg: float = 0.0
    cruise_max_thrust_per_motor_N: float = 0.0
    cruise_required_thrust_per_motor_N: float = 0.0
    cruise_max_power_per_motor_W: float = 0.0
    cruise_required_power_per_motor_W: float = 0.0
    cruise_meets_thrust_req: bool = False
    cruise_meets_power_req: bool = False
    cruise_esc_sizing_power_per_motor_W: float = 0.0
    cruise_esc_cont_current_A_per_motor: float = 0.0


@dataclass
class AvionicsResults:
    """Avionics and electronics results."""
    autopilot_name: str
    autopilot_mass_kg: float
    gps_mass_kg: float
    airspeed_mass_kg: float
    rc_rx_mass_kg: float
    telemetry_mass_kg: float
    power_module_mass_kg: float
    bec_mass_kg: float
    autopilot_powerbrick_mass_kg: float
    servos_mass_kg: float
    esc_mass_total_kg: float
    misc_electronics_mass_kg: float
    camera_mass_kg: float
    total_avionics_electrics_mass_kg: float
    lift_esc_mass_total_kg: float = 0.0
    cruise_esc_mass_total_kg: float = 0.0
    lift_esc_cont_current_A: float = 0.0
    cruise_esc_cont_current_A: float = 0.0


@dataclass
class CgLocations:
    """All center-of-gravity and related x-locations used in the model."""
    x_cg_m: float
    x_cg_over_fuselage_length: float
    x_ac_wing_m: float
    x_ac_tail_m: float
    x_ac_vertical_tail_m: float
    x_np_m: float
    cg_x_wing: float
    cg_x_tail: float
    cg_x_vertical_tail: float
    cg_x_battery: float
    cg_x_nose: float
    cg_x_body_section_A: float
    cg_x_body_section_B: float
    cg_x_body_section_C: float
    cg_x_fuselage_tail: float
    cg_x_propulsion: float
    cg_x_autopilot: float
    cg_x_power_module: float
    cg_x_bec: float
    cg_x_autopilot_powerbrick: float
    cg_x_esc: float
    cg_x_gps: float
    cg_x_airspeed: float
    cg_x_rx: float
    cg_x_tel: float
    cg_x_camera: float
    cg_x_servos: float
    cg_x_misc: float
    cg_x_payload: float
    cg_x_fuel_cell: float
    cg_x_hydrogen_tank: float
    cg_x_hydrogen_regulator: float
    cg_x_lift_propulsion_front: float
    cg_x_lift_propulsion_rear: float
    cg_x_cruise_propulsion: float
    cg_x_lift_esc_front: float
    cg_x_lift_esc_rear: float
    cg_x_cruise_esc: float
    cg_x_booms: float
    cg_x_central_pod: float
    cg_x_landing_gear: float
    wing_le_x_m: float
    tail_le_x_m: float
    vertical_tail_le_x_m: float
    x_primary_wing_spar_m: float
    x_secondary_wing_spar_m: float
    x_primary_tail_spar_m: float
    x_secondary_tail_spar_m: float
    x_primary_vertical_tail_spar_m: float
    x_secondary_vertical_tail_spar_m: float
    x_longerons_center_m: float


def build_cg_locations(
    inputs: "DesignInputs",
    *,
    x_cg_m: float,
    x_ac_wing_m: float,
    x_ac_tail_m: float,
    x_ac_vertical_tail_m: float,
    x_np_m: float,
) -> CgLocations:
    from design_inputs import CG_X_COMPONENT_FIELDS, compute_derived_cg_x_locations

    component = {name: float(getattr(inputs, name)) for name in CG_X_COMPONENT_FIELDS}
    derived = compute_derived_cg_x_locations(inputs)
    fuselage_length = float(inputs.fuselage_length)
    x_cg_over = x_cg_m / fuselage_length if fuselage_length > 0.0 else float("nan")

    return CgLocations(
        x_cg_m=float(x_cg_m),
        x_cg_over_fuselage_length=x_cg_over,
        x_ac_wing_m=float(x_ac_wing_m),
        x_ac_tail_m=float(x_ac_tail_m),
        x_ac_vertical_tail_m=float(x_ac_vertical_tail_m),
        x_np_m=float(x_np_m),
        **component,
        **derived,
    )


@dataclass
class MassPropertiesResults:
    """Detailed longitudinal mass table and fuel-burn CG excursion."""
    items: Dict[str, Dict[str, float]]
    takeoff_mass_kg: float
    end_mass_kg: float
    takeoff_moment_kgm: float
    end_moment_kgm: float
    x_cg_takeoff_m: float
    x_cg_end_m: float
    cg_shift_m: float
    hydrogen_takeoff_kg: float = 0.0
    hydrogen_end_kg: float = 0.0



@dataclass
class DragBreakdownResults:
    """Configuration-level cruise drag build-up."""
    dynamic_pressure_Pa: float
    wing_N: float
    horizontal_tail_N: float
    vertical_tails_N: float
    central_pod_N: float
    twin_booms_N: float
    lift_motor_nacelles_N: float
    lift_motor_supports_N: float
    stopped_lift_propellers_N: float
    fuel_cell_cooling_N: float
    miscellaneous_N: float
    junction_interference_N: float
    induced_total_N: float
    parasite_total_N: float
    total_N: float
    CD0_aircraft: float
    CD_total_ref: float
    pod_wetted_area_m2: float
    booms_wetted_area_m2: float
    lift_prop_drag_area_each_m2: float
    number_of_vertical_fins: float


@dataclass
class AircraftResults:
    """Overall aircraft results."""
    W0_mass_kg: float
    W0_weight_N: float
    L_wing_N: float
    x_cg_m: float
    drag_cruise_N: float
    cruise_power_W: float
    cruise_power_elec_W: float
    x_np_m: float
    static_margin: float
    static_margin_percent_MAC: float
    V_h: float
    Cn_beta: float
    V_v: float


@dataclass
class ManeuverResults:
    """Elevator control and maneuver capability results."""
    V_maneuver_mps: float
    # Elevator control derivatives
    CL_alpha_h_per_rad: float
    tau_e: float
    bE_over_bh_geom: float
    bE_over_bh_eff: float
    eta_h: float
    l_h_m: float
    V_H: float
    Cm_deltaE_per_rad: float
    CL_deltaE_per_rad: float
    CLh_deltaE_per_rad: float
    # Maximum load factor results
    n_max: float
    feasible: bool
    alpha_w_deg: Optional[float] = None
    CL_w: Optional[float] = None
    L_w_N: Optional[float] = None
    M_ac_w_Nm: Optional[float] = None
    M_cg_w_Nm: Optional[float] = None
    L_tail_req_N: Optional[float] = None
    CL_tail_req: Optional[float] = None
    delta_e_deg: Optional[float] = None
    downwash_deg: Optional[float] = None
    alpha_tail_geom_deg: Optional[float] = None
    alpha_stall_eff_tail_deg: Optional[float] = None
    phi_max_deg_level_turn: Optional[float] = None
    reason: Optional[str] = None


@dataclass
class RudderResults:
    """Rudder control and directional capability results."""
    V_rudder_mps: float
    # Rudder control derivatives (clean flow)
    a_v_per_rad: float
    l_v_m: float
    cR_over_cV: float
    tau_r: float
    bR_over_bV_geom: float
    bR_over_bV_eff: float
    eta_v: float
    dsigma_dbeta: float
    S_v_eff_m2: float
    V_Ve: float
    Cn_deltaR_per_rad: float
    CY_deltaR_per_rad: float
    CY_beta_v_per_rad: float
    # Asymmetric thrust results
    y_engine_m: float
    thrust_asym_N: float
    N_req_asym_Nm: float
    deltaR_req_asym_deg: float
    T_asym_max_N: float
    feasible_asym: bool
    # Spin recovery results
    V_spin_mps: float
    mass_kg: float
    Izz_kgm2: float
    spin_rate_rev_per_s: float
    r0_radps: float
    t_recover_s: float
    rdot_req_radps2: float
    N_req_spin_Nm: float
    deltaR_req_spin_deg: float
    rdot_max_radps2: float
    t_stop_best_s: float
    feasible_spin: bool
    # Optional fields (with defaults)
    reason_asym: Optional[str] = None
    reason_spin: Optional[str] = None
    # Coordinated turn results
    Cn_beta_per_rad: Optional[float] = None
    beta_trim_max_abs_deg: Optional[float] = None
    N_max_abs_Nm: Optional[float] = None
    deltaR_max_abs_deg: Optional[float] = None


@dataclass
class DesignResults:
    """Complete design results."""
    aircraft: AircraftResults
    wing: WingResults
    tail: TailResults
    # VERTICAL MAIN WING REMOVED FOR NOW
    # vertical_wing: VerticalWingResults
    vertical_tail: VerticalTailResults
    fuselage: FuselageResults
    booms: BoomResults
    battery: Optional[BatteryResults]
    propulsion: Optional[PropulsionResults]
    avionics: AvionicsResults
    drag_breakdown: Optional[DragBreakdownResults] = None
    maneuver: Optional[ManeuverResults] = None
    rudder: Optional[RudderResults] = None
    cg_locations: Optional[CgLocations] = None
    mass_properties: Optional[MassPropertiesResults] = None
    inputs: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert results to flat dictionary format (for backward compatibility)."""
        result = {}
        
        # Aircraft
        result['W0_mass_kg'] = self.aircraft.W0_mass_kg
        result['W0_weight_N'] = self.aircraft.W0_weight_N
        result['L_wing_N'] = self.aircraft.L_wing_N
        result['x_cg_m'] = self.aircraft.x_cg_m
        result['W_booms_kg'] = self.booms.total_mass_kg
        result['boom_OD_m'] = self.booms.outer_diameter_m
        result['boom_ID_m'] = self.booms.inner_diameter_m
        result['boom_strength_ok'] = self.booms.strength_ok
        if self.drag_breakdown is not None:
            db = self.drag_breakdown
            result['drag_cruise_N'] = db.total_N
            result['CD0_aircraft'] = db.CD0_aircraft
            result['CD_total_ref'] = db.CD_total_ref
            result['drag_wing_N'] = db.wing_N
            result['drag_horizontal_tail_N'] = db.horizontal_tail_N
            result['drag_vertical_tails_N'] = db.vertical_tails_N
            result['drag_central_pod_N'] = db.central_pod_N
            result['drag_twin_booms_N'] = db.twin_booms_N
            result['drag_stopped_lift_propellers_N'] = db.stopped_lift_propellers_N
        if self.cg_locations is not None:
            cg = self.cg_locations
            result['x_cg_over_fuselage_length'] = cg.x_cg_over_fuselage_length
            result['cg_x_wing'] = cg.cg_x_wing
            result['cg_x_tail'] = cg.cg_x_tail
            result['cg_x_vertical_tail'] = cg.cg_x_vertical_tail
            result['cg_x_battery'] = cg.cg_x_battery
            result['cg_x_nose'] = cg.cg_x_nose
            result['cg_x_body_section_A'] = cg.cg_x_body_section_A
            result['cg_x_body_section_B'] = cg.cg_x_body_section_B
            result['cg_x_body_section_C'] = cg.cg_x_body_section_C
            result['cg_x_fuselage_tail'] = cg.cg_x_fuselage_tail
            result['cg_x_propulsion'] = cg.cg_x_propulsion
            result['cg_x_fuel_cell'] = cg.cg_x_fuel_cell
            result['cg_x_hydrogen_tank'] = cg.cg_x_hydrogen_tank
            result['cg_x_hydrogen_regulator'] = cg.cg_x_hydrogen_regulator
            result['cg_x_lift_propulsion_front'] = cg.cg_x_lift_propulsion_front
            result['cg_x_lift_propulsion_rear'] = cg.cg_x_lift_propulsion_rear
            result['cg_x_cruise_propulsion'] = cg.cg_x_cruise_propulsion
            result['cg_x_lift_esc_front'] = cg.cg_x_lift_esc_front
            result['cg_x_lift_esc_rear'] = cg.cg_x_lift_esc_rear
            result['cg_x_cruise_esc'] = cg.cg_x_cruise_esc
            result['cg_x_booms'] = cg.cg_x_booms
            result['cg_x_central_pod'] = cg.cg_x_central_pod
            result['cg_x_booms'] = cg.cg_x_booms
            result['cg_x_landing_gear'] = cg.cg_x_landing_gear
            result['cg_x_autopilot'] = cg.cg_x_autopilot
            result['cg_x_power_module'] = cg.cg_x_power_module
            result['cg_x_bec'] = cg.cg_x_bec
            result['cg_x_autopilot_powerbrick'] = cg.cg_x_autopilot_powerbrick
            result['cg_x_esc'] = cg.cg_x_esc
            result['cg_x_gps'] = cg.cg_x_gps
            result['cg_x_airspeed'] = cg.cg_x_airspeed
            result['cg_x_rx'] = cg.cg_x_rx
            result['cg_x_tel'] = cg.cg_x_tel
            result['cg_x_camera'] = cg.cg_x_camera
            result['cg_x_servos'] = cg.cg_x_servos
            result['cg_x_misc'] = cg.cg_x_misc
            result['cg_x_payload'] = cg.cg_x_payload
            result['wing_le_x_m'] = cg.wing_le_x_m
            result['tail_le_x_m'] = cg.tail_le_x_m
            result['vertical_tail_le_x_m'] = cg.vertical_tail_le_x_m
            result['x_primary_wing_spar_m'] = cg.x_primary_wing_spar_m
            result['x_secondary_wing_spar_m'] = cg.x_secondary_wing_spar_m
            result['x_primary_tail_spar_m'] = cg.x_primary_tail_spar_m
            result['x_secondary_tail_spar_m'] = cg.x_secondary_tail_spar_m
            result['x_primary_vertical_tail_spar_m'] = cg.x_primary_vertical_tail_spar_m
            result['x_secondary_vertical_tail_spar_m'] = cg.x_secondary_vertical_tail_spar_m
            result['x_longerons_center_m'] = cg.x_longerons_center_m
        if self.mass_properties is not None:
            mp = self.mass_properties
            result['mass_takeoff_kg'] = mp.takeoff_mass_kg
            result['mass_end_kg'] = mp.end_mass_kg
            result['x_cg_takeoff_m'] = mp.x_cg_takeoff_m
            result['x_cg_end_m'] = mp.x_cg_end_m
            result['cg_shift_m'] = mp.cg_shift_m
            result['hydrogen_takeoff_kg'] = mp.hydrogen_takeoff_kg
            result['hydrogen_end_kg'] = mp.hydrogen_end_kg
        result['drag_cruise_N'] = self.aircraft.drag_cruise_N
        result['cruise_power_W'] = self.aircraft.cruise_power_W
        result['cruise_power_elec_W'] = self.aircraft.cruise_power_elec_W
        result['x_np_m'] = self.aircraft.x_np_m
        result['static_margin'] = self.aircraft.static_margin
        result['static_margin_percent_MAC'] = self.aircraft.static_margin_percent_MAC

        # Wing
        result['wing_area_S_m2'] = self.wing.geometry.area_m2
        result['span_b_m'] = self.wing.geometry.span_m
        result['AR'] = self.wing.geometry.AR
        result['c_root_m'] = self.wing.geometry.c_root_m
        result['c_tip_m'] = self.wing.geometry.c_tip_m
        result['MAC_m'] = self.wing.geometry.MAC_m
        result['y_MAC_from_centerline_m'] = self.wing.geometry.y_MAC_from_centerline_m
        result['x_ac_from_centerline_m'] = self.wing.geometry.ac_on_MAC_m
        result['x_ac_wing_m'] = self.wing.x_ac_wing_m
        result['CL_cruise'] = self.wing.aerodynamics.CL_cruise
        result['CL_max'] = self.wing.aerodynamics.CL_max
        result['CD0'] = self.wing.aerodynamics.CD0
        result['CD_cruise'] = self.wing.aerodynamics.CD_cruise
        result['CD0_fuselage'] = self.wing.aerodynamics.CD0_fuselage
        result['wing_wetted_area_m2'] = self.wing.wetted_area_m2
        result['CM_cruise'] = self.wing.aerodynamics.CM_cruise
        result['CM_max'] = self.wing.aerodynamics.CM_max
        result['alpha_cruise'] = self.wing.aerodynamics.alpha_cruise_deg
        result['L_over_D_cruise'] = self.wing.aerodynamics.L_over_D_cruise
        result['V_stall'] = self.wing.aerodynamics.V_stall
        result['Re_cruise'] = self.wing.aerodynamics.Re_cruise
        result['wing_bending_moment_Nm'] = self.wing.structure.bending_moment_Nm
        result['wing_torsion_Nm'] = self.wing.structure.torsion_Nm
        result['h_m'] = self.wing.structure.h_m
        result['t_skin_root_m'] = self.wing.structure.t_skin_root_m
        result['t_skin_tip_m'] = self.wing.structure.t_skin_tip_m
        wg = self.wing.structure
        _both = 2.0  # full wing (both sides); structure stores one lifting surface
        result['W_skin_kg'] = wg.W_skin_kg * _both
        result['t_web_root_m'] = wg.t_web_root_m
        result['t_web_tip_m'] = wg.t_web_tip_m
        result['W_web_kg'] = wg.W_web_kg * _both
        result['spar_area_root_m2'] = wg.spar_area_root_m2
        result['spar_area_tip_m2'] = wg.spar_area_tip_m2
        result['W_spars_kg'] = wg.W_spars_kg * _both
        result['N_ribs'] = wg.N_ribs
        result['rib_spacing_m'] = wg.rib_spacing_m
        result['W_ribs_kg'] = wg.W_ribs_kg * _both
        result['W_wings_kg'] = wg.W_total_kg * _both
        result['Wing Moment_Nm'] = self.wing.moment_at_cg_Nm
        result['wing_stiffener_N'] = self.wing.wing_stiffener_N
        result['wing_stiffener_spacing_m'] = self.wing.wing_stiffener_spacing_m
        result['wing_stiffener_thickness_m'] = self.wing.wing_stiffener_thickness_m
        result['wing_stiffener_height_m'] = self.wing.wing_stiffener_height_m
        result['wing_stiffener_area_one_mm2'] = self.wing.wing_stiffener_area_one_mm2
        result['wing_stiffener_I_one_mm4'] = self.wing.wing_stiffener_I_one_mm4
        result['wing_stiffener_tau_skin_MPa'] = self.wing.wing_stiffener_tau_skin_MPa
        result['wing_stiffener_tau_cr_panel_MPa'] = self.wing.wing_stiffener_tau_cr_panel_MPa
        result['wing_stiffener_q_shear_N_per_m'] = self.wing.wing_stiffener_q_shear_N_per_m
        result['wing_stiffener_aspect_ratio'] = self.wing.wing_stiffener_aspect_ratio
        result['wing_stiffener_mass_kg'] = self.wing.wing_stiffener_mass_kg * _both
        result['wing_stiffener_panel_ok'] = self.wing.wing_stiffener_panel_ok
        result['wing_spar_tip_deflection_m'] = self.wing.wing_spar_tip_deflection_m
        result['wing_spar_tip_deflection_mm'] = self.wing.wing_spar_tip_deflection_mm
        result['wing_spar_root_slope_rad'] = self.wing.wing_spar_root_slope_rad
        result['wing_spar_root_slope_deg'] = self.wing.wing_spar_root_slope_deg

        # Tail
        result['tail_area_S_m2'] = self.tail.geometry.area_m2
        result['tail_span_b_m'] = self.tail.geometry.span_m
        result['tail_AR'] = self.tail.geometry.AR
        result['tail_c_root_m'] = self.tail.geometry.c_root_m
        result['tail_c_tip_m'] = self.tail.geometry.c_tip_m
        result['tail_MAC_m'] = self.tail.geometry.MAC_m
        result['tail_y_MAC_from_centerline_m'] = self.tail.geometry.y_MAC_from_centerline_m
        result['tail_ac_on_MAC_m'] = self.tail.geometry.ac_on_MAC_m
        result['tail_x_ac_m'] = self.tail.x_ac_tail_m
        result['Tail Lift_N'] = self.tail.lift_N
        result['tail_CL_cruise'] = self.tail.aerodynamics.CL_cruise
        result['tail_CL_max'] = self.tail.aerodynamics.CL_max
        result['tail_CD0'] = self.tail.aerodynamics.CD0
        result['tail_CD_cruise'] = self.tail.aerodynamics.CD_cruise
        result['tail_CM_cruise'] = self.tail.aerodynamics.CM_cruise
        result['tail_CM_max'] = self.tail.aerodynamics.CM_max
        result['tail_alpha_body_deg'] = self.tail.alpha_body_deg
        result['tail_alpha_equiv_deg'] = self.tail.alpha_equiv_deg
        result['tail_L_over_D_cruise'] = self.tail.aerodynamics.L_over_D_cruise
        result['tail_V_stall'] = self.tail.aerodynamics.V_stall
        result['tail_Re_cruise'] = self.tail.aerodynamics.Re_cruise
        result['tail_bending_moment_Nm'] = self.tail.structure.bending_moment_Nm
        result['tail_torsion_Nm'] = self.tail.structure.torsion_Nm
        result['tail_h_m'] = self.tail.structure.h_m
        result['tail_t_skin_root_m'] = self.tail.structure.t_skin_root_m
        result['tail_t_skin_tip_m'] = self.tail.structure.t_skin_tip_m
        tt = self.tail.structure
        _both_tail = 2.0  # full horizontal tail pair; structure stores one lifting surface
        result['tail_W_skin_kg'] = tt.W_skin_kg * _both_tail
        result['tail_t_web_root_m'] = tt.t_web_root_m
        result['tail_t_web_tip_m'] = tt.t_web_tip_m
        result['tail_W_web_kg'] = tt.W_web_kg * _both_tail
        result['tail_spar_area_root_m2'] = tt.spar_area_root_m2
        result['tail_spar_area_tip_m2'] = tt.spar_area_tip_m2
        result['tail_W_spars_kg'] = tt.W_spars_kg * _both_tail
        result['tail_N_ribs'] = tt.N_ribs
        result['tail_rib_spacing_m'] = tt.rib_spacing_m
        result['tail_W_ribs_kg'] = tt.W_ribs_kg * _both_tail
        result['tail_W_tails_kg'] = tt.W_total_kg * _both_tail
        result['tail_CL_req'] = self.tail.CL_req
        result['tail_CL_no_elev'] = self.tail.CL_no_elev
        result['tail_dCL_required'] = self.tail.dCL_required
        result['tail_elevator_chord_ratio'] = self.tail.elevator_chord_ratio
        result['tail_eta_f'] = self.tail.eta_f
        result['tail_elevator_deflection_deg'] = self.tail.elevator_deflection_deg
        result['tail_incidence_deg'] = self.tail.tail_incidence_deg
        result['tail_cruise_elevator_deg'] = self.tail.cruise_elevator_deg
        result['elevator_hinge_moment_M_h_Nm'] = self.tail.elevator_hinge_moment_M_h_Nm
        result['elevator_area_S_e_m2'] = self.tail.elevator_area_S_e_m2
        result['elevator_chord_c_e_m'] = self.tail.elevator_chord_c_e_m
        result['servo_torque_T_servo_Nm'] = self.tail.servo_torque_T_servo_Nm
        result['servo_torque_with_margin_Nm'] = self.tail.servo_torque_with_margin_Nm
        result['tail_stiffener_N'] = self.tail.tail_stiffener_N
        result['tail_stiffener_spacing_m'] = self.tail.tail_stiffener_spacing_m
        result['tail_stiffener_thickness_m'] = self.tail.tail_stiffener_thickness_m
        result['tail_stiffener_height_m'] = self.tail.tail_stiffener_height_m
        result['tail_stiffener_area_one_mm2'] = self.tail.tail_stiffener_area_one_mm2
        result['tail_stiffener_I_one_mm4'] = self.tail.tail_stiffener_I_one_mm4
        result['tail_stiffener_tau_skin_MPa'] = self.tail.tail_stiffener_tau_skin_MPa
        result['tail_stiffener_tau_cr_panel_MPa'] = self.tail.tail_stiffener_tau_cr_panel_MPa
        result['tail_stiffener_q_shear_N_per_m'] = self.tail.tail_stiffener_q_shear_N_per_m
        result['tail_stiffener_aspect_ratio'] = self.tail.tail_stiffener_aspect_ratio
        result['tail_stiffener_mass_kg'] = self.tail.tail_stiffener_mass_kg * _both_tail
        result['tail_stiffener_panel_ok'] = self.tail.tail_stiffener_panel_ok
        result['tail_spar_tip_deflection_m'] = self.tail.tail_spar_tip_deflection_m
        result['tail_spar_tip_deflection_mm'] = self.tail.tail_spar_tip_deflection_mm
        result['tail_spar_root_slope_rad'] = self.tail.tail_spar_root_slope_rad
        result['tail_spar_root_slope_deg'] = self.tail.tail_spar_root_slope_deg
        result['tail_wetted_area_m2'] = self.tail.wetted_area_m2

        # Vertical Wing
        # VERTICAL MAIN WING REMOVED FOR NOW
        # result['vertical_wing_area_S_m2'] = self.vertical_wing.geometry.area_m2
        # result['vertical_wing_span_b_m'] = self.vertical_wing.geometry.span_m
        # result['vertical_wing_AR'] = self.vertical_wing.geometry.AR
        # result['vertical_wing_c_root_m'] = self.vertical_wing.geometry.c_root_m
        # result['vertical_wing_c_tip_m'] = self.vertical_wing.geometry.c_tip_m
        # result['vertical_wing_MAC_m'] = self.vertical_wing.geometry.MAC_m
        # result['vertical_wing_y_MAC_from_centerline_m'] = self.vertical_wing.geometry.y_MAC_from_centerline_m
        # result['vertical_wing_ac_on_MAC_m'] = self.vertical_wing.geometry.ac_on_MAC_m
        # result['vertical_wing_x_ac_m'] = self.vertical_wing.x_ac_wing_m
        # result['vertical_wing_CL_cruise'] = self.vertical_wing.aerodynamics.CL_cruise
        # result['vertical_wing_CL_max'] = self.vertical_wing.aerodynamics.CL_max
        # result['vertical_wing_CD0'] = self.vertical_wing.aerodynamics.CD0
        # result['vertical_wing_CD_cruise'] = self.vertical_wing.aerodynamics.CD_cruise
        # result['vertical_wing_CM_cruise'] = self.vertical_wing.aerodynamics.CM_cruise
        # result['vertical_wing_CM_max'] = self.vertical_wing.aerodynamics.CM_max
        # result['vertical_wing_alpha_cruise'] = self.vertical_wing.aerodynamics.alpha_cruise_deg
        # result['vertical_wing_L_over_D_cruise'] = self.vertical_wing.aerodynamics.L_over_D_cruise
        # result['vertical_wing_V_stall'] = self.vertical_wing.aerodynamics.V_stall
        # result['vertical_wing_Re_cruise'] = self.vertical_wing.aerodynamics.Re_cruise
        # result['vertical_wing_bending_moment_Nm'] = self.vertical_wing.structure.bending_moment_Nm
        # result['vertical_wing_torsion_Nm'] = self.vertical_wing.structure.torsion_Nm
        # result['vertical_wing_h_m'] = self.vertical_wing.structure.h_m
        # result['vertical_wing_t_skin_root_m'] = self.vertical_wing.structure.t_skin_root_m
        # result['vertical_wing_t_skin_tip_m'] = self.vertical_wing.structure.t_skin_tip_m
        # result['vertical_wing_W_skin_kg'] = self.vertical_wing.structure.W_skin_kg
        # result['vertical_wing_t_web_root_m'] = self.vertical_wing.structure.t_web_root_m
        # result['vertical_wing_t_web_tip_m'] = self.vertical_wing.structure.t_web_tip_m
        # result['vertical_wing_W_web_kg'] = self.vertical_wing.structure.W_web_kg
        # result['vertical_wing_spar_area_root_m2'] = self.vertical_wing.structure.spar_area_root_m2
        # result['vertical_wing_spar_area_tip_m2'] = self.vertical_wing.structure.spar_area_tip_m2
        # result['vertical_wing_W_spars_kg'] = self.vertical_wing.structure.W_spars_kg
        # result['vertical_wing_N_ribs'] = self.vertical_wing.structure.N_ribs
        # result['vertical_wing_W_ribs_kg'] = self.vertical_wing.structure.W_ribs_kg
        # result['vertical_wing_W_total_kg'] = self.vertical_wing.structure.W_total_kg
        
        # Vertical Tail
        result['vertical_tail_area_S_m2'] = self.vertical_tail.geometry.area_m2
        result['vertical_tail_span_b_m'] = self.vertical_tail.geometry.span_m
        result['vertical_tail_AR'] = self.vertical_tail.geometry.AR
        result['vertical_tail_c_root_m'] = self.vertical_tail.geometry.c_root_m
        result['vertical_tail_c_tip_m'] = self.vertical_tail.geometry.c_tip_m
        result['vertical_tail_MAC_m'] = self.vertical_tail.geometry.MAC_m
        result['vertical_tail_y_MAC_from_centerline_m'] = self.vertical_tail.geometry.y_MAC_from_centerline_m
        result['vertical_tail_ac_on_MAC_m'] = self.vertical_tail.geometry.ac_on_MAC_m
        result['vertical_tail_x_ac_m'] = self.vertical_tail.x_ac_tail_m
        result['vertical_tail_CL_cruise'] = self.vertical_tail.aerodynamics.CL_cruise
        result['vertical_tail_CL_max'] = self.vertical_tail.aerodynamics.CL_max
        result['vertical_tail_CD0'] = self.vertical_tail.aerodynamics.CD0
        result['vertical_tail_CD_cruise'] = self.vertical_tail.aerodynamics.CD_cruise
        result['vertical_tail_CM_cruise'] = self.vertical_tail.aerodynamics.CM_cruise
        result['vertical_tail_CM_max'] = self.vertical_tail.aerodynamics.CM_max
        result['vertical_tail_alpha_cruise'] = self.vertical_tail.aerodynamics.alpha_cruise_deg
        result['vertical_tail_L_over_D_cruise'] = self.vertical_tail.aerodynamics.L_over_D_cruise
        result['vertical_tail_V_stall'] = self.vertical_tail.aerodynamics.V_stall
        result['vertical_tail_Re_cruise'] = self.vertical_tail.aerodynamics.Re_cruise
        result['vertical_tail_bending_moment_Nm'] = self.vertical_tail.structure.bending_moment_Nm
        result['vertical_tail_torsion_Nm'] = self.vertical_tail.structure.torsion_Nm
        result['vertical_tail_h_m'] = self.vertical_tail.structure.h_m
        result['vertical_tail_t_skin_root_m'] = self.vertical_tail.structure.t_skin_root_m
        result['vertical_tail_t_skin_tip_m'] = self.vertical_tail.structure.t_skin_tip_m
        vt = self.vertical_tail.structure
        _both_vt = 2.0  # full vertical tail pair; structure stores one lifting surface
        result['vertical_tail_W_skin_kg'] = vt.W_skin_kg * _both_vt
        result['vertical_tail_t_web_root_m'] = vt.t_web_root_m
        result['vertical_tail_t_web_tip_m'] = vt.t_web_tip_m
        result['vertical_tail_W_web_kg'] = vt.W_web_kg * _both_vt
        result['vertical_tail_spar_area_root_m2'] = vt.spar_area_root_m2
        result['vertical_tail_spar_area_tip_m2'] = vt.spar_area_tip_m2
        result['vertical_tail_W_spars_kg'] = vt.W_spars_kg * _both_vt
        result['vertical_tail_N_ribs'] = vt.N_ribs
        result['vertical_tail_rib_spacing_m'] = vt.rib_spacing_m
        result['vertical_tail_W_ribs_kg'] = vt.W_ribs_kg * _both_vt
        result['vertical_tail_W_total_kg'] = vt.W_total_kg * _both_vt
        result['vertical_tail_stiffener_N'] = self.vertical_tail.vertical_tail_stiffener_N
        result['vertical_tail_stiffener_spacing_m'] = self.vertical_tail.vertical_tail_stiffener_spacing_m
        result['vertical_tail_stiffener_thickness_m'] = self.vertical_tail.vertical_tail_stiffener_thickness_m
        result['vertical_tail_stiffener_height_m'] = self.vertical_tail.vertical_tail_stiffener_height_m
        result['vertical_tail_stiffener_area_one_mm2'] = self.vertical_tail.vertical_tail_stiffener_area_one_mm2
        result['vertical_tail_stiffener_I_one_mm4'] = self.vertical_tail.vertical_tail_stiffener_I_one_mm4
        result['vertical_tail_stiffener_tau_skin_MPa'] = self.vertical_tail.vertical_tail_stiffener_tau_skin_MPa
        result['vertical_tail_stiffener_tau_cr_panel_MPa'] = self.vertical_tail.vertical_tail_stiffener_tau_cr_panel_MPa
        result['vertical_tail_stiffener_q_shear_N_per_m'] = self.vertical_tail.vertical_tail_stiffener_q_shear_N_per_m
        result['vertical_tail_stiffener_aspect_ratio'] = self.vertical_tail.vertical_tail_stiffener_aspect_ratio
        result['vertical_tail_stiffener_mass_kg'] = self.vertical_tail.vertical_tail_stiffener_mass_kg * _both_vt
        result['vertical_tail_stiffener_panel_ok'] = self.vertical_tail.vertical_tail_stiffener_panel_ok
        result['vertical_tail_spar_tip_deflection_m'] = self.vertical_tail.vertical_tail_spar_tip_deflection_m
        result['vertical_tail_spar_tip_deflection_mm'] = self.vertical_tail.vertical_tail_spar_tip_deflection_mm
        result['vertical_tail_spar_root_slope_rad'] = self.vertical_tail.vertical_tail_spar_root_slope_rad
        result['vertical_tail_spar_root_slope_deg'] = self.vertical_tail.vertical_tail_spar_root_slope_deg
        result['vertical_tail_wetted_area_m2'] = self.vertical_tail.wetted_area_m2

        # Fuselage
        result['fuselage_t_shell_m'] = self.fuselage.t_shell_m
        result['fuselage_wetted_area_m2'] = self.fuselage.wetted_area_m2
        result['W_fuselage_kg'] = self.fuselage.W_fuselage_kg
        result['W_fuselage_nose_kg'] = getattr(self.fuselage, 'W_fuselage_nose_kg', 0.0)
        result['W_fuselage_section_C_kg'] = getattr(self.fuselage, 'W_fuselage_section_C_kg', 0.0)
        result['W_fuselage_section_B_kg'] = getattr(self.fuselage, 'W_fuselage_section_B_kg', 0.0)
        result['W_fuselage_section_A_kg'] = getattr(self.fuselage, 'W_fuselage_section_A_kg', 0.0)
        result['W_fuselage_tail_kg'] = getattr(self.fuselage, 'W_fuselage_tail_kg', 0.0)
        result['fuselage_nose_longeron_boss_height_m'] = self.fuselage.fuselage_nose_longeron_boss_height_m
        result['fuselage_W_shell_kg'] = getattr(self.fuselage, 'W_shell_kg', 0.0)
        result['fuselage_W_frames_total_kg'] = getattr(self.fuselage, 'W_frames_total_kg', 0.0)
        result['fuselage_W_stringers_total_kg'] = getattr(self.fuselage, 'W_stringers_total_kg', 0.0)
        result['fuselage_W_longerons_total_kg'] = getattr(self.fuselage, 'W_longerons_total_kg', 0.0)
        result['fuselage_T_design_Nm'] = getattr(self.fuselage, 'T_design_Nm', 0.0)
        result['fuselage_N_frames'] = getattr(self.fuselage, 'N_frames', 0.0)
        result['fuselage_frame_spacing_m'] = getattr(self.fuselage, 'frame_spacing_m', 0.0)
        result['fuselage_frame_annulus_area_mm2'] = getattr(self.fuselage, 'frame_annulus_area_mm2', 0.0)
        result['fuselage_N_stringers'] = getattr(self.fuselage, 'N_stringers', 0.0)
        result['fuselage_stringer_spacing_m'] = getattr(self.fuselage, 'stringer_spacing_m', 0.0)
        result['fuselage_stringer_thickness_m'] = getattr(self.fuselage, 'stringer_thickness_m', 0.0)
        result['fuselage_stringer_height_m'] = getattr(self.fuselage, 'stringer_height_m', 0.0)
        result['fuselage_stringer_area_one_mm2'] = getattr(self.fuselage, 'stringer_area_one_mm2', 0.0)
        result['fuselage_N_longerons'] = getattr(self.fuselage, 'N_longerons', 0.0)
        result['fuselage_longeron_OD_m'] = getattr(self.fuselage, 'longeron_OD_m', 0.0)
        result['fuselage_longeron_ID_m'] = getattr(self.fuselage, 'longeron_ID_m', 0.0)
        result['fuselage_longeron_A_one_mm2'] = getattr(self.fuselage, 'longeron_A_one_mm2', 0.0)
        result['fuselage_sigma_x_MPa'] = getattr(self.fuselage, 'sigma_x_MPa', 0.0)
        result['fuselage_tau_xy_MPa'] = getattr(self.fuselage, 'tau_xy_MPa', 0.0)
        result['fuselage_sigma_cr_MPa'] = getattr(self.fuselage, 'sigma_cr_MPa', 0.0)
        result['fuselage_tau_cr_MPa'] = getattr(self.fuselage, 'tau_cr_MPa', 0.0)
        result['fuselage_q_torsion_N_per_m'] = getattr(self.fuselage, 'q_torsion_N_per_m', 0.0)
        result['fuselage_panel_aspect_ratio'] = getattr(self.fuselage, 'panel_aspect_ratio', 0.0)
        result['fuselage_panel_interaction_index'] = getattr(self.fuselage, 'panel_interaction_index', 0.0)
        result['fuselage_panel_comp_ok'] = getattr(self.fuselage, 'panel_comp_ok', 0.0)
        result['fuselage_panel_shear_ok'] = getattr(self.fuselage, 'panel_shear_ok', 0.0)
        result['fuselage_panel_interaction_ok'] = getattr(self.fuselage, 'panel_interaction_ok', 0.0)
        
        # Battery
        if self.battery:
            result['battery_name'] = self.battery.name
            result['battery_voltage_V'] = self.battery.voltage_V
            result['battery_capacity_mAh'] = self.battery.capacity_mAh
            result['battery_mass_kg'] = self.battery.mass_kg
            result['battery_length_m'] = self.battery.length_m
            result['battery_width_m'] = self.battery.width_m
            result['battery_height_m'] = self.battery.height_m
            result['battery_usable_energy_Wh'] = self.battery.usable_energy_Wh
            result['battery_endurance_min_at_cruise'] = self.battery.endurance_min_at_cruise
            result['battery_C_at_cruise'] = self.battery.C_at_cruise
            result['battery_meets_endurance_req'] = self.battery.meets_endurance_req
        
        # Propulsion
        if self.propulsion:
            result['n_propellers'] = self.propulsion.n_propellers
            result['propulsion_name'] = self.propulsion.name
            result['prop_diameter_in'] = self.propulsion.prop_diameter_in
            result['prop_pitch_in'] = self.propulsion.prop_pitch_in
            result['motor_kv'] = self.propulsion.motor_kv
            result['motor_mass_kg'] = self.propulsion.motor_mass_kg
            result['prop_mass_kg'] = self.propulsion.prop_mass_kg
            result['total_motor_mass_kg'] = self.propulsion.total_motor_mass_kg
            result['total_prop_mass_kg'] = self.propulsion.total_prop_mass_kg
            result['total_propulsion_mass_kg'] = self.propulsion.total_propulsion_mass_kg
            result['max_thrust_per_motor_N'] = self.propulsion.max_thrust_per_motor_N
            result['required_thrust_per_motor_N'] = self.propulsion.required_thrust_per_motor_N
            result['max_power_per_motor_W'] = self.propulsion.max_power_per_motor_W
            result['required_power_per_motor_W'] = self.propulsion.required_power_per_motor_W
            result['propulsion_meets_thrust_req'] = self.propulsion.meets_thrust_req
            result['propulsion_meets_power_req'] = self.propulsion.meets_power_req
            if self.propulsion.est_current_A_per_motor_at_cruise is not None:
                result['est_current_A_per_motor_at_cruise'] = self.propulsion.est_current_A_per_motor_at_cruise
        
        # Avionics
        result['autopilot_name'] = self.avionics.autopilot_name
        result['autopilot_mass_kg'] = self.avionics.autopilot_mass_kg
        result['gps_mass_kg'] = self.avionics.gps_mass_kg
        result['airspeed_mass_kg'] = self.avionics.airspeed_mass_kg
        result['rc_rx_mass_kg'] = self.avionics.rc_rx_mass_kg
        result['telemetry_mass_kg'] = self.avionics.telemetry_mass_kg
        result['power_module_mass_kg'] = self.avionics.power_module_mass_kg
        result['bec_mass_kg'] = self.avionics.bec_mass_kg
        result['autopilot_powerbrick_mass_kg'] = self.avionics.autopilot_powerbrick_mass_kg
        result['servos_mass_kg'] = self.avionics.servos_mass_kg
        result['esc_mass_total_kg'] = self.avionics.esc_mass_total_kg
        result['misc_electronics_mass_kg'] = self.avionics.misc_electronics_mass_kg
        result['camera_mass_kg'] = self.avionics.camera_mass_kg
        result['avionics_electrics_total_kg'] = self.avionics.total_avionics_electrics_mass_kg
        
        result['inputs'] = self.inputs
        
        return result

