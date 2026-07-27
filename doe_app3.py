# doe_app.py
from __future__ import annotations

import itertools
import json
import os
import sys
import traceback
import importlib
from datetime import datetime
from io import BytesIO
from typing import Any, Dict, List, Tuple, Optional, get_origin, get_args

import numpy as np
import pandas as pd
import streamlit as st
import matplotlib.pyplot as plt

import design_inputs
import script


# ============================================================
# HARD-CODED DROPDOWN OPTIONS
# ============================================================

OBJECTIVE_OPTIONS: List[Tuple[str, str]] = [
    ("MTOW (W0_mass_kg)", "W0_mass_kg"),
    ("cruise electric power (cruise_power_elec_W)", "cruise_power_elec_W"),
    ("cruise drag (drag_cruise_N)", "drag_cruise_N"),
    ("cruise L/D (L_over_D_cruise)", "L_over_D_cruise"),
    ("stall speed (V_stall)", "V_stall"),
    ("mission-limited cruise time (mission_profile.cruise_time_max_min)", "mission_profile.cruise_time_max_min"),
    ("mission total energy (mission_profile.total_energy_Wh)", "mission_profile.total_energy_Wh"),
    ("mission peak electrical power (mission_profile.peak_elec_power_W)", "mission_profile.peak_elec_power_W"),
]

DEFAULT_OBJECTIVE_KEY = "W0_mass_kg"
DEFAULT_VARY_PARAMS = ["wing_loading", "speed"]

DEFAULT_EXCEL_VISIBLE_SUCCESS_COLUMNS: List[str] = [
    "cg_x_wing",
    "payload_mass",
    "tail_volume_ratio",
    "wing_loading",
    "W0_mass_kg",
    "drag_cruise_N",
    "cruise_power_elec_W",
    "V_stall",
    "res.drag_cruise_N",
    "res.static_margin_percent_MAC",
    "res.span_b_m",
    "res.alpha_cruise",
    "res.tail_span_b_m",
    "res.tail_incidence_deg",
    "res.tail_cruise_elevator_deg",
]

# Pretty labels for the most important/common parameters.
# Any DesignInputs scalar field not listed here is still included automatically
# with a generated label.
PRETTY_PARAM_LABELS: Dict[str, str] = {
    "payload_mass": "Payload mass [kg]",
    "payload_length": "Payload length [m]",
    "wing_loading": "Wing loading W/S [N/m²]",
    "speed": "Cruise speed [m/s]",
    "dive_speed": "Dive speed [m/s]",
    "fuselage_length": "Fuselage length [m]",
    "fuselage_OD": "Fuselage outer diameter OD [m]",
    "fuselage_nose_length": "Fuselage nose length [m]",
    "fuselage_tail_length": "Fuselage tail length [m]",
    "fuselage_nonopt_factor": "Fuselage non-opt factor [-]",
    "fuselage_body_length": "Fuselage body length [m]",
    "tail_c_root": "Tail root chord [m]",
    "tail_volume_ratio": "Tail volume ratio [-]",
    "vertical_tail_volume_ratio": "Vertical tail volume ratio [-]",
    "c_root": "Wing root chord c_root [m]",
    "transition_fit_clearance": "Transition fit clearance [m]",
    "payload_fraction_hist": "Payload fraction historical [-]",
    "export_inventor_xml": "Export Inventor XML",
    "inventor_xml_path": "Inventor XML path",
    "cruise_endurance_min": "Cruise endurance [min]",
    "n_ult": "Ultimate load factor n_ult [-]",
    "carbon_fiber_safety_factor": "Carbon fiber safety factor [-]",
    "airfoil_n_points": "Airfoil discretization points",
    "horizontal_NACA_code": "Horizontal NACA code",
    "vertical_NACA_code": "Vertical NACA code",
    "sweep_angle_deg": "Wing sweep angle [deg]",
    "fuselage_longeron_boss_height": "Fuselage longeron boss height [m]",
    "fuselage_longeron_boss_thickness": "Fuselage longeron boss thickness [m]",
    "fuselage_longeron_diameter": "Fuselage longeron diameter [m]",
    "fuselage_torque_frame_diameter": "Fuselage torque frame diameter [m]",
    "fuselage_wing_interface_length": "Fuselage wing interface length [m]",
    "fuselage_wing_interface_fit_clearance": "Wing interface fit clearance [m]",
    "fuselage_wing_interface_insert_depth_factor": "Wing interface insert depth factor [-]",
    "fuselage_wing_interface_sleeve_thickness": "Wing interface sleeve thickness [m]",
    "fuselage_axial_restraint_bolt_length": "Fuselage axial restraint bolt length [m]",
    "tail_taper_ratio": "Tail taper ratio [-]",
    "tail_elevator_chord_ratio": "Elevator chord ratio (tail) [-]",
    "tail_e_oswald": "Oswald efficiency e (tail)",
    "tail_sweep_angle_deg": "Tail sweep angle [deg]",
    "tail_efficiency_factor": "Tail efficiency factor [-]",
    "tail_target_elevator_deg": "Tail target elevator [deg]",
    "tail_elevator_limit_deg": "Tail elevator limit [deg]",
    "tail_incidence_limit_deg": "Tail incidence limit [deg]",
    "tail_effective_alpha_limit_deg": "Tail effective alpha limit [deg]",
    "tail_stall_margin_deg": "Tail stall margin [deg]",
    "tail_span_extension_m": "Tail span extension [m]",
    "tail_downwash_k": "Tail downwash k [-]",
    "tail_downwash_offset_deg": "Tail downwash offset [deg]",
    "tail_downwash_min_deg": "Tail downwash min [deg]",
    "tail_downwash_max_deg": "Tail downwash max [deg]",
    "vertical_tail_Kf1": "Vertical tail Kf1 [-]",
    "vertical_tail_eta_v": "Vertical tail eta_v [-]",
    "vertical_tail_dsigma_dbeta": "Vertical tail dσ/dβ [-]",
    "prop_fuselage_clearance_m": "Prop-fuselage clearance [m]",
    "control_surface_drag_k": "Control surface drag factor k",
    "tail_elevator_span_start": "Tail elevator span start [-]",
    "tail_elevator_span_end": "Tail elevator span end [-]",
    "tail_elevator_max_up_deg": "Tail elevator max up [deg]",
    "tail_elevator_max_down_deg": "Tail elevator max down [deg]",
    "tail_cg_root_chord_ratio": "Tail CG root chord ratio [-]",
    "tail_elevator_rod_support_width": "Tail elevator rod support width [m]",
    "tail_elevator_max_deflection_deg": "Tail elevator max deflection [deg]",
    "elevator_Ch_delta_per_rad": "Elevator Ch delta per rad [-]",
    "servo_arm_radius_m": "Servo arm radius [m]",
    "elevator_horn_radius_m": "Elevator horn radius [m]",
    "servo_efficiency_eta": "Servo efficiency eta [-]",
    "servo_torque_margin": "Servo torque margin [-]",
    "servo_width": "Servo width [m]",
    "servo_height": "Servo height [m]",
    "servo_depth": "Servo depth [m]",
    "servo_cables_hole_width": "Servo cables hole width [m]",
    "servo_cables_hole_height": "Servo cables hole height [m]",
    "servo_mount_ears_width": "Servo mount ears width [m]",
    "servo_mount_ears_thickness": "Servo mount ears thickness [m]",
    "servo_mount_ears_location_from_top": "Servo mount ears location from top [m]",
    "servo_mount_bolt_length": "Servo mount bolt length [m]",
    "servo_arm_length": "Servo arm length [m]",
    "servo_housing_thickness": "Servo housing thickness [m]",
    "servo_cover_thickness": "Servo cover thickness [m]",
    "servo_cover_bolt_length": "Servo cover bolt length [m]",
    "tail_dbox_width_ratio": "Tail D-box width ratio [-]",
    "tail_elevator_rod_id_over_od": "Tail elevator rod ID/OD [-]",
    "elevator_rod_bearing_length_m": "Elevator rod bearing length [m]",
    "tail_elevator_tau_override": "Tail elevator tau override [-]",
    "tail_eta_h_maneuver": "Tail eta_h maneuver [-]",
    "vertical_tail_rudder_chord_ratio": "Vertical tail rudder chord ratio [-]",
    "vertical_tail_rudder_span_start": "Vertical tail rudder span start [-]",
    "vertical_tail_rudder_span_end": "Vertical tail rudder span end [-]",
    "vertical_tail_rudder_max_left_deg": "Vertical tail rudder max left [deg]",
    "vertical_tail_rudder_max_right_deg": "Vertical tail rudder max right [deg]",
    "vertical_tail_rudder_rod_oversize_factor": "Vertical tail rudder rod oversize factor [-]",
    "NACA2410_zero_lift_angle_of_attack_deg": "NACA2410 zero-lift AoA [deg]",
    "NACA0012_zero_lift_angle_of_attack_deg": "NACA0012 zero-lift AoA [deg]",
    "NACA0015_zero_lift_angle_of_attack_deg": "NACA0015 zero-lift AoA [deg]",
    "taper_ratio": "Wing taper ratio [-]",
    "wing_plain_flap_chord_ratio": "Plain flap chord ratio (wing)",
    "e_oswald": "Oswald efficiency e (wing)",
    "wing_spar_id_over_od": "Wing spar ID/OD [-]",
    "wing_dbox_width_ratio": "Wing D-box width ratio [-]",
    "wing_second_spar_scale": "Wing second spar scale [-]",
    "wing_bolt_hatch_depth": "Wing bolt hatch depth [m]",
    "wing_cg_root_chord_ratio": "Wing CG root chord ratio [-]",
    "wing_spar_boss_factor": "Wing spar boss factor [-]",
    "wing_assembly_interface_depth": "Wing assembly interface depth [m]",
    "wing_assembly_interface_sleeve_thickness": "Wing assembly interface sleeve thickness [m]",
    "fuselage_body_section_B_length": "Fuselage body section B length [m]",
    "fuselage_body_section_A_length": "Fuselage body section A length [m]",
    "fuselage_body_section_C_length": "Fuselage body section C length [m]",
    "wing_specific_weight_kg_per_m": "Wing specific weight [kg/m]",
    "tail_wing_specific_weight_kg_per_m": "Tail wing specific weight [kg/m]",
    "vertical_tail_wing_specific_weight_kg_per_m": "Vertical tail wing specific weight [kg/m]",
    "fuselage_nose_specific_weight_kg_per_m": "Fuselage nose specific weight [kg/m]",
    "fuselage_A_specific_weight_kg_per_m": "Fuselage A specific weight [kg/m]",
    "fuselage_B_specific_weight_kg_per_m": "Fuselage B specific weight [kg/m]",
    "fuselage_C_specific_weight_kg_per_m": "Fuselage C specific weight [kg/m]",
    "fuselage_tail_specific_weight_kg_per_m": "Fuselage tail specific weight [kg/m]",
    "fuselage_bolt_hatch_depth": "Fuselage bolt hatch depth [m]",
    "fuselage_nose_longeron_length": "Fuselage nose longeron length [m]",
    "fuselage_tail_longeron_length": "Fuselage tail longeron length [m]",
    "fuselage_longeron_lenght": "Fuselage longeron length [m]",
    "fuselage_rib_thickness": "Fuselage rib thickness [m]",
    "fuselage_stinger_thickness": "Fuselage stinger thickness [m]",
    "fuselage_stinger_height": "Fuselage stinger height [m]",
    "fuselage_bulkhead_thickness": "Fuselage bulkhead thickness [m]",
    "fuselage_bulkhead_inner_OD": "Fuselage bulkhead inner OD [m]",
    "fuselage_bulkhead_radial_ring_width_m": "Fuselage bulkhead radial ring width [m]",
    "fuselage_bulkhead_radial_ring_thickness": "Fuselage bulkhead radial ring thickness [m]",
    "fuselage_B_front_bulkhead_location": "Fuselage B front bulkhead location [m]",
    "fuselage_B_rear_bulkhead_location": "Fuselage B rear bulkhead location [m]",
    "fuselage_tail_front_bulkhead_location": "Fuselage tail front bulkhead location [m]",
    "fuselage_tail_rear_bulkhead_location": "Fuselage tail rear bulkhead location [m]",
    "fuselage_design_torsion_Nm": "Fuselage design torsion [Nm]",
    "fuselage_frame_spacing_target_m": "Fuselage frame spacing target [m]",
    "fuselage_frame_thickness": "Fuselage frame thickness [m]",
    "fuselage_stringer_spacing_target_m": "Fuselage stringer spacing target [m]",
    "fuselage_stringer_thickness": "Fuselage stringer thickness [m]",
    "fuselage_stringer_height": "Fuselage stringer height [m]",
    "fuselage_panel_buckling_margin": "Fuselage panel buckling margin [-]",
    "fuselage_panel_kc": "Fuselage panel kc [-]",
    "fuselage_panel_ks": "Fuselage panel ks [-]",
    "fuselage_longeron_id_over_od": "Fuselage longeron ID/OD [-]",
    "fuselage_longeron_bending_load_fraction": "Fuselage longeron bending load fraction [-]",
    "printed_vertical_small_hole_additional_clearance": "Printed small hole additional clearance [m]",
    "printed_parts_sf": "Printed parts safety factor [-]",
    "filament_density": "Filament density [kg/m³]",
    "filament_tensile_strength_MPa": "Filament tensile strength [MPa]",
    "filament_max_bending_stress_MPa": "Filament max bending stress [MPa]",
    "filament_compressive_strength_MPa": "Filament compressive strength [MPa]",
    "filament_shear_strength_MPa": "Filament shear strength [MPa]",
    "filament_E_Pa": "Filament Young's modulus [Pa]",
    "filament_nu": "Filament Poisson ratio [-]",
    "stiffener_panel_buckling_margin": "Stiffener panel buckling margin [-]",
    "stiffener_panel_k_shear": "Stiffener panel shear factor [-]",
    "stiffener_t_min_factor": "Stiffener min thickness factor [-]",
    "stiffener_t_max_factor": "Stiffener max thickness factor [-]",
    "stiffener_h_min_factor": "Stiffener min height factor [-]",
    "stiffener_h_max_factor": "Stiffener max height factor [-]",
    "carbon_fiber_density": "Carbon fiber density [kg/m³]",
    "carbon_fiber_max_bending_stress_MPa": "Carbon fiber max bending stress [MPa]",
    "carbon_fiber_E_Pa": "Carbon fiber Young's modulus [Pa]",
    "carbon_fiber_comp_strain_allow": "Carbon fiber compressive strain allow [-]",
    "snap_fit_extension_length": "Snap fit extension length [m]",
    "propeller_efficiency": "Propeller efficiency [-]",
    "n_propellers": "Number of propellers",
    "propulsion_margin": "Propulsion thrust margin [-]",
    "propulsion_power_margin": "Propulsion power margin [-]",
    "prop_clearance_from_wing_LE": "Prop clearance from wing LE [m]",
    "hole_boss_factor": "Hole boss factor [-]",
    "bolt_pocket_diameter_clearance": "Bolt pocket diameter clearance [m]",
    "tube_clamp_boss_width": "Tube clamp boss width [m]",
    "motor_OD": "Motor outer diameter [m]",
    "motor_body_length": "Motor body length [m]",
    "prop_mount_bolt_circular_location": "Prop mount bolt circular location [m]",
    "motor_mount_bolt_base_thickness": "Motor mount bolt base thickness [m]",
    "motor_cabling_clearance_width": "Motor cabling clearance width [m]",
    "motor_cabling_clearance_height": "Motor cabling clearance height [m]",
    "motor_mount_bolt_length": "Motor mount bolt length [m]",
    "tail_wing_mount_bolt_length": "Tail wing mount bolt length [m]",
    "elevator_pushrod_OD": "Elevator pushrod OD [m]",
    "clamp_M8_ID": "Clamp M8 ID [m]",
    "clamp_M8_OD": "Clamp M8 OD [m]",
    "clamp_M8_width": "Clamp M8 width [m]",
    "clamp_M8_washer_OD": "Clamp M8 washer OD [m]",
    "clamp_M8_washer_thickness": "Clamp M8 washer thickness [m]",
    "threaded_insert_M2_fixing_hole_OD": "Threaded insert M2 fixing hole OD [m]",
    "threaded_insert_M2_lenght": "Threaded insert M2 length [m]",
    "bolt_M2_OD": "Bolt M2 OD [m]",
    "bolt_M2_head_width": "Bolt M2 head width [m]",
    "bolt_M2_hole_OD": "Bolt M2 hole OD [m]",
    "bolt_M2_head_OD": "Bolt M2 head OD [m]",
    "threaded_insert_M3_fixing_hole_OD": "Threaded insert M3 fixing hole OD [m]",
    "threaded_insert_M3_lenght": "Threaded insert M3 length [m]",
    "threaded_insert_M3_lenght_short": "Threaded insert M3 short length [m]",
    "bolt_M3_OD": "Bolt M3 OD [m]",
    "bolt_M3_head_width": "Bolt M3 head width [m]",
    "bolt_M3_hole_OD": "Bolt M3 hole OD [m]",
    "bolt_M3_head_OD": "Bolt M3 head OD [m]",
    "spring_washer_M3_OD": "Spring washer M3 OD [m]",
    "spring_washer_M3_thickness": "Spring washer M3 thickness [m]",
    "threaded_insert_M4_fixing_hole_OD": "Threaded insert M4 fixing hole OD [m]",
    "threaded_insert_M4_lenght": "Threaded insert M4 length [m]",
    "bolt_M4_OD": "Bolt M4 OD [m]",
    "bolt_M4_head_width": "Bolt M4 head width [m]",
    "bolt_M4_hole_OD": "Bolt M4 hole OD [m]",
    "bolt_M4_head_OD": "Bolt M4 head OD [m]",
    "threaded_insert_M5_fixing_hole_OD": "Threaded insert M5 fixing hole OD [m]",
    "threaded_insert_M5_lenght": "Threaded insert M5 length [m]",
    "bolt_M5_OD": "Bolt M5 OD [m]",
    "bolt_M5_head_width": "Bolt M5 head width [m]",
    "bolt_M5_hole_OD": "Bolt M5 hole OD [m]",
    "bolt_M5_head_OD": "Bolt M5 head OD [m]",
    "threaded_insert_M6_fixing_hole_OD": "Threaded insert M6 fixing hole OD [m]",
    "threaded_insert_M6_lenght": "Threaded insert M6 length [m]",
    "bolt_M6_OD": "Bolt M6 OD [m]",
    "bolt_M6_head_width": "Bolt M6 head width [m]",
    "bolt_M6_hole_OD": "Bolt M6 hole OD [m]",
    "bolt_M6_head_OD": "Bolt M6 head OD [m]",
    "nose_axial_restraint_boss_width": "Nose axial restraint boss width [m]",
    "nose_axial_restraint_bolt_lenght": "Nose axial restraint bolt length [m]",
    "prop_y_position_fraction": "Prop Y position fraction [-]",
    "prop_x_position_fraction": "Prop X position fraction [-]",
    "prop_fuselage_clearance_min_m": "Min prop-fuselage clearance [m]",
    "autopilot_class": "Autopilot class",
    "n_servos": "Number of servos",
    "esc_efficiency": "ESC efficiency [-]",
    "motor_mechanical_efficiency": "Motor mechanical efficiency [-]",
    "vtol_enabled": "VTOL enabled",
    "hover_accel_mps2": "Hover acceleration [m/s²]",
    "hover_thrust_margin": "Hover thrust margin [-]",
    "hover_eta": "Hover eta [-]",
    "hover_power_margin": "Hover power margin [-]",
    "vtol_takeoff_hover_time_s": "VTOL takeoff hover time [s]",
    "vtol_climb_height_m": "VTOL climb height [m]",
    "vtol_climb_accel_mps2": "VTOL climb acceleration [m/s²]",
    "vtol_climb_vmax_mps": "VTOL climb vmax [m/s]",
    "transition_stall_margin": "Transition stall margin [-]",
    "transition_min_speed_mps": "Transition min speed [m/s]",
    "transition_time_s": "Transition time [s]",
    "accel_to_cruise_time_s": "Accel-to-cruise time [s]",
    "battery_energy_reserve_factor": "Battery energy reserve factor [-]",
    "mission_profile_steps": "Mission profile steps",
    "assembly_fit_factor": "Assembly fit factor [-]",
    "battery_location_adjustment_length": "Battery location adjustment length [m]",
    "battery_location_adjustment_gap": "Battery location adjustment gap [m]",
    "battery_case_thickness": "Battery case thickness [m]",
    "use_hardcoded_values": "Use hardcoded values",
    "print_weight_reduction": "Print weight reduction factor [-]",
    "hardcode_wing_weight_kg": "Hardcode wing weight [kg]",
    "hardcode_tail_weight_kg": "Hardcode tail weight [kg]",
    "hardcode_vertical_tail_weight_kg": "Hardcode vertical tail weight [kg]",
    "hardcode_fuselage_A_weight_kg": "Hardcode fuselage A weight [kg]",
    "hardcode_fuselage_B_weight_kg": "Hardcode fuselage B weight [kg]",
    "hardcode_fuselage_C_weight_kg": "Hardcode fuselage C weight [kg]",
    "hardcode_fuselage_nose_weight_kg": "Hardcode fuselage nose weight [kg]",
    "hardcode_fuselage_tail_weight_kg": "Hardcode fuselage tail weight [kg]",
    "hardcode_propulsion_weight_kg": "Hardcode propulsion weight [kg]",
    "hardcode_avionics_weight_kg": "Hardcode avionics weight [kg]",
    "hardcode_tail_span": "Hardcode tail span [m]",
    "hardcode_vertical_tail_span": "Hardcode vertical tail span [m]",
    "hardcode_wing_spar_OD": "Hardcode wing spar OD [m]",
    "hardcode_wing_secondary_spar_OD": "Hardcode wing secondary spar OD [m]",
    "hardcode_tail_wing_spar_OD": "Hardcode tail wing spar OD [m]",
    "hardcode_elevator_rod_OD": "Hardcode elevator rod OD [m]",
    "battery_length": "Battery length [m]",
    "battery_width": "Battery width [m]",
    "battery_height": "Battery height [m]",
    "cg_x_wing": "CG x: wing [m]",
    "cg_x_tail": "CG x: tail [m]",
    "cg_x_vertical_tail": "CG x: vertical tail [m]",
    "cg_x_battery": "CG x: battery [m]",
    "cg_x_fuselage": "CG x: fuselage [m]",
    "cg_x_nose": "CG x: nose [m]",
    "cg_x_body_section_A": "CG x: body section A [m]",
    "cg_x_body_section_B": "CG x: body section B [m]",
    "cg_x_body_section_C": "CG x: body section C [m]",
    "cg_x_fuselage_tail": "CG x: fuselage tail [m]",
    "cg_x_propulsion": "CG x: propulsion [m]",
    "cg_x_autopilot": "CG x: autopilot [m]",
    "cg_x_power_module": "CG x: power module [m]",
    "cg_x_esc": "CG x: ESC [m]",
    "cg_x_gps": "CG x: GPS [m]",
    "cg_x_airspeed": "CG x: airspeed sensor [m]",
    "cg_x_rx": "CG x: RX [m]",
    "cg_x_tel": "CG x: telemetry [m]",
    "cg_x_servos": "CG x: servos [m]",
    "cg_x_misc": "CG x: misc [m]",
    "cg_x_payload": "CG x: payload [m]",
    "fuselage_OR": "Fuselage outer radius [m]",
    "fuselage_cs_area": "Fuselage cross-section area [m²]",
    "fuselage_cs_perimeter": "Fuselage cross-section perimeter [m]",
    "fuselage_shell_area_moment_of_inertia": "Fuselage shell area MOI [m³]",
    "fuselage_wetted_area": "Fuselage wetted area [m²]",
}

OBJECTIVE_KEY_TO_LABEL = {key: label for (label, key) in OBJECTIVE_OPTIONS}
OBJECTIVE_KEYS = [key for (_, key) in OBJECTIVE_OPTIONS]


# ============================================================
# Plot sizing + text sizing
# ============================================================
PLOT_FIGSIZE_1D = (4.5, 2.2)
PLOT_FIGSIZE_2D = (4.8, 2.6)
PLOT_FIGSIZE_HIST = (4.5, 2.0)
PLOT_DPI = 120

FONT_LABEL = 6
FONT_TICK = 6
FONT_CBAR = 6


# ============================================================
# Type helpers
# ============================================================
def _normalize_annot(t: Any) -> Any:
    """
    Reduce Optional[T]/Union[T,None] -> T, keep plain types as-is.
    """
    origin = get_origin(t)
    if origin is None:
        return t
    if origin is Optional:
        args = [a for a in get_args(t) if a is not type(None)]
        return args[0] if args else t
    if origin is list or origin is dict or origin is tuple:
        return t
    if origin is type(Optional[int]).__origin__ or origin is type(Optional[float]).__origin__:
        args = [a for a in get_args(t) if a is not type(None)]
        return args[0] if args else t
    return t


def wants_int(param_name: str) -> bool:
    ann = getattr(design_inputs.DesignInputs, "__annotations__", {}).get(param_name, None)
    ann = _normalize_annot(ann)
    return ann is int


def wants_float(param_name: str) -> bool:
    ann = getattr(design_inputs.DesignInputs, "__annotations__", {}).get(param_name, None)
    ann = _normalize_annot(ann)
    return ann is float


def is_numeric(v: Any) -> bool:
    return isinstance(v, (int, float, np.number)) and not isinstance(v, bool)


def _is_excel_scalar(v: Any) -> bool:
    return (
        v is None
        or isinstance(v, (int, float, bool, str, np.integer, np.floating, np.bool_))
    )


def _is_supported_param_field(name: str, value: Any, ann: Any) -> bool:
    """
    Include scalar / optional scalar / bool / str fields only.
    Exclude dict-like hardcoded battery/propulsion blobs because current UI
    is not intended to edit nested dicts.
    """
    ann_n = _normalize_annot(ann)

    if ann_n in (dict, list, tuple, Dict, List, Tuple):
        return False

    origin = get_origin(ann_n)
    if origin in (dict, list, tuple):
        return False

    if isinstance(value, (dict, list, tuple, set)):
        return False

    return isinstance(value, (bool, int, float, str, np.integer, np.floating, np.bool_)) or ann_n in (bool, int, float, str)


def _auto_label_from_name(name: str) -> str:
    txt = name.replace("_", " ").strip()
    if not txt:
        return name
    txt = txt[0].upper() + txt[1:]
    txt = txt.replace(" cg ", " CG ")
    txt = txt.replace(" od", " OD")
    txt = txt.replace(" id", " ID")
    txt = txt.replace(" nacA", " NACA")
    txt = txt.replace("Mpa", "MPa")
    txt = txt.replace(" pa", " Pa")
    return txt


def build_vary_param_options() -> List[Tuple[str, str]]:
    base = design_inputs.DesignInputs()
    annotations = getattr(design_inputs.DesignInputs, "__annotations__", {})
    out: List[Tuple[str, str]] = []

    for name in annotations.keys():
        if not hasattr(base, name):
            continue
        value = getattr(base, name)
        ann = annotations.get(name)
        if not _is_supported_param_field(name, value, ann):
            continue

        label = PRETTY_PARAM_LABELS.get(name, _auto_label_from_name(name))
        out.append((label, name))

    out = sorted(out, key=lambda x: x[0].lower())
    return out


VARY_PARAM_OPTIONS: List[Tuple[str, str]] = build_vary_param_options()
PARAM_LABEL_TO_NAME = {label: name for (label, name) in VARY_PARAM_OPTIONS}
PARAM_NAME_TO_LABEL = {name: label for (label, name) in VARY_PARAM_OPTIONS}
PARAM_LABELS = [label for (label, _) in VARY_PARAM_OPTIONS]


# ============================================================
# Utilities
# ============================================================
def flatten_results(res: Any) -> Dict[str, Any]:
    """
    Flatten DesignResults -> dict of mostly scalars.

    - Uses results.to_dict() as authoritative.
    - Promotes inputs.mission_profile.* to top-level keys
    """
    flat = res.to_dict() if hasattr(res, "to_dict") else dict(res)

    inputs_blob = flat.get("inputs", {})
    if isinstance(inputs_blob, dict):
        mp = inputs_blob.get("mission_profile", None)
        if isinstance(mp, dict):
            for k, v in mp.items():
                if _is_excel_scalar(v):
                    flat[f"mission_profile.{k}"] = v

        for k in ("tail_span_total_m", "tail_span_extension_m", "tail_area_total_m2"):
            if k in inputs_blob and _is_excel_scalar(inputs_blob.get(k)):
                flat[k] = inputs_blob.get(k)

    return flat


def metric_from_flat(flat: Dict[str, Any], key: str) -> float:
    if key in flat and flat[key] is not None:
        return float(flat[key])
    raise KeyError(f"Objective '{key}' not found in result keys for this run.")


def validate_simulation_postrun(inp: "design_inputs.DesignInputs", flat: Dict[str, Any]) -> None:
    issues: List[str] = []

    try:
        if float(getattr(inp, "fuselage_length")) <= 0:
            issues.append(f"fuselage_length must be > 0 (got {getattr(inp,'fuselage_length')})")
    except Exception:
        pass

    try:
        V_stall = float(flat.get("V_stall", float("nan")))
    except Exception:
        V_stall = float("nan")
    try:
        V_cruise = float(getattr(inp, "speed"))
    except Exception:
        V_cruise = float("nan")

    if np.isfinite(V_stall) and np.isfinite(V_cruise) and V_cruise > 0:
        if V_stall > 0.75 * V_cruise:
            issues.append(
                f"Stall speed ({V_stall:.2f} m/s) exceeds 0.75 * cruise speed ({0.75 * V_cruise:.2f} m/s)"
            )

    if issues:
        raise ValueError("Configuration invalid: " + "; ".join(issues))


def parse_values_list_raw(s: str) -> List[Any]:
    """
    Raw parsing: bool/int/float/string.
    """
    out: List[Any] = []
    for raw in s.split(","):
        raw = raw.strip()
        if not raw:
            continue
        low = raw.lower()
        if low in {"true", "false"}:
            out.append(low == "true")
            continue
        if raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit()):
            out.append(int(raw))
            continue
        try:
            out.append(float(raw))
            continue
        except ValueError:
            out.append(raw)
    return out


def coerce_values_to_param_type(param_name: str, vals: List[Any]) -> List[Any]:
    """
    Keep float parameters as float even if user types '5' (parsed as int).
    """
    if wants_int(param_name):
        out: List[Any] = []
        for v in vals:
            if isinstance(v, bool):
                out.append(bool(v))
            elif isinstance(v, (int, np.integer)):
                out.append(int(v))
            elif isinstance(v, (float, np.floating)):
                out.append(int(round(float(v))))
            else:
                out.append(v)
        return out

    out2: List[Any] = []
    for v in vals:
        if isinstance(v, bool):
            out2.append(bool(v))
        elif isinstance(v, (int, np.integer, float, np.floating)):
            out2.append(float(v))
        else:
            out2.append(v)
    return out2


def values_to_csv_string(vals: List[Any]) -> str:
    def fmt(v: Any) -> str:
        if isinstance(v, bool):
            return "true" if v else "false"
        if isinstance(v, (int, np.integer)):
            return str(int(v))
        if isinstance(v, (float, np.floating)):
            return f"{float(v):.8g}"
        return str(v)

    return ",".join(fmt(v) for v in vals)


def make_range_values(lo: float, hi: float, n: int, *, force_int: bool) -> List[Any]:
    if n <= 1:
        v = float(lo)
        return [int(round(v))] if force_int else [float(v)]
    vals = [float(x) for x in np.linspace(lo, hi, n)]
    if force_int:
        return [int(round(v)) for v in vals]
    return vals


def detect_linspace(vals: List[Any]) -> Optional[Tuple[float, float, int]]:
    try:
        arr = np.array([float(v) for v in vals], dtype=float)
    except Exception:
        return None
    if len(arr) < 2 or not np.all(np.isfinite(arr)):
        return None
    diffs = np.diff(arr)
    if np.allclose(diffs, diffs[0], rtol=1e-6, atol=1e-9):
        return float(arr.min()), float(arr.max()), int(len(arr))
    return None


def compute_sensitivity(df_ok: pd.DataFrame, params: List[str], objective_col: str = "objective") -> pd.DataFrame:
    if df_ok.empty:
        return pd.DataFrame(columns=["param", "pearson_r", "spearman_rho", "std_slope", "abs_std_slope", "n"])

    y = pd.to_numeric(df_ok[objective_col], errors="coerce").astype(float).values
    y_std = float(np.nanstd(y)) if float(np.nanstd(y)) > 0 else 1.0

    rows = []
    for p in params:
        if p not in df_ok.columns:
            continue
        x = pd.to_numeric(df_ok[p], errors="coerce").astype(float).values
        mask = np.isfinite(x) & np.isfinite(y)
        if int(mask.sum()) < 3:
            continue

        xx = x[mask]
        yy = y[mask]

        pearson = float(pd.Series(xx).corr(pd.Series(yy), method="pearson"))
        spearman = float(pd.Series(xx).corr(pd.Series(yy), method="spearman"))

        x_std = float(np.std(xx)) if float(np.std(xx)) > 0 else 1.0
        slope = float(np.polyfit(xx, yy, 1)[0]) if len(xx) >= 2 else 0.0
        std_slope = float(slope * (x_std / y_std))

        rows.append(
            dict(
                param=p,
                pearson_r=pearson,
                spearman_rho=spearman,
                std_slope=std_slope,
                abs_std_slope=abs(std_slope),
                n=int(mask.sum()),
            )
        )

    df = pd.DataFrame(rows)
    if not df.empty:
        df = df.sort_values("abs_std_slope", ascending=False).reset_index(drop=True)
        df["param_label"] = df["param"].map(lambda n: PARAM_NAME_TO_LABEL.get(n, n))
        df = df[["param_label", "param", "pearson_r", "spearman_rho", "std_slope", "abs_std_slope", "n"]]
    return df


def _default_visible_success_columns(df_ok: Optional[pd.DataFrame] = None, params_used: Optional[List[str]] = None) -> List[str]:
    """
    Default columns shown in the Excel success sheet.
    If df_ok is provided, keep only columns that exist in the result.
    """
    preferred: List[str] = []

    # Always start with the project default export columns.
    preferred.extend(DEFAULT_EXCEL_VISIBLE_SUCCESS_COLUMNS)

    # Also include currently varied parameters, so sweep inputs remain visible by default.
    if params_used:
        preferred.extend(params_used)

    # Preserve order and remove duplicates.
    out: List[str] = []
    for c in preferred:
        if c and c not in out:
            out.append(c)

    if df_ok is not None and not df_ok.empty:
        out_existing = [c for c in out if c in df_ok.columns]
        return out_existing if out_existing else list(df_ok.columns)

    return out


def _excel_column_options() -> List[str]:
    """Known/selectable columns before a run exists. Unknown result columns can still be typed in the custom box."""
    options: List[str] = []

    # Input parameters from DesignInputs.
    options.extend([name for (_, name) in VARY_PARAM_OPTIONS])

    # Common direct result columns.
    options.extend([
        "status",
        "objective",
        "objective_name",
        "W0_mass_kg",
        "drag_cruise_N",
        "cruise_power_elec_W",
        "L_over_D_cruise",
        "V_stall",
        "mission_profile.cruise_time_max_min",
        "mission_profile.total_energy_Wh",
        "mission_profile.peak_elec_power_W",
    ])

    # Requested/default res.* columns and common res.* variants of known objective columns.
    options.extend(DEFAULT_EXCEL_VISIBLE_SUCCESS_COLUMNS)
    options.extend([f"res.{k}" for k in OBJECTIVE_KEYS])

    out: List[str] = []
    for c in options:
        if c and c not in out:
            out.append(c)
    return out


def _clean_column_list(cols: Optional[List[Any]]) -> List[str]:
    out: List[str] = []
    if not isinstance(cols, list):
        return out
    for c in cols:
        c = str(c).strip()
        if c and c not in out:
            out.append(c)
    return out


def _parse_custom_column_text(text: str) -> List[str]:
    if not text:
        return []
    normalized = text.replace("\t", ",").replace("\n", ",")
    return _clean_column_list([part.strip() for part in normalized.split(",")])

def _format_col_for_ui(col: str) -> str:
    """Readable labels for the Excel column selector."""
    if col in PARAM_NAME_TO_LABEL:
        return f"{PARAM_NAME_TO_LABEL[col]}  ({col})"
    if col in OBJECTIVE_KEY_TO_LABEL:
        return f"{OBJECTIVE_KEY_TO_LABEL[col]}  ({col})"
    return col


def _hide_unselected_columns_in_sheet(writer: Any, sheet_name: str, df: pd.DataFrame, visible_columns: Optional[List[str]]) -> None:
    """Hide every Excel column not selected by the user, while keeping the data in the file."""
    if visible_columns is None or df is None or df.empty:
        return

    from openpyxl.utils import get_column_letter

    visible_set = set(visible_columns)
    ws = writer.book[sheet_name]

    for idx, col_name in enumerate(df.columns, start=1):
        if col_name not in visible_set:
            ws.column_dimensions[get_column_letter(idx)].hidden = True

    ws.freeze_panes = "A2"
    ws.auto_filter.ref = ws.dimensions


def dataframe_to_excel_bytes(
    df_ok: pd.DataFrame,
    df_bad: pd.DataFrame,
    df_sens: pd.DataFrame,
    config: Dict[str, Any],
    visible_success_columns: Optional[List[str]] = None,
) -> bytes:
    bio = BytesIO()
    with pd.ExcelWriter(bio, engine="openpyxl") as writer:
        df_ok.to_excel(writer, sheet_name="success", index=False)
        df_bad.to_excel(writer, sheet_name="failed", index=False)
        df_sens.to_excel(writer, sheet_name="sensitivity", index=False)
        pd.DataFrame([{"config_json": json.dumps(config, indent=2)}]).to_excel(writer, sheet_name="config", index=False)

        _hide_unselected_columns_in_sheet(writer, "success", df_ok, visible_success_columns)

    return bio.getvalue()


def save_to_xlsx(
    df_ok: pd.DataFrame,
    df_bad: pd.DataFrame,
    df_sens: pd.DataFrame,
    config: Dict[str, Any],
    folder: str,
    exp_name: str,
    visible_success_columns: Optional[List[str]] = None,
) -> str:
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"{exp_name}.xlsx")
    with pd.ExcelWriter(path, engine="openpyxl") as writer:
        df_ok.to_excel(writer, sheet_name="success", index=False)
        df_bad.to_excel(writer, sheet_name="failed", index=False)
        df_sens.to_excel(writer, sheet_name="sensitivity", index=False)
        pd.DataFrame([{"config_json": json.dumps(config, indent=2)}]).to_excel(writer, sheet_name="config", index=False)

        _hide_unselected_columns_in_sheet(writer, "success", df_ok, visible_success_columns)

    return path


# ============================================================
# JSON CONFIG IO (LOAD + SAVE)
# ============================================================
def build_config(
    objective_key: str,
    direction: str,
    param_grid: Dict[str, List[Any]],
    excel_visible_success_columns: Optional[List[str]] = None,
) -> Dict[str, Any]:
    return {
        "objective": objective_key,
        "direction": direction,
        "param_grid": param_grid,
        "excel_visible_success_columns": _clean_column_list(
            excel_visible_success_columns if excel_visible_success_columns is not None else DEFAULT_EXCEL_VISIBLE_SUCCESS_COLUMNS
        ),
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "schema_version": 4,
    }


def load_config_json_bytes(file_bytes: bytes) -> Dict[str, Any]:
    txt = file_bytes.decode("utf-8-sig")
    cfg = json.loads(txt)
    if not isinstance(cfg, dict):
        raise ValueError("Config JSON must be an object/dict.")
    if "objective" not in cfg or "direction" not in cfg or "param_grid" not in cfg:
        raise ValueError("Missing required keys: objective, direction, param_grid.")
    if not isinstance(cfg["param_grid"], dict):
        raise ValueError("'param_grid' must be a dict.")
    return cfg


def config_to_json_bytes(cfg: Dict[str, Any]) -> bytes:
    txt = json.dumps(cfg, indent=2, sort_keys=True, ensure_ascii=False)
    return txt.encode("utf-8")


def save_config_to_disk(cfg: Dict[str, Any], folder: str, filename_no_ext: str) -> str:
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"{filename_no_ext}.json")
    with open(path, "wb") as f:
        f.write(config_to_json_bytes(cfg))
    return path


def safe_default_objective_key(loaded: Optional[str]) -> str:
    if OBJECTIVE_KEYS:
        if loaded in OBJECTIVE_KEYS:
            return loaded
        if DEFAULT_OBJECTIVE_KEY in OBJECTIVE_KEYS:
            return DEFAULT_OBJECTIVE_KEY
        return OBJECTIVE_KEYS[0]
    return DEFAULT_OBJECTIVE_KEY


def prime_sidebar_from_loaded_config(cfg: Dict[str, Any]) -> None:
    obj = safe_default_objective_key(cfg.get("objective", None))
    dirn = cfg.get("direction", "min")
    dirn = dirn if dirn in ("min", "max") else "min"

    st.session_state["objective_key_widget"] = obj
    st.session_state["direction_widget"] = dirn

    loaded_excel_cols = _clean_column_list(cfg.get("excel_visible_success_columns", [])) or list(DEFAULT_EXCEL_VISIBLE_SUCCESS_COLUMNS)
    excel_options = _excel_column_options()
    st.session_state["excel_visible_success_columns"] = loaded_excel_cols
    st.session_state["excel_visible_success_columns_known_widget"] = [c for c in loaded_excel_cols if c in excel_options]
    st.session_state["excel_visible_success_columns_custom_widget"] = "\n".join([c for c in loaded_excel_cols if c not in excel_options])

    pg = cfg.get("param_grid", {})
    if not isinstance(pg, dict):
        return

    names = [p for p in pg.keys() if p in PARAM_NAME_TO_LABEL]
    st.session_state["vary_params_widget"] = [PARAM_NAME_TO_LABEL[p] for p in names]

    base = design_inputs.DesignInputs()

    for p, vals in pg.items():
        if p not in PARAM_NAME_TO_LABEL or not isinstance(vals, list) or len(vals) == 0:
            continue

        base_val = getattr(base, p, None)
        vals_typed = coerce_values_to_param_type(p, vals)

        lin = detect_linspace(vals_typed) if is_numeric(base_val) else None

        if isinstance(base_val, bool):
            st.session_state[f"{p}_bool"] = [bool(v) for v in vals_typed if isinstance(v, bool)]
        elif wants_int(p):
            if lin is not None:
                lo, hi, n = lin
                st.session_state[f"{p}_mode_int"] = "Range"
                st.session_state[f"{p}_lo_int"] = int(round(lo))
                st.session_state[f"{p}_hi_int"] = int(round(hi))
                st.session_state[f"{p}_n_int"] = int(n)
            else:
                st.session_state[f"{p}_mode_int"] = "List"
                st.session_state[f"{p}_list_int"] = values_to_csv_string(vals_typed)
        else:
            if lin is not None:
                lo, hi, n = lin
                st.session_state[f"{p}_mode_float"] = "Range"
                st.session_state[f"{p}_lo_float"] = float(lo)
                st.session_state[f"{p}_hi_float"] = float(hi)
                st.session_state[f"{p}_n_float"] = int(n)
            else:
                st.session_state[f"{p}_mode_float"] = "List"
                st.session_state[f"{p}_list_float"] = values_to_csv_string(vals_typed)


# ============================================================
# DoE runner
# ============================================================

PROJECT_MODULES_TO_RELOAD: List[str] = [
    "constants",
    "design_inputs",
    "geometry",
    "aerodynamics",
    "structures",
    "avionics",
    "propulsion",
    "batteries",
    "propulsion_system",
    "convergence",
    "results",
    "reporting",
    "script",
]


def reload_project_modules(module_names: Optional[List[str]] = None) -> None:
    importlib.invalidate_caches()
    names = module_names or PROJECT_MODULES_TO_RELOAD
    for name in names:
        try:
            if name in sys.modules:
                importlib.reload(sys.modules[name])
            else:
                importlib.import_module(name)
        except Exception:
            continue


def run_doe(
    param_grid: Dict[str, List[Any]],
    objective_key: str,
    direction: str,
    *,
    reload_modules: bool = True,
    store_tracebacks: bool = True,
    export_input_columns: Optional[List[str]] = None,
) -> Tuple[pd.DataFrame, pd.DataFrame, Dict[str, Any]]:
    if reload_modules:
        reload_project_modules()
        globals()["design_inputs"] = importlib.import_module("design_inputs")
        globals()["script"] = importlib.import_module("script")

    params = list(param_grid.keys())
    combos = list(itertools.product(*[param_grid[p] for p in params])) if params else [()]

    successes: List[Dict[str, Any]] = []
    failures: List[Dict[str, Any]] = []

    total = len(combos)
    prog = st.progress(0.0)
    status = st.empty()

    for i, combo in enumerate(combos, start=1):
        status.write(f"Running {i}/{total} ...")
        overrides = {p: v for p, v in zip(params, combo)}

        try:
            inp = design_inputs.DesignInputs(**overrides)
            res = script.run_design(inp)
            flat = flatten_results(res)

            validate_simulation_postrun(inp, flat)
            obj = metric_from_flat(flat, objective_key)

            if not np.isfinite(float(obj)):
                raise ValueError(f"Objective '{objective_key}' is non-finite: {obj}")

            row: Dict[str, Any] = {}

            # Include selected input columns in the output even when they were not varied.
            # This makes config-selected fields such as cg_x_wing or payload_mass available
            # in the Excel export instead of only hiding/showing columns that already exist.
            input_cols_to_export = _clean_column_list((export_input_columns or []) + params)
            for p in input_cols_to_export:
                if hasattr(inp, p):
                    row[p] = getattr(inp, p, overrides.get(p, None))

            row["status"] = "OK"
            row["objective"] = float(obj)
            row["objective_name"] = objective_key

            for k in [
                "W0_mass_kg",
                "drag_cruise_N",
                "cruise_power_elec_W",
                "L_over_D_cruise",
                "V_stall",
                "mission_profile.cruise_time_max_min",
                "mission_profile.total_energy_Wh",
                "mission_profile.peak_elec_power_W",
            ]:
                if k in flat and flat[k] is not None:
                    row[k] = flat[k]

            for k, v in flat.items():
                if not _is_excel_scalar(v):
                    continue
                col = f"res.{k}"
                if col not in row:
                    row[col] = v

            successes.append(row)

        except Exception as e:
            fail_row: Dict[str, Any] = dict(overrides)
            fail_row["status"] = "FAILED"
            fail_row["error"] = f"{type(e).__name__}: {e}"
            if store_tracebacks:
                tb = traceback.format_exc()
                fail_row["traceback"] = tb[:32000]
            failures.append(fail_row)

        prog.progress(i / total)

    status.empty()

    df_ok = pd.DataFrame(successes)
    df_bad = pd.DataFrame(failures)

    if not df_ok.empty:
        ascending = (direction.lower() == "min")
        df_ok = df_ok.sort_values("objective", ascending=ascending).reset_index(drop=True)

    meta = dict(n_total=int(total), n_success=int(len(df_ok)), n_failed=int(len(df_bad)))
    return df_ok, df_bad, meta


# ============================================================
# Plot helpers
# ============================================================
def plot_objective_1d(df: pd.DataFrame, x: str) -> None:
    d = df.sort_values(x)
    fig = plt.figure(figsize=PLOT_FIGSIZE_1D, dpi=PLOT_DPI)
    ax = fig.add_subplot(111)
    ax.plot(d[x], d["objective"], marker="o", markersize=4)
    ax.set_xlabel(PARAM_NAME_TO_LABEL.get(x, x), fontsize=FONT_LABEL)
    ax.set_ylabel(OBJECTIVE_KEY_TO_LABEL.get(d["objective_name"].iloc[0], d["objective_name"].iloc[0]), fontsize=FONT_LABEL)
    ax.tick_params(axis="both", labelsize=FONT_TICK)
    ax.grid(True)
    fig.tight_layout()
    st.pyplot(fig, use_container_width=False)
    plt.close(fig)


def plot_objective_2d(df: pd.DataFrame, x: str, y: str) -> None:
    fig = plt.figure(figsize=PLOT_FIGSIZE_2D, dpi=PLOT_DPI)
    ax = fig.add_subplot(111)
    sc = ax.scatter(df[x], df[y], c=df["objective"], s=40)
    ax.set_xlabel(PARAM_NAME_TO_LABEL.get(x, x), fontsize=FONT_LABEL)
    ax.set_ylabel(PARAM_NAME_TO_LABEL.get(y, y), fontsize=FONT_LABEL)
    ax.tick_params(axis="both", labelsize=FONT_TICK)
    ax.grid(True)
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label(OBJECTIVE_KEY_TO_LABEL.get(df["objective_name"].iloc[0], df["objective_name"].iloc[0]), fontsize=FONT_CBAR)
    cbar.ax.tick_params(labelsize=FONT_TICK)
    fig.tight_layout()
    st.pyplot(fig, use_container_width=False)
    plt.close(fig)


def plot_objective_hist(df: pd.DataFrame) -> None:
    fig = plt.figure(figsize=PLOT_FIGSIZE_HIST, dpi=PLOT_DPI)
    ax = fig.add_subplot(111)
    ax.hist(df["objective"].values, bins=min(30, max(5, len(df) // 4)))
    ax.set_xlabel(OBJECTIVE_KEY_TO_LABEL.get(df["objective_name"].iloc[0], df["objective_name"].iloc[0]), fontsize=FONT_LABEL)
    ax.set_ylabel("count", fontsize=FONT_LABEL)
    ax.tick_params(axis="both", labelsize=FONT_TICK)
    ax.grid(True)
    fig.tight_layout()
    st.pyplot(fig, use_container_width=False)
    plt.close(fig)


# ============================================================
# Streamlit app
# ============================================================
st.set_page_config(page_title="Drone DoE", layout="wide")
st.title("Drone Design of Experiments (DoE)")

if "doe_results" not in st.session_state:
    st.session_state.doe_results = None
if "plot_x" not in st.session_state:
    st.session_state.plot_x = None
if "plot_y" not in st.session_state:
    st.session_state.plot_y = None
if "last_loaded_cfg_name" not in st.session_state:
    st.session_state.last_loaded_cfg_name = None
if "last_loaded_cfg_size" not in st.session_state:
    st.session_state.last_loaded_cfg_size = None
if "last_saved_config_path" not in st.session_state:
    st.session_state.last_saved_config_path = None
if "excel_visible_success_columns" not in st.session_state:
    st.session_state.excel_visible_success_columns = list(DEFAULT_EXCEL_VISIBLE_SUCCESS_COLUMNS)


with st.sidebar:
    st.header("Load config (JSON)")

    if st.button("Reset app state"):
        st.session_state.clear()
        st.rerun()

    uploaded_cfg = st.file_uploader("Upload config (.json)", type=["json"], key="cfg_uploader")

    if uploaded_cfg is not None:
        cfg_bytes = uploaded_cfg.getvalue()
        cfg_name = uploaded_cfg.name
        cfg_size = len(cfg_bytes) if cfg_bytes is not None else 0

        already_loaded = (
            st.session_state.last_loaded_cfg_name == cfg_name
            and st.session_state.last_loaded_cfg_size == cfg_size
        )

        if (not already_loaded) and cfg_bytes and cfg_size > 0:
            try:
                cfg = load_config_json_bytes(cfg_bytes)
                prime_sidebar_from_loaded_config(cfg)

                st.session_state.last_loaded_cfg_name = cfg_name
                st.session_state.last_loaded_cfg_size = cfg_size

                st.success(f"Loaded config: {cfg_name}")
                st.rerun()
            except Exception as e:
                st.error(f"Failed to load config: {type(e).__name__}: {e}")
        elif already_loaded:
            st.caption(f"Config loaded: {cfg_name}")
        elif cfg_size == 0:
            st.warning("Uploaded file appears empty (0 bytes). Re-upload it.")

    st.divider()
    st.header("DoE setup")

    if not OBJECTIVE_KEYS:
        st.error("OBJECTIVE_OPTIONS is empty. Add at least one objective.")
        st.stop()

    loaded_obj = st.session_state.get("objective_key_widget", None)
    obj_key = safe_default_objective_key(loaded_obj)
    obj_idx = OBJECTIVE_KEYS.index(obj_key) if obj_key in OBJECTIVE_KEYS else 0

    objective_key = st.selectbox(
        "Objective metric",
        options=OBJECTIVE_KEYS,
        index=obj_idx,
        format_func=lambda k: OBJECTIVE_KEY_TO_LABEL.get(k, k),
        key="objective_key_widget",
    )

    direction = st.radio(
        "Direction",
        ["min", "max"],
        horizontal=True,
        index=(0 if st.session_state.get("direction_widget", "min") == "min" else 1),
        key="direction_widget",
    )

    st.divider()

    default_labels = [PARAM_NAME_TO_LABEL[p] for p in DEFAULT_VARY_PARAMS if p in PARAM_NAME_TO_LABEL]
    selected_param_labels = st.multiselect(
        "Parameters to vary",
        options=PARAM_LABELS,
        default=st.session_state.get("vary_params_widget", default_labels),
        key="vary_params_widget",
    )

    param_grid: Dict[str, List[Any]] = {}
    base = design_inputs.DesignInputs()

    for lbl in selected_param_labels:
        p = PARAM_LABEL_TO_NAME[lbl]
        base_val = getattr(base, p)

        with st.expander(f"{lbl}  ({p})   base={base_val}", expanded=True):
            if isinstance(base_val, bool):
                default_bools = st.session_state.get(f"{p}_bool", [base_val])
                vals = st.multiselect(f"Values for {p}", options=[True, False], default=default_bools, key=f"{p}_bool")
                param_grid[p] = vals if vals else [base_val]
                continue

            if is_numeric(base_val) or wants_int(p) or wants_float(p):
                is_int_param = wants_int(p)

                if is_int_param:
                    mode = st.radio("Mode", ["Range", "List"], horizontal=True, key=f"{p}_mode_int")
                    if mode == "Range":
                        lo0 = int(round(float(base_val) * 0.9))
                        hi0 = int(round(float(base_val) * 1.1))
                        if hi0 == lo0:
                            hi0 = lo0 + 1
                        lo = st.number_input("min", value=int(st.session_state.get(f"{p}_lo_int", lo0)), step=1, key=f"{p}_lo_int")
                        hi = st.number_input("max", value=int(st.session_state.get(f"{p}_hi_int", hi0)), step=1, key=f"{p}_hi_int")
                        n = st.number_input("points", min_value=1, max_value=200, value=int(st.session_state.get(f"{p}_n_int", 5)), step=1, key=f"{p}_n_int")
                        vals = make_range_values(float(lo), float(hi), int(n), force_int=True)
                        param_grid[p] = vals
                    else:
                        s0 = str(st.session_state.get(f"{p}_list_int", str(int(round(float(base_val))))))
                        s = st.text_input("Values (comma-separated)", value=s0, key=f"{p}_list_int")
                        raw_vals = parse_values_list_raw(s)
                        typed_vals = coerce_values_to_param_type(p, raw_vals)
                        param_grid[p] = typed_vals if typed_vals else [int(round(float(base_val)))]
                    continue

                mode = st.radio("Mode", ["Range", "List"], horizontal=True, key=f"{p}_mode_float")
                if mode == "Range":
                    lo0 = float(base_val) * 0.9
                    hi0 = float(base_val) * 1.1
                    lo = st.number_input("min", value=float(st.session_state.get(f"{p}_lo_float", lo0)), key=f"{p}_lo_float")
                    hi = st.number_input("max", value=float(st.session_state.get(f"{p}_hi_float", hi0)), key=f"{p}_hi_float")
                    n = st.number_input("points", min_value=1, max_value=200, value=int(st.session_state.get(f"{p}_n_float", 5)), step=1, key=f"{p}_n_float")
                    vals = make_range_values(float(lo), float(hi), int(n), force_int=False)
                    param_grid[p] = coerce_values_to_param_type(p, vals)
                else:
                    s0 = str(st.session_state.get(f"{p}_list_float", str(float(base_val))))
                    s = st.text_input("Values (comma-separated)", value=s0, key=f"{p}_list_float")
                    raw_vals = parse_values_list_raw(s)
                    typed_vals = coerce_values_to_param_type(p, raw_vals)
                    param_grid[p] = typed_vals if typed_vals else [float(base_val)]
                continue

            s0 = str(st.session_state.get(f"{p}_cat", str(base_val)))
            s = st.text_input("Values (comma-separated)", value=s0, key=f"{p}_cat")
            raw_vals = parse_values_list_raw(s)
            param_grid[p] = raw_vals if raw_vals else [base_val]

    n_runs = 1
    for v in param_grid.values():
        n_runs *= max(1, len(v))
    st.info(f"Total runs: {n_runs}")
    if n_runs > 800:
        st.warning("Large sweep. Reduce points/params for faster iteration.")

    st.divider()

    st.header("Excel export columns")
    st.caption("These columns will be visible in the Excel 'success' sheet. Other columns are exported but hidden.")

    excel_options = _excel_column_options()
    current_excel_cols = _clean_column_list(st.session_state.get("excel_visible_success_columns", [])) or list(DEFAULT_EXCEL_VISIBLE_SUCCESS_COLUMNS)
    known_excel_cols = [c for c in current_excel_cols if c in excel_options]
    unknown_excel_cols = [c for c in current_excel_cols if c not in excel_options]

    selected_known_excel_cols = st.multiselect(
        "Visible success-sheet columns",
        options=excel_options,
        default=known_excel_cols,
        format_func=_format_col_for_ui,
        key="excel_visible_success_columns_known_widget",
        help="Choose the columns that should remain visible in the exported Excel file.",
    )

    custom_excel_cols_text = st.text_area(
        "Additional/custom result columns",
        value="\n".join(unknown_excel_cols),
        key="excel_visible_success_columns_custom_widget",
        help="Use this for result columns not shown in the selector. Separate by comma, tab, or new line.",
        height=80,
    )

    excel_visible_success_columns = _clean_column_list(selected_known_excel_cols + _parse_custom_column_text(custom_excel_cols_text))
    if not excel_visible_success_columns:
        excel_visible_success_columns = list(DEFAULT_EXCEL_VISIBLE_SUCCESS_COLUMNS)
    st.session_state.excel_visible_success_columns = excel_visible_success_columns

    st.divider()

    st.header("Config save / export")

    cfg_default = datetime.now().strftime("cfg_%Y%m%d_%H%M%S")
    cfg_name = st.text_input("Config name (no extension)", value=cfg_default, key="cfg_name")

    live_cfg_for_download = build_config(objective_key, direction, param_grid, excel_visible_success_columns)
    st.download_button(
        "Download current config as JSON",
        data=config_to_json_bytes(live_cfg_for_download),
        file_name=f"{cfg_name}.json",
        mime="application/json",
        help="Saves objective + direction + param grid + Excel-visible columns. You can upload it back later.",
    )

    save_config_enabled = st.checkbox("Also save config to disk (.json)", value=False, key="save_config_enabled")
    config_folder = st.text_input("Config folder", value="doe_configs", key="config_folder")

    st.divider()

    save_results_enabled = st.checkbox("Save results to disk (.xlsx)", value=False, key="save_results_enabled")
    results_folder = st.text_input("Results folder", value="doe_out", key="results_folder")
    exp_default = datetime.now().strftime("exp_%Y%m%d_%H%M%S")
    exp_name = st.text_input("Experiment name", value=exp_default, key="exp_name")

    st.divider()
    reload_modules = st.checkbox(
        "Reload sizing code before run",
        value=True,
        help="Reloads project modules once before the sweep (behaves like a fresh `python script.py`).",
        key="reload_modules",
    )
    store_tracebacks = st.checkbox(
        "Store tracebacks for failed runs",
        value=True,
        help="Adds a 'traceback' column to the failed-runs table.",
        key="store_tracebacks",
    )

    colA, colB = st.columns(2)
    with colA:
        apply_button = st.button("Apply settings (no run)")
    with colB:
        run_button = st.button("Run DoE", type="primary")


current_config_sig = json.dumps({"objective": objective_key, "direction": direction, "param_grid": param_grid, "excel_visible_success_columns": excel_visible_success_columns}, sort_keys=True)

if apply_button:
    st.toast("Settings applied.", icon="✅")

if run_button:
    config = build_config(objective_key, direction, param_grid, excel_visible_success_columns)

    saved_cfg_path = None
    if bool(st.session_state.get("save_config_enabled", False)):
        try:
            saved_cfg_path = save_config_to_disk(
                config,
                folder=str(st.session_state.get("config_folder", "doe_configs")),
                filename_no_ext=str(st.session_state.get("cfg_name", "doe_config")),
            )
            st.session_state.last_saved_config_path = saved_cfg_path
        except Exception as e:
            st.warning(f"Could not save config to disk: {type(e).__name__}: {e}")

    with st.spinner("Running DoE..."):
        if reload_modules:
            reload_project_modules()
            globals()["design_inputs"] = importlib.import_module("design_inputs")
            globals()["script"] = importlib.import_module("script")

        df_ok, df_bad, meta = run_doe(
            param_grid,
            objective_key,
            direction,
            reload_modules=False,
            store_tracebacks=bool(st.session_state.get("store_tracebacks", True)),
            export_input_columns=excel_visible_success_columns,
        )
        df_sens = compute_sensitivity(df_ok, list(param_grid.keys()))

    save_path = None
    if save_results_enabled:
        selected_visible_cols = _clean_column_list(config.get("excel_visible_success_columns", []))
        if not selected_visible_cols:
            selected_visible_cols = _default_visible_success_columns(df_ok, list(param_grid.keys()))
        save_path = save_to_xlsx(
            df_ok,
            df_bad,
            df_sens,
            config,
            results_folder,
            exp_name,
            visible_success_columns=selected_visible_cols,
        )

    params_used = list(param_grid.keys())
    st.session_state.plot_x = params_used[0] if len(params_used) >= 1 else None
    st.session_state.plot_y = params_used[1] if len(params_used) >= 2 else None

    st.session_state.doe_results = {
        "df_ok": df_ok,
        "df_bad": df_bad,
        "df_sens": df_sens,
        "meta": meta,
        "config": config,
        "config_sig": current_config_sig,
        "exp_name": exp_name,
        "save_path": save_path,
        "params_used": params_used,
        "saved_cfg_path": saved_cfg_path,
    }


# ============================================================
# Main display
# ============================================================
respack = st.session_state.doe_results

if respack is None:
    st.info("Load a JSON config (sidebar) or set parameters and click **Run DoE**.")
else:
    df_ok = respack.get("df_ok", pd.DataFrame())
    df_bad = respack.get("df_bad", pd.DataFrame())
    df_sens = respack.get("df_sens", pd.DataFrame())
    meta = respack.get("meta", {"n_total": 0, "n_success": 0, "n_failed": 0})
    config = respack.get("config", {"objective": DEFAULT_OBJECTIVE_KEY, "direction": "min", "param_grid": {}})
    exp_name_last = respack.get("exp_name", "experiment")

    if respack.get("config_sig") is not None and respack.get("config_sig") != current_config_sig:
        st.warning("Settings changed since the displayed experiment. Click **Run DoE** to generate new results.")

    st.subheader("Run summary")
    c1, c2, c3 = st.columns(3)
    c1.metric("Total runs", meta.get("n_total", 0))
    c2.metric("Success", meta.get("n_success", 0))
    c3.metric("Failed", meta.get("n_failed", 0))

    if respack.get("saved_cfg_path"):
        st.info(f"Config saved to: {respack['saved_cfg_path']}")
    elif st.session_state.get("last_saved_config_path"):
        st.caption(f"Last saved config: {st.session_state.last_saved_config_path}")

    if df_ok.empty and not df_bad.empty:
        st.error("No successful runs. See failures below.")
        st.dataframe(df_bad, use_container_width=True)
        st.stop()

    if df_ok.empty:
        st.info("No results to show yet.")
        st.stop()

    params_used = [p for p in respack.get("params_used", []) if p in df_ok.columns]

    best = df_ok.iloc[0]
    obj_key = config.get("objective", DEFAULT_OBJECTIVE_KEY)
    obj_label = OBJECTIVE_KEY_TO_LABEL.get(obj_key, obj_key)
    st.success(f"Best objective ({obj_label}, {config.get('direction','min')}): {float(best['objective']):.4g}")

    st.write("**Best configuration (varied params):**")
    best_cfg = pd.DataFrame(
        [{"param_label": PARAM_NAME_TO_LABEL.get(p, p), "param": p, "value": best.get(p, None)} for p in params_used]
    )
    st.dataframe(best_cfg, use_container_width=True)

    st.write("**Top 20 results:**")
    st.dataframe(df_ok.head(20), use_container_width=True)

    tab1, tab2, tab3, tab4 = st.tabs(["Plots", "Sensitivity", "All results", "Failures"])

    with tab1:
        st.write("**Objective plots**")
        if len(params_used) == 0:
            st.info("No varied parameters in this experiment.")
            plot_objective_hist(df_ok)
        elif len(params_used) == 1:
            plot_objective_1d(df_ok, params_used[0])
            plot_objective_hist(df_ok)
        else:
            if st.session_state.plot_x not in params_used:
                st.session_state.plot_x = params_used[0]

            x = st.selectbox("X axis", params_used, key="plot_x", format_func=lambda n: PARAM_NAME_TO_LABEL.get(n, n))
            y_options = [p for p in params_used if p != x]
            if not y_options:
                plot_objective_hist(df_ok)
            else:
                if st.session_state.plot_y not in y_options:
                    st.session_state.plot_y = y_options[0]

                y = st.selectbox("Y axis", y_options, key="plot_y", format_func=lambda n: PARAM_NAME_TO_LABEL.get(n, n))
                try:
                    pd.to_numeric(df_ok[x])
                    pd.to_numeric(df_ok[y])
                    plot_objective_2d(df_ok, x, y)
                except Exception:
                    st.warning("Selected X/Y must be numeric for 2D scatter. Pick numeric params.")
                plot_objective_hist(df_ok)

    with tab2:
        st.write("**Sensitivity (numeric params only)**")
        if df_sens is None or df_sens.empty:
            df_sens_local = compute_sensitivity(df_ok, params_used)
            if df_sens_local.empty:
                st.info("No numeric sensitivity could be computed.")
            else:
                st.dataframe(df_sens_local, use_container_width=True)
        else:
            st.dataframe(df_sens, use_container_width=True)

    with tab3:
        st.write("**All successful runs:**")
        st.dataframe(df_ok, use_container_width=True)

    with tab4:
        st.write("**Failed runs:**")
        st.dataframe(df_bad, use_container_width=True)

    visible_success_columns = _clean_column_list(config.get("excel_visible_success_columns", []))
    if not visible_success_columns:
        visible_success_columns = _default_visible_success_columns(df_ok, params_used)

    with st.expander("Excel export columns used for this run", expanded=False):
        st.write("These columns will be visible in the Excel 'success' sheet; the remaining success columns are hidden, not deleted.")
        st.code("\n".join(visible_success_columns), language="text")

    export_bytes = dataframe_to_excel_bytes(
        df_ok,
        df_bad,
        df_sens if df_sens is not None else pd.DataFrame(),
        config,
        visible_success_columns=visible_success_columns,
    )
    st.download_button(
        "Download displayed results as Excel",
        data=export_bytes,
        file_name=f"{exp_name_last}.xlsx",
        mime="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
    )

    if respack.get("save_path"):
        st.info(f"Saved to: {respack['save_path']}")
