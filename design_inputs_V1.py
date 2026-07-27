import math
from dataclasses import dataclass, field
from typing import Optional, Dict, Any

@dataclass
class DesignInputs:
    # --MAIN DRIVING VARIABLES--
    payload_mass: float = 1.4      # [kg]
    payload_length: float = 0.05653 # [m] length of payload
    wing_loading: float = 240.0     # [N/m²] design W/S (MAIN WING ONLY)
    speed: float = 28.0              # [m/s] cruise speed
    dive_speed: float = 50.0         # [m/s] (derived: 1.5*speed)
    fuselage_length: float = 0.7     # [m] length of fuselage
    fuselage_OD: float = 0.14         # [m] outer diameter of fuselage
    fuselage_nose_length: float = 0.1         # [m] length of fuselage nose
    fuselage_tail_length: float = 0.16         # [m] length of fuselage tail (KEEP AS INPUT)
    fuselage_nonopt_factor: float = 1.2       # [-] non-optimum factor to approximate total fuselage (structure + fittings)
    fuselage_body_length: float = fuselage_length - fuselage_nose_length - fuselage_tail_length         # [m] length of fuselage body
    tail_c_root: float = 0.096            # [m] tail root chord (design input)
    tail_volume_ratio: float = 0.30      # [-]
    vertical_tail_volume_ratio: float = 0.05   # [-] Vv = (Sv * l_vt) / (S * b)
    c_root: float = 0.2
    # --END MAIN DRIVING VARIABLES--
    
    transition_fit_clearance: float = 0.00024 # [m] clearance for transition fit (0.25 mm)

    payload_fraction_hist: float = 0.33  # W_payload / W0

    export_inventor_xml: bool = False
    inventor_xml_path: str = "C:/Users/User/OneDrive/Work/Hydrogenera/R&D/H2 Drone/CAD Model/parameters.xml"

    cruise_endurance_min: float = 10.0  # [min] desired endurance at cruise power

    n_ult: float = 3.0        # [-] max load factor in 45° banked turn

    carbon_fiber_safety_factor: float = 1.5       # [-] safety factor for spar

    airfoil_n_points: int = 241
    horizontal_NACA_code: int = 2410
    vertical_NACA_code: int = 6

    sweep_angle_deg: float = 0.0     # [deg] sweep angle of wing leading edge

    fuselage_longeron_boss_height: float = 0.018 # [m] height of longeron boss
    fuselage_longeron_boss_thickness: float = 0.01 # [m] width of longeron boss
    fuselage_longeron_diameter: float = 0.008 + transition_fit_clearance # [m] diameter of longeron
    fuselage_torque_frame_diameter: float = 0.008 # [m] diameter of torque frame

    fuselage_wing_interface_length: float = 0.01 # [m] length of wing interface with fuselage
    fuselage_wing_interface_fit_clearance: float = 0.00018 # [m] clearance for wing interface fit
    fuselage_wing_interface_insert_depth_factor: float = 2.5 # [-] factor to determine insert depth of wing interface
    fuselage_wing_interface_sleeve_thickness: float = 0.0014 # [m] thickness of wing interface sleeve

    fuselage_axial_restraint_bolt_length: float = 0.025 # [m] length of M3 axial restraint bolt

    tail_taper_ratio: float = 1.0       # [-] lambda = c_tip / c_root
    tail_elevator_chord_ratio: float = 3/10       # [-]
    tail_e_oswald: float = 0.8          # [-] Oswald efficiency factor
    tail_sweep_angle_deg: float = 0.0   # [deg]
    tail_efficiency_factor: float = 0.85 # [-] efficiency factor for x-wing tail

    # --- Tail trim / incidence targets ---
    tail_target_elevator_deg: float = 2.0
    tail_elevator_limit_deg: float = 3.0
    tail_incidence_limit_deg: float = 0.0
    tail_effective_alpha_limit_deg: float = 10.0   # soft geometric/aero limit for cruise trim
    tail_stall_margin_deg: float = 1.0             # margin from polar edge / stall

    tail_span_extension_m: float = 0.0  # [m] added equally to horiz + vert tail span

    tail_downwash_k: float = 0.5
    tail_downwash_offset_deg: float = 0.0
    tail_downwash_min_deg: float = -5.0
    tail_downwash_max_deg: float = 25.0

    # --- Vertical tail sizing (directional stability) ---
    # Factors for Cn_beta model (Eq. 6.73-style)
    vertical_tail_Kf1: float = 0.75            # [-] fin/fuselage effectiveness factor (~0.65–0.85)
    vertical_tail_eta_v: float = 1.0           # [-] q_v / q (dynamic pressure ratio at VT)
    vertical_tail_dsigma_dbeta: float = 0.0    # [-] sidewash gradient at VT (0..~0.4 typical)

    # --- Prop clearance margins for tail-mounted props ---
    prop_fuselage_clearance_m: float = 0.02   # clearance between prop disk and fuselage OD

    # --- Control surface drag model ---
    control_surface_drag_k: float = 0.015

    # --- Maneuver-only elevator modeling (separate from cruise trim) ---
    tail_elevator_span_start: float = 0.1   # [-] 0..1
    tail_elevator_span_end: float = 0.9     # [-] 0..1
    tail_elevator_max_up_deg: float = -35.0  # [deg] up elevator is negative (typ. -25) :contentReference[oaicite:2]{index=2}
    tail_elevator_max_down_deg: float = 35.0 # [deg] down elevator is positive (typ. +20) :contentReference[oaicite:3]{index=3}
    tail_cg_root_chord_ratio: float = 0.5 # [-] ratio of tail cg to root chord

    tail_elevator_rod_support_width: float = 0.01   # [m] width of elevator rod support

    # --- Elevator hinge moment estimation ---
    tail_elevator_max_deflection_deg: float = 25.0  # [deg] max allowed elevator deflection for hinge moment (use larger of |up|, |down|)
    elevator_Ch_delta_per_rad: float = 0.03  # [-] hinge moment coeff derivative C_hδ, typical 0.01–0.03 for plain trailing-edge

    # --- Servo torque (T_servo ≈ M_h * r_horn / (η * r_servo), add 2x–3x margin) ---
    servo_arm_radius_m: float = 0.020  # [m] r_servo: servo arm radius (pivot to pushrod hole)
    elevator_horn_radius_m: float = 0.018  # [m] r_horn: elevator horn radius (hinge line to pushrod hole)
    servo_efficiency_eta: float = 0.8  # [-] η: efficiency (0.7–0.9; friction, flex, slop)
    servo_torque_margin: float = 2.0  # [-] multiply T_servo by this (2x typical, 3x for fast/aggressive)

    servo_width: float = 0.0276 # [m] width of servo
    servo_height: float = 0.0276 # [m] height of servo
    servo_depth: float = 0.0082 # [m] depth of servo
    servo_cables_hole_width: float = 0.01 # [m] width of servo cables hole
    servo_cables_hole_height: float = 0.01 # [m] height of servo cables hole
    servo_mount_ears_width: float = 0.004 # [m] width of servo mount ears
    servo_mount_ears_thickness: float = 0.001 # [m] thickness of servo mount ears
    servo_mount_ears_location_from_top: float = 0.010 # [m] location of servo mount ears from top
    servo_mount_bolt_length: float = 0.004 # [m] length of servo mount bolt
    servo_arm_length: float = 0.0024 # [m] length of servo arm
    servo_housing_thickness: float = 0.006 # [m] thickness of servo housing
    servo_cover_thickness: float = 0.001 # [m] thickness of servo cover
    servo_cover_bolt_length: float = 0.004 # [m] length of servo cover bolt

    # servo_housing_bolt_M3_OD: float = 0.003 # [m] outer diameter of M3 prop mount bolt
    # servo_housing_bolt_M3_head_width: float = 0.003 # [m] width of M3 bolt head
    # servo_housing_bolt_M3_hole_OD: float = 0.0032 # [m] outer diameter of M3 bolt hole
    # servo_housing_bolt_M3_length: float = 0.008 # [m] length of M3 bolt
    # servo_housing_bolt_M3_head_OD: float = 0.0055 # [m] outer diameter of M3 bolt head

    tail_dbox_width_ratio: float = 0.0
    tail_elevator_rod_id_over_od: float = 0.0
    elevator_rod_bearing_length_m: float = 0.008

    # Optional: override elevator effectiveness tau_e (otherwise estimated from CE/Ch chart)
    tail_elevator_tau_override: float     = -1.0  # [-] set >0 to force tau_e

    # Maneuver reference dynamic pressure ratio at tail (q_h/q). If unsure, 0.9–1.0 is fine.
    tail_eta_h_maneuver: float = 1.0          # [-]

    vertical_tail_rudder_chord_ratio: float = 3/10       # [-]
    vertical_tail_rudder_span_start: float = 0.1  # [-] 0..1
    vertical_tail_rudder_span_end: float = 0.9    # [-] 0..1
    vertical_tail_rudder_max_left_deg: float = -35.0  # [deg] up rudder is negative (typ. -25) :contentReference[oaicite:2]{index=2}
    vertical_tail_rudder_max_right_deg: float = 35.0 # [deg] down rudder is positive (typ. +20) :contentReference[oaicite:3]{index=3}

    vertical_tail_rudder_rod_oversize_factor: float = 1.2

    NACA2410_zero_lift_angle_of_attack_deg: float = -2.250  # [deg]
    NACA0012_zero_lift_angle_of_attack_deg: float = 0.0  # [deg]
    NACA0015_zero_lift_angle_of_attack_deg: float = 0.0  # [deg]

    taper_ratio: float = 1.0
    wing_plain_flap_chord_ratio: float = 3/10
    e_oswald: float = 0.8

    wing_spar_id_over_od: float = 6/8
    wing_dbox_width_ratio: float = 0.1
    wing_second_spar_scale: float = 1/3
    wing_bolt_hatch_depth: float = 0.03 # [m] depth of wing bolt hatch
    wing_cg_root_chord_ratio: float = 0.5 # [-] ratio of wing cg to root chord
    wing_spar_boss_factor: float = 1.5 # [-] factor to calculate wing spar boss diameter
    wing_assembly_interface_depth: float = 0.01 # [m] depth of wing assembly interface
    wing_assembly_interface_sleeve_thickness: float = 0.003 # [m] thickness of wing assembly interface sleeve

    fuselage_body_section_B_length: float = c_root*1.1         # [m] length of fuselage body section B
    fuselage_body_section_A_length: float = (fuselage_body_length - c_root*1.1)*2.25/3 - 0.03         # [m] length of fuselage body section A
    fuselage_body_section_C_length: float = (fuselage_body_length - c_root*1.1)*0.75/3 + 0.03         # [m] length of fuselage body section C
    # fuselage_body_section_A_length: float = 0.2         # [m] length of fuselage body section A
    # fuselage_body_section_C_length: float = fuselage_body_section_B_length - fuselage_body_section_A_length          # [m] length of fuselage body section C

    fuselage_bolt_hatch_depth: float = 0.015 # [m] depth of wing bolt hatch
    fuselage_nose_longeron_length: float = 0.03444 # [m] length of fuselage nose longeron
    fuselage_tail_longeron_length: float = 0.03444 # [m] length of fuselage tail longeron
    fuselage_longeron_lenght: float = fuselage_nose_longeron_length + fuselage_tail_longeron_length + fuselage_body_section_A_length + fuselage_body_section_B_length + fuselage_body_section_C_length         # [m] length of fuselage longeron

    fuselage_rib_thickness: float = 0.004 # [m] thickness of fuselage rib
    fuselage_stinger_thickness: float = 0.004 # [m] thickness of fuselage stinger
    fuselage_stinger_height: float = 0.003 # [m] height of fuselage stinger

    fuselage_bulkhead_thickness: float = 0.004
    fuselage_bulkhead_inner_OD: float = 0.094   # [m] inner diameter of fuselage bulkhead (model: fuselage_OD - 2*fuselage_bulkhead_radial_ring_width_m)
    fuselage_bulkhead_radial_ring_width_m: float = 0.023 # [m] radial ring width w_ring; Di,frame = Do - 2*w_ring (sizing variable)
    fuselage_bulkhead_radial_ring_thickness: float = 0.005 # [m]

    fuselage_B_front_bulkhead_location: float = fuselage_body_section_B_length/4 # [m] 
    fuselage_B_rear_bulkhead_location: float = fuselage_body_section_B_length/3.2 # [m] 
    fuselage_tail_front_bulkhead_location: float = 0.04 # [m] 
    fuselage_tail_rear_bulkhead_location: float = 0.11 # [m] 

    # Fuselage panel buckling / layout
    fuselage_design_torsion_Nm: float = 0.0   # if 0, code uses 0.15 * wing bending moment
    fuselage_frame_spacing_target_m: float = 0.12
    fuselage_frame_thickness: float = 0.004 # [m] thickness of fuselage frame
    fuselage_stringer_spacing_target_m: float = 0.06
    fuselage_stringer_thickness: float = 0.004 # [m] thickness of fuselage stinger
    fuselage_stringer_height: float = 0.003 # [m] height of fuselage stinger

    fuselage_panel_buckling_margin: float = 0.50
    fuselage_panel_kc: float = 4.0
    fuselage_panel_ks: float = 5.5

    # Longeron modeling
    fuselage_longeron_id_over_od: float = 6/8
    fuselage_longeron_bending_load_fraction: float = 0.70

    # fuselage_torsion_cap_shoulder_thickness: float = 0.005 # [m] thickness of fuselage torsion cap shoulder thickness
    # fuselage_torsion_cap_shoulder_height: float = 0.005 # [m] height of fuselage torsion cap shoulder height

    printed_vertical_small_hole_additional_clearance: float = 0.0004 # [m] additional clearance for small hole (0.5 mm) (up to 3mm hole)
    printed_parts_sf: float = 4.0
    filament_density: float = 1210.0
    filament_tensile_strength_MPa: float = 32.0
    filament_max_bending_stress_MPa: float = 58.0
    filament_compressive_strength_MPa: float = 58.0
    filament_shear_strength_MPa: float = filament_tensile_strength_MPa / math.sqrt(3)

    # Material for printed skin / stiffeners
    filament_E_Pa = 2.5e9
    filament_nu = 0.35

    # Stiffener sizing knobs for torsional skin buckling
    stiffener_panel_buckling_margin = 0.50
    stiffener_panel_k_shear = 5.5

    # Printable stiffener geometry limits, in multiples of skin thickness
    stiffener_t_min_factor = 1.0
    stiffener_t_max_factor = 2.0
    stiffener_h_min_factor = 2.0
    stiffener_h_max_factor = 4.0

    carbon_fiber_density: float = 1800.0
    carbon_fiber_max_bending_stress_MPa: float = 1500.0
    carbon_fiber_E_Pa: float = 300e9  # [Pa] Young's modulus for CFRP tube (curvature / compressive strain screening)
    carbon_fiber_comp_strain_allow: float = 0.003  # [-] allowable compressive strain for tube microbuckling screening (e.g. 0.3%)

    snap_fit_extension_length: float = 0.03 # [m] length of snap fit extension

    propeller_efficiency: float = 0.65
    n_propellers: int = 4
    propulsion_margin: float = 1.3
    propulsion_power_margin: float = 1.3

    prop_clearance_from_wing_LE: float = 0.005 # [m] clearance from fuselage OD

    hole_boss_factor: float = 1.8 # [-] factor to calculate hole boss diameter
    bolt_pocket_diameter_clearance: float = 0.0017 # [m] clearance between bolt pocket and bolt head
    tube_clamp_boss_width: float = 0.01 # [m] width of tube clamp boss

    # motor_OD: float = 0.0339 # [m] outer diameter of motor
    # motor_body_length: float = 0.020 # [m] length of motor body
    # motor_housing_wall_thickness: float = 0.002 # [m] thickness of motor housing
    # motor_mount_bolt_M4_OD: float = 0.004 # [m] outer diameter of motor M4 mount bolt
    # motor_housing_flange_thickness: float = 0.004 # [m] thickness of motor housing flange thickness
    # prop_nacelle_OD: float = 0.0339 + motor_housing_wall_thickness*2 + motor_mount_bolt_M4_OD*hole_boss_factor*2 # [m] outer diameter of prop nacelle
    # prop_nacelle_length: float = motor_body_length + motor_housing_wall_thickness # [m] length of prop nacelle
    # motor_housing_wall_thickness: float = 0.004 # [m] thickness of motor housing
    # prop_housing_ID: float = motor_OD # [m] inner diameter of prop nacelle
    # prop_nacelle_OD: float = prop_housing_ID + motor_housing_wall_thickness*2 # [m] outer diameter of prop nacelle
    # prop_nacelle_firewall_thickness: float = 0.030 # [m] depth of prop nacelle firewall
    # prop_nacelle_groove_depth: float = 0.003 # [m] SHOULD EQUAL TO prop_mount_bolt_head_width; depth of prop nacelle groove

    motor_OD: float = 0.033 # [m] outer diameter of motor
    motor_body_length: float = 0.0261 # [m] length of motor body without shaft
    prop_mount_bolt_circular_location: float = 0.019 # [m] circular location of M3 prop mount bolt
    motor_mount_bolt_base_thickness: float = 0.005 # [m] length of motor mount bolt thread
    motor_cabling_clearance_width: float = 0.010 # [m] width of motor cabling clearance
    motor_cabling_clearance_height: float = 0.005 # [m] height of motor cabling clearance
    motor_mount_bolt_length: float = 0.009 # [m] length of motor mount bolt

    tail_wing_mount_bolt_length: float = 0.008 # [m] length of tail wing mount bolt

    elevator_pushrod_OD: float = 0.003 # [m] outer diameter of elevator pushrod

    clamp_M8_ID: float = 0.008 # [m] inner diameter of M8 clamp
    clamp_M8_OD: float = 0.010 # [m] outer diameter of M8 clamp
    clamp_M8_width: float = 0.007 # [m] width of M8 clamp
    clamp_M8_washer_OD: float = 0.016 # [m] outer diameter of M8 washer
    clamp_M8_washer_thickness: float = 0.002 # [m] thickness of M8 washer

    threaded_insert_M2_fixing_hole_OD: float = 0.003 # [m] outer diameter of M2 threaded insert fixing hole
    threaded_insert_M2_lenght: float = 0.0041 # [m] length of M2 threaded insert
    bolt_M2_OD: float = 0.002 # [m] outer diameter of M2 prop mount bolt
    bolt_M2_head_width: float = 0.002 # [m] width of M2 bolt head
    bolt_M2_hole_OD: float = 0.0022 # [m] outer diameter of M2 bolt hole
    bolt_M2_head_OD: float = 0.004 # [m] outer diameter of M2 bolt head

    threaded_insert_M3_fixing_hole_OD: float = 0.004 # [m] outer diameter of M3 threaded insert fixing hole
    threaded_insert_M3_lenght: float = 0.0059 # [m] length of M3 threaded insert
    threaded_insert_M3_lenght_short: float = 0.0041 # [m] length of M3 threaded insert
    bolt_M3_OD: float = 0.003 # [m] outer diameter of M3 prop mount bolt
    bolt_M3_head_width: float = 0.003 # [m] width of M3 bolt head
    bolt_M3_hole_OD: float = 0.0032 # [m] outer diameter of M3 bolt hole
    bolt_M3_head_OD: float = 0.0055 # [m] outer diameter of M3 bolt head
    spring_washer_M3_OD: float = 0.0062 # [m] outer diameter of M3 spring washer
    spring_washer_M3_thickness: float = 0.001 # [m] thickness of M3 spring washer

    threaded_insert_M4_fixing_hole_OD: float = 0.0056 # [m] outer diameter of M4 threaded insert fixing hole
    threaded_insert_M4_lenght: float = 0.0081 # [m] length of M4 threaded insert
    bolt_M4_OD: float = 0.004 # [m] outer diameter of M4 prop mount bolt
    bolt_M4_head_width: float = 0.004 # [m] width of M4 bolt head
    bolt_M4_hole_OD: float = 0.0042 # [m] outer diameter of M4 bolt hole
    bolt_M4_head_OD: float = 0.00722 # [m] outer diameter of M4 bolt head

    threaded_insert_M5_fixing_hole_OD: float = 0.0064 # [m] outer diameter of M5 threaded insert fixing hole
    threaded_insert_M5_lenght: float = 0.0095 # [m] length of M5 threaded insert
    bolt_M5_OD: float = 0.005 # [m] outer diameter of M5 prop mount bolt
    bolt_M5_head_width: float = 0.005 # [m] width of M5 bolt head
    bolt_M5_hole_OD: float = 0.0052 # [m] outer diameter of M5 bolt hole
    bolt_M5_head_OD: float = 0.00872 # [m] outer diameter of M5 bolt head

    threaded_insert_M6_fixing_hole_OD: float = 0.0085 # [m] outer diameter of M5 threaded insert fixing hole
    threaded_insert_M6_lenght: float = 0.0127 # [m] length of M5 threaded insert
    bolt_M6_OD: float = 0.006 # [m] outer diameter of M6 prop mount bolt
    bolt_M6_head_width: float = 0.006 # [m] width of M6 bolt head
    bolt_M6_hole_OD: float = 0.0062 # [m] outer diameter of M6 bolt hole
    bolt_M6_head_OD: float = 0.0103 # [m] outer diameter of M6 bolt head

    nose_axial_restraint_boss_width: float = 0.0055 # [m] height of nose axial restraint boss
    nose_axial_restraint_bolt_lenght: float = 0.0135 + 0.02 # [m] length of M6 nose axial restraint bolt

    # --- Propulsion location and clearance ---
    prop_y_position_fraction: float = 1.0
    prop_x_position_fraction: float = 1.0
    prop_fuselage_clearance_min_m: float = 0.02

    # --- Avionics / electronics assumptions ---
    autopilot_class: str = "standard"
    n_servos: int = 4
    esc_efficiency: float = 0.95
    motor_mechanical_efficiency: float = 0.85

    # --- VTOL / tailsitter mode ---
    vtol_enabled: bool = True
    hover_accel_mps2: float = 0.5
    hover_thrust_margin: float = 1.5
    hover_eta: float = 0.50
    hover_power_margin: float = 1.5

    # Tailsitter VTOL mission profile
    vtol_takeoff_hover_time_s: float = 2.0
    vtol_climb_height_m: float = 50.0
    vtol_climb_accel_mps2: float = 0.5
    vtol_climb_vmax_mps: float = 2.0

    transition_stall_margin: float = 1.2
    transition_min_speed_mps: float = 15.0
    transition_time_s: float = 15.0

    accel_to_cruise_time_s: float = 50.0

    battery_energy_reserve_factor: float = 1.08
    mission_profile_steps: int = 30

    assembly_fit_factor: float = 1.05

    battery_location_adjustment_length: float = 0.03 # [m] adjustment length for battery location
    battery_location_adjustment_gap: float = 0.003 # [m] gap for battery location adjustment
    battery_case_thickness: float = 0.002 # [m] thickness of battery case

    # =========================================================
    # Hardcoded value overrides (for testing/validation)
    # When use_hardcoded_values is True, all hardcoded values that are not None are used instead of calculated values
    # If a hardcoded value is None, it is calculated as usual
    # =========================================================
    use_hardcoded_values: bool = True  # Master toggle: if True, use all non-None hardcoded values below
    
    # hardcode_wing_weight_kg: Optional[float] = 0.751  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_tail_weight_kg: Optional[float] = 0.182  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_vertical_tail_weight_kg: Optional[float] = 0.146  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_battery_weight_kg: Optional[float] = 0.750  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_propulsion_weight_kg: Optional[float] = 0.460  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_avionics_weight_kg: Optional[float] = 0.274  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated

    # hardcode_wing_spar_OD: Optional[float] = 0.009  # [m]
    # hardcode_tail_wing_spar_OD: Optional[float] = 0.007  # [m]
    # hardcode_elevator_rod_OD: Optional[float] = 0.003  # [m]

    hardcode_wing_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_tail_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_vertical_tail_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_fuselage_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_fuselage_weight_kg: Optional[float] = 3.0  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_battery_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_propulsion_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_avionics_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated

    hardcode_tail_span: Optional[float] = 0.584  # if not None and use_hardcoded_values is True, span is not sized from tail_volume_ratio
    # hardcode_vertical_tail_span: Optional[float] = None
    hardcode_vertical_tail_span: Optional[float] = 0.584  # if not None and use_hardcoded_values is True, span is not sized from vertical_tail_volume_ratio

    # hardcode_wing_spar_OD: Optional[float] = None  # [m]
    hardcode_wing_spar_OD: Optional[float] = 0.008 + transition_fit_clearance  # [m]
    # hardcode_wing_secondary_spar_OD: Optional[float] = None  # [m]
    hardcode_wing_secondary_spar_OD: Optional[float] = 0.004 + transition_fit_clearance  # [m]
    # hardcode_tail_wing_spar_OD: Optional[float] = None  # [m]
    hardcode_tail_wing_spar_OD: Optional[float] = 0.006 + transition_fit_clearance  # [m]
    # hardcode_elevator_rod_OD: Optional[float] = None  # [m]
    hardcode_elevator_rod_OD: Optional[float] = 0.003 + transition_fit_clearance  # [m]

    # Mutable defaults require default_factory so each instance gets its own dict.
    hardcode_battery: Optional[Dict[str, Any]] = field(default_factory=lambda:
        {
            "name": "Lipo Battery Pack 5500mAh/22.2V 30/60C 177.6Wh",
            "voltage_V": 22.2,
            "capacity_mAh": 5500,
            "mass_kg": 0.821,
            "max_discharge_C": 60,
            "length_m": 0.163,
            "width_m": 0.046,
            "height_m": 0.051,
        }    
    ) # e.g. {"name": "LiPo_4S_8000mAh_30C", "voltage_V": 14.8, "capacity_mAh": 8000, "mass_kg": 0.75, "max_discharge_C": 30, "length_m": 0.165, "width_m": 0.055, "height_m": 0.038}
    # hardcode_battery: Optional[Dict[str, Any]] = None

    battery_length: float = 0.0
    battery_width: float = 0.0
    battery_height: float = 0.0

    hardcode_propulsion: Optional[Dict[str, Any]] = field(default_factory=lambda:
        {
            "name": "GEPRC EM2812 900Kv",
            "motor_kv": 900,
            "voltage_min_V": 23.7,
            "voltage_max_V": 24.0,
            "max_power_W": 1010.0,
            "max_thrust_N": 25.0,
            "prop_diameter_in": 9.0,
            "prop_pitch_in": 5.0,
            "motor_mass_kg": 0.065,
            "prop_mass_kg": 0.020,
        }
    ) # e.g. {"name": "BrotherHobby_AvengerV2_2507_1200kV_HQ7x3p5x3_6S", "motor_kv": 1200, "voltage_min_V": 14.8, "voltage_max_V": 25.2, "max_power_W": 650.0, "max_thrust_N": 19.81, "prop_diameter_in": 7.0, "prop_pitch_in": 3.5, "motor_mass_kg": 0.040, "prop_mass_kg": 0.010}
    # hardcode_propulsion: Optional[Dict[str, Any]] = None

    # =========================================================
    # CG locations (x from nose) [m]
    # IMPORTANT:
    # - If you set these to non-zero values, the model will use them.
    # - If you set them to 0.0, __post_init__ will assign reasonable defaults.
    # =========================================================
    cg_x_wing: float = 0.0
    cg_x_tail: float = 0.0
    cg_x_vertical_tail: float = 0.0
    cg_x_battery: float = 0.0
    cg_x_fuselage: float = 0.0
    cg_x_nose: float = 0.0
    cg_x_body_section_A: float = 0.0
    cg_x_body_section_B: float = 0.0
    cg_x_body_section_C: float = 0.0
    cg_x_fuselage_tail: float = 0.0
    cg_x_propulsion: float = 0.0
    cg_x_autopilot: float = 0.0
    cg_x_power_module: float = 0.0
    cg_x_esc: float = 0.0
    cg_x_gps: float = 0.0
    cg_x_airspeed: float = 0.0
    cg_x_rx: float = 0.0
    cg_x_tel: float = 0.0
    cg_x_servos: float = 0.0
    cg_x_misc: float = 0.0
    cg_x_payload: float = 0.0

    # --- Derived fuselage geometry (computed in __post_init__) ---
    fuselage_OR: float = 0.0
    fuselage_cs_area: float = 0.0
    fuselage_cs_perimeter: float = 0.0
    fuselage_shell_area_moment_of_inertia: float = 0.0
    fuselage_wetted_area: float = 0.0
    fuselage_body_length: float = 0.0

    def __post_init__(self) -> None:
        # Derived speeds
        self.dive_speed = float(self.speed) * 1.5

        # Derived fuselage geometry
        self.fuselage_OR = float(self.fuselage_OD) / 2.0
        self.fuselage_cs_area = math.pi * self.fuselage_OR**2
        self.fuselage_cs_perimeter = 2.0 * math.pi * self.fuselage_OR
        self.fuselage_shell_area_moment_of_inertia = math.pi * self.fuselage_OR**3
        self.fuselage_wetted_area = math.pi * float(self.fuselage_OD) * float(self.fuselage_length)

        # Body split + tail LE
        self.fuselage_body_length = float(self.fuselage_nose_length) + float(self.fuselage_body_section_A_length) + float(self.fuselage_body_section_B_length) + float(self.fuselage_body_section_C_length) + float(self.fuselage_tail_length)

        # Bulkhead inner OD from ring width: Di,frame = Do - 2*w_ring (Do = local fuselage OD)
        self.fuselage_bulkhead_inner_OD = float(self.fuselage_OD) - 2.0 * float(self.fuselage_bulkhead_radial_ring_width_m)

        # Derived battery dimensions from hardcode_battery (if present)
        if self.hardcode_battery is not None:
            self.battery_length = float(self.hardcode_battery.get("length_m", 0.0))
            self.battery_width = float(self.hardcode_battery.get("width_m", 0.0))
            self.battery_height = float(self.hardcode_battery.get("height_m", 0.0))

        # Defaults for CGs ONLY if still zero (so your manual set values are respected)
        def set_if_zero(field: str, value: float) -> None:
            if abs(float(getattr(self, field))) < 1e-12:
                setattr(self, field, float(value))

        set_if_zero("cg_x_wing", 0.27)
        set_if_zero("cg_x_tail", 0.61)
        set_if_zero("cg_x_vertical_tail", 0.625)
        # set_if_zero("cg_x_battery", 0.35)
        # set_if_zero("cg_x_battery", self.fuselage_nose_length + 0.5 * self.battery_length)
        set_if_zero("cg_x_battery", self.fuselage_nose_length + 0.5 * self.battery_length)
        set_if_zero("cg_x_payload", self.fuselage_nose_length/2 - 0.5 * self.payload_length)

        # set_if_zero("cg_x_tail", self.fuselage_length - self.fuselage_tail_length + 0.5 * self.tail_c_root)
        set_if_zero("cg_x_fuselage", 0.5 * self.fuselage_length)

        # If you want propulsion centered on the tailplane by default:
        set_if_zero("cg_x_propulsion", self.cg_x_tail - 0.5 * self.tail_c_root)

        set_if_zero("cg_x_autopilot", 0.5 * self.fuselage_nose_length)
        set_if_zero("cg_x_power_module", 0.9 * self.cg_x_battery)
        set_if_zero("cg_x_esc", self.cg_x_wing)

        set_if_zero("cg_x_gps", 0.5 * self.fuselage_nose_length)
        set_if_zero("cg_x_airspeed", 0.15 * self.fuselage_length)
        set_if_zero("cg_x_rx", 0.5 * self.fuselage_nose_length)
        set_if_zero("cg_x_tel", 0.5 * self.fuselage_nose_length)

        set_if_zero("cg_x_servos", self.cg_x_tail)
        set_if_zero("cg_x_misc", 0.5 * self.fuselage_length)

        def _check_span_segment(name: str, a: float, b: float) -> None:
            if not (0.0 <= a <= 1.0 and 0.0 <= b <= 1.0):
                raise ValueError(f"{name} must be within [0,1]. Got start={a}, end={b}")
            if b <= a:
                raise ValueError(f"{name} must satisfy end > start. Got start={a}, end={b}")

        _check_span_segment("tail_elevator_span_[start,end]",
                            float(self.tail_elevator_span_start),
                            float(self.tail_elevator_span_end))

        _check_span_segment("vertical_tail_rudder_span_[start,end]",
                            float(self.vertical_tail_rudder_span_start),
                            float(self.vertical_tail_rudder_span_end))

