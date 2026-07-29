import math
from dataclasses import dataclass, field
from typing import Optional, Dict, Any

@dataclass
class DesignInputs:
    # --MAIN DRIVING VARIABLES--
    payload_mass: float = 1.2      # [kg]
    payload_length: float = 0.053 # [m] length of payload
    wing_loading: float = 165.0     # [N/m²] provisional endurance-oriented design point
    speed: float = 20.0              # [m/s] provisional economical cruise speed
    dive_speed: float = 50.0         # [m/s]
    fuselage_length: float = 1.15    # [m] overall longitudinal reference length (nose to tail)
    fuselage_OD: float = 0.18        # [m] central pod maximum outer diameter / equivalent diameter
    fuselage_nose_length: float = 0.14        # [m] central pod nose length
    fuselage_tail_length: float = 0.25        # [m] aft pod / pusher-support reference section
    fuselage_nonopt_factor: float = 1.2       # [-] non-optimum factor to approximate total fuselage (structure + fittings)
    fuselage_body_length: float = fuselage_length - fuselage_nose_length - fuselage_tail_length         # [m] length of fuselage body
    tail_c_root: float = 0.18             # [m] horizontal/vertical tail reference root chord
    tail_volume_ratio: float = 0.50     # [-] provisional conventional-tail horizontal volume coefficient
    vertical_tail_volume_ratio: float = 0.045  # [-] total of both fins: Vv = Sv_total*l_v/(S*b)
    c_root: float = 0.28                # [m] main-wing root chord
    # --END MAIN DRIVING VARIABLES--

    # =========================================================
    # AIRCRAFT ARCHITECTURE (new configuration)
    # =========================================================
    n_lift_propellers: int = 4
    n_cruise_propellers: int = 1
    n_horizontal_tail_surfaces: int = 1
    n_vertical_tail_surfaces: int = 2
    n_booms: int = 2

    # Central pod and twin-boom reference geometry.
    # The old variable fuselage_length remains the global x-reference length so
    # existing CG logic stays usable until the fuselage/boom structural model is replaced.
    central_pod_length_m: float = 0.78
    central_pod_width_m: float = 0.18 # max pod width
    central_pod_height_m: float = 0.18 # max pod height
    boom_length_m: float = 0.76
    boom_outer_diameter_m: float = 0.035
    boom_inner_diameter_m: float = 0.031
    boom_lateral_spacing_m: float = 0.70
    boom_x_start_m: float = 0.34
    boom_x_end_m: float = 1.10
    boom_specific_mass_kg_per_m: float = 0.13
    boom_bending_safety_factor: float = 1.5
    boom_forward_mount_mass_each_kg: float = 0.055
    boom_tail_mount_mass_each_kg: float = 0.035
    tail_crossmember_mass_kg: float = 0.080
    hardcode_boom_mass_total_kg: Optional[float] = None
    # Booms are always auto-sized for strength (boom_outer_diameter_m acts as the
    # starting/minimum geometry). Set these to force the final tube diameters instead
    # (applied when use_hardcoded_values is True).
    hardcode_boom_outer_diameter_m: Optional[float] = None
    hardcode_boom_inner_diameter_m: Optional[float] = None

    # Central pod structure: printed shell + frames + wing carry-through fittings.
    central_pod_min_shell_thickness_m: float = 0.0012
    central_pod_frame_spacing_m: float = 0.12
    central_pod_frame_axial_thickness_m: float = 0.004
    central_pod_frame_ring_width_m: float = 0.012
    central_pod_wing_carrythrough_mass_kg: float = 0.16
    central_pod_nonstructural_factor: float = 1.15
    central_pod_bending_load_fraction: float = 0.25
    central_pod_design_torsion_fraction: float = 0.20
    hardcode_central_pod_mass_kg: Optional[float] = None

    # Lift-motor mounts and structural load path.
    lift_motor_mount_mass_each_kg: float = 0.045
    lift_motor_mount_reinforcement_factor: float = 1.15
    lift_motor_max_thrust_fraction_for_structure: float = 1.0
    wing_primary_spar_chord_fraction: float = 0.30
    use_lift_motor_loads_in_wing_structure: bool = True

    wing_z_above_cg_m: float = 0.10
    tailplane_z_above_cg_m: float = 0.03

    # No landing gear is fitted; take-off and recovery are VTOL.
    landing_gear_enabled: bool = False

    # Four dedicated vertical-lift units. Coordinates are aircraft axes:
    # x from nose, y from centreline (left negative), z positive upwards.
    lift_prop_x_positions_m: tuple = (0.31, 0.31, 0.69, 0.69)
    lift_prop_y_positions_m: tuple = (-0.48, 0.48, -0.48, 0.48)
    lift_prop_z_positions_m: tuple = (0.10, 0.10, 0.10, 0.10)
    lift_prop_rotation_directions: tuple = ("CCW", "CW", "CW", "CCW")

    # One aft pusher for wing-borne flight.
    cruise_prop_x_m: float = 0.94
    cruise_prop_y_m: float = 0.0
    cruise_prop_z_m: float = 0.02
    cruise_prop_is_pusher: bool = True

    # =========================================================
    # CONFIGURATION AERODYNAMICS / CRUISE-DRAG BUILD-UP
    # =========================================================
    wing_CLmax_3D_factor: float = 0.95
    tail_CLmax_3D_factor: float = 0.90

    # Printed surfaces generally have more skin-friction drag than smooth
    # moulded/composite surfaces. Set to 1.0 for a smooth reference finish.
    printed_surface_drag_multiplier: float = 1.15
    body_laminar_fraction: float = 0.05

    # Central pod and boom aerodynamic form/interference factors.
    central_pod_cross_section_form_factor: float = 1.10
    central_pod_interference_factor: float = 1.05
    boom_interference_factor: float = 1.10
    hardcode_central_pod_wetted_area_m2: Optional[float] = None

    # Drag of the four inactive vertical-lift units during wing-borne cruise.
    # Supported states: folded, stopped_aligned, stopped_random, windmilling.
    lift_prop_cruise_state: str = "stopped_random"
    lift_prop_blade_count: int = 2
    lift_prop_mean_blade_chord_ratio: float = 0.08
    lift_prop_blade_section_CD: float = 1.10
    lift_prop_drag_area_factor: float = 0.15
    lift_prop_equivalent_drag_area_each_m2: Optional[float] = None

    # Motor nacelle and support drag. support_CdA is an equivalent drag area.
    lift_motor_nacelle_diameter_m: float = 0.055
    lift_motor_nacelle_CD: float = 0.80
    lift_motor_support_CdA_each_m2: float = 0.0010

    # Fuel-cell cooling openings and miscellaneous exposed hardware, as CdA.
    fuel_cell_cooling_CdA_m2: float = 0.0025
    miscellaneous_CdA_m2: float = 0.0015
    configuration_junction_CD0: float = 0.0040

    # Surface counts are explicit: one continuous horizontal tailplane and two fins.
    
    export_inventor_xml: bool = True

    transition_fit_clearance: float = 0.00024 # [m] clearance for transition fit (0.24 mm)
    transition_fit_clearance_coated_carbon: float = 0.0005 # [m] clearance for transition fit (0.24 mm)

    payload_fraction_hist: float = 0.33  # W_payload / W0

    inventor_xml_path: str = "C:/Users/User/OneDrive/Work/Hydrogenera/R&D/H2 Drone/CAD Model/parameters.xml"

    cruise_endurance_min: float = 90.0  # [min] mission target; fuel-cell energy model will satisfy this

    n_ult: float = 3.0        # [-] max load factor in 45° banked turn

    carbon_fiber_safety_factor: float = 1.5       # [-] safety factor for spar

    airfoil_n_points: int = 241
    horizontal_NACA_code: int = 2410   # provisional; dedicated low-Re airfoil selection follows later
    vertical_NACA_code: int = 6

    sweep_angle_deg: float = 3.0     # [deg] mild leading-edge sweep

    fuselage_longeron_boss_height: float = 0.018 # [m] height of longeron boss
    fuselage_longeron_boss_thickness: float = 0.002 # [m] width of longeron boss
    fuselage_longeron_diameter: float = 0.008 + transition_fit_clearance_coated_carbon # [m] diameter of longeron
    fuselage_torque_frame_diameter: float = 0.008 # [m] diameter of torque frame

    fuselage_wing_interface_length: float = 0.01 # [m] length of wing interface with fuselage
    fuselage_wing_interface_fit_clearance: float = 0.00025 # [m] clearance for wing interface fit
    fuselage_wing_interface_insert_depth_factor: float = 4.5 # [-] factor to determine insert depth of wing interface
    fuselage_wing_interface_sleeve_thickness: float = 0.003 # [m] thickness of wing interface sleeve

    fuselage_axial_restraint_bolt_length: float = 0.0215 # [m] length of M3 axial restraint bolt

    tail_taper_ratio: float = 0.65      # [-] lambda = c_tip / c_root
    tail_elevator_chord_ratio: float = 3/10       # [-]
    tail_e_oswald: float = 0.8          # [-] Oswald efficiency factor
    tail_sweep_angle_deg: float = 0.0   # [deg]
    tail_efficiency_factor: float = 0.90 # [-] conventional tailplane efficiency

    # --- Tail trim / incidence targets ---
    tail_target_elevator_deg: float = 0.0
    tail_elevator_limit_deg: float = 15.0
    tail_incidence_limit_deg: float = 2.0
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

    tail_wing_stringer_thickness: float = 0.002 # [m] thickness of fuselage stinger
    tail_wing_stringer_height: float = 0.002 # [m] height of fuselage stinger

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

    camera_power_unit_length: float = 0.0335 # [m] length of camera power unit
    camera_power_unit_width: float = 0.0335 # [m] width of camera power unit
    camera_power_unit_height: float = 0.0112 # [m] height of camera power unit
    camera_power_unit_cooling_channel_thickness: float = 0.01 # [m] thickness of camera power unit cooling channel
    camera_power_unit_housing_front_part_length: float = 0.018 # [m] height of camera power unit
    camera_lens_diameter: float = 0.0139 # [m] diameter of camera
    camera_lens_length: float = 0.0113 # [m] length of camera lens
    camera_body_width: float = 0.0188 # [m] width of camera body
    camera_body_length: float = 0.0121 # [m] width of camera body
    camera_body_mount_hole_location: float = 0.0069 # [m] location of camera body mount hole from top
    camera_location_plane: float = 0.088 # [m] location of camera in plane

    camera_housing_thickness: float = 0.002 # [m] thickness of camera housing

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

    taper_ratio: float = 0.55
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

    fuselage_body_section_B_length: float = c_root*1.1         # [m] length of fuselage body section B (wing bay)
    # Body axial order from nose: nose → section C → section B → section A → tail (lengths estimated in __post_init__)
    fuselage_body_section_A_length: float = 0.0         # [m] aft body section (before tail)
    fuselage_body_section_C_length: float = 0.0         # [m] first body section after nose
    # fuselage_body_section_A_length: float = 0.2         # [m] length of fuselage body section A
    # fuselage_body_section_C_length: float = fuselage_body_section_B_length - fuselage_body_section_A_length          # [m] length of fuselage body section C

    # =========================================================
    # Component specific mass factors [kg/m]
    # These are the default source for structural component masses in script.py:
    #   mass_component = specific_weight_component * component_length
    # Hardcoded mass overrides below still take precedence when enabled.
    # =========================================================
    print_weight_reduction: float = 1.0 # 0.4305

    wing_specific_weight_kg_per_m: float = 0.77 * print_weight_reduction
    tail_wing_specific_weight_kg_per_m: float = 0.41 * print_weight_reduction
    vertical_tail_wing_specific_weight_kg_per_m: float = 0.41 * print_weight_reduction
    fuselage_nose_specific_weight_kg_per_m: float = 1.5 * print_weight_reduction
    fuselage_A_specific_weight_kg_per_m: float = 1.35 * print_weight_reduction
    fuselage_B_specific_weight_kg_per_m: float = 2.09 * print_weight_reduction
    fuselage_C_specific_weight_kg_per_m: float = 1.77 * print_weight_reduction
    fuselage_tail_specific_weight_kg_per_m: float = 1.53 * print_weight_reduction

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
    fuselage_assembly_fit_offset_from_top: float = 0.033 # [m]
    fuselage_assembly_fitting_depth: float = 0.020 # [m] depth of fuselage assembly fitting
    
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

    propeller_efficiency: float = 0.72  # cruise propeller efficiency; lift efficiency is separate
    n_propellers: int = 4              # legacy alias; kept equal to n_lift_propellers in __post_init__
    propulsion_margin: float = 1.30     # legacy cruise-thrust margin
    propulsion_power_margin: float = 1.20  # legacy cruise-power margin

    prop_clearance_from_wing_LE: float = 0.005 # [m] clearance from fuselage OD

    hole_boss_factor: float = 1.8 # [-] factor to calculate hole boss diameter
    bolt_pocket_diameter_clearance: float = 0.0017 # [m] clearance between bolt pocket and bolt head
    tube_clamp_boss_width: float = 0.01 # [m] width of tube clamp boss

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

    cubepilot_height: float = 0.034 # [m] height of Cubepilot
    cubepilot_width: float = 0.048 # [m] width of Cubepilot
    cubepilot_depth: float = 0.040 # [m] depth of Cubepilot
    cubepilot_pin_section_height: float = 0.011 # [m] height of Cubepilot pin section

    mount_hole_scale_factor: float = 1 # [-] factor to scale mount hole diameter

    threaded_insert_M2_fixing_hole_OD: float = 0.003*mount_hole_scale_factor # [m] outer diameter of M2 threaded insert fixing hole
    threaded_insert_M2_lenght: float = 0.0041 # [m] length of M2 threaded insert
    bolt_M2_OD: float = 0.002 # [m] outer diameter of M2 prop mount bolt
    bolt_M2_head_width: float = 0.002 # [m] width of M2 bolt head
    bolt_M2_hole_OD: float = 0.0022*mount_hole_scale_factor # [m] outer diameter of M2 bolt hole
    bolt_M2_head_OD: float = 0.005 # [m] outer diameter of M2 bolt head

    threaded_insert_M3_fixing_hole_OD: float = 0.004*mount_hole_scale_factor # [m] outer diameter of M3 threaded insert fixing hole
    threaded_insert_M3_lenght: float = 0.0059 # [m] length of M3 threaded insert
    threaded_insert_M3_lenght_short: float = 0.0041 # [m] length of M3 threaded insert
    bolt_M3_OD: float = 0.003 # [m] outer diameter of M3 prop mount bolt
    bolt_M3_head_width: float = 0.003 # [m] width of M3 bolt head
    bolt_M3_hole_OD: float = 0.0032*mount_hole_scale_factor # [m] outer diameter of M3 bolt hole
    bolt_M3_head_OD: float = 0.0065 # [m] outer diameter of M3 bolt head
    spring_washer_M3_OD: float = 0.0062 # [m] outer diameter of M3 spring washer
    spring_washer_M3_thickness: float = 0.001 # [m] thickness of M3 spring washer

    threaded_insert_M4_fixing_hole_OD: float = 0.0056*mount_hole_scale_factor # [m] outer diameter of M4 threaded insert fixing hole
    threaded_insert_M4_lenght: float = 0.0081 # [m] length of M4 threaded insert
    bolt_M4_OD: float = 0.004 # [m] outer diameter of M4 prop mount bolt
    bolt_M4_head_width: float = 0.004 # [m] width of M4 bolt head
    bolt_M4_hole_OD: float = 0.0042*mount_hole_scale_factor # [m] outer diameter of M4 bolt hole
    bolt_M4_head_OD: float = 0.00822 # [m] outer diameter of M4 bolt head

    threaded_insert_M5_fixing_hole_OD: float = 0.0064*mount_hole_scale_factor # [m] outer diameter of M5 threaded insert fixing hole
    threaded_insert_M5_lenght: float = 0.0095 # [m] length of M5 threaded insert
    bolt_M5_OD: float = 0.005 # [m] outer diameter of M5 prop mount bolt
    bolt_M5_head_width: float = 0.005 # [m] width of M5 bolt head
    bolt_M5_hole_OD: float = 0.0052*mount_hole_scale_factor # [m] outer diameter of M5 bolt hole
    bolt_M5_head_OD: float = 0.00972 # [m] outer diameter of M5 bolt head

    threaded_insert_M6_fixing_hole_OD: float = 0.0085*mount_hole_scale_factor # [m] outer diameter of M5 threaded insert fixing hole
    threaded_insert_M6_lenght: float = 0.0127 # [m] length of M5 threaded insert
    bolt_M6_OD: float = 0.006 # [m] outer diameter of M6 prop mount bolt
    bolt_M6_head_width: float = 0.006 # [m] width of M6 bolt head
    bolt_M6_hole_OD: float = 0.0062*mount_hole_scale_factor # [m] outer diameter of M6 bolt hole
    bolt_M6_head_OD: float = 0.0113 # [m] outer diameter of M6 bolt head

    nose_axial_restraint_boss_width: float = 0.0055 # [m] height of nose axial restraint boss
    nose_axial_restraint_bolt_lenght: float = 0.008 # [m] length of M6 nose axial restraint bolt

    # --- Propulsion location and clearance ---
    prop_y_position_fraction: float = 0.55  # legacy single-value approximation for lift-prop structural loading
    prop_x_position_fraction: float = 0.50  # legacy single-value approximation
    prop_fuselage_clearance_min_m: float = 0.02
    tail_mounted_propulsion: bool = False  # lift motors are on wing/booms; pusher is aft of the pod

    # --- Avionics / electronics assumptions ---
    autopilot_class: str = "standard"
    n_servos: int = 5  # 2 ailerons + 1 elevator + 2 rudders
    n_motors: int = 5  # derived again in __post_init__: 4 lift + 1 cruise
    esc_efficiency: float = 0.95
    motor_mechanical_efficiency: float = 0.85

    # --- VTOL / separate lift-plus-cruise mode ---
    vtol_enabled: bool = True
    hover_accel_mps2: float = 0.5
    hover_thrust_margin: float = 1.50
    hover_eta: float = 0.62  # combined induced/profile/electrical estimate for lift system
    hover_power_margin: float = 1.15

    # Lift-plus-cruise VTOL mission profile
    vtol_takeoff_hover_time_s: float = 10.0
    vtol_climb_height_m: float = 30.0
    vtol_climb_accel_mps2: float = 0.75
    vtol_climb_vmax_mps: float = 3.0

    transition_stall_margin: float = 1.30
    transition_min_speed_mps: float = 12.0
    transition_time_s: float = 8.0

    accel_to_cruise_time_s: float = 20.0

    battery_energy_reserve_factor: float = 1.15
    mission_profile_steps: int = 30

    assembly_fit_factor: float = 1.05

    # =========================================================
    # HYBRID POWER SYSTEM — provisional component values
    # =========================================================
    dc_bus_nominal_voltage_V: float = 22.2
    power_management_efficiency: float = 0.97
    fuel_cell_dc_dc_efficiency: float = 0.95

    fuel_cell_enabled: bool = True
    fuel_cell_name: str = "Placeholder_PEM_FC_800W_net"
    fuel_cell_rated_net_power_W: float = 800.0
    fuel_cell_max_continuous_fraction: float = 0.95
    fuel_cell_min_stable_fraction: float = 0.10
    fuel_cell_system_efficiency_LHV: float = 0.50
    fuel_cell_stack_mass_kg: float = 0.85
    fuel_cell_bop_mass_kg: float = 0.35
    fuel_cell_dc_dc_mass_kg: float = 0.15
    fuel_cell_system_mass_kg: float = 0.0       # derived in __post_init__
    fuel_cell_length_m: float = 0.26
    fuel_cell_width_m: float = 0.16
    fuel_cell_height_m: float = 0.12
    fuel_cell_startup_time_s: float = 30.0
    fuel_cell_ramp_rate_W_per_s: float = 100.0

    hydrogen_storage_enabled: bool = True
    hydrogen_storage_name: str = "Placeholder_300bar_120g_H2_composite_system"
    hydrogen_storage_pressure_bar: float = 300.0
    hydrogen_mass_nominal_kg: float = 0.120
    hydrogen_usable_fraction: float = 0.90
    hydrogen_tank_mass_kg: float = 1.10
    hydrogen_regulator_and_valves_mass_kg: float = 0.18
    hydrogen_tank_length_m: float = 0.45
    hydrogen_tank_outer_diameter_m: float = 0.12
    hydrogen_LHV_kWh_per_kg: float = 33.33
    hydrogen_usable_mass_kg: float = 0.0         # derived in __post_init__
    hydrogen_chemical_energy_Wh: float = 0.0     # derived in __post_init__
    fuel_cell_usable_electrical_energy_Wh: float = 0.0  # derived in __post_init__

    # Buffer battery provides VTOL/transition peak power and covers FC transients.
    buffer_battery_usable_fraction: float = 0.90
    buffer_battery_min_soc: float = 0.20
    buffer_battery_takeoff_min_soc: float = 0.80
    buffer_battery_target_soc_cruise: float = 0.70
    fuel_cell_cruise_power_control_mode: str = "load_follow_with_soc_trim"
    fuel_cell_ready_at_takeoff: bool = True  # stack is started and warmed before launch
    continuous_auxiliary_power_W: float = 35.0  # avionics, pumps, control electronics and payload hotel load

    buffer_battery_initial_soc: float = 1.00
    buffer_battery_discharge_margin: float = 0.80  # use 80% of nameplate C-rate for sizing

    # Transition and recovery approximations used by the hybrid mission model.
    transition_average_lift_fraction: float = 0.55
    include_landing_sequence: bool = True
    transition_to_hover_time_s: float = 8.0
    vtol_descent_height_m: float = 30.0
    vtol_descent_vmax_mps: float = 2.0
    vtol_descent_power_factor: float = 0.75
    vtol_landing_hover_time_s: float = 10.0

    hardcode_buffer_battery: Optional[Dict[str, Any]] = field(default_factory=lambda: {
        "name": "Placeholder_LiPo_6S_5000mAh_50C_buffer",
        "voltage_V": 22.2,
        "capacity_mAh": 5000,
        "mass_kg": 0.75,
        "max_discharge_C": 50,
        "length_m": 0.160,
        "width_m": 0.050,
        "height_m": 0.050,
    })

    hardcode_lift_propulsion: Optional[Dict[str, Any]] = field(default_factory=lambda: {
        # Provisional only: increased from the earlier 15-inch placeholder because
        # the converged twin-boom model requires roughly 46 N and 1.1 kW per lift unit.
        "name": "Placeholder_5015_370kV_18x6_6S",
        "motor_kv": 370,
        "voltage_min_V": 18.0,
        "voltage_max_V": 25.2,
        "max_power_W": 1600.0,
        "max_thrust_N": 55.0,
        "prop_diameter_in": 18.0,
        "prop_pitch_in": 6.0,
        "motor_mass_kg": 0.220,
        "prop_mass_kg": 0.055,
    })

    hardcode_cruise_propulsion: Optional[Dict[str, Any]] = field(default_factory=lambda: {
        "name": "Placeholder_4008_380kV_15x8_6S_pusher",
        "motor_kv": 380,
        "voltage_min_V": 18.0,
        "voltage_max_V": 25.2,
        "max_power_W": 800.0,
        "max_thrust_N": 35.0,
        "prop_diameter_in": 15.0,
        "prop_pitch_in": 8.0,
        "motor_mass_kg": 0.160,
        "prop_mass_kg": 0.040,
    })

    lift_thrust_margin: float = 1.50
    lift_power_margin: float = 1.15
    cruise_thrust_margin: float = 1.30
    cruise_power_margin: float = 1.20
    lift_propeller_efficiency: float = 0.62
    cruise_propeller_efficiency: float = 0.72

    battery_location_adjustment_length: float = 0.03 # [m] adjustment length for battery location
    battery_location_adjustment_gap: float = 0.003 # [m] gap for battery location adjustment
    battery_case_thickness: float = 0.002 # [m] thickness of battery case

    battery_case_fuselage_section_B_length: float = 0.110 # [m] length of battery case in fuselage section B

    # =========================================================
    # Hardcoded value overrides (for testing/validation)
    # When use_hardcoded_values is True, all hardcoded values that are not None are used instead of calculated values
    # If a hardcoded value is None, it is calculated as usual
    # =========================================================
    use_hardcoded_values: bool = False  # Start from analytical sizing; enable only verified component overrides
    
    hardcode_wing_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_wing_weight_kg: Optional[float] = 0.8 * print_weight_reduction  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_tail_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_tail_weight_kg: Optional[float] = None  # one complete horizontal tailplane
    # hardcode_vertical_tail_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_vertical_tail_weight_kg: Optional[float] = None  # total for both fins will be handled by surface count
    hardcode_fuselage_A_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_fuselage_A_weight_kg: Optional[float] = 0.190 * print_weight_reduction  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_fuselage_B_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_fuselage_B_weight_kg: Optional[float] = 0.482 * print_weight_reduction  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_fuselage_C_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_fuselage_C_weight_kg: Optional[float] = 0.204 * print_weight_reduction  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    # hardcode_fuselage_nose_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_fuselage_nose_weight_kg: Optional[float] = None
    # hardcode_fuselage_tail_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_fuselage_tail_weight_kg: Optional[float] = None
    # Total fuselage mass = sum of nose + A + B + C + tail when all five are set (see script._fuselage_total_and_segments).

    tail_wing_spar_specific_weight_kg_per_m: float = 0.025 # 6mm OD
    vertical_tail_wing_spar_specific_weight_kg_per_m: float = 0.025 # 6mm OD
    elevator_rod_specific_weight_kg_per_m: float = 0.011 # 3mm OD rod
    rudder_rod_specific_weight_kg_per_m: float = 0.011 # 3mm OD rod
    wing_spar_specific_weight_kg_per_m: float = 0.038 # 8mm OD
    wing_secondary_spar_specific_weight_kg_per_m: float = 0.0115 # 4mm OD
    fuselage_longeron_specific_weight_kg_per_m: float = 0.038 # 8mm OD

    # hardcode_battery_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_propulsion_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_avionics_weight_kg: Optional[float] = None  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_fuselage_longeron_weight_kg: Optional[float] = fuselage_longeron_specific_weight_kg_per_m * 0.7  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_wing_spar_weight_kg: Optional[float] = wing_spar_specific_weight_kg_per_m * 1  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_wing_secondary_spar_weight_kg: Optional[float] = wing_secondary_spar_specific_weight_kg_per_m * 1  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_tail_wing_spar_weight_kg: Optional[float] = tail_wing_spar_specific_weight_kg_per_m * 0.75   # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_vertical_tail_wing_spar_weight_kg: Optional[float] = vertical_tail_wing_spar_specific_weight_kg_per_m * 0.619  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_elevator_rod_weight_kg: Optional[float] = elevator_rod_specific_weight_kg_per_m * 0.75  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated
    hardcode_rudder_rod_weight_kg: Optional[float] = rudder_rod_specific_weight_kg_per_m * 0.619  # [kg] if not None and use_hardcoded_values is True, use this value instead of calculated

    # hardcode_wing_span: Optional[float] = None
    # low AR
    hardcode_wing_span: Optional[float] = None  # let wing loading, root chord and taper size the span
    # high AR
    # hardcode_wing_span: Optional[float] = 1.41613  # if not None and use_hardcoded_values is True, span is not sized from wing_loading
    # hardcode_tail_span: Optional[float] = None
    hardcode_tail_span: Optional[float] = None
    # hardcode_vertical_tail_span: Optional[float] = None
    hardcode_vertical_tail_span: Optional[float] = None

    # Optional wetted areas for drag [m²]. If None, code uses its estimates (fuselage: π·OD·L;
    # wing / horiz. tail / vertical tail: 2× planform for profile-drag scaling). When set and
    # use_hardcoded_values is True, that value is used instead.
    hardcode_fuselage_wetted_area_m2: Optional[float] = None
    hardcode_wing_wetted_area_m2: Optional[float] = None
    hardcode_tail_wetted_area_m2: Optional[float] = None
    hardcode_vertical_tail_wetted_area_m2: Optional[float] = None

    # hardcode_wing_spar_OD: Optional[float] = None  # [m]
    hardcode_wing_spar_OD: Optional[float] = None  # [m]
    # hardcode_wing_secondary_spar_OD: Optional[float] = None  # [m]
    hardcode_wing_secondary_spar_OD: Optional[float] = None  # [m]
    # hardcode_tail_wing_spar_OD: Optional[float] = None  # [m]
    hardcode_tail_wing_spar_OD: Optional[float] = None  # [m]
    # hardcode_elevator_rod_OD: Optional[float] = None  # [m]
    hardcode_elevator_rod_OD: Optional[float] = None  # [m]

    # Skin / shell thickness [m]. Root thickness for lifting surfaces; tip follows MATERIAL.tip_thickness_ratio
    # (same as sized surfaces). Fuselage uses monocoque shell thickness. None = use structural sizing.
    hardcode_fuselage_skin_thickness_m: Optional[float] = None
    hardcode_wing_skin_root_thickness_m: Optional[float] = None
    hardcode_tail_wing_skin_root_thickness_m: Optional[float] = None
    hardcode_vertical_tail_skin_root_thickness_m: Optional[float] = None

    # hardcode_esc_mass: Optional[float] = None  # [kg]
    hardcode_esc_mass: Optional[float] = None  # [kg], total for five ESCs when set
    hardcode_BEC_mass: Optional[float] = None  # [kg]
    hardcode_power_module_mass: Optional[float] = None  # [kg]
    hardcode_rc_rx_mass: Optional[float] = None  # [kg]
    hardcode_telemetry_mass: Optional[float] = None  # [kg]
    hardcode_autopilot_mass: Optional[float] = None  # [kg]
    hardcode_autopilot_powerbrick_mass: Optional[float] = None  # [kg]
    hardcode_servos_mass: Optional[float] = None  # [kg]
    hardcode_camera_mass: Optional[float] = None  # [kg]
    # TO EDIT:
    hardcode_gps_mass: Optional[float] = None  # [kg]
    hardcode_airspeed_mass: Optional[float] = None  # [kg]
    hardcode_misc_electronics_mass: Optional[float] = None  # [kg]

    # Mutable defaults require default_factory so each instance gets its own dict.
    # Legacy battery field retained so unmodified modules still import.
    # __post_init__ mirrors hardcode_buffer_battery into this field.
    hardcode_battery: Optional[Dict[str, Any]] = None
    # hardcode_battery: Optional[Dict[str, Any]] = None

    battery_length: float = 0.0
    battery_width: float = 0.0
    battery_height: float = 0.0

    # Legacy single-propulsion field. It is intentionally None because the new
    # aircraft has independent lift and cruise systems.
    hardcode_propulsion: Optional[Dict[str, Any]] = None
    # hardcode_propulsion: Optional[Dict[str, Any]] = None

    # =========================================================
    # CG locations (x from nose) [m]
    # IMPORTANT:
    # - If you set these to non-zero values, the model will use them.
    # - If you set them to 0.0, __post_init__ will assign reasonable defaults.
    # Mass partners (script._calculate_cg_position): cg_x_propulsion ↔ W_propulsion (sized or
    #   hardcode_propulsion_weight_kg / motor+prop from hardcode_propulsion); cg_x_battery ↔
    #   battery mass; cg_x_payload ↔ payload_mass; avionics rows ↔ hardcode_*_mass fields above
    #   (BEC, power brick, GPS, airspeed, etc.) via avionics.estimate_avionics_and_electrics.
    # =========================================================
    cg_x_wing: float = 0.0
    cg_x_tail: float = 0.0
    cg_x_vertical_tail: float = 0.0
    cg_x_battery: float = 0.0
    cg_x_nose: float = 0.0
    cg_x_body_section_A: float = 0.0
    cg_x_body_section_B: float = 0.0
    cg_x_body_section_C: float = 0.0
    cg_x_fuselage_tail: float = 0.0
    cg_x_propulsion: float = 0.0
    cg_x_autopilot: float = 0.0
    cg_x_power_module: float = 0.0
    cg_x_bec: float = 0.0
    cg_x_autopilot_powerbrick: float = 0.0
    cg_x_esc: float = 0.0
    cg_x_gps: float = 0.0
    cg_x_airspeed: float = 0.0
    cg_x_rx: float = 0.0
    cg_x_tel: float = 0.0
    cg_x_camera: float = 0.0
    cg_x_servos: float = 0.0
    cg_x_misc: float = 0.0
    cg_x_payload: float = 0.0

    # New component CGs for the hybrid twin-boom configuration.
    cg_x_fuel_cell: float = 0.0
    cg_x_hydrogen_tank: float = 0.0
    cg_x_hydrogen_regulator: float = 0.0
    cg_x_lift_propulsion_front: float = 0.0
    cg_x_lift_propulsion_rear: float = 0.0
    cg_x_cruise_propulsion: float = 0.0
    # ESC locations are independent so they can be mounted beside the motors or inside the pod.
    cg_x_lift_esc_front: float = 0.0
    cg_x_lift_esc_rear: float = 0.0
    cg_x_cruise_esc: float = 0.0
    cg_x_booms: float = 0.0
    cg_x_central_pod: float = 0.0
    # Kept only as compatibility aliases. No landing-gear mass enters the model.
    cg_x_landing_gear: float = 0.0
    landing_gear_mass_kg: float = 0.0

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

        # Keep legacy counters consistent while the downstream modules are migrated.
        self.n_propellers = int(self.n_lift_propellers)
        self.n_motors = int(self.n_lift_propellers + self.n_cruise_propellers)

        # Validate the four lift-prop layout.
        layout_lengths = {
            len(self.lift_prop_x_positions_m),
            len(self.lift_prop_y_positions_m),
            len(self.lift_prop_z_positions_m),
            len(self.lift_prop_rotation_directions),
        }
        if layout_lengths != {int(self.n_lift_propellers)}:
            raise ValueError(
                "Lift-prop coordinate and rotation tuples must each contain "
                f"{self.n_lift_propellers} entries."
            )
        if self.n_horizontal_tail_surfaces != 1:
            raise ValueError("This baseline assumes one continuous horizontal tailplane.")
        if self.n_vertical_tail_surfaces != 2:
            raise ValueError("This baseline assumes two vertical fins, one on each boom.")

        valid_prop_states = {"folded", "stopped_aligned", "stopped_random", "windmilling"}
        if str(self.lift_prop_cruise_state).strip().lower() not in valid_prop_states:
            raise ValueError(
                "lift_prop_cruise_state must be one of: "
                + ", ".join(sorted(valid_prop_states))
            )

        # Fuel-cell and hydrogen derived quantities.
        self.fuel_cell_system_mass_kg = (
            float(self.fuel_cell_stack_mass_kg)
            + float(self.fuel_cell_bop_mass_kg)
            + float(self.fuel_cell_dc_dc_mass_kg)
        )
        self.hydrogen_usable_mass_kg = (
            float(self.hydrogen_mass_nominal_kg) * float(self.hydrogen_usable_fraction)
        )
        self.hydrogen_chemical_energy_Wh = (
            self.hydrogen_usable_mass_kg * float(self.hydrogen_LHV_kWh_per_kg) * 1000.0
        )
        self.fuel_cell_usable_electrical_energy_Wh = (
            self.hydrogen_chemical_energy_Wh
            * float(self.fuel_cell_system_efficiency_LHV)
            * float(self.fuel_cell_dc_dc_efficiency)
            * float(self.power_management_efficiency)
        )

        # Mirror the new buffer battery into the legacy field until propulsion_system.py
        # is split into lift, cruise, fuel-cell and battery selections.
        if self.hardcode_buffer_battery is not None:
            self.hardcode_battery = dict(self.hardcode_buffer_battery)

        # Derived central-pod / twin-boom reference geometry.
        # Legacy circular-fuselage quantities are retained as equivalent values for
        # modules that have not yet been migrated.
        self.fuselage_OR = float(self.fuselage_OD) / 2.0
        self.fuselage_cs_area = math.pi * self.fuselage_OR**2
        self.fuselage_cs_perimeter = 2.0 * math.pi * self.fuselage_OR
        self.fuselage_shell_area_moment_of_inertia = math.pi * self.fuselage_OR**3

        pod_box_area = 2.0 * (
            float(self.central_pod_length_m) * float(self.central_pod_width_m)
            + float(self.central_pod_length_m) * float(self.central_pod_height_m)
            + float(self.central_pod_width_m) * float(self.central_pod_height_m)
        )
        # Rounded/nose-taper correction for the pod, plus cylindrical boom wetted area.
        pod_wetted = 0.82 * pod_box_area
        booms_wetted = (
            int(self.n_booms)
            * math.pi
            * float(self.boom_outer_diameter_m)
            * float(self.boom_length_m)
        )
        self.fuselage_wetted_area = pod_wetted + booms_wetted
        if self.use_hardcoded_values:
            hc_sw = self.hardcode_fuselage_wetted_area_m2
            if hc_sw is not None:
                self.fuselage_wetted_area = float(hc_sw)

        # Body split + tail LE (axial order: nose → C → B → A → tail)
        self.fuselage_body_length = (
            float(self.fuselage_nose_length)
            + float(self.fuselage_body_section_C_length)
            + float(self.fuselage_body_section_B_length)
            + float(self.fuselage_body_section_A_length)
            + float(self.fuselage_tail_length)
        )

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

        # PLA
        # low AR
        set_if_zero("cg_x_wing", 0.43)
        # high AR
        # set_if_zero("cg_x_wing", 0.333)
        # ASA Aero
        # set_if_zero("cg_x_wing", 0.287)
        # Wing CG at center of section B: nose + C + B/2 = cg_x_wing.
        self.fuselage_body_section_C_length = (
            float(self.cg_x_wing)
            - float(self.fuselage_nose_length)
            - 0.5 * float(self.fuselage_body_section_B_length)
        )
        # Section A is the leftover body length so nose + C + B + A + tail = fuselage_length.
        self.fuselage_body_section_A_length = (
            float(self.fuselage_length)
            - float(self.fuselage_nose_length)
            - float(self.fuselage_tail_length)
            - float(self.fuselage_body_section_B_length)
            - float(self.fuselage_body_section_C_length)
        )
        self.fuselage_body_length = (
            float(self.fuselage_nose_length)
            + float(self.fuselage_body_section_C_length)
            + float(self.fuselage_body_section_B_length)
            + float(self.fuselage_body_section_A_length)
            + float(self.fuselage_tail_length)
        )
        self.fuselage_longeron_lenght = (
            float(self.fuselage_nose_longeron_length)
            + float(self.fuselage_tail_longeron_length)
            + float(self.fuselage_body_section_A_length)
            + float(self.fuselage_body_section_B_length)
            + float(self.fuselage_body_section_C_length)
        )
        set_if_zero("cg_x_tail", 1.05)
        set_if_zero("cg_x_vertical_tail", 1.05)
        # battery can move 40mm back
        set_if_zero("cg_x_battery", 0.39)
        set_if_zero("cg_x_payload", 0.16)

        set_if_zero("cg_x_nose", 0.5 * self.fuselage_nose_length)
        set_if_zero(
            "cg_x_body_section_A",
            self.fuselage_nose_length
            + self.fuselage_body_section_C_length
            + self.fuselage_body_section_B_length
            + 0.5 * self.fuselage_body_section_A_length,
        )
        set_if_zero("cg_x_body_section_B", self.fuselage_nose_length + self.fuselage_body_section_C_length + 0.5 * (self.fuselage_body_section_B_length))
        set_if_zero("cg_x_body_section_C", self.fuselage_nose_length + 0.5 * (self.fuselage_body_section_C_length))
        set_if_zero(
            "cg_x_fuselage_tail",
            self.fuselage_nose_length
            + self.fuselage_body_section_C_length
            + self.fuselage_body_section_B_length
            + self.fuselage_body_section_A_length
            + 0.5 * (self.fuselage_tail_length),
        )

        # Propulsion CG default: tail region (mass = W_propulsion from propulsion sizing / overrides).
        set_if_zero("cg_x_propulsion", 0.59)  # legacy mass-lumped average of all five propulsors

        set_if_zero("cg_x_autopilot", self.cg_x_body_section_C)
        set_if_zero("cg_x_power_module", self.cg_x_battery)
        set_if_zero("cg_x_bec", self.cg_x_body_section_A)
        set_if_zero("cg_x_autopilot_powerbrick", self.cg_x_body_section_C)
        set_if_zero("cg_x_esc", 0.55)

        set_if_zero("cg_x_gps", self.cg_x_body_section_B)
        set_if_zero("cg_x_airspeed", 0.2 * self.fuselage_nose_length)
        set_if_zero("cg_x_rx", 0.5 * self.cg_x_body_section_A)
        set_if_zero("cg_x_tel", 0.5 * self.cg_x_body_section_A)
        set_if_zero("cg_x_camera", self.camera_lens_length + self.camera_body_length)

        set_if_zero("cg_x_servos", 0.88)
        # TO EDIT:
        set_if_zero("cg_x_misc", 0.5 * self.fuselage_length)

        set_if_zero("cg_x_fuel_cell", 0.57)
        set_if_zero("cg_x_hydrogen_tank", 0.34)
        set_if_zero("cg_x_hydrogen_regulator", 0.56)
        set_if_zero("cg_x_lift_propulsion_front", 0.31)
        set_if_zero("cg_x_lift_propulsion_rear", 0.69)
        set_if_zero("cg_x_cruise_propulsion", self.cruise_prop_x_m)
        set_if_zero("cg_x_lift_esc_front", self.cg_x_lift_propulsion_front)
        set_if_zero("cg_x_lift_esc_rear", self.cg_x_lift_propulsion_rear)
        set_if_zero("cg_x_cruise_esc", 0.82)  # provisional: aft equipment bay, ahead of pusher motor
        set_if_zero("cg_x_booms", 0.5 * (self.boom_x_start_m + self.boom_x_end_m))
        set_if_zero("cg_x_central_pod", 0.5 * self.central_pod_length_m)
        # Compatibility field only; there is no landing gear.
        set_if_zero("cg_x_landing_gear", self.cg_x_central_pod)
        self.landing_gear_mass_kg = 0.0

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


# Component mass CG x-positions on DesignInputs (partners in script._calculate_cg_position).
CG_X_COMPONENT_FIELDS = (
    "cg_x_wing",
    "cg_x_tail",
    "cg_x_vertical_tail",
    "cg_x_battery",
    "cg_x_nose",
    "cg_x_body_section_A",
    "cg_x_body_section_B",
    "cg_x_body_section_C",
    "cg_x_fuselage_tail",
    "cg_x_propulsion",
    "cg_x_autopilot",
    "cg_x_power_module",
    "cg_x_bec",
    "cg_x_autopilot_powerbrick",
    "cg_x_esc",
    "cg_x_gps",
    "cg_x_airspeed",
    "cg_x_rx",
    "cg_x_tel",
    "cg_x_camera",
    "cg_x_servos",
    "cg_x_misc",
    "cg_x_payload",
    "cg_x_fuel_cell",
    "cg_x_hydrogen_tank",
    "cg_x_hydrogen_regulator",
    "cg_x_lift_propulsion_front",
    "cg_x_lift_propulsion_rear",
    "cg_x_cruise_propulsion",
    "cg_x_lift_esc_front",
    "cg_x_lift_esc_rear",
    "cg_x_cruise_esc",
    "cg_x_booms",
    "cg_x_central_pod",
    "cg_x_landing_gear",
)


def compute_derived_cg_x_locations(inputs: DesignInputs) -> Dict[str, float]:
    """
    Chord-fraction and geometry-derived x-locations used in CG weighting
    (script._calculate_cg_position).
    """
    c_MGC_wing = float(inputs.c_root) * (1.0 + float(inputs.taper_ratio)) / 2.0
    c_MGC_tail = float(inputs.tail_c_root) * (1.0 + float(inputs.tail_taper_ratio)) / 2.0

    wing_le_x = float(inputs.cg_x_wing) - 0.5 * c_MGC_wing
    tail_le_x = float(inputs.cg_x_tail) - 0.5 * c_MGC_tail
    vertical_tail_le_x = float(inputs.cg_x_vertical_tail) - 0.5 * c_MGC_tail

    return {
        "wing_le_x_m": wing_le_x,
        "tail_le_x_m": tail_le_x,
        "vertical_tail_le_x_m": vertical_tail_le_x,
        "x_primary_wing_spar_m": wing_le_x + 0.3 * c_MGC_wing,
        "x_secondary_wing_spar_m": wing_le_x + 0.7 * c_MGC_wing,
        "x_primary_tail_spar_m": tail_le_x + 0.3 * c_MGC_tail,
        "x_secondary_tail_spar_m": tail_le_x + 0.7 * c_MGC_tail,
        "x_primary_vertical_tail_spar_m": vertical_tail_le_x + 0.3 * c_MGC_tail,
        "x_secondary_vertical_tail_spar_m": vertical_tail_le_x + 0.7 * c_MGC_tail,
        "x_longerons_center_m": 0.5 * float(inputs.fuselage_length),
    }
