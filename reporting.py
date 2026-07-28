"""
reporting.py

Output formatting and reporting functions.
Includes printing of full battery-aware flight profile / envelope (mission profile).
"""

from design_inputs import DesignInputs
from results import DesignResults


def print_design_summary(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== CONCEPTUAL SUAV DESIGN SUMMARY ===")
    print(f"MTOW mass           : {results.aircraft.W0_mass_kg:.3f} kg")
    print(f"MTOW weight         : {results.aircraft.W0_weight_N:.1f} N")
    print(f"payload mass        : {inputs.payload_mass:.3f} kg")
    print()
    print("Wetted areas (cruise drag model) [m²]:")
    print(f"  Central pod + booms : {results.fuselage.wetted_area_m2:.4f}")
    print(f"  Wing (per ref. S)   : {results.wing.wetted_area_m2:.4f}  (2× planform est. or hardcoded)")
    print(f"  Horizontal tailplane: {results.tail.wetted_area_m2:.4f}")
    print(f"  Twin vertical fins  : {results.vertical_tail.wetted_area_m2:.4f}")
    print()
    print("Mass breakdown:")
    print(f"  Payload           : {inputs.payload_mass:.3f} kg")
    print(f"  Wings (horizontal): {results.wing.structure.W_total_kg * 2:.3f} kg")
    # print(f"  Wings (vertical)  : {results.vertical_wing.structure.W_total_kg * 2:.3f} kg")
    print(f"  Tails (horizontal): {results.tail.structure.W_total_kg * 2:.3f} kg")
    print(f"  Tails (vertical) : {results.vertical_tail.structure.W_total_kg * 2:.3f} kg")
    print(f"  Central pod       : {results.fuselage.W_fuselage_kg:.3f} kg")
    print(f"  Twin booms        : {results.booms.total_mass_kg:.3f} kg")
    print("  Landing gear      : 0.000 kg (not fitted)")
    if results.battery is not None:
        energy_mass = getattr(results.battery, "total_energy_system_mass_kg", results.battery.mass_kg)
        print(f"  Hybrid energy sys.: {energy_mass:.3f} kg")
        print(f"    - Buffer battery: {results.battery.mass_kg:.3f} kg")
        print(f"    - Fuel-cell sys.: {getattr(results.battery, 'fuel_cell_system_mass_kg', 0.0):.3f} kg")
        print(f"    - H2 tank       : {getattr(results.battery, 'hydrogen_tank_mass_kg', 0.0):.3f} kg")
        print(f"    - Regulator     : {getattr(results.battery, 'hydrogen_regulator_mass_kg', 0.0):.3f} kg")
        print(f"    - Stored H2     : {getattr(results.battery, 'hydrogen_nominal_mass_kg', 0.0):.3f} kg")
    if results.propulsion is not None:
        print(f"  Propulsion        : {results.propulsion.total_propulsion_mass_kg:.3f} kg")
    if results.avionics is not None:
        print(f"  Avionics+Electrics: {results.avionics.total_avionics_electrics_mass_kg:.3f} kg")
    print()


def print_wing_results(results: DesignResults) -> None:
    print("=== WING ===")
    w = results.wing
    g = w.geometry
    a = w.aerodynamics
    s = w.structure

    print("Geometry:")
    print(f"  Wing area S         : {g.area_m2:.3f} m²")
    print(f"  Wing span b         : {g.span_m:.3f} m ({g.span_m*100:.1f} cm)")
    print(f"  Aspect ratio AR     : {g.AR:.3f}")
    print(f"  Root chord c_root   : {g.c_root_m:.3f} m ({g.c_root_m*100:.1f} cm)")
    print(f"  Tip chord c_tip     : {g.c_tip_m:.3f} m ({g.c_tip_m*100:.1f} cm)")
    print(f"  Mean Geometric Chord: {g.c_MGC_m:.3f} m ({g.c_MGC_m*100:.1f} cm)")
    print(f"  Mean Aerodynamic Chord: {g.MAC_m:.3f} m ({g.MAC_m*100:.1f} cm)")
    print(f"  MAC y-position      : {g.y_MAC_from_centerline_m:.3f} m from centerline")
    print(f"  AC on MAC           : {g.ac_on_MAC_m:.3f} m from LE")
    print(f"  AC x-position       : {w.x_ac_wing_m:.3f} m")
    print(f"  Wetted area (drag)  : {w.wetted_area_m2:.4f} m²")
    print()
    
    print("Aerodynamics:")
    print(f"  CL_cruise           : {a.CL_cruise:.3f}")
    print(f"  CL_max              : {a.CL_max:.3f}")
    print(f"  CD0                 : {a.CD0:.4f}")
    print(f"  CD0_fuselage        : {a.CD0_fuselage:.4f}")
    print(f"  CD_cruise           : {a.CD_cruise:.4f}")
    print(f"  L/D at cruise       : {a.L_over_D_cruise:.1f}")
    print(f"  Stall speed V_stall : {a.V_stall:.2f} m/s")
    print(f"  Alpha cruise        : {a.alpha_cruise_deg:.2f} deg")
    print(f"  Re at cruise        : {a.Re_cruise:.0f}")
    print(f"  Airfoil max t/c     : {a.max_thickness_fraction_of_chord:.2f}")
    print()
    
    print("Structure:")
    print(f"  Structural height h : {s.h_m*1000:.2f} mm")
    print(f"  Skin thickness (root): {s.t_skin_root_m*1000:.3f} mm")
    print(f"  Skin thickness (tip)  : {s.t_skin_tip_m*1000:.3f} mm")
    print(f"  Web thickness (root)  : {s.t_web_root_m*1000:.3f} mm")
    print(f"  Web thickness (tip)  : {s.t_web_tip_m*1000:.3f} mm")
    print(f"  Spar area (root)     : {s.spar_area_root_m2*1e6:.2f} mm²")
    print(f"  Spar area (tip)      : {s.spar_area_tip_m2*1e6:.2f} mm²")
    print(f"  Number of ribs       : {s.N_ribs}")
    print(f"  Rib spacing          : {s.rib_spacing_m*1000:.2f} mm")
    print(f"  Bending moment       : {s.bending_moment_Nm:.1f} N·m ({s.bending_moment_type})")
    print(f"  Torsion moment       : {s.torsion_Nm:.1f} N·m")
    if getattr(w, "lift_thrust_per_motor_structural_N", 0.0) > 0.0:
        print("  Lift-propulsor concentrated loads (one half-wing):")
        print(f"    Structural thrust/motor : {w.lift_thrust_per_motor_structural_N:.2f} N")
        print(f"    Thrust root moment      : {w.lift_thrust_root_moment_Nm_half:.2f} N·m")
        print(f"    Weight root moment      : {w.lift_weight_root_moment_Nm_half:.2f} N·m")
        print(f"    Shaft-reaction torque   : {w.lift_shaft_torque_Nm_half:.2f} N·m")
        print(f"    Thrust-offset torsion   : {w.lift_offset_torsion_Nm_half:.2f} N·m")
        print(f"    Motor-mount mass total  : {w.lift_mount_mass_total_kg:.3f} kg")
    print()
    print("Mass breakdown (both wings):")
    print(f"  Skin mass           : {s.W_skin_kg * 2:.3f} kg")
    print(f"  Web mass            : {s.W_web_kg * 2:.3f} kg")
    print(f"  Spars mass          : {s.W_spars_kg * 2:.3f} kg")
    print(f"  Ribs mass           : {s.W_ribs_kg * 2:.3f} kg")
    if getattr(w, "wing_stiffener_mass_kg", 0.0) > 0:
        print(f"  Stiffeners mass     : {w.wing_stiffener_mass_kg * 2:.3f} kg")
    print(f"  Total wing mass     : {s.W_total_kg * 2:.3f} kg")
    print()


# VERTICAL MAIN WING REMOVED FOR NOW
# def print_vertical_wing_results(results: DesignResults) -> None:
#     print("=== VERTICAL WING (NACA0012) ===")
#     vw = results.vertical_wing
#     g = vw.geometry
#     a = vw.aerodynamics
#     s = vw.structure

#     print("Geometry:")
#     print(f"  Wing area S         : {g.area_m2:.3f} m²")
#     print(f"  Wing span b         : {g.span_m:.3f} m ({g.span_m*100:.1f} cm)")
#     print(f"  Aspect ratio AR     : {g.AR:.3f}")
#     print(f"  Root chord c_root   : {g.c_root_m:.3f} m ({g.c_root_m*100:.1f} cm)")
#     print(f"  Tip chord c_tip     : {g.c_tip_m:.3f} m ({g.c_tip_m*100:.1f} cm)")
#     print(f"  Mean Geometric Chord: {g.c_MGC_m:.3f} m ({g.c_MGC_m*100:.1f} cm)")
#     print(f"  Mean Aerodynamic Chord: {g.MAC_m:.3f} m ({g.MAC_m*100:.1f} cm)")
#     print(f"  MAC y-position      : {g.y_MAC_from_centerline_m:.3f} m from centerline")
#     print(f"  AC on MAC           : {g.ac_on_MAC_m:.3f} m from LE")
#     print(f"  AC x-position       : {vw.x_ac_wing_m:.3f} m")
#     print()
    
#     print("Aerodynamics:")
#     print(f"  CL_cruise           : {a.CL_cruise:.3f} (no lift in cruise)")
#     print(f"  CL_max              : {a.CL_max:.3f}")
#     print(f"  CD0                 : {a.CD0:.4f}")
#     print(f"  CD_cruise           : {a.CD_cruise:.4f}")
#     print(f"  L/D at cruise       : {a.L_over_D_cruise:.1f}")
#     print(f"  Stall speed V_stall : {a.V_stall:.2f} m/s")
#     print(f"  Alpha cruise        : {a.alpha_cruise_deg:.2f} deg")
#     print(f"  Re at cruise        : {a.Re_cruise:.0f}")
#     print(f"  Airfoil max t/c     : {a.max_thickness_fraction_of_chord:.1%}")
#     print()
    
#     print("Structure:")
#     print(f"  Structural height h : {s.h_m*1000:.2f} mm")
#     print(f"  Skin thickness (root): {s.t_skin_root_m*1000:.3f} mm")
#     print(f"  Skin thickness (tip)  : {s.t_skin_tip_m*1000:.3f} mm")
#     print(f"  Web thickness (root)  : {s.t_web_root_m*1000:.3f} mm")
#     print(f"  Web thickness (tip)  : {s.t_web_tip_m*1000:.3f} mm")
#     print(f"  Spar area (root)     : {s.spar_area_root_m2*1e6:.2f} mm²")
#     print(f"  Spar area (tip)      : {s.spar_area_tip_m2*1e6:.2f} mm²")
#     print(f"  Number of ribs       : {s.N_ribs}")
#     print(f"  Bending moment       : {s.bending_moment_Nm:.1f} N·m")
#     print(f"  Torsion moment       : {s.torsion_Nm:.1f} N·m")
#     print()
#     print("Mass breakdown:")
#     print(f"  Skin mass           : {s.W_skin_kg:.3f} kg")
#     print(f"  Web mass            : {s.W_web_kg:.3f} kg")
#     print(f"  Spars mass          : {s.W_spars_kg:.3f} kg")
#     print(f"  Ribs mass           : {s.W_ribs_kg:.3f} kg")
#     print(f"  Total vertical wing mass: {s.W_total_kg:.3f} kg")
#     print()


def print_vertical_tail_results(results: DesignResults) -> None:
    print("=== VERTICAL TAIL (NACA0012) ===")
    vt = results.vertical_tail
    g = vt.geometry
    a = vt.aerodynamics
    s = vt.structure

    print("Geometry:")
    print(f"  Area per fin        : {g.area_m2:.3f} m²")
    print(f"  Total twin-fin area : {2.0*g.area_m2:.3f} m²")
    print(f"  Height per fin      : {g.span_m:.3f} m ({g.span_m*100:.1f} cm)")
    print(f"  Aspect ratio AR     : {g.AR:.3f}")
    print(f"  Root chord c_root   : {g.c_root_m:.3f} m ({g.c_root_m*100:.1f} cm)")
    print(f"  Tip chord c_tip     : {g.c_tip_m:.3f} m ({g.c_tip_m*100:.1f} cm)")
    print(f"  Mean Geometric Chord: {g.c_MGC_m:.3f} m ({g.c_MGC_m*100:.1f} cm)")
    print(f"  Mean Aerodynamic Chord: {g.MAC_m:.3f} m ({g.MAC_m*100:.1f} cm)")
    print(f"  MAC height station  : {g.y_MAC_from_centerline_m:.3f} m from fin root")
    print(f"  AC on MAC           : {g.ac_on_MAC_m:.3f} m from LE")
    print(f"  AC x-position       : {vt.x_ac_tail_m:.3f} m")
    print(f"  Wetted area (drag)  : {vt.wetted_area_m2:.4f} m²")
    print()
    
    print("Aerodynamics:")
    print(f"  CL_cruise           : {a.CL_cruise:.3f} (no lift in cruise)")
    print(f"  CL_max              : {a.CL_max:.3f}")
    print(f"  CD0                 : {a.CD0:.4f}")
    print(f"  CD_cruise           : {a.CD_cruise:.4f}")
    print(f"  L/D at cruise       : {a.L_over_D_cruise:.1f}")
    print(f"  Stall speed V_stall : {a.V_stall:.2f} m/s")
    print(f"  Alpha cruise        : {a.alpha_cruise_deg:.2f} deg")
    print(f"  Re at cruise        : {a.Re_cruise:.0f}")
    print(f"  Airfoil max t/c     : {a.max_thickness_fraction_of_chord:.2f}")
    print()
    
    print("Structure:")
    print(f"  Structural height h : {s.h_m*1000:.2f} mm")
    print(f"  Skin thickness (root): {s.t_skin_root_m*1000:.3f} mm")
    print(f"  Skin thickness (tip)  : {s.t_skin_tip_m*1000:.3f} mm")
    print(f"  Web thickness (root)  : {s.t_web_root_m*1000:.3f} mm")
    print(f"  Web thickness (tip)  : {s.t_web_tip_m*1000:.3f} mm")
    print(f"  Spar area (root)     : {s.spar_area_root_m2*1e6:.2f} mm²")
    print(f"  Spar area (tip)      : {s.spar_area_tip_m2*1e6:.2f} mm²")
    print(f"  Number of ribs       : {s.N_ribs}")
    print(f"  Rib spacing          : {s.rib_spacing_m*1000:.2f} mm")
    print(f"  Bending moment       : {s.bending_moment_Nm:.1f} N·m ({s.bending_moment_type})")
    print(f"  Torsion moment       : {s.torsion_Nm:.1f} N·m")
    print()
    print("Mass breakdown (both vertical tail surfaces):")
    print(f"  Skin mass           : {s.W_skin_kg * 2:.3f} kg")
    print(f"  Web mass            : {s.W_web_kg * 2:.3f} kg")
    print(f"  Spars mass          : {s.W_spars_kg * 2:.3f} kg")
    print(f"  Ribs mass           : {s.W_ribs_kg * 2:.3f} kg")
    if getattr(vt, "vertical_tail_stiffener_mass_kg", 0.0) > 0:
        print(f"  Stiffeners mass     : {vt.vertical_tail_stiffener_mass_kg * 2:.3f} kg")
    print(f"  Total vertical tail mass: {s.W_total_kg * 2:.3f} kg")
    print()


def print_tail_results(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== TAILPLANE ===")
    t = results.tail
    g = t.geometry
    a = t.aerodynamics
    s = t.structure

    print("Geometry:")
    print(f"  Tail Lift_N         : {t.lift_N:.3f} N")
    print(f"  Tail area S         : {g.area_m2:.3f} m²")
    print(f"  Tail span b         : {g.span_m:.3f} m ({g.span_m*100:.1f} cm)")
    print(f"  Aspect ratio AR     : {g.AR:.3f}")
    print(f"  Root chord c_root   : {g.c_root_m:.3f} m ({g.c_root_m*100:.1f} cm)")
    print(f"  Tip chord c_tip     : {g.c_tip_m:.3f} m ({g.c_tip_m*100:.1f} cm)")
    print(f"  Mean Geometric Chord: {g.c_MGC_m:.3f} m ({g.c_MGC_m*100:.1f} cm)")
    print(f"  Mean Aerodynamic Chord: {g.MAC_m:.3f} m ({g.MAC_m*100:.1f} cm)")
    print(f"  MAC y-position      : {g.y_MAC_from_centerline_m:.3f} m from centerline")
    print(f"  AC on MAC           : {g.ac_on_MAC_m:.3f} m from LE")
    print(f"  AC x-position       : {t.x_ac_tail_m:.3f} m")
    print(f"  Wetted area (drag)  : {t.wetted_area_m2:.4f} m²")
    print(f"  V_h                 : {results.aircraft.V_h:.3f}")
    
    print("Aerodynamics & Trim:")
    print(f"  CL_cruise           : {a.CL_cruise:.3f}")
    print(f"  CL_max              : {a.CL_max:.3f}")
    print(f"  CD0                 : {a.CD0:.4f}")
    print(f"  CD_cruise           : {a.CD_cruise:.4f}")
    print(f"  L/D at cruise       : {a.L_over_D_cruise:.1f}")
    print(f"  Built-in incidence  : {t.tail_incidence_deg:.2f} deg")
    print(f"  Cruise elevator     : {t.cruise_elevator_deg:.2f} deg (limit ±{inputs.tail_elevator_limit_deg:.1f})")
    print(f"  Elevator chord ratio: {t.elevator_chord_ratio:.2f}")
    print(f"  Alpha body          : {t.alpha_body_deg:.2f} deg")
    print(f"  Alpha equivalent    : {t.alpha_equiv_deg:.2f} deg")
    print(f"  Re at cruise        : {a.Re_cruise:.0f}")
    print(f"  Airfoil max t/c     : {a.max_thickness_fraction_of_chord:.2f}")
    print()
    print("Elevator hinge moment & servo torque (at max deflection):")
    print(f"  M_h                 : {t.elevator_hinge_moment_M_h_Nm:.4f} N·m")
    print(f"  S_e (elevator area) : {t.elevator_area_S_e_m2:.6f} m²")
    print(f"  c_e (elevator chord): {t.elevator_chord_c_e_m:.4f} m")
    print(f"  T_servo (bare)      : {t.servo_torque_T_servo_Nm:.4f} N·m")
    print(f"  T_servo (w margin)  : {t.servo_torque_with_margin_Nm:.4f} N·m")
    print()
    print("Structure:")
    print(f"  Structural height h : {s.h_m*1000:.2f} mm")
    print(f"  Skin thickness (root): {s.t_skin_root_m*1000:.3f} mm")
    print(f"  Skin thickness (tip)  : {s.t_skin_tip_m*1000:.3f} mm")
    print(f"  Web thickness (root)  : {s.t_web_root_m*1000:.3f} mm")
    print(f"  Web thickness (tip)  : {s.t_web_tip_m*1000:.3f} mm")
    print(f"  Spar area (root)     : {s.spar_area_root_m2*1e6:.2f} mm²")
    print(f"  Spar area (tip)      : {s.spar_area_tip_m2*1e6:.2f} mm²")
    print(f"  Number of ribs       : {s.N_ribs}")
    print(f"  Rib spacing          : {s.rib_spacing_m*1000:.2f} mm")
    print(f"  Bending moment       : {s.bending_moment_Nm:.1f} N·m ({s.bending_moment_type})")
    print(f"  Torsion moment       : {s.torsion_Nm:.1f} N·m")
    print()
    print("Mass breakdown (both horizontal tail surfaces):")
    print(f"  Skin mass           : {s.W_skin_kg * 2:.3f} kg")
    print(f"  Web mass            : {s.W_web_kg * 2:.3f} kg")
    print(f"  Spars mass          : {s.W_spars_kg * 2:.3f} kg")
    print(f"  Ribs mass           : {s.W_ribs_kg * 2:.3f} kg")
    if getattr(t, "tail_stiffener_mass_kg", 0.0) > 0:
        print(f"  Stiffeners mass     : {t.tail_stiffener_mass_kg * 2:.3f} kg")
    print(f"  Total tail mass     : {s.W_total_kg * 2:.3f} kg")
    print()


def print_wing_spar_results(results: DesignResults) -> None:
    print("=== WING SPAR TUBE SIZING ===")
    w = results.wing
    
    if w.wing_spar_length_m <= 0:
        print("No wing spar sizing data available")
        print()
        return
    
    print(f"  L_spar_m                = {w.wing_spar_length_m:.4f} m")
    print(f"  M_max_Nm                = {w.wing_spar_M_max_Nm:.4f} N*m")
    print(f"  sigma_allow_MPa         = {w.wing_spar_sigma_allow_MPa:.1f} MPa")
    print(f"  sigma_root_MPa          = {w.wing_spar_sigma_root_MPa:.1f} MPa")
    print(f"  OD_root_m               = {w.wing_spar_OD_root_m:.5f} m  ({w.wing_spar_OD_root_m*1e3:.2f} mm)")
    print(f"  ID_root_m               = {w.wing_spar_ID_root_m:.5f} m  ({w.wing_spar_ID_root_m*1e3:.2f} mm)")
    print(f"  OD_tip_m                = {w.wing_spar_OD_tip_m:.5f} m  ({w.wing_spar_OD_tip_m*1e3:.2f} mm)")
    print(f"  ID_tip_m                = {w.wing_spar_ID_tip_m:.5f} m  ({w.wing_spar_ID_tip_m*1e3:.2f} mm)")
    print(f"  A_root_mm2              = {w.wing_spar_A_root_mm2:.2f} mm^2")
    print(f"  A_tip_mm2               = {w.wing_spar_A_tip_mm2:.2f} mm^2")
    print(f"  I_root_mm4              = {w.wing_spar_I_root_mm4:.2f} mm^4")
    print(f"  I_tip_mm4               = {w.wing_spar_I_tip_mm4:.2f} mm^4")
    print(f"  W_spar_kg (per surface) = {w.wing_spar_mass_kg:.4f} kg")
    print(f"  N_ribs                  = {w.wing_spar_N_ribs:.0f}")
    print(f"  rib_spacing_m           = {w.structure.rib_spacing_m:.4f} m  ({w.structure.rib_spacing_m*1e3:.2f} mm)")
    print(f"  rib_contact_L_m         = {w.wing_spar_rib_contact_L_m:.4f} m")
    print(f"  bearing_allow_MPa       = {w.wing_spar_bearing_allow_MPa:.1f} MPa")
    print(f"  bearing_stress_MPa      = {w.wing_spar_bearing_stress_MPa:.1f} MPa")
    print(f"  bearing_ok              = {w.wing_spar_bearing_ok:.0f}")
    print(f"  tip_deflection_m           = {w.wing_spar_tip_deflection_m:.4f} m")
    print(f"  tip_deflection_mm          = {w.wing_spar_tip_deflection_mm:.2f} mm")
    print(f"  root_slope_rad             = {w.wing_spar_root_slope_rad:.4f} rad")
    print(f"  root_slope_deg             = {w.wing_spar_root_slope_deg:.2f} deg")
    print()


def print_wing_stiffener_results(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== WING SKIN STIFFENERS (TORSION / SHEAR BUCKLING) ===")
    w = results.wing

    if getattr(w, "wing_stiffener_N", 0.0) <= 0 and getattr(w, "wing_stiffener_mass_kg", 0.0) <= 0:
        print("No wing stiffener data available")
        print()
        return

    print("Design inputs (shear buckling):")
    print(f"  stiffener_panel_buckling_margin = {getattr(inputs, 'stiffener_panel_buckling_margin', 0.5):.2f} [-]")
    print(f"  stiffener_panel_k_shear          = {getattr(inputs, 'stiffener_panel_k_shear', 5.5):.2f} [-]")
    print(f"  stiffener_t_min_factor          = {getattr(inputs, 'stiffener_t_min_factor', 1.0):.2f} [-]")
    print(f"  stiffener_t_max_factor          = {getattr(inputs, 'stiffener_t_max_factor', 2.0):.2f} [-]")
    print(f"  stiffener_h_min_factor          = {getattr(inputs, 'stiffener_h_min_factor', 2.0):.2f} [-]")
    print(f"  stiffener_h_max_factor          = {getattr(inputs, 'stiffener_h_max_factor', 4.0):.2f} [-]")
    print()
    print("Sizing results:")
    print(f"  N_stiffeners              = {w.wing_stiffener_N:.0f}")
    print(f"  stiffener_spacing_m       = {w.wing_stiffener_spacing_m:.4f} m  ({w.wing_stiffener_spacing_m*1e3:.2f} mm)")
    print(f"  stiffener_thickness_m     = {w.wing_stiffener_thickness_m:.5f} m  ({w.wing_stiffener_thickness_m*1e3:.2f} mm)")
    print(f"  stiffener_height_m        = {w.wing_stiffener_height_m:.5f} m  ({w.wing_stiffener_height_m*1e3:.2f} mm)")
    print(f"  stiffener_area_one_mm2    = {w.wing_stiffener_area_one_mm2:.2f} mm^2")
    print(f"  stiffener_I_one_mm4       = {w.wing_stiffener_I_one_mm4:.2f} mm^4")
    print(f"  tau_skin_MPa              = {w.wing_stiffener_tau_skin_MPa:.2f} MPa")
    print(f"  tau_cr_panel_MPa          = {w.wing_stiffener_tau_cr_panel_MPa:.2f} MPa")
    print(f"  q_shear_N_per_m            = {w.wing_stiffener_q_shear_N_per_m:.2f} N/m")
    print(f"  aspect_ratio              = {w.wing_stiffener_aspect_ratio:.2f} [-]")
    print(f"  W_stiffeners_kg           = {w.wing_stiffener_mass_kg:.4f} kg")
    print(f"  panel_ok                  = {w.wing_stiffener_panel_ok:.0f}")
    print()


def print_tail_spar_results(results: DesignResults) -> None:
    print("=== TAIL SPAR TUBE SIZING ===")
    t = results.tail

    if t.tail_spar_length_m <= 0:
        print("No tail spar sizing data available")
        print()
        return

    print(f"  L_spar_m                = {t.tail_spar_length_m:.4f} m")
    print(f"  M_max_Nm                = {t.tail_spar_M_max_Nm:.4f} N*m")
    print(f"  sigma_allow_MPa         = {t.tail_spar_sigma_allow_MPa:.1f} MPa")
    print(f"  sigma_root_MPa          = {t.tail_spar_sigma_root_MPa:.1f} MPa")
    print(f"  OD_root_m               = {t.tail_spar_OD_root_m:.5f} m  ({t.tail_spar_OD_root_m*1e3:.2f} mm)")
    print(f"  ID_root_m               = {t.tail_spar_ID_root_m:.5f} m  ({t.tail_spar_ID_root_m*1e3:.2f} mm)")
    print(f"  OD_tip_m                = {t.tail_spar_OD_tip_m:.5f} m  ({t.tail_spar_OD_tip_m*1e3:.2f} mm)")
    print(f"  ID_tip_m                = {t.tail_spar_ID_tip_m:.5f} m  ({t.tail_spar_ID_tip_m*1e3:.2f} mm)")
    print(f"  A_root_mm2              = {t.tail_spar_A_root_mm2:.2f} mm^2")
    print(f"  A_tip_mm2               = {t.tail_spar_A_tip_mm2:.2f} mm^2")
    print(f"  I_root_mm4              = {t.tail_spar_I_root_mm4:.2f} mm^4")
    print(f"  I_tip_mm4               = {t.tail_spar_I_tip_mm4:.2f} mm^4")
    print(f"  W_spar_kg (per surface) = {t.tail_spar_mass_kg:.4f} kg")
    print(f"  N_ribs                  = {t.tail_spar_N_ribs:.0f}")
    print(f"  rib_spacing_m           = {t.structure.rib_spacing_m:.4f} m  ({t.structure.rib_spacing_m*1e3:.2f} mm)")
    print(f"  rib_contact_L_m         = {t.tail_spar_rib_contact_L_m:.4f} m")
    print(f"  bearing_allow_MPa       = {t.tail_spar_bearing_allow_MPa:.1f} MPa")
    print(f"  bearing_stress_MPa      = {t.tail_spar_bearing_stress_MPa:.1f} MPa")
    print(f"  bearing_ok              = {t.tail_spar_bearing_ok:.0f}")
    print(f"  tip_deflection_m           = {t.tail_spar_tip_deflection_m:.4f} m")
    print(f"  tip_deflection_mm          = {t.tail_spar_tip_deflection_mm:.2f} mm")
    print(f"  root_slope_rad             = {t.tail_spar_root_slope_rad:.4f} rad")
    print(f"  root_slope_deg             = {t.tail_spar_root_slope_deg:.2f} deg")
    print()


def print_tail_stiffener_results(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== TAIL SKIN STIFFENERS (TORSION / SHEAR BUCKLING) ===")
    t = results.tail

    if getattr(t, "tail_stiffener_N", 0.0) <= 0 and getattr(t, "tail_stiffener_mass_kg", 0.0) <= 0:
        print("No tail stiffener data available")
        print()
        return

    print("Design inputs (shear buckling):")
    print(f"  stiffener_panel_buckling_margin = {getattr(inputs, 'stiffener_panel_buckling_margin', 0.5):.2f} [-]")
    print(f"  stiffener_panel_k_shear          = {getattr(inputs, 'stiffener_panel_k_shear', 5.5):.2f} [-]")
    print()
    print("Sizing results:")
    print(f"  N_stiffeners              = {t.tail_stiffener_N:.0f}")
    print(f"  stiffener_spacing_m       = {t.tail_stiffener_spacing_m:.4f} m  ({t.tail_stiffener_spacing_m*1e3:.2f} mm)")
    print(f"  stiffener_thickness_m     = {t.tail_stiffener_thickness_m:.5f} m  ({t.tail_stiffener_thickness_m*1e3:.2f} mm)")
    print(f"  stiffener_height_m        = {t.tail_stiffener_height_m:.5f} m  ({t.tail_stiffener_height_m*1e3:.2f} mm)")
    print(f"  stiffener_area_one_mm2    = {t.tail_stiffener_area_one_mm2:.2f} mm^2")
    print(f"  stiffener_I_one_mm4       = {t.tail_stiffener_I_one_mm4:.2f} mm^4")
    print(f"  tau_skin_MPa              = {t.tail_stiffener_tau_skin_MPa:.2f} MPa")
    print(f"  tau_cr_panel_MPa          = {t.tail_stiffener_tau_cr_panel_MPa:.2f} MPa")
    print(f"  q_shear_N_per_m            = {t.tail_stiffener_q_shear_N_per_m:.2f} N/m")
    print(f"  aspect_ratio              = {t.tail_stiffener_aspect_ratio:.2f} [-]")
    print(f"  W_stiffeners_kg           = {t.tail_stiffener_mass_kg:.4f} kg")
    print(f"  panel_ok                  = {t.tail_stiffener_panel_ok:.0f}")
    print()


def print_vertical_tail_spar_results(results: DesignResults) -> None:
    print("=== VERTICAL TAIL SPAR TUBE SIZING ===")
    vt = results.vertical_tail
    
    if vt.vertical_tail_spar_length_m <= 0:
        print("No vertical tail spar sizing data available")
        print()
        return
    
    print(f"  L_spar_m                = {vt.vertical_tail_spar_length_m:.4f} m")
    print(f"  M_max_Nm                = {vt.vertical_tail_spar_M_max_Nm:.4f} N*m")
    print(f"  sigma_allow_MPa         = {vt.vertical_tail_spar_sigma_allow_MPa:.1f} MPa")
    print(f"  sigma_root_MPa          = {vt.vertical_tail_spar_sigma_root_MPa:.1f} MPa")
    print(f"  OD_root_m               = {vt.vertical_tail_spar_OD_root_m:.5f} m  ({vt.vertical_tail_spar_OD_root_m*1e3:.2f} mm)")
    print(f"  ID_root_m               = {vt.vertical_tail_spar_ID_root_m:.5f} m  ({vt.vertical_tail_spar_ID_root_m*1e3:.2f} mm)")
    print(f"  OD_tip_m                = {vt.vertical_tail_spar_OD_tip_m:.5f} m  ({vt.vertical_tail_spar_OD_tip_m*1e3:.2f} mm)")
    print(f"  ID_tip_m                = {vt.vertical_tail_spar_ID_tip_m:.5f} m  ({vt.vertical_tail_spar_ID_tip_m*1e3:.2f} mm)")
    print(f"  A_root_mm2              = {vt.vertical_tail_spar_A_root_mm2:.2f} mm^2")
    print(f"  A_tip_mm2               = {vt.vertical_tail_spar_A_tip_mm2:.2f} mm^2")
    print(f"  I_root_mm4              = {vt.vertical_tail_spar_I_root_mm4:.2f} mm^4")
    print(f"  I_tip_mm4               = {vt.vertical_tail_spar_I_tip_mm4:.2f} mm^4")
    print(f"  W_spar_kg (per surface) = {vt.vertical_tail_spar_mass_kg:.4f} kg")
    print(f"  N_ribs                  = {vt.vertical_tail_spar_N_ribs:.0f}")
    print(f"  rib_spacing_m           = {vt.structure.rib_spacing_m:.4f} m  ({vt.structure.rib_spacing_m*1e3:.2f} mm)")
    print(f"  rib_contact_L_m         = {vt.vertical_tail_spar_rib_contact_L_m:.4f} m")
    print(f"  bearing_allow_MPa       = {vt.vertical_tail_spar_bearing_allow_MPa:.1f} MPa")
    print(f"  bearing_stress_MPa      = {vt.vertical_tail_spar_bearing_stress_MPa:.1f} MPa")
    print(f"  bearing_ok              = {vt.vertical_tail_spar_bearing_ok:.0f}")
    print(f"  tip_deflection_m           = {vt.vertical_tail_spar_tip_deflection_m:.4f} m")
    print(f"  tip_deflection_mm          = {vt.vertical_tail_spar_tip_deflection_mm:.2f} mm")
    print(f"  root_slope_rad             = {vt.vertical_tail_spar_root_slope_rad:.4f} rad")
    print(f"  root_slope_deg             = {vt.vertical_tail_spar_root_slope_deg:.2f} deg")
    print()


def print_vertical_tail_stiffener_results(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== VERTICAL TAIL SKIN STIFFENERS (TORSION / SHEAR BUCKLING) ===")
    vt = results.vertical_tail

    if getattr(vt, "vertical_tail_stiffener_N", 0.0) <= 0 and getattr(vt, "vertical_tail_stiffener_mass_kg", 0.0) <= 0:
        print("No vertical tail stiffener data available")
        print()
        return

    print("Design inputs (shear buckling):")
    print(f"  stiffener_panel_buckling_margin = {getattr(inputs, 'stiffener_panel_buckling_margin', 0.5):.2f} [-]")
    print(f"  stiffener_panel_k_shear          = {getattr(inputs, 'stiffener_panel_k_shear', 5.5):.2f} [-]")
    print()
    print("Sizing results:")
    print(f"  N_stiffeners              = {vt.vertical_tail_stiffener_N:.0f}")
    print(f"  stiffener_spacing_m       = {vt.vertical_tail_stiffener_spacing_m:.4f} m  ({vt.vertical_tail_stiffener_spacing_m*1e3:.2f} mm)")
    print(f"  stiffener_thickness_m     = {vt.vertical_tail_stiffener_thickness_m:.5f} m  ({vt.vertical_tail_stiffener_thickness_m*1e3:.2f} mm)")
    print(f"  stiffener_height_m        = {vt.vertical_tail_stiffener_height_m:.5f} m  ({vt.vertical_tail_stiffener_height_m*1e3:.2f} mm)")
    print(f"  stiffener_area_one_mm2    = {vt.vertical_tail_stiffener_area_one_mm2:.2f} mm^2")
    print(f"  stiffener_I_one_mm4       = {vt.vertical_tail_stiffener_I_one_mm4:.2f} mm^4")
    print(f"  tau_skin_MPa              = {vt.vertical_tail_stiffener_tau_skin_MPa:.2f} MPa")
    print(f"  tau_cr_panel_MPa          = {vt.vertical_tail_stiffener_tau_cr_panel_MPa:.2f} MPa")
    print(f"  q_shear_N_per_m            = {vt.vertical_tail_stiffener_q_shear_N_per_m:.2f} N/m")
    print(f"  aspect_ratio              = {vt.vertical_tail_stiffener_aspect_ratio:.2f} [-]")
    print(f"  W_stiffeners_kg           = {vt.vertical_tail_stiffener_mass_kg:.4f} kg")
    print(f"  panel_ok                  = {vt.vertical_tail_stiffener_panel_ok:.0f}")
    print()


def print_elevator_rod_results(results: DesignResults) -> None:
    print("=== ELEVATOR TORQUE TUBE / HINGE ROD SIZING ===")
    t = results.tail
    
    if t.elevator_rod_length_m <= 0:
        print("No elevator rod sizing data available")
        print()
        return
    
    print(f"  span_frac               = {t.elevator_rod_span_frac:.3f}")
    print(f"  L_rod_m                 = {t.elevator_rod_length_m:.4f} m")
    print(f"  F_elev_N                = {t.elevator_rod_F_elev_N:.2f} N")
    print(f"  M_max_rod_Nm            = {t.elevator_rod_M_max_Nm:.4f} N*m")
    print(f"  sigma_allow_MPa         = {t.elevator_rod_sigma_allow_MPa:.1f} MPa")
    print(f"  sigma_rod_MPa           = {t.elevator_rod_sigma_rod_MPa:.1f} MPa")
    print(f"  OD_m                    = {t.elevator_rod_OD_m:.5f} m  ({t.elevator_rod_OD_m*1e3:.2f} mm)")
    print(f"  ID_m                    = {t.elevator_rod_ID_m:.5f} m  ({t.elevator_rod_ID_m*1e3:.2f} mm)")
    print(f"  A_rod_mm2               = {t.elevator_rod_area_mm2:.2f} mm^2")
    print(f"  I_rod_mm4               = {t.elevator_rod_I_mm4:.2f} mm^4")
    print(f"  W_rod_kg (per surface)  = {t.elevator_rod_mass_kg:.4f} kg")
    print(f"  N_support               = {t.elevator_rod_N_support:.0f}")
    print(f"  rod_contact_L_m         = {t.elevator_rod_contact_L_m:.4f} m")
    print(f"  rod_bearing_allow_MPa    = {t.elevator_rod_bearing_allow_MPa:.1f} MPa")
    print()


def print_fuselage_results(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== CENTRAL POD AND TWIN BOOMS ===")
    f = results.fuselage
    b = results.booms

    pod_inner_w = max(0.0, inputs.central_pod_width_m - 2.0 * f.t_shell_m)
    pod_inner_h = max(0.0, inputs.central_pod_height_m - 2.0 * f.t_shell_m)

    print("Central pod geometry:")
    print(f"  Length              : {inputs.central_pod_length_m:.3f} m")
    print(f"  External width      : {inputs.central_pod_width_m:.3f} m")
    print(f"  External height     : {inputs.central_pod_height_m:.3f} m")
    print(f"  Shell thickness     : {f.t_shell_m*1000:.3f} mm")
    print(f"  Approx. inner width : {pod_inner_w:.3f} m")
    print(f"  Approx. inner height: {pod_inner_h:.3f} m")
    print(f"  Pod + boom wetted area used in drag: {f.wetted_area_m2:.4f} m²")
    print()

    print("Central pod structure:")
    print(f"  Total pod mass      : {f.W_fuselage_kg:.3f} kg")
    print(f"  Printed shell       : {getattr(f, 'W_shell_kg', 0.0):.3f} kg")
    print(f"  Frames              : {getattr(f, 'W_frames_total_kg', 0.0):.3f} kg")
    print(f"  Wing carry-through  : {getattr(f, 'W_longerons_total_kg', 0.0):.3f} kg")
    print(f"  Number of frames    : {getattr(f, 'N_frames', 0.0):.0f}")
    print(f"  Frame spacing       : {getattr(f, 'frame_spacing_m', 0.0)*1000:.1f} mm")
    print(f"  Shell bending stress: {getattr(f, 'sigma_x_MPa', 0.0):.2f} MPa")
    print(f"  Shell shear stress  : {getattr(f, 'tau_xy_MPa', 0.0):.2f} MPa")
    print(f"  Bending check       : {'PASS' if getattr(f, 'panel_comp_ok', 0.0) > 0.5 else 'REVIEW'}")
    print(f"  Shear check         : {'PASS' if getattr(f, 'panel_shear_ok', 0.0) > 0.5 else 'REVIEW'}")
    print()

    print("Twin-boom structure:")
    print(f"  Number of booms     : {inputs.n_booms}")
    print(f"  Boom length         : {b.length_m:.3f} m")
    print(f"  Sized OD / ID       : {b.outer_diameter_m*1000:.1f} / {b.inner_diameter_m*1000:.1f} mm")
    print(f"  Tube mass           : {b.tube_mass_kg:.3f} kg")
    print(f"  Mount mass          : {b.mount_mass_kg:.3f} kg")
    print(f"  Tail crossmember    : {b.tail_crossmember_mass_kg:.3f} kg")
    print(f"  Total boom system   : {b.total_mass_kg:.3f} kg")
    print(f"  Cruise M/root boom  : {b.bending_moment_cruise_Nm_per_boom:.2f} N·m")
    print(f"  VTOL M/root boom    : {b.bending_moment_vtol_Nm_per_boom:.2f} N·m")
    print(f"  Design M/root boom  : {b.bending_moment_design_Nm_per_boom:.2f} N·m")
    print(f"  Combined stress     : {b.combined_stress_MPa:.2f} MPa")
    print(f"  Allowable stress    : {b.allowable_stress_MPa:.2f} MPa")
    print(f"  Strength check      : {'PASS' if b.strength_ok else 'FAIL'}")
    print(f"  Tip deflection      : {b.tip_deflection_mm:.2f} mm")
    print()
    print("Landing gear: not fitted; no landing-gear mass is included.")
    print()


def print_propulsion_results(results: DesignResults) -> None:
    print("=== PROPULSION — SEPARATE LIFT + CRUISE ===")
    a = results.aircraft
    print(f"drag_cruise_N       : {a.drag_cruise_N:.3f} N")
    print(f"cruise_power_W      : {a.cruise_power_W:.3f} W")
    print(f"cruise_power_elec_W : {a.cruise_power_elec_W:.3f} W")
    print()

    if results.propulsion is None:
        print("No propulsion system selected")
        print()
        return

    p = results.propulsion
    if getattr(p, "architecture", "") == "separate_lift_and_cruise":
        print("Lift propulsion:")
        print(f"  Number of units         : {p.n_lift_propellers}")
        print(f"  Combination             : {p.lift_name}")
        print(f"  Propeller               : {p.lift_prop_diameter_in:.1f} x {p.lift_prop_pitch_in:.1f} in")
        print(f"  Motor Kv                : {p.lift_motor_kv:.0f} rpm/V")
        print(f"  Max thrust per motor    : {p.lift_max_thrust_per_motor_N:.2f} N")
        print(f"  Required thrust/motor   : {p.lift_required_thrust_per_motor_N:.2f} N")
        print(f"  Max power per motor     : {p.lift_max_power_per_motor_W:.1f} W")
        print(f"  Required power/motor    : {p.lift_required_power_per_motor_W:.1f} W")
        print(f"  ESC sizing/motor        : {p.lift_esc_sizing_power_per_motor_W:.1f} W, {p.lift_esc_cont_current_A_per_motor:.1f} A")
        print(f"  Meets thrust / power    : {p.lift_meets_thrust_req} / {p.lift_meets_power_req}")
        print()
        print("Cruise propulsion:")
        print(f"  Number of units         : {p.n_cruise_propellers}")
        print(f"  Combination             : {p.cruise_name}")
        print(f"  Propeller               : {p.cruise_prop_diameter_in:.1f} x {p.cruise_prop_pitch_in:.1f} in")
        print(f"  Motor Kv                : {p.cruise_motor_kv:.0f} rpm/V")
        print(f"  Max thrust per motor    : {p.cruise_max_thrust_per_motor_N:.2f} N")
        print(f"  Required thrust/motor   : {p.cruise_required_thrust_per_motor_N:.2f} N")
        print(f"  Max power per motor     : {p.cruise_max_power_per_motor_W:.1f} W")
        print(f"  Required power/motor    : {p.cruise_required_power_per_motor_W:.1f} W")
        print(f"  ESC sizing/motor        : {p.cruise_esc_sizing_power_per_motor_W:.1f} W, {p.cruise_esc_cont_current_A_per_motor:.1f} A")
        print(f"  Meets thrust / power    : {p.cruise_meets_thrust_req} / {p.cruise_meets_power_req}")
        print()
        print(f"Total motor mass          : {p.total_motor_mass_kg:.3f} kg")
        print(f"Total propeller mass      : {p.total_prop_mass_kg:.3f} kg")
        print(f"Total propulsion mass     : {p.total_propulsion_mass_kg:.3f} kg")
    else:
        print(f"n_propellers        : {p.n_propellers}")
        print(f"combo               : {p.name}")
        print(f"prop_diameter       : {p.prop_diameter_in:.1f} in")
        print(f"required_power/motor: {p.required_power_per_motor_W:.1f} W")
        print(f"Total propulsion    : {p.total_propulsion_mass_kg:.3f} kg")
    print()

def print_battery_results(results: DesignResults) -> None:
    print("=== HYBRID ENERGY SYSTEM ===")
    if results.battery is None:
        print("No hybrid energy system selected")
        print()
        return

    b = results.battery
    print("Buffer battery:")
    print(f"  Name                    : {b.name}")
    print(f"  Voltage / capacity      : {b.voltage_V:.1f} V / {b.capacity_mAh:.0f} mAh")
    print(f"  Nominal / usable energy : {getattr(b, 'nominal_energy_Wh', 0.0):.1f} / {b.usable_energy_Wh:.1f} Wh")
    print(f"  Initial / minimum SOC   : {getattr(b, 'initial_soc', 0.0):.1%} / {getattr(b, 'minimum_soc', 0.0):.1%}")
    print(f"  Final mission SOC       : {getattr(b, 'final_soc', 0.0):.1%}")
    print(f"  Peak battery power      : {getattr(b, 'peak_power_W', 0.0):.1f} W ({getattr(b, 'peak_C', 0.0):.1f} C)")
    print(f"  Battery mass            : {b.mass_kg:.3f} kg")
    print()
    print("Fuel cell:")
    print(f"  Name                    : {getattr(b, 'fuel_cell_name', '')}")
    print(f"  Rated net power         : {getattr(b, 'fuel_cell_rated_power_W', 0.0):.1f} W")
    print(f"  Max power at DC bus     : {getattr(b, 'fuel_cell_max_bus_power_W', 0.0):.1f} W")
    print(f"  Usable electrical energy: {getattr(b, 'fuel_cell_usable_energy_Wh', 0.0):.1f} Wh")
    print(f"  Fuel-cell system mass   : {getattr(b, 'fuel_cell_system_mass_kg', 0.0):.3f} kg")
    print()
    print("Hydrogen storage:")
    print(f"  System                  : {getattr(b, 'hydrogen_storage_name', '')}")
    print(f"  Pressure                : {getattr(b, 'hydrogen_pressure_bar', 0.0):.0f} bar")
    print(f"  Nominal / usable H2     : {getattr(b, 'hydrogen_nominal_mass_kg', 0.0)*1000:.1f} / {getattr(b, 'hydrogen_usable_mass_kg', 0.0)*1000:.1f} g")
    print(f"  H2 used in mission      : {getattr(b, 'hydrogen_used_kg', 0.0)*1000:.1f} g")
    print(f"  Tank + regulator mass   : {getattr(b, 'hydrogen_tank_mass_kg', 0.0) + getattr(b, 'hydrogen_regulator_mass_kg', 0.0):.3f} kg")
    print()
    print(f"Total energy-system mass  : {getattr(b, 'total_energy_system_mass_kg', b.mass_kg):.3f} kg")
    print(f"Maximum cruise endurance  : {b.endurance_min_at_cruise:.1f} min")
    print(f"Meets requested endurance : {b.meets_endurance_req}")
    print()

def print_avionics_results(results: DesignResults) -> None:
    print("=== AVIONICS / ELECTRONICS ===")
    if results.avionics is None:
        print("No avionics data available")
        print()
        return
    
    a = results.avionics
    print(f"Autopilot          : {a.autopilot_name} ({a.autopilot_mass_kg:.3f} kg)")
    print(f"GPS                : {a.gps_mass_kg:.3f} kg")
    print(f"Airspeed sensor    : {a.airspeed_mass_kg:.3f} kg")
    print(f"RC Receiver        : {a.rc_rx_mass_kg:.3f} kg")
    print(f"Telemetry          : {a.telemetry_mass_kg:.3f} kg")
    print(f"Power module       : {a.power_module_mass_kg:.3f} kg")
    print(f"BEC                : {a.bec_mass_kg:.3f} kg")
    print(f"Autopilot powerbrick: {a.autopilot_powerbrick_mass_kg:.3f} kg")
    print(f"Servos             : {a.servos_mass_kg:.3f} kg")
    print(f"ESCs (total)       : {a.esc_mass_total_kg:.3f} kg")
    print(f"Misc electronics   : {a.misc_electronics_mass_kg:.3f} kg")
    print(f"Camera             : {a.camera_mass_kg:.3f} kg")
    print(f"TOTAL avionics+elec: {a.total_avionics_electrics_mass_kg:.3f} kg")
    print()


def print_mass_properties(results: DesignResults) -> None:
    print("=== MASS PROPERTIES & LONGITUDINAL CG ===")
    mp = getattr(results, "mass_properties", None)
    if mp is None:
        print("Detailed mass-properties data are not available.")
        print()
        return

    print(f"Take-off mass                  : {mp.takeoff_mass_kg:.3f} kg")
    print(f"End-of-mission mass            : {mp.end_mass_kg:.3f} kg")
    print(f"Take-off CG                    : {mp.x_cg_takeoff_m:.4f} m")
    print(f"End-of-mission CG              : {mp.x_cg_end_m:.4f} m")
    print(f"CG shift (end - take-off)      : {mp.cg_shift_m*1000.0:+.2f} mm")
    print(f"Stored H2, take-off / end      : {mp.hydrogen_takeoff_kg*1000.0:.1f} / {mp.hydrogen_end_kg*1000.0:.1f} g")
    print()
    print(f"{'Component':30s} {'m_TO[kg]':>10s} {'m_end[kg]':>10s} {'x[m]':>8s} {'M_TO[kgm]':>11s}")
    for name, item in mp.items.items():
        if item.get("mass_takeoff_kg", 0.0) <= 1e-9 and item.get("mass_end_kg", 0.0) <= 1e-9:
            continue
        print(
            f"{name[:30]:30s} "
            f"{item['mass_takeoff_kg']:10.4f} {item['mass_end_kg']:10.4f} "
            f"{item['x_m']:8.3f} {item['moment_takeoff_kgm']:11.4f}"
        )
    print()


def _print_cg_location_line(label: str, value_m: float) -> None:
    print(f"{label:40s}: {value_m:.3f} m")


def print_cg_summary(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== CG SUMMARY ===")

    if results.cg_locations is not None:
        cg = results.cg_locations
    else:
        from results import build_cg_locations

        cg = build_cg_locations(
            inputs,
            x_cg_m=results.aircraft.x_cg_m,
            x_ac_wing_m=results.wing.x_ac_wing_m,
            x_ac_tail_m=results.tail.x_ac_tail_m,
            x_ac_vertical_tail_m=results.vertical_tail.x_ac_tail_m,
            x_np_m=results.aircraft.x_np_m,
        )

    print("Aircraft CG (computed):")
    _print_cg_location_line("x_cg_m (take-off)", cg.x_cg_m)
    if getattr(results, "mass_properties", None) is not None:
        _print_cg_location_line("x_cg_end_m", results.mass_properties.x_cg_end_m)
        print(f"{'CG shift end - take-off':40s}: {results.mass_properties.cg_shift_m*1000.0:+.2f} mm")
    print(f"{'x_cg/fuselage_length':40s}: {cg.x_cg_over_fuselage_length:.3f}")
    print()

    print("Aerodynamic reference points:")
    _print_cg_location_line("x_ac_wing_m", cg.x_ac_wing_m)
    _print_cg_location_line("x_ac_tail_m", cg.x_ac_tail_m)
    _print_cg_location_line("x_ac_vertical_tail_m", cg.x_ac_vertical_tail_m)
    _print_cg_location_line("x_np_m (neutral point)", cg.x_np_m)
    print()

    print("Component mass CG x-positions (design_inputs):")
    _print_cg_location_line("cg_x_wing", cg.cg_x_wing)
    _print_cg_location_line("cg_x_tail", cg.cg_x_tail)
    _print_cg_location_line("cg_x_vertical_tail", cg.cg_x_vertical_tail)
    _print_cg_location_line("cg_x_battery", cg.cg_x_battery)
    _print_cg_location_line("cg_x_nose", cg.cg_x_nose)
    _print_cg_location_line("cg_x_body_section_C", cg.cg_x_body_section_C)
    _print_cg_location_line("cg_x_body_section_B", cg.cg_x_body_section_B)
    _print_cg_location_line("cg_x_body_section_A", cg.cg_x_body_section_A)
    _print_cg_location_line("cg_x_fuselage_tail", cg.cg_x_fuselage_tail)
    _print_cg_location_line("cg_x_propulsion (legacy only)", cg.cg_x_propulsion)
    _print_cg_location_line("cg_x_fuel_cell", cg.cg_x_fuel_cell)
    _print_cg_location_line("cg_x_hydrogen_tank", cg.cg_x_hydrogen_tank)
    _print_cg_location_line("cg_x_hydrogen_regulator", cg.cg_x_hydrogen_regulator)
    _print_cg_location_line("cg_x_lift_propulsion_front", cg.cg_x_lift_propulsion_front)
    _print_cg_location_line("cg_x_lift_propulsion_rear", cg.cg_x_lift_propulsion_rear)
    _print_cg_location_line("cg_x_cruise_propulsion", cg.cg_x_cruise_propulsion)
    _print_cg_location_line("cg_x_lift_esc_front", cg.cg_x_lift_esc_front)
    _print_cg_location_line("cg_x_lift_esc_rear", cg.cg_x_lift_esc_rear)
    _print_cg_location_line("cg_x_cruise_esc", cg.cg_x_cruise_esc)
    _print_cg_location_line("cg_x_payload", cg.cg_x_payload)
    _print_cg_location_line("cg_x_autopilot", cg.cg_x_autopilot)
    _print_cg_location_line("cg_x_power_module", cg.cg_x_power_module)
    _print_cg_location_line("cg_x_bec", cg.cg_x_bec)
    _print_cg_location_line("cg_x_autopilot_powerbrick", cg.cg_x_autopilot_powerbrick)
    _print_cg_location_line("cg_x_esc", cg.cg_x_esc)
    _print_cg_location_line("cg_x_gps", cg.cg_x_gps)
    _print_cg_location_line("cg_x_airspeed", cg.cg_x_airspeed)
    _print_cg_location_line("cg_x_rx", cg.cg_x_rx)
    _print_cg_location_line("cg_x_tel", cg.cg_x_tel)
    _print_cg_location_line("cg_x_camera", cg.cg_x_camera)
    _print_cg_location_line("cg_x_servos", cg.cg_x_servos)
    _print_cg_location_line("cg_x_misc", cg.cg_x_misc)
    print()

    print("Derived geometry CG x-positions (chord fractions / LE):")
    _print_cg_location_line("wing_le_x_m", cg.wing_le_x_m)
    _print_cg_location_line("tail_le_x_m", cg.tail_le_x_m)
    _print_cg_location_line("vertical_tail_le_x_m", cg.vertical_tail_le_x_m)
    _print_cg_location_line("x_primary_wing_spar_m (0.3 c)", cg.x_primary_wing_spar_m)
    _print_cg_location_line("x_secondary_wing_spar_m (0.7 c)", cg.x_secondary_wing_spar_m)
    _print_cg_location_line("x_primary_tail_spar_m (0.3 c)", cg.x_primary_tail_spar_m)
    _print_cg_location_line("x_secondary_tail_spar_m (0.7 c, elevator rod)", cg.x_secondary_tail_spar_m)
    _print_cg_location_line("x_primary_vertical_tail_spar_m (0.3 c)", cg.x_primary_vertical_tail_spar_m)
    _print_cg_location_line("x_secondary_vertical_tail_spar_m (0.7 c, rudder rod)", cg.x_secondary_vertical_tail_spar_m)
    _print_cg_location_line("x_longerons_center_m", cg.x_longerons_center_m)
    print()


def print_mission_profile(results: DesignResults) -> None:
    print("=== FLIGHT PROFILE / HYBRID POWER ALLOCATION ===")

    profile = None
    if results.propulsion is not None and hasattr(results.propulsion, "mission_profile"):
        profile = results.propulsion.mission_profile
    elif results.battery is not None and hasattr(results.battery, "mission_profile"):
        profile = results.battery.mission_profile

    if profile is None:
        print("No mission profile data found.")
        print()
        return

    print(f"Architecture                    : {profile.get('architecture', '')}")
    print(f"Handoff speed                  : {profile['handoff_speed_mps']:.2f} m/s")
    print(f"Requested / max cruise         : {profile['cruise_time_requested_min']:.1f} / {profile['cruise_time_max_min']:.1f} min")
    print(f"Cruise flown in reported mission: {profile.get('cruise_time_flown_min', profile['cruise_time_max_min']):.1f} min")
    print(f"Meets requested endurance      : {profile.get('meets_requested_endurance', False)}")
    print(f"Total mission time / energy    : {profile['total_time_min']:.1f} min / {profile['total_energy_Wh']:.1f} Wh")
    print(f"Peak electrical power          : {profile['peak_elec_power_W']:.1f} W")
    print(f"Peak lift / cruise motor power : {profile.get('peak_lift_power_per_motor_W', 0.0):.1f} / {profile.get('peak_cruise_power_per_motor_W', 0.0):.1f} W")
    print(f"Battery final SOC / peak C     : {profile.get('battery_final_soc', 0.0):.1%} / {profile.get('battery_peak_C', 0.0):.1f} C")
    print(f"Fuel-cell energy used          : {profile.get('fuel_cell_energy_used_Wh', 0.0):.1f} Wh")
    print(f"Hydrogen used                  : {profile.get('hydrogen_used_kg', 0.0)*1000:.1f} g")
    print()

    print("Stage breakdown:")
    print(f"{'Stage':31s} {'t[s]':>7s} {'Load[W]':>9s} {'FC[W]':>8s} {'Batt[W]':>9s} {'E[Wh]':>8s} {'SOC end':>8s}")
    for s in profile["stages"]:
        print(
            f"{s['stage'][:31]:31s} {s['time_s']:7.1f} {s['avg_elec_power_W']:9.1f} "
            f"{s.get('fuel_cell_power_W', 0.0):8.1f} {s.get('battery_power_W', 0.0):9.1f} "
            f"{s.get('energy_Wh', 0.0):8.2f} {s.get('battery_soc_end', 0.0):8.1%}"
        )
    print()

def print_static_stability(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== STATIC STABILITY (LONGITUDINAL) ===")
    print(f"Neutral point x_np_m        : {results.aircraft.x_np_m:.3f} m")
    print(f"Static margin               : {results.aircraft.static_margin:.4f} [-]")
    print(f"Static margin (%MAC)        : {results.aircraft.static_margin_percent_MAC:.2f} %")
    # Optional: also show SM as fraction of fuselage length
    print(f"SM * MAC (m)                : {(results.aircraft.static_margin * results.wing.geometry.MAC_m):.4f} m")
    print()

def print_directional_stability(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== Directional Stability ===")
    print(f"Cn_beta                     : {results.aircraft.Cn_beta:.4f} [-]")
    print(f"V_v                         : {results.aircraft.V_v:.4f} [-]")
    print()


def print_maneuver_results(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== ELEVATOR CONTROL & MANEUVER CAPABILITY ===")
    if results.maneuver is None:
        print("No maneuver analysis performed")
        print()
        return
    
    m = results.maneuver
    
    print("Analysis Conditions:")
    print(f"  Maneuver speed V_man      : {m.V_maneuver_mps:.2f} m/s")
    print()
    
    print("Elevator Control Derivatives:")
    print(f"  Tail lift curve slope     : {m.CL_alpha_h_per_rad:.3f} /rad")
    print(f"  Elevator effectiveness τ_e: {m.tau_e:.4f} [-]")
    print(f"  Elevator span ratio bE/bh (geom): {m.bE_over_bh_geom:.3f} [-]")
    print(f"  Elevator span ratio bE/bh (eff) : {m.bE_over_bh_eff:.3f} [-]")
    print(f"  Tail efficiency η_h       : {m.eta_h:.3f} [-]")
    print(f"  Tail moment arm l_h       : {m.l_h_m:.3f} m")
    print(f"  Tail volume ratio V_H     : {m.V_H:.3f} [-]")
    print(f"  Cm_deltaE                 : {m.Cm_deltaE_per_rad:.4f} /rad")
    print(f"  CL_deltaE                 : {m.CL_deltaE_per_rad:.4f} /rad")
    print(f"  CLh_deltaE                : {m.CLh_deltaE_per_rad:.4f} /rad")
    print()
    
    print("Maximum Load Factor (Elevator-Limited):")
    if m.feasible:
        print(f"  Maximum load factor n_max : {m.n_max:.3f} [-]")
        if m.phi_max_deg_level_turn is not None:
            print(f"  Max bank angle (level turn): {m.phi_max_deg_level_turn:.2f} deg")
        print()
        print("  At maximum load factor:")
        if m.alpha_w_deg is not None:
            print(f"    Wing angle of attack     : {m.alpha_w_deg:.2f} deg")
        if m.CL_w is not None:
            print(f"    Wing lift coefficient   : {m.CL_w:.3f} [-]")
        if m.L_w_N is not None:
            print(f"    Wing lift               : {m.L_w_N:.1f} N")
        if m.M_ac_w_Nm is not None:
            print(f"    Wing moment @ AC        : {m.M_ac_w_Nm:.2f} N·m")
        if m.M_cg_w_Nm is not None:
            print(f"    Wing moment @ CG        : {m.M_cg_w_Nm:.2f} N·m")
        if m.L_tail_req_N is not None:
            print(f"    Required tail lift      : {m.L_tail_req_N:.1f} N")
        if m.CL_tail_req is not None:
            print(f"    Required tail CL        : {m.CL_tail_req:.3f} [-]")
        if m.delta_e_deg is not None:
            print(f"    Elevator deflection     : {m.delta_e_deg:.2f} deg")
        if m.downwash_deg is not None:
            print(f"    Downwash                : {m.downwash_deg:.2f} deg")
        if m.alpha_tail_geom_deg is not None:
            print(f"    Tail geometric AoA      : {m.alpha_tail_geom_deg:.2f} deg")
        if m.alpha_stall_eff_tail_deg is not None:
            print(f"    Effective tail stall AoA: {m.alpha_stall_eff_tail_deg:.2f} deg")
    else:
        print(f"  Maneuver NOT feasible")
        if m.reason:
            print(f"  Reason                    : {m.reason}")
        if m.n_max is not None and m.n_max > 0:
            print(f"  Last feasible n           : {m.n_max:.3f} [-]")
    print()


def print_rudder_results(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== RUDDER CONTROL & DIRECTIONAL CAPABILITY ===")
    if results.rudder is None:
        print("No rudder analysis performed")
        print()
        return
    
    r = results.rudder
    
    print("Analysis Conditions:")
    print(f"  Rudder analysis speed V_rud: {r.V_rudder_mps:.2f} m/s")
    print()
    
    print("Rudder Control Derivatives (Clean Flow):")
    print(f"  VT lift curve slope a_v    : {r.a_v_per_rad:.3f} /rad")
    print(f"  VT moment arm l_v          : {r.l_v_m:.3f} m")
    print(f"  Rudder chord ratio cR/cV   : {r.cR_over_cV:.3f} [-]")
    print(f"  Rudder effectiveness τ_R   : {r.tau_r:.4f} [-]")
    print(f"  Rudder span ratio bR/bV (geom): {r.bR_over_bV_geom:.3f} [-]")
    print(f"  Rudder span ratio bR/bV (eff) : {r.bR_over_bV_eff:.3f} [-]")
    print(f"  VT efficiency η_v          : {r.eta_v:.3f} [-]")
    print(f"  Sidewash gradient dσ/dβ   : {r.dsigma_dbeta:.4f} [-]")
    print(f"  Effective VT area S_v_eff : {r.S_v_eff_m2:.4f} m²")
    print(f"  Effective VT volume V_Ve   : {r.V_Ve:.4f} [-]")
    print(f"  Cn_deltaR                  : {r.Cn_deltaR_per_rad:.4f} /rad")
    print(f"  CY_deltaR                  : {r.CY_deltaR_per_rad:.4f} /rad")
    print(f"  CY_beta_v                  : {r.CY_beta_v_per_rad:.4f} /rad")
    print()
    
    print("Asymmetric Thrust Capability:")
    print(f"  Engine lateral position    : {r.y_engine_m:.3f} m")
    print(f"  Test thrust asymmetry      : {r.thrust_asym_N:.2f} N")
    print(f"  Required yaw moment         : {r.N_req_asym_Nm:.2f} N·m")
    print(f"  Required rudder deflection  : {r.deltaR_req_asym_deg:.2f} deg")
    print(f"  Max trimmable asymmetry     : {r.T_asym_max_N:.2f} N")
    if r.feasible_asym:
        print(f"  Status                     : FEASIBLE")
    else:
        print(f"  Status                     : NOT FEASIBLE")
        if r.reason_asym:
            print(f"  Reason                     : {r.reason_asym}")
    print()
    
    print("Spin Recovery Capability:")
    print(f"  Spin speed V_spin          : {r.V_spin_mps:.2f} m/s")
    print(f"  Aircraft mass               : {r.mass_kg:.3f} kg")
    print(f"  Yaw moment of inertia Izz  : {r.Izz_kgm2:.4f} kg·m²")
    print(f"  Spin rate                   : {r.spin_rate_rev_per_s:.2f} rev/s")
    print(f"  Initial yaw rate r0        : {r.r0_radps:.2f} rad/s")
    print(f"  Recovery time target       : {r.t_recover_s:.2f} s")
    print(f"  Required yaw decel rdot_req: {r.rdot_req_radps2:.2f} rad/s²")
    print(f"  Required yaw moment         : {r.N_req_spin_Nm:.2f} N·m")
    print(f"  Required rudder deflection  : {r.deltaR_req_spin_deg:.2f} deg")
    print(f"  Max achievable rdot        : {r.rdot_max_radps2:.2f} rad/s²")
    print(f"  Best-case recovery time     : {r.t_stop_best_s:.2f} s")
    if r.feasible_spin:
        print(f"  Status                     : FEASIBLE")
    else:
        print(f"  Status                     : NOT FEASIBLE")
        if r.reason_spin:
            print(f"  Reason                     : {r.reason_spin}")
    print()
    
    print("Coordinated Turn Characteristics:")
    if r.Cn_beta_per_rad is not None:
        print(f"  Directional stability Cn_beta: {r.Cn_beta_per_rad:.4f} /rad")
        if r.beta_trim_max_abs_deg is not None:
            print(f"  Max trimmable sideslip      : {r.beta_trim_max_abs_deg:.2f} deg")
        if r.N_max_abs_Nm is not None:
            print(f"  Max yaw moment authority     : {r.N_max_abs_Nm:.2f} N·m")
        if r.deltaR_max_abs_deg is not None:
            print(f"  Max rudder deflection        : {r.deltaR_max_abs_deg:.2f} deg")
    else:
        print(f"  Directional stability Cn_beta: Not available")
    print()


def print_drag_breakdown(results: DesignResults, inputs: DesignInputs) -> None:
    print("=== CRUISE DRAG BUILD-UP ===")
    db = results.drag_breakdown
    if db is None:
        print("No configuration drag breakdown is available.")
        print()
        return

    print(f"Cruise speed                : {inputs.speed:.2f} m/s")
    print(f"Dynamic pressure            : {db.dynamic_pressure_Pa:.1f} Pa")
    print(f"Aircraft CD0 (wing ref.)    : {db.CD0_aircraft:.4f}")
    print(f"Aircraft CD at cruise       : {db.CD_total_ref:.4f}")
    print(f"Total cruise drag           : {db.total_N:.2f} N")
    print(f"Overall L/D (weight/drag)   : {results.aircraft.W0_weight_N/max(db.total_N, 1e-9):.1f}")
    print()
    print("Component drag:")
    rows = [
        ("Main wing", db.wing_N),
        ("Horizontal tailplane", db.horizontal_tail_N),
        ("Two vertical fins", db.vertical_tails_N),
        ("Central pod", db.central_pod_N),
        ("Twin booms", db.twin_booms_N),
        ("Lift-motor nacelles", db.lift_motor_nacelles_N),
        ("Lift-motor supports", db.lift_motor_supports_N),
        ("Stopped lift propellers", db.stopped_lift_propellers_N),
        ("Fuel-cell cooling", db.fuel_cell_cooling_N),
        ("Junction interference", db.junction_interference_N),
        ("Miscellaneous", db.miscellaneous_N),
    ]
    for name, value in rows:
        pct = 100.0 * value / max(db.total_N, 1e-9)
        print(f"  {name:<27}: {value:7.3f} N  ({pct:5.1f}%)")
    print()
    print(f"Induced drag total          : {db.induced_total_N:.3f} N")
    print(f"Parasite drag total         : {db.parasite_total_N:.3f} N")
    print(f"Pod wetted area             : {db.pod_wetted_area_m2:.4f} m²")
    print(f"Boom wetted area            : {db.booms_wetted_area_m2:.4f} m²")
    print(f"Lift-prop drag area each    : {db.lift_prop_drag_area_each_m2:.5f} m²")
    print(f"Lift-prop cruise state      : {inputs.lift_prop_cruise_state}")
    print()


def print_all_results(results: DesignResults, inputs: DesignInputs) -> None:
    """Print all results sections with error handling to ensure all sections print."""
    sections = [
        ("Design Summary", lambda: print_design_summary(results, inputs)),
        ("Cruise Drag Build-up", lambda: print_drag_breakdown(results, inputs)),
        ("Mass Properties", lambda: print_mass_properties(results)),
        ("Wing Results", lambda: print_wing_results(results)),
        ("Wing Spar Results", lambda: print_wing_spar_results(results)),
        ("Wing Stiffener Results", lambda: print_wing_stiffener_results(results, inputs)),
        ("Tail Results", lambda: print_tail_results(results, inputs)),
        ("Tail Spar Results", lambda: print_tail_spar_results(results)),
        ("Tail Stiffener Results", lambda: print_tail_stiffener_results(results, inputs)),
        ("Elevator Rod Results", lambda: print_elevator_rod_results(results)),
        # ("Vertical Wing Results", lambda: print_vertical_wing_results(results)),
        ("Vertical Tail Results", lambda: print_vertical_tail_results(results)),
        ("Vertical Tail Spar Results", lambda: print_vertical_tail_spar_results(results)),
        ("Vertical Tail Stiffener Results", lambda: print_vertical_tail_stiffener_results(results, inputs)),
        ("Central Pod and Twin Booms", lambda: print_fuselage_results(results, inputs)),
        ("Propulsion Results", lambda: print_propulsion_results(results)),
        ("Battery Results", lambda: print_battery_results(results)),
        ("Avionics Results", lambda: print_avionics_results(results)),
        ("CG Summary", lambda: print_cg_summary(results, inputs)),
        ("Static Stability", lambda: print_static_stability(results, inputs)),
        ("Directional Stability", lambda: print_directional_stability(results, inputs)),
        ("Maneuver Results", lambda: print_maneuver_results(results, inputs)),
        ("Rudder Results", lambda: print_rudder_results(results, inputs)),
        ("Mission Profile", lambda: print_mission_profile(results)),
    ]
    
    for section_name, print_func in sections:
        try:
            print_func()
        except Exception as e:
            print(f"=== ERROR PRINTING {section_name.upper()} ===")
            print(f"Error: {type(e).__name__}: {e}")
            print("Continuing with remaining sections...")
            print()
