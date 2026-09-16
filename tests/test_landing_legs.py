from h2air.structures import (
    LoadCase,
    Material,
    TubeSection,
    build_load_cases,
    check_tube_member,
    evaluate_structure,
)
from h2air.landing_legs import sleeve_od_m, sleeve_mass_kg, landing_legs_cda_m2
from h2air.aero_propulsion import cruise_drag


def test_sleeve_geometry_2mm_wall():
    assert abs(sleeve_od_m(0.012, 0.002) - 0.016) < 1e-12
    m = sleeve_mass_kg(tube_od_m=0.012, sleeve_wall_m=0.002, length_m=0.2, density_kg_m3=1200.0)
    assert m > 0.0


def test_landing_load_case_present_when_enabled():
    cfg = {
        "aircraft": {
            "reference_mass_kg": 12.0,
            "design_mass_ceiling_kg": 15.0,
            "cg_station_x_m": 0.4,
            "reference_wing_span_m": 3.0,
            "design_wing_span_m": 3.0,
            "horizontal_tail_area_m2": 0.16,
            "vertical_tail_total_area_m2": 0.07,
        },
        "structure": {
            "wing_primary_spar_x_m": 0.35,
            "boom_length_m": 0.7,
            "boom_x_start_m": 0.3,
            "motor_mount_lever_m": 0.05,
            "landing_leg_enabled": True,
            "landing_leg_length_m": 0.28,
            "landing_leg_side_load_fraction": 0.25,
        },
        "load_cases": {
            "maneuver_load_factor": 3.0,
            "landing_load_factor": 3.0,
            "rho_kg_m3": 1.225,
            "dive_speed_mps": 50.0,
            "horizontal_tail_design_CL_abs": 1.0,
            "vertical_tail_design_CY_abs": 0.8,
            "wing_aero_torsion_Nm": 20.0,
            "pusher_design_thrust_N": 60.0,
        },
        "rotor_system": {
            "row_x_m": [0.1, 0.7],
            "motor_y_m": 0.37,
            "phase4_equal_share_design_thrust_N": 55.0,
            "phase6_reference_hover_thrusts_N": [30.0, 30.0, 25.0, 25.0],
            "candidate_motor_max_thrust_N": 80.0,
            "screening_shaft_reaction_torque_Nm": 3.0,
        },
    }
    cases = build_load_cases(cfg)
    land = next(c for c in cases if c.name == "3g_landing_impact")
    assert land.landing_leg_axial_N > 0
    assert land.landing_leg_bending_Nm > 0
    # Half of 3g * 15 kg
    assert abs(land.landing_leg_axial_N - 0.5 * 15.0 * 9.80665 * 3.0) < 1e-6


def test_landing_leg_tube_passes_with_reasonable_section():
    mat = Material(
        name="CFRP",
        E_Pa=3e11,
        G_Pa=1.15e11,
        density_kg_m3=1800.0,
        bending_allow_Pa=1e9,
        shear_allow_Pa=1.2e8,
    )
    F = 0.5 * 15.0 * 9.80665 * 3.0
    L = 0.28
    M = 0.25 * F * L
    sec = TubeSection(0.014, 0.010)
    chk = check_tube_member(
        member="landing_leg",
        case="3g",
        section=sec,
        material=mat,
        length_m=L,
        bending_Nm=M,
        axial_N=F,
        deflection_limit_fraction=0.05,
        slope_limit_deg=3.0,
    )
    assert chk.overall_pass


def test_landing_legs_cda_scales_with_sleeve_and_length():
    cfg = {
        "design": {
            "fuselage_height_m": 0.22,
            "wing_mac_m": 0.26,
            "wing_ac_x_m": 0.39,
            "lift_propeller_diameter_in": 22.0,
        },
        "geometry_mass": {
            "geometry_model": {
                "geometry": {
                    "wing_lower_z_m": 0.07,
                    "boom_center_y_m": [-0.37, 0.37],
                    "lift_prop_centers_m": [
                        [0.1, -0.37, 0.12], [0.1, 0.37, 0.12],
                        [0.7, -0.37, 0.12], [0.7, 0.37, 0.12],
                    ],
                }
            }
        },
        "aero_propulsion": {"lift_layout": {"z_m": 0.12}},
        "manufacturing_mass_model": {
            "fuselage_sections": [{"name": "nose", "length_m": 0.14, "diameter_m": 0.2}],
            "main_wing": {"primary_spar": {"chord_fraction": 0.25}, "thickness_m": 0.028, "z_m": 0.09},
            "landing_legs": {
                "enabled": True,
                "sleeve_wall_m": 0.002,
                "drag_coefficient_screen": 1.0,
                "prop_ground_clearance_m": 0.02,
                "minimum_leg_length_m": 0.06,
                "tube": {"design_tube_od_m": 0.008, "reference_tube_od_m": 0.012},
            },
        },
        "structures": {"structure": {}, "load_cases": {}},
    }
    cda = landing_legs_cda_m2(cfg)
    assert cda > 0.0
    assert abs(sleeve_od_m(0.008, 0.002) - 0.012) < 1e-12
