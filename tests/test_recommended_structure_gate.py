from h2air.pipeline import recommended_structure_screen_pass


def test_reference_audit_fails_do_not_fail_recommended_screen():
    gates = [
        {"name": "reference_wing_spar_strength", "status": "FAIL"},
        {"name": "reference_wing_spar_stiffness", "status": "FAIL"},
        {"name": "reference_35x31_boom_screen", "status": "PASS"},
        {"name": "legacy_strength_only_boom_stiffness", "status": "FAIL"},
        {"name": "horizontal_tail_current_screen", "status": "FAIL"},
        {"name": "vertical_tail_current_screen", "status": "FAIL"},
        {"name": "recommended_wing_spar_screen", "status": "PASS"},
        {"name": "recommended_boom_screen", "status": "PASS"},
        {"name": "horizontal_tail_recommended_screen", "status": "PASS"},
        {"name": "vertical_tail_recommended_screen", "status": "PASS"},
        {"name": "central_pod_scaled_strength", "status": "PASS"},
        {"name": "motor_mount_strength", "status": "NOT_EVALUATED"},
        {"name": "mass_ceiling_after_partial_structural_feedback", "status": "FAIL"},
    ]
    assert recommended_structure_screen_pass(gates) is True


def test_recommended_member_fail_fails_screen():
    gates = [
        {"name": "reference_wing_spar_strength", "status": "FAIL"},
        {"name": "recommended_wing_spar_screen", "status": "FAIL"},
        {"name": "recommended_boom_screen", "status": "PASS"},
        {"name": "central_pod_scaled_strength", "status": "PASS"},
    ]
    assert recommended_structure_screen_pass(gates) is False
