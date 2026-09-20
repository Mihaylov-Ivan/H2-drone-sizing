from pathlib import Path

from h2air.avl_integration import (
    _normalize_controls,
    _parse_avl_output,
    find_avl_executable,
    render_avl_geometry,
)


def test_avl_parser_and_control_convention_mapping():
    text = """
 Stability-axis derivatives...
 z force     |    CLa =   4.500000    CLb =   0.000000
 y force     |    CYa =   0.000000    CYb =  -0.350000
 roll  x mom.|    Cla =   0.000000    Clb =  -0.060000
 pitch y mom.|    Cma =  -0.550000    Cmb =   0.000000
 yaw   z mom.|    Cna =   0.000000    Cnb =   0.090000
 pitch y mom.|    Cmp =   0.000000    Cmq =  -7.100000    Cmr = 0.000000
 yaw   z mom.|    Cnp =  -0.030000    Cnq =   0.000000    Cnr = -0.140000
 roll  x mom.|  Cld1 =   0.150000
 pitch y mom.|  Cmd2 =  -0.900000
 y force     |  CYd3 =   0.120000
 roll  x mom.|  Cld3 =   0.020000
 yaw   z mom.|  Cnd3 =  -0.080000
 Neutral point  Xnp =   0.500000
 """
    vals = _parse_avl_output(text)
    out = _normalize_controls(vals)
    assert out["CL_alpha_per_rad"] == 4.5
    assert out["Cm_alpha_per_rad"] == -0.55
    assert out["neutral_point_station_x_m"] == 0.5
    assert out["Cl_da"] > 0
    assert out["Cm_de"] < 0
    assert out["Cn_dr"] < 0
    assert out["Cn_beta"] > 0


def test_avl_parser_accepts_mit_zero_padded_control_labels():
    # MIT AVL 3.5x Windows builds print d01/d02/d03 instead of d1/d2/d3.
    text = """
 z' force CL |    CLa =   5.740725
 y  mom.  Cm |    Cma =  -1.353318
 x' mom.  Cl'|   Cld01 =  -0.006154
 y  mom.  Cm |   Cmd02 =  -0.020756
 y  force CY |   CYd03 =  -0.005703
 z' mom.  Cn'|   Cnd03 =   0.001121
 Neutral point  Xnp =   0.509607
 """
    out = _normalize_controls(_parse_avl_output(text))
    assert out["Cl_da"] > 0
    assert out["Cm_de"] < 0
    assert out["Cn_dr"] < 0
    assert out["CY_dr"] > 0


def test_avl_parser_keeps_first_cnb_not_spiral_ratio():
    text = """
 z' mom.  Cn'|    Cna =  -0.000000    Cnb =   0.096634
 Clb Cnr / Clr Cnb  =   0.053422    (  > 1 if spirally stable )
 """
    vals = _parse_avl_output(text)
    assert vals["Cnb"] == 0.096634


def test_avl_geometry_export_contains_current_surfaces_and_controls(model_config):
    # A mass result is supplied by the normal mass-properties pipeline in real use.
    from h2air.pipeline import _components_with_power, _mass_result
    mass = _mass_result(model_config, _components_with_power(model_config))
    text = render_avl_geometry(model_config, mass)
    assert "SURFACE\nMainWing" in text
    assert "SURFACE\nHorizontalTail" in text
    assert "SURFACE\nLeftVerticalTail" in text
    assert "SURFACE\nRightVerticalTail" in text
    assert "CONTROL\naileron" in text
    assert "CONTROL\nelevator" in text
    assert "CONTROL\nrudder" in text
    assert text.count("CONTROL\nelevator") == 2
    assert text.count("CONTROL\nrudder") == 4
    # Default user design opts into partial-span/height controls at 30% chord.
    assert "elevator 1.000000 0.700000" in text
    assert "rudder 1.000000 0.700000" in text
    assert "NACA\n6415" in text
    assert text.count("NACA\n0012") >= 7


def test_pinned_ci_avl_is_available():
    # The repository CI workflow intentionally installs the pinned AVL build.
    assert find_avl_executable({"mode": "avl"}) is not None
