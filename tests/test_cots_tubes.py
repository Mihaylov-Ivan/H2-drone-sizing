from h2air.user_inputs import ceil_even_mm


def test_ceil_even_mm():
    assert ceil_even_mm(20.5) == 22
    assert ceil_even_mm(22.0) == 22
    assert ceil_even_mm(22.75) == 24
    assert ceil_even_mm(17.06) == 18
    assert ceil_even_mm(9.5) == 10
    assert ceil_even_mm(7.25) == 8
    assert ceil_even_mm(6.5) == 8


from h2air.structures import Material, size_cots_tube_for_bending_and_stiffness


def test_cots_autosizer_returns_lightest_passing_even_mm_section():
    mat = Material(
        name="CFRP screen", E_Pa=300e9, G_Pa=115e9,
        density_kg_m3=1575.0, bending_allow_Pa=1e9, shear_allow_Pa=120e6,
    )
    sec, chk = size_cots_tube_for_bending_and_stiffness(
        moment_Nm=198.5846625, length_m=1.8, material=mat,
        deflection_limit_fraction=0.03, slope_limit_deg=3.0,
        min_od_m=0.010, max_od_m=0.040, min_wall_m=0.001,
    )
    assert chk.overall_pass
    assert round(sec.od_m * 1000) % 2 == 0
    assert round(sec.id_m * 1000) % 2 == 0
    assert sec.od_m - sec.id_m >= 0.002 - 1e-12
