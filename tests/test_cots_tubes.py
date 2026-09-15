from h2air.user_inputs import ceil_even_mm


def test_ceil_even_mm():
    assert ceil_even_mm(20.5) == 22
    assert ceil_even_mm(22.0) == 22
    assert ceil_even_mm(22.75) == 24
    assert ceil_even_mm(17.06) == 18
    assert ceil_even_mm(9.5) == 10
    assert ceil_even_mm(7.25) == 8
    assert ceil_even_mm(6.5) == 8
