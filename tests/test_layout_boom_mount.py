"""Layout architecture: high-wing boom-through-wing + top/bottom longerons."""
from h2air.config import load_model_config
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_high_wing_top_bottom_longeron_stackup():
    cfg = load_model_config(ROOT / "config/user_inputs.json")
    geom = cfg["geometry_mass"]["geometry_model"]["geometry"]
    lay = cfg["aero_propulsion"]["lift_layout"]
    mm = cfg["manufacturing_mass_model"]
    d = cfg["design"]
    res = cfg["coupling"]["resolution"]

    wing_z = float(mm["main_wing"]["z_m"])
    wing_th = float(mm["main_wing"]["thickness_m"])
    fuse_top = float(geom["fuselage_top_z_m"])
    flush = float(geom["wing_z_flush_m"])
    z_top = float(geom["fuselage_longeron_top_z_m"])
    z_bot = float(geom["fuselage_longeron_bottom_z_m"])

    assert mm["fuselage_longerons"]["layout"] == "top_bottom"
    assert z_top > 0 and z_bot < 0
    assert abs(z_top + z_bot) < 1e-9
    assert abs(float(geom["fuselage_longeron_y_abs_m"])) < 1e-12
    assert abs(wing_z - float(d["main_wing_z_m"])) < 1e-9
    assert abs(float(geom["boom_z_m"]) - wing_z) < 1e-9
    assert wing_z <= flush + 1e-9
    assert wing_z + 0.5 * wing_th <= fuse_top + 1e-6

    spar_od = float(mm["main_wing"]["primary_spar"]["tube_od_m"])
    boss_r = 0.5 * float(mm["fuselage_longerons"]["boss_od_m"])
    boss_bottom = z_top - boss_r
    spar_top = wing_z + 0.5 * spar_od
    assert spar_top <= boss_bottom - float(mm["fuselage_longerons"]["spar_stack_gap_m"]) + 1e-6

    disk_z = float(geom["lift_prop_disk_z_m"])
    wing_upper = wing_z + 0.5 * wing_th
    assert disk_z + 1e-9 >= wing_upper + float(lay["required_disk_edge_clearance_m"])
    assert res["fuselage_longeron_layout"] == "top_bottom"
