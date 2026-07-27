# propulsion_db.py
"""
Representative off-the-shelf motor + propeller combinations
for small / medium / heavy multirotor-style UAVs.

All values are approximate and conservative:
- max_thrust_N: approx. max static thrust per motor
- max_power_W:   recommended max continuous electrical power
"""
def get_propulsion_database():
    return [
        # ----------------------------------------------------
        # SMALL / LIGHT COMBOS (racing / light fixed-wing)
        # ----------------------------------------------------
        {
            "name": "Mini_2204_2300kV_6x4.5_3S",
            "motor_kv": 2300,
            "voltage_min_V": 9.0,
            "voltage_max_V": 12.6,
            "max_power_W": 250.0,
            "max_thrust_N": 7.0,
            "prop_diameter_in": 6.0,
            "prop_pitch_in": 4.5,
            "motor_mass_kg": 0.028,
            "prop_mass_kg": 0.006,
        },
        
        # T-HOBBY (LIGPOWER / T-Motor) F90 2806.5 KV1300 on 6S with HQ 7040x3 (HQ 70403)
        # Bench: ~2360 g thrust @ ~23.48 V, ~45.11 A, ~1059 W (100% throttle)
        {
            "name": "THobby_F90_2806p5_1300kV_HQ7040x3_6S",
            "motor_kv": 1300,
            "voltage_min_V": 18.0,      # rated 5–6S
            "voltage_max_V": 25.2,
            "max_power_W": 900.0,       # conservative continuous vs ~1059W 60s bench max
            "max_thrust_N": 23.15,      # 2360 g ≈ 23.15 N
            "prop_diameter_in": 7.0,
            "prop_pitch_in": 4.0,       # "7040"
            "motor_mass_kg": 0.0466,    # incl. cable
            "prop_mass_kg": 0.012,      # typical 7" tri-blade (approx.)
        },

        # T-HOBBY (LIGPOWER / T-Motor) F90 2806.5 KV1500 on 6S with HQ 7040x3 (HQ 70403)
        # Bench: ~2103 g thrust @ ~23.01 V, ~47.37 A, ~1090 W (100% throttle)
        {
            "name": "THobby_F90_2806p5_1500kV_HQ7040x3_6S",
            "motor_kv": 1500,
            "voltage_min_V": 18.0,      # rated 5–6S
            "voltage_max_V": 25.2,
            "max_power_W": 900.0,       # conservative continuous vs ~1090W 60s bench max
            "max_thrust_N": 20.62,      # 2103 g ≈ 20.62 N
            "prop_diameter_in": 7.0,
            "prop_pitch_in": 4.0,       # "7040"
            "motor_mass_kg": 0.0467,    # incl. cable (very close to KV1300)
            "prop_mass_kg": 0.012,      # approx.
        },

        # BrotherHobby Avenger V2 2507 1200KV on 6S with HQ 7x3.5x3 tri-blade
        # Bench: ~2020 g thrust @ 23.9–24.0 V, 33.4 A, ~798 W (100% throttle)
        {
            "name": "BrotherHobby_AvengerV2_2507_1200kV_HQ7x3p5x3_6S",
            "motor_kv": 1200,
            "voltage_min_V": 14.8,      # rated 4–6S
            "voltage_max_V": 25.2,
            "max_power_W": 650.0,       # conservative continuous vs ~798W bench max
            "max_thrust_N": 19.81,      # 2020 g ≈ 19.81 N
            "prop_diameter_in": 7.0,
            "prop_pitch_in": 3.5,
            "motor_mass_kg": 0.040,     # with wires
            "prop_mass_kg": 0.010,      # typical 7" tri-blade (approx.)
        },

        # ----------------------------------------------------
        # MEDIUM COMBOS (F450 / 1–2 kg class per 4 motors)
        # ----------------------------------------------------
        {
            "name": "A2212_920kV_10x4.5_3S",
            "motor_kv": 920,
            "voltage_min_V": 10.0,
            "voltage_max_V": 12.6,
            "max_power_W": 370.0,
            "max_thrust_N": 11.8,
            "prop_diameter_in": 10.0,
            "prop_pitch_in": 4.5,
            "motor_mass_kg": 0.053,
            "prop_mass_kg": 0.015,
        },
        {
            "name": "A2216_900kV_11x4.7_3S",
            "motor_kv": 900,
            "voltage_min_V": 10.0,
            "voltage_max_V": 12.6,
            "max_power_W": 450.0,
            "max_thrust_N": 15.0,
            "prop_diameter_in": 11.0,
            "prop_pitch_in": 4.7,
            "motor_mass_kg": 0.070,
            "prop_mass_kg": 0.018,
        },

        # ----------------------------------------------------
        # MID-HEAVY COMBOS (3–6 kg quad range)
        # ----------------------------------------------------
        {
            "name": "Multistar_2814_700kV_12x4.5_4S",
            "motor_kv": 700,
            "voltage_min_V": 12.0,
            "voltage_max_V": 16.8,
            "max_power_W": 350.0,
            "max_thrust_N": 14.5,
            "prop_diameter_in": 12.0,
            "prop_pitch_in": 4.5,
            "motor_mass_kg": 0.075,
            "prop_mass_kg": 0.020,
        },
        {
            "name": "U3_700kV_13x4.4_4S",
            "motor_kv": 700,
            "voltage_min_V": 12.0,
            "voltage_max_V": 16.8,
            "max_power_W": 500.0,
            "max_thrust_N": 20.0,
            "prop_diameter_in": 13.0,
            "prop_pitch_in": 4.4,
            "motor_mass_kg": 0.090,
            "prop_mass_kg": 0.025,
        },

        # ----------------------------------------------------
        # HEAVY COMBOS (heavy quad / hex class)
        # ----------------------------------------------------
        {
            "name": "MAD_4014_370kV_15x5_6S",
            "motor_kv": 370,
            "voltage_min_V": 18.0,
            "voltage_max_V": 25.2,
            "max_power_W": 500.0,
            "max_thrust_N": 28.0,
            "prop_diameter_in": 15.0,
            "prop_pitch_in": 5.0,
            "motor_mass_kg": 0.142,
            "prop_mass_kg": 0.035,
        },
        {
            "name": "Heavy_4014_370kV_17x5.8_6S",
            "motor_kv": 370,
            "voltage_min_V": 18.0,
            "voltage_max_V": 25.2,
            "max_power_W": 800.0,
            "max_thrust_N": 35.0,
            "prop_diameter_in": 17.0,
            "prop_pitch_in": 5.8,
            "motor_mass_kg": 0.150,
            "prop_mass_kg": 0.040,
        },

        # ----------------------------------------------------
        # HEAVY-LIFT COMBOS (40–60 N per motor)  [NEW]
        # ----------------------------------------------------

        # T-MOTOR U8 Lite KV150 @ 6S (24V) with G-series props
        {
            "name": "Tmotor_U8Lite_150kV_G28x9.2_6S",
            "motor_kv": 150,
            "voltage_min_V": 18.0,      # 6S window
            "voltage_max_V": 25.2,
            "max_power_W": 552.0,       # at 100% throttle in test table
            "max_thrust_N": 47.8,       # 4879 g
            "prop_diameter_in": 28.0,
            "prop_pitch_in": 9.2,
            "motor_mass_kg": 0.239,
            "prop_mass_kg": 0.085,
        },
        {
            "name": "Tmotor_U8Lite_150kV_G29x9.5_6S",
            "motor_kv": 150,
            "voltage_min_V": 18.0,
            "voltage_max_V": 25.2,
            "max_power_W": 629.0,       # at 100% throttle
            "max_thrust_N": 52.7,       # 5378 g
            "prop_diameter_in": 29.0,
            "prop_pitch_in": 9.5,
            "motor_mass_kg": 0.239,
            "prop_mass_kg": 0.095,
        },
        {
            "name": "Tmotor_U8Lite_150kV_G30x10.5_6S",
            "motor_kv": 150,
            "voltage_min_V": 18.0,
            "voltage_max_V": 25.2,
            "max_power_W": 713.0,       # at 100% throttle
            "max_thrust_N": 58.0,       # 5912 g
            "prop_diameter_in": 30.0,
            "prop_pitch_in": 10.5,
            "motor_mass_kg": 0.239,
            "prop_mass_kg": 0.097,
        },

        # T-MOTOR MN6007II (test-table combos)
        {
            "name": "Tmotor_MN6007II_320kV_P22x6.6_6S",
            "motor_kv": 320,
            "voltage_min_V": 18.0,      # 6S window
            "voltage_max_V": 25.2,
            "max_power_W": 1041.0,      # at 100% throttle
            "max_thrust_N": 57.7,       # 5882 g
            "prop_diameter_in": 22.0,
            "prop_pitch_in": 6.6,
            "motor_mass_kg": 0.161,
            "prop_mass_kg": 0.045,
        },
        {
            "name": "Tmotor_MN6007II_160kV_P21x6.3_12S",
            "motor_kv": 160,
            "voltage_min_V": 36.0,      # 12S window
            "voltage_max_V": 50.4,
            "max_power_W": 978.0,       # at 100% throttle
            "max_thrust_N": 57.3,       # 5838 g
            "prop_diameter_in": 21.0,
            "prop_pitch_in": 6.3,
            "motor_mass_kg": 0.159,
            "prop_mass_kg": 0.058,
        },

        # T-MOTOR MN501-S (Navigator) test-table combos
        {
            "name": "Tmotor_MN501S_300kV_P22x6.6_6S",
            "motor_kv": 300,
            "voltage_min_V": 18.0,      # 6S table
            "voltage_max_V": 25.2,
            "max_power_W": 916.0,       # at 100% throttle
            "max_thrust_N": 51.7,       # 5277 g
            "prop_diameter_in": 22.0,
            "prop_pitch_in": 6.6,
            "motor_mass_kg": 0.170,
            "prop_mass_kg": 0.045,
        },
        {
            "name": "Tmotor_MN501S_240kV_P15x5_12S",
            "motor_kv": 240,
            "voltage_min_V": 36.0,      # 12S table (~48.5V)
            "voltage_max_V": 50.4,
            "max_power_W": 1121.0,      # at 100% throttle
            "max_thrust_N": 45.7,       # 4660 g
            "prop_diameter_in": 15.0,
            "prop_pitch_in": 5.0,
            "motor_mass_kg": 0.170,
            "prop_mass_kg": 0.021,
        },
        {
            "name": "Tmotor_MN501S_240kV_P22x6.6_6S",
            "motor_kv": 240,
            "voltage_min_V": 18.0,
            "voltage_max_V": 25.2,
            "max_power_W": 612.0,       # at 100% throttle
            "max_thrust_N": 41.1,       # 4187 g
            "prop_diameter_in": 22.0,
            "prop_pitch_in": 6.6,
            "motor_mass_kg": 0.170,
            "prop_mass_kg": 0.045,
        },

        # T-MOTOR U7 V2.0 KV280 (Ligpower page test table)
        {
            "name": "Tmotor_U7V2_280kV_22x6.6_6S",
            "motor_kv": 280,
            "voltage_min_V": 18.0,      # 6S window (table uses 24V)
            "voltage_max_V": 25.2,
            "max_power_W": 842.0,       # at 100% throttle
            "max_thrust_N": 45.5,       # 4640 g
            "prop_diameter_in": 22.0,
            "prop_pitch_in": 6.6,
            "motor_mass_kg": 0.317,
            "prop_mass_kg": 0.045,
        },

        # ----------------------------------------------------
        # 60 N CLASS (throttle-limited / capped at 60 N)  [NEW]
        # ----------------------------------------------------

        # KDE Direct KDE7215XF-135 (motor specs: ~555 g, up to ~4405 W for 180 s, 6S–14S, props up to 30.5")
        # Note: max_thrust_N is capped at 60 N to fit your database constraint.
        {
            "name": "KDE7215XF_135kV_KDE24.5x8.1_DUAL_6S_capped60N",
            "motor_kv": 135,
            "voltage_min_V": 18.0,     # 6S
            "voltage_max_V": 25.2,
            "max_power_W": 4400.0,
            "max_thrust_N": 60.0,
            "prop_diameter_in": 24.5,
            "prop_pitch_in": 8.1,
            "motor_mass_kg": 0.555,
            "prop_mass_kg": 0.080,     # conservative estimate (update if you have exact prop mass)
        },
        {
            "name": "KDE7215XF_135kV_KDE24.5x8.1_TRIPLE_6S_capped60N",
            "motor_kv": 135,
            "voltage_min_V": 18.0,
            "voltage_max_V": 25.2,
            "max_power_W": 4400.0,
            "max_thrust_N": 60.0,
            "prop_diameter_in": 24.5,
            "prop_pitch_in": 8.1,
            "motor_mass_kg": 0.555,
            "prop_mass_kg": 0.090,
        },
        {
            "name": "KDE7215XF_135kV_KDE27.5x8.9_DUAL_6S_capped60N",
            "motor_kv": 135,
            "voltage_min_V": 18.0,
            "voltage_max_V": 25.2,
            "max_power_W": 4400.0,
            "max_thrust_N": 60.0,
            "prop_diameter_in": 27.5,
            "prop_pitch_in": 8.9,
            "motor_mass_kg": 0.555,
            "prop_mass_kg": 0.100,
        },
        {
            "name": "KDE7215XF_135kV_KDE27.5x8.9_TRIPLE_6S_capped60N",
            "motor_kv": 135,
            "voltage_min_V": 18.0,
            "voltage_max_V": 25.2,
            "max_power_W": 4400.0,
            "max_thrust_N": 60.0,
            "prop_diameter_in": 27.5,
            "prop_pitch_in": 8.9,
            "motor_mass_kg": 0.555,
            "prop_mass_kg": 0.110,
        },
        {
            "name": "KDE7215XF_135kV_KDE30.5x9.7_DUAL_6S_capped60N",
            "motor_kv": 135,
            "voltage_min_V": 18.0,
            "voltage_max_V": 25.2,
            "max_power_W": 4400.0,
            "max_thrust_N": 60.0,
            "prop_diameter_in": 30.5,
            "prop_pitch_in": 9.7,
            "motor_mass_kg": 0.555,
            "prop_mass_kg": 0.125,
        },
        {
            "name": "KDE7215XF_135kV_KDE30.5x9.7_TRIPLE_6S_capped60N",
            "motor_kv": 135,
            "voltage_min_V": 18.0,
            "voltage_max_V": 25.2,
            "max_power_W": 4400.0,
            "max_thrust_N": 60.0,
            "prop_diameter_in": 30.5,
            "prop_pitch_in": 9.7,
            "motor_mass_kg": 0.555,
            "prop_mass_kg": 0.135,
        },

        # T-MOTOR MN705-S based integrated propulsion (705-X / X-705). These kits can exceed 60 N, so we cap to 60 N.
        # Props shown on the kit page include G26*8.5 and P24*7.2 variants.
        {
            "name": "Tmotor_MN705S_125kV_G26x8.5_12S_capped60N",
            "motor_kv": 125,
            "voltage_min_V": 36.0,      # 12S
            "voltage_max_V": 50.4,
            "max_power_W": 2000.0,      # conservative placeholder (update from your own test table if available)
            "max_thrust_N": 60.0,
            "prop_diameter_in": 26.0,
            "prop_pitch_in": 8.5,
            "motor_mass_kg": 0.310,     # placeholder (use MN705-S motor mass if you want it exact)
            "prop_mass_kg": 0.095,
        },
        {
            "name": "Tmotor_MN705S_125kV_P24x7.2_12S_capped60N",
            "motor_kv": 125,
            "voltage_min_V": 36.0,
            "voltage_max_V": 50.4,
            "max_power_W": 2000.0,
            "max_thrust_N": 60.0,
            "prop_diameter_in": 24.0,
            "prop_pitch_in": 7.2,
            "motor_mass_kg": 0.310,
            "prop_mass_kg": 0.085,
        },
    ]
