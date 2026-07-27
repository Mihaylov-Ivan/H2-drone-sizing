def get_battery_database():
    """
    Representative off-the-shelf LiPo packs for small-to-medium UAVs.
    Values are approximate (V, mAh, mass, C-rating, dimensions).
    """

    return [
        # --- SMALL / MEDIUM PACKS ---

        {
            "name": "LiPo_3S_2200mAh_25C",
            "voltage_V": 11.1,
            "capacity_mAh": 2200,
            "mass_kg": 0.18,
            "max_discharge_C": 25,
            "length_m": 0.105,
            "width_m": 0.035,
            "height_m": 0.025,
        },
        {
            "name": "LiPo_3S_5000mAh_30C",
            "voltage_V": 11.1,
            "capacity_mAh": 5000,
            "mass_kg": 0.40,
            "max_discharge_C": 30,
            "length_m": 0.145,
            "width_m": 0.045,
            "height_m": 0.035,
        },
        {
            "name": "LiPo_4S_3000mAh_30C",
            "voltage_V": 14.8,
            "capacity_mAh": 3000,
            "mass_kg": 0.30,
            "max_discharge_C": 30,
            "length_m": 0.135,
            "width_m": 0.045,
            "height_m": 0.030,
        },
        {
            "name": "LiPo_4S_5000mAh_45C",
            "voltage_V": 14.8,
            "capacity_mAh": 5000,
            "mass_kg": 0.50,
            "max_discharge_C": 45,
            "length_m": 0.155,
            "width_m": 0.047,
            "height_m": 0.030,
        },
        {
            "name": "LiPo_2S_6000mAh_100C_TurnigyRapid",
            "voltage_V": 7.4,
            "capacity_mAh": 6000,
            "mass_kg": 0.310,
            "max_discharge_C": 100,
            "length_m": 0.156,
            "width_m": 0.046,
            "height_m": 0.022,
        },
        {
            "name": "LiPo_2S_8000mAh_100C_CNHL_Racing",
            "voltage_V": 7.4,
            "capacity_mAh": 8000,
            "mass_kg": 0.320,
            "max_discharge_C": 100,
            "length_m": 0.139,
            "width_m": 0.047,
            "height_m": 0.025,
        },
        {
            "name": "LiPo_3S_6000mAh_65C_CNHL_Black",
            "voltage_V": 11.1,
            "capacity_mAh": 6000,
            "mass_kg": 0.4895,
            "max_discharge_C": 65,
            "length_m": 0.155,
            "width_m": 0.045,
            "height_m": 0.034,
        },
        {
            "name": "LiPo_3S_7000mAh_25C_Tattu",
            "voltage_V": 11.1,
            "capacity_mAh": 7000,
            "mass_kg": 0.455,
            "max_discharge_C": 25,
            "length_m": 0.140,
            "width_m": 0.041,
            "height_m": 0.035,
        },
        {
            "name": "LiPo_3S_8000mAh_15C_Tattu",
            "voltage_V": 11.1,
            "capacity_mAh": 8000,
            "mass_kg": 0.481,
            "max_discharge_C": 15,
            "length_m": 0.146,
            "width_m": 0.070,
            "height_m": 0.022,
        },
        {
            "name": "LiPo_4S_8000mAh_30C",
            "voltage_V": 14.8,
            "capacity_mAh": 8000,
            "mass_kg": 0.75,
            "max_discharge_C": 30,
            "length_m": 0.165,
            "width_m": 0.055,
            "height_m": 0.038,
        },
        {
            "name": "LiPo_4S_10000mAh_25C",
            "voltage_V": 14.8,
            "capacity_mAh": 10000,
            "mass_kg": 0.93,
            "max_discharge_C": 25,
            "length_m": 0.170,
            "width_m": 0.066,
            "height_m": 0.039,
        },
        {
            "name": "LiPo_6S_8000mAh_30C",
            "voltage_V": 22.2,
            "capacity_mAh": 8000,
            "mass_kg": 1.10,
            "max_discharge_C": 30,
            "length_m": 0.170,
            "width_m": 0.060,
            "height_m": 0.055,
        },
        {
            "name": "LiPo_6S_10000mAh_30C",
            "voltage_V": 22.2,
            "capacity_mAh": 10000,
            "mass_kg": 1.35,
            "max_discharge_C": 30,
            "length_m": 0.176,
            "width_m": 0.066,
            "height_m": 0.058,
        },

        # --- NEW: LARGER / HIGH-CAPACITY PACKS ---

        # ~16 Ah 4S pack (typical “professional UAV” 4S 16000mAh)
        {
            "name": "LiPo_4S_16000mAh_30C",
            "voltage_V": 14.8,
            "capacity_mAh": 16000,
            "mass_kg": 1.35,   # ≈ 1.33–1.35 kg
            "max_discharge_C": 30,
            "length_m": 0.194, # 194 x 76 x 44 mm
            "width_m": 0.076,
            "height_m": 0.044,
        },

        # ~16 Ah 6S pack (big multirotor / mapping packs)
        {
            "name": "LiPo_6S_16000mAh_15C",
            "voltage_V": 22.2,
            "capacity_mAh": 16000,
            "mass_kg": 1.86,   # ≈ 1.85–1.88 kg
            "max_discharge_C": 15,
            "length_m": 0.185, # ~185 x 77 x 66 mm
            "width_m": 0.077,
            "height_m": 0.066,
        },

        # Higher-C 6S 16000 (if you want more power headroom)
        {
            "name": "LiPo_6S_16000mAh_25C",
            "voltage_V": 22.2,
            "capacity_mAh": 16000,
            "mass_kg": 1.88,   # slightly heavier HV / higher-C versions
            "max_discharge_C": 25,
            "length_m": 0.190, # ~190–202 mm long
            "width_m": 0.078,
            "height_m": 0.065,
        },

        # ~22 Ah 6S pack (DJI S1000 / heavy multirotor class)
        {
            "name": "LiPo_6S_22000mAh_25C",
            "voltage_V": 22.2,
            "capacity_mAh": 22000,
            "mass_kg": 2.43,   # ≈ 2.4–2.46 kg
            "max_discharge_C": 25,
            "length_m": 0.207, # 207 x 91 x 64 mm
            "width_m": 0.091,
            "height_m": 0.064,
        },

        # 12S 16 Ah pack (big ag / heavy UAV battery)
        {
            "name": "LiPo_12S_16000mAh_60C",
            "voltage_V": 44.4,
            "capacity_mAh": 16000,
            "mass_kg": 3.82,   # ≈ 3.8 kg
            "max_discharge_C": 60,
            "length_m": 0.186, # 186 x 76 x 160 mm
            "width_m": 0.076,
            "height_m": 0.160,
        },
    ]