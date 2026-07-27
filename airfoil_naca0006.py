import bisect

# ============================================================
# 1. XFOIL POLAR DATA FOR NACA 0006, Re = 1,000,000
# ============================================================

# Each entry is one line of your table
polar_data = [
    {"alpha": -8.750, "CL": -0.7405, "CD": 0.08222, "CDp": 0.08077, "CM": 0.0149, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0070},
    {"alpha": -8.500, "CL": -0.7508, "CD": 0.07216, "CDp": 0.07073, "CM": 0.0032, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0069},
    {"alpha": -8.250, "CL": -0.7533, "CD": 0.06306, "CDp": 0.06151, "CM": -0.0039, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0068},
    {"alpha": -8.000, "CL": -0.7521, "CD": 0.05597, "CDp": 0.05427, "CM": -0.0074, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0069},
    {"alpha": -7.750, "CL": -0.7462, "CD": 0.05056, "CDp": 0.04869, "CM": -0.0090, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0070},
    {"alpha": -7.500, "CL": -0.7370, "CD": 0.04574, "CDp": 0.04365, "CM": -0.0098, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0072},
    {"alpha": -7.250, "CL": -0.7382, "CD": 0.03375, "CDp": 0.03109, "CM": -0.0092, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0056},
    {"alpha": -7.000, "CL": -0.7345, "CD": 0.02389, "CDp": 0.02039, "CM": -0.0071, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0052},
    {"alpha": -6.750, "CL": -0.7169, "CD": 0.01969, "CDp": 0.01565, "CM": -0.0058, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0053},
    {"alpha": -6.500, "CL": -0.6954, "CD": 0.01711, "CDp": 0.01268, "CM": -0.0048, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0054},
    {"alpha": -6.250, "CL": -0.6709, "CD": 0.01593, "CDp": 0.01132, "CM": -0.0043, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0058},
    {"alpha": -6.000, "CL": -0.6461, "CD": 0.01480, "CDp": 0.01001, "CM": -0.0037, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0061},
    {"alpha": -5.750, "CL": -0.6209, "CD": 0.01377, "CDp": 0.00881, "CM": -0.0031, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0063},
    {"alpha": -5.500, "CL": -0.6001, "CD": 0.01111, "CDp": 0.00580, "CM": -0.0018, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0071},
    {"alpha": -5.250, "CL": -0.5746, "CD": 0.01047, "CDp": 0.00509, "CM": -0.0014, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0079},
    {"alpha": -5.000, "CL": -0.5488, "CD": 0.00994, "CDp": 0.00451, "CM": -0.0010, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0088},
    {"alpha": -4.750, "CL": -0.5225, "CD": 0.00959, "CDp": 0.00410, "CM": -0.0007, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0099},
    {"alpha": -4.500, "CL": -0.4973, "CD": 0.00878, "CDp": 0.00313, "CM": 0.0000, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0128},
    {"alpha": -4.250, "CL": -0.4711, "CD": 0.00839, "CDp": 0.00275, "CM": 0.0004, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0182},
    {"alpha": -4.000, "CL": -0.4448, "CD": 0.00812, "CDp": 0.00250, "CM": 0.0007, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0281},
    {"alpha": -3.750, "CL": -0.4184, "CD": 0.00796, "CDp": 0.00233, "CM": 0.0010, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0330},
    {"alpha": -3.500, "CL": -0.3923, "CD": 0.00774, "CDp": 0.00211, "CM": 0.0013, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0377},
    {"alpha": -3.250, "CL": -0.3662, "CD": 0.00761, "CDp": 0.00196, "CM": 0.0016, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0400},
    {"alpha": -3.000, "CL": -0.3403, "CD": 0.00737, "CDp": 0.00170, "CM": 0.0021, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0446},
    {"alpha": -2.750, "CL": -0.3145, "CD": 0.00718, "CDp": 0.00152, "CM": 0.0025, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0498},
    {"alpha": -2.500, "CL": -0.2890, "CD": 0.00689, "CDp": 0.00133, "CM": 0.0029, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0753},
    {"alpha": -2.250, "CL": -0.2645, "CD": 0.00622, "CDp": 0.00113, "CM": 0.0033, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1947},
    {"alpha": -2.000, "CL": -0.2396, "CD": 0.00570, "CDp": 0.00100, "CM": 0.0036, "Top_Xtr": 1.0000, "Bot_Xtr": 0.3040},
    {"alpha": -1.750, "CL": -0.2142, "CD": 0.00530, "CDp": 0.00090, "CM": 0.0038, "Top_Xtr": 1.0000, "Bot_Xtr": 0.3937},
    {"alpha": -1.500, "CL": -0.1884, "CD": 0.00495, "CDp": 0.00085, "CM": 0.0041, "Top_Xtr": 1.0000, "Bot_Xtr": 0.4785},
    {"alpha": -1.250, "CL": -0.1608, "CD": 0.00464, "CDp": 0.00081, "CM": 0.0039, "Top_Xtr": 0.9995, "Bot_Xtr": 0.5581},
    {"alpha": -1.000, "CL": -0.1251, "CD": 0.00426, "CDp": 0.00077, "CM": 0.0020, "Top_Xtr": 0.9958, "Bot_Xtr": 0.6541},
    {"alpha": -0.750, "CL": -0.0896, "CD": 0.00395, "CDp": 0.00074, "CM": 0.0002, "Top_Xtr": 0.9904, "Bot_Xtr": 0.7336},
    {"alpha": -0.500, "CL": -0.0529, "CD": 0.00365, "CDp": 0.00073, "CM": -0.0016, "Top_Xtr": 0.9776, "Bot_Xtr": 0.8123},
    {"alpha": -0.250, "CL": -0.0237, "CD": 0.00344, "CDp": 0.00073, "CM": -0.0015, "Top_Xtr": 0.9539, "Bot_Xtr": 0.8808},
    {"alpha": 0.000, "CL": 0.0000, "CD": 0.00338, "CDp": 0.00074, "CM": 0.0000, "Top_Xtr": 0.9238, "Bot_Xtr": 0.9241},
    {"alpha": 0.250, "CL": 0.0238, "CD": 0.00344, "CDp": 0.00073, "CM": 0.0015, "Top_Xtr": 0.8810, "Bot_Xtr": 0.9537},
    {"alpha": 0.500, "CL": 0.0528, "CD": 0.00365, "CDp": 0.00073, "CM": 0.0016, "Top_Xtr": 0.8123, "Bot_Xtr": 0.9774},
    {"alpha": 0.750, "CL": 0.0896, "CD": 0.00396, "CDp": 0.00074, "CM": -0.0002, "Top_Xtr": 0.7330, "Bot_Xtr": 0.9905},
    {"alpha": 1.000, "CL": 0.1251, "CD": 0.00426, "CDp": 0.00077, "CM": -0.0020, "Top_Xtr": 0.6539, "Bot_Xtr": 0.9958},
    {"alpha": 1.250, "CL": 0.1609, "CD": 0.00464, "CDp": 0.00081, "CM": -0.0039, "Top_Xtr": 0.5581, "Bot_Xtr": 0.9995},
    {"alpha": 1.500, "CL": 0.1883, "CD": 0.00495, "CDp": 0.00085, "CM": -0.0040, "Top_Xtr": 0.4790, "Bot_Xtr": 1.0000},
    {"alpha": 1.750, "CL": 0.2140, "CD": 0.00530, "CDp": 0.00090, "CM": -0.0038, "Top_Xtr": 0.3938, "Bot_Xtr": 1.0000},
    {"alpha": 2.000, "CL": 0.2394, "CD": 0.00570, "CDp": 0.00100, "CM": -0.0035, "Top_Xtr": 0.3043, "Bot_Xtr": 1.0000},
    {"alpha": 2.250, "CL": 0.2643, "CD": 0.00622, "CDp": 0.00113, "CM": -0.0032, "Top_Xtr": 0.1959, "Bot_Xtr": 1.0000},
    {"alpha": 2.500, "CL": 0.2888, "CD": 0.00689, "CDp": 0.00133, "CM": -0.0029, "Top_Xtr": 0.0756, "Bot_Xtr": 1.0000},
    {"alpha": 2.750, "CL": 0.3143, "CD": 0.00718, "CDp": 0.00152, "CM": -0.0024, "Top_Xtr": 0.0499, "Bot_Xtr": 1.0000},
    {"alpha": 3.000, "CL": 0.3401, "CD": 0.00737, "CDp": 0.00170, "CM": -0.0020, "Top_Xtr": 0.0447, "Bot_Xtr": 1.0000},
    {"alpha": 3.250, "CL": 0.3660, "CD": 0.00761, "CDp": 0.00196, "CM": -0.0016, "Top_Xtr": 0.0400, "Bot_Xtr": 1.0000},
    {"alpha": 3.500, "CL": 0.3921, "CD": 0.00774, "CDp": 0.00211, "CM": -0.0012, "Top_Xtr": 0.0377, "Bot_Xtr": 1.0000},
    {"alpha": 3.750, "CL": 0.4182, "CD": 0.00796, "CDp": 0.00233, "CM": -0.0009, "Top_Xtr": 0.0331, "Bot_Xtr": 1.0000},
    {"alpha": 4.000, "CL": 0.4446, "CD": 0.00812, "CDp": 0.00250, "CM": -0.0006, "Top_Xtr": 0.0281, "Bot_Xtr": 1.0000},
    {"alpha": 4.250, "CL": 0.4709, "CD": 0.00839, "CDp": 0.00274, "CM": -0.0003, "Top_Xtr": 0.0181, "Bot_Xtr": 1.0000},
    {"alpha": 4.500, "CL": 0.4972, "CD": 0.00878, "CDp": 0.00313, "CM": 0.0001, "Top_Xtr": 0.0127, "Bot_Xtr": 1.0000},
    {"alpha": 4.750, "CL": 0.5225, "CD": 0.00958, "CDp": 0.00409, "CM": 0.0007, "Top_Xtr": 0.0099, "Bot_Xtr": 1.0000},
    {"alpha": 5.000, "CL": 0.5487, "CD": 0.00996, "CDp": 0.00452, "CM": 0.0010, "Top_Xtr": 0.0088, "Bot_Xtr": 1.0000},
    {"alpha": 5.250, "CL": 0.5746, "CD": 0.01046, "CDp": 0.00509, "CM": 0.0014, "Top_Xtr": 0.0079, "Bot_Xtr": 1.0000},
    {"alpha": 5.500, "CL": 0.6001, "CD": 0.01111, "CDp": 0.00581, "CM": 0.0018, "Top_Xtr": 0.0071, "Bot_Xtr": 1.0000},
    {"alpha": 5.750, "CL": 0.6211, "CD": 0.01371, "CDp": 0.00874, "CM": 0.0031, "Top_Xtr": 0.0063, "Bot_Xtr": 1.0000},
    {"alpha": 6.000, "CL": 0.6462, "CD": 0.01479, "CDp": 0.01000, "CM": 0.0037, "Top_Xtr": 0.0061, "Bot_Xtr": 1.0000},
    {"alpha": 6.250, "CL": 0.6710, "CD": 0.01594, "CDp": 0.01133, "CM": 0.0043, "Top_Xtr": 0.0058, "Bot_Xtr": 1.0000},
    {"alpha": 6.500, "CL": 0.6954, "CD": 0.01713, "CDp": 0.01270, "CM": 0.0048, "Top_Xtr": 0.0054, "Bot_Xtr": 1.0000},
    {"alpha": 6.750, "CL": 0.7170, "CD": 0.01971, "CDp": 0.01568, "CM": 0.0058, "Top_Xtr": 0.0053, "Bot_Xtr": 1.0000},
    {"alpha": 7.000, "CL": 0.7348, "CD": 0.02384, "CDp": 0.02034, "CM": 0.0071, "Top_Xtr": 0.0052, "Bot_Xtr": 1.0000},
    {"alpha": 7.250, "CL": 0.7383, "CD": 0.03381, "CDp": 0.03116, "CM": 0.0091, "Top_Xtr": 0.0056, "Bot_Xtr": 1.0000},
    {"alpha": 10.500, "CL": 0.7164, "CD": 0.12067, "CDp": 0.11910, "CM": -0.0360, "Top_Xtr": 0.0063, "Bot_Xtr": 1.0000},
    {"alpha": 10.750, "CL": 0.7174, "CD": 0.12546, "CDp": 0.12388, "CM": -0.0379, "Top_Xtr": 0.0063, "Bot_Xtr": 1.0000},
]

# Pre-sort by alpha (XFOIL data are already monotonic in alpha, but we make sure)
polar_data = sorted(polar_data, key=lambda row: row["alpha"])
alpha_list = [row["alpha"] for row in polar_data]
CL_list = [row["CL"] for row in polar_data]
CM_list = [row["CM"] for row in polar_data]
CL_max = max(CL_list)  # Maximum CL value in the dataset
CL_min = min(CL_list)  # Minimum CL value in the dataset
CM_max = max(CM_list)  # Maximum CM value in the dataset
CM_min = min(CM_list)  # Minimum CM value in the dataset

# ============================================================
# 2. INTERPOLATION FUNCTION BY REQUIRED ALPHA
# ============================================================

def interpolate_by_alpha(alpha_req):
    """
    Given required angle of attack alpha_req,
    interpolate to get CL, CD, CDp, CM, Top_Xtr, Bot_Xtr.

    Returns a dictionary augmented with CL_max (largest CL in polar data).

    If alpha_req is outside the tabulated range, the nearest endpoint is returned.
    """
    # If below or above the range, clamp to endpoints
    if alpha_req <= alpha_list[0]:
        result = polar_data[0].copy()
        result["CL_max"] = CL_max
        result["CM_max"] = CM_max
        result["CL_max_pos"] = CL_max
        result["CL_max_neg"] = CL_min
        result["max_thickness_fraction_of_chord"] = 0.06
        return result
    if alpha_req >= alpha_list[-1]:
        result = polar_data[-1].copy()
        result["CL_max"] = CL_max
        result["CM_max"] = CM_max
        result["CL_max_pos"] = CL_max
        result["CL_max_neg"] = CL_min
        result["max_thickness_fraction_of_chord"] = 0.06
        return result

    # Find insertion point
    idx = bisect.bisect_left(alpha_list, alpha_req)

    # Bracketing points (alpha_low <= alpha_req <= alpha_high)
    low = polar_data[idx - 1]
    high = polar_data[idx]

    alpha_low = low["alpha"]
    alpha_high = high["alpha"]

    # Linear interpolation factor in alpha space
    t = (alpha_req - alpha_low) / (alpha_high - alpha_low)

    def lerp(key):
        return low[key] + t * (high[key] - low[key])

    result = {
        "alpha": alpha_req,
        "CL":     lerp("CL"),
        "CD":     lerp("CD"),
        "CDp":    lerp("CDp"),
        "CM":     lerp("CM"),
        "Top_Xtr": lerp("Top_Xtr"),
        "Bot_Xtr": lerp("Bot_Xtr"),
        "CL_max": CL_max,
        "CM_max": CM_max,  # Constant: maximum CM in the entire dataset
        "CL_max_pos": CL_max,
        "CL_max_neg": CL_min,
        "max_thickness_fraction_of_chord": 0.06,
    }
    return result