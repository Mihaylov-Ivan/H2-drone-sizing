import bisect

# ============================================================
# 1. XFOIL POLAR DATA FOR NACA 0006, Re = 50,000
# ============================================================

# Each entry is one line of your table
polar_data = [
    {"alpha": -8.250, "CL": -0.6744, "CD": 0.10463, "CDp": 0.09811, "CM":  0.0361, "Top_Xtr": 1.0000, "Bot_Xtr": 0.2841},
    {"alpha": -8.000, "CL": -0.6770, "CD": 0.10152, "CDp": 0.09507, "CM":  0.0360, "Top_Xtr": 1.0000, "Bot_Xtr": 0.3021},
    {"alpha": -7.750, "CL": -0.6873, "CD": 0.09891, "CDp": 0.09256, "CM":  0.0351, "Top_Xtr": 1.0000, "Bot_Xtr": 0.3183},
    {"alpha": -7.500, "CL": -0.6648, "CD": 0.09391, "CDp": 0.08753, "CM":  0.0381, "Top_Xtr": 1.0000, "Bot_Xtr": 0.3430},
    {"alpha": -7.000, "CL": -0.6541, "CD": 0.08634, "CDp": 0.08004, "CM":  0.0401, "Top_Xtr": 1.0000, "Bot_Xtr": 0.3826},
    {"alpha": -6.750, "CL": -0.6562, "CD": 0.08314, "CDp": 0.07691, "CM":  0.0410, "Top_Xtr": 1.0000, "Bot_Xtr": 0.4060},
    {"alpha": -6.500, "CL": -0.6571, "CD": 0.08012, "CDp": 0.07396, "CM":  0.0424, "Top_Xtr": 1.0000, "Bot_Xtr": 0.4327},
    {"alpha": -6.250, "CL": -0.6588, "CD": 0.05709, "CDp": 0.05000, "CM": -0.0072, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1590},
    {"alpha": -6.000, "CL": -0.6399, "CD": 0.04972, "CDp": 0.04189, "CM": -0.0102, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1308},
    {"alpha": -5.750, "CL": -0.6193, "CD": 0.04421, "CDp": 0.03547, "CM": -0.0109, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1188},
    {"alpha": -5.500, "CL": -0.5974, "CD": 0.03960, "CDp": 0.03037, "CM": -0.0106, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1141},
    {"alpha": -5.250, "CL": -0.5736, "CD": 0.03565, "CDp": 0.02578, "CM": -0.0099, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1133},
    {"alpha": -5.000, "CL": -0.5484, "CD": 0.03269, "CDp": 0.02204, "CM": -0.0090, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1202},
    {"alpha": -4.750, "CL": -0.5222, "CD": 0.02938, "CDp": 0.01850, "CM": -0.0083, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1264},
    {"alpha": -4.500, "CL": -0.4962, "CD": 0.02697, "CDp": 0.01584, "CM": -0.0074, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1435},
    {"alpha": -4.250, "CL": -0.4680, "CD": 0.02457, "CDp": 0.01319, "CM": -0.0064, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1608},
    {"alpha": -4.000, "CL": -0.4405, "CD": 0.02247, "CDp": 0.01106, "CM": -0.0054, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1897},
    {"alpha": -3.750, "CL": -0.4135, "CD": 0.02034, "CDp": 0.00925, "CM": -0.0046, "Top_Xtr": 1.0000, "Bot_Xtr": 0.2448},
    {"alpha": -3.500, "CL": -0.4071, "CD": 0.01635, "CDp": 0.00795, "CM":  0.0011, "Top_Xtr": 1.0000, "Bot_Xtr": 0.6026},
    {"alpha": -3.250, "CL": -0.2716, "CD": 0.01678, "CDp": 0.00780, "CM": -0.0089, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -3.000, "CL": -0.2535, "CD": 0.01614, "CDp": 0.00684, "CM": -0.0090, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -2.750, "CL": -0.2342, "CD": 0.01567, "CDp": 0.00607, "CM": -0.0088, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -2.500, "CL": -0.2140, "CD": 0.01531, "CDp": 0.00544, "CM": -0.0083, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -2.250, "CL": -0.1933, "CD": 0.01503, "CDp": 0.00489, "CM": -0.0078, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -2.000, "CL": -0.1723, "CD": 0.01481, "CDp": 0.00445, "CM": -0.0071, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -1.750, "CL": -0.1511, "CD": 0.01464, "CDp": 0.00409, "CM": -0.0063, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -1.500, "CL": -0.1297, "CD": 0.01450, "CDp": 0.00379, "CM": -0.0055, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -1.250, "CL": -0.1081, "CD": 0.01439, "CDp": 0.00352, "CM": -0.0047, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -1.000, "CL": -0.0866, "CD": 0.01431, "CDp": 0.00332, "CM": -0.0038, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -0.750, "CL": -0.0650, "CD": 0.01424, "CDp": 0.00318, "CM": -0.0029, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -0.500, "CL": -0.0434, "CD": 0.01420, "CDp": 0.00307, "CM": -0.0019, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha": -0.250, "CL": -0.0217, "CD": 0.01417, "CDp": 0.00300, "CM": -0.0010, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  0.000, "CL":  0.0000, "CD": 0.01416, "CDp": 0.00298, "CM":  0.0000, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  0.250, "CL":  0.0217, "CD": 0.01417, "CDp": 0.00300, "CM":  0.0010, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  0.500, "CL":  0.0434, "CD": 0.01420, "CDp": 0.00307, "CM":  0.0019, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  0.750, "CL":  0.0650, "CD": 0.01424, "CDp": 0.00317, "CM":  0.0028, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  1.000, "CL":  0.0866, "CD": 0.01430, "CDp": 0.00332, "CM":  0.0038, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  1.250, "CL":  0.1082, "CD": 0.01439, "CDp": 0.00352, "CM":  0.0047, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  1.500, "CL":  0.1297, "CD": 0.01450, "CDp": 0.00379, "CM":  0.0055, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  1.750, "CL":  0.1512, "CD": 0.01464, "CDp": 0.00409, "CM":  0.0063, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  2.000, "CL":  0.1724, "CD": 0.01481, "CDp": 0.00445, "CM":  0.0071, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  2.250, "CL":  0.1934, "CD": 0.01503, "CDp": 0.00488, "CM":  0.0077, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  2.500, "CL":  0.2141, "CD": 0.01531, "CDp": 0.00544, "CM":  0.0083, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  2.750, "CL":  0.2343, "CD": 0.01567, "CDp": 0.00607, "CM":  0.0087, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  3.000, "CL":  0.2537, "CD": 0.01614, "CDp": 0.00684, "CM":  0.0090, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  3.250, "CL":  0.2718, "CD": 0.01678, "CDp": 0.00780, "CM":  0.0089, "Top_Xtr": 1.0000, "Bot_Xtr": 1.0000},
    {"alpha":  3.500, "CL":  0.4071, "CD": 0.01635, "CDp": 0.00795, "CM": -0.0011, "Top_Xtr": 0.6032, "Bot_Xtr": 1.0000},
    {"alpha":  3.750, "CL":  0.4135, "CD": 0.02034, "CDp": 0.00925, "CM":  0.0046, "Top_Xtr": 0.2449, "Bot_Xtr": 1.0000},
    {"alpha":  4.000, "CL":  0.4405, "CD": 0.02247, "CDp": 0.01106, "CM":  0.0054, "Top_Xtr": 0.1897, "Bot_Xtr": 1.0000},
    {"alpha":  4.250, "CL":  0.4680, "CD": 0.02457, "CDp": 0.01319, "CM":  0.0064, "Top_Xtr": 0.1608, "Bot_Xtr": 1.0000},
    {"alpha":  4.500, "CL":  0.4962, "CD": 0.02697, "CDp": 0.01583, "CM":  0.0074, "Top_Xtr": 0.1435, "Bot_Xtr": 1.0000},
    {"alpha":  4.750, "CL":  0.5222, "CD": 0.02938, "CDp": 0.01849, "CM":  0.0083, "Top_Xtr": 0.1264, "Bot_Xtr": 1.0000},
    {"alpha":  5.000, "CL":  0.5484, "CD": 0.03269, "CDp": 0.02204, "CM":  0.0090, "Top_Xtr": 0.1202, "Bot_Xtr": 1.0000},
    {"alpha":  5.250, "CL":  0.5736, "CD": 0.03565, "CDp": 0.02578, "CM":  0.0099, "Top_Xtr": 0.1133, "Bot_Xtr": 1.0000},
    {"alpha":  5.500, "CL":  0.5974, "CD": 0.03960, "CDp": 0.03037, "CM":  0.0106, "Top_Xtr": 0.1141, "Bot_Xtr": 1.0000},
    {"alpha":  5.750, "CL":  0.6194, "CD": 0.04421, "CDp": 0.03548, "CM":  0.0109, "Top_Xtr": 0.1188, "Bot_Xtr": 1.0000},
    {"alpha":  6.000, "CL":  0.6399, "CD": 0.04973, "CDp": 0.04189, "CM":  0.0102, "Top_Xtr": 0.1308, "Bot_Xtr": 1.0000},
    {"alpha":  6.250, "CL":  0.6589, "CD": 0.05710, "CDp": 0.05001, "CM":  0.0072, "Top_Xtr": 0.1590, "Bot_Xtr": 1.0000},
    {"alpha":  7.000, "CL":  0.6546, "CD": 0.08637, "CDp": 0.08007, "CM": -0.0401, "Top_Xtr": 0.3824, "Bot_Xtr": 1.0000},
    {"alpha":  7.500, "CL":  0.6649, "CD": 0.09390, "CDp": 0.08752, "CM": -0.0383, "Top_Xtr": 0.3427, "Bot_Xtr": 1.0000},
    {"alpha":  7.750, "CL":  0.6872, "CD": 0.09890, "CDp": 0.09255, "CM": -0.0353, "Top_Xtr": 0.3183, "Bot_Xtr": 1.0000},
    {"alpha":  8.000, "CL":  0.6770, "CD": 0.10150, "CDp": 0.09505, "CM": -0.0362, "Top_Xtr": 0.3020, "Bot_Xtr": 1.0000},
    {"alpha":  8.250, "CL":  0.6750, "CD": 0.10464, "CDp": 0.09813, "CM": -0.0362, "Top_Xtr": 0.2836, "Bot_Xtr": 1.0000},
]

# Pre-sort by CL (XFOIL data are already monotonic in CL vs alpha, but we make sure)
polar_data = sorted(polar_data, key=lambda row: row["CL"])
CL_list = [row["CL"] for row in polar_data]
CM_list = [row["CM"] for row in polar_data]
CL_max = CL_list[-1]
CM_max = max(CM_list)  # Maximum CM value in the dataset

# ============================================================
# 2. INTERPOLATION FUNCTION BY REQUIRED CL
# ============================================================

def interpolate_by_CL(CL_req):
    """
    Given required section lift coefficient CL_req,
    interpolate to get alpha, CD, CDp, CM, Top_Xtr, Bot_Xtr.

    Returns a dictionary augmented with CL_max (largest CL in polar data).

    If CL_req is outside the tabulated range, the nearest endpoint is returned.
    """
    # If below or above the range, clamp to endpoints
    if CL_req <= CL_list[0]:
        result = polar_data[0].copy()
        result["CL_max"] = CL_max
        result["CM_max"] = CM_max
        return result
    if CL_req >= CL_list[-1]:
        result = polar_data[-1].copy()
        result["CL_max"] = CL_max
        result["CM_max"] = CM_max
        return result

    # Find insertion point
    idx = bisect.bisect_left(CL_list, CL_req)

    # Bracketing points (CL_low <= CL_req <= CL_high)
    low = polar_data[idx - 1]
    high = polar_data[idx]

    CL_low = low["CL"]
    CL_high = high["CL"]

    # Linear interpolation factor in CL space
    t = (CL_req - CL_low) / (CL_high - CL_low)

    def lerp(key):
        return low[key] + t * (high[key] - low[key])

    result = {
        "CL": CL_req,
        "alpha":  lerp("alpha"),
        "CD":     lerp("CD"),
        "CDp":    lerp("CDp"),
        "CM":     lerp("CM"),
        "Top_Xtr": lerp("Top_Xtr"),
        "Bot_Xtr": lerp("Bot_Xtr"),
        "CL_max": CL_max,
        "CM_max": CM_max,  # Constant: maximum CM in the entire dataset
        "max_thickness_fraction_of_chord": 0.06,
    }
    return result