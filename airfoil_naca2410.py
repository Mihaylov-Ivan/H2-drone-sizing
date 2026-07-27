import bisect

# ============================================================
# 1. XFOIL POLAR DATA FOR NACA 2410, Re = 500,000
# ============================================================

# Each entry is one line of your table
polar_data = [
    {"alpha": -11.250, "CL": -0.3936, "CD":  0.11300, "CDp":  0.10942, "CM": -0.0212, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0626},
    {"alpha": -11.000, "CL": -0.3908, "CD":  0.10977, "CDp":  0.10620, "CM": -0.0218, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0651},
    {"alpha": -10.750, "CL": -0.5323, "CD":  0.11221, "CDp":  0.10849, "CM": -0.0161, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0601},
    {"alpha": -10.500, "CL": -0.5207, "CD":  0.10975, "CDp":  0.10602, "CM": -0.0149, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0616},
    {"alpha": -10.250, "CL": -0.5144, "CD":  0.10664, "CDp":  0.10291, "CM": -0.0154, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0635},
    {"alpha": -10.000, "CL": -0.4153, "CD":  0.09108, "CDp":  0.08760, "CM": -0.0291, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0730},
    {"alpha":  -9.750, "CL": -0.4053, "CD":  0.08850, "CDp":  0.08503, "CM": -0.0280, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0750},
    {"alpha":  -9.500, "CL": -0.4044, "CD":  0.08480, "CDp":  0.08134, "CM": -0.0286, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0772},
    {"alpha":  -9.250, "CL": -0.4099, "CD":  0.08027, "CDp":  0.07683, "CM": -0.0302, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0798},
    {"alpha":  -9.000, "CL": -0.6155, "CD":  0.05953, "CDp":  0.05580, "CM": -0.0500, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0466},
    {"alpha":  -8.750, "CL": -0.6276, "CD":  0.05395, "CDp":  0.05008, "CM": -0.0505, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0457},
    {"alpha":  -8.500, "CL": -0.6408, "CD":  0.04803, "CDp":  0.04387, "CM": -0.0499, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0450},
    {"alpha":  -8.250, "CL": -0.6495, "CD":  0.04243, "CDp":  0.03787, "CM": -0.0484, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0450},
    {"alpha":  -8.000, "CL": -0.6516, "CD":  0.03762, "CDp":  0.03256, "CM": -0.0463, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0454},
    {"alpha":  -7.750, "CL": -0.6474, "CD":  0.03363, "CDp":  0.02804, "CM": -0.0441, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0461},
    {"alpha":  -7.500, "CL": -0.6367, "CD":  0.03108, "CDp":  0.02496, "CM": -0.0419, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0473},
    {"alpha":  -7.250, "CL": -0.6259, "CD":  0.02760, "CDp":  0.02116, "CM": -0.0404, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0495},
    {"alpha":  -7.000, "CL": -0.6080, "CD":  0.02652, "CDp":  0.02005, "CM": -0.0391, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0518},
    {"alpha":  -6.750, "CL": -0.5898, "CD":  0.02503, "CDp":  0.01832, "CM": -0.0377, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0541},
    {"alpha":  -6.500, "CL": -0.5707, "CD":  0.02382, "CDp":  0.01679, "CM": -0.0362, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0569},
    {"alpha":  -6.250, "CL": -0.5509, "CD":  0.02243, "CDp":  0.01507, "CM": -0.0349, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0593},
    {"alpha":  -6.000, "CL": -0.5302, "CD":  0.02077, "CDp":  0.01335, "CM": -0.0341, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0620},
    {"alpha":  -5.750, "CL": -0.5089, "CD":  0.02010, "CDp":  0.01261, "CM": -0.0331, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0653},
    {"alpha":  -5.500, "CL": -0.4867, "CD":  0.01934, "CDp":  0.01169, "CM": -0.0321, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0682},
    {"alpha":  -5.250, "CL": -0.4640, "CD":  0.01871, "CDp":  0.01088, "CM": -0.0312, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0705},
    {"alpha":  -5.000, "CL": -0.4408, "CD":  0.01748, "CDp":  0.00963, "CM": -0.0307, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0741},
    {"alpha":  -4.750, "CL": -0.4175, "CD":  0.01688, "CDp":  0.00903, "CM": -0.0300, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0774},
    {"alpha":  -4.500, "CL": -0.3935, "CD":  0.01634, "CDp":  0.00842, "CM": -0.0294, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0810},
    {"alpha":  -4.250, "CL": -0.3694, "CD":  0.01583, "CDp":  0.00784, "CM": -0.0289, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0852},
    {"alpha":  -4.000, "CL": -0.3456, "CD":  0.01525, "CDp":  0.00735, "CM": -0.0286, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0916},
    {"alpha":  -3.750, "CL": -0.3072, "CD":  0.01462, "CDp":  0.00675, "CM": -0.0309, "Top_Xtr": 0.9966, "Bot_Xtr": 0.1021},
    {"alpha":  -3.500, "CL": -0.2664, "CD":  0.01403, "CDp":  0.00625, "CM": -0.0339, "Top_Xtr": 0.9925, "Bot_Xtr": 0.1262},
    {"alpha":  -3.250, "CL": -0.2262, "CD":  0.01339, "CDp":  0.00589, "CM": -0.0368, "Top_Xtr": 0.9877, "Bot_Xtr": 0.1752},
    {"alpha":  -3.000, "CL": -0.1855, "CD":  0.01293, "CDp":  0.00570, "CM": -0.0399, "Top_Xtr": 0.9829, "Bot_Xtr": 0.2355},
    {"alpha":  -2.750, "CL": -0.1461, "CD":  0.01250, "CDp":  0.00553, "CM": -0.0427, "Top_Xtr": 0.9768, "Bot_Xtr": 0.2996},
    {"alpha":  -2.500, "CL": -0.1068, "CD":  0.01206, "CDp":  0.00539, "CM": -0.0453, "Top_Xtr": 0.9711, "Bot_Xtr": 0.3740},
    {"alpha":  -2.250, "CL": -0.0695, "CD":  0.01143, "CDp":  0.00529, "CM": -0.0476, "Top_Xtr": 0.9648, "Bot_Xtr": 0.4935},
    {"alpha":  -2.000, "CL": -0.0338, "CD":  0.01076, "CDp":  0.00529, "CM": -0.0490, "Top_Xtr": 0.9593, "Bot_Xtr": 0.6532},
    {"alpha":  -1.750, "CL": -0.0021, "CD":  0.01032, "CDp":  0.00529, "CM": -0.0490, "Top_Xtr": 0.9518, "Bot_Xtr": 0.7741},
    {"alpha":  -1.500, "CL":  0.0370, "CD":  0.01009, "CDp":  0.00529, "CM": -0.0500, "Top_Xtr": 0.9481, "Bot_Xtr": 0.8762},
    {"alpha":  -1.250, "CL":  0.0814, "CD":  0.01004, "CDp":  0.00525, "CM": -0.0525, "Top_Xtr": 0.9416, "Bot_Xtr": 0.9461},
    {"alpha":  -1.000, "CL":  0.1439, "CD":  0.00994, "CDp":  0.00509, "CM": -0.0592, "Top_Xtr": 0.9391, "Bot_Xtr": 0.9857},
    {"alpha":  -0.750, "CL":  0.2034, "CD":  0.00976, "CDp":  0.00482, "CM": -0.0657, "Top_Xtr": 0.9353, "Bot_Xtr": 1.0000},
    {"alpha":  -0.500, "CL":  0.2357, "CD":  0.00959, "CDp":  0.00458, "CM": -0.0664, "Top_Xtr": 0.9227, "Bot_Xtr": 1.0000},
    {"alpha":  -0.250, "CL":  0.2659, "CD":  0.00943, "CDp":  0.00435, "CM": -0.0666, "Top_Xtr": 0.9095, "Bot_Xtr": 1.0000},
    {"alpha":  0.000, "CL":  0.2938, "CD":  0.00929, "CDp":  0.00415, "CM": -0.0663, "Top_Xtr": 0.8952, "Bot_Xtr": 1.0000},
    {"alpha":  0.250, "CL":  0.3200, "CD":  0.00918, "CDp":  0.00397, "CM": -0.0656, "Top_Xtr": 0.8797, "Bot_Xtr": 1.0000},
    {"alpha":  0.500, "CL":  0.3453, "CD":  0.00910, "CDp":  0.00382, "CM": -0.0647, "Top_Xtr": 0.8633, "Bot_Xtr": 1.0000},
    {"alpha":  0.750, "CL":  0.3704, "CD":  0.00904, "CDp":  0.00370, "CM": -0.0637, "Top_Xtr": 0.8461, "Bot_Xtr": 1.0000},
    {"alpha":  1.000, "CL":  0.3954, "CD":  0.00902, "CDp":  0.00359, "CM": -0.0627, "Top_Xtr": 0.8285, "Bot_Xtr": 1.0000},
    {"alpha":  1.250, "CL":  0.4200, "CD":  0.00904, "CDp":  0.00353, "CM": -0.0616, "Top_Xtr": 0.8096, "Bot_Xtr": 1.0000},
    {"alpha":  1.500, "CL":  0.4445, "CD":  0.00909, "CDp":  0.00350, "CM": -0.0606, "Top_Xtr": 0.7893, "Bot_Xtr": 1.0000},
    {"alpha":  1.750, "CL":  0.4693, "CD":  0.00916, "CDp":  0.00348, "CM": -0.0597, "Top_Xtr": 0.7693, "Bot_Xtr": 1.0000},
    {"alpha":  2.000, "CL":  0.4941, "CD":  0.00926, "CDp":  0.00349, "CM": -0.0587, "Top_Xtr": 0.7492, "Bot_Xtr": 1.0000},
    {"alpha":  2.250, "CL":  0.5187, "CD":  0.00938, "CDp":  0.00354, "CM": -0.0578, "Top_Xtr": 0.7276, "Bot_Xtr": 1.0000},
    {"alpha":  2.500, "CL":  0.5436, "CD":  0.00952, "CDp":  0.00358, "CM": -0.0569, "Top_Xtr": 0.7071, "Bot_Xtr": 1.0000},
    {"alpha":  2.750, "CL":  0.5684, "CD":  0.00968, "CDp":  0.00367, "CM": -0.0561, "Top_Xtr": 0.6862, "Bot_Xtr": 1.0000},
    {"alpha":  3.000, "CL":  0.5932, "CD":  0.00985, "CDp":  0.00377, "CM": -0.0552, "Top_Xtr": 0.6646, "Bot_Xtr": 1.0000},
    {"alpha":  3.250, "CL":  0.6180, "CD":  0.01005, "CDp":  0.00386, "CM": -0.0544, "Top_Xtr": 0.6433, "Bot_Xtr": 1.0000},
    {"alpha":  3.500, "CL":  0.6423, "CD":  0.01023, "CDp":  0.00400, "CM": -0.0535, "Top_Xtr": 0.6187, "Bot_Xtr": 1.0000},
    {"alpha":  3.750, "CL":  0.6668, "CD":  0.01044, "CDp":  0.00412, "CM": -0.0526, "Top_Xtr": 0.5952, "Bot_Xtr": 1.0000},
    {"alpha":  4.000, "CL":  0.6911, "CD":  0.01066, "CDp":  0.00427, "CM": -0.0517, "Top_Xtr": 0.5702, "Bot_Xtr": 1.0000},
    {"alpha":  4.250, "CL":  0.7151, "CD":  0.01089, "CDp":  0.00444, "CM": -0.0508, "Top_Xtr": 0.5430, "Bot_Xtr": 1.0000},
    {"alpha":  4.500, "CL":  0.7387, "CD":  0.01115, "CDp":  0.00460, "CM": -0.0499, "Top_Xtr": 0.5135, "Bot_Xtr": 1.0000},
    {"alpha":  4.750, "CL":  0.7623, "CD":  0.01142, "CDp":  0.00479, "CM": -0.0489, "Top_Xtr": 0.4837, "Bot_Xtr": 1.0000},
    {"alpha":  5.000, "CL":  0.7860, "CD":  0.01171, "CDp":  0.00502, "CM": -0.0481, "Top_Xtr": 0.4540, "Bot_Xtr": 1.0000},
    {"alpha":  5.250, "CL":  0.8092, "CD":  0.01203, "CDp":  0.00526, "CM": -0.0472, "Top_Xtr": 0.4204, "Bot_Xtr": 1.0000},
    {"alpha":  5.500, "CL":  0.8318, "CD":  0.01242, "CDp":  0.00553, "CM": -0.0462, "Top_Xtr": 0.3825, "Bot_Xtr": 1.0000},
    {"alpha":  5.750, "CL":  0.8537, "CD":  0.01290, "CDp":  0.00586, "CM": -0.0451, "Top_Xtr": 0.3375, "Bot_Xtr": 1.0000},
    {"alpha":  6.000, "CL":  0.8740, "CD":  0.01355, "CDp":  0.00628, "CM": -0.0439, "Top_Xtr": 0.2798, "Bot_Xtr": 1.0000},
    {"alpha":  6.250, "CL":  0.8916, "CD":  0.01455, "CDp":  0.00688, "CM": -0.0425, "Top_Xtr": 0.2040, "Bot_Xtr": 1.0000},
    {"alpha":  6.500, "CL":  0.9062, "CD":  0.01601, "CDp":  0.00782, "CM": -0.0407, "Top_Xtr": 0.1264, "Bot_Xtr": 1.0000},
    {"alpha":  6.750, "CL":  0.9231, "CD":  0.01725, "CDp":  0.00883, "CM": -0.0390, "Top_Xtr": 0.0965, "Bot_Xtr": 1.0000},
    {"alpha":  7.000, "CL":  0.9421, "CD":  0.01822, "CDp":  0.00977, "CM": -0.0377, "Top_Xtr": 0.0849, "Bot_Xtr": 1.0000},
    {"alpha":  7.250, "CL":  0.9591, "CD":  0.01941, "CDp":  0.01087, "CM": -0.0361, "Top_Xtr": 0.0781, "Bot_Xtr": 1.0000},
    {"alpha":  7.500, "CL":  0.9803, "CD":  0.02018, "CDp":  0.01175, "CM": -0.0349, "Top_Xtr": 0.0741, "Bot_Xtr": 1.0000},
    {"alpha":  7.750, "CL":  1.0006, "CD":  0.02105, "CDp":  0.01264, "CM": -0.0338, "Top_Xtr": 0.0701, "Bot_Xtr": 1.0000},
    {"alpha":  8.000, "CL":  1.0186, "CD":  0.02240, "CDp":  0.01393, "CM": -0.0324, "Top_Xtr": 0.0662, "Bot_Xtr": 1.0000},
    {"alpha":  8.250, "CL":  1.0399, "CD":  0.02343, "CDp":  0.01506, "CM": -0.0314, "Top_Xtr": 0.0638, "Bot_Xtr": 1.0000},
    {"alpha":  8.500, "CL":  1.0613, "CD":  0.02435, "CDp":  0.01610, "CM": -0.0304, "Top_Xtr": 0.0609, "Bot_Xtr": 1.0000},
    {"alpha":  8.750, "CL":  1.0820, "CD":  0.02532, "CDp":  0.01711, "CM": -0.0294, "Top_Xtr": 0.0578, "Bot_Xtr": 1.0000},
    {"alpha":  9.000, "CL":  1.1028, "CD":  0.02701, "CDp":  0.01876, "CM": -0.0287, "Top_Xtr": 0.0549, "Bot_Xtr": 1.0000},
    {"alpha":  9.250, "CL":  1.1235, "CD":  0.02844, "CDp":  0.02038, "CM": -0.0277, "Top_Xtr": 0.0525, "Bot_Xtr": 1.0000},
    {"alpha":  9.500, "CL":  1.1436, "CD":  0.02960, "CDp":  0.02173, "CM": -0.0266, "Top_Xtr": 0.0499, "Bot_Xtr": 1.0000},
    {"alpha":  9.750, "CL":  1.1626, "CD":  0.03083, "CDp":  0.02308, "CM": -0.0255, "Top_Xtr": 0.0473, "Bot_Xtr": 1.0000},
    {"alpha": 10.000, "CL":  1.1818, "CD":  0.03268, "CDp":  0.02490, "CM": -0.0248, "Top_Xtr": 0.0446, "Bot_Xtr": 1.0000},
    {"alpha": 10.250, "CL":  1.1973, "CD":  0.03514, "CDp":  0.02766, "CM": -0.0234, "Top_Xtr": 0.0427, "Bot_Xtr": 1.0000},
    {"alpha": 10.500, "CL":  1.2111, "CD":  0.03687, "CDp":  0.02970, "CM": -0.0216, "Top_Xtr": 0.0409, "Bot_Xtr": 1.0000},
    {"alpha": 10.750, "CL":  1.2238, "CD":  0.03857, "CDp":  0.03162, "CM": -0.0199, "Top_Xtr": 0.0388, "Bot_Xtr": 1.0000},
    {"alpha": 11.000, "CL":  1.2361, "CD":  0.04000, "CDp":  0.03316, "CM": -0.0183, "Top_Xtr": 0.0370, "Bot_Xtr": 1.0000},
    {"alpha": 11.250, "CL":  1.2478, "CD":  0.04266, "CDp":  0.03588, "CM": -0.0172, "Top_Xtr": 0.0354, "Bot_Xtr": 1.0000},
    {"alpha": 11.500, "CL":  1.2451, "CD":  0.04728, "CDp":  0.04089, "CM": -0.0146, "Top_Xtr": 0.0345, "Bot_Xtr": 1.0000},
    {"alpha": 11.750, "CL":  1.2394, "CD":  0.04942, "CDp":  0.04337, "CM": -0.0111, "Top_Xtr": 0.0340, "Bot_Xtr": 1.0000},
    {"alpha": 12.000, "CL":  1.2283, "CD":  0.05198, "CDp":  0.04625, "CM": -0.0075, "Top_Xtr": 0.0335, "Bot_Xtr": 1.0000},
    {"alpha": 12.250, "CL":  1.2140, "CD":  0.05501, "CDp":  0.04958, "CM": -0.0045, "Top_Xtr": 0.0332, "Bot_Xtr": 1.0000},
    {"alpha": 12.500, "CL":  1.1974, "CD":  0.05852, "CDp":  0.05338, "CM": -0.0024, "Top_Xtr": 0.0329, "Bot_Xtr": 1.0000},
    {"alpha": 12.750, "CL":  1.1785, "CD":  0.06262, "CDp":  0.05774, "CM": -0.0014, "Top_Xtr": 0.0326, "Bot_Xtr": 1.0000},
    {"alpha": 13.000, "CL":  1.1572, "CD":  0.06742, "CDp":  0.06279, "CM": -0.0015, "Top_Xtr": 0.0326, "Bot_Xtr": 1.0000},
    {"alpha": 13.250, "CL":  1.1338, "CD":  0.07304, "CDp":  0.06865, "CM": -0.0030, "Top_Xtr": 0.0326, "Bot_Xtr": 1.0000},
    {"alpha": 13.500, "CL":  1.1081, "CD":  0.07966, "CDp":  0.07548, "CM": -0.0058, "Top_Xtr": 0.0328, "Bot_Xtr": 1.0000},
    {"alpha": 13.750, "CL":  1.0810, "CD":  0.08726, "CDp":  0.08327, "CM": -0.0099, "Top_Xtr": 0.0332, "Bot_Xtr": 1.0000},
    {"alpha": 14.000, "CL":  1.0529, "CD":  0.09590, "CDp":  0.09207, "CM": -0.0150, "Top_Xtr": 0.0337, "Bot_Xtr": 1.0000},
    {"alpha": 14.250, "CL":  1.0245, "CD":  0.10545, "CDp":  0.10173, "CM": -0.0206, "Top_Xtr": 0.0342, "Bot_Xtr": 1.0000},
    {"alpha": 14.500, "CL":  0.6680, "CD":  0.14879, "CDp":  0.14542, "CM": -0.0292, "Top_Xtr": 0.0778, "Bot_Xtr": 1.0000},
]

# Pre-sort by alpha (XFOIL data are already monotonic in alpha, but we make sure)
polar_data = sorted(polar_data, key=lambda row: row["alpha"])
alpha_list = [row["alpha"] for row in polar_data]
CL_list = [row["CL"] for row in polar_data]
CM_list = [row["CM"] for row in polar_data]
CL_max = max(CL_list)  # Maximum CL value in the dataset (positive)
CL_min = min(CL_list)  # Minimum CL value in the dataset (most negative)
CM_max = max(CM_list)  # Maximum CM value in the dataset
CM_min = min(CM_list)  # Minimum CM value in the dataset (most negative)

# ============================================================
# 2. INTERPOLATION FUNCTION BY REQUIRED ALPHA
# ============================================================

def interpolate_by_alpha(alpha_req):
    """
    Given required angle of attack alpha_req,
    interpolate to get CL, CD, CDp, CM, Top_Xtr, Bot_Xtr.

    Returns a dictionary augmented with CL_max and CM_max:
    - If alpha_req < 0: returns CL_min (most negative CL) and CM_min (most negative CM)
    - If alpha_req >= 0: returns CL_max (most positive CL) and CM_max (maximum CM)

    If alpha_req is outside the tabulated range, the nearest endpoint is returned.
    """
    # Determine which CL_max and CM_max to use based on alpha_req sign
    if alpha_req < 0:
        CL_max_to_use = CL_min  # Use minimum (most negative) CL for negative angles
        CM_max_to_use = CM_min  # Use minimum (most negative) CM for negative angles
    else:
        CL_max_to_use = CL_max  # Use maximum (most positive) CL for positive/zero angles
        CM_max_to_use = CM_max  # Use maximum CM for positive/zero angles
    
    # If below or above the range, clamp to endpoints
    if alpha_req <= alpha_list[0]:
        result = polar_data[0].copy()
        result["CL_max"] = CL_max_to_use
        result["CM_max"] = CM_max_to_use
        result["CL_max_pos"] = CL_max
        result["CL_max_neg"] = CL_min
        result["max_thickness_fraction_of_chord"] = 0.1
        return result
    if alpha_req >= alpha_list[-1]:
        result = polar_data[-1].copy()
        result["CL_max"] = CL_max_to_use
        result["CM_max"] = CM_max_to_use
        result["CL_max_pos"] = CL_max
        result["CL_max_neg"] = CL_min
        result["max_thickness_fraction_of_chord"] = 0.1
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
        "CL_max": CL_max_to_use,
        "CM_max": CM_max_to_use,
        "CL_max_pos": CL_max,
        "CL_max_neg": CL_min,
        "max_thickness_fraction_of_chord": 0.1,
    }
    return result