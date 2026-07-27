import bisect

# ============================================================
# 1. XFOIL POLAR DATA FOR NACA 0012, Re = 200,000
# ============================================================

# Each entry is one line of your table
polar_data = [
    {"alpha": -14.000, "CL": -0.9628, "CD": 0.07963, "CDp": 0.07518, "CM": -0.0278, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0399},
    {"alpha": -13.750, "CL": -0.9895, "CD": 0.07225, "CDp": 0.06762, "CM": -0.0322, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0398},
    {"alpha": -13.500, "CL": -1.0148, "CD": 0.06593, "CDp": 0.06110, "CM": -0.0354, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0397},
    {"alpha": -13.250, "CL": -1.0382, "CD": 0.06063, "CDp": 0.05557, "CM": -0.0371, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0396},
    {"alpha": -13.000, "CL": -1.0594, "CD": 0.05622, "CDp": 0.05090, "CM": -0.0373, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0396},
    {"alpha": -12.750, "CL": -1.0784, "CD": 0.05255, "CDp": 0.04697, "CM": -0.0361, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0397},
    {"alpha": -12.500, "CL": -1.0955, "CD": 0.04958, "CDp": 0.04371, "CM": -0.0335, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0399},
    {"alpha": -12.000, "CL": -1.1063, "CD": 0.04365, "CDp": 0.03734, "CM": -0.0286, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0410},
    {"alpha": -11.750, "CL": -1.0964, "CD": 0.04195, "CDp": 0.03559, "CM": -0.0273, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0420},
    {"alpha": -11.500, "CL": -1.0857, "CD": 0.04034, "CDp": 0.03389, "CM": -0.0259, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0430},
    {"alpha": -11.250, "CL": -1.0770, "CD": 0.03842, "CDp": 0.03176, "CM": -0.0241, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0441},
    {"alpha": -11.000, "CL": -1.0679, "CD": 0.03635, "CDp": 0.02942, "CM": -0.0222, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0451},
    {"alpha": -10.750, "CL": -1.0569, "CD": 0.03444, "CDp": 0.02717, "CM": -0.0203, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0462},
    {"alpha": -10.500, "CL": -1.0443, "CD": 0.03262, "CDp": 0.02503, "CM": -0.0186, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0474},
    {"alpha": -10.250, "CL": -1.0249, "CD": 0.03076, "CDp": 0.02323, "CM": -0.0180, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0492},
    {"alpha": -10.000, "CL": -1.0066, "CD": 0.02966, "CDp": 0.02206, "CM": -0.0169, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0511},
    {"alpha": -9.750, "CL": -0.9881, "CD": 0.02844, "CDp": 0.02068, "CM": -0.0156, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0532},
    {"alpha": -9.500, "CL": -0.9692, "CD": 0.02734, "CDp": 0.01932, "CM": -0.0143, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0550},
    {"alpha": -9.250, "CL": -0.9491, "CD": 0.02559, "CDp": 0.01762, "CM": -0.0135, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0576},
    {"alpha": -9.000, "CL": -0.9290, "CD": 0.02473, "CDp": 0.01674, "CM": -0.0124, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0604},
    {"alpha": -8.750, "CL": -0.9087, "CD": 0.02387, "CDp": 0.01574, "CM": -0.0112, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0635},
    {"alpha": -8.500, "CL": -0.8890, "CD": 0.02252, "CDp": 0.01438, "CM": -0.0101, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0667},
    {"alpha": -8.250, "CL": -0.8689, "CD": 0.02174, "CDp": 0.01362, "CM": -0.0089, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0707},
    {"alpha": -8.000, "CL": -0.8481, "CD": 0.02113, "CDp": 0.01286, "CM": -0.0077, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0749},
    {"alpha": -7.750, "CL": -0.8300, "CD": 0.01996, "CDp": 0.01181, "CM": -0.0062, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0799},
    {"alpha": -7.500, "CL": -0.8096, "CD": 0.01936, "CDp": 0.01114, "CM": -0.0049, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0858},
    {"alpha": -7.250, "CL": -0.7919, "CD": 0.01844, "CDp": 0.01031, "CM": -0.0033, "Top_Xtr": 1.0000, "Bot_Xtr": 0.0925},
    {"alpha": -7.000, "CL": -0.7726, "CD": 0.01776, "CDp": 0.00961, "CM": -0.0018, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1005},
    {"alpha": -6.750, "CL": -0.7535, "CD": 0.01712, "CDp": 0.00900, "CM": -0.0003, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1104},
    {"alpha": -6.500, "CL": -0.7351, "CD": 0.01641, "CDp": 0.00837, "CM": 0.0013, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1222},
    {"alpha": -6.250, "CL": -0.7162, "CD": 0.01577, "CDp": 0.00781, "CM": 0.0028, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1373},
    {"alpha": -6.000, "CL": -0.6971, "CD": 0.01517, "CDp": 0.00729, "CM": 0.0043, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1563},
    {"alpha": -5.750, "CL": -0.6785, "CD": 0.01455, "CDp": 0.00683, "CM": 0.0058, "Top_Xtr": 1.0000, "Bot_Xtr": 0.1796},
    {"alpha": -5.500, "CL": -0.6592, "CD": 0.01400, "CDp": 0.00643, "CM": 0.0073, "Top_Xtr": 1.0000, "Bot_Xtr": 0.2077},
    {"alpha": -5.250, "CL": -0.6396, "CD": 0.01350, "CDp": 0.00609, "CM": 0.0086, "Top_Xtr": 1.0000, "Bot_Xtr": 0.2398},
    {"alpha": -5.000, "CL": -0.6195, "CD": 0.01306, "CDp": 0.00580, "CM": 0.0099, "Top_Xtr": 1.0000, "Bot_Xtr": 0.2743},
    {"alpha": -4.750, "CL": -0.5990, "CD": 0.01268, "CDp": 0.00556, "CM": 0.0111, "Top_Xtr": 1.0000, "Bot_Xtr": 0.3101},
    {"alpha": -4.500, "CL": -0.5783, "CD": 0.01234, "CDp": 0.00537, "CM": 0.0123, "Top_Xtr": 1.0000, "Bot_Xtr": 0.3456},
    {"alpha": -4.250, "CL": -0.5572, "CD": 0.01203, "CDp": 0.00521, "CM": 0.0134, "Top_Xtr": 1.0000, "Bot_Xtr": 0.3809},
    {"alpha": -4.000, "CL": -0.5358, "CD": 0.01176, "CDp": 0.00508, "CM": 0.0145, "Top_Xtr": 1.0000, "Bot_Xtr": 0.4160},
    {"alpha": -3.750, "CL": -0.5139, "CD": 0.01154, "CDp": 0.00498, "CM": 0.0155, "Top_Xtr": 1.0000, "Bot_Xtr": 0.4509},
    {"alpha": -3.500, "CL": -0.4915, "CD": 0.01134, "CDp": 0.00492, "CM": 0.0164, "Top_Xtr": 1.0000, "Bot_Xtr": 0.4856},
    {"alpha": -3.250, "CL": -0.4689, "CD": 0.01118, "CDp": 0.00489, "CM": 0.0172, "Top_Xtr": 1.0000, "Bot_Xtr": 0.5202},
    {"alpha": -3.000, "CL": -0.4463, "CD": 0.01104, "CDp": 0.00489, "CM": 0.0180, "Top_Xtr": 1.0000, "Bot_Xtr": 0.5545},
    {"alpha": -2.750, "CL": -0.4236, "CD": 0.01094, "CDp": 0.00493, "CM": 0.0188, "Top_Xtr": 1.0000, "Bot_Xtr": 0.5885},
    {"alpha": -2.500, "CL": -0.3942, "CD": 0.01086, "CDp": 0.00497, "CM": 0.0183, "Top_Xtr": 0.9979, "Bot_Xtr": 0.6248},
    {"alpha": -2.250, "CL": -0.3513, "CD": 0.01075, "CDp": 0.00504, "CM": 0.0152, "Top_Xtr": 0.9915, "Bot_Xtr": 0.6638},
    {"alpha": -2.000, "CL": -0.3085, "CD": 0.01066, "CDp": 0.00509, "CM": 0.0122, "Top_Xtr": 0.9843, "Bot_Xtr": 0.7011},
    {"alpha": -1.750, "CL": -0.2684, "CD": 0.01056, "CDp": 0.00512, "CM": 0.0100, "Top_Xtr": 0.9755, "Bot_Xtr": 0.7357},
    {"alpha": -1.500, "CL": -0.2255, "CD": 0.01048, "CDp": 0.00517, "CM": 0.0072, "Top_Xtr": 0.9686, "Bot_Xtr": 0.7684},
    {"alpha": -1.250, "CL": -0.1861, "CD": 0.01040, "CDp": 0.00519, "CM": 0.0054, "Top_Xtr": 0.9593, "Bot_Xtr": 0.7978},
    {"alpha": -1.000, "CL": -0.1454, "CD": 0.01034, "CDp": 0.00519, "CM": 0.0033, "Top_Xtr": 0.9509, "Bot_Xtr": 0.8255},
    {"alpha": -0.750, "CL": -0.1048, "CD": 0.01028, "CDp": 0.00521, "CM": 0.0015, "Top_Xtr": 0.9424, "Bot_Xtr": 0.8482},
    {"alpha": -0.500, "CL": -0.0700, "CD": 0.01025, "CDp": 0.00522, "CM": 0.0009, "Top_Xtr": 0.9304, "Bot_Xtr": 0.8696},
    {"alpha": -0.250, "CL": -0.0350, "CD": 0.01022, "CDp": 0.00521, "CM": 0.0004, "Top_Xtr": 0.9182, "Bot_Xtr": 0.8886},
    {"alpha": 0.000, "CL": 0.0000, "CD": 0.01020, "CDp": 0.00520, "CM": 0.0000, "Top_Xtr": 0.9046, "Bot_Xtr": 0.9047},
    {"alpha": 0.250, "CL": 0.0350, "CD": 0.01022, "CDp": 0.00521, "CM": -0.0004, "Top_Xtr": 0.8886, "Bot_Xtr": 0.9182},
    {"alpha": 0.500, "CL": 0.0700, "CD": 0.01025, "CDp": 0.00522, "CM": -0.0009, "Top_Xtr": 0.8696, "Bot_Xtr": 0.9304},
    {"alpha": 0.750, "CL": 0.1048, "CD": 0.01028, "CDp": 0.00521, "CM": -0.0015, "Top_Xtr": 0.8482, "Bot_Xtr": 0.9424},
    {"alpha": 1.000, "CL": 0.1454, "CD": 0.01034, "CDp": 0.00519, "CM": -0.0033, "Top_Xtr": 0.8255, "Bot_Xtr": 0.9510},
    {"alpha": 1.250, "CL": 0.1861, "CD": 0.01040, "CDp": 0.00518, "CM": -0.0054, "Top_Xtr": 0.7978, "Bot_Xtr": 0.9593},
    {"alpha": 1.500, "CL": 0.2255, "CD": 0.01048, "CDp": 0.00517, "CM": -0.0072, "Top_Xtr": 0.7684, "Bot_Xtr": 0.9686},
    {"alpha": 1.750, "CL": 0.2684, "CD": 0.01056, "CDp": 0.00512, "CM": -0.0100, "Top_Xtr": 0.7357, "Bot_Xtr": 0.9756},
    {"alpha": 2.000, "CL": 0.3085, "CD": 0.01066, "CDp": 0.00509, "CM": -0.0122, "Top_Xtr": 0.7012, "Bot_Xtr": 0.9843},
    {"alpha": 2.250, "CL": 0.3513, "CD": 0.01075, "CDp": 0.00504, "CM": -0.0152, "Top_Xtr": 0.6637, "Bot_Xtr": 0.9915},
    {"alpha": 2.500, "CL": 0.3942, "CD": 0.01085, "CDp": 0.00497, "CM": -0.0183, "Top_Xtr": 0.6249, "Bot_Xtr": 0.9979},
    {"alpha": 2.750, "CL": 0.4235, "CD": 0.01094, "CDp": 0.00492, "CM": -0.0188, "Top_Xtr": 0.5886, "Bot_Xtr": 1.0000},
    {"alpha": 3.000, "CL": 0.4462, "CD": 0.01104, "CDp": 0.00489, "CM": -0.0180, "Top_Xtr": 0.5545, "Bot_Xtr": 1.0000},
    {"alpha": 3.250, "CL": 0.4689, "CD": 0.01118, "CDp": 0.00489, "CM": -0.0172, "Top_Xtr": 0.5202, "Bot_Xtr": 1.0000},
    {"alpha": 3.500, "CL": 0.4915, "CD": 0.01134, "CDp": 0.00492, "CM": -0.0163, "Top_Xtr": 0.4857, "Bot_Xtr": 1.0000},
    {"alpha": 3.750, "CL": 0.5138, "CD": 0.01153, "CDp": 0.00498, "CM": -0.0155, "Top_Xtr": 0.4509, "Bot_Xtr": 1.0000},
    {"alpha": 4.000, "CL": 0.5357, "CD": 0.01176, "CDp": 0.00507, "CM": -0.0145, "Top_Xtr": 0.4160, "Bot_Xtr": 1.0000},
    {"alpha": 4.250, "CL": 0.5571, "CD": 0.01203, "CDp": 0.00521, "CM": -0.0134, "Top_Xtr": 0.3810, "Bot_Xtr": 1.0000},
    {"alpha": 4.500, "CL": 0.5782, "CD": 0.01234, "CDp": 0.00536, "CM": -0.0123, "Top_Xtr": 0.3456, "Bot_Xtr": 1.0000},
    {"alpha": 4.750, "CL": 0.5990, "CD": 0.01268, "CDp": 0.00556, "CM": -0.0111, "Top_Xtr": 0.3101, "Bot_Xtr": 1.0000},
    {"alpha": 5.000, "CL": 0.6195, "CD": 0.01306, "CDp": 0.00580, "CM": -0.0099, "Top_Xtr": 0.2744, "Bot_Xtr": 1.0000},
    {"alpha": 5.250, "CL": 0.6395, "CD": 0.01350, "CDp": 0.00609, "CM": -0.0086, "Top_Xtr": 0.2398, "Bot_Xtr": 1.0000},
    {"alpha": 5.500, "CL": 0.6591, "CD": 0.01400, "CDp": 0.00643, "CM": -0.0072, "Top_Xtr": 0.2077, "Bot_Xtr": 1.0000},
    {"alpha": 5.750, "CL": 0.6784, "CD": 0.01455, "CDp": 0.00683, "CM": -0.0058, "Top_Xtr": 0.1796, "Bot_Xtr": 1.0000},
    {"alpha": 6.000, "CL": 0.6971, "CD": 0.01516, "CDp": 0.00729, "CM": -0.0043, "Top_Xtr": 0.1563, "Bot_Xtr": 1.0000},
    {"alpha": 6.250, "CL": 0.7161, "CD": 0.01577, "CDp": 0.00781, "CM": -0.0028, "Top_Xtr": 0.1374, "Bot_Xtr": 1.0000},
    {"alpha": 6.500, "CL": 0.7351, "CD": 0.01641, "CDp": 0.00837, "CM": -0.0013, "Top_Xtr": 0.1223, "Bot_Xtr": 1.0000},
    {"alpha": 6.750, "CL": 0.7535, "CD": 0.01712, "CDp": 0.00900, "CM": 0.0003, "Top_Xtr": 0.1104, "Bot_Xtr": 1.0000},
    {"alpha": 7.000, "CL": 0.7725, "CD": 0.01776, "CDp": 0.00961, "CM": 0.0018, "Top_Xtr": 0.1005, "Bot_Xtr": 1.0000},
    {"alpha": 7.250, "CL": 0.7919, "CD": 0.01844, "CDp": 0.01031, "CM": 0.0033, "Top_Xtr": 0.0925, "Bot_Xtr": 1.0000},
    {"alpha": 7.500, "CL": 0.8096, "CD": 0.01936, "CDp": 0.01114, "CM": 0.0049, "Top_Xtr": 0.0858, "Bot_Xtr": 1.0000},
    {"alpha": 7.750, "CL": 0.8300, "CD": 0.01995, "CDp": 0.01181, "CM": 0.0062, "Top_Xtr": 0.0799, "Bot_Xtr": 1.0000},
    {"alpha": 8.000, "CL": 0.8481, "CD": 0.02112, "CDp": 0.01285, "CM": 0.0077, "Top_Xtr": 0.0749, "Bot_Xtr": 1.0000},
    {"alpha": 8.250, "CL": 0.8690, "CD": 0.02173, "CDp": 0.01362, "CM": 0.0089, "Top_Xtr": 0.0707, "Bot_Xtr": 1.0000},
    {"alpha": 8.500, "CL": 0.8890, "CD": 0.02251, "CDp": 0.01438, "CM": 0.0101, "Top_Xtr": 0.0667, "Bot_Xtr": 1.0000},
    {"alpha": 8.750, "CL": 0.9087, "CD": 0.02387, "CDp": 0.01574, "CM": 0.0112, "Top_Xtr": 0.0635, "Bot_Xtr": 1.0000},
    {"alpha": 9.000, "CL": 0.9291, "CD": 0.02472, "CDp": 0.01674, "CM": 0.0124, "Top_Xtr": 0.0604, "Bot_Xtr": 1.0000},
    {"alpha": 9.250, "CL": 0.9492, "CD": 0.02559, "CDp": 0.01762, "CM": 0.0135, "Top_Xtr": 0.0575, "Bot_Xtr": 1.0000},
    {"alpha": 9.500, "CL": 0.9693, "CD": 0.02734, "CDp": 0.01932, "CM": 0.0143, "Top_Xtr": 0.0550, "Bot_Xtr": 1.0000},
    {"alpha": 9.750, "CL": 0.9882, "CD": 0.02844, "CDp": 0.02068, "CM": 0.0156, "Top_Xtr": 0.0532, "Bot_Xtr": 1.0000},
    {"alpha": 10.000, "CL": 1.0067, "CD": 0.02966, "CDp": 0.02206, "CM": 0.0168, "Top_Xtr": 0.0511, "Bot_Xtr": 1.0000},
    {"alpha": 10.250, "CL": 1.0251, "CD": 0.03077, "CDp": 0.02323, "CM": 0.0179, "Top_Xtr": 0.0492, "Bot_Xtr": 1.0000},
    {"alpha": 10.500, "CL": 1.0445, "CD": 0.03262, "CDp": 0.02503, "CM": 0.0185, "Top_Xtr": 0.0474, "Bot_Xtr": 1.0000},
    {"alpha": 10.750, "CL": 1.0571, "CD": 0.03445, "CDp": 0.02718, "CM": 0.0203, "Top_Xtr": 0.0462, "Bot_Xtr": 1.0000},
    {"alpha": 11.000, "CL": 1.0681, "CD": 0.03636, "CDp": 0.02942, "CM": 0.0222, "Top_Xtr": 0.0451, "Bot_Xtr": 1.0000},
    {"alpha": 11.250, "CL": 1.0773, "CD": 0.03842, "CDp": 0.03177, "CM": 0.0241, "Top_Xtr": 0.0441, "Bot_Xtr": 1.0000},
    {"alpha": 11.500, "CL": 1.0860, "CD": 0.04035, "CDp": 0.03390, "CM": 0.0258, "Top_Xtr": 0.0430, "Bot_Xtr": 1.0000},
    {"alpha": 11.750, "CL": 1.0969, "CD": 0.04194, "CDp": 0.03558, "CM": 0.0272, "Top_Xtr": 0.0419, "Bot_Xtr": 1.0000},
    {"alpha": 12.000, "CL": 1.1068, "CD": 0.04366, "CDp": 0.03735, "CM": 0.0286, "Top_Xtr": 0.0410, "Bot_Xtr": 1.0000},
    {"alpha": 12.250, "CL": 1.1101, "CD": 0.04727, "CDp": 0.04108, "CM": 0.0298, "Top_Xtr": 0.0401, "Bot_Xtr": 1.0000},
    {"alpha": 12.500, "CL": 1.0959, "CD": 0.04960, "CDp": 0.04372, "CM": 0.0334, "Top_Xtr": 0.0399, "Bot_Xtr": 1.0000},
    {"alpha": 12.750, "CL": 1.0790, "CD": 0.05258, "CDp": 0.04700, "CM": 0.0360, "Top_Xtr": 0.0397, "Bot_Xtr": 1.0000},
    {"alpha": 13.000, "CL": 1.0599, "CD": 0.05626, "CDp": 0.05095, "CM": 0.0372, "Top_Xtr": 0.0396, "Bot_Xtr": 1.0000},
    {"alpha": 13.250, "CL": 1.0385, "CD": 0.06069, "CDp": 0.05564, "CM": 0.0369, "Top_Xtr": 0.0396, "Bot_Xtr": 1.0000},
    {"alpha": 13.500, "CL": 1.0150, "CD": 0.06603, "CDp": 0.06120, "CM": 0.0352, "Top_Xtr": 0.0396, "Bot_Xtr": 1.0000},
    {"alpha": 13.750, "CL": 0.9898, "CD": 0.07237, "CDp": 0.06774, "CM": 0.0319, "Top_Xtr": 0.0398, "Bot_Xtr": 1.0000},
    {"alpha": 14.000, "CL": 0.9632, "CD": 0.07976, "CDp": 0.07532, "CM": 0.0275, "Top_Xtr": 0.0399, "Bot_Xtr": 1.0000},
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
        result["max_thickness_fraction_of_chord"] = 0.12
        return result
    if alpha_req >= alpha_list[-1]:
        result = polar_data[-1].copy()
        result["CL_max"] = CL_max
        result["CM_max"] = CM_max
        result["CL_max_pos"] = CL_max
        result["CL_max_neg"] = CL_min
        result["max_thickness_fraction_of_chord"] = 0.12
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
        "max_thickness_fraction_of_chord": 0.12,
    }
    return result