# helper_function.py

import math
from typing import Optional, Dict, Any, List


# -----------------------------
# Battery ranking helpers
# -----------------------------

def _battery_fits_fuselage(batt: Dict[str, Any], fuselage_ID: float) -> bool:
    """
    Rectangle fits in a circle if diagonal <= diameter.
    Uses width/height (your original logic).
    """
    if fuselage_ID is None or fuselage_ID <= 0:
        return True
    diag = math.sqrt(float(batt["width_m"]) ** 2 + float(batt["height_m"]) ** 2)
    return diag <= float(fuselage_ID)


def _battery_volume_m3(batt: Dict[str, Any]) -> float:
    return float(batt["length_m"]) * float(batt["width_m"]) * float(batt["height_m"])


def _battery_sort_key(candidate: Dict[str, Any]) -> tuple:
    """
    Your priority:
      2) smallest voltage
      3) smallest weight
      4) smallest size
    "size" -> primary: volume, tie-breakers: max dimension, then diagonal
    """
    b = candidate["battery"]
    V = float(b["voltage_V"])
    m = float(b["mass_kg"])
    vol = _battery_volume_m3(b)
    max_dim = max(float(b["length_m"]), float(b["width_m"]), float(b["height_m"]))
    diag = math.sqrt(float(b["width_m"]) ** 2 + float(b["height_m"]) ** 2)
    return (V, m, vol, max_dim, diag, str(b.get("name", "")))


def rank_battery_candidates(
    required_power_W: float,
    required_endurance_min: float,
    battery_db: List[Dict[str, Any]],
    fuselage_ID: float,
    discharge_margin: float = 0.7,
    usable_energy_frac: float = 0.9,
) -> List[Dict[str, Any]]:
    """
    Returns battery candidates filtered and sorted per your priorities.

    Priority enforcement:
      1) meet endurance time  -> we filter to ONLY those that meet endurance if any exist
      2) smallest voltage
      3) smallest weight
      4) smallest size (volume, then max dim, then diagonal)
    """
    req_endurance_h = float(required_endurance_min) / 60.0
    candidates: List[Dict[str, Any]] = []

    for batt in battery_db:
        # 1) fit check
        if not _battery_fits_fuselage(batt, fuselage_ID):
            continue

        capacity_Ah = float(batt["capacity_mAh"]) / 1000.0
        V = float(batt["voltage_V"])

        # max continuous-ish power estimate from C rating
        P_max_W = V * capacity_Ah * float(batt["max_discharge_C"])

        # 2) power capability check with margin
        if float(required_power_W) > float(discharge_margin) * P_max_W:
            continue

        # usable energy for endurance
        usable_Wh = V * capacity_Ah * float(usable_energy_frac)
        endurance_h = usable_Wh / float(required_power_W) if required_power_W > 1e-9 else 0.0

        C_at_cruise = float(required_power_W) / (V * capacity_Ah) if (V * capacity_Ah) > 1e-9 else float("inf")
        meets_endurance = endurance_h >= req_endurance_h

        candidates.append({
            "battery": batt,
            "P_max_W": P_max_W,
            "usable_energy_Wh": usable_Wh,
            "endurance_h": endurance_h,
            "endurance_min": endurance_h * 60.0,
            "C_at_cruise": C_at_cruise,
            "meets_endurance": meets_endurance,
        })

    if not candidates:
        return []

    # Priority (1): meet endurance time
    any_meets = any(c["meets_endurance"] for c in candidates)
    if any_meets:
        candidates = [c for c in candidates if c["meets_endurance"]]

    # Priorities (2)-(4)
    candidates.sort(key=_battery_sort_key)
    return candidates


def select_battery(
    required_power_W: float,
    required_endurance_min: float,
    battery_db,
    fuselage_ID: float,
    discharge_margin: float = 0.7,
    usable_energy_frac: float = 0.9,
):
    """
    Backwards-compatible wrapper.
    Returns the best-ranked candidate (or None).
    """
    ranked = rank_battery_candidates(
        required_power_W=required_power_W,
        required_endurance_min=required_endurance_min,
        battery_db=battery_db,
        fuselage_ID=fuselage_ID,
        discharge_margin=discharge_margin,
        usable_energy_frac=usable_energy_frac,
    )
    return ranked[0] if ranked else None


# -----------------------------
# Propulsion selection
# -----------------------------

def select_propulsion(
    drag_cruise_N: float,
    cruise_power_elec_W: float,
    propulsion_db: List[Dict[str, Any]],
    n_propellers: int,
    thrust_margin: float,
    power_margin: float,
    battery_voltage_V: float,
) -> Optional[Dict[str, Any]]:
    """
    Your priority (prop side):
      5) smallest prop diameter (for the chosen electrical properties / battery voltage)
      6) meets thrust required
      7) meets power required

    Implementation:
      - filter by voltage window
      - sort by prop diameter ascending
      - return first combo that meets BOTH thrust & power
    """
    if n_propellers <= 0:
        return None

    required_thrust_total_N = float(drag_cruise_N) * float(thrust_margin)
    required_thrust_per_motor_N = required_thrust_total_N / float(n_propellers)

    required_power_total_W = float(cruise_power_elec_W) * float(power_margin)
    required_power_per_motor_W = required_power_total_W / float(n_propellers)

    Vb = float(battery_voltage_V)

    # Filter by voltage window first
    filtered: List[Dict[str, Any]] = []
    for combo in propulsion_db:
        vmin = float(combo.get("voltage_min_V", -1e9))
        vmax = float(combo.get("voltage_max_V",  1e9))
        if vmin <= Vb <= vmax:
            filtered.append(combo)

    if not filtered:
        return None

    # Priority (5): smallest prop diameter
    filtered.sort(key=lambda c: float(c.get("prop_diameter_in", 1e9)))

    for combo in filtered:
        max_T = float(combo["max_thrust_N"])
        max_P = float(combo["max_power_W"])

        meets_thrust = max_T >= required_thrust_per_motor_N
        meets_power = max_P >= required_power_per_motor_W

        if meets_thrust and meets_power:
            # simple current estimate at cruise sizing power
            est_current_A_per_motor = required_power_per_motor_W / max(Vb, 1e-9)

            return {
                "combo": combo,
                "combo_name": combo.get("name", ""),
                "n_propellers": int(n_propellers),
                "required_thrust_per_motor_N": required_thrust_per_motor_N,
                "required_power_per_motor_W": required_power_per_motor_W,
                "meets_thrust": True,
                "meets_power": True,
                "est_current_A_per_motor_at_cruise": est_current_A_per_motor,
            }

    return None
