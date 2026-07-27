# avionics.py

from dataclasses import dataclass
from typing import Any, Optional


@dataclass
class AvionicsEstimate:
    autopilot_name: str
    autopilot_mass_kg: float
    gps_mass_kg: float
    airspeed_mass_kg: float
    rc_rx_mass_kg: float
    telemetry_mass_kg: float
    power_module_mass_kg: float
    bec_mass_kg: float
    autopilot_powerbrick_mass_kg: float
    servos_mass_kg: float
    esc_mass_total_kg: float
    misc_electronics_mass_kg: float
    camera_mass_kg: float
    total_avionics_electrics_mass_kg: float
    lift_esc_mass_total_kg: float = 0.0
    cruise_esc_mass_total_kg: float = 0.0
    lift_esc_cont_current_A: float = 0.0
    cruise_esc_cont_current_A: float = 0.0


def get_autopilot_database():
    return [
        {
            "name": "Matek_F765_WING",
            "class": "small",
            "mass_kg": 0.026,
            "length_m": 0.054,
            "width_m": 0.036,
            "height_m": 0.013,
            "pwm_outputs": 12,
        },
        {
            "name": "Pixhawk4",
            "class": "standard",
            "mass_kg": 0.016,
            "length_m": 0.084,
            "width_m": 0.044,
            "height_m": 0.012,
            "pwm_outputs": 14,
        },
        {
            "name": "CubeOrangePlus_StandardSet",
            "class": "redundant",
            "mass_kg": 0.073,
            "length_m": 0.095,
            "width_m": 0.045,
            "height_m": 0.022,
            "pwm_outputs": 14,
        },
    ]


def select_autopilot(autopilot_class: str):
    db = get_autopilot_database()
    for fc in db:
        if fc["class"] == autopilot_class:
            return fc
    for fc in db:
        if fc["class"] == "standard":
            return fc
    return db[0]


def estimate_single_esc_mass_kg(I_cont_A: float) -> float:
    """Concept-level ESC mass correlation: m_ESC [kg] ~= 0.0008 I_cont [A]."""
    return 0.0008 * max(0.0, float(I_cont_A))


def estimate_avionics_and_electrics(
    W0_mass_kg: float,
    n_motors: int,
    esc_cont_current_A: float,
    autopilot_class: str,
    n_servos: int,
    inputs: Optional[Any] = None,
    *,
    n_lift_motors: Optional[int] = None,
    n_cruise_motors: Optional[int] = None,
    lift_esc_cont_current_A: Optional[float] = None,
    cruise_esc_cont_current_A: Optional[float] = None,
) -> AvionicsEstimate:
    """
    Estimate avionics and electrical mass.

    The legacy n_motors / esc_cont_current_A interface remains valid. For the new
    lift-plus-cruise configuration, optional lift/cruise ESC counts and currents allow
    the four lift ESCs and one cruise ESC to be sized independently.
    """
    fc = select_autopilot(autopilot_class)
    m_fc = float(fc["mass_kg"])
    m_gps = 0.025
    m_airspeed = 0.022
    m_rc_rx = 0.010
    m_telemetry = 0.020
    m_power_module = 0.040
    m_servos = max(0, int(n_servos)) * 0.012

    if n_lift_motors is not None and n_cruise_motors is not None:
        n_lift = max(0, int(n_lift_motors))
        n_cruise = max(0, int(n_cruise_motors))
        I_lift = float(lift_esc_cont_current_A if lift_esc_cont_current_A is not None else esc_cont_current_A)
        I_cruise = float(cruise_esc_cont_current_A if cruise_esc_cont_current_A is not None else esc_cont_current_A)
        m_lift_esc = n_lift * estimate_single_esc_mass_kg(I_lift)
        m_cruise_esc = n_cruise * estimate_single_esc_mass_kg(I_cruise)
        m_esc_total = m_lift_esc + m_cruise_esc
    else:
        I_lift = float(esc_cont_current_A)
        I_cruise = 0.0
        m_lift_esc = max(0, int(n_motors)) * estimate_single_esc_mass_kg(esc_cont_current_A)
        m_cruise_esc = 0.0
        m_esc_total = m_lift_esc

    # Larger hybrid aircraft has additional HV/LV harnessing and power-management wiring.
    m_misc = 0.035 + 0.012 * max(0.0, float(W0_mass_kg))
    m_bec = 0.0
    m_autopilot_powerbrick = 0.0
    m_camera = 0.028

    if inputs is not None and getattr(inputs, "use_hardcoded_values", False):
        overrides = {
            "hardcode_autopilot_mass": "m_fc",
            "hardcode_rc_rx_mass": "m_rc_rx",
            "hardcode_telemetry_mass": "m_telemetry",
            "hardcode_power_module_mass": "m_power_module",
            "hardcode_servos_mass": "m_servos",
            "hardcode_misc_electronics_mass": "m_misc",
            "hardcode_BEC_mass": "m_bec",
            "hardcode_autopilot_powerbrick_mass": "m_autopilot_powerbrick",
            "hardcode_camera_mass": "m_camera",
            "hardcode_gps_mass": "m_gps",
            "hardcode_airspeed_mass": "m_airspeed",
        }
        values = locals()
        for attr, local_name in overrides.items():
            val = getattr(inputs, attr, None)
            if val is not None:
                values[local_name] = float(val)
        m_fc = values["m_fc"]
        m_rc_rx = values["m_rc_rx"]
        m_telemetry = values["m_telemetry"]
        m_power_module = values["m_power_module"]
        m_servos = values["m_servos"]
        m_misc = values["m_misc"]
        m_bec = values["m_bec"]
        m_autopilot_powerbrick = values["m_autopilot_powerbrick"]
        m_camera = values["m_camera"]
        m_gps = values["m_gps"]
        m_airspeed = values["m_airspeed"]

        hard_esc = getattr(inputs, "hardcode_esc_mass", None)
        if hard_esc is not None:
            m_esc_total = float(hard_esc)
            # Preserve the calculated split ratio for reporting.
            split_denom = max(m_lift_esc + m_cruise_esc, 1e-12)
            m_lift_esc = m_esc_total * m_lift_esc / split_denom
            m_cruise_esc = m_esc_total - m_lift_esc

    total = (
        m_fc + m_gps + m_airspeed + m_rc_rx + m_telemetry + m_power_module
        + m_bec + m_autopilot_powerbrick + m_servos + m_esc_total + m_misc + m_camera
    )

    return AvionicsEstimate(
        autopilot_name=fc["name"],
        autopilot_mass_kg=m_fc,
        gps_mass_kg=m_gps,
        airspeed_mass_kg=m_airspeed,
        rc_rx_mass_kg=m_rc_rx,
        telemetry_mass_kg=m_telemetry,
        power_module_mass_kg=m_power_module,
        bec_mass_kg=m_bec,
        autopilot_powerbrick_mass_kg=m_autopilot_powerbrick,
        servos_mass_kg=m_servos,
        esc_mass_total_kg=m_esc_total,
        misc_electronics_mass_kg=m_misc,
        camera_mass_kg=m_camera,
        total_avionics_electrics_mass_kg=total,
        lift_esc_mass_total_kg=m_lift_esc,
        cruise_esc_mass_total_kg=m_cruise_esc,
        lift_esc_cont_current_A=I_lift,
        cruise_esc_cont_current_A=I_cruise,
    )
