# avionics.py

from dataclasses import dataclass
import math
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


def get_autopilot_database():
    """
    Representative autopilot boards with approximate mass and size.
    Mass values are board+case only (no GPS).
    """

    return [
        {
            "name": "Matek_F765_WING",
            "class": "small",
            "mass_kg": 0.026,         # 26 g
            "length_m": 0.054,
            "width_m": 0.036,
            "height_m": 0.013,
            "pwm_outputs": 12,
        },
        {
            "name": "Pixhawk4",
            "class": "standard",
            "mass_kg": 0.016,         # 15.8 g
            "length_m": 0.084,
            "width_m": 0.044,
            "height_m": 0.012,
            "pwm_outputs": 14,
        },
        {
            "name": "CubeOrangePlus_StandardSet",
            "class": "redundant",
            "mass_kg": 0.073,         # 73 g (cube + carrier)
            "length_m": 0.095,        # carrier footprint
            "width_m": 0.045,
            "height_m": 0.022,
            "pwm_outputs": 14,
        },
    ]


def select_autopilot(autopilot_class: str):
    """
    Pick an autopilot from the DB based on class string.
    Fallback to 'standard' if class not found.
    """
    db = get_autopilot_database()

    # exact match first
    for fc in db:
        if fc["class"] == autopilot_class:
            return fc

    # fallback: standard
    for fc in db:
        if fc["class"] == "standard":
            return fc

    # extreme fallback: first entry
    return db[0]


def estimate_single_esc_mass_kg(I_cont_A: float) -> float:
    """
    Very rough ESC mass model for single ESCs:

        m_ESC [kg] ≈ 0.0008 * I_cont_A

    Example:
        30 A -> 0.024 kg
        40 A -> 0.032 kg
        60 A -> 0.048 kg
    """
    return 0.0008 * I_cont_A


def estimate_avionics_and_electrics(W0_mass_kg: float,
                                    n_motors: int,
                                    esc_cont_current_A: float,
                                    autopilot_class: str,
                                    n_servos: int,
                                    inputs: Optional[Any] = None) -> AvionicsEstimate:
    """
    Bundle all avionics + low-voltage electronics into one estimate.

    Inputs:
        W0_mass_kg         : MTOW mass (from your sizing loop)
        n_motors           : number of propulsors (4 for your cruciform)
        esc_cont_current_A : assumed continuous A-rating per ESC
        autopilot_class    : 'small', 'standard', 'redundant'
        n_servos           : number of control servos

    Returns:
        AvionicsEstimate dataclass with component masses and total.
    """

    # 1) Autopilot
    fc = select_autopilot(autopilot_class)
    m_fc = fc["mass_kg"]

    # 2) Navigation & comms (fixed assumed masses)
    m_gps = 0.025   # 25 g M8N GPS+compass
    m_airspeed = 0.022  # 22 g MS4525DO kit
    m_rc_rx = 0.010     # 10 g for receiver + antenna
    m_telemetry = 0.020 # 20 g for 433/915 MHz telemetry radio

    # 3) Power module
    m_power_module = 0.040  # 40 g PM07-class board

    # 4) Servos
    m_servo_each = 0.012    # 12 g mini metal gear servo
    m_servos = n_servos * m_servo_each

    # 5) ESCs
    m_esc_each = estimate_single_esc_mass_kg(esc_cont_current_A)
    m_esc_total = n_motors * m_esc_each

    # 6) Misc wiring, regulators, connectors
    #    simple scaling with aircraft size:
    m_misc = 0.02 + 0.01 * W0_mass_kg
    m_bec = 0.0
    m_autopilot_powerbrick = 0.0

    # 7) Camera / imaging payload (default mass; CG from inputs.cg_x_camera in script)
    m_camera = 0.028

    # Apply hardcoded overrides whenever enabled and values are provided.
    if inputs is not None and getattr(inputs, "use_hardcoded_values", False):
        hard_autopilot = getattr(inputs, "hardcode_autopilot_mass", None)
        hard_rc_rx = getattr(inputs, "hardcode_rc_rx_mass", None)
        hard_telemetry = getattr(inputs, "hardcode_telemetry_mass", None)
        hard_power_module = getattr(inputs, "hardcode_power_module_mass", None)
        hard_servos = getattr(inputs, "hardcode_servos_mass", None)
        hard_esc = getattr(inputs, "hardcode_esc_mass", None)
        hard_misc = getattr(inputs, "hardcode_misc_electronics_mass", None)
        hard_bec = getattr(inputs, "hardcode_BEC_mass", None)
        hard_autopilot_powerbrick = getattr(inputs, "hardcode_autopilot_powerbrick_mass", None)
        hard_camera = getattr(inputs, "hardcode_camera_mass", None)
        hard_gps = getattr(inputs, "hardcode_gps_mass", None)
        hard_airspeed = getattr(inputs, "hardcode_airspeed_mass", None)

        if hard_autopilot is not None:
            m_fc = float(hard_autopilot)
        if hard_rc_rx is not None:
            m_rc_rx = float(hard_rc_rx)
        if hard_telemetry is not None:
            m_telemetry = float(hard_telemetry)
        if hard_power_module is not None:
            m_power_module = float(hard_power_module)
        if hard_servos is not None:
            m_servos = float(hard_servos)
        if hard_esc is not None:
            m_esc_total = float(hard_esc)
        if hard_misc is not None:
            m_misc = float(hard_misc)
        if hard_bec is not None:
            m_bec = float(hard_bec)
        if hard_autopilot_powerbrick is not None:
            m_autopilot_powerbrick = float(hard_autopilot_powerbrick)
        if hard_camera is not None:
            m_camera = float(hard_camera)
        if hard_gps is not None:
            m_gps = float(hard_gps)
        if hard_airspeed is not None:
            m_airspeed = float(hard_airspeed)

    total = (m_fc + m_gps + m_airspeed + m_rc_rx +
             m_telemetry + m_power_module + m_bec + m_autopilot_powerbrick + m_servos +
             m_esc_total + m_misc + m_camera)

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
    )
