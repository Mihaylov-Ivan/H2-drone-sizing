"""Mission/transition time-domain VTOL <-> wing-borne transition model for H2AIR-4+1.

Fidelity boundary
-----------------
This module is a 2-D point-mass mission/transition model. It resolves horizontal
and vertical translation, wing lift/drag, fixed-axis lift and pusher thrust,
motor lag, pitch scheduling, altitude control, propeller folding state, power
and transition abort gates. It deliberately does *not* claim 6-DOF flight-control
fidelity; rigid-body rotational dynamics and control-surface allocation are a
6-DOF/control task.

Coordinates: x forward, z upward. Positive pitch is nose-up.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from math import atan2, cos, degrees, exp, pi, radians, sin, sqrt
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from .aero_propulsion import (
    G,
    INCH_TO_M,
    EPS,
    _prop_coefficients,
    calibrated_drag_constants,
    lift_prop_drag_CdA_each_m2,
    stall_speed_mps,
)


@dataclass
class TransitionState:
    t_s: float
    x_m: float
    z_m: float
    vx_mps: float
    vz_mps: float
    pitch_rad: float
    lift_thrust_N: float
    pusher_rpm: float
    low_lift_timer_s: float = 0.0


@dataclass(frozen=True)
class AeroForces:
    airspeed_mps: float
    flight_path_angle_rad: float
    alpha_rad: float
    CL: float
    lift_N: float
    drag_N: float
    parasite_drag_N: float
    induced_drag_N: float
    lift_prop_drag_N: float
    lift_prop_drag_state: str
    stall_limited: bool


@dataclass(frozen=True)
class PusherPoint:
    rpm: float
    thrust_N: float
    shaft_power_W: float
    electrical_power_W: float
    advance_ratio_J: float
    Ct: float
    Cp: float
    tip_mach: float
    power_limited: bool


@dataclass(frozen=True)
class TransitionSample:
    time_s: float
    x_m: float
    altitude_m: float
    vx_mps: float
    vz_mps: float
    airspeed_mps: float
    pitch_deg: float
    flight_path_deg: float
    alpha_deg: float
    CL: float
    wing_lift_N: float
    total_drag_N: float
    lift_prop_drag_N: float
    lift_thrust_N: float
    lift_thrust_fraction_weight: float
    pusher_thrust_N: float
    pusher_rpm: float
    lift_power_W: float
    pusher_power_W: float
    auxiliary_power_W: float
    total_bus_power_W: float
    vertical_accel_mps2: float
    horizontal_accel_mps2: float
    lift_prop_drag_state: str
    pusher_power_limited: bool
    lift_thrust_saturated: bool

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TransitionResult:
    direction: str
    success: bool
    handover_reason: str
    duration_s: float
    distance_m: float
    start_altitude_m: float
    end_altitude_m: float
    minimum_altitude_m: float
    maximum_altitude_m: float
    altitude_loss_m: float
    maximum_sink_rate_mps: float
    maximum_climb_rate_mps: float
    maximum_abs_alpha_deg: float
    maximum_bus_power_W: float
    maximum_lift_power_W: float
    maximum_pusher_power_W: float
    transition_energy_Wh: float
    final_speed_mps: float
    final_vertical_speed_mps: float
    final_pitch_deg: float
    final_wing_lift_fraction_weight: float
    final_lift_thrust_fraction_weight: float
    handover_stall_ratio: float
    abort_reasons: Tuple[str, ...]
    warnings: Tuple[str, ...]
    samples: Tuple[TransitionSample, ...]

    def to_dict(self, *, include_samples: bool = True) -> Dict[str, Any]:
        d = asdict(self)
        if not include_samples:
            d.pop("samples", None)
        return d


def _rate_limited_first_order(current: float, command: float, tau_s: float, rate_limit: float, dt_s: float) -> float:
    if tau_s <= 0.0:
        raw_rate = (command - current) / max(dt_s, EPS)
    else:
        raw_rate = (command - current) / tau_s
    rate = max(-rate_limit, min(rate_limit, raw_rate))
    return current + rate * dt_s


def _wing_aero(
    cfg4: Mapping[str, Any],
    cfg5: Mapping[str, Any],
    state: TransitionState,
    *,
    wing_area_m2: float,
    wing_span_m: float,
    lift_prop_diameter_in: float,
    lift_prop_drag_factor: float,
    lift_prop_drag_state: str,
) -> AeroForces:
    rho = float(cfg4["atmosphere"]["rho_kg_m3"])
    V = sqrt(state.vx_mps * state.vx_mps + state.vz_mps * state.vz_mps)
    gamma = atan2(state.vz_mps, max(state.vx_mps, 1e-6)) if V > 1e-5 else 0.0
    alpha = state.pitch_rad - gamma
    aero5 = cfg5["aerodynamics"]
    cl_alpha = float(aero5["CL_alpha_per_rad"])
    alpha0 = radians(float(aero5["zero_lift_alpha_deg"]))
    cl_raw = cl_alpha * (alpha - alpha0)
    cl_min = float(aero5["CL_min"])
    cl_max = float(aero5["CL_max"])
    stall_limited = cl_raw < cl_min or cl_raw > cl_max
    CL = max(cl_min, min(cl_max, cl_raw))

    q = 0.5 * rho * V * V
    L = q * wing_area_m2 * CL

    constants = calibrated_drag_constants(cfg4)
    ref = cfg4["reference_calibration"]
    ac4 = cfg4["aircraft"]
    S_ref = float(ac4["wing_area_m2"])
    q_ref = 0.5 * rho * float(ref["reference_speed_mps"]) ** 2
    D_wing_profile_ref = constants["reference_wing_profile_drag_N"]
    wing_profile_CD = D_wing_profile_ref / max(q_ref * S_ref, EPS)
    fixed_other_CdA = max(
        0.0,
        constants["non_lift_prop_parasite_CdA_m2"] - D_wing_profile_ref / max(q_ref, EPS),
    )
    D_parasite = q * fixed_other_CdA + q * wing_area_m2 * wing_profile_CD

    n_lift = int(cfg4["lift_system"]["number_of_propellers"])
    # We use the Aero/propulsion windmilling geometric CdA as the reference for an active rotor,
    # then blend it toward the folded CdA as lift thrust unloads.
    CdA_wind_each = lift_prop_drag_CdA_each_m2(cfg4, lift_prop_diameter_in, "windmilling")
    CdA_fold_each = lift_prop_drag_CdA_each_m2(cfg4, lift_prop_diameter_in, "folded")
    CdA_each = CdA_fold_each + max(0.0, lift_prop_drag_factor) * max(0.0, CdA_wind_each - CdA_fold_each)
    D_prop = q * n_lift * CdA_each

    AR = wing_span_m * wing_span_m / max(wing_area_m2, EPS)
    e = constants["equivalent_oswald_efficiency"]
    k = 1.0 / (pi * max(AR, EPS) * max(e, EPS))
    D_induced = k * L * L / max(q * wing_area_m2, EPS) if q > 1e-6 else 0.0
    D = D_parasite + D_prop + D_induced
    return AeroForces(
        airspeed_mps=V,
        flight_path_angle_rad=gamma,
        alpha_rad=alpha,
        CL=CL,
        lift_N=L,
        drag_N=D,
        parasite_drag_N=D_parasite,
        induced_drag_N=D_induced,
        lift_prop_drag_N=D_prop,
        lift_prop_drag_state=lift_prop_drag_state,
        stall_limited=stall_limited,
    )


def _lift_power_W(cfg4: Mapping[str, Any], cfg5: Mapping[str, Any], thrust_N: float, vertical_speed_mps: float, diameter_in: float) -> float:
    if thrust_N <= 1e-9:
        return 0.0
    rho = float(cfg4["atmosphere"]["rho_kg_m3"])
    n = int(cfg4["lift_system"]["number_of_propellers"])
    D = diameter_in * INCH_TO_M
    A = n * pi * (0.5 * D) ** 2
    vh = sqrt(thrust_N / max(2.0 * rho * A, EPS))
    # Axial momentum-theory climb/descent screen. Vortex-ring-state behavior is
    # not modeled; reverse-transition gates limit sink rate to stay outside that use.
    induced_term = 0.5 * vertical_speed_mps + sqrt((0.5 * vertical_speed_mps) ** 2 + vh * vh)
    ideal = thrust_N * max(0.0, induced_term)
    eta = float(cfg5["propulsion"]["lift_effective_efficiency"])
    return ideal / max(eta, EPS)


def _pusher_at_rpm(cfg4: Mapping[str, Any], cfg5: Mapping[str, Any], *, diameter_in: float, speed_mps: float, rpm: float) -> PusherPoint:
    if rpm <= 1e-9:
        return PusherPoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False)
    ps4 = cfg4["pusher_screen"]
    rho = float(cfg4["atmosphere"]["rho_kg_m3"])
    a = float(cfg4["atmosphere"]["speed_of_sound_mps"])
    D = diameter_in * INCH_TO_M
    n = rpm / 60.0
    J = max(0.0, speed_mps) / max(n * D, EPS)
    Ct, Cp = _prop_coefficients(ps4["generic_prop_map"], J)
    thrust = Ct * rho * n * n * D**4
    shaft = Cp * rho * n**3 * D**5
    eta_drive = float(ps4["motor_efficiency_assumption"]) * float(ps4["esc_efficiency_assumption"])
    elec = shaft / max(eta_drive, EPS)
    max_elec = float(cfg5["propulsion"]["pusher_max_electrical_power_W"])
    limited = elec > max_elec + 1e-9
    if limited and elec > EPS:
        # Scale torque/power and use a conservative linear thrust-power screening
        # relation at the same advance ratio. A selected prop map will replace this.
        ratio = max_elec / elec
        shaft *= ratio
        elec = max_elec
        thrust *= ratio
    tip = sqrt((pi * D * n) ** 2 + max(0.0, speed_mps) ** 2) / max(a, EPS)
    return PusherPoint(rpm, thrust, shaft, elec, J, Ct, Cp, tip, limited)


def _rpm_for_required_thrust(cfg4: Mapping[str, Any], cfg5: Mapping[str, Any], *, diameter_in: float, speed_mps: float, required_thrust_N: float) -> PusherPoint:
    if required_thrust_N <= 1e-6:
        return _pusher_at_rpm(cfg4, cfg5, diameter_in=diameter_in, speed_mps=speed_mps, rpm=0.0)
    p5 = cfg5["propulsion"]
    rpm_min = float(p5["pusher_min_rpm"])
    rpm_max = min(float(p5["pusher_max_rpm"]), float(cfg4["pusher_screen"]["rpm_max"]))
    step = float(p5["pusher_rpm_step"])
    best: Optional[PusherPoint] = None
    rpm = rpm_min
    while rpm <= rpm_max + EPS:
        pt = _pusher_at_rpm(cfg4, cfg5, diameter_in=diameter_in, speed_mps=speed_mps, rpm=rpm)
        if pt.thrust_N >= required_thrust_N - 1e-9:
            best = pt
            break
        rpm += step
    if best is None:
        best = _pusher_at_rpm(cfg4, cfg5, diameter_in=diameter_in, speed_mps=speed_mps, rpm=rpm_max)
    return best


def _pitch_update(state: TransitionState, cmd_deg: float, tau_s: float, rate_deg_s: float, dt_s: float) -> None:
    state.pitch_rad = _rate_limited_first_order(
        state.pitch_rad,
        radians(cmd_deg),
        tau_s,
        radians(rate_deg_s),
        dt_s,
    )


def _lift_drag_blend(cfg4: Mapping[str, Any], cfg5: Mapping[str, Any], state: TransitionState, mass_kg: float, dt_s: float) -> Tuple[float, str]:
    aero5 = cfg5["aerodynamics"]
    frac = max(0.0, state.lift_thrust_N / max(mass_kg * G, EPS))
    threshold = float(aero5["folding_start_thrust_fraction"])
    if frac <= threshold:
        state.low_lift_timer_s += dt_s
    else:
        state.low_lift_timer_s = 0.0
    if state.low_lift_timer_s >= float(aero5["folding_delay_s"]):
        return 0.0, "folded"
    active_rel = float(aero5["active_lift_prop_drag_factor_relative_windmilling"])
    # sqrt gives a smooth but deliberately conservative drag reduction as thrust unloads.
    return active_rel * sqrt(min(1.0, frac)), "powered_transition"


def _sample(
    state: TransitionState,
    aero: AeroForces,
    *,
    mass_kg: float,
    lift_power_W: float,
    pusher: PusherPoint,
    aux_W: float,
    ax: float,
    az: float,
    lift_saturated: bool,
) -> TransitionSample:
    W = mass_kg * G
    return TransitionSample(
        time_s=state.t_s,
        x_m=state.x_m,
        altitude_m=state.z_m,
        vx_mps=state.vx_mps,
        vz_mps=state.vz_mps,
        airspeed_mps=aero.airspeed_mps,
        pitch_deg=degrees(state.pitch_rad),
        flight_path_deg=degrees(aero.flight_path_angle_rad),
        alpha_deg=degrees(aero.alpha_rad),
        CL=aero.CL,
        wing_lift_N=aero.lift_N,
        total_drag_N=aero.drag_N,
        lift_prop_drag_N=aero.lift_prop_drag_N,
        lift_thrust_N=state.lift_thrust_N,
        lift_thrust_fraction_weight=state.lift_thrust_N / max(W, EPS),
        pusher_thrust_N=pusher.thrust_N,
        pusher_rpm=pusher.rpm,
        lift_power_W=lift_power_W,
        pusher_power_W=pusher.electrical_power_W,
        auxiliary_power_W=aux_W,
        total_bus_power_W=lift_power_W + pusher.electrical_power_W + aux_W,
        vertical_accel_mps2=az,
        horizontal_accel_mps2=ax,
        lift_prop_drag_state=aero.lift_prop_drag_state,
        pusher_power_limited=pusher.power_limited,
        lift_thrust_saturated=lift_saturated,
    )


def _summarize(
    direction: str,
    samples: Sequence[TransitionSample],
    *,
    mass_kg: float,
    success: bool,
    handover_reason: str,
    abort_reasons: Sequence[str],
    warnings: Sequence[str],
    stall_speed: float,
) -> TransitionResult:
    if not samples:
        raise ValueError("transition produced no samples")
    first, last = samples[0], samples[-1]
    min_alt = min(s.altitude_m for s in samples)
    max_alt = max(s.altitude_m for s in samples)
    energy = 0.0
    for a, b in zip(samples, samples[1:]):
        energy += a.total_bus_power_W * max(0.0, b.time_s - a.time_s) / 3600.0
    return TransitionResult(
        direction=direction,
        success=success,
        handover_reason=handover_reason,
        duration_s=last.time_s - first.time_s,
        distance_m=last.x_m - first.x_m,
        start_altitude_m=first.altitude_m,
        end_altitude_m=last.altitude_m,
        minimum_altitude_m=min_alt,
        maximum_altitude_m=max_alt,
        altitude_loss_m=max(0.0, first.altitude_m - min_alt),
        maximum_sink_rate_mps=max(0.0, -min(s.vz_mps for s in samples)),
        maximum_climb_rate_mps=max(s.vz_mps for s in samples),
        maximum_abs_alpha_deg=max(abs(s.alpha_deg) for s in samples),
        maximum_bus_power_W=max(s.total_bus_power_W for s in samples),
        maximum_lift_power_W=max(s.lift_power_W for s in samples),
        maximum_pusher_power_W=max(s.pusher_power_W for s in samples),
        transition_energy_Wh=energy,
        final_speed_mps=last.airspeed_mps,
        final_vertical_speed_mps=last.vz_mps,
        final_pitch_deg=last.pitch_deg,
        final_wing_lift_fraction_weight=last.wing_lift_N / max(mass_kg * G, EPS),
        final_lift_thrust_fraction_weight=last.lift_thrust_fraction_weight,
        handover_stall_ratio=last.airspeed_mps / max(stall_speed, EPS),
        abort_reasons=tuple(sorted(set(abort_reasons))),
        warnings=tuple(sorted(set(warnings))),
        samples=tuple(samples),
    )



def simulate_forward_transition(
    cfg4: Mapping[str, Any],
    cfg5: Mapping[str, Any],
    *,
    mass_kg: Optional[float] = None,
    wing_area_m2: Optional[float] = None,
    wing_span_m: Optional[float] = None,
    lift_prop_diameter_in: Optional[float] = None,
    pusher_prop_diameter_in: Optional[float] = None,
    available_lift_thrust_per_motor_N: Optional[float] = None,
) -> TransitionResult:
    ref = cfg5["reference_case"]
    ft = cfg5["forward_transition"]
    mass = float(ref["mass_kg"] if mass_kg is None else mass_kg)
    S = float(ref["wing_area_m2"] if wing_area_m2 is None else wing_area_m2)
    b = float(ref["wing_span_m"] if wing_span_m is None else wing_span_m)
    Dlift = float(ref["lift_prop_diameter_in"] if lift_prop_diameter_in is None else lift_prop_diameter_in)
    Dpush = float(ref["pusher_prop_diameter_in"] if pusher_prop_diameter_in is None else pusher_prop_diameter_in)
    Tmax_each = float(ref["lift_thrust_available_per_motor_N"] if available_lift_thrust_per_motor_N is None else available_lift_thrust_per_motor_N)
    Tmax = int(cfg4["lift_system"]["number_of_propellers"]) * Tmax_each
    dt = float(ft["dt_s"])
    target_speed = float(ft["target_speed_mps"])
    W = mass * G
    state = TransitionState(0.0, 0.0, float(ref["initial_altitude_m"]), 0.0, 0.0, 0.0, min(W, Tmax), 0.0)
    start_alt = state.z_m
    samples: List[TransitionSample] = []
    abort: List[str] = []
    warnings: List[str] = []
    handover_time: Optional[float] = None
    success = False
    reason = "maximum duration reached"
    aux = float(cfg4["aircraft"].get("continuous_auxiliary_power_W", 0.0))
    stall = stall_speed_mps(cfg4, mass, S)
    required_handover_speed = float(ft["handover_stall_ratio"]) * stall
    pusher_rpm_actual = 0.0

    while state.t_s <= float(ft["max_duration_s"]) + 1e-9:
        # Pitch schedule: establish angle of attack while accelerating, then hold near
        # the wing-borne trim screen. Pitch is kinematic here; 6-DOF/control closes moments.
        if state.vx_mps < 0.25 * target_speed:
            pitch_cmd = float(ft["pitch_initial_deg"])
        elif state.vx_mps < 0.85 * target_speed:
            pitch_cmd = float(ft["pitch_main_deg"])
        else:
            pitch_cmd = float(ft["pitch_final_deg"])
            if bool(ft.get("auto_final_pitch_for_handover", False)):
                rho = float(cfg4["atmosphere"]["rho_kg_m3"])
                q_target = 0.5 * rho * target_speed * target_speed
                required_CL = (float(ft["handover_min_wing_lift_fraction"]) * W) / max(q_target * S, EPS)
                a5 = cfg5["aerodynamics"]
                required_alpha = radians(float(a5["zero_lift_alpha_deg"])) + required_CL / max(float(a5["CL_alpha_per_rad"]), EPS)
                pitch_cmd = max(pitch_cmd, min(float(ft.get("auto_final_pitch_max_deg", 10.0)), degrees(required_alpha)))
        _pitch_update(state, pitch_cmd, float(ft["pitch_time_constant_s"]), float(ft["pitch_rate_limit_deg_s"]), dt)

        drag_factor, drag_state = _lift_drag_blend(cfg4, cfg5, state, mass, dt)
        aero = _wing_aero(cfg4, cfg5, state, wing_area_m2=S, wing_span_m=b,
                          lift_prop_diameter_in=Dlift, lift_prop_drag_factor=drag_factor,
                          lift_prop_drag_state=drag_state)
        gamma = aero.flight_path_angle_rad

        # Forward speed controller. Feed-forward acceleration fades to zero near target.
        speed_error = max(0.0, target_speed - state.vx_mps)
        a_des = min(float(ft["target_forward_accel_mps2"]), 0.35 * speed_error)
        drag_x = aero.drag_N * cos(gamma)
        lift_x = -aero.lift_N * sin(gamma)
        lift_motor_x = -state.lift_thrust_N * sin(state.pitch_rad)
        required_pusher_x = max(0.0, mass * a_des + drag_x - lift_x - lift_motor_x)
        cos_pitch = max(0.2, cos(state.pitch_rad))
        pusher_cmd = _rpm_for_required_thrust(cfg4, cfg5, diameter_in=Dpush,
                                               speed_mps=max(0.0, state.vx_mps),
                                               required_thrust_N=required_pusher_x / cos_pitch)
        pusher_rpm_actual = _rate_limited_first_order(
            pusher_rpm_actual, pusher_cmd.rpm,
            float(cfg5["propulsion"]["pusher_motor_time_constant_s"]),
            20000.0, dt,
        )
        pusher = _pusher_at_rpm(cfg4, cfg5, diameter_in=Dpush, speed_mps=max(0.0, state.vx_mps), rpm=pusher_rpm_actual)

        # Altitude-hold controller determines how quickly vertical lift can unload.
        z_err = start_alt - state.z_m
        az_cmd = float(ft["altitude_kp_s2"]) * z_err - float(ft["vertical_speed_kd_s"]) * state.vz_mps
        az_lim = float(ft["max_vertical_accel_command_mps2"])
        az_cmd = max(-az_lim, min(az_lim, az_cmd))
        wing_z = aero.lift_N * cos(gamma)
        drag_z = -aero.drag_N * sin(gamma)
        pusher_z = pusher.thrust_N * sin(state.pitch_rad)
        desired_lift_z = mass * (G + az_cmd) - wing_z - drag_z - pusher_z
        lift_cmd = max(0.0, desired_lift_z / max(cos(state.pitch_rad), 0.2))
        lift_saturated = lift_cmd > Tmax + 1e-9
        lift_cmd = min(Tmax, lift_cmd)
        state.lift_thrust_N = _rate_limited_first_order(
            state.lift_thrust_N, lift_cmd,
            float(cfg5["propulsion"]["lift_motor_thrust_time_constant_s"]),
            Tmax / max(float(cfg5["propulsion"]["lift_motor_thrust_time_constant_s"]), dt),
            dt,
        )

        # Recompute forces after actuator update using current aero.
        Fx = (
            pusher.thrust_N * cos(state.pitch_rad)
            - state.lift_thrust_N * sin(state.pitch_rad)
            - aero.drag_N * cos(gamma)
            - aero.lift_N * sin(gamma)
        )
        Fz = (
            pusher.thrust_N * sin(state.pitch_rad)
            + state.lift_thrust_N * cos(state.pitch_rad)
            - aero.drag_N * sin(gamma)
            + aero.lift_N * cos(gamma)
            - W
        )
        ax = Fx / mass
        az = Fz / mass
        lift_power = _lift_power_W(cfg4, cfg5, state.lift_thrust_N, state.vz_mps, Dlift)
        sample = _sample(state, aero, mass_kg=mass, lift_power_W=lift_power, pusher=pusher,
                         aux_W=aux, ax=ax, az=az, lift_saturated=lift_saturated)
        samples.append(sample)

        if pusher.power_limited:
            warnings.append("pusher_power_limit_active")
        if lift_saturated:
            warnings.append("lift_thrust_saturation_occurred")
        if abs(sample.alpha_deg) > float(ft["max_allowed_alpha_deg"]):
            warnings.append("alpha_exceeds_transition_screen")

        alt_loss = max(0.0, start_alt - state.z_m)
        if alt_loss > float(ft["max_allowed_altitude_loss_m"]):
            abort.append("forward transition altitude-loss limit exceeded")
        if state.vz_mps < -float(ft["max_allowed_sink_rate_mps"]):
            abort.append("forward transition sink-rate limit exceeded")

        wing_frac = aero.lift_N / max(W, EPS)
        lift_frac = state.lift_thrust_N / max(W, EPS)
        handover_ok = (
            aero.airspeed_mps >= required_handover_speed
            and wing_frac >= float(ft["handover_min_wing_lift_fraction"])
            and lift_frac <= float(ft["handover_max_lift_thrust_fraction"])
            and abs(state.vz_mps) <= float(ft["handover_max_abs_vertical_speed_mps"])
        )
        if handover_ok and handover_time is None:
            handover_time = state.t_s
            reason = "wing-borne handover criteria achieved"
        if handover_time is not None and state.t_s - handover_time >= float(ft["settle_time_after_handover_s"]):
            success = not abort
            break
        if abort:
            reason = abort[0]
            break

        # Semi-implicit point-mass integration; small dt is used and 6-DOF/control will
        # replace this with the full rigid-body RK4 simulation.
        state.vx_mps = max(0.0, state.vx_mps + ax * dt)
        state.vz_mps += az * dt
        state.x_m += state.vx_mps * dt
        state.z_m += state.vz_mps * dt
        state.t_s += dt

    if handover_time is None and not abort:
        abort.append("forward transition did not achieve handover criteria before timeout")
        reason = abort[-1]
    return _summarize("forward", samples, mass_kg=mass, success=success, handover_reason=reason,
                      abort_reasons=abort, warnings=warnings, stall_speed=stall)


def simulate_reverse_transition(
    cfg4: Mapping[str, Any],
    cfg5: Mapping[str, Any],
    *,
    mass_kg: Optional[float] = None,
    wing_area_m2: Optional[float] = None,
    wing_span_m: Optional[float] = None,
    lift_prop_diameter_in: Optional[float] = None,
    pusher_prop_diameter_in: Optional[float] = None,
    available_lift_thrust_per_motor_N: Optional[float] = None,
    entry_altitude_m: Optional[float] = None,
) -> TransitionResult:
    ref = cfg5["reference_case"]
    rt = cfg5["reverse_transition"]
    mass = float(ref["mass_kg"] if mass_kg is None else mass_kg)
    S = float(ref["wing_area_m2"] if wing_area_m2 is None else wing_area_m2)
    b = float(ref["wing_span_m"] if wing_span_m is None else wing_span_m)
    Dlift = float(ref["lift_prop_diameter_in"] if lift_prop_diameter_in is None else lift_prop_diameter_in)
    Dpush = float(ref["pusher_prop_diameter_in"] if pusher_prop_diameter_in is None else pusher_prop_diameter_in)
    Tmax_each = float(ref["lift_thrust_available_per_motor_N"] if available_lift_thrust_per_motor_N is None else available_lift_thrust_per_motor_N)
    Tmax = int(cfg4["lift_system"]["number_of_propellers"]) * Tmax_each
    dt = float(rt["dt_s"])
    W = mass * G
    z0 = float(ref["initial_altitude_m"] if entry_altitude_m is None else entry_altitude_m)
    state = TransitionState(0.0, 0.0, z0, float(rt["entry_speed_mps"]), 0.0, radians(5.0), 0.0, 0.0)
    samples: List[TransitionSample] = []
    abort: List[str] = []
    warnings: List[str] = []
    capture_time: Optional[float] = None
    success = False
    reason = "maximum duration reached"
    aux = float(cfg4["aircraft"].get("continuous_auxiliary_power_W", 0.0))
    stall = stall_speed_mps(cfg4, mass, S)

    while state.t_s <= float(rt["max_duration_s"]) + 1e-9:
        Vx = state.vx_mps
        if Vx > float(rt["lift_spool_start_speed_mps"]):
            pitch_cmd = 6.0
        elif Vx > 8.0:
            pitch_cmd = float(rt["pitch_brake_deg"])
        else:
            pitch_cmd = float(rt["pitch_low_speed_deg"])
        _pitch_update(state, pitch_cmd, float(rt["pitch_time_constant_s"]), float(rt["pitch_rate_limit_deg_s"]), dt)

        drag_factor, drag_state = _lift_drag_blend(cfg4, cfg5, state, mass, dt)
        aero = _wing_aero(cfg4, cfg5, state, wing_area_m2=S, wing_span_m=b,
                          lift_prop_diameter_in=Dlift, lift_prop_drag_factor=drag_factor,
                          lift_prop_drag_state=drag_state)
        gamma = aero.flight_path_angle_rad
        pusher = _pusher_at_rpm(cfg4, cfg5, diameter_in=Dpush, speed_mps=max(0.0, Vx), rpm=0.0)

        # Altitude hold progressively spools lift rotors as wing lift decays. Before
        # the configured speed, lift motors remain off unless altitude/sink demands it.
        z_err = z0 - state.z_m
        az_cmd = float(rt["altitude_kp_s2"]) * z_err - float(rt["vertical_speed_kd_s"]) * state.vz_mps
        az_lim = float(rt["max_vertical_accel_command_mps2"])
        az_cmd = max(-az_lim, min(az_lim, az_cmd))
        wing_z = aero.lift_N * cos(gamma)
        drag_z = -aero.drag_N * sin(gamma)
        desired_lift_z = mass * (G + az_cmd) - wing_z - drag_z
        lift_cmd = max(0.0, desired_lift_z / max(cos(state.pitch_rad), 0.2))
        if Vx > float(rt["lift_spool_start_speed_mps"]) and state.vz_mps > -0.5:
            lift_cmd = 0.0
        lift_saturated = lift_cmd > Tmax + 1e-9
        lift_cmd = min(Tmax, lift_cmd)
        state.lift_thrust_N = _rate_limited_first_order(
            state.lift_thrust_N, lift_cmd,
            float(cfg5["propulsion"]["lift_motor_thrust_time_constant_s"]),
            Tmax / max(float(cfg5["propulsion"]["lift_motor_thrust_time_constant_s"]), dt),
            dt,
        )

        Fx = (
            -state.lift_thrust_N * sin(state.pitch_rad)
            - aero.drag_N * cos(gamma)
            - aero.lift_N * sin(gamma)
        )
        Fz = (
            state.lift_thrust_N * cos(state.pitch_rad)
            - aero.drag_N * sin(gamma)
            + aero.lift_N * cos(gamma)
            - W
        )
        ax = Fx / mass
        az = Fz / mass
        lift_power = _lift_power_W(cfg4, cfg5, state.lift_thrust_N, state.vz_mps, Dlift)
        sample = _sample(state, aero, mass_kg=mass, lift_power_W=lift_power, pusher=pusher,
                         aux_W=aux, ax=ax, az=az, lift_saturated=lift_saturated)
        samples.append(sample)

        if lift_saturated:
            warnings.append("lift_thrust_saturation_occurred")
        alt_loss = max(0.0, z0 - state.z_m)
        if alt_loss > float(rt["max_allowed_altitude_loss_m"]):
            abort.append("reverse transition altitude-loss limit exceeded")
        if state.vz_mps < -float(rt["max_allowed_sink_rate_mps"]):
            abort.append("reverse transition sink-rate limit exceeded")

        capture_ok = (
            aero.airspeed_mps <= float(rt["hover_capture_speed_mps"])
            and state.lift_thrust_N / max(W, EPS) >= float(rt["hover_capture_min_lift_thrust_fraction"])
            and abs(state.vz_mps) <= float(rt["hover_capture_max_abs_vertical_speed_mps"])
        )
        if capture_ok and capture_time is None:
            capture_time = state.t_s
            reason = "hover-capture criteria achieved"
        if capture_time is not None and state.t_s - capture_time >= float(rt["settle_time_after_capture_s"]):
            success = not abort
            break
        if abort:
            reason = abort[0]
            break

        state.vx_mps = max(0.0, state.vx_mps + ax * dt)
        state.vz_mps += az * dt
        state.x_m += state.vx_mps * dt
        state.z_m += state.vz_mps * dt
        state.t_s += dt

    if capture_time is None and not abort:
        abort.append("reverse transition did not achieve hover-capture criteria before timeout")
        reason = abort[-1]
    return _summarize("reverse", samples, mass_kg=mass, success=success, handover_reason=reason,
                      abort_reasons=abort, warnings=warnings, stall_speed=stall)
