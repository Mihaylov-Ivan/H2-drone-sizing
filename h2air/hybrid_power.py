"""Phase-2 hybrid hydrogen-electric power system for H2AIR-4+1.

This module intentionally separates:
  1. supplier/reference evidence,
  2. engineering assumptions needed for preliminary simulation, and
  3. requirements / design gates.

It is dependency-light and can be used as a standalone power-system model or
integrated into the existing lift-plus-cruise mission model.

Sign convention
---------------
- power demand is positive at the aircraft DC bus;
- fuel-cell power is positive when delivered to the bus;
- battery power is positive for discharge and negative for charging;
- battery current is positive for discharge and negative for charging.

The default application-reference configuration is *not* a claim that all
hardware parameters are validated.  Unknown supplier values remain explicit
and checks that depend on them are reported as NOT_EVALUATED rather than PASS.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from math import isfinite, sqrt
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

EPS = 1e-12
H2_LHV_KWH_PER_KG = 33.33


class EvidenceStatus(str, Enum):
    MEASURED = "measured"
    SUPPLIER_CONFIRMED = "supplier_confirmed"
    SUPPLIER_REPORTED_CONDITIONAL = "supplier_reported_conditional"
    DERIVED_FROM_SUPPLIER_REFERENCE = "derived_from_supplier_reference"
    ENGINEERING_ASSUMPTION = "engineering_assumption"
    PRELIMINARY_TARGET = "preliminary_target"
    PLACEHOLDER = "placeholder"
    OPEN = "open"


class GateStatus(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    WARNING = "WARNING"
    NOT_EVALUATED = "NOT_EVALUATED"


class EMSState(str, Enum):
    OFF = "OFF"
    STARTUP = "STARTUP"
    READY = "READY"
    PEAK_SUPPORT = "PEAK_SUPPORT"
    LOAD_FOLLOW = "LOAD_FOLLOW"
    SOC_RECOVERY = "SOC_RECOVERY"
    FC_DEGRADED = "FC_DEGRADED"
    BATTERY_ONLY = "BATTERY_ONLY"
    SAFE_LANDING = "SAFE_LANDING"
    ENERGY_EXHAUSTED = "ENERGY_EXHAUSTED"


@dataclass(frozen=True)
class SourceTaggedValue:
    value: Any
    status: str
    source: str
    note: str = ""


@dataclass
class FuelCellConfig:
    name: str = "Intelligent Energy IE-SOAR 800 reference"
    enabled: bool = True
    rated_net_power_W: float = 800.0
    rating_basis: str = "net_module_output"
    output_voltage_min_V: float = 24.0
    output_voltage_max_V: float = 48.0
    module_mass_kg: float = 1.45
    # Optional packaging split of module_mass_kg (stack + control box). Unused by EMS.
    stack_mass_kg: Optional[float] = None
    control_box_mass_kg: Optional[float] = None

    # Dynamic/control values below are not supported by the current source set.
    # They remain configurable engineering assumptions until supplier/test data replace them.
    max_continuous_fraction: float = 1.0
    min_stable_fraction: float = 0.10
    ramp_up_W_per_s: float = 100.0
    ramp_down_W_per_s: float = 150.0
    startup_time_s: float = 30.0
    ready_at_mission_start: bool = True
    preflight_power_W: float = 400.0

    # If the supplier rating is already net at the usable aircraft bus, leave at 1.0.
    # Enter measured external converter/distribution efficiency when known.
    bus_path_efficiency: float = 1.0

    # Optional efficiency map: (net FC electrical output W, net electrical efficiency on H2 LHV).
    # When absent, H2 consumption is based on HydrogenConfig.reference_net_energy_Wh.
    efficiency_map: Optional[List[Tuple[float, float]]] = None

    # Optional derating/fault inputs for scenario studies.
    derating_fraction: float = 1.0
    forced_fault_time_s: Optional[float] = None

    # Optional lumped thermal model. Leave unset until module thermal data are available.
    initial_temperature_C: float = 25.0
    maximum_temperature_C: Optional[float] = None
    thermal_capacitance_J_per_K: Optional[float] = None
    thermal_resistance_K_per_W: Optional[float] = None
    ambient_temperature_C: float = 25.0

    def validate(self) -> List[str]:
        e: List[str] = []
        if self.rated_net_power_W <= 0:
            e.append("fuel-cell rated_net_power_W must be > 0")
        if not (0 < self.max_continuous_fraction <= 1.0):
            e.append("fuel-cell max_continuous_fraction must be in (0,1]")
        if not (0 <= self.min_stable_fraction <= self.max_continuous_fraction):
            e.append("fuel-cell min_stable_fraction must be within continuous range")
        if self.ramp_up_W_per_s <= 0 or self.ramp_down_W_per_s <= 0:
            e.append("fuel-cell ramp rates must be > 0")
        if not (0 < self.bus_path_efficiency <= 1.0):
            e.append("fuel-cell bus_path_efficiency must be in (0,1]")
        if not (0 <= self.derating_fraction <= 1.0):
            e.append("fuel-cell derating_fraction must be in [0,1]")
        if self.efficiency_map:
            xs = [p[0] for p in self.efficiency_map]
            if any(x <= 0 for x in xs) or xs != sorted(xs):
                e.append("fuel-cell efficiency_map power points must be positive and ascending")
            if any(not (0 < eta <= 1.0) for _, eta in self.efficiency_map):
                e.append("fuel-cell efficiency_map efficiencies must be in (0,1]")
        if self.stack_mass_kg is not None and self.control_box_mass_kg is not None:
            if abs((self.stack_mass_kg + self.control_box_mass_kg) - self.module_mass_kg) > 1e-6:
                e.append("fuel-cell stack_mass_kg + control_box_mass_kg must equal module_mass_kg")
        return e

    @property
    def thermal_model_enabled(self) -> bool:
        return all(v is not None and v > 0 for v in (
            self.maximum_temperature_C,
            self.thermal_capacitance_J_per_K,
            self.thermal_resistance_K_per_W,
        ))

    @property
    def max_module_power_W(self) -> float:
        return self.rated_net_power_W * self.max_continuous_fraction * self.derating_fraction

    @property
    def max_bus_power_W(self) -> float:
        return self.max_module_power_W * self.bus_path_efficiency

    @property
    def min_module_power_W(self) -> float:
        return self.rated_net_power_W * self.min_stable_fraction

    @property
    def min_bus_power_W(self) -> float:
        return self.min_module_power_W * self.bus_path_efficiency


@dataclass
class HydrogenConfig:
    storage_name: str = "AMS Composites 3 CF, 3.0 L / 300 bar Type III"
    pressure_bar: float = 300.0
    water_volume_L: float = 3.0
    nominal_h2_mass_kg: float = 0.0619
    regulator_mass_kg: float = 0.315
    cylinder_mass_kg: Optional[float] = 1.62

    # Supplier cylinder usable electrical energy (~1030 Wh) already at typical FC
    # efficiency; do not multiply by efficiency again. 15% mission reserve -> ~875.5 Wh.
    reference_load_W: float = 400.0
    reference_endurance_h: float = 2.575
    reference_net_energy_Wh: float = 1030.0
    mission_energy_reserve_fraction: float = 0.15

    # The actual residual pressure / physical unusable H2 is not yet in the sources.
    physical_usable_fraction: float = 1.0
    minimum_residual_h2_kg: Optional[float] = None

    # LHV convention retained so a measured efficiency map can replace the
    # supplier-reference energy basis without changing model architecture.
    h2_LHV_kWh_per_kg: float = H2_LHV_KWH_PER_KG

    def validate(self) -> List[str]:
        e: List[str] = []
        if self.pressure_bar <= 0 or self.water_volume_L <= 0:
            e.append("hydrogen pressure and water volume must be > 0")
        if self.nominal_h2_mass_kg <= 0:
            e.append("hydrogen nominal_h2_mass_kg must be > 0")
        if not (0 < self.physical_usable_fraction <= 1.0):
            e.append("hydrogen physical_usable_fraction must be in (0,1]")
        if not (0 <= self.mission_energy_reserve_fraction < 1.0):
            e.append("hydrogen mission_energy_reserve_fraction must be in [0,1)")
        if self.reference_net_energy_Wh <= 0:
            e.append("hydrogen reference_net_energy_Wh must be > 0")
        if self.cylinder_mass_kg is not None and self.cylinder_mass_kg <= 0:
            e.append("hydrogen cylinder_mass_kg must be > 0 when supplied")
        return e

    @property
    def physically_usable_h2_kg(self) -> float:
        usable = self.nominal_h2_mass_kg * self.physical_usable_fraction
        if self.minimum_residual_h2_kg is not None:
            usable = min(usable, max(0.0, self.nominal_h2_mass_kg - self.minimum_residual_h2_kg))
        return usable

    @property
    def operational_net_energy_Wh(self) -> float:
        # Scale the supplier/reference energy if a future physical usable fraction is entered.
        fraction = self.physically_usable_h2_kg / max(self.nominal_h2_mass_kg, EPS)
        return self.reference_net_energy_Wh * fraction * (1.0 - self.mission_energy_reserve_fraction)

    @property
    def reserve_net_energy_Wh(self) -> float:
        fraction = self.physically_usable_h2_kg / max(self.nominal_h2_mass_kg, EPS)
        return self.reference_net_energy_Wh * fraction * self.mission_energy_reserve_fraction

    @property
    def reference_effective_net_efficiency_LHV(self) -> float:
        chemical_Wh = self.nominal_h2_mass_kg * self.h2_LHV_kWh_per_kg * 1000.0
        return self.reference_net_energy_Wh / max(chemical_Wh, EPS)

    @property
    def reference_h2_kg_per_Wh(self) -> float:
        return self.nominal_h2_mass_kg / max(self.reference_net_energy_Wh, EPS)


@dataclass
class BatteryConfig:
    name: str = "Preliminary 12S 5 Ah high-power buffer"
    series_cells: int = 12
    capacity_Ah: float = 5.0
    nominal_cell_voltage_V: float = 3.70
    initial_soc: float = 1.00
    minimum_soc: float = 0.20
    takeoff_min_soc: float = 0.80
    cruise_target_soc: float = 0.70
    landing_reserve_soc: float = 0.30

    # Electrical model assumptions. Replace with pack test / supplier data.
    internal_resistance_ohm: float = 0.030
    ocv_curve_cell_V: List[Tuple[float, float]] = field(default_factory=lambda: [
        (0.00, 3.45),
        (0.10, 3.60),
        (0.20, 3.70),
        (0.50, 3.80),
        (0.80, 3.95),
        (1.00, 4.20),
    ])
    minimum_loaded_cell_voltage_V: float = 3.30
    maximum_charge_cell_voltage_V: float = 4.20

    # Unknown actual battery limits are represented by None, never silently passed.
    continuous_discharge_current_A: Optional[float] = None
    peak_discharge_current_A: Optional[float] = None
    peak_duration_s: float = 10.0
    continuous_charge_current_A: Optional[float] = None
    maximum_charge_power_W: Optional[float] = None

    # Optional lumped thermal model; disabled when any critical parameter is None.
    mass_kg: Optional[float] = None
    initial_temperature_C: float = 25.0
    maximum_temperature_C: Optional[float] = None
    heat_capacity_J_per_kgK: Optional[float] = None
    thermal_resistance_K_per_W: Optional[float] = None
    ambient_temperature_C: float = 25.0

    coulombic_efficiency_discharge: float = 0.995
    coulombic_efficiency_charge: float = 0.98

    def validate(self) -> List[str]:
        e: List[str] = []
        if self.series_cells <= 0 or self.capacity_Ah <= 0:
            e.append("battery series_cells and capacity_Ah must be > 0")
        if not (0 <= self.minimum_soc < self.initial_soc <= 1):
            e.append("battery SOC window must satisfy 0 <= min < initial <= 1")
        if not (self.minimum_soc <= self.landing_reserve_soc <= self.initial_soc):
            e.append("battery landing_reserve_soc must be inside the operating SOC window")
        if not (self.minimum_soc <= self.cruise_target_soc <= self.initial_soc):
            e.append("battery cruise_target_soc must be inside the operating SOC window")
        if self.internal_resistance_ohm < 0:
            e.append("battery internal resistance cannot be negative")
        pts = sorted(self.ocv_curve_cell_V)
        if pts != self.ocv_curve_cell_V or pts[0][0] > 0 or pts[-1][0] < 1:
            e.append("battery OCV curve must be ascending and cover SOC 0..1")
        return e

    @property
    def nominal_voltage_V(self) -> float:
        return self.series_cells * self.nominal_cell_voltage_V

    @property
    def nominal_energy_Wh(self) -> float:
        return self.nominal_voltage_V * self.capacity_Ah

    @property
    def minimum_loaded_voltage_V(self) -> float:
        return self.series_cells * self.minimum_loaded_cell_voltage_V

    def cell_ocv_V(self, soc: float) -> float:
        s = max(0.0, min(1.0, soc))
        pts = self.ocv_curve_cell_V
        if s <= pts[0][0]:
            return pts[0][1]
        for (s0, v0), (s1, v1) in zip(pts, pts[1:]):
            if s <= s1:
                f = (s - s0) / max(s1 - s0, EPS)
                return v0 + f * (v1 - v0)
        return pts[-1][1]

    def pack_ocv_V(self, soc: float) -> float:
        return self.series_cells * self.cell_ocv_V(soc)

    @property
    def thermal_model_enabled(self) -> bool:
        return all(v is not None and v > 0 for v in (
            self.mass_kg,
            self.maximum_temperature_C,
            self.heat_capacity_J_per_kgK,
            self.thermal_resistance_K_per_W,
        ))


@dataclass
class EMSConfig:
    soc_recovery_max_power_W: float = 150.0
    soc_recovery_deadband: float = 0.02
    fc_low_h2_derate_start_fraction: float = 0.05
    safe_landing_soc_trigger: float = 0.25
    safe_landing_h2_operational_energy_fraction: float = 0.05
    allow_battery_charge_from_fc: bool = True
    # Conservative advertised-endurance policy: battery may support transients and
    # recovery/landing, but normal cruise is not deliberately extended after the
    # protected H2 operational budget is exhausted.
    allow_planned_battery_endurance_extension: bool = False
    enforce_landing_reserve_at_end: bool = True


@dataclass
class RequirementConfig:
    application_peak_screening_W: float = 5000.0
    supplier_conditional_hybrid_peak_W: float = 7500.0
    representative_cruise_power_gate_W: float = 550.0
    minimum_mission_endurance_min: float = 60.0
    target_mission_endurance_min: float = 120.0


@dataclass
class HybridPowerConfig:
    fuel_cell: FuelCellConfig = field(default_factory=FuelCellConfig)
    hydrogen: HydrogenConfig = field(default_factory=HydrogenConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)
    ems: EMSConfig = field(default_factory=EMSConfig)
    requirements: RequirementConfig = field(default_factory=RequirementConfig)
    model_version: str = "H2AIR-phase2-hybrid-power-1.0"

    def validate(self) -> List[str]:
        e = self.fuel_cell.validate() + self.hydrogen.validate() + self.battery.validate()
        if not (self.fuel_cell.output_voltage_min_V <= self.battery.nominal_voltage_V <= self.fuel_cell.output_voltage_max_V):
            e.append(
                f"12S/battery nominal voltage {self.battery.nominal_voltage_V:.1f} V is outside "
                f"fuel-cell reported output range {self.fuel_cell.output_voltage_min_V:.1f}..{self.fuel_cell.output_voltage_max_V:.1f} V"
            )
        return e


@dataclass(frozen=True)
class PowerDemandPoint:
    time_s: float
    bus_power_W: float
    stage: str = ""
    is_peak_sample: bool = False


@dataclass
class HybridState:
    time_s: float = 0.0
    battery_soc: float = 1.0
    battery_temperature_C: float = 25.0
    fc_temperature_C: float = 25.0
    fc_module_power_W: float = 0.0
    h2_used_kg: float = 0.0
    fc_net_energy_used_Wh: float = 0.0
    battery_net_energy_out_Wh: float = 0.0
    startup_elapsed_s: float = 0.0
    fc_started: bool = False


@dataclass(frozen=True)
class GateResult:
    name: str
    status: str
    value: Optional[float]
    limit: Optional[float]
    units: str
    message: str


@dataclass
class StepResult:
    time_s: float
    dt_s: float
    stage: str
    ems_state: str
    demand_W: float
    fc_target_bus_W: float
    fc_bus_power_W: float
    fc_bus_current_A: float
    battery_power_W: float
    battery_current_A: float
    battery_voltage_V: float
    total_bus_current_A: float
    battery_soc: float
    battery_temperature_C: float
    fc_temperature_C: float
    h2_used_kg: float
    h2_remaining_kg: float
    h2_screening_pressure_bar: float
    h2_operational_energy_remaining_Wh: float
    fc_energy_used_Wh: float
    unserved_power_W: float
    curtailed_fc_power_W: float
    warnings: Tuple[str, ...] = ()


@dataclass
class SimulationResult:
    feasible: bool
    failure_reasons: List[str]
    warnings: List[str]
    steps: List[StepResult]
    gates: List[GateResult]
    summary: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "feasible": self.feasible,
            "failure_reasons": list(self.failure_reasons),
            "warnings": list(self.warnings),
            "gates": [asdict(x) for x in self.gates],
            "summary": self.summary,
            "steps": [asdict(x) for x in self.steps],
        }


def _interp(points: Sequence[Tuple[float, float]], x: float) -> float:
    if x <= points[0][0]:
        return points[0][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            f = (x - x0) / max(x1 - x0, EPS)
            return y0 + f * (y1 - y0)
    return points[-1][1]


def _battery_current_voltage_for_power(
    battery: BatteryConfig,
    soc: float,
    requested_power_W: float,
) -> Tuple[float, float, float, List[str]]:
    """Return achievable power, current and terminal voltage for a pack.

    Positive power is discharge. Negative power is charging.  The quadratic
    constant-resistance battery equation is solved exactly. Unknown current limits
    are not invented; voltage and resistance limits still apply.
    """
    warnings: List[str] = []
    voc = battery.pack_ocv_V(soc)
    R = max(0.0, battery.internal_resistance_ohm)
    P = float(requested_power_W)

    if abs(P) < 1e-9:
        return 0.0, 0.0, voc, warnings

    if R <= EPS:
        I = P / max(voc, EPS)
        V = voc
    else:
        disc = voc * voc - 4.0 * R * P
        if disc < 0.0:
            # The requested discharge power exceeds the maximum-power point.
            I = voc / (2.0 * R)
            V = voc / 2.0
            P = V * I
            warnings.append("battery_resistance_power_limit")
        else:
            I = (voc - sqrt(disc)) / (2.0 * R)
            V = voc - I * R

    # Charge acceptance limit.
    if I < 0.0:
        charge_limit_A = battery.continuous_charge_current_A
        if charge_limit_A is not None and -I > charge_limit_A:
            I = -charge_limit_A
            V = voc - I * R
            P = V * I
            warnings.append("battery_charge_current_limited")
        if battery.maximum_charge_power_W is not None and -P > battery.maximum_charge_power_W:
            P = -battery.maximum_charge_power_W
            if R <= EPS:
                I = P / max(voc, EPS)
                V = voc
            else:
                disc = voc * voc - 4.0 * R * P
                I = (voc - sqrt(max(0.0, disc))) / (2.0 * R)
                V = voc - I * R
            warnings.append("battery_charge_power_limited")
        max_charge_V = battery.series_cells * battery.maximum_charge_cell_voltage_V
        if V > max_charge_V:
            # Reduce charge current to the voltage ceiling.
            I = (voc - max_charge_V) / max(R, EPS)
            V = max_charge_V
            P = V * I
            warnings.append("battery_charge_voltage_limited")

    # Discharge current limits. Peak limit is used for step-wise transient model;
    # continuous limit is also reported by the simulator as a gate when known.
    if I > 0.0 and battery.peak_discharge_current_A is not None and I > battery.peak_discharge_current_A:
        I = battery.peak_discharge_current_A
        V = voc - I * R
        P = max(0.0, V * I)
        warnings.append("battery_peak_current_limited")

    return P, I, V, warnings


class HybridPowerSystem:
    def __init__(self, config: HybridPowerConfig):
        errors = config.validate()
        if errors:
            raise ValueError("Invalid Phase-2 hybrid-power configuration: " + "; ".join(errors))
        self.c = config

    def initial_state(self) -> HybridState:
        fc = self.c.fuel_cell
        batt = self.c.battery
        if fc.enabled and fc.ready_at_mission_start:
            p0 = min(max(0.0, fc.preflight_power_W), fc.max_bus_power_W)
            started = True
            startup_elapsed = fc.startup_time_s
        else:
            p0 = 0.0
            started = False
            startup_elapsed = 0.0
        return HybridState(
            battery_soc=batt.initial_soc,
            battery_temperature_C=batt.initial_temperature_C,
            fc_temperature_C=fc.initial_temperature_C,
            fc_module_power_W=p0 / max(fc.bus_path_efficiency, EPS),
            startup_elapsed_s=startup_elapsed,
            fc_started=started,
        )

    def _fc_h2_consumption_kg(self, module_power_W: float, dt_s: float) -> float:
        fc = self.c.fuel_cell
        h2 = self.c.hydrogen
        if module_power_W <= 0 or dt_s <= 0:
            return 0.0
        net_Wh = module_power_W * dt_s / 3600.0
        if fc.efficiency_map:
            eta = _interp(fc.efficiency_map, module_power_W)
            chemical_Wh = net_Wh / max(eta, EPS)
            return chemical_Wh / max(h2.h2_LHV_kWh_per_kg * 1000.0, EPS)
        # Reference-derived net-energy basis. This is the current application-level
        # model and should be replaced with measured load maps in bench validation.
        return net_Wh * h2.reference_h2_kg_per_Wh

    def _remaining_h2(self, state: HybridState) -> float:
        return max(0.0, self.c.hydrogen.physically_usable_h2_kg - state.h2_used_kg)

    def _remaining_operational_fc_energy_Wh(self, state: HybridState) -> float:
        return max(0.0, self.c.hydrogen.operational_net_energy_Wh - state.fc_net_energy_used_Wh)

    def _fc_available(self, state: HybridState, t: float) -> bool:
        fc = self.c.fuel_cell
        if not fc.enabled:
            return False
        if fc.forced_fault_time_s is not None and t >= fc.forced_fault_time_s:
            return False
        if self._remaining_operational_fc_energy_Wh(state) <= 1e-9:
            return False
        return True

    def _target_fc_bus_power(self, demand_W: float, state: HybridState, t: float) -> Tuple[float, str]:
        fc = self.c.fuel_cell
        batt = self.c.battery
        ems = self.c.ems
        demand = max(0.0, demand_W)

        if not self._fc_available(state, t):
            if state.battery_soc <= ems.safe_landing_soc_trigger:
                return 0.0, EMSState.SAFE_LANDING.value
            return 0.0, EMSState.BATTERY_ONLY.value

        if not state.fc_started:
            return 0.0, EMSState.STARTUP.value

        fc_max = fc.max_bus_power_W
        fc_min = fc.min_bus_power_W
        target = min(demand, fc_max)

        if demand > fc_max + 1e-9:
            return fc_max, EMSState.PEAK_SUPPORT.value

        if ems.allow_battery_charge_from_fc and state.battery_soc < batt.cruise_target_soc - ems.soc_recovery_deadband:
            target = min(fc_max, demand + max(0.0, ems.soc_recovery_max_power_W))
            state_name = EMSState.SOC_RECOVERY.value
        else:
            state_name = EMSState.LOAD_FOLLOW.value

        # If demand is below minimum stable FC output, use surplus to charge the
        # battery where possible; otherwise the step routine will curtail it.
        if 0.0 < target < fc_min:
            target = fc_min
        return target, state_name

    def step(self, state: HybridState, demand_W: float, dt_s: float, stage: str = "") -> StepResult:
        if dt_s <= 0:
            raise ValueError("dt_s must be > 0")
        c = self.c
        fc = c.fuel_cell
        batt = c.battery
        warnings: List[str] = []
        t = state.time_s

        # Startup progress when the stack was not declared ready at mission start.
        if fc.enabled and not state.fc_started:
            state.startup_elapsed_s += dt_s
            if state.startup_elapsed_s + 1e-9 >= fc.startup_time_s:
                state.fc_started = True

        target_bus_W, ems_state = self._target_fc_bus_power(demand_W, state, t)
        target_module_W = target_bus_W / max(fc.bus_path_efficiency, EPS)

        current_module = state.fc_module_power_W
        if target_module_W > current_module:
            next_module = min(target_module_W, current_module + fc.ramp_up_W_per_s * dt_s)
        else:
            next_module = max(target_module_W, current_module - fc.ramp_down_W_per_s * dt_s)
        if not self._fc_available(state, t):
            next_module = 0.0
        next_module = max(0.0, min(next_module, fc.max_module_power_W))

        # Never consume the protected H2 mission-energy reserve because of a
        # finite integration step. When the operational FC energy is nearly
        # exhausted, trim this step to the exact remaining usable energy and let
        # the battery carry any remaining bus demand.
        remaining_fc_Wh = self._remaining_operational_fc_energy_Wh(state)
        max_module_from_energy_W = remaining_fc_Wh * 3600.0 / max(dt_s, EPS)
        if next_module > max_module_from_energy_W + 1e-9:
            next_module = max(0.0, max_module_from_energy_W)
            warnings.append("fuel_cell_operational_reserve_limited")

        fc_bus_W = next_module * fc.bus_path_efficiency

        # First-pass battery request: positive discharge, negative charge.
        batt_request_W = max(0.0, demand_W) - fc_bus_W

        # Do not charge above full SOC; curtail surplus FC instead.
        if batt_request_W < 0.0 and state.battery_soc >= 1.0 - 1e-9:
            fc_bus_W = max(0.0, min(fc_bus_W, demand_W))
            next_module = fc_bus_W / max(fc.bus_path_efficiency, EPS)
            batt_request_W = 0.0

        batt_power_W, batt_I_A, batt_V, batt_warnings = _battery_current_voltage_for_power(
            batt, state.battery_soc, batt_request_W
        )
        warnings.extend(batt_warnings)

        # Energy/SOC limit for the step.
        if batt_power_W > 0.0:
            max_discharge_Wh = max(0.0, (state.battery_soc - batt.minimum_soc) * batt.nominal_energy_Wh)
            req_Wh = batt_power_W * dt_s / 3600.0 / max(batt.coulombic_efficiency_discharge, EPS)
            if req_Wh > max_discharge_Wh + 1e-12:
                achievable_W = max_discharge_Wh * 3600.0 / dt_s * batt.coulombic_efficiency_discharge
                batt_power_W, batt_I_A, batt_V, w = _battery_current_voltage_for_power(
                    batt, state.battery_soc, achievable_W
                )
                warnings.extend(w)
                warnings.append("battery_soc_floor_limited")
        elif batt_power_W < 0.0:
            max_charge_Wh = max(0.0, (1.0 - state.battery_soc) * batt.nominal_energy_Wh)
            req_store_Wh = -batt_power_W * dt_s / 3600.0 * batt.coulombic_efficiency_charge
            if req_store_Wh > max_charge_Wh + 1e-12:
                accept_bus_W = max_charge_Wh * 3600.0 / dt_s / max(batt.coulombic_efficiency_charge, EPS)
                batt_power_W, batt_I_A, batt_V, w = _battery_current_voltage_for_power(
                    batt, state.battery_soc, -accept_bus_W
                )
                warnings.extend(w)
                warnings.append("battery_soc_ceiling_limited")

        # If FC surplus cannot be accepted, curtail FC to load plus actual charge.
        if batt_request_W < 0.0 and batt_power_W > batt_request_W + 1e-6:
            fc_bus_required = max(0.0, demand_W - batt_power_W)
            curtailed = max(0.0, fc_bus_W - fc_bus_required)
            fc_bus_W = fc_bus_required
            next_module = fc_bus_W / max(fc.bus_path_efficiency, EPS)
        else:
            curtailed = 0.0

        delivered_W = fc_bus_W + batt_power_W
        unserved_W = max(0.0, demand_W - delivered_W)

        # Battery voltage is a hard electrical feasibility check even when current
        # limits are still unknown.
        if batt_power_W > 0 and batt_V < batt.minimum_loaded_voltage_V - 1e-9:
            warnings.append("battery_undervoltage")

        # SOC integration based on nominal Wh bookkeeping.
        if batt_power_W >= 0.0:
            dE_Wh = batt_power_W * dt_s / 3600.0 / max(batt.coulombic_efficiency_discharge, EPS)
            state.battery_soc -= dE_Wh / max(batt.nominal_energy_Wh, EPS)
            state.battery_net_energy_out_Wh += batt_power_W * dt_s / 3600.0
        else:
            dE_store_Wh = -batt_power_W * dt_s / 3600.0 * batt.coulombic_efficiency_charge
            state.battery_soc += dE_store_Wh / max(batt.nominal_energy_Wh, EPS)
            state.battery_net_energy_out_Wh += batt_power_W * dt_s / 3600.0
        state.battery_soc = max(0.0, min(1.0, state.battery_soc))

        # Lumped thermal model when actual parameters become available.
        if batt.thermal_model_enabled:
            heat_W = batt_I_A * batt_I_A * batt.internal_resistance_ohm
            cooling_W = (state.battery_temperature_C - batt.ambient_temperature_C) / max(batt.thermal_resistance_K_per_W or EPS, EPS)
            Cth = (batt.mass_kg or 0.0) * (batt.heat_capacity_J_per_kgK or 0.0)
            state.battery_temperature_C += (heat_W - cooling_W) * dt_s / max(Cth, EPS)
            if state.battery_temperature_C > float(batt.maximum_temperature_C):
                warnings.append("battery_overtemperature")

        # Optional lumped FC thermal model. Waste heat is estimated from the
        # configured efficiency map or the supplier-reference effective net
        # efficiency. No thermal PASS is issued until actual thermal parameters exist.
        if fc.thermal_model_enabled and next_module > 0.0:
            if fc.efficiency_map:
                eta_fc = _interp(fc.efficiency_map, next_module)
            else:
                eta_fc = c.hydrogen.reference_effective_net_efficiency_LHV
            waste_heat_W = next_module * (1.0 / max(eta_fc, EPS) - 1.0)
            cooling_W = (state.fc_temperature_C - fc.ambient_temperature_C) / max(fc.thermal_resistance_K_per_W or EPS, EPS)
            state.fc_temperature_C += (waste_heat_W - cooling_W) * dt_s / max(fc.thermal_capacitance_J_per_K or EPS, EPS)
            if state.fc_temperature_C > float(fc.maximum_temperature_C):
                warnings.append("fuel_cell_overtemperature")

        # H2 / fuel-cell energy accounting after actual FC output is known.
        h2_kg = self._fc_h2_consumption_kg(next_module, dt_s)
        fc_net_Wh = next_module * dt_s / 3600.0
        state.h2_used_kg += h2_kg
        state.fc_net_energy_used_Wh += fc_net_Wh
        state.fc_module_power_W = next_module
        state.time_s += dt_s

        h2_remaining = self._remaining_h2(state)
        op_energy_remaining = self._remaining_operational_fc_energy_Wh(state)
        if op_energy_remaining <= 1e-9:
            warnings.append("hydrogen_operational_reserve_reached")

        fc_bus_I = fc_bus_W / max(batt_V, EPS)
        total_bus_I = max(0.0, demand_W) / max(batt_V, EPS)
        h2_pressure_screen = c.hydrogen.pressure_bar * h2_remaining / max(c.hydrogen.nominal_h2_mass_kg, EPS)

        return StepResult(
            time_s=state.time_s,
            dt_s=dt_s,
            stage=stage,
            ems_state=ems_state,
            demand_W=float(demand_W),
            fc_target_bus_W=target_bus_W,
            fc_bus_power_W=fc_bus_W,
            fc_bus_current_A=fc_bus_I,
            battery_power_W=batt_power_W,
            battery_current_A=batt_I_A,
            battery_voltage_V=batt_V,
            total_bus_current_A=total_bus_I,
            battery_soc=state.battery_soc,
            battery_temperature_C=state.battery_temperature_C,
            fc_temperature_C=state.fc_temperature_C,
            h2_used_kg=state.h2_used_kg,
            h2_remaining_kg=h2_remaining,
            h2_screening_pressure_bar=h2_pressure_screen,
            h2_operational_energy_remaining_Wh=op_energy_remaining,
            fc_energy_used_Wh=state.fc_net_energy_used_Wh,
            unserved_power_W=unserved_W,
            curtailed_fc_power_W=curtailed,
            warnings=tuple(sorted(set(warnings))),
        )

    def simulate(
        self,
        profile: Sequence[PowerDemandPoint],
        *,
        capture_steps: bool = True,
        max_step_s: float = 1.0,
    ) -> SimulationResult:
        if len(profile) < 2:
            raise ValueError("Power profile must contain at least two time points.")
        if any(profile[i + 1].time_s <= profile[i].time_s for i in range(len(profile) - 1)):
            raise ValueError("Power profile times must be strictly increasing.")
        if max_step_s <= 0:
            raise ValueError("max_step_s must be > 0")

        state = self.initial_state()
        steps: List[StepResult] = []
        warnings: List[str] = []
        failures: List[str] = []
        peak_demand = 0.0
        peak_batt_power = 0.0
        peak_batt_current = 0.0
        min_batt_voltage = float("inf")
        min_soc = state.battery_soc
        peak_fc_bus = 0.0
        peak_bus_current = 0.0
        max_fc_temperature = state.fc_temperature_C
        energy_demand_Wh = 0.0

        # Profile power is piecewise constant at the left point until the next time.
        # Long stages are internally sub-stepped so FC ramp, SOC, H2 depletion and
        # fault timing are genuinely time-resolved rather than applied once per stage.
        for p0, p1 in zip(profile, profile[1:]):
            remaining = p1.time_s - p0.time_s
            while remaining > 1e-12:
                dt = min(max_step_s, remaining)
                res = self.step(state, p0.bus_power_W, dt, stage=p0.stage)
                if capture_steps:
                    steps.append(res)
                warnings.extend(res.warnings)
                peak_demand = max(peak_demand, p0.bus_power_W)
                peak_batt_power = max(peak_batt_power, max(0.0, res.battery_power_W))
                peak_batt_current = max(peak_batt_current, max(0.0, res.battery_current_A))
                min_batt_voltage = min(min_batt_voltage, res.battery_voltage_V)
                min_soc = min(min_soc, res.battery_soc)
                peak_fc_bus = max(peak_fc_bus, res.fc_bus_power_W)
                peak_bus_current = max(peak_bus_current, res.total_bus_current_A)
                max_fc_temperature = max(max_fc_temperature, res.fc_temperature_C)
                energy_demand_Wh += p0.bus_power_W * dt / 3600.0
                if res.unserved_power_W > 1e-6:
                    failures.append(f"unserved bus power {res.unserved_power_W:.1f} W at t={res.time_s:.1f}s ({res.stage})")
                if "battery_undervoltage" in res.warnings:
                    failures.append(f"battery undervoltage at t={res.time_s:.1f}s ({res.stage})")
                if "battery_overtemperature" in res.warnings:
                    failures.append(f"battery overtemperature at t={res.time_s:.1f}s ({res.stage})")
                if "fuel_cell_overtemperature" in res.warnings:
                    failures.append(f"fuel-cell overtemperature at t={res.time_s:.1f}s ({res.stage})")
                if (
                    not self.c.ems.allow_planned_battery_endurance_extension
                    and "fuel_cell_operational_reserve_limited" in res.warnings
                    and "cruise" in res.stage.lower()
                ):
                    failures.append(
                        f"planned cruise reaches protected H2 operational reserve at t={res.time_s:.1f}s; "
                        "battery endurance extension is disabled"
                    )
                remaining -= dt

        duration_s = profile[-1].time_s - profile[0].time_s
        final_soc = state.battery_soc
        if self.c.ems.enforce_landing_reserve_at_end and final_soc < self.c.battery.landing_reserve_soc - 1e-9:
            failures.append(
                f"final battery SOC {final_soc:.3f} below landing reserve {self.c.battery.landing_reserve_soc:.3f}"
            )

        # If a current/temperature rating is unknown, do not call it a pass.
        gates: List[GateResult] = []
        req = self.c.requirements
        gates.append(GateResult(
            "takeoff_initial_soc",
            GateStatus.PASS.value if self.c.battery.initial_soc >= self.c.battery.takeoff_min_soc else GateStatus.FAIL.value,
            self.c.battery.initial_soc, self.c.battery.takeoff_min_soc, "fraction",
            "Mission must not begin below the protected takeoff SOC threshold.",
        ))
        gates.append(GateResult(
            "application_peak_power_screening",
            GateStatus.PASS.value if peak_demand <= req.application_peak_screening_W else GateStatus.WARNING.value,
            peak_demand, req.application_peak_screening_W, "W",
            "Application preliminary peak estimate; exceedance is a design-review warning, not a supplier hardware limit.",
        ))
        gates.append(GateResult(
            "conditional_supplier_hybrid_peak",
            GateStatus.PASS.value if peak_demand <= req.supplier_conditional_hybrid_peak_W else GateStatus.FAIL.value,
            peak_demand, req.supplier_conditional_hybrid_peak_W, "W",
            "Supplier-reported hybrid peak is conditional on the selected battery; PASS remains conditional until battery configuration is verified.",
        ))
        if self.c.battery.peak_discharge_current_A is None:
            gates.append(GateResult(
                "battery_peak_current",
                GateStatus.NOT_EVALUATED.value,
                peak_batt_current, None, "A",
                "Actual selected battery peak-current limit is not yet available.",
            ))
        else:
            gates.append(GateResult(
                "battery_peak_current",
                GateStatus.PASS.value if peak_batt_current <= self.c.battery.peak_discharge_current_A else GateStatus.FAIL.value,
                peak_batt_current, self.c.battery.peak_discharge_current_A, "A",
                "Compared with configured battery peak-current rating.",
            ))
        if self.c.battery.continuous_discharge_current_A is None:
            gates.append(GateResult(
                "battery_continuous_current",
                GateStatus.NOT_EVALUATED.value,
                peak_batt_current, None, "A",
                "Continuous-current limit is unknown; use the time history with supplier/bench limits.",
            ))
        if self.c.fuel_cell.maximum_temperature_C is None or not self.c.fuel_cell.thermal_model_enabled:
            gates.append(GateResult(
                "fuel_cell_thermal",
                GateStatus.NOT_EVALUATED.value,
                max_fc_temperature, self.c.fuel_cell.maximum_temperature_C, "degC",
                "Fuel-cell thermal capacitance/cooling resistance/limit are not yet supplier-confirmed.",
            ))
        else:
            gates.append(GateResult(
                "fuel_cell_thermal",
                GateStatus.PASS.value if max_fc_temperature <= self.c.fuel_cell.maximum_temperature_C else GateStatus.FAIL.value,
                max_fc_temperature, self.c.fuel_cell.maximum_temperature_C, "degC",
                "Lumped fuel-cell thermal model.",
            ))
        if self.c.battery.maximum_temperature_C is None or not self.c.battery.thermal_model_enabled:
            gates.append(GateResult(
                "battery_thermal",
                GateStatus.NOT_EVALUATED.value,
                state.battery_temperature_C, self.c.battery.maximum_temperature_C, "degC",
                "Battery thermal properties/limit are not yet supplier-confirmed.",
            ))
        else:
            gates.append(GateResult(
                "battery_thermal",
                GateStatus.PASS.value if state.battery_temperature_C <= self.c.battery.maximum_temperature_C else GateStatus.FAIL.value,
                state.battery_temperature_C, self.c.battery.maximum_temperature_C, "degC",
                "Lumped battery thermal model.",
            ))
        gates.append(GateResult(
            "battery_minimum_loaded_voltage",
            GateStatus.PASS.value if min_batt_voltage >= self.c.battery.minimum_loaded_voltage_V else GateStatus.FAIL.value,
            min_batt_voltage, self.c.battery.minimum_loaded_voltage_V, "V",
            "Uses preliminary OCV/internal-resistance model until pack sag testing is available.",
        ))
        gates.append(GateResult(
            "landing_soc_reserve",
            GateStatus.PASS.value if final_soc >= self.c.battery.landing_reserve_soc else GateStatus.FAIL.value,
            final_soc, self.c.battery.landing_reserve_soc, "fraction",
            "Protected battery reserve for reverse transition / landing / go-around contingency.",
        ))
        gates.append(GateResult(
            "hydrogen_operational_reserve",
            GateStatus.PASS.value if state.fc_net_energy_used_Wh <= self.c.hydrogen.operational_net_energy_Wh + 1e-9 else GateStatus.FAIL.value,
            state.fc_net_energy_used_Wh, self.c.hydrogen.operational_net_energy_Wh, "Wh",
            "15% application mission-energy reserve is protected from normal mission use.",
        ))

        if any(g.status == GateStatus.FAIL.value for g in gates):
            failures.extend([g.name for g in gates if g.status == GateStatus.FAIL.value])

        summary = {
            "model_version": self.c.model_version,
            "duration_s": duration_s,
            "duration_min": duration_s / 60.0,
            "bus_energy_demand_Wh": energy_demand_Wh,
            "peak_bus_power_W": peak_demand,
            "peak_fc_bus_power_W": peak_fc_bus,
            "peak_bus_current_A": peak_bus_current,
            "peak_battery_power_W": peak_batt_power,
            "peak_battery_current_A": peak_batt_current,
            "minimum_battery_voltage_V": min_batt_voltage,
            "minimum_battery_soc": min_soc,
            "final_battery_soc": final_soc,
            "maximum_fuel_cell_temperature_C": max_fc_temperature,
            "battery_net_energy_out_Wh": state.battery_net_energy_out_Wh,
            "h2_used_kg": state.h2_used_kg,
            "h2_remaining_physical_kg": self._remaining_h2(state),
            "fc_net_energy_used_Wh": state.fc_net_energy_used_Wh,
            "fc_operational_energy_remaining_Wh": self._remaining_operational_fc_energy_Wh(state),
            "h2_reference_effective_net_efficiency_LHV": self.c.hydrogen.reference_effective_net_efficiency_LHV,
            "battery_nominal_voltage_V": self.c.battery.nominal_voltage_V,
            "battery_nominal_energy_Wh": self.c.battery.nominal_energy_Wh,
            "fuel_cell_max_bus_power_W": self.c.fuel_cell.max_bus_power_W,
            "fuel_cell_mass_kg": self.c.fuel_cell.module_mass_kg,
            "regulator_mass_kg": self.c.hydrogen.regulator_mass_kg,
            "cylinder_mass_kg": self.c.hydrogen.cylinder_mass_kg,
            "energy_system_mass_complete": self.c.hydrogen.cylinder_mass_kg is not None and self.c.battery.mass_kg is not None,
        }
        return SimulationResult(
            feasible=not failures,
            failure_reasons=sorted(set(failures)),
            warnings=sorted(set(warnings)),
            steps=steps,
            gates=gates,
            summary=summary,
        )


def build_piecewise_profile(stages: Sequence[Mapping[str, Any]], *, start_time_s: float = 0.0) -> List[PowerDemandPoint]:
    """Build a piecewise-constant power profile from simple stage dictionaries.

    Required keys per stage: `duration_s`, `power_W`; optional `name`.
    The final point repeats the last stage power only as a terminal timestamp.
    """
    t = float(start_time_s)
    out: List[PowerDemandPoint] = []
    for stage in stages:
        dt = max(0.0, float(stage["duration_s"]))
        if dt <= 0:
            continue
        name = str(stage.get("name", ""))
        p = max(0.0, float(stage["power_W"]))
        out.append(PowerDemandPoint(t, p, name))
        t += dt
    if not out:
        raise ValueError("No positive-duration stages supplied.")
    out.append(PowerDemandPoint(t, out[-1].bus_power_W, out[-1].stage))
    return out


def phase2_reference_sources() -> Dict[str, SourceTaggedValue]:
    """Machine-readable evidence ledger for the Phase-2 reference configuration."""
    src = "Hydrogenera IH-MIE Application Draft v8 (31 Aug 2026), citing Intelligent Energy IE-SOAR 800 supplier material"
    return {
        "fuel_cell_rated_net_power_W": SourceTaggedValue(800.0, EvidenceStatus.SUPPLIER_CONFIRMED.value, src, "continuous rating reported in application"),
        "fuel_cell_module_mass_kg": SourceTaggedValue(1.45, EvidenceStatus.SUPPLIER_CONFIRMED.value, src),
        "fuel_cell_output_voltage_V": SourceTaggedValue([24.0, 48.0], EvidenceStatus.SUPPLIER_CONFIRMED.value, src),
        "regulator_mass_kg": SourceTaggedValue(0.315, EvidenceStatus.SUPPLIER_CONFIRMED.value, src),
        "hydrogen_storage": SourceTaggedValue("AMS Composites 3 CF, 3.0 L / 300 bar Type III", EvidenceStatus.SUPPLIER_CONFIRMED.value, src, "1.62 kg cylinder, M18x1.5"),
        "hydrogen_inventory_kg": SourceTaggedValue(0.0619, EvidenceStatus.SUPPLIER_CONFIRMED.value, src, "61.9 g H2 from AMS cylinder datasheet"),
        "cylinder_mass_kg": SourceTaggedValue(1.62, EvidenceStatus.SUPPLIER_CONFIRMED.value, src, "AMS Composites 3 CF"),
        "hybrid_peak_power_W": SourceTaggedValue(7500.0, EvidenceStatus.SUPPLIER_REPORTED_CONDITIONAL.value, src, "depends on selected battery"),
        "reference_400W_endurance_h": SourceTaggedValue(2.575, EvidenceStatus.DERIVED_FROM_SUPPLIER_REFERENCE.value, src, "1030 Wh / 400 W; ~131 min before reserve"),
        "reference_net_energy_Wh": SourceTaggedValue(1030.0, EvidenceStatus.SUPPLIER_CONFIRMED.value, src, "AMS/IE usable electrical Wh; already at typical FC efficiency"),
        "mission_energy_reserve_fraction": SourceTaggedValue(0.15, EvidenceStatus.PRELIMINARY_TARGET.value, "Hydrogenera IH-MIE Application Draft v8"),
        "battery_architecture": SourceTaggedValue("12S 4-5 Ah / 178-222 Wh", EvidenceStatus.PRELIMINARY_TARGET.value, "Hydrogenera IH-MIE Application Draft v8"),
        "application_peak_screening_W": SourceTaggedValue(5000.0, EvidenceStatus.PRELIMINARY_TARGET.value, "Hydrogenera IH-MIE Application Draft v8"),
        "cruise_power_gate_W": SourceTaggedValue(550.0, EvidenceStatus.PRELIMINARY_TARGET.value, "Hydrogenera IH-MIE Application Draft v8"),
        "cruise_power_target_band_W": SourceTaggedValue([400.0, 435.0], EvidenceStatus.PRELIMINARY_TARGET.value, src, "450 W sensitivity; ~438 W is exact 2 h at 875.5 Wh post-reserve"),
    }

# Clean integrated API alias; legacy development name retained for compatibility.
reference_sources = phase2_reference_sources
