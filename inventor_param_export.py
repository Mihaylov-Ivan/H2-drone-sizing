# inventor_param_export.py
"""
Export Autodesk Inventor Parameters from DesignResults.

Outputs:
1) XML  -> Inventor: Manage -> Parameters -> Import
2) XLSX -> Inventor: Manage -> Parameters -> Link...

IMPORTANT for Inventor XLSX Link:
- Column A must be parameter Name
- Column B must be NUMERIC ONLY (no units, no text)
- Units are defined inside Inventor, not in Excel

Naming:
- Strips unit suffixes from names (e.g. *_m, *_m2, *_deg) because we export converted values:
  *_m  -> exported value is mm
  *_m2 -> exported value is mm2
  *_m3 -> exported value is mm3
  *_in -> exported value is mm
  *_deg-> exported value is deg

NEW:
- FORCE export now supports:
  (A) flat keys (existing): "c_root"
  (B) dotted paths: "results.vertical_wing.structure.spar_area_root_m2"
  (C) expressions: "results.wing.structure.spar_area_root_m2 / results.wing.structure.h_m"
      Allowed operators: + - * / and parentheses ( )
      Expressions may reference results.* and inputs.* paths.

Safe expression evaluation: uses AST whitelist (no calls, no attributes, no subscripting, etc.).
"""

from __future__ import annotations

import ast
import math
import os
import re
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import asdict, is_dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union

from openpyxl import Workbook

from results import DesignResults


# ----------------------------
# Unit conversion helpers
# ----------------------------
def _mm_from_m(m: float) -> float:
    return float(m) * 1000.0


def _mm2_from_m2(m2: float) -> float:
    return float(m2) * 1_000_000.0


def _mm3_from_m3(m3: float) -> float:
    return float(m3) * 1_000_000_000.0


def _mm_from_inch(inch: float) -> float:
    return float(inch) * 25.4


def _is_number(x: Any) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def _is_finite_number(x: Any) -> bool:
    return _is_number(x) and math.isfinite(float(x))


def _safe_param_name(name: str) -> str:
    name = str(name).strip()
    name = re.sub(r"[^\w]+", "_", name)
    name = re.sub(r"_+", "_", name).strip("_")
    if not name:
        name = "param"
    if name[0].isdigit():
        name = f"p_{name}"
    return name


def _fmt_value(value: float, unit: str) -> str:
    # Used for XML value formatting only
    if unit in ("mm", "mm2", "mm3"):
        return f"{value:.3f} {unit}"
    if unit == "deg":
        return f"{value:.3f} deg"
    if unit == "unitless":
        return f"{value:.6g}"
    return f"{value:.6g} {unit}".strip()


# ----------------------------
# Flatten helpers
# ----------------------------
def _flatten_dict(d: Dict[str, Any], prefix: str = "") -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for k, v in d.items():
        key = f"{prefix}{k}" if not prefix else f"{prefix}_{k}"
        if isinstance(v, dict):
            out.update(_flatten_dict(v, key))
        else:
            out[key] = v
    return out


def _inputs_to_flat_dict(inputs: Any) -> Dict[str, Any]:
    """Flatten DesignInputs WITHOUT adding any prefix."""
    if inputs is None:
        return {}
    if isinstance(inputs, dict):
        return _flatten_dict(inputs)
    if is_dataclass(inputs):
        return _flatten_dict(asdict(inputs))
    if hasattr(inputs, "__dict__"):
        return _flatten_dict(dict(inputs.__dict__))
    return {}


# ----------------------------
# Naming: strip unit suffixes
# ----------------------------
_UNIT_NAME_SUFFIXES = ("_m3", "_m2", "_m", "_in", "_deg")


def _strip_unit_suffix(name: str) -> str:
    s = str(name).strip()
    for suf in _UNIT_NAME_SUFFIXES:
        if s.endswith(suf):
            return s[: -len(suf)]
    return s


# ----------------------------
# Dimension detection + mapping
# ----------------------------
_DIM_SUFFIX_RULES = [
    ("_m3", "mm3", _mm3_from_m3),
    ("_m2", "mm2", _mm2_from_m2),
    ("_m", "mm", _mm_from_m),
    ("_in", "mm", _mm_from_inch),
    ("_deg", "deg", lambda x: float(x)),
]

_DIM_HINTS = (
    "span", "chord", "length", "width", "height", "diameter", "radius",
    "thickness", "clearance", "offset", "arm", "cg", "ac", "mac",
    "shell", "skin", "web", "spar", "rib", "fuselage", "boom", "tail",
)

_SKIP_PREFIXES = ("W",)

_SKIP_CONTAINS = (
    "mass", "_kg", "_N", "_W", "_A", "_Wh", "_mAh",
    "Re_", "Re", "C_", "current",
    "L_over_D", "AR", "CL", "CD", "CM", "eta",
    "fraction", "meets_", "name",
    "ratio", "coeff", "k_", "oswald", "h_m",
)


def _infer_unit_and_value(key: str, value: float) -> Optional[Tuple[str, float]]:
    k = key.strip()
    for suffix, unit, conv in _DIM_SUFFIX_RULES:
        if k.endswith(suffix):
            return unit, conv(value)

    lk = k.lower()
    if any(h in lk for h in _DIM_HINTS):
        return "mm", _mm_from_m(value)

    return None


def _should_skip_key(key: str) -> bool:
    k = key.strip()
    lk = k.lower()

    if k.startswith(_SKIP_PREFIXES):
        return True

    for bad in _SKIP_CONTAINS:
        if bad.lower() in lk:
            return True

    return False


# ----------------------------
# Path resolution
# ----------------------------
def _get_by_dotted_path(root: Any, path: str) -> Tuple[bool, Any]:
    """
    Resolve dotted path against objects/dicts.

    Supports:
      - "results.xxx.yyy" (results prefix optional)
      - "inputs.xxx.yyy"  (special marker; caller resolves against results.inputs)
    """
    if not path:
        return False, None

    p = str(path).strip()

    if p.startswith("results."):
        p = p[len("results.") :]

    if p.startswith("inputs."):
        return False, ("__INPUTS_PATH__", p[len("inputs.") :])

    tokens = [t for t in p.split(".") if t]
    cur = root
    for tok in tokens:
        if isinstance(cur, dict):
            if tok not in cur:
                return False, None
            cur = cur[tok]
        else:
            if not hasattr(cur, tok):
                return False, None
            cur = getattr(cur, tok)
    return True, cur


# ----------------------------
# SAFE expression evaluation (NEW)
# ----------------------------
_PATH_TOKEN_RE = re.compile(r"\b(?:results|inputs)\.[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)+\b")


# Build allowed AST nodes tuple (compatible with Python <3.8 and >=3.8)
_allowed_nodes_list = [
    ast.Expression,
    ast.BinOp,
    ast.UnaryOp,
    ast.Add,
    ast.Sub,
    ast.Mult,
    ast.Div,
    ast.USub,
    ast.UAdd,
    ast.Constant,   # py3.8+
    ast.Load,
    ast.Name,
    ast.Pow,        # (optional) keep disabled by default; not used in whitelist below
]
# Include ast.Num for Python <3.8 compatibility
if hasattr(ast, "Num"):
    _allowed_nodes_list.append(ast.Num)
_ALLOWED_AST_NODES = tuple(_allowed_nodes_list)

_ALLOWED_BINOPS = (ast.Add, ast.Sub, ast.Mult, ast.Div, ast.Pow)
_ALLOWED_UNARYOPS = (ast.UAdd, ast.USub)


def _validate_expr_ast(node: ast.AST) -> None:
    """
    Whitelist-only AST validation:
    - numbers
    - names (we substitute paths into names)
    - + - * / and unary +/- only
    """
    for n in ast.walk(node):
        if not isinstance(n, _ALLOWED_AST_NODES):
            raise ValueError(f"Disallowed expression element: {type(n).__name__}")

        # block anything that could be dangerous / not intended
        if isinstance(n, (ast.Call, ast.Attribute, ast.Subscript, ast.Lambda, ast.Dict, ast.List, ast.Tuple)):
            raise ValueError(f"Disallowed expression element: {type(n).__name__}")

        if isinstance(n, ast.BinOp):
            if not isinstance(n.op, _ALLOWED_BINOPS):
                raise ValueError(f"Disallowed operator: {type(n.op).__name__}")

        if isinstance(n, ast.UnaryOp):
            if not isinstance(n.op, _ALLOWED_UNARYOPS):
                raise ValueError(f"Disallowed unary operator: {type(n.op).__name__}")


def _eval_forced_expression(expr: str, results: DesignResults) -> Tuple[bool, Optional[float], Optional[str]]:
    """
    Evaluate expression containing results.* and/or inputs.* dotted paths.

    Returns:
      (ok, value, error_message)
    """
    expr = str(expr).strip()
    if not expr:
        return False, None, "Empty expression"

    # Find all dotted path tokens in the expression
    tokens = sorted(set(_PATH_TOKEN_RE.findall(expr)), key=len, reverse=True)

    # Resolve each token to a numeric value and replace with a safe variable name
    env: Dict[str, float] = {}
    replaced = expr

    def _token_to_varname(tok: str) -> str:
        # results.wing.structure.h_m -> __p_results_wing_structure_h_m
        return "__p_" + re.sub(r"[^\w]+", "_", tok)

    for tok in tokens:
        found, val = _get_by_dotted_path(results, tok)
        if found is False and isinstance(val, tuple) and val and val[0] == "__INPUTS_PATH__":
            found2, val2 = _get_by_dotted_path(results.inputs, val[1])
            if not found2:
                return False, None, f"Path not found: {tok}"
            val = val2
            found = True

        if not found:
            return False, None, f"Path not found: {tok}"

        if not _is_finite_number(val):
            return False, None, f"Non-numeric value at path: {tok} (value={val})"

        varname = _token_to_varname(tok)
        env[varname] = float(val)

        # Replace token occurrences with varname (whole token match)
        replaced = re.sub(rf"\b{re.escape(tok)}\b", varname, replaced)

    # Parse & validate
    try:
        tree = ast.parse(replaced, mode="eval")
        _validate_expr_ast(tree)
    except Exception as e:
        return False, None, f"Invalid expression: {e}"

    # Evaluate safely (no builtins)
    try:
        value = eval(compile(tree, "<forced_expr>", "eval"), {"__builtins__": {}}, env)
    except Exception as e:
        return False, None, f"Evaluation failed: {e}"

    if not _is_finite_number(value):
        return False, None, f"Expression result is not finite: {value}"

    return True, float(value), None


# ----------------------------
# Forced export conversion
# ----------------------------
def _normalize_unit_string(unit: Optional[str]) -> Optional[str]:
    """
    Normalize unit strings:
    - None -> None
    - "" or "none" -> "unitless"
    """
    if unit is None:
        return None
    u = str(unit).strip()
    if u == "" or u.lower() in ("none", "unitless", "dimensionless"):
        return "unitless"
    return u


def _convert_forced_value(key_for_inference: str, value: float, unit: Optional[str]) -> Tuple[str, float]:
    unit = _normalize_unit_string(unit)

    if unit is None:
        inferred = _infer_unit_and_value(key_for_inference, value)
        if inferred is not None:
            return inferred
        return "mm", _mm_from_m(value)

    u = unit.strip().lower()
    if u == "mm":
        return "mm", _mm_from_m(value)
    if u == "mm2":
        return "mm2", _mm2_from_m2(value)
    if u == "mm3":
        return "mm3", _mm3_from_m3(value)
    if u == "deg":
        return "deg", float(value)
    if u == "unitless":
        return "unitless", float(value)

    # Any other string: no conversion
    return unit, float(value)


# ----------------------------
# FORCE settings (your hardcoded info preserved)
# ----------------------------
FORCE_EXPORT_KEYS = [
    "fuselage_nose_length",
    "fuselage_body_section_A_length",
    "fuselage_body_section_B_length",
    "fuselage_body_section_C_length",
    "fuselage_wing_interface_length",
    "fuselage_wing_interface_fit_clearance",
    "fuselage_wing_interface_insert_depth_factor",
    "fuselage_wing_interface_sleeve_thickness",
    "fuselage_axial_restraint_bolt_length",
    "c_root",
    "c_tip_m",
    "tail_c_root",
    "fuselage_rib_thickness",
    "fuselage_spar_rib_thickness_factor",
    "fuselage_bulkhead_inner_OD",
    "fuselage_bulkhead_radial_ring_thickness",
    "fuselage_bulkhead_thickness",
    "fuselage_B_front_bulkhead_location",
    "fuselage_B_rear_bulkhead_location",
    "fuselage_frame_thickness",
    "fuselage_stringer_thickness",
    "fuselage_stringer_height",
    "fuselage_tail_front_bulkhead_location",
    "fuselage_tail_rear_bulkhead_location",
    "fuselage_bolt_hatch_depth",
    # "fuselage_torsion_cap_shoulder_thickness",
    # "fuselage_torsion_cap_shoulder_height",
    "wing_plain_flap_chord_ratio",
    "tail_cg_root_chord_ratio",
    "tail_elevator_chord_ratio",
    "tail_elevator_span_start",
    "tail_elevator_span_end",
    "tail_elevator_rod_support_width",
    "tail_wing_stringer_thickness",
    "tail_wing_stringer_height",
    "vertical_tail_rudder_chord_ratio",
    "vertical_tail_rudder_span_start",
    "vertical_tail_rudder_span_end",
    "vertical_tail_rudder_rod_oversize_factor",
    "cg_x_wing",
    "wing_dbox_width_ratio",
    "wing_cg_root_chord_ratio",
    "wing_spar_boss_factor",
    "wing_assembly_interface_depth",
    "wing_assembly_interface_sleeve_thickness",
    "wing_second_spar_scale",
    "wing_bolt_hatch_depth",
    "hardcode_wing_secondary_spar_OD",
    "tail_dbox_width_ratio",
    "airfoil_n_points",
    "horizontal_NACA_code",
    "vertical_NACA_code",
    "N_ribs",
    "tail_N_ribs",
    "vertical_tail_N_ribs",
    "hardcode_tail_wing_spar_OD",
    "hardcode_elevator_rod_OD",
    "hardcode_wing_spar_OD",
    "assembly_fit_factor",
    "prop_nacelle_length",
    "prop_clearance_from_wing_LE",
    "prop_nacelle_groove_depth",
    "prop_housing_ID",
    "prop_nacelle_OD",
    "motor_OD",
    "motor_body_length",
    "motor_mount_bolt_base_thickness",
    "motor_mount_bolt_length",
    "motor_cabling_clearance_width",
    "motor_cabling_clearance_height",
    "motor_housing_wall_thickness",
    "motor_firewall_seat_thickness",
    "prop_nacelle_firewall_thickness",
    "motor_firewall_seat_width",
    "motor_firewall_seat_depth",
    "motor_firewall_seat_shoulder_thickness",
    "motor_mount_bolt_housing_OD",
    "threaded_insert_M2_fixing_hole_OD",
    "threaded_insert_M2_lenght",
    "threaded_insert_M3_fixing_hole_OD",
    "threaded_insert_M3_lenght",
    "threaded_insert_M3_lenght_short",
    "hole_boss_factor",
    "bolt_pocket_diameter_clearance",
    "prop_mount_bolt_circular_location",
    "threaded_insert_M4_fixing_hole_OD",
    "threaded_insert_M4_lenght",
    "servo_width",
    "servo_height",
    "servo_depth",
    "servo_cables_hole_width",
    "servo_cables_hole_height",
    "servo_arm_length",
    "servo_cover_thickness",
    "servo_cover_bolt_length",
    "servo_housing_thickness",
    "servo_mount_ears_width",
    "servo_mount_ears_thickness",
    "servo_mount_ears_location_from_top",
    "servo_mount_bolt_length",
    "camera_power_unit_length",
    "camera_power_unit_width",
    "camera_power_unit_height",
    "camera_power_unit_housing_front_part_length",
    "camera_power_unit_cooling_channel_thickness",
    "camera_lens_diameter",
    "camera_lens_length",
    "camera_body_width",
    "camera_body_length",
    "camera_location_plane",
    "camera_housing_thickness",
    "camera_body_mount_hole_location",
    "battery_length",
    "battery_width",
    "battery_height",
    "battery_location_adjustment_length",
    "battery_location_adjustment_gap",
    "battery_case_fuselage_section_B_length",
    "battery_case_thickness",
    "fuselage_longeron_boss_height",
    "fuselage_longeron_boss_thickness",
    "fuselage_assembly_fit_offset_from_top",
    "fuselage_assembly_fitting_depth",
    "fuselage_longeron_diameter",
    "fuselage_torque_frame_diameter",
    "payload_length",
    "bolt_M2_OD",
    "bolt_M2_head_width",
    "bolt_M2_hole_OD",
    "bolt_M2_head_OD",
    "bolt_M3_OD",
    "bolt_M3_head_width",
    "bolt_M3_hole_OD",
    "bolt_M3_lenght",
    "bolt_M3_head_OD",
    "spring_washer_M3_OD",
    "spring_washer_M3_thickness",
    "bolt_M4_OD",
    "bolt_M4_head_width",
    "bolt_M4_hole_OD",
    "bolt_M4_lenght",
    "bolt_M4_head_OD",
    "bolt_M5_OD",
    "bolt_M5_head_width",
    "bolt_M5_hole_OD",
    "bolt_M5_lenght",
    "bolt_M5_head_OD",
    "bolt_M6_OD",
    "bolt_M6_head_width",
    "bolt_M6_hole_OD",
    "bolt_M6_lenght",
    "bolt_M6_head_OD",
    "tube_clamp_boss_width",
    "clamp_M8_ID",
    "clamp_M8_OD",
    "clamp_M8_width",
    "clamp_M8_washer_OD",
    "clamp_M8_washer_thickness",
    "threaded_insert_M6_fixing_hole_OD",
    "threaded_insert_M6_lenght",
    "nose_axial_restraint_boss_width",
    "nose_axial_restraint_bolt_lenght",
    "transition_fit_clearance",
    'transition_fit_clearance_coated_carbon',
    "printed_vertical_small_hole_additional_clearance",
    "cubepilot_height",
    "cubepilot_width",
    "cubepilot_depth",
    "cubepilot_pin_section_height",
]

FORCE_EXPORT_UNITS = {
    # If you want dimensionless, use "" or "unitless"
    "fuselage_nose_length": "mm",
    "fuselage_body_section_A_length": "mm",
    "fuselage_body_section_B_length": "mm",
    "fuselage_body_section_C_length": "mm",
    "fuselage_wing_interface_length": "mm",
    "fuselage_wing_interface_fit_clearance": "mm",
    "fuselage_wing_interface_insert_depth_factor": "unitless",
    "fuselage_wing_interface_sleeve_thickness": "mm",
    "fuselage_axial_restraint_bolt_length": "mm",
    "wing_plain_flap_chord_ratio": "unitless",
    "wing_second_spar_scale": "unitless",
    "wing_bolt_hatch_depth": "mm",
    "hardcode_wing_secondary_spar_OD": "mm",
    "tail_cg_root_chord_ratio": "unitless",
    "fuselage_rib_thickness": "mm",
    "fuselage_bulkhead_thickness": "mm",
    "fuselage_bulkhead_inner_OD": "mm",
    "fuselage_bulkhead_radial_ring_thickness": "mm",
    "fuselage_B_front_bulkhead_location": "mm",
    "fuselage_B_rear_bulkhead_location": "mm",
    "fuselage_frame_thickness": "mm",
    "fuselage_stringer_thickness": "mm",
    "fuselage_stringer_height": "mm",
    "fuselage_tail_front_bulkhead_location": "mm",
    "fuselage_tail_rear_bulkhead_location": "mm",
    "fuselage_bolt_hatch_depth": "mm",
    "tail_elevator_chord_ratio": "unitless",
    "tail_elevator_span_start": "unitless",
    "tail_elevator_span_end": "unitless",
    "tail_elevator_rod_support_width": "mm",
    "tail_wing_stringer_thickness": "mm",
    "tail_wing_stringer_height": "mm",
    "vertical_tail_rudder_chord_ratio": "unitless",
    "vertical_tail_rudder_span_start": "unitless",
    "vertical_tail_rudder_span_end": "unitless",
    "vertical_tail_rudder_rod_oversize_factor": "unitless",
    "wing_dbox_width_ratio": "unitless",
    "wing_cg_root_chord_ratio": "unitless",
    "wing_spar_boss_factor": "unitless",
    "wing_assembly_interface_depth": "mm",
    "wing_assembly_interface_sleeve_thickness": "mm",
    "tail_dbox_width_ratio": "unitless",
    "airfoil_n_points": "unitless",
    "horizontal_NACA_code": "unitless",
    "vertical_NACA_code": "unitless",
    "N_ribs": "unitless",
    "tail_N_ribs": "unitless",
    "vertical_tail_N_ribs": "unitless",
    "assembly_fit_factor": "unitless",
    "hardcode_wing_spar_OD": "mm",
    "hardcode_tail_wing_spar_OD": "mm",
    "hardcode_elevator_rod_OD": "mm",
    "prop_housing_ID": "mm",
    "prop_nacelle_OD": "mm",
    "motor_OD": "mm",
    "motor_mount_bolt_housing_OD": "mm",
    "motor_body_length": "mm",
    "motor_mount_bolt_base_thickness": "mm",
    "motor_mount_bolt_length": "mm",
    "motor_cabling_clearance_width": "mm",
    "motor_cabling_clearance_height": "mm",
    "motor_housing_wall_thickness": "mm",
    "motor_firewall_seat_thickness": "mm",
    "motor_firewall_seat_width": "mm",
    "motor_firewall_seat_depth": "mm",
    "motor_firewall_seat_shoulder_thickness": "mm",
    "prop_nacelle_groove_depth": "mm",
    "hole_boss_factor": "unitless",
    "bolt_pocket_diameter_clearance": "mm",
    "prop_mount_bolt_circular_location": "mm",
    "threaded_insert_M2_fixing_hole_OD": "mm",
    "threaded_insert_M2_lenght": "mm",
    "threaded_insert_M3_fixing_hole_OD": "mm",
    "threaded_insert_M3_lenght": "mm",
    "threaded_insert_M3_lenght_short": "mm",
    "threaded_insert_M4_fixing_hole_OD": "mm",
    "threaded_insert_M4_lenght": "mm",
    "threaded_insert_M5_fixing_hole_OD": "mm",
    "threaded_insert_M5_lenght": "mm",
    "servo_cables_hole_width": "mm",
    "servo_cables_hole_height": "mm",
    "servo_width": "mm",
    "servo_height": "mm",
    "servo_depth": "mm",
    "servo_arm_length": "mm",
    "servo_cover_thickness": "mm",
    "servo_cover_bolt_length": "mm",
    "servo_housing_thickness": "mm",
    "servo_mount_ears_width": "mm",
    "servo_mount_ears_thickness": "mm",
    "servo_mount_ears_location_from_top": "mm",
    "servo_mount_bolt_length": "mm",
    "camera_power_unit_length": "mm",
    "camera_power_unit_width": "mm",
    "camera_power_unit_height": "mm",
    "camera_power_unit_housing_front_part_length": "mm",
    "camera_power_unit_cooling_channel_thickness": "mm",
    "camera_lens_diameter": "mm",
    "camera_lens_length": "mm",
    "camera_body_width": "mm",
    "camera_body_length": "mm",
    "camera_location_plane": "mm",
    "camera_housing_thickness": "mm",
    "camera_body_mount_hole_location": "mm",
    "battery_length": "mm",
    "battery_width": "mm",
    "battery_height": "mm",
    "battery_location_adjustment_length": "mm",
    "battery_location_adjustment_gap": "mm",
    "battery_case_fuselage_section_B_length": "mm",
    "battery_case_thickness": "mm",
    "fuselage_longeron_boss_height": "mm",
    "fuselage_longeron_boss_thickness": "mm",
    "fuselage_assembly_fit_offset_from_top": "mm",
    "fuselage_assembly_fitting_depth": "mm",
    "fuselage_longeron_diameter": "mm",
    "fuselage_torque_frame_diameter": "mm",
    "payload_length": "mm",
    "bolt_M2_OD": "mm",
    "bolt_M2_head_width": "mm",
    "bolt_M2_hole_OD": "mm",
    "bolt_M2_head_OD": "mm",
    "bolt_M3_OD": "mm",
    "bolt_M3_head_width": "mm",
    "bolt_M3_hole_OD": "mm",
    "bolt_M3_head_OD": "mm",
    "spring_washer_M3_OD": "mm",
    "spring_washer_M3_thickness": "mm",
    "cubepilot_height": "mm",
    "cubepilot_width": "mm",
    "cubepilot_depth": "mm",
    "cubepilot_pin_section_height": "mm",
    "bolt_M4_OD": "mm",
    "bolt_M4_head_width": "mm",
    "bolt_M4_hole_OD": "mm",
    "bolt_M4_lenght": "mm",
    "bolt_M4_head_OD": "mm",
    "bolt_M5_OD": "mm",
    "bolt_M5_head_width": "mm",
    "bolt_M5_hole_OD": "mm",
    "bolt_M5_lenght": "mm",
    "bolt_M5_head_OD": "mm",
    "bolt_M6_OD": "mm",
    "bolt_M6_head_width": "mm",
    "bolt_M6_hole_OD": "mm",
    "bolt_M6_lenght": "mm",
    "bolt_M6_head_OD": "mm",
    "tube_clamp_boss_width": "mm",
    "clamp_M8_ID": "mm",
    "clamp_M8_OD": "mm",
    "clamp_M8_width": "mm",
    "clamp_M8_washer_OD": "mm",
    "clamp_M8_washer_thickness": "mm",
    "threaded_insert_M6_fixing_hole_OD": "mm",
    "threaded_insert_M6_lenght": "mm",
    "nose_axial_restraint_boss_width": "mm",
    "nose_axial_restraint_bolt_lenght": "mm",
    "transition_fit_clearance": "mm",
    "transition_fit_clearance_coated_carbon": "mm",
    "printed_vertical_small_hole_additional_clearance": "mm",
}

# Paths / Expressions:
# Item format:
#   - "results.x.y.z"
#   - ("alias_name", "results.x.y.z", "mm2")
#   - ("alias_name", "results.a/b", "mm")  <-- expression supported
FORCE_EXPORT_PATHS: List[Union[str, Tuple[str, str, str]]] = [
    # Example path:
    # VERTICAL MAIN WING REMOVED - entries removed
    ("vertical_tail_t_skin_root", "results.vertical_tail.structure.t_skin_root_m", "mm"),
    ("vertical_tail_t_skin_tip", "results.vertical_tail.structure.t_skin_tip_m", "mm"),

    # VERTICAL MAIN WING REMOVED - entries removed
    ("vertical_tail_t_web_root", "results.vertical_tail.structure.t_web_root_m", "mm"),
    ("vertical_tail_t_web_tip", "results.vertical_tail.structure.t_web_tip_m", "mm"),

    ("h_max_thickness", "results.wing.structure.h_m", "mm"),
    # VERTICAL MAIN WING REMOVED - entry removed
    ("h_tail_max_thickness", "results.tail.structure.h_m", "mm"),
    ("h_tail_vertical_max_thickness", "results.vertical_tail.structure.h_m", "mm"),
    
    ("wing_spar_OD_root", "results.wing.wing_spar_OD_root_m", "mm"),
    ("wing_spar_ID_root", "results.wing.wing_spar_ID_root_m", "mm"),
    ("wing_spar_OD_tip", "results.wing.wing_spar_OD_tip_m", "mm"),
    ("wing_spar_ID_tip", "results.wing.wing_spar_ID_tip_m", "mm"),

    ("tail_spar_OD_root", "results.tail.tail_spar_OD_root_m", "mm"),
    ("tail_spar_ID_root", "results.tail.tail_spar_ID_root_m", "mm"),
    ("tail_spar_OD_tip", "results.tail.tail_spar_OD_tip_m", "mm"),
    ("tail_spar_ID_tip", "results.tail.tail_spar_ID_tip_m", "mm"),
    ("elevator_rod_OD", "results.tail.elevator_rod_OD_m", "mm"),
    ("elevator_rod_ID", "results.tail.elevator_rod_ID_m", "mm"),

    ("fuselage_nose_longeron_boss_height", "results.fuselage.fuselage_nose_longeron_boss_height_m", "mm"),

    # Rib spacing (calculated in structures.py)
    ("rib_spacing", "results.wing.structure.rib_spacing_m", "mm"),
    ("rib_spacing_tail", "results.tail.structure.rib_spacing_m", "mm"),
    ("rib_spacing_vertical_tail", "results.vertical_tail.structure.rib_spacing_m", "mm"),
    # If you omit unit override, it will infer from suffix (_m2 -> mm2), else default mm.
]


# ----------------------------
# Collector
# ----------------------------
def _collect_params(
    results: DesignResults,
    *,
    export_all_numeric: bool,
    extra_params: Optional[List[Tuple[str, str, float, str]]],
    force_export_keys: Optional[Iterable[str]],
    force_export_units: Optional[Dict[str, str]],
    force_export_paths: Optional[Iterable[Union[str, Tuple[str, str, str]]]],
    force_missing: str,
) -> List[Tuple[str, str, float, str]]:
    """Return list of (name, unit, value_num, comment)."""
    model_flat: Dict[str, Any] = results.to_dict()
    model_flat.pop("inputs", None)

    inputs_flat = _inputs_to_flat_dict(results.inputs)

    combined: Dict[str, Any] = {}
    combined.update(model_flat)
    combined.update(inputs_flat)

    used_names = set()
    out: List[Tuple[str, str, float, str]] = []

    def add_unique(original_name: str, unit: str, val: float, comment: str) -> None:
        cleaned = _strip_unit_suffix(original_name)
        base = _safe_param_name(cleaned)
        nm = base
        i = 2
        while nm in used_names:
            nm = f"{base}_{i}"
            i += 1
        used_names.add(nm)
        out.append((nm, unit, float(val), comment))

    # Forced exports (flat keys)
    force_export_units = force_export_units or {}
    forced = list(force_export_keys) if force_export_keys else []
    forced_found = set()

    for k in forced:
        if k not in combined:
            continue
        v = combined[k]
        if not _is_finite_number(v):
            continue
        unit, conv_val = _convert_forced_value(k, float(v), force_export_units.get(k))
        add_unique(k, unit, conv_val, "FORCED export (key)")
        forced_found.add(k)

    # Forced exports (paths/expressions)
    forced_paths = list(force_export_paths) if force_export_paths else []
    forced_paths_missing: List[str] = []

    for item in forced_paths:
        if isinstance(item, tuple):
            alias, spec, unit_override = item
            alias = str(alias).strip() if alias else ""
            spec = str(spec).strip()
            unit_override = str(unit_override).strip() if unit_override is not None else None
        else:
            alias, spec, unit_override = "", str(item).strip(), None

        if not spec:
            forced_paths_missing.append(spec)
            continue

        # Decide: is it a plain path or expression?
        is_expression = any(op in spec for op in ("+", "-", "*", "/", "(", ")"))

        if is_expression:
            ok, val, err = _eval_forced_expression(spec, results)
            if not ok or val is None:
                forced_paths_missing.append(spec)
                continue

            export_name = alias if alias else "expr_value"
            unit, conv_val = _convert_forced_value(export_name, float(val), unit_override)
            add_unique(export_name, unit, conv_val, f"FORCED export (expr): {spec}")
            continue

        # Plain dotted path
        found, val = _get_by_dotted_path(results, spec)
        if found is False and isinstance(val, tuple) and val and val[0] == "__INPUTS_PATH__":
            found2, val2 = _get_by_dotted_path(results.inputs, val[1])
            if not found2:
                forced_paths_missing.append(spec)
                continue
            found, val = True, val2

        if not found or not _is_finite_number(val):
            forced_paths_missing.append(spec)
            continue

        default_name = spec.split(".")[-1]
        export_name = alias if alias else default_name

        unit, conv_val = _convert_forced_value(default_name, float(val), unit_override)
        add_unique(export_name, unit, conv_val, f"FORCED export (path): {spec}")

    # Missing forced handling
    if forced:
        missing = [k for k in forced if k not in forced_found]
        if missing and force_missing.lower() == "raise":
            raise KeyError(f"Forced export keys not found in results/inputs: {missing}")
        if missing and force_missing.lower() == "warn":
            print(f"[Inventor Export] Warning: forced KEYS not found (or non-numeric): {missing}")

    if forced_paths_missing:
        if force_missing.lower() == "raise":
            raise KeyError(f"Forced export PATHS/EXPRS not found (or non-numeric): {forced_paths_missing}")
        if forced_paths_missing and force_missing.lower() == "warn":
            print(f"[Inventor Export] Warning: forced PATHS/EXPRS not found (or non-numeric): {forced_paths_missing}")

    # Auto export
    for key in sorted(combined.keys()):
        if key in forced_found:
            continue

        val = combined[key]
        if not _is_finite_number(val):
            continue

        if not export_all_numeric and _should_skip_key(key):
            continue

        if export_all_numeric:
            inferred = _infer_unit_and_value(key, float(val))
            if inferred is None:
                add_unique(key, "unitless", float(val), "Auto-export (numeric)")
            else:
                unit, conv_val = inferred
                add_unique(key, unit, conv_val, "Auto-export (numeric)")
            continue

        inferred = _infer_unit_and_value(key, float(val))
        if inferred is None:
            continue
        unit, conv_val = inferred
        add_unique(key, unit, conv_val, "Auto-export (dimension)")

    # Extras last
    if extra_params:
        for nm, unit, val, com in extra_params:
            add_unique(nm, unit, float(val), com or "User extra param")

    return out


# ----------------------------
# Writers
# ----------------------------
def export_inventor_parameters_xml(
    results: DesignResults,
    out_xml_path: str,
    *,
    export_all_numeric: bool = False,
    extra_params: Optional[List[Tuple[str, str, float, str]]] = None,
    force_export_keys: Optional[Iterable[str]] = None,
    force_export_units: Optional[Dict[str, str]] = None,
    force_export_paths: Optional[Iterable[Union[str, Tuple[str, str, str]]]] = None,
    force_missing: str = "warn",
) -> str:
    out_xml_path = os.path.abspath(out_xml_path)
    os.makedirs(os.path.dirname(out_xml_path), exist_ok=True)

    params = _collect_params(
        results,
        export_all_numeric=export_all_numeric,
        extra_params=extra_params,
        force_export_keys=FORCE_EXPORT_KEYS if force_export_keys is None else force_export_keys,
        force_export_units=FORCE_EXPORT_UNITS if force_export_units is None else force_export_units,
        force_export_paths=FORCE_EXPORT_PATHS if force_export_paths is None else force_export_paths,
        force_missing=force_missing,
    )

    root = ET.Element(
        "ParamWithValueList",
        attrib={
            "xmlns:xsd": "http://www.w3.org/2001/XMLSchema",
            "xmlns:xsi": "http://www.w3.org/2001/XMLSchema-instance",
        },
    )
    ET.SubElement(root, "version").text = "20080502"

    parameter_types = ET.SubElement(root, "parameterTypes")
    for name, code in (("Inventor", "0"), ("String", "1"), ("Boolean", "2")):
        pt = ET.SubElement(parameter_types, "ParamType")
        ET.SubElement(pt, "typeName").text = name
        ET.SubElement(pt, "typeCode").text = code

    params_node = ET.SubElement(root, "parameters")

    for (name, unit, value_num, comment) in params:
        pwv = ET.SubElement(params_node, "ParamWithValue")
        ET.SubElement(pwv, "name").text = name
        ET.SubElement(pwv, "typeCode").text = unit
        ET.SubElement(pwv, "value").text = _fmt_value(float(value_num), unit)
        c = ET.SubElement(pwv, "comment")
        c.text = comment if comment else ""
        ET.SubElement(pwv, "isKey").text = "false"

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ", level=0)
    tree.write(out_xml_path, encoding="utf-8", xml_declaration=True)
    return out_xml_path


def export_inventor_parameters_xlsx(
    results: DesignResults,
    out_xlsx_path: str,
    *,
    export_all_numeric: bool = False,
    extra_params: Optional[List[Tuple[str, str, float, str]]] = None,
    force_export_keys: Optional[Iterable[str]] = None,
    force_export_units: Optional[Dict[str, str]] = None,
    force_export_paths: Optional[Iterable[Union[str, Tuple[str, str, str]]]] = None,
    force_missing: str = "warn",
) -> str:
    out_xlsx_path = os.path.abspath(out_xlsx_path)
    os.makedirs(os.path.dirname(out_xlsx_path), exist_ok=True)

    params = _collect_params(
        results,
        export_all_numeric=export_all_numeric,
        extra_params=extra_params,
        force_export_keys=FORCE_EXPORT_KEYS if force_export_keys is None else force_export_keys,
        force_export_units=FORCE_EXPORT_UNITS if force_export_units is None else force_export_units,
        force_export_paths=FORCE_EXPORT_PATHS if force_export_paths is None else force_export_paths,
        force_missing=force_missing,
    )

    wb = Workbook()
    ws = wb.active
    ws.title = "Parameters"

    ws["A1"] = "Name"
    ws["B1"] = "Value"
    ws["C1"] = "Unit"
    ws["D1"] = "Comment"

    for i, (name, unit, value_num, comment) in enumerate(params, start=2):
        ws[f"A{i}"].value = name
        ws[f"B{i}"].value = float(value_num)      # numeric only
        ws[f"B{i}"].number_format = "0.########"  # robust across locales
        ws[f"C{i}"].value = unit
        ws[f"D{i}"].value = comment

    ws.column_dimensions["A"].width = 45
    ws.column_dimensions["B"].width = 18
    ws.column_dimensions["C"].width = 10
    ws.column_dimensions["D"].width = 45

    # Atomic write (important with OneDrive)
    tmp_dir = os.path.dirname(out_xlsx_path) or "."
    with tempfile.NamedTemporaryFile(delete=False, suffix=".xlsx", dir=tmp_dir) as tf:
        tmp_path = tf.name
    try:
        wb.save(tmp_path)
        os.replace(tmp_path, out_xlsx_path)
    finally:
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except OSError:
                pass

    return out_xlsx_path


def export_inventor_parameters(
    results: DesignResults,
    out_base_path: str,
    *,
    write_xml: bool = True,
    write_xlsx: bool = True,
    export_all_numeric: bool = False,
    extra_params: Optional[List[Tuple[str, str, float, str]]] = None,
    force_export_keys: Optional[Iterable[str]] = FORCE_EXPORT_KEYS,
    force_export_units: Optional[Dict[str, str]] = FORCE_EXPORT_UNITS,
    force_export_paths: Optional[Iterable[Union[str, Tuple[str, str, str]]]] = FORCE_EXPORT_PATHS,
    force_missing: str = "warn",
) -> Dict[str, str]:
    out_base_path = os.path.abspath(out_base_path)
    os.makedirs(os.path.dirname(out_base_path), exist_ok=True)

    out: Dict[str, str] = {}
    if write_xml:
        out["xml"] = export_inventor_parameters_xml(
            results,
            out_base_path + ".xml",
            export_all_numeric=export_all_numeric,
            extra_params=extra_params,
            force_export_keys=force_export_keys,
            force_export_units=force_export_units,
            force_export_paths=force_export_paths,
            force_missing=force_missing,
        )
    if write_xlsx:
        out["xlsx"] = export_inventor_parameters_xlsx(
            results,
            out_base_path + ".xlsx",
            export_all_numeric=export_all_numeric,
            extra_params=extra_params,
            force_export_keys=force_export_keys,
            force_export_units=force_export_units,
            force_export_paths=force_export_paths,
            force_missing=force_missing,
        )
    return out
