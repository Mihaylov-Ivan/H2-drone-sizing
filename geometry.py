"""
Geometry calculations for wing and tail surfaces.
"""

import math
from typing import Tuple
from design_inputs import DesignInputs
from constants import ATMOSPHERIC


def calculate_trapezoidal_geometry(
    W_lift_N: float,
    c_root: float,
    taper_ratio: float,
    wing_loading: float,
) -> Tuple[float, float, float, float, float, float, float, float, float]:
    """
    Calculate trapezoidal wing geometry from root chord, taper ratio, and wing loading.
    
    Args:
        W_lift_N: Lift force [N]
        c_root: Root chord [m]
        taper_ratio: Taper ratio λ = c_tip / c_root
        wing_loading: Wing loading W/S [N/m²]
    
    Returns:
        Tuple of (S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC)
    """
    # Calculate wing area from wing loading
    S = W_lift_N / wing_loading
    
    # Calculate span from trapezoidal wing area formula: S = b * (c_root + c_tip) / 2
    # Rearranging: b = 2*S / (c_root + c_tip) = 2*S / (c_root * (1 + λ))
    b = 2.0 * S / (c_root * (1.0 + taper_ratio))
    
    # Calculate tip chord
    c_tip = taper_ratio * c_root
    
    # Calculate aspect ratio
    AR = b**2 / S
    
    # Calculate mean aerodynamic chord (standard trapezoidal wing formula)
    MAC = (2.0 / 3.0) * c_root * (1.0 + taper_ratio + taper_ratio**2) / (1.0 + taper_ratio)
    
    # Spanwise position of MAC from centerline
    y_MAC = (b / 6.0) * (1.0 + 2.0 * taper_ratio) / (1.0 + taper_ratio)
    
    # Mean geometric chord
    c_MGC = S / b
    
    # Aerodynamic center position (25% of MAC)
    ac_on_MAC = 0.25 * MAC
    
    return S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC


def wing_geometry(W_lift_N: float, inputs: DesignInputs) -> Tuple[float, float, float, float, float, float, float, float, float]:
    """
    Calculate wing geometry from design inputs.

    Actual manufactured span = design span + fuselage_OD (spar passes through fuselage).
    All estimates use this extended span.
    """
    S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC = calculate_trapezoidal_geometry(
        W_lift_N,
        inputs.c_root,
        inputs.taper_ratio,
        inputs.wing_loading,
    )
    # Fixed wing span: same convention as computed span (full physical span b [m]).
    # Area follows from trapezoid (c_root, taper); wing_loading does not set span.
    if getattr(inputs, "use_hardcoded_values", False):
        hc_span = getattr(inputs, "hardcode_wing_span", None)
        if hc_span is not None:
            b = float(hc_span)
            c_tip = float(inputs.taper_ratio) * float(inputs.c_root)
            S = b * (float(inputs.c_root) + c_tip) / 2.0
            AR = b**2 / S if S > 0 else 0.0
            lam = float(inputs.taper_ratio)
            y_MAC = (b / 6.0) * (1.0 + 2.0 * lam) / (1.0 + lam)
            c_MGC = S / b if b > 0 else 0.0
            return S, b, float(inputs.c_root), c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC

    fuselage_OD = float(getattr(inputs, "fuselage_OD", 0.0))
    b = b + fuselage_OD
    S = b * (c_root + c_tip) / 2.0
    AR = b**2 / S if S > 0 else 0.0
    lam = inputs.taper_ratio
    y_MAC = (b / 6.0) * (1.0 + 2.0 * lam) / (1.0 + lam)
    c_MGC = S / b if b > 0 else 0.0
    return S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC


def tail_geometry(x_cg: float, MAC_wing: float, S_wing: float, inputs: DesignInputs) -> Tuple[float, float, float, float, float, float, float, float, float]:
    """
    Calculate tail geometry from design inputs and wing properties.
    
    Args:
        x_cg: Center of gravity x-position [m]
        MAC_wing: Wing mean aerodynamic chord [m]
        S_wing: Wing area [m²]
        inputs: Design inputs
    
    Returns:
        Tuple of (S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC)
    
    Raises:
        ValueError: If tail AC is not behind CG
    """
    lam = inputs.tail_taper_ratio
    c_root = inputs.tail_c_root
    
    # Calculate MAC and AC position
    MAC = (2.0 / 3.0) * c_root * (1.0 + lam + lam**2) / (1.0 + lam)
    ac_on_MAC = 0.25 * MAC
    x_ac_tail = inputs.cg_x_tail - 0.5 * c_root + ac_on_MAC
    
    l_cg_to_ac = x_ac_tail - x_cg
    
    if l_cg_to_ac <= 0:
        raise ValueError(
            f"Tail AC is not behind CG (l_cg_to_ac = {l_cg_to_ac:.4f} m). "
            "Tail volume coefficient undefined for this geometry."
        )

    # Fixed horizontal tail span: same convention as computed span (full physical span b [m]).
    # Area follows from trapezoid (c_root, taper); tail_volume_ratio does not set span.
    if getattr(inputs, "use_hardcoded_values", False):
        hc_span = getattr(inputs, "hardcode_tail_span", None)
        if hc_span is not None:
            b = float(hc_span)
            c_tip = lam * c_root
            S = b * (c_root + c_tip) / 2.0
            AR = b**2 / S if S > 0 else 0.0
            y_MAC = (b / 6.0) * (1.0 + 2.0 * lam) / (1.0 + lam)
            c_MGC = S / b if b > 0 else 0.0
            return S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC

    # Calculate tail area from volume ratio
    S = inputs.tail_volume_ratio * MAC_wing * S_wing / l_cg_to_ac
    
    # Use generic function for remaining calculations
    _, b, _, c_tip, _, y_MAC, AR, c_MGC, _ = calculate_trapezoidal_geometry(
        S * inputs.wing_loading,  # Convert area to equivalent lift for calculation
        c_root,
        lam,
        inputs.wing_loading,
    )
    
    # Recalculate with correct area
    b = 2.0 * S / (c_root * (1.0 + lam))
    c_tip = lam * c_root

    # Twin-boom configuration: the horizontal tailplane is one continuous surface.
    # The volume-coefficient area already represents the full planform, so no
    # central-fuselage diameter is added to its span.
    if inputs.tail_span_extension_m > 0.0:
        b = b + float(inputs.tail_span_extension_m)

    S = b * (c_root + c_tip) / 2.0
    AR = b**2 / S if S > 0 else 0.0
    y_MAC = (b / 6.0) * (1.0 + 2.0 * lam) / (1.0 + lam)
    c_MGC = S / b if b > 0 else 0.0

    return S, b, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC

def vertical_tail_geometry(
    x_cg: float,
    b_wing: float,
    S_wing: float,
    inputs: DesignInputs,
) -> Tuple[float, float, float, float, float, float, float, float, float]:
    """
    Size the twin vertical tails from the total vertical-tail volume coefficient:
      Vv = (Sv_total * l_vt) / (S_w * b_w)

    The returned geometry is for ONE fin. Total area and drag are obtained by
    multiplying by inputs.n_vertical_tail_surfaces.
    """
    lam = float(inputs.tail_taper_ratio)
    c_root = float(inputs.tail_c_root)

    # MAC & AC on MAC (for x_ac estimate)
    MAC = (2.0 / 3.0) * c_root * (1.0 + lam + lam**2) / (1.0 + lam)
    ac_on_MAC = 0.25 * MAC

    # Assume vertical tail AC x-location same as horizontal tail (cg_x_vertical_tail)
    x_ac_vt = float(inputs.cg_x_vertical_tail) - 0.5 * c_root + ac_on_MAC
    l_cg_to_ac = x_ac_vt - float(x_cg)

    if l_cg_to_ac <= 0.0:
        raise ValueError(
            f"Vertical tail AC is not behind CG (l_cg_to_ac = {l_cg_to_ac:.4f} m). "
            "Vertical tail volume coefficient undefined for this geometry."
        )

    n_fins = max(1, int(getattr(inputs, "n_vertical_tail_surfaces", 2)))

    # A hardcoded vertical-tail span is interpreted as the height of ONE fin.
    if getattr(inputs, "use_hardcoded_values", False):
        hc_span = getattr(inputs, "hardcode_vertical_tail_span", None)
        if hc_span is not None:
            b_v = float(hc_span)
            c_tip = lam * c_root
            Sv_fin = b_v * (c_root + c_tip) / 2.0
            AR = b_v**2 / Sv_fin if Sv_fin > 0 else 0.0
            y_MAC = (b_v / 3.0) * (1.0 + 2.0 * lam) / (1.0 + lam)
            c_MGC = Sv_fin / b_v if b_v > 0 else 0.0
            return Sv_fin, b_v, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC

    # Total area from Vv, then divide equally between the two fins.
    Sv_total = float(inputs.vertical_tail_volume_ratio) * float(S_wing) * float(b_wing) / l_cg_to_ac
    Sv_fin = Sv_total / n_fins
    c_tip = lam * c_root
    b_v = 2.0 * Sv_fin / (c_root * (1.0 + lam))

    # Optional physical height extension applies to each fin.
    if float(inputs.tail_span_extension_m) > 0.0:
        b_v = b_v + float(inputs.tail_span_extension_m)
        Sv_fin = b_v * (c_root + c_tip) / 2.0

    AR = b_v**2 / Sv_fin if Sv_fin > 0 else 0.0
    # A fin is a single root-to-tip trapezoid, not a mirrored full wing.
    y_MAC = (b_v / 3.0) * (1.0 + 2.0 * lam) / (1.0 + lam)
    c_MGC = Sv_fin / b_v if b_v > 0 else 0.0

    return Sv_fin, b_v, c_root, c_tip, MAC, y_MAC, AR, c_MGC, ac_on_MAC


def lifting_surface_wetted_area_estimated_m2(planform_area_m2: float) -> float:
    """
    Approximate wetted area for skin-friction scaling on a thin lifting surface:
    upper + lower exposed mold lines → 2 × planform area [m²].
    """
    return 2.0 * float(planform_area_m2)
