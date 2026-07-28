"""
Structural calculations for wing, tail, and fuselage.

Updates vs your current script:
- Wing/tail spar bending sizing is now based on real tube beam bending (sigma = M*c/I),
  using id_over_od (hollow tube), not a solid/area-only proxy.
- Adds conservative rib->tube load transfer (bearing/crush proxy) checks for reliability.
- Elevator hinge rod remains tube-sized by bending (already), and now also gets:
  - explicit stress back-check
  - conservative support/bearing check
"""

from __future__ import annotations

import math
from typing import Tuple, Optional, Dict, Any

from design_inputs import DesignInputs
from constants import ATMOSPHERIC, MATERIAL
from results import Structure


def _get_hardcoded_float(inputs: DesignInputs, attr: str, calculated: float) -> float:
    """If use_hardcoded_values is True and inputs.<attr> is not None, return it; else return calculated."""
    if getattr(inputs, "use_hardcoded_values", False):
        val = getattr(inputs, attr, None)
        if val is not None:
            return float(val)
    return calculated


def _apply_hardcoded_surface_skin_root_thickness(
    structure: Structure,
    spar_results: Dict[str, float],
    inputs: DesignInputs,
    *,
    b: float,
    c_root: float,
    taper_ratio: float,
    max_thickness_fraction: float,
    flap_chord_ratio: float,
    hardcode_attr: str,
) -> None:
    """Replace sized skin thickness and recompute skin, rib, and stiffener masses when hardcode is set."""
    if not getattr(inputs, "use_hardcoded_values", False):
        return
    val = getattr(inputs, hardcode_attr, None)
    if val is None:
        return

    t_skin_root = float(val)
    t_skin_tip = max(t_skin_root * MATERIAL.tip_thickness_ratio, MATERIAL.min_skin_thickness_m)

    h = c_root * max_thickness_fraction
    c_cell_root = c_root * flap_chord_ratio
    A_cell_root = 2.0 * c_cell_root * h / 3.0
    s_cell_root = (
        math.sqrt((h / 2.0) ** 2 + 4.0 * c_cell_root**2)
        + (h**2 / (16.0 * c_cell_root)) * math.asinh(4.0 * c_cell_root / max(h, 1e-9))
    )

    torsion = float(structure.torsion_Nm)
    rib_spacing_m = float(structure.rib_spacing_m)
    N_ribs = float(structure.N_ribs)

    old_W_skin = float(structure.W_skin_kg)
    old_W_ribs = float(structure.W_ribs_kg)
    old_W_stiff = float(spar_results.get("W_stiffeners_kg", 0.0))

    W_skin = inputs.filament_density * b * (t_skin_root + t_skin_tip) * s_cell_root * (1.0 + taper_ratio) / 8.0
    W_ribs = (
        N_ribs
        * inputs.filament_density
        * A_cell_root
        * (1.0 + taper_ratio**2)
        * (t_skin_root + t_skin_tip)
        / 4.0
    )

    stiffener_E_Pa = float(getattr(inputs, "filament_E_Pa", 2.5e9))
    stiffener_nu = float(getattr(inputs, "filament_nu", 0.35))
    stiffener_margin = float(getattr(inputs, "stiffener_panel_buckling_margin", 0.50))
    stiffener_ks = float(getattr(inputs, "stiffener_panel_k_shear", 5.5))

    stiffener_results = _estimate_skin_stiffeners(
        torsion_Nm=torsion,
        A_cell_m2=A_cell_root,
        c_box_m=c_cell_root,
        t_skin_m=t_skin_root,
        rib_spacing_m=rib_spacing_m,
        h_m=h,
        E_skin_Pa=stiffener_E_Pa,
        nu_skin=stiffener_nu,
        density_skin_kg_m3=inputs.filament_density,
        panel_buckling_margin=stiffener_margin,
        k_shear=stiffener_ks,
        stiffener_t_min_factor=float(getattr(inputs, "stiffener_t_min_factor", 1.0)),
        stiffener_t_max_factor=float(getattr(inputs, "stiffener_t_max_factor", 2.0)),
        stiffener_h_min_factor=float(getattr(inputs, "stiffener_h_min_factor", 2.0)),
        stiffener_h_max_factor=float(getattr(inputs, "stiffener_h_max_factor", 4.0)),
    )

    stiffener_run_m = max(0.0, (b - inputs.fuselage_OD) / 2.0)
    n_stiff = int(stiffener_results["N_stiffeners"])
    A_st = stiffener_results["area_one_m2"]
    W_stiffeners = inputs.filament_density * n_stiff * A_st * stiffener_run_m

    if stiffener_results["panel_ok"] < 0.5:
        print(
            f"WARNING [STIFFENERS]: torsional skin shear-buckling check not satisfied. "
            f"tau_skin={stiffener_results['tau_skin_MPa']:.2f} MPa, "
            f"tau_cr={stiffener_results['tau_cr_panel_MPa']:.2f} MPa. "
            f"Increase skin thickness or add more stiffeners."
        )

    structure.t_skin_root_m = t_skin_root
    structure.t_skin_tip_m = t_skin_tip
    structure.W_skin_kg = W_skin
    structure.W_ribs_kg = W_ribs
    structure.W_total_kg = (
        structure.W_total_kg - old_W_skin - old_W_ribs - old_W_stiff + W_skin + W_ribs + W_stiffeners
    )

    spar_results["rib_contact_L_m"] = t_skin_root
    spar_results["N_stiffeners"] = stiffener_results["N_stiffeners"]
    spar_results["stiffener_spacing_m"] = stiffener_results["spacing_m"]
    spar_results["stiffener_thickness_m"] = stiffener_results["thickness_m"]
    spar_results["stiffener_height_m"] = stiffener_results["height_m"]
    spar_results["stiffener_area_one_mm2"] = stiffener_results["area_one_m2"] * 1e6
    spar_results["stiffener_I_one_mm4"] = stiffener_results["I_one_m4"] * 1e12
    spar_results["stiffener_tau_skin_MPa"] = stiffener_results["tau_skin_MPa"]
    spar_results["stiffener_tau_cr_panel_MPa"] = stiffener_results["tau_cr_panel_MPa"]
    spar_results["stiffener_q_shear_N_per_m"] = stiffener_results["q_shear_N_per_m"]
    spar_results["stiffener_aspect_ratio"] = stiffener_results["aspect_ratio"]
    spar_results["W_stiffeners_kg"] = W_stiffeners
    spar_results["stiffener_panel_ok"] = stiffener_results["panel_ok"]


# ============================================================
# Propulsion loads (unchanged)
# ============================================================
def calculate_propulsion_loads(
    propulsion_selection: Optional[Dict[str, Any]],
    prop_y_position_fraction: float,
    prop_x_position_fraction: float,
    wing_span_m: float,
    c_root: float,
    taper_ratio: float,
    inputs: DesignInputs,
) -> Dict[str, float]:
    """
    Calculate propulsion loads on wing: weight moments, thrust moments, and motor torque.
    """
    if propulsion_selection is None:
        return {
            "propulsion_load_N": 0.0,
            "weight_moment_Nm": 0.0,
            "thrust_moment_Nm": 0.0,
            "motor_torque_Nm": 0.0,
        }

    combo = propulsion_selection["combo"]
    n_prop = propulsion_selection["n_propellers"]

    # Motor and prop mass per unit
    motor_mass_kg = float(combo["motor_mass_kg"])
    prop_mass_kg = float(combo["prop_mass_kg"])
    total_propulsion_mass_per_unit_kg = motor_mass_kg + prop_mass_kg

    prop_y_ratio = prop_y_position_fraction  # fraction from centerline to tip (0 to 1)

    # Total propulsion weight (all motors/props on one wing half)
    # Assuming symmetric placement: n_prop/4 per wing half
    n_prop_per_wing_half = n_prop / 4.0
    propulsion_weight_N = total_propulsion_mass_per_unit_kg * n_prop_per_wing_half * ATMOSPHERIC.g

    # Absolute spanwise position
    prop_y_abs_m = prop_y_ratio * ((wing_span_m - inputs.fuselage_OD)/2)

    # Chord at prop position (linear taper)
    c_tip = c_root * taper_ratio
    c_at_prop = c_root - (c_root - c_tip) * prop_y_ratio

    # Weight moment
    weight_moment_Nm = propulsion_weight_N * prop_y_abs_m

    # Thrust moment (vertical force moment in VTOL assumption)
    max_thrust_per_motor_N = float(combo["max_thrust_N"])
    max_thrust_total_N = max_thrust_per_motor_N * n_prop_per_wing_half
    thrust_moment_Nm = -max_thrust_total_N * prop_y_abs_m

    # Motor torque estimation
    max_power_per_motor_W = float(combo["max_power_W"])
    motor_kv = float(combo["motor_kv"])
    voltage_max_V = combo.get("voltage_max_V", 25.2)
    motor_efficiency = inputs.motor_mechanical_efficiency

    RPM_max = motor_kv * voltage_max_V
    P_mech_per_motor_W = max_power_per_motor_W * motor_efficiency
    omega_rad_per_s = 2.0 * math.pi * RPM_max / 60.0
    torque_per_motor_Nm = P_mech_per_motor_W / max(omega_rad_per_s, 1e-6)

    motor_torque_Nm = torque_per_motor_Nm * n_prop_per_wing_half

    return {
        "propulsion_load_N": propulsion_weight_N,
        "weight_moment_Nm": weight_moment_Nm,
        "thrust_moment_Nm": thrust_moment_Nm,
        "motor_torque_Nm": motor_torque_Nm,
        "prop_y_abs_m": prop_y_abs_m,
        "c_at_prop_m": c_at_prop,
    }


# ============================================================
# Tube sizing + checks
# ============================================================
def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _size_carbon_tube_for_bending(
    *,
    M_max_Nm: float,
    sigma_allow_Pa: float,
    id_over_od: float,
    E_eff_Pa: float,
    eps_comp_allow: float,
) -> Tuple[float, float, float, float]:
    """
    Closed-form sizing of a circular tube for bending stress, then grows OD until
    additional moment-driven screening checks pass (so warnings don't occur).

    Keeps the same return signature: (OD_m, ID_m, area_m2, I_m4)

    Notes:
    - This enforces a set of conservative *screening* constraints (thin-wall ratio,
      compressive strain proxy, curvature proxy, Brazier ovalization proxy) in addition
      to bending stress.
    - Because the function only knows M and sigma_allow (no span/support details),
      these are heuristics. Still very useful to avoid "too thin / too flexible" spars.
    """
    k = _clamp(float(id_over_od), 0.0, 0.95)  # keep sane

    # Tube screening thresholds (from MATERIAL)
    D_over_t_warn = MATERIAL.tube_D_over_t_limit
    R_mult = MATERIAL.tube_min_bend_radius_mult
    C_brazier = MATERIAL.tube_brazier_C

    # Numerical safety
    M = abs(float(M_max_Nm))
    sig_allow = max(1e-12, float(sigma_allow_Pa))

    # ---- Base closed-form sizing from bending stress ----
    eta = max(1e-12, 1.0 - k**4)  # (1 - (ID/OD)^4)
    OD = (32.0 * M / (math.pi * sig_allow * eta)) ** (1.0 / 3.0)

    # ---- Grow OD until ALL checks pass (no warnings) ----
    # We keep ID/OD constant.
    max_iter = 60
    grow = 1.05  # 5% steps
    for _ in range(max_iter):
        ID = k * OD
        t = 0.5 * (OD - ID)
        if t <= 0.0:
            # If k is too close to 1.0, thickness collapses; force slightly thicker by reducing k effectively.
            k_eff = min(k, 0.90)
            ID = k_eff * OD
            t = 0.5 * (OD - ID)

        area = (math.pi / 4.0) * (OD**2 - ID**2)
        I = (math.pi / 64.0) * (OD**4 - ID**4)

        r = 0.5 * OD

        # 1) Bending stress must pass
        sigma_bend = M * r / max(I, 1e-18)
        ok_bend = sigma_bend <= sig_allow

        # 2) Thin-wall (D/t) must pass
        D_over_t = OD / max(t, 1e-12)
        ok_thinwall = D_over_t <= D_over_t_warn

        # 3) Compressive strain proxy must pass
        eps_comp = sigma_bend / max(E_eff_Pa, 1e-12)
        ok_strain = eps_comp <= eps_comp_allow

        # 4) Curvature proxy must pass: R_bend >= R_mult * OD
        kappa = M / max(E_eff_Pa * I, 1e-18)  # 1/m
        R_bend = 1.0 / max(kappa, 1e-18)
        ok_curv = R_bend >= (R_mult * OD)

        # 5) Brazier ovalization screening must pass
        M_brazier = C_brazier * E_eff_Pa * (t**2) * r
        ok_brazier = M <= M_brazier

        if ok_bend and ok_thinwall and ok_strain and ok_curv and ok_brazier:
            return OD, ID, area, I

        # If any fail, increase OD and retry.
        OD *= grow

    # If we couldn't satisfy everything, return the best (last) and emit ONE warning.
    ID = k * OD
    area = (math.pi / 4.0) * (OD**2 - ID**2)
    I = (math.pi / 64.0) * (OD**4 - ID**4)
    print(
        "WARNING [TUBE]: Could not satisfy all screening checks within iteration limit. "
        "Returning enlarged OD that satisfies bending stress, but review thin-wall/ovalization/curvature assumptions."
    )
    return OD, ID, area, I


def _tube_bending_stress(*, M_Nm: float, OD_m: float, ID_m: float) -> float:
    """Max bending stress [Pa] for circular tube: sigma = M*c/I."""
    I = (math.pi / 64.0) * (OD_m**4 - ID_m**4)
    c = OD_m / 2.0
    return abs(M_Nm) * c / max(I, 1e-18)

def _estimate_cantilever_tip_deflection_from_root_moment(
    *,
    L_m: float,
    M_root_Nm: float,
    E_Pa: float,
    I_root_m4: float,
) -> Dict[str, float]:
    """
    Cantilever half-wing deflection using equivalent uniformly distributed load.
    Uses:
        delta_tip = M_root * L^2 / (4 E I)
        theta_root = M_root * L / (2 E I)
    """
    L = max(float(L_m), 1e-9)
    M = abs(float(M_root_Nm))
    E = max(float(E_Pa), 1e6)
    I = max(float(I_root_m4), 1e-18)

    delta_tip = M * L**2 / (4.0 * E * I)
    theta_root = M * L / (2.0 * E * I)

    return {
        "delta_tip_m": delta_tip,
        "delta_tip_mm": delta_tip * 1e3,
        "theta_root_rad": theta_root,
        "theta_root_deg": math.degrees(theta_root),
    }

def _check_rib_to_tube_transfer(
    *,
    V_shear_N: float,
    N_ribs: int,
    OD_m: float,
    allowable_bearing_Pa: float,
    contact_length_m: float,
    warning_prefix: str = "STRUCT",
) -> Dict[str, float]:
    """
    Conservative rib->tube bearing/crush proxy.

    F_per_rib ~ V / N_ribs
    bearing area per rib ~ OD * contact_length
    bearing stress = F / A

    Returns a dict of computed values; prints WARNING if not OK.
    """
    N = max(1, int(N_ribs))
    F_per_rib = abs(V_shear_N) / N
    Lc = max(float(contact_length_m), 1e-6)
    A_bearing = max(float(OD_m) * Lc, 1e-12)

    sigma_bearing = F_per_rib / A_bearing
    ok = sigma_bearing <= float(allowable_bearing_Pa)

    if not ok:
        print(
            f"WARNING [{warning_prefix}]: rib->tube bearing {sigma_bearing/1e6:.1f} MPa "
            f"> allow {allowable_bearing_Pa/1e6:.1f} MPa. "
            f"Increase contact length, add clamps/wrap, increase OD, or add local reinforcement."
        )

    return {
        "F_per_rib_N": F_per_rib,
        "bearing_area_m2": A_bearing,
        "bearing_stress_Pa": sigma_bearing,
        "bearing_allow_Pa": float(allowable_bearing_Pa),
        "bearing_ok": 1.0 if ok else 0.0,
    }

# ============================================================
# Skin stiffener estimation
# ============================================================
def _estimate_skin_stiffeners(
    *,
    torsion_Nm: float,
    A_cell_m2: float,
    c_box_m: float,
    t_skin_m: float,
    rib_spacing_m: float,
    h_m: float,
    E_skin_Pa: float,
    nu_skin: float,
    density_skin_kg_m3: float,
    panel_buckling_margin: float = 0.50,
    k_shear: float = 5.5,
    stiffener_t_min_factor: float = 1.0,
    stiffener_t_max_factor: float = 2.0,
    stiffener_h_min_factor: float = 2.0,
    stiffener_h_max_factor: float = 4.0,
) -> Dict[str, float]:
    """
    Size skin stiffeners ONLY for torsional skin shear-buckling prevention.

    Assumptions:
    - Two spars carry the global bending.
    - Skin carries torsional shear flow in a thin-walled closed cell.
    - Stiffeners only subdivide the skin panel width to prevent shear buckling.
    - Stiffener geometry is capped to small printable values:
        thickness in [1, 2] * skin thickness
        height    in [2, 4] * skin thickness
      and NEVER grown beyond that; count increases instead.

    Returns:
      N_stiffeners
      spacing_m
      thickness_m
      height_m
      area_one_m2
      I_one_m4
      tau_skin_MPa
      tau_cr_panel_MPa
      mass_kg
      panel_ok
      aspect_ratio
    """
    # Guards
    T = abs(float(torsion_Nm))
    A_cell = max(float(A_cell_m2), 1e-9)
    c_box = max(float(c_box_m), 1e-6)
    t_skin = max(float(t_skin_m), 1e-6)
    a_panel = max(float(rib_spacing_m), 1e-6)
    h = max(float(h_m), 1e-6)
    E = max(float(E_skin_Pa), 1e6)
    nu = max(0.0, min(0.49, float(nu_skin)))
    k_s = max(0.1, float(k_shear))
    margin = max(1e-6, float(panel_buckling_margin))

    # Thin-walled closed-cell torsion:
    # shear flow q = T / (2 A_cell)
    # skin shear stress tau = q / t
    q_shear_N_per_m = T / (2.0 * A_cell)
    tau_skin_Pa = q_shear_N_per_m / t_skin

    # Required panel buckling stress with margin
    tau_target_Pa = tau_skin_Pa / margin

    # Plate shear-buckling:
    # tau_cr = k_s * pi^2 * E / (12 * (1 - nu^2)) * (t / b)^2
    coeff = k_s * (math.pi ** 2) * E / (12.0 * (1.0 - nu ** 2))
    b_max_m = t_skin * math.sqrt(coeff / max(tau_target_Pa, 1e-12))

    # Keep panel width reasonable; no point making it larger than the box width
    b_max_m = min(b_max_m, c_box)

    # Number of panels across box width => N_stiff = N_panels - 1
    n_panels = max(1, int(math.ceil(c_box / max(b_max_m, 1e-9))))
    n_stiff = max(0, n_panels - 1)
    spacing_m = c_box / n_panels

    # Back-check actual panel buckling stress
    tau_cr_panel_Pa = coeff * (t_skin / max(spacing_m, 1e-12)) ** 2
    panel_ok = tau_skin_Pa <= margin * tau_cr_panel_Pa

    # Small printable stiffener geometry: capped, not load-driven
    t_st = min(
        max(stiffener_t_min_factor * t_skin, t_skin),
        stiffener_t_max_factor * t_skin,
    )
    h_st = min(
        max(stiffener_h_min_factor * t_skin, 2.0 * t_skin),
        min(stiffener_h_max_factor * t_skin, 0.30 * h),
    )

    # Simple rectangular blade properties (for CAD / mass reporting only)
    A_st = t_st * h_st
    I_st = t_st * (h_st ** 3) / 12.0

    # Stiffeners run spanwise over exposed half-span
    stiffener_run_m = a_panel  # local bay length between ribs, for one bay reference
    mass_per_bay_kg = n_stiff * density_skin_kg_m3 * A_st * stiffener_run_m

    aspect_ratio = a_panel / max(spacing_m, 1e-12)

    return {
        "N_stiffeners": float(n_stiff),
        "spacing_m": spacing_m,
        "thickness_m": t_st,
        "height_m": h_st,
        "area_one_m2": A_st,
        "I_one_m4": I_st,
        "tau_skin_MPa": tau_skin_Pa / 1e6,
        "tau_cr_panel_MPa": tau_cr_panel_Pa / 1e6,
        "mass_per_bay_kg": mass_per_bay_kg,
        "panel_ok": 1.0 if panel_ok else 0.0,
        "aspect_ratio": aspect_ratio,
        "q_shear_N_per_m": q_shear_N_per_m,
    }

def _size_circular_longeron_for_area(
    *,
    A_req_m2: float,
    id_over_od: float,
    OD_min_m: float,
) -> Tuple[float, float, float]:
    """
    Size a circular tube longeron from required area only.
    Returns (OD_m, ID_m, A_m2)
    """
    k = _clamp(float(id_over_od), 0.0, 0.95)
    A_req = max(float(A_req_m2), 1e-12)

    eta = max(1e-12, 1.0 - k**2)
    OD = math.sqrt(4.0 * A_req / (math.pi * eta))
    OD = max(OD, float(OD_min_m))
    ID = k * OD
    A = (math.pi / 4.0) * (OD**2 - ID**2)
    return OD, ID, A


def _estimate_fuselage_skin_panels(
    *,
    M_bend_Nm: float,
    T_torsion_Nm: float,
    fuselage_radius_m: float,
    shell_t_m: float,
    frame_spacing_m: float,
    stringer_spacing_m: float,
    E_skin_Pa: float,
    nu_skin: float,
    kc_comp: float = 4.0,
    ks_shear: float = 5.5,
    panel_margin: float = 0.50,
) -> Dict[str, float]:
    """
    Panel-level shell check using a flat-plate approximation between frames and stringers.

    Axial compression from global fuselage bending:
        sigma_x ~= M * R / I_shell

    Torsional shear from closed circular shell:
        q = T / (2 A_enclosed), tau = q / t

    Panel size:
        a = frame spacing
        b = stringer spacing

    Buckling checks:
        sigma_cr = kc * pi^2 E / (12(1-nu^2)) * (t/b)^2
        tau_cr   = ks * pi^2 E / (12(1-nu^2)) * (t/b)^2

    This is intentionally conservative and preliminary.
    """
    R = max(float(fuselage_radius_m), 1e-6)
    t = max(float(shell_t_m), 1e-6)
    a = max(float(frame_spacing_m), 1e-6)
    b = max(float(stringer_spacing_m), 1e-6)
    E = max(float(E_skin_Pa), 1e6)
    nu = max(0.0, min(0.49, float(nu_skin)))
    M = abs(float(M_bend_Nm))
    T = abs(float(T_torsion_Nm))
    margin = max(1e-6, float(panel_margin))

    # Thin circular shell second moment (thin-wall approximation)
    I_shell = math.pi * (R ** 3) * t

    # Axial bending stress at outer fiber
    sigma_x = M * R / max(I_shell, 1e-18)

    # Closed circular shell torsion
    A_enclosed = math.pi * (R ** 2)
    q_torsion = T / max(2.0 * A_enclosed, 1e-18)
    tau_xy = q_torsion / t

    coeff = (math.pi ** 2) * E / (12.0 * (1.0 - nu ** 2))
    sigma_cr = float(kc_comp) * coeff * (t / b) ** 2
    tau_cr = float(ks_shear) * coeff * (t / b) ** 2

    comp_ok = sigma_x <= margin * sigma_cr
    shear_ok = tau_xy <= margin * tau_cr

    # Very simple interaction index
    interaction = sigma_x / max(margin * sigma_cr, 1e-18) + (tau_xy / max(margin * tau_cr, 1e-18)) ** 2
    interaction_ok = interaction <= 1.0

    return {
        "sigma_x_Pa": sigma_x,
        "tau_xy_Pa": tau_xy,
        "sigma_cr_Pa": sigma_cr,
        "tau_cr_Pa": tau_cr,
        "interaction_index": interaction,
        "comp_ok": 1.0 if comp_ok else 0.0,
        "shear_ok": 1.0 if shear_ok else 0.0,
        "interaction_ok": 1.0 if interaction_ok else 0.0,
        "I_shell_m4": I_shell,
        "q_torsion_N_per_m": q_torsion,
        "panel_aspect_ratio": a / b,
    }


def _estimate_fuselage_stringers_and_frames(
    *,
    M_bend_Nm: float,
    T_torsion_Nm: float,
    fuselage_length_m: float,
    fuselage_radius_m: float,
    shell_t_m: float,
    shell_density_kg_m3: float,
    shell_sigma_allow_Pa: float,
    E_skin_Pa: float,
    nu_skin: float,
    frame_thickness_m: float,
    bulkhead_inner_diameter_m: float,
    default_frame_spacing_m: float,
    target_stringer_spacing_m: float,
    stringer_thickness_m: float,
    stringer_height_m: float,
    stringer_density_kg_m3: float,
    longeron_density_kg_m3: float,
    longeron_sigma_allow_Pa: float,
    longeron_id_over_od: float,
    longeron_od_min_m: float,
    longeron_bending_load_fraction,
    kc_comp,
    ks_shear,
    panel_margin,
) -> Dict[str, float]:
    """
    Preliminary fuselage layout estimator.

    Outputs:
      - number of frames
      - frame spacing
      - number of stringers
      - stringer spacing and geometry
      - number/size of primary longerons
      - masses
      - shell panel back-checks
    """
    L = max(float(fuselage_length_m), 1e-6)
    R = max(float(fuselage_radius_m), 1e-6)
    t = max(float(shell_t_m), 1e-6)

    # --- Frames (ring ribs) ---
    # Start from user default target spacing
    frame_spacing = max(float(default_frame_spacing_m), 2.0 * R * 0.25)
    n_bays = max(1, int(math.ceil(L / frame_spacing)))
    n_frames = n_bays + 1
    frame_spacing = L / n_bays

    # --- Stringers / shell stiffeners around circumference ---
    circumference = 2.0 * math.pi * R
    target_s = max(float(target_stringer_spacing_m), 1e-6)
    n_stringers = max(4, int(math.ceil(circumference / target_s)))
    stringer_spacing = circumference / n_stringers

    # Panel back-check with initial spacing
    panel_check = _estimate_fuselage_skin_panels(
        M_bend_Nm=M_bend_Nm,
        T_torsion_Nm=T_torsion_Nm,
        fuselage_radius_m=R,
        shell_t_m=t,
        frame_spacing_m=frame_spacing,
        stringer_spacing_m=stringer_spacing,
        E_skin_Pa=E_skin_Pa,
        nu_skin=nu_skin,
        kc_comp=kc_comp,
        ks_shear=ks_shear,
        panel_margin=panel_margin,
    )

    # If panel fails, increase stringer count first, then frame count
    for _ in range(60):
        if panel_check["interaction_ok"] > 0.5:
            break

        # Increase circumferential support first
        if panel_check["comp_ok"] < 0.5 or panel_check["shear_ok"] < 0.5:
            n_stringers += 2
            stringer_spacing = circumference / n_stringers

        panel_check = _estimate_fuselage_skin_panels(
            M_bend_Nm=M_bend_Nm,
            T_torsion_Nm=T_torsion_Nm,
            fuselage_radius_m=R,
            shell_t_m=t,
            frame_spacing_m=frame_spacing,
            stringer_spacing_m=stringer_spacing,
            E_skin_Pa=E_skin_Pa,
            nu_skin=nu_skin,
            kc_comp=kc_comp,
            ks_shear=ks_shear,
            panel_margin=panel_margin,
        )

        if panel_check["interaction_ok"] > 0.5:
            break

        # Then reduce frame spacing
        n_frames += 1
        n_bays = max(1, n_frames - 1)
        frame_spacing = L / n_bays

        panel_check = _estimate_fuselage_skin_panels(
            M_bend_Nm=M_bend_Nm,
            T_torsion_Nm=T_torsion_Nm,
            fuselage_radius_m=R,
            shell_t_m=t,
            frame_spacing_m=frame_spacing,
            stringer_spacing_m=stringer_spacing,
            E_skin_Pa=E_skin_Pa,
            nu_skin=nu_skin,
            kc_comp=kc_comp,
            ks_shear=ks_shear,
            panel_margin=panel_margin,
        )

    # --- Stringer geometry and mass ---
    t_st = max(float(stringer_thickness_m), t)
    h_st = max(float(stringer_height_m), 2.0 * t)
    A_stringer = t_st * h_st
    W_stringers = n_stringers * stringer_density_kg_m3 * A_stringer * L

    # --- Primary longerons ---
    # Use 4 as a clean default for a circular fuselage with wing carry-through;
    # top and bottom pair mainly resist bending.
    n_longerons = 4
    n_top = 2  # effective top compression members; same for bottom tension members

    M_for_longerons = float(longeron_bending_load_fraction) * abs(float(M_bend_Nm))
    y_long = 0.85 * R  # keep slightly inboard of OD

    A_req_each = M_for_longerons / max(2.0 * n_top * y_long * longeron_sigma_allow_Pa, 1e-18)

    OD_long, ID_long, A_long = _size_circular_longeron_for_area(
        A_req_m2=A_req_each,
        id_over_od=longeron_id_over_od,
        OD_min_m=longeron_od_min_m,
    )

    W_longerons = n_longerons * longeron_density_kg_m3 * A_long * L

    # --- Frame mass ---
    # Ring frame approximated as a flat annulus
    Do = 2.0 * R
    Di = min(float(bulkhead_inner_diameter_m), Do - 2.0 * t)
    Di = max(Di, 0.2 * Do)

    A_frame = (math.pi / 4.0) * (Do**2 - Di**2)
    W_frames = n_frames * shell_density_kg_m3 * A_frame * max(float(frame_thickness_m), t)

    return {
        "N_frames": float(n_frames),
        "frame_spacing_m": frame_spacing,
        "N_stringers": float(n_stringers),
        "stringer_spacing_m": stringer_spacing,
        "stringer_thickness_m": t_st,
        "stringer_height_m": h_st,
        "stringer_area_one_mm2": A_stringer * 1e6,
        "W_stringers_kg": W_stringers,

        "N_longerons": float(n_longerons),
        "longeron_OD_m": OD_long,
        "longeron_ID_m": ID_long,
        "longeron_A_one_mm2": A_long * 1e6,
        "W_longerons_kg": W_longerons,

        "frame_annulus_area_mm2": A_frame * 1e6,
        "W_frames_kg": W_frames,

        "sigma_x_MPa": panel_check["sigma_x_Pa"] / 1e6,
        "tau_xy_MPa": panel_check["tau_xy_Pa"] / 1e6,
        "sigma_cr_MPa": panel_check["sigma_cr_Pa"] / 1e6,
        "tau_cr_MPa": panel_check["tau_cr_Pa"] / 1e6,
        "q_torsion_N_per_m": panel_check["q_torsion_N_per_m"],
        "panel_aspect_ratio": panel_check["panel_aspect_ratio"],
        "panel_interaction_index": panel_check["interaction_index"],
        "panel_comp_ok": panel_check["comp_ok"],
        "panel_shear_ok": panel_check["shear_ok"],
        "panel_interaction_ok": panel_check["interaction_ok"],
    }



# ============================================================
# Twin-boom VTOL concentrated loads and primary structure
# ============================================================
def calculate_lift_propulsion_wing_loads(
    propulsion_selection: Optional[Dict[str, Any]],
    wing_span_m: float,
    c_root: float,
    taper_ratio: float,
    inputs: DesignInputs,
) -> Dict[str, float]:
    """Return one-half-wing root moments from the dedicated lift propulsors.

    The four lift units are treated as concentrated masses and vertical forces at
    the coordinates in ``lift_prop_*_positions_m``.  The output keeps the legacy
    ``calculate_surface_structure`` load dictionary so the rest of the structural
    sizing logic remains unchanged.
    """
    zero = {
        "propulsion_load_N": 0.0,
        "weight_moment_Nm": 0.0,
        "thrust_moment_Nm": 0.0,
        "motor_torque_Nm": 0.0,
        "n_units_half": 0.0,
        "mount_mass_half_kg": 0.0,
        "thrust_per_motor_structural_N": 0.0,
        "shaft_torque_half_Nm": 0.0,
        "thrust_offset_torsion_half_Nm": 0.0,
    }
    if propulsion_selection is None or not bool(getattr(inputs, "use_lift_motor_loads_in_wing_structure", True)):
        return zero

    lift = propulsion_selection.get("lift", {})
    combo = lift.get("combo")
    if not combo:
        return zero

    xs = [float(x) for x in inputs.lift_prop_x_positions_m]
    ys = [float(y) for y in inputs.lift_prop_y_positions_m]
    # Right half is used because the layout is expected to be symmetric.
    half_indices = [i for i, y in enumerate(ys) if y > 1e-9]
    if not half_indices:
        half_indices = list(range(max(1, len(ys) // 2)))

    root_y = 0.5 * float(getattr(inputs, "central_pod_width_m", inputs.fuselage_OD))
    c_mgc = float(c_root) * (1.0 + float(taper_ratio)) / 2.0
    wing_le_x = float(inputs.cg_x_wing) - 0.5 * c_mgc
    x_spar = wing_le_x + float(getattr(inputs, "wing_primary_spar_chord_fraction", 0.30)) * c_mgc

    mount_mass_each = (
        float(getattr(inputs, "lift_motor_mount_mass_each_kg", 0.0))
        * float(getattr(inputs, "lift_motor_mount_reinforcement_factor", 1.0))
    )
    unit_mass = float(combo.get("motor_mass_kg", 0.0)) + float(combo.get("prop_mass_kg", 0.0)) + mount_mass_each

    required_thrust = float(lift.get("required_thrust_per_motor_N", 0.0))
    available_thrust = float(combo.get("max_thrust_N", required_thrust))
    thrust_fraction = float(getattr(inputs, "lift_motor_max_thrust_fraction_for_structure", 1.0))
    thrust_structural = max(required_thrust, thrust_fraction * available_thrust)

    weight_N = 0.0
    weight_moment = 0.0
    thrust_moment = 0.0
    thrust_offset_torsion = 0.0
    for i in half_indices:
        y_arm = max(0.0, abs(ys[i]) - root_y)
        weight_N += unit_mass * ATMOSPHERIC.g
        weight_moment += unit_mass * ATMOSPHERIC.g * y_arm
        thrust_moment += thrust_structural * y_arm
        thrust_offset_torsion += thrust_structural * abs(xs[i] - x_spar)

    max_power = float(combo.get("max_power_W", lift.get("required_power_per_motor_W", 0.0)))
    kv = float(combo.get("motor_kv", 0.0))
    voltage = float(combo.get("voltage_max_V", 0.0))
    rpm = kv * voltage
    omega = 2.0 * math.pi * rpm / 60.0
    torque_each = (max_power * float(inputs.motor_mechanical_efficiency) / omega) if omega > 1e-9 else 0.0
    shaft_torque = len(half_indices) * torque_each

    return {
        "propulsion_load_N": weight_N,
        "weight_moment_Nm": weight_moment,
        "thrust_moment_Nm": thrust_moment,
        "motor_torque_Nm": shaft_torque + thrust_offset_torsion,
        "n_units_half": float(len(half_indices)),
        "mount_mass_half_kg": float(len(half_indices)) * mount_mass_each,
        "thrust_per_motor_structural_N": thrust_structural,
        "shaft_torque_half_Nm": shaft_torque,
        "thrust_offset_torsion_half_Nm": thrust_offset_torsion,
        "root_y_m": root_y,
        "primary_spar_x_m": x_spar,
    }


def calculate_central_pod_structure(
    wing_bending_moment: float,
    inputs: DesignInputs,
) -> Tuple[float, float, Dict[str, float]]:
    """Preliminary printed central-pod shell and frame estimate.

    The pod is represented as a rounded rectangular thin-walled box.  The wing
    carry-through is assumed to carry most of the main-wing bending; only the
    configurable fraction enters the pod shell check.
    """
    L = float(inputs.central_pod_length_m)
    w = float(inputs.central_pod_width_m)
    h = float(inputs.central_pod_height_m)
    t_min = max(float(getattr(inputs, "central_pod_min_shell_thickness_m", 0.0012)), MATERIAL.min_skin_thickness_m)
    sigma_allow = float(inputs.filament_max_bending_stress_MPa) * 1e6 / float(inputs.printed_parts_sf)
    tau_allow = float(inputs.filament_shear_strength_MPa) * 1e6 / float(inputs.printed_parts_sf)
    M_design = abs(float(wing_bending_moment)) * float(getattr(inputs, "central_pod_bending_load_fraction", 0.25))
    T_design = abs(float(wing_bending_moment)) * float(getattr(inputs, "central_pod_design_torsion_fraction", 0.20))

    t = t_min
    for _ in range(80):
        wi = max(1e-6, w - 2.0 * t)
        hi = max(1e-6, h - 2.0 * t)
        I_box = (w * h**3 - wi * hi**3) / 12.0
        sigma = M_design * (0.5 * h) / max(I_box, 1e-18)
        A_enclosed = max(1e-9, wi * hi)
        tau = T_design / max(2.0 * A_enclosed * t, 1e-18)
        if sigma <= sigma_allow and tau <= tau_allow:
            break
        t *= 1.05

    pod_surface_area = 0.82 * 2.0 * (L * w + L * h + w * h)
    W_shell = float(inputs.filament_density) * pod_surface_area * t

    spacing = max(0.04, float(getattr(inputs, "central_pod_frame_spacing_m", 0.12)))
    n_frames = max(2, int(math.ceil(L / spacing)) + 1)
    ring_width = min(0.45 * min(w, h), float(getattr(inputs, "central_pod_frame_ring_width_m", 0.012)))
    A_frame = w * h - max(0.0, w - 2.0 * ring_width) * max(0.0, h - 2.0 * ring_width)
    frame_axial_t = float(getattr(inputs, "central_pod_frame_axial_thickness_m", 0.004))
    W_frames = n_frames * A_frame * frame_axial_t * float(inputs.filament_density)
    W_carrythrough = float(getattr(inputs, "central_pod_wing_carrythrough_mass_kg", 0.0))
    W_total_calc = (W_shell + W_frames + W_carrythrough) * float(getattr(inputs, "central_pod_nonstructural_factor", 1.0))

    W_total = W_total_calc
    if bool(getattr(inputs, "use_hardcoded_values", False)):
        hard = getattr(inputs, "hardcode_central_pod_mass_kg", None)
        if hard is not None:
            W_total = float(hard)

    return t, W_total, {
        "structure_type": "rounded_rectangular_central_pod",
        "W_shell_kg": W_shell,
        "W_frames_total_kg": W_frames,
        "W_stringers_total_kg": 0.0,
        "W_longerons_total_kg": W_carrythrough,
        "W_carrythrough_kg": W_carrythrough,
        "W_total_calculated_kg": W_total_calc,
        "T_design_Nm": T_design,
        "M_design_Nm": M_design,
        "N_frames": float(n_frames),
        "frame_spacing_m": L / max(n_frames - 1, 1),
        "frame_annulus_area_mm2": A_frame * 1e6,
        "sigma_x_MPa": sigma / 1e6,
        "tau_xy_MPa": tau / 1e6,
        "sigma_cr_MPa": sigma_allow / 1e6,
        "tau_cr_MPa": tau_allow / 1e6,
        "panel_comp_ok": 1.0 if sigma <= sigma_allow else 0.0,
        "panel_shear_ok": 1.0 if tau <= tau_allow else 0.0,
        "panel_interaction_ok": 1.0 if (sigma / max(sigma_allow, 1e-9) + tau / max(tau_allow, 1e-9)) <= 1.0 else 0.0,
        "pod_surface_area_m2": pod_surface_area,
    }


def calculate_twin_boom_structure(
    L_tail_N: float,
    horizontal_tail_mass_kg: float,
    vertical_tail_mass_kg: float,
    propulsion_selection: Optional[Dict[str, Any]],
    inputs: DesignInputs,
) -> Tuple[float, Dict[str, float]]:
    """Size/check the two tail booms and return total boom-system mass.

    Loads include tail aerodynamic force, tail structural weight, the aft lift
    propulsors attached downstream of the boom root, and axial pusher thrust.
    """
    n_booms = max(1, int(inputs.n_booms))
    L = max(1e-6, float(inputs.boom_length_m))
    OD_input = float(inputs.boom_outer_diameter_m)
    ID_input = min(float(inputs.boom_inner_diameter_m), 0.98 * OD_input)
    id_ratio = ID_input / max(OD_input, 1e-9)

    tail_force_per_boom = float(inputs.n_ult) * abs(float(L_tail_N)) / n_booms
    tail_weight_per_boom = (
        float(inputs.n_ult)
        * (float(horizontal_tail_mass_kg) + float(vertical_tail_mass_kg))
        * ATMOSPHERIC.g
        / n_booms
    )
    M_cruise = (tail_force_per_boom + tail_weight_per_boom) * L

    M_rear_lift = 0.0
    pusher_axial_per_boom = 0.0
    if propulsion_selection is not None:
        lift = propulsion_selection.get("lift", {})
        lift_combo = lift.get("combo", {})
        thrust_motor = max(
            float(lift.get("required_thrust_per_motor_N", 0.0)),
            float(getattr(inputs, "lift_motor_max_thrust_fraction_for_structure", 1.0))
            * float(lift_combo.get("max_thrust_N", 0.0)),
        )
        for x in inputs.lift_prop_x_positions_m:
            x = float(x)
            if x >= float(inputs.boom_x_start_m):
                M_rear_lift += thrust_motor * max(0.0, x - float(inputs.boom_x_start_m)) / n_booms

        cruise = propulsion_selection.get("cruise", {})
        pusher_axial_per_boom = float(cruise.get("required_thrust_per_motor_N", 0.0)) / n_booms

    M_vtol = float(inputs.n_ult) * M_rear_lift + tail_weight_per_boom * L
    M_design = max(M_cruise, M_vtol)

    sigma_allow = (
        float(inputs.carbon_fiber_max_bending_stress_MPa) * 1e6
        / max(1.0, float(getattr(inputs, "boom_bending_safety_factor", inputs.carbon_fiber_safety_factor)))
    )
    OD_req, ID_req, _, _ = _size_carbon_tube_for_bending(
        M_max_Nm=M_design,
        sigma_allow_Pa=sigma_allow,
        id_over_od=id_ratio,
        E_eff_Pa=float(inputs.carbon_fiber_E_Pa),
        eps_comp_allow=float(inputs.carbon_fiber_comp_strain_allow),
    )
    OD = max(OD_input, OD_req) if bool(getattr(inputs, "boom_auto_size", True)) else OD_input
    ID = id_ratio * OD
    A = math.pi / 4.0 * (OD**2 - ID**2)
    I = math.pi / 64.0 * (OD**4 - ID**4)
    sigma_bend = M_design * (0.5 * OD) / max(I, 1e-18)
    sigma_axial = pusher_axial_per_boom / max(A, 1e-18)
    sigma_combined = sigma_bend + sigma_axial
    tip_deflection = M_design * L**2 / max(4.0 * float(inputs.carbon_fiber_E_Pa) * I, 1e-18)

    A_input = math.pi / 4.0 * (OD_input**2 - ID_input**2)
    area_scale = A / max(A_input, 1e-18)
    W_tubes = n_booms * L * float(inputs.boom_specific_mass_kg_per_m) * area_scale
    W_mounts = n_booms * (
        float(getattr(inputs, "boom_forward_mount_mass_each_kg", 0.0))
        + float(getattr(inputs, "boom_tail_mount_mass_each_kg", 0.0))
    )
    W_cross = float(getattr(inputs, "tail_crossmember_mass_kg", 0.0))
    W_total_calc = W_tubes + W_mounts + W_cross
    W_total = W_total_calc
    if bool(getattr(inputs, "use_hardcoded_values", False)):
        hard = getattr(inputs, "hardcode_boom_mass_total_kg", None)
        if hard is not None:
            W_total = float(hard)

    return W_total, {
        "W_total_kg": W_total,
        "W_total_calculated_kg": W_total_calc,
        "W_tubes_kg": W_tubes,
        "W_mounts_kg": W_mounts,
        "W_tail_crossmember_kg": W_cross,
        "OD_m": OD,
        "ID_m": ID,
        "area_one_m2": A,
        "I_one_m4": I,
        "M_cruise_Nm_per_boom": M_cruise,
        "M_vtol_Nm_per_boom": M_vtol,
        "M_design_Nm_per_boom": M_design,
        "pusher_axial_N_per_boom": pusher_axial_per_boom,
        "sigma_bending_MPa": sigma_bend / 1e6,
        "sigma_axial_MPa": sigma_axial / 1e6,
        "sigma_combined_MPa": sigma_combined / 1e6,
        "sigma_allow_MPa": sigma_allow / 1e6,
        "strength_ok": 1.0 if sigma_combined <= sigma_allow else 0.0,
        "tip_deflection_m": tip_deflection,
        "tip_deflection_mm": tip_deflection * 1e3,
        "rear_lift_bending_Nm_per_boom": M_rear_lift,
        "tail_force_N_per_boom": tail_force_per_boom,
        "tail_weight_N_per_boom": tail_weight_per_boom,
    }


# ============================================================
# Main surface structure (wing or tail)
# ============================================================
def calculate_surface_structure(
    load_N: float,
    S: float,
    c_root: float,
    taper_ratio: float,
    b: float,
    c_MGC: float,
    y_MAC: float,
    max_thickness_fraction: float,
    flap_chord_ratio: float,
    CM_max: float,
    inputs: DesignInputs,
    propulsion_loads: Optional[Dict[str, Any]] = None,
    max_iterations: int = 20,
    convergence_tol: float = 0.01,  # kept for API compatibility (not used)
) -> Tuple[Structure, Dict[str, float]]:
    """
    Calculate structural properties for a lifting surface (wing or tail).

    - Skin sizing: torsion closed-cell approximation
    - Spar sizing: tube beam bending (Mc/I) with id_over_od
    - Adds conservative rib->tube bearing check
    - Adds torsion-only skin stiffener estimation from skin shear buckling
    """
    # Ultimate shear on half-wing/half-tail
    shear = inputs.n_ult * abs(load_N) / 2.0

    L_panel = max(0.0, (b - inputs.fuselage_OD) / 2.0)   # exposed half-span
    y_res = 0.5 * L_panel
    M_aero = shear * y_res

    M_weight = 0.0
    M_thrust = 0.0
    if propulsion_loads is not None:
        M_weight = inputs.n_ult * float(propulsion_loads["weight_moment_Nm"])
        M_thrust = inputs.n_ult * abs(float(propulsion_loads["thrust_moment_Nm"]))

    M_cruise = abs(M_aero + M_weight)
    M_vtol = abs(M_thrust + M_weight)
    if M_cruise >= M_vtol:
        bending_moment = M_cruise
        bending_moment_type = "cruise"
    else:
        bending_moment = M_vtol
        bending_moment_type = "thrust"

    # Base torsion from aerodynamic moment
    torsion = 0.25 * ATMOSPHERIC.rho * inputs.dive_speed**2 * S * c_MGC * CM_max

    # Add motor torque to torsion
    if propulsion_loads is not None:
        torsion += abs(float(propulsion_loads["motor_torque_Nm"]))

    # Airfoil thickness
    h = c_root * max_thickness_fraction

    # Cell geometry (approx)
    c_cell_root = c_root * flap_chord_ratio
    A_cell_root = 2.0 * c_cell_root * h / 3.0

    # Allowables
    tau_allow_Pa = inputs.filament_shear_strength_MPa * 1e6 / inputs.printed_parts_sf
    sigma_allow_Pa = (inputs.carbon_fiber_max_bending_stress_MPa * 1e6) / inputs.carbon_fiber_safety_factor

    # Inputs for tube sizing / transfer checks
    spar_id_over_od = float(inputs.wing_spar_id_over_od)
    filament_comp_strength_MPa = getattr(inputs, "filament_compressive_strength_MPa", inputs.filament_max_bending_stress_MPa)
    bearing_allow_Pa = (filament_comp_strength_MPa * 1e6) / inputs.printed_parts_sf

    # Start with minimum values
    t_skin_root = MATERIAL.min_skin_thickness_m
    rib_contact_length_m = t_skin_root

    # Tube state
    spar_OD_root_m = 0.0
    spar_ID_root_m = 0.0
    spar_area_root = MATERIAL.min_spar_area_m2

    for iteration in range(max_iterations):
        # -------- TORSION: skin thickness ----------
        t_skin_root_required = max(
            torsion * inputs.printed_parts_sf / (2.0 * A_cell_root * tau_allow_Pa),
            MATERIAL.min_skin_thickness_m,
        )
        t_skin_root = max(t_skin_root, t_skin_root_required)
        t_skin_tip = max(t_skin_root * MATERIAL.tip_thickness_ratio, MATERIAL.min_skin_thickness_m)

        torsion_capacity_Nm = 2.0 * A_cell_root * tau_allow_Pa * t_skin_root
        torsion_sufficient = torsion_capacity_Nm >= torsion

        # -------- BENDING: tube sizing (root controls) ----------
        OD_req_m, ID_req_m, A_req_m2, _I_req_m4 = _size_carbon_tube_for_bending(
            M_max_Nm=bending_moment,
            sigma_allow_Pa=sigma_allow_Pa,
            id_over_od=spar_id_over_od,
            E_eff_Pa=inputs.carbon_fiber_E_Pa,
            eps_comp_allow=inputs.carbon_fiber_comp_strain_allow,
        )

        A_req_m2 = max(A_req_m2, MATERIAL.min_spar_area_m2)

        spar_OD_root_m = max(spar_OD_root_m, OD_req_m)
        spar_ID_root_m = max(spar_ID_root_m, spar_id_over_od * spar_OD_root_m)
        spar_area_root = max(
            spar_area_root,
            (math.pi / 4.0) * (spar_OD_root_m**2 - spar_ID_root_m**2),
            A_req_m2,
        )

        sigma_bend = _tube_bending_stress(
            M_Nm=bending_moment,
            OD_m=spar_OD_root_m,
            ID_m=spar_ID_root_m,
        )
        bending_sufficient = sigma_bend <= sigma_allow_Pa

        if torsion_sufficient and bending_sufficient:
            break

        if not torsion_sufficient:
            t_skin_root = max(t_skin_root * 1.10, (torsion / (2.0 * A_cell_root * tau_allow_Pa)) * 1.05)

        if not bending_sufficient:
            spar_OD_root_m = max(spar_OD_root_m * 1.05, OD_req_m * 1.02)
            spar_ID_root_m = spar_id_over_od * spar_OD_root_m
            spar_area_root = (math.pi / 4.0) * (spar_OD_root_m**2 - spar_ID_root_m**2)

    # Final warnings
    if iteration == max_iterations - 1:
        torsion_capacity_final = 2.0 * A_cell_root * tau_allow_Pa * t_skin_root
        if torsion_capacity_final < torsion:
            print(
                f"WARNING: Torsion capacity ({torsion_capacity_final:.2f} N·m) "
                f"still below required ({torsion:.2f} N·m) after {max_iterations} iterations."
            )

        sigma_bend_final = _tube_bending_stress(
            M_Nm=bending_moment, OD_m=max(spar_OD_root_m, 1e-9), ID_m=max(spar_ID_root_m, 0.0)
        )
        if sigma_bend_final > sigma_allow_Pa:
            print(
                f"WARNING: Spar tube bending stress ({sigma_bend_final/1e6:.2f} MPa) "
                f"> allowable ({sigma_allow_Pa/1e6:.2f} MPa) after {max_iterations} iterations."
            )

    # ============================================================
    # Mass breakdown
    # ============================================================
    s_cell_root = (
        math.sqrt((h / 2.0) ** 2 + 4.0 * c_cell_root**2)
        + (h**2 / (16.0 * c_cell_root)) * math.asinh(4.0 * c_cell_root / max(h, 1e-9))
    )
    s_cell_tip = taper_ratio * s_cell_root

    # Skin mass
    W_skin = inputs.filament_density * b * (t_skin_root + t_skin_tip) * s_cell_root * (1.0 + taper_ratio) / 8.0

    # Web thickness (from shear)
    t_web_root = max(
        3.0 * inputs.n_ult * load_N * inputs.printed_parts_sf / (4.0 * h * inputs.filament_shear_strength_MPa * 1e6),
        MATERIAL.min_skin_thickness_m,
    )
    t_web_tip = max(t_web_root * MATERIAL.tip_thickness_ratio, MATERIAL.min_skin_thickness_m)

    # Web mass
    W_web = inputs.filament_density * b * (t_web_root + t_web_tip) * h * (1.0 + taper_ratio) / 8.0

    # Spar tip area scaling
    spar_area_tip = max(spar_area_root * MATERIAL.tip_spar_area_ratio, MATERIAL.min_spar_area_m2)

    # Spar length per surface
    L_spar_per_surface = b / 2.0

    # Spar mass
    W_spars = inputs.carbon_fiber_density * (
        (spar_area_root + spar_area_tip) * L_spar_per_surface
        + (spar_area_root + spar_area_tip) * L_spar_per_surface * inputs.wing_second_spar_scale
    )

    # Ribs
    rib_span = max(0.0, b - inputs.fuselage_OD - 2.0 * inputs.fuselage_wing_interface_length)
    N_ribs = math.ceil(rib_span / c_MGC) + 1
    if N_ribs % 2 != 0:
        N_ribs += 1

    rib_spacing_m = rib_span / max(2 * (N_ribs / 2 - 1), 1)

    W_ribs = (
        N_ribs
        * inputs.filament_density
        * A_cell_root
        * (1.0 + taper_ratio**2)
        * (t_skin_root + t_skin_tip)
        / 4.0
    )

    # Conservative rib->tube transfer check
    bearing_check = _check_rib_to_tube_transfer(
        V_shear_N=shear,
        N_ribs=N_ribs,
        OD_m=max(spar_OD_root_m, 1e-6),
        allowable_bearing_Pa=bearing_allow_Pa,
        contact_length_m=rib_contact_length_m,
        warning_prefix="SURFACE",
    )

    # ============================================================
    # NEW: torsion-only skin stiffeners
    # ============================================================
    stiffener_E_Pa = float(getattr(inputs, "filament_E_Pa", 2.5e9))
    stiffener_nu = float(getattr(inputs, "filament_nu", 0.35))
    stiffener_margin = float(getattr(inputs, "stiffener_panel_buckling_margin", 0.50))
    stiffener_ks = float(getattr(inputs, "stiffener_panel_k_shear", 5.5))

    stiffener_results = _estimate_skin_stiffeners(
        torsion_Nm=torsion,
        A_cell_m2=A_cell_root,
        c_box_m=c_cell_root,
        t_skin_m=t_skin_root,
        rib_spacing_m=rib_spacing_m,
        h_m=h,
        E_skin_Pa=stiffener_E_Pa,
        nu_skin=stiffener_nu,
        density_skin_kg_m3=inputs.filament_density,
        panel_buckling_margin=stiffener_margin,
        k_shear=stiffener_ks,
        stiffener_t_min_factor=float(getattr(inputs, "stiffener_t_min_factor", 1.0)),
        stiffener_t_max_factor=float(getattr(inputs, "stiffener_t_max_factor", 2.0)),
        stiffener_h_min_factor=float(getattr(inputs, "stiffener_h_min_factor", 2.0)),
        stiffener_h_max_factor=float(getattr(inputs, "stiffener_h_max_factor", 4.0)),
    )

    # Actual stiffener run is exposed half-span, not one rib bay
    stiffener_run_m = max(0.0, (b - inputs.fuselage_OD) / 2.0)
    n_stiff = int(stiffener_results["N_stiffeners"])
    A_st = stiffener_results["area_one_m2"]
    W_stiffeners = inputs.filament_density * n_stiff * A_st * stiffener_run_m

    if stiffener_results["panel_ok"] < 0.5:
        print(
            f"WARNING [STIFFENERS]: torsional skin shear-buckling check not satisfied. "
            f"tau_skin={stiffener_results['tau_skin_MPa']:.2f} MPa, "
            f"tau_cr={stiffener_results['tau_cr_panel_MPa']:.2f} MPa. "
            f"Increase skin thickness or add more stiffeners."
        )

    # Calculate tip spar dimensions
    if spar_area_root > 1e-12:
        area_ratio = spar_area_tip / spar_area_root
        OD_scale = math.sqrt(area_ratio) if area_ratio > 0 else MATERIAL.tip_spar_area_ratio
    else:
        OD_scale = MATERIAL.tip_spar_area_ratio

    spar_OD_tip_m = spar_OD_root_m * OD_scale
    spar_ID_tip_m = spar_ID_root_m * OD_scale

    I_root_m4 = (math.pi / 64.0) * (spar_OD_root_m**4 - spar_ID_root_m**4)
    I_tip_m4 = (math.pi / 64.0) * (spar_OD_tip_m**4 - spar_ID_tip_m**4) if spar_OD_tip_m > 1e-9 else 0.0

    sigma_bend_root = _tube_bending_stress(M_Nm=bending_moment, OD_m=spar_OD_root_m, ID_m=spar_ID_root_m)
    
    deflection_results = _estimate_cantilever_tip_deflection_from_root_moment(
        L_m=L_spar_per_surface-inputs.fuselage_OD/2 - inputs.fuselage_wing_interface_length,
        M_root_Nm=bending_moment,
        E_Pa=inputs.carbon_fiber_E_Pa,
        I_root_m4=I_root_m4,
    )

    # Total mass (for one surface)
    W_total = W_skin + W_web + W_spars + W_ribs + W_stiffeners

    spar_results = {
        "L_spar_m": L_spar_per_surface,
        "M_max_Nm": bending_moment,
        "sigma_allow_MPa": sigma_allow_Pa / 1e6,
        "sigma_root_MPa": sigma_bend_root / 1e6,
        "OD_root_m": spar_OD_root_m,
        "ID_root_m": spar_ID_root_m,
        "OD_tip_m": spar_OD_tip_m,
        "ID_tip_m": spar_ID_tip_m,
        "A_root_mm2": spar_area_root * 1e6,
        "A_tip_mm2": spar_area_tip * 1e6,
        "I_root_mm4": I_root_m4 * 1e12,
        "I_tip_mm4": I_tip_m4 * 1e12,
        "W_spar_kg": W_spars,
        "N_ribs": float(N_ribs),
        "rib_contact_L_m": rib_contact_length_m,
        "bearing_allow_MPa": bearing_allow_Pa / 1e6,
        "bearing_stress_MPa": bearing_check.get("bearing_stress_Pa", 0.0) / 1e6,
        "bearing_ok": bearing_check.get("bearing_ok", 0.0),
        "rib_spacing_m": rib_spacing_m,

        # NEW torsion-only stiffener outputs
        "N_stiffeners": stiffener_results["N_stiffeners"],
        "stiffener_spacing_m": stiffener_results["spacing_m"],
        "stiffener_thickness_m": stiffener_results["thickness_m"],
        "stiffener_height_m": stiffener_results["height_m"],
        "stiffener_area_one_mm2": stiffener_results["area_one_m2"] * 1e6,
        "stiffener_I_one_mm4": stiffener_results["I_one_m4"] * 1e12,
        "stiffener_tau_skin_MPa": stiffener_results["tau_skin_MPa"],
        "stiffener_tau_cr_panel_MPa": stiffener_results["tau_cr_panel_MPa"],
        "stiffener_q_shear_N_per_m": stiffener_results["q_shear_N_per_m"],
        "stiffener_aspect_ratio": stiffener_results["aspect_ratio"],
        "W_stiffeners_kg": W_stiffeners,
        "stiffener_panel_ok": stiffener_results["panel_ok"],

        "tip_deflection_m": deflection_results["delta_tip_m"],
        "tip_deflection_mm": deflection_results["delta_tip_mm"],
        "root_slope_rad": deflection_results["theta_root_rad"],
        "root_slope_deg": deflection_results["theta_root_deg"],
    }

    return Structure(
        bending_moment_Nm=bending_moment,
        bending_moment_type=bending_moment_type,
        torsion_Nm=torsion,
        h_m=h,
        t_skin_root_m=t_skin_root,
        t_skin_tip_m=t_skin_tip,
        W_skin_kg=W_skin,
        t_web_root_m=t_web_root,
        t_web_tip_m=t_web_tip,
        W_web_kg=W_web,
        spar_area_root_m2=spar_area_root,
        spar_area_tip_m2=spar_area_tip,
        W_spars_kg=W_spars,
        N_ribs=N_ribs,
        rib_spacing_m=rib_spacing_m,
        W_ribs_kg=W_ribs,
        W_total_kg=W_total,
    ), spar_results


# ============================================================
# Wing structure
# ============================================================
def calculate_wing_structure(
    L_wing: float,
    S: float,
    c_root: float,
    taper_ratio: float,
    b: float,
    c_MGC: float,
    y_MAC: float,
    max_thickness_fraction: float,
    CM_max: float,
    inputs: DesignInputs,
    propulsion_selection: Optional[Dict[str, Any]] = None,
) -> Tuple[Structure, float, Dict[str, float]]:
    """
    Calculate wing structural properties.
    b is the full wing span from geometry.wing_geometry
    (fixed when hardcode_wing_span is active).

    Returns: (Structure, total_wings_mass_kg, spar_results)
    Total mass is for both cruciform wings (2x single wing)
    """
    propulsion_loads = calculate_lift_propulsion_wing_loads(
        propulsion_selection=propulsion_selection,
        wing_span_m=b,
        c_root=c_root,
        taper_ratio=taper_ratio,
        inputs=inputs,
    )
    structure, spar_results = calculate_surface_structure(
        load_N=L_wing,
        S=S,
        c_root=c_root,
        taper_ratio=taper_ratio,
        b=b,
        c_MGC=c_MGC,
        y_MAC=y_MAC,
        max_thickness_fraction=max_thickness_fraction,
        flap_chord_ratio=inputs.wing_plain_flap_chord_ratio,
        CM_max=CM_max,
        inputs=inputs,
        propulsion_loads=propulsion_loads,
    )

    # Printed/local motor-mount reinforcement is part of the wing airframe mass.
    W_mounts_half = float(propulsion_loads.get("mount_mass_half_kg", 0.0))
    structure.W_total_kg += W_mounts_half
    spar_results.update({
        "lift_mount_mass_half_kg": W_mounts_half,
        "lift_mount_mass_total_kg": 2.0 * W_mounts_half,
        "lift_thrust_per_motor_structural_N": propulsion_loads.get("thrust_per_motor_structural_N", 0.0),
        "lift_weight_root_moment_Nm_half": propulsion_loads.get("weight_moment_Nm", 0.0),
        "lift_thrust_root_moment_Nm_half": propulsion_loads.get("thrust_moment_Nm", 0.0),
        "lift_shaft_torque_Nm_half": propulsion_loads.get("shaft_torque_half_Nm", 0.0),
        "lift_offset_torsion_Nm_half": propulsion_loads.get("thrust_offset_torsion_half_Nm", 0.0),
    })

    _apply_hardcoded_surface_skin_root_thickness(
        structure,
        spar_results,
        inputs,
        b=b,
        c_root=c_root,
        taper_ratio=taper_ratio,
        max_thickness_fraction=max_thickness_fraction,
        flap_chord_ratio=inputs.wing_plain_flap_chord_ratio,
        hardcode_attr="hardcode_wing_skin_root_thickness_m",
    )

    od_root = _get_hardcoded_float(inputs, "hardcode_wing_spar_OD", spar_results["OD_root_m"])
    if od_root != spar_results["OD_root_m"]:
        spar_id_over_od = float(inputs.wing_spar_id_over_od)
        od_tip_ratio = spar_results["OD_tip_m"] / max(spar_results["OD_root_m"], 1e-9)

        spar_results["OD_root_m"] = od_root
        spar_results["ID_root_m"] = od_root * spar_id_over_od
        spar_results["OD_tip_m"] = od_root * od_tip_ratio
        spar_results["ID_tip_m"] = spar_results["OD_tip_m"] * spar_id_over_od

        ar = (math.pi / 4.0) * (spar_results["OD_root_m"]**2 - spar_results["ID_root_m"]**2)
        at = (math.pi / 4.0) * (spar_results["OD_tip_m"]**2 - spar_results["ID_tip_m"]**2)

        spar_results["A_root_mm2"] = ar * 1e6
        spar_results["A_tip_mm2"] = at * 1e6
        spar_results["I_root_mm4"] = (math.pi / 64.0) * (spar_results["OD_root_m"]**4 - spar_results["ID_root_m"]**4) * 1e12
        spar_results["I_tip_mm4"] = (math.pi / 64.0) * (spar_results["OD_tip_m"]**4 - spar_results["ID_tip_m"]**4) * 1e12

        L_spar = spar_results["L_spar_m"]
        W_spars = inputs.carbon_fiber_density * (
            (ar + at) * L_spar * 0.5
            + (ar + at) * L_spar * 0.5 * inputs.wing_second_spar_scale
        )
        spar_results["W_spar_kg"] = W_spars

        structure.spar_area_root_m2 = ar
        structure.spar_area_tip_m2 = at
        structure.W_spars_kg = W_spars

        W_stiff = float(spar_results.get("W_stiffeners_kg", 0.0))
        structure.W_total_kg = structure.W_skin_kg + structure.W_web_kg + structure.W_spars_kg + structure.W_ribs_kg + W_stiff

    if getattr(inputs, "use_hardcoded_values", False) and getattr(inputs, "hardcode_wing_secondary_spar_OD", None) is not None:
        spar_results["OD_secondary_m"] = float(inputs.hardcode_wing_secondary_spar_OD)

    # Hardcoded spar tube mass override (used for weight breakdown consistency).
    if getattr(inputs, "use_hardcoded_values", False):
        hard_primary = getattr(inputs, "hardcode_wing_spar_weight_kg", None)
        hard_secondary = getattr(inputs, "hardcode_wing_secondary_spar_weight_kg", None)
        if hard_primary is not None or hard_secondary is not None:
            W_hardcoded_spars = (
                (float(hard_primary) if hard_primary is not None else 0.0)
                + (float(hard_secondary) if hard_secondary is not None else 0.0)
            )
            old_W_spars = float(getattr(structure, "W_spars_kg", 0.0))
            structure.W_spars_kg = W_hardcoded_spars
            spar_results["W_spar_kg"] = W_hardcoded_spars
            structure.W_total_kg = structure.W_total_kg - old_W_spars + W_hardcoded_spars

    W_wings = 2.0 * structure.W_total_kg
    return structure, W_wings, spar_results


# ============================================================
# Tail structure + elevator hinge rod
# ============================================================
def calculate_tail_structure(
    L_tail: float,
    S: float,
    c_root: float,
    taper_ratio: float,
    b: float,
    c_MGC: float,
    y_MAC: float,
    max_thickness_fraction: float,
    CM_max: float,
    inputs: DesignInputs,
    propulsion_selection: Optional[Dict[str, Any]] = None,
    skin_root_thickness_hardcode_attr: str = "hardcode_tail_wing_skin_root_thickness_m",
    spar_primary_weight_hardcode_attr: str = "hardcode_tail_wing_spar_weight_kg",
    spar_secondary_weight_hardcode_attr: str = "hardcode_elevator_rod_weight_kg",
) -> Tuple[Structure, float, Dict[str, float], Dict[str, float]]:
    """
    Calculate tail structural properties.

    b is the full tail span from geometry.tail_geometry/vertical_tail_geometry
    (fixed when hardcode_tail_span or hardcode_vertical_tail_span is active).

    Returns: (Structure, total_tails_mass_kg, elevator_rod_results, spar_results)
    Total mass is for both cruciform tail surfaces (2x single tail)
    """
    propulsion_loads = None
    if propulsion_selection is not None and bool(getattr(inputs, "tail_mounted_propulsion", True)):
        propulsion_loads = calculate_propulsion_loads(
            propulsion_selection=propulsion_selection,
            prop_y_position_fraction=inputs.prop_y_position_fraction,
            prop_x_position_fraction=inputs.prop_x_position_fraction,
            wing_span_m=b,
            c_root=c_root,
            taper_ratio=taper_ratio,
            inputs=inputs,
        )

    structure, spar_results = calculate_surface_structure(
        load_N=L_tail,
        S=S,
        c_root=c_root,
        taper_ratio=taper_ratio,
        b=b,
        c_MGC=c_MGC,
        y_MAC=y_MAC,
        max_thickness_fraction=max_thickness_fraction,
        flap_chord_ratio=inputs.tail_elevator_chord_ratio,
        CM_max=CM_max,
        inputs=inputs,
        propulsion_loads=propulsion_loads,
    )

    _apply_hardcoded_surface_skin_root_thickness(
        structure,
        spar_results,
        inputs,
        b=b,
        c_root=c_root,
        taper_ratio=taper_ratio,
        max_thickness_fraction=max_thickness_fraction,
        flap_chord_ratio=inputs.tail_elevator_chord_ratio,
        hardcode_attr=skin_root_thickness_hardcode_attr,
    )

    od_root = _get_hardcoded_float(inputs, "hardcode_tail_wing_spar_OD", spar_results["OD_root_m"])
    if od_root != spar_results["OD_root_m"]:
        spar_id_over_od = float(inputs.wing_spar_id_over_od)
        od_tip_ratio = spar_results["OD_tip_m"] / max(spar_results["OD_root_m"], 1e-9)

        spar_results["OD_root_m"] = od_root
        spar_results["ID_root_m"] = od_root * spar_id_over_od
        spar_results["OD_tip_m"] = od_root * od_tip_ratio
        spar_results["ID_tip_m"] = spar_results["OD_tip_m"] * spar_id_over_od

        ar = (math.pi / 4.0) * (spar_results["OD_root_m"]**2 - spar_results["ID_root_m"]**2)
        at = (math.pi / 4.0) * (spar_results["OD_tip_m"]**2 - spar_results["ID_tip_m"]**2)

        spar_results["A_root_mm2"] = ar * 1e6
        spar_results["A_tip_mm2"] = at * 1e6
        spar_results["I_root_mm4"] = (math.pi / 64.0) * (spar_results["OD_root_m"]**4 - spar_results["ID_root_m"]**4) * 1e12
        spar_results["I_tip_mm4"] = (math.pi / 64.0) * (spar_results["OD_tip_m"]**4 - spar_results["ID_tip_m"]**4) * 1e12

        L_spar = spar_results["L_spar_m"]
        W_spars = inputs.carbon_fiber_density * (
            (ar + at) * L_spar * 0.5
            + (ar + at) * L_spar * 0.5 * inputs.wing_second_spar_scale
        )
        spar_results["W_spar_kg"] = W_spars

        structure.spar_area_root_m2 = ar
        structure.spar_area_tip_m2 = at
        structure.W_spars_kg = W_spars

        W_stiff = float(spar_results.get("W_stiffeners_kg", 0.0))
        structure.W_total_kg = structure.W_skin_kg + structure.W_web_kg + structure.W_spars_kg + structure.W_ribs_kg + W_stiff

    # ------------------------------------------------------------
    # Elevator hinge rod sizing (tube beam bending)
    # ------------------------------------------------------------
    span_frac = float(inputs.tail_elevator_span_end) - float(inputs.tail_elevator_span_start)
    L_rod_m = span_frac * float(b)
    f_area = float(inputs.tail_elevator_chord_ratio) * span_frac

    L_ult_N = float(inputs.n_ult) * abs(float(L_tail))
    F_elev_N = f_area * L_ult_N

    M_max_rod_Nm = F_elev_N * L_rod_m / 8.0

    sigma_allow_Pa = (inputs.carbon_fiber_max_bending_stress_MPa * 1e6) / inputs.carbon_fiber_safety_factor

    OD_m, ID_m, A_rod_m2, I_rod_m4 = _size_carbon_tube_for_bending(
        M_max_Nm=M_max_rod_Nm,
        sigma_allow_Pa=sigma_allow_Pa,
        id_over_od=float(inputs.tail_elevator_rod_id_over_od),
        E_eff_Pa=inputs.carbon_fiber_E_Pa,
        eps_comp_allow=inputs.carbon_fiber_comp_strain_allow,
    )

    sigma_rod_Pa = _tube_bending_stress(M_Nm=M_max_rod_Nm, OD_m=OD_m, ID_m=ID_m)
    if sigma_rod_Pa > sigma_allow_Pa:
        print(
            f"WARNING [ELEV ROD]: bending stress {sigma_rod_Pa/1e6:.2f} MPa > "
            f"allow {sigma_allow_Pa/1e6:.2f} MPa"
        )

    filament_comp_strength_MPa = float(getattr(inputs, "filament_compressive_strength_MPa", inputs.filament_max_bending_stress_MPa))
    rod_bearing_allow_Pa = (filament_comp_strength_MPa * 1e6) / inputs.printed_parts_sf
    rod_contact_L = inputs.elevator_rod_bearing_length_m

    N_support = 3
    _check_rib_to_tube_transfer(
        V_shear_N=F_elev_N,
        N_ribs=N_support,
        OD_m=OD_m,
        allowable_bearing_Pa=rod_bearing_allow_Pa,
        contact_length_m=rod_contact_L,
        warning_prefix="ELEV ROD",
    )

    W_rod_kg = inputs.carbon_fiber_density * A_rod_m2 * L_rod_m
    structure.W_total_kg += W_rod_kg

    od_rod = _get_hardcoded_float(inputs, "hardcode_elevator_rod_OD", OD_m)
    if od_rod != OD_m:
        old_W_rod = W_rod_kg
        id_over_od = float(inputs.tail_elevator_rod_id_over_od)
        ID_m = od_rod * id_over_od
        A_rod_m2 = (math.pi / 4.0) * (od_rod**2 - ID_m**2)
        I_rod_m4 = (math.pi / 64.0) * (od_rod**4 - ID_m**4)
        W_rod_kg = inputs.carbon_fiber_density * A_rod_m2 * L_rod_m
        structure.W_total_kg -= old_W_rod
        structure.W_total_kg += W_rod_kg
        OD_m = od_rod
        sigma_rod_Pa = _tube_bending_stress(M_Nm=M_max_rod_Nm, OD_m=OD_m, ID_m=ID_m)

    # Hardcoded spar/rod mass override (attribute names are configurable for horizontal/vertical tails).
    if getattr(inputs, "use_hardcoded_values", False):
        hard_primary = getattr(inputs, spar_primary_weight_hardcode_attr, None)
        hard_secondary = getattr(inputs, spar_secondary_weight_hardcode_attr, None)
        if hard_primary is not None or hard_secondary is not None:
            W_hardcoded_spars = (
                (float(hard_primary) if hard_primary is not None else 0.0)
                + (float(hard_secondary) if hard_secondary is not None else 0.0)
            )
            old_W_spars = float(getattr(structure, "W_spars_kg", 0.0))
            structure.W_spars_kg = W_hardcoded_spars
            spar_results["W_spar_kg"] = W_hardcoded_spars
            structure.W_total_kg = structure.W_total_kg - old_W_spars + W_hardcoded_spars

    W_tails = 2.0 * structure.W_total_kg

    elevator_rod_results = {
        "span_frac": span_frac,
        "L_rod_m": L_rod_m,
        "F_elev_N": F_elev_N,
        "M_max_rod_Nm": M_max_rod_Nm,
        "sigma_allow_MPa": sigma_allow_Pa / 1e6,
        "sigma_rod_MPa": sigma_rod_Pa / 1e6,
        "OD_m": OD_m,
        "ID_m": ID_m,
        "A_rod_mm2": A_rod_m2 * 1e6,
        "I_rod_mm4": I_rod_m4 * 1e12,
        "W_rod_kg": W_rod_kg,
        "N_support": float(N_support),
        "rod_contact_L_m": rod_contact_L,
        "rod_bearing_allow_MPa": rod_bearing_allow_Pa / 1e6,
    }

    return structure, W_tails, elevator_rod_results, spar_results


# ============================================================
# Fuselage structure (unchanged)
# ============================================================
def calculate_fuselage_structure(
    wing_bending_moment: float,
    inputs: DesignInputs,
) -> Tuple[float, float, Dict[str, float]]:
    """
    Preliminary semi-monocoque fuselage sizing.

    Returns:
      (shell_thickness_m, total_fuselage_mass_kg, fuselage_details)
    """
    M_bend = abs(float(wing_bending_moment))

    R = float(inputs.fuselage_OR)
    L = float(inputs.fuselage_length)

    # --------------------------------------------
    # 1) Shell thickness from bending + torsion
    # --------------------------------------------
    sigma_allow_shell = inputs.filament_max_bending_stress_MPa * 1e6 / inputs.printed_parts_sf
    tau_allow_shell = inputs.filament_shear_strength_MPa * 1e6 / inputs.printed_parts_sf

    # Bending-based shell thickness:
    # sigma = M * R / I, with I_thin ~= pi * R^3 * t  =>  t = M / (pi * R^2 * sigma)
    t_bend = M_bend / max(math.pi * (R ** 2) * sigma_allow_shell, 1e-18)

    # Torsion load entering fuselage:
    # if you have a direct fuselage torsion input later, use it.
    # For now use a conservative fraction of wing bending moment as a torsion-like design load.
    T_fus = float(getattr(inputs, "fuselage_design_torsion_Nm", 0.15 * M_bend))

    # Closed circular shell torsion:
    # q = T/(2A), tau = q/t -> t = q/tau_allow
    A_enclosed = math.pi * (R ** 2)
    q_torsion = T_fus / max(2.0 * A_enclosed, 1e-18)
    t_torsion = q_torsion / max(tau_allow_shell, 1e-18)

    fuselage_t_shell = max(t_bend, t_torsion, MATERIAL.min_skin_thickness_m)
    fuselage_t_shell = _get_hardcoded_float(inputs, "hardcode_fuselage_skin_thickness_m", fuselage_t_shell)

    # --------------------------------------------
    # 2) Shell mass
    # --------------------------------------------
    fuselage_shell_area = inputs.fuselage_cs_perimeter * L
    W_fuselage_shell = inputs.filament_density * fuselage_t_shell * fuselage_shell_area

    # --------------------------------------------
    # 3) Frames, stringers, longerons
    # --------------------------------------------
    fuselage_details = _estimate_fuselage_stringers_and_frames(
        M_bend_Nm=M_bend,
        T_torsion_Nm=T_fus,
        fuselage_length_m=L,
        fuselage_radius_m=R,
        shell_t_m=fuselage_t_shell,
        shell_density_kg_m3=inputs.filament_density,
        shell_sigma_allow_Pa=sigma_allow_shell,
        E_skin_Pa=inputs.filament_E_Pa,
        nu_skin=inputs.filament_nu,
        frame_thickness_m=inputs.fuselage_rib_thickness,
        bulkhead_inner_diameter_m=inputs.fuselage_bulkhead_inner_OD,
        default_frame_spacing_m=inputs.fuselage_frame_spacing_target_m,
        target_stringer_spacing_m=inputs.fuselage_stringer_spacing_target_m,
        stringer_thickness_m=inputs.fuselage_stinger_thickness,
        stringer_height_m=inputs.fuselage_stinger_height,
        stringer_density_kg_m3=inputs.filament_density,
        longeron_density_kg_m3=inputs.carbon_fiber_density,
        longeron_sigma_allow_Pa=(inputs.carbon_fiber_max_bending_stress_MPa * 1e6) / inputs.carbon_fiber_safety_factor,
        longeron_id_over_od=inputs.fuselage_longeron_id_over_od,
        longeron_od_min_m=inputs.fuselage_longeron_diameter,
        longeron_bending_load_fraction=inputs.fuselage_longeron_bending_load_fraction,
        kc_comp=inputs.fuselage_panel_kc,
        ks_shear=inputs.fuselage_panel_ks,
        panel_margin=inputs.fuselage_panel_buckling_margin,
    )

    # Hardcoded longeron mass override (4 longerons on this fuselage layout).
    if getattr(inputs, "use_hardcoded_values", False):
        hard_longeron = getattr(inputs, "hardcode_fuselage_longeron_weight_kg", None)
        if hard_longeron is not None:
            n_longerons = int(fuselage_details.get("N_longerons", 4))
            fuselage_details["W_longerons_kg"] = float(hard_longeron) * float(n_longerons)

    # Add two reinforced wing carry-through frames near the wing section
    W_extra_wing_frames = 2.0 * fuselage_details["frame_annulus_area_mm2"] * 1e-6 * inputs.filament_density * max(float(getattr(inputs, "fuselage_rib_thickness", 0.004)), fuselage_t_shell)

    W_fuselage_frames = fuselage_details["W_frames_kg"] + W_extra_wing_frames
    W_fuselage_stringers = fuselage_details["W_stringers_kg"]
    W_fuselage_longerons = fuselage_details["W_longerons_kg"]

    W_fuselage_total = inputs.fuselage_nonopt_factor * (
        W_fuselage_shell + W_fuselage_frames + W_fuselage_stringers + W_fuselage_longerons
    )

    fuselage_details.update({
        "t_shell_m": fuselage_t_shell,
        "W_shell_kg": W_fuselage_shell,
        "W_frames_total_kg": W_fuselage_frames,
        "W_stringers_total_kg": W_fuselage_stringers,
        "W_longerons_total_kg": W_fuselage_longerons,
        "T_design_Nm": T_fus,
    })

    return fuselage_t_shell, W_fuselage_total, fuselage_details
