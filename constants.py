"""
Physical and design constants used throughout the design script.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class AtmosphericConstants:
    """Atmospheric constants at design altitude (~150m)."""
    g: float = 9.81  # gravitational acceleration [m/s²]
    rho: float = 1.2  # air density [kg/m³]
    mu: float = 1.8e-5  # dynamic viscosity [kg/(m·s)] ~ ISA sea-level


@dataclass(frozen=True)
class DesignConstants:
    """Design iteration and convergence constants."""
    max_iterations: int = 100  # max number of iterations to find MTOW
    convergence_threshold: float = 1e-3  # 0.1% relative change for convergence
    # Add these to DESIGN
    trim_inner_max_iter: int = 8          # inner loop count per outer iteration
    trim_inner_tol_N: float = 0.05        # convergence tolerance on L_tail [N]
    trim_inner_relax: float = 0.5         # relaxation factor (0..1), helps stability


@dataclass(frozen=True)
class MaterialConstants:
    """Material property constants."""
    min_skin_thickness_m: float = 0.0008  # 0.508 mm minimum printable gage
    min_spar_area_m2: float = 6.4516e-6  # 1 in² in m² (minimum spar area)
    tip_thickness_ratio: float = 0.15  # tip thickness as fraction of root
    tip_spar_area_ratio: float = 0.05  # tip spar area as fraction of root
    # Tube screening (bending/ovalization)
    tube_D_over_t_limit: float = 80.0  # max D/t for thin-wall screening (avoid ovalization)
    tube_min_bend_radius_mult: float = 20.0  # min bend radius as multiple of OD (curvature screening)
    tube_brazier_C: float = 0.2  # Brazier ovalization screening constant [-]


# Global instances for easy access
ATMOSPHERIC = AtmosphericConstants()
DESIGN = DesignConstants()
MATERIAL = MaterialConstants()

