"""
Convergence tracking and visualization.
"""

from typing import List, Tuple
import matplotlib.pyplot as plt
from constants import DESIGN


def check_convergence(
    W0_mass_new: float,
    W0_mass_old: float,
) -> Tuple[bool, float]:
    """
    Check if design has converged.
    
    Args:
        W0_mass_new: New mass estimate [kg]
        W0_mass_old: Previous mass estimate [kg]
    
    Returns:
        Tuple of (converged, relative_change)
    """
    if W0_mass_old > 0:
        relative_change = abs(W0_mass_new - W0_mass_old) / W0_mass_old
        converged = relative_change < DESIGN.convergence_threshold
    else:
        relative_change = 1.0
        converged = False
    
    return converged, relative_change


def plot_mass_convergence(iterations: List[int], mass_history: List[float]) -> None:
    """
    Create visualization of aircraft mass convergence.
    
    Args:
        iterations: List of iteration numbers
        mass_history: List of aircraft mass values at each iteration
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle('Aircraft Mass Convergence', fontsize=16, fontweight='bold')
    
    # Plot 1: Mass vs Iteration
    ax1.plot(iterations, mass_history, 'o-', linewidth=2, markersize=8,
             color='#2E86AB', markerfacecolor='#A23B72', markeredgecolor='white',
             markeredgewidth=1.5, label='Aircraft Mass (MTOW)')
    ax1.axhline(y=mass_history[-1], color='r', linestyle='--', linewidth=1.5,
                alpha=0.7, label=f'Final Mass: {mass_history[-1]:.3f} kg')
    ax1.set_xlabel('Iteration', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Mass [kg]', fontsize=12, fontweight='bold')
    ax1.set_title('Mass Convergence History', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.legend(loc='best', fontsize=10)
    ax1.set_xlim(-0.5, max(iterations) + 0.5)
    
    # Add annotation for convergence
    if len(mass_history) > 1:
        final_change = abs(mass_history[-1] - mass_history[-2]) / mass_history[-2] * 100
        ax1.text(0.02, 0.98, f'Final change: {final_change:.4f}%',
                transform=ax1.transAxes, fontsize=10,
                verticalalignment='top', bbox=dict(boxstyle='round',
                facecolor='wheat', alpha=0.5))
    
    # Plot 2: Relative change between iterations
    if len(mass_history) > 1:
        relative_changes = []
        change_iterations = []
        for i in range(1, len(mass_history)):
            if mass_history[i-1] > 0:
                rel_change = abs(mass_history[i] - mass_history[i-1]) / mass_history[i-1] * 100
                relative_changes.append(rel_change)
                change_iterations.append(iterations[i])
        
        ax2.semilogy(change_iterations, relative_changes, 's-', linewidth=2,
                    markersize=8, color='#F18F01', markerfacecolor='#C73E1D',
                    markeredgecolor='white', markeredgewidth=1.5,
                    label='Relative Change')
        ax2.axhline(y=0.1, color='g', linestyle='--', linewidth=1.5,
                    alpha=0.7, label='0.1% Convergence Threshold')
        ax2.set_xlabel('Iteration', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Relative Change [%]', fontsize=12, fontweight='bold')
        ax2.set_title('Convergence Rate (Relative Change per Iteration)',
                     fontsize=13, fontweight='bold')
        ax2.grid(True, alpha=0.3, linestyle='--', which='both')
        ax2.legend(loc='best', fontsize=10)
        ax2.set_xlim(-0.5, max(change_iterations) + 0.5)
    
    plt.tight_layout()
    plt.show()


def plot_tail_trim_convergence(
    inner_iterations: List[int],
    L_tail_history: List[float],
    outer_iteration: int,
) -> None:
    """
    Create visualization of tail trim (L_tail) convergence for inner loop.
    
    Args:
        inner_iterations: List of inner iteration numbers (0, 1, 2, ...)
        L_tail_history: List of tail lift values at each inner iteration [N]
        outer_iteration: Outer loop iteration number (for title)
    """
    if len(L_tail_history) < 2:
        return  # Need at least 2 points to plot
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle(f'Tail Trim Convergence (Outer Iteration {outer_iteration})', 
                 fontsize=16, fontweight='bold')
    
    # Plot 1: L_tail vs Inner Iteration
    ax1.plot(inner_iterations, L_tail_history, 'o-', linewidth=2, markersize=8,
             color='#2E86AB', markerfacecolor='#A23B72', markeredgecolor='white',
             markeredgewidth=1.5, label='Tail Lift (L_tail)')
    ax1.axhline(y=L_tail_history[-1], color='r', linestyle='--', linewidth=1.5,
                alpha=0.7, label=f'Final L_tail: {L_tail_history[-1]:.4f} N')
    ax1.set_xlabel('Inner Iteration', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Tail Lift [N]', fontsize=12, fontweight='bold')
    ax1.set_title('Tail Lift Convergence History', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.legend(loc='best', fontsize=10)
    ax1.set_xlim(-0.5, max(inner_iterations) + 0.5)
    
    # Add annotation for convergence
    if len(L_tail_history) > 1:
        final_change = abs(L_tail_history[-1] - L_tail_history[-2])
        ax1.text(0.02, 0.98, f'Final change: {final_change:.6f} N',
                transform=ax1.transAxes, fontsize=10,
                verticalalignment='top', bbox=dict(boxstyle='round',
                facecolor='wheat', alpha=0.5))
    
    # Plot 2: Change in L_tail between iterations
    if len(L_tail_history) > 1:
        L_tail_changes = []
        change_iterations = []
        for i in range(1, len(L_tail_history)):
            change = abs(L_tail_history[i] - L_tail_history[i-1])
            L_tail_changes.append(change)
            change_iterations.append(inner_iterations[i])
        
        ax2.semilogy(change_iterations, L_tail_changes, 's-', linewidth=2,
                    markersize=8, color='#F18F01', markerfacecolor='#C73E1D',
                    markeredgecolor='white', markeredgewidth=1.5,
                    label='|ΔL_tail|')
        # Add tolerance line if available
        try:
            from constants import DESIGN
            tol = getattr(DESIGN, "trim_inner_tol_N", 0.05)
            ax2.axhline(y=tol, color='g', linestyle='--', linewidth=1.5,
                        alpha=0.7, label=f'Convergence Threshold: {tol:.4f} N')
        except:
            pass
        ax2.set_xlabel('Inner Iteration', fontsize=12, fontweight='bold')
        ax2.set_ylabel('|ΔL_tail| [N]', fontsize=12, fontweight='bold')
        ax2.set_title('Convergence Rate (Change per Inner Iteration)',
                     fontsize=13, fontweight='bold')
        ax2.grid(True, alpha=0.3, linestyle='--', which='both')
        ax2.legend(loc='best', fontsize=10)
        ax2.set_xlim(-0.5, max(change_iterations) + 0.5)
    
    plt.tight_layout()
    plt.show()
