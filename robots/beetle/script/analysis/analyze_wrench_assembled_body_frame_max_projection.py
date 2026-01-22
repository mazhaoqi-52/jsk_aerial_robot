#!/usr/bin/env python3
"""
Created for assembled Beetle modules wrench analysis.

Compute maximum force or torque in the BODY frame by maximizing the projection
onto the target direction. This allows other wrench components to be non-zero.

This is different from the original definition (exact wrench matching) and will
produce a full ellipsoid-like envelope even with tilt angle constraints.

Usage examples:
    python analyze_wrench_assembled_body_frame_max_projection.py
    python analyze_wrench_assembled_body_frame_max_projection.py -m force -n 2 -r 10
    python analyze_wrench_assembled_body_frame_max_projection.py config.yaml -m torque
"""

import os
import sys
import argparse
import numpy as np
import cvxpy as cp
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib import cm
import scienceplots
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.axes_grid1 import make_axes_locatable
import yaml

from analyze_allocation_assembled import get_alloc_mtx_assembled, get_assembled_physical_params

# ------------------------------------------------------------------------
# CONSTANTS
# ------------------------------------------------------------------------
THRUST_MAX = 21.37  # Maximum thrust per rotor [N] for Beetle
THRUST_MIN = 0.0
DEFAULT_TILT_LIMIT = 90.0  # degrees

DEFAULT_CONFIG_FILE = "config_wrench_analysis.yaml"


# ------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------

def load_config(config_path=None):
    """Load configuration from YAML file if it exists."""
    if config_path is None:
        config_path = DEFAULT_CONFIG_FILE
    
    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        print(f"Loaded config from: {config_path}")
        return config
    return {}


def get_default_config():
    """Return default configuration."""
    return {
        "mode": "force",
        "n_modules": 1,
        "module_spacing": 0.52,
        "resolution": 10,
        "tilt_limit": 90,
        "thrust_max": THRUST_MAX,
        "save_to_npz": False,
    }


# ------------------------------------------------------------------------
# Core analysis functions
# ------------------------------------------------------------------------

def find_max_wrench_in_direction(
    alloc_mtx,
    n_rotors,
    direction,
    mode="force",
    f_th_max=THRUST_MAX,
    tilt_limit_deg=DEFAULT_TILT_LIMIT,
):
    """
    Find the maximum wrench magnitude in a specified direction by solving
    an optimization problem directly.
    
    This method maximizes the projection of the achievable wrench onto the
    target direction, allowing other wrench components to be non-zero.
    
    Parameters:
        alloc_mtx: allocation matrix (6 × 2*n_rotors)
        n_rotors: total number of rotors
        direction: unit direction vector [dx, dy, dz] in body frame
        mode: "force" or "torque"
        f_th_max: maximum thrust magnitude per rotor
        tilt_limit_deg: tilt angle limit (±degrees from vertical), None for no limit
    
    Returns:
        max_magnitude: maximum achievable wrench magnitude in the direction
    """
    n_vars = 2 * n_rotors
    u = cp.Variable(n_vars)
    
    # Build the wrench output: wrench = alloc_mtx @ u
    wrench = alloc_mtx @ u  # 6×1 wrench vector
    
    # Select force or torque components
    if mode == "force":
        wrench_vec = wrench[0:3]  # [Fx, Fy, Fz]
    else:  # torque
        wrench_vec = wrench[3:6]  # [τx, τy, τz]
    
    # Objective: maximize projection onto direction
    # projection = direction · wrench_vec
    direction_np = np.array(direction).reshape(3)
    projection = direction_np @ wrench_vec
    
    # Constraints
    constraints = []
    for i in range(n_rotors):
        fx_idx = 2 * i
        fy_idx = 2 * i + 1
        
        # Thrust magnitude constraint: Fx² + Fy² ≤ f_max²
        constraints.append(u[fx_idx]**2 + u[fy_idx]**2 <= f_th_max**2)
        
        # Tilt angle constraints (if specified)
        if tilt_limit_deg is not None:
            # Fy ≥ 0 (rotor can only push, not pull)
            constraints.append(u[fy_idx] >= 0)
            
            # For tilt_limit < 90°: |Fx| ≤ Fy * tan(limit)
            if tilt_limit_deg < 90.0:
                tan_limit = np.tan(np.radians(tilt_limit_deg))
                constraints.append(u[fx_idx] <= u[fy_idx] * tan_limit)
                constraints.append(u[fx_idx] >= -u[fy_idx] * tan_limit)
    
    prob = cp.Problem(cp.Maximize(projection), constraints)
    try:
        prob.solve(verbose=False)
    except cp.SolverError as e:
        print(f"Solver error: {e}")
        return 0.0
    
    if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        return 0.0
    
    return prob.value if prob.value is not None else 0.0


def get_direction_from_spherical(pitch_deg, yaw_deg):
    """
    Convert spherical coordinates to unit direction vector.
    
    This uses the same convention as the original code:
    R_bn @ [0,0,1] gives the direction vector.
    
    Parameters:
        pitch_deg: polar angle (0° = +z, 90° = xy plane, 180° = -z)
        yaw_deg: azimuthal angle in xy plane
    
    Returns:
        Unit direction vector [x, y, z]
    """
    R_bn = R.from_euler("zyx", [yaw_deg, pitch_deg, 0.0], degrees=True).as_matrix().T
    return R_bn @ np.array([0.0, 0.0, 1.0])


def main():
    parser = argparse.ArgumentParser(
        description="Analyze max wrench (projection method) in BODY frame for assembled modules.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This script maximizes the projection of wrench onto the target direction,
allowing other wrench components to be non-zero. This produces a full
ellipsoid-like envelope even with tilt angle constraints.

Examples:
  python analyze_wrench_assembled_body_frame_max_projection.py
  python analyze_wrench_assembled_body_frame_max_projection.py -m force -n 2
  python analyze_wrench_assembled_body_frame_max_projection.py -m torque -t 90 -r 10
        """
    )
    parser.add_argument(
        "config",
        nargs="?",
        default=None,
        help=f"Path to YAML config file (default: {DEFAULT_CONFIG_FILE} if exists)",
    )
    parser.add_argument(
        "--mode", "-m",
        choices=["force", "torque"],
        help="Select 'force' or 'torque' mode.",
    )
    parser.add_argument(
        "--n_modules", "-n",
        type=int,
        help="Number of modules in the assembled configuration.",
    )
    parser.add_argument(
        "--module_spacing", "-s",
        type=float,
        help="Distance between adjacent module centers [m].",
    )
    parser.add_argument(
        "--resolution", "-r",
        type=float,
        help="Resolution of angles in degrees (e.g., 10 means 10° steps).",
    )
    parser.add_argument(
        "--tilt_limit", "-t",
        type=float,
        help="Tilt angle limit in degrees (e.g., 90 for ±90°). Use 180 for no constraint.",
    )
    parser.add_argument(
        "--thrust_max",
        type=float,
        help="Maximum thrust per rotor [N].",
    )
    parser.add_argument(
        "--save_to_npz",
        action="store_true",
        help="If set, save the results to a .npz file.",
    )

    args = parser.parse_args()

    # Load config file
    config = get_default_config()
    file_config = load_config(args.config)
    config.update(file_config)

    # Override with command line arguments
    if args.mode:
        config["mode"] = args.mode
    if args.n_modules:
        config["n_modules"] = args.n_modules
    if args.module_spacing:
        config["module_spacing"] = args.module_spacing
    if args.resolution:
        config["resolution"] = args.resolution
    if args.tilt_limit is not None:
        config["tilt_limit"] = args.tilt_limit
    if args.thrust_max:
        config["thrust_max"] = args.thrust_max
    if args.save_to_npz:
        config["save_to_npz"] = True

    # Extract config values
    mode = config["mode"]
    n_modules = config["n_modules"]
    module_spacing = config["module_spacing"]
    resolution = config["resolution"]
    tilt_limit_deg = config["tilt_limit"]
    thrust_max = config["thrust_max"]
    save_to_npz = config["save_to_npz"]

    # Handle tilt_limit = 180 as no constraint
    if tilt_limit_deg >= 180:
        tilt_limit_deg = None
        print("Tilt limit: None (no constraint)")
    else:
        print(f"Tilt limit: ±{tilt_limit_deg}°")

    print(f"Configuration: {n_modules} module(s), spacing={module_spacing}m")
    print(f"Thrust max: {thrust_max}N")
    print(f"Method: Max projection (allows other wrench components)")

    # Load allocation matrix for assembled configuration
    alloc_mat = get_alloc_mtx_assembled(n_modules, module_spacing)
    n_rotors = 4 * n_modules
    phys_params = get_assembled_physical_params(n_modules, module_spacing)

    print(f"Allocation matrix shape: {alloc_mat.shape}")
    print(f"Total mass: {phys_params['total_mass']:.3f} kg")
    print(f"Combined inertia: {phys_params['combined_inertia']}")

    # Create grids of yaw ∈ [-180°, +180°], pitch ∈ [0°, 180°]
    yaw_list = np.linspace(-180, 180, int(360 / resolution) + 1)
    pitch_list = np.linspace(0, 180, int(180 / resolution) + 1)

    Nyaw = len(yaw_list)
    Npitch = len(pitch_list)

    result_map = np.zeros((Npitch, Nyaw))

    for i, pitch_deg in enumerate(pitch_list):
        for j, yaw_deg in enumerate(yaw_list):
            # Get direction vector using same convention as original code
            direction = get_direction_from_spherical(pitch_deg, yaw_deg)
            
            # Find maximum wrench in this direction
            max_val = find_max_wrench_in_direction(
                alloc_mat,
                n_rotors,
                direction,
                mode=mode,
                f_th_max=thrust_max,
                tilt_limit_deg=tilt_limit_deg,
            )
            result_map[i, j] = max_val
        print(f"[Body {mode.capitalize()} MaxProj] Completed pitch = {pitch_deg:.1f}°")

    # Save to file
    if save_to_npz:
        tilt_str = f"tilt{int(tilt_limit_deg)}" if tilt_limit_deg else "notilt"
        filename = f"body_{mode}_maxproj_{n_modules}modules_{tilt_str}.npz"
        np.savez(
            filename,
            yaw_list=yaw_list,
            pitch_list=pitch_list,
            result_map=result_map,
            n_modules=n_modules,
            module_spacing=module_spacing,
            tilt_limit_deg=tilt_limit_deg,
            thrust_max=thrust_max,
        )
        print(f"Saved to '{filename}'.")

    # ---------- Plotting 3D surface ----------
    dirs = []
    mags = []
    for i, pitch_deg in enumerate(pitch_list):
        for j, yaw_deg in enumerate(yaw_list):
            direction = get_direction_from_spherical(pitch_deg, yaw_deg)
            dirs.append(direction)
            mags.append(result_map[i, j])
    dirs = np.array(dirs)
    mags = np.array(mags)

    origins = np.zeros_like(dirs)
    U = dirs[:, 0] * mags
    V = dirs[:, 1] * mags
    W = dirs[:, 2] * mags

    # Reshape to (Npitch, Nyaw) grids for surface plotting
    endpoints = origins + np.column_stack((U, V, W))
    X = endpoints[:, 0].reshape(Npitch, Nyaw)
    Y = endpoints[:, 1].reshape(Npitch, Nyaw)
    Z = endpoints[:, 2].reshape(Npitch, Nyaw)

    # Colormap by magnitude
    norm = plt.Normalize(result_map.min(), result_map.max())
    colors = cm.viridis(norm(result_map)) if mode == "force" else cm.plasma(norm(result_map))

    # Create 3D surface plot
    plt.style.use(["science", "no-latex"])

    if mode == "force":
        unit = "N"
        label_prefix = "f"
    else:
        unit = "N·m"
        label_prefix = "τ"

    fig = plt.figure(figsize=(5, 4))
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_surface(
        X, Y, Z, facecolors=colors, rstride=1, cstride=1, linewidth=0, antialiased=False, shade=False
    )
    ax.set_xlabel(f"$^B {label_prefix}_x$ [{unit}]")
    ax.set_ylabel(f"$^B {label_prefix}_y$ [{unit}]")
    ax.set_zlabel(f"$^B {label_prefix}_z$ [{unit}]")
    
    # Set equal aspect ratio
    max_range = max(np.ptp(endpoints[:, 0]), np.ptp(endpoints[:, 1]), np.ptp(endpoints[:, 2]))
    if max_range > 0:
        ax.set_box_aspect((np.ptp(endpoints[:, 0])/max_range, 
                          np.ptp(endpoints[:, 1])/max_range, 
                          np.ptp(endpoints[:, 2])/max_range))
    
    title_suffix = f" ({n_modules} module{'s' if n_modules > 1 else ''})"
    tilt_str = f"tilt{int(tilt_limit_deg)}" if tilt_limit_deg else "notilt"
    ax.set_title(f"Max {mode.capitalize()} (Projection){title_suffix}")
    
    fig_filename = f"body_{mode}_maxproj_{n_modules}modules_{tilt_str}_3d.png"
    fig.savefig(fig_filename, bbox_inches="tight", pad_inches=0.3, dpi=150)
    print(f"Saved figure to '{fig_filename}'.")

    # ---------- 2D X–Z projection scatter plot ----------
    fig2 = plt.figure(figsize=(5, 4))
    ax2 = fig2.add_subplot(111)
    sc2 = ax2.scatter(endpoints[:, 0], endpoints[:, 2], c=mags, cmap="viridis" if mode == "force" else "plasma", s=8)
    ax2.set_xlabel(f"$^B {label_prefix}_x$ [{unit}]")
    ax2.set_ylabel(f"$^B {label_prefix}_z$ [{unit}]")
    ax2.set_aspect("equal", adjustable="box")
    ax2.grid(True)
    ax2.set_title(f"X-Z Projection{title_suffix}")
    divider = make_axes_locatable(ax2)
    cax = divider.append_axes("bottom", size="6%", pad=0.5)
    cb = fig2.colorbar(sc2, cax=cax, orientation="horizontal")
    cb.set_label(f"{mode.capitalize()} [{unit}]")
    plt.tight_layout(rect=[0.0, 0.0, 1.0, 1.1])
    
    fig2_filename = f"body_{mode}_maxproj_{n_modules}modules_{tilt_str}_xz.png"
    fig2.savefig(fig2_filename, bbox_inches="tight", pad_inches=0.3, dpi=150)
    print(f"Saved figure to '{fig2_filename}'.")
    
    # Print summary statistics
    print(f"\n--- Summary ---")
    print(f"Max {mode}: {result_map.max():.2f} {unit}")
    print(f"Min {mode}: {result_map.min():.2f} {unit}")
    print(f"Mean {mode}: {result_map.mean():.2f} {unit}")
    
    plt.show()


if __name__ == "__main__":
    main()
