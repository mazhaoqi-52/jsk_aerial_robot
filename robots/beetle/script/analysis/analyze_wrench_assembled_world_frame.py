"""
Created by li-jinjie on 25-6-1.
Extended for n-module assembled configuration.

Compute maximum thrust or torque in the world frame for n-module assembled 
omnidirectional aerial vehicles, accounting for gravity compensation.

Usage examples:
    python analyze_wrench_assembled_world_frame.py                              # Use config file
    python analyze_wrench_assembled_world_frame.py config_custom.yaml           # Use custom config
    python analyze_wrench_assembled_world_frame.py --mode force --n_modules 2   # Command line args
    python analyze_wrench_assembled_world_frame.py config.yaml -n 3             # Config + override
"""

import os
import yaml
import numpy as np
import cvxpy as cp
import argparse
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib import cm
import scienceplots
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.axes_grid1 import make_axes_locatable

from analyze_allocation_assembled import (
    get_alloc_mtx_assembled,
    get_assembled_physical_params,
    DEFAULT_MODULE_SPACING,
    DEFAULT_THRUST_MAX,
    GRAVITY,
)

# ------------------------------------------------------------------------
# CONSTANTS & DEFAULT CONFIG
# ------------------------------------------------------------------------
THRUST_MAX = DEFAULT_THRUST_MAX  # Maximum thrust per rotor [N] - updated to 21.37N
THRUST_MIN = 0.0  # Minimum thrust per rotor [N]
WRENCH_ERROR_LIMIT = 1e-3  # Acceptable wrench error
DEFAULT_TILT_LIMIT = 90.0  # Default tilt angle limit [degrees]

DEFAULT_CONFIG = {
    "mode": "force",
    "n_modules": 1,
    "module_spacing": DEFAULT_MODULE_SPACING,
    "resolution": 30,
    "tilt_limit": DEFAULT_TILT_LIMIT,
    "thrust_max": THRUST_MAX,
    "save_to_npz": False,
}

DEFAULT_CONFIG_FILE = "config_wrench_analysis.yaml"


def load_config(config_path=None):
    """Load configuration from YAML file, with defaults."""
    config = DEFAULT_CONFIG.copy()
    
    # Try to find config file
    if config_path is None:
        # Check if default config exists in current directory
        if os.path.exists(DEFAULT_CONFIG_FILE):
            config_path = DEFAULT_CONFIG_FILE
    
    if config_path and os.path.exists(config_path):
        with open(config_path, 'r') as f:
            user_config = yaml.safe_load(f)
            if user_config:
                config.update(user_config)
        print(f"Loaded config from: {config_path}")
    
    return config


# ------------------------------------------------------------------------


def check_wrench_available(
    alloc_mtx,
    tgt_wrench,
    n_rotors,
    f_th_max=THRUST_MAX,
    wrench_error_limit=WRENCH_ERROR_LIMIT,
    tilt_limit_deg=None,
):
    """
    Determine whether the target wrench can be produced.
    
    Parameters:
        alloc_mtx: allocation matrix (6 × 2*n_rotors)
        tgt_wrench: desired wrench vector (6×1)
        n_rotors: total number of rotors
        f_th_max: maximum thrust magnitude per rotor
        wrench_error_limit: tolerated wrench error
        tilt_limit_deg: tilt angle limit (±degrees from vertical)
        
    Returns:
        (is_available: bool, wrench_error: float)
    """
    n_vars = 2 * n_rotors
    x = cp.Variable(n_vars)

    objective = cp.Minimize(cp.sum_squares(alloc_mtx @ x - tgt_wrench[:, 0]))

    constraints = []
    
    for i in range(n_rotors):
        fx_idx = 2 * i
        fy_idx = 2 * i + 1
        
        # Thrust magnitude constraint
        constraints.append(x[fx_idx]**2 + x[fy_idx]**2 <= f_th_max**2)
        
        # Tilt angle constraint
        if tilt_limit_deg is not None:
            constraints.append(x[fy_idx] >= 0)
            
            if tilt_limit_deg < 90.0:
                tan_limit = np.tan(np.radians(tilt_limit_deg))
                constraints.append(x[fx_idx] <= x[fy_idx] * tan_limit)
                constraints.append(x[fx_idx] >= -x[fy_idx] * tan_limit)

    prob = cp.Problem(objective, constraints)
    try:
        prob.solve(verbose=False)
    except cp.SolverError as e:
        print(f"Solver error: {e}")
        return False, np.inf

    if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        return False, np.inf

    wrench_error = np.linalg.norm(alloc_mtx @ x.value - tgt_wrench[:, 0])
    return (wrench_error <= wrench_error_limit), wrench_error


def find_max_wrench_for_orientation_world(
    alloc_mtx,
    n_rotors,
    fg_w,
    roll_deg,
    pitch_deg,
    yaw_deg,
    search_min,
    search_max,
    mode="force",
    tol=1e-2,
    max_iters=30,
    tilt_limit_deg=DEFAULT_TILT_LIMIT,
    f_th_max=THRUST_MAX,
):
    """
    Binary search to find the maximum force or torque along the WORLD z-axis
    for a given orientation, accounting for gravity in the world frame.

    Parameters:
        alloc_mtx: allocation matrix (6 × 2*n_rotors)
        n_rotors: total number of rotors
        fg_w: gravity vector in world frame [N]
        roll_deg, pitch_deg, yaw_deg: Euler angles (degrees)
        search_min, search_max: initial search bounds
        mode: "force" or "torque"
        tol: convergence tolerance
        max_iters: maximum binary search iterations
        tilt_limit_deg: tilt angle limit
        f_th_max: maximum thrust per rotor [N]

    Returns:
        best_value: maximum achievable force [N] or torque [N·m]
        best_error: corresponding wrench error
    """
    # Rotation from world frame to body frame
    R_bw = R.from_euler("zyx", [yaw_deg, pitch_deg, roll_deg], degrees=True).as_matrix().T
    fg_b = R_bw @ fg_w

    lo, hi = search_min, search_max
    best_value = search_min
    best_error = np.inf

    iter_idx = 0
    for iter_idx in range(max_iters):
        mid = (lo + hi) / 2
        mid_b = np.array([0.0, 0.0, mid])

        tgt_wrench = np.zeros((6, 1))
        if mode == "force":
            # Force = gravity compensation + desired force along body z
            tgt_wrench[0:3, 0] = fg_b + mid_b
        elif mode == "torque":
            tgt_wrench[0:3, 0] = fg_b
            tgt_wrench[3:6, 0] = mid_b
        else:
            raise ValueError("mode must be 'force' or 'torque'")

        ok, err = check_wrench_available(
            alloc_mtx, tgt_wrench, n_rotors, f_th_max=f_th_max, tilt_limit_deg=tilt_limit_deg
        )
        if ok:
            best_value, best_error = mid, err
            lo = mid
        else:
            hi = mid

        if hi - lo < tol:
            break

    if iter_idx == max_iters - 1:
        print(f"Warning: reached maximum iterations ({max_iters})")

    return best_value, best_error


def main():
    parser = argparse.ArgumentParser(
        description="Analyze maximum force or torque in the WORLD frame for assembled modules.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python analyze_wrench_assembled_world_frame.py                    # Use default config file
  python analyze_wrench_assembled_world_frame.py config.yaml        # Use custom config file
  python analyze_wrench_assembled_world_frame.py -m force -n 2      # Command line args only
  python analyze_wrench_assembled_world_frame.py config.yaml -n 3   # Config file + override
        """
    )
    parser.add_argument(
        "config",
        nargs="?",
        default=None,
        help=f"Path to YAML config file (default: {DEFAULT_CONFIG_FILE} if exists)",
    )
    parser.add_argument(
        "--mode",
        "-m",
        choices=["force", "torque"],
        help="Select 'force' or 'torque'.",
    )
    parser.add_argument(
        "--n_modules",
        "-n",
        type=int,
        help="Number of modules.",
    )
    parser.add_argument(
        "--module_spacing",
        "-s",
        type=float,
        help="Distance between module centers [m].",
    )
    parser.add_argument(
        "--resolution",
        "-r",
        type=float,
        help="Resolution of angles in degrees.",
    )
    parser.add_argument(
        "--tilt_limit",
        "-t",
        type=float,
        help="Tilt angle limit.",
    )
    parser.add_argument(
        "--thrust_max",
        type=float,
        help="Maximum thrust per rotor [N].",
    )
    parser.add_argument("--save_to_npz", action="store_true", help="Save results to .npz file.")

    args = parser.parse_args()

    # Load config from file first
    config = load_config(args.config)

    # Override with command line arguments if provided
    if args.mode is not None:
        config["mode"] = args.mode
    if args.n_modules is not None:
        config["n_modules"] = args.n_modules
    if args.module_spacing is not None:
        config["module_spacing"] = args.module_spacing
    if args.resolution is not None:
        config["resolution"] = args.resolution
    if args.tilt_limit is not None:
        config["tilt_limit"] = args.tilt_limit
    if args.thrust_max is not None:
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

    print(f"Configuration: {n_modules} module(s), spacing={module_spacing}m")
    print(f"Thrust max: {thrust_max}N, Tilt limit: ±{tilt_limit_deg}°")

    # Load allocation matrix and physical parameters
    alloc_mat = get_alloc_mtx_assembled(n_modules, module_spacing)
    n_rotors = 4 * n_modules
    phys_params = get_assembled_physical_params(n_modules, module_spacing)
    
    # Gravity vector in world frame (combined mass)
    fg_w = np.array([0.0, 0.0, phys_params["total_mass"] * GRAVITY])

    print(f"Allocation matrix shape: {alloc_mat.shape}")
    print(f"Total mass: {phys_params['total_mass']:.3f} kg")
    print(f"Gravity compensation: {fg_w[2]:.2f} N")

    yaw_list = np.linspace(-180, 180, int(360 / resolution) + 1)
    pitch_list = np.linspace(0, 180, int(180 / resolution) + 1)

    Nyaw = len(yaw_list)
    Npitch = len(pitch_list)

    result_map = np.zeros((Npitch, Nyaw))
    thrust_limit = n_rotors * thrust_max

    for i, pitch_deg in enumerate(pitch_list):
        max_val_list = []
        for j, yaw_deg in enumerate(yaw_list):
            max_val, _ = find_max_wrench_for_orientation_world(
                alloc_mat,
                n_rotors,
                fg_w,
                roll_deg=0.0,
                pitch_deg=pitch_deg,
                yaw_deg=0.0,
                search_min=0.0,
                search_max=thrust_limit,
                mode=mode,
                tilt_limit_deg=tilt_limit_deg,
                f_th_max=thrust_max,
            )
            max_val_list.append(max_val)
        result_map[i, :] = min(max_val_list)
        print(f"[World {mode.capitalize()}] Completed pitch = {pitch_deg:.1f}°")

    if save_to_npz:
        filename = f"world_{mode}_map_{n_modules}modules.npz"
        np.savez(
            filename,
            yaw_list=yaw_list,
            pitch_list=pitch_list,
            result_map=result_map,
            n_modules=n_modules,
            module_spacing=module_spacing,
            tilt_limit_deg=tilt_limit_deg,
            thrust_max=thrust_max,
            total_mass=phys_params["total_mass"],
        )
        print(f"Saved to '{filename}'.")

    # ---------- Plotting 3D surface (same as original version) ----------
    dirs = []
    mags = []
    for i, pitch_deg in enumerate(pitch_list):
        for j, yaw_deg in enumerate(yaw_list):
            R_wb_zero_roll = R.from_euler("zyx", [yaw_deg, pitch_deg, 0.0], degrees=True).as_matrix().T
            dir_z = R_wb_zero_roll[:, 2]  # world ← body: body z-axis expressed in world
            dirs.append(dir_z)
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
    colors = cm.viridis(norm(result_map))

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
    ax.set_xlabel(f"$^W {label_prefix}_x$ [{unit}]")
    ax.set_ylabel(f"$^W {label_prefix}_y$ [{unit}]")
    ax.set_zlabel(f"$^W {label_prefix}_z$ [{unit}]")
    ax.set_box_aspect((np.ptp(endpoints[:, 0]), np.ptp(endpoints[:, 1]), np.ptp(endpoints[:, 2])))
    
    title_suffix = f" ({n_modules} module{'s' if n_modules > 1 else ''})" if n_modules > 1 else ""
    gravity_note = ", gravity compensated" if mode == "force" else ""
    ax.set_title(f"Max {mode.capitalize()} in World Frame{title_suffix}{gravity_note}")
    
    fig_filename = f"world_{mode}_{n_modules}modules_tilt{int(tilt_limit_deg)}_3d.png"
    fig.savefig(fig_filename, bbox_inches="tight", pad_inches=0.3, dpi=150)
    print(f"Saved figure to '{fig_filename}'.")

    # ---------- 2D X–Z projection scatter plot ----------
    fig2 = plt.figure(figsize=(5, 4))
    ax2 = fig2.add_subplot(111)
    sc2 = ax2.scatter(endpoints[:, 0], endpoints[:, 2], c=mags, cmap="viridis", s=8)
    ax2.set_xlabel(f"$^W {label_prefix}_x$ [{unit}]")
    ax2.set_ylabel(f"$^W {label_prefix}_z$ [{unit}]")
    ax2.set_aspect("equal", adjustable="box")
    ax2.grid(True)
    ax2.set_title(f"X-Z Projection{title_suffix}")
    divider = make_axes_locatable(ax2)
    cax = divider.append_axes("bottom", size="6%", pad=0.5)
    cb = fig2.colorbar(sc2, cax=cax, orientation="horizontal")
    cb.set_label(f"{mode.capitalize()} [{unit}]")
    plt.tight_layout(rect=[0.0, 0.0, 1.0, 1.1])
    
    fig2_filename = f"world_{mode}_{n_modules}modules_tilt{int(tilt_limit_deg)}_xz.png"
    fig2.savefig(fig2_filename, bbox_inches="tight", pad_inches=0.3, dpi=150)
    print(f"Saved figure to '{fig2_filename}'.")
    
    plt.show()


if __name__ == "__main__":
    main()
