"""
Created by Zhaoqi-MA on 26-1-23.

Compute maximum force or torque at End Effector position in the world frame
for n-module assembled omnidirectional aerial vehicles, accounting for gravity.

The key difference from analyze_wrench_assembled_world_frame.py:
- Instead of computing max wrench at CoG, we compute max wrench at EE
- The wrench at EE is transformed to CoG using the relation:
    F_cog = F_ee
    τ_cog = τ_ee + r_ee × F_ee
- Gravity compensation is still based on total mass
- Force direction in world frame allows EE to push/pull in that direction

Usage examples:
    python analyze_wrench_end_effector_world_frame.py                           # Use config file
    python analyze_wrench_end_effector_world_frame.py --ee_module 1             # EE on module 1
    python analyze_wrench_end_effector_world_frame.py -n 3 --ee_module 2        # 3 modules, EE on last
"""

import os
import yaml
import numpy as np
import cvxpy as cp
import argparse
import rospkg
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from matplotlib import cm
import scienceplots
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from mpl_toolkits.axes_grid1 import make_axes_locatable

from analyze_allocation_assembled import (
    get_alloc_mtx_assembled,
    get_assembled_physical_params,
    get_end_effector_position,
    get_ee_wrench_to_cog_wrench_matrix,
    DEFAULT_MODULE_SPACING,
    DEFAULT_THRUST_MAX,
    GRAVITY,
)

# ------------------------------------------------------------------------
# CONSTANTS & DEFAULT CONFIG
# ------------------------------------------------------------------------
THRUST_MAX = DEFAULT_THRUST_MAX  # Maximum thrust per rotor [N]
THRUST_MIN = 0.0  # Minimum thrust per rotor [N]
WRENCH_ERROR_LIMIT = 1e-2  # Acceptable wrench error tolerance
DEFAULT_TILT_LIMIT = 90.0  # Default tilt angle limit [degrees]

DEFAULT_CONFIG = {
    "mode": "force",
    "n_modules": 1,
    "module_spacing": DEFAULT_MODULE_SPACING,
    "resolution": 30,
    "tilt_limit": DEFAULT_TILT_LIMIT,
    "thrust_max": THRUST_MAX,
    "save_to_npz": False,
    "end_effector_module": 0,
    "end_effector_offset": {"x": 0.26, "y": 0.0, "z": 0.0},
}

# Get default config file path using rospkg
rospack = rospkg.RosPack()
DEFAULT_CONFIG_FILE = os.path.join(rospack.get_path("beetle"), "config", "config_wrench_analysis.yaml")


def load_config(config_path=None):
    """Load configuration from YAML file, with defaults."""
    config = DEFAULT_CONFIG.copy()
    
    if config_path is None:
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


def check_ee_wrench_available(
    alloc_mtx,
    tgt_ee_wrench,
    n_rotors,
    r_ee,
    f_th_max=THRUST_MAX,
    wrench_error_limit=WRENCH_ERROR_LIMIT,
    tilt_limit_deg=None,
):
    """
    Determine whether the target wrench at End Effector can be produced.
    
    Parameters:
        alloc_mtx: allocation matrix (6 × 2*n_rotors) maps rotor forces to CoG wrench
        tgt_ee_wrench: desired wrench at EE [Fx, Fy, Fz, τx, τy, τz]^T (6×1)
        n_rotors: total number of rotors
        r_ee: position of EE relative to CoG [x, y, z]
        f_th_max: maximum thrust magnitude per rotor
        wrench_error_limit: tolerated wrench error
        tilt_limit_deg: tilt angle limit (±degrees from vertical)
        
    Returns:
        (is_available: bool, wrench_error: float)
    """
    # Transform EE wrench to CoG wrench
    T = get_ee_wrench_to_cog_wrench_matrix(r_ee)
    tgt_cog_wrench = T @ tgt_ee_wrench
    
    n_vars = 2 * n_rotors
    x = cp.Variable(n_vars)

    objective = cp.Minimize(cp.sum_squares(alloc_mtx @ x - tgt_cog_wrench[:, 0]))

    constraints = []
    
    for i in range(n_rotors):
        fx_idx = 2 * i
        fy_idx = 2 * i + 1
        
        # Thrust magnitude constraint
        constraints.append(x[fx_idx]**2 + x[fy_idx]**2 <= f_th_max**2)
        
        # Tilt angle constraints
        if tilt_limit_deg is not None:
            constraints.append(x[fy_idx] >= 0)  # Fy ≥ 0
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

    wrench_error = np.linalg.norm(alloc_mtx @ x.value - tgt_cog_wrench[:, 0])
    return (wrench_error <= wrench_error_limit), wrench_error


def find_max_ee_wrench_for_orientation_world(
    alloc_mtx,
    n_rotors,
    r_ee,
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
    Binary search to find the maximum force or torque at End Effector
    along the world z-axis, accounting for gravity compensation.

    Parameters:
        alloc_mtx: allocation matrix (6 × 2*n_rotors)
        n_rotors: total number of rotors
        r_ee: position of EE relative to CoG [x, y, z] in body frame
        fg_w: gravity vector in world frame [N]
        roll_deg, pitch_deg, yaw_deg: Euler angles (degrees)
        search_min, search_max: initial search bounds
        mode: "force" or "torque"
        tol: convergence tolerance
        max_iters: maximum binary search iterations
        tilt_limit_deg: tilt angle limit

    Returns:
        best_value: maximum achievable force [N] or torque [N·m] at EE
        best_error: wrench error corresponding to best_value
    """
    # Rotation from world frame to body frame
    R_bw = R.from_euler("zyx", [yaw_deg, pitch_deg, roll_deg], degrees=True).as_matrix().T
    
    # Gravity compensation in body frame
    fg_b = R_bw @ fg_w

    lo, hi = search_min, search_max
    best_value = search_min
    best_error = np.inf

    for iter_idx in range(max_iters):
        mid = (lo + hi) / 2
        
        # Target EE wrench in body frame
        # For force mode: we want to apply force in world z direction → transform to body
        # For torque mode: we want to apply torque in world z direction → transform to body
        mid_w = np.array([0.0, 0.0, mid])  # Target in world frame
        mid_b = R_bw @ mid_w  # Transform to body frame

        tgt_ee_wrench = np.zeros((6, 1))
        if mode == "force":
            tgt_ee_wrench[0:3, 0] = mid_b
        elif mode == "torque":
            tgt_ee_wrench[3:6, 0] = mid_b
        else:
            raise ValueError("mode must be 'force' or 'torque'")

        # Transform EE wrench to CoG wrench
        T = get_ee_wrench_to_cog_wrench_matrix(r_ee)
        tgt_cog_wrench = T @ tgt_ee_wrench
        
        # Add gravity compensation to force component
        if mode == "force":
            # Total force at CoG = EE force + gravity compensation
            tgt_cog_wrench[0:3, 0] += fg_b
        else:
            # For torque mode, still need gravity compensation for hovering
            tgt_cog_wrench[0:3, 0] += fg_b

        # Check if this wrench is achievable
        n_vars = 2 * n_rotors
        x = cp.Variable(n_vars)
        objective = cp.Minimize(cp.sum_squares(alloc_mtx @ x - tgt_cog_wrench[:, 0]))
        
        constraints = []
        for i in range(n_rotors):
            fx_idx = 2 * i
            fy_idx = 2 * i + 1
            constraints.append(x[fx_idx]**2 + x[fy_idx]**2 <= f_th_max**2)
            if tilt_limit_deg is not None:
                constraints.append(x[fy_idx] >= 0)
                if tilt_limit_deg < 90.0:
                    tan_limit = np.tan(np.radians(tilt_limit_deg))
                    constraints.append(x[fx_idx] <= x[fy_idx] * tan_limit)
                    constraints.append(x[fx_idx] >= -x[fy_idx] * tan_limit)
        
        prob = cp.Problem(objective, constraints)
        try:
            prob.solve(verbose=False)
        except cp.SolverError:
            hi = mid
            continue
        
        if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            hi = mid
            continue
        
        wrench_error = np.linalg.norm(alloc_mtx @ x.value - tgt_cog_wrench[:, 0])
        if wrench_error <= WRENCH_ERROR_LIMIT:
            best_value, best_error = mid, wrench_error
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
        description="Analyze maximum force or torque at End Effector in the WORLD frame.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python analyze_wrench_end_effector_world_frame.py                      # Use default config
  python analyze_wrench_end_effector_world_frame.py -m force --ee_module 0
  python analyze_wrench_end_effector_world_frame.py -n 3 --ee_module 2   # EE on last module
        """
    )
    parser.add_argument(
        "config",
        nargs="?",
        default=None,
        help=f"Path to YAML config file (default: {DEFAULT_CONFIG_FILE})",
    )
    parser.add_argument("--mode", "-m", choices=["force", "torque"],
                        help="Select 'force' or 'torque' analysis.")
    parser.add_argument("--n_modules", "-n", type=int,
                        help="Number of modules in the assembly.")
    parser.add_argument("--module_spacing", "-s", type=float,
                        help="Distance between adjacent module centers [m].")
    parser.add_argument("--resolution", "-r", type=float,
                        help="Resolution of angles in degrees.")
    parser.add_argument("--tilt_limit", "-t", type=float,
                        help="Tilt angle limit (±degrees).")
    parser.add_argument("--thrust_max", type=float,
                        help="Maximum thrust per rotor [N].")
    parser.add_argument("--ee_module", type=int,
                        help="Module index where EE is attached (0 to n_modules-1).")
    parser.add_argument("--ee_offset_x", type=float,
                        help="EE offset in X from module CoG [m].")
    parser.add_argument("--ee_offset_y", type=float,
                        help="EE offset in Y from module CoG [m].")
    parser.add_argument("--ee_offset_z", type=float,
                        help="EE offset in Z from module CoG [m].")
    parser.add_argument("--save_to_npz", action="store_true",
                        help="Save results to .npz file.")

    args = parser.parse_args()

    # Load config
    config = load_config(args.config)

    # Override with command line arguments
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
    if args.ee_module is not None:
        config["end_effector_module"] = args.ee_module
    if args.ee_offset_x is not None:
        config["end_effector_offset"]["x"] = args.ee_offset_x
    if args.ee_offset_y is not None:
        config["end_effector_offset"]["y"] = args.ee_offset_y
    if args.ee_offset_z is not None:
        config["end_effector_offset"]["z"] = args.ee_offset_z
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
    ee_module = config["end_effector_module"]
    ee_offset = [
        config["end_effector_offset"]["x"],
        config["end_effector_offset"]["y"],
        config["end_effector_offset"]["z"],
    ]

    # Validate ee_module
    if ee_module < 0 or ee_module >= n_modules:
        raise ValueError(f"ee_module ({ee_module}) must be in range [0, {n_modules-1}]")

    # Calculate EE position in combined body frame
    r_ee = get_end_effector_position(ee_module, ee_offset, n_modules, module_spacing)

    # Get physical parameters
    phys_params = get_assembled_physical_params(n_modules, module_spacing)
    
    # Gravity vector in world frame
    fg_w = np.array([0.0, 0.0, phys_params["total_mass"] * GRAVITY])

    print("=" * 60)
    print("End Effector Wrench Analysis (World Frame)")
    print("=" * 60)
    print(f"Configuration: {n_modules} module(s), spacing={module_spacing}m")
    print(f"Thrust max: {thrust_max}N, Tilt limit: ±{tilt_limit_deg}°")
    print(f"EE Module: {ee_module}, EE Offset: {ee_offset}")
    print(f"EE Position in Body Frame: {r_ee}")
    print(f"Total mass: {phys_params['total_mass']:.3f} kg")
    print(f"Gravity compensation: {fg_w[2]:.2f} N")

    # Load allocation matrix
    alloc_mat = get_alloc_mtx_assembled(n_modules, module_spacing)
    n_rotors = 4 * n_modules

    print(f"Allocation matrix shape: {alloc_mat.shape}")

    # Create grids
    yaw_list = np.linspace(-180, 180, int(360 / resolution) + 1)
    pitch_list = np.linspace(0, 180, int(180 / resolution) + 1)

    Nyaw = len(yaw_list)
    Npitch = len(pitch_list)

    result_map = np.zeros((Npitch, Nyaw))
    thrust_limit = n_rotors * thrust_max

    for i, pitch_deg in enumerate(pitch_list):
        max_val_list = []
        for j, yaw_deg in enumerate(yaw_list):
            max_val, _ = find_max_ee_wrench_for_orientation_world(
                alloc_mat,
                n_rotors,
                r_ee,
                fg_w,
                roll_deg=0.0,
                pitch_deg=pitch_deg,
                yaw_deg=0.0,  # Roll and yaw = 0 for world frame analysis
                search_min=0.0,
                search_max=thrust_limit,
                mode=mode,
                tilt_limit_deg=tilt_limit_deg,
                f_th_max=thrust_max,
            )
            max_val_list.append(max_val)
        result_map[i, :] = min(max_val_list)
        print(f"[EE World {mode.capitalize()}] Completed pitch = {pitch_deg:.1f}°")

    # Save to file
    if save_to_npz:
        filename = f"ee_world_{mode}_map_{n_modules}modules_ee{ee_module}.npz"
        np.savez(
            filename,
            yaw_list=yaw_list,
            pitch_list=pitch_list,
            result_map=result_map,
            n_modules=n_modules,
            module_spacing=module_spacing,
            tilt_limit_deg=tilt_limit_deg,
            thrust_max=thrust_max,
            ee_module=ee_module,
            ee_offset=ee_offset,
            r_ee=r_ee,
            total_mass=phys_params["total_mass"],
        )
        print(f"Saved to '{filename}'.")

    # ---------- Plotting 3D surface ----------
    dirs = []
    mags = []
    for i, pitch_deg in enumerate(pitch_list):
        for j, yaw_deg in enumerate(yaw_list):
            R_wb_zero_roll = R.from_euler("zyx", [yaw_deg, pitch_deg, 0.0], degrees=True).as_matrix().T
            dir_z = R_wb_zero_roll[:, 2]
            dirs.append(dir_z)
            mags.append(result_map[i, j])
    dirs = np.array(dirs)
    mags = np.array(mags)

    origins = np.zeros_like(dirs)
    U = dirs[:, 0] * mags
    V = dirs[:, 1] * mags
    W = dirs[:, 2] * mags

    endpoints = origins + np.column_stack((U, V, W))
    X = endpoints[:, 0].reshape(Npitch, Nyaw)
    Y = endpoints[:, 1].reshape(Npitch, Nyaw)
    Z = endpoints[:, 2].reshape(Npitch, Nyaw)

    norm = plt.Normalize(result_map.min(), result_map.max())
    colors = cm.viridis(norm(result_map))

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
        X, Y, Z, facecolors=colors, rstride=1, cstride=1,
        linewidth=0, antialiased=False, shade=False
    )
    ax.set_xlabel(f"$^W {label_prefix}_x$ [{unit}]")
    ax.set_ylabel(f"$^W {label_prefix}_y$ [{unit}]")
    ax.set_zlabel(f"$^W {label_prefix}_z$ [{unit}]")
    ax.set_box_aspect((np.ptp(endpoints[:, 0]), np.ptp(endpoints[:, 1]), np.ptp(endpoints[:, 2])))
    
    title = f"Max EE {mode.capitalize()} in World Frame\n"
    title += f"({n_modules} module{'s' if n_modules > 1 else ''}, EE on module {ee_module})"
    if mode == "force":
        title += ", gravity compensated"
    ax.set_title(title)
    
    fig_filename = f"ee_world_{mode}_{n_modules}modules_ee{ee_module}_tilt{int(tilt_limit_deg)}_3d.png"
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
    ax2.set_title(f"X-Z Projection (EE on module {ee_module})")
    divider = make_axes_locatable(ax2)
    cax = divider.append_axes("bottom", size="6%", pad=0.5)
    cb = fig2.colorbar(sc2, cax=cax, orientation="horizontal")
    cb.set_label(f"{mode.capitalize()} [{unit}]")
    plt.tight_layout(rect=[0.0, 0.0, 1.0, 1.1])
    
    fig2_filename = f"ee_world_{mode}_{n_modules}modules_ee{ee_module}_tilt{int(tilt_limit_deg)}_xz.png"
    fig2.savefig(fig2_filename, bbox_inches="tight", pad_inches=0.3, dpi=150)
    print(f"Saved figure to '{fig2_filename}'.")

    # Print summary statistics
    print("\n" + "=" * 60)
    print("Summary Statistics")
    print("=" * 60)
    print(f"Mode: {mode}")
    print(f"EE position: {r_ee}")
    print(f"Min {mode}: {result_map.min():.3f} {unit}")
    print(f"Max {mode}: {result_map.max():.3f} {unit}")
    print(f"Mean {mode}: {result_map.mean():.3f} {unit}")
    
    plt.show()


if __name__ == "__main__":
    main()
