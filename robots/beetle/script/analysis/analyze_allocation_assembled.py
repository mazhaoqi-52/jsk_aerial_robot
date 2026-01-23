"""
Created by li-jinjie on 25-6-1.
Modified by Zhaoqi-MA on 26-1-23
Extended for n-module assembled configuration.

This module provides allocation matrix generation for n-module assembled Beetle robots
connected in a chain along the X-axis.
"""

import numpy as np
import yaml
import os
import rospkg

# Read parameters from yaml
rospack = rospkg.RosPack()

physical_param_path = os.path.join(rospack.get_path("beetle"), "config", "PhysParamBeetleHyper.yaml")
with open(physical_param_path, "r") as f:
    physical_param_dict = yaml.load(f, Loader=yaml.FullLoader)
physical_params = physical_param_dict["physical"]

# Single module parameters
SINGLE_MODULE_MASS = physical_params["mass"]
GRAVITY = physical_params["gravity"]
Ixx_single = physical_params["inertia_diag"][0]
Iyy_single = physical_params["inertia_diag"][1]
Izz_single = physical_params["inertia_diag"][2]

dr1 = physical_params["dr1"]
dr2 = physical_params["dr2"]
dr3 = physical_params["dr3"]
dr4 = physical_params["dr4"]
p1_b = physical_params["p1"]
p2_b = physical_params["p2"]
p3_b = physical_params["p3"]
p4_b = physical_params["p4"]
kq_d_kt = physical_params["kq_d_kt"]

# Default parameters
DEFAULT_MODULE_SPACING = 0.52  # meters, distance between adjacent module centers along X-axis
DEFAULT_THRUST_MAX = 21.37  # N, maximum thrust per rotor


def get_assembled_physical_params(n_modules, module_spacing=DEFAULT_MODULE_SPACING):
    """
    Calculate physical parameters for n-module assembled configuration.
    
    Assumptions:
    - Modules are connected in a chain along the X-axis
    - The combined CoG is at the geometric center of the chain
    - All modules have identical mass and inertia
    
    Parameters:
        n_modules: Number of modules in the assembly
        module_spacing: Distance between adjacent module centers [m]
        
    Returns:
        dict with keys:
            - total_mass: Combined mass [kg]
            - combined_inertia: [Ixx, Iyy, Izz] for combined system [kg·m²]
            - module_offsets: List of X-axis offsets from combined CoG for each module [m]
    """
    total_mass = n_modules * SINGLE_MODULE_MASS
    
    # Calculate module center positions relative to combined CoG
    # For n modules along X-axis, positions are:
    # Module 0: -((n-1)/2) * spacing
    # Module 1: -((n-1)/2 - 1) * spacing
    # ...
    # Module n-1: +((n-1)/2) * spacing
    module_offsets = []
    for i in range(n_modules):
        offset_x = (i - (n_modules - 1) / 2.0) * module_spacing
        module_offsets.append(offset_x)
    
    # Combined inertia using parallel axis theorem
    # For each module at offset d from combined CoG:
    # I_combined = sum(I_local + m * d²)
    Ixx_combined = n_modules * Ixx_single  # No offset in X direction for rotation about X
    Iyy_combined = 0.0
    Izz_combined = 0.0
    
    for offset_x in module_offsets:
        # Parallel axis theorem: I = I_cm + m * d²
        # For Y rotation (pitch) and Z rotation (yaw), the X offset matters
        Iyy_combined += Iyy_single + SINGLE_MODULE_MASS * offset_x**2
        Izz_combined += Izz_single + SINGLE_MODULE_MASS * offset_x**2
    
    return {
        "total_mass": total_mass,
        "combined_inertia": [Ixx_combined, Iyy_combined, Izz_combined],
        "module_offsets": module_offsets,
        "gravity": GRAVITY,
    }


def get_module_rotor_positions(module_index, n_modules, module_spacing=DEFAULT_MODULE_SPACING):
    """
    Get the 4 rotor positions for a specific module in the assembled configuration,
    expressed in the combined body frame (CoG at origin).
    
    Parameters:
        module_index: Index of the module (0 to n_modules-1)
        n_modules: Total number of modules
        module_spacing: Distance between adjacent module centers [m]
        
    Returns:
        List of 4 rotor positions [[x,y,z], ...] in combined body frame
    """
    # Offset of this module's center from combined CoG
    offset_x = (module_index - (n_modules - 1) / 2.0) * module_spacing
    
    # Original rotor positions in single module frame
    p_list = [p1_b, p2_b, p3_b, p4_b]
    
    # Transform to combined body frame
    transformed_positions = []
    for p in p_list:
        new_p = [p[0] + offset_x, p[1], p[2]]
        transformed_positions.append(new_p)
    
    return transformed_positions


def get_alloc_mtx_assembled(n_modules, module_spacing=DEFAULT_MODULE_SPACING):
    """
    Generate allocation matrix for n-module assembled configuration.
    
    The allocation matrix maps rotor force components to body wrench:
    [Fx, Fy, Fz, τx, τy, τz]ᵀ = A · [Fx1, Fy1, Fx2, Fy2, ..., Fx(4n), Fy(4n)]ᵀ
    
    Matrix size: 6 × (8*n_modules)
    
    Parameters:
        n_modules: Number of modules in the assembly
        module_spacing: Distance between adjacent module centers [m]
        
    Returns:
        alloc_matrix: 6 × (8*n_modules) allocation matrix
    """
    n_rotors = 4 * n_modules
    alloc_matrix = np.zeros((6, 2 * n_rotors))
    
    # Rotor direction pattern (same for all modules)
    dr_list = [dr1, dr2, dr3, dr4]
    
    for module_idx in range(n_modules):
        # Get rotor positions for this module in combined body frame
        rotor_positions = get_module_rotor_positions(module_idx, n_modules, module_spacing)
        
        for rotor_idx in range(4):
            # Global rotor index in the assembly
            global_rotor_idx = module_idx * 4 + rotor_idx
            
            p_b = rotor_positions[rotor_idx]
            sqrt_p_xy = np.sqrt(p_b[0]**2 + p_b[1]**2)
            dr = dr_list[rotor_idx]
            
            # Column indices for Fx and Fy of this rotor
            col_fx = 2 * global_rotor_idx
            col_fy = 2 * global_rotor_idx + 1
            
            # Force entries (same as single module)
            alloc_matrix[0, col_fx] = p_b[1] / sqrt_p_xy
            alloc_matrix[1, col_fx] = -p_b[0] / sqrt_p_xy
            alloc_matrix[2, col_fy] = 1
            
            # Torque entries
            alloc_matrix[3, col_fx] = -dr * kq_d_kt * p_b[1] / sqrt_p_xy + p_b[0] * p_b[2] / sqrt_p_xy
            alloc_matrix[4, col_fx] = dr * kq_d_kt * p_b[0] / sqrt_p_xy + p_b[1] * p_b[2] / sqrt_p_xy
            alloc_matrix[5, col_fx] = -p_b[0]**2 / sqrt_p_xy - p_b[1]**2 / sqrt_p_xy
            
            alloc_matrix[3, col_fy] = p_b[1]
            alloc_matrix[4, col_fy] = -p_b[0]
            alloc_matrix[5, col_fy] = -dr * kq_d_kt
    
    return alloc_matrix


def get_alloc_mtx_single():
    """
    Generate allocation matrix for single module (for comparison).
    This is equivalent to get_alloc_mtx_tilt_qd() in analyze_allocation.py.
    """
    return get_alloc_mtx_assembled(n_modules=1, module_spacing=0.0)


def get_end_effector_position(ee_module, ee_offset, n_modules, module_spacing=DEFAULT_MODULE_SPACING):
    """
    Calculate the End Effector position in the combined body frame (CoG at origin).
    
    The EE position is determined by:
    1. The module where EE is attached (ee_module)
    2. The offset from that module's CoG (ee_offset)
    
    Parameters:
        ee_module: Index of the module where EE is attached (0 to n_modules-1)
        ee_offset: [x, y, z] offset from the host module's CoG [m]
                   - From URDF: contact_point is at (0.26, 0, 0) relative to base_link
        n_modules: Total number of modules in the assembly
        module_spacing: Distance between adjacent module centers [m]
        
    Returns:
        numpy array [x, y, z] of EE position in combined body frame
        
    Example:
        For a 3-module assembly with EE on module 2 (last module):
        - Module 2 offset from combined CoG: +0.52m (along X)
        - EE offset from module 2: (0.26, 0, 0)
        - EE position in combined frame: (0.52 + 0.26, 0, 0) = (0.78, 0, 0)
    """
    if ee_module < 0 or ee_module >= n_modules:
        raise ValueError(f"ee_module ({ee_module}) must be in range [0, {n_modules-1}]")
    
    # Get the offset of the host module's center from combined CoG
    module_offset_x = (ee_module - (n_modules - 1) / 2.0) * module_spacing
    
    # EE position = module center position + EE offset relative to module
    ee_position = np.array([
        module_offset_x + ee_offset[0],
        ee_offset[1],
        ee_offset[2]
    ])
    
    return ee_position


def get_ee_wrench_to_cog_wrench_matrix(r_ee):
    """
    Generate the transformation matrix from wrench at EE to equivalent wrench at CoG.
    
    Given:
        - F_ee: Force applied at End Effector
        - τ_ee: Torque applied at End Effector
        - r_ee: Position vector from CoG to EE
        
    The equivalent wrench at CoG is:
        F_cog = F_ee  (force is invariant under translation)
        τ_cog = τ_ee + r_ee × F_ee  (torque is shifted by moment arm)
        
    In matrix form:
        [F_cog]   [  I    0  ] [F_ee]
        [τ_cog] = [[r_ee×] I ] [τ_ee]
        
    Where [r_ee×] is the skew-symmetric matrix of r_ee.
    
    Parameters:
        r_ee: [x, y, z] position of EE in body frame (from CoG)
        
    Returns:
        6×6 transformation matrix
    """
    rx, ry, rz = r_ee
    
    # Skew-symmetric matrix for cross product: [r×] such that [r×]v = r × v
    skew = np.array([
        [0, -rz, ry],
        [rz, 0, -rx],
        [-ry, rx, 0]
    ])
    
    # Build the 6×6 transformation matrix
    T = np.zeros((6, 6))
    T[0:3, 0:3] = np.eye(3)  # F_cog = F_ee
    T[3:6, 0:3] = skew       # τ_cog += r × F_ee
    T[3:6, 3:6] = np.eye(3)  # τ_cog += τ_ee
    
    return T


if __name__ == "__main__":
    print("=" * 60)
    print("Testing allocation matrix for assembled configurations")
    print("=" * 60)
    
    # Test single module
    print("\n--- Single Module ---")
    alloc_1 = get_alloc_mtx_assembled(n_modules=1)
    print(f"Allocation matrix shape: {alloc_1.shape}")
    print(f"Matrix:\n{alloc_1}")
    
    # Test 2 modules
    print("\n--- 2 Modules Assembled ---")
    alloc_2 = get_alloc_mtx_assembled(n_modules=2)
    print(f"Allocation matrix shape: {alloc_2.shape}")
    params_2 = get_assembled_physical_params(n_modules=2)
    print(f"Total mass: {params_2['total_mass']:.3f} kg")
    print(f"Combined inertia: {params_2['combined_inertia']}")
    print(f"Module offsets: {params_2['module_offsets']}")
    
    # Test 3 modules
    print("\n--- 3 Modules Assembled ---")
    alloc_3 = get_alloc_mtx_assembled(n_modules=3)
    print(f"Allocation matrix shape: {alloc_3.shape}")
    params_3 = get_assembled_physical_params(n_modules=3)
    print(f"Total mass: {params_3['total_mass']:.3f} kg")
    print(f"Combined inertia: {params_3['combined_inertia']}")
    print(f"Module offsets: {params_3['module_offsets']}")
    
    # Show rotor positions for 2-module assembly
    print("\n--- Rotor Positions for 2-Module Assembly ---")
    for mod_idx in range(2):
        positions = get_module_rotor_positions(mod_idx, n_modules=2)
        print(f"Module {mod_idx}:")
        for i, pos in enumerate(positions):
            print(f"  Rotor {i+1}: {pos}")
    
    # Test End Effector position calculation
    print("\n" + "=" * 60)
    print("Testing End Effector Position Calculation")
    print("=" * 60)
    
    # Default EE offset from URDF (contact_point)
    ee_offset = [0.26, 0.0, 0.0]
    
    # Single module: EE on module 0
    print("\n--- Single Module, EE on Module 0 ---")
    ee_pos_1 = get_end_effector_position(ee_module=0, ee_offset=ee_offset, n_modules=1)
    print(f"EE position in body frame: {ee_pos_1}")
    
    # 2 modules: EE on module 0 (front)
    print("\n--- 2 Modules, EE on Module 0 (front) ---")
    ee_pos_2_front = get_end_effector_position(ee_module=0, ee_offset=ee_offset, n_modules=2)
    print(f"EE position in body frame: {ee_pos_2_front}")
    
    # 2 modules: EE on module 1 (back)
    print("\n--- 2 Modules, EE on Module 1 (back) ---")
    ee_pos_2_back = get_end_effector_position(ee_module=1, ee_offset=ee_offset, n_modules=2)
    print(f"EE position in body frame: {ee_pos_2_back}")
    
    # 3 modules: EE on module 2 (last)
    print("\n--- 3 Modules, EE on Module 2 (last) ---")
    ee_pos_3_last = get_end_effector_position(ee_module=2, ee_offset=ee_offset, n_modules=3)
    print(f"EE position in body frame: {ee_pos_3_last}")
    
    # Test wrench transformation matrix
    print("\n--- Wrench Transformation Matrix for EE at (0.78, 0, 0) ---")
    T = get_ee_wrench_to_cog_wrench_matrix(ee_pos_3_last)
    print(f"Transformation matrix:\n{T}")
    
    # Example: force at EE -> wrench at CoG
    F_ee = np.array([10, 0, 0, 0, 0, 0])  # 10N force in X direction at EE
    W_cog = T @ F_ee
    print(f"\nApplying 10N in X at EE:")
    print(f"  Force at CoG: {W_cog[0:3]}")
    print(f"  Torque at CoG: {W_cog[3:6]}")
