#!/usr/bin/env python3
"""
Test script for URDF-based formation parameters integration
"""

import rospy
import math

def test_formation_parameters():
    """Test the formation parameter calculations"""
    
    print("=" * 60)
    print("URDF-based Formation Parameters Test")
    print("=" * 60)
    
    # Simulate different formation configurations
    test_cases = [
        {'module_ids': '1,2', 'dx': 1.0, 'airframe_size': 0.52},
        {'module_ids': '1,2', 'dx': 1.2, 'airframe_size': 0.52},  # Wider formation
        {'module_ids': '1,2', 'dx': 0.8, 'airframe_size': 0.52},  # Closer formation
        {'module_ids': '1,2,3', 'dx': 1.0, 'airframe_size': 0.52},  # 3-UAV formation
    ]
    
    for i, case in enumerate(test_cases):
        print(f"\nTest Case {i+1}: {case['module_ids']} UAVs")
        print("-" * 40)
        
        module_ids = case['module_ids'].split(',')
        uav_spacing = case['dx']
        airframe_size = case['airframe_size']
        
        # End-effector offset from URDF (beetle_fang_joint: xyz="0.246 0 0.0743823")
        end_effector_x_offset = 0.246  # From URDF beetle_fang_joint
        end_effector_insertion_z = 0.0221140  # For valve insertion
        end_effector_trajectory_z = 0.0743823  # For trajectory planning (from URDF)
        
        # Formation center to end-effector offset calculation
        if len(module_ids) == 2:
            # For 2-UAV formation: formation center is at midpoint between UAVs
            # End-effector is at the front UAV position + end_effector_x_offset
            formation_to_end_effector_x = uav_spacing / 2.0 + end_effector_x_offset
        else:
            # For other formations, use default calculation
            formation_to_end_effector_x = end_effector_x_offset
        
        print(f"  Module IDs: {module_ids}")
        print(f"  UAV spacing (dx): {uav_spacing}m")
        print(f"  Airframe size: {airframe_size}m")
        print(f"  End-effector X offset (URDF): {end_effector_x_offset}m")
        print(f"  Formation center to end-effector X: {formation_to_end_effector_x}m")
        print(f"  End-effector insertion Z: {end_effector_insertion_z}m")
        print(f"  End-effector trajectory Z: {end_effector_trajectory_z}m")
        
        # Calculate formation geometry
        if len(module_ids) == 2:
            uav1_x = -uav_spacing / 2.0  # Left UAV
            uav2_x = uav_spacing / 2.0   # Right UAV (with end-effector)
            formation_center_x = 0.0     # Midpoint
            
            print(f"  UAV1 position (X): {uav1_x}m")
            print(f"  UAV2 position (X): {uav2_x}m")
            print(f"  Formation center (X): {formation_center_x}m")
            print(f"  End-effector absolute position: {uav2_x + end_effector_x_offset}m")
            print(f"  Verification: Formation center + offset = {formation_center_x + formation_to_end_effector_x}m")
        
        # Verify calculation consistency
        if len(module_ids) == 2:
            calculated_offset = uav2_x + end_effector_x_offset
            expected_offset = formation_to_end_effector_x
            error = abs(calculated_offset - expected_offset)
            print(f"  Calculation error: {error}m (should be ~0.0)")
            if error < 1e-6:
                print("  ✓ PASS: Offset calculation is consistent")
            else:
                print("  ✗ FAIL: Offset calculation is inconsistent")
    
    print("\n" + "=" * 60)
    print("URDF Parameter Sources:")
    print("=" * 60)
    print("1. beetle_fang_joint XYZ: 0.246 0 0.0743823")
    print("   - X (0.246): End-effector forward offset from UAV base_link")
    print("   - Z (0.0743823): End-effector height for trajectory planning")
    print("   - Z (0.0221140): End-effector height for valve insertion (calculated)")
    print("2. Assembly API airframe_size: 0.52m")
    print("3. Launch file dx parameter: UAV spacing (default 1.0m)")
    print("=" * 60)

if __name__ == '__main__':
    test_formation_parameters()
