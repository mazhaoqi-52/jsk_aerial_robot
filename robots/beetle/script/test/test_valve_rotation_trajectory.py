#!/usr/bin/env python
"""
Test script for valve rotation trajectory generation
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

import rospy
import math
from trajectory import create_constant_distance_trajectory

def test_trajectory_generation():
    """Test trajectory generation with sample parameters"""
    rospy.init_node('test_trajectory')
    
    # Sample parameters
    current_uav_pos = (0.2, 0.0, 1.0)  # UAV position
    current_uav_yaw = 0.0               # UAV facing positive X
    valve_center = (0.0, 0.0, 1.0)     # Valve at origin
    rotation_angle = math.pi/2          # 90 degrees
    rotation_speed = 0.05               # 0.05 m/s
    
    # Calculate duration
    typical_radius = 0.2
    circumference = 2 * math.pi * typical_radius
    rotation_fraction = abs(rotation_angle) / (2 * math.pi)
    rotation_duration = (circumference * rotation_fraction) / rotation_speed
    
    print(f"Test parameters:")
    print(f"  UAV position: {current_uav_pos}")
    print(f"  UAV yaw: {current_uav_yaw:.3f} rad")
    print(f"  Valve center: {valve_center}")
    print(f"  Rotation angle: {rotation_angle:.3f} rad ({rotation_angle*180/math.pi:.1f}°)")
    print(f"  Rotation duration: {rotation_duration:.1f}s")
    
    # Create trajectory
    trajectory = create_constant_distance_trajectory(
        current_uav_pos=current_uav_pos,
        current_uav_yaw=current_uav_yaw,
        valve_center=valve_center,
        rotation_angle=rotation_angle,
        rotation_duration=rotation_duration
    )
    
    if trajectory is None:
        print("ERROR: Failed to create trajectory")
        return False
    
    print(f"SUCCESS: Trajectory created")
    print(f"  End-effector distance: {trajectory.end_effector_distance:.3f}m")
    print(f"  Start angle: {trajectory.start_angle:.3f}rad")
    
    # Start trajectory and test a few points
    trajectory.start_trajectory()
    
    print("\nTesting trajectory points:")
    for i in range(5):
        result = trajectory.get_next_position_and_yaw()
        if result is None:
            print(f"  Point {i}: Trajectory completed")
            break
        
        pos, yaw = result
        print(f"  Point {i}: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}), yaw={yaw:.3f}rad")
        
        # Sleep to simulate time passing
        rospy.sleep(rotation_duration / 10)
    
    print("\nTest completed successfully!")
    return True

if __name__ == '__main__':
    try:
        success = test_trajectory_generation()
        if success:
            print("All tests passed!")
        else:
            print("Tests failed!")
    except Exception as e:
        print(f"Test error: {e}")
