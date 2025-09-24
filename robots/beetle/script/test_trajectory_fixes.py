#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Test script to validate polynomial trajectory speed consistency fix
Simulates a Z trajectory movement to verify the 689mm/s -> 10mm/s fix
"""

import sys
import os
import numpy as np
import math

# Add the path to import trajectory module
sys.path.append('/home/ma/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle/script/demos/valve_rotation_demo/')

# Mock rospy for testing
class MockRospy:
    class Time:
        @staticmethod
        def now():
            return MockTime()
    
    class Duration:
        def __init__(self, secs=0.0):
            self.secs = secs
    
    @staticmethod
    def loginfo(msg):
        print(f"INFO: {msg}")
    
    @staticmethod
    def logwarn(msg):
        print(f"WARN: {msg}")
    
    @staticmethod
    def logerr(msg):
        print(f"ERROR: {msg}")
    
    @staticmethod
    def Rate(hz):
        return MockRate(hz)

class MockRate:
    def __init__(self, hz):
        self.hz = hz
    
    def sleep(self):
        pass

class MockTime:
    def __init__(self):
        self._time = 0.0
    
    def to_sec(self):
        return self._time
    
    def set_time(self, t):
        self._time = t

# Set up mock
sys.modules['rospy'] = MockRospy()
mock_time = MockTime()
MockRospy.Time.now = lambda: mock_time

# Now import trajectory
from trajectory import PolynomialTrajectory

def test_trajectory_speed_consistency():
    """Test that trajectory generates expected speeds for Z movement"""
    print("=== POLYNOMIAL TRAJECTORY SPEED CONSISTENCY TEST ===")
    
    # Test parameters matching the real scenario
    start_pos = [0.0, 0.0, 0.0]  # Starting Z position
    target_pos = [0.0, 0.0, -0.02]  # 20mm descent
    z_distance = abs(target_pos[2] - start_pos[2])
    target_speed = 0.01  # 10mm/s as specified in code
    duration = z_distance / target_speed  # Should be 2.0 seconds
    
    print(f"Test setup:")
    print(f"  Z distance: {z_distance*1000:.1f}mm")
    print(f"  Target speed: {target_speed*1000:.1f}mm/s")
    print(f"  Expected duration: {duration:.1f}s")
    
    # Create trajectory
    traj = PolynomialTrajectory(duration)
    traj.generate_trajectory(start_pos, target_pos)
    
    # Test at multiple time points
    test_points = 20
    speeds = []
    positions = []
    
    print(f"\nTesting {test_points} points along trajectory:")
    print("Time(s) | Z_pos(mm) | Z_speed(mm/s) | Expected(mm/s)")
    print("-" * 55)
    
    for i in range(test_points):
        t = i * duration / (test_points - 1)
        mock_time.set_time(t)
        
        # Get position
        pos = traj.evaluate_at_time(t)
        z_pos = pos[2] if pos else 0.0
        positions.append(z_pos)
        
        # Calculate instantaneous speed (if we have previous point)
        if i > 0:
            dt = duration / (test_points - 1)
            z_speed = abs(z_pos - prev_z_pos) / dt
            speeds.append(z_speed)
            
            print(f"{t:7.2f} | {z_pos*1000:8.1f} | {z_speed*1000:10.1f} | {target_speed*1000:11.1f}")
        else:
            print(f"{t:7.2f} | {z_pos*1000:8.1f} |          - |          -")
        
        prev_z_pos = z_pos
    
    # Analyze results
    if speeds:
        avg_speed = np.mean(speeds)
        max_speed = np.max(speeds)
        min_speed = np.min(speeds)
        
        print(f"\nSpeed Analysis:")
        print(f"  Average speed: {avg_speed*1000:.1f}mm/s (target: {target_speed*1000:.1f}mm/s)")
        print(f"  Max speed: {max_speed*1000:.1f}mm/s")
        print(f"  Min speed: {min_speed*1000:.1f}mm/s")
        print(f"  Speed variation: {(max_speed-min_speed)*1000:.1f}mm/s")
        
        # Check if speed is within reasonable bounds
        speed_error = abs(avg_speed - target_speed) / target_speed * 100
        print(f"  Speed error: {speed_error:.1f}%")
        
        if speed_error < 20:  # Within 20% is reasonable for polynomial
            print("✓ PASS: Speed is within acceptable range")
            return True
        else:
            print("✗ FAIL: Speed error too high")
            return False
    else:
        print("✗ FAIL: Could not calculate speeds")
        return False

def test_boundary_conditions():
    """Test that trajectory starts and ends at correct positions"""
    print("\n=== BOUNDARY CONDITIONS TEST ===")
    
    start_pos = [1.0, 2.0, 3.0]
    target_pos = [1.0, 2.0, 2.98]  # 20mm up
    duration = 2.0
    
    traj = PolynomialTrajectory(duration)
    traj.generate_trajectory(start_pos, target_pos)
    
    # Test start position
    start_result = traj.evaluate_at_time(0.0)
    end_result = traj.evaluate_at_time(duration)
    
    start_error = [abs(start_result[i] - start_pos[i]) for i in range(3)]
    end_error = [abs(end_result[i] - target_pos[i]) for i in range(3)]
    
    print(f"Start position:")
    print(f"  Expected: [{start_pos[0]:.6f}, {start_pos[1]:.6f}, {start_pos[2]:.6f}]")
    print(f"  Actual:   [{start_result[0]:.6f}, {start_result[1]:.6f}, {start_result[2]:.6f}]")
    print(f"  Error:    [{start_error[0]*1000:.3f}, {start_error[1]*1000:.3f}, {start_error[2]*1000:.3f}]mm")
    
    print(f"End position:")
    print(f"  Expected: [{target_pos[0]:.6f}, {target_pos[1]:.6f}, {target_pos[2]:.6f}]")
    print(f"  Actual:   [{end_result[0]:.6f}, {end_result[1]:.6f}, {end_result[2]:.6f}]")
    print(f"  Error:    [{end_error[0]*1000:.3f}, {end_error[1]*1000:.3f}, {end_error[2]*1000:.3f}]mm")
    
    max_error = max(max(start_error), max(end_error))
    if max_error < 1e-6:  # 1 micrometer tolerance
        print("✓ PASS: Boundary conditions satisfied")
        return True
    else:
        print(f"✗ FAIL: Boundary error {max_error*1000:.3f}mm too high")
        return False

if __name__ == "__main__":
    print("Testing polynomial trajectory fixes...")
    
    test1_passed = test_trajectory_speed_consistency()
    test2_passed = test_boundary_conditions()
    
    print(f"\n=== TEST SUMMARY ===")
    print(f"Speed consistency test: {'PASS' if test1_passed else 'FAIL'}")
    print(f"Boundary conditions test: {'PASS' if test2_passed else 'FAIL'}")
    
    if test1_passed and test2_passed:
        print("✓ ALL TESTS PASSED - Trajectory fixes are working correctly")
        sys.exit(0)
    else:
        print("✗ SOME TESTS FAILED - Further investigation needed")
        sys.exit(1)