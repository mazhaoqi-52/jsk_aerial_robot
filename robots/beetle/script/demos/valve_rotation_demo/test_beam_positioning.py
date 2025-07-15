#!/usr/bin/env python3
"""
Test script to verify beam positioning logic
"""
import sys
import os
import math

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from trajectory import AlignToGraspTrajectory

def test_beam_positioning():
    """Test the beam positioning logic"""
    print("Testing beam positioning logic...")
    
    # Test parameters
    valve_center = (3.0, 0.0, 0.57)  # From the log
    valve_yaw = 0.0  # Assuming beam0 aligns with X-axis
    grasp_height = 0.681
    
    # Create trajectory object
    align_traj = AlignToGraspTrajectory(
        approach_duration=5.0,
        valve_center=valve_center,
        valve_pose_yaw=valve_yaw,
        grasp_height=grasp_height,
        valve_radius=0.1225,
        valve_beam_width=0.0185,
        rotation_direction=1,
        insertion_offset=0.005
    )
    
    # Test the calculation
    print("\n=== Testing beam positioning calculation ===")
    
    try:
        body_pos, body_yaw, end_effector_pos = align_traj.calculate_grasp_position_and_yaw()
        
        print(f"\nCalculated positions:")
        print(f"  Body position: [{body_pos[0]:.3f}, {body_pos[1]:.3f}, {body_pos[2]:.3f}]")
        print(f"  Body yaw: {body_yaw:.3f} rad ({body_yaw*180/math.pi:.1f}°)")
        print(f"  End-effector position: [{end_effector_pos[0]:.3f}, {end_effector_pos[1]:.3f}, {end_effector_pos[2]:.3f}]")
        
        # Check if claw positions are stored
        if hasattr(align_traj, '_claw_positions') and align_traj._claw_positions:
            claw_pos = align_traj._claw_positions
            print(f"\nClaw positions:")
            print(f"  Left claw (beam0-beam1, near beam1): [{claw_pos['claw1_target'][0]:.3f}, {claw_pos['claw1_target'][1]:.3f}] @ {claw_pos['claw1_angle']*180/math.pi:.1f}°")
            print(f"  Right claw (beam1-beam2, near beam2): [{claw_pos['claw2_target'][0]:.3f}, {claw_pos['claw2_target'][1]:.3f}] @ {claw_pos['claw2_angle']*180/math.pi:.1f}°")
            
            print(f"\nBeam positions:")
            print(f"  Beam0 (X-axis): [{claw_pos['beam0_position'][0]:.3f}, {claw_pos['beam0_position'][1]:.3f}]")
            print(f"  Beam1 (120°): [{claw_pos['beam1_position'][0]:.3f}, {claw_pos['beam1_position'][1]:.3f}]")
            print(f"  Beam2 (240°): [{claw_pos['beam2_position'][0]:.3f}, {claw_pos['beam2_position'][1]:.3f}]")
            
            print(f"\nGap angles:")
            print(f"  Gap01 (beam0-beam1): {claw_pos['gap01_angle']*180/math.pi:.1f}°")
            print(f"  Gap12 (beam1-beam2): {claw_pos['gap12_angle']*180/math.pi:.1f}°")
            
            # Verify positioning
            print(f"\n=== Verification ===")
            expected_claw1_range = (60 - 30, 60 + 30)  # 30°-90° range for beam0-beam1 gap
            expected_claw2_range = (180 - 30, 180 + 30)  # 150°-210° range for beam1-beam2 gap
            
            claw1_angle_deg = claw_pos['claw1_angle'] * 180 / math.pi
            claw2_angle_deg = claw_pos['claw2_angle'] * 180 / math.pi
            
            print(f"  Left claw angle: {claw1_angle_deg:.1f}° (expected: {expected_claw1_range[0]:.1f}°-{expected_claw1_range[1]:.1f}°)")
            print(f"  Right claw angle: {claw2_angle_deg:.1f}° (expected: {expected_claw2_range[0]:.1f}°-{expected_claw2_range[1]:.1f}°)")
            
            claw1_ok = expected_claw1_range[0] <= claw1_angle_deg <= expected_claw1_range[1]
            claw2_ok = expected_claw2_range[0] <= claw2_angle_deg <= expected_claw2_range[1]
            
            print(f"  Left claw positioning: {'✓ CORRECT' if claw1_ok else '✗ INCORRECT'}")
            print(f"  Right claw positioning: {'✓ CORRECT' if claw2_ok else '✗ INCORRECT'}")
            
            if claw1_ok and claw2_ok:
                print(f"\n✓ SUCCESS: Beam positioning logic is correct!")
                return True
            else:
                print(f"\n✗ FAILED: Beam positioning logic needs adjustment.")
                return False
        else:
            print("Warning: Claw positions not calculated properly")
            return False
            
    except Exception as e:
        print(f"Error during calculation: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_beam_positioning()
    sys.exit(0 if success else 1)
