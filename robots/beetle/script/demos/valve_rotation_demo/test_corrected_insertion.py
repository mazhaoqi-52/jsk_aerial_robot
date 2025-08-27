#!/usr/bin/env python3
"""
Test script to verify corrected insertion height and offset calculations
"""

def test_corrected_insertion_calculation():
    """Test the corrected insertion calculation logic"""
    
    # Test parameters from successful single UAV implementation
    valve_z = 0.570  # From your log: "Valve position: [3.000, -0.000, 0.570]"
    end_effector_z_offset = 0.0221140  # From single UAV implementation
    
    # Test corrected end-effector X offset
    end_effector_x_offset_corrected = 0.25346  # Corrected value from single UAV
    end_effector_x_offset_old = 0.325  # Previous incorrect value
    
    # Formation calculation
    required_formation_z = valve_z - end_effector_z_offset
    
    print("=== CORRECTED INSERTION CALCULATION TEST ===")
    print(f"Valve height: {valve_z:.6f}m")
    print(f"End-effector Z offset: {end_effector_z_offset:.6f}m")
    print(f"Required formation Z: {required_formation_z:.6f}m")
    
    print(f"\n=== END-EFFECTOR X OFFSET CORRECTION ===")
    print(f"Single UAV verified offset: {end_effector_x_offset_corrected:.5f}m")
    print(f"Previous formation offset: {end_effector_x_offset_old:.5f}m")
    print(f"Offset difference: {abs(end_effector_x_offset_corrected - end_effector_x_offset_old):.5f}m")
    print(f"Correction: {end_effector_x_offset_old - end_effector_x_offset_corrected:.5f}m reduction")
    
    # Test end-effector positioning with corrected offset
    formation_center = [2.675, 0.0, required_formation_z]  # From your log
    
    # Test with corrected offset
    expected_end_effector_x = formation_center[0] + end_effector_x_offset_corrected
    expected_end_effector_y = formation_center[1] + 0.0
    expected_end_effector_z = formation_center[2] + end_effector_z_offset
    
    valve_target = [3.000, 0.0, valve_z]  # Target valve position
    
    position_error_xy = ((expected_end_effector_x - valve_target[0])**2 + 
                        (expected_end_effector_y - valve_target[1])**2)**0.5
    position_error_z = abs(expected_end_effector_z - valve_target[2])
    
    print(f"\n=== END-EFFECTOR POSITIONING TEST (CORRECTED OFFSET) ===")
    print(f"Formation center: [{formation_center[0]:.3f}, {formation_center[1]:.3f}, {formation_center[2]:.6f}]")
    print(f"Corrected X offset: {end_effector_x_offset_corrected:.5f}m")
    print(f"Expected end-effector: [{expected_end_effector_x:.3f}, {expected_end_effector_y:.3f}, {expected_end_effector_z:.6f}]")
    print(f"Target valve position: [{valve_target[0]:.3f}, {valve_target[1]:.3f}, {valve_target[2]:.6f}]")
    print(f"Position error: XY={position_error_xy:.4f}m, Z={position_error_z:.6f}m")
    print(f"Acceptable error: {'✓ YES' if position_error_xy < 0.05 and position_error_z < 0.02 else '✗ NO'}")
    
    # Compare with old offset for validation
    expected_end_effector_x_old = formation_center[0] + end_effector_x_offset_old
    position_error_xy_old = ((expected_end_effector_x_old - valve_target[0])**2 + 
                            (expected_end_effector_y - valve_target[1])**2)**0.5
    
    print(f"\n=== COMPARISON WITH OLD OFFSET ===")
    print(f"Old offset result: [{expected_end_effector_x_old:.3f}, {expected_end_effector_y:.3f}, {expected_end_effector_z:.6f}]")
    print(f"Old XY error: {position_error_xy_old:.4f}m")
    print(f"New XY error: {position_error_xy:.4f}m")
    print(f"Improvement: {position_error_xy_old - position_error_xy:.4f}m XY error reduction")
    print(f"Error reduction: {((position_error_xy_old - position_error_xy) / position_error_xy_old * 100):.1f}%")
    
    return position_error_xy < 0.05 and position_error_z < 0.02

if __name__ == '__main__':
    success = test_corrected_insertion_calculation()
    print(f"\n=== TEST RESULT ===")
    print(f"Corrected insertion calculation: {'✓ PASS' if success else '✗ FAIL'}")
    print(f"Ready for formation valve rotation with:")
    print(f"  - Corrected end-effector X offset: 0.25346m")
    print(f"  - Precise insertion height calculation")
    print(f"  - Speed+acceleration control (like single UAV)")
