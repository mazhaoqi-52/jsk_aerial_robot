#!/usr/bin/env python3
"""
Analyze actual formation geometry from log data
"""

def analyze_formation_geometry():
    """Analyze the actual formation geometry from your log data"""
    
    # Data from your log
    formation_cog_target = [2.675, 0.000, 0.620]  # Formation center target
    expected_end_effector = [2.964, 0.004, 0.784]  # Expected end-effector position
    target_valve_position = [3.000, 0.000, 0.570]  # Target valve position
    
    # Calculate actual offsets
    actual_x_offset = expected_end_effector[0] - formation_cog_target[0]
    actual_y_offset = expected_end_effector[1] - formation_cog_target[1]
    actual_z_offset = expected_end_effector[2] - formation_cog_target[2]
    
    # Calculate errors
    x_error = expected_end_effector[0] - target_valve_position[0]
    y_error = expected_end_effector[1] - target_valve_position[1]
    z_error = expected_end_effector[2] - target_valve_position[2]
    xy_error = (x_error**2 + y_error**2)**0.5
    
    print("=== FORMATION GEOMETRY ANALYSIS FROM LOG DATA ===")
    print(f"Formation CoG target: [{formation_cog_target[0]:.3f}, {formation_cog_target[1]:.3f}, {formation_cog_target[2]:.3f}]")
    print(f"Expected end-effector: [{expected_end_effector[0]:.3f}, {expected_end_effector[1]:.3f}, {expected_end_effector[2]:.3f}]")
    print(f"Target valve position: [{target_valve_position[0]:.3f}, {target_valve_position[1]:.3f}, {target_valve_position[2]:.3f}]")
    
    print(f"\n=== ACTUAL OFFSETS FROM LOG ===")
    print(f"Actual X offset: {actual_x_offset:.5f}m")
    print(f"Actual Y offset: {actual_y_offset:.5f}m")
    print(f"Actual Z offset: {actual_z_offset:.5f}m")
    
    print(f"\n=== POSITIONING ERRORS ===")
    print(f"X error: {x_error:.4f}m")
    print(f"Y error: {y_error:.4f}m")
    print(f"Z error: {z_error:.4f}m")
    print(f"XY error: {xy_error:.4f}m")
    
    # Calculate required formation target to hit valve exactly
    required_formation_x = target_valve_position[0] - actual_x_offset
    required_formation_y = target_valve_position[1] - actual_y_offset
    required_formation_z = target_valve_position[2] - actual_z_offset
    
    print(f"\n=== REQUIRED FORMATION TARGET FOR PERFECT INSERTION ===")
    print(f"Required formation target: [{required_formation_x:.3f}, {required_formation_y:.3f}, {required_formation_z:.3f}]")
    print(f"Current formation target:  [{formation_cog_target[0]:.3f}, {formation_cog_target[1]:.3f}, {formation_cog_target[2]:.3f}]")
    
    formation_correction_x = required_formation_x - formation_cog_target[0]
    formation_correction_y = required_formation_y - formation_cog_target[1]
    formation_correction_z = required_formation_z - formation_cog_target[2]
    
    print(f"Formation correction needed: [{formation_correction_x:.3f}, {formation_correction_y:.3f}, {formation_correction_z:.3f}]")
    
    # Test different end-effector offset assumptions
    print(f"\n=== TESTING DIFFERENT OFFSET ASSUMPTIONS ===")
    
    # Test with original formation offset (0.325)
    test_offset_325 = 0.325
    test_formation_x_325 = target_valve_position[0] - test_offset_325
    test_end_effector_x_325 = test_formation_x_325 + test_offset_325
    error_325 = abs(test_end_effector_x_325 - target_valve_position[0])
    
    print(f"Test offset 0.325m:")
    print(f"  Formation target: [{test_formation_x_325:.3f}, {target_valve_position[1]:.3f}, {target_valve_position[2]:.3f}]")
    print(f"  End-effector result: [{test_end_effector_x_325:.3f}, {target_valve_position[1]:.3f}, {target_valve_position[2]:.3f}]")
    print(f"  X error: {error_325:.6f}m ({'✓ PERFECT' if error_325 < 0.001 else '✗ ERROR'})")
    
    # Test with single UAV offset (0.25346)
    test_offset_25346 = 0.25346
    test_formation_x_25346 = target_valve_position[0] - test_offset_25346
    test_end_effector_x_25346 = test_formation_x_25346 + test_offset_25346
    error_25346 = abs(test_end_effector_x_25346 - target_valve_position[0])
    
    print(f"\nTest offset 0.25346m:")
    print(f"  Formation target: [{test_formation_x_25346:.3f}, {target_valve_position[1]:.3f}, {target_valve_position[2]:.3f}]")
    print(f"  End-effector result: [{test_end_effector_x_25346:.3f}, {target_valve_position[1]:.3f}, {target_valve_position[2]:.3f}]")
    print(f"  X error: {error_25346:.6f}m ({'✓ PERFECT' if error_25346 < 0.001 else '✗ ERROR'})")
    
    # Recommendation
    print(f"\n=== RECOMMENDATION ===")
    if error_325 < error_25346:
        print(f"✓ Use formation offset: 0.325m (perfect for formation geometry)")
        print(f"  This accounts for the formation center-to-end-effector transformation")
        recommended_offset = 0.325
    else:
        print(f"✓ Use single UAV offset: 0.25346m")
        recommended_offset = 0.25346
    
    return recommended_offset

if __name__ == '__main__':
    recommended = analyze_formation_geometry()
    print(f"\nRecommended end-effector X offset for formation: {recommended:.5f}m")
