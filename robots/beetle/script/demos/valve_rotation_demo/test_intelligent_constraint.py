#!/usr/bin/env python
"""
Test script to verify the intelligent dual-fang geometric constraint
"""
import math

def test_dual_fang_geometric_constraint():
    """Test the dual-fang geometric constraint logic"""
    print("=== INTELLIGENT DUAL-FANG GEOMETRIC CONSTRAINT TEST ===\n")
    
    # Test data from the log (simulated)
    valve_center_x, valve_center_y = 2.95, 0.0  # Assumed valve center
    
    # Original problematic positions
    fang1_approach = (3.0773413887993715, 0.3355808848459006)
    fang2_approach = (2.765346352138349, -0.25198892320426547)
    fang1_final = (3.152355571325218, 0.2636768963262563)
    fang2_final = (2.7363341586866654, -0.15221115124990564)
    
    print("Original dual-fang positions:")
    print(f"  Fang1 approach: {fang1_approach}")
    print(f"  Fang2 approach: {fang2_approach}")
    print(f"  Fang1 final: {fang1_final}")
    print(f"  Fang2 final: {fang2_final}")
    
    # Test coordination without constraint (old behavior)
    coordination_weight = 0.7
    primary_fang_id = 'fang2'  # From log
    
    def calculate_coordinated_position(fang1_pos, fang2_pos, primary_fang_id, coordination_weight):
        if primary_fang_id == 'fang1':
            coordinated_x = coordination_weight * fang1_pos[0] + (1-coordination_weight) * fang2_pos[0]
            coordinated_y = coordination_weight * fang1_pos[1] + (1-coordination_weight) * fang2_pos[1]
        else:
            coordinated_x = coordination_weight * fang2_pos[0] + (1-coordination_weight) * fang1_pos[0]
            coordinated_y = coordination_weight * fang2_pos[1] + (1-coordination_weight) * fang1_pos[1]
        return coordinated_x, coordinated_y
    
    # Test approach phase
    print(f"\n=== APPROACH PHASE TEST ===")
    coordinated_approach_x, coordinated_approach_y = calculate_coordinated_position(
        fang1_approach, fang2_approach, primary_fang_id, coordination_weight)
    
    # Calculate dual-fang geometric constraint
    fang_midpoint_x = (fang1_approach[0] + fang2_approach[0]) / 2.0
    fang_midpoint_y = (fang1_approach[1] + fang2_approach[1]) / 2.0
    
    fang_midpoint_to_valve_dist = math.sqrt((fang_midpoint_x - valve_center_x)**2 + 
                                           (fang_midpoint_y - valve_center_y)**2)
    uav_to_valve_dist = math.sqrt((coordinated_approach_x - valve_center_x)**2 + 
                                 (coordinated_approach_y - valve_center_y)**2)
    
    print(f"Fang midpoint: ({fang_midpoint_x:.4f}, {fang_midpoint_y:.4f})")
    print(f"Fang midpoint to valve distance: {fang_midpoint_to_valve_dist:.4f}m")
    print(f"UAV center to valve distance: {uav_to_valve_dist:.4f}m")
    print(f"Constraint satisfied: {uav_to_valve_dist >= fang_midpoint_to_valve_dist}")
    print(f"Distance ratio: {uav_to_valve_dist/fang_midpoint_to_valve_dist:.3f} (should be > 1.0)")
    
    # Apply intelligent constraint
    def apply_intelligent_constraint(coordinated_x, coordinated_y, fang_midpoint_to_valve_dist, 
                                   valve_center_x, valve_center_y, safety_margin=0.02):
        uav_to_valve_dist = math.sqrt((coordinated_x - valve_center_x)**2 + 
                                     (coordinated_y - valve_center_y)**2)
        
        min_required_uav_distance = fang_midpoint_to_valve_dist + safety_margin
        
        if uav_to_valve_dist < min_required_uav_distance:
            # Adjust UAV position
            if uav_to_valve_dist > 0.001:  # Avoid division by zero
                direction_x = (coordinated_x - valve_center_x) / uav_to_valve_dist
                direction_y = (coordinated_y - valve_center_y) / uav_to_valve_dist
            else:
                direction_x = 1.0
                direction_y = 0.0
            
            coordinated_x = valve_center_x + direction_x * min_required_uav_distance
            coordinated_y = valve_center_y + direction_y * min_required_uav_distance
            
            uav_to_valve_dist = math.sqrt((coordinated_x - valve_center_x)**2 + 
                                         (coordinated_y - valve_center_y)**2)
            
            return coordinated_x, coordinated_y, True
        
        return coordinated_x, coordinated_y, False
    
    # Test constraint application
    constrained_x, constrained_y, was_adjusted = apply_intelligent_constraint(
        coordinated_approach_x, coordinated_approach_y, fang_midpoint_to_valve_dist,
        valve_center_x, valve_center_y, 0.02)
    
    print(f"\nAfter intelligent constraint:")
    print(f"  Adjusted position: ({constrained_x:.4f}, {constrained_y:.4f}) - {'ADJUSTED' if was_adjusted else 'NO CHANGE'}")
    
    final_uav_to_valve_dist = math.sqrt((constrained_x - valve_center_x)**2 + 
                                       (constrained_y - valve_center_y)**2)
    print(f"  Final UAV to valve distance: {final_uav_to_valve_dist:.4f}m")
    print(f"  Final constraint satisfied: {final_uav_to_valve_dist >= fang_midpoint_to_valve_dist}")
    print(f"  Final distance ratio: {final_uav_to_valve_dist/fang_midpoint_to_valve_dist:.3f}")
    
    # Test final phase
    print(f"\n=== FINAL PHASE TEST ===")
    coordinated_final_x, coordinated_final_y = calculate_coordinated_position(
        fang1_final, fang2_final, primary_fang_id, coordination_weight)
    
    # Calculate dual-fang geometric constraint for final phase
    fang_final_midpoint_x = (fang1_final[0] + fang2_final[0]) / 2.0
    fang_final_midpoint_y = (fang1_final[1] + fang2_final[1]) / 2.0
    
    fang_final_midpoint_to_valve_dist = math.sqrt((fang_final_midpoint_x - valve_center_x)**2 + 
                                                 (fang_final_midpoint_y - valve_center_y)**2)
    uav_final_to_valve_dist = math.sqrt((coordinated_final_x - valve_center_x)**2 + 
                                       (coordinated_final_y - valve_center_y)**2)
    
    print(f"Final fang midpoint: ({fang_final_midpoint_x:.4f}, {fang_final_midpoint_y:.4f})")
    print(f"Final fang midpoint to valve distance: {fang_final_midpoint_to_valve_dist:.4f}m")
    print(f"Final UAV center to valve distance: {uav_final_to_valve_dist:.4f}m")
    print(f"Final constraint satisfied: {uav_final_to_valve_dist >= fang_final_midpoint_to_valve_dist}")
    print(f"Final distance ratio: {uav_final_to_valve_dist/fang_final_midpoint_to_valve_dist:.3f}")
    
    # Apply constraint to final phase
    constrained_final_x, constrained_final_y, was_final_adjusted = apply_intelligent_constraint(
        coordinated_final_x, coordinated_final_y, fang_final_midpoint_to_valve_dist,
        valve_center_x, valve_center_y, 0.015)
    
    print(f"\nAfter final intelligent constraint:")
    print(f"  Adjusted final position: ({constrained_final_x:.4f}, {constrained_final_y:.4f}) - {'ADJUSTED' if was_final_adjusted else 'NO CHANGE'}")
    
    final_final_uav_to_valve_dist = math.sqrt((constrained_final_x - valve_center_x)**2 + 
                                             (constrained_final_y - valve_center_y)**2)
    print(f"  Final UAV to valve distance: {final_final_uav_to_valve_dist:.4f}m")
    print(f"  Final constraint satisfied: {final_final_uav_to_valve_dist >= fang_final_midpoint_to_valve_dist}")
    print(f"  Final distance ratio: {final_final_uav_to_valve_dist/fang_final_midpoint_to_valve_dist:.3f}")
    
    print(f"\n=== CONSTRAINT EFFECTIVENESS SUMMARY ===")
    print(f"Approach phase: {'IMPROVED' if was_adjusted else 'ALREADY GOOD'}")
    print(f"Final phase: {'IMPROVED' if was_final_adjusted else 'ALREADY GOOD'}")
    print(f"The constraint ensures UAV center is always farther from valve than fang midpoint")
    print(f"This maintains proper dual-fang geometric relationship while preventing over-center movement")

if __name__ == '__main__':
    test_dual_fang_geometric_constraint()
