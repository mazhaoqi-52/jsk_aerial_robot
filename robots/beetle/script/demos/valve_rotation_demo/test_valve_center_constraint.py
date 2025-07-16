#!/usr/bin/env python
"""
Test script to verify valve center constraint fixes
"""
import math

def test_valve_center_constraint():
    """Test the valve center constraint logic"""
    print("=== VALVE CENTER CONSTRAINT TEST ===\n")
    
    # Test data from the log
    valve_center_x, valve_center_y = 2.95, 0.0  # Assumed valve center
    
    # Original problematic positions
    fang1_approach = (3.0773413887993715, 0.3355808848459006)
    fang2_approach = (2.765346352138349, -0.25198892320426547)
    fang1_final = (3.152355571325218, 0.2636768963262563)
    fang2_final = (2.7363341586866654, -0.15221115124990564)
    
    print("Original positions:")
    print(f"  Fang1 approach: {fang1_approach}")
    print(f"  Fang2 approach: {fang2_approach}")
    print(f"  Fang1 final: {fang1_final}")
    print(f"  Fang2 final: {fang2_final}")
    
    # Test coordination without constraint (old behavior)
    coordination_weight = 0.7
    primary_fang_id = 'fang2'  # From log
    
    if primary_fang_id == 'fang1':
        coordinated_approach_x = coordination_weight * fang1_approach[0] + (1-coordination_weight) * fang2_approach[0]
        coordinated_approach_y = coordination_weight * fang1_approach[1] + (1-coordination_weight) * fang2_approach[1]
        coordinated_final_x = coordination_weight * fang1_final[0] + (1-coordination_weight) * fang2_final[0]
        coordinated_final_y = coordination_weight * fang1_final[1] + (1-coordination_weight) * fang2_final[1]
    else:
        coordinated_approach_x = coordination_weight * fang2_approach[0] + (1-coordination_weight) * fang1_approach[0]
        coordinated_approach_y = coordination_weight * fang2_approach[1] + (1-coordination_weight) * fang1_approach[1]
        coordinated_final_x = coordination_weight * fang2_final[0] + (1-coordination_weight) * fang1_final[0]
        coordinated_final_y = coordination_weight * fang2_final[1] + (1-coordination_weight) * fang1_final[1]
    
    print(f"\nCoordinated positions (without constraint):")
    print(f"  Approach: ({coordinated_approach_x:.4f}, {coordinated_approach_y:.4f})")
    print(f"  Final: ({coordinated_final_x:.4f}, {coordinated_final_y:.4f})")
    
    # Calculate distances from valve center
    approach_distance = math.sqrt((coordinated_approach_x - valve_center_x)**2 + (coordinated_approach_y - valve_center_y)**2)
    final_distance = math.sqrt((coordinated_final_x - valve_center_x)**2 + (coordinated_final_y - valve_center_y)**2)
    
    print(f"\nDistances from valve center:")
    print(f"  Approach distance: {approach_distance:.3f}m")
    print(f"  Final distance: {final_distance:.3f}m")
    
    # Apply constraints
    def apply_valve_center_constraint(x, y, max_distance):
        distance = math.sqrt((x - valve_center_x)**2 + (y - valve_center_y)**2)
        if distance > max_distance:
            scale_factor = max_distance / distance
            constrained_x = valve_center_x + (x - valve_center_x) * scale_factor
            constrained_y = valve_center_y + (y - valve_center_y) * scale_factor
            return constrained_x, constrained_y, True
        return x, y, False
    
    # Test approach constraint (12cm)
    constrained_approach_x, constrained_approach_y, approach_constrained = apply_valve_center_constraint(
        coordinated_approach_x, coordinated_approach_y, 0.12)
    
    # Test final constraint (10cm)
    constrained_final_x, constrained_final_y, final_constrained = apply_valve_center_constraint(
        coordinated_final_x, coordinated_final_y, 0.10)
    
    print(f"\nWith valve center constraints:")
    print(f"  Approach (max 12cm): ({constrained_approach_x:.4f}, {constrained_approach_y:.4f}) - {'CONSTRAINED' if approach_constrained else 'OK'}")
    print(f"  Final (max 10cm): ({constrained_final_x:.4f}, {constrained_final_y:.4f}) - {'CONSTRAINED' if final_constrained else 'OK'}")
    
    # Final distances
    final_approach_distance = math.sqrt((constrained_approach_x - valve_center_x)**2 + (constrained_approach_y - valve_center_y)**2)
    final_final_distance = math.sqrt((constrained_final_x - valve_center_x)**2 + (constrained_final_y - valve_center_y)**2)
    
    print(f"\nFinal distances from valve center:")
    print(f"  Approach distance: {final_approach_distance:.3f}m (max: 0.120m)")
    print(f"  Final distance: {final_final_distance:.3f}m (max: 0.100m)")
    
    # Test descent constraint (8cm)
    constrained_descent_x, constrained_descent_y, descent_constrained = apply_valve_center_constraint(
        coordinated_final_x, coordinated_final_y, 0.08)
    
    descent_distance = math.sqrt((constrained_descent_x - valve_center_x)**2 + (constrained_descent_y - valve_center_y)**2)
    
    print(f"  Descent distance: {descent_distance:.3f}m (max: 0.080m) - {'CONSTRAINED' if descent_constrained else 'OK'}")
    
    print(f"\n=== CONSTRAINT EFFECTIVENESS ===")
    print(f"Before constraint: UAV would move to {approach_distance:.3f}m from valve center")
    print(f"After constraint: UAV moves to {final_approach_distance:.3f}m from valve center")
    print(f"Improvement: {(approach_distance - final_approach_distance)*100:.1f}cm closer to valve center")
    
if __name__ == '__main__':
    test_valve_center_constraint()
