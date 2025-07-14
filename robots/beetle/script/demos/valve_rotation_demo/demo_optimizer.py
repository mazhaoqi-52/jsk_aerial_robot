#!/usr/bin/env python
"""
Quick demonstration of the insertion optimizer functionality
"""

import sys
import os
sys.path.append('.')
from insertion_optimizer import InsertionOptimizer
import math

def demonstrate_optimizer():
    print("=== INSERTION OPTIMIZER DEMONSTRATION ===")
    
    # Create optimizer
    optimizer = InsertionOptimizer()
    
    # Test scenarios
    test_cases = [
        {
            'name': 'UAV closer to left side (should select Fang1)',
            'current_pos': (2.8, -0.2, 1.0),
            'current_yaw': 0.1,
            'valve_pos': (3.0, 0.0, 0.57),
            'valve_yaw': 0.0,
        },
        {
            'name': 'UAV closer to right side (should select Fang2)',
            'current_pos': (3.2, 0.2, 1.0),
            'current_yaw': 2.0,
            'valve_pos': (3.0, 0.0, 0.57),
            'valve_yaw': 0.0,
        },
        {
            'name': 'UAV at equal distance (should select based on yaw)',
            'current_pos': (3.0, 0.3, 1.0),
            'current_yaw': 0.5,
            'valve_pos': (3.0, 0.0, 0.57),
            'valve_yaw': 0.0,
        }
    ]
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n=== TEST CASE {i}: {test_case['name']} ===")
        
        # Run optimization
        strategy = optimizer.evaluate_insertion_strategy(
            current_pos=test_case['current_pos'],
            current_yaw=test_case['current_yaw'],
            valve_pos=test_case['valve_pos'],
            valve_yaw=test_case['valve_yaw']
        )
        
        if strategy:
            print(f"✓ SELECTED: {strategy['config']['name']}")
            print(f"  Complexity score: {strategy['complexity_score']:.3f}")
            print(f"  Approach distance: {strategy['approach_distance']:.3f}m")
            print(f"  Yaw adjustment: {strategy['yaw_adjustment']:.3f} rad ({strategy['yaw_adjustment']*180/math.pi:.1f}°)")
            
            # Get insertion parameters
            parameters = optimizer.get_insertion_parameters(strategy)
            if parameters:
                print(f"  Approach position: [{parameters['approach_position'][0]:.3f}, {parameters['approach_position'][1]:.3f}]")
                print(f"  Final position: [{parameters['final_position'][0]:.3f}, {parameters['final_position'][1]:.3f}]")
        else:
            print("✗ FAILED: No strategy found")
    
    print("\n=== DEMONSTRATION COMPLETE ===")
    print("The optimizer successfully selects the best insertion strategy based on:")
    print("1. Distance to insertion point")
    print("2. Required yaw adjustment")
    print("3. Beam clearance considerations")
    print("4. Fang configuration preferences")

if __name__ == '__main__':
    demonstrate_optimizer()
