#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from insertion_optimizer import InsertionOptimizer

def test_dual_fang_refactor():
    """Test the refactored dual-fang only system"""
    print("Testing dual-fang refactor...")
    
    # Create optimizer
    optimizer = InsertionOptimizer()
    print("✅ InsertionOptimizer created successfully")
    
    # Test dual-fang strategy
    dual_strategy = {
        'fang1': {
            'fang_id': 'fang1',
            'config': {'name': 'fang1', 'approach_bias': 0.0},
            'insertion_angle': 1.047,
            'insertion_position': (0.0, 0.0, 0.0),
            'approach_distance': 0.5,
            'yaw_adjustment': 0.0,
            'complexity_score': 1.0,
            'beam_gap': 0.1,
            'optimal_clearance_angle': 1.047,
            'adjacent_beam_clearance': 1.047
        },
        'fang2': {
            'fang_id': 'fang2',
            'config': {'name': 'fang2', 'approach_bias': 0.0},
            'insertion_angle': 3.665,
            'insertion_position': (0.0, 0.0, 0.0),
            'approach_distance': 0.5,
            'yaw_adjustment': 0.0,
            'complexity_score': 0.8,
            'beam_gap': 0.1,
            'optimal_clearance_angle': 1.047,
            'adjacent_beam_clearance': 1.047
        },
        'insertion_mode': 'dual_simultaneous',
        'coordination_required': True,
        'primary_fang': 'fang1',
        'secondary_fang': 'fang2'
    }
    
    try:
        # Get insertion parameters
        insertion_params = optimizer.get_insertion_parameters(
            dual_strategy=dual_strategy,
            pre_insertion_distance=0.03,
            circumferential_offset=0.02
        )
        
        print("✅ Dual-fang strategy processing successful!")
        print(f"Primary fang: {insertion_params['primary_fang']}")
        print(f"Secondary fang: {insertion_params['secondary_fang']}")
        print(f"Insertion mode: {insertion_params['insertion_mode']}")
        
        # Verify parameter extraction
        primary_fang_params = insertion_params[insertion_params['primary_fang']]
        print(f"Primary fang approach position: {primary_fang_params['approach_position']}")
        print(f"Primary fang target yaw: {primary_fang_params['target_yaw']}")
        
        print("✅ All tests passed! Dual-fang refactor is successful.")
        return True
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_dual_fang_refactor()
    sys.exit(0 if success else 1)
