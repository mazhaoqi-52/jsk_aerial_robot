#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from insertion_optimizer import InsertionOptimizer

def test_final_position_fix():
    """Test that the final_position KeyError is fixed"""
    print("Testing final_position KeyError fix...")
    
    # Create optimizer
    optimizer = InsertionOptimizer()
    print("✅ InsertionOptimizer created successfully")
    
    # Test dual-fang strategy with complete parameters
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
        
        # Test primary fang parameter extraction (what the fixed code does)
        primary_fang = insertion_params.get('primary_fang', 'fang1')
        primary_fang_params = insertion_params.get(primary_fang, {})
        
        if not primary_fang_params:
            print(f"❌ Primary fang parameters not found for {primary_fang}")
            return False
        
        # Test the specific parameters that were causing KeyError
        required_params = ['final_position', 'target_yaw', 'approach_position']
        for param in required_params:
            if param not in primary_fang_params:
                print(f"❌ Missing parameter: {param}")
                return False
            else:
                print(f"✅ Found parameter: {param} = {primary_fang_params[param]}")
        
        print("✅ All required parameters found in primary fang structure!")
        print("✅ KeyError: 'final_position' should be fixed!")
        
        return True
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_final_position_fix()
    if success:
        print("\n🎉 Fix verified: dual-fang parameter extraction working correctly!")
    else:
        print("\n❌ Fix needs more work")
    sys.exit(0 if success else 1)
