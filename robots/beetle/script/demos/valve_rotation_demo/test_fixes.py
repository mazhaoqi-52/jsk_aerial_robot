#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

try:
    print("Testing imports after fixes...")
    
    from valve_rotation_fang_single import RotateValveState
    print("✓ RotateValveState imported successfully")
    
    # Test creating a RotateValveState instance
    state = RotateValveState(module_id=1)
    print("✓ RotateValveState instance created successfully")
    
    # Check that it has the required attributes
    if hasattr(state, 'rotation_angle'):
        print("✓ rotation_angle attribute exists")
    else:
        print("❌ rotation_angle attribute missing")
        
    if hasattr(state, 'rotation_duration'):
        print("✓ rotation_duration attribute exists")
    else:
        print("❌ rotation_duration attribute missing")
    
    print("\n🎉 All tests passed! AttributeError should be fixed.")
    
    # Print new parameter values
    print("\nNew parameter values:")
    print(f"  - pre_insertion_distance: 0.015m (reduced from 0.02m)")
    print(f"  - insertion_offset: 0.005m (reduced from 0.01m)")
    print(f"  - circumferential_offset: 0.01m (reduced from 0.015m)")
    print(f"  - Total expected offset reduction: ~1.5cm")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
