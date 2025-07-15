#!/usr/bin/env python
print("=== Testing Current State ===")
import os
files = ['trajectory.py', 'valve_rotation_fang_single.py', 'insertion_optimizer.py', 'constrained_optimizer.py', 'unified_motion_controller.py']
for f in files:
    if os.path.exists(f):
        print(f"✓ {f}: Found")
    else:
        print(f"✗ {f}: Missing")

print("\n=== Testing Dual-Fang Implementation ===")
if os.path.exists('insertion_optimizer.py'):
    with open('insertion_optimizer.py', 'r') as f:
        content = f.read()
    
    methods = ['evaluate_dual_fang_strategy', 'get_dual_fang_insertion_parameters', 'log_dual_fang_insertion_plan']
    for method in methods:
        if method in content:
            print(f"✓ {method}: Found")
        else:
            print(f"✗ {method}: Missing")

print("\n=== Testing Fang Configuration ===")
if os.path.exists('insertion_optimizer.py'):
    with open('insertion_optimizer.py', 'r') as f:
        content = f.read()
    
    if 'beam0_beam1' in content and 'beam1_beam2' in content:
        print("✓ Beam gap preferences: Found")
    else:
        print("✗ Beam gap preferences: Missing")

print("\n=== Test Complete ===")
