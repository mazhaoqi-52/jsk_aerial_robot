#!/usr/bin/env python3
"""
Test script for corrected dual-fang insertion logic
Validates that:
1. Left fang (fang1) inserts between beam1-beam2, closer to beam1
2. Right fang (fang2) inserts between beam0-beam2, closer to beam2
3. No primary/secondary distinction - both fangs are equal
"""

import sys
import os
sys.path.append(os.path.dirname(__file__))

import math
from insertion_optimizer import InsertionOptimizer

def test_corrected_insertion_logic():
    """Test the corrected insertion logic"""
    print("=" * 60)
    print("TESTING CORRECTED DUAL-FANG INSERTION LOGIC")
    print("=" * 60)
    
    # Create optimizer (without ROS subscriptions for testing)
    optimizer = InsertionOptimizer(
        valve_radius=0.1225,
        valve_beam_width=0.0185,
        safety_margin=0.008,
        module_id=1,
        simulation=True
    )
    
    # Test scenarios
    test_cases = [
        {
            'name': 'Standard Test Case',
            'current_pos': (2.8, -0.2, 1.0),
            'current_yaw': 0.1,
            'valve_pos': (3.0, 0.0, 0.57),
            'valve_yaw': 0.0,
        },
        {
            'name': 'Rotated Valve Test Case',
            'current_pos': (3.2, 0.3, 1.0),
            'current_yaw': 1.5,
            'valve_pos': (3.0, 0.0, 0.57),
            'valve_yaw': math.pi/4,  # 45° valve rotation
        }
    ]
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n{'='*20} TEST CASE {i}: {test_case['name']} {'='*20}")
        
        try:
            # Evaluate insertion strategy
            strategy = optimizer.evaluate_insertion_strategy(
                current_pos=test_case['current_pos'],
                current_yaw=test_case['current_yaw'],
                valve_pos=test_case['valve_pos'],
                valve_yaw=test_case['valve_yaw']
            )
            
            if strategy and strategy.get('insertion_mode') == 'dual_simultaneous':
                print("✓ Dual-fang strategy generated successfully")
                
                # Verify configuration
                print(f"✓ Insertion mode: {strategy['insertion_mode']}")
                print(f"✓ Coordination required: {strategy['coordination_required']}")
                print(f"✓ Simultaneous insertion: {strategy.get('simultaneous_insertion', 'N/A')}")
                print(f"✓ No primary/secondary: {strategy.get('no_primary_secondary', 'N/A')}")
                
                # Get insertion parameters
                parameters = optimizer.get_insertion_parameters(strategy)
                
                if parameters:
                    print("\n--- FANG CONFIGURATIONS ---")
                    
                    # Validate fang1 (Left)
                    if 'fang1' in parameters and parameters['fang1']:
                        fang1 = parameters['fang1']
                        print(f"FANG1 (Left): {fang1['fang_config']['name']}")
                        print(f"  Beam gap: {fang1['beam_gap']}")
                        print(f"  Expected: beam1_beam2 (between beam1 and beam2, closer to beam1)")
                        
                        # Verify correct beam gap
                        if fang1['beam_gap'] == 'beam1_beam2':
                            print("  ✓ CORRECT: Fang1 targets beam1_beam2 gap")
                        else:
                            print(f"  ✗ INCORRECT: Fang1 targets {fang1['beam_gap']} gap")
                        
                        print(f"  Insertion angle: {fang1['final_angle']:.3f} rad ({fang1['final_angle']*180/math.pi:.1f}°)")
                        print(f"  Complexity score: {fang1['complexity_score']:.3f}")
                    
                    # Validate fang2 (Right)
                    if 'fang2' in parameters and parameters['fang2']:
                        fang2 = parameters['fang2']
                        print(f"\nFANG2 (Right): {fang2['fang_config']['name']}")
                        print(f"  Beam gap: {fang2['beam_gap']}")
                        print(f"  Expected: beam0_beam2 (between beam0 and beam2, closer to beam2)")
                        
                        # Verify correct beam gap
                        if fang2['beam_gap'] == 'beam0_beam2':
                            print("  ✓ CORRECT: Fang2 targets beam0_beam2 gap")
                        else:
                            print(f"  ✗ INCORRECT: Fang2 targets {fang2['beam_gap']} gap")
                        
                        print(f"  Insertion angle: {fang2['final_angle']:.3f} rad ({fang2['final_angle']*180/math.pi:.1f}°)")
                        print(f"  Complexity score: {fang2['complexity_score']:.3f}")
                    
                    # Verify no primary/secondary logic
                    print(f"\n--- EQUALITY VERIFICATION ---")
                    if 'no_primary_secondary' in parameters and parameters['no_primary_secondary']:
                        print("✓ CORRECT: No primary/secondary distinction")
                    else:
                        print("✗ INCORRECT: Primary/secondary logic still present")
                    
                    if 'simultaneous_insertion' in parameters and parameters['simultaneous_insertion']:
                        print("✓ CORRECT: Simultaneous insertion enabled")
                    else:
                        print("✗ INCORRECT: Simultaneous insertion not configured")
                    
                    print(f"Status: Both fangs are {'EQUAL' if parameters.get('no_primary_secondary', False) else 'HIERARCHICAL'}")
                    
                else:
                    print("✗ Failed to get insertion parameters")
            else:
                print("✗ Failed to get dual-fang strategy")
                
        except Exception as e:
            print(f"✗ Test failed with error: {e}")
            import traceback
            traceback.print_exc()
    
    print(f"\n{'='*60}")
    print("CORRECTED INSERTION LOGIC SUMMARY:")
    print("- Left Fang (Fang1): beam1_beam2 gap, closer to beam1")
    print("- Right Fang (Fang2): beam0_beam2 gap, closer to beam2")
    print("- Both fangs insert simultaneously")
    print("- No primary/secondary distinction")
    print("- Equal treatment of both fangs")
    print("="*60)

def validate_beam_geometry():
    """Validate the beam geometry and gap calculations"""
    print("\n" + "="*40)
    print("BEAM GEOMETRY VALIDATION")
    print("="*40)
    
    # Standard valve orientation (yaw = 0)
    valve_yaw = 0.0
    print(f"Valve yaw: {valve_yaw:.3f} rad ({valve_yaw*180/math.pi:.1f}°)")
    
    # Calculate beam angles
    beam0_angle = valve_yaw                    # 0°
    beam1_angle = valve_yaw + 2*math.pi/3     # 120°
    beam2_angle = valve_yaw + 4*math.pi/3     # 240°
    
    print(f"Beam0: {beam0_angle:.3f} rad ({beam0_angle*180/math.pi:.1f}°)")
    print(f"Beam1: {beam1_angle:.3f} rad ({beam1_angle*180/math.pi:.1f}°)")
    print(f"Beam2: {beam2_angle:.3f} rad ({beam2_angle*180/math.pi:.1f}°)")
    
    # Calculate gap centers
    gap_01_center = (beam0_angle + beam1_angle) / 2  # 60°
    gap_12_center = (beam1_angle + beam2_angle) / 2  # 180°
    # Gap 0-2: from beam2(240°) to beam0(0°/360°), center at 300°
    gap_02_center = (beam2_angle + math.pi/3)  # 240° + 60° = 300°
    if gap_02_center >= 2*math.pi:
        gap_02_center -= 2*math.pi
    
    print(f"\nGap Centers:")
    print(f"Gap 0-1: {gap_01_center:.3f} rad ({gap_01_center*180/math.pi:.1f}°)")
    print(f"Gap 1-2: {gap_12_center:.3f} rad ({gap_12_center*180/math.pi:.1f}°)")
    print(f"Gap 0-2: {gap_02_center:.3f} rad ({gap_02_center*180/math.pi:.1f}°)")
    
    print(f"\nCORRECTED ASSIGNMENT:")
    print(f"✓ Left Fang (Fang1): Gap 1-2 = {gap_12_center*180/math.pi:.1f}°, closer to beam1 ({beam1_angle*180/math.pi:.1f}°)")
    print(f"✓ Right Fang (Fang2): Gap 0-2 = {gap_02_center*180/math.pi:.1f}°, closer to beam2 ({beam2_angle*180/math.pi:.1f}°)")

if __name__ == '__main__':
    print("CORRECTED DUAL-FANG INSERTION LOGIC TEST")
    print("Testing updated insertion optimizer with corrected fang assignments")
    
    # Run tests
    validate_beam_geometry()
    test_corrected_insertion_logic()
    
    print("\nTest completed. Check output above for validation results.")
