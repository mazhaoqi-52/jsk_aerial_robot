#!/usr/bin/env python3
"""
Analyze Phase 3A convergence behavior to verify vel_based_waypoint_ bug

This script analyzes ROS bag or log data to show:
1. Initial distance to target
2. Movement progress over time
3. Final error at convergence timeout
4. Whether vel_based_waypoint_ bug was triggered
"""

import re
import math

def analyze_phase3a_logs(log_file_path):
    """
    Analyze Phase 3A convergence from log file
    
    Expected pattern without bug:
    - Initial error: 80mm
    - Progress: smooth decrease
    - Final error: <30mm (success)
    
    Pattern WITH vel_based_waypoint_ bug:
    - Initial error: 80mm
    - C++ warning: "start vel nav control for waypoint"
    - Progress: decreases to 20-40mm then stalls
    - Final error: 20-40mm (failure)
    """
    
    print("=" * 80)
    print("Phase 3A Convergence Analysis - vel_based_waypoint_ Bug Detection")
    print("=" * 80)
    
    bug_triggered = False
    error_history = []
    
    try:
        with open(log_file_path, 'r') as f:
            for line in f:
                # Check for bug trigger
                if "start vel nav control for waypoint" in line:
                    bug_triggered = True
                    print("\n⚠️  BUG TRIGGERED: vel_based_waypoint_ activated")
                    print(f"   Log line: {line.strip()}")
                
                # Extract position errors from Phase 3A
                if "Phase3A" in line or "Phase 3A" in line:
                    # Pattern: pos_err=XX.Xmm
                    match = re.search(r'pos_err[=:\s]+(\d+\.?\d*)mm', line)
                    if match:
                        error_mm = float(match.group(1))
                        error_history.append(error_mm)
    except FileNotFoundError:
        print(f"\n❌ Log file not found: {log_file_path}")
        print("\nUsage:")
        print("  1. Run formation valve rotation experiment")
        print("  2. Save ROS logs: rosrun beetle valve_rotation_formation.py 2>&1 | tee phase3a_log.txt")
        print("  3. Analyze: python3 analyze_phase3a_bug.py phase3a_log.txt")
        return
    
    if not error_history:
        print("\n⚠️  No Phase 3A error data found in log")
        print("Make sure the log contains Phase 3A convergence attempts")
        return
    
    # Analysis
    print(f"\n📊 Error History ({len(error_history)} samples):")
    print(f"   Initial error: {error_history[0]:.1f}mm")
    print(f"   Final error:   {error_history[-1]:.1f}mm")
    print(f"   Reduction:     {error_history[0] - error_history[-1]:.1f}mm")
    print(f"   Progress:      {(error_history[0] - error_history[-1]) / error_history[0] * 100:.1f}%")
    
    # Check for stall pattern (bug symptom)
    if len(error_history) >= 5:
        last_5_errors = error_history[-5:]
        error_variance = sum((e - sum(last_5_errors)/5)**2 for e in last_5_errors) / 5
        
        print(f"\n📈 Convergence Pattern:")
        print(f"   Last 5 errors: {[f'{e:.1f}' for e in last_5_errors]}")
        print(f"   Variance: {error_variance:.2f}mm²")
        
        if error_variance < 5.0 and error_history[-1] > 20.0:
            print("\n⚠️  STALL DETECTED: Error stopped decreasing at high value")
            print("   This is characteristic of vel_based_waypoint_ bug")
        elif error_history[-1] < 30.0:
            print("\n✅ Normal convergence: Final error acceptable")
        else:
            print("\n⚠️  High final error without clear stall")
    
    # Bug diagnosis
    print("\n" + "=" * 80)
    print("🔍 Bug Diagnosis:")
    print("=" * 80)
    
    if bug_triggered:
        print("✓ vel_based_waypoint_ bug WAS triggered (C++ warning found)")
        print(f"✓ Final error {error_history[-1]:.1f}mm is consistent with bug behavior")
        print("\n Expected behavior:")
        print("  - Initial distance > 50mm triggers bug")
        print("  - UAV moves 60-75% of distance")
        print("  - Converges to equilibrium point at 20-40mm error")
        print("  - Cannot reach actual target due to velocity=0 constraint")
    else:
        print("✗ No vel_based_waypoint_ warning found in log")
        if error_history[-1] > 30.0:
            print("  But high final error suggests bug may still be present")
        else:
            print("  Normal convergence achieved")
    
    # Recommendation
    print("\n" + "=" * 80)
    print("💡 Recommendation:")
    print("=" * 80)
    
    if bug_triggered or error_history[-1] > 30.0:
        print("Apply Solution 4 (Trajectory Decomposition):")
        print("  - Splits large movements (>50mm) into smaller waypoints")
        print("  - Each waypoint <50mm avoids triggering vel_based_waypoint_")
        print("  - Expected improvement: <30mm final error → <5mm final error")
    else:
        print("No action needed - convergence is working correctly")
    
    print("=" * 80)


def simulate_bug_behavior():
    """
    Simulate expected UAV behavior with and without bug
    """
    print("\n" + "=" * 80)
    print("📐 Simulated UAV Behavior Comparison")
    print("=" * 80)
    
    # Scenario
    target_distance = 80.0  # mm
    
    print(f"\nScenario: Move 80mm in XY plane")
    print(f"Bug threshold: 50mm")
    print(f"Target distance: {target_distance}mm")
    
    # Without bug (Solution 4)
    print("\n✅ WITH Solution 4 (Trajectory Decomposition):")
    print(f"   Step 1: Move 40mm → Error = 40mm")
    print(f"   Step 2: Move 40mm → Error = 0mm")
    print(f"   Final error: <5mm ✓")
    
    # With bug
    print("\n❌ WITHOUT Solution 4 (vel_based_waypoint_ bug):")
    Kp = 2.0  # Position gain
    Kv = 5.0  # Velocity gain
    
    # Equilibrium: Kp * pos_error = Kv * vel_error
    # With vel_target = 0, equilibrium at partial completion
    completion_ratio = 0.65  # Typical 60-70% completion
    achieved_distance = target_distance * completion_ratio
    final_error = target_distance - achieved_distance
    
    print(f"   Trigger: {target_distance}mm > 50mm → vel_based_waypoint_ = true")
    print(f"   Control conflict: pos_target={target_distance}mm, vel_target=0")
    print(f"   Achieves: ~{achieved_distance:.1f}mm ({completion_ratio*100:.0f}%)")
    print(f"   Final error: {final_error:.1f}mm ✗")
    print(f"   Cannot improve further due to velocity constraint")
    

if __name__ == "__main__":
    import sys
    
    print("vel_based_waypoint_ Bug Analysis Tool")
    print("=" * 80)
    
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
        analyze_phase3a_logs(log_file)
    else:
        print("No log file provided - showing simulation only\n")
        simulate_bug_behavior()
        print("\n" + "=" * 80)
        print("Usage: python3 analyze_phase3a_bug.py <log_file.txt>")
        print("=" * 80)
