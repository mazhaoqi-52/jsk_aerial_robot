#!/usr/bin/env python3
"""
Test the updated constrained optimizer without ROS
"""
import math
import numpy as np

def test_body_clearance_constraint():
    """Test the body clearance constraint"""
    
    print("=== TESTING BODY CLEARANCE CONSTRAINT ===")
    
    # Parameters
    valve_radius = 0.1225  # 12.25cm
    end_effector_length = 0.246  # 24.6cm 
    valve_center = (3.0, 0.0)
    
    print(f"Valve radius: {valve_radius:.3f}m ({valve_radius*100:.1f}cm)")
    print(f"End-effector length: {end_effector_length:.3f}m ({end_effector_length*100:.1f}cm)")
    
    # Test different UAV body positions
    test_cases = [
        {
            'name': '错误配置：机体过近(胸部趴在阀门上)',
            'body_pos': (3.0, 0.2),  # 20cm from valve center
            'body_yaw': math.pi,     # Point toward valve
        },
        {
            'name': '正确配置：机体适当距离(胸部离开阀门)',
            'body_pos': (2.75, 0.0),  # 25cm from valve center  
            'body_yaw': math.atan2(0.0 - 0.0, 3.0 - 2.75),  # Point toward valve center
        },
        {
            'name': '边界情况：最小允许距离',
            'body_pos': (2.73, 0.0), # 27cm from valve center
            'body_yaw': 0.0,
        }
    ]
    
    for i, case in enumerate(test_cases, 1):
        print(f"\n{i}. {case['name']}")
        
        body_x, body_y = case['body_pos']
        body_yaw = case['body_yaw']
        
        # Calculate end-effector position
        end_eff_x = body_x + end_effector_length * math.cos(body_yaw)
        end_eff_y = body_y + end_effector_length * math.sin(body_yaw)
        
        # Calculate distances
        body_to_valve = math.sqrt((body_x - valve_center[0])**2 + (body_y - valve_center[1])**2)
        end_eff_to_valve = math.sqrt((end_eff_x - valve_center[0])**2 + (end_eff_y - valve_center[1])**2)
        
        # Check constraints
        # Constraint 1: End-effector should be at valve radius
        radius_error = abs(end_eff_to_valve - valve_radius)
        radius_ok = radius_error < 0.01  # 1cm tolerance
        
        # Constraint 4: Body should be sufficiently far from valve
        # 重新计算：如果end_effector在valve_radius上，机体应该在多远？
        # 机体到阀门距离 = √((end_effector_length)² - (valve_radius)²) + 安全间隙
        theoretical_min = math.sqrt(end_effector_length**2 - valve_radius**2)
        min_body_distance = theoretical_min + 0.05  # 5cm安全间隙
        body_clearance = body_to_valve - min_body_distance
        body_ok = body_clearance >= 0
        
        print(f"  机体位置: [{body_x:.3f}, {body_y:.3f}]")
        print(f"  机体朝向: {body_yaw:.3f} rad ({body_yaw*180/math.pi:.1f}°)")
        print(f"  末端执行器位置: [{end_eff_x:.3f}, {end_eff_y:.3f}]")
        print(f"  机体到阀门距离: {body_to_valve:.3f}m ({body_to_valve*100:.1f}cm)")
        print(f"  末端执行器到阀门距离: {end_eff_to_valve:.3f}m ({end_eff_to_valve*100:.1f}cm)")
        print(f"  半径约束: {'✓ 满足' if radius_ok else '✗ 违反'} (误差: {radius_error*100:.1f}cm)")
        print(f"  机体间隙约束: {'✓ 满足' if body_ok else '✗ 违反'} (需要≥{min_body_distance*100:.1f}cm, 理论最小{theoretical_min*100:.1f}cm)")
        
        if radius_ok and body_ok:
            print(f"  ✅ 配置正确：胸部离开阀门本体，手抓住阀门")
        else:
            print(f"  ❌ 配置错误：可能出现胸部趴在阀门上的问题")
    
    print(f"\n=== 约束条件总结 ===")
    print(f"1. 末端执行器必须在阀门半径上: distance ≈ {valve_radius*100:.1f}cm")
    print(f"2. 机体必须距离阀门中心至少: {min_body_distance*100:.1f}cm")
    print(f"   - 阀门半径: {valve_radius*100:.1f}cm")
    print(f"   - 安全间隙: {15:.1f}cm")
    print(f"   - 总计: {min_body_distance*100:.1f}cm")
    print(f"\n这样确保：胸部离开阀门本体 + 手抓住阀门")

if __name__ == "__main__":
    test_body_clearance_constraint()
