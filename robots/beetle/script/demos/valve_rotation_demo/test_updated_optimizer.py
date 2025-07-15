#!/usr/bin/env python3
"""
Test the updated constrained optimizer with proper initial guess
"""
import sys
import math
sys.path.append('/home/ma/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle/script/demos/valve_rotation_demo')

from constrained_optimizer import ConstrainedInsertionOptimizer

def test_updated_optimizer():
    """Test the updated optimizer with geometric initial guess"""
    
    print("=== 测试更新后的约束优化器 ===")
    
    # Create optimizer
    optimizer = ConstrainedInsertionOptimizer()
    
    # Set test parameters
    valve_pos = (3.0, 0.0, 0.0)  # 阀门位置 (元组)
    valve_yaw = 0.0  # 阀门朝向
    current_uav_pos = (2.0, 1.0, 0.0)  # 当前UAV位置 (元组)
    current_uav_yaw = 0.0  # 当前UAV朝向
    
    # Set test parameters directly (simulating ROS callbacks)
    optimizer.current_uav_pos = current_uav_pos
    optimizer.current_uav_yaw = current_uav_yaw
    optimizer.valve_pos = valve_pos
    optimizer.valve_yaw = valve_yaw
    
    print(f"阀门位置: {valve_pos}")
    print(f"阀门朝向: {valve_yaw}")
    print(f"当前UAV位置: {current_uav_pos}")
    print(f"当前UAV朝向: {current_uav_yaw}")
    
    # Test optimization for both gaps
    for target_gap in ['beam0_beam1', 'beam1_beam2']:
        print(f"\n=== 测试 {target_gap} 间隙 ===")
        
        try:
            result = optimizer.optimize_insertion_position(target_gap)
            
            if result and result['success']:
                body_x, body_y, _ = result['body_position']
                body_yaw = result['body_yaw']
                end_eff_x, end_eff_y, _ = result['end_effector_position']
                selected_gap = result['selected_gap']
                
                # Calculate distances
                body_to_valve = math.sqrt((body_x - valve_pos[0])**2 + (body_y - valve_pos[1])**2)
                end_eff_to_valve = math.sqrt((end_eff_x - valve_pos[0])**2 + (end_eff_y - valve_pos[1])**2)
                
                print(f"✓ 优化成功!")
                print(f"  选择间隙: {selected_gap}")
                print(f"  机体位置: [{body_x:.3f}, {body_y:.3f}]")
                print(f"  机体朝向: {body_yaw:.3f} rad ({body_yaw*180/math.pi:.1f}°)")
                print(f"  末端执行器位置: [{end_eff_x:.3f}, {end_eff_y:.3f}]")
                print(f"  机体到阀门距离: {body_to_valve*100:.1f}cm")
                print(f"  末端执行器到阀门距离: {end_eff_to_valve*100:.1f}cm")
                print(f"  约束满足: {result['constraints_satisfied']}")
                print(f"  目标函数值: {result['objective_value']:.4f}")
                
                # Check body clearance
                valve_radius_cm = optimizer.valve_radius * 100
                body_clearance_cm = body_to_valve * 100
                if body_clearance_cm > valve_radius_cm + 5:
                    print(f"  ✓ 机体离开阀门本体: {body_clearance_cm:.1f}cm > {valve_radius_cm:.1f}cm + 5cm")
                else:
                    print(f"  ✗ 机体太靠近阀门: {body_clearance_cm:.1f}cm")
                    
            else:
                print(f"✗ 优化失败: {result['message'] if result else 'No result'}")
                
        except Exception as e:
            print(f"✗ 优化出错: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    test_updated_optimizer()
