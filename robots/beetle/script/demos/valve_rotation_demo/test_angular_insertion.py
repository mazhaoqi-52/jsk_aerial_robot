#!/usr/bin/env python3
"""
Test the angular insertion approach to avoid valve handle plane blockage
"""

import math

def test_angular_insertion_approach():
    """Test the angular insertion calculations to avoid handle plane blockage"""
    
    print("=" * 80)
    print("角度插入方法测试 - 避开阀门把手平面阻挡")
    print("=" * 80)
    
    # Parameters from the log
    valve_center = [3.000, 0.0, 0.570]  # 阀门中心
    current_pos = [2.857, -0.098, 0.683]  # 当前位置 (插入失败时的位置)
    end_effector_blocked = [3.052, 0.052, 0.755]  # 被阻挡的末端执行器位置
    
    # New angular insertion parameters
    angular_insertion_enabled = True
    insertion_angle_offset = 0.15  # 15cm offset for angular approach
    handle_clearance_height = 0.08  # 8cm above valve center
    z_offset = -0.03  # Adjusted z_offset
    
    print(f"输入参数:")
    print(f"  阀门中心: {valve_center}")
    print(f"  当前UAV位置: {current_pos}")
    print(f"  被阻挡的末端执行器位置: {end_effector_blocked}")
    print(f"  角度插入偏移: {insertion_angle_offset:.3f}m")
    print(f"  把手间隙高度: {handle_clearance_height:.3f}m")
    
    # Calculate current position relative to valve
    valve_center_x, valve_center_y, valve_center_z = valve_center
    start_x, start_y, start_z = current_pos
    
    current_angle = math.atan2(start_y - valve_center_y, start_x - valve_center_x)
    current_radius = math.sqrt((start_x - valve_center_x)**2 + (start_y - valve_center_y)**2)
    
    print(f"\n当前位置分析:")
    print(f"  相对角度: {current_angle:.3f} rad ({current_angle*180/math.pi:.1f}°)")
    print(f"  相对半径: {current_radius:.3f}m")
    print(f"  当前高度: {start_z:.3f}m")
    
    # Stage 1: Clearance height calculation
    clearance_height = valve_center_z + handle_clearance_height
    clearance_pos = [start_x, start_y, clearance_height]
    
    print(f"\n阶段1: 移动到把手间隙高度")
    print(f"  间隙高度: {clearance_height:.3f}m")
    print(f"  间隙位置: [{clearance_pos[0]:.3f}, {clearance_pos[1]:.3f}, {clearance_pos[2]:.3f}]")
    print(f"  垂直移动距离: {abs(clearance_height - start_z):.3f}m")
    
    # Stage 2: Angular approach calculation
    insertion_radius = current_radius - insertion_angle_offset
    if insertion_radius < 0.05:
        insertion_radius = 0.05
    
    angular_approach_x = valve_center_x + insertion_radius * math.cos(current_angle)
    angular_approach_y = valve_center_y + insertion_radius * math.sin(current_angle)
    angular_approach_pos = [angular_approach_x, angular_approach_y, clearance_height]
    
    print(f"\n阶段2: 角度接近插入点")
    print(f"  新半径: {insertion_radius:.3f}m")
    print(f"  角度接近位置: [{angular_approach_x:.3f}, {angular_approach_y:.3f}, {clearance_height:.3f}]")
    
    horizontal_movement = math.sqrt((angular_approach_x - start_x)**2 + (angular_approach_y - start_y)**2)
    print(f"  水平移动距离: {horizontal_movement:.3f}m")
    
    # Stage 3: Final target calculation
    final_target_z = valve_center_z + z_offset
    final_target_pos = [angular_approach_x, angular_approach_y, final_target_z]
    
    print(f"\n阶段3: 角度下降到最终插入位置")
    print(f"  最终目标: [{final_target_pos[0]:.3f}, {final_target_pos[1]:.3f}, {final_target_pos[2]:.3f}]")
    print(f"  下降距离: {clearance_height - final_target_z:.3f}m")
    print(f"  同时水平调整: 0.000m (保持角度接近位置)")
    
    # Calculate end-effector position after angular insertion
    # Assuming end-effector is 24.6cm forward from body center
    end_effector_length = 0.246
    # Assume UAV is pointing toward valve center
    body_to_valve_angle = math.atan2(valve_center_y - final_target_pos[1], 
                                    valve_center_x - final_target_pos[0])
    
    final_end_effector_x = final_target_pos[0] + end_effector_length * math.cos(body_to_valve_angle)
    final_end_effector_y = final_target_pos[1] + end_effector_length * math.sin(body_to_valve_angle)
    final_end_effector_z = final_target_pos[2] + 0.074  # Height offset
    
    print(f"\n预期末端执行器位置:")
    print(f"  位置: [{final_end_effector_x:.3f}, {final_end_effector_y:.3f}, {final_end_effector_z:.3f}]")
    print(f"  相对阀门高度: {final_end_effector_z - valve_center_z:.3f}m")
    
    # Compare with blocked position
    print(f"\n对比分析:")
    print(f"  原始被阻挡位置: {end_effector_blocked}")
    print(f"  原始相对高度: {end_effector_blocked[2] - valve_center_z:.3f}m (太高)")
    print(f"  新的相对高度: {final_end_effector_z - valve_center_z:.3f}m")
    
    height_improvement = end_effector_blocked[2] - final_end_effector_z
    print(f"  高度改进: {height_improvement:.3f}m (下降)")
    
    # Check if new position avoids handle plane
    handle_plane_height = valve_center_z + 0.05  # Assume handle plane is ~5cm above valve center
    if final_end_effector_z > handle_plane_height:
        print(f"  ⚠ 末端执行器仍可能在把手平面上方")
    else:
        print(f"  ✓ 末端执行器应该在把手平面下方，可以进入阀门内部")
    
    # Check angular approach benefits
    print(f"\n角度接近的优势:")
    print(f"  1. 避开正面撞击把手平面")
    print(f"  2. 从侧面角度进入阀门")
    print(f"  3. 分阶段移动降低碰撞风险")
    print(f"  4. 更好的间隙控制")
    
    # Safety checks
    print(f"\n安全检查:")
    if insertion_radius < 0.1:
        print(f"  ⚠ 插入半径较小 ({insertion_radius:.3f}m)，需要谨慎操作")
    else:
        print(f"  ✓ 插入半径合理 ({insertion_radius:.3f}m)")
    
    if horizontal_movement > 0.2:
        print(f"  ⚠ 水平移动距离较大 ({horizontal_movement:.3f}m)")
    else:
        print(f"  ✓ 水平移动距离合理 ({horizontal_movement:.3f}m)")
    
    print(f"\n" + "=" * 80)
    print("角度插入方法应该能够解决把手平面阻挡问题")
    print("建议测试此方法来避开阀门把手的直接碰撞")
    print("=" * 80)

if __name__ == "__main__":
    test_angular_insertion_approach()
