#!/usr/bin/env python3
"""
Calculate the correct UAV positioning for "胸部离开阀门本体手抓住阀门"
"""
import math

def calculate_correct_positioning():
    """Calculate the geometrically correct UAV positioning"""
    
    print("=== 计算正确的UAV定位 ===")
    
    # 已知参数
    valve_radius = 0.1225  # 12.25cm - 阀门半径
    end_effector_length = 0.246  # 24.6cm - 末端执行器长度
    safety_margin = 0.05  # 5cm - 安全间隙
    
    print(f"阀门半径: {valve_radius*100:.1f}cm")
    print(f"末端执行器长度: {end_effector_length*100:.1f}cm") 
    print(f"安全间隙: {safety_margin*100:.1f}cm")
    
    # 目标：末端执行器在阀门半径上，机体尽可能远离阀门
    # 使用几何关系计算
    
    valve_center = (3.0, 0.0)
    
    # 假设机体在阀门左侧，末端执行器指向阀门
    # 末端执行器目标位置（在阀门右侧边缘）
    end_eff_target_x = valve_center[0] + valve_radius  # 阀门右边缘
    end_eff_target_y = valve_center[1]
    
    # 机体位置：从末端执行器位置向后延伸24.6cm
    body_yaw = math.pi  # 朝向阀门中心（向右）
    body_x = end_eff_target_x - end_effector_length * math.cos(body_yaw)
    body_y = end_eff_target_y - end_effector_length * math.sin(body_yaw)
    
    # 验证计算
    actual_end_eff_x = body_x + end_effector_length * math.cos(body_yaw)
    actual_end_eff_y = body_y + end_effector_length * math.sin(body_yaw)
    
    # 计算距离
    body_to_valve = math.sqrt((body_x - valve_center[0])**2 + (body_y - valve_center[1])**2)
    end_eff_to_valve = math.sqrt((actual_end_eff_x - valve_center[0])**2 + (actual_end_eff_y - valve_center[1])**2)
    
    print(f"\n=== 计算结果 ===")
    print(f"阀门中心: [{valve_center[0]:.3f}, {valve_center[1]:.3f}]")
    print(f"机体位置: [{body_x:.3f}, {body_y:.3f}]")
    print(f"机体朝向: {body_yaw:.3f} rad ({body_yaw*180/math.pi:.1f}°)")
    print(f"末端执行器位置: [{actual_end_eff_x:.3f}, {actual_end_eff_y:.3f}]")
    print(f"机体到阀门距离: {body_to_valve*100:.1f}cm")
    print(f"末端执行器到阀门距离: {end_eff_to_valve*100:.1f}cm (应该≈{valve_radius*100:.1f}cm)")
    
    # 检查是否满足"胸部离开阀门本体"
    body_clear_of_valve = body_to_valve > (valve_radius + safety_margin)
    end_eff_at_valve = abs(end_eff_to_valve - valve_radius) < 0.01
    
    print(f"\n=== 验证结果 ===")
    print(f"✓ 末端执行器在阀门半径上: {'是' if end_eff_at_valve else '否'}")
    print(f"✓ 机体离开阀门本体: {'是' if body_clear_of_valve else '否'}")
    
    if body_clear_of_valve and end_eff_at_valve:
        print(f"🎯 配置正确：胸部离开阀门本体({body_to_valve*100:.1f}cm) + 手抓住阀门({end_eff_to_valve*100:.1f}cm)")
    else:
        print(f"❌ 配置需要调整")
    
    # 计算其他可能的配置（不同角度）
    print(f"\n=== 其他可能的配置 ===")
    for angle_deg in [90, 180, 270]:
        angle_rad = angle_deg * math.pi / 180
        
        # 末端执行器在阀门圆周上的不同位置
        end_eff_x = valve_center[0] + valve_radius * math.cos(angle_rad)
        end_eff_y = valve_center[1] + valve_radius * math.sin(angle_rad)
        
        # 机体朝向阀门中心
        body_yaw = math.atan2(valve_center[1] - end_eff_y, valve_center[0] - end_eff_x)
        
        # 机体位置
        body_x = end_eff_x - end_effector_length * math.cos(body_yaw)
        body_y = end_eff_y - end_effector_length * math.sin(body_yaw)
        
        body_dist = math.sqrt((body_x - valve_center[0])**2 + (body_y - valve_center[1])**2)
        
        print(f"  {angle_deg:3d}°: 机体[{body_x:.3f}, {body_y:.3f}], 距离{body_dist*100:.1f}cm")
    
    return {
        'body_position': (body_x, body_y),
        'body_yaw': body_yaw,
        'end_effector_position': (actual_end_eff_x, actual_end_eff_y),
        'body_to_valve_distance': body_to_valve,
        'configuration_valid': body_clear_of_valve and end_eff_at_valve
    }

if __name__ == "__main__":
    result = calculate_correct_positioning()
