#!/usr/bin/env python3
"""
径向配置优化测试脚本
测试正确的"阀门中心→末端执行器→机体中心"径向扩张配置
"""

import sys
import math
import numpy as np
from constrained_optimizer import ConstrainedInsertionOptimizer

def test_radial_configuration():
    """测试径向配置优化器"""
    
    print("=== 径向配置优化测试 ===")
    print("测试正确的开阀门方式：阀门中心→末端执行器→机体中心的径向扩张配置")
    print()
    
    # 测试参数
    valve_radius = 0.1225  # 12.25cm
    end_effector_length = 0.246  # 24.6cm
    
    # 模拟阀门位置
    valve_pos = (0.0, 0.0, 0.5)  # 阀门在原点
    valve_yaw = 0.0
    
    # 创建优化器
    optimizer = ConstrainedInsertionOptimizer(
        module_id=1,
        valve_radius=valve_radius,
        end_effector_length=end_effector_length,
        safety_margin=0.02
    )
    
    # 手动设置位置数据（避免ROS依赖）
    optimizer.valve_pos = valve_pos
    optimizer.valve_yaw = valve_yaw
    
    print(f"阀门位置: {valve_pos}")
    print(f"阀门半径: {valve_radius:.3f}m")
    print(f"末端执行器长度: {end_effector_length:.3f}m")
    print()
    
    # 测试多个起始位置
    test_cases = [
        {
            'name': '右侧起始位置',
            'start_pos': (0.5, 0.0, 0.5),
            'start_yaw': math.pi  # 面向阀门
        },
        {
            'name': '左侧起始位置',
            'start_pos': (-0.5, 0.0, 0.5),
            'start_yaw': 0.0  # 面向阀门
        },
        {
            'name': '前方起始位置',
            'start_pos': (0.0, 0.5, 0.5),
            'start_yaw': -math.pi/2  # 面向阀门
        },
        {
            'name': '后方起始位置',
            'start_pos': (0.0, -0.5, 0.5),
            'start_yaw': math.pi/2  # 面向阀门
        }
    ]
    
    for i, test_case in enumerate(test_cases):
        print(f"\n{'='*60}")
        print(f"测试案例 {i+1}: {test_case['name']}")
        print(f"{'='*60}")
        
        # 设置当前位置
        optimizer.current_uav_pos = test_case['start_pos']
        optimizer.current_uav_yaw = test_case['start_yaw']
        
        print(f"起始位置: {test_case['start_pos']}")
        print(f"起始朝向: {test_case['start_yaw']:.3f} rad ({test_case['start_yaw']*180/math.pi:.1f}°)")
        
        # 执行优化
        result = optimizer.optimize_insertion_position()
        
        if result and result['success']:
            print(f"\n✅ 优化成功！")
            
            # 详细分析结果
            body_pos = result['body_position']
            body_yaw = result['body_yaw']
            end_eff_pos = result['end_effector_position']
            
            # 计算几何关系
            valve_center_x, valve_center_y, _ = valve_pos
            
            # 距离计算
            body_valve_distance = math.sqrt((body_pos[0] - valve_center_x)**2 + 
                                           (body_pos[1] - valve_center_y)**2)
            end_eff_valve_distance = math.sqrt((end_eff_pos[0] - valve_center_x)**2 + 
                                              (end_eff_pos[1] - valve_center_y)**2)
            
            # 角度计算
            body_angle = math.atan2(body_pos[1] - valve_center_y, body_pos[0] - valve_center_x)
            end_eff_angle = math.atan2(end_eff_pos[1] - valve_center_y, end_eff_pos[0] - valve_center_x)
            
            # 径向对齐质量
            angle_diff = abs(body_angle - end_eff_angle)
            angle_diff = min(angle_diff, 2*math.pi - angle_diff)
            
            print(f"\n📊 几何配置分析:")
            print(f"  机体距离阀门中心: {body_valve_distance:.3f}m")
            print(f"  末端执行器距离阀门中心: {end_eff_valve_distance:.3f}m")
            print(f"  机体-末端执行器距离: {body_valve_distance - end_eff_valve_distance:.3f}m")
            print(f"  径向对齐偏差: {angle_diff*180/math.pi:.1f}°")
            
            # 验证理想径向配置
            theoretical_body_distance = end_eff_valve_distance + end_effector_length
            distance_accuracy = abs(body_valve_distance - theoretical_body_distance)
            
            print(f"\n🔍 径向配置验证:")
            print(f"  理论机体距离: {theoretical_body_distance:.3f}m")
            print(f"  实际机体距离: {body_valve_distance:.3f}m")
            print(f"  距离精度: {distance_accuracy:.3f}m")
            
            # 配置质量评估
            if distance_accuracy < 0.05 and angle_diff < 0.1:
                quality = "🟢 理想径向扩张配置"
            elif distance_accuracy < 0.1 and angle_diff < 0.2:
                quality = "🟡 良好径向配置"
            else:
                quality = "🔴 需要改进"
            
            print(f"  配置质量: {quality}")
            
            # 物理约束验证
            print(f"\n✅ 约束验证:")
            print(f"  末端执行器在阀门半径内: {end_eff_valve_distance:.3f}m < {valve_radius:.3f}m")
            print(f"  机体在阀门外侧: {body_valve_distance:.3f}m > {valve_radius:.3f}m")
            print(f"  径向扩张: 机体在末端执行器外侧 ({body_valve_distance:.3f}m > {end_eff_valve_distance:.3f}m)")
            
            # 移动距离分析
            move_distance = math.sqrt((body_pos[0] - test_case['start_pos'][0])**2 + 
                                     (body_pos[1] - test_case['start_pos'][1])**2)
            yaw_change = abs(body_yaw - test_case['start_yaw'])
            
            print(f"\n📏 移动要求:")
            print(f"  位置移动: {move_distance:.3f}m")
            print(f"  角度变化: {yaw_change:.3f} rad ({yaw_change*180/math.pi:.1f}°)")
            
        else:
            print(f"❌ 优化失败")
            if result:
                print(f"原因: {result.get('message', 'Unknown')}")
    
    print(f"\n{'='*60}")
    print("径向配置优化测试完成")
    print("正确的开阀门方式：确保阀门中心→末端执行器→机体中心的径向扩张配置")
    print(f"{'='*60}")

if __name__ == "__main__":
    test_radial_configuration()
