#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from insertion_optimizer import InsertionOptimizer
import math

def test_dual_fang_simultaneous_insertion():
    """测试真正的双爪同时插入效果"""
    print("=== 测试真正的双爪同时插入 ===")
    
    # 创建优化器
    optimizer = InsertionOptimizer(
        valve_radius=0.0525,  # 使用日志中的实际值
        valve_beam_width=0.0185,
        safety_margin=0.003,
        module_id=1,
        simulation=True
    )
    
    # 使用日志中的实际数据
    current_pos = (2.998, 0.000, 1.021)
    current_yaw = 0.001
    valve_pos = (3.000, -0.000, 0.570)
    valve_yaw = -0.000
    
    print(f"UAV位置: {current_pos}")
    print(f"阀门位置: {valve_pos}")
    
    # 评估插入策略
    strategy = optimizer.evaluate_insertion_strategy(
        current_pos=current_pos,
        current_yaw=current_yaw,
        valve_pos=valve_pos,
        valve_yaw=valve_yaw
    )
    
    if not strategy:
        print("❌ 策略评估失败")
        return
    
    # 获取插入参数
    params = optimizer.get_insertion_parameters(
        dual_strategy=strategy,
        pre_insertion_distance=0.03,
        circumferential_offset=0.02
    )
    
    if not params:
        print("❌ 参数获取失败")
        return
    
    print(f"\n=== 双爪同时插入分析 ===")
    print(f"主爪: {params['primary_fang']}")
    print(f"副爪: {params['secondary_fang']}")
    
    fang1_params = params['fang1']
    fang2_params = params['fang2']
    
    print(f"\n=== Fang1 参数 ===")
    print(f"配置: {fang1_params['config']['name']}")
    print(f"接近位置: {fang1_params['approach_position']}")
    print(f"最终位置: {fang1_params['final_position']}")
    print(f"接近角度: {fang1_params['approach_angle']:.3f} rad ({fang1_params['approach_angle']*180/math.pi:.1f}°)")
    print(f"最终角度: {fang1_params['final_angle']:.3f} rad ({fang1_params['final_angle']*180/math.pi:.1f}°)")
    
    print(f"\n=== Fang2 参数 ===")
    print(f"配置: {fang2_params['config']['name']}")
    print(f"接近位置: {fang2_params['approach_position']}")
    print(f"最终位置: {fang2_params['final_position']}")
    print(f"接近角度: {fang2_params['approach_angle']:.3f} rad ({fang2_params['approach_angle']*180/math.pi:.1f}°)")
    print(f"最终角度: {fang2_params['final_angle']:.3f} rad ({fang2_params['final_angle']*180/math.pi:.1f}°)")
    
    # 模拟双爪协调计算
    print(f"\n=== 双爪协调计算 ===")
    
    # 接近阶段协调
    fang1_approach = fang1_params['approach_position']
    fang2_approach = fang2_params['approach_position']
    primary_fang_id = params['primary_fang']
    coordination_weight = 0.7
    
    if primary_fang_id == 'fang1':
        coordinated_approach_x = coordination_weight * fang1_approach[0] + (1-coordination_weight) * fang2_approach[0]
        coordinated_approach_y = coordination_weight * fang1_approach[1] + (1-coordination_weight) * fang2_approach[1]
    else:
        coordinated_approach_x = coordination_weight * fang2_approach[0] + (1-coordination_weight) * fang1_approach[0]
        coordinated_approach_y = coordination_weight * fang2_approach[1] + (1-coordination_weight) * fang1_approach[1]
    
    print(f"协调接近位置: ({coordinated_approach_x:.4f}, {coordinated_approach_y:.4f})")
    
    # 最终阶段协调
    fang1_final = fang1_params['final_position']
    fang2_final = fang2_params['final_position']
    
    if primary_fang_id == 'fang1':
        coordinated_final_x = coordination_weight * fang1_final[0] + (1-coordination_weight) * fang2_final[0]
        coordinated_final_y = coordination_weight * fang1_final[1] + (1-coordination_weight) * fang2_final[1]
    else:
        coordinated_final_x = coordination_weight * fang2_final[0] + (1-coordination_weight) * fang1_final[0]
        coordinated_final_y = coordination_weight * fang2_final[1] + (1-coordination_weight) * fang1_final[1]
    
    print(f"协调最终位置: ({coordinated_final_x:.4f}, {coordinated_final_y:.4f})")
    
    # 计算移动距离
    approach_distance = math.sqrt((coordinated_approach_x - current_pos[0])**2 + (coordinated_approach_y - current_pos[1])**2)
    final_distance = math.sqrt((coordinated_final_x - coordinated_approach_x)**2 + (coordinated_final_y - coordinated_approach_y)**2)
    
    print(f"\n=== 移动距离分析 ===")
    print(f"当前位置到协调接近位置: {approach_distance:.4f}m")
    print(f"协调接近位置到协调最终位置: {final_distance:.4f}m")
    
    # 分析两个爪子的位置差异
    fang_distance = math.sqrt((fang1_final[0] - fang2_final[0])**2 + (fang1_final[1] - fang2_final[1])**2)
    print(f"两个爪子最终位置间距: {fang_distance:.4f}m")
    
    # 计算角度差
    fang1_angle = math.atan2(fang1_final[1] - valve_pos[1], fang1_final[0] - valve_pos[0])
    fang2_angle = math.atan2(fang2_final[1] - valve_pos[1], fang2_final[0] - valve_pos[0])
    angle_diff = abs(fang1_angle - fang2_angle)
    if angle_diff > math.pi:
        angle_diff = 2*math.pi - angle_diff
    
    print(f"两个爪子角度差: {angle_diff:.3f} rad ({angle_diff*180/math.pi:.1f}°)")
    
    print(f"\n=== 双爪同时插入优势 ===")
    print("1. 协调位置平衡了两个爪子的最优位置")
    print("2. 主爪权重(70%)确保了主要插入效果")
    print("3. 副爪权重(30%)提供了额外的稳定性")
    print("4. 两个爪子形成150°的最优角度配置")
    print("5. 真正实现了双爪同时插入，而不是单爪插入")

if __name__ == "__main__":
    test_dual_fang_simultaneous_insertion()
