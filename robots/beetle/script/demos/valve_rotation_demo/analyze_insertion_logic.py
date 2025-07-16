#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from insertion_optimizer import InsertionOptimizer
import math

def analyze_insertion_logic():
    """分析当前插入逻辑的详细参数计算"""
    print("=== 当前插入逻辑分析 ===")
    
    # 创建优化器
    optimizer = InsertionOptimizer(
        valve_radius=0.0525,  # 从日志中获取的实际值
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
    print(f"UAV偏航: {current_yaw:.3f} rad ({current_yaw*180/math.pi:.1f}°)")
    print(f"阀门位置: {valve_pos}")
    print(f"阀门偏航: {valve_yaw:.3f} rad ({valve_yaw*180/math.pi:.1f}°)")
    
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
    
    print(f"\n=== 双爪策略分析 ===")
    print(f"主爪: {params['primary_fang']}")
    print(f"副爪: {params['secondary_fang']}")
    print(f"插入模式: {params['insertion_mode']}")
    
    # 分析主爪参数
    primary_fang = params['primary_fang']
    primary_params = params[primary_fang]
    
    print(f"\n=== 主爪 ({primary_fang}) 参数分析 ===")
    print(f"配置: {primary_params['config']['name']}")
    print(f"接近位置: {primary_params['approach_position']}")
    print(f"最终位置: {primary_params['final_position']}")
    print(f"阀门中心: {primary_params['valve_center']}")
    print(f"接近半径: {primary_params['approach_radius']:.4f}m")
    print(f"最终半径: {primary_params['final_radius']:.4f}m")
    print(f"接近角度: {primary_params['approach_angle']:.3f} rad ({primary_params['approach_angle']*180/math.pi:.1f}°)")
    print(f"最终角度: {primary_params['final_angle']:.3f} rad ({primary_params['final_angle']*180/math.pi:.1f}°)")
    print(f"目标偏航: {primary_params['target_yaw']:.3f} rad ({primary_params['target_yaw']*180/math.pi:.1f}°)")
    
    # 计算距离差异
    approach_pos = primary_params['approach_position']
    final_pos = primary_params['final_position']
    
    distance_diff = math.sqrt((final_pos[0] - approach_pos[0])**2 + (final_pos[1] - approach_pos[1])**2)
    print(f"\n接近位置与最终位置的距离差: {distance_diff:.4f}m")
    
    # 分析为什么距离差这么小
    print(f"\n=== 距离分析 ===")
    print(f"接近半径: {primary_params['approach_radius']:.4f}m")
    print(f"最终半径: {primary_params['final_radius']:.4f}m")
    print(f"半径差: {primary_params['approach_radius'] - primary_params['final_radius']:.4f}m")
    print(f"角度差: {primary_params['approach_angle'] - primary_params['final_angle']:.3f} rad")
    
    # 分析副爪参数
    secondary_fang = params['secondary_fang']
    secondary_params = params[secondary_fang]
    
    print(f"\n=== 副爪 ({secondary_fang}) 参数分析 ===")
    print(f"配置: {secondary_params['config']['name']}")
    print(f"接近位置: {secondary_params['approach_position']}")
    print(f"最终位置: {secondary_params['final_position']}")
    print(f"复杂度评分: {secondary_params['complexity_score']:.3f}")
    
    print(f"\n=== 当前插入逻辑总结 ===")
    print("1. 系统计算了双爪策略，但只使用主爪参数")
    print("2. 接近位置和最终位置距离很近，可能是参数计算的问题")
    print("3. 系统按以下顺序执行:")
    print("   a) 移动到主爪接近位置")
    print("   b) 调整到主爪最终位置")
    print("   c) 下降到接触阀门")
    print("4. 整个过程中只使用了一个爪子的参数，没有真正实现双爪插入")

if __name__ == "__main__":
    analyze_insertion_logic()
