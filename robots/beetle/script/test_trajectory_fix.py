#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试轨迹修复验证 - 确保 evaluate_at_time() 正确返回边界值
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'demos/valve_rotation_demo'))

import rospy
from trajectory import PolynomialTrajectory

def test_trajectory_boundary_conditions():
    """测试轨迹边界条件是否正确"""
    print("=== 测试轨迹边界条件修复 ===")
    
    # 测试参数
    duration = 5.0  # 5秒轨迹
    start_pos = [1.0, 2.0, 3.0]
    target_pos = [4.0, 5.0, 6.0]
    
    # 创建轨迹
    traj = PolynomialTrajectory(duration)
    traj.generate_trajectory(start_pos, target_pos)
    
    print(f"轨迹时长: {duration}秒")
    print(f"起始位置: {start_pos}")
    print(f"目标位置: {target_pos}")
    print()
    
    # 测试边界条件
    print("--- 边界条件测试 ---")
    
    # 测试 t=0 时刻
    start_point = traj.evaluate_at_time(0.0)
    print(f"t=0.0时轨迹值: {start_point}")
    start_error = [abs(start_point[i] - start_pos[i]) for i in range(3)]
    print(f"起始位置误差: X={start_error[0]*1000:.3f}mm, Y={start_error[1]*1000:.3f}mm, Z={start_error[2]*1000:.3f}mm")
    
    # 测试 t=duration 时刻  
    end_point = traj.evaluate_at_time(duration)
    print(f"t={duration}时轨迹值: {end_point}")
    end_error = [abs(end_point[i] - target_pos[i]) for i in range(3)]
    print(f"目标位置误差: X={end_error[0]*1000:.3f}mm, Y={end_error[1]*1000:.3f}mm, Z={end_error[2]*1000:.3f}mm")
    print()
    
    # 测试中间点
    print("--- 中间点测试 ---")
    for t_ratio in [0.25, 0.5, 0.75]:
        t = t_ratio * duration
        point = traj.evaluate_at_time(t)
        print(f"t={t:.2f}s ({t_ratio*100:.0f}%): {[f'{p:.6f}' for p in point]}")
    print()
    
    # 验证结果
    max_start_error = max(start_error)
    max_end_error = max(end_error)
    
    tolerance = 1e-6  # 1微米容差
    
    if max_start_error <= tolerance and max_end_error <= tolerance:
        print("✅ 轨迹边界条件修复成功！")
        print(f"   最大起始误差: {max_start_error*1000000:.3f}μm")
        print(f"   最大目标误差: {max_end_error*1000000:.3f}μm")
        return True
    else:
        print("❌ 轨迹边界条件仍有问题")
        print(f"   最大起始误差: {max_start_error*1000:.3f}mm")
        print(f"   最大目标误差: {max_end_error*1000:.3f}mm")
        return False

def test_scalar_trajectory():
    """测试标量轨迹（角度等）"""
    print("\n=== 测试标量轨迹 ===")
    
    duration = 3.0
    start_angle = 0.0
    target_angle = 1.57  # π/2
    
    traj = PolynomialTrajectory(duration)
    traj.generate_trajectory(start_angle, target_angle)
    
    start_val = traj.evaluate_at_time(0.0)
    end_val = traj.evaluate_at_time(duration)
    
    print(f"起始角度: {start_angle:.6f}, 获得: {start_val:.6f}, 误差: {abs(start_val - start_angle)*1000:.3f}mrad")
    print(f"目标角度: {target_angle:.6f}, 获得: {end_val:.6f}, 误差: {abs(end_val - target_angle)*1000:.3f}mrad")
    
    start_error = abs(start_val - start_angle)
    end_error = abs(end_val - target_angle)
    
    if start_error <= 1e-6 and end_error <= 1e-6:
        print("✅ 标量轨迹边界条件正确")
        return True
    else:
        print("❌ 标量轨迹边界条件有问题")
        return False

if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node('trajectory_test', anonymous=True)
    
    success1 = test_trajectory_boundary_conditions()
    success2 = test_scalar_trajectory()
    
    if success1 and success2:
        print("\n🎉 所有轨迹测试通过！轨迹修复成功。")
        print("现在 evaluate_at_time() 方法应该正确返回边界值。")
    else:
        print("\n⚠️  仍有轨迹问题需要解决。")
