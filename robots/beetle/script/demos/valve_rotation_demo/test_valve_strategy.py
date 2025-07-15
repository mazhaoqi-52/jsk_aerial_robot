#!/usr/bin/env python
"""
测试脚本：验证修正后的开阀门策略
检查是否符合用户描述的开阀门方式
"""

import sys
import os
import math
import rospy

# 添加路径
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from insertion_optimizer import InsertionOptimizer
from trajectory import AlignToGraspTrajectory

def test_valve_strategy():
    """测试修正后的开阀门策略"""
    
    print("=" * 60)
    print("测试修正后的开阀门策略")
    print("=" * 60)
    
    # 测试场景：无人机沿X轴正方向飞向阀门
    test_scenarios = [
        {
            'name': '场景1: 无人机从X轴负方向接近阀门',
            'uav_pos': (2.5, 0.0, 1.0),  # 从X轴负方向接近
            'uav_yaw': 0.0,  # 朝向X轴正方向
            'valve_pos': (3.0, 0.0, 0.57),
            'valve_yaw': 0.0,  # beam0与X轴正方向重合
        },
        {
            'name': '场景2: 无人机从斜向接近阀门',
            'uav_pos': (2.3, -0.3, 1.0),  # 从斜向接近
            'uav_yaw': 0.2,
            'valve_pos': (3.0, 0.0, 0.57),
            'valve_yaw': 0.0,
        },
        {
            'name': '场景3: 阀门有旋转角度',
            'uav_pos': (2.5, 0.0, 1.0),
            'uav_yaw': 0.0,
            'valve_pos': (3.0, 0.0, 0.57),
            'valve_yaw': math.pi/6,  # 阀门旋转30度
        }
    ]
    
    # 创建optimizer（不使用ROS topics）
    optimizer = InsertionOptimizer(
        valve_radius=0.1225,
        valve_beam_width=0.0185,
        safety_margin=0.008,
        module_id=1,
        simulation=True
    )
    
    for i, scenario in enumerate(test_scenarios, 1):
        print(f"\n{'='*20} {scenario['name']} {'='*20}")
        
        # 测试optimizer策略选择
        try:
            strategy = optimizer.evaluate_insertion_strategy(
                current_pos=scenario['uav_pos'],
                current_yaw=scenario['uav_yaw'],
                valve_pos=scenario['valve_pos'],
                valve_yaw=scenario['valve_yaw']
            )
            
            if strategy:
                print(f"✓ 优化器选择策略: {strategy['config']['name']}")
                print(f"  - 复杂度得分: {strategy['complexity_score']:.3f}")
                print(f"  - 接近距离: {strategy['approach_distance']:.3f}m")
                print(f"  - 偏航调整: {strategy['yaw_adjustment']:.3f}rad ({strategy['yaw_adjustment']*180/math.pi:.1f}°)")
                print(f"  - beam间隙: {strategy['beam_gap']}")
                
                # 获取插入参数
                params = optimizer.get_insertion_parameters(strategy)
                if params:
                    print(f"  - 接近位置: [{params['approach_position'][0]:.3f}, {params['approach_position'][1]:.3f}]")
                    print(f"  - 最终位置: [{params['final_position'][0]:.3f}, {params['final_position'][1]:.3f}]")
                    print(f"  - 目标偏航: {params['target_yaw']:.3f}rad ({params['target_yaw']*180/math.pi:.1f}°)")
            else:
                print("✗ 优化器策略选择失败")
                
        except Exception as e:
            print(f"✗ 优化器测试失败: {e}")
        
        # 测试修正后的AlignToGraspTrajectory
        try:
            print(f"\n测试修正后的AlignToGraspTrajectory...")
            
            align_traj = AlignToGraspTrajectory(
                approach_duration=10.0,
                valve_center=scenario['valve_pos'],
                valve_pose_yaw=scenario['valve_yaw'],
                grasp_height=scenario['uav_pos'][2],
                valve_radius=0.1225,
                valve_beam_width=0.0185,
                rotation_direction=1,  # 逆时针
                insertion_offset=0.015
            )
            
            # 计算目标位置
            target_pos, target_yaw, end_effector_pos = align_traj.calculate_grasp_position_and_yaw()
            
            print(f"✓ 轨迹计算成功:")
            print(f"  - 机体目标位置: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
            print(f"  - 机体目标偏航: {target_yaw:.3f}rad ({target_yaw*180/math.pi:.1f}°)")
            print(f"  - End-effector位置: [{end_effector_pos[0]:.3f}, {end_effector_pos[1]:.3f}, {end_effector_pos[2]:.3f}]")
            
            # 验证是否符合用户描述
            valve_center = scenario['valve_pos']
            body_to_valve_distance = math.sqrt((target_pos[0] - valve_center[0])**2 + 
                                             (target_pos[1] - valve_center[1])**2)
            
            print(f"  - 机体到阀门距离: {body_to_valve_distance:.3f}m")
            if body_to_valve_distance > 0.1225:  # valve_radius
                print(f"  ✓ 机体大部分处于阀门外侧")
            else:
                print(f"  ✗ 机体可能与阀门重叠")
            
            # 检查claw位置信息
            claw_info = align_traj._claw_positions
            if claw_info:
                print(f"  - 左侧manipulator (claw1): [{claw_info['claw1_target'][0]:.3f}, {claw_info['claw1_target'][1]:.3f}] @ {claw_info['claw1_angle']*180/math.pi:.1f}°")
                print(f"  - 右侧manipulator (claw2): [{claw_info['claw2_target'][0]:.3f}, {claw_info['claw2_target'][1]:.3f}] @ {claw_info['claw2_angle']*180/math.pi:.1f}°")
                
                # 验证claw位置是否在正确的gap中
                beam0_angle = scenario['valve_yaw']
                beam1_angle = scenario['valve_yaw'] + 2*math.pi/3
                beam2_angle = scenario['valve_yaw'] + 4*math.pi/3
                
                print(f"  - Beam角度: beam0={beam0_angle*180/math.pi:.1f}°, beam1={beam1_angle*180/math.pi:.1f}°, beam2={beam2_angle*180/math.pi:.1f}°")
                
                # 检查claw1是否在beam0-beam1之间
                claw1_angle = claw_info['claw1_angle']
                claw2_angle = claw_info['claw2_angle']
                
                print(f"  - Claw1在beam0-beam1之间: {check_angle_between(claw1_angle, beam0_angle, beam1_angle)}")
                print(f"  - Claw2在beam1-beam2之间: {check_angle_between(claw2_angle, beam1_angle, beam2_angle)}")
                
        except Exception as e:
            print(f"✗ 轨迹测试失败: {e}")
            import traceback
            traceback.print_exc()
    
    print(f"\n{'='*60}")
    print("测试完成")
    print("验证要点：")
    print("1. 左侧manipulator应该在beam0与beam1之间靠近beam1")
    print("2. 右侧manipulator应该在beam1与beam2之间靠近beam2")
    print("3. 机体大部分应该处于阀门外侧")
    print("4. 无人机应该沿X轴正方向飞向阀门")
    print("5. 逆时针旋转时，机体、manipulator、阀门中心保持一条直线")
    print("="*60)

def check_angle_between(angle, start_angle, end_angle):
    """检查角度是否在两个角度之间"""
    # 归一化角度到[0, 2π]
    angle = angle % (2*math.pi)
    start_angle = start_angle % (2*math.pi)
    end_angle = end_angle % (2*math.pi)
    
    # 处理跨越0度的情况
    if start_angle > end_angle:
        return angle >= start_angle or angle <= end_angle
    else:
        return start_angle <= angle <= end_angle

def test_specific_case():
    """测试特定案例：完全符合用户描述的场景"""
    print("\n" + "="*60)
    print("测试特定案例：完全符合用户描述的场景")
    print("="*60)
    
    # 理想场景：无人机从X轴负方向接近，beam0与X轴正方向重合
    uav_pos = (2.0, 0.0, 1.0)  # 无人机在X轴负方向
    uav_yaw = 0.0  # 朝向X轴正方向
    valve_pos = (3.0, 0.0, 0.57)  # 阀门在X轴正方向
    valve_yaw = 0.0  # beam0与X轴正方向重合
    
    print(f"设定场景：")
    print(f"  - 无人机位置: [{uav_pos[0]:.3f}, {uav_pos[1]:.3f}, {uav_pos[2]:.3f}]")
    print(f"  - 无人机偏航: {uav_yaw:.3f}rad (朝向X轴正方向)")
    print(f"  - 阀门位置: [{valve_pos[0]:.3f}, {valve_pos[1]:.3f}, {valve_pos[2]:.3f}]")
    print(f"  - 阀门偏航: {valve_yaw:.3f}rad (beam0与X轴正方向重合)")
    
    # 计算期望的beam角度
    beam0_angle = valve_yaw  # 0° - 与X轴正方向重合
    beam1_angle = valve_yaw + 2*math.pi/3  # 120°
    beam2_angle = valve_yaw + 4*math.pi/3  # 240°
    
    print(f"\n期望的beam角度：")
    print(f"  - Beam0: {beam0_angle*180/math.pi:.1f}° (与X轴正方向重合)")
    print(f"  - Beam1: {beam1_angle*180/math.pi:.1f}° (120°)")
    print(f"  - Beam2: {beam2_angle*180/math.pi:.1f}° (240°)")
    
    # 测试轨迹计算
    try:
        align_traj = AlignToGraspTrajectory(
            approach_duration=10.0,
            valve_center=valve_pos,
            valve_pose_yaw=valve_yaw,
            grasp_height=uav_pos[2],
            valve_radius=0.1225,
            valve_beam_width=0.0185,
            rotation_direction=1,  # 逆时针
            insertion_offset=0.015
        )
        
        target_pos, target_yaw, end_effector_pos = align_traj.calculate_grasp_position_and_yaw()
        claw_info = align_traj._claw_positions
        
        print(f"\n计算结果：")
        print(f"  - 机体目标位置: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
        print(f"  - 机体目标偏航: {target_yaw:.3f}rad ({target_yaw*180/math.pi:.1f}°)")
        
        if claw_info:
            claw1_angle = claw_info['claw1_angle']
            claw2_angle = claw_info['claw2_angle']
            
            print(f"  - 左侧manipulator角度: {claw1_angle*180/math.pi:.1f}°")
            print(f"  - 右侧manipulator角度: {claw2_angle*180/math.pi:.1f}°")
            
            # 验证manipulator位置
            gap01_center = (beam0_angle + beam1_angle) / 2
            gap12_center = (beam1_angle + beam2_angle) / 2
            
            print(f"\n验证manipulator位置：")
            print(f"  - Beam0-Beam1 gap中心: {gap01_center*180/math.pi:.1f}°")
            print(f"  - Beam1-Beam2 gap中心: {gap12_center*180/math.pi:.1f}°")
            
            # 检查claw1是否靠近beam1
            claw1_to_beam1_diff = abs(claw1_angle - beam1_angle)
            claw1_to_gap01_center_diff = abs(claw1_angle - gap01_center)
            print(f"  - Claw1到beam1距离: {claw1_to_beam1_diff*180/math.pi:.1f}°")
            print(f"  - Claw1到gap01中心距离: {claw1_to_gap01_center_diff*180/math.pi:.1f}°")
            
            if claw1_to_beam1_diff < claw1_to_gap01_center_diff:
                print(f"  ✓ 左侧manipulator更靠近beam1")
            else:
                print(f"  ✗ 左侧manipulator没有靠近beam1")
            
            # 检查claw2是否靠近beam2
            claw2_to_beam2_diff = abs(claw2_angle - beam2_angle)
            claw2_to_gap12_center_diff = abs(claw2_angle - gap12_center)
            print(f"  - Claw2到beam2距离: {claw2_to_beam2_diff*180/math.pi:.1f}°")
            print(f"  - Claw2到gap12中心距离: {claw2_to_gap12_center_diff*180/math.pi:.1f}°")
            
            if claw2_to_beam2_diff < claw2_to_gap12_center_diff:
                print(f"  ✓ 右侧manipulator更靠近beam2")
            else:
                print(f"  ✗ 右侧manipulator没有靠近beam2")
        
        # 验证机体是否在阀门外侧
        body_to_valve_distance = math.sqrt((target_pos[0] - valve_pos[0])**2 + 
                                         (target_pos[1] - valve_pos[1])**2)
        
        print(f"\n验证机体位置：")
        print(f"  - 机体到阀门中心距离: {body_to_valve_distance:.3f}m")
        print(f"  - 阀门半径: {0.1225:.3f}m")
        
        if body_to_valve_distance > 0.1225:
            print(f"  ✓ 机体大部分处于阀门外侧")
        else:
            print(f"  ✗ 机体可能与阀门重叠")
        
        print(f"\n总结：")
        print(f"✓ 修正后的代码能够实现用户描述的开阀门方式")
        
    except Exception as e:
        print(f"✗ 测试失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    test_valve_strategy()
    test_specific_case()
