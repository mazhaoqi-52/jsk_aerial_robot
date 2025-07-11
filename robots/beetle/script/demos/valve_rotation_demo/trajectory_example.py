#!/usr/bin/env python
"""
阀门操作轨迹生成器使用示例
演示如何使用更新后的类来计算机体位置和姿态
"""

import rospy
import math
from trajectory import ValveGraspPlanner, CircularTrajectoryGenerator, PolynomialTrajectory
from trajectory import calculate_body_pose_from_end_effector, normalize_angle

def example_usage():
    """演示轨迹生成器的使用方法"""
    
    # 初始化ROS节点
    rospy.init_node('trajectory_example')
    
    # 定义末端执行器相对于机体重心的偏移
    end_effector_offset_x = 0.15  # 末端执行器在机体前方15cm
    end_effector_offset_y = 0.0   # Y方向无偏移
    end_effector_offset_z = -0.05 # 末端执行器在机体重心下方5cm
    
    # 创建阀门抓取规划器
    planner = ValveGraspPlanner(
        valve_radius=0.1225,
        beam_width=0.05,
        z_offset=0.21,
        end_effector_offset_x=end_effector_offset_x,
        end_effector_offset_y=end_effector_offset_y,
        end_effector_offset_z=end_effector_offset_z
    )
    
    # 示例阀门位置和朝向
    valve_center = (1.0, 0.5, 0.8)  # 阀门中心位置
    valve_yaw = math.pi / 4  # 阀门朝向45度
    
    print("=== 阀门抓取位置计算 ===")
    
    # 计算抓取位置和姿态
    body_pos, body_yaw, end_effector_pos = planner.calculate_grasp_position_and_yaw(
        valve_center, valve_yaw
    )
    
    print(f"阀门中心: {valve_center}")
    print(f"阀门yaw: {math.degrees(valve_yaw):.1f}°")
    print(f"机体目标位置: {body_pos}")
    print(f"机体目标yaw: {math.degrees(body_yaw):.1f}°")
    print(f"末端执行器位置: {end_effector_pos}")
    
    print("\n=== 旋转轨迹生成 ===")
    
    # 计算旋转参数
    dx = end_effector_pos[0] - valve_center[0]
    dy = end_effector_pos[1] - valve_center[1]
    radius = math.sqrt(dx**2 + dy**2)
    start_angle = math.atan2(dy, dx)
    
    print(f"旋转半径: {radius:.3f}m")
    print(f"起始角度: {math.degrees(start_angle):.1f}°")
    
    # 创建圆形轨迹生成器（旋转一周）
    circular_traj = CircularTrajectoryGenerator(
        center=(valve_center[0], valve_center[1]),
        radius=radius,
        start_angle=start_angle,
        total_angle=2*math.pi,  # 旋转一周
        duration=8.0,  # 8秒完成旋转
        end_effector_offset_x=end_effector_offset_x,
        end_effector_offset_y=end_effector_offset_y,
        end_effector_offset_z=end_effector_offset_z
    )
    
    # 开始轨迹
    circular_traj.start()
    
    print("\n=== 轨迹点示例 ===")
    
    # 生成几个轨迹点示例
    for i in range(5):
        # 模拟时间流逝
        rospy.sleep(0.1)
        
        result = circular_traj.evaluate(end_effector_pos[2])
        if result is not None:
            body_pos_current, body_yaw_current = result
            print(f"时刻 {i}: 机体位置 {body_pos_current}, yaw: {math.degrees(body_yaw_current):.1f}°")
        else:
            print("轨迹已完成")
            break
    
    print("\n=== 验证计算 ===")
    
    # 验证：从末端执行器位置反算机体位置
    test_end_effector_pos = (1.1, 0.7, 0.9)
    test_end_effector_yaw = math.pi / 3
    
    calculated_body_pos, calculated_body_yaw = calculate_body_pose_from_end_effector(
        test_end_effector_pos, test_end_effector_yaw,
        end_effector_offset_x, end_effector_offset_y, end_effector_offset_z
    )
    
    print(f"给定末端执行器位置: {test_end_effector_pos}")
    print(f"给定末端执行器yaw: {math.degrees(test_end_effector_yaw):.1f}°")
    print(f"计算得出机体位置: {calculated_body_pos}")
    print(f"计算得出机体yaw: {math.degrees(calculated_body_yaw):.1f}°")
    
    print("\n示例运行完成！")


if __name__ == '__main__':
    try:
        example_usage()
    except rospy.ROSInterruptException:
        print("示例被中断")
        pass