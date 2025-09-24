#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
独立的阀门旋转测试脚本
专门用于调试阀门旋转阶段的俯仰静差问题
复用valve_rotation_fang_single.py中的现有类和函数

测试流程：
1. UAV悬停在指定位置
2. 直接创建阀门旋转轨迹
3. 执行旋转控制
4. 观察俯仰角变化和解耦控制激活情况

作者: GitHub Copilot
日期: 2025-09-21
"""

import rospy
import math
import sys
import os
import threading
import time
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from aerial_robot_msgs.msg import FlightNav
from tf.transformations import euler_from_quaternion

# 添加路径以导入本地模块
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# 导入现有的类和函数
from valve_rotation_fang_single import (
    SingleUAVStateBase, 
    RotateValveState,
    PIDController,
    calculate_unified_end_effector_position
)
from unified_motion_controller import UnifiedMotionController
from trajectory import create_constant_distance_trajectory, ConstantDistanceValveRotationTrajectory

class ValveRotationDebugTester(SingleUAVStateBase):
    """独立的阀门旋转调试测试器，继承自SingleUAVStateBase以复用功能"""
    
    def __init__(self):
        # 初始化父类
        super().__init__(
            outcomes=['succeeded', 'failed'],
            module_id=1
        )
        
        # 测试参数
        self.valve_center = (3.0, 0.0, 0.643)  # 阀门中心位置
        self.test_uav_position = (2.85, -0.25, 1.0)  # 测试UAV位置 - 悬停在1m高度避免物理干涉
        self.test_uav_yaw = math.radians(60)  # 60度
        self.rotation_angle = math.radians(90)  # 90度旋转
        self.rotation_duration = 30.0  # 30秒
        
        # 测试状态
        self.test_running = False
        self.start_time = None
        self.trajectory = None
        
        # 统计数据
        self.pitch_errors = []
        self.distance_errors = []
        self.max_pitch_error = 0.0
        self.decoupled_activations = 0
        self.total_control_points = 0
        
        rospy.loginfo("=== 阀门旋转悬停调试测试器初始化完成 ===")
        rospy.loginfo(f"⚠️  悬停测试模式 - UAV在1m高度进行旋转测试，避免物理干涉")
        rospy.loginfo(f"测试配置:")
        rospy.loginfo(f"  阀门中心: {self.valve_center}")
        rospy.loginfo(f"  UAV悬停位置: {self.test_uav_position} (1m高度)")
        rospy.loginfo(f"  UAV测试朝向: {math.degrees(self.test_uav_yaw):.1f}°")
        rospy.loginfo(f"  旋转角度: {math.degrees(self.rotation_angle):.1f}°")
        rospy.loginfo(f"  旋转时长: {self.rotation_duration}s")
        rospy.loginfo(f"🎯 目标: 在无物理干涉下测试去耦合控制是否激活")
    
    def execute(self, userdata):
        """SMACH状态执行函数（为了复用父类功能）"""
        try:
            success = self.run_test()
            return 'succeeded' if success else 'failed'
        except Exception as e:
            rospy.logerr(f"测试执行异常: {e}")
            return 'failed'
    
    def move_to_test_position(self):
        """将UAV移动到测试位置，复用父类的移动功能"""
        rospy.loginfo("=== 步骤1: 移动到测试位置 ===")
        
        if not self.wait_for_positions():
            return False
        
        # 显示当前位置
        current_pos = self.get_current_position()
        current_yaw = self.get_current_yaw()
        current_pitch = self.get_current_pitch()
        
        rospy.loginfo(f"当前位置: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"当前姿态: pitch={math.degrees(current_pitch):.1f}°, "
                     f"yaw={math.degrees(current_yaw):.1f}°")
        
        # 移动到测试位置
        target_pos = self.test_uav_position
        target_yaw = self.test_uav_yaw
        
        rospy.loginfo(f"目标位置: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
        rospy.loginfo(f"目标朝向: {math.degrees(target_yaw):.1f}°")
        
        # 使用父类的运动控制器移动
        start_time = rospy.Time.now().to_sec()
        timeout = 20.0
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            current_pos = self.get_current_position()
            current_yaw = self.get_current_yaw()
            
            # 检查是否到达目标
            pos_error = math.sqrt(
                (current_pos[0] - target_pos[0])**2 + 
                (current_pos[1] - target_pos[1])**2 + 
                (current_pos[2] - target_pos[2])**2
            )
            yaw_error = abs(self.normalize_angle(current_yaw - target_yaw))
            
            if pos_error < 0.05 and yaw_error < math.radians(5):
                rospy.loginfo("✓ 到达测试位置")
                break
                
            if current_time - start_time > timeout:
                rospy.logwarn("⚠️ 移动到测试位置超时，继续测试")
                break
            
            # 发送移动指令（复用父类的运动控制器）
            self.motion_controller.send_trajectory_point(
                target_pos, 
                target_yaw,
                velocity=None,
                use_decoupled_control=False
            )
            
            rospy.sleep(0.05)
        
        return True
    
    def create_rotation_trajectory_using_existing_methods(self):
        """使用现有方法创建阀门旋转轨迹"""
        rospy.loginfo("=== 步骤2: 创建旋转轨迹（复用现有方法）===")
        
        # 获取当前位置作为起始点
        current_uav_pos = self.get_current_position()
        current_uav_yaw = self.get_current_yaw()
        
        rospy.loginfo(f"轨迹起始位置: ({current_uav_pos[0]:.3f}, {current_uav_pos[1]:.3f}, {current_uav_pos[2]:.3f})")
        rospy.loginfo(f"轨迹起始朝向: {math.degrees(current_uav_yaw):.1f}°")
        
        # 使用现有的端效器位置计算函数
        end_effector_pos = calculate_unified_end_effector_position(
            current_uav_pos, current_uav_yaw, include_z=True
        )
        
        rospy.loginfo(f"端效器位置: ({end_effector_pos[0]:.3f}, {end_effector_pos[1]:.3f}, {end_effector_pos[2]:.3f})")
        
        # 计算端效器到阀门中心的距离
        valve_x, valve_y, valve_z = self.valve_center
        end_effector_distance = math.sqrt(
            (end_effector_pos[0] - valve_x)**2 + 
            (end_effector_pos[1] - valve_y)**2
        )
        
        rospy.loginfo(f"端效器到阀门距离: {end_effector_distance*1000:.1f}mm")
        
        # 悬停测试 - 将阀门中心高度调整到与UAV相同的1m高度，避免物理干涉
        hovering_valve_center = (self.valve_center[0], self.valve_center[1], 1.0)
        rospy.loginfo(f"悬停测试阀门中心: {hovering_valve_center} (避免物理干涉)")
        
        # 创建轨迹 - 首先尝试使用create_constant_distance_trajectory
        try:
            rospy.loginfo("尝试方法1: create_constant_distance_trajectory")
            self.trajectory = create_constant_distance_trajectory(
                current_uav_pos, 
                current_uav_yaw, 
                hovering_valve_center,  # 使用悬停高度的阀门中心
                self.rotation_angle, 
                self.rotation_duration,
                enable_adaptive_control=True,
                control_mode='rotation'
            )
            
            if self.trajectory is not None:
                rospy.loginfo("✓ 方法1成功")
            else:
                raise Exception("create_constant_distance_trajectory返回None")
                
        except Exception as e:
            rospy.logwarn(f"方法1失败: {e}")
            rospy.loginfo("尝试方法2: 直接创建ConstantDistanceValveRotationTrajectory")
            
            try:
                # 方法2: 直接创建ConstantDistanceValveRotationTrajectory
                start_angle = math.atan2(
                    end_effector_pos[1] - hovering_valve_center[1], 
                    end_effector_pos[0] - hovering_valve_center[0]
                )
                
                self.trajectory = ConstantDistanceValveRotationTrajectory(
                    valve_center=hovering_valve_center,  # 使用悬停高度的阀门中心
                    end_effector_distance=end_effector_distance,
                    rotation_angle=self.rotation_angle,
                    rotation_duration=self.rotation_duration,
                    start_angle=start_angle,
                    rotation_direction=1,
                    grasp_height=current_uav_pos[2]  # 保持在1m悬停高度
                )
                
                rospy.loginfo("✓ 方法2成功")
                
            except Exception as e2:
                rospy.logerr(f"方法2也失败: {e2}")
                return False
        
        # 检查轨迹的get_velocity方法
        has_get_velocity = hasattr(self.trajectory, 'get_velocity')
        trajectory_type = type(self.trajectory).__name__
        
        rospy.loginfo(f"轨迹类型: {trajectory_type}")
        rospy.loginfo(f"是否有get_velocity方法: {has_get_velocity}")
        
        if has_get_velocity:
            rospy.loginfo("✓ 解耦控制应该可以激活")
        else:
            rospy.logwarn("⚠️ 没有get_velocity方法，解耦控制无法激活")
        
        # 启动轨迹
        if hasattr(self.trajectory, 'start_trajectory'):
            self.trajectory.start_trajectory()
            rospy.loginfo("✓ 轨迹已启动")
        else:
            rospy.logwarn("⚠️ 轨迹没有start_trajectory方法")
        
        return True
    
    def execute_rotation_test_with_existing_controller(self):
        """使用现有控制器执行旋转测试"""
        rospy.loginfo("=== 步骤3: 执行旋转测试（复用现有控制器）===")
        
        if self.trajectory is None:
            rospy.logerr("❌ 轨迹未创建，无法执行测试")
            return False
        
        self.test_running = True
        self.start_time = rospy.Time.now().to_sec()
        
        rospy.loginfo("开始旋转测试...")
        rospy.loginfo("监控项目:")
        rospy.loginfo("  - 俯仰角变化")
        rospy.loginfo("  - 解耦控制激活状态")
        rospy.loginfo("  - 距离跟踪误差")
        rospy.loginfo("  - 控制模式切换")
        
        point_count = 0
        
        while not rospy.is_shutdown() and self.test_running:
            current_time = rospy.Time.now().to_sec()
            elapsed_time = current_time - self.start_time
            
            # 检查是否超时
            if elapsed_time > self.rotation_duration + 5.0:
                rospy.loginfo("测试完成（超时）")
                break
            
            # 获取轨迹点
            try:
                result = self.trajectory.get_next_position_and_yaw()
                if result is None:
                    rospy.loginfo("轨迹完成")
                    break
                
                position, yaw = result
                point_count += 1
                self.total_control_points += 1
                
                # 尝试获取速度信息
                velocity = None
                use_decoupled_control = False
                
                if hasattr(self.trajectory, 'get_velocity'):
                    try:
                        velocity = self.trajectory.get_velocity()
                        if velocity is not None:
                            use_decoupled_control = True
                            self.decoupled_activations += 1
                            if point_count % 20 == 0:  # 每20个点输出一次
                                rospy.loginfo(f"🔄 解耦控制: 速度=({velocity[0]:.3f}, {velocity[1]:.3f}, {velocity[2]:.3f}) m/s")
                    except Exception as e:
                        if point_count % 50 == 0:  # 减少错误输出频率
                            rospy.logwarn(f"获取速度失败: {e}")
                
                # 发送控制指令（复用父类的运动控制器）
                self.motion_controller.send_trajectory_point(
                    position, 
                    yaw, 
                    velocity=velocity,
                    use_decoupled_control=use_decoupled_control
                )
                
                # 统计和监控
                if point_count % 10 == 0:  # 每10个点统计一次
                    self.collect_statistics()
                    
                if point_count % 50 == 0:  # 每50个点输出一次详细信息
                    self.print_detailed_status(elapsed_time, position, yaw, use_decoupled_control)
                
            except Exception as e:
                rospy.logwarn(f"轨迹执行异常: {e}")
                break
            
            rospy.sleep(0.05)  # 20Hz控制频率
        
        self.test_running = False
        rospy.loginfo("=== 旋转测试结束 ===")
        self.print_final_statistics()
        return True
    
    def collect_statistics(self):
        """收集统计数据（复用父类的状态获取方法）"""
        try:
            # 俯仰角误差
            current_pitch = self.get_current_pitch()
            pitch_error_deg = abs(math.degrees(current_pitch))
            self.pitch_errors.append(pitch_error_deg)
            self.max_pitch_error = max(self.max_pitch_error, pitch_error_deg)
            
            # 距离误差
            current_pos = self.get_current_position()
            valve_pos = self.valve_center
            distance_error = math.sqrt(
                (current_pos[0] - valve_pos[0])**2 + 
                (current_pos[1] - valve_pos[1])**2
            )
            self.distance_errors.append(distance_error)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"统计数据收集失败: {e}")
    
    def print_detailed_status(self, elapsed_time, target_pos, target_yaw, use_decoupled):
        """打印详细状态信息（复用父类的状态获取方法）"""
        try:
            current_pos = self.get_current_position()
            current_yaw = self.get_current_yaw()
            current_pitch = self.get_current_pitch()
            
            # 位置误差
            pos_error = math.sqrt(
                (current_pos[0] - target_pos[0])**2 + 
                (current_pos[1] - target_pos[1])**2 + 
                (current_pos[2] - target_pos[2])**2
            )
            
            # 姿态误差
            pitch_error = abs(math.degrees(current_pitch))
            yaw_error = abs(math.degrees(self.normalize_angle(current_yaw - target_yaw)))
            
            rospy.loginfo(f"⏱️ 时间: {elapsed_time:.1f}s")
            rospy.loginfo(f"📍 位置误差: {pos_error*1000:.1f}mm")
            rospy.loginfo(f"📐 俯仰误差: {pitch_error:.1f}°")
            rospy.loginfo(f"🧭 偏航误差: {yaw_error:.1f}°")
            rospy.loginfo(f"🔄 解耦控制: {'激活' if use_decoupled else '未激活'}")
            rospy.loginfo("---")
        except Exception as e:
            rospy.logwarn(f"状态打印失败: {e}")
    
    def print_final_statistics(self):
        """打印最终统计结果"""
        rospy.loginfo("=== 测试统计结果 ===")
        
        if self.pitch_errors:
            avg_pitch = sum(self.pitch_errors) / len(self.pitch_errors)
            rospy.loginfo(f"俯仰角统计:")
            rospy.loginfo(f"  平均误差: {avg_pitch:.2f}°")
            rospy.loginfo(f"  最大误差: {self.max_pitch_error:.2f}°")
            rospy.loginfo(f"  数据点数: {len(self.pitch_errors)}")
        
        if self.distance_errors:
            avg_distance = sum(self.distance_errors) / len(self.distance_errors)
            max_distance = max(self.distance_errors)
            rospy.loginfo(f"距离误差统计:")
            rospy.loginfo(f"  平均误差: {avg_distance*1000:.1f}mm")
            rospy.loginfo(f"  最大误差: {max_distance*1000:.1f}mm")
        
        # 解耦控制激活率
        if self.total_control_points > 0:
            activation_rate = (self.decoupled_activations / self.total_control_points) * 100
            rospy.loginfo(f"解耦控制统计:")
            rospy.loginfo(f"  激活次数: {self.decoupled_activations}/{self.total_control_points}")
            rospy.loginfo(f"  激活率: {activation_rate:.1f}%")
        
        # 轨迹类型信息
        if self.trajectory:
            trajectory_type = type(self.trajectory).__name__
            has_get_velocity = hasattr(self.trajectory, 'get_velocity')
            rospy.loginfo(f"轨迹信息:")
            rospy.loginfo(f"  类型: {trajectory_type}")
            rospy.loginfo(f"  get_velocity: {has_get_velocity}")
    
    def run_test(self):
        """运行完整测试"""
        rospy.loginfo("=== 开始阀门旋转调试测试（复用现有代码）===")
        
        try:
            # 步骤1: 移动到测试位置
            if not self.move_to_test_position():
                rospy.logerr("移动到测试位置失败")
                return False
            
            rospy.sleep(2.0)  # 稳定2秒
            
            # 步骤2: 创建旋转轨迹
            if not self.create_rotation_trajectory_using_existing_methods():
                rospy.logerr("创建旋转轨迹失败")
                return False
            
            rospy.sleep(1.0)  # 稳定1秒
            
            # 步骤3: 执行旋转测试
            if not self.execute_rotation_test_with_existing_controller():
                rospy.logerr("执行旋转测试失败")
                return False
            
            rospy.loginfo("✓ 测试完成")
            return True
            
        except Exception as e:
            rospy.logerr(f"测试过程中发生异常: {e}")
            return False
    
    def emergency_stop(self):
        """紧急停止"""
        rospy.logwarn("紧急停止测试")
        self.test_running = False

def main():
    """主函数"""
    try:
        rospy.init_node('valve_rotation_debug_tester', anonymous=True)
        
        tester = ValveRotationDebugTester()
        
        # 等待系统初始化
        rospy.sleep(2.0)
        
        # 运行测试
        success = tester.run_test()
        
        if success:
            rospy.loginfo("🎉 测试成功完成")
        else:
            rospy.logwarn("⚠️ 测试未完全成功")
        
        # 保持节点运行一段时间以观察结果
        rospy.loginfo("保持运行5秒以观察最终状态...")
        rospy.sleep(5.0)
        
    except KeyboardInterrupt:
        rospy.loginfo("用户中断测试")
    except Exception as e:
        rospy.logerr(f"测试失败: {e}")
    finally:
        rospy.loginfo("测试结束")

if __name__ == '__main__':
    main()
