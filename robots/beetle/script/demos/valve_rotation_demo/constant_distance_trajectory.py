#!/usr/bin/env python
"""
Constant Distance Trajectory Generator
保证末端执行器与阀门中心恒定距离的轨迹生成器

作者基于对问题的深入理解，实现了一个新的轨迹生成方案：
- 末端执行器保持与阀门中心的恒定距离
- 无人机中心轨迹根据末端执行器的圆周运动反向计算
- 解决了直接对齐无人机中心、末端执行器和阀门中心的困难
"""

import rospy
import math
from math import pi, cos, sin, atan2, sqrt
from trajectory import PolynomialTrajectory


class ConstantDistanceValveRotationTrajectory:
    """
    末端执行器保持与阀门中心恒定距离的旋转轨迹生成器
    
    核心思想：
    1. 末端执行器围绕阀门中心做圆周运动，保持恒定距离
    2. 无人机中心位置根据末端执行器目标位置反向计算
    3. 确保系统稳定性和控制精度
    """
    
    def __init__(self, valve_center, end_effector_distance, rotation_angle, 
                 rotation_duration, start_angle, 
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823,
                 rotation_direction=1, grasp_height=None):
        """
        初始化恒定距离旋转轨迹生成器
        
        Args:
            valve_center: 阀门中心坐标 (x, y, z)
            end_effector_distance: 末端执行器到阀门中心的恒定距离
            rotation_angle: 旋转角度（弧度）
            rotation_duration: 旋转持续时间（秒）
            start_angle: 起始角度（弧度）
            end_effector_offset_x/y/z: 末端执行器相对于无人机中心的偏移
            rotation_direction: 旋转方向（1=逆时针，-1=顺时针）
            grasp_height: 抓取高度（无人机中心高度）
        """
        
        self.valve_center = valve_center
        self.end_effector_distance = end_effector_distance
        self.rotation_angle = rotation_angle * rotation_direction
        self.rotation_duration = rotation_duration
        self.start_angle = start_angle
        
        # 末端执行器偏移参数
        self.end_effector_offset_x = end_effector_offset_x
        self.end_effector_offset_y = end_effector_offset_y
        self.end_effector_offset_z = end_effector_offset_z
        
        self.rotation_direction = rotation_direction
        self.grasp_height = grasp_height
        
        # 角度轨迹生成器
        self._angle_trajectory = None
        
        # 验证参数
        if end_effector_distance <= 0:
            raise ValueError("End-effector distance must be positive")
        if rotation_duration <= 0:
            raise ValueError("Rotation duration must be positive")
        
        rospy.loginfo("=== 恒定距离旋转轨迹生成器初始化 ===")
        rospy.loginfo(f"阀门中心: {valve_center}")
        rospy.loginfo(f"末端执行器恒定距离: {end_effector_distance:.3f}m")
        rospy.loginfo(f"旋转角度: {rotation_angle:.3f}rad ({rotation_angle*180/pi:.1f}°)")
        rospy.loginfo(f"旋转持续时间: {rotation_duration:.1f}s")
        rospy.loginfo(f"起始角度: {start_angle:.3f}rad ({start_angle*180/pi:.1f}°)")
        rospy.loginfo(f"末端执行器偏移: [{end_effector_offset_x:.3f}, {end_effector_offset_y:.3f}, {end_effector_offset_z:.3f}]")
        rospy.loginfo(f"旋转方向: {'逆时针' if rotation_direction > 0 else '顺时针'}")
        
    def start_trajectory(self):
        """启动轨迹生成"""
        end_angle = self.start_angle + self.rotation_angle
        self._angle_trajectory = PolynomialTrajectory(self.rotation_duration)
        self._angle_trajectory.generate_trajectory(self.start_angle, end_angle)
        
        rospy.loginfo("=== 恒定距离旋转轨迹启动 ===")
        rospy.loginfo(f"从 {self.start_angle:.3f}rad 到 {end_angle:.3f}rad")
        rospy.loginfo(f"末端执行器将保持与阀门中心 {self.end_effector_distance:.3f}m 的恒定距离")
        
    def get_next_uav_position_and_yaw(self):
        """
        获取下一个无人机位置和朝向
        
        Returns:
            tuple: ((x, y, z), yaw) 或 None（如果轨迹结束）
        """
        if self._angle_trajectory is None:
            rospy.logwarn("轨迹未启动，请先调用start_trajectory()")
            return None
        
        # 获取当前角度
        current_angle = self._angle_trajectory.evaluate()
        if current_angle is None:
            return None  # 轨迹结束
        
        # 步骤1：计算末端执行器目标位置（圆周运动）
        valve_x, valve_y, valve_z = self.valve_center
        
        end_effector_x = valve_x + self.end_effector_distance * cos(current_angle)
        end_effector_y = valve_y + self.end_effector_distance * sin(current_angle)
        
        # 步骤2：计算末端执行器目标高度
        if self.grasp_height is not None:
            # 使用指定的抓取高度作为无人机高度
            uav_z = self.grasp_height
            end_effector_z = uav_z + self.end_effector_offset_z
        else:
            # 使用阀门高度作为末端执行器高度
            end_effector_z = valve_z
            uav_z = end_effector_z - self.end_effector_offset_z
        
        # 步骤3：计算末端执行器朝向（指向阀门中心）
        dx_to_valve = valve_x - end_effector_x
        dy_to_valve = valve_y - end_effector_y
        end_effector_yaw = atan2(dy_to_valve, dx_to_valve)
        
        # 步骤4：无人机朝向与末端执行器朝向相同
        uav_yaw = end_effector_yaw
        
        # 步骤5：根据末端执行器目标位置反向计算无人机位置
        cos_yaw = cos(uav_yaw)
        sin_yaw = sin(uav_yaw)
        
        # 将末端执行器偏移从机体坐标系转换到世界坐标系
        world_offset_x = (cos_yaw * self.end_effector_offset_x - 
                         sin_yaw * self.end_effector_offset_y)
        world_offset_y = (sin_yaw * self.end_effector_offset_x + 
                         cos_yaw * self.end_effector_offset_y)
        
        # 反向计算无人机中心位置
        uav_x = end_effector_x - world_offset_x
        uav_y = end_effector_y - world_offset_y
        
        # 步骤6：验证计算结果
        self._verify_trajectory_point(uav_x, uav_y, uav_z, uav_yaw, current_angle)
        
        return ((uav_x, uav_y, uav_z), uav_yaw)
    
    def _verify_trajectory_point(self, uav_x, uav_y, uav_z, uav_yaw, current_angle):
        """验证轨迹点的正确性"""
        # 正向计算验证
        cos_yaw = cos(uav_yaw)
        sin_yaw = sin(uav_yaw)
        
        world_offset_x = (cos_yaw * self.end_effector_offset_x - 
                         sin_yaw * self.end_effector_offset_y)
        world_offset_y = (sin_yaw * self.end_effector_offset_x + 
                         cos_yaw * self.end_effector_offset_y)
        
        # 计算验证的末端执行器位置
        verify_ee_x = uav_x + world_offset_x
        verify_ee_y = uav_y + world_offset_y
        
        # 计算实际距离
        valve_x, valve_y, _ = self.valve_center
        actual_distance = sqrt((verify_ee_x - valve_x)**2 + (verify_ee_y - valve_y)**2)
        
        # 检查距离误差
        distance_error = abs(actual_distance - self.end_effector_distance)
        
        # 记录警告
        if distance_error > 0.001:  # 1mm误差阈值
            rospy.logwarn(f"轨迹点验证失败！")
            rospy.logwarn(f"  角度: {current_angle:.3f}rad ({current_angle*180/pi:.1f}°)")
            rospy.logwarn(f"  目标距离: {self.end_effector_distance:.3f}m")
            rospy.logwarn(f"  实际距离: {actual_distance:.3f}m")
            rospy.logwarn(f"  距离误差: {distance_error:.4f}m")
        
        # 每隔一段时间记录状态
        if abs(current_angle - self.start_angle) % (pi/6) < 0.01:  # 每30度
            rospy.loginfo(f"轨迹状态 - 角度: {current_angle*180/pi:.1f}°, 距离: {actual_distance:.3f}m, 误差: {distance_error:.4f}m")
    
    def is_complete(self):
        """检查轨迹是否完成"""
        if self._angle_trajectory is None:
            return True
        return self._angle_trajectory.evaluate() is None
    
    def get_current_angle(self):
        """获取当前角度"""
        if self._angle_trajectory is None:
            return None
        return self._angle_trajectory.evaluate()
    
    def get_trajectory_info(self):
        """获取轨迹信息"""
        return {
            'valve_center': self.valve_center,
            'end_effector_distance': self.end_effector_distance,
            'rotation_angle': self.rotation_angle,
            'rotation_duration': self.rotation_duration,
            'start_angle': self.start_angle,
            'rotation_direction': self.rotation_direction,
            'grasp_height': self.grasp_height
        }
    
    def check_trajectory_continuity(self, previous_uav_pos, current_uav_pos, max_step_size=0.05):
        """
        检查轨迹连续性，确保无人机运动平滑
        
        Args:
            previous_uav_pos: 上一个无人机位置
            current_uav_pos: 当前无人机位置
            max_step_size: 最大允许步长（米）
        
        Returns:
            bool: 轨迹是否连续
        """
        if previous_uav_pos is None or current_uav_pos is None:
            return True
        
        step_size = sqrt((current_uav_pos[0] - previous_uav_pos[0])**2 + 
                        (current_uav_pos[1] - previous_uav_pos[1])**2 + 
                        (current_uav_pos[2] - previous_uav_pos[2])**2)
        
        if step_size > max_step_size:
            rospy.logwarn(f"轨迹步长过大: {step_size:.3f}m > {max_step_size:.3f}m")
            return False
        
        return True
    
    def monitor_end_effector_distance(self, current_uav_pos, current_uav_yaw, tolerance=0.01):
        """
        实时监控末端执行器与阀门中心的距离
        
        Args:
            current_uav_pos: 当前无人机位置
            current_uav_yaw: 当前无人机朝向
            tolerance: 允许的距离偏差（米）
        
        Returns:
            dict: 监控结果
        """
        if current_uav_pos is None:
            return {'status': 'error', 'message': 'Invalid UAV position'}
        
        # 计算当前实际的末端执行器位置
        cos_yaw = cos(current_uav_yaw)
        sin_yaw = sin(current_uav_yaw)
        
        world_offset_x = (cos_yaw * self.end_effector_offset_x - 
                         sin_yaw * self.end_effector_offset_y)
        world_offset_y = (sin_yaw * self.end_effector_offset_x + 
                         cos_yaw * self.end_effector_offset_y)
        
        actual_ee_x = current_uav_pos[0] + world_offset_x
        actual_ee_y = current_uav_pos[1] + world_offset_y
        
        # 计算实际距离
        valve_x, valve_y, _ = self.valve_center
        actual_distance = sqrt((actual_ee_x - valve_x)**2 + (actual_ee_y - valve_y)**2)
        
        # 检查距离偏差
        distance_error = abs(actual_distance - self.end_effector_distance)
        
        result = {
            'status': 'ok' if distance_error <= tolerance else 'warning',
            'target_distance': self.end_effector_distance,
            'actual_distance': actual_distance,
            'distance_error': distance_error,
            'tolerance': tolerance,
            'actual_ee_pos': (actual_ee_x, actual_ee_y),
            'valve_center': (valve_x, valve_y)
        }
        
        if distance_error > tolerance:
            rospy.logwarn(f"末端执行器距离偏差: {distance_error:.3f}m > {tolerance:.3f}m")
            rospy.logwarn(f"目标距离: {self.end_effector_distance:.3f}m, 实际距离: {actual_distance:.3f}m")
        
        return result
def create_constant_distance_trajectory(current_uav_pos, current_uav_yaw, valve_center, 
                                      rotation_angle, rotation_duration, 
                                      end_effector_offset_x=0.246, end_effector_offset_y=0.0, 
                                      end_effector_offset_z=0.0743823, rotation_direction=1):
    """
    Convenience function to create constant distance rotation trajectory
    
    Args:
        current_uav_pos: 当前无人机位置 (x, y, z)
        current_uav_yaw: 当前无人机朝向（弧度）
        valve_center: 阀门中心位置 (x, y, z)
        rotation_angle: 旋转角度（弧度）
        rotation_duration: 旋转持续时间（秒）
        end_effector_offset_x/y/z: 末端执行器偏移
        rotation_direction: 旋转方向（1=逆时针，-1=顺时针）
    
    Returns:
        ConstantDistanceValveRotationTrajectory: 轨迹生成器对象
    """
    
    # 计算当前末端执行器位置
    cos_yaw = cos(current_uav_yaw)
    sin_yaw = sin(current_uav_yaw)
    
    world_offset_x = (cos_yaw * end_effector_offset_x - sin_yaw * end_effector_offset_y)
    world_offset_y = (sin_yaw * end_effector_offset_x + cos_yaw * end_effector_offset_y)
    
    current_ee_x = current_uav_pos[0] + world_offset_x
    current_ee_y = current_uav_pos[1] + world_offset_y
    
    # 计算末端执行器到阀门中心的距离
    valve_x, valve_y, _ = valve_center
    end_effector_distance = sqrt((current_ee_x - valve_x)**2 + (current_ee_y - valve_y)**2)
    
    # Calculate starting angle
    start_angle = atan2(current_ee_y - valve_y, current_ee_x - valve_x)
    
    rospy.loginfo("=== Creating Constant Distance Rotation Trajectory ===")
    rospy.loginfo(f"Current UAV position: {current_uav_pos}")
    rospy.loginfo(f"Current end-effector position: [{current_ee_x:.3f}, {current_ee_y:.3f}]")
    rospy.loginfo(f"Valve center: {valve_center}")
    rospy.loginfo(f"Calculated end-effector distance: {end_effector_distance:.3f}m")
    rospy.loginfo(f"Starting angle: {start_angle:.3f}rad ({start_angle*180/pi:.1f}°)")
    
    # Create trajectory generator
    trajectory = ConstantDistanceValveRotationTrajectory(
        valve_center=valve_center,
        end_effector_distance=end_effector_distance,
        rotation_angle=rotation_angle,
        rotation_duration=rotation_duration,
        start_angle=start_angle,
        end_effector_offset_x=end_effector_offset_x,
        end_effector_offset_y=end_effector_offset_y,
        end_effector_offset_z=end_effector_offset_z,
        rotation_direction=rotation_direction,
        grasp_height=current_uav_pos[2]  # 保持当前高度
    )
    
    return trajectory


if __name__ == "__main__":
    # 测试示例
    rospy.init_node("constant_distance_trajectory_test")
    
    # 测试参数
    valve_center = (0.5, 0.3, 0.2)
    current_uav_pos = (0.3, 0.5, 0.25)
    current_uav_yaw = 0.7854  # 45度
    rotation_angle = pi/2  # 90度
    rotation_duration = 8.0
    
    # 创建轨迹
    trajectory = create_constant_distance_trajectory(
        current_uav_pos, current_uav_yaw, valve_center, 
        rotation_angle, rotation_duration
    )
    
    # 启动轨迹
    trajectory.start_trajectory()
    
    # 模拟执行
    rate = rospy.Rate(50)
    point_count = 0
    
    rospy.loginfo("=== 开始轨迹测试 ===")
    
    while not rospy.is_shutdown() and not trajectory.is_complete():
        result = trajectory.get_next_uav_position_and_yaw()
        if result is None:
            break
        
        pos, yaw = result
        point_count += 1
        
        if point_count % 50 == 0:  # 每秒记录一次
            current_angle = trajectory.get_current_angle()
            rospy.loginfo(f"轨迹点 {point_count}: pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], yaw={yaw:.3f}rad, angle={current_angle*180/pi:.1f}°")
        
        rate.sleep()
    
    rospy.loginfo("=== 轨迹测试完成 ===")
    rospy.loginfo(f"总共生成了 {point_count} 个轨迹点")
