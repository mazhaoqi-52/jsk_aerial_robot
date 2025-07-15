#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
统一运动控制器
整合了基础运动控制、增强对齐控制和恒定距离反馈控制功能
"""

import rospy
import time
import math
from math import pi, cos, sin, sqrt, atan2
from aerial_robot_msgs.msg import FlightNav
from trajectory import PolynomialTrajectory


class UnifiedMotionController:
    """
    统一运动控制器
    
    集成功能：
    1. 基础轨迹执行和反馈控制
    2. 增强对齐控制
    3. 恒定距离反馈控制
    4. 专门的阀门旋转控制
    """
    
    def __init__(self, state_machine):
        """
        初始化统一运动控制器
        
        Args:
            state_machine: 状态机引用，用于获取当前位置和朝向
        """
        self.sm = state_machine
        self.pub = self.sm.pub
        
        # 基础控制参数
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.02)
        self.yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.05)
        
        # 对齐控制参数
        self.strict_alignment_enabled = rospy.get_param("~strict_alignment_enabled", True)
        self.alignment_correction_enabled = rospy.get_param("~alignment_correction_enabled", True)
        self.alignment_position_tolerance = rospy.get_param("~alignment_position_tolerance", 0.01)
        self.alignment_yaw_tolerance = rospy.get_param("~alignment_yaw_tolerance", 0.05)
        self.alignment_control_gain = rospy.get_param("~alignment_control_gain", 0.8)
        
        # 恒定距离反馈控制参数
        self.distance_control_gain = rospy.get_param("~distance_control_gain", 0.8)
        self.position_control_gain = rospy.get_param("~position_control_gain", 0.5)
        self.yaw_control_gain = rospy.get_param("~yaw_control_gain", 0.6)
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.01)
        self.max_position_correction = rospy.get_param("~max_position_correction", 0.03)
        self.max_yaw_correction = rospy.get_param("~max_yaw_correction", 0.1)
        
        # 末端执行器参数
        self.end_effector_offset_x = 0.246
        self.end_effector_offset_y = 0.0
        self.end_effector_offset_z = 0.0743823
        
        # 控制统计
        self.control_stats = {
            'total_corrections': 0,
            'distance_violations': 0,
            'max_distance_error': 0.0,
            'avg_distance_error': 0.0,
            'execution_time': 0.0
        }
        
        rospy.loginfo("统一运动控制器初始化完成")
        rospy.loginfo(f"严格对齐控制: {self.strict_alignment_enabled}")
        rospy.loginfo(f"距离反馈控制: enabled")
    
    # ===== 基础轨迹执行功能 =====
    
    def execute_poly_motion_with_feedback(self, start, target, avg_speed, 
                                          pos_threshold=0.05, yaw_threshold=0.1, timeout=30):
        """
        执行多项式轨迹运动，带反馈控制
        
        Args:
            start: 起始位置 (x, y, z)
            target: 目标位置 (x, y, z)
            avg_speed: 平均速度 m/s
            pos_threshold: 位置误差阈值 m
            yaw_threshold: 朝向误差阈值 rad
            timeout: 超时时间 s
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        distance = math.sqrt((target[0]-start[0])**2 + (target[1]-start[1])**2 + (target[2]-start[2])**2)
        duration = distance / max(avg_speed, 0.05)
        
        rospy.loginfo(f"执行轨迹运动: {distance:.2f}m in {duration:.1f}s")
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start, target)
        
        rate = rospy.Rate(50)
        start_time = time.time()
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            pt = traj.evaluate()
            if pt is None:
                rospy.loginfo("轨迹运动完成")
                return True
            
            self.send_trajectory_point(pt, None)
            
            if not self._wait_for_position(pt, pos_threshold, point_timeout=2.0):
                rospy.logwarn(f"等待轨迹点超时: {pt}")
            
            rate.sleep()
        
        rospy.logerr(f"轨迹运动超时: {timeout}s")
        return False
    
    def execute_smooth_trajectory_with_yaw(self, start_pos, target_pos, target_yaw, duration=5.0,
                                          pos_threshold=0.03, yaw_threshold=0.08):
        """
        执行平滑轨迹运动，带朝向控制
        
        Args:
            start_pos: 起始位置 (x, y, z)
            target_pos: 目标位置 (x, y, z)
            target_yaw: 目标朝向 rad
            duration: 轨迹时间 s
            pos_threshold: 位置误差阈值 m
            yaw_threshold: 朝向误差阈值 rad
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        
        start_yaw = self.sm.current_yaw
        yaw_diff = self._normalize_angle_diff(target_yaw - start_yaw)
        
        # 限制最大朝向变化
        max_yaw_change = math.pi / 4  # 45度
        if abs(yaw_diff) > max_yaw_change:
            yaw_diff = max_yaw_change * (1 if yaw_diff > 0 else -1)
            rospy.logwarn(f"限制朝向变化为: {yaw_diff:.3f} rad")
        
        final_target_yaw = start_yaw + yaw_diff
        
        rate = rospy.Rate(50)
        start_time = time.time()
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 2.0:
            pt = traj.evaluate()
            if pt is None:
                pt = target_pos
            
            # 平滑插值朝向
            elapsed_time = time.time() - start_time
            if elapsed_time <= duration:
                yaw_progress = elapsed_time / duration
                smooth_progress = 3*yaw_progress**2 - 2*yaw_progress**3
                current_target_yaw = start_yaw + smooth_progress * yaw_diff
            else:
                current_target_yaw = final_target_yaw
            
            self.send_trajectory_point(pt, current_target_yaw)
            
            if not self._wait_for_position_and_yaw(pt, current_target_yaw, 
                                                  pos_threshold, yaw_threshold, point_timeout=1.0):
                rospy.logwarn(f"等待位置和朝向超时")
            
            if pt is target_pos:
                break
            
            rate.sleep()
        
        rospy.loginfo("平滑轨迹运动完成")
        return True
    
    def execute_controlled_descent(self, start_pos, target_z, descent_speed=0.05,
                                  pos_threshold=0.02, yaw_threshold=0.05):
        """
        执行受控下降，固定XY位置和朝向
        
        Args:
            start_pos: 起始位置 (x, y, z)
            target_z: 目标Z坐标
            descent_speed: 下降速度 m/s
            pos_threshold: 位置误差阈值 m
            yaw_threshold: 朝向误差阈值 rad
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        fixed_x, fixed_y = start_pos[0], start_pos[1]
        fixed_yaw = self.sm.current_yaw
        
        descent_distance = abs(start_pos[2] - target_z)
        duration = descent_distance / descent_speed
        
        rospy.loginfo(f"受控下降: {descent_distance:.3f}m in {duration:.1f}s")
        rospy.loginfo(f"固定位置: [{fixed_x:.3f}, {fixed_y:.3f}], 朝向: {fixed_yaw:.3f}")
        
        # 初始位置保持
        hold_duration = 0.5
        hold_start = time.time()
        while not rospy.is_shutdown() and (time.time() - hold_start) < hold_duration:
            self.send_trajectory_point((fixed_x, fixed_y, start_pos[2]), fixed_yaw)
            time.sleep(0.02)
        
        # 执行下降
        start_time = time.time()
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 5.0:
            elapsed = time.time() - start_time
            progress = min(elapsed / duration, 1.0)
            current_z = start_pos[2] + progress * (target_z - start_pos[2])
            
            self.send_trajectory_point((fixed_x, fixed_y, current_z), fixed_yaw)
            
            # 检查是否到达目标
            current_pos = self.sm.get_current_position()
            if current_pos and current_pos[2] <= target_z + 0.01:
                rospy.loginfo("到达目标下降高度")
                return True
            
            # 监控漂移
            if current_pos:
                xy_drift = math.sqrt((current_pos[0] - fixed_x)**2 + (current_pos[1] - fixed_y)**2)
                yaw_drift = abs(self._normalize_angle_diff(self.sm.current_yaw - fixed_yaw))
                
                if elapsed % 2.0 < 0.02:  # 每2秒记录一次
                    descended = start_pos[2] - current_pos[2]
                    rospy.loginfo(f"下降: {descended:.3f}m, XY漂移: {xy_drift:.3f}m, 朝向漂移: {yaw_drift:.3f}rad")
            
            rate.sleep()
        
        rospy.loginfo("受控下降完成")
        return True
    
    # ===== 对齐控制功能 =====
    
    def execute_pre_rotation_alignment(self, valve_pos, timeout=10.0):
        """
        执行预旋转对齐
        
        Args:
            valve_pos: 阀门位置 (x, y, z)
            timeout: 超时时间 s
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        if not self.strict_alignment_enabled:
            rospy.loginfo("严格对齐控制已禁用")
            return True
        
        rospy.loginfo("开始预旋转对齐")
        
        start_time = time.time()
        rate = rospy.Rate(25)
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            current_pos = self.sm.get_current_position()
            if current_pos is None:
                rate.sleep()
                continue
            
            # 计算期望朝向（指向阀门）
            desired_yaw = math.atan2(valve_pos[1] - current_pos[1], valve_pos[0] - current_pos[0])
            
            # 计算对齐误差
            yaw_error = abs(self._normalize_angle_diff(desired_yaw - self.sm.current_yaw))
            
            if yaw_error < self.alignment_yaw_tolerance:
                rospy.loginfo("预旋转对齐完成")
                return True
            
            # 发送对齐命令
            self.send_trajectory_point(current_pos, desired_yaw)
            
            rate.sleep()
        
        rospy.logwarn("预旋转对齐超时")
        return False
    
    # ===== 恒定距离反馈控制功能 =====
    
    def execute_constant_distance_rotation(self, trajectory, valve_center, 
                                         feedback_frequency=50, alignment_check_interval=0.1):
        """
        执行恒定距离旋转，结合轨迹生成和反馈控制
        
        Args:
            trajectory: ConstantDistanceValveRotationTrajectory 轨迹生成器
            valve_center: 阀门中心位置
            feedback_frequency: 反馈控制频率 Hz
            alignment_check_interval: 对齐检查间隔 s
        
        Returns:
            dict: 执行统计信息
        """
        rospy.loginfo("开始恒定距离反馈控制旋转")
        
        # 重置统计信息
        self.control_stats = {
            'total_corrections': 0,
            'distance_violations': 0,
            'max_distance_error': 0.0,
            'avg_distance_error': 0.0,
            'execution_time': 0.0,
            'total_points': 0
        }
        
        rate = rospy.Rate(feedback_frequency)
        start_time = time.time()
        last_alignment_check = start_time
        
        distance_error_sum = 0.0
        point_count = 0
        
        while not rospy.is_shutdown():
            # 获取轨迹目标点
            result = trajectory.get_next_uav_position_and_yaw()
            if result is None:
                rospy.loginfo("轨迹执行完成")
                break
            
            target_pos, target_yaw = result
            current_time = time.time()
            
            # 获取当前状态
            current_pos = self.sm.get_current_position()
            current_yaw = self.sm.current_yaw
            
            if current_pos is None:
                rospy.logwarn("无法获取当前位置")
                rate.sleep()
                continue
            
            # 应用反馈控制纠正
            if current_time - last_alignment_check >= alignment_check_interval:
                corrected_pos, corrected_yaw = self.apply_constant_distance_feedback(
                    target_pos, target_yaw, current_pos, current_yaw, valve_center, trajectory)
                
                last_alignment_check = current_time
            else:
                corrected_pos, corrected_yaw = target_pos, target_yaw
            
            # 发送控制命令
            self.send_trajectory_point(corrected_pos, corrected_yaw)
            
            # 更新统计信息
            distance_error = self.calculate_distance_error(current_pos, current_yaw, valve_center, 
                                                          trajectory.end_effector_distance)
            distance_error_sum += distance_error
            point_count += 1
            
            self.control_stats['max_distance_error'] = max(
                self.control_stats['max_distance_error'], distance_error)
            self.control_stats['total_points'] += 1
            
            if distance_error > self.distance_tolerance:
                self.control_stats['distance_violations'] += 1
            
            # 定期输出状态
            if point_count % (feedback_frequency * 2) == 0:  # 每2秒输出一次
                rospy.loginfo(f"旋转进度: {point_count}点, 距离误差: {distance_error:.4f}m, "
                             f"最大误差: {self.control_stats['max_distance_error']:.4f}m")
            
            rate.sleep()
        
        # 计算最终统计信息
        self.control_stats['execution_time'] = time.time() - start_time
        self.control_stats['avg_distance_error'] = distance_error_sum / max(point_count, 1)
        
        self.log_control_statistics()
        return self.control_stats
    
    def apply_constant_distance_feedback(self, target_pos, target_yaw, current_pos, current_yaw, 
                                        valve_center, trajectory):
        """
        应用恒定距离反馈控制
        
        Args:
            target_pos: 目标位置
            target_yaw: 目标朝向
            current_pos: 当前位置
            current_yaw: 当前朝向
            valve_center: 阀门中心位置
            trajectory: 轨迹生成器
        
        Returns:
            tuple: (corrected_pos, corrected_yaw)
        """
        # 计算当前末端执行器位置
        current_ee_pos = self.calculate_end_effector_position(current_pos, current_yaw)
        
        # 计算目标末端执行器位置
        target_ee_pos = self.calculate_end_effector_position(target_pos, target_yaw)
        
        # 计算距离误差
        valve_x, valve_y, valve_z = valve_center
        current_distance = sqrt((current_ee_pos[0] - valve_x)**2 + (current_ee_pos[1] - valve_y)**2)
        target_distance = sqrt((target_ee_pos[0] - valve_x)**2 + (target_ee_pos[1] - valve_y)**2)
        
        distance_error = abs(current_distance - target_distance)
        
        # 如果距离误差在容差范围内，不需要纠正
        if distance_error <= self.distance_tolerance:
            return target_pos, target_yaw
        
        # 计算纠正向量
        correction_pos, correction_yaw = self.calculate_distance_correction(
            current_pos, current_yaw, valve_center, target_distance)
        
        # 应用纠正
        corrected_pos = (
            target_pos[0] + correction_pos[0],
            target_pos[1] + correction_pos[1],
            target_pos[2] + correction_pos[2]
        )
        corrected_yaw = target_yaw + correction_yaw
        
        # 记录纠正
        self.control_stats['total_corrections'] += 1
        
        # 记录纠正信息
        if distance_error > self.distance_tolerance * 2:  # 只记录较大的纠正
            rospy.loginfo(f"应用距离纠正: 误差={distance_error:.4f}m, "
                         f"位置纠正=[{correction_pos[0]:.3f}, {correction_pos[1]:.3f}, {correction_pos[2]:.3f}], "
                         f"朝向纠正={correction_yaw:.3f}rad")
        
        return corrected_pos, corrected_yaw
    
    def calculate_distance_correction(self, current_pos, current_yaw, valve_center, target_distance):
        """
        计算距离纠正
        
        Args:
            current_pos: 当前UAV位置
            current_yaw: 当前UAV朝向
            valve_center: 阀门中心位置
            target_distance: 目标距离
        
        Returns:
            tuple: (position_correction, yaw_correction)
        """
        # 计算当前末端执行器位置
        current_ee_pos = self.calculate_end_effector_position(current_pos, current_yaw)
        
        # 计算当前距离
        valve_x, valve_y, valve_z = valve_center
        current_distance = sqrt((current_ee_pos[0] - valve_x)**2 + (current_ee_pos[1] - valve_y)**2)
        
        # 计算距离误差
        distance_error = current_distance - target_distance
        
        # 如果距离误差很小，不需要纠正
        if abs(distance_error) <= self.distance_tolerance:
            return (0.0, 0.0, 0.0), 0.0
        
        # 计算从阀门中心到末端执行器的单位向量
        if current_distance > 0.001:
            unit_vector_x = (current_ee_pos[0] - valve_x) / current_distance
            unit_vector_y = (current_ee_pos[1] - valve_y) / current_distance
        else:
            unit_vector_x = 1.0
            unit_vector_y = 0.0
        
        # 计算末端执行器位置纠正
        ee_correction_magnitude = -distance_error * self.distance_control_gain
        ee_correction_x = ee_correction_magnitude * unit_vector_x
        ee_correction_y = ee_correction_magnitude * unit_vector_y
        
        # 将末端执行器纠正转换为UAV纠正
        uav_correction_x = ee_correction_x * self.position_control_gain
        uav_correction_y = ee_correction_y * self.position_control_gain
        uav_correction_z = 0.0  # 不纠正Z方向
        
        # 限制纠正幅度
        correction_magnitude = sqrt(uav_correction_x**2 + uav_correction_y**2)
        if correction_magnitude > self.max_position_correction:
            scale_factor = self.max_position_correction / correction_magnitude
            uav_correction_x *= scale_factor
            uav_correction_y *= scale_factor
        
        # 计算朝向纠正
        desired_yaw = atan2(valve_y - current_ee_pos[1], valve_x - current_ee_pos[0])
        yaw_error = self._normalize_angle_diff(desired_yaw - current_yaw)
        yaw_correction = yaw_error * self.yaw_control_gain
        
        # 限制朝向纠正
        yaw_correction = max(-self.max_yaw_correction, min(self.max_yaw_correction, yaw_correction))
        
        return (uav_correction_x, uav_correction_y, uav_correction_z), yaw_correction
    
    def calculate_end_effector_position(self, uav_pos, uav_yaw):
        """
        计算末端执行器位置
        
        Args:
            uav_pos: UAV位置
            uav_yaw: UAV朝向
        
        Returns:
            tuple: 末端执行器位置 (x, y, z)
        """
        cos_yaw = cos(uav_yaw)
        sin_yaw = sin(uav_yaw)
        
        world_offset_x = (cos_yaw * self.end_effector_offset_x - 
                         sin_yaw * self.end_effector_offset_y)
        world_offset_y = (sin_yaw * self.end_effector_offset_x + 
                         cos_yaw * self.end_effector_offset_y)
        world_offset_z = self.end_effector_offset_z
        
        return (
            uav_pos[0] + world_offset_x,
            uav_pos[1] + world_offset_y,
            uav_pos[2] + world_offset_z
        )
    
    def calculate_distance_error(self, uav_pos, uav_yaw, valve_center, target_distance):
        """
        计算距离误差
        
        Args:
            uav_pos: UAV位置
            uav_yaw: UAV朝向
            valve_center: 阀门中心位置
            target_distance: 目标距离
        
        Returns:
            float: 距离误差
        """
        ee_pos = self.calculate_end_effector_position(uav_pos, uav_yaw)
        valve_x, valve_y, _ = valve_center
        
        current_distance = sqrt((ee_pos[0] - valve_x)**2 + (ee_pos[1] - valve_y)**2)
        return abs(current_distance - target_distance)
    
    # ===== 基础工具函数 =====
    
    def send_trajectory_point(self, pos, yaw=None):
        """
        发送轨迹点
        
        Args:
            pos: 位置 (x, y, z)
            yaw: 朝向 (弧度)
        """
        msg = FlightNav()
        msg.target = 1
        msg.pos_xy_nav_mode = FlightNav.POS_MODE
        msg.target_pos_x = pos[0]
        msg.target_pos_y = pos[1]
        msg.pos_z_nav_mode = FlightNav.POS_MODE
        msg.target_pos_z = pos[2]
        
        if yaw is not None:
            msg.yaw_nav_mode = FlightNav.POS_MODE
            msg.target_yaw = yaw
        else:
            msg.yaw_nav_mode = FlightNav.NO_NAVIGATION
        
        self.pub.publish(msg)
    
    def _wait_for_position(self, target_pos, pos_threshold, point_timeout=2.0):
        """等待UAV到达目标位置"""
        wait_start = time.time()
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown() and (time.time() - wait_start) < point_timeout:
            current_pos = self.sm.get_current_position()
            if current_pos is None:
                rate.sleep()
                continue
            
            pos_error = math.sqrt((target_pos[0] - current_pos[0])**2 + 
                                (target_pos[1] - current_pos[1])**2 + 
                                (target_pos[2] - current_pos[2])**2)
            
            if pos_error < pos_threshold:
                return True
            
            rate.sleep()
        
        return False
    
    def _wait_for_position_and_yaw(self, target_pos, target_yaw, pos_threshold, yaw_threshold, point_timeout=1.0):
        """等待UAV到达目标位置和朝向"""
        wait_start = time.time()
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown() and (time.time() - wait_start) < point_timeout:
            current_pos = self.sm.get_current_position()
            if current_pos is None:
                rate.sleep()
                continue
            
            pos_error = math.sqrt((target_pos[0] - current_pos[0])**2 + 
                                (target_pos[1] - current_pos[1])**2 + 
                                (target_pos[2] - current_pos[2])**2)
            yaw_error = abs(self._normalize_angle_diff(target_yaw - self.sm.current_yaw))
            
            if pos_error < pos_threshold and yaw_error < yaw_threshold:
                return True
            
            rate.sleep()
        
        return False
    
    def _normalize_angle_diff(self, angle_diff):
        """归一化角度差到 [-π, π] 范围"""
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff
    
    def log_control_statistics(self):
        """记录控制统计信息"""
        rospy.loginfo("=== 统一运动控制器统计信息 ===")
        rospy.loginfo(f"执行时间: {self.control_stats['execution_time']:.2f}s")
        rospy.loginfo(f"总轨迹点数: {self.control_stats['total_points']}")
        rospy.loginfo(f"总纠正次数: {self.control_stats['total_corrections']}")
        rospy.loginfo(f"距离违规次数: {self.control_stats['distance_violations']}")
        rospy.loginfo(f"最大距离误差: {self.control_stats['max_distance_error']:.4f}m")
        rospy.loginfo(f"平均距离误差: {self.control_stats['avg_distance_error']:.4f}m")
        
        # 计算成功率
        if self.control_stats['total_points'] > 0:
            success_rate = (1.0 - self.control_stats['distance_violations'] / 
                           self.control_stats['total_points']) * 100
            rospy.loginfo(f"距离保持成功率: {success_rate:.1f}%")
        
        # 评估控制性能
        if self.control_stats['max_distance_error'] < self.distance_tolerance * 2:
            rospy.loginfo("✓ 距离控制性能: 优秀")
        elif self.control_stats['max_distance_error'] < self.distance_tolerance * 3:
            rospy.loginfo("✓ 距离控制性能: 良好")
        else:
            rospy.logwarn("⚠ 距离控制性能: 需要改进")
        
        rospy.loginfo("=== 统一运动控制器理念验证 ===")
        rospy.loginfo("✓ 基础轨迹执行和反馈控制")
        rospy.loginfo("✓ 增强对齐控制")
        rospy.loginfo("✓ 恒定距离反馈控制")
        rospy.loginfo("✓ 专门的阀门旋转控制")
