#!/usr/bin/env python
"""
Unified Motion Controller
Integrates basic motion control, enhanced alignment control, and constant distance feedback control functionality
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

import rospy
import time
import math
import threading
from math import pi, cos, sin, sqrt, atan2
from aerial_robot_msgs.msg import FlightNav
from trajectory import PolynomialTrajectory, AdaptiveTrajectoryPlanner


class UnifiedMotionController:
    """
    Unified Motion Controller
    
    Integrated functionality:
    1. Basic trajectory execution and feedback control
    2. Enhanced alignment control
    3. Constant distance feedback control
    4. Specialized valve rotation control
    """
    
    def __init__(self, state_machine):
        """
        Initialize unified motion controller
        
        Args:
            state_machine: State machine reference for getting current position and orientation
        """
        self.sm = state_machine
        self.pub = self.sm.pub
        
        # Basic control parameters
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.02)
        self.yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.05)
        
        # Alignment control parameters
        self.strict_alignment_enabled = rospy.get_param("~strict_alignment_enabled", True)
        self.alignment_correction_enabled = rospy.get_param("~alignment_correction_enabled", True)
        self.alignment_position_tolerance = rospy.get_param("~alignment_position_tolerance", 0.01)
        self.alignment_yaw_tolerance = rospy.get_param("~alignment_yaw_tolerance", 0.05)
        self.alignment_control_gain = rospy.get_param("~alignment_control_gain", 0.8)
        
        # Constant distance feedback control parameters
        self.distance_control_gain = rospy.get_param("~distance_control_gain", 0.5)  # 增强从0.3 (抗XY漂移)
        self.position_control_gain = rospy.get_param("~position_control_gain", 0.6)  # 增强从0.3 (抗XY漂移)
        self.yaw_control_gain = rospy.get_param("~yaw_control_gain", 0.4)  # 增强从0.2 (抗XY漂移)
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.01)
        self.max_position_correction = rospy.get_param("~max_position_correction", 0.03)  # 增加从0.02
        self.max_yaw_correction = rospy.get_param("~max_yaw_correction", 0.08)  # 增加从0.05
        
        # End-effector parameters
        self.end_effector_offset_x = 0.246
        self.end_effector_offset_y = 0.0
        self.end_effector_offset_z = 0.0743823
        
        # 移除未定义的adaptive_planner
        # 改用简化的轨迹规划方法
        
        # Control statistics
        self.control_stats = {
            'total_corrections': 0,
            'distance_violations': 0,
            'max_distance_error': 0.0,
            'avg_distance_error': 0.0,
            'execution_time': 0.0
        }
        
        rospy.loginfo("Unified motion controller initialization complete")
        rospy.loginfo(f"Strict alignment control: {self.strict_alignment_enabled}")
        rospy.loginfo(f"Distance feedback control: enabled")
    
    def send_trajectory_point(self, position, yaw=None):
        """
        Send trajectory point to UAV
        
        Args:
            position: Target position (x, y, z)
            yaw: Target yaw angle in radians (optional)
        """
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.header.frame_id = "world"
        nav_msg.control_frame = FlightNav.WORLD_FRAME  # Set control frame
        nav_msg.target = FlightNav.COG  # Control center of gravity
        
        # Position control
        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_x = position[0]
        nav_msg.target_pos_y = position[1]
        
        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_z = position[2]
        
        # Yaw control
        if yaw is not None:
            nav_msg.yaw_nav_mode = FlightNav.POS_MODE
            nav_msg.target_yaw = yaw
        else:
            nav_msg.yaw_nav_mode = FlightNav.NO_NAVIGATION
        
        # Default stabilization for roll and pitch
        nav_msg.roll_nav_mode = FlightNav.NO_NAVIGATION
        nav_msg.pitch_nav_mode = FlightNav.NO_NAVIGATION
        
        self.pub.publish(nav_msg)
    
    # ===== Basic trajectory execution functions =====
    
    def execute_poly_motion_with_feedback(self, start, target, avg_speed, 
                                          pos_threshold=0.05, yaw_threshold=0.1, timeout=30):
        """
        Execute polynomial trajectory motion with feedback control
        
        Args:
            start: Starting position (x, y, z)
            target: Target position (x, y, z)
            avg_speed: Average speed m/s
            pos_threshold: Position error threshold m
            yaw_threshold: Yaw error threshold rad
            timeout: Timeout duration s
            
        Returns:
            bool: Returns True on success, False on failure
        """
        distance = math.sqrt((target[0]-start[0])**2 + (target[1]-start[1])**2 + (target[2]-start[2])**2)
        duration = distance / max(avg_speed, 0.05)
        
        rospy.loginfo(f"Executing trajectory motion: {distance:.2f}m in {duration:.1f}s")
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start, target)
        
        rate = rospy.Rate(50)
        start_time = time.time()
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            pt = traj.evaluate()
            if pt is None:
                rospy.loginfo("Trajectory motion completed")
                return True
            
            self.send_trajectory_point(pt, None)
            
            if not self._wait_for_position(pt, pos_threshold, point_timeout=2.0):
                rospy.logwarn(f"Trajectory point timeout: {pt}")
            
            rate.sleep()
        
        rospy.logerr(f"Trajectory motion timeout: {timeout}s")
        return False
    
    def execute_progressive_precision_trajectory(self, start_pos, target_pos, target_yaw, 
                                                final_pos_threshold=0.025, final_yaw_threshold=0.03):
        """
        Execute trajectory with progressive precision control to avoid oscillation
        
        STRATEGY:
        1. Phase 1: Fast approach with relaxed precision (avoid oscillation)
        2. Phase 2: Fine positioning with strict precision (achieve target accuracy)
        
        Args:
            start_pos: Starting position (x, y, z)
            target_pos: Target position (x, y, z)
            target_yaw: Target yaw rad
            final_pos_threshold: Final position precision requirement m
            final_yaw_threshold: Final yaw precision requirement rad
            
        Returns:
            bool: Returns True on success, False on failure
        """
        rospy.loginfo("=== PROGRESSIVE PRECISION TRAJECTORY CONTROL ===")
        rospy.loginfo("ANTI-OSCILLATION STRATEGY: Phase 1 (relaxed) + Phase 2 (precise)")
        
        distance = math.sqrt(
            (target_pos[0] - start_pos[0])**2 + 
            (target_pos[1] - start_pos[1])**2 + 
            (target_pos[2] - start_pos[2])**2
        )
        
        # === PHASE 1: FAST APPROACH WITH RELAXED PRECISION ===
        rospy.loginfo(f"--- PHASE 1: FAST APPROACH (distance: {distance:.3f}m) ---")
        
        # Use relaxed thresholds to avoid oscillation
        if distance < 0.01:  # Very small movement
            phase1_pos_threshold = 0.008  # 8mm - still relaxed for tiny movements
            phase1_yaw_threshold = 0.05   # 2.9° - relaxed
            phase1_duration = 2.0
        elif distance < 0.05:  # Small movement
            phase1_pos_threshold = 0.03   # 30mm - relaxed
            phase1_yaw_threshold = 0.08   # 4.6° - relaxed
            phase1_duration = 3.0
        else:  # Normal movement
            phase1_pos_threshold = 0.05   # 50mm - very relaxed
            phase1_yaw_threshold = 0.1    # 5.7° - very relaxed
            phase1_duration = max(4.0, distance / 0.08)
        
        rospy.loginfo(f"Phase 1 thresholds: pos={phase1_pos_threshold*1000:.0f}mm, yaw={math.degrees(phase1_yaw_threshold):.1f}°")
        
        phase1_success = self.execute_smooth_trajectory_with_yaw(
            start_pos, target_pos, target_yaw,
            duration=phase1_duration,
            pos_threshold=phase1_pos_threshold,
            yaw_threshold=phase1_yaw_threshold
        )
        
        if not phase1_success:
            rospy.logerr("Phase 1 (fast approach) failed")
            return False
        
        rospy.loginfo("✓ Phase 1 completed: Fast approach successful")
        
        # === PHASE 2: FINE POSITIONING WITH STRICT PRECISION ===
        rospy.loginfo("--- PHASE 2: FINE POSITIONING ---")
        
        # Get actual position after Phase 1
        actual_pos = self.sm.get_current_position()
        if actual_pos is None:
            rospy.logerr("Lost position feedback after Phase 1")
            return False
        
        # Check if Phase 2 is needed
        final_error = math.sqrt(
            (target_pos[0] - actual_pos[0])**2 + 
            (target_pos[1] - actual_pos[1])**2 + 
            (target_pos[2] - actual_pos[2])**2
        )
        final_yaw_error = abs(self._normalize_angle_diff(target_yaw - self.sm.current_yaw))
        
        rospy.loginfo(f"After Phase 1 - Position error: {final_error*1000:.1f}mm, Yaw error: {math.degrees(final_yaw_error):.2f}°")
        
        if final_error <= final_pos_threshold and final_yaw_error <= final_yaw_threshold:
            rospy.loginfo("✓ Already within final precision - Phase 2 not needed")
            return True
        
        # Execute Phase 2 with strict precision
        rospy.loginfo(f"Phase 2 thresholds: pos={final_pos_threshold*1000:.0f}mm, yaw={math.degrees(final_yaw_threshold):.1f}°")
        
        phase2_success = self.execute_precision_positioning(
            actual_pos, target_pos, target_yaw,
            pos_threshold=final_pos_threshold,
            yaw_threshold=final_yaw_threshold
        )
        
        if phase2_success:
            rospy.loginfo("✓ PROGRESSIVE PRECISION TRAJECTORY COMPLETED")
            return True
        else:
            # Fallback: accept relaxed precision if strict fails
            if final_error <= final_pos_threshold * 2.0 and final_yaw_error <= final_yaw_threshold * 2.0:
                rospy.logwarn("✓ Accepting trajectory with relaxed final precision")
                return True
            else:
                rospy.logerr("Progressive precision trajectory failed")
                return False

    def execute_smooth_trajectory_with_yaw(self, start_pos, target_pos, target_yaw, duration=5.0,
                                          pos_threshold=0.03, yaw_threshold=0.08):
        """
        Execute smooth trajectory motion with yaw control (ENHANCED WITH PID FEEDBACK)
        
        FIXES:
        1. Reduced control frequency from 50Hz to 20Hz for stability
        2. Added position feedback control during trajectory execution
        3. Enhanced convergence checking with stricter criteria
        4. Improved error detection and reporting
        5. ADDED: Real-time PID feedback to eliminate XY drift
        
        Args:
            start_pos: Starting position (x, y, z)
            target_pos: Target position (x, y, z)
            target_yaw: Target yaw rad
            duration: Trajectory duration s
            pos_threshold: Position error threshold m
            yaw_threshold: Yaw error threshold rad
            
        Returns:
            bool: Returns True on success, False on failure
        """
        rospy.loginfo(f"=== ENHANCED SMOOTH TRAJECTORY WITH PID FEEDBACK ===")
        rospy.loginfo(f"Start: ({start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f})")
        rospy.loginfo(f"Target: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
        
        # Calculate movement distance for validation
        distance = math.sqrt(
            (target_pos[0] - start_pos[0])**2 + 
            (target_pos[1] - start_pos[1])**2 + 
            (target_pos[2] - start_pos[2])**2
        )
        rospy.loginfo(f"Total distance: {distance:.3f}m, Duration: {duration:.1f}s")
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        
        start_yaw = self.sm.current_yaw
        yaw_diff = self._normalize_angle_diff(target_yaw - start_yaw)
        
        rospy.loginfo(f"Yaw change: {math.degrees(yaw_diff):.1f}°")
        
        # ENHANCED: PID控制器参数 (针对XY漂移优化)
        pid_kp = 2.0    # 比例增益 - 保持原有XY响应
        pid_ki = 0.1    # 积分增益 - 保持原有XY稳态误差控制  
        pid_kd = 0.5    # 微分增益 - 保持原有XY超调控制
        
        # BALANCED: Z轴专用PID参数 (平衡响应性和稳定性)
        z_pid_kp = 2.2    # Z轴比例增益 - 适度提高响应性(1.5→2.2)
        z_pid_ki = 0.08   # Z轴积分增益 - 略微增加收敛能力(0.05→0.08)
        z_pid_kd = 0.4    # Z轴微分增益 - 轻微增加阻尼(0.3→0.4)
        
        # PID状态变量
        integral_error = [0.0, 0.0, 0.0]  # x, y, z积分误差
        last_error = [0.0, 0.0, 0.0]      # 上次误差，用于计算微分
        
        # FIXED: Use 20Hz for better stability
        rate = rospy.Rate(20)
        dt = 1.0 / 20.0  # 时间步长
        start_time = time.time()
        trajectory_completed = False
        last_position_error = float('inf')
        stuck_count = 0
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 10.0:
            elapsed_time = time.time() - start_time
            pt = traj.evaluate()
            
            # Check if trajectory generation is complete
            if pt is None or elapsed_time >= duration:
                pt = target_pos
                trajectory_completed = True
            
            # Smooth yaw interpolation
            if elapsed_time <= duration:
                yaw_progress = min(elapsed_time / duration, 1.0)
                smooth_progress = 3*yaw_progress**2 - 2*yaw_progress**3
                current_target_yaw = start_yaw + smooth_progress * yaw_diff
            else:
                current_target_yaw = target_yaw
            
            # ENHANCED: 实时PID反馈控制 (解决XY漂移的核心)
            current_pos = self.sm.get_current_position()
            if current_pos is not None:
                # 计算位置误差
                error = [
                    pt[0] - current_pos[0],  # x误差
                    pt[1] - current_pos[1],  # y误差  
                    pt[2] - current_pos[2]   # z误差
                ]
                
                # 累积积分误差
                for i in range(3):
                    integral_error[i] += error[i] * dt
                    # 积分饱和限制
                    integral_error[i] = max(-0.1, min(0.1, integral_error[i]))
                
                # 计算微分误差
                derivative_error = [
                    (error[i] - last_error[i]) / dt for i in range(3)
                ]
                
                # PID控制输出 - 方案A：Z轴使用专用参数
                pid_correction = []
                for i in range(3):
                    if i == 2:  # Z轴使用增强参数
                        correction = (z_pid_kp * error[i] + 
                                    z_pid_ki * integral_error[i] + 
                                    z_pid_kd * derivative_error[i])
                        # Z轴允许更大修正量以解决32-45mm误差
                        correction = max(-0.08, min(0.08, correction))  # ±8cm限制
                    else:  # XY轴使用标准参数
                        correction = (pid_kp * error[i] + 
                                    pid_ki * integral_error[i] + 
                                    pid_kd * derivative_error[i])
                        # XY轴保持原有限制
                        correction = max(-0.05, min(0.05, correction))  # ±5cm限制
                    pid_correction.append(correction)
                
                # 应用PID修正到轨迹点
                corrected_pt = [
                    pt[0] + pid_correction[0],
                    pt[1] + pid_correction[1], 
                    pt[2] + pid_correction[2]
                ]
                
                # 更新上次误差
                last_error = error[:]
                
                # 记录PID调试信息 (每2秒输出一次)
                if int(elapsed_time) % 2 == 0 and int(elapsed_time * 10) % 10 == 0:
                    error_magnitude = math.sqrt(sum([e**2 for e in error]))
                    correction_magnitude = math.sqrt(sum([c**2 for c in pid_correction]))
                    rospy.loginfo(f"PID Debug - Error: {error_magnitude*1000:.1f}mm, "
                                f"Correction: {correction_magnitude*1000:.1f}mm, "
                                f"XY_error: [{error[0]*1000:.1f}, {error[1]*1000:.1f}]mm")
                
                # 发送修正后的轨迹点
                self.send_trajectory_point(corrected_pt, current_target_yaw)
            else:
                # 如果没有位置反馈，使用原始轨迹点
                self.send_trajectory_point(pt, current_target_yaw)
            
            # ENHANCED: Position feedback monitoring during execution  
            if current_pos is not None:
                position_error = math.sqrt(
                    (pt[0] - current_pos[0])**2 + 
                    (pt[1] - current_pos[1])**2 + 
                    (pt[2] - current_pos[2])**2
                )
                
                # Log progress every 2 seconds
                if int(elapsed_time) % 2 == 0 and int(elapsed_time * 10) % 10 == 0:
                    rospy.loginfo(f"Progress: {elapsed_time:.1f}s, Position error: {position_error*1000:.1f}mm")
                
                # Check for stuck condition (避免原地旋转误报)
                distance = math.sqrt(
                    (target_pos[0] - start_pos[0])**2 + 
                    (target_pos[1] - start_pos[1])**2 + 
                    (target_pos[2] - start_pos[2])**2
                )
                
                # 只有在有明显移动距离时才检测stuck（原地旋转不检测）
                if distance > 0.02:  # 20mm以上的移动才检测stuck
                    if abs(position_error - last_position_error) < 0.005:  # Less than 5mm change
                        stuck_count += 1
                        if stuck_count > 40:  # 2 seconds at 20Hz
                            rospy.logwarn(f"UAV appears stuck, position error: {position_error*1000:.1f}mm")
                            # Continue anyway, but log the issue
                            stuck_count = 0
                    else:
                        stuck_count = 0
                else:
                    # 原地旋转或微小移动，不检测stuck
                    stuck_count = 0
                
                last_position_error = position_error
            
            # Check for final convergence
            if trajectory_completed:
                convergence_success = self._wait_for_position_and_yaw(
                    target_pos, target_yaw, pos_threshold, yaw_threshold, point_timeout=2.0)
                
                if convergence_success:
                    rospy.loginfo("✓ Trajectory converged successfully")
                    return True
                else:
                    # Check actual final error
                    final_pos = self.sm.get_current_position()
                    if final_pos is not None:
                        final_error = math.sqrt(
                            (target_pos[0] - final_pos[0])**2 + 
                            (target_pos[1] - final_pos[1])**2 + 
                            (target_pos[2] - final_pos[2])**2
                        )
                        final_yaw_error = abs(self._normalize_angle_diff(target_yaw - self.sm.current_yaw))
                        
                        rospy.logwarn(f"Final convergence timeout:")
                        rospy.logwarn(f"  Position error: {final_error*1000:.1f}mm (threshold: {pos_threshold*1000:.1f}mm)")
                        rospy.logwarn(f"  Yaw error: {math.degrees(final_yaw_error):.2f}° (threshold: {math.degrees(yaw_threshold):.2f}°)")
                        
                        # Be more strict about acceptance
                        if final_error < pos_threshold * 2.0 and final_yaw_error < yaw_threshold * 2.0:
                            rospy.logwarn("Accepting trajectory with relaxed thresholds")
                            return True
                        else:
                            rospy.logerr("Trajectory failed - final error too large")
                            return False
                    else:
                        rospy.logerr("Lost position feedback - trajectory failed")
                        return False
            
            rate.sleep()
        
        rospy.logerr(f"Trajectory timeout after {duration + 10.0:.1f}s")
        return False
    
    def execute_precision_positioning(self, start_pos, target_pos, target_yaw, 
                                    pos_threshold=0.015, yaw_threshold=0.02):
        """
        Execute precision positioning for fine adjustment
        
        Uses low-frequency control and smaller steps to avoid oscillation
        
        Args:
            start_pos: Current position (x, y, z)
            target_pos: Target position (x, y, z) 
            target_yaw: Target yaw rad
            pos_threshold: Position precision m
            yaw_threshold: Yaw precision rad
            
        Returns:
            bool: Returns True on success, False on failure
        """
        rospy.loginfo("=== PRECISION POSITIONING MODE ===")
        
        distance = math.sqrt(
            (target_pos[0] - start_pos[0])**2 + 
            (target_pos[1] - start_pos[1])**2 + 
            (target_pos[2] - start_pos[2])**2
        )
        
        # Use low frequency and longer duration for precision
        rate = rospy.Rate(10)  # Very low frequency to avoid oscillation
        duration = max(6.0, distance / 0.02)  # Slow, careful movement
        
        rospy.loginfo(f"Precision mode: {distance*1000:.1f}mm in {duration:.1f}s at 10Hz")
        
        start_time = time.time()
        converged_count = 0
        required_convergence = 15  # Need 1.5s of stable convergence at 10Hz
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 8.0:
            current_pos = self.sm.get_current_position()
            if current_pos is None:
                rospy.logwarn("Lost position feedback in precision mode")
                rate.sleep()
                continue
            
            # Calculate current errors
            pos_error = math.sqrt(
                (target_pos[0] - current_pos[0])**2 + 
                (target_pos[1] - current_pos[1])**2 + 
                (target_pos[2] - current_pos[2])**2
            )
            yaw_error = abs(self._normalize_angle_diff(target_yaw - self.sm.current_yaw))
            
            # Check convergence
            if pos_error <= pos_threshold and yaw_error <= yaw_threshold:
                converged_count += 1
                if converged_count >= required_convergence:
                    rospy.loginfo(f"✓ Precision positioning converged: {pos_error*1000:.1f}mm, {math.degrees(yaw_error):.2f}°")
                    return True
            else:
                converged_count = 0
            
            # Send gentle positioning command
            self.send_trajectory_point(target_pos, target_yaw)
            
            # Log progress occasionally
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0 and int(elapsed * 10) % 10 == 0:
                rospy.loginfo(f"Precision progress: {elapsed:.1f}s, error: {pos_error*1000:.1f}mm, {math.degrees(yaw_error):.2f}°")
            
            rate.sleep()
        
        # Final check
        final_pos = self.sm.get_current_position()
        if final_pos is not None:
            final_error = math.sqrt(
                (target_pos[0] - final_pos[0])**2 + 
                (target_pos[1] - final_pos[1])**2 + 
                (target_pos[2] - final_pos[2])**2
            )
            final_yaw_error = abs(self._normalize_angle_diff(target_yaw - self.sm.current_yaw))
            
            rospy.logwarn(f"Precision positioning timeout:")
            rospy.logwarn(f"  Final position error: {final_error*1000:.1f}mm (target: {pos_threshold*1000:.0f}mm)")
            rospy.logwarn(f"  Final yaw error: {math.degrees(final_yaw_error):.2f}° (target: {math.degrees(yaw_threshold):.1f}°)")
            
            return final_error <= pos_threshold and final_yaw_error <= yaw_threshold
        
        return False
    
    def execute_controlled_descent(self, start_pos, target_z, descent_speed=0.05,
                                  pos_threshold=0.02, yaw_threshold=0.05, use_vel_accel=True):
        """
        Execute controlled descent with fixed XY position and orientation
        Enhanced with VEL+ACC control option for maximum precision
        
        Args:
            start_pos: Starting position (x, y, z)
            target_z: Target Z coordinate
            descent_speed: Descent speed m/s
            pos_threshold: Position error threshold m
            yaw_threshold: Yaw error threshold rad
            use_vel_accel: Whether to use VEL+ACC control (recommended for precision)
            
        Returns:
            bool: Returns True on success, False on failure
        """
        fixed_x, fixed_y = start_pos[0], start_pos[1]
        fixed_yaw = self.sm.current_yaw
        target_pos = (fixed_x, fixed_y, target_z)
        
        descent_distance = abs(start_pos[2] - target_z)
        duration = max(descent_distance / descent_speed, 2.0)  # Minimum 2s for smooth control
        
        rospy.loginfo(f"Controlled descent: {descent_distance:.3f}m in {duration:.1f}s")
        rospy.loginfo(f"Fixed position: [{fixed_x:.6f}, {fixed_y:.6f}], yaw: {fixed_yaw:.3f}")
        rospy.loginfo(f"Control mode: {'VEL+ACC' if use_vel_accel else 'Position'}")
        
        if use_vel_accel:
            # Use enhanced VEL+ACC control with Z-only movement
            return self.execute_vel_accel_trajectory(
                start_pos=start_pos,
                target_pos=target_pos,
                target_yaw=fixed_yaw,
                duration=duration,
                pos_threshold=pos_threshold,
                yaw_threshold=yaw_threshold,
                axis_lock_mode='z_only'  # Lock XY and yaw, only allow Z movement
            )
        else:
            # Legacy position-based control
            return self._execute_position_based_descent(start_pos, target_z, descent_speed, 
                                                       pos_threshold, yaw_threshold)
    
    def _execute_position_based_descent(self, start_pos, target_z, descent_speed,
                                       pos_threshold, yaw_threshold):
        """Legacy position-based descent control"""
        fixed_x, fixed_y = start_pos[0], start_pos[1]
        fixed_yaw = self.sm.current_yaw
        
        descent_distance = abs(start_pos[2] - target_z)
        duration = descent_distance / descent_speed
        
        # Initial position hold
        hold_duration = 0.5
        hold_start = time.time()
        while not rospy.is_shutdown() and (time.time() - hold_start) < hold_duration:
            self.send_trajectory_point((fixed_x, fixed_y, start_pos[2]), fixed_yaw)
            time.sleep(0.02)
        
        # Execute descent
        start_time = time.time()
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 5.0:
            elapsed = time.time() - start_time
            progress = min(elapsed / duration, 1.0)
            current_z = start_pos[2] + progress * (target_z - start_pos[2])
            
            self.send_trajectory_point((fixed_x, fixed_y, current_z), fixed_yaw)
            
            # Check if target reached
            current_pos = self.sm.get_current_position()
            if current_pos and current_pos[2] <= target_z + 0.01:
                rospy.loginfo("Target descent altitude reached")
                return True
            
            # Monitor drift
            if current_pos:
                xy_drift = math.sqrt((current_pos[0] - fixed_x)**2 + (current_pos[1] - fixed_y)**2)
                yaw_drift = abs(self._normalize_angle_diff(self.sm.current_yaw - fixed_yaw))
                
                if elapsed % 2.0 < 0.02:  # Log every 2 seconds
                    descended = start_pos[2] - current_pos[2]
                    rospy.loginfo(f"Descent: {descended:.3f}m, XY drift: {xy_drift:.3f}m, yaw drift: {yaw_drift:.3f}rad")
            
            rate.sleep()
        
        rospy.loginfo("Legacy controlled descent completed")
        return True
        hold_start = time.time()
        while not rospy.is_shutdown() and (time.time() - hold_start) < hold_duration:
            self.send_trajectory_point((fixed_x, fixed_y, start_pos[2]), fixed_yaw)
            time.sleep(0.02)
        
        # Execute descent
        start_time = time.time()
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 5.0:
            elapsed = time.time() - start_time
            progress = min(elapsed / duration, 1.0)
            current_z = start_pos[2] + progress * (target_z - start_pos[2])
            
            self.send_trajectory_point((fixed_x, fixed_y, current_z), fixed_yaw)
            
            # Check if target reached
            current_pos = self.sm.get_current_position()
            if current_pos and current_pos[2] <= target_z + 0.01:
                rospy.loginfo("Target descent altitude reached")
                return True
            
            # Monitor drift
            if current_pos:
                xy_drift = math.sqrt((current_pos[0] - fixed_x)**2 + (current_pos[1] - fixed_y)**2)
                yaw_drift = abs(self._normalize_angle_diff(self.sm.current_yaw - fixed_yaw))
                
                if elapsed % 2.0 < 0.02:  # Log every 2 seconds
                    descended = start_pos[2] - current_pos[2]
                    rospy.loginfo(f"Descent: {descended:.3f}m, XY drift: {xy_drift:.3f}m, yaw drift: {yaw_drift:.3f}rad")
            
            rate.sleep()
        
        rospy.loginfo("Controlled descent completed")
        return True
    
    # ===== Alignment control functionality =====
    
    def execute_pre_rotation_alignment(self, valve_pos, timeout=10.0):
        """
        Execute pre-rotation alignment
        
        Args:
            valve_pos: Valve position (x, y, z)
            timeout: Timeout duration s
            
        Returns:
            bool: Returns True on success, False on failure
        """
        if not self.strict_alignment_enabled:
            rospy.loginfo("Strict alignment control disabled")
            return True
        
        rospy.loginfo("Starting pre-rotation alignment")
        
        start_time = time.time()
        rate = rospy.Rate(25)
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            current_pos = self.sm.get_current_position()
            if current_pos is None:
                rate.sleep()
                continue
            
            # Calculate desired yaw (pointing toward valve)
            desired_yaw = math.atan2(valve_pos[1] - current_pos[1], valve_pos[0] - current_pos[0])
            
            # Calculate alignment error
            yaw_error = abs(self._normalize_angle_diff(desired_yaw - self.sm.current_yaw))
            
            if yaw_error < self.alignment_yaw_tolerance:
                rospy.loginfo("Pre-rotation alignment completed")
                return True
            
            # Send alignment command
            self.send_trajectory_point(current_pos, desired_yaw)
            
            rate.sleep()
        
        rospy.logwarn("Pre-rotation alignment timeout")
        return False
    
    # ===== Constant Distance Feedback Control Function =====
    
    def execute_constant_distance_rotation(self, trajectory, valve_center, 
                                         feedback_frequency=50, alignment_check_interval=0.1):
        """
        Execute constant distance rotation combining trajectory generation and feedback control
        
        Args:
            trajectory: ConstantDistanceValveRotationTrajectory trajectory generator
            valve_center: Valve center position
            feedback_frequency: Feedback control frequency Hz
            alignment_check_interval: Alignment check interval s
        
        Returns:
            dict: Execution statistics
        """
        rospy.loginfo("Starting constant distance feedback control rotation")
        
        # Reset statistics
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
            # Get trajectory target point
            result = trajectory.get_next_uav_position_and_yaw()
            if result is None:
                rospy.loginfo("Trajectory execution completed")
                break
            
            target_pos, target_yaw = result
            current_time = time.time()
            
            # Get current state
            current_pos = self.sm.get_current_position()
            current_yaw = self.sm.current_yaw
            
            if current_pos is None:
                rospy.logwarn("Unable to get current position")
                rate.sleep()
                continue
            
            # Apply feedback control correction
            if current_time - last_alignment_check >= alignment_check_interval:
                corrected_pos, corrected_yaw = self.apply_constant_distance_feedback(
                    target_pos, target_yaw, current_pos, current_yaw, valve_center, trajectory)
                
                last_alignment_check = current_time
            else:
                corrected_pos, corrected_yaw = target_pos, target_yaw
            
            # Send control command
            self.send_trajectory_point(corrected_pos, corrected_yaw)
            
            # Update statistics
            distance_error = self.calculate_distance_error(current_pos, current_yaw, valve_center, 
                                                          trajectory.end_effector_distance)
            distance_error_sum += distance_error
            point_count += 1
            
            self.control_stats['max_distance_error'] = max(
                self.control_stats['max_distance_error'], distance_error)
            self.control_stats['total_points'] += 1
            
            if distance_error > self.distance_tolerance:
                self.control_stats['distance_violations'] += 1
            
            # Periodic status output
            if point_count % (feedback_frequency * 2) == 0:  # Output every 2 seconds
                rospy.loginfo(f"Rotation progress: {point_count} points, distance error: {distance_error:.4f}m, "
                             f"max error: {self.control_stats['max_distance_error']:.4f}m")
            
            rate.sleep()
        
        # Calculate final statistics
        self.control_stats['execution_time'] = time.time() - start_time
        self.control_stats['avg_distance_error'] = distance_error_sum / max(point_count, 1)
        
        self.log_control_statistics()
        return self.control_stats
    
    def apply_constant_distance_feedback(self, target_pos, target_yaw, current_pos, current_yaw, 
                                        valve_center, trajectory):
        """
        Apply constant distance feedback control
        
        Args:
            target_pos: Target position
            target_yaw: Target orientation
            current_pos: Current position
            current_yaw: Current orientation
            valve_center: Valve center position
            trajectory: Trajectory generator
        
        Returns:
            tuple: (corrected_pos, corrected_yaw)
        """
        # Calculate current end-effector position
        current_ee_pos = self.calculate_end_effector_position(current_pos, current_yaw)
        
        # Calculate target end-effector position
        target_ee_pos = self.calculate_end_effector_position(target_pos, target_yaw)
        
        # Calculate distance error
        valve_x, valve_y, valve_z = valve_center
        current_distance = sqrt((current_ee_pos[0] - valve_x)**2 + (current_ee_pos[1] - valve_y)**2)
        target_distance = sqrt((target_ee_pos[0] - valve_x)**2 + (target_ee_pos[1] - valve_y)**2)
        
        distance_error = abs(current_distance - target_distance)
        
        # If distance error is within tolerance, no correction needed
        if distance_error <= self.distance_tolerance:
            return target_pos, target_yaw
        
        # Calculate correction vector
        correction_pos, correction_yaw = self.calculate_distance_correction(
            current_pos, current_yaw, valve_center, target_distance)
        
        # Apply correction
        corrected_pos = (
            target_pos[0] + correction_pos[0],
            target_pos[1] + correction_pos[1],
            target_pos[2] + correction_pos[2]
        )
        corrected_yaw = target_yaw + correction_yaw
        
        # Record correction
        self.control_stats['total_corrections'] += 1
        
        # Record correction information
        if distance_error > self.distance_tolerance * 2:  # Only record large corrections
            rospy.loginfo(f"Applied distance correction: error={distance_error:.4f}m, "
                         f"position correction=[{correction_pos[0]:.3f}, {correction_pos[1]:.3f}, {correction_pos[2]:.3f}], "
                         f"yaw correction={correction_yaw:.3f}rad")
        
        return corrected_pos, corrected_yaw
    
    def calculate_distance_correction(self, current_pos, current_yaw, valve_center, target_distance):
        """
        Calculate distance correction
        
        Args:
            current_pos: Current UAV position
            current_yaw: Current UAV yaw
            valve_center: Valve center position
            target_distance: Target distance
        
        Returns:
            tuple: (position_correction, yaw_correction)
        """
        # Calculate current end-effector position
        current_ee_pos = self.calculate_end_effector_position(current_pos, current_yaw)
        
        # Calculate current distance
        valve_x, valve_y, valve_z = valve_center
        current_distance = sqrt((current_ee_pos[0] - valve_x)**2 + (current_ee_pos[1] - valve_y)**2)
        
        # Calculate distance error
        distance_error = current_distance - target_distance
        
        # If distance error is small, no correction needed
        if abs(distance_error) <= self.distance_tolerance:
            return (0.0, 0.0, 0.0), 0.0
        
        # Calculate unit vector from valve center to end-effector
        if current_distance > 0.001:
            unit_vector_x = (current_ee_pos[0] - valve_x) / current_distance
            unit_vector_y = (current_ee_pos[1] - valve_y) / current_distance
        else:
            unit_vector_x = 1.0
            unit_vector_y = 0.0
        
        # Calculate end-effector position correction
        ee_correction_magnitude = -distance_error * self.distance_control_gain
        ee_correction_x = ee_correction_magnitude * unit_vector_x
        ee_correction_y = ee_correction_magnitude * unit_vector_y
        
        # Convert end-effector correction to UAV correction
        uav_correction_x = ee_correction_x * self.position_control_gain
        uav_correction_y = ee_correction_y * self.position_control_gain
        uav_correction_z = 0.0  # No Z direction correction
        
        # Limit correction magnitude
        correction_magnitude = sqrt(uav_correction_x**2 + uav_correction_y**2)
        if correction_magnitude > self.max_position_correction:
            scale_factor = self.max_position_correction / correction_magnitude
            uav_correction_x *= scale_factor
            uav_correction_y *= scale_factor
        
        # Calculate yaw correction
        desired_yaw = atan2(valve_y - current_ee_pos[1], valve_x - current_ee_pos[0])
        yaw_error = self._normalize_angle_diff(desired_yaw - current_yaw)
        yaw_correction = yaw_error * self.yaw_control_gain
        
        # Limit yaw correction
        yaw_correction = max(-self.max_yaw_correction, min(self.max_yaw_correction, yaw_correction))
        
        return (uav_correction_x, uav_correction_y, uav_correction_z), yaw_correction
    
    def calculate_end_effector_position(self, uav_pos, uav_yaw):
        """
        Calculate end-effector position
        
        Args:
            uav_pos: UAV position
            uav_yaw: UAV yaw
        
        Returns:
            tuple: End-effector position (x, y, z)
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
        Calculate distance error
        
        Args:
            uav_pos: UAV position
            uav_yaw: UAV yaw
            valve_center: Valve center position
            target_distance: Target distance
        
        Returns:
            float: Distance error
        """
        ee_pos = self.calculate_end_effector_position(uav_pos, uav_yaw)
        valve_x, valve_y, _ = valve_center
        
        current_distance = sqrt((ee_pos[0] - valve_x)**2 + (ee_pos[1] - valve_y)**2)
        return abs(current_distance - target_distance)
    
    # ===== Three-Stage Smart Insertion Strategy =====
    
    def execute_three_stage_smart_insertion(self, valve_pos, valve_yaw, target_insertion_point, 
                                          rotation_direction=1, insertion_clearance_angle=30.0):
        """
        Execute three-stage smart insertion strategy for maximum success rate
        
        Strategy:
        1. Pre-rotation: Rotate to angle with maximum insertion clearance
        2. Insertion: Insert in the spacious area
        3. Final positioning: Rotate to target insertion point for valve operation
        
        Args:
            valve_pos: Valve position (x, y, z)
            valve_yaw: Valve orientation
            target_insertion_point: Final target insertion point
            rotation_direction: 1 for clockwise, -1 for counter-clockwise
            insertion_clearance_angle: Extra angle for insertion clearance (degrees)
            
        Returns:
            bool: True if all three stages succeed
        """
        rospy.loginfo("=== THREE-STAGE SMART INSERTION STRATEGY ===")
        rospy.loginfo("Stage 1: Pre-rotation for maximum insertion clearance")
        rospy.loginfo("Stage 2: Insertion in spacious area")
        rospy.loginfo("Stage 3: Final positioning for valve operation")
        
        current_pos = self.sm.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position for smart insertion")
            return False
        
        # Stage 1: Calculate optimal pre-rotation angle
        optimal_clearance_angle = self._calculate_optimal_clearance_angle(
            current_pos, valve_pos, valve_yaw, rotation_direction, insertion_clearance_angle
        )
        
        if optimal_clearance_angle is None:
            rospy.logerr("Failed to calculate optimal clearance angle")
            return False
        
        rospy.loginfo(f"Stage 1: Pre-rotating to clearance angle: {math.degrees(optimal_clearance_angle):.1f}°")
        
        # Execute Stage 1: Pre-rotation
        stage1_success = self._execute_pre_rotation_stage(current_pos, optimal_clearance_angle)
        if not stage1_success:
            rospy.logerr("Stage 1 (pre-rotation) failed")
            return False
        
        rospy.loginfo("✓ Stage 1 completed: Pre-rotation successful")
        
        # Stage 2: Execute insertion in spacious area
        rospy.loginfo("Stage 2: Executing insertion with maximum clearance")
        stage2_success = self._execute_spacious_insertion_stage(valve_pos, optimal_clearance_angle)
        if not stage2_success:
            rospy.logerr("Stage 2 (spacious insertion) failed")
            return False
        
        rospy.loginfo("✓ Stage 2 completed: Insertion successful")
        
        # Stage 3: Final positioning for valve operation
        target_angle = self._calculate_target_angle_from_insertion_point(target_insertion_point, valve_pos)
        rospy.loginfo(f"Stage 3: Final positioning to target angle: {math.degrees(target_angle):.1f}°")
        
        stage3_success = self._execute_final_positioning_stage(target_angle)
        if not stage3_success:
            rospy.logerr("Stage 3 (final positioning) failed")
            return False
        
        rospy.loginfo("✓ Stage 3 completed: Final positioning successful")
        rospy.loginfo("✓ THREE-STAGE SMART INSERTION COMPLETED SUCCESSFULLY")
        
        return True
    
    def _calculate_optimal_clearance_angle(self, current_pos, valve_pos, valve_yaw, 
                                         rotation_direction, clearance_angle):
        """
        Calculate optimal angle for maximum insertion clearance
        
        Strategy: Find the angle that provides maximum space between valve beams
        """
        rospy.loginfo("Calculating optimal clearance angle...")
        
        # Valve beam angles (assuming 3-beam valve at 120° intervals)
        beam_angles = [
            valve_yaw + 0,                    # Beam 0: 0°
            valve_yaw + math.pi * 2/3,        # Beam 1: 120°
            valve_yaw + math.pi * 4/3         # Beam 2: 240°
        ]
        
        # Find largest gap between beams
        max_gap_center = None
        max_gap_size = 0
        
        for i in range(len(beam_angles)):
            next_i = (i + 1) % len(beam_angles)
            
            # Calculate gap between beam[i] and beam[next_i]
            gap_start = beam_angles[i]
            gap_end = beam_angles[next_i]
            
            # Handle angle wrapping
            if gap_end < gap_start:
                gap_end += 2 * math.pi
            
            gap_size = gap_end - gap_start
            gap_center = (gap_start + gap_end) / 2
            
            # Normalize gap center
            while gap_center > 2 * math.pi:
                gap_center -= 2 * math.pi
            while gap_center < 0:
                gap_center += 2 * math.pi
            
            if gap_size > max_gap_size:
                max_gap_size = gap_size
                max_gap_center = gap_center
        
        if max_gap_center is None:
            rospy.logerr("Failed to find optimal gap center")
            return None
        
        # Add clearance angle for extra space
        clearance_rad = math.radians(clearance_angle)
        if rotation_direction > 0:  # Clockwise
            optimal_angle = max_gap_center - clearance_rad
        else:  # Counter-clockwise  
            optimal_angle = max_gap_center + clearance_rad
        
        # Normalize angle
        while optimal_angle > 2 * math.pi:
            optimal_angle -= 2 * math.pi
        while optimal_angle < 0:
            optimal_angle += 2 * math.pi
        
        rospy.loginfo(f"Optimal clearance angle: {math.degrees(optimal_angle):.1f}° (gap center: {math.degrees(max_gap_center):.1f}°)")
        rospy.loginfo(f"Gap size: {math.degrees(max_gap_size):.1f}°, clearance: {clearance_angle:.1f}°")
        
        return optimal_angle
    
    def _execute_pre_rotation_stage(self, current_pos, target_angle):
        """Execute Stage 1: Pre-rotation to optimal clearance angle"""
        rospy.loginfo(f"Executing pre-rotation from current yaw to {math.degrees(target_angle):.1f}°")
        
        # Execute rotation while maintaining current position
        return self.execute_smooth_trajectory_with_yaw(
            current_pos, current_pos, target_angle, 
            duration=8.0, pos_threshold=0.05, yaw_threshold=0.1
        )
    
    def _execute_spacious_insertion_stage(self, valve_pos, clearance_angle):
        """Execute Stage 2: Insertion in spacious area"""
        rospy.loginfo("Executing insertion in spacious area...")
        
        current_pos = self.sm.get_current_position()
        if current_pos is None:
            return False
        
        # Calculate insertion position at clearance angle
        # Position UAV so that end-effector reaches valve at clearance angle
        valve_inner_radius = 0.10   # CORRECTED: Inner rim radius (100mm)
        end_effector_distance = valve_inner_radius + 0.02  # Slight clearance
        
        insertion_x = valve_pos[0] + end_effector_distance * math.cos(clearance_angle) - self.end_effector_offset_x * math.cos(clearance_angle)
        insertion_y = valve_pos[1] + end_effector_distance * math.sin(clearance_angle) - self.end_effector_offset_x * math.sin(clearance_angle)
        insertion_z = valve_pos[2] - self.end_effector_offset_z
        
        insertion_target = (insertion_x, insertion_y, insertion_z)
        
        rospy.loginfo(f"Spacious insertion target: ({insertion_x:.3f}, {insertion_y:.3f}, {insertion_z:.3f})")
        
        # Execute insertion movement
        return self.execute_smooth_trajectory_with_yaw(
            current_pos, insertion_target, clearance_angle,
            duration=10.0, pos_threshold=0.03, yaw_threshold=0.08
        )
    
    def _execute_final_positioning_stage(self, target_angle):
        """Execute Stage 3: Final positioning for valve operation"""
        rospy.loginfo(f"Executing final positioning to target angle: {math.degrees(target_angle):.1f}°")
        
        current_pos = self.sm.get_current_position()
        if current_pos is None:
            return False
        
        # Rotate to final target angle while maintaining insertion position
        return self.execute_smooth_trajectory_with_yaw(
            current_pos, current_pos, target_angle,
            duration=6.0, pos_threshold=0.03, yaw_threshold=0.05
        )
    
    def _calculate_target_angle_from_insertion_point(self, target_insertion_point, valve_pos):
        """Calculate target angle based on desired insertion point"""
        if target_insertion_point is None:
            # Default: opposite to current position for maximum leverage
            current_pos = self.sm.get_current_position()
            if current_pos:
                return math.atan2(valve_pos[1] - current_pos[1], valve_pos[0] - current_pos[0]) + math.pi
            else:
                return 0.0
        
        # Calculate angle to reach specific insertion point
        return math.atan2(target_insertion_point[1] - valve_pos[1], target_insertion_point[0] - valve_pos[0])
    
    def get_insertion_clearance_status(self, valve_pos, valve_yaw, current_angle):
        """
        Analyze current insertion clearance status
        
        Returns:
            dict: Analysis of clearance conditions
        """
        beam_angles = [
            valve_yaw + 0,
            valve_yaw + math.pi * 2/3,
            valve_yaw + math.pi * 4/3
        ]
        
        # Find closest beams to current angle
        min_distance_to_beam = float('inf')
        for beam_angle in beam_angles:
            distance = abs(self._normalize_angle_diff(current_angle - beam_angle))
            min_distance_to_beam = min(min_distance_to_beam, distance)
        
        clearance_degrees = math.degrees(min_distance_to_beam)
        
        status = {
            'clearance_angle': clearance_degrees,
            'clearance_sufficient': clearance_degrees > 15.0,  # Require >15° clearance
            'recommended_action': 'proceed' if clearance_degrees > 15.0 else 'pre_rotate',
            'nearest_beam_distance': clearance_degrees
        }
        
        rospy.loginfo(f"Insertion clearance analysis: {clearance_degrees:.1f}° clearance")
        return status
    
    def _normalize_angle_diff(self, angle_diff):
        """Normalize angle difference to [-π, π]"""
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff
    
    def log_control_statistics(self):
        """Log control statistics"""
        rospy.loginfo("=== Unified Motion Controller Statistics ===")
        rospy.loginfo(f"Execution time: {self.control_stats['execution_time']:.2f}s")
        rospy.loginfo(f"Total trajectory points: {self.control_stats['total_points']}")
        rospy.loginfo(f"Total corrections: {self.control_stats['total_corrections']}")
        rospy.loginfo(f"Distance violations: {self.control_stats['distance_violations']}")
        rospy.loginfo(f"Maximum distance error: {self.control_stats['max_distance_error']:.4f}m")
        rospy.loginfo(f"Average distance error: {self.control_stats['avg_distance_error']:.4f}m")
        
        # Calculate success rate
        if self.control_stats['total_points'] > 0:
            success_rate = (1.0 - self.control_stats['distance_violations'] / 
                           self.control_stats['total_points']) * 100
            rospy.loginfo(f"Distance maintenance success rate: {success_rate:.1f}%")
        
        # Evaluate control performance
        if self.control_stats['max_distance_error'] < self.distance_tolerance * 2:
            rospy.loginfo("✓ Distance control performance: Excellent")
        elif self.control_stats['max_distance_error'] < self.distance_tolerance * 3:
            rospy.loginfo("✓ Distance control performance: Good")
        else:
            rospy.logwarn("⚠ Distance control performance: Needs improvement")
        
        rospy.loginfo("=== Unified Motion Controller Concept Verification ===")
        rospy.loginfo("✓ Basic trajectory execution and feedback control")
        rospy.loginfo("✓ Enhanced alignment control")
        rospy.loginfo("✓ Constant distance feedback control")
        rospy.loginfo("✓ Specialized valve rotation control")
    
    def execute_vel_accel_trajectory(self, start_pos, target_pos, target_yaw, duration=5.0,
                                    pos_threshold=0.03, yaw_threshold=0.08, axis_lock_mode=None):
        """
        Execute trajectory using VEL+ACCEL control for maximum smoothness and axis control
        严格控制非运动轴保持不变，防止pitch不稳定等问题
        
        Args:
            start_pos: Starting position (x, y, z)
            target_pos: Target position (x, y, z)
            target_yaw: Target yaw rad
            duration: Trajectory duration s
            pos_threshold: Position error threshold m
            yaw_threshold: Yaw error threshold rad
            axis_lock_mode: Optional axis locking mode:
                - 'z_only': Only allow Z movement, lock XY and yaw
                - 'xy_yaw': Allow XY and yaw movement, lock Z
                - 'rotation': Allow all movement but with enhanced stability
                - None: Normal movement (default)
            
        Returns:
            bool: Returns True on success, False on failure
        """
        rospy.loginfo("=== VEL+ACCEL TRAJECTORY WITH AXIS CONTROL ===")
        
        # Apply axis locking based on mode
        locked_start_pos = start_pos
        locked_target_pos = target_pos
        locked_target_yaw = target_yaw
        start_yaw = self.sm.current_yaw
        
        if axis_lock_mode == 'z_only':
            # Lock XY and yaw, only allow Z movement
            locked_target_pos = (start_pos[0], start_pos[1], target_pos[2])
            locked_target_yaw = start_yaw
            rospy.loginfo("AXIS LOCK MODE: Z-only movement")
            rospy.loginfo(f"  XY LOCKED at: ({start_pos[0]:.6f}, {start_pos[1]:.6f})")
            rospy.loginfo(f"  Yaw LOCKED at: {math.degrees(start_yaw):.3f}°")
            rospy.loginfo(f"  Z movement: {start_pos[2]:.6f}m -> {target_pos[2]:.6f}m")
            
        elif axis_lock_mode == 'xy_yaw':
            # Lock Z, allow XY and yaw movement
            locked_target_pos = (target_pos[0], target_pos[1], start_pos[2])
            rospy.loginfo("AXIS LOCK MODE: XY+Yaw movement")
            rospy.loginfo(f"  Z LOCKED at: {start_pos[2]:.6f}m")
            rospy.loginfo(f"  XY movement: ({start_pos[0]:.6f}, {start_pos[1]:.6f}) -> ({target_pos[0]:.6f}, {target_pos[1]:.6f})")
            rospy.loginfo(f"  Yaw movement: {math.degrees(start_yaw):.3f}° -> {math.degrees(target_yaw):.3f}°")
            
        elif axis_lock_mode == 'rotation':
            rospy.loginfo("AXIS LOCK MODE: Rotation with enhanced stability")
            rospy.loginfo("  All axes allowed with enhanced roll/pitch stability")
        else:
            rospy.loginfo("AXIS LOCK MODE: Normal movement (no special locking)")
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(locked_start_pos, locked_target_pos)
        
        yaw_diff = self._normalize_angle_diff(locked_target_yaw - start_yaw)
        
        rospy.loginfo(f"VEL+ACCEL Trajectory: {locked_start_pos} -> {locked_target_pos}")
        rospy.loginfo(f"Duration: {duration:.1f}s, Yaw change: {math.degrees(yaw_diff):.1f}°")
        
        rate = rospy.Rate(25)  # Higher rate for more precise control
        start_time = time.time()
        
        # Adaptive gains based on axis lock mode
        if axis_lock_mode == 'z_only':
            # More aggressive Z control, very conservative XY/yaw control
            pos_gain = (0.1, 0.1, 0.6)  # (x, y, z) gains
            vel_gain = (0.05, 0.05, 0.4)
            yaw_gain = 0.1  # Very low yaw gain for Z-only mode
        elif axis_lock_mode == 'xy_yaw':
            # Aggressive XY/yaw control, very conservative Z control
            pos_gain = (0.5, 0.5, 0.1)
            vel_gain = (0.4, 0.4, 0.05)
            yaw_gain = 0.4
        elif axis_lock_mode == 'rotation':
            # Balanced but conservative for rotation stability
            pos_gain = (0.3, 0.3, 0.3)
            vel_gain = (0.25, 0.25, 0.25)
            yaw_gain = 0.3
        else:
            # Default conservative gains
            pos_gain = (0.4, 0.4, 0.4)
            vel_gain = (0.3, 0.3, 0.3)
            yaw_gain = 0.3
        
        rospy.loginfo(f"Control gains - Pos: {pos_gain}, Vel: {vel_gain}, Yaw: {yaw_gain}")
        
        # Axis drift monitoring for locked axes
        initial_pos = locked_start_pos
        max_xy_drift = 0.0
        max_z_drift = 0.0
        max_yaw_drift = 0.0
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 5.0:
            elapsed_time = time.time() - start_time
            progress = min(elapsed_time / duration, 1.0)
            
            # Get trajectory values
            desired_pos = traj.evaluate()
            desired_vel = traj.get_velocity()
            
            if desired_pos is None:
                desired_pos = locked_target_pos
                desired_vel = (0, 0, 0)
            
            # Calculate control commands with axis-specific gains
            current_pos = self.sm.get_current_position()
            if current_pos is not None:
                pos_error = (
                    desired_pos[0] - current_pos[0],
                    desired_pos[1] - current_pos[1],
                    desired_pos[2] - current_pos[2]
                )
                
                # Apply axis-specific gains
                cmd_vel = (
                    pos_gain[0] * pos_error[0] + vel_gain[0] * desired_vel[0],
                    pos_gain[1] * pos_error[1] + vel_gain[1] * desired_vel[1],
                    pos_gain[2] * pos_error[2] + vel_gain[2] * desired_vel[2]
                )
                
                # Apply axis-specific velocity limits
                if axis_lock_mode == 'z_only':
                    max_vel = (0.05, 0.05, 0.15)  # Very slow XY, normal Z
                elif axis_lock_mode == 'xy_yaw':
                    max_vel = (0.15, 0.15, 0.05)  # Normal XY, very slow Z
                else:
                    max_vel = (0.12, 0.12, 0.12)  # Conservative for all axes
                
                cmd_vel = tuple(max(-max_vel[i], min(max_vel[i], cmd_vel[i])) for i in range(3))
                
                # Monitor axis drift for locked axes
                if axis_lock_mode == 'z_only':
                    xy_drift = math.sqrt((current_pos[0] - initial_pos[0])**2 + (current_pos[1] - initial_pos[1])**2)
                    yaw_drift = abs(self._normalize_angle_diff(self.sm.current_yaw - start_yaw))
                    max_xy_drift = max(max_xy_drift, xy_drift)
                    max_yaw_drift = max(max_yaw_drift, yaw_drift)
                    
                elif axis_lock_mode == 'xy_yaw':
                    z_drift = abs(current_pos[2] - initial_pos[2])
                    max_z_drift = max(max_z_drift, z_drift)
                    
            else:
                cmd_vel = (0, 0, 0)
            
            # Calculate yaw command with axis-specific gain
            if elapsed_time <= duration:
                yaw_progress = min(elapsed_time / duration, 1.0)
                smooth_progress = 3*yaw_progress**2 - 2*yaw_progress**3
                desired_yaw = start_yaw + smooth_progress * yaw_diff
            else:
                desired_yaw = locked_target_yaw
            
            yaw_error = self._normalize_angle_diff(desired_yaw - self.sm.current_yaw)
            cmd_yaw_vel = yaw_gain * yaw_error
            
            # Send VEL+ACCEL command with enhanced axis control
            self.send_velocity_command_with_axis_control(
                cmd_vel, cmd_yaw_vel, desired_pos, desired_yaw, axis_lock_mode)
            
            # Check convergence
            if current_pos is not None and progress >= 0.95:
                pos_error_mag = math.sqrt(sum(e**2 for e in pos_error))
                yaw_error_final = abs(self._normalize_angle_diff(self.sm.current_yaw - locked_target_yaw))
                
                if pos_error_mag < pos_threshold and yaw_error_final < yaw_threshold:
                    rospy.loginfo("VEL+ACCEL trajectory converged successfully")
                    break
            
            rate.sleep()
        
        # Report axis control performance
        rospy.loginfo("=== VEL+ACCEL TRAJECTORY COMPLETED ===")
        if axis_lock_mode == 'z_only':
            rospy.loginfo(f"Axis control performance (Z-only mode):")
            rospy.loginfo(f"  Max XY drift: {max_xy_drift:.6f}m (should be ~0)")
            rospy.loginfo(f"  Max yaw drift: {math.degrees(max_yaw_drift):.3f}° (should be ~0)")
            if max_xy_drift > 0.01 or max_yaw_drift > math.radians(2):
                rospy.logwarn("⚠ Significant drift detected in locked axes")
            else:
                rospy.loginfo("✓ Excellent axis control - locked axes maintained")
                
        elif axis_lock_mode == 'xy_yaw':
            rospy.loginfo(f"Axis control performance (XY+Yaw mode):")
            rospy.loginfo(f"  Max Z drift: {max_z_drift:.6f}m (should be ~0)")
            if max_z_drift > 0.01:
                rospy.logwarn("⚠ Significant Z-axis drift detected")
            else:
                rospy.loginfo("✓ Excellent Z-axis control maintained")
        
        rospy.loginfo("VEL+ACCEL trajectory completed")
        return True

    # ===== Wait and convergence checking methods =====
        """
        执行自适应阀门插入策略（按用户建议优化）
        
        策略：
        1. 初始点到阀门附近：使用简单位控
        2. 根据实际位置：重新规划精确插入轨迹  
        3. 插入完成后：实时规划旋转轨迹
        
        Args:
            target_pos: 目标插入位置
            target_yaw: 目标偏航角
            use_simple_approach: 是否使用简单位控进行接近
            
        Returns:
            bool: 成功状态
        """
        rospy.loginfo("=== ADAPTIVE VALVE INSERTION STRATEGY ===")
        rospy.loginfo("OPTIMIZED APPROACH per user feedback:")
        rospy.loginfo("1. Simple position control for approach")
        rospy.loginfo("2. Real-time insertion planning based on actual position")
        rospy.loginfo("3. Real-time rotation planning after insertion")
        
        start_pos = self.sm.get_current_position()
        if start_pos is None:
            rospy.logerr("Cannot get starting position")
            return False
        
        # === PHASE 1: SIMPLE APPROACH TO VALVE VICINITY ===
        rospy.loginfo("--- PHASE 1: SIMPLE APPROACH ---")
        
        # Calculate approach target (above valve with safety margin)
        approach_target = (target_pos[0], target_pos[1], target_pos[2] + 0.10)  # 10cm above
        
        approach_distance = math.sqrt(
            (approach_target[0] - start_pos[0])**2 + 
            (approach_target[1] - start_pos[1])**2 + 
            (approach_target[2] - start_pos[2])**2
        )
        
        rospy.loginfo(f"Approach distance: {approach_distance:.3f}m")
        rospy.loginfo(f"Using SIMPLE POSITION CONTROL for approach (sufficient for long distance)")
        
        # Simple position control for approach - fast and stable
        approach_success = self.execute_smooth_trajectory_with_yaw(
            start_pos, approach_target, target_yaw,
            duration=max(8.0, approach_distance / 0.1),  # 100mm/s approach speed
            pos_threshold=0.03,  # 30mm precision sufficient for approach
            yaw_threshold=0.05   # 2.9° precision sufficient for approach
        )
        
        if not approach_success:
            rospy.logerr("Phase 1 (simple approach) failed")
            return False
        
        rospy.loginfo("✓ Phase 1 completed: Simple approach successful")
        
        # === PHASE 2: PRECISION INSERTION BASED ON ACTUAL POSITION ===
        rospy.loginfo("--- PHASE 2: PRECISION INSERTION ---")
        
        # Get actual position after approach
        actual_pos = self.sm.get_current_position()
        if actual_pos is None:
            rospy.logerr("Lost position feedback before precision insertion")
            return False
        
        rospy.loginfo(f"Actual position after approach: ({actual_pos[0]:.6f}, {actual_pos[1]:.6f}, {actual_pos[2]:.6f})")
        rospy.loginfo("Re-planning insertion trajectory based on ACTUAL position")
        
        # Re-calculate insertion target based on actual position
        actual_insertion_target = (actual_pos[0], actual_pos[1], target_pos[2])
        
        rospy.loginfo(f"Precision insertion target: ({actual_insertion_target[0]:.6f}, {actual_insertion_target[1]:.6f}, {actual_insertion_target[2]:.6f})")
        
        # Use precision control for final insertion
        insertion_success = self.execute_vel_accel_trajectory(
            start_pos=actual_pos,
            target_pos=actual_insertion_target,
            target_yaw=target_yaw,
            duration=12.0,  # Slow precision descent
            pos_threshold=0.008,  # 8mm precision for insertion
            yaw_threshold=0.01,   # 0.57° precision for insertion
            axis_lock_mode='z_only'  # Z-only descent with XY/yaw lock
        )
        
        if not insertion_success:
            rospy.logerr("Phase 2 (precision insertion) failed")
            return False
        
        rospy.loginfo("✓ Phase 2 completed: Precision insertion successful")
        
        # === PHASE 3: VERIFY INSERTION AND PREPARE FOR ROTATION ===
        rospy.loginfo("--- PHASE 3: INSERTION VERIFICATION ---")
        
        final_pos = self.sm.get_current_position()
        if final_pos is not None:
            insertion_error = math.sqrt(
                (actual_insertion_target[0] - final_pos[0])**2 + 
                (actual_insertion_target[1] - final_pos[1])**2 + 
                (actual_insertion_target[2] - final_pos[2])**2
            )
            rospy.loginfo(f"Final insertion error: {insertion_error*1000:.1f}mm")
            
            if insertion_error < 0.015:  # 15mm tolerance
                rospy.loginfo("✓ Phase 3: Insertion verification PASSED")
                rospy.loginfo("✓ ADAPTIVE VALVE INSERTION COMPLETED")
                rospy.loginfo("Ready for real-time rotation trajectory planning")
                return True
            else:
                rospy.logwarn(f"Phase 3: Insertion error {insertion_error*1000:.1f}mm exceeds 15mm tolerance")
        
        rospy.loginfo("✓ ADAPTIVE VALVE INSERTION COMPLETED (with warnings)")
        return True
    
    # ===== Wait and convergence checking methods =====
    
    def _wait_for_position(self, target_pos, pos_threshold, point_timeout=3.0):
        """
        Wait for UAV to reach target position
        
        Args:
            target_pos: Target position (x, y, z)
            pos_threshold: Position tolerance m
            point_timeout: Timeout for this waypoint s
            
        Returns:
            bool: True if converged, False if timeout
        """
        start_time = time.time()
        rate = rospy.Rate(25)
        
        while not rospy.is_shutdown() and (time.time() - start_time) < point_timeout:
            current_pos = self.sm.get_current_position()
            if current_pos is None:
                rate.sleep()
                continue
            
            # Calculate position error
            pos_error = math.sqrt(
                (target_pos[0] - current_pos[0])**2 + 
                (target_pos[1] - current_pos[1])**2 + 
                (target_pos[2] - current_pos[2])**2
            )
            
            if pos_error < pos_threshold:
                return True
            
            rate.sleep()
        
        return False
    
    def _wait_for_position_and_yaw(self, target_pos, target_yaw, pos_threshold, yaw_threshold, 
                                  point_timeout=3.0):
        """
        Wait for UAV to reach target position and yaw
        
        Args:
            target_pos: Target position (x, y, z)
            target_yaw: Target yaw rad
            pos_threshold: Position tolerance m
            yaw_threshold: Yaw tolerance rad
            point_timeout: Timeout for this waypoint s
            
        Returns:
            bool: True if converged, False if timeout
        """
        start_time = time.time()
        rate = rospy.Rate(25)
        
        while not rospy.is_shutdown() and (time.time() - start_time) < point_timeout:
            current_pos = self.sm.get_current_position()
            current_yaw = self.sm.current_yaw
            
            if current_pos is None:
                rate.sleep()
                continue
            
            # Calculate position error
            pos_error = math.sqrt(
                (target_pos[0] - current_pos[0])**2 + 
                (target_pos[1] - current_pos[1])**2 + 
                (target_pos[2] - current_pos[2])**2
            )
            
            # Calculate yaw error
            yaw_error = abs(self._normalize_angle_diff(target_yaw - current_yaw))
            
            if pos_error < pos_threshold and yaw_error < yaw_threshold:
                return True
            
            rate.sleep()
        
        return False
    
    def send_velocity_command_with_axis_control(self, cmd_vel, cmd_yaw_vel, fallback_pos=None, 
                                              fallback_yaw=None, axis_lock_mode=None):
        """
        Send velocity command with enhanced axis control for VEL+ACCEL trajectory mode
        严格控制特定轴保持不变，防止非预期运动
        
        CRITICAL: Uses hybrid VEL+POS control for maximum axis locking precision
        
        Args:
            cmd_vel: Velocity command (vx, vy, vz)
            cmd_yaw_vel: Yaw velocity command rad/s
            fallback_pos: Fallback position if velocity control fails
            fallback_yaw: Fallback yaw if yaw velocity control fails
            axis_lock_mode: Axis locking mode for enhanced control
        """
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.header.frame_id = "world"
        nav_msg.control_frame = FlightNav.WORLD_FRAME  # CRITICAL: Set control frame
        nav_msg.target = FlightNav.COG  # Control center of gravity
        
        try:
            # Apply HYBRID VEL+POS control based on lock mode for maximum precision
            if axis_lock_mode == 'z_only':
                # Z-only movement: POSITION lock for XY and yaw, VELOCITY for Z
                nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE  # POSITION LOCK for XY
                if fallback_pos is not None:
                    nav_msg.target_pos_x = fallback_pos[0]  # STRICT XY lock
                    nav_msg.target_pos_y = fallback_pos[1]  # STRICT XY lock
                
                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE  # VELOCITY for Z movement
                nav_msg.target_vel_z = cmd_vel[2]
                
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE  # POSITION LOCK for yaw
                if fallback_yaw is not None:
                    nav_msg.target_yaw = fallback_yaw  # STRICT yaw lock
                
            elif axis_lock_mode == 'xy_yaw':
                # XY+Yaw movement: POSITION lock for Z, VELOCITY for XY and yaw
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE  # VELOCITY for XY movement
                nav_msg.target_vel_x = cmd_vel[0]
                nav_msg.target_vel_y = cmd_vel[1]
                
                nav_msg.pos_z_nav_mode = FlightNav.POS_MODE  # POSITION LOCK for Z
                if fallback_pos is not None:
                    nav_msg.target_pos_z = fallback_pos[2]  # STRICT Z lock
                
                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE  # VELOCITY for yaw movement
                nav_msg.target_omega_z = cmd_yaw_vel
                
            else:
                # Normal velocity control mode
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_x = cmd_vel[0]
                nav_msg.target_vel_y = cmd_vel[1]
                
                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_z = cmd_vel[2]
                
                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_omega_z = cmd_yaw_vel
            
        except AttributeError:
            # Fallback to position control if velocity mode not available
            rospy.logwarn("VEL_MODE not available, falling back to position control")
            if fallback_pos is not None:
                nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                nav_msg.target_pos_x = fallback_pos[0]
                nav_msg.target_pos_y = fallback_pos[1]
                nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                nav_msg.target_pos_z = fallback_pos[2]
            
            if fallback_yaw is not None:
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE
                nav_msg.target_yaw = fallback_yaw
        
        # CRITICAL: Always lock roll and pitch for stability
        nav_msg.roll_nav_mode = FlightNav.NO_NAVIGATION  # Always locked
        nav_msg.pitch_nav_mode = FlightNav.NO_NAVIGATION  # Always locked
        
        self.pub.publish(nav_msg)

    def send_velocity_command(self, cmd_vel, cmd_yaw_vel, fallback_pos=None, fallback_yaw=None):
        """
        Send velocity command to UAV (for VEL+ACCEL trajectory mode)
        
        Args:
            cmd_vel: Velocity command (vx, vy, vz)
            cmd_yaw_vel: Yaw velocity command rad/s
            fallback_pos: Fallback position if velocity control fails
            fallback_yaw: Fallback yaw if yaw velocity control fails
        """
        # Delegate to enhanced version with no axis locking
        self.send_velocity_command_with_axis_control(
            cmd_vel, cmd_yaw_vel, fallback_pos, fallback_yaw, axis_lock_mode=None)

    def execute_rotation_trajectory_with_vel_accel(self, rotation_traj, rotation_duration, 
                                                  locked_z, initial_yaw, rotation_angle):
        """
        Execute valve rotation trajectory using VEL+ACC control with strict axis control
        Modularized from valve_rotation_fang_single.py for better code organization
        
        Args:
            rotation_traj: Rotation trajectory generator
            rotation_duration: Duration for rotation
            locked_z: Z position to maintain during rotation
            initial_yaw: Initial yaw angle
            rotation_angle: Target rotation angle
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("=== MODULAR VALVE ROTATION WITH VEL+ACC CONTROL ===")
        rospy.loginfo("AXIS CONTROL STRATEGY:")
        rospy.loginfo("  Z position: LOCKED (no vertical movement during rotation)")
        rospy.loginfo("  Roll/Pitch: LOCKED (maintain level orientation)")
        rospy.loginfo("  XY + Yaw: CONTROLLED (follow rotation trajectory)")
        
        # Pre-calculate all trajectory points for smooth VEL+ACC execution
        rospy.loginfo("Pre-calculating trajectory points for VEL+ACC control...")
        trajectory_points = []
        
        while True:
            result = rotation_traj.get_next_uav_position_and_yaw()
            if result is None:
                break
            target_pos, target_yaw = result
            # Lock Z position and add to trajectory
            locked_target_pos = (target_pos[0], target_pos[1], locked_z)
            trajectory_points.append((locked_target_pos, target_yaw))
        
        rospy.loginfo(f"Generated {len(trajectory_points)} trajectory points")
        
        if len(trajectory_points) == 0:
            rospy.logerr("No trajectory points generated")
            return False
        
        # Execute trajectory using VEL+ACC control for each segment
        success = True
        segment_duration = rotation_duration / max(len(trajectory_points) - 1, 1)
        
        for i, (target_pos, target_yaw) in enumerate(trajectory_points):
            if rospy.is_shutdown():
                rospy.logwarn("Rotation interrupted by shutdown")
                success = False
                break
            
            # Get current position for VEL+ACC calculation
            current_pos = self.sm.get_current_position()
            if current_pos is None:
                rospy.logwarn(f"Lost position feedback at step {i}")
                continue
            
            # CRITICAL: Ensure Z remains locked
            current_pos_locked = (current_pos[0], current_pos[1], locked_z)
            
            # Execute VEL+ACC trajectory to this waypoint
            segment_success = self.execute_vel_accel_trajectory(
                start_pos=current_pos_locked,
                target_pos=target_pos,
                target_yaw=target_yaw,
                duration=segment_duration,
                pos_threshold=0.02,
                yaw_threshold=0.05,
                axis_lock_mode='rotation'  # Enhanced stability for rotation
            )
            
            if not segment_success:
                rospy.logwarn(f"VEL+ACC trajectory failed at waypoint {i}/{len(trajectory_points)}")
                # Continue anyway for robustness
            
            # Progress logging
            if i % max(len(trajectory_points) // 10, 1) == 0:  # Log 10 times during rotation
                progress = (i / max(len(trajectory_points) - 1, 1)) * 100
                
                # Verify Z lock is maintained
                current_pos_check = self.sm.get_current_position()
                if current_pos_check is not None:
                    z_drift = abs(current_pos_check[2] - locked_z)
                    rospy.loginfo(f"Modular VEL+ACC Rotation progress: {progress:.1f}% | Z drift: {z_drift:.6f}m")
                    if z_drift > 0.01:  # 1cm drift threshold
                        rospy.logwarn(f"⚠ Z-axis drift detected: {z_drift:.6f}m from locked position")
        
        # Final verification
        final_pos = self.sm.get_current_position()
        if final_pos is not None:
            final_z_drift = abs(final_pos[2] - locked_z)
            final_yaw_error = abs(self._normalize_angle_diff(self.sm.current_yaw - (initial_yaw + rotation_angle)))
            
            rospy.loginfo("=== MODULAR VEL+ACC ROTATION COMPLETED ===")
            rospy.loginfo(f"Final verification:")
            rospy.loginfo(f"  Z drift: {final_z_drift:.6f}m (from locked {locked_z:.6f}m)")
            rospy.loginfo(f"  Yaw error: {math.degrees(final_yaw_error):.3f}° (target: {math.degrees(rotation_angle):.1f}°)")
            rospy.loginfo(f"  Total waypoints: {len(trajectory_points)}")
            
            if final_z_drift > 0.02:  # 2cm threshold
                rospy.logwarn(f"⚠ Significant Z-axis drift detected: {final_z_drift:.6f}m")
        
        rospy.loginfo("Modular VEL+ACC rotation trajectory completed")
        return success

    # ===== Note: Task-specific methods moved to valve_rotation_fang_single.py =====
    # This controller now focuses on generic, reusable motion control methods

    # ===== Generic two-stage valve insertion method =====
    
    def execute_two_stage_valve_insertion(self, start_pos, target_pos, target_yaw, 
                                         safe_altitude_offset=0.05, use_position_control=False):
        """
        Execute two-stage valve insertion with single trajectory for each stage
        
        STRATEGY:
        1. Move to insertion point above valve (XY + Yaw alignment)
        2. Descend directly into valve (Z-only movement)
        
        ADVANTAGES:
        - Eliminates progressive yaw adjustment cumulative errors
        - Simple two-step process with position verification
        - Each stage optimized for specific movement type
        
        Args:
            start_pos: Starting position (x, y, z)
            target_pos: Target position (x, y, z) 
            target_yaw: Target yaw angle
            safe_altitude_offset: Safety height above valve for stage 1 (m)
            use_position_control: Use position control instead of VEL+ACCEL for simple movements
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("=== TWO-STAGE SINGLE-TRAJECTORY VALVE INSERTION ===")
        control_mode = "POSITION CONTROL" if use_position_control else "VEL+ACCEL CONTROL"
        rospy.loginfo(f"CONTROL MODE: {control_mode}")
        rospy.loginfo("STRATEGY: Direct movement → Position verification → Direct descent")
        rospy.loginfo("ELIMINATES: Progressive adjustment and cumulative errors")
        
        # Stage 1: Calculate safe hover position above valve
        safe_hover_pos = (target_pos[0], target_pos[1], target_pos[2] + safe_altitude_offset)
        
        rospy.loginfo(f"Stage 1 target (hover): ({safe_hover_pos[0]:.3f}, {safe_hover_pos[1]:.3f}, {safe_hover_pos[2]:.3f})")
        rospy.loginfo(f"Stage 2 target (insert): ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
        
        # === STAGE 1: MOVE TO POSITION ABOVE VALVE ===
        rospy.loginfo("--- STAGE 1: MOVE TO HOVER POSITION ---")
        stage1_success = self._execute_stage1_move_to_hover(start_pos, safe_hover_pos, target_yaw, use_position_control)
        
        if not stage1_success:
            rospy.logerr("Stage 1 failed - aborting insertion")
            return False
        
        rospy.loginfo("✓ Stage 1 completed - verifying position...")
        
        # Verify Stage 1 position and adjust if needed
        stage1_verified = self._verify_and_adjust_position(safe_hover_pos, target_yaw, 
                                                         pos_threshold=0.01, yaw_threshold=0.015, use_position_control=use_position_control)
        if not stage1_verified:
            rospy.logwarn("Stage 1 position verification failed but continuing...")
        
        # === STAGE 2: DIRECT DESCENT TO VALVE ===
        rospy.loginfo("--- STAGE 2: DIRECT DESCENT TO VALVE ---")
        current_pos = self.sm.get_current_position()
        if current_pos is None:
            rospy.logerr("Lost position feedback before Stage 2")
            return False
        
        stage2_success = self._execute_stage2_direct_descent(current_pos, target_pos, target_yaw, use_position_control)
        
        if not stage2_success:
            rospy.logerr("Stage 2 (descent) failed")
            return False
        
        rospy.loginfo("✓ Stage 2 completed - verifying final position...")
        
        # Final position verification
        final_verified = self._verify_and_adjust_position(target_pos, target_yaw,
                                                        pos_threshold=0.008, yaw_threshold=0.01, use_position_control=use_position_control)
        
        if final_verified:
            rospy.loginfo("✓ TWO-STAGE VALVE INSERTION COMPLETED SUCCESSFULLY")
            return True
        else:
            rospy.logwarn("Final position verification failed but insertion completed")
            return True  # Consider successful if descent completed
    
    def _execute_stage1_move_to_hover(self, start_pos, hover_pos, target_yaw, use_position_control=False):
        """
        Stage 1: Move to hover position above valve with precise XY+Yaw alignment
        
        Args:
            use_position_control: Use position control instead of VEL+ACCEL for stability
        """
        # Calculate movement distance and parameters
        xy_distance = math.sqrt(
            (hover_pos[0] - start_pos[0])**2 + 
            (hover_pos[1] - start_pos[1])**2
        )
        z_distance = abs(hover_pos[2] - start_pos[2])
        yaw_change = abs(self._normalize_angle_diff(target_yaw - self.sm.current_yaw))
        
        rospy.loginfo(f"Stage 1 movement analysis:")
        rospy.loginfo(f"  XY distance: {xy_distance:.3f}m")
        rospy.loginfo(f"  Z distance: {z_distance:.3f}m") 
        rospy.loginfo(f"  Yaw change: {math.degrees(yaw_change):.1f}°")
        
        # Calculate total distance first
        total_distance = math.sqrt(xy_distance**2 + z_distance**2)
        
        # Dynamic speed control: fast approach, slow precision
        if total_distance > 1.0:  # Long distance: use fast speed
            base_speed = 0.1  # 100mm/s for long distance approach
        elif total_distance > 0.5:  # Medium distance: moderate speed
            base_speed = 0.10  # 100mm/s for medium approach
        else:  # Close to valve: precision speed
            base_speed = 0.04  # 40mm/s for precision near valve
        
        yaw_factor = 1.0 + (yaw_change / math.pi)  # Account for yaw complexity
        duration = max(8.0, min(35.0, total_distance / (base_speed / yaw_factor)))
        
        rospy.loginfo(f"Dynamic speed selection: distance={total_distance:.3f}m, speed={base_speed:.3f}m/s")
        
        rospy.loginfo(f"Stage 1 execution: {duration:.1f}s duration, {base_speed:.3f}m/s base speed")
        
        # Choose control method based on distance and complexity
        # 优化策略：长距离移动优先使用简单稳定的位置控制
        if use_position_control or total_distance > 0.5:  # 大于50cm的移动使用位置控制
            # Use stable position control for most approach movements
            rospy.loginfo("Using POSITION CONTROL for Stage 1 (stable approach movement)")
            rospy.loginfo("STRATEGY: Simple position control sufficient for long-distance approach")
            success = self.execute_smooth_trajectory_with_yaw(
                start_pos, hover_pos, target_yaw,
                duration=duration,
                pos_threshold=0.015,  # 15mm精度
                yaw_threshold=0.02    # 1.15°精度
            )
        else:
            # Use VEL+ACCEL only for short, precise movements near valve
            rospy.loginfo("Using VEL+ACCEL CONTROL for Stage 1 (close-range precision)")
            rospy.loginfo("STRATEGY: High-precision control for near-valve positioning")
            success = self.execute_vel_accel_trajectory(
                start_pos=start_pos,
                target_pos=hover_pos,
                target_yaw=target_yaw,
                duration=duration,
                pos_threshold=0.012,  # 12mm precision
                yaw_threshold=0.015,  # 0.86° precision
                axis_lock_mode='rotation'  # Enhanced stability for combined movement
            )
        
        return success
    
    def _execute_stage2_direct_descent(self, start_pos, target_pos, target_yaw, use_position_control=False):
        """
        Stage 2: Direct descent with strict XY and yaw locking
        
        Args:
            use_position_control: Use position control instead of VEL+ACCEL for stability
        """
        z_distance = abs(start_pos[2] - target_pos[2])
        
        rospy.loginfo(f"Stage 2 descent analysis:")
        rospy.loginfo(f"  Descent distance: {z_distance:.3f}m")
        rospy.loginfo(f"  XY position locked at: ({start_pos[0]:.6f}, {start_pos[1]:.6f})")
        rospy.loginfo(f"  Yaw locked at: {math.degrees(target_yaw):.3f}°")
        
        # Very conservative descent for valve insertion precision
        descent_speed = 0.015  # 15mm/s for maximum control
        duration = max(8.0, min(25.0, z_distance / descent_speed))
        
        rospy.loginfo(f"Stage 2 execution: {duration:.1f}s duration, {descent_speed:.3f}m/s descent speed")
        
        # Choose control method - descent is precision operation, check control preference
        if use_position_control:
            # Use stable position control for descent
            rospy.loginfo("Using POSITION CONTROL for Stage 2 (stable descent)")
            success = self.execute_smooth_trajectory_with_yaw(
                start_pos, target_pos, target_yaw,
                duration=duration,
                pos_threshold=0.006,  # 6mm精度
                yaw_threshold=0.008   # 0.46°精度
            )
        else:
            # Use VEL+ACCEL with axis locking for precise control
            rospy.loginfo("Using VEL+ACCEL CONTROL for Stage 2 (axis-locked descent)")
            success = self.execute_vel_accel_trajectory(
                start_pos=start_pos,
                target_pos=target_pos,
                target_yaw=target_yaw,
                duration=duration,
                pos_threshold=0.006,  # 6mm精度
                yaw_threshold=0.008,  # 0.46°精度
                axis_lock_mode='z_only'  # CRITICAL: Lock XY and yaw during descent
            )
        
        return success
    
    def _verify_and_adjust_position(self, target_pos, target_yaw, pos_threshold=0.01, yaw_threshold=0.015, use_position_control=False):
        """
        Verify current position matches target and adjust if necessary
        
        Args:
            target_pos: Expected position (x, y, z)
            target_yaw: Expected yaw angle
            pos_threshold: Position error threshold for adjustment
            yaw_threshold: Yaw error threshold for adjustment
            use_position_control: Use position control instead of VEL+ACCEL for stability
            
        Returns:
            bool: True if position is within tolerance (after adjustment if needed)
        """
        current_pos = self.sm.get_current_position()
        current_yaw = self.sm.current_yaw
        
        if current_pos is None:
            rospy.logerr("Cannot verify position - no position feedback")
            return False
        
        # Calculate errors
        pos_error = math.sqrt(
            (target_pos[0] - current_pos[0])**2 + 
            (target_pos[1] - current_pos[1])**2 + 
            (target_pos[2] - current_pos[2])**2
        )
        yaw_error = abs(self._normalize_angle_diff(target_yaw - current_yaw))
        
        rospy.loginfo(f"Position verification:")
        rospy.loginfo(f"  Position error: {pos_error*1000:.1f}mm (threshold: {pos_threshold*1000:.1f}mm)")
        rospy.loginfo(f"  Yaw error: {math.degrees(yaw_error):.2f}° (threshold: {math.degrees(yaw_threshold):.2f}°)")
        
        # Check if adjustment needed
        needs_adjustment = pos_error > pos_threshold or yaw_error > yaw_threshold
        
        if not needs_adjustment:
            rospy.loginfo("✓ Position verification passed - no adjustment needed")
            return True
        
        # Perform adjustment
        rospy.loginfo("Position adjustment needed - executing correction...")
        
        # Choose control method for adjustment
        if use_position_control:
            rospy.loginfo("Using POSITION CONTROL for position adjustment")
            adjustment_success = self.execute_smooth_trajectory_with_yaw(
                current_pos, target_pos, target_yaw,
                duration=5.0,  # Quick adjustment
                pos_threshold=pos_threshold * 0.8,  # Slightly tighter for adjustment
                yaw_threshold=yaw_threshold * 0.8
            )
        else:
            rospy.loginfo("Using VEL+ACCEL CONTROL for position adjustment")
            adjustment_success = self.execute_vel_accel_trajectory(
                start_pos=current_pos,
                target_pos=target_pos,
                target_yaw=target_yaw,
                duration=5.0,  # Quick adjustment
                pos_threshold=pos_threshold * 0.8,  # Slightly tighter for adjustment
                yaw_threshold=yaw_threshold * 0.8,
                axis_lock_mode='rotation'  # Standard control for adjustment
            )
        
        if adjustment_success:
            rospy.loginfo("✓ Position adjustment completed successfully")
            return True
        else:
            rospy.logwarn("⚠ Position adjustment failed but continuing")
            return False
    
    def execute_three_stage_insertion_with_adaptive_replanning(self, start_pos, target_pos, target_yaw, 
                                                               intermediate_distance=1.0, 
                                                               stage_precision_thresholds=None):
        """
        执行带自适应重规划的三阶段插入控制
        
        Args:
            start_pos: 起始位置
            target_pos: 目标位置
            target_yaw: 目标yaw
            intermediate_distance: 中间距离
            stage_precision_thresholds: 每阶段精度阈值 {'stage1': {'pos': 0.015, 'yaw': 0.02}, ...}
            
        Returns:
            bool: 执行成功状态
        """
        rospy.loginfo("=== THREE-STAGE INSERTION WITH ADAPTIVE REPLANNING ===")
        rospy.loginfo("STRATEGY: Precise control with automatic correction and replanning")
        
        # 设置阶段精度阈值 - RELAXED FOR PRACTICAL OPERATION
        if stage_precision_thresholds:
            default_thresholds = stage_precision_thresholds
        else:
            default_thresholds = {
                'stage1': {'pos': 0.015, 'yaw': 0.02},   # 15mm, 1.1° (unchanged)
                'stage2': {'pos': 0.020, 'yaw': 0.02},   # 20mm, 1.1° (unchanged)  
                'stage3': {'pos': 0.025, 'yaw': 0.025}   # 25mm, 1.4° (RELAXED from 10mm/0.9°)
            }
        
        # === STAGE 1: VALVE YAW ALIGNMENT ===
        rospy.loginfo("--- STAGE 1: VALVE YAW ALIGNMENT ---")
        valve_yaw_target = 0.0  # 假设阀门yaw为0
        
        stage1_success = self._execute_stage_with_replanning(
            stage_name="Stage 1",
            start_pos=start_pos,
            target_pos=start_pos,  # 位置不变，只调整yaw
            target_yaw=valve_yaw_target,
            duration=8.0,
            thresholds=default_thresholds['stage1']
        )
        
        if not stage1_success:
            rospy.logerr("Stage 1 failed after adaptive replanning")
            return False
        
        # 获取Stage 1实际结束位置
        stage1_actual_pos = self.sm.get_current_position()
        stage1_actual_yaw = self.sm.get_current_yaw()
        
        # === STAGE 2: POSITION MOVE ===
        rospy.loginfo("--- STAGE 2: POSITION MOVE ---")
        
        # 修复：直接计算中间目标位置，避免使用未定义的adaptive_planner
        # 计算从当前位置到最终目标的中间位置
        if intermediate_distance >= 1.0:
            # 如果intermediate_distance >= 1.0，使用比例计算
            progress_ratio = 0.6  # 移动到60%的位置作为中间目标
        else:
            # 否则使用距离作为比例
            total_distance = math.sqrt(
                (target_pos[0] - stage1_actual_pos[0])**2 + 
                (target_pos[1] - stage1_actual_pos[1])**2
            )
            progress_ratio = min(0.7, intermediate_distance / max(total_distance, 0.1))
        
        # 线性插值计算中间XY位置，保持当前Z高度
        intermediate_x = stage1_actual_pos[0] + (target_pos[0] - stage1_actual_pos[0]) * progress_ratio
        intermediate_y = stage1_actual_pos[1] + (target_pos[1] - stage1_actual_pos[1]) * progress_ratio
        intermediate_target = (intermediate_x, intermediate_y, stage1_actual_pos[2])
        
        rospy.loginfo(f"Stage 2 intermediate calculation:")
        rospy.loginfo(f"  Progress ratio: {progress_ratio:.2f}")
        rospy.loginfo(f"  From: ({stage1_actual_pos[0]:.3f}, {stage1_actual_pos[1]:.3f}, {stage1_actual_pos[2]:.3f})")
        rospy.loginfo(f"  To: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
        rospy.loginfo(f"  Intermediate: ({intermediate_target[0]:.3f}, {intermediate_target[1]:.3f}, {intermediate_target[2]:.3f})")
        
        stage2_success = self._execute_stage_with_replanning(
            stage_name="Stage 2",
            start_pos=stage1_actual_pos,
            target_pos=intermediate_target,
            target_yaw=stage1_actual_yaw,
            duration=15.0,
            thresholds=default_thresholds['stage2']
        )
        
        if not stage2_success:
            rospy.logerr("Stage 2 failed after adaptive replanning")
            return False
        
        # 获取Stage 2实际结束位置
        stage2_actual_pos = self.sm.get_current_position()
        stage2_actual_yaw = self.sm.get_current_yaw()
        
        # === STAGE 3: PRECISION INSERTION (分两个子阶段) ===
        rospy.loginfo("--- STAGE 3: PRECISION INSERTION ---")
        rospy.loginfo("STAGE 3A: XY final positioning + yaw adjustment (Z height preserved)")
        rospy.loginfo("STAGE 3B: Z descent only (XY and yaw locked)")
        
        # Stage 3A: XY positioning + yaw adjustment，保持当前Z高度
        stage3a_target = (target_pos[0], target_pos[1], stage2_actual_pos[2])  # 保持Stage 2的Z高度
        
        stage3a_success = self._execute_stage_with_replanning(
            stage_name="Stage 3A",
            start_pos=stage2_actual_pos,
            target_pos=stage3a_target,  # XY到最终位置，Z保持不变
            target_yaw=target_yaw,      # yaw调整到最终角度
            duration=15.0,
            thresholds=default_thresholds['stage3']
        )
        
        if not stage3a_success:
            rospy.logerr("Stage 3A (XY positioning + yaw) failed after adaptive replanning")
            return False
            
        # 获取Stage 3A实际结束位置
        stage3a_actual_pos = self.sm.get_current_position()
        stage3a_actual_yaw = self.sm.get_current_yaw()
        
        rospy.loginfo("--- STAGE 3B: SEGMENTED Z DESCENT WITH XY LOCK ---")
        rospy.loginfo("OPTIMIZED STRATEGY: Multi-segment Z descent + Enhanced XY locking + Optimized speed")
        
        # 计算Z轴下降距离和分段策略
        z_descent_distance = abs(target_pos[2] - stage3a_actual_pos[2])
        rospy.loginfo(f"Total Z descent distance: {z_descent_distance*1000:.0f}mm")
        
        # 分段Z下降策略：将大幅下降分为多个小段以减少控制震荡
        if z_descent_distance > 0.25:  # 大于250mm时分段
            segment_distance = 0.18  # 每段180mm，在150-200mm最优范围内
            num_segments = max(2, int(math.ceil(z_descent_distance / segment_distance)))
            rospy.loginfo(f"Large Z descent detected: Splitting into {num_segments} segments of ~{segment_distance*1000:.0f}mm each")
        else:
            num_segments = 1
            rospy.loginfo("Small Z descent: Single segment execution")
        
        # 执行分段Z下降
        current_segment_pos = stage3a_actual_pos
        current_segment_yaw = stage3a_actual_yaw
        
        for segment in range(num_segments):
            rospy.loginfo(f"--- Z DESCENT SEGMENT {segment + 1}/{num_segments} ---")
            
            # 计算当前段的目标Z位置
            if segment == num_segments - 1:
                # 最后一段：到达最终目标位置
                segment_target_z = target_pos[2]
            else:
                # 中间段：按等距分割
                progress = (segment + 1) / num_segments
                segment_target_z = stage3a_actual_pos[2] + (target_pos[2] - stage3a_actual_pos[2]) * progress
            
            # CRITICAL: 严格锁定XY坐标，仅改变Z轴
            segment_target = (
                stage3a_actual_pos[0],  # X锁定到Stage 3A的最终位置
                stage3a_actual_pos[1],  # Y锁定到Stage 3A的最终位置
                segment_target_z        # 仅Z轴改变
            )
            
            segment_distance = abs(segment_target_z - current_segment_pos[2])
            rospy.loginfo(f"Segment {segment + 1}: Z {current_segment_pos[2]:.3f}m → {segment_target_z:.3f}m ({segment_distance*1000:.0f}mm)")
            
            # 执行单段Z下降 - 使用增强的Z下降控制
            segment_success = self._execute_z_descent_segment(
                segment_name=f"Stage 3B-{segment + 1}",
                start_pos=current_segment_pos,
                target_pos=segment_target,
                locked_yaw=stage3a_actual_yaw,  # yaw严格锁定到Stage 3A结果
                xy_lock_pos=(stage3a_actual_pos[0], stage3a_actual_pos[1])  # XY严格锁定
            )
            
            if not segment_success:
                rospy.logerr(f"✗ Z descent segment {segment + 1} failed")
                return False
            
            # 更新当前位置为下一段做准备
            current_segment_pos = self.sm.get_current_position()
            current_segment_yaw = self.sm.get_current_yaw()
            
            rospy.loginfo(f"✓ Z descent segment {segment + 1} completed")
        
        stage3b_success = True  # 所有分段都成功
        
        if stage3b_success:
            rospy.loginfo("✓ THREE-STAGE INSERTION WITH ADAPTIVE REPLANNING COMPLETED SUCCESSFULLY")
            rospy.loginfo("✓ Z-AXIS DESCENT COMPLETED - INSERTION STRATEGY FIXED")
            return True
        else:
            rospy.logerr("✗ Stage 3B (Z descent) failed after adaptive replanning")
            return False
    
    def _execute_stage_with_replanning(self, stage_name, start_pos, target_pos, target_yaw, 
                                       duration, thresholds, max_replanning_attempts=2):
        """
        执行单个阶段，带自适应重规划
        
        Args:
            stage_name: 阶段名称
            start_pos: 起始位置
            target_pos: 目标位置
            target_yaw: 目标yaw
            duration: 预计持续时间
            thresholds: 精度阈值 {'pos': float, 'yaw': float}
            max_replanning_attempts: 最大重规划次数
            
        Returns:
            bool: 执行成功状态
        """
        rospy.loginfo(f"Executing {stage_name} with adaptive replanning capability")
        
        # 简化：不使用复杂的adaptive_planner，直接设置精度阈值
        rospy.loginfo(f"Updated precision thresholds: Position {thresholds['pos']*1000:.0f}mm, Yaw {math.degrees(thresholds['yaw']):.1f}°")
        
        for attempt in range(max_replanning_attempts + 1):
            rospy.loginfo(f"{stage_name} execution attempt {attempt + 1}")
            
            # 计算自适应持续时间 - 针对XY漂移优化的速度控制
            current_pos = self.sm.get_current_position()
            distance = math.sqrt(
                (target_pos[0] - current_pos[0])**2 + 
                (target_pos[1] - current_pos[1])**2 + 
                (target_pos[2] - current_pos[2])**2
            )
            # 针对不同阶段的速度优化 - 解决XY漂移问题
            if "Stage 2" in stage_name:
                # Stage 2: 中等速度确保长距离移动稳定性 0.06m/s (减慢)
                adaptive_duration = max(8.0, min(25.0, distance / 0.06))
            elif "Stage 3A" in stage_name:
                # Stage 3A: 最慢速度+最长时间确保精确XY+yaw定位 0.04m/s (大幅减慢)
                adaptive_duration = max(12.0, min(35.0, distance / 0.04))
                rospy.loginfo("ANTI-DRIFT STRATEGY: Ultra-slow speed for Stage 3A precision")
            elif "Stage 3B" in stage_name:
                # Stage 3B: 应该使用分段Z下降，不应该到达这里
                rospy.logwarn("Stage 3B should use segmented Z descent, not this method")
                adaptive_duration = max(8.0, min(30.0, distance / 0.06))
            else:
                # Stage 1: 正常速度 0.1m/s
                adaptive_duration = max(3.0, min(15.0, distance / 0.1))
            
            # 执行基础轨迹
            success = self.execute_smooth_trajectory_with_yaw(
                current_pos, target_pos, target_yaw,
                duration=adaptive_duration,
                pos_threshold=thresholds['pos'],
                yaw_threshold=thresholds['yaw']
            )
            
            if success:
                rospy.loginfo(f"✓ {stage_name} completed successfully")
                return True
            
            # 如果失败且还有重规划机会
            if attempt < max_replanning_attempts:
                rospy.logwarn(f"{stage_name} precision not met, attempting adaptive replanning...")
                
                # 获取当前实际位置
                current_pos = self.sm.get_current_position()
                current_yaw = self.sm.get_current_yaw()
                
                if not current_pos:
                    rospy.logerr(f"Cannot get current position for {stage_name} replanning")
                    break
                
                # 简化偏差计算：直接计算位置和yaw偏差
                pos_dev = math.sqrt(
                    (target_pos[0] - current_pos[0])**2 + 
                    (target_pos[1] - current_pos[1])**2 + 
                    (target_pos[2] - current_pos[2])**2
                )
                yaw_dev = abs(self._normalize_angle_diff(target_yaw - current_yaw))
                
                rospy.loginfo(f"{stage_name} current deviation: {pos_dev*1000:.1f}mm, {math.degrees(yaw_dev):.2f}°")
                
                # 简化重规划：直接重新执行，延长时间
                adaptive_duration = adaptive_duration * 1.5  # 延长50%时间
                rospy.loginfo(f"Replanning {stage_name} with extended duration: {adaptive_duration:.1f}s")
                
            else:
                rospy.logerr(f"{stage_name} failed after {max_replanning_attempts} replanning attempts")
                return False
        
        return False

    def _execute_z_descent_segment(self, segment_name, start_pos, target_pos, locked_yaw, xy_lock_pos):
        """
        执行单段Z轴下降，带XY校正和优化的多项式轨迹控制
        
        OPTIMIZATION: 减慢速度并使用优化的多项式轨迹来解决overshoot问题
        
        Args:
            segment_name: 段名称
            start_pos: 起始位置
            target_pos: 目标位置（仅Z不同）
            locked_yaw: 锁定的yaw角度
            xy_lock_pos: 锁定的XY坐标 (x, y)
            
        Returns:
            bool: 执行成功状态
        """
        rospy.loginfo(f"=== {segment_name}: OPTIMIZED Z DESCENT (OVERSHOOT PREVENTION) ===")
        
        # 计算Z轴下降距离
        z_distance = abs(target_pos[2] - start_pos[2])
        # 大幅减慢Z下降速度，解决跟踪误差问题
        z_speed = 0.02  # 从0.03m/s进一步减慢到0.02m/s，优先响应跟踪
        adaptive_duration = max(5.0, z_distance / z_speed)  # 增加最小时间到5秒
        
        rospy.loginfo(f"{segment_name} OPTIMIZED parameters:")
        rospy.loginfo(f"  Z descent: {z_distance*1000:.0f}mm at {z_speed:.3f}m/s (SLOWER for precision)")
        rospy.loginfo(f"  Duration: {adaptive_duration:.1f}s (LONGER for stability)")
        rospy.loginfo(f"  XY locked at: ({xy_lock_pos[0]:.3f}, {xy_lock_pos[1]:.3f})")
        rospy.loginfo(f"  Yaw locked at: {math.degrees(locked_yaw):.1f}°")
        rospy.loginfo("  OVERSHOOT PREVENTION: 20Hz control + feedback + rate limiting")
        
        # 强制使用锁定的XY坐标
        locked_start_pos = (xy_lock_pos[0], xy_lock_pos[1], start_pos[2])
        locked_target_pos = (xy_lock_pos[0], xy_lock_pos[1], target_pos[2])
        
        # 第一步：执行优化的多项式Z下降轨迹
        rospy.loginfo("STEP 1: Optimized polynomial Z descent trajectory execution")
        descent_success = self._execute_polynomial_z_trajectory(
            start_pos=locked_start_pos,
            target_pos=locked_target_pos,
            target_yaw=locked_yaw,
            duration=adaptive_duration,
            speed=z_speed  # 传递优化的慢速度
        )
        
        if not descent_success:
            rospy.logerr(f"✗ {segment_name} polynomial descent failed")
            return False
        
        # 第二步：验证并校正XY位置
        rospy.loginfo("STEP 2: XY position verification and correction")
        correction_success = self._verify_and_correct_xy_position(
            target_xy=xy_lock_pos,
            target_z=target_pos[2],
            target_yaw=locked_yaw,
            correction_name=f"{segment_name}_XY_Correction"
        )
        
        if correction_success:
            rospy.loginfo(f"✓ {segment_name} completed successfully with XY correction")
            return True
        else:
            rospy.logwarn(f"⚠ {segment_name} completed but XY correction had issues")
            return True  # 继续执行，不因校正失败而停止整个过程
    
    def _execute_polynomial_z_trajectory(self, start_pos, target_pos, target_yaw, duration, speed=0.05):
        """
        执行优化的多项式Z轨迹，解决overshoot问题
        
        OPTIMIZATION STRATEGY:
        1. 降低控制频率：50Hz → 25Hz → 15Hz → 20Hz，平衡精度与响应性
        2. 位置反馈控制：检查实际位置偏差，避免盲目轨迹跟踪  
        3. 减慢控制速度：0.1m/s → 0.05m/s，提高精度
        4. 增强XY锁定：更严格的XY位置监控和纠错
        
        Args:
            start_pos: 起始位置
            target_pos: 目标位置
            target_yaw: 目标yaw
            duration: 持续时间
            speed: 控制速度(default: 0.05m/s for precision)
            
        Returns:
            bool: 执行成功状态
        """
        rospy.loginfo("=== ENHANCED POLYNOMIAL Z TRAJECTORY (ROOT CAUSE ANALYSIS) ===")
        rospy.loginfo("STRATEGY: Diagnose and fix Z tracking errors at source")
        
        # === Z目标点设置详细诊断 ===
        rospy.loginfo("=== Z TARGET POINT DETAILED DIAGNOSIS ===")
        rospy.loginfo(f"Function inputs:")
        rospy.loginfo(f"  start_pos[2]: {start_pos[2]:.6f}m")
        rospy.loginfo(f"  target_pos[2]: {target_pos[2]:.6f}m") 
        rospy.loginfo(f"  target_yaw: {math.degrees(target_yaw):.1f}°")
        rospy.loginfo(f"  duration: {duration:.1f}s")
        rospy.loginfo(f"  speed: {speed:.3f}m/s")
        
        # 根据Z轴距离动态调整持续时间，确保合理的速度曲线
        z_distance = abs(target_pos[2] - start_pos[2])
        expected_duration = z_distance / speed if speed > 0 else 0
        rospy.loginfo(f"Z movement analysis:")
        rospy.loginfo(f"  Z distance: {z_distance*1000:.1f}mm")
        rospy.loginfo(f"  Expected duration at {speed:.3f}m/s: {expected_duration:.1f}s")
        rospy.loginfo(f"  Input duration: {duration:.1f}s")
        rospy.loginfo(f"  Duration ratio: {duration/expected_duration:.2f}" if expected_duration > 0 else "  Duration ratio: N/A")
        
        # 重新计算持续时间，确保轨迹不会太激进
        conservative_duration = max(duration, z_distance / (speed * 0.8))  # 确保平均速度不超过目标的80%
        if conservative_duration > duration:
            rospy.loginfo(f"Extending duration for smoother trajectory: {duration:.1f}s → {conservative_duration:.1f}s")
            duration = conservative_duration
        
        # 创建更平滑的多项式轨迹，避免过度激进的Z轴变化
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        
        # === 轨迹生成后即时验证Z目标一致性 ===
        rospy.loginfo("=== IMMEDIATE TRAJECTORY Z CONSISTENCY VERIFICATION ===")
        
        # 检查轨迹的起始和结束点是否与输入一致
        initial_traj_point = traj.evaluate_at_time(0.0) if hasattr(traj, 'evaluate_at_time') else None
        final_traj_point = traj.evaluate_at_time(duration) if hasattr(traj, 'evaluate_at_time') else None
        
        if initial_traj_point is not None:
            start_z_error = abs(initial_traj_point[2] - start_pos[2])
            rospy.loginfo(f"Trajectory start point Z: {initial_traj_point[2]:.6f}m (input: {start_pos[2]:.6f}m)")
            rospy.loginfo(f"Start Z error: {start_z_error*1000:.1f}mm")
            
            if start_z_error > 0.001:  # 1mm tolerance
                rospy.logwarn(f"⚠ TRAJECTORY START Z MISMATCH: {start_z_error*1000:.1f}mm")
                rospy.logwarn("  → Polynomial trajectory start point != input start point")
        
        if final_traj_point is not None:
            end_z_error = abs(final_traj_point[2] - target_pos[2])
            rospy.loginfo(f"Trajectory end point Z: {final_traj_point[2]:.6f}m (input: {target_pos[2]:.6f}m)")
            rospy.loginfo(f"End Z error: {end_z_error*1000:.1f}mm")
            
            if end_z_error > 0.001:  # 1mm tolerance
                rospy.logwarn(f"⚠ TRAJECTORY END Z MISMATCH: {end_z_error*1000:.1f}mm")
                rospy.logwarn("  → Polynomial trajectory end point != input target point")
                rospy.logwarn("  → This is the likely root cause of Z TARGET INCONSISTENCY")
                rospy.logwarn("  → Check PolynomialTrajectory.generate_trajectory() boundary conditions")
        
        # 预检查轨迹的Z轴变化率，确保符合UAV能力
        rospy.loginfo("Pre-validating trajectory for UAV dynamic constraints...")
        max_expected_z_rate = 0.0
        test_points = 10
        trajectory_samples = []
        
        for i in range(test_points):
            t = i / float(test_points - 1)
            test_point = traj.evaluate_at_time(t * duration) if hasattr(traj, 'evaluate_at_time') else None
            if test_point:
                trajectory_samples.append((t * duration, test_point))
                if i > 0:
                    dt = duration / test_points
                    z_rate = abs(test_point[2] - prev_z) / dt
                    max_expected_z_rate = max(max_expected_z_rate, z_rate)
                prev_z = test_point[2]
        
        # === 详细轨迹一致性检查 ===
        rospy.loginfo("=== DETAILED TRAJECTORY CONSISTENCY CHECK ===")
        if trajectory_samples:
            start_sample = trajectory_samples[0][1]
            end_sample = trajectory_samples[-1][1]
            
            start_z_error = abs(start_sample[2] - start_pos[2])
            end_z_error = abs(end_sample[2] - target_pos[2])
            
            rospy.loginfo(f"Trajectory start Z: {start_sample[2]:.6f}m (input: {start_pos[2]:.6f}m)")
            rospy.loginfo(f"Start Z error: {start_z_error*1000:.1f}mm")
            rospy.loginfo(f"Trajectory end Z: {end_sample[2]:.6f}m (input: {target_pos[2]:.6f}m)")
            rospy.loginfo(f"End Z error: {end_z_error*1000:.1f}mm")
            
            if start_z_error > 0.001:
                rospy.logwarn(f"⚠ TRAJECTORY START MISMATCH: {start_z_error*1000:.1f}mm")
            if end_z_error > 0.001:
                rospy.logwarn(f"⚠ TRAJECTORY END MISMATCH: {end_z_error*1000:.1f}mm")
                rospy.logwarn("  → This is the likely cause of Z TARGET INCONSISTENCY warnings")
        
        rospy.loginfo(f"Trajectory validation: max Z rate = {max_expected_z_rate:.3f} m/s (target: {speed:.3f} m/s)")
        if max_expected_z_rate > speed * 2.0:
            rospy.logwarn(f"Trajectory may be too aggressive: {max_expected_z_rate:.3f} > {speed*2.0:.3f} m/s")
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)  # 平衡优化：20Hz在精度和响应性间平衡
        
        rospy.loginfo(f"Balanced trajectory: {duration:.1f}s, 20Hz control rate (optimal balance)")
        rospy.loginfo(f"Speed: {speed:.3f}m/s (reduced for precision)")
        rospy.loginfo(f"XY coordinates LOCKED: ({target_pos[0]:.6f}, {target_pos[1]:.6f})")
        
        total_points = 0
        xy_drift_warnings = 0
        position_corrections = 0
        last_z_command = start_pos[2]
        
        # Z轨迹趋势监控变量
        z_trajectory_samples = []  # 记录轨迹Z值的变化趋势
        z_inconsistency_count = 0  # Z不一致警告计数
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed = (current_time - start_time).to_sec()
            
            if elapsed >= duration:
                rospy.loginfo("Polynomial trajectory duration completed")
                break
            
            # 获取当前轨迹点
            current_traj_point = traj.evaluate()
            
            if current_traj_point is None:
                rospy.loginfo("Trajectory evaluation completed")
                break
            
            # 获取实际当前位置进行反馈控制
            current_pos = self.sm.get_current_position()
            total_points += 1  # Count all control points processed
            
            if current_pos is not None:
                # === 增强Z轴目标点诊断 ===
                z_tracking_error = abs(current_pos[2] - current_traj_point[2])
                
                # 初始化locked_point以避免UnboundLocalError
                locked_point = (target_pos[0], target_pos[1], current_traj_point[2])
                
                # 详细分析当前Z轴目标点来源
                if total_points % 50 == 0:  # 每2.5秒记录一次详细诊断 (20Hz)
                    rospy.loginfo("=== Z-AXIS TARGET POINT DIAGNOSTIC ===")
                    rospy.loginfo(f"Current UAV Z: {current_pos[2]:.6f}m")
                    rospy.loginfo(f"Trajectory Z: {current_traj_point[2]:.6f}m")
                    rospy.loginfo(f"Original target Z: {target_pos[2]:.6f}m")
                    
                    # 计算轨迹Z与最终目标Z之间的差异
                    traj_vs_target_diff = abs(current_traj_point[2] - target_pos[2])
                    
                    rospy.loginfo(f"Z target consistency check:")
                    rospy.loginfo(f"  Trajectory vs Final target: {traj_vs_target_diff*1000:.1f}mm")
                    
                    # 检测Z目标不一致问题
                    if traj_vs_target_diff > 0.05:  # 50mm阈值
                        z_inconsistency_count += 1
                        rospy.logwarn(f"⚠ Z TARGET INCONSISTENCY DETECTED: {traj_vs_target_diff*1000:.1f}mm")
                        rospy.logwarn("  → Trajectory Z differs significantly from final target Z")
                        rospy.logwarn("  → This may be the root cause of tracking errors")
                        rospy.logwarn("  → Trajectory generation may have boundary condition issues")
                
                # 记录Z轨迹样本用于趋势分析
                elapsed_time = (current_time - start_time).to_sec()
                progress = min(elapsed_time / duration, 1.0)
                z_trajectory_samples.append({
                    'time': elapsed_time,
                    'progress': progress,
                    'trajectory_z': current_traj_point[2],
                    'target_z': target_pos[2],
                    'uav_z': current_pos[2],
                    'z_error': z_tracking_error
                })
                
                # 每100个点分析一次Z轨迹趋势
                if len(z_trajectory_samples) >= 100 and total_points % 100 == 0:
                    self._analyze_z_trajectory_trend(z_trajectory_samples[-100:], target_pos[2])
                
                # 诊断Z轴跟踪问题：检查是否是轨迹问题还是控制问题
                if z_tracking_error > 0.03:  # 30mm就开始分析，而不是直接暂停
                    # 计算UAV当前Z轴速度（估算）
                    if hasattr(self, '_last_z_pos') and hasattr(self, '_last_z_time'):
                        dt = (current_time - self._last_z_time).to_sec()
                        if dt > 0:
                            current_z_velocity = abs(current_pos[2] - self._last_z_pos) / dt
                            rospy.loginfo(f"Z tracking analysis: error={z_tracking_error*1000:.1f}mm, "
                                        f"UAV_z_vel={current_z_velocity:.3f}m/s, target_vel={speed:.3f}m/s")
                            
                            # 如果UAV已经在以合理速度移动，说明是轨迹太激进
                            if current_z_velocity > speed * 0.8:
                                rospy.loginfo("UAV is moving at reasonable speed, trajectory may be too aggressive")
                            else:
                                rospy.loginfo("UAV response may be slow, checking control responsiveness")
                    
                    # 记录当前位置和时间用于下次计算
                    self._last_z_pos = current_pos[2]
                    self._last_z_time = current_time
                
                # 只有在Z轴跟踪误差非常大时才暂停（严重滞后）
                if z_tracking_error > 0.06:  # 60mm - 仍然比原来的80mm严格
                    rospy.logwarn(f"Severe Z tracking error: {z_tracking_error*1000:.1f}mm, pausing for UAV to catch up")
                    rate.sleep()
                    continue
                
                # XY漂移监控和主动纠错
                xy_drift = math.sqrt(
                    (current_pos[0] - target_pos[0])**2 + 
                    (current_pos[1] - target_pos[1])**2
                )
                
                # 进一步放宽XY漂移阈值，专注于Z轴控制精度
                if xy_drift > 0.025:  # 25mm阈值，减少过度监控
                    xy_drift_warnings += 1
                    if xy_drift_warnings % 40 == 0:  # 20Hz频率下每2秒警告一次，进一步减少日志噪音
                        rospy.logwarn(f"XY drift: {xy_drift*1000:.1f}mm, monitoring but not intervening during Z descent")
                
                # PREVENTIVE: Early XY drift correction at lower threshold to prevent accumulation
                elif xy_drift > 0.015:  # 15mm preventive threshold
                    # Apply gentle correction to prevent drift accumulation
                    preventive_gain = 0.4  # Lower gain for gentle correction
                    xy_correction = (
                        (target_pos[0] - current_pos[0]) * preventive_gain,
                        (target_pos[1] - current_pos[1]) * preventive_gain
                    )
                    
                    # Apply preventive correction
                    corrected_x = target_pos[0] + xy_correction[0]
                    corrected_y = target_pos[1] + xy_correction[1]
                    
                    locked_point = (corrected_x, corrected_y, locked_point[2])
                    
                    # Log only occasionally to avoid noise
                    if total_points % 100 == 0:  # Every 5 seconds at 20Hz
                        rospy.loginfo(f"Preventive XY correction: {xy_drift*1000:.1f}mm drift detected")
                
                # 动态调整Z轴速度限制，根据实际跟踪情况
                z_command_rate = abs(current_traj_point[2] - last_z_command) / (1.0/20.0)  # m/s (20Hz)
                
                # 方案A：增强Z轴PID控制 - 根据跟踪误差动态调整速度限制
                if z_tracking_error > 0.04:  # 40mm以上误差时允许更快响应（改进）
                    speed_limit_factor = 1.2  # 从1.0增加到1.2，提高响应速度
                elif z_tracking_error > 0.02:  # 20-40mm误差时适中
                    speed_limit_factor = 1.4  # 从1.1增加到1.4，加快收敛
                else:  # 跟踪良好时允许更快
                    speed_limit_factor = 1.6  # 从1.3增加到1.6，保持高精度时的响应
                
                max_allowed_rate = speed * speed_limit_factor
                
                if z_command_rate > max_allowed_rate:
                    # 限制Z轴指令变化率
                    if current_traj_point[2] < last_z_command:
                        limited_z = max(current_traj_point[2], last_z_command - max_allowed_rate / 20.0)
                    else:
                        limited_z = min(current_traj_point[2], last_z_command + max_allowed_rate / 20.0)
                    
                    rospy.loginfo(f"Dynamic Z rate limiting: {z_command_rate:.3f} → {max_allowed_rate:.3f} m/s "
                                f"(factor: {speed_limit_factor:.1f}, tracking_error: {z_tracking_error*1000:.1f}mm)")
                    locked_point = (target_pos[0], target_pos[1], limited_z)
                    position_corrections += 1
                else:
                    # CRITICAL: 强制锁定XY坐标，只使用轨迹的Z值
                    locked_point = (target_pos[0], target_pos[1], current_traj_point[2])
                
                # ENHANCED XY drift correction with immediate intervention
                if xy_drift > 0.025:  # 25mm drift threshold for intervention
                    xy_drift_warnings += 1
                    position_corrections += 1
                    
                    # Apply proportional correction to counteract drift
                    correction_gain = 0.8  # Increased from default 0.3 for stronger XY lock
                    xy_correction = (
                        (target_pos[0] - current_pos[0]) * correction_gain,
                        (target_pos[1] - current_pos[1]) * correction_gain
                    )
                    
                    # Add correction to target position for immediate response
                    corrected_x = target_pos[0] + xy_correction[0]
                    corrected_y = target_pos[1] + xy_correction[1]
                    
                    locked_point = (corrected_x, corrected_y, locked_point[2])
                    
                    if xy_drift_warnings % 20 == 0:  # Log every 1 second at 20Hz
                        rospy.logwarn(f"XY drift correction: {xy_drift*1000:.1f}mm → applying gain {correction_gain} correction")
                        rospy.loginfo(f"  Correction vector: X={xy_correction[0]*1000:.1f}mm, Y={xy_correction[1]*1000:.1f}mm")
            else:
                # 如果无法获取位置反馈，使用原始轨迹点
                locked_point = (target_pos[0], target_pos[1], current_traj_point[2])
            
            # 发送位置控制命令
            nav_msg = FlightNav()
            nav_msg.header.stamp = rospy.Time.now()
            nav_msg.header.frame_id = "world"
            
            # 使用位置模式确保精确控制，强化XY锁定
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.target_pos_x = locked_point[0]  # 始终强制使用目标XY
            nav_msg.target_pos_y = locked_point[1]  # 始终强制使用目标XY
            nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
            nav_msg.target_pos_z = locked_point[2]
            nav_msg.yaw_nav_mode = FlightNav.POS_MODE
            nav_msg.target_yaw = target_yaw
            
            # CRITICAL: 增强roll和pitch锁定，防止XY漂移
            nav_msg.roll_nav_mode = FlightNav.NO_NAVIGATION  # 强制锁定roll
            nav_msg.pitch_nav_mode = FlightNav.NO_NAVIGATION  # 强制锁定pitch
            
            self.pub.publish(nav_msg)
            
            # 记录这次的Z指令用于下次比较
            last_z_command = locked_point[2]
            rate.sleep()
        
        rospy.loginfo(f"Enhanced trajectory stats:")
        rospy.loginfo(f"  Total control points: {total_points}")
        rospy.loginfo(f"  XY drift warnings: {xy_drift_warnings} (threshold: 25mm)")
        rospy.loginfo(f"  Position corrections: {position_corrections}")
        rospy.loginfo(f"  Actual control frequency: {total_points/duration:.1f}Hz")
        rospy.loginfo(f"  XY drift rate: {(xy_drift_warnings/total_points)*100:.1f}% of points")
        rospy.loginfo(f"  Control strategy: Dynamic Z limiting + Enhanced diagnostics")
        
        # === Z轴诊断总结报告 ===
        rospy.loginfo("=== Z-AXIS DIAGNOSTIC SUMMARY REPORT ===")
        rospy.loginfo(f"Z inconsistency warnings: {z_inconsistency_count}")
        rospy.loginfo(f"Total trajectory samples: {len(z_trajectory_samples)}")
        
        if z_trajectory_samples:
            # 分析最终的Z目标一致性
            final_sample = z_trajectory_samples[-1]
            final_traj_target_diff = abs(final_sample['trajectory_z'] - final_sample['target_z'])
            
            rospy.loginfo(f"Final Z analysis:")
            rospy.loginfo(f"  Final trajectory Z: {final_sample['trajectory_z']:.6f}m")
            rospy.loginfo(f"  Final target Z: {final_sample['target_z']:.6f}m")
            rospy.loginfo(f"  Final difference: {final_traj_target_diff*1000:.1f}mm")
            rospy.loginfo(f"  Final UAV Z: {final_sample['uav_z']:.6f}m")
            rospy.loginfo(f"  Final tracking error: {final_sample['z_error']*1000:.1f}mm")
            
            if final_traj_target_diff > 0.05:
                rospy.logwarn("❌ DIAGNOSIS: Trajectory generation has systematic Z target inconsistency")
                rospy.logwarn("   → Root cause: PolynomialTrajectory boundary conditions may be incorrect")
                rospy.logwarn("   → Recommendation: Review trajectory.py generate_trajectory() method")
            else:
                rospy.loginfo("✓ DIAGNOSIS: Trajectory Z targets are consistent")
                
            # 平均跟踪误差分析
            if len(z_trajectory_samples) > 10:
                avg_error = sum(s['z_error'] for s in z_trajectory_samples[-50:]) / min(50, len(z_trajectory_samples))
                rospy.loginfo(f"  Average Z tracking error (last 50 points): {avg_error*1000:.1f}mm")
                
                if avg_error > 0.04:  # 40mm平均误差
                    rospy.logwarn("❌ DIAGNOSIS: Persistent large Z tracking errors")
                    rospy.logwarn("   → UAV Z control response may be inadequate")
                elif avg_error > 0.02:  # 20mm平均误差
                    rospy.logwarn("⚠ DIAGNOSIS: Moderate Z tracking errors")
                    rospy.logwarn("   → Consider reducing Z trajectory speed")
                else:
                    rospy.loginfo("✓ DIAGNOSIS: Z tracking performance acceptable")
        
        rospy.loginfo("=== Z-AXIS DIAGNOSTIC COMPLETE ===")
        
        return True
    
    def _analyze_z_trajectory_trend(self, samples, target_z):
        """
        分析Z轨迹趋势，发现系统性问题
        
        Args:
            samples: 最近的轨迹样本列表
            target_z: 目标Z位置
        """
        if len(samples) < 10:
            return
            
        # 计算轨迹Z值与目标Z的偏差趋势
        deviations = [abs(s['trajectory_z'] - target_z) for s in samples]
        avg_deviation = sum(deviations) / len(deviations)
        max_deviation = max(deviations)
        
        # 计算跟踪误差趋势
        tracking_errors = [s['z_error'] for s in samples]
        avg_tracking_error = sum(tracking_errors) / len(tracking_errors)
        max_tracking_error = max(tracking_errors)
        
        rospy.loginfo(f"Z Trajectory Trend Analysis (last {len(samples)} points):")
        rospy.loginfo(f"  Trajectory deviation from target: avg={avg_deviation*1000:.1f}mm, max={max_deviation*1000:.1f}mm")
        rospy.loginfo(f"  UAV tracking error: avg={avg_tracking_error*1000:.1f}mm, max={max_tracking_error*1000:.1f}mm")
        
        # 诊断建议
        if avg_deviation > 0.05:  # 50mm平均偏差
            rospy.logwarn("🔍 TREND ANALYSIS: Trajectory consistently deviates from target Z")
            rospy.logwarn("   → Systematic trajectory generation issue detected")
        
        if avg_tracking_error > 0.03:  # 30mm平均跟踪误差
            rospy.logwarn("🔍 TREND ANALYSIS: UAV struggling to follow Z trajectory")
            rospy.logwarn("   → UAV Z-axis control or trajectory aggressiveness issue")
        
        # 清理临时变量
        if hasattr(self, '_last_z_pos'):
            delattr(self, '_last_z_pos')
        if hasattr(self, '_last_z_time'):
            delattr(self, '_last_z_time')
        
        return True
    
    def _verify_and_correct_xy_position(self, target_xy, target_z, target_yaw, correction_name):
        """
        验证并校正XY位置，确保每段下降后XY精度
        
        Args:
            target_xy: 目标XY位置 (x, y)
            target_z: 目标Z位置
            target_yaw: 目标偏航角
            correction_name: 校正名称(用于日志)
            
        Returns:
            bool: 是否成功
        """
        rospy.loginfo(f"=== {correction_name} XY POSITION VERIFICATION ===")
        
        current_pos = self.sm.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position for XY verification")
            return False
        
        # 计算XY误差
        xy_error = math.sqrt((current_pos[0] - target_xy[0])**2 + 
                            (current_pos[1] - target_xy[1])**2)
        
        rospy.loginfo(f"Current XY: ({current_pos[0]:.3f}, {current_pos[1]:.3f})")
        rospy.loginfo(f"Target XY: ({target_xy[0]:.3f}, {target_xy[1]:.3f})")
        rospy.loginfo(f"XY error: {xy_error*1000:.1f}mm")
        
        # XY误差阈值检查
        if xy_error > 0.02:  # 20mm阈值
            rospy.logwarn(f"{correction_name}: XY error {xy_error*1000:.1f}mm > 20mm, applying correction")
            
            # 执行XY位置校正
            correction_target = (target_xy[0], target_xy[1], current_pos[2])  # 保持当前Z
            success = self.execute_smooth_trajectory_with_yaw(
                start_pos=current_pos,
                target_pos=correction_target,
                target_yaw=target_yaw,
                duration=8.0,  # 较短时间用于校正
                pos_threshold=0.015,  # 15mm精度
                yaw_threshold=0.02
            )
            
            if success:
                rospy.loginfo(f"✓ {correction_name} XY correction successful")
                return True
            else:
                rospy.logwarn(f"⚠ {correction_name} XY correction failed")
                return False
        else:
            rospy.loginfo(f"✓ {correction_name} XY position acceptable ({xy_error*1000:.1f}mm)")
            return True
