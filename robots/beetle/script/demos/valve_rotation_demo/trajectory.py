#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from math import pi, atan2, cos, sin

class PolynomialTrajectory:
    def __init__(self, duration):
        self.duration = duration
        self.coeffs_x = None
        self.coeffs_y = None
        self.coeffs_z = None
        self.coeffs_scalar = None  
        self.start_time = None
        self.is_scalar = False  

    def compute_coefficients(self, start, target):
        # Handle near-zero movement case
        if abs(target - start) < 1e-6:
            return np.array([0, 0, 0, 0, 0, start])
            
        T = self.duration
        if T <= 0:
            raise ValueError("Duration must be positive.")
        
        # 使用数值稳定的归一化时间 t/T ∈ [0,1]
        # 这避免了大T值造成的数值问题
        A = np.array([
            [0,    0,    0,    0,  0, 1],    # t=0: position = start
            [1,    1,    1,    1,  1, 1],    # t=1: position = target  
            [0,    0,    0,    0,  1, 0],    # t=0: velocity = 0
            [5,    4,    3,    2,  1, 0],    # t=1: velocity = 0
            [0,    0,    0,    2,  0, 0],    # t=0: acceleration = 0  
            [20,  12,    6,    2,  0, 0]     # t=1: acceleration = 0
        ])
        B = np.array([start, target, 0, 0, 0, 0])
        
        # 求解多项式系数
        coeffs = np.linalg.solve(A, B)
        
        # 检查数值稳定性 - 验证边界条件
        start_check = coeffs[5]  # t=0时的位置
        target_check = np.sum(coeffs)  # t=1时的位置
        
        start_error = abs(start_check - start)
        target_error = abs(target_check - target)
        
        if start_error > 1e-10 or target_error > 1e-10:
            rospy.logwarn(f"⚠ Polynomial coefficient numerical instability detected:")
            rospy.logwarn(f"  Start error: {start_error*1000:.6f}mm")
            rospy.logwarn(f"  Target error: {target_error*1000:.6f}mm")
            rospy.logwarn(f"  Duration: {T:.3f}s, Movement: {abs(target-start)*1000:.3f}mm")
        
        return coeffs
    
    def generate_trajectory(self, start_pos, target_pos):
        """生成多项式轨迹并存储目标位置用于边界验证"""
        if isinstance(start_pos, (int, float)) and isinstance(target_pos, (int, float)):
            self.is_scalar = True
            self.coeffs_scalar = self.compute_coefficients(start_pos, target_pos)
            self.start_value = start_pos
            self.target_value = target_pos
        else:
            self.is_scalar = False
            self.coeffs_x = self.compute_coefficients(start_pos[0], target_pos[0])
            self.coeffs_y = self.compute_coefficients(start_pos[1], target_pos[1])
            self.coeffs_z = self.compute_coefficients(start_pos[2], target_pos[2])
            self.start_pos = start_pos
            self.target_pos = target_pos
        self.start_time = rospy.Time.now().to_sec()

    def evaluate(self):
        if self.start_time is None:
            return None
        elapsed_time = rospy.Time.now().to_sec() - self.start_time
        
        # CRITICAL FIX: Return target position when trajectory completes instead of None
        if elapsed_time >= self.duration:
            if self.is_scalar:
                return self.target_value
            else:
                return self.target_pos
        
        # CRITICAL FIX: Use normalized time t/T ∈ [0,1] as designed in compute_coefficients
        normalized_time = elapsed_time / self.duration
        # CRITICAL FIX: Correct polynomial evaluation order to match coefficient matrix
        # coeffs order: [a5, a4, a3, a2, a1, a0] (highest to lowest degree)
        # T powers: [t^5, t^4, t^3, t^2, t^1, 1] (highest to lowest degree)
        T = np.array([normalized_time**5, normalized_time**4, normalized_time**3, 
                      normalized_time**2, normalized_time, 1])
        
        if self.is_scalar:
            return np.dot(self.coeffs_scalar, T)
        else:
            return (
                np.dot(self.coeffs_x, T),
                np.dot(self.coeffs_y, T),
                np.dot(self.coeffs_z, T)
            )

    def get_next_position(self):
        """Get next position from trajectory (compatibility method)"""
        return self.evaluate()
    
    def get_velocity(self):
        """Get current velocity from trajectory"""
        if self.start_time is None:
            return None
        elapsed_time = rospy.Time.now().to_sec() - self.start_time
        
        # CRITICAL FIX: Return zero velocity when trajectory completes instead of None
        if elapsed_time >= self.duration:
            if self.is_scalar:
                return 0.0
            else:
                return (0.0, 0.0, 0.0)
        
        # CRITICAL FIX: Use normalized time and correct derivative scaling
        normalized_time = elapsed_time / self.duration
        # Velocity = d/dt[p(t)] = d/dt[p(τ)] * dτ/dt = (1/T) * dp/dτ
        # where τ = t/T is normalized time
        # CRITICAL FIX: Correct velocity coefficient order to match polynomial coefficients
        # coeffs order: [a5, a4, a3, a2, a1, a0]
        # velocity coeffs: [5*a5, 4*a4, 3*a3, 2*a2, 1*a1, 0*a0] / duration
        T_vel = np.array([5*normalized_time**4, 4*normalized_time**3, 3*normalized_time**2, 
                         2*normalized_time, 1, 0]) / self.duration
        
        if self.is_scalar:
            return np.dot(self.coeffs_scalar, T_vel)
        else:
            return (
                np.dot(self.coeffs_x, T_vel),
                np.dot(self.coeffs_y, T_vel),
                np.dot(self.coeffs_z, T_vel)
            )

    def evaluate_at_time(self, elapsed_time):
        """计算轨迹开始后特定时间点的轨迹值，包含边界验证
        
        Args:
            elapsed_time: 轨迹开始后经过的时间（秒）
        """
        # CRITICAL FIX: 使用相对时间而不是绝对时间
        # elapsed_time 是轨迹开始后的相对时间，不需要减去 start_time
        normalized_time = elapsed_time / self.duration
        
        # 边界检查和处理
        if normalized_time <= 0:
            if self.is_scalar:
                return self.start_value
            else:
                return self.start_pos
        elif normalized_time >= 1:
            if self.is_scalar:
                return self.target_value
            else:
                return self.target_pos
        
        # 计算多项式值
        if self.is_scalar:
            return self._evaluate_polynomial(self.coeffs_scalar, normalized_time)
        else:
            x_val = self._evaluate_polynomial(self.coeffs_x, normalized_time)
            y_val = self._evaluate_polynomial(self.coeffs_y, normalized_time)
            z_val = self._evaluate_polynomial(self.coeffs_z, normalized_time)
            return [x_val, y_val, z_val]
    
    def _evaluate_polynomial(self, coeffs, t):
        """使用归一化时间计算5阶多项式值
        
        coeffs order: [a5, a4, a3, a2, a1, a0] (from compute_coefficients matrix)
        polynomial: a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0
        """
        return (coeffs[5] + coeffs[4]*t + coeffs[3]*t**2 + 
                coeffs[2]*t**3 + coeffs[1]*t**4 + coeffs[0]*t**5)

class ValveRotationTrajectory:
    """
    Core trajectory generator for rotating around valve center with constant end-effector distance
    """
    def __init__(self, rotation_duration, valve_center, end_effector_rotation_radius, start_angle, 
                 rotation_angle=2*pi, grasp_height=None, rotation_direction=1,
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823):
        self.rotation_duration = rotation_duration
        self.valve_center = valve_center
        self.end_effector_rotation_radius = end_effector_rotation_radius
        self.end_effector_distance = end_effector_rotation_radius  # Alias for compatibility
        self.start_angle = start_angle
        self.rotation_angle = rotation_angle * rotation_direction
        self.grasp_height = grasp_height
        self.rotation_direction = rotation_direction
        
        self.end_effector_offset_x = end_effector_offset_x
        self.end_effector_offset_y = end_effector_offset_y
        self.end_effector_offset_z = end_effector_offset_z
        
        self._angle_trajectory = None
        
        rospy.loginfo(f"Valve rotation trajectory initialized:")
        rospy.loginfo(f"  End-effector rotation radius: {self.end_effector_rotation_radius:.3f}m")
        rospy.loginfo(f"  Rotation angle: {self.rotation_angle:.3f}rad ({self.rotation_angle*180/pi:.1f}°)")
        rospy.loginfo(f"  Duration: {self.rotation_duration:.1f}s")
        
    def start_trajectory(self):
        """Start rotation trajectory"""
        end_angle = self.start_angle + self.rotation_angle
        self._angle_trajectory = PolynomialTrajectory(self.rotation_duration)
        self._angle_trajectory.generate_trajectory(self.start_angle, end_angle)
        
        rospy.loginfo("Rotation trajectory started")
        
    def get_next_position_and_yaw(self):
        """Get next UAV position and orientation"""
        if self._angle_trajectory is None:
            return None
            
        current_angle = self._angle_trajectory.evaluate()
        if current_angle is None:
            return None
            
        center_x, center_y, center_z = self.valve_center
        
        # Calculate end-effector target position
        end_effector_target_x = center_x + self.end_effector_rotation_radius * cos(current_angle)
        end_effector_target_y = center_y + self.end_effector_rotation_radius * sin(current_angle)
        
        # Calculate end-effector target height
        if self.grasp_height is not None:
            uav_target_z = self.grasp_height
            end_effector_target_z = uav_target_z + self.end_effector_offset_z
        else:
            end_effector_target_z = center_z
            uav_target_z = end_effector_target_z - self.end_effector_offset_z
        
        # Calculate end-effector target orientation (pointing toward valve center)
        dx_to_center = center_x - end_effector_target_x
        dy_to_center = center_y - end_effector_target_y
        end_effector_target_yaw = atan2(dy_to_center, dx_to_center)
        
        # UAV orientation same as end-effector orientation
        uav_target_yaw = end_effector_target_yaw
        
        # Calculate UAV center position from end-effector target position
        cos_yaw = cos(uav_target_yaw)
        sin_yaw = sin(uav_target_yaw)
        
        world_offset_x = (cos_yaw * self.end_effector_offset_x - 
                         sin_yaw * self.end_effector_offset_y)
        world_offset_y = (sin_yaw * self.end_effector_offset_x + 
                         cos_yaw * self.end_effector_offset_y)
        
        uav_target_x = end_effector_target_x - world_offset_x
        uav_target_y = end_effector_target_y - world_offset_y
        
        return ((uav_target_x, uav_target_y, uav_target_z), uav_target_yaw)
        
    def get_next_uav_position_and_yaw(self):
        """Get next UAV position and orientation (compatibility interface)"""
        return self.get_next_position_and_yaw()
    
    def get_next_position(self):
        """Get next position (compatibility interface)"""
        result = self.get_next_position_and_yaw()
        if result is None:
            return None
        return result[0]
        
    def is_complete(self):
        """Check if trajectory is complete"""
        return self.get_next_position() is None

def validate_uav_position(uav_pos, uav_yaw, valve_center, 
                         end_effector_offset_x=0.246, end_effector_offset_y=0.0):
    """
    Validate UAV position for safe trajectory execution
    """
    # Calculate UAV to valve center distance
    uav_to_valve_distance = math.sqrt((uav_pos[0] - valve_center[0])**2 + 
                                    (uav_pos[1] - valve_center[1])**2)
    
    # Calculate end-effector position
    cos_yaw = cos(uav_yaw)
    sin_yaw = sin(uav_yaw)
    
    ee_world_x = uav_pos[0] + cos_yaw * end_effector_offset_x - sin_yaw * end_effector_offset_y
    ee_world_y = uav_pos[1] + sin_yaw * end_effector_offset_x + cos_yaw * end_effector_offset_y
    
    ee_to_valve_distance = math.sqrt((ee_world_x - valve_center[0])**2 + 
                                   (ee_world_y - valve_center[1])**2)
    
    # Validation criteria
    valve_radius = 0.1225
    # OPTIMIZED: Practical safe distance for post-insertion rotation start
    # Based on actual insertion results, UAV should be positioned optimally for rotation
    min_safe_uav_distance = valve_radius + end_effector_offset_x * 0.2  # Further reduced from 50% to 40%
    
    errors = []
    
    if uav_to_valve_distance < min_safe_uav_distance:
        errors.append(f"UAV too close to valve center: {uav_to_valve_distance:.3f}m < {min_safe_uav_distance:.3f}m")
    
    # Very lenient end-effector clearance check - 25% of valve radius for successful insertion
    if ee_to_valve_distance < valve_radius * 0.25:
        errors.append(f"End-effector inside valve: {ee_to_valve_distance:.3f}m < {valve_radius * 0.25:.3f}m")
    
    return len(errors) == 0, errors

def create_constant_distance_trajectory(current_uav_pos, current_uav_yaw, valve_center, 
                                      rotation_angle, rotation_duration, 
                                      end_effector_offset_x=0.246, end_effector_offset_y=0.0, 
                                      end_effector_offset_z=0.0743823, rotation_direction=1):
    """
    Create constant distance rotation trajectory with validation
    """
    rospy.loginfo("Creating constant distance rotation trajectory...")
    
    # Validate UAV position
    is_valid, errors = validate_uav_position(
        current_uav_pos, current_uav_yaw, valve_center,
        end_effector_offset_x, end_effector_offset_y
    )
    
    if not is_valid:
        rospy.logerr("UAV position validation failed:")
        for error in errors:
            rospy.logerr(f"  - {error}")
        rospy.logerr("Cannot create safe trajectory - UAV position must be corrected")
        return None
    
    # Calculate current end-effector position
    cos_yaw = cos(current_uav_yaw)
    sin_yaw = sin(current_uav_yaw)
    
    ee_x = current_uav_pos[0] + cos_yaw * end_effector_offset_x - sin_yaw * end_effector_offset_y
    ee_y = current_uav_pos[1] + sin_yaw * end_effector_offset_x + cos_yaw * end_effector_offset_y
    
    # Calculate distance to valve center
    valve_x, valve_y, _ = valve_center
    dx = ee_x - valve_x
    dy = ee_y - valve_y
    end_effector_distance = math.sqrt(dx**2 + dy**2)
    
    # Calculate starting angle
    start_angle = atan2(dy, dx)
    
    rospy.loginfo(f"Trajectory parameters:")
    rospy.loginfo(f"  UAV position: {current_uav_pos}")
    rospy.loginfo(f"  End-effector distance: {end_effector_distance:.3f}m")
    rospy.loginfo(f"  Starting angle: {start_angle:.3f}rad ({start_angle*180/pi:.1f}°)")
    
    # Create trajectory
    trajectory = ValveRotationTrajectory(
        rotation_duration=rotation_duration,
        valve_center=valve_center,
        end_effector_rotation_radius=end_effector_distance,
        start_angle=start_angle,
        rotation_angle=rotation_angle,
        grasp_height=current_uav_pos[2],
        rotation_direction=rotation_direction,
        end_effector_offset_x=end_effector_offset_x,
        end_effector_offset_y=end_effector_offset_y,
        end_effector_offset_z=end_effector_offset_z
    )
    
    return trajectory

# Legacy compatibility classes (kept for backward compatibility)
class AlignToGraspTrajectory:
    """Simplified align trajectory - recommend using insertion optimizer instead"""
    def __init__(self, approach_duration, valve_center, valve_pose_yaw, grasp_height, 
                 valve_radius=0.1225, valve_beam_width=0.0185, 
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823,
                 claw_separation=0.150, rotation_direction=1, insertion_offset=0.015):
        # Minimal implementation for backward compatibility
        self.approach_duration = approach_duration
        self.valve_center = valve_center
        self.valve_pose_yaw = valve_pose_yaw
        self.grasp_height = grasp_height
        self.valve_radius = valve_radius
        self.rotation_direction = rotation_direction
        self.insertion_offset = insertion_offset
        self.end_effector_offset_x = end_effector_offset_x
        self.end_effector_offset_y = end_effector_offset_y
        self.end_effector_offset_z = end_effector_offset_z
        
        self._trajectory = None
        self._target_body_pos = None
        self._target_body_yaw = None
        
        rospy.logwarn("AlignToGraspTrajectory is deprecated - use insertion optimizer instead")
        
    def calculate_grasp_position_and_yaw(self):
        """Simplified grasp position calculation"""
        center_x, center_y, center_z = self.valve_center
        
        # Simple approach: position UAV at valve radius + offset distance
        target_distance = self.valve_radius + self.end_effector_offset_x
        target_angle = self.valve_pose_yaw + pi  # Opposite side of valve
        
        body_target_x = center_x + target_distance * cos(target_angle)
        body_target_y = center_y + target_distance * sin(target_angle)
        body_target_z = self.grasp_height
        body_yaw = target_angle
        
        end_effector_x = body_target_x + self.end_effector_offset_x * cos(body_yaw)
        end_effector_y = body_target_y + self.end_effector_offset_x * sin(body_yaw)
        end_effector_z = body_target_z + self.end_effector_offset_z
        
        return ((body_target_x, body_target_y, body_target_z), 
                body_yaw, 
                (end_effector_x, end_effector_y, end_effector_z))
    
    def set_start_position(self, start_pos):
        """Set start position and generate trajectory"""
        self._target_body_pos, self._target_body_yaw, _ = self.calculate_grasp_position_and_yaw()
        self._trajectory = PolynomialTrajectory(self.approach_duration)
        self._trajectory.generate_trajectory(start_pos, self._target_body_pos)
        
    def get_next_position_and_yaw(self):
        """Get next position and yaw"""
        if self._trajectory is None:
            return None
        pos = self._trajectory.evaluate()
        if pos is None:
            return None
        return (pos, self._target_body_yaw)
    
    def get_next_position(self):
        """Get next position"""
        result = self.get_next_position_and_yaw()
        if result is None:
            return None
        return result[0]
    
    def is_complete(self):
        """Check if trajectory is complete"""
        return self.get_next_position() is None

class ConstantDistanceValveRotationTrajectory(ValveRotationTrajectory):
    """Legacy compatibility class"""
    def __init__(self, valve_center, end_effector_distance, rotation_angle, 
                 rotation_duration, start_angle, 
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823,
                 rotation_direction=1, grasp_height=None):
        super().__init__(
            rotation_duration=rotation_duration,
            valve_center=valve_center,
            end_effector_rotation_radius=end_effector_distance,
            start_angle=start_angle,
            rotation_angle=rotation_angle,
            grasp_height=grasp_height,
            rotation_direction=rotation_direction,
            end_effector_offset_x=end_effector_offset_x,
            end_effector_offset_y=end_effector_offset_y,
            end_effector_offset_z=end_effector_offset_z
        )
        rospy.logwarn("ConstantDistanceValveRotationTrajectory is deprecated - use ValveRotationTrajectory instead")

class AdaptiveTrajectoryPlanner:
    """
    自适应轨迹重规划器 - 负责轨迹生成和重规划的数学逻辑
    职责：纯轨迹计算，不涉及控制执行
    """
    
    def __init__(self):
        self.precision_thresholds = {
            'position': 0.020,  # 20mm
            'yaw': 0.02         # 1.1°
        }
        
    def create_trajectory(self, start_pos, target_pos, duration):
        """创建基础轨迹"""
        trajectory = PolynomialTrajectory(duration)
        trajectory.generate_trajectory(start_pos, target_pos)
        return trajectory
    
    def calculate_deviation(self, planned_pos, actual_pos, planned_yaw=None, actual_yaw=None):
        """计算位置和角度偏差"""
        if isinstance(planned_pos, (list, tuple)) and len(planned_pos) >= 3:
            pos_deviation = math.sqrt(
                (actual_pos[0] - planned_pos[0])**2 + 
                (actual_pos[1] - planned_pos[1])**2 + 
                (actual_pos[2] - planned_pos[2])**2
            )
        else:
            pos_deviation = abs(actual_pos - planned_pos)
        
        yaw_deviation = 0
        if planned_yaw is not None and actual_yaw is not None:
            yaw_deviation = abs(self._normalize_angle(actual_yaw - planned_yaw))
        
        return pos_deviation, yaw_deviation
    
    def needs_replanning(self, pos_deviation, yaw_deviation, stage_name=""):
        """判断是否需要重规划"""
        pos_threshold = self.precision_thresholds['position']
        yaw_threshold = self.precision_thresholds['yaw']
        
        needs_replan = (pos_deviation > pos_threshold or yaw_deviation > yaw_threshold)
        
        if needs_replan:
            rospy.logwarn(f"{stage_name} deviation detected - Position: {pos_deviation*1000:.1f}mm (>{pos_threshold*1000:.0f}mm), Yaw: {math.degrees(yaw_deviation):.2f}° (>{math.degrees(yaw_threshold):.1f}°)")
        else:
            rospy.loginfo(f"{stage_name} deviation acceptable - Position: {pos_deviation*1000:.1f}mm, Yaw: {math.degrees(yaw_deviation):.2f}°")
        
        return needs_replan
    
    def replan_from_current(self, current_pos, current_yaw, original_target_pos, original_target_yaw, duration):
        """从当前位置重新规划到目标位置"""
        rospy.loginfo("=== ADAPTIVE TRAJECTORY REPLANNING ===")
        rospy.loginfo(f"Replanning from actual position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}) at {math.degrees(current_yaw):.1f}°")
        rospy.loginfo(f"To target: ({original_target_pos[0]:.3f}, {original_target_pos[1]:.3f}, {original_target_pos[2]:.3f}) at {math.degrees(original_target_yaw):.1f}°")
        
        # 重新计算距离和持续时间
        distance = math.sqrt(
            (original_target_pos[0] - current_pos[0])**2 + 
            (original_target_pos[1] - current_pos[1])**2 + 
            (original_target_pos[2] - current_pos[2])**2
        )
        
        # 自适应调整持续时间
        adjusted_duration = max(duration * 0.3, distance / 0.1)  # 最小30%原时间，或以0.1m/s速度
        
        rospy.loginfo(f"Replanned trajectory: {distance:.3f}m in {adjusted_duration:.1f}s")
        
        # 创建新轨迹
        new_trajectory = self.create_trajectory(current_pos, original_target_pos, adjusted_duration)
        
        return new_trajectory, adjusted_duration
    
    def replan_intermediate_target(self, current_pos, original_target, valve_center, intermediate_distance):
        """重新计算中间目标位置（用于多阶段插入）"""
        # 重新计算中间位置基于实际当前位置
        direction_to_valve = math.atan2(
            valve_center[1] - current_pos[1], 
            valve_center[0] - current_pos[0]
        )
        
        intermediate_x = valve_center[0] - intermediate_distance * math.cos(direction_to_valve)
        intermediate_y = valve_center[1] - intermediate_distance * math.sin(direction_to_valve)
        intermediate_z = original_target[2]  # 保持原始高度
        
        new_intermediate = (intermediate_x, intermediate_y, intermediate_z)
        
        rospy.loginfo(f"Recalculated intermediate target from actual position:")
        rospy.loginfo(f"  Original intermediate: {original_target}")
        rospy.loginfo(f"  New intermediate: {new_intermediate}")
        
        return new_intermediate
    
    def set_precision_thresholds(self, position_threshold, yaw_threshold):
        """设置精度阈值"""
        self.precision_thresholds['position'] = position_threshold
        self.precision_thresholds['yaw'] = yaw_threshold
        rospy.loginfo(f"Updated precision thresholds: Position {position_threshold*1000:.0f}mm, Yaw {math.degrees(yaw_threshold):.1f}°")
    
    def _normalize_angle(self, angle):
        """标准化角度到[-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def create_z_only_trajectory(self, start_pos, target_z, duration):
        """创建仅Z轴变化的轨迹，XY保持锁定"""
        target_pos = (start_pos[0], start_pos[1], target_z)
        trajectory = PolynomialTrajectory(duration)
        trajectory.generate_trajectory(start_pos, target_pos)
        return trajectory
    
    def calculate_optimal_z_segments(self, total_z_distance, max_segment_distance=0.18):
        """
        计算Z轴下降的最优分段策略
        
        Args:
            total_z_distance: 总Z下降距离
            max_segment_distance: 最大段距离（默认180mm）
            
        Returns:
            list: 每段的距离列表
        """
        if total_z_distance <= max_segment_distance:
            return [total_z_distance]
        
        # 计算分段数量和每段距离
        num_segments = math.ceil(total_z_distance / max_segment_distance)
        segment_distance = total_z_distance / num_segments
        
        segments = [segment_distance] * num_segments
        
        rospy.loginfo(f"Z descent segmentation: {total_z_distance*1000:.0f}mm → {num_segments} segments of {segment_distance*1000:.0f}mm each")
        
        return segments
    
    def calculate_z_descent_speed(self, z_distance, min_speed=0.06, max_speed=0.10):
        """
        根据Z下降距离计算最优速度，减少控制震荡
        
        Args:
            z_distance: Z下降距离
            min_speed: 最小速度（用于大幅下降）
            max_speed: 最大速度（用于小幅下降）
            
        Returns:
            float: 最优下降速度
        """
        if z_distance < 0.10:  # 小于100mm
            return max_speed  # 0.10m/s
        elif z_distance < 0.20:  # 100-200mm
            return 0.08  # 0.08m/s
        else:  # 大于200mm
            return min_speed  # 0.06m/s


class SelfRotationRevolutionTrajectory:
    """
    自转+公转阀门旋转轨迹生成器
    
    实现真正的"自转+公转"运动模式：
    - 公转：UAV绕阀门中心做圆周运动
    - 自转：UAV自身yaw角同步旋转
    
    这种运动模式可以保持爪子与阀门槽的接触角度恒定，避免脱出
    """
    
    def __init__(self, valve_center, radius, rotation_angle, duration, 
                 start_pos, start_yaw, clockwise=True):
        """
        初始化自转+公转轨迹
        
        Args:
            valve_center: 阀门中心坐标 (x, y, z)
            radius: 旋转半径（UAV到阀门中心的距离）
            rotation_angle: 旋转角度（弧度）
            duration: 运动持续时间（秒）
            start_pos: 起始位置 (x, y, z)
            start_yaw: 起始yaw角度（弧度）
            clockwise: 是否顺时针旋转
        """
        self.valve_center = valve_center
        self.radius = radius
        self.rotation_angle = rotation_angle if not clockwise else -rotation_angle
        self.duration = duration
        self.start_pos = start_pos
        self.start_yaw = start_yaw
        self.clockwise = clockwise
        
        # 计算起始角度
        dx = start_pos[0] - valve_center[0]
        dy = start_pos[1] - valve_center[1]
        self.start_angle = math.atan2(dy, dx)
        
        # 计算初始yaw参考角度（面向阀门中心的角度）
        self.initial_center_facing_yaw = math.atan2(
            valve_center[1] - start_pos[1],
            valve_center[0] - start_pos[0]
        )
        
        rospy.loginfo("=== SELF-ROTATION + REVOLUTION TRAJECTORY ===")
        rospy.loginfo(f"Valve center: ({valve_center[0]:.3f}, {valve_center[1]:.3f}, {valve_center[2]:.3f})")
        rospy.loginfo(f"Rotation radius: {radius*1000:.1f}mm")
        rospy.loginfo(f"Rotation angle: {math.degrees(abs(rotation_angle)):.1f}°")
        rospy.loginfo(f"Direction: {'Clockwise' if clockwise else 'Counter-clockwise'}")
        rospy.loginfo(f"Duration: {duration:.1f}s")
        rospy.loginfo(f"Start angle: {math.degrees(self.start_angle):.1f}°")
        rospy.loginfo(f"Start yaw: {math.degrees(start_yaw):.1f}°")
    
    def get_position_and_yaw_at_progress(self, progress):
        """
        根据进度计算位置和yaw角度
        
        Args:
            progress: 运动进度 [0.0, 1.0]
            
        Returns:
            tuple: ((x, y, z), yaw)
        """
        if progress < 0.0:
            progress = 0.0
        elif progress > 1.0:
            progress = 1.0
        
        # === 公转：计算圆周运动位置 ===
        current_angle = self.start_angle + progress * self.rotation_angle
        target_x = self.valve_center[0] + self.radius * math.cos(current_angle)
        target_y = self.valve_center[1] + self.radius * math.sin(current_angle)
        target_z = self.start_pos[2]  # 保持Z高度不变
        
        # === 自转：UAV yaw角同步旋转 ===
        # 方法：UAV的yaw随着圆周运动同步旋转，保持相对接触角度
        target_yaw = self.start_yaw + progress * self.rotation_angle
        
        return ((target_x, target_y, target_z), target_yaw)
    
    def get_velocity_at_progress(self, progress):
        """
        计算指定进度下的线速度和角速度
        
        Args:
            progress: 运动进度 [0.0, 1.0]
            
        Returns:
            tuple: ((vx, vy, vz), yaw_velocity)
        """
        # 计算角速度
        angular_velocity = abs(self.rotation_angle) / self.duration
        
        # 计算当前角度
        current_angle = self.start_angle + progress * self.rotation_angle
        
        # 计算切向角度（垂直于半径方向）
        tangential_angle = current_angle + math.pi/2
        if self.rotation_angle < 0:  # 顺时针旋转，反向切向角度
            tangential_angle += math.pi
        
        # 计算切向速度
        tangential_speed = self.radius * angular_velocity
        vx = tangential_speed * math.cos(tangential_angle)
        vy = tangential_speed * math.sin(tangential_angle)
        vz = 0.0  # Z方向速度为0
        
        # 计算yaw角速度
        yaw_direction = 1.0 if self.rotation_angle < 0 else -1.0  # 顺时针vs逆时针
        yaw_velocity = angular_velocity * yaw_direction
        
        return ((vx, vy, vz), yaw_velocity)
    
    def apply_velocity_smoothing(self, progress, velocity, yaw_velocity):
        """
        应用速度平滑（加速和减速）
        
        Args:
            progress: 运动进度 [0.0, 1.0]
            velocity: 原始速度 (vx, vy, vz)
            yaw_velocity: 原始yaw角速度
            
        Returns:
            tuple: (平滑后的速度, 平滑后的yaw速度)
        """
        smooth_factor = 1.0
        
        # 前10%：平滑加速
        if progress < 0.1:
            smooth_factor = progress / 0.1
        # 后10%：平滑减速
        elif progress > 0.9:
            smooth_factor = (1.0 - progress) / 0.1
        
        # 应用平滑因子
        smoothed_velocity = (
            velocity[0] * smooth_factor,
            velocity[1] * smooth_factor,
            velocity[2] * smooth_factor
        )
        smoothed_yaw_velocity = yaw_velocity * smooth_factor
        
        return (smoothed_velocity, smoothed_yaw_velocity)
    
    def validate_trajectory(self, progress_points=10):
        """
        验证轨迹的有效性
        
        Args:
            progress_points: 验证点数量
            
        Returns:
            tuple: (is_valid, error_messages)
        """
        errors = []
        
        # 检查基本参数
        if self.duration <= 0:
            errors.append("Duration must be positive")
        
        if self.radius <= 0:
            errors.append("Radius must be positive")
        
        if abs(self.rotation_angle) < math.radians(1):
            errors.append("Rotation angle too small (< 1°)")
        
        # 检查轨迹点
        for i in range(progress_points + 1):
            progress = i / progress_points
            position, yaw = self.get_position_and_yaw_at_progress(progress)
            
            # 检查半径一致性
            actual_radius = math.sqrt(
                (position[0] - self.valve_center[0])**2 + 
                (position[1] - self.valve_center[1])**2
            )
            radius_error = abs(actual_radius - self.radius)
            
            if radius_error > 0.001:  # 1mm tolerance
                errors.append(f"Radius inconsistency at progress {progress:.1f}: "
                             f"expected {self.radius:.3f}, got {actual_radius:.3f}")
        
        is_valid = len(errors) == 0
        return (is_valid, errors)
    
    def get_trajectory_info(self):
        """
        获取轨迹信息摘要
        
        Returns:
            dict: 轨迹信息
        """
        return {
            'type': 'self_rotation_revolution',
            'valve_center': self.valve_center,
            'radius': self.radius,
            'rotation_angle_deg': math.degrees(abs(self.rotation_angle)),
            'duration': self.duration,
            'clockwise': self.clockwise,
            'angular_velocity_deg_per_sec': math.degrees(abs(self.rotation_angle) / self.duration),
            'tangential_speed_m_per_sec': self.radius * abs(self.rotation_angle) / self.duration
        }


class ValveRotationTrajectoryManager:
    """
    阀门旋转轨迹管理器
    
    提供统一的接口来创建和管理不同类型的阀门旋转轨迹
    """
    
    @staticmethod
    def create_self_rotation_revolution_trajectory(valve_center, radius, rotation_angle_deg, 
                                                  duration, start_pos, start_yaw, clockwise=True):
        """
        创建自转+公转轨迹
        
        Args:
            valve_center: 阀门中心坐标
            radius: 旋转半径
            rotation_angle_deg: 旋转角度（度）
            duration: 持续时间
            start_pos: 起始位置
            start_yaw: 起始yaw角度
            clockwise: 是否顺时针
            
        Returns:
            SelfRotationRevolutionTrajectory: 轨迹对象
        """
        rotation_angle_rad = math.radians(rotation_angle_deg)
        return SelfRotationRevolutionTrajectory(
            valve_center=valve_center,
            radius=radius,
            rotation_angle=rotation_angle_rad,
            duration=duration,
            start_pos=start_pos,
            start_yaw=start_yaw,
            clockwise=clockwise
        )
    
    @staticmethod
    def create_legacy_trajectory(valve_center, radius, rotation_angle_deg, duration, start_angle):
        """
        创建传统轨迹（仅公转，始终面向中心）
        
        Args:
            valve_center: 阀门中心坐标
            radius: 旋转半径
            rotation_angle_deg: 旋转角度（度）
            duration: 持续时间
            start_angle: 起始角度
            
        Returns:
            ConstantDistanceValveRotationTrajectory: 轨迹对象
        """
        return ConstantDistanceValveRotationTrajectory(
            valve_center=valve_center,
            end_effector_distance=radius,
            rotation_angle=math.radians(rotation_angle_deg),
            rotation_duration=duration,
            start_angle=start_angle
        )
