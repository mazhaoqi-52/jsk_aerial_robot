#!/usr/bin/env python
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
        if abs(target - start) < 1e-6:
            return np.array([0, 0, 0, 0, 0, start])
        T = self.duration
        if T <= 0:
            raise ValueError("Duration must be positive.")
        A = np.array([
            [0,       0,      0,    0,  0, 1],
            [T**5,     T**4,   T**3,  T**2, T, 1],
            [0,         0,      0,    0,  1, 0],
            [5*T**4,   4*T**3, 3*T**2, 2*T, 1, 0],
            [0,         0,      0,    2,   0, 0],
            [20*T**3, 12*T**2, 6*T,    2,   0, 0]
        ])
        B = np.array([start, target, 0, 0, 0, 0])
        return np.linalg.solve(A, B)
    
    def generate_trajectory(self, start_pos, target_pos):
        if isinstance(start_pos, (int, float)) and isinstance(target_pos, (int, float)):
            self.is_scalar = True
            self.coeffs_scalar = self.compute_coefficients(start_pos, target_pos)
        else:
            self.is_scalar = False
            self.coeffs_x = self.compute_coefficients(start_pos[0], target_pos[0])
            self.coeffs_y = self.compute_coefficients(start_pos[1], target_pos[1])
            self.coeffs_z = self.compute_coefficients(start_pos[2], target_pos[2])
        self.start_time = rospy.Time.now().to_sec()

    def evaluate(self):
        if self.start_time is None:
            return None
        elapsed_time = rospy.Time.now().to_sec() - self.start_time
        if elapsed_time > self.duration:
            return None 
        T = np.array([elapsed_time**5, elapsed_time**4, elapsed_time**3, 
                      elapsed_time**2, elapsed_time, 1])
        if self.is_scalar:
            return np.dot(self.coeffs_scalar, T)
        else:
            return (
                np.dot(self.coeffs_x, T),
                np.dot(self.coeffs_y, T),
                np.dot(self.coeffs_z, T)
            )

class AlignToGraspTrajectory:
    def __init__(self, approach_duration, valve_center, valve_pose_yaw, grasp_height, 
                 valve_radius=0.1225, valve_beam_width=0.0185, 
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823,
                 claw_separation=0.16876, rotation_direction=1, insertion_offset=0.015):  # Distance between two claws (0.08438 * 2)
        """
        Args:
            rotation_direction: 1 for anticlockwise (default, typical valve operation), -1 for clockwise
            insertion_offset: Offset distance for smooth insertion (meters)
        """
        self.approach_duration = approach_duration
        self.valve_center = valve_center
        self.valve_pose_yaw = valve_pose_yaw
        self.grasp_height = grasp_height
        self.valve_radius = valve_radius
        self.valve_beam_width = valve_beam_width
        
        self.end_effector_offset_x = end_effector_offset_x
        self.end_effector_offset_y = end_effector_offset_y
        self.end_effector_offset_z = end_effector_offset_z
        
        self.claw_separation = claw_separation
        self.rotation_direction = rotation_direction  # 1 for anticlockwise, -1 for clockwise
        self.insertion_offset = insertion_offset  # Offset for smooth insertion
        
        self._trajectory = None
        self._target_body_pos = None
        self._target_body_yaw = None
        self._target_end_effector_pos = None
        self._claw_positions = None
        
        # CRITICAL FIX: According to your description, the UAV should fly toward the valve along the positive X-axis
        # beam0 is aligned with the positive X-axis, left manipulator is between beam0 and beam1, close to beam1
        # right manipulator is between beam1 and beam2, close to beam2
        rospy.loginfo("AlignToGraspTrajectory: Initializing valve opening method according to user description")
        rospy.loginfo("  - UAV flies toward valve along positive X-axis")
        rospy.loginfo("  - Left manipulator: between beam0 and beam1, close to beam1")
        rospy.loginfo("  - Right manipulator: between beam1 and beam2, close to beam2")
        rospy.loginfo("  - Counterclockwise rotation, most of the body is outside the valve")
        
    def calculate_grasp_position_and_yaw(self):
        """
        CRITICAL FIX: Re-implement valve opening method according to user description
        - UAV flies toward valve along positive X-axis (beam0 aligned with positive X-axis)
        - Left manipulator is between beam0 and beam1, close to beam1
        - Right manipulator is between beam1 and beam2, close to beam2
        - Counterclockwise rotation, most of the body is outside the valve
        """
        center_x, center_y, center_z = self.valve_center
        
        # Calculate beam angles: beam0(0°), beam1(120°), beam2(240°)
        beam_angles = [self.valve_pose_yaw + i * 2*pi/3 for i in range(3)]
        beam0_angle = beam_angles[0]  # 0° - aligned with positive X-axis
        beam1_angle = beam_angles[1]  # 120°
        beam2_angle = beam_angles[2]  # 240°
        
        rospy.loginfo(f"Beam angles: beam0={beam0_angle*180/pi:.1f}°, beam1={beam1_angle*180/pi:.1f}°, beam2={beam2_angle*180/pi:.1f}°")
        
        # Calculate beam positions on valve circumference
        beam0_x = center_x + self.valve_radius * cos(beam0_angle)
        beam0_y = center_y + self.valve_radius * sin(beam0_angle)
        beam1_x = center_x + self.valve_radius * cos(beam1_angle)
        beam1_y = center_y + self.valve_radius * sin(beam1_angle)
        beam2_x = center_x + self.valve_radius * cos(beam2_angle)
        beam2_y = center_y + self.valve_radius * sin(beam2_angle)
        
        # Calculate manipulator target positions based on description
        # Left manipulator (claw1): between beam0 and beam1, close to beam1
        # Right manipulator (claw2): between beam1 and beam2, close to beam2
        
        # Calculate gap center angles
        # For gap between beam0(0°) and beam1(120°), center angle is 60°
        # For gap between beam1(120°) and beam2(240°), center angle is 180°
        gap01_angle = beam0_angle + pi/3  # 60° = π/3
        gap12_angle = beam1_angle + pi/3  # 180° = beam1_angle + π/3
        
        # Ensure angles are within [0, 2π] range
        gap01_angle = gap01_angle % (2*pi)
        gap12_angle = gap12_angle % (2*pi)
        
        # Bias factor for approaching beam1 and beam2 (0.4 means 40% toward target beam, staying within gap)
        beam_bias = 0.4
        
        # Left manipulator angle: between beam0 and beam1, close to beam1
        # Offset from gap01_angle toward beam1_angle direction
        angle_diff_01 = ((beam1_angle - gap01_angle + pi) % (2*pi)) - pi
        claw1_angle = gap01_angle + beam_bias * angle_diff_01
        
        # Right manipulator angle: between beam1 and beam2, close to beam2
        # Offset from gap12_angle toward beam2_angle direction
        angle_diff_12 = ((beam2_angle - gap12_angle + pi) % (2*pi)) - pi
        claw2_angle = gap12_angle + beam_bias * angle_diff_12
        
        # Handle angle normalization
        claw1_angle = claw1_angle % (2*pi)
        claw2_angle = claw2_angle % (2*pi)
        
        rospy.loginfo(f"Claw target angles: claw1={claw1_angle*180/pi:.1f}° (beam0-beam1, near beam1)")
        rospy.loginfo(f"                   claw2={claw2_angle*180/pi:.1f}° (beam1-beam2, near beam2)")
        
        # Calculate claw target positions (on valve circumference)
        claw1_target_x = center_x + self.valve_radius * cos(claw1_angle)
        claw1_target_y = center_y + self.valve_radius * sin(claw1_angle)
        claw2_target_x = center_x + self.valve_radius * cos(claw2_angle)
        claw2_target_y = center_y + self.valve_radius * sin(claw2_angle)
        
        # Calculate end-effector center position (midpoint of two claws)
        end_effector_center_x = (claw1_target_x + claw2_target_x) / 2.0
        end_effector_center_y = (claw1_target_y + claw2_target_y) / 2.0
        end_effector_center_z = self.grasp_height
        
        # Calculate end-effector orientation: pointing toward valve center
        dx_to_center = center_x - end_effector_center_x
        dy_to_center = center_y - end_effector_center_y
        end_effector_yaw = atan2(dy_to_center, dx_to_center)
        
        # Apply insertion_offset for smooth insertion
        # For counterclockwise rotation, offset direction should be tangential
        tangent_x = -sin(end_effector_yaw)
        tangent_y = cos(end_effector_yaw)
        
        insertion_offset_x = self.rotation_direction * self.insertion_offset * tangent_x
        insertion_offset_y = self.rotation_direction * self.insertion_offset * tangent_y
        
        # Adjust end-effector and claw positions
        end_effector_center_x += insertion_offset_x
        end_effector_center_y += insertion_offset_y
        claw1_target_x += insertion_offset_x
        claw1_target_y += insertion_offset_y
        claw2_target_x += insertion_offset_x
        claw2_target_y += insertion_offset_y
        
        # 计算body CoG目标位置
        body_yaw = end_effector_yaw
        cos_yaw = cos(body_yaw)
        sin_yaw = sin(body_yaw)
        
        global_offset_x = (cos_yaw * self.end_effector_offset_x - 
                          sin_yaw * self.end_effector_offset_y)
        global_offset_y = (sin_yaw * self.end_effector_offset_x + 
                          cos_yaw * self.end_effector_offset_y)
        global_offset_z = self.end_effector_offset_z
        
        body_target_x = end_effector_center_x - global_offset_x
        body_target_y = end_effector_center_y - global_offset_y
        body_target_z = end_effector_center_z - global_offset_z
        
        # 验证：确保机体大部分处于阀门外侧
        body_to_center_distance = ((body_target_x - center_x)**2 + (body_target_y - center_y)**2)**0.5
        if body_to_center_distance < self.valve_radius:
            rospy.logwarn(f"机体距离阀门中心过近: {body_to_center_distance:.3f}m < {self.valve_radius:.3f}m")
            rospy.logwarn("可能需要调整end_effector_offset参数")
        
        # 存储claw位置信息
        self._claw_positions = {
            'claw1_target': (claw1_target_x, claw1_target_y, self.grasp_height),
            'claw2_target': (claw2_target_x, claw2_target_y, self.grasp_height),
            'claw1_angle': claw1_angle,
            'claw2_angle': claw2_angle,
            'beam0_position': (beam0_x, beam0_y, center_z),
            'beam1_position': (beam1_x, beam1_y, center_z),
            'beam2_position': (beam2_x, beam2_y, center_z),
            'gap01_angle': gap01_angle,
            'gap12_angle': gap12_angle,
        }
        
        rospy.loginfo(f"符合用户描述的开阀门方式:")
        rospy.loginfo(f"  Beam0 (X轴正方向): [{beam0_x:.3f}, {beam0_y:.3f}] @ {beam0_angle*180/pi:.1f}°")
        rospy.loginfo(f"  Beam1 (120°): [{beam1_x:.3f}, {beam1_y:.3f}] @ {beam1_angle*180/pi:.1f}°")
        rospy.loginfo(f"  Beam2 (240°): [{beam2_x:.3f}, {beam2_y:.3f}] @ {beam2_angle*180/pi:.1f}°")
        rospy.loginfo(f"  左侧manipulator (claw1): [{claw1_target_x:.3f}, {claw1_target_y:.3f}] @ {claw1_angle*180/pi:.1f}°")
        rospy.loginfo(f"  右侧manipulator (claw2): [{claw2_target_x:.3f}, {claw2_target_y:.3f}] @ {claw2_angle*180/pi:.1f}°")
        rospy.loginfo(f"  End-effector center: [{end_effector_center_x:.3f}, {end_effector_center_y:.3f}, {end_effector_center_z:.3f}]")
        rospy.loginfo(f"  Body target: [{body_target_x:.3f}, {body_target_y:.3f}, {body_target_z:.3f}]")
        rospy.loginfo(f"  Body yaw: {body_yaw:.3f} rad ({body_yaw*180/pi:.1f}°)")
        rospy.loginfo(f"  机体到阀门中心距离: {body_to_center_distance:.3f}m (应该 > {self.valve_radius:.3f}m)")
        rospy.loginfo(f"  旋转方向: {'逆时针' if self.rotation_direction > 0 else '顺时针'}")
        
        return ((body_target_x, body_target_y, body_target_z), 
                body_yaw, 
                (end_effector_center_x, end_effector_center_y, end_effector_center_z))
    
    def set_start_position(self, start_pos):
        """Set start position and generate trajectory"""
        # Calculate target position and attitude
        self._target_body_pos, self._target_body_yaw, self._target_end_effector_pos = self.calculate_grasp_position_and_yaw()
        
        # Generate trajectory to target position
        self._trajectory = PolynomialTrajectory(self.approach_duration)
        self._trajectory.generate_trajectory(start_pos, self._target_body_pos)
        
        rospy.loginfo(f"Generated align trajectory: from {start_pos} to {self._target_body_pos}")
        
    def get_next_position_and_yaw(self):
        """Get next position and yaw angle"""
        if self._trajectory is None:
            return None
            
        pos = self._trajectory.evaluate()
        if pos is None:
            return None
            
        # During trajectory, yaw angle can be interpolated or keep target yaw
        return (pos, self._target_body_yaw)
    
    def get_next_position(self):
        """Get next position (compatibility interface)"""
        result = self.get_next_position_and_yaw()
        if result is None:
            return None
        return result[0]
    
    def is_complete(self):
        """Check if trajectory is complete"""
        return self.get_next_position() is None
    
    def get_target_info(self):
        """Get target information including claw positions"""
        return {
            'body_position': self._target_body_pos,
            'body_yaw': self._target_body_yaw,
            'end_effector_position': self._target_end_effector_pos,
            'claw_positions': self._claw_positions
        }
    
    def get_claw_positions_in_global_frame(self, body_position, body_yaw):
        """
        Calculate claw positions in global coordinate system based on body position and attitude
        
        Args:
            body_position: Body CoG position (x, y, z)
            body_yaw: Body yaw angle (radians)
            
        Returns:
            tuple: (claw1_position, claw2_position)
        """
        # Calculate end-effector center position in global coordinate system
        cos_yaw = cos(body_yaw)
        sin_yaw = sin(body_yaw)
        
        global_offset_x = (cos_yaw * self.end_effector_offset_x - 
                          sin_yaw * self.end_effector_offset_y)
        global_offset_y = (sin_yaw * self.end_effector_offset_x + 
                          cos_yaw * self.end_effector_offset_y)
        global_offset_z = self.end_effector_offset_z
        
        end_effector_x = body_position[0] + global_offset_x
        end_effector_y = body_position[1] + global_offset_y
        end_effector_z = body_position[2] + global_offset_z
        
        # Claw positions in end-effector coordinate system (from urdf)
        # Offset relative to end-effector center
        claw1_local_x = 0.0072
        claw1_local_y = 0.08438
        claw1_local_z = -0.0522683
        
        claw2_local_x = 0.00772
        claw2_local_y = -0.08438
        claw2_local_z = -0.0522683
        
        # Transform claw positions to global coordinate system
        # Claw1
        global_claw1_x = end_effector_x + (cos_yaw * claw1_local_x - sin_yaw * claw1_local_y)
        global_claw1_y = end_effector_y + (sin_yaw * claw1_local_x + cos_yaw * claw1_local_y)
        global_claw1_z = end_effector_z + claw1_local_z
        
        # Claw2
        global_claw2_x = end_effector_x + (cos_yaw * claw2_local_x - sin_yaw * claw2_local_y)
        global_claw2_y = end_effector_y + (sin_yaw * claw2_local_x + cos_yaw * claw2_local_y)
        global_claw2_z = end_effector_z + claw2_local_z
        
        return ((global_claw1_x, global_claw1_y, global_claw1_z),
                (global_claw2_x, global_claw2_y, global_claw2_z))


class ValveRotationTrajectory:
    """
    Trajectory generator for rotating around valve center
    
    CRITICAL FIX: 实现末端执行器保持与阀门中心恒定距离的轨迹生成
    无人机中心轨迹确保末端执行器围绕阀门中心做圆周运动
    """
    def __init__(self, rotation_duration, valve_center, end_effector_rotation_radius, start_angle, 
                 rotation_angle=2*pi, grasp_height=None, rotation_direction=1,
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823):
        """
        Args:
            rotation_duration: 旋转持续时间
            valve_center: 阀门中心位置
            end_effector_rotation_radius: 末端执行器围绕阀门中心的旋转半径
            start_angle: 起始角度
            rotation_angle: 旋转角度
            grasp_height: 抓取高度
            rotation_direction: 1 for anticlockwise, -1 for clockwise
            end_effector_offset_x/y/z: 末端执行器相对于无人机中心的偏移
        """
        self.rotation_duration = rotation_duration
        self.valve_center = valve_center
        self.end_effector_rotation_radius = end_effector_rotation_radius  # 末端执行器的旋转半径
        self.start_angle = start_angle
        self.rotation_angle = rotation_angle * rotation_direction
        self.grasp_height = grasp_height
        self.rotation_direction = rotation_direction
        
        # End-effector offset relative to body COG (from urdf file)
        self.end_effector_offset_x = end_effector_offset_x
        self.end_effector_offset_y = end_effector_offset_y
        self.end_effector_offset_z = end_effector_offset_z
        
        # 兼容性：支持新的命名方式
        self.end_effector_distance = end_effector_rotation_radius
        
        self._angle_trajectory = None
        
        rospy.loginfo("=== 固定末端执行器-阀门距离的轨迹生成 ===")
        rospy.loginfo(f"末端执行器旋转半径: {self.end_effector_rotation_radius:.3f}m")
        rospy.loginfo(f"末端执行器偏移: [{end_effector_offset_x:.3f}, {end_effector_offset_y:.3f}, {end_effector_offset_z:.3f}]")
        rospy.loginfo("无人机中心轨迹将确保末端执行器保持与阀门中心恒定距离")
        
    def start_rotation(self):
        """Start rotation trajectory"""
        self.start_trajectory()
    
    def start_trajectory(self):
        """启动轨迹生成"""
        end_angle = self.start_angle + self.rotation_angle
        self._angle_trajectory = PolynomialTrajectory(self.rotation_duration)
        self._angle_trajectory.generate_trajectory(self.start_angle, end_angle)
        
        rospy.loginfo(f"末端执行器固定距离旋转轨迹启动:")
        rospy.loginfo(f"  阀门中心: {self.valve_center}")
        rospy.loginfo(f"  末端执行器旋转半径: {self.end_effector_rotation_radius:.3f}m")
        rospy.loginfo(f"  起始角度: {self.start_angle:.3f} rad")
        rospy.loginfo(f"  旋转角度: {self.rotation_angle:.3f} rad")
        rospy.loginfo(f"  旋转方向: {'逆时针' if self.rotation_direction > 0 else '顺时针'}")
        rospy.loginfo(f"  持续时间: {self.rotation_duration:.1f}s")
        
    def get_next_position_and_yaw(self):
        """
        Get next body position and yaw angle
        
        CRITICAL FIX: 确保末端执行器保持与阀门中心的恒定距离
        无人机中心位置根据末端执行器的目标位置反向计算
        """
        return self.get_next_uav_position_and_yaw()
    
    def get_next_uav_position_and_yaw(self):
        """
        获取下一个无人机位置和朝向
        
        Returns:
            tuple: ((x, y, z), yaw) 或 None（如果轨迹结束）
        """
        if self._angle_trajectory is None:
            return None
            
        current_angle = self._angle_trajectory.evaluate()
        if current_angle is None:
            return None
            
        center_x, center_y, center_z = self.valve_center
        
        # 1. 计算末端执行器的目标位置（围绕阀门中心的圆周运动）
        end_effector_target_x = center_x + self.end_effector_rotation_radius * cos(current_angle)
        end_effector_target_y = center_y + self.end_effector_rotation_radius * sin(current_angle)
        
        # 2. 计算末端执行器的目标高度
        if self.grasp_height is not None:
            # 如果指定了抓取高度，使用它作为无人机高度
            uav_target_z = self.grasp_height
            end_effector_target_z = uav_target_z + self.end_effector_offset_z
        else:
            # 否则使用阀门高度作为末端执行器高度
            end_effector_target_z = center_z
            uav_target_z = end_effector_target_z - self.end_effector_offset_z
        
        # 3. 计算末端执行器的目标朝向（始终指向阀门中心）
        dx_to_center = center_x - end_effector_target_x
        dy_to_center = center_y - end_effector_target_y
        end_effector_target_yaw = atan2(dy_to_center, dx_to_center)
        
        # 4. 无人机朝向与末端执行器朝向相同
        uav_target_yaw = end_effector_target_yaw
        
        # 5. 根据末端执行器目标位置反向计算无人机中心位置
        # 末端执行器位置 = 无人机位置 + 旋转后的偏移向量
        cos_yaw = cos(uav_target_yaw)
        sin_yaw = sin(uav_target_yaw)
        
        # 将末端执行器偏移从机体坐标系转换到世界坐标系
        world_offset_x = (cos_yaw * self.end_effector_offset_x - 
                         sin_yaw * self.end_effector_offset_y)
        world_offset_y = (sin_yaw * self.end_effector_offset_x + 
                         cos_yaw * self.end_effector_offset_y)
        world_offset_z = self.end_effector_offset_z
        
        # 反向计算无人机中心位置
        uav_target_x = end_effector_target_x - world_offset_x
        uav_target_y = end_effector_target_y - world_offset_y
        # uav_target_z 已经在步骤2中计算
        
        # 6. 验证计算结果
        # 正向验证：从计算的无人机位置推导末端执行器位置
        verify_ee_x = uav_target_x + world_offset_x
        verify_ee_y = uav_target_y + world_offset_y
        verify_ee_z = uav_target_z + world_offset_z
        
        # 检查末端执行器到阀门中心的距离是否保持恒定
        verify_distance = math.sqrt((verify_ee_x - center_x)**2 + 
                                  (verify_ee_y - center_y)**2)
        
        distance_error = abs(verify_distance - self.end_effector_rotation_radius)
        if distance_error > 0.001:  # 1mm 误差阈值
            rospy.logwarn(f"末端执行器距离误差: {distance_error:.4f}m")
            rospy.logwarn(f"目标距离: {self.end_effector_rotation_radius:.3f}m, 实际距离: {verify_distance:.3f}m")
        
        return ((uav_target_x, uav_target_y, uav_target_z), uav_target_yaw)
        
    def get_current_angle(self):
        """获取当前角度"""
        if self._angle_trajectory is None:
            return None
        return self._angle_trajectory.evaluate()
        
    def is_complete(self):
        """Check if trajectory is complete"""
        return self.get_next_position() is None
        
    def get_next_position(self):
        """Get next position (compatibility interface)"""
        result = self.get_next_position_and_yaw()
        if result is None:
            return None
        return result[0]
        
    def get_trajectory_info(self):
        """
        获取轨迹信息
        
        Returns:
            dict: 轨迹参数信息
        """
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
        
        step_size = math.sqrt((current_uav_pos[0] - previous_uav_pos[0])**2 + 
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
        actual_distance = math.sqrt((actual_ee_x - valve_x)**2 + (actual_ee_y - valve_y)**2)
        
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


# 兼容性类：为了不破坏现有代码
class ConstantDistanceValveRotationTrajectory(ValveRotationTrajectory):
    """
    恒定距离旋转轨迹生成器（兼容性类）
    实际上是 ValveRotationTrajectory 的别名
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


def create_constant_distance_trajectory(current_uav_pos, current_uav_yaw, valve_center, 
                                      rotation_angle, rotation_duration, 
                                      end_effector_offset_x=0.246, end_effector_offset_y=0.0, 
                                      end_effector_offset_z=0.0743823, rotation_direction=1):
    """
    创建恒定距离旋转轨迹的便捷函数
    
    Args:
        current_uav_pos: 当前无人机位置 (x, y, z)
        current_uav_yaw: 当前无人机朝向（弧度）
        valve_center: 阀门中心位置 (x, y, z)
        rotation_angle: 旋转角度（弧度）
        rotation_duration: 旋转持续时间（秒）
        end_effector_offset_x/y/z: 末端执行器偏移
        rotation_direction: 旋转方向（1=逆时针，-1=顺时针）
    
    Returns:
        ValveRotationTrajectory: 轨迹生成器对象
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
    end_effector_distance = math.sqrt((current_ee_x - valve_x)**2 + (current_ee_y - valve_y)**2)
    
    # 计算起始角度
    start_angle = atan2(current_ee_y - valve_y, current_ee_x - valve_x)
    
    rospy.loginfo("=== 创建恒定距离旋转轨迹 ===")
    rospy.loginfo(f"当前无人机位置: {current_uav_pos}")
    rospy.loginfo(f"当前末端执行器位置: [{current_ee_x:.3f}, {current_ee_y:.3f}]")
    rospy.loginfo(f"阀门中心: {valve_center}")
    rospy.loginfo(f"计算得到的末端执行器距离: {end_effector_distance:.3f}m")
    rospy.loginfo(f"起始角度: {start_angle:.3f}rad ({start_angle*180/pi:.1f}°)")
    
    # 创建轨迹生成器
    trajectory = ValveRotationTrajectory(
        rotation_duration=rotation_duration,
        valve_center=valve_center,
        end_effector_rotation_radius=end_effector_distance,
        start_angle=start_angle,
        rotation_angle=rotation_angle,
        grasp_height=current_uav_pos[2],  # 保持当前高度
        rotation_direction=rotation_direction,
        end_effector_offset_x=end_effector_offset_x,
        end_effector_offset_y=end_effector_offset_y,
        end_effector_offset_z=end_effector_offset_z
    )
    
    return trajectory
    
if __name__ == "__main__":
    rospy.init_node("valve_manipulation_trajectory_demo")
    rate = rospy.Rate(50)
    
    # Parameters
    valve_center = (0.5, 0.3, 0.2)
    valve_pose_yaw = 0.0  # Valve orientation
    start_pos = (0.2, 0.1, 0.2)  # Start position
    rotation_direction = 1  # 1 for anticlockwise, -1 for clockwise
    insertion_offset = 0.02  # offset for smooth insertion

    # End-effector offset parameters (from urdf file)
    end_effector_offset_x = 0.246    # End-effector 24.6cm in front of body
    end_effector_offset_y = 0.0      # No Y offset
    end_effector_offset_z = 0.0743823  # End-effector 7.4cm above body COG
    
    rospy.loginfo("=== Valve manipulation trajectory demo (grasp beams 2&3) ===")
    rospy.loginfo(f"Rotation direction: {'anticlockwise' if rotation_direction > 0 else 'clockwise'} (valve vector considered)")
    rospy.loginfo(f"Insertion offset: {insertion_offset:.3f}m")
    
    # Stage 1: Align to grasp point
    rospy.loginfo("Stage 1: Align to grasp point (beams 2&3)")
    align_traj = AlignToGraspTrajectory(
        approach_duration=3.0,
        valve_center=valve_center,
        valve_pose_yaw=valve_pose_yaw,
        grasp_height=0.2,
        valve_radius=0.1225,
        valve_beam_width=0.05,
        end_effector_offset_x=end_effector_offset_x,
        end_effector_offset_y=end_effector_offset_y,
        end_effector_offset_z=end_effector_offset_z,
        rotation_direction=rotation_direction,
        insertion_offset=insertion_offset
    )
    
    # Set start position and generate trajectory
    align_traj.set_start_position(start_pos)
    
    # Execute align trajectory
    while not rospy.is_shutdown() and not align_traj.is_complete():
        result = align_traj.get_next_position_and_yaw()
        if result is None:
            break
        pos, yaw = result
        rospy.loginfo(f"Align stage - pos: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], yaw: {yaw:.3f}")
        rate.sleep()
    
    rospy.loginfo("Align stage complete")
    
    # Get target info after align complete
    target_info = align_traj.get_target_info()
    final_body_pos = target_info['body_position']
    final_body_yaw = target_info['body_yaw']
    
    rospy.loginfo(f"Final body position: {final_body_pos}")
    rospy.loginfo(f"Final body yaw: {final_body_yaw:.3f}")
    
    # Stage 2: Rotate around valve center
    rospy.loginfo("Stage 2: Rotate around valve center")
    
    # Calculate current end-effector position from aligned body position
    cos_yaw = cos(final_body_yaw)
    sin_yaw = sin(final_body_yaw)
    
    global_offset_x = (cos_yaw * end_effector_offset_x - 
                      sin_yaw * end_effector_offset_y)
    global_offset_y = (sin_yaw * end_effector_offset_x + 
                      cos_yaw * end_effector_offset_y)
    
    current_end_effector_x = final_body_pos[0] + global_offset_x
    current_end_effector_y = final_body_pos[1] + global_offset_y
    current_end_effector_z = final_body_pos[2] + end_effector_offset_z
    
    # Calculate rotation radius and start angle
    dx = current_end_effector_x - valve_center[0]
    dy = current_end_effector_y - valve_center[1]
    end_effector_rotation_radius = (dx**2 + dy**2)**0.5  # 末端执行器到阀门中心的距离
    start_angle = atan2(dy, dx)
    
    rospy.loginfo(f"末端执行器旋转参数:")
    rospy.loginfo(f"  末端执行器当前位置: [{current_end_effector_x:.3f}, {current_end_effector_y:.3f}, {current_end_effector_z:.3f}]")
    rospy.loginfo(f"  阀门中心: {valve_center}")
    rospy.loginfo(f"  末端执行器旋转半径: {end_effector_rotation_radius:.3f}m")
    rospy.loginfo(f"  起始角度: {start_angle:.3f} rad ({start_angle*180/pi:.1f}°)")
    
    # Create rotation trajectory directly
    rotation_traj = ValveRotationTrajectory(
        rotation_duration=8.0,
        valve_center=valve_center,
        end_effector_rotation_radius=end_effector_rotation_radius,  # 使用末端执行器旋转半径
        start_angle=start_angle,
        rotation_angle=2*pi,  # Full circle
        grasp_height=0.2,
        rotation_direction=rotation_direction,
        end_effector_offset_x=end_effector_offset_x,
        end_effector_offset_y=end_effector_offset_y,
        end_effector_offset_z=end_effector_offset_z
    )
    
    # Start rotation trajectory
    rotation_traj.start_rotation()
    
    # Execute rotation trajectory
    while not rospy.is_shutdown() and not rotation_traj.is_complete():
        result = rotation_traj.get_next_position_and_yaw()
        if result is None:
            break
        pos, yaw = result
        rospy.loginfo(f"Rotation stage - pos: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], yaw: {yaw:.3f}")
        rate.sleep()
    
    rospy.loginfo("Rotation stage complete")
    rospy.loginfo("=== Valve manipulation trajectory demo complete ===")