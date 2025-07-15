#!/usr/bin/env python
import rospy
import numpy as np
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
        
        # CRITICAL FIX: 根据您的描述，无人机应该沿X轴正方向飞向阀门
        # beam0与X轴正方向重合，左侧manipulator卡在beam0与beam1之间靠近beam1
        # 右侧manipulator卡在beam1与beam2之间靠近beam2
        rospy.loginfo("AlignToGraspTrajectory: 初始化符合用户描述的开阀门方式")
        rospy.loginfo("  - 无人机沿X轴正方向飞向阀门")
        rospy.loginfo("  - 左侧manipulator: beam0与beam1之间靠近beam1")
        rospy.loginfo("  - 右侧manipulator: beam1与beam2之间靠近beam2")
        rospy.loginfo("  - 逆时针旋转，机体大部分处于阀门外侧")
        
    def calculate_grasp_position_and_yaw(self):
        """
        CRITICAL FIX: 重新实现符合用户描述的开阀门方式
        - 无人机沿X轴正方向飞向阀门（beam0与X轴正方向重合）
        - 左侧manipulator卡在beam0与beam1之间靠近beam1的一侧
        - 右侧manipulator卡在beam1与beam2之间靠近beam2的一侧
        - 逆时针旋转，机体大部分处于阀门外侧
        """
        center_x, center_y, center_z = self.valve_center
        
        # 计算beam角度：beam0(0°), beam1(120°), beam2(240°)
        beam_angles = [self.valve_pose_yaw + i * 2*pi/3 for i in range(3)]
        beam0_angle = beam_angles[0]  # 0° - 与X轴正方向重合
        beam1_angle = beam_angles[1]  # 120°
        beam2_angle = beam_angles[2]  # 240°
        
        rospy.loginfo(f"Beam angles: beam0={beam0_angle*180/pi:.1f}°, beam1={beam1_angle*180/pi:.1f}°, beam2={beam2_angle*180/pi:.1f}°")
        
        # 计算beam在阀门圆周上的位置
        beam0_x = center_x + self.valve_radius * cos(beam0_angle)
        beam0_y = center_y + self.valve_radius * sin(beam0_angle)
        beam1_x = center_x + self.valve_radius * cos(beam1_angle)
        beam1_y = center_y + self.valve_radius * sin(beam1_angle)
        beam2_x = center_x + self.valve_radius * cos(beam2_angle)
        beam2_y = center_y + self.valve_radius * sin(beam2_angle)
        
        # 根据您的描述，计算manipulator目标位置
        # 左侧manipulator (claw1): beam0与beam1之间，靠近beam1
        # 右侧manipulator (claw2): beam1与beam2之间，靠近beam2
        
        # 计算gap中心角度
        # 对于beam0(0°)和beam1(120°)之间的gap，中心角度是60°
        # 对于beam1(120°)和beam2(240°)之间的gap，中心角度是180°
        gap01_angle = beam0_angle + pi/3  # 60° = π/3
        gap12_angle = beam1_angle + pi/3  # 180° = beam1_angle + π/3
        
        # 确保角度在[0, 2π]范围内
        gap01_angle = gap01_angle % (2*pi)
        gap12_angle = gap12_angle % (2*pi)
        
        # 靠近beam1和beam2的偏移因子（0.4表示40%靠近目标beam，保持在gap内）
        beam_bias = 0.4
        
        # 左侧manipulator角度：beam0与beam1之间靠近beam1
        # 从gap01_angle向beam1_angle方向偏移
        angle_diff_01 = ((beam1_angle - gap01_angle + pi) % (2*pi)) - pi
        claw1_angle = gap01_angle + beam_bias * angle_diff_01
        
        # 右侧manipulator角度：beam1与beam2之间靠近beam2
        # 从gap12_angle向beam2_angle方向偏移
        angle_diff_12 = ((beam2_angle - gap12_angle + pi) % (2*pi)) - pi
        claw2_angle = gap12_angle + beam_bias * angle_diff_12
        
        # 处理角度归一化
        claw1_angle = claw1_angle % (2*pi)
        claw2_angle = claw2_angle % (2*pi)
        
        rospy.loginfo(f"Claw target angles: claw1={claw1_angle*180/pi:.1f}° (beam0-beam1, near beam1)")
        rospy.loginfo(f"                   claw2={claw2_angle*180/pi:.1f}° (beam1-beam2, near beam2)")
        
        # 计算claw目标位置（在阀门圆周上）
        claw1_target_x = center_x + self.valve_radius * cos(claw1_angle)
        claw1_target_y = center_y + self.valve_radius * sin(claw1_angle)
        claw2_target_x = center_x + self.valve_radius * cos(claw2_angle)
        claw2_target_y = center_y + self.valve_radius * sin(claw2_angle)
        
        # 计算end-effector中心位置（两个claw的中点）
        end_effector_center_x = (claw1_target_x + claw2_target_x) / 2.0
        end_effector_center_y = (claw1_target_y + claw2_target_y) / 2.0
        end_effector_center_z = self.grasp_height
        
        # 计算end-effector方向：指向阀门中心
        dx_to_center = center_x - end_effector_center_x
        dy_to_center = center_y - end_effector_center_y
        end_effector_yaw = atan2(dy_to_center, dx_to_center)
        
        # 应用insertion_offset以实现平滑插入
        # 对于逆时针旋转，offset方向应该是切线方向
        tangent_x = -sin(end_effector_yaw)
        tangent_y = cos(end_effector_yaw)
        
        insertion_offset_x = self.rotation_direction * self.insertion_offset * tangent_x
        insertion_offset_y = self.rotation_direction * self.insertion_offset * tangent_y
        
        # 调整end-effector和claw位置
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
    """Trajectory generator for rotating around valve center"""
    def __init__(self, rotation_duration, valve_center, rotation_radius, start_angle, 
                 rotation_angle=2*pi, grasp_height=None, rotation_direction=1,
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823):
        """
        Args:
            rotation_direction: 1 for anticlockwise, -1 for clockwise
        """
        self.rotation_duration = rotation_duration
        self.valve_center = valve_center
        self.rotation_radius = rotation_radius
        self.start_angle = start_angle
        self.rotation_angle = rotation_angle * rotation_direction  # Apply direction to rotation angle
        self.grasp_height = grasp_height
        self.rotation_direction = rotation_direction
        
        # End-effector offset relative to body COG (from urdf file)
        self.end_effector_offset_x = end_effector_offset_x
        self.end_effector_offset_y = end_effector_offset_y
        self.end_effector_offset_z = end_effector_offset_z
        
        self._angle_trajectory = None
        
    def start_rotation(self):
        """Start rotation trajectory"""
        end_angle = self.start_angle + self.rotation_angle
        self._angle_trajectory = PolynomialTrajectory(self.rotation_duration)
        self._angle_trajectory.generate_trajectory(self.start_angle, end_angle)
        
        rospy.loginfo(f"Rotation trajectory started:")
        rospy.loginfo(f"  Valve center: {self.valve_center}")
        rospy.loginfo(f"  Rotation radius: {self.rotation_radius:.3f}m")
        rospy.loginfo(f"  Start angle: {self.start_angle:.3f} rad")
        rospy.loginfo(f"  Rotation angle: {self.rotation_angle:.3f} rad")
        rospy.loginfo(f"  Rotation direction: {'anticlockwise' if self.rotation_direction > 0 else 'clockwise'}")
        rospy.loginfo(f"  Duration: {self.rotation_duration:.1f}s")
        
    def get_next_position_and_yaw(self):
        """Get next body position and yaw angle"""
        if self._angle_trajectory is None:
            return None
            
        current_angle = self._angle_trajectory.evaluate()
        if current_angle is None:
            return None
            
        # Calculate end-effector position
        center_x, center_y, center_z = self.valve_center
        end_effector_x = center_x + self.rotation_radius * cos(current_angle)
        end_effector_y = center_y + self.rotation_radius * sin(current_angle)
        
        # CRITICAL FIX: grasp_height should be the body height, not end-effector height
        # If grasp_height is provided, use it as body height directly
        if self.grasp_height is not None:
            body_z = self.grasp_height  # Use grasp_height as body height directly
            end_effector_z = body_z + self.end_effector_offset_z  # Add offset to get end-effector height
        else:
            end_effector_z = center_z  # Fallback to valve center height
            body_z = end_effector_z - self.end_effector_offset_z
        
        # Calculate end-effector orientation (always facing valve center)
        dx_to_center = center_x - end_effector_x
        dy_to_center = center_y - end_effector_y
        end_effector_yaw = atan2(dy_to_center, dx_to_center)
        
        # Body yaw same as end-effector yaw
        body_yaw = end_effector_yaw
        
        # Transform end-effector offset to global coordinate system
        cos_yaw = cos(body_yaw)
        sin_yaw = sin(body_yaw)
        
        global_offset_x = (cos_yaw * self.end_effector_offset_x - 
                          sin_yaw * self.end_effector_offset_y)
        global_offset_y = (sin_yaw * self.end_effector_offset_x + 
                          cos_yaw * self.end_effector_offset_y)
        global_offset_z = self.end_effector_offset_z
        
        # Calculate body COG position
        body_x = end_effector_x - global_offset_x
        body_y = end_effector_y - global_offset_y
        # body_z is already calculated above based on grasp_height
        
        return ((body_x, body_y, body_z), body_yaw)
    
    def get_next_position(self):
        """Get next position (compatibility interface)"""
        result = self.get_next_position_and_yaw()
        if result is None:
            return None
        return result[0]
    
    def is_complete(self):
        """Check if trajectory is complete"""
        return self.get_next_position() is None
    
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
    radius = (dx**2 + dy**2)**0.5
    start_angle = atan2(dy, dx)
    
    # Create rotation trajectory directly
    rotation_traj = ValveRotationTrajectory(
        rotation_duration=8.0,
        valve_center=valve_center,
        rotation_radius=radius,
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