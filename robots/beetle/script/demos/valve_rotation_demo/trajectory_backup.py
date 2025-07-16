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
        
        # Calculate body CoG target position
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
        
        # Verification: ensure body is mostly outside valve
        body_to_center_distance = ((body_target_x - center_x)**2 + (body_target_y - center_y)**2)**0.5
        if body_to_center_distance < self.valve_radius:
            rospy.logwarn(f"Body too close to valve center: {body_to_center_distance:.3f}m < {self.valve_radius:.3f}m")
            rospy.logwarn("May need to adjust end_effector_offset parameters")
        
        # Store claw position information
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
        
        rospy.loginfo(f"Valve opening method according to description:")
        rospy.loginfo(f"  Beam0 (positive X-axis): [{beam0_x:.3f}, {beam0_y:.3f}] @ {beam0_angle*180/pi:.1f}°")
        rospy.loginfo(f"  Beam1 (120°): [{beam1_x:.3f}, {beam1_y:.3f}] @ {beam1_angle*180/pi:.1f}°")
        rospy.loginfo(f"  Beam2 (240°): [{beam2_x:.3f}, {beam2_y:.3f}] @ {beam2_angle*180/pi:.1f}°")
        rospy.loginfo(f"  Left manipulator (claw1): [{claw1_target_x:.3f}, {claw1_target_y:.3f}] @ {claw1_angle*180/pi:.1f}°")
        rospy.loginfo(f"  Right manipulator (claw2): [{claw2_target_x:.3f}, {claw2_target_y:.3f}] @ {claw2_angle*180/pi:.1f}°")
        rospy.loginfo(f"  End-effector center: [{end_effector_center_x:.3f}, {end_effector_center_y:.3f}, {end_effector_center_z:.3f}]")
        rospy.loginfo(f"  Body target: [{body_target_x:.3f}, {body_target_y:.3f}, {body_target_z:.3f}]")
        rospy.loginfo(f"  Body yaw: {body_yaw:.3f} rad ({body_yaw*180/pi:.1f}°)")
        rospy.loginfo(f"  Body to valve center distance: {body_to_center_distance:.3f}m (should be > {self.valve_radius:.3f}m)")
        rospy.loginfo(f"  Rotation direction: {'counterclockwise' if self.rotation_direction > 0 else 'clockwise'}")
        
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
        
        global_offset_x = (cos_yaw * self.end_effector_offset_x - sin_yaw * self.end_effector_offset_y)
        global_offset_y = (sin_yaw * self.end_effector_offset_x + cos_yaw * self.end_effector_offset_y)
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
    
    CRITICAL FIX: Implement trajectory generation that maintains constant distance between end-effector and valve center
    UAV center trajectory ensures end-effector performs circular motion around valve center
    """
    def __init__(self, rotation_duration, valve_center, end_effector_rotation_radius, start_angle, 
                 rotation_angle=2*pi, grasp_height=None, rotation_direction=1,
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823):
        """
        Args:
            rotation_duration: Rotation duration
            valve_center: Valve center position
            end_effector_rotation_radius: End-effector rotation radius around valve center
            start_angle: Starting angle
            rotation_angle: Rotation angle
            grasp_height: Grasp height
            rotation_direction: 1 for anticlockwise, -1 for clockwise
            end_effector_offset_x/y/z: End-effector offset relative to UAV center
        """
        self.rotation_duration = rotation_duration
        self.valve_center = valve_center
        self.end_effector_rotation_radius = end_effector_rotation_radius  # End-effector rotation radius
        self.start_angle = start_angle
        self.rotation_angle = rotation_angle * rotation_direction
        self.grasp_height = grasp_height
        self.rotation_direction = rotation_direction
        
        # End-effector offset relative to body COG (from urdf file)
        self.end_effector_offset_x = end_effector_offset_x
        self.end_effector_offset_y = end_effector_offset_y
        self.end_effector_offset_z = end_effector_offset_z
        
        # Compatibility: support new naming convention
        self.end_effector_distance = end_effector_rotation_radius
        
        self._angle_trajectory = None
        
        rospy.loginfo("=== Fixed End-Effector-Valve Distance Trajectory Generation ===")
        rospy.loginfo(f"End-effector rotation radius: {self.end_effector_rotation_radius:.3f}m")
        rospy.loginfo(f"End-effector offset: [{end_effector_offset_x:.3f}, {end_effector_offset_y:.3f}, {end_effector_offset_z:.3f}]")
        rospy.loginfo("UAV center trajectory will ensure end-effector maintains constant distance from valve center")
        
    def start_rotation(self):
        """Start rotation trajectory"""
        self.start_trajectory()
    
    def start_trajectory(self):
        """Start trajectory generation"""
        end_angle = self.start_angle + self.rotation_angle
        self._angle_trajectory = PolynomialTrajectory(self.rotation_duration)
        self._angle_trajectory.generate_trajectory(self.start_angle, end_angle)
        
        rospy.loginfo(f"End-effector fixed distance rotation trajectory started:")
        rospy.loginfo(f"  Valve center: {self.valve_center}")
        rospy.loginfo(f"  End-effector rotation radius: {self.end_effector_rotation_radius:.3f}m")
        rospy.loginfo(f"  Starting angle: {self.start_angle:.3f} rad")
        rospy.loginfo(f"  Rotation angle: {self.rotation_angle:.3f} rad")
        rospy.loginfo(f"  Rotation direction: {'counterclockwise' if self.rotation_direction > 0 else 'clockwise'}")
        rospy.loginfo(f"  Duration: {self.rotation_duration:.1f}s")
        
    def get_next_position_and_yaw(self):
        """
        Get next body position and yaw angle
        
        CRITICAL FIX: Ensure end-effector maintains constant distance from valve center
        UAV center position is calculated inversely from end-effector target position
        """
        return self.get_next_uav_position_and_yaw()
    
    def get_next_uav_position_and_yaw(self):
        """
        Get next UAV position and orientation
        
        Returns:
            tuple: ((x, y, z), yaw) or None (if trajectory ends)
        """
        if self._angle_trajectory is None:
            return None
            
        current_angle = self._angle_trajectory.evaluate()
        if current_angle is None:
            return None
            
        center_x, center_y, center_z = self.valve_center
        
        # 1. Calculate end-effector target position (circular motion around valve center)
        end_effector_target_x = center_x + self.end_effector_rotation_radius * cos(current_angle)
        end_effector_target_y = center_y + self.end_effector_rotation_radius * sin(current_angle)
        
        # 2. Calculate end-effector target height
        if self.grasp_height is not None:
            # If grasp height is specified, use it as UAV height
            uav_target_z = self.grasp_height
            end_effector_target_z = uav_target_z + self.end_effector_offset_z
        else:
            # Otherwise use valve height as end-effector height
            end_effector_target_z = center_z
            uav_target_z = end_effector_target_z - self.end_effector_offset_z
        
        # 3. Calculate end-effector target orientation (always pointing toward valve center)
        dx_to_center = center_x - end_effector_target_x
        dy_to_center = center_y - end_effector_target_y
        end_effector_target_yaw = atan2(dy_to_center, dx_to_center)
        
        # 4. UAV orientation is same as end-effector orientation
        uav_target_yaw = end_effector_target_yaw
        
        # 5. Calculate UAV center position inversely from end-effector target position
        # End-effector position = UAV position + rotated offset vector
        cos_yaw = cos(uav_target_yaw)
        sin_yaw = sin(uav_target_yaw)
        
        # Transform end-effector offset from body coordinate system to world coordinate system
        world_offset_x = (cos_yaw * self.end_effector_offset_x - 
                         sin_yaw * self.end_effector_offset_y)
        world_offset_y = (sin_yaw * self.end_effector_offset_x + 
                         cos_yaw * self.end_effector_offset_y)
        world_offset_z = self.end_effector_offset_z
        
        # Calculate UAV center position inversely
        uav_target_x = end_effector_target_x - world_offset_x
        uav_target_y = end_effector_target_y - world_offset_y
        # uav_target_z already calculated in step 2
        
        # 6. CRITICAL FIX: 增强验证计算结果
        # Forward verification: derive end-effector position from calculated UAV position
        verify_ee_x = uav_target_x + world_offset_x
        verify_ee_y = uav_target_y + world_offset_y
        verify_ee_z = uav_target_z + world_offset_z
        
        # Check if end-effector to valve center distance remains constant
        verify_distance = math.sqrt((verify_ee_x - center_x)**2 + 
                                  (verify_ee_y - center_y)**2)
        
        distance_error = abs(verify_distance - self.end_effector_rotation_radius)
        if distance_error > 0.001:  # 1mm error threshold
            rospy.logwarn(f"End-effector distance error: {distance_error:.4f}m")
            rospy.logwarn(f"Target distance: {self.end_effector_rotation_radius:.3f}m, actual distance: {verify_distance:.3f}m")
            rospy.logwarn(f"Trajectory angle: {current_angle:.3f}rad ({current_angle*180/pi:.1f}°)")
            rospy.logwarn(f"Target EE position: [{end_effector_target_x:.3f}, {end_effector_target_y:.3f}]")
            rospy.logwarn(f"Calculated UAV position: [{uav_target_x:.3f}, {uav_target_y:.3f}]")
            rospy.logwarn(f"World offset: [{world_offset_x:.3f}, {world_offset_y:.3f}]")
            rospy.logwarn(f"Verification EE position: [{verify_ee_x:.3f}, {verify_ee_y:.3f}]")
            
            # CRITICAL FIX: 检查是否是初始距离设置错误导致的
            if self.end_effector_rotation_radius < 0.1:  # 如果半径异常小
                rospy.logerr(f"CRITICAL: End-effector rotation radius is abnormally small: {self.end_effector_rotation_radius:.3f}m")
                rospy.logerr("This suggests the initial distance calculation was wrong")
                rospy.logerr("Consider using valve radius as minimum distance")
        
        return ((uav_target_x, uav_target_y, uav_target_z), uav_target_yaw)
        
    def get_current_angle(self):
        """Get current angle"""
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
        Get trajectory information
        
        Returns:
            dict: Trajectory parameter information
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
        Check trajectory continuity to ensure smooth UAV motion
        
        Args:
            previous_uav_pos: Previous UAV position
            current_uav_pos: Current UAV position
            max_step_size: Maximum allowed step size (meters)
            
        Returns:
            bool: Whether trajectory is continuous
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
        
        world_offset_x = (cos_yaw * self.end_effector_offset_x - sin_yaw * self.end_effector_offset_y)
        world_offset_y = (sin_yaw * self.end_effector_offset_x + cos_yaw * self.end_effector_offset_y)
        
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
        ValveRotationTrajectory: 轨迹生成器对象
    """
    
    # CRITICAL FIX: 正确计算当前末端执行器位置
    valve_x, valve_y, _ = valve_center
    
    rospy.loginfo("=== Creating Constant Distance Rotation Trajectory ===")
    
    # CRITICAL FIX: 预先验证轨迹参数
    validation_result = validate_trajectory_parameters(
        current_uav_pos, current_uav_yaw, valve_center,
        end_effector_offset_x, end_effector_offset_y
    )
    
    if not validation_result['valid']:
        rospy.logerr("TRAJECTORY PARAMETER VALIDATION FAILED!")
        rospy.logerr("Cannot create safe trajectory with current parameters")
        rospy.logerr("Please fix the errors before proceeding")
        return None
    
    # CRITICAL FIX: 使用诊断函数验证计算
    (ee_x, ee_y), calculated_distance = diagnose_end_effector_calculation(
        current_uav_pos, current_uav_yaw, valve_center, 
        end_effector_offset_x, end_effector_offset_y
    )
    
    # CRITICAL FIX: 分析插入位置合理性
    insertion_position_ok = analyze_insertion_position(current_uav_pos, valve_center)
    
    # 使用诊断函数的结果
    current_ee_x, current_ee_y = ee_x, ee_y
    end_effector_distance = calculated_distance
    
    # CRITICAL FIX: 如果距离异常小，强制使用阀门半径
    valve_radius = 0.1225
    if end_effector_distance < valve_radius * 0.9:  # 如果小于阀门半径的90%
        rospy.logerr(f"CRITICAL: Calculated distance ({end_effector_distance:.3f}m) is too small!")
        rospy.logerr(f"This indicates end-effector is inside valve (radius: {valve_radius:.3f}m)")
        rospy.logerr("FORCING distance to valve radius for safety")
        end_effector_distance = valve_radius
        
        # 重新计算合理的末端执行器位置
        # 保持当前角度但使用阀门半径距离
        current_angle = atan2(current_ee_y - valve_y, current_ee_x - valve_x)
        current_ee_x = valve_x + valve_radius * cos(current_angle)
        current_ee_y = valve_y + valve_radius * sin(current_angle)
        
        rospy.logwarn(f"Corrected end-effector position: [{current_ee_x:.3f}, {current_ee_y:.3f}]")
        rospy.logwarn(f"Corrected distance: {end_effector_distance:.3f}m")
        
        # CRITICAL FIX: 如果插入位置不合理，提供额外的错误信息
        if not insertion_position_ok:
            rospy.logerr("=== ROOT CAUSE ANALYSIS ===")
            rospy.logerr("The UAV is positioned too close to the valve center")
            rospy.logerr("This is likely caused by:")
            rospy.logerr("1. Insertion process failure - UAV moved too close during insertion")
            rospy.logerr("2. Position feedback error - incorrect UAV position readings")
            rospy.logerr("3. Control system drift - UAV drifted closer after insertion")
            rospy.logerr("RECOMMENDATION: Re-execute insertion process or manually reposition UAV")
            
            # 使用新的位置修正建议函数
            correction_info = suggest_uav_position_correction(
                current_uav_pos, current_uav_yaw, valve_center, 
                end_effector_offset_x, end_effector_offset_y
            )
            
            # 提供紧急恢复方案
            emergency_recovery_info = emergency_position_recovery(
                current_uav_pos, valve_center, end_effector_offset_x
            )
            
            rospy.logerr("=== IMMEDIATE ACTION REQUIRED ===")
            rospy.logerr("To fix this issue:")
            rospy.logerr(f"1. Move UAV from {current_uav_pos} to {correction_info['ideal_uav_pos']}")
            rospy.logerr(f"2. Movement vector: {correction_info['movement_vector']}")
            rospy.logerr(f"3. Movement distance: {correction_info['movement_distance']:.3f}m")
            rospy.logerr("4. After repositioning, restart the rotation trajectory")
            rospy.logerr("=== END ROOT CAUSE ANALYSIS ===")
    else:
        # CRITICAL FIX: 额外验证计算结果
        analyze_insertion_position(current_uav_pos, valve_center, end_effector_distance)
    
    # Calculate starting angle
    start_angle = atan2(current_ee_y - valve_y, current_ee_x - valve_x)
    
    rospy.loginfo(f"=== TRAJECTORY CREATION SUMMARY ===")
    rospy.loginfo(f"Current UAV position: {current_uav_pos}")
    rospy.loginfo(f"Current UAV yaw: {current_uav_yaw:.3f}rad ({current_uav_yaw*180/pi:.1f}°)")
    rospy.loginfo(f"End-effector offset: [{end_effector_offset_x:.3f}, {end_effector_offset_y:.3f}, {end_effector_offset_z:.3f}]")
    rospy.loginfo(f"Calculated end-effector position: [{current_ee_x:.3f}, {current_ee_y:.3f}]")
    rospy.loginfo(f"Valve center: {valve_center}")
    rospy.loginfo(f"End-effector distance: {end_effector_distance:.3f}m")
    rospy.loginfo(f"Starting angle: {start_angle:.3f}rad ({start_angle*180/pi:.1f}°)")
    
    # Create trajectory generator
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
    
def diagnose_end_effector_calculation(uav_pos, uav_yaw, valve_center, 
                                    end_effector_offset_x=0.246, end_effector_offset_y=0.0):
    """
    诊断末端执行器位置计算的正确性
    
    Args:
        uav_pos: UAV位置 (x, y, z)
        uav_yaw: UAV朝向 (弧度)
        valve_center: 阀门中心 (x, y, z)
        end_effector_offset_x/y: 末端执行器偏移
    """
    rospy.loginfo("=== END-EFFECTOR POSITION CALCULATION DIAGNOSIS ===")
    rospy.loginfo(f"UAV position: [{uav_pos[0]:.3f}, {uav_pos[1]:.3f}, {uav_pos[2]:.3f}]")
    rospy.loginfo(f"UAV yaw: {uav_yaw:.3f}rad ({uav_yaw*180/pi:.1f}°)")
    rospy.loginfo(f"Valve center: [{valve_center[0]:.3f}, {valve_center[1]:.3f}, {valve_center[2]:.3f}]")
    
    # 计算末端执行器位置
    cos_yaw = cos(uav_yaw)
    sin_yaw = sin(uav_yaw)
    
    rospy.loginfo(f"Trigonometric values:")
    rospy.loginfo(f"  cos({uav_yaw:.3f}) = {cos_yaw:.6f}")
    rospy.loginfo(f"  sin({uav_yaw:.3f}) = {sin_yaw:.6f}")
    
    # CRITICAL FIX: 详细分析计算过程
    # 末端执行器偏移 = [0.246, 0.0] 表示在UAV前方24.6cm
    world_offset_x = cos_yaw * end_effector_offset_x - sin_yaw * end_effector_offset_y
    world_offset_y = sin_yaw * end_effector_offset_x + cos_yaw * end_effector_offset_y
    
    rospy.loginfo(f"Offset calculation:")
    rospy.loginfo(f"  world_offset_x = cos({uav_yaw:.3f}) * {end_effector_offset_x:.3f} - sin({uav_yaw:.3f}) * {end_effector_offset_y:.3f}")
    rospy.loginfo(f"                 = {cos_yaw:.6f} * {end_effector_offset_x:.3f} - {sin_yaw:.6f} * {end_effector_offset_y:.3f}")
    rospy.loginfo(f"                 = {world_offset_x:.6f}")
    rospy.loginfo(f"  world_offset_y = sin({uav_yaw:.3f}) * {end_effector_offset_x:.3f} + cos({uav_yaw:.3f}) * {end_effector_offset_y:.3f}")
    rospy.loginfo(f"                 = {sin_yaw:.6f} * {end_effector_offset_x:.3f} + {cos_yaw:.6f} * {end_effector_offset_y:.3f}")
    rospy.loginfo(f"                 = {world_offset_y:.6f}")
    
    # 计算末端执行器位置
    ee_x = uav_pos[0] + world_offset_x
    ee_y = uav_pos[1] + world_offset_y
    
    rospy.loginfo(f"End-effector position calculation:")
    rospy.loginfo(f"  ee_x = {uav_pos[0]:.6f} + {world_offset_x:.6f} = {ee_x:.6f}")
    rospy.loginfo(f"  ee_y = {uav_pos[1]:.6f} + {world_offset_y:.6f} = {ee_y:.6f}")
    
    # 计算到阀门中心的距离
    dx = ee_x - valve_center[0]
    dy = ee_y - valve_center[1]
    distance = math.sqrt(dx**2 + dy**2)
    
    rospy.loginfo(f"Distance to valve center:")
    rospy.loginfo(f"  dx = {ee_x:.6f} - {valve_center[0]:.6f} = {dx:.6f}")
    rospy.loginfo(f"  dy = {ee_y:.6f} - {valve_center[1]:.6f} = {dy:.6f}")
    rospy.loginfo(f"  distance = sqrt({dx:.6f}² + {dy:.6f}²) = {distance:.6f}m")
    
    # CRITICAL FIX: 增强合理性检查
    valve_radius = 0.1225
    uav_to_valve_distance = math.sqrt((uav_pos[0] - valve_center[0])**2 + (uav_pos[1] - valve_center[1])**2)
    
    rospy.loginfo(f"=== REASONABLENESS CHECK ===")
    rospy.loginfo(f"UAV to valve center distance: {uav_to_valve_distance:.6f}m")
    rospy.loginfo(f"End-effector offset magnitude: {end_effector_offset_x:.6f}m")
    rospy.loginfo(f"Expected EE distance range: [{uav_to_valve_distance - end_effector_offset_x:.3f}, {uav_to_valve_distance + end_effector_offset_x:.3f}]m")
    rospy.loginfo(f"Valve radius: {valve_radius:.6f}m")
    
    if distance < valve_radius * 0.8:
        rospy.logerr(f"CRITICAL: Distance ({distance:.3f}m) < 80% of valve radius ({valve_radius:.3f}m)")
        rospy.logerr("This suggests the end-effector is inside the valve, which is impossible!")
        rospy.logerr("Possible causes:")
        rospy.logerr("  1. UAV position is too close to valve center")
        rospy.logerr("  2. UAV yaw calculation is incorrect")
        rospy.logerr("  3. End-effector offset parameters are wrong")
    elif distance > valve_radius * 1.5:
        rospy.logwarn(f"WARNING: Distance ({distance:.3f}m) > 150% of valve radius ({valve_radius:.3f}m)")
        rospy.logwarn("This suggests the end-effector is far from the valve")
    else:
        rospy.loginfo(f"Distance appears reasonable for valve interaction")
    
    # CRITICAL FIX: 检查插入位置合理性
    if uav_to_valve_distance < valve_radius:
        rospy.logerr(f"CRITICAL: UAV ({uav_to_valve_distance:.3f}m) is inside valve radius ({valve_radius:.3f}m)!")
        rospy.logerr("This is impossible - UAV body cannot be inside valve")
        rospy.logerr("The insertion process may have failed or position calculation is wrong")
    
    rospy.loginfo("=== END DIAGNOSIS ===")
    
    return (ee_x, ee_y), distance

def analyze_insertion_position(uav_pos, valve_center, target_ee_distance=0.1225):
    """
    分析插入位置的合理性并提供修正建议
    
    Args:
        uav_pos: UAV位置 (x, y, z)
        valve_center: 阀门中心 (x, y, z)
        target_ee_distance: 目标末端执行器距离（默认为阀门半径）
    """
    rospy.loginfo("=== INSERTION POSITION ANALYSIS ===")
    
    # 计算UAV到阀门中心的距离
    uav_to_valve_distance = math.sqrt((uav_pos[0] - valve_center[0])**2 + 
                                     (uav_pos[1] - valve_center[1])**2)
    
    # 计算UAV相对于阀门中心的角度
    uav_angle = atan2(uav_pos[1] - valve_center[1], uav_pos[0] - valve_center[0])
    
    rospy.loginfo(f"UAV position: [{uav_pos[0]:.3f}, {uav_pos[1]:.3f}, {uav_pos[2]:.3f}]")
    rospy.loginfo(f"Valve center: [{valve_center[0]:.3f}, {valve_center[1]:.3f}, {valve_center[2]:.3f}]")
    rospy.loginfo(f"UAV to valve distance: {uav_to_valve_distance:.3f}m")
    rospy.loginfo(f"UAV angle from valve center: {uav_angle:.3f}rad ({uav_angle*180/pi:.1f}°)")
    
    # 分析位置合理性
    valve_radius = 0.1225
    end_effector_offset_magnitude = 0.246
    
    # 理论上，为了让末端执行器在阀门半径处：
    # UAV应该距离阀门中心约为: valve_radius + end_effector_offset_magnitude = 0.1225 + 0.246 = 0.369m
    # 或者至少: valve_radius + end_effector_offset_magnitude * cos(yaw_difference)
    
    expected_uav_distance_min = valve_radius + end_effector_offset_magnitude * 0.5  # 考虑角度影响
    expected_uav_distance_max = valve_radius + end_effector_offset_magnitude
    
    rospy.loginfo(f"Expected UAV distance range: [{expected_uav_distance_min:.3f}, {expected_uav_distance_max:.3f}]m")
    
    if uav_to_valve_distance < expected_uav_distance_min:
        rospy.logerr(f"CRITICAL: UAV too close to valve center!")
        rospy.logerr(f"Current: {uav_to_valve_distance:.3f}m < Expected min: {expected_uav_distance_min:.3f}m")
        rospy.logerr("This explains why end-effector distance is abnormally small")
        rospy.logerr("Insertion process may have failed or UAV moved too close")
        
        # 计算需要的修正距离
        required_correction = expected_uav_distance_min - uav_to_valve_distance
        rospy.logerr(f"Required correction: move UAV {required_correction:.3f}m away from valve center")
        
        # 建议修正位置
        suggested_uav_x = valve_center[0] + expected_uav_distance_min * cos(uav_angle)
        suggested_uav_y = valve_center[1] + expected_uav_distance_min * sin(uav_angle)
        rospy.logerr(f"Suggested UAV position: [{suggested_uav_x:.3f}, {suggested_uav_y:.3f}, {uav_pos[2]:.3f}]")
        
        return False
    elif uav_to_valve_distance > expected_uav_distance_max * 1.2:
        rospy.logwarn(f"WARNING: UAV may be too far from valve center")
        rospy.logwarn(f"Current: {uav_to_valve_distance:.3f}m > Expected max: {expected_uav_distance_max:.3f}m")
    else:
        rospy.loginfo("UAV distance appears reasonable")
    
    rospy.loginfo("=== END INSERTION POSITION ANALYSIS ===")
    return True

def suggest_uav_position_correction(current_uav_pos, current_uav_yaw, valve_center, 
                                   end_effector_offset_x=0.246, end_effector_offset_y=0.0):
    """
    建议UAV位置修正方案，当UAV过于接近阀门中心时
    
    Args:
        current_uav_pos: 当前UAV位置 (x, y, z)
        current_uav_yaw: 当前UAV朝向 (弧度)
        valve_center: 阀门中心 (x, y, z)
        end_effector_offset_x/y: 末端执行器偏移
        
    Returns:
        dict: 修正建议信息
    """
    rospy.loginfo("=== UAV POSITION CORRECTION SUGGESTION ===")
    
    valve_x, valve_y, valve_z = valve_center
    valve_radius = 0.1225
    
    # 计算当前UAV到阀门中心的距离和角度
    current_uav_distance = math.sqrt((current_uav_pos[0] - valve_x)**2 + 
                                   (current_uav_pos[1] - valve_y)**2)
    current_uav_angle = atan2(current_uav_pos[1] - valve_y, current_uav_pos[0] - valve_x)
    
    # 计算理想的UAV距离（确保末端执行器在阀门半径处）
    # 考虑UAV朝向对末端执行器位置的影响
    cos_yaw = cos(current_uav_yaw)
    sin_yaw = sin(current_uav_yaw)
    
    # 计算在当前朝向下，UAV需要的距离
    # 末端执行器相对于UAV的偏移向量
    ee_offset_world_x = cos_yaw * end_effector_offset_x - sin_yaw * end_effector_offset_y
    ee_offset_world_y = sin_yaw * end_effector_offset_x + cos_yaw * end_effector_offset_y
    
    # 方法1：简单方法 - 确保UAV距离阀门中心足够远
    ideal_uav_distance = valve_radius + end_effector_offset_x
    
    # 方法2：精确方法 - 考虑UAV朝向
    # 首先确定末端执行器应该在的位置（保持当前角度但距离为valve_radius）
    current_ee_angle = atan2(current_uav_pos[1] + ee_offset_world_y - valve_y,
                           current_uav_pos[0] + ee_offset_world_x - valve_x)
    
    target_ee_x = valve_x + valve_radius * cos(current_ee_angle)
    target_ee_y = valve_y + valve_radius * sin(current_ee_angle)
    
    # 反推理想的UAV位置
    ideal_uav_x = target_ee_x - ee_offset_world_x
    ideal_uav_y = target_ee_y - ee_offset_world_y
    
    # 重新计算理想距离
    ideal_uav_distance_precise = math.sqrt((ideal_uav_x - valve_x)**2 + (ideal_uav_y - valve_y)**2)
    
    # 计算需要的移动距离和方向
    movement_x = ideal_uav_x - current_uav_pos[0]
    movement_y = ideal_uav_y - current_uav_pos[1]
    movement_distance = math.sqrt(movement_x**2 + movement_y**2)
    movement_angle = atan2(movement_y, movement_x)
    
    correction_info = {
        'current_uav_pos': current_uav_pos,
        'current_uav_distance': current_uav_distance,
        'current_uav_angle': current_uav_angle,
        'ideal_uav_pos': (ideal_uav_x, ideal_uav_y, current_uav_pos[2]),
        'ideal_uav_distance': ideal_uav_distance_precise,
        'movement_vector': (movement_x, movement_y),
        'movement_distance': movement_distance,
        'movement_angle': movement_angle,
        'valve_center': valve_center,
        'valve_radius': valve_radius
    }
    
    rospy.loginfo(f"Current UAV position: [{current_uav_pos[0]:.3f}, {current_uav_pos[1]:.3f}, {current_uav_pos[2]:.3f}]")
    rospy.loginfo(f"Current UAV distance from valve: {current_uav_distance:.3f}m")
    rospy.loginfo(f"Current UAV angle from valve: {current_uav_angle:.3f}rad ({current_uav_angle*180/pi:.1f}°)")
    rospy.loginfo(f"Current UAV yaw: {current_uav_yaw:.3f}rad ({current_uav_yaw*180/pi:.1f}°)")
    rospy.loginfo(f"")
    rospy.loginfo(f"Ideal UAV position: [{ideal_uav_x:.3f}, {ideal_uav_y:.3f}, {current_uav_pos[2]:.3f}]")
    rospy.loginfo(f"Ideal UAV distance from valve: {ideal_uav_distance_precise:.3f}m")
    rospy.loginfo(f"")
    rospy.loginfo(f"Required movement: [{movement_x:.3f}, {movement_y:.3f}]")
    rospy.loginfo(f"Movement distance: {movement_distance:.3f}m")
    rospy.loginfo(f"Movement direction: {movement_angle:.3f}rad ({movement_angle*180/pi:.1f}°)")
    
    if movement_distance > 0.05:  # 如果需要移动超过5cm
        rospy.logwarn(f"SIGNIFICANT POSITION CORRECTION NEEDED!")
        rospy.logwarn(f"UAV needs to move {movement_distance:.3f}m away from valve center")
        rospy.logwarn(f"This suggests the insertion process failed")
    
    rospy.loginfo("=== END UAV POSITION CORRECTION SUGGESTION ===")
    return correction_info


def validate_trajectory_parameters(uav_pos, uav_yaw, valve_center, 
                                end_effector_offset_x=0.246, end_effector_offset_y=0.0):
    """
    验证轨迹参数的合理性
    
    Args:
        uav_pos: UAV位置 (x, y, z)
        uav_yaw: UAV朝向 (弧度)
        valve_center: 阀门中心 (x, y, z)
        end_effector_offset_x/y: 末端执行器偏移
        
    Returns:
        dict: 验证结果 {'valid': bool, 'warnings': list, 'errors': list}
    """
    rospy.loginfo("=== TRAJECTORY PARAMETER VALIDATION ===")
    
    warnings = []
    errors = []
    
    # 验证1: UAV到阀门中心的距离
    uav_to_valve_distance = math.sqrt((uav_pos[0] - valve_center[0])**2 + 
                                    (uav_pos[1] - valve_center[1])**2)
    
    valve_radius = 0.1225
    min_safe_distance = valve_radius + end_effector_offset_x * 0.5
    max_reasonable_distance = valve_radius + end_effector_offset_x * 1.5
    
    if uav_to_valve_distance < min_safe_distance:
        errors.append(f"UAV too close to valve center: {uav_to_valve_distance:.3f}m < {min_safe_distance:.3f}m")
    elif uav_to_valve_distance > max_reasonable_distance:
        warnings.append(f"UAV may be too far from valve center: {uav_to_valve_distance:.3f}m > {max_reasonable_distance:.3f}m")
    
    # 验证2: 末端执行器位置计算
    cos_yaw = cos(uav_yaw)
    sin_yaw = sin(uav_yaw)
    
    ee_world_x = uav_pos[0] + cos_yaw * end_effector_offset_x - sin_yaw * end_effector_offset_y
    ee_world_y = uav_pos[1] + sin_yaw * end_effector_offset_x + cos_yaw * end_effector_offset_y
    
    ee_to_valve_distance = math.sqrt((ee_world_x - valve_center[0])**2 + 
                                   (ee_world_y - valve_center[1])**2)
    
    if ee_to_valve_distance < valve_radius * 0.8:
        errors.append(f"End-effector inside valve: {ee_to_valve_distance:.3f}m < {valve_radius:.3f}m")
    elif ee_to_valve_distance < valve_radius * 0.95:
        warnings.append(f"End-effector very close to valve inner edge: {ee_to_valve_distance:.3f}m")
    elif ee_to_valve_distance > valve_radius * 1.5:
        warnings.append(f"End-effector far from valve: {ee_to_valve_distance:.3f}m > {valve_radius*1.5:.3f}m")
    
    # 验证3: 朝向合理性
    # 计算从UAV到阀门中心的角度
    uav_to_valve_angle = atan2(valve_center[1] - uav_pos[1], valve_center[0] - uav_pos[0])
    yaw_difference = abs(((uav_yaw - uav_to_valve_angle + pi) % (2*pi)) - pi)
    
    if yaw_difference > pi/2:  # 90度
        warnings.append(f"UAV not facing valve center: yaw difference {yaw_difference:.3f}rad ({yaw_difference*180/pi:.1f}°)")
    
    # 验证4: 高度合理性
    height_difference = abs(uav_pos[2] - valve_center[2])
    if height_difference > 1.0:  # 1米
        warnings.append(f"Large height difference: {height_difference:.3f}m between UAV and valve")
    
    # 总结验证结果
    is_valid = len(errors) == 0
    
    rospy.loginfo(f"Validation results:")
    rospy.loginfo(f"  Valid: {is_valid}")
    rospy.loginfo(f"  Errors: {len(errors)}")
    rospy.loginfo(f"  Warnings: {len(warnings)}")
    
    if errors:
        rospy.logerr("VALIDATION ERRORS:")
        for error in errors:
            rospy.logerr(f"  - {error}")
    
    if warnings:
        rospy.logwarn("VALIDATION WARNINGS:")
        for warning in warnings:
            rospy.logwarn(f"  - {warning}")
    
    rospy.loginfo("=== END TRAJECTORY PARAMETER VALIDATION ===")
    
    return {
        'valid': is_valid,
        'warnings': warnings,
        'errors': errors,
        'uav_to_valve_distance': uav_to_valve_distance,
        'ee_to_valve_distance': ee_to_valve_distance,
        'yaw_difference': yaw_difference
    }
    


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


def emergency_position_recovery(current_uav_pos, valve_center, 
                              end_effector_offset_x=0.246, 
                              emergency_distance_multiplier=1.2):
    """
    紧急位置恢复建议，当UAV过于接近阀门中心时的安全撤退方案
    
    Args:
        current_uav_pos: 当前UAV位置 (x, y, z)
        valve_center: 阀门中心 (x, y, z)
        end_effector_offset_x: 末端执行器X偏移
        emergency_distance_multiplier: 紧急距离倍数
        
    Returns:
        dict: 恢复方案信息
    """
    rospy.logerr("=== EMERGENCY POSITION RECOVERY ===")
    
    valve_radius = 0.1225
    current_distance = math.sqrt((current_uav_pos[0] - valve_center[0])**2 + 
                               (current_uav_pos[1] - valve_center[1])**2)
    
    # 计算安全距离
    safe_distance = (valve_radius + end_effector_offset_x) * emergency_distance_multiplier
    
    # 计算当前UAV相对于阀门中心的角度
    current_angle = atan2(current_uav_pos[1] - valve_center[1], 
                         current_uav_pos[0] - valve_center[0])
    
    # 计算紧急撤退位置
    emergency_x = valve_center[0] + safe_distance * cos(current_angle)
    emergency_y = valve_center[1] + safe_distance * sin(current_angle)
    emergency_z = current_uav_pos[2]  # 保持当前高度
    
    # 计算需要的移动距离
    movement_x = emergency_x - current_uav_pos[0]
    movement_y = emergency_y - current_uav_pos[1]
    movement_distance = math.sqrt(movement_x**2 + movement_y**2)
    
    recovery_info = {
        'current_pos': current_uav_pos,
        'current_distance': current_distance,
        'safe_distance': safe_distance,
        'emergency_pos': (emergency_x, emergency_y, emergency_z),
        'movement_vector': (movement_x, movement_y, 0),
        'movement_distance': movement_distance,
        'current_angle': current_angle
    }
    
    rospy.logerr(f"EMERGENCY SITUATION DETECTED:")
    rospy.logerr(f"  Current UAV distance: {current_distance:.3f}m")
    rospy.logerr(f"  Required safe distance: {safe_distance:.3f}m")
    rospy.logerr(f"  Distance deficit: {safe_distance - current_distance:.3f}m")
    rospy.logerr(f"")
    rospy.logerr(f"EMERGENCY RECOVERY PLAN:")
    rospy.logerr(f"  1. IMMEDIATELY move UAV to: [{emergency_x:.3f}, {emergency_y:.3f}, {emergency_z:.3f}]")
    rospy.logerr(f"  2. Movement vector: [{movement_x:.3f}, {movement_y:.3f}, 0.000]")
    rospy.logerr(f"  3. Movement distance: {movement_distance:.3f}m")
    rospy.logerr(f"  4. After repositioning, wait 2-3 seconds for stabilization")
    rospy.logerr(f"  5. Re-run insertion process from safe distance")
    rospy.logerr(f"")
    rospy.logerr(f"CRITICAL: Do not attempt rotation until UAV is repositioned!")
    rospy.logerr("=== END EMERGENCY POSITION RECOVERY ===")
    
    return recovery_info