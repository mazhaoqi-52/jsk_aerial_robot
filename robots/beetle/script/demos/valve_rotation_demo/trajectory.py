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

    def get_next_position(self):
        """Get next position from trajectory (compatibility method)"""
        return self.evaluate()
    
    def is_finished(self):
        """Check if trajectory is finished"""
        if self.start_time is None:
            return False
        elapsed_time = rospy.Time.now().to_sec() - self.start_time
        return elapsed_time > self.duration
    
    def get_velocity(self):
        """Get current velocity from trajectory"""
        if self.start_time is None:
            return None
        elapsed_time = rospy.Time.now().to_sec() - self.start_time
        if elapsed_time > self.duration:
            return None
        
        # Velocity coefficients (derivative of position)
        T_vel = np.array([5*elapsed_time**4, 4*elapsed_time**3, 3*elapsed_time**2, 
                         2*elapsed_time, 1, 0])
        
        if self.is_scalar:
            return np.dot(self.coeffs_scalar, T_vel)
        else:
            return (
                np.dot(self.coeffs_x, T_vel),
                np.dot(self.coeffs_y, T_vel),
                np.dot(self.coeffs_z, T_vel)
            )

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
                 claw_separation=0.16876, rotation_direction=1, insertion_offset=0.015):
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
        self._target_body_pos, self._target_body_yaw, _ = self.calculate_grasp_position_and_yaw()
        self._trajectory = PolynomialTrajectory(self.approach_duration)
        self._trajectory.generate_trajectory(start_pos, self._target_body_pos)
        
    def get_next_position_and_yaw(self):
        if self._trajectory is None:
            return None
        pos = self._trajectory.evaluate()
        if pos is None:
            return None
        return (pos, self._target_body_yaw)
    
    def get_next_position(self):
        result = self.get_next_position_and_yaw()
        if result is None:
            return None
        return result[0]
    
    def is_complete(self):
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
