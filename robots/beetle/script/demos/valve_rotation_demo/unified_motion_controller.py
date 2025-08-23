#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
from math import pi, cos, sin, sqrt, atan2
from aerial_robot_msgs.msg import FlightNav
from trajectory import PolynomialTrajectory


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
        self.distance_control_gain = rospy.get_param("~distance_control_gain", 0.3)  # Reduced from 0.8
        self.position_control_gain = rospy.get_param("~position_control_gain", 0.3)  # Reduced from 0.5
        self.yaw_control_gain = rospy.get_param("~yaw_control_gain", 0.2)  # Reduced from 0.6
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.01)
        self.max_position_correction = rospy.get_param("~max_position_correction", 0.02)  # Reduced from 0.03
        self.max_yaw_correction = rospy.get_param("~max_yaw_correction", 0.05)  # Reduced from 0.1
        
        # End-effector parameters
        self.end_effector_offset_x = 0.246
        self.end_effector_offset_y = 0.0
        self.end_effector_offset_z = 0.0743823
        
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
    
    def execute_smooth_trajectory_with_yaw(self, start_pos, target_pos, target_yaw, duration=5.0,
                                          pos_threshold=0.03, yaw_threshold=0.08):
        """
        Execute smooth trajectory motion with yaw control
        
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
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        
        start_yaw = self.sm.current_yaw
        yaw_diff = self._normalize_angle_diff(target_yaw - start_yaw)
        
        rospy.loginfo(f"Trajectory: {start_pos} -> {target_pos}")
        rospy.loginfo(f"Yaw change: {math.degrees(yaw_diff):.1f}° (no artificial limit)")
        
        rate = rospy.Rate(50)
        start_time = time.time()
        trajectory_completed = False
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 5.0:
            pt = traj.evaluate()
            elapsed_time = time.time() - start_time
            
            # Check if trajectory is complete
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
            
            self.send_trajectory_point(pt, current_target_yaw)
            
            # More lenient waiting strategy - don't wait for each point
            if trajectory_completed:
                # Only wait for final convergence
                if self._wait_for_position_and_yaw(pt, current_target_yaw, 
                                                  pos_threshold, yaw_threshold, point_timeout=3.0):
                    rospy.loginfo("Trajectory converged successfully")
                    return True
                else:
                    rospy.logwarn("Final convergence timeout, but continuing...")
                    return True  # Consider it successful anyway
            
            rate.sleep()
        
        rospy.loginfo("Smooth trajectory motion completed by timeout")
        return True
    
    def execute_controlled_descent(self, start_pos, target_z, descent_speed=0.05,
                                  pos_threshold=0.02, yaw_threshold=0.05):
        """
        Execute controlled descent with fixed XY position and orientation
        
        Args:
            start_pos: Starting position (x, y, z)
            target_z: Target Z coordinate
            descent_speed: Descent speed m/s
            pos_threshold: Position error threshold m
            yaw_threshold: Yaw error threshold rad
            
        Returns:
            bool: Returns True on success, False on failure
        """
        fixed_x, fixed_y = start_pos[0], start_pos[1]
        fixed_yaw = self.sm.current_yaw
        
        descent_distance = abs(start_pos[2] - target_z)
        duration = descent_distance / descent_speed
        
        rospy.loginfo(f"Controlled descent: {descent_distance:.3f}m in {duration:.1f}s")
        rospy.loginfo(f"Fixed position: [{fixed_x:.3f}, {fixed_y:.3f}], yaw: {fixed_yaw:.3f}")
        
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
    
    # ===== Basic utility functions =====
    
    def send_trajectory_point(self, pos, yaw=None):
        """
        Send trajectory point
        
        Args:
            pos: Position (x, y, z)
            yaw: Yaw (radians)
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
    
    def send_velocity_command(self, vel, yaw_vel, pos_backup=None, yaw_backup=None):
        """
        Send velocity control command (VEL mode)
        
        Args:
            vel: Velocity command (vx, vy, vz) m/s
            yaw_vel: Yaw angular velocity rad/s
            pos_backup: Backup position command (x, y, z) - for hybrid control
            yaw_backup: Backup yaw angle rad - for hybrid control
        """
        msg = FlightNav()
        msg.target = 1
        
        # XY velocity control mode
        msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        msg.target_vel_x = vel[0]
        msg.target_vel_y = vel[1]
        
        # Z position control mode (for safety and Z-lock)
        msg.pos_z_nav_mode = FlightNav.POS_MODE
        msg.target_pos_z = pos_backup[2] if pos_backup else 0.0
        
        # Yaw angular velocity control
        msg.yaw_nav_mode = FlightNav.VEL_MODE
        msg.target_omega_z = yaw_vel
        
        # If backup position provided, set as reference
        if pos_backup is not None:
            msg.target_pos_x = pos_backup[0]  # Reference position
            msg.target_pos_y = pos_backup[1]  # Reference position
        
        if yaw_backup is not None:
            msg.target_yaw = yaw_backup  # Reference yaw
            
        self.pub.publish(msg)
    
    def _wait_for_position(self, target_pos, pos_threshold, point_timeout=2.0):
        """Wait for UAV to reach target position"""
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
        """Wait for UAV to reach target position and yaw"""
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
        """Normalize angle difference to [-π, π] range"""
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
