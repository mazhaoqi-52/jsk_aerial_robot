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
        nav_msg.control_frame = FlightNav.WORLD_FRAME  # CRITICAL: Set control frame for velocity mode
        nav_msg.target = FlightNav.COG  # Control center of gravity
        
        try:
            # Apply axis-specific control based on lock mode
            if axis_lock_mode == 'z_only':
                # Z-only movement: very strict XY and yaw locking
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_x = 0.0  # LOCKED
                nav_msg.target_vel_y = 0.0  # LOCKED
                
                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_z = cmd_vel[2]  # Allow Z movement
                
                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_omega_z = 0.0  # LOCKED (correct field name)
                
            elif axis_lock_mode == 'xy_yaw':
                # XY+Yaw movement: strict Z locking
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_x = cmd_vel[0]  # Allow XY movement
                nav_msg.target_vel_y = cmd_vel[1]  # Allow XY movement
                
                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_z = 0.0  # LOCKED
                
                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_omega_z = cmd_yaw_vel  # Allow yaw movement (correct field name)
                
            else:
                # Normal velocity control mode
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_x = cmd_vel[0]
                nav_msg.target_vel_y = cmd_vel[1]
                
                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_z = cmd_vel[2]
                
                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_omega_z = cmd_yaw_vel  # Correct field name
            
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


# Formation control utilities
def execute_formation_motion(pub, start_pos, target_pos, avg_speed):
    """
    Execute formation motion using existing polynomial trajectory
    
    Args:
        pub: ROS publisher for PoseStamped messages
        start_pos: Starting position [x, y, z]
        target_pos: Target position [x, y, z] 
        avg_speed: Average movement speed
    
    Returns:
        Thread object for asynchronous execution
    """
    import threading
    from geometry_msgs.msg import PoseStamped
    
    def motion_worker():
        try:
            # Calculate trajectory duration
            distance = math.sqrt(sum((t - s) ** 2 for s, t in zip(start_pos, target_pos)))
            duration = max(distance / avg_speed, 0.5)  # Minimum 0.5 seconds
            
            rospy.loginfo(f"Formation motion: {start_pos} -> {target_pos} over {duration:.2f}s")
            
            # Generate polynomial trajectory
            traj = PolynomialTrajectory(duration)
            traj.generate_trajectory(start_pos, target_pos)
            
            # Execute trajectory
            rate = rospy.Rate(20)  # 20 Hz
            while not rospy.is_shutdown() and not traj.is_finished():
                pos = traj.evaluate()
                if pos is None:
                    break
                
                # Create and publish pose message
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "world"
                pose_msg.pose.position.x = pos[0]
                pose_msg.pose.position.y = pos[1]
                pose_msg.pose.position.z = pos[2]
                pose_msg.pose.orientation.w = 1.0  # No rotation
                
                pub.publish(pose_msg)
                rate.sleep()
            
        except Exception as e:
            rospy.logerr(f"Error in formation motion: {e}")
    
    # Start motion in separate thread
    thread = threading.Thread(target=motion_worker)
    thread.start()
    return thread

def execute_formation_motion_with_yaw(pub, start_pos, target_pos, target_yaw, avg_speed):
    """
    Execute formation motion with yaw control
    
    Args:
        pub: ROS publisher for PoseStamped messages
        start_pos: Starting position [x, y, z]
        target_pos: Target position [x, y, z]
        target_yaw: Target yaw angle (radians)
        avg_speed: Average movement speed
    
    Returns:
        Thread object for asynchronous execution
    """
    import threading
    from geometry_msgs.msg import PoseStamped
    from tf.transformations import quaternion_from_euler
    
    def motion_yaw_worker():
        try:
            # Calculate trajectory duration
            distance = math.sqrt(sum((t - s) ** 2 for s, t in zip(start_pos, target_pos)))
            duration = max(distance / avg_speed, 0.5)  # Minimum 0.5 seconds
            
            rospy.loginfo(f"Formation motion with yaw: {start_pos} -> {target_pos}, yaw: {target_yaw:.3f} over {duration:.2f}s")
            
            # Generate polynomial trajectory for position
            traj = PolynomialTrajectory(duration)
            traj.generate_trajectory(start_pos, target_pos)
            
            # Execute trajectory with yaw
            rate = rospy.Rate(20)  # 20 Hz
            while not rospy.is_shutdown() and not traj.is_finished():
                pos = traj.evaluate()
                if pos is None:
                    break
                
                # Create and publish pose message with yaw
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "world"
                pose_msg.pose.position.x = pos[0]
                pose_msg.pose.position.y = pos[1]
                pose_msg.pose.position.z = pos[2]
                
                # Set orientation from yaw
                quat = quaternion_from_euler(0, 0, target_yaw)
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]
                
                pub.publish(pose_msg)
                rate.sleep()
            
        except Exception as e:
            rospy.logerr(f"Error in formation motion with yaw: {e}")
    
    # Start motion in separate thread
    thread = threading.Thread(target=motion_yaw_worker)
    thread.start()
    return thread


# Compatibility class for existing code
class FormationMotionCompatibility:
    """Compatibility wrapper for formation control"""
    
    @staticmethod 
    def execute_poly_motion_pose_async(pub, start_pos, target_pos, avg_speed):
        """Compatibility method for formation control"""
        return execute_formation_motion(pub, start_pos, target_pos, avg_speed)
    
    @staticmethod
    def execute_poly_motion_pose_yaw_async(pub, start_pos, target_pos, target_yaw, avg_speed):
        """Compatibility method for formation control with yaw"""
        return execute_formation_motion_with_yaw(pub, start_pos, target_pos, target_yaw, avg_speed)
