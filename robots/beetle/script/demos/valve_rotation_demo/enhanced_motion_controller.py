#!/usr/bin/env python
"""
Enhanced Motion Controller with Integrated Alignment Feedback Control
Provides precise trajectory execution with position, yaw, and strict alignment feedback.
"""
import rospy
import time
import math
from aerial_robot_msgs.msg import FlightNav
from trajectory import PolynomialTrajectory


class EnhancedMotionController:
    """
    Enhanced motion controller with integrated alignment feedback control.
    Combines trajectory execution with strict COG-End_effector-Valve alignment.
    """
    
    def __init__(self, state_machine):
        """
        Initialize EnhancedMotionController with reference to state machine.
        
        Args:
            state_machine: Reference to the state machine for accessing current position and yaw
        """
        self.sm = state_machine
        self.pub = self.sm.pub
        
        # Load alignment control parameters
        self.strict_alignment_enabled = rospy.get_param("~strict_alignment_enabled", True)
        self.alignment_correction_enabled = rospy.get_param("~alignment_correction_enabled", True)
        
        # End-effector parameters
        self.end_effector_offset = rospy.get_param("~end_effector_offset", [0.0, 0.0, -0.246])
        self.end_effector_length = 0.246
        
        # Alignment tolerances
        self.alignment_position_tolerance = rospy.get_param("~alignment_position_tolerance", 0.01)
        self.alignment_yaw_tolerance = rospy.get_param("~alignment_yaw_tolerance", 0.05)
        
        # Control parameters
        self.alignment_control_gain = rospy.get_param("~alignment_control_gain", 0.8)
        self.alignment_check_frequency = rospy.get_param("~alignment_check_frequency", 25)
        
        # Trajectory stability parameters
        self.rotation_radius_tolerance = rospy.get_param("~rotation_radius_tolerance", 0.005)
        self.rotation_height_tolerance = rospy.get_param("~rotation_height_tolerance", 0.01)
        
        rospy.loginfo("Enhanced Motion Controller initialized with alignment feedback control")
        rospy.loginfo(f"Strict alignment: {self.strict_alignment_enabled}")
        rospy.loginfo(f"Alignment correction: {self.alignment_correction_enabled}")
        rospy.loginfo(f"Position tolerance: {self.alignment_position_tolerance:.3f}m")
        rospy.loginfo(f"Yaw tolerance: {self.alignment_yaw_tolerance:.3f}rad")
    
    # ==================== ALIGNMENT CONTROL METHODS ====================
    
    def calculate_end_effector_position(self, uav_pos, uav_yaw):
        """
        Calculate end-effector position based on UAV position and orientation.
        
        Args:
            uav_pos: UAV position (x, y, z)
            uav_yaw: UAV yaw angle in radians
            
        Returns:
            tuple: End-effector position (x, y, z)
        """
        # Transform end-effector offset from body frame to world frame
        cos_yaw = math.cos(uav_yaw)
        sin_yaw = math.sin(uav_yaw)
        
        # Apply rotation matrix for yaw
        world_offset_x = (self.end_effector_offset[0] * cos_yaw - 
                         self.end_effector_offset[1] * sin_yaw)
        world_offset_y = (self.end_effector_offset[0] * sin_yaw + 
                         self.end_effector_offset[1] * cos_yaw)
        world_offset_z = self.end_effector_offset[2]
        
        # Calculate end-effector position in world frame
        return (
            uav_pos[0] + world_offset_x,
            uav_pos[1] + world_offset_y,
            uav_pos[2] + world_offset_z
        )
    
    def check_alignment_status(self, uav_pos, uav_yaw, valve_pos):
        """
        Check if COG, end-effector center, and valve center are properly aligned.
        
        Args:
            uav_pos: UAV position (x, y, z)
            uav_yaw: UAV yaw angle in radians
            valve_pos: Valve position (x, y, z)
            
        Returns:
            tuple: (is_aligned, alignment_report)
        """
        if valve_pos is None:
            return False, {}
        
        # Calculate end-effector position
        end_effector_pos = self.calculate_end_effector_position(uav_pos, uav_yaw)
        
        # Calculate alignment metrics
        valve_center = valve_pos
        
        # 1. End-effector to valve distance
        end_effector_to_valve_distance = math.sqrt(
            (end_effector_pos[0] - valve_center[0])**2 + 
            (end_effector_pos[1] - valve_center[1])**2
        )
        
        # 2. UAV to valve distance
        uav_to_valve_distance = math.sqrt(
            (uav_pos[0] - valve_center[0])**2 + 
            (uav_pos[1] - valve_center[1])**2
        )
        
        # 3. Desired yaw (UAV should point toward valve)
        desired_yaw = math.atan2(valve_center[1] - uav_pos[1], valve_center[0] - uav_pos[0])
        yaw_error = abs(self._normalize_angle_diff(uav_yaw - desired_yaw))
        
        # 4. Ideal UAV distance from valve
        height_diff = uav_pos[2] - valve_center[2]
        if abs(height_diff) < self.end_effector_length:
            ideal_uav_distance = math.sqrt(self.end_effector_length**2 - height_diff**2)
        else:
            ideal_uav_distance = 0.0
        
        distance_error = abs(uav_to_valve_distance - ideal_uav_distance)
        
        # 5. Collinearity check
        if uav_to_valve_distance > 0.001:
            cog_to_ee_vector = (end_effector_pos[0] - uav_pos[0], end_effector_pos[1] - uav_pos[1])
            cog_to_valve_vector = (valve_center[0] - uav_pos[0], valve_center[1] - uav_pos[1])
            
            # Calculate angle between vectors
            dot_product = (cog_to_ee_vector[0] * cog_to_valve_vector[0] + 
                          cog_to_ee_vector[1] * cog_to_valve_vector[1])
            
            ee_magnitude = math.sqrt(cog_to_ee_vector[0]**2 + cog_to_ee_vector[1]**2)
            valve_magnitude = math.sqrt(cog_to_valve_vector[0]**2 + cog_to_valve_vector[1]**2)
            
            if ee_magnitude > 0.001 and valve_magnitude > 0.001:
                cos_angle = dot_product / (ee_magnitude * valve_magnitude)
                cos_angle = max(-1.0, min(1.0, cos_angle))
                collinearity_angle = math.acos(cos_angle)
            else:
                collinearity_angle = 0.0
        else:
            collinearity_angle = 0.0
        
        # Determine alignment status
        position_aligned = end_effector_to_valve_distance < self.alignment_position_tolerance
        yaw_aligned = yaw_error < self.alignment_yaw_tolerance
        distance_aligned = distance_error < self.alignment_position_tolerance
        collinear = collinearity_angle < self.alignment_yaw_tolerance
        
        is_aligned = position_aligned and yaw_aligned and distance_aligned and collinear
        
        # Create alignment report
        alignment_report = {
            'is_aligned': is_aligned,
            'end_effector_to_valve_distance': end_effector_to_valve_distance,
            'uav_to_valve_distance': uav_to_valve_distance,
            'ideal_uav_distance': ideal_uav_distance,
            'distance_error': distance_error,
            'yaw_error': yaw_error,
            'desired_yaw': desired_yaw,
            'collinearity_angle': collinearity_angle,
            'position_aligned': position_aligned,
            'yaw_aligned': yaw_aligned,
            'distance_aligned': distance_aligned,
            'collinear': collinear,
            'end_effector_pos': end_effector_pos,
            'valve_center': valve_center,
            'uav_pos': uav_pos,
            'uav_yaw': uav_yaw
        }
        
        return is_aligned, alignment_report
    
    def calculate_alignment_correction(self, alignment_report):
        """
        Calculate position and yaw corrections to improve alignment.
        
        Args:
            alignment_report: Alignment report from check_alignment_status
            
        Returns:
            tuple: (position_correction, yaw_correction)
        """
        if not alignment_report or alignment_report['is_aligned']:
            return (0.0, 0.0, 0.0), 0.0
        
        # Extract data
        uav_pos = alignment_report['uav_pos']
        valve_center = alignment_report['valve_center']
        desired_yaw = alignment_report['desired_yaw']
        current_yaw = alignment_report['uav_yaw']
        ideal_distance = alignment_report['ideal_uav_distance']
        
        # Calculate position corrections
        direction_to_valve = math.atan2(valve_center[1] - uav_pos[1], valve_center[0] - uav_pos[0])
        
        # Calculate ideal UAV position
        ideal_uav_x = valve_center[0] - ideal_distance * math.cos(direction_to_valve)
        ideal_uav_y = valve_center[1] - ideal_distance * math.sin(direction_to_valve)
        
        # Position correction with gain
        pos_correction_x = (ideal_uav_x - uav_pos[0]) * self.alignment_control_gain
        pos_correction_y = (ideal_uav_y - uav_pos[1]) * self.alignment_control_gain
        pos_correction_z = 0.0
        
        # Limit correction magnitude
        max_correction = 0.02  # 2cm max
        correction_magnitude = math.sqrt(pos_correction_x**2 + pos_correction_y**2)
        if correction_magnitude > max_correction:
            scale_factor = max_correction / correction_magnitude
            pos_correction_x *= scale_factor
            pos_correction_y *= scale_factor
        
        # Yaw correction
        yaw_correction = self._normalize_angle_diff(desired_yaw - current_yaw) * self.alignment_control_gain
        
        # Limit yaw correction
        max_yaw_correction = 0.1  # ~5.7 degrees
        yaw_correction = max(-max_yaw_correction, min(max_yaw_correction, yaw_correction))
        
        return (pos_correction_x, pos_correction_y, pos_correction_z), yaw_correction
    
    def apply_alignment_correction(self, trajectory_pos, trajectory_yaw, valve_pos):
        """
        Apply alignment correction to trajectory position and yaw.
        
        Args:
            trajectory_pos: Original trajectory position (x, y, z)
            trajectory_yaw: Original trajectory yaw
            valve_pos: Valve position (x, y, z)
            
        Returns:
            tuple: (corrected_pos, corrected_yaw)
        """
        if not self.strict_alignment_enabled or valve_pos is None:
            return trajectory_pos, trajectory_yaw
        
        current_pos = self.sm.get_current_position()
        if current_pos is None:
            return trajectory_pos, trajectory_yaw
        
        # Check alignment status
        is_aligned, alignment_report = self.check_alignment_status(current_pos, self.sm.current_yaw, valve_pos)
        
        if is_aligned or not self.alignment_correction_enabled:
            return trajectory_pos, trajectory_yaw
        
        # Calculate corrections
        pos_correction, yaw_correction = self.calculate_alignment_correction(alignment_report)
        
        # Apply corrections
        corrected_pos = (
            trajectory_pos[0] + pos_correction[0],
            trajectory_pos[1] + pos_correction[1],
            trajectory_pos[2] + pos_correction[2]
        )
        corrected_yaw = trajectory_yaw + yaw_correction
        
        return corrected_pos, corrected_yaw
    
    # ==================== ENHANCED TRAJECTORY METHODS ====================
    
    def execute_alignment_controlled_rotation(self, rotation_trajectory, valve_pos, 
                                            alignment_check_interval=0.04):
        """
        Execute rotation trajectory with strict alignment control.
        
        Args:
            rotation_trajectory: Rotation trajectory object
            valve_pos: Valve position (x, y, z)
            alignment_check_interval: Time interval for alignment checks (seconds)
            
        Returns:
            dict: Execution statistics
        """
        if not self.strict_alignment_enabled:
            return self._execute_standard_rotation(rotation_trajectory)
        
        rospy.loginfo("=== EXECUTING ALIGNMENT-CONTROLLED ROTATION ===")
        
        # Statistics tracking
        stats = {
            'alignment_violations': 0,
            'max_position_error': 0.0,
            'max_yaw_error': 0.0,
            'corrections_applied': 0,
            'execution_time': 0.0
        }
        
        rate = rospy.Rate(50)  # Main control loop
        last_alignment_check = time.time()
        start_time = time.time()
        
        while not rospy.is_shutdown() and not rotation_trajectory.is_complete():
            current_time = time.time()
            
            # Get next trajectory point
            result = rotation_trajectory.get_next_position_and_yaw()
            if result is None:
                break
            
            trajectory_pos, trajectory_yaw = result
            
            # Apply alignment correction if needed
            if current_time - last_alignment_check >= alignment_check_interval:
                corrected_pos, corrected_yaw = self.apply_alignment_correction(
                    trajectory_pos, trajectory_yaw, valve_pos)
                
                # Check if correction was applied
                if (corrected_pos != trajectory_pos or corrected_yaw != trajectory_yaw):
                    stats['corrections_applied'] += 1
                    
                    # Update statistics
                    current_pos = self.sm.get_current_position()
                    if current_pos is not None:
                        _, alignment_report = self.check_alignment_status(current_pos, self.sm.current_yaw, valve_pos)
                        if not alignment_report['is_aligned']:
                            stats['alignment_violations'] += 1
                            stats['max_position_error'] = max(
                                stats['max_position_error'], 
                                alignment_report['end_effector_to_valve_distance']
                            )
                            stats['max_yaw_error'] = max(
                                stats['max_yaw_error'], 
                                alignment_report['yaw_error']
                            )
                
                # Send corrected command
                self.send_trajectory_point(corrected_pos, corrected_yaw)
                last_alignment_check = current_time
            else:
                # Send original trajectory command
                self.send_trajectory_point(trajectory_pos, trajectory_yaw)
            
            rate.sleep()
        
        stats['execution_time'] = time.time() - start_time
        
        rospy.loginfo("=== ALIGNMENT-CONTROLLED ROTATION COMPLETED ===")
        rospy.loginfo(f"Execution time: {stats['execution_time']:.1f}s")
        rospy.loginfo(f"Alignment violations: {stats['alignment_violations']}")
        rospy.loginfo(f"Corrections applied: {stats['corrections_applied']}")
        rospy.loginfo(f"Max position error: {stats['max_position_error']:.3f}m")
        rospy.loginfo(f"Max yaw error: {stats['max_yaw_error']:.3f}rad")
        
        return stats
    
    def execute_pre_rotation_alignment(self, valve_pos, timeout=10.0):
        """
        Execute pre-rotation alignment to ensure optimal starting position.
        
        Args:
            valve_pos: Valve position (x, y, z)
            timeout: Maximum alignment time (seconds)
            
        Returns:
            bool: True if alignment achieved, False if timeout
        """
        if not self.strict_alignment_enabled:
            rospy.loginfo("Strict alignment disabled - skipping pre-rotation alignment")
            return True
        
        rospy.loginfo("=== PRE-ROTATION ALIGNMENT PHASE ===")
        rospy.loginfo("Ensuring COG-End_effector-Valve collinearity while maintaining current height")
        
        start_time = time.time()
        rate = rospy.Rate(self.alignment_check_frequency)
        last_report_time = start_time
        
        # Store initial height and maintain it throughout alignment
        initial_pos = self.sm.get_current_position()
        if initial_pos is None:
            rospy.logerr("Failed to get initial position for alignment")
            return False
        
        fixed_height = initial_pos[2]  # Fix height during alignment
        rospy.loginfo(f"Fixed height during alignment: {fixed_height:.3f}m")
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            current_pos = self.sm.get_current_position()
            if current_pos is None:
                rospy.logwarn("Lost position data during pre-rotation alignment")
                continue
            
            # Check alignment status
            is_aligned, alignment_report = self.check_alignment_status(
                current_pos, self.sm.current_yaw, valve_pos)
            
            if is_aligned:
                rospy.loginfo("Pre-rotation alignment achieved successfully")
                return True
            
            # Apply corrections if enabled
            if self.alignment_correction_enabled:
                pos_correction, yaw_correction = self.calculate_alignment_correction(alignment_report)
                
                # CRITICAL FIX: During pre-rotation alignment, strictly maintain fixed height
                # Only adjust XY position and yaw, absolutely no Z movement
                corrected_x = current_pos[0] + pos_correction[0]
                corrected_y = current_pos[1] + pos_correction[1]
                corrected_z = fixed_height  # FIXED: Use fixed height, not current height
                corrected_yaw = self.sm.current_yaw + yaw_correction
                
                # Log the correction being applied
                if abs(pos_correction[0]) > 0.001 or abs(pos_correction[1]) > 0.001 or abs(yaw_correction) > 0.001:
                    rospy.loginfo(f"Alignment correction: XY=[{pos_correction[0]:.3f}, {pos_correction[1]:.3f}], yaw={yaw_correction:.3f}")
                
                self.send_trajectory_point((corrected_x, corrected_y, corrected_z), corrected_yaw)
            
            # Progress reporting
            current_time = time.time()
            if current_time - last_report_time >= 2.0:
                rospy.loginfo(f"Pre-rotation alignment progress:")
                rospy.loginfo(f"  - End-effector to valve: {alignment_report['end_effector_to_valve_distance']:.3f}m")
                rospy.loginfo(f"  - Yaw error: {alignment_report['yaw_error']:.3f}rad")
                rospy.loginfo(f"  - Collinearity angle: {alignment_report['collinearity_angle']:.3f}rad")
                last_report_time = current_time
            
            rate.sleep()
        
        rospy.logwarn("Pre-rotation alignment timeout - proceeding with current position")
        return False
    
    # ==================== STANDARD TRAJECTORY METHODS ====================
    
    def _execute_standard_rotation(self, rotation_trajectory):
        """Execute standard rotation without alignment control."""
        rospy.loginfo("Executing standard rotation (alignment control disabled)")
        
        rate = rospy.Rate(50)
        start_time = time.time()
        
        while not rospy.is_shutdown() and not rotation_trajectory.is_complete():
            result = rotation_trajectory.get_next_position_and_yaw()
            if result is None:
                break
            
            pos, yaw = result
            self.send_trajectory_point(pos, yaw)
            rate.sleep()
        
        execution_time = time.time() - start_time
        return {'execution_time': execution_time}
    
    def execute_poly_motion_with_feedback(self, start, target, avg_speed, 
                                          pos_threshold=0.05, yaw_threshold=0.1, timeout=30):
        """
        Execute polynomial motion with closed-loop feedback control.
        """
        distance = math.sqrt((target[0]-start[0])**2 + (target[1]-start[1])**2 + (target[2]-start[2])**2)
        duration = distance / max(avg_speed, 0.05)
        
        rospy.loginfo(f"Executing trajectory with feedback: {distance:.2f}m in {duration:.1f}s")
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start, target)
        
        rate = rospy.Rate(50)
        start_time = time.time()
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            pt = traj.evaluate()
            if pt is None:
                rospy.loginfo("Trajectory completed successfully")
                return True
            
            self.send_trajectory_point(pt, None)
            
            if not self._wait_for_position(pt, pos_threshold, point_timeout=2.0):
                rospy.logwarn(f"Timeout waiting for trajectory point {pt}")
            
            rate.sleep()
        
        rospy.logerr(f"Trajectory timed out after {timeout}s")
        return False
    
    def execute_controlled_descent(self, start_pos, target_z, descent_speed=0.05,
                                  pos_threshold=0.02, yaw_threshold=0.05):
        """Execute controlled descent with fixed XY position and yaw."""
        fixed_x, fixed_y = start_pos[0], start_pos[1]
        fixed_yaw = self.sm.current_yaw
        
        descent_distance = abs(start_pos[2] - target_z)
        duration = descent_distance / descent_speed
        
        rospy.loginfo(f"Controlled descent: {descent_distance:.3f}m in {duration:.1f}s")
        
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
            
            # Check if reached target
            current_pos = self.sm.get_current_position()
            if current_pos and current_pos[2] <= target_z + 0.01:
                rospy.loginfo("Reached target descent height")
                return True
            
            rate.sleep()
        
        rospy.loginfo("Controlled descent completed")
        return True
    
    def send_trajectory_point(self, pos, yaw=None):
        """Send a single trajectory point."""
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
    
    # ==================== UTILITY METHODS ====================
    
    def _wait_for_position(self, target_pos, pos_threshold, point_timeout=2.0):
        """Wait for UAV to reach target position within threshold."""
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
    
    def _normalize_angle_diff(self, angle_diff):
        """Normalize angle difference to [-π, π] range."""
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff
