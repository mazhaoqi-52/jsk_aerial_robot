#!/usr/bin/env python
"""Unified Motion Controller - Integrates trajectory execution and feedback control."""

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
from trajectory import PolynomialTrajectory, AdaptiveTrajectoryPlanner


class UnifiedMotionController:
    """Unified Motion Controller for trajectory execution and feedback control."""
    
    def __init__(self, state_machine):
        """Initialize unified motion controller."""
        self.sm = state_machine
        self.pub = self.sm.pub
        
        # Control parameters
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.02)
        self.yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.05236)
        
        # Alignment control parameters
        self.strict_alignment_enabled = rospy.get_param("~strict_alignment_enabled", True)
        self.alignment_correction_enabled = rospy.get_param("~alignment_correction_enabled", True)
        self.alignment_position_tolerance = rospy.get_param("~alignment_position_tolerance", 0.01)
        self.alignment_yaw_tolerance = rospy.get_param("~alignment_yaw_tolerance", 0.05236)
        self.alignment_control_gain = rospy.get_param("~alignment_control_gain", 0.8)
        
        # Distance feedback control parameters
        self.distance_control_gain = rospy.get_param("~distance_control_gain", 0.5)
        self.position_control_gain = rospy.get_param("~position_control_gain", 0.6)
        self.yaw_control_gain = rospy.get_param("~yaw_control_gain", 0.3)
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.025)
        self.max_position_correction = rospy.get_param("~max_position_correction", 0.03)
        self.max_yaw_correction = rospy.get_param("~max_yaw_correction", 0.05)
        
        # Yaw control parameters
        self.yaw_deadzone = rospy.get_param("~yaw_deadzone", math.radians(3))
        self.yaw_rate_limit = rospy.get_param("~yaw_rate_limit", math.radians(30))
        self.min_yaw_correction_interval = rospy.get_param("~min_yaw_correction_interval", 0.2)
        self.last_yaw_correction_time = 0.0
        
        # Pitch constraint parameters
        self.max_pitch_angle = rospy.get_param("~max_pitch_angle", math.radians(15))
        self.pitch_control_gain = rospy.get_param("~pitch_control_gain", 0.3)
        self.pitch_warning_threshold = rospy.get_param("~pitch_warning_threshold", math.radians(10))
        self.yaw_damping_factor = rospy.get_param("~yaw_damping_factor", 0.7)
        
        # Completion detector
        try:
            from rotation_completion_detector import get_completion_detector
            self.completion_detector = get_completion_detector()
            self.use_completion_detection = True
        except ImportError:
            self.completion_detector = None
            self.use_completion_detection = False
        
        # End-effector parameters
        self.end_effector_offset_x = 0.246
        self.end_effector_offset_y = 0.0
        self.end_effector_offset_z = 0.0743823
        
        # Controller state
        self.control_active = False
        self.trajectory_active = False
        self.rotation_active = False
        self.pid_active = False
        self.final_descent_active = False
        self.return_to_start_active = False
        self.pre_final_descent = True
        
        # Control statistics
        self.control_stats = {
            'total_corrections': 0, 'distance_violations': 0,
            'max_distance_error': 0.0, 'avg_distance_error': 0.0, 'execution_time': 0.0
        }
        
    def _get_valve_surface_height(self):
        """Get valve surface height."""
        if hasattr(self.sm, 'valve_pos') and self.sm.valve_pos is not None:
            return self.sm.valve_pos[2]
        return 1.0
    
    def send_trajectory_point(self, position, yaw=None):
        """Send trajectory point with depth-adaptive control."""
        current_pos = self.sm.get_current_position()
        valve_surface_z = self._get_valve_surface_height()
        actual_insertion_depth = max(0, valve_surface_z - current_pos[2]) if current_pos else 0.0
        
        # Control mode selection by priority
        if hasattr(self, 'disengagement_active') and self.disengagement_active:
            control_mode = "PD_DISENGAGEMENT"
        elif hasattr(self, 'rotation_active') and self.rotation_active:
            control_mode = "PD_ROTATION"
            actual_insertion_depth = 0.0
        elif hasattr(self, 'return_to_start_active') and self.return_to_start_active:
            control_mode = "PD"
        elif hasattr(self, 'final_descent_active') and self.final_descent_active:
            control_mode = "PD_INSERTION"
        elif actual_insertion_depth > 0.02:
            control_mode = "PID_HIGH_PRECISION" if hasattr(self, 'pre_final_descent') and self.pre_final_descent else "PD_INSERTION"
        else:
            # Free space mode selection based on position error
            position_error = math.sqrt(sum((position[i] - current_pos[i])**2 for i in range(3))) if current_pos else 0.0
            if position_error > 0.08:
                control_mode = "PD"
            elif position_error > 0.03:
                control_mode = "PID_CONSERVATIVE"
            else:
                control_mode = "PID_HIGH_PRECISION"
            actual_insertion_depth = 0.0
        
        # Build navigation message
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.header.frame_id = "world"
        nav_msg.control_frame = 0  # WORLD_FRAME
        nav_msg.target = 1         # COG
        
        # Position control
        nav_msg.pos_xy_nav_mode = 2  # POS_MODE
        nav_msg.target_pos_x = position[0]
        nav_msg.target_pos_y = position[1]
        nav_msg.pos_z_nav_mode = 2
        nav_msg.target_pos_z = position[2]
        
        # Attitude control
        nav_msg.roll_nav_mode = 0
        nav_msg.target_roll = 0.0
        nav_msg.target_omega_x = 0.0
        nav_msg.pitch_nav_mode = 2
        nav_msg.target_pitch = 0.0
        nav_msg.target_omega_y = 0.0
        
        # Yaw control with rotation-specific limiting
        if yaw is not None:
            nav_msg.yaw_nav_mode = 2
            nav_msg.target_yaw = yaw
            nav_msg.target_omega_z = 0.0
            
            # Limit yaw corrections during rotation
            if hasattr(self, 'rotation_active') and self.rotation_active and hasattr(self.sm, 'current_yaw'):
                current_yaw = getattr(self.sm, 'current_yaw', 0.0)
                yaw_error = self._normalize_angle_diff(yaw - current_yaw)
                max_yaw_error = math.radians(20.0)
                if abs(yaw_error) > max_yaw_error:
                    nav_msg.target_yaw = current_yaw + math.copysign(max_yaw_error, yaw_error)
        else:
            nav_msg.yaw_nav_mode = 0
            nav_msg.target_yaw = 0.0
            nav_msg.target_omega_z = 0.0
        
        self.pub.publish(nav_msg)
    
    # ===== Basic trajectory execution functions =====
    
    def execute_progressive_precision_trajectory(self, start_pos, target_pos, target_yaw, 
                                                final_pos_threshold=0.025, final_yaw_threshold=0.03):
        """Execute trajectory with progressive precision control."""
        distance = math.sqrt(sum((target_pos[i] - start_pos[i])**2 for i in range(3)))
        
        # Adaptive thresholds based on distance and insertion depth
        current_pos = self.sm.get_current_position()
        valve_surface_z = self._get_valve_surface_height()
        insertion_depth = max(0, valve_surface_z - current_pos[2]) if current_pos else 0.0
        
        if distance < 0.01:  # Very small movement
            if insertion_depth > 0.30:
                pos_threshold = 0.015
            elif insertion_depth > 0.20:
                pos_threshold = 0.012
            elif insertion_depth > 0.05:
                pos_threshold = 0.010
            else:
                pos_threshold = 0.008
            yaw_threshold = final_yaw_threshold * 1.2
            duration = 8.0
        elif distance < 0.05:  # Small movement
            pos_threshold = final_pos_threshold * 2.0
            yaw_threshold = final_yaw_threshold * 1.5
            duration = 3.0
        else:  # Normal movement
            pos_threshold = final_pos_threshold * 3.0
            yaw_threshold = final_yaw_threshold * 2.0
            duration = max(4.0, distance / 0.08)
        
        return self.execute_smooth_trajectory_with_yaw(
            start_pos, target_pos, target_yaw,
            duration=duration,
            pos_threshold=pos_threshold,
            yaw_threshold=yaw_threshold
        )

    def execute_smooth_trajectory_with_yaw(self, start_pos, target_pos, target_yaw, duration=5.0,
                                          pos_threshold=0.03, yaw_threshold=0.02, 
                                          enable_divergence_detection=True):
        """Execute smooth trajectory motion with yaw control and PID feedback."""
        self.start_new_controller('trajectory')
        
        distance = math.sqrt(sum((target_pos[i] - start_pos[i])**2 for i in range(3)))
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        
        # Trajectory speed validation for small movements
        max_allowed_speed = max(0.1, distance / duration) * 2.5
        if distance < 0.05 and duration > 5.0:
            dt_check = 0.033
            check_times = [i * dt_check for i in range(min(20, int(duration / dt_check)))]
            speed_violations = 0
            
            for i, t in enumerate(check_times[1:], 1):
                if hasattr(traj, 'evaluate_at_time'):
                    curr_point = traj.evaluate_at_time(t)
                    prev_point = traj.evaluate_at_time(check_times[i-1])
                    point_distance = math.sqrt(sum((curr_point[j] - prev_point[j])**2 for j in range(3)))
                    if point_distance / dt_check > max_allowed_speed:
                        speed_violations += 1
            
            if speed_violations > len(check_times) * 0.2:
                # Regenerate with safer duration
                duration = duration * 1.3
                traj = PolynomialTrajectory(duration)
                traj.generate_trajectory(start_pos, target_pos)
        
        start_yaw = self.sm.current_yaw
        yaw_diff = self._normalize_angle_diff(target_yaw - start_yaw)
        
        # Depth-adaptive control mode selection
        current_pos = self.sm.get_current_position()
        valve_surface_z = self._get_valve_surface_height()
        actual_insertion_depth = max(0, valve_surface_z - current_pos[2]) if current_pos else 0.0
        
        # Lock depth during rotation phase
        if hasattr(self, 'rotation_active') and self.rotation_active:
            if not hasattr(self, '_rotation_insertion_depth'):
                self._rotation_insertion_depth = max(0.080, actual_insertion_depth)
            actual_insertion_depth = self._rotation_insertion_depth
        elif hasattr(self, '_rotation_insertion_depth'):
            delattr(self, '_rotation_insertion_depth')
        
            # CONTROL MODE SELECTION with precision correction override
            precision_correction_mode = rospy.get_param('/force_precision_correction_mode', False)
            force_rotation_mode = rospy.get_param('/force_rotation_control_mode', False)
            
            # Control mode selection by priority
            if hasattr(self, 'rotation_active') and self.rotation_active:
                control_mode = "PD_ROTATION"
                actual_insertion_depth = 0.0
            elif precision_correction_mode:
                control_mode = "PID_HIGH_PRECISION"
            elif force_rotation_mode:
                actual_insertion_depth = 0.35
                control_mode = "PD_ROTATION"
            elif actual_insertion_depth > 0.02:
                control_mode = "PD_INSERTION"
            else:
                control_mode = "PID_HIGH_PRECISION"
        else:
            control_mode = "PID_STANDARD"
            actual_insertion_depth = 0.0
        
        # Progressive mode transition
        if not hasattr(self, '_previous_control_mode'):
            self._previous_control_mode = control_mode
            self._transition_progress = 1.0
            self._transition_start_time = None
        
        if control_mode != self._previous_control_mode:
            self._transition_start_time = rospy.Time.now()
            self._transition_progress = 0.0
        
        if self._transition_start_time is not None:
            transition_duration = 3.0
            elapsed = (rospy.Time.now() - self._transition_start_time).to_sec()
            self._transition_progress = min(1.0, elapsed / transition_duration)
            if self._transition_progress >= 1.0:
                self._transition_start_time = None
                self._previous_control_mode = control_mode
        
        # Get parameters with interpolation during transitions
        target_params = self._get_mode_parameters(control_mode, actual_insertion_depth)
        
        if self._transition_progress < 1.0:
            previous_params = self._get_mode_parameters(self._previous_control_mode, actual_insertion_depth)
            alpha = self._ease_in_out_cubic(self._transition_progress)
            pid_kp = self._interpolate(previous_params['pid_kp'], target_params['pid_kp'], alpha)
            pid_ki = self._interpolate(previous_params['pid_ki'], target_params['pid_ki'], alpha)
            pid_kd = self._interpolate(previous_params['pid_kd'], target_params['pid_kd'], alpha)
            z_pid_kp = self._interpolate(previous_params['z_pid_kp'], target_params['z_pid_kp'], alpha)
            z_pid_ki = self._interpolate(previous_params['z_pid_ki'], target_params['z_pid_ki'], alpha)
            z_pid_kd = self._interpolate(previous_params['z_pid_kd'], target_params['z_pid_kd'], alpha)
            max_correction = self._interpolate(previous_params['max_correction'], target_params['max_correction'], alpha)
        else:
            pid_kp = target_params['pid_kp']
            pid_ki = target_params['pid_ki'] 
            pid_kd = target_params['pid_kd']
            z_pid_kp = target_params['z_pid_kp']
            z_pid_ki = target_params['z_pid_ki']
            z_pid_kd = target_params['z_pid_kd']
            max_correction = target_params['max_correction']
        
        # Override for position hold mode
        if hasattr(self, '_force_position_hold') and self._force_position_hold:
            control_mode = "POSITION_HOLD"
            pid_kp, pid_ki, pid_kd = 0.5, 0.005, 0.2
            z_pid_kp, z_pid_ki, z_pid_kd = 0.8, 0.003, 0.25
            max_correction = 8.0
        
        # Precision mode override for small movements
        if target_pos is not None and start_pos is not None:
            total_movement = math.sqrt(sum((target_pos[i] - start_pos[i])**2 for i in range(3)))
            if total_movement < 0.01:  # <10mm ultra precision
                pid_kp, pid_ki, pid_kd = 0.6, 0.015, 1.0
                z_pid_kp, z_pid_ki, z_pid_kd = 0.8, 0.01, 0.8
                max_correction = 12.0
            elif total_movement < 0.03:  # <30mm high precision
                precision_correction_mode = rospy.get_param('/force_precision_correction_mode', False)
                if precision_correction_mode:
                    pid_kp, pid_ki, pid_kd = 0.7, 0.02, 1.0
                    z_pid_kp, z_pid_ki, z_pid_kd = 1.0, 0.015, 0.8
                else:
                    pid_kp, pid_ki, pid_kd = 0.9, 0.03, 0.8
                    z_pid_kp, z_pid_ki, z_pid_kd = 1.2, 0.02, 0.6
                max_correction = 35.0
        
        # Store base gains for dynamic scaling
        base_pid_kp, base_pid_kd = pid_kp, pid_kd
        base_z_pid_kp, base_z_pid_kd = z_pid_kp, z_pid_kd
        base_max_correction = max_correction
        
        # PID state
        integral_error = [0.0, 0.0, 0.0]
        last_error = [0.0, 0.0, 0.0]
        
        # Control loop
        control_frequency = 30 if (hasattr(self, 'rotation_active') and self.rotation_active) else 20
        rate = rospy.Rate(control_frequency)
        dt = 1.0 / control_frequency
        start_time = time.time()
        trajectory_completed = False
        
        while not rospy.is_shutdown() and self.trajectory_active and (time.time() - start_time) < duration + 10.0:
            elapsed_time = time.time() - start_time
            
            # Force rotation mode if active
            if hasattr(self, 'rotation_active') and self.rotation_active:
                control_mode = "PD_ROTATION"
            
            pt = traj.evaluate()
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
            
            # Real-time PID feedback control
            current_pos = self.sm.get_current_position()
            if current_pos is not None:
                error = [pt[i] - current_pos[i] for i in range(3)]
                
                # Integral error accumulation (disabled for PD modes)
                for i in range(3):
                    if control_mode in ["PD", "PD_ROTATION", "PD_INSERTION", "PD_DISENGAGEMENT"]:
                        integral_error[i] = 0.0
                    else:
                        # Depth-adaptive integral limits
                        if actual_insertion_depth > 0.15:
                            limit_map = {"PI_REDUCED": 0.020, "PID_CONSERVATIVE": 0.035, 
                                       "PID_PRECISION": 0.004, "PID_HIGH_PRECISION": 0.010}
                            limit = limit_map.get(control_mode, 0.040)
                        else:
                            limit_map = {"PI_REDUCED": 0.015, "PID_CONSERVATIVE": 0.025, 
                                       "PID_PRECISION": 0.003, "PID_HIGH_PRECISION": 0.005}
                            limit = limit_map.get(control_mode, 0.030)
                        
                        # Contact detection - reduce limit on sudden error jump
                        position_error_magnitude = math.sqrt(sum(e**2 for e in error))
                        if not hasattr(self, '_last_position_error'):
                            self._last_position_error = position_error_magnitude
                        error_jump = position_error_magnitude - self._last_position_error
                        if error_jump > 0.015:
                            limit = min(limit, 0.005)
                        self._last_position_error = position_error_magnitude
                        
                        integral_error[i] += error[i] * dt
                        integral_error[i] = max(-limit, min(limit, integral_error[i]))
                
                # Global integral saturation protection
                integral_magnitude = math.sqrt(sum(ie**2 for ie in integral_error))
                global_limit = 0.015
                if integral_magnitude > global_limit:
                    scale_factor = global_limit / integral_magnitude
                    integral_error = [ie * scale_factor for ie in integral_error]
                
                # Derivative error
                derivative_error = [(error[i] - last_error[i]) / dt for i in range(3)]
                
                # Adaptive gain adjustment based on error magnitude
                position_error_magnitude = math.sqrt(sum(e**2 for e in error))
                
                if control_mode in ["PD", "PI_REDUCED", "PD_INSERTION"]:
                    if position_error_magnitude > 0.015:
                        gain_scale_factor, correction_scale_factor = 0.7, 0.8
                    elif position_error_magnitude > 0.010:
                        gain_scale_factor, correction_scale_factor = 0.85, 0.9
                    else:
                        gain_scale_factor, correction_scale_factor = 1.0, 1.0
                elif control_mode in ["PID_PRECISION", "PID_HIGH_PRECISION"]:
                    if position_error_magnitude > 0.008:
                        gain_scale_factor, correction_scale_factor = 0.5, 0.6
                    elif position_error_magnitude > 0.005:
                        gain_scale_factor, correction_scale_factor = 0.7, 0.8
                    else:
                        gain_scale_factor, correction_scale_factor = 1.0, 1.0
                else:
                    gain_scale_factor, correction_scale_factor = 1.0, 1.0
                
                adaptive_pid_kp = base_pid_kp * gain_scale_factor
                adaptive_pid_kd = base_pid_kd * gain_scale_factor
                adaptive_z_pid_kp = base_z_pid_kp * gain_scale_factor
                adaptive_z_pid_kd = base_z_pid_kd * gain_scale_factor
                adaptive_max_correction = base_max_correction * correction_scale_factor
                
                # PID correction calculation
                pid_correction = []
                for i in range(3):
                    if i == 2:  # Z-axis
                        correction = (adaptive_z_pid_kp * error[i] + 
                                    z_pid_ki * integral_error[i] + 
                                    adaptive_z_pid_kd * derivative_error[i])
                    else:  # XY-axis
                        correction = (adaptive_pid_kp * error[i] + 
                                    pid_ki * integral_error[i] + 
                                    adaptive_pid_kd * derivative_error[i])
                    max_corr_m = adaptive_max_correction / 1000.0
                    correction = max(-max_corr_m, min(max_corr_m, correction))
                    pid_correction.append(correction)
                    adaptive_gain_logging_interval = 60  # Log every 3s (20Hz/60)
                else:
                    adaptive_gain_logging_interval -= 1
                
                # Apply PID correction to trajectory point
                corrected_pt = [
                    pt[0] + pid_correction[0],
                    pt[1] + pid_correction[1], 
                    pt[2] + pid_correction[2]
                ]
                
                # Update last error for next iteration
                last_error = error.copy()
                
                # Divergence detection
                
                # Track error history for divergence detection
                if not hasattr(self, 'error_history'):
                    self.error_history = []
                    self.control_start_time = elapsed_time
                
                current_error_mag = math.sqrt(sum(e**2 for e in error))
                self.error_history.append((elapsed_time, current_error_mag))
                self.error_history = [(t, e) for t, e in self.error_history if elapsed_time - t <= 3.0]
                
                # Adaptive divergence thresholds
                total_movement = math.sqrt(sum((target_pos[i] - start_pos[i])**2 for i in range(3)))
                if total_movement > 1.0:
                    explosion_threshold, growth_ratio, growth_abs = 0.150, 2.0, 0.080
                elif total_movement > 0.3:
                    explosion_threshold, growth_ratio, growth_abs = 0.080, 1.8, 0.050
                else:
                    explosion_threshold, growth_ratio, growth_abs = 0.050, 2.5, 0.030
                
                # Check for divergence
                divergence_detected = False
                control_duration = elapsed_time - self.control_start_time
                if enable_divergence_detection and control_duration > 2.0 and len(self.error_history) >= 40:
                    if current_error_mag > explosion_threshold:
                        divergence_detected = True
                    
                    recent_errors = [e for t, e in self.error_history[-20:]]
                    older_errors = [e for t, e in self.error_history[-40:-20]]
                    if len(recent_errors) >= 10 and len(older_errors) >= 10:
                        recent_avg = sum(recent_errors) / len(recent_errors)
                        older_avg = sum(older_errors) / len(older_errors)
                        if recent_avg > older_avg * growth_ratio and recent_avg > growth_abs:
                            divergence_detected = True
                
                if divergence_detected:
                    current_pos = self.sm.get_current_position()
                    if current_pos is not None:
                        self._force_position_hold = True
                        self.send_trajectory_point(current_pos, current_target_yaw)
                        rospy.sleep(0.1)
                    self.trajectory_active = False
                    return False
                
                # Send corrected trajectory point
                self.send_trajectory_point(corrected_pt, current_target_yaw)
            else:
                self.send_trajectory_point(pt, current_target_yaw)
            
            # Check for final convergence
            if trajectory_completed:
                convergence_success = self._wait_for_position_and_yaw(
                    target_pos, target_yaw, pos_threshold, yaw_threshold, point_timeout=2.0)
                
                if convergence_success:
                    return True
                else:
                    # Check actual final error with depth-adaptive thresholds
                    final_pos = self.sm.get_current_position()
                    if final_pos is not None:
                        final_error = math.sqrt(sum((target_pos[i] - final_pos[i])**2 for i in range(3)))
                        final_yaw_error = abs(self._normalize_angle_diff(target_yaw - self.sm.current_yaw))
                        
                        # Depth-adaptive threshold multipliers
                        valve_surface_z = self._get_valve_surface_height()
                        insertion_depth = max(0, valve_surface_z - final_pos[2])
                        
                        if insertion_depth > 0.30:
                            threshold_multiplier, yaw_relaxation = 5.0, 1.2
                        elif insertion_depth > 0.20:
                            threshold_multiplier, yaw_relaxation = 4.5, 1.3
                        else:
                            threshold_multiplier, yaw_relaxation = 4.0, 1.5
                        
                        adaptive_pos_threshold = pos_threshold * threshold_multiplier
                        relaxed_yaw_threshold = yaw_threshold * yaw_relaxation
                        
                        if final_error < adaptive_pos_threshold and final_yaw_error < relaxed_yaw_threshold:
                            return True
                        else:
                            return False
                    else:
                        return False
            
            rate.sleep()
        
        return False
    
    def execute_precision_positioning(self, start_pos, target_pos, target_yaw, 
                                    pos_threshold=0.015, yaw_threshold=0.02):
        """Execute precision positioning for fine adjustment using PID control."""
        self.start_new_controller('trajectory')
        
        distance = math.sqrt(sum((target_pos[i] - start_pos[i])**2 for i in range(3)))
        duration = max(4.0, distance / 0.015)
        
        # Depth-adaptive PID parameters
        valve_surface_z = self._get_valve_surface_height()
        insertion_depth = max(0, valve_surface_z - start_pos[2])
        
        if insertion_depth > 0.20:
            pid_kp, pid_ki, pid_kd = 0.6, 0.01, 0.15
        else:
            pid_kp, pid_ki, pid_kd = 1.5, 0.05, 0.3
        
        z_pid_kp, z_pid_ki, z_pid_kd = 1.8, 0.04, 0.25
        base_pid_kp, base_pid_kd = pid_kp, pid_kd
        base_z_pid_kp, base_z_pid_kd = z_pid_kp, z_pid_kd
        max_correction = 15.0
        base_max_correction = max_correction
        
        integral_error = [0.0, 0.0, 0.0]
        last_error = [0.0, 0.0, 0.0]
        
        control_frequency = 30 if (hasattr(self, 'rotation_active') and self.rotation_active) else 20
        rate = rospy.Rate(control_frequency)
        dt = 1.0 / control_frequency
        start_time = time.time()
        
        start_yaw = self.sm.current_yaw
        yaw_diff = self._normalize_angle_diff(target_yaw - start_yaw)
        
        converged_count = 0
        required_convergence = 30
        
        while not rospy.is_shutdown() and self.trajectory_active and (time.time() - start_time) < duration + 8.0:
            elapsed_time = time.time() - start_time
            current_pos = self.sm.get_current_position()
            
            if current_pos is None:
                rate.sleep()
                continue
            
            # Smooth yaw interpolation
            if elapsed_time <= duration:
                yaw_progress = min(elapsed_time / duration, 1.0)
                smooth_progress = 3*yaw_progress**2 - 2*yaw_progress**3
                current_target_yaw = start_yaw + smooth_progress * yaw_diff
            else:
                current_target_yaw = target_yaw
            
            error = [target_pos[i] - current_pos[i] for i in range(3)]
            
            # Integral error with limits
            for i in range(3):
                integral_error[i] += error[i] * dt
                limit = 0.010
                position_error_magnitude = math.sqrt(sum(e**2 for e in error))
                if hasattr(self, '_last_precision_error'):
                    error_jump = position_error_magnitude - self._last_precision_error
                    if error_jump > 0.015:
                        limit = min(limit, 0.005)
                self._last_precision_error = position_error_magnitude
                integral_error[i] = max(-limit, min(limit, integral_error[i]))
            
            derivative_error = [(error[i] - last_error[i]) / dt for i in range(3)]
            
            # Adaptive gain scaling
            position_error_magnitude = math.sqrt(sum(e**2 for e in error))
            if position_error_magnitude > 0.015:
                gain_scale_factor, correction_scale_factor = 0.6, 0.7
            elif position_error_magnitude > 0.008:
                gain_scale_factor, correction_scale_factor = 0.8, 0.85
            else:
                gain_scale_factor, correction_scale_factor = 1.0, 1.0
            
            adaptive_pid_kp = base_pid_kp * gain_scale_factor
            adaptive_pid_kd = base_pid_kd * gain_scale_factor
            adaptive_z_pid_kp = base_z_pid_kp * gain_scale_factor
            adaptive_z_pid_kd = base_z_pid_kd * gain_scale_factor
            adaptive_max_correction = base_max_correction * correction_scale_factor
            
            # PID output
            pid_correction = []
            for i in range(3):
                if i == 2:
                    correction = adaptive_z_pid_kp * error[i] + z_pid_ki * integral_error[i] + adaptive_z_pid_kd * derivative_error[i]
                    max_corr_m = adaptive_max_correction / 1000.0 * 2.0
                else:
                    correction = adaptive_pid_kp * error[i] + pid_ki * integral_error[i] + adaptive_pid_kd * derivative_error[i]
                    max_corr_m = adaptive_max_correction / 1000.0
                correction = max(-max_corr_m, min(max_corr_m, correction))
                pid_correction.append(correction)
            
            corrected_target = [target_pos[i] + pid_correction[i] for i in range(3)]
            last_error = error[:]
            
            self.send_trajectory_point(corrected_target, current_target_yaw)
            
            # Convergence check
            pos_error = math.sqrt(sum(e**2 for e in error))
            yaw_error = abs(self._normalize_angle_diff(current_target_yaw - self.sm.current_yaw))
            
            if pos_error <= pos_threshold and yaw_error <= yaw_threshold:
                converged_count += 1
                if converged_count >= required_convergence:
                    return True
            else:
                converged_count = 0
            
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
            
            rospy.loginfo(f"Precision positioning final result:")
            rospy.loginfo(f"  Position error: {final_error*1000:.1f}mm (target: {pos_threshold*1000:.0f}mm)")
            rospy.loginfo(f"  Yaw error: {math.degrees(final_yaw_error):.2f}° (target: {math.degrees(yaw_threshold):.1f}°)")
            
            return final_error <= pos_threshold and final_yaw_error <= yaw_threshold
        
        return False
    
    # ===== Constant Distance Feedback Control =====
    
    def execute_constant_distance_rotation(self, trajectory, valve_center, 
                                         feedback_frequency=60, alignment_check_interval=0.1):
        """Execute constant distance rotation with feedback control."""
        self.start_new_controller('rotation')
        rospy.set_param('/force_rotation_control_mode', True)
        rospy.loginfo("Starting constant distance rotation")
        
        if hasattr(trajectory, 'start_trajectory'):
            trajectory.start_trajectory()
        
        # Initialize statistics and state
        self.control_stats = {
            'total_corrections': 0, 'distance_violations': 0,
            'max_distance_error': 0.0, 'avg_distance_error': 0.0,
            'execution_time': 0.0, 'total_points': 0
        }
        
        large_error_start_time = None
        large_error_reset_threshold = 5.0
        stable_contact_detected = False
        contact_position_history = []
        trajectory_replanned = False
        valve_contact_detected = False
        
        rate = rospy.Rate(feedback_frequency)
        start_time = time.time()
        last_alignment_check = start_time
        
        self.initial_valve_yaw = getattr(self.sm, 'valve_yaw', None) or 0.0
        distance_error_sum = 0.0
        point_count = 0
        
        while not rospy.is_shutdown() and self.rotation_active:
            result = trajectory.get_next_uav_position_and_yaw()
            if result is None:
                rospy.loginfo("Trajectory completed")
                break
            
            target_pos, target_yaw = result
            current_time = time.time()
            current_pos = self.sm.get_current_position()
            current_yaw = self.sm.current_yaw
            
            if current_pos is None:
                rate.sleep()
                continue
            
            # Apply feedback correction
            if current_time - last_alignment_check >= alignment_check_interval:
                corrected_pos, corrected_yaw = self.apply_constant_distance_feedback(
                    target_pos, target_yaw, current_pos, current_yaw, valve_center, trajectory)
                last_alignment_check = current_time
            else:
                corrected_pos, corrected_yaw = target_pos, target_yaw
            
            # Valve contact detection (skip in disengagement mode)
            is_disengagement = (hasattr(trajectory, 'adaptive_controller') and 
                              getattr(trajectory.adaptive_controller, 'control_mode', '') == 'disengagement')
            
            if not is_disengagement and hasattr(self.sm, 'valve_yaw') and self.sm.valve_yaw is not None:
                valve_yaw_change = abs(self.sm.valve_yaw - self.initial_valve_yaw)
                self.update_valve_rotation_progress(valve_yaw_change)
                
                if not valve_contact_detected and valve_yaw_change > math.radians(2.0):
                    valve_contact_detected = True
                    rospy.loginfo(f"Valve contact detected (yaw change: {math.degrees(valve_yaw_change):.1f}°)")
            
            # Check stable contact and replan if needed
            if (not is_disengagement and valve_contact_detected and 
                not stable_contact_detected and not trajectory_replanned):
                if self._check_stable_contact(current_pos, contact_position_history, 0.003, 2.0):
                    stable_contact_detected = True
                    rospy.loginfo(f"Stable contact achieved after {current_time - start_time:.1f}s")
                    
                    progress = trajectory.get_progress() if hasattr(trajectory, 'get_progress') else 0.5
                    remaining_rotation = math.radians(90) * (1.0 - progress)
                    remaining_duration = max(5.0, (current_time - start_time) * 0.5)
                    
                    if abs(remaining_rotation) > math.radians(10):
                        new_traj = self._replan_trajectory_from_contact(
                            current_pos[:3], current_yaw, valve_center, remaining_rotation, remaining_duration)
                        if new_traj:
                            trajectory, trajectory_replanned = new_traj, True
                            start_time = current_time
                            rospy.loginfo("Switched to replanned trajectory")
            
            # Update tracking performance
            position_error = math.sqrt(sum((c - t)**2 for c, t in zip(current_pos[:3], target_pos[:3])))
            if hasattr(trajectory, 'update_tracking_performance'):
                trajectory.update_tracking_performance(position_error)
            
            self.send_trajectory_point(corrected_pos, corrected_yaw)
            
            # Calculate distance error
            uav_target_dist = getattr(trajectory, 'uav_target_distance', 
                                     trajectory.end_effector_distance + self.end_effector_offset_x)
            distance_error = self.calculate_distance_error(current_pos, current_yaw, valve_center, uav_target_dist)
            distance_error_sum += distance_error
            point_count += 1
            
            # Periodic distance recalibration
            if not hasattr(self, '_last_distance_update_time'):
                self._last_distance_update_time = rospy.Time.now().to_sec()
                self._distance_update_interval = 2.0
                self._distance_drift_threshold = 0.05
            
            if rospy.Time.now().to_sec() - self._last_distance_update_time >= self._distance_update_interval:
                current_ee_pos = self.calculate_end_effector_position(current_pos, current_yaw)
                actual_dist = math.sqrt((current_ee_pos[0] - valve_center[0])**2 + 
                                       (current_ee_pos[1] - valve_center[1])**2)
                expected_dist = trajectory.end_effector_rotation_radius
                drift = abs(actual_dist - expected_dist)
                
                soft_thresh = getattr(self, '_rotation_distance_drift_threshold', 25.0) / 1000.0
                critical_thresh = getattr(self, '_rotation_distance_critical_threshold', 40.0) / 1000.0
                
                # Progressive correction based on drift level
                if drift >= soft_thresh and hasattr(trajectory, 'adaptive_controller'):
                    if hasattr(trajectory.adaptive_controller, 'speed_factor'):
                        trajectory.adaptive_controller.speed_factor = max(0.3, trajectory.adaptive_controller.speed_factor * 0.9)
                
                if drift >= critical_thresh:
                    if hasattr(trajectory.adaptive_controller, 'speed_factor'):
                        trajectory.adaptive_controller.speed_factor = 0.2
                    if drift >= critical_thresh * 1.5 and hasattr(trajectory, 'pause') and not trajectory.is_trajectory_paused():
                        trajectory.pause()
                        rospy.sleep(0.3)
                
                # Adaptive radius correction
                if drift >= self._distance_drift_threshold and not getattr(self, 'disable_distance_drift_correction', False):
                    new_radius = expected_dist + 0.3 * (actual_dist - expected_dist)
                    trajectory.end_effector_rotation_radius = new_radius
                    trajectory.end_effector_distance = new_radius
                    if hasattr(trajectory, 'uav_target_distance'):
                        trajectory.uav_target_distance = new_radius + self.end_effector_offset_x
                        trajectory.uav_rotation_radius = trajectory.uav_target_distance
                    stable_contact_detected = False
                    contact_position_history = []
                
                self._last_distance_update_time = rospy.Time.now().to_sec()
            
            # Large error recovery mechanism
            if distance_error >= 0.15:
                if large_error_start_time is None:
                    large_error_start_time = current_time
                    rospy.logwarn(f"Large error detected: {distance_error*1000:.0f}mm")
                else:
                    duration = current_time - large_error_start_time
                    if 2.0 <= duration < 3.0 and hasattr(trajectory, 'pause') and not getattr(trajectory, '_emergency_paused', False):
                        trajectory.pause()
                        trajectory._emergency_paused = True
                    elif 3.0 <= duration < large_error_reset_threshold and getattr(trajectory, '_emergency_paused', False):
                        if hasattr(trajectory, 'resume'):
                            trajectory.resume()
                            trajectory._emergency_paused = False
                            if hasattr(trajectory, 'adaptive_controller'):
                                trajectory.adaptive_controller.current_speed_factor = 0.3
                    elif duration >= large_error_reset_threshold and hasattr(trajectory, 'reset_to_current_position'):
                        try:
                            trajectory.reset_to_current_position(current_pos, current_yaw)
                            large_error_start_time = None
                            stable_contact_detected = False
                            contact_position_history = []
                            trajectory_replanned = False
                            valve_contact_detected = False
                        except Exception as e:
                            rospy.logerr(f"Trajectory reset failed: {e}")
            else:
                large_error_start_time = None
            
            # Update statistics
            self.control_stats['max_distance_error'] = max(self.control_stats['max_distance_error'], distance_error)
            self.control_stats['total_points'] += 1
            if distance_error > self.distance_tolerance:
                self.control_stats['distance_violations'] += 1
            
            rate.sleep()
        
        # Final statistics
        self.control_stats['execution_time'] = time.time() - start_time
        self.control_stats['avg_distance_error'] = distance_error_sum / max(point_count, 1)
        if hasattr(trajectory, 'get_current_speed_factor'):
            self.control_stats['final_speed_factor'] = trajectory.get_current_speed_factor()
        
        self.log_control_statistics()
        rospy.set_param('/force_rotation_control_mode', False)
        self.rotation_active = False
        
        return self.control_stats
    
    def _check_stable_contact(self, current_pos, contact_position_history, stability_threshold, stability_duration):
        """
        Check if UAV has achieved stable contact with valve
        """
        import time
        current_time = time.time()
        
        # Add current position to history
        contact_position_history.append((current_time, current_pos[:3]))
        
        # Keep only recent history
        cutoff_time = current_time - stability_duration - 1.0
        contact_position_history[:] = [(t, pos) for t, pos in contact_position_history if t >= cutoff_time]
        
        # Need sufficient history
        if len(contact_position_history) < 10:
            return False
        
        # Check duration
        oldest_time = contact_position_history[0][0]
        if current_time - oldest_time < stability_duration:
            return False
        
        # Calculate variance
        stable_positions = [pos for t, pos in contact_position_history if current_time - t <= stability_duration]
        
        if len(stable_positions) < 5:
            return False
        
        # Calculate centroid
        centroid = [
            sum(pos[0] for pos in stable_positions) / len(stable_positions),
            sum(pos[1] for pos in stable_positions) / len(stable_positions),
            sum(pos[2] for pos in stable_positions) / len(stable_positions)
        ]
        
        # Check stability
        for pos in stable_positions:
            deviation = math.sqrt(
                (pos[0] - centroid[0])**2 + 
                (pos[1] - centroid[1])**2 + 
                (pos[2] - centroid[2])**2
            )
            if deviation > stability_threshold:
                return False
        
        return True
    
    def _replan_trajectory_from_contact(self, contact_position, contact_yaw, valve_center, rotation_angle, remaining_duration):
        """
        Replan trajectory based on stable contact position
        """
        try:
            # Get optimal end-effector distance
            from insertion_optimizer import InsertionOptimizer
            optimizer = InsertionOptimizer()
            end_effector_distance = optimizer.get_optimal_end_effector_distance()
            
            # Calculate end-effector position at contact
            contact_ee_x = contact_position[0] + end_effector_distance * math.cos(contact_yaw)
            contact_ee_y = contact_position[1] + end_effector_distance * math.sin(contact_yaw)
            
            # Estimate valve center with weighted average
            original_valve_center = valve_center
            contact_based_center = (contact_ee_x, contact_ee_y, valve_center[2])
            weight = 0.7
            estimated_valve_center = (
                weight * contact_based_center[0] + (1-weight) * original_valve_center[0],
                weight * contact_based_center[1] + (1-weight) * original_valve_center[1],
                valve_center[2]
            )
            
            # Calculate rotation radius
            dx = contact_position[0] - estimated_valve_center[0]
            dy = contact_position[1] - estimated_valve_center[1]
            actual_radius = math.sqrt(dx*dx + dy*dy)
            
            # Validate radius
            if actual_radius < 0.05 or actual_radius > 0.2:
                rospy.logwarn(f"Calculated radius {actual_radius:.3f}m outside range, using original valve center")
                estimated_valve_center = original_valve_center
                dx = contact_position[0] - estimated_valve_center[0]
                dy = contact_position[1] - estimated_valve_center[1]
                actual_radius = math.sqrt(dx*dx + dy*dy)
            
            rospy.loginfo(f"=== TRAJECTORY REPLANNING FROM STABLE CONTACT ===")
            rospy.loginfo(f"Contact position: ({contact_position[0]:.3f}, {contact_position[1]:.3f}, {contact_position[2]:.3f})")
            rospy.loginfo(f"Estimated valve center: ({estimated_valve_center[0]:.3f}, {estimated_valve_center[1]:.3f}, {estimated_valve_center[2]:.3f})")
            rospy.loginfo(f"Rotation radius: {actual_radius:.3f}m")
            
            # Create new trajectory
            from trajectory import create_constant_distance_trajectory
            new_trajectory = create_constant_distance_trajectory(
                current_uav_pos=contact_position,
                current_uav_yaw=contact_yaw,
                valve_center=estimated_valve_center,
                rotation_angle=rotation_angle,
                rotation_duration=remaining_duration,
                enable_adaptive_control=True,
                control_mode='rotation'
            )
            
            if new_trajectory is not None:
                rospy.loginfo("Trajectory successfully replanned from stable contact position")
            return new_trajectory
            
        except Exception as e:
            rospy.logerr(f"Failed to replan trajectory from contact: {e}")
            return None
    
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
        valve_x, valve_y, valve_z = valve_center
        current_distance = sqrt((current_pos[0] - valve_x)**2 + (current_pos[1] - valve_y)**2)
        
        # Get target distance
        if hasattr(self, 'locked_reference_distance') and self.locked_reference_distance is not None:
            target_distance = self.locked_reference_distance + self.end_effector_offset_x
        else:
            target_distance = getattr(trajectory, 'uav_target_distance', 
                                     trajectory.end_effector_distance + self.end_effector_offset_x)
        
        distance_error = abs(current_distance - target_distance)
        
        # Disengagement mode: gentle correction
        if getattr(self, 'disengagement_active', False):
            if distance_error > 0.020:  # 20mm threshold
                correction_gain = 0.3
                error_dir_x = (valve_x - current_pos[0]) / current_distance if current_distance > 0 else 0
                error_dir_y = (valve_y - current_pos[1]) / current_distance if current_distance > 0 else 0
                correction_mag = (target_distance - current_distance) * correction_gain
                return (target_pos[0] + error_dir_x * correction_mag,
                       target_pos[1] + error_dir_y * correction_mag, target_pos[2]), target_yaw
            return target_pos, target_yaw
        
        # Rotation mode: smart drift correction
        if getattr(self, 'rotation_active', False):
            self.last_distance_error = distance_error
            if distance_error > 0.020:
                correction_strength = min(0.4, distance_error / 0.050)
                to_valve = [valve_x - current_pos[0], valve_y - current_pos[1]]
                dist_to_valve = sqrt(to_valve[0]**2 + to_valve[1]**2)
                
                if dist_to_valve > 0.001:
                    direction = [to_valve[i] / dist_to_valve for i in range(2)]
                    adj = current_distance - target_distance
                    correction = [direction[i] * adj * correction_strength for i in range(2)]
                    
                    # Limit to 5mm per cycle
                    corr_mag = sqrt(sum(c**2 for c in correction))
                    if corr_mag > 0.005:
                        correction = [c * 0.005 / corr_mag for c in correction]
                    
                    return (target_pos[0] - correction[0], target_pos[1] - correction[1], target_pos[2]), target_yaw
            return target_pos, target_yaw
        
        # Check completion status
        if self.use_completion_detection and self.completion_detector and self.completion_detector.is_completed():
            return target_pos, target_yaw
        
        # Skip correction for large errors (>150mm)
        if distance_error >= 0.15:
            if hasattr(trajectory, 'pause') and not trajectory.is_trajectory_paused():
                trajectory.pause()
            return target_pos, target_yaw
        elif hasattr(trajectory, 'resume') and trajectory.is_trajectory_paused():
            trajectory.resume()
        
        # Rotation mode uses relaxed tolerance
        effective_tolerance = 0.015 if rospy.get_param('/force_rotation_control_mode', False) else self.distance_tolerance
        
        if distance_error <= effective_tolerance:
            return target_pos, target_yaw
        
        # Calculate and apply correction
        correction_pos, correction_yaw = self.calculate_distance_correction(
            current_pos, current_yaw, valve_center, target_distance)
        
        corrected_pos = (target_pos[0] + correction_pos[0], target_pos[1] + correction_pos[1], target_pos[2] + correction_pos[2])
        corrected_yaw = target_yaw + correction_yaw
        
        self.control_stats['total_corrections'] += 1
        
        # Update completion detector
        if self.use_completion_detection and self.completion_detector:
            self.completion_detector.update_distance_error(distance_error)
            self.completion_detector.record_position_correction()
            if self.completion_detector.is_completed():
                return target_pos, target_yaw
        
        return corrected_pos, corrected_yaw
    
    def calculate_distance_correction(self, current_pos, current_yaw, valve_center, target_distance):
        """Calculate distance correction with predictive control and drift compensation."""
        # Initialize drift history
        if not hasattr(self, '_distance_drift_history'):
            self._distance_drift_history = {
                'errors': [], 'corrections': [], 'velocities': [],
                'last_position': None, 'last_time': None
            }
        
        current_time = rospy.Time.now()
        valve_x, valve_y, valve_z = valve_center
        current_distance = sqrt((current_pos[0] - valve_x)**2 + (current_pos[1] - valve_y)**2)
        distance_error = current_distance - target_distance
        
        # Estimate drift velocity for predictive control
        drift_velocity = 0.0
        predicted_error = distance_error
        
        if self._distance_drift_history['last_position'] and self._distance_drift_history['last_time']:
            dt = (current_time - self._distance_drift_history['last_time']).to_sec()
            if dt > 0.001:
                last_dist = sqrt((self._distance_drift_history['last_position'][0] - valve_x)**2 + 
                                (self._distance_drift_history['last_position'][1] - valve_y)**2)
                drift_velocity = (current_distance - last_dist) / dt
                predicted_error = current_distance + drift_velocity * 0.5 - target_distance
                
                self._distance_drift_history['velocities'].append(drift_velocity)
                if len(self._distance_drift_history['velocities']) > 20:
                    self._distance_drift_history['velocities'].pop(0)
        
        self._distance_drift_history['last_position'] = current_pos
        self._distance_drift_history['last_time'] = current_time
        
        # Adaptive tolerance based on rotation mode and speed
        force_rotation = rospy.get_param('/force_rotation_control_mode', False)
        base_tolerance = 0.015 if force_rotation else self.distance_tolerance
        adaptive_tolerance = base_tolerance * 1.5 if abs(drift_velocity) > 0.01 else base_tolerance
        
        effective_error = predicted_error if abs(predicted_error) > abs(distance_error) else distance_error
        
        if abs(effective_error) <= adaptive_tolerance:
            return (0.0, 0.0, 0.0), 0.0
        
        # Calculate direction from valve to UAV
        if current_distance > 0.001:
            unit_x = (current_pos[0] - valve_x) / current_distance
            unit_y = (current_pos[1] - valve_y) / current_distance
        else:
            unit_x, unit_y = 1.0, 0.0
        
        # Progressive correction factor based on error magnitude
        if abs(effective_error) > 0.08:
            corr_factor = 0.15
        elif abs(effective_error) > 0.05:
            corr_factor = 0.25
        elif abs(effective_error) > 0.03:
            corr_factor = 0.4
        else:
            corr_factor = 1.0
        
        # Adaptive gain based on drift velocity
        adaptive_gain = self.distance_control_gain
        if abs(drift_velocity) > 0.015:
            adaptive_gain *= 0.5
        elif abs(drift_velocity) > 0.008:
            adaptive_gain *= 0.75
        
        if abs(effective_error) > 0.05:
            adaptive_gain *= 0.4
        elif abs(effective_error) > 0.02:
            adaptive_gain *= 0.7
        
        # Calculate correction with drift compensation
        base_correction = -effective_error * adaptive_gain * corr_factor
        drift_comp = -drift_velocity * 0.2 if abs(drift_velocity) > 0.005 else 0.0
        effective_correction = base_correction + drift_comp
        
        uav_corr_x = effective_correction * unit_x * self.position_control_gain
        uav_corr_y = effective_correction * unit_y * self.position_control_gain
        
        # Dynamic magnitude limiting
        corr_mag = sqrt(uav_corr_x**2 + uav_corr_y**2)
        max_corr = 0.025 if abs(effective_error) > 0.06 else (0.035 if abs(effective_error) > 0.03 else 0.045)
        if corr_mag > max_corr:
            scale = max_corr / corr_mag
            uav_corr_x *= scale
            uav_corr_y *= scale
        
        # Yaw correction
        desired_yaw = atan2(valve_y - current_pos[1], valve_x - current_pos[0])
        yaw_error = self._normalize_angle_diff(desired_yaw - current_yaw)
        yaw_gain = self.yaw_control_gain * (0.6 if abs(drift_velocity) > 0.01 else 1.0)
        yaw_correction = max(-0.1, min(0.1, yaw_error * yaw_gain))
        
        return (uav_corr_x, uav_corr_y, 0.0), yaw_correction
    
    def calculate_end_effector_position(self, uav_pos, uav_yaw):
        """
        Calculate end-effector position using UNIFIED method
        
        Args:
            uav_pos: UAV position
            uav_yaw: UAV yaw
        
        Returns:
            tuple: End-effector position (x, y, z)
        """
        # Use UNIFIED calculation to ensure consistency
        from valve_rotation_fang_single import calculate_unified_end_effector_position
        return calculate_unified_end_effector_position(uav_pos, uav_yaw, include_z=True)
    
    def analyze_trajectory_segment(self, current_pos, target_pos, current_yaw, target_yaw, valve_center, elapsed_time):
        """
        Analyze trajectory segment for first segment diagnosis
        
        Returns detailed analysis for debugging pitch errors
        """
        # Calculate various trajectory metrics
        pos_error_3d = math.sqrt((current_pos[0] - target_pos[0])**2 + 
                                (current_pos[1] - target_pos[1])**2 + 
                                (current_pos[2] - target_pos[2])**2)
        
        pos_error_xy = math.sqrt((current_pos[0] - target_pos[0])**2 + 
                                (current_pos[1] - target_pos[1])**2)
        
        pos_error_z = abs(current_pos[2] - target_pos[2])
        yaw_error = abs(current_yaw - target_yaw)
        
        # Calculate distances to valve center
        current_to_valve = math.sqrt((current_pos[0] - valve_center[0])**2 + 
                                   (current_pos[1] - valve_center[1])**2)
        target_to_valve = math.sqrt((target_pos[0] - valve_center[0])**2 + 
                                  (target_pos[1] - valve_center[1])**2)
        valve_distance_error = abs(current_to_valve - target_to_valve)
        
        # Calculate end-effector analysis
        current_ee_pos = self.calculate_end_effector_position(current_pos, current_yaw)
        ee_to_valve = math.sqrt((current_ee_pos[0] - valve_center[0])**2 + 
                               (current_ee_pos[1] - valve_center[1])**2)
        
        # Calculate velocity requirements (approximate)
        if hasattr(self, '_prev_analysis_time') and hasattr(self, '_prev_target_pos'):
            dt = elapsed_time - self._prev_analysis_time
            if dt > 0:
                required_vel_x = (target_pos[0] - self._prev_target_pos[0]) / dt
                required_vel_y = (target_pos[1] - self._prev_target_pos[1]) / dt
                required_vel_magnitude = math.sqrt(required_vel_x**2 + required_vel_y**2)
            else:
                required_vel_magnitude = 0.0
        else:
            required_vel_magnitude = 0.0
        
        self._prev_analysis_time = elapsed_time
        self._prev_target_pos = target_pos
        
        return {
            'pos_error_3d': pos_error_3d,
            'pos_error_xy': pos_error_xy, 
            'pos_error_z': pos_error_z,
            'yaw_error': yaw_error,
            'current_to_valve': current_to_valve,
            'target_to_valve': target_to_valve,
            'valve_distance_error': valve_distance_error,
            'ee_to_valve': ee_to_valve,
            'required_vel_magnitude': required_vel_magnitude
        }
    
    def calculate_distance_error(self, uav_pos, uav_yaw, valve_center, target_distance):
        """
        Calculate distance error (CORRECTED: UAV COG based)
        
        Args:
            uav_pos: UAV COG position
            uav_yaw: UAV yaw
            valve_center: Valve center position
            target_distance: Target distance for UAV COG
        
        Returns:
            float: Distance error
        """
        # CORRECTED: Measure UAV COG to valve center distance
        valve_x, valve_y, _ = valve_center
        
        current_distance = math.sqrt((uav_pos[0] - valve_x)**2 + (uav_pos[1] - valve_y)**2)
        return abs(current_distance - target_distance)
    
    def _normalize_angle_diff(self, angle_diff):
        """Normalize angle difference to [-π, π]"""
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff
    
    def log_control_statistics(self):
        """Log control statistics."""
        rospy.loginfo("=== Motion Controller Statistics ===")
        rospy.loginfo(f"Time: {self.control_stats['execution_time']:.2f}s, Points: {self.control_stats['total_points']}")
        rospy.loginfo(f"Corrections: {self.control_stats['total_corrections']}, Violations: {self.control_stats['distance_violations']}")
        rospy.loginfo(f"Distance error - Max: {self.control_stats['max_distance_error']*1000:.1f}mm, Avg: {self.control_stats['avg_distance_error']*1000:.1f}mm")
        
        if 'final_speed_factor' in self.control_stats:
            rospy.loginfo(f"Adaptive speed factor: {self.control_stats['final_speed_factor']:.2f}")
        
        if self.control_stats['total_points'] > 0:
            success_rate = (1.0 - self.control_stats['distance_violations'] / self.control_stats['total_points']) * 100
            rospy.loginfo(f"Success rate: {success_rate:.1f}%")
    
    def execute_vel_accel_trajectory(self, start_pos, target_pos, target_yaw, duration=5.0,
                                    pos_threshold=0.03, yaw_threshold=0.08, axis_lock_mode=None):
        """Execute trajectory using VEL+ACCEL control with optional axis locking."""
        self.start_new_controller('trajectory')
        
        # Apply axis locking
        locked_start_pos = start_pos
        start_yaw = self.sm.current_yaw
        
        if axis_lock_mode == 'z_only':
            locked_target_pos = (start_pos[0], start_pos[1], target_pos[2])
            locked_target_yaw = start_yaw
            pos_gain, vel_gain, yaw_gain = (0.1, 0.1, 0.6), (0.05, 0.05, 0.4), 0.1
            max_vel = (0.05, 0.05, 0.15)
        elif axis_lock_mode == 'xy_yaw':
            locked_target_pos = (target_pos[0], target_pos[1], start_pos[2])
            locked_target_yaw = target_yaw
            pos_gain, vel_gain, yaw_gain = (0.5, 0.5, 0.1), (0.4, 0.4, 0.05), 0.4
            max_vel = (0.15, 0.15, 0.05)
        elif axis_lock_mode == 'rotation':
            locked_target_pos = target_pos
            locked_target_yaw = target_yaw
            pos_gain, vel_gain, yaw_gain = (0.3, 0.3, 0.3), (0.25, 0.25, 0.25), 0.3
            max_vel = (0.12, 0.12, 0.12)
        else:
            locked_target_pos = target_pos
            locked_target_yaw = target_yaw
            pos_gain, vel_gain, yaw_gain = (0.4, 0.4, 0.4), (0.3, 0.3, 0.3), 0.3
            max_vel = (0.12, 0.12, 0.12)
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(locked_start_pos, locked_target_pos)
        yaw_diff = self._normalize_angle_diff(locked_target_yaw - start_yaw)
        
        rospy.loginfo(f"VEL+ACCEL: {locked_start_pos} -> {locked_target_pos}, {duration:.1f}s, mode={axis_lock_mode}")
        
        rate = rospy.Rate(25)
        start_time = time.time()
        
        while not rospy.is_shutdown() and self.trajectory_active and (time.time() - start_time) < duration + 5.0:
            elapsed = time.time() - start_time
            progress = min(elapsed / duration, 1.0)
            
            desired_pos = traj.evaluate() or locked_target_pos
            desired_vel = traj.get_velocity() or (0, 0, 0)
            
            current_pos = self.sm.get_current_position()
            if current_pos is not None:
                pos_error = tuple(desired_pos[i] - current_pos[i] for i in range(3))
                cmd_vel = tuple(
                    max(-max_vel[i], min(max_vel[i], pos_gain[i] * pos_error[i] + vel_gain[i] * desired_vel[i]))
                    for i in range(3)
                )
            else:
                cmd_vel = (0, 0, 0)
                pos_error = (0, 0, 0)
            
            # Yaw interpolation
            if elapsed <= duration:
                yaw_progress = 3 * (elapsed/duration)**2 - 2 * (elapsed/duration)**3
                desired_yaw = start_yaw + yaw_progress * yaw_diff
            else:
                desired_yaw = locked_target_yaw
            
            yaw_error = self._normalize_angle_diff(desired_yaw - self.sm.current_yaw)
            cmd_yaw_vel = yaw_gain * yaw_error
            
            self.send_velocity_command_with_axis_control(cmd_vel, cmd_yaw_vel, desired_pos, desired_yaw, axis_lock_mode)
            
            # Convergence check
            if current_pos is not None and progress >= 0.95:
                if math.sqrt(sum(e**2 for e in pos_error)) < pos_threshold and \
                   abs(self._normalize_angle_diff(self.sm.current_yaw - locked_target_yaw)) < yaw_threshold:
                    rospy.loginfo("VEL+ACCEL trajectory converged")
                    break
            
            rate.sleep()
        
        rospy.loginfo("VEL+ACCEL trajectory completed")
        return True

    # ===== Additional motion methods =====
    
    def execute_adaptive_valve_insertion(self, target_pos, target_yaw, use_simple_approach=True):
        """Execute adaptive valve insertion strategy."""
        rospy.loginfo("=== ADAPTIVE VALVE INSERTION ===")
        
        start_pos = self.sm.get_current_position()
        if start_pos is None:
            rospy.logerr("Cannot get starting position")
            return False
        
        # Phase 1: Approach to valve vicinity
        approach_target = (target_pos[0], target_pos[1], target_pos[2] + 0.10)
        approach_distance = math.sqrt(sum((approach_target[i] - start_pos[i])**2 for i in range(3)))
        
        approach_success = self.execute_smooth_trajectory_with_yaw(
            start_pos, approach_target, target_yaw,
            duration=max(8.0, approach_distance / 0.1),
            pos_threshold=0.03, yaw_threshold=0.05
        )
        
        if not approach_success:
            rospy.logerr("Phase 1 approach failed")
            return False
        
        rospy.loginfo("Phase 1 completed: Approach successful")
        
        # Phase 2: Precision insertion
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
        
        rospy.loginfo("Phase 2 completed: Precision insertion successful")
        
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
                rospy.loginfo("Phase 3: Insertion verification PASSED")
                rospy.loginfo("ADAPTIVE VALVE INSERTION COMPLETED")
                rospy.loginfo("Ready for real-time rotation trajectory planning")
                return True
            else:
                rospy.logwarn(f"Phase 3: Insertion error {insertion_error*1000:.1f}mm exceeds 15mm tolerance")
        
        rospy.loginfo("ADAPTIVE VALVE INSERTION COMPLETED (with warnings)")
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
        
        while not rospy.is_shutdown() and self.is_controller_active() and (time.time() - start_time) < point_timeout:
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
        
        while not rospy.is_shutdown() and self.is_controller_active() and (time.time() - start_time) < point_timeout:
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
        Send velocity command with enhanced axis control for VEL+ACCEL trajectory mode.
        Uses hybrid VEL+POS control for maximum axis locking precision.
        
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
        nav_msg.control_frame = 0  # WORLD_FRAME = 0
        nav_msg.target = 1         # COG = 1
        
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
        
        # CRITICAL: Always lock roll and pitch for stability - USE PROPER PITCH LOCKING
        nav_msg.roll_nav_mode = FlightNav.NO_NAVIGATION  # Keep roll unlocked
        nav_msg.pitch_nav_mode = FlightNav.POS_MODE  # TRUE pitch control like insertion
        nav_msg.target_pitch = 0.0  # Explicitly lock pitch at 0 degrees
        
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

    # ===== Note: Task-specific methods moved to valve_rotation_fang_single.py =====
    # This controller now focuses on generic, reusable motion control methods

    def execute_three_stage_insertion_with_adaptive_replanning(self, start_pos, target_pos, target_yaw, 
                                                               intermediate_distance=1.0, 
                                                               stage_precision_thresholds=None):
        """Execute three-stage insertion with adaptive replanning."""
        rospy.loginfo("=== THREE-STAGE INSERTION ===")
        
        thresholds = stage_precision_thresholds or {
            'stage1': {'pos': 0.015, 'yaw': 0.02},
            'stage2': {'pos': 0.020, 'yaw': 0.02},
            'stage3': {'pos': 0.025, 'yaw': 0.025}
        }
        
        # Stage 1: Yaw alignment
        rospy.loginfo("Stage 1: Yaw alignment")
        stage1_success = self._execute_stage_with_replanning(
            "Stage 1", start_pos, start_pos, target_yaw, 8.0, thresholds['stage1'])
        if not stage1_success:
            return False
        
        stage1_pos = self.sm.get_current_position()
        stage1_yaw = self.sm.get_current_yaw()
        
        # Stage 2: Intermediate position
        rospy.loginfo("Stage 2: Position move")
        progress_ratio = 0.6 if intermediate_distance >= 1.0 else min(0.7, intermediate_distance / 
            max(math.sqrt((target_pos[0]-stage1_pos[0])**2 + (target_pos[1]-stage1_pos[1])**2), 0.1))
        
        intermediate = (
            stage1_pos[0] + (target_pos[0] - stage1_pos[0]) * progress_ratio,
            stage1_pos[1] + (target_pos[1] - stage1_pos[1]) * progress_ratio,
            stage1_pos[2]
        )
        
        stage2_success = self._execute_stage_with_replanning(
            "Stage 2", stage1_pos, intermediate, stage1_yaw, 15.0, thresholds['stage2'])
        if not stage2_success:
            return False
        
        stage2_pos = self.sm.get_current_position()
        stage2_yaw = self.sm.get_current_yaw()
        
        # Stage 3A: XY positioning + yaw adjustment
        rospy.loginfo("Stage 3A: XY+yaw final positioning")
        stage3a_target = (target_pos[0], target_pos[1], stage2_pos[2])
        stage3a_success = self._execute_stage_with_replanning(
            "Stage 3A", stage2_pos, stage3a_target, target_yaw, 15.0, thresholds['stage3'])
        if not stage3a_success:
            return False
        
        stage3a_pos = self.sm.get_current_position()
        stage3a_yaw = self.sm.get_current_yaw()
        
        # Stage 3B: Z descent with XY lock
        rospy.loginfo("Stage 3B: Z descent")
        z_dist = abs(target_pos[2] - stage3a_pos[2])
        num_segments = max(2, int(math.ceil(z_dist / 0.18))) if z_dist > 0.25 else 1
        
        current_pos = stage3a_pos
        for seg in range(num_segments):
            target_z = target_pos[2] if seg == num_segments - 1 else \
                stage3a_pos[2] + (target_pos[2] - stage3a_pos[2]) * (seg + 1) / num_segments
            
            segment_target = (stage3a_pos[0], stage3a_pos[1], target_z)
            if not self._execute_z_descent_segment(f"3B-{seg+1}", current_pos, segment_target, 
                                                   stage3a_yaw, (stage3a_pos[0], stage3a_pos[1])):
                return False
            current_pos = self.sm.get_current_position()
        
        rospy.loginfo("Three-stage insertion completed")
        
        # Final precision check
        final_pos = self.sm.get_current_position()
        if final_pos:
            xy_err = math.sqrt((final_pos[0]-target_pos[0])**2 + (final_pos[1]-target_pos[1])**2)
            z_err = abs(final_pos[2] - target_pos[2])
            rospy.loginfo(f"Final error: XY={xy_err*1000:.1f}mm, Z={z_err*1000:.1f}mm")
            
            if xy_err > 0.035 or z_err > 0.020:
                rospy.logwarn("Precision correction needed")
                self.execute_smooth_trajectory_with_yaw(
                    final_pos, target_pos, self.sm.current_yaw, 10.0, 0.018, 0.03)
        
        return True
    
    def _execute_stage_with_replanning(self, stage_name, start_pos, target_pos, target_yaw, 
                                       duration, thresholds, max_replanning_attempts=2):
        """Execute single stage with adaptive replanning."""
        rospy.loginfo(f"Executing {stage_name}")
        
        for attempt in range(max_replanning_attempts + 1):
            current_pos = self.sm.get_current_position()
            distance = math.sqrt(sum((target_pos[i] - current_pos[i])**2 for i in range(3)))
            
            # Adaptive duration based on stage
            if "Stage 2" in stage_name:
                adaptive_duration = max(8.0, min(25.0, distance / 0.06))
            elif "Stage 3A" in stage_name:
                adaptive_duration = max(12.0, min(35.0, distance / 0.04))
            else:
                adaptive_duration = max(3.0, min(15.0, distance / 0.1))
            
            enable_divergence = "Stage 2" not in stage_name
            
            # Execute basic trajectory
            success = self.execute_smooth_trajectory_with_yaw(
                current_pos, target_pos, target_yaw,
                duration=adaptive_duration,
                pos_threshold=thresholds['pos'],
                yaw_threshold=thresholds['yaw'],
                enable_divergence_detection=enable_divergence
            )
            
            if success:
                rospy.loginfo(f"{stage_name} completed successfully")
                return True
            
            # If failed and have replanning attempts remaining
            if attempt < max_replanning_attempts:
                rospy.logwarn(f"{stage_name} precision not met, attempting replanning...")
                
                current_pos = self.sm.get_current_position()
                current_yaw = self.sm.get_current_yaw()
                
                if not current_pos:
                    rospy.logerr(f"Cannot get current position for {stage_name} replanning")
                    break
                
                # Calculate deviation
                pos_dev = math.sqrt(
                    (target_pos[0] - current_pos[0])**2 + 
                    (target_pos[1] - current_pos[1])**2 + 
                    (target_pos[2] - current_pos[2])**2
                )
                
                # Extend duration for replanning
                adaptive_duration = adaptive_duration * 1.5
                rospy.loginfo(f"Replanning {stage_name} with extended duration: {adaptive_duration:.1f}s")
                
            else:
                rospy.logerr(f"{stage_name} failed after {max_replanning_attempts} replanning attempts")
                return False
        
        return False

    def _execute_z_descent_segment(self, segment_name, start_pos, target_pos, locked_yaw, xy_lock_pos):
        """Execute Z descent segment with XY lock."""
        z_distance = abs(target_pos[2] - start_pos[2])
        
        # Progressive speed based on segment
        if "3B-1" in segment_name:
            z_speed, speed_desc = 0.020, "fast"
        elif "3B-2" in segment_name:
            z_speed, speed_desc = 0.015, "medium"
        elif "3B-3" in segment_name:
            z_speed, speed_desc = 0.010, "slow"
        else:
            z_speed, speed_desc = 0.015, "default"
        
        adaptive_duration = max(5.0, z_distance / z_speed)
        rospy.loginfo(f"{segment_name}: Z={z_distance*1000:.0f}mm, speed={z_speed*1000:.0f}mm/s ({speed_desc}), {adaptive_duration:.1f}s")
        
        locked_start = (xy_lock_pos[0], xy_lock_pos[1], start_pos[2])
        locked_target = (xy_lock_pos[0], xy_lock_pos[1], target_pos[2])
        
        descent_success = self._execute_polynomial_z_trajectory(
            locked_start, locked_target, locked_yaw, adaptive_duration, z_speed)
        
        if not descent_success:
            return False
        
        # XY correction after descent
        return self._verify_and_correct_xy_position(xy_lock_pos, target_pos[2], locked_yaw, segment_name)
    
    def _execute_polynomial_z_trajectory(self, start_pos, target_pos, target_yaw, duration, speed=0.05):
        """Execute optimized polynomial Z trajectory with XY drift correction."""
        self.start_new_controller('trajectory')
        
        z_distance = abs(target_pos[2] - start_pos[2])
        duration = max(duration, z_distance / (speed * 0.8))
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        
        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        total_points = 0
        
        rospy.loginfo(f"Z trajectory: {start_pos[2]:.3f}→{target_pos[2]:.3f}m, {duration:.1f}s")
        
        while not rospy.is_shutdown() and self.trajectory_active:
            current_time = rospy.Time.now()
            elapsed = (current_time - start_time).to_sec()
            
            if elapsed >= duration:
                break
            
            current_traj_point = traj.evaluate()
            if current_traj_point is None:
                break
            
            current_pos = self.sm.get_current_position()
            total_points += 1
            
            if current_pos is not None:
                z_tracking_error = abs(current_pos[2] - current_traj_point[2])
                locked_point = (target_pos[0], target_pos[1], current_traj_point[2])
                
                # Pause if severe Z tracking error
                if z_tracking_error > 0.06:
                    rate.sleep()
                    continue
                
                # XY drift correction
                xy_drift = math.sqrt(
                    (current_pos[0] - target_pos[0])**2 + 
                    (current_pos[1] - target_pos[1])**2
                )
                
                valve_surface_z = self._get_valve_surface_height()
                current_insertion_depth = max(0, valve_surface_z - current_pos[2])
                
                # Depth-adaptive thresholds
                if current_insertion_depth > 0.24:
                    xy_drift_threshold = 0.012
                    xy_max_correction = 0.002
                elif current_insertion_depth > 0.15:
                    xy_drift_threshold = 0.015
                    xy_max_correction = 0.003
                else:
                    xy_drift_threshold = 0.020
                    xy_max_correction = 0.005
                
                # Apply XY correction
                if xy_drift > xy_drift_threshold:
                    xy_correction_x = (target_pos[0] - current_pos[0]) * 0.3
                    xy_correction_y = (target_pos[1] - current_pos[1]) * 0.3
                    correction_mag = math.sqrt(xy_correction_x**2 + xy_correction_y**2)
                    
                    if correction_mag > xy_max_correction:
                        scale = xy_max_correction / correction_mag
                        xy_correction_x *= scale
                        xy_correction_y *= scale
                    
                    locked_point = (
                        current_pos[0] + xy_correction_x,
                        current_pos[1] + xy_correction_y, 
                        current_traj_point[2]
                    )
                
                # Publish FlightNav
                from aerial_robot_msgs.msg import FlightNav
                nav_msg = FlightNav()
                nav_msg.header.stamp = rospy.Time.now()
                nav_msg.header.frame_id = "world"
                nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE
                nav_msg.target_pos_x = locked_point[0]
                nav_msg.target_pos_y = locked_point[1]
                nav_msg.target_pos_z = locked_point[2]
                nav_msg.target_yaw = target_yaw
                nav_msg.target_pitch = 0.0
                self.pub.publish(nav_msg)
            
            rate.sleep()
        
        rospy.loginfo(f"Z trajectory completed: {total_points} points")
        return True
    
    def _analyze_z_trajectory_trend(self, samples, start_z, target_z):
        """Analyze Z trajectory trend for systemic issues (simplified)."""
        if len(samples) < 10:
            return
            
        tracking_errors = [s['z_error'] for s in samples]
        avg_error = sum(tracking_errors) / len(tracking_errors)
        
        if avg_error > 0.03:
            rospy.logwarn(f"Z trajectory: avg tracking error {avg_error*1000:.1f}mm")
        
        # Cleanup temporary variables
        for attr in ['_last_z_pos', '_last_z_time']:
            if hasattr(self, attr):
                delattr(self, attr)
        return True

    def _verify_and_correct_xy_position(self, target_xy, target_z, target_yaw, correction_name):
        """Verify and correct XY position after descent segments."""
        current_pos = self.sm.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position for XY verification")
            return False
        
        xy_error = math.sqrt((current_pos[0] - target_xy[0])**2 + 
                            (current_pos[1] - target_xy[1])**2)
        
        # Depth-adaptive XY threshold
        valve_surface_z = self._get_valve_surface_height()
        insertion_depth = max(0, valve_surface_z - current_pos[2])
        
        if insertion_depth > 0.25:
            xy_threshold = 0.025
        elif insertion_depth > 0.15:
            xy_threshold = 0.018
        else:
            xy_threshold = 0.012
        
        if xy_error > xy_threshold:
            # Safety check for deep insertion
            if insertion_depth > 0.20 and xy_error > 0.050:
                rospy.logerr(f"SAFETY: Rejecting large XY correction at deep position")
                return False
            
            correction_target = (target_xy[0], target_xy[1], current_pos[2])
            yaw_precision = 0.008 if insertion_depth > 0.1 else 0.01
            correction_duration = 12.0 if insertion_depth > 0.1 else 8.0
            
            return self.execute_smooth_trajectory_with_yaw(
                start_pos=current_pos,
                target_pos=correction_target,
                target_yaw=target_yaw,
                duration=correction_duration,
                pos_threshold=0.015,
                yaw_threshold=yaw_precision
            )
        else:
            rospy.loginfo(f"{correction_name} XY position acceptable ({xy_error*1000:.1f}mm)")
            return True
    
    # ===== PITCH ANGLE CONSTRAINT CONTROL METHODS =====
    
    def check_and_constrain_pitch_angle(self, current_pitch, target_yaw_correction):
        """Check pitch angle and apply constraints to prevent excessive pitch."""
        abs_pitch = abs(current_pitch)
        pitch_warning = abs_pitch > self.pitch_warning_threshold
        
        if pitch_warning:
            constrained_yaw_correction = target_yaw_correction * self.yaw_damping_factor
        else:
            constrained_yaw_correction = target_yaw_correction
        
        # Hard limit check
        if abs_pitch > self.max_pitch_angle:
            rospy.logerr(f"PITCH LIMIT EXCEEDED: {math.degrees(current_pitch):.1f}°")
            return 0.0, True
        
        return constrained_yaw_correction, pitch_warning
    
    def get_current_pitch_from_uav_state(self):
        """Get current pitch angle from UAV state."""
        return 0.0  # Placeholder
    
    def apply_enhanced_yaw_control(self, yaw_error, current_pitch=0.0):
        """Apply enhanced yaw control with deadzone, rate limiting, and pitch consideration."""
        import time
        current_time = time.time()
        
        if abs(yaw_error) < self.yaw_deadzone:
            return 0.0
        
        if current_time - self.last_yaw_correction_time < self.min_yaw_correction_interval:
            return 0.0
        
        yaw_correction = yaw_error * self.yaw_control_gain
        yaw_correction, _ = self.check_and_constrain_pitch_angle(current_pitch, yaw_correction)
        
        # Rate limiting
        dt = current_time - self.last_yaw_correction_time if self.last_yaw_correction_time > 0 else 0.1
        max_correction = self.yaw_rate_limit * dt
        yaw_correction = max(-max_correction, min(max_correction, yaw_correction))
        yaw_correction = max(-self.max_yaw_correction, min(self.max_yaw_correction, yaw_correction))
        
        if abs(yaw_correction) > 0.001:
            self.last_yaw_correction_time = current_time
        
        return yaw_correction
    
    # ===== ROTATION COMPLETION DETECTION METHODS =====
    
    def start_rotation_completion_monitoring(self):
        """Start rotation completion monitoring session."""
        if self.use_completion_detection and self.completion_detector:
            self.completion_detector.start_rotation_monitoring()
    
    def update_valve_rotation_progress(self, current_valve_rotation):
        """Update valve rotation progress for completion detection."""
        if self.use_completion_detection and self.completion_detector:
            self.completion_detector.update_valve_rotation(current_valve_rotation)
    
    def check_rotation_completion_timeout(self):
        """Check for rotation completion timeout."""
        if self.use_completion_detection and self.completion_detector:
            return self.completion_detector.check_timeout()
        return False
    
    def is_rotation_completed(self):
        """Check if rotation is considered complete"""
        if self.use_completion_detection and self.completion_detector:
            return self.completion_detector.is_completed()
        return False
    
    def get_completion_reason(self):
        """Get the reason for completion"""
        if self.use_completion_detection and self.completion_detector:
            return self.completion_detector.get_completion_reason()
        return "legacy_mode"
    
    def get_rotation_progress_summary(self):
        """Get current rotation progress summary."""
        if self.use_completion_detection and self.completion_detector:
            return self.completion_detector.get_progress_summary()
        return {
            'elapsed_time': 0,
            'correction_count': self.control_stats.get('total_corrections', 0),
            'valve_rotation_achieved': False,
            'distance_stabilized': False,
            'completion_detected': False,
            'completion_reason': 'legacy_mode'
        }
    
    # ===== CONTROLLER STATE MANAGEMENT METHODS =====
    
    def stop_all_controllers(self):
        """Stop all controllers and lock UAV at current position."""
        current_pos = self.sm.get_current_position()
        current_yaw = getattr(self.sm, 'current_yaw', 0.0)
        
        if current_pos:
            self._locked_position = current_pos
            self._locked_yaw = current_yaw
        
        self.control_active = False
        self.trajectory_active = False
        self.rotation_active = False
        self.pid_active = False
        
        if hasattr(self, 'error_history'):
            self.error_history = []
        
        # Send position lock commands
        from aerial_robot_msgs.msg import FlightNav
        for _ in range(3):
            nav_msg = FlightNav()
            nav_msg.header.stamp = rospy.Time.now()
            nav_msg.header.frame_id = "world"
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
            nav_msg.yaw_nav_mode = FlightNav.POS_MODE
            if current_pos:
                nav_msg.target_pos_x = current_pos[0]
                nav_msg.target_pos_y = current_pos[1]
                nav_msg.target_pos_z = current_pos[2]
                nav_msg.target_yaw = current_yaw
            self.pub.publish(nav_msg)
            rospy.sleep(0.05)
    
    def _reset_all_integral_errors(self):
        """Reset all integral error accumulations."""
        if hasattr(self, 'integral_error'):
            self.integral_error = [0.0, 0.0, 0.0]
        for attr in ['integral_error_x', 'integral_error_y', 'integral_error_z']:
            if hasattr(self, attr):
                setattr(self, attr, 0.0)
        if hasattr(self, '_trajectory_integral_errors'):
            self._trajectory_integral_errors = [0.0, 0.0, 0.0]
        if hasattr(self, '_pid_internal_integrals'):
            self._pid_internal_integrals = [0.0, 0.0, 0.0]
    
    def _reset_trajectory_level_integrals(self):
        """Reset trajectory-level integral accumulations."""
        if hasattr(self, '_last_position_error'):
            self._last_position_error = 0.0
        self._trajectory_integral_state = [0.0, 0.0, 0.0]

    def start_new_controller(self, controller_type):
        """Start new controller with proper initialization."""
        self.control_active = True
        
        if controller_type == 'trajectory':
            self.trajectory_active = True
        elif controller_type == 'rotation':
            self.rotation_active = True
            self._reset_all_integral_errors()
            self._reset_trajectory_level_integrals()
        elif controller_type == 'pid':
            self.pid_active = True
    
    def is_controller_active(self, controller_type=None):
        """Check if controller is active."""
        if controller_type is None:
            return self.control_active
        elif controller_type == 'trajectory':
            return self.trajectory_active
        elif controller_type == 'rotation':
            return self.rotation_active
        elif controller_type == 'pid':
            return self.pid_active
        return False

    def move_to_position_with_yaw(self, target_pos, target_yaw, duration=3.0, pos_threshold=0.02, yaw_threshold=0.05):
        """Move UAV to target position with specific yaw orientation."""
        try:
            current_pos = None
            current_yaw = 0.0
            
            # Get current position from state machine
            if hasattr(self, 'sm') and self.sm is not None:
                if hasattr(self.sm, 'get_current_position'):
                    current_pos = list(self.sm.get_current_position())
                    current_yaw = getattr(self.sm, 'current_yaw', 0.0)
                elif hasattr(self.sm, 'current_pos'):
                    current_pos = list(self.sm.current_pos)
                    current_yaw = getattr(self.sm, 'current_yaw', 0.0)
            
            if current_pos is None:
                current_pos = [0, 0, 0]
                rospy.logwarn("No position source, using default")
            
            return self.execute_smooth_trajectory_with_yaw(
                start_pos=current_pos,
                target_pos=target_pos, 
                target_yaw=target_yaw,
                duration=duration,
                pos_threshold=pos_threshold,
                yaw_threshold=yaw_threshold
            )
        except Exception as e:
            rospy.logerr(f"Error in move_to_position_with_yaw: {e}")
            return False

    def _detect_valve_contact(self):
        """Detect if UAV is in contact with valve spokes."""
        if not hasattr(self, '_valve_contact_state'):
            self._valve_contact_state = {
                'is_contacted': False,
                'contact_start_time': None,
                'last_valve_yaw': None,
                'position_history': [],
                'contact_confidence': 0.0
            }
        
        current_time = rospy.Time.now()
        contact_indicators = 0
        
        # Valve rotation detection
        if hasattr(self, 'sm') and hasattr(self.sm, 'valve_yaw') and self.sm.valve_yaw is not None:
            if self._valve_contact_state['last_valve_yaw'] is not None:
                valve_yaw_change = abs(self.sm.valve_yaw - self._valve_contact_state['last_valve_yaw'])
                if valve_yaw_change > math.radians(1.5):
                    contact_indicators += 1
                    self._valve_contact_state['last_valve_yaw'] = self.sm.valve_yaw
            else:
                self._valve_contact_state['last_valve_yaw'] = self.sm.valve_yaw
        
        # Force feedback detection
        if hasattr(self, 'sm') and hasattr(self.sm, 'external_wrench') and self.sm.external_wrench is not None:
            force_magnitude = math.sqrt(
                self.sm.external_wrench.force.x**2 + 
                self.sm.external_wrench.force.y**2 + 
                self.sm.external_wrench.force.z**2
            )
            if force_magnitude > 5.0:
                contact_indicators += 1
        
        # Update confidence with hysteresis
        if contact_indicators >= 1:
            self._valve_contact_state['contact_confidence'] = min(1.0, self._valve_contact_state['contact_confidence'] + 0.2)
        else:
            self._valve_contact_state['contact_confidence'] = max(0.0, self._valve_contact_state['contact_confidence'] - 0.1)
        
        threshold = 0.3 if not self._valve_contact_state['is_contacted'] else 0.7
        new_contact_state = self._valve_contact_state['contact_confidence'] > threshold
        self._valve_contact_state['is_contacted'] = new_contact_state
        
        return new_contact_state

    # ===== PROGRESSIVE MODE TRANSITION SUPPORT FUNCTIONS =====
    
    def _get_mode_parameters(self, mode, insertion_depth):
        """Get control parameters for a specific control mode."""
        # Default parameter set
        params = {
            'pid_kp': 1.0,
            'pid_ki': 0.05,
            'pid_kd': 0.3,
            'z_pid_kp': 1.2,
            'z_pid_ki': 0.03,
            'z_pid_kd': 0.25,
            'max_correction': 20.0
        }
        
        if mode == "PD":
            params.update({
                'pid_kp': 0.25,
                'pid_ki': 0.0,
                'pid_kd': 0.12,
                'z_pid_kp': 0.3,
                'z_pid_ki': 0.0,
                'z_pid_kd': 0.18,
                'max_correction': 12.0
            })
        elif mode == "PID_CONSERVATIVE":
            params.update({
                'pid_kp': 1.5,
                'pid_ki': 0.05,
                'pid_kd': 0.4,
                'z_pid_kp': 1.8,
                'z_pid_ki': 0.04,
                'z_pid_kd': 0.3,
                'max_correction': 35.0
            })
        elif mode == "PD_ROTATION":
            # Check contact state for adaptive parameters
            valve_contact_detected = self._detect_valve_contact() if hasattr(self, '_detect_valve_contact') else False
            
            if not valve_contact_detected:  # PRE-CONTACT
                params.update({
                    'pid_kp': 0.08,
                    'pid_ki': 0.0,
                    'pid_kd': 0.22,
                    'z_pid_kp': 0.10,
                    'z_pid_ki': 0.0,
                    'z_pid_kd': 0.15,
                    'max_correction': 6.0
                })
            else:  # POST-CONTACT
                params.update({
                    'pid_kp': 0.12,
                    'pid_ki': 0.0,
                    'pid_kd': 0.28,
                    'z_pid_kp': 0.15,
                    'z_pid_ki': 0.0,
                    'z_pid_kd': 0.20,
                    'max_correction': 8.0
                })
        elif mode == "PD_INSERTION":
            params.update({
                'pid_kp': 0.4,
                'pid_ki': 0.0,
                'pid_kd': 0.8,
                'z_pid_kp': 0.6,
                'z_pid_ki': 0.0,
                'z_pid_kd': 0.75,
                'max_correction': 18.0
            })
        elif mode == "PID_HIGH_PRECISION":
            params.update({
                'pid_kp': 1.2,
                'pid_ki': 0.06,
                'pid_kd': 0.5,
                'z_pid_kp': 1.5,
                'z_pid_ki': 0.04,
                'z_pid_kd': 0.4,
                'max_correction': 30.0
            })
        elif mode == "PD_DISENGAGEMENT":
            params.update({
                'pid_kp': 0.2,
                'pid_ki': 0.0,
                'pid_kd': 0.15,
                'z_pid_kp': 0.3,
                'z_pid_ki': 0.0,
                'z_pid_kd': 0.20,
                'max_correction': 5.0
            })
            
        return params
    
    def _interpolate(self, start_value, end_value, alpha):
        """
        Linear interpolation between start and end values
        
        Args:
            start_value: Starting value
            end_value: Target value
            alpha: Interpolation factor [0, 1]
            
        Returns:
            float: Interpolated value
        """
        return start_value + alpha * (end_value - start_value)
    
    def _ease_in_out_cubic(self, t):
        """
        Cubic easing function for smooth transitions
        
        Args:
            t: Input value [0, 1]
            
        Returns:
            float: Eased value [0, 1]
        """
        if t < 0.5:
            return 4 * t * t * t
        else:
            return 1 - pow(-2 * t + 2, 3) / 2
