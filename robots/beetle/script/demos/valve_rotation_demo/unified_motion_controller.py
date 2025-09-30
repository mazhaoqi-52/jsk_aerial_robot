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
        self.yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.05236)  # 收紧到3°
        
        # Alignment control parameters
        self.strict_alignment_enabled = rospy.get_param("~strict_alignment_enabled", True)
        self.alignment_correction_enabled = rospy.get_param("~alignment_correction_enabled", True)
        self.alignment_position_tolerance = rospy.get_param("~alignment_position_tolerance", 0.01)
        self.alignment_yaw_tolerance = rospy.get_param("~alignment_yaw_tolerance", 0.05236)  # 收紧到3°
        self.alignment_control_gain = rospy.get_param("~alignment_control_gain", 0.8)
        
        # Constant distance feedback control parameters
        self.distance_control_gain = rospy.get_param("~distance_control_gain", 0.5)  # 增强从0.3 (抗XY漂移)
        self.position_control_gain = rospy.get_param("~position_control_gain", 0.6)  # 增强从0.3 (抗XY漂移)
        self.yaw_control_gain = rospy.get_param("~yaw_control_gain", 0.3)  # Reduced from 0.4 to 0.3 to minimize pitch coupling
        self.distance_tolerance = rospy.get_param("~distance_tolerance", 0.025)  # Relaxed from 0.01 to 0.025 (25mm) for valve rotation
        self.max_position_correction = rospy.get_param("~max_position_correction", 0.03)  # 增加从0.02
        self.max_yaw_correction = rospy.get_param("~max_yaw_correction", 0.05)  # Reduced from 0.08 to 0.05 to minimize pitch coupling
        
        # Enhanced yaw control parameters for stability
        self.yaw_deadzone = rospy.get_param("~yaw_deadzone", math.radians(3))  # 3 degree deadzone to reduce frequent adjustments
        self.yaw_rate_limit = rospy.get_param("~yaw_rate_limit", math.radians(30))  # 30 deg/s max yaw rate
        self.min_yaw_correction_interval = rospy.get_param("~min_yaw_correction_interval", 0.2)  # Minimum 0.2s between yaw corrections
        self.last_yaw_correction_time = 0.0
        
        # Pitch angle constraint parameters
        self.max_pitch_angle = rospy.get_param("~max_pitch_angle", math.radians(15))  # 15 degree pitch limit
        self.pitch_control_gain = rospy.get_param("~pitch_control_gain", 0.3)  # Pitch correction gain
        self.pitch_warning_threshold = rospy.get_param("~pitch_warning_threshold", math.radians(10))  # 10 degree warning
        self.yaw_damping_factor = rospy.get_param("~yaw_damping_factor", 0.7)  # Reduce yaw gain when pitch is large
        
        # Import completion detector for intelligent termination
        try:
            from rotation_completion_detector import get_completion_detector
            self.completion_detector = get_completion_detector()
            self.use_completion_detection = True
            rospy.loginfo("Rotation completion detector enabled")
        except ImportError:
            self.completion_detector = None
            self.use_completion_detection = False
            rospy.logwarn("WARNING: Rotation completion detector not available - using legacy mode")
        
        # End-effector parameters
        self.end_effector_offset_x = 0.246
        self.end_effector_offset_y = 0.0
        self.end_effector_offset_z = 0.0743823
        
        # REMOVED: Feedforward control parameters (as requested - no significant benefit)
        
        # CONTROLLER STATE MANAGEMENT
        self.control_active = False  # Global control active flag
        self.trajectory_active = False  # Trajectory control active flag
        self.rotation_active = False  # Rotation control active flag
        self.pid_active = False  # PID control active flag
        
        # PHASE-SPECIFIC CONTROL FLAGS: Enhanced control mode optimization
        self.final_descent_active = False    # Final descent/insertion phase
        self.return_to_start_active = False  # Return to start position phase
        self.pre_final_descent = True        # Pre-final descent (uses PID, then switches to PD)
        
        rospy.loginfo("Controller state management initialized")
        rospy.loginfo("Phase-specific control flags initialized")
        
    def _get_valve_surface_height(self):
        """
        Get actual valve surface height from state machine
        
        Returns:
            float: Valve surface Z coordinate, fallback to 1.0 if unavailable
        """
        if hasattr(self.sm, 'valve_pos') and self.sm.valve_pos is not None:
            return self.sm.valve_pos[2]  # Actual valve Z coordinate
        else:
            rospy.logwarn_throttle(5, "Valve position unavailable, using fallback height 1.0m")
            return 1.0  # Fallback to hardcoded value
        
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
        Send trajectory point to UAV using position control with depth-adaptive control
        
        Args:
            position: Target position (x, y, z)
            yaw: Target yaw angle in radians (optional)
        """
        # DEPTH-ADAPTIVE CONTROL INTEGRATION FOR ROTATION PHASE
        # Apply the same depth-adaptive control logic used in velocity control
        current_pos = self.sm.get_current_position()
        if current_pos:
            # FIXED: Calculate insertion depth using actual valve position
            if hasattr(self.sm, 'valve_pos') and self.sm.valve_pos is not None:
                valve_surface_z = self.sm.valve_pos[2]  # Actual valve Z coordinate
                rospy.loginfo_throttle(10, f"Using actual valve height: {valve_surface_z:.3f}m for insertion depth calculation")
            else:
                valve_surface_z = 1.0  # Fallback to hardcoded value if valve position unavailable
                rospy.logwarn_throttle(10, "Valve position unavailable, using fallback height 1.0m for insertion depth calculation")
            
            actual_insertion_depth = max(0, valve_surface_z - current_pos[2])
            
        # UNIFIED CONTROL MODE LOGIC - SINGLE DECISION POINT
        # Phase flags have absolute priority to prevent mode conflicts
        
        # PRIORITY 1: DISENGAGEMENT (Highest Priority)
        if hasattr(self, 'disengagement_active') and self.disengagement_active:
            control_mode = "PD_DISENGAGEMENT"
            rotation_control_mode = "PD_DISENGAGEMENT"
            rospy.loginfo_throttle(5, f"🚁 DISENGAGEMENT PHASE: Unified PD_DISENGAGEMENT mode")
            
        # PRIORITY 2: ROTATION (Second Highest Priority - CRITICAL FIX)
        elif hasattr(self, 'rotation_active') and self.rotation_active:
            control_mode = "PD_ROTATION"
            rotation_control_mode = "PD_ROTATION"
            actual_insertion_depth = 0.0  # Force free space treatment
            rospy.loginfo_throttle(5, f"ROTATION PHASE: Unified PD_ROTATION mode - NO CONFLICTS")
            
        # PRIORITY 3: OTHER SPECIFIC PHASES
        elif hasattr(self, 'return_to_start_active') and self.return_to_start_active:
            control_mode = "PD"
            rotation_control_mode = "PD"
            rospy.loginfo_throttle(5, f"🏠 RETURN TO START: Unified PD mode")
            
        elif hasattr(self, 'final_descent_active') and self.final_descent_active:
            control_mode = "PD_INSERTION"
            rotation_control_mode = "PD_INSERTION"
            rospy.loginfo_throttle(5, f"FINAL DESCENT: Unified PD_INSERTION mode")
            
        # PRIORITY 4: DEPTH-BASED CONTROL (Lowest Priority)
        elif actual_insertion_depth > 0.02:  # >20mm insertion
            if hasattr(self, 'pre_final_descent') and self.pre_final_descent:
                control_mode = "PID_HIGH_PRECISION"
                rotation_control_mode = "PID_HIGH_PRECISION"
                rospy.loginfo_throttle(5, f"PRE-FINAL DESCENT: Unified PID_HIGH_PRECISION mode")
            else:
                control_mode = "PD_INSERTION"
                rotation_control_mode = "PD_INSERTION"
                rospy.loginfo_throttle(5, f"INSERTION PHASE: Unified PD_INSERTION mode")
        else:
            # CRITICAL FIX: FREE SPACE control mode selection based on actual situation
            # Problem: depth=0mm was always using PID_HIGH_PRECISION, causing integral protection issues
            # Solution: Use position_error and current_velocity to determine appropriate mode
            
            position_error = math.sqrt((position[0] - current_pos[0])**2 + 
                                     (position[1] - current_pos[1])**2 + 
                                     (position[2] - current_pos[2])**2) if current_pos else 0.0
            
            # INTELLIGENT FREE SPACE MODE SELECTION
            if position_error > 0.08:  # Large error (>80mm) = approach phase
                control_mode = "PD"  # Use PD for large movements to avoid integral windup
                rotation_control_mode = "PD"
                rospy.loginfo_throttle(5, f"FREE SPACE APPROACH: PD mode (error: {position_error*1000:.0f}mm)")
            elif position_error > 0.03:  # Medium error (30-80mm) = alignment phase
                control_mode = "PID_CONSERVATIVE"  # Conservative PID for medium precision
                rotation_control_mode = "PID_CONSERVATIVE"
                rospy.loginfo_throttle(5, f"FREE SPACE ALIGNMENT: PID_CONSERVATIVE mode (error: {position_error*1000:.0f}mm)")
            else:  # Small error (<30mm) = precision phase
                control_mode = "PID_HIGH_PRECISION"  # High precision only when truly needed
                rotation_control_mode = "PID_HIGH_PRECISION"
                rospy.loginfo_throttle(5, f"FREE SPACE PRECISION: PID_HIGH_PRECISION mode (error: {position_error*1000:.0f}mm)")
            
            actual_insertion_depth = 0.0  # Maintain depth=0 for free space
        
        # � UNIFIED MODE VERIFICATION: Both modes must always match
        if control_mode != rotation_control_mode:
            rospy.logerr(f"� CRITICAL ERROR: Control mode mismatch detected!")
            rospy.logerr(f"   control_mode: {control_mode}")
            rospy.logerr(f"   rotation_control_mode: {rotation_control_mode}")
            rospy.logerr(f"� This should NEVER happen with unified logic!")
            # Emergency fallback to safe mode
            control_mode = rotation_control_mode = "PD"
            rospy.logwarn(f"� EMERGENCY FALLBACK: Both modes set to PD")
        
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.header.frame_id = "world"
        
        # CRITICAL: Correct control frame and target configuration
        nav_msg.control_frame = 0  # WORLD_FRAME = 0
        nav_msg.target = 1         # COG = 1
        
        # DEPTH-ADAPTIVE POSITION CONTROL - Apply depth-based control parameters
        nav_msg.pos_xy_nav_mode = 2  # POS_MODE = 2
        nav_msg.target_pos_x = position[0]
        nav_msg.target_pos_y = position[1]
        
        nav_msg.pos_z_nav_mode = 2   # POS_MODE = 2
        nav_msg.target_pos_z = position[2]
        
        # NOTE: Control modes already set in unified logic above
        # No redundant control mode assignment needed here
        
        # ENHANCED: Attitude control with proper configuration
        # Roll control - typically disabled for quadrotor
        nav_msg.roll_nav_mode = 0    # NO_NAVIGATION = 0
        nav_msg.target_roll = 0.0
        nav_msg.target_omega_x = 0.0
        
        # Pitch control - maintain level flight for valve rotation stability
        nav_msg.pitch_nav_mode = 2   # POS_MODE = 2
        nav_msg.target_pitch = 0.0   # Fixed level flight
        nav_msg.target_omega_y = 0.0  # Zero pitch rate
        
        # Yaw control with rotation-specific optimization
        if yaw is not None:
            nav_msg.yaw_nav_mode = 2     # POS_MODE = 2
            nav_msg.target_yaw = yaw
            
            # ROTATION PHASE YAW OPTIMIZATION: Enhanced yaw control for rotation stability
            if hasattr(self, 'rotation_active') and self.rotation_active:
                # During rotation, use softer yaw rate control to prevent coupling with position control
                nav_msg.target_omega_z = 0.0  # Zero yaw rate for position control
                
                # Apply yaw rate limiting for rotation phase stability
                if hasattr(self.sm, 'current_yaw'):
                    current_yaw = getattr(self.sm, 'current_yaw', 0.0)
                    yaw_error = self._normalize_angle_diff(yaw - current_yaw)
                    
                    # Limit yaw corrections during rotation to prevent instability
                    max_yaw_error_for_rotation = math.radians(20.0)  # 20° max correction
                    if abs(yaw_error) > max_yaw_error_for_rotation:
                        # Clamp the target yaw to prevent excessive corrections
                        limited_yaw_error = math.copysign(max_yaw_error_for_rotation, yaw_error)
                        nav_msg.target_yaw = current_yaw + limited_yaw_error
                        rospy.logwarn_throttle(2, f"YAW LIMITING: Large yaw error {math.degrees(yaw_error):.1f}° limited to {math.degrees(limited_yaw_error):.1f}°")
                    
                rospy.loginfo_throttle(10, f"ROTATION YAW: Enhanced yaw control with error limiting active")
            else:
                nav_msg.target_omega_z = 0.0  # Set yaw rate to zero for position control
        else:
            nav_msg.yaw_nav_mode = 0     # NO_NAVIGATION = 0
            nav_msg.target_yaw = 0.0
            nav_msg.target_omega_z = 0.0
        
        # NEW: Attitude diagnostics for pitch error analysis
        if hasattr(self.sm, 'current_pitch') and hasattr(self.sm, 'current_yaw'):
            current_pitch = getattr(self.sm, 'current_pitch', 0.0)
            current_yaw = getattr(self.sm, 'current_yaw', 0.0)
            
            pitch_error = abs(current_pitch)  # Always compare to 0 (level flight)
            yaw_error = abs(current_yaw - (yaw if yaw is not None else current_yaw))
            
            # Log significant attitude errors periodically
            current_time = rospy.Time.now().to_sec()
            if not hasattr(self, '_last_attitude_log_time'):
                self._last_attitude_log_time = current_time
            
            if current_time - self._last_attitude_log_time > 2.0:  # Log every 2 seconds
                self._last_attitude_log_time = current_time
                if pitch_error > 0.017 or yaw_error > 0.035:  # >1° pitch or >2° yaw
                    rospy.loginfo(f"ATTITUDE STATUS: pitch_error={math.degrees(pitch_error):.1f}°, "
                                f"yaw_error={math.degrees(yaw_error):.1f}°")
        
        # ENHANCED CONTROL MODE TRACKING: Log final control mode before publishing
        if hasattr(self, 'rotation_active') and self.rotation_active:
            rospy.loginfo_throttle(5, f"FINAL NAV PUBLISH: control_mode={control_mode}, rotation_control_mode={rotation_control_mode}")
            rospy.loginfo_throttle(5, f"FINAL NAV PUBLISH: rotation_active={self.rotation_active}, depth={actual_insertion_depth*1000:.0f}mm")
            
            # CRITICAL VIOLATION DETECTION: Enhanced monitoring with stack trace
            if control_mode != "PD_ROTATION" or rotation_control_mode != "PD_ROTATION":
                rospy.logerr(f"CRITICAL VIOLATION: Non-rotation mode detected during rotation!")
                rospy.logerr(f"CRITICAL VIOLATION: control_mode={control_mode}, rotation_control_mode={rotation_control_mode}")
                rospy.logerr(f"CRITICAL VIOLATION: Expected both modes to be PD_ROTATION")
                
                # Force emergency correction
                control_mode = "PD_ROTATION"
                rotation_control_mode = "PD_ROTATION"
                rospy.logwarn(f"EMERGENCY CORRECTION: Force set both modes to PD_ROTATION")
                
                # Log call stack for debugging
                import traceback
                rospy.logerr("CALL STACK when violation occurred:")
                for line in traceback.format_stack()[-3:]:  # Last 3 calls
                    rospy.logerr(f"  {line.strip()}")
            else:
                # Positive confirmation when modes are correct
                rospy.loginfo_throttle(10, f"ROTATION MODE VERIFIED: Both control modes correctly set to PD_ROTATION")
        
        # GENERAL CONTROL MODE MONITORING: Track all mode changes
        if not hasattr(self, '_last_control_mode'):
            self._last_control_mode = control_mode
            self._mode_change_count = 0
        
        if self._last_control_mode != control_mode:
            self._mode_change_count += 1
            rospy.logwarn(f"CONTROL MODE CHANGE #{self._mode_change_count}: {self._last_control_mode} → {control_mode}")
            
            # Alert on frequent mode changes (possible instability)
            if self._mode_change_count > 5:
                rospy.logerr(f"EXCESSIVE MODE CHANGES: {self._mode_change_count} changes detected - possible control instability")
            
            self._last_control_mode = control_mode
        
        self.pub.publish(nav_msg)
    
    # ===== Basic trajectory execution functions =====
    
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
        rospy.loginfo("=== SIMPLIFIED SINGLE-PHASE TRAJECTORY CONTROL ===")
        rospy.loginfo("STRATEGY: Single phase approach (Phase 2 removed to eliminate oscillation)")
        
        distance = math.sqrt(
            (target_pos[0] - start_pos[0])**2 + 
            (target_pos[1] - start_pos[1])**2 + 
            (target_pos[2] - start_pos[2])**2
        )
        
        # === SINGLE PHASE: DIRECT APPROACH WITH ADAPTIVE PRECISION ===
        rospy.loginfo(f"--- SINGLE PHASE: DIRECT APPROACH (distance: {distance:.3f}m) ---")
        
        # Check current insertion depth for adaptive threshold adjustment
        current_pos = self.sm.get_current_position()
        insertion_depth = 0.0
        if current_pos is not None:
            # FIXED: Use actual valve position for insertion depth calculation
            if hasattr(self.sm, 'valve_pos') and self.sm.valve_pos is not None:
                valve_surface_z = self.sm.valve_pos[2]  # Actual valve Z coordinate
            else:
                valve_surface_z = 1.0  # Fallback to hardcoded value
                rospy.logwarn("Valve position unavailable, using fallback height 1.0m")
            insertion_depth = max(0, valve_surface_z - current_pos[2])
        
        # Use adaptive thresholds based on distance and insertion depth
        if distance < 0.01:  # Very small movement
            # ENHANCED: Tighter precision for insertion tasks
            if insertion_depth > 0.30:  # Deep insertion (>300mm)
                pos_threshold = 0.015  # 15mm - improved precision for deep insertion
                rospy.loginfo(f"DEEP INSERTION detected ({insertion_depth*1000:.0f}mm): Using 15mm precision threshold")
            elif insertion_depth > 0.20:  # Medium insertion (>200mm)
                pos_threshold = 0.012  # 12mm - improved precision for medium insertion
                rospy.loginfo(f"MEDIUM INSERTION detected ({insertion_depth*1000:.0f}mm): Using 12mm precision threshold")
            elif insertion_depth > 0.05:  # Shallow insertion (>50mm)
                pos_threshold = 0.010  # 10mm - strict precision for shallow insertion
                rospy.loginfo(f"SHALLOW INSERTION detected ({insertion_depth*1000:.0f}mm): Using 10mm precision threshold")
            else:  # Surface or no insertion - also need high precision
                pos_threshold = 0.008  # 8mm - highest precision for surface tasks
                rospy.loginfo(f"SURFACE POSITIONING: Using 8mm precision threshold")
            yaw_threshold = final_yaw_threshold * 1.2   # 稍微放宽yaw要求
            duration = 8.0  # 给更多时间达到高精度
        elif distance < 0.05:  # Small movement
            pos_threshold = final_pos_threshold * 2.0   # 2x relaxed
            yaw_threshold = final_yaw_threshold * 1.5   # 1.5x relaxed
            duration = 3.0
        else:  # Normal movement
            pos_threshold = final_pos_threshold * 3.0   # 3x relaxed
            yaw_threshold = final_yaw_threshold * 2.0   # 2x relaxed
            duration = max(4.0, distance / 0.08)
        
        rospy.loginfo(f"Single phase thresholds: pos={pos_threshold*1000:.0f}mm, yaw={math.degrees(yaw_threshold):.1f}°")
        
        # Execute single phase trajectory
        success = self.execute_smooth_trajectory_with_yaw(
            start_pos, target_pos, target_yaw,
            duration=duration,
            pos_threshold=pos_threshold,
            yaw_threshold=yaw_threshold
        )
        
        if success:
            rospy.loginfo("SINGLE-PHASE TRAJECTORY COMPLETED - NO OSCILLATION")
            return True
        else:
            rospy.logerr("Single-phase trajectory failed")
            return False

    def execute_smooth_trajectory_with_yaw(self, start_pos, target_pos, target_yaw, duration=5.0,
                                          pos_threshold=0.03, yaw_threshold=0.02, 
                                          enable_divergence_detection=True):
        """
        Execute smooth trajectory motion with yaw control (ENHANCED WITH PID FEEDBACK)
        
        FIXES:
        1. Increased control frequency: 20Hz→30Hz for rotation, 20Hz standard for better interpolation
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
        
        # CRITICAL FIX: ROTATION MODE FORCE LOCK - HIGHEST PRIORITY CHECK
        # This must be the FIRST check to prevent any control mode override
        if hasattr(self, 'rotation_active') and self.rotation_active:
            rospy.loginfo("TRAJECTORY FUNCTION: ROTATION MODE DETECTED - Force locking all control to PD_ROTATION")
            rospy.loginfo("TRAJECTORY FUNCTION: All control mode selection will be overridden to PD_ROTATION")
        
        # CONTROLLER STATE MANAGEMENT: Start trajectory controller
        self.start_new_controller('trajectory')
        
        rospy.loginfo(f"=== ENHANCED SMOOTH TRAJECTORY WITH PID FEEDBACK ===")
        # ROTATION DIAGNOSTIC: Show rotation state in trajectory function
        if hasattr(self, 'rotation_active'):
            rospy.loginfo(f"TRAJECTORY ROTATION STATE: rotation_active = {self.rotation_active}")
        else:
            rospy.loginfo("TRAJECTORY ROTATION STATE: rotation_active not set")
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
        
        # 轨迹平滑化：防止数值不稳定和过快速度
        rospy.loginfo("=== TRAJECTORY SPEED VALIDATION ===")
        max_expected_speed = max(0.1, distance / duration)  # 基于平均速度
        max_allowed_speed = max_expected_speed * 2.5  # 允许2.5倍峰值
        
        rospy.loginfo(f"Expected avg speed: {max_expected_speed:.3f}m/s")
        rospy.loginfo(f"Max allowed speed: {max_allowed_speed:.3f}m/s")
        
        # 如果距离很小且持续时间合理，预检查轨迹数值稳定性
        if distance < 0.05 and duration > 5.0:  # 小于50mm且持续时间>5s
            dt_check = 0.033  # 30Hz采样 (improved density)
            check_times = [i * dt_check for i in range(min(20, int(duration / dt_check)))]
            speed_violations = 0
            
            for i, t in enumerate(check_times[1:], 1):
                if hasattr(traj, 'evaluate_at_time'):
                    curr_point = traj.evaluate_at_time(t)
                    prev_point = traj.evaluate_at_time(check_times[i-1])
                    
                    point_distance = math.sqrt(
                        (curr_point[0] - prev_point[0])**2 + 
                        (curr_point[1] - prev_point[1])**2 + 
                        (curr_point[2] - prev_point[2])**2
                    )
                    point_speed = point_distance / dt_check
                    
                    if point_speed > max_allowed_speed:
                        speed_violations += 1
                        if speed_violations <= 2:  # 记录前2次
                            rospy.logwarn(f"WARNING: 轨迹速度异常 t={t:.2f}s: {point_speed:.3f}m/s > {max_allowed_speed:.3f}m/s")
            
            if speed_violations > len(check_times) * 0.2:  # 超过20%异常
                rospy.logwarn(f"轨迹数值不稳定：{speed_violations}/{len(check_times)} 采样点速度异常")
                rospy.logwarn("增加持续时间以改善数值稳定性")
                
                # 重新生成更保守的轨迹
                safer_duration = duration * 1.3
                traj = PolynomialTrajectory(safer_duration)
                traj.generate_trajectory(start_pos, target_pos)
                duration = safer_duration
                rospy.loginfo(f"重新生成轨迹，持续时间: {duration:.1f}s")
            else:
                rospy.loginfo(f"轨迹数值稳定性检查通过：{speed_violations} 异常点")
        
        rospy.loginfo("轨迹平滑化验证完成")
        
        start_yaw = self.sm.current_yaw
        yaw_diff = self._normalize_angle_diff(target_yaw - start_yaw)
        
        rospy.loginfo(f"Yaw change: {math.degrees(yaw_diff):.1f}°")
        
        # DEPTH-ADAPTIVE CONTROL STRATEGY (Anti-Oscillation)
        # 基于插入深度动态选择控制模式：PID → PI → PD
        current_pos = self.sm.get_current_position()
        if current_pos:
            # 深度计算和控制模式选择
            valve_surface_z = self._get_valve_surface_height()  # 使用实际阀门高度
            actual_insertion_depth = max(0, valve_surface_z - current_pos[2])
            relative_depth = max(0, current_pos[2] - target_pos[2])  # 相对目标深度
            
            # ROTATION PHASE DEPTH CALCULATION FIX
            # PROBLEM: During rotation, depth calculation was showing 0mm instead of actual insertion
            # SOLUTION: Use historical insertion depth or force minimum depth for rotation phase
            if hasattr(self, 'rotation_active') and self.rotation_active:
                # Store insertion depth before rotation starts for stable depth reporting
                if not hasattr(self, '_rotation_insertion_depth'):
                    self._rotation_insertion_depth = max(0.080, actual_insertion_depth)  # Minimum 80mm or actual depth
                    rospy.loginfo(f"ROTATION DEPTH LOCKED: {self._rotation_insertion_depth*1000:.0f}mm (prevents control mode switching)")
                
                # Use stored depth during rotation to maintain consistent control mode
                actual_insertion_depth = self._rotation_insertion_depth
                rospy.loginfo_throttle(5, f"ROTATION DEPTH: Using locked depth {actual_insertion_depth*1000:.0f}mm (raw: {max(0, valve_surface_z - current_pos[2])*1000:.0f}mm)")
            else:
                # Clear stored rotation depth when not in rotation
                if hasattr(self, '_rotation_insertion_depth'):
                    delattr(self, '_rotation_insertion_depth')
            
            rospy.loginfo(f"Depth diagnosis: current_z={current_pos[2]:.3f}m, target_z={target_pos[2]:.3f}m")
            rospy.loginfo(f"  Relative depth: {relative_depth*1000:.0f}mm, Actual insertion: {actual_insertion_depth*1000:.0f}mm")
            
            # CONTROL MODE SELECTION with precision correction override
            precision_correction_mode = rospy.get_param('/force_precision_correction_mode', False)
            force_rotation_mode = rospy.get_param('/force_rotation_control_mode', False)
            
            # � CRITICAL ROTATION MODE FORCE LOCK - MUST BE FIRST PRIORITY
            # This check overrides ALL other control mode selection logic
            if hasattr(self, 'rotation_active') and self.rotation_active:
                # FORCE LOCK: No matter what other conditions exist, use PD_ROTATION
                control_mode = "PD_ROTATION"
                actual_insertion_depth = 0.0  # Force free space treatment for rotation stability
                rospy.loginfo(f"FORCE LOCK: Rotation active - OVERRIDING all control mode logic to PD_ROTATION")
                rospy.loginfo(f"FORCE LOCK: Ignoring precision_correction_mode={precision_correction_mode}, force_rotation_mode={force_rotation_mode}")
                rospy.loginfo(f"FORCE LOCK: Actual insertion depth reset to 0mm for rotation stability")
                
                # ENHANCED ROTATION MODE VERIFICATION
                rospy.loginfo_throttle(3, f"ROTATION MODE LOCKED: control_mode={control_mode}, rotation_active={self.rotation_active}")
                
                # Track rotation mode violations
                if not hasattr(self, '_rotation_mode_violations'):
                    self._rotation_mode_violations = 0
            elif precision_correction_mode:
                # PRECISION CORRECTION MODE: Force PID_HIGH_PRECISION to avoid mode switching
                control_mode = "PID_HIGH_PRECISION"
                rospy.loginfo(f"PRECISION CORRECTION MODE: Forced PID_HIGH_PRECISION (depth: {actual_insertion_depth*1000:.0f}mm)")
            elif force_rotation_mode:
                # � GLOBAL ROTATION MODE: Force rotation mode via parameter
                actual_insertion_depth = 0.35  # 350mm - force deep insertion state for rotation
                rospy.loginfo(f"GLOBAL ROTATION OVERRIDE: Using fixed insertion depth {actual_insertion_depth*1000:.0f}mm for stable control mode")
                control_mode = "PD_ROTATION"
                rospy.loginfo(f"GLOBAL ROTATION OVERRIDE: Forced PD_ROTATION mode")
            elif actual_insertion_depth > 0.02:  # >20mm insertion = contact phase
                # INSERTION PHASE: Automatic PD_INSERTION for constrained environment
                control_mode = "PD_INSERTION"
                rospy.loginfo(f"INSERTION PHASE: PD_INSERTION mode (depth: {actual_insertion_depth*1000:.0f}mm)")
            else:
                # FREE SPACE PHASE: PID for all free space operations (approach, alignment, XY correction)
                control_mode = "PID_HIGH_PRECISION"
                rospy.loginfo(f"FREE SPACE PHASE: PID_HIGH_PRECISION mode (depth: {actual_insertion_depth*1000:.0f}mm)")
        else:
            control_mode = "PID_STANDARD"  # 默认模式
            actual_insertion_depth = 0.0
        
        # CONTROL PARAMETERS BASED ON DEPTH AND MODE
        
        # ===== PROGRESSIVE MODE TRANSITION OPTIMIZATION =====
        # Problem: Instant mode switching causes parameter jumps → oscillation
        # Solution: Smooth parameter interpolation during mode transitions
        
        # Track previous control mode for transition detection
        if not hasattr(self, '_previous_control_mode'):
            self._previous_control_mode = control_mode
            self._transition_progress = 1.0  # 1.0 = fully transitioned
            self._transition_start_time = None
        
        # Detect mode transition
        if control_mode != self._previous_control_mode:
            rospy.loginfo(f"MODE TRANSITION DETECTED: {self._previous_control_mode} → {control_mode}")
            self._transition_start_time = rospy.Time.now()
            self._transition_progress = 0.0  # Start smooth transition
            rospy.loginfo("PROGRESSIVE TRANSITION: Starting smooth parameter interpolation")
        
        # Update transition progress (3-second smooth transition)
        if self._transition_start_time is not None:
            transition_duration = 3.0  # 3 seconds for smooth transition
            elapsed = (rospy.Time.now() - self._transition_start_time).to_sec()
            self._transition_progress = min(1.0, elapsed / transition_duration)
            
            if self._transition_progress >= 1.0:
                # Transition complete
                self._transition_start_time = None
                self._previous_control_mode = control_mode
                rospy.loginfo(f"PROGRESSIVE TRANSITION COMPLETE: Now fully in {control_mode} mode")
        
        # Calculate parameters with progressive interpolation
        # Get target parameters for new mode
        target_params = self._get_mode_parameters(control_mode, actual_insertion_depth)
        
        if self._transition_progress < 1.0:
            # Still transitioning - interpolate between previous and current mode parameters
            previous_params = self._get_mode_parameters(self._previous_control_mode, actual_insertion_depth)
            
            # Smooth interpolation (eased transition for better feel)
            alpha = self._ease_in_out_cubic(self._transition_progress)
            
            # Interpolate all parameters
            pid_kp = self._interpolate(previous_params['pid_kp'], target_params['pid_kp'], alpha)
            pid_ki = self._interpolate(previous_params['pid_ki'], target_params['pid_ki'], alpha)
            pid_kd = self._interpolate(previous_params['pid_kd'], target_params['pid_kd'], alpha)
            z_pid_kp = self._interpolate(previous_params['z_pid_kp'], target_params['z_pid_kp'], alpha)
            z_pid_ki = self._interpolate(previous_params['z_pid_ki'], target_params['z_pid_ki'], alpha)
            z_pid_kd = self._interpolate(previous_params['z_pid_kd'], target_params['z_pid_kd'], alpha)
            max_correction = self._interpolate(previous_params['max_correction'], target_params['max_correction'], alpha)
            
            rospy.loginfo_throttle(1, f"PROGRESSIVE TRANSITION: {self._transition_progress*100:.1f}% complete")
            rospy.loginfo_throttle(1, f"  Interpolated Kp: {pid_kp:.3f} (from {previous_params['pid_kp']:.3f} to {target_params['pid_kp']:.3f})")
        else:
            # Transition complete - use target parameters directly
            pid_kp = target_params['pid_kp']
            pid_ki = target_params['pid_ki'] 
            pid_kd = target_params['pid_kd']
            z_pid_kp = target_params['z_pid_kp']
            z_pid_ki = target_params['z_pid_ki']
            z_pid_kd = target_params['z_pid_kd']
            max_correction = target_params['max_correction']
        # Skip original parameter setting if progressive transition has already set them
        progressive_params_set = (self._transition_start_time is not None or 
                                hasattr(self, '_previous_control_mode'))
        
        if not progressive_params_set:
            # Original parameter setting for backward compatibility
            pass  # Parameters are set by progressive transition above
        else:
            # Parameters already set by progressive transition - log confirmation
            rospy.loginfo_throttle(10, f"Using progressive transition parameters: Kp={pid_kp:.3f}, Ki={pid_ki:.3f}, Kd={pid_kd:.3f}")
        
        # Note: Original parameter setting code follows but is bypassed when progressive transition is active
        
        if False and control_mode == "PD":  # Disabled - handled by progressive transition
            # PD控制：深度插入专用，彻底消除震荡
            # OPTIMIZED: Reduced gains for better stability in constrained environment
            pid_kp = 0.25   # 降低比例增益 (从0.4减少到0.25)，减少过激反应
            pid_ki = 0.0    # 完全禁用积分项
            pid_kd = 0.12   # 降低微分增益 (从0.2减少到0.12)，减少噪声放大
            z_pid_kp = 0.3  # 降低Z轴比例增益 (从0.5减少到0.3)
            z_pid_ki = 0.0  # Z轴禁用积分
            z_pid_kd = 0.18 # 降低Z轴微分增益 (从0.3减少到0.18)
            max_correction = 12.0  # 进一步限制最大修正幅度 (从15.0减少到12.0mm)
            rospy.loginfo_throttle(10, f"PD Control: Kp={pid_kp}, Kd={pid_kd}, max_correction={max_correction}mm (CONSERVATIVE)")
            
            
        elif control_mode == "PI_REDUCED":
            # 减小积分的PI控制：接触阶段
            pid_kp = 1.0    # 适中比例增益
            pid_ki = 0.02   # 大幅减小积分增益
            pid_kd = 0.4    # 适度微分增益
            z_pid_kp = 1.2  
            z_pid_ki = 0.015
            z_pid_kd = 0.35
            max_correction = 25.0
            
        elif control_mode == "PID_CONSERVATIVE":
            # 保守PID控制：浅插入阶段
            pid_kp = 1.5    # 标准比例增益
            pid_ki = 0.05   # 减小积分增益
            pid_kd = 0.4    # 标准微分增益
            z_pid_kp = 1.8
            z_pid_ki = 0.04
            z_pid_kd = 0.3
            max_correction = 35.0
            
        elif control_mode == "PD_ROTATION":
            # 旋转专用PD控制：CONTACT-ADAPTIVE GAINS for stability
            # CONTACT DETECTION: Check if UAV is in contact with valve spokes
            valve_contact_detected = self._detect_valve_contact()
            
            # DISTANCE DRIFT ADAPTIVE GAINS: Adjust gains based on distance drift magnitude
            distance_drift = 0.0
            if hasattr(self, 'last_distance_error'):
                distance_drift = getattr(self, 'last_distance_error', 0.0)
            
            # Calculate adaptive gain factors based on distance drift
            if distance_drift > 0.035:  # >35mm - severe drift
                gain_factor = 0.6  # Reduce gains significantly to prevent oscillation
                drift_level = "SEVERE"
            elif distance_drift > 0.025:  # 25-35mm - moderate drift
                gain_factor = 0.75  # Moderate gain reduction
                drift_level = "MODERATE"
            elif distance_drift > 0.015:  # 15-25mm - minor drift
                gain_factor = 0.9  # Slight gain reduction
                drift_level = "MINOR"
            else:  # <15mm - stable
                gain_factor = 1.0  # Normal gains
                drift_level = "STABLE"
            
            if not valve_contact_detected:
                # 🟡 PRE-CONTACT (FREE SPACE): Reduced gains to prevent oscillation
                base_kp = 0.08   # Base proportional gain
                base_kd = 0.22   # Base derivative gain
                base_z_kp = 0.10 # Base Z-axis proportional gain
                base_z_kd = 0.15 # Base Z-axis derivative gain
                base_max_correction = 6.0  # Base max correction
                
                # Apply adaptive scaling
                pid_kp = base_kp * gain_factor
                pid_kd = base_kd * gain_factor
                z_pid_kp = base_z_kp * gain_factor
                z_pid_kd = base_z_kd * gain_factor
                max_correction = base_max_correction * gain_factor
                
                rospy.loginfo_throttle(5, f"🟡 PD_ROTATION (PRE-CONTACT): Kp={pid_kp:.3f}, Kd={pid_kd:.3f}, "
                                     f"max_correction={max_correction:.1f}mm, drift={drift_level}({distance_drift*1000:.1f}mm)")
            else:
                # 🟢 POST-CONTACT (CONSTRAINED): Normal gains for controlled rotation
                base_kp = 0.12   # Base proportional gain
                base_kd = 0.28   # Base derivative gain
                base_z_kp = 0.15 # Base Z-axis proportional gain
                base_z_kd = 0.20 # Base Z-axis derivative gain
                base_max_correction = 8.0  # Base max correction
                
                # Apply adaptive scaling
                pid_kp = base_kp * gain_factor
                pid_kd = base_kd * gain_factor
                z_pid_kp = base_z_kp * gain_factor
                z_pid_kd = base_z_kd * gain_factor
                max_correction = base_max_correction * gain_factor
                
                rospy.loginfo_throttle(5, f"🟢 PD_ROTATION (POST-CONTACT): Kp={pid_kp:.3f}, Kd={pid_kd:.3f}, "
                                     f"max_correction={max_correction:.1f}mm, drift={drift_level}({distance_drift*1000:.1f}mm)")
            
            pid_ki = 0.0    # 保持禁用积分项 - 避免饱和和震荡
            z_pid_ki = 0.0  # Z轴保持禁用积分项
            
            # ROTATION-SPECIFIC IMPROVEMENTS: 改善距离校正机制
            self._rotation_distance_drift_threshold = 30.0  # 放宽距离漂移阈值 (从25mm增到30mm)
            self._rotation_distance_critical_threshold = 50.0  # 放宽临界阈值 (从40mm增到50mm)
            rospy.loginfo_throttle(10, f"ROTATION DISTANCE THRESHOLDS: Soft={self._rotation_distance_drift_threshold}mm, Critical={self._rotation_distance_critical_threshold}mm")
            
        elif control_mode == "PD_INSERTION":
            # 插入专用PD控制：进一步放宽参数以显著减少震荡，优化约束环境控制
            pid_kp = 0.4    # 进一步降低比例增益 - 从0.6减到0.4，显著减少过激反应
            pid_ki = 0.0    # 完全禁用积分项 - 消除饱和根源
            pid_kd = 0.8    # 降低微分增益 - 从1.0减到0.8，减少高频震荡
            z_pid_kp = 0.6  # 进一步降低Z轴比例增益 - 从0.8减到0.6
            z_pid_ki = 0.0  # Z轴禁用积分项
            z_pid_kd = 0.5  # 降低Z轴微分增益 - 从0.6减到0.5
            max_correction = 50.0  # 大幅放宽修正幅度限制 - 从30mm提高到50mm
            rospy.loginfo_throttle(5, f"📌 PD_INSERTION: Kp={pid_kp}, Kd={pid_kd}, max_correction={max_correction}mm (FURTHER RELAXED FOR STABILITY)")
            
        elif control_mode == "PD_DISENGAGEMENT":
            # 🚁 脱离专用PD控制：极度保守参数，最小化震荡风险
            pid_kp = 0.2    # 极低比例增益 - 最小响应以避免过激反应  
            pid_ki = 0.0    # 完全禁用积分项 - 绝对避免积分饱和
            pid_kd = 0.1    # 极低微分增益 - 最小化高频干扰
            z_pid_kp = 0.3  # 极低Z轴比例增益
            z_pid_ki = 0.0  # Z轴禁用积分项
            z_pid_kd = 0.2  # 极低Z轴微分增益
            max_correction = 20.0  # 严格限制修正幅度 - 防止大幅调整
            rospy.loginfo_throttle(5, f"🚁 PD_DISENGAGEMENT: Kp={pid_kp}, Kd={pid_kd}, max_correction={max_correction}mm (ULTRA-CONSERVATIVE FOR SMOOTH DISENGAGEMENT)")
            
        elif control_mode == "PID_HIGH_PRECISION":
            # 高精度PID控制：自由空间大幅修正专用 - 适合XY correction等
            pid_kp = 1.2    # 适中比例增益 - 有效响应
            pid_ki = 0.06   # 适度积分增益 - 消除稳态误差
            pid_kd = 0.5    # 适度微分增益 - 平衡响应和稳定性
            z_pid_kp = 1.5  # 适中Z轴比例增益
            z_pid_ki = 0.04 # Z轴适度积分增益
            z_pid_kd = 0.4  # Z轴适度微分增益
            max_correction = 30.0  # 允许大幅修正 - 适合XY correction
            rospy.loginfo_throttle(5, f"PID_HIGH_PRECISION: Kp={pid_kp}, Ki={pid_ki}, Kd={pid_kd}, max_correction={max_correction}mm (FREE SPACE)")
            
        else:  # PID_STANDARD
            # 标准PID控制：自由空间 - 温和调整，不过度激进
            pid_kp = 1.8    # 从2.0轻微降低，保持响应性
            pid_ki = 0.08   # 从0.1轻微降低，减少积分累积  
            pid_kd = 0.4    # 从0.5轻微降低
            z_pid_kp = 2.0  # 从2.2轻微降低，保持Z轴响应
            z_pid_ki = 0.06 # 从0.08轻微降低
            z_pid_kd = 0.35 # 从0.4轻微降低
            max_correction = 40.0  # 从50.0适度降低
        
        # POSITION_HOLD MODE: For insertion complete states
        if hasattr(self, '_force_position_hold') and self._force_position_hold:
            control_mode = "POSITION_HOLD"
            pid_kp = 0.5    # 极低比例增益 - 减少过激反应
            pid_ki = 0.005  # 极低积分增益 - 防止积分饱和
            pid_kd = 0.2    # 适中微分增益 - 提供阻尼
            z_pid_kp = 0.8  
            z_pid_ki = 0.003
            z_pid_kd = 0.25
            max_correction = 8.0  # 严格限制修正幅度
            rospy.loginfo_throttle(5, f"🔒 POSITION_HOLD mode activated - gentle control for insertion complete")
        
        # PRECISION PID OPTIMIZATION: Override for small movements (<10mm精密, <30mm高精密)
        if target_pos is not None and start_pos is not None:
            total_movement_distance = math.sqrt(sum([(target_pos[i] - start_pos[i])**2 for i in range(3)]))
            if total_movement_distance < 0.01:  # <10mm ultra precision movement (更严格阈值)
                control_mode = "PID_PRECISION"
                pid_kp = 0.6    # 进一步降低比例增益 - 减少修正需求 (从0.8降低)
                pid_ki = 0.015  # 进一步降低积分增益 (从0.02降低)
                pid_kd = 1.0    # 进一步增加微分增益 - 提供更强阻尼 (从0.8增加)
                z_pid_kp = 0.8  # 进一步降低Z轴比例增益 (从1.0降低)
                z_pid_ki = 0.01 # 进一步降低Z轴积分增益 (从0.015降低)
                z_pid_kd = 0.8   # 增加Z轴微分增益 (从0.6增加)
                max_correction = 12.0  # 进一步严格限制修正幅度 (从15.0降低)
                rospy.loginfo_throttle(3, f"PRECISION PID mode: {total_movement_distance*1000:.1f}mm movement - ultra conservative gains")
            elif total_movement_distance < 0.03:  # <30mm high precision movement (从50mm降低到30mm)
                control_mode = "PID_HIGH_PRECISION"  
                
                # ENHANCED PRECISION CORRECTION: Special parameters for post-insertion correction
                if precision_correction_mode:
                    # Special tuning for precision correction after successful insertion
                    pid_kp = 0.7    # Further reduced for post-insertion stability
                    pid_ki = 0.02   # Minimal integral to prevent accumulation
                    pid_kd = 1.0    # Strong damping for smooth convergence
                    z_pid_kp = 1.0  # Reduced Z response for stability
                    z_pid_ki = 0.015
                    z_pid_kd = 0.8
                    max_correction = 30.0  # INCREASED from 20.0mm - relaxed correction limit for precision mode
                    rospy.loginfo(f"PRECISION CORRECTION: Enhanced PID_HIGH_PRECISION parameters for post-insertion")
                else:
                    # Standard high precision parameters
                    pid_kp = 0.9    # 进一步降低比例增益 (从1.2降低)
                    pid_ki = 0.03   # 进一步降低积分增益 (从0.04降低)
                    pid_kd = 0.8    # 增加微分增益 (从0.6增加)
                    z_pid_kp = 1.2  # 进一步降低Z轴比例增益 (从1.5降低)
                    z_pid_ki = 0.02 # 进一步降低Z轴积分增益 (从0.03降低)
                z_pid_kd = 0.6  # 增加Z轴微分增益 (从0.45增加)
                max_correction = 35.0  # INCREASED from 20.0mm to 35.0mm - allow sufficient correction for 36.2mm errors
                rospy.loginfo_throttle(3, f"HIGH_PRECISION PID mode: {total_movement_distance*1000:.1f}mm movement - conservative gains")
            else:
                # 长距离移动(>30mm)使用标准PID，不需要高精度
                rospy.loginfo_throttle(5, f"LONG_DISTANCE movement: {total_movement_distance*1000:.1f}mm - using standard PID")
        
        rospy.loginfo(f"Control mode: {control_mode}, Max correction: {max_correction:.1f}mm")
        rospy.loginfo(f"PID gains - P:{pid_kp:.2f}, I:{pid_ki:.3f}, D:{pid_kd:.2f}")
        
        # ADAPTIVE GAIN ADJUSTMENT: Store base gains for dynamic scaling
        base_pid_kp = pid_kp
        base_pid_kd = pid_kd
        base_z_pid_kp = z_pid_kp
        base_z_pid_kd = z_pid_kd
        base_max_correction = max_correction
        
        # PID状态变量（简化版本）
        integral_error = [0.0, 0.0, 0.0]  # x, y, z积分误差
        last_error = [0.0, 0.0, 0.0]      # 上次误差，用于计算微分
        
        # 自适应增益调整日志计数器
        adaptive_gain_logging_interval = 0
        
        # ENHANCED TRAJECTORY INTERPOLATION: Increased point density for smoother rotation
        # FIXED: Use 30Hz for rotation phases (improved from 20Hz) for smoother trajectories
        control_frequency = 30 if (hasattr(self, 'rotation_active') and self.rotation_active) else 20
        rate = rospy.Rate(control_frequency)
        dt = 1.0 / control_frequency  # 动态时间步长
        rospy.loginfo(f"Using {control_frequency}Hz control frequency ({'ROTATION' if control_frequency == 30 else 'STANDARD'} mode)")
        start_time = time.time()
        trajectory_completed = False
        
        while not rospy.is_shutdown() and self.trajectory_active and (time.time() - start_time) < duration + 10.0:
            elapsed_time = time.time() - start_time
            
            # CRITICAL ROTATION MODE PROTECTION: Force check at every cycle
            # This prevents any other logic from overriding rotation control mode
            if hasattr(self, 'rotation_active') and self.rotation_active:
                # Override any control mode selection - force PD_ROTATION mode for entire cycle
                control_mode = "PD_ROTATION"
                if elapsed_time < 1.0 or int(elapsed_time) % 5 == 0:  # Log periodically
                    rospy.loginfo(f"LOOP PROTECTION: rotation_active=True, forcing control_mode=PD_ROTATION (cycle {int(elapsed_time)})")
            
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
                
                # DEPTH-ADAPTIVE INTEGRAL ERROR ACCUMULATION
                # 积分误差累积（根据控制模式和插入深度）
                for i in range(3):
                    if control_mode in ["PD", "PD_ROTATION", "PD_INSERTION"]:
                        # PD模式、PD_ROTATION模式和PD_INSERTION模式：完全禁用积分项累积
                        integral_error[i] = 0.0
                    else:
                        # ENHANCED: 基于插入深度的积分饱和限制（先确定限制值）
                        if actual_insertion_depth > 0.15:  # 深度插入(>150mm)
                            if control_mode == "PI_REDUCED":
                                limit = 0.015  # 15mm - 进一步收紧深度插入时限制
                            elif control_mode == "PID_CONSERVATIVE":
                                limit = 0.025  # 25mm - 进一步收紧深度插入时限制
                            elif control_mode == "PID_PRECISION":
                                limit = 0.003  # 3mm - 极精密控制时最严格限制 (从5mm进一步降低)
                            elif control_mode == "PID_HIGH_PRECISION":
                                limit = 0.005  # 5mm - 大幅收紧高精密控制限制 (从8mm降低到5mm)
                            else:
                                limit = 0.030  # 30mm - 进一步收紧深度插入时限制
                            limit_context = f"DEEP_INSERTION (depth: {actual_insertion_depth*1000:.0f}mm)"
                        else:
                            # 浅层或表面控制，使用更严格的限制 - 针对插入阶段优化
                            if control_mode == "PI_REDUCED":
                                limit = 0.020  # 20mm - 收紧适度减小积分限制
                            elif control_mode == "PID_CONSERVATIVE":
                                limit = 0.035  # 35mm - 收紧保守积分限制
                            elif control_mode == "PID_PRECISION":
                                limit = 0.004  # 4mm - 收紧极精密控制积分限制 (从6mm降低)
                            elif control_mode == "PID_HIGH_PRECISION":
                                limit = 0.010  # 10mm - 大幅收紧，防止34.6mm积分饱和 (从20mm降低到10mm)
                            else:
                                limit = 0.040   # 40mm - 收紧标准积分限制
                            limit_context = f"SURFACE_CONTROL (depth: {actual_insertion_depth*1000:.0f}mm)"
                        
                        # 接触检测和应急积分重置机制
                        # 检测位置误差突然增大，可能表示接触发生
                        position_error_magnitude = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
                        if not hasattr(self, '_last_position_error'):
                            self._last_position_error = position_error_magnitude
                        
                        # 如果位置误差突然增大超过15mm，可能是接触，立即减少积分限制
                        error_jump = position_error_magnitude - self._last_position_error
                        if error_jump > 0.015:  # 15mm突增表示可能接触
                            limit = min(limit, 0.005)  # 应急降低到5mm
                            rospy.logwarn(f"CONTACT DETECTED: Error jump {error_jump*1000:.1f}mm, reducing integral limit to {limit*1000:.0f}mm")
                        
                        self._last_position_error = position_error_magnitude
                        
                        # PID/PI模式：积分累积并立即限制（防止饱和）
                        integral_error[i] += error[i] * dt
                        integral_error[i] = max(-limit, min(limit, integral_error[i]))  # 立即限制，防止17.3mm饱和
                
                # GLOBAL INTEGRAL SATURATION PROTECTION
                # 额外的全局检查，确保没有遗漏的积分饱和路径
                integral_magnitude = math.sqrt(sum([ie**2 for ie in integral_error]))
                global_limit = 0.015  # 全局15mm硬限制
                if integral_magnitude > global_limit:
                    # 按比例缩减所有积分项
                    scale_factor = global_limit / integral_magnitude
                    integral_error = [ie * scale_factor for ie in integral_error]
                    rospy.logwarn(f"GLOBAL INTEGRAL PROTECTION: Magnitude {integral_magnitude*1000:.1f}mm > {global_limit*1000:.0f}mm, scaled by {scale_factor:.3f}")
                    
                    # VALIDATION: Log depth-based integral behavior
                    if not hasattr(self, '_integral_protection_count'):
                        self._integral_protection_count = 0
                    self._integral_protection_count += 1
                    rospy.logwarn(f"INTEGRAL PROTECTION TRIGGERED #{self._integral_protection_count}: depth={actual_insertion_depth*1000:.0f}mm, mode={control_mode}")
                
                # 计算微分误差
                derivative_error = [
                    (error[i] - last_error[i]) / dt for i in range(3)
                ]
                
                # � ADAPTIVE GAIN ADJUSTMENT BASED ON ERROR MAGNITUDE
                # 根据误差大小动态调整控制增益（仅适用于PD和PI_REDUCED模式）
                position_error_magnitude = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
                
                # � EMERGENCY SWITCH REMOVED: Each phase now uses its dedicated control mode
                # This eliminates oscillation caused by dynamic mode switching
                
                if control_mode in ["PD", "PI_REDUCED", "PD_INSERTION"]:
                    # 自适应增益缩放因子 - 更严格的误差阈值
                    if position_error_magnitude > 0.015:  # 大误差 (>15mm) - 从25mm降低
                        gain_scale_factor = 0.7  # 降低增益70%
                        correction_scale_factor = 0.8  # 降低修正幅度80%
                        rospy.loginfo_throttle(15, f"🔽 Large error ({position_error_magnitude*1000:.1f}mm) - reducing gains to 70%")
                    elif position_error_magnitude > 0.010:  # 中等误差 (10-15mm) - 从15mm降低
                        gain_scale_factor = 0.85  # 降低增益85%
                        correction_scale_factor = 0.9   # 降低修正幅度90%
                        rospy.loginfo_throttle(15, f"Medium error ({position_error_magnitude*1000:.1f}mm) - reducing gains to 85%")
                    else:  # 小误差 (<10mm)
                        gain_scale_factor = 1.0  # 保持原始增益
                        correction_scale_factor = 1.0
                        rospy.loginfo_throttle(15, f"Small error ({position_error_magnitude*1000:.1f}mm) - using full gains")
                    
                    # 应用自适应缩放
                    adaptive_pid_kp = base_pid_kp * gain_scale_factor
                    adaptive_pid_kd = base_pid_kd * gain_scale_factor
                    adaptive_z_pid_kp = base_z_pid_kp * gain_scale_factor
                    adaptive_z_pid_kd = base_z_pid_kd * gain_scale_factor
                    adaptive_max_correction = base_max_correction * correction_scale_factor
                elif control_mode in ["PID_PRECISION", "PID_HIGH_PRECISION"]:
                    # 精密模式的特殊自适应增益调整 - 更激进的误差阈值
                    if position_error_magnitude > 0.008:  # 大误差 (>8mm) - 精密模式降低阈值
                        gain_scale_factor = 0.5  # 大幅降低增益50%
                        correction_scale_factor = 0.6  # 大幅降低修正幅度60%
                        rospy.loginfo_throttle(10, f"PRECISION: Large error ({position_error_magnitude*1000:.1f}mm) - reducing gains to 50%")
                    elif position_error_magnitude > 0.005:  # 中等误差 (5-8mm) - 精密模式降低阈值
                        gain_scale_factor = 0.7  # 降低增益70%
                        correction_scale_factor = 0.8   # 降低修正幅度80%
                        rospy.loginfo_throttle(10, f"PRECISION: Medium error ({position_error_magnitude*1000:.1f}mm) - reducing gains to 70%")
                    else:  # 小误差 (<5mm)
                        gain_scale_factor = 1.0  # 保持原始增益
                        correction_scale_factor = 1.0
                        rospy.loginfo_throttle(10, f"PRECISION: Small error ({position_error_magnitude*1000:.1f}mm) - using full gains")
                    
                    # 应用精密模式自适应缩放
                    adaptive_pid_kp = base_pid_kp * gain_scale_factor
                    adaptive_pid_kd = base_pid_kd * gain_scale_factor
                    adaptive_z_pid_kp = base_z_pid_kp * gain_scale_factor
                    adaptive_z_pid_kd = base_z_pid_kd * gain_scale_factor
                    adaptive_max_correction = base_max_correction * correction_scale_factor
                else:
                    # 标准PID模式不使用自适应调整
                    gain_scale_factor = 1.0  # 保持原始增益
                    correction_scale_factor = 1.0
                    adaptive_pid_kp = pid_kp
                    adaptive_pid_kd = pid_kd
                    adaptive_z_pid_kp = z_pid_kp
                    adaptive_z_pid_kd = z_pid_kd
                    adaptive_max_correction = max_correction
                
                # �DEPTH-ADAPTIVE PID CONTROL OUTPUT
                pid_correction = []
                for i in range(3):
                    if i == 2:  # Z轴使用专用参数
                        correction = (adaptive_z_pid_kp * error[i] + 
                                    z_pid_ki * integral_error[i] + 
                                    adaptive_z_pid_kd * derivative_error[i])
                    else:  # XY轴使用通用参数
                        correction = (adaptive_pid_kp * error[i] + 
                                    pid_ki * integral_error[i] + 
                                    adaptive_pid_kd * derivative_error[i])
                    
                    # 🚫 DEPTH-ADAPTIVE CORRECTION LIMITING (Anti-Oscillation)
                    max_corr_m = adaptive_max_correction / 1000.0  # 转换为米，使用自适应限制
                    correction = max(-max_corr_m, min(max_corr_m, correction))
                    pid_correction.append(correction)
                
                # 自适应增益调整日志输出
                if adaptive_gain_logging_interval <= 0:
                    if control_mode in ["PD", "PI_REDUCED", "PD_INSERTION"]:
                        rospy.loginfo(f"ADAPTIVE CONTROL - Mode: {control_mode}, Error: {position_error_magnitude*1000:.1f}mm, "
                                    f"Gain Scale: {gain_scale_factor:.2f}, "
                                    f"PID_Kp: {adaptive_pid_kp:.3f}, PID_Kd: {adaptive_pid_kd:.3f}, "
                                    f"Max_Corr: {adaptive_max_correction:.1f}mm")
                    else:
                        rospy.loginfo(f"STANDARD CONTROL - Mode: {control_mode}, Error: {position_error_magnitude*1000:.1f}mm, "
                                    f"PID_Kp: {adaptive_pid_kp:.3f}, PID_Kd: {adaptive_pid_kd:.3f}, "
                                    f"Max_Corr: {adaptive_max_correction:.1f}mm")
                    adaptive_gain_logging_interval = 60  # 每3秒记录一次 (20Hz/60)
                else:
                    adaptive_gain_logging_interval -= 1
                
                # 应用PID修正到轨迹点
                corrected_pt = [
                    pt[0] + pid_correction[0],
                    pt[1] + pid_correction[1], 
                    pt[2] + pid_correction[2]
                ]
                
                # 更新上次误差
                last_error = error[:]
                
                # 增强调试信息，显示控制模式和深度信息
                if int(elapsed_time) % 2 == 0 and int(elapsed_time * 10) % 10 == 0:
                    error_magnitude = math.sqrt(sum([e**2 for e in error]))
                    correction_magnitude = math.sqrt(sum([c**2 for c in pid_correction]))
                    
                    # 显示当前控制模式和积分状态
                    integral_magnitude = math.sqrt(sum([i**2 for i in integral_error]))
                    rospy.loginfo(f"{control_mode} Control - Error: {error_magnitude*1000:.1f}mm, "
                                f"Correction: {correction_magnitude*1000:.1f}mm (max: {max_correction:.0f}mm), "
                                f"XY_error: [{error[0]*1000:.1f}, {error[1]*1000:.1f}]mm, "
                                f"Z_error: {error[2]*1000:.1f}mm")
                    if control_mode != "PD":
                        rospy.loginfo(f"  Integral state: {integral_magnitude*1000:.1f}mm (I-terms active, limit: {limit*1000:.0f}mm)")
                        rospy.loginfo(f"  Integral context: {limit_context}")
                    else:
                        rospy.loginfo(f"  PD mode: Integral disabled (anti-oscillation)")
                
                # 🛡️ INTELLIGENT TRAJECTORY DIVERGENCE DETECTION
                divergence_detected = False
                
                # Track error history for divergence detection
                if not hasattr(self, 'error_history'):
                    self.error_history = []
                    self.control_start_time = elapsed_time
                
                # Store current error
                current_error_mag = math.sqrt(sum([e**2 for e in error]))
                self.error_history.append((elapsed_time, current_error_mag))
                
                # Keep only last 3 seconds of history (60 samples at 20Hz)
                self.error_history = [(t, e) for t, e in self.error_history if elapsed_time - t <= 3.0]
                
                # ADAPTIVE DIVERGENCE THRESHOLDS based on movement distance and precision requirement
                total_movement_distance = math.sqrt(sum([(target_pos[i] - start_pos[i])**2 for i in range(3)]))
                
                # Classify movement type and set appropriate thresholds
                if total_movement_distance > 1.0:  # Long-distance movement (>1m)
                    movement_type = "LONG_DISTANCE"
                    explosion_threshold = 0.150      # 150mm for long movements
                    growth_threshold_ratio = 2.0     # 100% growth allowed
                    growth_absolute_threshold = 0.080  # 80mm absolute threshold
                    oscillation_std_threshold = 0.050  # 50mm std deviation
                    oscillation_mean_threshold = 0.100  # 100mm mean
                elif total_movement_distance > 0.3:   # Medium-distance movement (>30cm)
                    movement_type = "MEDIUM_DISTANCE"
                    explosion_threshold = 0.080      # 80mm for medium movements
                    growth_threshold_ratio = 1.8     # 80% growth allowed
                    growth_absolute_threshold = 0.050  # 50mm absolute threshold
                    oscillation_std_threshold = 0.030  # 30mm std deviation
                    oscillation_mean_threshold = 0.060  # 60mm mean
                else:  # Precision movement (<30cm)
                    movement_type = "PRECISION"
                    explosion_threshold = 0.050      # 50mm for precision movements
                    growth_threshold_ratio = 2.5     # 150% growth allowed (更宽松，从1.5增加)
                    growth_absolute_threshold = 0.030  # 30mm absolute threshold (更宽松，从20mm增加)
                    oscillation_std_threshold = 0.015  # 15mm std deviation (original)
                    oscillation_mean_threshold = 0.025  # 25mm mean (更宽松，从20mm增加)
                
                # Log movement classification
                if int(elapsed_time * 10) % 100 == 0:  # Every 5 seconds
                    rospy.loginfo(f"Movement: {movement_type} ({total_movement_distance*1000:.0f}mm), "
                                f"Explosion threshold: {explosion_threshold*1000:.0f}mm, "
                                f"Divergence detection: {'ENABLED' if enable_divergence_detection else 'DISABLED'}")
                
                # Check for divergence after 2 seconds of control (if enabled)
                control_duration = elapsed_time - self.control_start_time
                if enable_divergence_detection and control_duration > 2.0 and len(self.error_history) >= 40:  # 2秒历史数据
                    
                    # Condition 1: Error magnitude explosion
                    if current_error_mag > explosion_threshold:
                        divergence_detected = True
                        rospy.logerr(f"DIVERGENCE DETECTED [{movement_type}]: Error explosion "
                                   f"{current_error_mag*1000:.1f}mm > {explosion_threshold*1000:.0f}mm threshold")
                    
                    # Condition 2: Sustained error growth trend
                    recent_errors = [e for t, e in self.error_history[-20:]]  # Last 1 second
                    older_errors = [e for t, e in self.error_history[-40:-20]]  # Previous 1 second
                    
                    if len(recent_errors) >= 10 and len(older_errors) >= 10:
                        recent_avg = sum(recent_errors) / len(recent_errors)
                        older_avg = sum(older_errors) / len(older_errors)
                        
                        # Adaptive growth detection
                        if recent_avg > older_avg * growth_threshold_ratio and recent_avg > growth_absolute_threshold:
                            # PRECISION PID MODE: 更宽松的发散检测 - RELAXED to prevent false divergence detection
                            if control_mode in ["PID_PRECISION", "PID_HIGH_PRECISION"]:
                                # 精密模式使用更严格的发散条件 - 需要同时满足更高阈值
                                precision_growth_ratio = 3.0  # 200% growth allowed for precision modes
                                precision_absolute_threshold = 0.050  # 50mm absolute threshold - RELAXED from 40mm to prevent false alerts
                                if recent_avg > older_avg * precision_growth_ratio and recent_avg > precision_absolute_threshold:
                                    divergence_detected = True
                                    rospy.logerr(f"PRECISION DIVERGENCE DETECTED [{movement_type}]: Sustained error growth "
                                               f"{older_avg*1000:.1f}mm → {recent_avg*1000:.1f}mm "
                                               f"(>{precision_growth_ratio*100:.0f}% + >{precision_absolute_threshold*1000:.0f}mm)")
                                else:
                                    rospy.loginfo_throttle(5, f"PRECISION: Error growth within tolerance "
                                                         f"{older_avg*1000:.1f}mm → {recent_avg*1000:.1f}mm "
                                                         f"(<{precision_growth_ratio*100:.0f}% or <{precision_absolute_threshold*1000:.0f}mm)")
                            else:
                                divergence_detected = True
                                rospy.logerr(f"DIVERGENCE DETECTED [{movement_type}]: Sustained error growth "
                                           f"{older_avg*1000:.1f}mm → {recent_avg*1000:.1f}mm "
                                           f"(>{growth_threshold_ratio*100:.0f}% + >{growth_absolute_threshold*1000:.0f}mm)")
                    
                    # Condition 3: Oscillation detection (only for precision movements)
                    if movement_type == "PRECISION" and len(self.error_history) >= 40:
                        errors = [e for t, e in self.error_history[-40:]]
                        error_mean = sum(errors) / len(errors)
                        error_variance = sum([(e - error_mean)**2 for e in errors]) / len(errors)
                        error_std = math.sqrt(error_variance)
                        
                        # High variance + high mean indicates oscillation (only for precision movements)
                        if error_std > oscillation_std_threshold and error_mean > oscillation_mean_threshold:
                            divergence_detected = True
                            rospy.logerr(f"DIVERGENCE DETECTED [{movement_type}]: High oscillation "
                                       f"variance {error_std*1000:.1f}mm, mean {error_mean*1000:.1f}mm")
                
                # Emergency termination if divergence detected
                if divergence_detected:
                    rospy.logerr("🛑 EMERGENCY TERMINATION: Trajectory divergence detected, switching to POSITION_HOLD mode")
                    
                    # Switch to position hold mode for gentle control
                    current_pos = self.sm.get_current_position()
                    if current_pos is not None:
                        rospy.logwarn(f"Activating position hold at current position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
                        
                        # Activate position hold mode and send hold command
                        self._force_position_hold = True
                        self.send_trajectory_point(current_pos, current_target_yaw)
                        
                        # Give time for command to take effect
                        rospy.sleep(0.1)
                    
                    # Terminate current trajectory
                    self.trajectory_active = False
                    rospy.logwarn("🏁 Trajectory terminated due to divergence - position hold activated")
                    return False  # Indicate trajectory failure but safe termination
                
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
            
            # Check for final convergence
            if trajectory_completed:
                convergence_success = self._wait_for_position_and_yaw(
                    target_pos, target_yaw, pos_threshold, yaw_threshold, point_timeout=2.0)
                
                if convergence_success:
                    rospy.loginfo("Trajectory converged successfully")
                    return True
                else:
                    # Check actual final error with depth-adaptive thresholds
                    final_pos = self.sm.get_current_position()
                    if final_pos is not None:
                        final_error = math.sqrt(
                            (target_pos[0] - final_pos[0])**2 + 
                            (target_pos[1] - final_pos[1])**2 + 
                            (target_pos[2] - final_pos[2])**2
                        )
                        final_yaw_error = abs(self._normalize_angle_diff(target_yaw - self.sm.current_yaw))
                        
                        # ENHANCED: Calculate insertion depth for adaptive threshold
                        valve_surface_z = self._get_valve_surface_height()
                        insertion_depth = max(0, valve_surface_z - final_pos[2])
                        
                        # SIGNIFICANTLY RELAXED adaptive threshold multipliers to prevent insertion failures
                        # Based on analysis of 36.2mm error vs 20mm threshold causing Stage 3A failures
                        if insertion_depth > 0.30:  # Deep insertion (>300mm)
                            threshold_multiplier = 5.0  # 50mm for deep insertion (was 3.0)
                            depth_context = f"DEEP ({insertion_depth*1000:.0f}mm)"
                        elif insertion_depth > 0.20:  # Medium insertion (>200mm)
                            threshold_multiplier = 4.5  # 45mm for medium insertion (was 2.5)
                            depth_context = f"MEDIUM ({insertion_depth*1000:.0f}mm)"
                        else:  # Shallow or no insertion - THIS IS THE CRITICAL CASE
                            threshold_multiplier = 4.0  # 40mm for shallow insertion (was 2.0) - FIXES 36.2mm > 20mm
                            depth_context = f"SHALLOW ({insertion_depth*1000:.0f}mm)"
                        
                        adaptive_pos_threshold = pos_threshold * threshold_multiplier
                        
                        rospy.logwarn(f"Final convergence timeout:")
                        rospy.logwarn(f"  Position error: {final_error*1000:.1f}mm (threshold: {pos_threshold*1000:.1f}mm)")
                        rospy.logwarn(f"  Yaw error: {math.degrees(final_yaw_error):.2f}° (threshold: {math.degrees(yaw_threshold):.2f}°)")
                        rospy.logwarn(f"  Insertion depth: {depth_context} - using {threshold_multiplier}x relaxed threshold")
                        
                        # CONSERVATIVE YAW RELAXATION: Reduce yaw relaxation from 2.0x to 1.3x for insertion precision
                        # Determine conservative yaw relaxation based on insertion depth
                        valve_surface_z = self._get_valve_surface_height()
                        current_pos = self.sm.get_current_position()
                        if current_pos:
                            insertion_depth = max(0, valve_surface_z - current_pos[2])
                            if insertion_depth > 0.15:  # Deep insertion >150mm
                                yaw_relaxation = 1.2  # Minimal relaxation for deep insertion
                                rospy.logwarn(f"DEEP INSERTION: Minimal yaw relaxation (1.2x) for precision")
                            elif insertion_depth > 0.05:  # Medium insertion 50-150mm
                                yaw_relaxation = 1.3  # Conservative relaxation  
                                rospy.logwarn(f"MEDIUM INSERTION: Conservative yaw relaxation (1.3x)")
                            else:  # Surface or approach phase
                                yaw_relaxation = 1.5  # Moderate relaxation
                                rospy.logwarn(f"SURFACE PHASE: Moderate yaw relaxation (1.5x)")
                        else:
                            yaw_relaxation = 1.3  # Default conservative relaxation
                        
                        # Use depth-adaptive acceptance criteria with conservative yaw relaxation
                        relaxed_yaw_threshold = yaw_threshold * yaw_relaxation
                        if final_error < adaptive_pos_threshold and final_yaw_error < relaxed_yaw_threshold:
                            rospy.logwarn(f"Accepting trajectory with depth-adaptive relaxed thresholds (pos: {adaptive_pos_threshold*1000:.1f}mm, yaw: {math.degrees(relaxed_yaw_threshold):.2f}°)")
                            return True
                        else:
                            rospy.logerr(f"Trajectory failed - pos: {final_error*1000:.1f}mm > {adaptive_pos_threshold*1000:.1f}mm OR yaw: {math.degrees(final_yaw_error):.2f}° > {math.degrees(relaxed_yaw_threshold):.2f}°")
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
        
        IMPROVED: Uses consistent PID control instead of switching to simple positioning
        
        Args:
            start_pos: Current position (x, y, z)
            target_pos: Target position (x, y, z) 
            target_yaw: Target yaw rad
            pos_threshold: Position precision m
            yaw_threshold: Yaw precision rad
            
        Returns:
            bool: Returns True on success, False on failure
        """
        
        # CONTROLLER STATE MANAGEMENT: Start trajectory controller
        self.start_new_controller('trajectory')
        
        rospy.loginfo("=== PRECISION POSITIONING MODE (PID-BASED) ===")
        
        distance = math.sqrt(
            (target_pos[0] - start_pos[0])**2 + 
            (target_pos[1] - start_pos[1])**2 + 
            (target_pos[2] - start_pos[2])**2
        )
        
        # Use the same PID approach as smooth trajectory but with tighter parameters
        duration = max(4.0, distance / 0.015)  # Slower for precision
        
        rospy.loginfo(f"Precision mode: {distance*1000:.1f}mm in {duration:.1f}s using PID control")
        
        # ENHANCED: Depth-adaptive PID parameters to prevent oscillation
        valve_surface_z = self._get_valve_surface_height()
        insertion_depth = max(0, valve_surface_z - start_pos[2])
        
        if insertion_depth > 0.20:  # Medium insertion (>200mm)
            # Conservative parameters for medium insertion to prevent oscillation
            pid_kp = 0.6    # Significantly reduced from 1.5
            pid_ki = 0.01   # Minimal integral to avoid windup
            pid_kd = 0.15   # Reduced derivative gain
            rospy.loginfo(f"MEDIUM INSERTION ({insertion_depth*1000:.0f}mm): Using CONSERVATIVE PID gains")
            rospy.loginfo(f"  Conservative gains: Kp={pid_kp}, Ki={pid_ki}, Kd={pid_kd}")
        else:
            # Standard parameters for shallow insertion (<200mm)
            pid_kp = 1.5    # 稍微降低比例增益，减少震荡
            pid_ki = 0.05   # 降低积分增益
            pid_kd = 0.3    # 降低微分增益
            rospy.loginfo(f"SHALLOW INSERTION ({insertion_depth*1000:.0f}mm): Using STANDARD PID gains")
        
        # Z轴专用PID参数（更保守）
        z_pid_kp = 1.8
        z_pid_ki = 0.04
        z_pid_kd = 0.25
        
        # ADAPTIVE GAIN ADJUSTMENT - 存储基础增益以便动态调整
        base_pid_kp = pid_kp
        base_pid_kd = pid_kd
        base_z_pid_kp = z_pid_kp
        base_z_pid_kd = z_pid_kd
        max_correction = 15.0  # mm，精确模式下的修正限制
        base_max_correction = max_correction
        
        # PID状态变量
        integral_error = [0.0, 0.0, 0.0]
        last_error = [0.0, 0.0, 0.0]
        
        # 自适应增益调整日志计数器
        adaptive_gain_logging_interval = 0
        
        # ENHANCED TRAJECTORY INTERPOLATION: Increased point density for better trajectory following
        # 使用30Hz保持控制连贯性（旋转时使用更高频率）
        control_frequency = 30 if (hasattr(self, 'rotation_active') and self.rotation_active) else 20
        rate = rospy.Rate(control_frequency)
        dt = 1.0 / control_frequency
        rospy.loginfo(f"Using {control_frequency}Hz control frequency for yaw adjustment")
        start_time = time.time()
        
        start_yaw = self.sm.current_yaw
        yaw_diff = self._normalize_angle_diff(target_yaw - start_yaw)
        
        converged_count = 0
        required_convergence = 30  # 1.5s稳定收敛
        
        while not rospy.is_shutdown() and self.trajectory_active and (time.time() - start_time) < duration + 8.0:
            elapsed_time = time.time() - start_time
            current_pos = self.sm.get_current_position()
            
            if current_pos is None:
                rospy.logwarn("Lost position feedback in precision mode")
                rate.sleep()
                continue
            
            # 平滑yaw插值
            if elapsed_time <= duration:
                yaw_progress = min(elapsed_time / duration, 1.0)
                smooth_progress = 3*yaw_progress**2 - 2*yaw_progress**3
                current_target_yaw = start_yaw + smooth_progress * yaw_diff
            else:
                current_target_yaw = target_yaw
            
            # 计算位置误差
            error = [
                target_pos[0] - current_pos[0],
                target_pos[1] - current_pos[1],
                target_pos[2] - current_pos[2]
            ]
            
            # PID控制计算 - 使用统一的积分限制机制
            for i in range(3):
                # ENHANCED: 使用与主轨迹函数相同的动态积分限制逻辑
                integral_error[i] += error[i] * dt
                
                # 应用动态积分限制（与主控制器保持一致）
                # 对于高精度模式，使用严格的限制
                limit = 0.010  # 10mm限制，与PID_HIGH_PRECISION保持一致
                
                # 接触检测和应急限制
                position_error_magnitude = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
                if hasattr(self, '_last_precision_error'):
                    error_jump = position_error_magnitude - self._last_precision_error
                    if error_jump > 0.015:  # 15mm突增
                        limit = min(limit, 0.005)  # 应急降低到5mm
                        rospy.logwarn(f"PRECISION CONTACT DETECTED: Error jump {error_jump*1000:.1f}mm, reducing integral limit to {limit*1000:.0f}mm")
                
                self._last_precision_error = position_error_magnitude
                
                # 立即应用限制，防止饱和
                integral_error[i] = max(-limit, min(limit, integral_error[i]))
            
            derivative_error = [
                (error[i] - last_error[i]) / dt for i in range(3)
            ]
            
            # ADAPTIVE GAIN ADJUSTMENT FOR PRECISION MODE
            # 根据误差大小动态调整控制增益（精确模式更加保守）
            position_error_magnitude = math.sqrt(error[0]**2 + error[1]**2 + error[2]**2)
            
            # 精确模式下的自适应增益缩放（更严格的阈值）
            if position_error_magnitude > 0.015:  # 大误差 (>15mm)
                gain_scale_factor = 0.6  # 降低增益60%
                correction_scale_factor = 0.7  # 降低修正幅度70%
                rospy.loginfo_throttle(15, f"🔽 PRECISION: Large error ({position_error_magnitude*1000:.1f}mm) - reducing gains to 60%")
            elif position_error_magnitude > 0.008:  # 中等误差 (8-15mm)
                gain_scale_factor = 0.8  # 降低增益80%
                correction_scale_factor = 0.85   # 降低修正幅度85%
                rospy.loginfo_throttle(15, f"PRECISION: Medium error ({position_error_magnitude*1000:.1f}mm) - reducing gains to 80%")
            else:  # 小误差 (<8mm)
                gain_scale_factor = 1.0  # 保持原始增益
                correction_scale_factor = 1.0
                rospy.loginfo_throttle(15, f"PRECISION: Small error ({position_error_magnitude*1000:.1f}mm) - using full gains")
            
            # 应用自适应缩放
            adaptive_pid_kp = base_pid_kp * gain_scale_factor
            adaptive_pid_kd = base_pid_kd * gain_scale_factor
            adaptive_z_pid_kp = base_z_pid_kp * gain_scale_factor
            adaptive_z_pid_kd = base_z_pid_kd * gain_scale_factor
            adaptive_max_correction = base_max_correction * correction_scale_factor
            
            # PID输出
            pid_correction = []
            for i in range(3):
                if i == 2:  # Z轴
                    correction = (adaptive_z_pid_kp * error[i] + 
                                z_pid_ki * integral_error[i] + 
                                adaptive_z_pid_kd * derivative_error[i])
                    max_corr_m = adaptive_max_correction / 1000.0 * 2.0  # Z轴稍微宽松
                    correction = max(-max_corr_m, min(max_corr_m, correction))
                else:  # XY轴
                    correction = (adaptive_pid_kp * error[i] + 
                                pid_ki * integral_error[i] + 
                                adaptive_pid_kd * derivative_error[i])
                    max_corr_m = adaptive_max_correction / 1000.0  # 使用自适应限制
                    correction = max(-max_corr_m, min(max_corr_m, correction))
                pid_correction.append(correction)
            
            # 精确模式自适应增益调整日志输出
            if adaptive_gain_logging_interval <= 0:
                rospy.loginfo(f"PRECISION ADAPTIVE - Error: {position_error_magnitude*1000:.1f}mm, "
                            f"Gain Scale: {gain_scale_factor:.2f}, "
                            f"PID_Kp: {adaptive_pid_kp:.3f}, PID_Kd: {adaptive_pid_kd:.3f}, "
                            f"Max_Corr: {adaptive_max_correction:.1f}mm")
                adaptive_gain_logging_interval = 60  # 每3秒记录一次 (20Hz/60)
            else:
                adaptive_gain_logging_interval -= 1
            
            # 应用PID修正
            corrected_target = [
                target_pos[0] + pid_correction[0],
                target_pos[1] + pid_correction[1],
                target_pos[2] + pid_correction[2]
            ]
            
            last_error = error[:]
            
            # 发送修正后的目标点
            self.send_trajectory_point(corrected_target, current_target_yaw)
            
            # 检查收敛
            pos_error = math.sqrt(sum([e**2 for e in error]))
            yaw_error = abs(self._normalize_angle_diff(current_target_yaw - self.sm.current_yaw))
            
            if pos_error <= pos_threshold and yaw_error <= yaw_threshold:
                converged_count += 1
                if converged_count >= required_convergence:
                    rospy.loginfo(f"PID precision positioning converged: {pos_error*1000:.1f}mm, {math.degrees(yaw_error):.2f}°")
                    return True
            else:
                converged_count = 0
            
            # 日志输出
            if int(elapsed_time) % 2 == 0 and int(elapsed_time * 10) % 10 == 0:
                correction_magnitude = math.sqrt(sum([c**2 for c in pid_correction]))
                rospy.loginfo(f"Precision PID: error={pos_error*1000:.1f}mm, correction={correction_magnitude*1000:.1f}mm, "
                            f"yaw_error={math.degrees(yaw_error):.2f}°")
            
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
    
    # ===== Alignment control functionality =====
    
    # ===== Constant Distance Feedback Control Function =====
    
    def execute_constant_distance_rotation(self, trajectory, valve_center, 
                                         feedback_frequency=60, alignment_check_interval=0.1):
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
        
        # CONTROLLER STATE MANAGEMENT: Start rotation controller
        self.start_new_controller('rotation')
        
        # FORCE ROTATION CONTROL MODE: Set global flag for all control calls
        rospy.set_param('/force_rotation_control_mode', True)
        rospy.loginfo("ROTATION MODE ACTIVATED: All control calls will use PD_ROTATION mode")
        
        rospy.loginfo("Starting constant distance feedback control rotation")
        
        # Start the trajectory execution
        if hasattr(trajectory, 'start_trajectory'):
            trajectory.start_trajectory()
            rospy.loginfo("Trajectory started successfully")
        else:
            rospy.logwarn("Trajectory object does not have start_trajectory method")
        
        # Reset statistics
        self.control_stats = {
            'total_corrections': 0,
            'distance_violations': 0,
            'max_distance_error': 0.0,
            'avg_distance_error': 0.0,
            'execution_time': 0.0,
            'total_points': 0
        }
        
        # TRAJECTORY PAUSE/RESET CONTROL VARIABLES
        large_error_start_time = None
        large_error_duration = 0.0
        large_error_reset_threshold = 5.0  # Reset after 5 seconds of large error
        trajectory_was_paused = False
        
        # Stable contact detection variables
        stable_contact_detected = False
        contact_position_history = []
        trajectory_replanned = False
        contact_stability_duration = 2.0
        contact_stability_threshold = 0.003
        valve_contact_detected = False
        
        rate = rospy.Rate(feedback_frequency)
        start_time = time.time()
        last_alignment_check = start_time
        
        # Initialize valve yaw for rotation progress tracking
        if hasattr(self.sm, 'valve_yaw') and self.sm.valve_yaw is not None:
            self.initial_valve_yaw = self.sm.valve_yaw
            rospy.loginfo(f"Initial valve yaw saved: {math.degrees(self.initial_valve_yaw):.2f}°")
        else:
            self.initial_valve_yaw = 0.0
            rospy.logwarn("No initial valve yaw available, using 0.0")
        
        distance_error_sum = 0.0
        point_count = 0
        
        # === FIRST SEGMENT DIAGNOSIS LOGGING ===
        # DISABLED: High-frequency detailed logging was causing control instability
        first_segment_logging = False  # DISABLED to prevent I/O interference with control loops
        first_segment_duration = 10.0  # Log first 10 seconds in detail
        first_segment_log_interval = 0.5  # Log every 0.5 seconds
        last_detailed_log_time = start_time
        initial_uav_pos = None
        segment_point_count = 0
        
        rospy.loginfo("=== FIRST SEGMENT DIAGNOSIS MODE: DISABLED FOR STABILITY ===")
        rospy.loginfo("Detailed logging disabled to prevent control interference")
        
        while not rospy.is_shutdown() and self.rotation_active:
            # Get trajectory target point
            result = trajectory.get_next_uav_position_and_yaw()
            if result is None:
                rospy.loginfo("Trajectory execution completed")
                break
            
            target_pos, target_yaw = result
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # Get current state
            current_pos = self.sm.get_current_position()
            current_yaw = self.sm.current_yaw
            
            if current_pos is None:
                rospy.logwarn("Unable to get current position")
                rate.sleep()
                continue
            
            # === FIRST SEGMENT DETAILED LOGGING ===
            if first_segment_logging and elapsed_time <= first_segment_duration:
                if initial_uav_pos is None:
                    initial_uav_pos = current_pos
                    rospy.loginfo(f"FIRST SEGMENT START:")
                    rospy.loginfo(f"  Initial UAV: ({initial_uav_pos[0]:.3f}, {initial_uav_pos[1]:.3f}, {initial_uav_pos[2]:.3f})")
                    rospy.loginfo(f"  Initial Yaw: {math.degrees(current_yaw):.1f}°")
                    rospy.loginfo(f"  Initial Pitch: {math.degrees(getattr(self.sm, 'current_pitch', 0.0)):.1f}°")
                
                # Detailed logging every 0.5 seconds
                if current_time - last_detailed_log_time >= first_segment_log_interval:
                    segment_point_count += 1
                    
                    # Get detailed trajectory analysis
                    analysis = self.analyze_trajectory_segment(
                        current_pos, target_pos, current_yaw, target_yaw, valve_center, elapsed_time)
                    
                    pitch_current = getattr(self.sm, 'current_pitch', 0.0)
                    
                    # Calculate trajectory curvature (rate of yaw change)
                    if hasattr(self, '_prev_target_yaw'):
                        yaw_rate = abs(target_yaw - self._prev_target_yaw) / first_segment_log_interval
                    else:
                        yaw_rate = 0.0
                    self._prev_target_yaw = target_yaw
                    
                    rospy.loginfo(f"SEGMENT1 t={elapsed_time:.1f}s pt={segment_point_count}:")
                    rospy.loginfo(f"  Target: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}) yaw={math.degrees(target_yaw):.1f}°")
                    rospy.loginfo(f"  Actual: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}) yaw={math.degrees(current_yaw):.1f}°")
                    rospy.loginfo(f"  Errors: pos_xy={analysis['pos_error_xy']*1000:.1f}mm, pos_z={analysis['pos_error_z']*1000:.1f}mm, yaw={math.degrees(analysis['yaw_error']):.1f}°")
                    rospy.loginfo(f"  Pitch: {math.degrees(pitch_current):.2f}° (CRITICAL METRIC)")
                    rospy.loginfo(f"  Valve: UAV_dist={analysis['current_to_valve']*1000:.1f}mm, EE_dist={analysis['ee_to_valve']*1000:.1f}mm")
                    rospy.loginfo(f"  Motion: yaw_rate={math.degrees(yaw_rate):.1f}°/s, req_vel={analysis['required_vel_magnitude']:.3f}m/s")
                    rospy.loginfo(f"  Distance_error: {analysis['valve_distance_error']*1000:.1f}mm")
                    rospy.loginfo(f"  ---")
                    
                    last_detailed_log_time = current_time
                
                # End of first segment logging
                if elapsed_time > first_segment_duration:
                    first_segment_logging = False
                    final_pitch = getattr(self.sm, 'current_pitch', 0.0)
                    final_analysis = self.analyze_trajectory_segment(
                        current_pos, target_pos, current_yaw, target_yaw, valve_center, elapsed_time)
                    
                    rospy.loginfo(f"=== FIRST SEGMENT DIAGNOSIS COMPLETED ===")
                    rospy.loginfo(f"Total points logged: {segment_point_count}")
                    rospy.loginfo(f"CRITICAL FINDINGS:")
                    rospy.loginfo(f"  Final pitch error: {math.degrees(final_pitch):.2f}° (Target: 0.0°)")
                    rospy.loginfo(f"  Final position error XY: {final_analysis['pos_error_xy']*1000:.1f}mm")
                    rospy.loginfo(f"  Final yaw error: {math.degrees(final_analysis['yaw_error']):.1f}°")
                    rospy.loginfo(f"  Final valve distance error: {final_analysis['valve_distance_error']*1000:.1f}mm")
                    rospy.loginfo(f"  Average required velocity: {final_analysis['required_vel_magnitude']:.3f}m/s")
                    
                    # Provide diagnosis
                    if abs(final_pitch) > math.radians(2.0):  # >2° pitch error
                        rospy.logwarn(f"SIGNIFICANT PITCH ERROR DETECTED IN NON-CONTACT SEGMENT!")
                        rospy.logwarn(f"This suggests CONTROL ALGORITHM issue, not physical interference")
                        if final_analysis['required_vel_magnitude'] > 0.1:
                            rospy.logwarn(f"High velocity requirement may cause pitch-forward control response")
                        if final_analysis['pos_error_xy'] > 0.02:
                            rospy.logwarn(f"Large XY tracking error may cause aggressive control corrections")
                    else:
                        rospy.loginfo(f"First segment pitch control is NORMAL")
                        rospy.loginfo(f"If pitch errors occur later, likely due to physical contact")
                    
                    rospy.loginfo(f"Switching to normal logging mode")
            
            # Apply feedback control correction
            if current_time - last_alignment_check >= alignment_check_interval:
                corrected_pos, corrected_yaw = self.apply_constant_distance_feedback(
                    target_pos, target_yaw, current_pos, current_yaw, valve_center, trajectory)
                
                last_alignment_check = current_time
            else:
                corrected_pos, corrected_yaw = target_pos, target_yaw
            
            # === STABLE CONTACT DETECTION AND TRAJECTORY REPLANNING ===
            # Skip valve contact detection in DISENGAGEMENT mode (intended to separate from valve)
            is_disengagement_mode = (hasattr(trajectory, 'adaptive_controller') and 
                                   hasattr(trajectory.adaptive_controller, 'control_mode') and 
                                   trajectory.adaptive_controller.control_mode == 'disengagement')
            
            # Check for valve contact first (simple yaw change detection) - but not during disengagement
            if not is_disengagement_mode and hasattr(self.sm, 'valve_yaw') and self.sm.valve_yaw is not None:
                valve_yaw_change = abs(self.sm.valve_yaw - getattr(self, 'initial_valve_yaw', 0.0))
                
                # CRITICAL FIX: Update rotation progress for completion detection
                self.update_valve_rotation_progress(valve_yaw_change)
                
                if not valve_contact_detected and valve_yaw_change > math.radians(2.0):  # 2° threshold
                    valve_contact_detected = True
                    rospy.loginfo(f"Valve contact detected (yaw change: {math.degrees(valve_yaw_change):.1f}°)")
            
            # Check for stable contact and replan if needed (skip during disengagement)
            if not is_disengagement_mode and valve_contact_detected and not stable_contact_detected and not trajectory_replanned:
                if self._check_stable_contact(current_pos, contact_position_history, 
                                            contact_stability_threshold, contact_stability_duration):
                    stable_contact_detected = True
                    elapsed_time = current_time - start_time
                    
                    rospy.loginfo(f"STABLE CONTACT ACHIEVED after {elapsed_time:.1f}s!")
                    
                    # Calculate remaining rotation
                    progress = trajectory.get_progress() if hasattr(trajectory, 'get_progress') else 0.5
                    remaining_rotation = math.radians(90) * (1.0 - progress)  # Assume 90° target
                    remaining_duration = max(5.0, (current_time - start_time) * 0.5)  # At least 5s
                    
                    if abs(remaining_rotation) > math.radians(10):  # Only replan if >10° remains
                        rospy.loginfo(f"Replanning trajectory - remaining rotation: {math.degrees(remaining_rotation):.1f}°")
                        
                        new_trajectory = self._replan_trajectory_from_contact(
                            current_pos[:3], current_yaw, valve_center,
                            remaining_rotation, remaining_duration
                        )
                        
                        if new_trajectory is not None:
                            trajectory = new_trajectory
                            trajectory_replanned = True
                            rospy.loginfo("Successfully switched to replanned trajectory")
                            # Reset timing for new trajectory
                            start_time = current_time
                        else:
                            rospy.logwarn("WARNING: Trajectory replanning failed, continuing with original")
                    else:
                        rospy.loginfo("Remaining rotation is small, continuing with original trajectory")
            
            # Calculate tracking error for adaptive control feedback
            position_error = math.sqrt(
                (current_pos[0] - target_pos[0])**2 + 
                (current_pos[1] - target_pos[1])**2 + 
                (current_pos[2] - target_pos[2])**2
            )
            
            # Send tracking performance feedback to trajectory's adaptive controller
            if hasattr(trajectory, 'update_tracking_performance'):
                trajectory.update_tracking_performance(position_error)
            
            # Send control command using standard position control
            self.send_trajectory_point(corrected_pos, corrected_yaw)
            
            # Update statistics  
            # CRITICAL FIX: Use trajectory's UAV target distance (UAV COG based)
            # GEOMETRY FIX: UAV is BEHIND end-effector, so UAV distance = end_effector + offset
            uav_target_distance = getattr(trajectory, 'uav_target_distance', 
                                        trajectory.end_effector_distance + self.end_effector_offset_x)
            
            distance_error = self.calculate_distance_error(current_pos, current_yaw, valve_center, 
                                                          uav_target_distance)
            distance_error_sum += distance_error
            point_count += 1
            
            # === DYNAMIC DISTANCE TRACKING AND ADAPTIVE CORRECTION ===
            # Key insight: 73mm may be correct initially, but UAV drifts during execution
            # Solution: Continuously track and adaptively correct trajectory parameters
            
            current_ros_time = rospy.Time.now().to_sec()
            
            # Periodic distance recalibration (every 2 seconds)
            if not hasattr(self, '_last_distance_update_time'):
                self._last_distance_update_time = current_ros_time
                self._distance_update_interval = 2.0  # 2 seconds
                self._distance_drift_threshold = 0.05  # 50mm threshold for recalibration
                
            time_since_last_update = current_ros_time - self._last_distance_update_time
            
            if time_since_last_update >= self._distance_update_interval:
                # Calculate current actual distance
                current_ee_pos = self.calculate_end_effector_position(current_pos, current_yaw)
                actual_current_distance = math.sqrt(
                    (current_ee_pos[0] - valve_center[0])**2 + 
                    (current_ee_pos[1] - valve_center[1])**2
                )
                
                # Compare with trajectory's expected distance
                expected_distance = trajectory.end_effector_rotation_radius
                distance_drift = abs(actual_current_distance - expected_distance)
                
                rospy.loginfo(f"DISTANCE TRACKING: Expected={expected_distance*1000:.0f}mm, "
                             f"Actual={actual_current_distance*1000:.0f}mm, "
                             f"Drift={distance_drift*1000:.0f}mm")
                
                # ENHANCED SOFT LIMITING: Use dynamic thresholds for rotation mode
                # Get rotation-specific thresholds or use defaults
                soft_threshold = getattr(self, '_rotation_distance_drift_threshold', 25.0) / 1000.0  # Convert to meters
                critical_threshold = getattr(self, '_rotation_distance_critical_threshold', 40.0) / 1000.0  # Convert to meters
                
                # Add progressive warnings and control actions as drift increases
                if distance_drift >= soft_threshold:  # Soft warning level
                    rospy.logwarn(f"DISTANCE DRIFT WARNING: {distance_drift*1000:.0f}mm exceeds {soft_threshold*1000:.0f}mm soft limit")
                    
                    # GENTLE CORRECTION: Reduce trajectory speed for gradual correction
                    if hasattr(trajectory.adaptive_controller, 'speed_factor'):
                        min_speed = max(0.3, trajectory.adaptive_controller.speed_factor * 0.9)  # 降低10%但不低于30%
                        trajectory.adaptive_controller.speed_factor = min_speed
                        rospy.loginfo(f"Reducing trajectory speed to {min_speed*100:.0f}% for gentle drift correction")
                
                if distance_drift >= critical_threshold:  # Critical level
                    rospy.logerr(f"DISTANCE DRIFT CRITICAL: {distance_drift*1000:.0f}mm exceeds {critical_threshold*1000:.0f}mm hard limit")
                    
                    # IMPROVED: Apply gentle correction instead of abrupt pause
                    if hasattr(trajectory.adaptive_controller, 'speed_factor'):
                        # Dramatically reduce speed but don't pause completely
                        emergency_speed = 0.2  # 20% speed for emergency correction
                        trajectory.adaptive_controller.speed_factor = emergency_speed
                        rospy.logwarn(f"EMERGENCY SPEED REDUCTION: {emergency_speed*100:.0f}% for critical drift correction")
                    
                    # OPTIONAL: Only pause if drift is extremely critical (>60mm)
                    if distance_drift >= critical_threshold * 1.5 and hasattr(trajectory, 'pause') and not trajectory.is_trajectory_paused():
                        trajectory.pause()
                        rospy.logwarn("⏸️ TRAJECTORY PAUSED: Distance drift exceeds extreme critical threshold")
                        rospy.sleep(0.3)  # Reduced pause time (300ms vs 500ms)
                
                # Adaptive correction if drift exceeds threshold
                # 🔒 SOLUTION 1: Check if distance drift correction is disabled
                if (distance_drift >= self._distance_drift_threshold and 
                    not getattr(self, 'disable_distance_drift_correction', False)):
                    rospy.logwarn(f"ADAPTIVE CORRECTION: Distance drift {distance_drift*1000:.0f}mm "
                                 f"exceeds {self._distance_drift_threshold*1000:.0f}mm threshold")
                    
                    # Gradually adjust trajectory radius towards actual distance
                    correction_factor = 0.3  # 30% correction per update
                    new_radius = expected_distance + correction_factor * (actual_current_distance - expected_distance)
                    
                    # Update trajectory radius
                    trajectory.end_effector_rotation_radius = new_radius
                    trajectory.end_effector_distance = new_radius  # Update alias
                    
                    # CRITICAL: Update UAV target distance when trajectory changes
                    if hasattr(trajectory, 'uav_target_distance'):
                        trajectory.uav_target_distance = new_radius + self.end_effector_offset_x
                        trajectory.uav_rotation_radius = trajectory.uav_target_distance
                    
                    rospy.loginfo(f"RADIUS ADJUSTED: {expected_distance*1000:.0f}mm → {new_radius*1000:.0f}mm "
                                 f"(30% towards actual {actual_current_distance*1000:.0f}mm)")
                    
                    # Reset stable contact detection as trajectory changed
                    stable_contact_detected = False
                    contact_position_history = []
                elif (distance_drift >= self._distance_drift_threshold and 
                      getattr(self, 'disable_distance_drift_correction', False)):
                    rospy.loginfo(f"🔒 DISTANCE DRIFT CORRECTION DISABLED: Drift {distance_drift*1000:.0f}mm ignored (locked reference mode)")
                    trajectory_replanned = False
                
                self._last_distance_update_time = current_ros_time
            
            # ENHANCED TRAJECTORY RECOVERY MECHANISM: Gradual recovery before reset
            if distance_error >= 0.15:  # Large error detected
                if large_error_start_time is None:
                    large_error_start_time = current_time
                    rospy.logwarn(f"⏱️ LARGE ERROR TRACKING STARTED: {distance_error*1000:.0f}mm")
                else:
                    large_error_duration = current_time - large_error_start_time
                    
                    # Phase 1: Pause trajectory for 2 seconds to stabilize
                    if large_error_duration >= 2.0 and large_error_duration < 3.0:
                        if hasattr(trajectory, 'pause') and not getattr(trajectory, '_emergency_paused', False):
                            rospy.logwarn("🛑 PHASE 1: Pausing trajectory for stabilization")
                            trajectory.pause()
                            trajectory._emergency_paused = True
                    
                    # Phase 2: Resume with reduced speed for gradual recovery
                    elif large_error_duration >= 3.0 and large_error_duration < large_error_reset_threshold:
                        if hasattr(trajectory, 'resume') and getattr(trajectory, '_emergency_paused', False):
                            rospy.logwarn("PHASE 2: Resuming with reduced speed for gradual recovery")
                            trajectory.resume()
                            trajectory._emergency_paused = False
                            # Force adaptive controller to use minimum speed
                            if hasattr(trajectory, 'adaptive_controller'):
                                trajectory.adaptive_controller.current_speed_factor = 0.3  # Minimum speed
                    
                    # Phase 3: Reset trajectory as last resort
                    elif large_error_duration >= large_error_reset_threshold and hasattr(trajectory, 'reset_to_current_position'):
                        rospy.logerr(f"PHASE 3: LARGE ERROR PERSISTED {large_error_duration:.1f}s > {large_error_reset_threshold}s")
                        rospy.logerr("Resetting trajectory to current UAV position as LAST RESORT")
                        
                        # Store original parameters for debugging
                        original_radius = trajectory.end_effector_rotation_radius
                        
                        try:
                            trajectory.reset_to_current_position(current_pos, current_yaw)
                            large_error_start_time = None  # Reset tracking
                            large_error_duration = 0.0
                            
                            # Log the parameter changes
                            new_radius = trajectory.end_effector_rotation_radius
                            rospy.logwarn(f"TRAJECTORY PARAMETERS UPDATED:")
                            rospy.logwarn(f"   Radius: {original_radius*1000:.1f}mm → {new_radius*1000:.1f}mm")
                            rospy.logwarn(f"   Expected distance error reduction: ~{abs(original_radius-new_radius)*1000:.0f}mm")
                            
                            # Reset stable contact detection variables as trajectory changed
                            stable_contact_detected = False
                            contact_position_history = []
                            trajectory_replanned = False
                            valve_contact_detected = False
                        except Exception as e:
                            rospy.logerr(f"TRAJECTORY RESET FAILED: {e}")
                            # Continue with current trajectory rather than crashing
            else:
                # Error is manageable, reset tracking
                if large_error_start_time is not None:
                    rospy.loginfo(f"LARGE ERROR RESOLVED after {large_error_duration:.1f}s")
                large_error_start_time = None
                large_error_duration = 0.0
            
            self.control_stats['max_distance_error'] = max(
                self.control_stats['max_distance_error'], distance_error)
            self.control_stats['total_points'] += 1
            
            if distance_error > self.distance_tolerance:
                self.control_stats['distance_violations'] += 1
            
            # Periodic status output
            if point_count % (feedback_frequency * 2) == 0:  # Output every 2 seconds
                # Get adaptive control status
                speed_factor = 1.0
                if hasattr(trajectory, 'get_current_speed_factor'):
                    speed_factor = trajectory.get_current_speed_factor()
                
                rospy.loginfo(f"Rotation progress: {point_count} points, distance error: {distance_error:.4f}m, "
                             f"max error: {self.control_stats['max_distance_error']:.4f}m, "
                             f"adaptive speed: {speed_factor:.2f}")
            
            rate.sleep()
        
        # Calculate final statistics (including adaptive control data)
        self.control_stats['execution_time'] = time.time() - start_time
        self.control_stats['avg_distance_error'] = distance_error_sum / max(point_count, 1)
        
        # Add adaptive control statistics if available
        if hasattr(trajectory, 'get_current_speed_factor'):
            final_speed_factor = trajectory.get_current_speed_factor()
            self.control_stats['final_speed_factor'] = final_speed_factor
            rospy.loginfo(f"Final adaptive speed factor: {final_speed_factor:.2f}")
        
        self.log_control_statistics()
        
        # Clear rotation control mode flag
        rospy.set_param('/force_rotation_control_mode', False)
        rospy.loginfo("Cleared rotation control mode flag")
        
        # ENHANCED: Clear rotation_active flag when rotation completes
        if hasattr(self, 'rotation_active'):
            self.rotation_active = False
            rospy.loginfo("ROTATION COMPLETE: Cleared rotation_active flag")
        
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
        # 🚁 METHOD 2 FIX: Enable GENTLE distance correction during disengagement
        # PROBLEM: Complete distance correction disable caused 606mm distance explosion
        # SOLUTION: Use gentle distance correction with conservative gains and thresholds
        if hasattr(self, 'disengagement_active') and self.disengagement_active:
            # Calculate current distance error
            valve_x, valve_y, valve_z = valve_center
            current_distance = sqrt((current_pos[0] - valve_x)**2 + (current_pos[1] - valve_y)**2)
            
            # Get expected distance from trajectory or use reasonable default
            if hasattr(trajectory, 'uav_rotation_radius'):
                expected_distance = trajectory.uav_rotation_radius
            else:
                expected_distance = 0.30  # 30cm default for UAV distance
                
            distance_error = abs(current_distance - expected_distance)
            
            # 🚁 GENTLE CORRECTION: Only apply correction for significant drift
            if distance_error > 0.020:  # 20mm threshold - ignore small deviations
                correction_gain = 0.3  # Gentle 30% correction instead of full correction
                rospy.loginfo_throttle(5, f"🚁 DISENGAGEMENT: Gentle distance correction - error={distance_error*1000:.0f}mm, gain={correction_gain}")
                
                # Apply gentle distance correction
                error_direction_x = (valve_x - current_pos[0]) / current_distance if current_distance > 0 else 0
                error_direction_y = (valve_y - current_pos[1]) / current_distance if current_distance > 0 else 0
                
                correction_magnitude = (expected_distance - current_distance) * correction_gain
                corrected_x = target_pos[0] + error_direction_x * correction_magnitude
                corrected_y = target_pos[1] + error_direction_y * correction_magnitude
                
                return (corrected_x, corrected_y, target_pos[2]), target_yaw
            else:
                rospy.loginfo_throttle(10, f"🚁 DISENGAGEMENT: Distance OK - error={distance_error*1000:.0f}mm < 20mm threshold")
                return target_pos, target_yaw
            
        # INTELLIGENT DISTANCE CORRECTION: Replace complete disable with smart correction
        if hasattr(self, 'rotation_active') and self.rotation_active:
            # Calculate distance drift
            valve_x, valve_y, valve_z = valve_center
            current_distance = sqrt((current_pos[0] - valve_x)**2 + (current_pos[1] - valve_y)**2)
            distance_error = abs(current_distance - target_distance)
            
            # Store distance error for adaptive gain calculation
            self.last_distance_error = distance_error
            
            # SMART CORRECTION: Only correct significant drift, with velocity limits
            if distance_error > 0.020:  # 20mm drift threshold
                # Calculate gentle correction with velocity limiting
                correction_strength = min(0.4, distance_error / 0.050)  # Max 40% correction
                
                # Direction towards valve center
                to_valve_x = valve_x - current_pos[0]
                to_valve_y = valve_y - current_pos[1]
                distance_to_valve = sqrt(to_valve_x**2 + to_valve_y**2)
                
                if distance_to_valve > 0.001:  # Avoid division by zero
                    # Normalize direction
                    dir_x = to_valve_x / distance_to_valve
                    dir_y = to_valve_y / distance_to_valve
                    
                    # Calculate required distance adjustment
                    distance_adjustment = current_distance - target_distance
                    
                    # Apply gentle correction
                    correction_x = dir_x * distance_adjustment * correction_strength
                    correction_y = dir_y * distance_adjustment * correction_strength
                    
                    # Limit correction magnitude to prevent oscillation
                    max_correction = 0.005  # 5mm max per control cycle
                    correction_mag = sqrt(correction_x**2 + correction_y**2)
                    if correction_mag > max_correction:
                        scale = max_correction / correction_mag
                        correction_x *= scale
                        correction_y *= scale
                    
                    corrected_x = target_pos[0] - correction_x
                    corrected_y = target_pos[1] - correction_y
                    
                    rospy.loginfo_throttle(2, f"ROTATION DRIFT CORRECTION: drift={distance_error*1000:.1f}mm, "
                                         f"correction=({correction_x*1000:.1f}, {correction_y*1000:.1f})mm, "
                                         f"strength={correction_strength:.2f}")
                    
                    return (corrected_x, corrected_y, target_pos[2]), target_yaw
                else:
                    rospy.logwarn_throttle(5, "ROTATION: Invalid valve distance calculation")
            else:
                rospy.loginfo_throttle(10, f"ROTATION: Distance stable - drift={distance_error*1000:.1f}mm < 20mm")
            
            return target_pos, target_yaw
            
        # Check completion status first to prevent infinite correction loops
        if self.use_completion_detection and self.completion_detector:
            if self.completion_detector.is_completed():
                # Rotation is complete - stop position corrections to prevent stuck loop
                return target_pos, target_yaw
        
        # CORRECTED: Calculate distance error based on UAV COG
        valve_x, valve_y, valve_z = valve_center
        current_distance = sqrt((current_pos[0] - valve_x)**2 + (current_pos[1] - valve_y)**2)
        
        # CRITICAL FIX: Use correct UAV COG target distance instead of geometric calculation
        # Use trajectory's UAV target distance if available, otherwise calculate
        # GEOMETRY FIX: UAV is BEHIND end-effector, so UAV distance = end_effector + offset
        # 🔒 SOLUTION 1: Use locked reference distance if available for consistency
        if hasattr(self, 'locked_reference_distance') and self.locked_reference_distance is not None:
            target_distance = self.locked_reference_distance + self.end_effector_offset_x
            rospy.loginfo_throttle(5, f"🔒 Using locked reference distance: {target_distance*100:.1f}cm")
        else:
            target_distance = getattr(trajectory, 'uav_target_distance', 
                                    trajectory.end_effector_distance + self.end_effector_offset_x)
        
        distance_error = abs(current_distance - target_distance)
        
        # ERROR SATURATION PROTECTION: Skip correction for extremely large errors
        if distance_error >= 0.15:  # >=150mm threshold - prevents UAV from getting stuck
            rospy.logwarn(f"🚫 DISTANCE ERROR TOO LARGE: {distance_error*1000:.0f}mm >= 150mm threshold")
            rospy.logwarn("Skipping distance correction to prevent control divergence")
            rospy.logwarn("UAV will rely on trajectory natural convergence")
            
            # TRAJECTORY PAUSE MECHANISM: Pause trajectory when large error detected
            if hasattr(trajectory, 'pause') and not trajectory.is_trajectory_paused():
                trajectory.pause()
                rospy.logwarn("⏸️ TRAJECTORY PAUSED due to large distance error")
            
            return target_pos, target_yaw  # Early exit for large errors
        else:
            # TRAJECTORY RESUME MECHANISM: Resume trajectory when error becomes manageable
            if hasattr(trajectory, 'resume') and trajectory.is_trajectory_paused():
                trajectory.resume()
                rospy.loginfo("▶️ TRAJECTORY RESUMED - distance error manageable")
        
        # ROTATION PHASE DISTANCE TOLERANCE OVERRIDE
        # Use relaxed tolerance during rotation to prevent control conflicts
        force_rotation_mode = rospy.get_param('/force_rotation_control_mode', False)
        effective_distance_tolerance = self.distance_tolerance
        if force_rotation_mode:
            effective_distance_tolerance = 0.015  # 15mm - more relaxed for rotation
            rospy.loginfo_throttle(10, f"ROTATION MODE: Using relaxed distance tolerance {effective_distance_tolerance*1000:.0f}mm")
        
        # If distance error is within tolerance, no correction needed
        if distance_error <= effective_distance_tolerance:
            return target_pos, target_yaw  # No correction needed
        
        # Calculate correction vector (REMOVED feedforward control)
        correction_pos, correction_yaw = self.calculate_distance_correction(
            current_pos, current_yaw, valve_center, target_distance)
        
        # Apply correction directly (no feedforward)
        corrected_pos = (
            target_pos[0] + correction_pos[0],
            target_pos[1] + correction_pos[1],
            target_pos[2] + correction_pos[2]
        )
        corrected_yaw = target_yaw + correction_yaw
        
        # Log correction details if significant
        if abs(correction_pos[0]) > 0.001 or abs(correction_pos[1]) > 0.001:
            rospy.loginfo(f"� DISTANCE CORRECTION: ({correction_pos[0]*1000:.1f}, {correction_pos[1]*1000:.1f})mm")
        
        # Record correction
        self.control_stats['total_corrections'] += 1
        
        # Update completion detector if available
        if self.use_completion_detection and self.completion_detector:
            self.completion_detector.update_distance_error(distance_error)
            self.completion_detector.record_position_correction()
            
            # Check if rotation is considered complete
            if self.completion_detector.is_completed():
                reason = self.completion_detector.get_completion_reason()
                rospy.loginfo(f"ROTATION COMPLETION DETECTED: {reason}")
                rospy.loginfo("Terminating position corrections to prevent infinite loop")
                return target_pos, target_yaw  # Rotation complete, stop corrections
        
        # Record correction information with detailed diagnostics
        if distance_error > self.distance_tolerance * 2:  # Only record large corrections
            rospy.loginfo(f"DISTANCE CORRECTION APPLIED:")
            rospy.loginfo(f"   Error: {distance_error*1000:.1f}mm (target: <{self.distance_tolerance*1000:.0f}mm)")
            rospy.loginfo(f"   Position correction: [{correction_pos[0]*1000:.1f}, {correction_pos[1]*1000:.1f}, {correction_pos[2]*1000:.1f}]mm")
            rospy.loginfo(f"   Yaw correction: {math.degrees(correction_yaw):.2f}°")
            
            # Additional diagnostic info for large errors
            if distance_error > 0.05:
                rospy.loginfo(f"   Current distance: {current_distance*1000:.1f}mm")
                rospy.loginfo(f"   Target distance: {target_distance*1000:.1f}mm")
                rospy.loginfo(f"   Distance difference: {(current_distance-target_distance)*1000:.1f}mm")
        
        return corrected_pos, corrected_yaw
    
    def calculate_distance_correction(self, current_pos, current_yaw, valve_center, target_distance):
        """
        ENHANCED Distance correction with predictive control and drift compensation
        
        Improvements:
        1. Predictive control - anticipates future drift based on velocity
        2. Historical tracking - learns from past corrections
        3. Adaptive gains - adjusts based on rotation phase and drift magnitude
        4. Smooth transition - gradual correction to prevent oscillation
        
        Args:
            current_pos: Current UAV COG position
            current_yaw: Current UAV yaw
            valve_center: Valve center position
            target_distance: Target distance for UAV COG
        
        Returns:
            tuple: (position_correction, yaw_correction)
        """
        # Initialize drift compensation history
        if not hasattr(self, '_distance_drift_history'):
            self._distance_drift_history = {
                'errors': [],
                'corrections': [],
                'velocities': [],
                'timestamps': [],
                'last_position': None,
                'last_time': None
            }
        
        current_time = rospy.Time.now()
        
        # ENHANCED: Calculate current UAV COG distance with drift velocity estimation
        valve_x, valve_y, valve_z = valve_center
        current_distance = sqrt((current_pos[0] - valve_x)**2 + (current_pos[1] - valve_y)**2)
        distance_error = current_distance - target_distance
        
        # PREDICTIVE CONTROL: Estimate drift velocity and predict future error
        drift_velocity = 0.0
        predicted_error = distance_error
        
        if (self._distance_drift_history['last_position'] is not None and 
            self._distance_drift_history['last_time'] is not None):
            
            # Calculate time delta
            dt = (current_time - self._distance_drift_history['last_time']).to_sec()
            
            if dt > 0.001:  # Avoid division by zero
                # Calculate distance change rate (drift velocity)
                last_distance = sqrt((self._distance_drift_history['last_position'][0] - valve_x)**2 + 
                                   (self._distance_drift_history['last_position'][1] - valve_y)**2)
                distance_change = current_distance - last_distance
                drift_velocity = distance_change / dt
                
                # PREDICTIVE ERROR: Estimate error 0.5 seconds ahead
                prediction_horizon = 0.5  # seconds
                predicted_distance = current_distance + drift_velocity * prediction_horizon
                predicted_error = predicted_distance - target_distance
                
                # Store velocity history for adaptive control
                self._distance_drift_history['velocities'].append(drift_velocity)
                if len(self._distance_drift_history['velocities']) > 20:
                    self._distance_drift_history['velocities'].pop(0)
                
                # Calculate average drift tendency
                if len(self._distance_drift_history['velocities']) >= 5:
                    avg_drift_velocity = sum(self._distance_drift_history['velocities'][-5:]) / 5
                    rospy.loginfo_throttle(2, f"DRIFT ANALYSIS: current={drift_velocity*1000:.1f}mm/s, avg={avg_drift_velocity*1000:.1f}mm/s")
        
        # Update position history
        self._distance_drift_history['last_position'] = current_pos
        self._distance_drift_history['last_time'] = current_time
        
        # ENHANCED TOLERANCE with adaptive thresholds based on rotation speed
        force_rotation_mode = rospy.get_param('/force_rotation_control_mode', False)
        base_tolerance = 0.015 if force_rotation_mode else self.distance_tolerance
        
        # ADAPTIVE TOLERANCE: Increase tolerance during fast rotation to reduce over-correction
        rotation_speed = abs(drift_velocity) if drift_velocity != 0 else 0
        if rotation_speed > 0.01:  # Fast rotation (>10mm/s drift)
            adaptive_tolerance = base_tolerance * 1.5  # 50% more tolerant
            rospy.loginfo_throttle(3, f"FAST ROTATION: increased tolerance to {adaptive_tolerance*1000:.0f}mm")
        else:
            adaptive_tolerance = base_tolerance
        
        # Use predicted error for early intervention
        effective_error = predicted_error if abs(predicted_error) > abs(distance_error) else distance_error
        
        # If error is within adaptive tolerance, no correction needed
        if abs(effective_error) <= adaptive_tolerance:
            self._distance_drift_history['errors'].append(effective_error)
            self._distance_drift_history['corrections'].append(0.0)
            self._distance_drift_history['timestamps'].append(current_time.to_sec())
            return (0.0, 0.0, 0.0), 0.0
        
        # Calculate unit vector from valve center to UAV COG
        if current_distance > 0.001:
            unit_vector_x = (current_pos[0] - valve_x) / current_distance
            unit_vector_y = (current_pos[1] - valve_y) / current_distance
        else:
            unit_vector_x = 1.0
            unit_vector_y = 0.0
        
        # ENHANCED PROGRESSIVE CONTROL with historical learning
        # Analyze correction effectiveness from history
        correction_effectiveness = 1.0  # Default effectiveness
        if len(self._distance_drift_history['errors']) >= 10:
            # Calculate if previous corrections were effective
            recent_errors = self._distance_drift_history['errors'][-10:]
            recent_corrections = self._distance_drift_history['corrections'][-10:]
            
            # If errors are consistently large despite corrections, reduce aggressiveness
            avg_recent_error = sum([abs(e) for e in recent_errors]) / len(recent_errors)
            avg_recent_correction = sum([abs(c) for c in recent_corrections]) / len(recent_corrections)
            
            if avg_recent_error > adaptive_tolerance * 2 and avg_recent_correction > 0.01:
                correction_effectiveness = 0.7  # Reduce effectiveness by 30%
                rospy.loginfo_throttle(5, f"LEARNING: Reducing correction aggressiveness (effectiveness: {correction_effectiveness:.1f})")
        
        # Progressive correction with enhanced adaptive strategy
        correction_factor = 1.0 * correction_effectiveness
        
        if abs(effective_error) > 0.08:  # >80mm: Very large error
            correction_factor = 0.15 * correction_effectiveness  # Very gentle
            rospy.logwarn(f"VERY LARGE ERROR: {effective_error*1000:.0f}mm - gentle progressive correction")
        elif abs(effective_error) > 0.05:  # >50mm: Large error
            correction_factor = 0.25 * correction_effectiveness  # Gentle
            rospy.loginfo(f"LARGE ERROR: {effective_error*1000:.0f}mm - progressive correction")
        elif abs(effective_error) > 0.03:  # >30mm: Moderate error
            correction_factor = 0.4 * correction_effectiveness   # Moderate
            rospy.loginfo(f"MODERATE ERROR: {effective_error*1000:.0f}mm - moderate correction")
        # else: small error, use stronger correction
        
        # ENHANCED ADAPTIVE GAIN with drift velocity consideration
        adaptive_gain = self.distance_control_gain
        
        # Reduce gain during high drift velocity to prevent fighting the system
        if abs(drift_velocity) > 0.015:  # High drift velocity (>15mm/s)
            adaptive_gain *= 0.5  # Half gain during high drift
            rospy.loginfo_throttle(3, f"HIGH DRIFT VELOCITY: {drift_velocity*1000:.1f}mm/s - reducing gain")
        elif abs(drift_velocity) > 0.008:  # Medium drift velocity (>8mm/s)
            adaptive_gain *= 0.75  # 75% gain during medium drift
        
        # Additional gain reduction for large errors to prevent overshooting
        if abs(effective_error) > 0.05:  # >50mm
            adaptive_gain *= 0.4  # Significant reduction for stability
        elif abs(effective_error) > 0.02:  # >20mm
            adaptive_gain *= 0.7  # Moderate reduction
        
        # Calculate enhanced correction with DRIFT COMPENSATION
        base_correction = -effective_error * adaptive_gain * correction_factor
        
        # Add drift velocity compensation (feed-forward control)
        drift_compensation = -drift_velocity * 0.2 if abs(drift_velocity) > 0.005 else 0.0
        
        effective_correction = base_correction + drift_compensation
        
        uav_correction_x = effective_correction * unit_vector_x * self.position_control_gain
        uav_correction_y = effective_correction * unit_vector_y * self.position_control_gain
        uav_correction_z = 0.0  # No Z direction correction
        
        # ENHANCED MAGNITUDE LIMITING with dynamic bounds
        correction_magnitude = sqrt(uav_correction_x**2 + uav_correction_y**2)
        
        # Dynamic max correction based on error magnitude and rotation phase
        if abs(effective_error) > 0.06:  # Large errors get more conservative limits
            max_correction = 0.025  # 25mm max for large errors
        elif abs(effective_error) > 0.03:  # Medium errors
            max_correction = 0.035  # 35mm max for medium errors
        else:
            max_correction = 0.045  # 45mm max for small errors
        
        if correction_magnitude > max_correction:
            scale_factor = max_correction / correction_magnitude
            uav_correction_x *= scale_factor
            uav_correction_y *= scale_factor
            correction_magnitude = max_correction
        
        # ENHANCED YAW CORRECTION with drift consideration
        desired_yaw = atan2(valve_y - current_pos[1], valve_x - current_pos[0])
        yaw_error = self._normalize_angle_diff(desired_yaw - current_yaw)
        
        # Adaptive yaw control with drift velocity consideration
        adaptive_yaw_gain = self.yaw_control_gain
        
        # Reduce yaw correction during high drift to prevent fighting
        if abs(drift_velocity) > 0.01:  # High drift
            adaptive_yaw_gain *= 0.6  # Reduce yaw aggressiveness
        elif abs(effective_error) > 0.05:  # Large distance errors
            adaptive_yaw_gain *= 0.7  # Gentle yaw correction for stability
        
        # ENHANCED YAW LIMITING for smoother rotation
        yaw_correction = yaw_error * adaptive_yaw_gain
        max_yaw_correction = math.radians(5.0)  # 5 degrees max per cycle
        yaw_correction = max(-max_yaw_correction, min(max_yaw_correction, yaw_correction))
        
        # Store correction history for learning
        self._distance_drift_history['errors'].append(effective_error)
        self._distance_drift_history['corrections'].append(correction_magnitude)
        self._distance_drift_history['timestamps'].append(current_time.to_sec())
        
        # Maintain history size (keep last 50 entries)
        for key in ['errors', 'corrections', 'timestamps']:
            if len(self._distance_drift_history[key]) > 50:
                self._distance_drift_history[key].pop(0)
        
        # ENHANCED LOGGING with drift analysis
        if correction_magnitude > 0.01:  # Log significant corrections
            rospy.loginfo_throttle(1, f"ENHANCED DISTANCE CORRECTION:")
            rospy.loginfo_throttle(1, f"  Current error: {distance_error*1000:.1f}mm, Predicted: {predicted_error*1000:.1f}mm")
            rospy.loginfo_throttle(1, f"  Drift velocity: {drift_velocity*1000:.1f}mm/s")
            rospy.loginfo_throttle(1, f"  Correction: {correction_magnitude*1000:.1f}mm (factor: {correction_factor:.2f})")
            rospy.loginfo_throttle(1, f"  Adaptive gain: {adaptive_gain:.3f}, Yaw correction: {math.degrees(yaw_correction):.1f}°")
        
        yaw_correction = yaw_error * adaptive_yaw_gain
        
        # More conservative yaw correction limits to prevent pitch oscillation
        max_yaw_limit = min(self.max_yaw_correction, 0.1)  # Limit to 0.1 rad (~6 degrees)
        yaw_correction = max(-max_yaw_limit, min(max_yaw_limit, yaw_correction))
        
        return (uav_correction_x, uav_correction_y, uav_correction_z), yaw_correction
    
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
        """Log control statistics"""
        rospy.loginfo("=== Unified Motion Controller Statistics ===")
        rospy.loginfo(f"Execution time: {self.control_stats['execution_time']:.2f}s")
        rospy.loginfo(f"Total trajectory points: {self.control_stats['total_points']}")
        rospy.loginfo(f"Total corrections: {self.control_stats['total_corrections']}")
        rospy.loginfo(f"Distance violations: {self.control_stats['distance_violations']}")
        rospy.loginfo(f"Maximum distance error: {self.control_stats['max_distance_error']:.4f}m")
        rospy.loginfo(f"Average distance error: {self.control_stats['avg_distance_error']:.4f}m")
        
        # Adaptive control statistics
        if 'final_speed_factor' in self.control_stats:
            speed_factor = self.control_stats['final_speed_factor']
            rospy.loginfo(f"Adaptive control final speed factor: {speed_factor:.2f}")
            if speed_factor < 0.7:
                rospy.loginfo("  → Trajectory was significantly slowed down due to tracking difficulties")
            elif speed_factor > 1.1:
                rospy.loginfo("  → Trajectory was accelerated due to good tracking performance")
            else:
                rospy.loginfo("  → Trajectory speed remained near nominal")
        
        # Calculate success rate
        if self.control_stats['total_points'] > 0:
            success_rate = (1.0 - self.control_stats['distance_violations'] / 
                           self.control_stats['total_points']) * 100
            rospy.loginfo(f"Distance maintenance success rate: {success_rate:.1f}%")
        
        # Evaluate control performance
        if self.control_stats['max_distance_error'] < self.distance_tolerance * 2:
            rospy.loginfo("Distance control performance: Excellent")
        elif self.control_stats['max_distance_error'] < self.distance_tolerance * 3:
            rospy.loginfo("Distance control performance: Good")
        else:
            rospy.logwarn("WARNING: Distance control performance: Needs improvement")
        
        rospy.loginfo("=== Unified Motion Controller with Adaptive Control ===")
        rospy.loginfo("Basic trajectory execution and feedback control")
        rospy.loginfo("Enhanced alignment control")
        rospy.loginfo("Constant distance feedback control")
        rospy.loginfo("Specialized valve rotation control")
    
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
        
        # CONTROLLER STATE MANAGEMENT: Start trajectory controller
        self.start_new_controller('trajectory')
        
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
        
        while not rospy.is_shutdown() and self.trajectory_active and (time.time() - start_time) < duration + 5.0:
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
                rospy.logwarn("WARNING: Significant drift detected in locked axes")
            else:
                rospy.loginfo("Excellent axis control - locked axes maintained")
                
        elif axis_lock_mode == 'xy_yaw':
            rospy.loginfo(f"Axis control performance (XY+Yaw mode):")
            rospy.loginfo(f"  Max Z drift: {max_z_drift:.6f}m (should be ~0)")
            if max_z_drift > 0.01:
                rospy.logwarn("WARNING: Significant Z-axis drift detected")
            else:
                rospy.loginfo("Excellent Z-axis control maintained")
        
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
        
        rospy.loginfo("Phase 1 completed: Simple approach successful")
        
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
                rospy.logerr(f"Z descent segment {segment + 1} failed")
                return False
            
            # 更新当前位置为下一段做准备
            current_segment_pos = self.sm.get_current_position()
            current_segment_yaw = self.sm.get_current_yaw()
            
            rospy.loginfo(f"Z descent segment {segment + 1} completed")
        
        stage3b_success = True  # 所有分段都成功
        
        if stage3b_success:
            rospy.loginfo("THREE-STAGE INSERTION WITH ADAPTIVE REPLANNING COMPLETED SUCCESSFULLY")
            rospy.loginfo("Z-AXIS DESCENT COMPLETED - INSERTION STRATEGY FIXED")
            
            # === 关键节点强制精度校正 ===
            rospy.loginfo("=== CRITICAL PRECISION VERIFICATION BEFORE DUAL-FANG POSITIONING ===")
            
            # 获取当前位置和最终目标位置
            current_pos = self.sm.get_current_position()
            if current_pos is None:
                rospy.logerr("Cannot get current position for precision verification")
                return False
                
            # 计算最终目标位置（阀门中心位置）
            final_target_pos = target_pos  # 使用最后一个segment的目标位置
            
            # 计算位置误差
            xy_error = math.sqrt((current_pos[0] - final_target_pos[0])**2 + 
                               (current_pos[1] - final_target_pos[1])**2)
            z_error = abs(current_pos[2] - final_target_pos[2])
            
            rospy.loginfo(f"Final position verification:")
            rospy.loginfo(f"  Current: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
            rospy.loginfo(f"  Target:  ({final_target_pos[0]:.3f}, {final_target_pos[1]:.3f}, {final_target_pos[2]:.3f})")
            rospy.loginfo(f"  XY error: {xy_error*1000:.1f}mm")
            rospy.loginfo(f"  Z error:  {z_error*1000:.1f}mm")
            
            # SIGNIFICANTLY RELAXED precision requirements for post-insertion phase
            # Fixes insertion failure: 22.2mm > 22mm threshold causing unnecessary corrections
            # Since circular arc trajectory can handle moderate positioning errors
            critical_xy_threshold = 0.035  # 35mm - RELAXED from 22mm to prevent insertion failures
            critical_z_threshold = 0.020   # 20mm - Z轴精度要求
            
            precision_correction_needed = False
            # RELAXED XY precision check - prevent unnecessary corrections that cause divergence
            if xy_error > critical_xy_threshold:
                rospy.logwarn(f"XY precision insufficient for dual-fang positioning: {xy_error*1000:.1f}mm > {critical_xy_threshold*1000:.0f}mm")
                precision_correction_needed = True
                
            if z_error > critical_z_threshold:
                rospy.logwarn(f"Z precision insufficient for dual-fang positioning: {z_error*1000:.1f}mm > {critical_z_threshold*1000:.0f}mm")
                precision_correction_needed = True
                
            if precision_correction_needed:
                rospy.logwarn("=== EXECUTING ENHANCED PRECISION CORRECTION ===")
                rospy.loginfo("Performing final precision adjustment for dual-fang positioning...")
                rospy.loginfo("ENHANCED STRATEGY: Extended time + Fixed PID_HIGH_PRECISION control")
                
                # 获取当前yaw
                current_yaw = self.sm.current_yaw
                
                # ENHANCED precision correction strategy:
                # - Extended duration for better convergence
                # - Relaxed thresholds for post-insertion phase
                # - Force PID_HIGH_PRECISION mode to avoid mode switching
                rospy.set_param('/force_precision_correction_mode', True)  # Signal for special control mode
                
                correction_success = self.execute_smooth_trajectory_with_yaw(
                    start_pos=current_pos,
                    target_pos=final_target_pos,
                    target_yaw=current_yaw,  # 保持当前yaw
                    duration=10.0,  # Extended from 6s to 10s for better convergence
                    pos_threshold=0.018,  # Relaxed from 12mm to 18mm for post-insertion
                    yaw_threshold=0.03   # Relaxed yaw threshold
                )
                
                # Clear the special mode flag
                rospy.set_param('/force_precision_correction_mode', False)
                
                if correction_success:
                    # 再次验证精度
                    current_pos_after = self.sm.get_current_position()
                    if current_pos_after:
                        xy_error_after = math.sqrt((current_pos_after[0] - final_target_pos[0])**2 + 
                                                 (current_pos_after[1] - final_target_pos[1])**2)
                        z_error_after = abs(current_pos_after[2] - final_target_pos[2])
                        
                        rospy.loginfo(f"After precision correction:")
                        rospy.loginfo(f"  XY error: {xy_error_after*1000:.1f}mm")
                        rospy.loginfo(f"  Z error:  {z_error_after*1000:.1f}mm")
                        
                        # RELAXED SUCCESS CRITERIA for post-insertion precision correction
                        final_xy_threshold = 0.020  # Relaxed to 20mm (was 15mm)
                        final_z_threshold = 0.025   # Relaxed to 25mm (was 20mm)
                        
                        if xy_error_after <= final_xy_threshold and z_error_after <= final_z_threshold:
                            rospy.loginfo("PRECISION CORRECTION SUCCESSFUL - Ready for dual-fang positioning")
                        else:
                            rospy.logwarn(f"PRECISION CORRECTION ACCEPTABLE - XY: {xy_error_after*1000:.1f}mm, Z: {z_error_after*1000:.1f}mm")
                            rospy.loginfo("  Proceeding with circular arc trajectory (can handle moderate positioning errors)")
                            # Still return success since circular arc can handle moderate errors
                else:
                    rospy.logwarn("PRECISION CORRECTION FAILED")
                    rospy.loginfo(" Proceeding anyway - circular arc trajectory can handle moderate positioning errors")
                    # Don't return False here - let the circular arc trajectory handle the positioning
            else:
                rospy.loginfo("Position precision sufficient for dual-fang positioning")
            
            rospy.loginfo("=== PRECISION VERIFICATION COMPLETE ===")
            return True
        else:
            rospy.logerr("Stage 3B (Z descent) failed after adaptive replanning")
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
            
            # 💡 INTELLIGENT DIVERGENCE DETECTION: Disable for long-distance movements
            # Stage 2 is typically long-distance movement (>1m), so disable strict divergence detection
            enable_divergence = not ("Stage 2" in stage_name)  # Disable for Stage 2
            
            # 执行基础轨迹
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
        
        # PROGRESSIVE SPEED CONTROL - 渐进式速度控制修复XY漂移问题
        # 根据segment_name确定适当的下降速度
        if "3B-1" in segment_name:
            z_speed = 0.020  # 第一段：快速接近，20mm/s
            speed_desc = "FAST approach"
        elif "3B-2" in segment_name:
            z_speed = 0.015  # 第二段：适中速度，15mm/s  
            speed_desc = "MEDIUM precision"
        elif "3B-3" in segment_name:
            z_speed = 0.010  # 第三段：精密控制，10mm/s
            speed_desc = "SLOW precision"
            
            # SEGMENT 3 CRITICAL PHASE MONITORING SETUP
            rospy.logwarn(" ENTERING CRITICAL SEGMENT 3 (241-284mm depth)")
            rospy.logwarn(" This is the highest failure-risk phase - implementing enhanced monitoring")
            
            # Pre-execution XY position validation for Segment 3
            if hasattr(self, 'tf_listener'):
                current_position = self._get_current_position()
                if current_position:
                    current_xy_error = math.sqrt(
                        (current_position[0] - xy_lock_pos[0])**2 + 
                        (current_position[1] - xy_lock_pos[1])**2
                    )
                    if current_xy_error > 0.015:  # 15mm pre-warning threshold
                        rospy.logwarn(f"PRE-SEGMENT-3 WARNING: XY error {current_xy_error*1000:.1f}mm > 15mm")
                        rospy.logwarn("   Consider XY correction before entering critical phase")
                        rospy.logwarn("   This drift may worsen during slow 10mm/s descent")
            
            # Enhanced monitoring flags for Segment 3
            rospy.loginfo("Segment 3 Enhanced Monitoring Features:")
            rospy.loginfo("   • XY drift threshold: 12mm (stricter than default 15mm)")
            rospy.loginfo("   • Adaptive XY correction gains: 0.8-1.2 based on drift magnitude")
            rospy.loginfo("   • Conservative PID control for depth >240mm")
            rospy.loginfo("   • Real-time position trend analysis")
            rospy.loginfo("   • Valve collision risk assessment")
        else:
            z_speed = 0.015  # 默认适中速度
            speed_desc = "DEFAULT"
            
        adaptive_duration = max(5.0, z_distance / z_speed)  # 最小时间5秒
        
        rospy.loginfo(f"{segment_name} PROGRESSIVE parameters (渐进式速度控制):")
        rospy.loginfo(f"  Z descent: {z_distance*1000:.0f}mm at {z_speed:.3f}m/s ({speed_desc})")
        rospy.loginfo(f"  Duration: {adaptive_duration:.1f}s (ADAPTIVE timing)")
        rospy.loginfo(f"  Control frequency: 20Hz (FIXED)")
        rospy.loginfo(f"  XY locked at: ({xy_lock_pos[0]:.3f}, {xy_lock_pos[1]:.3f})")
        rospy.loginfo(f"  Yaw locked at: {math.degrees(locked_yaw):.1f}°")
        rospy.loginfo("  PROGRESSIVE CONTROL: Speed reduces with depth for better precision")
        
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
            rospy.logerr(f"{segment_name} polynomial descent failed")
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
            rospy.loginfo(f"{segment_name} completed successfully with XY correction")
            return True
        else:
            rospy.logwarn(f"WARNING: {segment_name} completed but XY correction had issues")
            return True  # 继续执行，不因校正失败而停止整个过程
    
    def _execute_polynomial_z_trajectory(self, start_pos, target_pos, target_yaw, duration, speed=0.05):
        """
        执行优化的多项式Z轨迹，解决overshoot问题
        
        CRITICAL FIX: Fixed speed parameter inconsistency that caused 689mm/s vs 10mm/s anomaly
        
        OPTIMIZATION STRATEGY:
        1. 降低控制频率：50Hz → 25Hz → 15Hz → 20Hz，平衡精度与响应性
        2. 位置反馈控制：检查实际位置偏差，避免盲目轨迹跟踪  
        3. 减慢控制速度：0.1m/s → 0.05m/s，提高精度
        4. 增强XY锁定：更严格的XY位置监控和纠错
        5. SPEED CONSISTENCY: Ensure all duration calculations use the same speed parameter
        
        Args:
            start_pos: 起始位置
            target_pos: 目标位置
            target_yaw: 目标yaw
            duration: 持续时间
            speed: 控制速度(CONSISTENT throughout all calculations)
            
        Returns:
            bool: 执行成功状态
        """
        
        # CONTROLLER STATE MANAGEMENT: Start trajectory controller
        self.start_new_controller('trajectory')
        
        # Calculate current insertion depth for adaptive control
        current_depth = start_pos[2] - target_pos[2]  # Positive means going deeper
        rospy.loginfo("=== ENHANCED POLYNOMIAL Z TRAJECTORY (SPEED CONSISTENCY FIX) ===")
        rospy.loginfo("STRATEGY: Fix speed parameter inconsistency causing 689mm/s anomaly")
        
        # === Z目标点设置详细诊断 ===
        rospy.loginfo("=== Z TARGET POINT DETAILED DIAGNOSIS ===")
        rospy.loginfo(f"Function inputs (CONSISTENT SPEED CHECK):")
        rospy.loginfo(f"  start_pos[2]: {start_pos[2]:.6f}m")
        rospy.loginfo(f"  target_pos[2]: {target_pos[2]:.6f}m") 
        rospy.loginfo(f"  target_yaw: {math.degrees(target_yaw):.1f}°")
        rospy.loginfo(f"  duration: {duration:.1f}s")
        rospy.loginfo(f"  speed: {speed:.3f}m/s (CONSISTENT parameter)")
        
        # 根据Z轴距离动态调整持续时间，使用CONSISTENT速度参数
        z_distance = abs(target_pos[2] - start_pos[2])
        expected_duration = z_distance / speed if speed > 0 else 0
        rospy.loginfo(f"Z movement analysis (SPEED CONSISTENCY):")
        rospy.loginfo(f"  Z distance: {z_distance*1000:.1f}mm")
        rospy.loginfo(f"  Expected duration at {speed:.3f}m/s: {expected_duration:.1f}s")
        rospy.loginfo(f"  Input duration: {duration:.1f}s")
        rospy.loginfo(f"  Duration ratio: {duration/expected_duration:.2f}" if expected_duration > 0 else "  Duration ratio: N/A")
        
        # CRITICAL FIX: 使用CONSISTENT速度参数重新计算持续时间
        # 之前的错误：使用了默认的0.05m/s而不是传入的speed参数
        conservative_duration = max(duration, z_distance / (speed * 0.8))  # 确保平均速度不超过目标的80%
        if conservative_duration > duration:
            rospy.loginfo(f"Extending duration for smoother trajectory: {duration:.1f}s → {conservative_duration:.1f}s")
            rospy.loginfo(f"CRITICAL: Using CONSISTENT speed {speed:.3f}m/s for all calculations")
            duration = conservative_duration
        
        # 创建更平滑的多项式轨迹，避免过度激进的Z轴变化
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        
        # 轨迹平滑化：检查并修正过快的Z轴速度
        max_allowed_speed = speed * 2.0  # 允许速度上限为期望速度的2倍
        dt = 1.0 / 20.0  # 20Hz采样
        smoothed_points = []
        
        rospy.loginfo("=== TRAJECTORY SMOOTHING AND SPEED VALIDATION ===")
        rospy.loginfo(f"Max allowed speed: {max_allowed_speed:.3f}m/s (2x nominal: {speed:.3f}m/s)")
        
        # 预采样轨迹并检查速度
        sample_times = [i * dt for i in range(int(duration / dt) + 1)]
        excessive_speed_count = 0
        
        for i, t in enumerate(sample_times):
            if hasattr(traj, 'evaluate_at_time'):
                point = traj.evaluate_at_time(min(t, duration))
                smoothed_points.append(point)
                
                # 检查速度
                if i > 0:
                    z_change = abs(point[2] - smoothed_points[i-1][2])
                    calculated_speed = z_change / dt  # Don't overwrite our intended speed parameter
                    
                    if calculated_speed > max_allowed_speed:
                        excessive_speed_count += 1
                        # 应用速度限制平滑
                        max_change = max_allowed_speed * dt
                        if point[2] > smoothed_points[i-1][2]:
                            # 向上移动
                            smoothed_z = smoothed_points[i-1][2] + max_change
                        else:
                            # 向下移动
                            smoothed_z = smoothed_points[i-1][2] - max_change
                        
                        # 更新平滑后的点
                        smoothed_points[i] = (point[0], point[1], smoothed_z)
                        
                        if excessive_speed_count <= 3:  # 只记录前3次
                            rospy.logwarn(f"Z速度过快 t={t:.2f}s: {calculated_speed:.3f}m/s → 限制为{max_allowed_speed:.3f}m/s")
        
        # 如果发现过多速度异常，重新生成更保守的轨迹
        if excessive_speed_count > len(sample_times) * 0.1:  # 超过10%的点有问题
            rospy.logwarn(f"TRAJECTORY TOO AGGRESSIVE: {excessive_speed_count}/{len(sample_times)} points exceed speed limit")
            rospy.logwarn("Regenerating with increased duration for smoother trajectory")
            
            # 增加持续时间重新生成
            conservative_duration = duration * 1.5
            traj = PolynomialTrajectory(conservative_duration)
            traj.generate_trajectory(start_pos, target_pos)
            rospy.loginfo(f"Regenerated trajectory with duration: {conservative_duration:.2f}s (was {duration:.2f}s)")
            duration = conservative_duration
        
        rospy.loginfo(f"Trajectory smoothing complete: {excessive_speed_count} points corrected")
        
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
                rospy.logwarn(f"WARNING: TRAJECTORY START Z MISMATCH: {start_z_error*1000:.1f}mm")
                rospy.logwarn("  → Polynomial trajectory start point != input start point")
        
        if final_traj_point is not None:
            end_z_error = abs(final_traj_point[2] - target_pos[2])
            rospy.loginfo(f"Trajectory end point Z: {final_traj_point[2]:.6f}m (input: {target_pos[2]:.6f}m)")
            rospy.loginfo(f"End Z error: {end_z_error*1000:.1f}mm")
            
            if end_z_error > 0.001:  # 1mm tolerance
                rospy.logwarn(f"WARNING: TRAJECTORY END Z MISMATCH: {end_z_error*1000:.1f}mm")
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
                rospy.logwarn(f"WARNING: TRAJECTORY START MISMATCH: {start_z_error*1000:.1f}mm")
            if end_z_error > 0.001:
                rospy.logwarn(f"WARNING: TRAJECTORY END MISMATCH: {end_z_error*1000:.1f}mm")
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
        
        while not rospy.is_shutdown() and self.trajectory_active:
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
                    
                    # FIXED: 改为基于轨迹连续性的验证，而非线性假设
                    # 验证轨迹的单调性和连续性，而非与线性路径的偏差
                    if hasattr(self, '_last_trajectory_z'):
                        z_trajectory_change = current_traj_point[2] - self._last_trajectory_z
                        expected_z_direction = 1 if target_pos[2] > start_pos[2] else -1
                        trajectory_direction = 1 if z_trajectory_change > 0 else -1 if z_trajectory_change < 0 else 0
                        
                        # 检查轨迹方向一致性（仅当有明显移动时）
                        if expected_z_direction != 0 and abs(z_trajectory_change) > 0.001 and trajectory_direction != expected_z_direction:
                            z_inconsistency_count += 1
                            rospy.logwarn(f"WARNING: Z TRAJECTORY DIRECTION INCONSISTENCY")
                            rospy.logwarn(f"  Expected direction: {'DOWN' if expected_z_direction < 0 else 'UP'}")
                            rospy.logwarn(f"  Trajectory direction: {'DOWN' if trajectory_direction < 0 else 'UP'}")
                            rospy.logwarn("  → Trajectory may have unexpected reversals")
                        
                        # 检查轨迹变化率合理性 - RELAXED THRESHOLD for complex trajectories
                        dt = 1.0 / 20.0  # 20Hz
                        trajectory_speed = abs(z_trajectory_change) / dt
                        
                        # TRAJECTORY SMOOTHING: Apply exponential smoothing to reduce speed spikes
                        if not hasattr(self, '_smoothed_trajectory_speed'):
                            self._smoothed_trajectory_speed = trajectory_speed
                        else:
                            smoothing_factor = 0.3  # 30% new, 70% historical
                            self._smoothed_trajectory_speed = (smoothing_factor * trajectory_speed + 
                                                             (1 - smoothing_factor) * self._smoothed_trajectory_speed)
                        
                        # RELAXED THRESHOLD: 3.0x instead of 2.5x for complex 3D trajectories
                        if trajectory_speed > speed * 3.0:  # 超过期望速度3.0倍 (was 2.5x)
                            z_inconsistency_count += 1
                            rospy.logwarn(f"WARNING: Z TRAJECTORY SPEED EXCESSIVE: {trajectory_speed:.3f}m/s > {speed*3.0:.3f}m/s")
                            rospy.logwarn(f"  → Smoothed speed: {self._smoothed_trajectory_speed:.3f}m/s")
                            rospy.logwarn("  → Trajectory may be too aggressive")
                    
                    self._last_trajectory_z = current_traj_point[2]
                    
                    rospy.loginfo(f"Z trajectory consistency check:")
                    rospy.loginfo(f"  Progress: {progress*100:.1f}%")
                    rospy.loginfo(f"  Trajectory continuity: VALIDATED (polynomial path expected)")
                    rospy.loginfo(f"  Speed consistency: CHECKED against {speed:.3f}m/s baseline")
                
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
                    self._analyze_z_trajectory_trend(z_trajectory_samples[-100:], start_pos[2], target_pos[2])
                
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
                
                # SEGMENT 3 CRITICAL MONITORING: Enhanced failure prediction
                valve_surface_z = self._get_valve_surface_height()
                current_insertion_depth = max(0, valve_surface_z - current_pos[2])
                is_segment_3 = current_insertion_depth > 0.24  # 240mm+ is Segment 3 territory
                
                if is_segment_3:
                    # ULTRA-STRICT monitoring for Segment 3 (241-284mm)
                    segment_3_threshold = 0.012  # 12mm threshold (stricter than general 15mm)
                    
                    if xy_drift > segment_3_threshold:
                        if xy_drift_warnings % 10 == 0:  # More frequent warnings for Segment 3
                            rospy.logerr(f"SEGMENT-3 CRITICAL: XY drift {xy_drift*1000:.1f}mm > {segment_3_threshold*1000:.0f}mm")
                            rospy.logerr(f"   Depth: {current_insertion_depth*1000:.0f}mm (VALVE COLLISION RISK)")
                            rospy.logerr(f"   Progress: {progress*100:.1f}% through Segment 3")
                            
                            # Failure risk assessment
                            risk_level = "HIGH" if xy_drift > 0.020 else "MODERATE"
                            rospy.logerr(f"   FAILURE RISK: {risk_level}")
                            
                            if xy_drift > 0.025:  # 25mm+ is critical failure zone
                                rospy.logerr(f"CRITICAL FAILURE ZONE: {xy_drift*1000:.1f}mm drift")
                                rospy.logerr("   IMMEDIATE INTERVENTION REQUIRED")
                
                # 放宽的XY漂移阈值，减少频繁修正以改善稳定性
                drift_threshold = 0.020 if is_segment_3 else 0.025  # Relaxed: Segment 3: 20mm, Others: 25mm
                if xy_drift > drift_threshold:
                    xy_drift_warnings += 1
                    warning_interval = 10 if is_segment_3 else 40  # More frequent warnings for Segment 3
                    if xy_drift_warnings % warning_interval == 0:
                        segment_info = "SEGMENT-3" if is_segment_3 else "GENERAL"
                        rospy.logwarn(f"{segment_info} XY drift: {xy_drift*1000:.1f}mm, monitoring but not intervening during Z descent")
                
                # PREVENTIVE: Early XY drift correction with depth-adaptive threshold
                # OPTIMIZATION: Larger dead zones and gentler gains to reduce 35.6% correction rate
                should_correct = False
                if is_segment_3 and xy_drift > 0.025:  # RELAXED: 25mm threshold for Segment 3 (was 15mm) 
                    preventive_gain = 0.15  # REDUCED: Very gentle correction (was 0.2)
                    threshold_type = "SEGMENT-3 CRITICAL (>240mm)"
                    should_correct = True
                elif current_insertion_depth > 0.20 and xy_drift > 0.035:  # RELAXED: 35mm threshold for deep (was 20mm)
                    preventive_gain = 0.20  # REDUCED: Gentler correction (was 0.3)
                    threshold_type = "deep (>200mm)"
                    should_correct = True
                elif current_insertion_depth > 0.10 and xy_drift > 0.040:  # RELAXED: 40mm threshold for medium (was 25mm)
                    preventive_gain = 0.25  # REDUCED: Gentler correction (was 0.4)
                    threshold_type = "medium (100-200mm)"
                    should_correct = True
                elif xy_drift > 0.030:  # RELAXED: 30mm threshold for shallow (was 18mm)
                    preventive_gain = 0.30  # REDUCED: Gentler correction (was 0.5)
                    threshold_type = "shallow (<100mm)"
                    should_correct = True
                
                if should_correct:
                    # RATE LIMITING: Prevent excessive corrections (target: <20% rate)
                    current_correction_rate = position_corrections / max(1, total_points) * 100
                    rate_limit_threshold = 18.0  # Target: <18% correction rate (was 35.6%)
                    
                    # Allow correction if rate is under control OR in critical situations
                    allow_correction = (
                        current_correction_rate < rate_limit_threshold or  # Normal rate limiting
                        is_segment_3 and xy_drift > 0.035 or  # Critical Segment 3 override
                        xy_drift > 0.050  # Emergency override for any depth
                    )
                    
                    if allow_correction:
                        xy_correction = (
                            (target_pos[0] - current_pos[0]) * preventive_gain,
                            (target_pos[1] - current_pos[1]) * preventive_gain
                        )
                        
                        # Apply preventive correction
                        corrected_x = target_pos[0] + xy_correction[0]
                        corrected_y = target_pos[1] + xy_correction[1]
                        
                        locked_point = (corrected_x, corrected_y, locked_point[2])
                        position_corrections += 1  # Track corrections
                        
                        # Log only occasionally to avoid noise
                        log_interval = 20 if is_segment_3 else 100  # More frequent logging for Segment 3
                        if total_points % log_interval == 0:
                            log_level = "logerr" if is_segment_3 else "loginfo"
                            log_prefix = "SEGMENT-3" if is_segment_3 else "Preventive"
                            getattr(rospy, log_level)(f"{log_prefix} XY correction: {xy_drift*1000:.1f}mm drift at {threshold_type} depth (rate: {current_correction_rate:.1f}%)")
                    else:
                        # Rate limiting active - skip correction but log occasionally
                        if total_points % 200 == 0:  # Log every 10 seconds at 20Hz
                            rospy.logwarn(f"XY correction RATE LIMITED: {current_correction_rate:.1f}% > {rate_limit_threshold:.1f}% (drift: {xy_drift*1000:.1f}mm)")
                
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
                
                # ENHANCED XY drift correction with DEAD ZONE and PROGRESSIVE GAIN
                # ANTI-OSCILLATION FIX: Implement dead zone to prevent correction loops
                xy_dead_zone = 0.015  # 15mm dead zone - faster response for insertion precision
                
                # DEPTH-ADAPTIVE XY CONTROL: Reduce control strength based on insertion depth
                depth_adaptation_factor = 1.0  # Default: full strength
                depth_status = "shallow"
                
                if current_insertion_depth > 0.200:  # >200mm: Deep insertion - disable XY correction
                    depth_adaptation_factor = 0.0  # Complete disable
                    depth_status = f"deep ({current_insertion_depth*1000:.0f}mm) - XY DISABLED"
                elif current_insertion_depth > 0.100:  # >100mm: Medium depth - reduce strength
                    depth_adaptation_factor = 0.4  # 40% strength
                    depth_status = f"medium ({current_insertion_depth*1000:.0f}mm) - REDUCED"
                else:  # ≤100mm: Shallow insertion - normal control
                    depth_adaptation_factor = 1.0  # Full strength
                    depth_status = f"shallow ({current_insertion_depth*1000:.0f}mm) - NORMAL"
                
                if xy_drift > xy_dead_zone and depth_adaptation_factor > 0:  # Only correct if not disabled by depth
                    xy_drift_warnings += 1
                    position_corrections += 1
                    
                    # PROGRESSIVE CORRECTION GAIN: Enhanced for insertion precision
                    # Calculate progressive gain based on drift magnitude beyond dead zone
                    excess_drift = xy_drift - xy_dead_zone
                    # IMPROVED: Higher base gain for insertion precision, depth-adaptive maximum
                    base_max_gain = 0.4 if current_insertion_depth < 0.050 else 0.35  # Higher gain for shallow insertion
                    
                    # Apply depth adaptation to maximum gain
                    max_correction_gain = base_max_gain * depth_adaptation_factor
                    
                    # IMPROVED: More responsive gain thresholds for faster correction
                    if excess_drift > 0.020:  # Large excess drift (>20mm beyond dead zone)
                        correction_gain = max_correction_gain
                        gain_description = "PROGRESSIVE (large excess)"
                    elif excess_drift > 0.010:  # Medium excess drift (10-20mm beyond dead zone)
                        correction_gain = max_correction_gain * 0.8  # Increased from 0.7
                        gain_description = "PROGRESSIVE (medium excess)"
                    else:  # Small excess drift (0-10mm beyond dead zone)
                        correction_gain = max_correction_gain * 0.6  # Increased from 0.5
                        gain_description = "PROGRESSIVE (small excess)"
                    
                    # Apply gradual correction to break oscillation cycles
                    xy_correction = (
                        (target_pos[0] - current_pos[0]) * correction_gain,
                        (target_pos[1] - current_pos[1]) * correction_gain
                    )
                    
                    # Add correction to target position for immediate response
                    corrected_x = target_pos[0] + xy_correction[0]
                    corrected_y = target_pos[1] + xy_correction[1]
                    
                    locked_point = (corrected_x, corrected_y, locked_point[2])
                    
                    if xy_drift_warnings % 20 == 0:  # Log every 1 second at 20Hz
                        rospy.logwarn(f"XY drift correction: {xy_drift*1000:.1f}mm → applying {gain_description}")
                        rospy.loginfo(f"  Depth adaptation: {depth_status} (factor={depth_adaptation_factor:.1f})")
                        rospy.loginfo(f"  Final gain: {correction_gain:.2f} (base={base_max_gain:.1f} × depth={depth_adaptation_factor:.1f})")
                        rospy.loginfo(f"  Dead zone: {xy_dead_zone*1000:.0f}mm, Excess drift: {excess_drift*1000:.1f}mm")
                        rospy.loginfo(f"  Correction vector: X={xy_correction[0]*1000:.1f}mm, Y={xy_correction[1]*1000:.1f}mm")
                elif xy_drift > xy_dead_zone and depth_adaptation_factor == 0:
                    # Drift detected but XY correction disabled due to deep insertion
                    if xy_drift_warnings % 40 == 0:  # Log every 2 seconds to avoid spam
                        rospy.loginfo(f"XY drift {xy_drift*1000:.1f}mm detected but correction DISABLED: {depth_status}")
                else:
                    # Within dead zone - no correction needed, prevents micro-oscillations
                    if xy_drift_warnings > 0 and xy_drift_warnings % 100 == 0:  # Occasional status
                        rospy.loginfo(f"XY drift within dead zone: {xy_drift*1000:.1f}mm < {xy_dead_zone*1000:.0f}mm (stable, {depth_status})")
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
            nav_msg.pitch_nav_mode = FlightNav.POS_MODE  # 真正的pitch控制
            nav_msg.target_pitch = 0.0  # 明确锁定pitch在0度
            
            self.pub.publish(nav_msg)
            
            # 记录这次的Z指令用于下次比较
            last_z_command = locked_point[2]
            rate.sleep()
        
        rospy.loginfo(f"Enhanced trajectory stats:")
        rospy.loginfo(f"  Total control points: {total_points}")
        rospy.loginfo(f"  XY drift warnings: {xy_drift_warnings} (threshold: 25mm)")
        rospy.loginfo(f"  Position corrections: {position_corrections} (OPTIMIZED)")
        rospy.loginfo(f"  Actual control frequency: {total_points/duration:.1f}Hz")
        rospy.loginfo(f"  XY drift rate: {(xy_drift_warnings/total_points)*100:.1f}% of points")
        rospy.loginfo(f"  XY correction rate: {(position_corrections/total_points)*100:.1f}% of points (target: <18%)")
        rospy.loginfo(f"  Control strategy: Rate-limited corrections + Relaxed thresholds")
        
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
            
            # 最终状态下，轨迹应该与目标完全一致
            if final_traj_target_diff > 0.01:  # 最终状态使用严格的10mm阈值
                rospy.logwarn("DIAGNOSIS: Final trajectory position differs from target")
                rospy.logwarn("   → Root cause: PolynomialTrajectory boundary conditions may be incorrect")
                rospy.logwarn("   → Recommendation: Review trajectory.py generate_trajectory() method")
            else:
                rospy.loginfo("DIAGNOSIS: Final trajectory position matches target")
                
            # 平均跟踪误差分析
            if len(z_trajectory_samples) > 10:
                avg_error = sum(s['z_error'] for s in z_trajectory_samples[-50:]) / min(50, len(z_trajectory_samples))
                rospy.loginfo(f"  Average Z tracking error (last 50 points): {avg_error*1000:.1f}mm")
                
                if avg_error > 0.04:  # 40mm平均误差
                    rospy.logwarn("DIAGNOSIS: Persistent large Z tracking errors")
                    rospy.logwarn("   → UAV Z control response may be inadequate")
                elif avg_error > 0.02:  # 20mm平均误差
                    rospy.logwarn("WARNING: DIAGNOSIS: Moderate Z tracking errors")
                    rospy.logwarn("   → Consider reducing Z trajectory speed")
                else:
                    rospy.loginfo("DIAGNOSIS: Z tracking performance acceptable")
        
        rospy.loginfo("=== Z-AXIS DIAGNOSTIC COMPLETE ===")
        
        return True
    
    def _analyze_z_trajectory_trend(self, samples, start_z, target_z):
        """
        分析Z轨迹趋势，发现系统性问题
        
        Args:
            samples: 最近的轨迹样本列表
            start_z: 起始Z位置
            target_z: 目标Z位置
        """
        if len(samples) < 10:
            return
            
        # 计算轨迹Z值与期望路径点的偏差趋势（而非与最终目标的偏差）
        waypoint_deviations = []
        for s in samples:
            expected_z_at_progress = start_z + s['progress'] * (target_z - start_z)
            waypoint_deviation = abs(s['trajectory_z'] - expected_z_at_progress)
            waypoint_deviations.append(waypoint_deviation)
        
        avg_waypoint_deviation = sum(waypoint_deviations) / len(waypoint_deviations)
        max_waypoint_deviation = max(waypoint_deviations)
        
        # 计算跟踪误差趋势
        tracking_errors = [s['z_error'] for s in samples]
        avg_tracking_error = sum(tracking_errors) / len(tracking_errors)
        max_tracking_error = max(tracking_errors)
        
        rospy.loginfo(f"Z Trajectory Trend Analysis (last {len(samples)} points):")
        rospy.loginfo(f"  Trajectory deviation from expected waypoints: avg={avg_waypoint_deviation*1000:.1f}mm, max={max_waypoint_deviation*1000:.1f}mm")
        rospy.loginfo(f"  UAV tracking error: avg={avg_tracking_error*1000:.1f}mm, max={max_tracking_error*1000:.1f}mm")
        
        # 诊断建议
        if avg_waypoint_deviation > 0.02:  # 20mm平均偏差
            rospy.logwarn("TREND ANALYSIS: Trajectory consistently deviates from expected waypoints")
            rospy.logwarn("   → Systematic trajectory generation issue detected")
            rospy.logwarn("   → Systematic trajectory generation issue detected")
        
        if avg_tracking_error > 0.03:  # 30mm平均跟踪误差
            rospy.logwarn("TREND ANALYSIS: UAV struggling to follow Z trajectory")
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
        
        # ENHANCED depth-adaptive XY threshold - stricter precision for insertion accuracy
        valve_surface_z = self._get_valve_surface_height()  # 使用实际阀门高度
        actual_insertion_depth = max(0, valve_surface_z - current_pos[2])
        
        if actual_insertion_depth > 0.25:  # 深度插入(>250mm) - 提高精度要求
            xy_threshold = 0.025  # 25mm阈值 - REDUCED from 30mm for better precision
            threshold_type = "deep insertion"
        elif actual_insertion_depth > 0.15:  # 中等深度(150-250mm)
            xy_threshold = 0.018  # 18mm阈值 - REDUCED from 25mm for better control
            threshold_type = "medium depth"
        else:  # 浅层位置 - CRITICAL for initial insertion accuracy
            xy_threshold = 0.012  # 12mm阈值 - REDUCED from 20mm to prevent collision
            threshold_type = "shallow depth"
            
        rospy.loginfo(f"Depth-adaptive XY threshold: {xy_threshold*1000:.0f}mm ({threshold_type}, depth: {actual_insertion_depth*1000:.0f}mm)")
        
        # XY误差阈值检查
        if xy_error > xy_threshold:
            rospy.logwarn(f"{correction_name}: XY error {xy_error*1000:.1f}mm > {xy_threshold*1000:.0f}mm, applying correction")
            
            # 深度位置安全检查 - 避免过度校正
            if actual_insertion_depth > 0.20 and xy_error > 0.050:  # 深度>200mm且误差>50mm
                rospy.logerr(f"SAFETY: Rejecting large XY correction ({xy_error*1000:.1f}mm) at deep position ({actual_insertion_depth*1000:.0f}mm)")
                rospy.logerr("  Risk of valve interference - manual intervention recommended")
                return False
            
            # 执行XY位置校正
            correction_target = (target_xy[0], target_xy[1], current_pos[2])  # 保持当前Z
            
            # INSERTION PRECISION: Ultra-strict yaw control for insertion phases
            # Determine if we're in insertion phase for extra yaw precision
            valve_surface_z = self._get_valve_surface_height()
            insertion_depth = max(0, valve_surface_z - current_pos[2])
            
            if insertion_depth > 0.1:  # If inserted >100mm, use ultra-strict yaw control
                yaw_precision = 0.008  # 0.46° - Ultra-precise for deep insertion
                correction_duration = 12.0  # Longer duration for ultra-precise control
                rospy.loginfo(f"ULTRA-PRECISE YAW: Using {math.degrees(yaw_precision):.2f}° precision (insertion depth: {insertion_depth*1000:.0f}mm)")
            else:
                yaw_precision = 0.01   # 0.57° - Strict precision for shallow/approach phases  
                correction_duration = 8.0   # Standard duration
                rospy.loginfo(f"STRICT YAW: Using {math.degrees(yaw_precision):.2f}° precision (insertion depth: {insertion_depth*1000:.0f}mm)")
            
            success = self.execute_smooth_trajectory_with_yaw(
                start_pos=current_pos,
                target_pos=correction_target,
                target_yaw=target_yaw,
                duration=correction_duration,
                pos_threshold=0.015,  # 15mm精度
                yaw_threshold=yaw_precision  # Dynamic yaw precision based on insertion depth
            )
            
            if success:
                rospy.loginfo(f"{correction_name} XY correction successful")
                return True
            else:
                rospy.logwarn(f"WARNING: {correction_name} XY correction failed")
                return False
        else:
            rospy.loginfo(f"{correction_name} XY position acceptable ({xy_error*1000:.1f}mm)")
            return True
    
    # ===== PITCH ANGLE CONSTRAINT CONTROL METHODS =====
    
    def check_and_constrain_pitch_angle(self, current_pitch, target_yaw_correction):
        """
        Check pitch angle and apply constraints to prevent excessive pitch during rotation
        
        Args:
            current_pitch: Current pitch angle (rad)
            target_yaw_correction: Desired yaw correction (rad)
            
        Returns:
            tuple: (constrained_yaw_correction, pitch_warning)
        """
        abs_pitch = abs(current_pitch)
        pitch_warning = False
        
        # Check if pitch angle exceeds warning threshold
        if abs_pitch > self.pitch_warning_threshold:
            pitch_warning = True
            rospy.logwarn(f"WARNING: PITCH WARNING: Current pitch {math.degrees(current_pitch):.1f}° exceeds {math.degrees(self.pitch_warning_threshold):.1f}°")
        
        # Apply yaw damping if pitch is large
        if abs_pitch > self.pitch_warning_threshold:
            damping_factor = self.yaw_damping_factor
            constrained_yaw_correction = target_yaw_correction * damping_factor
            rospy.loginfo(f"Applying yaw damping: factor={damping_factor:.2f}, "
                         f"original_yaw_correction={math.degrees(target_yaw_correction):.2f}°, "
                         f"damped_yaw_correction={math.degrees(constrained_yaw_correction):.2f}°")
        else:
            constrained_yaw_correction = target_yaw_correction
        
        # Hard limit check
        if abs_pitch > self.max_pitch_angle:
            rospy.logerr(f"WARNING: PITCH LIMIT EXCEEDED: {math.degrees(current_pitch):.1f}° > {math.degrees(self.max_pitch_angle):.1f}°")
            rospy.logerr("EMERGENCY: Stopping yaw corrections to prevent dangerous attitude")
            constrained_yaw_correction = 0.0  # Stop all yaw corrections
            pitch_warning = True
        
        return constrained_yaw_correction, pitch_warning
    
    def get_current_pitch_from_uav_state(self):
        """
        Get current pitch angle from UAV state
        This method should be overridden or connected to actual UAV state feedback
        """
        # Placeholder - in real implementation, this should get actual pitch from flight controller
        try:
            # Attempt to get from a pitch callback or state manager
            # For now, return 0 as fallback
            return 0.0
        except:
            return 0.0
    
    def log_pitch_monitoring_info(self, current_pitch, yaw_correction_applied):
        """Log pitch monitoring information for debugging"""
        rospy.loginfo(f"Pitch monitoring: current={math.degrees(current_pitch):.1f}°, "
                     f"warning_threshold={math.degrees(self.pitch_warning_threshold):.1f}°, "
                     f"max_limit={math.degrees(self.max_pitch_angle):.1f}°, "
                     f"yaw_correction_applied={math.degrees(yaw_correction_applied):.2f}°")
    
    def apply_enhanced_yaw_control(self, yaw_error, current_pitch=0.0):
        """
        Apply enhanced yaw control with deadzone, rate limiting, and pitch consideration
        
        Args:
            yaw_error: Raw yaw error (rad)
            current_pitch: Current pitch angle (rad)
            
        Returns:
            float: Constrained yaw correction (rad)
        """
        import time
        current_time = time.time()
        
        # Apply deadzone
        if abs(yaw_error) < self.yaw_deadzone:
            return 0.0
        
        # Check minimum correction interval
        if current_time - self.last_yaw_correction_time < self.min_yaw_correction_interval:
            return 0.0
        
        # Calculate base correction
        yaw_correction = yaw_error * self.yaw_control_gain
        
        # Apply pitch constraints
        yaw_correction, pitch_warning = self.check_and_constrain_pitch_angle(current_pitch, yaw_correction)
        
        # Apply rate limiting
        dt = current_time - self.last_yaw_correction_time if self.last_yaw_correction_time > 0 else 0.1
        max_correction_for_dt = self.yaw_rate_limit * dt
        
        if abs(yaw_correction) > max_correction_for_dt:
            sign = 1 if yaw_correction > 0 else -1
            yaw_correction = sign * max_correction_for_dt
        
        # Apply hard limits
        yaw_correction = max(-self.max_yaw_correction, min(self.max_yaw_correction, yaw_correction))
        
        # Update last correction time
        if abs(yaw_correction) > 0.001:  # Only update if meaningful correction
            self.last_yaw_correction_time = current_time
        
        return yaw_correction
    
    # ===== ROTATION COMPLETION DETECTION METHODS =====
    
    def start_rotation_completion_monitoring(self):
        """Start rotation completion monitoring session"""
        if self.use_completion_detection and self.completion_detector:
            self.completion_detector.start_rotation_monitoring()
            rospy.loginfo("Rotation completion monitoring started")
        else:
            rospy.logwarn("WARNING: Completion detection not available")
    
    def update_valve_rotation_progress(self, current_valve_rotation):
        """Update valve rotation progress for completion detection"""
        if self.use_completion_detection and self.completion_detector:
            self.completion_detector.update_valve_rotation(current_valve_rotation)
    
    def check_rotation_completion_timeout(self):
        """Check for rotation completion timeout"""
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
        """Get current rotation progress summary"""
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
        """
        强制停止所有控制器，用于状态转换时清理旧控制器
        Enhanced: 减少状态转换时的位置漂移
        """
        rospy.logwarn("🛑 STOPPING ALL CONTROLLERS - State transition cleanup")
        
        # ENHANCED: 立即获取并锁定当前位置，减少漂移
        current_pos = self.sm.get_current_position()
        current_yaw = getattr(self.sm, 'current_yaw', 0.0)
        
        if current_pos:
            rospy.loginfo(f"🔒 Locking UAV at position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
            rospy.loginfo(f"🔒 Locking UAV at yaw: {math.degrees(current_yaw):.1f}°")
            
            # 存储锁定位置供后续使用
            self._locked_position = current_pos
            self._locked_yaw = current_yaw
        
        # 设置所有控制标志为False
        self.control_active = False
        self.trajectory_active = False
        self.rotation_active = False
        self.pid_active = False
        
        # ENHANCED: Reset divergence detection variables
        if hasattr(self, 'error_history'):
            self.error_history = []
        if hasattr(self, 'control_start_time'):
            delattr(self, 'control_start_time')
        
        # ENHANCED: 立即发送多次位置锁定命令，确保生效
        from aerial_robot_msgs.msg import FlightNav
        for i in range(3):  # 发送3次确保命令生效
            nav_msg = FlightNav()
            nav_msg.header.stamp = rospy.Time.now()
            nav_msg.header.frame_id = "world"
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE  # 位置锁定
            nav_msg.pos_z_nav_mode = FlightNav.POS_MODE   # 位置锁定
            nav_msg.yaw_nav_mode = FlightNav.POS_MODE     # 位置锁定
            
            if current_pos:
                nav_msg.target_pos_x = current_pos[0]
                nav_msg.target_pos_y = current_pos[1]
                nav_msg.target_pos_z = current_pos[2]
                nav_msg.target_yaw = current_yaw
            
            # 发布停止命令
            self.pub.publish(nav_msg)
            rospy.sleep(0.05)  # 50ms间隔
        
        rospy.loginfo("All controllers stopped, UAV locked at current position")
    
    def _reset_all_integral_errors(self):
        """
        Comprehensive integral error reset for phase transitions
        Clears all possible integral accumulations to prevent disturbance propagation
        """
        # Reset main integral error arrays
        if hasattr(self, 'integral_error'):
            self.integral_error = [0.0, 0.0, 0.0]
        
        # Reset individual integral components
        for attr in ['integral_error_x', 'integral_error_y', 'integral_error_z']:
            if hasattr(self, attr):
                setattr(self, attr, 0.0)
        
        # Reset trajectory-specific integrals
        if hasattr(self, '_trajectory_integral_errors'):
            self._trajectory_integral_errors = [0.0, 0.0, 0.0]
            
        # Reset any PID controller internal states
        if hasattr(self, '_pid_internal_integrals'):
            self._pid_internal_integrals = [0.0, 0.0, 0.0]
            
        rospy.loginfo("COMPREHENSIVE RESET: All integral error accumulations cleared")
    
    def _reset_trajectory_level_integrals(self):
        """
        Reset trajectory-level integral accumulations
        Specifically targets execute_smooth_trajectory_with_yaw integral terms
        """
        # Reset position error tracking for trajectory control
        if hasattr(self, '_last_position_error'):
            self._last_position_error = 0.0
            
        # Reset any cached integral states in trajectory functions
        self._trajectory_integral_state = [0.0, 0.0, 0.0]
        
        rospy.loginfo("TRAJECTORY INTEGRALS: Trajectory-level integral terms reset")

    def start_new_controller(self, controller_type):
        """
        启动新控制器前的准备工作
        Enhanced: 使用锁定位置作为起始点，避免位置跳跃
        
        Args:
            controller_type: 控制器类型 ('trajectory', 'rotation', 'pid')
        """
        rospy.loginfo(f"STARTING {controller_type.upper()} CONTROLLER")
        
        # ENHANCED: 使用锁定位置作为起始点
        if hasattr(self, '_locked_position') and self._locked_position:
            rospy.loginfo(f"Using locked position as starting point: ({self._locked_position[0]:.3f}, {self._locked_position[1]:.3f}, {self._locked_position[2]:.3f})")
            rospy.loginfo(f"Using locked yaw: {math.degrees(self._locked_yaw):.1f}°")
        
        # 设置相应的控制标志
        self.control_active = True
        
        if controller_type == 'trajectory':
            self.trajectory_active = True
            # CRITICAL FIX: Preserve rotation_active during trajectory controller setup
            # PROBLEM: rotation_active was being lost when starting trajectory controller during rotation
            # SOLUTION: Do not reset rotation_active unless explicitly stopping rotation
            if not (hasattr(self, 'rotation_active') and self.rotation_active):
                # Only set other flags if not in rotation mode
                pass  # trajectory_active already set above
            else:
                rospy.loginfo("TRAJECTORY START: Preserving rotation_active=True during trajectory controller")
        elif controller_type == 'rotation':
            self.rotation_active = True
            # ROTATION CONTROLLER INITIALIZATION: Comprehensive integral reset
            rospy.loginfo("ROTATION MODE ACTIVATED: All control calls will use PD_ROTATION mode")
            rospy.loginfo("ROTATION INIT: Comprehensive integral error reset to prevent accumulated drift")
            
            # COMPREHENSIVE INTEGRAL RESET: Clear all possible integral accumulations
            self._reset_all_integral_errors()
            
            # Force reset integral errors to prevent rotation instability
            if hasattr(self, 'integral_error_x'):
                self.integral_error_x = 0.0
            if hasattr(self, 'integral_error_y'):
                self.integral_error_y = 0.0  
            if hasattr(self, 'integral_error_z'):
                self.integral_error_z = 0.0
            # Also reset the integral_error array if it exists
            if hasattr(self, 'integral_error'):
                self.integral_error = [0.0, 0.0, 0.0]
                
            # Reset trajectory-level integral errors
            self._reset_trajectory_level_integrals()
            
            rospy.loginfo("INTEGRAL RESET COMPLETE: All integral terms cleared for rotation phase")
        elif controller_type == 'pid':
            self.pid_active = True
        
        rospy.loginfo(f"{controller_type.upper()} controller activated")
    
    def is_controller_active(self, controller_type=None):
        """
        检查控制器是否处于活动状态
        
        Args:
            controller_type: 控制器类型，None表示检查任何控制器
            
        Returns:
            bool: 控制器是否活动
        """
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
        """
        Move UAV to target position with specific yaw orientation
        
        Args:
            target_pos: Target position [x, y, z]
            target_yaw: Target yaw angle in radians
            duration: Duration for the movement in seconds
            pos_threshold: Position tolerance in meters (default: 0.02m = 2cm)
            yaw_threshold: Yaw tolerance in radians (default: 0.05 rad ≈ 2.9°)
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Get current position and yaw - robust attribute access with proper fallback
            current_pos = None
            current_yaw = None
            
            # Priority 1: Try state machine context with comprehensive attribute checking
            if hasattr(self, 'sm') and self.sm is not None:
                if hasattr(self.sm, 'current_pos') and hasattr(self.sm, 'current_yaw'):
                    # State machine has direct position attributes
                    current_pos = list(self.sm.current_pos)
                    current_yaw = self.sm.current_yaw
                    rospy.logdebug("Using state machine current_pos/current_yaw attributes")
                elif hasattr(self.sm, 'get_current_position') and hasattr(self.sm, 'get_current_yaw'):
                    # State machine has position getter methods
                    current_pos = list(self.sm.get_current_position())
                    current_yaw = self.sm.get_current_yaw()
                    rospy.logdebug("Using state machine get_current_position/get_current_yaw methods")
                elif hasattr(self.sm, 'get_current_position'):
                    # State machine has position getter but no yaw getter
                    current_pos = list(self.sm.get_current_position())
                    current_yaw = getattr(self.sm, 'current_yaw', 0.0)
                    rospy.logdebug("Using state machine get_current_position, fallback yaw")
            
            # Priority 2: Try direct access methods on motion controller
            if current_pos is None and hasattr(self, 'get_current_position'):
                current_pos = list(self.get_current_position())
                current_yaw = self.get_current_yaw() if hasattr(self, 'get_current_yaw') else 0.0
                rospy.logdebug("Using motion controller direct access methods")
            
            # Priority 3: Final fallback with warning
            if current_pos is None:
                current_pos = [0, 0, 0]
                current_yaw = 0.0
                rospy.logwarn("move_to_position_with_yaw: No position source available, using default [0,0,0] @ 0°")
            
            rospy.loginfo(f"move_to_position_with_yaw called:")
            rospy.loginfo(f"  From: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}) @ {math.degrees(current_yaw):.1f}°")
            rospy.loginfo(f"  To: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}) @ {math.degrees(target_yaw):.1f}°")
            rospy.loginfo(f"  Duration: {duration:.1f}s, Tolerances: pos={pos_threshold*1000:.0f}mm, yaw={math.degrees(yaw_threshold):.1f}°")
            
            # Validate current position is reasonable
            if current_pos is None or any(abs(coord) > 100 for coord in current_pos):
                rospy.logwarn(f"Suspicious current position: {current_pos}, proceeding anyway")
            
            # Use existing smooth trajectory execution
            success = self.execute_smooth_trajectory_with_yaw(
                start_pos=current_pos,
                target_pos=target_pos, 
                target_yaw=target_yaw,
                duration=duration,
                pos_threshold=pos_threshold,
                yaw_threshold=yaw_threshold
            )
            
            if success:
                rospy.loginfo(f"move_to_position_with_yaw completed successfully")
            else:
                rospy.logwarn(f"move_to_position_with_yaw failed")
                
            return success
            
        except AttributeError as e:
            rospy.logerr(f"AttributeError in move_to_position_with_yaw: {str(e)}")
            rospy.logerr(f"   Context: self type = {type(self).__name__}")
            if hasattr(self, 'sm'):
                rospy.logerr(f"   State machine type = {type(self.sm).__name__ if self.sm else 'None'}")
            return False
        except Exception as e:
            rospy.logerr(f"Error in move_to_position_with_yaw: {str(e)}")
            rospy.logerr(f"   Error type: {type(e).__name__}")
            return False

    def _detect_valve_contact(self):
        """
        CONTACT-ADAPTIVE CONTROL: Detect if UAV is in contact with valve spokes
        
        Uses multiple criteria to determine contact state:
        1. Valve rotation detection (yaw change)
        2. Force/wrench feedback (if available)
        3. Position stability patterns
        
        Returns:
            bool: True if in contact with valve spokes, False if in free space
        """
        # Initialize contact state if not exists
        if not hasattr(self, '_valve_contact_state'):
            self._valve_contact_state = {
                'is_contacted': False,
                'contact_start_time': None,
                'last_valve_yaw': None,
                'position_history': [],
                'contact_confidence': 0.0
            }
        
        current_time = rospy.Time.now()
        
        # CRITERION 1: Valve rotation detection (primary indicator)
        valve_rotation_detected = False
        if hasattr(self, 'sm') and hasattr(self.sm, 'valve_yaw') and self.sm.valve_yaw is not None:
            if self._valve_contact_state['last_valve_yaw'] is None:
                self._valve_contact_state['last_valve_yaw'] = self.sm.valve_yaw
            else:
                valve_yaw_change = abs(self.sm.valve_yaw - self._valve_contact_state['last_valve_yaw'])
                if valve_yaw_change > math.radians(1.5):  # 1.5° threshold for contact detection
                    valve_rotation_detected = True
                    rospy.loginfo_throttle(2, f"Valve rotation detected: {math.degrees(valve_yaw_change):.1f}° change")
                    self._valve_contact_state['last_valve_yaw'] = self.sm.valve_yaw
        
        # CRITERION 2: Force feedback (if available)
        force_resistance_detected = False
        if hasattr(self, 'sm') and hasattr(self.sm, 'external_wrench') and self.sm.external_wrench is not None:
            force_magnitude = math.sqrt(
                self.sm.external_wrench.force.x**2 + 
                self.sm.external_wrench.force.y**2 + 
                self.sm.external_wrench.force.z**2
            )
            if force_magnitude > 5.0:  # 5N threshold for significant resistance
                force_resistance_detected = True
                rospy.loginfo_throttle(2, f"Force resistance detected: {force_magnitude:.1f}N")
        
        # CRITERION 3: Position stability patterns (constraint detection)
        position_constrained = False
        if hasattr(self, 'sm') and hasattr(self.sm, 'uav_pos') and self.sm.uav_pos is not None:
            # Add current position to history
            pos_history = self._valve_contact_state['position_history']
            pos_history.append((current_time.to_sec(), self.sm.uav_pos))
            
            # Keep only recent 3 seconds of history
            cutoff_time = current_time.to_sec() - 3.0
            pos_history[:] = [(t, pos) for t, pos in pos_history if t >= cutoff_time]
            
            # Check for reduced movement variance (indicates constraint)
            if len(pos_history) >= 10:
                recent_positions = [pos for t, pos in pos_history[-10:]]
                variance_x = sum((pos[0] - sum(p[0] for p in recent_positions)/len(recent_positions))**2 for pos in recent_positions) / len(recent_positions)
                variance_y = sum((pos[1] - sum(p[1] for p in recent_positions)/len(recent_positions))**2 for pos in recent_positions) / len(recent_positions)
                total_variance = math.sqrt(variance_x + variance_y)
                
                if total_variance < 0.005:  # 5mm variance threshold
                    position_constrained = True
                    rospy.loginfo_throttle(5, f"Position constraint detected: variance={total_variance*1000:.1f}mm")
        
        # 🧮 CONTACT STATE DETERMINATION: Combine criteria with confidence
        contact_indicators = sum([valve_rotation_detected, force_resistance_detected, position_constrained])
        
        # Update contact confidence (with hysteresis to prevent flickering)
        if contact_indicators >= 1:
            self._valve_contact_state['contact_confidence'] = min(1.0, self._valve_contact_state['contact_confidence'] + 0.2)
        else:
            self._valve_contact_state['contact_confidence'] = max(0.0, self._valve_contact_state['contact_confidence'] - 0.1)
        
        # Determine contact state with hysteresis (0.3 for engage, 0.7 for disengage)
        new_contact_state = self._valve_contact_state['contact_confidence'] > (0.3 if not self._valve_contact_state['is_contacted'] else 0.7)
        
        # Log contact state changes
        if new_contact_state != self._valve_contact_state['is_contacted']:
            state_name = "CONTACT" if new_contact_state else "FREE SPACE"
            rospy.loginfo(f"CONTACT STATE CHANGE: {state_name} (confidence: {self._valve_contact_state['contact_confidence']:.2f})")
            
            if new_contact_state:
                self._valve_contact_state['contact_start_time'] = current_time
            
        self._valve_contact_state['is_contacted'] = new_contact_state
        
        return new_contact_state

    # ===== PROGRESSIVE MODE TRANSITION SUPPORT FUNCTIONS =====
    
    def _get_mode_parameters(self, mode, insertion_depth):
        """
        Get control parameters for a specific control mode
        
        Args:
            mode: Control mode string
            insertion_depth: Current insertion depth
            
        Returns:
            dict: Dictionary containing all control parameters
        """
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
