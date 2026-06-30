#!/usr/bin/env python
"""
Simplified Motion Controller
A simplified motion controller that uses BeetleInterface internally
but maintains backward compatibility with the UnifiedMotionController API.

This follows the 分层控制 architecture:
- Geometric planning layer (this module)
- Flight controller internal PID layer (through BeetleInterface)
- No external PID controllers
"""

import os
import sys
import rospy
import time
import math
from math import pi, sqrt

# script/demos on the path so the shared demo helpers import regardless of how
# this module is launched.
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from demo_common import normalize_angle_diff


class SimplifiedMotionController:
    """
    Simplified motion controller using BeetleInterface
    
    Maintains backward compatibility with UnifiedMotionController methods
    but uses simplified geometric planning + convergence checking approach.
    """
    
    def __init__(self, beetle_interface):
        """
        Initialize simplified motion controller
        
        Args:
            beetle_interface: BeetleInterface instance for control
        """
        self.beetle = beetle_interface
        
        # Basic control parameters
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.02)
        self.yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.05236)  # 3°
        
        # Trajectory execution parameters
        self.default_timeout = rospy.get_param("~default_timeout", 30.0)
        self.waypoint_interval = rospy.get_param("~waypoint_interval", 0.1)  # 100ms
        
        # Phase control flags for compatibility
        self.final_descent_active = False
        self.return_to_start_active = False
        self.pre_final_descent = True
        self.rotation_active = False
        self.disengagement_active = False
        
        # Control state management
        self.control_active = False
        self.trajectory_active = False
        self.pid_active = False
        
        # Position locking for compatibility
        self._locked_position = None
        
        rospy.loginfo("SimplifiedMotionController initialized with BeetleInterface")
    
    def send_trajectory_point(self, position, yaw=None):
        """
        Send single trajectory point using BeetleInterface
        
        Args:
            position: Target position (x, y, z)
            yaw: Target yaw angle in radians (optional)
        """
        self.beetle.targetMotion([position[0], position[1], position[2]], yaw if yaw is not None else self.beetle.current_yaw)
    
    def execute_progressive_precision_trajectory(self, waypoints, yaw_targets=None, timeout=None, 
                                                position_tolerance=None, yaw_tolerance=None, **kwargs):
        """
        Execute trajectory with progressive precision using BeetleInterface
        
        Args:
            waypoints: List of target positions [(x,y,z), ...]
            yaw_targets: List of target yaw angles (optional)
            timeout: Maximum execution time
            position_tolerance: Position convergence tolerance
            yaw_tolerance: Yaw convergence tolerance
            **kwargs: Other parameters (ignored for compatibility)
            
        Returns:
            bool: True if successful, False if failed
        """
        if not waypoints:
            rospy.logwarn("Empty waypoints list")
            return False
        
        timeout = timeout or self.default_timeout
        pos_tol = position_tolerance or self.position_tolerance
        yaw_tol = yaw_tolerance or self.yaw_tolerance
        
        rospy.loginfo(f"Executing progressive precision trajectory with {len(waypoints)} waypoints")
        
        start_time = time.time()
        
        for i, waypoint in enumerate(waypoints):
            if time.time() - start_time > timeout:
                rospy.logwarn("Progressive precision trajectory timeout")
                return False
            
            target_yaw = yaw_targets[i] if yaw_targets and i < len(yaw_targets) else self.beetle.current_yaw
            
            rospy.loginfo(f"Moving to waypoint {i+1}/{len(waypoints)}: {waypoint}")
            
            # Use BeetleInterface's goPoseWaitConvergence
            success = self.beetle.goPoseWaitConvergence(
                target_pos=[waypoint[0], waypoint[1], waypoint[2]],
                target_yaw=target_yaw,
                pos_convergence_thresh=pos_tol,
                yaw_convergence_thresh=yaw_tol,
                timeout=timeout - (time.time() - start_time)
            )
            
            if not success:
                rospy.logwarn(f"Failed to reach waypoint {i+1}")
                return False
        
        rospy.loginfo("Progressive precision trajectory completed successfully")
        return True
    
    def execute_vel_accel_trajectory(self, target_position, target_yaw=None, timeout=None, **kwargs):
        """
        Execute velocity/acceleration trajectory using simplified approach
        
        Args:
            target_position: Target position (x, y, z)
            target_yaw: Target yaw angle (optional)
            timeout: Maximum execution time
            **kwargs: Other parameters (ignored for compatibility)
            
        Returns:
            bool: True if successful, False if failed
        """
        timeout = timeout or self.default_timeout
        target_yaw = target_yaw if target_yaw is not None else self.beetle.current_yaw
        
        rospy.loginfo(f"Executing velocity/acceleration trajectory to {target_position}")
        
        # Use BeetleInterface's goPoseWaitConvergence
        success = self.beetle.goPoseWaitConvergence(
            target_pos=target_position,
            target_yaw=target_yaw,
            timeout=timeout
        )
        
        if success:
            rospy.loginfo("Velocity/acceleration trajectory completed successfully")
        else:
            rospy.logwarn("Velocity/acceleration trajectory failed")
        
        return success
    
    def execute_smooth_trajectory_with_yaw(self, start_pos, end_pos, start_yaw, end_yaw, 
                                          duration=None, timeout=None, **kwargs):
        """
        Execute smooth trajectory with yaw using simplified approach
        
        Args:
            start_pos: Starting position (x, y, z)
            end_pos: Ending position (x, y, z)
            start_yaw: Starting yaw angle
            end_yaw: Ending yaw angle
            duration: Trajectory duration (optional)
            timeout: Maximum execution time
            **kwargs: Other parameters (ignored for compatibility)
            
        Returns:
            bool: True if successful, False if failed
        """
        timeout = timeout or self.default_timeout
        duration = duration or 5.0  # Default 5 second duration
        
        rospy.loginfo(f"Executing smooth trajectory from {start_pos} to {end_pos}")
        
        # Generate waypoints along the trajectory
        num_waypoints = max(5, int(duration / self.waypoint_interval))
        waypoints = []
        yaw_targets = []
        
        for i in range(num_waypoints + 1):
            t = float(i) / num_waypoints
            
            # Linear interpolation for position
            waypoint = [
                start_pos[0] + t * (end_pos[0] - start_pos[0]),
                start_pos[1] + t * (end_pos[1] - start_pos[1]),
                start_pos[2] + t * (end_pos[2] - start_pos[2])
            ]
            waypoints.append(waypoint)
            
            # Linear interpolation for yaw (handling angle wrapping)
            yaw_diff = self._normalize_angle_diff(end_yaw - start_yaw)
            target_yaw = start_yaw + t * yaw_diff
            yaw_targets.append(target_yaw)
        
        # Execute the trajectory
        return self.execute_progressive_precision_trajectory(
            waypoints, yaw_targets, timeout=timeout
        )
    
    def execute_three_stage_insertion_with_adaptive_replanning(self, target_position, target_yaw=None, **kwargs):
        """
        Execute three-stage insertion using simplified approach
        
        Args:
            target_position: Target position (x, y, z)
            target_yaw: Target yaw angle (optional)
            **kwargs: Other parameters (ignored for compatibility)
            
        Returns:
            bool: True if successful, False if failed
        """
        target_yaw = target_yaw if target_yaw is not None else self.beetle.current_yaw
        
        rospy.loginfo(f"Executing three-stage insertion to {target_position}")
        
        current_pos = self.beetle.uav_pos
        if not current_pos:
            rospy.logwarn("Cannot get current position for insertion")
            return False
        
        # Stage 1: Approach above target
        approach_pos = [target_position[0], target_position[1], current_pos[2]]
        rospy.loginfo("Stage 1: Approaching above target")
        if not self.beetle.goPoseWaitConvergence(approach_pos, target_yaw, timeout=15.0):
            rospy.logwarn("Stage 1 failed")
            return False
        
        # Stage 2: Descend to intermediate height
        intermediate_z = target_position[2] + 0.1  # 10cm above target
        intermediate_pos = [target_position[0], target_position[1], intermediate_z]
        rospy.loginfo("Stage 2: Descending to intermediate height")
        if not self.beetle.goPoseWaitConvergence(intermediate_pos, target_yaw, timeout=15.0):
            rospy.logwarn("Stage 2 failed")
            return False
        
        # Stage 3: Final descent to target
        rospy.loginfo("Stage 3: Final descent to target")
        self.final_descent_active = True  # Set flag for compatibility
        success = self.beetle.goPoseWaitConvergence(
            target_position, target_yaw, 
            pos_convergence_thresh=0.01,  # Higher precision for final descent
            timeout=20.0
        )
        self.final_descent_active = False
        
        if success:
            rospy.loginfo("Three-stage insertion completed successfully")
        else:
            rospy.logwarn("Stage 3 (final descent) failed")
        
        return success
    
    def stop_all_controllers(self):
        """
        Stop all controllers - compatibility method
        """
        rospy.loginfo("Stopping all controllers")
        self.control_active = False
        self.trajectory_active = False
        self.rotation_active = False
        self.final_descent_active = False
        # No actual stopping needed since BeetleInterface manages this
    
    def _normalize_angle_diff(self, angle_diff):
        """
        Normalize angle difference to [-pi, pi]
        
        Args:
            angle_diff: Angle difference in radians
            
        Returns:
            float: Normalized angle difference
        """
        return normalize_angle_diff(angle_diff)
    
    def reset_control_state(self):
        """
        Reset control state - compatibility method
        """
        self.stop_all_controllers()
        self._locked_position = None
        rospy.loginfo("Control state reset")
    
    def get_control_stats(self):
        """
        Get control statistics - compatibility method
        
        Returns:
            dict: Basic control statistics
        """
        return {
            'total_corrections': 0,
            'distance_violations': 0,
            'max_distance_error': 0.0,
            'avg_distance_error': 0.0,
            'execution_time': 0.0
        }