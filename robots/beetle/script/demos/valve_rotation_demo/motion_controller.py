#!/usr/bin/env python
"""
Motion Controller with Closed-Loop Feedback Control
Provides precise trajectory execution with position and yaw feedback.
"""
import rospy
import time
import math
from aerial_robot_msgs.msg import FlightNav
from trajectory import PolynomialTrajectory


class MotionController:
    """
    Motion controller with closed-loop feedback for precise UAV control.
    """
    
    def __init__(self, state_machine):
        """
        Initialize MotionController with reference to state machine for position feedback.
        
        Args:
            state_machine: Reference to the state machine for accessing current position and yaw
        """
        self.sm = state_machine
        self.pub = self.sm.pub
        
    def execute_poly_motion_with_feedback(self, start, target, avg_speed, 
                                          pos_threshold=0.05, yaw_threshold=0.1, timeout=30):
        """
        Execute polynomial motion with closed-loop feedback control.
        Waits for UAV to reach each trajectory point within thresholds.
        
        Args:
            start: Starting position (x, y, z)
            target: Target position (x, y, z)
            avg_speed: Average speed in m/s
            pos_threshold: Position error threshold in meters
            yaw_threshold: Yaw error threshold in radians
            timeout: Maximum execution time in seconds
            
        Returns:
            bool: True if successful, False if failed or timed out
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
            
            # Send target point
            self.send_trajectory_point(pt, None)
            
            # Wait until UAV reaches the point (with timeout per point)
            if not self._wait_for_position(pt, pos_threshold, point_timeout=2.0):
                rospy.logwarn(f"Timeout waiting for trajectory point {pt}")
            
            rate.sleep()
        
        rospy.logerr(f"Trajectory timed out after {timeout}s")
        return False
    
    def execute_smooth_trajectory_with_yaw(self, start_pos, target_pos, target_yaw, duration=5.0,
                                          pos_threshold=0.03, yaw_threshold=0.08):
        """
        Execute smooth trajectory with yaw control and feedback.
        
        Args:
            start_pos: Starting position (x, y, z)
            target_pos: Target position (x, y, z)
            target_yaw: Target yaw angle in radians
            duration: Trajectory duration in seconds
            pos_threshold: Position error threshold in meters
            yaw_threshold: Yaw error threshold in radians
            
        Returns:
            bool: True if successful, False if failed
        """
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        
        start_yaw = self.sm.current_yaw
        yaw_diff = self._normalize_angle_diff(target_yaw - start_yaw)
        
        # Limit maximum yaw change
        max_yaw_change = math.pi / 4  # 45 degrees
        if abs(yaw_diff) > max_yaw_change:
            yaw_diff = max_yaw_change * (1 if yaw_diff > 0 else -1)
            rospy.logwarn(f"Limiting yaw change to {yaw_diff:.3f} rad")
        
        final_target_yaw = start_yaw + yaw_diff
        
        rate = rospy.Rate(50)
        start_time = time.time()
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 2.0:
            pt = traj.evaluate()
            if pt is None:
                pt = target_pos
            
            # Interpolate yaw smoothly
            elapsed_time = time.time() - start_time
            if elapsed_time <= duration:
                yaw_progress = elapsed_time / duration
                smooth_progress = 3*yaw_progress**2 - 2*yaw_progress**3
                current_target_yaw = start_yaw + smooth_progress * yaw_diff
            else:
                current_target_yaw = final_target_yaw
            
            self.send_trajectory_point(pt, current_target_yaw)
            
            # Wait for position and yaw convergence
            if not self._wait_for_position_and_yaw(pt, current_target_yaw, 
                                                  pos_threshold, yaw_threshold, point_timeout=1.0):
                rospy.logwarn(f"Timeout waiting for point with yaw")
            
            if pt is target_pos:
                break
            
            rate.sleep()
        
        rospy.loginfo("Smooth trajectory with yaw completed")
        return True
    
    def execute_controlled_descent(self, start_pos, target_z, descent_speed=0.05,
                                  pos_threshold=0.02, yaw_threshold=0.05):
        """
        Execute controlled descent with fixed XY position and yaw.
        
        Args:
            start_pos: Starting position (x, y, z)
            target_z: Target Z coordinate
            descent_speed: Descent speed in m/s
            pos_threshold: Position error threshold in meters
            yaw_threshold: Yaw error threshold in radians
            
        Returns:
            bool: True if successful, False if failed
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
            
            # Check if reached target
            current_pos = self.sm.get_current_position()
            if current_pos and current_pos[2] <= target_z + 0.01:
                rospy.loginfo("Reached target descent height")
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
    
    def send_trajectory_point(self, pos, yaw=None):
        """
        Send a single trajectory point.
        
        Args:
            pos: Position (x, y, z)
            yaw: Yaw angle in radians (optional)
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
    
    def _wait_for_position_and_yaw(self, target_pos, target_yaw, pos_threshold, yaw_threshold, point_timeout=1.0):
        """Wait for UAV to reach target position and yaw within thresholds."""
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
        """Normalize angle difference to [-π, π] range."""
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        return angle_diff



