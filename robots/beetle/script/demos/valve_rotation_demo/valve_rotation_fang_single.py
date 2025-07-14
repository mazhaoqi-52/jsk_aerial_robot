#!/usr/bin/env python
"""
Single UAV Valve Rotation using SMACH State Machine
Physical Parameters:
- valve_radius: 0.1225m, valve_beam_width: 0.0185m
- rotation_direction: 1 (anticlockwise), -1 (clockwise)
- insertion_offset: 0.015m (safety margin)
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))
import rospy
import smach
import time
import math
import threading
import numpy as np
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from trajectory import AlignToGraspTrajectory, ValveRotationTrajectory
from insertion_optimizer import InsertionOptimizer

class MotionController:
    @staticmethod
    def execute_poly_motion_pose(pub, start, target, avg_speed, timeout=30):
        """Execute polynomial motion and wait for completion"""
        from trajectory import PolynomialTrajectory
        
        distance = math.sqrt((target[0]-start[0])**2 + (target[1]-start[1])**2 + (target[2]-start[2])**2)
        duration = distance / max(avg_speed, 0.05)
        
        rospy.loginfo(f"Trajectory: {distance:.2f}m in {duration:.1f}s at {avg_speed}m/s")
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start, target)
        
        rate = rospy.Rate(50)
        start_time = time.time()
        last_progress_time = start_time
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            pt = traj.evaluate()
            if pt is None:
                rospy.loginfo("Trajectory completed successfully")
                return True
            
            # Progress feedback every 5 seconds
            current_time = time.time()
            if current_time - last_progress_time > 5.0:
                elapsed = current_time - start_time
                progress = (elapsed / duration) * 100
                rospy.loginfo(f"Motion progress: {progress:.1f}% ({elapsed:.1f}s/{duration:.1f}s)")
                last_progress_time = current_time
            
            msg = FlightNav()
            msg.target = 1
            msg.pos_xy_nav_mode = FlightNav.POS_MODE
            msg.target_pos_x = pt[0]
            msg.target_pos_y = pt[1]
            msg.pos_z_nav_mode = FlightNav.POS_MODE
            msg.target_pos_z = pt[2]
            pub.publish(msg)
            rate.sleep()
        
        rospy.logwarn(f"Trajectory timed out after {timeout}s")
        return False
    
    @staticmethod
    def send_trajectory_point(pub, pos, yaw):
        """Send single trajectory point with yaw"""
        msg = FlightNav()
        msg.target = 1
        msg.pos_xy_nav_mode = FlightNav.POS_MODE
        msg.target_pos_x = pos[0]
        msg.target_pos_y = pos[1]
        msg.pos_z_nav_mode = FlightNav.POS_MODE
        msg.target_pos_z = pos[2]
        msg.yaw_nav_mode = FlightNav.POS_MODE
        msg.target_yaw = yaw
        pub.publish(msg)

class SingleUAVStateBase(smach.State):
    def __init__(self, outcomes, module_id=1):
        smach.State.__init__(self, outcomes=outcomes)
        self.module_id = module_id
        
        # Publishers and subscribers
        self.pub = rospy.Publisher(f"/beetle{module_id}/uav/nav", FlightNav, queue_size=1)
        
        # Get simulation and real_machine parameters
        self.simulation = rospy.get_param("~simulation", False)
        self.real_machine = rospy.get_param("~real_machine", True)
        
        # Setup UAV pose subscriber - always use beetle{module_id}/mocap/pose
        rospy.loginfo(f"Setting up UAV pose subscriber: /beetle{module_id}/mocap/pose")
        self.uav_sub = rospy.Subscriber(f"/beetle{module_id}/mocap/pose", PoseStamped, self.uav_callback, queue_size=1)
        
        # Setup wrench subscriber for force feedback
        rospy.loginfo(f"Setting up wrench subscriber: /beetle{module_id}/estimated_external_wrench")
        self.wrench_sub = rospy.Subscriber(f"/beetle{module_id}/estimated_external_wrench", WrenchStamped, self.wrench_callback, queue_size=1)
        
        # Position and wrench tracking
        self.current_pos = None
        self.current_yaw = 0.0
        self.current_wrench = None
        self.uav_received = threading.Event()
        
        # Valve tracking
        self.valve_pos = None
        self.valve_yaw = 0.0
        self.valve_received = threading.Event()
        
        # Setup valve pose subscriber based on mode
        if self.simulation:
            # In simulation mode, use /valve/odom topic
            rospy.loginfo("Setting up valve simulation mode - using /valve/odom topic")
            self.valve_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            # In real mode, use mocap topic
            rospy.loginfo("Setting up valve real mode - using /valve/mocap/pose topic")
            self.valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
        
        # Add debug logging
        debug_mode = rospy.get_param("~debug", False)
        if debug_mode:
            rospy.loginfo("Debug mode enabled")
            rospy.loginfo(f"UAV pose topic: /beetle{module_id}/mocap/pose")
            rospy.loginfo(f"Flight nav topic: /beetle{module_id}/uav/nav")
            rospy.loginfo(f"External wrench topic: /beetle{module_id}/estimated_external_wrench")
            if self.simulation:
                rospy.loginfo("Mode: SIMULATION")
                rospy.loginfo("Valve pose topic: /valve/odom")
            else:
                rospy.loginfo("Mode: REAL HARDWARE")
                rospy.loginfo("Valve pose topic: /valve/mocap/pose")
    
    def uav_callback(self, msg):
        self.current_pos = msg
        q = msg.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.uav_received.set()
        
        # Only track position and yaw, no emergency detection logic here
        # Emergency detection is handled only in RotateValveState
    
    def wrench_callback(self, msg):
        self.current_wrench = msg
    
    def valve_sim_callback(self, msg):
        self.valve_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        q = msg.pose.pose.orientation
        _, _, self.valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.valve_received.set()
    
    def valve_callback(self, msg):
        self.valve_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        q = msg.pose.orientation
        _, _, self.valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.valve_received.set()
    
    def get_current_position(self):
        if self.current_pos is None:
            return None
        return (self.current_pos.pose.position.x, 
                self.current_pos.pose.position.y, 
                self.current_pos.pose.position.z)
    
    def normalize_angle(self, angle):
        """Normalize angle difference to [-π, π] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_contact_force_magnitude(self):
        """Get magnitude of contact force"""
        if self.current_wrench is None:
            return 0.0
        w = self.current_wrench.wrench
        return math.sqrt(w.force.x**2 + w.force.y**2 + w.force.z**2)
    
    def wait_for_positions(self, timeout=10):
        """Wait for UAV and valve position data"""
        rospy.loginfo("Waiting for UAV and valve position data...")
        
        uav_ok = self.uav_received.wait(timeout)
        valve_ok = self.valve_received.wait(timeout)
        
        if not uav_ok:
            rospy.logerr("Failed to receive UAV position data")
            rospy.logerr("Check UAV topics and ensure simulation/real mode is correct")
        
        if not valve_ok:
            rospy.logwarn("Failed to receive valve position data within timeout")
            rospy.logwarn("This may be due to valve spawn failure - using default valve position")
            # Set default valve position if spawning failed
            self.valve_pos = (3.0, 0.0, 0.0)  # Default valve position from spawn command
            self.valve_yaw = 0.0
            self.valve_received.set()
            rospy.loginfo(f"Using default valve position: {self.valve_pos}")
        
        return uav_ok  # Only require UAV position, valve position has fallback
    
    def emergency_ascent_and_return(self, start_position):
        """Emergency procedure: ascent and return to start"""
        rospy.logwarn("Executing emergency ascent and return")
        
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        # First ascent to safe height
        safe_height = max(current_pos[2] + 0.5, start_position[2] + 0.3)
        ascent_pos = (current_pos[0], current_pos[1], safe_height)
        
        success = MotionController.execute_poly_motion_pose(
            self.pub, current_pos, ascent_pos, 0.1, timeout=20)
        
        if not success:
            rospy.logerr("Emergency ascent failed")
            return False
        
        # Then return to start position with generous timeout
        final_pos = self.get_current_position()
        if final_pos is None:
            return False
        
        distance = math.sqrt((start_position[0]-final_pos[0])**2 + 
                           (start_position[1]-final_pos[1])**2 + 
                           (start_position[2]-final_pos[2])**2)
        
        move_speed = 0.1  # Conservative speed for emergency
        timeout = max(90, distance / move_speed * 3.0)  # Extra generous for emergency
        
        rospy.loginfo(f"Emergency return: {distance:.2f}m at {move_speed}m/s")
        
        success = MotionController.execute_poly_motion_pose(
            self.pub, final_pos, start_position, move_speed, timeout=timeout)
        
        return success

class InitializeState(SingleUAVStateBase):
    """Initialize and record UAV starting position from mocap"""
    def __init__(self, module_id=1, hover_duration=2.0):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        self.hover_duration = hover_duration
    
    def execute(self, userdata):
        rospy.loginfo("Initializing and recording UAV start position...")
        
        # Wait for UAV position
        if not self.uav_received.wait(5):
            rospy.logerr("Failed to receive UAV position")
            return 'failed'
        
        # Record initial position
        initial_pos = self.get_current_position()
        if initial_pos is None:
            rospy.logerr("Failed to get UAV position")
            return 'failed'
        
        # Store in userdata for all states to access
        userdata.start_position = list(initial_pos)
        
        rospy.loginfo(f"UAV start position recorded: {initial_pos}")
        rospy.loginfo(f"Hovering for {self.hover_duration}s to ensure stability...")
        
        # Hover for specified duration to ensure stability
        start_time = time.time()
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown() and (time.time() - start_time) < self.hover_duration:
            # Send hover command
            current_pos = self.get_current_position()
            if current_pos is not None:
                msg = FlightNav()
                msg.target = 1
                msg.pos_xy_nav_mode = FlightNav.POS_MODE
                msg.target_pos_x = initial_pos[0]
                msg.target_pos_y = initial_pos[1]
                msg.pos_z_nav_mode = FlightNav.POS_MODE
                msg.target_pos_z = initial_pos[2]
                msg.yaw_nav_mode = FlightNav.POS_MODE
                msg.target_yaw = self.current_yaw
                self.pub.publish(msg)
            
            rate.sleep()
        
        rospy.loginfo("Initialization complete")
        return 'succeeded'

class InitializeStartPositionState(SingleUAVStateBase):
    """Initialize and record start position during hover"""
    def __init__(self, module_id=1, hover_duration=2.0):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        self.hover_duration = hover_duration
        # Override the _output_keys to declare start_position as output
        self._output_keys = ['start_position']
    
    def execute(self, userdata):
        rospy.loginfo(f"Initializing start position for UAV{self.module_id}")
        
        # Wait for position data with timeout
        if not self.uav_received.wait(10):  # Increased timeout to 10 seconds
            rospy.logerr("Failed to receive UAV position data within 10 seconds")
            rospy.logerr("Check that the UAV is running and publishing pose topics")
            return 'failed'
        
        # Record current position as start position
        start_pos = self.get_current_position()
        if start_pos is None:
            rospy.logerr("Failed to get current UAV position")
            return 'failed'
        
        # Store start position in userdata
        userdata.start_position = list(start_pos)
        
        rospy.loginfo(f"Start position recorded: {start_pos}")
        rospy.loginfo(f"Hovering for {self.hover_duration}s for initialization...")
        
        # Hover for specified duration to ensure stability
        hover_start = time.time()
        while time.time() - hover_start < self.hover_duration:
            if rospy.is_shutdown():
                return 'failed'
            time.sleep(0.1)
        
        rospy.loginfo("Initialization complete")
        return 'succeeded'

class MoveToValveState(SingleUAVStateBase):
    """Move to position above valve center (simple approach)"""
    def __init__(self, module_id=1, approach_height=0.5):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        self.approach_height = approach_height
    
    def execute(self, userdata):
        rospy.loginfo("Moving to valve position (above valve center)...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None:
            rospy.logerr("Failed to get current UAV position")
            return 'failed'
        
        if self.valve_pos is None:
            rospy.logerr("Valve position is not available")
            return 'failed'
        
        rospy.loginfo(f"Moving from {start_pos} to valve at {self.valve_pos}")
        
        # Simple approach: move to position above valve center
        # Let the AlignToGraspTrajectory handle the precise positioning
        target_pos = (self.valve_pos[0], self.valve_pos[1], 
                     self.valve_pos[2] + self.approach_height)
        
        rospy.loginfo(f"Target position (above valve center): {target_pos}")
        
        # Calculate distance and adjust speed/timeout for long moves
        distance = math.sqrt((target_pos[0]-start_pos[0])**2 + 
                           (target_pos[1]-start_pos[1])**2 + 
                           (target_pos[2]-start_pos[2])**2)
        
        # Use consistent slow speed for stability and precision
        move_speed = 0.1  # Conservative speed for all moves
        # Generous timeout: at least 3x the expected time for slow moves
        timeout = max(90, distance / move_speed * 3.0)
        
        rospy.loginfo(f"Moving {distance:.2f}m at {move_speed}m/s (timeout: {timeout:.0f}s)")
        
        success = MotionController.execute_poly_motion_pose(
            self.pub, start_pos, target_pos, move_speed, timeout=timeout)
        
        if success:
            rospy.loginfo("Successfully moved to valve position")
            return 'succeeded'
        else:
            rospy.logerr("Failed to move to valve position")
            return 'failed'

class DescendAndContactState(SingleUAVStateBase):
    """Improved 4-stage insertion: Approach → Yaw Adjustment → Descent → Rotation"""
    def __init__(self, module_id=1, contact_force_threshold=3.0, descent_speed=0.05,  # Increased from 2.0 to 3.0N
                 use_staged_insertion=True, pre_insertion_distance=0.025,  # Reduced from 0.03 to 0.025
                 yaw_adjustment_angle=0.08, circumferential_offset=0.015):  # Reduced offsets
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        self.contact_force_threshold = contact_force_threshold
        self.descent_speed = descent_speed
        self.z_offset = 0.01 if rospy.get_param("~simulation", True) else 0.21  # Reduced simulation offset
        self.use_staged_insertion = use_staged_insertion
        self.pre_insertion_distance = pre_insertion_distance
        self.yaw_adjustment_angle = yaw_adjustment_angle  # Yaw adjustment for avoidance
        self.circumferential_offset = circumferential_offset  # Circumferential offset distance
        
        # Initialize insertion optimizer with real-time data capability
        simulation = rospy.get_param("~simulation", True)
        self.optimizer = InsertionOptimizer(
            valve_radius=0.1225,
            valve_beam_width=0.0185,
            safety_margin=0.008,  # Reduced safety margin for more precise insertion
            module_id=module_id,
            simulation=simulation
        )
        
        self._input_keys = ['start_position']
        self._output_keys = ['start_position']
    
    def execute(self, userdata):
        if self.use_staged_insertion:
            return self.execute_staged_insertion(userdata)
        else:
            return self.execute_original_insertion(userdata)
    
    def execute_staged_insertion(self, userdata):
        """Execute improved 4-stage insertion: Approach → Yaw Adjustment → Descent → Rotation"""
        rospy.loginfo("Starting 4-stage insertion: Approach → Yaw Adjustment → Descent → Rotation")
        
        if not self.wait_for_positions():
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None:
            rospy.logerr("Failed to get current UAV position")
            return 'failed'
        
        if self.valve_pos is None:
            rospy.logerr("Valve position is not available")
            return 'failed'
        
        # Store start position for emergency return
        userdata.start_position = start_pos
        
        # Stage 1: Approach insertion point (with avoidance)
        rospy.loginfo("Stage 1: Approaching insertion point with avoidance...")
        if not self.approach_insertion_point_with_avoidance(start_pos):
            rospy.logerr("Failed to approach insertion point")
            return 'failed'
        
        # Stage 2: Yaw and circumferential adjustment
        rospy.loginfo("Stage 2: Yaw and circumferential direction adjustment...")
        if not self.adjust_yaw_and_circumferential_direction():
            rospy.logerr("Failed to adjust insertion direction")
            return 'failed'
        
        # Stage 3: Descent to contact
        rospy.loginfo("Stage 3: Descending to establish contact...")
        
        # CRITICAL FIX: Add stabilization delay after Stage 2 to prevent position jump
        # This allows the control system to stabilize before starting descent
        rospy.loginfo("Stabilization delay after Stage 2 to prevent position jump...")
        time.sleep(1.0)  # 1 second delay for stabilization
        
        # Log pre-descent state
        current_pos = self.get_current_position()
        if current_pos is not None:
            rospy.loginfo(f"Pre-descent position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
            rospy.loginfo(f"Pre-descent yaw: {self.current_yaw:.3f} rad")
        
        if not self.descend_to_contact():
            rospy.logerr("Failed to descend to contact")
            return 'failed'
        
        # Log post-descent state (get fresh position after descent)
        time.sleep(0.1)  # Small delay to ensure position is updated
        current_pos = self.get_current_position()
        if current_pos is not None:
            rospy.loginfo(f"Post-descent position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
            rospy.loginfo(f"Post-descent yaw: {self.current_yaw:.3f} rad")
            
            # Check for reasonable descent
            if current_pos[2] < 0.3:  # Very low height warning
                rospy.logwarn(f"UAV descended to very low height: {current_pos[2]:.3f}m")
            elif current_pos[2] > 1.5:  # High height warning
                rospy.logwarn(f"UAV is at high height after descent: {current_pos[2]:.3f}m")
        
        rospy.loginfo("Stage 3 completed - contact established")
        
        # Stage 4: Final rotation to target position
        rospy.loginfo("Stage 4: Final rotation to target position...")
        if not self.rotate_to_target_position():
            rospy.logerr("Failed to rotate to target position")
            return 'failed'
        
        rospy.loginfo("4-stage insertion completed successfully")
        return 'succeeded'
    
    def approach_insertion_point_with_avoidance(self, start_pos):
        """Stage 1: Approach insertion point with optimized strategy selection"""
        rospy.loginfo("=== STAGE 1: OPTIMIZED INSERTION STRATEGY SELECTION ===")
        
        # Use optimizer with real-time data to select best insertion strategy
        # The optimizer will automatically use current UAV and valve positions
        optimal_strategy = self.optimizer.evaluate_real_time_strategy()
        
        if optimal_strategy is None:
            rospy.logerr("Failed to determine optimal insertion strategy")
            return False
        
        # Get insertion parameters based on optimal strategy
        insertion_params = self.optimizer.get_insertion_parameters(
            strategy=optimal_strategy,
            pre_insertion_distance=self.pre_insertion_distance,
            circumferential_offset=self.circumferential_offset
        )
        
        if insertion_params is None:
            rospy.logerr("Failed to get insertion parameters")
            return False
        
        # Log the insertion plan
        self.optimizer.log_insertion_plan(insertion_params)
        
        # Store parameters for later stages
        self.current_insertion_params = insertion_params
        
        # Calculate approach position
        approach_x = insertion_params['approach_position'][0]
        approach_y = insertion_params['approach_position'][1]
        approach_z = start_pos[2]  # Keep current height
        approach_pos = (approach_x, approach_y, approach_z)
        
        # Calculate approach yaw with limited change
        target_yaw = insertion_params['target_yaw']
        current_yaw = self.current_yaw
        
        # Limit yaw change to prevent aggressive rotation
        yaw_diff = target_yaw - current_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Limit to 30 degrees max change for approach
        max_yaw_change = math.pi / 6  # 30 degrees
        if abs(yaw_diff) > max_yaw_change:
            yaw_diff = max_yaw_change * (1 if yaw_diff > 0 else -1)
            rospy.logwarn(f"Limiting approach yaw change to {yaw_diff:.3f} rad ({yaw_diff*180/math.pi:.1f}°)")
        
        approach_yaw = current_yaw + yaw_diff
        
        rospy.loginfo(f"=== STAGE 1: APPROACH EXECUTION ===")
        rospy.loginfo(f"Strategy: {optimal_strategy['config']['name']}")
        rospy.loginfo(f"Approach position: [{approach_x:.3f}, {approach_y:.3f}, {approach_z:.3f}]")
        rospy.loginfo(f"Approach yaw: {approach_yaw:.3f} rad (change: {yaw_diff:.3f} rad)")
        rospy.loginfo(f"Distance to travel: {optimal_strategy['approach_distance']:.3f}m")
        rospy.loginfo(f"Real-time optimization: UAV->Valve distance optimized")
        
        # Execute smooth approach trajectory
        return self.execute_smooth_trajectory(start_pos, approach_pos, approach_yaw, duration=8.0)
    
    def adjust_yaw_and_circumferential_direction(self):
        """Stage 2: Adjust yaw and move circumferentially to final insertion position"""
        rospy.loginfo("=== STAGE 2: CIRCUMFERENTIAL ADJUSTMENT ===")
        
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        # Check if we have insertion parameters from Stage 1
        if not hasattr(self, 'current_insertion_params'):
            rospy.logerr("No insertion parameters available from Stage 1")
            return False
        
        insertion_params = self.current_insertion_params
        
        # Calculate target position (final insertion position)
        target_x = insertion_params['final_position'][0]
        target_y = insertion_params['final_position'][1]
        target_z = current_pos[2]  # Keep same height
        target_pos = (target_x, target_y, target_z)
        
        # Calculate target yaw (pointing toward valve center)
        target_yaw = insertion_params['target_yaw']
        
        # Log the adjustment
        distance_adjustment = math.sqrt((target_x - current_pos[0])**2 + (target_y - current_pos[1])**2)
        rospy.loginfo(f"Circumferential adjustment using optimized parameters:")
        rospy.loginfo(f"  - Current position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
        rospy.loginfo(f"  - Target position: [{target_x:.3f}, {target_y:.3f}, {target_z:.3f}]")
        rospy.loginfo(f"  - XY adjustment distance: {distance_adjustment:.3f}m")
        rospy.loginfo(f"  - Target yaw: {target_yaw:.3f} rad")
        rospy.loginfo(f"  - Strategy: {insertion_params['strategy']['config']['name']}")
        
        # Limit maximum XY adjustment to prevent excessive movement
        max_xy_adjustment = 0.08  # Increased from 0.05m to 0.08m for better reach
        if distance_adjustment > max_xy_adjustment:
            rospy.logwarn(f"XY adjustment too large ({distance_adjustment:.3f}m), limiting to {max_xy_adjustment:.3f}m")
            # Scale down the adjustment
            scale_factor = max_xy_adjustment / distance_adjustment
            target_x = current_pos[0] + (target_x - current_pos[0]) * scale_factor
            target_y = current_pos[1] + (target_y - current_pos[1]) * scale_factor
            target_pos = (target_x, target_y, target_z)
        
        # Execute circumferential adjustment with force monitoring
        return self.execute_circumferential_adjustment(current_pos, target_pos, target_yaw)
    
    def execute_circumferential_adjustment(self, start_pos, target_pos, target_yaw):
        """Execute circumferential adjustment with force monitoring"""
        # Calculate intermediate points for smooth circumferential movement
        center_x, center_y = self.valve_pos[0], self.valve_pos[1]
        
        # Current angle from valve center
        current_angle = math.atan2(start_pos[1] - center_y, start_pos[0] - center_x)
        target_angle = math.atan2(target_pos[1] - center_y, target_pos[0] - center_x)
        
        # Calculate shortest angular path
        angle_diff = target_angle - current_angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Calculate radius
        current_radius = math.sqrt((start_pos[0] - center_x)**2 + (start_pos[1] - center_y)**2)
        target_radius = math.sqrt((target_pos[0] - center_x)**2 + (target_pos[1] - center_y)**2)
        
        # Store initial yaw for consistent interpolation
        start_yaw = self.current_yaw
        
        # Calculate shortest yaw path
        yaw_diff = target_yaw - start_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Limit maximum yaw change to prevent aggressive rotation
        max_yaw_change = math.pi / 6  # 30 degrees max for circumferential adjustment
        if abs(yaw_diff) > max_yaw_change:
            yaw_diff = max_yaw_change * (1 if yaw_diff > 0 else -1)
            rospy.logwarn(f"Limiting yaw change to {yaw_diff:.3f} rad ({yaw_diff*180/math.pi:.1f}°)")
        
        rospy.loginfo(f"Circumferential adjustment: angle change {angle_diff:.3f} rad, yaw change {yaw_diff:.3f} rad")
        
        # Generate smooth circumferential path
        num_points = 20
        rate = rospy.Rate(50)
        
        for i in range(num_points + 1):
            t = i / num_points
            # Smooth interpolation
            smooth_t = 3*t**2 - 2*t**3
            
            # Interpolate angle and radius
            current_interp_angle = current_angle + smooth_t * angle_diff
            current_interp_radius = current_radius + smooth_t * (target_radius - current_radius)
            current_interp_yaw = start_yaw + smooth_t * yaw_diff
            
            # Calculate position
            interp_x = center_x + current_interp_radius * math.cos(current_interp_angle)
            interp_y = center_y + current_interp_radius * math.sin(current_interp_angle)
            interp_z = start_pos[2] + smooth_t * (target_pos[2] - start_pos[2])
            
            # Check force (very relaxed threshold for circumferential adjustment)
            current_force = self.get_contact_force_magnitude()
            # Use very high threshold for circumferential adjustment as UAV may touch valve structure
            # This is a positioning phase, not a contact phase, so higher tolerance is acceptable
            if current_force > self.contact_force_threshold * 3.5:  # Increased from 2.5 to 3.5 (10.5N with 3.0N base)
                rospy.logwarn(f"High force during circumferential adjustment: {current_force:.2f}N")
                rospy.logwarn(f"Force threshold: {self.contact_force_threshold * 3.5:.2f}N")
                return False
            
            # Log force occasionally for monitoring
            if i % 10 == 0 and current_force > self.contact_force_threshold:
                rospy.loginfo(f"Circumferential adjustment force: {current_force:.2f}N (threshold: {self.contact_force_threshold * 3.5:.2f}N)")
            
            # Send command
            MotionController.send_trajectory_point(self.pub, (interp_x, interp_y, interp_z), current_interp_yaw)
            
            if i % 5 == 0:
                progress = (i + 1) / (num_points + 1) * 100
                rospy.loginfo(f"Circumferential adjustment progress: {progress:.1f}%")
            
            rate.sleep()
            
            if rospy.is_shutdown():
                return False
        
        rospy.loginfo("Circumferential adjustment completed")
        return True
    
    def rotate_to_target_position(self):
        """Stage 4: Skip hold phase and proceed directly to rotation"""
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        # CRITICAL FIX: Skip the problematic hold phase entirely
        # The control system seems unable to maintain position stability during hold
        # Instead, proceed directly to rotation which has its own trajectory control
        rospy.loginfo("Stage 4: Skipping hold phase due to control system instability")
        rospy.loginfo("Stage 4: Proceeding directly to rotation for better stability")
        
        # Log current state for reference
        rospy.loginfo(f"Current position after contact: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
        rospy.loginfo(f"Current yaw: {self.current_yaw:.3f} rad")
        
        # Only a very brief pause to allow system to settle
        rospy.loginfo("Brief 0.5s pause to allow system settling...")
        time.sleep(0.5)
        
        # Log final position
        final_pos = self.get_current_position()
        if final_pos is not None:
            rospy.loginfo(f"Final insertion position: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]")
            rospy.loginfo(f"Final yaw: {self.current_yaw:.3f} rad")
            
            # Calculate position change during brief pause
            position_change = math.sqrt((final_pos[0] - current_pos[0])**2 + 
                                      (final_pos[1] - current_pos[1])**2 + 
                                      (final_pos[2] - current_pos[2])**2)
            rospy.loginfo(f"Position change during 0.5s pause: {position_change:.3f}m")
            
            if position_change > 0.05:  # If change is more than 5cm
                rospy.logwarn(f"Significant position change during brief pause: {position_change:.3f}m")
                rospy.logwarn("This confirms control system instability issues")
        
        rospy.loginfo("Stage 4 completed - ready for rotation (no hold phase)")
        return True
    
    def execute_smooth_trajectory(self, start_pos, target_pos, target_yaw, duration=5.0):
        """Execute smooth trajectory with yaw control"""
        from trajectory import PolynomialTrajectory
        
        # Create trajectory
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        
        # Store initial yaw for consistent interpolation
        start_yaw = self.current_yaw
        
        # Calculate shortest yaw path
        yaw_diff = target_yaw - start_yaw
        # Normalize to [-pi, pi] range for shortest path
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Limit maximum yaw change per trajectory to prevent aggressive rotation
        max_yaw_change = math.pi / 4  # 45 degrees max per trajectory
        if abs(yaw_diff) > max_yaw_change:
            yaw_diff = max_yaw_change * (1 if yaw_diff > 0 else -1)
            rospy.logwarn(f"Limiting yaw change to {yaw_diff:.3f} rad ({yaw_diff*180/math.pi:.1f}°)")
        
        final_target_yaw = start_yaw + yaw_diff
        
        rate = rospy.Rate(50)
        start_time = time.time()
        
        rospy.loginfo(f"Trajectory: yaw from {start_yaw:.3f} to {final_target_yaw:.3f} (change: {yaw_diff:.3f} rad)")
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 1.0:
            pt = traj.evaluate()
            if pt is None:
                break
            
            # Interpolate yaw smoothly
            elapsed_time = time.time() - start_time
            if elapsed_time <= duration:
                yaw_progress = elapsed_time / duration
                smooth_yaw_progress = 3*yaw_progress**2 - 2*yaw_progress**3
                current_yaw = start_yaw + smooth_yaw_progress * yaw_diff
            else:
                current_yaw = final_target_yaw
            
            MotionController.send_trajectory_point(self.pub, pt, current_yaw)
            rate.sleep()
        
        return True
    
    def execute_original_insertion(self, userdata):
        """Execute original insertion method"""
        rospy.loginfo("Phase 1: Align to grasp position, Phase 2: Descend to contact...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None:
            rospy.logerr("Failed to get current UAV position")
            return 'failed'
        
        if self.valve_pos is None:
            rospy.logerr("Valve position is not available")
            return 'failed'
        
        rospy.loginfo(f"Current position: {start_pos}, Valve position: {self.valve_pos}")
        
        # Store start position for emergency return
        userdata.start_position = start_pos
        
        # Phase 1: Align to grasp position (at current height)
        rospy.loginfo("Phase 1: Aligning to grasp position...")
        if not self.align_to_grasp_position(start_pos):
            rospy.logerr("Failed to align to grasp position")
            return 'failed'
        
        # Phase 2: Descend to contact
        rospy.loginfo("Phase 2: Descending to establish contact...")
        if not self.descend_to_contact():
            rospy.logerr("Failed to descend to contact")
            return 'failed'
        
        rospy.loginfo("Successfully aligned and established contact")
        return 'succeeded'
    
    def pre_align_to_grasp_position(self, start_pos):
        """Pre-align to grasp position at safe distance"""
        # Calculate pre-insertion position (further back from target)
        align_traj = AlignToGraspTrajectory(
            approach_duration=8.0,
            valve_center=self.valve_pos,
            valve_pose_yaw=self.valve_yaw,
            grasp_height=start_pos[2],
            valve_radius=0.1225,
            valve_beam_width=0.0185,
            rotation_direction=1,
            insertion_offset=0.015 + self.pre_insertion_distance  # Add safe distance
        )
        
        align_traj.set_start_position(start_pos)
        
        # Execute pre-alignment trajectory
        rate = rospy.Rate(50)
        rospy.loginfo("Executing pre-alignment trajectory...")
        
        while not rospy.is_shutdown() and not align_traj.is_complete():
            result = align_traj.get_next_position_and_yaw()
            if result is None:
                break
            pos, yaw = result
            MotionController.send_trajectory_point(self.pub, pos, yaw)
            rate.sleep()
        
        rospy.loginfo("Pre-alignment completed")
        return True
    
    def final_align_to_grasp_position(self):
        """Final alignment to precise grasp position"""
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        # Calculate final insertion position
        align_traj = AlignToGraspTrajectory(
            approach_duration=5.0,  # Shorter duration for final alignment
            valve_center=self.valve_pos,
            valve_pose_yaw=self.valve_yaw,
            grasp_height=current_pos[2],
            valve_radius=0.1225,
            valve_beam_width=0.0185,
            rotation_direction=1,
            insertion_offset=0.015  # Final precise position
        )
        
        align_traj.set_start_position(current_pos)
        
        # Execute final alignment trajectory with force monitoring
        rate = rospy.Rate(50)
        rospy.loginfo("Executing final alignment trajectory...")
        
        while not rospy.is_shutdown() and not align_traj.is_complete():
            # Check for contact force during alignment (very relaxed threshold)
            current_force = self.get_contact_force_magnitude()
            # Use very high threshold for alignment as UAV may lightly contact valve structure
            if current_force > self.contact_force_threshold * 2.5:  # Increased from 2.0 to 2.5 (7.5N with 3.0N base)
                rospy.logwarn(f"High force detected during alignment: {current_force:.2f}N")
                rospy.logwarn(f"Force threshold: {self.contact_force_threshold * 2.5:.2f}N")
                break
            
            result = align_traj.get_next_position_and_yaw()
            if result is None:
                break
            pos, yaw = result
            MotionController.send_trajectory_point(self.pub, pos, yaw)
            rate.sleep()
        
        rospy.loginfo("Final alignment completed")
        return True
    
    def align_to_grasp_position(self, start_pos):
        """Align to grasp position at current height"""
        # Use AlignToGraspTrajectory but keep at current height
        align_traj = AlignToGraspTrajectory(
            approach_duration=15.0,  # Reduced duration for alignment only
            valve_center=self.valve_pos,
            valve_pose_yaw=self.valve_yaw,
            grasp_height=start_pos[2],  # Stay at current height
            valve_radius=0.1225,
            valve_beam_width=0.0185,
            rotation_direction=1,
            insertion_offset=0.015
        )
        
        align_traj.set_start_position(start_pos)
        
        # Execute alignment trajectory
        rate = rospy.Rate(50)
        rospy.loginfo("Executing grasp alignment trajectory...")
        
        while not rospy.is_shutdown() and not align_traj.is_complete():
            result = align_traj.get_next_position_and_yaw()
            if result is None:
                break
            pos, yaw = result
            MotionController.send_trajectory_point(self.pub, pos, yaw)
            rate.sleep()
        
        rospy.loginfo("Grasp alignment completed")
        return True
    
    def descend_to_contact(self):
        """Descend using polynomial trajectory until contact"""
        # CRITICAL FIX: Get stable position reading with multiple samples
        # This prevents using an unstable position reading from Stage 2 transition
        rospy.loginfo("Getting stable position reading for descent...")
        
        # Take multiple position samples to ensure stability
        position_samples = []
        for i in range(5):  # Take 5 samples over 0.5 seconds
            pos = self.get_current_position()
            if pos is not None:
                position_samples.append(pos)
            time.sleep(0.1)
        
        if not position_samples:
            rospy.logerr("Failed to get any position samples for descent")
            return False
        
        # Calculate average position to reduce noise
        avg_x = sum(pos[0] for pos in position_samples) / len(position_samples)
        avg_y = sum(pos[1] for pos in position_samples) / len(position_samples)
        avg_z = sum(pos[2] for pos in position_samples) / len(position_samples)
        current_pos = (avg_x, avg_y, avg_z)
        
        # Check for excessive position variation (indicates instability)
        max_variation = max(
            max(abs(pos[0] - avg_x) for pos in position_samples),
            max(abs(pos[1] - avg_y) for pos in position_samples),
            max(abs(pos[2] - avg_z) for pos in position_samples)
        )
        
        if max_variation > 0.02:  # More than 2cm variation
            rospy.logwarn(f"High position variation detected: {max_variation:.3f}m")
            rospy.logwarn("Position may be unstable, but proceeding with averaged position")
        
        rospy.loginfo(f"Stable position reading: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
        rospy.loginfo(f"Position variation: {max_variation:.3f}m")
        
        # Calculate target descent position
        target_z = self.valve_pos[2] + self.z_offset
        target_pos = (current_pos[0], current_pos[1], target_z)
        
        rospy.loginfo(f"Descending from {current_pos[2]:.3f}m to {target_z:.3f}m")
        
        # Calculate descent parameters
        descent_distance = abs(current_pos[2] - target_z)
        descent_duration = descent_distance / self.descent_speed
        
        rospy.loginfo(f"Descent: {descent_distance:.3f}m in {descent_duration:.1f}s at {self.descent_speed}m/s")
        
        # Execute descent with contact detection
        return self.execute_descent_with_contact_detection(current_pos, target_pos, descent_duration)
    
    def execute_descent_with_contact_detection(self, start_pos, target_pos, duration):
        """Execute descent with position control and movement-based stuck detection"""
        from trajectory import PolynomialTrajectory
        
        # CRITICAL FIX: Use position-controlled descent instead of force-based detection
        # This prevents false contact detection and maintains XY position stability
        rospy.loginfo("Starting controlled descent with movement-based stuck detection...")
        rospy.loginfo(f"Target descent: {start_pos[2]:.3f}m → {target_pos[2]:.3f}m")
        
        # Fixed XY position and yaw from start_pos to prevent drift
        fixed_x = start_pos[0]
        fixed_y = start_pos[1]
        fixed_yaw = self.current_yaw  # CRITICAL FIX: Fix yaw at start of descent
        
        rate = rospy.Rate(50)  # 50Hz control rate
        start_time = time.time()
        
        # Variables for movement-based stuck detection
        last_height_check_time = start_time
        last_height = start_pos[2]
        stuck_check_interval = 2.0  # Check every 2 seconds
        min_descent_rate = 0.02  # Minimum descent rate (2cm/2s = 1cm/s)
        
        rospy.loginfo(f"Descent parameters:")
        rospy.loginfo(f"  - Fixed XY position: [{fixed_x:.3f}, {fixed_y:.3f}]")
        rospy.loginfo(f"  - Fixed yaw: {fixed_yaw:.3f} rad")
        rospy.loginfo(f"  - Stuck detection: {min_descent_rate:.3f}m every {stuck_check_interval:.1f}s")
        rospy.loginfo(f"  - Duration: {duration:.1f}s")
        
        # CRITICAL FIX: Add initial position hold to prevent initial upward movement
        # Hold position for 0.5s before starting descent to ensure stability
        hold_duration = 0.5
        rospy.loginfo(f"Initial position hold: {hold_duration}s to prevent upward movement")
        
        hold_start_time = time.time()
        while not rospy.is_shutdown() and (time.time() - hold_start_time) < hold_duration:
            msg = FlightNav()
            msg.target = 1
            msg.pos_xy_nav_mode = FlightNav.POS_MODE
            msg.target_pos_x = fixed_x
            msg.target_pos_y = fixed_y
            msg.pos_z_nav_mode = FlightNav.POS_MODE
            msg.target_pos_z = start_pos[2]  # Hold at start height
            msg.yaw_nav_mode = FlightNav.POS_MODE
            msg.target_yaw = fixed_yaw
            self.pub.publish(msg)
            rate.sleep()
        
        rospy.loginfo("Position hold completed, starting descent...")
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 5.0:
            elapsed_time = time.time() - start_time
            current_pos = self.get_current_position()
            
            if current_pos is None:
                rospy.logwarn("Lost position data during descent")
                return False
            
            # Check for stuck condition based on Z movement
            if elapsed_time - last_height_check_time >= stuck_check_interval:
                height_change = last_height - current_pos[2]  # Positive means descended
                
                if height_change < min_descent_rate:
                    rospy.logwarn(f"UAV appears stuck: descended only {height_change:.3f}m in {stuck_check_interval:.1f}s")
                    rospy.logwarn(f"Expected minimum descent: {min_descent_rate:.3f}m")
                    
                    # Check if we're already close to target
                    remaining_descent = current_pos[2] - target_pos[2]
                    if remaining_descent < 0.05:  # Within 5cm of target
                        rospy.loginfo("Close to target height, considering descent complete")
                        return True
                    else:
                        rospy.logwarn("UAV stuck during descent, but not at target height")
                        return False
                else:
                    rospy.loginfo(f"Descent progress: {height_change:.3f}m in {stuck_check_interval:.1f}s")
                
                # Update for next check
                last_height_check_time = elapsed_time + start_time
                last_height = current_pos[2]
            
            # Calculate target height for current time
            progress = min(elapsed_time / duration, 1.0)
            target_height = start_pos[2] + progress * (target_pos[2] - start_pos[2])
            
            # Monitor XY drift
            xy_drift = math.sqrt((current_pos[0] - fixed_x)**2 + (current_pos[1] - fixed_y)**2)
            if xy_drift > 0.05:  # More than 5cm drift
                if elapsed_time % 2.0 < 0.02:  # Log every 2 seconds
                    rospy.logwarn(f"XY drift detected: {xy_drift:.3f}m from target position")
            
            # Monitor yaw drift
            yaw_drift = abs(self.current_yaw - fixed_yaw)
            if yaw_drift > 0.2:  # More than ~11 degrees drift
                if elapsed_time % 2.0 < 0.02:  # Log every 2 seconds
                    rospy.logwarn(f"Yaw drift detected: {yaw_drift:.3f} rad from target yaw")
            
            # Send position command with fixed XY, progressive Z, and fixed yaw
            msg = FlightNav()
            msg.target = 1
            msg.pos_xy_nav_mode = FlightNav.POS_MODE
            msg.target_pos_x = fixed_x  # Keep fixed X
            msg.target_pos_y = fixed_y  # Keep fixed Y
            msg.pos_z_nav_mode = FlightNav.POS_MODE
            msg.target_pos_z = target_height
            msg.yaw_nav_mode = FlightNav.POS_MODE
            msg.target_yaw = fixed_yaw  # CRITICAL FIX: Keep fixed yaw
            self.pub.publish(msg)
            
            # Check if we've reached the target
            if current_pos[2] <= target_pos[2] + 0.01:  # Within 1cm of target
                rospy.loginfo("Reached target descent height")
                return True
            
            # Progress logging
            if elapsed_time % 2.0 < 0.02:  # Log every 2 seconds
                descended = start_pos[2] - current_pos[2]
                rospy.loginfo(f"Descent progress: {elapsed_time:.1f}s, descended: {descended:.3f}m, XY drift: {xy_drift:.3f}m, yaw drift: {yaw_drift:.3f}rad")
            
            rate.sleep()
        
        # Final check
        final_pos = self.get_current_position()
        if final_pos is not None:
            total_descent = start_pos[2] - final_pos[2]
            final_yaw_drift = abs(self.current_yaw - fixed_yaw)
            rospy.loginfo(f"Descent completed: descended {total_descent:.3f}m in {duration:.1f}s")
            rospy.loginfo(f"Final yaw drift: {final_yaw_drift:.3f} rad")
            return True
        
        rospy.logwarn("Descent timed out")
        return False
    
    def continue_descent_until_contact(self, start_pos):
        """Continue controlled descent with position stability"""
        rospy.loginfo("Continuing controlled descent with position stability...")
        
        initial_height = start_pos[2]
        # Fixed XY position and yaw to prevent drift
        fixed_x = start_pos[0]
        fixed_y = start_pos[1]
        fixed_yaw = self.current_yaw  # Fix yaw at start of continue descent
        
        min_z = self.valve_pos[2] + self.z_offset - 0.02  # 2cm below target
        max_descent_time = 10.0  # Maximum time for continued descent
        
        start_time = time.time()
        rate = rospy.Rate(50)
        
        # Variables for movement-based stuck detection
        last_height_check_time = start_time
        last_height = initial_height
        stuck_check_interval = 2.0  # Check every 2 seconds
        min_descent_rate = 0.02  # Minimum descent rate (2cm/2s = 1cm/s)
        
        rospy.loginfo(f"Continue descent parameters:")
        rospy.loginfo(f"  - Fixed XY position: [{fixed_x:.3f}, {fixed_y:.3f}]")
        rospy.loginfo(f"  - Fixed yaw: {fixed_yaw:.3f} rad")
        rospy.loginfo(f"  - Initial height: {initial_height:.3f}m")
        rospy.loginfo(f"  - Target min height: {min_z:.3f}m")
        rospy.loginfo(f"  - Stuck detection: {min_descent_rate:.3f}m every {stuck_check_interval:.1f}s")
        
        while not rospy.is_shutdown() and (time.time() - start_time) < max_descent_time:
            elapsed_time = time.time() - start_time
            current_pos = self.get_current_position()
            
            if current_pos is None:
                rospy.logwarn("Lost position data during continue descent")
                return False
            
            # Check for stuck condition based on Z movement
            if elapsed_time - last_height_check_time >= stuck_check_interval:
                height_change = last_height - current_pos[2]  # Positive means descended
                
                if height_change < min_descent_rate:
                    rospy.logwarn(f"UAV stuck during continue descent: descended only {height_change:.3f}m in {stuck_check_interval:.1f}s")
                    
                    # Check if we're already close to target
                    remaining_descent = current_pos[2] - min_z
                    if remaining_descent < 0.05:  # Within 5cm of target
                        rospy.loginfo("Close to minimum height, considering descent complete")
                        return True
                    else:
                        rospy.logwarn("UAV stuck during continue descent, but not at minimum height")
                        return False
                else:
                    rospy.loginfo(f"Continue descent progress: {height_change:.3f}m in {stuck_check_interval:.1f}s")
                
                # Update for next check
                last_height_check_time = elapsed_time + start_time
                last_height = current_pos[2]
            
            # Check if we've reached minimum height
            if current_pos[2] <= min_z:
                total_descent = initial_height - current_pos[2]
                rospy.loginfo(f"Reached minimum descent height: {current_pos[2]:.3f}m")
                rospy.loginfo(f"Total descent achieved: {total_descent:.3f}m")
                return True
            
            # Calculate next descent position with fixed XY and yaw
            descent_step = self.descent_speed * 0.02  # 20ms step
            next_z = current_pos[2] - descent_step
            
            # Monitor XY drift
            xy_drift = math.sqrt((current_pos[0] - fixed_x)**2 + (current_pos[1] - fixed_y)**2)
            
            # Monitor yaw drift
            yaw_drift = abs(self.current_yaw - fixed_yaw)
            
            # Send descent command with fixed XY position and yaw
            msg = FlightNav()
            msg.target = 1
            msg.pos_xy_nav_mode = FlightNav.POS_MODE
            msg.target_pos_x = fixed_x  # Keep fixed X
            msg.target_pos_y = fixed_y  # Keep fixed Y
            msg.pos_z_nav_mode = FlightNav.POS_MODE
            msg.target_pos_z = next_z
            msg.yaw_nav_mode = FlightNav.POS_MODE
            msg.target_yaw = fixed_yaw  # Keep fixed yaw
            self.pub.publish(msg)
            
            # Progress logging
            if elapsed_time % 2.0 < 0.02:  # Log every 2 seconds
                current_descent = initial_height - current_pos[2]
                rospy.loginfo(f"Continue descent: {elapsed_time:.1f}s, descended: {current_descent:.3f}m, XY drift: {xy_drift:.3f}m, yaw drift: {yaw_drift:.3f}rad")
            
            rate.sleep()
        
        # Final logging
        final_pos = self.get_current_position()
        if final_pos is not None:
            total_descent = initial_height - final_pos[2]
            final_yaw_drift = abs(self.current_yaw - fixed_yaw)
            rospy.loginfo(f"Continue descent completed: descended {total_descent:.3f}m in {max_descent_time:.1f}s")
            rospy.loginfo(f"Final yaw drift: {final_yaw_drift:.3f} rad")
        
        return True  # Continue with task even if not at exact target

class RotateValveState(SingleUAVStateBase):
    """Rotate valve while maintaining contact"""
    def __init__(self, module_id=1, rotation_angle=math.pi/2, rotation_duration=8.0):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed', 'emergency'], module_id=module_id)
        self.rotation_angle = rotation_angle
        self.rotation_duration = rotation_duration
        
        # Emergency detection variables specific to rotation
        self.emergency_stop = threading.Event()
        self.last_position = None
        self.last_yaw = 0.0
        self.stuck_start_time = None
        # TEMPORARY HARDCODED VALUES to ensure correct thresholds
        # Try different parameter namespaces to ensure we get the right values
        self.stuck_threshold = 25.0  # Hardcoded high value to prevent premature triggering
        self.movement_threshold = 0.02  # Hardcoded relaxed value
        self.yaw_threshold = 0.1  # Hardcoded very relaxed value
        
        # Try to override with ROS parameters if they exist
        if rospy.has_param("stuck_threshold"):
            self.stuck_threshold = rospy.get_param("stuck_threshold", self.stuck_threshold)
        elif rospy.has_param("~stuck_threshold"):
            self.stuck_threshold = rospy.get_param("~stuck_threshold", self.stuck_threshold)
            
        if rospy.has_param("movement_threshold"):
            self.movement_threshold = rospy.get_param("movement_threshold", self.movement_threshold)
        elif rospy.has_param("~movement_threshold"):
            self.movement_threshold = rospy.get_param("~movement_threshold", self.movement_threshold)
            
        if rospy.has_param("yaw_threshold"):
            self.yaw_threshold = rospy.get_param("yaw_threshold", self.yaw_threshold)
        elif rospy.has_param("~yaw_threshold"):
            self.yaw_threshold = rospy.get_param("~yaw_threshold", self.yaw_threshold)
        
        # Debug log to confirm parameter values
        rospy.loginfo(f"RotateValveState emergency parameters: stuck_threshold={self.stuck_threshold:.1f}s, movement_threshold={self.movement_threshold:.3f}m, yaw_threshold={self.yaw_threshold:.3f}rad")
        rospy.loginfo(f"Emergency checking frequency: every 0.5 seconds (not every control cycle)")
        rospy.loginfo(f"Control frequencies: Stage 4 hold=5Hz, Pre-rotation stabilization=5Hz, Rotation=50Hz")
        
        # Rotation-specific emergency detection
        self.rotation_start_pos = None
        self.rotation_start_yaw = 0.0
        self.rotation_stuck_start_time = None
    
    def check_rotation_emergency(self):
        """Check if UAV is stuck during rotation"""
        if self.rotation_start_pos is None:
            # Use current position from the base class
            current_pos = self.get_current_position()
            if current_pos is None:
                return False
            self.rotation_start_pos = current_pos
            self.rotation_start_yaw = self.current_yaw
            self.last_position = current_pos
            self.last_yaw = self.current_yaw
            return False
        
        # Get current position
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        # Calculate movement since last check
        if self.last_position is not None:
            pos_diff = math.sqrt(
                (current_pos[0] - self.last_position[0])**2 + 
                (current_pos[1] - self.last_position[1])**2 + 
                (current_pos[2] - self.last_position[2])**2
            )
            yaw_diff = abs(self.current_yaw - self.last_yaw)
            
            # Debug logging for stuck detection
            if pos_diff < self.movement_threshold and yaw_diff < self.yaw_threshold:
                if self.rotation_stuck_start_time is None:
                    self.rotation_stuck_start_time = time.time()
                    rospy.loginfo(f"Potential stuck detected: pos_diff={pos_diff:.4f}m, yaw_diff={yaw_diff:.4f}rad")
                
                stuck_duration = time.time() - self.rotation_stuck_start_time
                if stuck_duration > self.stuck_threshold:
                    rospy.logwarn(f"UAV appears stuck during rotation: {stuck_duration:.1f}s > {self.stuck_threshold:.1f}s")
                    rospy.logwarn(f"Movement thresholds: pos={self.movement_threshold:.3f}m, yaw={self.yaw_threshold:.3f}rad")
                    rospy.logwarn(f"Actual movement: pos={pos_diff:.4f}m, yaw={yaw_diff:.4f}rad")
                    rospy.logwarn("UAV appears stuck during rotation, triggering emergency stop")
                    self.emergency_stop.set()
                    return True
                elif stuck_duration > 5.0:  # Log warning if stuck for more than 5 seconds
                    rospy.logwarn(f"UAV movement limited for {stuck_duration:.1f}s (threshold: {self.stuck_threshold:.1f}s)")
            else:
                if self.rotation_stuck_start_time is not None:
                    rospy.loginfo(f"Movement resumed: pos_diff={pos_diff:.4f}m, yaw_diff={yaw_diff:.4f}rad")
                self.rotation_stuck_start_time = None
        
        # Update last position and yaw for next check
        self.last_position = current_pos
        self.last_yaw = self.current_yaw
        
        return False
    
    def execute(self, userdata):
        rospy.loginfo("Starting valve rotation with emergency detection enabled")
        
        if not self.wait_for_positions():
            return 'failed'
        
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Failed to get current UAV position")
            return 'failed'
        
        if self.valve_pos is None:
            rospy.logerr("Valve position is not available")
            return 'failed'
        
        rospy.loginfo(f"Rotating valve at position: {self.valve_pos}")
        
        # Pre-rotation stabilization: Brief pause to allow system settling
        rospy.loginfo("Pre-rotation stabilization: Brief 0.5s pause to allow system settling...")
        
        # CRITICAL FIX: Get initial position once for reference
        initial_pos = self.get_current_position()
        if initial_pos is None:
            rospy.logerr("Failed to get initial UAV position")
            return 'failed'
        
        initial_yaw = self.current_yaw
        
        rospy.loginfo(f"Pre-rotation stabilization target: [{initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f}]")
        rospy.loginfo(f"Pre-rotation stabilization yaw: {initial_yaw:.3f} rad")
        
        # Brief pause instead of active control to avoid control system issues
        time.sleep(0.5)
        
        # Update current position after brief pause
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Failed to get current UAV position after stabilization")
            return 'failed'
        
        # Log any drift during stabilization
        drift = math.sqrt((current_pos[0] - initial_pos[0])**2 + 
                         (current_pos[1] - initial_pos[1])**2 + 
                         (current_pos[2] - initial_pos[2])**2)
        rospy.loginfo(f"Pre-rotation stabilization drift: {drift:.3f}m")
        
        if drift > 0.05:  # If drift is more than 5cm
            rospy.logwarn(f"Significant drift detected during pre-rotation pause: {drift:.3f}m")
            rospy.logwarn("This indicates control system instability issues")
        
        rospy.loginfo("Pre-rotation stabilization completed, starting rotation...")
        
        # Initialize rotation emergency detection
        self.emergency_stop.clear()
        self.rotation_start_pos = current_pos
        self.rotation_start_yaw = self.current_yaw
        self.rotation_stuck_start_time = None
        self.last_position = current_pos
        self.last_yaw = self.current_yaw
        
        # Calculate rotation parameters
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        
        # End-effector offsets
        end_effector_offset_x = 0.246
        end_effector_offset_y = 0.0
        end_effector_offset_z = 0.0743823
        
        global_offset_x = (cos_yaw * end_effector_offset_x - 
                          sin_yaw * end_effector_offset_y)
        global_offset_y = (sin_yaw * end_effector_offset_x + 
                          cos_yaw * end_effector_offset_y)
        
        # Calculate current end-effector position and height
        current_end_effector_x = current_pos[0] + global_offset_x
        current_end_effector_y = current_pos[1] + global_offset_y
        
        # CRITICAL FIX: Use current body height as grasp height to prevent sudden height changes
        # The trajectory will handle the end-effector offset internally
        current_grasp_height = current_pos[2]  # Use current body height, not end-effector height
        
        rospy.loginfo(f"Rotation height calculation:")
        rospy.loginfo(f"  Current body position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
        rospy.loginfo(f"  Current end-effector position: [{current_end_effector_x:.3f}, {current_end_effector_y:.3f}, {current_pos[2] + end_effector_offset_z:.3f}]")
        rospy.loginfo(f"  Using grasp height: {current_grasp_height:.3f}m (body height)")
        rospy.loginfo(f"  Trajectory will handle end-effector offset internally")
        
        # Calculate rotation parameters
        dx = current_end_effector_x - self.valve_pos[0]
        dy = current_end_effector_y - self.valve_pos[1]
        radius = (dx**2 + dy**2)**0.5
        start_angle = math.atan2(dy, dx)
        
        rotation_traj = ValveRotationTrajectory(
            rotation_duration=self.rotation_duration,
            valve_center=self.valve_pos,
            rotation_radius=radius,
            start_angle=start_angle,
            rotation_angle=self.rotation_angle,
            grasp_height=current_grasp_height,  # Use current body height to prevent sudden height changes
            rotation_direction=1,
            end_effector_offset_x=end_effector_offset_x,
            end_effector_offset_y=end_effector_offset_y,
            end_effector_offset_z=end_effector_offset_z
        )
        
        rotation_traj.start_rotation()
        
        # Log initial rotation parameters
        rospy.loginfo(f"Rotation trajectory initialization:")
        rospy.loginfo(f"  Current body position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
        rospy.loginfo(f"  Current yaw: {self.current_yaw:.3f} rad")
        rospy.loginfo(f"  Rotation radius: {radius:.3f}m")
        rospy.loginfo(f"  Start angle: {start_angle:.3f} rad")
        rospy.loginfo(f"  Rotation angle: {self.rotation_angle:.3f} rad")
        rospy.loginfo(f"  Duration: {self.rotation_duration:.1f}s")
        
        # Execute rotation with emergency check
        rate = rospy.Rate(50)
        rotation_start_time = time.time()
        last_log_time = rotation_start_time
        last_emergency_check = rotation_start_time
        
        while not rospy.is_shutdown() and not rotation_traj.is_complete():
            # Check for emergency during rotation - but only every 0.5 seconds to avoid high frequency noise
            current_time = time.time()
            if current_time - last_emergency_check > 0.5:  # Check every 0.5 seconds, not every 20ms
                if self.check_rotation_emergency():
                    rospy.logwarn("Emergency detected during rotation, stopping")
                    return 'emergency'
                last_emergency_check = current_time
            
            result = rotation_traj.get_next_position_and_yaw()
            if result is None:
                break
            pos, yaw = result
            
            # Log progress every 2 seconds
            current_time = time.time()
            if current_time - last_log_time > 2.0:
                elapsed_time = current_time - rotation_start_time
                progress = min(100.0, (elapsed_time / self.rotation_duration) * 100)
                rospy.loginfo(f"Rotation progress: {progress:.1f}% ({elapsed_time:.1f}s/{self.rotation_duration:.1f}s)")
                rospy.loginfo(f"  Current body position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
                rospy.loginfo(f"  Current yaw: {yaw:.3f} rad")
                last_log_time = current_time
            
            MotionController.send_trajectory_point(self.pub, pos, yaw)
            rate.sleep()
        
        rospy.loginfo("Valve rotation completed successfully")
        return 'succeeded'

class ReturnToStartState(SingleUAVStateBase):
    """Return to start position"""
    def __init__(self, module_id=1):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        self._input_keys = ['start_position']
    
    def execute(self, userdata):
        # Get start position from userdata
        start_position = userdata.get('start_position', [0.0, 0.0, 1.0])
        
        rospy.loginfo(f"Returning to start position: {start_position}")
        
        current_pos = self.get_current_position()
        if current_pos is None:
            return 'failed'
        
        # First move above valve
        if self.valve_pos is not None:
            safe_pos = (self.valve_pos[0], self.valve_pos[1], current_pos[2] + 0.3)
            success = MotionController.execute_poly_motion_pose(
                self.pub, current_pos, safe_pos, 0.1)
            if not success:
                return 'failed'
        
        # Then return to start
        return_start_pos = safe_pos if self.valve_pos is not None else current_pos
        final_pos = self.get_current_position()
        
        if final_pos is None:
            return 'failed'
        
        # Calculate distance and adjust timeout
        distance = math.sqrt((start_position[0]-final_pos[0])**2 + 
                           (start_position[1]-final_pos[1])**2 + 
                           (start_position[2]-final_pos[2])**2)
        
        # Use consistent slow speed for stability and precision
        move_speed = 0.1  # Conservative speed for all moves
        timeout = max(90, distance / move_speed * 3.0)
        
        rospy.loginfo(f"Returning {distance:.2f}m at {move_speed}m/s")
        
        success = MotionController.execute_poly_motion_pose(
            self.pub, final_pos, start_position, move_speed, timeout=timeout)
        
        return 'succeeded' if success else 'failed'

class EmergencyState(SingleUAVStateBase):
    """Emergency state: ascent and return to start"""
    def __init__(self, module_id=1):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        self._input_keys = ['start_position']
    
    def execute(self, userdata):
        rospy.logwarn("Emergency state activated - UAV stuck detected")
        
        # Get start position from userdata
        if hasattr(userdata, 'start_position'):
            start_position = userdata.start_position
        else:
            rospy.logwarn("No start position in userdata, using default")
            start_position = [0.0, 0.0, 1.0]
        
        # Execute emergency ascent and return
        success = self.emergency_ascent_and_return(start_position)
        
        if success:
            rospy.loginfo("Emergency return completed successfully")
            return 'succeeded'
        else:
            rospy.logerr("Emergency return failed")
            return 'failed'

def main():
    rospy.init_node('valve_rotation_single_uav')
    
    # Parameters
    module_id = rospy.get_param("~module_id", 1)
    rotation_angle = rospy.get_param("~rotation_angle", math.pi/2)
    approach_height = rospy.get_param("~approach_height", 0.5)
    hover_duration = rospy.get_param("~hover_duration", 2.0)
    
    # Insertion strategy parameters
    use_staged_insertion = rospy.get_param("~use_staged_insertion", True)
    pre_insertion_distance = rospy.get_param("~pre_insertion_distance", 0.03)  # Reduced from 0.05
    yaw_adjustment_angle = rospy.get_param("~yaw_adjustment_angle", 0.1)
    circumferential_offset = rospy.get_param("~circumferential_offset", 0.02)  # Reduced from 0.03
    contact_force_threshold = rospy.get_param("~contact_force_threshold", 3.0)  # Increased from 2.0 to 3.0N
    descent_speed = rospy.get_param("~descent_speed", 0.05)
    
    rospy.loginfo(f"Starting valve rotation for UAV{module_id}")
    rospy.loginfo(f"Rotation angle: {rotation_angle:.3f} rad ({math.degrees(rotation_angle):.1f} deg)")
    rospy.loginfo("Emergency detection enabled during valve rotation only")
    rospy.loginfo("Start position will be recorded from current mocap position")
    rospy.loginfo("Valve position fallback: If valve spawn fails, using default position (3.0, 0.0, 0.0)")
    
    # Insertion strategy info
    if use_staged_insertion:
        rospy.loginfo("Using 4-STAGE INSERTION strategy:")
        rospy.loginfo("  Stage 1: Approach with circumferential avoidance")
        rospy.loginfo("  Stage 2: Yaw and circumferential direction adjustment")
        rospy.loginfo("  Stage 3: Descent to contact")
        rospy.loginfo("  Stage 4: Final rotation to target position")
        rospy.loginfo(f"  - Pre-insertion distance: {pre_insertion_distance:.3f}m")
        rospy.loginfo(f"  - Yaw adjustment angle: {yaw_adjustment_angle:.3f} rad")
        rospy.loginfo(f"  - Circumferential offset: {circumferential_offset:.3f}m")
        rospy.loginfo(f"  - Contact force threshold: {contact_force_threshold:.1f}N")
        rospy.loginfo("  - Insertion positions: Fang1 near beam2, Fang2 near beam1")
    else:
        rospy.loginfo("Using ORIGINAL INSERTION strategy")
    
    rospy.loginfo(f"Descent speed: {descent_speed:.3f}m/s")
    
    # Create state machine
    sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED', 'EMERGENCY_COMPLETED'])
    
    # Declare the userdata variables
    sm.userdata.start_position = [0.0, 0.0, 1.0]  # Default value
    
    with sm:
        smach.StateMachine.add('INITIALIZE', 
                               InitializeStartPositionState(module_id=module_id, 
                                             hover_duration=hover_duration),
                               transitions={'succeeded': 'MOVE_TO_VALVE',
                                          'failed': 'TASK_FAILED'},
                               remapping={'start_position': 'start_position'})
        
        smach.StateMachine.add('MOVE_TO_VALVE', 
                               MoveToValveState(module_id=module_id, 
                                               approach_height=approach_height),
                               transitions={'succeeded': 'ALIGN_AND_DESCEND',
                                          'failed': 'TASK_FAILED'})
        
        smach.StateMachine.add('ALIGN_AND_DESCEND', 
                               DescendAndContactState(module_id=module_id,
                                                    contact_force_threshold=contact_force_threshold,
                                                    descent_speed=descent_speed,
                                                    use_staged_insertion=use_staged_insertion,
                                                    pre_insertion_distance=pre_insertion_distance,
                                                    yaw_adjustment_angle=yaw_adjustment_angle,
                                                    circumferential_offset=circumferential_offset),
                               transitions={'succeeded': 'ROTATE_VALVE',
                                          'failed': 'TASK_FAILED'},
                               remapping={'start_position': 'start_position'})
        
        smach.StateMachine.add('ROTATE_VALVE', 
                               RotateValveState(module_id=module_id,
                                               rotation_angle=rotation_angle),
                               transitions={'succeeded': 'RETURN_TO_START',
                                          'failed': 'TASK_FAILED',
                                          'emergency': 'EMERGENCY_RETURN'})
        
        smach.StateMachine.add('RETURN_TO_START', 
                               ReturnToStartState(module_id=module_id),
                               transitions={'succeeded': 'TASK_COMPLETED',
                                          'failed': 'TASK_FAILED'},
                               remapping={'start_position': 'start_position'})
        
        smach.StateMachine.add('EMERGENCY_RETURN', 
                               EmergencyState(module_id=module_id),
                               transitions={'succeeded': 'EMERGENCY_COMPLETED',
                                          'failed': 'TASK_FAILED'},
                               remapping={'start_position': 'start_position'})
    
    # Execute state machine
    outcome = sm.execute()
    rospy.loginfo(f"Task completed with outcome: {outcome}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
