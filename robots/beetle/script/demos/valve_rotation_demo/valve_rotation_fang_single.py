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
    """Align to grasp position, then descend to establish contact"""
    def __init__(self, module_id=1, contact_force_threshold=2.0, descent_speed=0.05):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        self.contact_force_threshold = contact_force_threshold
        self.descent_speed = descent_speed
        self.z_offset = 0 if rospy.get_param("~simulation", True) else 0.21
        self._input_keys = ['start_position']
        self._output_keys = ['start_position']
    
    def execute(self, userdata):
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
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
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
        """Execute descent using polynomial trajectory with contact detection"""
        from trajectory import PolynomialTrajectory
        
        # Create polynomial trajectory for descent
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        
        rate = rospy.Rate(50)  # 50Hz control rate
        start_time = time.time()
        
        rospy.loginfo("Starting descent with contact detection...")
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 5.0:
            # Check for contact force
            force_mag = self.get_contact_force_magnitude()
            if force_mag > self.contact_force_threshold:
                rospy.loginfo(f"Contact established during descent, force: {force_mag:.2f}N")
                return True
            
            # Get next trajectory point
            pt = traj.evaluate()
            if pt is None:
                # Trajectory completed, check if we need to continue descent for contact
                rospy.loginfo("Descent trajectory completed, checking for contact...")
                current_pos = self.get_current_position()
                if current_pos is None:
                    return False
                
                # Continue slow descent until contact or minimum height reached
                return self.continue_descent_until_contact(current_pos)
            
            # Send trajectory command (preserve current yaw)
            current_pos = self.get_current_position()
            if current_pos is None:
                return False
                
            msg = FlightNav()
            msg.target = 1
            msg.pos_xy_nav_mode = FlightNav.POS_MODE
            msg.target_pos_x = pt[0]
            msg.target_pos_y = pt[1]
            msg.pos_z_nav_mode = FlightNav.POS_MODE
            msg.target_pos_z = pt[2]
            msg.yaw_nav_mode = FlightNav.POS_MODE
            msg.target_yaw = self.current_yaw  # Preserve current yaw
            self.pub.publish(msg)
            
            rate.sleep()
        
        rospy.logwarn("Descent timed out without establishing contact")
        return False
    
    def continue_descent_until_contact(self, current_pos):
        """Continue slow descent until contact is established"""
        rospy.loginfo("Continuing slow descent until contact...")
        
        min_z = self.valve_pos[2] + self.z_offset - 0.02  # 2cm below target
        max_descent_time = 10.0  # Maximum time for final descent
        
        start_time = time.time()
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown() and (time.time() - start_time) < max_descent_time:
            # Check for contact
            force_mag = self.get_contact_force_magnitude()
            if force_mag > self.contact_force_threshold:
                rospy.loginfo(f"Contact established, force: {force_mag:.2f}N")
                return True
            
            # Get current position
            current_pos = self.get_current_position()
            if current_pos is None:
                return False
            
            # Check if we've reached minimum height
            if current_pos[2] <= min_z:
                rospy.loginfo("Reached minimum descent height")
                return True
            
            # Calculate next descent position
            descent_step = self.descent_speed * 0.02  # 20ms step
            next_z = current_pos[2] - descent_step
            
            # Send descent command (preserve yaw)
            msg = FlightNav()
            msg.target = 1
            msg.pos_xy_nav_mode = FlightNav.POS_MODE
            msg.target_pos_x = current_pos[0]
            msg.target_pos_y = current_pos[1]
            msg.pos_z_nav_mode = FlightNav.POS_MODE
            msg.target_pos_z = next_z
            msg.yaw_nav_mode = FlightNav.POS_MODE
            msg.target_yaw = self.current_yaw  # Preserve current yaw
            self.pub.publish(msg)
            
            rate.sleep()
        
        rospy.logwarn("Final descent completed without strong contact detection")
        return True  # Continue with task even if contact force is not strong

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
        self.stuck_threshold = rospy.get_param("~stuck_threshold", 3.0)  # seconds
        self.movement_threshold = rospy.get_param("~movement_threshold", 0.01)  # meters
        self.yaw_threshold = rospy.get_param("~yaw_threshold", 0.05)  # radians
        
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
            
            if pos_diff < self.movement_threshold and yaw_diff < self.yaw_threshold:
                if self.rotation_stuck_start_time is None:
                    self.rotation_stuck_start_time = time.time()
                elif time.time() - self.rotation_stuck_start_time > self.stuck_threshold:
                    rospy.logwarn("UAV appears stuck during rotation, triggering emergency stop")
                    self.emergency_stop.set()
                    return True
            else:
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
        
        current_end_effector_x = current_pos[0] + global_offset_x
        current_end_effector_y = current_pos[1] + global_offset_y
        
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
            grasp_height=current_pos[2],
            rotation_direction=1,
            end_effector_offset_x=end_effector_offset_x,
            end_effector_offset_y=end_effector_offset_y,
            end_effector_offset_z=end_effector_offset_z
        )
        
        rotation_traj.start_rotation()
        
        # Execute rotation with emergency check
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and not rotation_traj.is_complete():
            # Check for emergency during rotation
            if self.check_rotation_emergency():
                rospy.logwarn("Emergency detected during rotation, stopping")
                return 'emergency'
            
            result = rotation_traj.get_next_position_and_yaw()
            if result is None:
                break
            pos, yaw = result
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
        start_position = userdata.get('start_position', [0.0, 0.0, 1.0])
        
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
    
    rospy.loginfo(f"Starting valve rotation for UAV{module_id}")
    rospy.loginfo(f"Rotation angle: {rotation_angle:.3f} rad ({math.degrees(rotation_angle):.1f} deg)")
    rospy.loginfo("Emergency detection enabled during valve rotation only")
    rospy.loginfo("Start position will be recorded from current mocap position")
    rospy.loginfo("Valve position fallback: If valve spawn fails, using default position (3.0, 0.0, 0.0)")
    rospy.loginfo("This addresses valve spawn conflicts and duplicate spawn issues")
    rospy.loginfo("Corrected logic: 1) Move above valve center, 2) Align to grasp at safe height, 3) Descend to contact")
    
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
                               DescendAndContactState(module_id=module_id),
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
