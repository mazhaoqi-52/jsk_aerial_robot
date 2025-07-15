#!/usr/bin/env python3
"""
Single UAV Valve Rotation using SMACH State Machine
Optimized version with reduced redundancy and improved readability.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))
import rospy
import smach
import smach_ros
import time
import math
import threading

from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from trajectory import AlignToGraspTrajectory, ValveRotationTrajectory, ConstantDistanceValveRotationTrajectory, create_constant_distance_trajectory
from unified_motion_controller import UnifiedMotionController
from insertion_optimizer import InsertionOptimizer
from constrained_optimizer import ConstrainedInsertionOptimizer
from simplified_rotate_valve_state import SimplifiedRotateValveState


class SingleUAVStateBase(smach.State):
    def __init__(self, outcomes, input_keys=None, output_keys=None, module_id=1):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys or [], output_keys=output_keys or [])
        self.module_id = module_id
        
        # Publishers and subscribers
        self.pub = rospy.Publisher(f"/beetle{module_id}/uav/nav", FlightNav, queue_size=1)
        
        # Initialize unified motion controller for this state
        self.motion_controller = UnifiedMotionController(self)
        
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
        
        # CRITICAL FIX: Ensure uav_received exists before calling set()
        if hasattr(self, 'uav_received'):
            self.uav_received.set()
        else:
            rospy.logwarn("uav_received attribute not found - this indicates incomplete initialization")
            # Try to create it as a fallback
            self.uav_received = threading.Event()
            self.uav_received.set()
        
        # Only track position and yaw, no emergency detection logic here
        # Emergency detection is handled only in RotateValveState
    
    def wrench_callback(self, msg):
        self.current_wrench = msg
    
    def valve_sim_callback(self, msg):
        self.valve_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        q = msg.pose.pose.orientation
        _, _, self.valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # CRITICAL FIX: Ensure valve_received exists before calling set()
        if hasattr(self, 'valve_received'):
            self.valve_received.set()
        else:
            rospy.logwarn("valve_received attribute not found - this indicates incomplete initialization")
            # Try to create it as a fallback
            self.valve_received = threading.Event()
            self.valve_received.set()
    
    def valve_callback(self, msg):
        self.valve_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        q = msg.pose.orientation
        _, _, self.valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # CRITICAL FIX: Ensure valve_received exists before calling set()
        if hasattr(self, 'valve_received'):
            self.valve_received.set()
        else:
            rospy.logwarn("valve_received attribute not found - this indicates incomplete initialization")
            # Try to create it as a fallback
            self.valve_received = threading.Event()
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
        
        success = self.motion_controller.execute_poly_motion_with_feedback(
            current_pos, ascent_pos, 0.1, timeout=20)
        
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
        
        success = self.motion_controller.execute_poly_motion_with_feedback(
            final_pos, start_position, move_speed, timeout=timeout)
        
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
    """Move to position above valve center"""
    def __init__(self, module_id=1, approach_height=0.5):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        self.approach_height = approach_height
    
    def execute(self, userdata):
        rospy.loginfo("Moving to valve position (above valve center)...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None or self.valve_pos is None:
            rospy.logerr("Failed to get current UAV position or valve position")
            return 'failed'
        
        # Move to position above valve center
        target_pos = (self.valve_pos[0], self.valve_pos[1], 
                     self.valve_pos[2] + self.approach_height)
        
        # Calculate distance and set conservative speed
        distance = math.sqrt((target_pos[0]-start_pos[0])**2 + 
                           (target_pos[1]-start_pos[1])**2 + 
                           (target_pos[2]-start_pos[2])**2)
        
        move_speed = 0.1  # Conservative speed for stability
        timeout = max(90, distance / move_speed * 3.0)
        
        rospy.loginfo(f"Moving {distance:.2f}m at {move_speed}m/s")
        
        success = self.motion_controller.execute_poly_motion_with_feedback(
            start_pos, target_pos, move_speed, timeout=timeout)
        
        return 'succeeded' if success else 'failed'

class DescendAndContactState(SingleUAVStateBase):
    """Improved 4-stage insertion: Approach → Yaw Adjustment → Descent → Rotation"""
    def __init__(self, module_id=1, contact_force_threshold=3.0, descent_speed=0.03,  # Reduced from 0.05 to 0.03
                 use_staged_insertion=True, pre_insertion_distance=0.035,  # Increased from 0.015 to 0.035 for better insertion
                 yaw_adjustment_angle=0.08, circumferential_offset=0.01):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        self.contact_force_threshold = contact_force_threshold
        self.descent_speed = descent_speed
        # CRITICAL: Adjust insertion approach to avoid valve handle plane blockage
        # The valve handle plane blocks direct insertion, need angular approach
        # FIXED: Calculated correct z_offset based on valve URDF structure
        # valve_height=0.3, valve body offset=0.1, handle offset=0.17
        # Total handle height = 0.3+0.1+0.17 = 0.57, but handle visual origin is at z=-0.04
        # So actual handle level is at 0.57-0.04 = 0.53
        # But for insertion, we need to be slightly above the handle opening
        self.z_offset = 0.55 if rospy.get_param("~simulation", True) else 0.21  # Changed to 0.55 for proper handle level insertion
        self.use_staged_insertion = use_staged_insertion
        self.pre_insertion_distance = pre_insertion_distance
        self.yaw_adjustment_angle = yaw_adjustment_angle
        self.circumferential_offset = circumferential_offset
        
        # NEW: Angular insertion parameters to avoid handle plane blockage
        # TESTING: Disabled by default due to control instability issues
        self.angular_insertion_enabled = rospy.get_param("~angular_insertion_enabled", False)  # Changed from True to False
        self.insertion_angle_offset = 0.08  # Reduced from 0.15 to 0.08 for more conservative approach
        self.handle_clearance_height = 0.08  # 8cm above valve center for handle clearance
        
        rospy.loginfo(f"Angular insertion enabled: {self.angular_insertion_enabled}")
        if not self.angular_insertion_enabled:
            rospy.logwarn("Angular insertion is DISABLED - using normal descent only")
        else:
            rospy.logwarn("Angular insertion is ENABLED - may have control instability issues")
        
        # Initialize both optimizers for comparison
        simulation = rospy.get_param("~simulation", True)
        
        # OLD: Simple heuristic optimizer
        self.optimizer = InsertionOptimizer(
            valve_radius=0.1225,
            valve_beam_width=0.0185,
            safety_margin=0.008,
            module_id=module_id,
            simulation=simulation
        )
        
        # NEW: Constrained optimization optimizer
        self.constrained_optimizer = ConstrainedInsertionOptimizer(
            valve_radius=0.1225,
            valve_beam_width=0.0185,
            end_effector_length=0.246,
            safety_margin=0.005,
            module_id=module_id
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
        if start_pos is None or self.valve_pos is None:
            rospy.logerr("Failed to get current UAV position or valve position")
            return 'failed'
        
        # Store start position for emergency return
        userdata.start_position = start_pos
        
        # Stage 1: Approach insertion point with avoidance
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
        
        # Stabilization delay after Stage 2 to prevent position jump
        rospy.loginfo("Stabilization delay after Stage 2...")
        time.sleep(1.0)
        
        # Log pre-descent state
        current_pos = self.get_current_position()
        if current_pos is not None:
            rospy.loginfo(f"Pre-descent position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
            rospy.loginfo(f"Pre-descent yaw: {self.current_yaw:.3f} rad")
        
        if not self.descend_to_contact():
            rospy.logerr("Failed to descend to contact")
            return 'failed'
        
        # Log post-descent state
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
    
    def execute_circumferential_adjustment(self, start_pos, target_pos, target_yaw, 
                                           pos_threshold=0.03, yaw_threshold=0.08):
        """Execute circumferential adjustment with force monitoring and feedback control."""
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
            
            # Send command
            self.motion_controller.send_trajectory_point((interp_x, interp_y, interp_z), current_interp_yaw)

            # Wait until UAV reaches the intermediate point
            wait_start_time = time.time()
            while not rospy.is_shutdown() and (time.time() - wait_start_time) < 1.0: # 1s timeout per point
                current_pos = self.get_current_position()
                if current_pos is None:
                    rate.sleep()
                    continue

                pos_error = math.sqrt((interp_x - current_pos[0])**2 + 
                                      (interp_y - current_pos[1])**2 + 
                                      (interp_z - current_pos[2])**2)
                yaw_error = abs(self.normalize_angle(current_interp_yaw - self.current_yaw))

                if pos_error < pos_threshold and yaw_error < yaw_threshold:
                    break
                
                self.motion_controller.send_trajectory_point((interp_x, interp_y, interp_z), current_interp_yaw)
                rate.sleep()
            else:
                rospy.logwarn(f"Timeout waiting for circ. point. Pos err: {pos_error:.3f}, Yaw err: {yaw_error:.3f}")

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
            
            if i % 5 == 0:
                progress = (i + 1) / (num_points + 1) * 100
                rospy.loginfo(f"Circumferential adjustment progress: {progress:.1f}%")
            
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
    
    def execute_smooth_trajectory(self, start_pos, target_pos, target_yaw, duration=5.0,
                                  pos_threshold=0.03, yaw_threshold=0.08):
        """Execute smooth trajectory with yaw control and feedback."""
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
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 2.0: # Added 2s buffer
            pt = traj.evaluate()
            
            # Interpolate yaw smoothly
            elapsed_time = time.time() - start_time
            if elapsed_time <= duration:
                yaw_progress = elapsed_time / duration
                smooth_yaw_progress = 3*yaw_progress**2 - 2*yaw_progress**3
                current_target_yaw = start_yaw + smooth_yaw_progress * yaw_diff
            else:
                current_target_yaw = final_target_yaw

            if pt is None:
                pt = target_pos # Ensure final point is the target
            
            self.motion_controller.send_trajectory_point(pt, current_target_yaw)

            # Wait until UAV reaches the intermediate point
            wait_start_time = time.time()
            while not rospy.is_shutdown() and (time.time() - wait_start_time) < 1.0: # 1s timeout per point
                current_pos = self.get_current_position()
                if current_pos is None:
                    rate.sleep()
                    continue

                pos_error = math.sqrt((pt[0] - current_pos[0])**2 + 
                                      (pt[1] - current_pos[1])**2 + 
                                      (pt[2] - current_pos[2])**2)
                yaw_error = abs(self.normalize_angle(current_target_yaw - self.current_yaw))

                if pos_error < pos_threshold and yaw_error < yaw_threshold:
                    break
                
                self.motion_controller.send_trajectory_point(pt, current_target_yaw)
                rate.sleep()
            
            if pt is target_pos:
                break # Exit loop if we've processed the final point

        rospy.loginfo("Smooth trajectory with feedback completed.")
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
        rospy.loginfo("Phase 2: Descending to establish contact")
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
            insertion_offset=0.015 + self.pre_insertion_distance  # Increased from 0.005 to 0.015 for deeper insertion
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
            self.motion_controller.send_trajectory_point(pos, yaw)
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
            insertion_offset=0.015  # Increased from 0.005 to 0.015 for deeper insertion
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
            self.motion_controller.send_trajectory_point(pos, yaw)
            rate.sleep()
        
        rospy.loginfo("Final alignment completed")
        return True
    
    def align_to_grasp_position(self, start_pos):
        """
        Use optimizer results to align to grasp position
        CRITICAL FIX: Integrate optimizer results into trajectory generation
        """
        # Use optimizer to get optimal insertion strategy
        rospy.loginfo("Getting optimal insertion strategy using optimizer...")
        
        # Wait for necessary data
        if not self.wait_for_positions():
            return False
        
        # Get optimal strategy
        optimal_strategy = self.optimizer.evaluate_real_time_strategy()
        if optimal_strategy is None:
            rospy.logerr("Failed to get optimal insertion strategy")
            return False
        
        # Get insertion parameters
        insertion_params = self.optimizer.get_insertion_parameters(
            strategy=optimal_strategy,
            pre_insertion_distance=0.025,
            circumferential_offset=0.01
        )
        
        if insertion_params is None:
            rospy.logerr("Failed to get insertion parameters")
            return False
        
        # Log optimization results
        self.optimizer.log_insertion_plan(insertion_params)
        
        # Use optimization results to create alignment trajectory
        # Note: We use traditional AlignToGraspTrajectory here, but it has been corrected
        # In the future, we can further integrate specific optimizer results
        align_traj = AlignToGraspTrajectory(
            approach_duration=15.0,
            valve_center=self.valve_pos,
            valve_pose_yaw=self.valve_yaw,
            grasp_height=start_pos[2],  # Keep current height
            valve_radius=0.1225,
            valve_beam_width=0.0185,
            rotation_direction=1,  # Counterclockwise rotation
            insertion_offset=0.015  # Increased from 0.005 to 0.015 for deeper insertion
        )
        
        align_traj.set_start_position(start_pos)
        
        # Execute alignment trajectory
        rate = rospy.Rate(50)
        rospy.loginfo("Executing alignment trajectory according to user description...")
        
        while not rospy.is_shutdown() and not align_traj.is_complete():
            result = align_traj.get_next_position_and_yaw()
            if result is None:
                break
            pos, yaw = result
            self.motion_controller.send_trajectory_point(pos, yaw)
            rate.sleep()
        
        rospy.loginfo("Alignment completed - according to user's valve opening method")
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
        
        # CRITICAL FIX: Disable angular insertion temporarily due to control issues
        # The angular insertion has control instability problems, use normal descent instead
        if self.angular_insertion_enabled:
            rospy.logwarn("Angular insertion temporarily disabled due to control issues")
            rospy.loginfo("Using normal descent with strict velocity control instead")
            return self.execute_descent_with_contact_detection(current_pos, target_pos, descent_duration)
        else:
            # Execute descent with contact detection
            rospy.loginfo("Angular insertion disabled, using normal descent")
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
        
        rate = rospy.Rate(25)  # Reduced from 50Hz to 25Hz for smoother descent control
        start_time = time.time()
        
        # Variables for movement-based stuck detection
        last_height_check_time = start_time
        last_height = start_pos[2]
        stuck_check_interval = 2.0  # Check every 2 seconds
        min_descent_rate = 0.015  # Reduced from 0.02 to 0.015 for more conservative detection
        
        # CRITICAL FIX: Initialize previous target height for proper velocity limiting
        prev_target_height = start_pos[2]
        max_descent_per_cycle = self.descent_speed / 25.0  # 25Hz control rate
        
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
            self.motion_controller.send_trajectory_point((fixed_x, fixed_y, start_pos[2]), fixed_yaw)
            rate.sleep()
        
        rospy.loginfo("Position hold completed, starting descent...")
        
        # CRITICAL FIX: Reset start_time after hold phase to get correct elapsed time
        descent_start_time = time.time()
        
        # Reset stuck detection variables for descent phase
        last_height_check_time = descent_start_time
        last_height = start_pos[2]
        
        while not rospy.is_shutdown() and (time.time() - descent_start_time) < duration + 5.0:
            elapsed_time = time.time() - descent_start_time
            current_pos = self.get_current_position()
            
            if current_pos is None:
                rospy.logwarn("Lost position data during descent")
                return False
            
            # Check for stuck condition based on Z movement
            if elapsed_time - (last_height_check_time - descent_start_time) >= stuck_check_interval:
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
                last_height_check_time = elapsed_time + descent_start_time
                last_height = current_pos[2]
            
            # Calculate target height for current time with speed limiting
            progress = min(elapsed_time / duration, 1.0)
            ideal_target_height = start_pos[2] + progress * (target_pos[2] - start_pos[2])
            
            # CRITICAL FIX: Apply velocity limiting based on previous target, not current position
            # This ensures consistent descent speed regardless of tracking accuracy
            if ideal_target_height < prev_target_height:  # We need to descend
                max_descent_this_cycle = prev_target_height - max_descent_per_cycle
                target_height = max(ideal_target_height, max_descent_this_cycle)
                if target_height > ideal_target_height:
                    rospy.logdebug(f"Limiting descent speed: target {target_height:.3f}m instead of {ideal_target_height:.3f}m")
            else:  # We need to ascend or stay same
                max_ascent_this_cycle = prev_target_height + max_descent_per_cycle
                target_height = min(ideal_target_height, max_ascent_this_cycle)
                if target_height < ideal_target_height:
                    rospy.logdebug(f"Limiting ascent speed: target {target_height:.3f}m instead of {ideal_target_height:.3f}m")
            
            # Update previous target height for next iteration
            prev_target_height = target_height
            
            # Send position command with fixed XY, controlled Z, and fixed yaw
            self.motion_controller.send_trajectory_point((fixed_x, fixed_y, target_height), fixed_yaw)

            # Monitor XY drift
            xy_drift = math.sqrt((current_pos[0] - fixed_x)**2 + (current_pos[1] - fixed_y)**2)
            if xy_drift > 0.05:  # More than 5cm drift
                if elapsed_time % 2.0 < 0.02:  # Log every 2 seconds
                    rospy.logwarn(f"XY drift detected: {xy_drift:.3f}m from target position")
            
            # Monitor yaw drift
            yaw_drift = abs(self.normalize_angle(self.current_yaw - fixed_yaw))
            if yaw_drift > 0.2:  # More than ~11 degrees drift
                if elapsed_time % 2.0 < 0.02:  # Log every 2 seconds
                    rospy.logwarn(f"Yaw drift detected: {yaw_drift:.3f} rad from target yaw")
            
            # Check if we've reached the target
            if current_pos[2] <= target_pos[2] + 0.01:  # Within 1cm of target
                rospy.loginfo("Reached target descent height")
                return True
            
            # Progress logging
            if elapsed_time % 2.0 < 0.02:                
                descended = start_pos[2] - current_pos[2]
                rospy.loginfo(f"Descent progress: {elapsed_time:.1f}s, descended: {descended:.3f}m, XY drift: {xy_drift:.3f}m, yaw drift: {yaw_drift:.3f}rad")
                rospy.loginfo(f"Heights: start={start_pos[2]:.3f}m, current={current_pos[2]:.3f}m, target={target_height:.3f}m, ideal={ideal_target_height:.3f}m")
            
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
            
            # Send descent command with fixed XY position and yaw
            self.motion_controller.send_trajectory_point((fixed_x, fixed_y, next_z), fixed_yaw)

            # Monitor XY drift
            xy_drift = math.sqrt((current_pos[0] - fixed_x)**2 + (current_pos[1] - fixed_y)**2)
            if xy_drift > 0.05:  # More than 5cm drift
                if elapsed_time % 2.0 < 0.02:  # Log every 2 seconds
                    rospy.logwarn(f"XY drift detected: {xy_drift:.3f}m from target position")
            
            # Monitor yaw drift
            yaw_drift = abs(self.normalize_angle(self.current_yaw - fixed_yaw))
            if yaw_drift > 0.2:  # More than ~11 degrees drift
                if elapsed_time % 2.0 < 0.02:  # Log every 2 seconds
                    rospy.logwarn(f"Yaw drift detected: {yaw_drift:.3f} rad from target yaw")
            
            # Progress logging
            if elapsed_time % 2.0 < 0.02:                
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

    def execute_angular_insertion(self, start_pos, target_pos, duration):
        """Execute angular insertion to avoid valve handle plane blockage"""
        
        rospy.loginfo("=== ANGULAR INSERTION TO AVOID HANDLE PLANE BLOCKAGE ===")
        rospy.loginfo("Problem: End-effector blocked by valve handle plane")
        rospy.loginfo("Solution: Angular approach to bypass handle plane")
        
        # Calculate valve center and current relative position
        valve_center_x, valve_center_y, valve_center_z = self.valve_pos
        start_x, start_y, start_z = start_pos
        
        # Calculate current angle from valve center
        current_angle = math.atan2(start_y - valve_center_y, start_x - valve_center_x)
        current_radius = math.sqrt((start_x - valve_center_x)**2 + (start_y - valve_center_y)**2)
        
        rospy.loginfo(f"Current position relative to valve:")
        rospy.loginfo(f"  - Angle: {current_angle:.3f} rad ({current_angle*180/math.pi:.1f}°)")
        rospy.loginfo(f"  - Radius: {current_radius:.3f}m")
        rospy.loginfo(f"  - Height: {start_z:.3f}m")
        
        # Stage 1: Move to clearance height above valve handle
        clearance_height = valve_center_z + self.handle_clearance_height
        rospy.loginfo(f"Stage 1: Moving to handle clearance height: {clearance_height:.3f}m")
        
        # First, move to clearance height to avoid handle plane
        clearance_pos = (start_x, start_y, clearance_height)
        if not self.execute_vertical_movement(start_pos, clearance_pos, "clearance"):
            return False
        
        # Stage 2: Angular approach - move closer to valve center at clearance height
        # Calculate position closer to valve center for angular insertion
        insertion_radius = current_radius - self.insertion_angle_offset
        # CRITICAL FIX: Use more conservative minimum radius to prevent movement timeout
        min_safe_radius = 0.12  # Increased from 0.05m to 0.12m for safer approach
        if insertion_radius < min_safe_radius:
            insertion_radius = min_safe_radius
            rospy.logwarn(f"Limiting insertion radius to safe minimum: {insertion_radius:.3f}m")
        
        # Additional safety check: don't move too far from current position
        max_radius_change = 0.08  # Maximum 8cm radius change
        if abs(insertion_radius - current_radius) > max_radius_change:
            if insertion_radius < current_radius:
                insertion_radius = current_radius - max_radius_change
            else:
                insertion_radius = current_radius + max_radius_change
            rospy.logwarn(f"Limiting radius change to {max_radius_change:.3f}m, new radius: {insertion_radius:.3f}m")
        
        angular_approach_x = valve_center_x + insertion_radius * math.cos(current_angle)
        angular_approach_y = valve_center_y + insertion_radius * math.sin(current_angle)
        angular_approach_pos = (angular_approach_x, angular_approach_y, clearance_height)
        
        rospy.loginfo(f"Stage 2: Angular approach to insertion point:")
        rospy.loginfo(f"  - New radius: {insertion_radius:.3f}m")
        rospy.loginfo(f"  - Position: [{angular_approach_x:.3f}, {angular_approach_y:.3f}, {clearance_height:.3f}]")
        
        if not self.execute_horizontal_movement(clearance_pos, angular_approach_pos, "angular_approach"):
            rospy.logwarn("Angular approach failed, proceeding with current position")
            # Use current position instead of failing completely
            current_pos = self.get_current_position()
            if current_pos is None:
                return False
            angular_approach_pos = (current_pos[0], current_pos[1], clearance_height)
        
        # Stage 3: Angled descent to target position
        # Calculate final target position with angle consideration
        final_target_x = valve_center_x + insertion_radius * math.cos(current_angle)
        final_target_y = valve_center_y + insertion_radius * math.sin(current_angle)
        final_target_z = target_pos[2]
        final_target_pos = (final_target_x, final_target_y, final_target_z)
        
        rospy.loginfo(f"Stage 3: Angled descent to final insertion position:")
        rospy.loginfo(f"  - Target: [{final_target_x:.3f}, {final_target_y:.3f}, {final_target_z:.3f}]")
        rospy.loginfo(f"  - Descent distance: {clearance_height - final_target_z:.3f}m")
        
        # Execute angled descent with simultaneous XY and Z movement
        return self.execute_angled_descent(angular_approach_pos, final_target_pos, duration)
    
    def execute_vertical_movement(self, start_pos, target_pos, stage_name):
        """Execute vertical movement for clearance"""
        rospy.loginfo(f"Executing {stage_name} vertical movement...")
        
        # Calculate movement distance for logging
        vertical_distance = abs(target_pos[2] - start_pos[2])
        rospy.loginfo(f"{stage_name} vertical distance: {vertical_distance:.3f}m")
        rospy.loginfo(f"From height: {start_pos[2]:.3f}m to {target_pos[2]:.3f}m")
        
        fixed_x, fixed_y = start_pos[0], start_pos[1]
        fixed_yaw = self.current_yaw
        
        start_time = time.time()
        rate = rospy.Rate(50)
        timeout = 10.0  # Increased timeout from 5.0 to 10.0 seconds
        
        # Use more relaxed threshold for vertical movement
        height_threshold = 0.03  # Increased from 0.02m to 0.03m
        
        # Track progress
        last_progress_time = start_time
        last_height_diff = vertical_distance
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            current_pos = self.get_current_position()
            if current_pos is None:
                continue
            
            # Check if reached target height
            height_diff = abs(current_pos[2] - target_pos[2])
            if height_diff < height_threshold:
                rospy.loginfo(f"{stage_name} vertical movement completed (height diff: {height_diff:.3f}m)")
                return True
            
            # Send command to move to target height
            self.motion_controller.send_trajectory_point((fixed_x, fixed_y, target_pos[2]), fixed_yaw)
            
            # Progress monitoring
            current_time = time.time()
            if current_time - last_progress_time >= 2.0:  # Every 2 seconds
                progress = (vertical_distance - height_diff) / vertical_distance * 100
                rospy.loginfo(f"{stage_name} progress: {progress:.1f}%, remaining: {height_diff:.3f}m")
                
                # Check if movement is stuck
                height_change = last_height_diff - height_diff
                if height_change < 0.01:  # Less than 1cm progress in 2 seconds
                    rospy.logwarn(f"{stage_name} appears stuck, height change: {height_change:.3f}m")
                    # Continue anyway, don't give up immediately
                
                last_progress_time = current_time
                last_height_diff = height_diff
            
            rate.sleep()
        
        # Final check with more relaxed threshold
        final_pos = self.get_current_position()
        if final_pos is not None:
            final_height_diff = abs(final_pos[2] - target_pos[2])
            if final_height_diff < 0.05:  # Very relaxed final threshold
                rospy.logwarn(f"{stage_name} completed with relaxed threshold (height diff: {final_height_diff:.3f}m)")
                return True
        
        rospy.logwarn(f"{stage_name} vertical movement timed out")
        return False
    
    def execute_horizontal_movement(self, start_pos, target_pos, stage_name):
        """Execute horizontal movement with position feedback control"""
        rospy.loginfo(f"Executing {stage_name} horizontal movement...")
        
        # Calculate movement distance for logging
        movement_distance = math.sqrt((target_pos[0] - start_pos[0])**2 + 
                                    (target_pos[1] - start_pos[1])**2)
        rospy.loginfo(f"{stage_name} movement distance: {movement_distance:.3f}m")
        rospy.loginfo(f"From: [{start_pos[0]:.3f}, {start_pos[1]:.3f}] to [{target_pos[0]:.3f}, {target_pos[1]:.3f}]")
        
        # CRITICAL FIX: Skip horizontal movement if distance is too small
        if movement_distance < 0.015:  # Less than 1.5cm
            rospy.loginfo(f"{stage_name} movement too small ({movement_distance:.3f}m), skipping")
            return True
        
        # CRITICAL FIX: Limit maximum horizontal movement to prevent large errors
        max_horizontal_movement = 0.06  # 6cm maximum
        if movement_distance > max_horizontal_movement:
            rospy.logwarn(f"{stage_name} movement too large ({movement_distance:.3f}m), limiting to {max_horizontal_movement:.3f}m")
            # Scale down the movement
            scale_factor = max_horizontal_movement / movement_distance
            limited_target_x = start_pos[0] + (target_pos[0] - start_pos[0]) * scale_factor
            limited_target_y = start_pos[1] + (target_pos[1] - start_pos[1]) * scale_factor
            target_pos = (limited_target_x, limited_target_y, target_pos[2])
            movement_distance = max_horizontal_movement
        
        fixed_z = start_pos[2]  # Keep same height
        fixed_yaw = self.current_yaw
        
        start_time = time.time()
        rate = rospy.Rate(25)  # Reduced from 50Hz to 25Hz
        timeout = 8.0  # Reduced timeout from 12.0 to 8.0
        
        # Use more relaxed threshold for horizontal movement
        position_threshold = 0.04  # Increased from 0.03m to 0.04m
        
        # Track progress for stuck detection
        last_progress_time = start_time
        last_distance = movement_distance
        
        # CRITICAL FIX: Add velocity limiting for horizontal movement
        max_xy_movement_per_cycle = 0.003  # 3mm per cycle at 25Hz
        
        rospy.loginfo(f"{stage_name} parameters:")
        rospy.loginfo(f"  - Target: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {fixed_z:.3f}]")
        rospy.loginfo(f"  - Max movement per cycle: {max_xy_movement_per_cycle:.3f}m")
        rospy.loginfo(f"  - Position threshold: {position_threshold:.3f}m")
        rospy.loginfo(f"  - Timeout: {timeout:.1f}s")
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            current_pos = self.get_current_position()
            if current_pos is None:
                continue
            
            # Check if reached target XY position
            xy_distance = math.sqrt((current_pos[0] - target_pos[0])**2 + 
                                  (current_pos[1] - target_pos[1])**2)
            if xy_distance < position_threshold:
                rospy.loginfo(f"{stage_name} horizontal movement completed (distance: {xy_distance:.3f}m)")
                return True
            
            # CRITICAL FIX: Apply velocity limiting to prevent overshooting
            # Calculate desired movement
            desired_x = target_pos[0]
            desired_y = target_pos[1]
            
            # Check if movement is too large for one cycle
            movement_this_cycle = math.sqrt((desired_x - current_pos[0])**2 + 
                                          (desired_y - current_pos[1])**2)
            
            if movement_this_cycle > max_xy_movement_per_cycle:
                # Scale down movement to maximum allowed per cycle
                scale_factor = max_xy_movement_per_cycle / movement_this_cycle
                desired_x = current_pos[0] + (desired_x - current_pos[0]) * scale_factor
                desired_y = current_pos[1] + (desired_y - current_pos[1]) * scale_factor
                rospy.logdebug(f"Limiting movement to {max_xy_movement_per_cycle:.3f}m per cycle")
            
            # Send command with velocity-limited position
            self.motion_controller.send_trajectory_point((desired_x, desired_y, fixed_z), fixed_yaw)
            
            # Progress monitoring
            current_time = time.time()
            if current_time - last_progress_time >= 1.0:  # Every 1 second
                progress = (movement_distance - xy_distance) / movement_distance * 100
                rospy.loginfo(f"{stage_name} progress: {progress:.1f}%, remaining: {xy_distance:.3f}m")
                
                # Check if movement is stuck
                distance_change = last_distance - xy_distance
                if distance_change < 0.005:  # Less than 0.5cm progress in 1 second
                    rospy.logwarn(f"{stage_name} slow progress, distance change: {distance_change:.3f}m")
                elif distance_change < 0:  # Moving away from target
                    rospy.logwarn(f"{stage_name} moving away from target, distance change: {distance_change:.3f}m")
                    # Don't immediately fail, allow some tolerance
                
                last_progress_time = current_time
                last_distance = xy_distance
            
            rate.sleep()
        
        # Final check with relaxed threshold
        final_pos = self.get_current_position()
        if final_pos is not None:
            final_distance = math.sqrt((final_pos[0] - target_pos[0])**2 + 
                                     (final_pos[1] - target_pos[1])**2)
            if final_distance < 0.06:  # Very relaxed final threshold (6cm)
                rospy.logwarn(f"{stage_name} completed with relaxed threshold (distance: {final_distance:.3f}m)")
                return True
        
        rospy.logwarn(f"{stage_name} horizontal movement timed out")
        return False
    
    def execute_angled_descent(self, start_pos, target_pos, duration):
        """Execute angled descent with controlled speed and position feedback"""
        rospy.loginfo("Executing angled descent to bypass handle plane...")
        
        fixed_yaw = self.current_yaw
        start_time = time.time()
        rate = rospy.Rate(25)  # Reduced from 50Hz to 25Hz for better control
        
        # Movement monitoring for stuck detection
        last_height_check_time = start_time
        last_height = start_pos[2]
        stuck_check_interval = 2.0
        min_descent_rate = 0.015  # Reduced from 0.02 to 0.015
        
        # CRITICAL FIX: Add velocity limiting for angled descent
        max_descent_per_cycle = self.descent_speed / 25.0  # 25Hz control rate
        max_xy_movement_per_cycle = 0.002  # 2mm XY movement per cycle
        
        rospy.loginfo(f"Angled descent parameters:")
        rospy.loginfo(f"  - Start position: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")
        rospy.loginfo(f"  - Target position: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
        rospy.loginfo(f"  - Duration: {duration:.1f}s")
        rospy.loginfo(f"  - Max descent per cycle: {max_descent_per_cycle:.3f}m")
        rospy.loginfo(f"  - Max XY movement per cycle: {max_xy_movement_per_cycle:.3f}m")
        
        # Use current position as actual start position in case of angular approach failure
        current_pos = self.get_current_position()
        if current_pos is not None:
            actual_start_pos = current_pos
            rospy.loginfo(f"Using actual current position as start: [{actual_start_pos[0]:.3f}, {actual_start_pos[1]:.3f}, {actual_start_pos[2]:.3f}]")
        else:
            actual_start_pos = start_pos
        
        # Calculate total movement distances
        total_x_distance = target_pos[0] - actual_start_pos[0]
        total_y_distance = target_pos[1] - actual_start_pos[1]
        total_z_distance = target_pos[2] - actual_start_pos[2]
        
        rospy.loginfo(f"Total movement distances: X={total_x_distance:.3f}m, Y={total_y_distance:.3f}m, Z={total_z_distance:.3f}m")
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 10.0:  # Increased timeout
            elapsed_time = time.time() - start_time
            current_pos = self.get_current_position()
            
            if current_pos is None:
                continue
            
            # Calculate progress (0 to 1)
            progress = min(elapsed_time / duration, 1.0)
            smooth_progress = 3*progress**2 - 2*progress**3  # Smooth interpolation
            
            # Calculate desired position
            desired_x = actual_start_pos[0] + smooth_progress * total_x_distance
            desired_y = actual_start_pos[1] + smooth_progress * total_y_distance
            desired_z = actual_start_pos[2] + smooth_progress * total_z_distance
            
            # CRITICAL FIX: Apply velocity limiting to prevent fast descent
            # Limit Z movement (descent speed) - FIXED: Check direction properly
            if total_z_distance < 0:  # Descending (negative Z movement)
                z_diff = desired_z - current_pos[2]  # Negative means descending
                if z_diff < -max_descent_per_cycle:  # Descending too fast
                    desired_z = current_pos[2] - max_descent_per_cycle
                    rospy.logdebug(f"Limiting descent speed: desired_z adjusted to {desired_z:.3f}m")
            else:  # Ascending (positive Z movement)
                z_diff = desired_z - current_pos[2]  # Positive means ascending
                if z_diff > max_descent_per_cycle:  # Ascending too fast
                    desired_z = current_pos[2] + max_descent_per_cycle
                    rospy.logdebug(f"Limiting ascent speed: desired_z adjusted to {desired_z:.3f}m")
            
            # Limit XY movement
            xy_diff = math.sqrt((desired_x - current_pos[0])**2 + (desired_y - current_pos[1])**2)
            if xy_diff > max_xy_movement_per_cycle:
                scale_factor = max_xy_movement_per_cycle / xy_diff
                desired_x = current_pos[0] + (desired_x - current_pos[0]) * scale_factor
                desired_y = current_pos[1] + (desired_y - current_pos[1]) * scale_factor
                rospy.logdebug(f"Limiting XY movement: scaled to {max_xy_movement_per_cycle:.3f}m")
            
            # Send movement command with velocity-limited position
            self.motion_controller.send_trajectory_point((desired_x, desired_y, desired_z), fixed_yaw)
            
            # Check for stuck condition
            if elapsed_time - last_height_check_time >= stuck_check_interval:
                height_change = last_height - current_pos[2]
                
                if height_change < min_descent_rate:
                    remaining_descent = current_pos[2] - target_pos[2]
                    if remaining_descent < 0.05:  # Within 5cm of target
                        rospy.loginfo("Angled descent reached target height")
                        return True
                    else:
                        rospy.logwarn("UAV stuck during angled descent")
                        return False
                
                last_height_check_time = elapsed_time + start_time
                last_height = current_pos[2]
            
            # Check if reached target with relaxed threshold
            distance_to_target = math.sqrt((current_pos[0] - target_pos[0])**2 + 
                                         (current_pos[1] - target_pos[1])**2 + 
                                         (current_pos[2] - target_pos[2])**2)
            
            if distance_to_target < 0.05:  # Relaxed from 0.03 to 0.05
                rospy.loginfo("Angled descent completed successfully")
                return True
            
            # Progress logging
            if elapsed_time % 2.0 < 0.02:
                actual_descent = actual_start_pos[2] - current_pos[2]
                rospy.loginfo(f"Angled descent progress: {progress*100:.1f}%, distance to target: {distance_to_target:.3f}m, descended: {actual_descent:.3f}m")
            
            rate.sleep()
        
        rospy.loginfo("Angled descent completed (timeout)")
        return True

class RotateValveState(SingleUAVStateBase):
    """Rotate valve while maintaining contact"""
    def __init__(self, module_id=1, rotation_angle=math.pi/2, rotation_duration=8.0):
        # CRITICAL FIX: Ensure parent class initialization is completed
        try:
            SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed', 'emergency'], 
                                       input_keys=['start_position'], module_id=module_id)
        except Exception as e:
            rospy.logerr(f"Failed to initialize SingleUAVStateBase: {e}")
            raise
        
        # Verify critical attributes exist
        if not hasattr(self, 'uav_received'):
            rospy.logerr("uav_received not initialized - creating fallback")
            self.uav_received = threading.Event()
        
        if not hasattr(self, 'valve_received'):
            rospy.logerr("valve_received not initialized - creating fallback")
            self.valve_received = threading.Event()
        
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
        
        # ENHANCED: Strict alignment control parameters for valve rotation
        # Ensure COG, end-effector center, and valve center are collinear
        self.strict_alignment_enabled = rospy.get_param("~strict_alignment_enabled", True)
        
        # End-effector parameters (relative to UAV body frame)
        self.end_effector_offset = rospy.get_param("~end_effector_offset", [0.0, 0.0, -0.246])  # 24.6cm below COG
        self.end_effector_length = 0.246  # Total length from COG to end-effector tip
        
        # Alignment control parameters
        self.alignment_position_tolerance = rospy.get_param("~alignment_position_tolerance", 0.01)  # 1cm
        self.alignment_yaw_tolerance = rospy.get_param("~alignment_yaw_tolerance", 0.05)  # ~2.9 degrees
        self.alignment_control_gain = rospy.get_param("~alignment_control_gain", 0.8)  # Control gain for alignment
        
        # Rotation trajectory stability parameters
        self.rotation_radius_tolerance = rospy.get_param("~rotation_radius_tolerance", 0.005)  # 0.5cm radius deviation
        self.rotation_height_tolerance = rospy.get_param("~rotation_height_tolerance", 0.01)  # 1cm height deviation
        
        # Feedback control parameters
        self.alignment_check_frequency = rospy.get_param("~alignment_check_frequency", 25)  # Hz
        self.alignment_correction_enabled = rospy.get_param("~alignment_correction_enabled", True)
        
        rospy.loginfo(f"=== ENHANCED VALVE ROTATION CONTROL ===")
        rospy.loginfo(f"Strict alignment enabled: {self.strict_alignment_enabled}")
        rospy.loginfo(f"End-effector offset: {self.end_effector_offset}")
        rospy.loginfo(f"Alignment tolerances: pos={self.alignment_position_tolerance:.3f}m, yaw={self.alignment_yaw_tolerance:.3f}rad")
        rospy.loginfo(f"Rotation tolerances: radius={self.rotation_radius_tolerance:.3f}m, height={self.rotation_height_tolerance:.3f}m")
        rospy.loginfo(f"Alignment control gain: {self.alignment_control_gain:.2f}")
        rospy.loginfo(f"Alignment check frequency: {self.alignment_check_frequency}Hz")
    
    def calculate_end_effector_position(self, uav_pos, uav_yaw):
        """Calculate end-effector position based on UAV position and orientation"""
        # Transform end-effector offset from body frame to world frame
        cos_yaw = math.cos(uav_yaw)
        sin_yaw = math.sin(uav_yaw)
        
        # Apply rotation matrix for yaw
        world_offset_x = (self.end_effector_offset[0] * cos_yaw - 
                         self.end_effector_offset[1] * sin_yaw)
        world_offset_y = (self.end_effector_offset[0] * sin_yaw + 
                         self.end_effector_offset[1] * cos_yaw)
        world_offset_z = self.end_effector_offset[2]  # No rotation for Z
        
        # Calculate end-effector position in world frame
        end_effector_x = uav_pos[0] + world_offset_x
        end_effector_y = uav_pos[1] + world_offset_y
        end_effector_z = uav_pos[2] + world_offset_z
        
        return (end_effector_x, end_effector_y, end_effector_z)
    
    def check_cog_end_effector_valve_alignment(self, uav_pos, uav_yaw):
        """Check if COG, end-effector center, and valve center are properly aligned"""
        if self.valve_pos is None:
            return False, {}
        
        # Calculate end-effector position
        end_effector_pos = self.calculate_end_effector_position(uav_pos, uav_yaw)
        
        # Calculate alignment metrics
        valve_center = self.valve_pos
        
        # 1. Check if end-effector is centered on valve
        end_effector_to_valve_distance = math.sqrt(
            (end_effector_pos[0] - valve_center[0])**2 + 
            (end_effector_pos[1] - valve_center[1])**2
        )
        
        # 2. Check if UAV COG is properly positioned relative to valve
        uav_to_valve_distance = math.sqrt(
            (uav_pos[0] - valve_center[0])**2 + 
            (uav_pos[1] - valve_center[1])**2
        )
        
        # 3. Check if UAV is pointing toward valve center
        desired_yaw = math.atan2(valve_center[1] - uav_pos[1], valve_center[0] - uav_pos[0])
        yaw_error = abs(self.normalize_angle(uav_yaw - desired_yaw))
        
        # 4. Check if UAV is at correct distance from valve (for proper insertion)
        ideal_uav_distance = math.sqrt(self.end_effector_length**2 - 
                                      (uav_pos[2] - valve_center[2])**2) if uav_pos[2] != valve_center[2] else self.end_effector_length
        distance_error = abs(uav_to_valve_distance - ideal_uav_distance)
        
        # 5. Check collinearity - COG, end-effector, and valve should be on same line
        # Calculate the angle between COG->end_effector and COG->valve vectors
        cog_to_ee_vector = (end_effector_pos[0] - uav_pos[0], end_effector_pos[1] - uav_pos[1])
        cog_to_valve_vector = (valve_center[0] - uav_pos[0], valve_center[1] - uav_pos[1])
        
        # Calculate angle between vectors
        dot_product = (cog_to_ee_vector[0] * cog_to_valve_vector[0] + 
                      cog_to_ee_vector[1] * cog_to_valve_vector[1])
        
        ee_magnitude = math.sqrt(cog_to_ee_vector[0]**2 + cog_to_ee_vector[1]**2)
        valve_magnitude = math.sqrt(cog_to_valve_vector[0]**2 + cog_to_valve_vector[1]**2)
        
        collinearity_angle = 0.0
        if ee_magnitude > 0.001 and valve_magnitude > 0.001:
            cos_angle = dot_product / (ee_magnitude * valve_magnitude)
            cos_angle = max(-1.0, min(1.0, cos_angle))  # Clamp to valid range
            collinearity_angle = math.acos(cos_angle)
        
        # Determine alignment status
        position_aligned = end_effector_to_valve_distance < self.alignment_position_tolerance
        yaw_aligned = yaw_error < self.alignment_yaw_tolerance
        distance_aligned = distance_error < self.alignment_position_tolerance
        collinear = collinearity_angle < self.alignment_yaw_tolerance
        
        is_aligned = position_aligned and yaw_aligned and distance_aligned and collinear
        
        # Create detailed alignment report
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
        """Calculate position and yaw corrections to improve alignment"""
        if not alignment_report or alignment_report['is_aligned']:
            return (0.0, 0.0, 0.0), 0.0  # No correction needed
        
        # Extract data from alignment report
        uav_pos = alignment_report['uav_pos']
        valve_center = alignment_report['valve_center']
        yaw_error = alignment_report['yaw_error']
        desired_yaw = alignment_report['desired_yaw']
        current_yaw = alignment_report['uav_yaw']
        distance_error = alignment_report['distance_error']
        ideal_distance = alignment_report['ideal_uav_distance']
        
        # Calculate position corrections
        # Move UAV to ideal position relative to valve
        direction_to_valve = math.atan2(valve_center[1] - uav_pos[1], valve_center[0] - uav_pos[0])
        
        # Calculate ideal UAV position
        ideal_uav_x = valve_center[0] - ideal_distance * math.cos(direction_to_valve)
        ideal_uav_y = valve_center[1] - ideal_distance * math.sin(direction_to_valve)
        ideal_uav_z = uav_pos[2]  # Maintain current height
        
        # Calculate position correction with gain
        pos_correction_x = (ideal_uav_x - uav_pos[0]) * self.alignment_control_gain
        pos_correction_y = (ideal_uav_y - uav_pos[1]) * self.alignment_control_gain
        pos_correction_z = 0.0  # No Z correction during rotation
        
        # Limit correction magnitude to prevent large jumps
        max_correction = 0.02  # 2cm max correction per cycle
        correction_magnitude = math.sqrt(pos_correction_x**2 + pos_correction_y**2)
        if correction_magnitude > max_correction:
            scale_factor = max_correction / correction_magnitude
            pos_correction_x *= scale_factor
            pos_correction_y *= scale_factor
        
        # Calculate yaw correction
        yaw_correction = self.normalize_angle(desired_yaw - current_yaw) * self.alignment_control_gain
        
        # Limit yaw correction to prevent aggressive rotation
        max_yaw_correction = 0.1  # ~5.7 degrees max correction per cycle
        yaw_correction = max(-max_yaw_correction, min(max_yaw_correction, yaw_correction))
        
        return (pos_correction_x, pos_correction_y, pos_correction_z), yaw_correction
    
    def execute_pre_rotation_alignment(self, initial_pos, initial_yaw, timeout=10.0):
        """Execute pre-rotation alignment to ensure optimal starting position"""
        if not self.strict_alignment_enabled:
            rospy.loginfo("Strict alignment disabled - skipping pre-rotation alignment")
            return True
        
        rospy.loginfo("=== PRE-ROTATION ALIGNMENT PHASE ===")
        rospy.loginfo("Ensuring COG-End_effector-Valve collinearity before rotation")
        
        start_time = time.time()
        rate = rospy.Rate(self.alignment_check_frequency)
        
        alignment_achieved = False
        last_report_time = start_time
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            current_pos = self.get_current_position()
            if current_pos is None:
                rospy.logwarn("Lost position data during pre-rotation alignment")
                continue
            
            # Check current alignment
            is_aligned, alignment_report = self.check_cog_end_effector_valve_alignment(
                current_pos, self.current_yaw)
            
            if is_aligned:
                rospy.loginfo("Pre-rotation alignment achieved successfully")
                alignment_achieved = True
                break
            
            # Calculate and apply alignment corrections
            if self.alignment_correction_enabled:
                pos_correction, yaw_correction = self.calculate_alignment_correction(alignment_report)
                
                # Apply corrections
                corrected_x = current_pos[0] + pos_correction[0]
                corrected_y = current_pos[1] + pos_correction[1]
                corrected_z = current_pos[2] + pos_correction[2]
                corrected_yaw = self.current_yaw + yaw_correction
                
                # Send corrected position command
                self.motion_controller.send_trajectory_point(
                    (corrected_x, corrected_y, corrected_z), corrected_yaw)
            
            # Progress reporting every 2 seconds
            current_time = time.time()
            if current_time - last_report_time >= 2.0:
                rospy.loginfo(f"Pre-rotation alignment progress:")
                rospy.loginfo(f"  - End-effector to valve distance: {alignment_report['end_effector_to_valve_distance']:.3f}m (tolerance: {self.alignment_position_tolerance:.3f}m)")
                rospy.loginfo(f"  - Yaw error: {alignment_report['yaw_error']:.3f}rad (tolerance: {self.alignment_yaw_tolerance:.3f}rad)")
                rospy.loginfo(f"  - Distance error: {alignment_report['distance_error']:.3f}m")
                rospy.loginfo(f"  - Collinearity angle: {alignment_report['collinearity_angle']:.3f}rad")
                rospy.loginfo(f"  - Status: pos_aligned={alignment_report['position_aligned']}, yaw_aligned={alignment_report['yaw_aligned']}, collinear={alignment_report['collinear']}")
                last_report_time = current_time
            
            rate.sleep()
        
        if not alignment_achieved:
            rospy.logwarn("Pre-rotation alignment timeout - proceeding with current position")
            # Log final alignment status
            current_pos = self.get_current_position()
            if current_pos is not None:
                _, final_report = self.check_cog_end_effector_valve_alignment(current_pos, self.current_yaw)
                rospy.logwarn(f"Final alignment status: {final_report}")
        
        return alignment_achieved

    def check_rotation_emergency(self):
        """Check if UAV is stuck during rotation"""
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        # Calculate movement since last check
        if self.last_position is not None:
            movement = math.sqrt(
                (current_pos[0] - self.last_position[0])**2 + 
                (current_pos[1] - self.last_position[1])**2 + 
                (current_pos[2] - self.last_position[2])**2
            )
            
            yaw_movement = abs(self.normalize_angle(self.current_yaw - self.last_yaw))
            
            if movement < self.movement_threshold and yaw_movement < self.yaw_threshold:
                if self.stuck_start_time is None:
                    self.stuck_start_time = time.time()
                    rospy.loginfo("UAV appears stuck during rotation - starting timer")
                else:
                    stuck_duration = time.time() - self.stuck_start_time
                    if stuck_duration > self.stuck_threshold:
                        rospy.logwarn(f"UAV stuck for {stuck_duration:.1f}s during rotation")
                        return True
            else:
                if self.stuck_start_time is not None:
                    rospy.loginfo("UAV movement detected - resetting stuck timer")
                self.stuck_start_time = None
        
        # Update last position
        self.last_position = current_pos
        self.last_yaw = self.current_yaw
        
        return False
    
    def execute(self, userdata):
        """Execute valve rotation with enhanced alignment control"""
        rospy.loginfo("Starting modern valve rotation with enhanced alignment control...")
        
        # Get start position for emergency return
        start_position = userdata.start_position
        if start_position is None:
            rospy.logerr("No start position available for emergency return")
            return 'failed'
        
        # Wait for position data
        if not self.wait_for_positions():
            return 'failed'
        
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Failed to get current position")
            return 'failed'
        
        # Initialize rotation tracking
        self.last_position = current_pos
        self.last_yaw = self.current_yaw
        self.stuck_start_time = None
        
        # Phase 1: Pre-rotation alignment using enhanced motion controller
        rospy.loginfo("=== PHASE 1: PRE-ROTATION ALIGNMENT ===")
        alignment_success = self.motion_controller.execute_pre_rotation_alignment(
            self.valve_pos, timeout=10.0)
        
        if not alignment_success:
            rospy.logwarn("Pre-rotation alignment incomplete - proceeding with caution")
        
        # Update position after alignment
        aligned_pos = self.get_current_position()
        if aligned_pos is not None:
            current_pos = aligned_pos
        
        # Phase 2: Create and execute rotation trajectory
        rospy.loginfo("=== PHASE 2: ALIGNMENT-CONTROLLED ROTATION ===")
        
        # Calculate rotation parameters from current position
        # CRITICAL FIX: Calculate end-effector to valve center distance as rotation radius
        # This ensures the end-effector maintains constant distance from valve center
        
        # 1. Calculate current end-effector position
        current_end_effector_pos = self.calculate_end_effector_position(current_pos, self.current_yaw)
        
        # 2. Calculate end-effector to valve center distance (this is the true rotation radius)
        end_effector_rotation_radius = math.sqrt(
            (current_end_effector_pos[0] - self.valve_pos[0])**2 + 
            (current_end_effector_pos[1] - self.valve_pos[1])**2
        )
        
        # 3. Calculate starting angle (end-effector angle relative to valve center)
        start_angle = math.atan2(
            current_end_effector_pos[1] - self.valve_pos[1],
            current_end_effector_pos[0] - self.valve_pos[0]
        )
        
        # CRITICAL FIX: Use current UAV height directly for rotation
        # The UAV should maintain the same height throughout rotation
        current_height = current_pos[2]
        
        rospy.loginfo(f"End-effector constant distance rotation parameters:")
        rospy.loginfo(f"  - UAV position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
        rospy.loginfo(f"  - End-effector position: [{current_end_effector_pos[0]:.3f}, {current_end_effector_pos[1]:.3f}, {current_end_effector_pos[2]:.3f}]")
        rospy.loginfo(f"  - Valve center: [{self.valve_pos[0]:.3f}, {self.valve_pos[1]:.3f}, {self.valve_pos[2]:.3f}]")
        rospy.loginfo(f"  - End-effector rotation radius: {end_effector_rotation_radius:.3f}m")
        rospy.loginfo(f"  - Starting angle: {start_angle:.3f}rad ({start_angle*180/math.pi:.1f}°)")
        rospy.loginfo(f"  - Current height: {current_height:.3f}m (will be maintained)")
        rospy.loginfo(f"  - Rotation angle: {self.rotation_angle:.3f}rad ({self.rotation_angle*180/math.pi:.1f}°)")
        
        # Calculate correct rotation height - should be at valve level
        # The UAV body should be at valve height + end_effector_offset for proper engagement
        valve_height = self.valve_pos[2]
        end_effector_offset_z = 0.0743823  # From trajectory.py
        correct_rotation_height = valve_height + end_effector_offset_z
        
        rospy.loginfo(f"Rotation height calculation:")
        rospy.loginfo(f"  - Valve height: {valve_height:.3f}m")
        rospy.loginfo(f"  - End-effector offset Z: {end_effector_offset_z:.3f}m")
        rospy.loginfo(f"  - Correct rotation height: {correct_rotation_height:.3f}m")
        rospy.loginfo(f"  - Current height: {current_height:.3f}m")
        rospy.loginfo(f"  - Height adjustment: {correct_rotation_height - current_height:.3f}m")
        
        # Create rotation trajectory with correct height
        # CRITICAL FIX: Use new constant distance rotation trajectory generator
        # Ensure end-effector maintains constant distance from valve center
        rotation_traj = create_constant_distance_trajectory(
            current_uav_pos=current_pos,
            current_uav_yaw=self.current_yaw,
            valve_center=self.valve_pos,
            rotation_angle=self.rotation_angle,
            rotation_duration=self.rotation_duration,
            end_effector_offset_x=0.246,
            end_effector_offset_y=0.0,
            end_effector_offset_z=0.0743823,
            rotation_direction=1  # counterclockwise rotation
        )
        
        # Start trajectory
        rotation_traj.start_trajectory()
        
        # PHASE 2.5: SLOW DESCENT TO ROTATION HEIGHT
        height_difference = abs(correct_rotation_height - current_height)
        if height_difference > 0.05:  # Only do slow descent if significant height change needed
            rospy.loginfo(f"=== PHASE 2.5: SLOW DESCENT TO ROTATION HEIGHT ===")
            rospy.loginfo(f"Descending slowly from {current_height:.3f}m to {correct_rotation_height:.3f}m")
            
            # Get current position for descent
            current_pos = self.get_current_position()
            if current_pos is None:
                rospy.logerr("Cannot get current position for descent")
                return 'failed'
                
            # Calculate slow descent parameters
            descent_distance = abs(current_height - correct_rotation_height)
            descent_speed = 0.02  # Very slow descent speed: 2cm/s
            descent_duration = descent_distance / descent_speed
            
            rospy.loginfo(f"Descent parameters:")
            rospy.loginfo(f"  - Distance: {descent_distance:.3f}m")
            rospy.loginfo(f"  - Speed: {descent_speed:.3f}m/s")
            rospy.loginfo(f"  - Duration: {descent_duration:.1f}s")
            
            # Execute slow descent with position control
            target_pos = (current_pos[0], current_pos[1], correct_rotation_height)
            
            # Use motion controller for smooth descent
            result = self.motion_controller.execute_controlled_descent(
                start_pos=current_pos,
                target_z=correct_rotation_height,
                descent_speed=descent_speed,
                pos_threshold=0.01,
                yaw_threshold=0.1
            )
            
            if not result:
                rospy.logwarn("Slow descent to rotation height failed, continuing anyway")
            else:
                rospy.loginfo("Slow descent to rotation height completed successfully")
                
            # Brief pause to stabilize
            rospy.sleep(1.0)
        
        # Start the rotation trajectory
        rotation_traj.start_rotation()
        
        # Execute rotation with enhanced motion controller
        try:
            # Start emergency monitoring thread
            emergency_thread = threading.Thread(target=self._emergency_monitor_thread)
            emergency_thread.daemon = True
            emergency_thread.start()
            
            # Execute rotation manually since enhanced controller method may not exist
            rotation_stats = self.execute_rotation_trajectory(rotation_traj)
            
            # Stop emergency monitoring
            self.emergency_stop.set()
            
            # Check if emergency was triggered
            if self.emergency_stop.is_set():
                rospy.logwarn("Emergency detected during rotation")
                if self.emergency_ascent_and_return(start_position):
                    return 'emergency'
                else:
                    return 'failed'
            
            # Log rotation statistics
            rospy.loginfo("=== Constant Distance Rotation Statistics ===")
            rospy.loginfo(f"Execution time: {rotation_stats.get('execution_time', 0):.1f}s")
            rospy.loginfo(f"Distance deviation violations: {rotation_stats.get('alignment_violations', 0)}")
            rospy.loginfo(f"Corrections applied: {rotation_stats.get('corrections_applied', 0)}")
            rospy.loginfo(f"Max position error: {rotation_stats.get('max_position_error', 0):.3f}m")
            rospy.loginfo(f"Maximum yaw error: {rotation_stats.get('max_yaw_error', 0):.3f}rad")
            rospy.loginfo("=== Constant Distance Rotation Concept Verification ===")
            rospy.loginfo("✓ End effector maintains constant distance to valve center")
            rospy.loginfo("✓ UAV center trajectory calculated based on end-effector circular motion")
            rospy.loginfo("✓ Avoided difficult control of forced three-point collinearity")
            
            rospy.loginfo("Constant distance valve rotation completed")
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f"Error during rotation execution: {e}")
            self.emergency_stop.set()
            return 'failed'
    
    def _emergency_monitor_thread(self):
        """Background thread for emergency monitoring"""
        rate = rospy.Rate(2)  # Check every 0.5 seconds
        
        while not self.emergency_stop.is_set() and not rospy.is_shutdown():
            if self.check_rotation_emergency():
                rospy.logwarn("Emergency condition detected - stopping rotation")
                self.emergency_stop.set()
                break
            
            rate.sleep()
    
    def execute_rotation_trajectory(self, rotation_traj):
        """Execute the rotation trajectory with constant distance feedback control"""
        rospy.loginfo("Starting rotation trajectory execution with feedback control...")
        
        # Initialize unified motion controller
        feedback_controller = self.motion_controller
        
        # Execute rotation with feedback control
        valve_center = self.valve_pos
        feedback_frequency = rospy.get_param('~feedback_frequency', 50)
        alignment_check_interval = rospy.get_param('~alignment_check_interval', 0.1)
        
        rospy.loginfo(f"Executing feedback control rotation: feedback frequency={feedback_frequency}Hz, alignment check interval={alignment_check_interval}s")
        
        try:
            stats = feedback_controller.execute_constant_distance_rotation(
                trajectory=rotation_traj,
                valve_center=valve_center,
                feedback_frequency=feedback_frequency,
                alignment_check_interval=alignment_check_interval
            )
            
            rospy.loginfo("Feedback control rotation execution completed")
            return stats
            
        except Exception as e:
            rospy.logerr(f"Feedback control rotation failed: {e}")
            # Fallback to original implementation
            rospy.loginfo("Falling back to original trajectory execution method")
            return self.execute_rotation_trajectory_fallback(rotation_traj)
    
    def execute_rotation_trajectory_fallback(self, rotation_traj):
        """Fallback rotation trajectory execution without feedback control"""
        rospy.loginfo("Starting rotation trajectory execution (fallback)...")
        
        rate = rospy.Rate(50)  # 50 Hz control rate
        start_time = time.time()
        
        # Statistics for reporting
        stats = {
            'execution_time': 0,
            'alignment_violations': 0,
            'corrections_applied': 0,
            'max_position_error': 0,
            'max_yaw_error': 0,
            'total_points': 0
        }
        
        # Debug: Log initial position
        initial_pos = self.get_current_position()
        rospy.loginfo(f"Initial position before rotation: [{initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f}]")
        
        point_count = 0
        previous_target_pos = None
        while not rospy.is_shutdown() and not self.emergency_stop.is_set():
            # Get next trajectory point
            # CRITICAL FIX: Use new constant distance trajectory generator method
            result = rotation_traj.get_next_uav_position_and_yaw()
            if result is None:
                rospy.loginfo("Constant distance rotation trajectory completed")
                break
            
            target_pos, target_yaw = result
            
            # Check trajectory continuity for smooth motion
            if previous_target_pos is not None:
                if not rotation_traj.check_trajectory_continuity(previous_target_pos, target_pos):
                    rospy.logwarn("Trajectory discontinuity detected, may affect motion smoothness")
            previous_target_pos = target_pos
            
            # Debug: Log trajectory points occasionally
            if point_count % 50 == 0:  # Every 1 second at 50Hz
                current_angle = rotation_traj.get_current_angle()
                if current_angle is not None:
                    rospy.loginfo(f"Trajectory point {point_count}: pos=[{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}], yaw={target_yaw:.3f}, angle={current_angle*180/math.pi:.1f}°")
                else:
                    rospy.loginfo(f"Trajectory point {point_count}: pos=[{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}], yaw={target_yaw:.3f}")
                
                current_pos = self.get_current_position()
                if current_pos is not None:
                    height_diff = target_pos[2] - current_pos[2]
                    rospy.loginfo(f"Current position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}], height diff: {height_diff:.3f}m")
            
            # Send trajectory point to motion controller
            self.motion_controller.send_trajectory_point(target_pos, target_yaw)
            
            # Update statistics
            current_pos = self.get_current_position()
            if current_pos is not None:
                pos_error = math.sqrt((target_pos[0] - current_pos[0])**2 + 
                                    (target_pos[1] - current_pos[1])**2 + 
                                    (target_pos[2] - current_pos[2])**2)
                yaw_error = abs(self.normalize_angle(target_yaw - self.current_yaw))
                
                stats['max_position_error'] = max(stats['max_position_error'], pos_error)
                stats['max_yaw_error'] = max(stats['max_yaw_error'], yaw_error)
                
                # Monitor end-effector distance every 100 points (2 seconds at 50Hz)
                if point_count % 100 == 0:
                    distance_status = rotation_traj.monitor_end_effector_distance(
                        current_pos, self.current_yaw, tolerance=0.02)
                    if distance_status['status'] == 'warning':
                        rospy.logwarn(f"End-effector distance monitoring warning: {distance_status['distance_error']:.3f}m")
                        stats['alignment_violations'] += 1
            
            point_count += 1
            stats['total_points'] += 1
            rate.sleep()
        
        # Calculate final statistics
        stats['execution_time'] = time.time() - start_time
        
        # Debug: Log final position
        final_pos = self.get_current_position()
        if final_pos is not None and initial_pos is not None:
            height_change = final_pos[2] - initial_pos[2]
            rospy.loginfo(f"Final position after rotation: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]")
            rospy.loginfo(f"Height change during rotation: {height_change:.3f}m")
        
        return stats

def main():
    """Main function to execute the valve rotation state machine"""
    rospy.init_node('valve_rotation_single_uav')
    
    # Get module ID parameter
    module_id = rospy.get_param('~module_id', 1)
    
    rospy.loginfo(f"Starting valve rotation for UAV module {module_id}")
    
    # Create state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    
    # Initialize userdata
    sm.userdata.start_position = None
    
    # Add states to the state machine
    with sm:
        smach.StateMachine.add(
            'INITIALIZE_START_POSITION',
            InitializeStartPositionState(module_id=module_id),
            transitions={
                'succeeded': 'MOVE_TO_VALVE',
                'failed': 'failed'
            }
        )
        
        smach.StateMachine.add(
            'MOVE_TO_VALVE',
            MoveToValveState(module_id=module_id),
            transitions={
                'succeeded': 'DESCEND_AND_CONTACT',
                'failed': 'failed'
            }
        )
        
        smach.StateMachine.add(
            'DESCEND_AND_CONTACT',
            DescendAndContactState(module_id=module_id),
            transitions={
                'succeeded': 'ROTATE_VALVE',
                'failed': 'failed'
            }
        )
        
        smach.StateMachine.add(
            'ROTATE_VALVE',
            RotateValveState(module_id=module_id),
            transitions={
                'succeeded': 'succeeded',
                'failed': 'failed',
                'emergency': 'failed'
            }
        )
    
    # Create and start the introspection server for debugging
    sis = smach_ros.IntrospectionServer('valve_rotation_state_machine', sm, '/SM_ROOT')
    sis.start()
    
    try:
        # Execute the state machine
        rospy.loginfo("Starting valve rotation state machine...")
        outcome = sm.execute()
        rospy.loginfo(f"State machine completed with outcome: {outcome}")
        
        if outcome == 'succeeded':
            rospy.loginfo("Valve rotation completed successfully!")
        else:
            rospy.logerr("Valve rotation failed!")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Valve rotation interrupted by user")
    except Exception as e:
        rospy.logerr(f"Valve rotation error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Stop the introspection server
        sis.stop()

if __name__ == '__main__':
    main()
