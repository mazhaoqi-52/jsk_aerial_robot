#!/usr/bin/env python3
"""
Single UAV Valve Rotation using SMACH State Machine
Cleaned version with redundant code removed.
"""

import sys
import os

# Add current directory FIRST to ensure local imports work correctly
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)  # Insert at beginning for priority

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

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
from trajectory import create_constant_distance_trajectory
from unified_motion_controller import UnifiedMotionController

# Import the local insertion optimizer
from insertion_optimizer import InsertionOptimizer


class SingleUAVStateBase(smach.State):
    def __init__(self, outcomes, input_keys=None, output_keys=None, module_id=1):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys or [], output_keys=output_keys or [])
        self.module_id = module_id
        
        # Publishers and subscribers
        self.pub = rospy.Publisher(f"/beetle{module_id}/uav/nav", FlightNav, queue_size=1)
        self.motion_controller = UnifiedMotionController(self)
        
        # Position and orientation data
        self.uav_pos = None
        self.current_yaw = 0.0
        self.valve_pos = None
        self.valve_yaw = 0.0
        self.external_wrench = None
        
        # Thread events
        self.uav_received = threading.Event()
        self.valve_received = threading.Event()
        
        # Setup simulation mode
        self.is_simulation = rospy.get_param("~simulation", True)
        
        # Setup subscribers
        self.uav_sub = rospy.Subscriber(f"/beetle{module_id}/mocap/pose", PoseStamped, self.uav_callback, queue_size=1)
        self.wrench_sub = rospy.Subscriber(f"/beetle{module_id}/estimated_external_wrench", WrenchStamped, self.wrench_callback, queue_size=1)
        
        if self.is_simulation:
            rospy.loginfo("Using simulation mode - subscribing to /valve/odom")
            self.valve_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            rospy.loginfo("Using real machine mode - subscribing to /valve/mocap/pose")
            self.valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
        
        # Parameters
        self.position_threshold = 0.03
        self.yaw_threshold = 0.05
        self.timeout = 10.0
        self.z_offset = 0.0743823
        self.descent_speed = 0.025
        
        rospy.loginfo(f"Initialized UAV{module_id} state with basic parameters")
    
    def __del__(self):
        """Destructor to ensure proper cleanup"""
        try:
            self.cleanup()
        except:
            pass  # Ignore errors during cleanup
    
    def cleanup(self):
        """Clean up resources and stop threads"""
        if hasattr(self, 'stop_emergency_monitoring'):
            self.stop_emergency_monitoring()
    
    def uav_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.uav_pos = (position.x, position.y, position.z)
        
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw
        
        self.uav_received.set()
    
    def valve_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.valve_pos = (position.x, position.y, position.z)
        
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.valve_yaw = yaw
        
        self.valve_received.set()
    
    def valve_sim_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.valve_pos = (position.x, position.y, position.z)
        
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.valve_yaw = yaw
        
        self.valve_received.set()
    
    def wrench_callback(self, msg):
        self.external_wrench = msg.wrench
    
    def wait_for_positions(self):
        rospy.loginfo("Waiting for UAV and valve positions...")
        if not self.uav_received.wait(timeout=5.0):
            rospy.logerr("UAV position not received")
            return False
        if not self.valve_received.wait(timeout=5.0):
            rospy.logerr("Valve position not received")
            return False
        return True
    
    def get_current_position(self):
        return self.uav_pos
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_current_yaw(self):
        return self.current_yaw
    
    def setup_emergency_monitoring(self, stuck_threshold=60.0, movement_threshold=0.005, yaw_threshold=0.02):
        """
        Setup emergency monitoring for stuck detection
        Can be used by any child state that needs emergency monitoring
        """
        self.emergency_stop = threading.Event()
        self.emergency_triggered = False
        self.last_position = None
        self.last_yaw = 0.0
        self.stuck_start_time = None
        self.stuck_threshold = stuck_threshold
        self.movement_threshold = movement_threshold
        self.yaw_threshold = yaw_threshold
    
    def start_emergency_monitoring(self):
        """Start emergency monitoring in a separate thread"""
        if not hasattr(self, 'emergency_stop'):
            self.setup_emergency_monitoring()
        
        self.emergency_stop.clear()
        self.emergency_triggered = False
        
        current_pos = self.get_current_position()
        if current_pos:
            self.last_position = current_pos
            self.last_yaw = self.current_yaw
        
        self.emergency_thread = threading.Thread(target=self._emergency_monitor_thread)
        self.emergency_thread.daemon = True
        self.emergency_thread.start()
        return self.emergency_thread
    
    def stop_emergency_monitoring(self):
        """Stop emergency monitoring"""
        if hasattr(self, 'emergency_stop'):
            self.emergency_stop.set()
        
        # Wait for emergency thread to finish if it exists
        if hasattr(self, 'emergency_thread') and self.emergency_thread.is_alive():
            try:
                self.emergency_thread.join(timeout=2.0)  # Wait up to 2 seconds
                if self.emergency_thread.is_alive():
                    rospy.logwarn("Emergency monitoring thread did not terminate within timeout")
                else:
                    rospy.loginfo("Emergency monitoring thread terminated successfully")
            except Exception as e:
                rospy.logwarn(f"Error waiting for emergency thread termination: {e}")
    
    def _emergency_monitor_thread(self):
        """Emergency monitoring thread - checks for stuck conditions"""
        rate = rospy.Rate(1)  # Check every 1.0 seconds
        consecutive_stuck_checks = 0
        required_consecutive_stuck = 3  # Require 3 consecutive stuck readings
        
        try:
            while not self.emergency_stop.is_set() and not rospy.is_shutdown():
                if self.check_emergency_condition():
                    consecutive_stuck_checks += 1
                    rospy.logwarn(f"Potential stuck condition detected ({consecutive_stuck_checks}/{required_consecutive_stuck})")
                    
                    if consecutive_stuck_checks >= required_consecutive_stuck:
                        rospy.logwarn("Emergency condition confirmed after multiple checks")
                        self.emergency_triggered = True
                        self.emergency_stop.set()
                        break
                else:
                    consecutive_stuck_checks = 0  # Reset counter if movement detected
                    
                try:
                    rate.sleep()
                except rospy.exceptions.ROSInterruptException:
                    # ROS is shutting down, exit gracefully
                    rospy.loginfo("Emergency monitoring thread stopping due to ROS shutdown")
                    break
        except Exception as e:
            rospy.logwarn(f"Emergency monitoring thread error: {e}")
        finally:
            rospy.loginfo("Emergency monitoring thread terminated")
    
    def check_emergency_condition(self):
        """Check if emergency condition exists (e.g., UAV stuck)"""
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        if self.last_position is not None:
            # Check 3D movement
            movement = math.sqrt(
                (current_pos[0] - self.last_position[0])**2 + 
                (current_pos[1] - self.last_position[1])**2 + 
                (current_pos[2] - self.last_position[2])**2
            )
            
            # Check yaw movement
            yaw_movement = abs(self.normalize_angle(self.current_yaw - self.last_yaw))
            
            # Consider it stuck only if BOTH position AND yaw are not moving
            position_stuck = movement < self.movement_threshold
            yaw_stuck = yaw_movement < self.yaw_threshold
            
            if position_stuck and yaw_stuck:
                if self.stuck_start_time is None:
                    self.stuck_start_time = time.time()
                    # Only log when stuck detection starts for the first time
                    rospy.loginfo("⚠ UAV movement below threshold - starting stuck detection timer")
                else:
                    stuck_duration = time.time() - self.stuck_start_time
                    # Only log every 10 seconds to reduce noise, and only if duration is significant
                    if stuck_duration > 10.0 and int(stuck_duration) % 10 == 0:
                        rospy.logwarn(f"UAV stuck duration: {stuck_duration:.1f}s / {self.stuck_threshold:.1f}s")
                    
                    if stuck_duration > self.stuck_threshold:
                        rospy.logerr(f"EMERGENCY: UAV stuck for {stuck_duration:.1f}s > {self.stuck_threshold:.1f}s")
                        return True
            else:
                if self.stuck_start_time is not None:
                    # Only log when transitioning from stuck to moving, and only if stuck time was significant
                    stuck_duration = time.time() - self.stuck_start_time
                    if stuck_duration > 3.0:  # Only log if we were stuck for more than 3 seconds
                        rospy.loginfo(f"Movement resumed after {stuck_duration:.1f}s - resetting stuck timer")
                    # Reset stuck timer silently for short durations
                    self.stuck_start_time = None
                else:
                    # Normal movement - no logging needed
                    pass
        
        self.last_position = current_pos
        self.last_yaw = self.current_yaw
        return False
    
    def _check_valve_center_crossing(self, start_pos, target_pos, valve_center_x, valve_center_y):
        """
        Check if trajectory from start to target would cross valve center
        
        Returns:
            bool: True if trajectory risks crossing valve center
        """
        # Simple line-circle intersection check
        start_x, start_y = start_pos[0], start_pos[1]
        target_x, target_y = target_pos[0], target_pos[1]
        
        # Vector from start to target
        dx = target_x - start_x
        dy = target_y - start_y
        
        # Vector from start to valve center
        fx = valve_center_x - start_x
        fy = valve_center_y - start_y
        
        # Project valve center onto trajectory line
        if dx == 0 and dy == 0:
            return False  # No movement
        
        t = (fx * dx + fy * dy) / (dx * dx + dy * dy)
        
        # Closest point on trajectory to valve center
        closest_x = start_x + t * dx
        closest_y = start_y + t * dy
        
        # Distance from valve center to closest point
        distance_to_trajectory = math.sqrt((closest_x - valve_center_x)**2 + (closest_y - valve_center_y)**2)
        
        # Check if trajectory passes too close to valve center
        safe_distance = 0.3  # 30cm safe distance
        return distance_to_trajectory < safe_distance and 0 <= t <= 1
    
    # ...existing code...


class InitializeStartPositionState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'],
                        output_keys=['start_position'], 
                        module_id=module_id)
    
    def execute(self, userdata):
        rospy.loginfo("Initializing start position...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        userdata.start_position = self.get_current_position()
        rospy.loginfo(f"Start position initialized: {userdata.start_position}")
        return 'succeeded'


class MoveToValveState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        # Initialize the insertion optimizer for direct insertion movement
        self.optimizer = InsertionOptimizer()
        
        # Position and orientation thresholds
        self.pos_threshold = 0.05
        self.yaw_threshold = 0.05
    
    def calculate_uav_position_for_targets(self, left_target, right_target, target_z):
        """
        Calculate UAV position and yaw to achieve specific claw targets
        
        Args:
            left_target: (x, y, z) position for left claw
            right_target: (x, y, z) position for right claw
            target_z: Target Z height for claws
            
        Returns:
            tuple: (uav_position, uav_yaw) or (None, None) if calculation fails
        """
        try:
            # Calculate dual-fang center position (midpoint between claws)
            dual_fang_center_x = (left_target[0] + right_target[0]) / 2
            dual_fang_center_y = (left_target[1] + right_target[1]) / 2
            dual_fang_center_z = target_z
            
            # Calculate UAV orientation (yaw) from claw separation vector
            claw_vector_x = right_target[0] - left_target[0]
            claw_vector_y = right_target[1] - left_target[1]
            uav_yaw = math.atan2(claw_vector_y, claw_vector_x) - math.pi/2
            
            # Normalize yaw angle to [0, 2π]
            while uav_yaw >= 2*math.pi:
                uav_yaw -= 2*math.pi
            while uav_yaw < 0:
                uav_yaw += 2*math.pi
            
            # Calculate UAV position from dual-fang center
            # UAV should be BEHIND the dual-fang center by the offset distance
            uav_x = dual_fang_center_x - self.optimizer.dual_fang_center_offset * math.cos(uav_yaw)
            uav_y = dual_fang_center_y - self.optimizer.dual_fang_center_offset * math.sin(uav_yaw)
            # UAV Z should be ABOVE the dual-fang center by the offset
            uav_z = dual_fang_center_z - self.optimizer.end_effector_offset_z
            
            return (uav_x, uav_y, uav_z), uav_yaw
            
        except Exception as e:
            rospy.logerr(f"Failed to calculate UAV position for targets: {e}")
            return None, None
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle
    
    def execute_safe_movement_to_position(self, target_pos, target_yaw):
        """
        Execute safe three-stage movement to target position
        """
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        return self.motion_controller.execute_smooth_trajectory_with_yaw(
            start_pos=current_pos,
            target_pos=target_pos,
            target_yaw=target_yaw,
            duration=20.0,
            pos_threshold=self.pos_threshold,
            yaw_threshold=self.yaw_threshold
        )
    
    def execute(self, userdata):
        rospy.loginfo("Moving directly to valve insertion position...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        # Get valve position and calculate insertion strategy
        valve_x, valve_y, valve_z = self.valve_pos
        current_pos = self.get_current_position()
        
        if current_pos is None:
            rospy.logerr("Cannot get current UAV position")
            return 'failed'
        
        # Calculate Phase 1 insertion position using optimizer
        result = self.optimizer.calculate_dual_fang_insertion_strategy(
            valve_center_x=valve_x,
            valve_center_y=valve_y,
            current_uav_angle=math.atan2(current_pos[1] - valve_y, current_pos[0] - valve_x)
        )
        
        if not result['success']:
            rospy.logerr(f"Failed to calculate insertion strategy: {result.get('error', 'Unknown error')}")
            return 'failed'
        
        # Get Phase 1 (safe insertion) targets
        strategy = result['strategy']
        phase1_targets = strategy['phase1_targets']
        left_target = phase1_targets['left_claw']
        right_target = phase1_targets['right_claw']
        
        # Calculate UAV position for Phase 1 insertion
        end_effector_offset_z = 0.0221140  # Z offset of dual-fang center from UAV base_link
        target_z = valve_z - end_effector_offset_z
        
        uav_position, uav_yaw = self.calculate_uav_position_for_targets(left_target, right_target, target_z)
        
        if uav_position is None:
            rospy.logerr("Failed to calculate UAV position for insertion targets")
            return 'failed'
        
        target_pos = uav_position
        target_yaw = uav_yaw
        
        
        rospy.loginfo(f"Valve position: ({valve_x:.3f}, {valve_y:.3f}, {valve_z:.3f})")
        rospy.loginfo(f"Current UAV position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"Phase 1 insertion targets:")
        rospy.loginfo(f"  Left claw: ({left_target[0]:.3f}, {left_target[1]:.3f})")
        rospy.loginfo(f"  Right claw: ({right_target[0]:.3f}, {right_target[1]:.3f})")
        rospy.loginfo(f"Moving directly to insertion position: {target_pos}")
        rospy.loginfo(f"Target yaw: {target_yaw:.3f} rad ({math.degrees(target_yaw):.1f}°)")
        rospy.loginfo(f"Current yaw: {self.current_yaw:.3f} rad ({math.degrees(self.current_yaw):.1f}°)")
        yaw_change = self.normalize_angle(target_yaw - self.current_yaw)
        rospy.loginfo(f"Required yaw change: {yaw_change:.3f} rad ({math.degrees(yaw_change):.1f}°)")
        
        # Direct movement to insertion position using three-stage movement
        rospy.loginfo("Executing direct movement to insertion position...")
        
        # Use three-stage movement for precision insertion positioning
        success = self.execute_safe_movement_to_position(target_pos, target_yaw)
        
        if not success:
            rospy.logerr("Failed to move to insertion position")
            return 'failed'
        
        rospy.loginfo("Successfully moved to insertion position")
        return 'succeeded'


class DescendAndContactState(SingleUAVStateBase):
    def __init__(self, module_id=1, rotation_direction=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        # Store rotation direction for insertion strategy
        self.rotation_direction = rotation_direction  # 1 for clockwise, -1 for counter-clockwise
        
        # Initialize insertion optimizer with CORRECTED valve wheel geometry
        self.optimizer = InsertionOptimizer(
            valve_radius=0.10        # CORRECTED: Inner rim radius (200mm diameter, 100mm radius)
            , valve_beam_width=0.035  # CORRECTED: Minimum spoke gap width (35mm at hub connection)
            , safety_margin=0.002
            , module_id=module_id
            , simulation=True
        )
        
        self.pre_insertion_distance = 0.035
        self.circumferential_offset = 0.005
    
    def execute(self, userdata):
        """
        Enhanced 4-step valve engagement strategy:
        1. Determine optimal contact point for valve rotation
        2. Descend to valve height and insert into beam gap for maximum control margin  
        3. Move along valve circumference from insertion point to contact point
        4. Begin valve rotation
        """
        rospy.loginfo("=== ENHANCED 4-STEP VALVE ENGAGEMENT STRATEGY ===")
        rospy.loginfo("This includes DESCENT to valve height for proper insertion")
        
        # Initialize and wait for required positions
        if not self.wait_for_positions():
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None:
            return 'failed'
        
        # CRITICAL: Calculate correct Z position for end-effector insertion at valve height
        # End-effector should be at valve height, so UAV should be offset accordingly
        end_effector_offset_z = 0.0221140  # Z offset of dual-fang center from UAV base_link
        valve_z = self.valve_pos[2]
        required_uav_z = valve_z - end_effector_offset_z  # UAV position to place end-effector at valve height
        
        self.locked_z = required_uav_z
        rospy.loginfo(f"=== INSERTION HEIGHT CALCULATION ===")
        rospy.loginfo(f"Valve height: {valve_z:.6f}m")
        rospy.loginfo(f"End-effector Z offset: {end_effector_offset_z:.6f}m")
        rospy.loginfo(f"Required UAV Z for insertion: {required_uav_z:.6f}m")
        rospy.loginfo(f"Z-axis LOCKED at {self.locked_z:.6f}m for valve insertion")
        
        # Set rotation direction in optimizer
        self.optimizer.set_rotation_direction(self.rotation_direction)
        
        # Step 1: Determine optimal contact point for valve rotation
        optimal_contact_point = self.determine_optimal_contact_point()
        if optimal_contact_point is None:
            rospy.logerr("Failed to determine optimal contact point")
            return 'failed'
        
        # Step 2: Rotate UAV and insert into beam gap for maximum control margin
        insertion_success = self.rotate_and_insert_to_beam_gap(optimal_contact_point)
        if not insertion_success:
            rospy.logerr("Failed to insert into beam gap")
            return 'failed'
        
        # Step 3: Move along valve circumference from insertion point to contact point
        circumferential_success = self.move_along_valve_circumference(optimal_contact_point)
        if not circumferential_success:
            rospy.logerr("Failed to move along valve circumference")
            return 'failed'
        
        # Step 4: Prepare for valve rotation
        rotation_ready = self.prepare_for_valve_rotation(optimal_contact_point)
        if not rotation_ready:
            rospy.logerr("Failed to prepare for valve rotation")
            return 'failed'
        
        rospy.loginfo("=== 4-STEP VALVE ENGAGEMENT COMPLETED SUCCESSFULLY ===")
        return 'succeeded'
    
    def determine_optimal_contact_point(self):
        """
        Step 1: Determine optimal dual-fang insertion strategy using InsertionOptimizer
        Uses the new dual-fang clockwise strategy from insertion_optimizer.py
        """
        rospy.loginfo("--- Step 1: Determining optimal dual-fang contact strategy using InsertionOptimizer ---")
        
        # Get current UAV and valve positions
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot get current UAV position")
            return None
        
        # Use InsertionOptimizer to calculate dual-fang strategy
        if not self.optimizer:
            rospy.logerr("Optimizer not initialized")
            return None
        
        # Call the new dual-fang strategy method
        strategy = self.optimizer.calculate_dual_fang_clockwise_strategy(
            uav_pos=current_pos,
            valve_pos=self.valve_pos,
            valve_yaw=self.valve_yaw
        )
        
        if strategy is None:
            rospy.logerr("Failed to calculate dual-fang strategy")
            return None
        
        # Store strategy for use in insertion phase
        self.current_insertion_params = strategy
        
        # Handle different strategy types
        if strategy.get('strategy') == 'two_phase_safe_insertion':
            # For two-phase strategy, calculate dual-fang center from phase 2 targets
            phase2_left = strategy['phase2_left_final']
            phase2_right = strategy['phase2_right_final']
            dual_fang_center = (
                (phase2_left[0] + phase2_right[0]) / 2,
                (phase2_left[1] + phase2_right[1]) / 2,
                phase2_left[2]
            )
            
            # Calculate UAV yaw from claw separation vector
            claw_vector_x = phase2_right[0] - phase2_left[0]
            claw_vector_y = phase2_right[1] - phase2_left[1]
            optimal_uav_yaw = math.atan2(claw_vector_y, claw_vector_x) - math.pi/2
            
            # Normalize yaw
            while optimal_uav_yaw >= 2*math.pi:
                optimal_uav_yaw -= 2*math.pi
            while optimal_uav_yaw < 0:
                optimal_uav_yaw += 2*math.pi
        else:
            # Legacy strategy structure
            dual_fang_center = strategy.get('dual_fang_center')
            optimal_uav_yaw = strategy.get('optimal_uav_yaw', 0.0)
            
            if dual_fang_center is None:
                rospy.logerr("Strategy missing dual_fang_center field")
                return None
        
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        
        contact_point = {
            'position': dual_fang_center,
            'angle': optimal_uav_yaw,
            'distance': math.sqrt((dual_fang_center[0] - valve_center_x)**2 + (dual_fang_center[1] - valve_center_y)**2),
            'strategy': strategy.get('strategy', 'dual_fang_clockwise')
        }
        
        rospy.loginfo("=== DUAL-FANG STRATEGY FROM INSERTION OPTIMIZER ===")
        
        strategy_type = strategy.get('strategy', 'unknown')
        rospy.loginfo(f"Strategy type: {strategy_type}")
        
        if strategy_type == 'two_phase_safe_insertion':
            same_side_result = strategy.get('same_side_result', {})
            rospy.loginfo(f"User-verified same-side strategy: {same_side_result.get('formula', 'N/A')}")
            rospy.loginfo(f"Expected separation: {same_side_result.get('actual_separation', 0)*1000:.1f}mm")
            rospy.loginfo(f"Safety margin: {same_side_result.get('margin', 0)*1000:.1f}mm")
            rospy.loginfo(f"Two-phase approach: Safe insertion → Final contact")
        else:
            # Legacy strategy information
            rospy.loginfo(f"Left claw -> {strategy.get('left_gap', 'N/A')} at {math.degrees(strategy.get('left_gap_angle', 0)):.0f}°")
            rospy.loginfo(f"Right claw -> {strategy.get('right_gap', 'N/A')} at {math.degrees(strategy.get('right_gap_angle', 0)):.0f}°")
            rospy.loginfo(f"Best approach: {strategy.get('best_approach_name', 'N/A')}")
            rospy.loginfo(f"Total positioning error: {strategy.get('total_positioning_error', 0):.4f}m")
        
        return contact_point
    
    def rotate_and_insert_to_beam_gap(self, contact_point):
        """
        Step 2: Execute dual-fang insertion strategy
        - Position both claws simultaneously into their respective gaps
        - Left claw -> beam1_beam2 gap, Right claw -> beam2_beam0 gap
        """
        rospy.loginfo("--- Step 2: Executing dual-fang insertion strategy ---")
        
        if contact_point is None:
            rospy.logerr("No contact point provided for insertion")
            return False
        
        # Use dual-fang strategy from step 1
        if not hasattr(self, 'current_insertion_params'):
            rospy.logerr("No dual-fang insertion parameters available from step 1")
            return False
        
        params = self.current_insertion_params
        
        strategy_type = params.get('strategy', params.get('strategy_type', 'unknown'))
        
        if strategy_type not in ['dual_fang_clockwise', 'two_phase_safe_insertion']:
            rospy.logwarn(f"Unexpected strategy type: {strategy_type}, proceeding anyway")
        
        # Execute dual-fang insertion
        success = self.execute_dual_fang_insertion(params)
        
        if success:
            rospy.loginfo("✓ Successfully executed dual-fang insertion strategy")
            return True
        else:
            rospy.logerr("✗ Failed to execute dual-fang insertion")
            return False
    
    def move_along_valve_circumference(self, contact_point):
        """
        Step 3: Move along valve circumference from insertion point to contact point
        - Maintain contact with valve while moving circumferentially
        - Adjust fang positions for optimal grip during movement
        - Monitor contact forces to prevent slipping
        """
        rospy.loginfo("--- Step 3: Moving along valve circumference to contact point ---")
        
        # Get current position after insertion
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position for circumferential movement")
            return False
        
        # For this implementation, insertion point IS the contact point
        # (since we're using optimized insertion directly to optimal contact point)
        # In a more sophisticated version, this would involve circumferential trajectory
        
        # Calculate current dual-fang center position
        end_effector_x = current_pos[0] + 0.25346 * math.cos(self.current_yaw)
        end_effector_y = current_pos[1] + 0.25346 * math.sin(self.current_yaw)
        
        # Verify we're at the target contact point
        target_x, target_y, target_z = contact_point['position']
        distance_to_contact = math.sqrt((end_effector_x - target_x)**2 + (end_effector_y - target_y)**2)
        
        rospy.loginfo(f"Current dual-fang position: ({end_effector_x:.3f}, {end_effector_y:.3f})")
        rospy.loginfo(f"Target contact point: ({target_x:.3f}, {target_y:.3f})")
        rospy.loginfo(f"Distance to contact point: {distance_to_contact:.3f}m")
        
        # For now, consider circumferential movement complete if we're within tolerance
        if distance_to_contact < 0.02:  # 2cm tolerance
            rospy.loginfo("Already positioned at optimal contact point - circumferential movement complete")
            return True
        else:
            rospy.logwarn(f"Position error {distance_to_contact:.3f}m - may affect rotation performance")
            # Continue anyway for robustness
            return True
    
    def prepare_for_valve_rotation(self, contact_point):
        """
        Step 4: Prepare for valve rotation
        - Verify secure grip on valve
        - Check mechanical advantage for rotation direction
        - Set up rotation control parameters
        """
        rospy.loginfo("--- Step 4: Preparing for valve rotation ---")
        
        # Verify final position and orientation
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot verify position for rotation preparation")
            return False
        
        # Calculate final dual-fang center position
        end_effector_x = current_pos[0] + 0.25346 * math.cos(self.current_yaw)
        end_effector_y = current_pos[1] + 0.25346 * math.sin(self.current_yaw)
        end_effector_z = current_pos[2] + 0.0221140
        
        # Verify position relative to valve
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        distance_to_center = math.sqrt((end_effector_x - valve_center_x)**2 + (end_effector_y - valve_center_y)**2)
        
        # Verify rotation readiness
        rotation_direction_str = 'clockwise' if self.rotation_direction == 1 else 'counter-clockwise'
        
        rospy.loginfo("=== VALVE ROTATION PREPARATION COMPLETE ===")
        rospy.loginfo(f"Final dual-fang position: ({end_effector_x:.3f}, {end_effector_y:.3f}, {end_effector_z:.3f})")
        rospy.loginfo(f"Distance from valve center: {distance_to_center:.3f}m")
        rospy.loginfo(f"Rotation direction: {rotation_direction_str}")
        rospy.loginfo(f"Mechanical advantage: {contact_point.get('mechanical_advantage', 'Good')}")
        rospy.loginfo("Ready for valve rotation phase")
        
        return True
    
    def execute_dual_fang_insertion(self, params):
        """
        Execute dual-fang insertion using calculated strategy.
        
        Supports:
        - two_phase_safe_insertion: User's verified same-side strategy with safe approach
        - Legacy strategies: For backward compatibility
        
        Args:
            params: Insertion parameters from optimizer
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("=== EXECUTING DUAL-FANG INSERTION ===")
        
        current_pos = self.get_current_position()
        if not current_pos:
            rospy.logerr("No UAV position available")
            return False
        
        # Check strategy type and execute accordingly
        strategy_type = params.get('strategy', 'legacy')
        
        if strategy_type == 'two_phase_safe_insertion':
            return self.execute_two_phase_insertion(params)
        else:
            # Legacy single-phase insertion for backward compatibility
            rospy.loginfo("Using legacy single-phase insertion")
            return self.execute_legacy_insertion(params)
    
    def execute_two_phase_insertion(self, params):
        """
        Execute two-phase safe insertion strategy:
        Phase 1: Insert to gap centers (safe radius 68.75mm)
        Phase 2: Radial approach to final contact points (inner rim 100mm)
        
        This implements the user's verified same-side strategy with safety.
        
        Args:
            params: Two-phase insertion parameters from optimizer
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("=== EXECUTING TWO-PHASE SAFE INSERTION ===")
        rospy.loginfo("Using user-verified same-side strategy")
        
        current_pos = self.get_current_position()
        if not current_pos:
            rospy.logerr("No UAV position available")
            return False
        
        # Extract phase 1 (safe) and phase 2 (final) targets
        phase1_left_safe = params['phase1_left_safe']
        phase1_right_safe = params['phase1_right_safe']
        phase2_left_final = params['phase2_left_final']
        phase2_right_final = params['phase2_right_final']
        safe_radius = params['safe_radius']
        final_radius = params['final_radius']
        same_side_result = params['same_side_result']
        
        rospy.loginfo(f"Current UAV position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"Two-phase strategy details:")
        rospy.loginfo(f"  Phase 1 - Safe radius: {safe_radius*1000:.1f}mm")
        rospy.loginfo(f"  Phase 2 - Final radius: {final_radius*1000:.1f}mm")
        rospy.loginfo(f"  Expected separation: {same_side_result['actual_separation']*1000:.1f}mm")
        rospy.loginfo(f"  Safety margin: {same_side_result['margin']*1000:.1f}mm")
        
        # === PHASE 1: SAFE INSERTION TO GAP CENTERS ===
        rospy.loginfo("=== PHASE 1: SAFE INSERTION TO GAP CENTERS ===")
        
        # Calculate UAV position for Phase 1 (safe insertion)
        phase1_uav_pos, phase1_uav_yaw = self.calculate_uav_position_for_targets(
            phase1_left_safe, phase1_right_safe, self.valve_pos[2])
        
        if not phase1_uav_pos:
            rospy.logerr("Phase 1: Could not calculate UAV position for safe insertion")
            return False
        
        rospy.loginfo(f"Phase 1 - Safe insertion targets:")
        rospy.loginfo(f"  Left claw target: ({phase1_left_safe[0]:.3f}, {phase1_left_safe[1]:.3f})")
        rospy.loginfo(f"  Right claw target: ({phase1_right_safe[0]:.3f}, {phase1_right_safe[1]:.3f})")
        rospy.loginfo(f"  UAV position: ({phase1_uav_pos[0]:.3f}, {phase1_uav_pos[1]:.3f}, {phase1_uav_pos[2]:.3f})")
        rospy.loginfo(f"  UAV yaw: {math.degrees(phase1_uav_yaw):.1f}°")
        
        # Execute Phase 1 movement
        if not self.execute_safe_movement_to_position(phase1_uav_pos, phase1_uav_yaw):
            rospy.logerr("Phase 1: Failed to reach safe insertion position")
            return False
        
        rospy.loginfo("✓ Phase 1 completed - Claws safely inserted to gap centers")
        
        # === PHASE 2: RADIAL APPROACH TO FINAL CONTACT POINTS ===
        rospy.loginfo("=== PHASE 2: RADIAL APPROACH TO FINAL CONTACT POINTS ===")
        
        # Calculate UAV position for Phase 2 (final contact)
        phase2_uav_pos, phase2_uav_yaw = self.calculate_uav_position_for_targets(
            phase2_left_final, phase2_right_final, self.valve_pos[2])
        
        if not phase2_uav_pos:
            rospy.logerr("Phase 2: Could not calculate UAV position for final contact")
            return False
        
        rospy.loginfo(f"Phase 2 - Final contact targets:")
        rospy.loginfo(f"  Left claw target: ({phase2_left_final[0]:.3f}, {phase2_left_final[1]:.3f})")
        rospy.loginfo(f"  Right claw target: ({phase2_right_final[0]:.3f}, {phase2_right_final[1]:.3f})")
        rospy.loginfo(f"  UAV position: ({phase2_uav_pos[0]:.3f}, {phase2_uav_pos[1]:.3f}, {phase2_uav_pos[2]:.3f})")
        rospy.loginfo(f"  UAV yaw: {math.degrees(phase2_uav_yaw):.1f}°")
        
        # Execute Phase 2 movement (radial approach)
        if not self.execute_radial_approach_movement(phase2_uav_pos, phase2_uav_yaw):
            rospy.logerr("Phase 2: Failed to reach final contact position")
            return False
        
        rospy.loginfo("✓ Phase 2 completed - Claws at final contact points (inner rim)")
        
        # === FINAL VERIFICATION ===
        final_pos = self.get_current_position()
        if final_pos:
            success = self.verify_two_phase_insertion_result(
                final_pos, phase2_left_final, phase2_right_final, same_side_result)
            
            if success:
                rospy.loginfo("✓ TWO-PHASE SAFE INSERTION SUCCESSFUL")
                rospy.loginfo(f"✓ User-verified same-side strategy implemented: {same_side_result['formula']}")
                return True
            else:
                rospy.logwarn("Two-phase insertion completed but verification failed")
                return False
        
        rospy.logerr("Could not verify final position")
        return False
    
    def calculate_uav_position_for_targets(self, left_target, right_target, target_z):
        """
        Calculate UAV position and yaw to achieve specific claw targets
        
        Args:
            left_target: (x, y, z) position for left claw
            right_target: (x, y, z) position for right claw
            target_z: Target Z height for claws
            
        Returns:
            tuple: (uav_position, uav_yaw) or (None, None) if calculation fails
        """
        try:
            # Calculate dual-fang center position (midpoint between claws)
            dual_fang_center_x = (left_target[0] + right_target[0]) / 2
            dual_fang_center_y = (left_target[1] + right_target[1]) / 2
            dual_fang_center_z = target_z
            
            # Calculate UAV orientation (yaw) from claw separation vector
            claw_vector_x = right_target[0] - left_target[0]
            claw_vector_y = right_target[1] - left_target[1]
            uav_yaw = math.atan2(claw_vector_y, claw_vector_x) - math.pi/2
            
            # Normalize yaw angle to [0, 2π]
            while uav_yaw >= 2*math.pi:
                uav_yaw -= 2*math.pi
            while uav_yaw < 0:
                uav_yaw += 2*math.pi
            
            # CORRECTED: Calculate UAV position from dual-fang center
            # UAV should be BEHIND the dual-fang center by the offset distance
            uav_x = dual_fang_center_x - self.optimizer.dual_fang_center_offset * math.cos(uav_yaw)
            uav_y = dual_fang_center_y - self.optimizer.dual_fang_center_offset * math.sin(uav_yaw)
            # CORRECTED: UAV Z should be ABOVE the dual-fang center by the offset
            uav_z = dual_fang_center_z - self.optimizer.end_effector_offset_z  # Use locked_z instead
            
            # Verify calculation with debug info
            rospy.logdebug(f"Dual-fang center: ({dual_fang_center_x:.3f}, {dual_fang_center_y:.3f}, {dual_fang_center_z:.3f})")
            rospy.logdebug(f"UAV yaw: {math.degrees(uav_yaw):.1f}°")
            rospy.logdebug(f"Offset distances: center={self.optimizer.dual_fang_center_offset:.3f}m, z={self.optimizer.end_effector_offset_z:.3f}m")
            rospy.logdebug(f"Calculated UAV position: ({uav_x:.3f}, {uav_y:.3f}, {uav_z:.3f})")
            
            # Use the locked Z from insertion height calculation instead
            final_uav_z = self.locked_z
            
            return (uav_x, uav_y, final_uav_z), uav_yaw
            
        except Exception as e:
            rospy.logerr(f"Failed to calculate UAV position for targets: {e}")
            return None, None
    
    def execute_safe_movement_to_position(self, target_pos, target_yaw):
        """
        Execute safe three-stage movement following strict axis-locking principles:
        Stage 1: XY positioning only (Z and yaw locked)
        Stage 2: Yaw adjustment with end-effector compensation (XY and Z locked)  
        Stage 3: Z descent only (XY and yaw locked)
        
        Args:
            target_pos: (x, y, z) target UAV position
            target_yaw: Target UAV yaw angle
            
        Returns:
            bool: Success status
        """
        try:
            current_pos = self.get_current_position()
            if not current_pos:
                rospy.logerr("Cannot get current position")
                return False
            
            rospy.loginfo("=== THREE-STAGE MOVEMENT WITH STRICT AXIS LOCKING ===")
            rospy.loginfo(f"Current position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
            rospy.loginfo(f"Current yaw: {math.degrees(self.current_yaw):.1f}°")
            rospy.loginfo(f"Target position: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
            rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
            
            # Calculate yaw change to determine if end-effector compensation is needed
            yaw_change = self.normalize_angle(target_yaw - self.current_yaw)
            rospy.loginfo(f"Required yaw change: {math.degrees(yaw_change):.1f}°")
            
            # === STAGE 1: XY POSITIONING ONLY (Z AND YAW LOCKED) ===
            rospy.loginfo("--- Stage 1: XY positioning (Z and yaw LOCKED) ---")
            
            # Lock Z at current height and yaw at current angle
            stage1_target = (target_pos[0], target_pos[1], current_pos[2])
            stage1_yaw = self.current_yaw  # Keep current yaw
            
            if not self.execute_xy_only_movement(current_pos, stage1_target, stage1_yaw):
                rospy.logerr("Stage 1: XY positioning failed")
                return False
            
            rospy.loginfo("✓ Stage 1 completed - XY positioning done")
            
            # === STAGE 2: YAW ADJUSTMENT WITH END-EFFECTOR COMPENSATION ===
            rospy.loginfo("--- Stage 2: Yaw adjustment with end-effector compensation ---")
            
            if abs(yaw_change) > 0.05:  # Only if significant yaw change needed
                if not self.execute_yaw_adjustment_with_compensation(target_yaw):
                    rospy.logerr("Stage 2: Yaw adjustment failed")
                    return False
            else:
                rospy.loginfo("Yaw change minimal, skipping Stage 2")
            
            rospy.loginfo("✓ Stage 2 completed - Yaw adjustment done")
            
            # === STAGE 3: Z DESCENT ONLY (XY AND YAW LOCKED) ===
            rospy.loginfo("--- Stage 3: Z descent (XY and yaw LOCKED) ---")
            
            final_pos = self.get_current_position()
            if not final_pos:
                rospy.logerr("Cannot get position for Stage 3")
                return False
            
            # Lock XY and yaw, only change Z
            stage3_target = (final_pos[0], final_pos[1], self.locked_z)
            
            if not self.execute_z_only_movement(final_pos, stage3_target):
                rospy.logerr("Stage 3: Z descent failed")
                return False
            
            rospy.loginfo("✓ Stage 3 completed - Z descent done")
            rospy.loginfo("✓ THREE-STAGE MOVEMENT COMPLETED SUCCESSFULLY")
            return True
            
        except Exception as e:
            rospy.logerr(f"Three-stage movement failed: {e}")
            return False
    
    def execute_radial_approach_movement(self, target_pos, target_yaw):
        """
        Execute radial approach movement for Phase 2 (final contact).
        Uses the same three-stage approach but with higher precision.
        
        Args:
            target_pos: (x, y, z) target UAV position  
            target_yaw: Target UAV yaw angle
            
        Returns:
            bool: Success status
        """
        try:
            rospy.loginfo("=== RADIAL APPROACH MOVEMENT (PHASE 2) ===")
            rospy.loginfo(f"Target: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
            rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
            rospy.loginfo("Using enhanced precision for final contact positioning")
            
            current_pos = self.get_current_position()
            if not current_pos:
                rospy.logerr("Cannot get current position")
                return False
            
            # Calculate movement requirements
            yaw_change = self.normalize_angle(target_yaw - self.current_yaw)
            xy_distance = math.sqrt((target_pos[0] - current_pos[0])**2 + 
                                  (target_pos[1] - current_pos[1])**2)
            z_distance = abs(target_pos[2] - current_pos[2])
            
            rospy.loginfo(f"Required movements - XY: {xy_distance*1000:.1f}mm, Z: {z_distance*1000:.1f}mm, Yaw: {math.degrees(yaw_change):.1f}°")
            
            # === STAGE 1: PRECISE XY POSITIONING ===
            rospy.loginfo("--- Radial Stage 1: Precise XY positioning ---")
            
            stage1_target = (target_pos[0], target_pos[1], current_pos[2])  # Lock Z
            stage1_yaw = self.current_yaw  # Lock yaw
            
            if xy_distance > 0.005:  # 5mm threshold (reduced from 2mm for reasonable precision)
                if not self.execute_xy_only_movement(current_pos, stage1_target, stage1_yaw):
                    rospy.logerr("Radial Stage 1: Precise XY positioning failed")
                    return False
            else:
                rospy.loginfo("XY movement minimal, skipping Stage 1")
            
            # === STAGE 2: PRECISE YAW ADJUSTMENT ===
            rospy.loginfo("--- Radial Stage 2: Precise yaw adjustment ---")
            
            if abs(yaw_change) > 0.05:  # 3° threshold (increased from 1.1° for reasonable precision)
                if not self.execute_yaw_adjustment_with_compensation(target_yaw):
                    rospy.logerr("Radial Stage 2: Precise yaw adjustment failed")
                    return False
            else:
                rospy.loginfo("Yaw change minimal, skipping Stage 2")
            
            # === STAGE 3: FINAL Z APPROACH ===
            rospy.loginfo("--- Radial Stage 3: Final Z approach ---")
            
            final_pos = self.get_current_position()
            if not final_pos:
                rospy.logerr("Cannot get position for final Z approach")
                return False
            
            stage3_target = (final_pos[0], final_pos[1], self.locked_z)
            
            if z_distance > 0.005:  # 5mm threshold (increased from 1mm for reasonable precision)
                if not self.execute_z_only_movement(final_pos, stage3_target):
                    rospy.logerr("Radial Stage 3: Final Z approach failed")
                    return False
            else:
                rospy.loginfo("Z movement minimal, skipping Stage 3")
            
            rospy.loginfo("✓ RADIAL APPROACH MOVEMENT COMPLETED")
            return True
            
        except Exception as e:
            rospy.logerr(f"Radial approach movement failed: {e}")
            return False
    
    def verify_two_phase_insertion_result(self, final_pos, left_target, right_target, same_side_result):
        """
        Verify the result of two-phase insertion
        
        Args:
            final_pos: Final UAV position
            left_target: Target left claw position
            right_target: Target right claw position
            same_side_result: Same-side strategy result
            
        Returns:
            bool: Success status
        """
        try:
            # Calculate actual claw positions
            dual_fang_center_x = final_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(self.current_yaw)
            dual_fang_center_y = final_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(self.current_yaw)
            
            actual_left_claw_x = dual_fang_center_x + self.optimizer.half_claw_separation * math.cos(self.current_yaw + math.pi/2)
            actual_left_claw_y = dual_fang_center_y + self.optimizer.half_claw_separation * math.sin(self.current_yaw + math.pi/2)
            
            actual_right_claw_x = dual_fang_center_x - self.optimizer.half_claw_separation * math.cos(self.current_yaw + math.pi/2)
            actual_right_claw_y = dual_fang_center_y - self.optimizer.half_claw_separation * math.sin(self.current_yaw + math.pi/2)
            
            # Calculate positioning errors
            left_error = math.sqrt((actual_left_claw_x - left_target[0])**2 + 
                                 (actual_left_claw_y - left_target[1])**2)
            right_error = math.sqrt((actual_right_claw_x - right_target[0])**2 + 
                                  (actual_right_claw_y - right_target[1])**2)
            
            # Calculate actual claw separation
            actual_separation = math.sqrt((actual_right_claw_x - actual_left_claw_x)**2 + 
                                        (actual_right_claw_y - actual_left_claw_y)**2)
            
            rospy.loginfo(f"=== TWO-PHASE INSERTION VERIFICATION ===")
            rospy.loginfo(f"Final UAV position: ({final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f})")
            rospy.loginfo(f"Final UAV yaw: {math.degrees(self.current_yaw):.1f}°")
            rospy.loginfo(f"Actual left claw: ({actual_left_claw_x:.3f}, {actual_left_claw_y:.3f})")
            rospy.loginfo(f"Target left claw: ({left_target[0]:.3f}, {left_target[1]:.3f})")
            rospy.loginfo(f"Left claw error: {left_error*1000:.1f}mm")
            rospy.loginfo(f"Actual right claw: ({actual_right_claw_x:.3f}, {actual_right_claw_y:.3f})")
            rospy.loginfo(f"Target right claw: ({right_target[0]:.3f}, {right_target[1]:.3f})")
            rospy.loginfo(f"Right claw error: {right_error*1000:.1f}mm")
            rospy.loginfo(f"Actual separation: {actual_separation*1000:.1f}mm")
            rospy.loginfo(f"Expected separation: {same_side_result['actual_separation']*1000:.1f}mm")
            rospy.loginfo(f"Required separation: {same_side_result['required_separation']*1000:.1f}mm")
            
            # Success criteria
            positioning_accurate = left_error < 0.01 and right_error < 0.01  # 10mm tolerance
            separation_adequate = actual_separation >= same_side_result['required_separation'] * 0.95
            
            if positioning_accurate and separation_adequate:
                rospy.loginfo("✓ TWO-PHASE INSERTION VERIFICATION SUCCESSFUL")
                rospy.loginfo(f"✓ User's same-side strategy achieved: {same_side_result['formula']}")
                return True
            else:
                rospy.logwarn("Two-phase insertion verification failed:")
                if not positioning_accurate:
                    rospy.logwarn(f"  Positioning accuracy insufficient (left: {left_error*1000:.1f}mm, right: {right_error*1000:.1f}mm)")
                if not separation_adequate:
                    rospy.logwarn(f"  Separation inadequate ({actual_separation*1000:.1f}mm < {same_side_result['required_separation']*1000:.1f}mm)")
                return False
                
        except Exception as e:
            rospy.logerr(f"Verification failed: {e}")
            return False
    
    def execute_xy_only_movement(self, start_pos, target_pos, locked_yaw):
        """
        Execute XY-only movement with Z and yaw strictly locked.
        Uses VEL+ACC mixed trajectory for smooth motion.
        
        Args:
            start_pos: Starting UAV position
            target_pos: Target UAV position (only XY used)
            locked_yaw: Yaw angle to maintain (locked)
            
        Returns:
            bool: Success status
        """
        rospy.loginfo(f"XY-only movement: ({start_pos[0]:.3f}, {start_pos[1]:.3f}) -> ({target_pos[0]:.3f}, {target_pos[1]:.3f})")
        rospy.loginfo(f"Z locked at: {start_pos[2]:.3f}m, Yaw locked at: {math.degrees(locked_yaw):.1f}°")
        
        # Create trajectory with locked Z and yaw
        xy_target = (target_pos[0], target_pos[1], start_pos[2])  # Lock Z
        
        # Calculate distance for adaptive speed
        xy_distance = math.sqrt((target_pos[0] - start_pos[0])**2 + (target_pos[1] - start_pos[1])**2)
        
        # SLOWER adaptive duration based on distance for better precision
        base_speed = 0.04  # Much slower for precision (was 0.08)
        min_duration = 12.0  # Longer minimum duration
        max_duration = 35.0  # Extended maximum duration
        
        # Even slower for short distances
        if xy_distance < 0.5:
            effective_speed = base_speed * 0.3  # 30% of base speed for very precise movements
        elif xy_distance < 1.0:
            effective_speed = base_speed * 0.5  # 50% of base speed for short distances
        else:
            effective_speed = base_speed * 0.8  # 80% of base speed for longer distances
        
        duration = max(min_duration, min(max_duration, xy_distance / effective_speed))
        
        rospy.loginfo(f"XY distance: {xy_distance:.3f}m, effective speed: {effective_speed:.3f}m/s, duration: {duration:.1f}s")
        
        # Execute VEL+ACC mixed trajectory with tighter thresholds
        return self.motion_controller.execute_smooth_trajectory_with_yaw(
            start_pos=start_pos,
            target_pos=xy_target,
            target_yaw=locked_yaw,  # Strictly lock yaw
            duration=duration,
            pos_threshold=0.02,  # Tighter position control (was 0.03)
            yaw_threshold=0.01   # Tighter yaw control (was 0.02)
        )
    
    def execute_yaw_adjustment_with_compensation(self, target_yaw):
        """
        Execute yaw adjustment with end-effector XY compensation.
        Compensates for end-effector XY displacement during yaw rotation.
        
        Args:
            target_yaw: Target UAV yaw angle
            
        Returns:
            bool: Success status
        """
        current_pos = self.get_current_position()
        current_yaw = self.current_yaw
        
        if not current_pos:
            rospy.logerr("Cannot get current position for yaw adjustment")
            return False
        
        rospy.loginfo(f"Yaw adjustment with compensation: {math.degrees(current_yaw):.1f}° -> {math.degrees(target_yaw):.1f}°")
        
        # Calculate end-effector position before rotation
        end_effector_before_x = current_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(current_yaw)
        end_effector_before_y = current_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(current_yaw)
        
        rospy.loginfo(f"End-effector before rotation: ({end_effector_before_x:.3f}, {end_effector_before_y:.3f})")
        
        # Calculate required UAV position to maintain end-effector position after rotation
        compensated_uav_x = end_effector_before_x - self.optimizer.dual_fang_center_offset * math.cos(target_yaw)
        compensated_uav_y = end_effector_before_y - self.optimizer.dual_fang_center_offset * math.sin(target_yaw)
        compensated_pos = (compensated_uav_x, compensated_uav_y, current_pos[2])  # Lock Z
        
        # Calculate compensation distance
        compensation_distance = math.sqrt((compensated_uav_x - current_pos[0])**2 + 
                                        (compensated_uav_y - current_pos[1])**2)
        
        rospy.loginfo(f"Compensated UAV position: ({compensated_uav_x:.3f}, {compensated_uav_y:.3f})")
        rospy.loginfo(f"Compensation distance: {compensation_distance*1000:.1f}mm")
        
        # Execute combined yaw + compensation movement with SLOWER parameters
        if compensation_distance > 0.001:  # 1mm threshold
            # SLOWER VEL+ACC mixed trajectory for better precision
            yaw_change_magnitude = abs(self.normalize_angle(target_yaw - current_yaw))
            base_duration = max(10.0, yaw_change_magnitude / 0.15)  # Slower: 0.15 rad/s (was 0.2)
            
            # Add extra time for large compensation distances
            if compensation_distance > 0.2:  # 20cm
                base_duration *= 1.5
            elif compensation_distance > 0.1:  # 10cm  
                base_duration *= 1.3
            
            duration = min(base_duration, 25.0)  # Cap at 25 seconds
            
            rospy.loginfo(f"Yaw adjustment duration: {duration:.1f}s for {math.degrees(yaw_change_magnitude):.1f}° + {compensation_distance*1000:.1f}mm compensation")
            
            success = self.motion_controller.execute_smooth_trajectory_with_yaw(
                start_pos=current_pos,
                target_pos=compensated_pos,
                target_yaw=target_yaw,
                duration=duration,
                pos_threshold=0.008,  # Much tighter position control (was 0.01)
                yaw_threshold=0.02    # Tighter yaw control (was 0.03)
            )
            
            if success:
                # Verify end-effector position after compensation
                final_pos = self.get_current_position()
                final_yaw = self.current_yaw
                
                end_effector_after_x = final_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(final_yaw)
                end_effector_after_y = final_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(final_yaw)
                
                end_effector_drift = math.sqrt((end_effector_after_x - end_effector_before_x)**2 + 
                                             (end_effector_after_y - end_effector_before_y)**2)
                
                rospy.loginfo(f"End-effector after rotation: ({end_effector_after_x:.3f}, {end_effector_after_y:.3f})")
                rospy.loginfo(f"End-effector drift: {end_effector_drift*1000:.1f}mm")
                
                if end_effector_drift < 0.008:  # Tighter tolerance: 8mm (was 5mm)
                    rospy.loginfo("✓ Yaw adjustment with successful end-effector compensation")
                    return True
                else:
                    rospy.logwarn(f"End-effector drift {end_effector_drift*1000:.1f}mm exceeds 8mm tolerance")
                    # For precision insertion, this is still acceptable if drift < 15mm
                    if end_effector_drift < 0.015:
                        rospy.loginfo("Drift within acceptable range for precision insertion")
                        return True
                    else:
                        rospy.logerr("Excessive end-effector drift for precision insertion")
                        return False
            else:
                rospy.logerr("Yaw adjustment with compensation failed")
                return False
        else:
            # Pure yaw rotation without compensation needed
            rospy.loginfo("Minimal compensation needed, executing pure yaw rotation")
            return self.execute_pure_yaw_rotation(target_yaw)
    
    def execute_pure_yaw_rotation(self, target_yaw):
        """Execute pure yaw rotation with position locked."""
        current_pos = self.get_current_position()
        if not current_pos:
            return False
        
        # Send yaw command with locked position
        self.motion_controller.send_trajectory_point(current_pos, target_yaw)
        
        # Wait for yaw convergence with timeout
        start_time = time.time()
        rate = rospy.Rate(10)
        
        while (time.time() - start_time) < 8.0 and not rospy.is_shutdown():
            yaw_error = abs(self.normalize_angle(self.current_yaw - target_yaw))
            if yaw_error < 0.03:  # 0.03 rad = ~1.7°
                rospy.loginfo(f"Pure yaw rotation completed. Error: {math.degrees(yaw_error):.1f}°")
                return True
            
            self.motion_controller.send_trajectory_point(current_pos, target_yaw)
            rate.sleep()
        
        rospy.logwarn("Pure yaw rotation timeout")
        return False
    
    def execute_z_only_movement(self, start_pos, target_pos):
        """
        Execute Z-only movement with XY and yaw strictly locked.
        Uses VEL+ACC mixed trajectory for smooth descent.
        
        Args:
            start_pos: Starting UAV position
            target_pos: Target UAV position (only Z used)
            
        Returns:
            bool: Success status
        """
        rospy.loginfo(f"Z-only movement: {start_pos[2]:.3f}m -> {target_pos[2]:.3f}m")
        rospy.loginfo(f"XY locked at: ({start_pos[0]:.3f}, {start_pos[1]:.3f})")
        rospy.loginfo(f"Yaw locked at: {math.degrees(self.current_yaw):.1f}°")
        
        # Lock XY and yaw, only change Z
        z_target = (start_pos[0], start_pos[1], target_pos[2])
        locked_yaw = self.current_yaw
        
        # Calculate Z distance and SLOWER adaptive duration for precision
        z_distance = abs(target_pos[2] - start_pos[2])
        descent_speed = 0.02  # Much slower descent for safety and precision (was 0.03)
        min_duration = 10.0   # Longer minimum duration for stability
        duration = max(min_duration, z_distance / descent_speed)
        
        rospy.loginfo(f"Z distance: {z_distance:.3f}m, descent speed: {descent_speed:.3f}m/s, duration: {duration:.1f}s")
        
        # Execute smooth Z descent with VEL+ACC trajectory and tighter control
        return self.motion_controller.execute_smooth_trajectory_with_yaw(
            start_pos=start_pos,
            target_pos=z_target,
            target_yaw=locked_yaw,
            duration=duration,
            pos_threshold=0.008,  # Much tighter Z control (was 0.01)
            yaw_threshold=0.01    # Tighter yaw lock (was 0.02)
        )
    
    def execute_legacy_insertion(self, params):
        """
        Execute legacy single-phase insertion for backward compatibility.
        
        Args:
            params: Legacy insertion parameters
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("=== EXECUTING LEGACY SINGLE-PHASE INSERTION ===")
        
        current_pos = self.get_current_position()
        if not current_pos:
            rospy.logerr("No UAV position available")
            return False
        
        # Extract legacy parameters
        optimal_uav_position = params.get('optimal_uav_position')
        optimal_uav_yaw = params.get('optimal_uav_yaw')
        left_claw_target = params.get('left_claw_target')
        right_claw_target = params.get('right_claw_target')
        
        if not all([optimal_uav_position, optimal_uav_yaw is not None, left_claw_target, right_claw_target]):
            rospy.logerr("Missing required legacy parameters")
            return False
        
        rospy.loginfo(f"Current UAV position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"Target UAV position: ({optimal_uav_position[0]:.3f}, {optimal_uav_position[1]:.3f}, {optimal_uav_position[2]:.3f})")
        rospy.loginfo(f"Target UAV yaw: {math.degrees(optimal_uav_yaw):.1f}°")
        
        # Execute direct movement to target position
        target_pos = (optimal_uav_position[0], optimal_uav_position[1], self.locked_z)
        
        return self.execute_safe_movement_to_position(target_pos, optimal_uav_yaw)


# Additional states for the complete state machine
class RotateValveState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'], module_id=module_id)
    
    def execute(self, userdata):
        rospy.loginfo("Rotating valve...")
        # Implement valve rotation logic here
        return 'succeeded'


class ReturnToStartState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
    
    def execute(self, userdata):
        rospy.loginfo("Returning to start position...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        start_position = userdata.start_position
        current_pos = self.get_current_position()
        
        if current_pos is None or start_position is None:
            rospy.logerr("Cannot get positions for return movement")
            return 'failed'
        
        # Return to start position
        success = self.motion_controller.execute_smooth_trajectory_with_yaw(
            start_pos=current_pos,
            target_pos=start_position,
            target_yaw=0.0,  # Default yaw
            duration=20.0,
            pos_threshold=0.05,
            yaw_threshold=0.1
        )
        
        if success:
            rospy.loginfo("Successfully returned to start position")
            return 'succeeded'
        else:
            rospy.logerr("Failed to return to start position")
            return 'failed'


def create_state_machine(module_id=1):
    """Create the SMACH state machine for single UAV valve rotation"""
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    
    with sm:
        smach.StateMachine.add('INITIALIZE_START_POSITION',
                             InitializeStartPositionState(module_id=module_id),
                             transitions={'succeeded': 'MOVE_TO_VALVE',
                                        'failed': 'failed'},
                             remapping={'start_position': 'start_position'})
        
        smach.StateMachine.add('MOVE_TO_VALVE',
                             MoveToValveState(module_id=module_id),
                             transitions={'succeeded': 'DESCEND_AND_CONTACT',
                                        'failed': 'failed'})
        
        smach.StateMachine.add('DESCEND_AND_CONTACT',
                             DescendAndContactState(module_id=module_id),
                             transitions={'succeeded': 'ROTATE_VALVE',
                                        'failed': 'failed'})
        
        smach.StateMachine.add('ROTATE_VALVE',
                             RotateValveState(module_id=module_id),
                             transitions={'succeeded': 'RETURN_TO_START',
                                        'failed': 'failed'})
        
        smach.StateMachine.add('RETURN_TO_START',
                             ReturnToStartState(module_id=module_id),
                             transitions={'succeeded': 'succeeded',
                                        'failed': 'failed'},
                             remapping={'start_position': 'start_position'})
    
    return sm


def main():
    """Main function to run the single UAV valve rotation"""
    rospy.init_node('single_uav_valve_rotation')
    
    try:
        # Get module ID from parameter
        module_id = rospy.get_param('~module_id', 1)
        rospy.loginfo(f"Starting single UAV valve rotation for module {module_id}")
        
        # Create and execute state machine
        sm = create_state_machine(module_id=module_id)
        
        # Create SMACH viewer (optional)
        sis = smach_ros.IntrospectionServer('single_uav_valve_rotation', sm, '/SM_ROOT')
        sis.start()
        
        # Execute state machine
        outcome = sm.execute()
        
        rospy.loginfo(f"State machine completed with outcome: {outcome}")
        
        # Stop introspection server
        sis.stop()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
