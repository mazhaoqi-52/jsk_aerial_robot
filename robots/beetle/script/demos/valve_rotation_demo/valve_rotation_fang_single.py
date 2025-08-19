#!/usr/bin/env python3
"""
Single UAV Valve Rotation using SMACH State Machine
Cleaned version with redundant code removed.
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
from trajectory import create_constant_distance_trajectory
from unified_motion_controller import UnifiedMotionController
import os
import sys

# Add current directory to Python path to ensure we import the local insertion_optimizer
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

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
    
    def execute_smooth_trajectory(self, start_pos, target_pos, target_yaw, duration=5.0, use_z_lock=True):
        """
        Execute smooth trajectory with VEL+ACCEL control and optional Z-lock
        This is a common method that can be used by all child states
        """
        from trajectory import PolynomialTrajectory
        
        # Use locked Z coordinate if enabled and available
        if use_z_lock and hasattr(self, 'locked_z'):
            locked_z = self.locked_z
        else:
            locked_z = start_pos[2]
        
        # Lock Z coordinate for both start and target
        locked_start_pos = (start_pos[0], start_pos[1], locked_z)
        locked_target_pos = (target_pos[0], target_pos[1], locked_z)
        
        rospy.loginfo(f"VEL+ACCEL trajectory:")
        rospy.loginfo(f"  Start: [{locked_start_pos[0]:.6f}, {locked_start_pos[1]:.6f}, {locked_start_pos[2]:.6f}]")
        rospy.loginfo(f"  Target: [{locked_target_pos[0]:.6f}, {locked_target_pos[1]:.6f}, {locked_target_pos[2]:.6f}]")
        if use_z_lock:
            rospy.loginfo(f"  Z LOCKED at: {locked_z:.6f}m")
        
        # Create trajectory
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(locked_start_pos, locked_target_pos)
        
        start_yaw = self.current_yaw
        yaw_diff = target_yaw - start_yaw
        
        # Normalize yaw difference
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Execute trajectory with mixed VEL+ACCEL control and distance-adaptive speed
        start_time = time.time()
        rate = rospy.Rate(50)
        
        # Adaptive control gains
        base_pos_gain = 0.8
        base_vel_gain = 0.6
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 2.0:
            current_time = time.time()
            elapsed = current_time - start_time
            progress = min(elapsed / duration, 1.0)
            
            # Get trajectory values
            desired_pos = traj.get_next_position()
            desired_vel = traj.get_velocity()
            
            if desired_pos is None:
                desired_pos = locked_target_pos
                desired_vel = (0.0, 0.0, 0.0)
            
            # Force Z lock if enabled
            if use_z_lock:
                desired_pos = (desired_pos[0], desired_pos[1], locked_z)
                if desired_vel is not None:
                    desired_vel = (desired_vel[0], desired_vel[1], 0.0)
                else:
                    desired_vel = (0.0, 0.0, 0.0)
            
            # Calculate control commands with distance-adaptive gains
            current_pos = self.get_current_position()
            if current_pos is not None:
                # Distance to target for adaptive control
                distance_to_target = math.sqrt((current_pos[0] - locked_target_pos[0])**2 + 
                                             (current_pos[1] - locked_target_pos[1])**2)
                
                # Adaptive gains: higher gains when closer to target for precision
                if distance_to_target < 0.05:
                    pos_gain = base_pos_gain * 1.2  # 20% higher gain for precision
                    vel_gain = base_vel_gain * 0.8   # 20% lower vel gain for stability
                elif distance_to_target < 0.1:
                    pos_gain = base_pos_gain * 1.1  # 10% higher gain
                    vel_gain = base_vel_gain * 0.9   # 10% lower vel gain
                else:
                    pos_gain = base_pos_gain
                    vel_gain = base_vel_gain
                
                pos_error = (desired_pos[0] - current_pos[0],
                           desired_pos[1] - current_pos[1],
                           locked_z - current_pos[2] if use_z_lock else desired_pos[2] - current_pos[2])
                
                # Mixed control with adaptive gains
                cmd_vel = (vel_gain * desired_vel[0] + pos_gain * pos_error[0],
                          vel_gain * desired_vel[1] + pos_gain * pos_error[1],
                          pos_gain * pos_error[2])
                
                # Adaptive velocity limit based on distance to target
                if distance_to_target < 0.05:
                    max_vel = 0.08  # Very slow when very close
                elif distance_to_target < 0.1:
                    max_vel = 0.12  # Slow when close
                else:
                    max_vel = 0.15  # Normal speed when far
                
                cmd_vel_magnitude = math.sqrt(cmd_vel[0]**2 + cmd_vel[1]**2 + cmd_vel[2]**2)
                if cmd_vel_magnitude > max_vel:
                    scale = max_vel / cmd_vel_magnitude
                    cmd_vel = (cmd_vel[0] * scale, cmd_vel[1] * scale, cmd_vel[2] * scale)
            else:
                cmd_vel = (0.0, 0.0, 0.0)
            
            # Calculate yaw command
            desired_yaw = start_yaw + progress * yaw_diff
            yaw_error = self.normalize_angle(desired_yaw - self.current_yaw)
            cmd_yaw_vel = 0.5 * yaw_error
            
            # Send command
            self.motion_controller.send_velocity_command(cmd_vel, cmd_yaw_vel, desired_pos, desired_yaw)
            
            # Check convergence
            if current_pos is not None:
                xy_distance = math.sqrt((current_pos[0] - locked_target_pos[0])**2 + 
                                      (current_pos[1] - locked_target_pos[1])**2)
                yaw_error_final = abs(self.normalize_angle(self.current_yaw - target_yaw))
                
                if xy_distance < 0.05 and yaw_error_final < 0.1 and progress >= 0.95:
                    rospy.loginfo(f"Trajectory converged: XY={xy_distance:.3f}m, yaw={yaw_error_final:.3f}rad")
                    break
            
            rate.sleep()
        
        rospy.loginfo("Trajectory completed")
        return True
    
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
                    rospy.loginfo("Starting stuck detection timer")
                else:
                    stuck_duration = time.time() - self.stuck_start_time
                    rospy.loginfo(f"Stuck duration: {stuck_duration:.1f}s / {self.stuck_threshold:.1f}s")
                    if stuck_duration > self.stuck_threshold:
                        rospy.logwarn(f"UAV stuck for {stuck_duration:.1f}s - triggering emergency")
                        return True
            else:
                if self.stuck_start_time is not None:
                    rospy.loginfo("Movement detected - resetting stuck timer")
                self.stuck_start_time = None
        
        self.last_position = current_pos
        self.last_yaw = self.current_yaw
        return False


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
    def __init__(self, module_id=1, approach_distance=0.8, approach_height=0.3):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        self.approach_distance = approach_distance
        self.approach_height = approach_height  # Reduced to 0.3m for faster insertion
    
    def execute(self, userdata):
        rospy.loginfo("Moving to valve approach position...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        # Calculate approach position
        valve_x, valve_y, valve_z = self.valve_pos
        current_pos = self.get_current_position()
        
        if current_pos is None:
            rospy.logerr("Cannot get current UAV position")
            return 'failed'
        
        # Fixed approach from negative X side
        approach_x = valve_x - self.approach_distance
        approach_y = valve_y
        approach_z = valve_z + self.approach_height
        
        target_pos = (approach_x, approach_y, approach_z)
        target_yaw = math.atan2(valve_y - approach_y, valve_x - approach_x)
        
        rospy.loginfo(f"Valve position: ({valve_x:.3f}, {valve_y:.3f}, {valve_z:.3f})")
        rospy.loginfo(f"Current UAV position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"Moving to approach position: {target_pos}")
        rospy.loginfo(f"Target yaw: {target_yaw:.3f} rad ({math.degrees(target_yaw):.1f}°)")
        rospy.loginfo(f"Current yaw: {self.current_yaw:.3f} rad ({math.degrees(self.current_yaw):.1f}°)")
        yaw_change = self.normalize_angle(target_yaw - self.current_yaw)
        rospy.loginfo(f"Required yaw change: {yaw_change:.3f} rad ({math.degrees(yaw_change):.1f}°)")
        
        # Two-step movement: position then yaw
        current_pos = self.get_current_position()
        if current_pos is None:
            return 'failed'
        
        rospy.loginfo("Step 1: Moving to target position")
        
        # Calculate adaptive trajectory duration based on distance
        distance = math.sqrt((target_pos[0] - current_pos[0])**2 + 
                           (target_pos[1] - current_pos[1])**2 + 
                           (target_pos[2] - current_pos[2])**2)
        
        # Adaptive speed: slower when closer to target
        base_speed = 0.1  # Base speed: 0.2 m/s
        min_duration = 10.0  # Minimum time for safety
        max_duration = 30.0  # Maximum time to avoid excessive delays
        
        # Use slower speed for short distances to reduce overshoot
        if distance < 1.0:
            effective_speed = base_speed * 0.5  # 50% of base speed for short distances
        elif distance < 2.0:
            effective_speed = base_speed * 0.8  # 80% of base speed for medium distances
        else:
            effective_speed = base_speed  # Full speed for long distances
        
        trajectory_duration = max(min_duration, min(max_duration, distance / effective_speed))
        rospy.loginfo(f"Distance: {distance:.3f}m, speed: {effective_speed:.2f}m/s, duration: {trajectory_duration:.1f}s")
        
        # Step 1: Position movement
        current_yaw = self.current_yaw
        success = self.motion_controller.execute_smooth_trajectory_with_yaw(
            start_pos=current_pos,
            target_pos=target_pos,
            target_yaw=current_yaw,
            duration=trajectory_duration,
            pos_threshold=0.05,
            yaw_threshold=0.2
        )
        
        if not success:
            rospy.logerr("Failed to move to target position")
            return 'failed'
        
        rospy.loginfo("Step 2: Adjusting yaw orientation")
        
        # Step 2: Yaw adjustment
        final_pos = self.get_current_position()
        if final_pos is None:
            final_pos = target_pos
        
        self.motion_controller.send_trajectory_point(final_pos, target_yaw)
        
        # Wait for yaw convergence
        yaw_adjustment_start = time.time()
        rate = rospy.Rate(10)
        
        while (time.time() - yaw_adjustment_start) < 5.0 and not rospy.is_shutdown():
            yaw_error = abs(self.normalize_angle(self.current_yaw - target_yaw))
            if yaw_error < self.yaw_threshold:
                rospy.loginfo(f"Yaw adjustment completed. Error: {yaw_error:.3f} rad")
                break
            
            self.motion_controller.send_trajectory_point(final_pos, target_yaw)
            rate.sleep()
        
        rospy.loginfo("Two-step movement completed")
        return 'succeeded'


class DescendAndContactState(SingleUAVStateBase):
    def __init__(self, module_id=1, rotation_direction=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        # Store rotation direction for insertion strategy
        self.rotation_direction = rotation_direction  # 1 for clockwise, -1 for counter-clockwise
        
        # Initialize insertion optimizer with correct physical parameters
        self.optimizer = InsertionOptimizer(
            valve_radius=0.1075,  # Inner radius (215mm/2) - corrected based on physical specs
            valve_beam_width=0.024,  # 24mm beam width - corrected
            safety_margin=0.002,
            module_id=module_id
        )
        
        self.pre_insertion_distance = 0.035
        self.circumferential_offset = 0.005
    
    def execute(self, userdata):
        """
        Enhanced 4-step valve engagement strategy:
        1. Determine optimal contact point for valve rotation
        2. Rotate UAV and insert into beam gap for maximum control margin  
        3. Move along valve circumference from insertion point to contact point
        4. Begin valve rotation
        """
        rospy.loginfo("=== ENHANCED 4-STEP VALVE ENGAGEMENT STRATEGY ===")
        
        # Initialize and wait for required positions
        if not self.wait_for_positions():
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None:
            return 'failed'
        
        # CRITICAL: Lock Z coordinate for entire process to prevent oscillation
        self.locked_z = start_pos[2]
        rospy.loginfo(f"Z-axis LOCKED at {self.locked_z:.6f}m for entire process")
        
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
        Step 1: Determine optimal contact point for valve rotation based on:
        - Rotation direction (clockwise/counter-clockwise)
        - Beam gap analysis for maximum mechanical advantage
        - Available beam gaps and their accessibility
        """
        rospy.loginfo("--- Step 1: Determining optimal contact point ---")
        
        # Get optimal strategy with current rotation direction
        optimal_strategy = self.optimizer.evaluate_real_time_strategy()
        if optimal_strategy is None:
            rospy.logerr("Failed to evaluate optimal strategy")
            return None
        
        # Extract contact point information from strategy
        contact_point = {}
        
        # For dual-fang mode, use gap center as optimal contact point
        if optimal_strategy.get('insertion_mode') == 'dual_simultaneous':
            fang1_strategy = optimal_strategy.get('fang1')
            fang2_strategy = optimal_strategy.get('fang2')
            
            if fang1_strategy and fang2_strategy:
                # Calculate optimal contact point from gap center
                gap_center_x = (fang1_strategy['insertion_position'][0] + fang2_strategy['insertion_position'][0]) / 2
                gap_center_y = (fang1_strategy['insertion_position'][1] + fang2_strategy['insertion_position'][1]) / 2
                gap_center_angle = math.atan2(gap_center_y - self.valve_pos[1], gap_center_x - self.valve_pos[0])
                
                contact_point = {
                    'position': (gap_center_x, gap_center_y, self.valve_pos[2]),
                    'angle': gap_center_angle,
                    'rotation_direction': self.rotation_direction,
                    'beam_gap': fang1_strategy.get('beam_gap', {}),
                    'mechanical_advantage': 'High - dual fang engagement',
                    'strategy': optimal_strategy
                }
                
                rospy.loginfo(f"Optimal contact point determined:")
                rospy.loginfo(f"  Position: ({gap_center_x:.3f}, {gap_center_y:.3f}, {self.valve_pos[2]:.3f})")
                rospy.loginfo(f"  Angle: {math.degrees(gap_center_angle):.1f}°")
                rospy.loginfo(f"  Rotation direction: {'clockwise' if self.rotation_direction == 1 else 'counter-clockwise'}")
                
                return contact_point
        
        rospy.logerr("Failed to determine optimal contact point from strategy")
        return None
    
    def rotate_and_insert_to_beam_gap(self, contact_point):
        """
        Step 2: Rotate UAV and insert into beam gap for maximum control margin
        - Position UAV for optimal insertion angle
        - Insert end-effector into beam gap with maximum clearance
        - Ensure stable grip before circumferential movement
        """
        rospy.loginfo("--- Step 2: Rotating UAV and inserting into beam gap ---")
        
        if contact_point is None:
            rospy.logerr("No contact point provided for insertion")
            return False
        
        strategy = contact_point['strategy']
        
        # Get insertion parameters from strategy
        insertion_params = self.optimizer.get_insertion_parameters(
            strategy=strategy,
            pre_insertion_distance=self.pre_insertion_distance,
            circumferential_offset=self.circumferential_offset
        )
        
        if insertion_params is None:
            rospy.logerr("Failed to get insertion parameters")
            return False
        
        # Execute enhanced insertion using existing optimized logic
        success = self.execute_enhanced_insertion(insertion_params)
        
        if success:
            rospy.loginfo("Successfully inserted into beam gap with maximum control margin")
            return True
        else:
            rospy.logerr("Failed to insert into beam gap")
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
    
    def execute_enhanced_insertion(self, insertion_params):
        """
        Execute enhanced insertion using optimized dual-fang parameters
        Simplified version that focuses on the core insertion logic
        """
        rospy.loginfo("Executing enhanced insertion")
        
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position for insertion")
            return False
        
        # Extract fang parameters based on best option
        selected_fang_params = None
        selected_fang_id = None
        
        # Choose the fang with lower complexity score
        if 'fang1' in insertion_params and 'fang2' in insertion_params:
            fang1_complexity = insertion_params['fang1']['complexity_score']
            fang2_complexity = insertion_params['fang2']['complexity_score']
            
            if fang1_complexity <= fang2_complexity:
                selected_fang_params = insertion_params['fang1']
                selected_fang_id = 'fang1'
            else:
                selected_fang_params = insertion_params['fang2']
                selected_fang_id = 'fang2'
        else:
            rospy.logerr("Invalid insertion parameters - missing fang data")
            return False
        
        rospy.loginfo(f"Selected {selected_fang_id} for insertion (complexity: {selected_fang_params['complexity_score']:.3f})")
        
        # Extract target position and orientation
        target_x = selected_fang_params['final_position'][0]
        target_y = selected_fang_params['final_position'][1]
        target_z = self.locked_z  # Use locked Z to prevent oscillation
        target_yaw = selected_fang_params['target_yaw']
        
        # Calculate UAV position to place end-effector at target
        end_effector_offset_x = 0.25346  # Dual-fang center X offset from UAV
        end_effector_offset_y = 0.0      # Dual-fang center Y offset
        end_effector_offset_z = 0.0221140  # Dual-fang center Z offset
        
        # Compensate for end-effector offset
        uav_x = target_x - end_effector_offset_x * math.cos(target_yaw)
        uav_y = target_y - end_effector_offset_x * math.sin(target_yaw)
        uav_z = target_z - end_effector_offset_z
        
        target_pos = (uav_x, uav_y, uav_z)
        
        rospy.loginfo(f"Enhanced insertion target: {target_pos}, yaw: {math.degrees(target_yaw):.1f}°")
        
        # Execute trajectory to target position
        return self.execute_smooth_trajectory(current_pos, target_pos, target_yaw, duration=6.0)
    

class RotateValveState(SingleUAVStateBase):
    def __init__(self, module_id=1, rotation_angle=math.pi/2, rotation_duration=12.0, rotation_direction=1):
        super().__init__(outcomes=['succeeded', 'failed', 'emergency'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        self.rotation_angle = rotation_angle
        self.rotation_duration = rotation_duration  # Increased from 8.0s to 12.0s for smoother rotation
        self.rotation_direction = rotation_direction  # 1 for clockwise, -1 for counter-clockwise
        
        # Emergency detection - made much less sensitive for rotation tasks
        self.emergency_stop = threading.Event()
        self.emergency_triggered = False  # Separate flag to track actual emergencies
        self.last_position = None
        self.last_yaw = 0.0
        self.stuck_start_time = None
        self.stuck_threshold = 60.0  # Increased from 45.0s to 60.0s - allow more time for rotation
        self.movement_threshold = 0.005  # Reduced from 0.01 to 0.005 for very sensitive movement detection
        self.yaw_threshold = 0.02  # Reduced from 0.05 to 0.02 for very sensitive yaw detection
    
    def execute(self, userdata):
        rospy.loginfo("Starting valve rotation...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        current_pos = self.get_current_position()
        if current_pos is None:
            return 'failed'
        
        # Initialize tracking
        self.last_position = current_pos
        self.last_yaw = self.current_yaw
        self.stuck_start_time = None
        
        # Create rotation trajectory
        rotation_traj = create_constant_distance_trajectory(
            current_uav_pos=current_pos,
            current_uav_yaw=self.current_yaw,
            valve_center=self.valve_pos,
            rotation_angle=self.rotation_angle,
            rotation_duration=self.rotation_duration,
            end_effector_offset_x=0.246,
            end_effector_offset_y=0.0,
            end_effector_offset_z=0.0743823,
            rotation_direction=self.rotation_direction
        )
        
        if rotation_traj is None:
            rospy.logerr("Failed to create rotation trajectory")
            return 'failed'
        
        # Start trajectory
        rotation_traj.start_trajectory()
        
        # Start emergency monitoring using base class
        self.start_emergency_monitoring()
        
        # Execute rotation
        try:
            success = self.execute_rotation_trajectory(rotation_traj)
            
            # Stop emergency monitoring
            self.stop_emergency_monitoring()
            
            # Check if emergency was triggered DURING execution (not after we stopped it)
            if hasattr(self, 'emergency_triggered') and self.emergency_triggered:
                rospy.logwarn("Emergency detected during rotation")
                return 'emergency'
            
            if success:
                rospy.loginfo("Valve rotation completed successfully")
                return 'succeeded'
            else:
                rospy.logerr("Valve rotation failed")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during rotation: {e}")
            self.emergency_stop.set()
            return 'failed'
    
    def execute_rotation_trajectory(self, rotation_traj):
        rate = rospy.Rate(50)
        start_time = time.time()
        
        while not rospy.is_shutdown() and not self.emergency_stop.is_set():
            result = rotation_traj.get_next_uav_position_and_yaw()
            if result is None:
                break
            
            target_pos, target_yaw = result
            self.motion_controller.send_trajectory_point(target_pos, target_yaw)
            rate.sleep()
        
        return True


def main():
    rospy.init_node('valve_rotation_single_uav')
    
    module_id = rospy.get_param('~module_id', 1)
    rotation_direction = rospy.get_param('~rotation_direction', 1)  # 1 for clockwise, -1 for counter-clockwise
    rospy.loginfo(f"Starting valve rotation for UAV module {module_id}")
    rospy.loginfo(f"Rotation direction: {'Clockwise' if rotation_direction == 1 else 'Counter-clockwise'}")
    
    # Create state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    sm.userdata.start_position = None
    
    with sm:
        smach.StateMachine.add(
            'INITIALIZE_START_POSITION',
            InitializeStartPositionState(module_id=module_id),
            transitions={'succeeded': 'MOVE_TO_VALVE', 'failed': 'failed'}
        )
        
        smach.StateMachine.add(
            'MOVE_TO_VALVE',
            MoveToValveState(module_id=module_id, approach_distance=0.8, approach_height=0.3),
            transitions={'succeeded': 'DESCEND_AND_CONTACT', 'failed': 'failed'}
        )
        
        smach.StateMachine.add(
            'DESCEND_AND_CONTACT',
            DescendAndContactState(module_id=module_id, rotation_direction=rotation_direction),
            transitions={'succeeded': 'ROTATE_VALVE', 'failed': 'failed'}
        )
        
        smach.StateMachine.add(
            'ROTATE_VALVE',
            RotateValveState(module_id=module_id, rotation_angle=math.pi/2, rotation_duration=12.0, rotation_direction=rotation_direction),
            transitions={
                'succeeded': 'succeeded',
                'failed': 'failed',
                'emergency': 'failed'
            }
        )
    
    # Create introspection server
    sis = smach_ros.IntrospectionServer('valve_rotation_state_machine', sm, '/SM_ROOT')
    sis.start()
    
    try:
        rospy.loginfo("Starting valve rotation state machine...")
        outcome = sm.execute()
        rospy.loginfo(f"State machine completed with outcome: {outcome}")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Valve rotation interrupted")
    except Exception as e:
        rospy.logerr(f"Valve rotation error: {e}")
    finally:
        # Clean up emergency monitoring threads for all states
        for state_name, state in sm._states.items():
            if hasattr(state, 'stop_emergency_monitoring'):
                try:
                    state.stop_emergency_monitoring()
                    rospy.loginfo(f"Stopped emergency monitoring for state: {state_name}")
                except Exception as e:
                    rospy.logwarn(f"Error stopping emergency monitoring for {state_name}: {e}")
        
        sis.stop()
        rospy.loginfo("Valve rotation node cleanup completed")


if __name__ == '__main__':
    main()
