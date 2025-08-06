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
        
        # Thread events for data synchronization
        self.uav_received = threading.Event()
        self.valve_received = threading.Event()
        
        # Check simulation mode
        self.is_simulation = rospy.get_param("~simulation", True)
        
        # Setup subscribers
        self.uav_sub = rospy.Subscriber(f"/beetle{module_id}/mocap/pose", PoseStamped, self.uav_callback, queue_size=1)
        self.wrench_sub = rospy.Subscriber(f"/beetle{module_id}/estimated_external_wrench", WrenchStamped, self.wrench_callback, queue_size=1)
        
        # Subscribe to valve position based on simulation mode
        if self.is_simulation:
            self.valve_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            self.valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
        
        # Basic parameters
        self.position_threshold = 0.03
        self.yaw_threshold = 0.05
        self.timeout = 10.0
        self.z_offset = 0.0743823
        self.descent_speed = 0.025
        
        rospy.loginfo(f"Initialized UAV{module_id} state with basic parameters")
    
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
        """Callback for valve position updates in simulation mode (Odometry)"""
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


class InitializeStartPositionState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
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
    def __init__(self, module_id=1, approach_distance=0.5, approach_height=1.5):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        self.approach_distance = approach_distance
        self.approach_height = approach_height
    
    def execute(self, userdata):
        rospy.loginfo("Moving to valve approach position...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        # Calculate approach position
        valve_x, valve_y, valve_z = self.valve_pos
        approach_x = valve_x + self.approach_distance
        approach_y = valve_y
        approach_z = valve_z + self.approach_height
        
        target_pos = (approach_x, approach_y, approach_z)
        target_yaw = math.atan2(valve_y - approach_y, valve_x - approach_x)
        
        rospy.loginfo(f"Moving to approach position: {target_pos}")
        
        # Execute movement
        return self.motion_controller.move_to_position(
            target_pos, target_yaw, 
            timeout=self.timeout, 
            pos_threshold=self.position_threshold,
            yaw_threshold=self.yaw_threshold
        )


class DescendAndContactState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        # Initialize insertion optimizer - reduce radius for closer approach (20mm total)
        self.optimizer = InsertionOptimizer(
            valve_radius=0.1225 - 0.020,  # Reduced by 20mm for closer approach to valve center
            valve_beam_width=0.0185,
            safety_margin=0.002,  # Further reduced safety margin for even closer approach
            module_id=module_id
        )
        
        self.pre_insertion_distance = 0.035
        self.circumferential_offset = 0.005
    
    def execute(self, userdata):
        rospy.loginfo("Starting 4-stage insertion process...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None:
            return 'failed'
        
        # CRITICAL: Lock Z coordinate for entire insertion process to prevent oscillation
        self.locked_z = start_pos[2]
        rospy.loginfo(f"Z-axis LOCKED at {self.locked_z:.6f}m for entire insertion process")
        
        # Stage 1: Approach insertion point
        if not self.approach_insertion_point(start_pos):
            rospy.logerr("Failed to approach insertion point")
            return 'failed'
        
        # Stage 2: Yaw and circumferential adjustment
        if not self.adjust_yaw_and_circumferential_direction():
            rospy.logerr("Failed to adjust insertion direction")
            return 'failed'
        
        # Stage 3: Descent to contact
        if not self.descend_to_contact():
            rospy.logerr("Failed to descend to contact")
            return 'failed'
        
        # Stage 4: Final rotation to target position
        if not self.rotate_to_target_position():
            rospy.logerr("Failed to rotate to target position")
            return 'failed'
        
        rospy.loginfo("4-stage insertion completed successfully")
        return 'succeeded'
    
    def approach_insertion_point(self, start_pos):
        optimal_strategy = self.optimizer.evaluate_real_time_strategy()
        if optimal_strategy is None:
            rospy.logerr("Failed to determine optimal insertion strategy")
            return False
        
        insertion_params = self.optimizer.get_insertion_parameters(
            strategy=optimal_strategy,
            pre_insertion_distance=self.pre_insertion_distance,
            circumferential_offset=self.circumferential_offset
        )
        
        if insertion_params is None:
            rospy.logerr("Failed to get insertion parameters")
            return False
        
        # Extract single UAV insertion parameters from dual-fang strategy
        # For single UAV, choose the fang with better approach characteristics
        if 'fang1' in insertion_params and 'fang2' in insertion_params:
            # Choose fang with lower complexity score for single UAV
            fang1_complexity = insertion_params['fang1']['complexity_score']
            fang2_complexity = insertion_params['fang2']['complexity_score']
            
            selected_fang = 'fang1' if fang1_complexity <= fang2_complexity else 'fang2'
            selected_params = insertion_params[selected_fang]
            
            rospy.loginfo(f"Single UAV: Selected {selected_fang} (complexity: {selected_params['complexity_score']:.3f})")
            rospy.loginfo(f"Single UAV: {selected_params['fang_config']['name']}")
            rospy.loginfo(f"Single UAV: Beam gap {selected_params['beam_gap']}")
            
            # Convert to single UAV format
            single_uav_params = {
                'approach_position': selected_params['approach_position'],
                'final_position': selected_params['final_position'],
                'target_yaw': selected_params['target_yaw'],
                'selected_fang': selected_fang,
                'fang_config': selected_params['fang_config'],
                'beam_gap': selected_params['beam_gap']
            }
            self.current_insertion_params = single_uav_params
        else:
            rospy.logerr("Invalid dual-fang insertion parameters")
            return False
        
        # Use locked Z coordinate to prevent oscillation
        approach_x = self.current_insertion_params['approach_position'][0]
        approach_y = self.current_insertion_params['approach_position'][1]
        approach_z = self.locked_z  # Use locked Z instead of current position
        approach_pos = (approach_x, approach_y, approach_z)
        
        # Calculate approach yaw
        target_yaw = self.current_insertion_params['target_yaw']
        current_yaw = self.current_yaw
        
        # Limit yaw change
        yaw_diff = target_yaw - current_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        max_yaw_change = math.pi / 6  # 30 degrees
        if abs(yaw_diff) > max_yaw_change:
            yaw_diff = max_yaw_change * (1 if yaw_diff > 0 else -1)
        
        approach_yaw = current_yaw + yaw_diff
        
        rospy.loginfo(f"Z-oscillation fix: Using LOCKED Z={approach_z:.6f}m (no Z changes)")
        rospy.loginfo(f"Approaching insertion point: {approach_pos}")
        return self.execute_smooth_trajectory(start_pos, approach_pos, approach_yaw, duration=8.0)
    
    def adjust_yaw_and_circumferential_direction(self):
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        if not hasattr(self, 'current_insertion_params'):
            rospy.logerr("No insertion parameters available")
            return False
        
        insertion_params = self.current_insertion_params
        
        # CRITICAL: Always use current UAV Z to prevent oscillation
        current_uav_pos = self.get_current_position()
        if current_uav_pos is None:
            rospy.logerr("Cannot get current UAV position")
            return False
        
        # Calculate target position - use locked Z coordinate
        target_x = insertion_params['final_position'][0]
        target_y = insertion_params['final_position'][1]
        target_z = self.locked_z  # Use locked Z instead of current position
        target_pos = (target_x, target_y, target_z)
        
        target_yaw = insertion_params['target_yaw']
        
        # Limit maximum XY adjustment
        distance_adjustment = math.sqrt((target_x - current_pos[0])**2 + (target_y - current_pos[1])**2)
        max_xy_adjustment = 0.06  # Reduced for smoother movement
        if distance_adjustment > max_xy_adjustment:
            scale_factor = max_xy_adjustment / distance_adjustment
            target_x = current_pos[0] + (target_x - current_pos[0]) * scale_factor
            target_y = current_pos[1] + (target_y - current_pos[1]) * scale_factor
            target_pos = (target_x, target_y, target_z)
        
        # Move 20mm closer to valve center
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        to_valve_x = valve_center_x - target_x
        to_valve_y = valve_center_y - target_y
        
        # Move 20mm closer to valve center
        closer_adjustment = 0.020  # 20mm closer
        if math.sqrt(to_valve_x**2 + to_valve_y**2) > 0.001:  # Avoid division by zero
            target_x = target_x + closer_adjustment * to_valve_x / math.sqrt(to_valve_x**2 + to_valve_y**2)
            target_y = target_y + closer_adjustment * to_valve_y / math.sqrt(to_valve_x**2 + to_valve_y**2)
            target_pos = (target_x, target_y, target_z)
        
        rospy.loginfo(f"Z-oscillation fix: Using LOCKED Z={target_z:.6f}m (no Z changes)")
        rospy.loginfo(f"Adjusting to final position (20mm closer): {target_pos}")
        return self.execute_smooth_trajectory(current_pos, target_pos, target_yaw, duration=6.0)
    
    def descend_to_contact(self):
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        # Calculate descent target
        target_z = self.valve_pos[2] + self.z_offset
        target_pos = (current_pos[0], current_pos[1], target_z)
        
        rospy.loginfo(f"Descending to contact height: {target_z:.3f}m")
        
        # Execute controlled descent
        return self.motion_controller.execute_controlled_descent(
            start_pos=current_pos,
            target_z=target_z,
            descent_speed=self.descent_speed,
            pos_threshold=0.02,
            yaw_threshold=0.1
        )
    
    def rotate_to_target_position(self):
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        rospy.loginfo("Completing insertion - ready for rotation")
        rospy.sleep(0.5)  # Brief pause to stabilize
        
        final_pos = self.get_current_position()
        if final_pos is not None:
            rospy.loginfo(f"Final insertion position: {final_pos}")
        
        return True
    
    def execute_smooth_trajectory(self, start_pos, target_pos, target_yaw, duration=5.0):
        """Execute smooth trajectory with VEL+ACCEL mixed control and ABSOLUTE Z-lock"""
        from trajectory import PolynomialTrajectory
        
        # CRITICAL: Use locked Z coordinate from the class to prevent oscillation
        if hasattr(self, 'locked_z'):
            locked_z = self.locked_z
        else:
            locked_z = start_pos[2]
        
        # Force both start and target to use the same Z coordinate
        locked_start_pos = (start_pos[0], start_pos[1], locked_z)
        locked_target_pos = (target_pos[0], target_pos[1], locked_z)
        
        rospy.loginfo(f"VEL+ACCEL trajectory with Z-lock:")
        rospy.loginfo(f"  Start: [{locked_start_pos[0]:.6f}, {locked_start_pos[1]:.6f}, {locked_start_pos[2]:.6f}]")
        rospy.loginfo(f"  Target: [{locked_target_pos[0]:.6f}, {locked_target_pos[1]:.6f}, {locked_target_pos[2]:.6f}]")
        rospy.loginfo(f"  Z ABSOLUTELY LOCKED at: {locked_z:.6f}m (NO Z movement allowed)")
        
        # Create trajectory with absolutely locked Z coordinates
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(locked_start_pos, locked_target_pos)
        
        start_yaw = self.current_yaw
        yaw_diff = target_yaw - start_yaw
        
        # Normalize yaw difference
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Execute trajectory with VEL+ACCEL mixed control
        start_time = time.time()
        rate = rospy.Rate(50)  # Higher frequency for smoother control
        prev_pos = locked_start_pos
        prev_time = start_time
        
        # Control gains for mixed mode
        pos_gain = 0.8  # Position feedback gain
        vel_gain = 0.6  # Velocity feedforward gain
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 2.0:
            current_time = time.time()
            elapsed = current_time - start_time
            progress = min(elapsed / duration, 1.0)
            dt = current_time - prev_time
            
            # Get desired position and velocity from trajectory
            desired_pos = traj.get_next_position()
            desired_vel = traj.get_velocity()
            
            if desired_pos is None:
                desired_pos = locked_target_pos
                desired_vel = (0.0, 0.0, 0.0)
            
            # CRITICAL: Force Z to be absolutely locked
            desired_pos = (desired_pos[0], desired_pos[1], locked_z)
            
            # Force Z velocity to zero for lock
            if desired_vel is not None:
                desired_vel = (desired_vel[0], desired_vel[1], 0.0)
            else:
                desired_vel = (0.0, 0.0, 0.0)
            
            # Get current position for feedback
            current_pos = self.get_current_position()
            if current_pos is not None:
                # Position error
                pos_error = (desired_pos[0] - current_pos[0],
                           desired_pos[1] - current_pos[1],
                           locked_z - current_pos[2])
                
                # Mixed control: feedforward velocity + feedback position correction
                cmd_vel = (vel_gain * desired_vel[0] + pos_gain * pos_error[0],
                          vel_gain * desired_vel[1] + pos_gain * pos_error[1],
                          pos_gain * pos_error[2])  # Z correction for lock
                
                # Limit velocity commands for safety
                max_vel = 0.15  # 15cm/s max velocity
                cmd_vel_magnitude = math.sqrt(cmd_vel[0]**2 + cmd_vel[1]**2 + cmd_vel[2]**2)
                if cmd_vel_magnitude > max_vel:
                    scale = max_vel / cmd_vel_magnitude
                    cmd_vel = (cmd_vel[0] * scale, cmd_vel[1] * scale, cmd_vel[2] * scale)
            else:
                cmd_vel = (0.0, 0.0, 0.0)
            
            # Calculate yaw command
            desired_yaw = start_yaw + progress * yaw_diff
            yaw_error = self.normalize_angle(desired_yaw - self.current_yaw)
            cmd_yaw_vel = 0.5 * yaw_error  # Proportional yaw control
            
            # Send mixed VEL+ACCEL command
            self.motion_controller.send_velocity_command(cmd_vel, cmd_yaw_vel, desired_pos, desired_yaw)
            
            # Check convergence using relaxed thresholds for VEL control
            if current_pos is not None:
                xy_distance = math.sqrt((current_pos[0] - locked_target_pos[0])**2 + 
                                      (current_pos[1] - locked_target_pos[1])**2)
                yaw_error_final = abs(self.normalize_angle(self.current_yaw - target_yaw))
                
                # More relaxed convergence criteria for VEL control
                if xy_distance < 0.05 and yaw_error_final < 0.1 and progress >= 0.95:
                    rospy.loginfo(f"VEL trajectory converged: XY={xy_distance:.3f}m, yaw={yaw_error_final:.3f}rad")
                    break
            
            # Update previous values
            prev_pos = desired_pos
            prev_time = current_time
            rate.sleep()
        
        rospy.loginfo("VEL+ACCEL mixed control trajectory completed")
        return True


class RotateValveState(SingleUAVStateBase):
    def __init__(self, module_id=1, rotation_angle=math.pi/2, rotation_duration=8.0):
        super().__init__(outcomes=['succeeded', 'failed', 'emergency'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        self.rotation_angle = rotation_angle
        self.rotation_duration = rotation_duration
        
        # Emergency detection
        self.emergency_stop = threading.Event()
        self.last_position = None
        self.last_yaw = 0.0
        self.stuck_start_time = None
        self.stuck_threshold = 25.0
        self.movement_threshold = 0.02
        self.yaw_threshold = 0.1
    
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
            rotation_direction=1
        )
        
        if rotation_traj is None:
            rospy.logerr("Failed to create rotation trajectory")
            return 'failed'
        
        # Start trajectory
        rotation_traj.start_trajectory()
        
        # Start emergency monitoring
        emergency_thread = threading.Thread(target=self._emergency_monitor_thread)
        emergency_thread.daemon = True
        emergency_thread.start()
        
        # Execute rotation
        try:
            success = self.execute_rotation_trajectory(rotation_traj)
            
            # Stop emergency monitoring
            self.emergency_stop.set()
            
            if self.emergency_stop.is_set():
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
    
    def _emergency_monitor_thread(self):
        rate = rospy.Rate(2)  # Check every 0.5 seconds
        
        while not self.emergency_stop.is_set() and not rospy.is_shutdown():
            if self.check_rotation_emergency():
                rospy.logwarn("Emergency condition detected")
                self.emergency_stop.set()
                break
            rate.sleep()
    
    def check_rotation_emergency(self):
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
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
                else:
                    stuck_duration = time.time() - self.stuck_start_time
                    if stuck_duration > self.stuck_threshold:
                        return True
            else:
                self.stuck_start_time = None
        
        self.last_position = current_pos
        self.last_yaw = self.current_yaw
        return False
    
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
    rospy.loginfo(f"Starting valve rotation for UAV module {module_id}")
    
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
            MoveToValveState(module_id=module_id),
            transitions={'succeeded': 'DESCEND_AND_CONTACT', 'failed': 'failed'}
        )
        
        smach.StateMachine.add(
            'DESCEND_AND_CONTACT',
            DescendAndContactState(module_id=module_id),
            transitions={'succeeded': 'ROTATE_VALVE', 'failed': 'failed'}
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
        sis.stop()


if __name__ == '__main__':
    main()
