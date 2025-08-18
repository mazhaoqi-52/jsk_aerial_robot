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
        rospy.loginfo("Starting 4-stage insertion process...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None:
            return 'failed'
        
        # CRITICAL: Lock Z coordinate for entire insertion process to prevent oscillation
        self.locked_z = start_pos[2]
        rospy.loginfo(f"Z-axis LOCKED at {self.locked_z:.6f}m for entire insertion process")
        
        # 4-stage insertion process
        if not self.approach_insertion_point(start_pos):
            rospy.logerr("Failed to approach insertion point")
            return 'failed'
        
        if not self.adjust_yaw_and_circumferential_direction():
            rospy.logerr("Failed to adjust insertion direction")
            return 'failed'
        
        if not self.descend_to_contact():
            rospy.logerr("Failed to descend to contact")
            return 'failed'
        
        if not self.rotate_to_target_position():
            rospy.logerr("Failed to rotate to target position")
            return 'failed'
        
        rospy.loginfo("4-stage insertion completed successfully")
        return 'succeeded'
    
    def approach_insertion_point(self, start_pos):
        # Set rotation direction in optimizer before evaluating strategy
        self.optimizer.set_rotation_direction(self.rotation_direction)
        
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
        
        # Extract dual-fang insertion parameters based on rotation direction
        if 'fang1' in insertion_params and 'fang2' in insertion_params:
            fang1_complexity = insertion_params['fang1']['complexity_score']
            fang2_complexity = insertion_params['fang2']['complexity_score']
            
            # Select insertion strategy based on rotation direction
            rospy.loginfo(f"=== ROTATION-AWARE DUAL-FANG SELECTION ===")
            rospy.loginfo(f"Rotation direction: {'Clockwise' if self.rotation_direction == 1 else 'Counter-clockwise'}")
            
            if self.rotation_direction == 1:  # Clockwise rotation
                # For clockwise: fang1 (beam1_beam2, closer to beam1) + fang2 (beam0_beam2, closer to beam2)
                rospy.loginfo("Clockwise rotation strategy:")
                rospy.loginfo("  - Fang1: beam1_beam2 gap, closer to beam1 (provides leading edge resistance)")
                rospy.loginfo("  - Fang2: beam0_beam2 gap, closer to beam2 (provides trailing edge stability)")
                
                # For single UAV, choose the fang with better approach characteristics
                if fang1_complexity <= fang2_complexity:
                    selected_fang = 'fang1'
                    selected_params = insertion_params['fang1']
                    rospy.loginfo("Selected fang1 for single UAV (lower complexity)")
                else:
                    selected_fang = 'fang2'
                    selected_params = insertion_params['fang2']
                    rospy.loginfo("Selected fang2 for single UAV (lower complexity)")
                    
            else:  # Counter-clockwise rotation (rotation_direction == -1)
                # For counter-clockwise: fang1 (beam0_beam1, closer to beam1) + fang2 (beam1_beam2, closer to beam2)
                rospy.loginfo("Counter-clockwise rotation strategy:")
                rospy.loginfo("  - Fang1: beam0_beam1 gap, closer to beam1 (provides leading edge resistance)")
                rospy.loginfo("  - Fang2: beam1_beam2 gap, closer to beam2 (provides trailing edge stability)")
                
                # For single UAV, choose the fang with better approach characteristics
                if fang1_complexity <= fang2_complexity:
                    selected_fang = 'fang1'
                    selected_params = insertion_params['fang1']
                    rospy.loginfo("Selected fang1 for single UAV (lower complexity)")
                else:
                    selected_fang = 'fang2'
                    selected_params = insertion_params['fang2']
                    rospy.loginfo("Selected fang2 for single UAV (lower complexity)")
            
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
        
        # CRITICAL: Calculate UAV position to place end-effector INSIDE the valve
        # End-effector offset: (0.246, 0.0, 0.0743823) relative to UAV
        end_effector_offset_x = 0.246
        end_effector_offset_y = 0.0
        
        # Calculate target position - use locked Z coordinate
        target_x = insertion_params['final_position'][0]
        target_y = insertion_params['final_position'][1]
        target_z = self.locked_z
        
        # IMPORTANT: Adjust UAV position to compensate for end-effector offset
        # We want end-effector to be at target position, so UAV should be offset back
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        
        # Calculate direction from UAV to valve center for proper end-effector alignment
        uav_to_valve_x = valve_center_x - current_pos[0]
        uav_to_valve_y = valve_center_y - current_pos[1]
        distance_to_valve = math.sqrt(uav_to_valve_x**2 + uav_to_valve_y**2)
        
        if distance_to_valve > 0.001:  # Avoid division by zero
            # CORRECTED INSERTION LOGIC:
            # Goal: Place end-effector at valve inner edge for optimal gripping
            
            # Step 1: Use the calculated insertion position from optimizer
            # This position is already at the correct radius (valve inner edge)
            target_end_effector_x = insertion_params['final_position'][0]
            target_end_effector_y = insertion_params['final_position'][1]
            
            # Step 2: Calculate required UAV yaw - UAV should face toward valve center
            # This ensures end-effector can grip the valve edge properly
            target_yaw = math.atan2(valve_center_y - target_end_effector_y, valve_center_x - target_end_effector_x)
            
            # Step 3: Calculate required UAV position considering yaw orientation
            # End-effector is at UAV + (0.246, 0.0) in UAV's local frame
            # In world frame: end_effector = UAV + (0.246*cos(yaw), 0.246*sin(yaw))
            # Therefore: UAV = end_effector - (0.246*cos(yaw), 0.246*sin(yaw))
            
            target_x = target_end_effector_x - end_effector_offset_x * math.cos(target_yaw)
            target_y = target_end_effector_y - end_effector_offset_x * math.sin(target_yaw)
            
            rospy.loginfo(f"=== CORRECTED INSERTION CALCULATION ===")
            rospy.loginfo(f"Valve center: ({valve_center_x:.3f}, {valve_center_y:.3f})")
            rospy.loginfo(f"Target yaw (face valve center): {target_yaw:.3f} rad ({math.degrees(target_yaw):.1f}°)")
            rospy.loginfo(f"End-effector target (valve edge): ({target_end_effector_x:.3f}, {target_end_effector_y:.3f})")
            rospy.loginfo(f"UAV target position: ({target_x:.3f}, {target_y:.3f})")
            rospy.loginfo(f"End-effector offset compensation: ({-end_effector_offset_x * math.cos(target_yaw):.3f}, {-end_effector_offset_x * math.sin(target_yaw):.3f})")
            
            # Verification: Calculate expected end-effector position
            expected_ee_x = target_x + end_effector_offset_x * math.cos(target_yaw)
            expected_ee_y = target_y + end_effector_offset_x * math.sin(target_yaw)
            ee_error = math.sqrt((expected_ee_x - target_end_effector_x)**2 + (expected_ee_y - target_end_effector_y)**2)
            ee_distance_from_center = math.sqrt((target_end_effector_x - valve_center_x)**2 + (target_end_effector_y - valve_center_y)**2)
            rospy.loginfo(f"Verification: Expected end-effector at ({expected_ee_x:.3f}, {expected_ee_y:.3f})")
            rospy.loginfo(f"Verification: Position error: {ee_error:.6f}m")
            rospy.loginfo(f"Verification: Distance from valve center: {ee_distance_from_center:.3f}m (target: ~{self.optimizer.valve_radius:.3f}m)")
        
        target_pos = (target_x, target_y, target_z)
        # target_yaw already calculated above
        
        # Limit maximum XY adjustment for safety, but allow larger movement for proper insertion
        distance_adjustment = math.sqrt((target_x - current_pos[0])**2 + (target_y - current_pos[1])**2)
        max_xy_adjustment = 0.25  # Increased to allow proper insertion to valve center
        if distance_adjustment > max_xy_adjustment:
            scale_factor = max_xy_adjustment / distance_adjustment
            target_x = current_pos[0] + (target_x - current_pos[0]) * scale_factor
            target_y = current_pos[1] + (target_y - current_pos[1]) * scale_factor
            target_pos = (target_x, target_y, target_z)
            rospy.logwarn(f"Insertion movement limited to {max_xy_adjustment}m for safety")
        else:
            rospy.loginfo(f"Insertion movement: {distance_adjustment:.3f}m (within {max_xy_adjustment}m limit)")
        
        rospy.loginfo(f"Z-oscillation fix: Using LOCKED Z={target_z:.6f}m (no Z changes)")
        rospy.loginfo(f"Final UAV target for end-effector at valve center: {target_pos}")
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
        
        # Verify final insertion position
        if current_pos is not None:
            # Calculate actual end-effector position
            end_effector_x = current_pos[0] + 0.246 * math.cos(self.current_yaw)
            end_effector_y = current_pos[1] + 0.246 * math.sin(self.current_yaw)
            
            # Calculate distance from valve center
            valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
            distance_to_center = math.sqrt((end_effector_x - valve_center_x)**2 + (end_effector_y - valve_center_y)**2)
            
            rospy.loginfo(f"=== FINAL INSERTION VERIFICATION ===")
            rospy.loginfo(f"UAV final position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
            rospy.loginfo(f"UAV final yaw: {self.current_yaw:.3f} rad ({math.degrees(self.current_yaw):.1f}°)")
            rospy.loginfo(f"End-effector calculated position: ({end_effector_x:.3f}, {end_effector_y:.3f})")
            rospy.loginfo(f"Valve center: ({valve_center_x:.3f}, {valve_center_y:.3f})")
            rospy.loginfo(f"Distance from valve center: {distance_to_center:.3f}m")
            rospy.loginfo(f"Target valve radius (inner edge): {self.optimizer.valve_radius:.3f}m")
            
            # Check if end-effector is at correct valve edge position
            radius_error = abs(distance_to_center - self.optimizer.valve_radius)
            if radius_error < 0.01:  # Within 1cm of target radius
                rospy.loginfo("✓ End-effector successfully positioned at valve inner edge")
            elif distance_to_center < self.optimizer.valve_radius * 0.5:
                rospy.logwarn("△ End-effector positioned inside valve but too close to center")
            elif distance_to_center > self.optimizer.valve_radius * 1.2:
                rospy.logerr("✗ End-effector positioned outside valve radius")
            else:
                rospy.loginfo("○ End-effector positioned near valve edge (acceptable)")
                
            rospy.loginfo(f"Radius error from target: {radius_error:.3f}m")
        
        rospy.sleep(0.5)  # Brief pause to stabilize
        
        final_pos = self.get_current_position()
        if final_pos is not None:
            rospy.loginfo(f"Final insertion position: {final_pos}")
        
        return True
    
    def execute_smooth_trajectory(self, start_pos, target_pos, target_yaw, duration=5.0):
        """Execute smooth trajectory with VEL+ACCEL control and Z-lock"""
        from trajectory import PolynomialTrajectory
        
        # Use locked Z coordinate
        if hasattr(self, 'locked_z'):
            locked_z = self.locked_z
        else:
            locked_z = start_pos[2]
        
        # Lock Z coordinate for both start and target
        locked_start_pos = (start_pos[0], start_pos[1], locked_z)
        locked_target_pos = (target_pos[0], target_pos[1], locked_z)
        
        rospy.loginfo(f"VEL+ACCEL trajectory with Z-lock:")
        rospy.loginfo(f"  Start: [{locked_start_pos[0]:.6f}, {locked_start_pos[1]:.6f}, {locked_start_pos[2]:.6f}]")
        rospy.loginfo(f"  Target: [{locked_target_pos[0]:.6f}, {locked_target_pos[1]:.6f}, {locked_target_pos[2]:.6f}]")
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
        prev_pos = locked_start_pos
        prev_time = start_time
        
        # Adaptive control gains
        base_pos_gain = 0.8
        base_vel_gain = 0.6
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 2.0:
            current_time = time.time()
            elapsed = current_time - start_time
            progress = min(elapsed / duration, 1.0)
            dt = current_time - prev_time
            
            # Get trajectory values
            desired_pos = traj.get_next_position()
            desired_vel = traj.get_velocity()
            
            if desired_pos is None:
                desired_pos = locked_target_pos
                desired_vel = (0.0, 0.0, 0.0)
            
            # Force Z lock
            desired_pos = (desired_pos[0], desired_pos[1], locked_z)
            
            # Force Z velocity to zero
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
                           locked_z - current_pos[2])
                
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
            
            # Update for next iteration
            prev_pos = desired_pos
            prev_time = current_time
            rate.sleep()
        
        rospy.loginfo("Trajectory completed")
        return True


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
        rate = rospy.Rate(1)  # Check every 1.0 seconds (reduced frequency)
        consecutive_stuck_checks = 0
        required_consecutive_stuck = 3  # Require 3 consecutive stuck readings
        
        while not self.emergency_stop.is_set() and not rospy.is_shutdown():
            if self.check_rotation_emergency():
                consecutive_stuck_checks += 1
                rospy.logwarn(f"Potential stuck condition detected ({consecutive_stuck_checks}/{required_consecutive_stuck})")
                
                if consecutive_stuck_checks >= required_consecutive_stuck:
                    rospy.logwarn("Emergency condition confirmed after multiple checks")
                    self.emergency_stop.set()
                    break
            else:
                consecutive_stuck_checks = 0  # Reset counter if movement detected
                
            rate.sleep()
    
    def check_rotation_emergency(self):
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
        sis.stop()


if __name__ == '__main__':
    main()
