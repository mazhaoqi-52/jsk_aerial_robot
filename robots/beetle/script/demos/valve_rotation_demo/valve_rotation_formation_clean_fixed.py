#!/usr/bin/env python3
"""
Formation Valve Rotation - Clean Version
Reuses single UAV logic with minimal coordinate transformation overhead
"""

import sys
import os
import math
import threading
import time
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
from tf.transformations import euler_from_quaternion
import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(current_dir, '../..'))
sys.path.insert(0, os.path.join(current_dir, '..'))
sys.path.insert(0, current_dir)

from n_modules_tf import NModuleTFCalculator
from valve_rotation_fang_single import (
    SingleUAVStateBase, 
    InitializeStartPositionState,
    MoveToValveState, 
    DescendAndContactState,
    RotateValveState
)
from insertion_optimizer import InsertionOptimizer
from trajectory import OnlineCircularTrajectoryGenerator, PolynomialTrajectory
from beetle_interface import BeetleInterface


class WaitState(smach.State):
    """
    Simple wait state that pauses for a specified duration between state transitions.
    Provides smooth transitions and better observability for state machine execution.
    
    Args:
        wait_time (float): Duration to wait in seconds (default: 1.0)
        state_name (str): Name of the next state for logging purposes
    
    Outcomes:
        succeeded: Always returns after wait completes
    
    Example:
        # Add 1 second wait before INITIALIZE state
        smach.StateMachine.add(
            'WAIT_BEFORE_INIT',
            WaitState(wait_time=1.0, state_name="INITIALIZE"),
            transitions={'succeeded': 'INITIALIZE'}
        )
    """
    def __init__(self, wait_time=1.0, state_name=""):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.wait_time = wait_time
        self.state_name = state_name
    
    def execute(self, userdata):
        if self.state_name:
            rospy.loginfo(f"⏳ Waiting {self.wait_time}s before {self.state_name}...")
        else:
            rospy.loginfo(f"⏳ Waiting {self.wait_time}s...")
        
        rospy.sleep(self.wait_time)
        
        if self.state_name:
            rospy.loginfo(f"✓ Wait complete, proceeding to {self.state_name}")
        else:
            rospy.loginfo(f"✓ Wait complete")
        
        return 'succeeded'


class FormationUtils:
    """Utility methods shared across formation control classes"""
    
    @staticmethod
    def format_vec(vec):
        """Format 3D vector as string"""
        if vec is None:
            return "None"
        return f"({vec[0]:.3f}, {vec[1]:.3f}, {vec[2]:.3f})"
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    @staticmethod
    def get_valve_yaw_safe(beetle_interface, fallback):
        """Safe valve yaw getter with fallback"""
        valve_yaw = beetle_interface.getValveYaw()
        if valve_yaw is None:
            rospy.logwarn_once("getValveYaw() returned None, using fallback")
            return fallback
        return valve_yaw
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    @staticmethod
    def monitor_valve_rotation_continuous(current_valve_yaw, last_valve_yaw, 
                                          accumulated_rotation, last_check_time, 
                                          current_time, update_interval=0.2):
        """
        Monitor valve rotation with continuous tracking (handles >180° rotations).
        Uses incremental angle tracking to avoid discontinuities at ±π boundary.
        
        Args:
            current_valve_yaw: Current valve yaw angle
            last_valve_yaw: Previous valve yaw reading
            accumulated_rotation: Total accumulated rotation so far
            last_check_time: Time of last update
            current_time: Current time
            update_interval: Minimum interval between updates
            
        Returns: (accumulated_rotation, valve_angular_velocity, should_update, new_last_yaw, new_last_time)
        """
        valve_angular_velocity = 0.0
        should_update = current_time > last_check_time + update_interval
        
        if should_update:
            # Calculate incremental rotation with wrap-around handling
            delta_yaw = current_valve_yaw - last_valve_yaw
            
            # Handle wrap-around at ±π boundary
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
            
            # Accumulate rotation (absolute value for total rotation)
            accumulated_rotation += abs(delta_yaw)
            
            dt = current_time - last_check_time
            valve_angular_velocity = abs(delta_yaw) / dt if dt > 0 else 0.0
            
            return accumulated_rotation, valve_angular_velocity, True, current_valve_yaw, current_time
        
        return accumulated_rotation, valve_angular_velocity, False, last_valve_yaw, last_check_time
    
    @staticmethod
    def monitor_valve_rotation(current_valve_yaw, initial_valve_yaw, last_valve_yaw, 
                               last_check_time, current_time, update_interval=0.2):
        """
        Monitor valve rotation progress and calculate angular velocity (legacy, for <180° rotations)
        Returns: (valve_rotation, valve_angular_velocity, should_update, new_last_yaw, new_last_time)
        """
        valve_rotation = abs(current_valve_yaw - initial_valve_yaw)
        valve_angular_velocity = 0.0
        should_update = current_time > last_check_time + update_interval
        
        if should_update:
            dt = current_time - last_check_time
            delta_yaw = current_valve_yaw - last_valve_yaw
            # Handle wrap-around
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
            valve_angular_velocity = abs(delta_yaw) / dt if dt > 0 else 0.0
            return valve_rotation, valve_angular_velocity, True, current_valve_yaw, current_time
        
        return valve_rotation, valve_angular_velocity, False, last_valve_yaw, last_check_time


class FormationAdapter:
    """Dynamic adapter for multi-UAV formation with configurable module_ids"""
    
    def __init__(self, module_ids_str=None):
        if module_ids_str is None:
            module_ids_str = rospy.get_param("~module_ids", "2,3")
        
        rospy.loginfo(f"Initializing FormationAdapter with module_ids: {module_ids_str}")
        
        rospy.set_param("~module_ids", module_ids_str)
        self.tf_calculator = NModuleTFCalculator(module_ids_str)
        
        if not self.tf_calculator.validate_module_configuration():
            raise ValueError(f"Invalid module configuration: {module_ids_str}")
        
        self.module_ids = self.tf_calculator.module_ids
        self.leader_id = self.tf_calculator.get_leader_id()
        self.follower_ids = self.tf_calculator.get_follower_ids()
        self.number_of_modules = len(self.module_ids)
        
        rospy.loginfo(f"Formation: Leader={self.leader_id}, Followers={self.follower_ids}, Total={self.number_of_modules} modules")
        
        self.uav_positions = {}
        self.uav_orientations = {}
        self.assembly_pos = None
        self.assembly_yaw = 0.0
        
        self.position_received = threading.Event()
        
        self.uav_subscribers = {}
        for module_id in self.module_ids:
            topic = f"/beetle{module_id}/mocap/pose"
            self.uav_subscribers[module_id] = rospy.Subscriber(
                topic, PoseStamped, 
                lambda msg, mid=module_id: self.uav_callback(msg, mid),
                queue_size=1
            )
            rospy.loginfo(f"Subscribed to {topic} for module {module_id}")
        
        self.assembly_pub = rospy.Publisher(
            "/assembly/uav/nav", FlightNav, queue_size=1
        )
        
        self._calculate_offset_parameters()
        
        rospy.loginfo("FormationAdapter initialization complete")
    
    def _calculate_offset_parameters(self):
        """Calculate and store offset parameters for coordinate transformations"""
        reference_yaw = 0.0
        assembly_to_leader = self.tf_calculator.calculate_assembly_to_leader_transform()
        leader_to_ee = self.tf_calculator.calculate_leader_to_end_effector_transform(reference_yaw)
        
        self.base_offset_x = assembly_to_leader['offset_x'] + leader_to_ee['x']
        self.base_offset_y = assembly_to_leader['offset_y'] + leader_to_ee['y']
        self.total_offset_z = assembly_to_leader['offset_z'] + leader_to_ee['z']
        
        rospy.loginfo(f"Assembly→EE offsets: X={self.base_offset_x:.3f}m, Y={self.base_offset_y:.3f}m, Z={self.total_offset_z:.3f}m")
    
    def uav_callback(self, msg, module_id):
        """Receive individual UAV position from mocap (PoseStamped format)"""
        pos = msg.pose.position
        ori = msg.pose.orientation
        
        self.uav_positions[module_id] = (pos.x, pos.y, pos.z)
        
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.uav_orientations[module_id] = yaw
        
        self._update_assembly_position()
    
    def _update_assembly_position(self):
        """Calculate assembly CoG position from all participating UAVs"""
        if len(self.uav_positions) < self.number_of_modules:
            return
        
        for module_id in self.module_ids:
            if module_id not in self.uav_positions or module_id not in self.uav_orientations:
                return
        
        # Calculate assembly CoG as average of all UAV positions
        avg_x = sum(self.uav_positions[mid][0] for mid in self.module_ids) / self.number_of_modules
        avg_y = sum(self.uav_positions[mid][1] for mid in self.module_ids) / self.number_of_modules
        avg_z = sum(self.uav_positions[mid][2] for mid in self.module_ids) / self.number_of_modules
        
        self.assembly_pos = (avg_x, avg_y, avg_z)
        
        yaw_sum = 0.0
        base_yaw = self.uav_orientations[self.module_ids[0]]
        
        for module_id in self.module_ids:
            yaw = self.uav_orientations[module_id]
            yaw_diff = yaw - base_yaw
            while yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            yaw_sum += base_yaw + yaw_diff
        
        self.assembly_yaw = yaw_sum / self.number_of_modules
        
        if not self.position_received.is_set():
            rospy.loginfo(f"Formation position received: {self.assembly_pos}")
            for mid in self.module_ids:
                rospy.loginfo(f"UAV{mid}: {self.uav_positions[mid]}")
            self.position_received.set()
    
    def get_assembly_position(self):
        """Get current assembly CoG position"""
        return self.assembly_pos
    
    def get_assembly_yaw(self):
        """Get current assembly yaw"""
        return self.assembly_yaw
    
    def get_end_effector_position(self):
        """Get end-effector position using coordinate transformation"""
        if self.assembly_pos is None:
            rospy.logwarn("Assembly position not available for end-effector calculation")
            return None
            
        return self.tf_calculator.transform_assembly_to_end_effector(
            self.assembly_pos, self.assembly_yaw
        )
    
    def transform_end_effector_to_assembly_command(self, target_end_effector_pos, target_yaw=None):
        """Transform end-effector target to assembly CoG command"""
        if target_yaw is None:
            target_yaw = self.assembly_yaw if self.assembly_yaw is not None else 0.0
        
        assembly_to_leader = self.tf_calculator.calculate_assembly_to_leader_transform()
        leader_to_ee = self.tf_calculator.calculate_leader_to_end_effector_transform(target_yaw)
        
        cos_yaw = math.cos(target_yaw)
        sin_yaw = math.sin(target_yaw)
        
        total_offset_x = (assembly_to_leader['offset_x'] * cos_yaw - 
                         assembly_to_leader['offset_y'] * sin_yaw + leader_to_ee['x'])
        total_offset_y = (assembly_to_leader['offset_x'] * sin_yaw + 
                         assembly_to_leader['offset_y'] * cos_yaw + leader_to_ee['y'])
        total_offset_z = assembly_to_leader['offset_z'] + leader_to_ee['z']
        
        assembly_target_x = target_end_effector_pos[0] - total_offset_x
        assembly_target_y = target_end_effector_pos[1] - total_offset_y
        assembly_target_z = target_end_effector_pos[2] - total_offset_z
        
        return (assembly_target_x, assembly_target_y, assembly_target_z)
    
    def get_leader_id(self):
        """Get leader UAV ID"""
        return self.leader_id
    
    def get_follower_ids(self):
        """Get follower UAV IDs"""
        return self.follower_ids
    
    def wait_for_formation_ready(self, timeout=10.0):
        """Wait for all UAVs to report their positions"""
        rospy.loginfo(f"Waiting for formation to be ready ({self.number_of_modules} UAVs)...")
        
        if self.position_received.wait(timeout):
            rospy.loginfo("Formation ready - all UAVs reporting positions")
            return True
        else:
            rospy.logerr(f"Formation not ready after {timeout}s timeout")
            return False
    
    def get_end_effector_yaw(self):
        """End-effector yaw is same as assembly yaw"""
        return self.assembly_yaw
    
    def send_assembly_command(self, target_pos, target_yaw=None, linear_vel=None):
        """Send FlightNav command to assembly controller"""
        try:
            if self.assembly_pos is None:
                rospy.logwarn("Assembly position not available - command may be ineffective")
            
            nav_msg = FlightNav()
            nav_msg.header.stamp = rospy.Time.now()
            nav_msg.header.frame_id = "world"
            
            nav_msg.target = FlightNav.COG
            
            nav_msg.target_pos_x = target_pos[0]
            nav_msg.target_pos_y = target_pos[1] 
            nav_msg.target_pos_z = target_pos[2]
            
            if linear_vel is not None:
                nav_msg.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
                nav_msg.pos_z_nav_mode = FlightNav.POS_VEL_MODE
                nav_msg.target_vel_x = linear_vel[0]
                nav_msg.target_vel_y = linear_vel[1]
                nav_msg.target_vel_z = linear_vel[2]
            else:
                nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                nav_msg.target_vel_x = 0.0
                nav_msg.target_vel_y = 0.0
                nav_msg.target_vel_z = 0.0
            
            if target_yaw is not None:
                nav_msg.target_yaw = target_yaw
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE
            else:
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE
                nav_msg.target_yaw = 0.0
            
            self.assembly_pub.publish(nav_msg)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error sending assembly command: {e}")
            return False


class FormationTakeoffState(smach.State):
    """
    Wait for formation ready and takeoff (ground assembly already completed).
    This replaces FormationAssembleState as the assembly is done manually on ground.
    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
        modules_str = rospy.get_param("~module_ids", "1,2")
        self.modules = []
        if modules_str:
            self.modules = [int(x) for x in modules_str.split(',')]
        else:
            rospy.logerr("No module ID is designated!")
        
        rospy.loginfo(f"FormationTakeoffState configured for modules: {self.modules}")
        rospy.loginfo("Ground assembly assumed complete - will wait for position data and takeoff")
        
        # Formation adapter to wait for position data
        self.formation_adapter = FormationAdapter(modules_str)
    
    def execute(self, userdata):
        rospy.loginfo("=== FORMATION TAKEOFF (Ground Assembly Complete) ===")
        rospy.loginfo("Waiting for all UAV positions to be received...")
        
        try:
            # Wait for formation position data to be ready
            if not self.formation_adapter.wait_for_formation_ready(timeout=30.0):
                rospy.logerr("Timeout waiting for formation position data")
                return 'failed'
            
            rospy.loginfo("✓ All UAV positions received")
            rospy.loginfo(f"Assembly CoG position: {self.formation_adapter.get_assembly_position()}")
            rospy.loginfo(f"Assembly yaw: {self.formation_adapter.get_assembly_yaw():.3f} rad")
            
            # Give a brief moment for the system to stabilize
            rospy.loginfo("System ready for flight operations")
            rospy.sleep(1.0)
            
            rospy.loginfo("Formation takeoff state completed successfully")
            return 'succeeded'
            
        except rospy.ROSInterruptException:
            rospy.logerr("Takeoff state interrupted")
            return 'failed'
        except Exception as e:
            rospy.logerr(f"Takeoff state failed: {e}")
            return 'failed'


class FormationSingleUAVStateBase(smach.State):
    """Base class for formation states that reuse single UAV logic"""
    
    _shared_target_z = None
    
    def __init__(self, outcomes, input_keys=None, output_keys=None):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys or [], output_keys=output_keys or [])
        
        module_ids_str = rospy.get_param("~module_ids", "2,3") 
        
        self.formation_adapter = FormationAdapter(module_ids_str)
        self.leader_id = self.formation_adapter.get_leader_id()
        
        if not self.formation_adapter.wait_for_formation_ready(timeout=15.0):
            raise RuntimeError("Formation not ready - cannot initialize state")
        
        self.beetle = BeetleInterface(
            module_id=self.leader_id,
            assembly_mode=True,
            assembly_tf_calculator=self.formation_adapter.tf_calculator
        )
        
        self.optimizer = InsertionOptimizer()
        
        self.max_linear_velocity = 0.12
        self.average_linear_velocity = 0.08
        self.max_angular_velocity = 0.02
        
        rospy.loginfo(f"FormationState initialized for leader UAV{self.leader_id}")
    
    def get_assembly_position(self):
        return self.formation_adapter.get_assembly_position()
    
    def get_assembly_yaw(self):
        return self.formation_adapter.get_assembly_yaw()
    
    def get_end_effector_position(self):
        return self.formation_adapter.get_end_effector_position()
    
    def get_end_effector_yaw(self):
        return self.formation_adapter.get_end_effector_yaw()
    
    def active_position_convergence(self, target_pos, target_yaw, pos_thresh=0.025, yaw_thresh=0.0175, timeout=15.0, max_yaw_step=None, max_linear_vel=None, max_angular_vel=None, adjustment_pos_thresh=None, yaw_adjustment_thresh=None, lock_z=False, decomposition_threshold=None):
        """Formation-style active convergence with trajectory decomposition
        
        Args:
            pos_thresh: Convergence success threshold (default: 25mm)
            adjustment_pos_thresh: Threshold to trigger position adjustment (default: same as pos_thresh)
                                   Smaller value = more aggressive correction
            yaw_thresh: Yaw convergence success threshold (for determining if converged)
            yaw_adjustment_thresh: Threshold to trigger yaw adjustment (default: 0.5°)
                                   Smaller value = more aggressive yaw correction
            lock_z: If True, lock Z velocity to 0 and only correct XY (prevents pitch oscillation during XY correction)
            decomposition_threshold: Distance threshold for trajectory decomposition (default: 0.08m for Z descent, 0.04m for XY)
        """
        start_time = rospy.get_time()
        
        # Dual-threshold: adjustment_pos_thresh triggers correction, pos_thresh determines success
        if adjustment_pos_thresh is None:
            adjustment_pos_thresh = pos_thresh
        
        # Dual-threshold for yaw: yaw_adjustment_thresh triggers correction, yaw_thresh determines success
        # Default: adjust when yaw error > 0.5° (0.00873 rad)
        if yaw_adjustment_thresh is None:
            yaw_adjustment_thresh = 0.00873  # 0.5 degrees - always actively correct yaw
        
        current_pos = self.get_end_effector_position()
        if current_pos is None:
            rospy.logerr("[Convergence] Cannot get current position, skipping decomposition")
        
        if current_pos is not None:
            initial_distance = np.linalg.norm(np.array(target_pos) - np.array(current_pos))
            
            # Use provided decomposition_threshold or auto-detect based on motion pattern
            if decomposition_threshold is None:
                # Auto-detect: Z descent (large Z change) uses 80mm, XY motion uses 40mm
                z_change = abs(target_pos[2] - current_pos[2])
                xy_change = math.sqrt((target_pos[0] - current_pos[0])**2 + (target_pos[1] - current_pos[1])**2)
                if z_change > xy_change * 0.5:  # Primarily Z motion
                    VEL_NAV_THRESHOLD = 0.08  # 80mm for Z descent
                else:  # Primarily XY motion
                    VEL_NAV_THRESHOLD = 0.04  # 40mm for XY approach
            else:
                VEL_NAV_THRESHOLD = decomposition_threshold
            
            SAFE_STEP_SIZE = 0.03  # 30mm per step for finer control
            
            if initial_distance > VEL_NAV_THRESHOLD:
                rospy.loginfo(f"[Trajectory Decomposition] Distance {initial_distance*1000:.1f}mm > {VEL_NAV_THRESHOLD*1000:.0f}mm threshold")
                
                num_steps = max(2, int(np.ceil(initial_distance / SAFE_STEP_SIZE)))
                rospy.loginfo(f"Executing {num_steps} waypoints, ~{SAFE_STEP_SIZE*1000:.0f}mm per step")
                
                # Get speed limits for trajectory decomposition (defaults if not provided)
                decomp_linear_vel_limit = max_linear_vel if max_linear_vel is not None else 0.08
                decomp_angular_vel_limit = max_angular_vel if max_angular_vel is not None else 0.05
                
                # Determine if Z should be locked during decomposition
                # Use parameter lock_z if provided, otherwise auto-detect based on motion pattern
                z_change = abs(target_pos[2] - current_pos[2])
                xy_change = math.sqrt((target_pos[0] - current_pos[0])**2 + (target_pos[1] - current_pos[1])**2)
                decomp_lock_z = lock_z or (z_change < 0.03 and xy_change > z_change * 3)
                
                for i in range(1, num_steps):
                    alpha = i / num_steps
                    intermediate_pos = (
                        current_pos[0] + alpha * (target_pos[0] - current_pos[0]),
                        current_pos[1] + alpha * (target_pos[1] - current_pos[1]),
                        target_pos[2] if decomp_lock_z else current_pos[2] + alpha * (target_pos[2] - current_pos[2])
                    )
                    
                    # Calculate velocity direction from current actual position to target
                    step_vector = np.array(intermediate_pos) - np.array(current_pos)
                    step_distance = np.linalg.norm(step_vector)
                    
                    if step_distance > 0.001:
                        step_direction = step_vector / step_distance
                        # When lock_z is True, force Z velocity to 0
                        z_vel_component = 0.0 if decomp_lock_z else step_direction[2] * decomp_linear_vel_limit
                        controlled_linear_vel = [
                            step_direction[0] * decomp_linear_vel_limit,
                            step_direction[1] * decomp_linear_vel_limit,
                            z_vel_component
                        ]
                    else:
                        controlled_linear_vel = None
                    
                    self.send_assembly_command_from_end_effector(
                        intermediate_pos, 
                        target_yaw,
                        linear_vel=controlled_linear_vel,
                        angular_vel=decomp_angular_vel_limit
                    )
                    
                    # No fixed wait - let controller naturally converge
                    # rospy.sleep(0.15)  # REMOVED: Causes jerky motion
                    
                    current_pos = self.get_end_effector_position()
                    if current_pos is None:
                        rospy.logwarn(f"[Decomposition] Lost position at waypoint {i}, using predicted position")
                        current_pos = intermediate_pos
                    
                    if i % max(1, num_steps // 5) == 0:
                        rospy.loginfo(f"Waypoint {i}/{num_steps-1} ({100*i/num_steps:.0f}%)")
                
                rospy.loginfo("Trajectory decomposition complete")
                current_pos = self.get_end_effector_position()
                if current_pos is not None:
                    remaining = np.linalg.norm(np.array(target_pos) - np.array(current_pos))
                    rospy.loginfo(f"Remaining distance: {remaining*1000:.1f}mm")
        
        max_angular_vel_limit = max_angular_vel if max_angular_vel is not None else 0.05
        max_linear_vel_limit = max_linear_vel if max_linear_vel is not None else 0.08
        
        consecutive_good_readings = 0
        required_consecutive = 8
        
        # Initialize error variables to avoid UnboundLocalError on timeout
        pos_error = float('inf')
        yaw_error = float('inf')
        
        rospy.loginfo(f"Active convergence: target={FormationUtils.format_vec(target_pos)}, yaw={math.degrees(target_yaw):.1f}°, "
                      f"conv_thresh={pos_thresh*1000:.1f}mm, adj_thresh={adjustment_pos_thresh*1000:.1f}mm, "
                      f"yaw_thresh={math.degrees(yaw_thresh):.1f}°, yaw_adj_thresh={math.degrees(yaw_adjustment_thresh):.1f}°")
        
        while rospy.get_time() - start_time < timeout:
            current_pos = self.get_end_effector_position()
            if current_pos is None:
                rospy.sleep(0.1)
                continue
            
            current_yaw = self.get_end_effector_yaw()
            if current_yaw is None:
                rospy.sleep(0.1)
                continue
            
            pos_error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
            yaw_error = abs(FormationUtils.normalize_angle(target_yaw - current_yaw))
            
            # Dual-threshold for position: converged_ok for success, needs_adjustment for correction
            converged_ok = pos_error < pos_thresh
            needs_adjustment = pos_error >= adjustment_pos_thresh
            # Dual-threshold for yaw: yaw_ok for success判定, yaw_needs_adjustment for correction
            yaw_ok = yaw_error < yaw_thresh  # Success判定: 8° (relaxed)
            yaw_needs_adjustment = yaw_error >= yaw_adjustment_thresh  # Adjustment trigger: 0.5° (aggressive)
            
            if converged_ok and yaw_ok:
                consecutive_good_readings += 1
                if consecutive_good_readings >= required_consecutive:
                    rospy.loginfo(f"Convergence success: pos={pos_error*1000:.1f}mm, yaw={math.degrees(yaw_error):.1f}°")
                    return True
            else:
                consecutive_good_readings = 0
            
            # Apply adjustment if position needs correction OR yaw needs adjustment (>0.5°)
            if needs_adjustment or yaw_needs_adjustment:
                command_yaw = target_yaw
                if max_yaw_step is not None and max_yaw_step > 0.0:
                    yaw_delta = FormationUtils.normalize_angle(target_yaw - current_yaw)
                    if abs(yaw_delta) > max_yaw_step:
                        yaw_delta = math.copysign(max_yaw_step, yaw_delta)
                        command_yaw = FormationUtils.normalize_angle(current_yaw + yaw_delta)
                        elapsed = rospy.get_time() - start_time
                        if int(elapsed * 5.0) % 20 == 0:
                            rospy.loginfo(f"[Formation YawLimit] Step limited to {math.degrees(yaw_delta):.1f}° "
                                        f"(remaining {math.degrees(abs(FormationUtils.normalize_angle(target_yaw - current_yaw))):.1f}°)")
                
                command_pos = target_pos
                
                max_angular_vel_limit = max_angular_vel if max_angular_vel is not None else 0.05
                max_linear_vel_limit = max_linear_vel if max_linear_vel is not None else 0.08
                
                actual_yaw_error = abs(FormationUtils.normalize_angle(command_yaw - current_yaw))
                
                if actual_yaw_error > 0.52:
                    angular_divisor = 2.0
                elif actual_yaw_error > 0.17:
                    angular_divisor = 2.5
                else:
                    angular_divisor = 4.0
                
                smooth_angular_vel = min(actual_yaw_error / angular_divisor, max_angular_vel_limit) if actual_yaw_error > 0.005 else 0.0
                
                if pos_error > 0.02:
                    speed_magnitude = min(pos_error / 4.0, max_linear_vel_limit)
                    direction = np.array(target_pos) - np.array(current_pos)
                    direction_norm = np.linalg.norm(direction)
                    if direction_norm > 0.001:
                        direction = direction / direction_norm
                        # CRITICAL: When lock_z=True, force Z velocity to 0 to prevent pitch oscillation
                        z_vel = 0.0 if lock_z else direction[2] * speed_magnitude
                        smooth_linear_vel = [direction[0] * speed_magnitude, 
                                           direction[1] * speed_magnitude, 
                                           z_vel]
                    else:
                        smooth_linear_vel = None
                else:
                    smooth_linear_vel = None
                
                self.send_assembly_command_from_end_effector(
                    target_end_effector_pos=command_pos,
                    target_yaw=command_yaw,
                    linear_vel=smooth_linear_vel,
                    angular_vel=smooth_angular_vel
                )
                
                elapsed = rospy.get_time() - start_time
                if int(elapsed * 2.0) % 10 == 0:
                    rospy.loginfo(f"[Converging] pos_err={pos_error*1000:.1f}mm, "
                                f"yaw_err={math.degrees(yaw_error):.1f}°, t={elapsed:.1f}s")
            
            rospy.sleep(0.04)
        
        rospy.logwarn(f"Formation active convergence timeout after {timeout:.1f}s: "
                     f"pos_err={pos_error*1000:.1f}mm, yaw_err={math.degrees(yaw_error):.1f}°")
        return False

    def send_assembly_command_from_end_effector(self, target_end_effector_pos, target_yaw=None,
                                               linear_vel=None, angular_vel=None):
        """Send assembly command by transforming end-effector target to assembly CoG"""
        assembly_target = self.formation_adapter.transform_end_effector_to_assembly_command(
            target_end_effector_pos, target_yaw
        )

        ee_target_vec = np.array(target_end_effector_pos, dtype=float)
        assembly_target_vec = np.array(assembly_target, dtype=float)
        
        linear_vel_vec = None
        if linear_vel is not None:
            if isinstance(linear_vel, (int, float)):
                linear_vel_vec = np.array([0.0, 0.0, float(linear_vel)], dtype=float)
            else:
                linear_array = np.array(linear_vel, dtype=float).flatten()
                if linear_array.size == 3:
                    linear_vel_vec = linear_array
                else:
                    rospy.logwarn_once("Invalid linear_vel size; ignoring")

        omega_vec = np.zeros(3, dtype=float)
        if angular_vel is not None:
            if isinstance(angular_vel, (int, float)):
                omega_vec[2] = float(angular_vel)
            else:
                angular_array = np.array(angular_vel, dtype=float).flatten()
                if angular_array.size == 1:
                    omega_vec[2] = float(angular_array[0])
                elif angular_array.size == 3:
                    omega_vec = angular_array
                else:
                    rospy.logwarn_once("Invalid angular_vel size; ignoring")
                    omega_vec = np.zeros(3, dtype=float)

        angular_vel_cmd = omega_vec.tolist() if not np.allclose(omega_vec, 0.0) else None

        assembly_linear_vec = None
        if linear_vel_vec is not None or angular_vel_cmd is not None:
            v_ee = linear_vel_vec if linear_vel_vec is not None else np.zeros(3, dtype=float)
            ee_offset = ee_target_vec - assembly_target_vec
            v_assembly = v_ee - np.cross(omega_vec, ee_offset)
            assembly_linear_vec = v_assembly.tolist()
        
        self.beetle.targetMotion(
            pos=assembly_target,
            rot=target_yaw,
            linear_vel=assembly_linear_vec,
            angular_vel=angular_vel_cmd
        )


    def controlled_z_descent(self, start_pos, final_target, final_yaw, descent_speed=0.12):
        """Controlled Z-axis descent in fixed steps with contact detection"""
        rospy.loginfo("=== Formation Z Descent ===")

        if start_pos is None or final_target is None:
            rospy.logerr("[Z Descent] Missing start or target position")
            return False, None

        target_x, target_y, target_z = final_target
        start_z = start_pos[2]
        total_z_descent = start_z - target_z
        rospy.loginfo(f"[Z Descent] Total descent {total_z_descent*1000:.1f}mm")

        if total_z_descent <= 0.01:
            rospy.loginfo("[Z Descent] Small descent, direct convergence")
            self.send_assembly_command_from_end_effector((target_x, target_y, target_z), final_yaw)
            success = self.active_position_convergence(
                (target_x, target_y, target_z),
                target_yaw=final_yaw,
                pos_thresh=0.080,
                yaw_thresh=0.087,
                timeout=10.0,
                adjustment_pos_thresh=0.030  # 30mm triggers adjustment
            )
            achieved = self.get_end_effector_position() or (target_x, target_y, target_z)
            return success, achieved

        # Step size parameters
        fixed_step_size = 0.05
        planned_steps = max(3, int(math.ceil(total_z_descent / fixed_step_size)))
        max_single_step = 0.03

        rospy.loginfo(f"[Z Descent] {planned_steps} steps planned, {fixed_step_size*1000:.1f}mm per step")

        current_pos = start_pos
        previous_z = current_pos[2]
        planned_z = current_pos[2]  # Track planned trajectory Z (never regresses)
        total_descent_completed = 0.0
        step_count = 0
        progress_ratio = 0.0
        max_step_iterations = max(planned_steps * 6, planned_steps + 40)
        achieved_position = current_pos
        contact_xy_tolerance = 0.080  # 80mm (RELAXED from 22mm for formation)
        contact_z_tolerance = 0.05
        plateau_z_tolerance = 0.25
        plateau_progress_threshold = 0.4
        stagnation_threshold = 0.012
        contact_required_hits = 3
        plateau_required_hits = 5
        stagnation_hit_counter = 0
        proximity_hit_counter = 0
        final_descent_margin = 0.03
        
        # Motion tracking for contact detection
        # Single detection is sufficient - robot is stable enough for immediate response
        consecutive_small_motions = 0
        small_motion_threshold = 0.005  # 5mm threshold for contact detection
        max_consecutive_small_motions = 1  # Reduced from 3 to 1 - single detection sufficient
        
        # One-time stabilization flag at 50% progress
        stabilization_50_done = False

        while planned_z - target_z > 1e-4:
            if step_count >= max_step_iterations:
                # Check if within tolerance despite timeout
                current_pos = self.get_end_effector_position() or achieved_position
                final_xy_error = math.sqrt((current_pos[0] - target_x)**2 + (current_pos[1] - target_y)**2)
                final_z_error = abs(current_pos[2] - target_z)
                
                if final_xy_error <= 0.030 and final_z_error <= 0.020:
                    rospy.logwarn(
                        f"[Z Descent] Timeout but within tolerance: XY {final_xy_error*1000:.1f}mm, "
                        f"Z {final_z_error*1000:.1f}mm"
                    )
                    return True, current_pos
                
                rospy.logerr("[Z Descent] Exceeded safety iteration limit")
                return False, achieved_position

            # Use planned_z for step calculation (never regresses)
            remaining_descent = max(0.0, planned_z - target_z)
            if remaining_descent <= final_descent_margin:
                rospy.loginfo(f"[Z Descent] Remaining {remaining_descent*1000:.1f}mm within margin")
                break

            # Adaptive step size for final approach
            if remaining_descent <= 0.06:
                adaptive_step_size = 0.015
            elif remaining_descent <= 0.12:
                adaptive_step_size = 0.025
            else:
                adaptive_step_size = fixed_step_size
            
            step_size = min(adaptive_step_size, max_single_step, remaining_descent)
            current_z = planned_z - step_size  # Use planned_z instead of previous_z
            if current_z < target_z:
                current_z = target_z

            step_count += 1
            progress = progress_ratio

            step_target = [target_x, target_y, current_z]

            # Log first step Z transition for debugging
            if step_count == 1:
                rospy.loginfo(f"[Z_DESCENT_STEP1] Planned Z={planned_z:.3f}m → Step target Z={current_z:.3f}m (Δ={-(current_z - planned_z)*1000:.1f}mm)")
                current_ee = self.get_end_effector_position()
                if current_ee:
                    rospy.loginfo(f"[Z_DESCENT_STEP1] Current EE: ({current_ee[0]:.3f}, {current_ee[1]:.3f}, {current_ee[2]:.3f})")
                    initial_3d_dist = math.sqrt((step_target[0]-current_ee[0])**2 + (step_target[1]-current_ee[1])**2 + (step_target[2]-current_ee[2])**2)
                    rospy.loginfo(f"[Z_DESCENT_STEP1] 3D distance to step1 target: {initial_3d_dist*1000:.1f}mm")

            # Dual-threshold: 30mm triggers adjustment, 80mm for convergence success
            step_pos_thresh = 0.080
            step_adjustment_thresh = 0.050  # Relaxed from 30mm to 50mm for smoother motion
            # Relaxed yaw threshold for Z descent: formation has coupling effects during descent
            # Changed from 0.087 (5°) to 0.14 (~8°) to accommodate yaw drift during Z motion
            step_yaw_thresh = 0.14
            is_final_step = current_z <= target_z + 1e-4 or remaining_descent <= step_size + 1e-6
            is_second_last_step = (planned_steps - step_count) == 2
            is_third_last_step = (planned_steps - step_count) == 3

            # Unified descent speed strategy - proven 0.036 m/s works well for stability
            # Reverted from 0.05 m/s which caused rapid descent issues
            effective_descent_speed = max(0.03, descent_speed * 0.3)  # ≈ 0.036 m/s
            linear_vel = [0.0, 0.0, -effective_descent_speed]

            rospy.loginfo(f"[Z Descent] Step {step_count}/{planned_steps} Z={step_target[2]:.3f}m, remain {remaining_descent*1000:.1f}mm, speed={effective_descent_speed:.3f}m/s")

            self.send_assembly_command_from_end_effector(step_target, final_yaw, linear_vel=linear_vel, angular_vel=0.0)

            # Extended timeout for final steps (keep longer timeout, but remove max_pos_step restriction)
            if is_final_step or is_second_last_step or is_third_last_step:
                step_timeout = 17.0
                step_max_attempts = 100
                rospy.loginfo(f"  Extended timeout: {step_timeout}s")
            else:
                step_timeout = 12.0
                step_max_attempts = 80

            # Unified convergence with dual-threshold (30mm adjustment, 80mm success)
            step_converged = self.active_position_convergence(
                target_pos=step_target,
                target_yaw=final_yaw,
                pos_thresh=step_pos_thresh,
                yaw_thresh=step_yaw_thresh,
                timeout=step_timeout,
                adjustment_pos_thresh=step_adjustment_thresh,
                decomposition_threshold=0.08  # Force 80mm threshold for Z descent
            )

            if not step_converged:
                achieved_position = self.get_end_effector_position() or tuple(step_target)
                xy_residual = math.sqrt(
                    (achieved_position[0] - target_x)**2 +
                    (achieved_position[1] - target_y)**2
                )
                z_residual = achieved_position[2] - target_z
                last_motion = abs(previous_z - achieved_position[2])
                attempted_descent_completed = max(0.0, start_z - achieved_position[2])
                attempted_progress_ratio = (
                    0.0 if total_z_descent <= 1e-6
                    else min(1.0, attempted_descent_completed / total_z_descent)
                )

                # Check small motion during convergence failure
                if last_motion <= small_motion_threshold:
                    consecutive_small_motions += 1
                    rospy.loginfo(f"[Z Descent] Small motion detected {consecutive_small_motions}/{max_consecutive_small_motions}: dZ={last_motion*1000:.1f}mm")
                    
                    if consecutive_small_motions >= max_consecutive_small_motions:
                        # Physical contact detected - require consecutive small motions
                        if xy_residual <= 0.030:
                            rospy.loginfo(
                                f"[Z Descent] Contact success: {max_consecutive_small_motions} small motions + XY {xy_residual*1000:.1f}mm ≤30mm"
                            )
                            rospy.loginfo(f"[Z Descent] Physical limit reached at Z={achieved_position[2]:.3f}m")
                            rospy.loginfo(f"[Z Descent] Z residual {z_residual*1000:.1f}mm due to contact")
                            return True, achieved_position
                        elif z_residual <= 0.060 and xy_residual <= 0.040:
                            rospy.loginfo(
                                f"[Z Descent] Z tolerance success: Z_res={z_residual*1000:.1f}mm ≤60mm, "
                                f"XY={xy_residual*1000:.1f}mm ≤40mm"
                            )
                            rospy.loginfo(f"[Z Descent] Accepting insertion depth Z={achieved_position[2]:.3f}m")
                            return True, achieved_position
                        else:
                            rospy.logwarn(f"[Z Descent] Small motion but insufficient precision: XY={xy_residual*1000:.1f}mm, Z_res={z_residual*1000:.1f}mm")
                else:
                    consecutive_small_motions = 0

                motion_within_threshold = last_motion <= stagnation_threshold
                if motion_within_threshold:
                    stagnation_hit_counter += 1
                else:
                    stagnation_hit_counter = 0

                within_proximity = (
                    0.0 <= z_residual <= contact_z_tolerance
                    and xy_residual <= contact_xy_tolerance
                )

                if within_proximity:
                    proximity_hit_counter += 1
                else:
                    proximity_hit_counter = 0

                contact_ready = (
                    within_proximity
                    and (
                        (motion_within_threshold and stagnation_hit_counter >= contact_required_hits)
                        or proximity_hit_counter >= contact_required_hits
                    )
                )
                plateau_ready = motion_within_threshold and stagnation_hit_counter >= plateau_required_hits

                within_contact_window = contact_ready

                plateau_reached = (
                    plateau_ready
                    and within_proximity
                    and 0.0 <= z_residual <= plateau_z_tolerance
                    and xy_residual <= contact_xy_tolerance * 1.4
                    and total_z_descent > 1e-4
                    and attempted_progress_ratio >= plateau_progress_threshold
                )

                if within_contact_window or plateau_reached:
                    contact_label = "Contact" if within_contact_window else "Plateau"
                    rospy.logwarn(f"[Z Descent] {contact_label} at step {step_count}: XY {xy_residual*1000:.1f}mm, Z {z_residual*1000:.1f}mm, locking height")
                    target_z = achieved_position[2]
                    previous_z = achieved_position[2]
                    proximity_hit_counter = 0
                    stagnation_hit_counter = 0
                    break

                # Tolerance-oriented success check
                z_tolerance_success = abs(z_residual) <= 0.040
                xy_reasonable_success = xy_residual <= 0.030
                
                if z_tolerance_success and xy_reasonable_success:
                    rospy.loginfo(f"[Z Descent] Tolerance success at step {step_count}: XY={xy_residual*1000:.1f}mm, Z_err={abs(z_residual)*1000:.1f}mm, Z={achieved_position[2]:.3f}m")
                    return True, achieved_position
                
                rospy.logerr(f"[Z Descent] Step {step_count} failed: XY={xy_residual*1000:.1f}mm, Z_res={z_residual*1000:.1f}mm")
                return False, achieved_position

            current_pos = self.get_end_effector_position() or tuple(step_target)
            actual_descent = max(0.0, previous_z - current_pos[2])
            
            # Consecutive small motion detection on success path
            if actual_descent <= small_motion_threshold:
                if step_converged:
                    consecutive_small_motions += 1
                    rospy.loginfo(f"[Z Descent] Small motion on success {consecutive_small_motions}/{max_consecutive_small_motions}: dZ={actual_descent*1000:.1f}mm")
                
                if consecutive_small_motions >= max_consecutive_small_motions:
                    # Check minimum descent progress before considering contact
                    descent_completed = max(0.0, start_z - current_pos[2])
                    min_descent_for_contact = total_z_descent * 0.90  # Increased from 70% to 90% for stricter validation
                    z_residual = current_pos[2] - target_z
                    
                    # Three-level validation to prevent false contact detection
                    if descent_completed < min_descent_for_contact:
                        rospy.loginfo(f"[Z Descent] Small motion but only {descent_completed*1000:.1f}mm/{total_z_descent*1000:.1f}mm ({descent_completed/total_z_descent*100:.0f}%) descended, need 90%, ignoring")
                        consecutive_small_motions = 0
                    elif z_residual > 0.030:  # NEW: Z residual check - must be within 30mm of target
                        rospy.loginfo(f"[Z Descent] Small motion but Z residual {z_residual*1000:.1f}mm > 30mm, continue descent")
                        consecutive_small_motions = 0
                    else:
                        # Physical contact confirmed - all criteria met:
                        # 1. 90%+ progress  2. Z residual ≤30mm  3. XY precision ≤30mm
                        current_xy_error = math.sqrt((current_pos[0] - target_x)**2 + (current_pos[1] - target_y)**2)
                        if current_xy_error <= 0.030:
                            rospy.loginfo(f"[Z Descent] Contact confirmed: progress={descent_completed/total_z_descent*100:.0f}%, z_res={z_residual*1000:.1f}mm, xy={current_xy_error*1000:.1f}mm, Z={current_pos[2]:.3f}m")
                            return True, current_pos
                        else:
                            rospy.logwarn(f"[Z Descent] Small motion but XY={current_xy_error*1000:.1f}mm>30mm")
                            consecutive_small_motions = 0
            else:
                if step_converged:
                    consecutive_small_motions = 0
            
            if actual_descent < 1e-4 and remaining_descent > final_descent_margin:
                rospy.logdebug(
                    f"[Formation Z Descent] Step {step_count} 实际下降不足 0.1mm (remaining {remaining_descent*1000:.1f}mm)"
                )
            stagnation_hit_counter = 0
            proximity_hit_counter = 0
            
            # Post-step XY verification at specific progress milestones only (20%, 40%, 60%, 80%)
            # Removed per-step checking to maintain descent continuity
            progress_milestones = [0.2, 0.4, 0.6, 0.8]
            milestone_tolerance = 0.05  # ±5% tolerance for milestone detection
            should_check_xy = any(abs(progress_ratio - m) <= milestone_tolerance for m in progress_milestones)
            
            if should_check_xy:
                xy_error = math.sqrt(
                    (current_pos[0] - step_target[0])**2 +
                    (current_pos[1] - step_target[1])**2
                )
                xy_check_threshold = 0.045  # Relaxed from 25mm to 45mm
                
                if xy_error > xy_check_threshold:
                    rospy.logwarn(f"[Formation Z Descent] Progress {progress_ratio*100:.0f}%, XY error {xy_error*1000:.1f}mm > {xy_check_threshold*1000:.0f}mm, performing XY correction...")
                    
                    # XY correction: Lock Z and yaw, only correct XY (following single UAV pattern)
                    # Use current Z position, not step target Z
                    xy_correction_target = [step_target[0], step_target[1], current_pos[2]]
                    self.send_assembly_command_from_end_effector(
                        xy_correction_target, 
                        final_yaw,
                        linear_vel=None,  # Use default controller (like single UAV)
                        angular_vel=0.0   # Lock yaw
                    )
                    
                    # Wait for XY correction convergence with strict XY tolerance
                    xy_corrected = self.active_position_convergence(
                        target_pos=xy_correction_target,
                        target_yaw=final_yaw,
                        pos_thresh=0.015,  # Strict 15mm XY tolerance (like single UAV)
                        yaw_thresh=0.087,
                        timeout=8.0,
                        adjustment_pos_thresh=0.015
                        # Note: No lock_z - let controller handle Z naturally
                    )
                    
                    if xy_corrected:
                        rospy.loginfo(f"✓ XY correction successful at {progress_ratio*100:.0f}% progress")
                    else:
                        rospy.logwarn(f"⚠ XY correction timeout at {progress_ratio*100:.0f}% progress, but continuing")
                        # Don't fail - continue with descent (like single UAV behavior)
                else:
                    rospy.loginfo(f"[Formation Z Descent] Progress {progress_ratio*100:.0f}%, XY error {xy_error*1000:.1f}mm acceptable")

            # One-time stabilization at 50% progress with XY correction (only if needed)
            if not stabilization_50_done and progress_ratio >= 0.5:
                # Check if XY correction is actually needed
                xy_error_50 = math.sqrt(
                    (current_pos[0] - target_x)**2 +
                    (current_pos[1] - target_y)**2
                )
                xy_correction_threshold = 0.030  # 30mm threshold
                
                if xy_error_50 > xy_correction_threshold:
                    rospy.loginfo(f"[Z Descent] 50% progress reached ({progress_ratio*100:.0f}%), XY error {xy_error_50*1000:.1f}mm > 30mm, performing XY correction + 3s stabilization")
                    
                    # XY correction target: use target XY with current Z
                    correction_target = [target_x, target_y, current_pos[2]]
                    
                    # First do XY correction convergence
                    xy_corrected = self.active_position_convergence(
                        target_pos=correction_target,
                        target_yaw=final_yaw,
                        pos_thresh=0.020,  # 20mm strict tolerance
                        yaw_thresh=0.087,
                        timeout=5.0,
                        adjustment_pos_thresh=0.015
                    )
                    if xy_corrected:
                        rospy.loginfo("✓ 50% XY correction successful")
                    else:
                        rospy.logwarn("⚠ 50% XY correction timeout")
                    
                    # Then 3s active stabilization at corrected position
                    stabilization_target = [target_x, target_y, current_pos[2]]
                    self.active_stabilization_wait(stabilization_target, final_yaw, 3.0, "50% progress XY stabilization")
                    
                    # Update current position after stabilization
                    current_pos = self.get_end_effector_position() or current_pos
                else:
                    rospy.loginfo(f"[Z Descent] 50% progress reached ({progress_ratio*100:.0f}%), XY error {xy_error_50*1000:.1f}mm ≤ 30mm, skipping stabilization for continuity")
                
                stabilization_50_done = True

            # Update tracking variables
            # planned_z: follows planned trajectory (never regresses) - for step calculation
            # previous_z: actual position - for safety checks and motion detection
            planned_z = step_target[2]  # Planned trajectory Z (never regresses)
            previous_z = current_pos[2]  # Actual Z for safety checks
            achieved_position = current_pos
            total_descent_completed = max(0.0, start_z - planned_z)
            progress_ratio = 0.0 if total_z_descent <= 1e-6 else min(1.0, total_descent_completed / total_z_descent)

        achieved_position = self.get_end_effector_position() or achieved_position

        rospy.loginfo("[Formation Z Descent] Completed all steps successfully")
        return True, (target_x, target_y, achieved_position[2])

    def active_stabilization_wait(self, target_pos, target_yaw, duration, description="position stabilization"):
        """
        Active stabilization: continuously send target commands during wait period.
        Uses POS_VEL_MODE with zero velocity to maintain control mode consistency.
        
        Args:
            target_pos: Target end-effector position (x, y, z)
            target_yaw: Target yaw angle (radians)
            duration: Wait duration in seconds
            description: Description for logging
        """
        rospy.loginfo(f"Active stabilization: {duration}s {description}")
        rospy.loginfo(f"  Target: pos=({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}), yaw={math.degrees(target_yaw):.1f}°")
        
        start_time = rospy.get_time()
        rate = rospy.Rate(10)
        
        while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
            # Use zero velocity to maintain POS_VEL_MODE consistency with Z_DESCENT
            self.send_assembly_command_from_end_effector(
                target_pos, target_yaw,
                linear_vel=[0.0, 0.0, 0.0],
                angular_vel=0.0
            )
            
            # Log progress every second
            elapsed = rospy.get_time() - start_time
            if int(elapsed) != int(elapsed - 0.1):
                current_pos = self.get_end_effector_position()
                current_yaw = self.get_end_effector_yaw()
                if current_pos is not None and current_yaw is not None:
                    pos_error = ((target_pos[0] - current_pos[0])**2 + 
                                (target_pos[1] - current_pos[1])**2 + 
                                (target_pos[2] - current_pos[2])**2)**0.5
                    yaw_error = abs(FormationUtils.normalize_angle(target_yaw - current_yaw))
                    rospy.loginfo(f"  Progress: {elapsed:.1f}s, pos_err={pos_error*1000:.1f}mm, yaw_err={math.degrees(yaw_error):.1f}°")
            
            rate.sleep()
        
        rospy.loginfo(f"Stabilization complete: {duration}s {description}")

    # 已删除 convert_wrench_to_list 方法 - 直接使用轨迹生成器的numpy格式输出
    
    def generate_polynomial_trajectory(self, start_pos, target_pos, target_yaw, num_points=15, lock_yaw=False):
        """
        Generate smooth polynomial trajectory using PolynomialTrajectory class (Formation version)
        Shared method for all formation states
        
        Args:
            start_pos: Starting position [x, y, z]
            target_pos: Target position [x, y, z]
            target_yaw: Target yaw angle (radians)
            num_points: Number of trajectory points to generate
            lock_yaw: If True, all trajectory points use target_yaw instead of smooth transition
                     If False (default), yaw smoothly transitions from current_yaw to target_yaw
        """
        rospy.loginfo(f"=== Generating Formation Polynomial Trajectory: {num_points} points ===")
        if lock_yaw:
            rospy.loginfo(f"[Lock Yaw] All trajectory points will use target_yaw={math.degrees(target_yaw):.1f}°")
        
        # Get current state for trajectory planning
        current_yaw = self.get_end_effector_yaw()
        if current_yaw is None:
            current_yaw = 0.0
        
        # Calculate total distance and trajectory time
        total_distance = math.sqrt(sum((target_pos[i] - start_pos[i])**2 for i in range(3)))
        yaw_change = abs(FormationUtils.normalize_angle(target_yaw - current_yaw))
        
        # Calculate trajectory time based on formation parameters
        time_for_distance = total_distance / self.average_linear_velocity
        time_for_rotation = yaw_change / self.max_angular_velocity
        
        trajectory_duration = max(
            time_for_distance,
            time_for_rotation,
            8.0  # Minimum 8 seconds for smooth motion
        )
        
        # Apply safety factor
        safety_factor = 0.7
        trajectory_duration = trajectory_duration / safety_factor
        
        actual_avg_velocity = total_distance / trajectory_duration if trajectory_duration > 0 else 0
        
        rospy.loginfo(f"Trajectory params: distance={total_distance:.3f}m, angle={math.degrees(yaw_change):.1f}°")
        rospy.loginfo(f"Duration={trajectory_duration:.1f}s, avg_vel={actual_avg_velocity:.3f}m/s")
        
        # Create polynomial trajectories
        traj_x = PolynomialTrajectory(duration=trajectory_duration)
        traj_y = PolynomialTrajectory(duration=trajectory_duration)
        traj_z = PolynomialTrajectory(duration=trajectory_duration)
        traj_yaw = PolynomialTrajectory(duration=trajectory_duration)
        
        # Compute coefficients
        traj_x.is_scalar = True
        traj_x.coeffs_scalar = traj_x.compute_coefficients(start_pos[0], target_pos[0])
        traj_x.start_value = start_pos[0]
        traj_x.target_value = target_pos[0]
        
        traj_y.is_scalar = True
        traj_y.coeffs_scalar = traj_y.compute_coefficients(start_pos[1], target_pos[1])
        traj_y.start_value = start_pos[1]
        traj_y.target_value = target_pos[1]
        
        traj_z.is_scalar = True
        traj_z.coeffs_scalar = traj_z.compute_coefficients(start_pos[2], target_pos[2])
        traj_z.start_value = start_pos[2]
        traj_z.target_value = target_pos[2]
        
        # Yaw trajectory: lock or smooth transition based on lock_yaw parameter
        if lock_yaw:
            # Lock yaw: all points use target_yaw (no polynomial trajectory needed)
            traj_yaw = None
            rospy.loginfo(f"[Lock Yaw] Yaw fixed at {math.degrees(target_yaw):.1f}° for all points")
        else:
            # Smooth transition: polynomial trajectory from current_yaw to target_yaw
            traj_yaw = PolynomialTrajectory(duration=trajectory_duration)
            traj_yaw.is_scalar = True
            traj_yaw.coeffs_scalar = traj_yaw.compute_coefficients(current_yaw, target_yaw)
            traj_yaw.start_value = current_yaw
            traj_yaw.target_value = target_yaw
        
        trajectory_points = []
        
        # Generate trajectory points
        for i in range(num_points + 1):
            elapsed_time = (i / num_points) * trajectory_duration
            
            pos_x = traj_x.evaluate_at_time(elapsed_time)
            pos_y = traj_y.evaluate_at_time(elapsed_time)
            pos_z = traj_z.evaluate_at_time(elapsed_time)
            
            # Yaw: use target_yaw if locked, otherwise evaluate polynomial
            if lock_yaw:
                yaw = target_yaw
            else:
                yaw = traj_yaw.evaluate_at_time(elapsed_time)
            
            # Calculate velocity
            normalized_time = elapsed_time / trajectory_duration
            
            if normalized_time <= 0 or normalized_time >= 1:
                vel_x = vel_y = vel_z = 0.0
            else:
                T_vel = np.array([5*normalized_time**4, 4*normalized_time**3, 3*normalized_time**2, 
                                 2*normalized_time, 1, 0]) / trajectory_duration
                
                vel_x = np.dot(traj_x.coeffs_scalar, T_vel)
                vel_y = np.dot(traj_y.coeffs_scalar, T_vel)
                vel_z = np.dot(traj_z.coeffs_scalar, T_vel)
            
            # Apply trapezoidal velocity profile
            progress = normalized_time
            if progress < 0.2:
                velocity_factor = 0.6 + 0.4 * (progress / 0.2)
            elif progress > 0.8:
                velocity_factor = 0.6 + 0.4 * ((1.0 - progress) / 0.2)
            else:
                velocity_factor = 1.0
            
            vel_x *= velocity_factor
            vel_y *= velocity_factor
            vel_z *= velocity_factor
            
            pos = [pos_x, pos_y, pos_z]
            velocity = [vel_x, vel_y, vel_z]
            
            vel_magnitude = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
            if vel_magnitude > self.max_linear_velocity:
                scale_factor = self.max_linear_velocity / vel_magnitude
                velocity = [v * scale_factor for v in velocity]
                vel_magnitude = self.max_linear_velocity
            
            trajectory_points.append((pos, yaw, velocity, vel_magnitude))
        
        rospy.loginfo(f"Generated {len(trajectory_points)} trajectory points")
        return trajectory_points

    def execute_polynomial_trajectory(self, trajectory_points):
        """
        Execute polynomial trajectory using active convergence for each point
        Shared method for all formation states
        """
        rospy.loginfo(f"Executing Formation Polynomial Trajectory: {len(trajectory_points)} points")
        total_points = len(trajectory_points)
        
        for i, (pos, yaw, velocity, vel_magnitude) in enumerate(trajectory_points):
            is_final = (i >= total_points - 2)
            
            # Log progress
            if i % 3 == 0 or i == total_points - 1:
                mode = "precision" if is_final else "trajectory"
                rospy.loginfo(f"Point {i+1}/{total_points} ({mode}): pos={FormationUtils.format_vec(pos)}, yaw={math.degrees(yaw):.1f}°")
            
            # Converge to point with appropriate thresholds
            pos_thresh, yaw_thresh, timeout = (0.125, 0.08, 15.0) if is_final else (0.08, 0.15, 3.0)
            success = self.active_position_convergence(pos, yaw, pos_thresh, yaw_thresh, timeout)
            
            if not success:
                rospy.logwarn(f"Point {i+1} convergence issue - continuing")
            
            # Stabilize precision points
            if is_final:
                rospy.loginfo(f"Stabilizing point {i+1} for 2s")
                stabilize_end = rospy.Time.now().to_sec() + 2.0
                rate = rospy.Rate(10)
                while rospy.Time.now().to_sec() < stabilize_end and not rospy.is_shutdown():
                    self.send_assembly_command_from_end_effector(pos, yaw)
                    rate.sleep()
            
            # Dynamic pause
            rospy.sleep(0.2 if is_final else 0.067)
        
        rospy.loginfo("Polynomial trajectory execution completed")
        return True


class FormationInitializeStartPositionState(FormationSingleUAVStateBase):
    """Formation version of InitializeStartPositionState"""
    
    def __init__(self):
        FormationSingleUAVStateBase.__init__(self, 
            outcomes=['succeeded', 'failed'], 
            output_keys=['start_position', 'start_ee_position', 'start_yaw', 'valve_position', 'valve_yaw'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Formation Initialize Start Position State ===")
        
        # Wait for position data (reduced from 2.0s to 1.0s)
        rospy.sleep(1.0)
        
        # Get formation positions
        assembly_pos = self.get_assembly_position()
        assembly_yaw = self.get_assembly_yaw()
        ee_pos = self.get_end_effector_position()
        
        if assembly_pos is None:
            rospy.logerr("Assembly position not available")
            return 'failed'
        
        # Get valve position from leader UAV's sensors
        valve_pos = self.beetle.getValvePos()
        valve_yaw = self.beetle.getValveYaw()
        
        if valve_pos is None:
            rospy.logerr("Valve position not available")
            return 'failed'
        
        # Store positions in userdata
        userdata.start_position = assembly_pos
        userdata.start_ee_position = ee_pos  # Save end-effector position for disengage phase
        userdata.start_yaw = assembly_yaw  # Save start yaw for disengage phase
        userdata.valve_position = valve_pos
        userdata.valve_yaw = valve_yaw
        
        rospy.loginfo(f"Formation start position (assembly CoG): {assembly_pos}")
        rospy.loginfo(f"Formation start position (end-effector): {ee_pos}")
        rospy.loginfo(f"Formation start yaw: {math.degrees(assembly_yaw):.1f}°")
        rospy.loginfo(f"Valve position: {valve_pos}")
        rospy.loginfo(f"Valve yaw: {valve_yaw}")
        
        return 'succeeded'


class FormationMoveToValveState(FormationSingleUAVStateBase):
    # Shared optimizer strategy for later states (class variable)
    _shared_optimizer_strategy = None
    """Formation version of MoveToValveState with multi-phase control logic from single UAV version"""
    
    def __init__(self):
        FormationSingleUAVStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['valve_position', 'valve_yaw'],
            output_keys=['insertion_contact_pose', 'insertion_contact_yaw'])
            
        # Formation control parameters based on single UAV proven values
        self.max_linear_velocity = 0.12   # 0.12 m/s maximum speed (same as single)
        self.average_linear_velocity = 0.08  # 0.08 m/s average speed (slightly slower than single's 0.1 for formation stability)
        self.max_angular_velocity = 0.02  # 0.02 rad/s angular speed (same as single for stability)
        
        # Convergence thresholds (based on single UAV proven values)
        self.pos_threshold = 0.02         # 20mm position threshold (same as single)
        self.yaw_threshold = 0.05         # ~2.9° yaw threshold (same as single) 
        self.vel_threshold = 0.01         # 10mm/s velocity threshold (same as single)
        
        # Final positioning accuracy
        self.final_pos_threshold = 0.015  # 15mm final accuracy (slightly relaxed from single's 10mm)
        self.final_yaw_threshold = 0.0175 # 1° final yaw accuracy (same as single)
        
        # Active convergence parameters
        self.min_consecutive_readings = 5  # 5 consecutive good readings (same as single)
        
        # Formation-specific safety margins (more conservative than single)
        self.formation_safety_factor = 0.8  # 20% safety margin for formation coordination
    
    def execute(self, userdata):
        """Execute 4-phase movement using optimizer-driven targets (single UAV logic)"""
        rospy.loginfo("=== Formation Move-To-Valve (Optimizer Guided) ===")
        valve_pos = userdata.valve_position
        valve_yaw = userdata.valve_yaw

        current_ee_pos = self.get_end_effector_position()
        if current_ee_pos is None:
            rospy.logerr("Cannot get current end-effector position")
            return 'failed'

        current_yaw = self.get_end_effector_yaw() or 0.0

        # OPTIMIZER STRATEGY: Same-side insertion
        strategy = self.optimizer.calculate_same_side_insertion_strategy(
            valve_pos=valve_pos,
            valve_yaw=valve_yaw,
            uav_pos=current_ee_pos
        )
        if not strategy or not strategy.get('feasible', True):
            rospy.logerr("Optimizer failed to provide feasible insertion strategy")
            return 'failed'

        # Cache for later states (descend/contact, etc.)
        FormationMoveToValveState._shared_optimizer_strategy = strategy

        safe_ee_pos = strategy.get('safe_end_effector_center')
        if safe_ee_pos is None:
            fallback_pos = strategy.get('safe_uav_position')
            if fallback_pos is not None:
                rospy.logwarn_once("Optimizer strategy missing safe_end_effector_center; using safe_uav_position as fallback")
                safe_ee_pos = fallback_pos
            else:
                rospy.logwarn_once("Optimizer strategy missing end-effector target; using current EE pose")
                safe_ee_pos = current_ee_pos
        safe_ee_yaw = strategy['safe_uav_yaw'] if 'safe_uav_yaw' in strategy else valve_yaw
        safe_ee_pos = tuple(safe_ee_pos)

        # Optional Z offset compensation (disabled by default)
        formation_z_offset = rospy.get_param("~formation_z_descent_offset", 0.0)
        if abs(formation_z_offset) > 1e-4:
            compensated_z = safe_ee_pos[2] + formation_z_offset
            rospy.loginfo(
                f"[Z_DESCENT] Apply Z compensation {formation_z_offset*1000:.0f}mm -> {compensated_z:.3f}m"
            )
            safe_ee_pos = (safe_ee_pos[0], safe_ee_pos[1], compensated_z)
        else:
            rospy.loginfo(f"[Z_DESCENT] Using optimizer target: {safe_ee_pos[2]:.3f}m (valve height)")
        
        # Validate target height matches valve height
        expected_valve_height = valve_pos[2]
        actual_target_height = safe_ee_pos[2]
        height_difference = abs(actual_target_height - expected_valve_height)
        if height_difference > 0.01:
            rospy.logwarn(f"Z_DESCENT target {actual_target_height:.3f}m differs from valve {expected_valve_height:.3f}m by {height_difference*1000:.1f}mm")
        else:
            rospy.loginfo(f"Z_DESCENT target {actual_target_height:.3f}m matches valve {expected_valve_height:.3f}m (diff: {height_difference*1000:.1f}mm)")
        
        FormationSingleUAVStateBase._shared_target_z = safe_ee_pos[2]
        rospy.loginfo(f"Current: {FormationUtils.format_vec(current_ee_pos)}, yaw={math.degrees(current_yaw):.1f}°")
        rospy.loginfo(f"Target: {FormationUtils.format_vec(safe_ee_pos)}, yaw={math.degrees(safe_ee_yaw):.1f}°")

        # XY_APPROACH: Move to optimizer XY position
        current_ee_pos = self.get_end_effector_position()
        if current_ee_pos is None:
            rospy.logerr("[XY_APPROACH] Cannot get position")
            return 'failed'
        
        current_yaw = self.get_end_effector_yaw() or 0.0
        xy_target = (safe_ee_pos[0], safe_ee_pos[1], current_ee_pos[2])
        rospy.loginfo(f"[XY_APPROACH] Target: {FormationUtils.format_vec(xy_target)}")
        if not self._execute_xy_approach(xy_target, current_yaw):
            rospy.logerr("[XY_APPROACH] Failed")
            return 'failed'

        # XY_PRECISION: Fine-tune XY position
        current_ee_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        if current_ee_pos is None or current_yaw is None:
            rospy.logerr("[XY_PRECISION] Cannot get position/yaw")
            return 'failed'

        precision_target = (safe_ee_pos[0], safe_ee_pos[1], current_ee_pos[2])
        rospy.loginfo(f"[XY_PRECISION] Lock Z={precision_target[2]:.3f}m")
        if not self._execute_xy_precision(precision_target, current_yaw):
            rospy.logerr("[XY_PRECISION] Failed")
            return 'failed'

        # YAW_ALIGN: Align yaw to spoke direction
        current_ee_pos = self.get_end_effector_position()
        if current_ee_pos is None:
            rospy.logerr("[YAW_ALIGN] Cannot get position")
            return 'failed'
            
        optimal_spoke_yaw = self.calculate_shortest_yaw_path(current_yaw, safe_ee_yaw)
        if not self._execute_yaw_align(current_ee_pos, optimal_spoke_yaw, precision_target):
            rospy.logerr("[YAW_ALIGN] Failed")
            return 'failed'

        # Brief stabilization before Z descent (increased to 5s for better stability after YAW alignment)
        # Use XY from optimizer target but Z from current position to prevent unwanted descent
        current_pos_for_stab = self.get_end_effector_position()
        stab_yaw = self.get_end_effector_yaw() or optimal_spoke_yaw
        # CRITICAL: Lock current Z during stabilization, only stabilize XY toward target
        stab_pos = (safe_ee_pos[0], safe_ee_pos[1], current_pos_for_stab[2])
        rospy.loginfo(f"[STABILIZE] 5s stabilization before Z descent (Z locked at {current_pos_for_stab[2]:.3f}m)...")
        self.active_stabilization_wait(stab_pos, stab_yaw, 5.0, "Pre-Z-descent stabilization")

        # Z_DESCENT: Descend to insertion height
        final_target = (safe_ee_pos[0], safe_ee_pos[1], safe_ee_pos[2])
        final_yaw = self.get_end_effector_yaw() or safe_ee_yaw
        
        # Log Z target transition for debugging oscillation issues
        current_pos_before_descent = self.get_end_effector_position()
        if current_pos_before_descent:
            z_jump = current_pos_before_descent[2] - final_target[2]
            rospy.loginfo(f"[Z_TARGET_TRANSITION] STABILIZE Z={stab_pos[2]:.3f}m → Z_DESCENT target Z={final_target[2]:.3f}m")
            rospy.loginfo(f"[Z_TARGET_TRANSITION] Current EE Z={current_pos_before_descent[2]:.3f}m, total descent={z_jump*1000:.1f}mm")

        if not self._execute_z_descent(final_target, final_yaw):
            rospy.logerr("[Z_DESCENT] Failed")
            return 'failed'
        rospy.loginfo("[Z_DESCENT] Complete")

        # CRITICAL FIX: Use valve height (final_target) for stabilization, NOT contact position
        # This ensures consistent Z throughout insertion and rotation phases
        insertion_pose = final_target  # Use valve height directly
        userdata.insertion_contact_pose = insertion_pose
        userdata.insertion_contact_yaw = self.get_end_effector_yaw() or final_yaw

        # Post-insertion stabilization at valve height
        rospy.loginfo(f"1.5s stabilization at valve height {FormationUtils.format_vec(insertion_pose)}")
        self.active_stabilization_wait(insertion_pose, final_yaw, 1.5, "Post-insertion")

        rospy.loginfo("=== Formation move-to-valve complete ===")
        return 'succeeded'
        
    def calculate_shortest_yaw_path(self, current_yaw, target_yaw):
        """Calculate the shortest yaw rotation path to avoid reverse direction rotation (from single UAV)"""
        # Calculate the difference
        diff = target_yaw - current_yaw
        
        # Handle angle wrap-around to ensure shortest path
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
        
        # Return the shortest path target
        optimal_target = current_yaw + diff
        rospy.loginfo(f"[YawPath] {math.degrees(current_yaw):.1f}°→{math.degrees(target_yaw):.1f}° (Δ{math.degrees(diff):.1f}°)")
        return optimal_target
    
    def _check_positioning_error(self, target_pos, target_yaw=None, pos_thresh=0.050, yaw_thresh=None):
        """Check current position/yaw error against target. Returns (pos_ok, yaw_ok, xy_err, z_err, yaw_err)"""
        current_pos = self.get_end_effector_position()
        if not current_pos:
            return False, False, None, None, None
        
        xy_error = math.sqrt((current_pos[0] - target_pos[0])**2 + (current_pos[1] - target_pos[1])**2)
        z_error = abs(current_pos[2] - target_pos[2])
        pos_ok = (xy_error <= pos_thresh and z_error <= 0.015)
        
        yaw_ok = True
        yaw_error = None
        if target_yaw is not None and yaw_thresh is not None:
            current_yaw = self.get_end_effector_yaw()
            if current_yaw is not None:
                yaw_error = abs(FormationUtils.normalize_angle(target_yaw - current_yaw))
                yaw_ok = (yaw_error <= yaw_thresh)
        
        return pos_ok, yaw_ok, xy_error, z_error, yaw_error
    
    def _execute_xy_positioning(self, target_pos, target_yaw, phase_name="XY", 
                                use_trajectory=True, pos_thresh=0.050, yaw_thresh=0.087, 
                                timeout=15.0, num_traj_points=12, adjustment_pos_thresh=None):
        """Generic XY positioning method used by Phase 2, 3A, 3B"""
        rospy.loginfo(f"[{phase_name}] Target: {FormationUtils.format_vec(target_pos)}, yaw={math.degrees(target_yaw):.1f}°")
        
        current_pos = self.get_end_effector_position()
        if current_pos is None:
            rospy.logerr(f"[{phase_name}] Cannot get current position")
            return False
        
        xy_distance = math.sqrt((target_pos[0] - current_pos[0])**2 + (target_pos[1] - current_pos[1])**2)
        rospy.loginfo(f"[{phase_name}] XY distance: {xy_distance*1000:.1f}mm")
        
        # Direct convergence for small movements
        if xy_distance < 0.01 or not use_trajectory:
            rospy.loginfo(f"[{phase_name}] Using direct convergence")
            return self.active_position_convergence(
                target_pos=target_pos, target_yaw=target_yaw,
                pos_thresh=pos_thresh, yaw_thresh=yaw_thresh, timeout=timeout,
                adjustment_pos_thresh=adjustment_pos_thresh,
                decomposition_threshold=0.04  # Force 40mm threshold for XY motion
            )
        
        # Trajectory-based movement for larger distances
        rospy.loginfo(f"[{phase_name}] Using polynomial trajectory ({num_traj_points} points)")
        trajectory_points = self.generate_polynomial_trajectory(
            start_pos=current_pos, target_pos=target_pos,
            target_yaw=target_yaw, num_points=num_traj_points
        )
        
        if not trajectory_points:
            rospy.logwarn(f"[{phase_name}] Trajectory generation failed, using direct convergence")
            return self.active_position_convergence(
                target_pos=target_pos, target_yaw=target_yaw,
                pos_thresh=pos_thresh, yaw_thresh=yaw_thresh, timeout=timeout * 1.5,
                adjustment_pos_thresh=adjustment_pos_thresh
            )
        
        # Execute trajectory
        self.execute_polynomial_trajectory(trajectory_points)
        
        # Final convergence check
        rospy.loginfo(f"[{phase_name}] Final convergence check")
        final_success = self.active_position_convergence(
            target_pos=target_pos, target_yaw=target_yaw,
            pos_thresh=pos_thresh, yaw_thresh=yaw_thresh, timeout=timeout,
            adjustment_pos_thresh=adjustment_pos_thresh,
            decomposition_threshold=0.04  # Force 40mm threshold for XY motion
        )
        
        # Stabilization for final position
        rospy.loginfo(f"[{phase_name}] Stabilizing for 2s")
        stabilize_end = rospy.Time.now().to_sec() + 2.0
        rate = rospy.Rate(10)
        while rospy.Time.now().to_sec() < stabilize_end and not rospy.is_shutdown():
            self.send_assembly_command_from_end_effector(target_pos, target_yaw)
            rate.sleep()
        
        return final_success
        
    def _execute_xy_approach(self, xy_target_ee_pos, maintain_yaw):
        """XY_APPROACH: Move to optimizer XY position"""
        return self._execute_xy_positioning(
            target_pos=xy_target_ee_pos, target_yaw=maintain_yaw,
            phase_name="XY_APPROACH", use_trajectory=True,
            pos_thresh=0.050, yaw_thresh=0.087, timeout=15.0, num_traj_points=12
        )
    
    def _calculate_spoke_alignment_yaw(self, current_ee_pos, valve_pos):
        """
        Calculate the yaw angle needed to align with the selected beam/spoke.
        Based on single UAV logic for spoke gap alignment.
        
        Args:
            current_ee_pos: Current end-effector position
            valve_pos: Valve center position
            
        Returns:
            float: Spoke alignment yaw angle in radians
        """
        # Calculate spoke angle (which beam was selected) - same as single UAV
        valve_x, valve_y = valve_pos[0], valve_pos[1]
        spoke_angle = math.atan2(
            current_ee_pos[1] - valve_y,
            current_ee_pos[0] - valve_x
        )
        
        # For insertion, UAV should align with the spoke direction
        # The yaw should point from valve center toward the current position (radial alignment)
        spoke_yaw = spoke_angle
        
        rospy.loginfo(f"Spoke alignment calculation:")
        rospy.loginfo(f"Valve center: ({valve_x:.3f}, {valve_y:.3f})")
        rospy.loginfo(f"Current EE pos: ({current_ee_pos[0]:.3f}, {current_ee_pos[1]:.3f})")  
        rospy.loginfo(f"Spoke angle: {math.degrees(spoke_angle):.1f}°")
        rospy.loginfo(f"Required spoke yaw: {math.degrees(spoke_yaw):.1f}°")
        
        return spoke_yaw
    
    def _execute_xy_precision(self, target_ee_pos, maintain_yaw):
        """XY_PRECISION: XY precision positioning with Z locked"""
        current_state = self.get_end_effector_position()
        if current_state is None:
            rospy.logerr("XY_PRECISION: Cannot get current position")
            return False

        locked_z = target_ee_pos[2] if target_ee_pos[2] else current_state[2]
        xy_target = (target_ee_pos[0], target_ee_pos[1], locked_z)
        
        # Multiple attempts with verification
        for attempt in range(1, 4):
            rospy.loginfo(f"[XY_PRECISION] Attempt {attempt}/3")
            success = self._execute_xy_positioning(
                target_pos=xy_target, target_yaw=maintain_yaw,
                phase_name=f"XY_PRECISION-{attempt}", use_trajectory=False,
                pos_thresh=0.050, yaw_thresh=0.087, timeout=5.0
            )
            
            if success:
                pos_ok, _, xy_error, z_error, _ = self._check_positioning_error(
                    (xy_target[0], xy_target[1], locked_z), pos_thresh=0.050
                )
                if pos_ok:
                    rospy.loginfo(f"[XY_PRECISION] Successful: XY={xy_error*1000:.1f}mm, Z={z_error*1000:.1f}mm")
                    return True
        
        rospy.logerr("[XY_PRECISION] Failed after 3 attempts")
        return False

    def _execute_yaw_align(self, current_ee_pos, spoke_yaw, target_pos=None):
        """YAW_ALIGN: Spoke yaw alignment with XY/Z locked"""
        reference_xy = (target_pos[0], target_pos[1]) if target_pos else (current_ee_pos[0], current_ee_pos[1])
        reference_z = target_pos[2] if target_pos else current_ee_pos[2]
        locked_pos = (reference_xy[0], reference_xy[1], reference_z)
        
        for attempt in range(1, 4):
            rospy.loginfo(f"[YAW_ALIGN] Attempt {attempt}/3 - Yaw to {math.degrees(spoke_yaw):.1f}°")
            
            success = self.active_position_convergence(
                target_pos=locked_pos, target_yaw=spoke_yaw,
                pos_thresh=0.050, yaw_thresh=0.087, timeout=35.0,
                max_yaw_step=math.radians(3.0)
            )
            
            if success:
                pos_ok, yaw_ok, xy_error, z_error, yaw_error = self._check_positioning_error(
                    locked_pos, spoke_yaw, pos_thresh=0.050, yaw_thresh=0.087
                )
                if pos_ok and yaw_ok:
                    rospy.loginfo(f"[YAW_ALIGN] Successful: XY={xy_error*1000:.1f}mm, yaw={math.degrees(yaw_error):.1f}°")
                    return True
                
                if not pos_ok:
                    rospy.logwarn(f"[YAW_ALIGN] Position drift, correcting...")
                    final_yaw = self.get_end_effector_yaw()
                    self._execute_xy_precision(locked_pos, final_yaw)
        
        rospy.logerr("[YAW_ALIGN] Failed after 3 attempts")
        return False
        
    def _execute_z_descent(self, final_ee_pos, final_yaw):
        """Z_DESCENT: Pure Z descent to insertion height"""
        rospy.loginfo(f"[Z_DESCENT] Target: {FormationUtils.format_vec(final_ee_pos)}, yaw={math.degrees(final_yaw):.1f}°")
        self._last_z_descent_contact_position = None

        current_pos = self.get_end_effector_position()
        if current_pos is None:
            rospy.logerr("[Z_DESCENT] Cannot get position")
            return False

        descent_success, achieved_pos = self.controlled_z_descent(current_pos, final_ee_pos, final_yaw)
        if not descent_success:
            return False

        if achieved_pos is None:
            achieved_pos = self.get_end_effector_position() or final_ee_pos

        self._last_z_descent_contact_position = achieved_pos
        
        # CRITICAL FIX: Use target height (valve height), NOT achieved height
        # achieved_pos may be higher due to early contact detection
        # Use final_ee_pos[2] which is the optimizer's target (valve height)
        FormationSingleUAVStateBase._shared_target_z = final_ee_pos[2]
        rospy.loginfo(f"[Z_DESCENT] Saved target Z={final_ee_pos[2]:.3f}m (valve height), achieved Z={achieved_pos[2]:.3f}m (contact position)")

        # CRITICAL FIX 2: Use valve height for convergence, NOT contact position
        # This ensures UAV converges to correct insertion depth, not where contact was detected
        final_convergence_target = (final_ee_pos[0], final_ee_pos[1], final_ee_pos[2])
        rospy.loginfo(f"[Z_DESCENT] Converging to valve height: {FormationUtils.format_vec(final_convergence_target)}")
        
        final_success = self.active_position_convergence(
            target_pos=final_convergence_target,  # Use valve height, NOT achieved_pos
            target_yaw=final_yaw,
            pos_thresh=0.080,
            yaw_thresh=0.087,
            timeout=12.0,
            adjustment_pos_thresh=0.050,
            decomposition_threshold=0.08  # Force 80mm for Z convergence
        )

        if not final_success:
            rospy.logwarn("[Z_DESCENT] Convergence incomplete")

        return final_success
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class FormationRotateValveState(FormationSingleUAVStateBase):
    """Formation valve contact and rotation - single unified state like Single UAV"""

    def __init__(self, rotation_direction=1, target_rotation=None):
        FormationSingleUAVStateBase.__init__(
            self,
            outcomes=['succeeded', 'failed', 'emergency'],
            input_keys=['valve_position', 'valve_yaw', 'insertion_contact_pose', 'insertion_contact_yaw'],
            output_keys=['trajectory_state', 'contact_final_torque']
        )
        self.rotation_direction = rotation_direction
        self.target_rotation = target_rotation if target_rotation is not None else math.radians(90.0)
        self.nominal_angular_velocity = 0.1  # rad/s = 5.7°/s
        # Dynamic timeout: base time + time for rotation + buffer
        # Formula: (target_angle / angular_velocity) * safety_factor + contact_time
        estimated_rotation_time = abs(self.target_rotation) / self.nominal_angular_velocity
        self.max_rotation_time = max(60.0, estimated_rotation_time * 1.5 + 20.0)  # 1.5x safety + 20s for contact
        self.min_rotation_time = 8.0
        self.control_rate = 25.0
        self.max_linear_speed = 0.035

    def execute(self, userdata):
        rospy.loginfo("=== Formation Contact and Rotate Valve State (Unified) ===")
        rospy.loginfo(f"Target rotation: {math.degrees(self.target_rotation):.1f}° ({self.target_rotation:.3f} rad)")
        rospy.loginfo(f"Timeout: {self.max_rotation_time:.1f}s (dynamic based on target angle)")
        
        rospy.loginfo(">>> CONTACT: Establishing circular contact with valve")
        contact_result = self._execute_contact_phase(userdata)
        if contact_result != 'succeeded':
            return contact_result
            
        rospy.loginfo(">>> ROTATION: Executing valve rotation")
        rotation_result = self._execute_rotation_phase(userdata)
        return rotation_result
    
    def _execute_contact_phase(self, userdata):
        """Contact establishment phase - replicate FormationDescendAndContactState logic"""
        rospy.loginfo("Using TWO-PHASE SAFE INSERTION strategy")
        
        valve_pos = userdata.valve_position
        valve_yaw = userdata.valve_yaw
        insertion_contact_pos = getattr(userdata, 'insertion_contact_pose', None)
        insertion_contact_yaw = getattr(userdata, 'insertion_contact_yaw', None)

        # Cache valve info for formation helpers
        self.optimizer.update_valve_info(valve_pos, valve_yaw)

        # Get position info after insertion complete (fixed Z coordinate)
        current_ee_pos = self.get_end_effector_position()
        if current_ee_pos is None:
            rospy.logerr("Failed to get current end-effector position")
            return 'failed'

        rospy.loginfo(f"Current EE pos: {FormationUtils.format_vec(current_ee_pos)}")
        rospy.loginfo("SKIPPING: Insertion already completed by FormationApproachValveState")
        
        # Circular Contact Establishment  
        current_assembly_pos = self.get_assembly_position()
        if current_assembly_pos is None:
            rospy.logerr("Cannot get Assembly CoG position")
            return 'failed'
            
        rospy.loginfo(f"Assembly CoG: {FormationUtils.format_vec(current_assembly_pos)}, EE: {FormationUtils.format_vec(current_ee_pos)}")
        contact_result = self._execute_circular_contact_phase(userdata, current_assembly_pos, valve_pos, valve_yaw)
        if not contact_result:
            return 'failed'

        # Position verification and insertion data setup
        valve_center = valve_pos
        # Use end-effector position for geometric calculations (but control target is Assembly CoG)
        if current_ee_pos is None:
            rospy.logerr("Cannot get end-effector position for geometric calculation")
            return 'failed'
            
        approach_angle = math.atan2(current_ee_pos[1] - valve_center[1], 
                                   current_ee_pos[0] - valve_center[0])
        spoke_angles = self.optimizer.get_spoke_angles()
        angle_diffs = [abs(angle - approach_angle) for angle in spoke_angles]
        selected_spoke = spoke_angles[angle_diffs.index(min(angle_diffs))]
        
        rospy.loginfo(f"Spoke selection: approach={math.degrees(approach_angle):.1f}°, selected={math.degrees(selected_spoke):.1f}°")

        ee_to_valve_distance = math.sqrt((current_ee_pos[0] - valve_pos[0])**2 + 
                                 (current_ee_pos[1] - valve_pos[1])**2)
        rospy.loginfo(f"Current distance to valve center: {ee_to_valve_distance*1000:.1f}mm")

        if ee_to_valve_distance > 0.025:
            rospy.logwarn(f"Distance to valve center too large: {ee_to_valve_distance*1000:.1f}mm")

        # Note: self.contact_radius is already set in _execute_circular_contact_phase with trajectory radius
        # Do NOT overwrite it here with ee_to_valve_distance
        self.contact_angle = selected_spoke
        self.valve_center = valve_pos
        self.initial_valve_yaw = valve_yaw
        self.rotation_start_position = current_assembly_pos

        rospy.loginfo("Contact phase complete - ready for rotation")
        return 'succeeded'

    def _execute_circular_contact_phase(self, userdata, current_pos, valve_pos, initial_valve_yaw):
        """Circular contact establishment - adapted from single UAV"""
        rospy.loginfo(">>> Starting circular contact establishment")
        
        # Calculate current radius
        relative_pos = np.array(current_pos[:2]) - np.array(valve_pos[:2])
        current_radius = np.linalg.norm(relative_pos)
        rospy.loginfo(f"Valve: {FormationUtils.format_vec(valve_pos)}, radius={current_radius*1000:.1f}mm, yaw={math.degrees(initial_valve_yaw):.1f}°")
        
        # Use rotation_direction to control angular velocity sign
        contact_angular_velocity = self.rotation_direction * 0.1  # 0.1 rad/s ≈ 5.7°/s
        
        # Correct valve Z to insertion depth
        if FormationSingleUAVStateBase._shared_target_z is not None:
            valve_pos = [valve_pos[0], valve_pos[1], FormationSingleUAVStateBase._shared_target_z]
            rospy.loginfo(f"Valve Z corrected to {FormationSingleUAVStateBase._shared_target_z:.3f}m")
        else:
            rospy.logwarn(f"Using original valve Z={valve_pos[2]:.3f}m")
        
        # Use Assembly CoG as trajectory basis
        current_assembly_pos = self.get_assembly_position()
        if current_assembly_pos is None:
            rospy.logerr("Cannot get Assembly CoG position")
            return False
        
        # Initialize trajectory generator
        trajectory_generator = OnlineCircularTrajectoryGenerator(
            valve_center=valve_pos,
            initial_radius=current_radius,
            target_angular_velocity=contact_angular_velocity,
            control_rate=self.control_rate
        )
        trajectory_generator.initialize_from_current_position(current_assembly_pos)
        
        contact_threshold = 0.1
        rospy.loginfo(f"Target: {math.degrees(contact_threshold):.1f}° valve rotation, timeout=20s")
        
        max_contact_time = 20.0
        target_valve_rotation = contact_threshold
        min_valve_rotation = math.radians(1.0)
        contact_start_time = rospy.Time.now().to_sec()
        
        start_valve_yaw = FormationUtils.get_valve_yaw_safe(self.beetle, initial_valve_yaw)
        last_valve_yaw = start_valve_yaw
        last_valve_check_time = contact_start_time
        max_detected_rotation = 0.0
        
        # Main contact establishment loop
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            contact_duration = current_time - contact_start_time
            
            current_valve_yaw = FormationUtils.get_valve_yaw_safe(self.beetle, start_valve_yaw)
            valve_rotation, valve_angular_velocity, updated, last_valve_yaw, last_valve_check_time = \
                FormationUtils.monitor_valve_rotation(
                    current_valve_yaw, start_valve_yaw, last_valve_yaw, 
                    last_valve_check_time, current_time, update_interval=0.2
                )
            max_detected_rotation = max(max_detected_rotation, valve_rotation)
            
            current_assembly_pos_loop = self.get_assembly_position()
            current_yaw = self.get_end_effector_yaw()
            current_ee_pos_loop = self.get_end_effector_position()
            
            if current_assembly_pos_loop is None or current_yaw is None:
                rospy.logwarn("Formation pose information lost, continuing to wait")
                rospy.sleep(0.04)
                continue
                
            # Use Assembly CoG position as trajectory update input
            current_pos = current_assembly_pos_loop
            
            # Check for successful contact establishment
            if valve_rotation >= target_valve_rotation:
                rospy.loginfo(f"Valve engagement detected: {math.degrees(valve_rotation):.1f}° "
                             f"(threshold: {math.degrees(target_valve_rotation):.1f}°)")
                rospy.loginfo(f"Contact establishment time: {contact_duration:.1f}s")
                
                # Safety check: warn if contact too fast
                if contact_duration < 1.0:
                    rospy.logwarn(f"Contact established too quickly ({contact_duration:.1f}s), may need parameter adjustment")
                
                # Store trajectory generator for rotation phase use
                self.trajectory_generator = trajectory_generator  # Save instance
                
                # CRITICAL FIX: Store the actual trajectory radius from contact phase
                # This radius is used for reverse rotation during disengagement
                final_state = trajectory_generator.update_state(current_pos, current_yaw, valve_angular_velocity)
                self.contact_radius = final_state.get('current_radius', 0.8)  # Use trajectory radius, not EE-to-valve distance
                rospy.loginfo(f"Contact radius stored for disengage: {self.contact_radius*1000:.1f}mm")
                
                rospy.loginfo("Trajectory generator state transferred to rotation phase")
                
                # Store contact success information
                contact_final_torque = [0.0, 0.0, -0.08]  # Fixed baseline torque
                rospy.loginfo(f"Final contact torque (aligned): {contact_final_torque}")
                
                # Set rotation baseline
                rospy.loginfo(f"Rotation baseline set: initial={math.degrees(initial_valve_yaw):.1f}°, "
                             f"current={math.degrees(current_valve_yaw):.1f}°, achieved={math.degrees(valve_rotation):.1f}°")
                
                return True
            
            # Timeout check
            if contact_duration > max_contact_time:
                break
                
            # Enhanced trajectory monitoring (like Single UAV)
            if contact_duration > 1.0:  # Start logging after 1 second
                rospy.loginfo_throttle(2.0, 
                    f"Formation contact: radius={state_info['current_radius']*1000:.1f}mm, "
                    f"angular_vel={math.degrees(state_info['angular_velocity']):.1f}°/s, "
                    f"valve_rotation={math.degrees(valve_rotation):.2f}°, "
                    f"time={contact_duration:.1f}s")
            
            # Update trajectory generator state (Critical: like Single UAV)
            state_info = trajectory_generator.update_state(
                current_pos, current_yaw, valve_angular_velocity
            )
            
            # Execute trajectory  
            target_state = trajectory_generator.generate_target_state()
            if target_state:
                # Use same SE(3) control method as single UAV
                self.beetle.executeTrajectoryWithWrench(
                    pos=target_state['position'],
                    rot=target_state['yaw'],
                    linear_vel=target_state['linear_velocity'],
                    angular_vel=target_state['angular_velocity'],
                    force=target_state['force'],
                    torque=target_state['torque']
                )
            
            rospy.sleep(0.04)  # 25Hz control rate
        
        # Contact establishment failed
        rospy.logerr(f"Contact establishment failed: max valve movement {math.degrees(max_detected_rotation):.2f}°")
        return False
    
    def _execute_rotation_phase(self, userdata):
        """Rotation execution phase - reuse trajectory generator instance from contact phase"""
        
        # Use trajectory generator instance stored from contact phase
        if not hasattr(self, 'trajectory_generator') or self.trajectory_generator is None:
            rospy.logerr("No trajectory generator available from contact phase!")
            return 'failed'
            
        rospy.loginfo("Reusing trajectory generator from contact phase")
        
        # CRITICAL FIX: Correct valve_center Z coordinate in rotation phase, use saved insertion depth to avoid Z jumps
        current_pos = self.get_end_effector_position()
        if current_pos is not None:
            # Use saved insertion depth to avoid Z coordinate jumps in Contact→Rotation transition
            insertion_depth_z = FormationSingleUAVStateBase._shared_target_z or current_pos[2]
            original_valve_center = self.valve_center
            corrected_valve_center = [original_valve_center[0], original_valve_center[1], insertion_depth_z]
            
            # Update trajectory generator valve_center
            if hasattr(self.trajectory_generator, 'valve_center'):
                self.trajectory_generator.valve_center = np.array(corrected_valve_center)
                self.valve_center = corrected_valve_center
                z_source = "saved_target_z" if FormationSingleUAVStateBase._shared_target_z else "current_pos"
                rospy.loginfo(f"Rotation phase Z correction: ({original_valve_center[2]:.3f}) → ({insertion_depth_z:.3f}) [source: {z_source}]")
        
        # Get current state information
        current_valve_yaw_now = FormationUtils.get_valve_yaw_safe(self.beetle, self.initial_valve_yaw)
        
        rospy.loginfo(f"Starting rotation from baseline, contact achieved: {math.degrees(abs(current_valve_yaw_now - self.initial_valve_yaw)):.1f}°")
        rospy.loginfo(f"Rotation start position: {self.rotation_start_position}")
        rospy.loginfo(f"Valve center: {self.valve_center}")
        rospy.loginfo(f"Initial rotation radius: {self.contact_radius*1000:.1f}mm")
        rospy.loginfo(f"Rotation directions consistent: {self.rotation_direction}")
        
        # SINGLE UAV COMPATIBILITY: Reuse Contact phase trajectory generator for continuity
        effective_angular_velocity = abs(self.nominal_angular_velocity)
        rospy.loginfo(f">>> ROTATION: {math.degrees(self.target_rotation):.1f}° @ {math.degrees(effective_angular_velocity):.1f}°/s")
        
        # Rotation control loop - use continuous tracking for >180° rotations
        rotation_start_time = rospy.Time.now().to_sec()
        initial_valve_yaw = self.initial_valve_yaw
        last_valve_yaw = current_valve_yaw_now
        last_valve_check_time = rotation_start_time
        
        # For continuous rotation tracking (handles >180° without discontinuity)
        accumulated_rotation = abs(current_valve_yaw_now - initial_valve_yaw)  # Start with contact-achieved rotation
        max_rotation_detected = accumulated_rotation
        
        # Determine if we need continuous tracking (target > 170°)
        use_continuous_tracking = self.target_rotation > math.radians(170.0)
        if use_continuous_tracking:
            rospy.loginfo(f"Using continuous rotation tracking for target > 170° ({math.degrees(self.target_rotation):.1f}°)")
        
        # Valve stuck detection variables (from legacy version)
        valve_motion_stalled_time = 0.0
        valve_stuck_threshold = 6.0  # 6-second detection threshold for Formation system
        last_loop_time = rotation_start_time
        
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - rotation_start_time
            current_time = rospy.Time.now().to_sec()
            dt = current_time - last_loop_time
            last_loop_time = current_time
            
            current_valve_yaw = FormationUtils.get_valve_yaw_safe(self.beetle, initial_valve_yaw)
            
            if use_continuous_tracking:
                # Use continuous tracking for large rotations
                accumulated_rotation, valve_angular_velocity, updated, last_valve_yaw, last_valve_check_time = \
                    FormationUtils.monitor_valve_rotation_continuous(
                        current_valve_yaw, last_valve_yaw, accumulated_rotation,
                        last_valve_check_time, current_time, update_interval=0.2
                    )
                valve_rotation = accumulated_rotation
            else:
                # Use legacy method for small rotations
                valve_rotation, valve_angular_velocity, updated, last_valve_yaw, last_valve_check_time = \
                    FormationUtils.monitor_valve_rotation(
                        current_valve_yaw, initial_valve_yaw, last_valve_yaw,
                        last_valve_check_time, current_time, update_interval=0.2
                    )
            
            max_rotation_detected = max(max_rotation_detected, valve_rotation)

            # Periodic status logging
            rospy.loginfo_throttle(5.0, f"Rotation: {math.degrees(valve_rotation):.1f}°/{math.degrees(self.target_rotation):.1f}°, "
                                   f"ω={math.degrees(valve_angular_velocity):.2f}°/s")
            
            # Valve stuck detection (from legacy version)
            if elapsed > 5.0:  # Only start detecting after 5 seconds
                if valve_angular_velocity < 0.008:  # < 0.5°/s considered stalled
                    valve_motion_stalled_time += dt
                else:
                    valve_motion_stalled_time = 0.0
            
            # Check for valve stuck condition
            if valve_motion_stalled_time > valve_stuck_threshold:
                rospy.logerr("Valve motion stall detected!")
                rospy.logerr(f"  Stall time: {valve_motion_stalled_time:.1f}s > threshold {valve_stuck_threshold:.1f}s")
                rospy.logerr(f"  Achieved rotation: {math.degrees(max_rotation_detected):.1f}°")
                
                if max_rotation_detected > self.target_rotation * 0.7:  # 70% considered partial success
                    rospy.loginfo(f"Partial success: rotated {math.degrees(max_rotation_detected):.1f}°, continuing attempt")
                    valve_motion_stalled_time = 0.0  # Reset timer
                else:
                    rospy.logerr("Valve may be stuck or resistance too high, rotation failed!")
                    return 'failed'
            
            # Check rotation complete
            if valve_rotation >= self.target_rotation:
                rospy.loginfo(f"Valve rotation completed: {math.degrees(valve_rotation):.1f}° "
                             f"(target: {math.degrees(self.target_rotation):.1f}°) in {elapsed:.1f}s")
                
                # Restore trajectory_state save for Disengage phase use
                userdata.trajectory_state = {
                    'current_radius': self.contact_radius,
                    'current_angle': getattr(self, 'contact_angle', 0),
                    'radius_locked': True,
                    'valve_center': self.valve_center,
                    'rotation_direction': self.rotation_direction
                }
                userdata.contact_final_torque = [0.0, 0.0, 0.1]
                
                return 'succeeded'
            
            # 超时检查
            if elapsed > self.max_rotation_time:
                rospy.logwarn(f"Rotation timeout after {elapsed:.1f}s, achieved: {math.degrees(valve_rotation):.1f}°")
                break
            
            # Execute trajectory commands - update state first then generate target  
            # Solution A fix: Use real position for state update, consistent with single UAV version
            current_pos = self.get_end_effector_position()[:3]
            current_yaw = self.get_end_effector_yaw() or 0.0
            
            self.trajectory_generator.update_state(current_pos, current_yaw, valve_angular_velocity)
            target_state = self.trajectory_generator.generate_target_state()
            
            # Solution A monitoring: Rotation phase trajectory output check
            if target_state and elapsed > 1.0:  # Avoid initial noise
                target_pos = target_state.get('position', [0, 0, 0])
                pos_diff = [(target_pos[i] - current_pos[i]) * 1000 for i in range(3)]
                rospy.loginfo_throttle(8.0,
                    f"Rotation trajectory: current=({current_pos[0]:.3f},{current_pos[1]:.3f},{current_pos[2]:.3f}), "
                    f"target=({target_pos[0]:.3f},{target_pos[1]:.3f},{target_pos[2]:.3f}), "
                    f"diff=({pos_diff[0]:.1f},{pos_diff[1]:.1f},{pos_diff[2]:.1f})mm")
            
            # Important addition: Valve response adaptive control (mimic single version)
            if target_state and valve_angular_velocity > 0.05:  # Valve is rotating
                # Dynamic torque adjustment: reduce torque if valve rotates fast, increase if slow
                resistance_factor = max(0.5, min(2.0, 0.3 / max(0.05, valve_angular_velocity)))
                rospy.loginfo_throttle(3.0, f"Torque adaptation: valve_ω={valve_angular_velocity:.3f}rad/s, factor={resistance_factor:.2f}")
                
                # Adjust trajectory generator output torque
                if 'torque' in target_state and target_state['torque'] is not None:
                    original_torque = target_state['torque']
                    adapted_torque = [t * resistance_factor for t in original_torque]
                    target_state['torque'] = adapted_torque
                    rospy.logdebug(f"Torque adjustment: {original_torque} → {adapted_torque}")
            
            # Apply 3D speed limiting for formation safety (from legacy version)
            if target_state and 'linear_velocity' in target_state:
                linear_vel_vec = np.array(target_state['linear_velocity'])
                speed_total = np.linalg.norm(linear_vel_vec)
                
                # Limit total 3D speed
                max_speed = getattr(self, 'max_linear_speed', 0.08)
                if speed_total > max_speed and speed_total > 1e-6:
                    scale = max_speed / speed_total
                    linear_vel_vec *= scale
                    target_state['linear_velocity'] = linear_vel_vec.tolist() if hasattr(linear_vel_vec, 'tolist') else list(linear_vel_vec)
                    rospy.logdebug(f"Speed clamped: {speed_total:.3f}→{max_speed:.3f}m/s")
            
            if target_state:
                # Directly use same SE(3) control method as single UAV, eliminate wrapper layer
                self.beetle.executeTrajectoryWithWrench(
                    pos=target_state['position'],
                    rot=target_state['yaw'],
                    linear_vel=target_state['linear_velocity'],
                    angular_vel=target_state['angular_velocity'],
                    force=target_state['force'],
                    torque=target_state['torque']
                )
                    
            # 进度输出（每5秒）
            if int(elapsed) % 5 == 0 and (elapsed - int(elapsed)) < 0.1:
                progress = (valve_rotation / self.target_rotation) * 100
                rospy.loginfo(f"Rotation progress: {progress:.1f}% | Valve Δ={math.degrees(valve_rotation):.1f}° | "
                             f"Max={math.degrees(max_rotation_detected):.1f}° | Radius={self.contact_radius*1000:.1f}mm | "
                             f"ω={math.degrees(effective_angular_velocity):.1f}°/s")
            
            rospy.sleep(0.04)  # 25Hz
        
        # 旋转失败
        rospy.logerr(f"Valve rotation failed: achieved {math.degrees(max_rotation_detected):.1f}° "
                    f"of {math.degrees(self.target_rotation):.1f}° target")
        return 'failed'


class FormationDisengageFromValveState(FormationSingleUAVStateBase):
    """Formation disengage - reverse rotate, ascend, return to start"""
    
    def __init__(self):
        FormationSingleUAVStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['valve_position', 'start_position', 'start_ee_position', 'start_yaw', 'trajectory_state'],
            output_keys=['disengagement_position'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Formation Disengage ===")
        
        valve_pos = getattr(userdata, 'valve_position', None)
        if valve_pos is None:
            rospy.logerr("Valve position missing")
            return 'failed'
        
        start_assembly_pos = getattr(userdata, 'start_position', None)
        start_ee_pos = getattr(userdata, 'start_ee_position', None)
        start_yaw = getattr(userdata, 'start_yaw', 0.0)
        trajectory_state = getattr(userdata, 'trajectory_state', {}) or {}
        
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        if current_pos is None or current_yaw is None:
            rospy.logerr("Cannot get current pose")
            return 'failed'
        
        rospy.loginfo(f"Current: {FormationUtils.format_vec(current_pos)}, yaw={math.degrees(current_yaw):.1f}°")
        rospy.loginfo(f"Start: {FormationUtils.format_vec(start_ee_pos)}")
        
        # REVERSE_ROTATE
        rospy.loginfo("[REVERSE_ROTATE] Reverse 40° to disengage")
        
        generator_center = trajectory_state.get('valve_center', valve_pos)
        generator_radius = trajectory_state.get('current_radius')
        if generator_radius is None:
            generator_radius = math.sqrt((current_pos[0] - generator_center[0])**2 +
                                        (current_pos[1] - generator_center[1])**2)
        
        disengagement_radius = max(generator_radius, 0.01)
        rotation_direction = trajectory_state.get('rotation_direction', 1)
        reverse_angle = math.radians(40.0)  # Increased from 25° to 40°
        reverse_velocity = -rotation_direction * 0.033
        rospy.loginfo(f"  Reverse @ {math.degrees(reverse_velocity):.1f}°/s, radius={disengagement_radius*1000:.1f}mm")
        
        try:
            reverse_generator = OnlineCircularTrajectoryGenerator(
                valve_center=generator_center,
                initial_radius=disengagement_radius,
                target_angular_velocity=reverse_velocity,
                control_rate=25.0
            )
            reverse_generator.initialize_from_current_position(current_pos)
            
            # Warm-up phase: send stable commands before starting reverse rotation
            # This prevents the initial velocity spike
            rospy.loginfo("[REVERSE_ROTATE] Warm-up phase (1s)...")
            warmup_start = rospy.Time.now().to_sec()
            warmup_duration = 1.0
            control_rate = rospy.Rate(25)
            
            while (rospy.Time.now().to_sec() - warmup_start) < warmup_duration and not rospy.is_shutdown():
                updated_pos = self.get_end_effector_position()
                updated_yaw = self.get_end_effector_yaw()
                if updated_pos is not None and updated_yaw is not None:
                    # Send current position with zero velocity during warm-up
                    self.send_assembly_command_from_end_effector(
                        updated_pos, updated_yaw,
                        linear_vel=[0.0, 0.0, 0.0], angular_vel=0.0
                    )
                control_rate.sleep()
            
            # Re-initialize from current position after warm-up
            current_pos = self.get_end_effector_position()
            reverse_generator.initialize_from_current_position(current_pos)
            
            reverse_start = rospy.Time.now().to_sec()
            reverse_duration = abs(reverse_angle / reverse_velocity)
            
            while (rospy.Time.now().to_sec() - reverse_start) < reverse_duration and not rospy.is_shutdown():
                updated_pos = self.get_end_effector_position()
                updated_yaw = self.get_end_effector_yaw()
                if updated_pos is None or updated_yaw is None:
                    control_rate.sleep()
                    continue
                
                reverse_generator.update_state(updated_pos, updated_yaw, 0.0)
                target_state = reverse_generator.generate_target_state()
                self.send_assembly_command_from_end_effector(
                    target_state['position'].tolist(),
                    target_state['yaw'],
                    linear_vel=target_state['linear_velocity'].tolist(),
                    angular_vel=target_state.get('angular_velocity')
                )
                control_rate.sleep()
            
            rospy.loginfo("[REVERSE_ROTATE] Complete")
            
            # Stop command
            final_pos = self.get_end_effector_position()
            final_yaw = self.get_end_effector_yaw()
            if final_pos is not None and final_yaw is not None:
                for _ in range(5):
                    self.send_assembly_command_from_end_effector(
                        final_pos, final_yaw, linear_vel=[0.0, 0.0, 0.0], angular_vel=0.0
                    )
                    rospy.sleep(0.04)
            
        except Exception as e:
            rospy.logwarn(f"Reverse motion failed: {e}")
        
        # ASCENT: Rise to start height with XY shifted 20mm toward valve center
        rospy.sleep(1.0)
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        
        target_z = (start_ee_pos[2] + 0.1) if start_ee_pos else (current_pos[2] + 0.15)
        z_distance = target_z - current_pos[2]
        rospy.loginfo(f"[ASCENT] Rising {z_distance*1000:.1f}mm with XY shift toward valve center")
        
        step_size = 0.02
        
        # Calculate direction from Assembly CoG to valve center (not from EE)
        assembly_cog = self.get_assembly_position()
        valve_center_xy = (generator_center[0], generator_center[1])
        cog_to_valve = (valve_center_xy[0] - assembly_cog[0], valve_center_xy[1] - assembly_cog[1])
        dist_to_valve = math.sqrt(cog_to_valve[0]**2 + cog_to_valve[1]**2)
        
        rospy.loginfo(f"[ASCENT] Assembly CoG: ({assembly_cog[0]:.3f}, {assembly_cog[1]:.3f})")
        rospy.loginfo(f"[ASCENT] Valve center: ({valve_center_xy[0]:.3f}, {valve_center_xy[1]:.3f})")
        rospy.loginfo(f"[ASCENT] Distance CoG→Valve: {dist_to_valve*1000:.1f}mm")
        
        if dist_to_valve > 0.001:
            offset_dist = 0.020  # 20mm toward valve center
            unit_vec = (cog_to_valve[0] / dist_to_valve, cog_to_valve[1] / dist_to_valve)
            # Apply offset to current EE position along CoG→Valve direction
            locked_xy = (
                current_pos[0] + unit_vec[0] * offset_dist,
                current_pos[1] + unit_vec[1] * offset_dist
            )
            rospy.loginfo(f"[ASCENT] Direction unit vec: ({unit_vec[0]:.3f}, {unit_vec[1]:.3f})")
            rospy.loginfo(f"[ASCENT] EE: ({current_pos[0]:.3f}, {current_pos[1]:.3f}) → Locked XY: ({locked_xy[0]:.3f}, {locked_xy[1]:.3f})")
        else:
            locked_xy = (current_pos[0], current_pos[1])
            rospy.loginfo(f"[ASCENT] CoG already at valve center, using current EE XY")
        
        current_z = current_pos[2]
        
        while current_z < target_z and not rospy.is_shutdown():
            current_z = min(current_z + step_size, target_z)
            self.send_assembly_command_from_end_effector(
                (locked_xy[0], locked_xy[1], current_z), current_yaw, linear_vel=[0.01, 0.01, 0.0075]  # Added XY vel for 20mm offset
            )
            rospy.sleep(1.0)  # Increased from 0.5 to 1.0 for very slow ascent
        
        rospy.loginfo("[ASCENT] Complete")
        
        # 1.5-second stabilization at current position after ascent (reduced from 3s)
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        rospy.loginfo(f"[POST_ASCENT_STABILIZE] 1.5s stabilization at {FormationUtils.format_vec(current_pos)}")
        
        stabilize_start = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - stabilize_start) < 1.5 and not rospy.is_shutdown():
            self.send_assembly_command_from_end_effector(
                current_pos, current_yaw, linear_vel=[0, 0, 0]
            )
            rospy.sleep(0.1)
        rospy.loginfo("[POST_ASCENT_STABILIZE] Complete")
        
        rospy.sleep(1.0)
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        
        # YAW_RESTORE & XY_RETURN
        if start_ee_pos is not None:
            rospy.loginfo(f"[YAW_RESTORE] Adjusting yaw to {math.degrees(start_yaw):.1f}°")
            yaw_success = self.active_position_convergence(
                current_pos,
                target_yaw=start_yaw,
                pos_thresh=0.10,
                yaw_thresh=0.035,
                timeout=45.0,
                max_linear_vel=0.02,
                max_angular_vel=0.06,
                max_yaw_step=0.10
            )
            
            if not yaw_success:
                rospy.logwarn("[YAW_RESTORE] Incomplete")
            
            rospy.sleep(1.5)
            current_pos = self.get_end_effector_position()
            
            # XY_RETURN
            return_target = (start_ee_pos[0], start_ee_pos[1], current_pos[2])
            xy_distance = math.sqrt(
                (return_target[0] - current_pos[0])**2 + 
                (return_target[1] - current_pos[1])**2
            )
            rospy.loginfo(f"[XY_RETURN] Distance: {xy_distance*1000:.1f}mm")
            
            if xy_distance > 0.1:
                # Use polynomial trajectory for long distance
                num_points = max(12, min(20, int(xy_distance / 0.2)))
                trajectory_points = self.generate_polynomial_trajectory(
                    start_pos=current_pos,
                    target_pos=return_target,
                    target_yaw=start_yaw,
                    num_points=num_points,
                    lock_yaw=True
                )
                
                if trajectory_points:
                    self.execute_polynomial_trajectory(trajectory_points)
                
                # Final convergence
                self.active_position_convergence(
                    return_target, target_yaw=start_yaw,
                    pos_thresh=0.05, yaw_thresh=0.1, timeout=15.0,
                    max_linear_vel=0.05, max_angular_vel=0.035
                )
            else:
                # Short distance: direct convergence
                self.active_position_convergence(
                    return_target, target_yaw=start_yaw,
                    pos_thresh=0.05, yaw_thresh=0.1, timeout=20.0,
                    max_linear_vel=0.05, max_angular_vel=0.035
                )
        
        final_pos = self.get_end_effector_position()
        userdata.disengagement_position = final_pos
        rospy.loginfo(f"=== Disengagement complete: {FormationUtils.format_vec(final_pos)} ===")
        return 'succeeded'


# Main execution function
def main():
    rospy.init_node('formation_valve_rotation')
    
    # Get rotation angle from parameter (in radians)
    rotation_angle = rospy.get_param("~rotation_angle", 1.571)  # Default 90° in radians
    rospy.loginfo(f"Formation valve rotation angle: {math.degrees(rotation_angle):.1f}°")
    
    # Get rotation direction from parameter
    direction_param = rospy.get_param("~valve_rotation_direction", "counterclockwise")
    direction_normalized = direction_param.strip().lower()
    if direction_normalized in ["clockwise", "cw"]:
        rotation_direction = -1
        direction_label = "Clockwise (negative yaw)"
    elif direction_normalized in ["counterclockwise", "counter-clockwise", "ccw"]:
        rotation_direction = 1
        direction_label = "Counter-clockwise (positive yaw)"
    else:
        rotation_direction = 1
        direction_label = f"Counter-clockwise (fallback for '{direction_param}')"
        rospy.logwarn(f"Unknown valve rotation direction '{direction_param}', defaulting to counter-clockwise")

    rospy.loginfo(f"Formation valve rotation direction: {direction_label}")
    
    try:
        # Create Formation SMACH state machine
        sm = smach.StateMachine(outcomes=['success', 'failure'])
        
        with sm:
            # Step 1: Wait for formation ready and prepare for takeoff (ground assembly already done)
            smach.StateMachine.add('FORMATION_TAKEOFF',
                                   FormationTakeoffState(),
                                   transitions={'succeeded': 'FORMATION_INITIALIZE',
                                               'failed': 'failure'})
            
            # Step 2: Initialize positions and valve data
            smach.StateMachine.add('FORMATION_INITIALIZE',
                                   FormationInitializeStartPositionState(),
                                   transitions={'succeeded': 'FORMATION_MOVE_TO_VALVE',
                                               'failed': 'failure'})
            
            # Step 3: Move formation to valve approach position
            smach.StateMachine.add('FORMATION_MOVE_TO_VALVE',
                                   FormationMoveToValveState(),
                                   transitions={'succeeded': 'FORMATION_ROTATE_VALVE',
                                               'failed': 'failure'})
            
            # Step 4: Contact and rotate the valve (unified state)
            smach.StateMachine.add('FORMATION_ROTATE_VALVE',
                                   FormationRotateValveState(rotation_direction=rotation_direction, 
                                                            target_rotation=rotation_angle),
                                   transitions={'succeeded': 'FORMATION_DISENGAGE',
                                               'failed': 'failure',
                                               'emergency': 'failure'})
            
            # Step 6: Disengage from valve and return to start
            smach.StateMachine.add('FORMATION_DISENGAGE',
                                   FormationDisengageFromValveState(),
                                   transitions={'succeeded': 'success',
                                               'failed': 'failure'})
        
        # Execute state machine
        rospy.loginfo("Starting Formation valve rotation state machine...")
        outcome = sm.execute()
        rospy.loginfo(f"Formation valve rotation completed with outcome: {outcome}")
        
    except Exception as e:
        rospy.logerr(f"Error raised during SMACH container construction: \n{e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
