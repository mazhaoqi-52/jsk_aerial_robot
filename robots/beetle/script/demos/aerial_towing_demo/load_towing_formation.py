#!/usr/bin/env python3
"""
Formation Load Towing - Towing a load box using assembled Beetle UAVs
Reuses formation valve rotation infrastructure with linear trajectory generation
"""

import sys
import os
import math
import threading
import time
import rospy
import smach
import smach_ros
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
from tf.transformations import euler_from_quaternion

# Add parent paths for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
valve_demo_dir = os.path.join(current_dir, '../valve_rotation_demo')
sys.path.insert(0, valve_demo_dir)
sys.path.insert(0, os.path.join(current_dir, '../..'))
sys.path.insert(0, os.path.join(current_dir, '..'))

# Reuse components from valve rotation demo
from valve_rotation_formation_clean import (
    FormationAdapter,
    FormationSingleUAVStateBase,
    FormationAssembleState,
    FormationUtils,
    WaitState
)
from beetle_interface import BeetleInterface
from trajectory import PolynomialTrajectory


# ============== Load Box Parameters (from load.urdf) ==============
LOAD_BOX_LENGTH = 0.5    # x dimension (m)
LOAD_BOX_WIDTH = 0.3     # y dimension (m)
LOAD_BOX_HEIGHT = 0.3    # z dimension (m)
LOAD_WALL_THICKNESS = 0.02  # wall thickness (m)

# ============== Towing Task Parameters ==============
INSERTION_DEPTH = 0.03   # 30mm insertion depth
RETRACT_DISTANCE = 0.03  # 30mm retract to hook edge (wall_thickness + margin)
TOWING_DISTANCE = 1.0    # 1m towing distance
TOWING_FORCE = 5.0       # 5N initial feedforward force
APPROACH_HEIGHT_OFFSET = 0.05  # 50mm above box top for approach


class LinearTowingTrajectoryGenerator:
    """
    Linear trajectory generator for load towing task.
    Generates straight-line trajectory with force feedforward.
    """
    
    def __init__(self, start_pos, towing_direction, target_distance, 
                 target_velocity=0.05, control_rate=25.0, feedforward_force=5.0):
        """
        Initialize linear towing trajectory generator.
        
        Args:
            start_pos: Starting position [x, y, z]
            towing_direction: Unit vector of towing direction [dx, dy, 0]
            target_distance: Total distance to tow (m)
            target_velocity: Target towing velocity (m/s)
            control_rate: Control frequency (Hz)
            feedforward_force: Force to apply in towing direction (N)
        """
        self.start_pos = np.array(start_pos)
        self.towing_direction = np.array(towing_direction)
        # Normalize direction
        norm = np.linalg.norm(self.towing_direction[:2])
        if norm > 0.001:
            self.towing_direction = self.towing_direction / norm
        self.towing_direction[2] = 0.0  # Ensure horizontal
        
        self.target_distance = target_distance
        self.target_velocity = target_velocity
        self.control_rate = control_rate
        self.dt = 1.0 / control_rate
        self.feedforward_force = feedforward_force
        
        # State variables
        self.current_distance = 0.0
        self.current_velocity = 0.0
        self.target_pos = self.start_pos.copy()
        
        # Velocity ramp-up parameters
        self.ramp_up_distance = 0.05  # 50mm ramp-up zone
        self.ramp_down_distance = 0.05  # 50mm ramp-down zone
        
        # Force adaptation parameters
        self.min_force = feedforward_force * 0.5
        self.max_force = feedforward_force * 2.0
        self.current_force = feedforward_force
        
        # Performance monitoring
        self.start_time = rospy.Time.now().to_sec()
        self.last_update_time = self.start_time
        self.stall_counter = 0
        
        rospy.loginfo(f"LinearTowingTrajectory: dir={self.towing_direction}, "
                     f"dist={target_distance}m, vel={target_velocity}m/s, force={feedforward_force}N")
    
    def update_state(self, current_pos, load_pos=None):
        """
        Update trajectory state based on current position and load position.
        
        Args:
            current_pos: Current end-effector position [x, y, z]
            load_pos: Current load position (optional, for progress tracking)
            
        Returns:
            dict: Updated state info
        """
        current_time = rospy.Time.now().to_sec()
        actual_dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Limit time step
        safe_dt = min(actual_dt, 0.1)
        
        # Calculate current distance traveled
        displacement = np.array(current_pos) - self.start_pos
        self.current_distance = np.dot(displacement[:2], self.towing_direction[:2])
        
        # Velocity profile with ramp-up and ramp-down
        remaining_distance = self.target_distance - self.current_distance
        
        if self.current_distance < self.ramp_up_distance:
            # Ramp up phase
            velocity_factor = self.current_distance / self.ramp_up_distance
            self.current_velocity = self.target_velocity * max(0.2, velocity_factor)
        elif remaining_distance < self.ramp_down_distance:
            # Ramp down phase
            velocity_factor = remaining_distance / self.ramp_down_distance
            self.current_velocity = self.target_velocity * max(0.1, velocity_factor)
        else:
            # Constant velocity phase
            self.current_velocity = self.target_velocity
        
        # Update target position
        target_distance_increment = self.current_velocity * safe_dt
        new_target_distance = min(
            self.target_distance,
            np.dot(self.target_pos[:2] - self.start_pos[:2], self.towing_direction[:2]) + target_distance_increment
        )
        
        self.target_pos[:2] = self.start_pos[:2] + self.towing_direction[:2] * new_target_distance
        # Keep Z constant
        self.target_pos[2] = self.start_pos[2]
        
        # Adaptive force based on progress
        actual_velocity = self.current_distance / max(0.1, current_time - self.start_time)
        if actual_velocity < self.target_velocity * 0.3:
            # Increase force if moving too slow
            self.current_force = min(self.max_force, self.current_force * 1.1)
            self.stall_counter += 1
        else:
            self.current_force = self.feedforward_force
            self.stall_counter = 0
        
        return {
            'current_distance': self.current_distance,
            'target_distance': self.target_distance,
            'current_velocity': self.current_velocity,
            'current_force': self.current_force,
            'progress': self.current_distance / self.target_distance,
            'stall_counter': self.stall_counter
        }
    
    def generate_target_state(self, current_yaw):
        """
        Generate target state for control.
        
        Args:
            current_yaw: Current assembly yaw (maintain during towing)
            
        Returns:
            dict: Target state with position, velocity, force
        """
        # Target velocity in world frame
        target_linear_vel = self.towing_direction * self.current_velocity
        
        # Force in towing direction
        target_force = self.towing_direction * self.current_force
        # Add small vertical force component for stability
        target_force[2] = 0.5  # Small upward bias
        
        return {
            'position': self.target_pos.copy(),
            'yaw': current_yaw,
            'linear_velocity': target_linear_vel,
            'angular_velocity': 0.0,
            'force': target_force,
            'torque': np.array([0.0, 0.0, 0.0])
        }
    
    def is_complete(self):
        """Check if towing is complete."""
        return self.current_distance >= self.target_distance * 0.95
    
    def get_progress(self):
        """Get towing progress [0.0, 1.0]."""
        return min(1.0, self.current_distance / self.target_distance)


class LoadInterface:
    """Interface for load box position tracking."""
    
    def __init__(self):
        self.load_pos = None
        self.load_odom = None
        self.position_received = threading.Event()
        
        # Subscribe to load odometry
        rospy.Subscriber('/load/odom', Odometry, self._load_odom_cb, queue_size=1)
        rospy.loginfo("LoadInterface: Subscribed to /load/odom")
    
    def _load_odom_cb(self, msg):
        """Callback for load odometry."""
        pos = msg.pose.pose.position
        self.load_pos = np.array([pos.x, pos.y, pos.z])
        self.load_odom = msg
        
        if not self.position_received.is_set():
            rospy.loginfo(f"Load position received: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
            self.position_received.set()
    
    def get_load_position(self):
        """Get current load position."""
        return self.load_pos
    
    def get_load_center_xy(self):
        """Get load center XY position."""
        if self.load_pos is not None:
            return self.load_pos[:2]
        return None
    
    def get_load_top_z(self):
        """Get load box top Z coordinate (open top)."""
        if self.load_pos is not None:
            # Load center is at box_height/2, top is at center + height/2
            return self.load_pos[2] + LOAD_BOX_HEIGHT / 2
        return None
    
    def wait_for_load(self, timeout=10.0):
        """Wait for load position data."""
        rospy.loginfo("Waiting for load position...")
        if self.position_received.wait(timeout):
            rospy.loginfo("Load position available")
            return True
        rospy.logerr(f"Load position not received after {timeout}s")
        return False


class TowingStateBase(FormationSingleUAVStateBase):
    """Base class for towing states, extends FormationSingleUAVStateBase."""
    
    # Shared load interface across states
    _shared_load_interface = None
    
    def __init__(self, outcomes, input_keys=None, output_keys=None):
        FormationSingleUAVStateBase.__init__(self, outcomes, input_keys, output_keys)
        
        # Initialize shared load interface
        if TowingStateBase._shared_load_interface is None:
            TowingStateBase._shared_load_interface = LoadInterface()
        self.load_interface = TowingStateBase._shared_load_interface
    
    def get_load_position(self):
        """Get current load position."""
        return self.load_interface.get_load_position()
    
    def get_load_top_z(self):
        """Get load box top Z coordinate."""
        return self.load_interface.get_load_top_z()
    
    def calculate_approach_side(self, assembly_pos, load_pos):
        """
        Calculate which side of the load to approach (auto-detect nearest side).
        
        Returns:
            tuple: (approach_direction_vector, side_name, edge_position)
        """
        if assembly_pos is None or load_pos is None:
            return None, None, None
        
        # Calculate relative position of assembly to load
        rel_x = assembly_pos[0] - load_pos[0]
        rel_y = assembly_pos[1] - load_pos[1]
        
        # Determine nearest side based on which axis has larger relative displacement
        # and the sign of that displacement
        half_length = LOAD_BOX_LENGTH / 2
        half_width = LOAD_BOX_WIDTH / 2
        
        # Calculate distance to each side
        dist_to_plus_x = abs(rel_x - half_length)
        dist_to_minus_x = abs(rel_x + half_length)
        dist_to_plus_y = abs(rel_y - half_width)
        dist_to_minus_y = abs(rel_y + half_width)
        
        distances = {
            '+X': (dist_to_plus_x, np.array([1, 0, 0]), load_pos[0] + half_length),
            '-X': (dist_to_minus_x, np.array([-1, 0, 0]), load_pos[0] - half_length),
            '+Y': (dist_to_plus_y, np.array([0, 1, 0]), load_pos[1] + half_width),
            '-Y': (dist_to_minus_y, np.array([0, -1, 0]), load_pos[1] - half_width),
        }
        
        # Find nearest side
        nearest_side = min(distances.keys(), key=lambda k: distances[k][0])
        _, approach_dir, edge_coord = distances[nearest_side]
        
        # Calculate edge position
        if nearest_side in ['+X', '-X']:
            edge_pos = np.array([edge_coord, load_pos[1], load_pos[2]])
        else:
            edge_pos = np.array([load_pos[0], edge_coord, load_pos[2]])
        
        rospy.loginfo(f"Approach side: {nearest_side}, direction: {approach_dir}, edge: {edge_pos}")
        return approach_dir, nearest_side, edge_pos


class TowingInitializeState(TowingStateBase):
    """Initialize towing task - get positions and calculate approach strategy."""
    
    def __init__(self):
        TowingStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            output_keys=['start_position', 'start_yaw', 'load_position', 
                        'approach_direction', 'approach_side', 'edge_position'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Towing Initialize State ===")
        
        # Wait for load position
        if not self.load_interface.wait_for_load(timeout=15.0):
            rospy.logerr("Failed to get load position")
            return 'failed'
        
        rospy.sleep(2.0)  # Wait for stable readings
        
        # Get positions
        assembly_pos = self.get_assembly_position()
        assembly_yaw = self.get_assembly_yaw()
        load_pos = self.get_load_position()
        
        if assembly_pos is None:
            rospy.logerr("Assembly position not available")
            return 'failed'
        
        if load_pos is None:
            rospy.logerr("Load position not available")
            return 'failed'
        
        # Calculate approach side
        approach_dir, side_name, edge_pos = self.calculate_approach_side(assembly_pos, load_pos)
        
        if approach_dir is None:
            rospy.logerr("Failed to calculate approach direction")
            return 'failed'
        
        # Store in userdata
        userdata.start_position = assembly_pos
        userdata.start_yaw = assembly_yaw
        userdata.load_position = load_pos
        userdata.approach_direction = approach_dir
        userdata.approach_side = side_name
        userdata.edge_position = edge_pos
        
        rospy.loginfo(f"Assembly position: {FormationUtils.format_vec(assembly_pos)}")
        rospy.loginfo(f"Assembly yaw: {math.degrees(assembly_yaw):.1f}°")
        rospy.loginfo(f"Load position: {FormationUtils.format_vec(load_pos)}")
        rospy.loginfo(f"Approach side: {side_name}")
        
        return 'succeeded'


class ApproachLoadState(TowingStateBase):
    """Move end-effector to position above load edge, slightly over the box."""
    
    def __init__(self):
        TowingStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['load_position', 'approach_direction', 'approach_side', 'edge_position'],
            output_keys=['insertion_position', 'insertion_yaw'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Approach Load State ===")
        
        load_pos = userdata.load_position
        approach_dir = userdata.approach_direction
        edge_pos = userdata.edge_position
        
        # Calculate approach position: above box edge, slightly over the top
        load_top_z = load_pos[2] + LOAD_BOX_HEIGHT / 2
        approach_height = load_top_z + APPROACH_HEIGHT_OFFSET
        
        # Position slightly inside the box edge (overshoot by 30mm for insertion)
        overshoot = 0.03  # 30mm inside box edge
        approach_xy = edge_pos[:2] - approach_dir[:2] * overshoot
        
        approach_pos = np.array([approach_xy[0], approach_xy[1], approach_height])
        
        # Calculate target yaw: facing into the box (opposite of approach direction)
        target_yaw = math.atan2(-approach_dir[1], -approach_dir[0])
        
        rospy.loginfo(f"Approach position: {FormationUtils.format_vec(approach_pos)}")
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
        
        # Execute approach movement
        current_pos = self.get_end_effector_position()
        if current_pos is None:
            rospy.logerr("Cannot get current end-effector position")
            return 'failed'
        
        # Phase 1: Adjust height first
        rospy.loginfo("[Phase 1] Adjusting height")
        height_target = (current_pos[0], current_pos[1], approach_height)
        success = self.active_position_convergence(
            height_target, target_yaw=self.get_end_effector_yaw(),
            pos_thresh=0.03, yaw_thresh=0.1, timeout=15.0
        )
        
        if not success:
            rospy.logwarn("Height adjustment incomplete, continuing...")
        
        # Phase 2: Move to XY position above box
        rospy.loginfo("[Phase 2] Moving to approach XY position")
        trajectory_points = self.generate_polynomial_trajectory(
            start_pos=self.get_end_effector_position(),
            target_pos=approach_pos,
            target_yaw=target_yaw,
            num_points=15
        )
        
        if trajectory_points:
            self.execute_polynomial_trajectory(trajectory_points)
        
        # Final convergence
        success = self.active_position_convergence(
            approach_pos, target_yaw=target_yaw,
            pos_thresh=0.025, yaw_thresh=0.05, timeout=20.0
        )
        
        if not success:
            rospy.logwarn("Approach convergence incomplete")
        
        # Stabilize
        rospy.loginfo("Stabilizing at approach position...")
        self.active_stabilization_wait(approach_pos, target_yaw, duration=2.0)
        
        # Store for next state
        insertion_z = load_top_z - INSERTION_DEPTH
        userdata.insertion_position = np.array([approach_xy[0], approach_xy[1], insertion_z])
        userdata.insertion_yaw = target_yaw
        
        rospy.loginfo("Approach complete")
        return 'succeeded'


class DescendAndInsertState(TowingStateBase):
    """Descend to insert end-effector into the load box."""
    
    def __init__(self):
        TowingStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['insertion_position', 'insertion_yaw', 'approach_direction'],
            output_keys=['hook_position', 'hook_yaw'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Descend and Insert State ===")
        
        insertion_pos = userdata.insertion_position
        insertion_yaw = userdata.insertion_yaw
        
        rospy.loginfo(f"Insertion target: {FormationUtils.format_vec(insertion_pos)}")
        rospy.loginfo(f"Insertion depth: {INSERTION_DEPTH*1000:.0f}mm into box")
        
        current_pos = self.get_end_effector_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position")
            return 'failed'
        
        # Use controlled Z descent
        success, achieved_pos = self.controlled_z_descent(
            current_pos, 
            insertion_pos, 
            insertion_yaw,
            descent_speed=0.05  # Slow descent
        )
        
        if not success:
            rospy.logwarn("Z descent incomplete, checking position...")
            achieved_pos = self.get_end_effector_position()
        
        # Verify insertion depth
        if achieved_pos is not None:
            actual_depth = (userdata.insertion_position[2] + INSERTION_DEPTH) - achieved_pos[2]
            rospy.loginfo(f"Achieved insertion: {actual_depth*1000:.1f}mm")
        
        # Stabilize at insertion position
        rospy.loginfo("Stabilizing at insertion position...")
        self.active_stabilization_wait(insertion_pos, insertion_yaw, duration=2.0)
        
        # Calculate hook position (retract to hook edge)
        approach_dir = userdata.approach_direction
        hook_pos = insertion_pos.copy()
        hook_pos[:2] = insertion_pos[:2] + approach_dir[:2] * RETRACT_DISTANCE
        
        userdata.hook_position = hook_pos
        userdata.hook_yaw = insertion_yaw
        
        rospy.loginfo("Insertion complete")
        return 'succeeded'


class RetractAndHookState(TowingStateBase):
    """Retract horizontally to hook the box edge."""
    
    def __init__(self):
        TowingStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['hook_position', 'hook_yaw', 'approach_direction'],
            output_keys=['towing_start_position', 'towing_direction'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Retract and Hook State ===")
        
        hook_pos = userdata.hook_position
        hook_yaw = userdata.hook_yaw
        approach_dir = userdata.approach_direction
        
        rospy.loginfo(f"Hook target: {FormationUtils.format_vec(hook_pos)}")
        rospy.loginfo(f"Retract distance: {RETRACT_DISTANCE*1000:.0f}mm")
        
        current_pos = self.get_end_effector_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position")
            return 'failed'
        
        # Slow horizontal retraction
        success = self.active_position_convergence(
            hook_pos, target_yaw=hook_yaw,
            pos_thresh=0.015, yaw_thresh=0.05, timeout=15.0,
            max_linear_vel=0.03  # Very slow for precise hooking
        )
        
        if not success:
            rospy.logwarn("Hook convergence incomplete")
        
        # Verify hook by checking if we can sense resistance
        rospy.loginfo("Verifying hook contact...")
        rospy.sleep(1.0)
        
        # Stabilize
        self.active_stabilization_wait(hook_pos, hook_yaw, duration=2.0)
        
        # Towing direction is same as approach direction (pulling outward)
        towing_direction = approach_dir.copy()
        
        userdata.towing_start_position = self.get_end_effector_position()
        userdata.towing_direction = towing_direction
        
        rospy.loginfo(f"Hook complete. Towing direction: {towing_direction}")
        return 'succeeded'


class TowingWithFeedforwardState(TowingStateBase):
    """Tow the load with force feedforward."""
    
    def __init__(self):
        TowingStateBase.__init__(self,
            outcomes=['succeeded', 'failed', 'timeout'],
            input_keys=['towing_start_position', 'towing_direction', 'hook_yaw'],
            output_keys=['towing_end_position'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Towing With Feedforward State ===")
        
        start_pos = userdata.towing_start_position
        towing_dir = userdata.towing_direction
        maintain_yaw = userdata.hook_yaw
        
        rospy.loginfo(f"Towing start: {FormationUtils.format_vec(start_pos)}")
        rospy.loginfo(f"Towing direction: {towing_dir}")
        rospy.loginfo(f"Towing distance: {TOWING_DISTANCE}m")
        rospy.loginfo(f"Feedforward force: {TOWING_FORCE}N")
        
        # Create trajectory generator
        trajectory_gen = LinearTowingTrajectoryGenerator(
            start_pos=start_pos,
            towing_direction=towing_dir,
            target_distance=TOWING_DISTANCE,
            target_velocity=0.05,  # 50mm/s towing speed
            feedforward_force=TOWING_FORCE
        )
        
        # Towing control loop
        towing_start_time = rospy.Time.now().to_sec()
        max_towing_time = TOWING_DISTANCE / 0.03 + 30  # Expected time + margin
        control_rate = rospy.Rate(25)  # 25Hz
        
        rospy.loginfo("Starting towing loop...")
        
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - towing_start_time
            
            # Get current state
            current_pos = self.get_end_effector_position()
            current_yaw = self.get_end_effector_yaw()
            load_pos = self.get_load_position()
            
            if current_pos is None:
                rospy.logwarn("Lost position, waiting...")
                control_rate.sleep()
                continue
            
            # Update trajectory
            state_info = trajectory_gen.update_state(current_pos, load_pos)
            
            # Check completion
            if trajectory_gen.is_complete():
                rospy.loginfo(f"Towing complete! Distance: {state_info['current_distance']*1000:.0f}mm")
                break
            
            # Check timeout
            if elapsed > max_towing_time:
                rospy.logwarn(f"Towing timeout after {elapsed:.1f}s")
                userdata.towing_end_position = current_pos
                return 'timeout'
            
            # Check for stall
            if state_info['stall_counter'] > 50:
                rospy.logwarn("Towing stalled - load may be stuck")
            
            # Generate and execute target
            target_state = trajectory_gen.generate_target_state(maintain_yaw)
            
            # Execute with force feedforward
            self.beetle.executeTrajectoryWithWrench(
                pos=target_state['position'].tolist(),
                rot=target_state['yaw'],
                linear_vel=target_state['linear_velocity'].tolist(),
                angular_vel=target_state['angular_velocity'],
                force=target_state['force'].tolist(),
                torque=target_state['torque'].tolist()
            )
            
            # Log progress
            if int(elapsed) % 5 == 0 and int(elapsed * 10) % 50 == 0:
                rospy.loginfo(f"Towing: {state_info['progress']*100:.1f}%, "
                             f"dist={state_info['current_distance']*1000:.0f}mm, "
                             f"force={state_info['current_force']:.1f}N, "
                             f"time={elapsed:.1f}s")
            
            control_rate.sleep()
        
        # Clear external wrench
        self.beetle.clearExternalWrench()
        
        final_pos = self.get_end_effector_position()
        userdata.towing_end_position = final_pos
        
        rospy.loginfo(f"Towing phase complete at {FormationUtils.format_vec(final_pos)}")
        return 'succeeded'


class DisengageAndReturnState(TowingStateBase):
    """Disengage from load and return to start position."""
    
    def __init__(self):
        TowingStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['start_position', 'start_yaw', 'towing_end_position'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Disengage and Return State ===")
        
        start_pos = userdata.start_position
        start_yaw = userdata.start_yaw
        
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        
        if current_pos is None:
            rospy.logerr("Cannot get current position")
            return 'failed'
        
        # Phase 1: Ascend to safe height
        rospy.loginfo("[Phase 1] Ascending to safe height")
        safe_height = current_pos[2] + 0.15  # 150mm up
        ascent_target = (current_pos[0], current_pos[1], safe_height)
        
        success = self.active_position_convergence(
            ascent_target, target_yaw=current_yaw,
            pos_thresh=0.05, yaw_thresh=0.1, timeout=20.0,
            max_linear_vel=0.05
        )
        
        rospy.sleep(1.0)
        
        # Phase 2: Adjust yaw to start yaw
        rospy.loginfo(f"[Phase 2] Adjusting yaw to {math.degrees(start_yaw):.1f}°")
        current_pos = self.get_end_effector_position()
        
        success = self.active_position_convergence(
            current_pos, target_yaw=start_yaw,
            pos_thresh=0.1, yaw_thresh=0.05, timeout=30.0,
            max_yaw_step=0.05, yaw_only=True
        )
        
        rospy.sleep(1.0)
        
        # Phase 3: Return to start XY position
        rospy.loginfo("[Phase 3] Returning to start position")
        current_pos = self.get_end_effector_position()
        return_target = (start_pos[0], start_pos[1], current_pos[2])
        
        # Use polynomial trajectory for smooth return
        trajectory_points = self.generate_polynomial_trajectory(
            start_pos=current_pos,
            target_pos=return_target,
            target_yaw=start_yaw,
            num_points=15,
            lock_yaw=True
        )
        
        if trajectory_points:
            self.execute_polynomial_trajectory(trajectory_points)
        
        success = self.active_position_convergence(
            return_target, target_yaw=start_yaw,
            pos_thresh=0.05, yaw_thresh=0.1, timeout=30.0
        )
        
        rospy.loginfo("=== Towing Task Complete ===")
        return 'succeeded'


def main():
    rospy.init_node('formation_load_towing')
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("Formation Load Towing Task")
    rospy.loginfo("=" * 60)
    rospy.loginfo(f"Load box: {LOAD_BOX_LENGTH}m x {LOAD_BOX_WIDTH}m x {LOAD_BOX_HEIGHT}m")
    rospy.loginfo(f"Insertion depth: {INSERTION_DEPTH*1000:.0f}mm")
    rospy.loginfo(f"Retract distance: {RETRACT_DISTANCE*1000:.0f}mm")
    rospy.loginfo(f"Towing distance: {TOWING_DISTANCE}m")
    rospy.loginfo(f"Towing force: {TOWING_FORCE}N")
    rospy.loginfo("=" * 60)
    
    try:
        # Create SMACH state machine
        sm = smach.StateMachine(outcomes=['success', 'failure'])
        
        with sm:
            # Step 1: Assemble the formation
            smach.StateMachine.add('FORMATION_ASSEMBLE',
                                   FormationAssembleState(),
                                   transitions={'succeeded': 'WAIT_AFTER_ASSEMBLE',
                                               'failed': 'failure'})
            
            # Wait after assembly
            smach.StateMachine.add('WAIT_AFTER_ASSEMBLE',
                                   WaitState(wait_time=3.0, state_name="INITIALIZE"),
                                   transitions={'succeeded': 'TOWING_INITIALIZE'})
            
            # Step 2: Initialize towing task
            smach.StateMachine.add('TOWING_INITIALIZE',
                                   TowingInitializeState(),
                                   transitions={'succeeded': 'APPROACH_LOAD',
                                               'failed': 'failure'})
            
            # Step 3: Approach load box
            smach.StateMachine.add('APPROACH_LOAD',
                                   ApproachLoadState(),
                                   transitions={'succeeded': 'DESCEND_AND_INSERT',
                                               'failed': 'failure'})
            
            # Step 4: Descend and insert
            smach.StateMachine.add('DESCEND_AND_INSERT',
                                   DescendAndInsertState(),
                                   transitions={'succeeded': 'RETRACT_AND_HOOK',
                                               'failed': 'failure'})
            
            # Step 5: Retract and hook
            smach.StateMachine.add('RETRACT_AND_HOOK',
                                   RetractAndHookState(),
                                   transitions={'succeeded': 'TOWING_WITH_FEEDFORWARD',
                                               'failed': 'failure'})
            
            # Step 6: Tow with feedforward
            smach.StateMachine.add('TOWING_WITH_FEEDFORWARD',
                                   TowingWithFeedforwardState(),
                                   transitions={'succeeded': 'DISENGAGE_AND_RETURN',
                                               'failed': 'failure',
                                               'timeout': 'DISENGAGE_AND_RETURN'})
            
            # Step 7: Disengage and return
            smach.StateMachine.add('DISENGAGE_AND_RETURN',
                                   DisengageAndReturnState(),
                                   transitions={'succeeded': 'success',
                                               'failed': 'failure'})
        
        # Execute state machine
        rospy.loginfo("Starting Formation Load Towing state machine...")
        outcome = sm.execute()
        rospy.loginfo(f"Formation Load Towing completed with outcome: {outcome}")
        
    except Exception as e:
        rospy.logerr(f"Error during state machine execution: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
