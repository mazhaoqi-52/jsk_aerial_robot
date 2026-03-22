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
LOAD_BOX_HEIGHT = 0.65   # z dimension (m)
LOAD_WALL_THICKNESS = 0.02  # wall thickness (m)

# ============== Towing Task Parameters ==============
# Note: end_effector_offset_z in n_modules_tf.py (0.074382) differs from actual fang tip offset (~0.041)
# This causes ~33mm extra insertion. Compensate by reducing INSERTION_DEPTH.
# Target actual insertion: 30mm, so set to approximately 0 or negative to compensate.
INSERTION_DEPTH = 0.0    # Compensated insertion depth (actual ~30mm due to offset error)
RETRACT_DISTANCE = 0.03  # 30mm retract to hook edge (wall_thickness + margin)
HOOK_POSITION_TOLERANCE = 0.030  # 30mm tolerance for hook convergence (formation control noise)
TOWING_DISTANCE = 0.5    # towing distance (0.5m sufficient for validation)
TOWING_MAX_FORCE = 20.0  # Maximum adaptive force (starts from 0, increases when stalled)
# Approach height above box top for entering the load area.
# Default is 50mm. Tune as needed; with box_top_z=0.65m:
# - offset 0.15 -> approach 0.80m
# - offset 0.18 -> approach 0.83m
APPROACH_HEIGHT_OFFSET = 0.18


class LinearTowingTrajectoryGenerator:
    """
    Linear trajectory generator for load towing task.
    Generates straight-line trajectory with force feedforward.
    """
    
    def __init__(self, start_pos, towing_direction, target_distance, 
                 target_velocity=0.05, control_rate=25.0, max_force=30.0):
        """
        Initialize linear towing trajectory generator.
        
        Args:
            start_pos: Starting position [x, y, z]
            towing_direction: Unit vector of towing direction [dx, dy, 0]
            target_distance: Total distance to tow (m)
            target_velocity: Target towing velocity (m/s)
            control_rate: Control frequency (Hz)
            max_force: Maximum adaptive force in towing direction (N)
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
        self.max_force = max_force
        
        # State variables
        self.current_distance = 0.0
        self.current_velocity = 0.0
        self.target_pos = self.start_pos.copy()
        
        # Velocity ramp-up parameters
        self.ramp_up_distance = 0.05  # 50mm ramp-up zone
        self.ramp_down_distance = 0.05  # 50mm ramp-down zone
        
        # Force adaptation: starts from 0, increases when stalled (reactive control)
        self.current_force = 0.0  # Start from 0N, not preset value
        
        # Performance monitoring
        self.start_time = rospy.Time.now().to_sec()
        self.last_update_time = self.start_time
        self.stall_counter = 0
        
        # Sliding window for stall-based abort detection
        self.stall_window_time = 5.0    # seconds per window
        self.stall_last_check_time = self.start_time
        self.stall_last_check_distance = 0.0
        
        # Z anti-windup: target Z pre-compensation to prevent I-term buildup
        # When towing force is applied, the drone dips slightly; this offset
        # pre-adjusts the target Z to reduce I-term accumulation.
        self.z_offset = 0.0           # current Z offset applied to target (m)
        self.z_offset_decay_rate = 0.003  # decay rate (m/s) — ~10s to recover 30mm
        self.z_force_coupling = 0.0015    # m/N — Z offset per unit force
        self.z_offset_decaying = False    # True once force stabilized, decay begins
        self.z_force_stable_time = None   # time when force first reached max
        
        rospy.loginfo(f"LinearTowingTrajectory: dir={self.towing_direction}, "
                     f"dist={target_distance}m, vel={target_velocity}m/s, max_force={max_force}N")
    
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
        
        # Z anti-windup: manage target Z pre-compensation
        # Phase 1 (force ramping): lower target Z to match expected pitch-induced drop
        # Phase 2 (force stable >5s): gradually decay offset back to zero
        force_at_max = (self.current_force >= self.max_force * 0.99)
        
        if not self.z_offset_decaying:
            # Still in tracking phase
            force_induced_offset = -self.z_force_coupling * self.current_force
            self.z_offset = force_induced_offset
            
            # Check if force has been at max long enough to start decay
            if force_at_max:
                if self.z_force_stable_time is None:
                    self.z_force_stable_time = current_time
                elif current_time - self.z_force_stable_time > 5.0:
                    # Force stable for 5s — Z transient should have settled
                    self.z_offset_decaying = True
                    rospy.loginfo(f"Z offset decay starting: z_offset={self.z_offset*1000:.1f}mm")
            else:
                self.z_force_stable_time = None  # reset if force drops
        else:
            # Decay phase: gradually return z_offset to zero
            if self.z_offset < 0:
                self.z_offset = min(0.0, self.z_offset + self.z_offset_decay_rate * safe_dt)
        
        self.target_pos[2] = self.start_pos[2] + self.z_offset
        
        # Stall detection using sliding window (for abort decision only)
        window_elapsed = current_time - self.stall_last_check_time
        if window_elapsed >= self.stall_window_time:
            window_distance = self.current_distance - self.stall_last_check_distance
            recent_velocity = window_distance / window_elapsed
            self._recent_velocity = recent_velocity
            self.stall_last_check_time = current_time
            self.stall_last_check_distance = self.current_distance
            
            if recent_velocity < self.target_velocity * 0.05:  # < 5% of target
                self.stall_counter += 1  # increments once per window
            else:
                self.stall_counter = max(0, self.stall_counter - 1)
        
        # Adaptive force based on recent windowed velocity (not global average)
        # Use the same sliding window velocity computed above for stall detection.
        # Between window updates, approximate with global average as fallback.
        if hasattr(self, '_recent_velocity'):
            inst_vel = self._recent_velocity
        else:
            inst_vel = self.current_distance / max(0.1, current_time - self.start_time)
        
        if inst_vel < self.target_velocity * 0.5:
            # Below 50% target: ramp up +0.5N/cycle = +12.5N/s at 25Hz
            self.current_force = min(self.max_force, self.current_force + 0.5)
        elif inst_vel > self.target_velocity * 0.8:
            # Above 80% target: decay -0.2N/cycle = -5N/s at 25Hz
            self.current_force = max(0.0, self.current_force - 0.2)
        # Between 50%-80%: hold current force (hysteresis band)
        
        return {
            'current_distance': self.current_distance,
            'target_distance': self.target_distance,
            'current_velocity': self.current_velocity,
            'current_force': self.current_force,
            'progress': self.current_distance / self.target_distance,
            'stall_counter': self.stall_counter,
            'z_offset': self.z_offset
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
        
        # Force feedforward: horizontal force in towing direction
        # Z: set to 0. C++ LEADER injects body_z directly (bypassing cog_rot)
        # to avoid pitch-coupling instability where cog_rot cross-couples
        # large body_x into world_z at non-zero pitch angles.
        target_force = self.towing_direction * self.current_force
        target_force[2] = 0.0
        
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
        Calculate which side of the load to approach (nearest edge midpoint).
        Approach direction is perpendicular to the selected edge, pointing outward.
        
        Returns:
            tuple: (approach_direction_vector, side_name, edge_midpoint_position)
        """
        if assembly_pos is None or load_pos is None:
            return None, None, None
        
        half_length = LOAD_BOX_LENGTH / 2
        half_width = LOAD_BOX_WIDTH / 2
        
        # Calculate each edge midpoint position
        edge_midpoints = {
            '+X': np.array([load_pos[0] + half_length, load_pos[1], load_pos[2]]),
            '-X': np.array([load_pos[0] - half_length, load_pos[1], load_pos[2]]),
            '+Y': np.array([load_pos[0], load_pos[1] + half_width, load_pos[2]]),
            '-Y': np.array([load_pos[0], load_pos[1] - half_width, load_pos[2]]),
        }
        
        # Approach direction: perpendicular to edge, pointing outward
        approach_dirs = {
            '+X': np.array([1, 0, 0]),
            '-X': np.array([-1, 0, 0]),
            '+Y': np.array([0, 1, 0]),
            '-Y': np.array([0, -1, 0]),
        }
        
        # Calculate distance from assembly to each edge midpoint, select nearest
        distances = {side: np.linalg.norm(np.array(assembly_pos) - midpoint) 
                     for side, midpoint in edge_midpoints.items()}
        
        nearest_side = min(distances.keys(), key=lambda k: distances[k])
        edge_pos = edge_midpoints[nearest_side]
        approach_dir = approach_dirs[nearest_side]
        
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

        # Calculate positions
        # NOTE: `/load/odom` provides the load pose at the model origin (center) in this setup.
        # Therefore: top_z = center_z + height/2.
        load_center_z = float(load_pos[2])
        load_top_z = load_center_z + LOAD_BOX_HEIGHT / 2
        approach_height = load_top_z + APPROACH_HEIGHT_OFFSET

        # Position slightly inside the box edge (overshoot by 30mm for insertion)
        overshoot = 0.03  # 30mm inside box edge
        approach_xy = edge_pos[:2] - approach_dir[:2] * overshoot

        # Calculate target yaw: facing into the box (opposite of approach direction)
        target_yaw = math.atan2(-approach_dir[1], -approach_dir[0])

        # Get current state
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        if current_pos is None:
            rospy.logerr("Cannot get current end-effector position")
            return 'failed'

        rospy.loginfo(f"Current position: {FormationUtils.format_vec(current_pos)}")
        rospy.loginfo(f"Target XY: ({approach_xy[0]:.3f}, {approach_xy[1]:.3f})")
        rospy.loginfo(
            f"Load Z: center={load_center_z:.3f}m, top={load_top_z:.3f}m, "
            f"approach_offset={APPROACH_HEIGHT_OFFSET:.3f}m"
        )
        rospy.loginfo(f"Target height: {approach_height:.3f}m")
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")

        # Phase 1: XY movement (keep current Z)
        rospy.loginfo("[Phase 1] Moving to XY position above box (Z unchanged)")
        phase1_target = np.array([approach_xy[0], approach_xy[1], current_pos[2]])

        trajectory_points = self.generate_polynomial_trajectory(
            start_pos=current_pos,
            target_pos=phase1_target,
            target_yaw=current_yaw,  # Keep current yaw during XY movement
            lock_yaw=True
        )

        if trajectory_points:
            self.execute_polynomial_trajectory(trajectory_points)

        success = self.active_position_convergence(
            phase1_target, target_yaw=current_yaw,
            pos_thresh=0.03, yaw_thresh=0.1, timeout=20.0
        )
        if not success:
            rospy.logwarn("Phase 1 XY convergence incomplete, continuing...")

        # Phase 2: Adjust yaw (keep position)
        rospy.loginfo(f"[Phase 2] Adjusting yaw to {math.degrees(target_yaw):.1f}°")
        current_pos = self.get_end_effector_position()

        success = self.active_position_convergence(
            current_pos, target_yaw=target_yaw,
            pos_thresh=0.05, yaw_thresh=0.05, timeout=15.0,
            yaw_only=True
        )
        if not success:
            rospy.logwarn("Phase 2 yaw adjustment incomplete, continuing...")

        # Phase 3: Z descent to approach height (keep XY) - Use polynomial trajectory for smooth descent
        rospy.loginfo(f"[Phase 3] Descending to approach height {approach_height:.3f}m using polynomial trajectory")
        current_pos = self.get_end_effector_position()
        phase3_target = np.array([current_pos[0], current_pos[1], approach_height])

        # Calculate descent distance
        descent_distance = abs(current_pos[2] - approach_height)
        rospy.loginfo(f"Descent distance: {descent_distance*1000:.1f}mm")

        # Use streaming polynomial trajectory for smooth descent
        traj_desc = self.generate_polynomial_trajectory(
            start_pos=current_pos,
            target_pos=phase3_target,
            target_yaw=target_yaw,
            lock_yaw=True   # Lock yaw during descent, focus on Z axis
        )

        if traj_desc:
            self.execute_polynomial_trajectory(traj_desc)

        # Final precision convergence with gentle velocity to avoid "impact"
        rospy.loginfo("Final precision positioning with gentle velocity...")
        success = self.active_position_convergence(
            phase3_target, target_yaw=target_yaw,
            pos_thresh=0.040, yaw_thresh=0.05, timeout=15.0,
            max_linear_vel=0.015,  # Very gentle: 15mm/s for final adjustment
            max_angular_vel=0.03
        )
        if not success:
            rospy.logwarn("Phase 3 Z descent incomplete, continuing...")

        # Stabilize
        rospy.loginfo("Stabilizing at approach position...")
        approach_pos = np.array([approach_xy[0], approach_xy[1], approach_height])
        self.active_stabilization_wait(approach_pos, target_yaw, duration=2.0)

        # Store for next state
        # Z=approach_height is already "inserted" (end-effector inside the box opening).
        # No further descent needed; RETRACT_AND_HOOK will operate at this height.
        insertion_z = approach_height
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
            descent_speed=0.03  # Slower descent (gentler insertion)
        )
        
        if not success:
            # If descent aborted (e.g., retry limit hit), don't continue with hook/tow.
            rospy.logerr("Z descent failed (aborted). Exiting state machine.")
            return 'failed'
        
        # Verify insertion depth
        if achieved_pos is not None:
            # Positive means deeper into the box (below the planned top surface).
            box_top_z = (userdata.insertion_position[2] + INSERTION_DEPTH)
            actual_depth = box_top_z - achieved_pos[2]
            rospy.loginfo(f"Achieved insertion: {actual_depth*1000:.1f}mm (positive=deeper)")
        
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
            pos_thresh=HOOK_POSITION_TOLERANCE, yaw_thresh=0.05, timeout=15.0,
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
        
        # Disable pitch compensation during towing to avoid wrench_comp Z-drift.
        # Root cause: C++ momentum observer detects body-frame drag force during towing.
        # At pitch -1.5°, body-X force (5N) couples ~131mN into world-Z through rotation.
        # This Z component flows: wrench_comp → I_reconfig_acc → pid_FZ → I_comp_Fz →
        # pid_Z.setICompTerm → monotonic Z-PID I-term accumulation → upward drift.
        # With pitch compensation ON, the mocap-based observation amplifies this coupling.
        # Theoretical pitch effect on EE position is only ~2mm, much less than the 7mm drift.
        self.formation_adapter.set_pitch_compensation(False)
        
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
    
    def _clear_external_wrench(self):
        """Clear external wrench feedforward via BeetleInterface.
        
        Sends multiple zero-wrench messages for reliability, then marks
        wrench as inactive. Wrench topic is automatically selected based
        on unified/leader-follower mode.
        """
        rospy.loginfo("Clearing external wrench feedforward")
        zero = [0.0, 0.0, 0.0]
        for _ in range(10):
            self.beetle.addExternalWrench(zero, zero)
            rospy.sleep(0.04)
        self.beetle.clearExternalWrench()
    
    def execute(self, userdata):
        rospy.loginfo("=== Towing With Feedforward State ===")
        
        # Pitch compensation is DISABLED in RetractAndHookState to avoid wrench_comp Z-drift
        
        start_pos = userdata.towing_start_position
        towing_dir = userdata.towing_direction
        maintain_yaw = userdata.hook_yaw
        
        rospy.loginfo(f"Towing start: {FormationUtils.format_vec(start_pos)}")
        rospy.loginfo(f"Towing direction: {towing_dir}")
        rospy.loginfo(f"Towing distance: {TOWING_DISTANCE}m")
        
        # ---- Wrench feedforward via BeetleInterface ----
        # BeetleInterface.addExternalWrench() automatically routes to:
        #   - formation_desired_wrench  when unified_control_mode is active
        #   - desired_external_wrench   when leader-follower (wrench_comp) is active
        # It also handles world→body frame rotation internally.
        control_mode = 'unified' if self.beetle.isUnifiedMode() else 'leader-follower'
        rospy.loginfo(f"Wrench feedforward via BeetleInterface (mode: {control_mode})")
        
        trajectory_gen = LinearTowingTrajectoryGenerator(
            start_pos=start_pos,
            towing_direction=towing_dir,
            target_distance=TOWING_DISTANCE,
            target_velocity=0.05,  # 50mm/s towing speed
            max_force=TOWING_MAX_FORCE
        )
        
        # Note: We rely on the low-level PID controller (BeetleControl.yaml)
        # for Z-axis stabilization. The controller has d_gain=5.0 which provides
        # damping to suppress oscillation. We only provide position targets.
        
        # Towing control loop
        towing_start_time = rospy.Time.now().to_sec()
        # Stall-based timeout: abort only if truly stuck for STALL_TIMEOUT consecutive windows
        STALL_TIMEOUT_COUNT = 4   # 4 consecutive stall windows (4×5s = 20s stuck) → abort
        ABSOLUTE_MAX_TIME = 180.0 # safety net: 3 minutes absolute max
        control_rate = rospy.Rate(25)  # 25Hz
        
        rospy.loginfo(f"Starting towing loop (stall abort after {STALL_TIMEOUT_COUNT} consecutive stalls, "
                      f"absolute max: {ABSOLUTE_MAX_TIME:.0f}s)...")
        
        # PI compensator REMOVED: testing showed it was counterproductive.
        # When EE is below target (z_err>0), integral saturates negative, pushes Z target UP,
        # UAV spends control effort climbing instead of pushing horizontally → can't tow.
        # Z-axis stabilization is handled by C++ Z-PID (p=8, d=5) alone.
        # Future: use ff_inter_wrench topic to provide proper feedforward.
        
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
            
            # Debug: log actual vs target position every 0.5s
            if int(elapsed * 2) != int((elapsed - 0.04) * 2):
                rospy.loginfo(f"[Towing Pos] actual={current_pos}, start={start_pos}")
            
            # Check completion
            if trajectory_gen.is_complete():
                rospy.loginfo(f"Towing complete! Distance: {state_info['current_distance']*1000:.0f}mm")
                break
            
            # Timeout: stall-based (consecutive stall windows) + absolute safety net
            if state_info['stall_counter'] >= STALL_TIMEOUT_COUNT:
                rospy.logwarn(f"Towing aborted: stalled for {state_info['stall_counter']} "
                             f"consecutive windows ({state_info['stall_counter']*5}s no progress)")
                self._clear_external_wrench()
                userdata.towing_end_position = current_pos
                self.formation_adapter.set_pitch_compensation(False)
                return 'timeout'
            if elapsed > ABSOLUTE_MAX_TIME:
                rospy.logwarn(f"Towing absolute timeout after {elapsed:.1f}s")
                self._clear_external_wrench()
                userdata.towing_end_position = current_pos
                self.formation_adapter.set_pitch_compensation(False)
                return 'timeout'
            
            # Stall warning (throttled to avoid log spam)
            if state_info['stall_counter'] > 0:
                rospy.logwarn_throttle(5.0, f"Towing stalled (window {state_info['stall_counter']}/{STALL_TIMEOUT_COUNT})")
            
            # Generate and execute target
            target_state = trajectory_gen.generate_target_state(maintain_yaw)
            
            # Debug: log position and pitch every 0.5s
            if int(elapsed * 2) != int((elapsed - 0.04) * 2):
                rpy_result = self.beetle.getAssemblyRPY()
                pitch_deg = np.degrees(rpy_result[1]) if rpy_result is not None else 0.0
                raw_z_err = target_state['position'][2] - current_pos[2]
                z_comp = target_state['force'][2]
                z_offset = state_info.get('z_offset', 0.0)
                rospy.loginfo(f"[Towing Debug] target_pos={target_state['position']}, "
                             f"z_err={raw_z_err*1000:.1f}mm, pitch={pitch_deg:.2f}°, "
                             f"z_comp={z_comp:.3f}N, z_offset={z_offset*1000:.1f}mm")
            
            self.send_assembly_command_from_end_effector(
                target_state['position'],
                target_state['yaw'],
                linear_vel=target_state['linear_velocity']
            )
            
            # ---- Publish desired external wrench via BeetleInterface ----
            # addExternalWrench() handles world→body rotation and unified/LF topic routing.
            ff_world = target_state['force']
            self.beetle.addExternalWrench(force=ff_world, torque=[0.0, 0.0, 0.0])
            
            # Debug: log ff force and progress every 0.5s
            if int(elapsed * 2) != int((elapsed - 0.04) * 2):
                rospy.loginfo(f"[Towing FF] ff_world=({ff_world[0]:.2f},{ff_world[1]:.2f},{ff_world[2]:.2f})N, "
                             f"mag={np.linalg.norm(ff_world):.2f}N, progress={state_info['progress']*100:.1f}%, "
                             f"mode={'unified' if self.beetle.isUnifiedMode() else 'LF'}")
            
            # Log progress every 5s
            if int(elapsed) % 5 == 0 and int(elapsed * 10) % 50 == 0:
                rospy.loginfo(f"Towing: {state_info['progress']*100:.1f}%, "
                             f"dist={state_info['current_distance']*1000:.0f}mm, "
                             f"ff={state_info['current_force']:.1f}N, time={elapsed:.1f}s")
            
            control_rate.sleep()
        
        # ---- Clear feedforward after towing completes ----
        self._clear_external_wrench()
        
        # Ensure pitch compensation stays disabled (already disabled, but defensive)
        self.formation_adapter.set_pitch_compensation(False)
        
        final_pos = self.get_end_effector_position()
        userdata.towing_end_position = final_pos
        
        rospy.loginfo(f"Towing phase complete at {FormationUtils.format_vec(final_pos)}")
        return 'succeeded'


class DisengageAndReturnState(TowingStateBase):
    """Disengage from load and return to start position."""
    
    def __init__(self):
        TowingStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['start_position', 'start_yaw', 'towing_end_position', 'towing_direction'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Disengage and Return State ===")
        
        start_pos = userdata.start_position
        start_yaw = userdata.start_yaw
        
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        
        if current_pos is None:
            rospy.logerr("Cannot get current position")
            return 'failed'
        
        # Phase 0: Brief stabilization after load release
        rospy.loginfo("[Phase 0] Stabilizing after load release")
        
        stabilize_duration = 3.0
        stabilize_start = rospy.Time.now()
        
        while (rospy.Time.now() - stabilize_start).to_sec() < stabilize_duration:
            current_pos = self.get_end_effector_position()
            if current_pos:
                self.send_assembly_command_from_end_effector(current_pos, current_yaw)
            
            elapsed = (rospy.Time.now() - stabilize_start).to_sec()
            rpy = self.beetle.getAssemblyRPY()
            if rpy is not None:
                rospy.loginfo_throttle(0.5, f"[Phase 0] t={elapsed:.1f}s, "
                    f"roll={np.degrees(rpy[0]):.2f}°, pitch={np.degrees(rpy[1]):.2f}°, "
                    f"pos={FormationUtils.format_vec(current_pos)}")
            rospy.sleep(0.1)
        
        rospy.loginfo(f"[Phase 0] Stabilization complete after {stabilize_duration}s")
        
        # DEBUG: log position drift to confirm PID integral residual
        post_stab_pos = self.get_end_effector_position()
        if post_stab_pos and current_pos:
            drift = np.array(post_stab_pos) - np.array(current_pos)
            rospy.loginfo(f"[DEBUG] Position drift during stabilization: "
                         f"({drift[0]*1000:.1f}, {drift[1]*1000:.1f}, {drift[2]*1000:.1f})mm")

        # Phase 1: Ascend to start height to fully clear the box
        # The fang may still be hooked on the box wall; ascending to the
        # original start_pos height guarantees physical clearance.
        current_pos = self.get_end_effector_position()
        safe_height = start_pos[2]
        rospy.loginfo(f"[Phase 1] Ascending to start height {safe_height:.3f}m "
                      f"(current {current_pos[2]:.3f}m, delta {(safe_height - current_pos[2])*1000:.0f}mm)")
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
        
        # Use streaming polynomial trajectory for smooth return
        traj_desc = self.generate_polynomial_trajectory(
            start_pos=current_pos,
            target_pos=return_target,
            target_yaw=start_yaw,
            lock_yaw=True
        )
        
        if traj_desc:
            self.execute_polynomial_trajectory(traj_desc)
        
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
    rospy.loginfo(f"Max towing force: {TOWING_MAX_FORCE}N (adaptive from 0N)")
    rospy.loginfo("=" * 60)
    
    try:
        # Create SMACH state machine
        sm = smach.StateMachine(outcomes=['success', 'failure'])
        
        with sm:
            # Step 1: Initialize towing task
            # (Assembly and mode switching done manually before launching this script)
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
