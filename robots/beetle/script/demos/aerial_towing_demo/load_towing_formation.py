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
import rosgraph
import smach
import smach_ros
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import UInt8
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
from beetle_interface import BeetleInterface, smoothstep01
from trajectory import PolynomialTrajectory


# ============== Load Box Parameters (measured actual box) ==============
LOAD_BOX_LENGTH = 0.88   # x dimension (m)
LOAD_BOX_WIDTH = 0.49    # y dimension (m)
LOAD_BOX_HEIGHT = 0.86   # z dimension (m)
LOAD_WALL_THICKNESS = 0.02  # wall thickness (m)

# ============== Towing Task Parameters ==============
RETRACT_DISTANCE = 0.03  # 30mm retract to hook edge (wall_thickness + margin)
HOOK_POSITION_TOLERANCE = 0.030  # 30mm tolerance for hook convergence (formation control noise)
TOWING_DISTANCE = 0.6    # towing distance (m), overridden by ~towing_distance param
TOWING_MAX_FORCE = 5.0   # max adaptive force (N), overridden by ~towing_force param
# Approach height above box top for entering the load area.
# Default is 50mm. Tune as needed; with box_top_z=0.65m:
# - offset 0.15 -> approach 0.80m
# - offset 0.18 -> approach 0.83m
APPROACH_HEIGHT_OFFSET = 0.18
# XY-transit clearance above box top - used as a SAFETY FLOOR for the
# horizontal-transit phase and the disengage-ascent phase. Acts as a hard lower
# bound on the transit Z target so even if takeoff_height is set low or the EE
# drifts down during XY motion, the formation still clears the box.
TRANSIT_CLEARANCE_OFFSET = 0.30

# Feedforward wrench should be shifted from the virtual EE reference to the
# actual hook contact point. From beetle_fang.urdf.xacro:
#   virtual EE reference: x=0.246m, z=0.074382m
#   hook bottom center:   x=0.246-0.02404m, z=0.0113823-0.0718283-0.0025m
# The two hooks are symmetric in Y, so the equivalent towing contact uses y=0.
TOWING_HOOK_CONTACT_DX_FROM_EE = -0.02404
TOWING_HOOK_CONTACT_DZ_FROM_EE = -0.137328
# Final insertion is defined by the hook contact point, not by virtual EE descent.
# Positive clearance keeps the hook contact above the box top; negative means
# intentionally inserting below the box top.
HOOK_CONTACT_INSERT_CLEARANCE = 0.02
# Only split the descent when the final hook-insertion segment is long enough
# to be meaningful. Shorter insertions are handled in one continuous descent.
MIN_SPLIT_INSERTION_DROP = 0.10

# Unified QP task weights [Fx,Fy,Fz,Tx,Ty,Tz]. Towing primarily tracks
# horizontal pull force. The r x F torque from the hook offset is kept as a
# soft feedforward bias, with very low yaw-torque authority to protect heading.
TOWING_TASK_WRENCH_WEIGHTS = [2.0, 2.0, 0.0, 0.12, 0.12, 0.02]


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
        self.current_load_distance = 0.0
        self.current_velocity = 0.0
        self.target_pos = self.start_pos.copy()
        self.load_start_pos = None

        # Velocity ramp-up parameters
        self.ramp_up_distance = 0.05  # 50mm ramp-up zone
        self.ramp_down_distance = 0.05  # 50mm ramp-down zone

        # Max lead distance: target position must not exceed actual EE position
        # by more than this amount along towing direction, preventing PID saturation
        # when the load is stuck or slow.
        self.max_lead_distance = 0.17  # 170mm
        self.max_lag_distance = 0.08   # 80mm

        # Force adaptation: S-curve ramp over 3 stall windows.
        self.current_force = 0.0
        self.force_locked = False
        self.force_ramp_progress = 0.0
        self.breakaway_detected = False
        self.breakaway_relief_applied = False
        self.breakaway_force_ratio = 0.65
        self.breakaway_distance = 0.05  # 50mm load motion confirms contact release
        self.breakaway_velocity = self.target_velocity * 0.2
        self.last_motion_distance = 0.0
        self.motion_velocity = 0.0

        # Performance monitoring
        self.start_time = rospy.Time.now().to_sec()
        self.last_update_time = self.start_time
        self.stall_counter = 0

        # Sliding window for stall-based abort detection
        self.stall_window_time = 5.0    # seconds per window
        self.stall_last_check_time = self.start_time
        self.stall_last_check_distance = 0.0

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

        # Calculate current end-effector distance traveled
        displacement = np.array(current_pos) - self.start_pos
        self.current_distance = np.dot(displacement[:2], self.towing_direction[:2])

        # Track load displacement when available (primary towing success metric)
        if load_pos is not None:
            load_pos_arr = np.array(load_pos)
            if self.load_start_pos is None:
                self.load_start_pos = load_pos_arr.copy()
                self.stall_last_check_time = current_time
                self.stall_last_check_distance = 0.0
                rospy.loginfo(f"[Towing] Load reference locked at {self.load_start_pos}")
            load_displacement = load_pos_arr - self.load_start_pos
            self.current_load_distance = np.dot(load_displacement[:2], self.towing_direction[:2])

        # Prefer load displacement for progress/completion when available.
        motion_distance = self.current_load_distance if self.load_start_pos is not None else self.current_distance
        motion_delta = motion_distance - self.last_motion_distance
        self.motion_velocity = motion_delta / max(safe_dt, 1e-3)
        self.last_motion_distance = motion_distance

        # Velocity profile with ramp-up and ramp-down
        remaining_distance = self.target_distance - motion_distance

        if motion_distance < self.ramp_up_distance:
            # Ramp up phase
            velocity_factor = motion_distance / self.ramp_up_distance
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

        # Clamp target so it never leads actual EE position by more than max_lead_distance.
        # This prevents PID saturation when the load is stuck or slow.
        new_target_distance = min(new_target_distance,
                                  self.current_distance + self.max_lead_distance)
        # Also keep the target from lagging too far behind the EE after breakaway.
        # Otherwise position PID fights the towing feedforward while the load is moving.
        new_target_distance = max(new_target_distance,
                                  self.current_distance - self.max_lag_distance)
        new_target_distance = min(self.target_distance, max(0.0, new_target_distance))

        self.target_pos[:2] = self.start_pos[:2] + self.towing_direction[:2] * new_target_distance

        # Stall detection using sliding window (for abort decision only)
        window_elapsed = current_time - self.stall_last_check_time
        if window_elapsed >= self.stall_window_time:
            window_distance = motion_distance - self.stall_last_check_distance
            recent_velocity = window_distance / window_elapsed
            self._recent_velocity = recent_velocity
            self.stall_last_check_time = current_time
            self.stall_last_check_distance = motion_distance

            if recent_velocity < self.target_velocity * 0.05:  # < 5% of target
                self.stall_counter += 1  # increments once per window
            else:
                self.stall_counter = max(0, self.stall_counter - 1)

        # Adaptive force: ease in while stuck, then unload once the load breaks away.
        if (not self.breakaway_detected and self.load_start_pos is not None and
                motion_distance >= self.breakaway_distance and
                self.motion_velocity >= self.breakaway_velocity):
            self.breakaway_detected = True

        if self.breakaway_detected and not self.breakaway_relief_applied:
            prev_force = self.current_force
            self.current_force = prev_force * self.breakaway_force_ratio
            self.force_locked = True
            self.breakaway_relief_applied = True
            rospy.loginfo(f"[Towing] Load breakaway detected: dist={motion_distance*1000:.0f}mm, "
                         f"vel={self.motion_velocity*1000:.0f}mm/s, "
                         f"ff {prev_force:.1f}N -> {self.current_force:.1f}N")
        elif not self.force_locked:
            ramp_time = 3.0 * self.stall_window_time  # 15s
            self.force_ramp_progress = min(
                1.0, self.force_ramp_progress + 1.0 / (ramp_time * self.control_rate))
            self.current_force = self.max_force * smoothstep01(self.force_ramp_progress)

        return {
            'current_distance': self.current_distance,
            'current_load_distance': self.current_load_distance,
            'target_distance': self.target_distance,
            'current_velocity': self.current_velocity,
            'current_force': self.current_force,
            'motion_velocity': self.motion_velocity,
            'breakaway_detected': self.breakaway_detected,
            'progress': motion_distance / self.target_distance,
            'stall_counter': self.stall_counter,
            'using_load_tracking': self.load_start_pos is not None
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

        # Force feedforward: world-horizontal force in towing direction.
        # The state executor maps this to the frame required by each control mode.
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
        distance_for_completion = self.current_load_distance if self.load_start_pos is not None else self.current_distance
        return distance_for_completion >= self.target_distance * 0.95

    def get_progress(self):
        """Get towing progress [0.0, 1.0]."""
        distance_for_progress = self.current_load_distance if self.load_start_pos is not None else self.current_distance
        return min(1.0, distance_for_progress / self.target_distance)


class LoadInterface:
    """Interface for load box position tracking (simulation and real machine)."""

    def __init__(self):
        self.load_pos = None
        self.load_yaw = 0.0
        self.position_received = threading.Event()

        is_simulation = rospy.get_param("~simulation", True)
        if is_simulation:
            rospy.Subscriber('/load/odom', Odometry, self._load_sim_cb, queue_size=1)
        else:
            rospy.Subscriber('/load/mocap/pose', PoseStamped, self._load_cb, queue_size=1)
        rospy.loginfo(f"LoadInterface: mode={'simulation' if is_simulation else 'real_machine'}")

    def _load_cb(self, msg):
        """Callback for real machine (PoseStamped from mocap)."""
        pos = msg.pose.position
        ori = msg.pose.orientation
        self.load_pos = np.array([pos.x, pos.y, pos.z])
        self.load_yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        if not self.position_received.is_set():
            rospy.loginfo(f"Load position received: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}), yaw={math.degrees(self.load_yaw):.1f} deg")
            self.position_received.set()

    def _load_sim_cb(self, msg):
        """Callback for simulation (Odometry from Gazebo)."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.load_pos = np.array([pos.x, pos.y, pos.z])
        self.load_yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        if not self.position_received.is_set():
            rospy.loginfo(f"Load position received: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}), yaw={math.degrees(self.load_yaw):.1f} deg")
            self.position_received.set()

    def get_load_position(self):
        return self.load_pos

    def get_load_yaw(self):
        return self.load_yaw

    def get_load_top_z(self):
        if self.load_pos is not None:
            return self.load_pos[2] + LOAD_BOX_HEIGHT / 2
        return None

    def wait_for_load(self, timeout=10.0):
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
    _debug_subscribers = []
    _debug_module_ids = set()
    _debug_lock = threading.Lock()
    _last_four_axis_stamp = {}
    _last_flight_state_stamp = {}
    _last_flight_state = {}
    _last_config_ack_stamp = {}
    _last_config_ack = {}

    def __init__(self, outcomes, input_keys=None, output_keys=None):
        FormationSingleUAVStateBase.__init__(self, outcomes, input_keys, output_keys)

        # Initialize shared load interface
        if TowingStateBase._shared_load_interface is None:
            TowingStateBase._shared_load_interface = LoadInterface()
        self.load_interface = TowingStateBase._shared_load_interface
        self._init_module_debug_monitors()

    def _init_module_debug_monitors(self):
        """Track command and force-landing heartbeat during insertion debug."""
        with TowingStateBase._debug_lock:
            missing_ids = [
                module_id for module_id in self.formation_adapter.module_ids
                if module_id not in TowingStateBase._debug_module_ids
            ]
            for module_id in missing_ids:
                TowingStateBase._debug_subscribers.extend([
                    rospy.Subscriber(f"/beetle{module_id}/four_axes/command",
                                     rospy.AnyMsg, self._four_axis_debug_cb,
                                     callback_args=module_id, queue_size=1),
                    rospy.Subscriber(f"/beetle{module_id}/flight_state",
                                     UInt8, self._flight_state_debug_cb,
                                     callback_args=module_id, queue_size=1),
                    rospy.Subscriber(f"/beetle{module_id}/flight_config_ack",
                                     UInt8, self._config_ack_debug_cb,
                                     callback_args=module_id, queue_size=1),
                ])
                TowingStateBase._debug_module_ids.add(module_id)

    def _four_axis_debug_cb(self, _msg, module_id):
        with TowingStateBase._debug_lock:
            TowingStateBase._last_four_axis_stamp[module_id] = rospy.Time.now().to_sec()

    def _flight_state_debug_cb(self, msg, module_id):
        with TowingStateBase._debug_lock:
            TowingStateBase._last_flight_state[module_id] = msg.data
            TowingStateBase._last_flight_state_stamp[module_id] = rospy.Time.now().to_sec()

    def _config_ack_debug_cb(self, msg, module_id):
        with TowingStateBase._debug_lock:
            TowingStateBase._last_config_ack[module_id] = msg.data
            TowingStateBase._last_config_ack_stamp[module_id] = rospy.Time.now().to_sec()

    def format_module_debug_status(self):
        now = rospy.Time.now().to_sec()

        def age_text(stamp, stale_limit=0.5):
            if stamp is None:
                return "NA"
            age = now - stamp
            suffix = "!" if age > stale_limit else ""
            return f"{age:.2f}s{suffix}"

        with TowingStateBase._debug_lock:
            parts = []
            for module_id in self.formation_adapter.module_ids:
                cmd_stamp = TowingStateBase._last_four_axis_stamp.get(module_id)
                state_stamp = TowingStateBase._last_flight_state_stamp.get(module_id)
                ack_stamp = TowingStateBase._last_config_ack_stamp.get(module_id)
                state = TowingStateBase._last_flight_state.get(module_id, "NA")
                ack = TowingStateBase._last_config_ack.get(module_id, "NA")
                parts.append(
                    f"b{module_id}:cmd_age={age_text(cmd_stamp)} "
                    f"state={state}/age={age_text(state_stamp)} "
                    f"ack={ack}/age={age_text(ack_stamp, stale_limit=2.0)}"
                )
        return " module_debug=[" + "; ".join(parts) + "]"

    def log_module_debug_status(self, prefix):
        rospy.loginfo(f"{prefix}{self.format_module_debug_status()}")

    def get_load_position(self):
        """Get current load position."""
        return self.load_interface.get_load_position()

    def get_load_top_z(self):
        """Get load box top Z coordinate."""
        return self.load_interface.get_load_top_z()

    def calculate_approach_side(self, assembly_pos, load_pos):
        """
        Calculate which side of the load to approach (nearest edge midpoint).
        Accounts for load yaw: edges and approach directions are rotated accordingly.

        Returns:
            tuple: (approach_direction_vector, side_name, edge_midpoint_position)
        """
        if assembly_pos is None or load_pos is None:
            return None, None, None

        load_yaw = self.load_interface.get_load_yaw()
        cos_y, sin_y = math.cos(load_yaw), math.sin(load_yaw)

        def rotate_2d(v):
            return np.array([cos_y*v[0] - sin_y*v[1], sin_y*v[0] + cos_y*v[1], v[2]])

        half_length = LOAD_BOX_LENGTH / 2
        half_width = LOAD_BOX_WIDTH / 2

        # Local frame edge offsets and outward directions, rotated by load_yaw
        local_edges = {
            '+X': np.array([half_length, 0, 0]),
            '-X': np.array([-half_length, 0, 0]),
            '+Y': np.array([0, half_width, 0]),
            '-Y': np.array([0, -half_width, 0]),
        }
        local_dirs = {
            '+X': np.array([1, 0, 0]),
            '-X': np.array([-1, 0, 0]),
            '+Y': np.array([0, 1, 0]),
            '-Y': np.array([0, -1, 0]),
        }

        edge_midpoints = {side: load_pos + rotate_2d(off) for side, off in local_edges.items()}
        approach_dirs = {side: rotate_2d(d) for side, d in local_dirs.items()}

        distances = {side: np.linalg.norm(np.array(assembly_pos) - midpoint)
                     for side, midpoint in edge_midpoints.items()}

        nearest_side = min(distances.keys(), key=lambda k: distances[k])
        edge_pos = edge_midpoints[nearest_side]
        approach_dir = approach_dirs[nearest_side]

        rospy.loginfo(f"Approach side: {nearest_side}, direction: {approach_dir}, edge: {edge_pos}")
        if abs(load_yaw) > 0.01:
            rospy.loginfo(f"Load yaw: {math.degrees(load_yaw):.1f} deg (edges rotated)")
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

        # Store in userdata (use EE frame - all movement commands operate in EE coordinates)
        userdata.start_position = self.get_end_effector_position()
        userdata.start_yaw = self.get_end_effector_yaw()
        userdata.load_position = load_pos
        userdata.approach_direction = approach_dir
        userdata.approach_side = side_name
        userdata.edge_position = edge_pos

        rospy.loginfo(f"Assembly position: {FormationUtils.format_vec(assembly_pos)}")
        rospy.loginfo(f"Assembly yaw: {math.degrees(assembly_yaw):.1f} deg")
        rospy.loginfo(f"Load position: {FormationUtils.format_vec(load_pos)}")
        rospy.loginfo(f"Approach side: {side_name}")

        return 'succeeded'


class ApproachLoadState(TowingStateBase):
    """Move end-effector to position above load edge, slightly over the box."""

    def __init__(self):
        TowingStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['load_position', 'approach_direction', 'approach_side', 'edge_position'],
            output_keys=['approach_position', 'insertion_position', 'insertion_yaw'])

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
        insertion_z = (load_top_z + HOOK_CONTACT_INSERT_CLEARANCE -
                       TOWING_HOOK_CONTACT_DZ_FROM_EE)
        approach_hook_clearance = approach_height + TOWING_HOOK_CONTACT_DZ_FROM_EE - load_top_z

        # Position slightly inside the box edge (overshoot by 110mm for insertion)
        overshoot = 0.11  # 110mm inside box edge
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
        rospy.loginfo(
            f"Hook contact clearance: approach={approach_hook_clearance*1000:.0f}mm, "
            f"insert={HOOK_CONTACT_INSERT_CLEARANCE*1000:.0f}mm above box top"
        )
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f} deg")

        # Phase 1: XY movement (Z = box-anchored safety floor or current Z, whichever higher)
        transit_z = max(current_pos[2], load_top_z + TRANSIT_CLEARANCE_OFFSET)
        rospy.loginfo(f"[Phase 1] Moving to XY position above box "
                      f"(transit_z={transit_z:.3f}m, floor=box_top+{TRANSIT_CLEARANCE_OFFSET:.2f}m)")
        phase1_target = np.array([approach_xy[0], approach_xy[1], transit_z])

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
        rospy.loginfo(f"[Phase 2] Adjusting yaw to {math.degrees(target_yaw):.1f} deg")
        current_pos = self.get_end_effector_position()

        success = self.active_position_convergence(
            current_pos, target_yaw=target_yaw,
            pos_thresh=0.05, yaw_thresh=0.05, timeout=15.0,
            yaw_only=True
        )
        if not success:
            rospy.logwarn("Phase 2 yaw adjustment incomplete, continuing...")

        # DESCEND_AND_INSERT conditionally splits at this approach height. If the
        # final hook insertion would be too short, it descends directly instead.
        approach_pos = np.array([approach_xy[0], approach_xy[1], approach_height])
        insertion_pos = np.array([approach_xy[0], approach_xy[1], insertion_z])
        userdata.insertion_position = insertion_pos
        userdata.approach_position = approach_pos
        userdata.insertion_yaw = target_yaw

        final_insert_drop = max(0.0, approach_height - insertion_z)
        rospy.loginfo(
            f"Insertion plan: approach_pos={FormationUtils.format_vec(approach_pos)}, "
            f"insert_pos={FormationUtils.format_vec(insertion_pos)}, "
            f"final_insert_drop={final_insert_drop*1000:.1f}mm "
            f"(split_min={MIN_SPLIT_INSERTION_DROP*1000:.0f}mm)"
        )

        rospy.loginfo("Approach complete")
        return 'succeeded'


class DescendAndInsertState(TowingStateBase):
    """Descend to insert end-effector into the load box."""

    def __init__(self):
        TowingStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['approach_position', 'insertion_position', 'insertion_yaw',
                        'approach_direction', 'load_position'],
            output_keys=['hook_position', 'hook_yaw'])

    def execute(self, userdata):
        rospy.loginfo("=== Descend and Insert State ===")

        insertion_pos = userdata.insertion_position
        insertion_yaw = userdata.insertion_yaw
        approach_pos = userdata.approach_position
        load_top_z = float(userdata.load_position[2]) + LOAD_BOX_HEIGHT / 2

        rospy.loginfo(f"Approach-height target: {FormationUtils.format_vec(approach_pos)}")
        rospy.loginfo(f"Insertion target: {FormationUtils.format_vec(insertion_pos)}")
        final_insert_drop = max(0.0, approach_pos[2] - insertion_pos[2])
        rospy.loginfo(
            f"Final hook insertion drop: {final_insert_drop*1000:.1f}mm "
            f"(split_min={MIN_SPLIT_INSERTION_DROP*1000:.0f}mm)"
        )
        rospy.loginfo(
            f"Hook contact target clearance: {HOOK_CONTACT_INSERT_CLEARANCE*1000:.0f}mm above box top "
            f"(box_top={load_top_z:.3f}m, hook_dz_from_ee={TOWING_HOOK_CONTACT_DZ_FROM_EE:.3f}m)"
        )

        current_pos = self.get_end_effector_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position")
            return 'failed'

        if final_insert_drop < MIN_SPLIT_INSERTION_DROP:
            rospy.loginfo("[Insertion] Final insertion segment is short; using one continuous descent")
            self.log_module_debug_status("[Insertion Debug] before single insertion descent")
            success, achieved_pos = self.streaming_z_descent(
                current_pos,
                insertion_pos,
                insertion_yaw,
                descent_speed=0.05
            )
        else:
            self.log_module_debug_status("[Insertion Debug] before approach descent")

            # Stage 1: descend from transit height to load-anchored approach height.
            success, achieved_pos = self.streaming_z_descent(
                current_pos,
                approach_pos,
                insertion_yaw,
                descent_speed=0.05
            )

            if not success:
                rospy.logerr("Approach-height descent failed (aborted). Exiting state machine.")
                self.log_module_debug_status("[Insertion Debug] approach descent failed")
                return 'failed'

            self.log_module_debug_status("[Insertion Debug] before approach recenter")
            recentered = self.active_position_convergence(
                approach_pos, target_yaw=insertion_yaw,
                pos_thresh=0.035, yaw_thresh=0.05, timeout=10.0,
                max_linear_vel=0.04
            )
            if not recentered:
                rospy.logwarn("Approach-height recenter incomplete; continuing to final insertion.")
                self.log_module_debug_status("[Insertion Debug] approach recenter incomplete")

            current_pos = self.get_end_effector_position()
            if current_pos is None:
                rospy.logerr("Cannot get current position before final insertion")
                return 'failed'

            # Stage 2: final insertion based on hook contact clearance.
            self.log_module_debug_status("[Insertion Debug] before final insertion")
            success, achieved_pos = self.streaming_z_descent(
                current_pos,
                insertion_pos,
                insertion_yaw,
                descent_speed=0.03
            )

        if not success:
            # If descent aborted (e.g., retry limit hit), don't continue with hook/tow.
            rospy.logerr("Z descent failed (aborted). Exiting state machine.")
            self.log_module_debug_status("[Insertion Debug] insertion descent failed")
            return 'failed'

        # Verify achieved hook contact clearance relative to the load top.
        if achieved_pos is not None:
            achieved_clearance = achieved_pos[2] + TOWING_HOOK_CONTACT_DZ_FROM_EE - load_top_z
            rospy.loginfo(
                f"Achieved hook contact clearance: {achieved_clearance*1000:.1f}mm "
                f"(target={HOOK_CONTACT_INSERT_CLEARANCE*1000:.1f}mm, positive=above box top)"
            )

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
        # At pitch -1.5 deg, body-X force (5N) couples ~131mN into world-Z through rotation.
        # This Z component flows: wrench_comp -> I_reconfig_acc -> pid_FZ -> I_comp_Fz ->
        # pid_Z.setICompTerm -> monotonic Z-PID I-term accumulation -> upward drift.
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

        Smoothly unloads the last command, then sends a few zero-wrench
        messages for reliability. Topic routing is handled by BeetleInterface.
        """
        rospy.loginfo("Clearing external wrench feedforward")
        zero = [0.0, 0.0, 0.0]
        self.beetle.clearExternalWrench(duration=1.0, rate_hz=25.0)
        for _ in range(3):
            self.beetle.addExternalWrench(zero, zero, frame_id="fc")
            rospy.sleep(0.04)
        self.beetle.clearExternalWrench()
        # Disable internal-wrench auto-publish (also broadcasts a final zero).
        self.beetle.setAttachModule(None)

    def _build_towing_wrench_command(self, force_world, unified_mode):
        """Return (force, torque, frame_id) for BeetleInterface.addExternalWrench()."""
        if not unified_mode:
            return force_world, [0.0, 0.0, 0.0], "world_yaw"

        contact_offset_body = np.array([
            self.formation_adapter.base_offset_x + TOWING_HOOK_CONTACT_DX_FROM_EE,
            self.formation_adapter.base_offset_y,
            self.formation_adapter.total_offset_z + TOWING_HOOK_CONTACT_DZ_FROM_EE,
        ])
        # Keep towing feedforward horizontal in the formation-yaw frame. Current
        # pitch/roll are tracking errors and should not create body-Z force.
        force_body, torque_body = self.beetle.buildFormationCoGWrench(
            force_world, application_offset_body=contact_offset_body,
            yaw_only=True)
        return force_body, torque_body, "fc"

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
        # Unified allocation expects a formation-body wrench at the CoG. LF keeps
        # the existing yaw-only path because wrench_comp reprojects body-yaw force.
        control_mode = 'unified' if self.beetle.isUnifiedMode() else 'leader-follower'
        rospy.loginfo(f"Wrench feedforward via BeetleInterface (mode: {control_mode})")

        # ---- Per-module observer task prediction auto-publish ----
        # Declare which module physically carries the towing hook (= the EE
        # module on which BeetleInterface is instantiated). BeetleInterface
        # will then publish per-module y_hat^task = (m_i/m_total) * W_ext on
        # /beetle{i}/est_wrench_task. The C++ controller subtracts this from
        # the raw observer output BEFORE the inter-wrench recursion, so both
        # unified-mode diff damping and LF wrench_comp cascade consume the
        # parasitic residual only - the task-induced load distribution is
        # not cancelled or fed back into formation acceleration.
        module_masses = rospy.get_param("~module_masses", None)
        module_positions = rospy.get_param("~module_positions", None)
        module_inertias_diag = rospy.get_param("~module_inertias_diag", None)
        attach_ok = self.beetle.setAttachModule(self.beetle.module_id,
                                                module_masses=module_masses,
                                                module_positions=module_positions,
                                                module_inertias_diag=module_inertias_diag)
        if not attach_ok and control_mode == 'leader-follower':
            rospy.logwarn("[Towing] LF task feedforward is disabled because setAttachModule failed")

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
        STALL_TIMEOUT_COUNT = 4   # 4 consecutive stall windows (4x5s = 20s stuck) -> abort
        ABSOLUTE_MAX_TIME = 180.0 # safety net: 3 minutes absolute max
        control_rate = rospy.Rate(25)  # 25Hz
        unified_mode_seen = False

        rospy.loginfo(f"Starting towing loop (stall abort after {STALL_TIMEOUT_COUNT} consecutive stalls, "
                      f"absolute max: {ABSOLUTE_MAX_TIME:.0f}s)...")

        # PI compensator REMOVED: testing showed it was counterproductive.
        # When EE is below target (z_err>0), integral saturates negative, pushes Z target UP,
        # UAV spends control effort climbing instead of pushing horizontally -> can't tow.
        # Z-axis stabilization is handled by C++ Z-PID (p=8, d=5) alone.

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
                done_dist = state_info['current_load_distance'] if state_info['using_load_tracking'] else state_info['current_distance']
                done_source = 'load' if state_info['using_load_tracking'] else 'ee'
                rospy.loginfo(f"Towing complete! Distance: {done_dist*1000:.0f}mm ({done_source})")
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
            unified_mode = self.beetle.isUnifiedMode()
            if unified_mode:
                unified_mode_seen = True

            rpy_result = self.beetle.getAssemblyRPY()
            pitch_deg = np.degrees(rpy_result[1]) if rpy_result is not None else 0.0
            raw_z_err = target_state['position'][2] - current_pos[2]

            if unified_mode_seen and not unified_mode:
                rospy.logwarn("Towing safety abort: unified mode exited during towing")
                self._clear_external_wrench()
                userdata.towing_end_position = current_pos
                self.formation_adapter.set_pitch_compensation(False)
                return 'timeout'

            # Debug: log position and pitch every 0.5s
            if int(elapsed * 2) != int((elapsed - 0.04) * 2):
                rospy.loginfo(f"[Towing Debug] target_pos={target_state['position']}, "
                             f"z_err={raw_z_err*1000:.1f}mm, pitch={pitch_deg:.2f} deg")

            self.send_assembly_command_from_end_effector(
                target_state['position'],
                target_state['yaw'],
                linear_vel=target_state['linear_velocity']
            )

            # ---- Publish desired external wrench via BeetleInterface ----
            ff_world = target_state['force']
            ff_force, ff_torque, ff_frame = self._build_towing_wrench_command(
                ff_world, unified_mode)
            self.beetle.addExternalWrench(force=ff_force, torque=ff_torque,
                                          frame_id=ff_frame,
                                          task_weights=TOWING_TASK_WRENCH_WEIGHTS if unified_mode else None)

            # Debug: log ff force and progress every 0.5s
            if int(elapsed * 2) != int((elapsed - 0.04) * 2):
                rospy.loginfo(f"[Towing FF] ff_world=({ff_world[0]:.2f},{ff_world[1]:.2f},{ff_world[2]:.2f})N, "
                             f"mag={np.linalg.norm(ff_world):.2f}N, progress={state_info['progress']*100:.1f}%, "
                             f"mode={'unified' if unified_mode else 'LF'}, "
                             f"cmd_frame={ff_frame}, tau=({ff_torque[0]:.2f},{ff_torque[1]:.2f},{ff_torque[2]:.2f})Nm")

            # Log progress every 5s
            if int(elapsed) % 5 == 0 and int(elapsed * 10) % 50 == 0:
                dist_source = 'load' if state_info['using_load_tracking'] else 'ee'
                dist_value = state_info['current_load_distance'] if state_info['using_load_tracking'] else state_info['current_distance']
                rospy.loginfo(f"Towing: {state_info['progress']*100:.1f}%, "
                             f"dist={dist_value*1000:.0f}mm({dist_source}), "
                             f"ff={state_info['current_force']:.1f}N, time={elapsed:.1f}s")

            control_rate.sleep()

        # If loop ends due to ROS shutdown/Ctrl-C, this is not a successful towing completion.
        if rospy.is_shutdown():
            self._clear_external_wrench()
            userdata.towing_end_position = self.get_end_effector_position()
            self.formation_adapter.set_pitch_compensation(False)
            return 'timeout'

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
            input_keys=['start_position', 'start_yaw', 'towing_end_position',
                        'towing_direction', 'load_position'])

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
        # Hold a FIXED target position to prevent upward drift from pitch transient
        rospy.loginfo("[Phase 0] Stabilizing after load release")

        stabilize_duration = 3.0
        stabilize_start = rospy.Time.now()
        stabilize_target = current_pos  # fixed target, NOT updated each loop

        while (rospy.Time.now() - stabilize_start).to_sec() < stabilize_duration:
            self.send_assembly_command_from_end_effector(stabilize_target, current_yaw)

            elapsed = (rospy.Time.now() - stabilize_start).to_sec()
            actual_pos = self.get_end_effector_position()
            rpy = self.beetle.getAssemblyRPY()
            if rpy is not None and actual_pos is not None:
                rospy.loginfo_throttle(0.5, f"[Phase 0] t={elapsed:.1f}s, "
                    f"roll={np.degrees(rpy[0]):.2f} deg, pitch={np.degrees(rpy[1]):.2f} deg, "
                    f"pos={FormationUtils.format_vec(actual_pos)}")
            rospy.sleep(0.1)

        rospy.loginfo(f"[Phase 0] Stabilization complete after {stabilize_duration}s")

        # Phase 0.5: Retract along towing reverse direction to disengage hook
        towing_dir = np.array(userdata.towing_direction)
        retract_dir = -towing_dir  # reverse of towing = back into box, then past wall
        retract_distance = 0.13  # 130mm
        current_pos = self.get_end_effector_position()
        retract_target = np.array(current_pos) + retract_dir * retract_distance
        rospy.loginfo(f"[Phase 0.5] Retracting {retract_distance*1000:.0f}mm along "
                      f"{retract_dir} to disengage hook")

        success = self.active_position_convergence(
            retract_target, target_yaw=current_yaw,
            pos_thresh=0.03, yaw_thresh=0.1, timeout=10.0,
            max_linear_vel=0.03
        )
        rospy.sleep(0.5)

        # Phase 1: Return to start height (hook is already disengaged in Phase 0.5).
        # Apply box-anchored safety floor - same convention as ApproachLoadState
        # Phase 1 - so transit Z above the box is consistent across the whole task.
        current_pos = self.get_end_effector_position()
        load_top_z = float(userdata.load_position[2]) + LOAD_BOX_HEIGHT / 2
        target_height = max(start_pos[2], load_top_z + TRANSIT_CLEARANCE_OFFSET)
        rospy.loginfo(f"[Phase 1] Returning to start height {target_height:.3f}m "
                      f"(start_pos[2]={start_pos[2]:.3f}m, "
                      f"floor=box_top+{TRANSIT_CLEARANCE_OFFSET:.2f}m={load_top_z+TRANSIT_CLEARANCE_OFFSET:.3f}m, "
                      f"current {current_pos[2]:.3f}m, delta {(target_height - current_pos[2])*1000:.0f}mm)")
        ascent_target = (current_pos[0], current_pos[1], target_height)

        success = self.active_position_convergence(
            ascent_target, target_yaw=current_yaw,
            pos_thresh=0.05, yaw_thresh=0.1, timeout=20.0,
            max_linear_vel=0.05
        )

        rospy.sleep(1.0)

        # Phase 2: Adjust yaw to start yaw
        rospy.loginfo(f"[Phase 2] Adjusting yaw to {math.degrees(start_yaw):.1f} deg")
        current_pos = self.get_end_effector_position()

        success = self.active_position_convergence(
            current_pos, target_yaw=start_yaw,
            pos_thresh=0.1, yaw_thresh=0.05, timeout=30.0,
            max_yaw_step=0.05, yaw_only=True
        )

        rospy.sleep(1.0)

        # Phase 3: Return to start position (XY + original Z)
        rospy.loginfo("[Phase 3] Returning to start position")
        current_pos = self.get_end_effector_position()
        return_target = (start_pos[0], start_pos[1], start_pos[2])

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


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _parse_module_ids(module_ids_str):
    try:
        return [int(x.strip()) for x in str(module_ids_str).split(',') if x.strip()]
    except ValueError:
        rospy.logerr("[TowingPreflight] Invalid module_ids: %s", module_ids_str)
        return []


def _names_for_topic(system_state_entries, topic_name):
    for name, nodes in system_state_entries:
        if name == topic_name:
            return nodes
    return []


def log_towing_preflight(module_ids, real_machine, simulation):
    """Log master/module readiness without changing task behavior."""
    master_uri = os.environ.get('ROS_MASTER_URI', '<unset>')
    ros_ip = os.environ.get('ROS_IP', '<unset>')
    ros_hostname = os.environ.get('ROS_HOSTNAME', '<unset>')
    rospy.loginfo("[TowingPreflight] ROS_MASTER_URI=%s ROS_IP=%s ROS_HOSTNAME=%s",
                  master_uri, ros_ip, ros_hostname)
    rospy.loginfo("[TowingPreflight] module_ids=%s real_machine=%s simulation=%s",
                  module_ids, real_machine, simulation)
    if real_machine and ('localhost' in master_uri or '127.0.0.1' in master_uri):
        rospy.logwarn("[TowingPreflight] real_machine with local ROS_MASTER_URI; "
                      "this is only valid on the machine running the shared ROS master")
    if ros_ip != '<unset>' and ros_hostname != '<unset>':
        rospy.logwarn("[TowingPreflight] Both ROS_IP and ROS_HOSTNAME are set; "
                      "prefer setting only one to avoid advertised-address ambiguity")

    try:
        master = rosgraph.Master('/formation_load_towing_preflight')
        pubs, subs, srvs = master.getSystemState()
    except Exception as e:
        rospy.logerr("[TowingPreflight] Cannot query ROS master: %s", e)
        return

    published_topics = {name for name, _ in pubs}
    service_names = {name for name, _ in srvs}

    missing_mocap = [
        f"/beetle{module_id}/mocap/pose"
        for module_id in module_ids
        if f"/beetle{module_id}/mocap/pose" not in published_topics
    ]
    if missing_mocap:
        rospy.logwarn("[TowingPreflight] Missing module mocap topics: %s", missing_mocap)

    load_topic = '/load/odom' if simulation else '/load/mocap/pose'
    if load_topic not in published_topics:
        rospy.logwarn("[TowingPreflight] Missing load pose topic: %s", load_topic)

    nav_subscribers = _names_for_topic(subs, '/assembly/uav/nav')
    if nav_subscribers:
        rospy.loginfo("[TowingPreflight] /assembly/uav/nav subscribers: %s", nav_subscribers)
    else:
        rospy.logwarn("[TowingPreflight] No subscriber on /assembly/uav/nav; towing commands would be ignored")

    missing_unified_services = [
        f"/beetle{module_id}/controller/set_unified_mode"
        for module_id in module_ids
        if f"/beetle{module_id}/controller/set_unified_mode" not in service_names
    ]
    if missing_unified_services:
        rospy.logwarn("[TowingPreflight] Missing set_unified_mode services: %s", missing_unified_services)

    cpp_leaders = {}
    for module_id in module_ids:
        param_name = f"/beetle{module_id}/assembly_leader_id"
        if rospy.has_param(param_name):
            cpp_leaders[module_id] = rospy.get_param(param_name)
    if cpp_leaders:
        python_ee_leader = module_ids[-1] if module_ids else None
        rospy.loginfo("[TowingPreflight] C++ assembly_leader_id params: %s; Python EE leader=%s",
                      cpp_leaders, python_ee_leader)
        unique_cpp_leaders = set(cpp_leaders.values())
        if len(unique_cpp_leaders) == 1 and python_ee_leader not in unique_cpp_leaders:
            rospy.logwarn("[TowingPreflight] Python EE leader differs from C++ control leader; "
                          "this is OK only if the EE is on the last module and wrench routing uses assembly_leader_id")
    else:
        rospy.logwarn("[TowingPreflight] assembly_leader_id params are not available yet; "
                      "launch after assembly/unified navigation has published them")


def main():
    rospy.init_node('formation_load_towing')

    global TOWING_DISTANCE, TOWING_MAX_FORCE
    TOWING_DISTANCE = rospy.get_param("~towing_distance", 0.6)
    TOWING_MAX_FORCE = rospy.get_param("~towing_force", 5.0)
    module_ids = _parse_module_ids(rospy.get_param("~module_ids", ""))
    real_machine = _as_bool(rospy.get_param("~real_machine", False))
    simulation = _as_bool(rospy.get_param("~simulation", True))

    rospy.loginfo("=" * 60)
    rospy.loginfo("Formation Load Towing Task")
    rospy.loginfo("=" * 60)
    rospy.loginfo(f"Load box: {LOAD_BOX_LENGTH}m x {LOAD_BOX_WIDTH}m x {LOAD_BOX_HEIGHT}m")
    rospy.loginfo(f"Hook contact insert clearance: {HOOK_CONTACT_INSERT_CLEARANCE*1000:.0f}mm above box top")
    rospy.loginfo(f"Retract distance: {RETRACT_DISTANCE*1000:.0f}mm")
    rospy.loginfo(f"Towing distance: {TOWING_DISTANCE}m")
    rospy.loginfo(f"Max towing force: {TOWING_MAX_FORCE}N (adaptive from 0N)")
    rospy.loginfo("=" * 60)
    log_towing_preflight(module_ids, real_machine, simulation)

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
