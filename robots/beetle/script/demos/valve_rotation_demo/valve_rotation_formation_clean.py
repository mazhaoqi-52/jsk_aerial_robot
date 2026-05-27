#!/usr/bin/env python3
"""Formation valve-rotation demo built on shared Beetle interfaces."""

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

from task.assembly_motion import AssemblyDemo
from n_modules_tf import NModuleTFCalculator
from valve_rotation_fang_single import (
    SingleUAVStateBase,
    InitializeStartPositionState,
    MoveToValveState,
    DescendAndContactState,
    RotateValveState
)
from insertion_optimizer import InsertionOptimizer
from trajectory import PolynomialTrajectory
from beetle_interface import BeetleInterface


class WaitState(smach.State):
    """Pause between SMACH states while continuing to report progress."""
    def __init__(self, wait_time=3.0, state_name=""):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.wait_time = wait_time
        self.state_name = state_name

    def execute(self, userdata):
        if self.state_name:
            rospy.loginfo(f"Waiting {self.wait_time}s before {self.state_name}...")
        else:
            rospy.loginfo(f"Waiting {self.wait_time}s...")

        rospy.sleep(self.wait_time)

        if self.state_name:
            rospy.loginfo(f"Wait complete, proceeding to {self.state_name}")
        else:
            rospy.loginfo("Wait complete")

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
    def _angle_diff(a, b):
        """Shortest signed difference a - b, result in [-pi, pi]."""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    @staticmethod
    def monitor_valve_rotation(current_valve_yaw, initial_valve_yaw, last_valve_yaw,
                               last_check_time, current_time, update_interval=0.2):
        """
        Monitor valve rotation progress and calculate angular velocity.
        Uses shortest-path angle difference to handle +/-pi boundary correctly.
        Returns: (valve_rotation, valve_angular_velocity, should_update, new_last_yaw, new_last_time)
        """
        valve_rotation = abs(FormationUtils._angle_diff(current_valve_yaw, initial_valve_yaw))
        valve_angular_velocity = 0.0
        should_update = current_time > last_check_time + update_interval

        if should_update:
            dt = current_time - last_check_time
            valve_angular_velocity = abs(FormationUtils._angle_diff(current_valve_yaw, last_valve_yaw)) / dt
            return valve_rotation, valve_angular_velocity, True, current_valve_yaw, current_time

        return valve_rotation, valve_angular_velocity, False, last_valve_yaw, last_check_time

    @staticmethod
    def monitor_cumulative_valve_rotation(current_valve_yaw, last_valve_yaw,
                                          last_check_time, current_time,
                                          cumulative_rotation, rotation_direction,
                                          update_interval=0.2):
        """
        Monitor cumulative valve rotation progress in the commanded direction.
        Returns: (cumulative_rotation, valve_angular_velocity, should_update, new_last_yaw, new_last_time)
        """
        valve_angular_velocity = 0.0
        should_update = current_time > last_check_time + update_interval

        if should_update:
            dt = current_time - last_check_time
            delta_yaw = FormationUtils._angle_diff(current_valve_yaw, last_valve_yaw)
            directed_delta = rotation_direction * delta_yaw
            cumulative_rotation = max(0.0, cumulative_rotation + directed_delta)
            valve_angular_velocity = max(0.0, directed_delta / dt)
            return cumulative_rotation, valve_angular_velocity, True, current_valve_yaw, current_time

        return cumulative_rotation, valve_angular_velocity, False, last_valve_yaw, last_check_time


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
        self.uav_pitches = {}
        self.assembly_pos = None
        self.assembly_yaw = 0.0
        self.assembly_pitch = 0.0
        self.pitch_compensation_enabled = False  # Disabled by default; enable for towing phase

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

        rospy.loginfo(f"Assembly->EE offsets: X={self.base_offset_x:.3f}m, Y={self.base_offset_y:.3f}m, Z={self.total_offset_z:.3f}m")

    def uav_callback(self, msg, module_id):
        """Receive individual UAV position from mocap (PoseStamped format)"""
        pos = msg.pose.position
        ori = msg.pose.orientation

        self.uav_positions[module_id] = (pos.x, pos.y, pos.z)

        _, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.uav_orientations[module_id] = yaw
        self.uav_pitches[module_id] = pitch

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

        # Calculate average pitch
        self.assembly_pitch = sum(self.uav_pitches.get(mid, 0.0) for mid in self.module_ids) / self.number_of_modules

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

    def get_assembly_pitch(self):
        """Get current assembly pitch"""
        return self.assembly_pitch

    def set_pitch_compensation(self, enabled):
        """Enable/disable pitch compensation in coordinate transforms.
        Enable during towing to correct Z-drift from pitch coupling.
        Disable during approach/insertion for stable convergence."""
        self.pitch_compensation_enabled = enabled
        rospy.loginfo(f"Pitch compensation {'ENABLED' if enabled else 'DISABLED'}")

    def get_end_effector_position(self):
        """Get end-effector position using coordinate transformation.
        Uses pitch compensation only when explicitly enabled."""
        if self.assembly_pos is None:
            rospy.logwarn("Assembly position not available for end-effector calculation")
            return None

        pitch = self.assembly_pitch if self.pitch_compensation_enabled else 0.0
        return self.tf_calculator.transform_assembly_to_end_effector(
            self.assembly_pos, self.assembly_yaw, pitch
        )

    def transform_end_effector_to_assembly_command(self, target_end_effector_pos, target_yaw=None):
        """Transform end-effector target to assembly CoG command"""
        if target_yaw is None:
            target_yaw = self.assembly_yaw if self.assembly_yaw is not None else 0.0

        # Use pitch only when compensation is enabled
        current_pitch = self.assembly_pitch if self.pitch_compensation_enabled else 0.0

        assembly_to_leader = self.tf_calculator.calculate_assembly_to_leader_transform()
        leader_to_ee = self.tf_calculator.calculate_leader_to_end_effector_transform(target_yaw, current_pitch)

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


class FormationAssembleState(smach.State):
    """Execute physical assembly of two UAVs"""

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        modules_str = rospy.get_param("~module_ids", "1,2")
        real_machine = rospy.get_param("~real_machine", False)
        modules = []
        if modules_str:
            modules = [int(x) for x in modules_str.split(',')]
        else:
            rospy.logerr("No module ID is designated!")

        rospy.loginfo(f"Formation assembly configured for modules: {modules}, real_machine: {real_machine}")

        self.assemble_demo = AssemblyDemo(module_ids=modules, real_machine=real_machine)
        rospy.loginfo("Assembly demo loaded from task.assembly_motion")

    def execute(self, userdata):
        rospy.loginfo("=== ASSEMBLING UAVs ===")
        try:
            self.assemble_demo.main()
            rospy.loginfo("Assembly completed successfully")
            return 'succeeded'
        except rospy.ROSInterruptException:
            rospy.logerr("Assembly process interrupted")
            return 'failed'
        except Exception as e:
            rospy.logerr(f"Assembly process failed: {e}")
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

        self.max_linear_velocity = 0.24
        self.average_linear_velocity = 0.16
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

    def active_position_convergence(self, target_pos, target_yaw, pos_thresh=0.025, yaw_thresh=0.0175, timeout=15.0, max_yaw_step=None, max_linear_vel=None, max_angular_vel=None, yaw_only=False):
        """Active convergence: repeatedly send target and wait for position/yaw to settle.

        Args:
            yaw_only: If True, only check yaw convergence (ignore position error)
        """
        start_time = rospy.get_time()

        max_angular_vel_limit = max_angular_vel if max_angular_vel is not None else 0.05
        max_linear_vel_limit = max_linear_vel if max_linear_vel is not None else 0.16

        consecutive_good_readings = 0
        required_consecutive = 8

        rospy.loginfo(f"Active convergence: target={FormationUtils.format_vec(target_pos)}, yaw={math.degrees(target_yaw):.1f} deg, thresh={pos_thresh*1000:.1f}mm/{math.degrees(yaw_thresh):.1f} deg")

        while rospy.get_time() - start_time < timeout:
            current_pos = self.get_end_effector_position()
            if current_pos is None:
                rospy.sleep(0.1)
                continue

            current_yaw = self.get_end_effector_yaw()
            if current_yaw is None:
                rospy.sleep(0.1)
                continue

            xy_error = math.sqrt((current_pos[0] - target_pos[0])**2 + (current_pos[1] - target_pos[1])**2)
            z_error = abs(current_pos[2] - target_pos[2])
            pos_error = math.sqrt(xy_error**2 + z_error**2)
            yaw_error = abs(FormationUtils.normalize_angle(target_yaw - current_yaw))

            position_ok = (xy_error < pos_thresh and z_error < 0.050)
            yaw_ok = yaw_error < yaw_thresh

            # Safety abort: if position diverges beyond 300mm, stop immediately
            if not yaw_only and pos_error > 0.300:
                rospy.logwarn(f"Position diverged to {pos_error*1000:.0f}mm, aborting convergence")
                return False

            # If yaw_only mode, only check yaw convergence
            converged = (yaw_ok) if yaw_only else (position_ok and yaw_ok)

            if converged:
                consecutive_good_readings += 1
                if consecutive_good_readings >= required_consecutive:
                    rospy.loginfo(f"Convergence success: pos={pos_error*1000:.1f}mm, yaw={math.degrees(yaw_error):.1f} deg")
                    return True
                # Keep sending hold command during confirmation to prevent drift
                self.send_assembly_command_from_end_effector(
                    target_end_effector_pos=target_pos, target_yaw=target_yaw)
            else:
                consecutive_good_readings = 0

                # Apply max_yaw_step limitation like Formation version
                command_yaw = target_yaw
                if max_yaw_step is not None and max_yaw_step > 0.0:
                    yaw_delta = FormationUtils.normalize_angle(target_yaw - current_yaw)
                    if abs(yaw_delta) > max_yaw_step:
                        yaw_delta = math.copysign(max_yaw_step, yaw_delta)
                        command_yaw = FormationUtils.normalize_angle(current_yaw + yaw_delta)
                        elapsed = rospy.get_time() - start_time
                        if int(elapsed * 5.0) % 20 == 0:
                            rospy.loginfo(f"[Formation YawLimit] Step limited to {math.degrees(yaw_delta):.1f} deg "
                                        f"(remaining {math.degrees(abs(FormationUtils.normalize_angle(target_yaw - current_yaw))):.1f} deg)")

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
                        smooth_linear_vel = [direction[0] * speed_magnitude,
                                           direction[1] * speed_magnitude,
                                           direction[2] * speed_magnitude]
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
                                f"yaw_err={math.degrees(yaw_error):.1f} deg, t={elapsed:.1f}s")

            rospy.sleep(0.04)

        rospy.logwarn(f"Formation active convergence timeout after {timeout:.1f}s: "
                     f"pos_err={pos_error*1000:.1f}mm, yaw_err={math.degrees(yaw_error):.1f} deg")
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


    def streaming_z_descent(self, start_pos, final_target, final_yaw, descent_speed=0.05):
        """Streaming Z descent with inline contact detection at 25Hz.

        Uses polynomial trajectory for smooth continuous descent while monitoring
        actual Z motion each cycle. If Z stops moving (contact), locks height.

        Returns:
            (success, achieved_position)
        """
        rospy.loginfo("=== Formation Streaming Z Descent ===")

        if start_pos is None or final_target is None:
            rospy.logerr("[Z Descent] Missing start or target position")
            return False, None

        target_x, target_y, target_z = final_target
        start_z = start_pos[2]
        total_descent = start_z - target_z

        if total_descent <= 0.01:
            rospy.loginfo("[Z Descent] Small descent, direct convergence")
            self.send_assembly_command_from_end_effector(final_target, final_yaw)
            success = self.active_position_convergence(
                final_target, target_yaw=final_yaw,
                pos_thresh=0.080, yaw_thresh=0.087, timeout=10.0)
            achieved = self.get_end_effector_position() or final_target
            return success, achieved

        rospy.loginfo(f"[Z Descent] {total_descent*1000:.1f}mm at {descent_speed*1000:.0f}mm/s")

        # Generate Z-only polynomial trajectory (XY locked, yaw locked)
        duration = max(total_descent / descent_speed, 2.0) / 0.7  # safety factor
        traj_z = PolynomialTrajectory(duration=duration)
        traj_z.is_scalar = True
        traj_z.coeffs_scalar = traj_z.compute_coefficients(start_z, target_z)
        traj_z.start_value = start_z
        traj_z.target_value = target_z

        rospy.loginfo(f"[Z Descent] Trajectory T={duration:.1f}s, target_Z={target_z:.3f}m")

        # Contact detection state
        small_motion_thresh = 0.008  # 8mm
        consecutive_small = 0
        required_small = 2
        prev_actual_z = start_z
        contact_pos = None

        # XY adaptive pause: threshold narrows linearly with descent progress
        xy_pause_far = 0.120   # 120mm at start (loose)
        xy_pause_near = 0.050  # 50mm near valve (tight)
        xy_resume_ratio = 0.6  # resume when error < 60% of pause threshold
        xy_diverge_limit = 0.300  # absolute abort
        paused = False
        pause_z = None         # frozen cmd_z while paused
        pause_elapsed = 0.0    # accumulated pause time
        pause_timeout = 5.0    # max pause before abort
        traj_time = 0.0        # trajectory time (freezes during pause)

        rate = rospy.Rate(25)
        t0 = rospy.get_time()
        dt = 1.0 / 25.0

        while not rospy.is_shutdown():
            wall_t = rospy.get_time() - t0
            if traj_time >= duration:
                break

            # Commanded Z from trajectory (frozen when paused)
            if not paused:
                traj_time = min(wall_t - pause_elapsed, duration)
            cmd_z = traj_z.evaluate_at_time(traj_time) if not paused else pause_z
            cmd_pos = [target_x, target_y, cmd_z]

            # Velocity: zero when paused, polynomial otherwise
            vel_z = 0.0
            if not paused:
                nt = traj_time / duration
                if 0 < nt < 1:
                    T_vel = np.array([5*nt**4, 4*nt**3, 3*nt**2, 2*nt, 1, 0]) / duration
                    vel_z = float(np.dot(traj_z.coeffs_scalar, T_vel))
            vel = [0.0, 0.0, vel_z]

            self.send_assembly_command_from_end_effector(cmd_pos, final_yaw, linear_vel=vel)

            # Read actual position
            actual_pos = self.get_end_effector_position()
            if actual_pos is None:
                rate.sleep()
                continue

            actual_z = actual_pos[2]
            xy_error = math.sqrt((actual_pos[0] - target_x)**2 + (actual_pos[1] - target_y)**2)

            # Safety: XY divergence abort
            if xy_error > xy_diverge_limit:
                rospy.logwarn(f"[Z Descent] XY diverged to {xy_error*1000:.0f}mm, aborting")
                return False, actual_pos

            # Adaptive XY pause threshold (linearly narrows with descent progress)
            progress = min((start_z - actual_z) / total_descent, 1.0) if total_descent > 0 else 0
            xy_pause_thresh = xy_pause_far + (xy_pause_near - xy_pause_far) * progress
            xy_resume_thresh = xy_pause_thresh * xy_resume_ratio

            if paused:
                if xy_error < xy_resume_thresh:
                    paused = False
                    pause_elapsed += rospy.get_time() - pause_start
                    rospy.loginfo(f"[Z Descent] XY recovered to {xy_error*1000:.0f}mm, resuming "
                                 f"(thresh={xy_resume_thresh*1000:.0f}mm)")
                elif rospy.get_time() - pause_start > pause_timeout:
                    rospy.logwarn(f"[Z Descent] XY pause timeout {pause_timeout}s, "
                                 f"XY_err={xy_error*1000:.0f}mm, aborting")
                    return False, actual_pos
            else:
                if xy_error > xy_pause_thresh:
                    paused = True
                    pause_z = cmd_z
                    pause_start = rospy.get_time()
                    rospy.logwarn(f"[Z Descent] XY drift {xy_error*1000:.0f}mm > "
                                 f"thresh {xy_pause_thresh*1000:.0f}mm, pausing at Z={cmd_z:.3f}m")

            # Contact detection: actual Z stopped AND command is significantly below actual
            if not paused:
                dz = abs(actual_z - prev_actual_z)
                cmd_descended = start_z - cmd_z
                cmd_actual_gap = actual_z - cmd_z
                if (dz < small_motion_thresh
                        and cmd_descended > 0.050
                        and cmd_actual_gap > 0.030):
                    consecutive_small += 1
                    if consecutive_small >= required_small:
                        contact_pos = actual_pos
                        rospy.loginfo(f"[Z Descent] Contact detected at Z={actual_z:.3f}m "
                                     f"(cmd_Z={cmd_z:.3f}m, gap={cmd_actual_gap*1000:.0f}mm, "
                                     f"dZ={dz*1000:.1f}mm, XY_err={xy_error*1000:.1f}mm)")
                        break
                else:
                    consecutive_small = 0

            prev_actual_z = actual_z

            # Periodic logging
            elapsed_int = int(wall_t)
            if elapsed_int % 3 == 0 and abs(wall_t - round(wall_t)) < 0.025:
                descended = start_z - actual_z
                status = " [PAUSED]" if paused else ""
                rospy.loginfo(f"[Z Descent] t={traj_time:.1f}/{duration:.1f}s Z={actual_z:.3f}m "
                              f"descended={descended*1000:.0f}mm XY_err={xy_error*1000:.0f}mm"
                              f" thresh={xy_pause_thresh*1000:.0f}mm{status}")

            rate.sleep()

        # Determine achieved position
        achieved = contact_pos or self.get_end_effector_position() or final_target
        descended = start_z - achieved[2]
        remaining = achieved[2] - target_z

        if contact_pos is not None:
            rospy.loginfo(f"[Z Descent] Contact: descended {descended*1000:.0f}mm, "
                          f"remaining {remaining*1000:.0f}mm (physical limit)")
            return True, achieved

        rospy.loginfo(f"[Z Descent] Trajectory complete, descended {descended*1000:.0f}mm")
        return True, achieved

    def active_stabilization_wait(self, target_pos, target_yaw, duration, description="position stabilization"):
        """
        Active stabilization: continuously send target commands during wait period.

        Args:
            target_pos: Target end-effector position (x, y, z)
            target_yaw: Target yaw angle (radians)
            duration: Wait duration in seconds
            description: Description for logging
        """
        rospy.loginfo(f"Active stabilization: {duration}s {description}")
        rospy.loginfo(f"  Target: pos=({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}), yaw={math.degrees(target_yaw):.1f} deg")

        start_time = rospy.get_time()
        rate = rospy.Rate(10)

        while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
            self.send_assembly_command_from_end_effector(target_pos, target_yaw)

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
                    rospy.loginfo(f"  Progress: {elapsed:.1f}s, pos_err={pos_error*1000:.1f}mm, yaw_err={math.degrees(yaw_error):.1f} deg")

            rate.sleep()

        rospy.loginfo(f"Stabilization complete: {duration}s {description}")

    def generate_polynomial_trajectory(self, start_pos, target_pos, target_yaw, lock_yaw=False):
        """Generate polynomial trajectory objects for streaming execution.

        Returns:
            dict with trajectory objects and parameters, or None on failure
        """
        current_yaw = self.get_end_effector_yaw()
        if current_yaw is None:
            current_yaw = 0.0

        total_distance = math.sqrt(sum((target_pos[i] - start_pos[i])**2 for i in range(3)))
        yaw_change = abs(FormationUtils.normalize_angle(target_yaw - current_yaw))

        trajectory_duration = max(
            total_distance / self.average_linear_velocity,
            yaw_change / self.max_angular_velocity,
            8.0
        ) / 0.7  # safety factor

        rospy.loginfo(f"=== Streaming Trajectory: dist={total_distance:.3f}m, "
                      f"yaw={math.degrees(yaw_change):.1f} deg, T={trajectory_duration:.1f}s ===")

        traj_x = PolynomialTrajectory(duration=trajectory_duration)
        traj_x.is_scalar = True
        traj_x.coeffs_scalar = traj_x.compute_coefficients(start_pos[0], target_pos[0])
        traj_x.start_value = start_pos[0]
        traj_x.target_value = target_pos[0]

        traj_y = PolynomialTrajectory(duration=trajectory_duration)
        traj_y.is_scalar = True
        traj_y.coeffs_scalar = traj_y.compute_coefficients(start_pos[1], target_pos[1])
        traj_y.start_value = start_pos[1]
        traj_y.target_value = target_pos[1]

        traj_z = PolynomialTrajectory(duration=trajectory_duration)
        traj_z.is_scalar = True
        traj_z.coeffs_scalar = traj_z.compute_coefficients(start_pos[2], target_pos[2])
        traj_z.start_value = start_pos[2]
        traj_z.target_value = target_pos[2]

        if lock_yaw:
            traj_yaw = None
        else:
            traj_yaw = PolynomialTrajectory(duration=trajectory_duration)
            traj_yaw.is_scalar = True
            traj_yaw.coeffs_scalar = traj_yaw.compute_coefficients(current_yaw, target_yaw)
            traj_yaw.start_value = current_yaw
            traj_yaw.target_value = target_yaw

        return {
            'traj_x': traj_x, 'traj_y': traj_y, 'traj_z': traj_z,
            'traj_yaw': traj_yaw,
            'target_pos': list(target_pos),
            'target_yaw': target_yaw,
            'duration': trajectory_duration,
        }

    def execute_polynomial_trajectory(self, traj_desc):
        """Stream polynomial trajectory at 25Hz, then brief final convergence."""
        traj_x = traj_desc['traj_x']
        traj_y = traj_desc['traj_y']
        traj_z = traj_desc['traj_z']
        traj_yaw_obj = traj_desc['traj_yaw']
        target_yaw = traj_desc['target_yaw']
        target_pos = traj_desc['target_pos']
        duration = traj_desc['duration']

        rospy.loginfo(f"Streaming trajectory: T={duration:.1f}s, target={FormationUtils.format_vec(target_pos)}")

        rate = rospy.Rate(25)
        t0 = rospy.get_time()

        while not rospy.is_shutdown():
            t = rospy.get_time() - t0
            if t >= duration:
                break

            pos = [traj_x.evaluate_at_time(t),
                   traj_y.evaluate_at_time(t),
                   traj_z.evaluate_at_time(t)]
            yaw = traj_yaw_obj.evaluate_at_time(t) if traj_yaw_obj else target_yaw

            nt = t / duration
            vel = None
            if 0 < nt < 1:
                T_vel = np.array([5*nt**4, 4*nt**3, 3*nt**2, 2*nt, 1, 0]) / duration
                vel = [float(np.dot(traj_x.coeffs_scalar, T_vel)),
                       float(np.dot(traj_y.coeffs_scalar, T_vel)),
                       float(np.dot(traj_z.coeffs_scalar, T_vel))]
                vel_mag = math.sqrt(sum(v**2 for v in vel))
                if vel_mag > self.max_linear_velocity:
                    scale = self.max_linear_velocity / vel_mag
                    vel = [v * scale for v in vel]

            self.send_assembly_command_from_end_effector(pos, yaw, linear_vel=vel)

            if int(t) % 5 == 0 and abs(t - round(t)) < 0.025:
                rospy.loginfo(f"[Streaming] t={t:.1f}/{duration:.1f}s pos={FormationUtils.format_vec(pos)}")

            rate.sleep()

        # Send final target
        self.send_assembly_command_from_end_effector(target_pos, target_yaw)

        # Brief final convergence
        rospy.loginfo("Streaming complete, final convergence")
        return self.active_position_convergence(
            target_pos, target_yaw,
            pos_thresh=0.050, yaw_thresh=0.087, timeout=15.0
        )


class FormationInitializeStartPositionState(FormationSingleUAVStateBase):
    """Formation version of InitializeStartPositionState"""

    def __init__(self):
        FormationSingleUAVStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            output_keys=['start_position', 'start_ee_position', 'start_yaw', 'valve_position', 'valve_yaw'])

    def execute(self, userdata):
        rospy.loginfo("=== Formation Initialize Start Position State ===")

        # Wait for position data
        rospy.sleep(2.0)

        # Get formation positions
        assembly_pos = self.get_assembly_position()
        assembly_yaw = self.get_assembly_yaw()

        if assembly_pos is None:
            rospy.logerr("Assembly position not available")
            return 'failed'

        # Poll for valve position (mocap may need extra time to start publishing)
        valve_pos = None
        valve_yaw = None
        deadline = rospy.get_time() + 10.0
        while rospy.get_time() < deadline and not rospy.is_shutdown():
            valve_pos = self.beetle.getValvePos()
            valve_yaw = self.beetle.getValveYaw()
            if valve_pos is not None and valve_yaw is not None:
                break
            rospy.sleep(0.5)

        if valve_pos is None:
            rospy.logerr("Valve position not available")
            return 'failed'

        # Store positions in userdata
        userdata.start_position = assembly_pos
        userdata.start_ee_position = self.get_end_effector_position()
        userdata.start_yaw = assembly_yaw
        userdata.valve_position = valve_pos
        userdata.valve_yaw = valve_yaw

        rospy.loginfo(f"Formation start position: {assembly_pos}, yaw: {math.degrees(assembly_yaw):.1f} deg")
        rospy.loginfo(f"Valve position: {valve_pos}")
        rospy.loginfo(f"Valve yaw: {valve_yaw}")

        return 'succeeded'


class FormationMoveToValveState(FormationSingleUAVStateBase):
    # Shared optimizer strategy for later states (class variable)
    _shared_optimizer_strategy = None
    """Formation version of MoveToValveState with 3-phase control logic from single UAV version"""

    def __init__(self):
        FormationSingleUAVStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['valve_position', 'valve_yaw'],
            output_keys=['phase4_contact_pose', 'phase4_contact_yaw'])

        # Convergence thresholds (based on single UAV proven values)
        self.pos_threshold = 0.02         # 20mm position threshold (same as single)
        self.yaw_threshold = 0.05         # ~2.9 deg yaw threshold (same as single)
        self.vel_threshold = 0.01         # 10mm/s velocity threshold (same as single)

        # Final positioning accuracy
        self.final_pos_threshold = 0.015  # 15mm final accuracy (slightly relaxed from single's 10mm)
        self.final_yaw_threshold = 0.0175 # 1 deg final yaw accuracy (same as single)

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
        formation_z_offset = rospy.get_param("~formation_phase4_z_offset", 0.0)
        if abs(formation_z_offset) > 1e-4:
            compensated_z = safe_ee_pos[2] + formation_z_offset
            rospy.loginfo(
                f"[Phase4] Apply Z compensation {formation_z_offset*1000:.0f}mm -> {compensated_z:.3f}m"
            )
            safe_ee_pos = (safe_ee_pos[0], safe_ee_pos[1], compensated_z)
        else:
            rospy.loginfo(f"[Phase4] Using optimizer target: {safe_ee_pos[2]:.3f}m (valve height)")

        # Validate target height matches valve height
        expected_valve_height = valve_pos[2]
        actual_target_height = safe_ee_pos[2]
        height_difference = abs(actual_target_height - expected_valve_height)
        if height_difference > 0.01:
            rospy.logwarn(f"Phase4 target {actual_target_height:.3f}m differs from valve {expected_valve_height:.3f}m by {height_difference*1000:.1f}mm")
        else:
            rospy.loginfo(f"Phase4 target {actual_target_height:.3f}m matches valve {expected_valve_height:.3f}m (diff: {height_difference*1000:.1f}mm)")

        # Share target Z for Contact phase consistency
        FormationSingleUAVStateBase._shared_target_z = safe_ee_pos[2]
        rospy.loginfo(f"Current: {FormationUtils.format_vec(current_ee_pos)}, yaw={math.degrees(current_yaw):.1f} deg")
        rospy.loginfo(f"Target: {FormationUtils.format_vec(safe_ee_pos)}, yaw={math.degrees(safe_ee_yaw):.1f} deg, valve_yaw={math.degrees(valve_yaw):.1f} deg")

        rospy.loginfo("[Phase 1] Skipped: Focusing on XY positioning only")

        # PHASE 2: XY MOVEMENT TO OPTIMIZER TARGET
        phase1_ee_pos = self.get_end_effector_position()
        if phase1_ee_pos is None:
            rospy.logerr("Cannot get end-effector position before Phase 2")
            return 'failed'

        current_yaw_for_phase2 = self.get_end_effector_yaw()
        if current_yaw_for_phase2 is None:
            rospy.logwarn("Cannot get current yaw, using 0.0")
            current_yaw_for_phase2 = 0.0

        xy_target_ee_pos = (safe_ee_pos[0], safe_ee_pos[1], phase1_ee_pos[2])
        rospy.loginfo(f"[Phase 2] Move to safe XY: Target {FormationUtils.format_vec(xy_target_ee_pos)}")
        if not self._execute_formation_phase2_xy_movement(xy_target_ee_pos, current_yaw_for_phase2):
            rospy.logerr("Phase 2 failed: XY movement unsuccessful")
            return 'failed'
        rospy.loginfo("Phase 2 completed: Reached optimizer XY position")

        # PHASE 3: PRECISION XY & YAW (spoke alignment)
        phase2_ee_pos = self.get_end_effector_position()
        phase2_yaw = self.get_end_effector_yaw()
        if phase2_ee_pos is None or phase2_yaw is None:
            rospy.logerr("Cannot get end-effector position/yaw after Phase 2")
            return 'failed'

        # PHASE 3A: XY precision positioning (keep Z, do not descend yet)
        phase3a_target_pos = (safe_ee_pos[0], safe_ee_pos[1], phase2_ee_pos[2])
        rospy.loginfo(f"[Phase 3A] Precision XY: Lock Z {phase3a_target_pos[2]:.3f}m")
        if not self._execute_formation_phase3a_xy_positioning(phase3a_target_pos, phase2_yaw):
            rospy.logerr("Phase 3A failed")
            return 'failed'
        rospy.loginfo("Phase 3A complete: XY precision achieved")

        # PHASE 3B: Spoke-aligned yaw
        phase3a_ee_pos = self.get_end_effector_position()
        if phase3a_ee_pos is None:
            rospy.logerr("Cannot get position after Phase 3A")
            return 'failed'

        optimal_spoke_yaw = self.calculate_shortest_yaw_path(phase2_yaw, safe_ee_yaw)
        if not self._execute_formation_phase3b_spoke_alignment(phase3a_ee_pos, optimal_spoke_yaw, phase3a_target_pos):
            rospy.logerr("Phase 3B failed")
            return 'failed'
        rospy.loginfo("Phase 3B complete: Spoke alignment achieved")

        # PHASE 4: Z DESCENT TO OPTIMIZER HEIGHT
        final_target_pos = (safe_ee_pos[0], safe_ee_pos[1], safe_ee_pos[2])

        final_yaw = optimal_spoke_yaw

        if not self._execute_formation_phase4_z_descent(final_target_pos, final_yaw):
            rospy.logerr("Phase 4 failed: Z descent unsuccessful")
            return 'failed'
        rospy.loginfo("Phase 4 complete: insertion height reached")

        # Share Phase4 locking pose and yaw with subsequent states
        phase4_pose = getattr(self, '_last_phase4_contact_position', None)
        if phase4_pose is None:
            phase4_pose = self.get_end_effector_position() or final_target_pos
        userdata.phase4_contact_pose = phase4_pose
        userdata.phase4_contact_yaw = self.get_end_effector_yaw() or final_yaw

        # Debug: Log Phase4 completion state (use logdebug to reduce verbosity)
        if rospy.get_param("~debug_verbose", False):
            current_ee_pos = self.get_end_effector_position()
            current_assembly_pos = self.get_assembly_position()
            current_ee_yaw = self.get_end_effector_yaw()
            rospy.logdebug("=" * 80)
            rospy.logdebug(f"[PHASE4_COMPLETE] Formation insertion complete:")
            rospy.logdebug(f"  Optimizer target (end-effector): {FormationUtils.format_vec(final_target_pos)}")
            rospy.logdebug(f"  Current end-effector: {FormationUtils.format_vec(current_ee_pos) if current_ee_pos else 'None'}")
            rospy.logdebug(f"  Current Assembly CoG: {FormationUtils.format_vec(current_assembly_pos) if current_assembly_pos else 'None'}")
            rospy.logdebug(f"  Current yaw: {math.degrees(current_ee_yaw):.1f} deg" if current_ee_yaw is not None else "  Current yaw: None")
            rospy.logdebug(f"  Passed to Contact phase: {FormationUtils.format_vec(phase4_pose)}")
            contact_yaw = self.get_end_effector_yaw() or final_yaw
            rospy.logdebug(f"  Contact phase yaw: {math.degrees(contact_yaw):.1f} deg" if contact_yaw is not None else "  Contact yaw: None")
            if current_ee_pos and current_assembly_pos:
                ee_assembly_diff = [(current_ee_pos[i] - current_assembly_pos[i]) * 1000 for i in range(3)]
                rospy.logdebug(f"  EE-Assembly offset: ({ee_assembly_diff[0]:.1f}, {ee_assembly_diff[1]:.1f}, {ee_assembly_diff[2]:.1f})mm")
            rospy.logdebug("=" * 80)

        # Post-insertion stabilization
        if hasattr(self, '_last_phase4_contact_position') and self._last_phase4_contact_position is not None:
            rospy.loginfo(f"5s stabilization at {FormationUtils.format_vec(self._last_phase4_contact_position)}, yaw={math.degrees(final_yaw):.1f} deg")
            self.active_stabilization_wait(self._last_phase4_contact_position, final_yaw, 5.0, "Post-insertion")
        else:
            rospy.logwarn("Phase4 position not found, using current position")
            current_ee_pos = self.get_end_effector_position()
            current_ee_yaw = self.get_end_effector_yaw()
            if current_ee_pos and current_ee_yaw is not None:
                self.active_stabilization_wait(current_ee_pos, current_ee_yaw, 5.0, "Post-insertion")
            else:
                rospy.logwarn("Cannot get position, passive wait")
                rospy.sleep(5.0)

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
        rospy.loginfo(f"[YawPath] {math.degrees(current_yaw):.1f} deg -> "
                      f"{math.degrees(target_yaw):.1f} deg (delta={math.degrees(diff):.1f} deg)")
        return optimal_target

    def _check_positioning_error(self, target_pos, target_yaw=None, pos_thresh=0.050, yaw_thresh=None):
        """Check current position/yaw error against target. Returns (pos_ok, yaw_ok, xy_err, z_err, yaw_err)"""
        current_pos = self.get_end_effector_position()
        if not current_pos:
            return False, False, None, None, None

        xy_error = math.sqrt((current_pos[0] - target_pos[0])**2 + (current_pos[1] - target_pos[1])**2)
        z_error = abs(current_pos[2] - target_pos[2])
        pos_ok = (xy_error <= pos_thresh and z_error <= 0.050)

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
                                timeout=15.0):
        """
        Generic XY positioning method used by Phase 2, 3A, 3B
        Reduces code duplication by parametrizing behavior
        """
        rospy.loginfo(f"[{phase_name}] Target: {FormationUtils.format_vec(target_pos)}, yaw={math.degrees(target_yaw):.1f} deg")

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
                pos_thresh=pos_thresh, yaw_thresh=yaw_thresh, timeout=timeout
            )

        # Streaming trajectory execution (includes final convergence)
        rospy.loginfo(f"[{phase_name}] Using streaming polynomial trajectory")
        traj_desc = self.generate_polynomial_trajectory(
            start_pos=current_pos, target_pos=target_pos,
            target_yaw=target_yaw
        )

        if not traj_desc:
            rospy.logwarn(f"[{phase_name}] Trajectory generation failed, using direct convergence")
            return self.active_position_convergence(
                target_pos=target_pos, target_yaw=target_yaw,
                pos_thresh=pos_thresh, yaw_thresh=yaw_thresh, timeout=timeout * 1.5
            )

        return self.execute_polynomial_trajectory(traj_desc)

    def _execute_formation_phase2_xy_movement(self, xy_target_ee_pos, maintain_yaw):
        """Phase 2: Pure XY movement using streaming polynomial trajectory (Formation version)"""
        return self._execute_xy_positioning(
            target_pos=xy_target_ee_pos, target_yaw=maintain_yaw,
            phase_name="Phase2", use_trajectory=True,
            pos_thresh=0.050, yaw_thresh=0.087, timeout=15.0
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
        rospy.loginfo(f"Spoke angle: {math.degrees(spoke_angle):.1f} deg")
        rospy.loginfo(f"Required spoke yaw: {math.degrees(spoke_yaw):.1f} deg")

        return spoke_yaw

    def _execute_formation_phase3a_xy_positioning(self, target_ee_pos, maintain_yaw):
        """Phase 3A: XY precision positioning with Z locked"""
        current_state = self.get_end_effector_position()
        if current_state is None:
            rospy.logerr("Phase 3A: Cannot get current position")
            return False

        locked_z = target_ee_pos[2] if target_ee_pos[2] else current_state[2]
        xy_target = (target_ee_pos[0], target_ee_pos[1], locked_z)

        # Multiple attempts with verification
        for attempt in range(1, 4):
            rospy.loginfo(f"[Phase3A] Attempt {attempt}/3")
            success = self._execute_xy_positioning(
                target_pos=xy_target, target_yaw=maintain_yaw,
                phase_name=f"Phase3A-{attempt}", use_trajectory=False,
                pos_thresh=0.050, yaw_thresh=0.087, timeout=5.0
            )

            if success:
                pos_ok, _, xy_error, z_error, _ = self._check_positioning_error(
                    (xy_target[0], xy_target[1], locked_z), pos_thresh=0.050
                )
                if pos_ok:
                    rospy.loginfo(f"Phase 3A successful: XY={xy_error*1000:.1f}mm, Z={z_error*1000:.1f}mm")
                    return True

        rospy.logerr("Phase 3A failed after 3 attempts")
        return False

    def _execute_formation_phase3b_spoke_alignment(self, current_ee_pos, spoke_yaw, target_pos=None):
        """Phase 3B: Spoke yaw alignment with XY/Z locked"""
        reference_xy = (target_pos[0], target_pos[1]) if target_pos else (current_ee_pos[0], current_ee_pos[1])
        reference_z = target_pos[2] if target_pos else current_ee_pos[2]
        locked_pos = (reference_xy[0], reference_xy[1], reference_z)

        for attempt in range(1, 4):
            rospy.loginfo(f"[Phase3B] Attempt {attempt}/3 - Yaw to {math.degrees(spoke_yaw):.1f} deg")

            # Strict: try 2.5 deg with full timeout (large rotation needs time)
            success = self.active_position_convergence(
                target_pos=locked_pos, target_yaw=spoke_yaw,
                pos_thresh=0.050, yaw_thresh=0.044, timeout=10.0,
                max_yaw_step=math.radians(10.0)
            )

            if success:
                pos_ok, yaw_ok, xy_error, z_error, yaw_error = self._check_positioning_error(
                    locked_pos, spoke_yaw, pos_thresh=0.050, yaw_thresh=0.044
                )
                if pos_ok and yaw_ok:
                    rospy.loginfo(f"Phase 3B successful (strict 2.5 deg): XY={xy_error*1000:.1f}mm, yaw={math.degrees(yaw_error):.1f} deg")
                    return True
            else:
                # Lenient fallback: accept if within 5 deg
                pos_ok, yaw_ok, xy_error, z_error, yaw_error = self._check_positioning_error(
                    locked_pos, spoke_yaw, pos_thresh=0.050, yaw_thresh=0.087
                )
                if pos_ok and yaw_ok:
                    rospy.logwarn(f"[Phase3B] 2.5 deg not reached, yaw={math.degrees(yaw_error):.1f} deg < 5 deg - accepting")
                    return True

            pos_ok, _, xy_error, _, _ = self._check_positioning_error(locked_pos, pos_thresh=0.050)
            if not pos_ok:
                rospy.logwarn(f"[Phase3B] Position drift, correcting...")
                final_yaw = self.get_end_effector_yaw()
                self._execute_formation_phase3a_xy_positioning(locked_pos, final_yaw)

        rospy.logerr("Phase 3B failed after 3 attempts")
        return False

    def _execute_formation_phase4_z_descent(self, final_ee_pos, final_yaw):
        """Phase 4: Pure Z descent to insertion height (Formation version)"""
        rospy.loginfo(
            f"[Phase4] target={FormationUtils.format_vec(final_ee_pos)}, "
            f"yaw_hold={math.degrees(final_yaw):.1f} deg"
        )

        # Reset cached contact pose for this attempt
        self._last_phase4_contact_position = None

        current_pos = self.get_end_effector_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position for Phase 4")
            return False

        final_target_pos = (final_ee_pos[0], final_ee_pos[1], final_ee_pos[2])

        descent_success, achieved_pos = self.streaming_z_descent(current_pos, final_target_pos, final_yaw)
        if not descent_success:
            fallback_pos = achieved_pos or self.get_end_effector_position()
            if fallback_pos is not None:
                xy_error = math.sqrt(
                    (fallback_pos[0] - final_target_pos[0])**2 +
                    (fallback_pos[1] - final_target_pos[1])**2
                )
                z_error = abs(fallback_pos[2] - final_target_pos[2])
                rospy.logwarn(
                    f"[Phase4] Z descent aborted with residual XY {xy_error*1000:.1f}mm, Z {z_error*1000:.1f}mm"
                )
            return False

        if achieved_pos is None:
            achieved_pos = self.get_end_effector_position() or final_target_pos

        if abs(achieved_pos[2] - final_target_pos[2]) > 1e-3:
            rospy.loginfo(
                f"[Phase4] Contact depth locked at {achieved_pos[2]:.3f}m (planned {final_target_pos[2]:.3f}m)"
            )


        convergence_target = achieved_pos
        rospy.loginfo(f"[Phase4] Final convergence target: {FormationUtils.format_vec(convergence_target)}")
        self._last_phase4_contact_position = convergence_target

        rospy.loginfo("[Phase4] Performing final insertion pose convergence")
        # Strict: try 2.5 deg within 5s
        final_success = self.active_position_convergence(
            target_pos=convergence_target,
            target_yaw=final_yaw,
            pos_thresh=0.080,  # 80mm convergence threshold (RELAXED for formation)
            yaw_thresh=0.044,  # 2.5 degrees (0.044 rad)
            timeout=5.0
        )
        if not final_success:
            # Lenient fallback: accept if within 5 deg
            pos_ok, yaw_ok, xy_err, z_err, yaw_err = self._check_positioning_error(
                convergence_target, final_yaw, pos_thresh=0.080, yaw_thresh=0.087
            )
            if pos_ok and yaw_ok:
                rospy.logwarn(f"[Phase4] 2.5 deg not reached in 5s, yaw={math.degrees(yaw_err):.1f} deg < 5 deg - accepting")
                final_success = True

        # CRITICAL: Regardless of final_success, always save the actual insertion depth Z coordinate reached
        # Store Z coordinate for Contact/Rotation phases to avoid pitch errors
        FormationSingleUAVStateBase._shared_target_z = convergence_target[2]
        rospy.loginfo(f"Z continuity: saved _shared_target_z={FormationSingleUAVStateBase._shared_target_z:.3f}m")

        if final_success:
            rospy.loginfo("Phase 4 completed successfully - ready for valve insertion")
        else:
            fallback_pos = self.get_end_effector_position()
            if fallback_pos is not None:
                xy_error = math.sqrt(
                    (fallback_pos[0] - convergence_target[0])**2 +
                    (fallback_pos[1] - convergence_target[1])**2
                )
                z_error = abs(fallback_pos[2] - convergence_target[2])
                rospy.logwarn(
                    f"[Phase4] Final residual: XY {xy_error*1000:.1f}mm, Z {z_error*1000:.1f}mm"
                )
            rospy.logwarn("Phase 4 completed with convergence issues")

        return final_success

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class FormationRotateValveState(FormationSingleUAVStateBase):
    """Formation valve contact and rotation using streaming circular trajectory."""

    def __init__(self, rotation_direction=1, target_rotation=math.radians(90.0)):
        FormationSingleUAVStateBase.__init__(
            self,
            outcomes=['succeeded', 'failed', 'emergency'],
            input_keys=['valve_position', 'valve_yaw', 'phase4_contact_pose', 'phase4_contact_yaw'],
            output_keys=['trajectory_state', 'contact_final_torque']
        )
        self.rotation_direction = rotation_direction
        self.target_rotation = abs(target_rotation)
        self.max_rotation_time = 60.0
        self.contact_angular_velocity = 0.1   # rad/s for contact phase
        self.rotation_angular_velocity = 0.1  # rad/s for rotation phase

    def execute(self, userdata):
        rospy.loginfo("=== Formation Contact and Rotate Valve State (Streaming) ===")

        valve_pos = list(userdata.valve_position)
        valve_yaw = userdata.valve_yaw
        self.optimizer.update_valve_info(valve_pos, valve_yaw)

        current_ee_pos = self.get_end_effector_position()
        if current_ee_pos is None:
            rospy.logerr("Failed to get current end-effector position")
            return 'failed'

        rospy.loginfo(f"Current EE pos: {FormationUtils.format_vec(current_ee_pos)}")
        rospy.loginfo("SKIPPING Phase 1 & 2: Phase 4 insertion already completed")

        # Correct valve Z to insertion depth
        if FormationSingleUAVStateBase._shared_target_z is not None:
            valve_pos[2] = FormationSingleUAVStateBase._shared_target_z
            rospy.loginfo(f"Valve Z corrected to {valve_pos[2]:.3f}m")

        # Compute geometric parameters from EE position
        ee_radius = math.sqrt((current_ee_pos[0] - valve_pos[0])**2 +
                              (current_ee_pos[1] - valve_pos[1])**2)
        ee_start_angle = math.atan2(current_ee_pos[1] - valve_pos[1],
                                    current_ee_pos[0] - valve_pos[0])
        rospy.loginfo(f"EE radius={ee_radius*1000:.1f}mm, start_angle={math.degrees(ee_start_angle):.1f} deg")

        # Spoke selection
        spoke_angles = self.optimizer.get_spoke_angles()
        angle_diffs = [abs(a - ee_start_angle) for a in spoke_angles]
        selected_spoke = spoke_angles[angle_diffs.index(min(angle_diffs))]
        rospy.loginfo(f"Spoke selection: approach={math.degrees(ee_start_angle):.1f} deg, selected={math.degrees(selected_spoke):.1f} deg")

        if ee_radius > 0.025:
            rospy.logwarn(f"Distance to valve center too large: {ee_radius*1000:.1f}mm")

        self.valve_center = valve_pos
        self.initial_valve_yaw = valve_yaw

        # --- Phase 1: Contact establishment via streaming circular trajectory ---
        rospy.loginfo(">>> Phase 1: Streaming circular contact establishment")
        contact_ok = self._streaming_circular_contact(valve_pos, ee_radius, ee_start_angle)
        if not contact_ok:
            return 'failed'

        # --- Phase 2: Rotation via streaming circular trajectory ---
        rospy.loginfo(">>> Phase 2: Streaming valve rotation")
        rotation_result = self._streaming_circular_rotation(userdata, valve_pos, ee_radius, ee_start_angle)
        return rotation_result

    # ------------------------------------------------------------------
    # helpers shared by contact & rotation
    # ------------------------------------------------------------------
    def _circular_ee_target(self, valve_center, radius, angle, z):
        """Compute EE target position and yaw from circular parameters."""
        x = valve_center[0] + radius * math.cos(angle)
        y = valve_center[1] + radius * math.sin(angle)
        yaw = angle + math.pi  # face valve center
        yaw = FormationUtils.normalize_angle(yaw)
        return [x, y, z], yaw

    def _circular_ee_velocity(self, radius, angular_vel, angle):
        """Compute tangential linear velocity for streaming."""
        speed = radius * angular_vel
        tang_angle = angle + math.pi / 2  # perpendicular to radius
        return [speed * math.cos(tang_angle), speed * math.sin(tang_angle), 0.0]

    def _enable_task_wrench_prediction(self):
        module_masses = rospy.get_param("~module_masses", None)
        module_positions = rospy.get_param("~module_positions", None)
        module_inertias_diag = rospy.get_param("~module_inertias_diag", None)
        self.beetle.setAttachModule(self.beetle.module_id,
                                    module_masses=module_masses,
                                    module_positions=module_positions,
                                    module_inertias_diag=module_inertias_diag)

    def _clamp_directed_torque(self, torque_z, torque_min, torque_limit):
        torque_limit = max(torque_min, abs(torque_limit))
        mag = min(torque_limit, max(torque_min, abs(torque_z)))
        return math.copysign(mag, self.rotation_direction)

    def _centripetal_force_world(self, radius, angular_vel, angle):
        if radius < 1e-3:
            return [0.0, 0.0, 0.0]
        vel = self.beetle.getUavLinearVel()
        speed = np.linalg.norm(vel[:2]) if vel is not None else 0.0
        speed = max(speed, abs(radius * angular_vel))
        mass = self.beetle.getFormationMass()
        force_mag = mass * speed * speed / radius
        radial = np.array([math.cos(angle), math.sin(angle), 0.0])
        force = -force_mag * radial
        return force.tolist()

    def _pid_axis_effort(self, axis):
        try:
            return float(axis.p_term[0]) + float(axis.i_term[0])
        except (AttributeError, IndexError, TypeError, ValueError):
            return 0.0

    def _dragon_roll_moment_adjust(self, target_yaw, dt, moment_thresh, adjust_gain):
        control_pid = self.beetle.getControlPid()
        if control_pid is None:
            return 0.0, 0.0
        moment = np.array([
            self._pid_axis_effort(control_pid.roll),
            self._pid_axis_effort(control_pid.pitch),
            self._pid_axis_effort(control_pid.yaw)
        ])
        # Dragon projects the controller moment into the valve target frame.
        # Beetle valve tasks are yaw-only here, so use the target-yaw x-axis.
        roll_moment = math.cos(target_yaw) * moment[0] + math.sin(target_yaw) * moment[1]
        if abs(roll_moment) < moment_thresh:
            roll_moment = 0.0
        return roll_moment * adjust_gain * dt, roll_moment

    def _formation_yaw_rate(self):
        omega = self.beetle.getUavAngularVel()
        if omega is None or len(omega) < 3 or not np.isfinite(omega[2]):
            return 0.0
        return float(omega[2])

    def _ee_offset_body(self):
        return [
            self.formation_adapter.base_offset_x,
            self.formation_adapter.base_offset_y,
            self.formation_adapter.total_offset_z,
        ]

    def _build_valve_wrench_command(self, force_world, torque_world):
        if not self.beetle.isUnifiedMode():
            return force_world, torque_world, "world_yaw"
        force_body, torque_body = self.beetle.buildFormationCoGWrench(
            force_world, torque_world, application_offset_body=self._ee_offset_body())
        return force_body, torque_body, "fc"

    # ------------------------------------------------------------------
    def _streaming_circular_contact(self, valve_center, radius, start_angle):
        """Stream circular trajectory at 25Hz until valve engagement detected."""
        contact_threshold = math.radians(5.7)  # ~0.1 rad
        angular_vel = self.contact_angular_velocity * self.rotation_direction
        max_contact_time = 20.0
        ee_z = valve_center[2]
        ff_enabled = rospy.get_param("controller/valve_rotation_feedforward/enabled", False)
        contact_torque = rospy.get_param("controller/valve_rotation_feedforward/contact_init_torque", 0.1)
        torque_ramp = rospy.get_param("controller/valve_rotation_feedforward/contact_torque_ramp", 0.1)
        torque_limit = abs(rospy.get_param("controller/valve_rotation_feedforward/torque_z", 3.0)) or 3.0
        torque_z = self._clamp_directed_torque(
            contact_torque * self.rotation_direction, 0.05, torque_limit)

        start_valve_yaw = FormationUtils.get_valve_yaw_safe(self.beetle, self.initial_valve_yaw)
        last_valve_yaw = start_valve_yaw
        last_valve_check_time = rospy.Time.now().to_sec()
        max_detected = 0.0

        rospy.loginfo(f"Contact: radius={radius*1000:.1f}mm, omega={math.degrees(angular_vel):.1f} deg/s, "
                      f"threshold={math.degrees(contact_threshold):.1f} deg, timeout={max_contact_time}s, "
                      f"ff={ff_enabled}, torque0={torque_z:.2f}N*m")

        if ff_enabled:
            self._enable_task_wrench_prediction()

        rate = rospy.Rate(25)
        t0 = rospy.get_time()
        last_t = t0
        angle = start_angle

        while not rospy.is_shutdown():
            now = rospy.get_time()
            t = now - t0
            dt = max(0.0, min(now - last_t, 0.1))
            last_t = now
            if t > max_contact_time:
                break

            # Advance angle
            angle += angular_vel / 25.0

            # EE target
            ee_pos, ee_yaw = self._circular_ee_target(valve_center, radius, angle, ee_z)
            ee_vel = self._circular_ee_velocity(radius, angular_vel, angle)

            # Dragon-style contact torque: start small and ramp slowly.
            if ff_enabled:
                ff_force, ff_torque, ff_frame = self._build_valve_wrench_command(
                    [0.0, 0.0, 0.0], [0.0, 0.0, torque_z])
                self.beetle.addExternalWrench(force=ff_force, torque=ff_torque,
                                              frame_id=ff_frame)
            self.send_assembly_command_from_end_effector(ee_pos, ee_yaw, linear_vel=ee_vel)

            # Monitor valve rotation
            cur_time = rospy.get_time()
            cur_valve_yaw = FormationUtils.get_valve_yaw_safe(self.beetle, self.initial_valve_yaw)
            valve_rot, valve_omega, updated, last_valve_yaw, last_valve_check_time = \
                FormationUtils.monitor_valve_rotation(
                    cur_valve_yaw, start_valve_yaw, last_valve_yaw,
                    last_valve_check_time, cur_time, update_interval=0.2)
            max_detected = max(max_detected, valve_rot)

            rospy.loginfo_throttle(2.0,
                f"Formation contact: radius={radius*1000:.1f}mm, angular_vel={math.degrees(angular_vel):.1f} deg/s, "
                f"valve_rotation={math.degrees(valve_rot):.2f} deg, time={t:.1f}s")

            if valve_rot >= contact_threshold:
                rospy.loginfo(f"Valve engagement detected: {math.degrees(valve_rot):.1f} deg "
                              f"(threshold: {math.degrees(contact_threshold):.1f} deg)")
                rospy.loginfo(f"Contact establishment time: {t:.1f}s")
                rospy.loginfo(f"Rotation baseline set: initial={math.degrees(self.initial_valve_yaw):.1f} deg, "
                              f"current={math.degrees(cur_valve_yaw):.1f} deg, achieved={math.degrees(valve_rot):.1f} deg")
                # Save angle for rotation phase continuity
                self._contact_end_angle = angle
                self._contact_final_torque_z = torque_z
                return True

            torque_z = self._clamp_directed_torque(
                torque_z + self.rotation_direction * torque_ramp * dt,
                0.05, torque_limit)
            rate.sleep()

        rospy.logerr(f"Contact failed: max valve movement {math.degrees(max_detected):.2f} deg")
        self.beetle.clearExternalWrench(duration=1.0, rate_hz=25.0)
        self.beetle.setAttachModule(None)
        return False

    # ------------------------------------------------------------------
    def _streaming_circular_rotation(self, userdata, valve_center, radius, start_angle):
        """Stream circular rotation trajectory at 25Hz with adaptive wrench feedforward."""
        # Continue from contact end angle for smooth transition
        angle = getattr(self, '_contact_end_angle', start_angle)
        angular_vel = self.rotation_angular_velocity * self.rotation_direction
        ee_z = valve_center[2]

        # Read feedforward parameters from launch config. Defaults mirror the
        # Dragon valve task: small initial torque, slow adaptation, capped output.
        ff_enabled = rospy.get_param("controller/valve_rotation_feedforward/enabled", False)
        ff_force_z = rospy.get_param("controller/valve_rotation_feedforward/force_z", 0.0)
        ff_torque_z_max = rospy.get_param("controller/valve_rotation_feedforward/torque_z", 0.0)
        torque_min = rospy.get_param("controller/valve_rotation_feedforward/torque_min", 0.1)
        torque_limit = rospy.get_param(
            "controller/valve_rotation_feedforward/torque_limit",
            abs(ff_torque_z_max) if ff_torque_z_max != 0.0 else 3.0)
        torque_ramp_up = rospy.get_param("controller/valve_rotation_feedforward/torque_ramp_up", 0.3)
        torque_ramp_down = rospy.get_param("controller/valve_rotation_feedforward/torque_ramp_down", 0.2)
        roll_moment_thresh = rospy.get_param("controller/valve_rotation_feedforward/roll_moment_thresh", 0.2)
        torque_adjust_roll_k = rospy.get_param("controller/valve_rotation_feedforward/torque_adjust_roll_k", 0.01)
        torque_adjust_yaw_k = rospy.get_param("controller/valve_rotation_feedforward/torque_adjust_yaw_k", 1.0)
        yaw_velocity_thresh = rospy.get_param("controller/valve_rotation_feedforward/yaw_velocity_thresh", 0.05)
        rp_guard = rospy.get_param("controller/valve_rotation_feedforward/rp_guard", math.radians(12.0))

        target_omega = abs(self.rotation_angular_velocity)
        torque_z = getattr(self, '_contact_final_torque_z',
                           torque_min * self.rotation_direction)
        torque_z = self._clamp_directed_torque(torque_z, torque_min, torque_limit)
        slow_thresh = 0.3
        fast_thresh = 1.5

        rospy.loginfo(f">>> Phase 2: Rotation {math.degrees(self.target_rotation):.1f} deg "
                      f"@ {math.degrees(angular_vel):.1f} deg/s")
        rospy.loginfo(f"Feedforward: enabled={ff_enabled}, force_z={ff_force_z}, "
                      f"torque_limit={torque_limit:.1f} N*m (adaptive from {torque_z:.2f})")

        # Enable per-module y_hat^task auto-publish for the duration of the
        # rotation. The valve reaction wrench is applied at the EE module;
        # BeetleInterface publishes y_hat_i^task = (m_i/m_total) * W_ext on
        # /beetle{i}/est_wrench_task so the C++ controller can subtract the
        # task component before forming inter_wrench_list_ (Step C).
        if ff_enabled:
            self._enable_task_wrench_prediction()


        start_valve_yaw = FormationUtils.get_valve_yaw_safe(self.beetle, self.initial_valve_yaw)
        last_valve_yaw = start_valve_yaw
        last_valve_check_time = rospy.get_time()
        cumulative_rotation = 0.0
        max_rotation_detected = 0.0

        rate = rospy.Rate(25)
        t0 = rospy.get_time()
        last_t = t0

        while not rospy.is_shutdown():
            now = rospy.get_time()
            t = now - t0
            dt = max(0.0, min(now - last_t, 0.1))
            last_t = now

            # Monitor valve rotation
            cur_time = rospy.get_time()
            cur_valve_yaw = FormationUtils.get_valve_yaw_safe(self.beetle, start_valve_yaw)
            valve_rot, valve_omega, updated, last_valve_yaw, last_valve_check_time = \
                FormationUtils.monitor_cumulative_valve_rotation(
                    cur_valve_yaw, last_valve_yaw,
                    last_valve_check_time, cur_time,
                    cumulative_rotation, self.rotation_direction,
                    update_interval=0.2)
            cumulative_rotation = valve_rot
            max_rotation_detected = max(max_rotation_detected, valve_rot)

            # Check completion
            if valve_rot >= self.target_rotation:
                rospy.loginfo(f"Valve rotation completed: {math.degrees(valve_rot):.1f} deg "
                              f"(target: {math.degrees(self.target_rotation):.1f} deg) in {t:.1f}s")
                rospy.loginfo(f"Final adaptive torque: {torque_z:.2f} N*m")
                self.beetle.clearExternalWrench(duration=1.0, rate_hz=25.0)
                self.beetle.setAttachModule(None)
                userdata.trajectory_state = {
                    'current_radius': radius,
                    'current_angle': angle,
                    'radius_locked': True,
                    'valve_center': self.valve_center
                }
                userdata.contact_final_torque = [0.0, 0.0, torque_z]
                return 'succeeded'

            # Timeout
            if t > self.max_rotation_time:
                rospy.logwarn(
                    f"Rotation timeout after {t:.1f}s, current: {math.degrees(valve_rot):.1f} deg, "
                    f"max: {math.degrees(max_rotation_detected):.1f} deg")
                break

            # --- Adaptive torque based on valve response ---
            if updated and t > 1.0:  # skip first second for sensor settling
                if valve_omega < target_omega * slow_thresh:
                    # Valve barely moving -> increase torque
                    torque_z += self.rotation_direction * torque_ramp_up * dt
                elif valve_omega > target_omega * fast_thresh:
                    # Valve moving too fast -> decrease torque
                    torque_z -= self.rotation_direction * torque_ramp_down * dt
                # else: within [30%, 150%] of target -> hold (hysteresis band)

            # Advance angle (streaming)
            angle += angular_vel / 25.0

            # EE target
            ee_pos, ee_yaw = self._circular_ee_target(valve_center, radius, angle, ee_z)
            ee_vel = self._circular_ee_velocity(radius, angular_vel, angle)

            # Wrench feedforward with adaptive torque
            if ff_enabled:
                roll_adjust, roll_moment = self._dragon_roll_moment_adjust(
                    ee_yaw, dt, roll_moment_thresh, torque_adjust_roll_k)
                delta_yaw_rate = angular_vel - self._formation_yaw_rate()
                if abs(delta_yaw_rate) < yaw_velocity_thresh:
                    delta_yaw_rate = 0.0
                torque_z += roll_adjust + delta_yaw_rate * torque_adjust_yaw_k * dt
                rpy = self.beetle.getAssemblyRPY()
                if rpy is not None and max(abs(rpy[0]), abs(rpy[1])) > rp_guard:
                    torque_z *= 0.8
                    rospy.logwarn_throttle(
                        1.0, "Valve FF roll/pitch guard active: roll=%.1f deg, pitch=%.1f deg",
                        math.degrees(rpy[0]), math.degrees(rpy[1]))
                torque_z = self._clamp_directed_torque(torque_z, torque_min, torque_limit)
                ff_force = self._centripetal_force_world(radius, angular_vel, angle)
                ff_force[2] += ff_force_z
                ff_force_cmd, ff_torque_cmd, ff_frame = self._build_valve_wrench_command(
                    ff_force, [0.0, 0.0, torque_z])
                self.beetle.addExternalWrench(force=ff_force_cmd, torque=ff_torque_cmd,
                                              frame_id=ff_frame)

            self.send_assembly_command_from_end_effector(ee_pos, ee_yaw, linear_vel=ee_vel)

            # Log every 5s
            rospy.loginfo_throttle(5.0,
                f"Rotation: {math.degrees(valve_rot):.1f} deg/{math.degrees(self.target_rotation):.1f} deg, "
                f"omega={math.degrees(valve_omega):.2f} deg/s, torque={torque_z:.2f}N*m, t={t:.1f}s")

            rate.sleep()

        # Failed
        self.beetle.clearExternalWrench(duration=1.0, rate_hz=25.0)
        self.beetle.setAttachModule(None)
        rospy.logerr(f"Valve rotation failed: achieved {math.degrees(max_rotation_detected):.1f} deg "
                     f"of {math.degrees(self.target_rotation):.1f} deg target")
        return 'failed'


class FormationDisengageFromValveState(FormationSingleUAVStateBase):
    """Formation version of DisengageFromValveState - simplified 3-phase approach"""

    def __init__(self):
        FormationSingleUAVStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['valve_position', 'start_position', 'start_ee_position', 'start_yaw', 'trajectory_state'],
            output_keys=['disengagement_position'])

    def execute(self, userdata):
        rospy.loginfo("=== Formation Disengage From Valve State ===")

        valve_pos = getattr(userdata, 'valve_position', None)
        if valve_pos is None:
            rospy.logerr("Valve position missing from userdata")
            return 'failed'

        start_assembly_pos = getattr(userdata, 'start_position', None)
        start_ee_pos = getattr(userdata, 'start_ee_position', None)
        start_yaw = getattr(userdata, 'start_yaw', 0.0)
        trajectory_state = getattr(userdata, 'trajectory_state', {}) or {}

        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        if current_pos is None or current_yaw is None:
            rospy.logerr("Unable to get current pose for disengage")
            return 'failed'

        rospy.loginfo(f"Current: {FormationUtils.format_vec(current_pos)}, yaw={math.degrees(current_yaw):.1f} deg")

        # Phase 1: Streaming reverse circular motion (same pattern as rotation)
        rospy.loginfo("[Phase 1] Reverse 8 deg circular motion (streaming)")

        generator_center = trajectory_state.get('valve_center', valve_pos)
        generator_radius = trajectory_state.get('current_radius')
        if generator_radius is None:
            generator_radius = math.sqrt((current_pos[0] - generator_center[0])**2 +
                                        (current_pos[1] - generator_center[1])**2)

        reverse_angle_total = math.radians(8.0)
        reverse_angular_vel = -0.05  # negative = reverse direction
        reverse_duration = abs(reverse_angle_total / reverse_angular_vel)

        # Current angle on the circle
        angle = math.atan2(current_pos[1] - generator_center[1],
                           current_pos[0] - generator_center[0])
        ee_z = current_pos[2]

        rospy.loginfo(f"Reverse: {math.degrees(reverse_angle_total):.1f} deg @ {math.degrees(reverse_angular_vel):.1f} deg/s, "
                      f"radius={generator_radius*1000:.1f}mm, duration={reverse_duration:.1f}s")

        rate = rospy.Rate(25)
        t0 = rospy.get_time()
        while not rospy.is_shutdown():
            t = rospy.get_time() - t0
            if t >= reverse_duration:
                break
            angle += reverse_angular_vel / 25.0
            ee_x = generator_center[0] + generator_radius * math.cos(angle)
            ee_y = generator_center[1] + generator_radius * math.sin(angle)
            ee_yaw = FormationUtils.normalize_angle(angle + math.pi)
            speed = generator_radius * abs(reverse_angular_vel)
            tang = angle + math.pi / 2
            ee_vel = [speed * math.cos(tang) * (-1), speed * math.sin(tang) * (-1), 0.0]  # reverse direction
            self.send_assembly_command_from_end_effector([ee_x, ee_y, ee_z], ee_yaw, linear_vel=ee_vel)
            rate.sleep()

        rospy.loginfo("Reverse motion completed")

        # Phase 2: Ascend to start height
        rospy.sleep(1.0)
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()

        # Phase 2: Ascend to start height
        rospy.loginfo("[Phase 2] Ascending to start height")

        # Determine target Z height
        if start_ee_pos is not None:
            target_z = start_ee_pos[2] + 0.1  # 10cm above start EE height
        else:
            target_z = current_pos[2] + 0.15  # 15cm up as fallback

        ascent_target = (current_pos[0], current_pos[1], target_z)
        z_distance = abs(target_z - current_pos[2])
        rospy.loginfo(f"Ascending to Z={target_z:.3f}m (distance: {z_distance*1000:.1f}mm)")
        rospy.loginfo(f"[Phase 2] Ascent with XY locked at ({current_pos[0]:.3f}, {current_pos[1]:.3f})")

        # Use direct convergence with very slow speed and locked XY position
        # Simpler and more reliable than polynomial trajectory for pure vertical motion
        rospy.loginfo("[Phase 2] Using slow vertical convergence (XY locked)")
        success = self.active_position_convergence(
            ascent_target,
            target_yaw=current_yaw,  # Maintain current yaw during ascent
            pos_thresh=0.05,         # 50mm threshold
            yaw_thresh=0.1,          # 5.7 deg yaw tolerance (not critical during ascent)
            timeout=30.0,
            max_linear_vel=0.03,     # Slow ascent: 30mm/s (was 50mm/s)
            max_angular_vel=0.02     # Minimal rotation during ascent
        )

        if not success:
            rospy.logwarn("Ascent convergence incomplete, but continuing")

        rospy.sleep(1.0)
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()

        # Phase 3: Return to start XY position
        rospy.loginfo("[Phase 3] Returning to start XY position")

        if start_ee_pos is not None:
            # Phase 3A: First adjust yaw to start_yaw (yaw-only convergence)
            rospy.loginfo(f"[Phase 3A] Adjusting yaw to {math.degrees(start_yaw):.1f} deg before XY movement")
            yaw_adjust_target = (current_pos[0], current_pos[1], current_pos[2])

            # Yaw-only convergence: ignore position error, only check yaw
            yaw_success = self.active_position_convergence(
                yaw_adjust_target,
                target_yaw=start_yaw,
                yaw_thresh=0.05,  # ~2.9 deg threshold
                timeout=60.0,
                max_linear_vel=0.02,
                max_angular_vel=0.05,
                max_yaw_step=math.radians(10.0),
                yaw_only=True  # Only check yaw convergence
            )

            if not yaw_success:
                rospy.logwarn("[Phase 3A] Yaw adjustment timeout, but continuing")
            else:
                rospy.loginfo("[Phase 3A] Yaw adjustment complete")

            rospy.sleep(1.0)

            # Update current position after yaw adjustment
            current_pos = self.get_end_effector_position()
            current_yaw = self.get_end_effector_yaw()

            # Phase 3B: Now perform XY movement with yaw locked at start_yaw
            rospy.loginfo(f"[Phase 3B] XY movement to start position (yaw locked at {math.degrees(start_yaw):.1f} deg)")
            return_target = (start_ee_pos[0], start_ee_pos[1], start_ee_pos[2])
            rospy.loginfo(f"Target: ({return_target[0]:.3f}, {return_target[1]:.3f}, {return_target[2]:.3f})")

            # Calculate XY distance for trajectory selection
            xy_distance = math.sqrt(
                (return_target[0] - current_pos[0])**2 +
                (return_target[1] - current_pos[1])**2
            )
            rospy.loginfo(f"[Phase 3B] XY return distance: {xy_distance*1000:.1f}mm")

            # Use polynomial trajectory for smooth return (same as Phase 2 approach to valve)
            if xy_distance > 0.1:  # Use trajectory for movements > 100mm
                rospy.loginfo("[Phase 3B] Using streaming polynomial trajectory for smooth XY return")
                traj_desc = self.generate_polynomial_trajectory(
                    start_pos=current_pos,
                    target_pos=return_target,
                    target_yaw=start_yaw,
                    lock_yaw=True
                )

                if traj_desc:
                    success = self.execute_polynomial_trajectory(traj_desc)
                else:
                    rospy.logwarn("[Phase 3B] Trajectory generation failed, using direct convergence")
                    success = self.active_position_convergence(
                        return_target,
                        target_yaw=start_yaw,
                        pos_thresh=0.05,
                        yaw_thresh=0.1,
                        timeout=90.0,
                        max_linear_vel=0.05,
                        max_angular_vel=0.035
                    )
            else:
                # Short distance: direct convergence
                rospy.loginfo("[Phase 3B] Short distance, using direct convergence")
                success = self.active_position_convergence(
                    return_target,
                    target_yaw=start_yaw,
                    pos_thresh=0.05,
                    yaw_thresh=0.1,
                    timeout=20.0,
                    max_linear_vel=0.05,
                    max_angular_vel=0.035
                )

            if not success:
                rospy.logwarn("[Phase 3B] Return to start incomplete")
        else:
            rospy.logwarn("Start position unavailable, skipping return")

        final_pos = self.get_end_effector_position()
        userdata.disengagement_position = final_pos

        rospy.loginfo(f"=== Disengagement complete: {FormationUtils.format_vec(final_pos)} ===")
        return 'succeeded'


# Main execution function
def main():
    rospy.init_node('formation_valve_rotation')

    # Get target rotation angle (degrees)
    rotation_angle_deg_param = rospy.get_param("~rotation_angle_deg", 90.0)
    try:
        rotation_angle_deg = float(rotation_angle_deg_param)
    except (TypeError, ValueError):
        rospy.logwarn(f"Invalid rotation_angle_deg '{rotation_angle_deg_param}', fallback to 90 deg")
        rotation_angle_deg = 90.0

    if rotation_angle_deg == 0.0:
        rospy.logwarn("rotation_angle_deg is 0.0 deg; no effective valve rotation target")
    elif rotation_angle_deg < 0.0:
        rospy.logwarn(f"rotation_angle_deg ({rotation_angle_deg}) is negative; using its absolute value")

    target_rotation = math.radians(abs(rotation_angle_deg))

    # Get rotation direction from parameter
    direction_param = rospy.get_param("~valve_rotation_direction", "cw")
    direction_normalized = direction_param.strip().lower()
    if direction_normalized in ["clockwise", "cw", "右转", "顺时针"]:
        rotation_direction = -1
        direction_label = "CW (negative yaw)"
    elif direction_normalized in ["counterclockwise", "counter-clockwise", "ccw", "左转", "逆时针"]:
        rotation_direction = 1
        direction_label = "CCW (positive yaw)"
    else:
        rotation_direction = 1
        direction_label = f"CCW (fallback for '{direction_param}')"
        rospy.logwarn(f"Unknown valve rotation direction '{direction_param}', defaulting to 'ccw'")

    rospy.loginfo(f"Formation valve rotation direction: {direction_label}")
    rospy.loginfo(f"Formation valve target rotation: {math.degrees(target_rotation):.1f} deg ({target_rotation:.3f} rad)")

    try:
        # Create Formation SMACH state machine
        sm = smach.StateMachine(outcomes=['success', 'failure'])

        with sm:
            # Step 1: Initialize positions and valve data
            # (Assembly and mode switching done manually before launching this script)
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
                                                             target_rotation=target_rotation),
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
