#!/usr/bin/env python3
"""
Formation Wall Pushing - push a fixed wall from the current end-effector pose.

The task intentionally reuses the valve-rotation formation base and the towing
demo's wrench-weight helpers. The pushing-specific logic is limited to wall
pose/wrench observation, contact approach, force ramping, and retreat.
"""

import math
import os
import sys
import threading

import numpy as np
import rosgraph
import rospy
import smach
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


current_dir = os.path.dirname(os.path.abspath(__file__))
valve_demo_dir = os.path.join(current_dir, '../valve_rotation_demo')
towing_demo_dir = os.path.join(current_dir, '../aerial_towing_demo')
sys.path.insert(0, valve_demo_dir)
sys.path.insert(0, towing_demo_dir)
sys.path.insert(0, os.path.join(current_dir, '../..'))
sys.path.insert(0, os.path.join(current_dir, '..'))

from beetle_interface import smoothstep01
from load_towing_formation import (
    TOWING_UNLOAD_FORCE_RATE,
    TOWING_UNLOAD_MAX_DURATION,
    _as_bool,
    _names_for_topic,
    _parse_module_ids,
    build_towing_task_wrench_weights,
)
from valve_rotation_formation_clean import (
    FormationSingleUAVStateBase,
    FormationUtils,
)


# Wall geometry. The launch file overrides these through private params.
WALL_THICKNESS = 0.1
WALL_WIDTH = 2.0
WALL_HEIGHT = 2.0

# Contact approach.
PUSH_APPROACH_DISTANCE = 0.8
PUSH_APPROACH_SPEED = 0.03
PUSH_APPROACH_OVERRUN = 0.12
PUSH_MIN_APPROACH_DISTANCE = 0.04
PUSH_CONTACT_WRENCH_THRESHOLD = 0.8
PUSH_CONTACT_STALL_LEAD = 0.06
PUSH_CONTACT_STALL_VELOCITY = 0.004
PUSH_CONTACT_REQUIRED_CYCLES = 8

# Pushing feedforward.
PUSH_FORCE = 5.0
PUSH_FORCE_RAMP_TIME = 3.0
PUSH_DURATION = 10.0
PUSH_FULL_FORCE_HOLD_TIME = 2.0
PUSH_POSITION_LEAD = 0.03
PUSH_TASK_WEIGHT_SCALE = 1.0
PUSH_MAX_ROLL_PITCH = math.radians(30.0)
PUSH_UNLOAD_MIN_DURATION = 1.0

# Contact point offset from the virtual end-effector in formation body frame.
PUSH_CONTACT_DX_FROM_EE = 0.0
PUSH_CONTACT_DY_FROM_EE = 0.0
PUSH_CONTACT_DZ_FROM_EE = 0.0

# Exit behavior.
PUSH_RETREAT_AFTER = True
PUSH_RETREAT_DISTANCE = 0.20


class WallInterface(object):
    """Wall pose and wrench observer for simulation and real-machine layouts."""

    def __init__(self):
        self.is_simulation = _as_bool(rospy.get_param("~simulation", True))
        self.use_wall_mocap = _as_bool(rospy.get_param("~use_wall_mocap", False))
        self.use_wall_wrench = _as_bool(rospy.get_param("~use_wall_wrench", False))
        self.use_wall_pose = self.is_simulation or self.use_wall_mocap
        self.wall_x = float(rospy.get_param("~wall_x", 1.5))
        self.wall_y = float(rospy.get_param("~wall_y", 0.0))
        self.wall_z = float(rospy.get_param("~wall_z", 0.0))
        self.wall_yaw = float(rospy.get_param("~wall_yaw", 0.0))

        self.wall_pos = np.array([
            self.wall_x,
            self.wall_y,
            self.wall_z + WALL_HEIGHT * 0.5,
        ], dtype=float)
        self.wall_force = None
        self.wall_torque = None

        self.position_received = threading.Event()
        self.wrench_received = threading.Event()

        if self.is_simulation:
            rospy.Subscriber('/wall/odom', Odometry, self._wall_sim_cb, queue_size=1)
        elif self.use_wall_mocap:
            rospy.Subscriber('/wall/mocap/pose', PoseStamped, self._wall_cb, queue_size=1)
        if self.use_wall_wrench:
            rospy.Subscriber('/wall/wrench', WrenchStamped, self._wrench_cb, queue_size=1)

        rospy.loginfo(
            "WallInterface: mode=%s, use_wall_pose=%s, use_wall_wrench=%s, "
            "fallback center=(%.3f, %.3f, %.3f), yaw=%.1f deg",
            'simulation' if self.is_simulation else 'real_machine',
            self.use_wall_pose, self.use_wall_wrench,
            self.wall_pos[0], self.wall_pos[1], self.wall_pos[2],
            math.degrees(self.wall_yaw),
        )

    def _wall_sim_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.wall_pos = np.array([pos.x, pos.y, pos.z], dtype=float)
        self.wall_yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        if not self.position_received.is_set():
            rospy.loginfo(
                "Wall pose received: center=(%.3f, %.3f, %.3f), yaw=%.1f deg",
                pos.x, pos.y, pos.z, math.degrees(self.wall_yaw))
            self.position_received.set()

    def _wall_cb(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation
        self.wall_pos = np.array([pos.x, pos.y, pos.z], dtype=float)
        self.wall_yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
        if not self.position_received.is_set():
            rospy.loginfo(
                "Wall pose received: center=(%.3f, %.3f, %.3f), yaw=%.1f deg",
                pos.x, pos.y, pos.z, math.degrees(self.wall_yaw))
            self.position_received.set()

    def _wrench_cb(self, msg):
        f = msg.wrench.force
        t = msg.wrench.torque
        self.wall_force = np.array([f.x, f.y, f.z], dtype=float)
        self.wall_torque = np.array([t.x, t.y, t.z], dtype=float)
        if not self.wrench_received.is_set():
            rospy.loginfo("Wall wrench received on /wall/wrench")
            self.wrench_received.set()

    def wait_for_wall(self, timeout=10.0):
        if not self.use_wall_pose:
            rospy.loginfo("Wall pose disabled; using +x/stall-only contact approach")
            return True
        rospy.loginfo("Waiting for wall pose...")
        if self.position_received.wait(timeout):
            return True
        rospy.logwarn(
            "Wall pose not received after %.1fs; using launch fallback pose", timeout)
        return True

    def get_wall_center(self):
        return np.array(self.wall_pos, dtype=float)

    def get_wall_yaw(self):
        return self.wall_yaw

    def get_wall_normal(self):
        return np.array([math.cos(self.wall_yaw), math.sin(self.wall_yaw), 0.0],
                        dtype=float)

    def get_force_projection(self, push_direction):
        if self.wall_force is None:
            return None
        push_dir = np.array(push_direction, dtype=float)
        raw = float(np.dot(self.wall_force, push_dir))
        return abs(raw), float(np.linalg.norm(self.wall_force)), raw

    def distance_to_near_face(self, point, push_direction):
        if not self.use_wall_pose:
            return None
        wall_normal = self.get_wall_normal()
        push_dir = np.array(push_direction, dtype=float)
        alignment = float(np.dot(push_dir[:2], wall_normal[:2]))
        if abs(alignment) < 0.5:
            return None
        face_center = self.get_wall_center() - math.copysign(1.0, alignment) * wall_normal * (WALL_THICKNESS * 0.5)
        return float(np.dot(face_center - np.array(point, dtype=float), push_dir))


class PushingStateBase(FormationSingleUAVStateBase):
    """Base state for wall pushing."""

    _shared_wall_interface = None

    def __init__(self, outcomes, input_keys=None, output_keys=None):
        FormationSingleUAVStateBase.__init__(
            self, outcomes=outcomes,
            input_keys=input_keys or [], output_keys=output_keys or [])

        if PushingStateBase._shared_wall_interface is None:
            PushingStateBase._shared_wall_interface = WallInterface()
        self.wall_interface = PushingStateBase._shared_wall_interface

    def _normalize_horizontal(self, vec, label):
        arr = np.array(vec, dtype=float)
        arr[2] = 0.0
        norm = np.linalg.norm(arr[:2])
        if norm < 1e-6:
            rospy.logwarn("%s is near zero; falling back to +X", label)
            return np.array([1.0, 0.0, 0.0], dtype=float)
        return arr / norm

    def resolve_push_direction(self, current_pos):
        requested = str(rospy.get_param("~push_direction", "auto")).strip().lower()

        named = {
            '+x': np.array([1.0, 0.0, 0.0]),
            'x': np.array([1.0, 0.0, 0.0]),
            '-x': np.array([-1.0, 0.0, 0.0]),
            '+y': np.array([0.0, 1.0, 0.0]),
            'y': np.array([0.0, 1.0, 0.0]),
            '-y': np.array([0.0, -1.0, 0.0]),
        }
        if requested in named:
            return named[requested]

        if requested and requested != "auto":
            try:
                parts = [float(v.strip()) for v in requested.split(',')]
                if len(parts) == 2:
                    parts.append(0.0)
                if len(parts) == 3:
                    return self._normalize_horizontal(parts, "~push_direction")
            except ValueError:
                rospy.logwarn("Invalid push_direction=%s; using auto", requested)

        if not self.wall_interface.use_wall_pose:
            rospy.logwarn("push_direction=auto needs wall pose; falling back to +X")
            return np.array([1.0, 0.0, 0.0], dtype=float)

        wall_normal = self.wall_interface.get_wall_normal()
        wall_center = self.wall_interface.get_wall_center()
        rel = float(np.dot(np.array(current_pos, dtype=float) - wall_center, wall_normal))
        push_dir = wall_normal if rel <= 0.0 else -wall_normal
        return self._normalize_horizontal(push_dir, "auto push direction")

    def wall_force_along_push(self, push_direction):
        force_data = self.wall_interface.get_force_projection(push_direction)
        if force_data is None:
            return None, None, None
        return force_data


class PushingInitializeState(PushingStateBase):
    """Capture the current manually-flown pose and compute push geometry."""

    def __init__(self):
        PushingStateBase.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            output_keys=['start_position', 'start_yaw', 'push_direction',
                         'wall_face_distance'])

    def execute(self, userdata):
        rospy.loginfo("=== Pushing Initialize State ===")

        if not self.wall_interface.wait_for_wall(timeout=10.0):
            rospy.logerr("Failed to initialize wall pose")
            return 'failed'

        rospy.sleep(1.0)
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        if current_pos is None or current_yaw is None:
            rospy.logerr("Current end-effector pose is not available")
            return 'failed'

        push_dir = self.resolve_push_direction(current_pos)
        face_distance = self.wall_interface.distance_to_near_face(current_pos, push_dir)
        if face_distance is None:
            face_distance = float('nan')

        self.formation_adapter.set_pitch_compensation(False)

        userdata.start_position = np.array(current_pos, dtype=float)
        userdata.start_yaw = current_yaw
        userdata.push_direction = push_dir
        userdata.wall_face_distance = face_distance

        face_text = "NA" if not math.isfinite(face_distance) else f"{face_distance:.3f}m"
        rospy.loginfo("Manual start EE position: %s", FormationUtils.format_vec(current_pos))
        rospy.loginfo("Manual start yaw: %.1f deg", math.degrees(current_yaw))
        rospy.loginfo("Push direction: (%.3f, %.3f, %.3f)", push_dir[0], push_dir[1], push_dir[2])
        rospy.loginfo("Estimated distance to wall near face: %s", face_text)
        return 'succeeded'


class ApproachWallUntilContactState(PushingStateBase):
    """Move forward from the current pose until the wall contact is detected."""

    def __init__(self):
        PushingStateBase.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['start_position', 'start_yaw', 'push_direction',
                        'wall_face_distance'],
            output_keys=['contact_position', 'push_yaw'])

    def execute(self, userdata):
        rospy.loginfo("=== Approach Wall Until Contact State ===")

        start_pos = np.array(userdata.start_position, dtype=float)
        push_dir = np.array(userdata.push_direction, dtype=float)
        maintain_yaw = userdata.start_yaw

        face_distance = getattr(userdata, 'wall_face_distance', float('nan'))
        if math.isfinite(face_distance):
            target_distance = max(PUSH_MIN_APPROACH_DISTANCE,
                                  face_distance + PUSH_APPROACH_OVERRUN)
            target_distance = min(PUSH_APPROACH_DISTANCE, target_distance)
        else:
            target_distance = PUSH_APPROACH_DISTANCE

        max_duration = target_distance / max(PUSH_APPROACH_SPEED, 1e-3) + 5.0
        rospy.loginfo(
            "Approach target distance=%.3fm, speed=%.3fm/s, max_duration=%.1fs",
            target_distance, PUSH_APPROACH_SPEED, max_duration)

        rate = rospy.Rate(25)
        t0 = rospy.get_time()
        prev_t = t0
        prev_actual_dist = 0.0
        stall_count = 0

        while not rospy.is_shutdown():
            now = rospy.get_time()
            elapsed = now - t0
            if elapsed > max_duration:
                rospy.logerr("Approach timeout before wall contact")
                return 'failed'

            cmd_dist = min(target_distance, elapsed * PUSH_APPROACH_SPEED)
            target_pos = start_pos + push_dir * cmd_dist
            linear_vel = push_dir * PUSH_APPROACH_SPEED
            if cmd_dist >= target_distance:
                linear_vel = np.zeros(3)

            self.send_assembly_command_from_end_effector(
                target_pos, maintain_yaw, linear_vel=linear_vel.tolist())

            actual_pos = self.get_end_effector_position()
            if actual_pos is None:
                rate.sleep()
                continue

            dt = max(now - prev_t, 1e-3)
            actual_dist = float(np.dot(np.array(actual_pos) - start_pos, push_dir))
            actual_vel = (actual_dist - prev_actual_dist) / dt
            lead = cmd_dist - actual_dist
            distance_to_face = self.wall_interface.distance_to_near_face(actual_pos, push_dir)

            force_abs, force_norm, force_raw = self.wall_force_along_push(push_dir)
            contact_by_wrench = (
                force_abs is not None and
                force_abs >= PUSH_CONTACT_WRENCH_THRESHOLD)

            contact_by_plane = (
                distance_to_face is not None and
                distance_to_face <= -0.01 and
                lead > 0.02)

            if (cmd_dist > PUSH_MIN_APPROACH_DISTANCE and
                    lead > PUSH_CONTACT_STALL_LEAD and
                    abs(actual_vel) < PUSH_CONTACT_STALL_VELOCITY):
                stall_count += 1
            else:
                stall_count = 0
            contact_by_stall = stall_count >= PUSH_CONTACT_REQUIRED_CYCLES

            if contact_by_wrench or contact_by_stall or contact_by_plane:
                reasons = []
                if contact_by_wrench:
                    reasons.append(f"wrench={force_abs:.2f}N")
                if contact_by_stall:
                    reasons.append(f"stall lead={lead*1000:.0f}mm")
                if contact_by_plane:
                    reasons.append("wall_plane")
                contact_pos = np.array(actual_pos, dtype=float)
                userdata.contact_position = contact_pos
                userdata.push_yaw = maintain_yaw
                rospy.loginfo(
                    "Wall contact detected (%s) at %s",
                    ", ".join(reasons),
                    FormationUtils.format_vec(contact_pos))
                return 'succeeded'

            force_text = "NA" if force_abs is None else f"{force_abs:.2f}N"
            face_text = "NA" if distance_to_face is None else f"{distance_to_face*1000:.0f}mm"
            rospy.loginfo_throttle(
                1.0,
                f"[Approach Wall] cmd={cmd_dist*1000:.0f}mm actual={actual_dist*1000:.0f}mm "
                f"lead={lead*1000:.0f}mm vel={actual_vel*1000:.1f}mm/s "
                f"face={face_text} wall_force={force_text} stall={stall_count}/{PUSH_CONTACT_REQUIRED_CYCLES}")

            if cmd_dist >= target_distance and elapsed > max_duration - 2.0:
                rospy.logwarn_throttle(1.0, "At approach target without contact; waiting for contact gate")

            prev_t = now
            prev_actual_dist = actual_dist
            rate.sleep()

        return 'failed'


class PushWithFeedforwardState(PushingStateBase):
    """Ramp pushing force and hold the wall contact for a fixed duration."""

    def __init__(self):
        PushingStateBase.__init__(
            self,
            outcomes=['succeeded', 'failed', 'timeout'],
            input_keys=['contact_position', 'push_direction', 'push_yaw'],
            output_keys=['push_end_position', 'push_exit_reason'])
        self.diag_pub = rospy.Publisher('~pushing_diag', String, queue_size=1)

    def _scaled_unload_duration(self, min_duration):
        start_force = np.asarray(self.beetle.current_ff_force, dtype=float)
        force_norm = float(np.linalg.norm(start_force))
        if not math.isfinite(force_norm):
            force_norm = 0.0
        duration = max(float(min_duration), force_norm / TOWING_UNLOAD_FORCE_RATE)
        duration = min(TOWING_UNLOAD_MAX_DURATION, duration)
        rospy.loginfo(
            "[Pushing Unload] ff_mag=%.2fN, rate=%.1fN/s, duration=%.1fs",
            force_norm, TOWING_UNLOAD_FORCE_RATE, duration)
        return duration

    def _build_pushing_wrench_command(self, force_world, unified_mode):
        if not unified_mode:
            return force_world, [0.0, 0.0, 0.0], "world_yaw"

        contact_offset_body = np.array([
            self.formation_adapter.base_offset_x + PUSH_CONTACT_DX_FROM_EE,
            self.formation_adapter.base_offset_y + PUSH_CONTACT_DY_FROM_EE,
            self.formation_adapter.total_offset_z + PUSH_CONTACT_DZ_FROM_EE,
        ], dtype=float)
        force_body, torque_body = self.beetle.buildFormationCoGWrench(
            force_world,
            application_offset_body=contact_offset_body,
            yaw_only=True)
        return force_body, torque_body, "fc"

    def _clear_external_wrench(self, hold_pos=None, hold_yaw=None,
                               duration=PUSH_UNLOAD_MIN_DURATION):
        duration = self._scaled_unload_duration(duration)
        zero = [0.0, 0.0, 0.0]
        start_force = np.asarray(self.beetle.current_ff_force, dtype=float)
        start_torque = np.asarray(self.beetle.current_ff_torque, dtype=float)

        rospy.loginfo("Clearing pushing external wrench")
        rate = rospy.Rate(25)
        steps = max(1, int(duration * 25.0))
        for step in range(1, steps + 1):
            if hold_pos is not None:
                self.send_assembly_command_from_end_effector(hold_pos, hold_yaw)
            blend = smoothstep01(float(step) / steps)
            force = ((1.0 - blend) * start_force).tolist()
            torque = ((1.0 - blend) * start_torque).tolist()
            task_weights = (
                build_towing_task_wrench_weights(force, PUSH_TASK_WEIGHT_SCALE)
                if self.beetle.isUnifiedMode() else None)
            self.beetle.addExternalWrench(
                force, torque, frame_id="fc", task_weights=task_weights)
            if rospy.is_shutdown():
                break
            rate.sleep()

        for _ in range(3):
            if hold_pos is not None:
                self.send_assembly_command_from_end_effector(hold_pos, hold_yaw)
            self.beetle.addExternalWrench(zero, zero, frame_id="fc")
            rospy.sleep(0.04)
        self.beetle.clearExternalWrench()
        self.beetle.setAttachModule(None)

    def _finish_push_exit(self, userdata, reason, hold_pos, hold_yaw,
                          clear_duration=PUSH_UNLOAD_MIN_DURATION):
        self._clear_external_wrench(
            hold_pos=hold_pos, hold_yaw=hold_yaw, duration=clear_duration)
        final_pos = self.get_end_effector_position()
        if final_pos is None:
            final_pos = hold_pos
        userdata.push_end_position = np.array(final_pos, dtype=float)
        userdata.push_exit_reason = reason
        rospy.loginfo(
            "Pushing exit: reason=%s, pos=%s",
            reason, FormationUtils.format_vec(final_pos))

    def execute(self, userdata):
        rospy.loginfo("=== Push With Feedforward State ===")

        contact_pos = np.array(userdata.contact_position, dtype=float)
        push_dir = np.array(userdata.push_direction, dtype=float)
        maintain_yaw = userdata.push_yaw
        target_pos = contact_pos + push_dir * PUSH_POSITION_LEAD

        control_mode = 'unified' if self.beetle.isUnifiedMode() else 'leader-follower'
        ramp_time = max(PUSH_FORCE_RAMP_TIME, 1e-3)
        effective_duration = max(
            PUSH_DURATION, ramp_time + PUSH_FULL_FORCE_HOLD_TIME)
        rospy.loginfo(
            "Pushing force %.2fN, ramp %.1fs, duration %.1fs, "
            "full_force_hold %.1fs, effective %.1fs, mode=%s",
            PUSH_FORCE, PUSH_FORCE_RAMP_TIME, PUSH_DURATION,
            PUSH_FULL_FORCE_HOLD_TIME, effective_duration, control_mode)
        if effective_duration > PUSH_DURATION + 1e-3:
            rospy.logwarn(
                "[Pushing] Extending duration to %.1fs so feedforward reaches %.2fN "
                "and holds for %.1fs",
                effective_duration, PUSH_FORCE, PUSH_FULL_FORCE_HOLD_TIME)
        rospy.loginfo("Pushing hold target: %s", FormationUtils.format_vec(target_pos))

        module_masses = rospy.get_param("~module_masses", None)
        module_positions = rospy.get_param("~module_positions", None)
        module_inertias_diag = rospy.get_param("~module_inertias_diag", None)
        attach_ok = self.beetle.setAttachModule(
            self.beetle.module_id,
            module_masses=module_masses,
            module_positions=module_positions,
            module_inertias_diag=module_inertias_diag)
        if not attach_ok and control_mode == 'leader-follower':
            rospy.logwarn("[Pushing] LF task feedforward auto-publish is disabled")

        self.formation_adapter.set_pitch_compensation(False)

        start_time = rospy.get_time()
        rate = rospy.Rate(25)
        unified_mode_seen = self.beetle.isUnifiedMode()
        force_ready_logged = False

        while not rospy.is_shutdown():
            elapsed = rospy.get_time() - start_time
            if elapsed >= effective_duration:
                break

            current_pos = self.get_end_effector_position()
            if current_pos is None:
                rospy.logwarn("Lost EE position during pushing")
                rate.sleep()
                continue

            rpy = self.beetle.getAssemblyRPY()
            if rpy is not None:
                max_rp = max(abs(rpy[0]), abs(rpy[1]))
                if max_rp > PUSH_MAX_ROLL_PITCH:
                    rospy.logerr(
                        "Pushing abort: roll/pitch too large (%.1f deg)",
                        math.degrees(max_rp))
                    self._finish_push_exit(
                        userdata, 'attitude_limit', current_pos, maintain_yaw)
                    return 'timeout'
            else:
                max_rp = 0.0

            unified_mode = self.beetle.isUnifiedMode()
            if unified_mode:
                unified_mode_seen = True
            if unified_mode_seen and not unified_mode:
                rospy.logwarn("Pushing abort: unified mode exited during pushing")
                self._finish_push_exit(
                    userdata, 'unified_exit', current_pos, maintain_yaw)
                return 'timeout'

            ramp = smoothstep01(elapsed / ramp_time)
            ff_world = push_dir * (PUSH_FORCE * ramp)
            if not force_ready_logged and elapsed >= ramp_time:
                force_ready_logged = True
                rospy.loginfo(
                    "[Pushing] Feedforward reached target %.2fN at %.1fs",
                    PUSH_FORCE, elapsed)
            ff_force, ff_torque, ff_frame = self._build_pushing_wrench_command(
                ff_world, unified_mode)
            task_weights = (
                build_towing_task_wrench_weights(ff_force, PUSH_TASK_WEIGHT_SCALE)
                if unified_mode else None)

            self.send_assembly_command_from_end_effector(target_pos, maintain_yaw)
            self.beetle.addExternalWrench(
                force=ff_force, torque=ff_torque, frame_id=ff_frame,
                task_weights=task_weights)

            force_abs, force_norm, _ = self.wall_force_along_push(push_dir)
            wall_force_text = "NA" if force_abs is None else f"{force_abs:.2f}N"
            progress = min(1.0, elapsed / max(effective_duration, 1e-3))
            full_force_hold = max(0.0, elapsed - ramp_time)
            diag_text = (
                f"[Pushing FF] t={elapsed:.1f}s/{effective_duration:.1f}s "
                f"ff_world=({ff_world[0]:.2f},{ff_world[1]:.2f},{ff_world[2]:.2f})N "
                f"mag={np.linalg.norm(ff_world):.2f}N wall_force={wall_force_text} "
                f"full_hold={full_force_hold:.1f}s "
                f"mode={'unified' if unified_mode else 'LF'} frame={ff_frame} "
                f"tau=({ff_torque[0]:.2f},{ff_torque[1]:.2f},{ff_torque[2]:.2f})Nm "
                f"progress={progress*100:.0f}% max_rp={math.degrees(max_rp):.1f}deg")
            rospy.loginfo_throttle(1.0, diag_text)
            self.diag_pub.publish(String(data=diag_text))

            rate.sleep()

        if rospy.is_shutdown():
            hold_pos = self.get_end_effector_position()
            self._finish_push_exit(userdata, 'ros_shutdown', hold_pos, maintain_yaw)
            return 'timeout'

        hold_pos = self.get_end_effector_position()
        if hold_pos is None:
            hold_pos = target_pos
        self._finish_push_exit(
            userdata, 'succeeded', hold_pos, maintain_yaw,
            clear_duration=max(2.0, PUSH_UNLOAD_MIN_DURATION))
        return 'succeeded'


class RetreatFromWallState(PushingStateBase):
    """Optionally retreat after the force is unloaded."""

    def __init__(self):
        PushingStateBase.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['push_direction', 'push_yaw', 'push_end_position'])

    def execute(self, userdata):
        rospy.loginfo("=== Retreat From Wall State ===")

        if not PUSH_RETREAT_AFTER:
            rospy.loginfo("retreat_after_push=false; holding final pose")
            return 'succeeded'

        push_dir = np.array(userdata.push_direction, dtype=float)
        maintain_yaw = userdata.push_yaw
        current_pos = self.get_end_effector_position()
        if current_pos is None:
            current_pos = np.array(userdata.push_end_position, dtype=float)

        retreat_target = np.array(current_pos, dtype=float) - push_dir * PUSH_RETREAT_DISTANCE
        rospy.loginfo(
            "Retreating %.0fmm to %s",
            PUSH_RETREAT_DISTANCE * 1000.0,
            FormationUtils.format_vec(retreat_target))

        success = self.active_position_convergence(
            retreat_target, target_yaw=maintain_yaw,
            pos_thresh=0.05, yaw_thresh=0.1, timeout=20.0,
            max_linear_vel=0.04)
        if not success:
            final_pos = self.get_end_effector_position()
            if final_pos is not None:
                err = np.linalg.norm(np.array(final_pos) - retreat_target)
                if err <= 0.20:
                    rospy.logwarn(
                        "Retreat convergence incomplete but close enough: %.0fmm", err * 1000.0)
                    return 'succeeded'
            rospy.logerr("Retreat failed")
            return 'failed'

        rospy.loginfo("Wall pushing task complete")
        return 'succeeded'


def log_pushing_preflight(module_ids, real_machine, simulation):
    """Log ROS readiness without changing task behavior."""
    master_uri = os.environ.get('ROS_MASTER_URI', '<unset>')
    ros_ip = os.environ.get('ROS_IP', '<unset>')
    ros_hostname = os.environ.get('ROS_HOSTNAME', '<unset>')
    rospy.loginfo("[PushingPreflight] ROS_MASTER_URI=%s ROS_IP=%s ROS_HOSTNAME=%s",
                  master_uri, ros_ip, ros_hostname)
    rospy.loginfo("[PushingPreflight] module_ids=%s real_machine=%s simulation=%s",
                  module_ids, real_machine, simulation)

    try:
        master = rosgraph.Master('/formation_wall_pushing_preflight')
        pubs, subs, srvs = master.getSystemState()
    except Exception as e:
        rospy.logerr("[PushingPreflight] Cannot query ROS master: %s", e)
        return

    published_topics = {name for name, _ in pubs}
    service_names = {name for name, _ in srvs}

    missing_mocap = [
        f"/beetle{module_id}/mocap/pose"
        for module_id in module_ids
        if f"/beetle{module_id}/mocap/pose" not in published_topics
    ]
    if missing_mocap:
        rospy.logwarn("[PushingPreflight] Missing module mocap topics: %s", missing_mocap)

    use_wall_mocap = _as_bool(rospy.get_param("~use_wall_mocap", False))
    use_wall_wrench = _as_bool(rospy.get_param("~use_wall_wrench", False))
    if simulation:
        wall_pose_topic = '/wall/odom'
        if wall_pose_topic not in published_topics:
            rospy.logwarn("[PushingPreflight] Missing wall pose topic: %s", wall_pose_topic)
    elif use_wall_mocap:
        wall_pose_topic = '/wall/mocap/pose'
        if wall_pose_topic not in published_topics:
            rospy.logwarn("[PushingPreflight] Missing wall pose topic: %s", wall_pose_topic)
    else:
        rospy.loginfo("[PushingPreflight] Wall mocap disabled; contact uses stall detection")
    if use_wall_wrench and '/wall/wrench' not in published_topics:
        rospy.logwarn("[PushingPreflight] Missing wall wrench topic: /wall/wrench")

    nav_subscribers = _names_for_topic(subs, '/assembly/uav/nav')
    if nav_subscribers:
        rospy.loginfo("[PushingPreflight] /assembly/uav/nav subscribers: %s", nav_subscribers)
    else:
        rospy.logwarn("[PushingPreflight] No subscriber on /assembly/uav/nav")

    missing_unified_services = [
        f"/beetle{module_id}/controller/set_unified_mode"
        for module_id in module_ids
        if f"/beetle{module_id}/controller/set_unified_mode" not in service_names
    ]
    if missing_unified_services:
        rospy.logwarn("[PushingPreflight] Missing set_unified_mode services: %s",
                      missing_unified_services)


def _load_params():
    global WALL_THICKNESS, WALL_WIDTH, WALL_HEIGHT
    global PUSH_APPROACH_DISTANCE, PUSH_APPROACH_SPEED, PUSH_APPROACH_OVERRUN
    global PUSH_MIN_APPROACH_DISTANCE, PUSH_CONTACT_WRENCH_THRESHOLD
    global PUSH_CONTACT_STALL_LEAD, PUSH_CONTACT_STALL_VELOCITY
    global PUSH_CONTACT_REQUIRED_CYCLES
    global PUSH_FORCE, PUSH_FORCE_RAMP_TIME, PUSH_DURATION
    global PUSH_FULL_FORCE_HOLD_TIME
    global PUSH_POSITION_LEAD, PUSH_TASK_WEIGHT_SCALE, PUSH_MAX_ROLL_PITCH
    global PUSH_UNLOAD_MIN_DURATION
    global PUSH_CONTACT_DX_FROM_EE, PUSH_CONTACT_DY_FROM_EE, PUSH_CONTACT_DZ_FROM_EE
    global PUSH_RETREAT_AFTER, PUSH_RETREAT_DISTANCE

    WALL_THICKNESS = float(rospy.get_param("~wall_thickness", WALL_THICKNESS))
    WALL_WIDTH = float(rospy.get_param("~wall_width", WALL_WIDTH))
    WALL_HEIGHT = float(rospy.get_param("~wall_height", WALL_HEIGHT))

    PUSH_APPROACH_DISTANCE = float(rospy.get_param("~approach_distance", PUSH_APPROACH_DISTANCE))
    PUSH_APPROACH_SPEED = float(rospy.get_param("~approach_speed", PUSH_APPROACH_SPEED))
    PUSH_APPROACH_OVERRUN = float(rospy.get_param("~approach_overrun", PUSH_APPROACH_OVERRUN))
    PUSH_MIN_APPROACH_DISTANCE = float(rospy.get_param("~min_approach_distance", PUSH_MIN_APPROACH_DISTANCE))
    PUSH_CONTACT_WRENCH_THRESHOLD = float(rospy.get_param("~contact_wrench_threshold", PUSH_CONTACT_WRENCH_THRESHOLD))
    PUSH_CONTACT_STALL_LEAD = float(rospy.get_param("~contact_stall_lead", PUSH_CONTACT_STALL_LEAD))
    PUSH_CONTACT_STALL_VELOCITY = float(rospy.get_param("~contact_stall_velocity", PUSH_CONTACT_STALL_VELOCITY))
    PUSH_CONTACT_REQUIRED_CYCLES = int(rospy.get_param("~contact_required_cycles", PUSH_CONTACT_REQUIRED_CYCLES))

    PUSH_FORCE = float(rospy.get_param("~push_force", PUSH_FORCE))
    PUSH_FORCE_RAMP_TIME = max(
        0.1, float(rospy.get_param("~force_ramp_time", PUSH_FORCE_RAMP_TIME)))
    PUSH_DURATION = max(0.1, float(rospy.get_param("~push_duration", PUSH_DURATION)))
    PUSH_FULL_FORCE_HOLD_TIME = max(
        0.0, float(rospy.get_param(
            "~full_force_hold_time", PUSH_FULL_FORCE_HOLD_TIME)))
    PUSH_POSITION_LEAD = float(rospy.get_param("~push_position_lead", PUSH_POSITION_LEAD))
    PUSH_TASK_WEIGHT_SCALE = float(rospy.get_param("~push_task_weight_scale", PUSH_TASK_WEIGHT_SCALE))
    PUSH_MAX_ROLL_PITCH = math.radians(float(rospy.get_param("~max_roll_pitch_deg", 30.0)))
    PUSH_UNLOAD_MIN_DURATION = float(rospy.get_param("~unload_min_duration", PUSH_UNLOAD_MIN_DURATION))

    PUSH_CONTACT_DX_FROM_EE = float(rospy.get_param("~push_contact_dx_from_ee", PUSH_CONTACT_DX_FROM_EE))
    PUSH_CONTACT_DY_FROM_EE = float(rospy.get_param("~push_contact_dy_from_ee", PUSH_CONTACT_DY_FROM_EE))
    PUSH_CONTACT_DZ_FROM_EE = float(rospy.get_param("~push_contact_dz_from_ee", PUSH_CONTACT_DZ_FROM_EE))

    PUSH_RETREAT_AFTER = _as_bool(rospy.get_param("~retreat_after_push", PUSH_RETREAT_AFTER))
    PUSH_RETREAT_DISTANCE = float(rospy.get_param("~retreat_distance", PUSH_RETREAT_DISTANCE))


def main():
    rospy.init_node('formation_wall_pushing')
    _load_params()

    module_ids = _parse_module_ids(rospy.get_param("~module_ids", ""))
    real_machine = _as_bool(rospy.get_param("~real_machine", False))
    simulation = _as_bool(rospy.get_param("~simulation", True))

    rospy.loginfo("=" * 60)
    rospy.loginfo("Formation Wall Pushing Task")
    rospy.loginfo("=" * 60)
    rospy.loginfo(
        "Wall: thickness=%.3fm, width=%.3fm, height=%.3fm",
        WALL_THICKNESS, WALL_WIDTH, WALL_HEIGHT)
    rospy.loginfo(
        "Approach: distance=%.3fm, speed=%.3fm/s, contact_wrench=%.2fN",
        PUSH_APPROACH_DISTANCE, PUSH_APPROACH_SPEED, PUSH_CONTACT_WRENCH_THRESHOLD)
    rospy.loginfo(
        "Push: force=%.2fN, ramp=%.1fs, duration=%.1fs, "
        "full_force_hold=%.1fs, lead=%.0fmm",
        PUSH_FORCE, PUSH_FORCE_RAMP_TIME, PUSH_DURATION,
        PUSH_FULL_FORCE_HOLD_TIME, PUSH_POSITION_LEAD * 1000.0)
    rospy.loginfo("=" * 60)
    log_pushing_preflight(module_ids, real_machine, simulation)

    try:
        sm = smach.StateMachine(outcomes=['success', 'failure'])

        with sm:
            smach.StateMachine.add(
                'PUSHING_INITIALIZE',
                PushingInitializeState(),
                transitions={'succeeded': 'APPROACH_WALL',
                             'failed': 'failure'})

            smach.StateMachine.add(
                'APPROACH_WALL',
                ApproachWallUntilContactState(),
                transitions={'succeeded': 'PUSH_WITH_FEEDFORWARD',
                             'failed': 'failure'})

            smach.StateMachine.add(
                'PUSH_WITH_FEEDFORWARD',
                PushWithFeedforwardState(),
                transitions={'succeeded': 'RETREAT_FROM_WALL',
                             'failed': 'failure',
                             'timeout': 'RETREAT_FROM_WALL'})

            smach.StateMachine.add(
                'RETREAT_FROM_WALL',
                RetreatFromWallState(),
                transitions={'succeeded': 'success',
                             'failed': 'failure'})

        rospy.loginfo("Starting Formation Wall Pushing state machine...")
        outcome = sm.execute()
        rospy.loginfo("Formation Wall Pushing completed with outcome: %s", outcome)

    except Exception as e:
        rospy.logerr("Error during wall pushing state machine execution: %s", e)
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
