#!/usr/bin/env python3
"""Static downward-valve 6D wrench test using a valve-centred arc search."""

import math
import os
import sys
from collections import deque
from types import SimpleNamespace

import numpy as np
import rospkg
import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String


current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)
package_script_dir = os.path.join(rospkg.RosPack().get_path("beetle"), "script")
sys.path.insert(0, package_script_dir)
sys.path.insert(0, os.path.join(package_script_dir, "demos"))
sys.path.insert(0, os.path.join(
    package_script_dir, "demos", "valve_rotation_demo"))
sys.path.insert(0, os.path.join(
    package_script_dir, "demos", "aerial_pushing_demo"))

from beetle_interface import smoothstep01  # noqa: E402
from force_sensor_auto_calib import calibrate_force_sensor  # noqa: E402
from valve_rotation_formation_clean import FormationRotateValveState  # noqa: E402


CONTROL_RATE_HZ = 25.0
STATIC_VALVE_TASK_WRENCH_WEIGHTS = [0.4, 0.4, 0.8, 0.4, 0.8, 2.0]


def normalize_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def circular_start_angle(valve_center, end_effector_position,
                         end_effector_yaw):
    """Choose a continuous circular angle, including the zero-radius case."""
    dx = float(end_effector_position[0] - valve_center[0])
    dy = float(end_effector_position[1] - valve_center[1])
    if math.hypot(dx, dy) <= 1e-9:
        return normalize_angle(float(end_effector_yaw) - math.pi)
    return math.atan2(dy, dx)


def parse_rotation_direction(value, view="below"):
    """Return signed world-Z rotation for a visual CW/CCW request."""
    key = str(value).strip().lower()
    if key in ("clockwise", "cw", "right", "右转", "顺时针"):
        visual_sign = -1
        label = "CW"
    elif key in (
            "counterclockwise", "counter-clockwise", "ccw", "left",
            "左转", "逆时针"):
        visual_sign = 1
        label = "CCW"
    else:
        raise ValueError("rotation_direction must be cw or ccw")

    view_key = str(view).strip().lower()
    if view_key in ("above", "+z", "top"):
        return visual_sign, "%s viewed from above" % label
    if view_key in ("below", "-z", "bottom", "tool"):
        return -visual_sign, "%s viewed from below" % label
    raise ValueError("rotation_view must be above or below")


def _finite_float_param(name, default):
    value = float(rospy.get_param(name, default))
    if not math.isfinite(value):
        raise ValueError("%s must be finite" % name)
    return value


def _positive_float_param(name, default):
    value = _finite_float_param(name, default)
    if value <= 0.0:
        raise ValueError("%s must be positive" % name)
    return value


def _nonnegative_float_param(name, default):
    value = _finite_float_param(name, default)
    if value < 0.0:
        raise ValueError("%s must be nonnegative" % name)
    return value


def minimum_smoothstep_duration(force, torque, minimum,
                                force_rate, torque_rate):
    """Return a duration that bounds a smoothstep's six wrench slew rates."""
    force_norm = float(np.linalg.norm(np.asarray(force, dtype=float)))
    torque_norm = float(np.linalg.norm(np.asarray(torque, dtype=float)))
    return max(
        float(minimum),
        1.5 * force_norm / float(force_rate),
        1.5 * torque_norm / float(torque_rate))


def smooth_start_angular_motion(elapsed, target_speed, ramp_time):
    """Return progress and speed for a smoothstep angular-velocity ramp."""
    elapsed = max(0.0, float(elapsed))
    target_speed = float(target_speed)
    ramp_time = float(ramp_time)
    if ramp_time <= 0.0:
        return target_speed * elapsed, target_speed

    ratio = min(1.0, elapsed / ramp_time)
    speed = target_speed * smoothstep01(ratio)
    if ratio < 1.0:
        progress = target_speed * ramp_time * (
            ratio ** 3 - 0.5 * ratio ** 4)
    else:
        progress = target_speed * (elapsed - 0.5 * ramp_time)
    return progress, speed


class DirectedAngularStallDetector(object):
    """Angular counterpart of aerial pushing's command-lead stall detector."""

    def __init__(self, direction, min_search_angle, lead_threshold,
                 stall_rate, velocity_window, required_cycles):
        self.direction = int(direction)
        self.min_search_angle = float(min_search_angle)
        self.lead_threshold = float(lead_threshold)
        self.stall_rate = float(stall_rate)
        self.velocity_window = float(velocity_window)
        self.required_cycles = int(required_cycles)
        self.reset()

    def reset(self, command_yaw=None, actual_yaw=None, stamp=None):
        self.last_command = command_yaw
        self.last_actual = actual_yaw
        self.command_progress = 0.0
        self.actual_progress = 0.0
        self.history = deque()
        self.candidate_cycles = 0
        if actual_yaw is not None and stamp is not None:
            self.history.append((float(stamp), 0.0))

    def update(self, command_yaw, actual_yaw, stamp):
        if self.last_command is not None:
            self.command_progress += self.direction * normalize_angle(
                command_yaw - self.last_command)
        if self.last_actual is not None:
            self.actual_progress += self.direction * normalize_angle(
                actual_yaw - self.last_actual)
        self.last_command = float(command_yaw)
        self.last_actual = float(actual_yaw)

        stamp = float(stamp)
        self.history.append((stamp, self.actual_progress))
        while (len(self.history) > 2 and
               stamp - self.history[1][0] >= self.velocity_window):
            self.history.popleft()
        window_ready = (
            len(self.history) >= 2 and
            stamp - self.history[0][0] >= 0.8 * self.velocity_window)
        actual_rate = float("inf")
        if window_ready:
            dt = stamp - self.history[0][0]
            actual_rate = abs(
                (self.actual_progress - self.history[0][1]) / dt)

        lead = self.command_progress - self.actual_progress
        candidate = (
            window_ready and
            self.command_progress >= self.min_search_angle and
            lead >= self.lead_threshold and
            actual_rate <= self.stall_rate)
        self.candidate_cycles = self.candidate_cycles + 1 if candidate else 0
        return {
            "contact": self.candidate_cycles >= self.required_cycles,
            "candidate_cycles": self.candidate_cycles,
            "command_progress": self.command_progress,
            "actual_progress": self.actual_progress,
            "directed_lead": lead,
            "actual_rate": actual_rate,
        }


def read_config():
    real_machine = bool(rospy.get_param("~real_machine", False))
    simulation = bool(rospy.get_param("~simulation", True))
    force_sensor_enabled = (
        bool(rospy.get_param("~force_sensor_enabled", True)) and
        real_machine and not simulation)
    direction, direction_label = parse_rotation_direction(
        rospy.get_param("~rotation_direction", "cw"),
        rospy.get_param("~rotation_view", "below"))
    config = SimpleNamespace(
        direction=direction,
        direction_label=direction_label,
        search_speed=math.radians(_positive_float_param(
            "~contact_search_speed_deg_s", 3.0)),
        search_ramp_time=_nonnegative_float_param(
            "~contact_search_ramp_time", 2.0),
        center_tolerance=_positive_float_param(
            "~contact_center_tolerance", 0.100),
        max_search_angle=math.radians(_positive_float_param(
            "~contact_search_max_angle_deg", 45.0)),
        min_search_angle=math.radians(_positive_float_param(
            "~contact_min_search_angle_deg", 2.0)),
        stall_lead=math.radians(_positive_float_param(
            "~contact_stall_lead_deg", 3.0)),
        stall_rate=math.radians(_positive_float_param(
            "~contact_stall_velocity_deg_s", 0.5)),
        velocity_window=_positive_float_param(
            "~contact_velocity_window", 0.4),
        required_cycles=int(rospy.get_param("~contact_required_cycles", 8)),
        target_force=np.array([
            _finite_float_param("~feedforward_force_x", 0.0),
            _finite_float_param("~feedforward_force_y", 0.0),
            _finite_float_param("~feedforward_force_z", 5.0),
        ]),
        target_torque=np.array([
            _finite_float_param("~feedforward_torque_x", 0.0),
            _finite_float_param("~feedforward_torque_y", 0.0),
            direction * _positive_float_param("~target_torque", 1.0),
        ]),
        end_effector_offset_body=np.array([
            _finite_float_param("~tool_center_x", 0.240),
            _finite_float_param("~tool_center_y", 0.0),
            _finite_float_param("~tool_center_z", 0.11053),
        ]),
        valve_feedback_timeout=_positive_float_param(
            "~valve_feedback_timeout", 10.0),
        pose_feedback_max_age=_positive_float_param(
            "~pose_feedback_max_age", 0.5),
        force_sensor_enabled=force_sensor_enabled,
        force_sensor_auto_calib=bool(rospy.get_param(
            "~force_sensor_auto_calib", True)),
        force_sensor_topic=str(rospy.get_param(
            "~force_sensor_topic", "cfs/data")),
        force_sensor_calib_service=str(rospy.get_param(
            "~force_sensor_calib_service", "cfs_sensor_calib")),
        force_sensor_auto_calib_delay=_nonnegative_float_param(
            "~force_sensor_auto_calib_delay", 0.5),
        force_sensor_auto_calib_timeout=_positive_float_param(
            "~force_sensor_auto_calib_timeout", 10.0),
        force_sensor_data_max_age=_positive_float_param(
            "~force_sensor_data_max_age", 0.5),
        ramp_time=_positive_float_param("~wrench_ramp_time", 3.0),
        hold_time=_positive_float_param("~full_torque_hold_time", 3.0),
        unload_min_duration=_positive_float_param(
            "~unload_min_duration", 2.0),
        wrench_force_rate=_positive_float_param(
            "~wrench_force_rate", 3.0),
        wrench_torque_rate=_positive_float_param(
            "~wrench_torque_rate", 0.5),
    )
    if config.required_cycles < 1:
        raise ValueError("~contact_required_cycles must be positive")
    if config.max_search_angle <= config.min_search_angle:
        raise ValueError(
            "~contact_search_max_angle_deg must exceed the minimum search angle")
    return config


class StaticDownwardValveTorqueTest(FormationRotateValveState):
    """Arc-search contact followed by a static 6D wrench profile."""

    def __init__(self, config):
        FormationRotateValveState.__init__(
            self, rotation_direction=config.direction,
            target_rotation=config.max_search_angle,
            end_effector_offset_body=config.end_effector_offset_body)
        self.config = config
        self.phase_pub = rospy.Publisher("~phase", String, queue_size=1, latch=True)
        self.application_wrench_pub = rospy.Publisher(
            "~application_wrench", WrenchStamped, queue_size=1)
        self.cog_wrench_pub = rospy.Publisher(
            "~cog_wrench", WrenchStamped, queue_size=1)
        self.measured_wrench = None
        self.measured_wrench_received_time = None
        self.measured_wrench_topic = (
            config.force_sensor_topic
            if config.force_sensor_enabled else "/valve/wrench")
        self.measured_wrench_sub = rospy.Subscriber(
            self.measured_wrench_topic, WrenchStamped,
            self._measured_wrench_cb,
            queue_size=1)
        self.hold_position = None
        self.hold_yaw = None
        self.last_application_force = np.zeros(3)
        self.last_application_torque = np.zeros(3)
        self.last_cog_force = np.zeros(3)
        self.last_cog_torque = np.zeros(3)

    def _set_phase(self, phase):
        self.phase_pub.publish(String(data=phase))
        rospy.loginfo("[StaticValveTorque] phase=%s", phase)

    def _measured_wrench_cb(self, message):
        measured_wrench = np.array([
            message.wrench.force.x, message.wrench.force.y,
            message.wrench.force.z, message.wrench.torque.x,
            message.wrench.torque.y, message.wrench.torque.z,
        ], dtype=float)
        if not np.all(np.isfinite(measured_wrench)):
            rospy.logwarn_throttle(
                1.0, "Ignoring non-finite wrench from %s",
                self.measured_wrench_topic)
            return
        self.measured_wrench = measured_wrench
        self.measured_wrench_received_time = rospy.get_time()

    def _measured_wrench_is_fresh(self):
        if (self.measured_wrench is None or
                self.measured_wrench_received_time is None):
            return False
        age = rospy.get_time() - self.measured_wrench_received_time
        return (math.isfinite(age) and
                0.0 <= age <= self.config.force_sensor_data_max_age)

    def _prepare_force_sensor(self):
        if not self.config.force_sensor_enabled:
            return True

        self._set_phase("force_sensor_calibration")
        if self.config.force_sensor_auto_calib:
            if not calibrate_force_sensor(
                    service_name=self.config.force_sensor_calib_service,
                    topic_name=self.config.force_sensor_topic,
                    service_timeout=self.config.force_sensor_auto_calib_timeout,
                    topic_timeout=self.config.force_sensor_auto_calib_timeout,
                    startup_delay=self.config.force_sensor_auto_calib_delay,
                    wait_for_sample=True):
                return False

        try:
            sample = rospy.wait_for_message(
                self.config.force_sensor_topic, WrenchStamped,
                timeout=self.config.force_sensor_auto_calib_timeout)
        except rospy.ROSException as exc:
            rospy.logerr("No post-calibration CFS sample: %s", exc)
            return False
        self._measured_wrench_cb(sample)
        if not self._measured_wrench_is_fresh():
            rospy.logerr("Post-calibration CFS sample is not fresh")
            return False
        rospy.loginfo(
            "CFS ready on %s: F=(%.3f, %.3f, %.3f)N "
            "tau=(%.4f, %.4f, %.4f)Nm",
            self.config.force_sensor_topic,
            *self.measured_wrench.tolist())
        return True

    @staticmethod
    def _wrench_message(force, torque, frame_id):
        message = WrenchStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = frame_id
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = force
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = torque
        return message

    def _publish_cog_wrench(self, application_force, application_torque,
                            cog_force, cog_torque):
        application_force = np.asarray(application_force, dtype=float)
        application_torque = np.asarray(application_torque, dtype=float)
        cog_force = np.asarray(cog_force, dtype=float)
        cog_torque = np.asarray(cog_torque, dtype=float)
        task_weights = (
            STATIC_VALVE_TASK_WRENCH_WEIGHTS
            if self.beetle.isUnifiedMode() else None)
        self.beetle.addExternalWrench(
            cog_force.tolist(), cog_torque.tolist(), frame_id="fc",
            task_weights=task_weights)
        self.application_wrench_pub.publish(self._wrench_message(
            application_force.tolist(), application_torque.tolist(), "world"))
        self.cog_wrench_pub.publish(self._wrench_message(
            cog_force.tolist(), cog_torque.tolist(), "fc"))
        self.last_application_force = application_force
        self.last_application_torque = application_torque
        self.last_cog_force = cog_force
        self.last_cog_torque = cog_torque

    def _publish_application_wrench(self, force_world, torque_world):
        cog_force, cog_torque = self.beetle.buildCoGWrench(
            force_world, torque_world,
            application_offset_body=self._ee_offset_body(),
            frame_id="world")
        self._publish_cog_wrench(
            force_world, torque_world, cog_force, cog_torque)

    def _hold_contact_pose(self):
        self.send_assembly_command_from_end_effector(
            self.hold_position, self.hold_yaw)

    def _stop_contact_search_motion(self, fallback_position, fallback_yaw):
        """Replace the streaming velocity command with a pose-only hold."""
        hold_position = self.get_end_effector_position()
        hold_yaw = self.get_end_effector_yaw()
        if hold_position is None or hold_yaw is None:
            hold_position = fallback_position
            hold_yaw = fallback_yaw
        else:
            hold_position = np.asarray(hold_position, dtype=float)
            hold_yaw = float(hold_yaw)
            if (hold_position.size != 3 or
                    not np.all(np.isfinite(hold_position)) or
                    not math.isfinite(hold_yaw)):
                hold_position = fallback_position
                hold_yaw = fallback_yaw
        self.send_assembly_command_from_end_effector(
            hold_position, hold_yaw)

    def _end_effector_feedback_is_fresh(self):
        return self.formation_adapter.has_fresh_end_effector_feedback(
            self.config.pose_feedback_max_age)

    def _wait_for_feedback(self):
        self._set_phase("wait_for_feedback")
        deadline = rospy.get_time() + self.config.valve_feedback_timeout
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            valve_position = self.beetle.getFreshValvePos(
                self.config.pose_feedback_max_age)
            ee_position = self.get_end_effector_position()
            ee_yaw = self.get_end_effector_yaw()
            ee_valid = False
            if ee_position is not None and ee_yaw is not None:
                ee_position = np.asarray(ee_position, dtype=float)
                ee_valid = (
                    self._end_effector_feedback_is_fresh() and
                    ee_position.size == 3 and
                    np.all(np.isfinite(ee_position)) and
                    math.isfinite(float(ee_yaw)))

            if valve_position is not None and ee_valid:
                valve_position = np.asarray(valve_position, dtype=float)
                if (valve_position.size == 3 and
                        np.all(np.isfinite(valve_position))):
                    return valve_position, ee_position, float(ee_yaw)

            if rospy.get_time() >= deadline:
                if not ee_valid:
                    rospy.logerr(
                        "Valve feedback timed out and end-effector feedback "
                        "is unavailable; refusing to start contact search")
                    return None
                valve_position = ee_position.copy()
                rospy.logwarn(
                    "Valve feedback timed out after %.1fs; using current "
                    "end-effector centre (%.3f, %.3f, %.3f)m as the fixed "
                    "rotation centre",
                    self.config.valve_feedback_timeout,
                    *valve_position.tolist())
                return valve_position, ee_position, float(ee_yaw)
            rospy.loginfo_throttle(
                2.0, "Waiting for valve centre and end-effector feedback "
                "(fallback after %.1fs)",
                self.config.valve_feedback_timeout)
            rate.sleep()
        return None

    def _search_contact(self, valve_center):
        self._set_phase("circular_contact_search")
        if not self._end_effector_feedback_is_fresh():
            rospy.logerr(
                "Module pose feedback is stale at contact-search start")
            return False
        start_ee_position = self.get_end_effector_position()
        start_ee_yaw = self.get_end_effector_yaw()
        if start_ee_position is None or start_ee_yaw is None:
            rospy.logerr(
                "End-effector pose is unavailable at contact-search start")
            return False
        start_ee_position = np.asarray(start_ee_position, dtype=float)
        start_ee_yaw = float(start_ee_yaw)
        if (start_ee_position.size != 3 or
                not np.all(np.isfinite(start_ee_position)) or
                not math.isfinite(start_ee_yaw)):
            rospy.logerr(
                "End-effector pose is invalid at contact-search start")
            return False

        # The downward dual-pin task needs only the valve centre in XY;
        # valve orientation is intentionally absent from the trajectory.
        radius = float(np.linalg.norm(
            start_ee_position[:2] - valve_center[:2]))
        if radius > self.config.center_tolerance:
            rospy.logerr(
                "EE/valve center offset %.1fmm exceeds the %.1fmm manual-"
                "insertion tolerance; refusing to start contact search",
                radius * 1000.0,
                self.config.center_tolerance * 1000.0)
            return False
        start_angle = circular_start_angle(
            valve_center, start_ee_position, start_ee_yaw)
        detector = DirectedAngularStallDetector(
            self.config.direction, self.config.min_search_angle,
            self.config.stall_lead, self.config.stall_rate,
            self.config.velocity_window, self.config.required_cycles)
        start_time = rospy.get_time()
        detector.reset(start_ee_yaw, start_ee_yaw, start_time)
        rate = rospy.Rate(CONTROL_RATE_HZ)

        rospy.loginfo(
            "Valve-centred contact search: center=(%.3f, %.3f), "
            "radius=%.1fmm (limit %.1fmm), direction=%s, speed=%.1fdeg/s, "
            "ramp=%.1fs",
            valve_center[0], valve_center[1], radius * 1000.0,
            self.config.center_tolerance * 1000.0,
            self.config.direction_label,
            math.degrees(self.config.direction * self.config.search_speed),
            self.config.search_ramp_time)
        last_target_position = start_ee_position
        last_target_yaw = start_ee_yaw
        while not rospy.is_shutdown():
            if self.beetle.getTaskHaltFlag():
                rospy.logerr("Contact search stopped by operator")
                self._stop_contact_search_motion(
                    last_target_position, last_target_yaw)
                return False
            if not self._end_effector_feedback_is_fresh():
                rospy.logerr(
                    "Module pose feedback became stale during contact search")
                self._stop_contact_search_motion(
                    last_target_position, last_target_yaw)
                return False
            if (self.config.force_sensor_enabled and
                    not self._measured_wrench_is_fresh()):
                rospy.logerr("CFS data became stale during contact search")
                self._stop_contact_search_motion(
                    last_target_position, last_target_yaw)
                return False
            elapsed = rospy.get_time() - start_time
            progress, current_speed = smooth_start_angular_motion(
                elapsed, self.config.search_speed,
                self.config.search_ramp_time)
            progress = min(self.config.max_search_angle, progress)
            signed_speed = self.config.direction * current_speed
            angle = start_angle + self.config.direction * progress
            target_position, _ = self._circular_ee_target(
                valve_center, radius, angle, start_ee_position[2])
            target_yaw = normalize_angle(
                start_ee_yaw + self.config.direction * progress)
            target_velocity = self._circular_ee_velocity(
                radius, signed_speed, angle)
            self.send_assembly_command_from_end_effector(
                target_position, target_yaw,
                linear_vel=target_velocity, angular_vel=signed_speed)
            last_target_position = target_position
            last_target_yaw = target_yaw

            actual_position = self.get_end_effector_position()
            actual_yaw = self.get_end_effector_yaw()
            if actual_position is not None and actual_yaw is not None:
                observation = detector.update(
                    target_yaw, float(actual_yaw), rospy.get_time())
                if observation["contact"]:
                    self.hold_position = np.asarray(actual_position, dtype=float)
                    self.hold_yaw = float(actual_yaw)
                    self._hold_contact_pose()
                    rospy.loginfo(
                        "Static valve contact detected: command=%.1fdeg, "
                        "actual=%.1fdeg, lead=%.1fdeg, rate=%.2fdeg/s",
                        math.degrees(observation["command_progress"]),
                        math.degrees(observation["actual_progress"]),
                        math.degrees(observation["directed_lead"]),
                        math.degrees(observation["actual_rate"]))
                    return True
                rospy.loginfo_throttle(
                    1.0,
                    "[ContactSearch] command=%.1f/%.1fdeg actual=%.1fdeg "
                    "lead=%.1fdeg rate=%s candidate=%d/%d" % (
                        math.degrees(observation["command_progress"]),
                        math.degrees(self.config.max_search_angle),
                        math.degrees(observation["actual_progress"]),
                        math.degrees(observation["directed_lead"]),
                        ("%.2fdeg/s" % math.degrees(observation["actual_rate"])
                         if math.isfinite(observation["actual_rate"])
                         else "warming"),
                        observation["candidate_cycles"],
                        self.config.required_cycles))
            if progress >= self.config.max_search_angle:
                rospy.logerr(
                    "No angular stall contact within %.1fdeg",
                    math.degrees(self.config.max_search_angle))
                self._stop_contact_search_motion(
                    last_target_position, last_target_yaw)
                return False
            rate.sleep()
        return False

    def _run_wrench_profile(self):
        self._set_phase("wrench_ramp_and_hold")
        self._enable_task_wrench_prediction()
        target_cog_force, target_cog_torque = self.beetle.buildCoGWrench(
            self.config.target_force.tolist(),
            self.config.target_torque.tolist(),
            application_offset_body=self._ee_offset_body(),
            frame_id="world")
        ramp_time = minimum_smoothstep_duration(
            target_cog_force, target_cog_torque,
            self.config.ramp_time,
            self.config.wrench_force_rate,
            self.config.wrench_torque_rate)
        start_time = rospy.get_time()
        target_logged = False
        rate = rospy.Rate(CONTROL_RATE_HZ)
        total_duration = ramp_time + self.config.hold_time
        rospy.loginfo(
            "6D wrench ramp %.2fs, then full-target hold %.2fs",
            ramp_time, self.config.hold_time)

        while not rospy.is_shutdown():
            if self.beetle.getTaskHaltFlag():
                rospy.logerr("Wrench profile stopped by operator")
                return False
            if not self._end_effector_feedback_is_fresh():
                rospy.logerr(
                    "Module pose feedback became stale during wrench profile")
                return False
            if (self.config.force_sensor_enabled and
                    not self._measured_wrench_is_fresh()):
                rospy.logerr("CFS data became stale during wrench profile")
                return False
            elapsed = rospy.get_time() - start_time
            scale = smoothstep01(elapsed / ramp_time)
            application_force = scale * self.config.target_force
            application_torque = scale * self.config.target_torque
            self._hold_contact_pose()
            self._publish_application_wrench(
                application_force.tolist(), application_torque.tolist())

            if elapsed >= ramp_time and not target_logged:
                target_logged = True
                rospy.loginfo(
                    "Target application wrench reached: "
                    "F=(%.2f, %.2f, %.2f)N tau=(%.2f, %.2f, %.2f)Nm; "
                    "holding %.1fs",
                    *(self.config.target_force.tolist() +
                      self.config.target_torque.tolist() +
                      [self.config.hold_time]))
            measured = (
                self.measured_wrench
                if self._measured_wrench_is_fresh() else None)
            measured_text = (
                "unavailable" if measured is None else
                "F=(%.2f,%.2f,%.2f)N tau=(%.2f,%.2f,%.2f)Nm" %
                tuple(measured.tolist()))
            rospy.loginfo_throttle(
                1.0,
                "[StaticValveWrench] t=%.1f/%.1fs scale=%.2f "
                "Tz=%.3fNm sensor=%s" % (
                    elapsed, total_duration, scale,
                    application_torque[2], measured_text))
            if elapsed >= total_duration:
                return True
            rate.sleep()
        return False

    def _unload(self):
        self._set_phase("smooth_6d_unload")
        duration = minimum_smoothstep_duration(
            self.last_cog_force, self.last_cog_torque,
            self.config.unload_min_duration,
            self.config.wrench_force_rate,
            self.config.wrench_torque_rate)
        start_application_force = self.last_application_force.copy()
        start_application_torque = self.last_application_torque.copy()
        start_cog_force = self.last_cog_force.copy()
        start_cog_torque = self.last_cog_torque.copy()
        rospy.loginfo("Unloading complete 6D wrench over %.2fs", duration)
        start_time = rospy.get_time()
        rate = rospy.Rate(CONTROL_RATE_HZ)
        while not rospy.is_shutdown():
            elapsed = rospy.get_time() - start_time
            scale = 1.0 - smoothstep01(elapsed / duration)
            self._hold_contact_pose()
            self._publish_cog_wrench(
                scale * start_application_force,
                scale * start_application_torque,
                scale * start_cog_force,
                scale * start_cog_torque)
            if elapsed >= duration:
                break
            rate.sleep()

        zero = [0.0, 0.0, 0.0]
        for _ in range(3):
            self._hold_contact_pose()
            self.beetle.addExternalWrench(zero, zero, frame_id="fc")
            rospy.sleep(0.04)
        self.beetle.clearExternalWrench()
        self.beetle.setAttachModule(None)
        rospy.loginfo("All six feedforward wrench components are zero")
        return not rospy.is_shutdown()

    def execute(self, userdata=None):
        normal_completion = False
        try:
            if not self._prepare_force_sensor():
                return "failed"
            feedback = self._wait_for_feedback()
            if feedback is None:
                return "failed"
            valve_center, _, _ = feedback
            if not self._search_contact(valve_center):
                return "failed"
            if not self._run_wrench_profile():
                return "failed"
            if not self._unload():
                return "failed"
            normal_completion = True
            self._set_phase("complete")
            return "succeeded"
        finally:
            if not normal_completion and self.beetle.external_wrench_active:
                rospy.logwarn("Abnormal exit: immediately clearing 6D wrench")
                self.beetle.clearExternalWrench()
            self.beetle.setAttachModule(None)


def main():
    rospy.init_node("static_downward_valve_torque")
    outcome = "failed"
    try:
        state = StaticDownwardValveTorqueTest(read_config())
        outcome = state.execute()
    except (ValueError, RuntimeError) as exc:
        rospy.logerr("Cannot run static valve torque test: %s", exc)
    except rospy.ROSInterruptException:
        rospy.loginfo("Static valve torque test interrupted")
    rospy.loginfo("Static valve torque test outcome: %s", outcome)
    if outcome != "succeeded" and not rospy.is_shutdown():
        raise SystemExit(1)


if __name__ == "__main__":
    main()
