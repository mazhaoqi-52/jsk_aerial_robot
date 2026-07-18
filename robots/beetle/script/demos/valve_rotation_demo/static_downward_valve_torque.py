#!/usr/bin/env python3
"""Static torque-limit test for a downward-facing four-spoke valve.

The operator first aligns the assembled vehicle and inserts the symmetric
dual-pin tool manually. The node captures the midpoint of the two pins as the
fixed valve centre, searches slowly around that centre until angular stall,
ramps a full 6D feedforward wrench to the requested torque, holds it, and then
unloads all six components.

All application-point wrench values are expressed at the captured valve/tool
centre. They are transformed and shifted to the assembled CoG before a single
WrenchStamped command is published on every control cycle.
"""

import math
import os
import sys
import threading

import numpy as np
import rospkg
import rospy
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String, UInt8
from std_srvs.srv import SetBool
from tf.transformations import euler_from_quaternion, quaternion_matrix


current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(current_dir, "../.."))
sys.path.insert(0, os.path.join(current_dir, ".."))
sys.path.insert(0, current_dir)
# catkin_install_python places this executable in lib/beetle, whereas the
# reusable demo modules are installed under share/beetle/script. Add that path
# explicitly so install-space behavior matches the source/devel-space relay.
package_script_dir = os.path.join(rospkg.RosPack().get_path("beetle"), "script")
sys.path.insert(0, package_script_dir)
sys.path.insert(0, os.path.join(package_script_dir, "demos"))
sys.path.insert(0, os.path.join(
    package_script_dir, "demos", "valve_rotation_demo"))

from beetle_interface import smoothstep01
from downward_valve_torque_common import (
    DirectedAngularStallDetector,
    DualPinToolGeometry,
    finite_vector,
    minimum_unload_duration,
    normalize_angle,
    parse_rotation_direction,
    parse_unified_heartbeat_id,
    slew_toward,
)
from valve_rotation_formation_clean import (
    FormationSingleUAVStateBase,
    FormationUtils,
)


N_MODULE_SESSION_NS = "/beetle/n_module_test"
REQUIRED_END_EFFECTOR_MODEL = "beetle_hyper_fang_upward"
REQUIRED_END_EFFECTOR_LINK = "beetle_hyper_fang_upward_link"


def _as_bool(value):
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, str):
        key = value.strip().lower()
        if key in ("1", "true", "yes", "on"):
            return True
        if key in ("0", "false", "no", "off"):
            return False
        raise ValueError("invalid boolean value: %r" % value)
    if isinstance(value, (int, float, np.integer, np.floating)):
        numeric = float(value)
        if math.isfinite(numeric) and numeric in (0.0, 1.0):
            return bool(int(numeric))
    raise ValueError("invalid boolean value: %r" % value)


def _parse_module_ids(value):
    if isinstance(value, str):
        values = [part.strip() for part in value.split(",") if part.strip()]
    else:
        values = list(value)
    result = [int(module_id) for module_id in values]
    if not result or any(module_id <= 0 for module_id in result):
        raise ValueError("module_ids must contain positive integers")
    if len(set(result)) != len(result):
        raise ValueError("module_ids must not contain duplicates")
    return result


def _vector_slew(current, target, max_rate, dt):
    current = np.asarray(current, dtype=float)
    target = np.asarray(target, dtype=float)
    delta = target - current
    norm = float(np.linalg.norm(delta))
    max_step = max(0.0, float(max_rate)) * max(0.0, float(dt))
    if norm <= max_step or norm <= 1e-12:
        return target
    if max_step <= 0.0:
        return current
    return current + delta * (max_step / norm)


class TorqueTestConfig(object):
    """Read and validate private ROS parameters once at startup."""

    def __init__(self):
        self.real_machine = _as_bool(rospy.get_param("~real_machine", True))
        self.simulation = _as_bool(rospy.get_param("~simulation", False))
        if self.real_machine == self.simulation:
            raise ValueError(
                "exactly one of ~real_machine and ~simulation must be true")

        module_ids_request = str(
            rospy.get_param("~module_ids", "auto")).strip()
        if module_ids_request.lower() in ("", "auto"):
            session_name = N_MODULE_SESSION_NS + "/module_ids"
            if self.simulation and rospy.has_param(session_name):
                module_ids_request = str(rospy.get_param(session_name))
            else:
                module_ids_request = "1,2"
        self.module_ids_text = module_ids_request
        self.module_ids = _parse_module_ids(self.module_ids_text)
        host_request = rospy.get_param("~end_effector_module_id", "auto")
        if str(host_request).strip().lower() in ("", "auto"):
            session_name = N_MODULE_SESSION_NS + "/end_effector_module_id"
            if self.simulation and rospy.has_param(session_name):
                host_request = rospy.get_param(session_name)
            else:
                host_request = self.module_ids[-1]
        try:
            self.end_effector_module_id = int(host_request)
        except (TypeError, ValueError):
            raise ValueError("end_effector_module_id must be an integer")
        if self.end_effector_module_id not in self.module_ids:
            raise ValueError(
                "end_effector_module_id %d is not in module_ids %s" % (
                    self.end_effector_module_id, self.module_ids))

        # When the dedicated n_module_test session is available, refuse a
        # separately launched torque node whose ordered formation or EE host
        # does not match the models that are actually running in Gazebo.
        session_ids_name = N_MODULE_SESSION_NS + "/module_ids"
        session_host_name = N_MODULE_SESSION_NS + "/end_effector_module_id"
        if (self.simulation and rospy.has_param(session_ids_name) and
                rospy.has_param(session_host_name)):
            session_ids = _parse_module_ids(
                rospy.get_param(session_ids_name))
            session_host = int(rospy.get_param(session_host_name))
            if (session_ids != self.module_ids or
                    session_host != self.end_effector_module_id):
                raise ValueError(
                    "static test formation/EE host %s/beetle%d does not "
                    "match n_module_test session %s/beetle%d" % (
                        self.module_ids, self.end_effector_module_id,
                        session_ids, session_host))
            self._validate_simulation_end_effector_model()
        self.require_hover_state = _as_bool(
            rospy.get_param("~require_hover_state", True))
        self.experiment_armed = _as_bool(
            rospy.get_param("~experiment_armed", False))
        self.auto_enable_unified = _as_bool(
            rospy.get_param("~auto_enable_unified", False))

        self.tool_geometry = DualPinToolGeometry(
            center_x=rospy.get_param("~tool_center_x", 0.240),
            half_spacing_y=rospy.get_param(
                "~tool_half_spacing_y", 0.075),
            center_z=rospy.get_param("~tool_center_z", 0.11053))
        self.tool_offset_reference = str(rospy.get_param(
            "~tool_offset_reference", "host_cog")).strip().lower()
        if self.tool_offset_reference not in ("host_cog", "formation_cog"):
            raise ValueError(
                "tool_offset_reference must be host_cog or formation_cog")
        # These are deliberately tight because the pins are already inserted
        # when the first navigation command is issued.  The first limit checks
        # the physical host/tool pose against the nominal formation transform;
        # the second only compares two independently published CoG estimates.
        self.tool_model_tolerance = float(rospy.get_param(
            "~tool_model_tolerance", 0.005))
        self.assembly_centroid_tolerance = float(rospy.get_param(
            "~assembly_centroid_tolerance", 0.015))
        self.host_assembly_yaw_tolerance = math.radians(float(rospy.get_param(
            "~host_assembly_yaw_tolerance_deg", 2.0)))

        self.rotation_view = str(rospy.get_param(
            "~rotation_view", "below"))
        self.rotation_direction, self.rotation_label = parse_rotation_direction(
            rospy.get_param("~rotation_direction", "cw"),
            view=self.rotation_view)

        self.control_rate = float(rospy.get_param("~control_rate", 25.0))
        self.feedback_timeout = float(rospy.get_param(
            "~feedback_timeout", 0.5))
        self.startup_delay = float(rospy.get_param("~startup_delay", 3.0))
        self.capture_duration = float(rospy.get_param(
            "~capture_duration", 0.6))
        self.capture_max_position_span = float(rospy.get_param(
            "~capture_max_position_span", 0.010))
        self.capture_max_yaw_span = math.radians(float(rospy.get_param(
            "~capture_max_yaw_span_deg", 1.0)))
        self.initial_rp_max = math.radians(float(rospy.get_param(
            "~initial_rp_max_deg", 1.0)))

        self.contact_search_speed = math.radians(float(rospy.get_param(
            "~contact_search_speed_deg_s", 3.0)))
        self.contact_search_max_angle = math.radians(float(rospy.get_param(
            "~contact_search_max_angle_deg", 45.0)))
        self.contact_search_timeout = float(rospy.get_param(
            "~contact_search_timeout", 20.0))
        self.contact_min_search_angle = math.radians(float(rospy.get_param(
            "~contact_min_search_angle_deg", 2.0)))
        self.contact_stall_lead = math.radians(float(rospy.get_param(
            "~contact_stall_lead_deg", 3.0)))
        self.contact_stall_rate = math.radians(float(rospy.get_param(
            "~contact_stall_rate_deg_s", 0.5)))
        self.contact_velocity_window = float(rospy.get_param(
            "~contact_velocity_window", 0.4))
        self.contact_required_cycles = int(rospy.get_param(
            "~contact_required_cycles", 8))
        self.contact_estimated_torque_threshold = float(rospy.get_param(
            "~contact_estimated_torque_threshold", 0.10))
        self.contact_estimated_torque_cycles = int(rospy.get_param(
            "~contact_estimated_torque_cycles", 3))
        self.contact_confirmation_mode = str(rospy.get_param(
            "~contact_confirmation_mode", "stall_and_torque")).strip().lower()
        self.allow_unsafe_contact_mode = _as_bool(rospy.get_param(
            "~allow_unsafe_contact_mode", False))
        self.contact_observer_baseline_duration = float(rospy.get_param(
            "~contact_observer_baseline_duration", 0.5))
        self.contact_observer_baseline_span = float(rospy.get_param(
            "~contact_observer_baseline_span", 0.08))
        self.contact_reference_lead = math.radians(float(rospy.get_param(
            "~contact_reference_lead_deg", 0.0)))
        self.contact_settle_time = float(rospy.get_param(
            "~contact_settle_time", 1.0))

        self.application_force = finite_vector([
            rospy.get_param("~feedforward_force_x", 0.0),
            rospy.get_param("~feedforward_force_y", 0.0),
            rospy.get_param("~feedforward_force_z", 5.0),
        ], 3, "feedforward force")
        self.application_torque_xy = finite_vector([
            rospy.get_param("~feedforward_torque_x", 0.0),
            rospy.get_param("~feedforward_torque_y", 0.0),
        ], 2, "feedforward torque XY")
        self.target_torque = float(rospy.get_param("~target_torque", 1.0))
        self.preload_ramp_time = float(rospy.get_param(
            "~preload_ramp_time", 2.0))
        self.force_ramp_rate = float(rospy.get_param(
            "~force_ramp_rate", 3.0))
        self.preload_cog_torque_rate = float(rospy.get_param(
            "~preload_cog_torque_rate", 1.0))
        self.torque_ramp_time = float(rospy.get_param(
            "~torque_ramp_time", 5.0))
        self.torque_ramp_rate = float(rospy.get_param(
            "~torque_ramp_rate", 0.25))
        self.full_torque_hold_time = float(rospy.get_param(
            "~full_torque_hold_time", 3.0))
        self.unload_min_duration = float(rospy.get_param(
            "~unload_min_duration", 2.0))
        self.unload_force_rate = float(rospy.get_param(
            "~unload_force_rate", 3.0))
        self.unload_torque_rate = float(rospy.get_param(
            "~unload_torque_rate", 0.5))

        self.task_weights = finite_vector([
            rospy.get_param("~wrench_weight_fx", 0.4),
            rospy.get_param("~wrench_weight_fy", 0.4),
            rospy.get_param("~wrench_weight_fz", 0.8),
            rospy.get_param("~wrench_weight_tx", 0.4),
            rospy.get_param("~wrench_weight_ty", 0.8),
            rospy.get_param("~wrench_weight_tz", 2.0),
        ], 6, "wrench task weights")

        self.guard_rp_hold = math.radians(float(rospy.get_param(
            "~guard_rp_hold_deg", 8.0)))
        self.guard_rp_relief = math.radians(float(rospy.get_param(
            "~guard_rp_relief_deg", 12.0)))
        self.guard_xy_hold = float(rospy.get_param(
            "~guard_center_xy_hold", 0.020))
        self.guard_xy_relief = float(rospy.get_param(
            "~guard_center_xy_relief", 0.050))
        self.guard_z_hold = float(rospy.get_param(
            "~guard_center_z_hold", 0.015))
        self.guard_z_relief = float(rospy.get_param(
            "~guard_center_z_relief", 0.040))
        self.guard_hold_timeout = float(rospy.get_param(
            "~guard_hold_timeout", 3.0))
        self.max_static_slip_angle = math.radians(float(rospy.get_param(
            "~max_static_slip_angle_deg", 3.0)))
        self.max_estimated_torque = float(rospy.get_param(
            "~max_estimated_torque", 4.0))
        self.require_realized_wrench = _as_bool(rospy.get_param(
            "~require_realized_wrench", True))
        self.min_realized_torque_ratio = float(rospy.get_param(
            "~min_realized_torque_ratio", 0.80))
        self.max_realized_torque = float(rospy.get_param(
            "~max_realized_torque", 5.0))
        self.yaw_reference_error_max = math.radians(float(rospy.get_param(
            "~yaw_reference_error_max_deg", 0.5)))
        self.max_yaw_pid_command = float(rospy.get_param(
            "~max_yaw_pid_command", 0.10))
        self.max_controller_wrench_timeout = float(rospy.get_param(
            "~max_controller_wrench_timeout", 0.75))

        # Independent hard command envelopes. These cannot be disabled with
        # zero; increasing them is an explicit launch-time action.
        self.max_application_force = float(rospy.get_param(
            "~max_application_force", 10.0))
        self.max_application_torque = float(rospy.get_param(
            "~max_application_torque", 3.5))
        self.max_cog_force = float(rospy.get_param(
            "~max_cog_force", 15.0))
        self.max_cog_torque = float(rospy.get_param(
            "~max_cog_torque", 5.0))

        self._validate()

    def _validate_simulation_end_effector_model(self):
        """Fail closed if the live n_module session has the wrong tool."""
        session_model_name = N_MODULE_SESSION_NS + "/end_effector_model"
        if not rospy.has_param(session_model_name):
            raise ValueError(
                "n_module_test session does not record end_effector_model; "
                "relaunch it with end_effector_model:=%s" %
                REQUIRED_END_EFFECTOR_MODEL)
        session_model = str(rospy.get_param(session_model_name)).strip()
        if session_model != REQUIRED_END_EFFECTOR_MODEL:
            raise ValueError(
                "static downward-valve test requires end_effector_model=%s, "
                "but n_module_test loaded %s" % (
                    REQUIRED_END_EFFECTOR_MODEL, session_model))

        marker = 'name="%s"' % REQUIRED_END_EFFECTOR_LINK
        host_description_name = "/beetle%d/robot_description" % (
            self.end_effector_module_id)
        if not rospy.has_param(host_description_name):
            raise ValueError(
                "%s is missing; cannot verify the live end-effector model" %
                host_description_name)
        host_description = str(rospy.get_param(host_description_name))
        if marker not in host_description:
            raise ValueError(
                "beetle%d robot_description does not contain %s" % (
                    self.end_effector_module_id,
                    REQUIRED_END_EFFECTOR_LINK))

        duplicate_hosts = []
        for module_id in self.module_ids:
            if module_id == self.end_effector_module_id:
                continue
            description_name = "/beetle%d/robot_description" % module_id
            if not rospy.has_param(description_name):
                raise ValueError(
                    "%s is missing; cannot verify unique EE placement" %
                    description_name)
            if marker in str(rospy.get_param(description_name)):
                duplicate_hosts.append(module_id)
        if duplicate_hosts:
            raise ValueError(
                "%s appears on non-host modules %s" % (
                    REQUIRED_END_EFFECTOR_LINK, duplicate_hosts))

    def _validate(self):
        if self.auto_enable_unified and self.real_machine:
            raise ValueError(
                "auto_enable_unified is simulation-only; switch hardware "
                "control mode explicitly before arming")
        if self.auto_enable_unified and not self.require_hover_state:
            raise ValueError(
                "auto_enable_unified requires require_hover_state:=true")
        positive = {
            "control_rate": self.control_rate,
            "feedback_timeout": self.feedback_timeout,
            "capture_duration": self.capture_duration,
            "initial_rp_max_deg": self.initial_rp_max,
            "contact_search_speed_deg_s": self.contact_search_speed,
            "contact_search_max_angle_deg": self.contact_search_max_angle,
            "contact_search_timeout": self.contact_search_timeout,
            "contact_velocity_window": self.contact_velocity_window,
            "contact_observer_baseline_duration": (
                self.contact_observer_baseline_duration),
            "contact_observer_baseline_span": (
                self.contact_observer_baseline_span),
            "target_torque": self.target_torque,
            "force_ramp_rate": self.force_ramp_rate,
            "preload_cog_torque_rate": self.preload_cog_torque_rate,
            "torque_ramp_rate": self.torque_ramp_rate,
            "unload_force_rate": self.unload_force_rate,
            "unload_torque_rate": self.unload_torque_rate,
            "max_application_force": self.max_application_force,
            "max_application_torque": self.max_application_torque,
            "max_cog_force": self.max_cog_force,
            "max_cog_torque": self.max_cog_torque,
            "max_estimated_torque": self.max_estimated_torque,
            "max_realized_torque": self.max_realized_torque,
            "yaw_reference_error_max_deg": self.yaw_reference_error_max,
            "max_yaw_pid_command": self.max_yaw_pid_command,
            "max_controller_wrench_timeout": (
                self.max_controller_wrench_timeout),
            "guard_hold_timeout": self.guard_hold_timeout,
            "tool_model_tolerance": self.tool_model_tolerance,
            "assembly_centroid_tolerance": (
                self.assembly_centroid_tolerance),
            "host_assembly_yaw_tolerance_deg": (
                self.host_assembly_yaw_tolerance),
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("%s must be positive and finite" % name)
        if self.control_rate < 10.0:
            raise ValueError("control_rate must be at least 10Hz")
        if self.contact_search_max_angle <= max(
                self.contact_min_search_angle, self.contact_stall_lead):
            raise ValueError(
                "contact_search_max_angle must exceed the contact gates")
        if self.contact_required_cycles < 1:
            raise ValueError("contact_required_cycles must be positive")
        if self.contact_estimated_torque_cycles < 1:
            raise ValueError(
                "contact_estimated_torque_cycles must be positive")
        if self.contact_confirmation_mode not in (
                "stall_and_torque", "stall_only", "torque_only", "either"):
            raise ValueError(
                "contact_confirmation_mode must be stall_and_torque, "
                "stall_only, torque_only, or either")
        if (self.real_machine and
                self.contact_confirmation_mode != "stall_and_torque" and
                not self.allow_unsafe_contact_mode):
            raise ValueError(
                "real-machine operation requires stall_and_torque contact "
                "confirmation; set allow_unsafe_contact_mode:=true only for "
                "an explicitly reviewed alternative")
        if ("torque" in self.contact_confirmation_mode or
                self.contact_confirmation_mode == "either"):
            if (not math.isfinite(self.contact_estimated_torque_threshold) or
                    self.contact_estimated_torque_threshold <= 0.0):
                raise ValueError(
                    "torque-based contact confirmation requires a positive "
                    "contact_estimated_torque_threshold")
        if (self.contact_estimated_torque_threshold >=
                self.max_estimated_torque):
            raise ValueError(
                "contact torque threshold must be below the mandatory "
                "estimated-torque hard limit")
        if any(weight < 0.0 for weight in self.task_weights):
            raise ValueError("wrench task weights must be nonnegative")
        if (not math.isfinite(self.min_realized_torque_ratio) or
                not 0.0 < self.min_realized_torque_ratio <= 1.0):
            raise ValueError(
                "min_realized_torque_ratio must be in (0, 1]")
        if self.guard_rp_relief < self.guard_rp_hold:
            raise ValueError("guard_rp_relief must be >= guard_rp_hold")
        if self.guard_xy_relief < self.guard_xy_hold:
            raise ValueError(
                "guard_center_xy_relief must be >= guard_center_xy_hold")
        if self.guard_z_relief < self.guard_z_hold:
            raise ValueError(
                "guard_center_z_relief must be >= guard_center_z_hold")
        nonnegative = {
            "startup_delay": self.startup_delay,
            "preload_ramp_time": self.preload_ramp_time,
            "torque_ramp_time": self.torque_ramp_time,
            "full_torque_hold_time": self.full_torque_hold_time,
            "unload_min_duration": self.unload_min_duration,
            "contact_reference_lead_deg": self.contact_reference_lead,
            "contact_settle_time": self.contact_settle_time,
            "max_static_slip_angle_deg": self.max_static_slip_angle,
            "capture_max_position_span": self.capture_max_position_span,
            "capture_max_yaw_span_deg": self.capture_max_yaw_span,
            "contact_min_search_angle_deg": self.contact_min_search_angle,
            "contact_stall_lead_deg": self.contact_stall_lead,
            "contact_stall_rate_deg_s": self.contact_stall_rate,
            "contact_estimated_torque_threshold": (
                self.contact_estimated_torque_threshold),
            "guard_rp_hold_deg": self.guard_rp_hold,
            "guard_rp_relief_deg": self.guard_rp_relief,
            "guard_center_xy_hold": self.guard_xy_hold,
            "guard_center_xy_relief": self.guard_xy_relief,
            "guard_center_z_hold": self.guard_z_hold,
            "guard_center_z_relief": self.guard_z_relief,
        }
        for name, value in nonnegative.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("%s must be nonnegative and finite" % name)


class FormationFeedbackWatchdog(object):
    """Local receive/source-advance watchdog for every pose used by the test."""

    def __init__(self, module_ids):
        self._lock = threading.Lock()
        self._state = {}
        self._module_poses = {}
        self._assembly_pose = None
        self._flight_states = {}
        self._unified_heartbeats = {}
        self._topic_to_module = {}
        self._subscribers = []
        for module_id in module_ids:
            name = "/beetle%d/mocap/pose" % module_id
            self._state[name] = [None, None, None]
            self._topic_to_module[name] = int(module_id)
            self._subscribers.append(rospy.Subscriber(
                name, PoseStamped, self._callback,
                callback_args=name, queue_size=1))
            self._subscribers.append(rospy.Subscriber(
                "/beetle%d/flight_state" % module_id,
                UInt8, self._flight_callback,
                callback_args=int(module_id), queue_size=1))
            self._subscribers.append(rospy.Subscriber(
                "/beetle%d/unified_control/heartbeat" % module_id,
                KeyValue, self._unified_heartbeat_callback,
                callback_args=int(module_id), queue_size=1))
        assembly_name = "/assemble/cog/odom"
        self._state[assembly_name] = [None, None, None]
        self._subscribers.append(rospy.Subscriber(
            assembly_name, Odometry, self._callback,
            callback_args=assembly_name, queue_size=1))

    def _callback(self, msg, name):
        now = rospy.get_time()
        stamp = msg.header.stamp.to_sec()
        with self._lock:
            receive_time, old_stamp, advanced_time = self._state[name]
            accepted = (
                math.isfinite(stamp) and stamp > 0.0 and
                (old_stamp is None or stamp > old_stamp))
            if accepted:
                advanced_time = now
                old_stamp = stamp
            self._state[name] = [now, old_stamp, advanced_time]
            module_id = self._topic_to_module.get(name)
            if accepted and module_id is not None:
                position = msg.pose.position
                orientation = msg.pose.orientation
                self._module_poses[module_id] = (
                    np.array([position.x, position.y, position.z], dtype=float),
                    np.array([
                        orientation.x, orientation.y,
                        orientation.z, orientation.w], dtype=float))
            elif accepted and module_id is None:
                position = msg.pose.pose.position
                orientation = msg.pose.pose.orientation
                self._assembly_pose = (
                    np.array([position.x, position.y, position.z], dtype=float),
                    np.array([
                        orientation.x, orientation.y,
                        orientation.z, orientation.w], dtype=float))

    def _flight_callback(self, msg, module_id):
        with self._lock:
            self._flight_states[int(module_id)] = (
                int(msg.data), rospy.get_time())

    def _unified_heartbeat_callback(self, msg, module_id):
        with self._lock:
            self._unified_heartbeats[int(module_id)] = (
                rospy.get_time(), parse_unified_heartbeat_id(msg.value))

    def get_module_pose(self, module_id):
        with self._lock:
            pose = self._module_poses.get(int(module_id))
            if pose is None:
                return None
            return pose[0].copy(), pose[1].copy()

    def get_assembly_pose(self):
        with self._lock:
            if self._assembly_pose is None:
                return None
            return self._assembly_pose[0].copy(), self._assembly_pose[1].copy()

    def wait_ready(self, timeout):
        deadline = rospy.get_time() + max(0.0, float(timeout))
        rate = rospy.Rate(25.0)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            if not self.stale_sources(float("inf")):
                return True
            rate.sleep()
        return not self.stale_sources(float("inf"))

    def stale_sources(self, timeout):
        now = rospy.get_time()
        result = []
        with self._lock:
            state = dict(self._state)
        for name, (receive_time, source_stamp, advanced_time) in state.items():
            if receive_time is None or advanced_time is None:
                result.append(name + " (missing)")
                continue
            if math.isfinite(timeout):
                receive_age = now - receive_time
                advance_age = now - advanced_time
                if (receive_age < 0.0 or receive_age > timeout or
                        advance_age < 0.0 or advance_age > timeout):
                    result.append(
                        "%s (receive %.2fs, advance %.2fs)" % (
                            name, receive_age, advance_age))
        return result

    def flight_state_status(self, timeout, expected_state):
        now = rospy.get_time()
        problems = []
        with self._lock:
            states = dict(self._flight_states)
        for module_id in self._topic_to_module.values():
            state = states.get(module_id)
            if state is None:
                problems.append("beetle%d missing" % module_id)
                continue
            value, receive_time = state
            age = now - receive_time
            if (not math.isfinite(age) or age < 0.0 or age > timeout):
                problems.append(
                    "beetle%d stale %.2fs" % (module_id, age))
            elif value != int(expected_state):
                problems.append(
                    "beetle%d state=%d" % (module_id, value))
        return not problems, problems

    def unified_heartbeat_status(self, timeout):
        """Check the heartbeat emitted only by the active unified C++ path."""
        now = rospy.get_time()
        problems = []
        with self._lock:
            heartbeats = dict(self._unified_heartbeats)
        for module_id in self._topic_to_module.values():
            heartbeat = heartbeats.get(module_id)
            if heartbeat is None:
                problems.append("beetle%d missing" % module_id)
                continue
            receive_time, reported_id = heartbeat
            age = now - receive_time
            if reported_id != module_id:
                problems.append(
                    "beetle%d reports id=%r" % (module_id, reported_id))
            elif (not math.isfinite(age) or age < 0.0 or age > timeout):
                problems.append(
                    "beetle%d stale %.2fs" % (module_id, age))
        return not problems, problems


class FreshWrenchMonitor(object):
    """Store a finite WrenchStamped only while receive/source data are fresh."""

    def __init__(self, topic, allowed_frames=None):
        self.topic = topic
        self.allowed_frames = set(
            str(frame).lstrip("/") for frame in (allowed_frames or []))
        self._lock = threading.Lock()
        self._wrench = None
        self._receive_time = None
        self._source_stamp = None
        self._source_advanced_time = None
        self._subscriber = rospy.Subscriber(
            topic, WrenchStamped, self._callback, queue_size=1)

    def _callback(self, msg):
        now = rospy.get_time()
        frame_id = str(msg.header.frame_id).lstrip("/")
        if self.allowed_frames and frame_id not in self.allowed_frames:
            rospy.logerr_throttle(
                1.0, "%s frame_id=%r is not one of %s",
                self.topic, msg.header.frame_id,
                sorted(self.allowed_frames))
            return
        stamp = msg.header.stamp.to_sec()
        if not math.isfinite(stamp) or stamp <= 0.0:
            rospy.logerr_throttle(
                1.0, "%s has an invalid/non-advancing source stamp",
                self.topic)
            return
        values = np.array([
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
        ], dtype=float)
        if not np.all(np.isfinite(values)):
            return
        with self._lock:
            if (self._source_stamp is not None and
                    stamp <= self._source_stamp):
                return
            if (self._source_stamp is None or
                    stamp > self._source_stamp):
                self._source_advanced_time = now
                self._source_stamp = stamp
            self._receive_time = now
            self._wrench = values

    def get_fresh(self, timeout):
        sample = self.get_fresh_sample(timeout)
        return None if sample is None else sample[0]

    def get_fresh_sample(self, timeout):
        """Return ``(wrench, source_stamp)`` for a fresh unique-able sample."""
        now = rospy.get_time()
        with self._lock:
            if (self._wrench is None or self._receive_time is None or
                    self._source_advanced_time is None):
                return None
            ages = (
                now - self._receive_time,
                now - self._source_advanced_time)
            if not all(
                    math.isfinite(age) and 0.0 <= age <= timeout
                    for age in ages):
                return None
            return self._wrench.copy(), float(self._source_stamp)

    def wait_ready(self, timeout, freshness):
        deadline = rospy.get_time() + max(0.0, float(timeout))
        rate = rospy.Rate(25.0)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            if self.get_fresh(freshness) is not None:
                return True
            rate.sleep()
        return self.get_fresh(freshness) is not None


class StaticDownwardValveTorqueTest(FormationSingleUAVStateBase):
    """One shared formation interface with modular experiment phases."""

    def __init__(self, config):
        self.config = config
        rospy.set_param("~module_ids", config.module_ids_text)
        rospy.set_param(
            "~end_effector_module_id", config.end_effector_module_id)
        FormationSingleUAVStateBase.__init__(
            self, outcomes=["succeeded", "failed"],
            end_effector_module_id=config.end_effector_module_id)
        self._apply_tool_geometry()
        self.feedback = FormationFeedbackWatchdog(config.module_ids)
        self.formation_observer = FreshWrenchMonitor(
            "/assemble/formation_observer/est_ext_wrench",
            allowed_frames=["formation_body"])
        self.realized_wrench = FreshWrenchMonitor(
            "/assemble/debug/formation_wrench",
            allowed_frames=["assembly_cog"])
        self.application_wrench_pub = rospy.Publisher(
            "~commanded_application_wrench", WrenchStamped, queue_size=1)
        self.cog_wrench_pub = rospy.Publisher(
            "~commanded_cog_wrench", WrenchStamped, queue_size=1)
        self.phase_pub = rospy.Publisher(
            "~phase", String, queue_size=1, latch=True)
        self._last_application_force = np.zeros(3)
        self._last_application_torque = np.zeros(3)
        self._last_cog_force = np.zeros(3)
        self._last_cog_torque = np.zeros(3)
        self._pivot = None
        self._hold_yaw = None
        self._last_safe_assembly_position = None
        self._last_safe_assembly_yaw = None
        self._mode_at_start = None
        self._runtime_realized_required = False
        self._runtime_realized_ratio_required = False
        self._normal_completion = False
        self._identity_check_time = -float("inf")
        self._identity_check_result = (True, "")
        self._ownership_check_time = -float("inf")
        self._ownership_check_result = (True, "")
        self._watchdog_check_time = -float("inf")
        self._watchdog_check_result = (True, "")
        self._unified_restore_modes = None

    def _apply_tool_geometry(self):
        tf_calculator = self.formation_adapter.tf_calculator
        geometry = self.config.tool_geometry
        assembly_to_host = (
            tf_calculator.
            calculate_assembly_to_end_effector_host_transform())
        if self.config.tool_offset_reference == "host_cog":
            host_center_x = geometry.center_x
            host_center_z = geometry.center_z
        else:
            host_center_x = (
                geometry.center_x - assembly_to_host["offset_x"])
            host_center_z = (
                geometry.center_z - assembly_to_host["offset_z"])

        tf_calculator.dual_fang_center_offset = host_center_x
        tf_calculator.end_effector_offset_z = host_center_z
        self.host_tool_offset_body = np.array([
            host_center_x, geometry.center_y, host_center_z], dtype=float)
        self.formation_adapter._calculate_offset_parameters()

        self.application_offset_body = np.array([
            self.formation_adapter.base_offset_x,
            self.formation_adapter.base_offset_y,
            self.formation_adapter.total_offset_z,
        ], dtype=float)
        expected = (
            np.array(geometry.center, dtype=float)
            if self.config.tool_offset_reference == "formation_cog" else
            np.array([
                assembly_to_host["offset_x"] + geometry.center_x,
                assembly_to_host["offset_y"] + geometry.center_y,
                assembly_to_host["offset_z"] + geometry.center_z,
            ], dtype=float))
        if not np.allclose(self.application_offset_body, expected, atol=1e-9):
            raise RuntimeError(
                "tool-centre transform mismatch: calculated=%s expected=%s" % (
                    self.application_offset_body, expected))

    @staticmethod
    def _wrench_message(force, torque, frame_id):
        msg = WrenchStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = torque
        return msg

    def _set_phase(self, phase):
        self.phase_pub.publish(String(data=phase))
        rospy.loginfo("[DownwardValveTorque] phase=%s", phase)

    def _measured_tool_center(self):
        """Return the physical tool centre from full EE-host mocap pose.

        FormationAdapter's legacy end-effector getter is yaw-only by default.
        That is sufficient for a level target transform but unsafe as feedback:
        with a long X offset, a few degrees of pitch move the real tool centre
        by centimetres. The guard and contact capture therefore use the full
        quaternion from the host module mocap. This also exposes connector
        torsion that an averaged assembly orientation could hide.
        """
        host_pose = self.feedback.get_module_pose(
            self.end_effector_module_id)
        if host_pose is None:
            return None
        host_position, quaternion = host_pose
        if (host_position.size != 3 or quaternion.size != 4 or
                not np.all(np.isfinite(host_position)) or
                not np.all(np.isfinite(quaternion)) or
                np.linalg.norm(quaternion) < 1e-6):
            return None
        rotation = quaternion_matrix(
            quaternion / np.linalg.norm(quaternion))[:3, :3]
        return host_position + np.dot(rotation, self.host_tool_offset_body)

    def _measured_host_yaw(self):
        host_pose = self.feedback.get_module_pose(
            self.end_effector_module_id)
        if host_pose is None:
            return None
        quaternion = host_pose[1]
        if (not np.all(np.isfinite(quaternion)) or
                np.linalg.norm(quaternion) < 1e-6):
            return None
        rpy = euler_from_quaternion(
            quaternion / np.linalg.norm(quaternion))
        return float(rpy[2]) if math.isfinite(rpy[2]) else None

    def _measured_assembly_pose(self):
        pose = self.feedback.get_assembly_pose()
        if pose is None:
            return None
        position = np.asarray(pose[0], dtype=float)
        quaternion = np.asarray(pose[1], dtype=float)
        if (position.size != 3 or quaternion.size != 4 or
                not np.all(np.isfinite(position)) or
                not np.all(np.isfinite(quaternion)) or
                np.linalg.norm(quaternion) < 1e-6):
            return None
        quaternion /= np.linalg.norm(quaternion)
        return position, quaternion

    def _measured_assembly_rpy(self):
        pose = self._measured_assembly_pose()
        if pose is None:
            return None
        rpy = np.asarray(euler_from_quaternion(pose[1]), dtype=float)
        return rpy if np.all(np.isfinite(rpy)) else None

    def _measured_assembly_yaw(self):
        rpy = self._measured_assembly_rpy()
        return None if rpy is None else float(rpy[2])

    def _measured_host_rpy(self):
        host_pose = self.feedback.get_module_pose(
            self.end_effector_module_id)
        if host_pose is None:
            return None
        quaternion = host_pose[1]
        if (not np.all(np.isfinite(quaternion)) or
                np.linalg.norm(quaternion) < 1e-6):
            return None
        rpy = np.asarray(euler_from_quaternion(
            quaternion / np.linalg.norm(quaternion)), dtype=float)
        return rpy if np.all(np.isfinite(rpy)) else None

    def _formation_geometry_status(self):
        """Cross-check configured formation/tool geometry against feedback."""
        assembly_pose = self._measured_assembly_pose()
        tool_position = self._measured_tool_center()
        host_yaw = self._measured_host_yaw()
        if (assembly_pose is None or tool_position is None or
                host_yaw is None):
            return False, "assembly/host geometry feedback is unavailable", None

        module_positions = []
        for module_id in self.config.module_ids:
            pose = self.feedback.get_module_pose(module_id)
            if pose is None or not np.all(np.isfinite(pose[0])):
                return (
                    False,
                    "module %d geometry feedback is unavailable" % module_id,
                    None)
            module_positions.append(pose[0])

        assembly_position, assembly_quaternion = assembly_pose
        assembly_rotation = quaternion_matrix(assembly_quaternion)[:3, :3]
        predicted_tool = assembly_position + np.dot(
            assembly_rotation, self.application_offset_body)
        tool_error = float(np.linalg.norm(tool_position - predicted_tool))
        centroid = np.mean(np.asarray(module_positions), axis=0)
        centroid_error = float(np.linalg.norm(centroid - assembly_position))
        assembly_yaw = float(euler_from_quaternion(assembly_quaternion)[2])
        yaw_error = abs(normalize_angle(host_yaw - assembly_yaw))
        metrics = {
            "tool_model_error": tool_error,
            "centroid_error": centroid_error,
            "host_assembly_yaw_error": yaw_error,
            "predicted_tool": predicted_tool,
            "measured_tool": np.asarray(tool_position, dtype=float),
        }
        if tool_error > self.config.tool_model_tolerance:
            return (
                False,
                "tool/formation model error %.1fmm exceeds %.1fmm" % (
                    tool_error * 1000.0,
                    self.config.tool_model_tolerance * 1000.0),
                metrics)
        if centroid_error > self.config.assembly_centroid_tolerance:
            return (
                False,
                "module centroid/assembly CoG error %.1fmm exceeds %.1fmm" % (
                    centroid_error * 1000.0,
                    self.config.assembly_centroid_tolerance * 1000.0),
                metrics)
        if yaw_error > self.config.host_assembly_yaw_tolerance:
            return (
                False,
                "host/assembly yaw mismatch %.2fdeg exceeds %.2fdeg" % (
                    math.degrees(yaw_error),
                    math.degrees(
                        self.config.host_assembly_yaw_tolerance)),
                metrics)
        return True, "", metrics

    def _command_path_status(self):
        if self.beetle.nav_pub.get_num_connections() < 1:
            return False, "/assembly/uav/nav has no subscriber"
        if self._mode_at_start:
            wrench_publisher = self.beetle.formation_wrench_pub
            weight_publisher = self.beetle.formation_wrench_weights_pub
        else:
            wrench_publisher = self.beetle.desired_ext_wrench_pub
            weight_publisher = self.beetle.desired_ext_wrench_weights_pub
        if wrench_publisher.get_num_connections() < 1:
            return False, "selected wrench route has no subscriber"
        if weight_publisher.get_num_connections() < 1:
            return False, "selected wrench-weight route has no subscriber"
        return True, ""

    def _exclusive_command_publishers_status(self):
        """Reject another node publishing a selected command topic."""
        now = rospy.get_time()
        if now - self._ownership_check_time < 0.5:
            return self._ownership_check_result
        self._ownership_check_time = now
        # The task requires unified control.  The branch is retained so this
        # ownership check remains conservative if called before the unified
        # preflight gate has reported an error.
        publishers = [self.beetle.nav_pub]
        if self._mode_at_start:
            publishers.extend([
                self.beetle.formation_wrench_pub,
                self.beetle.formation_wrench_weights_pub,
            ])
        command_topics = set()
        for publisher in publishers:
            topic = getattr(publisher, "resolved_name", None)
            if topic:
                command_topics.add(topic)
        try:
            code, message, state = rospy.get_master().getSystemState()
            if code != 1:
                raise RuntimeError(message)
            topic_publishers = dict(state[0])
            own_name = rospy.get_name()
            conflicts = []
            for topic in sorted(command_topics):
                others = [
                    node for node in topic_publishers.get(topic, [])
                    if node != own_name]
                if others:
                    conflicts.append("%s <- %s" % (topic, others))
            result = (
                (False, "other command publishers: %s" % conflicts)
                if conflicts else (True, ""))
        except Exception as exc:
            result = (False, "cannot verify exclusive command ownership: %s" % exc)
        self._ownership_check_result = result
        return result

    def _controller_watchdog_status(self):
        """Verify every module stays unified and has a bounded FF timeout."""
        now = rospy.get_time()
        if now - self._watchdog_check_time < 0.5:
            return self._watchdog_check_result
        self._watchdog_check_time = now
        result = (True, "")
        for module_id in self.config.module_ids:
            mode_name = (
                "/beetle%d/controller/unified_control_mode" % module_id)
            if not rospy.has_param(mode_name):
                result = (False, "%s is missing" % mode_name)
                break
            try:
                unified = _as_bool(rospy.get_param(mode_name))
            except (TypeError, ValueError) as exc:
                result = (
                    False, "%s is invalid: %s" % (mode_name, exc))
                break
            if not unified:
                result = (False, "%s is false" % mode_name)
                break

            timeout_name = (
                "/beetle%d/controller/desired_wrench_timeout" % module_id)
            if not rospy.has_param(timeout_name):
                result = (False, "%s is missing" % timeout_name)
                break
            try:
                timeout = float(rospy.get_param(timeout_name))
            except (TypeError, ValueError):
                result = (False, "%s is not numeric" % timeout_name)
                break
            if (not math.isfinite(timeout) or timeout <= 0.0 or
                    timeout > self.config.max_controller_wrench_timeout):
                result = (
                    False,
                    "%s=%.3f must be in (0, %.3f]s" % (
                        timeout_name, timeout,
                        self.config.max_controller_wrench_timeout))
                break
        self._watchdog_check_result = result
        return result

    def _yaw_pid_status(self):
        pid = self.beetle.getFreshControlPid(self.config.feedback_timeout)
        if pid is None:
            return False, "formation pose PID feedback is stale", None
        try:
            yaw_error = float(pid.yaw.err_p)
            yaw_command = float(pid.yaw.total[0])
        except (AttributeError, IndexError, TypeError, ValueError):
            return False, "formation yaw PID feedback is malformed", None
        if not math.isfinite(yaw_error) or not math.isfinite(yaw_command):
            return False, "formation yaw PID feedback is non-finite", None
        metrics = {
            "yaw_error": yaw_error,
            "yaw_pid_command": yaw_command,
        }
        if abs(yaw_error) > self.config.yaw_reference_error_max:
            return (
                False,
                "yaw reference error %.2fdeg exceeds %.2fdeg" % (
                    math.degrees(abs(yaw_error)),
                    math.degrees(self.config.yaw_reference_error_max)),
                metrics)
        if abs(yaw_command) > self.config.max_yaw_pid_command:
            return (
                False,
                "yaw PID command %.3f exceeds %.3f" % (
                    abs(yaw_command), self.config.max_yaw_pid_command),
                metrics)
        return True, "", metrics

    def _runtime_identity_status(self):
        now = rospy.get_time()
        if now - self._identity_check_time < 0.5:
            return self._identity_check_result
        self._identity_check_time = now
        leader_param = (
            "/beetle%d/assembly_leader_id" %
            self.end_effector_module_id)
        if not rospy.has_param(leader_param):
            result = (False, "assembly_leader_id disappeared")
        else:
            runtime_leader = int(rospy.get_param(leader_param))
            ids_param = "/beetle%d/assembly_ids" % runtime_leader
            if runtime_leader != int(self.beetle.wrench_target_id):
                result = (False, "assembly leader changed")
            elif rospy.has_param(ids_param):
                runtime_ids = _parse_module_ids(rospy.get_param(ids_param))
                result = (
                    (True, "") if set(runtime_ids) == set(self.config.module_ids)
                    else (False, "assembly membership changed"))
            else:
                # The current C++ assembly implementation publishes the leader
                # id but not membership.  Continuous mocap/CoG geometry checks
                # below are the authoritative fallback in that configuration.
                result = (True, "")
        self._identity_check_result = result
        return result

    def _validate_wrench_envelope(self, force_world, torque_world,
                                  force_body, torque_body):
        vectors = [
            np.asarray(force_world, dtype=float),
            np.asarray(torque_world, dtype=float),
            np.asarray(force_body, dtype=float),
            np.asarray(torque_body, dtype=float),
        ]
        if any(vector.size != 3 or not np.all(np.isfinite(vector))
               for vector in vectors):
            raise ValueError("refusing non-finite or malformed 6D wrench")
        limits = [
            ("application force", np.linalg.norm(vectors[0]),
             self.config.max_application_force),
            ("application torque", np.linalg.norm(vectors[1]),
             self.config.max_application_torque),
            ("CoG force", np.linalg.norm(vectors[2]),
             self.config.max_cog_force),
            ("CoG torque", np.linalg.norm(vectors[3]),
             self.config.max_cog_torque),
        ]
        for label, magnitude, limit in limits:
            if magnitude > limit + 1e-9:
                raise ValueError(
                    "%s %.3f exceeds hard limit %.3f" % (
                        label, magnitude, limit))
        if self._mode_at_start:
            cog_wrench = np.concatenate([vectors[2], vectors[3]])
            for index, (value, weight) in enumerate(zip(
                    cog_wrench, self.config.task_weights)):
                if abs(value) > 1e-6 and weight <= 0.0:
                    raise ValueError(
                        "active CoG wrench axis %d has zero task weight" % index)

    def _publish_application_wrench(self, force_world, torque_world):
        force_world = finite_vector(force_world, 3, "application force")
        torque_world = finite_vector(torque_world, 3, "application torque")
        assembly_pose = self._measured_assembly_pose()
        if assembly_pose is None:
            raise ValueError(
                "trusted assembly orientation is unavailable for wrench "
                "transformation")
        force_body, torque_body = self.beetle.buildCoGWrench(
            force_world,
            torque_world,
            application_offset_body=self.application_offset_body,
            frame_id="world",
            orientation_quaternion=assembly_pose[1])
        self._publish_wrench_sample(
            force_world, torque_world, force_body, torque_body)

    def _publish_wrench_sample(self, force_world, torque_world,
                               force_body, torque_body):
        """Publish one precomputed application/CoG 6D wrench sample."""
        force_world = finite_vector(force_world, 3, "application force")
        torque_world = finite_vector(torque_world, 3, "application torque")
        force_body = finite_vector(force_body, 3, "CoG force")
        torque_body = finite_vector(torque_body, 3, "CoG torque")
        self._validate_wrench_envelope(
            force_world, torque_world, force_body, torque_body)

        unified_now = _as_bool(self.beetle.isUnifiedMode())
        command_nonzero = (
            np.linalg.norm(force_body) > 1e-9 or
            np.linalg.norm(torque_body) > 1e-9)
        if (command_nonzero and self._mode_at_start is not None and
                unified_now != self._mode_at_start):
            raise RuntimeError(
                "refusing to publish nonzero wrench after controller mode "
                "changed")
        self.beetle.addExternalWrench(
            force_body,
            torque_body,
            frame_id="fc",
            task_weights=(
                self.config.task_weights if self._mode_at_start else None),
            route_unified=self._mode_at_start)
        self.application_wrench_pub.publish(self._wrench_message(
            force_world, torque_world, "world"))
        self.cog_wrench_pub.publish(self._wrench_message(
            force_body, torque_body, "fc"))
        self._last_application_force = np.asarray(force_world, dtype=float)
        self._last_application_torque = np.asarray(torque_world, dtype=float)
        self._last_cog_force = np.asarray(force_body, dtype=float)
        self._last_cog_torque = np.asarray(torque_body, dtype=float)

    def _publish_zero_all_routes(self):
        zero_wrench = self._wrench_message(
            [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "fc")
        zero_weights = Float32MultiArray(data=[0.0] * 6)
        zero_module_wrenches = {
            module_id: ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            for module_id in self.config.module_ids}
        for _ in range(3):
            # Always clear LF task-share topics, even if this process started
            # in unified mode and never enabled auto-publish. Otherwise a
            # unified->LF mode transition could briefly combine a fresh zero
            # desired-wrench heartbeat with stale per-module task shares.
            self.beetle.setInternalWrenchPerModule(zero_module_wrenches)
            for name in ("formation_wrench_pub", "desired_ext_wrench_pub"):
                publisher = getattr(self.beetle, name, None)
                if publisher is not None:
                    publisher.publish(zero_wrench)
            for name in (
                    "formation_wrench_weights_pub",
                    "desired_ext_wrench_weights_pub"):
                publisher = getattr(self.beetle, name, None)
                if publisher is not None:
                    publisher.publish(zero_weights)
            self.application_wrench_pub.publish(self._wrench_message(
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "world"))
            self.cog_wrench_pub.publish(zero_wrench)
            try:
                rospy.sleep(0.04)
            except rospy.ROSInterruptException:
                break
        self.beetle.clearExternalWrench()
        self._last_application_force = np.zeros(3)
        self._last_application_torque = np.zeros(3)
        self._last_cog_force = np.zeros(3)
        self._last_cog_torque = np.zeros(3)

    @staticmethod
    def _set_module_unified_mode(module_id, enabled):
        service_name = "/beetle%d/controller/set_unified_mode" % module_id
        try:
            rospy.wait_for_service(service_name, timeout=2.0)
            response = rospy.ServiceProxy(service_name, SetBool)(bool(enabled))
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("Unified-mode service %s failed: %s", service_name, exc)
            return False
        if not response.success:
            rospy.logerr(
                "Unified-mode service %s rejected request: %s",
                service_name, response.message)
            return False
        return True

    def _restore_auto_enabled_unified_mode(self):
        if not self._unified_restore_modes:
            return
        restore_modes = self._unified_restore_modes
        self._unified_restore_modes = None
        failures = []
        for module_id, original_mode in restore_modes.items():
            if not self._set_module_unified_mode(module_id, original_mode):
                failures.append(module_id)
        if failures:
            rospy.logerr(
                "Failed to restore unified mode for modules %s; operator "
                "intervention is required", failures)
        else:
            rospy.loginfo(
                "Restored pre-test unified mode for modules %s",
                sorted(restore_modes))

    def _auto_enable_unified_mode(self):
        """Enter unified mode only after assembly identity and HOVER checks."""
        if not self.config.auto_enable_unified:
            return True

        original_modes = {}
        for module_id in self.config.module_ids:
            param_name = (
                "/beetle%d/controller/unified_control_mode" % module_id)
            if not rospy.has_param(param_name):
                rospy.logerr("Cannot auto-enable unified: %s is missing", param_name)
                return False
            try:
                original_modes[module_id] = _as_bool(
                    rospy.get_param(param_name))
            except (TypeError, ValueError) as exc:
                rospy.logerr(
                    "Cannot auto-enable unified: %s is invalid: %s",
                    param_name, exc)
                return False

        # Record every false mode before the first service call. A leader call
        # may auto-latch followers, but all modules must still be restored to
        # their pre-test state on every exit path.
        self._unified_restore_modes = {
            module_id: False
            for module_id, enabled in original_modes.items() if not enabled}
        for module_id, enabled in original_modes.items():
            if enabled:
                continue
            if not self._set_module_unified_mode(module_id, True):
                self._restore_auto_enabled_unified_mode()
                return False

        deadline = rospy.get_time() + 3.0
        all_enabled = False
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            try:
                all_enabled = all(_as_bool(rospy.get_param(
                    "/beetle%d/controller/unified_control_mode" % module_id,
                    False)) for module_id in self.config.module_ids)
            except (TypeError, ValueError):
                all_enabled = False
            if all_enabled:
                break
            rospy.sleep(0.04)
        if not all_enabled:
            rospy.logerr("Unified mode did not become active on every module")
            self._restore_auto_enabled_unified_mode()
            return False
        rospy.loginfo(
            "Unified mode enabled after assembly/HOVER verification: %s",
            self.config.module_ids)
        return True

    def _preflight(self):
        self._set_phase("preflight")
        if not self.config.experiment_armed:
            rospy.logerr(
                "Torque output is DISARMED. Re-run with "
                "experiment_armed:=true only after the formation is hovering, "
                "the pins are inserted, and the area is clear.")
            return False
        if self.beetle.getTaskHaltFlag():
            rospy.logerr(
                "Joystick halt is already latched; release/reset it before "
                "arming a new experiment")
            return False
        if not self.feedback.wait_ready(timeout=5.0):
            rospy.logerr(
                "Pose feedback is not ready: %s",
                self.feedback.stale_sources(float("inf")))
            return False

        host_id = self.end_effector_module_id
        leader_param = "/beetle%d/assembly_leader_id" % host_id
        if not rospy.has_param(leader_param):
            rospy.logerr(
                "%s is unavailable. The wrench interface may have been "
                "constructed before assembly routing was ready; relaunch this "
                "test after assembly completes.", leader_param)
            return False
        runtime_leader = int(rospy.get_param(leader_param))
        if runtime_leader != int(self.beetle.wrench_target_id):
            rospy.logerr(
                "Stale wrench routing: BeetleInterface targets beetle%d but "
                "runtime assembly leader is beetle%d; relaunch the test",
                self.beetle.wrench_target_id, runtime_leader)
            return False
        assembly_ids_param = "/beetle%d/assembly_ids" % runtime_leader
        if rospy.has_param(assembly_ids_param):
            runtime_ids = _parse_module_ids(
                rospy.get_param(assembly_ids_param))
            if set(runtime_ids) != set(self.config.module_ids):
                rospy.logerr(
                    "Configured module_ids=%s do not match runtime "
                    "assembly_ids=%s", self.config.module_ids, runtime_ids)
                return False
            rospy.loginfo(
                "Assembly identity verified: C++ leader=beetle%d, "
                "members=%s, configured EE host=beetle%d",
                runtime_leader, runtime_ids, host_id)
        else:
            rospy.logwarn(
                "%s is not published by the current assembly controller; "
                "membership/order will be verified from continuous mocap/CoG "
                "geometry instead", assembly_ids_param)

        if self.config.require_hover_state:
            deadline = rospy.get_time() + 2.0
            flight_ok, flight_problems = self.feedback.flight_state_status(
                self.config.feedback_timeout, self.beetle.HOVER_STATE)
            while (not rospy.is_shutdown() and not flight_ok and
                   rospy.get_time() < deadline):
                rospy.sleep(0.04)
                flight_ok, flight_problems = (
                    self.feedback.flight_state_status(
                        self.config.feedback_timeout,
                        self.beetle.HOVER_STATE))
            if not flight_ok:
                rospy.logerr(
                    "Every module must have fresh HOVER_STATE=%d: %s",
                    self.beetle.HOVER_STATE, flight_problems)
                return False

        # Finish every read-only identity/pose check before mutating controller
        # mode.  This prevents a stale or partial assembly from being switched
        # merely because its flight-state topics still say HOVER.
        pre_mode_rpy = self._measured_assembly_rpy()
        pre_mode_host_rpy = self._measured_host_rpy()
        if (pre_mode_rpy is None or pre_mode_host_rpy is None or
                not np.all(np.isfinite(pre_mode_rpy)) or
                not np.all(np.isfinite(pre_mode_host_rpy))):
            rospy.logerr(
                "Assembly/EE-host attitude is unavailable before mode switch")
            return False
        pre_mode_max_rp = max(
            abs(pre_mode_rpy[0]), abs(pre_mode_rpy[1]),
            abs(pre_mode_host_rpy[0]), abs(pre_mode_host_rpy[1]))
        if pre_mode_max_rp > self.config.initial_rp_max:
            rospy.logerr(
                "Initial assembly/host roll-pitch %.2fdeg exceeds %.2fdeg "
                "before mode switch",
                math.degrees(pre_mode_max_rp),
                math.degrees(self.config.initial_rp_max))
            return False
        pre_mode_geometry_ok, pre_mode_geometry_reason, _ = (
            self._formation_geometry_status())
        if not pre_mode_geometry_ok:
            rospy.logerr(
                "Formation geometry rejected before mode switch: %s",
                pre_mode_geometry_reason)
            return False

        if not self._auto_enable_unified_mode():
            return False
        self._mode_at_start = _as_bool(self.beetle.isUnifiedMode())
        if not self._mode_at_start:
            rospy.logerr(
                "This high-torque experiment requires unified control because "
                "the LF path has no equivalent formation realized-wrench, "
                "observer and pose-PID safety diagnostics; beetle%d is not "
                "currently unified",
                self.beetle.wrench_target_id)
            return False
        # Unified mode always requires the realized-wrench diagnostic for the
        # independent hard cap. The user flag controls only whether achieved
        # directed torque also gates successful hold-time accumulation.
        self._runtime_realized_required = self._mode_at_start
        self._runtime_realized_ratio_required = (
            self.config.require_realized_wrench and self._mode_at_start)

        connection_deadline = rospy.get_time() + 2.0
        path_ok, path_reason = self._command_path_status()
        while (not path_ok and not rospy.is_shutdown() and
               rospy.get_time() < connection_deadline):
            rospy.sleep(0.04)
            path_ok, path_reason = self._command_path_status()
        if not path_ok:
            rospy.logerr("Command path is incomplete: %s", path_reason)
            return False
        ownership_ok, ownership_reason = (
            self._exclusive_command_publishers_status())
        if not ownership_ok:
            rospy.logerr("Command ownership check failed: %s", ownership_reason)
            return False
        watchdog_ok, watchdog_reason = self._controller_watchdog_status()
        if not watchdog_ok:
            rospy.logerr(
                "Controller wrench watchdog check failed: %s",
                watchdog_reason)
            return False

        heartbeat_deadline = rospy.get_time() + 2.0
        heartbeat_ok, heartbeat_problems = (
            self.feedback.unified_heartbeat_status(
                self.config.feedback_timeout))
        while (not rospy.is_shutdown() and not heartbeat_ok and
               rospy.get_time() < heartbeat_deadline):
            rospy.sleep(0.04)
            heartbeat_ok, heartbeat_problems = (
                self.feedback.unified_heartbeat_status(
                    self.config.feedback_timeout))
        if not heartbeat_ok:
            rospy.logerr(
                "Every module must have a fresh active-unified heartbeat: %s",
                heartbeat_problems)
            return False

        pid_deadline = rospy.get_time() + 2.0
        while (not rospy.is_shutdown() and
               self.beetle.getFreshControlPid(
                   self.config.feedback_timeout) is None and
               rospy.get_time() < pid_deadline):
            rospy.sleep(0.04)
        if self.beetle.getFreshControlPid(
                self.config.feedback_timeout) is None:
            rospy.logerr("Fresh /assemble/debug/pose/pid is required")
            return False

        needs_observer = (
            self.config.contact_confirmation_mode in (
                "stall_and_torque", "torque_only", "either") or
            self.config.max_estimated_torque > 0.0)
        if (needs_observer and
                not self.formation_observer.wait_ready(
                    timeout=3.0, freshness=self.config.feedback_timeout)):
            rospy.logerr(
                "Fresh formation observer wrench is required by the selected "
                "contact/safety settings")
            return False
        if (self._runtime_realized_required and
                not self.realized_wrench.wait_ready(
                    timeout=3.0, freshness=self.config.feedback_timeout)):
            rospy.logerr(
                "Fresh /assemble/debug/formation_wrench is required to "
                "validate achieved torque")
            return False

        rpy = self._measured_assembly_rpy()
        host_rpy = self._measured_host_rpy()
        if (rpy is None or host_rpy is None or
                not np.all(np.isfinite(rpy))):
            rospy.logerr("Assembly/EE-host attitude is unavailable")
            return False
        max_rp = max(
            abs(rpy[0]), abs(rpy[1]),
            abs(host_rpy[0]), abs(host_rpy[1]))
        if max_rp > self.config.initial_rp_max:
            rospy.logerr(
                "Initial assembly/host roll-pitch %.2fdeg exceeds the strict "
                "inserted-tool startup limit %.2fdeg",
                math.degrees(max_rp),
                math.degrees(self.config.initial_rp_max))
            return False

        geometry_ok, geometry_reason, geometry_metrics = (
            self._formation_geometry_status())
        if not geometry_ok:
            rospy.logerr(
                "Inserted-tool geometry check failed: %s", geometry_reason)
            return False
        rospy.loginfo(
            "Geometry verified: tool-model=%.1fmm, module-centroid=%.1fmm, "
            "host/assembly-yaw=%.2fdeg",
            geometry_metrics["tool_model_error"] * 1000.0,
            geometry_metrics["centroid_error"] * 1000.0,
            math.degrees(
                geometry_metrics["host_assembly_yaw_error"]))

        preview_torque = [
            self.config.application_torque_xy[0],
            self.config.application_torque_xy[1],
            self.config.rotation_direction * self.config.target_torque,
        ]
        preview_force_body, preview_torque_body = self.beetle.buildCoGWrench(
            self.config.application_force,
            preview_torque,
            application_offset_body=self.application_offset_body,
            frame_id="world",
            orientation_quaternion=self._measured_assembly_pose()[1])
        try:
            self._validate_wrench_envelope(
                self.config.application_force, preview_torque,
                preview_force_body, preview_torque_body)
        except ValueError as exc:
            rospy.logerr("Full-wrench preview rejected: %s", exc)
            return False
        pins = self.config.tool_geometry.pin_centers
        rospy.loginfo(
            "Dual pins relative to %s: p1=(%.3f, %.3f, %.3f)m, "
            "p2=(%.3f, %.3f, %.3f)m, separation=%.3fm",
            self.config.tool_offset_reference,
            pins[0][0], pins[0][1], pins[0][2],
            pins[1][0], pins[1][1], pins[1][2],
            self.config.tool_geometry.separation)
        rospy.loginfo(
            "Formation-CoG -> tool centre (body): (%.3f, %.3f, %.3f)m",
            *self.application_offset_body)
        rospy.loginfo("Direction: %s", self.config.rotation_label)
        rospy.loginfo(
            "Target application wrench at full torque: "
            "F_world=(%.2f, %.2f, %.2f)N, "
            "tau_world=(%.2f, %.2f, %.2f)Nm",
            *(self.config.application_force + preview_torque))
        rospy.loginfo(
            "Preview full CoG wrench: F_fc=(%.2f, %.2f, %.2f)N, "
            "tau_fc=(%.2f, %.2f, %.2f)Nm",
            *(preview_force_body + preview_torque_body))
        rospy.loginfo(
            "Record commanded wrench topics plus "
            "/assemble/debug/formation_wrench and the formation observer "
            "when evaluating the realized/reaction torque")
        if self.config.contact_reference_lead > 1e-6:
            rospy.logwarn(
                "contact_reference_lead_deg=%.2f leaves yaw-PID preload in "
                "the torque result; use 0 for an interpretable FF-only ramp",
                math.degrees(self.config.contact_reference_lead))
        return True

    def _capture_manual_pose(self):
        self._set_phase("capture_manual_insertion")
        if self.config.startup_delay > 0.0:
            rospy.logwarn(
                "Manual insertion capture begins in %.1fs; keep the two pins "
                "inserted and the formation still",
                self.config.startup_delay)
            deadline = rospy.get_time() + self.config.startup_delay
            while not rospy.is_shutdown() and rospy.get_time() < deadline:
                if self.beetle.getTaskHaltFlag():
                    rospy.logerr("Joystick halt requested before pose capture")
                    return False
                stale = self.feedback.stale_sources(
                    self.config.feedback_timeout)
                if stale:
                    rospy.logerr("Feedback became stale: %s", stale)
                    return False
                rospy.sleep(0.04)

        positions = []
        assembly_positions = []
        assembly_yaws = []
        host_yaws = []
        start = rospy.get_time()
        rate = rospy.Rate(self.config.control_rate)
        while (not rospy.is_shutdown() and
               rospy.get_time() - start < self.config.capture_duration):
            stale = self.feedback.stale_sources(
                self.config.feedback_timeout)
            if stale:
                rospy.logerr("Feedback stale during pose capture: %s", stale)
                return False
            if self.beetle.getTaskHaltFlag():
                rospy.logerr("Joystick halt requested during pose capture")
                return False
            position = self._measured_tool_center()
            assembly_pose = self._measured_assembly_pose()
            assembly_yaw = self._measured_assembly_yaw()
            host_yaw = self._measured_host_yaw()
            if (position is not None and assembly_pose is not None and
                    assembly_yaw is not None and host_yaw is not None):
                positions.append(np.asarray(position, dtype=float))
                assembly_positions.append(assembly_pose[0])
                assembly_yaws.append(float(assembly_yaw))
                host_yaws.append(float(host_yaw))
            rate.sleep()
        if len(positions) < max(3, int(0.5 * self.config.control_rate)):
            rospy.logerr("Insufficient pose samples during manual capture")
            return False

        pivot = np.mean(positions, axis=0)
        position_span = max(
            float(np.linalg.norm(position - pivot)) for position in positions)
        yaw_spans = []
        for samples in (assembly_yaws, host_yaws):
            reference_yaw = samples[0]
            unwrapped = np.array([
                reference_yaw + normalize_angle(yaw - reference_yaw)
                for yaw in samples])
            yaw_spans.append(float(np.max(unwrapped) - np.min(unwrapped)))
        yaw_span = max(yaw_spans)
        assembly_yaw = math.atan2(
            sum(math.sin(value) for value in assembly_yaws),
            sum(math.cos(value) for value in assembly_yaws))
        host_yaw = math.atan2(
            sum(math.sin(value) for value in host_yaws),
            sum(math.cos(value) for value in host_yaws))
        if position_span > self.config.capture_max_position_span:
            rospy.logerr(
                "Formation moved %.1fmm during capture (limit %.1fmm)",
                position_span * 1000.0,
                self.config.capture_max_position_span * 1000.0)
            return False
        if yaw_span > self.config.capture_max_yaw_span:
            rospy.logerr(
                "Formation yaw moved %.2fdeg during capture (limit %.2fdeg)",
                math.degrees(yaw_span),
                math.degrees(self.config.capture_max_yaw_span))
            return False

        geometry_ok, geometry_reason, _ = self._formation_geometry_status()
        if not geometry_ok:
            rospy.logerr(
                "Geometry changed during manual capture: %s", geometry_reason)
            return False

        # An inserted tool must not receive a step when the first nav message
        # is emitted.  Check the exact yaw-only inverse used by the reused
        # formation command helper against the captured assembly CoG.
        captured_assembly_position = np.mean(assembly_positions, axis=0)
        captured_assembly_target = np.asarray(
            self.formation_adapter.transform_end_effector_to_assembly_command(
                pivot, assembly_yaw), dtype=float)
        captured_nav_delta = float(np.linalg.norm(
            captured_assembly_target - captured_assembly_position))
        if captured_nav_delta > self.config.tool_model_tolerance:
            rospy.logerr(
                "Captured inserted-tool inverse mismatch %.1fmm exceeds "
                "%.1fmm", captured_nav_delta * 1000.0,
                self.config.tool_model_tolerance * 1000.0)
            return False

        if (self.beetle.getTaskHaltFlag() or
                self.feedback.stale_sources(self.config.feedback_timeout)):
            rospy.logerr(
                "Halt or stale feedback immediately before first nav command")
            return False
        current_pose = self._measured_assembly_pose()
        current_tool = self._measured_tool_center()
        current_assembly_yaw = self._measured_assembly_yaw()
        current_host_yaw = self._measured_host_yaw()
        if (current_pose is None or current_tool is None or
                current_assembly_yaw is None or current_host_yaw is None):
            rospy.logerr("Fresh pose vanished before first nav command")
            return False
        current_tool_delta = float(np.linalg.norm(
            np.asarray(current_tool, dtype=float) - pivot))
        initial_assembly_target = np.asarray(
            self.formation_adapter.transform_end_effector_to_assembly_command(
                pivot, current_assembly_yaw), dtype=float)
        initial_nav_jump = float(np.linalg.norm(
            initial_assembly_target - current_pose[0]))
        if initial_nav_jump > self.config.tool_model_tolerance:
            rospy.logerr(
                "First inserted-tool nav command would jump %.1fmm (limit "
                "%.1fmm); check module order, 0.52m spacing, CoG reference, "
                "and mocap frames",
                initial_nav_jump * 1000.0,
                self.config.tool_model_tolerance * 1000.0)
            return False
        if current_tool_delta > self.config.tool_model_tolerance:
            rospy.logerr(
                "Tool moved %.1fmm from the captured valve centre before the "
                "first nav command (limit %.1fmm)",
                current_tool_delta * 1000.0,
                self.config.tool_model_tolerance * 1000.0)
            return False

        self._pivot = pivot
        self._hold_yaw = current_assembly_yaw
        current_rpy = self._measured_assembly_rpy()
        current_host_rpy = self._measured_host_rpy()
        if current_rpy is None or current_host_rpy is None:
            rospy.logerr("Attitude vanished before first nav command")
            return False
        current_max_rp = max(
            abs(float(current_rpy[0])), abs(float(current_rpy[1])),
            abs(float(current_host_rpy[0])),
            abs(float(current_host_rpy[1])))
        if current_max_rp > self.config.initial_rp_max:
            rospy.logerr(
                "Roll/pitch changed to %.2fdeg before first nav command "
                "(strict limit %.2fdeg)",
                math.degrees(current_max_rp),
                math.degrees(self.config.initial_rp_max))
            return False
        status, reason, _ = self._guard_status()
        if status != "ok":
            rospy.logerr(
                "Final guard rejected the first nav command: %s", reason)
            return False
        self.send_assembly_command_from_end_effector(
            pivot, current_assembly_yaw)
        rospy.loginfo(
            "Captured valve/tool centre=%s, assembly yaw=%.2fdeg, host "
            "yaw=%.2fdeg, initial nav delta=%.1fmm",
            FormationUtils.format_vec(pivot),
            math.degrees(current_assembly_yaw),
            math.degrees(current_host_yaw), initial_nav_jump * 1000.0)
        rospy.sleep(0.2)
        return True

    def _guard_status(self):
        if self.beetle.getTaskHaltFlag():
            return "abort", "joystick halt requested", None
        stale = self.feedback.stale_sources(self.config.feedback_timeout)
        if stale:
            return "abort", "stale feedback: %s" % stale, None
        if _as_bool(self.beetle.isUnifiedMode()) != self._mode_at_start:
            return "abort", "controller mode changed during test", None
        path_ok, path_reason = self._command_path_status()
        if not path_ok:
            return "abort", "command path lost: %s" % path_reason, None
        identity_ok, identity_reason = self._runtime_identity_status()
        if not identity_ok:
            return "abort", identity_reason, None
        ownership_ok, ownership_reason = (
            self._exclusive_command_publishers_status())
        if not ownership_ok:
            return "abort", ownership_reason, None
        watchdog_ok, watchdog_reason = self._controller_watchdog_status()
        if not watchdog_ok:
            return "abort", watchdog_reason, None
        heartbeat_ok, heartbeat_problems = (
            self.feedback.unified_heartbeat_status(
                self.config.feedback_timeout))
        if not heartbeat_ok:
            return (
                "abort",
                "active-unified heartbeat guard: %s" % heartbeat_problems,
                None)
        geometry_ok, geometry_reason, geometry_metrics = (
            self._formation_geometry_status())
        if not geometry_ok:
            return "abort", geometry_reason, geometry_metrics
        if self.config.require_hover_state:
            flight_ok, flight_problems = self.feedback.flight_state_status(
                self.config.feedback_timeout, self.beetle.HOVER_STATE)
            if not flight_ok:
                return (
                    "abort",
                    "module flight-state guard: %s" % flight_problems,
                    None)

        position = self._measured_tool_center()
        host_yaw = self._measured_host_yaw()
        assembly_yaw = self._measured_assembly_yaw()
        rpy = self._measured_assembly_rpy()
        host_rpy = self._measured_host_rpy()
        if (position is None or host_yaw is None or assembly_yaw is None or
                rpy is None or host_rpy is None):
            return "abort", "formation pose is unavailable", None
        position = np.asarray(position, dtype=float)
        if (not np.all(np.isfinite(position)) or
                not np.all(np.isfinite(rpy)) or
                not np.all(np.isfinite(host_rpy))):
            return "abort", "formation pose contains non-finite data", None

        delta = position - self._pivot
        xy_error = float(np.linalg.norm(delta[:2]))
        z_error = abs(float(delta[2]))
        max_rp = max(
            abs(float(rpy[0])), abs(float(rpy[1])),
            abs(float(host_rpy[0])), abs(float(host_rpy[1])))
        metrics = {
            "position": position,
            "host_yaw": float(host_yaw),
            "assembly_yaw": float(assembly_yaw),
            "xy_error": xy_error,
            "z_error": z_error,
            "max_rp": max_rp,
            "tool_model_error": geometry_metrics["tool_model_error"],
            "centroid_error": geometry_metrics["centroid_error"],
        }
        # Cache a direct zero-velocity hold target. If feedback later becomes
        # stale during POS_VEL contact search, emergency handling must replace
        # the last nonzero omega command rather than leave it active.
        assembly_pose = self._measured_assembly_pose()
        if assembly_pose is not None:
            self._last_safe_assembly_position = assembly_pose[0].copy()
            self._last_safe_assembly_yaw = float(assembly_yaw)
        if self._runtime_realized_required:
            realized = self.realized_wrench.get_fresh(
                self.config.feedback_timeout)
            if realized is None:
                return "abort", "realized formation wrench is stale", metrics
            realized_torque_norm = float(np.linalg.norm(realized[3:]))
            metrics["realized_wrench"] = realized
            metrics["realized_torque_norm"] = realized_torque_norm
            if realized_torque_norm > self.config.max_realized_torque:
                return (
                    "abort",
                    "realized torque norm %.2fNm exceeds %.2fNm" % (
                        realized_torque_norm,
                        self.config.max_realized_torque),
                    metrics)
        if (xy_error > self.config.guard_xy_relief or
                z_error > self.config.guard_z_relief or
                max_rp > self.config.guard_rp_relief):
            return (
                "abort",
                "hard guard: xy=%.1fmm z=%.1fmm rp=%.1fdeg" % (
                    xy_error * 1000.0, z_error * 1000.0,
                    math.degrees(max_rp)),
                metrics)
        if self.config.max_estimated_torque > 0.0:
            observer = self.formation_observer.get_fresh(
                self.config.feedback_timeout)
            if observer is None:
                return "abort", "formation observer wrench is stale", metrics
            if abs(float(observer[5])) > self.config.max_estimated_torque:
                return (
                    "abort",
                    "estimated |Tz| %.2fNm exceeds %.2fNm" % (
                        abs(float(observer[5])),
                        self.config.max_estimated_torque),
                    metrics)
        if (xy_error > self.config.guard_xy_hold or
                z_error > self.config.guard_z_hold or
                max_rp > self.config.guard_rp_hold):
            return (
                "hold",
                "soft guard: xy=%.1fmm z=%.1fmm rp=%.1fdeg" % (
                    xy_error * 1000.0, z_error * 1000.0,
                    math.degrees(max_rp)),
                metrics)
        return "ok", "", metrics

    def _ramp_preload(self):
        self._set_phase("preload_6d_wrench")
        force_target = np.asarray(
            self.config.application_force, dtype=float)
        torque_target = np.array([
            self.config.application_torque_xy[0],
            self.config.application_torque_xy[1],
            0.0,
        ], dtype=float)
        force_time = (
            1.5 * np.linalg.norm(force_target) /
            self.config.force_ramp_rate)
        torque_time = (
            1.5 * np.linalg.norm(torque_target) /
            self.config.torque_ramp_rate)
        _, cog_torque_target = self.beetle.buildCoGWrench(
            force_target.tolist(), torque_target.tolist(),
            application_offset_body=self.application_offset_body,
            frame_id="world",
            orientation_quaternion=self._measured_assembly_pose()[1])
        cog_torque_time = (
            1.5 * np.linalg.norm(cog_torque_target) /
            self.config.preload_cog_torque_rate)
        ramp_time = max(
            self.config.preload_ramp_time, force_time, torque_time,
            cog_torque_time, 1e-3)
        rospy.loginfo(
            "Preload ramp %.2fs (application F %.2fs, application tau %.2fs, "
            "shifted CoG tau %.2fs)",
            ramp_time, force_time, torque_time, cog_torque_time)
        current_force = np.zeros(3)
        current_torque = np.zeros(3)
        start = rospy.get_time()
        previous = start
        rate = rospy.Rate(self.config.control_rate)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            elapsed = now - start
            dt = max(0.0, min(now - previous, 0.2))
            previous = now
            status, reason, _ = self._guard_status()
            if status != "ok":
                rospy.logerr("Preload stopped by %s", reason)
                return False
            blend = smoothstep01(elapsed / ramp_time)
            current_force = _vector_slew(
                current_force, force_target * blend,
                self.config.force_ramp_rate, dt)
            current_torque = _vector_slew(
                current_torque, torque_target * blend,
                self.config.torque_ramp_rate, dt)
            self.send_assembly_command_from_end_effector(
                self._pivot, self._hold_yaw)
            self._publish_application_wrench(
                current_force.tolist(), current_torque.tolist())
            if (elapsed >= ramp_time and
                    np.allclose(current_force, force_target, atol=1e-4) and
                    np.allclose(current_torque, torque_target, atol=1e-4)):
                rospy.loginfo(
                    "6D preload reached: F=(%.2f, %.2f, %.2f)N, "
                    "tau=(%.2f, %.2f, %.2f)Nm",
                    *(current_force.tolist() + current_torque.tolist()))
                return True
            rate.sleep()
        return False

    def _capture_contact_observer_baseline(self):
        self._set_phase("contact_observer_baseline")
        samples = []
        sample_stamps = []
        start = rospy.get_time()
        rate = rospy.Rate(self.config.control_rate)
        preload_torque = [
            self.config.application_torque_xy[0],
            self.config.application_torque_xy[1],
            0.0,
        ]
        while (not rospy.is_shutdown() and
               rospy.get_time() - start <
               self.config.contact_observer_baseline_duration):
            status, reason, _ = self._guard_status()
            if status != "ok":
                rospy.logerr("Observer baseline stopped by %s", reason)
                return None
            observer_sample = self.formation_observer.get_fresh_sample(
                self.config.feedback_timeout)
            if observer_sample is None:
                rospy.logerr("Formation observer became stale during baseline")
                return None
            observer, source_stamp = observer_sample
            if not sample_stamps or source_stamp > sample_stamps[-1]:
                samples.append(float(observer[5]))
                sample_stamps.append(source_stamp)
            self.send_assembly_command_from_end_effector(
                self._pivot, self._hold_yaw)
            self._publish_application_wrench(
                self.config.application_force, preload_torque)
            rate.sleep()
        minimum_samples = max(
            3, int(0.5 * self.config.control_rate *
                   self.config.contact_observer_baseline_duration))
        if len(samples) < minimum_samples:
            rospy.logerr(
                "Insufficient unique formation-observer baseline samples: "
                "%d < %d", len(samples), minimum_samples)
            return None
        source_coverage = sample_stamps[-1] - sample_stamps[0]
        if source_coverage < 0.5 * self.config.contact_observer_baseline_duration:
            rospy.logerr(
                "Formation-observer baseline source time covers only %.3fs",
                source_coverage)
            return None
        span = float(np.max(samples) - np.min(samples))
        if span > self.config.contact_observer_baseline_span:
            rospy.logerr(
                "Formation-observer Tz baseline span %.3fNm exceeds %.3fNm",
                span, self.config.contact_observer_baseline_span)
            return None
        baseline = float(np.median(samples))
        rospy.loginfo(
            "Formation-observer Tz baseline %.3fNm (span %.3fNm, n=%d); "
            "contact uses directed delta from this bias",
            baseline, span, len(samples))
        return baseline

    def _search_contact(self):
        # Discard any RB press latched before the bounded search. Holding RB
        # does not create a new edge; the operator must release and press it
        # after motion starts.
        self.beetle.resetForceSkipFlag()
        observer_baseline = self._capture_contact_observer_baseline()
        if observer_baseline is None:
            return None
        self.beetle.resetForceSkipFlag()
        self._set_phase("bounded_angular_contact_search")
        measured_start_assembly_yaw = self._measured_assembly_yaw()
        measured_start_host_yaw = self._measured_host_yaw()
        if (measured_start_assembly_yaw is None or
                measured_start_host_yaw is None):
            rospy.logerr(
                "Assembly/host yaw is unavailable at contact-search entry")
            return None
        start_yaw = float(measured_start_assembly_yaw)
        command_yaw = start_yaw
        detector = DirectedAngularStallDetector(
            direction=self.config.rotation_direction,
            min_search_angle=self.config.contact_min_search_angle,
            lead_threshold=self.config.contact_stall_lead,
            stall_rate=self.config.contact_stall_rate,
            velocity_window=self.config.contact_velocity_window,
            required_cycles=self.config.contact_required_cycles)
        start_time = rospy.get_time()
        # Command progress and actual progress have independent zero points:
        # navigation always uses assembly yaw, while physical stall/slip uses
        # the end-effector host mocap yaw. A fixed mounting offset cancels.
        detector.reset(
            command_yaw, float(measured_start_host_yaw), start_time)
        estimated_cycles = 0
        rate = rospy.Rate(self.config.control_rate)
        preload_torque = [
            self.config.application_torque_xy[0],
            self.config.application_torque_xy[1],
            0.0,
        ]

        while not rospy.is_shutdown():
            now = rospy.get_time()
            elapsed = now - start_time
            progress = min(
                self.config.contact_search_max_angle,
                elapsed * self.config.contact_search_speed)
            command_yaw = normalize_angle(
                start_yaw + self.config.rotation_direction * progress)
            # Keep abnormal-exit unloading at the latest bounded search pose;
            # never snap the yaw reference back to the pre-search angle.
            self._hold_yaw = command_yaw
            status, reason, metrics = self._guard_status()
            if status != "ok":
                rospy.logerr("Contact search stopped by %s", reason)
                return None

            self.send_assembly_command_from_end_effector(
                self._pivot,
                command_yaw,
                linear_vel=[0.0, 0.0, 0.0],
                angular_vel=(self.config.rotation_direction *
                             self.config.contact_search_speed))
            self._publish_application_wrench(
                self.config.application_force, preload_torque)
            observation = detector.update(
                command_yaw, metrics["host_yaw"], now)

            wrench_triggered = False
            reaction_torque = None
            torque_mode = self.config.contact_confirmation_mode in (
                "stall_and_torque", "torque_only", "either")
            if torque_mode:
                observer = self.formation_observer.get_fresh(
                    self.config.feedback_timeout)
                if observer is None:
                    rospy.logerr(
                        "Formation observer became stale during torque-based "
                        "contact confirmation")
                    return None
                # The observer estimates environment-on-robot reaction, which
                # opposes the commanded visual/world-Z search direction.
                reaction_torque = (
                    -self.config.rotation_direction *
                    (float(observer[5]) - observer_baseline))
                if (observation["command_progress"] >=
                        self.config.contact_min_search_angle and
                        reaction_torque >=
                        self.config.contact_estimated_torque_threshold):
                    estimated_cycles += 1
                else:
                    estimated_cycles = 0
                wrench_triggered = (
                    estimated_cycles >=
                    self.config.contact_estimated_torque_cycles)

            mode = self.config.contact_confirmation_mode
            if mode == "stall_and_torque":
                automatic_contact = (
                    observation["contact"] and wrench_triggered)
                source = "angular stall AND directed formation reaction torque"
            elif mode == "stall_only":
                automatic_contact = observation["contact"]
                source = "angular stall (explicitly selected)"
            elif mode == "torque_only":
                automatic_contact = wrench_triggered
                source = "directed formation reaction torque"
            else:
                automatic_contact = (
                    observation["contact"] or wrench_triggered)
                source = "angular stall OR directed formation reaction torque"

            manual_contact = False
            if self.beetle.getForceSkipFlag():
                self.beetle.resetForceSkipFlag()
                if (observation["command_progress"] >=
                        self.config.contact_min_search_angle):
                    manual_contact = True
                    source = "new operator RB confirmation"
                else:
                    rospy.logwarn(
                        "Ignoring RB contact confirmation before the %.1fdeg "
                        "minimum search angle; release and press again later",
                        math.degrees(self.config.contact_min_search_angle))

            if automatic_contact or manual_contact:
                actual_contact_assembly_yaw = metrics["assembly_yaw"]
                actual_contact_host_yaw = metrics["host_yaw"]
                reference_yaw = normalize_angle(
                    actual_contact_assembly_yaw +
                    self.config.rotation_direction *
                    self.config.contact_reference_lead)
                rospy.loginfo(
                    "Valve contact detected by %s: search=%.2fdeg, "
                    "lead=%.2fdeg, actual_rate=%.2fdeg/s; resetting yaw "
                    "reference from %.2fdeg to assembly %.2fdeg before FF "
                    "ramp (host actual %.2fdeg)",
                    source,
                    math.degrees(observation["command_progress"]),
                    math.degrees(observation["directed_lead"]),
                    (math.degrees(observation["actual_rate"])
                     if math.isfinite(observation["actual_rate"])
                    else float("inf")),
                    math.degrees(command_yaw),
                    math.degrees(reference_yaw),
                    math.degrees(actual_contact_host_yaw))
                return reference_yaw

            rospy.loginfo_throttle(
                1.0,
                "[ContactSearch] cmd=%.1f/%.1fdeg actual=%.1fdeg "
                "lead=%.1fdeg rate=%s candidate=%d/%d reaction_delta=%s",
                math.degrees(observation["command_progress"]),
                math.degrees(self.config.contact_search_max_angle),
                math.degrees(observation["actual_progress"]),
                math.degrees(observation["directed_lead"]),
                ("%.2fdeg/s" % math.degrees(observation["actual_rate"])
                 if math.isfinite(observation["actual_rate"])
                 else "warming"),
                observation["candidate_cycles"],
                self.config.contact_required_cycles,
                ("%.3fNm" % reaction_torque
                 if reaction_torque is not None else "disabled"))
            if (elapsed >= self.config.contact_search_timeout or
                    progress >= self.config.contact_search_max_angle):
                rospy.logerr(
                    "No contact within %.1fdeg / %.1fs bounded search",
                    math.degrees(progress), elapsed)
                return None
            rate.sleep()
        return None

    def _settle_contact_reference(self, reference_yaw):
        self._set_phase("zero_yaw_pid_preload")
        start = rospy.get_time()
        rate = rospy.Rate(self.config.control_rate)
        preload_torque = [
            self.config.application_torque_xy[0],
            self.config.application_torque_xy[1],
            0.0,
        ]
        while (not rospy.is_shutdown() and
               rospy.get_time() - start < self.config.contact_settle_time):
            status, reason, _ = self._guard_status()
            if status != "ok":
                rospy.logerr("Contact settling stopped by %s", reason)
                return None
            self.send_assembly_command_from_end_effector(
                self._pivot, reference_yaw)
            self._publish_application_wrench(
                self.config.application_force, preload_torque)
            rate.sleep()
        settled_host_yaw = self._measured_host_yaw()
        settled_assembly_yaw = self._measured_assembly_yaw()
        if settled_host_yaw is None or settled_assembly_yaw is None:
            return None
        pid_ok, pid_reason, pid_metrics = self._yaw_pid_status()
        if not pid_ok:
            rospy.logerr(
                "Yaw PID preload was not removed after contact settling: %s",
                pid_reason)
            return None
        rospy.loginfo(
            "Contact reference settled: command/assembly=%.2f/%.2fdeg, "
            "host actual=%.2fdeg, yaw PID err/command=%.2fdeg/%.3f",
            math.degrees(reference_yaw),
            math.degrees(settled_assembly_yaw),
            math.degrees(settled_host_yaw),
            math.degrees(pid_metrics["yaw_error"]),
            pid_metrics["yaw_pid_command"])
        return float(settled_host_yaw)

    def _run_torque_profile(self, settled_yaw):
        self._set_phase("torque_ramp_and_hold")
        target_magnitude = self.config.target_torque
        current_magnitude = 0.0
        virtual_ramp_elapsed = 0.0
        full_hold_elapsed = 0.0
        target_logged = False
        guard_started = None
        realized_shortfall_started = None
        start = rospy.get_time()
        previous = start
        maximum_duration = (
            max(self.config.torque_ramp_time,
                1.5 * target_magnitude / self.config.torque_ramp_rate) +
            self.config.full_torque_hold_time +
            self.config.guard_hold_timeout + 10.0)
        rate = rospy.Rate(self.config.control_rate)

        while not rospy.is_shutdown():
            now = rospy.get_time()
            dt = max(0.0, min(now - previous, 0.2))
            previous = now
            status, reason, metrics = self._guard_status()
            if status == "abort":
                rospy.logerr("Torque profile stopped by %s", reason)
                return False

            pid_ok, pid_reason, pid_metrics = self._yaw_pid_status()
            if not pid_ok:
                rospy.logerr(
                    "Torque profile would include excessive yaw PID: %s",
                    pid_reason)
                return False

            realized_directed_torque = None
            if self._runtime_realized_ratio_required:
                realized = metrics.get("realized_wrench")
                if realized is None:
                    rospy.logerr("Realized formation wrench became unavailable")
                    return False
                realized_directed_torque = (
                    self.config.rotation_direction * float(realized[5]))

            slip = abs(normalize_angle(
                metrics["host_yaw"] - settled_yaw))
            if (self.config.max_static_slip_angle > 0.0 and
                    slip > self.config.max_static_slip_angle):
                rospy.logerr(
                    "Static valve/tool slip %.2fdeg exceeds %.2fdeg; "
                    "unloading",
                    math.degrees(slip),
                    math.degrees(self.config.max_static_slip_angle))
                return False

            if status == "hold":
                if guard_started is None:
                    guard_started = now
                    rospy.logwarn(
                        "Holding torque increase at %.3fNm: %s",
                        current_magnitude, reason)
                elif now - guard_started > self.config.guard_hold_timeout:
                    rospy.logerr(
                        "Soft guard persisted for %.1fs; unloading",
                        now - guard_started)
                    return False
            else:
                if guard_started is not None:
                    rospy.loginfo("Torque guard cleared; resuming ramp")
                    guard_started = None
                virtual_ramp_elapsed += dt

            nominal = target_magnitude * smoothstep01(
                virtual_ramp_elapsed /
                max(self.config.torque_ramp_time, 1e-6))
            if status == "ok":
                current_magnitude = slew_toward(
                    current_magnitude,
                    nominal,
                    self.config.torque_ramp_rate,
                    dt)

            torque_world = [
                self.config.application_torque_xy[0],
                self.config.application_torque_xy[1],
                self.config.rotation_direction * current_magnitude,
            ]
            # Re-lock the yaw target to fresh assembly feedback each cycle so
            # the test measures feedforward torque, not a growing pose-PID
            # preload. Host yaw remains the independent valve-slip signal.
            neutral_reference_yaw = metrics["assembly_yaw"]
            self._hold_yaw = neutral_reference_yaw
            self.send_assembly_command_from_end_effector(
                self._pivot, neutral_reference_yaw)
            self._publish_application_wrench(
                self.config.application_force, torque_world)

            if (current_magnitude >= target_magnitude - 1e-4 and
                    virtual_ramp_elapsed >= self.config.torque_ramp_time):
                if not target_logged:
                    target_logged = True
                    rospy.loginfo(
                        "Target torque reached: %.3fNm; holding for %.1fs",
                        self.config.rotation_direction * current_magnitude,
                        self.config.full_torque_hold_time)
                realized_ok = (
                    realized_directed_torque is None or
                    realized_directed_torque >=
                    self.config.min_realized_torque_ratio * target_magnitude)
                if not realized_ok:
                    if realized_shortfall_started is None:
                        realized_shortfall_started = now
                        rospy.logwarn(
                            "Command reached target but realized directed Tz "
                            "is %.3fNm; need at least %.3fNm",
                            realized_directed_torque,
                            (self.config.min_realized_torque_ratio *
                             target_magnitude))
                    elif (now - realized_shortfall_started >
                          self.config.guard_hold_timeout):
                        rospy.logerr(
                            "Realized torque stayed below target ratio for "
                            "%.1fs; allocation/actuation limit reached",
                            now - realized_shortfall_started)
                        return False
                else:
                    realized_shortfall_started = None
                if status == "ok" and realized_ok:
                    full_hold_elapsed += dt
                if full_hold_elapsed >= self.config.full_torque_hold_time:
                    rospy.loginfo(
                        "Full-torque hold complete: %.3fNm for %.1fs",
                        self.config.rotation_direction * current_magnitude,
                        full_hold_elapsed)
                    return True
            else:
                full_hold_elapsed = 0.0
                target_logged = False

            rospy.loginfo_throttle(
                1.0,
                "[TorqueRamp] Tz=%.3f/%.3fNm virtual_t=%.1fs "
                "realized=%s slip=%.2fdeg xy=%.1fmm z=%.1fmm "
                "rp=%.1fdeg yaw_pid=%.3f guard=%s",
                self.config.rotation_direction * current_magnitude,
                self.config.rotation_direction * target_magnitude,
                virtual_ramp_elapsed,
                ("%.3fNm" % realized_directed_torque
                 if realized_directed_torque is not None else "disabled"),
                math.degrees(slip),
                metrics["xy_error"] * 1000.0,
                metrics["z_error"] * 1000.0,
                math.degrees(metrics["max_rp"]),
                pid_metrics["yaw_pid_command"], status)
            if now - start > maximum_duration:
                rospy.logerr("Torque profile exceeded %.1fs timeout",
                             maximum_duration)
                return False
            rate.sleep()
        return False

    def _hold_safe_assembly_pose(self):
        """Freeze a fresh or last-validated CoG pose without a pivot inverse."""
        stale = self.feedback.stale_sources(self.config.feedback_timeout)
        pose = None if stale else self._measured_assembly_pose()
        if pose is not None:
            position, quaternion = pose
            yaw = float(euler_from_quaternion(quaternion)[2])
            self._last_safe_assembly_position = position.copy()
            self._last_safe_assembly_yaw = yaw
            source = "fresh"
        elif (self._last_safe_assembly_position is not None and
              self._last_safe_assembly_yaw is not None):
            position = self._last_safe_assembly_position.copy()
            yaw = float(self._last_safe_assembly_yaw)
            source = "last validated (feedback now stale)"
        else:
            rospy.logwarn(
                "No validated assembly pose is available for emergency hold")
            return False
        # targetMotion without velocity selects POS_MODE and writes zero target
        # omega, explicitly cancelling the contact-search POS_VEL command.
        self.beetle.targetMotion(position.tolist(), rot=yaw)
        rospy.logwarn(
            "Emergency nav hold (%s) at CoG=%s yaw=%.2fdeg with zero velocity",
            source, FormationUtils.format_vec(position), math.degrees(yaw))
        return True

    def _emergency_zero(self, reason):
        """Hold the last validated pose, then zero every wrench route."""
        self._set_phase("emergency_unload")
        rospy.logerr("Emergency all-route zero: %s", reason)
        self._hold_safe_assembly_pose()
        self._publish_zero_all_routes()
        rospy.logwarn(
            "All six wrench components were zeroed on every route without "
            "publishing another nonzero sample")
        return False

    def _unload(self, emergency=False):
        if emergency:
            # Do not ramp through BeetleInterface's dynamically selected route:
            # a controller mode change could otherwise refresh nearly the full
            # wrench on the newly active alias. Freeze only a fresh measured
            # pose, then immediately zero both aliases, weights and LF shares.
            return self._emergency_zero("abnormal experiment exit")

        self._set_phase("normal_unload")

        if not self.beetle.external_wrench_active:
            self._publish_zero_all_routes()
            return True
        status, reason, metrics = self._guard_status()
        if status != "ok":
            return self._emergency_zero(
                "normal-unload entry guard: %s" % reason)
        pid_ok, pid_reason, _ = self._yaw_pid_status()
        if not pid_ok:
            return self._emergency_zero(
                "normal-unload yaw PID guard: %s" % pid_reason)
        self._hold_yaw = metrics["assembly_yaw"]

        start_force = self._last_application_force.copy()
        start_torque = self._last_application_torque.copy()
        start_cog_force = self._last_cog_force.copy()
        start_cog_torque = self._last_cog_torque.copy()
        # The CoG torque includes r x F (2.5 Nm Ty with the default geometry
        # and Fz).  Sizing only from application-point torque would violate the
        # configured torque slew during normal unloading.  1.5 is the maximum
        # derivative of smoothstep01.
        duration = max(
            self.config.unload_min_duration,
            1.5 * minimum_unload_duration(
                start_cog_force, start_cog_torque, 0.0,
                self.config.unload_force_rate,
                self.config.unload_torque_rate))
        rospy.logwarn(
            "Unloading 6D wrench over %.2fs from F=(%.2f, %.2f, %.2f)N, "
            "tau=(%.2f, %.2f, %.2f)Nm",
            duration, *(start_force.tolist() + start_torque.tolist()))
        start = rospy.get_time()
        rate = rospy.Rate(self.config.control_rate)
        while not rospy.is_shutdown():
            status, reason, metrics = self._guard_status()
            if status != "ok":
                return self._emergency_zero(
                    "normal-unload runtime guard: %s" % reason)
            pid_ok, pid_reason, _ = self._yaw_pid_status()
            if not pid_ok:
                return self._emergency_zero(
                    "normal-unload yaw PID guard: %s" % pid_reason)
            self._hold_yaw = metrics["assembly_yaw"]
            elapsed = rospy.get_time() - start
            blend = smoothstep01(elapsed / max(duration, 1e-6))
            scale = 1.0 - blend
            force = scale * start_force
            torque = scale * start_torque
            cog_force = scale * start_cog_force
            cog_torque = scale * start_cog_torque
            if self._pivot is not None and self._hold_yaw is not None:
                self.send_assembly_command_from_end_effector(
                    self._pivot, self._hold_yaw)
            # Scale the cached body-frame CoG wrench directly. This preserves
            # the exact six-axis ratio and slew bound even if attitude moves a
            # little during release, and avoids re-routing from live mode.
            self._publish_wrench_sample(
                force.tolist(), torque.tolist(),
                cog_force.tolist(), cog_torque.tolist())
            if elapsed >= duration:
                break
            rate.sleep()
        self._publish_zero_all_routes()
        rospy.loginfo("All six feedforward wrench components are zero")
        return True

    def execute(self, userdata=None):
        try:
            if not self._preflight():
                return "failed"
            if not self._capture_manual_pose():
                return "failed"
            if not self._ramp_preload():
                return "failed"
            contact_reference = self._search_contact()
            if contact_reference is None:
                return "failed"
            self._hold_yaw = contact_reference
            settled_yaw = self._settle_contact_reference(contact_reference)
            if settled_yaw is None:
                return "failed"
            if not self._run_torque_profile(settled_yaw):
                return "failed"
            if not self._unload(emergency=False):
                return "failed"
            self._normal_completion = True
            self._set_phase("complete")
            return "succeeded"
        except rospy.ROSInterruptException:
            rospy.logerr("Torque test interrupted by ROS shutdown")
            return "failed"
        except Exception as exc:
            rospy.logerr("Unhandled torque-test exception: %s", exc)
            import traceback
            traceback.print_exc()
            return "failed"
        finally:
            if self.beetle.external_wrench_active:
                try:
                    self._unload(emergency=not self._normal_completion)
                except Exception as exc:
                    rospy.logerr("6D unload failed: %s", exc)
                    try:
                        self._publish_zero_all_routes()
                    except Exception as zero_exc:
                        rospy.logerr("Final all-route zero failed: %s", zero_exc)
            self._restore_auto_enabled_unified_mode()


def main():
    rospy.init_node("static_downward_valve_torque")
    try:
        config = TorqueTestConfig()
        rospy.loginfo("=" * 70)
        rospy.loginfo("Static Downward-Valve Formation Torque Test")
        rospy.loginfo(
            "mode=%s module_ids=%s EE_host=beetle%d target=%.3fNm "
            "direction=%s",
            "real" if config.real_machine else "simulation",
            config.module_ids, config.end_effector_module_id,
            config.target_torque,
            config.rotation_label)
        rospy.loginfo("=" * 70)
        experiment = StaticDownwardValveTorqueTest(config)
        outcome = experiment.execute()
    except Exception as exc:
        rospy.logerr("Failed to construct torque test: %s", exc)
        import traceback
        traceback.print_exc()
        outcome = "failed"
    rospy.loginfo("Static downward-valve torque test outcome: %s", outcome)
    if outcome != "succeeded":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
