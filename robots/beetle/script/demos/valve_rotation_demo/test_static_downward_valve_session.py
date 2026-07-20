#!/usr/bin/env python3
"""Unit tests for the static downward-valve contact and wrench profile."""

import math
import os
import sys
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import static_downward_valve_torque as static_torque
from beetle_interface import BeetleInterface
from static_downward_valve_torque import (
    DirectedAngularStallDetector,
    circular_start_angle,
    minimum_smoothstep_duration,
    parse_rotation_direction,
    smooth_start_angular_motion,
)
from valve_rotation_formation_clean import (
    FormationAdapter,
    FormationRotateValveState,
)


class StaticDownwardValveRotationTest(unittest.TestCase):

    def test_downward_task_has_its_own_host_to_tool_offset(self):
        with mock.patch.object(
                static_torque.rospy, "get_param",
                side_effect=lambda name, default: default):
            config = static_torque.read_config()
        self.assertEqual(
            config.end_effector_offset_body.tolist(),
            [0.240, 0.0, 0.11053])
        self.assertEqual(config.valve_feedback_timeout, 10.0)
        self.assertEqual(config.pose_feedback_max_age, 0.5)
        self.assertEqual(config.search_ramp_time, 2.0)
        self.assertEqual(config.zero_radius_tolerance, 0.030)

    def test_cfs_is_required_only_for_real_hardware(self):
        hardware_params = {
            "~real_machine": True,
            "~simulation": False,
            "~force_sensor_enabled": True,
        }
        with mock.patch.object(
                static_torque.rospy, "get_param",
                side_effect=lambda name, default: hardware_params.get(
                    name, default)):
            hardware_config = static_torque.read_config()
        self.assertTrue(hardware_config.force_sensor_enabled)

        with mock.patch.object(
                static_torque.rospy, "get_param",
                side_effect=lambda name, default: default):
            simulation_config = static_torque.read_config()
        self.assertFalse(simulation_config.force_sensor_enabled)

    def test_cfs_calibration_completes_before_task_can_start(self):
        state = static_torque.StaticDownwardValveTorqueTest.__new__(
            static_torque.StaticDownwardValveTorqueTest)
        state.config = static_torque.SimpleNamespace(
            force_sensor_enabled=True,
            force_sensor_auto_calib=True,
            force_sensor_calib_service="cfs_sensor_calib",
            force_sensor_topic="cfs/data",
            force_sensor_auto_calib_timeout=10.0,
            force_sensor_auto_calib_delay=0.5,
            force_sensor_data_max_age=0.5)
        state.measured_wrench_topic = "cfs/data"
        state.measured_wrench = None
        state.measured_wrench_received_time = None
        state._set_phase = mock.Mock()
        sample = static_torque.WrenchStamped()
        sample.wrench.force.z = 0.02
        sample.wrench.torque.z = -0.01

        with mock.patch.object(
                static_torque, "calibrate_force_sensor",
                return_value=True) as calibrate, mock.patch.object(
                    static_torque.rospy, "wait_for_message",
                    return_value=sample), mock.patch.object(
                        static_torque.rospy, "get_time", return_value=1.0):
            self.assertTrue(state._prepare_force_sensor())

        calibrate.assert_called_once()
        state._set_phase.assert_called_once_with("force_sensor_calibration")
        self.assertAlmostEqual(state.measured_wrench[2], 0.02)
        self.assertAlmostEqual(state.measured_wrench[5], -0.01)

    def test_cfs_calibration_failure_blocks_task(self):
        state = static_torque.StaticDownwardValveTorqueTest.__new__(
            static_torque.StaticDownwardValveTorqueTest)
        state.config = static_torque.SimpleNamespace(
            force_sensor_enabled=True,
            force_sensor_auto_calib=True,
            force_sensor_calib_service="cfs_sensor_calib",
            force_sensor_topic="cfs/data",
            force_sensor_auto_calib_timeout=10.0,
            force_sensor_auto_calib_delay=0.5)
        state._set_phase = mock.Mock()
        with mock.patch.object(
                static_torque, "calibrate_force_sensor",
                return_value=False), mock.patch.object(
                    static_torque.rospy, "wait_for_message") as wait:
            self.assertFalse(state._prepare_force_sensor())
        wait.assert_not_called()

    def test_downward_view_reverses_visual_direction(self):
        self.assertEqual(parse_rotation_direction("cw", "below")[0], 1)
        self.assertEqual(parse_rotation_direction("ccw", "below")[0], -1)
        self.assertEqual(parse_rotation_direction("cw", "above")[0], -1)
        self.assertEqual(parse_rotation_direction("ccw", "above")[0], 1)

    def test_feedback_prefers_valid_valve_mocap(self):
        state = static_torque.StaticDownwardValveTorqueTest.__new__(
            static_torque.StaticDownwardValveTorqueTest)
        state.config = static_torque.SimpleNamespace(
            valve_feedback_timeout=10.0,
            pose_feedback_max_age=0.5)
        state._set_phase = mock.Mock()
        state.beetle = mock.Mock()
        state.beetle.getFreshValvePos.return_value = [3.0, 2.0, 1.5]
        state.formation_adapter = mock.Mock()
        state.formation_adapter.has_fresh_end_effector_feedback.return_value = True
        state.get_end_effector_position = mock.Mock(
            return_value=[0.2, -0.1, 1.0])
        state.get_end_effector_yaw = mock.Mock(return_value=0.3)

        with mock.patch.object(
                static_torque.rospy, "get_time", return_value=5.0), \
                mock.patch.object(static_torque.rospy, "Rate"), \
                mock.patch.object(
                    static_torque.rospy, "is_shutdown", return_value=False):
            valve_center, ee_position, ee_yaw = state._wait_for_feedback()

        np.testing.assert_allclose(valve_center, [3.0, 2.0, 1.5])
        np.testing.assert_allclose(ee_position, [0.2, -0.1, 1.0])
        self.assertAlmostEqual(ee_yaw, 0.3)

    def test_feedback_uses_current_end_effector_center_after_timeout(self):
        state = static_torque.StaticDownwardValveTorqueTest.__new__(
            static_torque.StaticDownwardValveTorqueTest)
        state.config = static_torque.SimpleNamespace(
            valve_feedback_timeout=10.0,
            pose_feedback_max_age=0.5)
        state._set_phase = mock.Mock()
        state.beetle = mock.Mock()
        state.beetle.getFreshValvePos.return_value = None
        state.formation_adapter = mock.Mock()
        state.formation_adapter.has_fresh_end_effector_feedback.return_value = True
        state.get_end_effector_position = mock.Mock(
            return_value=[0.2, -0.1, 1.0])
        state.get_end_effector_yaw = mock.Mock(return_value=0.3)

        with mock.patch.object(
                static_torque.rospy, "get_time",
                side_effect=[5.0, 15.0]), mock.patch.object(
                    static_torque.rospy, "logwarn") as logwarn, \
                mock.patch.object(static_torque.rospy, "Rate"), \
                mock.patch.object(
                    static_torque.rospy, "is_shutdown", return_value=False):
            valve_center, ee_position, ee_yaw = state._wait_for_feedback()

        np.testing.assert_allclose(valve_center, ee_position)
        np.testing.assert_allclose(valve_center, [0.2, -0.1, 1.0])
        self.assertAlmostEqual(ee_yaw, 0.3)
        self.assertTrue(logwarn.called)

    def test_feedback_timeout_rejects_stale_end_effector(self):
        state = static_torque.StaticDownwardValveTorqueTest.__new__(
            static_torque.StaticDownwardValveTorqueTest)
        state.config = static_torque.SimpleNamespace(
            valve_feedback_timeout=10.0,
            pose_feedback_max_age=0.5)
        state._set_phase = mock.Mock()
        state.beetle = mock.Mock()
        state.beetle.getFreshValvePos.return_value = None
        state.formation_adapter = mock.Mock()
        state.formation_adapter.has_fresh_end_effector_feedback.return_value = False
        state.get_end_effector_position = mock.Mock(
            return_value=[0.2, -0.1, 1.0])
        state.get_end_effector_yaw = mock.Mock(return_value=0.3)

        with mock.patch.object(
                static_torque.rospy, "get_time",
                side_effect=[5.0, 15.0]), mock.patch.object(
                    static_torque.rospy, "logerr") as logerr, \
                mock.patch.object(static_torque.rospy, "Rate"), \
                mock.patch.object(
                    static_torque.rospy, "is_shutdown", return_value=False):
            self.assertIsNone(state._wait_for_feedback())

        self.assertTrue(logerr.called)

    def test_valve_and_module_pose_freshness_expires(self):
        valve = BeetleInterface.__new__(BeetleInterface)
        valve.valve_pose_received_time = 10.0
        valve.valve_pose = static_torque.SimpleNamespace(
            position=static_torque.SimpleNamespace(x=1.0, y=2.0, z=3.0))
        formation = FormationAdapter.__new__(FormationAdapter)
        formation.module_ids = [1, 3]
        formation.uav_pose_received_times = {1: 10.0, 3: 10.0}

        with mock.patch.object(
                static_torque.rospy, "get_time", return_value=10.4):
            np.testing.assert_allclose(
                valve.getFreshValvePos(0.5), [1.0, 2.0, 3.0])
            self.assertTrue(
                formation.has_fresh_end_effector_feedback(0.5))

        with mock.patch.object(
                static_torque.rospy, "get_time", return_value=10.6):
            self.assertIsNone(valve.getFreshValvePos(0.5))
            self.assertFalse(
                formation.has_fresh_end_effector_feedback(0.5))

    def test_stale_module_pose_blocks_contact_search(self):
        state = static_torque.StaticDownwardValveTorqueTest.__new__(
            static_torque.StaticDownwardValveTorqueTest)
        state.config = static_torque.SimpleNamespace(
            direction=1,
            min_search_angle=math.radians(2.0),
            max_search_angle=math.radians(45.0),
            stall_lead=math.radians(3.0),
            stall_rate=math.radians(0.5),
            velocity_window=0.4,
            required_cycles=8,
            search_speed=math.radians(3.0),
            search_ramp_time=2.0,
            zero_radius_tolerance=0.030,
            direction_label="CCW viewed from above",
            force_sensor_enabled=False)
        state._set_phase = mock.Mock()
        state._end_effector_feedback_is_fresh = mock.Mock(
            return_value=False)
        state.beetle = mock.Mock()
        state.beetle.getTaskHaltFlag.return_value = False

        with mock.patch.object(
                static_torque.rospy, "get_time", return_value=5.0), \
                mock.patch.object(static_torque.rospy, "Rate"), \
                mock.patch.object(
                    static_torque.rospy, "is_shutdown", return_value=False):
            self.assertFalse(state._search_contact(
                np.array([0.0, 0.0, 1.0])))

        state._end_effector_feedback_is_fresh.assert_called_once_with()

    def test_contact_search_starts_at_actual_pose_with_small_radius(self):
        state = static_torque.StaticDownwardValveTorqueTest.__new__(
            static_torque.StaticDownwardValveTorqueTest)
        state.config = static_torque.SimpleNamespace(
            direction=-1,
            min_search_angle=math.radians(2.0),
            max_search_angle=math.radians(3.0),
            stall_lead=math.radians(3.0),
            stall_rate=math.radians(0.5),
            velocity_window=0.4,
            required_cycles=8,
            search_speed=math.radians(3.0),
            search_ramp_time=2.0,
            zero_radius_tolerance=0.030,
            direction_label="CCW viewed from below",
            force_sensor_enabled=False)
        state._set_phase = mock.Mock()
        state._end_effector_feedback_is_fresh = mock.Mock(
            return_value=True)
        state.beetle = mock.Mock()
        state.beetle.getTaskHaltFlag.return_value = False
        start_position = np.array([0.503, -0.844, 1.0])
        valve_center = np.array([0.526, -0.854, 1.0])
        start_yaw = math.radians(-1.0)
        state.get_end_effector_position = mock.Mock(
            return_value=start_position)
        state.get_end_effector_yaw = mock.Mock(return_value=start_yaw)
        state.send_assembly_command_from_end_effector = mock.Mock()

        with mock.patch.object(
                static_torque.rospy, "get_time",
                side_effect=[10.0, 10.0, 10.0, 12.0, 12.0]), \
                mock.patch.object(
                    static_torque.rospy, "Rate"), mock.patch.object(
                        static_torque.rospy, "is_shutdown",
                        side_effect=[False, False, True]), mock.patch.object(
                            static_torque.rospy, "loginfo_throttle"):
            self.assertFalse(state._search_contact(
                valve_center))

        first_command, second_command, stop_command = (
            state.send_assembly_command_from_end_effector.call_args_list)
        np.testing.assert_allclose(first_command.args[0], start_position)
        self.assertAlmostEqual(first_command.args[1], start_yaw)
        np.testing.assert_allclose(
            first_command.kwargs["linear_vel"], [0.0, 0.0, 0.0])
        self.assertAlmostEqual(first_command.kwargs["angular_vel"], 0.0)

        np.testing.assert_allclose(second_command.args[0], start_position)
        self.assertAlmostEqual(
            second_command.args[1], start_yaw - math.radians(3.0))
        np.testing.assert_allclose(
            second_command.kwargs["linear_vel"], [0.0, 0.0, 0.0])
        self.assertAlmostEqual(
            second_command.kwargs["angular_vel"], -math.radians(3.0))
        np.testing.assert_allclose(stop_command.args[0], start_position)
        self.assertAlmostEqual(stop_command.args[1], start_yaw)
        self.assertEqual(stop_command.kwargs, {})

    def test_contact_search_speed_uses_smoothstep_ramp(self):
        target_speed = math.radians(3.0)
        ramp_time = 2.0

        start_progress, start_speed = smooth_start_angular_motion(
            0.0, target_speed, ramp_time)
        half_progress, half_speed = smooth_start_angular_motion(
            1.0, target_speed, ramp_time)
        end_progress, end_speed = smooth_start_angular_motion(
            2.0, target_speed, ramp_time)
        later_progress, later_speed = smooth_start_angular_motion(
            3.0, target_speed, ramp_time)

        self.assertAlmostEqual(start_progress, 0.0)
        self.assertAlmostEqual(start_speed, 0.0)
        self.assertAlmostEqual(half_progress, 0.1875 * target_speed)
        self.assertAlmostEqual(half_speed, 0.5 * target_speed)
        self.assertAlmostEqual(end_progress, target_speed)
        self.assertAlmostEqual(end_speed, target_speed)
        self.assertAlmostEqual(later_progress, 2.0 * target_speed)
        self.assertAlmostEqual(later_speed, target_speed)

    def test_existing_circular_target_is_valve_centered(self):
        state = FormationRotateValveState.__new__(FormationRotateValveState)
        target, yaw = state._circular_ee_target(
            [3.0, 2.0, 1.2], 0.1, math.pi / 2.0, 1.1)
        self.assertAlmostEqual(target[0], 3.0)
        self.assertAlmostEqual(target[1], 2.1)
        self.assertAlmostEqual(target[2], 1.1)
        self.assertAlmostEqual(yaw, -math.pi / 2.0)

    def test_zero_radius_fallback_starts_from_current_yaw(self):
        center = np.array([0.2, -0.1, 1.0])
        current_yaw = -0.7
        start_angle = circular_start_angle(
            center, center.copy(), current_yaw)
        state = FormationRotateValveState.__new__(FormationRotateValveState)
        target, target_yaw = state._circular_ee_target(
            center, 0.0, start_angle, center[2])
        next_target, next_yaw = state._circular_ee_target(
            center, 0.0, start_angle + 0.1, center[2])

        np.testing.assert_allclose(target, center)
        np.testing.assert_allclose(next_target, center)
        self.assertAlmostEqual(target_yaw, current_yaw)
        self.assertAlmostEqual(
            static_torque.normalize_angle(next_yaw - target_yaw), 0.1)

    def test_angular_stall_detects_fixed_valve_contact_across_wrap(self):
        detector = DirectedAngularStallDetector(
            direction=1,
            min_search_angle=math.radians(2.0),
            lead_threshold=math.radians(3.0),
            stall_rate=math.radians(0.5),
            velocity_window=0.4,
            required_cycles=3)
        start = math.radians(179.0)
        detector.reset(start, start, 0.0)
        observation = None
        for index in range(1, 18):
            observation = detector.update(
                start + math.radians(0.5 * index), start, 0.1 * index)
        self.assertTrue(observation["contact"])
        self.assertGreater(
            observation["command_progress"], math.radians(8.0))
        self.assertAlmostEqual(observation["actual_progress"], 0.0)

    def test_tracking_yaw_is_not_mistaken_for_contact(self):
        detector = DirectedAngularStallDetector(
            direction=-1,
            min_search_angle=math.radians(2.0),
            lead_threshold=math.radians(3.0),
            stall_rate=math.radians(0.5),
            velocity_window=0.4,
            required_cycles=3)
        detector.reset(0.0, 0.0, 0.0)
        for index in range(1, 30):
            yaw = -math.radians(0.5 * index)
            observation = detector.update(yaw, yaw, 0.1 * index)
            self.assertFalse(observation["contact"])

    def test_unload_duration_accounts_for_shifted_cog_torque(self):
        duration = minimum_smoothstep_duration(
            force=[0.0, 0.0, 5.0],
            torque=[0.0, -2.5, 1.0],
            minimum=0.5,
            force_rate=3.0,
            torque_rate=0.5)
        expected = 1.5 * math.sqrt(2.5 ** 2 + 1.0) / 0.5
        self.assertAlmostEqual(duration, expected)

    def test_static_task_publishes_transformed_complete_6d_wrench(self):
        class RecordingPublisher(object):
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        class FakeBeetle(object):
            def __init__(self):
                self.build_args = None
                self.command = None

            def buildCoGWrench(self, force, torque,
                               application_offset_body, frame_id):
                self.build_args = (
                    list(force), list(torque),
                    list(application_offset_body), frame_id)
                return [0.0, 0.0, 5.0], [0.0, -2.5, 1.0]

            def isUnifiedMode(self):
                return True

            def addExternalWrench(self, force, torque, frame_id,
                                  task_weights):
                self.command = (
                    list(force), list(torque), frame_id, list(task_weights))

        state = static_torque.StaticDownwardValveTorqueTest.__new__(
            static_torque.StaticDownwardValveTorqueTest)
        state.beetle = FakeBeetle()
        state._ee_offset_body = lambda: [0.5, 0.0, 0.1]
        state.application_wrench_pub = RecordingPublisher()
        state.cog_wrench_pub = RecordingPublisher()
        with mock.patch.object(
                static_torque.rospy.Time, "now",
                return_value=static_torque.rospy.Time(1.0)):
            state._publish_application_wrench(
                [0.0, 0.0, 5.0], [0.0, 0.0, 1.0])

        self.assertEqual(
            state.beetle.build_args,
            ([0.0, 0.0, 5.0], [0.0, 0.0, 1.0],
             [0.5, 0.0, 0.1], "world"))
        self.assertEqual(
            state.beetle.command,
            ([0.0, 0.0, 5.0], [0.0, -2.5, 1.0], "fc",
             static_torque.STATIC_VALVE_TASK_WRENCH_WEIGHTS))
        self.assertEqual(len(state.application_wrench_pub.messages), 1)
        self.assertEqual(len(state.cog_wrench_pub.messages), 1)

    def test_session_order_is_contact_then_ramp_hold_then_unload(self):
        calls = []

        class FakeBeetle(object):
            external_wrench_active = False

            def setAttachModule(self, module_id):
                calls.append(("detach", module_id))

        state = static_torque.StaticDownwardValveTorqueTest.__new__(
            static_torque.StaticDownwardValveTorqueTest)
        state.beetle = FakeBeetle()
        state._prepare_force_sensor = lambda: (
            calls.append("cfs_ready") or True)
        state._wait_for_feedback = lambda: (
            calls.append("feedback") or
            ([0.0, 0.0, 1.0], [0.1, 0.0, 1.0], 0.0))
        state._search_contact = lambda *args: (
            calls.append("contact") or True)
        state._run_wrench_profile = lambda: (
            calls.append("ramp_hold") or True)
        state._unload = lambda: calls.append("unload") or True
        state._set_phase = lambda phase: calls.append(phase)

        self.assertEqual(state.execute(), "succeeded")
        self.assertEqual(
            calls,
            ["cfs_ready", "feedback", "contact", "ramp_hold", "unload", "complete",
             ("detach", None)])

    def test_abnormal_exit_immediately_clears_active_wrench(self):
        calls = []

        class FakeBeetle(object):
            external_wrench_active = True

            def clearExternalWrench(self):
                calls.append("clear")
                self.external_wrench_active = False

            def setAttachModule(self, module_id):
                calls.append(("detach", module_id))

        state = static_torque.StaticDownwardValveTorqueTest.__new__(
            static_torque.StaticDownwardValveTorqueTest)
        state.beetle = FakeBeetle()
        state._prepare_force_sensor = lambda: True
        state._wait_for_feedback = lambda: (
            [0.0, 0.0, 1.0], [0.1, 0.0, 1.0], 0.0)
        state._search_contact = lambda *args: False

        self.assertEqual(state.execute(), "failed")
        self.assertEqual(calls, ["clear", ("detach", None)])


if __name__ == "__main__":
    unittest.main()
