#!/usr/bin/env python3
"""Unit tests for the static downward-valve contact and wrench profile."""

import math
import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import static_downward_valve_torque as static_torque
from static_downward_valve_torque import (
    DirectedAngularStallDetector,
    minimum_smoothstep_duration,
    parse_rotation_direction,
)
from valve_rotation_formation_clean import FormationRotateValveState


class StaticDownwardValveRotationTest(unittest.TestCase):

    def test_downward_task_has_its_own_host_to_tool_offset(self):
        with mock.patch.object(
                static_torque.rospy, "get_param",
                side_effect=lambda name, default: default):
            config = static_torque.read_config()
        self.assertEqual(
            config.end_effector_offset_body.tolist(),
            [0.240, 0.0, 0.11053])

    def test_downward_view_reverses_visual_direction(self):
        self.assertEqual(parse_rotation_direction("cw", "below")[0], 1)
        self.assertEqual(parse_rotation_direction("ccw", "below")[0], -1)
        self.assertEqual(parse_rotation_direction("cw", "above")[0], -1)
        self.assertEqual(parse_rotation_direction("ccw", "above")[0], 1)

    def test_existing_circular_target_is_valve_centered(self):
        state = FormationRotateValveState.__new__(FormationRotateValveState)
        target, yaw = state._circular_ee_target(
            [3.0, 2.0, 1.2], 0.1, math.pi / 2.0, 1.1)
        self.assertAlmostEqual(target[0], 3.0)
        self.assertAlmostEqual(target[1], 2.1)
        self.assertAlmostEqual(target[2], 1.1)
        self.assertAlmostEqual(yaw, -math.pi / 2.0)

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
            ["feedback", "contact", "ramp_hold", "unload", "complete",
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
        state._wait_for_feedback = lambda: (
            [0.0, 0.0, 1.0], [0.1, 0.0, 1.0], 0.0)
        state._search_contact = lambda *args: False

        self.assertEqual(state.execute(), "failed")
        self.assertEqual(calls, ["clear", ("detach", None)])


if __name__ == "__main__":
    unittest.main()
