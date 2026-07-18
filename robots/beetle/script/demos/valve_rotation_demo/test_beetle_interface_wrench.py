#!/usr/bin/env python3
"""Unit tests for BeetleInterface CoG wrench construction."""

import math
import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import beetle_interface
from beetle_interface import BeetleInterface


class RecordingPublisher(object):

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class BeetleInterfaceWrenchTest(unittest.TestCase):

    def setUp(self):
        self.interface = BeetleInterface.__new__(BeetleInterface)
        self.interface.getUavRPY = lambda: (0.0, 0.0, math.pi / 2.0)

    def test_world_yaw_point_force_builds_body_cog_wrench(self):
        force, torque = self.interface.buildCoGWrench(
            [0.0, 5.0, 0.0],
            application_offset_body=[0.32, 0.0, 0.00053],
            frame_id="world_yaw")

        self.assertAlmostEqual(force[0], 5.0)
        self.assertAlmostEqual(force[1], 0.0)
        self.assertAlmostEqual(force[2], 0.0)
        self.assertAlmostEqual(torque[0], 0.0)
        self.assertAlmostEqual(torque[1], 0.00265)
        self.assertAlmostEqual(torque[2], 0.0)

    def test_body_frame_input_is_not_rotated_again(self):
        force, torque = self.interface.buildCoGWrench(
            [5.0, 0.0, 0.0],
            application_offset_body=[0.32, 0.0, 0.00053],
            frame_id="fc")

        self.assertEqual(force, [5.0, 0.0, 0.0])
        self.assertAlmostEqual(torque[0], 0.0)
        self.assertAlmostEqual(torque[1], 0.00265)
        self.assertAlmostEqual(torque[2], 0.0)

    def test_explicit_trusted_orientation_overrides_cached_odom(self):
        force, torque = self.interface.buildCoGWrench(
            [5.0, 0.0, 0.0],
            application_offset_body=[0.0, 0.0, 0.0],
            frame_id="world",
            orientation_quaternion=[0.0, 0.0, 0.0, 1.0])

        # setUp() exposes a stale 90deg cached yaw; the explicit trusted
        # quaternion must make this identity-frame conversion instead.
        self.assertEqual(force, [5.0, 0.0, 0.0])
        self.assertEqual(torque, [0.0, 0.0, 0.0])

    def test_downward_valve_full_6d_wrench_is_shifted_to_formation_cog(self):
        force, torque = self.interface.buildCoGWrench(
            [0.0, 0.0, 5.0],
            torque=[0.0, 0.0, 1.0],
            application_offset_body=[0.500, 0.0, 0.11053],
            frame_id="world")

        # The complete CoG wrench keeps Fz/Tz and includes r x F: Ty=-x*Fz.
        self.assertAlmostEqual(force[0], 0.0)
        self.assertAlmostEqual(force[1], 0.0)
        self.assertAlmostEqual(force[2], 5.0)
        self.assertAlmostEqual(torque[0], 0.0)
        self.assertAlmostEqual(torque[1], -2.5)
        self.assertAlmostEqual(torque[2], 1.0)

    def test_safety_route_is_pinned_independently_of_live_mode(self):
        interface = BeetleInterface.__new__(BeetleInterface)
        interface.assembly_mode = True
        interface.single_uav_wrench_mode = False
        interface.external_wrench_active = False
        interface._inter_auto_publish = False
        interface.formation_wrench_pub = RecordingPublisher()
        interface.formation_wrench_weights_pub = RecordingPublisher()
        interface.desired_ext_wrench_pub = RecordingPublisher()
        interface.desired_ext_wrench_weights_pub = RecordingPublisher()

        # Live mode says LF, but the preflight-fixed unified route wins for
        # both the six-axis wrench and its six weights.
        interface.isUnifiedMode = lambda: False
        with mock.patch.object(
                beetle_interface.rospy.Time, "now",
                return_value=beetle_interface.rospy.Time(1.0)):
            interface.addExternalWrench(
                [0.0, 0.0, 5.0], [0.0, -2.4, 1.0], frame_id="fc",
                task_weights=[0.4, 0.4, 0.8, 0.4, 0.8, 2.0],
                route_unified=True)
        self.assertEqual(len(interface.formation_wrench_pub.messages), 1)
        self.assertEqual(
            len(interface.formation_wrench_weights_pub.messages), 1)
        self.assertEqual(len(interface.desired_ext_wrench_pub.messages), 0)

        # And the reverse transition cannot refresh a nonzero sample on the
        # newly live unified alias.
        interface.isUnifiedMode = lambda: True
        with mock.patch.object(
                beetle_interface.rospy.Time, "now",
                return_value=beetle_interface.rospy.Time(2.0)):
            interface.addExternalWrench(
                [0.0, 0.0, 4.0], [0.0, -1.9, 0.5], frame_id="fc",
                task_weights=[1.0] * 6, route_unified=False)
        self.assertEqual(len(interface.formation_wrench_pub.messages), 1)
        self.assertEqual(len(interface.desired_ext_wrench_pub.messages), 1)
        self.assertEqual(
            len(interface.desired_ext_wrench_weights_pub.messages), 1)

    def test_first_joy_frame_and_held_halt_are_safe(self):
        interface = BeetleInterface.__new__(BeetleInterface)
        interface.prev_joy_state = beetle_interface.Joy()
        interface.halt_task = False
        interface.force_skip = False
        held = beetle_interface.Joy()
        held.buttons = [0, 0, 0, 0, 1, 1]

        interface._joy_cb(held)
        self.assertTrue(interface.halt_task)
        self.assertTrue(interface.force_skip)

        # A reset cannot make a continuously held emergency button ineffective.
        interface.halt_task = False
        interface._joy_cb(held)
        self.assertTrue(interface.halt_task)


if __name__ == '__main__':
    unittest.main()
