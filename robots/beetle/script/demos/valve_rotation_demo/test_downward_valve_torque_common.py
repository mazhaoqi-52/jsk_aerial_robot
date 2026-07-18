#!/usr/bin/env python3
"""Unit tests for the ROS-independent downward-valve torque helpers."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from downward_valve_torque_common import (
    DirectedAngularStallDetector,
    DualPinToolGeometry,
    minimum_unload_duration,
    parse_rotation_direction,
    parse_unified_heartbeat_id,
    slew_toward,
)


class DownwardValveTorqueCommonTest(unittest.TestCase):

    def test_dual_pin_midpoint_geometry(self):
        geometry = DualPinToolGeometry(0.240, 0.075, 0.11053)
        self.assertEqual(geometry.center, [0.240, 0.0, 0.11053])
        self.assertEqual(
            geometry.pin_centers[0], [0.240, 0.075, 0.11053])
        self.assertEqual(
            geometry.pin_centers[1], [0.240, -0.075, 0.11053])
        self.assertAlmostEqual(geometry.separation, 0.150)

    def test_downward_visual_direction_reverses_world_yaw_sign(self):
        self.assertEqual(parse_rotation_direction("cw", "above")[0], -1)
        self.assertEqual(parse_rotation_direction("ccw", "above")[0], 1)
        self.assertEqual(parse_rotation_direction("cw", "below")[0], 1)
        self.assertEqual(parse_rotation_direction("ccw", "below")[0], -1)

    def test_active_unified_heartbeat_id_is_unambiguous(self):
        payload = (
            "event=stage id=2 nav=5 module_state=3 force_landing=0")
        self.assertEqual(parse_unified_heartbeat_id(payload), 2)
        self.assertIsNone(parse_unified_heartbeat_id("event=stage id=bad"))
        self.assertIsNone(parse_unified_heartbeat_id("event=stage nav=5"))

    def test_stall_detector_crosses_angle_wrap(self):
        detector = DirectedAngularStallDetector(
            direction=1,
            min_search_angle=math.radians(2.0),
            lead_threshold=math.radians(3.0),
            stall_rate=math.radians(0.5),
            velocity_window=0.4,
            required_cycles=3)
        start = math.radians(179.0)
        detector.reset(start, start, 0.0)
        result = None
        # Command crosses +pi while actual yaw remains mechanically blocked.
        for index in range(1, 18):
            stamp = 0.1 * index
            command = start + math.radians(0.5 * index)
            result = detector.update(command, start, stamp)
        self.assertTrue(result["contact"])
        self.assertGreater(result["command_progress"], math.radians(8.0))
        self.assertAlmostEqual(result["actual_progress"], 0.0)

    def test_detector_does_not_call_tracking_motion_contact(self):
        detector = DirectedAngularStallDetector(
            direction=-1,
            min_search_angle=math.radians(2.0),
            lead_threshold=math.radians(3.0),
            stall_rate=math.radians(0.5),
            velocity_window=0.4,
            required_cycles=3)
        detector.reset(0.0, 0.0, 0.0)
        for index in range(1, 30):
            stamp = 0.1 * index
            yaw = -math.radians(0.5 * index)
            result = detector.update(yaw, yaw, stamp)
            self.assertFalse(result["contact"])

    def test_detector_cancels_fixed_host_assembly_yaw_offset(self):
        detector = DirectedAngularStallDetector(
            direction=1,
            min_search_angle=math.radians(2.0),
            lead_threshold=math.radians(3.0),
            stall_rate=math.radians(0.5),
            velocity_window=0.4,
            required_cycles=3)
        host_mounting_offset = math.radians(12.0)
        detector.reset(0.0, host_mounting_offset, 0.0)
        for index in range(1, 30):
            stamp = 0.1 * index
            progress = math.radians(0.5 * index)
            result = detector.update(
                progress, host_mounting_offset + progress, stamp)
            self.assertFalse(result["contact"])
            self.assertAlmostEqual(result["directed_lead"], 0.0)

    def test_slew_and_6d_unload_duration_respect_torque(self):
        self.assertAlmostEqual(slew_toward(0.0, 1.0, 0.25, 1.0), 0.25)
        self.assertAlmostEqual(slew_toward(0.9, 1.0, 0.25, 1.0), 1.0)
        duration = minimum_unload_duration(
            force=[0.0, 0.0, 3.0],
            torque=[0.0, -2.4, 1.0],
            minimum=0.5,
            force_rate=3.0,
            torque_rate=0.5)
        self.assertAlmostEqual(duration, math.sqrt(2.4 ** 2 + 1.0) / 0.5)


if __name__ == "__main__":
    unittest.main()
