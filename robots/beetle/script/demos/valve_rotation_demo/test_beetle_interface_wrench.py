#!/usr/bin/env python3
"""Unit tests for BeetleInterface CoG wrench construction."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from beetle_interface import BeetleInterface


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


if __name__ == '__main__':
    unittest.main()
