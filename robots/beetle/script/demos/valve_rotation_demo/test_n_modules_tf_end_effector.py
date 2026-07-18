#!/usr/bin/env python3
"""Tests for explicit end-effector carrier selection in formation geometry."""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import n_modules_tf
from n_modules_tf import NModuleTFCalculator


class EndEffectorModuleSelectionTest(unittest.TestCase):

    def _calculator(self, module_ids, host=None):
        with mock.patch.object(n_modules_tf.rospy, "loginfo"):
            return NModuleTFCalculator(
                module_ids, end_effector_module_id=host)

    def test_default_remains_final_module(self):
        calculator = self._calculator("8,2,5")
        self.assertEqual(calculator.get_end_effector_module_id(), 5)
        self.assertEqual(calculator.get_leader_id(), 5)
        self.assertAlmostEqual(
            calculator.
            calculate_assembly_to_end_effector_host_transform()["offset_x"],
            0.52)

    def test_first_middle_and_final_hosts_follow_physical_list_order(self):
        expected_offsets = {8: -0.52, 2: 0.0, 5: 0.52}
        for host, expected_x in expected_offsets.items():
            calculator = self._calculator("8,2,5", host)
            transform = (
                calculator.
                calculate_assembly_to_end_effector_host_transform())
            self.assertAlmostEqual(transform["offset_x"], expected_x)
            self.assertEqual(
                transform,
                calculator.calculate_assembly_to_leader_transform())

    def test_two_module_host_changes_application_point_moment_arm(self):
        first = self._calculator("1,2", 1)
        final = self._calculator("1,2", 2)
        first_x = (first.calculate_assembly_to_end_effector_host_transform()
                   ["offset_x"] + 0.240)
        final_x = (final.calculate_assembly_to_end_effector_host_transform()
                   ["offset_x"] + 0.240)
        self.assertAlmostEqual(first_x, -0.020)
        self.assertAlmostEqual(final_x, 0.500)
        # For Fz=5 N, (r x F)y = -rx*Fz.
        self.assertAlmostEqual(-first_x * 5.0, 0.10)
        self.assertAlmostEqual(-final_x * 5.0, -2.50)

    def test_invalid_configurations_fail_closed(self):
        with self.assertRaises(ValueError):
            self._calculator("1,2", 3)
        with self.assertRaises(ValueError):
            self._calculator("1,1", 1)
        with self.assertRaises(ValueError):
            self._calculator("0,2", 2)
        with self.assertRaises(ValueError):
            self._calculator("1,2", "not-an-id")


if __name__ == "__main__":
    unittest.main()
