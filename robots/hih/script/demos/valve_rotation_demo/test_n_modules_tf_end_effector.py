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

    def _calculator(self, module_ids, host=None, end_effector_offset_body=None):
        with mock.patch.object(n_modules_tf.rospy, "loginfo"):
            return NModuleTFCalculator(
                module_ids, end_effector_module_id=host,
                end_effector_offset_body=end_effector_offset_body)

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
        downward_offset = [0.240, 0.0, 0.11053]
        first = self._calculator("1,2", 1, downward_offset)
        final = self._calculator("1,2", 2, downward_offset)
        first_point = first.transform_assembly_to_end_effector(
            (0.0, 0.0, 0.0), 0.0)
        final_point = final.transform_assembly_to_end_effector(
            (0.0, 0.0, 0.0), 0.0)
        self.assertAlmostEqual(first_point[0], -0.020)
        self.assertAlmostEqual(final_point[0], 0.500)
        self.assertAlmostEqual(first_point[2], 0.11053)
        self.assertAlmostEqual(final_point[2], 0.11053)
        # For Fz=5 N, (r x F)y = -rx*Fz.
        self.assertAlmostEqual(-first_point[0] * 5.0, 0.10)
        self.assertAlmostEqual(-final_point[0] * 5.0, -2.50)

    def test_task_specific_offset_does_not_change_legacy_default(self):
        legacy = self._calculator("2,3", 3)
        downward = self._calculator(
            "2,3", 3, [0.240, 0.0, 0.11053])
        self.assertEqual(
            legacy.transform_assembly_to_end_effector(
                (0.0, 0.0, 0.0), 0.0),
            (0.506, 0.0, 0.074382))
        self.assertEqual(
            downward.transform_assembly_to_end_effector(
                (0.0, 0.0, 0.0), 0.0),
            (0.5, 0.0, 0.11053))

    def test_invalid_configurations_fail_closed(self):
        with self.assertRaises(ValueError):
            self._calculator("1,2", 3)
        with self.assertRaises(ValueError):
            self._calculator("1,1", 1)
        with self.assertRaises(ValueError):
            self._calculator("0,2", 2)
        with self.assertRaises(ValueError):
            self._calculator("1,2", "not-an-id")
        with self.assertRaises(ValueError):
            self._calculator("1,2", 2, [0.24, 0.0])


if __name__ == "__main__":
    unittest.main()
