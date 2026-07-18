#!/usr/bin/env python3
"""Tests for n_module session reuse and simulation-only mode switching."""

import os
import re
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import static_downward_valve_torque as torque


class StaticDownwardValveSessionTest(unittest.TestCase):

    @staticmethod
    def _upward_session(module_ids, host_id):
        session = {
            torque.N_MODULE_SESSION_NS + "/module_ids": ",".join(
                str(module_id) for module_id in module_ids),
            torque.N_MODULE_SESSION_NS + "/end_effector_module_id": host_id,
            torque.N_MODULE_SESSION_NS + "/end_effector_model": (
                torque.REQUIRED_END_EFFECTOR_MODEL),
        }
        for module_id in module_ids:
            marker = (
                '<link name="%s"/>' % torque.REQUIRED_END_EFFECTOR_LINK
                if module_id == host_id else
                '<robot name="beetle"/>')
            session["/beetle%d/robot_description" % module_id] = marker
        return session

    @staticmethod
    def _config(parameter_overrides, session=None):
        values = dict(parameter_overrides)
        values.update(session or {})

        def get_param(name, default=None):
            return values.get(name, default)

        with mock.patch.object(torque.rospy, "get_param", get_param), \
                mock.patch.object(
                    torque.rospy, "has_param", lambda name: name in values):
            return torque.TorqueTestConfig()

    def test_simulation_auto_reuses_ordered_n_module_session(self):
        session = self._upward_session([8, 2, 5], 2)
        config = self._config({
            "~real_machine": False,
            "~simulation": True,
        }, session=session)
        self.assertEqual(config.module_ids, [8, 2, 5])
        self.assertEqual(config.end_effector_module_id, 2)

    def test_explicit_static_host_mismatch_with_session_is_rejected(self):
        session = self._upward_session([1, 2], 2)
        with self.assertRaises(ValueError):
            self._config({
                "~real_machine": False,
                "~simulation": True,
                "~module_ids": "1,2",
                "~end_effector_module_id": 1,
            }, session=session)

    def test_wrong_session_model_is_rejected(self):
        session = self._upward_session([1, 2], 2)
        session[
            torque.N_MODULE_SESSION_NS + "/end_effector_model"
        ] = "beetle_fang"
        with self.assertRaises(ValueError):
            self._config({
                "~real_machine": False,
                "~simulation": True,
            }, session=session)

    def test_live_model_on_wrong_module_is_rejected(self):
        session = self._upward_session([1, 2], 2)
        session["/beetle1/robot_description"] = (
            '<link name="%s"/>' % torque.REQUIRED_END_EFFECTOR_LINK)
        with self.assertRaises(ValueError):
            self._config({
                "~real_machine": False,
                "~simulation": True,
            }, session=session)

    def test_auto_enable_unified_is_rejected_on_hardware(self):
        with self.assertRaises(ValueError):
            self._config({
                "~real_machine": True,
                "~simulation": False,
                "~auto_enable_unified": True,
            })

    def test_service_switch_is_restored_on_exit(self):
        experiment = torque.StaticDownwardValveTorqueTest.__new__(
            torque.StaticDownwardValveTorqueTest)
        experiment.config = SimpleNamespace(
            auto_enable_unified=True, module_ids=[1, 2])
        experiment._unified_restore_modes = None
        modes = {
            "/beetle1/controller/unified_control_mode": False,
            "/beetle2/controller/unified_control_mode": False,
        }
        calls = []

        def service_proxy(name, _service_type):
            match = re.match(r"/beetle(\d+)/controller/set_unified_mode", name)
            self.assertIsNotNone(match)
            module_id = int(match.group(1))

            def invoke(enabled):
                calls.append((module_id, bool(enabled)))
                modes[
                    "/beetle%d/controller/unified_control_mode" % module_id
                ] = bool(enabled)
                return SimpleNamespace(success=True, message="ok")
            return invoke

        with mock.patch.object(
                torque.rospy, "has_param", lambda name: name in modes), \
                mock.patch.object(
                    torque.rospy, "get_param",
                    lambda name, default=None: modes.get(name, default)), \
                mock.patch.object(torque.rospy, "wait_for_service"), \
                mock.patch.object(
                    torque.rospy, "ServiceProxy", service_proxy), \
                mock.patch.object(torque.rospy, "get_time", return_value=0.0), \
                mock.patch.object(torque.rospy, "loginfo"), \
                mock.patch.object(torque.rospy, "logerr"):
            self.assertTrue(experiment._auto_enable_unified_mode())
            self.assertTrue(all(modes.values()))
            experiment._restore_auto_enabled_unified_mode()
            self.assertFalse(any(modes.values()))

        self.assertEqual(
            calls, [(1, True), (2, True), (1, False), (2, False)])


if __name__ == "__main__":
    unittest.main()
