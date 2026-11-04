#!/usr/bin/env python3
"""Offline regressions for the procedural keyboard node; no ROS publishers."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import yaml
import rospy
from aerial_robot_msgs.msg import FlightNav

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("hih_key", ROOT / "script/hih_key.py")
key = importlib.util.module_from_spec(spec)
with patch.object(rospy, "init_node") as initialize:
    spec.loader.exec_module(key)
    assert not initialize.called


class KeyboardTest(unittest.TestCase):
    def setUp(self):
        self.config = key.load_config(ROOT / "config/hih_key.yaml")

    def run_keys(self, steps, connected=("/hih1", "/hih2")):
        import copy
        import io
        from contextlib import ExitStack, redirect_stdout
        clock = [10.0]
        steps = iter(list(steps) + [("\x03", .05)])
        received = {}
        connections = {}

        def read_key(*_args):
            character, duration = next(steps)
            clock[0] += duration
            return character

        def publisher(topic, *_args, **_kwargs):
            pub = Mock()
            pub.get_num_connections.return_value = int(any(topic.startswith(ns + "/") for ns in connected))
            connections[topic] = pub
            received[topic] = []
            pub.publish.side_effect = lambda msg: received[topic].append(copy.deepcopy(msg))
            return pub

        with ExitStack() as stack, redirect_stdout(io.StringIO()):
            stack.enter_context(patch.object(key.rospy, "myargv", return_value=["hih_key.py", "--config", str(ROOT / "config/hih_key.yaml")]))
            stack.enter_context(patch.object(key.sys, "stdin"))
            stack.enter_context(patch.object(key, "get_key", side_effect=read_key))
            stack.enter_context(patch.object(key.time, "monotonic", side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(key.time, "sleep"))
            stack.enter_context(patch.object(key.termios, "tcgetattr", return_value=[]))
            restore = stack.enter_context(patch.object(key.termios, "tcsetattr"))
            stack.enter_context(patch.object(key.tty, "setcbreak"))
            stack.enter_context(patch.object(key.signal, "signal"))
            stack.enter_context(patch.object(key.rospy, "init_node"))
            stack.enter_context(patch.object(key.rospy, "is_shutdown", return_value=False))
            stack.enter_context(patch.object(key.rospy, "signal_shutdown"))
            stack.enter_context(patch.object(key.rospy, "logwarn"))
            stack.enter_context(patch.object(key.rospy, "loginfo"))
            stack.enter_context(patch.object(key.rospy, "Publisher", side_effect=publisher))
            stack.enter_context(patch.object(key.rospy.Time, "now", return_value=rospy.Time.from_sec(42)))
            key.main()
            restore.assert_called_once()
        return received, connections

    def test_all_requested_motion_keys(self):
        expected = {"w": ("hih2", 0, .1), "s": ("hih2", 0, -.1),
                    "a": ("hih2", 1, .1), "d": ("hih2", 1, -.1),
                    "r": ("hih2", 2, .1), "f": ("hih2", 2, -.1),
                    "q": ("hih2", 3, .1), "e": ("hih2", 3, -.1),
                    "i": ("hih1", 0, .1), "k": ("hih1", 0, -.1),
                    "j": ("hih1", 1, .1), "l": ("hih1", 1, -.1),
                    "p": ("hih1", 2, .1), ";": ("hih1", 2, -.1),
                    "u": ("hih1", 3, .1), "o": ("hih1", 3, -.1)}
        for character, (namespace, axis, value) in expected.items():
            with self.subTest(character=character):
                received, _ = self.run_keys([(character, .05)])
                msg = received[f"/{namespace}/uav/nav"][0]
                vector = [0.0] * 4; vector[axis] = value
                self.assertEqual([getattr(msg, field) for field in key.VELOCITY_FIELDS], vector)
                other = "hih1" if namespace == "hih2" else "hih2"
                self.assertFalse(any(any(getattr(msg, field) for field in key.VELOCITY_FIELDS)
                                     for msg in received[f"/{other}/uav/nav"]))

    def test_one_robot_connected_still_receives_every_common_command(self):
        for ns in ("/hih1", "/hih2"):
            with self.subTest(connected=ns):
                received, pubs = self.run_keys([(char, .1) for char in "vbnm/"], connected=(ns,))
                for topic in ("start", "takeoff", "land", "force_landing", "halt"):
                    name = ns + "/teleop_command/" + topic
                    self.assertEqual(pubs[name].get_num_connections.return_value, 1)
                    self.assertEqual(len(received[name]), 1)

    def test_one_robot_connected_still_receives_its_movement(self):
        for ns, char in (("/hih1", "i"), ("/hih2", "w")):
            received, _ = self.run_keys([(char, .05)], connected=(ns,))
            self.assertEqual(received[ns + "/uav/nav"][0].target_vel_x, .1)

    def test_timeout_sends_three_zero_messages_then_idle(self):
        received, _ = self.run_keys([("w", .05)] + [(None, .1)] * 9)
        messages = received["/hih2/uav/nav"]
        moving = [msg.target_vel_x for msg in messages]
        first_zero = moving.index(0)
        # Three timeout zeros, then three zeros on Ctrl+C; no idle stream.
        self.assertEqual(len(moving[first_zero:]), 6)
        self.assertTrue(all(value == 0 for value in moving[first_zero:]))

    def test_axes_expire_independently(self):
        received, _ = self.run_keys([("w", .05), ("q", .2), (None, .2)])
        msg = received["/hih2/uav/nav"][-4]
        self.assertEqual((msg.target_vel_x, msg.target_omega_z), (0, .1))

    def test_opposite_direction_replaces_axis(self):
        received, _ = self.run_keys([("w", .05), ("s", .1)])
        self.assertEqual(received["/hih2/uav/nav"][1].target_vel_x, -.1)

    def test_common_command_debounce(self):
        received, _ = self.run_keys([("v", .05), ("v", .1), ("v", 1)])
        self.assertEqual(len(received["/hih1/teleop_command/start"]), 2)

    def test_space_stops_both_robots(self):
        received, _ = self.run_keys([("w", .05), ("i", .1), (" ", .1)])
        for ns in ("/hih1", "/hih2"):
            msgs = received[ns + "/uav/nav"]
            self.assertTrue(any(msg.target_vel_x == .1 for msg in msgs))
            self.assertEqual(msgs[-4].target_vel_x, 0)

    def test_unknown_keys_and_startup_do_not_send_motion_or_flight_commands(self):
        received, _ = self.run_keys([(None, .1), ("W", .1), ("h", .1)])
        for topic, messages in received.items():
            if topic.endswith("/uav/nav"):
                self.assertEqual(len(messages), 3)  # Exit zeros only.
                self.assertTrue(all(not any(getattr(msg, f) for f in key.VELOCITY_FIELDS) for msg in messages))
            else:
                self.assertEqual(messages, [])

    def test_actual_nav_message_does_not_command_absolute_origin(self):
        with patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(42)):
            msg = key.make_nav((.1, -.2, .3, -.4), self.config)
        self.assertEqual(msg.control_frame, FlightNav.WORLD_FRAME)
        self.assertEqual(msg.target, FlightNav.COG)
        self.assertEqual((msg.pos_xy_nav_mode, msg.pos_z_nav_mode, msg.yaw_nav_mode), (1, 1, 1))
        self.assertEqual((msg.target_vel_x, msg.target_vel_y, msg.target_vel_z, msg.target_omega_z), (.1, -.2, .3, -.4))
        self.assertEqual((msg.roll_nav_mode, msg.pitch_nav_mode), (0, 0))

    def test_local_frame(self):
        self.config["frame"] = "local"
        with patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(42)):
            self.assertEqual(key.make_nav((0, 0, 0, 0), self.config).control_frame, FlightNav.LOCAL_FRAME)

    def test_common_mapping(self):
        common = self.config["keys"]["common"]
        self.assertEqual({a: common[a] for a in key.COMMANDS},
                         {"arm": "v", "takeoff": "b", "land": "n", "force_landing": "m", "halt": "/"})
        self.assertNotEqual(key.COMMANDS["force_landing"], key.COMMANDS["halt"])

    def test_rejects_duplicate_mapping_and_bad_speed(self):
        for change in ("duplicate", "speed", "namespace"):
            data = yaml.safe_load((ROOT / "config/hih_key.yaml").read_text())
            if change == "duplicate": data["keys"]["leader"]["+X"] = "v"
            if change == "speed": data["robots"]["leader"]["xy_speed"] = float("nan")
            if change == "namespace": data["robots"]["leader"]["namespace"] = "/hih2"
            with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as stream:
                yaml.safe_dump(data, stream); stream.flush()
                with self.subTest(change=change), self.assertRaises(ValueError): key.load_config(stream.name)


if __name__ == "__main__": unittest.main()
