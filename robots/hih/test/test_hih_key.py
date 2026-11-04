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

    def run_keys(self, steps, connected=("/hih1", "/hih2"), config=None):
        import copy
        import io
        from contextlib import ExitStack, redirect_stdout
        clock = [10.0]
        steps = iter(list(steps) + [("\x03", .05)])
        received = {}
        connections = {}
        self.received_times = {}
        stream = io.StringIO()

        def read_key(*_args):
            character, duration = next(steps)
            clock[0] += duration
            return character

        def publisher(topic, *_args, **_kwargs):
            pub = Mock()
            pub.get_num_connections.return_value = int(any(topic.startswith(ns + "/") for ns in connected))
            connections[topic] = pub
            received[topic] = []
            self.received_times[topic] = []
            def deliver(msg):
                received[topic].append(copy.deepcopy(msg))
                self.received_times[topic].append(clock[0])
            pub.publish.side_effect = deliver
            return pub

        with ExitStack() as stack, redirect_stdout(stream):
            stack.enter_context(patch.object(key, "load_config", return_value=copy.deepcopy(config or self.config)))
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
            stack.enter_context(patch.object(key.rospy, "loginfo", side_effect=lambda text, *args: print(text % args if args else text)))
            stack.enter_context(patch.object(key.rospy, "Publisher", side_effect=publisher))
            stack.enter_context(patch.object(key.rospy.Time, "now", return_value=rospy.Time.from_sec(42)))
            key.main()
            restore.assert_called_once()
        self.output = stream.getvalue()
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

    def test_trajectory_segment_integrals_and_order(self):
        import math
        first = self.config["trajectories"]["1"]["displacement"]
        second = self.config["trajectories"]["2"]["displacement"]
        yaw = math.radians(self.config["trajectories"]["2"]["yaw_displacement_deg"])
        expected = {
            "1": [(0, first), (2, first), (0, -first), (2, -first)],
            "2": [(3, yaw), (1, -second), (3, -yaw), (0, -second)]}
        for number, segments in expected.items():
            params = self.config["trajectories"][number]
            duration = params["period"] / 4
            for segment, (axis, displacement) in enumerate(segments):
                with self.subTest(number=number, segment=segment):
                    dt = duration / 1000
                    total = [0.0] * 4
                    for sample in range(1000):
                        velocity = key.trajectory_velocity(number, segment * duration + (sample + .5) * dt, params)
                        total = [value + speed * dt for value, speed in zip(total, velocity)]
                    for index, value in enumerate(total):
                        self.assertAlmostEqual(value, displacement if index == axis else 0, places=8)

    def test_trajectory_endpoints_are_zero_and_end_once(self):
        for number, params in self.config["trajectories"].items():
            for segment in range(4):
                velocity = key.trajectory_velocity(number, segment * params["period"] / 4, params)
                self.assertEqual(velocity, (0, 0, 0, 0))
            self.assertIsNone(key.trajectory_velocity(number, params["period"], params))
            self.assertIsNone(key.trajectory_velocity(number, -1, params))

    def test_trajectory_uses_yaml_displacement_and_period(self):
        params = dict(self.config["trajectories"]["1"])
        first = key.trajectory_velocity("1", params["period"] / 8, params)[0]
        params["displacement"] *= 2
        self.assertAlmostEqual(key.trajectory_velocity("1", params["period"] / 8, params)[0], 2 * first)
        params["period"] *= 2
        self.assertAlmostEqual(key.trajectory_velocity("1", params["period"] / 8, params)[0], first)

    def test_trajectory_world_frame_and_leader_only(self):
        config = __import__("copy").deepcopy(self.config)
        config["frame"] = "local"
        received, _ = self.run_keys([("1", .05), (None, .25), (None, .25)], config=config)
        moving = [msg for msg in received["/hih1/uav/nav"] if msg.target_vel_x]
        self.assertTrue(moving)
        self.assertTrue(all(msg.control_frame == FlightNav.WORLD_FRAME for msg in moving))
        self.assertTrue(all(not any(getattr(msg, f) for f in key.VELOCITY_FIELDS)
                            for msg in received["/hih2/uav/nav"]))
        self.assertEqual(received["/hih1/uav/nav"][-1].control_frame, FlightNav.LOCAL_FRAME)

    def test_follower_input_does_not_cancel_leader_trajectory(self):
        received, _ = self.run_keys([("1", .05), ("w", .25), (None, .1)])
        self.assertTrue(any(msg.target_vel_x > 0 for msg in received["/hih1/uav/nav"]))
        self.assertTrue(any(msg.target_vel_x == .1 for msg in received["/hih2/uav/nav"]))
        self.assertNotIn("cancelled", self.output)

    def test_space_and_flight_commands_cancel_trajectory(self):
        for character in " vbnm/":
            with self.subTest(character=character):
                received, _ = self.run_keys([("1", .05), (None, .25), (character, .1), (None, .2)])
                messages = received["/hih1/uav/nav"]
                self.assertTrue(any(msg.target_vel_x > 0 for msg in messages))
                self.assertTrue(all(not any(getattr(msg, f) for f in key.VELOCITY_FIELDS)
                                    for msg, stamp in zip(messages, self.received_times["/hih1/uav/nav"]) if stamp >= 10.4 - 1e-8))
                self.assertIn("cancelled", self.output)

    def test_manual_leader_input_cancels_trajectory(self):
        received, _ = self.run_keys([("1", .05), (None, .25), ("o", .1), (None, .1)])
        messages = received["/hih1/uav/nav"]
        self.assertTrue(any(msg.target_omega_z == -.1 and msg.target_vel_x == 0 for msg in messages))
        self.assertIn("cancelled", self.output)

    def test_trajectory_finishes_without_repetition(self):
        config = __import__("copy").deepcopy(self.config)
        config["trajectories"]["1"]["period"] = 4.0
        received, _ = self.run_keys([("1", .05)] + [(None, .05)] * 100, config=config)
        messages = received["/hih1/uav/nav"]
        self.assertTrue(any(msg.target_vel_x < 0 for msg in messages))
        self.assertTrue(any(msg.target_vel_z < 0 for msg in messages))
        self.assertIn("finished", self.output)
        for msg, stamp in zip(messages, self.received_times["/hih1/uav/nav"]):
            if stamp >= 14.1:
                self.assertFalse(any(getattr(msg, f) for f in key.VELOCITY_FIELDS))
        self.assertEqual(received["/hih2/teleop_command/start"], [])

    def test_held_trigger_does_not_restart_trajectory(self):
        config = __import__("copy").deepcopy(self.config)
        config["trajectories"]["1"]["period"] = 4.0
        received, _ = self.run_keys([("1", .1)] * 60, config=config)
        self.assertEqual(self.output.count("started"), 1)
        moving_after_end = [msg for msg, stamp in zip(received["/hih1/uav/nav"], self.received_times["/hih1/uav/nav"])
                            if stamp > 14.2 and any(getattr(msg, f) for f in key.VELOCITY_FIELDS)]
        self.assertEqual(moving_after_end, [])
        self.assertIn("finished", self.output)

    def test_no_leader_connection_does_not_block_follower(self):
        received, _ = self.run_keys([("1", .05), ("w", .1), (None, .1)], connected=("/hih2",))
        self.assertFalse(any(msg.target_vel_x for msg in received["/hih1/uav/nav"]))
        self.assertTrue(any(msg.target_vel_x == .1 for msg in received["/hih2/uav/nav"]))

    def test_rejects_invalid_trajectory_config_and_reserved_keys(self):
        import copy
        for change in ("zero_period", "nan_displacement", "bad_yaw", "too_fast", "reserved_key"):
            config = yaml.safe_load((ROOT / "config/hih_key.yaml").read_text())
            if change == "zero_period": config["trajectories"]["1"]["period"] = 0
            if change == "nan_displacement": config["trajectories"]["1"]["displacement"] = float("nan")
            if change == "bad_yaw": config["trajectories"]["2"]["yaw_displacement_deg"] = -1
            if change == "too_fast": config["trajectories"]["1"]["period"] = .01
            if change == "reserved_key": config["keys"]["leader"]["+X"] = "1"
            with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as stream:
                yaml.safe_dump(config, stream); stream.flush()
                with self.subTest(change=change), self.assertRaises(ValueError): key.load_config(stream.name)

    def test_g_toggles_continuous_local_approach_for_both(self):
        received, _ = self.run_keys([("g", .05)] + [(None, .2)] * 5 + [("g", .1), (None, .2)])
        for ns in ("/hih1", "/hih2"):
            topic = ns + "/uav/nav"
            moving = [msg for msg in received[topic] if msg.target_vel_x]
            self.assertTrue(moving)
            self.assertTrue(all(msg.target_vel_x == .02 and msg.control_frame == FlightNav.LOCAL_FRAME
                                and msg.target_vel_y == msg.target_vel_z == msg.target_omega_z == 0 for msg in moving))
            self.assertTrue(any(stamp > 10.8 for msg, stamp in zip(received[topic], self.received_times[topic])
                                if msg.target_vel_x))
            self.assertTrue(all(not any(getattr(msg, f) for f in key.VELOCITY_FIELDS)
                                for msg, stamp in zip(received[topic], self.received_times[topic]) if stamp >= 11.15 - 1e-8))
        self.assertEqual(self.output.count("Approach ON"), 1)
        self.assertEqual(self.output.count("Approach OFF"), 1)
        self.assertFalse(any("approach" in topic for topic in received))

    def test_approach_uses_yaml_speed_and_stops_on_exit(self):
        config = __import__("copy").deepcopy(self.config)
        config["approach_speed"] = .015
        received, _ = self.run_keys([("g", .05), (None, .2)], config=config)
        for ns in ("/hih1", "/hih2"):
            messages = received[ns + "/uav/nav"]
            self.assertTrue(any(msg.target_vel_x == .015 for msg in messages))
            self.assertFalse(any(getattr(messages[-1], f) for f in key.VELOCITY_FIELDS))

    def test_space_and_common_flight_commands_cancel_approach(self):
        for character in " vbnm/":
            with self.subTest(character=character):
                received, _ = self.run_keys([("g", .05), (None, .2), (character, .1), (None, .2)])
                for ns in ("/hih1", "/hih2"):
                    topic = ns + "/uav/nav"
                    self.assertTrue(any(msg.target_vel_x == .02 for msg in received[topic]))
                    self.assertTrue(all(not any(getattr(msg, f) for f in key.VELOCITY_FIELDS)
                                        for msg, stamp in zip(received[topic], self.received_times[topic]) if stamp >= 10.35 - 1e-8))

    def test_manual_input_cancels_approach_before_applying_the_key(self):
        received, _ = self.run_keys([("g", .05), (None, .2), ("r", .1), (None, .2)])
        self.assertTrue(any(msg.target_vel_z == .1 and msg.target_vel_x == 0 for msg in received["/hih2/uav/nav"]))
        self.assertTrue(all(msg.target_vel_x == 0 for msg, stamp in zip(received["/hih1/uav/nav"],
                             self.received_times["/hih1/uav/nav"]) if stamp >= 10.35 - 1e-8))

    def test_approach_and_trajectory_replace_each_other(self):
        received, _ = self.run_keys([("1", .05), (None, .2), ("g", .1), (None, .2)])
        self.assertTrue(any(msg.target_vel_x == .02 and msg.control_frame == FlightNav.LOCAL_FRAME
                            for msg in received["/hih1/uav/nav"]))
        received, _ = self.run_keys([("g", .05), (None, .2), ("1", .1), (None, .2)])
        self.assertTrue(any(msg.target_vel_x > 0 and msg.control_frame == FlightNav.WORLD_FRAME
                            for msg in received["/hih1/uav/nav"]))
        self.assertTrue(all(msg.target_vel_x == 0 for msg, stamp in zip(received["/hih2/uav/nav"],
                             self.received_times["/hih2/uav/nav"]) if stamp >= 10.35 - 1e-8))

    def test_held_g_does_not_repeatedly_toggle(self):
        received, _ = self.run_keys([("g", .1)] * 15)
        self.assertEqual(self.output.count("Approach ON"), 1)
        self.assertNotIn("Approach OFF", self.output)

    def test_approach_does_not_wait_for_second_robot(self):
        for connected in (("/hih1",), ("/hih2",)):
            received, _ = self.run_keys([("g", .05), (None, .2)], connected=connected)
            self.assertTrue(any(msg.target_vel_x == .02 for msg in received[connected[0] + "/uav/nav"]))

    def test_rejects_duplicate_mapping_and_bad_speed(self):
        for change in ("duplicate", "speed", "namespace", "approach_speed"):
            data = yaml.safe_load((ROOT / "config/hih_key.yaml").read_text())
            if change == "duplicate": data["keys"]["leader"]["+X"] = "v"
            if change == "speed": data["robots"]["leader"]["xy_speed"] = float("nan")
            if change == "namespace": data["robots"]["leader"]["namespace"] = "/hih2"
            if change == "approach_speed": data["approach_speed"] = float("nan")
            with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as stream:
                yaml.safe_dump(data, stream); stream.flush()
                with self.subTest(change=change), self.assertRaises(ValueError): key.load_config(stream.name)


if __name__ == "__main__": unittest.main()
