#!/usr/bin/env python3
"""HiH keyboard commander: Follower on the left, Leader on the right."""
import argparse
import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import time
import tty

import rospkg
import rospy
import yaml
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Empty

ROBOTS = ("follower", "leader")
VELOCITY_FIELDS = ("target_vel_x", "target_vel_y", "target_vel_z", "target_omega_z")
COMMANDS = {"arm": "start", "takeoff": "takeoff", "land": "land",
            "force_landing": "force_landing", "halt": "halt"}
MOVEMENTS = ("+X", "-X", "+Y", "-Y",
             "+Z", "-Z", "+yaw", "-yaw")


class MultiPublisher:
    """Send common commands to each robot without waiting for both robots."""
    def __init__(self, topics, dry_run=False):
        self.topics = topics
        self.dry_run = dry_run
        self.publishers = [] if dry_run else [
            rospy.Publisher(topic, Empty, queue_size=1, latch=False) for topic in topics]

    def publish(self, msg):
        for index, topic in enumerate(self.topics):
            if self.dry_run:
                print(f"[DRY RUN] {topic}", flush=True)
            else:
                pub = self.publishers[index]
                if pub.get_num_connections() == 0:
                    rospy.logwarn("No subscriber for %s; command is not retried", topic)
                pub.publish(msg)
                rospy.loginfo("Command -> %s", topic)


def load_config(path):
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if config["frame"] not in ("world", "local"):
        raise ValueError("frame must be world or local")
    for name in ("publish_rate", "key_timeout", "command_repeat_gap"):
        if not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(f"{name} must be positive and finite")
    if not 1 <= config["publish_rate"] <= 100 or config["key_timeout"] < 1 / config["publish_rate"]:
        raise ValueError("Check publish_rate (1..100 Hz) and key_timeout")
    if type(config["stop_repeats"]) is not int or not 1 <= config["stop_repeats"] <= 10:
        raise ValueError("stop_repeats must be 1..10")
    if not config["common_targets"] or any(r not in ROBOTS for r in config["common_targets"]):
        raise ValueError("common_targets must contain leader and/or follower")
    if set(config["keys"]["common"]) != set(COMMANDS) | {"help", "stop_motion"}:
        raise ValueError("Check common command names in keys/common")
    if any(set(config["keys"][role]) != set(MOVEMENTS) for role in ROBOTS):
        raise ValueError("Check movement names in keys/leader and keys/follower")
    all_keys = []
    for group in ("common",) + ROBOTS:
        for action, key in config["keys"][group].items():
            key = " " if key == "SPACE" else key
            if not isinstance(key, str) or len(key) != 1 or not key.isprintable():
                raise ValueError(f"Invalid key: {group}/{action}")
            config["keys"][group][action] = key
            all_keys.append(key)
    if len(all_keys) != len(set(all_keys)):
        raise ValueError("Duplicate keyboard mapping")
    for role in ROBOTS:
        robot = config["robots"][role]
        robot["namespace"] = "/" + robot["namespace"].strip("/")
        if robot["namespace"] == "/":
            raise ValueError("Robot namespace must not be empty")
        for name in ("xy_speed", "z_speed", "yaw_speed"):
            if not math.isfinite(robot[name]) or robot[name] <= 0:
                raise ValueError(f"Invalid {role}/{name}")
    if config["robots"]["leader"]["namespace"] == config["robots"]["follower"]["namespace"]:
        raise ValueError("Leader and Follower namespaces must differ")
    return config


def print_help(config):
    keys = config["keys"]
    print("\nHiH keyboard — " + config["frame"].upper() + " frame")
    print(" | ".join(f"{keys['common'][action]}: {action}" for action in COMMANDS))
    print("                 Follower    Leader")
    for action in MOVEMENTS:
        print(f"{action:16} {keys['follower'][action]:8}    {keys['leader'][action]}")
    print(f"{keys['common']['stop_motion']!r}: zero keyboard motion | {keys['common']['help']}: help")
    print("Please disable Caps Lock. Ctrl+C: zero keyboard velocities and quit.\n", flush=True)


def get_key(fd, timeout, input_state):
    """Read one key; skip arrow/escape sequences and bracketed pasted text."""
    if not select.select([fd], [], [], timeout)[0]:
        return None
    raw = os.read(fd, 1)
    if not raw:
        raise EOFError
    key = raw.decode("utf-8", errors="ignore")
    if input_state["escape"] or key == "\x1b":
        sequence = input_state["escape"] + key
        if sequence not in ("\x1b", "\x1b[", "\x1bO"):
            if len(sequence) >= 3 and "@" <= key <= "~":
                if sequence in ("\x1b[200~", "\x1b[201~"):
                    input_state["paste"] = sequence == "\x1b[200~"
                sequence = ""
            elif (len(sequence) == 2 and sequence[1] not in "[O") or len(sequence) > 64:
                sequence = ""
        input_state["escape"] = sequence
        return None
    return None if input_state["paste"] else key


def stop_motion(roles, motion, expires, stops, config):
    for role in roles:
        for field in VELOCITY_FIELDS:
            setattr(motion[role], field, 0.0)
        expires[role] = [0.0] * 4
        stops[role] = config["stop_repeats"]


def make_nav(velocity, config):
    msg = FlightNav()
    msg.control_frame = FlightNav.WORLD_FRAME if config["frame"] == "world" else FlightNav.LOCAL_FRAME
    msg.target = FlightNav.COG
    msg.pos_xy_nav_mode = msg.pos_z_nav_mode = msg.yaw_nav_mode = FlightNav.VEL_MODE
    msg.target_vel_x, msg.target_vel_y, msg.target_vel_z, msg.target_omega_z = velocity
    return msg


def send_motion(config, nav_pubs, motion, expires, stops, now, dry_run, last_printed):
    for role in ROBOTS:
        msg = motion[role]
        was_moving = any(getattr(msg, field) for field in VELOCITY_FIELDS)
        for field, deadline in zip(VELOCITY_FIELDS, expires[role]):
            if now >= deadline:
                setattr(msg, field, 0.0)
        velocity = tuple(getattr(msg, field) for field in VELOCITY_FIELDS)
        moving = any(velocity)
        if was_moving and not moving:
            stops[role] = config["stop_repeats"]
        if not moving and stops[role] == 0:
            continue
        if dry_run:
            if last_printed.get(role) != velocity:
                print(f"[DRY RUN] {config['robots'][role]['namespace']}/uav/nav: {velocity}", flush=True)
            last_printed[role] = velocity
        else:
            msg.header.stamp = rospy.Time.now()
            nav_pubs[role].publish(msg)
        stops[role] = 0 if moving else stops[role] - 1


def quit_keyboard(_signal, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--dry-run", action="store_true", help="Print only; no ROS node or publishers")
    args = parser.parse_args(rospy.myargv()[1:])
    if not sys.stdin.isatty():
        parser.error("Run this keyboard node in an interactive terminal")
    path = args.config or Path(rospkg.RosPack().get_path("hih")) / "config/hih_key.yaml"
    try:
        config = load_config(path)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        parser.error(str(error))

    # Common commands go to both robots; movement publishers stay separate.
    if not args.dry_run:
        rospy.init_node("hih_key", disable_signals=True)
    namespaces = [config["robots"][role]["namespace"] for role in config["common_targets"]]
    start_pub = MultiPublisher([ns + "/teleop_command/start" for ns in namespaces], args.dry_run)
    takeoff_pub = MultiPublisher([ns + "/teleop_command/takeoff" for ns in namespaces], args.dry_run)
    land_pub = MultiPublisher([ns + "/teleop_command/land" for ns in namespaces], args.dry_run)
    force_landing_pub = MultiPublisher([ns + "/teleop_command/force_landing" for ns in namespaces], args.dry_run)
    halt_pub = MultiPublisher([ns + "/teleop_command/halt" for ns in namespaces], args.dry_run)
    follower_ns = config["robots"]["follower"]["namespace"]
    leader_ns = config["robots"]["leader"]["namespace"]
    follower_nav_pub = None if args.dry_run else rospy.Publisher(follower_ns + "/uav/nav", FlightNav, queue_size=1)
    leader_nav_pub = None if args.dry_run else rospy.Publisher(leader_ns + "/uav/nav", FlightNav, queue_size=1)
    nav_pubs = {"follower": follower_nav_pub, "leader": leader_nav_pub}
    common = config["keys"]["common"]
    follower = config["keys"]["follower"]
    leader = config["keys"]["leader"]
    follower_xy_vel = config["robots"]["follower"]["xy_speed"]
    follower_z_vel = config["robots"]["follower"]["z_speed"]
    follower_yaw_vel = config["robots"]["follower"]["yaw_speed"]
    leader_xy_vel = config["robots"]["leader"]["xy_speed"]
    leader_z_vel = config["robots"]["leader"]["z_speed"]
    leader_yaw_vel = config["robots"]["leader"]["yaw_speed"]

    follower_nav = make_nav((0.0, 0.0, 0.0, 0.0), config)
    leader_nav = make_nav((0.0, 0.0, 0.0, 0.0), config)
    motion = {"follower": follower_nav, "leader": leader_nav}
    expires = {role: [0.0] * 4 for role in ROBOTS}
    stops = dict.fromkeys(ROBOTS, 0)
    last_command, last_printed = {}, {}
    input_state = {"escape": "", "paste": False}
    fd = sys.stdin.fileno()
    terminal_settings = termios.tcgetattr(fd)
    previous_sigterm = signal.signal(signal.SIGTERM, quit_keyboard)
    print_help(config)

    try:
        tty.setcbreak(fd)
        print("\x1b[?2004h", end="", flush=True)
        next_tick = time.monotonic()
        while args.dry_run or not rospy.is_shutdown():
            key = get_key(fd, max(0.0, min(0.05, next_tick - time.monotonic())), input_state)
            now = time.monotonic()
            if key == "\x03":
                break
            # Flight commands: same direct key -> publisher flow as Beetle.
            if key == common["help"]:
                print_help(config)
            if key == common["stop_motion"]:
                stop_motion(ROBOTS, motion, expires, stops, config)
            if key in (common["arm"], common["takeoff"], common["land"],
                       common["force_landing"], common["halt"]):
                gap = now - last_command.get(key, -math.inf)
                last_command[key] = now
                if gap >= config["command_repeat_gap"]:
                    stop_motion(config["common_targets"], motion, expires, stops, config)
                    if key == common["arm"]:
                        start_pub.publish(Empty())
                    if key == common["takeoff"]:
                        takeoff_pub.publish(Empty())
                    if key == common["land"]:
                        land_pub.publish(Empty())
                    if key == common["force_landing"]:
                        force_landing_pub.publish(Empty())
                    if key == common["halt"]:
                        halt_pub.publish(Empty())

            # Follower: w/s = X, a/d = Y, r/f = Z, q/e = yaw.
            deadline = now + config["key_timeout"]
            if key == follower["+X"]:
                follower_nav.target_vel_x = follower_xy_vel
                expires["follower"][0] = deadline
            if key == follower["-X"]:
                follower_nav.target_vel_x = -follower_xy_vel
                expires["follower"][0] = deadline
            if key == follower["+Y"]:
                follower_nav.target_vel_y = follower_xy_vel
                expires["follower"][1] = deadline
            if key == follower["-Y"]:
                follower_nav.target_vel_y = -follower_xy_vel
                expires["follower"][1] = deadline
            if key == follower["+Z"]:
                follower_nav.target_vel_z = follower_z_vel
                expires["follower"][2] = deadline
            if key == follower["-Z"]:
                follower_nav.target_vel_z = -follower_z_vel
                expires["follower"][2] = deadline
            if key == follower["+yaw"]:
                follower_nav.target_omega_z = follower_yaw_vel
                expires["follower"][3] = deadline
            if key == follower["-yaw"]:
                follower_nav.target_omega_z = -follower_yaw_vel
                expires["follower"][3] = deadline

            # Leader: i/k = X, j/l = Y, p/; = Z, u/o = yaw.
            if key == leader["+X"]:
                leader_nav.target_vel_x = leader_xy_vel
                expires["leader"][0] = deadline
            if key == leader["-X"]:
                leader_nav.target_vel_x = -leader_xy_vel
                expires["leader"][0] = deadline
            if key == leader["+Y"]:
                leader_nav.target_vel_y = leader_xy_vel
                expires["leader"][1] = deadline
            if key == leader["-Y"]:
                leader_nav.target_vel_y = -leader_xy_vel
                expires["leader"][1] = deadline
            if key == leader["+Z"]:
                leader_nav.target_vel_z = leader_z_vel
                expires["leader"][2] = deadline
            if key == leader["-Z"]:
                leader_nav.target_vel_z = -leader_z_vel
                expires["leader"][2] = deadline
            if key == leader["+yaw"]:
                leader_nav.target_omega_z = leader_yaw_vel
                expires["leader"][3] = deadline
            if key == leader["-yaw"]:
                leader_nav.target_omega_z = -leader_yaw_vel
                expires["leader"][3] = deadline

            if now >= next_tick:
                send_motion(config, nav_pubs, motion, expires, stops, now, args.dry_run, last_printed)
                next_tick = now + 1.0 / config["publish_rate"]
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, terminal_settings)
        signal.signal(signal.SIGTERM, previous_sigterm)
        print("\x1b[?2004l", flush=True)
        if args.dry_run or not rospy.is_shutdown():
            stop_motion(ROBOTS, motion, expires, stops, config)
            for _ in range(config["stop_repeats"]):
                send_motion(config, nav_pubs, motion, expires, stops, time.monotonic(), args.dry_run, last_printed)
                if not args.dry_run:
                    time.sleep(0.02)
        if not args.dry_run:
            rospy.signal_shutdown("Keyboard closed")


if __name__ == "__main__":
    main()
