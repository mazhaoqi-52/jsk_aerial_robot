#!/usr/bin/env python3
"""ROS-independent helpers for the downward-valve static torque test."""

import math
from collections import deque


def normalize_angle(angle):
    """Return *angle* wrapped to [-pi, pi]."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def parse_rotation_direction(value, view="above"):
    """Return the signed world-Z yaw direction and a human-readable label.

    ``view`` removes the downward-valve visual ambiguity. ``above`` means +Z
    looking down and matches the existing valve demo. ``below`` means the
    operator/tool side looking upward, for which visual CW/CCW is reversed.
    """
    key = str(value).strip().lower()
    if key in ("clockwise", "cw", "right", "\u53f3\u8f6c", "\u987a\u65f6\u9488"):
        visual_sign = -1
        visual_label = "CW"
    elif key in (
            "counterclockwise", "counter-clockwise", "ccw", "left",
            "\u5de6\u8f6c", "\u9006\u65f6\u9488"):
        visual_sign = 1
        visual_label = "CCW"
    else:
        raise ValueError(
            "rotation_direction must be cw/clockwise or "
            "ccw/counterclockwise")

    view_key = str(view).strip().lower()
    if view_key in ("above", "+z", "top"):
        direction = visual_sign
        view_label = "viewed from above (+Z looking down)"
    elif view_key in ("below", "-z", "bottom", "tool"):
        direction = -visual_sign
        view_label = "viewed from below/tool side (-Z looking up)"
    else:
        raise ValueError("rotation_view must be above or below")
    world_label = "positive" if direction > 0 else "negative"
    return direction, "%s %s; %s world-Z yaw" % (
        visual_label, view_label, world_label)


def finite_vector(value, length, name):
    """Convert an iterable to a finite float list with an exact length."""
    try:
        result = [float(component) for component in value]
    except (TypeError, ValueError):
        raise ValueError("%s must be a numeric sequence" % name)
    if len(result) != int(length):
        raise ValueError("%s must contain exactly %d values" % (name, length))
    if not all(math.isfinite(component) for component in result):
        raise ValueError("%s must contain only finite values" % name)
    return result


def parse_unified_heartbeat_id(value):
    """Extract the module id from the controller's KeyValue payload."""
    for field in str(value).split():
        if field.startswith("id="):
            try:
                return int(field.split("=", 1)[1])
            except ValueError:
                return None
    return None


class DualPinToolGeometry(object):
    """Symmetric dual-pin geometry expressed in the carrier UAV body frame."""

    def __init__(self, center_x, half_spacing_y, center_z, center_y=0.0):
        values = finite_vector(
            [center_x, center_y, center_z, half_spacing_y], 4,
            "dual-pin geometry")
        self.center_x = values[0]
        self.center_y = values[1]
        self.center_z = values[2]
        self.half_spacing_y = values[3]
        if self.half_spacing_y <= 0.0:
            raise ValueError("tool_half_spacing_y must be positive")

    @property
    def center(self):
        return [self.center_x, self.center_y, self.center_z]

    @property
    def pin_centers(self):
        return [
            [self.center_x,
             self.center_y + self.half_spacing_y,
             self.center_z],
            [self.center_x,
             self.center_y - self.half_spacing_y,
             self.center_z],
        ]

    @property
    def separation(self):
        return 2.0 * self.half_spacing_y


class DirectedAngularStallDetector(object):
    """Detect bounded rotational contact from command lead and yaw stall.

    A contact candidate requires all of the following:

    * the command has moved through ``min_search_angle`` in ``direction``;
    * command yaw leads measured yaw by at least ``lead_threshold``;
    * measured yaw speed over ``velocity_window`` is below ``stall_rate``;
    * the candidate persists for ``required_cycles`` updates.

    Angles are unwrapped incrementally, so crossing +/-pi does not reset the
    search or invert clockwise/counter-clockwise progress.
    """

    def __init__(self, direction, min_search_angle, lead_threshold,
                 stall_rate, velocity_window, required_cycles):
        if direction not in (-1, 1):
            raise ValueError("direction must be -1 or +1")
        self.direction = int(direction)
        self.min_search_angle = max(0.0, float(min_search_angle))
        self.lead_threshold = max(0.0, float(lead_threshold))
        self.stall_rate = max(0.0, float(stall_rate))
        self.velocity_window = max(1e-3, float(velocity_window))
        self.required_cycles = max(1, int(required_cycles))
        self.reset()

    def reset(self, command_yaw=None, actual_yaw=None, stamp=None):
        self._last_command = (
            None if command_yaw is None else float(command_yaw))
        self._last_actual = None if actual_yaw is None else float(actual_yaw)
        self._command_progress = 0.0
        self._actual_progress = 0.0
        self._history = deque()
        self._candidate_cycles = 0
        if actual_yaw is not None and stamp is not None:
            self._history.append((float(stamp), self._actual_progress))

    def update(self, command_yaw, actual_yaw, stamp):
        command_yaw = float(command_yaw)
        actual_yaw = float(actual_yaw)
        stamp = float(stamp)

        if self._last_command is None:
            self._last_command = command_yaw
        else:
            delta = normalize_angle(command_yaw - self._last_command)
            self._command_progress += self.direction * delta
            self._last_command = command_yaw

        if self._last_actual is None:
            self._last_actual = actual_yaw
        else:
            delta = normalize_angle(actual_yaw - self._last_actual)
            self._actual_progress += self.direction * delta
            self._last_actual = actual_yaw

        self._history.append((stamp, self._actual_progress))
        while (len(self._history) > 2 and
               stamp - self._history[1][0] >= self.velocity_window):
            self._history.popleft()

        window_ready = (
            len(self._history) >= 2 and
            stamp - self._history[0][0] >= 0.8 * self.velocity_window)
        actual_rate = float("inf")
        if window_ready:
            dt = stamp - self._history[0][0]
            actual_rate = abs(
                (self._actual_progress - self._history[0][1]) / dt)

        directed_lead = self._command_progress - self._actual_progress
        candidate = (
            window_ready and
            self._command_progress >= self.min_search_angle and
            directed_lead >= self.lead_threshold and
            actual_rate <= self.stall_rate)
        if candidate:
            self._candidate_cycles += 1
        else:
            self._candidate_cycles = 0

        return {
            "contact": self._candidate_cycles >= self.required_cycles,
            "candidate": candidate,
            "candidate_cycles": self._candidate_cycles,
            "command_progress": self._command_progress,
            "actual_progress": self._actual_progress,
            "directed_lead": directed_lead,
            "actual_rate": actual_rate,
            "window_ready": window_ready,
        }


def slew_toward(current, target, max_rate, dt):
    """Move scalar ``current`` toward ``target`` without exceeding a rate."""
    current = float(current)
    target = float(target)
    max_step = max(0.0, float(max_rate)) * max(0.0, float(dt))
    delta = target - current
    if abs(delta) <= max_step:
        return target
    if max_step <= 0.0:
        return current
    return current + math.copysign(max_step, delta)


def minimum_unload_duration(force, torque, minimum, force_rate, torque_rate):
    """Compute a 6D unload duration respecting force and torque slew rates."""
    force = finite_vector(force, 3, "force")
    torque = finite_vector(torque, 3, "torque")
    force_norm = math.sqrt(sum(component * component for component in force))
    torque_norm = math.sqrt(sum(component * component for component in torque))
    duration = max(0.0, float(minimum))
    if force_norm > 0.0:
        if force_rate <= 0.0:
            raise ValueError("force_rate must be positive for nonzero force")
        duration = max(duration, force_norm / float(force_rate))
    if torque_norm > 0.0:
        if torque_rate <= 0.0:
            raise ValueError("torque_rate must be positive for nonzero torque")
        duration = max(duration, torque_norm / float(torque_rate))
    return duration
