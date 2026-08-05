#!/usr/bin/env python
"""Shared helpers for beetle demo scripts.

Self-contained (depends only on the standard ``math`` module) so it can be
imported by the demo state machines regardless of their ROS setup, and so it can
be unit-tested standalone. Consolidates small control/math helpers shared across
the manipulation demos.
"""

import math
from collections import deque


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_angle_diff(angle_diff):
    """Wrap an angle difference to [-pi, pi]."""
    return math.atan2(math.sin(angle_diff), math.cos(angle_diff))


def smooth_start_angular_motion(elapsed, target_speed, ramp_time):
    """Return angular progress and speed for a smoothstep start ramp."""
    elapsed = max(0.0, float(elapsed))
    target_speed = float(target_speed)
    ramp_time = float(ramp_time)
    if ramp_time <= 0.0:
        return target_speed * elapsed, target_speed

    ratio = min(1.0, elapsed / ramp_time)
    speed = target_speed * ratio * ratio * (3.0 - 2.0 * ratio)
    if ratio < 1.0:
        progress = target_speed * ramp_time * (
            ratio ** 3 - 0.5 * ratio ** 4)
    else:
        progress = target_speed * (elapsed - 0.5 * ramp_time)
    return progress, speed


class DirectedAngularStallDetector(object):
    """Detect sustained directed command lead while measured yaw is stalled."""

    def __init__(self, direction, min_search_angle, lead_threshold,
                 stall_rate, velocity_window, required_cycles):
        self.direction = int(direction)
        self.min_search_angle = float(min_search_angle)
        self.lead_threshold = float(lead_threshold)
        self.stall_rate = float(stall_rate)
        self.velocity_window = float(velocity_window)
        self.required_cycles = int(required_cycles)
        self.reset()

    def reset(self, command_yaw=None, actual_yaw=None, stamp=None):
        self.last_command = command_yaw
        self.last_actual = actual_yaw
        self.command_progress = 0.0
        self.actual_progress = 0.0
        self.history = deque()
        self.candidate_cycles = 0
        if actual_yaw is not None and stamp is not None:
            self.history.append((float(stamp), 0.0))

    def update(self, command_yaw, actual_yaw, stamp):
        if self.last_command is not None:
            self.command_progress += self.direction * normalize_angle(
                command_yaw - self.last_command)
        if self.last_actual is not None:
            self.actual_progress += self.direction * normalize_angle(
                actual_yaw - self.last_actual)
        self.last_command = float(command_yaw)
        self.last_actual = float(actual_yaw)

        stamp = float(stamp)
        self.history.append((stamp, self.actual_progress))
        while (len(self.history) > 2 and
               stamp - self.history[1][0] >= self.velocity_window):
            self.history.popleft()
        window_ready = (
            len(self.history) >= 2 and
            stamp - self.history[0][0] >= 0.8 * self.velocity_window)
        actual_rate = float("inf")
        if window_ready:
            dt = stamp - self.history[0][0]
            actual_rate = abs(
                (self.actual_progress - self.history[0][1]) / dt)

        lead = self.command_progress - self.actual_progress
        candidate = (
            window_ready and
            self.command_progress >= self.min_search_angle and
            lead >= self.lead_threshold and
            actual_rate <= self.stall_rate)
        self.candidate_cycles = self.candidate_cycles + 1 if candidate else 0
        return {
            "contact": self.candidate_cycles >= self.required_cycles,
            "candidate_cycles": self.candidate_cycles,
            "command_progress": self.command_progress,
            "actual_progress": self.actual_progress,
            "directed_lead": lead,
            "actual_rate": actual_rate,
        }


def velocity_error_adjustment(target_velocity, measured_velocity, gain, dt,
                              deadband=0.0):
    """Return the incremental wrench change driven by velocity error.

    This is the discrete Dragon valve adaptation law with a continuous
    dead-zone around zero error:
    ``delta_u = gain * DB(target_velocity - measured_velocity) * dt``.
    The caller owns the wrench state and its task-specific saturation limits.
    """
    values = (target_velocity, measured_velocity, gain, dt, deadband)
    if not all(math.isfinite(float(value)) for value in values):
        return 0.0

    dt = max(0.0, float(dt))
    error = float(target_velocity) - float(measured_velocity)
    deadband = max(0.0, float(deadband))
    if abs(error) <= deadband:
        return 0.0
    # Subtract the deadband magnitude so the adjustment is continuous at the
    # boundary instead of reintroducing a relay-sized step there.
    error = math.copysign(abs(error) - deadband, error)
    return float(gain) * error * dt


def low_pass_update(previous, sample, dt, time_constant):
    """Update a first-order low-pass filter without ROS dependencies.

    ``time_constant <= 0`` disables filtering. Invalid samples are ignored so
    a transient mocap/odometry fault cannot inject a non-finite wrench command.
    """
    values = (previous, sample, dt, time_constant)
    if not all(math.isfinite(float(value)) for value in values):
        return float(previous) if math.isfinite(float(previous)) else 0.0

    previous = float(previous)
    sample = float(sample)
    dt = max(0.0, float(dt))
    time_constant = max(0.0, float(time_constant))
    if time_constant <= 0.0:
        return sample
    if dt <= 0.0:
        return previous
    alpha = dt / (time_constant + dt)
    return previous + alpha * (sample - previous)
