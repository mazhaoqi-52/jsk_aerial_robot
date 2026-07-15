#!/usr/bin/env python
"""Shared helpers for beetle demo scripts.

Self-contained (depends only on the standard ``math`` module) so it can be
imported by the demo state machines regardless of their ROS setup, and so it can
be unit-tested standalone. Consolidates small control/math helpers shared across
the manipulation demos.
"""

import math


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_angle_diff(angle_diff):
    """Wrap an angle difference to [-pi, pi]."""
    return math.atan2(math.sin(angle_diff), math.cos(angle_diff))


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
