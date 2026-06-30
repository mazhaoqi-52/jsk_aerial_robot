#!/usr/bin/env python
"""Shared helpers for beetle demo scripts.

Self-contained (depends only on the standard ``math`` module) so it can be
imported by the demo state machines regardless of their ROS setup, and so it can
be unit-tested standalone. Consolidates the angle-wrapping helpers that were
duplicated (with mixed while-loop / atan2 implementations) across the demos.
"""

import math


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_angle_diff(angle_diff):
    """Wrap an angle difference to [-pi, pi]."""
    return math.atan2(math.sin(angle_diff), math.cos(angle_diff))
