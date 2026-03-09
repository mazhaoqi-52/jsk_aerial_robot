"""
Trajectory classes for beetle unified mode performance testing.

Provides parametric trajectories (position + velocity + optional orientation)
that can be sampled at arbitrary time. Used by unified_traj_test.py to publish
FlightNav commands for the assembled formation.

Based on trajectory library by li-jinjie (24-1-5), adapted for beetle assembly.
"""
import numpy as np
from typing import Tuple
import tf_conversions as tf


class BaseTraj:
    def __init__(self, loop_num=np.inf):
        self.T = float()
        self.loop_num = loop_num

    def check_finished(self, t):
        return t > self.T * self.loop_num

    def get_3d_pt(self, t):
        """Returns (x, y, z, vx, vy, vz)."""
        return 0.0, 0.0, 0.8, 0.0, 0.0, 0.0

    def get_yaw(self, t):
        """Returns (yaw, omega_z)."""
        return 0.0, 0.0

    def get_rp(self, t):
        """Returns (roll, pitch)."""
        return 0.0, 0.0


class CircleTraj(BaseTraj):
    def __init__(self, loop_num=1, radius=0.5, period=20.0, z=0.8):
        super().__init__(loop_num)
        self.r = radius
        self.T = period
        self.z = z
        self.omega = 2 * np.pi / self.T

    def get_3d_pt(self, t):
        x = self.r * np.cos(self.omega * t)
        y = self.r * np.sin(self.omega * t)
        vx = -self.r * self.omega * np.sin(self.omega * t)
        vy = self.r * self.omega * np.cos(self.omega * t)
        return x, y, self.z, vx, vy, 0.0


class LemniscateTraj(BaseTraj):
    """3D Lemniscate (figure-8) trajectory with sinusoidal Z variation."""

    def __init__(self, loop_num=1, a=0.8, z_range=0.2, period=20.0, z_center=1.0):
        super().__init__(loop_num)
        self.a = a
        self.z_range = z_range
        self.z_center = z_center
        self.T = period
        self.omega = 2 * np.pi / self.T

    def get_3d_pt(self, t):
        t = t + self.T / 4  # phase shift to start at origin

        x = self.a * np.cos(self.omega * t)
        y = self.a * np.sin(2 * self.omega * t) / 2
        z = self.z_range * np.sin(2 * self.omega * t + np.pi / 2) + self.z_center

        vx = -self.a * self.omega * np.sin(self.omega * t)
        vy = self.a * self.omega * np.cos(2 * self.omega * t)
        vz = 2 * self.z_range * self.omega * np.cos(2 * self.omega * t + np.pi)

        return x, y, z, vx, vy, vz


class LemniscateTrajYaw(LemniscateTraj):
    """Lemniscate trajectory with simultaneous yaw rotation."""

    def get_yaw(self, t):
        t = t + self.T / 4
        yaw = np.pi / 2 * np.sin(self.omega * t + np.pi) + np.pi / 2
        yaw_rate = np.pi / 2 * self.omega * np.cos(self.omega * t + np.pi / 2)
        return yaw, yaw_rate


class LemniscateTrajOmni(LemniscateTraj):
    """Lemniscate trajectory with simultaneous roll/pitch/yaw variation (omnidirectional)."""

    def __init__(self, loop_num=1, a=0.8, z_range=0.2, period=20.0, z_center=1.0, a_orientation=0.5):
        super().__init__(loop_num, a, z_range, period, z_center)
        self.a_ori = a_orientation

    def get_yaw(self, t):
        t = t + self.T / 4
        yaw = np.pi / 2 * np.sin(self.omega * t + np.pi) + np.pi / 2
        yaw_rate = np.pi / 2 * self.omega * np.cos(self.omega * t + np.pi / 2)
        return yaw, yaw_rate

    def get_rp(self, t):
        t = t + self.T / 4
        roll = -self.a_ori * np.sin(2 * self.omega * t)
        pitch = self.a_ori * np.cos(self.omega * t)
        return roll, pitch


class SetPointTraj(BaseTraj):
    """Step-wise setpoint trajectory — tests position/yaw step response."""

    def __init__(self, loop_num=1, t_converge=8.0):
        super().__init__(loop_num)
        self.t_converge = t_converge
        self.T = 4 * self.t_converge

    def get_3d_pt(self, t):
        x, y, z = 0.0, 0.0, 0.8
        if self.t_converge < t <= 2 * self.t_converge:
            x, y, z = 0.3, 0.2, 1.2
        elif 2 * self.t_converge < t <= 3 * self.t_converge:
            x, y, z = -0.3, 0.0, 1.0
        return x, y, z, 0.0, 0.0, 0.0

    def get_yaw(self, t):
        yaw = 0.0
        if self.t_converge < t <= 2 * self.t_converge:
            yaw = 0.3
        elif 2 * self.t_converge < t <= 3 * self.t_converge:
            yaw = -0.3
        return yaw, 0.0
