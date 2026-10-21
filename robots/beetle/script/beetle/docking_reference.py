"""Frozen assembly geometry, attitude slew and bounded relative Y/Z trim.

No ROS transport or state-machine dependencies. Mating frames have parallel axes.
"""
from dataclasses import dataclass
import math
import numpy as np
from tf import transformations as tr


@dataclass
class ReferenceConfig:
    attitude_rate: float = math.radians(10)
    trim_time_constant: float = 0.5
    trim_gain: float = 0.3
    trim_speed: float = 0.01
    trim_limit: float = 0.05
    leader_drift_limit: float = 0.05

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('%s must be finite and positive' % name)


def limit_norm(vector, limit):
    norm = np.linalg.norm(vector)
    return vector * min(1.0, limit/max(norm, 1e-12))


class FixedDockingReference:
    def __init__(self, config, leader_connector, follower_connector, direction):
        self.config = config
        self.connectors = (leader_connector, follower_connector)
        self.direction = direction
        self.frame = None
        self.leader_anchor = None
        self.orientations = None
        self.trim = np.zeros(2)
        self.filtered_error = None
        self.suspended = False

    def capture(self, roots, baselinks):
        """Capture once in HOVER; preserve the leader's FC position target.

        Desired interface attitude is level with the initial leader interface yaw.
        Account for the root/FC offset when levelling the leader.
        """
        if self.frame is not None:
            return
        interfaces = [root @ connector for root, connector in zip(roots, self.connectors)]
        yaw = tr.euler_from_matrix(interfaces[0])[2]
        self.frame = tr.euler_matrix(0, 0, yaw)
        self.leader_anchor = (roots[0] @ baselinks[0])[:3, 3].copy()
        desired_root_rotation = self.frame[:3, :3] @ self.connectors[0][:3, :3].T
        self.frame[:3, 3] = self.leader_anchor + desired_root_rotation @ (
            self.connectors[0][:3, 3]-baselinks[0][:3, 3])
        self.orientations = [tr.quaternion_from_matrix(pose) for pose in interfaces]

    def leader_displacement(self, root, baselink):
        return float(np.linalg.norm((root @ baselink)[:3, 3]-self.leader_anchor))

    def feedback(self, roots):
        interfaces = [root @ connector for root, connector in zip(roots, self.connectors)]
        if self.suspended:
            # Resume attitude slew from actual poses without recapturing the goal.
            self.orientations = [tr.quaternion_from_matrix(pose) for pose in interfaces]
            self.suspended = False
        rotation = self.frame[:3, :3]
        relative = rotation.T @ (interfaces[1][:3, 3]-interfaces[0][:3, 3])
        trajectory = rotation.T @ (interfaces[1][:3, 3]-self.frame[:3, 3])
        relative[0] *= self.direction
        trajectory[0] *= self.direction
        # Both mutual alignment AND agreement with the fixed insertion axis matter.
        errors = [tr.euler_from_matrix(np.linalg.inv(interfaces[0]) @ interfaces[1])]
        errors.extend(tr.euler_from_matrix(np.linalg.inv(self.frame) @ pose) for pose in interfaces)
        angles = np.max(np.abs(errors), axis=0)
        return relative, trajectory, angles

    def update(self, dt, relative_position, trim_enabled):
        if dt <= 0:
            return
        target = tr.quaternion_from_matrix(self.frame)
        for i, current in enumerate(self.orientations):
            angle = 2*math.acos(np.clip(abs(np.dot(current, target)), 0, 1))
            fraction = min(1.0, self.config.attitude_rate*dt/max(angle, 1e-12))
            self.orientations[i] = tr.quaternion_slerp(current, target, fraction)
        if not trim_enabled:
            self.filtered_error = None
            return
        error = np.asarray(relative_position)[1:]
        if self.filtered_error is None:
            self.filtered_error = np.zeros(2)
        alpha = 1-math.exp(-dt/self.config.trim_time_constant)
        self.filtered_error += alpha*(error-self.filtered_error)
        velocity = limit_norm(-self.config.trim_gain*self.filtered_error, self.config.trim_speed)
        self.trim = limit_norm(self.trim+velocity*dt, self.config.trim_limit)

    def suspend(self):
        """Drop filter history on TF interruption; never move the frozen frame."""
        self.filtered_error = None
        self.suspended = True

    def follower_pose(self, command, root_baselink):
        coordinates = np.asarray(command, dtype=float).copy()
        coordinates[0] *= self.direction
        coordinates[1:] += self.trim
        interface = tr.quaternion_matrix(self.orientations[1])
        interface[:3, 3] = self.frame[:3, 3]+self.frame[:3, :3] @ coordinates
        return interface @ np.linalg.inv(self.connectors[1]) @ root_baselink

    def leader_attitude(self, root_baselink):
        return tr.quaternion_matrix(self.orientations[0]) @ np.linalg.inv(self.connectors[0]) @ root_baselink

    def axial_feedforward(self, speed):
        return self.frame[:3, 0]*self.direction*speed
