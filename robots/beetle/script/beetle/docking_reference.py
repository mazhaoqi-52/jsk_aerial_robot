"""Frozen assembly geometry and common attitude slew.

No ROS transport or state-machine dependencies. Mating frames have parallel axes.
"""
from dataclasses import dataclass
import math
import numpy as np
from tf import transformations as tr


@dataclass
class ReferenceConfig:
    attitude_rate: float = math.radians(10)

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('%s must be finite and positive' % name)


class FixedDockingReference:
    def __init__(self, config, leader_connector, follower_connector, direction):
        self.config = config
        self.connectors = (leader_connector, follower_connector)
        self.direction = direction
        self.frame = None
        self.leader_anchor = None
        self.orientations = None
        self.suspended = False

    def capture(self, roots, baselinks):
        """Capture fixed axes once, anchored at the measured leader FC position.

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
        angles = np.array(tr.euler_from_matrix(np.linalg.inv(interfaces[0]) @ interfaces[1]))
        return relative, trajectory, angles

    def update(self, dt):
        if dt <= 0:
            return
        target = tr.quaternion_from_matrix(self.frame)
        for i, current in enumerate(self.orientations):
            angle = 2*math.acos(np.clip(abs(np.dot(current, target)), 0, 1))
            fraction = min(1.0, self.config.attitude_rate*dt/max(angle, 1e-12))
            self.orientations[i] = tr.quaternion_slerp(current, target, fraction)

    def suspend(self):
        """Resume attitude slew from measured poses, retaining the fixed goal."""
        self.suspended = True

    def follower_pose(self, command, root_baselink):
        coordinates = np.asarray(command, dtype=float).copy()
        coordinates[0] *= self.direction
        interface = tr.quaternion_matrix(self.orientations[1])
        interface[:3, 3] = self.frame[:3, 3]+self.frame[:3, :3] @ coordinates
        return interface @ np.linalg.inv(self.connectors[1]) @ root_baselink

    def leader_attitude(self, root_baselink):
        return tr.quaternion_matrix(self.orientations[0]) @ np.linalg.inv(self.connectors[0]) @ root_baselink

    def world_velocity(self, velocity):
        coordinates = np.asarray(velocity, dtype=float).copy()
        coordinates[0] *= self.direction
        return self.frame[:3, :3] @ coordinates
