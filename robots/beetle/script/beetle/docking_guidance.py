"""Position trajectory and basic arrival tests for prealignment and docking."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class DockingConfig:
    prealign_gap: float = 0.15
    position_tolerance: float = 0.05
    angle_tolerance: float = math.radians(8.6)
    approach_speed: float = 0.1
    correction_speed: float = 0.05
    slowdown_gain: float = 2.0
    insertion_depth: float = 0.05
    final_x_tolerance: float = 0.02
    final_y_tolerance: float = 0.02
    final_z_tolerance: float = 0.015

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('%s must be finite and positive' % name)


def move_towards(current, target, step):
    return current + np.clip(target-current, -step, step)


class DockingGuidance:
    def __init__(self, config):
        self.config = config
        self.phase = 'prealign'
        self.command = None
        self.previous_time = None
        self.axial_velocity = 0.0

    def suspend(self):
        """Resume from the held aircraft position after unavailable TF, in the same phase."""
        self.command = None
        self.previous_time = None
        self.axial_velocity = 0.0

    def update(self, stamp, position, angles, trajectory_position):
        """Plan in fixed coordinates; relative geometry only decides stage completion.

        stamp is the local control clock, not a comparison with remote TF time.
        """
        c = self.config
        p, r, q = (np.asarray(value, dtype=float) for value in
                   (position, angles, trajectory_position))
        if not np.all(np.isfinite(np.r_[stamp, p, r, q])):
            raise ValueError('Non-finite docking feedback')
        self.axial_velocity = 0.0
        if self.phase == 'complete':
            return self.phase
        dt = 0.0 if self.previous_time is None else max(0.0, stamp-self.previous_time)
        self.previous_time = stamp
        if self.command is None:
            self.command = q.copy()
        angles_ok = np.all(np.abs((r+np.pi) % (2*np.pi)-np.pi) < c.angle_tolerance)
        if self.phase == 'prealign':
            goal = c.prealign_gap
            arrived = (abs(p[0]-c.prealign_gap) < c.position_tolerance and
                       np.all(np.abs(p[1:]) < c.position_tolerance) and angles_ok)
            if arrived:
                self.phase = 'approach'
        else:
            arrived = (-c.insertion_depth <= p[0] < c.final_x_tolerance and
                       abs(p[1]) < c.final_y_tolerance and
                       abs(p[2]) < c.final_z_tolerance and angles_ok)
            if arrived:
                self.command = q.copy()
                self.phase = 'complete'
                return self.phase
            goal = -c.insertion_depth
        # Feedback errors do not pause, reset, retreat or move the waypoint.
        # The trajectory advances toward its fixed goal at a bounded speed.
        old_x = self.command[0]
        speed = min(c.approach_speed, c.slowdown_gain*abs(goal-old_x))
        self.command[0] = move_towards(old_x, goal, speed*dt)
        self.command[1:] = move_towards(self.command[1:], np.zeros(2), c.correction_speed*dt)
        if dt > 0:
            self.axial_velocity = float((self.command[0]-old_x)/dt)
        return self.phase
