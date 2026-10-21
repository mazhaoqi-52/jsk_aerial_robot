"""Position trajectory and basic arrival tests for prealignment and docking."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class DockingConfig:
    prealign_gap: float = 0.15
    position_tolerance: float = 0.02
    prealign_angle_tolerance: float = math.radians(3.0)
    angle_tolerance: float = math.radians(8.6)
    approach_speed: float = 0.1
    near_approach_speed: float = 0.05
    approach_slowdown_distance: float = 0.15
    correction_speed: float = 0.02
    relative_correction_gain: float = 0.5
    contact_lock_gap: float = 0.05
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


class RelativePositionCorrection:
    """Turn measured interface errors into bounded waypoint velocities.

    Integrating these velocities keeps position targets continuous and also
    corrects persistent aircraft tracking offsets, without moving the frame.
    """
    def __init__(self, gain):
        self.gain = gain

    def velocity(self, error, speed_limit):
        return np.clip(-self.gain*np.asarray(error), -np.asarray(speed_limit), speed_limit)


class DockingGuidance:
    def __init__(self, config):
        self.config = config
        self.phase = 'prealign'
        self.command = None
        self.previous_time = None
        self.velocity = np.zeros(3)
        self.correction = RelativePositionCorrection(config.relative_correction_gain)
        self.approach_goal_x = None
        self.correction_locked = False

    @property
    def axial_velocity(self):
        return float(self.velocity[0])

    def suspend(self):
        """Resume from the held aircraft position after unavailable TF, in the same phase."""
        self.command = None
        self.previous_time = None
        self.velocity[:] = 0.0

    def axial_speed_limit(self, gap):
        """Reduce approach speed using measured separation, in fixed coordinates."""
        c = self.config
        if self.phase == 'prealign':
            return c.approach_speed
        # Continuous ramp: near speed inside the slowdown distance, rising
        # toward the far speed as the actual interface separation increases.
        return min(c.approach_speed,
                   c.near_approach_speed*max(1.0, gap/c.approach_slowdown_distance))

    def update(self, stamp, position, angles, trajectory_position):
        """Align relative position, then approach along a fixed insertion axis.

        stamp is the local control clock, not a comparison with remote TF time.
        """
        c = self.config
        p, r, q = (np.asarray(value, dtype=float) for value in
                   (position, angles, trajectory_position))
        if not np.all(np.isfinite(np.r_[stamp, p, r, q])):
            raise ValueError('Non-finite docking feedback')
        self.velocity[:] = 0.0
        if self.phase == 'complete':
            return self.phase
        dt = 0.0 if self.previous_time is None else max(0.0, stamp-self.previous_time)
        self.previous_time = stamp
        if self.command is None:
            self.command = q.copy()
        angle_errors = np.abs((r+np.pi) % (2*np.pi)-np.pi)
        old_command = self.command.copy()
        if self.phase == 'prealign':
            error = p-np.array([c.prealign_gap, 0, 0])
            velocity = self.correction.velocity(
                error, [c.approach_speed, c.correction_speed, c.correction_speed])
            self.command += velocity*dt
            arrived = (abs(p[0]-c.prealign_gap) < c.position_tolerance and
                       np.all(np.abs(p[1:]) < c.position_tolerance) and
                       np.all(angle_errors < c.prealign_angle_tolerance))
            if arrived:
                # Fix X once using the leader's current interface position,
                # not its potentially displaced position at initial capture.
                self.approach_goal_x = float(q[0]-p[0]-c.insertion_depth)
                self.phase = 'approach'
        else:
            aligned = (abs(p[1]) < c.final_y_tolerance and
                       abs(p[2]) < c.final_z_tolerance and
                       np.all(angle_errors < c.angle_tolerance))
            arrived = -c.insertion_depth <= p[0] < c.final_x_tolerance and aligned
            if arrived:
                self.command = q.copy()
                self.phase = 'complete'
                return self.phase
            if -c.insertion_depth <= p[0] <= c.contact_lock_gap and aligned:
                self.correction_locked = True
            old_x = self.command[0]
            speed = min(self.axial_speed_limit(p[0]),
                        c.slowdown_gain*abs(self.approach_goal_x-old_x))
            self.command[0] = move_towards(old_x, self.approach_goal_x, speed*dt)
            if not self.correction_locked:
                self.command[1:] += self.correction.velocity(p[1:], c.correction_speed)*dt
        if dt > 0:
            self.velocity = (self.command-old_command)/dt
        return self.phase
