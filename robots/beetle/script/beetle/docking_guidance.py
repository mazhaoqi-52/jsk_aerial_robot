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
    near_approach_speed: float = 0.03
    approach_slowdown_distance: float = 0.15
    approach_alignment_y_scale: float = 0.01
    approach_alignment_z_scale: float = 0.01
    approach_alignment_yaw_scale: float = math.radians(2.0)
    approach_min_speed_scale: float = 0.2
    correction_speed: float = 0.02
    relative_correction_gain: float = 0.5
    lateral_velocity_damping: float = 0.4
    lateral_velocity_filter_time: float = 0.25
    lateral_tracking_error_limit: float = 0.03
    contact_lateral_correction_limit: float = 0.01
    contact_lateral_correction_gap: float = 0.05
    height_correction_gain: float = 0.2
    height_correction_speed: float = 0.01
    height_velocity_damping: float = 0.4
    height_velocity_filter_time: float = 0.25
    height_tracking_error_limit: float = 0.03
    contact_height_correction_limit: float = 0.015
    contact_height_correction_gap: float = 0.05
    contact_lock_gap: float = 0.04
    slowdown_gain: float = 2.0
    insertion_depth: float = 0.05
    final_x_tolerance: float = 0.02
    final_y_tolerance: float = 0.02
    final_z_tolerance: float = 0.015

    def __post_init__(self):
        for name, value in vars(self).items():
            minimum_ok = value >= 0 if name in ('height_velocity_damping', 'lateral_velocity_damping') else value > 0
            if not math.isfinite(value) or not minimum_ok:
                raise ValueError('Invalid docking parameter: %s' % name)
        if self.approach_min_speed_scale > 1:
            raise ValueError('approach_min_speed_scale must be in (0, 1]')


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


class FilteredRelativeVelocity:
    """Differentiate interface error on the local control clock and smooth it."""
    def __init__(self, time_constant):
        self.time_constant = time_constant
        self.reset()

    def reset(self):
        self.previous_error = None
        self.velocity = 0.0

    def update(self, error, dt):
        if dt > 0 and self.previous_error is not None:
            measured_velocity = (error-self.previous_error)/dt
            alpha = dt/(self.time_constant+dt)
            self.velocity += alpha*(measured_velocity-self.velocity)
        self.previous_error = error
        return self.velocity


class AlignmentAwareApproachSpeed:
    """Continuously slow insertion when Y, Z or yaw still needs correction."""
    def __init__(self, config):
        self.config = config

    def alignment_scale(self, yz, yaw):
        c = self.config
        yaw = (yaw+math.pi) % (2*math.pi)-math.pi
        error = max(abs(yz[0])/c.approach_alignment_y_scale,
                    abs(yz[1])/c.approach_alignment_z_scale,
                    abs(yaw)/c.approach_alignment_yaw_scale)
        return c.approach_min_speed_scale+(1-c.approach_min_speed_scale)/(1+error*error)

    def limit(self, gap, yz, yaw):
        c = self.config
        distance_limit = min(c.approach_speed,
                             c.near_approach_speed*max(1.0, gap/c.approach_slowdown_distance))
        return distance_limit*self.alignment_scale(yz, yaw)


class BoundedDampedCorrection:
    """Integrate damped interface feedback within tracking and contact bounds.

    Feedback may reduce the available step, but never drags an existing target.
    A latched contact anchor also bounds motion when both aircraft move together.
    """
    def __init__(self, gain, speed_limit, damping, filter_time,
                 tracking_error_limit, contact_correction_limit):
        self.gain, self.speed_limit, self.damping = gain, speed_limit, damping
        self.tracking_error_limit = tracking_error_limit
        self.contact_correction_limit = contact_correction_limit
        self.contact_anchor = None
        self.velocity_filter = FilteredRelativeVelocity(filter_time)
        self.reset_velocity()

    @property
    def relative_velocity(self):
        return self.velocity_filter.velocity

    def reset_velocity(self):
        self.velocity_filter.reset()
        self.integration_limited = False

    def velocity(self, error, dt, enabled=True):
        relative_velocity = self.velocity_filter.update(error, dt)
        if not enabled:
            return 0.0
        return float(np.clip(-self.gain*error-self.damping*relative_velocity,
                             -self.speed_limit, self.speed_limit))

    def update(self, command, error, actual, dt, contact=False, enabled=True):
        self.integration_limited = False
        velocity = self.velocity(error, dt, enabled)
        if contact and self.contact_anchor is None:
            self.contact_anchor = command
        if dt <= 0 or not enabled:
            return command
        step = float(velocity*dt)
        lower = actual-self.tracking_error_limit
        upper = actual+self.tracking_error_limit
        if self.contact_anchor is not None:
            lower = max(lower, self.contact_anchor-self.contact_correction_limit)
            upper = min(upper, self.contact_anchor+self.contact_correction_limit)
        # Stop outward integration at a bound; allow motion back toward it.
        # If feedback moves beyond a bound, keep the existing target continuous.
        limited_step = (min(step, max(0.0, upper-command)) if step > 0 else
                        max(step, -max(0.0, command-lower)))
        self.integration_limited = abs(limited_step-step) > 1e-12
        return command+limited_step


class DampedLateralCorrection(BoundedDampedCorrection):
    def __init__(self, config):
        super().__init__(config.relative_correction_gain, config.correction_speed,
                         config.lateral_velocity_damping, config.lateral_velocity_filter_time,
                         config.lateral_tracking_error_limit, config.contact_lateral_correction_limit)


class DampedHeightCorrection(BoundedDampedCorrection):
    def __init__(self, config):
        super().__init__(config.height_correction_gain, config.height_correction_speed,
                         config.height_velocity_damping, config.height_velocity_filter_time,
                         config.height_tracking_error_limit, config.contact_height_correction_limit)


class DockingGuidance:
    def __init__(self, config):
        self.config = config
        self.phase = 'prealign'
        self.command = None
        self.previous_time = None
        self.velocity = np.zeros(3)
        self.correction = RelativePositionCorrection(config.relative_correction_gain)
        self.approach_speed_policy = AlignmentAwareApproachSpeed(config)
        self.lateral_correction = DampedLateralCorrection(config)
        self.height_correction = DampedHeightCorrection(config)
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
        self.lateral_correction.reset_velocity()
        self.height_correction.reset_velocity()

    def axial_speed_limit(self, gap, yz=(0, 0), yaw=0.0):
        """Reduce approach speed using measured separation, in fixed coordinates."""
        c = self.config
        if self.phase == 'prealign':
            return c.approach_speed
        return self.approach_speed_policy.limit(gap, yz, yaw)

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
            self.command[0] += self.correction.velocity(error[0], c.approach_speed)*dt
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
            speed = min(self.axial_speed_limit(p[0], p[1:], r[2]),
                        c.slowdown_gain*abs(self.approach_goal_x-old_x))
            self.command[0] = move_towards(old_x, self.approach_goal_x, speed*dt)
        lateral_contact = self.phase == 'approach' and p[0] <= c.contact_lateral_correction_gap
        self.command[1] = self.lateral_correction.update(
            self.command[1], p[1], q[1], dt, lateral_contact, enabled=not self.correction_locked)
        contact = self.phase == 'approach' and p[0] <= c.contact_height_correction_gap
        self.command[2] = self.height_correction.update(
            self.command[2], p[2], q[2], dt, contact, enabled=not self.correction_locked)
        if dt > 0:
            self.velocity = (self.command-old_command)/dt
        return self.phase
