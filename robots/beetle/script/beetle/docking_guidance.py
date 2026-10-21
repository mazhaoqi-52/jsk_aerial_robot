"""ROS-independent relative-interface guidance; SI units, positive gap = apart."""
from dataclasses import dataclass
from collections import deque
import math
import numpy as np


@dataclass
class DockingConfig:
    position_tolerance: float = 0.03
    angle_tolerance: float = math.radians(4)
    settle_time: float = 1.0
    alignment_pass_ratio: float = 0.9
    approach_speed: float = 0.1
    slowdown_gain: float = 2.0
    realign_timeout: float = 3.0
    retreat_hysteresis: float = 0.015
    retreat_speed: float = 0.03
    prealign_gap: float = 0.15
    correction_speed: float = 0.05
    max_target_lead: float = 0.02
    insertion_depth: float = 0.05
    final_x_tolerance: float = 0.02
    final_y_tolerance: float = 0.02
    final_z_tolerance: float = 0.015
    final_hold_time: float = 0.3
    max_sample_gap: float = 0.2

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('%s must be finite and positive' % name)
        if self.alignment_pass_ratio > 1:
            raise ValueError('alignment_pass_ratio must be <= 1')
        if self.retreat_hysteresis >= self.final_x_tolerance:
            raise ValueError('retreat_hysteresis must be smaller than final_x_tolerance')


def move_towards(current, target, step):
    return current + np.clip(target-current, -step, step)


class DockingGuidance:
    def __init__(self, config):
        self.config = config
        self.phase = 'standby'
        self.command = None  # [positive axial gap, lateral offset, height offset]
        self.previous_time = None
        self.stable_since = None
        self.final_realign = False
        self.axial_velocity = 0.0
        self.alignment_history = deque()
        self.paused_since = None
        self.alignment_fraction = 0.0
        self.window_age = 0.0
        self.reason = 'initializing'

    def invalidate(self):
        """Require a new stable interval after unavailable/old feedback."""
        self.previous_time = None
        self.stable_since = None
        self.final_realign = False
        self.command = None
        self.axial_velocity = 0.0
        self._clear_window()
        self.paused_since = None
        if self.phase in ('approach', 'complete'):
            self.phase = 'prealign'

    def _clear_window(self):
        self.alignment_history.clear()
        self.alignment_fraction = 0.0
        self.window_age = 0.0

    def _geometry_ready(self, good, stamp):
        """Full fresh one-second window, >=90% good samples, current sample good."""
        self.alignment_history.append((stamp, bool(good)))
        cutoff = stamp-self.config.settle_time
        # Keep one boundary sample to prove that the window is fully covered.
        while len(self.alignment_history) > 1 and self.alignment_history[1][0] <= cutoff:
            self.alignment_history.popleft()
        self.window_age = min(self.config.settle_time, stamp-self.alignment_history[0][0])
        samples = [ok for t, ok in self.alignment_history if t >= cutoff-1e-9]
        self.alignment_fraction = sum(samples)/len(samples)
        return (good and self.window_age >= self.config.settle_time-1e-9 and
                self.alignment_fraction >= self.config.alignment_pass_ratio)

    def _ramp_gap(self, goal, dt, limit):
        remaining = abs(goal-self.command[0])
        speed = min(limit, self.config.slowdown_gain*remaining)
        self.command[0] = move_towards(self.command[0], goal, speed*dt)

    @property
    def final_holding(self):
        return self.stable_since is not None

    @property
    def final_hold_elapsed(self):
        if not self.final_holding or self.previous_time is None:
            return 0.0
        return max(0.0, self.previous_time-self.stable_since)

    def update(self, stamp, position, angles, velocity, trajectory_position=None):
        """Gate on relative geometry; generate waypoints in a fixed reference.

        trajectory_position is the measured follower position in that reference.
        Omitting it retains the coincident-reference offline guidance interface.
        velocity is retained for caller compatibility; speed is diagnostic only.
        """
        c = self.config
        p, r = np.asarray(position, float), np.asarray(angles, float)
        q = p if trajectory_position is None else np.asarray(trajectory_position, float)
        if not np.all(np.isfinite(np.r_[stamp, p, r, q])):
            raise ValueError('Non-finite docking feedback')
        r = (r+np.pi) % (2*np.pi)-np.pi
        if self.previous_time is not None and stamp <= self.previous_time:
            # Repeated TF cannot advance a dwell timer or a motion target.
            self.axial_velocity = 0.0
            if stamp < self.previous_time:
                self.invalidate()
            return self.phase
        dt = 0.0 if self.previous_time is None else stamp-self.previous_time
        if dt > c.max_sample_gap:
            self.invalidate()
            dt = 0.0
        self.previous_time = stamp
        if self.command is None:
            self.command = q.copy()
        aligned = (np.all(np.abs(p[1:]) < c.position_tolerance) and
                   np.all(np.abs(r) < c.angle_tolerance))
        final = (aligned and -c.insertion_depth <= p[0] < c.final_x_tolerance and
                 abs(p[1]) < c.final_y_tolerance and abs(p[2]) < c.final_z_tolerance)
        self.axial_velocity = 0.0
        self.reason = self.phase
        hold_axial = False

        if self.phase == 'approach':
            if self.final_holding and not final:
                self.stable_since = None
                self.final_realign = True
            alignment_ok = final if self.final_realign else aligned
            if not alignment_ok and self.paused_since is None:
                # Anchor once, rather than following noisy measured X every cycle.
                self.command[0] = q[0]
                self.paused_since = stamp
                self.stable_since = None
                self._clear_window()
            if self.paused_since is not None:
                hold_axial = True
                recovered = self._geometry_ready(alignment_ok, stamp)
                severe = (np.any(np.abs(p[1:]) >= 2*c.position_tolerance) or
                          np.any(np.abs(r) >= 2*c.angle_tolerance) or
                          (self.final_realign and p[0] < -c.insertion_depth))
                if recovered:
                    self.paused_since = None
                    self.final_realign = False
                    self._clear_window()
                    self.reason = 'alignment_recovered'
                elif severe or stamp-self.paused_since >= c.realign_timeout:
                    self.phase = ('retreat' if p[0] < c.prealign_gap-c.final_x_tolerance
                                  else 'prealign')
                    self.paused_since = None
                    self.final_realign = False
                    self._clear_window()
                    self.reason = 'retreat_required' if self.phase == 'retreat' else 'realign_required'
                else:
                    self.reason = 'paused_for_alignment'

        if self.phase in ('standby', 'prealign') and not aligned:
            self.command[0] = max(self.command[0], q[0])
        old_gap = self.command[0]
        if hold_axial:
            pass  # Still correct Y/Z below, with exactly zero axial feedforward.
        elif self.phase == 'standby':
            # Do not close axially before coarse alignment, even on the initial leg.
            goal = c.prealign_gap if aligned or p[0] < c.prealign_gap else max(old_gap, q[0])
            speed = c.retreat_speed if goal > old_gap else c.approach_speed
            self._ramp_gap(goal, dt, speed)
            if abs(p[0]-c.prealign_gap) < c.final_x_tolerance:
                self.phase = 'prealign'
                self.stable_since = None
                self._clear_window()
        elif self.phase == 'retreat':
            self._ramp_gap(c.prealign_gap, dt, c.retreat_speed)
            if p[0] >= c.prealign_gap-c.final_x_tolerance+c.retreat_hysteresis:
                self.phase = 'prealign'
                self.stable_since = None
                self._clear_window()
        elif self.phase == 'prealign':
            # An interrupted approach close to contact must regain clearance first.
            if p[0] < c.prealign_gap-c.final_x_tolerance:
                self.phase = 'retreat'
                self.stable_since = None
                self._clear_window()
                self.command[0] = q[0]
                hold_axial = True
                self.reason = 'insufficient_clearance'
            else:
                goal = c.prealign_gap if aligned or old_gap < c.prealign_gap else old_gap
                self._ramp_gap(goal, dt, c.retreat_speed)
                # PreAlign uses geometry; TF-derived velocity is diagnostic only.
                ready = aligned and abs(p[0]-c.prealign_gap) < c.final_x_tolerance
                if self._geometry_ready(ready, stamp):
                    self.phase = 'approach'
                    self.stable_since = None
                    self._clear_window()
                    self.reason = 'geometry_window_passed'
                else:
                    self.reason = 'geometry_outside_tolerance' if not ready else 'collecting_geometry_window'
        elif self.phase == 'approach':
            if final:
                # Capture X once. Keep the existing Y/Z waypoint and trim fixed.
                # This is a geometry dwell, not a claim of measured zero velocity.
                if not self.final_holding:
                    self.command[0] = q[0]
                    self.stable_since = stamp
                if self.final_hold_elapsed >= c.final_hold_time-1e-9:
                    self.phase = 'complete'
                self.reason = 'final_geometry_hold'
                # Bypass measured-position clamping and all waypoint ramps.
                return self.phase
            else:
                self.stable_since = None
                # Slow before nominal contact, then creep if final Y/Z is not yet met.
                speed = min(c.approach_speed, max(0.005, c.slowdown_gain*max(0.0, p[0])))
                self.command[0] = max(-c.insertion_depth, old_gap-speed*dt)

        # Bound waypoint tracking error so a stalled aircraft cannot accumulate
        # a large inward target, then lunge when the constraint disappears.
        if not hold_axial:
            self.command[0] = np.clip(self.command[0], q[0]-c.max_target_lead, q[0]+c.max_target_lead)
        if dt > 0 and not hold_axial:
            self.axial_velocity = float(np.clip((self.command[0]-old_gap)/dt,
                                               -c.approach_speed, c.retreat_speed))
        self.command[1:] = move_towards(self.command[1:], np.zeros(2), c.correction_speed*dt)
        return self.phase
