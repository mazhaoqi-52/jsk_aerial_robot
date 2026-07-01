#!/usr/bin/env python
"""ROS-free adaptive push/tow force manager.

Captures the core mocap-feedback force behavior of the aerial-towing
feedforward controller so it can be shared and unit-tested offline:

  - ramp the feedforward force up while the object has not broken away
    (has not started moving);
  - once the object breaks away (advance distance / speed crosses a threshold),
    fall back to a lower "maintain" force and relieve it further if the object
    moves faster than the target speed;
  - if the force is already high but the object still does not move, count
    consecutive stall windows and raise an abort (the object is effectively
    fixed -> the push degrades to a static force test).

Deliberately free of ROS: time is passed in as ``dt``, configuration is plain
arguments, and it returns a diagnostics dict instead of logging. The caller
feeds the measured ``advance`` (progress along the push direction, in metres,
>= 0) each tick and applies the returned force magnitude.

This is derived from LinearTowingTrajectoryGenerator (aerial towing); the plan
is to fold towing onto this shared core once it is proven.
"""


class AdaptiveForceManager:
    def __init__(self, max_force,
                 ramp_time=3.0,
                 breakaway_distance=0.03,
                 breakaway_velocity=0.02,
                 maintain_force_ratio=0.6,
                 target_velocity=0.05,
                 overspeed_relief_time=8.0,
                 stall_window_time=5.0,
                 stall_min_advance=0.01,
                 stall_min_force_ratio=0.8,
                 stall_timeout_count=4):
        """
        Args:
            max_force: force ceiling / target while ramping (N).
            ramp_time: time to ramp from 0 to max_force before breakaway (s).
            breakaway_distance: object advance that confirms breakaway (m).
            breakaway_velocity: object advance speed that confirms breakaway (m/s).
            maintain_force_ratio: post-breakaway hold force as a ratio of max_force.
            target_velocity: desired advance speed after breakaway (m/s).
            overspeed_relief_time: time constant for reducing force when overspeeding (s).
            stall_window_time: stall detection window length (s).
            stall_min_advance: min advance within a window to not be stalled (m).
            stall_min_force_ratio: only count stall while force >= this ratio of max_force.
            stall_timeout_count: consecutive stalled windows before abort.
        """
        self.max_force = max(0.0, float(max_force))
        self.ramp_rate = self.max_force / max(float(ramp_time), 1e-3)
        self.breakaway_distance = float(breakaway_distance)
        self.breakaway_velocity = float(breakaway_velocity)
        self.maintain_force = self.max_force * float(maintain_force_ratio)
        self.target_velocity = float(target_velocity)
        self.overspeed_relief_rate = self.max_force / max(float(overspeed_relief_time), 1e-3)
        self.stall_window_time = float(stall_window_time)
        self.stall_min_advance = float(stall_min_advance)
        self.stall_min_force = self.max_force * float(stall_min_force_ratio)
        self.stall_timeout_count = int(stall_timeout_count)

        # State
        self.current_force = 0.0
        self.breakaway_detected = False
        self.breakaway_force = None
        self.advance_velocity = 0.0
        self.stall_windows = 0
        self.abort = False

        self._prev_advance = 0.0
        self._have_prev = False
        self._window_elapsed = 0.0
        self._window_start_advance = 0.0

    def update(self, advance, dt):
        """Advance the force state by one control tick.

        Args:
            advance: measured progress along the push direction (m, >= 0).
            dt: time since the previous update (s).

        Returns:
            dict: force, breakaway, stalled, abort, phase, advance_velocity.
        """
        dt = max(float(dt), 1e-3)
        advance = max(0.0, float(advance))

        if not self._have_prev:
            self._prev_advance = advance
            self._window_start_advance = advance
            self._have_prev = True
        self.advance_velocity = (advance - self._prev_advance) / dt
        self._prev_advance = advance

        if not self.breakaway_detected:
            # Phase 1: keep raising the force until the object starts moving.
            self.current_force = min(self.max_force, self.current_force + self.ramp_rate * dt)
            if (advance >= self.breakaway_distance or
                    self.advance_velocity >= self.breakaway_velocity):
                self.breakaway_detected = True
                self.breakaway_force = self.current_force
                self.stall_windows = 0
                self._window_elapsed = 0.0
                self._window_start_advance = advance
        else:
            # Phase 2: fall back to the maintain force; relieve if overspeeding,
            # recover toward maintain if slow. Never exceed the maintain force.
            if self.advance_velocity > self.target_velocity:
                self.current_force = max(0.0, self.current_force - self.overspeed_relief_rate * dt)
            else:
                self.current_force = min(self.maintain_force,
                                         self.current_force + self.ramp_rate * dt)
            self.current_force = min(self.current_force, self.maintain_force)

        # Stall detection: only meaningful while ramping (pushing hard, no motion).
        stalled_window = False
        self._window_elapsed += dt
        if self._window_elapsed >= self.stall_window_time:
            gained = advance - self._window_start_advance
            pushing_hard = self.current_force >= self.stall_min_force
            if (not self.breakaway_detected and pushing_hard and
                    gained < self.stall_min_advance):
                self.stall_windows += 1
                stalled_window = True
                if self.stall_windows >= self.stall_timeout_count:
                    self.abort = True
            else:
                self.stall_windows = 0
            self._window_elapsed = 0.0
            self._window_start_advance = advance

        return {
            'force': self.current_force,
            'breakaway': self.breakaway_detected,
            'stalled': stalled_window or self.stall_windows > 0,
            'abort': self.abort,
            'phase': 'breakaway' if self.breakaway_detected else 'ramp',
            'advance_velocity': self.advance_velocity,
        }
