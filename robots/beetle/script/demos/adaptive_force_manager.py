#!/usr/bin/env python
"""ROS-free adaptive push/tow force manager.

Captures the core mocap-feedback force behavior of the aerial-towing
feedforward controller so it can be shared and unit-tested offline:

  - ramp the feedforward force up while the object has not broken away
    (has not started moving);
  - treat early motion as a candidate, confirm breakaway only after both
    distance and speed thresholds are met, then keep pushing until stable
    motion is established before falling back to a lower "maintain" force;
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
                 early_breakaway_distance=0.008,
                 early_breakaway_velocity=None,
                 maintain_force_ratio=0.6,
                 breakaway_confirm_time=0.25,
                 breakaway_hold_time=0.8,
                 breakaway_relief_min_advance=0.05,
                 target_velocity=0.05,
                 stable_motion_distance=None,
                 stable_motion_velocity=None,
                 stable_motion_time=0.8,
                 force_relief_rate=3.0,
                 overspeed_relief_time=8.0,
                 stall_window_time=5.0,
                 stall_min_advance=0.01,
                 stall_min_force_ratio=0.8,
                 stall_timeout_count=4):
        """
        Args:
            max_force: force ceiling / target while ramping (N).
            ramp_time: time to ramp from 0 to max_force before breakaway (s).
            breakaway_distance: object advance required to confirm breakaway (m).
            breakaway_velocity: object speed required to confirm breakaway (m/s).
            early_breakaway_distance: advance that marks a motion candidate (m).
            early_breakaway_velocity: speed that marks a motion candidate (m/s).
            maintain_force_ratio: post-breakaway hold force as a ratio of max_force.
            breakaway_confirm_time: continuous motion time before breakaway is confirmed (s).
            breakaway_hold_time: minimum time to keep force before relief (s).
            breakaway_relief_min_advance: extra confirmed advance before relief (m).
            target_velocity: desired advance speed after breakaway (m/s).
            stable_motion_distance: advance required before force relief (m).
            stable_motion_velocity: speed required before force relief (m/s).
            stable_motion_time: continuous stable-motion time before force relief (s).
            force_relief_rate: normal post-breakaway force reduction rate (N/s).
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
        self.breakaway_confirm_time = max(0.0, float(breakaway_confirm_time))
        self.breakaway_hold_time = max(0.0, float(breakaway_hold_time))
        self.breakaway_relief_advance = (
            self.breakaway_distance + max(0.0, float(breakaway_relief_min_advance)))
        self.target_velocity = float(target_velocity)
        self.early_breakaway_distance = max(0.0, float(early_breakaway_distance))
        self.early_breakaway_velocity = (
            self.target_velocity * 0.6 if early_breakaway_velocity is None
            else float(early_breakaway_velocity))
        self.stable_motion_distance = (
            self.breakaway_relief_advance if stable_motion_distance is None
            else float(stable_motion_distance))
        self.stable_motion_distance = max(
            self.breakaway_distance, self.stable_motion_distance)
        self.stable_motion_velocity = (
            self.target_velocity * 0.6 if stable_motion_velocity is None
            else float(stable_motion_velocity))
        self.stable_motion_time = max(0.0, float(stable_motion_time))
        self.force_relief_rate = max(0.0, float(force_relief_rate))
        self.overspeed_relief_rate = self.max_force / max(float(overspeed_relief_time), 1e-3)
        self.stall_window_time = float(stall_window_time)
        self.stall_min_advance = float(stall_min_advance)
        self.stall_min_force = self.max_force * float(stall_min_force_ratio)
        self.stall_timeout_count = int(stall_timeout_count)

        # State
        self.current_force = 0.0
        self.breakaway_detected = False
        self.breakaway_force = None
        self.breakaway_elapsed = 0.0
        self.advance_velocity = 0.0
        self.stall_windows = 0
        self.abort = False
        self.breakaway_candidate_detected = False
        self.breakaway_candidate_force = None
        self.stable_motion_detected = False
        self.breakaway_confirm_elapsed = 0.0
        self.stable_motion_elapsed = 0.0

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

        holding_breakaway = False
        motion_candidate = False
        if not self.breakaway_detected:
            # Phase 1: keep raising the force until the object starts moving.
            self.current_force = min(self.max_force, self.current_force + self.ramp_rate * dt)
            confirmed_motion_now = (
                advance >= self.breakaway_distance and
                self.advance_velocity >= self.breakaway_velocity)
            if confirmed_motion_now:
                self.breakaway_confirm_elapsed += dt
            else:
                self.breakaway_confirm_elapsed = 0.0
            confirmed_motion = (
                confirmed_motion_now and
                self.breakaway_confirm_elapsed >= self.breakaway_confirm_time)
            early_motion = (
                advance >= self.early_breakaway_distance and
                self.advance_velocity >= self.early_breakaway_velocity)
            if confirmed_motion:
                self.breakaway_detected = True
                self.breakaway_force = self.current_force
                self.breakaway_elapsed = 0.0
                holding_breakaway = True
                self.stall_windows = 0
                self._window_elapsed = 0.0
                self._window_start_advance = advance
            elif early_motion:
                motion_candidate = True
                if not self.breakaway_candidate_detected:
                    self.breakaway_candidate_detected = True
                    self.breakaway_candidate_force = self.current_force
        else:
            self.breakaway_elapsed += dt
            if not self.stable_motion_detected:
                stable_motion_now = (
                    advance >= self.stable_motion_distance and
                    self.advance_velocity >= self.stable_motion_velocity)
                if stable_motion_now:
                    self.stable_motion_elapsed += dt
                    if self.stable_motion_elapsed >= self.stable_motion_time:
                        self.stable_motion_detected = True
                else:
                    self.stable_motion_elapsed = 0.0
            holding_breakaway = (
                self.breakaway_elapsed < self.breakaway_hold_time or
                not self.stable_motion_detected)
            if holding_breakaway:
                self.current_force = min(self.max_force, self.current_force + self.ramp_rate * dt)
            else:
                # Phase 2: fall back to the maintain force gradually; relieve
                # below maintain only when the object is overspeeding.
                if self.advance_velocity > self.target_velocity:
                    self.current_force = max(
                        0.0, self.current_force - self.overspeed_relief_rate * dt)
                elif self.current_force > self.maintain_force:
                    self.current_force = max(
                        self.maintain_force,
                        self.current_force - self.force_relief_rate * dt)
                else:
                    self.current_force = min(self.maintain_force,
                                             self.current_force + self.ramp_rate * dt)

        # Stall detection: only meaningful until stable object motion is established.
        stalled_window = False
        self._window_elapsed += dt
        if self._window_elapsed >= self.stall_window_time:
            gained = advance - self._window_start_advance
            pushing_hard = self.current_force >= self.stall_min_force
            if (not self.stable_motion_detected and pushing_hard and
                    gained < self.stall_min_advance):
                self.stall_windows += 1
                stalled_window = True
                if self.stall_windows >= self.stall_timeout_count:
                    self.abort = True
            else:
                self.stall_windows = 0
            self._window_elapsed = 0.0
            self._window_start_advance = advance

        phase = 'motion_candidate' if motion_candidate else 'ramp'
        if self.breakaway_detected:
            phase = 'breakaway_hold' if holding_breakaway else 'breakaway'

        return {
            'force': self.current_force,
            'breakaway': self.breakaway_detected,
            'breakaway_candidate': self.breakaway_candidate_detected,
            'stable_motion': self.stable_motion_detected,
            'breakaway_confirm_elapsed': self.breakaway_confirm_elapsed,
            'stable_motion_elapsed': self.stable_motion_elapsed,
            'stalled': stalled_window or self.stall_windows > 0,
            'abort': self.abort,
            'phase': phase,
            'advance_velocity': self.advance_velocity,
        }
