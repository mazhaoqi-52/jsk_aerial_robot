#!/usr/bin/env python
"""Offline unit tests for AdaptiveForceManager (no ROS, no hardware).

Run:  python3 -m unittest test_adaptive_force_manager   (from script/demos/)
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from adaptive_force_manager import AdaptiveForceManager
from demo_common import low_pass_update, velocity_error_adjustment


DT = 0.04  # 25 Hz


def run(mgr, advances):
    """Drive the manager through a sequence of advances; return the last result."""
    res = None
    for a in advances:
        res = mgr.update(a, DT)
    return res


class AdaptiveForceManagerTest(unittest.TestCase):

    def test_velocity_error_adjustment_is_proportional(self):
        slow = velocity_error_adjustment(0.05, 0.03, 100.0, DT, 0.005)
        fast = velocity_error_adjustment(0.05, 0.07, 100.0, DT, 0.005)
        self.assertAlmostEqual(slow, 0.06)
        self.assertAlmostEqual(fast, -0.06)

    def test_velocity_error_adjustment_has_deadband(self):
        adjustment = velocity_error_adjustment(
            0.05, 0.053, 100.0, DT, 0.005)
        self.assertEqual(adjustment, 0.0)
        just_outside = velocity_error_adjustment(
            0.05, 0.0449, 100.0, DT, 0.005)
        self.assertAlmostEqual(just_outside, 0.0004)

    def test_velocity_error_adjustment_preserves_rotation_direction(self):
        clockwise = velocity_error_adjustment(-0.1, 0.0, 1.0, DT, 0.05)
        counterclockwise = velocity_error_adjustment(0.1, 0.0, 1.0, DT, 0.05)
        self.assertAlmostEqual(clockwise, -0.002)
        self.assertAlmostEqual(counterclockwise, 0.002)

    def test_low_pass_rejects_spikes_and_invalid_samples(self):
        filtered = low_pass_update(0.0, 1.0, DT, 0.16)
        self.assertAlmostEqual(filtered, 0.2)
        self.assertEqual(low_pass_update(filtered, float('nan'), DT, 0.16),
                         filtered)
        self.assertEqual(low_pass_update(filtered, 1.0, DT, 0.0), 1.0)

    def test_ramps_up_while_blocked(self):
        # Object never moves (advance stays 0): force ramps toward max, no breakaway.
        mgr = AdaptiveForceManager(max_force=20.0, ramp_time=1.0)
        res = run(mgr, [0.0] * 25)  # ~1s
        self.assertFalse(res['breakaway'])
        self.assertGreater(res['force'], 15.0)
        self.assertLessEqual(res['force'], 20.0 + 1e-6)
        self.assertEqual(res['phase'], 'ramp')

    def test_breakaway_on_advance(self):
        # Once advance crosses breakaway_distance with real motion, breakaway
        # is detected and the force is not relieved immediately.
        mgr = AdaptiveForceManager(max_force=20.0, ramp_time=1.0,
                                   breakaway_distance=0.03, maintain_force_ratio=0.6)
        advances = [0.0] * 25
        a = 0.031
        for _ in range(10):
            a += 0.04 * DT
            advances.append(a)
        res = run(mgr, advances)
        self.assertTrue(res['breakaway'])
        self.assertEqual(res['phase'], 'breakaway_floor')
        self.assertGreater(res['force'], 0.6 * 20.0)

    def test_transient_flex_does_not_confirm_breakaway(self):
        # A top-mounted marker can jump when the wall flexes at impact. If the
        # displacement does not keep increasing, it must not unlock force relief.
        mgr = AdaptiveForceManager(max_force=20.0, ramp_time=0.5,
                                   breakaway_distance=0.03,
                                   breakaway_velocity=0.02,
                                   stable_motion_distance=0.08)
        seq = [0.0] * 10 + [0.05, 0.052, 0.051, 0.050, 0.050]
        seq += [0.050] * 20
        res = run(mgr, seq)
        self.assertFalse(res['breakaway'])
        self.assertFalse(res['stable_motion'])
        self.assertEqual(res['phase'], 'ramp')

    def test_velocity_spike_is_only_candidate(self):
        # A tiny mocap jump can have a large instantaneous velocity, but it
        # must not confirm breakaway without enough displacement.
        mgr = AdaptiveForceManager(max_force=20.0, ramp_time=1.0,
                                   breakaway_distance=0.03,
                                   breakaway_velocity=0.02,
                                   early_breakaway_distance=0.001,
                                   early_breakaway_velocity=0.02)
        mgr.update(0.0, DT)
        res = mgr.update(0.002, DT)
        self.assertFalse(res['breakaway'])
        self.assertTrue(res['breakaway_candidate'])
        self.assertEqual(res['phase'], 'motion_candidate')

    def test_breakaway_floor_waits_for_stable_motion(self):
        mgr = AdaptiveForceManager(max_force=20.0, ramp_time=0.2,
                                   breakaway_distance=0.03,
                                   breakaway_velocity=0.02,
                                   stable_motion_distance=0.10,
                                   stable_motion_velocity=0.03,
                                   maintain_force_ratio=0.5)
        run(mgr, [0.0] * 10)
        a = 0.031
        for _ in range(10):
            a += 0.04 * DT
            res = mgr.update(a, DT)  # confirmed breakaway after sustained motion
        self.assertTrue(res['breakaway'])
        self.assertFalse(res['stable_motion'])
        self.assertEqual(res['phase'], 'breakaway_floor')
        for _ in range(50):
            res = mgr.update(0.05, DT)  # not enough stable progress
        self.assertFalse(res['stable_motion'])
        self.assertEqual(res['phase'], 'breakaway_floor')
        self.assertGreater(res['force'], 0.5 * 20.0)

        a = 0.05
        for _ in range(70):
            a += 0.04 * DT
            res = mgr.update(a, DT)
        self.assertTrue(res['stable_motion'])
        self.assertEqual(res['phase'], 'velocity_feedback')
        self.assertGreater(res['force'], 0.5 * 20.0)

        force_before_overspeed = res['force']
        for _ in range(80):
            a += 0.08 * DT
            res = mgr.update(a, DT)
        self.assertLess(res['force'], force_before_overspeed)
        self.assertLess(res['force_adjust_rate'], 0.0)

    def test_post_breakaway_force_tracks_velocity_error(self):
        mgr = AdaptiveForceManager(
            max_force=30.0, ramp_time=6.0,
            breakaway_distance=0.02, breakaway_velocity=0.01,
            breakaway_confirm_time=0.0, breakaway_hold_time=0.0,
            stable_motion_distance=0.02, stable_motion_velocity=0.01,
            stable_motion_time=0.0, target_velocity=0.05,
            velocity_force_gain=100.0, velocity_error_deadband=0.005,
            velocity_filter_time_constant=0.0,
            overspeed_relief_time=1.0)
        mgr.update(0.0, DT)
        a = 0.03
        mgr.update(a, DT)

        a += 0.05 * DT
        at_target = mgr.update(a, DT)
        self.assertTrue(at_target['breakaway'])
        self.assertTrue(at_target['stable_motion'])
        self.assertAlmostEqual(at_target['force_adjust_rate'], 0.0)

        a += 0.03 * DT
        too_slow = mgr.update(a, DT)
        self.assertGreater(too_slow['force_adjust_rate'], 0.0)

        a += 0.07 * DT
        too_fast = mgr.update(a, DT)
        self.assertLess(too_fast['force_adjust_rate'], 0.0)

    def test_safety_relief_below_floor_recovers_without_jump(self):
        mgr = AdaptiveForceManager(
            max_force=20.0, ramp_time=2.0, maintain_force_ratio=0.6,
            target_velocity=0.05, velocity_force_gain=100.0,
            velocity_filter_time_constant=0.0)
        mgr.breakaway_detected = True
        mgr.breakaway_force = 18.0
        mgr.current_force = 2.0  # caller-side safety guard already unloaded it
        mgr._have_prev = True
        mgr._prev_advance = 0.1

        res = mgr.update(0.1, DT)
        self.assertLess(res['force'], mgr.maintain_force)
        self.assertAlmostEqual(res['force'], 2.18)

    def test_stale_measurement_holds_force_and_watchdog(self):
        mgr = AdaptiveForceManager(max_force=20.0, ramp_time=1.0,
                                   stall_window_time=0.1)
        mgr.update(0.0, DT)
        mgr.breakaway_confirm_elapsed = 0.1
        mgr.stable_motion_elapsed = 0.1
        force_before = mgr.current_force
        result = mgr.update(0.0, 1.0, measurement_valid=False)
        self.assertEqual(result['phase'], 'measurement_hold')
        self.assertFalse(result['measurement_valid'])
        self.assertEqual(result['force'], force_before)
        self.assertEqual(mgr._window_elapsed, 0.0)
        self.assertEqual(mgr.breakaway_confirm_elapsed, 0.0)
        self.assertEqual(mgr.stable_motion_elapsed, 0.0)
        relock = mgr.update(0.2, DT, measurement_valid=True)
        self.assertEqual(relock['phase'], 'measurement_relock')
        self.assertEqual(relock['force'], force_before)
        self.assertEqual(relock['raw_advance_velocity'], 0.0)

    def test_delayed_tick_uses_measurement_dt_but_caps_control_dt(self):
        mgr = AdaptiveForceManager(
            max_force=20.0, ramp_time=1.0, target_velocity=0.05,
            velocity_force_gain=100.0, velocity_error_deadband=0.005,
            velocity_filter_time_constant=0.0)
        mgr.breakaway_detected = True
        mgr.breakaway_force = 10.0
        mgr.stable_motion_detected = True
        mgr.current_force = 5.0
        mgr._have_prev = True
        mgr._prev_advance = 0.0

        res = mgr.update(0.0, 0.2)
        self.assertAlmostEqual(res['raw_advance_velocity'], 0.0)
        # Requested rate is 100 * (0.05 - 0.005) = 4.5 N/s, but only a
        # 0.1 s integration step is allowed after the delayed callback.
        self.assertAlmostEqual(res['force'], 5.45)

    def test_overspeed_relief(self):
        # Moving much faster than target after breakaway relieves the force below
        # the maintain level.
        mgr = AdaptiveForceManager(max_force=20.0, ramp_time=0.5,
                                   breakaway_distance=0.02, maintain_force_ratio=0.6,
                                   target_velocity=0.05, breakaway_hold_time=0.0,
                                   stable_motion_distance=0.03,
                                   stable_motion_time=0.0,
                                   overspeed_relief_time=1.0)
        seq = [0.0] * 10
        a = 0.03
        for _ in range(30):
            a += 0.5 * DT  # 10x target speed -> overspeed
            seq.append(a)
        res = run(mgr, seq)
        self.assertTrue(res['breakaway'])
        self.assertLess(res['force'], 0.6 * 20.0)

    def test_stall_aborts(self):
        # Pushing hard with no motion for enough windows -> abort.
        mgr = AdaptiveForceManager(max_force=20.0, ramp_time=0.5,
                                   stall_window_time=1.0, stall_min_advance=0.01,
                                   stall_min_force_ratio=0.8, stall_timeout_count=3)
        res = run(mgr, [0.0] * int(25 * 4))  # ~4s, 4 windows
        self.assertFalse(res['breakaway'])
        self.assertTrue(res['abort'])

    def test_no_abort_if_moving(self):
        # If the object keeps advancing, stall must never fire.
        mgr = AdaptiveForceManager(max_force=20.0, ramp_time=0.5,
                                   breakaway_distance=0.03,
                                   stall_window_time=1.0, stall_timeout_count=3)
        a = 0.0
        seq = []
        for _ in range(int(25 * 5)):
            a += 0.05 * DT
            seq.append(a)
        res = run(mgr, seq)
        self.assertTrue(res['breakaway'])
        self.assertFalse(res['abort'])


if __name__ == '__main__':
    unittest.main()
