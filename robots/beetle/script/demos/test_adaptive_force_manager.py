#!/usr/bin/env python
"""Offline unit tests for AdaptiveForceManager (no ROS, no hardware).

Run:  python3 -m unittest test_adaptive_force_manager   (from script/demos/)
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from adaptive_force_manager import AdaptiveForceManager


DT = 0.04  # 25 Hz


def run(mgr, advances):
    """Drive the manager through a sequence of advances; return the last result."""
    res = None
    for a in advances:
        res = mgr.update(a, DT)
    return res


class AdaptiveForceManagerTest(unittest.TestCase):

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
        self.assertEqual(res['phase'], 'breakaway_hold')
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

    def test_breakaway_waits_for_stable_motion_before_relief(self):
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
        self.assertEqual(res['phase'], 'breakaway_hold')
        for _ in range(50):
            res = mgr.update(0.05, DT)  # not enough stable progress
        self.assertFalse(res['stable_motion'])
        self.assertEqual(res['phase'], 'breakaway_hold')
        self.assertGreater(res['force'], 0.5 * 20.0)

        a = 0.05
        for _ in range(70):
            a += 0.05 * DT
            res = mgr.update(a, DT)
        self.assertTrue(res['stable_motion'])
        self.assertEqual(res['phase'], 'breakaway')
        self.assertLessEqual(res['force'], 0.5 * 20.0 + 1e-6)

    def test_force_falls_back_after_breakaway(self):
        # After breakaway the held force must not exceed the maintain force even
        # if the pre-breakaway force was higher.
        mgr = AdaptiveForceManager(max_force=30.0, ramp_time=0.5,
                                   breakaway_distance=0.03, maintain_force_ratio=0.5,
                                   target_velocity=0.05)
        # Ramp hard while blocked, then start moving slowly at ~target speed.
        seq = [0.0] * 20
        a = 0.031
        for _ in range(80):
            a += 0.05 * DT  # ~target_velocity
            seq.append(a)
        res = run(mgr, seq)
        self.assertTrue(res['breakaway'])
        self.assertEqual(res['phase'], 'breakaway')
        self.assertLessEqual(res['force'], 0.5 * 30.0 + 1e-6)

    def test_overspeed_relief(self):
        # Moving much faster than target after breakaway relieves the force below
        # the maintain level.
        mgr = AdaptiveForceManager(max_force=20.0, ramp_time=0.5,
                                   breakaway_distance=0.02, maintain_force_ratio=0.6,
                                   target_velocity=0.05, overspeed_relief_time=1.0)
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
