#!/usr/bin/env python3
"""Alignment-dependent approach speed, without new state-transition gates."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'script'))
from beetle.docking_guidance import AlignmentAwareApproachSpeed, DockingConfig, DockingGuidance


class ApproachSpeedTest(unittest.TestCase):
    def setUp(self):
        self.c = DockingConfig()
        self.policy = AlignmentAwareApproachSpeed(self.c)

    def test_each_error_smoothly_slows_both_signs_and_still_progresses(self):
        for axis, scale in enumerate((.01, .01, np.radians(2))):
            for sign in (-1, 1):
                speeds = []
                for ratio in (0, .5, 1, 2, 4, 20):
                    e = np.zeros(3)
                    e[axis] = sign*scale*ratio
                    speeds.append(self.policy.limit(.1, e[:2], e[2]))
                self.assertAlmostEqual(speeds[0], .03)
                self.assertTrue(all(a > b for a, b in zip(speeds, speeds[1:])))
                self.assertTrue(all(.006 <= s <= .03 for s in speeds))

    def test_two_cm_height_error_slows_insertion_while_height_corrects(self):
        g = DockingGuidance(self.c)
        g.update(0, [.15, 0, 0], [0, 0, 0], [.4, 0, 0])
        g.update(.025, [.1, 0, .02], [0, 0, 0], [.4, 0, 0])
        self.assertEqual(g.phase, 'approach')
        self.assertAlmostEqual(g.axial_velocity, -.0108)
        self.assertLess(g.velocity[2], 0)

    def test_bag_entry_with_three_near_boundary_errors_is_not_full_speed(self):
        limit = self.policy.limit(.156, [-.0188917, .0197988], np.radians(-2.99635))
        self.assertGreater(limit, 0)
        self.assertLess(limit, .012)
        self.assertGreater(self.policy.limit(.156, [0, 0], 0), limit)

    def test_yaw_wrap_and_geometry_noise_do_not_create_speed_jumps(self):
        self.assertAlmostEqual(self.policy.limit(.1, [0, 0], .01),
                               self.policy.limit(.1, [0, 0], 2*np.pi+.01))
        for error in (.0, .01, .02):
            lo = self.policy.limit(.1, [error-1e-7, 0], 0)
            hi = self.policy.limit(.1, [error+1e-7, 0], 0)
            self.assertLess(abs(hi-lo), 1e-6)

    def test_prealign_speed_and_common_yaw_goal_are_unaffected(self):
        g = DockingGuidance(self.c)
        g.update(0, [.9, .1, .1], [0, 0, .3], [.9, .1, .1])
        g.update(.025, [.9, .1, .1], [0, 0, .3], [.9, .1, .1])
        self.assertEqual(g.phase, 'prealign')
        self.assertAlmostEqual(g.axial_velocity, -.1)

    def test_speed_scales_are_loaded_and_validated_independently(self):
        for name in ('approach_alignment_y_scale', 'approach_alignment_z_scale',
                     'approach_alignment_yaw_scale', 'approach_min_speed_scale'):
            for value in (0, -1, float('nan'), float('inf')):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    DockingConfig(**{name: value})
        with self.assertRaises(ValueError):
            DockingConfig(approach_min_speed_scale=1.01)
        disabled = AlignmentAwareApproachSpeed(DockingConfig(approach_min_speed_scale=1))
        self.assertAlmostEqual(disabled.limit(.1, [.1, .1], 1), .03)


if __name__ == '__main__':
    unittest.main()
