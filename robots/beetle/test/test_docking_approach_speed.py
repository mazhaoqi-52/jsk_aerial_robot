#!/usr/bin/env python3
"""Fine prealignment followed by prompt, bounded insertion."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'script'))
from beetle.docking_guidance import GapLimitedApproachSpeed, DockingConfig, DockingGuidance


class ApproachSpeedTest(unittest.TestCase):
    def test_failed_bag_coarse_alignment_no_longer_starts_insertion(self):
        for yz, angles in [([-.0189095, .0187643], [2.34185, -2.54283, -1.27007]),
                           ([-.0188917, .0197988], [.06313, -1.39309, -2.99635])]:
            g = DockingGuidance(DockingConfig())
            self.assertEqual(g.update(0, [.165, *yz], np.radians(angles), [.165, 0, 0]), 'prealign')
            self.assertIsNone(g.approach_goal_x)

    def test_each_fine_alignment_error_keeps_correction_at_noncontact_gap(self):
        cases = [([.15, .009, 0], [0, 0, 0]), ([.15, 0, -.009], [0, 0, 0]),
                 ([.15, 0, 0], [0, 0, 2.1]), ([.15, 0, 0], [3.1, 0, 0])]
        for p, deg in cases:
            g = DockingGuidance(DockingConfig())
            for t in (0, .025):
                phase = g.update(t, p, np.radians(deg), [.15, 0, 0])
            self.assertEqual(phase, 'prealign')
            self.assertEqual(g.command[0], .15)
            self.assertEqual(g.axial_velocity, 0)

    def test_fine_alignment_immediately_releases_faster_insertion(self):
        g = DockingGuidance(DockingConfig())
        self.assertEqual(g.update(0, [.15, .006, -.007], np.radians([2.5, -2.5, 1.5]), [.4, 0, 0]), 'approach')
        # Residual disturbance keeps bounded Y/Z correction active, without crawling.
        g.update(.025, [.1, .006, -.014], np.radians([0, 0, 5]), [.4, 0, 0])
        self.assertAlmostEqual(g.axial_velocity, -.05)
        self.assertGreater(g.velocity[2], 0)

    def test_distance_speed_is_continuous_and_respects_global_cap(self):
        policy = GapLimitedApproachSpeed(DockingConfig())
        for gap, expected in [(.4, .1), (.225, .075), (.15, .05), (.05, .05)]:
            self.assertAlmostEqual(policy.limit(gap), expected)
        for gap in (.15, .3):
            self.assertLess(abs(policy.limit(gap-1e-7)-policy.limit(gap+1e-7)), 1e-6)
        self.assertAlmostEqual(GapLimitedApproachSpeed(DockingConfig(approach_speed=.03)).limit(.05), .03)

    def test_prealign_releases_without_dwell_and_angles_wrap(self):
        g = DockingGuidance(DockingConfig())
        self.assertEqual(g.update(0, [.15, .02, .02], [0, 0, 0], [.15, 0, 0]), 'prealign')
        self.assertEqual(g.update(.025, [.15, .005, .005], [0, 0, 2*np.pi+.01], [.15, 0, 0]), 'approach')

    def test_fine_alignment_parameters_are_positive_and_finite(self):
        for name in ('prealign_y_tolerance', 'prealign_z_tolerance', 'prealign_yaw_tolerance'):
            for value in (0, -1, float('nan'), float('inf')):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    DockingConfig(**{name: value})


if __name__ == '__main__':
    unittest.main()
