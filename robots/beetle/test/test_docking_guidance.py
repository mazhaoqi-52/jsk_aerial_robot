#!/usr/bin/env python3
"""Offline regressions for the two-stage assembly trajectory."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'script'))
from beetle.docking_guidance import DockingConfig, DockingGuidance


class GuidanceTest(unittest.TestCase):
    def setUp(self):
        self.g = DockingGuidance(DockingConfig())

    def step(self, t, p=(.15, 0, 0), r=(0, 0, 0), q=None):
        return self.g.update(t, p, r, p if q is None else q)

    def test_prealignment_requires_basic_arrival_but_no_dwell(self):
        self.assertEqual(self.step(0, (.3, .1, .08)), 'prealign')
        self.assertEqual(self.step(.025, (.15, .02, .02), np.radians([5, 0, 0])), 'approach')

    def test_prealignment_checks_each_coordinate_and_relative_angle(self):
        for p, r in [((.21, 0, 0), (0, 0, 0)), ((.15, .06, 0), (0, 0, 0)),
                     ((.15, 0, .06), (0, 0, 0)), ((.15, 0, 0), (0, .2, 0))]:
            self.g = DockingGuidance(DockingConfig())
            self.assertEqual(self.step(0, p, r), 'prealign')

    def test_misalignment_does_not_block_motion_to_prealignment_point(self):
        self.step(0, (.9, -.1, .05))
        self.step(.025, (.9, -.1, .05))
        np.testing.assert_allclose(self.g.command, [.8975, -.09875, .04875])
        self.assertAlmostEqual(self.g.axial_velocity, -.1)
        self.assertEqual(self.g.phase, 'prealign')

    def test_approach_continues_under_disturbance_without_retreat_or_abort(self):
        self.step(0)
        for i in range(1, 300):
            self.step(i*.025, (.1, .12, -.1), (.3, .3, .3))
            self.assertEqual(self.g.phase, 'approach')
        self.assertGreaterEqual(self.g.command[0], -.05)
        self.assertAlmostEqual(self.g.command[0], -.05, places=5)

    def test_measurement_motion_cannot_drag_the_fixed_waypoint(self):
        self.step(0, (.925, -.103, .051))
        self.step(.025, (.895, -.103, .051))
        self.assertAlmostEqual(self.g.command[0], .9225)
        self.assertAlmostEqual(self.g.axial_velocity, -.1)
        self.step(.05, (.5, -.103, .051))
        self.assertAlmostEqual(self.g.command[0], .92)

    def test_immediate_lock_when_final_geometry_arrives(self):
        self.step(0)
        # Even a rapid measured change is not a velocity or holding-time gate.
        self.assertEqual(self.step(.025, (.01, .005, .005)), 'complete')
        self.assertEqual(self.g.axial_velocity, 0)
        target = self.g.command.copy()
        self.step(.05, (.002, 0, 0))
        np.testing.assert_allclose(self.g.command, target)

    def test_lock_still_requires_final_position_and_angle(self):
        for p, r in [((.03, 0, 0), (0, 0, 0)), ((-.06, 0, 0), (0, 0, 0)),
                     ((0, .021, 0), (0, 0, 0)), ((0, 0, .016), (0, 0, 0)),
                     ((0, 0, 0), (0, .2, 0))]:
            self.g = DockingGuidance(DockingConfig())
            self.step(0)
            self.assertEqual(self.step(.025, p, r), 'approach')
        self.assertEqual(self.step(.05, (0, 0, 0)), 'complete')

    def test_tf_interruption_resumes_same_phase_without_target_jump(self):
        self.step(0)
        self.step(.025, (.1, 0, 0))
        self.g.suspend()
        self.assertEqual(self.g.phase, 'approach')
        self.assertEqual(self.step(10, (.08, 0, 0)), 'approach')
        np.testing.assert_allclose(self.g.command, [.08, 0, 0])
        self.assertEqual(self.g.axial_velocity, 0)

    def test_complete_sequence_and_speed_bound_with_ideal_tracking(self):
        p = np.array([.9, -.1, .05])
        phases = set()
        for i in range(1000):
            phase = self.step(i*.025, p)
            phases.add(phase)
            self.assertLessEqual(abs(self.g.axial_velocity), .1+1e-10)
            if phase == 'complete':
                break
            p = self.g.command.copy()
        self.assertEqual(phase, 'complete')
        self.assertEqual(phases, {'prealign', 'approach', 'complete'})

    def test_invalid_feedback_does_not_generate_a_target(self):
        with self.assertRaises(ValueError):
            self.step(0, (float('nan'), 0, 0))
        with self.assertRaises(ValueError):
            DockingConfig(approach_speed=-1)


if __name__ == '__main__':
    unittest.main()
