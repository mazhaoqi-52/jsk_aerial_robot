#!/usr/bin/env python3
"""Short alignment transients and cancellation of inward FC position lead."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'script'))
from beetle.docking_guidance import DockingConfig, DockingGuidance


class AlignmentHoldTest(unittest.TestCase):
    def setUp(self):
        self.g = DockingGuidance(DockingConfig())

    def step(self, t, p=(.15, 0, 0), r=(0, 0, 0), tracking=None):
        return self.g.update(t, p, r, p, tracking)

    def start_approach(self):
        self.step(-.25)
        self.assertEqual(self.step(0), 'approach')

    def test_brief_alignment_from_failed_trial_does_not_release_insertion(self):
        p = (.152, .0047544, -.0079094)
        r = np.radians([-1.2773, -.7366, .9264])
        for t in (0, .025, .05, .075):
            self.assertEqual(self.step(t, p, r), 'prealign')
        self.assertEqual(self.step(.1, (.15, 0, -.0184)), 'prealign')
        self.assertIsNone(self.g.approach_goal_x)

    def test_prealignment_dwell_restarts_for_each_geometry_error(self):
        cases = [((.18, 0, 0), (0, 0, 0)), ((.15, .009, 0), (0, 0, 0)),
                 ((.15, 0, -.009), (0, 0, 0)),
                 ((.15, 0, 0), np.radians([3.1, 0, 0])),
                 ((.15, 0, 0), np.radians([0, 3.1, 0])),
                 ((.15, 0, 0), np.radians([0, 0, 2.1]))]
        for p, r in cases:
            with self.subTest(p=p, r=r):
                self.g = DockingGuidance(DockingConfig())
                self.step(0)
                self.step(.2)
                self.step(.225, p, r)
                self.assertEqual(self.step(.25), 'prealign')
                self.assertEqual(self.step(.475), 'prealign')
                self.assertEqual(self.step(.5), 'approach')

    def test_each_near_alignment_error_cancels_inward_position_and_velocity(self):
        cases = [((.09, .009, 0), (0, 0, 0)), ((.09, 0, -.009), (0, 0, 0)),
                 ((.09, 0, 0), np.radians([3.1, 0, 0])),
                 ((.09, 0, 0), np.radians([0, 3.1, 0])),
                 ((.09, 0, 0), np.radians([0, 0, 2.1]))]
        for p, r in cases:
            with self.subTest(p=p, r=r):
                self.g = DockingGuidance(DockingConfig())
                self.start_approach()
                self.g.command[0] = .04  # old waypoint already inside contact zone
                self.step(.025, p, r, tracking=(.12, 0, 0))
                self.assertTrue(self.g.approach_alignment.holding)
                self.assertEqual(self.g.command[0], .12)
                self.assertEqual(self.g.axial_velocity, 0)
                self.assertAlmostEqual(self.g.approach_goal_x, -.05)

    def test_leading_waypoint_is_held_before_measured_gap_reaches_gate(self):
        self.start_approach()
        self.g.command[0] = .105
        self.step(.125, (.14, 0, -.02), tracking=(.14, 0, 0))
        self.assertTrue(self.g.approach_alignment.holding)
        self.assertEqual(self.g.command[0], .14)
        self.assertEqual(self.g.axial_velocity, 0)

    def test_far_misalignment_does_not_stop_motion_toward_alignment_gate(self):
        self.start_approach()
        self.step(.025, (.14, 0, -.02))
        self.assertFalse(self.g.approach_alignment.holding)
        self.assertLess(self.g.axial_velocity, 0)

    def test_hold_is_fixed_despite_subsequent_aircraft_and_leader_drift(self):
        self.start_approach()
        self.step(.025, (.09, 0, -.02), tracking=(.12, 0, 0))
        for t in (.05, .1, .2):
            self.step(t, (.14, 0, -.02), tracking=(.18, 0, 0))
            self.assertEqual(self.g.command[0], .12)
            self.assertEqual(self.g.axial_velocity, 0)
            self.assertTrue(self.g.approach_alignment.holding)

    def test_hold_keeps_lateral_and_height_correction_active_and_bounded(self):
        self.start_approach()
        before = self.g.command[1:].copy()
        self.step(.025, (.09, .02, -.02), tracking=(.09, 0, 0))
        self.assertLess(self.g.command[1], before[0])
        self.assertGreater(self.g.command[2], before[1])
        self.assertLessEqual(abs(self.g.velocity[1]), .02)
        self.assertLessEqual(abs(self.g.velocity[2]), .01)
        self.assertEqual(self.g.axial_velocity, 0)

    def test_recovery_requires_continuous_alignment_and_preserves_insertion_goal(self):
        self.start_approach()
        self.step(.025, (.09, 0, -.02))
        held = self.g.command[0]
        for t in (.05, .2):
            self.step(t, (.09, 0, 0))
            self.assertEqual(self.g.command[0], held)
        self.step(.225, (.09, .009, 0))
        for t in (.25, .475):
            self.step(t, (.09, 0, 0))
            self.assertEqual(self.g.command[0], held)
        self.step(.5, (.09, 0, 0))
        self.assertFalse(self.g.approach_alignment.holding)
        self.assertAlmostEqual(self.g.axial_velocity, -.05)
        self.assertAlmostEqual(held-self.g.command[0], .05*.025)
        self.assertAlmostEqual(self.g.approach_goal_x, -.05)

    def test_suspension_resets_prealignment_observations(self):
        self.step(0)
        self.step(.2)
        self.g.suspend()
        self.assertEqual(self.step(10), 'prealign')
        self.assertEqual(self.step(10.25), 'approach')

    def test_hold_and_resume_compensate_changing_attitude_lever_arm(self):
        self.start_approach()
        def update(t, z, offset):
            return self.g.update(t, [.09, 0, z], [0, 0, 0], [.09, 0, z],
                                 [.09, 0, 0], [offset, 0, 0])
        update(.025, -.02, .26)
        held_fc_x = self.g.command[0]+.26
        for t in (.05, .275):
            update(t, 0, .27)
            self.assertAlmostEqual(self.g.command[0]+.27, held_fc_x)
            self.assertEqual(self.g.axial_velocity, 0)
        update(.3, 0, .28)
        self.assertFalse(self.g.approach_alignment.holding)
        self.assertAlmostEqual(self.g.command[0]+.28, held_fc_x-.05*.025)
        self.assertAlmostEqual(self.g.axial_velocity, -.05)

    def test_suspension_during_hold_resumes_at_current_fc_without_velocity_jump(self):
        self.start_approach()
        self.step(.025, (.09, 0, -.02))
        self.g.suspend()
        self.step(10, (.09, 0, -.02), tracking=(.11, 0, -.02))
        self.assertEqual(self.g.command[0], .11)
        np.testing.assert_allclose(self.g.velocity, 0)
        self.assertTrue(self.g.approach_alignment.holding)

    def test_parameters_enforce_positive_dwell_and_gate_before_contact(self):
        for name in ('prealign_hold_duration', 'approach_alignment_gap'):
            for value in (0, -1, float('nan'), float('inf')):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    DockingConfig(**{name: value})
        for value in (.04, .05, .15, .2):
            with self.subTest(value=value), self.assertRaises(ValueError):
                DockingConfig(approach_alignment_gap=value)


if __name__ == '__main__':
    unittest.main()
