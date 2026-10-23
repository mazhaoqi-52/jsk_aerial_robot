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
        self.assertEqual(self.step(.025, (.15, .005, .005), np.radians([2, 0, 0])), 'approach')

    def test_prealignment_checks_each_coordinate_and_relative_angle(self):
        for p, r in [((.175, 0, 0), (0, 0, 0)), ((.125, 0, 0), (0, 0, 0)),
                     ((.15, .025, 0), (0, 0, 0)), ((.15, 0, -.025), (0, 0, 0)),
                     ((.15, 0, 0), np.radians([4, 0, 0])),
                     ((.15, 0, 0), np.radians([0, -4, 0])),
                     ((.15, 0, 0), np.radians([0, 0, 4]))]:
            self.g = DockingGuidance(DockingConfig())
            self.assertEqual(self.step(0, p, r), 'prealign')

    def test_real_trial_entry_with_height_and_pitch_error_stays_in_prealign(self):
        self.assertEqual(self.step(0, (.199, -.014, -.044),
                                   np.radians([-4.35, -5.57, 2.70])), 'prealign')

    def test_near_approach_uses_measured_gap_even_with_far_fixed_waypoint(self):
        self.step(0, q=(.4, 0, 0))
        self.step(.025, (.12, 0, 0), q=(.4, 0, 0))
        self.assertAlmostEqual(self.g.axial_velocity, -.05)
        self.assertAlmostEqual(self.g.command[0], .39875)

    def test_approach_speed_ramps_continuously_with_actual_gap(self):
        speeds = []
        for gap, expected in [(.4, .1), (.225, .075), (.15, .05), (.08, .05), (-.02, .05)]:
            self.g = DockingGuidance(DockingConfig())
            self.step(0, q=(.4, 0, 0))
            # Roll prevents completion so distance scaling can be checked.
            self.step(.025, (gap, 0, 0), r=(.2, 0, 0), q=(.4, 0, 0))
            speeds.append(abs(self.g.axial_velocity))
            self.assertAlmostEqual(speeds[-1], expected)
        self.assertTrue(all(a >= b for a, b in zip(speeds, speeds[1:])))

    def test_global_speed_limit_also_applies_to_near_approach(self):
        self.g = DockingGuidance(DockingConfig(approach_speed=.03))
        self.step(0, q=(.4, 0, 0))
        self.step(.025, (.08, 0, 0), q=(.4, 0, 0))
        self.assertAlmostEqual(self.g.axial_velocity, -.03)

    def test_final_angle_tolerance_is_independent_of_prealignment(self):
        self.step(0)
        self.assertEqual(self.step(.025, (.01, 0, 0), np.radians([0, 5, 0])), 'complete')

    def test_misalignment_does_not_block_motion_to_prealignment_point(self):
        self.step(0, (.9, -.1, .05))
        self.step(.025, (.9, -.1, .05))
        np.testing.assert_allclose(self.g.command, [.8975, -.0995, .04975])
        self.assertAlmostEqual(self.g.axial_velocity, -.1)
        self.assertEqual(self.g.phase, 'prealign')

    def test_approach_continues_under_disturbance_without_retreat_or_abort(self):
        self.step(0)
        for i in range(1, 2000):
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

    def test_prealign_corrects_leader_displacement_and_persistent_tracking_bias(self):
        # Reproduce a displaced leader and persistent tracking offsets instead
        # of assuming that following the old absolute waypoint means alignment.
        leader = np.array([.03, .03, .02227])
        tracking_bias = np.array([-.03, -.025, 0])
        actual = np.array([.15, 0, 0])+tracking_bias
        for i in range(1000):
            phase = self.step(i*.025, actual-leader, q=actual)
            self.assertLessEqual(np.max(np.abs(self.g.velocity[1:])), .02+1e-10)
            if phase == 'approach':
                break
            # First order position tracking with a steady offset.
            actual += (self.g.command+tracking_bias-actual)*(.025/.4)
        self.assertEqual(phase, 'approach')
        self.assertGreater(self.g.command[0], .18)
        self.assertGreater(self.g.command[1], .03)
        self.assertGreater(self.g.command[2], 0)
        self.assertLess(abs(actual[2]-leader[2]), .02)
        self.assertAlmostEqual(self.g.approach_goal_x, leader[0]-.05)

    def test_relative_height_error_moves_target_without_copying_measurement(self):
        self.step(0, (.3, 0, -.04), q=(.3, 0, 0))
        self.step(.025, (.3, 0, -.04), q=(.3, 0, 0))
        self.assertAlmostEqual(self.g.command[2], .0002)
        self.assertAlmostEqual(self.g.velocity[2], .008)
        self.step(.05, (.3, 0, .04), q=(.3, 0, 0))
        self.assertAlmostEqual(self.g.command[2], -.00005)
        self.assertAlmostEqual(self.g.velocity[2], -.01)

    def test_approach_goal_uses_current_leader_and_stays_fixed(self):
        self.step(0, (.15, 0, 0), q=(.22, 0, 0))
        self.assertAlmostEqual(self.g.approach_goal_x, .02)
        self.step(.025, (.1, .03, -.04), q=(.4, .05, .06))
        self.assertAlmostEqual(self.g.approach_goal_x, .02)
        # The old Y target is already 5 cm behind actual: do not add more lead.
        self.assertEqual(self.g.velocity[1], 0)
        self.assertTrue(self.g.lateral_correction.integration_limited)
        self.assertGreater(self.g.velocity[2], 0)

    def test_contact_keeps_correcting_small_offsets_while_insertion_continues(self):
        self.step(0)
        self.step(.025, (.08, .006, -.014), q=(.08, 0, 0))
        yz = self.g.command[1:].copy()
        x = self.g.command[0]
        self.step(.05, (.04, .006, -.014), q=(.04, 0, 0))
        self.assertLess(self.g.command[1], yz[0])
        self.assertGreater(self.g.command[2], yz[1])
        self.assertLess(self.g.command[0], x)
        self.assertAlmostEqual(self.g.approach_goal_x, -.05)

    def test_contact_correction_remains_active_despite_angle_error(self):
        self.step(0)
        self.step(.025, (.04, .03, 0), q=(.04, 0, 0))
        self.assertLess(self.g.velocity[1], 0)
        self.step(.05, (.04, .01, .01), r=(0, .2, 0))
        self.assertEqual(self.g.phase, 'approach')
        self.assertNotEqual(self.g.velocity[2], 0)

    def test_tf_interruption_preserves_insertion_goal_and_contact_budgets(self):
        self.step(0)
        self.step(.025, (.04, .01, .01))
        anchors = (self.g.lateral_correction.contact_anchor, self.g.height_correction.contact_anchor)
        self.g.suspend()
        self.step(10, (.035, 0, 0), q=(.06, .02, .03))
        self.assertAlmostEqual(self.g.approach_goal_x, -.05)
        self.assertEqual((self.g.lateral_correction.contact_anchor, self.g.height_correction.contact_anchor), anchors)
        np.testing.assert_allclose(self.g.command, [.06, .02, .03])
        np.testing.assert_allclose(self.g.velocity, [0, 0, 0])

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
