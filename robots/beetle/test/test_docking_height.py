#!/usr/bin/env python3
"""Height overshoot and blocked-contact regressions without ROS transport."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'script'))
from beetle.docking_guidance import DockingConfig, DockingGuidance, DampedHeightCorrection


class HeightCorrectionTest(unittest.TestCase):
    def test_height_tuning_does_not_change_xy_motion(self):
        commands = []
        for gain in (.1, .3):
            g = DockingGuidance(DockingConfig(height_correction_gain=gain))
            for t in (0, .025):
                g.update(t, [.4, .03, -.02], [0, 0, 0], [.4, .03, 0])
            commands.append(g.command.copy())
        np.testing.assert_allclose(commands[0][:2], commands[1][:2])
        self.assertGreater(commands[1][2], commands[0][2])

    def test_brakes_upward_motion_before_height_error_changes_sign(self):
        h = DampedHeightCorrection(DockingConfig())
        command = h.update(0, -.02, 0, 0)
        for i in range(1, 8):
            error = -.02+i*.002
            previous = command
            command = h.update(command, error, 0, .025)
        self.assertLess(error, 0)  # still below the leader, but rapidly rising
        self.assertGreater(h.relative_velocity, 0)
        self.assertLess(command, previous)

    def test_brakes_downward_motion_before_height_error_changes_sign(self):
        h = DampedHeightCorrection(DockingConfig())
        command = h.update(0, .02, 0, 0)
        for i in range(1, 8):
            error = .02-i*.002
            previous = command
            command = h.update(command, error, 0, .025)
        self.assertGreater(error, 0)
        self.assertLess(h.relative_velocity, 0)
        self.assertGreater(command, previous)

    def test_stationary_height_error_still_corrects_in_both_directions(self):
        for error in (-.04, .04):
            h = DampedHeightCorrection(DockingConfig())
            h.update(0, error, 0, 0)
            command = h.update(0, error, 0, .025)
            self.assertLess(command*error, 0)
            self.assertLessEqual(abs(command)/.025, .01)

    def test_tracking_lag_cannot_accumulate_an_unreachable_height_target(self):
        for error in (-.2, .2):
            h = DampedHeightCorrection(DockingConfig())
            command = h.update(0, error, 0, 0)
            for _ in range(1000):
                command = h.update(command, error, 0, .025)
            self.assertAlmostEqual(abs(command), .03)
            self.assertTrue(h.integration_limited)

    def test_feedback_displacement_does_not_drag_held_target(self):
        h = DampedHeightCorrection(DockingConfig(height_velocity_damping=0))
        command = .03
        h.update(command, -.2, 0, 0)
        command = h.update(command, -.2, -.1, .025)
        self.assertEqual(command, .03)  # no clipping the old target to actual+limit
        self.assertTrue(h.integration_limited)
        unwind = h.update(command, .05, -.1, .025)
        self.assertLess(unwind, command)
        self.assertLessEqual(command-unwind, .01*.025+1e-12)

    def test_tracking_bound_limits_only_the_remaining_step(self):
        h = DampedHeightCorrection(DockingConfig())
        h.update(.0299, -.2, 0, 0)
        command = h.update(.0299, -.2, 0, .025)
        self.assertAlmostEqual(command, .03)
        self.assertTrue(h.integration_limited)

    def test_contact_budget_stops_drift_even_when_both_aircraft_move_together(self):
        # Like the 20:31 bag: persistent 2 cm interface error, while local
        # position tracking is good because the aircraft descend together.
        h = DampedHeightCorrection(DockingConfig())
        command = h.update(0, .02, 0, 0, contact=True)
        for _ in range(1000):
            actual = command
            command = h.update(command, .02, actual, .025, contact=True)
        self.assertAlmostEqual(command, -.015)
        self.assertTrue(h.integration_limited)
        # A noisy gap leaving/reentering the contact zone must not renew the budget.
        command = h.update(command, .02, command, .025, contact=False)
        command = h.update(command, .02, command, .025, contact=True)
        self.assertAlmostEqual(command, -.015)

    def test_contact_budget_allows_a_small_error_to_be_corrected(self):
        h = DampedHeightCorrection(DockingConfig())
        actual = 0.0
        command = h.update(actual, -.005, actual, 0, contact=True)
        for _ in range(1000):
            command = h.update(command, actual-.005, actual, .025, contact=True)
            actual = command
        self.assertLess(abs(actual-.005), .0002)
        self.assertGreater(command, 0)

    def test_guidance_limits_contact_height_drift_without_aborting_approach(self):
        g = DockingGuidance(DockingConfig())
        g.update(-.25, [.15, 0, 0], [0, 0, 0], [.15, 0, 0])
        g.update(0, [.15, 0, 0], [0, 0, 0], [.15, 0, 0])
        for i in range(1, 1601):
            # Good FC tracking but a persistent interface mismatch.
            phase = g.update(i*.025, [.044, 0, .02], [0, 0, 0], g.command.copy())
        self.assertEqual(phase, 'approach')
        self.assertAlmostEqual(g.command[2], -.015)
        self.assertAlmostEqual(g.command[0], .15)
        self.assertEqual(g.axial_velocity, 0)
        self.assertEqual(g.velocity[2], 0)

    def test_filter_reset_keeps_contact_budget_without_velocity_spike(self):
        h = DampedHeightCorrection(DockingConfig())
        h.update(0, .02, 0, 0, contact=True)
        h.update(0, .03, 0, .025, contact=True)
        self.assertGreater(h.relative_velocity, 0)
        h.reset_velocity()
        command = h.update(-.004, -.05, -.004, 0, contact=True)
        self.assertEqual(command, -.004)
        self.assertEqual(h.relative_velocity, 0)
        self.assertEqual(h.contact_anchor, 0)

    def test_height_keeps_correcting_old_lock_geometry_until_actual_completion(self):
        g = DockingGuidance(DockingConfig())
        g.update(-.25, [.15, 0, 0], [0, 0, 0], [.15, 0, 0])
        g.update(0, [.15, 0, 0], [0, 0, 0], [.15, 0, 0])
        g.update(.025, [.04, .006, -.014], [0, 0, 0], [.04, 0, 0])
        z = g.command[2]
        g.update(.05, [.04, .006, -.014], [0, 0, 0], [.04, 0, 0])
        self.assertGreater(g.command[2], z)
        self.assertGreater(g.velocity[2], 0)
        self.assertEqual(g.update(.075, [.01, .005, .005], [0, 0, 0], [.01, 0, 0]), 'complete')
        self.assertEqual(g.velocity[2], 0)

    def test_height_resumes_when_fc_recovers_from_tracking_limit_at_four_cm(self):
        g = DockingGuidance(DockingConfig())
        g.update(-.25, [.15, 0, 0], [0, 0, 0], [.15, 0, 0])
        g.update(0, [.15, 0, 0], [0, 0, 0], [.15, 0, 0])
        # The interface remains low; FC initially cannot follow the target.
        g.update(.025, [.04, .006, -.014], [0, 0, 0], [.04, 0, -.05], [.04, 0, -.04])
        self.assertEqual(g.velocity[2], 0)
        self.assertEqual(g.height_correction.limit_reason, 'tracking')
        held = g.command[2]
        # FC tracking recovers despite a remaining connector tilt/alignment error.
        g.update(.05, [.04, .006, -.014], [0, 0, 0], [.04, 0, -.014], [.04, 0, 0])
        self.assertGreater(g.command[2], held)
        self.assertGreater(g.velocity[2], 0)
        self.assertEqual(g.height_correction.limit_reason, 'none')

    def test_height_parameter_validation_and_zero_damping(self):
        DockingConfig(height_velocity_damping=0)
        for name in ('height_correction_gain', 'height_correction_speed',
                     'height_velocity_filter_time', 'height_tracking_error_limit',
                     'contact_height_correction_limit'):
            for value in (0, -1, float('nan'), float('inf')):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    DockingConfig(**{name: value})
        with self.assertRaises(ValueError):
            DockingConfig(height_velocity_damping=-1)


if __name__ == '__main__':
    unittest.main()
