#!/usr/bin/env python3
"""Lateral braking and continued correction before the 4 cm target lock."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'script'))
from beetle.docking_guidance import DockingConfig, DockingGuidance, DampedLateralCorrection


class LateralCorrectionTest(unittest.TestCase):
    def test_brakes_before_crossing_alignment_in_both_directions(self):
        for sign in (-1, 1):
            c = DampedLateralCorrection(DockingConfig())
            c.velocity(sign*.015, 0)
            for i in range(1, 8):
                error = sign*(.015-i*.0015)
                velocity = c.velocity(error, .025)
            self.assertGreater(sign*error, 0)  # still on the initial side
            self.assertGreater(sign*velocity, 0)  # brake the motion toward zero
            self.assertLess(sign*c.relative_velocity, 0)

    def test_static_offset_corrects_with_bounded_target_speed(self):
        for error in (-.1, -.01, .01, .1):
            c = DampedLateralCorrection(DockingConfig())
            c.velocity(error, 0)
            velocity = c.velocity(error, .025)
            self.assertLess(velocity*error, 0)
            self.assertLessEqual(abs(velocity), .02)

    def test_zero_damping_retains_position_feedback_for_comparison(self):
        c = DampedLateralCorrection(DockingConfig(lateral_velocity_damping=0))
        c.velocity(-.015, 0)
        for i in range(1, 8):
            error = -.015+i*.0015
            velocity = c.velocity(error, .025)
        self.assertGreater(velocity, 0)  # position-only control keeps chasing
        self.assertAlmostEqual(velocity, -.5*error)

    def test_braking_is_active_in_prealign_and_approach(self):
        for x, expected_phase in ((.4, 'prealign'), (.15, 'approach')):
            g = DockingGuidance(DockingConfig())
            g.update(0, [x, -.015, 0], [0, 0, 0], [x, -.015, 0])
            for i in range(1, 8):
                phase = g.update(i*.025, [x, -.015+i*.0015, 0], [0, 0, 0], [x, 0, 0])
            self.assertEqual(phase, expected_phase)
            self.assertLess(g.velocity[1], 0)
            self.assertLessEqual(abs(g.velocity[1]), .02)

    def test_lateral_damping_does_not_change_x_or_height_policy(self):
        commands = []
        for damping in (0, .4):
            g = DockingGuidance(DockingConfig(lateral_velocity_damping=damping))
            for i in range(12):
                g.update(i*.025, [.4, -.02+i*.002, -.04], [0, 0, 0], [.4, 0, 0])
            commands.append(g.command.copy())
        np.testing.assert_allclose(commands[0][[0, 2]], commands[1][[0, 2]])
        self.assertNotAlmostEqual(commands[0][1], commands[1][1])

    def test_four_to_five_cm_band_keeps_bounded_correction_and_height_budget(self):
        g = DockingGuidance(DockingConfig())
        g.update(0, [.15, 0, 0], [0, 0, 0], [.15, 0, 0])
        g.update(.025, [.045, .01, -.01], [0, 0, 0], [.045, .01, -.01])
        self.assertFalse(g.correction_locked)
        self.assertLess(g.velocity[1], 0)
        self.assertLessEqual(abs(g.velocity[1]), .02)
        self.assertGreater(g.velocity[2], 0)
        self.assertLessEqual(abs(g.velocity[2]), .01+1e-12)
        self.assertEqual(g.height_correction.contact_anchor, 0)
        yz = g.command[1:].copy()
        g.update(.05, [.04, .01, -.01], [0, 0, 0], [.04, .01, -.01])
        self.assertTrue(g.correction_locked)
        np.testing.assert_allclose(g.command[1:], yz)
        np.testing.assert_allclose(g.velocity[1:], [0, 0])

    def test_tf_recovery_resets_lateral_derivative_without_target_jump(self):
        g = DockingGuidance(DockingConfig())
        g.update(0, [.4, -.02, 0], [0, 0, 0], [.4, -.02, 0])
        g.update(.025, [.4, -.01, 0], [0, 0, 0], [.4, -.01, 0])
        self.assertGreater(g.lateral_correction.relative_velocity, 0)
        g.suspend()
        g.update(10, [.3, .1, 0], [0, 0, 0], [.3, .1, 0])
        self.assertEqual(g.lateral_correction.relative_velocity, 0)
        np.testing.assert_allclose(g.command, [.3, .1, 0])
        np.testing.assert_allclose(g.velocity, [0, 0, 0])

    def test_lateral_parameters_are_validated(self):
        DockingConfig(lateral_velocity_damping=0)
        for value in (-1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                DockingConfig(lateral_velocity_damping=value)
        for name in ('lateral_velocity_filter_time', 'contact_height_correction_gap'):
            for value in (0, -1, float('nan'), float('inf')):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    DockingConfig(**{name: value})


if __name__ == '__main__':
    unittest.main()
