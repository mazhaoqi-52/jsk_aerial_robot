#!/usr/bin/env python3
"""Offline trajectory/state regression tests; no ROS master or aircraft."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'script'))
from beetle.docking_guidance import DockingConfig, DockingGuidance


class GuidanceTest(unittest.TestCase):
    def setUp(self):
        self.g = DockingGuidance(DockingConfig())

    def step(self, t, p=(.15, 0, 0), r=(0, 0, 0), v=(0, 0, 0)):
        return self.g.update(t, p, r, v)

    def arm_approach(self):
        for i in range(43):
            self.step(i*.025)
        self.assertEqual(self.g.phase, 'approach')

    def test_window_tolerates_isolated_spikes_but_requires_current_alignment(self):
        for i in range(42):
            self.step(i*.025, p=(.15, .031 if i in (10, 41) else 0, 0))
        self.assertEqual(self.g.phase, 'prealign')  # current sample is bad
        self.step(1.05)
        self.assertEqual(self.g.phase, 'approach')

    def test_mostly_misaligned_window_cannot_release(self):
        for i in range(43):
            self.step(i*.025, p=(.15, .031 if i % 3 == 0 else 0, 0))
        self.assertEqual(self.g.phase, 'prealign')

    def test_velocity_is_not_a_prealign_gate(self):
        for velocity in (None, (.08,0,0), (float('nan'),0,0)):
            self.g = DockingGuidance(DockingConfig())
            for i in range(43):
                self.step(i*.025, v=velocity)
            self.assertEqual(self.g.phase, 'approach')

    def test_static_hover_with_two_independent_1mm_noise_sources_releases(self):
        rng = np.random.default_rng(20261020)
        positions, speeds = [], []
        for i in range(81):
            p = np.array([.15,0,0])+rng.normal(0,.001,3)-rng.normal(0,.001,3)
            positions.append(p)
            v = None if i < 6 else (p-positions[i-6])/.15
            if v is not None:
                speeds.append(np.linalg.norm(v))
            self.step(i*.025, p, v=v)
            if self.g.phase == 'approach':
                break
        self.assertTrue(any(speed >= .02 for speed in speeds))
        self.assertEqual(self.g.phase, 'approach')
        self.assertLessEqual(i*.025, 1.2)

    def test_strict_thresholds(self):
        for p, r, v in [((.15, .03, 0), (0,0,0), (0,0,0)),
                        ((.15, 0, .03), (0,0,0), (0,0,0)),
                        ((.15, 0, 0), (0,math.radians(4.1),0), (0,0,0))]:
            self.g = DockingGuidance(DockingConfig())
            for i in range(100):
                self.step(i*.025, p, r, v)
            self.assertNotEqual(self.g.phase, 'approach')

    def test_01_speed_cap_and_slowdown_near_contact(self):
        self.arm_approach()
        p = np.array([.15, 0, 0])
        speeds = []
        for i in range(43, 105):
            before = self.g.command[0]
            self.step(i*.025, p, v=(-.1, 0, 0))
            speed = (before-self.g.command[0])/.025
            self.assertLessEqual(speed, .1+1e-10)
            self.assertGreaterEqual(speed, 0)
            speeds.append(speed)
            p = self.g.command.copy()
        self.assertAlmostEqual(speeds[0], .1)
        self.assertLess(speeds[-1], .05)
        self.assertEqual(self.g.phase, 'approach')

    def test_coarse_leg_slows_before_prealign(self):
        p = np.array([.4,0,0])
        speeds = []
        for i in range(200):
            self.step(i*.025, p)
            speeds.append(-self.g.axial_velocity)
            p = self.g.command.copy()
            if self.g.phase == 'prealign':
                break
        self.assertAlmostEqual(max(speeds), .1)
        self.assertLess(speeds[-1], .05)

    def test_stalled_aircraft_does_not_accumulate_inward_waypoint(self):
        self.arm_approach()
        for i in range(43, 200):
            self.step(i*.025)
            self.assertGreaterEqual(self.g.command[0], .13-1e-12)
        self.assertAlmostEqual(self.g.axial_velocity, 0)

    def test_any_misalignment_stops_but_does_not_immediately_retreat(self):
        for p, r in [((.08,.031,0), (0,0,0)), ((.08,0,.031), (0,0,0)),
                     ((.08,0,0), (0,math.radians(5),0))]:
            self.g = DockingGuidance(DockingConfig())
            self.arm_approach()
            self.step(1.075, p, r)
            self.assertEqual(self.g.phase, 'approach')
            self.assertIsNotNone(self.g.paused_since)
            self.assertAlmostEqual(self.g.command[0], .08)
            self.assertEqual(self.g.axial_velocity, 0)
            self.step(1.1, p, r)
            self.assertAlmostEqual(self.g.command[0], .08)
            self.assertEqual(self.g.axial_velocity, 0)

    def test_pause_recovers_without_retreat(self):
        self.arm_approach()
        self.step(1.075, (.08,.04,0))
        for i in range(44, 87):
            self.step(i*.025, (.08,0,0))
        self.assertEqual(self.g.phase, 'approach')
        self.assertIsNone(self.g.paused_since)
        self.assertLess(self.g.axial_velocity, 0)

    def test_persistent_or_severe_error_retreats(self):
        self.arm_approach()
        for i in range(43, 167):
            self.step(i*.025, (.08,.04,0))
        self.assertEqual(self.g.phase, 'retreat')
        self.g = DockingGuidance(DockingConfig())
        self.arm_approach()
        self.step(1.075, (.08,.061,0))
        self.assertEqual(self.g.phase, 'retreat')
        self.assertEqual(self.g.axial_velocity, 0)  # brake on this frame

    def test_retreat_hysteresis_avoids_13cm_chatter(self):
        self.g.phase = 'retreat'
        for i in range(80):
            self.step(i*.025, (.129 if i%2 else .131,0,0))
            self.assertEqual(self.g.phase, 'retreat')
        self.step(2.0, (.146,0,0))
        self.assertEqual(self.g.phase, 'prealign')

    def test_initial_leg_does_not_close_while_misaligned(self):
        for i in range(80):
            self.step(i*.025, (.5,.1,0))
        self.assertAlmostEqual(self.g.command[0], .5)
        self.assertEqual(self.g.phase, 'standby')

    def test_duplicate_or_lost_tf_cannot_release(self):
        self.step(0)
        self.step(.025)
        for _ in range(100):
            self.step(.025)
        self.assertEqual(self.g.phase, 'prealign')
        self.step(10)  # discontinuity resets dwell
        self.assertEqual(self.g.phase, 'prealign')
        self.assertEqual(self.g.axial_velocity, 0)

    def test_tf_recovery_near_contact_requires_retreat(self):
        self.arm_approach()
        self.g.invalidate()
        self.step(4, (.04,0,0))
        self.assertEqual(self.g.phase, 'retreat')

    def test_pause_anchors_fixed_position_not_live_leader_gap(self):
        self.arm_approach()
        self.g.update(1.075, [.08, .04, 0], [0, 0, 0], [0, 0, 0], [.10, 0, 0])
        self.assertAlmostEqual(self.g.command[0], .10)
        self.g.update(1.1, [.07, .04, 0], [0, 0, 0], [0, 0, 0], [.10, 0, 0])
        self.assertAlmostEqual(self.g.command[0], .10)
        self.assertEqual(self.g.axial_velocity, 0)

    def test_final_hold_ignores_speed_but_requires_full_duration(self):
        for velocity in (None, (.11, 0, 0), (float('nan'), 0, 0)):
            self.g = DockingGuidance(DockingConfig())
            self.arm_approach()
            for i in range(43, 55):
                self.step(i*.025, (.01, 0, 0), v=velocity)
                self.assertEqual(self.g.phase, 'approach')
                self.assertEqual(self.g.axial_velocity, 0)
            self.step(55*.025, (.01, 0, 0), v=velocity)
            self.assertEqual(self.g.phase, 'complete')
            self.assertAlmostEqual(self.g.final_hold_elapsed, .3)

    def test_final_waypoint_is_captured_once_despite_measured_motion(self):
        self.arm_approach()
        self.g.command[1:] = [.003, -.002]
        self.g.update(1.075, [.01, 0, 0], [0, 0, 0], None, [.04, 0, 0])
        target = self.g.command.copy()
        for i in range(44, 55):
            # Motion exceeds the normal 2 cm lead clamp; the held target stays fixed.
            q = [.01 if i % 2 else .045, .002, -.003]
            self.g.update(i*.025, [.001, .002, -.003], [0, 0, 0], [.03, 0, 0], q)
            np.testing.assert_allclose(self.g.command, target)
            self.assertEqual(self.g.axial_velocity, 0)

    def test_final_geometry_failure_cancels_hold_even_inside_coarse_tolerance(self):
        for position, angles in [((.01, .021, 0), (0, 0, 0)),
                                 ((.01, 0, .016), (0, 0, 0)),
                                 ((.021, 0, 0), (0, 0, 0)),
                                 ((.01, 0, 0), (0, math.radians(5), 0))]:
            self.g = DockingGuidance(DockingConfig())
            self.arm_approach()
            self.step(1.075, (.01, 0, 0))
            self.step(1.1, position, angles)
            self.assertFalse(self.g.final_holding)
            self.assertIsNotNone(self.g.paused_since)
            self.assertEqual(self.g.axial_velocity, 0)
            for i in range(45, 65):
                self.step(i*.025, position, angles)
                self.assertNotEqual(self.g.phase, 'complete')

    def test_final_hold_restarts_after_alignment_recovery(self):
        self.arm_approach()
        self.step(1.075, (.01, 0, 0))
        self.step(1.1, (.01, 0, .016))
        for i in range(45, 85):
            self.step(i*.025, (.01, 0, 0))
            self.assertFalse(self.g.final_holding)
        self.step(85*.025, (.01, 0, 0))
        self.assertTrue(self.g.final_holding)
        self.assertAlmostEqual(self.g.final_hold_elapsed, 0)
        for i in range(86, 97):
            self.step(i*.025, (.01, 0, 0))
            self.assertEqual(self.g.phase, 'approach')
        self.step(97*.025, (.01, 0, 0))
        self.assertEqual(self.g.phase, 'complete')

    def test_persistent_final_geometry_loss_retreats(self):
        self.arm_approach()
        self.step(1.075, (.01, 0, 0))
        for i in range(44, 165):
            self.step(i*.025, (.01, 0, .016))
        self.assertEqual(self.g.phase, 'retreat')
        self.assertFalse(self.g.final_holding)

    def test_final_hold_cannot_accumulate_on_duplicate_or_missing_tf(self):
        self.arm_approach()
        self.step(1.075, (.01, 0, 0))
        for _ in range(100):
            self.step(1.075, (.01, 0, 0))
        self.assertEqual(self.g.final_hold_elapsed, 0)
        self.step(2, (.01, 0, 0))
        self.assertFalse(self.g.final_holding)
        self.assertEqual(self.g.phase, 'retreat')

    def test_mm_noise_and_alternating_speed_spikes_do_not_block_final_hold(self):
        self.arm_approach()
        rng = np.random.default_rng(20261021)
        for i in range(13):
            p = np.array([.003, 0, 0])+rng.normal(0, .001, 3)
            self.step(1.075+i*.025, p, v=(.03 if i%2 else .01, 0, 0))
        self.assertEqual(self.g.phase, 'complete')

    def test_bad_config_rejected(self):
        with self.assertRaises(ValueError):
            DockingConfig(approach_speed=-1)
        with self.assertRaises(ValueError):
            DockingConfig(position_tolerance=float('nan'))

    def test_full_sequence_and_recovery_with_ideal_tracking(self):
        p, r, velocity = np.array([.5,.10,.04]), np.radians([0,8,0]), np.zeros(3)
        phases, injected = set(), False
        for i in range(1600):
            phase = self.step(i*.025, p, r, velocity)
            phases.add(phase)
            if phase == 'approach' and p[0] < .10 and not injected:
                p[1] = .05
                injected = True
                continue
            if phase == 'complete':
                break
            new_p = self.g.command.copy()
            velocity = (new_p-p)/.025
            p, r = new_p, np.zeros(3)  # attitude planner is tested separately
        self.assertTrue(injected)
        self.assertEqual(phase, 'complete')
        self.assertEqual(phases, {'standby', 'prealign', 'approach', 'complete'})


if __name__ == '__main__':
    unittest.main()
