#!/usr/bin/env python3
"""Geometry regressions without ROS transport or an aircraft."""
from pathlib import Path
import sys
import unittest
import numpy as np
from tf import transformations as tr

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'script'))
from beetle.docking_reference import FixedDockingReference, ReferenceConfig


def pose(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = tr.euler_matrix(*rpy)
    result[:3, 3] = xyz
    return result


class ReferenceTest(unittest.TestCase):
    def make_reference(self, direction=1, leader_angles=(0, 0, .3)):
        self.connectors = (pose((direction*.26, 0, 0)), pose((-direction*.26, 0, 0)))
        self.baselinks = (pose((0, 0, .06747)), pose((0, 0, .06747)))
        self.roots = [pose((1, 2, .8), leader_angles),
                      pose((1, 2, .8), leader_angles) @ pose((direction*.67, 0, 0))]
        reference = FixedDockingReference(ReferenceConfig(), *self.connectors, direction)
        reference.capture(self.roots, self.baselinks)
        return reference

    def test_capture_is_frozen_including_yaw(self):
        reference = self.make_reference()
        frame = reference.frame.copy()
        command = np.array([.15, 0, 0])
        before = reference.follower_pose(command, self.baselinks[1])
        self.roots[0] = self.roots[0] @ pose((.01, .01, .01), (.03, -.04, .02))
        reference.capture(self.roots, self.baselinks)
        reference.feedback(self.roots)
        np.testing.assert_allclose(reference.frame, frame)
        np.testing.assert_allclose(reference.follower_pose(command, self.baselinks[1]), before)
        np.testing.assert_allclose(reference.world_velocity([-.1, 0, 0]), frame[:3, 0]*-.1)

    def test_levelling_preserves_leader_fc_anchor(self):
        reference = self.make_reference(leader_angles=(.12, -.09, .3))
        anchor = reference.leader_anchor.copy()
        reference.update(5)
        desired_interface = reference.frame
        desired_fc = desired_interface @ np.linalg.inv(self.connectors[0]) @ self.baselinks[0]
        np.testing.assert_allclose(desired_fc[:3, 3], anchor)
        np.testing.assert_allclose(tr.euler_from_matrix(reference.leader_attitude(self.baselinks[0])), [0, 0, .3], atol=1e-9)

    def test_attitude_slew_uses_short_path_across_pi(self):
        reference = self.make_reference(leader_angles=(0, 0, np.deg2rad(179)))
        reference.orientations[1] = tr.quaternion_from_euler(0, 0, np.deg2rad(-179))
        reference.update(.1)
        angle = tr.euler_from_quaternion(reference.orientations[1])[2]
        self.assertAlmostEqual(abs(angle), np.pi)
        reference.update(.1)
        angle = tr.euler_from_quaternion(reference.orientations[1])[2]
        self.assertAlmostEqual(angle, np.deg2rad(179))

    def test_arrival_uses_mutual_alignment_without_extra_fixed_attitude_gate(self):
        reference = self.make_reference(leader_angles=(0, np.deg2rad(7), .3))
        relative, trajectory, errors = reference.feedback(self.roots)
        np.testing.assert_allclose(errors, np.zeros(3), atol=1e-12)

    def test_mirrored_axis_and_calibrated_connectors(self):
        reference = self.make_reference(direction=-1)
        relative, trajectory, _ = reference.feedback(self.roots)
        self.assertAlmostEqual(relative[0], .15)
        np.testing.assert_allclose(reference.follower_pose(trajectory, self.baselinks[1]),
                                   self.roots[1] @ self.baselinks[1], atol=1e-9)
        np.testing.assert_allclose(reference.world_velocity([-.1, 0, 0]), reference.frame[:3, 0]*.1)
        # A connector calibration changes the root attitude needed for mating.
        reference.connectors = (pose((.26, .01, 0), (.02, -.01, .03)),
                                pose((-.26, -.01, 0), (-.02, .01, -.03)))
        reference.frame = None
        reference.capture(self.roots, self.baselinks)
        reference.update(20)
        follower_fc = reference.follower_pose([0, 0, 0], self.baselinks[1])
        follower_interface = follower_fc @ np.linalg.inv(self.baselinks[1]) @ reference.connectors[1]
        np.testing.assert_allclose(follower_interface, reference.frame, atol=1e-9)

    def test_full_velocity_rotates_and_mirrors_like_the_position_target(self):
        for direction in (-1, 1):
            reference = self.make_reference(direction=direction)
            command = np.array([.3, .02, -.04])
            velocity = np.array([-.05, -.01, .02])
            before = reference.follower_pose(command, self.baselinks[1])[:3, 3]
            after = reference.follower_pose(command+velocity*.025, self.baselinks[1])[:3, 3]
            np.testing.assert_allclose(reference.world_velocity(velocity), (after-before)/.025,
                                       atol=1e-12)


    def test_interruption_keeps_reference_but_resets_attitude_slew(self):
        reference = self.make_reference()
        frame = reference.frame.copy()
        reference.suspend()
        self.roots[1] = self.roots[1] @ pose(rpy=(.1, 0, 0))
        reference.feedback(self.roots)
        np.testing.assert_allclose(reference.frame, frame)
        expected = self.roots[1] @ self.connectors[1]
        np.testing.assert_allclose(tr.quaternion_matrix(reference.orientations[1])[:3, :3], expected[:3, :3], atol=1e-12)

    def test_configuration_is_validated(self):
        for value in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                ReferenceConfig(attitude_rate=value)


if __name__ == '__main__':
    unittest.main()
