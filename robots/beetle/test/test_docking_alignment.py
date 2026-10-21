#!/usr/bin/env python3
"""Exercise actual ROS message construction with transport/time mocked out."""
from pathlib import Path
import sys
import unittest
import importlib.util
from unittest.mock import MagicMock, patch
import numpy as np
import rospy
import tf
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import UInt8

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'script'))
from beetle.assembly_alignment import DockingMotion, transform_matrix, connector_matrix
from beetle.assembly_api import AssemblyState
from beetle.assembly_hardware import AssemblyHardware


class AdapterTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.patches = [patch('rospy.get_param', side_effect=lambda k, default=None: default),
                        patch('rospy.Publisher', side_effect=lambda *a, **kw: MagicMock()), patch('rospy.Subscriber'), patch('rospy.Rate'),
                        patch('rospy.Time.now', side_effect=lambda: rospy.Time.from_sec(self.now)),
                        patch('rospy.loginfo'), patch('rospy.logwarn'), patch('rospy.loginfo_throttle'),
                        patch('rospy.logwarn_throttle'), patch('tf.TransformListener')]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)
        self.m = DockingMotion('beetle2', 'beetle3')
        self.refresh_flight()
        self.leader = connector_matrix([1,2,.8], [.1,-.08,.3])
        self.root_fc = connector_matrix([0,0,.06747], [0,0,0])
        self.follower = self.leader @ connector_matrix([.67,0,0], [0,0,0])
        self.m.listener.getLatestCommonTime.side_effect = lambda *args: rospy.Time.from_sec(self.now-.01)
        self.m.listener.lookupTransform.side_effect = self.lookup

    def refresh_flight(self):
        for robot in ('beetle2', 'beetle3'):
            self.m.flight_cb(UInt8(data=5), robot)

    def lookup(self, parent, child, stamp):
        a = self.root_fc if parent in ('beetle2/root', 'beetle3/root') else self.leader if child == 'beetle2/root' else self.follower
        return a[:3,3], tf.transformations.quaternion_from_matrix(a)

    def test_pose_transform_and_rp_navigation(self):
        self.m.tick()
        msg = self.m.publisher.publish.call_args[0][0]
        self.assertEqual(msg.roll_nav_mode, FlightNav.POS_MODE)
        self.assertEqual(msg.pitch_nav_mode, FlightNav.POS_MODE)
        self.assertEqual(msg.pos_xy_nav_mode, FlightNav.POS_VEL_MODE)
        np.testing.assert_allclose([msg.target_roll, msg.target_pitch, msg.target_yaw], [.1,-.08,.3], atol=1e-9)
        expected = self.follower @ self.root_fc
        np.testing.assert_allclose([msg.target_pos_x,msg.target_pos_y,msg.target_pos_z], expected[:3,3])
        stamps = [call.args[2].to_sec() for call in self.m.listener.lookupTransform.call_args_list]
        self.assertTrue(all(abs(t-(self.now-.01)) < 1e-8 for t in stamps))

    def test_ros_loads_separate_angle_and_near_speed_parameters(self):
        values = {'~docking/prealign_angle_tolerance_deg': 2.5,
                  '~docking/angle_tolerance_deg': 7.0,
                  '~docking/near_approach_speed': .04,
                  '~docking/approach_slowdown_distance': .12,
                  '~docking/relative_correction_gain': .6,
                  '~docking/contact_lock_gap': .04}
        with patch('rospy.get_param', side_effect=lambda key, default=None: values.get(key, default)):
            motion = DockingMotion('beetle2', 'beetle3')
        config = motion.guidance.config
        self.assertAlmostEqual(config.prealign_angle_tolerance, np.radians(2.5))
        self.assertAlmostEqual(config.angle_tolerance, np.radians(7.0))
        self.assertEqual(config.near_approach_speed, .04)
        self.assertEqual(config.approach_slowdown_distance, .12)
        self.assertEqual(config.relative_correction_gain, .6)
        self.assertEqual(config.contact_lock_gap, .04)

    def test_mirrored_connectors(self):
        self.m = DockingMotion('beetle2', 'beetle3', -1)
        self.refresh_flight()
        self.follower = self.leader @ connector_matrix([-.67,0,0], [0,0,0])
        self.m.listener.getLatestCommonTime.side_effect = lambda *a: rospy.Time.from_sec(self.now-.01)
        self.m.listener.lookupTransform.side_effect = self.lookup
        self.m.tick()
        expected_gap = self.m.reference.feedback((self.leader, self.follower))[1][0]
        self.assertAlmostEqual(self.m.guidance.command[0], expected_gap)
        msg = self.m.publisher.publish.call_args[0][0]
        np.testing.assert_allclose([msg.target_pos_x,msg.target_pos_y,msg.target_pos_z], (self.follower @ self.root_fc)[:3,3])

    def test_missing_tf_holds_and_clears_dwell(self):
        self.m.tick()
        self.m.guidance.phase = 'approach'
        self.now += .25
        self.refresh_flight()
        self.m.listener.getLatestCommonTime.side_effect = tf.Exception('missing root TF')
        self.m.tick()
        msg = self.m.publisher.publish.call_args[0][0]
        self.assertEqual([msg.target_vel_x,msg.target_vel_y,msg.target_vel_z], [0,0,0])
        self.assertEqual(self.m.guidance.phase, 'approach')
        self.assertIsNone(self.m.guidance.previous_time)

    def test_ground_clock_offset_does_not_block_navigation(self):
        self.leader = connector_matrix([1, 2, .8], [0, 0, .3])
        self.follower = self.leader @ connector_matrix([.67, 0, 0], [0, 0, 0])
        for offset in (.45, -.45):
            with self.subTest(tf_offset=offset):
                self.m = DockingMotion('beetle2', 'beetle3')
                self.m.listener.getLatestCommonTime.side_effect = lambda *a: rospy.Time.from_sec(self.now+offset)
                self.m.listener.lookupTransform.side_effect = self.lookup
                for _ in range(45):
                    self.refresh_flight()
                    phase = self.m.tick()
                    self.now += .025
                self.assertEqual(phase, 'approach')
                self.assertFalse(self.m.aborted)
                self.assertEqual(self.m.publisher.publish.call_count, 45)
                self.assertIsNotNone(self.m.reference.frame)

    def test_land_aborts_and_prevents_late_publish(self):
        self.m.tick()
        self.m.flight_cb(UInt8(data=4), 'beetle3')
        self.assertEqual(self.m.tick(), 'aborted')
        count = self.m.publisher.publish.call_count
        self.m.publish_pose(np.eye(4), np.ones(3))
        self.assertEqual(self.m.publisher.publish.call_count, count)

    def test_emergency_holds_once_without_escape_waypoint(self):
        self.m.tick()
        self.m.abort()
        msg = self.m.publisher.publish.call_args[0][0]
        np.testing.assert_allclose([msg.target_pos_x,msg.target_pos_y,msg.target_pos_z], self.m.last_pose[:3,3])
        self.assertEqual(msg.target_vel_x, 0)
        count = self.m.publisher.publish.call_count
        self.m.abort()
        self.assertEqual(self.m.publisher.publish.call_count, count)

    def test_cannot_lock_after_land(self):
        self.m.tick()
        self.m.flight_cb(UInt8(data=4), 'beetle3')
        state = AssemblyState(AssemblyHardware(robot_name='beetle3', leader='beetle2', real_machine=True, motion=self.m))
        with patch('beetle.assembly_hardware.DynamixelControl') as servo:
            self.assertEqual(state.execute(None), 'emergency')
            servo.assert_not_called()

    def test_leader_commands_attitude_without_changing_position_target(self):
        self.m.tick()
        msg = self.m.leader_publisher.publish.call_args[0][0]
        self.assertEqual(msg.pos_xy_nav_mode, FlightNav.NO_NAVIGATION)
        self.assertEqual(msg.pos_z_nav_mode, FlightNav.NO_NAVIGATION)
        self.assertEqual(msg.roll_nav_mode, FlightNav.POS_MODE)
        self.assertEqual(msg.pitch_nav_mode, FlightNav.POS_MODE)

    def test_leader_perturbation_keeps_fixed_axes_and_insertion_goal(self):
        # Isolate the frozen frame from the separately tested gap-based speed cap.
        self.m.guidance.config.near_approach_speed = .1
        self.leader = connector_matrix([1, 2, .8], [0, 0, .3])
        self.follower = self.leader @ connector_matrix([.67, 0, 0], [0, 0, 0])
        self.m.tick()
        before = self.m.publisher.publish.call_args[0][0]
        frozen = self.m.reference.frame.copy()
        goal_x = self.m.guidance.approach_goal_x
        # A disturbance may correct Y/Z without rotating the frame or moving X goal.
        self.leader = self.leader @ connector_matrix([0, 0, 0], [.02, -.02, .01])
        self.now += .025
        self.refresh_flight()
        self.m.tick()
        after = self.m.publisher.publish.call_args[0][0]
        np.testing.assert_allclose([after.target_roll, after.target_pitch, after.target_yaw],
                                   [before.target_roll, before.target_pitch, before.target_yaw], atol=1e-10)
        np.testing.assert_allclose(self.m.reference.frame, frozen)
        self.assertEqual(self.m.guidance.approach_goal_x, goal_x)
        delta = np.array([after.target_pos_x, after.target_pos_y, after.target_pos_z])-np.array(
            [before.target_pos_x, before.target_pos_y, before.target_pos_z])
        np.testing.assert_allclose(delta, self.m.reference.world_velocity(self.m.guidance.velocity)*.025,
                                   atol=1e-9)
        self.assertLessEqual(np.max(np.abs(self.m.guidance.velocity[1:])), .02)

    def test_leader_height_recovery_publishes_increasing_follower_height(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.m = DockingMotion('beetle2', 'beetle3', direction)
                self.refresh_flight()
                self.leader = connector_matrix([1, 2, .70], [0, 0, .3])
                self.follower = self.leader @ connector_matrix([direction*.9, 0, 0], [0, 0, 0])
                self.m.listener.getLatestCommonTime.side_effect = lambda *args: rospy.Time.from_sec(self.now-.01)
                self.m.listener.lookupTransform.side_effect = self.lookup
                self.m.tick()
                before = self.m.publisher.publish.call_args[0][0]
                self.leader[2, 3] += .03
                self.now += .025
                self.m.tick()
                after = self.m.publisher.publish.call_args[0][0]
                self.assertEqual(self.m.guidance.phase, 'prealign')
                self.assertGreater(after.target_pos_z, before.target_pos_z)
                self.assertAlmostEqual(after.target_vel_z, .015)
                self.assertAlmostEqual(after.target_pos_z-before.target_pos_z, after.target_vel_z*.025)

    def test_leader_height_drift_does_not_abort_or_reset_reference(self):
        self.m.tick()
        frozen = self.m.reference.frame.copy()
        self.leader[2, 3] += .06  # reproduce the displacement that aborted the real trial
        self.now += .025
        phase = self.m.tick()
        self.assertNotEqual(phase, 'aborted')
        self.assertFalse(self.m.aborted)
        np.testing.assert_allclose(self.m.reference.frame, frozen)
        self.assertEqual(self.m.publisher.publish.call_count, 2)

    def test_flight_feedback_age_does_not_abort_navigation(self):
        self.m.tick()
        self.now += 1.0
        # Last reported state remains HOVER; no receipt-time expiration gate.
        self.assertNotEqual(self.m.tick(), 'aborted')
        self.assertFalse(self.m.aborted)


    def test_tf_recovery_keeps_frame_and_has_no_pose_jump(self):
        self.m.tick()
        frozen = self.m.reference.frame.copy()
        self.m.listener.lookupTransform.side_effect = tf.Exception('missing')
        self.now += .025
        self.m.tick()
        self.m.listener.lookupTransform.side_effect = self.lookup
        self.follower = self.follower @ connector_matrix([0, .01, 0], [.03, 0, 0])
        self.now += .025
        self.refresh_flight()
        self.m.tick()
        np.testing.assert_allclose(self.m.reference.frame, frozen)
        msg = self.m.publisher.publish.call_args[0][0]
        actual = self.follower @ self.root_fc
        np.testing.assert_allclose([msg.target_pos_x, msg.target_pos_y, msg.target_pos_z], actual[:3, 3])
        np.testing.assert_allclose([msg.target_roll, msg.target_pitch, msg.target_yaw],
                                   tf.transformations.euler_from_matrix(actual))

    def test_complete_fixed_reference_sequence_with_ideal_tracking(self):
        self.follower = self.leader @ connector_matrix([1.0, .08, .03], [0, .03, 0])
        anchor = (self.leader @ self.root_fc)[:3, 3].copy()
        phases = set()
        for _ in range(1000):
            self.refresh_flight()
            phase = self.m.tick()
            phases.add(phase)
            if phase == 'complete':
                break
            follower_msg = self.m.publisher.publish.call_args[0][0]
            leader_msg = self.m.leader_publisher.publish.call_args[0][0]
            self.follower = connector_matrix(
                [follower_msg.target_pos_x, follower_msg.target_pos_y, follower_msg.target_pos_z],
                [follower_msg.target_roll, follower_msg.target_pitch, follower_msg.target_yaw]) @ np.linalg.inv(self.root_fc)
            self.leader = connector_matrix(anchor, [leader_msg.target_roll, leader_msg.target_pitch,
                                                    leader_msg.target_yaw]) @ np.linalg.inv(self.root_fc)
            self.now += .025
        self.assertEqual(phase, 'complete')
        self.assertEqual(phases, {'prealign', 'approach', 'complete'})

    def test_complete_sequence_after_height_recovery_with_tracking_bias(self):
        self.leader = connector_matrix([1, 2, .77-.06747], [0, 0, .3])
        self.follower = self.leader @ connector_matrix([1.0, .08, .03], [0, 0, 0])
        bias = tf.transformations.euler_matrix(0, 0, .3)[:3, :3] @ [-.025, -.025, -.01]
        phases = set()
        for i in range(1200):
            # Leader recovers 3 cm after reference capture, like the real test.
            self.leader[2, 3] = .77-.06747+.03*min(i*.025/2, 1)
            phase = self.m.tick()
            phases.add(phase)
            if phase == 'complete':
                break
            msg = self.m.publisher.publish.call_args[0][0]
            actual_fc = self.follower @ self.root_fc
            target = np.array([msg.target_pos_x, msg.target_pos_y, msg.target_pos_z])
            # Simple lagged tracking with a persistent positioning error.
            position = actual_fc[:3, 3]+(target+bias-actual_fc[:3, 3])*(.025/.4)
            self.follower = connector_matrix(position, [msg.target_roll, msg.target_pitch,
                                                        msg.target_yaw]) @ np.linalg.inv(self.root_fc)
            self.now += .025
        self.assertEqual(phase, 'complete')
        self.assertEqual(phases, {'prealign', 'approach', 'complete'})
        self.assertGreater(msg.target_pos_z, .79)


    def test_hardware_preparation_is_once_and_mirrors_male_female(self):
        for direction, expected in [(1, ['beetle2', 'beetle3']), (-1, ['beetle3', 'beetle2'])]:
            hardware = AssemblyHardware(robot_name='beetle3', robot_id=3, leader='beetle2', leader_id=2,
                                        real_machine=True, attach_dir=direction, motion=self.m)
            with patch('beetle.assembly_hardware.DynamixelControl') as servo:
                servo.return_value.dx_pos_pub.get_num_connections.return_value = 1
                self.assertTrue(hardware.prepare())
                self.assertTrue(hardware.prepare())
                self.assertEqual([c.args[0] for c in servo.call_args_list], expected)
                self.assertEqual([c.args[0] for c in servo.return_value.sendTargetAngle.call_args_list], [1820, 3000])

    def test_hardware_interruption_during_preparation_stops_second_servo(self):
        hardware = AssemblyHardware(real_machine=True, motion=self.m)
        with patch('beetle.assembly_hardware.DynamixelControl') as servo:
            servo.return_value.dx_pos_pub.get_num_connections.return_value = 1
            servo.return_value.sendTargetAngle.side_effect = lambda _: self.m.abort()
            self.assertFalse(hardware.prepare())
            self.assertFalse(hardware.prepared)
            self.assertEqual(servo.return_value.sendTargetAngle.call_count, 1)

    def test_servo_subscriber_timeout_never_reports_prepared(self):
        hardware = AssemblyHardware(real_machine=True, motion=self.m)
        self.m.started = True
        def advance(_):
            self.now += 1
            self.refresh_flight()
        with patch('beetle.assembly_hardware.DynamixelControl') as servo, patch('rospy.sleep', side_effect=advance):
            servo.return_value.dx_pos_pub.get_num_connections.return_value = 0
            self.assertFalse(hardware.prepare())
            servo.return_value.sendTargetAngle.assert_not_called()
            self.assertTrue(self.m.aborted)

    def test_handoff_exception_aborts_even_after_guidance_complete(self):
        self.m.tick()
        self.m.guidance.phase = 'complete'
        hardware = AssemblyHardware(motion=self.m)
        with patch.object(hardware, 'finish', side_effect=RuntimeError('handoff failure')):
            with self.assertRaises(RuntimeError):
                AssemblyState(hardware).execute(None)
        self.assertTrue(self.m.aborted)

    def test_launch_task_state_machine_is_consistent(self):
        path = Path(__file__).resolve().parents[1]/'script/task/assembly_motion.py'
        spec = importlib.util.spec_from_file_location('assembly_task_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        captured = []
        def inspect(machine):
            machine.check_consistency()
            sub = machine.get_children()['SUB1']
            sub.check_consistency()
            captured.extend(sub.get_children().keys())
            return 'succeeded'
        with patch('smach_ros.IntrospectionServer'), patch('smach.StateMachine.execute', inspect):
            self.assertEqual(module.AssemblyDemo([2,3], False).main(), 'succeeded')
        self.assertEqual(set(captured), {'StandbyState3_2', 'PreAlignState3_2', 'ApproachState3_2',
                                        'AssemblyState3_2'})


if __name__ == '__main__':
    unittest.main()
