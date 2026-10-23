"""ROS adapter for prealignment and position/attitude docking navigation.

Connector frames use parallel axes at mating, with X along the insertion axis.
The defaults are nominal root offsets; calibrated offsets can be supplied in YAML.
"""
import math
import threading
import numpy as np
import rospy
import tf
from aerial_robot_msgs.msg import FlightNav
from std_msgs.msg import Empty, UInt8

from beetle.docking_guidance import DockingConfig, DockingGuidance
from beetle.docking_reference import ReferenceConfig, FixedDockingReference


def transform_matrix(translation, quaternion):
    matrix = tf.transformations.quaternion_matrix(quaternion)
    matrix[:3, 3] = translation
    return matrix


def connector_matrix(xyz, rpy):
    matrix = tf.transformations.euler_matrix(*rpy)
    matrix[:3, 3] = xyz
    if not np.all(np.isfinite(matrix)):
        raise ValueError('Invalid connector transform')
    return matrix


class DockingMotion:
    HOVER = 5

    def __init__(self, leader, follower, attach_dir=1):
        self.leader, self.follower = leader, follower
        self.direction = float(attach_dir)
        if self.direction not in (-1.0, 1.0):
            raise ValueError('attach_dir must be +1 or -1')
        defaults = DockingConfig()
        parameters = {}
        for name, value in vars(defaults).items():
            if name.endswith('angle_tolerance') or name == 'approach_alignment_yaw_scale':
                parameters[name] = math.radians(rospy.get_param('~docking/'+name+'_deg', math.degrees(value)))
            else:
                parameters[name] = rospy.get_param('~docking/'+name, value)
        self.guidance = DockingGuidance(DockingConfig(**parameters))
        self.world = rospy.get_param('~docking/world_frame', 'world')
        self.baselink = rospy.get_param('~docking/baselink_frame', 'fc')
        self.leader_connector = connector_matrix(
            rospy.get_param('~docking/leader_connector_xyz', [self.direction*.26, 0, 0]),
            rospy.get_param('~docking/leader_connector_rpy', [0, 0, 0]))
        self.follower_connector = connector_matrix(
            rospy.get_param('~docking/follower_connector_xyz', [-self.direction*.26, 0, 0]),
            rospy.get_param('~docking/follower_connector_rpy', [0, 0, 0]))
        reference_parameters = {}
        for name, value in vars(ReferenceConfig()).items():
            key = '~docking/reference/'+name
            reference_parameters[name] = (math.radians(rospy.get_param(key+'_deg', math.degrees(value)))
                                          if name == 'attitude_rate' else rospy.get_param(key, value))
        self.reference = FixedDockingReference(ReferenceConfig(**reference_parameters),
                                              self.leader_connector, self.follower_connector, self.direction)
        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher(follower+'/uav/nav', FlightNav, queue_size=1)
        self.leader_publisher = rospy.Publisher(leader+'/uav/nav', FlightNav, queue_size=1)
        self.flight = {}
        self.command_lock = threading.RLock()
        self.started = False
        self.aborted = False
        self.holding = False
        self.last_pose = None
        self.last_phase = None
        self.rate = rospy.Rate(40)
        self.subscribers = [rospy.Subscriber(robot+'/flight_state', UInt8, self.flight_cb, robot)
                            for robot in (leader, follower)]
        self.subscribers.append(rospy.Subscriber('/emergency_assembly_interuption', Empty, self.abort))

    def flight_cb(self, msg, robot):
        self.flight[robot] = msg.data

    def ready(self):
        good = all(self.flight.get(robot) == self.HOVER for robot in (self.leader, self.follower))
        if self.started and not good:
            self.abort(reason='left HOVER')
        return good and not self.aborted

    def abort(self, _msg=None, reason='operator interruption'):
        with self.command_lock:
            if not self.aborted:
                rospy.logwarn('[Docking] aborted: %s', reason)
                self.aborted = True
                self.hold()

    def hold(self):
        """Remove the previous inward waypoint; do not send a far-away escape target."""
        if self.holding or self.last_pose is None:
            return
        self.holding = True
        self.publish_pose(self.last_pose, np.zeros(3), holding=True)

    def snapshot(self):
        # Use one common TF timestamp without comparing it to the ground PC clock.
        frames = [self.leader+'/root', self.follower+'/root']
        stamp = min(self.listener.getLatestCommonTime(self.world, frame) for frame in frames)
        world_leader, world_follower = [transform_matrix(*self.listener.lookupTransform(self.world, frame, stamp))
                                        for frame in frames]
        baselinks = [transform_matrix(*self.listener.lookupTransform(
            robot+'/root', robot+'/'+self.baselink, stamp)) for robot in (self.leader, self.follower)]
        roots = (world_leader, world_follower)
        if not all(np.all(np.isfinite(matrix)) for matrix in (*roots, *baselinks)):
            raise ValueError('non-finite docking TF')
        self.last_pose = world_follower @ baselinks[1]
        if self.reference.frame is None:
            self.reference.capture(roots, baselinks)
            rospy.loginfo('[Docking] frozen interface reference xyz=%s yaw=%.1f deg',
                          self.reference.frame[:3, 3].tolist(),
                          math.degrees(tf.transformations.euler_from_matrix(self.reference.frame)[2]))
        position, trajectory, angles = self.reference.feedback(roots)
        return position, trajectory, angles, baselinks

    def publish_pose(self, pose, velocity, holding=False):
        rpy = tf.transformations.euler_from_matrix(pose)
        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.BASELINK
        # POS_VEL also explicitly exits waypoint velocity navigation and sets v=0 on hold.
        msg.pos_xy_nav_mode = msg.pos_z_nav_mode = FlightNav.POS_VEL_MODE
        msg.roll_nav_mode = msg.pitch_nav_mode = msg.yaw_nav_mode = FlightNav.POS_MODE
        msg.target_pos_x, msg.target_pos_y, msg.target_pos_z = pose[:3, 3]
        msg.target_vel_x, msg.target_vel_y, msg.target_vel_z = velocity
        msg.target_roll, msg.target_pitch, msg.target_yaw = rpy
        with self.command_lock:
            if not self.aborted or holding:
                self.publisher.publish(msg)

    def publish_leader_attitude(self, root_baselink):
        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.BASELINK
        # Preserve the leader's existing position target; only set attitude.
        msg.pos_xy_nav_mode = msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
        msg.roll_nav_mode = msg.pitch_nav_mode = msg.yaw_nav_mode = FlightNav.POS_MODE
        msg.target_roll, msg.target_pitch, msg.target_yaw = tf.transformations.euler_from_matrix(
            self.reference.leader_attitude(root_baselink))
        with self.command_lock:
            if not self.aborted:
                self.leader_publisher.publish(msg)

    def tick(self):
        if not self.ready():
            self.guidance.suspend()
            self.rate.sleep()
            return 'aborted' if self.aborted else self.guidance.phase
        self.started = True
        try:
            position, trajectory, angles, baselinks = self.snapshot()
        except (tf.Exception, ValueError, np.linalg.LinAlgError) as exc:
            rospy.logwarn_throttle(1, '[Docking] holding for valid TF: %s' % exc)
            self.hold()
            self.guidance.suspend()
            self.reference.suspend()
            self.rate.sleep()
            return self.guidance.phase
        if self.aborted:
            return 'aborted'
        now = rospy.Time.now().to_sec()
        previous = self.guidance.previous_time
        dt = 0.0 if previous is None else max(0.0, now-previous)
        phase = self.guidance.update(now, position, angles, trajectory)
        if self.guidance.command is not None:
            self.reference.update(dt)
            target_pose = self.reference.follower_pose(self.guidance.command, baselinks[1])
            feedforward = self.reference.world_velocity(self.guidance.velocity)
            self.publish_leader_attitude(baselinks[0])
            self.publish_pose(target_pose, feedforward)
            self.holding = False
        if phase != self.last_phase:
            rospy.loginfo('[Docking] phase=%s gap=%.3f yz=%s relative_rpy_deg=%s', phase, position[0],
                          position[1:].tolist(), np.degrees(angles).tolist())
            self.last_phase = phase
        rospy.loginfo_throttle(.5, '[Docking] phase=%s gap=%.3f yz=%s relative_rpy_deg=%s target_xyz=%s axial_cmd=%.3f target_fc_z=%.3f correction_locked=%s relative_vz=%.3f height_limited=%s relative_vy=%.3f lateral_limited=%s approach_scale=%.3f' %
                               (phase, position[0], position[1:].tolist(), np.degrees(angles).tolist(),
                                self.guidance.command.tolist(), self.guidance.axial_velocity,
                                target_pose[2, 3], self.guidance.correction_locked,
                                self.guidance.height_correction.relative_velocity,
                                self.guidance.height_correction.integration_limited,
                                self.guidance.lateral_correction.relative_velocity,
                                self.guidance.lateral_correction.integration_limited,
                                self.guidance.approach_speed_policy.alignment_scale(position[1:], angles[2])))
        self.rate.sleep()
        return phase
