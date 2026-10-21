"""ROS adapter for guarded interface alignment and docking.

Connector frames use parallel axes at mating, with X along the insertion axis.
The defaults are nominal root offsets; calibrated offsets can be supplied in YAML.
"""
from collections import deque
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
            if name == 'angle_tolerance':
                parameters[name] = math.radians(rospy.get_param('~docking/'+name+'_deg', math.degrees(value)))
            else:
                parameters[name] = rospy.get_param('~docking/'+name, value)
        self.guidance = DockingGuidance(DockingConfig(**parameters))
        self.world = rospy.get_param('~docking/world_frame', 'world')
        self.baselink = rospy.get_param('~docking/baselink_frame', 'fc')
        self.tf_timeout = rospy.get_param('~docking/tf_timeout', 0.2)
        self.flight_timeout = rospy.get_param('~docking/flight_state_timeout', 0.5)
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
        self.history = deque()
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
        self.flight[robot] = (msg.data, rospy.Time.now().to_sec())

    def ready(self):
        now = rospy.Time.now().to_sec()
        good = all(robot in self.flight and self.flight[robot][0] == self.HOVER and
                   0 <= now-self.flight[robot][1] < self.flight_timeout
                   for robot in (self.leader, self.follower))
        if self.started and not good:
            self.abort(reason='left HOVER or flight-state feedback expired')
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
        # All dynamic transforms are looked up at one common, recent timestamp.
        frames = [self.leader+'/root', self.follower+'/root']
        stamp = min(self.listener.getLatestCommonTime(self.world, frame) for frame in frames)
        age = (rospy.Time.now()-stamp).to_sec()
        if age > self.tf_timeout or age < -.05:
            raise ValueError('stale/future root TF (age %.3f s)' % age)
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
        if self.reference.leader_displacement(world_leader, baselinks[0]) > self.reference.config.leader_drift_limit:
            self.abort(reason='leader moved beyond fixed reference limit; stabilize and restart assembly')
        timestamp = stamp.to_sec()
        if self.history and (timestamp < self.history[-1][0] or
                             timestamp-self.history[-1][0] > self.guidance.config.max_sample_gap):
            self.history.clear()
            self.guidance.invalidate()
            self.reference.suspend()
        position, trajectory, angles = self.reference.feedback(roots)
        # Guidance owns the coarse waypoint; the reference owns the additive trim.
        trajectory[1:] -= self.reference.trim
        if not self.history or timestamp > self.history[-1][0]:
            self.history.append((timestamp, position.copy()))
        while len(self.history) > 2 and timestamp-self.history[1][0] >= .15:
            self.history.popleft()
        elapsed = timestamp-self.history[0][0]
        velocity = ((position-self.history[0][1])/elapsed if .1 <= elapsed <= .3 else None)
        return timestamp, position, trajectory, angles, velocity, baselinks

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
            self.guidance.invalidate()
            self.rate.sleep()
            return 'aborted' if self.aborted else self.guidance.phase
        self.started = True
        try:
            stamp, position, trajectory, angles, velocity, baselinks = self.snapshot()
        except (tf.Exception, ValueError, np.linalg.LinAlgError) as exc:
            rospy.logwarn_throttle(1, '[Docking] holding for valid TF: %s' % exc)
            self.hold()
            self.guidance.invalidate()
            self.reference.suspend()
            self.history.clear()
            self.rate.sleep()
            return self.guidance.phase
        if self.aborted:
            return 'aborted'
        previous = self.guidance.previous_time
        dt = 0 if previous is None else stamp-previous
        if dt < 0 or dt > self.guidance.config.max_sample_gap:
            dt = 0
            self.reference.suspend()
        phase = self.guidance.update(stamp, position, angles, velocity, trajectory)
        if self.guidance.command is not None:
            trim_enabled = (phase in ('standby', 'prealign', 'approach') and
                            not self.guidance.final_holding and
                            np.linalg.norm(self.guidance.command[1:]) < .005 and
                            np.all(angles < self.guidance.config.angle_tolerance))
            self.reference.update(dt, position, trim_enabled)
            target_pose = self.reference.follower_pose(self.guidance.command, baselinks[1])
            feedforward = self.reference.axial_feedforward(self.guidance.axial_velocity)
            self.publish_leader_attitude(baselinks[0])
            self.publish_pose(target_pose, feedforward)
            self.holding = False
        if phase != self.last_phase:
            rospy.loginfo('[Docking] phase=%s gap=%.3f yz=%s attitude_error_deg=%s', phase, position[0],
                          position[1:].tolist(), np.degrees(angles).tolist())
            self.last_phase = phase
        rospy.loginfo_throttle(.5, '[Docking] phase=%s gap=%.3f yz=%s attitude_error_deg=%s trim_yz=%s speed=%s reason=%s window=%.2fs pass=%.0f%% paused=%s axial_cmd=%.3f final_hold=%.2f/%.2fs' %
                               (phase, position[0], position[1:].tolist(), np.degrees(angles).tolist(),
                                self.reference.trim.tolist(),
                                None if velocity is None else float(np.linalg.norm(velocity)),
                                self.guidance.reason, self.guidance.window_age,
                                100*self.guidance.alignment_fraction,
                                self.guidance.paused_since is not None, self.guidance.axial_velocity,
                                self.guidance.final_hold_elapsed, self.guidance.config.final_hold_time))
        self.rate.sleep()
        return phase
