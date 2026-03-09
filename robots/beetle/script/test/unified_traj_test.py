#!/usr/bin/env python
"""
Trajectory tracking test node for beetle unified / leader-follower modes.

Publishes FlightNav to /assembly/uav/nav at 40 Hz, sampling from a selected
trajectory class. Suitable for comparing unified vs leader-follower performance
on the same trajectory.

Usage:
  rosrun beetle unified_traj_test.py _traj:=lemniscate _loops:=2
  rosrun beetle unified_traj_test.py _traj:=circle _loops:=1 _period:=30
  rosrun beetle unified_traj_test.py _traj:=lemniscate_yaw
  rosrun beetle unified_traj_test.py _traj:=lemniscate_omni
  rosrun beetle unified_traj_test.py _traj:=setpoint

Params (private):
  ~traj         : trajectory name (circle | lemniscate | lemniscate_yaw | lemniscate_omni | setpoint)
  ~loops        : number of loops (default 1)
  ~period       : override trajectory period [s] (default: use class default)
  ~radius       : circle radius [m] (default 0.5)
  ~a            : lemniscate amplitude [m] (default 0.8)
  ~z_center     : base altitude [m] (default 1.0)
  ~z_range      : z oscillation amplitude [m] (default 0.2)
  ~a_ori        : orientation amplitude for omni [rad] (default 0.5)
  ~t_converge   : setpoint convergence time [s] (default 8.0)
  ~settle_time  : hover time before/after trajectory [s] (default 3.0)
"""
import sys
import rospy
import numpy as np
from aerial_robot_msgs.msg import FlightNav

# Allow running from the script directory without installing the package
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from traj_classes import (
    CircleTraj, LemniscateTraj, LemniscateTrajYaw,
    LemniscateTrajOmni, SetPointTraj
)


TRAJ_MAP = {
    'circle': CircleTraj,
    'lemniscate': LemniscateTraj,
    'lemniscate_yaw': LemniscateTrajYaw,
    'lemniscate_omni': LemniscateTrajOmni,
    'setpoint': SetPointTraj,
}


def build_traj(name, loops, params):
    """Instantiate the requested trajectory with user-overridden parameters."""
    if name == 'circle':
        return CircleTraj(
            loop_num=loops,
            radius=params.get('radius', 0.5),
            period=params.get('period', 20.0),
            z=params.get('z_center', 0.8),
        )
    elif name in ('lemniscate', 'lemniscate_yaw'):
        cls = LemniscateTrajYaw if name == 'lemniscate_yaw' else LemniscateTraj
        return cls(
            loop_num=loops,
            a=params.get('a', 0.8),
            z_range=params.get('z_range', 0.2),
            period=params.get('period', 20.0),
            z_center=params.get('z_center', 1.0),
        )
    elif name == 'lemniscate_omni':
        return LemniscateTrajOmni(
            loop_num=loops,
            a=params.get('a', 0.8),
            z_range=params.get('z_range', 0.2),
            period=params.get('period', 20.0),
            z_center=params.get('z_center', 1.0),
            a_orientation=params.get('a_ori', 0.5),
        )
    elif name == 'setpoint':
        return SetPointTraj(
            loop_num=loops,
            t_converge=params.get('t_converge', 8.0),
        )
    else:
        rospy.logfatal('Unknown trajectory: %s (choices: %s)', name, list(TRAJ_MAP.keys()))
        sys.exit(1)


def make_nav_msg(traj, t):
    """Sample trajectory at time t and build a FlightNav message."""
    msg = FlightNav()
    msg.header.stamp = rospy.Time.now()

    # Position + velocity (POS_VEL_MODE for feedforward)
    pt = traj.get_3d_pt(t)
    if len(pt) == 6:
        x, y, z, vx, vy, vz = pt
    else:
        # fallback: 9-element legacy format (ignore acc)
        x, y, z, vx, vy, vz = pt[0], pt[1], pt[2], pt[3], pt[4], pt[5]

    has_vel = not (vx == 0.0 and vy == 0.0 and vz == 0.0)
    xy_mode = FlightNav.POS_VEL_MODE if has_vel else FlightNav.POS_MODE
    z_mode = FlightNav.POS_VEL_MODE if (has_vel and vz != 0.0) else FlightNav.POS_MODE

    msg.pos_xy_nav_mode = xy_mode
    msg.target_pos_x = x
    msg.target_pos_y = y
    msg.target_vel_x = vx
    msg.target_vel_y = vy

    msg.pos_z_nav_mode = z_mode
    msg.target_pos_z = z
    msg.target_vel_z = vz

    # Yaw
    yaw, omega_z = traj.get_yaw(t)
    has_yaw_vel = omega_z != 0.0
    msg.yaw_nav_mode = FlightNav.POS_VEL_MODE if has_yaw_vel else FlightNav.POS_MODE
    msg.target_yaw = yaw
    msg.target_omega_z = omega_z

    # Roll / Pitch
    roll, pitch = traj.get_rp(t)
    if roll != 0.0:
        msg.roll_nav_mode = FlightNav.POS_MODE
        msg.target_roll = roll
    if pitch != 0.0:
        msg.pitch_nav_mode = FlightNav.POS_MODE
        msg.target_pitch = pitch

    msg.target = FlightNav.COG
    msg.control_frame = FlightNav.WORLD_FRAME

    return msg


def make_stay_msg():
    """Build a STAY_HERE FlightNav message."""
    msg = FlightNav()
    msg.header.stamp = rospy.Time.now()
    msg.pos_xy_nav_mode = FlightNav.STAY_HERE_MODE
    return msg


def main():
    rospy.init_node('unified_traj_test')

    traj_name = rospy.get_param('~traj', 'lemniscate')
    loops = rospy.get_param('~loops', 1)
    settle = rospy.get_param('~settle_time', 3.0)

    # Collect optional overrides
    params = {}
    for key in ('period', 'radius', 'a', 'z_center', 'z_range', 'a_ori', 't_converge'):
        if rospy.has_param('~' + key):
            params[key] = rospy.get_param('~' + key)

    traj = build_traj(traj_name, loops, params)

    nav_pub = rospy.Publisher('/assembly/uav/nav', FlightNav, queue_size=1)
    rate = rospy.Rate(40)

    rospy.loginfo('=== Trajectory test: %s  loops=%d  T=%.1fs ===', traj_name, loops, traj.T)
    rospy.loginfo('Settling for %.1f s ...', settle)

    # --- Settle phase: send starting position ---
    t_settle_start = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - t_settle_start).to_sec()
        if elapsed >= settle:
            break
        nav_pub.publish(make_nav_msg(traj, 0.0))
        rate.sleep()

    rospy.loginfo('Starting trajectory ...')
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        t = (rospy.Time.now() - t0).to_sec()
        if traj.check_finished(t):
            break
        nav_pub.publish(make_nav_msg(traj, t))
        rate.sleep()

    # --- Post-trajectory: stay here ---
    rospy.loginfo('Trajectory complete — holding position for %.1f s', settle)
    t_end = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - t_end).to_sec()
        if elapsed >= settle:
            break
        nav_pub.publish(make_stay_msg())
        rate.sleep()

    rospy.loginfo('Done.')


if __name__ == '__main__':
    main()
