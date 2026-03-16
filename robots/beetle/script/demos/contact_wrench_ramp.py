#!/usr/bin/env python3
"""
contact_wrench_ramp.py
Ramp-up (and optional ramp-down) of formation_desired_wrench for unified-mode
contact manipulation tasks (pulling cart, rotating valve, steady push).

Usage examples
--------------
# Ramp Fx to 7 N over 3 s, hold 10 s, then ramp back to 0:
  rosrun beetle contact_wrench_ramp.py _fx:=7.0 _ramp_time:=3.0 _hold_time:=10.0 _leader:=beetle1

# Apply yaw torque for valve rotation (10 N·m, hold indefinitely until Ctrl-C):
  rosrun beetle contact_wrench_ramp.py _tz:=10.0 _ramp_time:=2.0 _hold_time:=-1 _leader:=beetle1

Parameters (all optional, see defaults below)
----------------------------------------------
  _leader      : leader robot name (default: beetle1)
  _fx,_fy,_fz  : target force components in leader body frame [N]  (default: 0)
  _tx,_ty,_tz  : target torque components in leader body frame [Nm] (default: 0)
  _ramp_time   : seconds to ramp from 0 to target                   (default: 2.0)
  _hold_time   : seconds to hold at target; -1 = hold until Ctrl-C  (default: -1)
  _rate_hz     : publish rate [Hz]                                   (default: 40)

Topic published
---------------
  /{leader}/formation_desired_wrench  (geometry_msgs/WrenchStamped, body frame)
"""

import math
import signal
import sys

import rospy
from geometry_msgs.msg import WrenchStamped


def make_msg(fx, fy, fz, tx, ty, tz):
    msg = WrenchStamped()
    msg.header.frame_id = "fc"
    msg.wrench.force.x  = fx
    msg.wrench.force.y  = fy
    msg.wrench.force.z  = fz
    msg.wrench.torque.x = tx
    msg.wrench.torque.y = ty
    msg.wrench.torque.z = tz
    return msg


def publish_scaled(pub, scale, fx, fy, fz, tx, ty, tz):
    msg = make_msg(
        scale * fx, scale * fy, scale * fz,
        scale * tx, scale * ty, scale * tz
    )
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)


def publish_zero(pub, n=10, interval=0.04):
    """Publish zero wrench several times to guarantee the controller sees it."""
    for _ in range(n):
        msg = make_msg(0, 0, 0, 0, 0, 0)
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.sleep(interval)


def main():
    rospy.init_node("contact_wrench_ramp", anonymous=True)

    leader    = rospy.get_param("~leader",     "beetle1")
    fx_target = rospy.get_param("~fx",          0.0)
    fy_target = rospy.get_param("~fy",          0.0)
    fz_target = rospy.get_param("~fz",          0.0)
    tx_target = rospy.get_param("~tx",          0.0)
    ty_target = rospy.get_param("~ty",          0.0)
    tz_target = rospy.get_param("~tz",          0.0)
    ramp_time = rospy.get_param("~ramp_time",   2.0)
    hold_time = rospy.get_param("~hold_time",  -1.0)   # -1 = indefinite
    rate_hz   = rospy.get_param("~rate_hz",    40.0)

    topic = f"/{leader}/formation_desired_wrench"
    pub   = rospy.Publisher(topic, WrenchStamped, queue_size=1)
    rate  = rospy.Rate(rate_hz)

    # Allow publisher to register before we start sending
    rospy.sleep(0.5)

    target_norm = math.sqrt(fx_target**2 + fy_target**2 + fz_target**2 +
                            tx_target**2 + ty_target**2 + tz_target**2)
    if target_norm < 1e-9:
        rospy.logwarn("[contact_wrench_ramp] All wrench components are zero — nothing to do.")
        return

    rospy.loginfo(
        "[contact_wrench_ramp] Publishing to %s\n"
        "  target: F=(%.2f,%.2f,%.2f) N  T=(%.2f,%.2f,%.2f) Nm\n"
        "  ramp_time=%.1fs  hold_time=%s",
        topic,
        fx_target, fy_target, fz_target,
        tx_target, ty_target, tz_target,
        ramp_time,
        f"{hold_time:.1f}s" if hold_time >= 0 else "indefinite (Ctrl-C to stop)"
    )

    # --- Phase 1: Ramp up ---
    rospy.loginfo("[contact_wrench_ramp] Phase 1: Ramp up over %.1f s", ramp_time)
    ramp_steps = max(1, int(ramp_time * rate_hz))
    for step in range(ramp_steps):
        if rospy.is_shutdown():
            break
        scale = (step + 1) / ramp_steps
        publish_scaled(pub, scale, fx_target, fy_target, fz_target,
                       tx_target, ty_target, tz_target)
        rate.sleep()

    if rospy.is_shutdown():
        publish_zero(pub)
        return

    # --- Phase 2: Hold ---
    if hold_time < 0:
        rospy.loginfo("[contact_wrench_ramp] Phase 2: Holding at full wrench (Ctrl-C to release)")
        while not rospy.is_shutdown():
            publish_scaled(pub, 1.0, fx_target, fy_target, fz_target,
                           tx_target, ty_target, tz_target)
            rate.sleep()
    else:
        rospy.loginfo("[contact_wrench_ramp] Phase 2: Holding for %.1f s", hold_time)
        hold_start = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - hold_start).to_sec()
            if elapsed >= hold_time:
                break
            publish_scaled(pub, 1.0, fx_target, fy_target, fz_target,
                           tx_target, ty_target, tz_target)
            rate.sleep()

    # --- Phase 3: Ramp down ---
    if not rospy.is_shutdown():
        rospy.loginfo("[contact_wrench_ramp] Phase 3: Ramp down")
        ramp_steps = max(1, int(ramp_time * rate_hz))
        for step in range(ramp_steps):
            if rospy.is_shutdown():
                break
            scale = 1.0 - (step + 1) / ramp_steps
            publish_scaled(pub, scale, fx_target, fy_target, fz_target,
                           tx_target, ty_target, tz_target)
            rate.sleep()

    # Always publish zero on exit
    publish_zero(pub)
    rospy.loginfo("[contact_wrench_ramp] Done — wrench cleared.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
