#!/usr/bin/env python3
"""Prepare recorded static TF and convert the mixed-frame debug wrench for RViz."""
import collections
import copy
import threading
import time

import numpy as np
import rosbag
import rospy
import tf2_ros
from geometry_msgs.msg import WrenchStamped
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker


def force_in_cog(force_world, rotation_world_from_cog):
    """Rotate a free vector; torque already refers to the CoG origin and axes."""
    q = rotation_world_from_cog
    rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    f = force_world
    return rotation.T.dot([f.x, f.y, f.z])


def main():
    rospy.init_node("rosbag_wrench_viz")
    bag_path = rospy.get_param("~bag")
    # Multiple recorded latched /tf_static publishers must be combined. This also
    # restores transforms when rosbag play starts partway through the recording.
    static = {}
    with rosbag.Bag(bag_path) as bag:
        bag_start = bag.get_start_time()
        for _, msg, _ in bag.read_messages(topics=["/tf_static"]):
            for transform in msg.transforms:
                previous = static.get(transform.child_frame_id)
                if previous and (previous.header.frame_id != transform.header.frame_id
                                 or previous.transform != transform.transform):
                    raise RuntimeError("Changing static TF: " + transform.child_frame_id)
                transform.header.stamp = rospy.Time(0)
                static[transform.child_frame_id] = transform
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(list(static.values()))
    if not rospy.get_param("~interaction_wrench", False):
        rospy.loginfo("Loaded %d static transforms; original RViz topics unchanged", len(static))
        rospy.spin()
        return
    buffer = tf2_ros.Buffer(rospy.Duration(30))
    listener = tf2_ros.TransformListener(buffer)
    pub = rospy.Publisher("/beetle1/rosbag_viz/interaction_wrench", WrenchStamped, queue_size=10)
    labels = rospy.Publisher("/beetle1/rosbag_viz/wrench_label", Marker, queue_size=10)
    pending = collections.deque(maxlen=200)
    lock = threading.Lock()

    def receive(msg):
        with lock:
            pending.append((msg, time.monotonic()))

    subscriber = rospy.Subscriber("/beetle1/dist_w_f_cog_tq/ext", WrenchStamped, receive, queue_size=100)
    rospy.loginfo("Loaded %d static transforms; force: world -> cog, torque: cog", len(static))
    # Wall time keeps this worker alive when playback is paused. Wait for TF at
    # the message timestamp, never silently substitute the latest orientation.
    while not rospy.is_shutdown():
        with lock:
            item = pending[0] if pending else None
        if item:
            msg, arrival = item
            try:
                transform = buffer.lookup_transform("world", "beetle1/cog", msg.header.stamp)
                output = copy.deepcopy(msg)
                output.header.frame_id = "beetle1/cog"
                force = force_in_cog(msg.wrench.force, transform.transform.rotation)
                output.wrench.force.x, output.wrench.force.y, output.wrench.force.z = force
                pub.publish(output)
                label = Marker()
                label.header = copy.deepcopy(output.header)
                label.header.frame_id = "world"
                label.ns = "interaction_wrench"
                label.id = 0
                label.type = Marker.TEXT_VIEW_FACING
                label.pose.orientation.w = 1
                position = transform.transform.translation
                label.pose.position.x = position.x
                label.pose.position.y = position.y
                label.pose.position.z = position.z + 0.65
                label.scale.z = 0.10
                label.color.r = label.color.g = label.color.b = label.color.a = 1
                f, t = msg.wrench.force, msg.wrench.torque
                label.text = ("bag t=%.2fs |F|=%.2f N\nF(world)=[%.1f, %.1f, %.1f] N\n"
                              "T(cog)=[%.2f, %.2f, %.2f] Nm") % (
                    msg.header.stamp.to_sec() - bag_start, np.linalg.norm(force),
                    f.x, f.y, f.z, t.x, t.y, t.z)
                labels.publish(label)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
                if time.monotonic() - arrival < 2:
                    time.sleep(0.01)
                    continue
                rospy.logwarn_throttle(5, "Skipping wrench without timestamp-matched TF: %s", exc)
            with lock:
                if pending and pending[0] is item:
                    pending.popleft()
        else:
            time.sleep(0.01)


if __name__ == "__main__":
    main()
