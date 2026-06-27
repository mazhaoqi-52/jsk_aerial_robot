#!/usr/bin/env python3

import sys

import rospy
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty


def main():
    rospy.init_node("force_sensor_auto_calib")

    service_name = rospy.get_param("~service_name", "cfs_sensor_calib")
    topic_name = rospy.get_param("~topic_name", "cfs/data")
    service_timeout = rospy.get_param("~service_timeout", 10.0)
    topic_timeout = rospy.get_param("~topic_timeout", 10.0)
    startup_delay = rospy.get_param("~startup_delay", 0.5)
    wait_for_sample = rospy.get_param("~wait_for_sample", True)

    try:
        rospy.loginfo("Waiting for force sensor calibration service: %s", service_name)
        rospy.wait_for_service(service_name, timeout=service_timeout)

        if wait_for_sample:
            rospy.loginfo("Waiting for first force sensor sample: %s", topic_name)
            rospy.wait_for_message(topic_name, WrenchStamped, timeout=topic_timeout)

        if startup_delay > 0.0:
            rospy.sleep(startup_delay)

        rospy.loginfo("Calling force sensor calibration service. Do not touch the sensor.")
        calibrate = rospy.ServiceProxy(service_name, Empty)
        calibrate()
        rospy.loginfo("Force sensor auto calibration completed.")
        return 0
    except rospy.ROSException as exc:
        rospy.logerr("Force sensor auto calibration timed out: %s", exc)
    except rospy.ServiceException as exc:
        rospy.logerr("Force sensor auto calibration failed: %s", exc)

    return 1


if __name__ == "__main__":
    sys.exit(main())
