#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
import rosgraph
from geometry_msgs.msg import PoseStamped

current_pose_z = None      
commanded_altitude = None  

def pose_callback(msg):
    global current_pose_z, commanded_altitude
    current_pose_z = msg.pose.position.z
    if commanded_altitude is None:
        commanded_altitude = current_pose_z
        rospy.loginfo("Initial commanded altitude set to current z: %.2f", commanded_altitude)

class TriPublisher:
    def __init__(self, topic1, topic2, topic3, data_class, queue_size=1):
        self.pub1 = rospy.Publisher(topic1, data_class, queue_size=queue_size)
        self.pub2 = rospy.Publisher(topic2, data_class, queue_size=queue_size)
        self.pub3 = rospy.Publisher(topic3, data_class, queue_size=queue_size)
    def publish(self, msg):
        self.pub1.publish(msg)
        self.pub2.publish(msg)
        self.pub3.publish(msg)


msg = """
Instruction:

---------------------------

r:  arming motor (please do before takeoff)
t:  takeoff
l:  land
f:  force landing
h:  halt (force stop motor)

     q           w           e           [
(turn left)  (forward)  (turn right)  (move up)

     a           s           d           ]
(move left)  (backward) (move right) (move down)


Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def printMsg(msg, msg_len = 50):
        print(msg.ljust(msg_len) + "\r", end="")

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("keyboard_command")
        robot_ns_1 = "beetle1"
        robot_ns_2 = "beetle2"
        robot_ns_3 = "assembly"
        print(msg)

        rospy.Subscriber("/beetle1/mocap/pose", PoseStamped, pose_callback)
        ns_1 = robot_ns_1 + "/teleop_command"
        ns_2 = robot_ns_2 + "/teleop_command"
        ns_3 = robot_ns_3 + "/teleop_command"
        land_pub = TriPublisher(ns_1 + '/land', ns_2 + '/land', ns_3 + '/land', Empty, queue_size=1)
        halt_pub = TriPublisher(ns_1 + '/halt', ns_2 +'/halt', ns_3 +'/halt', Empty, queue_size=1)
        start_pub = TriPublisher(ns_1 + '/start', ns_2 + '/start',ns_3 + '/start', Empty, queue_size=1)
        takeoff_pub = TriPublisher(ns_1 + '/takeoff', ns_2 + '/takeoff', ns_3 + '/takeoff', Empty, queue_size=1)
        force_landing_pub = TriPublisher(ns_1 + '/force_landing', ns_2 + '/force_landing', ns_3 + '/force_landing', Empty, queue_size=1)
        nav_pub = TriPublisher(robot_ns_1 + '/uav/nav', robot_ns_2 + '/uav/nav', robot_ns_3 + '/uav/nav', FlightNav, queue_size=1)

        xy_vel   = rospy.get_param("xy_vel", 0.1)
        z_step  = 0.1
        yaw_vel  = rospy.get_param("yaw_vel", 0.1)

        motion_start_pub   = TriPublisher('task_start', 'task_start', 'task_start', Empty, queue_size=1)
        current_z_vel = 0.0
        try:
                while(True):
                        nav_msg = FlightNav()
                        nav_msg.control_frame = FlightNav.WORLD_FRAME
                        nav_msg.target = FlightNav.COG

                        key = getKey()

                        msg = ""

                        if key == 'l':
                                land_pub.publish(Empty())
                                msg = "send land command"
                        if key == 'r':
                                start_pub.publish(Empty())
                                msg = "send motor-arming command"
                        if key == 'h':
                                halt_pub.publish(Empty())
                                msg = "send motor-disarming (halt) command"
                        if key == 'f':
                                force_landing_pub.publish(Empty())
                                msg = "send force landing command"
                        if key == 't':
                                takeoff_pub.publish(Empty())
                                msg = "send takeoff command"
                        if key == 'x':
                                motion_start_pub.publish()
                                msg = "send task-start command"
                        if key == 'w':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +x vel command"
                        if key == 's':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_x = -xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -x vel command"
                        if key == 'a':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +y vel command"
                        if key == 'd':
                                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_y = -xy_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -y vel command"
                        if key == 'q':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = yaw_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +yaw vel command"
                        if key == 'e':
                                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_omega_z = -yaw_vel
                                msg = "send -yaw vel command"
                                nav_pub.publish(nav_msg)
                        if current_pose_z is None:
                                current_pose_z = 0.0

                        if key == '[':
                                if current_pose_z is not None:
                                        commanded_altitude = current_pose_z + z_step
                                        msg_str = "set altitude to: {:.2f}".format(commanded_altitude)
                                if commanded_altitude is not None:
                                        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                                        nav_msg.target_pos_z = commanded_altitude
                                        nav_pub.publish(nav_msg)
                        if key == ']':
                                if current_pose_z is not None:
                                        commanded_altitude = current_pose_z - z_step
                                        msg_str = "set altitude to: {:.2f}".format(commanded_altitude)
                                if commanded_altitude is not None:
                                        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                                        nav_msg.target_pos_z = commanded_altitude
                                        nav_pub.publish(nav_msg)


                        if key == '\x03':
                                break

                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)