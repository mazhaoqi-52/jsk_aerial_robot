#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
import rosgraph
from geometry_msgs.msg import PoseStamped

class MultiPublisher:
    def __init__(self, topic1, topic2, topic3, topic4, topic5, data_class, queue_size=1):
        self.pub1 = rospy.Publisher(topic1, data_class, queue_size=queue_size)
        self.pub2 = rospy.Publisher(topic2, data_class, queue_size=queue_size)
        self.pub3 = rospy.Publisher(topic3, data_class, queue_size=queue_size)
        self.pub4 = rospy.Publisher(topic4, data_class, queue_size=queue_size)
        self.pub5 = rospy.Publisher(topic5, data_class, queue_size=queue_size)
    def publish(self, msg):
        self.pub1.publish(msg)
        self.pub2.publish(msg)
        self.pub3.publish(msg)
        self.pub4.publish(msg)
        self.pub5.publish(msg)


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
        robot_ns_3 = "beetle3"
        robot_ns_4 = "beetle4"
        robot_ns_assemble = "assembly"
        print(msg)

        ns_1 = robot_ns_1 + "/teleop_command"
        ns_2 = robot_ns_2 + "/teleop_command"
        ns_3 = robot_ns_3 + "/teleop_command"
        ns_4 = robot_ns_4 + "/teleop_command"
        ns_assemble = robot_ns_assemble + "/teleop_command"
        land_pub = MultiPublisher(ns_1 + '/land', ns_2 + '/land', ns_3 + '/land', ns_4 + '/land', ns_assemble + '/land', Empty, queue_size=1)
        halt_pub = MultiPublisher(ns_1 + '/halt', ns_2 +'/halt', ns_3 +'/halt', ns_4 +'/halt', ns_assemble +'/halt', Empty, queue_size=1)
        start_pub = MultiPublisher(ns_1 + '/start', ns_2 + '/start',ns_3 + '/start', ns_4 + '/start', ns_assemble + '/start', Empty, queue_size=1)
        takeoff_pub = MultiPublisher(ns_1 + '/takeoff', ns_2 + '/takeoff', ns_3 + '/takeoff', ns_4 + '/takeoff', ns_assemble + '/takeoff', Empty, queue_size=1)
        force_landing_pub = MultiPublisher(ns_1 + '/force_landing', ns_2 + '/force_landing', ns_3 + '/force_landing', ns_4 + '/force_landing', ns_assemble + '/force_landing', Empty, queue_size=1)
        nav_pub = MultiPublisher(robot_ns_1 + '/uav/nav', robot_ns_2 + '/uav/nav', robot_ns_3 + '/uav/nav', robot_ns_4 + '/uav/nav', robot_ns_assemble + '/uav/nav', FlightNav, queue_size=1)

        xy_vel   = rospy.get_param("xy_vel", 0.04)
        yaw_vel  = rospy.get_param("yaw_vel", 0.02)
        z_vel = rospy.get_param("z_vel", 0.04)

        motion_start_pub   = MultiPublisher('task_start', 'task_start', 'task_start', 'task_start', 'task_start', Empty, queue_size=1)
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

                        if key == '[':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = z_vel
                                nav_pub.publish(nav_msg)
                                msg = "send +z vel command"
                        if key == ']':
                                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                nav_msg.target_vel_z = -z_vel
                                nav_pub.publish(nav_msg)
                                msg = "send -z vel command"
                        if key == '\x03':
                                break

                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


