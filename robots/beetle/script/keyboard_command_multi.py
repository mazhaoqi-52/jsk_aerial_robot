#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
import rosgraph

class DualPublisher:
    def __init__(self, topic1, topic2, data_class, queue_size=1):
        self.pub1 = rospy.Publisher(topic1, data_class, queue_size=queue_size)
        self.pub2 = rospy.Publisher(topic2, data_class, queue_size=queue_size)
    def publish(self, msg):
        self.pub1.publish(msg)
        self.pub2.publish(msg)


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
        # robot_ns = rospy.get_param("~robot_ns", "");
        robot_ns_1 = "beetle1"
        robot_ns_2 = "beetle2"
        print(msg)

        # if not robot_ns:
        #         master = rosgraph.Master('/rostopic')
        #         try:
        #                 _, subs, _ = master.getSystemState()

        #         except socket.error:
        #                 raise ROSTopicIOException("Unable to communicate with master!")

        #         teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
        #         if len(teleop_topics) == 1:
        #                 robot_ns = teleop_topics[0].split('/teleop')[0]

        ns_1 = robot_ns_1 + "/teleop_command"
        ns_2 = robot_ns_2 + "/teleop_command"
        land_pub = DualPublisher(ns_1 + '/land',ns_2 + '/land', Empty, queue_size=1)
        halt_pub = DualPublisher(ns_1 + '/halt',ns_2 +'/halt', Empty, queue_size=1)
        start_pub = DualPublisher(ns_1 + '/start',ns_2 + '/start', Empty, queue_size=1)
        takeoff_pub = DualPublisher(ns_1 + '/takeoff',ns_2 + '/takeoff', Empty, queue_size=1)
        force_landing_pub = DualPublisher(ns_1 + '/force_landing',ns_2 + '/force_landing', Empty, queue_size=1)
        nav_pub = DualPublisher(robot_ns_1 + '/uav/nav',robot_ns_2 + '/uav/nav', FlightNav, queue_size=1)

        xy_vel   = rospy.get_param("xy_vel", 0.2)
        z_vel    = rospy.get_param("z_vel", 0.2)
        yaw_vel  = rospy.get_param("yaw_vel", 0.2)

        # motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)
        motion_start_pub   = DualPublisher('task_start', 'task_start', Empty, queue_size=1)
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


