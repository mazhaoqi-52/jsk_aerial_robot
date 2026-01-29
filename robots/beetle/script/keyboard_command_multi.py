#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty
import socket

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
import rosgraph
from geometry_msgs.msg import PoseStamped

class MultiPublisher:
    def __init__(self, topics, data_class, queue_size=1):
        """
        Initialize multiple publishers for a list of topics.
        Args:
            topics: List of topic names
            data_class: ROS message class
            queue_size: Queue size for publishers
        """
        self.publishers = [rospy.Publisher(topic, data_class, queue_size=queue_size) for topic in topics]
    
    def publish(self, msg):
        """Publish message to all topics."""
        for pub in self.publishers:
            pub.publish(msg)


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
        rospy.init_node("keyboard_command_multi")
        
        # Get robot namespaces from parameter
        robot_namespaces = rospy.get_param("~robot_namespaces", [])
        
        # If no parameter is set, try to auto-detect robot namespaces
        if not robot_namespaces:
                rospy.loginfo("No robot_namespaces parameter found, attempting auto-detection...")
                try:
                        master = rosgraph.Master('/rostopic')
                        _, subs, _ = master.getSystemState()
                        
                        # Find all topics with 'teleop_command/start'
                        teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
                        
                        # Extract robot namespaces from topics
                        robot_namespaces = []
                        for topic in teleop_topics:
                                # Extract namespace: /namespace/teleop_command/start -> namespace
                                ns = topic.split('/teleop_command')[0].strip('/')
                                if ns and ns not in robot_namespaces:
                                        robot_namespaces.append(ns)
                        
                        # Also check for assembly namespace (for nav control after docking)
                        nav_topics = [topic[0] for topic in subs if '/uav/nav' in topic[0]]
                        for topic in nav_topics:
                                ns = topic.split('/uav/nav')[0].strip('/')
                                if ns and ns not in robot_namespaces:
                                        robot_namespaces.append(ns)
                                        rospy.loginfo("Added namespace from nav topic: %s" % ns)
                        
                        if robot_namespaces:
                                rospy.loginfo("Auto-detected robot namespaces: %s" % str(robot_namespaces))
                        else:
                                # If auto-detection fails, use default values
                                robot_namespaces = ["beetle1", "beetle2", "assembly"]
                                rospy.logwarn("Auto-detection failed, using default namespaces: %s" % str(robot_namespaces))
                
                except socket.error:
                        rospy.logerr("Unable to communicate with ROS master!")
                        robot_namespaces = ["beetle1", "beetle2", "assembly"]
                        rospy.logwarn("Using default namespaces: %s" % str(robot_namespaces))
        else:
                rospy.loginfo("Using configured robot namespaces: %s" % str(robot_namespaces))
        
        print(msg)

        # Build topic lists dynamically based on robot namespaces
        land_topics = [ns + "/teleop_command/land" for ns in robot_namespaces]
        halt_topics = [ns + "/teleop_command/halt" for ns in robot_namespaces]
        start_topics = [ns + "/teleop_command/start" for ns in robot_namespaces]
        takeoff_topics = [ns + "/teleop_command/takeoff" for ns in robot_namespaces]
        force_landing_topics = [ns + "/teleop_command/force_landing" for ns in robot_namespaces]
        nav_topics = [ns + "/uav/nav" for ns in robot_namespaces]
        
        # Create multi-publishers
        land_pub = MultiPublisher(land_topics, Empty, queue_size=1)
        halt_pub = MultiPublisher(halt_topics, Empty, queue_size=1)
        start_pub = MultiPublisher(start_topics, Empty, queue_size=1)
        takeoff_pub = MultiPublisher(takeoff_topics, Empty, queue_size=1)
        force_landing_pub = MultiPublisher(force_landing_topics, Empty, queue_size=1)
        nav_pub = MultiPublisher(nav_topics, FlightNav, queue_size=1)

        xy_vel   = rospy.get_param("~xy_vel", 0.04)
        yaw_vel  = rospy.get_param("~yaw_vel", 0.02)
        z_vel = rospy.get_param("~z_vel", 0.04)

        motion_start_pub = MultiPublisher(['task_start'], Empty, queue_size=1)
        current_z_vel = 0.0
        
        rospy.loginfo("Keyboard command multi started. Press keys to control %d robots." % len(robot_namespaces))
        
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


