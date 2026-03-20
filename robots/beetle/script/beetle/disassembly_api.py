#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math,time,logging
from std_msgs.msg import Empty, String, Bool
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import ServoControlCmd
from diagnostic_msgs.msg import KeyValue
from beetle.dynamixel_control_api import DynamixelControl
import numpy as np
import tf
from beetle.gazebo_link_attacher import GazeboLinkAttacher
from beetle.gazebo_link_detacher import GazeboLinkDetacher
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

#### state classes ####

"""""""""""""""""""""""""""
Switch -> Separate -> Stop
"""""""""""""""""""""""""""

class Filter(logging.Filter):
    def filter(self, record):
        return 'State machine transitioning' not in record.msg

class SwitchState(smach.State):
    # release docking mechanism and switch control mode
    def __init__(self,
                 robot_name = 'beetle1',
                 robot_id = 1,
                 male_servo_id = 4,
                 real_machine = False,
                 unlock_servo_angle_male = 1820,
                 lock_servo_angle_male = 1580,
                 unlock_servo_angle_female = 2000,
                 lock_servo_angle_female = 3200,
                 neighboring = 'beetle2',
                 neighboring_id = 2,
                 female_servo_id = 5,
                 separate_dir = -1):
        smach.State.__init__(self, outcomes=['done','emergency'])

        self.robot_name = robot_name 
        self.robot_id = robot_id 
        self.male_servo_id = male_servo_id
        self.real_machine = real_machine
        self.unlock_servo_angle_male = unlock_servo_angle_male
        self.lock_servo_angle_male = lock_servo_angle_male
        self.unlock_servo_angle_female = unlock_servo_angle_female
        self.lock_servo_angle_female = lock_servo_angle_female
        self.neighboring = neighboring
        self.neighboring_id = neighboring_id
        self.female_servo_id = female_servo_id
        self.separate_dir = separate_dir
        if(separate_dir > 0):
            self.dynamixel_servo = DynamixelControl(self.robot_name,self.robot_id,self.female_servo_id,self.real_machine)
            self.dynamixel_servo_neighboring = DynamixelControl(self.neighboring,self.neighboring_id,self.male_servo_id,self.real_machine)
        else:
            self.dynamixel_servo = DynamixelControl(self.robot_name,self.robot_id,self.male_servo_id,self.real_machine)
            self.dynamixel_servo_neighboring = DynamixelControl(self.neighboring,self.neighboring_id,self.female_servo_id,self.real_machine)
            
        self.flag_pub = rospy.Publisher('/' + self.robot_name + '/assembly_flag', KeyValue, queue_size = 1)
        self.flag_pub_neighboring = rospy.Publisher('/' + self.neighboring + '/assembly_flag', KeyValue, queue_size = 1)

        if(separate_dir > 0):
            self.docking_pub = rospy.Publisher('/' + self.neighboring  + '/docking_cmd', Bool, queue_size = 1)
        else:
            self.docking_pub = rospy.Publisher('/' + self.robot_name  + '/docking_cmd', Bool, queue_size = 1)

        # subscriber
        self.emergency_stop_sub = rospy.Subscriber("/emergency_assembly_interuption",Empty,self.emergencyCb)

        #messeges
        self.flag_msg = KeyValue()
        self.docking_msg = Bool()
        self.emergency_flag = False
        time.sleep(0.5)

    def execute(self, userdata):
        if self.emergency_flag:
            return 'emergency'
        if not self.real_machine:
            try:
                link_detacher = GazeboLinkDetacher(self.neighboring, 'root', self.robot_name, 'root')
                link_detacher.detach_links()
            except rospy.ServiceException:
                rospy.loginfo("Dettacher failed")
        time.sleep(1)
        # Notify both robots to switch to single mode
        self.flag_msg.key = str(self.robot_id)
        self.flag_msg.value = '0'
        self.flag_pub.publish(self.flag_msg)
        self.flag_msg.key = str(self.neighboring_id)
        self.flag_msg.value = '0'
        self.flag_pub_neighboring.publish(self.flag_msg)
        rospy.loginfo("[SwitchState] assembly_flag=0 published for both robots")
        if self.real_machine:
            if(self.separate_dir > 0):
                self.dynamixel_servo.sendTargetAngle(self.unlock_servo_angle_female)
                self.dynamixel_servo_neighboring.sendTargetAngle(self.unlock_servo_angle_male)
            else:
                self.dynamixel_servo.sendTargetAngle(self.unlock_servo_angle_male)
                self.dynamixel_servo_neighboring.sendTargetAngle(self.unlock_servo_angle_female)
        else:
            self.docking_msg.data = False
            self.docking_pub.publish(self.docking_msg)
        time.sleep(5.0)
        return 'done'

    def emergencyCb(self,msg):
        self.emergency_flag = True

class SeparateState(smach.State):
    # keep away target robot from leader
    def __init__(self,
                 robot_name = 'beetle1',
                 robot_id = 1,
                 separate_vel = -0.5,
                 neighboring = 'beetle2',
                 target_dist_from_neighboring = 1.25):

        smach.State.__init__(self, outcomes=['done','in_process','emergency'])

        self.emergency_flag = False
        self.run_rate = rospy.Rate(40)

        self.robot_name = robot_name
        self.robot_id = robot_id
        self.separate_vel = separate_vel
        self.neighboring = neighboring
        self.target_dist_from_neighboring = target_dist_from_neighboring

        # tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # publisher
        self.nav_pub = rospy.Publisher('/' + self.robot_name+"/uav/nav", FlightNav, queue_size=10)
        self.nav_pub_neighboring = rospy.Publisher('/' + self.neighboring+"/uav/nav", FlightNav, queue_size=10)

        # subscriber
        self.emergency_stop_sub = rospy.Subscriber("/emergency_assembly_interuption",Empty,self.emergencyCb)

        #messages
        self.nav_msg = FlightNav()

    def execute(self, userdata):
        if self.emergency_flag:
            return 'emergency'
        x_dist = 0
        try:
            tf_from_neighboring = self.listener.lookupTransform('/' + self.neighboring+'/root', '/' + self.robot_name+'/root', rospy.Time(0))
            x_dist = math.fabs(tf_from_neighboring[0][0])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            x_dist = 0
            rospy.loginfo('out')
            return 'in_process'
        if x_dist <= self.target_dist_from_neighboring:
            self.nav_msg.target = 1
            self.nav_msg.control_frame = 1 #local frame
            self.nav_msg.pos_xy_nav_mode= 1
            self.nav_msg.target_vel_x = self.separate_vel
            self.nav_pub.publish(self.nav_msg)
            self.run_rate.sleep()
            return 'in_process'
        else:
            self.nav_msg.pos_xy_nav_mode= 6
            self.nav_pub.publish(self.nav_msg)
            return 'done'

    def emergencyCb(self,msg):
        self.emergency_flag = True
