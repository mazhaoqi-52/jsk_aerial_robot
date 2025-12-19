#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
import numpy as np
import os
import sys
sys.path.append('/home/mzq0502/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/ninja/script/')
from tasks.assembly_motion import * 
from tasks.disassembly_motion import *
from ninja.assembly_api import *
from ninja.disassembly_api import *
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped
class valverotationDemo():
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node("valverotation_demo", anonymous=True)
        # rospy.init_node("valverotation_demo", anonymous=True)DesireCoor
        self.pos_ninja_1_pub = rospy.Publisher("/ninja1/uav/nav", FlightNav, queue_size=10)
        self.pos_ninja_2_pub = rospy.Publisher("/ninja2/uav/nav", FlightNav, queue_size=10)
        self.pos_assembly_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=10)
        self.rotation_pub = rospy.Publisher("/target_com_rot", DesireCoord, queue_size=10)
        self.pos_ninja_1_sub = rospy.Subscriber("/ninja1/mocap/pose", PoseStamped, queue_size=10)
        self.pos_ninja_2_sub = rospy.Subscriber("/ninja2/mocap/pose", PoseStamped, queue_size=10)
        self.flag = 0
    def main(self):
        # Step 0: StandbyState
        try:
            # demo_assemble = AssemblyDemo(module_ids = "1,2", real_machine=False)
            demo_assemble = AssembleDemo()
            outcome=demo_assemble.main() 
            time.sleep(0.1) 
            rospy.loginfo("The first step is completed!")
            # if outcome == 'succeeded':
            rospy.loginfo("AssembleDemo succeeded, moving to next step.")
            self.flag = 1  
            # else:
            #     rospy.loginfo("AssembleDemo did not succeed, stopping.")
                # return 
        except rospy.ROSInterruptException: pass
        # Step 1: Moving to gate
        if self.flag == 1:
            pos_assembly = FlightNav()
            pos_assembly.target=1
            pos_assembly.pos_xy_nav_mode = 2
            pos_assembly.target_pos_x = 5.5
            # pos_assembly.target_vel_x = 0.1
            self.pos_assembly_pub.publish(pos_assembly)
            time.sleep(4)
            rospy.loginfo("Reaching the entrance, moving to next step.")
            self.flag = 2
        # Step 2: Disassemble to pass the path
        try:
            if self.flag == 2:
                # demo_disassemble = DisassemblyDemo(module_ids = "1,2", real_machine=False);
                demo_disassemble = DisassembleDemo();
                demo_disassemble.main()
                time.sleep(3)
                self.flag = 3
                rospy.loginfo("Disassemble at the entrance, moving to next step.")
        except rospy.ROSInterruptException: pass
        # Step 3: Moving to the valve
        while self.flag == 3:
            pos_ninja_1 = FlightNav()
            pos_ninja_1.pos_xy_nav_mode = 2
            pos_ninja_1.target_pos_x = 7.0
            pos_ninja_1.target_pos_y = 0.85
            pos_ninja_2 = FlightNav()
            pos_ninja_2.pos_xy_nav_mode = 2
            pos_ninja_2.target_pos_x = 7.0
            pos_ninja_2.target_pos_y = -0.8      
            self.pos_ninja_1_pub.publish(pos_ninja_1)
            self.pos_ninja_2_pub.publish(pos_ninja_2)
            time.sleep(10)
            pos_ninja_1.pos_xy_nav_mode = 2
            pos_ninja_1.target_pos_x = 13.2
            pos_ninja_1.target_pos_y = 0.85
            self.pos_ninja_1_pub.publish(pos_ninja_1)
            # time.sleep(3)
            pos_ninja_2.pos_xy_nav_mode = 2
            pos_ninja_2.target_pos_x = 13.4
            pos_ninja_2.target_pos_y = -0.8
            self.pos_ninja_2_pub.publish(pos_ninja_2)
            time.sleep(26)
            self.flag = 4
            rospy.loginfo("Reaching the valve, moving to next step.")
        # Step 4: Assembly under the valve
        if self.flag == 4:
            try:
                pos_ninja_1.target_pos_y = 0.0
                pos_ninja_1.target_pos_x = 14.7
                # pos_ninja_2.target_pos_x = 14.5
                pos_ninja_2.target_pos_y = 0.0
                self.pos_ninja_1_pub.publish(pos_ninja_1)
                self.pos_ninja_2_pub.publish(pos_ninja_2)
                time.sleep(7)
                # demo_assemble = AssemblyDemo(module_ids = "1,2", real_machine=False)
                demo_assemble = AssembleDemo();
                demo_assemble.main()
                time.sleep(15)
                self.flag = 5
            except rospy.ROSInterruptException:
                pass
            rospy.loginfo("Assemble under the valve, moving to next step.")
        # Step 5: Excuting the valve rotation task
        while self.flag == 5:
            pos_assembly = FlightNav()
            pos_assembly.target=1
            pos_assembly.pos_z_nav_mode = 2
            pos_assembly.target_pos_z = 1.2
            rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=10).publish(pos_assembly)
            time.sleep(5)
            rotation_assembly = DesireCoord()
            rotation_assembly.yaw = 1.57
            self.rotation_pub.publish(rotation_assembly)
            time.sleep(3)
            rospy.loginfo("The valve rotation task is completed! Moving to next step.")
            self.flag = 6
        # Step 6: Disassemble to pass the path
        if self.flag == 6:
            try:
                demo_disassemble.main()
                time.sleep(0.1)
                rospy.loginfo("Disassemble at the opposite, moving to next step.")
                self.flag = 7
            except rospy.ROSInterruptException: pass
        # Step 7: Moving to the opposite gate
        while self.flag == 7:
            pos_ninja_1.pos_xy_nav_mode = 2
            pos_ninja_1.target_pos_x = 30.0
            pos_ninja_1.target_pos_y = 0.80
            pos_ninja_2.pos_xy_nav_mode = 2
            pos_ninja_2.target_pos_x = 30.0
            pos_ninja_2.target_pos_y = -0.80           
            self.pos_ninja_1_pub.publish(pos_ninja_1)
            self.pos_ninja_2_pub.publish(pos_ninja_2)
            time.sleep(20)
            self.flag = 8
            rospy.loginfo("The whole task is completed!")

        rospy.spin()
if __name__ == '__main__':
    try:
        demo = valverotationDemo();
        demo.main()
    except rospy.ROSInterruptException: pass
