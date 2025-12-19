#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

import rospy
import smach
import smach_ros
import time
import math
import threading
import numpy as np
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from task.assembly_motion import *
from task.disassembly_motion import *
from valve_rotation_demo.trajectory import PolynomialTrajectory
from valve_rotation_demo.motion_controller import *
# from valve_rotation_demo.base_UAV_state import AssemblyMotionStateBase

class AssembleState(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Module IDs
        modules_str = rospy.get_param("~module_ids", "")
        real_machine = rospy.get_param("~real_machine", True)
        modules = []
        if modules_str:
            modules = [int(x) for x in modules_str.split(',')]
        else:
            rospy.logerr("No module ID is designated!")

        # AssembleDemo 
        self.assemble_demo = AssemblyDemo(module_ids=modules, real_machine=real_machine)
    
    def execute(self, userdata):
        rospy.loginfo("Assembling UAVs...")
        try:
            self.assemble_demo.main()
            rospy.loginfo("Assembly completed successfully.")
            return 'succeeded'
        except rospy.ROSInterruptException:
            rospy.logerr("Assembly process failed.")
            return 'failed'



class MoveAndRotateValveState(smach.State):
    def __init__(self,
                 z_offset_real = 0.21,# 0.21(when use the real valve instead of the valve_fake),0.47(when use the valve_fake)
                 z_offset_sim = 0.23,
                 yaw_offset = math.pi / 8.0,
                 valve_rotation_angle_compenstation = math.pi / 10.0,
                 valve_rotation_angle = math.pi / 2.0,
                 avg_speed = 0.15,
                 avg_yaw_speed = 0.15,
                 avg_valve_rotation_speed = 0.3,
                 pos_initialization = False,
                 exit_target = [4.0, 0.0, 1.0]):
        
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self.z_offset_real = z_offset_real
        self.z_offset_sim = z_offset_sim
        self.yaw_offset = yaw_offset
        self.valve_rotation_angle_compenstation = valve_rotation_angle_compenstation
        self.valve_rotation_angle = valve_rotation_angle + self.valve_rotation_angle_compenstation
        self.avg_speed = avg_speed
        self.avg_yaw_speed = avg_yaw_speed
        self.avg_valve_rotation_speed = avg_valve_rotation_speed
        self.pos_initialization = pos_initialization
        self.exit_target = exit_target

        # Simulation flag
        self.is_simulation = rospy.get_param("~simulation", True)

        # Module IDs
        module_ids_str = rospy.get_param("~module_ids", "1,2")
        rospy.loginfo("SeparatedMoveToGateState: Module IDs: %s" % module_ids_str)
        self.module_ids = [int(x) for x in module_ids_str.split(',')]
        if len(self.module_ids) < 2:
            rospy.logerr("SeparatedMoveToGateState: At least 2 module IDs are required.")

        # Subscribers
        self.beetle1_sub_topic = "/beetle{}/mocap/pose".format(self.module_ids[0])
        self.beetle2_sub_topic = "/beetle{}/mocap/pose".format(self.module_ids[1])
        rospy.Subscriber(self.beetle1_sub_topic, PoseStamped, self.beetle1_callback, queue_size=1)
        rospy.Subscriber(self.beetle2_sub_topic, PoseStamped, self.beetle2_callback, queue_size=1)
        if self.is_simulation:
            self.pos_valve_sim_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            self.pos_valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)

        self.pos_beetle2 = PoseStamped()
        self.pos_beetle1 = PoseStamped()
        self.pos_valve = PoseStamped()
        self.pos_valve_sim = Odometry()
        self.beetle1_received = threading.Event()
        self.beetle2_received = threading.Event()
        self.valve_received = threading.Event()

        # Publisher
        self.pos_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=1)
        self.wait_for_initialization(timeout=10)
        
    def beetle2_callback(self, msg):
        self.pos_beetle2 = msg
        self.beetle2_received.set()

    def beetle1_callback(self, msg):
        self.pos_beetle1 = msg
        self.beetle1_received.set()
    
    def valve_callback(self, msg):
        self.pos_valve = msg
        self.pos_valve_sub.unregister()
        self.valve_received.set()
    
    def valve_sim_callback(self, msg):
        self.pos_valve_sim = msg
        self.pos_valve_sim_sub.unregister()
        self.valve_received.set()

    def wait_for_initialization(self, timeout=10):
        rospy.sleep(0.5)
        rospy.logwarn("Waiting for position messages...")
        events = [self.beetle1_received, self.beetle2_received, self.valve_received]
        all_received = all(event.wait(timeout) for event in events)
        if all_received:
            rospy.loginfo("All positions received, initializing...")
            self.pos_initialize()
            self.pos_initialization = True
        else:
            rospy.logwarn("Timeout waiting for position messages.")
            self.pos_initialization = False

    def update_current_pos(self):
        self.current_pos = PoseStamped()
        self.current_pos.pose.position.x = (self.pos_beetle2.pose.position.x + self.pos_beetle1.pose.position.x) / len(self.module_ids)
        self.current_pos.pose.position.y = (self.pos_beetle2.pose.position.y + self.pos_beetle1.pose.position.y) / len(self.module_ids)
        self.current_pos.pose.position.z = (self.pos_beetle2.pose.position.z + self.pos_beetle1.pose.position.z) / len(self.module_ids)
    def pos_initialize(self):

        self.start_x = (self.pos_beetle2.pose.position.x + self.pos_beetle1.pose.position.x) / len(self.module_ids)
        self.start_y = (self.pos_beetle2.pose.position.y + self.pos_beetle1.pose.position.y) / len(self.module_ids)
        self.start_z = (self.pos_beetle2.pose.position.z + self.pos_beetle1.pose.position.z) / len(self.module_ids)
        if self.is_simulation:
            self.valve_x = self.pos_valve_sim.pose.pose.position.x
            self.valve_y = self.pos_valve_sim.pose.pose.position.y
            self.valve_z = self.pos_valve_sim.pose.pose.position.z
        else:
            self.valve_x = self.pos_valve.pose.position.x
            self.valve_y = self.pos_valve.pose.position.y
            self.valve_z = self.pos_valve.pose.position.z
        self.update_current_pos()
        rospy.loginfo(f"start position is [{self.start_x}, {self.start_y}, {self.start_z}]")
        rospy.loginfo(f"valve position is [{self.valve_x}, {self.valve_y}, {self.valve_z}]")
        self.pos_initialization = True
    
    def get_uav_yaw(self):
        orientation_q1 = self.pos_beetle1.pose.orientation
        orientation_q2 = self.pos_beetle2.pose.orientation
        quaternion1 = (orientation_q1.x, orientation_q1.y, orientation_q1.z, orientation_q1.w)
        quaternion2 = (orientation_q2.x, orientation_q2.y, orientation_q2.z, orientation_q2.w)
        _, _, yaw1 = euler_from_quaternion(quaternion1)
        _, _, yaw2 = euler_from_quaternion(quaternion2)
        return (yaw1 + yaw2) / 2.0

    def pos_check(self, tolerance, checkpoint_x, checkpoint_y, checkpoint_z):
        self.update_current_pos()  
        error_x = abs(self.current_pos.pose.position.x - checkpoint_x)
        error_y = abs(self.current_pos.pose.position.y - checkpoint_y)
        error_z = abs(self.current_pos.pose.position.z - checkpoint_z)
        rospy.loginfo(f"Checking position - Errors: x={error_x}, y={error_y}, z={error_z}")
        if error_x < tolerance and error_y < tolerance and error_z < tolerance:
            rospy.loginfo("Successfully went to the position.")
            return True
        else:
            rospy.logwarn("Failed to went to the position.")
            return False

    def move_to_target_poly(self, target_x, target_y, target_z, avg_speed=None, max_correction_duration=5):
        if avg_speed is None:
            avg_speed = self.avg_speed
        self.update_current_pos()
        start_pos = (
            self.current_pos.pose.position.x,
            self.current_pos.pose.position.y,
            self.current_pos.pose.position.z
        )
        target_pos = (target_x, target_y, target_z)
        distance = math.sqrt((target_x - start_pos[0])**2 +
                             (target_y - start_pos[1])**2 +
                             (target_z - start_pos[2])**2)
        if avg_speed <= 0:
            avg_speed = 0.1
        duration = distance / avg_speed
        rospy.loginfo(f"Moving from {start_pos} to {target_pos} over duration {duration:.2f} s (avg_speed = {avg_speed} m/s).")
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target_pos)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            pos = traj.evaluate()
            if pos is None:
                break
            pos_cmd = FlightNav()
            pos_cmd.target = 1
            pos_cmd.pos_xy_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_x = pos[0]
            pos_cmd.target_pos_y = pos[1]
            pos_cmd.pos_z_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_z = pos[2]
            self.pos_pub.publish(pos_cmd)
            rate.sleep()
        rospy.loginfo("Reached target position.")
        tolerance = 0.1  
        k_p = 0.2        
        start_correction_time = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown():
          
            self.update_current_pos()
            current_pos = (
                self.current_pos.pose.position.x,
                self.current_pos.pose.position.y,
                self.current_pos.pose.position.z
            )
            error_x = target_x - current_pos[0]
            error_y = target_y - current_pos[1]
            error_z = target_z - current_pos[2]
            error_norm = math.sqrt(error_x**2 + error_y**2 + error_z**2)
            rospy.loginfo(f"Closed-loop correction: current error norm = {error_norm:.3f}")

            if error_norm < tolerance:
                rospy.loginfo("Terminal position correction complete. Error within tolerance.")
                break

            correction_x = k_p * error_x
            correction_y = k_p * error_y
            correction_z = k_p * error_z

            pos_cmd = FlightNav()
            pos_cmd.target = 1
            pos_cmd.pos_xy_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_x = current_pos[0] + correction_x
            pos_cmd.target_pos_y = current_pos[1] + correction_y
            pos_cmd.pos_z_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_z = current_pos[2] + correction_z
            self.pos_pub.publish(pos_cmd)
            rate.sleep()

            if rospy.Time.now().to_sec() - start_correction_time > max_correction_duration:
                rospy.logwarn("Terminal correction exceeded maximum duration.")
                break

    def rotate_to_target_poly(self, target_yaw, avg_yaw_speed=None, fixed_x=None, fixed_y=None, fixed_z=None, max_correction_duration=5):
        if avg_yaw_speed is None:
            avg_yaw_speed = self.avg_yaw_speed
        current_yaw = self.get_uav_yaw()
        yaw_diff = target_yaw - current_yaw
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
        duration = abs(yaw_diff) / avg_yaw_speed if avg_yaw_speed > 0 else 0.1
        rospy.loginfo(f"Rotating from yaw {current_yaw:.4f} to {target_yaw:.4f} (diff={yaw_diff:.4f}) over duration {duration:.2f} s (avg_yaw_speed = {avg_yaw_speed} rad/s).")
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(current_yaw, target_yaw)
        rate = rospy.Rate(20)
        if fixed_x is None or fixed_y is None or fixed_z is None:
            self.update_current_pos()
            fixed_x = self.current_pos.pose.position.x
            fixed_y = self.current_pos.pose.position.y
            fixed_z = self.current_pos.pose.position.z
        while not rospy.is_shutdown():
            yaw_val = traj.evaluate()
            if yaw_val is None:
                break
            pos_cmd = FlightNav()
            pos_cmd.target = 1
            pos_cmd.pos_xy_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_x = fixed_x
            pos_cmd.target_pos_y = fixed_y
            pos_cmd.pos_z_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_z = fixed_z
            pos_cmd.yaw_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_yaw = yaw_val
            self.pos_pub.publish(pos_cmd)
            rate.sleep()
        rospy.loginfo("Rotation complete, reached target yaw.")
        time.sleep(2)
         # Closed-loop position correction
        tolerance = 0.01  
        k_p = 0.2        
        start_correction_time = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown():
            self.update_current_pos()
            current_x = self.current_pos.pose.position.x
            current_y = self.current_pos.pose.position.y
            current_z = self.current_pos.pose.position.z
            error_x = fixed_x - current_x
            error_y = fixed_y - current_y
            error_z = fixed_z - current_z
            error_norm = math.sqrt(error_x**2 + error_y**2 + error_z**2)
            rospy.loginfo(f"Position correction: error norm = {error_norm:.3f}")
            if error_norm < tolerance:
                rospy.loginfo("Position correction complete. Error within tolerance.")
                break
            correction_x = k_p * error_x
            correction_y = k_p * error_y
            correction_z = k_p * error_z
            pos_cmd = FlightNav()
            pos_cmd.target = 1
            pos_cmd.pos_xy_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_x = current_x + correction_x
            pos_cmd.target_pos_y = current_y + correction_y
            pos_cmd.pos_z_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_pos_z = current_z + correction_z
            pos_cmd.yaw_nav_mode = FlightNav.POS_MODE
            pos_cmd.target_yaw = target_yaw  # Maintain yaw angle
            self.pos_pub.publish(pos_cmd)
            rate.sleep()
            if rospy.Time.now().to_sec() - start_correction_time > max_correction_duration:
                rospy.logwarn("Position correction exceeded maximum duration.")
                break
        rospy.loginfo("Rotation and position correction complete.")
        time.sleep(1)

    def execute(self, userdata):
        if not self.pos_initialization:
            rospy.logwarn("Positions are not initialized. Aborting mission.")
            return 'failed'
        self.update_current_pos()
        rospy.loginfo("Reinitiallizing the position...")
        self.wait_for_initialization(timeout=10)
        rospy.loginfo("Initializing position with yaw 0...")
        self.rotate_to_target_poly(0, avg_yaw_speed=self.avg_yaw_speed,
                                     fixed_x=self.start_x, fixed_y=self.start_y, fixed_z=self.start_z)
        rospy.loginfo("Moving horizontally to the valve position...")
        self.move_to_target_poly(self.valve_x, self.valve_y, self.current_pos.pose.position.z, avg_speed=self.avg_speed)
        time.sleep(1)
        if self.pos_check(0.3, self.valve_x, self.valve_y, self.current_pos.pose.position.z):
            rospy.loginfo("Arrived at the valve position.")
        else:
            rospy.logwarn("Failed to arrive at the valve position.")
            return 'failed'
        rospy.loginfo("Descending to the valve operation height...")
        target_z = self.valve_z + (self.z_offset_sim if self.is_simulation else self.z_offset_real)
        self.move_to_target_poly(self.valve_x, self.valve_y, target_z, avg_speed=self.avg_speed)
        time.sleep(2)
        if self.pos_check(0.3, self.valve_x, self.valve_y, target_z):
            rospy.loginfo("Arrived at the valve operation height.")
        else:
            rospy.logwarn("Failed to arrive at the valve operation height.")
            return 'failed'
        rospy.loginfo("Arrived at the valve position. Starting valve rotation...")
        self.rotate_to_target_poly(self.valve_rotation_angle + self.yaw_offset, avg_yaw_speed=self.avg_valve_rotation_speed,
                                     fixed_x=self.valve_x, fixed_y=self.valve_y, fixed_z=target_z)
        time.sleep(2)
        rospy.loginfo("Valve rotation completed. Hovering at start altitude...")
        self.move_to_target_poly(self.valve_x, self.valve_y, self.start_z, avg_speed=self.avg_speed)
        self.rotate_to_target_poly(0.0, avg_yaw_speed=self.avg_yaw_speed,
                                   fixed_x=self.valve_x, fixed_y=self.valve_y, fixed_z=self.start_z)
        time.sleep(1)
        rospy.loginfo("Returning to start position...")
        self.move_to_target_poly(self.start_x, self.start_y, self.start_z, avg_speed=self.avg_speed)
        if self.pos_check(0.3, self.start_x, self.start_y, self.start_z):
            rospy.loginfo("Returned to start position successfully.")
            return 'succeeded'
        else:
            rospy.logwarn("Failed to return to start position.")
            return 'failed'
class DisassembleState(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        # Module IDs
        modules_str = rospy.get_param("~module_ids", "")
        real_machine = rospy.get_param("~real_machine", True)
        modules = []
        if modules_str:
            modules = [int(x) for x in modules_str.split(',')]
        else:
            rospy.logerr("No module ID is designated!")

        # disassembleDemo 
        self.disassemble_demo = DisassemblyDemo(module_ids=modules, real_machine=real_machine)
    
    def execute(self, userdata):
        rospy.loginfo("Disassembling UAVs...")
        try:
            self.disassemble_demo.main()
            rospy.loginfo("Disassembly completed successfully.")
            return 'succeeded'
        except rospy.ROSInterruptException:
            rospy.logerr("Disassembly process failed.")
            return 'failed'

def main():
    rospy.init_node('valve_task_state_machine')
    sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])
    with sm:
        smach.StateMachine.add('ASSEMBLE', AssembleState(),
                                 transitions={'succeeded': 'ASSEMBLED_VALVE_TASK',
                                              'failed': 'TASK_FAILED'})
        smach.StateMachine.add('ASSEMBLED_VALVE_TASK', MoveAndRotateValveState(),
                                 transitions={'succeeded': 'DISASSEMBLE',
                                              'failed': 'TASK_FAILED'})
        smach.StateMachine.add('DISASSEMBLE', DisassembleState(),
                                    transitions={'succeeded': 'TASK_COMPLETED',
                                                'failed': 'TASK_FAILED'})
    outcome = sm.execute()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass