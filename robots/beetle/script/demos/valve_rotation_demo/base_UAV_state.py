#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

import math
import threading
import time
import rospy
import smach
import smach_ros
import numpy as np
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from valve_rotation_demo.trajectory import PolynomialTrajectory
from task.assembly_motion import AssemblyDemo

class SeperatedMotionStateBase(smach.State):
    def __init__(self, outcomes, pub_topic_format="/beetle{}/target_pose", sub_topic_format="/beetle{}/mocap/pose"):
        smach.State.__init__(self, outcomes=outcomes)
        module_ids_str = rospy.get_param("~module_ids", "1,2")
        rospy.loginfo("DualUAVStateBase: Module IDs: %s" % module_ids_str)
        self.module_ids = [int(x) for x in module_ids_str.split(',')]
        if len(self.module_ids) < 2:
            rospy.logerr("DualUAVStateBase: At least 2 module IDs are required.")
        # Publishers
        self.beetle1_pub = rospy.Publisher(pub_topic_format.format(self.module_ids[0]), PoseStamped, queue_size=1)
        self.beetle2_pub = rospy.Publisher(pub_topic_format.format(self.module_ids[1]), PoseStamped, queue_size=1)
        # Subscribers  
        rospy.Subscriber(sub_topic_format.format(self.module_ids[0]), PoseStamped, self.beetle1_callback, queue_size=1)
        rospy.Subscriber(sub_topic_format.format(self.module_ids[1]), PoseStamped, self.beetle2_callback, queue_size=1)
        self.current_pose_beetle1 = None
        self.current_pose_beetle2 = None
        self.beetle1_received = threading.Event()
        self.beetle2_received = threading.Event()

    def beetle1_callback(self, msg):
        self.current_pose_beetle1 = msg
        self.beetle1_received.set()

    def beetle2_callback(self, msg):
        self.current_pose_beetle2 = msg
        self.beetle2_received.set()

    def wait_for_uav_positions(self, timeout=5):
        return self.beetle1_received.wait(timeout) and self.beetle2_received.wait(timeout)

    def get_uav_positions(self):
        pos1 = [self.current_pose_beetle1.pose.position.x,
                self.current_pose_beetle1.pose.position.y,
                self.current_pose_beetle1.pose.position.z]
        pos2 = [self.current_pose_beetle2.pose.position.x,
                self.current_pose_beetle2.pose.position.y,
                self.current_pose_beetle2.pose.position.z]
        return pos1, pos2

class AssemblyMotionStateBase(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        module_ids_str = rospy.get_param("~module_ids", "1,2")
        rospy.loginfo("AssemblyMotionStateBase: Module IDs: %s" % module_ids_str)
        self.module_ids = [int(x) for x in module_ids_str.split(',')]
        if len(self.module_ids) < 2:
            rospy.logerr("AssemblyMotionStateBase: At least 2 module IDs are required.")
        # Publisher for assembly navigation (formation control)
        self.pos_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=1)
        
        # Subscribers for position feedback
        rospy.Subscriber("/beetle{}/mocap/pose".format(self.module_ids[0]), PoseStamped, self.beetle1_callback, queue_size=1)
        rospy.Subscriber("/beetle{}/mocap/pose".format(self.module_ids[1]), PoseStamped, self.beetle2_callback, queue_size=1)
        self.beetle1_pose = None
        self.beetle2_pose = None
        self.beetle1_received = threading.Event()
        self.beetle2_received = threading.Event()
        self.current_pos = PoseStamped()  # Average position of two UAVs

    def beetle1_callback(self, msg):
        self.beetle1_pose = msg
        self.beetle1_received.set()

    def beetle2_callback(self, msg):
        self.beetle2_pose = msg
        self.beetle2_received.set()

    def wait_for_uav_positions(self, timeout=5):
        """Wait for both UAV positions to be received"""
        return self.beetle1_received.wait(timeout) and self.beetle2_received.wait(timeout)

    def get_uav_positions(self):
        """Get current UAV positions as [x, y, z] lists"""
        if self.beetle1_pose and self.beetle2_pose:
            pos1 = [self.beetle1_pose.pose.position.x,
                    self.beetle1_pose.pose.position.y,
                    self.beetle1_pose.pose.position.z]
            pos2 = [self.beetle2_pose.pose.position.x,
                    self.beetle2_pose.pose.position.y,
                    self.beetle2_pose.pose.position.z]
            return pos1, pos2
        else:
            rospy.logwarn("UAV positions not yet received")
            return None, None

    def update_current_pos(self):
        if self.beetle1_pose and self.beetle2_pose:
            self.current_pos.pose.position.x = (self.beetle1_pose.pose.position.x + self.beetle2_pose.pose.position.x) / len(self.module_ids)
            self.current_pos.pose.position.y = (self.beetle1_pose.pose.position.y + self.beetle2_pose.pose.position.y) / len(self.module_ids)
            self.current_pos.pose.position.z = (self.beetle1_pose.pose.position.z + self.beetle2_pose.pose.position.z) / len(self.module_ids)

    def get_uav_yaw(self):
        orientation_q1 = self.beetle1_pose.pose.orientation
        orientation_q2 = self.beetle2_pose.pose.orientation
        quaternion1 = (orientation_q1.x, orientation_q1.y, orientation_q1.z, orientation_q1.w)
        quaternion2 = (orientation_q2.x, orientation_q2.y, orientation_q2.z, orientation_q2.w)
        _, _, yaw1 = euler_from_quaternion(quaternion1)
        _, _, yaw2 = euler_from_quaternion(quaternion2)
        return (yaw1 + yaw2) / 2.0

    # Move to target position using polynomial trajectory and PID 
    def move_to_target_poly(self, target, avg_speed=None, max_correction_duration=5):
        if avg_speed is None or avg_speed <= 0:
            avg_speed = 0.1
        self.update_current_pos()
        start_pos = (
            self.current_pos.pose.position.x,
            self.current_pos.pose.position.y,
            self.current_pos.pose.position.z
        )
        distance = math.sqrt(sum((t - s) ** 2 for s, t in zip(start_pos, target)))
        duration = distance / avg_speed
        rospy.loginfo("Moving from {} to {} over {:.2f} s.".format(start_pos, target, duration))
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_pos, target)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            pos = traj.evaluate()
            if pos is None:
                break
            cmd = FlightNav()
            cmd.target = 1
            cmd.pos_xy_nav_mode = FlightNav.POS_MODE
            cmd.target_pos_x, cmd.target_pos_y, cmd.target_pos_z = pos
            self.pos_pub.publish(cmd)
            rate.sleep()
        rospy.loginfo("Initial trajectory reached target, starting closed-loop correction.")
        
        tolerance = 0.27    
        k_p = 0.2          
        start_correction_time = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown():
            self.update_current_pos()
            current_pos = (
                self.current_pos.pose.position.x,
                self.current_pos.pose.position.y,
                self.current_pos.pose.position.z
            )
            error = math.sqrt(sum((t - c) ** 2 for t, c in zip(target, current_pos)))
            rospy.loginfo("Closed-loop correction: current error norm = {:.3f}".format(error))
            if error < tolerance:
                rospy.loginfo("Terminal position correction complete. Error within tolerance.")
                return True  
            
            correction = [k_p * (t - c) for t, c in zip(target, current_pos)]
            cmd = FlightNav()
            cmd.target = 1
            cmd.pos_xy_nav_mode = FlightNav.POS_MODE
            cmd.target_pos_x = current_pos[0] + correction[0]
            cmd.target_pos_y = current_pos[1] + correction[1]
            cmd.pos_z_nav_mode = FlightNav.POS_MODE
            cmd.target_pos_z = current_pos[2] + correction[2]
            self.pos_pub.publish(cmd)
            rate.sleep()
            
            if rospy.Time.now().to_sec() - start_correction_time > max_correction_duration:
                rospy.logwarn("Terminal correction exceeded maximum duration.")
                if error < tolerance * 2:  
                    rospy.loginfo(f"Movement completed with acceptable error {error:.3f}")
                    return True
                else:
                    rospy.logerr(f"Movement failed - final error {error:.3f} too large")
                    return False
        
        rospy.logwarn("Move to target loop ended unexpectedly")
        return False


    def rotate_to_target_poly(self, target_yaw, avg_yaw_speed=None, fixed_pos=None):
        if avg_yaw_speed is None or avg_yaw_speed <= 0:
            avg_yaw_speed = 0.10
        current_yaw = self.get_uav_yaw()
        yaw_diff = (target_yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi
        duration = abs(yaw_diff) / avg_yaw_speed
        rospy.loginfo("Rotating from yaw {:.4f} to {:.4f} over {:.2f} s.".format(current_yaw, target_yaw, duration))
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(current_yaw, target_yaw)
        if fixed_pos is None:
            self.update_current_pos()
            fixed_pos = (self.current_pos.pose.position.x,
                         self.current_pos.pose.position.y,
                         self.current_pos.pose.position.z)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            yaw_val = traj.evaluate()
            if yaw_val is None:
                break
            cmd = FlightNav()
            cmd.target = 1
            cmd.pos_xy_nav_mode = FlightNav.POS_MODE
            cmd.target_pos_x, cmd.target_pos_y, cmd.target_pos_z = fixed_pos
            cmd.yaw_nav_mode = FlightNav.POS_MODE
            cmd.target_yaw = yaw_val
            self.pos_pub.publish(cmd)
            rate.sleep()
        rospy.loginfo("Rotation complete.")

    def pos_check(self, target, tolerance=0.3):
        self.update_current_pos()
        current = (self.current_pos.pose.position.x,
                   self.current_pos.pose.position.y,
                   self.current_pos.pose.position.z)
        error = math.sqrt(sum((t - c) ** 2 for t, c in zip(target, current)))
        
        # 方案A：根据阶段调整容差(插入阶段需要更高精度)
        if hasattr(self, '_insertion_phase') and self._insertion_phase:
            # 插入阶段使用更严格的容差
            adjusted_tolerance = min(tolerance, 0.15)  # 最大15cm容差
            rospy.loginfo("Position error: {:.2f} m (insertion phase, tolerance: {:.2f} m)".format(error, adjusted_tolerance))
            return error < adjusted_tolerance
        else:
            rospy.loginfo("Position error: {:.2f} m (normal phase, tolerance: {:.2f} m)".format(error, tolerance))
            return error < tolerance