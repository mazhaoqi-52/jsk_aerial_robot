#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))
import rospy
import smach
import time
import math
import threading
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from task.assembly_motion import *
from task.disassembly_motion import *
from trajectory import PolynomialTrajectory
from base_UAV_state import SeperatedMotionStateBase, AssemblyMotionStateBase


def execute_poly_motion_pose_async(pub, start, target, avg_speed):
    """Execute polynomial trajectory asynchronously (helper function)."""
    def motion_thread():
        distance = math.sqrt(sum((target[i] - start[i])**2 for i in range(3)))
        duration = max(1.0, distance / avg_speed)
        traj = PolynomialTrajectory(start, target, duration)
        rate = rospy.Rate(20)
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start_time
            if elapsed >= duration:
                break
            pos = traj.get_position(elapsed)
            nav_msg = FlightNav()
            nav_msg.header.stamp = rospy.Time.now()
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
            nav_msg.target_pos_x = pos[0]
            nav_msg.target_pos_y = pos[1]
            nav_msg.target_pos_z = pos[2]
            pub.publish(nav_msg)
            rate.sleep()
    t = threading.Thread(target=motion_thread)
    t.start()
    return t


class SeparatedMoveToGateState(SeperatedMotionStateBase):
    def __init__(self,
                 maze_entrance_x = 1.0,
                 maze_entrance_y = 0.0,
                 maze_entrance_z = 1.0,
                 maze_offset_x = 0.2,
                 maze_offset_y = 0.5,
                 avg_speed = 0.1):
        
        SeperatedMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])
        self.maze_entrance_x = maze_entrance_x
        self.maze_entrance_y = maze_entrance_y
        self.maze_entrance_z = maze_entrance_z
        self.maze_offset_x = maze_offset_x
        self.maze_offset_y = maze_offset_y
        self.avg_speed = avg_speed

    def execute(self, userdata):
        rospy.loginfo("SeparatedMoveToGateState: Waiting for current UAV positions...")
        if not self.wait_for_uav_positions(timeout=5):
            rospy.logwarn("SeparatedMoveToGateState: Timeout waiting for UAV positions.")
            return 'failed'
        start1, start2 = self.get_uav_positions()
        rospy.loginfo("Current pos of UAV1: {}, UAV2: {}".format(start1, start2))
        target_x = self.maze_entrance_x - self.maze_offset_x
        target_y = self.maze_entrance_y
        target_z = self.maze_entrance_z
        target1 = [target_x, target_y + self.maze_offset_y, target_z]
        target2 = [target_x, target_y - self.maze_offset_y, target_z]
        rospy.loginfo("Moving UAVs to target positions: UAV1: {}, UAV2: {}".format(target1, target2))
        threads = []
        t1 = execute_poly_motion_pose_async(self.beetle1_pub, start1, target1, self.avg_speed)
        threads.append(t1)
        t2 = execute_poly_motion_pose_async(self.beetle2_pub, start2, target2, self.avg_speed)
        threads.append(t2)
        for t in threads:
            t.join()
        time.sleep(2)
        rospy.loginfo("Reached maze entrance.")
        return 'succeeded'

class SeparatedMoveToValveState(SeperatedMotionStateBase):
    def __init__(self,
                 maze_length = 1.0,
                 x_offset = 0.65,
                 y_offset = 0.25,
                 avg_speed = 0.1,
                 safety_margin = 0.3):

        SeperatedMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])
        self.maze_length = maze_length
        self.x_offset = x_offset
        self.y_offset = y_offset 
        self.avg_speed = avg_speed 
        self.safety_margin = safety_margin 

        self.beetle1_received.clear()
        self.beetle2_received.clear()
        self.valve_received = threading.Event()
        self.is_simulation = rospy.get_param("~simulation", True)

        # Subscribe to Valve position
        if self.is_simulation:
            self.pose_valve_sim = None
            self.valve_sim_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            self.pose_valve = None
            self.valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)

    def valve_sim_callback(self, msg):
        self.pose_valve_sim = msg
        self.valve_received.set()

    def valve_callback(self, msg):
        self.pose_valve = msg
        self.valve_received.set()

    def execute(self, userdata):
        rospy.loginfo("SeparatedMoveToValveState: Waiting for current UAV positions...")
        if not self.wait_for_uav_positions(timeout=5):
            rospy.logwarn("SeparatedMoveToValveState: Timeout waiting for UAV positions.")
            return 'failed'
        if not self.valve_received.wait(5):
            rospy.logwarn("SeparatedMoveToValveState: Timeout waiting for valve position.")
            return 'failed'
        start1, start2 = self.get_uav_positions()
        rospy.loginfo("Current UAV positions:\n  UAV1: {}\n  UAV2: {}".format(start1, start2))
        
        if self.is_simulation:
            valve_x = self.pose_valve_sim.pose.pose.position.x
            valve_y = self.pose_valve_sim.pose.pose.position.y
            valve_z = self.pose_valve_sim.pose.pose.position.z
            offset = 0.23
        else:
            valve_x = self.pose_valve.pose.position.x
            valve_y = self.pose_valve.pose.position.y
            valve_z = self.pose_valve.pose.position.z
            offset = 0.47
        safe_altitude = valve_z + offset

        threads = []
        for pub, start, target in [
            (self.beetle1_pub, start1, [start1[0], start1[1], safe_altitude]),
            (self.beetle2_pub, start2, [start2[0], start2[1], safe_altitude])
        ]:
            t = execute_poly_motion_pose_async(pub, start, target, self.avg_speed)
            threads.append(t)
        for t in threads:
            t.join()
        time.sleep(2)

        current1 = [start1[0], start1[1], safe_altitude]
        current2 = [start2[0], start2[1], safe_altitude]
        horiz_target1 = [valve_x, valve_y + self.y_offset, safe_altitude + self.safety_margin]
        horiz_target2 = [valve_x + self.x_offset, valve_y - self.y_offset, safe_altitude + self.safety_margin]
        threads = []
        for pub, current, target in [
            (self.beetle1_pub, current1, horiz_target1),
            (self.beetle2_pub, current2, horiz_target2)
        ]:
            t = execute_poly_motion_pose_async(pub, current, target, self.avg_speed)
            threads.append(t)
        for t in threads:
            t.join()
        time.sleep(2)

        threads = []
        final_target1 = [valve_x, valve_y, safe_altitude + self.safety_margin]
        final_target2 = [valve_x + self.x_offset, valve_y, safe_altitude + self.safety_margin]
        for pub, current, target in [
            (self.beetle1_pub, horiz_target1, final_target1),
            (self.beetle2_pub, horiz_target2, final_target2)
        ]:
            t = execute_poly_motion_pose_async(pub, current, target, self.avg_speed)
            threads.append(t)
        for t in threads:
            t.join()
        time.sleep(2)
        
        rospy.loginfo("SeparatedMoveToValveState: UAVs have reached above the valve safely.")
        return 'succeeded'
    
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

class MoveAndRotateValveState(AssemblyMotionStateBase):
    def __init__(self,
                 z_offset_real = 0.47,# 0.21(when use the real valve instead of the valve_fake)
                 z_offset_sim = 0.05,#0.23
                 z_offset_safe = 0.3,
                 yaw_offset = 0,
                 valve_rotation_angle_compenstation = 0.06,
                 valve_rotation_angle = math.pi / 2.0,
                 avg_speed = 0.15,
                 avg_yaw_speed = 0.15,
                 avg_valve_rotation_speed = 0.3,
                 pos_initialization = False,
                 exit_target = [4.0, 0.0, 1.0]):
        
        AssemblyMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])

        self.z_offset_real = z_offset_real
        self.z_offset_sim = z_offset_sim
        self.z_offset_safe = z_offset_safe
        self.yaw_offset = yaw_offset
        self.valve_rotation_angle_compenstation = valve_rotation_angle_compenstation
        self.valve_rotation_angle = valve_rotation_angle + self.valve_rotation_angle_compenstation
        self.avg_speed = avg_speed
        self.avg_yaw_speed = avg_yaw_speed
        self.avg_valve_rotation_speed = avg_valve_rotation_speed
        self.pos_initialization = pos_initialization
        self.exit_target = exit_target
        self.valve_received = threading.Event()
        # Simulation flag
        self.is_simulation = rospy.get_param("~simulation", True)
        if self.is_simulation:
            self.pos_valve_sim_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            self.pos_valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
        self.pos_valve_sim = None
        self.pos_valve = None
        self.wait_for_initialization(timeout=10)
    
    def valve_callback(self, msg):
        self.pos_valve = msg
        self.pos_valve_sub.unregister()
        self.valve_received.set()
    
    def valve_sim_callback(self, msg):
        self.pos_valve_sim = msg
        # self.pos_valve_sim_sub.unregister()
        self.valve_received.set()

    def wait_for_initialization(self, timeout=10):
        rospy.sleep(0.5)
        from std_msgs.msg import Empty
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

    def pos_initialize(self):
        self.update_current_pos()
        self.start_pos = (self.current_pos.pose.position.x,
                          self.current_pos.pose.position.y,
                          self.current_pos.pose.position.z)
        if self.is_simulation and self.pos_valve_sim is not None:
            self.valve_pos = (self.pos_valve_sim.pose.pose.position.x,
                              self.pos_valve_sim.pose.pose.position.y,
                              self.pos_valve_sim.pose.pose.position.z)
        elif self.pos_valve is not None:
            self.valve_pos = (self.pos_valve.pose.position.x,
                              self.pos_valve.pose.position.y,
                              self.pos_valve.pose.position.z)
        else:
            rospy.logwarn("Valve position not available.")
            return 'failed'
        rospy.loginfo("start position: {}".format(self.start_pos))
        rospy.loginfo("valve position: {}".format(self.valve_pos))

    def execute(self, userdata):
        if not self.pos_initialization:
            rospy.logwarn("Positions are not initialized. Aborting mission.")
            return 'failed'
        
        self.update_current_pos()
        rospy.loginfo("Reinitializing the position...")
        self.wait_for_initialization(timeout=10)
        
        rospy.loginfo("Initializing position with yaw 0...")
        self.rotate_to_target_poly(0, avg_yaw_speed=self.avg_yaw_speed, fixed_pos=(self.start_pos[0], self.start_pos[1], self.start_pos[2]))
        rospy.loginfo("Moving horizontally to the valve position...")
        target = (self.valve_pos[0], self.valve_pos[1], self.current_pos.pose.position.z)
        self.move_to_target_poly(target, avg_speed=self.avg_speed-0.05)
        time.sleep(1)
        if self.pos_check((self.valve_pos[0], self.valve_pos[1], self.valve_pos[2]), self.current_pos.pose.position.z):
            rospy.loginfo("Arrived at the valve position.")
        else:
            rospy.logwarn("Failed to arrive at the valve position.")
            return 'failed'
        
        rospy.loginfo("Descending to the valve operation height...")
        target_z = self.valve_pos[2] + (self.z_offset_sim if self.is_simulation else self.z_offset_real)
        self.move_to_target_poly((self.valve_pos[0], self.valve_pos[1], target_z), self.avg_speed)
        time.sleep(2)
        if self.pos_check((self.valve_pos[0], self.valve_pos[1], target_z)):
            rospy.loginfo("Arrived at the valve operation height.")
        else:
            rospy.logwarn("Failed to arrive at the valve operation height.")
            return 'failed'
        
        rospy.loginfo("Arrived at the valve position. Starting valve rotation...")
        self.rotate_to_target_poly(self.valve_rotation_angle + self.yaw_offset, avg_yaw_speed=self.avg_valve_rotation_speed, fixed_pos=(self.valve_pos[0], self.valve_pos[1], target_z))
        time.sleep(2)
        
        rospy.loginfo("Valve rotation completed. Hovering at start altitude...")
        rospy.loginfo("start pos_z: {}".format(self.start_pos[2]))
        rospy.loginfo("current pos_z: {}".format(self.current_pos.pose.position.z))
        self.move_to_target_poly((self.valve_pos[0], self.valve_pos[1], self.start_pos[2]+self.z_offset_safe),self.avg_speed)
        self.rotate_to_target_poly(0.0, avg_yaw_speed=self.avg_yaw_speed, fixed_pos=(self.valve_pos[0], self.valve_pos[1], self.start_pos[2]))
        time.sleep(1)
        
        rospy.loginfo("Returning to start position...")
        self.move_to_target_poly((self.start_pos[0], self.start_pos[1], self.start_pos[2]), self.avg_speed)
        if self.pos_check(self.start_pos, 0.3):
            rospy.loginfo("Returned to start position successfully.")
            return 'succeeded'
        else:
            rospy.logwarn("Failed to return to start position.")
            return 'failed'

class AssembledLeaveState(AssemblyMotionStateBase):
    def __init__(self,
                 exit_target = [6.0, 0.0, 1.0],
                 avg_speed = 0.1):
        
        AssemblyMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])

        self.exit_maze = exit_target
        self.avg_speed = avg_speed
        self.beetle1_received.clear()
        self.beetle2_received.clear()
 
    def get_current_pose(self):
        self.update_current_pos()
        return (self.current_pos.pose.position.x,
                self.current_pos.pose.position.y,
                self.current_pos.pose.position.z)

    def execute(self, userdata):
        rospy.loginfo("AssembledLeaveState: Leaving the valve...")
        start_time = rospy.Time.now().to_sec()
        while (not self.beetle1_received.is_set() or not self.beetle2_received.is_set()) and (rospy.Time.now().to_sec() - start_time < 5):
            rospy.sleep(0.1)
        current = self.get_current_pose()
        rospy.loginfo("Current pose is: {}".format(current))
        self.move_to_target_poly(self.exit_maze, avg_speed=self.avg_speed)
        rospy.sleep(2)
        if self.pos_check(self.exit_maze):
            rospy.loginfo("AssembledLeaveState: Successfully left the valve area.")
            return 'succeeded'
        else:
            rospy.logwarn("AssembledLeaveState: Failed to leave the valve area.")
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
        smach.StateMachine.add('SEPARATED_MOVE_GATE', SeparatedMoveToGateState(),
                                 transitions={'succeeded': 'SEPARATED_MOVE_VALVE',
                                              'failed': 'TASK_FAILED'})
        smach.StateMachine.add('SEPARATED_MOVE_VALVE', SeparatedMoveToValveState(),
                                 transitions={'succeeded': 'ASSEMBLE',
                                              'failed': 'TASK_FAILED'})
        smach.StateMachine.add('ASSEMBLE', AssembleState(),
                                 transitions={'succeeded': 'ASSEMBLED_VALVE_TASK',
                                              'failed': 'TASK_FAILED'})
        smach.StateMachine.add('ASSEMBLED_VALVE_TASK', MoveAndRotateValveState(),
                                 transitions={'succeeded': 'ASSEMBLED_LEAVE',
                                              'failed': 'TASK_FAILED'})
        smach.StateMachine.add('ASSEMBLED_LEAVE', AssembledLeaveState(),
                                 transitions={'succeeded': 'TASK_COMPLETED',
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
