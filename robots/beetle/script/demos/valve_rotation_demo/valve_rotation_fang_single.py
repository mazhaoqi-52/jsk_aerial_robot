#!/usr/bin/env python
"""
Single UAV Valve Rotation Demo using SMACH State Machine

This script implements a modular state machine for single UAV valve manipulation,
following the style of valve_rotation_smach_test.py but simplified for single UAV operation.

States:
- MoveToValveState: Move UAV to approach position above valve
- ValveManipulationState: Perform valve alignment and rotation
- ReturnToStartState: Return UAV to starting position

Physical Parameters:
- valve_radius: 0.1225m (effective grasp radius from valve geometry)
- valve_beam_width: 0.0185m (beam thickness for grasp calculation)
- rotation_direction: 1 for anticlockwise (positive), -1 for clockwise (negative)
- insertion_offset: 0.015m (safety margin for approach to valve beam)
"""

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
from trajectory import AlignToGraspTrajectory, ValveRotationTrajectory
from motion_controller import MotionController

class SingleUAVStateBase(smach.State):
    """Base class for single UAV states with common functionality"""
    def __init__(self, outcomes, module_id=1):
        smach.State.__init__(self, outcomes=outcomes)
        self.module_id = module_id
        
        # Publishers and subscribers
        self.pub = rospy.Publisher(f"/uav{module_id}/flight_nav", FlightNav, queue_size=1)
        self.uav_sub = rospy.Subscriber(f"/uav{module_id}/mocap/pose", PoseStamped, self.uav_callback, queue_size=1)
        
        # Position tracking
        self.current_pos = None
        self.current_yaw = 0.0
        self.uav_received = threading.Event()
        
        # Valve position tracking
        self.valve_pos = None
        self.valve_yaw = 0.0
        self.valve_received = threading.Event()
        self.is_simulation = rospy.get_param("~simulation", True)
        
        if self.is_simulation:
            self.valve_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            self.valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
    
    def uav_callback(self, msg):
        self.current_pos = msg
        # Extract yaw from quaternion
        q = msg.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.uav_received.set()
    def valve_sim_callback(self, msg):
        self.valve_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        # Extract valve orientation
        q = msg.pose.pose.orientation
        _, _, self.valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.valve_received.set()
    
    def valve_callback(self, msg):
        self.valve_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        # Extract valve orientation
        q = msg.pose.orientation
        _, _, self.valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.valve_received.set()
    
    def get_current_position(self):
        """Get current UAV position as tuple"""
        if self.current_pos is None:
            return None
        return (self.current_pos.pose.position.x, 
                self.current_pos.pose.position.y, 
                self.current_pos.pose.position.z)
    
    def wait_for_positions(self, timeout=5):
        """Wait for UAV and valve positions"""
        return (self.uav_received.wait(timeout) and 
                self.valve_received.wait(timeout))

class MoveToValveState(SingleUAVStateBase):
    """Move UAV to approach position above valve"""
    def __init__(self, module_id=1, approach_height_offset=0.5, avg_speed=0.1):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        
        self.approach_height_offset = approach_height_offset
        self.avg_speed = avg_speed
    
    def execute(self, userdata):
        rospy.loginfo(f"MoveToValveState: Moving UAV{self.module_id} to valve approach position...")
        
        # Wait for position data
        if not self.wait_for_positions(timeout=5):
            rospy.logwarn("Timeout waiting for position data")
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None or self.valve_pos is None:
            rospy.logwarn("Failed to get required position data")
            return 'failed'
        
        rospy.loginfo(f"Current UAV position: {start_pos}")
        rospy.loginfo(f"Valve position: {self.valve_pos}")
        
        # Calculate approach position above valve
        approach_pos = (self.valve_pos[0], self.valve_pos[1], 
                       self.valve_pos[2] + self.approach_height_offset)
        
        rospy.loginfo(f"Target approach position: {approach_pos}")
        
        try:
            # Execute trajectory to approach position
            success = MotionController.execute_poly_motion_pose(
                self.pub, start_pos, approach_pos, self.avg_speed)
            
            if success:
                rospy.loginfo("Successfully reached valve approach position")
                return 'succeeded'
            else:
                rospy.logwarn("Failed to reach valve approach position")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during approach movement: {e}")
            return 'failed'

class ValveManipulationState(SingleUAVStateBase):
    """Perform valve alignment and rotation using trajectory classes"""
    def __init__(self, module_id=1, valve_radius=0.1225, valve_beam_width=0.0185,
                 rotation_direction=1, insertion_offset=0.015, 
                 approach_duration=3.0, rotation_duration=8.0, rotation_angle=math.pi/2):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        
        # Physical parameters
        self.valve_radius = valve_radius
        self.valve_beam_width = valve_beam_width
        self.rotation_direction = rotation_direction
        self.insertion_offset = insertion_offset
        self.approach_duration = approach_duration
        self.rotation_duration = rotation_duration
        self.rotation_angle = rotation_angle
        
        # Offset for valve operation height
        self.z_offset_sim = 0.05  # Simulation offset
        self.z_offset_real = 0.21  # Real world offset
    
    def execute(self, userdata):
        rospy.loginfo(f"ValveManipulationState: Starting valve manipulation with UAV{self.module_id}...")
        
        # Wait for position data
        if not self.wait_for_positions(timeout=5):
            rospy.logwarn("Timeout waiting for position data")
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None or self.valve_pos is None:
            rospy.logwarn("Failed to get required position data")
            return 'failed'
        
        rospy.loginfo(f"Starting valve manipulation from position: {start_pos}")
        rospy.loginfo(f"Valve position: {self.valve_pos}, yaw: {self.valve_yaw}")
        rospy.loginfo(f"Rotation direction: {'anticlockwise' if self.rotation_direction > 0 else 'clockwise'}")
        
        # Calculate operation height
        z_offset = self.z_offset_sim if self.is_simulation else self.z_offset_real
        operation_height = self.valve_pos[2] + z_offset
        
        try:
            # Stage 1: Move to operation height above valve
            rospy.loginfo("Stage 1: Moving to operation height...")
            pre_align_pos = (self.valve_pos[0], self.valve_pos[1], operation_height)
            
            success = MotionController.execute_poly_motion_pose(
                self.pub, start_pos, pre_align_pos, 0.1)
            
            if not success:
                rospy.logwarn("Failed to reach operation height")
                return 'failed'
            
            time.sleep(1)  # Stabilize
            
            # Stage 2: Align to grasp point
            rospy.loginfo("Stage 2: Aligning to grasp point...")
            align_traj = AlignToGraspTrajectory(
                approach_duration=self.approach_duration,
                valve_center=self.valve_pos,
                valve_pose_yaw=self.valve_yaw,
                grasp_height=operation_height,
                valve_radius=self.valve_radius,
                valve_beam_width=self.valve_beam_width,
                rotation_direction=self.rotation_direction,
                insertion_offset=self.insertion_offset
            )
            
            # Set start position for alignment
            align_traj.set_start_position(pre_align_pos)
            
            # Execute alignment trajectory
            rate = rospy.Rate(50)
            while not rospy.is_shutdown() and not align_traj.is_complete():
                result = align_traj.get_next_position_and_yaw()
                if result is None:
                    break
                pos, yaw = result
                
                # Send trajectory point
                MotionController.send_trajectory_point(self.pub, pos, yaw)
                rate.sleep()
            
            rospy.loginfo("Alignment stage complete")
            
            # Get final position after alignment
            target_info = align_traj.get_target_info()
            final_body_pos = target_info['body_position']
            final_body_yaw = target_info['body_yaw']
            
            rospy.loginfo(f"Final aligned position: {final_body_pos}, yaw: {final_body_yaw}")
            
            # Stage 3: Rotate around valve center
            rospy.loginfo("Stage 3: Rotating valve...")
            
            rotation_traj = ValveRotationTrajectory.create_from_mocap_data(
                rotation_duration=self.rotation_duration,
                valve_center=self.valve_pos,
                body_cog_position=final_body_pos,
                body_yaw=final_body_yaw,
                rotation_angle=self.rotation_angle,
                grasp_height=operation_height,
                rotation_direction=self.rotation_direction
            )
            
            # Start rotation trajectory
            rotation_traj.start_rotation()
            
            # Execute rotation trajectory
            while not rospy.is_shutdown() and not rotation_traj.is_complete():
                result = rotation_traj.get_next_position_and_yaw()
                if result is None:
                    break
                pos, yaw = result
                
                # Send trajectory point
                MotionController.send_trajectory_point(self.pub, pos, yaw)
                rate.sleep()
            
            rospy.loginfo("Rotation stage complete")
            
            # Stage 4: Move to safe position
            rospy.loginfo("Stage 4: Moving to safe position...")
            safe_pos = (self.valve_pos[0], self.valve_pos[1], start_pos[2])
            
            success = MotionController.execute_poly_motion_pose(
                self.pub, final_body_pos, safe_pos, 0.1)
            
            if success:
                rospy.loginfo("Valve manipulation completed successfully")
                return 'succeeded'
            else:
                rospy.logwarn("Failed to return to safe position")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during valve manipulation: {e}")
            return 'failed'

class ReturnToStartState(SingleUAVStateBase):
    """Return UAV to starting position"""
    def __init__(self, module_id=1, start_position=[0.0, 0.0, 1.0], avg_speed=0.1):
        SingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], module_id=module_id)
        
        self.start_position = start_position
        self.avg_speed = avg_speed
    
    def execute(self, userdata):
        rospy.loginfo(f"ReturnToStartState: Returning UAV{self.module_id} to start position...")
        
        # Wait for position data
        if not self.uav_received.wait(5):
            rospy.logwarn("Timeout waiting for UAV position")
            return 'failed'
        
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logwarn("Failed to get current position")
            return 'failed'
        
        rospy.loginfo(f"Current position: {current_pos}")
        rospy.loginfo(f"Target start position: {self.start_position}")
        
        try:
            # Execute trajectory to start position
            success = MotionController.execute_poly_motion_pose(
                self.pub, current_pos, self.start_position, self.avg_speed)
            
            if success:
                rospy.loginfo("Successfully returned to start position")
                return 'succeeded'
            else:
                rospy.logwarn("Failed to return to start position")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during return movement: {e}")
            return 'failed'

def main():
    rospy.init_node('valve_manipulation_single_uav')
    
    # Get parameters
    module_id = rospy.get_param("~module_id", 1)
    start_x = rospy.get_param("~start_x", 0.0)
    start_y = rospy.get_param("~start_y", 0.0)
    start_z = rospy.get_param("~start_z", 1.0)
    rotation_direction = rospy.get_param("~rotation_direction", 1)  # 1 for anticlockwise
    rotation_angle = rospy.get_param("~rotation_angle", math.pi/2)  # 90 degrees default
    approach_height_offset = rospy.get_param("~approach_height_offset", 0.5)
    
    rospy.loginfo(f"Starting valve manipulation for UAV{module_id}")
    rospy.loginfo(f"Start position: ({start_x}, {start_y}, {start_z})")
    rospy.loginfo(f"Rotation direction: {'anticlockwise' if rotation_direction > 0 else 'clockwise'}")
    rospy.loginfo(f"Rotation angle: {rotation_angle:.3f} rad ({math.degrees(rotation_angle):.1f} deg)")
    
    # Create state machine
    sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])
    
    with sm:
        smach.StateMachine.add('MOVE_TO_VALVE', 
                               MoveToValveState(module_id=module_id, 
                                               approach_height_offset=approach_height_offset,
                                               avg_speed=0.1),
                               transitions={'succeeded': 'VALVE_MANIPULATION',
                                          'failed': 'TASK_FAILED'})
        
        smach.StateMachine.add('VALVE_MANIPULATION', 
                               ValveManipulationState(module_id=module_id,
                                                     rotation_direction=rotation_direction,
                                                     rotation_angle=rotation_angle),
                               transitions={'succeeded': 'RETURN_TO_START',
                                          'failed': 'TASK_FAILED'})
        
        smach.StateMachine.add('RETURN_TO_START', 
                               ReturnToStartState(module_id=module_id,
                                                 start_position=[start_x, start_y, start_z],
                                                 avg_speed=0.1),
                               transitions={'succeeded': 'TASK_COMPLETED',
                                          'failed': 'TASK_FAILED'})
    
    # Execute state machine
    rospy.loginfo("Starting valve manipulation state machine...")
    outcome = sm.execute()
    rospy.loginfo(f"Valve manipulation task completed with outcome: {outcome}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
