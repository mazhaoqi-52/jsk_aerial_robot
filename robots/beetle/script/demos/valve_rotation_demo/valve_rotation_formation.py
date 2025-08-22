#!/usr/bin/env python3
"""
Two UAV Formation Valve Rotation using SMACH State Machine
Dual UAV assembly and valve rotation task based on unified control and feedforward control

Basic workflow:
1. Two UAVs fly to assembly positions near the valve
2. Perform formation assembly
3. Switch to unified control mode
4. Enable feedforward control compensation
5. Execute valve rotation task
6. Disassemble and return after completion
"""

import sys
import os

# Add current directory FIRST to ensure local imports work correctly
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import smach
import smach_ros
import time
import math
import threading

from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from trajectory import create_constant_distance_trajectory
from unified_motion_controller import UnifiedMotionController
from base_UAV_state import SeperatedMotionStateBase, AssemblyMotionStateBase

# Import task modules
try:
    from task.assembly_motion import AssemblyDemo
    from task.disassembly_motion import DisassemblyDemo
    from beetle.assembly_api import *
except ImportError as e:
    rospy.logwarn(f"Cannot import assembly modules: {e}")
    # Provide fallback simple implementation
    class AssemblyDemo:
        def __init__(self, module_ids=[1,2], real_machine=False):
            pass
        def main(self):
            return True
    
    class DisassemblyDemo:
        def __init__(self, module_ids=[1,2], real_machine=False):
            pass  
        def main(self):
            return True

class FormationValveRotationController:
    """Unified control and feedforward control manager"""
    
    def __init__(self):
        self.feedforward_pub_1 = rospy.Publisher(
            '/beetle1/controller/feedforward_wrench', 
            WrenchStamped, 
            queue_size=1
        )
        self.feedforward_pub_2 = rospy.Publisher(
            '/beetle2/controller/feedforward_wrench', 
            WrenchStamped, 
            queue_size=1
        )
        
        # Monitor formation control status
        self.formation_wrench_sub = rospy.Subscriber(
            '/beetle1/controller/formation_wrench_debug',
            WrenchStamped,
            self.formation_wrench_callback
        )
        
        self.unified_control_enabled = False
        self.feedforward_enabled = False
        
    def enable_unified_control(self):
        """Enable unified 4n-rotor control mode"""
        rospy.set_param("controller/unified_control_mode", True)
        self.unified_control_enabled = True
        rospy.loginfo("Dual UAV unified control mode enabled")
        
    def disable_unified_control(self):
        """Disable unified control, return to Leader-Follower mode"""
        rospy.set_param("controller/unified_control_mode", False)
        self.unified_control_enabled = False
        rospy.loginfo("Returned to Leader-Follower control mode")
        
    def set_valve_rotation_feedforward(self, valve_torque=3.0, valve_force_z=-8.0):
        """
        Set valve rotation feedforward compensation
        
        Args:
            valve_torque: Expected valve resistance torque (Nm)
            valve_force_z: Expected vertical resistance force (N)
        """
        ff_wrench = WrenchStamped()
        ff_wrench.header.stamp = rospy.Time.now()
        ff_wrench.header.frame_id = "cog"
        
        # Set expected external forces/torques
        ff_wrench.wrench.force.x = 0.0
        ff_wrench.wrench.force.y = 0.0
        ff_wrench.wrench.force.z = valve_force_z  # Valve resistance force
        ff_wrench.wrench.torque.x = 0.0
        ff_wrench.wrench.torque.y = 0.0
        ff_wrench.wrench.torque.z = valve_torque  # Valve rotation resistance
        
        # Enable feedforward and publish to both UAVs
        rospy.set_param("controller/valve_rotation_feedforward/enabled", True)
        self.feedforward_pub_1.publish(ff_wrench)
        self.feedforward_pub_2.publish(ff_wrench)
        self.feedforward_enabled = True
        
        rospy.loginfo(f"Valve rotation feedforward compensation set: torque={valve_torque} Nm, vertical_force={valve_force_z} N")
        
    def disable_feedforward(self):
        """Disable feedforward compensation"""
        rospy.set_param("controller/valve_rotation_feedforward/enabled", False)
        self.feedforward_enabled = False
        rospy.loginfo("Feedforward compensation disabled")
        
    def formation_wrench_callback(self, msg):
        """Monitor formation control wrench"""
        rospy.loginfo_throttle(2.0, 
            f"Formation control wrench - Force: [{msg.wrench.force.x:.2f}, {msg.wrench.force.y:.2f}, {msg.wrench.force.z:.2f}] N, "
            f"Torque: [{msg.wrench.torque.x:.2f}, {msg.wrench.torque.y:.2f}, {msg.wrench.torque.z:.2f}] Nm"
        )

class MoveToAssemblyPositionState(SeperatedMotionStateBase):
    """Move to assembly position state"""
    
    def __init__(self, 
                 valve_x=1.5, 
                 valve_y=0.0, 
                 valve_z=1.2,
                 assembly_distance=0.4,
                 avg_speed=0.15):
        
        SeperatedMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])
        self.valve_x = valve_x
        self.valve_y = valve_y 
        self.valve_z = valve_z
        self.assembly_distance = assembly_distance
        self.avg_speed = avg_speed
        
        # Subscribe to valve position (simulation or real)
        self.is_simulation = rospy.get_param("~simulation", True)
        self.valve_received = threading.Event()
        
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
        rospy.loginfo("Moving to assembly position...")
        
        # Wait for valve position information
        if not self.valve_received.wait(timeout=5):
            rospy.logwarn("Failed to get valve position information")
            return 'failed'
            
        # Get valve position
        if self.is_simulation and self.pose_valve_sim:
            valve_pos = [
                self.pose_valve_sim.pose.pose.position.x,
                self.pose_valve_sim.pose.pose.position.y,
                self.pose_valve_sim.pose.pose.position.z
            ]
        elif not self.is_simulation and self.pose_valve:
            valve_pos = [
                self.pose_valve.pose.position.x,
                self.pose_valve.pose.position.y,
                self.pose_valve.pose.position.z
            ]
        else:
            rospy.logerr("Cannot get valid valve position")
            return 'failed'
            
        rospy.loginfo(f"Valve position: {valve_pos}")
        
        # Wait for UAV positions
        if not self.wait_for_uav_positions(timeout=5):
            rospy.logwarn("Timeout waiting for UAV positions")
            return 'failed'
            
        start1, start2 = self.get_uav_positions()
        rospy.loginfo(f"Current UAV positions - UAV1: {start1}, UAV2: {start2}")
        
        # Calculate assembly positions (in front of valve, horizontal arrangement)
        assembly_offset_x = -0.6  # 0.6m in front of valve
        target1 = [
            valve_pos[0] + assembly_offset_x,
            valve_pos[1] - self.assembly_distance/2,  # Left side
            valve_pos[2]
        ]
        target2 = [
            valve_pos[0] + assembly_offset_x,
            valve_pos[1] + self.assembly_distance/2,  # Right side
            valve_pos[2]
        ]
        
        rospy.loginfo(f"Target assembly positions - UAV1: {target1}, UAV2: {target2}")
        
        # Move both UAVs to assembly positions simultaneously
        threads = []
        t1 = UnifiedMotionController.execute_poly_motion_pose_async(
            self.beetle1_pub, start1, target1, self.avg_speed)
        threads.append(t1)
        
        t2 = UnifiedMotionController.execute_poly_motion_pose_async(
            self.beetle2_pub, start2, target2, self.avg_speed)
        threads.append(t2)
        
        # Wait for movement completion
        for t in threads:
            t.join()
            
        time.sleep(2)  # Stabilize
        rospy.loginfo("Reached assembly positions")
        return 'succeeded'

class AssemblyFormationState(AssemblyMotionStateBase):
    """Formation assembly state"""
    
    def __init__(self):
        AssemblyMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])
        
    def execute(self, userdata):
        rospy.loginfo("Starting formation assembly...")
        
        try:
            # Use existing assembly functionality
            assembly_demo = AssemblyDemo(module_ids=[1, 2], real_machine=False)
            
            # Execute assembly
            result = assembly_demo.main()
            
            if result:
                rospy.loginfo("Formation assembly completed")
                time.sleep(2)  # Wait for system stabilization
                return 'succeeded'
            else:
                rospy.logerr("Formation assembly failed")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during assembly process: {e}")
            return 'failed'

class EnableUnifiedControlState(smach.State):
    """Enable unified control state"""
    
    def __init__(self, formation_controller):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.formation_controller = formation_controller
        
    def execute(self, userdata):
        rospy.loginfo("Enabling unified 4n-rotor control mode...")
        
        try:
            # Enable unified control
            self.formation_controller.enable_unified_control()
            time.sleep(1)  # Give system time to switch mode
            
            # Configure advanced parameters
            rospy.set_param("controller/unified_control_advanced/dynamic_tilt_optimization", True)
            rospy.set_param("controller/unified_control_advanced/max_rotor_tilt_angle", 0.3)
            rospy.set_param("controller/unified_control_advanced/attitude_stability_weight", 8.0)
            rospy.set_param("controller/unified_control_advanced/torque_optimization_weight", 3.0)
            
            rospy.loginfo("Unified control mode enabled, parameters configured")
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f"Failed to enable unified control: {e}")
            return 'failed'

class FormationValveRotationState(AssemblyMotionStateBase):
    """Formation valve rotation state"""
    
    def __init__(self, formation_controller, rotation_angle=90.0):
        AssemblyMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])
        self.formation_controller = formation_controller
        self.rotation_angle = rotation_angle  # degrees
        
    def execute(self, userdata):
        rospy.loginfo(f"Starting formation valve rotation task (target angle: {self.rotation_angle} degrees)...")
        
        try:
            # 1. Enable valve rotation feedforward compensation
            self.formation_controller.set_valve_rotation_feedforward(
                valve_torque=4.0,  # Larger torque for dual UAV
                valve_force_z=-10.0  # Larger resistance force
            )
            time.sleep(1)
            
            # 2. Execute valve rotation
            rospy.loginfo("Starting valve rotation...")
            
            # Here should call your existing valve rotation logic
            # but adapted to formation control mode
            rotation_success = self._execute_valve_rotation()
            
            if rotation_success:
                rospy.loginfo("Valve rotation task completed")
                return 'succeeded'
            else:
                rospy.logerr("Valve rotation task failed")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during valve rotation process: {e}")
            return 'failed'
        finally:
            # Disable feedforward compensation
            self.formation_controller.disable_feedforward()
            
    def _execute_valve_rotation(self):
        """Execute actual valve rotation motion"""
        # Here needs to implement specific valve rotation logic
        # Can refer to your valve_rotation_fang_single.py implementation
        
        # Simulate rotation process
        rotation_steps = 20
        step_angle = self.rotation_angle / rotation_steps
        
        for step in range(rotation_steps):
            if rospy.is_shutdown():
                return False
                
            # Gradually increase feedforward compensation, simulating changing rotation resistance
            current_progress = step / float(rotation_steps)
            
            # Dynamically adjust feedforward torque
            dynamic_torque = 4.0 + current_progress * 2.0  # From 4Nm to 6Nm
            dynamic_force_z = -10.0 - current_progress * 3.0  # From -10N to -13N
            
            self.formation_controller.set_valve_rotation_feedforward(
                valve_torque=dynamic_torque,
                valve_force_z=dynamic_force_z
            )
            
            rospy.loginfo(f"Rotation progress: {current_progress*100:.1f}% "
                         f"(torque: {dynamic_torque:.1f}Nm, force: {dynamic_force_z:.1f}N)")
            
            time.sleep(0.5)  # 0.5 seconds per step, 10 seconds total
            
        rospy.loginfo("Valve rotation completed")
        return True

class DisassemblyState(AssemblyMotionStateBase):
    """Disassembly state"""
    
    def __init__(self, formation_controller):
        AssemblyMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])
        self.formation_controller = formation_controller
        
    def execute(self, userdata):
        rospy.loginfo("Starting disassembly...")
        
        try:
            # 1. Disable unified control
            self.formation_controller.disable_unified_control()
            time.sleep(1)
            
            # 2. Execute disassembly
            disassembly_demo = DisassemblyDemo(module_ids=[1, 2], real_machine=False)
            result = disassembly_demo.main()
            
            if result:
                rospy.loginfo("Disassembly completed")
                return 'succeeded'
            else:
                rospy.logerr("Disassembly failed")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during disassembly process: {e}")
            return 'failed'

def create_formation_valve_rotation_sm():
    """Create dual UAV formation valve rotation state machine"""
    
    # Create formation controller
    formation_controller = FormationValveRotationController()
    
    # Create top-level state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'aborted'])
    
    with sm:
        # 1. Move to assembly position
        smach.StateMachine.add(
            'MOVE_TO_ASSEMBLY_POSITION',
            MoveToAssemblyPositionState(),
            transitions={
                'succeeded': 'ASSEMBLY_FORMATION',
                'failed': 'failed'
            }
        )
        
        # 2. Execute formation assembly
        smach.StateMachine.add(
            'ASSEMBLY_FORMATION',
            AssemblyFormationState(),
            transitions={
                'succeeded': 'ENABLE_UNIFIED_CONTROL',
                'failed': 'failed'
            }
        )
        
        # 3. Enable unified control
        smach.StateMachine.add(
            'ENABLE_UNIFIED_CONTROL',
            EnableUnifiedControlState(formation_controller),
            transitions={
                'succeeded': 'FORMATION_VALVE_ROTATION',
                'failed': 'DISASSEMBLY'
            }
        )
        
        # 4. Formation valve rotation
        smach.StateMachine.add(
            'FORMATION_VALVE_ROTATION',
            FormationValveRotationState(formation_controller),
            transitions={
                'succeeded': 'DISASSEMBLY',
                'failed': 'DISASSEMBLY'
            }
        )
        
        # 5. Disassembly
        smach.StateMachine.add(
            'DISASSEMBLY',
            DisassemblyState(formation_controller),
            transitions={
                'succeeded': 'succeeded',
                'failed': 'failed'
            }
        )
    
    return sm

def main():
    """Main function"""
    rospy.init_node('formation_valve_rotation_smach')
    
    try:
        # Create state machine
        sm = create_formation_valve_rotation_sm()
        
        # Create state machine visualization
        sis = smach_ros.IntrospectionServer('formation_valve_rotation', sm, '/SM_ROOT')
        sis.start()
        
        rospy.loginfo("Dual UAV formation valve rotation task started...")
        rospy.loginfo("State machine visualization: rosrun smach_viewer smach_viewer.py")
        
        # Execute state machine
        outcome = sm.execute()
        
        rospy.loginfo(f"Task completed with result: {outcome}")
        
        # Keep visualization server running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Task interrupted")
    except Exception as e:
        rospy.logerr(f"Error during task execution: {e}")
    finally:
        try:
            sis.stop()
        except:
            pass

if __name__ == '__main__':
    main()
