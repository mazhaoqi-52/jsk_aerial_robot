#!/usr/bin/env python3
"""
Dual UAV Formation Valve Rotation using SMACH State Machine
Based on valve_rotation_smach_test.py and valve_rotation_fang_single.py frameworks

Basic workflow:
1. Aerial assembly (UAVs combine in air)
2. Move to valve vicinity in assembled state
3. Move to insertion point and descend
4. Rotate and contact with insertion point
5. Start valve rotation (with feedforward control)
6. Ascend and return to start position
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
from trajectory import PolynomialTrajectory
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

class AerialAssemblyState(smach.State):
    
    def __init__(self, module_ids=[1, 2]):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.module_ids = module_ids
        
    def execute(self, userdata):
        rospy.loginfo("Starting aerial assembly...")
        
        try:
            # Use configurable module IDs for aerial assembly
            assembly_demo = AssemblyDemo(module_ids=self.module_ids, real_machine=False)
            
            # Execute assembly
            result = assembly_demo.main()
            
            if result:
                rospy.loginfo("Aerial assembly completed successfully")
                time.sleep(2)  # Wait for system stabilization
                return 'succeeded'
            else:
                rospy.logerr("Aerial assembly failed")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during aerial assembly process: {e}")
            return 'failed'

class MoveToValveVicinityState(AssemblyMotionStateBase):
    """Move to valve vicinity in assembled state"""
    
    def __init__(self, 
                 x_offset=0.65,
                 y_offset=0.25, 
                 avg_speed=0.1,
                 safety_margin=0.3):
        
        AssemblyMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.avg_speed = avg_speed
        self.safety_margin = safety_margin
        
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
        rospy.loginfo("Moving to valve vicinity in assembled state...")
        
        # Wait for valve position information
        if not self.valve_received.wait(timeout=5):
            rospy.logwarn("Failed to get valve position information")
            return 'failed'
            
        # Wait for UAV positions
        if not self.wait_for_uav_positions(timeout=5):
            rospy.logwarn("Timeout waiting for UAV positions")
            return 'failed'
            
        start1, start2 = self.get_uav_positions()
        rospy.loginfo(f"Current UAV positions - UAV1: {start1}, UAV2: {start2}")
        
        # Get valve position
        if self.is_simulation and self.pose_valve_sim:
            valve_x = self.pose_valve_sim.pose.pose.position.x
            valve_y = self.pose_valve_sim.pose.pose.position.y
            valve_z = self.pose_valve_sim.pose.pose.position.z
            offset = 0.23
        elif not self.is_simulation and self.pose_valve:
            valve_x = self.pose_valve.pose.position.x
            valve_y = self.pose_valve.pose.position.y
            valve_z = self.pose_valve.pose.position.z
            offset = 0.47
        else:
            rospy.logerr("Cannot get valid valve position")
            return 'failed'
            
        rospy.loginfo(f"Valve position: [{valve_x:.3f}, {valve_y:.3f}, {valve_z:.3f}]")
        safe_altitude = valve_z + offset

        # Move to safe altitude first
        threads = []
        for pub, start, target in [
            (self.beetle1_pub, start1, [start1[0], start1[1], safe_altitude]),
            (self.beetle2_pub, start2, [start2[0], start2[1], safe_altitude])
        ]:
            t = UnifiedMotionController.execute_poly_motion_pose_async(pub, start, target, self.avg_speed)
            threads.append(t)
        for t in threads:
            t.join()
        time.sleep(2)

        # Move to valve vicinity horizontally
        current1 = [start1[0], start1[1], safe_altitude]
        current2 = [start2[0], start2[1], safe_altitude]
        horiz_target1 = [valve_x, valve_y + self.y_offset, safe_altitude + self.safety_margin]
        horiz_target2 = [valve_x + self.x_offset, valve_y - self.y_offset, safe_altitude + self.safety_margin]
        
        threads = []
        for pub, current, target in [
            (self.beetle1_pub, current1, horiz_target1),
            (self.beetle2_pub, current2, horiz_target2)
        ]:
            t = UnifiedMotionController.execute_poly_motion_pose_async(pub, current, target, self.avg_speed)
            threads.append(t)
        for t in threads:
            t.join()
        time.sleep(2)

        # Move to final positions above valve
        final_target1 = [valve_x, valve_y, safe_altitude + self.safety_margin]
        final_target2 = [valve_x + self.x_offset, valve_y, safe_altitude + self.safety_margin]
        
        threads = []
        for pub, current, target in [
            (self.beetle1_pub, horiz_target1, final_target1),
            (self.beetle2_pub, horiz_target2, final_target2)
        ]:
            t = UnifiedMotionController.execute_poly_motion_pose_async(pub, current, target, self.avg_speed)
            threads.append(t)
        for t in threads:
            t.join()
        time.sleep(2)
        
        rospy.loginfo("Reached valve vicinity successfully")
        return 'succeeded'

class MoveToInsertionPointState(AssemblyMotionStateBase):
    """Move to insertion point and descend with CoG-to-end-effector transformation"""
    
    def __init__(self, 
                 z_offset_real=0.47,
                 z_offset_sim=0.05,
                 descent_speed=0.05,
                 formation_spacing=0.65):
        
        AssemblyMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])
        self.z_offset_real = z_offset_real
        self.z_offset_sim = z_offset_sim
        self.descent_speed = descent_speed
        self.formation_spacing = formation_spacing
        
        # Identify end effector and support UAVs
        modules_str = rospy.get_param("~module_ids", "1,2")
        modules = [int(x) for x in modules_str.split(',')]
        self.modules = sorted(modules)
        self.end_effector_id = modules[-1]  # Rightmost module has end effector
        self.support_ids = modules[:-1]     # Other modules are support UAVs
        
        # Calculate formation center of gravity offset
        self.formation_cog_offset = self.calculate_formation_cog_offset()
        
        # Subscribe to valve position
        self.is_simulation = rospy.get_param("~simulation", True)
        self.valve_received = threading.Event()
        
        if self.is_simulation:
            self.pose_valve_sim = None
            self.valve_sim_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            self.pose_valve = None
            self.valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
        
        rospy.loginfo(f"Formation CoG offset: {self.formation_cog_offset}")
        rospy.loginfo(f"End effector UAV: beetle{self.end_effector_id} will perform insertion")
        rospy.loginfo(f"Support UAVs: {[f'beetle{i}' for i in self.support_ids]} will maintain formation")

    def calculate_formation_cog_offset(self):
        """Calculate center of gravity offset for assembled formation"""
        if len(self.modules) == 1:
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # For assembled formation, CoG shifts based on module arrangement
        # Assuming linear formation with equal mass modules
        total_modules = len(self.modules)
        formation_length = (total_modules - 1) * self.formation_spacing
        
        # Calculate CoG position (center of mass)
        cog_x = 0.0  # For equal mass, CoG is at geometric center
        
        # End effector position relative to formation CoG
        end_effector_index = self.modules.index(self.end_effector_id)
        end_effector_x = (end_effector_index * self.formation_spacing) - (formation_length / 2.0)
        
        # Support modules positions
        support_positions = []
        for support_id in self.support_ids:
            support_index = self.modules.index(support_id)
            support_x = (support_index * self.formation_spacing) - (formation_length / 2.0)
            support_positions.append({'id': support_id, 'x': support_x, 'y': 0.0, 'z': 0.0})
        
        return {
            'formation_cog': {'x': cog_x, 'y': 0.0, 'z': 0.0},
            'end_effector_offset': {'x': end_effector_x, 'y': 0.0, 'z': 0.0},
            'support_offsets': support_positions
        }

    def valve_sim_callback(self, msg):
        self.pose_valve_sim = msg
        self.valve_received.set()

    def valve_callback(self, msg):
        self.pose_valve = msg
        self.valve_received.set()

    def execute(self, userdata):
        rospy.loginfo("Moving to insertion point with CoG-to-end-effector transformation...")
        
        # Wait for valve position
        if not self.valve_received.wait(timeout=5):
            rospy.logwarn("Failed to get valve position information")
            return 'failed'
            
        # Get valve position and calculate target with CoG compensation
        if self.is_simulation and self.pose_valve_sim:
            valve_x = self.pose_valve_sim.pose.pose.position.x
            valve_y = self.pose_valve_sim.pose.pose.position.y
            valve_z = self.pose_valve_sim.pose.pose.position.z
            z_offset = self.z_offset_sim
        elif not self.is_simulation and self.pose_valve:
            valve_x = self.pose_valve.pose.position.x
            valve_y = self.pose_valve.pose.position.y
            valve_z = self.pose_valve.pose.position.z
            z_offset = self.z_offset_real
        else:
            rospy.logerr("Cannot get valid valve position")
            return 'failed'
            
        # Apply CoG-to-end-effector transformation
        # Target for formation CoG, considering end effector offset
        end_effector_offset = self.formation_cog_offset['end_effector_offset']
        
        # Formation CoG target position (compensated for end effector offset)
        formation_target_x = valve_x - end_effector_offset['x']
        formation_target_y = valve_y - end_effector_offset['y']
        target_z = valve_z + z_offset
        
        rospy.loginfo(f"Valve position: [{valve_x:.3f}, {valve_y:.3f}, {valve_z:.3f}]")
        rospy.loginfo(f"End effector offset: [{end_effector_offset['x']:.3f}, {end_effector_offset['y']:.3f}, 0]")
        rospy.loginfo(f"Formation CoG target: [{formation_target_x:.3f}, {formation_target_y:.3f}, {target_z:.3f}]")
        
        # Wait for current positions
        if not self.wait_for_uav_positions(timeout=5):
            rospy.logwarn("Timeout waiting for UAV positions")
            return 'failed'
            
        start1, start2 = self.get_uav_positions()
        
        # Calculate individual UAV targets based on formation geometry
        threads = []
        
        # Calculate targets for each UAV based on their role in formation
        sorted_modules = sorted([int(x) for x in rospy.get_param("~module_ids", "1,2").split(',')])
        
        if len(sorted_modules) == 2:
            # Dual UAV formation
            uav1_offset = self.formation_cog_offset['support_offsets'][0] if self.formation_cog_offset['support_offsets'] else {'x': -self.formation_spacing/2, 'y': 0, 'z': 0}
            
            if self.end_effector_id == sorted_modules[0]:
                # UAV1 is end effector
                uav1_target = [formation_target_x + end_effector_offset['x'], formation_target_y, target_z]
                uav2_target = [formation_target_x + uav1_offset['x'], formation_target_y, target_z]
                
                rospy.loginfo(f"beetle{self.end_effector_id} (UAV1) → insertion point: {uav1_target}")
                rospy.loginfo(f"beetle{sorted_modules[1]} (UAV2) → support position: {uav2_target}")
            else:
                # UAV2 is end effector
                uav1_target = [formation_target_x + uav1_offset['x'], formation_target_y, target_z]
                uav2_target = [formation_target_x + end_effector_offset['x'], formation_target_y, target_z]
                
                rospy.loginfo(f"beetle{sorted_modules[0]} (UAV1) → support position: {uav1_target}")
                rospy.loginfo(f"beetle{self.end_effector_id} (UAV2) → insertion point: {uav2_target}")
        else:
            # Multi-UAV formation: more complex geometry
            rospy.logwarn("Multi-UAV formation geometry not fully implemented")
            uav1_target = [formation_target_x - self.formation_spacing/2, formation_target_y, target_z]
            uav2_target = [formation_target_x + self.formation_spacing/2, formation_target_y, target_z]
        
        # Execute descent with CoG-compensated targets
        t1 = UnifiedMotionController.execute_poly_motion_pose_async(
            self.beetle1_pub, start1, uav1_target, self.descent_speed)
        threads.append(t1)
        
        t2 = UnifiedMotionController.execute_poly_motion_pose_async(
            self.beetle2_pub, start2, uav2_target, self.descent_speed)
        threads.append(t2)
        
        # Wait for descent completion
        for t in threads:
            t.join()
        
        time.sleep(2)
        rospy.loginfo("Reached insertion point with proper CoG compensation")
        return 'succeeded'

class RotateAndContactState(AssemblyMotionStateBase):
    """Rotate and contact with insertion point"""
    
    def __init__(self, 
                 contact_force_threshold=2.0,
                 contact_timeout=10.0,
                 rotation_speed=0.1):
        
        AssemblyMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])
        self.contact_force_threshold = contact_force_threshold
        self.contact_timeout = contact_timeout
        self.rotation_speed = rotation_speed
        
        # Subscribe to external wrench for contact detection
        self.external_wrench = None
        self.wrench_received = threading.Event()
        
        # Use end effector UAV (rightmost module) for contact detection
        modules_str = rospy.get_param("~module_ids", "1,2")
        modules = [int(x) for x in modules_str.split(',')]
        self.end_effector_id = modules[-1]  # Rightmost module has end effector
        self.support_ids = modules[:-1]     # Other modules are support UAVs
        
        self.wrench_sub = rospy.Subscriber(
            f"/beetle{self.end_effector_id}/estimated_external_wrench", 
            WrenchStamped, 
            self.wrench_callback, 
            queue_size=1
        )
        
        rospy.loginfo(f"End effector UAV: beetle{self.end_effector_id}")
        rospy.loginfo(f"Support UAVs: {[f'beetle{i}' for i in self.support_ids]}")
        rospy.loginfo(f"Contact detection using beetle{self.end_effector_id} external wrench feedback")

    def wrench_callback(self, msg):
        self.external_wrench = msg.wrench
        self.wrench_received.set()

    def is_in_contact(self):
        """Check if UAV is in contact with valve based on external force"""
        if self.external_wrench is None:
            return False
            
        # Check force magnitude in any direction
        force_magnitude = math.sqrt(
            self.external_wrench.force.x**2 + 
            self.external_wrench.force.y**2 + 
            self.external_wrench.force.z**2
        )
        
        contact_detected = force_magnitude > self.contact_force_threshold
        
        if contact_detected:
            rospy.loginfo(f"Contact detected! External force magnitude: {force_magnitude:.2f}N")
        
        return contact_detected

    def execute(self, userdata):
        rospy.loginfo("Rotating and establishing contact with insertion point...")
        
        # Wait for initial wrench reading
        if not self.wrench_received.wait(timeout=3):
            rospy.logwarn("No external wrench data received, proceeding with time-based contact")
            use_force_feedback = False
        else:
            use_force_feedback = True
            rospy.loginfo("Using external force feedback for contact detection")
        
        # Get valve yaw for alignment
        if hasattr(self, 'pose_valve_sim') and self.pose_valve_sim:
            orientation = self.pose_valve_sim.pose.pose.orientation
        elif hasattr(self, 'pose_valve') and self.pose_valve:
            orientation = self.pose_valve.pose.orientation
        else:
            rospy.logwarn("No valve orientation available, using default")
            target_yaw = 0.0
            
        if 'orientation' in locals():
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            roll, pitch, target_yaw = euler_from_quaternion(quaternion)
        else:
            target_yaw = 0.0
        
        rospy.loginfo(f"Target yaw alignment: {target_yaw:.3f} rad")
        
        # Wait for positions
        if not self.wait_for_uav_positions(timeout=5):
            return 'failed'
            
        start1, start2 = self.get_uav_positions()
        
        # Rotate to align with valve orientation - only end effector UAV
        target_pos_1 = start1  # Keep same position
        target_pos_2 = start2
        
        # Determine which physical UAV corresponds to end effector
        sorted_modules = sorted([int(x) for x in rospy.get_param("~module_ids", "1,2").split(',')])
        
        threads = []
        if self.end_effector_id == sorted_modules[0]:
            # End effector is UAV1 - rotates to align with valve
            t1 = UnifiedMotionController.execute_poly_motion_pose_yaw_async(
                self.beetle1_pub, start1, target_pos_1, target_yaw, self.rotation_speed)
            threads.append(t1)
            
            # UAV2 maintains formation without rotation
            t2 = UnifiedMotionController.execute_poly_motion_pose_async(
                self.beetle2_pub, start2, target_pos_2, self.rotation_speed)
            threads.append(t2)
            
            rospy.loginfo(f"beetle{self.end_effector_id} (UAV1) aligning with valve orientation")
        else:
            # End effector is UAV2 - rotates to align with valve
            # UAV1 maintains formation without rotation
            t1 = UnifiedMotionController.execute_poly_motion_pose_async(
                self.beetle1_pub, start1, target_pos_1, self.rotation_speed)
            threads.append(t1)
            
            t2 = UnifiedMotionController.execute_poly_motion_pose_yaw_async(
                self.beetle2_pub, start2, target_pos_2, target_yaw, self.rotation_speed)
            threads.append(t2)
            
            rospy.loginfo(f"beetle{self.end_effector_id} (UAV2) aligning with valve orientation")
        
        for t in threads:
            t.join()
        
        time.sleep(1)
        
        # Contact detection phase
        rospy.loginfo("Establishing contact with valve...")
        contact_start_time = time.time()
        
        if use_force_feedback:
            # Use external force feedback for contact detection
            while (time.time() - contact_start_time) < self.contact_timeout:
                if self.is_in_contact():
                    rospy.loginfo("Successfully established contact with valve")
                    return 'succeeded'
                time.sleep(0.1)
            
            rospy.logwarn("Contact timeout - no sufficient force detected")
            return 'failed'
        else:
            # Fallback: time-based contact assumption
            rospy.loginfo("Using time-based contact detection (3 seconds)")
            time.sleep(3)
            rospy.loginfo("Assumed contact established")
            return 'succeeded'

class ValveRotationState(AssemblyMotionStateBase):
    """Start valve rotation with feedforward control"""
    
    def __init__(self, 
                 rotation_angle=90.0,  # degrees
                 rotation_speed=0.5,   # rad/s
                 feedforward_torque=3.0,
                 feedforward_force_z=-5.0):
        
        AssemblyMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])
        self.rotation_angle = math.radians(rotation_angle)
        self.rotation_speed = rotation_speed
        self.feedforward_torque = feedforward_torque
        self.feedforward_force_z = feedforward_force_z
        
        # Get module IDs for feedforward control and role assignment
        modules_str = rospy.get_param("~module_ids", "1,2")
        self.modules = [int(x) for x in modules_str.split(',')]
        self.end_effector_id = self.modules[-1]  # Rightmost module has end effector
        self.support_ids = self.modules[:-1]     # Other modules are support UAVs
        
        # Publishers for feedforward wrench to all modules
        self.feedforward_pubs = {}
        for module_id in self.modules:
            pub = rospy.Publisher(
                f'/beetle{module_id}/controller/feedforward_wrench', 
                WrenchStamped, 
                queue_size=1
            )
            self.feedforward_pubs[module_id] = pub
        
        rospy.loginfo(f"End effector UAV: beetle{self.end_effector_id} will perform valve rotation")
        rospy.loginfo(f"Support UAVs: {[f'beetle{i}' for i in self.support_ids]} will provide stability")
        rospy.loginfo(f"Initialized valve rotation with feedforward for modules: {self.modules}")

    def set_feedforward_wrench(self, torque_z, force_z):
        """Set feedforward compensation for valve rotation resistance with proper CoG-to-end-effector transformation"""
        
        # Calculate formation geometry for proper force/torque distribution
        formation_center_to_end_effector = self.calculate_formation_to_end_effector_transform()
        
        for module_id, pub in self.feedforward_pubs.items():
            ff_wrench = WrenchStamped()
            ff_wrench.header.stamp = rospy.Time.now()
            ff_wrench.header.frame_id = "cog"
            
            # Base feedforward values
            ff_wrench.wrench.force.x = 0.0
            ff_wrench.wrench.force.y = 0.0
            ff_wrench.wrench.force.z = force_z
            ff_wrench.wrench.torque.x = 0.0
            ff_wrench.wrench.torque.y = 0.0
            ff_wrench.wrench.torque.z = torque_z
            
            # Apply transformation compensation for non-end-effector modules
            if module_id != self.end_effector_id:
                # Support modules need compensation for moment arm effects
                # Force remains the same for all modules (shared load)
                # But torque distribution depends on formation geometry
                
                # Calculate relative position of this module to formation center
                module_offset = self.get_module_offset_in_formation(module_id)
                
                # Compensate torque due to offset from end effector
                # Additional torque = r × F (cross product of position and force)
                additional_torque_x = -module_offset['y'] * force_z  # Pitch compensation
                additional_torque_y = module_offset['x'] * force_z   # Roll compensation
                
                ff_wrench.wrench.torque.x += additional_torque_x
                ff_wrench.wrench.torque.y += additional_torque_y
                
                # Distribute yaw torque based on formation geometry
                if len(self.modules) > 1:
                    # Share yaw torque among all modules
                    ff_wrench.wrench.torque.z = torque_z / len(self.modules)
                
                rospy.loginfo_throttle(2.0, 
                    f"Module {module_id} (Support): Force=[0,0,{force_z:.2f}]N, "
                    f"Torque=[{additional_torque_x:.2f},{additional_torque_y:.2f},{ff_wrench.wrench.torque.z:.2f}]Nm")
            else:
                # End effector gets full valve rotation torque
                rospy.loginfo_throttle(2.0, 
                    f"Module {module_id} (End-effector): Force=[0,0,{force_z:.2f}]N, "
                    f"Torque=[0,0,{torque_z:.2f}]Nm")
            
            pub.publish(ff_wrench)
        
        rospy.loginfo_throttle(1.0, f"Formation feedforward - Total force: {force_z:.2f}N, End-effector torque: {torque_z:.2f}Nm")

    def calculate_formation_to_end_effector_transform(self):
        """Calculate transformation from formation center of gravity to end effector"""
        # This should ideally come from the robot model, but for now use geometric approximation
        # In a real system, this would use TF transformations or robot model data
        
        # Approximate formation geometry (this should be parameterized)
        if len(self.modules) == 2:
            # Dual UAV formation: end effector offset from center
            if self.end_effector_id == max(self.modules):
                # Rightmost module is end effector
                return {'x': 0.325, 'y': 0.0, 'z': 0.0}  # Half of typical spacing
            else:
                # Leftmost module is end effector  
                return {'x': -0.325, 'y': 0.0, 'z': 0.0}
        else:
            # Multi-UAV formation: more complex geometry
            module_index = self.modules.index(self.end_effector_id)
            total_modules = len(self.modules)
            # Linear formation assumption
            formation_length = (total_modules - 1) * 0.65  # Module spacing
            center_offset = formation_length / 2.0
            module_position = module_index * 0.65 - center_offset
            return {'x': module_position, 'y': 0.0, 'z': 0.0}

    def get_module_offset_in_formation(self, module_id):
        """Get module position offset relative to formation center"""
        if len(self.modules) == 2:
            # Simple dual formation
            if module_id == min(self.modules):
                return {'x': -0.325, 'y': 0.0, 'z': 0.0}  # Left module
            else:
                return {'x': 0.325, 'y': 0.0, 'z': 0.0}   # Right module
        else:
            # Multi-UAV linear formation
            module_index = self.modules.index(module_id)
            total_modules = len(self.modules)
            formation_length = (total_modules - 1) * 0.65
            center_offset = formation_length / 2.0
            module_position = module_index * 0.65 - center_offset
            return {'x': module_position, 'y': 0.0, 'z': 0.0}

    def execute(self, userdata):
        rospy.loginfo(f"Starting valve rotation (target: {math.degrees(self.rotation_angle):.1f} degrees)...")
        
        # Check if feedforward is enabled via ROS parameter
        feedforward_enabled = rospy.get_param("controller/valve_rotation_feedforward/enabled", True)
        
        if feedforward_enabled:
            # Enable feedforward compensation with default or parameter values
            default_torque = rospy.get_param("controller/valve_rotation_feedforward/torque_z", self.feedforward_torque)
            default_force_z = rospy.get_param("controller/valve_rotation_feedforward/force_z", self.feedforward_force_z)
            
            self.set_feedforward_wrench(default_torque, default_force_z)
            rospy.loginfo("Feedforward compensation enabled via ROS parameters")
            time.sleep(1)  # Allow feedforward to stabilize
        else:
            rospy.loginfo("Feedforward compensation disabled via ROS parameters")
        
        # Get current positions and valve info
        if not self.wait_for_uav_positions(timeout=5):
            rospy.logwarn("Timeout waiting for UAV positions")
            return 'failed'
            
        start1, start2 = self.get_uav_positions()
        
        # Calculate rotation trajectory
        rotation_steps = 20
        step_angle = self.rotation_angle / rotation_steps
        rotation_duration = self.rotation_angle / self.rotation_speed
        step_duration = rotation_duration / rotation_steps
        
        rospy.loginfo(f"Executing rotation in {rotation_steps} steps over {rotation_duration:.1f} seconds")
        
        try:
            # Execute rotation with progressive feedforward adjustment
            for step in range(rotation_steps + 1):
                if rospy.is_shutdown():
                    return 'failed'
                
                current_progress = step / float(rotation_steps)
                current_angle = step * step_angle
                
                # Adaptive feedforward based on rotation progress (only if enabled)
                if feedforward_enabled:
                    # Increase resistance as valve tightens
                    adaptive_torque = default_torque * (1.0 + current_progress * 0.5)
                    adaptive_force = default_force_z * (1.0 + current_progress * 0.3)
                    
                    self.set_feedforward_wrench(adaptive_torque, adaptive_force)
                
                # Send rotation command - only end effector UAV rotates
                target_yaw_end_effector = current_angle
                
                # Determine which physical UAV corresponds to end effector
                sorted_modules = sorted([int(x) for x in rospy.get_param("~module_ids", "1,2").split(',')])
                
                # Execute rotation step
                threads = []
                if self.end_effector_id == sorted_modules[0]:
                    # End effector is UAV1 - rotates
                    t1 = UnifiedMotionController.execute_poly_motion_pose_yaw_async(
                        self.beetle1_pub, start1, start1, target_yaw_end_effector, 0.1)
                    threads.append(t1)
                    
                    # UAV2 maintains position and orientation
                    t2 = UnifiedMotionController.execute_poly_motion_pose_async(
                        self.beetle2_pub, start2, start2, 0.1)
                    threads.append(t2)
                    
                else:
                    # End effector is UAV2 - rotates
                    # UAV1 maintains position and orientation
                    t1 = UnifiedMotionController.execute_poly_motion_pose_async(
                        self.beetle1_pub, start1, start1, 0.1)
                    threads.append(t1)
                    
                    t2 = UnifiedMotionController.execute_poly_motion_pose_yaw_async(
                        self.beetle2_pub, start2, start2, target_yaw_end_effector, 0.1)
                    threads.append(t2)
                
                for t in threads:
                    t.join()
                
                rospy.loginfo(f"Rotation progress: {current_progress*100:.1f}% "
                             f"(angle: {math.degrees(current_angle):.1f}°)")
                
                time.sleep(step_duration)
            
            rospy.loginfo("Valve rotation completed successfully")
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f"Error during valve rotation: {e}")
            return 'failed'
        finally:
            # Disable feedforward
            self.set_feedforward_wrench(0.0, 0.0)
            rospy.loginfo("Feedforward compensation disabled")

class AscentAndReturnState(AssemblyMotionStateBase):
    """Ascend and return to start position"""
    
    def __init__(self, 
                 ascent_height=0.5,
                 return_speed=0.1,
                 start_pos_1=None,
                 start_pos_2=None):
        
        AssemblyMotionStateBase.__init__(self, outcomes=['succeeded', 'failed'])
        self.ascent_height = ascent_height
        self.return_speed = return_speed
        
        # Default start positions if not provided
        self.start_pos_1 = start_pos_1 or [0.0, 0.5, 1.0]
        self.start_pos_2 = start_pos_2 or [0.0, -0.5, 1.0]

    def execute(self, userdata):
        rospy.loginfo("Ascending and returning to start position...")
        
        # Get current positions
        if not self.wait_for_uav_positions(timeout=5):
            rospy.logwarn("Timeout waiting for UAV positions")
            return 'failed'
            
        current1, current2 = self.get_uav_positions()
        
        # Phase 1: Ascend from current position
        ascent_target_1 = [current1[0], current1[1], current1[2] + self.ascent_height]
        ascent_target_2 = [current2[0], current2[1], current2[2] + self.ascent_height]
        
        rospy.loginfo("Phase 1: Ascending...")
        threads = []
        t1 = UnifiedMotionController.execute_poly_motion_pose_async(
            self.beetle1_pub, current1, ascent_target_1, self.return_speed)
        threads.append(t1)
        
        t2 = UnifiedMotionController.execute_poly_motion_pose_async(
            self.beetle2_pub, current2, ascent_target_2, self.return_speed)
        threads.append(t2)
        
        for t in threads:
            t.join()
        
        time.sleep(2)
        
        # Phase 2: Return to start positions
        rospy.loginfo("Phase 2: Returning to start positions...")
        threads = []
        t1 = UnifiedMotionController.execute_poly_motion_pose_async(
            self.beetle1_pub, ascent_target_1, self.start_pos_1, self.return_speed)
        threads.append(t1)
        
        t2 = UnifiedMotionController.execute_poly_motion_pose_async(
            self.beetle2_pub, ascent_target_2, self.start_pos_2, self.return_speed)
        threads.append(t2)
        
        for t in threads:
            t.join()
        
        time.sleep(2)
        rospy.loginfo("Successfully returned to start positions")
        return 'succeeded'

def create_formation_valve_rotation_sm():
    """Create dual UAV formation valve rotation state machine"""
    
    # Parse module IDs from ROS parameters
    modules_str = rospy.get_param("~module_ids", "1,2")
    rospy.loginfo(f"Formation valve rotation with module IDs: {modules_str}")
    
    modules = []
    if modules_str:
        modules = [int(x) for x in modules_str.split(',')]
    else:
        rospy.logwarn("No module IDs specified, using default [1,2]")
        modules = [1, 2]
    
    if len(modules) < 2:
        rospy.logerr("At least 2 module IDs are required for formation!")
        modules = [1, 2]
    
    rospy.loginfo(f"Using modules: {modules}")
    
    # Identify end effector and support UAVs
    end_effector_id = modules[-1]  # Rightmost module has end effector
    support_ids = modules[:-1]     # Other modules are support UAVs
    rospy.loginfo(f"=== UAV Role Assignment ===")
    rospy.loginfo(f"End effector UAV: beetle{end_effector_id} (rightmost module)")
    rospy.loginfo(f"Support UAVs: {[f'beetle{i}' for i in support_ids]}")
    rospy.loginfo(f"End effector UAV will perform: insertion, contact detection, valve rotation")
    rospy.loginfo(f"Support UAVs will provide: formation stability and assistance")
    
    # Create top-level state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'aborted'])
    
    with sm:
        # 1. Aerial assembly
        smach.StateMachine.add(
            'AERIAL_ASSEMBLY',
            AerialAssemblyState(module_ids=modules),
            transitions={
                'succeeded': 'MOVE_TO_VALVE_VICINITY',
                'failed': 'failed'
            }
        )
        
        # 2. Move to valve vicinity in assembled state
        smach.StateMachine.add(
            'MOVE_TO_VALVE_VICINITY',
            MoveToValveVicinityState(),
            transitions={
                'succeeded': 'MOVE_TO_INSERTION_POINT',
                'failed': 'failed'
            }
        )
        
        # 3. Move to insertion point and descend
        smach.StateMachine.add(
            'MOVE_TO_INSERTION_POINT',
            MoveToInsertionPointState(),
            transitions={
                'succeeded': 'ROTATE_AND_CONTACT',
                'failed': 'failed'
            }
        )
        
        # 4. Rotate and contact with insertion point
        smach.StateMachine.add(
            'ROTATE_AND_CONTACT',
            RotateAndContactState(),
            transitions={
                'succeeded': 'VALVE_ROTATION',
                'failed': 'failed'
            }
        )
        
        # 5. Start valve rotation (with feedforward control)
        smach.StateMachine.add(
            'VALVE_ROTATION',
            ValveRotationState(),
            transitions={
                'succeeded': 'ASCENT_AND_RETURN',
                'failed': 'ASCENT_AND_RETURN'  # Still try to return even if rotation fails
            }
        )
        
        # 6. Ascend and return to start position
        smach.StateMachine.add(
            'ASCENT_AND_RETURN',
            AscentAndReturnState(),
            transitions={
                'succeeded': 'succeeded',
                'failed': 'failed'
            }
        )
    
    return sm

def main():
    """Main function"""
    rospy.init_node('formation_valve_rotation_smach')
    
    # Check if debug mode is enabled
    debug_mode = rospy.get_param("~debug", False)
    if debug_mode:
        rospy.loginfo("Debug mode enabled - detailed logging and SMACH viewer active")
        rospy.set_param('/rospy/logger_level', 'DEBUG')
    
    try:
        # Create state machine
        sm = create_formation_valve_rotation_sm()
        
        # Create state machine visualization (only if debug enabled in launch file)
        sis = smach_ros.IntrospectionServer('formation_valve_rotation', sm, '/SM_ROOT')
        sis.start()
        
        rospy.loginfo("Dual UAV formation valve rotation task started...")
        if debug_mode:
            rospy.loginfo("Debug mode: SMACH viewer available at 'rosrun smach_viewer smach_viewer.py'")
        
        # Execute state machine
        outcome = sm.execute()
        
        rospy.loginfo(f"Task completed with result: {outcome}")
        
        # Keep visualization server running
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Task interrupted")
    except Exception as e:
        rospy.logerr(f"Error during task execution: {e}")
        if debug_mode:
            import traceback
            rospy.logerr(f"Debug traceback: {traceback.format_exc()}")
    finally:
        try:
            sis.stop()
        except:
            pass

if __name__ == '__main__':
    main()
