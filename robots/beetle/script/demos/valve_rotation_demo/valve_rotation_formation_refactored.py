#!/usr/bin/env python3
"""
Formation UAV Valve Rotation using SMACH State Machine
REFACTORED VERSION - Reuses classes and methods from single UAV implementation
Key insight: Transform from assembly CoG to end-effector UAV CoG, then use single UAV logic
"""

import sys
import os
import time
import math
import threading
import rospy
import tf

import smach
import smach_ros

# Add path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.dirname(__file__))  # Add current directory

from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Import single UAV modules for reuse
from trajectory import create_constant_distance_trajectory
from unified_motion_controller import UnifiedMotionController
from insertion_optimizer import InsertionOptimizer

# Import assembly demo for formation control
try:
    import sys
    sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
    from task.assembly_motion import AssemblyDemo
    ASSEMBLY_AVAILABLE = True
except ImportError as e:
    rospy.logwarn(f"Assembly demo not available: {e}")
    ASSEMBLY_AVAILABLE = False

# CRITICAL: Import single UAV classes and reuse them
# Always add the source directory first since ROS launch copies scripts to devel space
source_dir = '/home/ma/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle/script/demos/valve_rotation_demo'

# Remove any devel paths that might interfere with imports
paths_to_remove = []
for path in sys.path:
    if 'devel' in path and 'beetle' in path:
        paths_to_remove.append(path)

for path in paths_to_remove:
    sys.path.remove(path)
    rospy.loginfo(f"Removed devel path: {path}")

# Insert source directory at the beginning
if source_dir not in sys.path:
    sys.path.insert(0, source_dir)
    rospy.loginfo(f"Added source directory: {source_dir}")

try:
    from valve_rotation_fang_single import (
        SingleUAVStateBase,
        MoveToValveState, 
        DescendAndContactState,
        RotateValveState,
        InitializeStartPositionState
    )
    # NEW: Import base_UAV_state classes for maximum reuse
    from base_UAV_state import AssemblyMotionStateBase, SeperatedMotionStateBase
    
    from unified_motion_controller import UnifiedMotionController, FormationMotionCompatibility
    from trajectory import create_constant_distance_trajectory, PolynomialTrajectory
    from insertion_optimizer import InsertionOptimizer
    rospy.loginfo(f"✓ Successfully imported single UAV classes from: {source_dir}")
    rospy.loginfo(f"✓ Successfully imported base_UAV_state classes for maximum reuse")

except ImportError as e:
    rospy.logerr(f"Failed to import single UAV classes: {e}")
    rospy.logerr(f"Source directory: {source_dir}")
    rospy.logerr("Available paths:")
    for p in sys.path:
        rospy.logerr(f"  - {p}")
    sys.exit(1)


class FormationToSingleUAVAdapter:
    """
    CORRECTED Formation coordinate transformation adapter 
    
    Complete coordinate transform chain:
    1. Assembly CoG (center of two UAVs) 
    2. → Leader UAV center (rightmost UAV in formation)
    3. → End Effector (tool on leader UAV)
    """
    
    def __init__(self):
        # === Formation Configuration ===
        # End effector is attached to the rightmost UAV (UAV1 = leader)
        self.end_effector_id = 1  # Leader UAV module ID
        
        # === Step 1: Assembly CoG → Leader UAV center ===
        # CORRECTED FORMATION GEOMETRY based on urdf and assembly_api.py:
        # - Two UAVs combine in X direction with airframe_size = 0.52m spacing
        # - Assembly CoG is the CENTER between Leader and Follower
        # - Leader UAV is at +X relative to Assembly CoG
        # - Follower UAV is at -X relative to Assembly CoG
        self.airframe_size = 0.52  # From assembly_api.py
        self.assembly_to_leader_offset_x = self.airframe_size / 2.0  # +0.26m (Leader is rightmost)
        self.assembly_to_leader_offset_y = 0.0   # No Y offset - Assembly CoG and Leader UAV Y-aligned  
        self.assembly_to_leader_offset_z = 0.0   # Same Z level
        
        # === Step 2: Leader UAV center → End Effector ===  
        # Use EXACT same offsets as from urdf/beetle_fang.urdf.xacro
        # <joint name="beetle_fang_joint" ... origin xyz="0.246 0 0.0743823" .../>
        self.leader_to_end_effector_offset_x = 0.246      # From urdf: Leader UAV → End-effector X
        self.leader_to_end_effector_offset_y = 0.0        # From urdf: Centered on leader UAV Y
        self.leader_to_end_effector_offset_z_trajectory = 0.0743823  # From urdf: Z offset
        self.leader_to_end_effector_offset_z_insertion = 0.0743823   # Same for insertion
        
        # Z offsets for different phases  
        self.leader_to_end_effector_offset_z_trajectory = 0.0743823  # From urdf: Z offset for trajectory
        self.leader_to_end_effector_offset_z_insertion = 0.0743823   # From urdf: Same Z offset for insertion
        
        # === Combined offsets: Assembly CoG → End Effector (TOTAL) ===
        # This is what gets used when we need to go directly from assembly to end-effector
        self.total_offset_x = (self.assembly_to_leader_offset_x + 
                              self.leader_to_end_effector_offset_x)  # = 0.26 + 0.246 = 0.506
        self.total_offset_y = (self.assembly_to_leader_offset_y + 
                              self.leader_to_end_effector_offset_y)  # = 0.0 + 0.0 = 0.0 (Y-ALIGNED!)
        self.total_offset_z_trajectory = (self.assembly_to_leader_offset_z + 
                                         self.leader_to_end_effector_offset_z_trajectory)  # = 0.0 + 0.0743823
        self.total_offset_z_insertion = (self.assembly_to_leader_offset_z + 
                                        self.leader_to_end_effector_offset_z_insertion)    # = 0.0 + 0.0743823
        
        # COORDINATE TRANSFORM VERIFICATION: Test that transforms are true inverses
        self.verify_coordinate_transforms()
        
        rospy.loginfo("=== CORRECTED FORMATION COORDINATE SYSTEM (BASED ON URDF) ===")
        rospy.loginfo("Formation: Two UAVs combine in X direction, Assembly CoG is their center")
        rospy.loginfo(f"Assembly → Leader UAV: X={self.assembly_to_leader_offset_x:.3f}, "
                     f"Y={self.assembly_to_leader_offset_y:.3f}, Z={self.assembly_to_leader_offset_z:.3f}")
        rospy.loginfo(f"Leader UAV → End Effector: X={self.leader_to_end_effector_offset_x:.6f}, "
                     f"Y={self.leader_to_end_effector_offset_y:.3f}, Z_traj={self.leader_to_end_effector_offset_z_trajectory:.6f}")
        rospy.loginfo(f"TOTAL Assembly → End Effector: X={self.total_offset_x:.6f}, "
                     f"Y={self.total_offset_y:.3f} (SHOULD BE 0.0!), Z_traj={self.total_offset_z_trajectory:.6f}")
        rospy.loginfo("Y-axis alignment: Assembly CoG and End-effector center are Y-aligned!")
        rospy.loginfo("=" * 60)
        
        # === ROS Communication Setup ===
        # Publisher for assembly navigation commands (CORRECTED TOPIC)
        self.assembly_pub = rospy.Publisher('/assembly/uav/nav', FlightNav, queue_size=10)
        
        # Position tracking variables
        self.assembly_pos = None
        self.assembly_yaw = 0.0
        self.module1_pos = None
        self.module2_pos = None
        self.module1_pose = None  # Full pose for module1
        self.module2_pose = None  # Full pose for module2
        
        # Synchronization
        self.position_received = threading.Event()
        
        # Subscribers for position feedback (CORRECTED TOPICS)
        # Note: Assembly odometry might not exist, so we rely mainly on individual modules
        rospy.Subscriber('/beetle1/mocap/pose', PoseStamped, self.module1_callback)
        rospy.Subscriber('/beetle2/mocap/pose', PoseStamped, self.module2_callback)
        
        rospy.loginfo("=== FORMATION ROS TOPICS CONFIGURED ===")
        rospy.loginfo("Assembly Publisher: /assembly/uav/nav")
        rospy.loginfo("Module1 Subscriber: /beetle1/mocap/pose")  
        rospy.loginfo("Module2 Subscriber: /beetle2/mocap/pose")
        rospy.loginfo("=" * 50)
    
    def module1_callback(self, msg):
        """Handle position from module 1 (PoseStamped format)"""
        self.module1_pos = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        self.module1_pose = msg.pose  # Store full pose for orientation access
        rospy.logdebug(f"Module1 position updated: {self.module1_pos}")
        # Always update assembly position from individual modules when both are available
        if self.module1_pos and self.module2_pos:
            self.calculate_assembly_position_from_modules()
        
    def module2_callback(self, msg):
        """Handle position from module 2 (PoseStamped format)"""
        self.module2_pos = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        self.module2_pose = msg.pose  # Store full pose for orientation access
        # Extract yaw from module 2 (end-effector module)
        orientation_q = msg.pose.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, self.assembly_yaw = euler_from_quaternion(quaternion)
        
        rospy.logdebug(f"Module2 position updated: {self.module2_pos}, yaw: {self.assembly_yaw:.3f}")
        # Always update assembly position from individual modules when both are available
        if self.module1_pos and self.module2_pos:
            self.calculate_assembly_position_from_modules()
    
    def calculate_assembly_position_from_modules(self):
        """Calculate assembly CoG position from individual module positions (backup method)"""
        if self.module1_pos and self.module2_pos:
            # Calculate average position (assembly center of gravity)
            self.assembly_pos = (
                (self.module1_pos[0] + self.module2_pos[0]) / 2.0,
                (self.module1_pos[1] + self.module2_pos[1]) / 2.0,
                (self.module1_pos[2] + self.module2_pos[2]) / 2.0
            )
            self.position_received.set()
            rospy.logdebug(f"Assembly position updated: {self.assembly_pos}")
            rospy.logdebug(f"Module1: {self.module1_pos}, Module2: {self.module2_pos}")
    
    def get_assembly_position(self):
        """Get current assembly CoG position"""
        # Try assembly odometry first
        if self.assembly_pos is not None:
            return self.assembly_pos
        
        # Fallback: calculate CoG from individual modules
        if self.module1_pos is not None and self.module2_pos is not None:
            cog = (
                (self.module1_pos[0] + self.module2_pos[0]) / 2.0,
                (self.module1_pos[1] + self.module2_pos[1]) / 2.0,
                (self.module1_pos[2] + self.module2_pos[2]) / 2.0
            )
            return cog
        
        return None
    
    def get_assembly_yaw(self):
        """Get current assembly yaw"""
        return self.assembly_yaw
    
    def get_module2_yaw(self):
        """Get Module 2 yaw specifically (for end-effector control)"""
        try:
            if self.module2_pose is None:
                rospy.logwarn("No Module 2 pose data available")
                return None
            
            # Extract yaw from quaternion
            yaw2 = tf.transformations.euler_from_quaternion([
                self.module2_pose.orientation.x,
                self.module2_pose.orientation.y,
                self.module2_pose.orientation.z,
                self.module2_pose.orientation.w
            ])[2]
            
            return yaw2
            
        except Exception as e:
            rospy.logerr(f"Error getting Module 2 yaw: {e}")
            return None
    
    def assembly_to_end_effector_transform(self, assembly_pos, assembly_yaw, use_insertion_offset=False):
        if assembly_pos is None:
            return None
        cos_yaw = math.cos(assembly_yaw)
        sin_yaw = math.sin(assembly_yaw)
        
        # Step 1: Assembly CoG → Module2 (End-effector UAV) position
        # Module2 is offset from assembly center by half the inter-module distance
        module2_x = (assembly_pos[0] + 
                    self.assembly_to_leader_offset_x * cos_yaw - 
                    self.assembly_to_leader_offset_y * sin_yaw)
        module2_y = (assembly_pos[1] + 
                    self.assembly_to_leader_offset_x * sin_yaw + 
                    self.assembly_to_leader_offset_y * cos_yaw)
        module2_z = assembly_pos[2] + self.assembly_to_leader_offset_z
        
        # Step 2: Module2 UAV center → End Effector (dual-fang center)
        # This is the SAME transformation as single UAV mode
        end_effector_x = (module2_x + 
                         self.leader_to_end_effector_offset_x * cos_yaw - 
                         self.leader_to_end_effector_offset_y * sin_yaw)
        end_effector_y = (module2_y + 
                         self.leader_to_end_effector_offset_x * sin_yaw + 
                         self.leader_to_end_effector_offset_y * cos_yaw)
        end_effector_z = module2_z + (self.leader_to_end_effector_offset_z_insertion 
                                     if use_insertion_offset 
                                     else self.leader_to_end_effector_offset_z_trajectory)
        
        return (end_effector_x, end_effector_y, end_effector_z)
    
    def end_effector_to_assembly_transform(self, end_effector_pos, assembly_yaw, use_insertion_offset=False):
        """
        CORRECTED: Transform end-effector position to assembly CoG position
        This is the EXACT INVERSE of assembly_to_end_effector_transform
        """
        if end_effector_pos is None:
            return None
        
        cos_yaw = math.cos(assembly_yaw)
        sin_yaw = math.sin(assembly_yaw)
        
        # Step 1: End Effector → Module2 UAV center (EXACT reverse of Module2 → End Effector)
        z_offset = (self.leader_to_end_effector_offset_z_insertion 
                   if use_insertion_offset 
                   else self.leader_to_end_effector_offset_z_trajectory)
        
        # CORRECT INVERSE: Reverse the rotation transformation
        module2_x = (end_effector_pos[0] - 
                    self.leader_to_end_effector_offset_x * cos_yaw + 
                    self.leader_to_end_effector_offset_y * sin_yaw)
        module2_y = (end_effector_pos[1] - 
                    self.leader_to_end_effector_offset_x * sin_yaw - 
                    self.leader_to_end_effector_offset_y * cos_yaw)
        module2_z = end_effector_pos[2] - z_offset
        
        # Step 2: Module2 → Assembly CoG (EXACT reverse of Assembly → Module2)
        # CORRECT INVERSE: Reverse the rotation transformation
        assembly_x = (module2_x - 
                     self.assembly_to_leader_offset_x * cos_yaw + 
                     self.assembly_to_leader_offset_y * sin_yaw)
        assembly_y = (module2_y - 
                     self.assembly_to_leader_offset_x * sin_yaw - 
                     self.assembly_to_leader_offset_y * cos_yaw)
        assembly_z = module2_z - self.assembly_to_leader_offset_z
        
        return (assembly_x, assembly_y, assembly_z)
        
        return (assembly_x, assembly_y, assembly_z)
    
    def verify_coordinate_transforms(self):
        """
        Verify that assembly_to_end_effector_transform and end_effector_to_assembly_transform 
        are exact inverses of each other
        """
        rospy.loginfo("=== COORDINATE TRANSFORM VERIFICATION ===")
        
        # Test with several assembly positions and yaw angles
        test_cases = [
            ((1.0, 0.0, 0.5), 0.0),      # Assembly at origin, 0 yaw
            ((2.0, 1.0, 0.8), math.pi/4), # Assembly offset, 45° yaw
            ((0.5, -0.5, 1.2), -math.pi/3), # Assembly offset, -60° yaw
            ((3.0, 0.0, 0.6), math.pi),    # Assembly forward, 180° yaw
        ]
        
        for i, (test_assembly_pos, test_yaw) in enumerate(test_cases):
            # Forward transform: Assembly → End-effector
            end_effector_pos = self.assembly_to_end_effector_transform(
                test_assembly_pos, test_yaw, use_insertion_offset=True
            )
            
            # Inverse transform: End-effector → Assembly
            recovered_assembly_pos = self.end_effector_to_assembly_transform(
                end_effector_pos, test_yaw, use_insertion_offset=True
            )
            
            # Calculate error
            error_x = abs(recovered_assembly_pos[0] - test_assembly_pos[0])
            error_y = abs(recovered_assembly_pos[1] - test_assembly_pos[1])
            error_z = abs(recovered_assembly_pos[2] - test_assembly_pos[2])
            total_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
            
            rospy.loginfo(f"Test {i+1}: Assembly {test_assembly_pos} @ {math.degrees(test_yaw):.1f}°")
            rospy.loginfo(f"  → End-effector: {end_effector_pos}")
            rospy.loginfo(f"  → Recovered assembly: {recovered_assembly_pos}")
            rospy.loginfo(f"  → Transform error: {total_error:.8f}m (X:{error_x:.8f}, Y:{error_y:.8f}, Z:{error_z:.8f})")
            
            if total_error > 1e-6:  # 1 micrometer tolerance
                rospy.logwarn(f"Transform error too large: {total_error:.8f}m > 1e-6m")
            else:
                rospy.loginfo(f"  ✓ Transform accuracy verified")
        
        rospy.loginfo("=== COORDINATE TRANSFORM VERIFICATION COMPLETE ===")

    def send_assembly_command(self, target_pos, target_yaw=None):
        try:
            # Create FlightNav message for formation control
            nav_msg = FlightNav()
            nav_msg.header.stamp = rospy.Time.now()
            nav_msg.header.frame_id = "world"
            
            # CRITICAL: Set target to COG for assembled state
            nav_msg.target = FlightNav.COG  # This was missing! Required for assembled state
            
            # Position control
            nav_msg.target_pos_x = target_pos[0]
            nav_msg.target_pos_y = target_pos[1] 
            nav_msg.target_pos_z = target_pos[2]
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
            
            # Yaw control
            if target_yaw is not None:
                nav_msg.target_yaw = target_yaw
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE
            
            self.assembly_pub.publish(nav_msg)
            yaw_str = f"{target_yaw:.3f}" if target_yaw is not None else "None"
            rospy.logdebug(f"Sent assembly FlightNav: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}), yaw: {yaw_str}")
            
        except Exception as e:
            rospy.logerr(f"Error sending assembly command: {e}")
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def execute_assembly_trajectory(self, start_assembly_pos, target_assembly_pos, target_yaw, duration=5.0,
                                   pos_threshold=0.1, yaw_threshold=0.15):
        """
        Execute trajectory directly in assembly coordinates with IMPROVED yaw handling
        This method is called by FormationMotionController.execute_assembly_trajectory
        """
        # CRITICAL FIX: Always use real-time current position as true start point
        actual_start_pos = self.get_assembly_position()
        if actual_start_pos is None:
            rospy.logwarn("Cannot get current assembly position, using provided start position")
            actual_start_pos = start_assembly_pos
        else:
            rospy.loginfo(f"Using real-time start position: {actual_start_pos} (provided: {start_assembly_pos})")
        
        # Execute trajectory in assembly coordinates
        from trajectory import PolynomialTrajectory
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(actual_start_pos, target_assembly_pos)
        
        start_assembly_yaw = self.get_assembly_yaw()
        if start_assembly_yaw is None:
            start_assembly_yaw = 0.0
            
        yaw_diff = target_yaw - start_assembly_yaw
        
        # CRITICAL FIX: Normalize yaw difference properly
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        rospy.loginfo(f"Assembly trajectory: {actual_start_pos} -> {target_assembly_pos}")
        rospy.loginfo(f"Yaw change: {math.degrees(yaw_diff):.1f}°")
        
        # CRITICAL: Detect large yaw changes and handle them differently
        large_yaw_change = abs(yaw_diff) > math.pi/2  # 90 degrees
        if large_yaw_change:
            rospy.logwarn(f"Large yaw change detected: {math.degrees(yaw_diff):.1f}°")
            rospy.logwarn("Using position-first, yaw-second approach")
            
            return self.execute_position_then_yaw_trajectory(actual_start_pos, target_assembly_pos, 
                                                           start_assembly_yaw, target_yaw, 
                                                           duration, pos_threshold, yaw_threshold)
        else:
            # Standard simultaneous position and yaw trajectory
            rate = rospy.Rate(20)  # 20Hz control loop
            start_time = time.time()
            
            while not rospy.is_shutdown():
                elapsed = time.time() - start_time
                if elapsed >= duration:
                    break
                
                # Calculate smooth progress [0, 1]
                progress = elapsed / duration
                smooth_progress = 3 * progress**2 - 2 * progress**3  # Smooth S-curve
                
                # Generate current target position using linear interpolation
                # Since we don't have get_position(elapsed), use linear interpolation
                pt = [
                    actual_start_pos[0] + smooth_progress * (target_assembly_pos[0] - actual_start_pos[0]),
                    actual_start_pos[1] + smooth_progress * (target_assembly_pos[1] - actual_start_pos[1]),
                    actual_start_pos[2] + smooth_progress * (target_assembly_pos[2] - actual_start_pos[2])
                ]
                
                # Generate current target yaw
                if elapsed >= duration * 0.8:  # Final approach phase
                    current_target_yaw = target_yaw
                else:
                    current_target_yaw = start_assembly_yaw + smooth_progress * yaw_diff
                
                self.send_assembly_command(pt, current_target_yaw)
                
                # Check convergence periodically
                if elapsed > 2.0 and elapsed % 1.0 < 0.05:  # Check every second after 2s
                    current_assembly_pos = self.get_assembly_position()
                    if current_assembly_pos:
                        pos_error = math.sqrt(sum((current_assembly_pos[i] - target_assembly_pos[i])**2 for i in range(3)))
                        yaw_error = abs(self.get_assembly_yaw() - target_yaw)
                        if pos_error < pos_threshold and yaw_error < yaw_threshold:
                            rospy.loginfo(f"Early convergence at {elapsed:.1f}s: pos_error={pos_error:.4f}, yaw_error={yaw_error:.4f}")
                            break
                
                rate.sleep()
            
            # Final stabilization
            self.send_assembly_command(target_assembly_pos, target_yaw)
            time.sleep(1.0)
            return True
            time.sleep(1.0)
            return True
    
    def execute_position_then_yaw_trajectory(self, start_pos, target_pos, start_yaw, target_yaw, 
                                           duration=8.0, pos_threshold=0.1, yaw_threshold=0.15):
        """
        Execute trajectory with position first, then yaw adjustment
        Used for large yaw changes to improve stability
        """
        rospy.loginfo("Executing position-first, then yaw trajectory")
        
        # CRITICAL FIX: Always use real-time current position as true start point
        actual_start_pos = self.get_assembly_position()
        if actual_start_pos is None:
            rospy.logwarn("Cannot get current assembly position, using provided start position")
            actual_start_pos = start_pos
        else:
            rospy.loginfo(f"Using real-time start position: {actual_start_pos} (provided: {start_pos})")
        
        # Phase 1: Move to target position while maintaining current yaw
        from trajectory import PolynomialTrajectory
        pos_duration = duration * 0.7  # 70% of time for position
        yaw_duration = duration * 0.3  # 30% of time for yaw
        
        traj = PolynomialTrajectory(pos_duration)
        traj.generate_trajectory(actual_start_pos, target_pos)
        
        # Position phase
        rate = rospy.Rate(20)
        start_time = time.time()
        
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= pos_duration:
                break
            
            # Use linear interpolation instead of get_position(elapsed)
            progress = elapsed / pos_duration
            smooth_progress = 3 * progress**2 - 2 * progress**3
            pt = [
                actual_start_pos[0] + smooth_progress * (target_pos[0] - actual_start_pos[0]),
                actual_start_pos[1] + smooth_progress * (target_pos[1] - actual_start_pos[1]),
                actual_start_pos[2] + smooth_progress * (target_pos[2] - actual_start_pos[2])
            ]
            self.send_assembly_command(pt, start_yaw)  # Keep original yaw
            rate.sleep()
        
        # Brief pause between phases
        time.sleep(0.5)
        
        # Phase 2: Adjust yaw while maintaining final position
        yaw_diff = target_yaw - start_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        start_time = time.time()
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= yaw_duration:
                break
            
            progress = elapsed / yaw_duration
            smooth_progress = 3 * progress**2 - 2 * progress**3
            current_yaw = start_yaw + smooth_progress * yaw_diff
            
            self.send_assembly_command(target_pos, current_yaw)
            
            # Check yaw convergence
            if elapsed > 1.0:
                current_assembly_yaw = self.get_assembly_yaw()
                if current_assembly_yaw is not None:
                    yaw_error = abs(current_assembly_yaw - target_yaw)
                    if yaw_error < yaw_threshold:
                        rospy.loginfo(f"Yaw converged early at {elapsed:.1f}s")
                        break
            
            rate.sleep()
        
        # Final command
        self.send_assembly_command(target_pos, target_yaw)
        time.sleep(1.0)
        return True
    
    def execute_vertical_descent_trajectory(self, start_pos, target_pos, target_yaw, duration):
        """
        Execute vertical descent with strict XY control for Formation
        """
        rospy.loginfo("=== FORMATION VERTICAL DESCENT WITH LOCKED XY ===")
        
        # CRITICAL FIX: Always use real-time current position as true start point
        actual_start_pos = self.get_assembly_position()
        if actual_start_pos is None:
            rospy.logwarn("Cannot get current assembly position, using provided start position")
            actual_start_pos = start_pos
        else:
            rospy.loginfo(f"Using real-time start position: {actual_start_pos} (provided: {start_pos})")
        
        rospy.loginfo(f"Descent: {actual_start_pos} -> {target_pos}")
        
        # Verify this is indeed vertical movement
        xy_movement = math.sqrt((target_pos[0] - actual_start_pos[0])**2 + (target_pos[1] - actual_start_pos[1])**2)
        z_movement = abs(target_pos[2] - actual_start_pos[2])
        
        if xy_movement > 0.05:
            rospy.logwarn(f"Large XY movement detected in vertical descent: {xy_movement:.3f}m")
        
        # Use polynomial trajectory for smooth Z descent
        from trajectory import PolynomialTrajectory
        
        # Create trajectory for Z-only movement
        z_traj = PolynomialTrajectory(duration)
        z_start = (0, 0, actual_start_pos[2])  # Only Z matters
        z_target = (0, 0, target_pos[2])
        z_traj.generate_trajectory(z_start, z_target)
        
        rate = rospy.Rate(20)
        start_time = time.time()
        
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            
            # Use S-curve interpolation for smooth Z movement
            progress = elapsed / duration
            smooth_progress = 3 * progress**2 - 2 * progress**3
            
            # Keep XY locked at actual start position, only change Z
            current_z = actual_start_pos[2] + smooth_progress * (target_pos[2] - actual_start_pos[2])
            pt = [actual_start_pos[0], actual_start_pos[1], current_z]
            
            # Get smooth Z position using linear interpolation
            progress = elapsed / duration
            smooth_progress = 3 * progress**2 - 2 * progress**3
            current_z = start_pos[2] + smooth_progress * (target_pos[2] - start_pos[2])
            
            # Lock XY to start position, only move Z
            vertical_command = (
                start_pos[0],  # X locked
                start_pos[1],  # Y locked  
                current_z      # Z moves smoothly
            )
            
            self.send_assembly_command(vertical_command, target_yaw)
            rate.sleep()
        
        # Final position
        self.send_assembly_command(target_pos, target_yaw)
        time.sleep(1.0)
        return True


class FormationSingleUAVStateBase(SingleUAVStateBase):
    
    def __init__(self, outcomes, input_keys=None, output_keys=None, module_id=None):
        # Initialize adapter for coordinate transformation
        self.adapter = FormationToSingleUAVAdapter()
        
        # Override module_id to use end-effector module for external wrench
        actual_module_id = self.adapter.end_effector_id if module_id is None else module_id
        
        # Initialize parent class
        super().__init__(outcomes, input_keys, output_keys, actual_module_id)
        
        # Override publisher to use assembly navigation
        self.assembly_pub = self.adapter.assembly_pub
        
        # Create formation motion controller that wraps the single UAV motion controller
        self.formation_motion_controller = FormationMotionController(self.motion_controller, self.adapter)
        
        rospy.loginfo(f"Formation single UAV state initialized with module_id={actual_module_id}")
    
    def get_current_position(self):

        # Get Module2 position (the UAV with end-effector)
        if self.adapter.module2_pos is None:
            rospy.logwarn("Module2 position not available for end-effector calculation")
            return None
        
        module2_pos = self.adapter.module2_pos
        module2_yaw = self.adapter.assembly_yaw  # Module2's yaw is assembly yaw
        
        # EXACT same calculation as single UAV version
        # From valve_rotation_fang_single.py line 712:
        # end_effector_x = current_pos[0] + 0.25346 * math.cos(self.current_yaw)
        # end_effector_y = current_pos[1] + 0.25346 * math.sin(self.current_yaw)
        end_effector_x = module2_pos[0] + 0.25346 * math.cos(module2_yaw)
        end_effector_y = module2_pos[1] + 0.25346 * math.sin(module2_yaw)
        end_effector_z = module2_pos[2] + 0.0221140  # Z offset for dual-fang center
        
        rospy.logdebug(f"Formation end-effector: Module2={module2_pos}, yaw={module2_yaw:.3f}rad, end-effector=({end_effector_x:.3f}, {end_effector_y:.3f}, {end_effector_z:.3f})")
        
        return (end_effector_x, end_effector_y, end_effector_z)
    
    def get_assembly_position(self):
        """Get assembly position for motion control"""
        return self.adapter.get_assembly_position()
    
    def get_current_yaw(self):
        """Override to return assembly yaw instead of individual UAV yaw"""
        return self.adapter.get_assembly_yaw()
    
    @property
    def current_yaw(self):
        """Property that returns formation yaw instead of individual UAV yaw"""
        return self.adapter.get_assembly_yaw()
    
    @current_yaw.setter
    def current_yaw(self, value):
        """Allow setting current_yaw for compatibility, but ignore the value"""
        # The formation adapter manages yaw through assembly odometry
        pass
    
    def wait_for_positions(self):
        """Override to wait for assembly and valve positions with extended timeout and fallback"""
        rospy.loginfo("Waiting for assembly and valve positions...")
        
        # First try to get assembly position with extended timeout
        rospy.loginfo("Waiting for assembly position...")
        if not self.adapter.position_received.wait(timeout=15.0):
            rospy.logwarn("Assembly odometry not received, trying to use individual module positions...")
            
            # Try to force calculation from individual modules
            if self.adapter.module1_pos and self.adapter.module2_pos:
                self.adapter.calculate_assembly_position_from_modules()
                rospy.loginfo("✓ Using assembly position calculated from individual modules")
            elif self.adapter.module1_pos or self.adapter.module2_pos:
                # Wait a bit more for the missing module
                rospy.loginfo("Waiting for missing module position...")
                rospy.sleep(3.0)
                if self.adapter.module1_pos and self.adapter.module2_pos:
                    self.adapter.calculate_assembly_position_from_modules()
                    rospy.loginfo("✓ Assembly position calculated after waiting")
                else:
                    rospy.logerr("Assembly position calculation failed - missing module positions")
                    rospy.logerr(f"Module1: {self.adapter.module1_pos}, Module2: {self.adapter.module2_pos}")
                    return False
            else:
                rospy.logerr("No assembly or module positions available")
                return False
        else:
            rospy.loginfo("✓ Assembly position received from odometry")
        
        # Wait for valve position
        rospy.loginfo("Waiting for valve position...")
        if not self.valve_received.wait(timeout=5.0):
            rospy.logerr("Valve position not received")
            return False
        
        rospy.loginfo("✓ All positions received successfully")
        return True

    def execute_assembly_trajectory(self, start_assembly_pos, target_assembly_pos, target_yaw, duration=5.0,
                                   pos_threshold=0.1, yaw_threshold=0.15):
        """
        Execute trajectory directly in assembly coordinates with IMPROVED yaw handling
        """
        # CRITICAL FIX: Always use real-time current position as true start point
        actual_start_pos = self.adapter.get_assembly_position()
        if actual_start_pos is None:
            rospy.logwarn("Cannot get current assembly position, using provided start position")
            actual_start_pos = start_assembly_pos
        else:
            rospy.loginfo(f"Using real-time start position: {actual_start_pos} (provided: {start_assembly_pos})")
        
        # Execute trajectory in assembly coordinates
        from trajectory import PolynomialTrajectory
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(actual_start_pos, target_assembly_pos)
        
        start_assembly_yaw = self.adapter.get_assembly_yaw()
        yaw_diff = target_yaw - start_assembly_yaw
        
        # CRITICAL FIX: Normalize yaw difference properly
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        rospy.loginfo(f"Assembly trajectory: {actual_start_pos} -> {target_assembly_pos}")
        rospy.loginfo(f"Yaw change: {math.degrees(yaw_diff):.1f}°")
        
        # CRITICAL: Detect large yaw changes and handle them differently
        large_yaw_change = abs(yaw_diff) > math.pi/2  # 90 degrees
        if large_yaw_change:
            rospy.logwarn(f"Large yaw change detected: {math.degrees(yaw_diff):.1f}°")
            rospy.logwarn("Using position-first, yaw-second approach")
            
            # For large yaw changes, prioritize position convergence first
            return self.execute_position_then_yaw_trajectory(
                actual_start_pos, target_assembly_pos, start_assembly_yaw, 
                target_yaw, duration, pos_threshold, yaw_threshold
            )
        
        # Normal trajectory execution for small yaw changes
        rate = rospy.Rate(50)
        start_time = time.time()
        trajectory_completed = False
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 10.0:
            pt = traj.evaluate()
            elapsed_time = time.time() - start_time
            
            # Check if trajectory is complete
            if pt is None or elapsed_time >= duration:
                pt = target_assembly_pos
                trajectory_completed = True
            
            # Smooth yaw interpolation
            if elapsed_time <= duration:
                yaw_progress = min(elapsed_time / duration, 1.0)
                smooth_progress = 3*yaw_progress**2 - 2*yaw_progress**3
                current_target_yaw = start_assembly_yaw + smooth_progress * yaw_diff
            else:
                current_target_yaw = target_yaw
            
            self.adapter.send_assembly_command(pt, current_target_yaw)
            
            # Check for convergence
            if trajectory_completed:
                current_pos = self.adapter.get_assembly_position()
                if current_pos is not None:
                    pos_error = math.sqrt(sum((c - t) ** 2 for c, t in zip(current_pos, target_assembly_pos)))
                    yaw_error = abs(self.adapter.get_assembly_yaw() - target_yaw)
                    if yaw_error > math.pi:
                        yaw_error = 2 * math.pi - yaw_error
                    
                    # Debug: Show current position and errors
                    rospy.loginfo_throttle(2.0, f"Position check: current={current_pos}, target={target_assembly_pos}")
                    rospy.loginfo_throttle(2.0, f"Errors: pos={pos_error:.3f}m (limit={pos_threshold}), yaw={math.degrees(yaw_error):.1f}° (limit={math.degrees(yaw_threshold):.1f}°)")
                    
                    if pos_error < pos_threshold and yaw_error < yaw_threshold:
                        rospy.loginfo(f"✓ Assembly trajectory completed successfully")
                        rospy.loginfo(f"Final position error: {pos_error:.3f}m, yaw error: {math.degrees(yaw_error):.1f}°")
                        return True
            
            rate.sleep()
        
        rospy.logerr("Assembly trajectory timeout")
        return False
    
    def execute_position_then_yaw_trajectory(self, start_pos, target_pos, start_yaw, target_yaw, 
                                           duration, pos_threshold, yaw_threshold):
        """
        Execute trajectory with position first, then yaw (for large yaw changes)
        """
        rospy.loginfo("=== POSITION-FIRST, YAW-SECOND TRAJECTORY ===")
        
        # Phase 1: Move to position while keeping current yaw
        rospy.loginfo("Phase 1: Moving to position (keeping current yaw)")
        
        from trajectory import PolynomialTrajectory
        pos_duration = duration * 0.7  # 70% of time for position
        
        pos_traj = PolynomialTrajectory(pos_duration)
        pos_traj.generate_trajectory(start_pos, target_pos)
        
        rate = rospy.Rate(50)
        start_time = time.time()
        
        while not rospy.is_shutdown() and (time.time() - start_time) < pos_duration + 5.0:
            pt = pos_traj.evaluate()
            elapsed_time = time.time() - start_time
            
            if pt is None or elapsed_time >= pos_duration:
                pt = target_pos
                break
            
            self.adapter.send_assembly_command(pt, start_yaw)
            rate.sleep()
        
        rospy.loginfo("Phase 1 completed: Position reached")
        time.sleep(1.0)  # Brief stabilization
        
        # Phase 2: Adjust yaw at fixed position
        rospy.loginfo("Phase 2: Adjusting yaw at fixed position")
        
        yaw_duration = duration * 0.3  # 30% of time for yaw
        yaw_start_time = time.time()
        
        yaw_diff = target_yaw - start_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        while not rospy.is_shutdown() and (time.time() - yaw_start_time) < yaw_duration + 5.0:
            elapsed_time = time.time() - yaw_start_time
            
            if elapsed_time >= yaw_duration:
                current_yaw = target_yaw
            else:
                yaw_progress = min(elapsed_time / yaw_duration, 1.0)
                smooth_progress = 3*yaw_progress**2 - 2*yaw_progress**3
                current_yaw = start_yaw + smooth_progress * yaw_diff
            
            self.adapter.send_assembly_command(target_pos, current_yaw)
            
            # Check convergence
            if elapsed_time >= yaw_duration:
                current_pos = self.adapter.get_assembly_position()
                if current_pos is not None:
                    pos_error = math.sqrt(sum((c - t) ** 2 for c, t in zip(current_pos, target_pos)))
                    yaw_error = abs(self.adapter.get_assembly_yaw() - target_yaw)
                    if yaw_error > math.pi:
                        yaw_error = 2 * math.pi - yaw_error
                    
                    if pos_error < pos_threshold and yaw_error < yaw_threshold:
                        rospy.loginfo(f"✓ Position-then-yaw trajectory completed successfully")
                        return True
            
            rate.sleep()
        
        rospy.loginfo("✓ Position-then-yaw trajectory completed")
        return True

    def execute_vertical_descent_trajectory(self, start_pos, target_pos, target_yaw, duration):
        """
        Execute PURE vertical descent with STRICT XY position control (mimicking single UAV)
        This ensures zero XY drift during descent, exactly like single UAV implementation
        """
        rospy.loginfo("=== EXECUTING VERTICAL DESCENT WITH STRICT XY CONTROL ===")
        rospy.loginfo(f"Start: {start_pos}, Target: {target_pos}")
        rospy.loginfo(f"XY position LOCKED at: ({start_pos[0]:.6f}, {start_pos[1]:.6f})")
        
        # Verify this is indeed a vertical-only movement
        xy_movement = math.sqrt((target_pos[0] - start_pos[0])**2 + (target_pos[1] - start_pos[1])**2)
        if xy_movement > 0.01:  # 1cm tolerance
            rospy.logwarn(f"Warning: Not pure vertical descent, XY movement: {xy_movement:.3f}m")
        
        # Create trajectory that LOCKS XY coordinates
        rate = rospy.Rate(50)  # High frequency for precise control
        start_time = time.time()
        
        # Lock XY coordinates from start position (no drift allowed)
        locked_x = start_pos[0]
        locked_y = start_pos[1]
        start_z = start_pos[2]
        target_z = target_pos[2]
        
        rospy.loginfo(f"Vertical descent: Z from {start_z:.6f} to {target_z:.6f}")
        rospy.loginfo(f"XY coordinates LOCKED at: X={locked_x:.6f}, Y={locked_y:.6f}")
        
        trajectory_completed = False
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 5.0:
            elapsed_time = time.time() - start_time
            
            # Calculate Z position using smooth interpolation
            if elapsed_time >= duration:
                current_z = target_z
                trajectory_completed = True
            else:
                z_progress = min(elapsed_time / duration, 1.0)
                # Use smooth cubic interpolation for gentle descent
                smooth_progress = 3*z_progress**2 - 2*z_progress**3
                current_z = start_z + smooth_progress * (target_z - start_z)
            
            # Create command with LOCKED XY, only changing Z
            vertical_command = [locked_x, locked_y, current_z]
            
            # Send command with strict XY control
            self.adapter.send_assembly_command(vertical_command, target_yaw)
            
            # Check for completion and convergence
            if trajectory_completed:
                # Verify final position convergence
                current_pos = self.adapter.get_assembly_position()
                if current_pos is not None:
                    pos_error = math.sqrt(sum((c - t) ** 2 for c, t in zip(current_pos, target_pos)))
                    xy_error = math.sqrt((current_pos[0] - locked_x)**2 + (current_pos[1] - locked_y)**2)
                    z_error = abs(current_pos[2] - target_z)
                    
                    rospy.loginfo(f"Vertical descent convergence check:")
                    rospy.loginfo(f"  XY error: {xy_error:.4f}m (should be <0.01m)")
                    rospy.loginfo(f"  Z error: {z_error:.4f}m")
                    rospy.loginfo(f"  Total position error: {pos_error:.4f}m")
                    
                    if pos_error < 0.05:  # Position threshold
                        rospy.loginfo("✓ Vertical descent completed with high precision")
                        return True
            
            rate.sleep()
        
        rospy.loginfo("✓ Vertical descent trajectory completed")
        return True

    def execute_pure_yaw_adjustment(self, locked_position, target_yaw, duration):
        """
        Execute PURE yaw adjustment with COMPLETELY LOCKED XY position
        This is critical for ensuring zero XY drift during rotation
        """
        rospy.loginfo("=== EXECUTING PURE YAW ADJUSTMENT WITH LOCKED XY ===")
        rospy.loginfo(f"Position LOCKED at: {locked_position}")
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
        
        # High frequency control for precise yaw adjustment
        rate = rospy.Rate(50)  # 50Hz for precise control
        start_time = time.time()
        
        # Lock ALL coordinates - only yaw can change
        locked_x = locked_position[0]
        locked_y = locked_position[1]
        locked_z = locked_position[2]
        
        trajectory_completed = False
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 3.0:
            elapsed_time = time.time() - start_time
            
            # Create command with EXACTLY the same position, only yaw changes
            locked_command = [locked_x, locked_y, locked_z]
            
            # Send command with locked position and target yaw
            self.adapter.send_assembly_command(locked_command, target_yaw)
            
            # Check for completion
            if elapsed_time >= duration:
                trajectory_completed = True
                # Verify yaw convergence
                current_yaw = self.adapter.get_assembly_yaw()
                if current_yaw is not None:
                    yaw_error = abs(self.normalize_angle(current_yaw - target_yaw))
                    rospy.loginfo(f"Pure yaw adjustment completed: yaw error = {math.degrees(yaw_error):.1f}°")
                break
            
            rate.sleep()
        
        rospy.loginfo("✓ Pure yaw adjustment completed with locked XY position")
        return True


class FormationMotionController:
    """
    Formation motion controller that wraps single UAV motion controller
    Transforms coordinates between formation and single UAV coordinate systems
    """
    
    def __init__(self, single_uav_motion_controller, adapter):
        self.single_motion_controller = single_uav_motion_controller
        self.adapter = adapter
    
    def send_trajectory_point(self, pos, yaw=None):
        """
        Send trajectory point for formation
        Transforms from single UAV logic (end-effector position) to formation command (assembly position)
        """
        # Transform end-effector position to assembly position
        assembly_yaw = yaw if yaw is not None else self.adapter.get_assembly_yaw()
        assembly_pos = self.adapter.end_effector_to_assembly_transform(pos, assembly_yaw)
        
        if assembly_pos is not None:
            self.adapter.send_assembly_command(assembly_pos, yaw)
        else:
            rospy.logerr("Failed to transform end-effector position to assembly position")
    
    def execute_smooth_trajectory_with_yaw(self, start_pos, target_pos, target_yaw, duration=5.0,
                                          pos_threshold=0.03, yaw_threshold=0.08):
        """
        Execute smooth trajectory for formation with STRICT XY control (mimicking single UAV)
        Transforms trajectory from end-effector coordinates to assembly coordinates
        """
        rospy.loginfo("=== FORMATION SMOOTH TRAJECTORY WITH STRICT XY CONTROL ===")
        
        # Transform start and target positions from end-effector to assembly coordinates
        start_assembly_yaw = self.adapter.get_assembly_yaw()
        target_assembly_yaw = target_yaw
        
        start_assembly_pos = self.adapter.end_effector_to_assembly_transform(start_pos, start_assembly_yaw)
        target_assembly_pos = self.adapter.end_effector_to_assembly_transform(target_pos, target_assembly_yaw)
        
        if start_assembly_pos is None or target_assembly_pos is None:
            rospy.logerr("Failed to transform trajectory positions")
            return False
        
        # Determine if this is primarily vertical movement (like single UAV descent)
        xy_movement = math.sqrt(
            (target_assembly_pos[0] - start_assembly_pos[0])**2 + 
            (target_assembly_pos[1] - start_assembly_pos[1])**2
        )
        z_movement = abs(target_assembly_pos[2] - start_assembly_pos[2])
        
        # If primarily vertical movement, use strict XY control
        if xy_movement < 0.05 and z_movement > 0.1:  # Vertical descent detected
            rospy.loginfo(f"Vertical movement detected: XY={xy_movement:.3f}m, Z={z_movement:.3f}m")
            rospy.loginfo("Using STRICT XY control for vertical descent")
            
            # Create pure vertical descent with locked XY coordinates
            locked_xy_target = [
                start_assembly_pos[0],  # Lock X coordinate
                start_assembly_pos[1],  # Lock Y coordinate  
                target_assembly_pos[2]  # Only change Z
            ]
            
            return self.execute_vertical_descent_trajectory(
                start_assembly_pos, locked_xy_target, target_assembly_yaw, duration
            )
        else:
            # Normal trajectory for horizontal or mixed movement
            rospy.loginfo(f"Normal trajectory: XY={xy_movement:.3f}m, Z={z_movement:.3f}m")
            return self.execute_assembly_trajectory(start_assembly_pos, target_assembly_pos, target_assembly_yaw, 
                                                  duration, pos_threshold, yaw_threshold)
    
    def execute_assembly_trajectory(self, start_assembly_pos, target_assembly_pos, target_yaw, duration=5.0,
                                   pos_threshold=0.1, yaw_threshold=0.15):
        """
        Execute trajectory directly in assembly coordinates with IMPROVED yaw handling
        """
        # Delegate to the main trajectory execution method
        return self.adapter.execute_assembly_trajectory(start_assembly_pos, target_assembly_pos, target_yaw, 
                                                       duration, pos_threshold, yaw_threshold)
    
    def execute_vertical_descent_trajectory(self, start_pos, target_pos, target_yaw, duration):
        """
        Execute vertical descent with strict XY control (delegate to adapter)
        """
        return self.adapter.execute_vertical_descent_trajectory(start_pos, target_pos, target_yaw, duration)
    
    def send_velocity_command(self, vel, yaw_vel, pos_backup=None, yaw_backup=None):
        """
        Send velocity command for formation
        Transforms velocity commands from end-effector frame to assembly frame
        
        Args:
            vel: Velocity command (vx, vy, vz) m/s in end-effector coordinates
            yaw_vel: Yaw angular velocity rad/s for assembly
            pos_backup: Backup position (x, y, z) in end-effector coordinates
            yaw_backup: Backup yaw angle rad for assembly
        """
        # Transform backup position from end-effector to assembly coordinates if provided
        assembly_pos_backup = None
        if pos_backup is not None:
            assembly_yaw = yaw_backup if yaw_backup is not None else self.adapter.get_assembly_yaw()
            assembly_pos_backup = self.adapter.end_effector_to_assembly_transform(pos_backup, assembly_yaw)
            if assembly_pos_backup is None:
                rospy.logerr("Failed to transform backup position for velocity command")
                return
        
        # For velocity commands, we assume the transformation is rotation-based
        # Get current assembly orientation for velocity frame transformation
        current_assembly_yaw = self.adapter.get_assembly_yaw()
        if current_assembly_yaw is None:
            rospy.logerr("Cannot get assembly yaw for velocity transformation")
            return
        
        # Transform velocity vector from end-effector frame to assembly frame
        # Since end-effector is at assembly center with same orientation, velocities are the same
        # (The geometric offset doesn't affect velocity transformation, only position)
        assembly_vel = vel  # Velocity transformation is identity for our geometry
        
        # Send velocity command through adapter's command interface
        # We'll need to implement this in the adapter or use the existing command system
        if assembly_pos_backup is not None:
            self.adapter.send_assembly_command(assembly_pos_backup, yaw_backup)
        else:
            rospy.logwarn("Velocity-only command not yet implemented, using last known position")
            # Fallback: get current position and use as backup
            current_assembly_pos = self.adapter.get_assembly_position()
            if current_assembly_pos is not None:
                self.adapter.send_assembly_command(current_assembly_pos, yaw_backup)


# Formation-specific states that inherit from single UAV states
class FormationAssembleState(smach.State):
    """Formation assembly state - makes two UAVs physically connect"""
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
        # Get module configuration
        modules_str = rospy.get_param("~module_ids", "1,2")
        real_machine = rospy.get_param("~real_machine", False)
        modules = [int(x) for x in modules_str.split(',')]
        
        # Initialize assembly demo
        if ASSEMBLY_AVAILABLE:
            try:
                self.assemble_demo = AssemblyDemo(module_ids=modules, real_machine=real_machine)
                self.assembly_available = True
                rospy.loginfo(f"Assembly demo initialized for modules: {modules}")
            except Exception as e:
                rospy.logerr(f"Failed to initialize assembly demo: {e}")
                self.assembly_available = False
        else:
            self.assembly_available = False
            rospy.logwarn("Assembly demo not available, skipping assembly step")
    
    def execute(self, userdata):
        if not self.assembly_available:
            rospy.logwarn("Assembly not available, proceeding without physical assembly")
            return 'succeeded'
        
        rospy.loginfo("=== STARTING FORMATION ASSEMBLY ===")
        rospy.loginfo("Executing physical assembly of UAV modules...")
        
        try:
            # Execute assembly process
            self.assemble_demo.main()
            rospy.loginfo("✓ Formation assembly completed successfully")
            
            # Wait a moment for assembly to stabilize
            rospy.sleep(2.0)
            
            return 'succeeded'
            
        except rospy.ROSInterruptException:
            rospy.logerr("✗ Formation assembly interrupted")
            return 'failed'
        except Exception as e:
            rospy.logerr(f"✗ Formation assembly failed: {e}")
            return 'failed'


class FormationInitializeStartPositionState(FormationSingleUAVStateBase):
    """Formation version of InitializeStartPositionState - uses assembly position"""
    
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'],
                        output_keys=['start_position'])
    
    def execute(self, userdata):
        rospy.loginfo("Initializing formation start position...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        userdata.start_position = self.get_current_position()
        rospy.loginfo(f"Formation start position initialized: {userdata.start_position}")
        return 'succeeded'


class FormationMoveToValveState(FormationSingleUAVStateBase):
    """Formation version of MoveToValveState - reuses single UAV logic completely"""
    
    def __init__(self, approach_distance=0.8, approach_height=0.3):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'])
        
        self.approach_distance = approach_distance
        self.approach_height = approach_height
        
        # Override motion controller to use formation version
        self.motion_controller = self.formation_motion_controller
    
    def execute(self, userdata):
        rospy.loginfo("Formation moving to valve approach position using single UAV logic...")
        
        # Use EXACT same logic as single UAV MoveToValveState.execute()
        # The coordinate transformation is handled transparently by FormationMotionController
        
        if not self.wait_for_positions():
            return 'failed'
        
        # Calculate approach position with IMPROVED logic considering current position
        valve_x, valve_y, valve_z = self.valve_pos
        current_pos = self.get_current_position()
        
        if current_pos is None:
            rospy.logerr("Cannot get current formation position")
            return 'failed'
        
        # IMPROVED: Calculate approach direction based on current position relative to valve
        # This ensures we approach from the CURRENT side, minimizing unnecessary yaw rotations
        current_to_valve_x = valve_x - current_pos[0]
        current_to_valve_y = valve_y - current_pos[1]
        current_distance = math.sqrt(current_to_valve_x**2 + current_to_valve_y**2)
        
        if current_distance > 0.01:  # Avoid division by zero
            # Normalize direction vector and apply approach distance
            approach_direction_x = current_to_valve_x / current_distance
            approach_direction_y = current_to_valve_y / current_distance
            
            # Calculate approach position at desired distance from valve center
            approach_x = valve_x - approach_direction_x * self.approach_distance
            approach_y = valve_y - approach_direction_y * self.approach_distance
            approach_z = valve_z + self.approach_height
            
            # Calculate yaw to face valve from approach position
            target_yaw = math.atan2(valve_y - approach_y, valve_x - approach_x)
        else:
            # Fallback: Use original logic if already at valve position
            rospy.logwarn("Formation already at valve position, using default approach")
            approach_x = valve_x - self.approach_distance
            approach_y = valve_y
            approach_z = valve_z + self.approach_height
            target_yaw = math.atan2(valve_y - approach_y, valve_x - approach_x)
        
        target_end_effector_pos = (approach_x, approach_y, approach_z)  # End-effector target
        
        # Convert end-effector target to assembly target
        target_assembly_pos = self.adapter.end_effector_to_assembly_transform(
            target_end_effector_pos, target_yaw, use_insertion_offset=False
        )
        
        rospy.loginfo(f"Valve position: ({valve_x:.3f}, {valve_y:.3f}, {valve_z:.3f})")
        rospy.loginfo(f"Current assembly position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"Approach distance: {self.approach_distance:.3f}m")
        rospy.loginfo(f"End-effector approach target: {target_end_effector_pos}")
        rospy.loginfo(f"Assembly approach target: {target_assembly_pos}")
        rospy.loginfo(f"Target yaw: {target_yaw:.3f} rad ({math.degrees(target_yaw):.1f}°)")
        
        # Two-step movement using corrected assembly targets
        current_assembly_pos = self.get_assembly_position()  # Get assembly position for motion control
        if current_assembly_pos is None:
            return 'failed'
        
        rospy.loginfo("Step 1: Moving assembly to target position")
        
        # Calculate adaptive trajectory duration using assembly positions
        distance = math.sqrt((target_assembly_pos[0] - current_assembly_pos[0])**2 + 
                           (target_assembly_pos[1] - current_assembly_pos[1])**2 + 
                           (target_assembly_pos[2] - current_assembly_pos[2])**2)
        
        # Adaptive speed (same as single UAV)
        base_speed = 0.1
        min_duration = 10.0
        max_duration = 30.0
        
        if distance < 1.0:
            effective_speed = base_speed * 0.5
        elif distance < 2.0:
            effective_speed = base_speed * 0.8
        else:
            effective_speed = base_speed
        
        trajectory_duration = max(min_duration, min(max_duration, distance / effective_speed))
        rospy.loginfo(f"Distance: {distance:.3f}m, speed: {effective_speed:.2f}m/s, duration: {trajectory_duration:.1f}s")
        
        # Step 1: Position movement using DIRECT control like valve_rotation_smach_test.py
        rospy.loginfo("Step 1: Moving to target position using DIRECT control")
        
        # Get current assembly yaw
        current_assembly_yaw = self.adapter.get_assembly_yaw()
        
        # Use internal assembly trajectory execution instead of FormationMotionCompatibility
        success = self.execute_assembly_trajectory(current_assembly_pos, target_assembly_pos, 
                                                 current_assembly_yaw, trajectory_duration)
        if not success:
            rospy.logerr("Failed to move to target position")
            return 'failed'
        
        rospy.loginfo("✓ Position movement completed")
        
        rospy.loginfo("Step 2: Adjusting yaw orientation using DIRECT control")
        
        # Step 2: Yaw adjustment using direct control
        final_assembly_pos = self.get_assembly_position()
        if final_assembly_pos is None:
            final_assembly_pos = target_assembly_pos
        
        # Use internal assembly trajectory execution for yaw adjustment
        yaw_success = self.execute_assembly_trajectory(final_assembly_pos, final_assembly_pos, 
                                                     target_yaw, 1.0)  # 1 second for yaw adjustment
        if not yaw_success:
            rospy.logwarn("Yaw adjustment failed, continuing")
        
        rospy.loginfo("✓ Yaw adjustment completed")
        
        # Verify final end-effector position
        final_assembly_pos = self.get_current_position()
        if final_assembly_pos:
            actual_end_effector_pos = self.adapter.assembly_to_end_effector_transform(
                final_assembly_pos, target_yaw, use_insertion_offset=False
            )
            rospy.loginfo(f"Final end-effector position: {actual_end_effector_pos}")
            # target_end_effector_pos is from parent scope
        
        rospy.loginfo("Formation two-step movement completed")
        return 'succeeded'


class FormationDescendAndContactState(DescendAndContactState, FormationSingleUAVStateBase):
    """Formation version of DescendAndContactState - reuses single UAV logic"""
    
    def __init__(self, rotation_direction=1):
        # Initialize both parent classes
        DescendAndContactState.__init__(self, rotation_direction=rotation_direction)
        FormationSingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed'], 
                                           input_keys=['start_position'])
        
        self.rotation_direction = rotation_direction
        
        # Override motion controller to use formation version
        self.motion_controller = self.formation_motion_controller
        
        # Initialize insertion optimizer (same as single UAV)
        self.optimizer = InsertionOptimizer(
            valve_radius=0.1075,
            valve_beam_width=0.024,
            safety_margin=0.002,
            module_id=self.module_id
        )
        
        self.pre_insertion_distance = 0.035
        self.circumferential_offset = 0.005
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def execute(self, userdata):
        """Formation descend and contact with CORRECTED Z coordinate calculation"""
        rospy.loginfo("Formation descend and contact using single UAV logic...")
        
        # Initialize and wait for required positions
        if not self.wait_for_positions():
            return 'failed'
        
        # Get current end-effector position (from Module2)
        start_pos = self.get_current_position()
        if start_pos is None:
            return 'failed'
        
        valve_z = self.valve_pos[2]
        
        # CRITICAL: Use SAME Z offset calculation as single UAV but adjust for formation
        # From valve_rotation_fang_single.py: end_effector_offset_z = 0.0221140
        # However, for formation, we need to be more conservative to avoid excessive descent
        
        # Get current assembly position for reference
        current_assembly_pos = self.get_assembly_position()
        current_assembly_z = current_assembly_pos[2] if current_assembly_pos else 1.0
        
        # Conservative approach: Only descend enough to bring end-effector close to valve height
        # Target: end-effector slightly above valve for safety
        safety_margin = 0.05  # 5cm above valve for safety
        target_end_effector_z = valve_z + safety_margin
        
        # Work backwards: target_end_effector_z = target_module2_z + end_effector_offset
        end_effector_z_offset = 0.0221140  # Single UAV value - actual end-effector offset (downward)
        target_module2_z = target_end_effector_z + end_effector_z_offset  # Module2 should be ABOVE end-effector
        
        rospy.loginfo(f"=== FORMATION INSERTION HEIGHT CALCULATION ===")
        rospy.loginfo(f"Valve height: {valve_z:.6f}m")
        rospy.loginfo(f"Current assembly Z: {current_assembly_z:.6f}m")
        rospy.loginfo(f"Safety margin: {safety_margin:.6f}m")
        rospy.loginfo(f"Target end-effector Z: {target_end_effector_z:.6f}m")
        rospy.loginfo(f"End-effector Z offset: {end_effector_z_offset:.6f}m")
        rospy.loginfo(f"Target Module2 Z for insertion: {target_module2_z:.6f}m")
        
        # Target Assembly Z: Module2 Z - assembly_to_leader_offset_z  
        target_assembly_z = target_module2_z - self.adapter.assembly_to_leader_offset_z
        
        self.locked_z = target_assembly_z
        rospy.loginfo(f"Formation assembly Z locked at: {self.locked_z:.6f}m")
        
        # Verify the calculation by working forward
        expected_module2_z = target_assembly_z + self.adapter.assembly_to_leader_offset_z
        expected_end_effector_z = expected_module2_z - end_effector_z_offset
        descent_distance = current_assembly_z - target_assembly_z
        
        rospy.loginfo(f"Verification:")
        rospy.loginfo(f"  Expected Module2 Z: {expected_module2_z:.6f}m")
        rospy.loginfo(f"  Expected end-effector Z: {expected_end_effector_z:.6f}m")
        rospy.loginfo(f"  Descent distance: {descent_distance:.6f}m")
        rospy.loginfo(f"  End-effector will be {expected_end_effector_z - valve_z:.6f}m above valve")
        
        # Set rotation direction in optimizer
        self.optimizer.set_rotation_direction(self.rotation_direction)
        
        # Step 1: Determine optimal contact point
        optimal_contact_point = self.determine_optimal_contact_point()
        if optimal_contact_point is None:
            rospy.logerr("Failed to determine optimal contact point")
            return 'failed'
        
        # Step 2: Insert into beam gap
        insertion_success = self.rotate_and_insert_to_beam_gap(optimal_contact_point)
        if not insertion_success:
            rospy.logerr("Failed to insert into beam gap")
            return 'failed'
        
        # Step 3: Move along valve circumference
        circumferential_success = self.move_along_valve_circumference(optimal_contact_point)
        if not circumferential_success:
            rospy.logerr("Failed to move along valve circumference")
            return 'failed'
        
        # Step 4: Prepare for valve rotation
        rotation_ready = self.prepare_for_valve_rotation(optimal_contact_point)
        if not rotation_ready:
            rospy.logerr("Failed to prepare for valve rotation")
            return 'failed'
        
        rospy.loginfo("=== FORMATION 4-STEP VALVE ENGAGEMENT COMPLETED SUCCESSFULLY ===")
        return 'succeeded'
    
    def execute_simple_insertion(self):
        """Formation版本：修复插入点位置计算的根本问题"""
        if not hasattr(self, 'current_insertion_params'):
            rospy.logerr("No insertion parameters available")
            return False
        
        params = self.current_insertion_params
        target_end_effector_pos = params['target_end_effector_position']  # Gap的世界坐标位置
        approach_angle = params['approach_angle']  # Gap中心角度
        
        rospy.loginfo(f"Formation执行修复版插入逻辑")
        rospy.loginfo(f"目标end-effector位置(世界坐标): {target_end_effector_pos}")
        rospy.loginfo(f"目标朝向角度: {approach_angle:.3f} rad ({math.degrees(approach_angle):.1f}°)")
        
        try:
            # 获取当前状态
            current_assembly_pos = self.get_assembly_position()
            current_yaw = self.adapter.get_assembly_yaw()
            if current_assembly_pos is None or current_yaw is None:
                rospy.logerr("无法获取当前assembly状态")
                return False
            
            rospy.loginfo(f"当前assembly位置: {current_assembly_pos}")
            rospy.loginfo(f"当前assembly yaw: {math.degrees(current_yaw):.1f}°")
            
            # === 关键修复：直接计算目标assembly位置 ===
            # 目标：end-effector到达gap位置，assembly朝向gap中心角度
            
            # 计算目标assembly位置：从end-effector目标位置反推
            target_assembly_pos = self.adapter.end_effector_to_assembly_transform(
                target_end_effector_pos, approach_angle, use_insertion_offset=True
            )
            
            if target_assembly_pos is None:
                rospy.logerr("无法计算目标assembly位置")
                return False
            
            rospy.loginfo(f"计算出的目标assembly位置: {target_assembly_pos}")
            
            # 验证计算的正确性
            verify_end_effector = self.adapter.assembly_to_end_effector_transform(
                target_assembly_pos, approach_angle, use_insertion_offset=True
            )
            
            if verify_end_effector:
                error = [abs(a-b) for a,b in zip(target_end_effector_pos, verify_end_effector)]
                rospy.loginfo(f"坐标变换验证 - 目标end-effector: {target_end_effector_pos}")
                rospy.loginfo(f"坐标变换验证 - 验算end-effector: {verify_end_effector}")
                rospy.loginfo(f"坐标变换验证 - 误差: {error}")
            
            # === PHASE 1: 移动到目标XY位置，保持当前高度和yaw ===
            rospy.loginfo("=== PHASE 1: 移动到目标XY位置 ===")
            
            # 创建中间目标：XY使用目标位置，Z保持当前高度，yaw保持当前角度
            intermediate_assembly_pos = (
                target_assembly_pos[0],  # 目标X
                target_assembly_pos[1],  # 目标Y
                current_assembly_pos[2]  # 保持当前Z
            )
            
            rospy.loginfo(f"Phase 1目标: XY到达目标位置，保持当前Z和yaw")
            rospy.loginfo(f"中间目标assembly位置: {intermediate_assembly_pos}")
            rospy.loginfo(f"保持当前yaw: {math.degrees(current_yaw):.1f}°")
            
            # 计算移动距离和时间
            horizontal_distance = math.sqrt(
                (intermediate_assembly_pos[0] - current_assembly_pos[0])**2 + 
                (intermediate_assembly_pos[1] - current_assembly_pos[1])**2
            )
            
            # 慢速安全移动
            base_speed = 0.04  # 4cm/s
            duration = max(12.0, horizontal_distance / base_speed)
            rospy.loginfo(f"水平移动距离: {horizontal_distance:.3f}m, 时长: {duration:.1f}s")
            
            # 执行水平移动
            success = self.execute_assembly_trajectory(
                current_assembly_pos, intermediate_assembly_pos, current_yaw, duration
            )
            if not success:
                rospy.logerr("Phase 1: 水平移动失败")
                return False
            
            rospy.loginfo("✓ Phase 1完成: 已到达目标XY位置")
            time.sleep(2.0)
            
            # === PHASE 2: yaw角度调整到目标朝向 ===
            rospy.loginfo("=== PHASE 2: yaw角度调整 ===")
            
            # 获取Phase 1后的位置（应该保持XY不变）
            current_pos_after_move = self.get_assembly_position()
            if current_pos_after_move is None:
                current_pos_after_move = intermediate_assembly_pos
            
            rospy.loginfo(f"Phase 1后的位置: {current_pos_after_move}")
            rospy.loginfo(f"目标yaw角度: {math.degrees(approach_angle):.1f}°")
            
            # 计算yaw变化
            current_yaw_after_move = self.adapter.get_assembly_yaw()
            if current_yaw_after_move is None:
                current_yaw_after_move = current_yaw
            
            yaw_diff = self.normalize_angle(approach_angle - current_yaw_after_move)
            rospy.loginfo(f"需要的yaw变化: {math.degrees(yaw_diff):.1f}°")
            
            # 执行yaw调整（位置锁定）
            if abs(yaw_diff) > 0.1:  # 超过5.7°才调整
                self.execute_pure_yaw_adjustment(current_pos_after_move, approach_angle, 4.0)
                rospy.loginfo("✓ Yaw调整完成")
            else:
                rospy.loginfo("✓ Yaw调整跳过（变化太小）")
            
            time.sleep(2.0)
            
            # === PHASE 3: 垂直下降到目标高度 ===
            rospy.loginfo("=== PHASE 3: 垂直下降 ===")
            
            # 获取yaw调整后的位置
            final_pos_before_descent = self.get_assembly_position()
            if final_pos_before_descent is None:
                final_pos_before_descent = current_pos_after_move
            
            # 目标位置：保持XY不变，只改变Z到目标高度
            final_target_pos = (
                final_pos_before_descent[0],  # X锁定
                final_pos_before_descent[1],  # Y锁定  
                target_assembly_pos[2]        # Z下降到目标
            )
            
            vertical_distance = abs(final_pos_before_descent[2] - final_target_pos[2])
            vertical_duration = max(4.0, vertical_distance / 0.03)  # 3cm/s下降
            
            rospy.loginfo(f"垂直下降距离: {vertical_distance:.3f}m, 时长: {vertical_duration:.1f}s")
            rospy.loginfo(f"下降前位置: {final_pos_before_descent}")
            rospy.loginfo(f"下降后目标: {final_target_pos}")
            
            # 执行垂直下降
            success = self.execute_vertical_descent_trajectory(
                final_pos_before_descent, final_target_pos, approach_angle, vertical_duration
            )
            
            if not success:
                rospy.logerr("Phase 3: 垂直下降失败")
                return False
            
            rospy.loginfo("✓ Phase 3完成: 垂直下降成功")
            
            # === 最终验证 ===
            time.sleep(1.0)
            final_assembly_pos = self.get_assembly_position()
            final_yaw = self.adapter.get_assembly_yaw()
            
            if final_assembly_pos and final_yaw is not None:
                actual_end_effector = self.adapter.assembly_to_end_effector_transform(
                    final_assembly_pos, final_yaw, use_insertion_offset=True
                )
                
                if actual_end_effector:
                    error = math.sqrt(sum((a-b)**2 for a,b in zip(target_end_effector_pos, actual_end_effector)))
                    rospy.loginfo("=== 最终验证 ===")
                    rospy.loginfo(f"目标end-effector: {target_end_effector_pos}")
                    rospy.loginfo(f"实际end-effector: {actual_end_effector}")
                    rospy.loginfo(f"总位置误差: {error:.4f}m")
                    
                    # 检查Y坐标是否修复
                    y_error = abs(actual_end_effector[1] - target_end_effector_pos[1])
                    rospy.loginfo(f"Y坐标误差修复检查: {y_error:.4f}m (应该<0.05m)")
            
            rospy.loginfo("✓ Formation插入逻辑修复版本完成")
            return True
            
        except Exception as e:
            rospy.logerr(f"Formation插入错误: {str(e)}")
            import traceback
            traceback.print_exc()
            return False
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def determine_optimal_contact_point(self):
        """
        Formation version: CORRECTED coordinate system usage
        1. Use ASSEMBLY position for distance calculation (not Module2)
        2. Calculate gaps relative to END-EFFECTOR position 
        3. Consider both distance and yaw for optimization
        """
        rospy.loginfo("--- Formation Step 1: Determining optimal contact point (CORRECTED COORDINATE SYSTEM) ---")
        
        if not self.optimizer:
            rospy.logerr("Optimizer not initialized")
            return None
        
        # CORRECTED: Use Assembly position for planning, but calculate end-effector position
        assembly_pos = self.adapter.get_assembly_position()
        if assembly_pos is None:
            rospy.logerr("Assembly position not available for insertion planning")
            return None
        
        # Get current assembly yaw for comparison and end-effector calculation
        current_yaw = self.adapter.get_assembly_yaw()
        if current_yaw is None:
            current_yaw = 0.0
            rospy.logwarn("Using default yaw=0.0 for gap selection")
        
        # CORRECTED: Calculate current end-effector position for distance measurement
        current_end_effector_pos = self.adapter.assembly_to_end_effector_transform(
            assembly_pos, current_yaw, use_insertion_offset=False
        )
        
        if current_end_effector_pos is None:
            rospy.logerr("Failed to calculate current end-effector position")
            return None
        
        rospy.loginfo(f"Current assembly position: {assembly_pos}")
        rospy.loginfo(f"Current end-effector position: {current_end_effector_pos}")
        rospy.loginfo(f"Current assembly yaw: {math.degrees(current_yaw):.1f}°")
        
        # Calculate all beam gaps manually
        beam_gaps = self.optimizer.calculate_beam_gaps(self.valve_pos, self.valve_yaw)
        
        if not beam_gaps:
            rospy.logerr("No beam gaps available")
            return None
        
        # CORRECTED: Evaluate each gap based on END-EFFECTOR distance and yaw change
        best_gap = None
        best_score = float('inf')
        best_gap_name = None
        
        rospy.loginfo("=== EVALUATING ALL BEAM GAPS (CORRECTED DISTANCE CALCULATION) ===")
        
        for gap_name, gap_info in beam_gaps.items():
            gap_pos = gap_info['position']
            gap_angle = gap_info['center_angle']
            
            # CORRECTED: Calculate 3D distance from CURRENT END-EFFECTOR to gap position
            dx = gap_pos[0] - current_end_effector_pos[0]
            dy = gap_pos[1] - current_end_effector_pos[1] 
            dz = self.valve_pos[2] - current_end_effector_pos[2]  # Z difference to valve height
            gap_distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # Calculate yaw change required
            yaw_diff = abs(self.normalize_angle(gap_angle - current_yaw))
            
            # Combined score: distance (m) + yaw_factor * yaw_change (rad)
            # Weight yaw change more heavily to avoid unnecessary rotations
            yaw_weight = 1.5  # meters equivalent per radian - reduced from 2.0 for balance
            combined_score = gap_distance + yaw_weight * yaw_diff
            
            rospy.loginfo(f"{gap_name}:")
            rospy.loginfo(f"  Gap position: ({gap_pos[0]:.3f}, {gap_pos[1]:.3f}, {self.valve_pos[2]:.3f})")
            rospy.loginfo(f"  End-effector distance: {gap_distance:.3f}m")
            rospy.loginfo(f"  Gap angle: {math.degrees(gap_angle):.1f}°")
            rospy.loginfo(f"  Yaw change: {math.degrees(yaw_diff):.1f}° ({yaw_diff:.3f} rad)")
            rospy.loginfo(f"  Combined score: {combined_score:.3f}")
            
            if combined_score < best_score:
                best_score = combined_score
                best_gap = gap_info
                best_gap_name = gap_name
                best_gap['distance'] = gap_distance
        
        if best_gap is None:
            rospy.logerr("Failed to select optimal gap")
            return None
        
        # CORRECTED: Target position is for END-EFFECTOR at valve height
        gap_pos = best_gap['position']
        target_end_effector_position = (gap_pos[0], gap_pos[1], self.valve_pos[2])  # End-effector target at valve Z
        
        # Build optimized insertion strategy
        strategy = {
            'insertion_mode': 'formation_corrected_coordinates',
            'selected_gap': best_gap_name,
            'gap_center_angle': best_gap['center_angle'],
            'target_end_effector_position': target_end_effector_position,  # CORRECTED: Specify this is end-effector target
            'approach_distance': best_gap['distance'],
            'beam_gap_radius': best_gap['radius'],
            'complexity_score': best_score,  # Combined distance + yaw score
        }
        
        # Store strategy for use in insertion phase
        self.current_insertion_params = {
            'simple_strategy': strategy,
            'selected_gap': strategy['selected_gap'],
            'gap_center_angle': strategy['gap_center_angle'],
            'target_end_effector_position': strategy['target_end_effector_position'],  # CORRECTED: Store as end-effector target
            'approach_distance': strategy['approach_distance'],
            'approach_angle': strategy['gap_center_angle']
        }
        
        # Create contact point information for compatibility with single UAV interface
        contact_point = {
            'position': strategy['target_end_effector_position'],  # CORRECTED: This is end-effector position
            'angle': strategy['gap_center_angle'],
            'distance': strategy['approach_distance'],
            'strategy': 'formation_corrected_coordinates'
        }
        
        rospy.loginfo("=== OPTIMAL GAP SELECTED (CORRECTED COORDINATE SYSTEM) ===")
        rospy.loginfo(f"Selected gap: {best_gap_name}")
        rospy.loginfo(f"Combined score: {best_score:.3f}")
        rospy.loginfo(f"Gap center angle: {math.degrees(contact_point['angle']):.1f}°")
        rospy.loginfo(f"End-effector distance: {contact_point['distance']:.3f}m")
        yaw_change = abs(self.normalize_angle(contact_point['angle'] - current_yaw))
        rospy.loginfo(f"Required yaw change: {math.degrees(yaw_change):.1f}°")
        
        rospy.loginfo("=== FORMATION CORRECTED COORDINATE INSERTION STRATEGY ===")
        rospy.loginfo(f"Target end-effector position: ({contact_point['position'][0]:.3f}, {contact_point['position'][1]:.3f}, {contact_point['position'][2]:.3f})")
        rospy.loginfo(f"Target assembly angle: {math.degrees(contact_point['angle']):.1f}°")
        rospy.loginfo(f"Selected gap: {best_gap_name}")
        rospy.loginfo(f"Distance optimization score: {contact_point['distance']:.3f}m")
        
        return contact_point
    
    def execute_simple_insertion(self):
        """
        FORMATION VERSION: Execute simplified insertion strategy with correct coordinate transformations
        This overrides the single UAV version to handle Formation coordinate system properly
        """
        if not hasattr(self, 'current_insertion_params'):
            rospy.logerr("No insertion parameters available")
            return False
        
        params = self.current_insertion_params
        target_end_effector_position = params['target_end_effector_position']  # This is end-effector target
        approach_angle = params['approach_angle']  # This is assembly yaw angle
        
        rospy.loginfo(f"=== FORMATION EXECUTE SIMPLE INSERTION ===")
        rospy.loginfo(f"Target end-effector position: {target_end_effector_position}")
        rospy.loginfo(f"Target assembly angle: {approach_angle:.3f} rad ({approach_angle*180/3.14159:.1f}°)")
        
        try:
            # === CRITICAL FIX: 分阶段插入以避免Y坐标错误 ===
            # PHASE 1: Move to correct XY position with CURRENT yaw (avoid Y-axis drift)
            
            # Get current assembly state
            current_assembly_pos = self.adapter.get_assembly_position()
            current_yaw = self.adapter.get_assembly_yaw()
            if not current_assembly_pos or current_yaw is None:
                rospy.logerr("Cannot get current assembly state")
                return False
            
            rospy.loginfo("PHASE 1: Moving to correct XY position with current yaw...")
            rospy.loginfo(f"Current assembly yaw: {math.degrees(current_yaw):.1f}°")
            rospy.loginfo(f"Target approach yaw: {math.degrees(approach_angle):.1f}°")
            
            # Calculate pre-insertion position using CURRENT yaw to avoid Y-drift
            pre_insertion_end_effector = list(target_end_effector_position)
            pre_insertion_end_effector[2] += self.pre_insertion_distance  # Hover above target
            
            # CRITICAL FIX: Use CURRENT yaw for coordinate transform to avoid Y-drift
            pre_insertion_assembly_pos_current_yaw = self.adapter.end_effector_to_assembly_transform(
                pre_insertion_end_effector, current_yaw, use_insertion_offset=False
            )
            
            if pre_insertion_assembly_pos_current_yaw is None:
                rospy.logerr("Failed to transform pre-insertion position with current yaw")
                return False
            
            rospy.loginfo(f"Pre-insertion end-effector position: {pre_insertion_end_effector}")
            rospy.loginfo(f"Pre-insertion assembly (current yaw): {pre_insertion_assembly_pos_current_yaw}")
            
            # Move to XY position while keeping current yaw
            success = self.motion_controller.execute_smooth_trajectory_with_yaw(
                start_pos=current_assembly_pos,
                target_pos=pre_insertion_assembly_pos_current_yaw,
                target_yaw=current_yaw,  # Keep current yaw during XY movement
                duration=12.0,
                pos_threshold=0.05,
                yaw_threshold=0.2
            )
            
            if not success:
                rospy.logerr("Failed to reach pre-insertion position")
                return False
            
            # Step 1b: Adjust yaw orientation to approach angle
            final_assembly_pos = self.adapter.get_assembly_position()
            if final_assembly_pos is None:
                final_assembly_pos = pre_insertion_assembly_pos_current_yaw
            
            rospy.loginfo("PHASE 1b: Adjusting yaw for insertion approach...")
            rospy.loginfo(f"Target yaw: {approach_angle:.3f} rad ({math.degrees(approach_angle):.1f}°)")
            
            # Send trajectory point through formation motion controller
            self.motion_controller.send_trajectory_point(final_assembly_pos, approach_angle)
            
            # Wait for yaw convergence
            yaw_adjustment_start = time.time()
            rate = rospy.Rate(10)
            
            while (time.time() - yaw_adjustment_start) < 8.0 and not rospy.is_shutdown():
                current_assembly_yaw = self.adapter.get_assembly_yaw()
                if current_assembly_yaw is not None:
                    yaw_error = abs(self.normalize_angle(current_assembly_yaw - approach_angle))
                    if yaw_error < 0.2:  # yaw_threshold
                        rospy.loginfo(f"Yaw adjustment completed. Error: {yaw_error:.3f} rad")
                        break
                
                self.motion_controller.send_trajectory_point(final_assembly_pos, approach_angle)
                rate.sleep()
            
            time.sleep(2.0)  # Pause for stabilization
            
            # === PHASE 2: 严格的垂直下降（只改变Z，锁定XY） ===
            rospy.loginfo("PHASE 2: 纯垂直下降到插入位置...")
            
            # 获取当前assembly位置（旋转后的位置）
            current_assembly_pos = self.adapter.get_assembly_position()
            if not current_assembly_pos:
                rospy.logerr("Cannot get current assembly position for descent")
                return False
            
            # 计算目标Z坐标（仅下降Z，XY保持不变）
            target_end_effector_z = target_end_effector_position[2]  # 目标end-effector Z
            current_end_effector_pos = self.adapter.assembly_to_end_effector_transform(
                current_assembly_pos, approach_angle, use_insertion_offset=False
            )
            
            if current_end_effector_pos is None:
                rospy.logerr("Cannot transform current assembly to end-effector position")
                return False
            
            # 计算需要的Z下降距离
            z_descent = current_end_effector_pos[2] - target_end_effector_z
            target_assembly_z = current_assembly_pos[2] - z_descent  # 相应的assembly Z下降
            
            # 目标assembly位置：只改变Z，XY完全锁定
            final_assembly_target = (
                current_assembly_pos[0],  # X锁定
                current_assembly_pos[1],  # Y锁定
                target_assembly_z         # 只改变Z
            )
            
            rospy.loginfo(f"当前end-effector位置: {current_end_effector_pos}")
            rospy.loginfo(f"目标end-effector位置: {target_end_effector_position}")
            rospy.loginfo(f"Z下降距离: {z_descent:.3f}m")
            rospy.loginfo(f"当前assembly位置: {current_assembly_pos}")
            rospy.loginfo(f"目标assembly位置(仅Z变化): {final_assembly_target}")
            rospy.loginfo(f"XY坐标完全锁定，只进行垂直下降")
            
            # 执行纯垂直下降
            descent_duration = max(3.0, z_descent / 0.05)  # 5cm/s的慢速下降
            rospy.loginfo(f"垂直下降时间: {descent_duration:.1f}s")
            
            success = self.motion_controller.execute_vertical_descent(
                start_pos=current_assembly_pos,
                target_pos=final_assembly_target,
                target_yaw=approach_angle,  # 保持旋转后的yaw角度
                duration=descent_duration
            )
            
            if not success:
                rospy.logerr("Failed to reach insertion position")
                return False
            
            time.sleep(1.0)  # Brief pause for stabilization
            
            # VERIFICATION: Check final end-effector position
            final_assembly_pos = self.adapter.get_assembly_position()
            final_assembly_yaw = self.adapter.get_assembly_yaw()
            
            if final_assembly_pos and final_assembly_yaw is not None:
                actual_end_effector_pos = self.adapter.assembly_to_end_effector_transform(
                    final_assembly_pos, final_assembly_yaw, use_insertion_offset=False
                )
                if actual_end_effector_pos:
                    error_x = actual_end_effector_pos[0] - target_end_effector_position[0]
                    error_y = actual_end_effector_pos[1] - target_end_effector_position[1]
                    error_z = actual_end_effector_pos[2] - target_end_effector_position[2]
                    position_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)
                    
                    rospy.loginfo("=== INSERTION VERIFICATION ===")
                    rospy.loginfo(f"Target end-effector: {target_end_effector_position}")
                    rospy.loginfo(f"Actual end-effector: {actual_end_effector_pos}")
                    rospy.loginfo(f"Position error: {position_error:.4f}m")
                    rospy.loginfo(f"Error components: X={error_x:.4f}, Y={error_y:.4f}, Z={error_z:.4f}")
            
            rospy.loginfo("Formation simple insertion completed successfully")
            return True
            
        except Exception as e:
            rospy.logerr(f"Error during formation simple insertion: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    # All other methods from DescendAndContactState are inherited automatically through multiple inheritance


class FormationRotateValveState(RotateValveState, FormationSingleUAVStateBase):
    """Formation version of RotateValveState - reuses single UAV logic"""
    
    def __init__(self, rotation_angle=math.pi/2, rotation_duration=18.0, rotation_direction=1):
        # Initialize both parent classes
        RotateValveState.__init__(self, rotation_angle=rotation_angle, 
                                 rotation_duration=rotation_duration, 
                                 rotation_direction=rotation_direction)
        FormationSingleUAVStateBase.__init__(self, outcomes=['succeeded', 'failed', 'emergency'], 
                                           input_keys=['start_position'])
        
        # Override motion controller to use formation version
        self.motion_controller = self.formation_motion_controller
        
        # Emergency detection parameters (same as single UAV)
        self.emergency_stop = threading.Event()
        self.emergency_triggered = False
        self.last_position = None
        self.last_yaw = 0.0
        self.stuck_start_time = None
        self.stuck_threshold = 60.0
        self.movement_threshold = 0.005
        self.yaw_threshold = 0.02
    
    def execute(self, userdata):
        """Formation valve rotation using EXACT single UAV logic - rotate around valve center"""
        rospy.loginfo("Formation valve rotation - rotating around VALVE CENTER (like single UAV)...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        # CRITICAL FIX: Use end-effector position for rotation trajectory 
        # This ensures rotation happens around valve center, NOT around assembly CoG
        current_end_effector_pos = self.get_current_position()
        if current_end_effector_pos is None:
            rospy.logerr("Cannot get current end-effector position")
            return 'failed'
        
        # Get current UAV yaw (for end-effector, use Module2 yaw)
        current_module2_yaw = self.adapter.get_module2_yaw()
        if current_module2_yaw is None:
            rospy.logerr("Cannot get Module2 yaw")
            return 'failed'
        
        # Initialize tracking with end-effector position
        self.last_position = current_end_effector_pos
        self.last_yaw = current_module2_yaw
        self.stuck_start_time = None
        
        rospy.loginfo(f"Formation rotation starting from end-effector position: {current_end_effector_pos}")
        rospy.loginfo(f"Valve center: {self.valve_pos}")
        rospy.loginfo(f"Current Module2 yaw: {math.degrees(current_module2_yaw):.1f}°")
        rospy.loginfo(f"Rotation angle: {math.degrees(self.rotation_angle):.1f}°")
        rospy.loginfo(f"Rotation direction: {'Clockwise' if self.rotation_direction == 1 else 'Counter-clockwise'}")
        
        # Calculate distance from end-effector to valve center (should maintain this)
        valve_distance = math.sqrt(
            (current_end_effector_pos[0] - self.valve_pos[0])**2 + 
            (current_end_effector_pos[1] - self.valve_pos[1])**2
        )
        rospy.loginfo(f"End-effector distance from valve center: {valve_distance:.3f}m")
        
        # Create rotation trajectory using SAME logic as single UAV
        # Key: Use end-effector position as "current_uav_pos" and zero offsets
        rotation_traj = create_constant_distance_trajectory(
            current_uav_pos=current_end_effector_pos,  # End-effector position (not assembly!)
            current_uav_yaw=current_module2_yaw,       # Module2 yaw (not assembly yaw!)
            valve_center=self.valve_pos,
            rotation_angle=self.rotation_angle,
            rotation_duration=self.rotation_duration,
            end_effector_offset_x=0.0,  # Zero offset since we're already at end-effector
            end_effector_offset_y=0.0,  # Zero offset since we're already at end-effector  
            end_effector_offset_z=0.0,  # Zero offset since we're already at end-effector
            rotation_direction=self.rotation_direction
        )
        
        if rotation_traj is None:
            rospy.logerr("Failed to create rotation trajectory")
            return 'failed'
        
        rospy.loginfo("✓ Rotation trajectory created - rotating around valve center")
        
        # Start trajectory
        rotation_traj.start_trajectory()
        
        # Start emergency monitoring
        self.start_emergency_monitoring()
        
        # Execute rotation using formation transformation
        try:
            success = self.execute_formation_rotation_trajectory(rotation_traj)
            
            # Stop emergency monitoring
            self.stop_emergency_monitoring()
            
            # Check if emergency was triggered
            if hasattr(self, 'emergency_triggered') and self.emergency_triggered:
                rospy.logwarn("Emergency detected during rotation")
                return 'emergency'
            
            if success:
                rospy.loginfo("✓ Formation valve rotation completed successfully")
                return 'succeeded'
            else:
                rospy.logerr("Formation valve rotation failed")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during formation rotation: {e}")
            import traceback
            traceback.print_exc()
            self.emergency_stop.set()
            return 'failed'
    
    def execute_formation_rotation_trajectory(self, rotation_traj):
        """
        执行Formation旋转轨迹：实现标准的"公转+自转"控制
        - 公转：end-effector围绕阀门中心做圆周运动
        - 自转：保持end-effector朝向始终指向阀门中心
        - Z坐标严格锁定，禁止垂直漂移
        """
        rate = rospy.Rate(10)  # 降低频率到10Hz以实现更稳定控制
        start_time = time.time()
        
        rospy.loginfo("开始Formation旋转轨迹执行：标准'公转+自转'模式，Z坐标锁定")
        
        # 严格锁定Z坐标在当前assembly位置
        initial_assembly_pos = self.get_assembly_position()
        if initial_assembly_pos is None:
            rospy.logerr("无法获取初始assembly位置进行Z坐标锁定")
            return False
        
        locked_assembly_z = initial_assembly_pos[2]
        rospy.loginfo(f"Formation旋转Z坐标严格锁定在: {locked_assembly_z:.6f}m")
        
        # 降低速度以实现精确控制
        max_linear_velocity = 0.025  # 2.5cm/s的慢速线性运动
        max_angular_velocity = 0.15  # 降低角速度到0.15 rad/s (约8.6°/s)
        position_tolerance = 0.015   # 1.5cm位置公差
        yaw_tolerance = 0.04         # 约2.3°的yaw公差
        
        while not rospy.is_shutdown() and not self.emergency_stop.is_set():
            # 获取下一个轨迹点（end-effector坐标）
            result = rotation_traj.get_next_uav_position_and_yaw()
            if result is None:
                rospy.loginfo("旋转轨迹完成")
                break
            
            target_end_effector_pos, target_yaw = result
            current_time = time.time()
            
            # 将end-effector目标转换为assembly控制指令
            target_assembly_pos = self.adapter.end_effector_to_assembly_transform(
                target_end_effector_pos, target_yaw, use_insertion_offset=True
            )
            
            if target_assembly_pos is None:
                rospy.logwarn("无法将end-effector坐标转换为assembly坐标")
                continue
            
            # 严格强制Z坐标锁定（完全禁止垂直移动）
            target_assembly_pos = (target_assembly_pos[0], target_assembly_pos[1], locked_assembly_z)
            
            # 获取当前assembly位置和yaw进行精确反馈控制
            current_assembly_pos = self.get_assembly_position()
            current_yaw = self.adapter.get_assembly_yaw()
            
            if current_assembly_pos is None:
                rospy.logwarn("无法获取当前assembly位置进行控制")
                self.motion_controller.send_trajectory_point(target_assembly_pos, target_yaw)
                rate.sleep()
                continue
            
            # 计算位置误差（XYZ分量）
            pos_error_x = target_assembly_pos[0] - current_assembly_pos[0]
            pos_error_y = target_assembly_pos[1] - current_assembly_pos[1] 
            pos_error_z = locked_assembly_z - current_assembly_pos[2]  # Z误差修正到锁定高度
            
            # 计算yaw误差
            if current_yaw is not None:
                yaw_error = self.normalize_angle(target_yaw - current_yaw)
            else:
                yaw_error = 0.0
                rospy.logwarn("无法获取当前yaw角度")
            
            # 使用比例控制计算速度指令（简单但稳定）
            kp_pos = 1.5  # 位置比例增益
            kp_z = 2.5    # Z方向更强的修正增益
            kp_yaw = 0.8  # yaw比例增益
            
            # 计算速度指令
            cmd_vel_x = kp_pos * pos_error_x
            cmd_vel_y = kp_pos * pos_error_y
            cmd_vel_z = kp_z * pos_error_z     # Z方向强制修正到锁定高度
            cmd_yaw_vel = kp_yaw * yaw_error
            
            # 速度限制（防止过快运动）
            cmd_vel_x = max(-max_linear_velocity, min(max_linear_velocity, cmd_vel_x))
            cmd_vel_y = max(-max_linear_velocity, min(max_linear_velocity, cmd_vel_y))
            cmd_vel_z = max(-max_linear_velocity, min(max_linear_velocity, cmd_vel_z))
            cmd_yaw_vel = max(-max_angular_velocity, min(max_angular_velocity, cmd_yaw_vel))
            
            # 发送速度控制指令
            cmd_vel = (cmd_vel_x, cmd_vel_y, cmd_vel_z)
            self.motion_controller.send_velocity_command(
                cmd_vel, cmd_yaw_vel, target_assembly_pos, target_yaw
            )
            
            # 定期记录旋转进度和精度验证
            elapsed = current_time - start_time
            if elapsed % 1.0 < 0.1:  # 每秒记录一次进度
                pos_error_magnitude = math.sqrt(pos_error_x**2 + pos_error_y**2 + pos_error_z**2)
                yaw_error_deg = math.degrees(abs(yaw_error))
                
                rospy.loginfo(f"Formation旋转[{elapsed:.1f}s]: 位置误差={pos_error_magnitude:.4f}m, yaw误差={yaw_error_deg:.2f}°")
                rospy.loginfo(f"  目标: end-effector={target_end_effector_pos}, assembly={target_assembly_pos}")
                rospy.loginfo(f"  当前: assembly={current_assembly_pos}, yaw={math.degrees(current_yaw):.1f}°")
                rospy.loginfo(f"  Z锁定验证: 目标={locked_assembly_z:.6f}, 当前={current_assembly_pos[2]:.6f}, 误差={abs(pos_error_z):.6f}")
                
                # 检查收敛状态
                if pos_error_magnitude < position_tolerance and abs(yaw_error) < yaw_tolerance:
                    rospy.loginfo(f"  ✓ 已收敛到目标精度 (位置<{position_tolerance}m, yaw<{math.degrees(yaw_tolerance):.1f}°)")
            
            rate.sleep()
        
        rospy.loginfo("Formation旋转轨迹完成 - 允许长时间稳定")
        time.sleep(6.0)  # 延长稳定时间到6秒确保完全稳定
        return True
    
    def execute_rotation_trajectory(self, rotation_traj):
        """Reuse single UAV rotation trajectory logic"""
        rate = rospy.Rate(20)
        start_time = time.time()
        
        while not rospy.is_shutdown() and not self.emergency_stop.is_set():
            result = rotation_traj.get_next_uav_position_and_yaw()
            if result is None:
                break
            
            target_pos, target_yaw = result
            self.motion_controller.send_trajectory_point(target_pos, target_yaw)
            rate.sleep()
        
        rospy.loginfo("Formation rotation trajectory completed")
        time.sleep(3.0)
        return True


def main():
    rospy.init_node('valve_rotation_formation_refactored')
    
    rotation_direction = rospy.get_param('~rotation_direction', 1)
    rospy.loginfo(f"Starting formation valve rotation (refactored version)")
    rospy.loginfo(f"Rotation direction: {'Clockwise' if rotation_direction == 1 else 'Counter-clockwise'}")
    
    # Create state machine using formation versions of single UAV states
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    sm.userdata.start_position = None
    
    with sm:
        # Step 1: Physical assembly of UAV modules
        smach.StateMachine.add(
            'ASSEMBLE',
            FormationAssembleState(),
            transitions={'succeeded': 'INITIALIZE_START_POSITION', 'failed': 'failed'}
        )
        
        # Step 2: Initialize start position (now that UAVs are assembled)
        smach.StateMachine.add(
            'INITIALIZE_START_POSITION',
            FormationInitializeStartPositionState(),
            transitions={'succeeded': 'MOVE_TO_VALVE', 'failed': 'failed'}
        )
        
        # Step 3: Move to valve approach position
        smach.StateMachine.add(
            'MOVE_TO_VALVE',
            FormationMoveToValveState(approach_distance=0.8, approach_height=0.3),
            transitions={'succeeded': 'DESCEND_AND_CONTACT', 'failed': 'failed'}
        )
        
        # Step 4: Descend and contact valve
        smach.StateMachine.add(
            'DESCEND_AND_CONTACT',
            FormationDescendAndContactState(rotation_direction=rotation_direction),
            transitions={'succeeded': 'ROTATE_VALVE', 'failed': 'failed'}
        )
        
        # Step 5: Rotate valve
        smach.StateMachine.add(
            'ROTATE_VALVE',
            FormationRotateValveState(rotation_angle=math.pi/2, rotation_duration=18.0, rotation_direction=rotation_direction),
            transitions={
                'succeeded': 'succeeded',
                'failed': 'failed',
                'emergency': 'failed'
            }
        )
    
    # Create introspection server
    sis = smach_ros.IntrospectionServer('formation_valve_rotation_state_machine', sm, '/SM_ROOT')
    sis.start()
    
    try:
        rospy.loginfo("Starting formation valve rotation state machine (refactored)...")
        outcome = sm.execute()
        rospy.loginfo(f"Formation state machine completed with outcome: {outcome}")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Formation valve rotation interrupted")
    except Exception as e:
        rospy.logerr(f"Formation valve rotation error: {e}")
    finally:
        # Clean up emergency monitoring threads
        for state_name, state in sm._states.items():
            if hasattr(state, 'stop_emergency_monitoring'):
                try:
                    state.stop_emergency_monitoring()
                    rospy.loginfo(f"Stopped emergency monitoring for state: {state_name}")
                except Exception as e:
                    rospy.logwarn(f"Error stopping emergency monitoring for {state_name}: {e}")
        
        sis.stop()
        rospy.loginfo("Formation valve rotation node cleanup completed")


if __name__ == '__main__':
    main()
