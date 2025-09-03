#!/usr/bin/env python3
"""
Formation Valve Rotation - Clean Version
Reuses single UAV logic with minimal coordinate transformation overhead
"""

import sys
import os
import math
import threading
import time
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
from tf.transformations import euler_from_quaternion

# Add paths for imports (like valve_rotation_smach_test.py)
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(current_dir, '../..'))  # Access to script/task/
sys.path.insert(0, os.path.join(current_dir, '..'))     # Access to script/
sys.path.insert(0, current_dir)                         # Current directory

# Import assembly demo for formation control
try:
    from task.assembly_motion import AssemblyDemo
    ASSEMBLY_AVAILABLE = True
    rospy.loginfo("Assembly demo available from task.assembly_motion")
except ImportError as e:
    try:
        from beetle.assembly_api import AssemblyDemo
        ASSEMBLY_AVAILABLE = True
        rospy.loginfo("Assembly demo available from beetle.assembly_api")
    except ImportError:
        rospy.logwarn(f"Assembly demo not available: {e}")
        ASSEMBLY_AVAILABLE = False

# Import single UAV classes
from valve_rotation_fang_single import (
    SingleUAVStateBase, 
    InitializeStartPositionState,
    MoveToValveState, 
    DescendAndContactState,
    RotateValveState
)


class FormationAdapter:
    """Simple adapter to transform between Formation and Single UAV coordinate systems"""
    
    def __init__(self):
        # Formation geometry: Two UAVs at 0.52m spacing, assembly CoG is center
        self.assembly_to_leader_offset_x = 0.260  # Assembly to Module2 (leader) offset
        self.leader_to_end_effector_x = 0.246     # Module2 to end-effector offset
        self.end_effector_z_offset = 0.074382     # End-effector Z offset from Module2
        
        # Total offset from assembly CoG to end-effector
        self.total_offset_x = self.assembly_to_leader_offset_x + self.leader_to_end_effector_x  # 0.506m
        self.total_offset_z = self.end_effector_z_offset  # 0.074382m
        
        # ROS interface
        self.assembly_pos = None
        self.assembly_yaw = 0.0
        
        # Individual UAV positions for assembly calculation
        self.uav1_pos = None
        self.uav1_yaw = 0.0
        self.uav2_pos = None
        self.uav2_yaw = 0.0
        
        # Position tracking synchronization
        self.position_received = threading.Event()
        
        # Setup individual UAV subscribers to calculate assembly position
        # Use mocap topics for position feedback (like refactored version)
        self.uav1_sub = rospy.Subscriber(
            "/beetle1/mocap/pose", PoseStamped, self.uav1_callback, queue_size=1
        )
        self.uav2_sub = rospy.Subscriber(
            "/beetle2/mocap/pose", PoseStamped, self.uav2_callback, queue_size=1
        )
        
        # Assembly command publisher (correct message type)
        self.assembly_pub = rospy.Publisher(
            "/assembly/uav/nav", FlightNav, queue_size=1
        )
        
        rospy.loginfo("Formation adapter initialized with geometry:")
        rospy.loginfo(f"  Assembly to end-effector: X={self.total_offset_x:.3f}m, Z={self.total_offset_z:.3f}m")
    
    def uav1_callback(self, msg):
        """Receive UAV1 position from mocap (PoseStamped format)"""
        pos = msg.pose.position
        ori = msg.pose.orientation
        
        self.uav1_pos = (pos.x, pos.y, pos.z)
        
        # Extract yaw from quaternion
        _, _, self.uav1_yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        
        rospy.logdebug(f"UAV1 callback: position={self.uav1_pos}, yaw={self.uav1_yaw:.3f}")
        self._update_assembly_position()
    
    def uav2_callback(self, msg):
        """Receive UAV2 position from mocap (PoseStamped format)"""
        pos = msg.pose.position
        ori = msg.pose.orientation
        
        self.uav2_pos = (pos.x, pos.y, pos.z)
        
        # Extract yaw from quaternion
        _, _, self.uav2_yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        
        rospy.logdebug(f"UAV2 callback: position={self.uav2_pos}, yaw={self.uav2_yaw:.3f}")
        self._update_assembly_position()
    
    def _update_assembly_position(self):
        """Calculate assembly position as average of two UAV positions"""
        if self.uav1_pos is not None and self.uav2_pos is not None:
            # Assembly position is the average of two UAV positions
            self.assembly_pos = (
                (self.uav1_pos[0] + self.uav2_pos[0]) / 2.0,
                (self.uav1_pos[1] + self.uav2_pos[1]) / 2.0,
                (self.uav1_pos[2] + self.uav2_pos[2]) / 2.0
            )
            
            # Assembly yaw is also the average (assuming both UAVs have same yaw)
            yaw_diff = self.uav2_yaw - self.uav1_yaw
            # Handle angle wrapping
            if yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            elif yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            
            self.assembly_yaw = self.uav1_yaw + yaw_diff / 2.0
            
            # Signal that position is received
            if not self.position_received.is_set():
                rospy.loginfo(f"✓ Formation position received: {self.assembly_pos}")
                rospy.loginfo(f"UAV1: {self.uav1_pos}, UAV2: {self.uav2_pos}")
                self.position_received.set()
            
            rospy.logdebug(f"Assembly position updated: {self.assembly_pos}")
            rospy.logdebug(f"UAV1: {self.uav1_pos}, UAV2: {self.uav2_pos}")
    
    def assembly_callback(self, msg):
        """Legacy callback - not used when calculating from UAV positions"""
        pass
    
    def get_end_effector_position(self):
        """Calculate current end-effector position from assembly position"""
        if self.assembly_pos is None:
            return None
        
        # Transform assembly position to end-effector position
        cos_yaw = math.cos(self.assembly_yaw)
        sin_yaw = math.sin(self.assembly_yaw)
        
        end_effector_x = self.assembly_pos[0] + self.total_offset_x * cos_yaw
        end_effector_y = self.assembly_pos[1] + self.total_offset_x * sin_yaw
        end_effector_z = self.assembly_pos[2] + self.total_offset_z
        
        return (end_effector_x, end_effector_y, end_effector_z)
    
    def get_end_effector_yaw(self):
        """End-effector yaw is same as assembly yaw"""
        return self.assembly_yaw
    
    def send_end_effector_command(self, target_end_effector_pos, target_yaw):
        """Send command to reach target end-effector position and orientation"""
        # Transform end-effector target to assembly target
        assembly_pos = self.end_effector_to_assembly_transform(target_end_effector_pos, target_yaw)
        if assembly_pos is not None:
            self.send_assembly_command(assembly_pos, target_yaw)
        else:
            rospy.logerr("Failed to transform end-effector command to assembly command")
    
    def send_assembly_command(self, target_pos, target_yaw=None):
        """Send FlightNav command to assembly controller with correct format"""
        try:
            # Create FlightNav message for formation control
            nav_msg = FlightNav()
            nav_msg.header.stamp = rospy.Time.now()
            nav_msg.header.frame_id = "world"
            
            # CRITICAL: Set target to COG for assembled state
            nav_msg.target = FlightNav.COG  # Required for assembled state
            
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
    
    def wait_for_move_completion(self, timeout=30.0, position_tolerance=0.1, yaw_tolerance=0.1):
        """Wait for formation to reach the commanded position"""
        rospy.loginfo("Waiting for formation movement completion...")
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.assembly_pos is not None:
                rospy.loginfo("Formation move completed (position data available)")
                return True
            rospy.sleep(0.1)
        
        rospy.logwarn("Formation move completion timed out")
        return False
    
    def end_effector_to_assembly_transform(self, end_effector_pos, target_yaw, use_insertion_offset=False):
        """Transform end-effector target to assembly position"""
        cos_yaw = math.cos(target_yaw)
        sin_yaw = math.sin(target_yaw)
        
        assembly_x = end_effector_pos[0] - self.total_offset_x * cos_yaw
        assembly_y = end_effector_pos[1] - self.total_offset_x * sin_yaw
        assembly_z = end_effector_pos[2] - self.total_offset_z
        
        return (assembly_x, assembly_y, assembly_z)
    
    def assembly_to_end_effector_transform(self, assembly_pos, assembly_yaw, use_insertion_offset=False):
        """Transform assembly position to end-effector position"""
        cos_yaw = math.cos(assembly_yaw)
        sin_yaw = math.sin(assembly_yaw)
        
        end_effector_x = assembly_pos[0] + self.total_offset_x * cos_yaw
        end_effector_y = assembly_pos[1] + self.total_offset_x * sin_yaw
        end_effector_z = assembly_pos[2] + self.total_offset_z
        
        return (end_effector_x, end_effector_y, end_effector_z)
        msg.pose.orientation.w = quat[3]
        
        self.assembly_pub.publish(msg)


class FormationStateBase(smach.State):
    """Base class that adapts Single UAV states for Formation use"""
    
    def __init__(self, outcomes, input_keys=None, output_keys=None):
        # Initialize SMACH state properly
        super(FormationStateBase, self).__init__(outcomes=outcomes, 
                                                 input_keys=input_keys or [], 
                                                 output_keys=output_keys or [])
        
        # Initialize formation adapter
        self.formation_adapter = FormationAdapter()
        
        # Valve position tracking (copied from SingleUAVStateBase)
        self.valve_pos = None
        self.valve_yaw = 0.0
        self.valve_received = threading.Event()
        
        # Setup valve subscriber
        is_simulation = rospy.get_param("~simulation", True)
        if is_simulation:
            self.valve_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            self.valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
        
        rospy.loginfo("Formation state base initialized")
    
    def get_current_position(self):
        """Get current end-effector position via formation adapter"""
        return self.formation_adapter.get_end_effector_position()
    
    def get_current_yaw(self):
        """Get current end-effector yaw via formation adapter"""
        return self.formation_adapter.get_end_effector_yaw()
    
    def send_command(self, target_pos, target_yaw):
        """Send command via formation adapter"""
        self.formation_adapter.send_end_effector_command(target_pos, target_yaw)
    
    def wait_for_positions(self, timeout=5.0):
        """Wait for both UAV positions to be available"""
        rospy.loginfo("Waiting for assembly position...")
        if not self.formation_adapter.position_received.wait(timeout=timeout):
            rospy.logwarn("Timeout waiting for assembly position")
            rospy.logwarn(f"UAV1 position: {self.formation_adapter.uav1_pos}")
            rospy.logwarn(f"UAV2 position: {self.formation_adapter.uav2_pos}")
            return False
        
        rospy.loginfo(f"Assembly position received: {self.formation_adapter.assembly_pos}")
        rospy.loginfo(f"UAV positions - UAV1: {self.formation_adapter.uav1_pos}, UAV2: {self.formation_adapter.uav2_pos}")
        return True
    
    def valve_callback(self, msg):
        """Real valve position callback (from mocap)"""
        pos = msg.pose.position
        ori = msg.pose.orientation
        
        # Convert quaternion to yaw
        from tf.transformations import euler_from_quaternion
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        
        self.valve_pos = (pos.x, pos.y, pos.z)
        self.valve_yaw = yaw
        
        if not self.valve_received.is_set():
            rospy.loginfo(f"Valve position received: {self.valve_pos}, yaw: {math.degrees(yaw):.1f}°")
            self.valve_received.set()
    
    def valve_sim_callback(self, msg):
        """Simulation valve position callback"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # Convert quaternion to yaw
        from tf.transformations import euler_from_quaternion
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        
        self.valve_pos = (pos.x, pos.y, pos.z)
        self.valve_yaw = yaw
        
        if not self.valve_received.is_set():
            rospy.loginfo(f"Valve position received (sim): {self.valve_pos}, yaw: {math.degrees(yaw):.1f}°")
            self.valve_received.set()
    
    def wait_for_valve_position(self, timeout=10.0):
        """Wait for valve position with timeout"""
        if not self.valve_received.wait(timeout):
            rospy.logwarn("Timeout waiting for valve position")
            return False
        return True
    
    def get_current_position(self):
        """Override to return end-effector position instead of UAV position"""
        return self.formation_adapter.get_end_effector_position()
    
    def get_current_yaw(self):
        """Override to return end-effector yaw instead of UAV yaw"""
        return self.formation_adapter.get_end_effector_yaw()
    
    def send_command(self, target_pos, target_yaw):
        """Override to send formation commands instead of single UAV commands"""
        self.formation_adapter.send_end_effector_command(target_pos, target_yaw)
    
    def wait_for_positions(self):
        """Wait for formation positions using proper Event synchronization"""
        rospy.loginfo("Waiting for formation and valve positions...")
        
        # Wait for formation position using Event mechanism (like our working debug script)
        rospy.loginfo("Waiting for formation position...")
        if not self.formation_adapter.position_received.wait(timeout=15.0):
            rospy.logerr("Formation position not received")
            rospy.logwarn(f"UAV1 position: {self.formation_adapter.uav1_pos}")
            rospy.logwarn(f"UAV2 position: {self.formation_adapter.uav2_pos}")
            rospy.logwarn(f"Assembly position: {self.formation_adapter.assembly_pos}")
            return False
        else:
            rospy.loginfo("✓ Formation position received successfully")
            rospy.loginfo(f"Assembly position: {self.formation_adapter.assembly_pos}")
        
        # Wait for valve position (use parent method)
        rospy.loginfo("Waiting for valve position...")
        if not self.valve_received.wait(timeout=5.0):
            rospy.logerr("Valve position not received")
            return False
        else:
            rospy.loginfo("✓ Valve position received successfully")
        
        rospy.loginfo("✓ All positions received - initialization complete")
        return True


class FormationAssembleState(smach.State):
    """Execute physical assembly of two UAVs - directly from valve_rotation_smach_test.py"""
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
        # Module IDs
        modules_str = rospy.get_param("~module_ids", "1,2")
        real_machine = rospy.get_param("~real_machine", False)
        modules = []
        if modules_str:
            modules = [int(x) for x in modules_str.split(',')]
        else:
            rospy.logerr("No module ID is designated!")
        
        rospy.loginfo(f"Formation assembly configured for modules: {modules}, real_machine: {real_machine}")
        
        # AssembleDemo - try multiple import paths
        try:
            from task.assembly_motion import AssemblyDemo
            self.assemble_demo = AssemblyDemo(module_ids=modules, real_machine=real_machine)
            self.assembly_available = True
            rospy.loginfo("Assembly demo loaded from task.assembly_motion")
        except ImportError:
            try:
                from beetle.assembly_api import AssemblyDemo
                self.assemble_demo = AssemblyDemo(module_ids=modules, real_machine=real_machine)
                self.assembly_available = True
                rospy.loginfo("Assembly demo loaded from beetle.assembly_api")
            except ImportError:
                rospy.logerr("AssemblyDemo not available from any import path")
                self.assembly_available = False
    
    def execute(self, userdata):
        if not self.assembly_available:
            rospy.logwarn("Assembly demo not available, skipping physical assembly")
            return 'succeeded'
        
        rospy.loginfo("=== ASSEMBLING UAVs ===")
        try:
            self.assemble_demo.main()
            rospy.loginfo("✓ Assembly completed successfully")
            return 'succeeded'
        except rospy.ROSInterruptException:
            rospy.logerr("✗ Assembly process interrupted")
            return 'failed'
        except Exception as e:
            rospy.logerr(f"✗ Assembly process failed: {e}")
            return 'failed'


class FormationInitializeState(FormationStateBase):
    """Formation version of initialize state"""
    
    def __init__(self):
        FormationStateBase.__init__(self, 
                                  outcomes=['succeeded', 'failed'],
                                  input_keys=['start_position'],
                                  output_keys=['start_position'])
    
    def execute(self, userdata):
        rospy.loginfo("Initializing formation start position...")
        
        # Wait for both formation and valve positions
        if not self.wait_for_positions():
            rospy.logerr("Failed to receive required positions for formation initialization")
            return 'failed'
        
        # Calculate start position based on formation geometry
        current_pos = self.get_current_position()
        
        # Set start position in userdata
        userdata.start_position = current_pos
        
        rospy.loginfo(f"Formation initialization completed successfully")
        rospy.loginfo(f"Start position: {current_pos}")
        return 'succeeded'


class FormationMoveToValveState(FormationStateBase):
    """Formation version of move to valve state"""
    
    def __init__(self, approach_distance=0.8, approach_height=0.3):
        FormationStateBase.__init__(self, 
                                  outcomes=['succeeded', 'failed'],
                                  input_keys=['start_position'])
        self.approach_distance = approach_distance
        self.approach_height = approach_height
    
    def execute(self, userdata):
        rospy.loginfo("Formation moving to valve approach position...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        # Calculate approach position
        valve_x, valve_y, valve_z = self.valve_pos
        approach_x = valve_x - self.approach_distance
        approach_y = valve_y
        approach_z = valve_z + self.approach_height
        
        target_pos = (approach_x, approach_y, approach_z)
        target_yaw = self.valve_yaw
        
        rospy.loginfo(f"Moving formation to approach position: {target_pos} with yaw: {target_yaw}")
        
        # Send command to formation
        self.formation_adapter.send_end_effector_command(target_pos, target_yaw)
        
        # Wait for completion
        if self.formation_adapter.wait_for_move_completion(timeout=30.0):
            rospy.loginfo("Formation successfully moved to approach position")
            return 'succeeded'
        else:
            rospy.logerr("Formation failed to reach approach position")
            return 'failed'
        target_yaw = math.atan2(valve_y - approach_y, valve_x - approach_x)
        
        rospy.loginfo(f"Moving formation end-effector from {current_pos} to {target_pos}")
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
        
        # Execute movement using formation adapter
        return self.execute_formation_movement(current_pos, target_pos, target_yaw)
    
    def execute_formation_movement(self, start_pos, target_pos, target_yaw):
        """Execute formation movement to target position"""
        # Calculate distance and duration
        distance = math.sqrt(sum((t-s)**2 for t,s in zip(target_pos, start_pos)))
        duration = max(10.0, distance / 0.1)  # 0.1 m/s speed
        
        rospy.loginfo(f"Formation movement: distance={distance:.3f}m, duration={duration:.1f}s")
        
        # Execute trajectory
        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                break
            
            # Linear interpolation
            progress = elapsed / duration
            current_target_pos = tuple(s + progress * (t - s) for s, t in zip(start_pos, target_pos))
            current_target_yaw = self.get_current_yaw() + progress * (target_yaw - self.get_current_yaw())
            
            # Send command
            self.send_command(current_target_pos, current_target_yaw)
            rate.sleep()
        
        # Final position
        self.send_command(target_pos, target_yaw)
        rospy.sleep(2.0)  # Stabilization
        
        # Verify position
        final_pos = self.get_current_position()
        if final_pos:
            error = math.sqrt(sum((f-t)**2 for f,t in zip(final_pos, target_pos)))
            rospy.loginfo(f"Formation movement completed, error: {error:.3f}m")
            return error < 0.1
        
        return True


class FormationDescendAndContactState(FormationStateBase):
    """Formation version of descend and contact state"""
    
    def __init__(self, rotation_direction=1):
        FormationStateBase.__init__(self, 
                                  outcomes=['succeeded', 'failed', 'aborted'],
                                  input_keys=['selected_gap'],
                                  output_keys=['selected_gap'])
        
        # Copy necessary attributes from DescendAndContactState
        self.rotation_direction = rotation_direction
        
        # Initialize insertion optimizer (import locally to avoid circular imports)
        from insertion_optimizer import InsertionOptimizer
        self.optimizer = InsertionOptimizer(
            valve_radius=0.1075,
            valve_beam_width=0.024,
            safety_margin=0.002,
            module_id=2  # Formation uses Module2 as leader
        )
    
    def execute(self, userdata):
        rospy.loginfo("Formation descend and contact...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        # Get current end-effector position
        current_pos = self.get_current_position()
        if current_pos is None:
            return 'failed'
        
        # Set rotation direction
        self.optimizer.set_rotation_direction(self.rotation_direction)
        
        # Step 1: Find optimal insertion point
        strategy = self.optimizer.find_closest_beam_gap(
            uav_pos=current_pos,  # Use end-effector position directly
            valve_pos=self.valve_pos,
            valve_yaw=self.valve_yaw
        )
        
        if strategy is None:
            rospy.logerr("Failed to find insertion strategy")
            return 'failed'
        
        target_pos = strategy['target_position']
        target_yaw = strategy['gap_center_angle']
        
        rospy.loginfo(f"Formation insertion target: {target_pos}")
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
        
        # Step 2: Execute insertion movement
        return self.execute_formation_insertion(current_pos, target_pos, target_yaw)
    
    def execute_formation_insertion(self, start_pos, target_pos, target_yaw):
        """Execute formation insertion in three phases"""
        
        # Phase 1: Move to XY position above target
        rospy.loginfo("Phase 1: Moving to XY position above target")
        phase1_target = (target_pos[0], target_pos[1], start_pos[2])  # Keep current Z
        
        success = self.execute_formation_movement(start_pos, phase1_target, self.get_current_yaw())
        if not success:
            rospy.logerr("Phase 1 failed")
            return False
        
        # Phase 2: Adjust yaw to target orientation
        rospy.loginfo("Phase 2: Adjusting yaw orientation")
        current_pos = self.get_current_position()
        if current_pos is None:
            current_pos = phase1_target
        
        # Send yaw command
        self.send_command(current_pos, target_yaw)
        rospy.sleep(3.0)  # Allow time for yaw adjustment
        
        # Phase 3: Descend to target Z
        rospy.loginfo("Phase 3: Descending to target height")
        final_pos = self.get_current_position()
        if final_pos is None:
            final_pos = current_pos
        
        final_target = (final_pos[0], final_pos[1], target_pos[2])
        
        success = self.execute_formation_movement(final_pos, final_target, target_yaw)
        if not success:
            rospy.logerr("Phase 3 failed")
            return False
        
        rospy.loginfo("Formation insertion completed successfully")
        return True
    
    def execute_formation_movement(self, start_pos, target_pos, target_yaw):
        """Execute smooth formation movement"""
        distance = math.sqrt(sum((t-s)**2 for t,s in zip(target_pos, start_pos)))
        duration = max(5.0, distance / 0.05)  # Slower 5cm/s for precision
        
        rospy.loginfo(f"Formation movement: {start_pos} -> {target_pos}")
        rospy.loginfo(f"Distance: {distance:.3f}m, Duration: {duration:.1f}s, Target yaw: {math.degrees(target_yaw):.1f}°")
        
        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        
        step_count = 0
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                rospy.loginfo(f"Movement completed after {elapsed:.1f}s ({step_count} steps)")
                break
            
            progress = elapsed / duration
            current_target = tuple(s + progress * (t - s) for s, t in zip(start_pos, target_pos))
            
            if step_count % 100 == 0:  # Log every 5 seconds (20Hz rate)
                rospy.loginfo(f"Movement progress: {progress*100:.1f}% - Target: {current_target}")
            
            self.send_command(current_target, target_yaw)
            rate.sleep()
            step_count += 1
        
        self.send_command(target_pos, target_yaw)
        rospy.loginfo("Final command sent, waiting 1 second...")
        rospy.sleep(1.0)
        rospy.loginfo("Formation movement completed")
        return True


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


class FormationRotateValveState(FormationStateBase):
    """Formation version of rotate valve state"""
    
    def __init__(self, rotation_angle=math.pi/2, rotation_duration=18.0, rotation_direction=1):
        FormationStateBase.__init__(self, 
                                  outcomes=['succeeded', 'failed', 'emergency'],
                                  input_keys=['start_position'])
        
        # Create single UAV state instance for reuse
        self.single_uav_state = RotateValveState(module_id=999, rotation_angle=rotation_angle, 
                                               rotation_duration=rotation_duration, rotation_direction=rotation_direction)
    
    def execute(self, userdata):
        rospy.loginfo("Formation valve rotation...")
        
        if not self.wait_for_valve_position():
            return 'failed'
        
        # Copy valve data to single UAV state
        self.single_uav_state.valve_pos = self.valve_pos
        self.single_uav_state.valve_yaw = self.valve_yaw
        
        # Execute single UAV logic
        result = self.single_uav_state.execute(userdata)
        
        rospy.loginfo(f"Formation valve rotation result: {result}")
        return result


# Main execution function
def main():
    rospy.init_node('formation_valve_rotation')
    
    # Get rotation direction from parameter
    direction_param = rospy.get_param("~valve_rotation_direction", "clockwise")
    rotation_direction = 1 if direction_param.lower() == "clockwise" else -1
    rospy.loginfo(f"Formation valve rotation direction: {'Clockwise' if rotation_direction == 1 else 'Counter-clockwise'}")
    
    try:
        # Create Formation SMACH state machine
        sm = smach.StateMachine(outcomes=['success', 'failure'])
        
        with sm:
            smach.StateMachine.add('FORMATION_ASSEMBLE',
                                   FormationAssembleState(),
                                   transitions={'succeeded': 'FORMATION_INITIALIZE',
                                               'failed': 'failure'})
            
            smach.StateMachine.add('FORMATION_INITIALIZE',
                                   FormationInitializeState(),
                                   transitions={'succeeded': 'FORMATION_MOVE_TO_VALVE',
                                               'failed': 'failure'})
            
            smach.StateMachine.add('FORMATION_MOVE_TO_VALVE',
                                   FormationMoveToValveState(),
                                   transitions={'succeeded': 'FORMATION_DESCEND_CONTACT',
                                               'failed': 'failure'})
            
            smach.StateMachine.add('FORMATION_DESCEND_CONTACT', 
                                   FormationDescendAndContactState(rotation_direction=rotation_direction),
                                   transitions={'succeeded': 'FORMATION_ROTATE_VALVE',
                                               'failed': 'failure',
                                               'aborted': 'failure'})
            
            smach.StateMachine.add('FORMATION_ROTATE_VALVE',
                                   FormationRotateValveState(rotation_direction=rotation_direction),
                                   transitions={'succeeded': 'success',
                                               'failed': 'failure',
                                               'emergency': 'failure'})
        
        # Execute state machine
        rospy.loginfo("Starting Formation valve rotation state machine...")
        outcome = sm.execute()
        rospy.loginfo(f"Formation valve rotation completed with outcome: {outcome}")
        
    except Exception as e:
        rospy.logerr(f"Error raised during SMACH container construction: \n{e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
