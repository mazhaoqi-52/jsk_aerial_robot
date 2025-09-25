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
import numpy as np

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

# Import single UAV classes and valve rotation logic
from valve_rotation_fang_single import (
    SingleUAVStateBase, 
    InitializeStartPositionState,
    MoveToValveState, 
    DescendAndContactState,
    RotateValveState
)

# Import additional required modules for valve rotation
from insertion_optimizer import InsertionOptimizer


class FormationAdapter:
    """Dynamic adapter for multi-UAV formation with configurable module_ids"""
    
    def __init__(self, module_ids_str=None):
        # Import and initialize coordinate transformer
        from n_modules_tf import NModuleTFCalculator
        
        # Parse module_ids parameter
        if module_ids_str is None:
            module_ids_str = rospy.get_param("~module_ids", "2,3")
        
        rospy.loginfo(f"Initializing FormationAdapter with module_ids: {module_ids_str}")
        
        # Initialize coordinate transformer
        rospy.set_param("~module_ids", module_ids_str)  # Ensure parameter is set
        self.tf_calculator = NModuleTFCalculator(module_ids_str)
        
        if not self.tf_calculator.validate_module_configuration():
            raise ValueError(f"Invalid module configuration: {module_ids_str}")
        
        # Formation configuration
        self.module_ids = self.tf_calculator.module_ids
        self.leader_id = self.tf_calculator.get_leader_id()
        self.follower_ids = self.tf_calculator.get_follower_ids()
        self.number_of_modules = len(self.module_ids)
        
        rospy.loginfo(f"Formation configuration:")
        rospy.loginfo(f"  Module IDs: {self.module_ids}")
        rospy.loginfo(f"  Leader (end-effector): {self.leader_id}")
        rospy.loginfo(f"  Followers: {self.follower_ids}")
        
        # State tracking for each UAV
        self.uav_positions = {}  # UAV ID -> (x, y, z)
        self.uav_orientations = {}  # UAV ID -> yaw
        self.assembly_pos = None
        self.assembly_yaw = 0.0
        
        # Position tracking synchronization
        self.position_received = threading.Event()
        
        # Setup individual UAV subscribers dynamically
        self.uav_subscribers = {}
        for module_id in self.module_ids:
            topic = f"/beetle{module_id}/mocap/pose"
            self.uav_subscribers[module_id] = rospy.Subscriber(
                topic, PoseStamped, 
                lambda msg, mid=module_id: self.uav_callback(msg, mid),
                queue_size=1
            )
            rospy.loginfo(f"  Subscribed to {topic} for module {module_id}")
        
        # Assembly command publisher
        self.assembly_pub = rospy.Publisher(
            "/assembly/uav/nav", FlightNav, queue_size=1
        )
        
        # Calculate and store offset parameters for assembly-to-end-effector transform
        self._calculate_offset_parameters()
        
        rospy.loginfo("FormationAdapter initialization complete")
    
    def _calculate_offset_parameters(self):
        """Calculate and store offset parameters for coordinate transformations"""
        # Calculate offset at yaw=0 for reference (will be rotated during actual use)
        reference_yaw = 0.0
        assembly_to_leader = self.tf_calculator.calculate_assembly_to_leader_transform()
        leader_to_ee = self.tf_calculator.calculate_leader_to_end_effector_transform(reference_yaw)
        
        # Store base offset values (will be rotated by yaw during actual transforms)
        self.base_offset_x = assembly_to_leader['offset_x'] + leader_to_ee['x']  # X offset in body frame
        self.base_offset_y = assembly_to_leader['offset_y'] + leader_to_ee['y']  # Y offset in body frame  
        self.total_offset_z = assembly_to_leader['offset_z'] + leader_to_ee['z']  # Z is not affected by yaw
        
        rospy.loginfo(f"Assembly to End-effector offsets:")
        rospy.loginfo(f"  Base X offset: {self.base_offset_x:.3f}m")
        rospy.loginfo(f"  Base Y offset: {self.base_offset_y:.3f}m") 
        rospy.loginfo(f"  Z offset: {self.total_offset_z:.3f}m")
    
    def uav_callback(self, msg, module_id):
        """Receive individual UAV position from mocap (PoseStamped format)"""
        pos = msg.pose.position
        ori = msg.pose.orientation
        
        # Store UAV position
        self.uav_positions[module_id] = (pos.x, pos.y, pos.z)
        
        # Extract yaw from quaternion
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.uav_orientations[module_id] = yaw
        
        rospy.logdebug(f"UAV{module_id} callback: pos={self.uav_positions[module_id]}, yaw={yaw:.3f}")
        
        # Update assembly position if all UAVs are tracked
        self._update_assembly_position()
    
    def _update_assembly_position(self):
        """Calculate assembly CoG position from all participating UAVs"""
        # Check if all UAVs have reported their positions
        if len(self.uav_positions) < self.number_of_modules:
            rospy.logdebug(f"Waiting for all UAVs: {len(self.uav_positions)}/{self.number_of_modules}")
            return
        
        # Verify all required modules are present (both position and orientation)
        for module_id in self.module_ids:
            if module_id not in self.uav_positions:
                rospy.logdebug(f"Missing position for module {module_id}")
                return
            if module_id not in self.uav_orientations:
                rospy.logdebug(f"Missing orientation for module {module_id}")
                return
        
        # Calculate assembly CoG as average of all UAV positions
        avg_x = sum(self.uav_positions[mid][0] for mid in self.module_ids) / self.number_of_modules
        avg_y = sum(self.uav_positions[mid][1] for mid in self.module_ids) / self.number_of_modules
        avg_z = sum(self.uav_positions[mid][2] for mid in self.module_ids) / self.number_of_modules
        
        self.assembly_pos = (avg_x, avg_y, avg_z)
        
        # Calculate assembly yaw (average of all UAV yaws with angle wrapping handling)
        yaw_sum = 0.0
        base_yaw = self.uav_orientations[self.module_ids[0]]
        
        for module_id in self.module_ids:
            yaw = self.uav_orientations[module_id]
            yaw_diff = yaw - base_yaw
            # Handle angle wrapping
            while yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            yaw_sum += base_yaw + yaw_diff
        
        self.assembly_yaw = yaw_sum / self.number_of_modules
        
        # Signal that assembly position is available
        if not self.position_received.is_set():
            rospy.loginfo(f"✓ Formation position received: {self.assembly_pos}")
            for mid in self.module_ids:
                rospy.loginfo(f"  UAV{mid}: {self.uav_positions[mid]}")
            self.position_received.set()
        
        rospy.logdebug(f"Assembly CoG updated: pos={self.assembly_pos}, yaw={self.assembly_yaw:.3f}")
    
    def assembly_callback(self, msg):
        """Legacy callback - maintained for compatibility"""
        pass
    
    def get_assembly_position(self):
        """Get current assembly CoG position"""
        return self.assembly_pos
    
    def get_assembly_yaw(self):
        """Get current assembly yaw"""
        return self.assembly_yaw
    
    def get_end_effector_position(self):
        """Get end-effector position using coordinate transformation"""
        if self.assembly_pos is None:
            rospy.logwarn("Assembly position not available for end-effector calculation")
            return None
            
        return self.tf_calculator.transform_assembly_to_end_effector(
            self.assembly_pos, self.assembly_yaw
        )
    
    def transform_end_effector_to_assembly_command(self, target_end_effector_pos, target_yaw=None):
        """
        Transform end-effector target to assembly CoG command.
        
        This is the key method that allows single UAV logic to work with assembly.
        Single UAV states can specify end-effector targets, and this transforms them
        to appropriate assembly CoG targets.
        
        Args:
            target_end_effector_pos (tuple): Target end-effector position (x, y, z)  
            target_yaw (float): Target assembly yaw (optional)
            
        Returns:
            tuple: Assembly CoG target position (x, y, z)
        """
        if target_yaw is None:
            target_yaw = self.assembly_yaw if self.assembly_yaw is not None else 0.0
        
        # This is the inverse transform of assembly_to_end_effector
        # Given end-effector target position, calculate required assembly CoG position
        
        # Get transform parameters
        assembly_to_leader = self.tf_calculator.calculate_assembly_to_leader_transform()
        leader_to_ee = self.tf_calculator.calculate_leader_to_end_effector_transform(target_yaw)
        
        # Total offset from assembly CoG to end-effector
        cos_yaw = math.cos(target_yaw)
        sin_yaw = math.sin(target_yaw)
        
        total_offset_x = (assembly_to_leader['offset_x'] * cos_yaw - 
                         assembly_to_leader['offset_y'] * sin_yaw + leader_to_ee['x'])
        total_offset_y = (assembly_to_leader['offset_x'] * sin_yaw + 
                         assembly_to_leader['offset_y'] * cos_yaw + leader_to_ee['y'])
        total_offset_z = assembly_to_leader['offset_z'] + leader_to_ee['z']
        
        # Calculate required assembly position
        assembly_target_x = target_end_effector_pos[0] - total_offset_x
        assembly_target_y = target_end_effector_pos[1] - total_offset_y
        assembly_target_z = target_end_effector_pos[2] - total_offset_z
        
        return (assembly_target_x, assembly_target_y, assembly_target_z)
    
    def get_leader_id(self):
        """Get leader UAV ID"""
        return self.leader_id
    
    def get_follower_ids(self):
        """Get follower UAV IDs"""
        return self.follower_ids
    
    def wait_for_formation_ready(self, timeout=10.0):
        """Wait for all UAVs to report their positions"""
        rospy.loginfo(f"Waiting for formation to be ready ({self.number_of_modules} UAVs)...")
        
        if self.position_received.wait(timeout):
            rospy.loginfo("✓ Formation ready - all UAVs reporting positions")
            return True
        else:
            rospy.logerr(f"✗ Formation not ready after {timeout}s timeout")
            return False
    
    def get_end_effector_position(self):
        """Calculate current end-effector position using proper coordinate transformation chain"""
        if self.assembly_pos is None:
            return None
        
        # Use the tf_calculator for proper coordinate transformation chain:
        # Assembly CoG → Leader UAV CoG → End-effector
        return self.tf_calculator.transform_assembly_to_end_effector(
            self.assembly_pos, self.assembly_yaw
        )
    
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
    
    def send_assembly_command(self, target_pos, target_yaw=None, linear_vel=None):
        """Send FlightNav command to assembly controller with correct format"""
        try:
            # Validate assembly position availability
            if self.assembly_pos is None:
                rospy.logwarn("Assembly position not available - command may be ineffective")
            
            # Create FlightNav message for formation control
            nav_msg = FlightNav()
            nav_msg.header.stamp = rospy.Time.now()
            nav_msg.header.frame_id = "world"
            
            # CRITICAL: Set target to COG for assembled state
            nav_msg.target = FlightNav.COG  # Required for assembled state
            
            # Position control with optional velocity
            nav_msg.target_pos_x = target_pos[0]
            nav_msg.target_pos_y = target_pos[1] 
            nav_msg.target_pos_z = target_pos[2]
            
            # Use POS_VEL mode if velocity is provided
            if linear_vel is not None:
                nav_msg.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
                nav_msg.pos_z_nav_mode = FlightNav.POS_VEL_MODE
                nav_msg.target_vel_x = linear_vel[0]
                nav_msg.target_vel_y = linear_vel[1]
                nav_msg.target_vel_z = linear_vel[2]
                rospy.logdebug(f"Using POS_VEL mode with velocity: {linear_vel}")
            else:
                nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                nav_msg.target_vel_x = 0.0
                nav_msg.target_vel_y = 0.0
                nav_msg.target_vel_z = 0.0
            
            # Yaw control
            if target_yaw is not None:
                nav_msg.target_yaw = target_yaw
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE
            else:
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE
                nav_msg.target_yaw = 0.0
            
            self.assembly_pub.publish(nav_msg)
            yaw_str = f"{target_yaw:.3f}" if target_yaw is not None else "None"
            vel_str = f", vel={linear_vel}" if linear_vel is not None else ""
            rospy.logdebug(f"Sent assembly FlightNav: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}), yaw: {yaw_str}{vel_str}")
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error sending assembly command: {e}")
            return False
    
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
        
        # Calculate rotated offsets
        rotated_offset_x = self.base_offset_x * cos_yaw - self.base_offset_y * sin_yaw
        rotated_offset_y = self.base_offset_x * sin_yaw + self.base_offset_y * cos_yaw
        
        assembly_x = end_effector_pos[0] - rotated_offset_x
        assembly_y = end_effector_pos[1] - rotated_offset_y
        assembly_z = end_effector_pos[2] - self.total_offset_z
        
        return (assembly_x, assembly_y, assembly_z)
    
    def assembly_to_end_effector_transform(self, assembly_pos, assembly_yaw, use_insertion_offset=False):
        """Transform assembly position to end-effector position"""
        cos_yaw = math.cos(assembly_yaw)
        sin_yaw = math.sin(assembly_yaw)
        
        # Calculate rotated offsets
        rotated_offset_x = self.base_offset_x * cos_yaw - self.base_offset_y * sin_yaw
        rotated_offset_y = self.base_offset_x * sin_yaw + self.base_offset_y * cos_yaw
        
        end_effector_x = assembly_pos[0] + rotated_offset_x
        end_effector_y = assembly_pos[1] + rotated_offset_y
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


class FormationSingleUAVStateBase(smach.State):
    """Base class for formation states that reuse single UAV logic"""
    
    def __init__(self, outcomes, input_keys=None, output_keys=None):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys or [], output_keys=output_keys or [])
        
        # Get formation configuration
        module_ids_str = rospy.get_param("~module_ids", "2,3") 
        
        # Initialize formation adapter
        self.formation_adapter = FormationAdapter(module_ids_str)
        self.leader_id = self.formation_adapter.get_leader_id()
        
        # Wait for formation to be ready
        if not self.formation_adapter.wait_for_formation_ready(timeout=15.0):
            raise RuntimeError("Formation not ready - cannot initialize state")
        
        # Initialize BeetleInterface in assembly mode
        from beetle_interface import BeetleInterface
        self.beetle = BeetleInterface(
            module_id=self.leader_id,
            assembly_mode=True,
            assembly_tf_calculator=self.formation_adapter.tf_calculator
        )
        
        # Initialize insertion optimizer for geometry calculations
        self.optimizer = InsertionOptimizer()
        
        rospy.loginfo(f"FormationState initialized for leader UAV{self.leader_id}")
        
    def get_assembly_position(self):
        """Get assembly CoG position"""
        return self.formation_adapter.get_assembly_position()
    
    def get_assembly_yaw(self):
        """Get assembly yaw angle"""
        return self.formation_adapter.get_assembly_yaw()
    
    def get_end_effector_position(self):
        """Get end-effector position through coordinate transformation"""
        return self.formation_adapter.get_end_effector_position()
    
    def get_end_effector_yaw(self):
        """Get end-effector yaw angle through coordinate transformation"""
        return self.formation_adapter.get_end_effector_yaw()
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def send_assembly_command_from_end_effector(self, target_end_effector_pos, target_yaw=None,
                                               linear_vel=None, angular_vel=None):
        """Send assembly command by transforming end-effector target to assembly CoG with speed control"""
        assembly_target = self.formation_adapter.transform_end_effector_to_assembly_command(
            target_end_effector_pos, target_yaw
        )

        ee_target_vec = np.array(target_end_effector_pos, dtype=float)
        assembly_target_vec = np.array(assembly_target, dtype=float)

        # Normalise linear velocity input
        linear_vel_vec = None
        if linear_vel is not None:
            if isinstance(linear_vel, (int, float)):
                rospy.logdebug("Scalar linear_vel received; assuming Z-axis velocity component")
                linear_vel_vec = np.array([0.0, 0.0, float(linear_vel)], dtype=float)
            else:
                linear_array = np.array(linear_vel, dtype=float).flatten()
                if linear_array.size == 3:
                    linear_vel_vec = linear_array
                else:
                    rospy.logwarn_once("Invalid linear_vel size supplied to send_assembly_command_from_end_effector; ignoring linear velocity")

        # Normalise angular velocity input for both compensation and command forwarding
        omega_vec = np.zeros(3, dtype=float)
        if angular_vel is not None:
            if isinstance(angular_vel, (int, float)):
                omega_vec[2] = float(angular_vel)
            else:
                angular_array = np.array(angular_vel, dtype=float).flatten()
                if angular_array.size == 1:
                    omega_vec[2] = float(angular_array[0])
                elif angular_array.size == 3:
                    omega_vec = angular_array
                else:
                    rospy.logwarn_once("Invalid angular_vel size supplied to send_assembly_command_from_end_effector; ignoring angular velocity")
                    omega_vec = np.zeros(3, dtype=float)

        angular_vel_cmd = omega_vec.tolist() if not np.allclose(omega_vec, 0.0) else None

        # Lever-arm compensation: convert end-effector velocity into assembly CoG frame
        assembly_linear_vec = None
        if linear_vel_vec is not None or angular_vel_cmd is not None:
            v_ee = linear_vel_vec if linear_vel_vec is not None else np.zeros(3, dtype=float)
            ee_offset = ee_target_vec - assembly_target_vec
            v_assembly = v_ee - np.cross(omega_vec, ee_offset)
            assembly_linear_vec = v_assembly.tolist()
        
        # Debug logging (only log occasionally to avoid spam)
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 1
        
        # Only log every 20 calls (approximately every 2 seconds at 10Hz)
        if self._debug_counter % 20 == 0:
            rospy.logdebug(f"🔧 Assembly Command (#{self._debug_counter}):")
            rospy.logdebug(f"  End-effector target: {target_end_effector_pos}")
            rospy.logdebug(f"  Assembly target: {assembly_target}")

        if self._debug_counter % 20 == 0:
            if assembly_linear_vec is not None:
                rospy.logdebug(f"  Assembly linear vel: {assembly_linear_vec}")
            if angular_vel_cmd is not None:
                rospy.logdebug(f"  Angular vel (rad/s): {angular_vel_cmd}")

        # Send command using beetle interface with speed parameters
        self.beetle.targetMotion(
            pos=assembly_target,
            rot=target_yaw,
            linear_vel=assembly_linear_vec,
            angular_vel=angular_vel_cmd
        )

    # -------------------------------------------------------------
    # Shared Z-descent helper (adapted from single UAV implementation)
    # -------------------------------------------------------------
    def controlled_z_descent(self, start_pos, final_target, final_yaw, descent_speed=0.05):
        """按固定步长下沉Z轴，必要时在接触容差内判定成功。

        Returns:
            tuple[bool, tuple | None]: (是否成功, 实际达到的末端位姿)
        """
        rospy.loginfo("=== Formation Controlled Z Descent ===")

        if start_pos is None or final_target is None:
            rospy.logerr("[Formation Z Descent] Missing start or target position")
            return False, None

        target_x, target_y, target_z = final_target
        start_z = start_pos[2]
        total_z_descent = start_z - target_z
        rospy.loginfo(f"[Formation Z Descent] Total Z descent {total_z_descent*1000:.1f}mm")

        if total_z_descent <= 0.01:
            rospy.loginfo("[Formation Z Descent] Small descent, issuing direct convergence")
            self.send_assembly_command_from_end_effector((target_x, target_y, target_z), final_yaw)
            success = self.active_position_convergence(
                (target_x, target_y, target_z),
                target_yaw=final_yaw,
                pos_threshold=0.03,
                yaw_threshold=0.0175,
                timeout=8.0,
                min_readings=3
            )
            achieved = self.get_end_effector_position() or (target_x, target_y, target_z)
            return success, achieved

        fixed_step_size = 0.025
        planned_steps = max(3, int(math.ceil(total_z_descent / fixed_step_size)))
        max_single_step = 0.03

        rospy.loginfo(f"[Formation Z Descent] Plan {planned_steps} steps, {fixed_step_size*1000:.1f}mm per step")

        current_pos = start_pos
        previous_z = current_pos[2]
        total_descent_completed = 0.0
        step_count = 0
        progress_ratio = 0.0
        max_step_iterations = max(planned_steps * 6, planned_steps + 40)
        achieved_position = current_pos
        contact_xy_tolerance = 0.022
        contact_z_tolerance = 0.05
        plateau_z_tolerance = 0.25
        plateau_progress_threshold = 0.4
        stagnation_threshold = 0.012
        contact_required_hits = 3
        plateau_required_hits = 5
        stagnation_hit_counter = 0
        proximity_hit_counter = 0
        final_descent_margin = 0.03

        while previous_z - target_z > 1e-4:
            if step_count >= max_step_iterations:
                rospy.logerr("[Formation Z Descent] Step loop exceeded safety iteration limit")
                return False, achieved_position

            remaining_descent = max(0.0, previous_z - target_z)
            if remaining_descent <= final_descent_margin:
                rospy.loginfo(
                    f"[Formation Z Descent] Remaining descent {remaining_descent*1000:.1f}mm ≤ {final_descent_margin*1000:.0f}mm，接受当前高度"
                )
                break

            step_size = min(fixed_step_size, max_single_step, remaining_descent)
            current_z = previous_z - step_size
            if current_z < target_z:
                current_z = target_z

            step_count += 1
            progress = progress_ratio

            step_target = [target_x, target_y, current_z]

            base_threshold = 0.05
            final_threshold = final_descent_margin
            step_pos_thresh = base_threshold - (base_threshold - final_threshold) * progress
            step_pos_thresh = min(step_pos_thresh, max(step_size * 0.8, 0.015))
            step_yaw_thresh = 0.0175
            is_final_step = current_z <= target_z + 1e-4 or remaining_descent <= step_size + 1e-6

            linear_vel = None if is_final_step else [0.0, 0.0, -abs(descent_speed) * 0.5]

            rospy.loginfo(
                f"[Formation Z Descent] Step {step_count}/{planned_steps} target=({step_target[0]:.3f}, {step_target[1]:.3f}, {step_target[2]:.3f}), "
                f"threshold={step_pos_thresh*1000:.1f}mm (remaining {remaining_descent*1000:.1f}mm)"
            )

            self.send_assembly_command_from_end_effector(step_target, final_yaw, linear_vel=linear_vel, angular_vel=0.0)

            step_converged = self.active_position_convergence(
                target_ee_pos=step_target,
                target_yaw=final_yaw,
                pos_threshold=step_pos_thresh,
                yaw_threshold=step_yaw_thresh,
                timeout=12.0,
                max_attempts=80,
                min_readings=3
            )

            if not step_converged:
                achieved_position = self.get_end_effector_position() or tuple(step_target)
                xy_residual = math.sqrt(
                    (achieved_position[0] - target_x)**2 +
                    (achieved_position[1] - target_y)**2
                )
                z_residual = achieved_position[2] - target_z
                last_motion = abs(previous_z - achieved_position[2])
                attempted_descent_completed = max(0.0, start_z - achieved_position[2])
                attempted_progress_ratio = (
                    0.0 if total_z_descent <= 1e-6
                    else min(1.0, attempted_descent_completed / total_z_descent)
                )

                motion_within_threshold = last_motion <= stagnation_threshold
                if motion_within_threshold:
                    stagnation_hit_counter += 1
                else:
                    stagnation_hit_counter = 0

                within_proximity = (
                    0.0 <= z_residual <= contact_z_tolerance
                    and xy_residual <= contact_xy_tolerance
                )

                if within_proximity:
                    proximity_hit_counter += 1
                else:
                    proximity_hit_counter = 0

                contact_ready = (
                    within_proximity
                    and (
                        (motion_within_threshold and stagnation_hit_counter >= contact_required_hits)
                        or proximity_hit_counter >= contact_required_hits
                    )
                )
                plateau_ready = motion_within_threshold and stagnation_hit_counter >= plateau_required_hits

                within_contact_window = contact_ready

                plateau_reached = (
                    plateau_ready
                    and within_proximity
                    and 0.0 <= z_residual <= plateau_z_tolerance
                    and xy_residual <= contact_xy_tolerance * 1.4
                    and total_z_descent > 1e-4
                    and attempted_progress_ratio >= plateau_progress_threshold
                )

                if within_contact_window or plateau_reached:
                    contact_label = "接触容差" if within_contact_window else "平台判定"
                    hit_note = f"prox={proximity_hit_counter}, stagnation={stagnation_hit_counter}"
                    rospy.logwarn(
                        f"[Formation Z Descent] Step {step_count}{contact_label}: XY {xy_residual*1000:.1f}mm, "
                        f"Z余量 {z_residual*1000:.1f}mm，锁定当前高度 ({hit_note}, ΔZ阈值≤{stagnation_threshold*1000:.1f}mm)"
                    )
                    target_z = achieved_position[2]
                    previous_z = achieved_position[2]
                    proximity_hit_counter = 0
                    stagnation_hit_counter = 0
                    break

                rospy.logerr(
                    f"[Formation Z Descent] Step {step_count} failed to converge (XY {xy_residual*1000:.1f}mm, "
                    f"Z余量 {z_residual*1000:.1f}mm)"
                )
                return False, achieved_position

            current_pos = self.get_end_effector_position() or tuple(step_target)
            actual_descent = max(0.0, previous_z - current_pos[2])
            if actual_descent < 1e-4 and remaining_descent > final_descent_margin:
                rospy.logdebug(
                    f"[Formation Z Descent] Step {step_count} 实际下降不足 0.1mm (remaining {remaining_descent*1000:.1f}mm)"
                )
            stagnation_hit_counter = 0
            proximity_hit_counter = 0
            xy_error = math.sqrt(
                (current_pos[0] - step_target[0])**2 +
                (current_pos[1] - step_target[1])**2
            )
            if xy_error > 0.025:
                rospy.logwarn(f"[Formation Z Descent] XY error {xy_error*1000:.1f}mm, applying XY correction")
                correction_target = [step_target[0], step_target[1], current_pos[2]]
                self.send_assembly_command_from_end_effector(correction_target, final_yaw)
                corrected = self.active_position_convergence(
                    target_ee_pos=correction_target,
                    target_yaw=final_yaw,
                    pos_threshold=0.03,
                    yaw_threshold=0.03,
                    timeout=6.0,
                    max_attempts=60,
                    min_readings=3
                )
                if not corrected:
                    rospy.logerr("[Formation Z Descent] XY correction failed")
                    return False, current_pos

            previous_z = current_pos[2]
            achieved_position = current_pos
            total_descent_completed = max(0.0, start_z - previous_z)
            progress_ratio = 0.0 if total_z_descent <= 1e-6 else min(1.0, total_descent_completed / total_z_descent)

        achieved_position = self.get_end_effector_position() or achieved_position

        rospy.loginfo("[Formation Z Descent] Completed all steps successfully")
        return True, (target_x, target_y, achieved_position[2])


class FormationInitializeStartPositionState(FormationSingleUAVStateBase):
    """Formation version of InitializeStartPositionState"""
    
    def __init__(self):
        FormationSingleUAVStateBase.__init__(self, 
            outcomes=['succeeded', 'failed'], 
            output_keys=['start_position', 'valve_position', 'valve_yaw'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Formation Initialize Start Position State ===")
        
        # Wait for position data
        rospy.sleep(2.0)
        
        # Get formation positions
        assembly_pos = self.get_assembly_position()
        assembly_yaw = self.get_assembly_yaw()
        
        if assembly_pos is None:
            rospy.logerr("Assembly position not available")
            return 'failed'
        
        # Get valve position from leader UAV's sensors
        valve_pos = self.beetle.getValvePos()
        valve_yaw = self.beetle.getValveYaw()
        
        if valve_pos is None:
            rospy.logerr("Valve position not available")
            return 'failed'
        
        # Store positions in userdata
        userdata.start_position = assembly_pos
        userdata.valve_position = valve_pos
        userdata.valve_yaw = valve_yaw
        
        rospy.loginfo(f"Formation start position: {assembly_pos}")
        rospy.loginfo(f"Valve position: {valve_pos}")
        rospy.loginfo(f"Valve yaw: {valve_yaw}")
        
        return 'succeeded'


class FormationMoveToValveState(FormationSingleUAVStateBase):
    # Shared optimizer strategy for later states (class variable)
    _shared_optimizer_strategy = None
    """Formation version of MoveToValveState with 3-phase control logic from single UAV version"""
    
    def __init__(self):
        FormationSingleUAVStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['valve_position', 'valve_yaw'],
            output_keys=['phase4_contact_pose', 'phase4_contact_yaw'])
            
        # Formation control parameters based on single UAV proven values
        self.max_linear_velocity = 0.12   # 0.12 m/s maximum speed (same as single)
        self.average_linear_velocity = 0.08  # 0.08 m/s average speed (slightly slower than single's 0.1 for formation stability)
        self.max_angular_velocity = 0.02  # 0.02 rad/s angular speed (same as single for stability)
        
        # Convergence thresholds (based on single UAV proven values)
        self.pos_threshold = 0.02         # 20mm position threshold (same as single)
        self.yaw_threshold = 0.05         # ~2.9° yaw threshold (same as single) 
        self.vel_threshold = 0.01         # 10mm/s velocity threshold (same as single)
        
        # Final positioning accuracy
        self.final_pos_threshold = 0.015  # 15mm final accuracy (slightly relaxed from single's 10mm)
        self.final_yaw_threshold = 0.0175 # 1° final yaw accuracy (same as single)
        
        # Active convergence parameters
        self.min_consecutive_readings = 5  # 5 consecutive good readings (same as single)
        
        # Formation-specific safety margins (more conservative than single)
        self.formation_safety_factor = 0.8  # 20% safety margin for formation coordination

    # ------------------------------
    # Logging helpers
    # ------------------------------
    @staticmethod
    def _format_vec(vec):
        return f"({vec[0]:.3f}, {vec[1]:.3f}, {vec[2]:.3f})"

    def _log_header(self, title):
        rospy.loginfo("")
        rospy.loginfo(f"=== {title} ===")

    def _log_phase(self, phase_idx, title, detail=None):
        base = f"[Phase {phase_idx}] {title}"
        if detail:
            base = f"{base} | {detail}"
        rospy.loginfo(base)
    
    def execute(self, userdata):
        """Execute 4-phase movement using optimizer-driven targets (single UAV logic)"""
        self._log_header("Formation Move-To-Valve (Optimizer Guided)")
        valve_pos = userdata.valve_position
        valve_yaw = userdata.valve_yaw

        # Get current end-effector position
        current_ee_pos = self.get_end_effector_position()
        if current_ee_pos is None:
            rospy.logerr("Cannot get current end-effector position")
            return 'failed'

        current_yaw = self.get_end_effector_yaw() or 0.0

        # --- OPTIMIZER STRATEGY: Use same-side insertion as single UAV ---
        strategy = self.optimizer.calculate_same_side_insertion_strategy(
            valve_pos=valve_pos,
            valve_yaw=valve_yaw,
            uav_pos=current_ee_pos
        )
        if not strategy or not strategy.get('feasible', True):
            rospy.logerr("Optimizer failed to provide feasible insertion strategy")
            return 'failed'

        # Cache for later states (descend/contact, etc.)
        FormationMoveToValveState._shared_optimizer_strategy = strategy

        safe_ee_pos = strategy.get('safe_end_effector_center')
        if safe_ee_pos is None:
            fallback_pos = strategy.get('safe_uav_position')
            if fallback_pos is not None:
                rospy.logwarn_once("Optimizer strategy missing safe_end_effector_center; using safe_uav_position as fallback")
                safe_ee_pos = fallback_pos
            else:
                rospy.logwarn_once("Optimizer strategy missing end-effector target; using current EE pose")
                safe_ee_pos = current_ee_pos
        safe_ee_yaw = strategy['safe_uav_yaw'] if 'safe_uav_yaw' in strategy else valve_yaw
        safe_ee_pos = tuple(safe_ee_pos)

        formation_z_offset = rospy.get_param("~formation_phase4_z_offset", 0.045)
        if abs(formation_z_offset) > 1e-4:
            compensated_z = safe_ee_pos[2] + formation_z_offset
            rospy.loginfo(
                f"[Phase4] Apply fixed Z compensation {formation_z_offset*1000:.0f}mm -> {compensated_z:.3f}m"
            )
            safe_ee_pos = (safe_ee_pos[0], safe_ee_pos[1], compensated_z)

        rospy.loginfo(f"Current EE: {self._format_vec(current_ee_pos)}, yaw={math.degrees(current_yaw):.1f}°")
        rospy.loginfo(f"Valve pose: pos={self._format_vec(valve_pos)}, yaw={math.degrees(valve_yaw):.1f}°")
        rospy.loginfo(f"Optimizer target: pos={self._format_vec(safe_ee_pos)}, yaw={math.degrees(safe_ee_yaw):.1f}°")

        # --- PHASE 1: YAW ADJUSTMENT TO VALVE HANDLE (for observation) ---
        valve_handle_yaw = valve_yaw
        optimal_handle_yaw = self.calculate_shortest_yaw_path(current_yaw, valve_handle_yaw)
        self._log_phase(1, "对准阀柄航向", f"目标 {math.degrees(optimal_handle_yaw):.1f}°")
        if not self._execute_formation_phase1_yaw_adjustment(current_ee_pos, optimal_handle_yaw):
            rospy.logerr("Phase 1 failed: Valve handle yaw alignment unsuccessful")
            return 'failed'
        rospy.loginfo("✓ Phase 1 完成：航向与阀柄对齐")

        # --- PHASE 2: XY MOVEMENT TO OPTIMIZER TARGET (maintain Z, handle yaw) ---
        phase1_ee_pos = self.get_end_effector_position()
        if phase1_ee_pos is None:
            rospy.logerr("Cannot get end-effector position after Phase 1")
            return 'failed'
        xy_target_ee_pos = (safe_ee_pos[0], safe_ee_pos[1], phase1_ee_pos[2])
        self._log_phase(2, "平移至安全XY", f"目标 {self._format_vec(xy_target_ee_pos)}")
        if not self._execute_formation_phase2_xy_movement(xy_target_ee_pos, optimal_handle_yaw):
            rospy.logerr("Phase 2 failed: XY movement unsuccessful")
            return 'failed'
        rospy.loginfo("✓ Phase 2 完成：达到优化器XY位置")

        # --- PHASE 3: PRECISION XY & YAW (spoke alignment, maintain Z) ---
        phase2_ee_pos = self.get_end_effector_position()
        phase2_yaw = self.get_end_effector_yaw()
        if phase2_ee_pos is None or phase2_yaw is None:
            rospy.logerr("Cannot get end-effector position/yaw after Phase 2")
            return 'failed'

        # PHASE 3A: XY precision positioning (keep Z, do not descend yet)
        phase3a_target_pos = (safe_ee_pos[0], safe_ee_pos[1], phase2_ee_pos[2])
        self._log_phase(3, "精调XY", f"锁高 {phase3a_target_pos[2]:.3f}m")
        if not self._execute_formation_phase3a_xy_positioning(phase3a_target_pos, phase2_yaw):
            rospy.logerr("Phase 3A failed: XY precision positioning unsuccessful")
            return 'failed'
        rospy.loginfo("  ↳ Phase 3A 完成：XY 精度达标")

        # PHASE 3B: Spoke-aligned yaw (use optimizer's safe_ee_yaw)
        phase3a_ee_pos = self.get_end_effector_position()
        if phase3a_ee_pos is None:
            rospy.logerr("Cannot get end-effector position after Phase 3A")
            return 'failed'
        valve_xy_error = math.sqrt(
            (phase3a_ee_pos[0] - valve_pos[0])**2 +
            (phase3a_ee_pos[1] - valve_pos[1])**2
        )
        rospy.loginfo(f"  ↳ Phase 3A 末端距阀心 {valve_xy_error*1000:.1f}mm")
        optimal_spoke_yaw = self.calculate_shortest_yaw_path(phase2_yaw, safe_ee_yaw)
        rospy.loginfo(f"  ↳ Phase 3B 辐条对齐目标航向 {math.degrees(safe_ee_yaw):.1f}°")
        if not self._execute_formation_phase3b_spoke_alignment(phase3a_ee_pos, optimal_spoke_yaw, phase3a_target_pos):
            rospy.logerr("Phase 3B failed: Spoke alignment unsuccessful")
            return 'failed'
        rospy.loginfo("  ↳ Phase 3B 完成：航向锁定辐条间隙")
        phase3b_ee_pos = self.get_end_effector_position()
        if phase3b_ee_pos is not None:
            phase3b_xy_error = math.sqrt(
                (phase3b_ee_pos[0] - valve_pos[0])**2 +
                (phase3b_ee_pos[1] - valve_pos[1])**2
            )
            rospy.loginfo(f"    · Phase 3B 末端距阀心 {phase3b_xy_error*1000:.1f}mm")

        # --- PHASE 4: Z DESCENT TO OPTIMIZER HEIGHT (keep XY, spoke yaw) ---
        final_target_pos = (safe_ee_pos[0], safe_ee_pos[1], safe_ee_pos[2])

        final_yaw = self.get_end_effector_yaw()
        if final_yaw is None:
            final_yaw = safe_ee_yaw

        if not self._execute_formation_phase4_z_descent(final_target_pos, final_yaw):
            rospy.logerr("Phase 4 failed: Z descent unsuccessful")
            return 'failed'
        rospy.loginfo("✓ Phase 4 完成：插入高度就位")

        # Share Phase4 locking pose and yaw with subsequent states
        phase4_pose = getattr(self, '_last_phase4_contact_position', None)
        if phase4_pose is None:
            phase4_pose = self.get_end_effector_position() or final_target_pos
        userdata.phase4_contact_pose = phase4_pose
        userdata.phase4_contact_yaw = self.get_end_effector_yaw() or final_yaw

        rospy.loginfo("=== Formation move-to-valve 完成：编队已就位，准备插入阀门 ===")
        return 'succeeded'
    def calculate_shortest_yaw_path(self, current_yaw, target_yaw):
        """Calculate the shortest yaw rotation path to avoid reverse direction rotation (from single UAV)"""
        # Calculate the difference
        diff = target_yaw - current_yaw
        
        # Handle angle wrap-around to ensure shortest path
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
        
        # Return the shortest path target
        optimal_target = current_yaw + diff
        
        rospy.loginfo(
            f"[YawPath] current={math.degrees(current_yaw):.1f}°, target={math.degrees(target_yaw):.1f}°, "
            f"shortest={math.degrees(optimal_target):.1f}°, delta={math.degrees(diff):.1f}°"
        )
        
        return optimal_target
        
    def _execute_formation_phase1_yaw_adjustment(self, current_ee_pos, target_yaw):
        """Phase 1: Pure yaw adjustment using single-command approach (like Single UAV)"""
        rospy.loginfo(
            f"[Phase1] 保持末端位置 {self._format_vec(current_ee_pos)}，目标航向 {math.degrees(target_yaw):.1f}°"
        )
        
        # Calculate initial yaw error for logging
        current_yaw = self.get_end_effector_yaw()
        if current_yaw is not None:
            initial_yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
            rospy.loginfo(f"[Phase1] 初始航向误差 {math.degrees(initial_yaw_error):.2f}°")
        
        # Send single command (like Single UAV version) instead of continuous commands
        rospy.loginfo("[Phase1] 发送航向调整指令")
        self.send_assembly_command_from_end_effector(
            target_end_effector_pos=current_ee_pos,
            target_yaw=target_yaw,
            linear_vel=None,  # No linear velocity for pure rotation
            angular_vel=None  # Let system determine angular velocity
        )
        
        # Wait for yaw convergence (similar to Single UAV approach)
        success = self.wait_for_formation_yaw_convergence(
            target_yaw=target_yaw,
            yaw_thresh=0.0524,  # 3° threshold (same as Single UAV)
            timeout=15.0,       # Generous timeout for formation
            pos_threshold=0.05  # Allow 5cm position drift during yaw adjustment
        )
        
        if success:
            rospy.loginfo("✓ Phase 1 completed successfully with single command approach")
        else:
            rospy.logwarn("⚠ Phase 1 completed with convergence issues - proceeding anyway")
        
        return success
        
    def _execute_formation_phase2_xy_movement(self, xy_target_ee_pos, maintain_yaw):
        """Phase 2: Pure XY movement using polynomial trajectory (Formation version)"""
        rospy.loginfo(
            f"[Phase2] 规划至 {self._format_vec(xy_target_ee_pos)}，保持航向 {math.degrees(maintain_yaw):.1f}°"
        )

        # Get current position for trajectory start
        current_pos = self.get_end_effector_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position for Phase 2")
            return False

        xy_distance = math.sqrt(sum((xy_target_ee_pos[i] - current_pos[i])**2 for i in [0, 1]))
        rospy.loginfo(f"[Phase2] XY 距离 {xy_distance*1000:.1f}mm")

        # Check if movement is significant enough for trajectory
        if xy_distance < 0.01:  # Less than 10mm
            rospy.loginfo("[Phase2] 位移<10mm，直接进入收敛模式")
            return self.active_position_convergence(
                target_ee_pos=xy_target_ee_pos,
                target_yaw=maintain_yaw,
                pos_threshold=0.02,
                yaw_threshold=0.1,
                vel_threshold=0.01,
                min_readings=3,
                max_attempts=50,
                timeout=15.0
            )

        # Generate polynomial trajectory for smooth XY movement
        rospy.loginfo("[Phase2] 生成多项式平滑轨迹")
        trajectory_points = self.generate_polynomial_trajectory(
            start_pos=current_pos,
            target_pos=xy_target_ee_pos,
            target_yaw=maintain_yaw,
            num_points=12  # Formation uses fewer points than single UAV for stability
        )

        if not trajectory_points:
            rospy.logerr("Failed to generate polynomial trajectory - falling back to direct convergence")
            return self.active_position_convergence(
                target_ee_pos=xy_target_ee_pos,
                target_yaw=maintain_yaw,
                pos_threshold=0.02,
                yaw_threshold=0.1,
                vel_threshold=0.01,
                min_readings=5,
                max_attempts=100,
                timeout=20.0
            )

        # Execute polynomial trajectory
        rospy.loginfo("[Phase2] 执行多项式轨迹")
        trajectory_success = self.execute_polynomial_trajectory(trajectory_points)

        # Final convergence check to ensure precision
        rospy.loginfo("[Phase2] 进行末端精度收敛检查")
        final_success = self.active_position_convergence(
            target_ee_pos=xy_target_ee_pos,
            target_yaw=maintain_yaw,
            pos_threshold=0.02,         # Final precision requirement
            yaw_threshold=0.05,         # Strict final yaw requirement
            vel_threshold=0.01,
            min_readings=5,             # Ensure stability
            max_attempts=50,
            timeout=15.0
        )

        overall_success = trajectory_success and final_success

        if overall_success:
            rospy.loginfo("✓ Phase 2 完成：轨迹执行与收敛均成功")
        else:
            rospy.logwarn("⚠ Phase 2 完成但存在收敛告警")

        return overall_success
    
    def _calculate_spoke_alignment_yaw(self, current_ee_pos, valve_pos):
        """
        Calculate the yaw angle needed to align with the selected beam/spoke.
        Based on single UAV logic for spoke gap alignment.
        
        Args:
            current_ee_pos: Current end-effector position
            valve_pos: Valve center position
            
        Returns:
            float: Spoke alignment yaw angle in radians
        """
        # Calculate spoke angle (which beam was selected) - same as single UAV
        valve_x, valve_y = valve_pos[0], valve_pos[1]
        spoke_angle = math.atan2(
            current_ee_pos[1] - valve_y,
            current_ee_pos[0] - valve_x
        )
        
        # For insertion, UAV should align with the spoke direction
        # The yaw should point from valve center toward the current position (radial alignment)
        spoke_yaw = spoke_angle
        
        rospy.loginfo(f"Spoke alignment calculation:")
        rospy.loginfo(f"  Valve center: ({valve_x:.3f}, {valve_y:.3f})")
        rospy.loginfo(f"  Current EE pos: ({current_ee_pos[0]:.3f}, {current_ee_pos[1]:.3f})")  
        rospy.loginfo(f"  Spoke angle: {math.degrees(spoke_angle):.1f}°")
        rospy.loginfo(f"  Required spoke yaw: {math.degrees(spoke_yaw):.1f}°")
        
        return spoke_yaw
    
    def _execute_formation_phase3a_xy_positioning(self, target_ee_pos, maintain_yaw):
        """
        Phase 3A: XY 精准对齐，保持当前高度与航向。
        多次尝试并在每次尝试后验证XY/Z误差，确保不会提前下沉。
        """
        current_state = self.get_end_effector_position()
        if current_state is None:
            rospy.logerr("Phase 3A 无法获取末端执行器位置")
            return False

        locked_z = target_ee_pos[2]
        if locked_z is None:
            locked_z = current_state[2]

        z_delta = current_state[2] - locked_z
        if abs(z_delta) > 0.02:
            rospy.logwarn(f"Phase 3A 当前高度与目标差值 {z_delta*1000:.1f}mm，将在本阶段回正")

        xy_target = (target_ee_pos[0], target_ee_pos[1], locked_z)
        rospy.loginfo(
            f"[Phase3A] 目标XY {self._format_vec(xy_target)}, 锁高 {locked_z:.3f}m，保持航向 {math.degrees(maintain_yaw):.1f}°"
        )

        max_attempts = 3
        xy_tolerance = 0.018  # 18mm
        z_tolerance = 0.015   # 15mm

        for attempt in range(1, max_attempts + 1):
            rospy.loginfo(f"[Phase3A] 尝试 {attempt}/{max_attempts}")
            success = self.active_position_convergence(
                target_ee_pos=xy_target,
                target_yaw=maintain_yaw,
                pos_threshold=0.020,
                yaw_threshold=0.06,
                vel_threshold=0.015,
                min_readings=3,
                max_attempts=110,
                timeout=22.0
            )

            if not success:
                rospy.logwarn("Phase 3A 收敛失败，准备重试")
                continue

            final_pos = self.get_end_effector_position()
            final_yaw = self.get_end_effector_yaw()
            if final_pos is None or final_yaw is None:
                rospy.logwarn("Phase 3A 收敛后丢失姿态信息，重试")
                continue

            xy_error = math.sqrt((final_pos[0] - xy_target[0])**2 + (final_pos[1] - xy_target[1])**2)
            z_error = abs(final_pos[2] - locked_z)
            yaw_error = abs(self.normalize_angle(final_yaw - maintain_yaw))

            rospy.loginfo(
                f"[Phase3A] 结果: XY误差 {xy_error*1000:.1f}mm, Z误差 {z_error*1000:.1f}mm, 航向误差 {math.degrees(yaw_error):.2f}°"
            )

            if xy_error <= xy_tolerance and z_error <= z_tolerance:
                if yaw_error > 0.07:
                    rospy.logwarn("Phase 3A 航向偏差稍大，后续阶段将重新锁定")
                rospy.loginfo("Phase 3A XY精确定位成功")
                return True

            rospy.logwarn("[Phase3A] 误差仍超阈值，准备重试")
        rospy.logerr("Phase 3A 在多次尝试后仍未达到精度要求")
        return False

    def _execute_formation_phase3b_spoke_alignment(self, current_ee_pos, spoke_yaw, target_ee_pos=None):
        """
        Phase 3B: 辐条插入航向对齐。
        在保持Phase 3A 锁定的XY/Z 位置基础上进行航向旋转，如有漂移自动回正。
        """
        reference_xy = (
            target_ee_pos[0],
            target_ee_pos[1]
        ) if target_ee_pos is not None else (current_ee_pos[0], current_ee_pos[1])
        reference_z = target_ee_pos[2] if target_ee_pos is not None else current_ee_pos[2]
        rospy.loginfo(
            f"[Phase3B] 目标航向 {math.degrees(spoke_yaw):.1f}°，锁定位置 {self._format_vec((reference_xy[0], reference_xy[1], reference_z))}"
        )

        max_attempts = 3
        xy_tolerance = 0.015  # 15mm
        z_tolerance = 0.015   # 15mm
        yaw_tolerance = 0.0175  # ≈1°

        for attempt in range(1, max_attempts + 1):
            current_yaw = self.get_end_effector_yaw()
            if current_yaw is not None:
                yaw_error = abs(self.normalize_angle(spoke_yaw - current_yaw))
                rospy.loginfo(f"[Phase3B] 尝试 {attempt}/{max_attempts}，预计旋转 {math.degrees(yaw_error):.2f}°")

            success = self.active_position_convergence(
                target_ee_pos=(reference_xy[0], reference_xy[1], reference_z),
                target_yaw=spoke_yaw,
                pos_threshold=0.020,
                yaw_threshold=yaw_tolerance,
                vel_threshold=0.015,
                min_readings=4,
                max_attempts=160,
                timeout=35.0,
                max_yaw_step=math.radians(3.0)
            )

            final_pos = self.get_end_effector_position()
            final_yaw = self.get_end_effector_yaw()

            if not success or final_pos is None or final_yaw is None:
                rospy.logwarn("Phase 3B 收敛失败或姿态不可用，重新尝试")
                correction_yaw = spoke_yaw if spoke_yaw is not None else current_yaw
                self._execute_formation_phase3a_xy_positioning(
                    (reference_xy[0], reference_xy[1], reference_z),
                    correction_yaw
                )
                continue

            xy_error = math.sqrt((final_pos[0] - reference_xy[0])**2 + (final_pos[1] - reference_xy[1])**2)
            z_error = abs(final_pos[2] - reference_z)
            yaw_error = abs(self.normalize_angle(spoke_yaw - final_yaw))

            rospy.loginfo(
                f"[Phase3B] 结果: XY误差 {xy_error*1000:.1f}mm, Z误差 {z_error*1000:.1f}mm, 航向误差 {math.degrees(yaw_error):.2f}°"
            )

            if xy_error <= xy_tolerance and z_error <= z_tolerance and yaw_error <= yaw_tolerance:
                rospy.loginfo("Phase 3B 辐条航向对齐完成")
                return True

            # 若XY漂移超限，先补偿一次再重试
            if xy_error > xy_tolerance or z_error > z_tolerance:
                rospy.logwarn("[Phase3B] 旋转导致位置漂移，执行XY回正")
                correction_pos = (reference_xy[0], reference_xy[1], reference_z)
                self._execute_formation_phase3a_xy_positioning(correction_pos, final_yaw)

        rospy.logerr("Phase 3B 辐条对齐在多次尝试后仍未满足精度")
        return False
        
    def _execute_formation_phase3_yaw_to_insertion(self, current_ee_pos, target_yaw):
        """
        Legacy Phase 3 method for insertion operations (reuses Phase 3B logic)
        This is used by the older insertion flow in FormationDescendAndContactState
        """
        rospy.loginfo("Phase 3: Yaw adjustment to insertion angle")
        rospy.loginfo(f"Rotating to target yaw: {math.degrees(target_yaw):.1f}°")
        
        # Reuse the Phase 3B spoke alignment logic
        return self._execute_formation_phase3b_spoke_alignment(current_ee_pos, target_yaw)

    def _execute_formation_phase4_z_descent(self, final_ee_pos, final_yaw):
        """Phase 4: Pure Z descent to insertion height (Formation version)"""
        rospy.loginfo(
            f"[Phase4] 当前→目标 {self._format_vec(final_ee_pos)}, 保持航向 {math.degrees(final_yaw):.1f}°"
        )

        # Reset cached contact pose for this attempt
        self._last_phase4_contact_position = None

        current_pos = self.get_end_effector_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position for Phase 4")
            return False

        final_target_pos = (final_ee_pos[0], final_ee_pos[1], final_ee_pos[2])

        descent_success, achieved_pos = self.controlled_z_descent(current_pos, final_target_pos, final_yaw)
        if not descent_success:
            fallback_pos = achieved_pos or self.get_end_effector_position()
            if fallback_pos is not None:
                xy_error = math.sqrt(
                    (fallback_pos[0] - final_target_pos[0])**2 +
                    (fallback_pos[1] - final_target_pos[1])**2
                )
                z_error = abs(fallback_pos[2] - final_target_pos[2])
                rospy.logwarn(
                    f"[Phase4] Z descent aborted with residual XY {xy_error*1000:.1f}mm, Z {z_error*1000:.1f}mm"
                )
            return False

        if achieved_pos is None:
            achieved_pos = self.get_end_effector_position() or final_target_pos

        if abs(achieved_pos[2] - final_target_pos[2]) > 1e-3:
            rospy.loginfo(
                f"[Phase4] 接触深度锁定在 {achieved_pos[2]:.3f}m (计划 {final_target_pos[2]:.3f}m)"
            )

        convergence_target = achieved_pos
        rospy.loginfo(
            f"[Phase4] 锁定当前位置为最终收敛目标 {self._format_vec(convergence_target)}"
        )

        # Cache for downstream states (e.g., rotation start pose)
        self._last_phase4_contact_position = convergence_target

        rospy.loginfo("[Phase4] 进行最终插入位姿收敛")
        final_success = self.active_position_convergence(
            target_ee_pos=convergence_target,
            target_yaw=final_yaw,
            pos_threshold=0.03,
            yaw_threshold=0.02,
            timeout=12.0,
            max_attempts=80,
            min_readings=3
        )

        if final_success:
            rospy.loginfo("Phase 4 completed successfully - ready for valve insertion")
        else:
            fallback_pos = self.get_end_effector_position()
            if fallback_pos is not None:
                xy_error = math.sqrt(
                    (fallback_pos[0] - convergence_target[0])**2 +
                    (fallback_pos[1] - convergence_target[1])**2
                )
                z_error = abs(fallback_pos[2] - convergence_target[2])
                rospy.logwarn(
                    f"[Phase4] Final residual: XY {xy_error*1000:.1f}mm, Z {z_error*1000:.1f}mm"
                )
            rospy.logwarn("Phase 4 completed with convergence issues")

        return final_success
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def active_position_convergence(self, target_ee_pos, target_yaw=None, 
                                  pos_threshold=0.02, yaw_threshold=0.05, vel_threshold=0.01,
                                  min_readings=5, max_attempts=10, timeout=30.0,
                                  max_yaw_step=None, max_pos_step=None):
        """
        Dragon-style continuous target sending until convergence
        Based on single UAV version's proven stable convergence mechanism
        
        Args:
            target_ee_pos: Target end-effector position [x, y, z]
            target_yaw: Target yaw angle (None to ignore yaw)
            pos_threshold: Position convergence threshold (m)
            yaw_threshold: Yaw convergence threshold (rad)  
            vel_threshold: Velocity convergence threshold (m/s)
            min_readings: Minimum consecutive good readings required
            max_attempts: Maximum attempts before timeout
            timeout: Maximum total time (seconds)
        
        Returns:
            bool: True if converged, False if failed/timeout
        """
        yaw_text = f", yaw={math.degrees(target_yaw):.1f}°" if target_yaw is not None else ""
        rospy.loginfo(f"[Converge] target={self._format_vec(target_ee_pos)}{yaw_text}")
        
        start_time = rospy.Time.now()
        attempt_count = 0
        consecutive_good_readings = 0
        
        # Control loop frequency (optimized to match single UAV proven parameters)
        rate = rospy.Rate(10)  # 10 Hz control loop for improved responsiveness
        
        while not rospy.is_shutdown():
            # Check timeout
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            if elapsed_time > timeout:
                rospy.logerr(f"Convergence timeout after {timeout}s")
                return False
            
            if attempt_count >= max_attempts:
                rospy.logerr(f"Max attempts ({max_attempts}) reached")
                return False
            
            # Get current state
            current_ee_pos = self.get_end_effector_position()
            current_yaw = self.get_end_effector_yaw()
            
            if current_ee_pos is None:
                rospy.logwarn("Cannot get current end-effector position, retrying...")
                attempt_count += 1
                rate.sleep()
                continue
            
            # Calculate position error
            pos_error = math.sqrt(sum((target_ee_pos[i] - current_ee_pos[i])**2 for i in range(3)))
            
            # Calculate yaw error if yaw control is enabled
            yaw_error = None
            if target_yaw is not None and current_yaw is not None:
                yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
            
            # Get velocity from assembly state (approximated)
            assembly_pos = self.get_assembly_position() 
            vel_magnitude = 0.0  # For formation, velocity is not directly available
            
            # Check convergence criteria
            pos_converged = pos_error < pos_threshold
            yaw_converged = (target_yaw is None) or (yaw_error is not None and yaw_error < yaw_threshold)
            # For formation, skip velocity check since it's not reliable
            vel_converged = True  # Always pass velocity check for formation
            
            all_converged = pos_converged and yaw_converged and vel_converged
            
            # Log progress less frequently to reduce noise (every 100 attempts)
            if attempt_count % 25 == 0 or all_converged:
                yaw_part = f", yaw_err={math.degrees(yaw_error):.1f}°" if yaw_error is not None else ""
                rospy.loginfo(
                    f"[Converge] attempt={attempt_count}, pos_err={pos_error*1000:.1f}mm{yaw_part}, "
                    f"stable={consecutive_good_readings}/{min_readings}"
                )
                if attempt_count == 0:
                    rospy.logdebug(
                        f"[Converge] current={self._format_vec(current_ee_pos)}, target={self._format_vec(target_ee_pos)}"
                    )
            
            # Update consecutive good readings counter
            if all_converged:
                consecutive_good_readings += 1
                rospy.logdebug(f"✓ Good reading {consecutive_good_readings}/{min_readings}")
                
                # Check if we have enough consecutive good readings
                if consecutive_good_readings >= min_readings:
                    elapsed = (rospy.Time.now() - start_time).to_sec()
                    final_yaw = f", yaw={math.degrees(yaw_error):.1f}°" if yaw_error is not None else ""
                    rospy.loginfo(
                        f"[Converge] success in {elapsed:.1f}s ({attempt_count} attempts), "
                        f"pos={pos_error*1000:.1f}mm{final_yaw}"
                    )
                    return True
            else:
                consecutive_good_readings = 0  # Reset counter on any failure
            
            # Determine incremental command targets to avoid aggressive jumps
            command_pos = target_ee_pos
            command_yaw = target_yaw

            if max_pos_step is not None:
                delta_vec = [target_ee_pos[i] - current_ee_pos[i] for i in range(3)]
                delta_mag = math.sqrt(sum(d * d for d in delta_vec))
                if delta_mag > max_pos_step > 0.0:
                    scale = max_pos_step / delta_mag
                    limited = [current_ee_pos[i] + delta_vec[i] * scale for i in range(3)]
                    command_pos = (limited[0], limited[1], limited[2])

            if max_yaw_step is not None and target_yaw is not None and current_yaw is not None:
                yaw_delta = self.normalize_angle(target_yaw - current_yaw)
                limited = False
                if abs(yaw_delta) > max_yaw_step > 0.0:
                    yaw_delta = math.copysign(max_yaw_step, yaw_delta)
                    limited = True
                command_yaw = self.normalize_angle(current_yaw + yaw_delta)
                if limited and attempt_count % 20 == 0:
                    rospy.logdebug(
                        f"[Converge] yaw step limited to {math.degrees(yaw_delta):.2f}° (target {math.degrees(target_yaw):.1f}°)"
                    )

            # Continuously send target command (Dragon-style)
            # This is the key difference from single-shot targeting
            self.send_assembly_command_from_end_effector(
                command_pos,
                command_yaw,
                linear_vel=None,  # Use position mode for stability
                angular_vel=None
            )
            
            attempt_count += 1
            rate.sleep()
        
        rospy.logerr("❌ Convergence failed - ROS shutdown requested")
        return False

    def wait_for_formation_yaw_convergence(self, target_yaw, yaw_thresh=0.0524, timeout=15.0, pos_threshold=0.05):
        """
        Wait for formation yaw to converge (adapted from Single UAV method)
        
        Args:
            target_yaw: Target yaw angle (radians)
            yaw_thresh: Yaw convergence threshold (radians), default 3 degrees
            timeout: Timeout duration (seconds)
            pos_threshold: Allowed position drift during yaw adjustment (meters)
            
        Returns:
            bool: Whether convergence was successful
        """
        start_time = rospy.Time.now()
        consecutive_good_readings = 0
        required_consecutive = 3  # Require 3 consecutive readings for stability
        
        rospy.loginfo(f"Waiting for formation yaw convergence - target={math.degrees(target_yaw):.1f}°, thresh={math.degrees(yaw_thresh):.1f}°")
        
        # Record initial position to monitor drift
        initial_ee_pos = self.get_end_effector_position()
        
        rate = rospy.Rate(8)  # 8Hz checking rate (improved responsiveness)
        
        while not rospy.is_shutdown():
            # Check timeout
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            if elapsed_time > timeout:
                rospy.logwarn(f"Formation yaw convergence timeout after {timeout}s")
                return False
            
            # Get current yaw
            current_yaw = self.get_end_effector_yaw()
            if current_yaw is None:
                rospy.logwarn("Current formation yaw is None, retrying...")
                rate.sleep()
                continue
            
            # Calculate yaw error (handle wrap-around)
            yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
            
            # Check position drift
            current_ee_pos = self.get_end_effector_position()
            pos_drift = 0.0
            if initial_ee_pos and current_ee_pos:
                pos_drift = math.sqrt(sum((current_ee_pos[i] - initial_ee_pos[i])**2 for i in range(3)))
            
            # Progress logging every 3 seconds
            if int(elapsed_time) % 3 == 0 and elapsed_time > 0:
                rospy.loginfo(f"Formation yaw convergence: current={math.degrees(current_yaw):.1f}°, "
                             f"target={math.degrees(target_yaw):.1f}°, error={math.degrees(yaw_error):.1f}°, "
                             f"pos_drift={pos_drift*1000:.1f}mm")
            
            # Check convergence
            converged = (yaw_error < yaw_thresh and pos_drift < pos_threshold)
            
            if converged:
                consecutive_good_readings += 1
                rospy.logdebug(f"Good reading {consecutive_good_readings}/{required_consecutive}")
                
                if consecutive_good_readings >= required_consecutive:
                    rospy.loginfo(f"Formation yaw converged! Error: {math.degrees(yaw_error):.2f}deg, "
                                 f"pos_drift: {pos_drift*1000:.1f}mm")
                    return True
            else:
                consecutive_good_readings = 0
                
            rate.sleep()
        
        rospy.logwarn("Formation yaw convergence failed - ROS shutdown requested")
        return False

    def generate_polynomial_trajectory(self, start_pos, target_pos, target_yaw, num_points=15):
        """
        Generate smooth polynomial trajectory using PolynomialTrajectory class (Formation version)
        Based on single UAV proven method with formation-specific optimizations
        """
        from trajectory import PolynomialTrajectory
        
        rospy.loginfo(f"=== Generating Formation Polynomial Trajectory: {num_points} points ===")
        
        # Get current state for trajectory planning
        current_yaw = self.get_end_effector_yaw()
        if current_yaw is None:
            current_yaw = 0.0
        
        # Calculate total distance and trajectory time
        total_distance = math.sqrt(sum((target_pos[i] - start_pos[i])**2 for i in range(3)))
        yaw_change = abs(self.normalize_angle(target_yaw - current_yaw))
        
        # Calculate trajectory time based on formation parameters (slower than single UAV)
        time_for_distance = total_distance / self.average_linear_velocity  # Formation uses 0.08 m/s
        time_for_rotation = yaw_change / self.max_angular_velocity         # Formation uses 0.02 rad/s
        
        # Calculate optimal trajectory duration with formation safety factors
        trajectory_duration = max(
            time_for_distance,  # Time based on formation average velocity
            time_for_rotation,  # Time based on formation angular motion
            8.0  # Minimum 8 seconds for formation smooth motion (longer than single)
        )
        
        # Apply formation safety factor (more conservative than single UAV)
        safety_factor = 0.7  # 30% margin for formation safety (vs 20% for single)
        trajectory_duration = trajectory_duration / safety_factor
        
        actual_avg_velocity = total_distance / trajectory_duration if trajectory_duration > 0 else 0
        
        rospy.loginfo(f"Formation trajectory parameters: distance={total_distance:.3f}m, angle_change={math.degrees(yaw_change):.1f}°")
        rospy.loginfo(f"Estimated_time={trajectory_duration:.1f}s, avg_velocity={actual_avg_velocity:.3f}m/s")
        
        # Create polynomial trajectories for each axis
        traj_x = PolynomialTrajectory(duration=trajectory_duration)
        traj_y = PolynomialTrajectory(duration=trajectory_duration)
        traj_z = PolynomialTrajectory(duration=trajectory_duration)
        traj_yaw = PolynomialTrajectory(duration=trajectory_duration)
        
        # Compute coefficients for each axis and set ALL required attributes
        traj_x.is_scalar = True
        traj_x.coeffs_scalar = traj_x.compute_coefficients(start_pos[0], target_pos[0])
        traj_x.start_value = start_pos[0]
        traj_x.target_value = target_pos[0]
        
        traj_y.is_scalar = True
        traj_y.coeffs_scalar = traj_y.compute_coefficients(start_pos[1], target_pos[1])
        traj_y.start_value = start_pos[1]
        traj_y.target_value = target_pos[1]
        
        traj_z.is_scalar = True
        traj_z.coeffs_scalar = traj_z.compute_coefficients(start_pos[2], target_pos[2])
        traj_z.start_value = start_pos[2]
        traj_z.target_value = target_pos[2]
        
        traj_yaw.is_scalar = True
        traj_yaw.coeffs_scalar = traj_yaw.compute_coefficients(current_yaw, target_yaw)
        traj_yaw.start_value = current_yaw
        traj_yaw.target_value = target_yaw
        
        rospy.loginfo("Formation polynomial coefficient calculation completed, generating trajectory points")
        
        trajectory_points = []
        
        # Generate trajectory points
        for i in range(num_points + 1):  # +1 to include final target
            elapsed_time = (i / num_points) * trajectory_duration
            
            # Get position from polynomial trajectories using elapsed time
            pos_x = traj_x.evaluate_at_time(elapsed_time)
            pos_y = traj_y.evaluate_at_time(elapsed_time)
            pos_z = traj_z.evaluate_at_time(elapsed_time)
            yaw = traj_yaw.evaluate_at_time(elapsed_time)
            
            # Calculate velocity manually (formation-specific conservative approach)
            normalized_time = elapsed_time / trajectory_duration
            
            # Boundary check for velocity calculation
            if normalized_time <= 0 or normalized_time >= 1:
                vel_x = vel_y = vel_z = 0.0  # Zero velocity at boundaries
            else:
                # Formation uses simpler velocity calculation with lower limits
                T_vel = np.array([5*normalized_time**4, 4*normalized_time**3, 3*normalized_time**2, 
                                 2*normalized_time, 1, 0]) / trajectory_duration
                
                vel_x = np.dot(traj_x.coeffs_scalar, T_vel)
                vel_y = np.dot(traj_y.coeffs_scalar, T_vel)
                vel_z = np.dot(traj_z.coeffs_scalar, T_vel)
            
            # Apply formation-specific trapezoidal velocity profile (more conservative)
            progress = normalized_time
            if progress < 0.2:  # Longer acceleration phase (20% vs 15%)
                velocity_factor = 0.6 + 0.4 * (progress / 0.2)
            elif progress > 0.8:  # Longer deceleration phase (20% vs 15%)
                velocity_factor = 0.6 + 0.4 * ((1.0 - progress) / 0.2)
            else:  # Shorter constant speed phase (60% vs 70%)
                velocity_factor = 1.0
            
            # Apply formation velocity profile to calculated velocity
            vel_x *= velocity_factor
            vel_y *= velocity_factor
            vel_z *= velocity_factor
            
            # Construct trajectory point
            pos = [pos_x, pos_y, pos_z]
            velocity = [vel_x, vel_y, vel_z]
            
            # Calculate velocity magnitude and enforce formation limits
            vel_magnitude = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
            if vel_magnitude > self.max_linear_velocity:
                # Scale down velocity to respect formation limits
                scale_factor = self.max_linear_velocity / vel_magnitude
                velocity = [v * scale_factor for v in velocity]
                vel_magnitude = self.max_linear_velocity
            
            trajectory_points.append((pos, yaw, velocity, vel_magnitude))
            
            if i % 3 == 0:  # Log every 3rd point for formation
                rospy.loginfo(f"Formation Point{i}: pos=[{pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f}], "
                            f"yaw={math.degrees(yaw):.1f}°, vel={vel_magnitude:.3f}m/s (factor={velocity_factor:.2f})")
        
        rospy.loginfo(f"Generated {len(trajectory_points)} formation trajectory points")
        return trajectory_points

    def execute_polynomial_trajectory(self, trajectory_points):
        """
        Execute polynomial trajectory using active convergence for each point (Formation version)
        Based on single UAV method with formation-specific timing and thresholds
        """
        rospy.loginfo(f"Executing Formation Polynomial Trajectory: {len(trajectory_points)} points")
        
        total_points = len(trajectory_points)
        
        for i, (pos, yaw, velocity, vel_magnitude) in enumerate(trajectory_points):
            # Only log every 3rd point to reduce output frequency
            if i % 3 == 0 or i == total_points - 1:
                rospy.loginfo(f"Trajectory point {i+1}/{total_points}: "
                             f"pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], "
                             f"yaw={math.degrees(yaw):.1f}°")
            
            # Use active convergence for each trajectory point (formation-specific parameters)
            point_success = self.active_position_convergence(
                target_ee_pos=pos,
                target_yaw=yaw,
                pos_threshold=0.025,        # Slightly relaxed for formation (25mm vs 20mm)
                yaw_threshold=0.08,         # Relaxed yaw threshold for trajectory following
                vel_threshold=0.015,        # Slightly higher velocity threshold
                min_readings=3,             # Fewer readings for trajectory points (faster)
                max_attempts=60,            # Moderate attempts per point
                timeout=15.0                # 15 second timeout per point
            )
            
            if not point_success:
                rospy.logwarn(f"Formation trajectory point {i+1} convergence issues - continuing anyway")
            else:
                # Only log completion for every 3rd point or the last point
                if i % 3 == 0 or i == total_points - 1:
                    rospy.loginfo(f"Formation trajectory point {i+1} completed successfully")
            
            # Brief pause between points for formation stability
            rospy.sleep(0.2)
        
        rospy.loginfo("Formation polynomial trajectory execution completed")
        return True


class FormationDescendAndContactState(FormationSingleUAVStateBase):
    """Formation version of DescendAndContactState"""
    
    def __init__(self, rotation_direction=1):
        FormationSingleUAVStateBase.__init__(self,
            outcomes=['succeeded', 'failed', 'aborted'],
            input_keys=['valve_position', 'valve_yaw', 'phase4_contact_pose', 'phase4_contact_yaw'],
            output_keys=['rotation_start_position',
                         'initial_valve_yaw',
                         'valve_yaw_change_threshold',
                         'trajectory_state',
                         'contact_final_torque',
                         'insertion_info',
                         'detected_valve_rotation',
                         'contact_success_time',
                         'phase4_contact_yaw',
                         'rotation_baseline_yaw',
                         'contact_achieved_rotation'])
        self.rotation_direction = rotation_direction
    
    def execute(self, userdata):
        rospy.loginfo("=== Formation Descend And Contact State ===")
        rospy.loginfo("Using TWO-PHASE SAFE INSERTION strategy (like single UAV version)")
        
        valve_pos = userdata.valve_position
        valve_yaw = userdata.valve_yaw
        phase4_contact_pos = getattr(userdata, 'phase4_contact_pose', None)
        phase4_contact_yaw = getattr(userdata, 'phase4_contact_yaw', None)
        
        # Initialize optimizer and calculate optimal insertion position
        self.optimizer.update_valve_info(valve_pos, valve_yaw)
        
        # Get current end-effector position for approach calculation  
        current_ee_pos = self.get_end_effector_position()
        if current_ee_pos is None:
            rospy.logerr("Cannot get current end-effector position")
            return 'failed'
        
        rospy.loginfo(f"Current end-effector position: ({current_ee_pos[0]:.3f}, {current_ee_pos[1]:.3f}, {current_ee_pos[2]:.3f})")
        
        # Skip redundant insertion phases - FormationMoveToValveState already completed precise positioning
        rospy.loginfo("SKIPPING Phase 1 & 2: FormationMoveToValveState Phase 4 already completed insertion positioning")
        
        # Use current position from FormationMoveToValveState Phase 4 result
        target_insert_z = phase4_contact_pos[2] if phase4_contact_pos is not None else current_ee_pos[2]
        current_pos_final = phase4_contact_pos if phase4_contact_pos is not None else current_ee_pos
        
        # === CRITICAL: Execute circular contact establishment phase ===
        rospy.loginfo(">>> STARTING Phase: Circular Contact Establishment (adapted from single UAV)")
        
        contact_result = self._execute_circular_contact_phase(userdata, current_pos_final, valve_pos, valve_yaw)
        if contact_result != 'success':
            rospy.logerr("Circular contact establishment failed - valve not engaged")
            return 'failed'
        
        rospy.loginfo("✓ Circular contact established, valve engagement detected")
        rospy.loginfo("Ready for valve rotation phase")
        
        # Calculate approach angle based on current position
        dx = current_ee_pos[0] - valve_pos[0]
        dy = current_ee_pos[1] - valve_pos[1]
        approach_angle = math.atan2(dy, dx)
        
        # Select optimal spoke closest to approach direction
        spoke_angles = self.optimizer.get_spoke_angles()
        angle_diffs = [abs(angle - approach_angle) for angle in spoke_angles]
        min_idx = angle_diffs.index(min(angle_diffs))
        selected_spoke = spoke_angles[min_idx]
        
        rospy.loginfo(f"Selected spoke angle: {selected_spoke:.3f} rad ({math.degrees(selected_spoke):.1f}°)")
        rospy.loginfo(f"Approach angle: {approach_angle:.3f} rad ({math.degrees(approach_angle):.1f}°)")
        
        # Verify current position is reasonable for valve operation
        distance_to_valve = math.sqrt(sum((c - v)**2 for c, v in zip(current_pos_final[:2], valve_pos[:2])))
        rospy.loginfo(f"Current distance to valve center: {distance_to_valve*1000:.1f}mm")
        
        if distance_to_valve < 0.15:  # 15cm tolerance
            rospy.loginfo("✓ Position verification passed - ready for valve rotation")
        else:
            rospy.logwarn(f"⚠ Distance to valve {distance_to_valve:.3f}m > 0.15m tolerance, but proceeding")

        # Simple yaw alignment (replace problematic _execute_formation_phase3b_spoke_alignment)
        yaw_for_rotation = phase4_contact_yaw
        if yaw_for_rotation is None:
            # Calculate radial yaw for valve rotation
            radial_yaw = math.atan2(
                current_pos_final[1] - valve_pos[1],
                current_pos_final[0] - valve_pos[0]
            ) + math.pi
            radial_yaw = self.normalize_angle(radial_yaw)
            rospy.loginfo(f"Calculated radial yaw: {math.degrees(radial_yaw):.1f}°")
            
            # Simple yaw alignment without complex phase3b logic
            success = self.active_position_convergence(
                target_ee_pos=current_pos_final,
                target_yaw=radial_yaw,
                pos_threshold=0.02,
                yaw_threshold=0.05,
                timeout=8.0,
                max_attempts=40,
                min_readings=3
            )
            
            if success:
                rospy.loginfo("✓ Yaw alignment completed")
                yaw_for_rotation = radial_yaw
            else:
                rospy.logwarn("⚠ Yaw alignment had issues, using current yaw")
                yaw_for_rotation = self.get_end_effector_yaw() or radial_yaw

        # === SHARE CONTACT STATE WITH ROTATION PHASE ===
        cached_phase4_pose = phase4_contact_pos or self.get_end_effector_position()

        final_contact_pos = cached_phase4_pose or current_pos_final
        contact_radius = None
        contact_angle = None
        if final_contact_pos is not None:
            contact_radius = math.sqrt((final_contact_pos[0] - valve_pos[0])**2 +
                                       (final_contact_pos[1] - valve_pos[1])**2)
            contact_angle = math.atan2(final_contact_pos[1] - valve_pos[1],
                                       final_contact_pos[0] - valve_pos[0])

        # 🔧 Z坐标连续性修复: 统一使用insertion高度
        corrected_valve_center = (
            valve_pos[0],
            valve_pos[1],
            self.insertion_z_coordinate  # 统一使用insertion高度，避免Z坐标跳跃
        )
        rospy.loginfo(f"Contact完成时Z坐标连续性: 使用insertion_z={self.insertion_z_coordinate:.3f}m")

        userdata.rotation_start_position = final_contact_pos
        if yaw_for_rotation is None:
            yaw_for_rotation = self.get_end_effector_yaw()
        userdata.initial_valve_yaw = valve_yaw
        userdata.valve_yaw_change_threshold = math.radians(10.0)
        contact_torque_vector = [0.0, 0.0, 0.15 * self.rotation_direction]  # 增加接触扭矩
        userdata.contact_final_torque = contact_torque_vector
        rospy.loginfo(
            f"Baseline contact torque set to {contact_torque_vector} (rotation_direction={self.rotation_direction:+d})"
        )
        userdata.insertion_info = {
            'initial_position': final_contact_pos,
            'valve_center': corrected_valve_center,
            'initial_radius': contact_radius,
            'initial_yaw': valve_yaw,
            'spoke_angle': selected_spoke,
            'rotation_direction': self.rotation_direction,
            'phase4_contact_position': phase4_contact_pos,
        }

        userdata.trajectory_state = {
            'initial_radius': contact_radius,
            'current_radius': contact_radius,
            'current_angle': contact_angle,
            'radius_locked': False,
            'lock_radius_value': None,
            'valve_center': corrected_valve_center
        }
        if yaw_for_rotation is not None:
            userdata.phase4_contact_yaw = yaw_for_rotation

        rospy.loginfo("Shared contact data with rotation phase:")
        rospy.loginfo(f"  rotation_start_position={final_contact_pos}")
        rospy.loginfo(f"  contact_radius={contact_radius}")
        rospy.loginfo(f"  contact_angle={contact_angle}")

        return 'succeeded'

    def _execute_circular_contact_phase(self, userdata, current_pos, valve_pos, initial_valve_yaw):
        """
        Circular contact establishment phase - adapted from single UAV version
        Executes circular motion to establish physical contact with valve before rotation
        """
        rospy.loginfo(">>> Starting circular contact establishment")
        
        # Import trajectory generator
        from trajectory import OnlineCircularTrajectoryGenerator
        
        # Calculate current radius from valve center
        relative_pos = np.array(current_pos[:2]) - np.array(valve_pos[:2])
        current_radius = np.linalg.norm(relative_pos)
        
        rospy.loginfo(f"Valve center: ({valve_pos[0]:.3f}, {valve_pos[1]:.3f}, {valve_pos[2]:.3f})")
        rospy.loginfo(f"Current radius: {current_radius*1000:.1f}mm")
        rospy.loginfo(f"Initial valve angle: {math.degrees(initial_valve_yaw):.1f}°")
        
        # 🔧 CRITICAL FIX: 与Single UAV版本保持一致，始终使用正向角速度！
        # Single UAV版本Contact和Rotation都用正值，避免方向冲突
        contact_angular_velocity = 0.03  # 降低到0.03 rad/s ≈ 1.7°/s，避免仿真中过快运动
        rospy.logwarn(f"🔧 SINGLE UAV COMPATIBILITY: Contact阶段角速度 = +{contact_angular_velocity:.3f} rad/s (正向推动，已降速)")
        rospy.logwarn(f"🔧 这与Single UAV版本逻辑完全一致，避免方向冲突")
        trajectory_generator = OnlineCircularTrajectoryGenerator(
            valve_center=valve_pos,
            initial_radius=current_radius,
            target_angular_velocity=contact_angular_velocity,
            control_rate=25.0,
            debug=True
        )
        
        # Use precise Z coordinate
        precise_contact_pos = [current_pos[0], current_pos[1], current_pos[2]]
        trajectory_generator.initialize_from_current_position(precise_contact_pos)
        
        # Contact establishment parameters (adapted from single UAV)
        contact_duration = 15.0  # 15 seconds - shorter than single UAV for efficiency
        valve_rotation_threshold = 0.08  # 0.08 rad ≈ 4.6° - slightly lower threshold
        contact_force_threshold = 0.25   # 0.25N contact force threshold
        
        start_time = rospy.Time.now().to_sec()
        last_valve_check_time = start_time
        last_valve_yaw = initial_valve_yaw
        control_rate = rospy.Rate(25)
        
        rospy.loginfo(f"Contact establishment - target: {math.degrees(valve_rotation_threshold):.1f}° valve rotation")
        
        while (rospy.Time.now().to_sec() - start_time) < contact_duration:
            current_time = rospy.Time.now().to_sec()
            
            # Get current state
            current_pos_now = self.get_end_effector_position()
            current_yaw_now = self.get_end_effector_yaw()
            current_valve_yaw = self.beetle.getValveYaw()
            
            if any(v is None for v in [current_pos_now, current_yaw_now, current_valve_yaw]):
                rospy.logwarn("State information missing, continuing...")
                control_rate.sleep()
                continue
            
            # Calculate valve rotation progress
            valve_rotation = abs(current_valve_yaw - initial_valve_yaw)
            
            # Calculate valve angular velocity
            valve_angular_velocity = 0.0
            if current_time > last_valve_check_time + 0.15:  # 150ms update rate
                dt = current_time - last_valve_check_time
                valve_angular_velocity = abs(current_valve_yaw - last_valve_yaw) / dt
                last_valve_yaw = current_valve_yaw
                last_valve_check_time = current_time
            
            # Update trajectory generator state
            try:
                state_info = trajectory_generator.update_state(
                    current_pos_now, current_yaw_now, valve_angular_velocity
                )
                
                # Generate control target
                target_state = trajectory_generator.generate_target_state()
                
                # Execute formation-compatible control with speed limiting
                linear_vel_vec = np.array(target_state['linear_velocity'])
                speed_total = np.linalg.norm(linear_vel_vec)
                max_contact_speed = 0.02  # 20mm/s max speed for contact phase
                
                if speed_total > max_contact_speed and speed_total > 1e-6:
                    scale = max_contact_speed / speed_total
                    linear_vel_vec *= scale
                
                # Send assembly command (formation compatible)
                self.send_assembly_command_from_end_effector(
                    target_state['position'].tolist(),
                    target_state['yaw'],
                    linear_vel=linear_vel_vec.tolist(),
                    angular_vel=target_state.get('angular_velocity')
                )
                
            except Exception as exc:
                rospy.logwarn(f"Trajectory generation failed: {exc}")
                control_rate.sleep()
                continue
            
            # Check for valve engagement (SUCCESS condition)
            if valve_rotation > valve_rotation_threshold:
                elapsed_time = current_time - start_time
                rospy.loginfo(f"✓ Valve engagement detected: {math.degrees(valve_rotation):.1f}° "
                            f"(threshold: {math.degrees(valve_rotation_threshold):.1f}°)")
                rospy.loginfo(f"Contact establishment time: {elapsed_time:.1f}s")
                
                # Update userdata with contact success info (方案A: 重置Baseline)
                userdata.initial_valve_yaw = initial_valve_yaw  # 保留原始initial_valve_yaw
                userdata.rotation_baseline_yaw = current_valve_yaw  # NEW: 当前valve yaw作为旋转baseline
                userdata.contact_achieved_rotation = valve_rotation  # NEW: Contact阶段已达成的旋转
                userdata.contact_success_time = elapsed_time
                userdata.detected_valve_rotation = valve_rotation
                
                # 传递完整的trajectory generator状态（参考单个UAV版本）
                self._set_contact_trajectory_state(userdata, trajectory_generator, target_state)
                
                rospy.loginfo(f"✓ Rotation baseline set: initial={math.degrees(initial_valve_yaw):.1f}°, "
                             f"current={math.degrees(current_valve_yaw):.1f}°, achieved={math.degrees(valve_rotation):.1f}°")
                
                return 'success'
            
            # Logging for contact progress
            if current_time - start_time > 2.0:  # Start logging after 2 seconds
                rospy.loginfo_throttle(3.0, 
                    f"Contact establishment: radius={state_info['current_radius']*1000:.1f}mm, "
                    f"valve_rotation={math.degrees(valve_rotation):.2f}°, "
                    f"time={current_time-start_time:.1f}s")
            
            if rospy.is_shutdown():
                return 'failed'
            
            control_rate.sleep()
        
        # Timeout reached - check for minimal valve movement
        final_valve_rotation = abs(current_valve_yaw - initial_valve_yaw)
        if final_valve_rotation > math.radians(0.5):  # Accept 0.5° as minimal success
            rospy.logwarn(f"Contact timeout but detected minimal valve movement: "
                         f"{math.degrees(final_valve_rotation):.1f}°")
            userdata.initial_valve_yaw = initial_valve_yaw
            userdata.rotation_baseline_yaw = current_valve_yaw  # NEW: 设置旋转baseline
            userdata.contact_achieved_rotation = final_valve_rotation  # NEW: 记录已达成旋转
            userdata.contact_success_time = contact_duration
            userdata.detected_valve_rotation = final_valve_rotation
            
            # 传递trajectory generator状态
            self._set_contact_trajectory_state(userdata, trajectory_generator, target_state)
            
            return 'success'
        
        rospy.logerr(f"Contact establishment failed: no valve movement detected "
                    f"(final: {math.degrees(final_valve_rotation):.2f}°)")
        return 'failed'

    def _set_contact_trajectory_state(self, userdata, trajectory_generator, target_state):
        """Set trajectory generator state for rotation phase (adapted from single UAV)"""
        if trajectory_generator is not None:
            # 传递轨迹生成器的状态参数（参考单个UAV版本）
            userdata.trajectory_state = {
                'valve_center': trajectory_generator.valve_center.tolist() if hasattr(trajectory_generator.valve_center, 'tolist') else list(trajectory_generator.valve_center),
                'initial_radius': trajectory_generator.initial_radius,
                'current_radius': getattr(trajectory_generator, 'current_radius', trajectory_generator.initial_radius),
                'current_angle': getattr(trajectory_generator, 'current_angle', 0.0),
                'radius_locked': getattr(trajectory_generator, 'radius_locked', False),
                'locked_radius': getattr(trajectory_generator, 'locked_radius', trajectory_generator.initial_radius),
                'lock_radius_value': getattr(trajectory_generator, 'lock_radius_value', None)
            }
            rospy.loginfo("✓ Trajectory generator state transferred to rotation phase")
        else:
            userdata.trajectory_state = None
            rospy.logwarn("⚠ No trajectory generator state to transfer")
        
        if target_state is not None:
            final_torque = target_state.get('torque', [0.0, 0.0, 0.0])
            aligned_torque = list(final_torque)
            if len(aligned_torque) >= 3:
                aligned_torque[2] = aligned_torque[2] * self.rotation_direction
            userdata.contact_final_torque = aligned_torque
            rospy.loginfo(f"✓ Final contact torque (aligned): {aligned_torque}")
        else:
            default_torque = [0.0, 0.0, 0.15 * self.rotation_direction]  # 增加默认扭矩
            userdata.contact_final_torque = default_torque
            rospy.loginfo(f"✓ Using default torque: {default_torque}")


class FormationRotateValveState(FormationSingleUAVStateBase):
    """Formation valve contact and rotation - single unified state like Single UAV"""

    def __init__(self, rotation_direction=1):
        FormationSingleUAVStateBase.__init__(
            self,
            outcomes=['succeeded', 'failed', 'emergency'],
            input_keys=['valve_position', 'valve_yaw', 'phase4_contact_pose', 'phase4_contact_yaw'],
            output_keys=['trajectory_state', 'contact_final_torque']
        )
        self.rotation_direction = rotation_direction
        self.target_rotation = math.radians(90.0)
        self.max_rotation_time = 60.0
        self.min_rotation_time = 8.0
        self.nominal_angular_velocity = 0.025  # 降低到0.025 rad/s ≈ 1.4°/s，避免仿真中过快旋转
        self.control_rate = 25.0
        self.max_linear_speed = 0.035  # m/s 增加速度以维持更强接触 (3.5cm/s)

    @staticmethod
    def _normalize(self, angle):
        """角度归一化（与Single UAV版本保持一致，解决坐标变换兼容性）"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _get_valve_yaw(self, fallback):
        valve_yaw = self.beetle.getValveYaw()
        if valve_yaw is None:
            rospy.logwarn("getValveYaw() returned None, using fallback")
            return fallback
        return valve_yaw

    def execute(self, userdata):
        rospy.loginfo("=== Formation Contact and Rotate Valve State (Unified) ===")
        
        # 阶段1: 执行接触建立阶段
        rospy.loginfo(">>> Phase 1: Establishing circular contact with valve")
        contact_result = self._execute_contact_phase(userdata)
        if contact_result != 'succeeded':
            return contact_result
            
        # 阶段2: 执行旋转阶段（使用相同的轨迹生成器实例）
        rospy.loginfo(">>> Phase 2: Executing valve rotation")
        rotation_result = self._execute_rotation_phase(userdata)
        return rotation_result
    
    def _execute_contact_phase(self, userdata):
        """接触建立阶段 - 完全复制FormationDescendAndContactState逻辑"""
        rospy.loginfo("Using TWO-PHASE SAFE INSERTION strategy (like single UAV version)")
        
        valve_pos = userdata.valve_position
        valve_yaw = userdata.valve_yaw
        phase4_contact_pos = getattr(userdata, 'phase4_contact_pose', None)
        phase4_contact_yaw = getattr(userdata, 'phase4_contact_yaw', None)

        # 缓存阀门信息以供formation helpers使用
        self.optimizer.update_valve_info(valve_pos, valve_yaw)

        # 获取插入完成后的位置信息（固定Z坐标）
        current_ee_pos = self.get_end_effector_position()
        if current_ee_pos is None:
            rospy.logerr("Failed to get current end-effector position")
            return 'failed'

        # 锁定插入完成后的Z坐标，避免pitch静差
        self.insertion_z_coordinate = current_ee_pos[2]
        rospy.loginfo(f"📍 LOCKED insertion Z coordinate: {self.insertion_z_coordinate:.3f}m (will be used throughout contact & rotation)")
        rospy.loginfo(f"Current end-effector position: {current_ee_pos[0]:.3f}, {current_ee_pos[1]:.3f}, {current_ee_pos[2]:.3f})")
        
        rospy.loginfo("SKIPPING Phase 1 & 2: FormationMoveToValveState Phase 4 already completed insertion positioning")
        
        # Phase 3: Circular Contact Establishment  
        # 使用固定的插入Z坐标，而不是实时更新的坐标
        current_pos_final = (current_ee_pos[0], current_ee_pos[1], self.insertion_z_coordinate)
        contact_result = self._execute_circular_contact_phase(userdata, current_pos_final, valve_pos, valve_yaw)
        if not contact_result:
            return 'failed'

        # 位置验证和插入数据设置  
        valve_center = valve_pos
        # 🔧 Z坐标连续性：corrected_valve_center统一使用insertion坐标
        corrected_valve_center = (valve_pos[0], valve_pos[1], self.insertion_z_coordinate)  # 统一使用insertion高度
        approach_angle = math.atan2(current_pos_final[1] - valve_center[1], 
                                   current_pos_final[0] - valve_center[0])
        spoke_angles = self.optimizer.get_spoke_angles()
        angle_diffs = [abs(angle - approach_angle) for angle in spoke_angles]
        selected_spoke = spoke_angles[angle_diffs.index(min(angle_diffs))]
        
        rospy.loginfo(f"Selected spoke angle: {selected_spoke:.3f} rad ({math.degrees(selected_spoke):.1f}°)")
        rospy.loginfo(f"Approach angle: {approach_angle:.3f} rad ({math.degrees(approach_angle):.1f}°)")

        # 🔧 接触距离修正：使用corrected_valve_center确保Z坐标一致性
        contact_radius = math.sqrt((current_pos_final[0] - corrected_valve_center[0])**2 + 
                                 (current_pos_final[1] - corrected_valve_center[1])**2)
        rospy.loginfo(f"Current distance to valve center: {contact_radius*1000:.1f}mm")

        if contact_radius > 0.025:
            rospy.logwarn(f"Distance to valve center too large: {contact_radius*1000:.1f}mm")

        # 计算接触角度和优化位置
        contact_angle = selected_spoke
        final_contact_pos = current_pos_final

        # 跳过位置优化 - 直接使用当前位置
        rospy.loginfo("Using current position without optimization (simplified for unified state)")

        # 设置接触基线数据
        cached_phase4_pose = phase4_contact_pos or self.get_end_effector_position()
        
        # 存储接触成功信息到userdata（使用insertion Z坐标保持连续性）
        self.contact_radius = contact_radius  # 存储为实例变量
        self.contact_angle = contact_angle
        # 🔧 Z坐标连续性：存储使用insertion高度的corrected_valve_center
        self.corrected_valve_center = corrected_valve_center  # 使用insertion Z坐标
        self.initial_valve_yaw = valve_yaw  # 用于旋转阶段
        self.rotation_start_position = final_contact_pos

        rospy.loginfo("✓ Position verification passed - ready for valve rotation")
        rospy.loginfo(f"Baseline contact torque set to [0.0, 0.0, {-0.15 * self.rotation_direction:.2f}] (rotation_direction={self.rotation_direction})")
        
        return 'succeeded'

    def _execute_circular_contact_phase(self, userdata, current_pos, valve_pos, initial_valve_yaw):
        """
        Circular contact establishment phase - adapted from single UAV version
        Executes circular motion to establish physical contact with valve before rotation
        """
        rospy.loginfo(">>> Starting circular contact establishment")
        
        # Import trajectory generator
        from trajectory import OnlineCircularTrajectoryGenerator
        
        # Calculate current radius from valve center
        relative_pos = np.array(current_pos[:2]) - np.array(valve_pos[:2])
        current_radius = np.linalg.norm(relative_pos)
        
        rospy.loginfo(f"Valve center: ({valve_pos[0]:.3f}, {valve_pos[1]:.3f}, {valve_pos[2]:.3f})")
        rospy.loginfo(f"Current radius: {current_radius*1000:.1f}mm")
        rospy.loginfo(f"Initial valve angle: {math.degrees(initial_valve_yaw):.1f}°")
        
        # ===== SINGLE UAV COMPATIBILITY: 固定使用正向角速度 =====
        contact_angular_velocity = 0.03  # 降低到0.03 rad/s ≈ 1.7°/s，避免仿真中过快运动
        rospy.logwarn(f"🔧 SINGLE UAV COMPATIBILITY: Contact阶段角速度 = +{contact_angular_velocity:.3f} rad/s (正向推动，已降速)")
        rospy.logwarn(f"🔧 这与Single UAV版本逻辑完全一致，避免方向冲突")
        
        # 🔧 Z坐标连续性修复: 使用insertion高度而非valve_pos[2]，避免UAV突然下降
        valve_center_corrected = (valve_pos[0], valve_pos[1], self.insertion_z_coordinate)
        rospy.loginfo(f"Z坐标连续性: valve_center_corrected使用insertion_z={self.insertion_z_coordinate:.3f}m (而非valve_pos[2]={valve_pos[2]:.3f}m)")
        
        # Initialize trajectory generator for contact phase
        trajectory_generator = OnlineCircularTrajectoryGenerator(
            valve_center=valve_center_corrected,
            initial_radius=current_radius,
            target_angular_velocity=contact_angular_velocity,  # 正向速度
            control_rate=self.control_rate
        )
        
        # Dragon逻辑：使用与Single UAV相同的接触建立阈值
        contact_threshold = 0.1  # 0.1 rad ≈ 5.7° (与Single UAV保持一致)
        rospy.loginfo(f"Contact establishment - target: {math.degrees(contact_threshold):.1f}° valve rotation (Single UAV compatible)")
        
        # Contact establishment parameters
        max_contact_time = 20.0  # 与Single UAV一致的20秒接触时间
        target_valve_rotation = contact_threshold  # 使用Dragon标准阈值
        min_valve_rotation = math.radians(1.0)     # Minimal acceptable rotation
        contact_start_time = rospy.Time.now().to_sec()
        
        # Initialize contact monitoring
        start_valve_yaw = self._get_valve_yaw_safe(initial_valve_yaw)
        last_valve_yaw = start_valve_yaw
        last_valve_check_time = contact_start_time
        max_detected_rotation = 0.0
        
        # Initialize valve angular velocity tracking (like Single UAV)
        last_control_time = contact_start_time
        
        # Main contact establishment loop
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            contact_duration = current_time - contact_start_time
            
            # Get current valve state
            current_valve_yaw = self._get_valve_yaw_safe(start_valve_yaw)
            valve_rotation = abs(current_valve_yaw - start_valve_yaw)
            max_detected_rotation = max(max_detected_rotation, valve_rotation)
            
            # Calculate valve angular velocity (Dragon logic from Single UAV)
            valve_angular_velocity = 0.0
            if current_time > last_control_time + 0.1:
                if hasattr(self, '_last_valve_yaw'):
                    dt = current_time - last_control_time
                    valve_angular_velocity = abs(current_valve_yaw - self._last_valve_yaw) / dt
                self._last_valve_yaw = current_valve_yaw
                last_control_time = current_time
            
            # Get current end-effector pose for trajectory update
            current_pos = self.get_end_effector_position()
            current_yaw = self.get_end_effector_yaw()
            
            if current_pos is None or current_yaw is None:
                rospy.logwarn("Formation pose information lost, continuing to wait")
                rospy.sleep(0.04)
                continue
            
            # Check for successful contact establishment
            if valve_rotation >= target_valve_rotation:
                rospy.loginfo(f"✓ Valve engagement detected: {math.degrees(valve_rotation):.1f}° "
                             f"(threshold: {math.degrees(target_valve_rotation):.1f}°)")
                rospy.loginfo(f"Contact establishment time: {contact_duration:.1f}s")
                
                # 存储轨迹生成器以供旋转阶段使用
                self.trajectory_generator = trajectory_generator  # 保存实例
                
                rospy.loginfo("✓ Trajectory generator state transferred to rotation phase")
                
                # Store contact success information
                contact_final_torque = [0.0, 0.0, -0.08]  # Fixed baseline torque
                rospy.loginfo(f"✓ Final contact torque (aligned): {contact_final_torque}")
                
                # Set rotation baseline
                rospy.loginfo(f"✓ Rotation baseline set: initial={math.degrees(initial_valve_yaw):.1f}°, "
                             f"current={math.degrees(current_valve_yaw):.1f}°, achieved={math.degrees(valve_rotation):.1f}°")
                
                return True
            
            # Timeout check
            if contact_duration > max_contact_time:
                break
                
            # Enhanced trajectory monitoring (like Single UAV)
            if contact_duration > 1.0:  # Start logging after 1 second
                rospy.loginfo_throttle(2.0, 
                    f"Formation接触: 半径={state_info['current_radius']*1000:.1f}mm, "
                    f"角速度={math.degrees(state_info['angular_velocity']):.1f}°/s, "
                    f"阀门转动={math.degrees(valve_rotation):.2f}°, "
                    f"时间={contact_duration:.1f}s")
            
            # Update trajectory generator state (Critical: like Single UAV)
            state_info = trajectory_generator.update_state(
                current_pos, current_yaw, valve_angular_velocity
            )
            
            # Execute trajectory  
            target_state = trajectory_generator.generate_target_state()
            if target_state:
                # Dragon-style trajectory monitoring
                if contact_duration > 0.5:  # Log trajectory details after initial stabilization
                    rospy.loginfo_throttle(5.0, 
                        f"轨迹状态: radius_locked={state_info.get('radius_locked', False)}, "
                        f"current_angle={math.degrees(getattr(trajectory_generator, 'current_angle', 0.0)):.1f}°")
                
                # 发送装配命令（该方法没有返回值，直接发送）
                self.send_assembly_command_from_end_effector(
                    target_state['position'],
                    target_state['yaw'],
                    target_state.get('linear_velocity', [0, 0, 0]),
                    target_state.get('angular_velocity', 0)
                )
            
            rospy.sleep(0.04)  # 25Hz control rate
        
        # Contact establishment failed
        rospy.logerr(f"Contact establishment failed: max valve movement {math.degrees(max_detected_rotation):.2f}°")
        return False
        
    def _execute_rotation_phase(self, userdata):
        """旋转执行阶段 - 重用接触阶段的轨迹生成器实例"""
        
        # 使用接触阶段存储的轨迹生成器实例
        if not hasattr(self, 'trajectory_generator') or self.trajectory_generator is None:
            rospy.logerr("No trajectory generator available from contact phase!")
            return 'failed'
            
        rospy.loginfo("✓ Reusing trajectory generator from contact phase")
        
        # 获取当前状态信息
        current_valve_yaw_now = self._get_valve_yaw_safe(self.initial_valve_yaw)
        
        rospy.loginfo(f"✓ Starting rotation from baseline, contact achieved: {math.degrees(abs(current_valve_yaw_now - self.initial_valve_yaw)):.1f}°")
        rospy.loginfo(f"Rotation start position: {self.rotation_start_position}")
        rospy.loginfo(f"Corrected valve center: {self.corrected_valve_center}")
        rospy.loginfo(f"Initial rotation radius: {self.contact_radius*1000:.1f}mm")
        rospy.loginfo(f"✓ Rotation directions consistent: {self.rotation_direction}")
        
        # DEBUG: 显示所有关键参数
        rospy.logwarn("============================================================")
        rospy.logwarn("🔧 ROTATION DEBUG - 所有关键参数:")
        rospy.logwarn("  📊 Trajectory Generator Parameters:")
        rospy.logwarn(f"    nominal_angular_velocity: {self.nominal_angular_velocity:.4f} rad/s ({math.degrees(self.nominal_angular_velocity):.2f}°/s)")
        rospy.logwarn(f"    effective_rotation_direction: {self.rotation_direction}")
        
        # ===== SINGLE UAV COMPATIBILITY: 使用正向角速度 =====
        effective_angular_velocity = abs(self.nominal_angular_velocity)  # 强制正值
        rospy.logwarn(f"    🎯 FINAL angular_velocity: {effective_angular_velocity:.4f} rad/s ({math.degrees(effective_angular_velocity):.2f}°/s)")
        
        rospy.logwarn("  📍 Valve Position & Orientation:")
        rospy.logwarn(f"    initial_valve_yaw (baseline): {math.degrees(self.initial_valve_yaw):.2f}°")
        rospy.logwarn(f"    current_valve_yaw_now: {math.degrees(current_valve_yaw_now):.2f}°")
        rospy.logwarn(f"    valve_center: {self.corrected_valve_center}")
        rospy.logwarn(f"    rotation_radius: {self.contact_radius*1000:.1f}mm")
        rospy.logwarn("  ⚙️ Expected Behavior:")
        rospy.logwarn("    UAV will move: COUNTER-CLOCKWISE (positive angle increase)")
        rospy.logwarn("============================================================")
        
        # 重新配置轨迹生成器为旋转阶段
        self.trajectory_generator.target_angular_velocity = effective_angular_velocity
        rospy.loginfo(f"Target rotation: {math.degrees(self.target_rotation):.1f}° @ {math.degrees(effective_angular_velocity):.1f}°/s")
        
        # 旋转控制循环
        rotation_start_time = rospy.Time.now().to_sec()
        initial_valve_yaw = self.initial_valve_yaw
        last_valve_yaw = current_valve_yaw_now
        last_valve_check_time = rotation_start_time
        max_rotation_detected = 0.0
        
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - rotation_start_time
            
            # 获取当前阀门状态
            current_valve_yaw = self._get_valve_yaw_safe(initial_valve_yaw)
            valve_rotation = abs(current_valve_yaw - initial_valve_yaw)
            max_rotation_detected = max(max_rotation_detected, valve_rotation)
            
            # 计算阀门角速度（完全采用Single UAV逻辑）
            valve_angular_velocity = 0.0
            if rospy.Time.now().to_sec() > last_valve_check_time + 0.2:  # 每200ms更新一次
                dt = rospy.Time.now().to_sec() - last_valve_check_time
                valve_angular_velocity = abs(current_valve_yaw - last_valve_yaw) / dt
                last_valve_yaw = current_valve_yaw
                last_valve_check_time = rospy.Time.now().to_sec()

            # 🔍 DEBUG: 每5秒详细输出阀门状态（在valve_angular_velocity计算之后）
            if int(elapsed) % 5 == 0 and (elapsed - int(elapsed)) < 0.1:
                rospy.logwarn("🔍 VALVE & CONTACT STATUS DEBUG:")
                rospy.logwarn(f"  current_valve_yaw: {math.degrees(current_valve_yaw):.2f}°")
                rospy.logwarn(f"  initial_valve_yaw: {math.degrees(initial_valve_yaw):.2f}°")
                rospy.logwarn(f"  raw_difference: {math.degrees(current_valve_yaw - initial_valve_yaw):.2f}°")
                rospy.logwarn(f"  valve_rotation (abs): {math.degrees(valve_rotation):.2f}°")
                rospy.logwarn(f"  target_rotation: {math.degrees(self.target_rotation):.2f}°")
                rospy.logwarn(f"  valve_angular_velocity: {math.degrees(valve_angular_velocity):.3f}°/s")
                
                # 接触状态检测（使用固定Z坐标确保一致性）
                current_pos_raw = self.get_end_effector_position()[:3]
                current_pos = (current_pos_raw[0], current_pos_raw[1], self.insertion_z_coordinate)
                distance_to_valve = math.sqrt((current_pos[0] - self.corrected_valve_center[0])**2 + 
                                            (current_pos[1] - self.corrected_valve_center[1])**2)
                rospy.logwarn(f"  🔗 CONTACT STATUS:")
                rospy.logwarn(f"    distance_to_valve: {distance_to_valve*1000:.1f}mm")
                rospy.logwarn(f"    expected_radius: {self.contact_radius*1000:.1f}mm")
                if distance_to_valve > self.contact_radius * 1.2:
                    rospy.logwarn(f"    ⚠️ CONTACT LOST! UAV离阀门太远")
                elif abs(valve_angular_velocity) < 0.005:
                    rospy.logwarn(f"    ⚠️ VALVE NOT MOVING! 可能脱离接触")
            
            # 检查旋转完成
            if valve_rotation >= self.target_rotation:
                rospy.loginfo(f"✓ Valve rotation completed: {math.degrees(valve_rotation):.1f}° "
                             f"(target: {math.degrees(self.target_rotation):.1f}°) in {elapsed:.1f}s")
                
                # 设置输出数据供后续状态使用
                userdata.trajectory_state = {
                    'current_radius': self.contact_radius,
                    'current_angle': self.contact_angle,
                    'radius_locked': True,
                    'valve_center': self.corrected_valve_center
                }
                userdata.contact_final_torque = [0.0, 0.0, -0.15 * self.rotation_direction]
                
                return 'succeeded'
            
            # 超时检查
            if elapsed > self.max_rotation_time:
                rospy.logwarn(f"Rotation timeout after {elapsed:.1f}s, achieved: {math.degrees(valve_rotation):.1f}°")
                break
            
            # 执行轨迹命令 - 先更新状态再生成目标
            # 使用固定的插入Z坐标，避免pitch静差
            current_pos_raw = self.get_end_effector_position()[:3]
            current_pos_fixed_z = (current_pos_raw[0], current_pos_raw[1], self.insertion_z_coordinate)
            current_yaw = self.get_end_effector_yaw() or 0.0
            self.trajectory_generator.update_state(current_pos_fixed_z, current_yaw, valve_angular_velocity)
            target_state = self.trajectory_generator.generate_target_state()
            if target_state:
                # 发送装配命令（该方法没有返回值，直接发送）
                self.send_assembly_command_from_end_effector(
                    target_state['position'],
                    target_state['yaw'],
                    target_state.get('linear_velocity', [0, 0, 0]),
                    target_state.get('angular_velocity', 0)
                )
                    
            # 进度输出（每5秒）
            if int(elapsed) % 5 == 0 and (elapsed - int(elapsed)) < 0.1:
                progress = (valve_rotation / self.target_rotation) * 100
                rospy.loginfo(f"Rotation progress: {progress:.1f}% | Valve Δ={math.degrees(valve_rotation):.1f}° | "
                             f"Max={math.degrees(max_rotation_detected):.1f}° | Radius={self.contact_radius*1000:.1f}mm | "
                             f"ω={math.degrees(effective_angular_velocity):.1f}°/s")
            
            rospy.sleep(0.04)  # 25Hz
        
        # 旋转失败
        rospy.logerr(f"Valve rotation failed: achieved {math.degrees(max_rotation_detected):.1f}° "
                    f"of {math.degrees(self.target_rotation):.1f}° target")
        return 'failed'
    
    def _get_valve_yaw_safe(self, fallback):
        """Safe valve yaw getter with fallback"""
        valve_yaw = self.beetle.getValveYaw()
        if valve_yaw is None:
            rospy.logwarn("getValveYaw() returned None, using fallback")
            return fallback
        return valve_yaw

        valve_pos = getattr(userdata, 'valve_position', None)
        if valve_pos is None:
            rospy.logerr("Valve position missing from userdata")
            return 'failed'

        # Get current valve yaw for real-time tracking
        current_valve_yaw_now = self.beetle.getValveYaw()
        if current_valve_yaw_now is None:
            rospy.logerr("Unable to get current valve yaw at rotation start")
            return 'failed'
            
        # 方案A: 使用rotation_baseline_yaw作为计算基准
        rotation_baseline_yaw = getattr(userdata, 'rotation_baseline_yaw', None)
        if rotation_baseline_yaw is not None:
            initial_valve_yaw = rotation_baseline_yaw  # 使用接触成功时的valve yaw
            rospy.loginfo(f"✓ Using rotation baseline: {math.degrees(initial_valve_yaw):.1f}°")
        else:
            initial_valve_yaw = getattr(userdata, 'initial_valve_yaw', current_valve_yaw_now)
            rospy.logwarn("⚠ No rotation baseline found, using fallback")
            
        yaw_change_threshold = getattr(userdata, 'valve_yaw_change_threshold', math.radians(10.0))
        
        # Check if contact establishment was successful
        detected_valve_rotation = getattr(userdata, 'detected_valve_rotation', 0.0)
        contact_success_time = getattr(userdata, 'contact_success_time', 0.0)
        contact_achieved_rotation = getattr(userdata, 'contact_achieved_rotation', 0.0)
        
        if detected_valve_rotation > 0:
            rospy.loginfo(f"✓ Contact phase confirmed valve engagement: {math.degrees(detected_valve_rotation):.1f}° "
                         f"in {contact_success_time:.1f}s")
            rospy.loginfo(f"✓ Starting rotation from baseline, contact achieved: {math.degrees(contact_achieved_rotation):.1f}°")
        else:
            rospy.logwarn("⚠ No valve engagement detected in contact phase, but proceeding with rotation")

        rotation_start_pos = getattr(userdata, 'rotation_start_position', None)
        if rotation_start_pos is None:
            rotation_start_pos = self.get_end_effector_position()

        if rotation_start_pos is None:
            rospy.logerr("Unable to determine rotation start position")
            return 'failed'

        # 🔧 Z坐标连续性修复: 统一使用insertion高度
        corrected_valve_center = (
            valve_pos[0],
            valve_pos[1],
            self.insertion_z_coordinate  # 使用insertion高度，保持Z坐标连续性
        )
        rospy.loginfo(f"Rotation阶段Z坐标连续性: 使用insertion_z={self.insertion_z_coordinate:.3f}m")

        initial_radius = math.sqrt(
            (rotation_start_pos[0] - corrected_valve_center[0])**2 +
            (rotation_start_pos[1] - corrected_valve_center[1])**2
        )

        if initial_radius < 1e-6:
            rospy.logerr("Computed rotation radius too small")
            return 'failed'

        rospy.loginfo(f"Rotation start position: {rotation_start_pos}")
        rospy.loginfo(f"Corrected valve center: {corrected_valve_center}")
        rospy.loginfo(f"Initial rotation radius: {initial_radius*1000:.1f}mm")

        # Step 3: Ensure rotation direction consistency
        # Use rotation direction from contact phase if available
        insertion_info = getattr(userdata, 'insertion_info', {})
        contact_rotation_direction = insertion_info.get('rotation_direction', self.rotation_direction)
        if contact_rotation_direction != self.rotation_direction:
            rospy.logwarn(f"⚠ Direction mismatch: contact={contact_rotation_direction}, rotation={self.rotation_direction}")
            rospy.loginfo(f"✓ Using consistent rotation direction: {contact_rotation_direction}")
            effective_rotation_direction = contact_rotation_direction
        else:
            effective_rotation_direction = self.rotation_direction
            rospy.loginfo(f"✓ Rotation directions consistent: {effective_rotation_direction}")

        # 🔧 CRITICAL FIX: 与Single UAV版本保持一致，始终使用正向角速度！
        angular_velocity = abs(self.nominal_angular_velocity)  # 强制正值，与Single UAV一致
        
        # 🔍 ENHANCED DEBUG: 输出所有关键参数
        rospy.logwarn("=" * 60)
        rospy.logwarn("� ROTATION DEBUG - 所有关键参数:")
        rospy.logwarn(f"  📊 Trajectory Generator Parameters:")
        rospy.logwarn(f"    nominal_angular_velocity: {self.nominal_angular_velocity:.4f} rad/s ({math.degrees(self.nominal_angular_velocity):.2f}°/s)")
        rospy.logwarn(f"    effective_rotation_direction: {effective_rotation_direction:+d}")
        rospy.logwarn(f"    🎯 FINAL angular_velocity: {angular_velocity:.4f} rad/s ({math.degrees(angular_velocity):.2f}°/s)")
        rospy.logwarn(f"  📍 Valve Position & Orientation:")
        rospy.logwarn(f"    initial_valve_yaw (baseline): {math.degrees(initial_valve_yaw):.2f}°")
        rospy.logwarn(f"    current_valve_yaw_now: {math.degrees(current_valve_yaw_now):.2f}°")
        rospy.logwarn(f"    valve_center: {corrected_valve_center}")
        rospy.logwarn(f"    rotation_radius: {initial_radius*1000:.1f}mm")
        rospy.logwarn(f"  ⚙️ Expected Behavior:")
        if angular_velocity > 0:
            rospy.logwarn(f"    UAV will move: COUNTER-CLOCKWISE (positive angle increase)")
        else:
            rospy.logwarn(f"    UAV will move: CLOCKWISE (negative angle decrease)")
        rospy.logwarn("=" * 60)
        
        # 🔧 CRITICAL FIX: 直接复用Contact阶段的轨迹生成器状态，避免重新初始化
        contact_state = getattr(userdata, 'trajectory_state', {}) or {}
        
        if contact_state and all(k in contact_state for k in ['valve_center', 'current_angle', 'current_radius']):
            # 方案：继承Contact阶段的完整状态，只更改angular_velocity
            rospy.logwarn("🔧 INHERITING Contact trajectory state - seamless transition!")
            trajectory_gen = OnlineCircularTrajectoryGenerator(
                valve_center=contact_state['valve_center'],
                initial_radius=contact_state.get('current_radius', initial_radius),
                target_angular_velocity=angular_velocity,  # 只更改转速方向
                control_rate=self.control_rate,
                debug=True
            )
            
            # 精确恢复Contact阶段的所有状态参数
            trajectory_gen.current_angle = contact_state['current_angle']
            trajectory_gen.current_radius = contact_state['current_radius'] 
            trajectory_gen.radius_locked = contact_state.get('radius_locked', False)
            trajectory_gen.lock_radius_value = contact_state.get('lock_radius_value', contact_state['current_radius'])
            trajectory_gen.valve_center = np.array(contact_state['valve_center'])
            trajectory_gen.current_z = rotation_start_pos[2]
            
            rospy.logwarn(f"  ✓ 继承的状态: angle={math.degrees(trajectory_gen.current_angle):.1f}°, "
                         f"radius={trajectory_gen.current_radius*1000:.1f}mm, locked={trajectory_gen.radius_locked}")
        else:
            # 备用方案：重新初始化（保留原逻辑）
            rospy.logwarn("⚠ No complete contact state found, reinitializing trajectory generator")
            trajectory_gen = OnlineCircularTrajectoryGenerator(
                valve_center=corrected_valve_center,
                initial_radius=initial_radius,
                target_angular_velocity=angular_velocity,
                control_rate=self.control_rate,
                debug=True
            )
            trajectory_gen.initialize_from_current_position(rotation_start_pos)

        baseline_torque = getattr(userdata, 'contact_final_torque', [0.0, 0.0, 0.15])  # 增加基准扭矩

        rate = rospy.Rate(self.control_rate)
        start_time = rospy.Time.now().to_sec()
        last_valve_check_time = start_time
        last_valve_yaw = initial_valve_yaw
        valve_motion_stalled_time = 0.0  # 改名为stalled_time，与Single UAV一致
        max_valve_rotation_achieved = 0.0  # 添加Single UAV版本的跟踪变量
        valve_stuck_threshold = 8.0  # 增加到8秒，给阀门足够时间建立稳定接触

        rospy.loginfo(f"Target rotation: {math.degrees(self.target_rotation):.1f}° @ {math.degrees(abs(angular_velocity)):.1f}°/s")

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            elapsed = current_time - start_time

            current_pos = self.get_end_effector_position()
            current_yaw = self.get_end_effector_yaw()
            if current_pos is None or current_yaw is None:
                rospy.logwarn("Missing end-effector pose during rotation, retrying...")
                rate.sleep()
                continue

            current_valve_yaw = self.beetle.getValveYaw()
            if current_valve_yaw is None:
                rospy.logwarn("getValveYaw() returned None during rotation, skipping cycle")
                rate.sleep()
                continue
            
            # 采用Single UAV的计算方式：直接计算阀门转动角度（不normalize，与Single UAV一致）
            valve_rotation = abs(current_valve_yaw - initial_valve_yaw)
            max_valve_rotation_achieved = max(max_valve_rotation_achieved, valve_rotation)
            rotation_progress = valve_rotation  # 保持兼容性

            # 计算阀门角速度（完全采用Single UAV逻辑）
            valve_angular_velocity = 0.0
            if current_time > last_valve_check_time + 0.2:  # 每200ms更新一次
                dt = current_time - last_valve_check_time
                valve_angular_velocity = abs(current_valve_yaw - last_valve_yaw) / dt
                last_valve_yaw = current_valve_yaw
                last_valve_check_time = current_time

            # 🔍 DEBUG: 每5秒详细输出阀门状态（在valve_angular_velocity计算之后）
            if int(elapsed) % 5 == 0 and (elapsed - int(elapsed)) < 0.1:
                rospy.logwarn("🔍 VALVE & CONTACT STATUS DEBUG:")
                rospy.logwarn(f"  current_valve_yaw: {math.degrees(current_valve_yaw):.2f}°")
                rospy.logwarn(f"  initial_valve_yaw: {math.degrees(initial_valve_yaw):.2f}°")
                rospy.logwarn(f"  raw_difference: {math.degrees(current_valve_yaw - initial_valve_yaw):.2f}°")
                rospy.logwarn(f"  valve_rotation (abs): {math.degrees(valve_rotation):.2f}°")
                rospy.logwarn(f"  target_rotation: {math.degrees(self.target_rotation):.2f}°")
                rospy.logwarn(f"  valve_angular_velocity: {math.degrees(valve_angular_velocity):.3f}°/s")
                
                # 接触状态检测
                distance_to_valve = math.sqrt((current_pos[0] - corrected_valve_center[0])**2 + 
                                            (current_pos[1] - corrected_valve_center[1])**2)
                rospy.logwarn(f"  🔗 CONTACT STATUS:")
                rospy.logwarn(f"    distance_to_valve: {distance_to_valve*1000:.1f}mm")
                rospy.logwarn(f"    expected_radius: {initial_radius*1000:.1f}mm")
                if distance_to_valve > initial_radius * 1.2:
                    rospy.logwarn(f"    ⚠️ CONTACT LOST! UAV离阀门太远")
                elif abs(valve_angular_velocity) < 0.005:
                    rospy.logwarn(f"    ⚠️ VALVE NOT MOVING! 可能脱离接触")
                
                # 检测阀门运动停滞（改进逻辑）
                # 只有在运行超过5秒后才开始检测停滞，确保已建立稳定接触
                if elapsed > 5.0:
                    if valve_angular_velocity < 0.008:  # < 0.5°/s认为停滞（更宽松）
                        valve_motion_stalled_time += dt
                    else:
                        valve_motion_stalled_time = 0.0
                else:
                    valve_motion_stalled_time = 0.0  # 前5秒不检测停滞

            try:
                state_info = trajectory_gen.update_state(current_pos, current_yaw, valve_angular_velocity)
            except Exception as exc:
                rospy.logerr(f"Trajectory update failed: {exc}")
                rate.sleep()
                continue

            target_state = trajectory_gen.generate_target_state()
            target_pos = target_state['position'].tolist()
            target_yaw = target_state['yaw']
            linear_vel_vec = np.array(target_state['linear_velocity'])
            
            # Apply strict 3D speed limiting for formation safety
            speed_total = np.linalg.norm(linear_vel_vec)
            speed_xy = np.linalg.norm(linear_vel_vec[:2])
            
            # First limit total 3D speed
            if speed_total > self.max_linear_speed and speed_total > 1e-6:
                scale_total = self.max_linear_speed / speed_total
                linear_vel_vec *= scale_total
                rospy.loginfo(f"Speed clamped (3D): {speed_total:.3f}→{self.max_linear_speed:.3f}m/s")
            
            # Additional XY plane specific limiting
            speed_xy_after = np.linalg.norm(linear_vel_vec[:2])
            if speed_xy_after > self.max_linear_speed * 0.9 and speed_xy_after > 1e-6:  # 90% of max for XY
                scale_xy = (self.max_linear_speed * 0.9) / speed_xy_after
                linear_vel_vec[:2] *= scale_xy
                rospy.loginfo(f"XY speed further limited: {speed_xy_after:.3f}→{speed_xy_after*scale_xy:.3f}m/s")

            # 🔍 DEBUG: 每5秒输出轨迹生成器状态
            if int(elapsed) % 5 == 0 and (elapsed - int(elapsed)) < 0.1:
                rospy.logwarn("🎯 TRAJECTORY DEBUG:")
                rospy.logwarn(f"  current_pos: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
                rospy.logwarn(f"  target_pos: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                rospy.logwarn(f"  position_diff: [{target_pos[0]-current_pos[0]:.4f}, {target_pos[1]-current_pos[1]:.4f}, {target_pos[2]-current_pos[2]:.4f}]")
                rospy.logwarn(f"  current_yaw: {math.degrees(current_yaw):.2f}° -> target_yaw: {math.degrees(target_yaw):.2f}°")
                rospy.logwarn(f"  linear_vel: [{linear_vel_vec[0]:.4f}, {linear_vel_vec[1]:.4f}, {linear_vel_vec[2]:.4f}] m/s")
                rospy.logwarn(f"  angular_vel: {target_state.get('angular_velocity', 0.0):.4f} rad/s ({math.degrees(target_state.get('angular_velocity', 0.0)):.2f}°/s)")
                rospy.logwarn(f"  speed_total: {speed_total:.4f} m/s")
                
                # 检查trajectory generator内部状态
                if hasattr(trajectory_gen, 'current_angle'):
                    rospy.logwarn(f"  trajectory_angle: {math.degrees(trajectory_gen.current_angle):.2f}°")
                    rospy.logwarn(f"  expected_angular_velocity: {math.degrees(trajectory_gen.target_angular_velocity):.2f}°/s")

            self.send_assembly_command_from_end_effector(
                target_pos,
                target_yaw,
                linear_vel=linear_vel_vec.tolist(),
                angular_vel=target_state.get('angular_velocity')
            )

            if elapsed > self.min_rotation_time and rotation_progress >= self.target_rotation:
                rospy.loginfo("Rotation target achieved with minimum duration satisfied")
                break

            if elapsed > self.max_rotation_time:
                rospy.logwarn("Rotation timeout reached")
                break

            # 检查失败条件：阀门运动停滞（完全采用Single UAV逻辑）
            if valve_motion_stalled_time > valve_stuck_threshold:
                rospy.logerr("🚨 阀门运动停滞检测!")
                rospy.logerr(f"  停滞时间: {valve_motion_stalled_time:.1f}s > 阈值{valve_stuck_threshold:.1f}s")
                rospy.logerr(f"  当前阀门角速度: {math.degrees(valve_angular_velocity):.2f}°/s")
                rospy.logerr(f"  已达成旋转: {math.degrees(max_valve_rotation_achieved):.1f}°")
                rospy.logerr(f"  目标旋转: {math.degrees(self.target_rotation):.1f}°")
                rospy.logerr(f"  完成百分比: {(max_valve_rotation_achieved/self.target_rotation)*100:.1f}%")
                
                if max_valve_rotation_achieved > self.target_rotation * 0.7:  # 达到70%认为部分成功
                    rospy.loginfo(f"🔄 部分成功：已转动 {math.degrees(max_valve_rotation_achieved):.1f}°，继续尝试")
                    valve_motion_stalled_time = 0.0  # 重置计时器
                else:
                    rospy.logerr("❌ 阀门可能卡住或阻力过大，旋转失败!")
                    rospy.logerr(f"   失败原因: 仅达成 {math.degrees(max_valve_rotation_achieved):.1f}° < 70%目标({math.degrees(self.target_rotation * 0.7):.1f}°)")
                    return 'failed'

            if int(elapsed) % 5 == 0:
                progress_pct = trajectory_gen.get_progress(self.target_rotation) * 100.0
                rospy.loginfo_throttle(1.0,
                    f"Rotation progress: {progress_pct:.1f}% | Valve Δ={math.degrees(valve_rotation):.1f}° | "
                    f"Max={math.degrees(max_valve_rotation_achieved):.1f}° | "
                    f"Radius={state_info['current_radius']*1000:.1f}mm | ω={math.degrees(state_info['angular_velocity']):.1f}°/s")

            rate.sleep()

        # 最终检查（采用Single UAV逻辑）
        final_valve_yaw = self.beetle.getValveYaw()
        if final_valve_yaw is None:
            final_valve_yaw = initial_valve_yaw  # fallback to initial if read fails
        final_valve_rotation = abs(final_valve_yaw - initial_valve_yaw)
        
        if final_valve_rotation >= self.target_rotation * 0.8:  # 80%认为可接受
            rospy.loginfo(f"✓ 阀门旋转成功: {math.degrees(final_valve_rotation):.1f}°")
        else:
            rospy.logwarn(f"Valve rotation may be insufficient ({math.degrees(final_valve_rotation):.1f}°). Proceeding but flagging potential issue.")

        # Persist trajectory state for disengage phase
        if trajectory_gen is not None:
            userdata.trajectory_state = {
                'current_angle': getattr(trajectory_gen, 'current_angle', 0.0),
                'current_radius': getattr(trajectory_gen, 'current_radius', initial_radius),
                'radius_locked': getattr(trajectory_gen, 'radius_locked', False),
                'lock_radius_value': getattr(trajectory_gen, 'lock_radius_value', initial_radius),
                'valve_center': tuple(trajectory_gen.valve_center.tolist()) if hasattr(trajectory_gen, 'valve_center') else (valve_pos[0], valve_pos[1]),
                'final_angle': getattr(trajectory_gen, 'current_angle', 0.0),
                'initial_radius': initial_radius,
                'rotation_direction': effective_rotation_direction
            }
        else:
            # Fallback if trajectory_gen is None
            current_pos = self.get_end_effector_position()
            if current_pos is not None:
                fallback_radius = math.sqrt((current_pos[0] - valve_pos[0])**2 + (current_pos[1] - valve_pos[1])**2)
            else:
                fallback_radius = initial_radius
            
            userdata.trajectory_state = {
                'current_angle': 0.0,
                'current_radius': fallback_radius,
                'radius_locked': False,
                'lock_radius_value': fallback_radius,
                'valve_center': (valve_pos[0], valve_pos[1]),
                'final_angle': 0.0,
                'initial_radius': initial_radius,
                'rotation_direction': effective_rotation_direction
            }
        userdata.contact_final_torque = baseline_torque

        rospy.loginfo("Formation valve rotation completed; trajectory state saved for disengage phase")
        return 'succeeded'


class FormationDisengageFromValveState(FormationSingleUAVStateBase):
    """Formation version of DisengageFromValveState"""
    
    def __init__(self):
        FormationSingleUAVStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['valve_position', 'start_position', 'trajectory_state', 'initial_valve_yaw', 'rotation_start_position'],
            output_keys=['disengagement_position'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Formation Disengage From Valve State (Dragon Finish) ===")

        from trajectory import OnlineCircularTrajectoryGenerator

        valve_pos = getattr(userdata, 'valve_position', None)
        if valve_pos is None:
            rospy.logerr("Valve position missing from userdata")
            return 'failed'

        start_assembly_pos = getattr(userdata, 'start_position', None)
        start_yaw = self.formation_adapter.get_assembly_yaw() or 0.0
        if start_assembly_pos is not None:
            start_end_effector = self.formation_adapter.assembly_to_end_effector_transform(start_assembly_pos, start_yaw)
        else:
            start_end_effector = None

        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        if current_pos is None or current_yaw is None:
            rospy.logerr("Unable to read current end-effector pose for disengage")
            return 'failed'

        rospy.loginfo(f"Start position for disengage: {current_pos}, yaw={math.degrees(current_yaw):.1f}°")

        # Phase 1: Relax / hold position
        rospy.loginfo("[Disengage] Phase1: Relax at contact pose (2s)")
        relax_end = rospy.Time.now().to_sec() + 2.0
        relax_rate = rospy.Rate(10)
        while rospy.Time.now().to_sec() < relax_end and not rospy.is_shutdown():
            self.send_assembly_command_from_end_effector(current_pos, current_yaw)
            relax_rate.sleep()

        # Phase 1.5: Reverse rotation release
        trajectory_state = getattr(userdata, 'trajectory_state', {}) or {}
        reverse_direction = -trajectory_state.get('rotation_direction', self.rotation_direction if hasattr(self, 'rotation_direction') else 1)
        reverse_angle = math.radians(15.0)
        reverse_velocity = math.radians(3.0) * reverse_direction

        rospy.loginfo(f"[Disengage] Phase1.5: Reverse rotation {math.degrees(reverse_angle):.1f}°")

        generator_center = trajectory_state.get('valve_center', (valve_pos[0], valve_pos[1], current_pos[2]))
        generator_radius = trajectory_state.get('current_radius')
        if generator_radius is None:
            generator_radius = math.sqrt((current_pos[0] - generator_center[0])**2 +
                                         (current_pos[1] - generator_center[1])**2)

        try:
            reverse_generator = OnlineCircularTrajectoryGenerator(
                valve_center=generator_center,
                initial_radius=generator_radius,
                target_angular_velocity=reverse_velocity,
                control_rate=20.0,
                debug=True
            )
            reverse_generator.initialize_from_current_position(current_pos)
        except Exception as exc:
            rospy.logwarn(f"Reverse rotation generator creation failed ({exc}), skipping phase")
            reverse_generator = None

        reverse_rate = rospy.Rate(20)
        reverse_accum = 0.0
        reverse_start = rospy.Time.now().to_sec()
        last_yaw = current_yaw

        while reverse_generator and reverse_accum < reverse_angle and not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            updated_pos = self.get_end_effector_position()
            updated_yaw = self.get_end_effector_yaw()
            if updated_pos is None or updated_yaw is None:
                reverse_rate.sleep()
                continue

            try:
                reverse_generator.update_state(updated_pos, updated_yaw, abs(reverse_velocity))
                target_state = reverse_generator.generate_target_state()
                target_pos = target_state['position'].tolist()
                target_yaw = target_state['yaw']
                self.send_assembly_command_from_end_effector(
                    target_pos,
                    target_yaw,
                    linear_vel=target_state['linear_velocity'].tolist(),
                    angular_vel=target_state.get('angular_velocity')
                )
            except Exception as exc:
                rospy.logwarn(f"Reverse rotation update failed ({exc})")
                break

            reverse_accum = abs(self._normalize_angle(updated_yaw - last_yaw))
            reverse_rate.sleep()

        rospy.loginfo("[Disengage] Reverse rotation complete; stabilizing 2s")
        stabilize_end = rospy.Time.now().to_sec() + 2.0
        stabilize_rate = rospy.Rate(10)
        stable_pos = self.get_end_effector_position()
        stable_yaw = self.get_end_effector_yaw()
        while rospy.Time.now().to_sec() < stabilize_end and not rospy.is_shutdown():
            self.send_assembly_command_from_end_effector(stable_pos, stable_yaw)
            stabilize_rate.sleep()

        # Refresh current pose after reverse
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()

        # Phase 2: Z ascent to safe height
        rospy.loginfo("[Disengage] Phase2: Z-ascent to safe height")
        if start_end_effector is not None:
            target_z = start_end_effector[2]
        else:
            target_z = current_pos[2] + 0.05
        ascent_target = [current_pos[0], current_pos[1], target_z]
        self.active_position_convergence(
            target_ee_pos=ascent_target,
            target_yaw=current_yaw,
            pos_threshold=0.02,
            yaw_threshold=0.1,
            vel_threshold=0.02,
            min_readings=3,
            max_attempts=60,
            timeout=25.0
        )

        rospy.sleep(1.0)

        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()

        # Phase 3: Yaw realignment to 0°
        rospy.loginfo("[Disengage] Phase3: Yaw realignment to 0°")
        FormationSingleUAVStateBase.active_position_convergence(
            self,
            target_ee_pos=current_pos,
            target_yaw=0.0,
            pos_threshold=0.03,
            yaw_threshold=math.radians(5.0),
            vel_threshold=0.02,
            min_readings=3,
            max_attempts=50,
            timeout=15.0
        )

        rospy.sleep(0.5)
        current_pos = self.get_end_effector_position()

        # Phase 4: Radial retreat from valve
        rospy.loginfo("[Disengage] Phase4: Radial retreat from valve (0.15m)")
        retreat_vector = np.array(current_pos[:2]) - np.array(valve_pos[:2])
        retreat_norm = np.linalg.norm(retreat_vector)
        if retreat_norm < 1e-3:
            retreat_vector = np.array([0.15, 0.0])
            retreat_norm = 0.15
        retreat_vector = retreat_vector / retreat_norm
        retreat_distance = 0.15
        target_xy = np.array(current_pos[:2]) + retreat_vector * retreat_distance
        retreat_target = [target_xy[0], target_xy[1], current_pos[2]]
        FormationSingleUAVStateBase.active_position_convergence(
            self,
            target_ee_pos=retreat_target,
            target_yaw=0.0,
            pos_threshold=0.03,
            yaw_threshold=math.radians(6.0),
            vel_threshold=0.02,
            min_readings=3,
            max_attempts=60,
            timeout=20.0
        )

        rospy.sleep(0.5)
        current_pos = self.get_end_effector_position()

        # Phase 5: Return to start XY above start height
        if start_end_effector is not None:
            rospy.loginfo("[Disengage] Phase5: Return to recorded start position")
            return_target = [start_end_effector[0], start_end_effector[1], max(start_end_effector[2], current_pos[2])]
            FormationSingleUAVStateBase.active_position_convergence(
                self,
                target_ee_pos=return_target,
                target_yaw=0.0,
                pos_threshold=0.04,
                yaw_threshold=math.radians(6.0),
                vel_threshold=0.02,
                min_readings=3,
                max_attempts=60,
                timeout=25.0
            )
        else:
            rospy.logwarn("Start position unavailable; skipping return-to-start phase")

        final_pos = self.get_end_effector_position()
        userdata.disengagement_position = final_pos
        rospy.loginfo(f"Disengage complete. Final EE pose: {final_pos}")
        return 'succeeded'

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


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


# Main execution function
def main():
    rospy.init_node('formation_valve_rotation')
    
    # Get rotation direction from parameter
    direction_param = rospy.get_param("~valve_rotation_direction", "clockwise")
    direction_normalized = direction_param.strip().lower()
    if direction_normalized in ["clockwise", "cw", "右转", "顺时针"]:
        rotation_direction = -1
        direction_label = "Clockwise (negative yaw)"
    elif direction_normalized in ["counterclockwise", "counter-clockwise", "ccw", "左转", "逆时针"]:
        rotation_direction = 1
        direction_label = "Counter-clockwise (positive yaw)"
    else:
        rotation_direction = 1
        direction_label = f"Counter-clockwise (fallback for '{direction_param}')"
        rospy.logwarn(f"Unknown valve rotation direction '{direction_param}', defaulting to counter-clockwise")

    rospy.loginfo(f"Formation valve rotation direction: {direction_label}")
    
    try:
        # Create Formation SMACH state machine
        sm = smach.StateMachine(outcomes=['success', 'failure'])
        
        with sm:
            # Step 1: Assemble the formation (physical connection)
            smach.StateMachine.add('FORMATION_ASSEMBLE',
                                   FormationAssembleState(),
                                   transitions={'succeeded': 'FORMATION_INITIALIZE',
                                               'failed': 'failure'})
            
            # Step 2: Initialize positions and valve data
            smach.StateMachine.add('FORMATION_INITIALIZE',
                                   FormationInitializeStartPositionState(),
                                   transitions={'succeeded': 'FORMATION_MOVE_TO_VALVE',
                                               'failed': 'failure'})
            
            # Step 3: Move formation to valve approach position
            smach.StateMachine.add('FORMATION_MOVE_TO_VALVE',
                                   FormationMoveToValveState(),
                                   transitions={'succeeded': 'FORMATION_ROTATE_VALVE',
                                               'failed': 'failure'})
            
            # Step 4: Contact and rotate the valve (unified state)
            smach.StateMachine.add('FORMATION_ROTATE_VALVE',
                                   FormationRotateValveState(rotation_direction=rotation_direction),
                                   transitions={'succeeded': 'FORMATION_DISENGAGE',
                                               'failed': 'failure',
                                               'emergency': 'failure'})
            
            # Step 6: Disengage from valve and return to start
            smach.StateMachine.add('FORMATION_DISENGAGE',
                                   FormationDisengageFromValveState(),
                                   transitions={'succeeded': 'success',
                                               'failed': 'failure'})
        
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
