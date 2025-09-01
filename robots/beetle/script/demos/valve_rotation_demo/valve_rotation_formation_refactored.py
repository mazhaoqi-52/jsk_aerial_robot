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

import smach
import smach_ros

# Add path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.dirname(__file__))  # Add current directory

from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

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
    from unified_motion_controller import UnifiedMotionController, FormationMotionCompatibility
    from trajectory import create_constant_distance_trajectory
    from insertion_optimizer import InsertionOptimizer
    rospy.loginfo(f"✓ Successfully imported single UAV classes from: {source_dir}")
    rospy.loginfo(f"✓ Successfully imported single UAV classes from: {source_dir}")

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
        # Formation geometry: two UAVs side by side
        # CORRECTED: Leader UAV (rightmost) is offset from formation center
        # Based on valve_rotation_formation.py: y_offset=0.25
        self.assembly_to_leader_offset_y = 0.25  # Leader UAV at +Y=0.25m from formation center
        self.assembly_to_leader_offset_x = 0.0   # No X offset (aligned)
        self.assembly_to_leader_offset_z = 0.0   # Same Z level
        
        # === Step 2: Leader UAV center → End Effector ===  
        # Use EXACT same offsets as single UAV (from valve_rotation_fang_single.py)
        self.leader_to_end_effector_offset_x = 0.25346  # CORRECTED: Use single UAV value
        self.leader_to_end_effector_offset_y = 0.0      # Centered on leader UAV
        
        # Z offsets for different phases
        self.leader_to_end_effector_offset_z_trajectory = 0.0743823  # Trajectory phase
        self.leader_to_end_effector_offset_z_insertion = 0.0221140   # Insertion phase
        
        # === Combined offsets: Assembly CoG → End Effector (TOTAL) ===
        # This is what gets used when we need to go directly from assembly to end-effector
        self.total_offset_x = (self.assembly_to_leader_offset_x + 
                              self.leader_to_end_effector_offset_x)  # = 0.0 + 0.25346
        self.total_offset_y = (self.assembly_to_leader_offset_y + 
                              self.leader_to_end_effector_offset_y)  # = 0.5 + 0.0
        self.total_offset_z_trajectory = (self.assembly_to_leader_offset_z + 
                                         self.leader_to_end_effector_offset_z_trajectory)
        self.total_offset_z_insertion = (self.assembly_to_leader_offset_z + 
                                        self.leader_to_end_effector_offset_z_insertion)
        
        rospy.loginfo("=== CORRECTED FORMATION COORDINATE SYSTEM ===")
        rospy.loginfo(f"Assembly → Leader UAV: X={self.assembly_to_leader_offset_x:.3f}, "
                     f"Y={self.assembly_to_leader_offset_y:.3f}, Z={self.assembly_to_leader_offset_z:.3f}")
        rospy.loginfo(f"Leader UAV → End Effector: X={self.leader_to_end_effector_offset_x:.6f}, "
                     f"Y={self.leader_to_end_effector_offset_y:.3f}, Z_traj={self.leader_to_end_effector_offset_z_trajectory:.6f}")
        rospy.loginfo(f"TOTAL Assembly → End Effector: X={self.total_offset_x:.6f}, "
                     f"Y={self.total_offset_y:.3f}, Z_traj={self.total_offset_z_trajectory:.6f}")
        rospy.loginfo("=" * 50)
        
        # === ROS Communication Setup ===
        # Publisher for assembly navigation commands (CORRECTED TOPIC)
        self.assembly_pub = rospy.Publisher('/assembly/uav/nav', FlightNav, queue_size=10)
        
        # Position tracking variables
        self.assembly_pos = None
        self.assembly_yaw = 0.0
        self.module1_pos = None
        self.module2_pos = None
        
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
        rospy.logdebug(f"Module1 position updated: {self.module1_pos}")
        # If no assembly odom, calculate CoG from individual modules
        if self.assembly_pos is None and self.module1_pos and self.module2_pos:
            self.calculate_assembly_position_from_modules()
        
    def module2_callback(self, msg):
        """Handle position from module 2 (PoseStamped format)"""
        self.module2_pos = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        # Extract yaw from module 2 (end-effector module)
        orientation_q = msg.pose.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, self.assembly_yaw = euler_from_quaternion(quaternion)
        
        rospy.logdebug(f"Module2 position updated: {self.module2_pos}, yaw: {self.assembly_yaw:.3f}")
        # If no assembly odom, calculate CoG from individual modules
        if self.assembly_pos is None and self.module1_pos and self.module2_pos:
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
            rospy.loginfo(f"Assembly position calculated from modules: {self.assembly_pos}")
            rospy.loginfo(f"Module1: {self.module1_pos}, Module2: {self.module2_pos}")
    
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
    
    def assembly_to_end_effector_transform(self, assembly_pos, assembly_yaw, use_insertion_offset=False):
        """
        CORRECTED: Complete coordinate transformation chain
        Assembly CoG → Leader UAV → End Effector
        """
        if assembly_pos is None:
            return None
        
        # Step 1: Assembly CoG → Leader UAV center
        # Formation rotation: leader UAV position relative to assembly center
        cos_yaw = math.cos(assembly_yaw)
        sin_yaw = math.sin(assembly_yaw)
        
        leader_uav_x = (assembly_pos[0] + 
                       self.assembly_to_leader_offset_x * cos_yaw - 
                       self.assembly_to_leader_offset_y * sin_yaw)
        leader_uav_y = (assembly_pos[1] + 
                       self.assembly_to_leader_offset_x * sin_yaw + 
                       self.assembly_to_leader_offset_y * cos_yaw)
        leader_uav_z = assembly_pos[2] + self.assembly_to_leader_offset_z
        
        # Step 2: Leader UAV center → End Effector
        # Tool offset from leader UAV (same as single UAV transformation)
        end_effector_x = (leader_uav_x + 
                         self.leader_to_end_effector_offset_x * cos_yaw - 
                         self.leader_to_end_effector_offset_y * sin_yaw)
        end_effector_y = (leader_uav_y + 
                         self.leader_to_end_effector_offset_x * sin_yaw + 
                         self.leader_to_end_effector_offset_y * cos_yaw)
        end_effector_z = leader_uav_z + (self.leader_to_end_effector_offset_z_insertion 
                                        if use_insertion_offset 
                                        else self.leader_to_end_effector_offset_z_trajectory)
        
        return (end_effector_x, end_effector_y, end_effector_z)
    
    def end_effector_to_assembly_transform(self, end_effector_pos, assembly_yaw, use_insertion_offset=False):
        """
        CORRECTED: Reverse transformation chain
        End Effector → Leader UAV → Assembly CoG
        """
        if end_effector_pos is None:
            return None
        
        cos_yaw = math.cos(assembly_yaw)
        sin_yaw = math.sin(assembly_yaw)
        
        # Step 1: End Effector → Leader UAV center (reverse transformation)
        leader_uav_x = (end_effector_pos[0] - 
                       self.leader_to_end_effector_offset_x * cos_yaw + 
                       self.leader_to_end_effector_offset_y * sin_yaw)
        leader_uav_y = (end_effector_pos[1] - 
                       self.leader_to_end_effector_offset_x * sin_yaw - 
                       self.leader_to_end_effector_offset_y * cos_yaw)
        leader_uav_z = end_effector_pos[2] - (self.leader_to_end_effector_offset_z_insertion 
                                             if use_insertion_offset 
                                             else self.leader_to_end_effector_offset_z_trajectory)
        
        # Step 2: Leader UAV center → Assembly CoG (reverse transformation)
        assembly_x = (leader_uav_x - 
                     self.assembly_to_leader_offset_x * cos_yaw + 
                     self.assembly_to_leader_offset_y * sin_yaw)
        assembly_y = (leader_uav_y - 
                     self.assembly_to_leader_offset_x * sin_yaw - 
                     self.assembly_to_leader_offset_y * cos_yaw)
        assembly_z = leader_uav_z - self.assembly_to_leader_offset_z
        
        return (assembly_x, assembly_y, assembly_z)
    
    def send_assembly_command(self, target_pos, target_yaw=None):
        """Send command to assembly formation (equivalent to single UAV send_trajectory_point)"""
        try:
            nav_msg = FlightNav()
            nav_msg.target = FlightNav.COG  # Formation uses COG target
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            nav_msg.target_pos_x = target_pos[0]
            nav_msg.target_pos_y = target_pos[1]
            nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
            nav_msg.target_pos_z = target_pos[2]
            
            if target_yaw is not None:
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE
                nav_msg.target_yaw = target_yaw
            else:
                nav_msg.yaw_nav_mode = FlightNav.NO_NAVIGATION
            
            self.assembly_pub.publish(nav_msg)
            yaw_str = f"{target_yaw:.3f}" if target_yaw is not None else "None"
            rospy.logdebug(f"Sent assembly command: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}), yaw: {yaw_str}")
            
        except Exception as e:
            rospy.logerr(f"Error sending assembly command: {e}")


class FormationSingleUAVStateBase(SingleUAVStateBase):
    """
    Formation state base that inherits from SingleUAVStateBase and overrides only necessary methods
    This allows complete reuse of single UAV logic with minimal changes
    """
    
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
        """Override to return END-EFFECTOR position for compatibility with single UAV logic"""
        # Get assembly position
        assembly_pos = self.adapter.get_assembly_position()
        if assembly_pos is None:
            return None
        
        # Transform assembly position to end-effector position for rotation logic compatibility
        assembly_yaw = self.adapter.get_assembly_yaw()
        end_effector_pos = self.adapter.assembly_to_end_effector_transform(
            assembly_pos, assembly_yaw, use_insertion_offset=False
        )
        
        return end_effector_pos
    
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
        Execute trajectory directly in assembly coordinates (no coordinate transformation)
        Used when positions are already in assembly coordinate system
        """
        # Execute trajectory in assembly coordinates
        from trajectory import PolynomialTrajectory
        
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(start_assembly_pos, target_assembly_pos)
        
        start_assembly_yaw = self.adapter.get_assembly_yaw()
        yaw_diff = target_yaw - start_assembly_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        rospy.loginfo(f"Assembly trajectory: {start_assembly_pos} -> {target_assembly_pos}")
        rospy.loginfo(f"Yaw change: {math.degrees(yaw_diff):.1f}°")
        
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
        Execute smooth trajectory for formation
        Transforms trajectory from end-effector coordinates to assembly coordinates
        """
        # Transform start and target positions from end-effector to assembly coordinates
        start_assembly_yaw = self.adapter.get_assembly_yaw()
        target_assembly_yaw = target_yaw
        
        start_assembly_pos = self.adapter.end_effector_to_assembly_transform(start_pos, start_assembly_yaw)
        target_assembly_pos = self.adapter.end_effector_to_assembly_transform(target_pos, target_assembly_yaw)
        
        if start_assembly_pos is None or target_assembly_pos is None:
            rospy.logerr("Failed to transform trajectory positions")
            return False
        
        # Execute trajectory in assembly coordinates
        return self.execute_assembly_trajectory(start_assembly_pos, target_assembly_pos, target_assembly_yaw, 
                                              duration, pos_threshold, yaw_threshold)


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
        
        # Calculate approach position (same as single UAV)
        valve_x, valve_y, valve_z = self.valve_pos
        current_pos = self.get_current_position()
        
        if current_pos is None:
            rospy.logerr("Cannot get current formation position")
            return 'failed'
        
        # Fixed approach from negative X side (same as single UAV)  
        # BUT: target_pos is END-EFFECTOR position, need to convert to ASSEMBLY position
        approach_x = valve_x - self.approach_distance
        approach_y = valve_y
        approach_z = valve_z + self.approach_height
        
        target_end_effector_pos = (approach_x, approach_y, approach_z)  # End-effector target
        target_yaw = math.atan2(valve_y - approach_y, valve_x - approach_x)
        
        # Convert end-effector target to assembly target
        target_assembly_pos = self.adapter.end_effector_to_assembly_transform(
            target_end_effector_pos, target_yaw, use_insertion_offset=False
        )
        
        rospy.loginfo(f"Valve position: ({valve_x:.3f}, {valve_y:.3f}, {valve_z:.3f})")
        rospy.loginfo(f"Current assembly position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
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
            rospy.loginfo(f"Target end-effector position: {target_end_effector_pos}")
        
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
    
    def execute(self, userdata):
        """Reuse EXACT same logic as single UAV DescendAndContactState.execute()"""
        rospy.loginfo("Formation descend and contact using single UAV logic...")
        
        # Initialize and wait for required positions
        if not self.wait_for_positions():
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None:
            return 'failed'
        
        # CRITICAL: Calculate correct Z position for end-effector insertion at valve height
        end_effector_offset_z = 0.0221140
        valve_z = self.valve_pos[2]
        required_uav_z = valve_z - end_effector_offset_z
        
        self.locked_z = required_uav_z
        rospy.loginfo(f"=== FORMATION INSERTION HEIGHT CALCULATION ===")
        rospy.loginfo(f"Valve height: {valve_z:.6f}m")
        rospy.loginfo(f"End-effector Z offset: {end_effector_offset_z:.6f}m")
        rospy.loginfo(f"Required formation Z for insertion: {required_uav_z:.6f}m")
        rospy.loginfo(f"Z-axis LOCKED at {self.locked_z:.6f}m for valve insertion")
        
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
        """Formation version using DIRECT control with CORRECT coordinate transformation"""
        if not hasattr(self, 'current_insertion_params'):
            rospy.logerr("No insertion parameters available")
            return False
        
        params = self.current_insertion_params
        target_end_effector_pos = params['target_position']  # This is end-effector target
        approach_angle = params['approach_angle']
        
        rospy.loginfo(f"Formation executing DIRECT insertion with end-effector target: {target_end_effector_pos}")
        rospy.loginfo(f"Approach angle: {approach_angle:.3f} rad ({approach_angle*180/3.14159:.1f} degrees)")
        
        try:
            current_assembly_pos = self.get_assembly_position()  # Get assembly position for motion control
            if current_assembly_pos is None:
                rospy.logerr("No assembly position available")
                return False
            
            # Use DIRECT trajectory control like valve_rotation_smach_test.py
            from unified_motion_controller import FormationMotionCompatibility
            avg_speed = 0.12  # Increased speed: was 0.08, now 0.12m/s
            
            # === KEY INSIGHT: Convert end-effector targets to assembly targets ===
            
            # Step 1: Calculate assembly target for horizontal approach position
            approach_height = max(target_end_effector_pos[2] + 0.3, current_assembly_pos[2])  # 30cm above target
            horizontal_end_effector_target = [target_end_effector_pos[0], target_end_effector_pos[1], approach_height]
            
            # Convert end-effector target to assembly target
            horizontal_assembly_target = self.adapter.end_effector_to_assembly_transform(
                horizontal_end_effector_target, approach_angle, use_insertion_offset=False
            )
            
            rospy.loginfo(f"Step 1: Moving assembly to: {horizontal_assembly_target}")
            rospy.loginfo(f"  (to place end-effector at: {horizontal_end_effector_target})")
            
            # Execute horizontal movement using direct assembly control
            current_assembly_yaw = self.adapter.get_assembly_yaw()
            distance = math.sqrt(sum((t - s) ** 2 for s, t in zip(current_assembly_pos, horizontal_assembly_target)))
            duration = max(distance / avg_speed, 1.0)
            
            success = self.execute_assembly_trajectory(current_assembly_pos, horizontal_assembly_target, 
                                                     current_assembly_yaw, duration)
            if not success:
                rospy.logerr("Formation failed to reach horizontal approach position")
                return False
            
            rospy.loginfo("✓ Horizontal approach completed")
            
            # Step 2: Adjust yaw orientation while maintaining position
            rospy.loginfo(f"Step 2: Adjusting yaw to insertion angle: {approach_angle:.3f} rad")
            
            # Update current position
            current_assembly_pos = self.get_assembly_position()
            if current_assembly_pos is None:
                current_assembly_pos = horizontal_assembly_target
            
            # Use direct yaw control 
            yaw_success = self.execute_assembly_trajectory(current_assembly_pos, current_assembly_pos, 
                                                         approach_angle, 0.5)  # 0.5 seconds for yaw
            if not yaw_success:
                rospy.logwarn("Yaw adjustment failed, continuing")
            
            rospy.loginfo("✓ Yaw adjustment completed")
            
            # Step 3: Descend to pre-insertion height
            pre_insertion_end_effector = [target_end_effector_pos[0], target_end_effector_pos[1], target_end_effector_pos[2] + 0.1]  # 10cm above valve
            pre_insertion_assembly_target = self.adapter.end_effector_to_assembly_transform(
                pre_insertion_end_effector, approach_angle, use_insertion_offset=False
            )
            
            rospy.loginfo(f"Step 3: Moving assembly to: {pre_insertion_assembly_target}")
            rospy.loginfo(f"  (to place end-effector at: {pre_insertion_end_effector})")
            
            # Update current position again
            current_assembly_pos = self.get_assembly_position()
            if current_assembly_pos is None:
                current_assembly_pos = horizontal_assembly_target
            
            # Execute descent to pre-insertion height
            distance = math.sqrt(sum((t - s) ** 2 for s, t in zip(current_assembly_pos, pre_insertion_assembly_target)))
            duration = max(distance / (avg_speed * 0.7), 1.0)
            
            descent_success = self.execute_assembly_trajectory(current_assembly_pos, pre_insertion_assembly_target, 
                                                             approach_angle, duration)
            if not descent_success:
                rospy.logerr("Pre-insertion descent failed")
                return False
            
            rospy.loginfo("✓ Pre-insertion descent completed")
            
            # Step 4: Final precise insertion
            final_assembly_target = self.adapter.end_effector_to_assembly_transform(
                target_end_effector_pos, approach_angle, use_insertion_offset=True  # Use insertion offset
            )
            
            rospy.loginfo(f"Step 4: Final insertion - moving assembly to: {final_assembly_target}")
            rospy.loginfo(f"  (to place end-effector exactly at valve: {target_end_effector_pos})")
            
            # Update current position
            current_assembly_pos = self.get_assembly_position()
            if current_assembly_pos is None:
                current_assembly_pos = pre_insertion_assembly_target
            
            # Execute final insertion
            distance = math.sqrt(sum((t - s) ** 2 for s, t in zip(current_assembly_pos, final_assembly_target)))
            duration = max(distance / (avg_speed * 0.5), 1.0)
            
            final_success = self.execute_assembly_trajectory(current_assembly_pos, final_assembly_target, 
                                                           approach_angle, duration)
            if not final_success:
                rospy.logerr("Final insertion failed")
                return False
            
            rospy.loginfo("✓ Formation DIRECT insertion completed successfully")
            
            # Verify final position
            final_assembly_pos = self.get_current_position()
            if final_assembly_pos:
                actual_end_effector_pos = self.adapter.assembly_to_end_effector_transform(
                    final_assembly_pos, approach_angle, use_insertion_offset=True
                )
                rospy.loginfo(f"Final end-effector position: {actual_end_effector_pos}")
                rospy.loginfo(f"Target end-effector position: {target_end_effector_pos}")
                
                return True
            else:
                rospy.logerr("Formation final insertion failed")
                return False
            
        except Exception as e:
            rospy.logerr(f"Formation insertion error: {str(e)}")
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
        """Reuse EXACT same logic as single UAV RotateValveState.execute()"""
        rospy.loginfo("Formation valve rotation using single UAV logic...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        # For rotation trajectory, we need assembly position (UAV coordinate system)
        current_assembly_pos = self.get_assembly_position()
        if current_assembly_pos is None:
            return 'failed'
        
        # For tracking and monitoring, we use end-effector position
        current_end_effector_pos = self.get_current_position()
        if current_end_effector_pos is None:
            return 'failed'
        
        # Initialize tracking with end-effector position
        self.last_position = current_end_effector_pos
        self.last_yaw = self.current_yaw
        self.stuck_start_time = None
        
        # Create rotation trajectory using assembly position (UAV coordinate system)
        rotation_traj = create_constant_distance_trajectory(
            current_uav_pos=current_assembly_pos,
            current_uav_yaw=self.current_yaw,
            valve_center=self.valve_pos,
            rotation_angle=self.rotation_angle,
            rotation_duration=self.rotation_duration,
            end_effector_offset_x=0.246,
            end_effector_offset_y=0.0,
            end_effector_offset_z=0.0743823,
            rotation_direction=self.rotation_direction
        )
        
        if rotation_traj is None:
            rospy.logerr("Failed to create rotation trajectory")
            return 'failed'
        
        # Start trajectory
        rotation_traj.start_trajectory()
        
        # Start emergency monitoring
        self.start_emergency_monitoring()
        
        # Execute rotation
        try:
            success = self.execute_rotation_trajectory(rotation_traj)
            
            # Stop emergency monitoring
            self.stop_emergency_monitoring()
            
            # Check if emergency was triggered
            if hasattr(self, 'emergency_triggered') and self.emergency_triggered:
                rospy.logwarn("Emergency detected during rotation")
                return 'emergency'
            
            if success:
                rospy.loginfo("Formation valve rotation completed successfully")
                return 'succeeded'
            else:
                rospy.logerr("Formation valve rotation failed")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during formation rotation: {e}")
            self.emergency_stop.set()
            return 'failed'
    
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
