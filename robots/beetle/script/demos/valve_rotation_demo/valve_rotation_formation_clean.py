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

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(current_dir, '../..'))
sys.path.insert(0, os.path.join(current_dir, '..'))
sys.path.insert(0, current_dir)

from task.assembly_motion import AssemblyDemo
from n_modules_tf import NModuleTFCalculator
from valve_rotation_fang_single import (
    SingleUAVStateBase, 
    InitializeStartPositionState,
    MoveToValveState, 
    DescendAndContactState,
    RotateValveState
)
from insertion_optimizer import InsertionOptimizer
from trajectory import OnlineCircularTrajectoryGenerator, PolynomialTrajectory
from beetle_interface import BeetleInterface


class FormationAdapter:
    """Dynamic adapter for multi-UAV formation with configurable module_ids"""
    
    def __init__(self, module_ids_str=None):
        if module_ids_str is None:
            module_ids_str = rospy.get_param("~module_ids", "2,3")
        
        rospy.loginfo(f"Initializing FormationAdapter with module_ids: {module_ids_str}")
        
        rospy.set_param("~module_ids", module_ids_str)
        self.tf_calculator = NModuleTFCalculator(module_ids_str)
        
        if not self.tf_calculator.validate_module_configuration():
            raise ValueError(f"Invalid module configuration: {module_ids_str}")
        
        self.module_ids = self.tf_calculator.module_ids
        self.leader_id = self.tf_calculator.get_leader_id()
        self.follower_ids = self.tf_calculator.get_follower_ids()
        self.number_of_modules = len(self.module_ids)
        
        rospy.loginfo(f"Formation configuration:")
        rospy.loginfo(f"Module IDs: {self.module_ids}")
        rospy.loginfo(f"Leader (end-effector): {self.leader_id}")
        rospy.loginfo(f"Followers: {self.follower_ids}")
        
        self.uav_positions = {}
        self.uav_orientations = {}
        self.assembly_pos = None
        self.assembly_yaw = 0.0
        
        self.position_received = threading.Event()
        
        self.uav_subscribers = {}
        for module_id in self.module_ids:
            topic = f"/beetle{module_id}/mocap/pose"
            self.uav_subscribers[module_id] = rospy.Subscriber(
                topic, PoseStamped, 
                lambda msg, mid=module_id: self.uav_callback(msg, mid),
                queue_size=1
            )
            rospy.loginfo(f"Subscribed to {topic} for module {module_id}")
        
        # Assembly command publisher
        self.assembly_pub = rospy.Publisher(
            "/assembly/uav/nav", FlightNav, queue_size=1
        )
        
        # Calculate and store offset parameters for assembly-to-end-effector transform
        self._calculate_offset_parameters()
        
        rospy.loginfo("FormationAdapter initialization complete")
    
    def _calculate_offset_parameters(self):
        """Calculate and store offset parameters for coordinate transformations"""
        reference_yaw = 0.0
        assembly_to_leader = self.tf_calculator.calculate_assembly_to_leader_transform()
        leader_to_ee = self.tf_calculator.calculate_leader_to_end_effector_transform(reference_yaw)
        
        self.base_offset_x = assembly_to_leader['offset_x'] + leader_to_ee['x']
        self.base_offset_y = assembly_to_leader['offset_y'] + leader_to_ee['y']
        self.total_offset_z = assembly_to_leader['offset_z'] + leader_to_ee['z']
        
        rospy.loginfo(f"Assembly to End-effector offsets:")
        rospy.loginfo(f"Base X offset: {self.base_offset_x:.3f}m")
        rospy.loginfo(f"Base Y offset: {self.base_offset_y:.3f}m") 
        rospy.loginfo(f"Z offset: {self.total_offset_z:.3f}m")
    
    def uav_callback(self, msg, module_id):
        """Receive individual UAV position from mocap (PoseStamped format)"""
        pos = msg.pose.position
        ori = msg.pose.orientation
        
        self.uav_positions[module_id] = (pos.x, pos.y, pos.z)
        
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        self.uav_orientations[module_id] = yaw
        
        self._update_assembly_position()
    
    def _update_assembly_position(self):
        """Calculate assembly CoG position from all participating UAVs"""
        if len(self.uav_positions) < self.number_of_modules:
            return
        
        for module_id in self.module_ids:
            if module_id not in self.uav_positions or module_id not in self.uav_orientations:
                return
        
        # Calculate assembly CoG as average of all UAV positions
        avg_x = sum(self.uav_positions[mid][0] for mid in self.module_ids) / self.number_of_modules
        avg_y = sum(self.uav_positions[mid][1] for mid in self.module_ids) / self.number_of_modules
        avg_z = sum(self.uav_positions[mid][2] for mid in self.module_ids) / self.number_of_modules
        
        self.assembly_pos = (avg_x, avg_y, avg_z)
        
        yaw_sum = 0.0
        base_yaw = self.uav_orientations[self.module_ids[0]]
        
        for module_id in self.module_ids:
            yaw = self.uav_orientations[module_id]
            yaw_diff = yaw - base_yaw
            while yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            yaw_sum += base_yaw + yaw_diff
        
        self.assembly_yaw = yaw_sum / self.number_of_modules
        
        if not self.position_received.is_set():
            rospy.loginfo(f"Formation position received: {self.assembly_pos}")
            for mid in self.module_ids:
                rospy.loginfo(f"UAV{mid}: {self.uav_positions[mid]}")
            self.position_received.set()
    
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
        """Transform end-effector target to assembly CoG command"""
        if target_yaw is None:
            target_yaw = self.assembly_yaw if self.assembly_yaw is not None else 0.0
        
        assembly_to_leader = self.tf_calculator.calculate_assembly_to_leader_transform()
        leader_to_ee = self.tf_calculator.calculate_leader_to_end_effector_transform(target_yaw)
        
        cos_yaw = math.cos(target_yaw)
        sin_yaw = math.sin(target_yaw)
        
        total_offset_x = (assembly_to_leader['offset_x'] * cos_yaw - 
                         assembly_to_leader['offset_y'] * sin_yaw + leader_to_ee['x'])
        total_offset_y = (assembly_to_leader['offset_x'] * sin_yaw + 
                         assembly_to_leader['offset_y'] * cos_yaw + leader_to_ee['y'])
        total_offset_z = assembly_to_leader['offset_z'] + leader_to_ee['z']
        
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
            rospy.loginfo("Formation ready - all UAVs reporting positions")
            return True
        else:
            rospy.logerr(f"Formation not ready after {timeout}s timeout")
            return False
    
    def get_end_effector_position(self):
        """Calculate current end-effector position using coordinate transformation"""
        if self.assembly_pos is None:
            return None
        
        return self.tf_calculator.transform_assembly_to_end_effector(
            self.assembly_pos, self.assembly_yaw
        )
    
    def get_end_effector_yaw(self):
        """End-effector yaw is same as assembly yaw"""
        return self.assembly_yaw
    
    def send_assembly_command(self, target_pos, target_yaw=None, linear_vel=None):
        """Send FlightNav command to assembly controller"""
        try:
            if self.assembly_pos is None:
                rospy.logwarn("Assembly position not available - command may be ineffective")
            
            nav_msg = FlightNav()
            nav_msg.header.stamp = rospy.Time.now()
            nav_msg.header.frame_id = "world"
            
            nav_msg.target = FlightNav.COG
            
            nav_msg.target_pos_x = target_pos[0]
            nav_msg.target_pos_y = target_pos[1] 
            nav_msg.target_pos_z = target_pos[2]
            
            if linear_vel is not None:
                nav_msg.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
                nav_msg.pos_z_nav_mode = FlightNav.POS_VEL_MODE
                nav_msg.target_vel_x = linear_vel[0]
                nav_msg.target_vel_y = linear_vel[1]
                nav_msg.target_vel_z = linear_vel[2]
            else:
                nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                nav_msg.target_vel_x = 0.0
                nav_msg.target_vel_y = 0.0
                nav_msg.target_vel_z = 0.0
            
            if target_yaw is not None:
                nav_msg.target_yaw = target_yaw
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE
            else:
                nav_msg.yaw_nav_mode = FlightNav.POS_MODE
                nav_msg.target_yaw = 0.0
            
            self.assembly_pub.publish(nav_msg)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Error sending assembly command: {e}")
            return False


class FormationAssembleState(smach.State):
    """Execute physical assembly of two UAVs"""
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        
        modules_str = rospy.get_param("~module_ids", "1,2")
        real_machine = rospy.get_param("~real_machine", False)
        modules = []
        if modules_str:
            modules = [int(x) for x in modules_str.split(',')]
        else:
            rospy.logerr("No module ID is designated!")
        
        rospy.loginfo(f"Formation assembly configured for modules: {modules}, real_machine: {real_machine}")
        
        self.assemble_demo = AssemblyDemo(module_ids=modules, real_machine=real_machine)
        rospy.loginfo("Assembly demo loaded from task.assembly_motion")
    
    def execute(self, userdata):
        rospy.loginfo("=== ASSEMBLING UAVs ===")
        try:
            self.assemble_demo.main()
            rospy.loginfo("Assembly completed successfully")
            return 'succeeded'
        except rospy.ROSInterruptException:
            rospy.logerr("Assembly process interrupted")
            return 'failed'
        except Exception as e:
            rospy.logerr(f"Assembly process failed: {e}")
            return 'failed'


class FormationSingleUAVStateBase(smach.State):
    """Base class for formation states that reuse single UAV logic"""
    
    _shared_target_z = None
    
    def __init__(self, outcomes, input_keys=None, output_keys=None):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys or [], output_keys=output_keys or [])
        
        module_ids_str = rospy.get_param("~module_ids", "2,3") 
        
        self.formation_adapter = FormationAdapter(module_ids_str)
        self.leader_id = self.formation_adapter.get_leader_id()
        
        if not self.formation_adapter.wait_for_formation_ready(timeout=15.0):
            raise RuntimeError("Formation not ready - cannot initialize state")
        
        self.beetle = BeetleInterface(
            module_id=self.leader_id,
            assembly_mode=True,
            assembly_tf_calculator=self.formation_adapter.tf_calculator
        )
        
        self.optimizer = InsertionOptimizer()
        
        self.max_linear_velocity = 0.12
        self.average_linear_velocity = 0.08
        self.max_angular_velocity = 0.02
        
        rospy.loginfo(f"FormationState initialized for leader UAV{self.leader_id}")
    
    def get_assembly_position(self):
        return self.formation_adapter.get_assembly_position()
    
    def get_assembly_yaw(self):
        return self.formation_adapter.get_assembly_yaw()
    
    def get_end_effector_position(self):
        return self.formation_adapter.get_end_effector_position()
    
    def get_end_effector_yaw(self):
        return self.formation_adapter.get_end_effector_yaw()
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def active_position_convergence(self, target_pos, target_yaw, pos_thresh=0.025, yaw_thresh=0.0175, timeout=15.0, max_yaw_step=None, max_linear_vel=None, max_angular_vel=None):
        """Formation-style active convergence with trajectory decomposition"""
        start_time = rospy.get_time()
        
        current_pos = self.get_end_effector_position()
        if current_pos is None:
            rospy.logerr("[Convergence] Cannot get current position, skipping decomposition")
        
        if current_pos is not None:
            initial_distance = np.linalg.norm(np.array(target_pos) - np.array(current_pos))
            VEL_NAV_THRESHOLD = 0.05
            SAFE_STEP_SIZE = 0.04
            
            if initial_distance > VEL_NAV_THRESHOLD:
                rospy.loginfo(f"[Trajectory Decomposition] Distance {initial_distance*1000:.1f}mm > {VEL_NAV_THRESHOLD*1000:.0f}mm threshold")
                
                num_steps = max(2, int(np.ceil(initial_distance / SAFE_STEP_SIZE)))
                rospy.loginfo(f"Executing {num_steps} waypoints, ~{SAFE_STEP_SIZE*1000:.0f}mm per step")
                
                # Get speed limits for trajectory decomposition (defaults if not provided)
                decomp_linear_vel_limit = max_linear_vel if max_linear_vel is not None else 0.08
                decomp_angular_vel_limit = max_angular_vel if max_angular_vel is not None else 0.05
                
                for i in range(1, num_steps):
                    alpha = i / num_steps
                    intermediate_pos = (
                        current_pos[0] + alpha * (target_pos[0] - current_pos[0]),
                        current_pos[1] + alpha * (target_pos[1] - current_pos[1]),
                        current_pos[2] + alpha * (target_pos[2] - current_pos[2])
                    )
                    
                    step_vector = np.array(intermediate_pos) - np.array(current_pos)
                    step_distance = np.linalg.norm(step_vector)
                    
                    if step_distance > 0.001:
                        step_direction = step_vector / step_distance
                        controlled_linear_vel = [
                            step_direction[0] * decomp_linear_vel_limit,
                            step_direction[1] * decomp_linear_vel_limit,
                            step_direction[2] * decomp_linear_vel_limit
                        ]
                    else:
                        controlled_linear_vel = None
                    
                    self.send_assembly_command_from_end_effector(
                        intermediate_pos, 
                        target_yaw,
                        linear_vel=controlled_linear_vel,
                        angular_vel=decomp_angular_vel_limit
                    )
                    
                    rospy.sleep(0.15)
                    
                    current_pos = self.get_end_effector_position()
                    if current_pos is None:
                        rospy.logwarn(f"[Decomposition] Lost position at waypoint {i}, using predicted position")
                        current_pos = intermediate_pos
                    
                    if i % max(1, num_steps // 5) == 0:
                        rospy.loginfo(f"Waypoint {i}/{num_steps-1} ({100*i/num_steps:.0f}%)")
                
                rospy.loginfo("Trajectory decomposition complete")
                current_pos = self.get_end_effector_position()
                if current_pos is not None:
                    remaining = np.linalg.norm(np.array(target_pos) - np.array(current_pos))
                    rospy.loginfo(f"Remaining distance: {remaining*1000:.1f}mm")
        
        max_angular_vel_limit = max_angular_vel if max_angular_vel is not None else 0.05
        max_linear_vel_limit = max_linear_vel if max_linear_vel is not None else 0.08
        
        consecutive_good_readings = 0
        required_consecutive = 8
        
        rospy.loginfo(f"Formation active convergence: target=({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}), "
                     f"yaw={math.degrees(target_yaw):.1f}°")
        rospy.loginfo(f"Thresholds: pos={pos_thresh*1000:.1f}mm, yaw={math.degrees(yaw_thresh):.1f}°, consecutive={required_consecutive}")
        
        while rospy.get_time() - start_time < timeout:
            current_pos = self.get_end_effector_position()
            if current_pos is None:
                rospy.sleep(0.1)
                continue
            
            current_yaw = self.get_end_effector_yaw()
            if current_yaw is None:
                rospy.sleep(0.1)
                continue
            
            pos_error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
            yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
            
            position_ok = pos_error < pos_thresh
            yaw_ok = yaw_error < yaw_thresh
            
            if position_ok and yaw_ok:
                consecutive_good_readings += 1
                if consecutive_good_readings >= required_consecutive:
                    rospy.loginfo(f"Formation active convergence successful: pos_err={pos_error*1000:.1f}mm, "
                                f"yaw_err={math.degrees(yaw_error):.1f}°")
                    return True
            else:
                consecutive_good_readings = 0
                
                # Apply max_yaw_step limitation like Formation version
                command_yaw = target_yaw
                if max_yaw_step is not None and max_yaw_step > 0.0:
                    yaw_delta = self.normalize_angle(target_yaw - current_yaw)
                    if abs(yaw_delta) > max_yaw_step:
                        yaw_delta = math.copysign(max_yaw_step, yaw_delta)
                        command_yaw = self.normalize_angle(current_yaw + yaw_delta)
                        elapsed = rospy.get_time() - start_time
                        if int(elapsed * 5.0) % 20 == 0:
                            rospy.loginfo(f"[Formation YawLimit] Step limited to {math.degrees(yaw_delta):.1f}° "
                                        f"(remaining {math.degrees(abs(self.normalize_angle(target_yaw - current_yaw))):.1f}°)")
                
                command_pos = target_pos
                
                max_angular_vel_limit = max_angular_vel if max_angular_vel is not None else 0.05
                max_linear_vel_limit = max_linear_vel if max_linear_vel is not None else 0.08
                
                actual_yaw_error = abs(self.normalize_angle(command_yaw - current_yaw))
                
                if actual_yaw_error > 0.52:
                    angular_divisor = 2.0
                elif actual_yaw_error > 0.17:
                    angular_divisor = 2.5
                else:
                    angular_divisor = 4.0
                
                smooth_angular_vel = min(actual_yaw_error / angular_divisor, max_angular_vel_limit) if actual_yaw_error > 0.005 else 0.0
                
                if pos_error > 0.02:
                    speed_magnitude = min(pos_error / 4.0, max_linear_vel_limit)
                    direction = np.array(target_pos) - np.array(current_pos)
                    direction_norm = np.linalg.norm(direction)
                    if direction_norm > 0.001:
                        direction = direction / direction_norm
                        smooth_linear_vel = [direction[0] * speed_magnitude, 
                                           direction[1] * speed_magnitude, 
                                           direction[2] * speed_magnitude]
                    else:
                        smooth_linear_vel = None
                else:
                    smooth_linear_vel = None
                
                self.send_assembly_command_from_end_effector(
                    target_end_effector_pos=command_pos,
                    target_yaw=command_yaw,
                    linear_vel=smooth_linear_vel,
                    angular_vel=smooth_angular_vel
                )
                
                elapsed = rospy.get_time() - start_time
                if int(elapsed * 2.0) % 10 == 0:
                    rospy.loginfo(f"[Converging] pos_err={pos_error*1000:.1f}mm, "
                                f"yaw_err={math.degrees(yaw_error):.1f}°, t={elapsed:.1f}s")
            
            rospy.sleep(0.04)
        
        rospy.logwarn(f"Formation active convergence timeout after {timeout:.1f}s: "
                     f"pos_err={pos_error*1000:.1f}mm, yaw_err={math.degrees(yaw_error):.1f}°")
        return False

    def send_assembly_command_from_end_effector(self, target_end_effector_pos, target_yaw=None,
                                               linear_vel=None, angular_vel=None):
        """Send assembly command by transforming end-effector target to assembly CoG"""
        assembly_target = self.formation_adapter.transform_end_effector_to_assembly_command(
            target_end_effector_pos, target_yaw
        )

        ee_target_vec = np.array(target_end_effector_pos, dtype=float)
        assembly_target_vec = np.array(assembly_target, dtype=float)
        
        linear_vel_vec = None
        if linear_vel is not None:
            if isinstance(linear_vel, (int, float)):
                linear_vel_vec = np.array([0.0, 0.0, float(linear_vel)], dtype=float)
            else:
                linear_array = np.array(linear_vel, dtype=float).flatten()
                if linear_array.size == 3:
                    linear_vel_vec = linear_array
                else:
                    rospy.logwarn_once("Invalid linear_vel size; ignoring")

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
                    rospy.logwarn_once("Invalid angular_vel size; ignoring")
                    omega_vec = np.zeros(3, dtype=float)

        angular_vel_cmd = omega_vec.tolist() if not np.allclose(omega_vec, 0.0) else None

        assembly_linear_vec = None
        if linear_vel_vec is not None or angular_vel_cmd is not None:
            v_ee = linear_vel_vec if linear_vel_vec is not None else np.zeros(3, dtype=float)
            ee_offset = ee_target_vec - assembly_target_vec
            v_assembly = v_ee - np.cross(omega_vec, ee_offset)
            assembly_linear_vec = v_assembly.tolist()
        
        self.beetle.targetMotion(
            pos=assembly_target,
            rot=target_yaw,
            linear_vel=assembly_linear_vec,
            angular_vel=angular_vel_cmd
        )


    def controlled_z_descent(self, start_pos, final_target, final_yaw, descent_speed=0.12):
        """Controlled Z-axis descent in fixed steps with contact detection"""
        rospy.loginfo("=== Formation Z Descent ===")

        if start_pos is None or final_target is None:
            rospy.logerr("[Z Descent] Missing start or target position")
            return False, None

        target_x, target_y, target_z = final_target
        start_z = start_pos[2]
        total_z_descent = start_z - target_z
        rospy.loginfo(f"[Z Descent] Total descent {total_z_descent*1000:.1f}mm")

        if total_z_descent <= 0.01:
            rospy.loginfo("[Z Descent] Small descent, direct convergence")
            self.send_assembly_command_from_end_effector((target_x, target_y, target_z), final_yaw)
            success = self.active_position_convergence(
                (target_x, target_y, target_z),
                target_yaw=final_yaw,
                pos_thresh=0.080,  # RELAXED: 80mm (consistent with descent strategy)
                yaw_thresh=0.087,  # RELAXED: 5 degrees (0.087 rad)
                timeout=10.0
            )
            achieved = self.get_end_effector_position() or (target_x, target_y, target_z)
            return success, achieved

        # Step size parameters
        fixed_step_size = 0.05
        planned_steps = max(3, int(math.ceil(total_z_descent / fixed_step_size)))
        max_single_step = 0.03

        rospy.loginfo(f"[Z Descent] {planned_steps} steps planned, {fixed_step_size*1000:.1f}mm per step")

        current_pos = start_pos
        previous_z = current_pos[2]
        total_descent_completed = 0.0
        step_count = 0
        progress_ratio = 0.0
        max_step_iterations = max(planned_steps * 6, planned_steps + 40)
        achieved_position = current_pos
        contact_xy_tolerance = 0.080  # 80mm (RELAXED from 22mm for formation)
        contact_z_tolerance = 0.05
        plateau_z_tolerance = 0.25
        plateau_progress_threshold = 0.4
        stagnation_threshold = 0.012
        contact_required_hits = 3
        plateau_required_hits = 5
        stagnation_hit_counter = 0
        proximity_hit_counter = 0
        final_descent_margin = 0.03
        
        # Motion tracking for contact detection
        consecutive_small_motions = 0
        small_motion_threshold = 0.008
        max_consecutive_small_motions = 2

        while previous_z - target_z > 1e-4:
            if step_count >= max_step_iterations:
                # Check if within tolerance despite timeout
                current_pos = self.get_end_effector_position() or achieved_position
                final_xy_error = math.sqrt((current_pos[0] - target_x)**2 + (current_pos[1] - target_y)**2)
                final_z_error = abs(current_pos[2] - target_z)
                
                if final_xy_error <= 0.030 and final_z_error <= 0.020:
                    rospy.logwarn(
                        f"[Z Descent] Timeout but within tolerance: XY {final_xy_error*1000:.1f}mm, "
                        f"Z {final_z_error*1000:.1f}mm"
                    )
                    return True, current_pos
                
                rospy.logerr("[Z Descent] Exceeded safety iteration limit")
                return False, achieved_position

            remaining_descent = max(0.0, previous_z - target_z)
            if remaining_descent <= final_descent_margin:
                rospy.loginfo(f"[Z Descent] Remaining {remaining_descent*1000:.1f}mm within margin")
                break

            # Adaptive step size for final approach
            if remaining_descent <= 0.06:
                adaptive_step_size = 0.015
            elif remaining_descent <= 0.12:
                adaptive_step_size = 0.025
            else:
                adaptive_step_size = fixed_step_size
            
            step_size = min(adaptive_step_size, max_single_step, remaining_descent)
            current_z = previous_z - step_size
            if current_z < target_z:
                current_z = target_z

            step_count += 1
            progress = progress_ratio

            step_target = [target_x, target_y, current_z]

            # RELAXED: Formation control requires much larger XY tolerance during descent
            base_threshold = 0.080  # 80mm base threshold (RELAXED from 50mm)
            final_threshold = 0.080  # 80mm final threshold (keep consistent)
            step_pos_thresh = base_threshold - (base_threshold - final_threshold) * progress
            step_pos_thresh = max(step_pos_thresh, 0.080)  # Never go below 80mm
            step_yaw_thresh = 0.087  # 5 degrees (relaxed from 1 degree)
            is_final_step = current_z <= target_z + 1e-4 or remaining_descent <= step_size + 1e-6
            is_second_last_step = (planned_steps - step_count) == 2
            is_third_last_step = (planned_steps - step_count) == 3

            # Unified descent speed strategy - proven 0.036 m/s works well for all steps
            effective_descent_speed = max(0.03, descent_speed * 0.3)
            linear_vel = [0.0, 0.0, -effective_descent_speed]
            rospy.loginfo(f"  Descent speed: {effective_descent_speed:.3f} m/s")

            rospy.loginfo(
                f"[Z Descent] Step {step_count}/{planned_steps} target=({step_target[0]:.3f}, {step_target[1]:.3f}, {step_target[2]:.3f}), "
                f"threshold={step_pos_thresh*1000:.1f}mm (remaining {remaining_descent*1000:.1f}mm)"
            )

            self.send_assembly_command_from_end_effector(step_target, final_yaw, linear_vel=linear_vel, angular_vel=0.0)

            # Extended timeout for final steps (keep longer timeout, but remove max_pos_step restriction)
            if is_final_step or is_second_last_step or is_third_last_step:
                step_timeout = 17.0
                step_max_attempts = 100
                rospy.loginfo(f"  Extended timeout: {step_timeout}s")
            else:
                step_timeout = 12.0
                step_max_attempts = 80

            # Unified convergence strategy for all steps (removed max_pos_step to prevent control instability)
            step_converged = self.active_position_convergence(
                target_pos=step_target,
                target_yaw=final_yaw,
                pos_thresh=step_pos_thresh,
                yaw_thresh=step_yaw_thresh,
                timeout=step_timeout
                # NOTE: max_pos_step removed - it caused Step 8 divergence in testing
                # The 20mm step limit prevented quick correction when coordinate transform fluctuates
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

                # Check small motion during convergence failure
                if last_motion <= small_motion_threshold:
                    consecutive_small_motions += 1
                    rospy.loginfo(f"[Z Descent] Small motion detected {consecutive_small_motions}/2: dZ={last_motion*1000:.1f}mm")
                    
                    if consecutive_small_motions >= max_consecutive_small_motions:
                        # Physical contact detected - 2 consecutive small motions
                        if xy_residual <= 0.030:
                            rospy.loginfo(
                                f"[Z Descent] Contact success: 2 small motions + XY {xy_residual*1000:.1f}mm ≤30mm"
                            )
                            rospy.loginfo(f"[Z Descent] Physical limit reached at Z={achieved_position[2]:.3f}m")
                            rospy.loginfo(f"[Z Descent] Z residual {z_residual*1000:.1f}mm due to contact")
                            return True, achieved_position
                        elif z_residual <= 0.060 and xy_residual <= 0.040:
                            rospy.loginfo(
                                f"[Z Descent] Z tolerance success: Z_res={z_residual*1000:.1f}mm ≤60mm, "
                                f"XY={xy_residual*1000:.1f}mm ≤40mm"
                            )
                            rospy.loginfo(f"[Z Descent] Accepting insertion depth Z={achieved_position[2]:.3f}m")
                            return True, achieved_position
                        else:
                            rospy.logwarn(f"[Z Descent] Small motion but insufficient precision: XY={xy_residual*1000:.1f}mm, Z_res={z_residual*1000:.1f}mm")
                else:
                    consecutive_small_motions = 0

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
                    contact_label = "Contact tolerance" if within_contact_window else "Plateau detection"
                    hit_note = f"prox={proximity_hit_counter}, stagnation={stagnation_hit_counter}"
                    rospy.logwarn(
                        f"[Formation Z Descent] Step {step_count} {contact_label}: XY {xy_residual*1000:.1f}mm, "
                        f"Z residual {z_residual*1000:.1f}mm, locking current height ({hit_note}, ΔZ threshold≤{stagnation_threshold*1000:.1f}mm)"
                    )
                    target_z = achieved_position[2]
                    previous_z = achieved_position[2]
                    proximity_hit_counter = 0
                    stagnation_hit_counter = 0
                    break

                # Tolerance-oriented success check
                z_tolerance_success = abs(z_residual) <= 0.040
                xy_reasonable_success = xy_residual <= 0.030
                
                if z_tolerance_success and xy_reasonable_success:
                    rospy.loginfo(
                        f"[Z Descent] Tolerance success Step {step_count}: "
                        f"XY {xy_residual*1000:.1f}mm ≤30mm, Z_err {abs(z_residual)*1000:.1f}mm ≤40mm"
                    )
                    rospy.loginfo(f"[Z Descent] Best feasible position reached Z={achieved_position[2]:.3f}m")
                    return True, achieved_position
                
                rospy.logerr(
                    f"[Z Descent] Step {step_count} convergence failed (XY {xy_residual*1000:.1f}mm, "
                    f"Z_res {z_residual*1000:.1f}mm)"
                )
                return False, achieved_position

            current_pos = self.get_end_effector_position() or tuple(step_target)
            actual_descent = max(0.0, previous_z - current_pos[2])
            
            # Consecutive small motion detection on success path
            if actual_descent <= small_motion_threshold:
                if step_converged:
                    consecutive_small_motions += 1
                    rospy.loginfo(f"[Z Descent] Small motion on success {consecutive_small_motions}/2: dZ={actual_descent*1000:.1f}mm")
                
                if consecutive_small_motions >= max_consecutive_small_motions:
                    # Physical contact stable - 2 consecutive small motions
                    current_xy_error = math.sqrt((current_pos[0] - target_x)**2 + (current_pos[1] - target_y)**2)
                    current_z_error = abs(current_pos[2] - target_z)
                    
                    if current_xy_error <= 0.030:
                        rospy.loginfo(
                            f"[Z Descent] Physical contact stable: 2 small motions + XY {current_xy_error*1000:.1f}mm ≤30mm"
                        )
                        rospy.loginfo(
                            f"[Z Descent] Cannot descend further, Z_err {current_z_error*1000:.1f}mm at contact position"
                        )
                        rospy.loginfo(f"[Z Descent] Stable insertion Z={current_pos[2]:.3f}m (target {target_z:.3f}m)")
                        return True, current_pos
                    else:
                        rospy.logwarn(
                            f"[Z Descent] Small motion but XY insufficient: XY {current_xy_error*1000:.1f}mm>30mm, "
                            f"Z {current_z_error*1000:.1f}mm"
                        )
            else:
                if step_converged:
                    consecutive_small_motions = 0
            
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
            if xy_error > 0.080:  # RELAXED: 80mm threshold (consistent with descent tolerance)
                rospy.logwarn(f"[Formation Z Descent] XY error {xy_error*1000:.1f}mm, applying XY correction")
                correction_target = [step_target[0], step_target[1], current_pos[2]]
                self.send_assembly_command_from_end_effector(correction_target, final_yaw)
                corrected = self.active_position_convergence(
                    target_pos=correction_target,
                    target_yaw=final_yaw,
                    pos_thresh=0.080,  # RELAXED: 80mm convergence threshold
                    yaw_thresh=0.087,  # RELAXED: 5 degrees (0.087 rad)
                    timeout=8.0
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

    def active_stabilization_wait(self, target_pos, target_yaw, duration, description="position stabilization"):
        """
        Active stabilization: continuously send target commands during wait period.
        
        Args:
            target_pos: Target end-effector position (x, y, z)
            target_yaw: Target yaw angle (radians)
            duration: Wait duration in seconds
            description: Description for logging
        """
        rospy.loginfo(f"Active stabilization: {duration}s {description}")
        rospy.loginfo(f"  Target: pos=({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}), yaw={math.degrees(target_yaw):.1f}°")
        
        start_time = rospy.get_time()
        rate = rospy.Rate(10)
        
        while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
            self.send_assembly_command_from_end_effector(target_pos, target_yaw)
            
            # Log progress every second
            elapsed = rospy.get_time() - start_time
            if int(elapsed) != int(elapsed - 0.1):
                current_pos = self.get_end_effector_position()
                current_yaw = self.get_end_effector_yaw()
                if current_pos is not None and current_yaw is not None:
                    pos_error = ((target_pos[0] - current_pos[0])**2 + 
                                (target_pos[1] - current_pos[1])**2 + 
                                (target_pos[2] - current_pos[2])**2)**0.5
                    yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
                    rospy.loginfo(f"  Progress: {elapsed:.1f}s, pos_err={pos_error*1000:.1f}mm, yaw_err={math.degrees(yaw_error):.1f}°")
            
            rate.sleep()
        
        rospy.loginfo(f"Stabilization complete: {duration}s {description}")

    # 已删除 convert_wrench_to_list 方法 - 直接使用轨迹生成器的numpy格式输出
    
    def generate_polynomial_trajectory(self, start_pos, target_pos, target_yaw, num_points=15):
        """
        Generate smooth polynomial trajectory using PolynomialTrajectory class (Formation version)
        Shared method for all formation states
        """
        rospy.loginfo(f"=== Generating Formation Polynomial Trajectory: {num_points} points ===")
        
        # Get current state for trajectory planning
        current_yaw = self.get_end_effector_yaw()
        if current_yaw is None:
            current_yaw = 0.0
        
        # Calculate total distance and trajectory time
        total_distance = math.sqrt(sum((target_pos[i] - start_pos[i])**2 for i in range(3)))
        yaw_change = abs(self.normalize_angle(target_yaw - current_yaw))
        
        # Calculate trajectory time based on formation parameters
        time_for_distance = total_distance / self.average_linear_velocity
        time_for_rotation = yaw_change / self.max_angular_velocity
        
        trajectory_duration = max(
            time_for_distance,
            time_for_rotation,
            8.0  # Minimum 8 seconds for smooth motion
        )
        
        # Apply safety factor
        safety_factor = 0.7
        trajectory_duration = trajectory_duration / safety_factor
        
        actual_avg_velocity = total_distance / trajectory_duration if trajectory_duration > 0 else 0
        
        rospy.loginfo(f"Trajectory params: distance={total_distance:.3f}m, angle={math.degrees(yaw_change):.1f}°")
        rospy.loginfo(f"Duration={trajectory_duration:.1f}s, avg_vel={actual_avg_velocity:.3f}m/s")
        
        # Create polynomial trajectories
        traj_x = PolynomialTrajectory(duration=trajectory_duration)
        traj_y = PolynomialTrajectory(duration=trajectory_duration)
        traj_z = PolynomialTrajectory(duration=trajectory_duration)
        traj_yaw = PolynomialTrajectory(duration=trajectory_duration)
        
        # Compute coefficients
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
        
        trajectory_points = []
        
        # Generate trajectory points
        for i in range(num_points + 1):
            elapsed_time = (i / num_points) * trajectory_duration
            
            pos_x = traj_x.evaluate_at_time(elapsed_time)
            pos_y = traj_y.evaluate_at_time(elapsed_time)
            pos_z = traj_z.evaluate_at_time(elapsed_time)
            yaw = traj_yaw.evaluate_at_time(elapsed_time)
            
            # Calculate velocity
            normalized_time = elapsed_time / trajectory_duration
            
            if normalized_time <= 0 or normalized_time >= 1:
                vel_x = vel_y = vel_z = 0.0
            else:
                T_vel = np.array([5*normalized_time**4, 4*normalized_time**3, 3*normalized_time**2, 
                                 2*normalized_time, 1, 0]) / trajectory_duration
                
                vel_x = np.dot(traj_x.coeffs_scalar, T_vel)
                vel_y = np.dot(traj_y.coeffs_scalar, T_vel)
                vel_z = np.dot(traj_z.coeffs_scalar, T_vel)
            
            # Apply trapezoidal velocity profile
            progress = normalized_time
            if progress < 0.2:
                velocity_factor = 0.6 + 0.4 * (progress / 0.2)
            elif progress > 0.8:
                velocity_factor = 0.6 + 0.4 * ((1.0 - progress) / 0.2)
            else:
                velocity_factor = 1.0
            
            vel_x *= velocity_factor
            vel_y *= velocity_factor
            vel_z *= velocity_factor
            
            pos = [pos_x, pos_y, pos_z]
            velocity = [vel_x, vel_y, vel_z]
            
            vel_magnitude = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
            if vel_magnitude > self.max_linear_velocity:
                scale_factor = self.max_linear_velocity / vel_magnitude
                velocity = [v * scale_factor for v in velocity]
                vel_magnitude = self.max_linear_velocity
            
            trajectory_points.append((pos, yaw, velocity, vel_magnitude))
        
        rospy.loginfo(f"Generated {len(trajectory_points)} trajectory points")
        return trajectory_points

    def execute_polynomial_trajectory(self, trajectory_points):
        """
        Execute polynomial trajectory using active convergence for each point
        Shared method for all formation states
        """
        rospy.loginfo(f"Executing Formation Polynomial Trajectory: {len(trajectory_points)} points")
        
        total_points = len(trajectory_points)
        
        for i, (pos, yaw, velocity, vel_magnitude) in enumerate(trajectory_points):
            if i % 3 == 0 or i == total_points - 1:
                rospy.loginfo(f"Trajectory point {i+1}/{total_points}: "
                             f"pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], "
                             f"yaw={math.degrees(yaw):.1f}°")
            
            is_final_precision_point = (i >= total_points - 2)
            
            # Use base class active_position_convergence (compatible with both versions)
            if is_final_precision_point:
                if i % 3 == 0:
                    rospy.loginfo(f"  Using precision convergence mode (point {i+1})")
                point_success = self.active_position_convergence(
                    target_pos=pos,
                    target_yaw=yaw,
                    pos_thresh=0.125,
                    yaw_thresh=0.08,
                    timeout=15.0
                )
            else:
                if i % 3 == 0:
                    rospy.loginfo(f"  Using trajectory following mode (point {i+1})")
                point_success = self.active_position_convergence(
                    target_pos=pos,
                    target_yaw=yaw,
                    pos_thresh=0.08,
                    yaw_thresh=0.15,
                    timeout=3.0
                )
            
            if not point_success:
                rospy.logwarn(f"Trajectory point {i+1} convergence issues - continuing")
            else:
                if i % 3 == 0 or i == total_points - 1:
                    rospy.loginfo(f"Trajectory point {i+1} completed")
            
            # Continuous send for precision points
            if is_final_precision_point:
                rospy.loginfo(f"  Stabilizing point {i+1} for 2s")
                stabilize_end_time = rospy.Time.now().to_sec() + 2.0
                stabilize_rate = rospy.Rate(10)
                while rospy.Time.now().to_sec() < stabilize_end_time and not rospy.is_shutdown():
                    self.send_assembly_command_from_end_effector(pos, yaw)
                    stabilize_rate.sleep()
            
            # Dynamic pause
            if is_final_precision_point:
                rospy.sleep(0.2)
            else:
                rospy.sleep(0.067)
        
        rospy.loginfo("Polynomial trajectory execution completed")
        return True


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

        # Optional Z offset compensation (disabled by default)
        formation_z_offset = rospy.get_param("~formation_phase4_z_offset", 0.0)
        if abs(formation_z_offset) > 1e-4:
            compensated_z = safe_ee_pos[2] + formation_z_offset
            rospy.loginfo(
                f"[Phase4] Apply Z compensation {formation_z_offset*1000:.0f}mm -> {compensated_z:.3f}m"
            )
            safe_ee_pos = (safe_ee_pos[0], safe_ee_pos[1], compensated_z)
        else:
            rospy.loginfo(f"[Phase4] Using optimizer target: {safe_ee_pos[2]:.3f}m (valve height)")
        
        # Validate target height matches valve height
        expected_valve_height = valve_pos[2]
        actual_target_height = safe_ee_pos[2]
        height_difference = abs(actual_target_height - expected_valve_height)
        if height_difference > 0.01:
            rospy.logwarn(f"Phase4 target {actual_target_height:.3f}m differs from valve {expected_valve_height:.3f}m by {height_difference*1000:.1f}mm")
        else:
            rospy.loginfo(f"Phase4 target {actual_target_height:.3f}m matches valve {expected_valve_height:.3f}m (diff: {height_difference*1000:.1f}mm)")
        
        # Share target Z for Contact phase consistency
        FormationSingleUAVStateBase._shared_target_z = safe_ee_pos[2]
        rospy.loginfo(f"Z continuity: saved target Z={safe_ee_pos[2]:.3f}m")

        rospy.loginfo(f"Current EE: {self._format_vec(current_ee_pos)}, yaw={math.degrees(current_yaw):.1f}°")
        rospy.loginfo(f"Valve pose: pos={self._format_vec(valve_pos)}, yaw={math.degrees(valve_yaw):.1f}°")
        rospy.loginfo(f"Optimizer target: pos={self._format_vec(safe_ee_pos)}, yaw={math.degrees(safe_ee_yaw):.1f}°")

        # --- PHASE 1: YAW ADJUSTMENT TO VALVE HANDLE (SKIPPED) ---
        # Yaw adjustment removed: focus on XY positioning only
        # Phase 3B will handle final yaw alignment if needed
        rospy.loginfo("[Phase 1] Skipped: Yaw adjustment removed, focusing on XY positioning")
        
        # --- PHASE 2: XY MOVEMENT TO OPTIMIZER TARGET (maintain Z, current yaw) ---
        phase1_ee_pos = self.get_end_effector_position()
        if phase1_ee_pos is None:
            rospy.logerr("Cannot get end-effector position before Phase 2")
            return 'failed'
        
        # Use current yaw since Phase 1 yaw adjustment is skipped
        current_yaw_for_phase2 = self.get_end_effector_yaw()
        if current_yaw_for_phase2 is None:
            rospy.logwarn("Cannot get current yaw, using 0.0 as default")
            current_yaw_for_phase2 = 0.0
        
        xy_target_ee_pos = (safe_ee_pos[0], safe_ee_pos[1], phase1_ee_pos[2])
        self._log_phase(2, "Move to safe XY", f"Target {self._format_vec(xy_target_ee_pos)}")
        if not self._execute_formation_phase2_xy_movement(xy_target_ee_pos, current_yaw_for_phase2):
            rospy.logerr("Phase 2 failed: XY movement unsuccessful")
            return 'failed'
        rospy.loginfo("Phase 2 completed: Reached optimizer XY position")

        # --- PHASE 3: PRECISION XY & YAW (spoke alignment, maintain Z) ---
        phase2_ee_pos = self.get_end_effector_position()
        phase2_yaw = self.get_end_effector_yaw()
        if phase2_ee_pos is None or phase2_yaw is None:
            rospy.logerr("Cannot get end-effector position/yaw after Phase 2")
            return 'failed'

        # PHASE 3A: XY precision positioning (keep Z, do not descend yet)
        phase3a_target_pos = (safe_ee_pos[0], safe_ee_pos[1], phase2_ee_pos[2])
        self._log_phase(3, "Precision XY", f"Lock Z {phase3a_target_pos[2]:.3f}m")
        if not self._execute_formation_phase3a_xy_positioning(phase3a_target_pos, phase2_yaw):
            rospy.logerr("Phase 3A failed: XY precision positioning unsuccessful")
            return 'failed'
        rospy.loginfo("  Phase 3A complete: XY precision achieved")

        # PHASE 3B: Spoke-aligned yaw (use optimizer's safe_ee_yaw)
        phase3a_ee_pos = self.get_end_effector_position()
        if phase3a_ee_pos is None:
            rospy.logerr("Cannot get end-effector position after Phase 3A")
            return 'failed'
        valve_xy_error = math.sqrt(
            (phase3a_ee_pos[0] - valve_pos[0])**2 +
            (phase3a_ee_pos[1] - valve_pos[1])**2
        )
        rospy.loginfo(f"Phase 3A end-effector to valve center: {valve_xy_error*1000:.1f}mm")
        optimal_spoke_yaw = self.calculate_shortest_yaw_path(phase2_yaw, safe_ee_yaw)
        rospy.loginfo(f"Phase 3B spoke alignment target yaw: {math.degrees(safe_ee_yaw):.1f}°")
        if not self._execute_formation_phase3b_spoke_alignment(phase3a_ee_pos, optimal_spoke_yaw, phase3a_target_pos):
            rospy.logerr("Phase 3B failed: Spoke alignment unsuccessful")
            return 'failed'
        rospy.loginfo("  Phase 3B complete: Yaw locked to spoke gap")
        phase3b_ee_pos = self.get_end_effector_position()
        if phase3b_ee_pos is not None:
            phase3b_xy_error = math.sqrt(
                (phase3b_ee_pos[0] - valve_pos[0])**2 +
                (phase3b_ee_pos[1] - valve_pos[1])**2
            )
            rospy.loginfo(f"  · Phase 3B 末端距阀心 {phase3b_xy_error*1000:.1f}mm")

        # --- PHASE 4: Z DESCENT TO OPTIMIZER HEIGHT (keep XY, spoke yaw) ---
        final_target_pos = (safe_ee_pos[0], safe_ee_pos[1], safe_ee_pos[2])

        final_yaw = self.get_end_effector_yaw()
        if final_yaw is None:
            final_yaw = safe_ee_yaw

        if not self._execute_formation_phase4_z_descent(final_target_pos, final_yaw):
            rospy.logerr("Phase 4 failed: Z descent unsuccessful")
            return 'failed'
        rospy.loginfo("Phase 4 完成：插入高度就位")

        # Share Phase4 locking pose and yaw with subsequent states
        phase4_pose = getattr(self, '_last_phase4_contact_position', None)
        if phase4_pose is None:
            phase4_pose = self.get_end_effector_position() or final_target_pos
        userdata.phase4_contact_pose = phase4_pose
        userdata.phase4_contact_yaw = self.get_end_effector_yaw() or final_yaw
        
        # Debug: Log Phase4 completion state (use logdebug to reduce verbosity)
        if rospy.get_param("~debug_verbose", False):
            current_ee_pos = self.get_end_effector_position()
            current_assembly_pos = self.get_assembly_position()
            current_ee_yaw = self.get_end_effector_yaw()
            rospy.logdebug("=" * 80)
            rospy.logdebug(f"[PHASE4_COMPLETE] Formation insertion complete:")
            rospy.logdebug(f"  Optimizer target (end-effector): {self._format_vec(final_target_pos)}")
            rospy.logdebug(f"  Current end-effector: {self._format_vec(current_ee_pos) if current_ee_pos else 'None'}")
            rospy.logdebug(f"  Current Assembly CoG: {self._format_vec(current_assembly_pos) if current_assembly_pos else 'None'}")
            rospy.logdebug(f"  Current yaw: {math.degrees(current_ee_yaw):.1f}°" if current_ee_yaw is not None else "  Current yaw: None")
            rospy.logdebug(f"  Passed to Contact phase: {self._format_vec(phase4_pose)}")
            contact_yaw = self.get_end_effector_yaw() or final_yaw
            rospy.logdebug(f"  Contact phase yaw: {math.degrees(contact_yaw):.1f}°" if contact_yaw is not None else "  Contact yaw: None")
            if current_ee_pos and current_assembly_pos:
                ee_assembly_diff = [(current_ee_pos[i] - current_assembly_pos[i]) * 1000 for i in range(3)]
                rospy.logdebug(f"  EE-Assembly offset: ({ee_assembly_diff[0]:.1f}, {ee_assembly_diff[1]:.1f}, {ee_assembly_diff[2]:.1f})mm")
            rospy.logdebug("=" * 80)

        # Use Phase4 converged position for stabilization
        if hasattr(self, '_last_phase4_contact_position') and self._last_phase4_contact_position is not None:
            target_pos = self._last_phase4_contact_position
            target_yaw = final_yaw
            rospy.loginfo("Starting 5s active stabilization using Phase4 position...")
            rospy.loginfo(f"  Target: pos={self._format_vec(target_pos)}, yaw={math.degrees(target_yaw):.1f}°")
            self.active_stabilization_wait(target_pos, target_yaw, 5.0, 
                                          "Post-insertion stabilization")
        else:
            rospy.logwarn("Phase4 converged position not found, using current position...")
            current_ee_pos = self.get_end_effector_position()
            current_ee_yaw = self.get_end_effector_yaw()
            if current_ee_pos and current_ee_yaw is not None:
                self.active_stabilization_wait(current_ee_pos, current_ee_yaw, 5.0, 
                                              "Formation插入完成后稳定")
            else:
                rospy.logwarn("无法获取当前end-effector位置，使用被动等待...")
                rospy.sleep(5.0)

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
                target_pos=xy_target_ee_pos,
                target_yaw=maintain_yaw,
                pos_thresh=0.050,  # RELAXED: 50mm (consistent strategy)
                yaw_thresh=0.087,  # RELAXED: 5 degrees (0.087 rad)
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
                target_pos=xy_target_ee_pos,
                target_yaw=maintain_yaw,
                pos_thresh=0.050,  # RELAXED: 50mm (consistent strategy)
                yaw_thresh=0.087,  # RELAXED: 5 degrees (0.087 rad)
                timeout=20.0
            )

        # Execute polynomial trajectory
        rospy.loginfo("[Phase2] 执行多项式轨迹")
        trajectory_success = self.execute_polynomial_trajectory(trajectory_points)

        # Final convergence check to ensure precision
        rospy.loginfo("[Phase2] 进行末端精度收敛检查")
        final_success = self.active_position_convergence(
            target_pos=xy_target_ee_pos,
            target_yaw=maintain_yaw,
            pos_thresh=0.050,           # Relaxed to 50mm for formation tolerance
            yaw_thresh=0.087,           # Relaxed to 5 degrees (0.087 rad)
            timeout=15.0
        )

        # Real machine optimization: Phase2 final position continuous send for 2s to ensure stable convergence
        rospy.loginfo("[Phase2] 最终位置稳定发送2秒")
        final_stabilize_end = rospy.Time.now().to_sec() + 2.0
        final_stabilize_rate = rospy.Rate(10)  # 10Hz持续发送
        while rospy.Time.now().to_sec() < final_stabilize_end and not rospy.is_shutdown():
            self.send_assembly_command_from_end_effector(xy_target_ee_pos, maintain_yaw)
            final_stabilize_rate.sleep()
        rospy.loginfo("[Phase2] 最终稳定发送完成")

        overall_success = trajectory_success and final_success

        if overall_success:
            rospy.loginfo("Phase 2 完成：轨迹执行与收敛均成功")
        else:
            rospy.logwarn("Phase 2 completed but has convergence warnings")

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
        rospy.loginfo(f"Valve center: ({valve_x:.3f}, {valve_y:.3f})")
        rospy.loginfo(f"Current EE pos: ({current_ee_pos[0]:.3f}, {current_ee_pos[1]:.3f})")  
        rospy.loginfo(f"Spoke angle: {math.degrees(spoke_angle):.1f}°")
        rospy.loginfo(f"Required spoke yaw: {math.degrees(spoke_yaw):.1f}°")
        
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
        xy_tolerance = 0.050  # 50mm (RELAXED: practical positioning threshold)
        z_tolerance = 0.015   # 15mm

        for attempt in range(1, max_attempts + 1):
            rospy.loginfo(f"[Phase3A] 尝试 {attempt}/{max_attempts}")
            success = self.active_position_convergence(
                target_pos=xy_target,
                target_yaw=maintain_yaw,
                pos_thresh=0.050,  # 50mm convergence threshold (RELAXED)
                yaw_thresh=0.087,  # 5 degrees (0.087 rad) - relaxed for tolerance
                timeout=5.0  # 5s timeout for quick attempts
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
                    rospy.logwarn("Phase 3A yaw deviation slightly large, will re-lock in subsequent stages")
                rospy.loginfo("Phase 3A XY precise positioning successful")
                return True

            rospy.logwarn("[Phase3A] Error still exceeds threshold, preparing retry")
        rospy.logerr("Phase 3A failed to achieve accuracy after multiple attempts")
        return False

    def _execute_formation_phase3b_spoke_alignment(self, current_ee_pos, spoke_yaw, target_pos=None):
        """
        Phase 3B: Spoke insertion yaw alignment.
        Rotate yaw while maintaining Phase 3A locked XY/Z position with automatic drift correction.
        """
        reference_xy = (
            target_pos[0],
            target_pos[1]
        ) if target_pos is not None else (current_ee_pos[0], current_ee_pos[1])
        reference_z = target_pos[2] if target_pos is not None else current_ee_pos[2]
        rospy.loginfo(
            f"[Phase3B] Target yaw {math.degrees(spoke_yaw):.1f}°, locked position {self._format_vec((reference_xy[0], reference_xy[1], reference_z))}"
        )

        max_attempts = 3
        xy_tolerance = 0.050  # 50mm (RELAXED for final check tolerance)
        z_tolerance = 0.015   # 15mm
        yaw_tolerance = 0.087  # 5 degrees (0.087 rad) - relaxed tolerance

        for attempt in range(1, max_attempts + 1):
            current_yaw = self.get_end_effector_yaw()
            if current_yaw is not None:
                yaw_error = abs(self.normalize_angle(spoke_yaw - current_yaw))
                rospy.loginfo(f"[Phase3B] 尝试 {attempt}/{max_attempts}，预计旋转 {math.degrees(yaw_error):.2f}°")

            success = self.active_position_convergence(
                target_pos=(reference_xy[0], reference_xy[1], reference_z),
                target_yaw=spoke_yaw,
                pos_thresh=0.050,  # 50mm convergence threshold (RELAXED)
                yaw_thresh=0.087,  # 5 degrees (0.087 rad) - relaxed threshold
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
                f"[Phase4] Contact depth locked at {achieved_pos[2]:.3f}m (planned {final_target_pos[2]:.3f}m)"
            )

        convergence_target = achieved_pos
        rospy.loginfo(
            f"[Phase4] Lock current position as final convergence target {self._format_vec(convergence_target)}"
        )

        self._last_phase4_contact_position = convergence_target

        rospy.loginfo("[Phase4] Performing final insertion pose convergence")
        final_success = self.active_position_convergence(
            target_pos=convergence_target,
            target_yaw=final_yaw,
            pos_thresh=0.080,  # 80mm convergence threshold (RELAXED for formation)
            yaw_thresh=0.087,  # 5 degrees (0.087 rad) - relaxed threshold
            timeout=12.0
        )

        # CRITICAL: Regardless of final_success, always save the actual insertion depth Z coordinate reached
        # Store Z coordinate for Contact/Rotation phases to avoid pitch errors
        FormationSingleUAVStateBase._shared_target_z = convergence_target[2]
        rospy.loginfo(f"Z continuity: saved _shared_target_z={FormationSingleUAVStateBase._shared_target_z:.3f}m")

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
        self.nominal_angular_velocity = 0.1
        self.control_rate = 25.0
        self.max_linear_speed = 0.035

    @staticmethod
    def _format_vec(vec):
        """Format 3D vector as string"""
        return f"({vec[0]:.3f}, {vec[1]:.3f}, {vec[2]:.3f})"

    @staticmethod
    def _normalize(self, angle):
        """Normalize angle to [-pi, pi]"""
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
        
        rospy.loginfo(">>> Phase 1: Establishing circular contact with valve")
        contact_result = self._execute_contact_phase(userdata)
        if contact_result != 'succeeded':
            return contact_result
            
        rospy.loginfo(">>> Phase 2: Executing valve rotation")
        rotation_result = self._execute_rotation_phase(userdata)
        return rotation_result
    
    def _execute_contact_phase(self, userdata):
        """Contact establishment phase - replicate FormationDescendAndContactState logic"""
        rospy.loginfo("Using TWO-PHASE SAFE INSERTION strategy")
        
        valve_pos = userdata.valve_position
        valve_yaw = userdata.valve_yaw
        phase4_contact_pos = getattr(userdata, 'phase4_contact_pose', None)
        phase4_contact_yaw = getattr(userdata, 'phase4_contact_yaw', None)

        # Cache valve info for formation helpers
        self.optimizer.update_valve_info(valve_pos, valve_yaw)

        # Get position info after insertion complete (fixed Z coordinate)
        current_ee_pos = self.get_end_effector_position()
        if current_ee_pos is None:
            rospy.logerr("Failed to get current end-effector position")
            return 'failed'

        rospy.loginfo(f"Current end-effector position: ({current_ee_pos[0]:.3f}, {current_ee_pos[1]:.3f}, {current_ee_pos[2]:.3f})")
        
        rospy.loginfo("SKIPPING Phase 1 & 2: FormationMoveToValveState Phase 4 already completed insertion positioning")
        
        # Phase 3: Circular Contact Establishment  
        # Use Assembly CoG coordinate system, avoid mixing with single UAV coordinate system
        current_assembly_pos = self.get_assembly_position()
        if current_assembly_pos is None:
            rospy.logerr("Cannot get Assembly CoG position, Contact phase failed")
            return 'failed'
            
        rospy.loginfo(f"Contact phase based on Assembly CoG coordinates: ({current_assembly_pos[0]:.3f}, {current_assembly_pos[1]:.3f}, {current_assembly_pos[2]:.3f})")
        rospy.loginfo(f"End-effector position used for trajectory calculation: ({current_ee_pos[0]:.3f}, {current_ee_pos[1]:.3f}, {current_ee_pos[2]:.3f})")
        contact_result = self._execute_circular_contact_phase(userdata, current_assembly_pos, valve_pos, valve_yaw)
        if not contact_result:
            return 'failed'

        # Position verification and insertion data setup
        valve_center = valve_pos
        # Use end-effector position for geometric calculations (but control target is Assembly CoG)
        if current_ee_pos is None:
            rospy.logerr("Cannot get end-effector position for geometric calculation")
            return 'failed'
            
        approach_angle = math.atan2(current_ee_pos[1] - valve_center[1], 
                                   current_ee_pos[0] - valve_center[0])
        spoke_angles = self.optimizer.get_spoke_angles()
        angle_diffs = [abs(angle - approach_angle) for angle in spoke_angles]
        selected_spoke = spoke_angles[angle_diffs.index(min(angle_diffs))]
        
        rospy.loginfo(f"Selected spoke angle: {selected_spoke:.3f} rad ({math.degrees(selected_spoke):.1f}°)")
        rospy.loginfo(f"Approach angle: {approach_angle:.3f} rad ({math.degrees(approach_angle):.1f}°)")

        contact_radius = math.sqrt((current_ee_pos[0] - valve_pos[0])**2 + 
                                 (current_ee_pos[1] - valve_pos[1])**2)
        rospy.loginfo(f"Current distance to valve center: {contact_radius*1000:.1f}mm")

        if contact_radius > 0.025:
            rospy.logwarn(f"Distance to valve center too large: {contact_radius*1000:.1f}mm")

        final_contact_pos = current_assembly_pos

        rospy.loginfo("Using current position without optimization (simplified for unified state)")

        cached_phase4_pose = phase4_contact_pos or self.get_end_effector_position()
        
        self.contact_radius = contact_radius
        self.contact_angle = selected_spoke
        self.valve_center = valve_pos
        self.initial_valve_yaw = valve_yaw
        self.rotation_start_position = final_contact_pos

        rospy.loginfo("Position verification passed - ready for valve rotation")
        rospy.loginfo(f"Baseline contact torque set to [0.0, 0.0, 0.1] (fixed positive value like Single UAV)")
        
        return 'succeeded'

    def _execute_circular_contact_phase(self, userdata, current_pos, valve_pos, initial_valve_yaw):
        """
        Circular contact establishment phase - adapted from single UAV version
        Executes circular motion to establish physical contact with valve before rotation
        """
        rospy.loginfo(">>> Starting circular contact establishment")
        
        # Calculate current radius from valve center
        relative_pos = np.array(current_pos[:2]) - np.array(valve_pos[:2])
        current_radius = np.linalg.norm(relative_pos)
        
        rospy.loginfo(f"Valve center: ({valve_pos[0]:.3f}, {valve_pos[1]:.3f}, {valve_pos[2]:.3f})")
        rospy.loginfo(f"Current radius: {current_radius*1000:.1f}mm")
        rospy.loginfo(f"Initial valve angle: {math.degrees(initial_valve_yaw):.1f}°")
        
        # ===== SINGLE UAV COMPATIBILITY: Correct valve_center in Contact phase =====
        contact_angular_velocity = 0.1  # 0.1 rad/s ≈ 5.7°/s, consistent with single UAV version
        
        # CRITICAL FIX: Correct valve_center Z coordinate to insertion depth in Contact phase, eliminate Z coordinate discontinuity
        original_valve_z = valve_pos[2]
        if FormationSingleUAVStateBase._shared_target_z is not None:
            corrected_valve_pos = [valve_pos[0], valve_pos[1], FormationSingleUAVStateBase._shared_target_z]
            rospy.loginfo(f"Contact phase valve_center correction: Z {original_valve_z:.3f}m → {FormationSingleUAVStateBase._shared_target_z:.3f}m (eliminate {abs(original_valve_z - FormationSingleUAVStateBase._shared_target_z)*1000:.0f}mm discontinuity)")
            valve_pos = corrected_valve_pos
        else:
            rospy.logwarn(f"Saved insertion Z coordinate not found, using original valve_center (Z={valve_pos[2]:.3f}m)")
        
        # Fix: Use Assembly CoG coordinate system, avoid mixing end-effector coordinates
        # Get current Assembly CoG position as trajectory basis
        current_assembly_pos = self.get_assembly_position()
        if current_assembly_pos is None:
            rospy.logerr("Cannot get Assembly CoG position, Contact phase failed")
            return False
            
        rospy.loginfo(f"Contact阶段基于Assembly CoG坐标: ({current_assembly_pos[0]:.3f}, {current_assembly_pos[1]:.3f}, {current_assembly_pos[2]:.3f})")
        rospy.loginfo(f"使用原始valve_center: ({valve_pos[0]:.3f}, {valve_pos[1]:.3f}, {valve_pos[2]:.3f})")
        
        # 计算当前end-effector相对于阀门中心的位置（用于轨迹初始化）
        current_ee_pos = self.get_end_effector_position()
        if current_ee_pos:
            rospy.loginfo(f"当前end-effector位置: ({current_ee_pos[0]:.3f}, {current_ee_pos[1]:.3f}, {current_ee_pos[2]:.3f})")
        
        # Initialize trajectory generator for contact phase
        trajectory_generator = OnlineCircularTrajectoryGenerator(
            valve_center=valve_pos,  # Use original valve_pos to avoid coordinate system mixing
            initial_radius=current_radius,
            target_angular_velocity=contact_angular_velocity,
            control_rate=self.control_rate
        )
        
        # Initialize trajectory generator with Assembly CoG position to avoid Z coordinate jumps
        rospy.loginfo("Trajectory generator init: using Assembly CoG position for Z coordinate consistency")
        rospy.loginfo(f"   Assembly CoG: ({current_assembly_pos[0]:.3f}, {current_assembly_pos[1]:.3f}, {current_assembly_pos[2]:.3f})")
        rospy.loginfo(f"   End-effector: ({current_ee_pos[0]:.3f}, {current_ee_pos[1]:.3f}, {current_ee_pos[2]:.3f})" if current_ee_pos else "   End-effector: N/A")
        trajectory_generator.initialize_from_current_position(current_assembly_pos)
        
        contact_threshold = 0.1
        rospy.loginfo(f"Contact establishment - target: {math.degrees(contact_threshold):.1f}° valve rotation")
        
        max_contact_time = 20.0
        target_valve_rotation = contact_threshold
        min_valve_rotation = math.radians(1.0)
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
            
            # Fix: Get Assembly CoG position for trajectory update, consistent with initialization
            current_assembly_pos_loop = self.get_assembly_position()
            current_yaw = self.get_end_effector_yaw()  # Yaw still uses end-effector
            current_ee_pos_loop = self.get_end_effector_position()  # Only for debug comparison
            
            if current_assembly_pos_loop is None or current_yaw is None:
                rospy.logwarn("Formation pose information lost, continuing to wait")
                rospy.sleep(0.04)
                continue
                
            # Use Assembly CoG position as trajectory update input
            current_pos = current_assembly_pos_loop
            
            # Check for successful contact establishment
            if valve_rotation >= target_valve_rotation:
                rospy.loginfo(f"Valve engagement detected: {math.degrees(valve_rotation):.1f}° "
                             f"(threshold: {math.degrees(target_valve_rotation):.1f}°)")
                rospy.loginfo(f"Contact establishment time: {contact_duration:.1f}s")
                
                # Safety check: warn if contact too fast
                if contact_duration < 1.0:
                    rospy.logwarn(f"Contact established too quickly ({contact_duration:.1f}s), may need parameter adjustment")
                
                # Store trajectory generator for rotation phase use
                self.trajectory_generator = trajectory_generator  # Save instance
                
                rospy.loginfo("Trajectory generator state transferred to rotation phase")
                
                # Store contact success information
                contact_final_torque = [0.0, 0.0, -0.08]  # Fixed baseline torque
                rospy.loginfo(f"Final contact torque (aligned): {contact_final_torque}")
                
                # Set rotation baseline
                rospy.loginfo(f"Rotation baseline set: initial={math.degrees(initial_valve_yaw):.1f}°, "
                             f"current={math.degrees(current_valve_yaw):.1f}°, achieved={math.degrees(valve_rotation):.1f}°")
                
                return True
            
            # Timeout check
            if contact_duration > max_contact_time:
                break
                
            # Enhanced trajectory monitoring (like Single UAV)
            if contact_duration > 1.0:  # Start logging after 1 second
                rospy.loginfo_throttle(2.0, 
                    f"Formation contact: radius={state_info['current_radius']*1000:.1f}mm, "
                    f"angular_vel={math.degrees(state_info['angular_velocity']):.1f}°/s, "
                    f"valve_rotation={math.degrees(valve_rotation):.2f}°, "
                    f"time={contact_duration:.1f}s")
            
            # Update trajectory generator state (Critical: like Single UAV)
            state_info = trajectory_generator.update_state(
                current_pos, current_yaw, valve_angular_velocity
            )
            
            # Execute trajectory  
            target_state = trajectory_generator.generate_target_state()
            if target_state:
                # Use same SE(3) control method as single UAV
                self.beetle.executeTrajectoryWithWrench(
                    pos=target_state['position'],
                    rot=target_state['yaw'],
                    linear_vel=target_state['linear_velocity'],
                    angular_vel=target_state['angular_velocity'],
                    force=target_state['force'],
                    torque=target_state['torque']
                )
            
            rospy.sleep(0.04)  # 25Hz control rate
        
        # Contact establishment failed
        rospy.logerr(f"Contact establishment failed: max valve movement {math.degrees(max_detected_rotation):.2f}°")
        return False
    
    def _get_valve_yaw_safe(self, fallback):
        """Safe valve yaw getter with fallback"""
        valve_yaw = self.beetle.getValveYaw()
        if valve_yaw is None:
            rospy.logwarn("getValveYaw() returned None, using fallback")
            return fallback
        return valve_yaw
    
    def _execute_rotation_phase(self, userdata):
        """Rotation execution phase - reuse trajectory generator instance from contact phase"""
        
        # Use trajectory generator instance stored from contact phase
        if not hasattr(self, 'trajectory_generator') or self.trajectory_generator is None:
            rospy.logerr("No trajectory generator available from contact phase!")
            return 'failed'
            
        rospy.loginfo("Reusing trajectory generator from contact phase")
        
        # CRITICAL FIX: Correct valve_center Z coordinate in rotation phase, use saved insertion depth to avoid Z jumps
        current_pos = self.get_end_effector_position()
        if current_pos is not None:
            # Use saved insertion depth to avoid Z coordinate jumps in Contact→Rotation transition
            insertion_depth_z = FormationSingleUAVStateBase._shared_target_z or current_pos[2]
            original_valve_center = self.valve_center
            corrected_valve_center = [original_valve_center[0], original_valve_center[1], insertion_depth_z]
            
            # Update trajectory generator valve_center
            if hasattr(self.trajectory_generator, 'valve_center'):
                self.trajectory_generator.valve_center = np.array(corrected_valve_center)
                self.valve_center = corrected_valve_center
                z_source = "saved_target_z" if FormationSingleUAVStateBase._shared_target_z else "current_pos"
                rospy.loginfo(f"Rotation phase Z correction: ({original_valve_center[2]:.3f}) → ({insertion_depth_z:.3f}) [source: {z_source}]")
        
        # Get current state information
        current_valve_yaw_now = self._get_valve_yaw_safe(self.initial_valve_yaw)
        
        rospy.loginfo(f"Starting rotation from baseline, contact achieved: {math.degrees(abs(current_valve_yaw_now - self.initial_valve_yaw)):.1f}°")
        rospy.loginfo(f"Rotation start position: {self.rotation_start_position}")
        rospy.loginfo(f"Valve center: {self.valve_center}")
        rospy.loginfo(f"Initial rotation radius: {self.contact_radius*1000:.1f}mm")
        rospy.loginfo(f"Rotation directions consistent: {self.rotation_direction}")
        
        # SINGLE UAV COMPATIBILITY: Reuse Contact phase trajectory generator for continuity
        effective_angular_velocity = abs(self.nominal_angular_velocity)
        rospy.loginfo(f">>> Phase 2: Rotation {math.degrees(self.target_rotation):.1f}° @ {math.degrees(effective_angular_velocity):.1f}°/s")
        
            # Rotation control loop
        rotation_start_time = rospy.Time.now().to_sec()
        initial_valve_yaw = self.initial_valve_yaw
        last_valve_yaw = current_valve_yaw_now
        last_valve_check_time = rotation_start_time
        max_rotation_detected = 0.0
        
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - rotation_start_time
            
            # Get current valve state
            current_valve_yaw = self._get_valve_yaw_safe(initial_valve_yaw)
            valve_rotation = abs(current_valve_yaw - initial_valve_yaw)
            max_rotation_detected = max(max_rotation_detected, valve_rotation)
            
            # Calculate valve angular velocity (fully adopt single UAV logic)
            valve_angular_velocity = 0.0
            if rospy.Time.now().to_sec() > last_valve_check_time + 0.2:  # Update every 200ms
                dt = rospy.Time.now().to_sec() - last_valve_check_time
                valve_angular_velocity = abs(current_valve_yaw - last_valve_yaw) / dt
                last_valve_yaw = current_valve_yaw
                last_valve_check_time = rospy.Time.now().to_sec()

            # DEBUG: Output detailed valve status every 5 seconds (after valve_angular_velocity calculation)
            if int(elapsed) % 5 == 0 and (elapsed - int(elapsed)) < 0.1:
                rospy.logwarn("VALVE & CONTACT STATUS DEBUG:")
                rospy.logwarn(f"  current_valve_yaw: {math.degrees(current_valve_yaw):.2f}°")
                rospy.logwarn(f"  initial_valve_yaw: {math.degrees(initial_valve_yaw):.2f}°")
                rospy.logwarn(f"  raw_difference: {math.degrees(current_valve_yaw - initial_valve_yaw):.2f}°")
                rospy.logwarn(f"  valve_rotation (abs): {math.degrees(valve_rotation):.2f}°")
                rospy.logwarn(f"  target_rotation: {math.degrees(self.target_rotation):.2f}°")
                rospy.logwarn(f"  valve_angular_velocity: {math.degrees(valve_angular_velocity):.3f}°/s")
                
                # Contact state detection (use dynamic Z coordinate to avoid pitch issues)
                current_pos_raw = self.get_end_effector_position()[:3]
                current_pos = (current_pos_raw[0], current_pos_raw[1], current_pos_raw[2])
                distance_to_valve = math.sqrt((current_pos[0] - self.valve_center[0])**2 + 
                                            (current_pos[1] - self.valve_center[1])**2)
                if distance_to_valve > self.contact_radius * 1.2:
                    rospy.logwarn(f"Contact distance: {distance_to_valve*1000:.0f}mm (expected {self.contact_radius*1000:.0f}mm)")
                elif abs(valve_angular_velocity) < 0.005:
                    rospy.logwarn(f"Valve rotation: {math.degrees(valve_rotation):.1f}° (stagnant)")
            
            # Check rotation complete
            if valve_rotation >= self.target_rotation:
                rospy.loginfo(f"Valve rotation completed: {math.degrees(valve_rotation):.1f}° "
                             f"(target: {math.degrees(self.target_rotation):.1f}°) in {elapsed:.1f}s")
                
                # Restore trajectory_state save for Disengage phase use
                userdata.trajectory_state = {
                    'current_radius': self.contact_radius,
                    'current_angle': getattr(self, 'contact_angle', 0),
                    'radius_locked': True,
                    'valve_center': self.valve_center
                }
                userdata.contact_final_torque = [0.0, 0.0, 0.1]
                
                return 'succeeded'
            
            # 超时检查
            if elapsed > self.max_rotation_time:
                rospy.logwarn(f"Rotation timeout after {elapsed:.1f}s, achieved: {math.degrees(valve_rotation):.1f}°")
                break
            
            # Execute trajectory commands - update state first then generate target  
            # Solution A fix: Use real position for state update, consistent with single UAV version
            current_pos = self.get_end_effector_position()[:3]
            current_yaw = self.get_end_effector_yaw() or 0.0
            
            self.trajectory_generator.update_state(current_pos, current_yaw, valve_angular_velocity)
            target_state = self.trajectory_generator.generate_target_state()
            
            # Solution A monitoring: Rotation phase trajectory output check
            if target_state and elapsed > 1.0:  # Avoid initial noise
                target_pos = target_state.get('position', [0, 0, 0])
                pos_diff = [(target_pos[i] - current_pos[i]) * 1000 for i in range(3)]
                rospy.loginfo_throttle(8.0,
                    f"Rotation trajectory: current=({current_pos[0]:.3f},{current_pos[1]:.3f},{current_pos[2]:.3f}), "
                    f"target=({target_pos[0]:.3f},{target_pos[1]:.3f},{target_pos[2]:.3f}), "
                    f"diff=({pos_diff[0]:.1f},{pos_diff[1]:.1f},{pos_diff[2]:.1f})mm")
            
            # Important addition: Valve response adaptive control (mimic single version)
            if target_state and valve_angular_velocity > 0.05:  # Valve is rotating
                # Dynamic torque adjustment: reduce torque if valve rotates fast, increase if slow
                resistance_factor = max(0.5, min(2.0, 0.3 / max(0.05, valve_angular_velocity)))
                rospy.loginfo_throttle(3.0, f"Torque adaptation: valve_ω={valve_angular_velocity:.3f}rad/s, factor={resistance_factor:.2f}")
                
                # Adjust trajectory generator output torque
                if 'torque' in target_state and target_state['torque'] is not None:
                    original_torque = target_state['torque']
                    adapted_torque = [t * resistance_factor for t in original_torque]
                    target_state['torque'] = adapted_torque
                    rospy.logdebug(f"Torque adjustment: {original_torque} → {adapted_torque}")
            
            if target_state:
                # Directly use same SE(3) control method as single UAV, eliminate wrapper layer
                self.beetle.executeTrajectoryWithWrench(
                    pos=target_state['position'],
                    rot=target_state['yaw'],
                    linear_vel=target_state['linear_velocity'],
                    angular_vel=target_state['angular_velocity'],
                    force=target_state['force'],
                    torque=target_state['torque']
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
            initial_valve_yaw = rotation_baseline_yaw  # Use valve yaw at contact success
            rospy.loginfo(f"Using rotation baseline: {math.degrees(initial_valve_yaw):.1f}°")
        else:
            initial_valve_yaw = getattr(userdata, 'initial_valve_yaw', current_valve_yaw_now)
            rospy.logwarn("No rotation baseline found, using fallback")
            
        yaw_change_threshold = getattr(userdata, 'valve_yaw_change_threshold', math.radians(10.0))
        
        # Check if contact establishment was successful
        detected_valve_rotation = getattr(userdata, 'detected_valve_rotation', 0.0)
        contact_success_time = getattr(userdata, 'contact_success_time', 0.0)
        contact_achieved_rotation = getattr(userdata, 'contact_achieved_rotation', 0.0)
        
        if detected_valve_rotation > 0:
            rospy.loginfo(f"Contact phase confirmed valve engagement: {math.degrees(detected_valve_rotation):.1f}° "
                         f"in {contact_success_time:.1f}s")
            rospy.loginfo(f"Starting rotation from baseline, contact achieved: {math.degrees(contact_achieved_rotation):.1f}°")
        else:
            rospy.logwarn("No valve engagement detected in contact phase, but proceeding with rotation")

        rotation_start_pos = getattr(userdata, 'rotation_start_position', None)
        if rotation_start_pos is None:
            rotation_start_pos = self.get_end_effector_position()

        if rotation_start_pos is None:
            rospy.logerr("Unable to determine rotation start position")
            return 'failed'

        # Restore single UAV logic: directly use original valve_pos, let trajectory generator's current_z mechanism handle Z coordinate continuity
        rospy.loginfo(f"Rotation phase using single UAV logic: valve_center=valve_pos (Z={valve_pos[2]:.3f}m)")
        rospy.loginfo(f"Trajectory generator will automatically handle Z coordinate continuity via current_z={rotation_start_pos[2]:.3f}m")

        initial_radius = math.sqrt(
            (rotation_start_pos[0] - valve_pos[0])**2 +
            (rotation_start_pos[1] - valve_pos[1])**2
        )

        if initial_radius < 1e-6:
            rospy.logerr("Computed rotation radius too small")
            return 'failed'

        rospy.loginfo(f"Rotation start position: {rotation_start_pos}")
        rospy.loginfo(f"Valve center: {valve_pos}")
        rospy.loginfo(f"Initial rotation radius: {initial_radius*1000:.1f}mm")

        # Step 3: Ensure rotation direction consistency
        # Use rotation direction from contact phase if available
        insertion_info = getattr(userdata, 'insertion_info', {})
        contact_rotation_direction = insertion_info.get('rotation_direction', self.rotation_direction)
        if contact_rotation_direction != self.rotation_direction:
            rospy.logwarn(f"Direction mismatch: contact={contact_rotation_direction}, rotation={self.rotation_direction}")
            rospy.loginfo(f"Using consistent rotation direction: {contact_rotation_direction}")
            effective_rotation_direction = contact_rotation_direction
        else:
            effective_rotation_direction = self.rotation_direction
            rospy.loginfo(f"Rotation directions consistent: {effective_rotation_direction}")

        # CRITICAL FIX: Consistent with single UAV version, always use positive angular velocity
        angular_velocity = abs(self.nominal_angular_velocity)  # Force positive, consistent with single UAV
        
        # ENHANCED DEBUG: Output all key parameters
        rospy.logwarn("=" * 60)
        rospy.logwarn("ROTATION DEBUG - All key parameters:")
        rospy.logwarn(f"  Trajectory Generator Parameters:")
        rospy.logwarn(f"    nominal_angular_velocity: {self.nominal_angular_velocity:.4f} rad/s ({math.degrees(self.nominal_angular_velocity):.2f}°/s)")
        rospy.logwarn(f"    effective_rotation_direction: {effective_rotation_direction:+d}")
        rospy.logwarn(f"    FINAL angular_velocity: {angular_velocity:.4f} rad/s ({math.degrees(angular_velocity):.2f}°/s)")
        rospy.logwarn(f"  Valve Position & Orientation:")
        rospy.logwarn(f"    initial_valve_yaw (baseline): {math.degrees(initial_valve_yaw):.2f}°")
        rospy.logwarn(f"    current_valve_yaw_now: {math.degrees(current_valve_yaw_now):.2f}°")
        rospy.logwarn(f"    valve_center: {valve_pos}")
        rospy.logwarn(f"    rotation_radius: {initial_radius*1000:.1f}mm")
        rospy.logwarn(f"  Expected Behavior:")
        if angular_velocity > 0:
            rospy.logwarn(f"    UAV will move: COUNTER-CLOCKWISE (positive angle increase)")
        else:
            rospy.logwarn(f"    UAV will move: CLOCKWISE (negative angle decrease)")
        rospy.logwarn("=" * 60)
        
        # CRITICAL FIX: Directly reuse Contact phase trajectory generator state, avoid reinitialization
        contact_state = getattr(userdata, 'trajectory_state', {}) or {}
        
        if contact_state and all(k in contact_state for k in ['valve_center', 'current_angle', 'current_radius']):
            # Solution: Inherit complete Contact phase state, only change angular_velocity
            rospy.logwarn("INHERITING Contact trajectory state - seamless transition!")
            trajectory_gen = OnlineCircularTrajectoryGenerator(
                valve_center=contact_state['valve_center'],
                initial_radius=contact_state.get('current_radius', initial_radius),
                target_angular_velocity=angular_velocity,  # Only change rotation speed/direction
                control_rate=self.control_rate,
                debug=True
            )
            
            # Precisely restore all Contact phase state parameters
            trajectory_gen.current_angle = contact_state['current_angle']
            trajectory_gen.current_radius = contact_state['current_radius'] 
            trajectory_gen.radius_locked = contact_state.get('radius_locked', False)
            trajectory_gen.lock_radius_value = contact_state.get('lock_radius_value', contact_state['current_radius'])
            trajectory_gen.valve_center = np.array(contact_state['valve_center'])
            trajectory_gen.current_z = rotation_start_pos[2]
            
            rospy.logwarn(f"  Inherited state: angle={math.degrees(trajectory_gen.current_angle):.1f}°, "
                         f"radius={trajectory_gen.current_radius*1000:.1f}mm, locked={trajectory_gen.radius_locked}")
        else:
            # Fallback: Reinitialize (preserve original logic)
            rospy.logwarn("No complete contact state found, reinitializing trajectory generator")
            trajectory_gen = OnlineCircularTrajectoryGenerator(
                valve_center=valve_pos,  # Use original valve_pos, consistent with single UAV
                initial_radius=initial_radius,
                target_angular_velocity=angular_velocity,
                control_rate=self.control_rate,
                debug=True
            )
            trajectory_gen.initialize_from_current_position(rotation_start_pos)

        baseline_torque = getattr(userdata, 'contact_final_torque', [0.0, 0.0, 0.15])  # Increase baseline torque

        rate = rospy.Rate(self.control_rate)
        start_time = rospy.Time.now().to_sec()
        last_valve_check_time = start_time
        last_valve_yaw = initial_valve_yaw
        valve_motion_stalled_time = 0.0  # Renamed to stalled_time, consistent with single UAV
        max_valve_rotation_achieved = 0.0  # Add tracking variable from single UAV version
        valve_stuck_threshold = 6.0  # 6-second detection threshold: balance Formation system stability with timely detection
        
        # Valve stuck detection mechanism explanation:
        # - Single UAV version uses 3 seconds (single robot system reacts quickly)
        # - Formation version originally 8 seconds (too conservative, delayed detection)
        # - Adjusted to 6 seconds technical rationale:
        #   1. Formation dual-UAV coordination needs longer adjustment time than single UAV
        #   2. Multi-UAV system force transmission chain is more complex, torque transfer has delay
        #   3. 6 seconds provides sufficient contact establishment time while avoiding long waits
        #   4. Detection logic: continuous monitoring valve_angular_velocity < 0.008 rad/s (0.5°/s)
        rospy.loginfo(f"Valve stuck detection: {valve_stuck_threshold:.1f}s threshold (Formation dual-UAV coordination optimized)")

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
            
            # Adopt single UAV calculation method: directly calculate valve rotation angle (no normalize, consistent with single UAV)
            valve_rotation = abs(current_valve_yaw - initial_valve_yaw)
            max_valve_rotation_achieved = max(max_valve_rotation_achieved, valve_rotation)
            rotation_progress = valve_rotation  # Maintain compatibility

            # Calculate valve angular velocity (fully adopt single UAV logic)
            valve_angular_velocity = 0.0
            if current_time > last_valve_check_time + 0.2:  # Update every 200ms
                dt = current_time - last_valve_check_time
                valve_angular_velocity = abs(current_valve_yaw - last_valve_yaw) / dt
                last_valve_yaw = current_valve_yaw
                last_valve_check_time = current_time

            # DEBUG: Output detailed valve status every 5 seconds (after valve_angular_velocity calculation)
            if int(elapsed) % 5 == 0 and (elapsed - int(elapsed)) < 0.1:
                rospy.logwarn("VALVE & CONTACT STATUS DEBUG:")
                rospy.logwarn(f"  current_valve_yaw: {math.degrees(current_valve_yaw):.2f}°")
                rospy.logwarn(f"  initial_valve_yaw: {math.degrees(initial_valve_yaw):.2f}°")
                rospy.logwarn(f"  raw_difference: {math.degrees(current_valve_yaw - initial_valve_yaw):.2f}°")
                rospy.logwarn(f"  valve_rotation (abs): {math.degrees(valve_rotation):.2f}°")
                rospy.logwarn(f"  target_rotation: {math.degrees(self.target_rotation):.2f}°")
                rospy.logwarn(f"  valve_angular_velocity: {math.degrees(valve_angular_velocity):.3f}°/s")
                
                # Contact state detection
                distance_to_valve = math.sqrt((current_pos[0] - valve_pos[0])**2 + 
                                            (current_pos[1] - valve_pos[1])**2)
                rospy.logwarn(f"  CONTACT STATUS:")
                rospy.logwarn(f"    distance_to_valve: {distance_to_valve*1000:.1f}mm")
                rospy.logwarn(f"    expected_radius: {initial_radius*1000:.1f}mm")
                if distance_to_valve > initial_radius * 1.2:
                    rospy.logwarn(f"    CONTACT LOST! UAV too far from valve")
                elif abs(valve_angular_velocity) < 0.005:
                    rospy.logwarn(f"    VALVE NOT MOVING! Possibly lost contact")
                
                # Detect valve motion stall (improved logic)
                # Only start detecting stall after running for 5 seconds to ensure stable contact established
                if elapsed > 5.0:
                    if valve_angular_velocity < 0.008:  # < 0.5°/s considered stalled (more lenient)
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

            # DEBUG: Output trajectory generator status every 5 seconds
            if int(elapsed) % 5 == 0 and (elapsed - int(elapsed)) < 0.1:
                rospy.logwarn("TRAJECTORY DEBUG:")
                rospy.logwarn(f"  current_pos: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
                rospy.logwarn(f"  target_pos: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                rospy.logwarn(f"  position_diff: [{target_pos[0]-current_pos[0]:.4f}, {target_pos[1]-current_pos[1]:.4f}, {target_pos[2]-current_pos[2]:.4f}]")
                rospy.logwarn(f"  current_yaw: {math.degrees(current_yaw):.2f}° -> target_yaw: {math.degrees(target_yaw):.2f}°")
                rospy.logwarn(f"  linear_vel: [{linear_vel_vec[0]:.4f}, {linear_vel_vec[1]:.4f}, {linear_vel_vec[2]:.4f}] m/s")
                rospy.logwarn(f"  angular_vel: {target_state.get('angular_velocity', 0.0):.4f} rad/s ({math.degrees(target_state.get('angular_velocity', 0.0)):.2f}°/s)")
                rospy.logwarn(f"  speed_total: {speed_total:.4f} m/s")
                
                # Check trajectory generator internal state
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

            # Check failure condition: valve motion stalled (fully adopt single UAV logic)
            if valve_motion_stalled_time > valve_stuck_threshold:
                rospy.logerr("Valve motion stall detected!")
                rospy.logerr(f"  Stall time: {valve_motion_stalled_time:.1f}s > threshold {valve_stuck_threshold:.1f}s")
                rospy.logerr(f"  Current valve angular velocity: {math.degrees(valve_angular_velocity):.2f}°/s")
                rospy.logerr(f"  Achieved rotation: {math.degrees(max_valve_rotation_achieved):.1f}°")
                rospy.logerr(f"  Target rotation: {math.degrees(self.target_rotation):.1f}°")
                rospy.logerr(f"  Completion percentage: {(max_valve_rotation_achieved/self.target_rotation)*100:.1f}%")
                
                if max_valve_rotation_achieved > self.target_rotation * 0.7:  # 70% considered partial success
                    rospy.loginfo(f"Partial success: rotated {math.degrees(max_valve_rotation_achieved):.1f}°, continuing attempt")
                    valve_motion_stalled_time = 0.0  # Reset timer
                else:
                    rospy.logerr("Valve may be stuck or resistance too high, rotation failed!")
                    rospy.logerr(f"   Failure reason: only achieved {math.degrees(max_valve_rotation_achieved):.1f}° < 70% target ({math.degrees(self.target_rotation * 0.7):.1f}°)")
                    return 'failed'

            if int(elapsed) % 5 == 0:
                progress_pct = trajectory_gen.get_progress(self.target_rotation) * 100.0
                rospy.loginfo_throttle(1.0,
                    f"Rotation progress: {progress_pct:.1f}% | Valve Δ={math.degrees(valve_rotation):.1f}° | "
                    f"Max={math.degrees(max_valve_rotation_achieved):.1f}° | "
                    f"Radius={state_info['current_radius']*1000:.1f}mm | ω={math.degrees(state_info['angular_velocity']):.1f}°/s")

            rate.sleep()

        # Final check (adopt single UAV logic)
        final_valve_yaw = self.beetle.getValveYaw()
        if final_valve_yaw is None:
            final_valve_yaw = initial_valve_yaw  # fallback to initial if read fails
        final_valve_rotation = abs(final_valve_yaw - initial_valve_yaw)
        
        if final_valve_rotation >= self.target_rotation * 0.8:  # 80% considered acceptable
            rospy.loginfo(f"Valve rotation successful: {math.degrees(final_valve_rotation):.1f}°")
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
    """Formation version of DisengageFromValveState - simplified 3-phase approach"""
    
    def __init__(self):
        FormationSingleUAVStateBase.__init__(self,
            outcomes=['succeeded', 'failed'],
            input_keys=['valve_position', 'start_position', 'trajectory_state'],
            output_keys=['disengagement_position'])
    
    def execute(self, userdata):
        rospy.loginfo("=== Formation Disengage From Valve State ===")
        
        valve_pos = getattr(userdata, 'valve_position', None)
        if valve_pos is None:
            rospy.logerr("Valve position missing from userdata")
            return 'failed'
        
        start_assembly_pos = getattr(userdata, 'start_position', None)
        trajectory_state = getattr(userdata, 'trajectory_state', {}) or {}
        
        # Get current position
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        if current_pos is None or current_yaw is None:
            rospy.logerr("Unable to get current pose for disengage")
            return 'failed'
        
        rospy.loginfo(f"Current position: {self._format_vec(current_pos)}, yaw={math.degrees(current_yaw):.1f}°")
        
        # === Phase 1: Reverse circular motion to disengage from valve ===
        rospy.loginfo("[Phase 1] Reverse circular motion (self-rotation + revolution)")
        
        # Get rotation parameters from trajectory state
        generator_center = trajectory_state.get('valve_center', valve_pos)
        generator_radius = trajectory_state.get('current_radius')
        if generator_radius is None:
            generator_radius = math.sqrt((current_pos[0] - generator_center[0])**2 +
                                        (current_pos[1] - generator_center[1])**2)
        
        # Reverse rotation: -15° to disengage
        reverse_angle = math.radians(15.0)
        reverse_velocity = -0.05  # Reverse direction, half speed for safety (2.9°/s)
        
        rospy.loginfo(f"Reverse parameters: angle={math.degrees(reverse_angle):.1f}°, "
                     f"velocity={math.degrees(reverse_velocity):.1f}°/s, radius={generator_radius*1000:.1f}mm")
        
        try:
            reverse_generator = OnlineCircularTrajectoryGenerator(
                valve_center=generator_center,
                initial_radius=generator_radius,
                target_angular_velocity=reverse_velocity,
                control_rate=25.0
            )
            reverse_generator.initialize_from_current_position(current_pos)
            
            # Execute reverse motion
            reverse_start = rospy.Time.now().to_sec()
            reverse_duration = abs(reverse_angle / reverse_velocity)  # Time needed
            control_rate = rospy.Rate(25)
            
            while (rospy.Time.now().to_sec() - reverse_start) < reverse_duration and not rospy.is_shutdown():
                updated_pos = self.get_end_effector_position()
                updated_yaw = self.get_end_effector_yaw()
                
                if updated_pos is None or updated_yaw is None:
                    control_rate.sleep()
                    continue
                
                # Update and generate trajectory
                reverse_generator.update_state(updated_pos, updated_yaw, 0.0)
                target_state = reverse_generator.generate_target_state()
                
                # Send command
                self.send_assembly_command_from_end_effector(
                    target_state['position'].tolist(),
                    target_state['yaw'],
                    linear_vel=target_state['linear_velocity'].tolist(),
                    angular_vel=target_state.get('angular_velocity')
                )
                
                control_rate.sleep()
            
            rospy.loginfo("Reverse motion completed")
            
        except Exception as e:
            rospy.logwarn(f"Reverse motion failed: {e}, continuing to next phase")
        
        # Stabilize after reverse
        rospy.sleep(1.0)
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        
        # === Phase 2: Ascend to start height ===
        rospy.loginfo("[Phase 2] Ascending to start height")
        
        # Determine target Z height
        if start_assembly_pos is not None:
            target_z = start_assembly_pos[2] + 0.1  # 10cm above start
        else:
            target_z = current_pos[2] + 0.15  # 15cm up as fallback
        
        ascent_target = (current_pos[0], current_pos[1], target_z)
        z_distance = abs(target_z - current_pos[2])
        rospy.loginfo(f"Ascending to Z={target_z:.3f}m (distance: {z_distance*1000:.1f}mm)")
        rospy.loginfo(f"[Phase 2] Ascent with XY locked at ({current_pos[0]:.3f}, {current_pos[1]:.3f})")
        
        # Use direct convergence with very slow speed and locked XY position
        # Simpler and more reliable than polynomial trajectory for pure vertical motion
        rospy.loginfo("[Phase 2] Using slow vertical convergence (XY locked)")
        success = self.active_position_convergence(
            ascent_target,
            target_yaw=current_yaw,  # Maintain current yaw during ascent
            pos_thresh=0.05,         # 50mm threshold
            yaw_thresh=0.1,          # 5.7° yaw tolerance (not critical during ascent)
            timeout=30.0,
            max_linear_vel=0.03,     # Slow ascent: 30mm/s (was 50mm/s)
            max_angular_vel=0.02     # Minimal rotation during ascent
        )
        
        if not success:
            rospy.logwarn("Ascent convergence incomplete, but continuing")
        
        rospy.sleep(1.0)
        current_pos = self.get_end_effector_position()
        current_yaw = self.get_end_effector_yaw()
        
        # === Phase 3: Return to start XY position ===
        rospy.loginfo("[Phase 3] Returning to start XY position")
        
        if start_assembly_pos is not None:
            # Phase 3A: First adjust yaw to 0° (maintain current XY position)
            rospy.loginfo("[Phase 3A] Adjusting yaw to 0° before XY movement")
            yaw_adjust_target = (current_pos[0], current_pos[1], current_pos[2])
            
            # Strict yaw convergence: must reach < 1.5° before proceeding
            yaw_success = self.active_position_convergence(
                yaw_adjust_target,
                target_yaw=0.0,  # Target yaw = 0°
                pos_thresh=0.10,  # Very loose position tolerance (100mm) - only care about yaw
                yaw_thresh=0.026,  # Strict yaw threshold: 1.5° (was 2.9°)
                timeout=30.0,      # Longer timeout for strict convergence
                max_linear_vel=0.02,  # Very slow XY to minimize drift (was 0.03)
                max_angular_vel=0.035,  # 2°/s for smooth yaw adjustment
                max_yaw_step=0.05  # Limit yaw step to 2.9° per control cycle (prevent overshoot)
            )
            
            if not yaw_success:
                rospy.logwarn("[Phase 3A] Yaw adjustment incomplete, but continuing")
            else:
                rospy.loginfo("[Phase 3A] Yaw adjustment complete")
            
            rospy.sleep(1.5)  # Longer stabilization after yaw adjustment
            
            # Update current position after yaw adjustment
            current_pos = self.get_end_effector_position()
            current_yaw = self.get_end_effector_yaw()
            
            # Phase 3B: Now perform XY movement with yaw locked at 0°
            rospy.loginfo("[Phase 3B] XY movement to start position (yaw locked at 0°)")
            return_target = (start_assembly_pos[0], start_assembly_pos[1], current_pos[2])
            rospy.loginfo(f"Target: ({return_target[0]:.3f}, {return_target[1]:.3f}, {return_target[2]:.3f})")
            
            # Calculate XY distance for trajectory selection
            xy_distance = math.sqrt(
                (return_target[0] - current_pos[0])**2 + 
                (return_target[1] - current_pos[1])**2
            )
            rospy.loginfo(f"[Phase 3B] XY return distance: {xy_distance*1000:.1f}mm")
            
            # Use polynomial trajectory for smooth return (same as Phase 2 approach to valve)
            if xy_distance > 0.1:  # Use trajectory for movements > 100mm
                rospy.loginfo("[Phase 3B] Using polynomial trajectory for smooth XY return")
                
                # Generate smooth polynomial trajectory (more points for very long distance)
                num_points = max(12, min(20, int(xy_distance / 0.2)))  # 12-20 points based on distance
                trajectory_points = self.generate_polynomial_trajectory(
                    start_pos=current_pos,
                    target_pos=return_target,
                    target_yaw=0.0,  # Keep yaw locked at 0° during return
                    num_points=num_points
                )
                
                if trajectory_points:
                    rospy.loginfo(f"[Phase 3B] Executing {len(trajectory_points)} trajectory points")
                    trajectory_success = self.execute_polynomial_trajectory(trajectory_points)
                else:
                    rospy.logwarn("[Phase 3B] Trajectory generation failed, using direct convergence")
                    trajectory_success = False
                
                # Final precision convergence
                if trajectory_success:
                    rospy.loginfo("[Phase 3B] Final precision convergence")
                    success = self.active_position_convergence(
                        return_target,
                        target_yaw=0.0,
                        pos_thresh=0.05,
                        yaw_thresh=0.1,
                        timeout=15.0,
                        max_linear_vel=0.05,
                        max_angular_vel=0.035
                    )
                else:
                    # Fallback to direct convergence if trajectory failed
                    rospy.loginfo("[Phase 3B] Using direct convergence (fallback)")
                    success = self.active_position_convergence(
                        return_target,
                        target_yaw=0.0,
                        pos_thresh=0.05,
                        yaw_thresh=0.1,
                        timeout=90.0,
                        max_linear_vel=0.05,
                        max_angular_vel=0.035
                    )
            else:
                # Short distance: direct convergence
                rospy.loginfo("[Phase 3B] Short distance, using direct convergence")
                success = self.active_position_convergence(
                    return_target,
                    target_yaw=0.0,
                    pos_thresh=0.05,
                    yaw_thresh=0.1,
                    timeout=20.0,
                    max_linear_vel=0.05,
                    max_angular_vel=0.035
                )
            
            if not success:
                rospy.logwarn("[Phase 3B] Return to start incomplete")
        else:
            rospy.logwarn("Start position unavailable, skipping return")
        
        final_pos = self.get_end_effector_position()
        userdata.disengagement_position = final_pos
        
        rospy.loginfo(f"=== Disengagement complete: {self._format_vec(final_pos)} ===")
        return 'succeeded'
    
    @staticmethod
    def _format_vec(vec):
        """Format 3D vector for logging"""
        if vec is None:
            return "None"
        return f"({vec[0]:.3f}, {vec[1]:.3f}, {vec[2]:.3f})"
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))


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
