#!/usr/bin/env python
"""
Single UAV Valve Rotation using SMACH State Machine
Cleaned version with redundant code removed.
"""

import sys
import os

# Add beetle package paths
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

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
from insertion_optimizer import InsertionOptimizer


class SingleUAVStateBase(smach.State):
    def __init__(self, outcomes, input_keys=None, output_keys=None, module_id=1):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys or [], output_keys=output_keys or [])
        self.module_id = module_id
        
        # Publishers and subscribers
        self.pub = rospy.Publisher(f"/beetle{module_id}/uav/nav", FlightNav, queue_size=1)
        self.motion_controller = UnifiedMotionController(self)
        
        # Position and orientation data
        self.uav_pos = None
        self.current_yaw = 0.0
        self.valve_pos = None
        self.valve_yaw = 0.0
        self.external_wrench = None
        
        # Thread events for data synchronization
        self.uav_received = threading.Event()
        self.valve_received = threading.Event()
        
        # Get simulation and real_machine parameters
        self.simulation = rospy.get_param("~simulation", False)
        self.real_machine = rospy.get_param("~real_machine", True)
        
        # Setup subscribers
        rospy.loginfo(f"Subscribing to UAV pose: /beetle{module_id}/mocap/pose")
        self.uav_sub = rospy.Subscriber(f"/beetle{module_id}/mocap/pose", PoseStamped, self.uav_callback, queue_size=1)
        self.wrench_sub = rospy.Subscriber(f"/beetle{module_id}/estimated_external_wrench", WrenchStamped, self.wrench_callback, queue_size=1)
        
        # Setup valve pose subscriber based on mode
        if self.simulation:
            # In simulation mode, use /valve/odom topic
            rospy.loginfo("Subscribing to valve pose (simulation): /valve/odom")
            self.valve_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            # In real mode, use mocap topic
            rospy.loginfo("Subscribing to valve pose (real): /valve/mocap/pose")
            self.valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
        
        # Basic parameters
        self.position_threshold = 0.03
        self.yaw_threshold = 0.05
        self.timeout = 60.0  # Increased timeout for valve rotation tasks
        self.z_offset = 0.0743823
        self.descent_speed = 0.025
        
        rospy.loginfo(f"Initialized UAV{module_id} state with basic parameters")
    
    def uav_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.uav_pos = (position.x, position.y, position.z)
        
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.current_yaw = yaw
        
        self.uav_received.set()
    
    def valve_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.valve_pos = (position.x, position.y, position.z)
        
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.valve_yaw = yaw
        
        self.valve_received.set()
    
    def valve_sim_callback(self, msg):
        # Handle Odometry message for simulation mode
        self.valve_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        q = msg.pose.pose.orientation
        _, _, self.valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.valve_received.set()
    
    def wrench_callback(self, msg):
        self.external_wrench = msg.wrench
    
    def wait_for_positions(self):
        rospy.loginfo("Waiting for UAV and valve positions...")
        if not self.uav_received.wait(timeout=5.0):
            rospy.logerr("UAV position not received")
            return False
        if not self.valve_received.wait(timeout=5.0):
            rospy.logerr("Valve position not received")
            return False
        return True
    
    def get_current_position(self):
        return self.uav_pos
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class InitializeStartPositionState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'],
                        output_keys=['start_position'], 
                        module_id=module_id)
    
    def execute(self, userdata):
        rospy.loginfo("Initializing start position...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        userdata.start_position = self.get_current_position()
        rospy.loginfo(f"Start position initialized: {userdata.start_position}")
        return 'succeeded'


class MoveToValveState(SingleUAVStateBase):
    def __init__(self, module_id=1, approach_height=0.5):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        self.approach_height = approach_height
    
    def execute(self, userdata):
        rospy.loginfo("Moving to valve position (above valve center)...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None or self.valve_pos is None:
            rospy.logerr("Failed to get current UAV position or valve position")
            return 'failed'
        
        # FIXED: Use current UAV Z position to avoid Z-axis oscillation
        # Never use valve_pos[2] + height calculations as they cause Z jumps
        current_z = start_pos[2]
        target_pos = (self.valve_pos[0], self.valve_pos[1], current_z)
        
        rospy.loginfo(f"Moving to valve XY position while maintaining current Z height: {current_z:.3f}m")
        rospy.loginfo("Z-axis oscillation fix: Using current UAV Z instead of valve_pos[2] + approach_height")
        
        # Calculate distance and set conservative speed for stability
        distance = math.sqrt((target_pos[0]-start_pos[0])**2 + 
                           (target_pos[1]-start_pos[1])**2 + 
                           (target_pos[2]-start_pos[2])**2)
        
        move_speed = 0.08  # Reduced from 0.1 for smoother motion
        timeout = max(120, distance / move_speed * 4.0)  # Increased timeout multiplier
        
        rospy.loginfo(f"Moving {distance:.2f}m at {move_speed}m/s (timeout: {timeout}s)")
        
        success = self.motion_controller.execute_poly_motion_with_feedback(
            start_pos, target_pos, move_speed, timeout=timeout)
        
        return 'succeeded' if success else 'failed'


class DescendAndContactState(SingleUAVStateBase):
    def __init__(self, module_id=1, contact_force_threshold=3.0, descent_speed=0.03,
                 use_staged_insertion=True, pre_insertion_distance=0.035,
                 yaw_adjustment_angle=0.08, circumferential_offset=0.01):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        self.contact_force_threshold = contact_force_threshold
        self.descent_speed = descent_speed
        
        # CRITICAL: Adjust insertion approach to avoid valve handle plane blockage
        # The valve handle plane blocks direct insertion, need angular approach
        # FIXED: Calculated correct z_offset based on valve URDF structure
        # valve_pos[2] = 0.570, so 0.570 + 0.0743823 = 0.644m for proper insertion
        self.z_offset = 0.0743823  # Use the original z_offset for proper insertion depth
        self.use_staged_insertion = use_staged_insertion
        self.pre_insertion_distance = pre_insertion_distance
        self.yaw_adjustment_angle = yaw_adjustment_angle
        self.circumferential_offset = circumferential_offset
        
        # FIXED: Insertion positioning - adjust distance from valve center for closer approach
        self.insertion_radius_adjustment = 0.07  # Increased from 5cm to 7cm for closer approach to valve center
        
        # Angular insertion parameters to avoid handle plane blockage
        self.angular_insertion_enabled = rospy.get_param("~angular_insertion_enabled", False)
        self.insertion_angle_offset = 0.12  # Increased from 0.08 to 0.12 for better yaw adjustment during insertion
        self.handle_clearance_height = 0.08  # 8cm above valve center for handle clearance
        
        rospy.loginfo(f"Angular insertion enabled: {self.angular_insertion_enabled}")
        if not self.angular_insertion_enabled:
            rospy.logwarn("Angular insertion is DISABLED - using normal descent only")
        else:
            rospy.logwarn("Angular insertion is ENABLED - may have control instability issues")
        
        # Initialize insertion optimizer with more balanced settings
        simulation = rospy.get_param("~simulation", True)
        self.optimizer = InsertionOptimizer(
            valve_radius=0.1225 - self.insertion_radius_adjustment,  # Reduced radius for closer approach
            valve_beam_width=0.0185,
            safety_margin=0.003,  # Reduced safety margin for closer approach
            module_id=module_id,
            simulation=simulation
        )
    
    def execute(self, userdata):
        rospy.loginfo("Starting improved 4-stage insertion process...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        start_pos = self.get_current_position()
        if start_pos is None:
            return 'failed'
        
        # Calculate proper insertion target Z (valve center + offset)
        target_insertion_z = self.valve_pos[2] + self.z_offset
        
        rospy.loginfo(f"=== INSERTION Z-AXIS PLANNING ===")
        rospy.loginfo(f"Starting Z position: {start_pos[2]:.6f}m")
        rospy.loginfo(f"Valve center Z: {self.valve_pos[2]:.6f}m")
        rospy.loginfo(f"Z offset: {self.z_offset:.6f}m")
        rospy.loginfo(f"Target insertion Z: {target_insertion_z:.6f}m")
        rospy.loginfo(f"Required descent: {start_pos[2] - target_insertion_z:.6f}m")
        
        # FIXED: Use approach Z for stages 1-2, then actual descent for stage 3
        # Stage 1-2: Position at approach height (above valve)
        self.approach_z = start_pos[2]  # Keep current height for positioning
        self.target_insertion_z = target_insertion_z  # Store target for final descent
        
        if self.use_staged_insertion:
            # Stage 1: Approach insertion point (at current height)
            if not self.approach_insertion_point(start_pos):
                rospy.logerr("Failed to approach insertion point")
                return 'failed'
            
            # Stage 2: Yaw and circumferential adjustment (at current height)
            if not self.adjust_yaw_and_circumferential_direction():
                rospy.logerr("Failed to adjust insertion direction")
                return 'failed'
            
            # Stage 3: ACTUAL DESCENT to contact (with angular insertion if enabled)
            if self.angular_insertion_enabled:
                if not self.execute_angular_insertion():
                    rospy.logerr("Failed to execute angular insertion")
                    return 'failed'
            else:
                if not self.descend_to_contact():
                    rospy.logerr("Failed to descend to contact")
                    return 'failed'
            
            # Stage 4: Final rotation to target position
            if not self.rotate_to_target_position():
                rospy.logerr("Failed to rotate to target position")
                return 'failed'
        else:
            # Simple direct descent
            if not self.descend_to_contact():
                rospy.logerr("Failed to descend to contact")
                return 'failed'
        
        rospy.loginfo("4-stage insertion completed successfully")
        return 'succeeded'
    
    def approach_insertion_point(self, start_pos):
        optimal_strategy = self.optimizer.evaluate_real_time_strategy()
        if optimal_strategy is None:
            rospy.logerr("Failed to determine optimal insertion strategy")
            return False
        
        insertion_params = self.optimizer.get_insertion_parameters(
            dual_strategy=optimal_strategy,
            pre_insertion_distance=self.pre_insertion_distance,
            circumferential_offset=self.circumferential_offset
        )
        
        if insertion_params is None:
            rospy.logerr("Failed to get insertion parameters")
            return False
        
        self.current_insertion_params = insertion_params
        
        # === 真正的双爪同时插入策略 ===
        rospy.loginfo("=== DUAL-FANG SIMULTANEOUS INSERTION APPROACH ===")
        rospy.loginfo("Both fangs will approach their optimal positions simultaneously")
        
        # 获取两个爪子的参数
        fang1_params = insertion_params.get('fang1')
        fang2_params = insertion_params.get('fang2')
        primary_fang_id = insertion_params.get('primary_fang')
        secondary_fang_id = insertion_params.get('secondary_fang')
        
        if not fang1_params or not fang2_params:
            rospy.logerr("Missing fang parameters for dual-fang insertion")
            return False
        
        rospy.loginfo(f"Fang1 ({fang1_params['config']['name']}):")
        rospy.loginfo(f"  Approach: {fang1_params['approach_position']}")
        rospy.loginfo(f"  Final: {fang1_params['final_position']}")
        rospy.loginfo(f"  Role: {'PRIMARY' if primary_fang_id == 'fang1' else 'SECONDARY'}")
        
        rospy.loginfo(f"Fang2 ({fang2_params['config']['name']}):")
        rospy.loginfo(f"  Approach: {fang2_params['approach_position']}")
        rospy.loginfo(f"  Final: {fang2_params['final_position']}")
        rospy.loginfo(f"  Role: {'PRIMARY' if primary_fang_id == 'fang2' else 'SECONDARY'}")
        
        # 计算双爪中心位置作为协调位置
        fang1_approach = fang1_params['approach_position']
        fang2_approach = fang2_params['approach_position']
        
        # 使用主爪位置作为主要目标，但考虑双爪的协调
        primary_fang_params = insertion_params[primary_fang_id]
        
        # 计算协调后的接近位置 (在两个爪子之间找到平衡点)
        coordination_weight = 0.7  # 70% 主爪权重，30% 副爪权重
        if primary_fang_id == 'fang1':
            coordinated_x = coordination_weight * fang1_approach[0] + (1-coordination_weight) * fang2_approach[0]
            coordinated_y = coordination_weight * fang1_approach[1] + (1-coordination_weight) * fang2_approach[1]
        else:
            coordinated_x = coordination_weight * fang2_approach[0] + (1-coordination_weight) * fang1_approach[0]
            coordinated_y = coordination_weight * fang2_approach[1] + (1-coordination_weight) * fang1_approach[1]
        
        # === 智能双爪几何约束 ===
        # 约束条件：阀门中心到无人机中心的距离 > 双爪中点到阀门中心的距离
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        
        # 计算双爪中点位置
        fang_midpoint_x = (fang1_approach[0] + fang2_approach[0]) / 2.0
        fang_midpoint_y = (fang1_approach[1] + fang2_approach[1]) / 2.0
        
        # 计算双爪中点到阀门中心的距离
        fang_midpoint_to_valve_dist = math.sqrt((fang_midpoint_x - valve_center_x)**2 + 
                                               (fang_midpoint_y - valve_center_y)**2)
        
        # 计算当前协调位置（无人机中心）到阀门中心的距离
        uav_to_valve_dist = math.sqrt((coordinated_x - valve_center_x)**2 + 
                                     (coordinated_y - valve_center_y)**2)
        
        rospy.loginfo(f"=== DUAL-FANG GEOMETRIC CONSTRAINT ANALYSIS ===")
        rospy.loginfo(f"Fang midpoint: ({fang_midpoint_x:.4f}, {fang_midpoint_y:.4f})")
        rospy.loginfo(f"Fang midpoint to valve distance: {fang_midpoint_to_valve_dist:.4f}m")
        rospy.loginfo(f"UAV center to valve distance: {uav_to_valve_dist:.4f}m")
        
        # 核心约束：无人机中心到阀门中心的距离必须大于双爪中点到阀门中心的距离
        # 这确保了无人机位置既考虑双爪物理关系，又保证合理的几何配置
        min_required_uav_distance = fang_midpoint_to_valve_dist + 0.02  # 添加2cm安全边距
        
        if uav_to_valve_dist < min_required_uav_distance:
            rospy.logwarn(f"UAV too close to valve center: {uav_to_valve_dist:.4f}m < {min_required_uav_distance:.4f}m")
            rospy.logwarn("Adjusting UAV position to satisfy dual-fang geometric constraint")
            
            # 沿着阀门中心到无人机中心的方向，调整无人机位置
            if uav_to_valve_dist > 0.001:  # 避免除零
                direction_x = (coordinated_x - valve_center_x) / uav_to_valve_dist
                direction_y = (coordinated_y - valve_center_y) / uav_to_valve_dist
            else:
                # 如果无人机正好在阀门中心，选择一个默认方向
                direction_x = 1.0
                direction_y = 0.0
            
            # 调整无人机位置到满足约束的最小距离
            coordinated_x = valve_center_x + direction_x * min_required_uav_distance
            coordinated_y = valve_center_y + direction_y * min_required_uav_distance
            
            # 重新计算调整后的距离
            uav_to_valve_dist = math.sqrt((coordinated_x - valve_center_x)**2 + 
                                         (coordinated_y - valve_center_y)**2)
            rospy.loginfo(f"Adjusted UAV position: ({coordinated_x:.4f}, {coordinated_y:.4f})")
            rospy.loginfo(f"New UAV to valve distance: {uav_to_valve_dist:.4f}m")
        
        # 额外的最大距离约束（防止过度偏离）
        max_distance_from_valve = 0.15  # 增加到15cm，给几何约束更多空间
        if uav_to_valve_dist > max_distance_from_valve:
            rospy.logwarn(f"UAV too far from valve center: {uav_to_valve_dist:.4f}m > {max_distance_from_valve:.4f}m")
            rospy.logwarn("Constraining UAV position to maximum allowed distance")
            
            scale_factor = max_distance_from_valve / uav_to_valve_dist
            coordinated_x = valve_center_x + (coordinated_x - valve_center_x) * scale_factor
            coordinated_y = valve_center_y + (coordinated_y - valve_center_y) * scale_factor
            
            uav_to_valve_dist = math.sqrt((coordinated_x - valve_center_x)**2 + 
                                         (coordinated_y - valve_center_y)**2)
            rospy.loginfo(f"Final constrained UAV position: ({coordinated_x:.4f}, {coordinated_y:.4f})")
            rospy.loginfo(f"Final UAV to valve distance: {uav_to_valve_dist:.4f}m")
        
        # 验证最终约束满足情况
        constraint_satisfied = uav_to_valve_dist >= fang_midpoint_to_valve_dist
        rospy.loginfo(f"Dual-fang geometric constraint: {'SATISFIED' if constraint_satisfied else 'VIOLATED'}")
        rospy.loginfo(f"Distance ratio: {uav_to_valve_dist/fang_midpoint_to_valve_dist:.3f} (should be > 1.0)")
        
        approach_x = coordinated_x
        approach_y = coordinated_y
        approach_z = self.approach_z  # Use approach Z for positioning phase
        approach_pos = (approach_x, approach_y, approach_z)
        
        rospy.loginfo(f"Coordinated dual-fang approach position: {approach_pos}")
        rospy.loginfo(f"Z coordinate for positioning: {approach_z:.6f}m (no descent yet)")
        
        # 使用主爪的目标偏航角
        target_yaw = primary_fang_params['target_yaw']
        
        # 保存双爪信息用于后续步骤
        self.dual_fang_info = {
            'fang1': fang1_params,
            'fang2': fang2_params,
            'primary_fang': primary_fang_id,
            'secondary_fang': secondary_fang_id,
            'coordination_weight': coordination_weight
        }
        
        current_yaw = self.current_yaw
        
        # Limit yaw change - increased range for better positioning
        yaw_diff = target_yaw - current_yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        max_yaw_change = math.pi / 2.5  # Increased from 60 degrees to 72 degrees for better insertion yaw adjustment
        if abs(yaw_diff) > max_yaw_change:
            yaw_diff = max_yaw_change * (1 if yaw_diff > 0 else -1)
        
        approach_yaw = current_yaw + yaw_diff
        
        rospy.loginfo(f"Approaching insertion point: {approach_pos}")
        rospy.loginfo(f"Z coordinate maintained at: {approach_z:.3f}m (positioning phase)")
        
        # Create a start position with the same approach Z
        start_pos_with_approach_z = (start_pos[0], start_pos[1], self.approach_z)
        return self.execute_smooth_trajectory(start_pos_with_approach_z, approach_pos, approach_yaw, duration=8.0, allow_z_movement=False)
    
    def adjust_yaw_and_circumferential_direction(self):
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        if not hasattr(self, 'current_insertion_params'):
            rospy.logerr("No insertion parameters available")
            return False
        
        if not hasattr(self, 'dual_fang_info'):
            rospy.logerr("No dual-fang coordination info available")
            return False
        
        insertion_params = self.current_insertion_params
        dual_fang_info = self.dual_fang_info
        
        # === 双爪协调的最终位置调整 ===
        rospy.loginfo("=== DUAL-FANG COORDINATED FINAL POSITION ADJUSTMENT ===")
        
        fang1_final = dual_fang_info['fang1']['final_position']
        fang2_final = dual_fang_info['fang2']['final_position']
        primary_fang_id = dual_fang_info['primary_fang']
        coordination_weight = dual_fang_info['coordination_weight']
        
        # 计算协调后的最终位置
        if primary_fang_id == 'fang1':
            coordinated_x = coordination_weight * fang1_final[0] + (1-coordination_weight) * fang2_final[0]
            coordinated_y = coordination_weight * fang1_final[1] + (1-coordination_weight) * fang2_final[1]
        else:
            coordinated_x = coordination_weight * fang2_final[0] + (1-coordination_weight) * fang1_final[0]
            coordinated_y = coordination_weight * fang2_final[1] + (1-coordination_weight) * fang1_final[1]
        
        # === 智能双爪几何约束（调整阶段）===
        # 约束条件：阀门中心到无人机中心的距离 > 双爪中点到阀门中心的距离
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        
        # 计算双爪中点位置
        fang_midpoint_x = (fang1_final[0] + fang2_final[0]) / 2.0
        fang_midpoint_y = (fang1_final[1] + fang2_final[1]) / 2.0
        
        # 计算双爪中点到阀门中心的距离
        fang_midpoint_to_valve_dist = math.sqrt((fang_midpoint_x - valve_center_x)**2 + 
                                               (fang_midpoint_y - valve_center_y)**2)
        
        # 计算当前协调位置（无人机中心）到阀门中心的距离
        uav_to_valve_dist = math.sqrt((coordinated_x - valve_center_x)**2 + 
                                     (coordinated_y - valve_center_y)**2)
        
        rospy.loginfo(f"=== DUAL-FANG GEOMETRIC CONSTRAINT (ADJUSTMENT PHASE) ===")
        rospy.loginfo(f"Fang midpoint: ({fang_midpoint_x:.4f}, {fang_midpoint_y:.4f})")
        rospy.loginfo(f"Fang midpoint to valve distance: {fang_midpoint_to_valve_dist:.4f}m")
        rospy.loginfo(f"UAV center to valve distance: {uav_to_valve_dist:.4f}m")
        
        # 核心约束：无人机中心到阀门中心的距离必须大于双爪中点到阀门中心的距离
        min_required_uav_distance = fang_midpoint_to_valve_dist + 0.015  # 添加1.5cm安全边距
        
        if uav_to_valve_dist < min_required_uav_distance:
            rospy.logwarn(f"UAV too close to valve center: {uav_to_valve_dist:.4f}m < {min_required_uav_distance:.4f}m")
            rospy.logwarn("Adjusting UAV position to satisfy dual-fang geometric constraint")
            
            # 沿着阀门中心到无人机中心的方向，调整无人机位置
            if uav_to_valve_dist > 0.001:  # 避免除零
                direction_x = (coordinated_x - valve_center_x) / uav_to_valve_dist
                direction_y = (coordinated_y - valve_center_y) / uav_to_valve_dist
            else:
                # 如果无人机正好在阀门中心，选择一个默认方向
                direction_x = 1.0
                direction_y = 0.0
            
            # 调整无人机位置到满足约束的最小距离
            coordinated_x = valve_center_x + direction_x * min_required_uav_distance
            coordinated_y = valve_center_y + direction_y * min_required_uav_distance
            
            # 重新计算调整后的距离
            uav_to_valve_dist = math.sqrt((coordinated_x - valve_center_x)**2 + 
                                         (coordinated_y - valve_center_y)**2)
            rospy.loginfo(f"Adjusted UAV position: ({coordinated_x:.4f}, {coordinated_y:.4f})")
            rospy.loginfo(f"New UAV to valve distance: {uav_to_valve_dist:.4f}m")
        
        # 额外的最大距离约束（防止过度偏离）
        max_distance_from_valve = 0.12  # 增加到12cm，给几何约束更多空间
        if uav_to_valve_dist > max_distance_from_valve:
            rospy.logwarn(f"UAV too far from valve center: {uav_to_valve_dist:.4f}m > {max_distance_from_valve:.4f}m")
            rospy.logwarn("Constraining UAV position to maximum allowed distance")
            
            scale_factor = max_distance_from_valve / uav_to_valve_dist
            coordinated_x = valve_center_x + (coordinated_x - valve_center_x) * scale_factor
            coordinated_y = valve_center_y + (coordinated_y - valve_center_y) * scale_factor
            
            uav_to_valve_dist = math.sqrt((coordinated_x - valve_center_x)**2 + 
                                         (coordinated_y - valve_center_y)**2)
            rospy.loginfo(f"Final constrained UAV position: ({coordinated_x:.4f}, {coordinated_y:.4f})")
            rospy.loginfo(f"Final UAV to valve distance: {uav_to_valve_dist:.4f}m")
        
        # 验证最终约束满足情况
        constraint_satisfied = uav_to_valve_dist >= fang_midpoint_to_valve_dist
        rospy.loginfo(f"Dual-fang geometric constraint: {'SATISFIED' if constraint_satisfied else 'VIOLATED'}")
        rospy.loginfo(f"Distance ratio: {uav_to_valve_dist/fang_midpoint_to_valve_dist:.3f} (should be > 1.0)")
        
        target_x = coordinated_x
        target_y = coordinated_y
        target_z = self.locked_z  # Use locked Z instead of current_pos[2]
        target_pos = (target_x, target_y, target_z)
        
        # 使用主爪的目标偏航角
        primary_fang_params = insertion_params[primary_fang_id]
        target_yaw = primary_fang_params['target_yaw']
        
        rospy.loginfo(f"Fang1 final position: {fang1_final}")
        rospy.loginfo(f"Fang2 final position: {fang2_final}")
        rospy.loginfo(f"Coordinated final position: {target_pos}")
        rospy.loginfo(f"Z coordinate LOCKED at: {target_z:.6f}m (prevents oscillation)")
        
        # Limit maximum XY adjustment for smooth motion 
        distance_adjustment = math.sqrt((target_x - current_pos[0])**2 + (target_y - current_pos[1])**2)
        max_xy_adjustment = 0.15  # Reduced to 0.15 for more stable motion
        if distance_adjustment > max_xy_adjustment:
            scale_factor = max_xy_adjustment / distance_adjustment
            target_x = current_pos[0] + (target_x - current_pos[0]) * scale_factor
            target_y = current_pos[1] + (target_y - current_pos[1]) * scale_factor
            target_pos = (target_x, target_y, target_z)
            rospy.loginfo(f"Limited XY adjustment to {max_xy_adjustment:.3f}m for smooth motion")
        
        rospy.loginfo(f"Adjusting to final position: {target_pos}")
        rospy.loginfo(f"Z coordinate maintained at: {target_z:.3f}m (positioning phase)")
        
        # Create a start position with the same approach Z
        start_pos_with_approach_z = (current_pos[0], current_pos[1], self.approach_z)
        return self.execute_smooth_trajectory(start_pos_with_approach_z, target_pos, target_yaw, duration=6.0, allow_z_movement=False)
    
    def descend_to_contact(self):
        """Execute ACTUAL descent to target insertion Z position"""
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        rospy.loginfo("=== ACTUAL DESCENT TO INSERTION POSITION ===")
        rospy.loginfo(f"Current position: {current_pos}")
        rospy.loginfo(f"Target insertion Z: {self.target_insertion_z:.6f}m")
        rospy.loginfo(f"Required descent: {current_pos[2] - self.target_insertion_z:.6f}m")
        
        # Get target insertion parameters for precise positioning
        if not hasattr(self, 'current_insertion_params'):
            rospy.logerr("No insertion parameters available for precise descent")
            return False
            
        if not hasattr(self, 'dual_fang_info'):
            rospy.logerr("No dual-fang coordination info available for descent")
            return False
        
        insertion_params = self.current_insertion_params
        dual_fang_info = self.dual_fang_info
        
        # === 双爪协调的精确下降 ===
        rospy.loginfo("=== DUAL-FANG COORDINATED PRECISE DESCENT ===")
        
        fang1_final = dual_fang_info['fang1']['final_position']
        fang2_final = dual_fang_info['fang2']['final_position']
        primary_fang_id = dual_fang_info['primary_fang']
        coordination_weight = dual_fang_info['coordination_weight']
        
        # 计算协调后的下降目标位置
        if primary_fang_id == 'fang1':
            coordinated_x = coordination_weight * fang1_final[0] + (1-coordination_weight) * fang2_final[0]
            coordinated_y = coordination_weight * fang1_final[1] + (1-coordination_weight) * fang2_final[1]
        else:
            coordinated_x = coordination_weight * fang2_final[0] + (1-coordination_weight) * fang1_final[0]
            coordinated_y = coordination_weight * fang2_final[1] + (1-coordination_weight) * fang1_final[1]
        
        # === 智能双爪几何约束（下降阶段）===
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        
        # 计算双爪中点位置
        fang_midpoint_x = (fang1_final[0] + fang2_final[0]) / 2.0
        fang_midpoint_y = (fang1_final[1] + fang2_final[1]) / 2.0
        
        # 计算双爪中点到阀门中心的距离
        fang_midpoint_to_valve_dist = math.sqrt((fang_midpoint_x - valve_center_x)**2 + 
                                               (fang_midpoint_y - valve_center_y)**2)
        
        # 计算当前协调位置（无人机中心）到阀门中心的距离
        uav_to_valve_dist = math.sqrt((coordinated_x - valve_center_x)**2 + 
                                     (coordinated_y - valve_center_y)**2)
        
        rospy.loginfo(f"=== DUAL-FANG GEOMETRIC CONSTRAINT (DESCENT PHASE) ===")
        rospy.loginfo(f"Fang midpoint: ({fang_midpoint_x:.4f}, {fang_midpoint_y:.4f})")
        rospy.loginfo(f"Fang midpoint to valve distance: {fang_midpoint_to_valve_dist:.4f}m")
        rospy.loginfo(f"UAV center to valve distance: {uav_to_valve_dist:.4f}m")
        
        # 核心约束：无人机中心到阀门中心的距离必须大于双爪中点到阀门中心的距离
        min_required_uav_distance = fang_midpoint_to_valve_dist + 0.01  # 添加1cm安全边距
        
        if uav_to_valve_dist < min_required_uav_distance:
            rospy.logwarn(f"UAV too close to valve center: {uav_to_valve_dist:.4f}m < {min_required_uav_distance:.4f}m")
            rospy.logwarn("Adjusting UAV position to satisfy dual-fang geometric constraint")
            
            # 沿着阀门中心到无人机中心的方向，调整无人机位置
            if uav_to_valve_dist > 0.001:  # 避免除零
                direction_x = (coordinated_x - valve_center_x) / uav_to_valve_dist
                direction_y = (coordinated_y - valve_center_y) / uav_to_valve_dist
            else:
                # 如果无人机正好在阀门中心，选择一个默认方向
                direction_x = 1.0
                direction_y = 0.0
            
            # 调整无人机位置到满足约束的最小距离
            coordinated_x = valve_center_x + direction_x * min_required_uav_distance
            coordinated_y = valve_center_y + direction_y * min_required_uav_distance
            
            # 重新计算调整后的距离
            uav_to_valve_dist = math.sqrt((coordinated_x - valve_center_x)**2 + 
                                         (coordinated_y - valve_center_y)**2)
            rospy.loginfo(f"Adjusted UAV position: ({coordinated_x:.4f}, {coordinated_y:.4f})")
            rospy.loginfo(f"New UAV to valve distance: {uav_to_valve_dist:.4f}m")
        
        # 额外的最大距离约束（防止过度偏离）
        max_distance_from_valve = 0.10  # 增加到10cm，给几何约束更多空间
        if uav_to_valve_dist > max_distance_from_valve:
            rospy.logwarn(f"UAV too far from valve center: {uav_to_valve_dist:.4f}m > {max_distance_from_valve:.4f}m")
            rospy.logwarn("Constraining UAV position to maximum allowed distance")
            
            scale_factor = max_distance_from_valve / uav_to_valve_dist
            coordinated_x = valve_center_x + (coordinated_x - valve_center_x) * scale_factor
            coordinated_y = valve_center_y + (coordinated_y - valve_center_y) * scale_factor
            
            uav_to_valve_dist = math.sqrt((coordinated_x - valve_center_x)**2 + 
                                         (coordinated_y - valve_center_y)**2)
            rospy.loginfo(f"Final constrained UAV position: ({coordinated_x:.4f}, {coordinated_y:.4f})")
            rospy.loginfo(f"Final UAV to valve distance: {uav_to_valve_dist:.4f}m")
        
        # 验证最终约束满足情况
        constraint_satisfied = uav_to_valve_dist >= fang_midpoint_to_valve_dist
        rospy.loginfo(f"Dual-fang geometric constraint: {'SATISFIED' if constraint_satisfied else 'VIOLATED'}")
        rospy.loginfo(f"Distance ratio: {uav_to_valve_dist/fang_midpoint_to_valve_dist:.3f} (should be > 1.0)")
        
        # CRITICAL: Use target insertion Z for actual descent
        target_x = coordinated_x
        target_y = coordinated_y
        target_z = self.target_insertion_z  # Use actual insertion Z
        target_pos = (target_x, target_y, target_z)
        
        # 使用主爪的目标偏航角
        primary_fang_params = insertion_params[primary_fang_id]
        target_yaw = primary_fang_params['target_yaw']
        
        rospy.loginfo(f"Fang1 final position: {fang1_final}")
        rospy.loginfo(f"Fang2 final position: {fang2_final}")
        rospy.loginfo(f"Final descent target: {target_pos}")
        rospy.loginfo(f"ACTUAL DESCENT: {current_pos[2]:.6f}m → {target_z:.6f}m")
        
        # Execute descent with precise feedback control
        return self.execute_precise_descent_with_feedback(
            start_pos=current_pos,
            target_pos=target_pos,
            target_yaw=target_yaw,
            descent_speed=self.descent_speed
        )
    
    def execute_precise_descent_with_feedback(self, start_pos, target_pos, target_yaw, descent_speed=0.03):
        """Execute precise descent with adaptive XY and yaw feedback control for accurate insertion"""
        rospy.loginfo("=== PRECISE DESCENT WITH ADAPTIVE XY/YAW FEEDBACK ===")
        
        # Adaptive thresholds for precise insertion
        xy_threshold_initial = 0.015  # Start with 15mm XY positioning tolerance
        xy_threshold_final = 0.025   # Relax to 25mm if needed
        yaw_threshold = 0.08  # Relaxed 4.6° yaw tolerance
        z_threshold = 0.02   # 2cm Z positioning tolerance
        
        # Control parameters for stable descent
        max_descent_step = 0.025  # Reduced to 2.5cm for more stable descent
        control_rate = 15  # Reduced to 15Hz for more stable control
        feedback_interval = 1.5  # 1.5 second feedback logging interval
        
        rospy.loginfo(f"Adaptive control thresholds: XY={xy_threshold_initial*1000:.0f}mm→{xy_threshold_final*1000:.0f}mm, Yaw={yaw_threshold*180/math.pi:.1f}°, Z={z_threshold*1000:.0f}mm")
        
        current_pos = start_pos
        target_x, target_y, target_z = target_pos
        
        # Calculate total descent distance
        total_descent = abs(target_z - start_pos[2])
        rospy.loginfo(f"Total descent distance: {total_descent:.3f}m")
        
        rate = rospy.Rate(control_rate)
        start_time = time.time()
        last_feedback_time = start_time
        
        # Adaptive control variables
        xy_error_history = []
        consecutive_high_error_count = 0
        current_xy_threshold = xy_threshold_initial
        
        # Collision detection variables
        position_history = []
        stuck_detection_window = 10  # 10 iterations for stuck detection
        
        while not rospy.is_shutdown():
            current_pos = self.get_current_position()
            if current_pos is None:
                rospy.logwarn("Lost position feedback during descent")
                rate.sleep()
                continue
            
            # Calculate current errors
            xy_error = math.sqrt((current_pos[0] - target_x)**2 + (current_pos[1] - target_y)**2)
            yaw_error = abs(self.normalize_angle(target_yaw - self.current_yaw))
            z_error = abs(current_pos[2] - target_z)
            
            # Store position history for collision detection
            position_history.append(current_pos)
            if len(position_history) > stuck_detection_window:
                position_history.pop(0)
            
            # Detect if UAV is stuck (collision with valve structure)
            if len(position_history) >= stuck_detection_window:
                recent_movement = math.sqrt(
                    (position_history[-1][0] - position_history[0][0])**2 +
                    (position_history[-1][1] - position_history[0][1])**2 +
                    (position_history[-1][2] - position_history[0][2])**2
                )
                
                if recent_movement < 0.01 and z_error > z_threshold:  # Less than 1cm movement but not at target
                    rospy.logwarn(f"UAV appears stuck - movement: {recent_movement*1000:.1f}mm in last {stuck_detection_window} iterations")
                    rospy.logwarn("Possible collision with valve structure - relaxing XY constraints")
                    current_xy_threshold = xy_threshold_final
            
            # Adaptive XY threshold based on error history
            xy_error_history.append(xy_error)
            if len(xy_error_history) > 20:  # Keep last 20 readings
                xy_error_history.pop(0)
            
            if len(xy_error_history) >= 10:
                avg_xy_error = sum(xy_error_history[-10:]) / 10
                if avg_xy_error > current_xy_threshold:
                    consecutive_high_error_count += 1
                    if consecutive_high_error_count > 15:  # 15 consecutive high errors
                        rospy.logwarn(f"Persistent high XY error ({avg_xy_error*1000:.1f}mm) - relaxing threshold")
                        current_xy_threshold = xy_threshold_final
                        consecutive_high_error_count = 0
                else:
                    consecutive_high_error_count = 0
            
            # Feedback logging
            current_time = time.time()
            if current_time - last_feedback_time > feedback_interval:
                rospy.loginfo(f"Precise descent feedback - XY: {xy_error*1000:.1f}mm (threshold: {current_xy_threshold*1000:.1f}mm), Yaw: {yaw_error*180/math.pi:.1f}°, Z: {z_error*1000:.1f}mm")
                last_feedback_time = current_time
            
            # Check if descent is complete with adaptive threshold
            if z_error < z_threshold and xy_error < current_xy_threshold and yaw_error < yaw_threshold:
                rospy.loginfo("Precise descent completed successfully")
                rospy.loginfo(f"Final precision - XY: {xy_error*1000:.1f}mm, Yaw: {yaw_error*180/math.pi:.1f}°, Z: {z_error*1000:.1f}mm")
                return True
            
            # Calculate next descent step - slower if XY error is high
            remaining_descent = current_pos[2] - target_z
            if remaining_descent > 0:
                # Slow down descent if XY error is high
                if xy_error > current_xy_threshold:
                    descent_step = min(max_descent_step * 0.5, remaining_descent)  # Half speed
                    rospy.logdebug(f"Slowing descent due to high XY error: {xy_error*1000:.1f}mm")
                else:
                    descent_step = min(max_descent_step, remaining_descent)
                
                next_z = current_pos[2] - descent_step
            else:
                # Already at or below target height
                next_z = target_z
            
            # Adaptive XY correction - more aggressive when error is high
            if xy_error > current_xy_threshold:
                # More aggressive correction for high errors
                correction_factor = min(0.6, xy_error / 0.03)  # Increased correction scaling
                corrected_x = current_pos[0] + correction_factor * (target_x - current_pos[0])
                corrected_y = current_pos[1] + correction_factor * (target_y - current_pos[1])
            else:
                # Gentle correction for small errors
                correction_factor = min(0.2, xy_error / 0.01)  # Gentle correction scaling
                corrected_x = current_pos[0] + correction_factor * (target_x - current_pos[0])
                corrected_y = current_pos[1] + correction_factor * (target_y - current_pos[1])
            
            # Send precise command
            precise_command_pos = (corrected_x, corrected_y, next_z)
            self.motion_controller.send_trajectory_point(precise_command_pos, target_yaw)
            
            # Safety timeout
            if time.time() - start_time > 45.0:  # Extended timeout to 45 seconds
                rospy.logerr("Precise descent timed out")
                rospy.logerr(f"Final state - XY: {xy_error*1000:.1f}mm, Yaw: {yaw_error*180/math.pi:.1f}°, Z: {z_error*1000:.1f}mm")
                return False
            
            rate.sleep()
        
        rospy.logerr("Precise descent interrupted")
        return False
    
    def execute_angular_insertion(self):
        """Execute angular insertion to avoid valve handle plane blockage"""
        
        rospy.loginfo("=== ANGULAR INSERTION TO AVOID HANDLE PLANE BLOCKAGE ===")
        rospy.loginfo("Problem: End-effector blocked by valve handle plane")
        rospy.loginfo("Solution: Angular approach to bypass handle plane")
        
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        # Calculate valve center and current relative position
        valve_center_x, valve_center_y, valve_center_z = self.valve_pos
        start_x, start_y, start_z = current_pos
        
        # Calculate current angle from valve center
        current_angle = math.atan2(start_y - valve_center_y, start_x - valve_center_x)
        current_radius = math.sqrt((start_x - valve_center_x)**2 + (start_y - valve_center_y)**2)
        
        rospy.loginfo(f"Current position relative to valve:")
        rospy.loginfo(f"  - Angle: {current_angle:.3f} rad ({current_angle*180/math.pi:.1f}°)")
        rospy.loginfo(f"  - Radius: {current_radius:.3f}m")
        rospy.loginfo(f"  - Height: {start_z:.3f}m")
        
        # FIXED: Use current UAV Z position to avoid Z-axis oscillation
        # Never use valve_center_z + height calculations as they cause Z jumps
        current_z = start_z
        clearance_height = current_z  # Use current Z position, no vertical movement
        rospy.loginfo(f"Z-axis oscillation fix: Using current UAV Z position: {clearance_height:.3f}m")
        rospy.loginfo("Avoiding valve_center_z + handle_clearance_height calculation")
        
        # First, move to clearance height to avoid handle plane
        clearance_pos = (start_x, start_y, clearance_height)
        if not self.execute_vertical_movement(current_pos, clearance_pos, "clearance"):
            return False
        
        # Stage 2: Angular approach - move closer to valve center at clearance height
        insertion_radius = current_radius - self.insertion_angle_offset
        min_safe_radius = 0.12  # Increased from 0.05m to 0.12m for safer approach
        if insertion_radius < min_safe_radius:
            insertion_radius = min_safe_radius
            rospy.logwarn(f"Limiting insertion radius to safe minimum: {insertion_radius:.3f}m")
        
        # Calculate angular approach position
        angular_approach_x = valve_center_x + insertion_radius * math.cos(current_angle)
        angular_approach_y = valve_center_y + insertion_radius * math.sin(current_angle)
        angular_approach_pos = (angular_approach_x, angular_approach_y, clearance_height)
        
        rospy.loginfo(f"Stage 2: Angular approach to: [{angular_approach_x:.3f}, {angular_approach_y:.3f}, {clearance_height:.3f}]")
        
        # Execute angular approach
        if not self.execute_smooth_trajectory(clearance_pos, angular_approach_pos, self.current_yaw, duration=6.0):
            return False
        
        # Stage 3: Descent to final insertion position
        # FIXED: Use absolute Z position to avoid oscillation
        final_target_z = valve_center_z + self.z_offset
        final_target_pos = (angular_approach_x, angular_approach_y, final_target_z)
        
        rospy.loginfo(f"Stage 3: Angled descent to final insertion position:")
        rospy.loginfo(f"  - Target: [{angular_approach_x:.3f}, {angular_approach_y:.3f}, {final_target_z:.6f}]")
        rospy.loginfo(f"  - Descent distance: {clearance_height - final_target_z:.3f}m")
        
        # Execute descent
        return self.motion_controller.execute_controlled_descent(
            start_pos=angular_approach_pos,
            target_z=final_target_z,
            descent_speed=self.descent_speed,
            pos_threshold=0.02,
            yaw_threshold=0.1
        )
    
    def execute_vertical_movement(self, start_pos, target_pos, stage_name):
        """Execute vertical movement for clearance"""
        rospy.loginfo(f"Executing {stage_name} vertical movement...")
        
        # Calculate movement distance for logging
        vertical_distance = abs(target_pos[2] - start_pos[2])
        rospy.loginfo(f"{stage_name} vertical distance: {vertical_distance:.3f}m")
        rospy.loginfo(f"From height: {start_pos[2]:.3f}m to {target_pos[2]:.3f}m")
        
        fixed_x, fixed_y = start_pos[0], start_pos[1]
        fixed_yaw = self.current_yaw
        
        start_time = time.time()
        rate = rospy.Rate(50)
        timeout = 10.0  # Increased timeout
        
        # Use more relaxed threshold for vertical movement
        height_threshold = 0.03  # Increased from 0.02m to 0.03m
        
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            current_pos = self.get_current_position()
            if current_pos is None:
                continue
            
            # Check if reached target height
            height_diff = abs(current_pos[2] - target_pos[2])
            if height_diff < height_threshold:
                rospy.loginfo(f"{stage_name} vertical movement completed (height diff: {height_diff:.3f}m)")
                return True
            
            # Send command to move to target height
            self.motion_controller.send_trajectory_point((fixed_x, fixed_y, target_pos[2]), fixed_yaw)
            
            rate.sleep()
        
        rospy.logerr(f"{stage_name} vertical movement timed out")
        return False
    
    def execute_smooth_trajectory(self, start_pos, target_pos, target_yaw, duration=5.0,
                                  pos_threshold=0.035, yaw_threshold=0.10, allow_z_movement=None):
        """Execute smooth trajectory with yaw control and feedback, with optional Z movement control."""
        from trajectory import PolynomialTrajectory
        
        # Auto-detect Z movement based on whether it's needed
        if allow_z_movement is None:
            z_diff = abs(target_pos[2] - start_pos[2])
            allow_z_movement = z_diff > 0.01  # Allow Z movement if difference > 1cm
        
        if allow_z_movement:
            # Normal trajectory with Z movement allowed
            stable_start_pos = start_pos
            stable_target_pos = target_pos
            rospy.loginfo(f"Executing trajectory with Z movement allowed:")
            rospy.loginfo(f"  Start: [{stable_start_pos[0]:.3f}, {stable_start_pos[1]:.3f}, {stable_start_pos[2]:.6f}]")
            rospy.loginfo(f"  Target: [{stable_target_pos[0]:.3f}, {stable_target_pos[1]:.3f}, {stable_target_pos[2]:.6f}]")
            rospy.loginfo(f"  Z movement: {stable_start_pos[2]:.6f} → {stable_target_pos[2]:.6f} (Δ{stable_target_pos[2]-stable_start_pos[2]:.6f}m)")
        else:
            # Z-locked trajectory for positioning phases
            if hasattr(self, 'approach_z'):
                stable_z = self.approach_z
            else:
                stable_z = start_pos[2]
            
            stable_start_pos = (start_pos[0], start_pos[1], stable_z)
            stable_target_pos = (target_pos[0], target_pos[1], stable_z)
            
            rospy.loginfo(f"Executing trajectory with Z-lock:")
            rospy.loginfo(f"  Start: [{stable_start_pos[0]:.3f}, {stable_start_pos[1]:.3f}, {stable_start_pos[2]:.6f}]")
            rospy.loginfo(f"  Target: [{stable_target_pos[0]:.3f}, {stable_target_pos[1]:.3f}, {stable_target_pos[2]:.6f}]")
            rospy.loginfo(f"  Z LOCKED at: {stable_z:.6f}m (positioning phase)")
        
        # Create trajectory
        traj = PolynomialTrajectory(duration)
        traj.generate_trajectory(stable_start_pos, stable_target_pos)
        
        # Store initial yaw for consistent interpolation
        start_yaw = self.current_yaw
        
        # Calculate shortest yaw path with improved normalization
        yaw_diff = target_yaw - start_yaw
        # Normalize to [-pi, pi] range for shortest path
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Limit maximum yaw change per trajectory
        max_yaw_change = math.pi / 2.8  # ~64 degrees
        if abs(yaw_diff) > max_yaw_change:
            yaw_diff = max_yaw_change * (1 if yaw_diff > 0 else -1)
            rospy.loginfo(f"Limiting yaw change to {yaw_diff:.3f} rad ({yaw_diff*180/math.pi:.1f}°)")
        
        final_target_yaw = start_yaw + yaw_diff
        
        rate = rospy.Rate(6)  # 6Hz for smooth motion
        start_time = time.time()
        
        rospy.loginfo(f"Trajectory: yaw from {start_yaw:.3f} to {final_target_yaw:.3f} (change: {yaw_diff:.3f} rad)")
        
        # Track previous command to avoid redundant sends
        last_command_pos = None
        last_command_yaw = None
        
        while not rospy.is_shutdown() and (time.time() - start_time) < duration + 8.0:
            pt = traj.evaluate()
            
            # Interpolate yaw smoothly with quintic smoothing
            elapsed_time = time.time() - start_time
            if elapsed_time <= duration:
                yaw_progress = elapsed_time / duration
                # Quintic smoothing for gentle motion
                smooth_yaw_progress = 6*yaw_progress**5 - 15*yaw_progress**4 + 10*yaw_progress**3
                current_target_yaw = start_yaw + smooth_yaw_progress * yaw_diff
            else:
                current_target_yaw = final_target_yaw

            if pt is None:
                pt = stable_target_pos
            
            # Only send command if position or yaw has changed significantly
            if (last_command_pos is None or 
                abs(pt[0] - last_command_pos[0]) > 0.003 or
                abs(pt[1] - last_command_pos[1]) > 0.003 or
                abs(pt[2] - last_command_pos[2]) > 0.003 or
                abs(current_target_yaw - (last_command_yaw or 0)) > 0.02):
                
                # Send trajectory point
                self.motion_controller.send_trajectory_point(pt, current_target_yaw)
                last_command_pos = pt
                last_command_yaw = current_target_yaw

            # Wait until UAV reaches the intermediate point
            wait_start_time = time.time()
            while not rospy.is_shutdown() and (time.time() - wait_start_time) < 8.0:
                current_pos = self.get_current_position()
                if current_pos is None:
                    rate.sleep()
                    continue

                # Calculate position error
                pos_error = math.sqrt((pt[0] - current_pos[0])**2 + 
                                    (pt[1] - current_pos[1])**2 + 
                                    (pt[2] - current_pos[2])**2)
                yaw_error = abs(self.normalize_angle(current_target_yaw - self.current_yaw))

                # Relaxed thresholds
                relaxed_pos_threshold = pos_threshold * 1.5
                relaxed_yaw_threshold = yaw_threshold * 1.3

                if pos_error < relaxed_pos_threshold and yaw_error < relaxed_yaw_threshold:
                    break
                
                rate.sleep()
            
            # Check if we've reached the final target
            if (abs(pt[0] - stable_target_pos[0]) < 0.001 and 
                abs(pt[1] - stable_target_pos[1]) < 0.001 and
                abs(pt[2] - stable_target_pos[2]) < 0.001):
                break

        rospy.loginfo("Trajectory execution completed.")
        return True
    
    def rotate_to_target_position(self):
        """Complete insertion and prepare for rotation by moving to safe rotation distance"""
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        rospy.loginfo("=== PREPARING FOR ROTATION ===")
        rospy.loginfo("Step 1: Completing insertion")
        rospy.sleep(0.5)  # Brief pause to stabilize
        
        # Check if we have insertion parameters
        if not hasattr(self, 'current_insertion_params'):
            rospy.logwarn("No insertion parameters available for final rotation")
            return True  # Continue anyway
        
        # Get the primary fang's target yaw for final adjustment
        insertion_params = self.current_insertion_params
        target_yaw = self.current_yaw  # Default to current yaw
        
        if 'primary_fang' in insertion_params:
            primary_fang_id = insertion_params['primary_fang']
            primary_fang_params = insertion_params[primary_fang_id]
            target_yaw = primary_fang_params['target_yaw']
            
            # Make a small yaw adjustment to final target orientation
            yaw_error = abs(self.normalize_angle(target_yaw - self.current_yaw))
            if yaw_error > 0.05:  # If yaw error > 2.9 degrees
                rospy.loginfo(f"Making final yaw adjustment: {yaw_error*180/math.pi:.1f}°")
                
                # Send a gentle yaw adjustment command
                final_pos = (current_pos[0], current_pos[1], current_pos[2])
                self.motion_controller.send_trajectory_point(final_pos, target_yaw)
                
                # Wait for yaw adjustment to complete
                rospy.sleep(1.0)
        
        rospy.loginfo("Step 2: Moving to safe rotation distance")
        
        # Calculate current distance to valve center
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        current_distance = math.sqrt((current_pos[0] - valve_center_x)**2 + 
                                   (current_pos[1] - valve_center_y)**2)
        
        # Required distance for safe rotation (based on trajectory creation requirements)
        required_rotation_distance = 0.25  # 25cm minimum distance for safe rotation
        
        rospy.loginfo(f"Current distance to valve center: {current_distance:.3f}m")
        rospy.loginfo(f"Required distance for rotation: {required_rotation_distance:.3f}m")
        
        if current_distance < required_rotation_distance:
            rospy.loginfo("UAV too close for rotation - moving to safe distance")
            
            # Calculate direction away from valve center
            direction_x = (current_pos[0] - valve_center_x) / current_distance
            direction_y = (current_pos[1] - valve_center_y) / current_distance
            
            # Calculate new position at safe distance
            safe_x = valve_center_x + direction_x * required_rotation_distance
            safe_y = valve_center_y + direction_y * required_rotation_distance
            safe_z = current_pos[2]  # Keep same height
            
            safe_rotation_pos = (safe_x, safe_y, safe_z)
            
            rospy.loginfo(f"Moving to safe rotation position: ({safe_x:.3f}, {safe_y:.3f}, {safe_z:.3f})")
            
            # Execute smooth movement to safe rotation position
            success = self.execute_smooth_trajectory(
                start_pos=current_pos,
                target_pos=safe_rotation_pos,
                target_yaw=target_yaw,
                duration=3.0,
                pos_threshold=0.02,
                yaw_threshold=0.1,
                allow_z_movement=True
            )
            
            if not success:
                rospy.logerr("Failed to move to safe rotation position")
                return False
            
            # Verify final position
            final_pos = self.get_current_position()
            if final_pos is not None:
                final_distance = math.sqrt((final_pos[0] - valve_center_x)**2 + 
                                         (final_pos[1] - valve_center_y)**2)
                rospy.loginfo(f"Final distance to valve center: {final_distance:.3f}m")
                
                if final_distance >= required_rotation_distance:
                    rospy.loginfo("Successfully positioned for rotation")
                else:
                    rospy.logwarn(f"Still too close for rotation: {final_distance:.3f}m < {required_rotation_distance:.3f}m")
            
        else:
            rospy.loginfo("UAV already at safe distance for rotation")
        
        rospy.loginfo("Insertion completed - ready for valve rotation")
        return True


class RotateValveState(SingleUAVStateBase):
    def __init__(self, module_id=1, rotation_angle=math.pi/2, rotation_duration=8.0):
        super().__init__(outcomes=['succeeded', 'failed', 'emergency'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        self.rotation_angle = rotation_angle
        self.rotation_duration = rotation_duration
        
        # Emergency detection - made less sensitive for rotation
        self.emergency_stop = threading.Event()
        self.last_position = None
        self.last_yaw = 0.0
        self.stuck_start_time = None
        self.stuck_threshold = 30.0  # Increased from 25s to 30s
        self.movement_threshold = 0.04  # Increased from 0.02 to 0.04
        self.yaw_threshold = 0.15  # Increased from 0.1 to 0.15
    
    def execute(self, userdata):
        rospy.loginfo("Starting valve rotation...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        current_pos = self.get_current_position()
        if current_pos is None:
            return 'failed'
        
        # Initialize tracking
        self.last_position = current_pos
        self.last_yaw = self.current_yaw
        self.stuck_start_time = None
        
        # Create rotation trajectory
        rotation_traj = create_constant_distance_trajectory(
            current_uav_pos=current_pos,
            current_uav_yaw=self.current_yaw,
            valve_center=self.valve_pos,
            rotation_angle=self.rotation_angle,
            rotation_duration=self.rotation_duration,
            end_effector_offset_x=0.246,
            end_effector_offset_y=0.0,
            end_effector_offset_z=0.0743823,
            rotation_direction=1
        )
        
        if rotation_traj is None:
            rospy.logerr("Failed to create rotation trajectory")
            return 'failed'
        
        # Start trajectory
        rotation_traj.start_trajectory()
        
        # Start emergency monitoring with less frequent checks
        emergency_thread = threading.Thread(target=self._emergency_monitor_thread)
        emergency_thread.daemon = True
        emergency_thread.start()
        
        # Execute rotation
        try:
            success = self.execute_rotation_trajectory(rotation_traj)
            
            # Stop emergency monitoring
            self.emergency_stop.set()
            
            if self.emergency_stop.is_set():
                rospy.logwarn("Emergency detected during rotation")
                return 'emergency'
            
            if success:
                rospy.loginfo("Valve rotation completed successfully")
                return 'succeeded'
            else:
                rospy.logerr("Valve rotation failed")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"Error during rotation: {e}")
            self.emergency_stop.set()
            return 'failed'
    
    def _emergency_monitor_thread(self):
        rate = rospy.Rate(1)  # Reduced frequency from 2Hz to 1Hz
        
        while not self.emergency_stop.is_set() and not rospy.is_shutdown():
            if self.check_rotation_emergency():
                rospy.logwarn("Emergency condition detected - UAV stuck for too long")
                self.emergency_stop.set()
                break
            rate.sleep()
    
    def check_rotation_emergency(self):
        """Check for emergency conditions during rotation with more lenient thresholds"""
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        if self.last_position is not None:
            # Calculate movement since last check
            movement = math.sqrt(
                (current_pos[0] - self.last_position[0])**2 + 
                (current_pos[1] - self.last_position[1])**2 + 
                (current_pos[2] - self.last_position[2])**2
            )
            
            yaw_movement = abs(self.normalize_angle(self.current_yaw - self.last_yaw))
            
            # More lenient movement detection during rotation
            # During rotation, UAV should be moving in circular path
            movement_threshold = getattr(self, 'movement_threshold', 0.02)
            yaw_threshold = getattr(self, 'yaw_threshold', 0.1)
            
            # Check if UAV is stuck (no movement for extended period)
            if movement < movement_threshold and yaw_movement < yaw_threshold:
                if self.stuck_start_time is None:
                    self.stuck_start_time = time.time()
                    rospy.loginfo("UAV movement below threshold, starting stuck timer")
                else:
                    stuck_duration = time.time() - self.stuck_start_time
                    if stuck_duration > self.stuck_threshold:
                        rospy.logerr(f"UAV stuck for {stuck_duration:.1f}s - triggering emergency")
                        return True
                    elif stuck_duration > 10.0:  # Log warning every 10s
                        rospy.logwarn(f"UAV potentially stuck for {stuck_duration:.1f}s")
            else:
                # Reset stuck timer if movement detected
                if self.stuck_start_time is not None:
                    rospy.loginfo("UAV movement detected, resetting stuck timer")
                self.stuck_start_time = None
        
        # Update last position and yaw
        self.last_position = current_pos
        self.last_yaw = self.current_yaw
        return False
    
    def execute_rotation_trajectory(self, rotation_traj):
        """Execute rotation trajectory with proper feedback control"""
        rate = rospy.Rate(30)  # Reduced from 50Hz to 30Hz for smoother control
        start_time = rospy.get_time()
        
        # More lenient emergency detection parameters during rotation
        rotation_movement_threshold = 0.08  # Further increased from 0.05
        rotation_yaw_threshold = 0.20  # Further increased from 0.15
        
        last_feedback_time = start_time
        feedback_interval = 1.0  # Increased from 0.5s to 1.0s for less frequent logging
        
        rospy.loginfo("Starting rotation trajectory execution with feedback")
        
        while not rospy.is_shutdown() and not self.emergency_stop.is_set():
            # Get next trajectory point
            result = rotation_traj.get_next_position_and_yaw()
            if result is None:
                rospy.loginfo("Rotation trajectory completed")
                break
            
            target_pos, target_yaw = result
            
            # Send trajectory point
            self.motion_controller.send_trajectory_point(target_pos, target_yaw)
            
            # Feedback control - wait for UAV to reach close to target
            current_time = rospy.get_time()
            if current_time - last_feedback_time > feedback_interval:
                current_pos = self.get_current_position()
                if current_pos is not None:
                    pos_error = math.sqrt((target_pos[0] - current_pos[0])**2 + 
                                        (target_pos[1] - current_pos[1])**2 + 
                                        (target_pos[2] - current_pos[2])**2)
                    yaw_error = abs(self.normalize_angle(target_yaw - self.current_yaw))
                    
                    rospy.loginfo(f"Rotation feedback - Pos error: {pos_error:.3f}m, Yaw error: {yaw_error:.3f}rad")
                    
                    # Adjust emergency detection thresholds during rotation
                    self.movement_threshold = rotation_movement_threshold
                    self.yaw_threshold = rotation_yaw_threshold
                    
                last_feedback_time = current_time
            
            # Wait for position to converge (with longer timeout)
            wait_start = rospy.get_time()
            while not rospy.is_shutdown() and not self.emergency_stop.is_set():
                if rospy.get_time() - wait_start > 0.2:  # Increased from 0.1s to 0.2s
                    break
                    
                current_pos = self.get_current_position()
                if current_pos is not None:
                    pos_error = math.sqrt((target_pos[0] - current_pos[0])**2 + 
                                        (target_pos[1] - current_pos[1])**2 + 
                                        (target_pos[2] - current_pos[2])**2)
                    
                    if pos_error < 0.12:  # Increased from 8cm to 12cm tolerance for rotation
                        break
                
                # Re-send command for better tracking
                self.motion_controller.send_trajectory_point(target_pos, target_yaw)
                rate.sleep()
            
            rate.sleep()
        
        rospy.loginfo("Rotation trajectory execution completed")
        return True


def main():
    rospy.init_node('valve_rotation_single_uav')
    
    module_id = rospy.get_param('~module_id', 1)
    rospy.loginfo(f"Starting valve rotation for UAV module {module_id}")
    
    # Create state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    sm.userdata.start_position = None
    
    with sm:
        smach.StateMachine.add(
            'INITIALIZE_START_POSITION',
            InitializeStartPositionState(module_id=module_id),
            transitions={'succeeded': 'MOVE_TO_VALVE', 'failed': 'failed'}
        )
        
        smach.StateMachine.add(
            'MOVE_TO_VALVE',
            MoveToValveState(module_id=module_id),
            transitions={'succeeded': 'DESCEND_AND_CONTACT', 'failed': 'failed'}
        )
        
        smach.StateMachine.add(
            'DESCEND_AND_CONTACT',
            DescendAndContactState(module_id=module_id),
            transitions={'succeeded': 'ROTATE_VALVE', 'failed': 'failed'}
        )
        
        smach.StateMachine.add(
            'ROTATE_VALVE',
            RotateValveState(module_id=module_id),
            transitions={
                'succeeded': 'succeeded',
                'failed': 'failed',
                'emergency': 'failed'
            }
        )
    
    # Create introspection server
    sis = smach_ros.IntrospectionServer('valve_rotation_state_machine', sm, '/SM_ROOT')
    sis.start()
    
    try:
        rospy.loginfo("Starting valve rotation state machine...")
        outcome = sm.execute()
        rospy.loginfo(f"State machine completed with outcome: {outcome}")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Valve rotation interrupted")
    except Exception as e:
        rospy.logerr(f"Valve rotation error: {e}")
    finally:
        sis.stop()


if __name__ == '__main__':
    main()
