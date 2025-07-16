#!/usr/bin/env python
"""
Constrained Insertion Optimizer with Real Constraints
Implements proper constraint optimization for valve insertion task
"""

import math
import numpy as np
from scipy.optimize import minimize
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class ConstrainedInsertionOptimizer:
    """
    Proper constraint optimization for valve insertion
    """
    
    def __init__(self, valve_radius=0.1225, valve_beam_width=0.0185, 
                 end_effector_length=0.246, safety_margin=0.005, module_id=1):
        """
        Initialize constrained optimizer
        
        Args:
            valve_radius: Valve radius (m)
            valve_beam_width: Beam width (m)
            end_effector_length: End-effector length (m)
            safety_margin: Safety margin (m)
            module_id: UAV module ID
        """
        self.valve_radius = valve_radius
        self.valve_beam_width = valve_beam_width
        self.end_effector_length = end_effector_length
        self.safety_margin = safety_margin
        self.module_id = module_id
        
        # Physical constraints
        self.max_yaw_change = math.pi / 2  # 90 degrees max
        self.max_approach_distance = 2.0  # 2m max approach
        self.min_beam_clearance = 0.01  # 1cm minimum clearance
        self.beam_clearance = 0.01  # 1cm minimum beam clearance
        
        # URDF-based fang geometry (from beetle.urdf.xacro)
        self.fang_to_base_x = 0.246        # Fang连接到base_link的X偏移
        self.fang_to_base_y = 0.0          # Fang连接到base_link的Y偏移
        self.fang_to_base_z = 0.0743823    # Fang连接到base_link的Z偏移
        
        # 两个fang的collision位置 (相对于beetle_fang_link)
        self.left_fang_y = 0.08438         # 左fang的Y偏移
        self.right_fang_y = -0.08438       # 右fang的Y偏移
        self.fang_collision_x = 0.0072     # 两个fang的X偏移 (近似相同)
        self.fang_collision_z = -0.0522683 # 两个fang的Z偏移
        
        # 两个fang之间的固定距离
        self.fixed_inter_fang_distance = 0.16876  # 168.76mm (固定值)
        
        # Current state
        self.current_uav_pos = None
        self.current_uav_yaw = 0.0
        self.valve_pos = None
        self.valve_yaw = 0.0
        
        # Setup ROS subscribers
        self._setup_subscribers()
        
        rospy.loginfo("Constrained insertion optimizer initialized")
        rospy.loginfo(f"Valve radius: {valve_radius:.4f}m")
        rospy.loginfo(f"End-effector length: {end_effector_length:.4f}m")
        rospy.loginfo(f"Safety margin: {safety_margin:.4f}m")
        rospy.loginfo(f"Fixed inter-fang distance: {self.fixed_inter_fang_distance:.4f}m")
    
    def _setup_subscribers(self):
        """Setup ROS subscribers"""
        uav_topic = f"/beetle{self.module_id}/mocap/pose"
        valve_topic = "/valve/odom"
        
        self.uav_sub = rospy.Subscriber(uav_topic, PoseStamped, self._uav_callback)
        self.valve_sub = rospy.Subscriber(valve_topic, Odometry, self._valve_callback)
    
    def _uav_callback(self, msg):
        """UAV position callback"""
        self.current_uav_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        q = msg.pose.orientation
        _, _, self.current_uav_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def _valve_callback(self, msg):
        """Valve position callback"""
        self.valve_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        q = msg.pose.pose.orientation
        _, _, self.valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def _objective_function(self, x, current_pos, current_yaw):
        """
        Radial configuration optimization objective function
        
        Args:
            x: [body_x, body_y, body_yaw] - optimization variables
            current_pos: Current UAV position
            current_yaw: Current UAV yaw
            
        Returns:
            float: Objective value (to minimize)
        """
        body_x, body_y, body_yaw = x
        valve_center_x, valve_center_y, _ = self.valve_pos
        
        # Calculate end-effector position
        end_eff_x = body_x + self.end_effector_length * math.cos(body_yaw)
        end_eff_y = body_y + self.end_effector_length * math.sin(body_yaw)
        
        # Primary objective: optimize radial configuration geometric quality
        # 1. End-effector to valve center distance (should be close but less than valve radius)
        end_eff_valve_distance = math.sqrt((end_eff_x - valve_center_x)**2 + 
                                          (end_eff_y - valve_center_y)**2)
        
        # Ideal end-effector distance: close to valve radius but with small gap
        ideal_end_eff_distance = self.valve_radius - 0.02  # 2cm gap
        distance_error = abs(end_eff_valve_distance - ideal_end_eff_distance)
        
        # 2. Body safety distance (no longer enforces three-point collinearity)
        body_valve_distance = math.sqrt((body_x - valve_center_x)**2 + 
                                       (body_y - valve_center_y)**2)
        
        # Ensure body maintains safe distance outside end-effector
        # CRITICAL FIX: Consider end-effector length (0.246m) in safety calculation
        end_effector_length = 0.246  # Distance from UAV center to end-effector
        min_body_distance = self.valve_radius + end_effector_length + 0.01  # Additional 1cm safety margin
        body_safety_error = max(0, min_body_distance - body_valve_distance)
        
        # 3. Secondary objective: minimize movement distance and angle change
        approach_distance = math.sqrt((body_x - current_pos[0])**2 + 
                                     (body_y - current_pos[1])**2)
        yaw_change = abs(self._normalize_angle(body_yaw - current_yaw))
        
        # Comprehensive objective function (weighted)
        # Removed radial alignment quality (three-point collinearity) constraint
        objective = (5.0 * distance_error +      # Primary: end-effector distance optimization
                    3.0 * body_safety_error +    # Primary: body safety distance
                    1.0 * approach_distance +    # Secondary: movement distance
                    1.0 * yaw_change)           # Secondary: angle change
        
        return objective
    
    def _constraints(self, x):
        """
        Optimization constraints
        
        Args:
            x: [body_x, body_y, body_yaw] - optimization variables
            
        Returns:
            list: Constraint violations (should be >= 0)
        """
        body_x, body_y, body_yaw = x
        constraints = []
        
        # Calculate end-effector position
        end_eff_x = body_x + self.end_effector_length * math.cos(body_yaw)
        end_eff_y = body_y + self.end_effector_length * math.sin(body_yaw)
        
        # Constraint 1: End-effector should be at valve radius
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        valve_distance = math.sqrt((end_eff_x - valve_center_x)**2 + 
                                  (end_eff_y - valve_center_y)**2)
        radius_constraint = self.safety_margin - abs(valve_distance - self.valve_radius)
        constraints.append(radius_constraint)
        
        # Constraint 2: End-effector should be in correct gap
        end_eff_angle = math.atan2(end_eff_y - valve_center_y, end_eff_x - valve_center_x)
        end_eff_angle = self._normalize_angle(end_eff_angle - self.valve_yaw)
        
        # Check if in beam0-beam1 gap (0° to 120°) or beam1-beam2 gap (120° to 240°)
        in_gap01 = (end_eff_angle >= 0 and end_eff_angle <= 2*math.pi/3)
        in_gap12 = (end_eff_angle >= 2*math.pi/3 and end_eff_angle <= 4*math.pi/3)
        
        if not (in_gap01 or in_gap12):
            constraints.append(-1.0)  # Violation
        else:
            constraints.append(0.1)  # Satisfied
        
        # Constraint 3: Minimum clearance from beams
        beam_angles = [0, 2*math.pi/3, 4*math.pi/3]  # 0°, 120°, 240°
        min_clearance = float('inf')
        
        for beam_angle in beam_angles:
            clearance = abs(self._normalize_angle(end_eff_angle - beam_angle))
            clearance = min(clearance, 2*math.pi - clearance)
            min_clearance = min(min_clearance, clearance)
        
        beam_clearance_constraint = min_clearance - self.min_beam_clearance
        constraints.append(beam_clearance_constraint)
        
        # Constraint 4: 正确的径向扩张配置约束
        # 理想配置：阀门中心 → 末端执行器中心 → 机体中心 (径向扩张)
        # 这确保了"胸部离开阀门本体，手抓住阀门"的正确姿态
        
        # 机体距离阀门中心的距离
        body_valve_distance = math.sqrt((body_x - valve_center_x)**2 + 
                                       (body_y - valve_center_y)**2)
        
        # 末端执行器距离阀门中心的距离
        end_eff_valve_distance = math.sqrt((end_eff_x - valve_center_x)**2 + 
                                          (end_eff_y - valve_center_y)**2)
        
        # 径向扩张约束：机体必须在末端执行器外侧，且呈径向排列
        # 计算阀门中心到机体的向量和阀门中心到末端执行器的向量
        body_angle = math.atan2(body_y - valve_center_y, body_x - valve_center_x)
        end_eff_angle = math.atan2(end_eff_y - valve_center_y, end_eff_x - valve_center_x)
        
        # REMOVED: 三点共线约束 (机体COG - 末端执行器 - 阀门中心)
        # 之前的角度差异约束已被移除，允许更灵活的插入配置
        # angle_diff = abs(self._normalize_angle(body_angle - end_eff_angle))
        # angle_diff = min(angle_diff, 2*math.pi - angle_diff)  # 取较小角度
        # max_angle_deviation = math.pi/6  # 30°最大偏差
        # radial_alignment_constraint = max_angle_deviation - angle_diff
        # constraints.append(radial_alignment_constraint)
        
        # NEW CONSTRAINT: Valve center to body center distance must be greater than 
        # end-effector center to valve center distance
        # This ensures the body stays outside the end-effector working area
        valve_body_distance_constraint = body_valve_distance - end_eff_valve_distance - 0.02  # 2cm buffer
        constraints.append(valve_body_distance_constraint)
        
        # End-effector must be within valve radius to grasp
        end_eff_reach_constraint = self.valve_radius - end_eff_valve_distance
        constraints.append(end_eff_reach_constraint)
        
        # Body minimum safety distance constraint
        # CRITICAL FIX: Consider end-effector length (0.246m) in safety calculation
        end_effector_length = 0.246  # Distance from UAV center to end-effector
        min_body_distance = self.valve_radius + end_effector_length + 0.01  # Additional 1cm safety margin
        body_safety_constraint = body_valve_distance - min_body_distance
        constraints.append(body_safety_constraint)
        
        return constraints
    
    def _normalize_angle(self, angle):
        """Normalize angle to [0, 2π]"""
        while angle < 0:
            angle += 2*math.pi
        while angle >= 2*math.pi:
            angle -= 2*math.pi
        return angle
    
    def optimize_insertion_position(self, target_gap='auto'):
        """
        Optimize insertion position using constrained optimization
        
        Args:
            target_gap: 'beam0_beam1', 'beam1_beam2', or 'auto'
            
        Returns:
            dict: Optimization result
        """
        if self.current_uav_pos is None or self.valve_pos is None:
            rospy.logerr("Missing UAV or valve position data")
            return None
        
        rospy.loginfo("Starting constrained optimization...")
        rospy.loginfo(f"Current UAV: [{self.current_uav_pos[0]:.3f}, {self.current_uav_pos[1]:.3f}]")
        rospy.loginfo(f"Valve center: [{self.valve_pos[0]:.3f}, {self.valve_pos[1]:.3f}]")
        
        # Initial guess: use geometrically calculated optimal position
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        
        # Calculate a good initial guess based on geometry
        # Place end-effector at valve edge, body extended back
        if target_gap == 'beam0_beam1' or target_gap == 'auto':
            # Place end-effector closer to beam1 (at 90° instead of 60°)
            target_angle = 90 * math.pi / 180  # 90° (closer to beam1 at 120°)
        else:  # beam1_beam2
            # Place end-effector closer to beam2 (at 190° instead of 180°)
            target_angle = 190 * math.pi / 180  # 190° (closer to beam2 at 240°)
        
        # Convert to world coordinates
        global_angle = target_angle + self.valve_yaw
        
        # End-effector position on valve circumference
        end_eff_x = valve_center_x + self.valve_radius * math.cos(global_angle)
        end_eff_y = valve_center_y + self.valve_radius * math.sin(global_angle)
        
        # Body position: point toward valve center
        body_yaw = math.atan2(valve_center_y - end_eff_y, valve_center_x - end_eff_x)
        body_x = end_eff_x - self.end_effector_length * math.cos(body_yaw)
        body_y = end_eff_y - self.end_effector_length * math.sin(body_yaw)
        
        x0 = [body_x, body_y, body_yaw]
        
        rospy.loginfo(f"Initial guess: body[{body_x:.3f}, {body_y:.3f}], yaw={body_yaw:.3f}")
        rospy.loginfo(f"Target end-effector: [{end_eff_x:.3f}, {end_eff_y:.3f}]")
        
        # Bounds for optimization variables
        bounds = [
            (self.current_uav_pos[0] - 1.0, self.current_uav_pos[0] + 1.0),  # body_x
            (self.current_uav_pos[1] - 1.0, self.current_uav_pos[1] + 1.0),  # body_y
            (self.current_uav_yaw - self.max_yaw_change, 
             self.current_uav_yaw + self.max_yaw_change)  # body_yaw
        ]
        
        # Constraint function for scipy
        def constraint_func(x):
            return self._constraints(x)
        
        constraint_dict = {
            'type': 'ineq',
            'fun': constraint_func
        }
        
        # Optimize
        try:
            result = minimize(
                fun=lambda x: self._objective_function(x, self.current_uav_pos, self.current_uav_yaw),
                x0=x0,
                method='SLSQP',
                bounds=bounds,
                constraints=constraint_dict,
                options={'maxiter': 100, 'ftol': 1e-6}
            )
            
            if result.success:
                optimal_body_x, optimal_body_y, optimal_body_yaw = result.x
                
                # Calculate end-effector position
                optimal_end_eff_x = optimal_body_x + self.end_effector_length * math.cos(optimal_body_yaw)
                optimal_end_eff_y = optimal_body_y + self.end_effector_length * math.sin(optimal_body_yaw)
                
                # Calculate which gap we're in
                end_eff_angle = math.atan2(optimal_end_eff_y - self.valve_pos[1], 
                                          optimal_end_eff_x - self.valve_pos[0])
                normalized_angle = self._normalize_angle(end_eff_angle - self.valve_yaw)
                
                if normalized_angle <= 2*math.pi/3:
                    selected_gap = 'beam0_beam1'
                elif normalized_angle <= 4*math.pi/3:
                    selected_gap = 'beam1_beam2'
                else:
                    selected_gap = 'beam2_beam0'
                
                rospy.loginfo("=== Radial Configuration Optimization Results ===")
                rospy.loginfo(f"Optimization successful: {result.success}")
                rospy.loginfo(f"Objective value: {result.fun:.4f}")
                rospy.loginfo(f"Selected gap: {selected_gap}")
                
                # Calculate radial configuration geometric relationships
                valve_center_x, valve_center_y, _ = self.valve_pos
                
                # Body to valve center distance and angle
                body_valve_distance = math.sqrt((optimal_body_x - valve_center_x)**2 + 
                                               (optimal_body_y - valve_center_y)**2)
                body_angle = math.atan2(optimal_body_y - valve_center_y, optimal_body_x - valve_center_x)
                
                # End-effector to valve center distance and angle
                end_eff_valve_distance = math.sqrt((optimal_end_eff_x - valve_center_x)**2 + 
                                                  (optimal_end_eff_y - valve_center_y)**2)
                end_eff_angle = math.atan2(optimal_end_eff_y - valve_center_y, optimal_end_eff_x - valve_center_x)
                
                # Radial alignment quality
                angle_diff = abs(self._normalize_angle(body_angle - end_eff_angle))
                angle_diff = min(angle_diff, 2*math.pi - angle_diff)
                
                rospy.loginfo("=== Radial Configuration Geometric Relationships ===")
                rospy.loginfo(f"Valve center: [{valve_center_x:.3f}, {valve_center_y:.3f}]")
                rospy.loginfo(f"Body position: [{optimal_body_x:.3f}, {optimal_body_y:.3f}] (distance to valve center: {body_valve_distance:.3f}m)")
                rospy.loginfo(f"End-effector position: [{optimal_end_eff_x:.3f}, {optimal_end_eff_y:.3f}] (distance to valve center: {end_eff_valve_distance:.3f}m)")
                rospy.loginfo(f"Body angle: {body_angle*180/math.pi:.1f}°")
                rospy.loginfo(f"End-effector angle: {end_eff_angle*180/math.pi:.1f}°")
                rospy.loginfo(f"Radial alignment deviation: {angle_diff*180/math.pi:.1f}°")
                rospy.loginfo(f"Radial configuration quality: {'Excellent' if angle_diff < 0.1 else 'Good' if angle_diff < 0.2 else 'Needs improvement'}")
                
                # Verify ideal radial expansion configuration
                theoretical_body_distance = end_eff_valve_distance + self.end_effector_length
                distance_accuracy = abs(body_valve_distance - theoretical_body_distance)
                
                rospy.loginfo("=== Radial Expansion Configuration Verification ===")
                rospy.loginfo(f"Theoretical body distance: {theoretical_body_distance:.3f}m")
                rospy.loginfo(f"Actual body distance: {body_valve_distance:.3f}m")
                rospy.loginfo(f"Distance accuracy: {distance_accuracy:.3f}m")
                rospy.loginfo(f"Configuration type: {'Ideal radial expansion' if distance_accuracy < 0.05 and angle_diff < 0.1 else 'Constrained optimization result'}")
                
                rospy.loginfo(f"Optimal body yaw: {optimal_body_yaw:.3f} rad ({optimal_body_yaw*180/math.pi:.1f}°)")
                rospy.loginfo(f"End-effector angle in valve frame: {normalized_angle*180/math.pi:.1f}°")
                
                return {
                    'success': True,
                    'body_position': (optimal_body_x, optimal_body_y, self.current_uav_pos[2]),
                    'body_yaw': optimal_body_yaw,
                    'end_effector_position': (optimal_end_eff_x, optimal_end_eff_y, self.current_uav_pos[2]),
                    'selected_gap': selected_gap,
                    'objective_value': result.fun,
                    'constraints_satisfied': all(c >= -1e-6 for c in self._constraints(result.x))
                }
            else:
                rospy.logerr(f"Optimization failed: {result.message}")
                return {'success': False, 'message': result.message}
                
        except Exception as e:
            rospy.logerr(f"Optimization error: {e}")
            return {'success': False, 'message': str(e)}

    def optimize_dual_insertion_position(self):
        """
        Optimize dual end-effector simultaneous insertion position using constrained optimization
        Both left and right end-effectors insert simultaneously
        
        Returns:
            dict: Dual insertion optimization result
        """
        if self.current_uav_pos is None or self.valve_pos is None:
            rospy.logerr("Missing UAV or valve position data")
            return None
        
        rospy.loginfo("Starting dual end-effector constrained optimization...")
        rospy.loginfo(f"Current UAV: [{self.current_uav_pos[0]:.3f}, {self.current_uav_pos[1]:.3f}]")
        rospy.loginfo(f"Valve center: [{self.valve_pos[0]:.3f}, {self.valve_pos[1]:.3f}]")
        
        # Calculate geometrically optimal positions for both end-effectors
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        
        # Left end-effector (fang1): beam0_beam1 gap (closer to beam1 at 120°)
        left_target_angle = 90 * math.pi / 180  # 90° (closer to beam1 at 120°)
        left_global_angle = left_target_angle + self.valve_yaw
        
        # Right end-effector (fang2): beam1_beam2 gap (closer to beam2 at 240°)
        right_target_angle = 190 * math.pi / 180  # 190° (closer to beam2 at 240°)
        right_global_angle = right_target_angle + self.valve_yaw
        
        # Calculate end-effector positions on valve circumference
        left_end_eff_x = valve_center_x + self.valve_radius * math.cos(left_global_angle)
        left_end_eff_y = valve_center_y + self.valve_radius * math.sin(left_global_angle)
        
        right_end_eff_x = valve_center_x + self.valve_radius * math.cos(right_global_angle)
        right_end_eff_y = valve_center_y + self.valve_radius * math.sin(right_global_angle)
        
        # Calculate body position based on dual end-effector configuration
        # Body is positioned to balance between both end-effectors
        center_end_eff_x = (left_end_eff_x + right_end_eff_x) / 2
        center_end_eff_y = (left_end_eff_y + right_end_eff_y) / 2
        
        # Body yaw points toward valve center
        body_yaw = math.atan2(valve_center_y - center_end_eff_y, valve_center_x - center_end_eff_x)
        
        # Body position: extended back from center of end-effectors
        body_x = center_end_eff_x - self.end_effector_length * math.cos(body_yaw)
        body_y = center_end_eff_y - self.end_effector_length * math.sin(body_yaw)
        
        x0 = [body_x, body_y, body_yaw]
        
        rospy.loginfo(f"Initial guess: body[{body_x:.3f}, {body_y:.3f}], yaw={body_yaw:.3f}")
        rospy.loginfo(f"Left end-effector: [{left_end_eff_x:.3f}, {left_end_eff_y:.3f}]")
        rospy.loginfo(f"Right end-effector: [{right_end_eff_x:.3f}, {right_end_eff_y:.3f}]")
        
        # Bounds for optimization variables
        bounds = [
            (self.current_uav_pos[0] - 1.0, self.current_uav_pos[0] + 1.0),  # body_x
            (self.current_uav_pos[1] - 1.0, self.current_uav_pos[1] + 1.0),  # body_y
            (self.current_uav_yaw - self.max_yaw_change, 
             self.current_uav_yaw + self.max_yaw_change)  # body_yaw
        ]
        
        # Constraint function for dual end-effector
        def dual_constraint_func(x):
            return self._dual_constraints(x)
        
        constraint_dict = {
            'type': 'ineq',
            'fun': dual_constraint_func
        }
        
        # Optimize
        try:
            result = minimize(
                fun=lambda x: self._dual_objective_function(x, self.current_uav_pos, self.current_uav_yaw),
                x0=x0,
                method='SLSQP',
                bounds=bounds,
                constraints=constraint_dict,
                options={'maxiter': 100, 'ftol': 1e-6}
            )
            
            if result.success:
                optimal_body_x, optimal_body_y, optimal_body_yaw = result.x
                
                # Calculate dual fang positions using URDF geometry
                (left_fang_x, left_fang_y, left_fang_z), (right_fang_x, right_fang_y, right_fang_z) = \
                    self._calculate_fang_positions(optimal_body_x, optimal_body_y, optimal_body_yaw)
                
                # Calculate inter-fang distance (should be fixed)
                inter_fang_distance = math.sqrt((left_fang_x - right_fang_x)**2 + 
                                               (left_fang_y - right_fang_y)**2)
                
                # Calculate angular separation
                valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
                left_fang_angle = math.atan2(left_fang_y - valve_center_y, left_fang_x - valve_center_x)
                right_fang_angle = math.atan2(right_fang_y - valve_center_y, right_fang_x - valve_center_x)
                
                angular_separation = abs(self._normalize_angle(left_fang_angle - right_fang_angle))
                angular_separation = min(angular_separation, 2*math.pi - angular_separation)
                
                rospy.loginfo("=== Dual End-Effector Optimization Results ===")
                rospy.loginfo(f"Optimization successful: {result.success}")
                rospy.loginfo(f"Objective value: {result.fun:.4f}")
                rospy.loginfo(f"Body position: [{optimal_body_x:.3f}, {optimal_body_y:.3f}]")
                rospy.loginfo(f"Body yaw: {optimal_body_yaw:.3f} rad ({optimal_body_yaw*180/math.pi:.1f}°)")
                rospy.loginfo(f"Left fang: [{left_fang_x:.3f}, {left_fang_y:.3f}]")
                rospy.loginfo(f"Right fang: [{right_fang_x:.3f}, {right_fang_y:.3f}]")
                rospy.loginfo(f"Inter-fang distance: {inter_fang_distance:.5f}m (Fixed: {self.fixed_inter_fang_distance:.5f}m)")
                rospy.loginfo(f"Angular separation: {angular_separation:.3f} rad ({angular_separation*180/math.pi:.1f}°)")
                
                # Evaluate constraint satisfaction
                constraint_values = self._dual_constraints(result.x)
                constraints_satisfied = all(c >= -1e-6 for c in constraint_values)
                
                rospy.loginfo("=== Constraint Satisfaction Status ===")
                rospy.loginfo(f"All constraints satisfied: {constraints_satisfied}")
                rospy.loginfo(f"Number of constraints: {len(constraint_values)}")
                
                # Detailed constraint analysis
                constraint_names = [
                    "Left fang at valve radius",
                    "Right fang at valve radius", 
                    "Left fang in beam0_beam1 gap",
                    "Right fang in beam1_beam2 gap",
                    "Body safety distance",
                    "Left fang beam clearance",
                    "Right fang beam clearance",
                    "Workspace constraint",
                    "Inter-fang distance constraint"
                ]
                
                for i, (name, value) in enumerate(zip(constraint_names, constraint_values)):
                    status = "✓" if value >= -1e-6 else "✗"
                    rospy.loginfo(f"  {status} {name}: {value:.4f}")
                
                return {
                    'success': True,
                    'body_position': (optimal_body_x, optimal_body_y, self.current_uav_pos[2]),
                    'body_yaw': optimal_body_yaw,
                    'left_end_effector_position': (left_fang_x, left_fang_y, left_fang_z),
                    'right_end_effector_position': (right_fang_x, right_fang_y, right_fang_z),
                    'inter_fang_distance': inter_fang_distance,
                    'angular_separation': angular_separation,
                    'insertion_type': 'dual_simultaneous',
                    'objective_value': result.fun,
                    'constraints_satisfied': constraints_satisfied,
                    'constraint_values': constraint_values
                }
            else:
                rospy.logerr(f"Dual optimization failed: {result.message}")
                return {'success': False, 'message': result.message}
                
        except Exception as e:
            rospy.logerr(f"Dual optimization error: {e}")
            return {'success': False, 'message': str(e)}

    def _dual_objective_function(self, x, current_pos, current_yaw):
        """
        Dual end-effector optimization objective function (corrected for fixed fang geometry)
        
        Args:
            x: [body_x, body_y, body_yaw] - optimization variables
            current_pos: Current UAV position
            current_yaw: Current UAV yaw
            
        Returns:
            float: Objective value (to minimize)
        """
        body_x, body_y, body_yaw = x
        valve_center_x, valve_center_y, _ = self.valve_pos
        
        # Calculate actual fang positions based on URDF geometry
        (left_fang_x, left_fang_y, left_fang_z), (right_fang_x, right_fang_y, right_fang_z) = \
            self._calculate_fang_positions(body_x, body_y, body_yaw)
        
        # Primary objective: optimize both fangs to valve radius
        left_valve_distance = math.sqrt((left_fang_x - valve_center_x)**2 + 
                                       (left_fang_y - valve_center_y)**2)
        right_valve_distance = math.sqrt((right_fang_x - valve_center_x)**2 + 
                                        (right_fang_y - valve_center_y)**2)
        
        # Ideal distance: close to valve radius
        ideal_distance = self.valve_radius - 0.01  # 1cm gap
        left_distance_error = abs(left_valve_distance - ideal_distance)
        right_distance_error = abs(right_valve_distance - ideal_distance)
        
        # Body safety distance
        body_valve_distance = math.sqrt((body_x - valve_center_x)**2 + 
                                       (body_y - valve_center_y)**2)
        # CRITICAL FIX: Consider end-effector length (0.246m) in safety calculation
        end_effector_length = 0.246  # Distance from UAV center to end-effector
        min_body_distance = self.valve_radius + end_effector_length + 0.01  # Additional 1cm safety margin
        body_safety_error = max(0, min_body_distance - body_valve_distance)
        
        # Movement minimization
        approach_distance = math.sqrt((body_x - current_pos[0])**2 + 
                                     (body_y - current_pos[1])**2)
        yaw_change = abs(self._normalize_angle(body_yaw - current_yaw))
        
        # Symmetry objective: both fangs should be equidistant from valve center
        symmetry_error = abs(left_valve_distance - right_valve_distance)
        
        # Calculate angular separation between fangs (relative to valve center)
        left_fang_angle = math.atan2(left_fang_y - valve_center_y, left_fang_x - valve_center_x)
        right_fang_angle = math.atan2(right_fang_y - valve_center_y, right_fang_x - valve_center_x)
        
        angular_separation = abs(self._normalize_angle(left_fang_angle - right_fang_angle))
        angular_separation = min(angular_separation, 2*math.pi - angular_separation)
        
        # Optimal angular separation: around 120° (2π/3) for effective valve gripping
        optimal_angular_separation = 2*math.pi/3  # 120°
        angular_error = abs(angular_separation - optimal_angular_separation)
        
        # Comprehensive objective function (removed inter-fang distance error since it's fixed)
        objective = (5.0 * left_distance_error +      # Left fang distance
                    5.0 * right_distance_error +      # Right fang distance
                    3.0 * body_safety_error +         # Body safety
                    2.0 * symmetry_error +            # Dual symmetry
                    1.5 * angular_error +             # Angular separation optimization
                    1.0 * approach_distance +         # Movement minimization
                    1.0 * yaw_change)                 # Yaw change minimization
        
        return objective

    def _dual_constraints(self, x):
        """
        Dual end-effector optimization constraints (corrected for fixed fang geometry)
        
        Args:
            x: [body_x, body_y, body_yaw] - optimization variables
            
        Returns:
            list: Constraint violations (should be >= 0)
        """
        body_x, body_y, body_yaw = x
        constraints = []
        
        # Calculate actual fang positions based on URDF geometry
        (left_fang_x, left_fang_y, left_fang_z), (right_fang_x, right_fang_y, right_fang_z) = \
            self._calculate_fang_positions(body_x, body_y, body_yaw)
        
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        
        # Constraint 1: Left fang should be at valve radius
        left_valve_distance = math.sqrt((left_fang_x - valve_center_x)**2 + 
                                       (left_fang_y - valve_center_y)**2)
        left_radius_constraint = self.safety_margin - abs(left_valve_distance - self.valve_radius)
        constraints.append(left_radius_constraint)
        
        # Constraint 2: Right fang should be at valve radius
        right_valve_distance = math.sqrt((right_fang_x - valve_center_x)**2 + 
                                        (right_fang_y - valve_center_y)**2)
        right_radius_constraint = self.safety_margin - abs(right_valve_distance - self.valve_radius)
        constraints.append(right_radius_constraint)
        
        # Constraint 3: Left fang should be in beam0_beam1 gap
        left_angle = math.atan2(left_fang_y - valve_center_y, left_fang_x - valve_center_x)
        left_normalized = self._normalize_angle(left_angle - self.valve_yaw)
        left_in_gap = (left_normalized >= 0 and left_normalized <= 2*math.pi/3)
        constraints.append(0.1 if left_in_gap else -1.0)
        
        # Constraint 4: Right fang should be in beam1_beam2 gap
        right_angle = math.atan2(right_fang_y - valve_center_y, right_fang_x - valve_center_x)
        right_normalized = self._normalize_angle(right_angle - self.valve_yaw)
        right_in_gap = (right_normalized >= 2*math.pi/3 and right_normalized <= 4*math.pi/3)
        constraints.append(0.1 if right_in_gap else -1.0)
        
        # Constraint 5: Body safety distance
        body_valve_distance = math.sqrt((body_x - valve_center_x)**2 + 
                                       (body_y - valve_center_y)**2)
        # CRITICAL FIX: Consider end-effector length (0.246m) in safety calculation
        end_effector_length = 0.246  # Distance from UAV center to end-effector
        min_body_distance = self.valve_radius + end_effector_length + 0.01  # Additional 1cm safety margin
        body_safety_constraint = body_valve_distance - min_body_distance
        constraints.append(body_safety_constraint)
        
        # Constraint 6: Minimum clearance from beams for both fangs
        beam_angles = [0, 2*math.pi/3, 4*math.pi/3]  # 0°, 120°, 240°
        
        # Left fang beam clearance
        left_clearances = []
        for beam_angle in beam_angles:
            clearance = abs(self._normalize_angle(left_normalized - beam_angle))
            clearance = min(clearance, 2*math.pi - clearance)
            left_clearances.append(clearance)
        
        min_left_clearance = min(left_clearances)
        left_clearance_constraint = min_left_clearance - self.beam_clearance
        constraints.append(left_clearance_constraint)
        
        # Right fang beam clearance
        right_clearances = []
        for beam_angle in beam_angles:
            clearance = abs(self._normalize_angle(right_normalized - beam_angle))
            clearance = min(clearance, 2*math.pi - clearance)
            right_clearances.append(clearance)
        
        min_right_clearance = min(right_clearances)
        right_clearance_constraint = min_right_clearance - self.beam_clearance
        constraints.append(right_clearance_constraint)
        
        # Constraint 7: Workspace boundary
        workspace_radius = 1.0  # meters
        workspace_constraint = workspace_radius - math.sqrt(body_x**2 + body_y**2)
        constraints.append(workspace_constraint)
        
        # Constraint 8: Fixed inter-fang distance validation (not for optimization)
        # This verifies the URDF geometry is correctly maintained
        expected_inter_fang_distance = 0.16876  # From URDF analysis
        actual_inter_fang_distance = math.sqrt((left_fang_x - right_fang_x)**2 + 
                                              (left_fang_y - right_fang_y)**2)
        distance_tolerance = 0.001  # 1mm tolerance for numerical precision
        inter_fang_constraint = distance_tolerance - abs(actual_inter_fang_distance - expected_inter_fang_distance)
        constraints.append(inter_fang_constraint)
        
        return constraints
        # Both end-effectors should be roughly equidistant from valve center
        distance_balance = abs(left_valve_distance - right_valve_distance)
        max_distance_imbalance = 0.02  # 2cm maximum imbalance
        balance_constraint = max_distance_imbalance - distance_balance
        constraints.append(balance_constraint)
        
        # Constraint 8: Inter-fang distance constraint
        # Distance between two fangs should be within acceptable range
        inter_fang_distance = math.sqrt((left_end_eff_x - right_end_eff_x)**2 + 
                                       (left_end_eff_y - right_end_eff_y)**2)
        
        # Physical constraints based on actual fang configuration
        min_inter_fang_distance = 0.05  # 5cm minimum separation (avoid collision)
        max_inter_fang_distance = 0.35  # 35cm maximum separation (maintain coordination)
        
        # Two-sided constraint: distance should be within [min, max] range
        min_distance_constraint = inter_fang_distance - min_inter_fang_distance
        max_distance_constraint = max_inter_fang_distance - inter_fang_distance
        
        constraints.append(min_distance_constraint)  # >= min_distance
        constraints.append(max_distance_constraint)  # <= max_distance
        
        # Constraint 9: Fang angular separation constraint
        # Angular separation between fangs should be reasonable for valve operation
        left_fang_angle = math.atan2(left_end_eff_y - valve_center_y, left_end_eff_x - valve_center_x)
        right_fang_angle = math.atan2(right_end_eff_y - valve_center_y, right_end_eff_x - valve_center_x)
        
        # Calculate angular separation (always positive)
        angular_separation = abs(self._normalize_angle(left_fang_angle - right_fang_angle))
        angular_separation = min(angular_separation, 2*math.pi - angular_separation)
        
        # Optimal angular separation: around 90° to 180° for effective valve rotation
        min_angular_separation = math.pi/3      # 60° minimum
        max_angular_separation = 5*math.pi/6    # 150° maximum
        
        min_angular_constraint = angular_separation - min_angular_separation
        max_angular_constraint = max_angular_separation - angular_separation
        
        constraints.append(min_angular_constraint)  # >= 60°
        constraints.append(max_angular_constraint)  # <= 150°
        
        return constraints

    def _calculate_fang_positions(self, body_x, body_y, body_yaw, body_z=None):
        """
        Calculate world positions of both fangs based on URDF geometry
        
        Args:
            body_x: Body X position
            body_y: Body Y position  
            body_yaw: Body yaw angle
            body_z: Body Z position (optional, defaults to valve height)
            
        Returns:
            tuple: (left_fang_pos, right_fang_pos) each as (x, y, z)
        """
        # Use valve height if body_z not provided
        if body_z is None:
            body_z = self.valve_pos[2] if self.valve_pos else 0.5
        
        # Transform from body frame to world frame
        cos_yaw = math.cos(body_yaw)
        sin_yaw = math.sin(body_yaw)
        
        # Fang link position in world frame
        fang_link_x = body_x + self.fang_to_base_x * cos_yaw - self.fang_to_base_y * sin_yaw
        fang_link_y = body_y + self.fang_to_base_x * sin_yaw + self.fang_to_base_y * cos_yaw
        fang_link_z = body_z + self.fang_to_base_z
        
        # Left fang position in world frame
        left_fang_x = fang_link_x + self.fang_collision_x * cos_yaw - self.left_fang_y * sin_yaw
        left_fang_y = fang_link_y + self.fang_collision_x * sin_yaw + self.left_fang_y * cos_yaw
        left_fang_z = fang_link_z + self.fang_collision_z
        
        # Right fang position in world frame
        right_fang_x = fang_link_x + self.fang_collision_x * cos_yaw - self.right_fang_y * sin_yaw
        right_fang_y = fang_link_y + self.fang_collision_x * sin_yaw + self.right_fang_y * cos_yaw
        right_fang_z = fang_link_z + self.fang_collision_z
        
        return ((left_fang_x, left_fang_y, left_fang_z), 
                (right_fang_x, right_fang_y, right_fang_z))

def test_constrained_optimizer():
    """Test the constrained optimizer"""
    rospy.init_node('constrained_optimizer_test')
    
    optimizer = ConstrainedInsertionOptimizer(
        valve_radius=0.1225,
        valve_beam_width=0.0185,
        end_effector_length=0.246,
        safety_margin=0.005,
        module_id=1
    )
    
    # Set test data
    optimizer.current_uav_pos = (2.8, -0.1, 0.68)
    optimizer.current_uav_yaw = 0.1
    optimizer.valve_pos = (3.0, 0.0, 0.57)
    optimizer.valve_yaw = 0.0
    
    # Test 1: Single end-effector optimization
    rospy.loginfo("=== Test 1: Single End-Effector Optimization ===")
    result = optimizer.optimize_insertion_position()
    
    if result and result['success']:
        rospy.loginfo("✓ Single end-effector optimization successful!")
        rospy.loginfo(f"Selected gap: {result['selected_gap']}")
        rospy.loginfo(f"Constraints satisfied: {result['constraints_satisfied']}")
    else:
        rospy.logerr("✗ Single end-effector optimization failed!")
    
    # Test 2: Dual end-effector optimization
    rospy.loginfo("\n=== Test 2: Dual End-Effector Optimization ===")
    dual_result = optimizer.optimize_dual_insertion_position()
    
    if dual_result and dual_result['success']:
        rospy.loginfo("✓ Dual end-effector optimization successful!")
        rospy.loginfo(f"Insertion type: {dual_result['insertion_type']}")
        rospy.loginfo(f"Constraints satisfied: {dual_result['constraints_satisfied']}")
        rospy.loginfo(f"Body position: {dual_result['body_position']}")
        rospy.loginfo(f"Left end-effector: {dual_result['left_end_effector_position']}")
        rospy.loginfo(f"Right end-effector: {dual_result['right_end_effector_position']}")
    else:
        rospy.logerr("✗ Dual end-effector optimization failed!")
    
    # Test 3: Comparison
    rospy.loginfo("\n=== Test 3: Comparison ===")
    if result and result['success'] and dual_result and dual_result['success']:
        rospy.loginfo(f"Single objective: {result['objective_value']:.4f}")
        rospy.loginfo(f"Dual objective: {dual_result['objective_value']:.4f}")
        rospy.loginfo(f"Recommendation: {'Dual' if dual_result['objective_value'] < result['objective_value'] else 'Single'} end-effector approach")
    
    rospy.loginfo("=== Constrained Optimizer Test Complete ===")
    rospy.loginfo("Both single and dual end-effector optimization modes are now available!")


if __name__ == '__main__':
    test_constrained_optimizer()
