#!/usr/bin/env python
"""Dual-Fang Insertion Optimizer for Valve Rotation Task."""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from demo_common import normalize_angle, normalize_angle_diff
import math
import rospy


class InsertionOptimizer:
    """Dual-fang insertion optimizer for clockwise valve rotation."""
    
    def __init__(self, valve_radius=0.10, valve_beam_width=0.035, safety_margin=0.002, 
                 module_id=1, simulation=True):
        """
        Initialize dual-fang insertion optimizer.
        
        Args:
            valve_radius: Inner rim radius (meters), default 100mm
            valve_beam_width: Minimum spoke gap width (meters), default 35mm
            safety_margin: Safety margin for insertion (meters)
            module_id: UAV module ID
            simulation: Simulation mode flag
        """
        # Valve geometry
        self.valve_inner_radius = valve_radius        # 100mm inner rim
        self.valve_outer_radius = 0.12                # 120mm outer rim
        self.hub_radius = 0.0375                      # 37.5mm hub
        self.wheel_thickness = 0.025
        
        # Spoke configuration
        self.spoke_count = 3
        self.spoke_hub_width = 0.035
        self.spoke_rim_width = 0.060
        self.min_spoke_gap = valve_beam_width
        
        # Legacy compatibility
        self.valve_radius = valve_radius
        self.valve_beam_width = valve_beam_width
        self.safety_margin = safety_margin
        self.module_id = module_id
        self.simulation = simulation
        
        # Dual-fang physical parameters
        self.dual_fang_center_offset = 0.25346  # 253.46mm from UAV center
        self.claw_separation = 0.150            # 150mm between claws
        self.half_claw_separation = self.claw_separation / 2
        self.end_effector_offset_z = 0.0221140
        
        # Spoke locking for session consistency
        self.locked_spoke_index = None
        self.locked_spoke_angle = None
        self.insertion_session_active = False
        self.session_start_time = None
        self.insertion_safety_margin_z = 0.0673 + 0.0221140
        
        # Safety constraints
        self.min_end_effector_to_valve_center_distance = 0.020
        self.min_uav_to_valve_outer_rim_distance = 0.180
        self.min_uav_to_valve_center_distance = 0.200
        
        # Rotation direction
        self.rotation_direction = 1  # 1=clockwise, -1=counter-clockwise

        # Cached valve pose
        self._valve_position = None
        self._valve_yaw = None
        self._spoke_angles_world = None
        
        rospy.loginfo("InsertionOptimizer initialized")
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        return normalize_angle(angle)

    def update_valve_info(self, valve_pos, valve_yaw):
        """Cache valve pose information."""
        if valve_pos is None:
            return
        try:
            self._valve_position = tuple(valve_pos[:3])
        except TypeError:
            self._valve_position = tuple(valve_pos)
        self._valve_yaw = float(valve_yaw) if valve_yaw is not None else 0.0
        offsets = (0.0, math.pi * 2 / 3, math.pi * 4 / 3)
        self._spoke_angles_world = [self._normalize_angle(self._valve_yaw + offset) for offset in offsets]

    def get_spoke_angles(self):
        """Return world-frame spoke angles."""
        if self._spoke_angles_world is None:
            if self._valve_yaw is None:
                base_angles = (0.0, math.pi * 2 / 3, math.pi * 4 / 3)
                return [self._normalize_angle(angle) for angle in base_angles]
            offsets = (0.0, math.pi * 2 / 3, math.pi * 4 / 3)
            self._spoke_angles_world = [self._normalize_angle(self._valve_yaw + offset) for offset in offsets]
        return list(self._spoke_angles_world)

    def get_optimal_end_effector_distance(self):
        """Get optimal end-effector distance from valve center."""
        return self.min_end_effector_to_valve_center_distance

    def calculate_insertion_uav_z(self, valve_z):
        """Calculate UAV Z position for insertion with safety margin."""
        return valve_z - self.end_effector_offset_z + self.insertion_safety_margin_z
    
    # =================== SPOKE LOCKING MECHANISM ===================
    
    def start_insertion_session(self):
        """Start a new insertion session with spoke locking."""
        self.insertion_session_active = True
        self.session_start_time = rospy.get_time()
        self.locked_spoke_index = None
        self.locked_spoke_angle = None
        
    def end_insertion_session(self):
        """End current insertion session and reset spoke lock."""
        self.insertion_session_active = False
        self.session_start_time = None
        self.locked_spoke_index = None
        self.locked_spoke_angle = None
        
    def lock_spoke_selection(self, spoke_index, spoke_angle):
        """Lock spoke selection for current session."""
        self.locked_spoke_index = spoke_index
        self.locked_spoke_angle = spoke_angle
        
    def get_locked_spoke(self):
        """Get currently locked spoke information."""
        if self.insertion_session_active and self.locked_spoke_index is not None:
            return self.locked_spoke_index, self.locked_spoke_angle
        return None, None
        
    def is_spoke_locked(self):
        """Check if spoke is currently locked."""
        return (self.insertion_session_active and 
                self.locked_spoke_index is not None and 
                self.locked_spoke_angle is not None)
        
    def check_session_timeout(self, timeout_seconds=300):
        """Check if insertion session has timed out."""
        if not self.insertion_session_active or self.session_start_time is None:
            return False
        session_duration = rospy.get_time() - self.session_start_time
        if session_duration > timeout_seconds:
            self.end_insertion_session()
            return True
        return False
    
    def set_rotation_direction(self, direction):
        """Set rotation direction (1=clockwise, -1=counter-clockwise)."""
        self.rotation_direction = direction
    
    def calculate_beam_gaps(self, valve_pos, valve_yaw):
        """Calculate all available beam gap positions."""
        beam_angles = {
            'beam0': valve_yaw + 0,
            'beam1': valve_yaw + math.pi * 2/3,
            'beam2': valve_yaw + math.pi * 4/3,
        }
        
        # Normalize angles
        for beam_name in beam_angles:
            while beam_angles[beam_name] >= 2*math.pi:
                beam_angles[beam_name] -= 2*math.pi
            while beam_angles[beam_name] < 0:
                beam_angles[beam_name] += 2*math.pi
        
        beam_gaps = {}
        
        # Gap 1: Between beam0 and beam1 (60°)
        gap1_angle = (beam_angles['beam0'] + beam_angles['beam1']) / 2
        beam_gaps['beam0_beam1'] = {
            'center_angle': gap1_angle,
            'beam1': 'beam0', 'beam2': 'beam1',
            'beam1_angle': beam_angles['beam0'],
            'beam2_angle': beam_angles['beam1']
        }
        
        # Gap 2: Between beam1 and beam2 (180°)
        gap2_angle = (beam_angles['beam1'] + beam_angles['beam2']) / 2
        beam_gaps['beam1_beam2'] = {
            'center_angle': gap2_angle,
            'beam1': 'beam1', 'beam2': 'beam2',
            'beam1_angle': beam_angles['beam1'],
            'beam2_angle': beam_angles['beam2']
        }
        
        # Gap 3: Between beam2 and beam0 (300°)
        gap3_angle = beam_angles['beam2'] + math.pi/3
        if gap3_angle >= 2*math.pi:
            gap3_angle -= 2*math.pi
        beam_gaps['beam0_beam2'] = {
            'center_angle': gap3_angle,
            'beam1': 'beam2', 'beam2': 'beam0',
            'beam1_angle': beam_angles['beam2'],
            'beam2_angle': beam_angles['beam0']
        }
        
        valve_center_x, valve_center_y = valve_pos[0], valve_pos[1]
        
        # Calculate optimal insertion radius
        required_radius = self.claw_separation / math.sqrt(3)
        min_safe_radius = self.hub_radius + 0.005
        max_safe_radius = self.valve_inner_radius - 0.005
        gap_radius = max(min_safe_radius, min(required_radius, max_safe_radius))
        
        for gap_name, gap_info in beam_gaps.items():
            gap_x = valve_center_x + gap_radius * math.cos(gap_info['center_angle'])
            gap_y = valve_center_y + gap_radius * math.sin(gap_info['center_angle'])
            gap_info['position'] = (gap_x, gap_y)
            gap_info['radius'] = gap_radius
        
        return beam_gaps
    
    def find_closest_beam_gap(self, uav_pos, valve_pos, valve_yaw):
        """Find the closest beam gap to the UAV."""
        beam_gaps = self.calculate_beam_gaps(valve_pos, valve_yaw)
        
        closest_gap = None
        min_distance = float('inf')
        
        for gap_name, gap_info in beam_gaps.items():
            gap_pos = gap_info['position']
            dx = gap_pos[0] - uav_pos[0]
            dy = gap_pos[1] - uav_pos[1]
            dz = valve_pos[2] - uav_pos[2]
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance < min_distance:
                min_distance = distance
                closest_gap = {
                    'gap_name': gap_name,
                    'gap_info': gap_info,
                    'distance': distance
                }
        
        if closest_gap is None:
            return None
        
        gap_info = closest_gap['gap_info']
        required_uav_z = self.calculate_insertion_uav_z(valve_pos[2])
        
        return {
            'insertion_mode': 'single_closest',
            'selected_gap': closest_gap['gap_name'],
            'gap_center_angle': gap_info['center_angle'],
            'target_position': gap_info['position'] + (required_uav_z,),
            'approach_distance': closest_gap['distance'],
            'beam_gap_radius': gap_info['radius'],
            'complexity_score': closest_gap['distance'],
        }
        
    def calculate_dual_fang_clockwise_strategy(self, uav_pos, valve_pos, valve_yaw):
        """Calculate optimal dual-fang insertion strategy for CLOCKWISE rotation."""
        if not self.insertion_session_active:
            self.start_insertion_session()
        
        if self.check_session_timeout():
            self.start_insertion_session()
        
        strategy_result = self.calculate_same_side_insertion_strategy(valve_pos, valve_yaw, uav_pos)
        
        if not strategy_result['feasible']:
            return None
        
        return {
            'strategy_type': 'dual_fang_clockwise_spoke_aligned',
            'left_claw_target': tuple(strategy_result['left_target']),
            'right_claw_target': tuple(strategy_result['right_target']),
            'optimal_uav_position': tuple(strategy_result['safe_uav_position']),
            'optimal_uav_yaw': strategy_result['safe_uav_yaw'],
            'dual_fang_center': tuple(strategy_result['safe_end_effector_center']),
            'feasible': strategy_result['feasible'],
            'spoke_aligned': True,
            'collision_safe': strategy_result['safety_adequate'],
            'separation_accurate': strategy_result['verification']['separation_accuracy'],
            'base_strategy': strategy_result
        }
    
    def _normalize_angle_diff(self, angle):
        """Normalize angle difference to [-pi, pi]."""
        return normalize_angle_diff(angle)
    
    def verify_dual_fang_geometry(self, uav_pos, uav_yaw):
        """Verify dual-fang geometry calculations."""
        dual_fang_center_x = uav_pos[0] + self.dual_fang_center_offset * math.cos(uav_yaw)
        dual_fang_center_y = uav_pos[1] + self.dual_fang_center_offset * math.sin(uav_yaw)
        dual_fang_center_z = uav_pos[2] + self.end_effector_offset_z
        
        left_claw_x = dual_fang_center_x + self.half_claw_separation * math.cos(uav_yaw + math.pi/2)
        left_claw_y = dual_fang_center_y + self.half_claw_separation * math.sin(uav_yaw + math.pi/2)
        right_claw_x = dual_fang_center_x - self.half_claw_separation * math.cos(uav_yaw + math.pi/2)
        right_claw_y = dual_fang_center_y - self.half_claw_separation * math.sin(uav_yaw + math.pi/2)
        
        actual_claw_separation = math.sqrt((left_claw_x - right_claw_x)**2 + (left_claw_y - right_claw_y)**2)
        dual_fang_distance = math.sqrt((dual_fang_center_x - uav_pos[0])**2 + (dual_fang_center_y - uav_pos[1])**2)
        
        geometry_ok = (abs(actual_claw_separation - self.claw_separation) < 0.001 and 
                      abs(dual_fang_distance - self.dual_fang_center_offset) < 0.001)
        
        return {
            'dual_fang_center': (dual_fang_center_x, dual_fang_center_y, dual_fang_center_z),
            'left_claw': (left_claw_x, left_claw_y),
            'right_claw': (right_claw_x, right_claw_y),
            'geometry_valid': geometry_ok,
            'claw_separation_error': abs(actual_claw_separation - self.claw_separation),
            'center_offset_error': abs(dual_fang_distance - self.dual_fang_center_offset)
        }
    
    def calculate_same_side_insertion_strategy(self, valve_pos, valve_yaw, uav_pos=None):
        """
        Spoke-aligned symmetric insertion strategy.
        
        UAV aligns with valve spoke, claws distributed symmetrically on both sides.
        """
        valve_center_x, valve_center_y = valve_pos[0], valve_pos[1]
        
        # Spoke angles in world coordinates
        spoke_angles_relative = [0, math.pi*2/3, math.pi*4/3]
        spoke_angles_world = [angle + valve_yaw for angle in spoke_angles_relative]
        
        # Spoke selection with locking mechanism
        locked_spoke_index, locked_spoke_angle = self.get_locked_spoke()
        
        if self.is_spoke_locked():
            optimal_spoke_index = locked_spoke_index
            optimal_spoke_angle = locked_spoke_angle
        elif uav_pos is not None:
            uav_angle_to_valve = math.atan2(uav_pos[1] - valve_center_y, uav_pos[0] - valve_center_x)
            
            # Find best spoke based on minimum rotation cost
            best_spoke_index = 0
            min_total_cost = float('inf')
            
            for i, spoke_world_angle in enumerate(spoke_angles_world):
                uav_rotation_needed = abs(self._normalize_angle_diff(spoke_world_angle - uav_angle_to_valve))
                insertion_distance = math.sqrt((uav_pos[0] - valve_center_x)**2 + (uav_pos[1] - valve_center_y)**2)
                total_cost = uav_rotation_needed * 2.0 + abs(insertion_distance - 0.34)
                
                if total_cost < min_total_cost:
                    min_total_cost = total_cost
                    best_spoke_index = i
            
            optimal_spoke_index = best_spoke_index
            optimal_spoke_angle = spoke_angles_world[optimal_spoke_index]
            
            if self.insertion_session_active:
                self.lock_spoke_selection(optimal_spoke_index, optimal_spoke_angle)
        else:
            # Fallback: use beam1 (120°) as default
            optimal_spoke_index = 1
            optimal_spoke_angle = valve_yaw + math.pi * 2/3
            if self.insertion_session_active:
                self.lock_spoke_selection(optimal_spoke_index, optimal_spoke_angle)
        
        # UAV yaw aligned with spoke, facing away from valve
        optimal_uav_yaw = self._normalize_angle(optimal_spoke_angle + math.pi)
        
        # Calculate end-effector distance constraints
        min_end_effector_distance = self.min_end_effector_to_valve_center_distance
        valve_inner_radius = 0.100
        half_claw_separation = 0.075
        max_end_effector_distance = math.sqrt(valve_inner_radius**2 - half_claw_separation**2)
        optimal_end_effector_distance = min_end_effector_distance
        
        # End-effector center position along spoke direction
        end_effector_center_x = valve_center_x + optimal_end_effector_distance * math.cos(optimal_spoke_angle)
        end_effector_center_y = valve_center_y + optimal_end_effector_distance * math.sin(optimal_spoke_angle)
        end_effector_center_z = valve_pos[2] + self.insertion_safety_margin_z
        
        # Claw positions perpendicular to spoke
        perpendicular_angle = optimal_spoke_angle + math.pi/2
        
        left_claw_target_x = end_effector_center_x + self.half_claw_separation * math.cos(perpendicular_angle)
        left_claw_target_y = end_effector_center_y + self.half_claw_separation * math.sin(perpendicular_angle)
        left_claw_target_z = valve_pos[2] + self.insertion_safety_margin_z
        
        right_claw_target_x = end_effector_center_x - self.half_claw_separation * math.cos(perpendicular_angle)
        right_claw_target_y = end_effector_center_y - self.half_claw_separation * math.sin(perpendicular_angle)
        right_claw_target_z = valve_pos[2] + self.insertion_safety_margin_z
        
        # Verify claw distances
        left_claw_distance = math.sqrt((left_claw_target_x - valve_center_x)**2 + 
                                       (left_claw_target_y - valve_center_y)**2)
        right_claw_distance = math.sqrt((right_claw_target_x - valve_center_x)**2 + 
                                        (right_claw_target_y - valve_center_y)**2)
        
        # Claw separation verification
        calculated_separation = math.sqrt((right_claw_target_x - left_claw_target_x)**2 + 
                                          (right_claw_target_y - left_claw_target_y)**2)
        
        # Check clearance from all spokes
        safety_margins = []
        for spoke_idx, spoke_world_angle in enumerate(spoke_angles_world):
            spoke_x = valve_center_x + self.valve_inner_radius * math.cos(spoke_world_angle)
            spoke_y = valve_center_y + self.valve_inner_radius * math.sin(spoke_world_angle)
            left_dist = math.sqrt((left_claw_target_x - spoke_x)**2 + (left_claw_target_y - spoke_y)**2)
            right_dist = math.sqrt((right_claw_target_x - spoke_x)**2 + (right_claw_target_y - spoke_y)**2)
            safety_margins.append(min(left_dist, right_dist))
        
        min_safety_margin = min(safety_margins)
        safety_adequate = min_safety_margin > 0.015  # 15mm minimum clearance
        
        # UAV positioning along spoke direction
        total_distance = optimal_end_effector_distance + self.dual_fang_center_offset
        uav_center_x = valve_center_x + total_distance * math.cos(optimal_spoke_angle)
        uav_center_y = valve_center_y + total_distance * math.sin(optimal_spoke_angle)
        uav_center_z = end_effector_center_z - self.end_effector_offset_z
        
        uav_to_valve_distance = math.sqrt((uav_center_x - valve_center_x)**2 + (uav_center_y - valve_center_y)**2)
        expected_distance = optimal_end_effector_distance + self.dual_fang_center_offset
        
        # Adjust if UAV too close to valve
        if uav_to_valve_distance < self.min_uav_to_valve_center_distance:
            required_ee_distance = self.min_uav_to_valve_center_distance - self.dual_fang_center_offset
            optimal_end_effector_distance = max(required_ee_distance, min_end_effector_distance)
            
            end_effector_center_x = valve_center_x + optimal_end_effector_distance * math.cos(optimal_spoke_angle)
            end_effector_center_y = valve_center_y + optimal_end_effector_distance * math.sin(optimal_spoke_angle)
            
            total_distance = optimal_end_effector_distance + self.dual_fang_center_offset
            uav_center_x = valve_center_x + total_distance * math.cos(optimal_spoke_angle)
            uav_center_y = valve_center_y + total_distance * math.sin(optimal_spoke_angle)
            uav_to_valve_distance = math.sqrt((uav_center_x - valve_center_x)**2 + (uav_center_y - valve_center_y)**2)
        
        # Final claw positions
        final_left_claw_x = end_effector_center_x + self.half_claw_separation * math.cos(perpendicular_angle)
        final_left_claw_y = end_effector_center_y + self.half_claw_separation * math.sin(perpendicular_angle)
        final_right_claw_x = end_effector_center_x - self.half_claw_separation * math.cos(perpendicular_angle)
        final_right_claw_y = end_effector_center_y - self.half_claw_separation * math.sin(perpendicular_angle)
        
        final_end_effector_to_valve_distance = math.sqrt(
            (end_effector_center_x - valve_center_x)**2 + 
            (end_effector_center_y - valve_center_y)**2
        )
        final_uav_to_valve_distance = math.sqrt(
            (uav_center_x - valve_center_x)**2 + 
            (uav_center_y - valve_center_y)**2
        )
        uav_to_outer_rim_distance = final_uav_to_valve_distance - self.valve_outer_radius
        
        # Safety result
        safety_result = {
            'uav_position': [uav_center_x, uav_center_y, uav_center_z],
            'uav_yaw': optimal_uav_yaw,
            'end_effector_center': [end_effector_center_x, end_effector_center_y, end_effector_center_z],
            'safety_check': {
                'safety_corrections_applied': final_uav_to_valve_distance != expected_distance,
                'direct_spoke_aligned': True,
                'natural_safety_margin': min_safety_margin,
                'positioning_error': abs(final_uav_to_valve_distance - expected_distance),
                'end_effector_to_valve_distance': final_end_effector_to_valve_distance,
                'uav_to_valve_distance': final_uav_to_valve_distance,
                'uav_to_outer_rim_distance': uav_to_outer_rim_distance,
                'constraint_violations': [],
                'violations': [],
                'all_constraints_satisfied': (final_end_effector_to_valve_distance >= self.min_end_effector_to_valve_center_distance and 
                                              final_uav_to_valve_distance >= self.min_uav_to_valve_center_distance)
            }
        }

        return {
            'strategy': 'perpendicular_claw_positioning',
            'description': 'UAV aligns with spoke, claws perpendicular to spoke',
            'feasible': safety_adequate and abs(calculated_separation - self.claw_separation) < 0.001,
            'left_target': [left_claw_target_x, left_claw_target_y, left_claw_target_z],
            'right_target': [right_claw_target_x, right_claw_target_y, right_claw_target_z],
            'safe_uav_position': safety_result['uav_position'],
            'safe_uav_yaw': safety_result['uav_yaw'],
            'safe_end_effector_center': safety_result['end_effector_center'],
            'safety_check': safety_result['safety_check'],
            'uav_position': [uav_center_x, uav_center_y, uav_center_z],
            'uav_yaw': optimal_uav_yaw,
            'insertion_radius': optimal_end_effector_distance,
            'safety_margin': min_safety_margin,
            'safety_adequate': safety_adequate,
            'spoke_alignment': {
                'optimal_spoke_index': optimal_spoke_index,
                'optimal_spoke_angle': optimal_spoke_angle,
                'perpendicular_angle': perpendicular_angle,
                'end_effector_distance': optimal_end_effector_distance,
                'min_end_effector_distance': min_end_effector_distance,
                'max_end_effector_distance': max_end_effector_distance,
                'spoke_angles_world': spoke_angles_world
            },
            'verification': {
                'calculated_separation': calculated_separation,
                'required_separation': self.claw_separation,
                'separation_accuracy': abs(calculated_separation - self.claw_separation) < 0.001,
                'left_claw_distance': left_claw_distance,
                'right_claw_distance': right_claw_distance,
                'distance_equality': abs(left_claw_distance - right_claw_distance) < 0.001
            }
        }

    def calculate_safe_uav_position_with_constraints(self, left_target, right_target, valve_pos):
        """Calculate UAV position with safety constraints."""
        valve_center_x, valve_center_y, valve_center_z = valve_pos[0], valve_pos[1], valve_pos[2]
        
        # Dual-fang center is midpoint between claws
        dual_fang_center_x = (left_target[0] + right_target[0]) / 2
        dual_fang_center_y = (left_target[1] + right_target[1]) / 2
        dual_fang_center_z = left_target[2]
        
        # UAV yaw perpendicular to claw separation line
        claw_separation_angle = math.atan2(right_target[1] - left_target[1], 
                                           right_target[0] - left_target[0])
        uav_yaw = self._normalize_angle(claw_separation_angle + math.pi/2)
        
        # Initial UAV position
        initial_uav_x = dual_fang_center_x - self.dual_fang_center_offset * math.cos(uav_yaw)
        initial_uav_y = dual_fang_center_y - self.dual_fang_center_offset * math.sin(uav_yaw)
        initial_uav_z = dual_fang_center_z - self.end_effector_offset_z
        
        # Safety constraint checks
        end_effector_to_valve_distance = math.sqrt(
            (dual_fang_center_x - valve_center_x)**2 + 
            (dual_fang_center_y - valve_center_y)**2
        )
        initial_uav_to_valve_distance = math.sqrt(
            (initial_uav_x - valve_center_x)**2 + 
            (initial_uav_y - valve_center_y)**2
        )
        initial_uav_to_outer_rim_distance = initial_uav_to_valve_distance - self.valve_outer_radius
        
        constraint_violations = []
        if end_effector_to_valve_distance < self.min_end_effector_to_valve_center_distance:
            constraint_violations.append("End-effector too close")
        if initial_uav_to_valve_distance < self.min_uav_to_valve_center_distance:
            constraint_violations.append("UAV too close to valve center")
        if initial_uav_to_outer_rim_distance < self.min_uav_to_valve_outer_rim_distance:
            constraint_violations.append("UAV too close to valve rim")
        
        # Calculate required UAV distance
        required_uav_distances = [
            self.min_uav_to_valve_center_distance,
            self.valve_outer_radius + self.min_uav_to_valve_outer_rim_distance,
            self.min_end_effector_to_valve_center_distance + self.dual_fang_center_offset
        ]
        required_uav_distance = max(required_uav_distances)
        
        current_uav_radial_distance = math.sqrt((initial_uav_x - valve_center_x)**2 + 
                                                (initial_uav_y - valve_center_y)**2)
        uav_direction_from_valve = math.atan2(initial_uav_y - valve_center_y, 
                                              initial_uav_x - valve_center_x)
        
        # Apply radial adjustment
        if current_uav_radial_distance < required_uav_distance:
            radial_adjustment = required_uav_distance - current_uav_radial_distance
            adjustment_type = "outward"
            final_uav_distance = required_uav_distance
        else:
            current_outer_rim_clearance = current_uav_radial_distance - self.valve_outer_radius
            if current_outer_rim_clearance < self.min_uav_to_valve_outer_rim_distance:
                radial_adjustment = self.min_uav_to_valve_outer_rim_distance - current_outer_rim_clearance
                adjustment_type = "outward"
                final_uav_distance = self.valve_outer_radius + self.min_uav_to_valve_outer_rim_distance
            else:
                radial_adjustment = 0
                adjustment_type = "none"
                final_uav_distance = current_uav_radial_distance
        
        # Safe UAV position
        safe_uav_x = valve_center_x + final_uav_distance * math.cos(uav_direction_from_valve)
        safe_uav_y = valve_center_y + final_uav_distance * math.sin(uav_direction_from_valve)
        safe_uav_z = initial_uav_z
        
        # Recalculate end-effector position
        safe_dual_fang_center_x = safe_uav_x + self.dual_fang_center_offset * math.cos(uav_yaw)
        safe_dual_fang_center_y = safe_uav_y + self.dual_fang_center_offset * math.sin(uav_yaw)
        safe_dual_fang_center_z = safe_uav_z + self.end_effector_offset_z
        
        safe_end_effector_to_valve_distance = math.sqrt(
            (safe_dual_fang_center_x - valve_center_x)**2 + 
            (safe_dual_fang_center_y - valve_center_y)**2
        )
        safe_uav_to_valve_distance = math.sqrt(
            (safe_uav_x - valve_center_x)**2 + 
            (safe_uav_y - valve_center_y)**2
        )
        safe_uav_to_outer_rim_distance = safe_uav_to_valve_distance - self.valve_outer_radius
        
        safety_check = {
            'constraints_satisfied': True,
            'violations': constraint_violations,
            'end_effector_to_valve_distance': safe_end_effector_to_valve_distance,
            'uav_to_valve_distance': safe_uav_to_valve_distance,
            'uav_to_outer_rim_distance': safe_uav_to_outer_rim_distance,
            'radial_optimization_applied': radial_adjustment > 0,
            'radial_adjustment': radial_adjustment,
            'adjustment_type': adjustment_type,
            'safety_corrections_applied': radial_adjustment > 0
        }
        
        return {
            'uav_position': (safe_uav_x, safe_uav_y, safe_uav_z),
            'uav_yaw': uav_yaw,
            'end_effector_center': (safe_dual_fang_center_x, safe_dual_fang_center_y, safe_dual_fang_center_z),
            'safety_check': safety_check,
            'feasible': True
        }
    
    def calculate_two_phase_safe_insertion(self, same_side_result, valve_pos):
        """Calculate two-phase safe insertion strategy."""
        valve_center_x, valve_center_y = valve_pos[0], valve_pos[1]
        
        # Phase 1: Safe insertion to gap centers
        safe_radius = (self.hub_radius + self.valve_inner_radius) / 2
        gap1_angle, gap2_angle = same_side_result['gap_angles']
        
        left_safe = (valve_center_x + safe_radius * math.cos(gap1_angle),
                     valve_center_y + safe_radius * math.sin(gap1_angle),
                     valve_pos[2])
        right_safe = (valve_center_x + safe_radius * math.cos(gap2_angle),
                      valve_center_y + safe_radius * math.sin(gap2_angle),
                      valve_pos[2])
        
        return {
            'strategy': 'two_phase_safe_insertion',
            'feasible': same_side_result['feasible'],
            'phase1_left_safe': left_safe,
            'phase1_right_safe': right_safe,
            'phase2_left_final': same_side_result['left_target'],
            'phase2_right_final': same_side_result['right_target'],
            'safe_radius': safe_radius,
            'final_radius': self.valve_inner_radius,
            'gap_angles': (gap1_angle, gap2_angle),
            'same_side_result': same_side_result
        }
    
    def complete_insertion_task(self):
        """Mark insertion task complete and end session."""
        rospy.loginfo("Insertion task completed")
        self.end_insertion_session()
        
    def abort_insertion_task(self):
        """Abort current insertion task and end session."""
        rospy.logwarn("Insertion task aborted")
        self.end_insertion_session()
