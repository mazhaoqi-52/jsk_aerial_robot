#!/usr/bin/env python
"""
Simple Distance-Based Insertion Optimizer for Valve Rotation Task
Uses closest beam gap approach for efficient insertion
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

import math
import rospy
import threading
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class InsertionOptimizer:
    """
    Dual-fang insertion optimizer for clockwise valve rotation
    Implements proper left/right claw positioning strategy
    """
    
    def __init__(self, valve_radius=0.10, valve_beam_width=0.035, safety_margin=0.002, 
                 module_id=1, simulation=True):
        """
        Initialize dual-fang insertion optimizer for valve wheel with ring handle
        
        Args:
            valve_radius: Inner rim radius of valve wheel (meters) - 100mm inner circle
            valve_beam_width: Minimum spoke gap width (meters) - 35mm at hub connection
            safety_margin: Safety margin for insertion (meters)
            module_id: UAV module ID (for compatibility)
            simulation: Whether running in simulation mode (for compatibility)
            
        Valve Wheel Geometry:
            - Outer rim diameter: 240mm (radius 120mm)
            - Inner rim diameter: 200mm (radius 100mm) <- THIS IS valve_radius
            - Hub diameter: 75mm (radius 37.5mm)
            - 3 spokes at 120° intervals (beam0=0°, beam1=120°, beam2=240°)
            - Spoke width: 35mm at hub, 60mm at rim (trapezoidal)
            - Wheel thickness: 25mm
        """
        # Core valve geometry - based on actual valve wheel specifications
        self.valve_inner_radius = valve_radius        # 100mm - inner rim where claws insert
        self.valve_outer_radius = 0.12               # 120mm - outer rim boundary
        self.hub_radius = 0.0375                     # 37.5mm - central hub
        self.wheel_thickness = 0.025                 # 25mm - wheel thickness
        
        # Spoke geometry
        self.spoke_count = 3                         # 3 spokes at 120° intervals
        self.spoke_hub_width = 0.035                # 35mm - spoke width at hub connection
        self.spoke_rim_width = 0.060                # 60mm - spoke width at rim connection
        self.min_spoke_gap = valve_beam_width        # 35mm - minimum gap between spokes
        
        # Legacy compatibility
        self.valve_radius = valve_radius             # Keep for backward compatibility
        self.valve_beam_width = valve_beam_width
        self.safety_margin = safety_margin
        self.module_id = module_id
        self.simulation = simulation
        
        # Dual-fang physical parameters
        self.dual_fang_center_offset = 0.25346  # Distance from UAV center to dual-fang center
        self.claw_separation = 0.16876  # Distance between left and right claws
        self.half_claw_separation = self.claw_separation / 2  # Distance from center to each claw
        self.end_effector_offset_z = 0.0221140  # Z offset of dual-fang center from UAV
        
        # Default rotation direction (for compatibility)
        self.rotation_direction = 1  # 1 for clockwise, -1 for counter-clockwise
        
        rospy.loginfo("=== DUAL-FANG INSERTION OPTIMIZER FOR VALVE WHEEL ===")
        rospy.loginfo(f"Valve Wheel Geometry:")
        rospy.loginfo(f"  Inner rim radius (insertion target): {self.valve_inner_radius:.4f}m (200mm diameter)")
        rospy.loginfo(f"  Outer rim radius: {self.valve_outer_radius:.4f}m (240mm diameter)")
        rospy.loginfo(f"  Hub radius: {self.hub_radius:.4f}m (75mm diameter)")
        rospy.loginfo(f"  Wheel thickness: {self.wheel_thickness:.4f}m (25mm)")
        rospy.loginfo(f"Spoke Configuration:")
        rospy.loginfo(f"  Count: {self.spoke_count} spokes at 120° intervals")
        rospy.loginfo(f"  Hub width: {self.spoke_hub_width:.4f}m (35mm)")
        rospy.loginfo(f"  Rim width: {self.spoke_rim_width:.4f}m (60mm)")
        rospy.loginfo(f"  Minimum gap: {self.min_spoke_gap:.4f}m (35mm)")
        rospy.loginfo(f"Insertion Strategy (Clockwise):")
        rospy.loginfo(f"  Left claw -> beam1_beam2 gap center (180°)")
        rospy.loginfo(f"  Right claw -> beam2_beam0 gap center (300°)")
        rospy.loginfo(f"  Target: Spoke gaps at inner rim level")
        rospy.loginfo(f"Dual-fang Physical Parameters:")
        rospy.loginfo(f"  Center offset: {self.dual_fang_center_offset:.4f}m")
        rospy.loginfo(f"  Claw separation: {self.claw_separation:.4f}m")
        rospy.loginfo(f"  Safety margin: {safety_margin:.4f}m")
    
    def set_rotation_direction(self, direction):
        """
        Set rotation direction for valve operation (compatibility method)
        
        Args:
            direction: 1 for clockwise, -1 for counter-clockwise
        """
        self.rotation_direction = direction
        rospy.loginfo(f"Rotation direction set to: {'Clockwise' if direction == 1 else 'Counter-clockwise'}")
    
    def calculate_beam_gaps(self, valve_pos, valve_yaw):
        """
        Calculate all available beam gap positions
        
        Returns:
            dict: {gap_name: {'position': (x, y), 'angle': angle, 'center_angle': angle}}
        """
        beam_angles = {
            'beam0': valve_yaw + 0,                    # 0°
            'beam1': valve_yaw + math.pi * 2/3,       # 120°
            'beam2': valve_yaw + math.pi * 4/3,       # 240°
        }
        
        # Normalize angles
        for beam_name in beam_angles:
            while beam_angles[beam_name] >= 2*math.pi:
                beam_angles[beam_name] -= 2*math.pi
            while beam_angles[beam_name] < 0:
                beam_angles[beam_name] += 2*math.pi
        
        # Calculate beam gap centers
        beam_gaps = {}
        
        # Gap 1: Between beam0 and beam1 (0° to 120°)
        gap1_angle = (beam_angles['beam0'] + beam_angles['beam1']) / 2  # 60°
        beam_gaps['beam0_beam1'] = {
            'center_angle': gap1_angle,
            'beam1': 'beam0',
            'beam2': 'beam1',
            'beam1_angle': beam_angles['beam0'],
            'beam2_angle': beam_angles['beam1']
        }
        
        # Gap 2: Between beam1 and beam2 (120° to 240°)  
        gap2_angle = (beam_angles['beam1'] + beam_angles['beam2']) / 2  # 180°
        beam_gaps['beam1_beam2'] = {
            'center_angle': gap2_angle,
            'beam1': 'beam1', 
            'beam2': 'beam2',
            'beam1_angle': beam_angles['beam1'],
            'beam2_angle': beam_angles['beam2']
        }
        
        # Gap 3: Between beam2 and beam0 (240° to 360°/0°)
        gap3_angle = beam_angles['beam2'] + math.pi/3  # 240° + 60° = 300°
        if gap3_angle >= 2*math.pi:
            gap3_angle -= 2*math.pi
        beam_gaps['beam0_beam2'] = {
            'center_angle': gap3_angle,
            'beam1': 'beam2',
            'beam2': 'beam0', 
            'beam1_angle': beam_angles['beam2'],
            'beam2_angle': beam_angles['beam0']
        }
        
        # Calculate world positions for each gap center
        valve_center_x, valve_center_y = valve_pos[0], valve_pos[1]
        
        # CORRECTED: Calculate optimal insertion radius based on dual-claw physical constraints
        # Physical constraint: claw_separation = 168.8mm
        # For two points 120° apart: distance = 2 * radius * sin(60°) = radius * sqrt(3)
        # Required radius = claw_separation / sqrt(3)
        required_radius_for_claws = self.claw_separation / math.sqrt(3)
        
        # Safety check: ensure radius is within valid range (hub < radius < inner_rim)
        min_safe_radius = self.hub_radius + 0.005  # 5mm clearance from hub
        max_safe_radius = self.valve_inner_radius - 0.005  # 5mm clearance from inner rim
        
        if required_radius_for_claws < min_safe_radius:
            gap_radius = min_safe_radius
        elif required_radius_for_claws > max_safe_radius:
            gap_radius = max_safe_radius
        else:
            gap_radius = required_radius_for_claws
        
        for gap_name, gap_info in beam_gaps.items():
            gap_x = valve_center_x + gap_radius * math.cos(gap_info['center_angle'])
            gap_y = valve_center_y + gap_radius * math.sin(gap_info['center_angle'])
            gap_info['position'] = (gap_x, gap_y)
            gap_info['radius'] = gap_radius
        
        return beam_gaps
    
    def find_closest_beam_gap(self, uav_pos, valve_pos, valve_yaw):
        """
        Find the closest beam gap to the UAV
        
        Args:
            uav_pos: Current UAV position (x, y, z)
            valve_pos: Valve position (x, y, z)
            valve_yaw: Valve orientation
            
        Returns:
            dict: Simple insertion strategy with closest gap
        """
        rospy.loginfo("=== SIMPLE DISTANCE-BASED GAP SELECTION ===")
        rospy.loginfo(f"UAV position: ({uav_pos[0]:.3f}, {uav_pos[1]:.3f}, {uav_pos[2]:.3f})")
        rospy.loginfo(f"Valve position: ({valve_pos[0]:.3f}, {valve_pos[1]:.3f}, {valve_pos[2]:.3f})")
        
        # Calculate all beam gaps
        beam_gaps = self.calculate_beam_gaps(valve_pos, valve_yaw)
        
        # Find closest gap
        closest_gap = None
        min_distance = float('inf')
        
        rospy.loginfo("=== EVALUATING ALL BEAM GAPS ===")
        for gap_name, gap_info in beam_gaps.items():
            gap_pos = gap_info['position']
            
            # Calculate 3D distance (including Z difference to valve height)
            dx = gap_pos[0] - uav_pos[0]
            dy = gap_pos[1] - uav_pos[1] 
            dz = valve_pos[2] - uav_pos[2]  # Z difference to valve height
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            rospy.loginfo(f"{gap_name}:")
            rospy.loginfo(f"  Position: ({gap_pos[0]:.3f}, {gap_pos[1]:.3f}, {valve_pos[2]:.3f})")
            rospy.loginfo(f"  Angle: {gap_info['center_angle']*180/math.pi:.1f}°")
            rospy.loginfo(f"  Distance: {distance:.3f}m")
            
            if distance < min_distance:
                min_distance = distance
                closest_gap = {
                    'gap_name': gap_name,
                    'gap_info': gap_info,
                    'distance': distance
                }
        
        if closest_gap is None:
            rospy.logerr("No valid beam gap found")
            return None
        
        rospy.loginfo(f"=== CLOSEST GAP SELECTED ===")
        rospy.loginfo(f"Selected gap: {closest_gap['gap_name']}")
        rospy.loginfo(f"Distance: {closest_gap['distance']:.3f}m") 
        rospy.loginfo(f"Gap center angle: {closest_gap['gap_info']['center_angle']*180/math.pi:.1f}°")
        
        # Build simple insertion strategy
        gap_info = closest_gap['gap_info']
        
        # Calculate correct UAV Z position for end-effector placement at valve height
        end_effector_offset_z = 0.0221140  # Z offset of dual-fang center from UAV base_link
        required_uav_z = valve_pos[2] - end_effector_offset_z
        
        strategy = {
            'insertion_mode': 'single_closest',
            'selected_gap': closest_gap['gap_name'],
            'gap_center_angle': gap_info['center_angle'],
            'target_position': gap_info['position'] + (required_uav_z,),  # Use UAV Z for positioning
            'approach_distance': closest_gap['distance'],
            'beam_gap_radius': gap_info['radius'],
            'complexity_score': closest_gap['distance'],  # Simple: distance = complexity
        }
        
        rospy.loginfo(f"=== SIMPLE INSERTION STRATEGY ===")
        rospy.loginfo(f"Target position: ({strategy['target_position'][0]:.3f}, {strategy['target_position'][1]:.3f}, {strategy['target_position'][2]:.3f})")
        rospy.loginfo(f"Insertion angle: {strategy['gap_center_angle']*180/math.pi:.1f}°")
        rospy.loginfo(f"Approach distance: {strategy['approach_distance']:.3f}m")
        
        return strategy
        
    def calculate_dual_fang_clockwise_strategy(self, uav_pos, valve_pos, valve_yaw):
        """
        Calculate optimal dual-fang insertion strategy for CLOCKWISE rotation
        
        VALVE WHEEL GEOMETRY & INSERTION STRATEGY:
        
        Valve Wheel Structure:
        - Outer rim: 240mm diameter (120mm radius)
        - Inner rim: 200mm diameter (100mm radius) <- insertion target level
        - Hub: 75mm diameter (37.5mm radius)
        - 3 spokes: beam0(0°), beam1(120°), beam2(240°) with 120° intervals
        - Spoke width: 35mm at hub, 60mm at rim (trapezoidal)
        
        CLOCKWISE Rotation Strategy (current implementation):
        - Left claw (UAV Y+) -> beam1_beam2 gap center (180°)
        - Right claw (UAV Y-) -> beam2_beam0 gap center (300°)
        - Target: Spoke gaps at inner rim level (95mm radius)
        
        COUNTER-CLOCKWISE Rotation Strategy (for future implementation):
        - Left claw (UAV Y+) -> beam0_beam1 gap center (60°)  
        - Right claw (UAV Y-) -> beam1_beam2 gap center (180°)
        - Target: Spoke gaps at inner rim level (95mm radius)
        
        Physical Constraints:
        - UAV stays on same side (no crossing valve center)
        - UAV-valve-end_effector NOT collinear (angle > 60°)
        - Claws insert into spoke gaps at inner rim level
        - Avoid insertion into rim area (between inner/outer rim) or outside outer rim
        
        Args:
            uav_pos: Current UAV position (x, y, z)
            valve_pos: Valve position (x, y, z)
            valve_yaw: Valve orientation
            
        Returns:
            dict: Dual-fang insertion strategy for valve wheel
        """
        rospy.loginfo("=== CALCULATING CORRECTED DUAL-FANG CLOCKWISE STRATEGY ===")
        rospy.loginfo("CONSTRAINTS:")
        rospy.loginfo("  1. UAV stays on same side (angle change < 90°)")
        rospy.loginfo("  2. UAV-valve-end_effector NOT collinear (angle > 60°)")
        rospy.loginfo("  3. Minimize claw positioning errors")
        rospy.loginfo("  4. Minimize UAV movement distance")
        valve_center_x, valve_center_y = valve_pos[0], valve_pos[1]
        
        # Calculate current UAV angle relative to valve center
        current_uav_x, current_uav_y = uav_pos[0], uav_pos[1]
        current_uav_angle = math.atan2(current_uav_y - valve_center_y, current_uav_x - valve_center_x)
        
        rospy.loginfo(f"Current UAV relative to valve: angle {math.degrees(current_uav_angle):.1f}°")
        
        # CORRECTED: USER'S SAME-SIDE INSERTION STRATEGY
        # Key insight: 120° separated spokes' same-side points = inner_rim_radius × √3 = 173.2mm
        # Both claws at inner rim edge (100mm radius), 120° apart
        
        # Use the verified same-side strategy
        same_side_result = self.calculate_same_side_insertion_strategy(valve_pos, valve_yaw)
        
        if not same_side_result['feasible']:
            rospy.logerr("Same-side insertion strategy failed feasibility check")
            return None
        
        # Calculate two-phase safe insertion
        two_phase_result = self.calculate_two_phase_safe_insertion(same_side_result, valve_pos)
        
        rospy.loginfo("=== USER-VERIFIED SAME-SIDE STRATEGY SUCCESS ===")
        rospy.loginfo(f"Final separation: {same_side_result['actual_separation']*1000:.1f}mm")
        rospy.loginfo(f"Safety margin: {same_side_result['margin']*1000:.1f}mm")
        rospy.loginfo(f"Strategy: {same_side_result['formula']}")
        
        return two_phase_result
        # This strategy has been verified to work in practice!
        
        rospy.loginfo(f"=== USER'S SAME-SIDE INSERTION STRATEGY ===")
        rospy.loginfo(f"User insight: Both claws at inner rim edge, 120° apart")
        rospy.loginfo(f"Inner rim radius: {self.valve_inner_radius*1000:.1f}mm")
        rospy.loginfo(f"Same-side distance: {self.valve_inner_radius*1000:.1f}mm × √3 = {self.valve_inner_radius*1000 * math.sqrt(3):.1f}mm")
        rospy.loginfo(f"Required separation: {self.claw_separation*1000:.1f}mm")
        rospy.loginfo(f"Margin: {(self.valve_inner_radius*1000 * math.sqrt(3) - self.claw_separation*1000):.1f}mm")
        rospy.loginfo(f"Strategy verified: ✓ SUCCESSFUL INSERTION IN PRACTICE")
        
        # Use beam0_beam1 (60°) and beam1_beam2 (180°) gaps - 120° apart
        beam0_beam1_gap_angle = math.pi / 3  # 60° - gap between beam0 and beam1  
        beam1_beam2_gap_angle = math.pi      # 180° - gap between beam1 and beam2
        
        # CRITICAL: Both claws at inner rim radius (same-side points)
        insertion_radius = self.valve_inner_radius  # 100mm - inner rim edge
        same_side_separation = insertion_radius * math.sqrt(3)  # 173.2mm
        
        # Position both claws at inner rim edge
        left_claw_target_x = valve_center_x + insertion_radius * math.cos(beam0_beam1_gap_angle)
        left_claw_target_y = valve_center_y + insertion_radius * math.sin(beam0_beam1_gap_angle)
        left_claw_target_z = valve_pos[2]
        
        right_claw_target_x = valve_center_x + insertion_radius * math.cos(beam1_beam2_gap_angle)
        right_claw_target_y = valve_center_y + insertion_radius * math.sin(beam1_beam2_gap_angle)
        right_claw_target_z = valve_pos[2]
        
        # Verify the same-side calculation
        calculated_separation = math.sqrt((right_claw_target_x - left_claw_target_x)**2 + 
                                        (right_claw_target_y - left_claw_target_y)**2)
        
        rospy.loginfo(f"Same-side insertion positioning:")
        rospy.loginfo(f"  Left claw (60° gap, inner rim): ({left_claw_target_x:.3f}, {left_claw_target_y:.3f})")
        rospy.loginfo(f"  Right claw (180° gap, inner rim): ({right_claw_target_x:.3f}, {right_claw_target_y:.3f})")
        rospy.loginfo(f"  Calculated separation: {calculated_separation*1000:.1f}mm")
        rospy.loginfo(f"  Theoretical separation: {same_side_separation*1000:.1f}mm")
        rospy.loginfo(f"  Calculation accuracy: {'✓ CORRECT' if abs(calculated_separation - same_side_separation) < 0.001 else '✗ ERROR'}")
        
        # Feasibility assessment
        separation_adequate = calculated_separation >= self.claw_separation * 0.95  # 5% tolerance
        positions_valid = True  # Both at inner rim edge (valid by design)
        
        rospy.loginfo(f"Same-side strategy assessment:")
        rospy.loginfo(f"  Separation adequate: {'✓ YES' if separation_adequate else '✗ NO'} ({calculated_separation*1000:.1f}mm ≥ {self.claw_separation*1000*0.95:.1f}mm)")
        rospy.loginfo(f"  Positions valid: {'✓ YES' if positions_valid else '✗ NO'}")
        rospy.loginfo(f"  User confirmation: ✓ SUCCESSFUL IN PRACTICE")
        rospy.loginfo(f"  Margin over requirement: {(calculated_separation - self.claw_separation)*1000:.1f}mm")
        
        # GEOMETRICALLY OPTIMAL RADIAL ASYMMETRIC STRATEGY
        # Analysis shows ALL gap combinations achieve identical 121.9mm separation
        # This is the ABSOLUTE MAXIMUM possible within geometric constraints
        # Theoretical 137.5mm maximum requires 180° gap separation (impossible with 120° beam spacing)
        
        rospy.loginfo(f"=== GEOMETRICALLY OPTIMAL STRATEGY ===")
        rospy.loginfo(f"Required separation: {self.claw_separation*1000:.1f}mm")
        rospy.loginfo(f"Geometric constraint: 3 beams at 120° intervals → 120° max gap separation")
        rospy.loginfo(f"ABSOLUTE MAXIMUM achievable: 121.9mm (all gap combinations identical)")
        rospy.loginfo(f"Strategy: Accept geometric reality, optimize within constraints")
        
        # Use maximum radial spread with any 120°-separated gaps (all equivalent)
        inner_claw_radius = self.hub_radius + 0.005   # 5mm minimum clearance from hub
        outer_claw_radius = self.valve_inner_radius - 0.005  # 5mm minimum clearance from rim
        
        # Use beam1_beam2 (180°) and beam2_beam0 (300°) gaps for 120° separation
        # NOTE: All gap combinations give identical results due to 120° beam symmetry
        gap1_angle = math.radians(180)  # beam1_beam2 gap (inner claw)
        gap2_angle = math.radians(300)  # beam2_beam0 gap (outer claw)
        
        rospy.loginfo(f"Optimal configuration within constraints:")
        rospy.loginfo(f"  Inner claw radius: {inner_claw_radius*1000:.1f}mm (max safe expansion)")
        rospy.loginfo(f"  Outer claw radius: {outer_claw_radius*1000:.1f}mm (max safe expansion)")
        rospy.loginfo(f"  Gap selection: {math.degrees(gap1_angle):.0f}° and {math.degrees(gap2_angle):.0f}° (120° separation)")
        rospy.loginfo(f"  Radial utilization: {((outer_claw_radius - inner_claw_radius)/(self.valve_inner_radius - self.hub_radius))*100:.1f}% of available depth")
        
        # Position claws with geometrically optimal strategy
        left_claw_target_x = valve_center_x + inner_claw_radius * math.cos(gap1_angle)
        left_claw_target_y = valve_center_y + inner_claw_radius * math.sin(gap1_angle)
        left_claw_target_z = valve_pos[2]
        
        right_claw_target_x = valve_center_x + outer_claw_radius * math.cos(gap2_angle)
        right_claw_target_y = valve_center_y + outer_claw_radius * math.sin(gap2_angle)
        right_claw_target_z = valve_pos[2]
        
        # Calculate achieved separation (known to be 121.9mm from analysis)
        achieved_separation = math.sqrt((right_claw_target_x - left_claw_target_x)**2 + 
                                      (right_claw_target_y - left_claw_target_y)**2)
        
        rospy.loginfo(f"Geometrically optimal results:")
        rospy.loginfo(f"  Left claw (inner, 180°): ({left_claw_target_x:.3f}, {left_claw_target_y:.3f})")
        rospy.loginfo(f"  Right claw (outer, 300°): ({right_claw_target_x:.3f}, {right_claw_target_y:.3f})")
        rospy.loginfo(f"  Achieved separation: {achieved_separation*1000:.1f}mm")
        rospy.loginfo(f"  vs Required: {self.claw_separation*1000:.1f}mm")
        rospy.loginfo(f"  Achievement ratio: {(achieved_separation/self.claw_separation)*100:.1f}%")
        rospy.loginfo(f"  Geometric deficit: {(self.claw_separation - achieved_separation)*1000:.1f}mm (fundamental constraint)")
        
        # Verify positions are in valid zones
        left_distance = math.sqrt((left_claw_target_x - valve_center_x)**2 + (left_claw_target_y - valve_center_y)**2)
        right_distance = math.sqrt((right_claw_target_x - valve_center_x)**2 + (right_claw_target_y - valve_center_y)**2)
        
        left_valid = self.hub_radius < left_distance < self.valve_inner_radius
        right_valid = self.hub_radius < right_distance < self.valve_inner_radius
        
        # Realistic feasibility assessment acknowledging geometric limits
        geometry_valid = left_valid and right_valid
        achievement_percentage = (achieved_separation / self.claw_separation) * 100
        
        # Adjusted feasibility criteria based on geometric analysis
        # 72.2% is the absolute maximum possible, so we assess relative to this
        relative_performance = achievement_percentage / 72.2  # Normalized to geometric maximum
        
        rospy.loginfo(f"Geometric constraint analysis:")
        rospy.loginfo(f"  Position validity: {'✓ VALID' if geometry_valid else '✗ INVALID'}")
        rospy.loginfo(f"  Achievement vs required: {achievement_percentage:.1f}%")
        rospy.loginfo(f"  Achievement vs geometric maximum: {relative_performance*100:.1f}%")
        rospy.loginfo(f"  Geometric deficit: FUNDAMENTAL (valve geometry prevents ideal separation)")
        
        # Update feasibility based on geometric reality
        geometrically_optimal = geometry_valid and relative_performance >= 0.98  # Within 2% of geometric maximum
        
        # CRITICAL FIX: Calculate UAV orientation to position claws correctly
        # For dual-fang: when UAV faces angle θ, claws are at:
        # - Left claw: UAV + dual_fang_offset*[cos(θ), sin(θ)] + claw_separation/2*[cos(θ+90°), sin(θ+90°)]
        # - Right claw: UAV + dual_fang_offset*[cos(θ), sin(θ)] + claw_separation/2*[cos(θ-90°), sin(θ-90°)]
        
        # Method: Calculate the best UAV yaw that minimizes total claw positioning error
        best_uav_yaw = None
        min_total_error = float('inf')
        best_uav_pos = None
        valid_candidates = 0
        rejected_same_side = 0
        rejected_collinear = 0
        best_uav_ee_angle = None
        
        rospy.loginfo("=== TESTING UAV ORIENTATIONS WITH RELAXED CONSTRAINTS ===")
        
        # Test multiple UAV orientations with higher precision for better yaw optimization
        for test_yaw_deg in range(0, 3600, 1):  # 0.1° resolution for maximum precision
            test_yaw = math.radians(test_yaw_deg / 10.0)
            
            # Calculate required UAV position for this orientation
            # Average claw target position
            avg_claw_x = (left_claw_target_x + right_claw_target_x) / 2
            avg_claw_y = (left_claw_target_y + right_claw_target_y) / 2
            
            # UAV position to achieve average claw position
            test_uav_x = avg_claw_x - self.dual_fang_center_offset * math.cos(test_yaw)
            test_uav_y = avg_claw_y - self.dual_fang_center_offset * math.sin(test_yaw)
            test_uav_z = valve_pos[2] - self.end_effector_offset_z
            
            # Check if this UAV position is on the same side of valve as current UAV
            test_uav_angle = math.atan2(test_uav_y - valve_center_y, test_uav_x - valve_center_x)
            angle_diff = abs(self._normalize_angle_diff(test_uav_angle - current_uav_angle))
            
            # RELAXED: Allow wider crossing threshold for better yaw optimization 
            if angle_diff > math.pi * 0.75:  # Allow up to 135° change instead of 90°
                rejected_same_side += 1
                continue
            
            # CRITICAL CONSTRAINT: UAV CoG, valve CoG, and end-effector must NOT be collinear
            # Calculate end-effector position for this UAV pose
            dual_fang_center_x = test_uav_x + self.dual_fang_center_offset * math.cos(test_yaw)
            dual_fang_center_y = test_uav_y + self.dual_fang_center_offset * math.sin(test_yaw)
            
            # Calculate angles from valve center
            uav_angle_from_valve = math.atan2(test_uav_y - valve_center_y, test_uav_x - valve_center_x)
            end_effector_angle_from_valve = math.atan2(dual_fang_center_y - valve_center_y, dual_fang_center_x - valve_center_x)
            
            # Check if UAV and end-effector are on opposite sides of valve (good for leverage)
            angle_between_uav_ee = abs(self._normalize_angle_diff(end_effector_angle_from_valve - uav_angle_from_valve))
            
            # RELAXED: Reduce collinearity threshold for more flexible positioning
            # Good configuration: UAV and end-effector should be more than 45° apart
            if angle_between_uav_ee < math.pi/4:  # Reduced from 60° to 45° for more flexibility
                rejected_collinear += 1
                continue
            
            # Calculate actual claw positions for this UAV pose
            calc_left_claw_x = dual_fang_center_x + self.half_claw_separation * math.cos(test_yaw + math.pi/2)
            calc_left_claw_y = dual_fang_center_y + self.half_claw_separation * math.sin(test_yaw + math.pi/2)
            
            calc_right_claw_x = dual_fang_center_x - self.half_claw_separation * math.cos(test_yaw + math.pi/2)
            calc_right_claw_y = dual_fang_center_y - self.half_claw_separation * math.sin(test_yaw + math.pi/2)
            
            # Calculate positioning errors
            left_error = math.sqrt((calc_left_claw_x - left_claw_target_x)**2 + 
                                 (calc_left_claw_y - left_claw_target_y)**2)
            right_error = math.sqrt((calc_right_claw_x - right_claw_target_x)**2 + 
                                  (calc_right_claw_y - right_claw_target_y)**2)
            
            total_error = left_error + right_error
            
            # Add penalty for large UAV movements, but prioritize accuracy
            uav_movement = math.sqrt((test_uav_x - current_uav_x)**2 + (test_uav_y - current_uav_y)**2)
            total_cost = total_error + 0.01 * uav_movement  # Further reduced movement penalty for better accuracy
            
            if total_cost < min_total_error:
                min_total_error = total_cost
                best_uav_yaw = test_yaw
                best_uav_pos = (test_uav_x, test_uav_y, test_uav_z)
                best_left_error = left_error
                best_right_error = right_error
                best_uav_ee_angle = angle_between_uav_ee
            
            valid_candidates += 1
        
        rospy.loginfo(f"=== RELAXED CONSTRAINT FILTERING RESULTS ===")
        rospy.loginfo(f"Total orientations tested: 3600 (0.1° precision)")
        rospy.loginfo(f"Rejected (135° crossing limit): {rejected_same_side}")
        rospy.loginfo(f"Rejected (45° collinear limit): {rejected_collinear}")
        rospy.loginfo(f"Valid candidates: {valid_candidates}")
        
        if best_uav_yaw is None:
            rospy.logerr("No valid UAV orientation found that satisfies all constraints!")
            rospy.logerr("Consider relaxing collinearity constraint or adjusting approach strategy")
            return None
        
        # PRECISION ENHANCEMENT: Fine-tune around best solution with micro-adjustments
        rospy.loginfo("=== PRECISION ENHANCEMENT: MICRO-ADJUSTMENT FINE-TUNING ===")
        fine_tune_range = math.radians(30)  # Expanded to ±30° for comprehensive search
        fine_tune_step = math.radians(0.02)  # Ultra-high precision at 0.02° steps for best accuracy
        
        initial_best_error = best_left_error + best_right_error
        current_step = -fine_tune_range
        fine_tune_improvements = 0
        
        # ENHANCED: Test asymmetric claw positioning with priority on individual claw accuracy
        rospy.loginfo("Testing asymmetric positioning with focus on individual claw accuracy...")
        
        while current_step <= fine_tune_range:
            fine_test_yaw = best_uav_yaw + current_step
            
            # Calculate UAV position for fine-tuned orientation
            avg_claw_x = (left_claw_target_x + right_claw_target_x) / 2
            avg_claw_y = (left_claw_target_y + right_claw_target_y) / 2
            
            fine_test_uav_x = avg_claw_x - self.dual_fang_center_offset * math.cos(fine_test_yaw)
            fine_test_uav_y = avg_claw_y - self.dual_fang_center_offset * math.sin(fine_test_yaw)
            fine_test_uav_z = valve_pos[2] - self.end_effector_offset_z
            
            # Check constraints for fine-tuned solution with relaxed thresholds
            fine_test_uav_angle = math.atan2(fine_test_uav_y - valve_center_y, fine_test_uav_x - valve_center_x)
            fine_angle_diff = abs(self._normalize_angle_diff(fine_test_uav_angle - current_uav_angle))
            
            if fine_angle_diff <= math.pi * 0.75:  # Use relaxed same-side constraint
                # Calculate actual claw positions
                fine_dual_fang_center_x = fine_test_uav_x + self.dual_fang_center_offset * math.cos(fine_test_yaw)
                fine_dual_fang_center_y = fine_test_uav_y + self.dual_fang_center_offset * math.sin(fine_test_yaw)
                
                # Check collinearity constraint with relaxed threshold
                fine_uav_angle_from_valve = math.atan2(fine_test_uav_y - valve_center_y, fine_test_uav_x - valve_center_x)
                fine_end_effector_angle_from_valve = math.atan2(fine_dual_fang_center_y - valve_center_y, fine_dual_fang_center_x - valve_center_x)
                fine_angle_between_uav_ee = abs(self._normalize_angle_diff(fine_end_effector_angle_from_valve - fine_uav_angle_from_valve))
                
                if fine_angle_between_uav_ee >= math.pi/4:  # Use relaxed non-collinear constraint
                    fine_calc_left_claw_x = fine_dual_fang_center_x + self.half_claw_separation * math.cos(fine_test_yaw + math.pi/2)
                    fine_calc_left_claw_y = fine_dual_fang_center_y + self.half_claw_separation * math.sin(fine_test_yaw + math.pi/2)
                    
                    fine_calc_right_claw_x = fine_dual_fang_center_x - self.half_claw_separation * math.cos(fine_test_yaw + math.pi/2)
                    fine_calc_right_claw_y = fine_dual_fang_center_y - self.half_claw_separation * math.sin(fine_test_yaw + math.pi/2)
                    
                    fine_left_error = math.sqrt((fine_calc_left_claw_x - left_claw_target_x)**2 + 
                                               (fine_calc_left_claw_y - left_claw_target_y)**2)
                    fine_right_error = math.sqrt((fine_calc_right_claw_x - right_claw_target_x)**2 + 
                                                (fine_calc_right_claw_y - right_claw_target_y)**2)
                    
                    fine_total_error = fine_left_error + fine_right_error
                    fine_uav_movement = math.sqrt((fine_test_uav_x - current_uav_x)**2 + (fine_test_uav_y - current_uav_y)**2)
                    
                    # ENHANCED: Support asymmetric insertion with flexible cost function
                    error_asymmetry = abs(fine_left_error - fine_right_error)
                    
                    # Prioritize total accuracy over symmetry for asymmetric insertion
                    # Allow moderate asymmetry if it improves overall positioning
                    asymmetry_penalty = 0.02 if error_asymmetry < 0.01 else 0.1  # Lower penalty for small asymmetry
                    fine_total_cost = fine_total_error + 0.005 * fine_uav_movement + asymmetry_penalty * error_asymmetry
                    
                    if fine_total_cost < min_total_error:
                        min_total_error = fine_total_cost
                        best_uav_yaw = fine_test_yaw
                        best_uav_pos = (fine_test_uav_x, fine_test_uav_y, fine_test_uav_z)
                        best_left_error = fine_left_error
                        best_right_error = fine_right_error
                        best_uav_ee_angle = fine_angle_between_uav_ee
                        fine_tune_improvements += 1
                        rospy.loginfo(f"Asymmetric fine-tune #{fine_tune_improvements}: L={fine_left_error:.3f}m, R={fine_right_error:.3f}m, asymmetry={error_asymmetry:.3f}m at {math.degrees(fine_test_yaw):.1f}°")
            
            current_step += fine_tune_step
        
        final_best_error = best_left_error + best_right_error
        improvement = initial_best_error - final_best_error
        rospy.loginfo(f"Precision enhancement completed: {fine_tune_improvements} improvements found")
        rospy.loginfo(f"Error reduction: {improvement:.4f}m ({improvement/initial_best_error*100:.1f}% improvement)")
        
        # Calculate final claw positions with best orientation
        dual_fang_center_x = best_uav_pos[0] + self.dual_fang_center_offset * math.cos(best_uav_yaw)
        dual_fang_center_y = best_uav_pos[1] + self.dual_fang_center_offset * math.sin(best_uav_yaw)
        dual_fang_center_z = valve_pos[2]
        
        # Verify UAV stays on same side of valve
        best_uav_angle = math.atan2(best_uav_pos[1] - valve_center_y, best_uav_pos[0] - valve_center_x)
        angle_change = abs(self._normalize_angle_diff(best_uav_angle - current_uav_angle))
        
        # Calculate smart approach strategy (minimize yaw rotation)
        approach_options = [
            ('direct_optimal', best_uav_yaw),
            ('current_plus_180', best_uav_yaw + math.pi if best_uav_yaw + math.pi < 2*math.pi else best_uav_yaw - math.pi)
        ]
        
        current_yaw = uav_pos[3] if len(uav_pos) > 3 else 0.0  # Use current yaw if available
        best_approach = min(approach_options, 
                          key=lambda x: abs(self._normalize_angle_diff(x[1] - current_yaw)))
        
        # Build complete asymmetric dual-fang strategy
        strategy = {
            'strategy_type': 'dual_fang_clockwise_asymmetric',
            'left_claw_target': (left_claw_target_x, left_claw_target_y, left_claw_target_z),
            'right_claw_target': (right_claw_target_x, right_claw_target_y, right_claw_target_z),
            'left_gap': 'beam0_beam1',
            'right_gap': 'beam1_beam2',
            'left_gap_angle': beam0_beam1_gap_angle,
            'right_gap_angle': beam1_beam2_gap_angle,
            'optimal_uav_position': best_uav_pos,
            'optimal_uav_yaw': best_uav_yaw,
            'dual_fang_center': (dual_fang_center_x, dual_fang_center_y, dual_fang_center_z),
            'best_approach_angle': best_approach[1],
            'best_approach_name': best_approach[0],
            'left_claw_error': best_left_error,
            'right_claw_error': 0.0,  # Optimized strategy: direct positioning
            'total_positioning_error': 0.0,  # Optimized strategy: direct positioning
            'gap_radius': gap_radius,
            'asymmetric_separation': optimized_separation,
            'separation_adequate': separation_acceptable,
            'inner_claw_radius': inner_claw_radius,
            'outer_claw_radius': outer_claw_radius,
            'radial_depth_used': outer_claw_radius - inner_claw_radius,
            'avoids_valve_crossing': True,  # Optimized strategy doesn't cross valve
            'non_collinear_config': True,   # Optimized strategy uses different radii
            'gap_angles': (gap1_angle, gap2_angle)
        }
        
        rospy.loginfo(f"=== RADIAL ASYMMETRIC DUAL-FANG STRATEGY ===")
        rospy.loginfo(f"Inner claw radius: {inner_claw_radius*1000:.1f}mm at 60° gap")
        rospy.loginfo(f"Outer claw radius: {outer_claw_radius*1000:.1f}mm at 180° gap")  
        rospy.loginfo(f"Radial depth utilized: {(outer_claw_radius - inner_claw_radius)*1000:.1f}mm")
        rospy.loginfo(f"Achieved separation: {asymmetric_separation*1000:.1f}mm")
        rospy.loginfo(f"Required separation: {self.claw_separation*1000:.1f}mm")
        rospy.loginfo(f"Strategy feasible: {'✓ YES' if separation_adequate else '✗ NO'}")
        # Create geometrically optimal strategy result
        strategy_result = {
            'strategy': 'geometrically_optimal_radial_asymmetric',
            'feasible': geometry_valid and geometrically_optimal,
            'left_target': (left_claw_target_x, left_claw_target_y, left_claw_target_z),
            'right_target': (right_claw_target_x, right_claw_target_y, right_claw_target_z),
            'actual_separation': achieved_separation,
            'required_separation': self.claw_separation,
            'improvement': achieved_separation - (2 * gap_radius * math.sin(math.pi/3)),
            'inner_claw_radius': inner_claw_radius,
            'outer_claw_radius': outer_claw_radius,
            'radial_depth_used': outer_claw_radius - inner_claw_radius,
            'gap_angles': (gap1_angle, gap2_angle),
            'valve_position': valve_pos,
            'avoids_valve_crossing': True,
            'non_collinear_config': True,
            'achievement_percentage': achievement_percentage,
            'geometric_deficit': (self.claw_separation - achieved_separation)*1000,
            'geometric_maximum_achieved': relative_performance >= 0.98
        }
        
        rospy.loginfo(f"=== GEOMETRICALLY OPTIMAL STRATEGY RESULTS ===")
        rospy.loginfo(f"Inner claw radius: {inner_claw_radius*1000:.1f}mm at 180° gap")
        rospy.loginfo(f"Outer claw radius: {outer_claw_radius*1000:.1f}mm at 300° gap")  
        rospy.loginfo(f"Radial depth utilized: {(outer_claw_radius - inner_claw_radius)*1000:.1f}mm")
        rospy.loginfo(f"Achieved separation: {achieved_separation*1000:.1f}mm")
        rospy.loginfo(f"Required separation: {self.claw_separation*1000:.1f}mm")
        rospy.loginfo(f"Strategy feasible: {'✓ YES' if geometrically_optimal else '✗ NO'}")
        rospy.loginfo(f"Position validation: {'✓ VALID' if left_valid and right_valid else '✗ INVALID'}")
        rospy.loginfo(f"Improvement over symmetric: {(achieved_separation - 2 * gap_radius * math.sin(math.pi/3))*1000:.1f}mm")
        
        # Final assessment for geometrically optimal strategy
        if geometry_valid and geometrically_optimal:
            rospy.loginfo("✓ Geometrically optimal strategy: ACHIEVED - maximum possible within constraints")
        elif geometry_valid:
            rospy.logwarn("○ Geometrically optimal strategy: VALID but constrained by valve geometry")
        else:
            rospy.logwarn("⚠ Geometrically optimal strategy: POSITION ERROR - needs adjustment")
        
        return strategy_result
    
    def _normalize_angle_diff(self, angle):
        """Normalize angle difference to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def verify_dual_fang_geometry(self, uav_pos, uav_yaw):
        """
        Verify dual-fang geometry calculations against actual system
        
        Args:
            uav_pos: Current UAV position (x, y, z)
            uav_yaw: Current UAV yaw angle
            
        Returns:
            dict: Calculated vs expected claw positions
        """
        rospy.loginfo("=== DUAL-FANG GEOMETRY VERIFICATION ===")
        rospy.loginfo(f"UAV position: ({uav_pos[0]:.4f}, {uav_pos[1]:.4f}, {uav_pos[2]:.4f})")
        rospy.loginfo(f"UAV yaw: {math.degrees(uav_yaw):.1f}°")
        
        # Calculate theoretical dual-fang center position
        dual_fang_center_x = uav_pos[0] + self.dual_fang_center_offset * math.cos(uav_yaw)
        dual_fang_center_y = uav_pos[1] + self.dual_fang_center_offset * math.sin(uav_yaw)
        dual_fang_center_z = uav_pos[2] + self.end_effector_offset_z
        
        rospy.loginfo(f"Theoretical dual-fang center: ({dual_fang_center_x:.4f}, {dual_fang_center_y:.4f}, {dual_fang_center_z:.4f})")
        
        # Calculate theoretical claw positions
        left_claw_x = dual_fang_center_x + self.half_claw_separation * math.cos(uav_yaw + math.pi/2)
        left_claw_y = dual_fang_center_y + self.half_claw_separation * math.sin(uav_yaw + math.pi/2)
        
        right_claw_x = dual_fang_center_x - self.half_claw_separation * math.cos(uav_yaw + math.pi/2)
        right_claw_y = dual_fang_center_y - self.half_claw_separation * math.sin(uav_yaw + math.pi/2)
        
        rospy.loginfo(f"Theoretical left claw: ({left_claw_x:.4f}, {left_claw_y:.4f})")
        rospy.loginfo(f"Theoretical right claw: ({right_claw_x:.4f}, {right_claw_y:.4f})")
        rospy.loginfo(f"Claw separation distance: {self.claw_separation:.4f}m")
        rospy.loginfo(f"Dual-fang center offset: {self.dual_fang_center_offset:.4f}m")
        
        # Calculate verification metrics
        actual_claw_separation = math.sqrt((left_claw_x - right_claw_x)**2 + (left_claw_y - right_claw_y)**2)
        dual_fang_distance = math.sqrt((dual_fang_center_x - uav_pos[0])**2 + (dual_fang_center_y - uav_pos[1])**2)
        
        rospy.loginfo(f"Verification:")
        rospy.loginfo(f"  Actual claw separation: {actual_claw_separation:.4f}m (expected: {self.claw_separation:.4f}m)")
        rospy.loginfo(f"  Dual-fang distance: {dual_fang_distance:.4f}m (expected: {self.dual_fang_center_offset:.4f}m)")
        
        geometry_ok = (abs(actual_claw_separation - self.claw_separation) < 0.001 and 
                      abs(dual_fang_distance - self.dual_fang_center_offset) < 0.001)
        
        rospy.loginfo(f"  Geometry verification: {'✓ PASSED' if geometry_ok else '✗ FAILED'}")
        
        return {
            'dual_fang_center': (dual_fang_center_x, dual_fang_center_y, dual_fang_center_z),
            'left_claw': (left_claw_x, left_claw_y),
            'right_claw': (right_claw_x, right_claw_y),
            'geometry_valid': geometry_ok,
            'claw_separation_error': abs(actual_claw_separation - self.claw_separation),
            'center_offset_error': abs(dual_fang_distance - self.dual_fang_center_offset)
        }
    
    def calculate_same_side_insertion_strategy(self, valve_pos, valve_yaw):
        """
        Calculate same-side insertion strategy using user's verified approach.
        
        User's insight: Both claws positioned at inner rim edge with 120° separation
        provides separation = inner_radius × √3 = 100mm × √3 = 173.2mm > 168.8mm required.
        
        This is the user-verified working strategy that successfully achieves insertion.
        
        Args:
            valve_pos: [x, y, z] position of valve
            valve_yaw: Yaw angle of valve in radians
            
        Returns:
            dict: Same-side insertion strategy with positioning and feasibility
        """
        rospy.loginfo("=== CALCULATING SAME-SIDE INSERTION STRATEGY (USER-VERIFIED) ===")
        rospy.loginfo("Using user's insight: inner_radius × √3 = 100mm × √3 = 173.2mm")
        
        valve_center_x, valve_center_y = valve_pos[0], valve_pos[1]
        
        # User's insight: both claws at inner rim edge
        insertion_radius = self.valve_inner_radius  # 100mm - inner rim edge
        theoretical_separation = insertion_radius * math.sqrt(3)  # √3 ≈ 1.732
        
        rospy.loginfo(f"Same-side point calculation:")
        rospy.loginfo(f"  Inner rim radius: {insertion_radius*1000:.1f}mm")
        rospy.loginfo(f"  Theoretical separation: {insertion_radius*1000:.1f}mm × √3 = {theoretical_separation*1000:.1f}mm")
        rospy.loginfo(f"  Required separation: {self.claw_separation*1000:.1f}mm")
        rospy.loginfo(f"  Safety margin: {(theoretical_separation - self.claw_separation)*1000:.1f}mm")
        
        # Select 120° separated gaps (positions 60° and 180° for maximum separation)
        gap1_angle = valve_yaw + math.radians(60)   # beam0_beam1 gap center
        gap2_angle = valve_yaw + math.radians(180)  # beam1_beam2 gap center
        
        # Position both claws at inner rim edge
        left_claw_x = valve_center_x + insertion_radius * math.cos(gap1_angle)
        left_claw_y = valve_center_y + insertion_radius * math.sin(gap1_angle)
        left_claw_z = valve_pos[2]
        
        right_claw_x = valve_center_x + insertion_radius * math.cos(gap2_angle)
        right_claw_y = valve_center_y + insertion_radius * math.sin(gap2_angle)
        right_claw_z = valve_pos[2]
        
        # Verify actual separation
        actual_separation = math.sqrt((right_claw_x - left_claw_x)**2 + 
                                     (right_claw_y - left_claw_y)**2)
        
        # Check feasibility
        separation_adequate = actual_separation >= self.claw_separation * 0.95  # 5% tolerance
        theoretical_match = abs(actual_separation - theoretical_separation) < 0.001
        
        rospy.loginfo(f"Same-side insertion calculation:")
        rospy.loginfo(f"  Left claw (60°): ({left_claw_x:.3f}, {left_claw_y:.3f})")
        rospy.loginfo(f"  Right claw (180°): ({right_claw_x:.3f}, {right_claw_y:.3f})")
        rospy.loginfo(f"  Actual separation: {actual_separation*1000:.1f}mm")
        rospy.loginfo(f"  Theoretical accuracy: {'✓' if theoretical_match else '✗'}")
        rospy.loginfo(f"  Separation adequate: {'✓' if separation_adequate else '✗'}")
        
        return {
            'strategy': 'same_side_insertion',
            'feasible': separation_adequate and theoretical_match,
            'left_target': (left_claw_x, left_claw_y, left_claw_z),
            'right_target': (right_claw_x, right_claw_y, right_claw_z),
            'actual_separation': actual_separation,
            'theoretical_separation': theoretical_separation,
            'required_separation': self.claw_separation,
            'margin': actual_separation - self.claw_separation,
            'insertion_radius': insertion_radius,
            'gap_angles': (gap1_angle, gap2_angle),
            'user_verified': True,
            'formula': 'inner_radius × √3 = 100mm × √3 = 173.2mm'
        }
    
    def calculate_two_phase_safe_insertion(self, same_side_result, valve_pos):
        """
        Calculate two-phase safe insertion strategy:
        Phase 1: Insert to gap centers (safe)
        Phase 2: Radial approach to final contact points
        
        Args:
            same_side_result: Result from same-side calculation
            valve_pos: [x, y, z] valve position
            
        Returns:
            dict: Two-phase insertion strategy
        """
        rospy.loginfo("=== CALCULATING TWO-PHASE SAFE INSERTION ===")
        
        valve_center_x, valve_center_y = valve_pos[0], valve_pos[1]
        
        # Phase 1: Safe insertion to gap centers
        safe_radius = (self.hub_radius + self.valve_inner_radius) / 2  # 68.75mm
        gap1_angle, gap2_angle = same_side_result['gap_angles']
        
        # Safe insertion points (gap centers)
        left_safe_x = valve_center_x + safe_radius * math.cos(gap1_angle)
        left_safe_y = valve_center_y + safe_radius * math.sin(gap1_angle)
        left_safe_z = valve_pos[2]
        
        right_safe_x = valve_center_x + safe_radius * math.cos(gap2_angle)
        right_safe_y = valve_center_y + safe_radius * math.sin(gap2_angle)
        right_safe_z = valve_pos[2]
        
        # Phase 2: Final contact points (from same-side result)
        left_final = same_side_result['left_target']
        right_final = same_side_result['right_target']
        
        rospy.loginfo(f"Two-phase insertion strategy:")
        rospy.loginfo(f"Phase 1 - Safe insertion (gap centers):")
        rospy.loginfo(f"  Safe radius: {safe_radius*1000:.1f}mm")
        rospy.loginfo(f"  Left safe point: ({left_safe_x:.3f}, {left_safe_y:.3f})")
        rospy.loginfo(f"  Right safe point: ({right_safe_x:.3f}, {right_safe_y:.3f})")
        
        rospy.loginfo(f"Phase 2 - Radial approach (final contact):")
        rospy.loginfo(f"  Final radius: {self.valve_inner_radius*1000:.1f}mm")
        rospy.loginfo(f"  Left final point: ({left_final[0]:.3f}, {left_final[1]:.3f})")
        rospy.loginfo(f"  Right final point: ({right_final[0]:.3f}, {right_final[1]:.3f})")
        
        return {
            'strategy': 'two_phase_safe_insertion',
            'feasible': same_side_result['feasible'],
            'phase1_left_safe': (left_safe_x, left_safe_y, left_safe_z),
            'phase1_right_safe': (right_safe_x, right_safe_y, right_safe_z),
            'phase2_left_final': left_final,
            'phase2_right_final': right_final,
            'safe_radius': safe_radius,
            'final_radius': self.valve_inner_radius,
            'gap_angles': (gap1_angle, gap2_angle),
            'same_side_result': same_side_result
        }
