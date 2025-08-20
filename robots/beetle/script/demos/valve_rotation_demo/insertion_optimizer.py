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
    Simple optimizer that selects the closest beam gap for insertion
    """
    
    def __init__(self, valve_radius=0.1075, valve_beam_width=0.024, safety_margin=0.002, 
                 module_id=1, simulation=True):
        """
        Initialize simple insertion optimizer
        
        Args:
            valve_radius: Radius of the valve (meters)
            valve_beam_width: Width of valve beams (meters)
            safety_margin: Safety margin for insertion (meters)
            module_id: UAV module ID (for compatibility)
            simulation: Whether running in simulation mode (for compatibility)
        """
        self.valve_radius = valve_radius
        self.valve_beam_width = valve_beam_width
        self.safety_margin = safety_margin
        self.module_id = module_id
        self.simulation = simulation
        
        # Default rotation direction (for compatibility)
        self.rotation_direction = 1  # 1 for clockwise, -1 for counter-clockwise
        
        rospy.loginfo("=== SIMPLE DISTANCE-BASED INSERTION OPTIMIZER ===")
        rospy.loginfo(f"Strategy: Select closest beam gap for insertion")
        rospy.loginfo(f"Valve radius: {valve_radius:.4f}m")
        rospy.loginfo(f"Beam width: {valve_beam_width:.4f}m")
        rospy.loginfo(f"Safety margin: {safety_margin:.4f}m")
    
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
        gap_radius = self.valve_radius - 0.012  # Insert 12mm into valve
        
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
