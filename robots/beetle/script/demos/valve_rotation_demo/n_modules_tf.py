#!/usr/bin/env python
"""
Calculator of tf between n UAVs, end-effectors and valve center
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from math import pi, atan2, cos, sin
import tf

class NModuleTFCalculator:
    def __init__(self, n_modules=2):
        self.n_modules = n_modules
        self.claw_separation = 0.150  # Distance between left and right claws (updated from 168.8mm to 150mm)
        self.half_claw_separation = self.claw_separation / 2
        self.valve_inner_radius = 0.100  # Valve inner radius in meters
        self.valve_center = (3.000, 0.000)  # Valve center position in world frame
        self.claw_insertation_depth = 0.030  # Insertion depth of the claw into the valve

    def calculate_end_effector_positions(self, valve_yaw):
        """Calculate end-effector positions based on valve yaw and number of modules"""
        positions = []
        angle_increment = 360 / self.n_modules
        
        for i in range(self.n_modules):
            angle_deg = valve_yaw + i * angle_increment
            angle_rad = math.radians(angle_deg)
            x = self.valve_center[0] + self.valve_inner_radius * math.cos(angle_rad)
            y = self.valve_center[1] + self.valve_inner_radius * math.sin(angle_rad)
            positions.append((x, y))
        
        return positions

    def calculate_uav_positions(self, end_effector_positions):
        """Calculate UAV positions based on end-effector positions"""
        uav_positions = []
        
        for (ee_x, ee_y) in end_effector_positions:
            # Calculate the direction vector from valve center to end-effector
            dir_x = ee_x - self.valve_center[0]
            dir_y = ee_y - self.valve_center[1]
            length = math.sqrt(dir_x**2 + dir_y**2)
            norm_dir_x = dir_x / length
            norm_dir_y = dir_y / length
            
            # Calculate UAV position by moving back along the direction vector by half_claw_separation
            uav_x = ee_x - norm_dir_x * self.half_claw_separation
            uav_y = ee_y - norm_dir_y * self.half_claw_separation
            uav_positions.append((uav_x, uav_y))
        
        return uav_positions

    def run(self, valve_yaw):
        """Run the TF calculation and print results"""
        end_effector_positions = self.calculate_end_effector_positions(valve_yaw)
        uav_positions = self.calculate_uav_positions(end_effector_positions)