#!/usr/bin/env python
"""
Calculator of tf between n UAV modules, end-effectors and valve center
By recieveing the robot id of moules in swarm, it calculate the tf from assembled cog to leader cog,and tf from cog of assebled Beetle to end-effector of leader_fang
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion
from math import pi, atan2, cos, sin
import tf

class NModuleTFCalculator:
    def __init__(self, module_ids_str = rospy.get_param("~module_ids", "1,2")):
        self.nuber_of_modules = len(module_ids_str.split(','))
        self.leader_id = module_ids_str.split(',')[0] if self.nuber_of_modules > 0 else None
        for i in range(self.nuber_of_modules):
        self.claw_separation = 0.150  # Distance between left and right claws (updated from 168.8mm to 150mm)
        self.half_claw_separation = self.claw_separation / 2
        self.valve_inner_radius = 0.100  # Valve inner radius in meters
        self.valve_center = (3.000, 0.000)  # Valve center position in world frame
        self.claw_insertation_depth = 0.030  # Insertion depth of the claw into the valve
        self.module_distance = 0.52 #Disntance between neibouring modules in assemble state
