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
        # Parse module IDs and determine assembly parameters
        self.module_ids = [int(x.strip()) for x in module_ids_str.split(',')]
        self.number_of_modules = len(self.module_ids)
        self.leader_id = self.module_ids[-1]  # Leader is the last in the chain (carries end-effector)
        
        rospy.loginfo(f"Initialized NModuleTFCalculator:")
        rospy.loginfo(f"  Module IDs: {self.module_ids}")
        rospy.loginfo(f"  Number of modules: {self.number_of_modules}")
        rospy.loginfo(f"  Leader ID: {self.leader_id}")
        
        # Physical parameters for valve rotation task
        self.claw_separation = 0.150  # Distance between left and right claws (updated from 168.8mm to 150mm)
        self.half_claw_separation = self.claw_separation / 2
        self.valve_inner_radius = 0.100  # Valve inner radius in meters
        self.valve_center = (3.000, 0.000)  # Valve center position in world frame
        self.claw_insertion_depth = 0.030  # Insertion depth of the claw into the valve
        self.module_distance = 0.52  # Distance between neighboring modules in assemble state
        
        # End-effector offset parameters (from single UAV version)
        self.dual_fang_center_offset = 0.246  # Distance from UAV CoG to end-effector center
        self.end_effector_offset_z = 0.074382  # Z-axis offset of end-effector
        
    def calculate_assembly_to_leader_transform(self):
        """
        Calculate transformation from assembly CoG to leader UAV CoG.
        
        Assembly arrangement: UAVs are arranged linearly with module_distance spacing
        The leader (highest ID) is positioned at the furthest end from valve center.
        Assembly CoG is at the geometric center of all UAVs.
        
        Returns:
            dict: Transform information with 'offset_x', 'offset_y', 'offset_z'
        """
        if self.number_of_modules == 1:
            # Single UAV case - no assembly offset
            return {'offset_x': 0.0, 'offset_y': 0.0, 'offset_z': 0.0}
        
        # Calculate leader's position relative to assembly center
        # For n modules: positions are at -d*(n-1)/2, -d*(n-3)/2, ..., +d*(n-1)/2
        # where d = module_distance
        
        # Use input order as physical arrangement (X-axis small to large)
        leader_index = self.module_ids.index(self.leader_id)
        
        # Leader's position relative to assembly center
        # Assembly center is at index position (n-1)/2
        assembly_center_index = (self.number_of_modules - 1) / 2.0
        leader_offset_index = leader_index - assembly_center_index
        
        # Convert index offset to physical distance (X-axis in body frame)
        offset_x = leader_offset_index * self.module_distance
        
        return {
            'offset_x': offset_x,
            'offset_y': 0.0,  # No Y offset for linear arrangement
            'offset_z': 0.0   # No Z offset for planar arrangement
        }
    
    def calculate_leader_to_end_effector_transform(self, leader_yaw=0.0, leader_pitch=0.0):
        """
        Calculate transformation from leader UAV CoG to end-effector position.
        
        When pitch != 0, the end-effector arm rotates about the body Y-axis,
        causing a coupled shift in X/Z in the world frame.
        
        Args:
            leader_yaw (float): Leader UAV's yaw angle in radians
            leader_pitch (float): Leader UAV's pitch angle in radians
            
        Returns:
            dict: Transform with 'x', 'y', 'z' in world coordinates
        """
        cos_yaw = math.cos(leader_yaw)
        sin_yaw = math.sin(leader_yaw)
        cos_pitch = math.cos(leader_pitch)
        sin_pitch = math.sin(leader_pitch)
        
        # Body-frame X projection shrinks by cos(pitch), and couples into Z by -sin(pitch)
        arm_body_x = self.dual_fang_center_offset * cos_pitch
        arm_body_z = -self.dual_fang_center_offset * sin_pitch
        
        # Rotate body-frame horizontal component into world frame via yaw
        end_effector_x = arm_body_x * cos_yaw
        end_effector_y = arm_body_x * sin_yaw
        end_effector_z = self.end_effector_offset_z + arm_body_z
        
        return {
            'x': end_effector_x,
            'y': end_effector_y, 
            'z': end_effector_z
        }
    
    def transform_assembly_to_end_effector(self, assembly_pos, assembly_yaw, assembly_pitch=0.0):
        """
        Complete transformation from assembly CoG to end-effector position.
        
        Args:
            assembly_pos (tuple): Assembly CoG position (x, y, z)
            assembly_yaw (float): Assembly yaw angle in radians
            assembly_pitch (float): Assembly pitch angle in radians (default 0)
            
        Returns:
            tuple: End-effector position (x, y, z)
        """
        # Step 1: Transform from assembly CoG to leader CoG
        assembly_to_leader = self.calculate_assembly_to_leader_transform()
        
        cos_assembly_yaw = math.cos(assembly_yaw)
        sin_assembly_yaw = math.sin(assembly_yaw)
        
        leader_x = assembly_pos[0] + (assembly_to_leader['offset_x'] * cos_assembly_yaw - 
                                    assembly_to_leader['offset_y'] * sin_assembly_yaw)
        leader_y = assembly_pos[1] + (assembly_to_leader['offset_x'] * sin_assembly_yaw + 
                                    assembly_to_leader['offset_y'] * cos_assembly_yaw)
        leader_z = assembly_pos[2] + assembly_to_leader['offset_z']
        
        # Step 2: Transform from leader CoG to end-effector (pitch-aware)
        leader_to_ee = self.calculate_leader_to_end_effector_transform(assembly_yaw, assembly_pitch)
        
        end_effector_x = leader_x + leader_to_ee['x']
        end_effector_y = leader_y + leader_to_ee['y']
        end_effector_z = leader_z + leader_to_ee['z']
        
        return (end_effector_x, end_effector_y, end_effector_z)
    
    def get_leader_id(self):
        """Return the leader UAV ID (carries end-effector)"""
        return self.leader_id
    
    def get_follower_ids(self):
        """Return list of follower UAV IDs"""
        return [mid for mid in self.module_ids if mid != self.leader_id]
    
    def validate_module_configuration(self):
        """
        Validate that the module configuration is suitable for valve rotation.
        
        Returns:
            bool: True if configuration is valid
        """
        if self.number_of_modules < 1:
            rospy.logerr("No modules specified!")
            return False
            
        if self.leader_id not in self.module_ids:
            rospy.logerr(f"Leader ID {self.leader_id} not in module list {self.module_ids}")
            return False
            
        rospy.loginfo(f"Module configuration validated:")
        rospy.loginfo(f"  Total modules: {self.number_of_modules}")
        rospy.loginfo(f"  Leader (end-effector carrier): {self.leader_id}")
        rospy.loginfo(f"  Followers: {self.get_follower_ids()}")
        
        return True
    
    def test_coordinate_transforms(self, test_assembly_pos=(2.5, 0.0, 1.5), test_assembly_yaw=0.0):
        """
        Test coordinate transformations to verify correctness.
        
        This method tests the forward and inverse transforms to ensure consistency.
        
        Args:
            test_assembly_pos (tuple): Test assembly position
            test_assembly_yaw (float): Test assembly yaw angle
        """
        rospy.loginfo("=== Testing Coordinate Transformations ===")
        
        # Test 1: Forward transform (Assembly -> End-effector)
        end_effector_pos = self.transform_assembly_to_end_effector(test_assembly_pos, test_assembly_yaw)
        
        rospy.loginfo(f"Test assembly position: {test_assembly_pos}")
        rospy.loginfo(f"Test assembly yaw: {math.degrees(test_assembly_yaw):.1f}°")
        rospy.loginfo(f"Calculated end-effector position: {end_effector_pos}")
        
        # Test 2: Calculate individual transform components
        assembly_to_leader = self.calculate_assembly_to_leader_transform()
        leader_to_ee = self.calculate_leader_to_end_effector_transform(test_assembly_yaw)
        
        rospy.loginfo(f"Assembly to leader offset: {assembly_to_leader}")
        rospy.loginfo(f"Leader to end-effector offset: {leader_to_ee}")
        
        # Test 3: Verify expected distances
        distance_assembly_to_ee = math.sqrt(
            (end_effector_pos[0] - test_assembly_pos[0])**2 + 
            (end_effector_pos[1] - test_assembly_pos[1])**2 + 
            (end_effector_pos[2] - test_assembly_pos[2])**2
        )
        
        expected_distance = math.sqrt(
            (assembly_to_leader['offset_x'] + leader_to_ee['x'])**2 + 
            (assembly_to_leader['offset_y'] + leader_to_ee['y'])**2 + 
            (assembly_to_leader['offset_z'] + leader_to_ee['z'])**2
        )
        
        rospy.loginfo(f"Calculated distance Assembly->EE: {distance_assembly_to_ee:.4f}m")
        rospy.loginfo(f"Expected distance Assembly->EE: {expected_distance:.4f}m")
        rospy.loginfo(f"Distance error: {abs(distance_assembly_to_ee - expected_distance):.6f}m")
        
        # Test 4: Configuration-specific tests
        if self.number_of_modules == 2:
            expected_assembly_to_leader_x = self.module_distance / 2
            actual_offset_x = abs(assembly_to_leader['offset_x'])
            rospy.loginfo(f"2-UAV config: Expected leader offset {expected_assembly_to_leader_x:.3f}m, got {actual_offset_x:.3f}m")
            
        elif self.number_of_modules == 3:
            # For 3 UAVs, leader should be at +module_distance from center
            expected_assembly_to_leader_x = self.module_distance
            actual_offset_x = assembly_to_leader['offset_x']
            rospy.loginfo(f"3-UAV config: Expected leader offset {expected_assembly_to_leader_x:.3f}m, got {actual_offset_x:.3f}m")
        
        # Test 5: Different yaw angles
        test_yaws = [0, math.pi/4, math.pi/2, math.pi, -math.pi/2]
        rospy.loginfo("Testing different yaw angles:")
        for yaw in test_yaws:
            ee_pos = self.transform_assembly_to_end_effector(test_assembly_pos, yaw)
            distance = math.sqrt(sum((ee - ass)**2 for ee, ass in zip(ee_pos, test_assembly_pos)))
            rospy.loginfo(f"  Yaw {math.degrees(yaw):6.1f}°: EE at {ee_pos}, distance {distance:.3f}m")
        
        rospy.loginfo("=== Coordinate Transform Test Complete ===")
        return True


# Usage example and testing
if __name__ == "__main__":
    rospy.init_node("n_module_tf_test")
    
    # Test with different configurations
    test_configs = ["2,3", "1,2,3", "1"]
    
    for config in test_configs:
        rospy.loginfo(f"\n=== Testing configuration: {config} ===")
        
        # Set test parameter
        rospy.set_param("~module_ids", config)
        
        calculator = NModuleTFCalculator()
        
        if calculator.validate_module_configuration():
            # Test transform calculation
            test_assembly_pos = (2.5, 0.0, 1.5)  # Example position
            test_assembly_yaw = 0.0  # Facing forward
            
            end_effector_pos = calculator.transform_assembly_to_end_effector(
                test_assembly_pos, test_assembly_yaw
            )
            
            rospy.loginfo(f"Assembly position: {test_assembly_pos}")
            rospy.loginfo(f"End-effector position: {end_effector_pos}")
            rospy.loginfo(f"Transform offset: {[end_effector_pos[i] - test_assembly_pos[i] for i in range(3)]}")
        else:
            rospy.logerr(f"Invalid configuration: {config}")
    
    rospy.loginfo("N-Module TF Calculator test completed")
