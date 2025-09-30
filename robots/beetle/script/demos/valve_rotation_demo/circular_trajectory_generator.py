#!/usr/bin/env python

"""
Online Circular Trajectory Generator for Valve Rotation Tasks

This module implements Dragon-style online trajectory generation for "self-rotation + revolution" 
circular motions around valve center of gravity (CoG). It provides real-time
trajectory planning with constant angular velocity, radius adaptation, and 
centripetal force compensation.

Key Features:
- Online trajectory generation with constant angular velocity
- Radius adaptation and half-radius locking mechanism  
- Tangential velocity calculation for SE(3) position-velocity control
- Centripetal force compensation for stable circular motion
- Support for both contact and manipulation phases

Based on Dragon interface Contact.execute() and Manipulate.execute() patterns.
"""

import rospy
import numpy as np
import math
from geometry_msgs.msg import Point, Vector3


class CircularTrajectoryGenerator(object):
    """
    Online circular trajectory generator for valve rotation tasks
    
    Implements Dragon-style "self-rotation + revolution" trajectory generation with:
    - Constant angular velocity around valve CoG
    - Radius adaptation based on current geometry
    - Half-radius locking when system reaches 80% target velocity
    - Real-time trajectory updates for SE(3) control
    """
    
    def __init__(self, 
                 angular_velocity=0.5,      # rad/s - constant angular velocity
                 init_torque=0.1,           # N·m - initial torque for engagement
                 torque_limit=3.0,          # N·m - maximum torque limit
                 rate=20.0,                 # Hz - control update rate
                 radius_lock_threshold=0.8, # Lock radius when reaching 80% target velocity
                 debug_view=False):
        """
        Initialize circular trajectory generator
        
        Args:
            angular_velocity: Constant angular velocity around valve CoG (rad/s)
            init_torque: Initial engagement torque (N·m)
            torque_limit: Maximum allowable torque (N·m)
            rate: Control update frequency (Hz)
            radius_lock_threshold: Velocity ratio threshold for radius locking
            debug_view: Enable debug logging
        """
        self.angular_velocity = angular_velocity
        self.init_torque = init_torque
        self.torque_limit = torque_limit
        self.dt = 1.0 / rate
        self.radius_lock_threshold = radius_lock_threshold
        self.debug_view = debug_view
        
        # Trajectory state
        self.valve_center = None
        self.current_angle = 0.0
        self.target_radius = None
        self.locked_radius = None
        self.is_radius_locked = False
        
        # Trajectory history for adaptation
        self.angle_history = []
        self.radius_history = []
        self.velocity_history = []
        
        # Convergence tracking
        self.velocity_samples = []
        self.velocity_sample_size = 10  # samples for moving average
        
        rospy.loginfo(f"CircularTrajectoryGenerator initialized - ω={angular_velocity:.2f} rad/s, rate={rate:.1f} Hz")
    
    def initialize(self, valve_center, initial_uav_pos):
        """
        Initialize trajectory generation with valve center and current UAV position
        
        Args:
            valve_center: Valve CoG position [x, y, z]
            initial_uav_pos: Current UAV position [x, y, z]
        """
        self.valve_center = np.array(valve_center)
        uav_pos = np.array(initial_uav_pos)
        
        # Calculate initial trajectory parameters
        relative_pos = uav_pos - self.valve_center
        self.target_radius = np.linalg.norm(relative_pos[:2])  # XY plane distance
        self.current_angle = math.atan2(relative_pos[1], relative_pos[0])
        
        # Reset state
        self.locked_radius = None
        self.is_radius_locked = False
        self.angle_history = [self.current_angle]
        self.radius_history = [self.target_radius]
        self.velocity_history = []
        self.velocity_samples = []
        
        rospy.loginfo(f"Trajectory initialized - Center: [{valve_center[0]:.3f}, {valve_center[1]:.3f}, {valve_center[2]:.3f}], "
                     f"Initial radius: {self.target_radius:.3f}m, angle: {math.degrees(self.current_angle):.1f}°")
    
    def update(self, current_uav_pos, current_valve_yaw):
        """
        Update trajectory generation based on current state
        
        Args:
            current_uav_pos: Current UAV position [x, y, z]
            current_valve_yaw: Current valve orientation (rad)
            
        Returns:
            dict: Trajectory update with position, velocity, and force compensation
        """
        if self.valve_center is None:
            rospy.logerr("Trajectory generator not initialized!")
            return None
            
        uav_pos = np.array(current_uav_pos)
        relative_pos = uav_pos - self.valve_center
        current_radius = np.linalg.norm(relative_pos[:2])
        
        # Update trajectory angle with constant angular velocity
        self.current_angle += self.angular_velocity * self.dt
        
        # Normalize angle to [-π, π]
        while self.current_angle > math.pi:
            self.current_angle -= 2 * math.pi
        while self.current_angle < -math.pi:
            self.current_angle += 2 * math.pi
        
        # Radius adaptation logic
        if not self.is_radius_locked:
            # Update target radius based on current geometry
            self.target_radius = current_radius
            
            # Check for radius locking condition
            if len(self.velocity_samples) >= self.velocity_sample_size:
                avg_velocity = np.mean(self.velocity_samples[-self.velocity_sample_size:])
                target_tangential_velocity = self.angular_velocity * self.target_radius
                
                if avg_velocity >= self.radius_lock_threshold * target_tangential_velocity:
                    self.locked_radius = self.target_radius
                    self.is_radius_locked = True
                    rospy.loginfo(f"Radius locked at {self.locked_radius:.3f}m (velocity reached {avg_velocity:.3f}/{target_tangential_velocity:.3f} m/s)")
        else:
            # Use locked radius
            self.target_radius = self.locked_radius
        
        # Calculate target position on circular trajectory
        target_pos = self.valve_center.copy()
        target_pos[0] += self.target_radius * math.cos(self.current_angle)
        target_pos[1] += self.target_radius * math.sin(self.current_angle)
        target_pos[2] = uav_pos[2]  # Maintain current height
        
        # Calculate tangential velocity for SE(3) control
        tangential_velocity = self.angular_velocity * self.target_radius
        target_vel = np.array([
            -tangential_velocity * math.sin(self.current_angle),  # vx = -v*sin(θ)
             tangential_velocity * math.cos(self.current_angle),  # vy = v*cos(θ) 
             0.0                                                  # vz = 0
        ])
        
        # Calculate target yaw (tangent to circle)
        target_yaw = self.current_angle + math.pi / 2  # 90° ahead of radius
        
        # Calculate centripetal force compensation
        centripetal_acceleration = (tangential_velocity ** 2) / self.target_radius
        centripetal_force = np.array([
            -centripetal_acceleration * math.cos(self.current_angle),  # inward force
            -centripetal_acceleration * math.sin(self.current_angle),
             0.0
        ])
        
        # Torque feedforward (proportional to angular velocity with limits)
        feedforward_torque = min(self.init_torque + 
                               abs(self.angular_velocity) * 0.5, self.torque_limit)
        target_torque = np.array([0.0, 0.0, feedforward_torque])
        
        # Update velocity tracking
        actual_velocity = np.linalg.norm(target_vel)
        self.velocity_samples.append(actual_velocity)
        if len(self.velocity_samples) > self.velocity_sample_size * 2:
            self.velocity_samples.pop(0)
        
        # Update history
        self.angle_history.append(self.current_angle)
        self.radius_history.append(self.target_radius)
        self.velocity_history.append(actual_velocity)
        
        # Limit history size
        max_history = 100
        if len(self.angle_history) > max_history:
            self.angle_history.pop(0)
            self.radius_history.pop(0)
            self.velocity_history.pop(0)
        
        # Debug logging
        if self.debug_view and len(self.angle_history) % 50 == 0:  # Every 2.5 seconds at 20Hz
            rospy.loginfo(f"Trajectory: θ={math.degrees(self.current_angle):.1f}°, "
                         f"r={self.target_radius:.3f}m, v={actual_velocity:.3f}m/s, "
                         f"locked={self.is_radius_locked}")
        
        return {
            'position': target_pos,
            'velocity': target_vel,
            'yaw': target_yaw,
            'angular_velocity': self.angular_velocity,
            'force': centripetal_force,
            'torque': target_torque,
            'radius': self.target_radius,
            'angle': self.current_angle,
            'locked': self.is_radius_locked
        }
    
    def get_trajectory_status(self):
        """
        Get current trajectory generation status
        
        Returns:
            dict: Status information including convergence metrics
        """
        if len(self.velocity_history) < 5:
            return {'status': 'initializing', 'convergence': 0.0}
        
        # Calculate velocity convergence
        recent_velocities = self.velocity_history[-10:]
        target_velocity = self.angular_velocity * self.target_radius
        velocity_error = abs(np.mean(recent_velocities) - target_velocity)
        convergence = max(0.0, 1.0 - velocity_error / target_velocity)
        
        return {
            'status': 'locked' if self.is_radius_locked else 'adapting',
            'convergence': convergence,
            'target_velocity': target_velocity,
            'actual_velocity': np.mean(recent_velocities) if recent_velocities else 0.0,
            'radius': self.target_radius,
            'angle_degrees': math.degrees(self.current_angle),
            'rotations_completed': abs(self.current_angle) / (2 * math.pi)
        }
    
    def reset(self):
        """Reset trajectory generator state"""
        self.valve_center = None
        self.current_angle = 0.0
        self.target_radius = None
        self.locked_radius = None
        self.is_radius_locked = False
        self.angle_history = []
        self.radius_history = []
        self.velocity_history = []
        self.velocity_samples = []
        
        rospy.loginfo("Trajectory generator reset")
        
    def set_angular_velocity(self, new_angular_velocity):
        """
        Update angular velocity (for dynamic adjustment)
        
        Args:
            new_angular_velocity: New angular velocity (rad/s)
        """
        old_velocity = self.angular_velocity
        self.angular_velocity = new_angular_velocity
        rospy.loginfo(f"Angular velocity updated: {old_velocity:.3f} → {new_angular_velocity:.3f} rad/s")


class CircularTrajectoryValidator(object):
    """
    Validation and monitoring for circular trajectory execution
    
    Provides convergence checking, error detection, and performance metrics
    for the circular trajectory generator.
    """
    
    def __init__(self, position_tolerance=0.05, velocity_tolerance=0.1):
        """
        Initialize trajectory validator
        
        Args:
            position_tolerance: Maximum allowable position error (m)
            velocity_tolerance: Maximum allowable velocity error (m/s)
        """
        self.position_tolerance = position_tolerance
        self.velocity_tolerance = velocity_tolerance
        
        # Tracking variables
        self.error_history = []
        self.max_error_samples = 50
        
    def validate_trajectory_point(self, target, actual):
        """
        Validate single trajectory point execution
        
        Args:
            target: Target trajectory point (position, velocity)
            actual: Actual UAV state (position, velocity)
            
        Returns:
            dict: Validation results with error metrics
        """
        target_pos = np.array(target['position'])
        actual_pos = np.array(actual['position'])
        
        target_vel = np.array(target['velocity']) if 'velocity' in target else np.zeros(3)
        actual_vel = np.array(actual['velocity']) if 'velocity' in actual else np.zeros(3)
        
        # Calculate errors
        pos_error = np.linalg.norm(target_pos - actual_pos)
        vel_error = np.linalg.norm(target_vel - actual_vel)
        
        # Store error history
        self.error_history.append({
            'timestamp': rospy.Time.now().to_sec(),
            'position_error': pos_error,
            'velocity_error': vel_error
        })
        
        if len(self.error_history) > self.max_error_samples:
            self.error_history.pop(0)
        
        # Determine validation status
        position_valid = pos_error < self.position_tolerance
        velocity_valid = vel_error < self.velocity_tolerance
        
        return {
            'valid': position_valid and velocity_valid,
            'position_error': pos_error,
            'velocity_error': vel_error,
            'position_valid': position_valid,
            'velocity_valid': velocity_valid
        }
    
    def get_performance_metrics(self):
        """
        Calculate trajectory execution performance metrics
        
        Returns:
            dict: Performance statistics
        """
        if len(self.error_history) < 5:
            return {'status': 'insufficient_data'}
        
        recent_errors = self.error_history[-20:]  # Last 1 second at 20Hz
        pos_errors = [e['position_error'] for e in recent_errors]
        vel_errors = [e['velocity_error'] for e in recent_errors]
        
        return {
            'mean_position_error': np.mean(pos_errors),
            'max_position_error': np.max(pos_errors),
            'mean_velocity_error': np.mean(vel_errors),
            'max_velocity_error': np.max(vel_errors),
            'samples': len(recent_errors),
            'tracking_quality': 1.0 - np.mean(pos_errors) / self.position_tolerance
        }