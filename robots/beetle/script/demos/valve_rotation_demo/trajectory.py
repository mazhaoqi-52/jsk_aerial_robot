#!/usr/bin/env python
"""
Trajectory generation module for valve rotation tasks.
Provides polynomial trajectories and online circular trajectory generation.
"""

import rospy
import numpy as np
import math
from math import pi, atan2, cos, sin


class AdaptiveTrajectoryController:
    """Adaptive trajectory controller - adjusts parameters based on tracking performance."""
    
    CONTROL_MODES = {
        'rotation': {
            'speed_min': 0.3, 'speed_max': 1.2,
            'error_threshold_high': 0.05, 'error_threshold_low': 0.015,
            'adaptation_rate': 0.1, 'history_window': 20, 'update_frequency': 5,
            'speed_reduction_factor': 0.95, 'speed_increase_factor': 1.1,
            'trend_reduction_factor': 0.95,
        },
        'disengagement': {
            'speed_min': 0.5, 'speed_max': 1.0,
            'error_threshold_high': 0.03, 'error_threshold_low': 0.040,
            'adaptation_rate': 0.15, 'history_window': 15, 'update_frequency': 3,
            'speed_reduction_factor': 0.7, 'speed_increase_factor': 1.05,
            'trend_reduction_factor': 0.85,
        }
    }
    
    def __init__(self, base_trajectory, control_mode='rotation'):
        self.base_trajectory = base_trajectory
        self.current_speed_factor = 1.0
        self.error_history = []
        self.update_counter = 0
        self.control_mode = control_mode
        self.convergence_check_window = 10
        self.convergence_threshold = 0.005
        self.min_speed_duration = 0
        self.convergence_detected = False
        self.performance_analyzer = PerformanceAnalyzer()
        self.start_time = rospy.Time.now().to_sec()
        self.min_execution_time = 20.0
        
        if control_mode not in self.CONTROL_MODES:
            control_mode = 'rotation'
        self.adaptation_params = self.CONTROL_MODES[control_mode].copy()
        
        rospy.loginfo(f"AdaptiveTrajectoryController: {control_mode} mode")
    
    def update_tracking_performance(self, position_error, velocity_error=None):
        """Update tracking performance data."""
        self.error_history.append({
            'position_error': position_error,
            'velocity_error': velocity_error,
            'timestamp': rospy.Time.now().to_sec()
        })
        
        if len(self.error_history) > self.adaptation_params['history_window']:
            self.error_history.pop(0)
        
        self.update_counter += 1
        if self.update_counter >= self.adaptation_params['update_frequency']:
            self._update_adaptive_parameters()
            self.update_counter = 0
    
    def _update_adaptive_parameters(self):
        """Update adaptive parameters based on performance."""
        if len(self.error_history) < 3:
            return
        
        performance = self.performance_analyzer.analyze_performance(self.error_history)
        new_speed_factor = self._compute_adaptive_speed(performance)
        
        # Smooth transition
        speed_change = new_speed_factor - self.current_speed_factor
        self.current_speed_factor += speed_change * self.adaptation_params['adaptation_rate']
        
        # Apply depth-adaptive speed limit if available
        effective_speed_max = self.adaptation_params['speed_max']
        if hasattr(self.base_trajectory, 'adaptive_speed_limit'):
            effective_speed_max = min(effective_speed_max, self.base_trajectory.adaptive_speed_limit)
        
        self.current_speed_factor = max(self.adaptation_params['speed_min'], 
                                        min(effective_speed_max, self.current_speed_factor))
    
    def _compute_adaptive_speed(self, performance):
        """Calculate adaptive speed factor."""
        avg_error = performance['avg_error']
        error_trend = performance['error_trend']
        
        if avg_error > self.adaptation_params['error_threshold_high']:
            return max(self.current_speed_factor * self.adaptation_params['speed_reduction_factor'], 
                       self.adaptation_params['speed_min'])
        elif avg_error < self.adaptation_params['error_threshold_low'] and error_trend < 0:
            return min(self.current_speed_factor * self.adaptation_params['speed_increase_factor'], 
                       self.adaptation_params['speed_max'])
        elif error_trend > 0.01:
            return max(self.current_speed_factor * self.adaptation_params['trend_reduction_factor'], 
                       self.adaptation_params['speed_min'])
        return self.current_speed_factor
    
    def get_current_speed_factor(self):
        return self.current_speed_factor
    
    def process_trajectory_point(self, base_point):
        """Process trajectory point and apply adaptive adjustments."""
        if base_point is None:
            return None
        
        if self.control_mode == 'rotation' and self._check_convergence():
            rospy.loginfo("Trajectory convergence detected")
            return None
        
        if hasattr(self.base_trajectory, '_apply_speed_factor_to_point'):
            return self.base_trajectory._apply_speed_factor_to_point(base_point, self.current_speed_factor)
        return base_point
    
    def _check_convergence(self):
        """Check if convergence has been achieved."""
        if len(self.error_history) < self.convergence_check_window:
            return False
        
        execution_time = rospy.Time.now().to_sec() - self.start_time
        if execution_time < self.min_execution_time:
            return False
        
        recent_errors = [e['position_error'] for e in self.error_history[-self.convergence_check_window:]]
        error_variance = np.var(recent_errors)
        mean_error = np.mean(recent_errors)
        
        position_converged = (error_variance < self.convergence_threshold ** 2) and (mean_error < 0.050)
        
        # Check valve rotation completion
        valve_rotation_complete = True
        if self.control_mode == 'rotation' and hasattr(self.base_trajectory, 'get_progress'):
            try:
                target_rotation = abs(getattr(self.base_trajectory, 'rotation_angle', math.pi/2))
                trajectory_progress = self.base_trajectory.get_progress()
                current_valve_rotation = rospy.get_param('/valve_rotation_current', 0.0)
                current_valve_rotation = max(current_valve_rotation, trajectory_progress * target_rotation)
                
                rotation_error = abs(target_rotation - abs(current_valve_rotation))
                angle_tolerance = math.radians(5.0)
                valve_rotation_complete = rotation_error < angle_tolerance or trajectory_progress > 0.95
            except Exception as e:
                rospy.logwarn(f"Could not verify valve rotation: {e}")
                valve_rotation_complete = True
        
        if position_converged and valve_rotation_complete:
            self.min_speed_duration += 1
            if self.min_speed_duration >= 10:
                self.convergence_detected = True
                return True
        else:
            self.min_speed_duration = 0
        
        return False
    
    @classmethod
    def create_for_rotation(cls, base_trajectory):
        return cls(base_trajectory, control_mode='rotation')
    
    @classmethod  
    def create_for_disengagement(cls, base_trajectory):
        return cls(base_trajectory, control_mode='disengagement')


class PerformanceAnalyzer:
    """Analyzes tracking error trends and characteristics."""
    
    def analyze_performance(self, error_history):
        if len(error_history) < 3:
            return {'avg_error': 0.0, 'max_error': 0.0, 'error_trend': 0.0, 'stability': 1.0}
        
        position_errors = [e['position_error'] for e in error_history]
        avg_error = np.mean(position_errors)
        max_error = np.max(position_errors)
        error_trend = np.polyfit(np.arange(len(position_errors)), position_errors, 1)[0]
        stability = 1.0 / (1.0 + np.var(position_errors) * 1000)
        
        return {'avg_error': avg_error, 'max_error': max_error, 
                'error_trend': error_trend, 'stability': stability}


class PolynomialTrajectory:
    """5th-order polynomial trajectory for smooth motion."""
    
    def __init__(self, duration):
        self.duration = duration
        self.coeffs_x = None
        self.coeffs_y = None
        self.coeffs_z = None
        self.coeffs_scalar = None  
        self.start_time = None
        self.is_scalar = False  
        self.is_paused = False
        self.pause_start_time = None
        self.total_pause_duration = 0.0  

    def compute_coefficients(self, start, target):
        """Compute 5th-order polynomial coefficients with adaptive strategy."""
        if abs(target - start) < 1e-6:
            return np.array([0, 0, 0, 0, 0, start])
            
        T = self.duration
        if T <= 0:
            raise ValueError("Duration must be positive.")
        
        movement_distance = abs(target - start)
        
        # Use linear trajectory for short movements (<200mm) to avoid speed spikes
        if movement_distance < 0.200:
            return np.array([0, 0, 0, 0, target - start, start])
        
        # Use polynomial trajectory for larger movements with adaptive boundary velocity
        if movement_distance < 0.300:
            boundary_velocity_fraction = 0.30
        elif movement_distance < 0.500:
            boundary_velocity_fraction = 0.20
        else:
            boundary_velocity_fraction = 0.10
        
        average_velocity = movement_distance / T
        boundary_velocity = average_velocity * boundary_velocity_fraction
        velocity_direction = 1 if target > start else -1
        start_velocity = boundary_velocity * velocity_direction
        end_velocity = boundary_velocity * velocity_direction
        
        start_vel_normalized = start_velocity * T
        end_vel_normalized = end_velocity * T
        
        A = np.array([
            [0,    0,    0,    0,  0, 1],
            [1,    1,    1,    1,  1, 1],
            [0,    0,    0,    0,  1, 0],
            [5,    4,    3,    2,  1, 0],
            [0,    0,    0,    2,  0, 0],
            [20,  12,    6,    2,  0, 0]
        ])
        B = np.array([start, target, start_vel_normalized, end_vel_normalized, 0, 0])
        
        return np.linalg.solve(A, B)
    
    def generate_trajectory(self, start_pos, target_pos):
        """Generate polynomial trajectory."""
        if isinstance(start_pos, (int, float)) and isinstance(target_pos, (int, float)):
            self.is_scalar = True
            self.coeffs_scalar = self.compute_coefficients(start_pos, target_pos)
            self.start_value = start_pos
            self.target_value = target_pos
        else:
            self.is_scalar = False
            self.coeffs_x = self.compute_coefficients(start_pos[0], target_pos[0])
            self.coeffs_y = self.compute_coefficients(start_pos[1], target_pos[1])
            self.coeffs_z = self.compute_coefficients(start_pos[2], target_pos[2])
            self.start_pos = start_pos
            self.target_pos = target_pos
        self.start_time = rospy.Time.now().to_sec()

    def evaluate(self):
        """Evaluate current trajectory position."""
        if self.start_time is None:
            return None
            
        current_time = rospy.Time.now().to_sec()
        if self.is_paused:
            elapsed_time = self.pause_start_time - self.start_time - self.total_pause_duration
        else:
            elapsed_time = current_time - self.start_time - self.total_pause_duration
        
        elapsed_time = max(0.0, elapsed_time)
        
        if elapsed_time >= self.duration:
            return self.target_value if self.is_scalar else self.target_pos
        
        normalized_time = elapsed_time / self.duration
        
        if self.is_scalar:
            return self._evaluate_polynomial(self.coeffs_scalar, normalized_time)
        else:
            return (self._evaluate_polynomial(self.coeffs_x, normalized_time),
                    self._evaluate_polynomial(self.coeffs_y, normalized_time),
                    self._evaluate_polynomial(self.coeffs_z, normalized_time))

    def get_next_position(self):
        """Get next position (compatibility method)."""
        return self.evaluate()
    
    def get_velocity(self):
        """Get current velocity from trajectory."""
        if self.start_time is None:
            return None
        elapsed_time = rospy.Time.now().to_sec() - self.start_time
        
        if elapsed_time >= self.duration:
            return 0.0 if self.is_scalar else (0.0, 0.0, 0.0)
        
        normalized_time = elapsed_time / self.duration
        T_vel = np.array([5*normalized_time**4, 4*normalized_time**3, 3*normalized_time**2, 
                         2*normalized_time, 1, 0]) / self.duration
        
        if self.is_scalar:
            return np.dot(self.coeffs_scalar, T_vel)
        else:
            return (np.dot(self.coeffs_x, T_vel),
                    np.dot(self.coeffs_y, T_vel),
                    np.dot(self.coeffs_z, T_vel))

    def evaluate_at_time(self, elapsed_time):
        """Calculate trajectory value at specific elapsed time."""
        normalized_time = elapsed_time / self.duration
        
        if normalized_time <= 0:
            return self.start_value if self.is_scalar else self.start_pos
        elif normalized_time >= 1:
            return self.target_value if self.is_scalar else self.target_pos
        
        if self.is_scalar:
            return self._evaluate_polynomial(self.coeffs_scalar, normalized_time)
        else:
            return [self._evaluate_polynomial(self.coeffs_x, normalized_time),
                    self._evaluate_polynomial(self.coeffs_y, normalized_time),
                    self._evaluate_polynomial(self.coeffs_z, normalized_time)]
    
    def _evaluate_polynomial(self, coeffs, t):
        """Evaluate 5th order polynomial at normalized time t."""
        return coeffs[5] + coeffs[4]*t + coeffs[3]*t**2 + coeffs[2]*t**3 + coeffs[1]*t**4 + coeffs[0]*t**5

    def pause(self):
        """Pause trajectory execution."""
        if not self.is_paused and self.start_time is not None:
            self.is_paused = True
            self.pause_start_time = rospy.Time.now().to_sec()

    def resume(self):
        """Resume trajectory execution."""
        if self.is_paused and self.pause_start_time is not None:
            self.total_pause_duration += rospy.Time.now().to_sec() - self.pause_start_time
            self.is_paused = False
            self.pause_start_time = None

    def is_trajectory_paused(self):
        """Check if trajectory is paused."""
        return self.is_paused

class ValveRotationTrajectory:
    """Trajectory generator for rotating around valve center with constant end-effector distance."""
    
    def __init__(self, rotation_duration, valve_center, end_effector_rotation_radius, start_angle, 
                 rotation_angle=2*pi, grasp_height=None, rotation_direction=1,
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823,
                 enable_adaptive_control=True, control_mode='rotation'):
        self.valve_center = valve_center
        self.end_effector_rotation_radius = end_effector_rotation_radius
        self.end_effector_distance = end_effector_rotation_radius
        self.end_effector_offset_x = end_effector_offset_x
        self.end_effector_offset_y = end_effector_offset_y
        self.end_effector_offset_z = end_effector_offset_z
        
        # UAV rotation radius = end_effector distance + offset
        self.uav_rotation_radius = end_effector_rotation_radius + end_effector_offset_x
        self.uav_target_distance = self.uav_rotation_radius
        self.start_angle = start_angle
        self.rotation_angle = rotation_angle * rotation_direction
        self.grasp_height = grasp_height
        self.rotation_direction = rotation_direction
        
        # Depth-adaptive rotation parameters
        insertion_depth = self._estimate_insertion_depth(grasp_height)
        if insertion_depth > 0.25:
            self.adaptive_duration_factor = 1.5
            self.adaptive_speed_limit = 0.6
        elif insertion_depth > 0.15:
            self.adaptive_duration_factor = 1.25
            self.adaptive_speed_limit = 0.8
        else:
            self.adaptive_duration_factor = 1.0
            self.adaptive_speed_limit = 1.0
        
        self.rotation_duration = rotation_duration * self.adaptive_duration_factor
        self._angle_trajectory = None
        self._current_speed_factor = 1.0
        
        # Adaptive control settings
        self.enable_adaptive_control = enable_adaptive_control
        self.control_mode = control_mode
        self.adaptive_controller = None
        if self.enable_adaptive_control:
            self.adaptive_controller = AdaptiveTrajectoryController(self, control_mode=control_mode)
    
    def _estimate_insertion_depth(self, grasp_height):
        """Estimate insertion depth from grasp height."""
        if grasp_height is None:
            return 0.0
        return max(0, 1.0 - grasp_height)
        
    def start_trajectory(self):
        """Start rotation trajectory."""
        end_angle = self.start_angle + self.rotation_angle
        self._angle_trajectory = PolynomialTrajectory(self.rotation_duration)
        self._angle_trajectory.generate_trajectory(self.start_angle, end_angle)
        
    def get_progress(self):
        """Get trajectory completion progress (0.0 to 1.0)."""
        if not self._angle_trajectory:
            return 0.0
        try:
            if hasattr(self._angle_trajectory, 'start_time') and hasattr(self._angle_trajectory, 'duration'):
                elapsed_time = rospy.Time.now().to_sec() - self._angle_trajectory.start_time
                return min(1.0, max(0.0, elapsed_time / self._angle_trajectory.duration))
        except:
            pass
        return 0.0
        
    def set_speed_factor(self, speed_factor):
        """Set speed factor."""
        self._current_speed_factor = speed_factor
        if self._angle_trajectory:
            self._angle_trajectory.duration = self.rotation_duration / speed_factor
    
    def update_tracking_performance(self, position_error, velocity_error=None):
        """Update tracking performance."""
        if self.adaptive_controller:
            self.adaptive_controller.update_tracking_performance(position_error, velocity_error)
    
    def get_current_speed_factor(self):
        """Get current speed factor."""
        if self.adaptive_controller:
            return self.adaptive_controller.get_current_speed_factor()
        return 1.0
        
    def get_next_position_and_yaw(self):
        """Get next UAV position and orientation."""
        base_point = self._get_base_trajectory_point()
        if base_point is None:
            return None
        if self.adaptive_controller and self.enable_adaptive_control:
            return self.adaptive_controller.process_trajectory_point(base_point)
        return base_point
    
    def _get_base_trajectory_point(self):
        """Get base trajectory point (original implementation)"""
        if self._angle_trajectory is None:
            return None
            
        current_angle = self._angle_trajectory.evaluate()
        if current_angle is None:
            return None
            
        center_x, center_y, center_z = self.valve_center
        uav_rotation_radius = getattr(self, 'uav_rotation_radius', 
                                    self.end_effector_rotation_radius + self.end_effector_offset_x)
        
        uav_target_x = center_x + uav_rotation_radius * cos(current_angle)
        uav_target_y = center_y + uav_rotation_radius * sin(current_angle)
        uav_target_z = self.grasp_height if self.grasp_height is not None else center_z - self.end_effector_offset_z
        
        # Calculate yaw pointing toward valve center
        uav_target_yaw = atan2(center_y - uav_target_y, center_x - uav_target_x)
        
        return ((uav_target_x, uav_target_y, uav_target_z), uav_target_yaw)
        
    def get_next_uav_position_and_yaw(self):
        """Get next UAV position and orientation (compatibility)."""
        return self.get_next_position_and_yaw()
    
    def get_next_position(self):
        """Get next position."""
        result = self.get_next_position_and_yaw()
        return result[0] if result else None
        
    def is_complete(self):
        """Check if trajectory is complete."""
        return self.get_next_position() is None

    def pause(self):
        """Pause trajectory execution."""
        if self._angle_trajectory:
            self._angle_trajectory.pause()

    def resume(self):
        """Resume trajectory execution."""
        if self._angle_trajectory:
            self._angle_trajectory.resume()

    def is_trajectory_paused(self):
        """Check if trajectory is paused."""
        return self._angle_trajectory.is_trajectory_paused() if self._angle_trajectory else False

    def reset_to_current_position(self, current_uav_pos, current_uav_yaw):
        """Reset trajectory to start from current UAV position."""
        center_x, center_y, _ = self.valve_center
        
        # Calculate current end-effector position
        cos_yaw, sin_yaw = cos(current_uav_yaw), sin(current_uav_yaw)
        current_ee_x = current_uav_pos[0] + cos_yaw * self.end_effector_offset_x - sin_yaw * self.end_effector_offset_y
        current_ee_y = current_uav_pos[1] + sin_yaw * self.end_effector_offset_x + cos_yaw * self.end_effector_offset_y
        
        # Calculate new start angle and distance
        new_start_angle = atan2(current_ee_y - center_y, current_ee_x - center_x)
        actual_distance = math.sqrt((current_ee_x - center_x)**2 + (current_ee_y - center_y)**2)
        
        # Update radius
        self.end_effector_rotation_radius = actual_distance
        self.end_effector_distance = actual_distance
        if hasattr(self, 'uav_target_distance'):
            self.uav_target_distance = max(0.03, actual_distance - self.end_effector_offset_x)
            self.uav_rotation_radius = self.uav_target_distance
        
        # Reinitialize trajectory
        self.start_angle = new_start_angle
        self._angle_trajectory = PolynomialTrajectory(self.rotation_duration)
        self._angle_trajectory.generate_trajectory(new_start_angle, new_start_angle + self.rotation_angle)


def validate_uav_position(uav_pos, uav_yaw, valve_center, 
                         end_effector_offset_x=0.246, end_effector_offset_y=0.0):
    """Validate UAV position for safe trajectory execution."""
    uav_to_valve_distance = math.sqrt((uav_pos[0] - valve_center[0])**2 + 
                                    (uav_pos[1] - valve_center[1])**2)
    
    cos_yaw, sin_yaw = cos(uav_yaw), sin(uav_yaw)
    ee_world_x = uav_pos[0] + cos_yaw * end_effector_offset_x - sin_yaw * end_effector_offset_y
    ee_world_y = uav_pos[1] + sin_yaw * end_effector_offset_x + cos_yaw * end_effector_offset_y
    ee_to_valve_distance = math.sqrt((ee_world_x - valve_center[0])**2 + (ee_world_y - valve_center[1])**2)
    
    valve_radius = 0.1225
    min_safe_uav_distance = valve_radius + end_effector_offset_x * 0.2
    
    errors = []
    if uav_to_valve_distance < min_safe_uav_distance:
        errors.append(f"UAV too close: {uav_to_valve_distance:.3f}m < {min_safe_uav_distance:.3f}m")
    if ee_to_valve_distance < valve_radius * 0.25:
        errors.append(f"End-effector inside valve: {ee_to_valve_distance:.3f}m")
    
    return len(errors) == 0, errors


def create_constant_distance_trajectory(current_uav_pos, current_uav_yaw, valve_center, 
                                      rotation_angle, rotation_duration, 
                                      end_effector_offset_x=0.246, end_effector_offset_y=0.0, 
                                      end_effector_offset_z=0.0743823, rotation_direction=1,
                                      enable_adaptive_control=True, control_mode='rotation'):
    """Create constant distance rotation trajectory with validation."""
    # Validate UAV position
    is_valid, errors = validate_uav_position(current_uav_pos, current_uav_yaw, valve_center,
                                            end_effector_offset_x, end_effector_offset_y)
    if not is_valid:
        for error in errors:
            rospy.logerr(f"Position validation failed: {error}")
        return None
    
    # Calculate end-effector position and distance
    cos_yaw, sin_yaw = cos(current_uav_yaw), sin(current_uav_yaw)
    ee_x = current_uav_pos[0] + cos_yaw * end_effector_offset_x - sin_yaw * end_effector_offset_y
    ee_y = current_uav_pos[1] + sin_yaw * end_effector_offset_x + cos_yaw * end_effector_offset_y
    
    valve_x, valve_y, _ = valve_center
    end_effector_distance = math.sqrt((ee_x - valve_x)**2 + (ee_y - valve_y)**2)
    start_angle = atan2(ee_y - valve_y, ee_x - valve_x)
    
    return ValveRotationTrajectory(
        rotation_duration=rotation_duration,
        valve_center=valve_center,
        end_effector_rotation_radius=end_effector_distance,
        start_angle=start_angle,
        rotation_angle=rotation_angle,
        grasp_height=current_uav_pos[2],
        rotation_direction=rotation_direction,
        end_effector_offset_x=end_effector_offset_x,
        end_effector_offset_y=end_effector_offset_y,
        end_effector_offset_z=end_effector_offset_z,
        enable_adaptive_control=enable_adaptive_control,
        control_mode=control_mode
    )


class AlignToGraspTrajectory:
    """Legacy align trajectory (deprecated - use insertion optimizer)."""
    
    def __init__(self, approach_duration, valve_center, valve_pose_yaw, grasp_height, 
                 valve_radius=0.1225, valve_beam_width=0.0185, 
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823,
                 claw_separation=0.150, rotation_direction=1, insertion_offset=0.015):
        self.approach_duration = approach_duration
        self.valve_center = valve_center
        self.valve_pose_yaw = valve_pose_yaw
        self.grasp_height = grasp_height
        self.valve_radius = valve_radius
        self.rotation_direction = rotation_direction
        self.insertion_offset = insertion_offset
        self.end_effector_offset_x = end_effector_offset_x
        self.end_effector_offset_y = end_effector_offset_y
        self.end_effector_offset_z = end_effector_offset_z
        self._trajectory = None
        self._target_body_pos = None
        self._target_body_yaw = None
        
    def calculate_grasp_position_and_yaw(self):
        """Calculate grasp position and yaw."""
        center_x, center_y, center_z = self.valve_center
        target_distance = self.valve_radius + self.end_effector_offset_x
        target_angle = self.valve_pose_yaw + pi
        
        body_target_x = center_x + target_distance * cos(target_angle)
        body_target_y = center_y + target_distance * sin(target_angle)
        body_target_z = self.grasp_height
        
        end_effector_x = body_target_x + self.end_effector_offset_x * cos(target_angle)
        end_effector_y = body_target_y + self.end_effector_offset_x * sin(target_angle)
        end_effector_z = body_target_z + self.end_effector_offset_z
        
        return ((body_target_x, body_target_y, body_target_z), target_angle, 
                (end_effector_x, end_effector_y, end_effector_z))
    
    def set_start_position(self, start_pos):
        """Set start position and generate trajectory."""
        self._target_body_pos, self._target_body_yaw, _ = self.calculate_grasp_position_and_yaw()
        self._trajectory = PolynomialTrajectory(self.approach_duration)
        self._trajectory.generate_trajectory(start_pos, self._target_body_pos)
        
    def get_next_position_and_yaw(self):
        """Get next position and yaw."""
        if self._trajectory is None:
            return None
        pos = self._trajectory.evaluate()
        return (pos, self._target_body_yaw) if pos else None
    
    def get_next_position(self):
        """Get next position."""
        result = self.get_next_position_and_yaw()
        return result[0] if result else None
    
    def is_complete(self):
        """Check if trajectory is complete."""
        return self.get_next_position() is None


class ConstantDistanceValveRotationTrajectory(ValveRotationTrajectory):
    """Legacy compatibility class - use ValveRotationTrajectory instead."""
    
    def __init__(self, valve_center, end_effector_distance, rotation_angle, 
                 rotation_duration, start_angle, 
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823,
                 rotation_direction=1, grasp_height=None):
        super().__init__(
            rotation_duration=rotation_duration, valve_center=valve_center,
            end_effector_rotation_radius=end_effector_distance, start_angle=start_angle,
            rotation_angle=rotation_angle, grasp_height=grasp_height,
            rotation_direction=rotation_direction, end_effector_offset_x=end_effector_offset_x,
            end_effector_offset_y=end_effector_offset_y, end_effector_offset_z=end_effector_offset_z
        )


class AdaptiveTrajectoryPlanner:
    """Adaptive trajectory replanner for trajectory generation and replanning."""
    
    def __init__(self):
        self.precision_thresholds = {'position': 0.015, 'yaw': 0.02}
        
    def create_trajectory(self, start_pos, target_pos, duration):
        """Create base trajectory."""
        trajectory = PolynomialTrajectory(duration)
        trajectory.generate_trajectory(start_pos, target_pos)
        return trajectory
    
    def calculate_deviation(self, planned_pos, actual_pos, planned_yaw=None, actual_yaw=None):
        """Calculate position and angle deviation."""
        if isinstance(planned_pos, (list, tuple)) and len(planned_pos) >= 3:
            pos_deviation = math.sqrt(sum((a - p)**2 for a, p in zip(actual_pos[:3], planned_pos[:3])))
        else:
            pos_deviation = abs(actual_pos - planned_pos)
        
        yaw_deviation = abs(self._normalize_angle(actual_yaw - planned_yaw)) if planned_yaw and actual_yaw else 0
        return pos_deviation, yaw_deviation
    
    def needs_replanning(self, pos_deviation, yaw_deviation, stage_name=""):
        """Determine if replanning is needed."""
        return (pos_deviation > self.precision_thresholds['position'] or 
                yaw_deviation > self.precision_thresholds['yaw'])
    
    def replan_from_current(self, current_pos, current_yaw, original_target_pos, original_target_yaw, duration):
        """Replan from current position to target position."""
        distance = math.sqrt(sum((t - c)**2 for t, c in zip(original_target_pos[:3], current_pos[:3])))
        adjusted_duration = max(duration * 0.3, distance / 0.1)
        new_trajectory = self.create_trajectory(current_pos, original_target_pos, adjusted_duration)
        return new_trajectory, adjusted_duration
    
    def replan_intermediate_target(self, current_pos, original_target, valve_center, intermediate_distance):
        """Recalculate intermediate target position."""
        direction_to_valve = math.atan2(valve_center[1] - current_pos[1], valve_center[0] - current_pos[0])
        
        intermediate_x = valve_center[0] - intermediate_distance * math.cos(direction_to_valve)
        intermediate_y = valve_center[1] - intermediate_distance * math.sin(direction_to_valve)
        intermediate_z = original_target[2]  # Maintain original height
        
        return (intermediate_x, intermediate_y, original_target[2])
    
    def set_precision_thresholds(self, position_threshold, yaw_threshold):
        """Set precision thresholds."""
        self.precision_thresholds['position'] = position_threshold
        self.precision_thresholds['yaw'] = yaw_threshold
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-π, π]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def create_z_only_trajectory(self, start_pos, target_z, duration):
        """Create Z-axis only trajectory."""
        target_pos = (start_pos[0], start_pos[1], target_z)
        trajectory = PolynomialTrajectory(duration)
        trajectory.generate_trajectory(start_pos, target_pos)
        return trajectory
    
    def calculate_optimal_z_segments(self, total_z_distance, max_segment_distance=0.18):
        """Calculate optimal Z-axis segmentation."""
        if total_z_distance <= max_segment_distance:
            return [total_z_distance]
        num_segments = math.ceil(total_z_distance / max_segment_distance)
        return [total_z_distance / num_segments] * num_segments
    
    def calculate_z_descent_speed(self, z_distance, min_speed=0.06, max_speed=0.10):
        """Calculate optimal Z descent speed."""
        if z_distance < 0.10:
            return max_speed
        elif z_distance < 0.20:
            return 0.08
        return min_speed


class SelfRotationRevolutionTrajectory:
    """Self-rotation + revolution trajectory for valve rotation."""
    
    def __init__(self, valve_center, radius, rotation_angle, duration, 
                 start_pos, start_yaw, clockwise=True):
        self.valve_center = valve_center
        self.radius = radius
        self.rotation_angle = -rotation_angle if clockwise else rotation_angle
        self.duration = duration
        self.start_pos = start_pos
        self.start_yaw = start_yaw
        self.clockwise = clockwise
        
        dx = start_pos[0] - valve_center[0]
        dy = start_pos[1] - valve_center[1]
        self.start_angle = math.atan2(dy, dx)
        self.initial_center_facing_yaw = math.atan2(valve_center[1] - start_pos[1], valve_center[0] - start_pos[0])
    
    def get_position_and_yaw_at_progress(self, progress):
        """Calculate position and yaw at given progress [0.0, 1.0]."""
        progress = max(0.0, min(1.0, progress))
        
        current_angle = self.start_angle + progress * self.rotation_angle
        target_x = self.valve_center[0] + self.radius * math.cos(current_angle)
        target_y = self.valve_center[1] + self.radius * math.sin(current_angle)
        target_z = self.start_pos[2]
        target_yaw = self.start_yaw + progress * self.rotation_angle
        
        return ((target_x, target_y, target_z), target_yaw)
    
    def get_velocity_at_progress(self, progress):
        """Calculate linear and angular velocity at specified progress."""
        angular_velocity = abs(self.rotation_angle) / self.duration
        current_angle = self.start_angle + progress * self.rotation_angle
        tangential_angle = current_angle + math.pi/2 + (math.pi if self.rotation_angle < 0 else 0)
        
        tangential_speed = self.radius * angular_velocity
        vx = tangential_speed * math.cos(tangential_angle)
        vy = tangential_speed * math.sin(tangential_angle)
        yaw_velocity = angular_velocity * (1.0 if self.rotation_angle < 0 else -1.0)
        
        return ((vx, vy, 0.0), yaw_velocity)
    
    def apply_velocity_smoothing(self, progress, velocity, yaw_velocity):
        """Apply velocity smoothing at trajectory boundaries."""
        if progress < 0.1:
            smooth_factor = progress / 0.1
        elif progress > 0.9:
            smooth_factor = (1.0 - progress) / 0.1
        else:
            smooth_factor = 1.0
        
        return ((velocity[0] * smooth_factor, velocity[1] * smooth_factor, 0.0), 
                yaw_velocity * smooth_factor)
    
    def validate_trajectory(self, progress_points=10):
        """Validate trajectory validity."""
        errors = []
        if self.duration <= 0:
            errors.append("Duration must be positive")
        if self.radius <= 0:
            errors.append("Radius must be positive")
        if abs(self.rotation_angle) < math.radians(1):
            errors.append("Rotation angle too small")
        return len(errors) == 0, errors
    
    def get_trajectory_info(self):
        """Get trajectory information summary."""
        return {
            'type': 'self_rotation_revolution',
            'valve_center': self.valve_center,
            'radius': self.radius,
            'rotation_angle_deg': math.degrees(abs(self.rotation_angle)),
            'duration': self.duration,
            'clockwise': self.clockwise,
            'angular_velocity_deg_per_sec': math.degrees(abs(self.rotation_angle) / self.duration),
            'tangential_speed_m_per_sec': self.radius * abs(self.rotation_angle) / self.duration
        }


class ValveRotationTrajectoryManager:
    """Factory class for creating valve rotation trajectories."""
    
    @staticmethod
    def create_self_rotation_revolution_trajectory(valve_center, radius, rotation_angle_deg, 
                                                  duration, start_pos, start_yaw, clockwise=True):
        """Create self-rotation + revolution trajectory."""
        return SelfRotationRevolutionTrajectory(
            valve_center=valve_center, radius=radius,
            rotation_angle=math.radians(rotation_angle_deg),
            duration=duration, start_pos=start_pos, start_yaw=start_yaw, clockwise=clockwise
        )
    
    @staticmethod
    def create_legacy_trajectory(valve_center, radius, rotation_angle_deg, duration, start_angle):
        """Create legacy trajectory (revolution only, always facing center)."""
        return ConstantDistanceValveRotationTrajectory(
            valve_center=valve_center, end_effector_distance=radius,
            rotation_angle=math.radians(rotation_angle_deg),
            rotation_duration=duration, start_angle=start_angle
        )


class OnlineCircularTrajectoryGenerator:
    """
    Online circular trajectory generator for Beetle tilt-rotor UAV.
    
    Features:
    - Online trajectory replanning with constant angular velocity
    - Radius adaptation and locking mechanism
    - Centripetal force compensation and torque feedforward
    """
    
    def __init__(self, valve_center, initial_radius, target_angular_velocity=0.5, 
                 control_rate=15.0, debug=False):
        """
        Initialize online circular trajectory generator.
        
        Args:
            valve_center: Valve center position [x, y, z]
            initial_radius: Initial radius (m)
            target_angular_velocity: Target angular velocity (rad/s)
            control_rate: Control frequency (Hz)
            debug: Debug mode
        """
        self.valve_center = np.array(valve_center)
        self.initial_radius = initial_radius
        self.target_angular_velocity = target_angular_velocity
        self.control_rate = control_rate
        self.debug = debug
        self.dt = 1.0 / control_rate
        
        # State variables
        self.current_angle = 0.0
        self.current_radius = initial_radius
        self.radius_locked = False
        self.total_rotation = 0.0
        
        # Trajectory parameters
        self.angular_velocity = 0.0
        self.radius_adaptation_rate = 0.1
        self.min_radius = initial_radius * 0.8
        self.max_radius = initial_radius * 1.2
        
        # Force control parameters
        self.mass = 1.5  # kg
        self.init_torque = 0.1  # N·m
        self.torque_limit = 3.0  # N·m
        self.current_torque = self.init_torque
        
        # Z coordinate management
        self.current_z = None
        
        # Radius locking mechanism
        self.velocity_threshold_for_lock = 0.8
        self.lock_radius_value = None
        
        # Performance monitoring
        self.start_time = rospy.Time.now().to_sec()
        self.last_update_time = self.start_time
    
    def update_state(self, current_pos, current_yaw, valve_angular_velocity=0.0):
        """
        Update trajectory state using saturated cumulative integration.
        
        Strategy: Accumulate target angle like old version for push force,
        but clamp the error between target and actual to prevent excessive force.
        
        Args:
            current_pos: Current UAV position [x, y, z]
            current_yaw: Current UAV yaw angle (rad)
            valve_angular_velocity: Valve angular velocity (rad/s)
            
        Returns:
            dict: Updated state info
        """
        current_time = rospy.Time.now().to_sec()
        actual_dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Limit time step
        MAX_DT_LIMIT = 0.08
        safe_dt = min(actual_dt, MAX_DT_LIMIT)
        
        # Calculate actual angle and radius from current position
        relative_pos = np.array(current_pos[:2]) - self.valve_center[:2]
        actual_radius = np.linalg.norm(relative_pos)
        actual_angle = math.atan2(relative_pos[1], relative_pos[0])
        
        # Always use target angular velocity
        self.angular_velocity = self.target_angular_velocity
        
        # Speed safety limit (0.15 rad/s ≈ 8.6°/s max)
        SPEED_SAFETY_LIMIT = 0.15
        if abs(self.angular_velocity) > SPEED_SAFETY_LIMIT:
            self.angular_velocity = SPEED_SAFETY_LIMIT * (1 if self.angular_velocity > 0 else -1)
        
        # SATURATED CUMULATIVE INTEGRATION
        # Step 1: Accumulate target angle (like old version, provides push force)
        delta_angle = self.angular_velocity * safe_dt
        self.current_angle += delta_angle
        
        # Step 2: Calculate error between target and actual
        angle_error = self.current_angle - actual_angle
        # Handle angle wrapping
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Step 3: ADAPTIVE error saturation based on valve resistance
        # If valve is slow/stuck, allow larger error to generate more push force
        BASE_MAX_ERROR = 0.2    # ~11.5° normal operation
        HIGH_MAX_ERROR = 0.35   # ~20° when valve is stuck (increased push force)
        WARMUP_ROTATION = 0.7   # ~40° warmup phase (before contact with valve)
        
        # During warmup phase (first 40° of rotation), use conservative error limit
        # This prevents excessive initial speed before UAV contacts the valve
        if self.total_rotation < WARMUP_ROTATION:
            adaptive_max_error = BASE_MAX_ERROR
        elif valve_angular_velocity < 0.02:  # Valve nearly stalled - maximum push
            adaptive_max_error = HIGH_MAX_ERROR
        elif valve_angular_velocity < 0.05:  # Valve moving slowly - increased push
            # Linear interpolation between HIGH and BASE
            t = (valve_angular_velocity - 0.02) / (0.05 - 0.02)
            adaptive_max_error = HIGH_MAX_ERROR - t * (HIGH_MAX_ERROR - BASE_MAX_ERROR)
        else:  # Valve moving normally
            adaptive_max_error = BASE_MAX_ERROR
        
        if abs(angle_error) > adaptive_max_error:
            # Clamp target to actual + max_error
            sign = 1 if angle_error > 0 else -1
            self.current_angle = actual_angle + sign * adaptive_max_error
        
        # Track total rotation using actual angle changes (handles ±π wrap)
        if not hasattr(self, '_last_actual_angle'):
            self._last_actual_angle = actual_angle
        
        angle_change = actual_angle - self._last_actual_angle
        # Handle angle wrapping at ±π
        if angle_change > math.pi:
            angle_change -= 2 * math.pi
        elif angle_change < -math.pi:
            angle_change += 2 * math.pi
        
        self.total_rotation += abs(angle_change)
        self._last_actual_angle = actual_angle
        
        # Radius adaptation
        if not self.radius_locked:
            radius_error = actual_radius - self.current_radius
            self.current_radius += radius_error * self.radius_adaptation_rate
            self.current_radius = max(self.min_radius, min(self.max_radius, self.current_radius))
            
            velocity_ratio = abs(self.angular_velocity) / abs(self.target_angular_velocity) if self.target_angular_velocity != 0 else 0
            if velocity_ratio >= self.velocity_threshold_for_lock:
                self.radius_locked = True
                self.lock_radius_value = self.current_radius
        else:
            self.current_radius = self.lock_radius_value
        
        # Torque adaptation - increase torque when valve is slow (high resistance)
        # When valve_angular_velocity is low, we need MORE torque, not less
        if valve_angular_velocity < 0.02:  # Valve nearly stalled
            self.current_torque = self.torque_limit  # Maximum torque
        elif valve_angular_velocity < 0.1:  # Valve moving slowly
            # Linear interpolation: slower valve = more torque
            resistance_factor = 1.5 + (0.1 - valve_angular_velocity) / 0.08 * 0.5
            self.current_torque = min(self.torque_limit, self.init_torque * resistance_factor)
        else:
            self.current_torque = self.init_torque  # Normal torque
        
        return {
            'current_angle': self.current_angle, 'current_radius': self.current_radius,
            'angular_velocity': self.angular_velocity, 'radius_locked': self.radius_locked,
            'total_rotation': self.total_rotation, 'current_torque': self.current_torque,
            'actual_radius': actual_radius, 'actual_angle': actual_angle
        }
    
    def generate_target_state(self):
        """Generate target state for control."""
        target_x = self.valve_center[0] + self.current_radius * math.cos(self.current_angle)
        target_y = self.valve_center[1] + self.current_radius * math.sin(self.current_angle)
        target_z = self.current_z if self.current_z is not None else self.valve_center[2]
        target_pos = np.array([target_x, target_y, target_z])
        
        target_yaw = self.current_angle + math.pi
        if target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        
        # Tangential velocity
        tangential_speed = self.current_radius * self.angular_velocity
        tangential_angle = self.current_angle + math.pi/2
        target_linear_vel = np.array([tangential_speed * math.cos(tangential_angle),
                                      tangential_speed * math.sin(tangential_angle), 0.0])
        
        # Centripetal force
        centripetal_force = self.mass * tangential_speed**2 / self.current_radius if self.current_radius > 0 else 0
        target_force = np.array([-centripetal_force * math.cos(self.current_angle),
                                 -centripetal_force * math.sin(self.current_angle),
                                 self.mass * 9.81 * 0.1])
        
        return {
            'position': target_pos, 'yaw': target_yaw,
            'linear_velocity': target_linear_vel, 'angular_velocity': self.angular_velocity,
            'force': target_force, 'torque': np.array([0.0, 0.0, self.current_torque]),
            'radius': self.current_radius, 'angle': self.current_angle
        }
    
    def is_motion_completed(self, target_rotation_angle):
        """Check if motion is complete."""
        return self.total_rotation >= abs(target_rotation_angle)
    
    def get_progress(self, target_rotation_angle):
        """Get motion progress [0.0, 1.0]."""
        return min(1.0, self.total_rotation / abs(target_rotation_angle))
    
    def reset(self, new_valve_center=None, new_radius=None):
        """Reset trajectory generator."""
        if new_valve_center is not None:
            self.valve_center = np.array(new_valve_center)
        if new_radius is not None:
            self.initial_radius = new_radius
            self.current_radius = new_radius
            
        self.current_angle = 0.0
        self.total_rotation = 0.0
        self.angular_velocity = 0.0
        self.radius_locked = False
        self.lock_radius_value = None
        self.current_torque = self.init_torque
    
    def get_status_info(self):
        """Get status info for debugging."""
        runtime = rospy.Time.now().to_sec() - self.start_time
        return {
            'runtime': runtime,
            'total_rotation_deg': math.degrees(self.total_rotation),
            'current_angular_velocity_deg': math.degrees(self.angular_velocity),
            'target_angular_velocity_deg': math.degrees(self.target_angular_velocity),
            'velocity_ratio': self.angular_velocity / self.target_angular_velocity if self.target_angular_velocity != 0 else 0,
            'radius_mm': self.current_radius * 1000,
            'radius_locked': self.radius_locked,
            'current_torque': self.current_torque
        }
    
    def initialize_from_current_position(self, current_pos):
        """Initialize trajectory generator from current UAV position."""
        relative_pos = np.array(current_pos[:2]) - self.valve_center[:2]
        self.current_angle = math.atan2(relative_pos[1], relative_pos[0])
        self.current_radius = np.linalg.norm(relative_pos)
        self.initial_radius = self.current_radius
        self.current_z = current_pos[2]
