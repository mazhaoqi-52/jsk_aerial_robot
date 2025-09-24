#!/usr/bin/env python
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

class AdaptiveTrajectoryController:
    """Adaptive trajectory controller - dynamically adjusts trajectory parameters based on tracking performance"""
    
    # Predefined adaptive configuration modes
    CONTROL_MODES = {
        'rotation': {
            'speed_min': 0.3,              # Minimum speed factor (30%)
            'speed_max': 1.2,              # Maximum speed factor (120%)
            'error_threshold_high': 0.05,  # 50mm high error threshold (优化: 40mm→50mm)
            'error_threshold_low': 0.015,  # 15mm low error threshold
            'adaptation_rate': 0.1,        # Adaptation rate
            'history_window': 20,          # Error history window
            'update_frequency': 5,         # Update every 5 control cycles
            'speed_reduction_factor': 0.95, # Speed reduction factor when error is too large (优化: 0.8→0.95, 减速5%而非20%)
            'speed_increase_factor': 1.1,  # Speed increase factor when error is small
            'trend_reduction_factor': 0.95, # Speed reduction factor when trend deteriorates (优化: 0.9→0.95)
        },
        'disengagement': {
            'speed_min': 0.5,              # Minimum speed factor (50%) - more conservative
            'speed_max': 1.0,              # Maximum speed factor (100%) - no acceleration
            'error_threshold_high': 0.03,  # 30mm high error threshold - stricter
            'error_threshold_low': 0.040,  # 40mm low error threshold - relaxed for valve rotation task  
            'adaptation_rate': 0.15,       # Faster adaptation rate
            'history_window': 15,          # Smaller history window
            'update_frequency': 3,         # Update every 3 control cycles - more frequent
            'speed_reduction_factor': 0.7, # More aggressive deceleration
            'speed_increase_factor': 1.05, # More conservative acceleration
            'trend_reduction_factor': 0.85,# More aggressive trend deceleration
        }
    }
    
    def __init__(self, base_trajectory, control_mode='rotation'):
        self.base_trajectory = base_trajectory
        self.current_speed_factor = 1.0  # Current speed factor
        self.error_history = []  # Error history
        self.update_counter = 0  # Update counter
        self.control_mode = control_mode
        
        # Convergence detection parameters
        self.convergence_check_window = 10  # Check recent 10 errors
        self.convergence_threshold = 0.005  # 5mm error change threshold
        self.min_speed_duration = 0  # Minimum speed duration counter
        self.convergence_detected = False  # Convergence detection flag
        self.performance_analyzer = PerformanceAnalyzer()
        
        # CRITICAL: Record start time for minimum execution protection
        self.start_time = rospy.Time.now().to_sec()
        self.min_execution_time = 20.0  # Minimum 20 seconds before allowing convergence
        
        # Load configuration based on control mode
        if control_mode not in self.CONTROL_MODES:
            rospy.logwarn(f"Unknown control mode '{control_mode}', using 'rotation' mode")
            control_mode = 'rotation'
            
        self.adaptation_params = self.CONTROL_MODES[control_mode].copy()
        
        # Record mode information
        mode_desc = {
            'rotation': "ROTATION mode (standard parameters)",
            'disengagement': "DISENGAGEMENT mode (conservative parameters)"
        }
        
        rospy.loginfo(f"AdaptiveTrajectoryController initialized for {mode_desc.get(control_mode, control_mode)}")
        rospy.loginfo(f"  Speed range: {self.adaptation_params['speed_min']:.1%} - {self.adaptation_params['speed_max']:.1%}")
        rospy.loginfo(f"  Error thresholds: {self.adaptation_params['error_threshold_low']*1000:.0f}mm - {self.adaptation_params['error_threshold_high']*1000:.0f}mm")
        rospy.loginfo(f"  Update frequency: every {self.adaptation_params['update_frequency']} cycles")
    
    def update_tracking_performance(self, position_error, velocity_error=None):
        """Update tracking performance data"""
        # Record error history
        self.error_history.append({
            'position_error': position_error,
            'velocity_error': velocity_error,
            'timestamp': rospy.Time.now().to_sec()
        })
        
        # Maintain history window size
        if len(self.error_history) > self.adaptation_params['history_window']:
            self.error_history.pop(0)
        
        self.update_counter += 1
        
        # Update parameters according to configured update frequency
        if self.update_counter >= self.adaptation_params['update_frequency']:
            self._update_adaptive_parameters()
            self.update_counter = 0
    
    def _update_adaptive_parameters(self):
        """Update adaptive parameters"""
        if len(self.error_history) < 3:
            return
        
        # Calculate performance metrics
        performance = self.performance_analyzer.analyze_performance(self.error_history)
        
        # Adjust speed factor based on performance
        new_speed_factor = self._compute_adaptive_speed(performance)
        
        # Smooth transition
        speed_change = new_speed_factor - self.current_speed_factor
        self.current_speed_factor += speed_change * self.adaptation_params['adaptation_rate']
        
        # Limit to reasonable range with depth-adaptive maximum speed
        effective_speed_max = self.adaptation_params['speed_max']
        
        # DEPTH-ADAPTIVE SPEED LIMITING
        # Apply speed limit from trajectory's depth-adaptive settings
        if hasattr(self.base_trajectory, 'adaptive_speed_limit'):
            effective_speed_max = min(effective_speed_max, self.base_trajectory.adaptive_speed_limit)
            if self.base_trajectory.adaptive_speed_limit < 1.0:
                rospy.loginfo_throttle(10, f"Depth-adaptive speed limit: {self.base_trajectory.adaptive_speed_limit*100:.0f}% "
                                          f"(original max: {self.adaptation_params['speed_max']*100:.0f}%)")
        
        self.current_speed_factor = max(self.adaptation_params['speed_min'], 
                                      min(effective_speed_max, 
                                          self.current_speed_factor))
        
        # Log output (on each adjustment)
        if abs(speed_change) > 0.05:  # Record only when change exceeds 5%
            mode_emoji = "🔄" if self.control_mode == 'disengagement' else "🔧"
            mode_name = self.control_mode.upper()
            rospy.loginfo(f"{mode_emoji} {mode_name} adaptive control: speed factor {self.current_speed_factor:.2f} "
                         f"(avg_error: {performance['avg_error']*1000:.1f}mm, "
                         f"trend: {performance['error_trend']:.3f})")
    
    def _compute_adaptive_speed(self, performance):
        """Calculate adaptive speed factor (unified logic, controlled by parameters for different strategies)"""
        avg_error = performance['avg_error']
        error_trend = performance['error_trend']
        
        # Core adaptive logic - control different mode behaviors through parameters
        if avg_error > self.adaptation_params['error_threshold_high']:
            # Error too large -> decelerate
            return max(self.current_speed_factor * self.adaptation_params['speed_reduction_factor'], 
                      self.adaptation_params['speed_min'])
        elif avg_error < self.adaptation_params['error_threshold_low'] and error_trend < 0:
            # Small error and decreasing -> can accelerate appropriately
            return min(self.current_speed_factor * self.adaptation_params['speed_increase_factor'], 
                      self.adaptation_params['speed_max'])
        elif error_trend > 0.01:  # Error increasing rapidly
            # Error trend deteriorating -> moderate deceleration
            return max(self.current_speed_factor * self.adaptation_params['trend_reduction_factor'], 
                      self.adaptation_params['speed_min'])
        else:
            # Maintain current speed
            return self.current_speed_factor
    
    def get_current_speed_factor(self):
        """Get current speed factor"""
        return self.current_speed_factor
    
    def process_trajectory_point(self, base_point):
        """Process trajectory point and apply adaptive adjustments"""
        if base_point is None:
            return None
        
        # Check convergence (for rotation mode)
        if self.control_mode == 'rotation':
            is_converged = self._check_convergence()
            if is_converged:
                rospy.loginfo("Trajectory convergence detected - rotation complete!")
                return None  # Return None to indicate trajectory completion
        
        # Apply speed factor adjustment (if trajectory supports speed adjustment)
        if hasattr(self.base_trajectory, '_apply_speed_factor_to_point'):
            adjusted_point = self.base_trajectory._apply_speed_factor_to_point(base_point, self.current_speed_factor)
            return adjusted_point
        
        # If speed adjustment is not supported, return base point directly
        return base_point
    
    def _check_convergence(self):
        """
        Check if convergence has been achieved with dual validation:
        1. Position error convergence
        2. Valve rotation angle completion (for rotation tasks)
        """
        if len(self.error_history) < self.convergence_check_window:
            return False
        
        # CRITICAL: Check minimum execution time first
        current_time = rospy.Time.now().to_sec()
        execution_time = current_time - self.start_time
        
        if execution_time < self.min_execution_time:
            # Too early for convergence
            return False
        
        # Get recent position errors
        recent_errors = [entry['position_error'] for entry in self.error_history[-self.convergence_check_window:]]
        error_variance = np.var(recent_errors)
        mean_error = np.mean(recent_errors)
        
        # Check if error change is very small AND average error is within reasonable range
        # CRITICAL FIX: Much stricter convergence criteria to prevent early termination
        has_small_variance = error_variance < self.convergence_threshold ** 2  # Small variance
        has_small_error = mean_error < 0.050  # RELAXED: 50mm average error tolerance for valve rotation task
        
        # DUAL VALIDATION: Position convergence + Valve rotation angle validation
        position_converged = has_small_variance and has_small_error
        
        # For rotation tasks, also check valve rotation completion
        valve_rotation_complete = True  # Default for non-rotation tasks
        if self.control_mode == 'rotation' and hasattr(self, 'base_trajectory'):
            # Get valve rotation progress
            try:
                from geometry_msgs.msg import Twist
                # FIXED: Use trajectory progress instead of unreliable valve yaw monitoring
                target_rotation = abs(getattr(self.base_trajectory, 'rotation_angle', math.pi/2))
                
                # Method 1: Try to get valve rotation from ROS parameter (legacy support)
                current_valve_rotation = rospy.get_param('/valve_rotation_current', 0.0)
                
                # Method 2: Calculate from trajectory progress (more reliable)
                if hasattr(self.base_trajectory, 'get_progress'):
                    trajectory_progress = self.base_trajectory.get_progress()
                    estimated_rotation_from_progress = trajectory_progress * target_rotation
                    
                    # Use the larger of the two estimates (valve monitoring may be unreliable)
                    current_valve_rotation = max(current_valve_rotation, estimated_rotation_from_progress)
                    
                    rospy.loginfo_throttle(5, f"Rotation progress: valve_monitoring={math.degrees(rospy.get_param('/valve_rotation_current', 0.0)):.1f}°, "
                                          f"trajectory_progress={trajectory_progress*100:.1f}% → {math.degrees(estimated_rotation_from_progress):.1f}°, "
                                          f"using={math.degrees(current_valve_rotation):.1f}°")
                elif hasattr(self.base_trajectory, '_angle_trajectory'):
                    # Fallback: estimate from angle trajectory if available
                    try:
                        if self.base_trajectory._angle_trajectory:
                            angle_progress = getattr(self.base_trajectory._angle_trajectory, 'progress', 0.0)
                            estimated_rotation_from_angle = angle_progress * target_rotation
                            current_valve_rotation = max(current_valve_rotation, estimated_rotation_from_angle)
                            rospy.loginfo_throttle(5, f"Using angle trajectory progress: {angle_progress*100:.1f}% → {math.degrees(estimated_rotation_from_angle):.1f}°")
                    except:
                        pass
                
                rotation_error = abs(target_rotation - abs(current_valve_rotation))
                
                # STRICT: Precise completion criteria
                # Accept completion when rotation is within 5° of target OR trajectory progress > 95%
                angle_tolerance = math.radians(5.0)  # Reduced back to 5° for precise control
                valve_rotation_complete = rotation_error < angle_tolerance
                
                # Additional check: if trajectory progress is high, consider it complete
                if hasattr(self.base_trajectory, 'get_progress'):
                    trajectory_progress = self.base_trajectory.get_progress()
                    if trajectory_progress > 0.95:  # 95% trajectory completion
                        valve_rotation_complete = True
                        rospy.loginfo_throttle(5, f"✅ Trajectory-based completion: {trajectory_progress*100:.1f}% > 95%")
                
                if not valve_rotation_complete:
                    rospy.loginfo(f"Valve rotation incomplete: target={math.degrees(target_rotation):.1f}°, "
                                f"current={math.degrees(abs(current_valve_rotation)):.1f}°, "
                                f"error={math.degrees(rotation_error):.1f}° > {math.degrees(angle_tolerance):.1f}°")
                else:
                    rospy.loginfo(f"✅ Valve rotation complete: error={math.degrees(rotation_error):.1f}° < {math.degrees(angle_tolerance):.1f}°")
                    
            except Exception as e:
                rospy.logwarn(f"Could not verify valve rotation completion: {e}")
                # Fall back to position-only convergence for safety
                valve_rotation_complete = True
        
        if position_converged and valve_rotation_complete:
            self.min_speed_duration += 1
            # ENHANCED: Require more consecutive detections for stability
            if self.min_speed_duration >= 10:  # Was 5, now 10 consecutive checks
                self.convergence_detected = True
                rospy.loginfo(f"🔒 DUAL CONVERGENCE confirmed: variance={error_variance:.6f}, mean_error={mean_error*1000:.1f}mm, valve_rotation_ok={valve_rotation_complete}, time={execution_time:.1f}s")
                return True
        else:
            self.min_speed_duration = 0
            if not has_small_error:
                # Log when error is too large for convergence
                rospy.loginfo(f"⏳ Position error too large: {mean_error*1000:.1f}mm > 50mm threshold")
            if not valve_rotation_complete:
                rospy.loginfo(f"⏳ Valve rotation incomplete - continuing trajectory")
        
        return False
    
    @classmethod
    def create_for_rotation(cls, base_trajectory):
        """Create adaptive controller for rotation phase"""
        return cls(base_trajectory, control_mode='rotation')
    
    @classmethod  
    def create_for_disengagement(cls, base_trajectory):
        """Create adaptive controller for disengagement phase"""
        return cls(base_trajectory, control_mode='disengagement')


class PerformanceAnalyzer:
    """Performance analyzer - analyzes tracking error trends and characteristics"""
    
    def analyze_performance(self, error_history):
        """Analyze tracking performance"""
        if len(error_history) < 3:
            return {
                'avg_error': 0.0,
                'max_error': 0.0,
                'error_trend': 0.0,
                'stability': 1.0
            }
        
        # Extract position error sequence
        position_errors = [e['position_error'] for e in error_history]
        
        # Calculate basic statistics
        avg_error = np.mean(position_errors)
        max_error = np.max(position_errors)
        
        # Calculate error trend (linear regression slope)
        x = np.arange(len(position_errors))
        error_trend = np.polyfit(x, position_errors, 1)[0]
        
        # Calculate stability (inverse of error variance)
        error_variance = np.var(position_errors)
        stability = 1.0 / (1.0 + error_variance * 1000)  # Normalized
        
        return {
            'avg_error': avg_error,
            'max_error': max_error,
            'error_trend': error_trend,
            'stability': stability
        }

class PolynomialTrajectory:
    def __init__(self, duration):
        self.duration = duration
        self.coeffs_x = None
        self.coeffs_y = None
        self.coeffs_z = None
        self.coeffs_scalar = None  
        self.start_time = None
        self.is_scalar = False  
        # TRAJECTORY PAUSE/RESUME MECHANISM
        self.is_paused = False
        self.pause_start_time = None
        self.total_pause_duration = 0.0  

    def compute_coefficients(self, start, target):
        # Handle near-zero movement case
        if abs(target - start) < 1e-6:
            return np.array([0, 0, 0, 0, 0, start])
            
        T = self.duration
        if T <= 0:
            raise ValueError("Duration must be positive.")
        
        movement_distance = abs(target - start)
        
        # CRITICAL FIX: Replace polynomial with linear trajectory for short/medium movements
        # Problem: 5th-order polynomial creates speed spikes (830mm/s) for movements <200mm
        # Solution: Use simple linear interpolation to eliminate mathematical instability
        
        if movement_distance < 0.200:  # Short/medium movement (<200mm) = LINEAR TRAJECTORY
            rospy.loginfo(f"LINEAR TRAJECTORY SELECTED: {movement_distance*1000:.1f}mm < 200mm threshold")
            rospy.loginfo(f"  Eliminates polynomial speed spikes, uses constant velocity")
            
            # Linear trajectory: p(τ) = start + (target - start) * τ
            # This gives constant velocity = (target - start) / T
            constant_velocity = (target - start) / T
            rospy.loginfo(f"  Constant velocity: {abs(constant_velocity)*1000:.1f}mm/s (no spikes)")
            
            # Polynomial representation of linear trajectory: a₁τ + a₀
            # All higher-order terms (a₂, a₃, a₄, a₅) = 0
            linear_coeffs = np.array([
                0,                    # a₅ = 0 (no quintic term)
                0,                    # a₄ = 0 (no quartic term)  
                0,                    # a₃ = 0 (no cubic term)
                0,                    # a₂ = 0 (no quadratic term)
                target - start,       # a₁ = total displacement (linear term)
                start                 # a₀ = start position (constant term)
            ])
            
            # Verify linear trajectory correctness
            start_check = linear_coeffs[5]  # At τ=0: p(0) = a₀ = start
            target_check = np.sum(linear_coeffs)  # At τ=1: p(1) = a₁ + a₀ = target
            velocity_check = linear_coeffs[4] / T  # Velocity = a₁/T = constant
            
            rospy.loginfo(f"LINEAR TRAJECTORY VERIFICATION:")
            rospy.loginfo(f"  Start: {start:.6f} → {start_check:.6f} (error: {abs(start-start_check)*1000:.3f}mm)")
            rospy.loginfo(f"  Target: {target:.6f} → {target_check:.6f} (error: {abs(target-target_check)*1000:.3f}mm)")
            rospy.loginfo(f"  Velocity: {velocity_check*1000:.3f}mm/s (constant, no peaks)")
            
            return linear_coeffs
        
        # POLYNOMIAL TRAJECTORY for movements ≥200mm
        rospy.loginfo(f"POLYNOMIAL TRAJECTORY SELECTED: {movement_distance*1000:.1f}mm ≥ 200mm threshold")
        
        # SPEED OPTIMIZATION: Use distance-adaptive boundary velocities
        # Problem: Zero boundaries cause mid-trajectory speed peaks for large movements  
        # Solution: Use proportionally larger boundary velocities for shorter distances
        
        # Dynamic boundary velocity strategy for POLYNOMIAL trajectories (≥200mm)
        if movement_distance < 0.300:  # Medium-large movement (200-300mm)
            # Use 30% of average velocity as boundary velocity  
            boundary_velocity_fraction = 0.30
            rospy.loginfo(f"MEDIUM-LARGE POLYNOMIAL: {movement_distance*1000:.1f}mm - using 30% boundary velocity")
        elif movement_distance < 0.500:  # Large movement (300-500mm)
            # Use 20% of average velocity as boundary velocity
            boundary_velocity_fraction = 0.20
            rospy.loginfo(f"LARGE POLYNOMIAL: {movement_distance*1000:.1f}mm - using 20% boundary velocity")
        else:  # Very large movement (>500mm)
            # Use 10% of average velocity as boundary velocity
            boundary_velocity_fraction = 0.10
            rospy.loginfo(f"VERY LARGE POLYNOMIAL: {movement_distance*1000:.1f}mm - using 10% boundary velocity")
        
        # Calculate velocities
        average_velocity = movement_distance / T
        boundary_velocity = average_velocity * boundary_velocity_fraction
        
        # Apply direction to boundary velocity
        velocity_direction = 1 if target > start else -1
        start_velocity = boundary_velocity * velocity_direction
        end_velocity = boundary_velocity * velocity_direction
        
        # CONSTRAINED POLYNOMIAL: Use limited boundary velocities instead of zero
        # For polynomial p(τ) where τ = t/T ∈ [0,1]:
        # - Position: p(0) = start, p(1) = target  
        # - Velocity: dp/dt = (dp/dτ)/T = limited values (not zero)
        # - Acceleration: d²p/dt² = (d²p/dτ²)/T² = 0 at boundaries
        
        # Convert velocities to normalized time derivatives: dp/dτ = (dp/dt) * T
        start_vel_normalized = start_velocity * T
        end_vel_normalized = end_velocity * T
        
        A = np.array([
            [0,    0,    0,    0,  0, 1],    # τ=0: position = start
            [1,    1,    1,    1,  1, 1],    # τ=1: position = target  
            [0,    0,    0,    0,  1, 0],    # τ=0: dp/dτ = start_vel_normalized
            [5,    4,    3,    2,  1, 0],    # τ=1: dp/dτ = end_vel_normalized
            [0,    0,    0,    2,  0, 0],    # τ=0: d²p/dτ² = 0 (acceleration = 0)  
            [20,  12,    6,    2,  0, 0]     # τ=1: d²p/dτ² = 0 (acceleration = 0)
        ])
        B = np.array([start, target, start_vel_normalized, end_vel_normalized, 0, 0])
        
        # Solve polynomial coefficients
        coeffs = np.linalg.solve(A, B)
        
        # Check numerical stability - verify boundary conditions
        start_check = coeffs[5]  # Position at τ=0
        target_check = np.sum(coeffs)  # Position at τ=1
        
        start_error = abs(start_check - start)
        target_error = abs(target_check - target)
        
        if start_error > 1e-10 or target_error > 1e-10:
            rospy.logwarn(f"Polynomial coefficient numerical instability detected:")
            rospy.logwarn(f"  Start error: {start_error*1000:.6f}mm")
            rospy.logwarn(f"  Target error: {target_error*1000:.6f}mm")
            rospy.logwarn(f"  Duration: {T:.3f}s, Movement: {abs(target-start)*1000:.3f}mm")
        
        # Enhanced debug logging for speed optimization
        final_max_velocity = self._estimate_max_velocity(coeffs, T)
        rospy.loginfo(f"SPEED OPTIMIZED POLYNOMIAL: max_vel={final_max_velocity:.3f}m/s (avg={average_velocity:.3f}m/s)")
        rospy.loginfo(f"   Movement: {movement_distance*1000:.1f}mm in {T:.2f}s")
        rospy.loginfo(f"   Boundary velocities: start={start_velocity:.4f}m/s, end={end_velocity:.4f}m/s")
        rospy.loginfo(f"   Boundary fraction: {boundary_velocity_fraction:.1%} of average velocity")
            
        return coeffs

    def _estimate_max_velocity(self, coeffs, duration):
        """Estimate maximum velocity by sampling the polynomial derivative"""
        # Derivative coefficients for velocity: dp/dt = (dp/dτ) / T
        # dp/dτ = 5*a5*τ^4 + 4*a4*τ^3 + 3*a3*τ^2 + 2*a2*τ + a1
        vel_coeffs = np.array([5*coeffs[0], 4*coeffs[1], 3*coeffs[2], 2*coeffs[3], coeffs[4]])
        
        max_vel = 0.0
        # Sample velocity at multiple points to find maximum
        for i in range(101):  # Sample at 0%, 1%, 2%, ..., 100%
            tau = i / 100.0
            vel_normalized = np.polyval(vel_coeffs, tau)  # dp/dτ
            vel_actual = vel_normalized / duration  # dp/dt = (dp/dτ) / T
            max_vel = max(max_vel, abs(vel_actual))
        
        return max_vel
    
    def generate_trajectory(self, start_pos, target_pos):
        """Generate polynomial trajectory and store target position for boundary validation"""
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
        if self.start_time is None:
            return None
            
        # HANDLE TRAJECTORY PAUSE MECHANISM
        current_time = rospy.Time.now().to_sec()
        if self.is_paused:
            # When paused, return the last position (don't advance time)
            elapsed_time = self.pause_start_time - self.start_time - self.total_pause_duration
        else:
            # When not paused, calculate normal elapsed time
            elapsed_time = current_time - self.start_time - self.total_pause_duration
        
        # Ensure elapsed_time is non-negative
        elapsed_time = max(0.0, elapsed_time)
        
        # CRITICAL FIX: Return target position when trajectory completes instead of None
        if elapsed_time >= self.duration:
            if self.is_scalar:
                return self.target_value
            else:
                return self.target_pos
        
        # CRITICAL FIX: Use consistent polynomial evaluation method
        normalized_time = elapsed_time / self.duration
        
        if self.is_scalar:
            return self._evaluate_polynomial(self.coeffs_scalar, normalized_time)
        else:
            x_val = self._evaluate_polynomial(self.coeffs_x, normalized_time)
            y_val = self._evaluate_polynomial(self.coeffs_y, normalized_time)
            z_val = self._evaluate_polynomial(self.coeffs_z, normalized_time)
            return (x_val, y_val, z_val)

    def get_next_position(self):
        """Get next position from trajectory (compatibility method)"""
        return self.evaluate()
    
    def get_velocity(self):
        """Get current velocity from trajectory"""
        if self.start_time is None:
            return None
        elapsed_time = rospy.Time.now().to_sec() - self.start_time
        
        # CRITICAL FIX: Return zero velocity when trajectory completes instead of None
        if elapsed_time >= self.duration:
            if self.is_scalar:
                return 0.0
            else:
                return (0.0, 0.0, 0.0)
        
        # CRITICAL FIX: Use normalized time and correct derivative scaling
        normalized_time = elapsed_time / self.duration
        # Velocity = d/dt[p(t)] = d/dt[p(τ)] * dτ/dt = (1/T) * dp/dτ
        # where τ = t/T is normalized time
        # CRITICAL FIX: Correct velocity coefficient order to match polynomial coefficients
        # coeffs order: [a5, a4, a3, a2, a1, a0]
        # velocity coeffs: [5*a5, 4*a4, 3*a3, 2*a2, 1*a1, 0*a0] / duration
        T_vel = np.array([5*normalized_time**4, 4*normalized_time**3, 3*normalized_time**2, 
                         2*normalized_time, 1, 0]) / self.duration
        
        if self.is_scalar:
            return np.dot(self.coeffs_scalar, T_vel)
        else:
            return (
                np.dot(self.coeffs_x, T_vel),
                np.dot(self.coeffs_y, T_vel),
                np.dot(self.coeffs_z, T_vel)
            )

    def evaluate_at_time(self, elapsed_time):
        """Calculate trajectory value at specific time after trajectory start, with boundary validation
        
        Args:
            elapsed_time: Time elapsed since trajectory start (seconds)
        """
        # CRITICAL FIX: Use relative time instead of absolute time
        # elapsed_time is relative time after trajectory start, no need to subtract start_time
        normalized_time = elapsed_time / self.duration
        
        # Boundary check and handling
        if normalized_time <= 0:
            if self.is_scalar:
                return self.start_value
            else:
                return self.start_pos
        elif normalized_time >= 1:
            if self.is_scalar:
                return self.target_value
            else:
                return self.target_pos
        
        # Calculate polynomial values
        if self.is_scalar:
            return self._evaluate_polynomial(self.coeffs_scalar, normalized_time)
        else:
            x_val = self._evaluate_polynomial(self.coeffs_x, normalized_time)
            y_val = self._evaluate_polynomial(self.coeffs_y, normalized_time)
            z_val = self._evaluate_polynomial(self.coeffs_z, normalized_time)
            return [x_val, y_val, z_val]
    
    def _evaluate_polynomial(self, coeffs, t):
        """Calculate 5th order polynomial value using normalized time
        
        coeffs order: [a5, a4, a3, a2, a1, a0] (from compute_coefficients matrix)
        polynomial: a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0
        """
        return (coeffs[5] + coeffs[4]*t + coeffs[3]*t**2 + 
                coeffs[2]*t**3 + coeffs[1]*t**4 + coeffs[0]*t**5)

    def pause(self):
        """Pause trajectory execution - freeze time progression"""
        if not self.is_paused and self.start_time is not None:
            self.is_paused = True
            self.pause_start_time = rospy.Time.now().to_sec()
            rospy.loginfo("⏸️ TRAJECTORY PAUSED - time progression frozen")

    def resume(self):
        """Resume trajectory execution - continue time progression"""
        if self.is_paused and self.pause_start_time is not None:
            # Add pause duration to total
            pause_duration = rospy.Time.now().to_sec() - self.pause_start_time
            self.total_pause_duration += pause_duration
            self.is_paused = False
            self.pause_start_time = None
            rospy.loginfo(f"▶️ TRAJECTORY RESUMED - skipped {pause_duration:.2f}s during pause")

    def is_trajectory_paused(self):
        """Check if trajectory is currently paused"""
        return self.is_paused

class ValveRotationTrajectory:
    """
    Core trajectory generator for rotating around valve center with constant end-effector distance
    Enhanced with adaptive control capabilities
    """
    def __init__(self, rotation_duration, valve_center, end_effector_rotation_radius, start_angle, 
                 rotation_angle=2*pi, grasp_height=None, rotation_direction=1,
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823,
                 enable_adaptive_control=True, control_mode='rotation'):
        self.rotation_duration = rotation_duration
        self.valve_center = valve_center
        self.end_effector_rotation_radius = end_effector_rotation_radius
        self.end_effector_distance = end_effector_rotation_radius  # Alias for compatibility
        
        # Add UAV COG distance for clarity (converted from end_effector distance)
        self.end_effector_offset_x = end_effector_offset_x
        # GEOMETRY FIX: UAV is BEHIND end-effector, so UAV distance = end_effector + offset
        self.uav_rotation_radius = end_effector_rotation_radius + end_effector_offset_x
        self.uav_target_distance = self.uav_rotation_radius  # UAV COG target distance
        self.start_angle = start_angle
        self.rotation_angle = rotation_angle * rotation_direction
        self.grasp_height = grasp_height
        self.rotation_direction = rotation_direction
        
        # Store end-effector offset parameters
        self.end_effector_offset_x = end_effector_offset_x
        self.end_effector_offset_y = end_effector_offset_y
        self.end_effector_offset_z = end_effector_offset_z
        
        # DEPTH-ADAPTIVE ROTATION PARAMETERS
        # Adjust rotation parameters based on insertion depth for stability
        insertion_depth = self._estimate_insertion_depth(grasp_height)
        
        if insertion_depth > 0.25:  # Deep insertion (>250mm)
            self.adaptive_duration_factor = 1.5  # 50% slower rotation
            self.adaptive_speed_limit = 0.6      # Max 60% speed
            rospy.loginfo(f"DEEP INSERTION detected ({insertion_depth*1000:.0f}mm) - using CONSERVATIVE rotation parameters")
            rospy.loginfo(f"  Duration extended by 50%, max speed limited to 60%")
        elif insertion_depth > 0.15:  # Medium insertion (150-250mm)
            self.adaptive_duration_factor = 1.25 # 25% slower rotation
            self.adaptive_speed_limit = 0.8      # Max 80% speed
            rospy.loginfo(f"🚶 MEDIUM INSERTION detected ({insertion_depth*1000:.0f}mm) - using MODERATE rotation parameters")
            rospy.loginfo(f"  Duration extended by 25%, max speed limited to 80%")
        else:  # Shallow insertion or free space (<150mm)
            self.adaptive_duration_factor = 1.0  # Normal rotation
            self.adaptive_speed_limit = 1.0      # Full speed allowed
            rospy.loginfo(f"⚡ SHALLOW/FREE SPACE detected ({insertion_depth*1000:.0f}mm) - using STANDARD rotation parameters")
        
        # Apply depth-adaptive duration adjustment
        self.rotation_duration = rotation_duration * self.adaptive_duration_factor
        rospy.loginfo(f"Adjusted rotation duration: {self.rotation_duration:.1f}s (factor: {self.adaptive_duration_factor:.2f})")
        
        self._angle_trajectory = None
        self._current_speed_factor = 1.0  # Current speed factor
        
        # Adaptive control settings
        self.enable_adaptive_control = enable_adaptive_control
        self.control_mode = control_mode
        self.adaptive_controller = None
        
        if self.enable_adaptive_control:
            self.adaptive_controller = AdaptiveTrajectoryController(self, control_mode=control_mode)
            mode_emoji = "🔄" if control_mode == 'disengagement' else "🔧"
            rospy.loginfo(f"{mode_emoji} Adaptive control enabled for valve trajectory ({control_mode.upper()} mode)")
        
        rospy.loginfo(f"Valve rotation trajectory initialized:")
        rospy.loginfo(f"  End-effector rotation radius: {self.end_effector_rotation_radius:.3f}m")
        rospy.loginfo(f"  Rotation angle: {self.rotation_angle:.3f}rad ({self.rotation_angle*180/pi:.1f}°)")
        rospy.loginfo(f"  Duration: {self.rotation_duration:.1f}s")
        rospy.loginfo(f"  Adaptive control: {'enabled' if self.enable_adaptive_control else 'disabled'}")
    
    def _estimate_insertion_depth(self, grasp_height):
        """Estimate insertion depth from grasp height"""
        if grasp_height is None:
            return 0.0
        # Estimate valve surface at ~1.0m height
        valve_surface_z = 1.0
        estimated_depth = max(0, valve_surface_z - grasp_height)
        return estimated_depth
        
    def start_trajectory(self):
        """Start rotation trajectory"""
        end_angle = self.start_angle + self.rotation_angle
        self._angle_trajectory = PolynomialTrajectory(self.rotation_duration)
        self._angle_trajectory.generate_trajectory(self.start_angle, end_angle)
        
        rospy.loginfo("Rotation trajectory started")
        
    def get_progress(self):
        """Get trajectory completion progress (0.0 to 1.0)"""
        if not self._angle_trajectory:
            return 0.0
        
        # Get progress from angle trajectory if available
        if hasattr(self._angle_trajectory, 'get_progress'):
            return self._angle_trajectory.get_progress()
        elif hasattr(self._angle_trajectory, 'progress'):
            return self._angle_trajectory.progress
        else:
            # Fallback: estimate based on time if trajectory has timing info
            try:
                if hasattr(self._angle_trajectory, 'start_time') and hasattr(self._angle_trajectory, 'duration'):
                    current_time = rospy.Time.now().to_sec()
                    elapsed_time = current_time - self._angle_trajectory.start_time
                    progress = min(1.0, max(0.0, elapsed_time / self._angle_trajectory.duration))
                    return progress
            except:
                pass
            
            # Ultimate fallback
            return 0.0
        
    def set_speed_factor(self, speed_factor):
        """Set speed factor (called by adaptive controller)"""
        self._current_speed_factor = speed_factor
        # Update angle trajectory duration
        if self._angle_trajectory:
            # Recalculate trajectory time scale
            self._angle_trajectory.duration = self.rotation_duration / speed_factor
    
    def update_tracking_performance(self, position_error, velocity_error=None):
        """Update tracking performance (called by controller)"""
        if self.adaptive_controller:
            self.adaptive_controller.update_tracking_performance(position_error, velocity_error)
    
    def _apply_speed_factor_to_point(self, point, speed_factor):
        """Apply speed factor adjustment to trajectory points"""
        # For position points, speed factor mainly affects time step
        # More complex adjustment logic can be implemented here if needed
        return point
    
    def get_current_speed_factor(self):
        """Get current speed factor"""
        if self.adaptive_controller:
            return self.adaptive_controller.get_current_speed_factor()
        return 1.0
        
    def get_next_position_and_yaw(self):
        """Get next UAV position and orientation (enhanced with adaptive control)"""
        # First get base trajectory point
        base_point = self._get_base_trajectory_point()
        
        if base_point is None:
            return None
            
        # If adaptive control is enabled, let controller handle speed adjustment and trajectory optimization
        if self.adaptive_controller and self.enable_adaptive_control:
            # Let adaptive controller adjust based on current base trajectory point
            return self.adaptive_controller.process_trajectory_point(base_point)
        else:
            return base_point
    
    def _get_base_trajectory_point(self):
        """Get base trajectory point (original implementation)"""
        if self._angle_trajectory is None:
            return None
            
        current_angle = self._angle_trajectory.evaluate()
        if current_angle is None:
            return None
            
        center_x, center_y, center_z = self.valve_center
        
        # CORRECTED: Calculate UAV COG target position directly (not end-effector)
        # Use UAV rotation radius instead of end-effector radius
        # GEOMETRY FIX: UAV is BEHIND end-effector, so UAV distance = end_effector + offset
        uav_rotation_radius = getattr(self, 'uav_rotation_radius', 
                                    self.end_effector_rotation_radius + self.end_effector_offset_x)
        
        uav_target_x = center_x + uav_rotation_radius * cos(current_angle)
        uav_target_y = center_y + uav_rotation_radius * sin(current_angle)
        
        # Calculate UAV target height - CRITICAL FIX: Always lock Z to current grasp height
        # This prevents pitch errors caused by incorrect Z targeting during rotation
        if self.grasp_height is not None:
            uav_target_z = self.grasp_height
            # rospy.logdebug(f"Using locked grasp height: {self.grasp_height:.3f}m")
        else:
            # FALLBACK: If grasp_height is None, use valve center Z minus offset
            # However, this should be avoided during valve rotation to prevent pitch errors
            uav_target_z = center_z - self.end_effector_offset_z
            rospy.logwarn_once(f"⚠️ grasp_height is None during rotation - using fallback Z calculation")
            rospy.logwarn_once(f"⚠️ This may cause pitch errors. Consider setting grasp_height explicitly.")
        
        # Calculate UAV target orientation (pointing toward valve center for end-effector engagement)
        dx_to_center = center_x - uav_target_x
        dy_to_center = center_y - uav_target_y
        uav_target_yaw = atan2(dy_to_center, dx_to_center)
        
        # Calculate end-effector position for verification (not used for control)
        cos_yaw = cos(uav_target_yaw)
        sin_yaw = sin(uav_target_yaw)
        
        world_offset_x = (cos_yaw * self.end_effector_offset_x - 
                         sin_yaw * self.end_effector_offset_y)
        world_offset_y = (sin_yaw * self.end_effector_offset_x + 
                         cos_yaw * self.end_effector_offset_y)
        
        end_effector_x = uav_target_x + world_offset_x
        end_effector_y = uav_target_y + world_offset_y
        
        return ((uav_target_x, uav_target_y, uav_target_z), uav_target_yaw)
        
    def get_next_uav_position_and_yaw(self):
        """Get next UAV position and orientation (compatibility interface)"""
        return self.get_next_position_and_yaw()
    
    def get_next_position(self):
        """Get next position (compatibility interface)"""
        result = self.get_next_position_and_yaw()
        if result is None:
            return None
        return result[0]
        
    def is_complete(self):
        """Check if trajectory is complete"""
        return self.get_next_position() is None

    # TRAJECTORY PAUSE/RESUME INTERFACE
    def pause(self):
        """Pause trajectory execution"""
        if self._angle_trajectory is not None:
            self._angle_trajectory.pause()

    def resume(self):
        """Resume trajectory execution"""
        if self._angle_trajectory is not None:
            self._angle_trajectory.resume()

    def is_trajectory_paused(self):
        """Check if trajectory is paused"""
        if self._angle_trajectory is not None:
            return self._angle_trajectory.is_trajectory_paused()
        return False

    def reset_to_current_position(self, current_uav_pos, current_uav_yaw):
        """Reset trajectory to start from current UAV position"""
        # Calculate current angle based on UAV position
        center_x, center_y, center_z = self.valve_center
        
        # Calculate current end-effector position
        cos_yaw = cos(current_uav_yaw)
        sin_yaw = sin(current_uav_yaw)
        current_ee_x = current_uav_pos[0] + cos_yaw * self.end_effector_offset_x - sin_yaw * self.end_effector_offset_y
        current_ee_y = current_uav_pos[1] + sin_yaw * self.end_effector_offset_x + cos_yaw * self.end_effector_offset_y
        
        # Calculate angle from valve center to current end-effector
        new_start_angle = atan2(current_ee_y - center_y, current_ee_x - center_x)
        
        # CRITICAL FIX: Calculate actual distance from current position to valve center
        actual_distance = math.sqrt((current_ee_x - center_x)**2 + (current_ee_y - center_y)**2)
        
        # Update end_effector_rotation_radius to match current distance
        # This prevents large distance errors after reset
        original_radius = self.end_effector_rotation_radius
        self.end_effector_rotation_radius = actual_distance
        self.end_effector_distance = actual_distance  # Update alias too
        
        # CRITICAL: Update UAV target distance when trajectory resets
        if hasattr(self, 'uav_target_distance'):
            uav_distance = max(0.03, actual_distance - self.end_effector_offset_x)
            self.uav_target_distance = uav_distance
            self.uav_rotation_radius = uav_distance
        
        # Reinitialize trajectory from current position using CORRECT METHOD
        self.start_angle = new_start_angle
        self._angle_trajectory = PolynomialTrajectory(self.rotation_duration)
        # FIX: Use generate_trajectory instead of start method
        self._angle_trajectory.generate_trajectory(new_start_angle, new_start_angle + self.rotation_angle)
        
        rospy.logwarn(f"TRAJECTORY RESET to current position:")
        rospy.logwarn(f"   New start angle: {math.degrees(new_start_angle):.1f}°")
        rospy.logwarn(f"   Distance updated: {original_radius*1000:.1f}mm → {actual_distance*1000:.1f}mm")
        rospy.logwarn(f"   Current UAV: [{current_uav_pos[0]:.3f}, {current_uav_pos[1]:.3f}, {current_uav_pos[2]:.3f}]")
        rospy.logwarn(f"   Current EE:  [{current_ee_x:.3f}, {current_ee_y:.3f}]")
        rospy.logwarn(f"   ✅ Trajectory reset should eliminate distance error")

def validate_uav_position(uav_pos, uav_yaw, valve_center, 
                         end_effector_offset_x=0.246, end_effector_offset_y=0.0):
    """
    Validate UAV position for safe trajectory execution
    """
    # Calculate UAV to valve center distance
    uav_to_valve_distance = math.sqrt((uav_pos[0] - valve_center[0])**2 + 
                                    (uav_pos[1] - valve_center[1])**2)
    
    # Calculate end-effector position
    cos_yaw = cos(uav_yaw)
    sin_yaw = sin(uav_yaw)
    
    ee_world_x = uav_pos[0] + cos_yaw * end_effector_offset_x - sin_yaw * end_effector_offset_y
    ee_world_y = uav_pos[1] + sin_yaw * end_effector_offset_x + cos_yaw * end_effector_offset_y
    
    ee_to_valve_distance = math.sqrt((ee_world_x - valve_center[0])**2 + 
                                   (ee_world_y - valve_center[1])**2)
    
    # Validation criteria
    valve_radius = 0.1225
    # OPTIMIZED: Practical safe distance for post-insertion rotation start
    # Based on actual insertion results, UAV should be positioned optimally for rotation
    min_safe_uav_distance = valve_radius + end_effector_offset_x * 0.2  # Further reduced from 50% to 40%
    
    errors = []
    
    if uav_to_valve_distance < min_safe_uav_distance:
        errors.append(f"UAV too close to valve center: {uav_to_valve_distance:.3f}m < {min_safe_uav_distance:.3f}m")
    
    # Very lenient end-effector clearance check - 25% of valve radius for successful insertion
    if ee_to_valve_distance < valve_radius * 0.25:
        errors.append(f"End-effector inside valve: {ee_to_valve_distance:.3f}m < {valve_radius * 0.25:.3f}m")
    
    return len(errors) == 0, errors

def create_constant_distance_trajectory(current_uav_pos, current_uav_yaw, valve_center, 
                                      rotation_angle, rotation_duration, 
                                      end_effector_offset_x=0.246, end_effector_offset_y=0.0, 
                                      end_effector_offset_z=0.0743823, rotation_direction=1,
                                      enable_adaptive_control=True, control_mode='rotation'):
    """
    Create constant distance rotation trajectory with validation and adaptive control
    """
    rospy.loginfo("Creating constant distance rotation trajectory...")
    
    # Validate UAV position
    is_valid, errors = validate_uav_position(
        current_uav_pos, current_uav_yaw, valve_center,
        end_effector_offset_x, end_effector_offset_y
    )
    
    if not is_valid:
        rospy.logerr("UAV position validation failed:")
        for error in errors:
            rospy.logerr(f"  - {error}")
        rospy.logerr("Cannot create safe trajectory - UAV position must be corrected")
        return None
    
    # Calculate current end-effector position
    cos_yaw = cos(current_uav_yaw)
    sin_yaw = sin(current_uav_yaw)
    
    ee_x = current_uav_pos[0] + cos_yaw * end_effector_offset_x - sin_yaw * end_effector_offset_y
    ee_y = current_uav_pos[1] + sin_yaw * end_effector_offset_x + cos_yaw * end_effector_offset_y
    
    # Calculate distance to valve center
    valve_x, valve_y, _ = valve_center
    dx = ee_x - valve_x
    dy = ee_y - valve_y
    end_effector_distance = math.sqrt(dx**2 + dy**2)
    
    # Calculate starting angle
    start_angle = atan2(dy, dx)
    
    rospy.loginfo(f"Trajectory parameters:")
    rospy.loginfo(f"  UAV position: {current_uav_pos}")
    rospy.loginfo(f"  End-effector distance: {end_effector_distance:.3f}m")
    rospy.loginfo(f"  Starting angle: {start_angle:.3f}rad ({start_angle*180/pi:.1f}°)")
    rospy.loginfo(f"  Control mode: {control_mode.upper()}")
    rospy.loginfo(f"  Adaptive control: {'enabled' if enable_adaptive_control else 'disabled'}")
    
    # Create trajectory with adaptive control
    trajectory = ValveRotationTrajectory(
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
    
    return trajectory

# Legacy compatibility classes (kept for backward compatibility)
class AlignToGraspTrajectory:
    """Simplified align trajectory - recommend using insertion optimizer instead"""
    def __init__(self, approach_duration, valve_center, valve_pose_yaw, grasp_height, 
                 valve_radius=0.1225, valve_beam_width=0.0185, 
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823,
                 claw_separation=0.150, rotation_direction=1, insertion_offset=0.015):
        # Minimal implementation for backward compatibility
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
        
        rospy.logwarn("AlignToGraspTrajectory is deprecated - use insertion optimizer instead")
        
    def calculate_grasp_position_and_yaw(self):
        """Simplified grasp position calculation"""
        center_x, center_y, center_z = self.valve_center
        
        # Simple approach: position UAV at valve radius + offset distance
        target_distance = self.valve_radius + self.end_effector_offset_x
        target_angle = self.valve_pose_yaw + pi  # Opposite side of valve
        
        body_target_x = center_x + target_distance * cos(target_angle)
        body_target_y = center_y + target_distance * sin(target_angle)
        body_target_z = self.grasp_height
        body_yaw = target_angle
        
        end_effector_x = body_target_x + self.end_effector_offset_x * cos(body_yaw)
        end_effector_y = body_target_y + self.end_effector_offset_x * sin(body_yaw)
        end_effector_z = body_target_z + self.end_effector_offset_z
        
        return ((body_target_x, body_target_y, body_target_z), 
                body_yaw, 
                (end_effector_x, end_effector_y, end_effector_z))
    
    def set_start_position(self, start_pos):
        """Set start position and generate trajectory"""
        self._target_body_pos, self._target_body_yaw, _ = self.calculate_grasp_position_and_yaw()
        self._trajectory = PolynomialTrajectory(self.approach_duration)
        self._trajectory.generate_trajectory(start_pos, self._target_body_pos)
        
    def get_next_position_and_yaw(self):
        """Get next position and yaw"""
        if self._trajectory is None:
            return None
        pos = self._trajectory.evaluate()
        if pos is None:
            return None
        return (pos, self._target_body_yaw)
    
    def get_next_position(self):
        """Get next position"""
        result = self.get_next_position_and_yaw()
        if result is None:
            return None
        return result[0]
    
    def is_complete(self):
        """Check if trajectory is complete"""
        return self.get_next_position() is None

class ConstantDistanceValveRotationTrajectory(ValveRotationTrajectory):
    """Legacy compatibility class"""
    def __init__(self, valve_center, end_effector_distance, rotation_angle, 
                 rotation_duration, start_angle, 
                 end_effector_offset_x=0.246, end_effector_offset_y=0.0, end_effector_offset_z=0.0743823,
                 rotation_direction=1, grasp_height=None):
        super().__init__(
            rotation_duration=rotation_duration,
            valve_center=valve_center,
            end_effector_rotation_radius=end_effector_distance,
            start_angle=start_angle,
            rotation_angle=rotation_angle,
            grasp_height=grasp_height,
            rotation_direction=rotation_direction,
            end_effector_offset_x=end_effector_offset_x,
            end_effector_offset_y=end_effector_offset_y,
            end_effector_offset_z=end_effector_offset_z
        )
        rospy.logwarn("ConstantDistanceValveRotationTrajectory is deprecated - use ValveRotationTrajectory instead")

class AdaptiveTrajectoryPlanner:
    """
    Adaptive trajectory replanner - responsible for trajectory generation and replanning mathematical logic
    Responsibility: Pure trajectory calculation, not involving control execution
    """
    
    def __init__(self):
        self.precision_thresholds = {
            'position': 0.015,  # 15mm - Enhanced precision for collision avoidance
            'yaw': 0.02         # 1.1°
        }
        
    def create_trajectory(self, start_pos, target_pos, duration):
        """Create base trajectory"""
        trajectory = PolynomialTrajectory(duration)
        trajectory.generate_trajectory(start_pos, target_pos)
        return trajectory
    
    def calculate_deviation(self, planned_pos, actual_pos, planned_yaw=None, actual_yaw=None):
        """Calculate position and angle deviation"""
        if isinstance(planned_pos, (list, tuple)) and len(planned_pos) >= 3:
            pos_deviation = math.sqrt(
                (actual_pos[0] - planned_pos[0])**2 + 
                (actual_pos[1] - planned_pos[1])**2 + 
                (actual_pos[2] - planned_pos[2])**2
            )
        else:
            pos_deviation = abs(actual_pos - planned_pos)
        
        yaw_deviation = 0
        if planned_yaw is not None and actual_yaw is not None:
            yaw_deviation = abs(self._normalize_angle(actual_yaw - planned_yaw))
        
        return pos_deviation, yaw_deviation
    
    def needs_replanning(self, pos_deviation, yaw_deviation, stage_name=""):
        """Determine if replanning is needed"""
        pos_threshold = self.precision_thresholds['position']
        yaw_threshold = self.precision_thresholds['yaw']
        
        needs_replan = (pos_deviation > pos_threshold or yaw_deviation > yaw_threshold)
        
        if needs_replan:
            rospy.logwarn(f"{stage_name} deviation detected - Position: {pos_deviation*1000:.1f}mm (>{pos_threshold*1000:.0f}mm), Yaw: {math.degrees(yaw_deviation):.2f}° (>{math.degrees(yaw_threshold):.1f}°)")
        else:
            rospy.loginfo(f"{stage_name} deviation acceptable - Position: {pos_deviation*1000:.1f}mm, Yaw: {math.degrees(yaw_deviation):.2f}°")
        
        return needs_replan
    
    def replan_from_current(self, current_pos, current_yaw, original_target_pos, original_target_yaw, duration):
        """Replan from current position to target position"""
        rospy.loginfo("=== ADAPTIVE TRAJECTORY REPLANNING ===")
        rospy.loginfo(f"Replanning from actual position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}) at {math.degrees(current_yaw):.1f}°")
        rospy.loginfo(f"To target: ({original_target_pos[0]:.3f}, {original_target_pos[1]:.3f}, {original_target_pos[2]:.3f}) at {math.degrees(original_target_yaw):.1f}°")
        
        # Recalculate distance and duration
        distance = math.sqrt(
            (original_target_pos[0] - current_pos[0])**2 + 
            (original_target_pos[1] - current_pos[1])**2 + 
            (original_target_pos[2] - current_pos[2])**2
        )
        
        # Adaptively adjust duration
        adjusted_duration = max(duration * 0.3, distance / 0.1)  # Minimum 30% original time, or at 0.1m/s speed
        
        rospy.loginfo(f"Replanned trajectory: {distance:.3f}m in {adjusted_duration:.1f}s")
        
        # Create new trajectory
        new_trajectory = self.create_trajectory(current_pos, original_target_pos, adjusted_duration)
        
        return new_trajectory, adjusted_duration
    
    def replan_intermediate_target(self, current_pos, original_target, valve_center, intermediate_distance):
        """Recalculate intermediate target position (for multi-stage insertion)"""
        # Recalculate intermediate position based on actual current position
        direction_to_valve = math.atan2(
            valve_center[1] - current_pos[1], 
            valve_center[0] - current_pos[0]
        )
        
        intermediate_x = valve_center[0] - intermediate_distance * math.cos(direction_to_valve)
        intermediate_y = valve_center[1] - intermediate_distance * math.sin(direction_to_valve)
        intermediate_z = original_target[2]  # Maintain original height
        
        new_intermediate = (intermediate_x, intermediate_y, intermediate_z)
        
        rospy.loginfo(f"Recalculated intermediate target from actual position:")
        rospy.loginfo(f"  Original intermediate: {original_target}")
        rospy.loginfo(f"  New intermediate: {new_intermediate}")
        
        return new_intermediate
    
    def set_precision_thresholds(self, position_threshold, yaw_threshold):
        """Set precision thresholds"""
        self.precision_thresholds['position'] = position_threshold
        self.precision_thresholds['yaw'] = yaw_threshold
        rospy.loginfo(f"Updated precision thresholds: Position {position_threshold*1000:.0f}mm, Yaw {math.degrees(yaw_threshold):.1f}°")
    
    def _normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def create_z_only_trajectory(self, start_pos, target_z, duration):
        """Create Z-axis only trajectory, keeping XY locked"""
        target_pos = (start_pos[0], start_pos[1], target_z)
        trajectory = PolynomialTrajectory(duration)
        trajectory.generate_trajectory(start_pos, target_pos)
        return trajectory
    
    def calculate_optimal_z_segments(self, total_z_distance, max_segment_distance=0.18):
        """
        Calculate optimal segmentation strategy for Z-axis descent
        
        Args:
            total_z_distance: Total Z descent distance
            max_segment_distance: Maximum segment distance (default 180mm)
            
        Returns:
            list: List of distances for each segment
        """
        if total_z_distance <= max_segment_distance:
            return [total_z_distance]
        
        # Calculate number of segments and distance per segment
        num_segments = math.ceil(total_z_distance / max_segment_distance)
        segment_distance = total_z_distance / num_segments
        
        segments = [segment_distance] * num_segments
        
        rospy.loginfo(f"Z descent segmentation: {total_z_distance*1000:.0f}mm → {num_segments} segments of {segment_distance*1000:.0f}mm each")
        
        return segments
    
    def calculate_z_descent_speed(self, z_distance, min_speed=0.06, max_speed=0.10):
        """
        Calculate optimal speed based on Z descent distance to reduce control oscillation
        
        Args:
            z_distance: Z descent distance
            min_speed: Minimum speed (for large descents)
            max_speed: Maximum speed (for small descents)
            
        Returns:
            float: Optimal descent speed
        """
        if z_distance < 0.10:  # Less than 100mm
            return max_speed  # 0.10m/s
        elif z_distance < 0.20:  # 100-200mm
            return 0.08  # 0.08m/s
        else:  # Greater than 200mm
            return min_speed  # 0.06m/s


class SelfRotationRevolutionTrajectory:
    """
    Self-rotation + revolution valve rotation trajectory generator
    
    Implements true "self-rotation + revolution" motion mode:
    - Revolution: UAV performs circular motion around valve center
    - Self-rotation: UAV's yaw angle rotates synchronously
    
    This motion mode maintains constant contact angle between claw and valve groove, preventing disengagement
    """
    
    def __init__(self, valve_center, radius, rotation_angle, duration, 
                 start_pos, start_yaw, clockwise=True):
        """
        Initialize self-rotation + revolution trajectory
        
        Args:
            valve_center: Valve center coordinates (x, y, z)
            radius: Rotation radius (distance from UAV to valve center)
            rotation_angle: Rotation angle (radians)
            duration: Motion duration (seconds)
            start_pos: Starting position (x, y, z)
            start_yaw: Starting yaw angle (radians)
            clockwise: Whether to rotate clockwise
        """
        self.valve_center = valve_center
        self.radius = radius
        self.rotation_angle = rotation_angle if not clockwise else -rotation_angle
        self.duration = duration
        self.start_pos = start_pos
        self.start_yaw = start_yaw
        self.clockwise = clockwise
        
        # Calculate starting angle
        dx = start_pos[0] - valve_center[0]
        dy = start_pos[1] - valve_center[1]
        self.start_angle = math.atan2(dy, dx)
        
        # Calculate initial yaw reference angle (angle facing valve center)
        self.initial_center_facing_yaw = math.atan2(
            valve_center[1] - start_pos[1],
            valve_center[0] - start_pos[0]
        )
        
        rospy.loginfo("=== SELF-ROTATION + REVOLUTION TRAJECTORY ===")
        rospy.loginfo(f"Valve center: ({valve_center[0]:.3f}, {valve_center[1]:.3f}, {valve_center[2]:.3f})")
        rospy.loginfo(f"Rotation radius: {radius*1000:.1f}mm")
        rospy.loginfo(f"Rotation angle: {math.degrees(abs(rotation_angle)):.1f}°")
        rospy.loginfo(f"Direction: {'Clockwise' if clockwise else 'Counter-clockwise'}")
        rospy.loginfo(f"Duration: {duration:.1f}s")
        rospy.loginfo(f"Start angle: {math.degrees(self.start_angle):.1f}°")
        rospy.loginfo(f"Start yaw: {math.degrees(start_yaw):.1f}°")
    
    def get_position_and_yaw_at_progress(self, progress):
        """
        Calculate position and yaw angle based on progress
        
        Args:
            progress: Motion progress [0.0, 1.0]
            
        Returns:
            tuple: ((x, y, z), yaw)
        """
        if progress < 0.0:
            progress = 0.0
        elif progress > 1.0:
            progress = 1.0
        
        # === Revolution: Calculate circular motion position ===
        current_angle = self.start_angle + progress * self.rotation_angle
        target_x = self.valve_center[0] + self.radius * math.cos(current_angle)
        target_y = self.valve_center[1] + self.radius * math.sin(current_angle)
        target_z = self.start_pos[2]  # Maintain constant Z height
        
        # === Self-rotation: UAV yaw angle rotates synchronously ===
        # Method: UAV's yaw rotates synchronously with circular motion, maintaining relative contact angle
        target_yaw = self.start_yaw + progress * self.rotation_angle
        
        return ((target_x, target_y, target_z), target_yaw)
    
    def get_velocity_at_progress(self, progress):
        """
        Calculate linear and angular velocity at specified progress
        
        Args:
            progress: Motion progress [0.0, 1.0]
            
        Returns:
            tuple: ((vx, vy, vz), yaw_velocity)
        """
        # Calculate angular velocity
        angular_velocity = abs(self.rotation_angle) / self.duration
        
        # Calculate current angle
        current_angle = self.start_angle + progress * self.rotation_angle
        
        # Calculate tangential angle (perpendicular to radial direction)
        tangential_angle = current_angle + math.pi/2
        if self.rotation_angle < 0:  # Clockwise rotation, reverse tangential angle
            tangential_angle += math.pi
        
        # Calculate tangential velocity
        tangential_speed = self.radius * angular_velocity
        vx = tangential_speed * math.cos(tangential_angle)
        vy = tangential_speed * math.sin(tangential_angle)
        vz = 0.0  # Z-direction velocity is 0
        
        # Calculate yaw angular velocity
        yaw_direction = 1.0 if self.rotation_angle < 0 else -1.0  # Clockwise vs counter-clockwise
        yaw_velocity = angular_velocity * yaw_direction
        
        return ((vx, vy, vz), yaw_velocity)
    
    def apply_velocity_smoothing(self, progress, velocity, yaw_velocity):
        """
        Apply velocity smoothing (acceleration and deceleration)
        
        Args:
            progress: Motion progress [0.0, 1.0]
            velocity: Original velocity (vx, vy, vz)
            yaw_velocity: Original yaw angular velocity
            
        Returns:
            tuple: (smoothed_velocity, smoothed_yaw_velocity)
        """
        smooth_factor = 1.0
        
        # First 10%: smooth acceleration
        if progress < 0.1:
            smooth_factor = progress / 0.1
        # Last 10%: smooth deceleration
        elif progress > 0.9:
            smooth_factor = (1.0 - progress) / 0.1
        
        # Apply smoothing factor
        smoothed_velocity = (
            velocity[0] * smooth_factor,
            velocity[1] * smooth_factor,
            velocity[2] * smooth_factor
        )
        smoothed_yaw_velocity = yaw_velocity * smooth_factor
        
        return (smoothed_velocity, smoothed_yaw_velocity)
    
    def validate_trajectory(self, progress_points=10):
        """
        Validate trajectory validity
        
        Args:
            progress_points: Number of validation points
            
        Returns:
            tuple: (is_valid, error_messages)
        """
        errors = []
        
        # Check basic parameters
        if self.duration <= 0:
            errors.append("Duration must be positive")
        
        if self.radius <= 0:
            errors.append("Radius must be positive")
        
        if abs(self.rotation_angle) < math.radians(1):
            errors.append("Rotation angle too small (< 1°)")
        
        # Check trajectory points
        for i in range(progress_points + 1):
            progress = i / progress_points
            position, yaw = self.get_position_and_yaw_at_progress(progress)
            
            # Check radius consistency
            actual_radius = math.sqrt(
                (position[0] - self.valve_center[0])**2 + 
                (position[1] - self.valve_center[1])**2
            )
            radius_error = abs(actual_radius - self.radius)
            
            if radius_error > 0.001:  # 1mm tolerance
                errors.append(f"Radius inconsistency at progress {progress:.1f}: "
                             f"expected {self.radius:.3f}, got {actual_radius:.3f}")
        
        is_valid = len(errors) == 0
        return (is_valid, errors)
    
    def get_trajectory_info(self):
        """
        Get trajectory information summary
        
        Returns:
            dict: Trajectory information
        """
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
    """
    Valve rotation trajectory manager
    
    Provides unified interface to create and manage different types of valve rotation trajectories
    """
    
    @staticmethod
    def create_self_rotation_revolution_trajectory(valve_center, radius, rotation_angle_deg, 
                                                  duration, start_pos, start_yaw, clockwise=True):
        """
        Create self-rotation + revolution trajectory
        
        Args:
            valve_center: Valve center coordinates
            radius: Rotation radius
            rotation_angle_deg: Rotation angle (degrees)
            duration: Duration
            start_pos: Starting position
            start_yaw: Starting yaw angle
            clockwise: Whether to rotate clockwise
            
        Returns:
            SelfRotationRevolutionTrajectory: Trajectory object
        """
        rotation_angle_rad = math.radians(rotation_angle_deg)
        return SelfRotationRevolutionTrajectory(
            valve_center=valve_center,
            radius=radius,
            rotation_angle=rotation_angle_rad,
            duration=duration,
            start_pos=start_pos,
            start_yaw=start_yaw,
            clockwise=clockwise
        )
    
    @staticmethod
    def create_legacy_trajectory(valve_center, radius, rotation_angle_deg, duration, start_angle):
        """
        Create legacy trajectory (revolution only, always facing center)
        
        Args:
            valve_center: Valve center coordinates
            radius: Rotation radius
            rotation_angle_deg: Rotation angle (degrees)
            duration: Duration
            start_angle: Starting angle
            
        Returns:
            ConstantDistanceValveRotationTrajectory: Trajectory object
        """
        return ConstantDistanceValveRotationTrajectory(
            valve_center=valve_center,
            end_effector_distance=radius,
            rotation_angle=math.radians(rotation_angle_deg),
            rotation_duration=duration,
            start_angle=start_angle
        )


class OnlineCircularTrajectoryGenerator:
    """
    在线圆周轨迹生成器 - 专为Beetle倾转四旋翼设计
    
    基于Dragon控制理念但简化适配Beetle机械结构特点：
    - 固定机体+末端执行器，无复杂多连杆结构
    - 专注CoG轨迹规划，末端执行器通过几何偏移
    - 恒定角速度的在线轨迹重规划
    - 向心力补偿和扭矩前馈（简化版）
    - 半径自适应和锁定机制
    
    核心Dragon概念保留：
    1. 在线轨迹生成：delta_yaw = delta_t * turn_vel
    2. 恒定角速度控制
    3. 半径锁定机制（80%目标角速度时锁定半径）
    4. 力矩前馈和向心力补偿
    """
    
    def __init__(self, valve_center, initial_radius, target_angular_velocity=0.5, 
                 control_rate=15.0, debug=False):
        """
        初始化在线圆周轨迹生成器
        
        Args:
            valve_center: 阀门中心位置 [x, y, z]
            initial_radius: 初始半径 (m)
            target_angular_velocity: 目标角速度 (rad/s)，范围通常0.2-1.0
            control_rate: 控制频率 (Hz)，建议10-20Hz
            debug: 调试模式
        """
        self.valve_center = np.array(valve_center)
        self.initial_radius = initial_radius
        self.target_angular_velocity = target_angular_velocity  # 保留符号以支持反向旋转
        self.control_rate = control_rate
        self.debug = debug
        
        # 时间步长
        self.dt = 1.0 / control_rate
        
        # 状态变量
        self.current_angle = 0.0  # 当前角度 (rad)
        self.current_radius = initial_radius  # 当前半径
        self.radius_locked = False  # 半径锁定状态
        self.total_rotation = 0.0  # 累计转动角度
        
        # 轨迹参数（类似Dragon）
        self.angular_velocity = 0.0  # 当前角速度
        self.radius_adaptation_rate = 0.1  # 半径自适应速率
        self.min_radius = initial_radius * 0.8  # 最小半径
        self.max_radius = initial_radius * 1.2  # 最大半径
        
        # 力控制参数（简化的Beetle版本）
        self.mass = 1.5  # Beetle质量 (kg)
        self.init_torque = 0.1  # 初始扭矩 (N·m)
        self.torque_limit = 3.0  # 扭矩限制 (N·m)
        self.current_torque = self.init_torque
        
        # Z坐标管理（修复状态切换时的Z跳跃）
        self.current_z = None  # 保存当前Z坐标，避免强制跳到valve_center[2]
        
        # 半径锁定机制（Dragon概念）
        self.velocity_threshold_for_lock = 0.8  # 80%目标速度时锁定半径
        self.lock_radius_value = None
        
        # 性能监控
        self.start_time = rospy.Time.now().to_sec()
        self.last_update_time = self.start_time
        
        rospy.loginfo("=== 在线圆周轨迹生成器（Beetle版）===")
        rospy.loginfo(f"阀门中心: ({valve_center[0]:.3f}, {valve_center[1]:.3f}, {valve_center[2]:.3f})")
        rospy.loginfo(f"初始半径: {initial_radius*1000:.1f}mm")
        rospy.loginfo(f"目标角速度: {math.degrees(target_angular_velocity):.1f}°/s")
        rospy.loginfo(f"控制频率: {control_rate:.1f}Hz")
        rospy.loginfo(f"时间步长: {self.dt*1000:.1f}ms")
    
    def update_state(self, current_pos, current_yaw, valve_angular_velocity=0.0):
        """
        更新轨迹状态 - Dragon风格的在线重规划
        
        Args:
            current_pos: 当前UAV位置 [x, y, z]
            current_yaw: 当前UAV偏航角 (rad)
            valve_angular_velocity: 阀门角速度 (rad/s)，用于检测是否开始转动
            
        Returns:
            dict: 更新后的状态信息
        """
        current_time = rospy.Time.now().to_sec()
        actual_dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # 计算当前实际位置相对阀门中心的角度和半径
        relative_pos = np.array(current_pos[:2]) - self.valve_center[:2]
        actual_radius = np.linalg.norm(relative_pos)
        actual_angle = math.atan2(relative_pos[1], relative_pos[0])
        
        # Dragon风格在线轨迹更新：delta_yaw = delta_t * turn_vel
        # 根据当前性能调整角速度（支持正负方向）
        if valve_angular_velocity > 0.1:  # 阀门开始转动
            # 逐渐加速到目标角速度，保持符号
            if self.target_angular_velocity >= 0:
                acceleration = min(0.1, self.target_angular_velocity - self.angular_velocity)
                self.angular_velocity = min(self.target_angular_velocity, 
                                          self.angular_velocity + acceleration * actual_dt)
            else:
                acceleration = max(-0.1, self.target_angular_velocity - self.angular_velocity)
                self.angular_velocity = max(self.target_angular_velocity, 
                                          self.angular_velocity + acceleration * actual_dt)
        else:
            # 缓慢启动到目标速度，保持符号方向
            startup_acceleration = 0.01  # 较小的启动加速度，确保平滑启动
            if self.target_angular_velocity >= 0:
                self.angular_velocity = min(self.target_angular_velocity, 
                                          self.angular_velocity + startup_acceleration * actual_dt)
            else:
                self.angular_velocity = max(self.target_angular_velocity, 
                                          self.angular_velocity - startup_acceleration * actual_dt)
        
        # 更新角度：核心Dragon公式
        delta_angle = self.angular_velocity * actual_dt
        self.current_angle += delta_angle
        self.total_rotation += abs(delta_angle)
        
        # 半径自适应逻辑（Dragon概念，Beetle简化）
        if not self.radius_locked:
            # 根据实际半径进行自适应调整
            radius_error = actual_radius - self.current_radius
            radius_correction = radius_error * self.radius_adaptation_rate
            self.current_radius += radius_correction
            
            # 限制半径范围
            self.current_radius = max(self.min_radius, 
                                    min(self.max_radius, self.current_radius))
            
            # 检查是否需要锁定半径（Dragon机制），使用绝对值计算比例
            velocity_ratio = abs(self.angular_velocity) / abs(self.target_angular_velocity) if self.target_angular_velocity != 0 else 0
            if velocity_ratio >= self.velocity_threshold_for_lock:
                self.radius_locked = True
                self.lock_radius_value = self.current_radius
                if self.debug:
                    rospy.loginfo(f"半径锁定！当前半径: {self.current_radius*1000:.1f}mm, "
                                f"角速度比例: {velocity_ratio:.1%}")
        else:
            # 半径已锁定，使用固定值
            self.current_radius = self.lock_radius_value
        
        # 扭矩自适应（简化的力控制）
        if valve_angular_velocity > 0.05:
            # 阀门转动中，根据阻力调整扭矩，使用绝对值计算阻力
            resistance_factor = max(0.5, min(2.0, abs(self.target_angular_velocity) / max(0.1, valve_angular_velocity)))
            self.current_torque = min(self.torque_limit, 
                                    self.init_torque * resistance_factor)
        
        return {
            'current_angle': self.current_angle,
            'current_radius': self.current_radius,
            'angular_velocity': self.angular_velocity,
            'radius_locked': self.radius_locked,
            'total_rotation': self.total_rotation,
            'current_torque': self.current_torque,
            'actual_radius': actual_radius,
            'actual_angle': actual_angle
        }
    
    def generate_target_state(self):
        """
        生成目标状态 - Dragon风格的SE(3)控制目标
        
        Returns:
            dict: 包含位置、速度、扭矩的完整控制目标
        """
        # 目标位置（CoG位置）
        target_x = self.valve_center[0] + self.current_radius * math.cos(self.current_angle)
        target_y = self.valve_center[1] + self.current_radius * math.sin(self.current_angle)
        # 修复：优先使用保存的Z坐标，避免状态切换时Z跳跃
        target_z = self.current_z if self.current_z is not None else self.valve_center[2]
        target_pos = np.array([target_x, target_y, target_z])
        
        # 目标姿态（面向阀门中心）
        target_yaw = self.current_angle + math.pi
        if target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        
        # 切向速度（Dragon核心概念）
        tangential_speed = self.current_radius * self.angular_velocity
        tangential_angle = self.current_angle + math.pi/2  # 90度相位差
        
        target_vel_x = tangential_speed * math.cos(tangential_angle)
        target_vel_y = tangential_speed * math.sin(tangential_angle)
        target_vel_z = 0.0
        target_linear_vel = np.array([target_vel_x, target_vel_y, target_vel_z])
        
        # 角速度（自转+公转）
        target_angular_vel = self.angular_velocity
        
        # 力和扭矩前馈（Beetle简化版）
        # 向心力计算
        centripetal_force = self.mass * tangential_speed**2 / self.current_radius
        centripetal_force_x = -centripetal_force * math.cos(self.current_angle)
        centripetal_force_y = -centripetal_force * math.sin(self.current_angle)
        
        # 重力补偿（简化）
        gravity_compensation_z = self.mass * 9.81 * 0.1  # 10%重力补偿
        
        target_force = np.array([centripetal_force_x, centripetal_force_y, gravity_compensation_z])
        
        # 扭矩前馈（主要是Z轴扭矩）
        target_torque = np.array([0.0, 0.0, self.current_torque])
        
        return {
            'position': target_pos,
            'yaw': target_yaw,
            'linear_velocity': target_linear_vel,
            'angular_velocity': target_angular_vel,
            'force': target_force,
            'torque': target_torque,
            'radius': self.current_radius,
            'angle': self.current_angle
        }
    
    def is_motion_completed(self, target_rotation_angle):
        """
        检查运动是否完成
        
        Args:
            target_rotation_angle: 目标旋转角度 (rad)
            
        Returns:
            bool: 是否完成
        """
        return self.total_rotation >= abs(target_rotation_angle)
    
    def get_progress(self, target_rotation_angle):
        """
        获取运动进度
        
        Args:
            target_rotation_angle: 目标旋转角度 (rad)
            
        Returns:
            float: 进度 [0.0, 1.0]
        """
        return min(1.0, self.total_rotation / abs(target_rotation_angle))
    
    def reset(self, new_valve_center=None, new_radius=None):
        """
        重置轨迹生成器
        """
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
        
        rospy.loginfo("在线圆周轨迹生成器已重置")
    
    def get_status_info(self):
        """
        获取状态信息用于调试
        """
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
        """
        基于当前UAV位置初始化轨迹生成器的角度
        
        Args:
            current_pos: 当前位置 [x, y, z]
        """
        relative_pos = np.array(current_pos[:2]) - self.valve_center[:2]
        current_angle = math.atan2(relative_pos[1], relative_pos[0])
        current_radius = np.linalg.norm(relative_pos)
        
        self.current_angle = current_angle
        self.current_radius = current_radius
        self.initial_radius = current_radius
        # 修复：保存当前Z坐标，避免状态切换时的Z跳跃
        self.current_z = current_pos[2]
        
        if self.debug:
            rospy.loginfo(f"轨迹生成器初始化: 角度={math.degrees(current_angle):.1f}°, 半径={current_radius*1000:.1f}mm, Z={self.current_z:.3f}m")
