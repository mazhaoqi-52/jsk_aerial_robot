#!/usr/bin/env python
"""
Rotation Completion Detector - Intelligent multi-criteria completion detection
Resolves the issue of UAV getting "stuck" after valve rotation completion
"""

import rospy
import math
import time
from threading import Lock

class RotationCompletionDetector:
    """
    Multi-criteria completion detection to prevent infinite position correction loops
    
    Completion Criteria:
    1. Valve rotation angle target reached (primary)
    2. Distance error stabilization (secondary) 
    3. Time-based completion (fallback)
    4. Position correction count limit (safety)
    """
    
    def __init__(self):
        # Valve rotation criteria  
        self.target_valve_rotation = math.radians(90)  # 90 degrees target
        self.valve_rotation_threshold = math.radians(80)  # 90% completion = 80° (reasonable threshold)
        self.valve_rotation_achieved = False
        
        # Distance error criteria  
        self.distance_tolerance = 0.015  # STRICT: Only 15mm tolerance (was 30mm!)
        self.distance_stabilization_threshold = 0.003  # STRICT: 3mm variation (was 5mm)
        self.distance_stabilization_duration = 5.0  # LONGER: 5 seconds stability required
        self.distance_error_history = []
        self.distance_stable_start_time = None
        
        # Time-based criteria
        self.min_rotation_duration = 20.0  # NEW: Minimum 20 seconds before allowing completion
        self.max_rotation_duration = 180.0  # 3 minutes maximum rotation time
        self.rotation_start_time = None
        
        # Position correction limit
        self.max_correction_attempts = 50  # Limit to 50 position corrections
        self.correction_count = 0
        
        # Thread safety
        self.lock = Lock()
        
        # Status tracking
        self.completion_detected = False
        self.completion_reason = ""
        
        rospy.loginfo("=== ROTATION COMPLETION DETECTOR INITIALIZED ===")
        rospy.loginfo(f"Valve rotation target: {math.degrees(self.target_valve_rotation):.0f}°")
        rospy.loginfo(f"Valve rotation threshold: {math.degrees(self.valve_rotation_threshold):.0f}°")
        rospy.loginfo(f"Distance tolerance: {self.distance_tolerance*1000:.0f}mm")
        rospy.loginfo(f"Max rotation duration: {self.max_rotation_duration:.0f}s")
        rospy.loginfo(f"Max correction attempts: {self.max_correction_attempts}")
    
    def start_rotation_monitoring(self):
        """Start rotation monitoring session"""
        with self.lock:
            self.rotation_start_time = time.time()
            self.completion_detected = False
            self.completion_reason = ""
            self.correction_count = 0
            self.distance_error_history = []
            self.distance_stable_start_time = None
            self.valve_rotation_achieved = False
            
        rospy.loginfo("Rotation completion monitoring started")
    
    def update_valve_rotation(self, current_valve_rotation):
        """Update valve rotation progress"""
        with self.lock:
            if current_valve_rotation >= self.valve_rotation_threshold:
                if not self.valve_rotation_achieved:
                    self.valve_rotation_achieved = True
                    rospy.loginfo(f"VALVE ROTATION TARGET ACHIEVED: {math.degrees(current_valve_rotation):.1f}°")
                    
                    if current_valve_rotation >= self.target_valve_rotation:
                        self.completion_detected = True
                        self.completion_reason = "valve_rotation_complete"
                        rospy.loginfo("COMPLETION DETECTED: Valve rotation target fully achieved")
    
    def update_distance_error(self, distance_error):
        """Update distance error and check for stabilization"""
        with self.lock:
            current_time = time.time()
            self.distance_error_history.append((current_time, distance_error))
            
            # Keep only recent history (last 10 seconds)
            cutoff_time = current_time - 10.0
            self.distance_error_history = [(t, e) for t, e in self.distance_error_history if t >= cutoff_time]
            
            # Check for distance stabilization
            if len(self.distance_error_history) >= 10:  # At least 10 readings
                recent_errors = [e for _, e in self.distance_error_history[-10:]]
                error_variance = self._calculate_variance(recent_errors)
                mean_error = sum(recent_errors) / len(recent_errors)
                
                # CRITICAL FIX: Only consider as stable if error is actually SMALL
                # Don't accept "stable" large errors as completion!
                is_error_small = mean_error <= self.distance_tolerance  # < 15mm now
                is_variance_small = error_variance < self.distance_stabilization_threshold**2
                
                # NEW: Check minimum execution time protection
                execution_time = current_time - self.rotation_start_time if self.rotation_start_time else 0
                has_min_time = execution_time >= self.min_rotation_duration
                
                rospy.loginfo(f"🔒 Distance analysis: variance={error_variance:.6f}, mean_error={mean_error*1000:.1f}mm, time={execution_time:.1f}s")
                
                if is_variance_small and is_error_small and has_min_time:
                    # Distance is stable, small, AND minimum time elapsed
                    if self.distance_stable_start_time is None:
                        self.distance_stable_start_time = current_time
                        rospy.loginfo(f"🔒 TRUE convergence detected: variance={error_variance:.6f}, mean_error={mean_error*1000:.1f}mm, time={execution_time:.1f}s")
                    
                    # Check if stable for required duration
                    elif current_time - self.distance_stable_start_time >= self.distance_stabilization_duration:
                        if self.valve_rotation_achieved:
                            self.completion_detected = True
                            self.completion_reason = "distance_stabilized_with_rotation"
                            rospy.loginfo("COMPLETION DETECTED: Distance stabilized after valve rotation")
                elif not has_min_time:
                    # Too early - minimum time not reached
                    rospy.loginfo(f"⏱ Minimum time protection: {execution_time:.1f}s < {self.min_rotation_duration:.1f}s required")
                    self.distance_stable_start_time = None
                elif is_variance_small and not is_error_small:
                    # Stable but large error - this is a problem, not completion!
                    rospy.logwarn(f"🚫 STABLE LARGE ERROR detected: {mean_error*1000:.0f}mm - NOT completion!")
                    self.distance_stable_start_time = None
                else:
                    # Distance not stable, reset timer
                    self.distance_stable_start_time = None
    
    def record_position_correction(self):
        """Record a position correction attempt"""
        with self.lock:
            self.correction_count += 1
            
            if self.correction_count >= self.max_correction_attempts:
                self.completion_detected = True
                self.completion_reason = "max_corrections_reached"
                rospy.logwarn(f"WARNING: COMPLETION FORCED: Maximum correction attempts reached ({self.max_correction_attempts})")
    
    def check_timeout(self):
        """Check for timeout condition"""
        if self.rotation_start_time is None:
            return False
            
        with self.lock:
            current_time = time.time()
            elapsed_time = current_time - self.rotation_start_time
            
            if elapsed_time >= self.max_rotation_duration:
                self.completion_detected = True
                self.completion_reason = "timeout"
                rospy.logwarn(f"WARNING: COMPLETION FORCED: Maximum rotation duration reached ({self.max_rotation_duration:.0f}s)")
                return True
        
        return False
    
    def is_completed(self):
        """Check if rotation is considered complete"""
        with self.lock:
            return self.completion_detected
    
    def get_completion_reason(self):
        """Get the reason for completion"""
        with self.lock:
            return self.completion_reason
    
    def get_progress_summary(self):
        """Get current progress summary"""
        with self.lock:
            current_time = time.time()
            elapsed_time = current_time - self.rotation_start_time if self.rotation_start_time else 0
            
            return {
                'elapsed_time': elapsed_time,
                'correction_count': self.correction_count,
                'valve_rotation_achieved': self.valve_rotation_achieved,
                'distance_stabilized': self.distance_stable_start_time is not None,
                'completion_detected': self.completion_detected,
                'completion_reason': self.completion_reason
            }
    
    def _calculate_variance(self, values):
        """Calculate variance of a list of values"""
        if len(values) < 2:
            return float('inf')
        
        mean = sum(values) / len(values)
        variance = sum((x - mean)**2 for x in values) / len(values)
        return variance

# Global instance for easy access
completion_detector = RotationCompletionDetector()

def get_completion_detector():
    """Get the global completion detector instance"""
    return completion_detector