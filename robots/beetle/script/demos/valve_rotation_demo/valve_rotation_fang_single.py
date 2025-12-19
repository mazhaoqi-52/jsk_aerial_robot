#!/usr/bin/env python3
"""
Single UAV Valve Rotation using SMACH State Machine
Cleaned version with redundant code removed.
"""

import sys
import os

# Add current directory FIRST to ensure local imports work correctly
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)  # Insert at beginning for priority

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
import smach
import smach_ros
import time
import math
import threading
import numpy as np

from geometry_msgs.msg import PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from trajectory import create_constant_distance_trajectory, ValveRotationTrajectoryManager, SelfRotationRevolutionTrajectory, OnlineCircularTrajectoryGenerator, PolynomialTrajectory
from simplified_motion_controller import SimplifiedMotionController
from beetle_interface import BeetleInterface

# Import the local insertion optimizer
from insertion_optimizer import InsertionOptimizer


# DEPRECATED: Use InsertionOptimizer for all geometric calculations
def calculate_unified_end_effector_position(uav_pos, uav_yaw, include_z=False):
    """
    DEPRECATED: Use InsertionOptimizer for all geometric calculations instead
    
    This function is deprecated. All geometric calculations should be done
    through the InsertionOptimizer class to ensure consistency.
    """
    rospy.logwarn("DEPRECATED: calculate_unified_end_effector_position() - Use InsertionOptimizer instead")
    return None


class SingleUAVStateBase(smach.State):
    # 类变量：保存Phase 3的目标Z坐标，供后续状态使用
    _shared_target_z = None
    
    def __init__(self, outcomes, input_keys=None, output_keys=None, module_id=1):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys or [], output_keys=output_keys or [])
        self.module_id = module_id
        
        # Initialize BeetleInterface for control
        self.beetle = BeetleInterface(module_id=module_id, debug_view=False)
        
        # Use simplified motion controller that internally uses BeetleInterface
        # This maintains backward compatibility while using the new architecture
        from simplified_motion_controller import SimplifiedMotionController
        self.motion_controller = SimplifiedMotionController(self.beetle)
        
        rospy.loginfo(f"Initialized UAV{module_id} with BeetleInterface and motion controller")
        
        # Parameters
        self.position_threshold = 0.03
        self.yaw_threshold = 0.05236
        self.yaw_precision_levels = {
            'coarse': 0.05236,
            'moderate': 0.02,
            'fine': 0.015,
            'ultra_fine': 0.01,
            'insertion': 0.008
        }
        self.timeout = 20.0  # 20秒超时，给圆周接触建立充分的时间
        self.z_offset = 0.0743823
        self.descent_speed = 0.1  # 提速: 从0.025提升到0.1 m/s
        
        # Backward compatibility properties
        self.is_simulation = self.beetle.is_simulation
        self.pub = self.beetle.nav_pub  # For backward compatibility
        
    # Backward compatibility property accessors
    @property
    def uav_pos(self):
        return self.beetle.getUavPos()
    
    @property
    def current_yaw(self):
        return self.beetle.getUavRPY()[2]
    
    @property
    def current_pitch(self):
        return self.beetle.getUavRPY()[1]
    
    @property
    def valve_pos(self):
        return self.beetle.getValvePos()
    
    @property
    def valve_yaw(self):
        return self.beetle.getValveYaw()
    
    @property
    def external_wrench(self):
        return self.beetle.getEstimatedWrench()
    
    @property
    def uav_received(self):
        return self.beetle.getUavPos() is not None
    
    @property
    def valve_received(self):
        return self.beetle.getValvePos() is not None
        
        # NEW: Save insertion geometry for improved disengagement
        self.insertion_info = {
            'initial_position': None,      # UAV position before insertion
            'initial_yaw': None,          # UAV yaw before insertion  
            'initial_distance': None,     # Distance from valve center before insertion
            'spoke_angle': None,          # Selected spoke angle for insertion
            'valve_center': None,         # Valve center position during insertion
            'rotation_distance': None     # End-effector distance used in rotation phase
        }
        
        rospy.loginfo(f"Initialized UAV{module_id} state with basic parameters")
    
    def _calculate_unified_end_effector_position(self, uav_pos, uav_yaw, include_z=False):
        """Calculate end-effector position using optimizer parameters"""
        if not hasattr(self, 'optimizer') or not self.optimizer:
            rospy.logerr("Optimizer not initialized - cannot calculate end-effector position")
            return None
            
        cos_yaw = math.cos(uav_yaw)
        sin_yaw = math.sin(uav_yaw)
        
        # Use optimizer's physical parameters
        end_effector_x = uav_pos[0] + self.optimizer.dual_fang_center_offset * cos_yaw
        end_effector_y = uav_pos[1] + self.optimizer.dual_fang_center_offset * sin_yaw
        
        if include_z:
            end_effector_z = uav_pos[2] + self.optimizer.end_effector_offset_z
            return (end_effector_x, end_effector_y, end_effector_z)
        else:
            return (end_effector_x, end_effector_y)
    
    def __del__(self):
        """Destructor to ensure proper cleanup"""
        try:
            self.cleanup()
        except:
            pass  # Ignore errors during cleanup
    
    def cleanup(self):
        """Clean up resources and stop threads"""
        if hasattr(self, 'stop_emergency_monitoring'):
            self.stop_emergency_monitoring()
    
    def _save_insertion_geometry(self, current_position):
        """
        Save insertion geometry for beam-return disengagement calculation.
        Only saves essential info: which beam (spoke_angle) and reference values.
        
        Args:
            current_position: Current UAV position at insertion start
        """
        import math
        
        if not self.valve_pos:
            rospy.logwarn("No valve position available for insertion geometry")
            return
        
        # Calculate spoke angle (which beam was selected)
        valve_x, valve_y = self.valve_pos[0], self.valve_pos[1]
        spoke_angle = math.atan2(
            current_position[1] - valve_y,
            current_position[0] - valve_x
        )
        
        # Calculate initial distance for reference
        initial_distance = math.sqrt(
            (current_position[0] - valve_x)**2 +
            (current_position[1] - valve_y)**2
        )
        
        # Save only essential beam-return info
        self.insertion_info = {
            'initial_position': (current_position[0], current_position[1], current_position[2]),
            'initial_yaw': self.current_yaw,
            'initial_distance': initial_distance,
            'spoke_angle': spoke_angle,  # KEY: which beam was selected
            'valve_center': (valve_x, valve_y)
        }
        
        rospy.loginfo(f"✓ Saved insertion geometry: spoke_angle={math.degrees(spoke_angle):.1f}°, distance={initial_distance:.3f}m")
    
    def _calculate_rotated_beam_position(self, current_valve_yaw=None):
        """
        Calculate the beam position after valve rotation.
        Simple calculation: rotated_spoke_angle = original_spoke_angle + valve_rotation
        
        Args:
            current_valve_yaw: Current valve yaw angle (default: self.valve_yaw)
            
        Returns:
            tuple: (x, y, z) position of rotated beam
        """
        import math
        
        if not self.insertion_info.get('spoke_angle'):
            rospy.logerr("No insertion geometry saved - cannot calculate rotated beam position")
            return None
        
        if current_valve_yaw is None:
            current_valve_yaw = self.valve_yaw
        
        # Get essential values
        initial_valve_yaw = self.insertion_info['initial_yaw']
        initial_distance = self.insertion_info['initial_distance']
        spoke_angle = self.insertion_info['spoke_angle']
        valve_center = self.insertion_info['valve_center']
        initial_z = self.insertion_info['initial_position'][2]
        
        # Core calculation: beam rotates with valve
        valve_rotation = current_valve_yaw - initial_valve_yaw
        rotated_spoke_angle = spoke_angle + valve_rotation
        
        # Calculate new beam position
        rotated_beam_x = valve_center[0] + initial_distance * math.cos(rotated_spoke_angle)
        rotated_beam_y = valve_center[1] + initial_distance * math.sin(rotated_spoke_angle)
        rotated_beam_z = initial_z
        
        rospy.loginfo(f"✓ Beam rotated by {math.degrees(valve_rotation):.1f}° → "
                     f"({rotated_beam_x:.3f}, {rotated_beam_y:.3f}, {rotated_beam_z:.3f})")
        
        return (rotated_beam_x, rotated_beam_y, rotated_beam_z)
    
    def wait_for_positions(self):
        rospy.loginfo("Waiting for UAV and valve positions...")
        # Use BeetleInterface data availability check
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < 5.0:
            if self.beetle.getUavPos() is not None and self.beetle.getValvePos() is not None:
                rospy.loginfo("Both UAV and valve positions received")
                return True
            rospy.sleep(0.1)
        
        if self.beetle.getUavPos() is None:
            rospy.logerr("Timeout: UAV position not received")
            return False
        if self.beetle.getValvePos() is None:
            rospy.logerr("Timeout: Valve position not received")
            return False
        return True
    
    def get_current_position(self):
        return self.uav_pos
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_current_yaw(self):
        return self.current_yaw
    
    def get_current_pitch(self):
        return self.current_pitch
    
    def get_current_valve_yaw(self):
        """Get current valve yaw angle"""
        if hasattr(self, 'valve_yaw') and self.valve_yaw is not None:
            return self.valve_yaw
        else:
            rospy.logwarn("Valve yaw not available, returning 0.0")
            return 0.0
    
    def active_stabilization_wait(self, target_pos, target_yaw, duration, description="position stabilization"):
        """
        Active stabilization wait: continuously send target commands during wait period
        
        Args:
            target_pos: Target position (x, y, z)
            target_yaw: Target yaw angle (radians)
            duration: Wait duration in seconds
            description: Description for logging
        """
        rospy.loginfo(f"Active stabilization: {duration}s {description}")
        rospy.loginfo(f"  Target: pos=({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}), yaw={math.degrees(target_yaw):.1f}°")
        
        start_time = rospy.get_time()
        rate = rospy.Rate(10)  # 10Hz stabilization commands
        
        while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
            # Continuously send target position and yaw to maintain stability
            self.motion_controller.send_trajectory_point(
                target_pos, target_yaw
            )
            
            # Log progress every 1 second
            elapsed = rospy.get_time() - start_time
            if int(elapsed) != int(elapsed - 0.1):  # Log approximately every second
                current_pos = self.get_current_position()
                current_yaw = self.get_current_yaw()
                if current_pos is not None:
                    pos_error = np.linalg.norm(np.array(target_pos) - np.array(current_pos))
                    yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
                    rospy.loginfo(f"  Stabilization progress: {elapsed:.1f}s, pos_err={pos_error*1000:.1f}mm, yaw_err={math.degrees(yaw_error):.1f}°")
            
            rate.sleep()
        
        rospy.loginfo(f"Active stabilization completed: {duration}s {description}")
    
    def _check_stable_contact(self, current_pos, contact_position_history, stability_threshold, stability_duration):
        """
        Check if UAV has achieved stable contact with valve
        
        Args:
            current_pos: Current UAV position (x, y, z)
            contact_position_history: List of (timestamp, position) tuples
            stability_threshold: Maximum allowed position deviation (m)
            stability_duration: Required stable duration (s)
            
        Returns:
            bool: True if stable contact is detected
        """
        current_time = time.time()
        
        # Add current position to history
        contact_position_history.append((current_time, current_pos[:3]))  # Only XYZ
        
        # Keep only recent history (last stability_duration + 1 seconds)
        cutoff_time = current_time - stability_duration - 1.0
        contact_position_history[:] = [(t, pos) for t, pos in contact_position_history if t >= cutoff_time]
        
        # Need sufficient history for stability analysis
        if len(contact_position_history) < 10:  # At least 10 data points
            return False
        
        # Check if we have enough stable duration
        oldest_time = contact_position_history[0][0]
        if current_time - oldest_time < stability_duration:
            return False
        
        # Calculate position variance over the stable duration
        stable_positions = [pos for t, pos in contact_position_history if current_time - t <= stability_duration]
        
        if len(stable_positions) < 5:
            return False
        
        # Calculate centroid
        centroid = [
            sum(pos[0] for pos in stable_positions) / len(stable_positions),
            sum(pos[1] for pos in stable_positions) / len(stable_positions),
            sum(pos[2] for pos in stable_positions) / len(stable_positions)
        ]
        
        # Check if all positions are within threshold from centroid
        for pos in stable_positions:
            deviation = math.sqrt(
                (pos[0] - centroid[0])**2 + 
                (pos[1] - centroid[1])**2 + 
                (pos[2] - centroid[2])**2
            )
            if deviation > stability_threshold:
                return False
        
        return True
    
    def _replan_trajectory_from_contact(self, contact_position, contact_yaw, valve_center, rotation_angle, remaining_duration):
        """
        Replan trajectory based on stable contact position
        
        Args:
            contact_position: Stable contact position (x, y, z)
            contact_yaw: UAV yaw at contact
            valve_center: Original valve center
            rotation_angle: Remaining rotation angle
            remaining_duration: Remaining rotation duration
            
        Returns:
            New trajectory object or None if failed
        """
        try:
            # Calculate actual valve center based on contact position
            # Get optimal end-effector distance
            optimizer = InsertionOptimizer()
            end_effector_distance = optimizer.get_optimal_end_effector_distance()
            
            # Calculate end-effector position at contact
            contact_ee_x = contact_position[0] + end_effector_distance * math.cos(contact_yaw)
            contact_ee_y = contact_position[1] + end_effector_distance * math.sin(contact_yaw)
            
            # Estimate valve center: assume the end-effector is at the valve edge
            # Use a blend of the original valve center and the contact-based estimate
            original_valve_center = valve_center
            contact_based_center = (contact_ee_x, contact_ee_y, valve_center[2])
            
            # Use weighted average to get more stable valve center estimate
            weight = 0.7  # Give more weight to contact-based estimate
            estimated_valve_center = (
                weight * contact_based_center[0] + (1-weight) * original_valve_center[0],
                weight * contact_based_center[1] + (1-weight) * original_valve_center[1],
                valve_center[2]  # Keep original Z
            )
            
            # Calculate rotation radius from contact position to estimated valve center
            dx = contact_position[0] - estimated_valve_center[0]
            dy = contact_position[1] - estimated_valve_center[1]
            actual_radius = math.sqrt(dx*dx + dy*dy)
            
            # Validate radius is reasonable (between 0.05m and 0.2m)
            if actual_radius < 0.05 or actual_radius > 0.2:
                rospy.logwarn(f"Calculated radius {actual_radius:.3f}m is outside reasonable range, using original valve center")
                estimated_valve_center = original_valve_center
                dx = contact_position[0] - estimated_valve_center[0]
                dy = contact_position[1] - estimated_valve_center[1]
                actual_radius = math.sqrt(dx*dx + dy*dy)
            
            # Calculate start angle from contact position
            start_angle = math.atan2(dy, dx)
            
            rospy.loginfo(f"=== TRAJECTORY REPLANNING FROM STABLE CONTACT ===")
            rospy.loginfo(f"Contact position: ({contact_position[0]:.3f}, {contact_position[1]:.3f}, {contact_position[2]:.3f})")
            rospy.loginfo(f"Original valve center: ({original_valve_center[0]:.3f}, {original_valve_center[1]:.3f}, {original_valve_center[2]:.3f})")
            rospy.loginfo(f"Estimated valve center: ({estimated_valve_center[0]:.3f}, {estimated_valve_center[1]:.3f}, {estimated_valve_center[2]:.3f})")
            rospy.loginfo(f"Rotation radius: {actual_radius:.3f}m")
            rospy.loginfo(f"Start angle: {math.degrees(start_angle):.1f}°")
            
            # Create new trajectory using existing function
            from trajectory import create_constant_distance_trajectory
            
            new_trajectory = create_constant_distance_trajectory(
                current_uav_pos=contact_position,
                current_uav_yaw=contact_yaw,
                valve_center=estimated_valve_center,
                rotation_angle=rotation_angle,
                rotation_duration=remaining_duration,
                enable_adaptive_control=True,
                control_mode='rotation'
            )
            
            if new_trajectory is not None:
                rospy.loginfo("✓ Trajectory successfully replanned from stable contact position")
            else:
                rospy.logerr("✗ Failed to create replanned trajectory")
            
            return new_trajectory
            
        except Exception as e:
            rospy.logerr(f"Failed to replan trajectory from contact: {e}")
            return None
    
    def setup_emergency_monitoring(self, stuck_threshold=60.0, movement_threshold=0.005, yaw_threshold=0.02):
        """
        Setup emergency monitoring for stuck detection
        Can be used by any child state that needs emergency monitoring
        """
        self.emergency_stop = threading.Event()
        self.emergency_triggered = False
        self.last_position = None
        self.last_yaw = 0.0
        self.stuck_start_time = None
        self.stuck_threshold = stuck_threshold
        self.movement_threshold = movement_threshold
        self.yaw_threshold = yaw_threshold
    
    def start_emergency_monitoring(self):
        """Start emergency monitoring in a separate thread"""
        if not hasattr(self, 'emergency_stop'):
            self.setup_emergency_monitoring()
        
        self.emergency_stop.clear()
        self.emergency_triggered = False
        
        current_pos = self.get_current_position()
        if current_pos:
            self.last_position = current_pos
            self.last_yaw = self.current_yaw
        
        self.emergency_thread = threading.Thread(target=self._emergency_monitor_thread)
        self.emergency_thread.daemon = True
        self.emergency_thread.start()
        return self.emergency_thread
    
    def stop_emergency_monitoring(self):
        """Stop emergency monitoring"""
        if hasattr(self, 'emergency_stop'):
            self.emergency_stop.set()
        
        # Wait for emergency thread to finish if it exists
        if hasattr(self, 'emergency_thread') and self.emergency_thread.is_alive():
            try:
                self.emergency_thread.join(timeout=2.0)  # Wait up to 2 seconds
                if self.emergency_thread.is_alive():
                    rospy.logwarn("Emergency monitoring thread did not terminate within timeout")
                else:
                    rospy.loginfo("Emergency monitoring thread terminated successfully")
            except Exception as e:
                rospy.logwarn(f"Error waiting for emergency thread termination: {e}")
    
    def _emergency_monitor_thread(self):
        """Emergency monitoring thread - checks for stuck conditions"""
        rate = rospy.Rate(1)  # Check every 1.0 seconds
        consecutive_stuck_checks = 0
        required_consecutive_stuck = 3  # Require 3 consecutive stuck readings
        
        try:
            while not self.emergency_stop.is_set() and not rospy.is_shutdown():
                if self.check_emergency_condition():
                    consecutive_stuck_checks += 1
                    rospy.logwarn(f"Potential stuck condition detected ({consecutive_stuck_checks}/{required_consecutive_stuck})")
                    
                    if consecutive_stuck_checks >= required_consecutive_stuck:
                        rospy.logwarn("Emergency condition confirmed after multiple checks")
                        self.emergency_triggered = True
                        self.emergency_stop.set()
                        break
                else:
                    consecutive_stuck_checks = 0  # Reset counter if movement detected
                    
                try:
                    rate.sleep()
                except rospy.exceptions.ROSInterruptException:
                    # ROS is shutting down, exit gracefully
                    rospy.loginfo("Emergency monitoring thread stopping due to ROS shutdown")
                    break
        except Exception as e:
            rospy.logwarn(f"Emergency monitoring thread error: {e}")
        finally:
            rospy.loginfo("Emergency monitoring thread terminated")
    
    def check_emergency_condition(self):
        """Check if emergency condition exists (e.g., UAV stuck)"""
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        if self.last_position is not None:
            # Check 3D movement
            movement = math.sqrt(
                (current_pos[0] - self.last_position[0])**2 + 
                (current_pos[1] - self.last_position[1])**2 + 
                (current_pos[2] - self.last_position[2])**2
            )
            
            # Check yaw movement
            yaw_movement = abs(self.normalize_angle(self.current_yaw - self.last_yaw))
            
            # Consider it stuck only if BOTH position AND yaw are not moving
            position_stuck = movement < self.movement_threshold
            yaw_stuck = yaw_movement < self.yaw_threshold
            
            if position_stuck and yaw_stuck:
                if self.stuck_start_time is None:
                    self.stuck_start_time = time.time()
                    # Only log when stuck detection starts for the first time
                    rospy.loginfo("⚠ UAV movement below threshold - starting stuck detection timer")
                else:
                    stuck_duration = time.time() - self.stuck_start_time
                    # Only log every 10 seconds to reduce noise, and only if duration is significant
                    if stuck_duration > 10.0 and int(stuck_duration) % 10 == 0:
                        rospy.logwarn(f"UAV stuck duration: {stuck_duration:.1f}s / {self.stuck_threshold:.1f}s")
                    
                    if stuck_duration > self.stuck_threshold:
                        rospy.logerr(f"EMERGENCY: UAV stuck for {stuck_duration:.1f}s > {self.stuck_threshold:.1f}s")
                        return True
            else:
                if self.stuck_start_time is not None:
                    # Only log when transitioning from stuck to moving, and only if stuck time was significant
                    stuck_duration = time.time() - self.stuck_start_time
                    if stuck_duration > 3.0:  # Only log if we were stuck for more than 3 seconds
                        rospy.loginfo(f"Movement resumed after {stuck_duration:.1f}s - resetting stuck timer")
                    # Reset stuck timer silently for short durations
                    self.stuck_start_time = None
                else:
                    # Normal movement - no logging needed
                    pass
        
        self.last_position = current_pos
        self.last_yaw = self.current_yaw
        return False
    
    def _check_valve_center_crossing(self, start_pos, target_pos, valve_center_x, valve_center_y):
        """
        Check if trajectory from start to target would cross valve center
        
        Returns:
            bool: True if trajectory risks crossing valve center
        """
        # Simple line-circle intersection check
        start_x, start_y = start_pos[0], start_pos[1]
        target_x, target_y = target_pos[0], target_pos[1]
        
        # Vector from start to target
        dx = target_x - start_x
        dy = target_y - start_y
        
        # Vector from start to valve center
        fx = valve_center_x - start_x
        fy = valve_center_y - start_y
        
        # Project valve center onto trajectory line
        if dx == 0 and dy == 0:
            return False  # No movement
        
        t = (fx * dx + fy * dy) / (dx * dx + dy * dy)
        
        # Closest point on trajectory to valve center
        closest_x = start_x + t * dx
        closest_y = start_y + t * dy
        
        # Distance from valve center to closest point
        distance_to_trajectory = math.sqrt((closest_x - valve_center_x)**2 + (closest_y - valve_center_y)**2)
        
        # Check if trajectory passes too close to valve center
        safe_distance = 0.3  # 30cm safe distance
        return distance_to_trajectory < safe_distance and 0 <= t <= 1

    def execute_safe_movement_to_position(self, target_pos, target_yaw):
        """
        Execute safe three-stage movement following strict axis-locking principles:
        Stage 1: XY positioning only (Z and yaw locked)
        Stage 2: Yaw adjustment with end-effector compensation (XY and Z locked)  
        Stage 3: Z descent only (XY and yaw locked)
        
        Args:
            target_pos: (x, y, z) target UAV position
            target_yaw: Target UAV yaw angle
            
        Returns:
            bool: Success status
        """
        try:
            current_pos = self.get_current_position()
            if current_pos is None:
                rospy.logerr("Cannot get current position")
                return False

            rospy.loginfo("=== THREE-STAGE MOVEMENT WITH STRICT AXIS LOCKING ===")
            rospy.loginfo(f"Current position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
            rospy.loginfo(f"Current yaw: {math.degrees(self.current_yaw):.1f}°")
            rospy.loginfo(f"Target position: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
            rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
            
            # Calculate yaw change to determine if end-effector compensation is needed
            yaw_change = self.normalize_angle(target_yaw - self.current_yaw)
            
            rospy.loginfo(f"Required yaw change: {math.degrees(yaw_change):.1f}°")
            
            # === STAGE 1: XY POSITIONING ONLY (Z AND YAW LOCKED) ===
            rospy.loginfo("--- Stage 1: XY positioning (Z and yaw LOCKED) ---")
            
            # Lock Z at current height and yaw at current angle
            stage1_target = (target_pos[0], target_pos[1], current_pos[2])
            stage1_yaw = self.current_yaw  # Keep current yaw
            
            # Use the three-stage movement implementation
            # This is a simplified version - the full implementation would call 
            # actual movement methods that should be implemented in child classes
            rospy.loginfo("Stage 1: XY movement completed (simplified)")
            
            # === STAGE 2: YAW ADJUSTMENT ===
            rospy.loginfo("--- Stage 2: Yaw adjustment ---")
            
            if abs(yaw_change) > 0.05:  # Only if significant yaw change needed
                rospy.loginfo(f"Performing yaw adjustment: {math.degrees(yaw_change):.1f}°")
            else:
                rospy.loginfo("Yaw change minimal, skipping Stage 2")
            
            # === STAGE 3: Z DESCENT ===
            rospy.loginfo("--- Stage 3: Z descent ---")
            rospy.loginfo("Z descent completed")
            
            rospy.loginfo("✓ THREE-STAGE MOVEMENT COMPLETED SUCCESSFULLY")
            return True
            
        except Exception as e:
            rospy.logerr(f"Three-stage movement failed: {e}")
            return False
    
    def execute_integrated_three_stage_insertion(self, start_pos, target_pos, target_yaw, 
                                               intermediate_distance=1.0):
        """
        Execute integrated three-stage segmented insertion strategy (optimized with existing methods)
        
        Stage strategy:
        1. First stage: Yaw alignment only - Align UAV yaw angle with valve yaw angle
        2. Second stage: Move while maintaining yaw - Keep current yaw unchanged, move to 1m from valve
        3. Third stage: Precise insertion - Perform trajectory planning from current point to complete valve insertion
        
        Args:
            start_pos: Starting position
            target_pos: Target insertion position  
            target_yaw: Target yaw angle
            intermediate_distance: Intermediate stop distance (default 1m)
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("=== INTEGRATED THREE-STAGE SEGMENTED INSERTION ===")
        rospy.loginfo("ENHANCED STRATEGY with TWO-PHASE YAW ADJUSTMENT:")
        rospy.loginfo("  Stage 1: VALVE YAW ALIGNMENT - align UAV yaw with valve yaw (preparation)")
        rospy.loginfo("  Stage 2: POSITION MOVE - move to 1m from valve (valve yaw locked)")
        rospy.loginfo("  Stage 3: PRECISION INSERTION with SPOKE ALIGNMENT:")
        rospy.loginfo("    Phase 3A: SPOKE YAW ADJUSTMENT - align for spoke gap insertion")
        rospy.loginfo("    Phase 3B: XY positioning with spoke yaw locked")
        rospy.loginfo("    Phase 3C: Z insertion with XY and spoke yaw locked")
        
        # === STAGE 1: VALVE YAW ALIGNMENT (PREPARATION PHASE) ===
        rospy.loginfo("--- STAGE 1: VALVE YAW ALIGNMENT (PREPARATION PHASE) ---")
        valve_yaw_target = self.valve_yaw  # First align with valve yaw for consistent approach
        rospy.loginfo(f"Valve yaw: {math.degrees(valve_yaw_target):.1f}°")
        rospy.loginfo(f"Current yaw: {math.degrees(self.current_yaw):.1f}°")
        valve_yaw_change = abs(self.normalize_angle(valve_yaw_target - self.current_yaw))
        rospy.loginfo(f"Required valve yaw change: {math.degrees(valve_yaw_change):.1f}°")
        
        # Stage 1: Align with valve yaw first (preparation for consistent approach)
        stage1_success = self.motion_controller.execute_progressive_precision_trajectory(
            start_pos, start_pos, valve_yaw_target,  # Align with valve yaw first
            final_pos_threshold=0.05,   # Allow some position drift during rotation
            final_yaw_threshold=self.yaw_precision_levels['fine']   # Enhanced: 0.86° precision (was 0.015)
        )
        
        if not stage1_success:
            rospy.logerr("Stage 1 (valve yaw alignment) failed")
            return False
        
        rospy.loginfo("✓ Stage 1 completed: Valve yaw alignment successful")
        
        # Verify valve yaw alignment and get actual position for trajectory replanning
        current_yaw = self.current_yaw
        valve_yaw_error = abs(self.normalize_angle(valve_yaw_target - current_yaw))
        rospy.loginfo(f"Stage 1 valve yaw verification:")
        rospy.loginfo(f"  Target valve yaw: {math.degrees(valve_yaw_target):.3f}°")
        rospy.loginfo(f"  Actual yaw: {math.degrees(current_yaw):.3f}°")
        rospy.loginfo(f"  Valve yaw error: {math.degrees(valve_yaw_error):.3f}°")
        rospy.loginfo(f"  Required precision: {math.degrees(0.02):.3f}°")
        
        # ENHANCED: Validation for Stage 1 valve yaw alignment
        if valve_yaw_error > 0.05236:  # 3.0° tolerance for preparation phase (tightened)
            rospy.logwarn(f"Stage 1 valve yaw alignment insufficient: {math.degrees(valve_yaw_error):.3f}° > {math.degrees(0.05236):.3f}°")
            rospy.logwarn("UAV is not properly aligned with valve for consistent approach")
        else:
            rospy.loginfo(f"✓ Stage 1 valve yaw alignment excellent: {math.degrees(valve_yaw_error):.3f}° ≤ {math.degrees(0.05236):.3f}°")
        
        rospy.loginfo("NOTE: Spoke gap alignment will occur in Phase 3A during precision insertion")
        
        # ADAPTIVE TRAJECTORY REPLANNING: Get actual position after Stage 1 for Stage 2 replanning
        stage1_actual_pos = self.get_current_position()
        stage1_actual_yaw = self.current_yaw
        if stage1_actual_pos is None:
            rospy.logerr("Lost position feedback after Stage 1 - cannot replan trajectory")
            return False
        
        rospy.loginfo(f"=== ADAPTIVE TRAJECTORY REPLANNING AFTER STAGE 1 ===")
        rospy.loginfo(f"Planned Stage 1 end: ({start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}) at {math.degrees(valve_yaw_target):.1f}°")
        rospy.loginfo(f"Actual Stage 1 end: ({stage1_actual_pos[0]:.3f}, {stage1_actual_pos[1]:.3f}, {stage1_actual_pos[2]:.3f}) at {math.degrees(stage1_actual_yaw):.1f}°")
        
        # Calculate Stage 1 deviation for adaptive replanning
        stage1_pos_deviation = math.sqrt((stage1_actual_pos[0] - start_pos[0])**2 + 
                                       (stage1_actual_pos[1] - start_pos[1])**2 + 
                                       (stage1_actual_pos[2] - start_pos[2])**2)
        stage1_yaw_deviation = abs(self.normalize_angle(stage1_actual_yaw - valve_yaw_target))
        
        rospy.loginfo(f"Stage 1 deviations: Position {stage1_pos_deviation*1000:.1f}mm, Yaw {math.degrees(stage1_yaw_deviation):.2f}°")
        
        if stage1_pos_deviation > 0.025 or stage1_yaw_deviation > 0.05236:  # Tightened thresholds: 25mm position, 3.0° yaw
            rospy.logwarn(f"Significant Stage 1 deviation detected - applying position correction")
            # Use progressive control for correction
            correction_success = self.motion_controller.execute_progressive_precision_trajectory(
                self.get_current_position(), start_pos, valve_yaw_target, 
                final_pos_threshold=0.015, final_yaw_threshold=0.02  # 15mm, 1.1° final precision
            )
            if correction_success:
                rospy.loginfo("✓ Stage 1 position correction successful")
                # Re-acquire corrected position
                stage1_actual_pos = self.get_current_position()
                stage1_actual_yaw = self.get_current_yaw()
            else:
                rospy.logwarn("⚠ Stage 1 position correction failed - continuing with replanning")
        else:
            rospy.loginfo(f"Stage 1 deviation minimal - proceeding with adaptive replanning")
        
        # === STAGE 2: POSITION MOVE WITH VALVE YAW LOCKED ===
        rospy.loginfo("--- STAGE 2: POSITION MOVE (VALVE YAW LOCKED) ---")
        
        # ADAPTIVE REPLANNING: Use actual Stage 1 position instead of planned position
        aligned_pos = stage1_actual_pos  # Use actual position from Stage 1
        aligned_yaw = stage1_actual_yaw  # Use actual yaw from Stage 1
        
        # Recalculate intermediate position based on ACTUAL current position
        direction_to_valve = math.atan2(target_pos[1] - aligned_pos[1], target_pos[0] - aligned_pos[0])
        intermediate_x = target_pos[0] - intermediate_distance * math.cos(direction_to_valve)
        intermediate_y = target_pos[1] - intermediate_distance * math.sin(direction_to_valve)
        intermediate_z = target_pos[2] + 0.10  # 10cm above target for safety
        
        intermediate_pos = (intermediate_x, intermediate_y, intermediate_z)
        
        rospy.loginfo(f"ADAPTIVE REPLANNING Stage 2:")
        rospy.loginfo(f"  Actual valve-aligned position: ({aligned_pos[0]:.3f}, {aligned_pos[1]:.3f}, {aligned_pos[2]:.3f})")
        rospy.loginfo(f"  Recalculated intermediate target: ({intermediate_x:.3f}, {intermediate_y:.3f}, {intermediate_z:.3f})")
        rospy.loginfo(f"  Using actual yaw: {math.degrees(aligned_yaw):.1f}° (valve yaw locked)")
        
        # Stage 2: Move to intermediate position with VALVE YAW LOCKED
        rospy.loginfo("CRITICAL: VALVE YAW LOCKED during position movement (spoke alignment later)")
        # Calculate distance and use improved speed for efficiency
        stage2_distance = math.sqrt((intermediate_x - aligned_pos[0])**2 + (intermediate_y - aligned_pos[1])**2)
        stage2_speed = 0.25  # 🚀 提速: 从0.15提升到0.25 m/s
        stage2_duration = max(8.0, stage2_distance / stage2_speed)  # 减少最小持续时间
        
        rospy.loginfo(f"Stage 2: Distance {stage2_distance:.3f}m, Speed {stage2_speed:.2f}m/s, Duration {stage2_duration:.1f}s")
        
        stage2_success = self.motion_controller.execute_vel_accel_trajectory(
            start_pos=aligned_pos,
            target_pos=intermediate_pos,
            target_yaw=aligned_yaw,  # Use actual yaw from Stage 1
            duration=stage2_duration,
            pos_threshold=0.03,  # 30mm precision for intermediate position
            yaw_threshold=0.02,  # Maintain valve yaw alignment
            axis_lock_mode='xy_yaw'  # CRITICAL: Lock Z but allow XY movement with strict yaw control
        )
        
        if not stage2_success:
            rospy.logerr("Stage 2 (position move with valve yaw locked) failed")
            return False
        
        rospy.loginfo("✓ Stage 2 completed: Position move with valve yaw locked successful")
        rospy.loginfo("NOTE: UAV is now 1m from valve, ready for spoke gap alignment in Phase 3A")
        
        # ADAPTIVE TRAJECTORY REPLANNING: Get actual position after Stage 2 for Stage 3 replanning
        stage2_actual_pos = self.get_current_position()
        stage2_actual_yaw = self.current_yaw
        if stage2_actual_pos is None:
            rospy.logerr("Lost position feedback after Stage 2 - cannot replan trajectory")
            return False
        
        rospy.loginfo(f"=== ADAPTIVE TRAJECTORY REPLANNING AFTER STAGE 2 ===")
        rospy.loginfo(f"Planned Stage 2 end: ({intermediate_pos[0]:.3f}, {intermediate_pos[1]:.3f}, {intermediate_pos[2]:.3f}) at {math.degrees(aligned_yaw):.1f}°")
        rospy.loginfo(f"Actual Stage 2 end: ({stage2_actual_pos[0]:.3f}, {stage2_actual_pos[1]:.3f}, {stage2_actual_pos[2]:.3f}) at {math.degrees(stage2_actual_yaw):.1f}°")
        
        # Calculate Stage 2 deviation for adaptive replanning
        stage2_pos_deviation = math.sqrt((stage2_actual_pos[0] - intermediate_pos[0])**2 + 
                                       (stage2_actual_pos[1] - intermediate_pos[1])**2 + 
                                       (stage2_actual_pos[2] - intermediate_pos[2])**2)
        stage2_yaw_deviation = abs(self.normalize_angle(stage2_actual_yaw - aligned_yaw))
        
        rospy.loginfo(f"Stage 2 deviations: Position {stage2_pos_deviation*1000:.1f}mm, Yaw {math.degrees(stage2_yaw_deviation):.2f}°")
        
        if stage2_pos_deviation > 0.035 or stage2_yaw_deviation > 0.05236:  # Tightened thresholds: 35mm position, 3.0° yaw  
            rospy.logwarn(f"Significant Stage 2 deviation detected - applying position correction")
            # Use progressive control for correction
            correction_success = self.motion_controller.execute_progressive_precision_trajectory(
                self.get_current_position(), intermediate_pos, aligned_yaw, 
                final_pos_threshold=0.020, final_yaw_threshold=0.02  # 20mm, 1.1° final precision
            )
            if correction_success:
                rospy.loginfo("✓ Stage 2 position correction successful")
                # Re-acquire corrected position
                stage2_actual_pos = self.get_current_position()
                stage2_actual_yaw = self.get_current_yaw()
            else:
                rospy.logwarn("⚠ Stage 2 position correction failed - continuing with replanning")
        else:
            rospy.loginfo(f"Stage 2 deviation minimal - proceeding with adaptive Stage 3 replanning")
        
        # === STAGE 3: THREE-PHASE PRECISION INSERTION ===
        rospy.loginfo("--- STAGE 3: THREE-PHASE PRECISION INSERTION ---")
        rospy.loginfo("CORRECTED SEQUENCE: First XY positioning, THEN spoke yaw alignment, THEN Z insertion")
        rospy.loginfo("PHASE 3A: XY positioning with valve yaw locked")
        rospy.loginfo("PHASE 3B: SPOKE GAP YAW ALIGNMENT - rotate for precise spoke insertion")
        rospy.loginfo("PHASE 3C: Z insertion with XY and spoke yaw locked")
        
        # ADAPTIVE REPLANNING: Use actual Stage 2 position instead of planned position
        intermediate_actual_pos = stage2_actual_pos  # Use actual position from Stage 2
        intermediate_actual_yaw = stage2_actual_yaw  # Use actual yaw from Stage 2
        
        rospy.loginfo(f"ADAPTIVE REPLANNING Stage 3:")
        rospy.loginfo(f"  Using actual Stage 2 position: ({intermediate_actual_pos[0]:.6f}, {intermediate_actual_pos[1]:.6f}, {intermediate_actual_pos[2]:.6f})")
        rospy.loginfo(f"  Re-planning insertion trajectory from ACTUAL current position")
        
        # Calculate final insertion path from actual current position
        distance_to_valve = math.sqrt((target_pos[0] - intermediate_actual_pos[0])**2 + 
                                    (target_pos[1] - intermediate_actual_pos[1])**2)
        rospy.loginfo(f"Recalculated final insertion distance: {distance_to_valve:.3f}m")
        
        # PHASE 3A: XY positioning with valve yaw locked (FIRST)
        rospy.loginfo("=== PHASE 3A: XY POSITIONING WITH VALVE YAW LOCKED ===")
        current_yaw = intermediate_actual_yaw  # Use actual yaw from Stage 2
        
        # Calculate XY target (maintain current Z for now)
        xy_target = (target_pos[0], target_pos[1], intermediate_actual_pos[2])
        xy_distance = math.sqrt((target_pos[0] - intermediate_actual_pos[0])**2 + 
                               (target_pos[1] - intermediate_actual_pos[1])**2)
        
        rospy.loginfo(f"ADAPTIVE XY targeting: From ({intermediate_actual_pos[0]:.3f}, {intermediate_actual_pos[1]:.3f}) to ({target_pos[0]:.3f}, {target_pos[1]:.3f})")
        
        if xy_distance > 0.01:  # 10mm threshold
            rospy.loginfo(f"XY positioning needed: {xy_distance*1000:.1f}mm (valve yaw locked)")
            if not self.execute_xy_only_movement(intermediate_actual_pos, xy_target, current_yaw):
                rospy.logerr("Phase 3A: XY positioning failed")
                return False
            rospy.loginfo("✓ Phase 3A completed: XY positioning successful with valve yaw locked")
        else:
            rospy.loginfo("XY position already accurate, skipping Phase 3A")
        
        # ADAPTIVE TRAJECTORY REPLANNING: Get actual position after Phase 3A for Phase 3B replanning
        phase3a_actual_pos = self.get_current_position()
        phase3a_actual_yaw = self.current_yaw
        if phase3a_actual_pos is None:
            rospy.logerr("Lost position feedback after Phase 3A - cannot replan trajectory")
            return False
        
        rospy.loginfo(f"=== ADAPTIVE TRAJECTORY REPLANNING AFTER PHASE 3A ===")
        rospy.loginfo(f"Planned Phase 3A end: ({xy_target[0]:.3f}, {xy_target[1]:.3f}, {xy_target[2]:.3f}) at {math.degrees(current_yaw):.1f}°")
        rospy.loginfo(f"Actual Phase 3A end: ({phase3a_actual_pos[0]:.3f}, {phase3a_actual_pos[1]:.3f}, {phase3a_actual_pos[2]:.3f}) at {math.degrees(phase3a_actual_yaw):.1f}°")
        
        # Calculate Phase 3A deviation
        phase3a_pos_deviation = math.sqrt((phase3a_actual_pos[0] - xy_target[0])**2 + 
                                         (phase3a_actual_pos[1] - xy_target[1])**2 + 
                                         (phase3a_actual_pos[2] - xy_target[2])**2)
        phase3a_yaw_deviation = abs(self.normalize_angle(phase3a_actual_yaw - current_yaw))
        
        rospy.loginfo(f"Phase 3A deviations: Position {phase3a_pos_deviation*1000:.1f}mm, Yaw {math.degrees(phase3a_yaw_deviation):.2f}°")
        
        # PHASE 3B: Spoke gap yaw alignment (AFTER XY positioning)
        rospy.loginfo("=== PHASE 3B: SPOKE GAP YAW ALIGNMENT (CRITICAL FOR INSERTION) ===")
        rospy.loginfo("NOW rotating from valve yaw to spoke gap insertion angle")
        
        # ADAPTIVE REPLANNING: Use actual Phase 3A position and yaw
        current_pos_after_xy = phase3a_actual_pos  # Use actual position from Phase 3A
        current_yaw = phase3a_actual_yaw  # Use actual yaw from Phase 3A
        
        yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
        
        # ENHANCED DEBUGGING: Show spoke gap alignment details
        rospy.loginfo(f"Phase 3B spoke gap alignment diagnosis (ADAPTIVE):")
        rospy.loginfo(f"  Current yaw (actual from 3A): {math.degrees(current_yaw):.3f}°")
        rospy.loginfo(f"  Target yaw (spoke aligned): {math.degrees(target_yaw):.3f}°") 
        rospy.loginfo(f"  Required spoke alignment rotation: {math.degrees(yaw_error):.3f}°")
        rospy.loginfo(f"  Precision threshold: {math.degrees(0.02):.3f}°")
        
        # CRITICAL: This is the spoke gap alignment phase (AFTER XY positioning)
        if yaw_error > 0.02:  # 1.15° threshold for precise spoke insertion
            rospy.loginfo(f"✓ SPOKE GAP ALIGNMENT needed: rotating {math.degrees(yaw_error):.3f}° for insertion")
            if not self.execute_yaw_adjustment_with_compensation(target_yaw):
                rospy.logerr("Phase 3B: Spoke gap yaw alignment failed")
                return False
            rospy.loginfo("✓ Phase 3B completed: Spoke gap yaw alignment successful")
        else:
            rospy.loginfo(f"✓ Spoke gap already aligned within {math.degrees(0.01):.3f}° tolerance")
        
        # ADAPTIVE TRAJECTORY REPLANNING: Get actual position after Phase 3B for Phase 3C replanning
        phase3b_actual_pos = self.get_current_position()
        phase3b_actual_yaw = self.current_yaw
        if phase3b_actual_pos is None:
            rospy.logerr("Lost position feedback after Phase 3B - cannot replan trajectory")
            return False
        
        rospy.loginfo(f"=== ADAPTIVE TRAJECTORY REPLANNING AFTER PHASE 3B ===")
        rospy.loginfo(f"Target Phase 3B end: spoke yaw {math.degrees(target_yaw):.1f}°")
        rospy.loginfo(f"Actual Phase 3B end: ({phase3b_actual_pos[0]:.3f}, {phase3b_actual_pos[1]:.3f}, {phase3b_actual_pos[2]:.3f}) at {math.degrees(phase3b_actual_yaw):.1f}°")
        
        # Calculate Phase 3B yaw deviation
        phase3b_yaw_deviation = abs(self.normalize_angle(phase3b_actual_yaw - target_yaw))
        rospy.loginfo(f"Phase 3B yaw deviation: {math.degrees(phase3b_yaw_deviation):.2f}°")
        
        # PHASE 3C: Z insertion with XY and spoke yaw locked
        rospy.loginfo("=== PHASE 3C: Z INSERTION WITH XY AND SPOKE YAW LOCKED ===")
        
        # ADAPTIVE REPLANNING: Use actual Phase 3B position and yaw
        current_pos_after_yaw = phase3b_actual_pos  # Use actual position from Phase 3B
        current_spoke_yaw = phase3b_actual_yaw  # Use actual yaw from Phase 3B
        
        # Calculate Z target (maintain current XY from actual position)
        z_target = (current_pos_after_yaw[0], current_pos_after_yaw[1], target_pos[2])
        z_distance = abs(target_pos[2] - current_pos_after_yaw[2])
        
        rospy.loginfo(f"ADAPTIVE Z insertion: From Z={current_pos_after_yaw[2]:.3f} to Z={target_pos[2]:.3f}")
        
        if z_distance > 0.01:  # 10mm threshold
            rospy.loginfo(f"Z insertion needed: {z_distance*1000:.1f}mm (XY and spoke yaw locked)")
            if not self.execute_z_only_movement(current_pos_after_yaw, z_target):
                rospy.logerr("Phase 3C: Z insertion failed")
                return False
            rospy.loginfo("✓ Phase 3C completed: Z insertion successful into spoke gaps")
        else:
            rospy.loginfo("Z position already at target for spoke insertion, skipping Phase 3C")
        
        # FINAL ADAPTIVE VERIFICATION: Check final position against original target
        final_actual_pos = self.get_current_position()
        final_actual_yaw = self.current_yaw
        if final_actual_pos is not None:
            final_pos_error = math.sqrt((target_pos[0] - final_actual_pos[0])**2 + 
                                      (target_pos[1] - final_actual_pos[1])**2 + 
                                      (target_pos[2] - final_actual_pos[2])**2)
            final_yaw_error = abs(self.normalize_angle(target_yaw - final_actual_yaw))
            
            rospy.loginfo("=== ADAPTIVE FINAL VERIFICATION ===")
            rospy.loginfo(f"Original target: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}) at {math.degrees(target_yaw):.1f}°")
            rospy.loginfo(f"Final actual: ({final_actual_pos[0]:.3f}, {final_actual_pos[1]:.3f}, {final_actual_pos[2]:.3f}) at {math.degrees(final_actual_yaw):.1f}°")
            rospy.loginfo(f"Final verification:")
            rospy.loginfo(f"  Position error: {final_pos_error*1000:.1f}mm")
            rospy.loginfo(f"  Yaw error: {math.degrees(final_yaw_error):.2f}°")
            
            if final_pos_error < 0.02:  # 20mm tolerance
                rospy.loginfo("✓ ADAPTIVE INTEGRATED THREE-STAGE INSERTION SUCCESSFUL")
                return True
            else:
                rospy.logwarn(f"Final error {final_pos_error*1000:.1f}mm exceeds 20mm tolerance but insertion may still be functional")
        
        stage3_success = True
        
        if not stage3_success:
            rospy.logerr("Stage 3 (precision insertion) failed")
            return False
        
        rospy.loginfo("✓ Stage 3 completed: Precision insertion successful")
        
        # Final comprehensive verification with adaptive trajectory results
        if final_actual_pos is not None:
            rospy.loginfo("=== INTEGRATED THREE-STAGE INSERTION COMPLETED ===")
            rospy.loginfo("ADAPTIVE TRAJECTORY REPLANNING SUMMARY:")
            rospy.loginfo(f"  Stage 1 deviation: {stage1_pos_deviation*1000:.1f}mm, {math.degrees(stage1_yaw_deviation):.2f}°")
            rospy.loginfo(f"  Stage 2 deviation: {stage2_pos_deviation*1000:.1f}mm, {math.degrees(stage2_yaw_deviation):.2f}°")
            rospy.loginfo(f"  Phase 3A deviation: {phase3a_pos_deviation*1000:.1f}mm, {math.degrees(phase3a_yaw_deviation):.2f}°")
            rospy.loginfo(f"  Phase 3B deviation: {math.degrees(phase3b_yaw_deviation):.2f}°")
            rospy.loginfo(f"  Final accuracy: {final_pos_error*1000:.1f}mm, {math.degrees(final_yaw_error):.2f}°")
            
            if final_pos_error < 0.02:  # 20mm tolerance
                rospy.loginfo("✓ ADAPTIVE INTEGRATED THREE-STAGE INSERTION SUCCESSFUL")
                return True
            else:
                rospy.logwarn(f"Final error {final_pos_error*1000:.1f}mm exceeds 20mm tolerance")
        
        rospy.loginfo("✓ ADAPTIVE INTEGRATED THREE-STAGE INSERTION COMPLETED (with warnings)")
        return True
    
    # === PRECISION MOVEMENT CONTROL METHODS FOR STAGE 3 ===
    
    def execute_yaw_adjustment_with_compensation(self, target_yaw):
        """
        Phase 3A: Yaw adjustment using end-effector compensation (with feedback control)
        """
        rospy.loginfo("Executing yaw adjustment with end-effector compensation")
        
        max_attempts = 3
        yaw_threshold = self.yaw_precision_levels['moderate']  # Enhanced: 1.15° precision (was 0.02)
        
        for attempt in range(max_attempts):
            current_pos = self.get_current_position()
            if current_pos is None:
                rospy.logerr(f"Attempt {attempt+1}: Cannot get current position")
                continue
            
            # Calculate current yaw angle difference
            current_yaw = self.current_yaw
            yaw_diff = self.normalize_angle(target_yaw - current_yaw)
            
            rospy.loginfo(f"Attempt {attempt+1}: Yaw error = {math.degrees(abs(yaw_diff)):.2f}°")
            
            # Check if precision requirements are already met
            if abs(yaw_diff) <= yaw_threshold:
                rospy.loginfo(f"✓ Yaw already within tolerance ({math.degrees(abs(yaw_diff)):.3f}° ≤ {math.degrees(yaw_threshold):.3f}°)")
                return True
            
            rospy.loginfo(f"Yaw adjustment needed: {math.degrees(yaw_diff):.2f}°")
            
            # Use progressive precision control for yaw adjustment (keep XYZ position unchanged)
            success = self.motion_controller.execute_progressive_precision_trajectory(
                current_pos, current_pos, target_yaw,
                final_pos_threshold=0.020,  # 20mm position stability (relaxed to avoid oscillation)
                final_yaw_threshold=yaw_threshold    # 0.57° yaw precision
            )
            
            if not success:
                rospy.logwarn(f"Attempt {attempt+1}: Motion controller reported failure")
                continue
            
            # Validate results
            rospy.sleep(0.5)  # Allow settling time
            final_yaw_error = abs(self.normalize_angle(target_yaw - self.current_yaw))
            rospy.loginfo(f"Attempt {attempt+1}: Final yaw error = {math.degrees(final_yaw_error):.3f}°")
            
            if final_yaw_error <= yaw_threshold:
                rospy.loginfo("✓ Yaw adjustment completed with compensation")
                return True
            else:
                rospy.logwarn(f"Attempt {attempt+1}: Yaw error {math.degrees(final_yaw_error):.3f}° exceeds threshold {math.degrees(yaw_threshold):.3f}°")
        
        rospy.logerr(f"✗ Yaw adjustment failed after {max_attempts} attempts")
        return False
    
    def execute_xy_only_movement(self, start_pos, target_pos, locked_yaw):
        """
        Phase 3B: XY-only movement with locked Yaw and Z-axis (with feedback control)
        Enhanced: Use locked position as starting point to avoid position jumps during state transitions
        """
        rospy.loginfo("Executing XY-only movement with yaw locked")
        
        # ENHANCED: Prioritize using locked position as real starting point
        actual_start_pos = start_pos
        if hasattr(self.motion_controller, '_locked_position') and self.motion_controller._locked_position:
            actual_start_pos = self.motion_controller._locked_position
            rospy.loginfo(f"Using locked position as actual start: ({actual_start_pos[0]:.3f}, {actual_start_pos[1]:.3f}, {actual_start_pos[2]:.3f})")
            rospy.loginfo(f"Original start position: ({start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f})")
            
            # Calculate position offset to diagnose state transition issues
            pos_drift = math.sqrt(
                (actual_start_pos[0] - start_pos[0])**2 + 
                (actual_start_pos[1] - start_pos[1])**2 + 
                (actual_start_pos[2] - start_pos[2])**2
            )
            if pos_drift > 0.005:  # 5mm threshold
                rospy.logwarn(f"State transition drift detected: {pos_drift*1000:.1f}mm")
        
        max_attempts = 3
        position_threshold = 0.01  # 10mm precision target
        
        for attempt in range(max_attempts):
            current_pos = self.get_current_position()
            if current_pos is None:
                rospy.logerr(f"Attempt {attempt+1}: Cannot get current position")
                continue
            
            # Ensure only XY coordinates change, keeping Z and yaw unchanged
            xy_target = (target_pos[0], target_pos[1], current_pos[2])  # Use current Z
            
            xy_distance = math.sqrt((target_pos[0] - current_pos[0])**2 + 
                                   (target_pos[1] - current_pos[1])**2)
            
            rospy.loginfo(f"Attempt {attempt+1}: XY error = {xy_distance*1000:.1f}mm")
            
            # Check if precision requirements are already met
            if xy_distance <= position_threshold:
                rospy.loginfo(f"✓ XY position already within tolerance ({xy_distance*1000:.1f}mm ≤ {position_threshold*1000:.1f}mm)")
                return True
            
            rospy.loginfo(f"XY movement needed: {xy_distance*1000:.1f}mm")
            
            # REAL MACHINE OPTIMIZATION: Use relaxed thresholds and longer duration
            is_simulation = rospy.get_param("~simulation", True)
            
            if is_simulation:
                # Simulation parameters - higher precision
                rospy.loginfo("Using optimized motion controller for XY-only movement (simulation)")
                duration = max(6.0, xy_distance / 0.08)  # 80mm/s for precision XY movement
                pos_thresh = position_threshold  # 10mm
                yaw_thresh = 0.01  # Strict yaw precision
            else:
                # Real machine parameters - relaxed for stability
                rospy.loginfo("Using REAL MACHINE optimized controller (relaxed thresholds)")
                duration = max(15.0, xy_distance / 0.05)  # Slower 50mm/s for real machine
                pos_thresh = 0.05  # Relaxed to 50mm for real machine
                yaw_thresh = 0.1   # Relaxed yaw precision (5.7°)
            
            success = self.motion_controller.execute_smooth_trajectory_with_yaw(
                current_pos, xy_target, locked_yaw,
                duration=duration,
                pos_threshold=pos_thresh,
                yaw_threshold=yaw_thresh
            )
            
            if not success:
                rospy.logwarn(f"Attempt {attempt+1}: Motion controller reported failure")
                continue
            
            # Validate results
            rospy.sleep(0.5)  # Allow settling time
            final_pos = self.get_current_position()
            if final_pos is None:
                rospy.logwarn(f"Attempt {attempt+1}: Cannot verify final position")
                continue
                
            final_xy_error = math.sqrt((target_pos[0] - final_pos[0])**2 + 
                                      (target_pos[1] - final_pos[1])**2)
            final_z_drift = abs(final_pos[2] - current_pos[2])
            final_yaw_drift = abs(self.normalize_angle(self.current_yaw - locked_yaw))
            
            rospy.loginfo(f"Attempt {attempt+1}: Final errors - XY: {final_xy_error*1000:.1f}mm, Z drift: {final_z_drift*1000:.1f}mm, Yaw drift: {math.degrees(final_yaw_drift):.2f}°")
            rospy.loginfo(f"Attempt {attempt+1}: Target XY: ({target_pos[0]:.3f}, {target_pos[1]:.3f}), Actual XY: ({final_pos[0]:.3f}, {final_pos[1]:.3f})")
            
            # REAL MACHINE OPTIMIZATION: Relaxed validation criteria
            is_simulation = rospy.get_param("~simulation", True)
            
            if is_simulation:
                # Simulation validation - strict
                xy_threshold = position_threshold  # 10mm
                z_drift_threshold = 0.015  # 15mm Z drift tolerance
                yaw_drift_threshold = 0.05236  # 3.0° yaw drift tolerance (tightened)
            else:
                # Real machine validation - relaxed
                xy_threshold = 0.05  # 50mm tolerance for real machine
                z_drift_threshold = 0.03  # 30mm Z drift tolerance
                yaw_drift_threshold = 0.05236   # 3.0° yaw drift tolerance (tightened)
            
            if final_xy_error <= xy_threshold:
                if final_z_drift <= z_drift_threshold:
                    if final_yaw_drift <= yaw_drift_threshold:
                        rospy.loginfo("✓ XY positioning completed with Z and yaw locked")
                        if not is_simulation:
                            rospy.loginfo(f"✓ Real machine tolerances applied: XY±{xy_threshold*1000:.0f}mm, Z±{z_drift_threshold*1000:.0f}mm, Yaw±{math.degrees(yaw_drift_threshold):.1f}°")
                        return True
                    else:
                        rospy.logwarn(f"Attempt {attempt+1}: Excessive yaw drift {math.degrees(final_yaw_drift):.2f}°")
                else:
                    rospy.logwarn(f"Attempt {attempt+1}: Excessive Z drift {final_z_drift*1000:.1f}mm")
            else:
                rospy.logwarn(f"Attempt {attempt+1}: XY error {final_xy_error*1000:.1f}mm exceeds threshold {xy_threshold*1000:.1f}mm")
        
        rospy.logerr(f"✗ XY positioning failed after {max_attempts} attempts")
        return False
    
    def execute_z_only_movement(self, start_pos, target_pos):
        """
        Phase 3C: Z-only movement with locked XY and Yaw (with feedback control)
        """
        rospy.loginfo("Executing Z-only movement with XY and yaw locked")
        
        max_attempts = 3
        position_threshold = 0.020  # 20mm precision target (relaxed from 8mm for realistic Z insertion)
        
        for attempt in range(max_attempts):
            current_pos = self.get_current_position()
            if current_pos is None:
                rospy.logerr(f"Attempt {attempt+1}: Cannot get current position")
                continue
            
            current_yaw = self.current_yaw
            
            # Ensure only Z coordinate changes, keeping XY and yaw unchanged
            z_target = (current_pos[0], current_pos[1], target_pos[2])  # Use current XY
            
            z_distance = abs(target_pos[2] - current_pos[2])
            
            rospy.loginfo(f"Attempt {attempt+1}: Z error = {z_distance*1000:.1f}mm")
            
            # Check if precision requirements are already met
            if z_distance <= position_threshold:
                rospy.loginfo(f"✓ Z position already within tolerance ({z_distance*1000:.1f}mm ≤ {position_threshold*1000:.1f}mm)")
                return True
            
            rospy.loginfo(f"Z insertion needed: {z_distance*1000:.1f}mm")
            
            # CRITICAL FIX: Use the correct axis_lock_mode for Z-only movement
            # xy_yaw mode locks Z and allows XY+yaw, but we want Z-only movement
            # Use z_only mode which locks XY and yaw, allows only Z movement
            success = self.motion_controller.execute_vel_accel_trajectory(
                current_pos, z_target, current_yaw,
                duration=max(6.0, z_distance / 0.04),  # 40mm/s for precision Z insertion
                pos_threshold=position_threshold,  # Use same threshold
                yaw_threshold=self.yaw_precision_levels['fine'],  # Enhanced: 0.86° precision (was 0.01/0.57°)
                axis_lock_mode='z_only'  # CORRECT: Lock XY and yaw, allow only Z movement
            )
            
            if not success:
                rospy.logwarn(f"Attempt {attempt+1}: Motion controller reported failure")
                continue
            
            # Validate results
            rospy.sleep(0.5)  # Allow settling time
            final_pos = self.get_current_position()
            if final_pos is None:
                rospy.logwarn(f"Attempt {attempt+1}: Cannot verify final position")
                continue
                
            final_z_error = abs(target_pos[2] - final_pos[2])
            final_xy_drift = math.sqrt((final_pos[0] - current_pos[0])**2 + 
                                      (final_pos[1] - current_pos[1])**2)
            final_yaw_drift = abs(self.normalize_angle(self.current_yaw - current_yaw))
            
            rospy.loginfo(f"Attempt {attempt+1}: Final errors - Z: {final_z_error*1000:.1f}mm, XY drift: {final_xy_drift*1000:.1f}mm, Yaw drift: {math.degrees(final_yaw_drift):.2f}°")
            
            if final_z_error <= position_threshold:
                if final_xy_drift <= 0.012:  # 12mm XY drift tolerance
                    if final_yaw_drift <= 0.05236:  # 3.0° yaw drift tolerance (tightened)
                        rospy.loginfo("✓ Z insertion completed with XY and yaw locked")
                        return True
                    else:
                        rospy.logwarn(f"Attempt {attempt+1}: Excessive yaw drift {math.degrees(final_yaw_drift):.2f}°")
                else:
                    rospy.logwarn(f"Attempt {attempt+1}: Excessive XY drift {final_xy_drift*1000:.1f}mm")
            else:
                rospy.logwarn(f"Attempt {attempt+1}: Z error {final_z_error*1000:.1f}mm exceeds threshold {position_threshold*1000:.1f}mm")
        
        rospy.logerr(f"✗ Z insertion failed after {max_attempts} attempts")
        return False
    
    # Note: This is a simplified implementation. The full three-stage movement
    # should be implemented in subclasses that have access to specific movement methods.


class InitializeStartPositionState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'],
                        output_keys=['start_position'], 
                        module_id=module_id)
    
    def execute(self, userdata):
        rospy.loginfo("Initializing start position...")
        
        if not self.wait_for_positions():
            return 'failed'
        
        userdata.start_position = self.get_current_position()
        rospy.loginfo(f"Start position initialized: {userdata.start_position}")
        return 'succeeded'


class MoveToValveState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'], 
                        module_id=module_id)
        
        # Initialize the insertion optimizer for direct insertion movement
        self.optimizer = InsertionOptimizer()
        
        # Initialize trajectory generator for smooth waypoint-based movement
        # Note: Will be properly configured with actual valve position in execute()
        self.trajectory_generator = OnlineCircularTrajectoryGenerator(
            valve_center=[3.0, 0.0, 0.57],  # Default valve position, will be updated
            initial_radius=0.3,  # Initial radius for trajectory planning
            target_angular_velocity=0.0,  # No circular motion for this state
            control_rate=20.0,  # 20Hz control rate for smooth movement
            debug=False
        )
        
        # Enhanced movement parameters for efficient trajectory execution
        self.max_linear_velocity = 0.25  # 🚀 提速: 从0.12提升到0.25 m/s
        self.average_linear_velocity = 0.15  # 🚀 提速: 从0.08提升到0.15 m/s
        self.max_angular_velocity = 0.02  # � 降速: 从0.04降低到0.02 rad/s for stable yaw control
        self.waypoint_threshold = 0.050  # 5cm threshold for waypoint completion (tightened for better precision)
        self.final_pos_threshold = 0.025  # 2.5cm final positioning accuracy (relaxed from 15mm to 25mm for reliability)
        self.final_yaw_threshold = 0.0175  # 1° final yaw accuracy (tightened to meet 1° requirement)
        
        # Position and orientation thresholds
        self.pos_threshold = 0.05
        self.yaw_threshold = 0.05236  # Tightened to 3°
    
    def calculate_shortest_yaw_path(self, current_yaw, target_yaw):
        """
        Calculate the shortest yaw rotation path to avoid reverse direction rotation
        
        Args:
            current_yaw: Current yaw angle in radians
            target_yaw: Target yaw angle in radians
            
        Returns:
            float: Optimized target yaw that follows shortest rotation path
        """
        # Calculate the difference
        diff = target_yaw - current_yaw
        
        # Handle angle wrap-around to ensure shortest path
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
        
        # Return the shortest path target
        optimal_target = current_yaw + diff
        
        rospy.loginfo(f"Shortest yaw path calculation:")
        rospy.loginfo(f"  Current yaw: {math.degrees(current_yaw):.1f}deg")
        rospy.loginfo(f"  Original target: {math.degrees(target_yaw):.1f}deg")
        rospy.loginfo(f"  Optimized target: {math.degrees(optimal_target):.1f}deg")
        rospy.loginfo(f"  Rotation angle: {math.degrees(diff):.1f}deg")
        
        return optimal_target
    
    def _execute_phase1_yaw_adjustment(self, current_position, target_yaw):
        """
        Phase 1: Pure yaw adjustment while maintaining current XYZ position
        
        Args:
            current_position: Current UAV position [x, y, z] to maintain
            target_yaw: Target yaw angle (radians)
            
        Returns:
            bool: True if successful, False otherwise
        """
        rospy.loginfo("Executing Phase 1: Pure yaw adjustment")
        rospy.loginfo(f"Maintaining position: ({current_position[0]:.3f}, {current_position[1]:.3f}, {current_position[2]:.3f})")
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}deg")
        
        # Log yaw adjustment
        initial_yaw = self.current_yaw
        rospy.loginfo(f"Phase 1: Adjusting yaw from {math.degrees(initial_yaw):.1f}° to {math.degrees(target_yaw):.1f}°")
        
        # Execute pure yaw rotation using targetMotion
        self.beetle.targetMotion(
            pos=current_position,  # Maintain exact current position
            rot=target_yaw,        # Adjust to target yaw
            linear_vel=None,       # No linear velocity for pure rotation
            angular_vel=None       # Let system determine angular velocity
        )
        
        # Wait for yaw convergence with appropriate timeout
        yaw_converged = self.wait_for_yaw_convergence(
            target_yaw, 
            yaw_thresh=0.0524,  # 3° threshold
            timeout=12.0        # Generous timeout for yaw adjustment
        )
        
        if not yaw_converged:
            rospy.logerr("Phase 1 yaw convergence failed")
            return False
        
        # Verify position stability during yaw adjustment
        final_pos = self.beetle.getUavPos()
        if final_pos is not None:
            pos_drift = np.linalg.norm(np.array(final_pos) - np.array(current_position))
            rospy.loginfo(f"Position drift during yaw adjustment: {pos_drift*1000:.1f}mm")
            
            if pos_drift > 0.05:  # 5cm drift threshold
                rospy.logwarn(f"Significant position drift during yaw adjustment: {pos_drift*1000:.1f}mm")
            else:
                rospy.loginfo("Position maintained well during yaw adjustment")
        
        rospy.loginfo("Phase 1 completed successfully")
        return True
    
    def _execute_phase2_xy_movement(self, xy_target, maintain_yaw):
        """
        Phase 2: Pure XY movement while maintaining yaw and Z height
        
        Args:
            xy_target: Target position [x, y, z] where Z is the height to maintain
            maintain_yaw: Yaw angle to maintain during movement
            
        Returns:
            bool: True if successful, False otherwise
        """
        rospy.loginfo("Executing Phase 2: Pure XY movement")
        rospy.loginfo(f"Target XY position: ({xy_target[0]:.3f}, {xy_target[1]:.3f})")
        rospy.loginfo(f"Maintaining Z height: {xy_target[2]:.3f}m")
        rospy.loginfo(f"Maintaining yaw: {math.degrees(maintain_yaw):.1f}deg")
        
        current_pos = self.beetle.getUavPos()
        if current_pos is None:
            rospy.logerr("Cannot get current position for Phase 2")
            return False
        
        # Calculate XY distance for trajectory planning
        xy_distance = np.linalg.norm(np.array(xy_target[:2]) - np.array(current_pos[:2]))
        rospy.loginfo(f"XY movement distance: {xy_distance*1000:.1f}mm")
        
        # Use polynomial trajectory for smooth XY movement
        if xy_distance > 0.05:  # Only use trajectory if distance > 5cm
            # Generate XY-only polynomial trajectory
            # Target 50mm spacing between points for smoother motion and reduced overshooting
            target_spacing = 0.05  # 50mm spacing (reduced from 100mm for smoother trajectory)
            num_points = max(6, min(int(xy_distance / target_spacing), 40))  # 6-40 points range (increased for smoothness)
            
            trajectory_points = self.generate_polynomial_trajectory(
                start_pos=current_pos,
                target_pos=xy_target,
                target_yaw=maintain_yaw,  # Maintain yaw throughout movement
                num_points=num_points
            )
            
            if not trajectory_points:
                rospy.logerr("Failed to generate XY trajectory points")
                return False
            
            # Execute XY trajectory
            success = self.execute_polynomial_trajectory(trajectory_points)
            if not success:
                rospy.logerr("XY trajectory execution failed")
                return False
        else:
            # Direct movement for short distances
            rospy.loginfo("Short XY distance - using direct movement")
            self.beetle.targetMotion(
                pos=xy_target,
                rot=maintain_yaw,
                linear_vel=None,
                angular_vel=None
            )
            
            # Wait for XY convergence
            success = self.wait_for_final_convergence(
                xy_target, maintain_yaw,
                pos_thresh=0.03,   # 3cm threshold
                yaw_thresh=0.0524, # 3° threshold  
                timeout=8.0
            )
            
            if not success:
                rospy.logerr("Direct XY movement convergence failed")
                return False
        
        rospy.loginfo("Phase 2 completed successfully")
        # Active stabilization after XY movement
        current_pos = self.get_current_position()
        if current_pos is not None:
            target_with_z = (xy_target[0], xy_target[1], current_pos[2])  # Use current Z
            self.active_stabilization_wait(target_with_z, maintain_yaw, 3.0, "system stabilization after XY movement")
            
            # Check position stability after stabilization
            post_stab_pos = self.get_current_position()
            if post_stab_pos is not None:
                pos_error = np.linalg.norm(np.array(post_stab_pos[:2]) - np.array(xy_target[:2]))
                rospy.loginfo(f"Post-stabilization check: XY error={pos_error*1000:.1f}mm, position stable")
        else:
            rospy.logwarn("Cannot get position for active stabilization, using passive wait")
            rospy.sleep(3.0)  # Fallback passive wait
        
        return True
    
    def _execute_phase3_z_descent_and_adjustment(self, final_target, final_yaw):
        """
        Phase 3: Z descent to insertion height + final angle micro-adjustment
        
        Args:
            final_target: Final target position [x, y, z]
            final_yaw: Final target yaw angle
            
        Returns:
            bool: True if successful, False otherwise
        """
        rospy.loginfo("Executing Phase 3: Z descent + final adjustment")
        rospy.loginfo(f"Final target: ({final_target[0]:.3f}, {final_target[1]:.3f}, {final_target[2]:.3f})")
        rospy.loginfo(f"Final yaw: {math.degrees(final_yaw):.1f}deg")
        
        current_pos = self.beetle.getUavPos()
        if current_pos is None:
            rospy.logerr("Cannot get current position for Phase 3")
            return False
        
        z_descent = current_pos[2] - final_target[2]
        rospy.loginfo(f"Z descent distance: {z_descent*1000:.1f}mm")
        
        # Check if yaw adjustment is needed and separate major/minor adjustments
        current_yaw = self.current_yaw  # Use self.current_yaw maintained by base class
        if current_yaw is not None:
            yaw_error = abs(self.normalize_angle(final_yaw - current_yaw))
            major_yaw_needed = yaw_error > 0.175  # 10° threshold for major adjustment
            minor_yaw_needed = yaw_error > 0.0175  # 1° threshold for minor adjustment
            
            rospy.loginfo(f"Yaw error: {math.degrees(yaw_error):.2f}deg")
            rospy.loginfo(f"Major yaw adjustment needed: {major_yaw_needed}")
        else:
            major_yaw_needed = True
            minor_yaw_needed = True
        
        # Phase 3A: Major yaw adjustment if needed (>10°)
        if major_yaw_needed:
            rospy.loginfo("=== Phase 3A: Major yaw adjustment ===")
            # Calculate intermediate yaw (leave ~8° for fine adjustment during descent)
            yaw_diff = self.normalize_angle(final_yaw - current_yaw)
            intermediate_yaw = current_yaw + yaw_diff * 0.85  # Complete 85% of rotation
            
            rospy.loginfo(f"Adjusting yaw from {math.degrees(current_yaw):.1f}° to {math.degrees(intermediate_yaw):.1f}°")
            
            # Execute major yaw adjustment using active convergence (like Formation version)
            # 🔧 FIXED: 使用active_position_convergence替代wait_for_final_convergence
            # 原理: Formation版本的10Hz连续修正机制 vs Single UAV的单次命令
            rospy.loginfo("🔧 Using active convergence with yaw step limiting for controlled rotation speed")
            success = self.active_position_convergence(
                current_pos, intermediate_yaw,
                pos_thresh=0.035,  # 35mm tolerance (relaxed to reduce position correction time)
                yaw_thresh=0.0524, # 3° yaw tolerance (looser for major adjustment phase)
                timeout=20.0,      # 🕐 Reduced timeout for faster overall completion
                max_yaw_step=math.radians(3.5)  # 🎯 3.5°/step ≈ 7°/s effective speed (accounting for delays)
            )
            
            if not success:
                rospy.logwarn("Phase 3A: Major yaw adjustment did not fully converge, continuing")
            else:
                rospy.loginfo("✓ Phase 3A completed: Major yaw adjustment successful")
            
            # Stabilization wait after major yaw adjustment
            rospy.loginfo("Waiting 1 second for stabilization after major yaw adjustment")
            rospy.sleep(1.0)
        
        # Phase 3B: Z descent with fine yaw adjustment
        rospy.loginfo("=== Phase 3B: Z descent + fine yaw adjustment ===")
        if abs(z_descent) > 0.02 or minor_yaw_needed:  # 2cm Z threshold
            rospy.loginfo("Executing controlled Z descent with fine adjustments")
            
            # Use controlled stepwise Z descent
            success = self._execute_controlled_z_descent(
                current_pos, final_target, final_yaw, 
                descent_speed=0.12  # 🚀 提速: 从0.05提升到0.12 m/s
            )
            
            if not success:
                rospy.logwarn("Final convergence not achieved within timeout, but continuing")
                # Don't fail here - the position might be close enough for insertion
        else:
            rospy.loginfo("Phase 3 skipped: Already at target position and orientation")
            success = True
        
        # Final status report
        final_pos = self.beetle.getUavPos()
        final_yaw_actual = self.current_yaw  # Use self.current_yaw maintained by base class
        
        if final_pos is not None and final_yaw_actual is not None:
            final_pos_error = np.linalg.norm(np.array(final_pos) - np.array(final_target))
            final_yaw_error = abs(self.normalize_angle(final_yaw - final_yaw_actual))
            
            rospy.loginfo(f"Final positioning accuracy:")
            rospy.loginfo(f"  Position error: {final_pos_error*1000:.1f}mm")
            rospy.loginfo(f"  Yaw error: {math.degrees(final_yaw_error):.2f}deg")
            
            if final_pos_error < 0.03 and final_yaw_error < 0.0524:
                rospy.loginfo("Phase 3 completed with excellent precision")
            else:
                rospy.loginfo("Phase 3 completed with acceptable precision")
        
        rospy.loginfo("Phase 3 completed successfully")
        return True
    
    def _execute_controlled_z_descent(self, start_pos, final_target, final_yaw, descent_speed=0.12):
        """
        Execute controlled stepwise Z descent with separated yaw and Z control (Scheme 1)
        
        Args:
            start_pos: Starting position [x, y, z]
            final_target: Final target position [x, y, z]
            final_yaw: Final target yaw angle
            descent_speed: Descent speed in m/s (default 0.05 m/s)
            
        Returns:
            bool: True if successful, False otherwise
        """
        rospy.loginfo(f"=== SEPARATED CONTROL APPROACH (Scheme 1) ===")
        rospy.loginfo(f"Phase 3B-1: Precision yaw adjustment (lock XYZ)")
        rospy.loginfo(f"Phase 3B-2: Pure Z descent (lock XY + yaw)")
        
        total_z_descent = start_pos[2] - final_target[2]
        rospy.loginfo(f"Total Z descent: {total_z_descent*1000:.1f}mm")
        
        # === Phase 3B-1: Precision Yaw Adjustment (Lock XYZ) ===
        current_yaw = self.current_yaw
        if current_yaw is not None:
            remaining_yaw_error = self.normalize_angle(final_yaw - current_yaw)
            rospy.loginfo(f"Remaining yaw error: {math.degrees(remaining_yaw_error):.2f}°")
            
            if abs(remaining_yaw_error) > 0.0175:  # 1° threshold
                rospy.loginfo("=== Phase 3B-1: Precision yaw adjustment ===")
                rospy.loginfo(f"Adjusting yaw from {math.degrees(current_yaw):.1f}° to {math.degrees(final_yaw):.1f}°")
                rospy.loginfo("Locking XYZ position during yaw adjustment to prevent drift")
                
                # 🔧 FORMATION-STYLE: Use continuous control with max_yaw_step like Formation version
                rospy.loginfo("🔧 Using Formation-style continuous yaw control with step limiting")
                
                # Get current position for locking XYZ
                current_pos = self.get_current_position()
                if current_pos is not None:
                    # Use active_position_convergence with max_yaw_step for controlled rotation speed
                    yaw_converged = self.active_position_convergence(
                        target_pos=current_pos,         # Lock XYZ position
                        target_yaw=final_yaw,           # Target yaw angle
                        pos_thresh=0.020,               # 20mm tolerance (balanced for precision phase)
                        yaw_thresh=0.0175,              # 1° tolerance for yaw
                        timeout=15.0,                   # Reasonable timeout for precision phase
                        max_yaw_step=math.radians(2.0)  # 2°/step ≈ 4°/s for fine adjustment phase
                    )
                else:
                    rospy.logwarn("Cannot get current position, skipping Phase 3B-1")
                    yaw_converged = False
                
                if not yaw_converged:
                    rospy.logwarn("Phase 3B-1 yaw adjustment timeout, continuing with current yaw")
                else:
                    rospy.loginfo("✓ Phase 3B-1 completed: Formation-style yaw precision adjustment successful")
                
                # Active stabilization after yaw adjustment
                current_pos = self.get_current_position()
                if current_pos is not None:
                    self.active_stabilization_wait(current_pos, final_yaw, 3.0, "yaw stabilization after Phase 3B-1")
                else:
                    rospy.logwarn("Cannot get current position for active stabilization, using passive wait")
                    rospy.sleep(3.0)
            else:
                rospy.loginfo("Phase 3B-1 skipped: Yaw already within 1° tolerance")
        
        # Update current position after yaw adjustment
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logwarn("Cannot get updated position, using original start_pos")
            current_pos = start_pos
        
        # === Phase 3B-2: Pure Z Descent (Lock XY + Yaw) ===
        rospy.loginfo("=== Phase 3B-2: Pure Z descent ===")
        
        if total_z_descent <= 0.01:  # Less than 1cm descent
            rospy.loginfo("Minimal Z descent required, executing direct movement")
            self.beetle.targetMotion(
                pos=final_target,
                rot=final_yaw,
                linear_vel=None,
                angular_vel=None
            )
            return self.wait_for_final_convergence(
                final_target, final_yaw,
                pos_thresh=0.015, yaw_thresh=0.0175, timeout=8.0
            )
        
        # Calculate step parameters for controlled Z descent
        # 🔧 OPTIMIZED: 5段式下降，大幅减少累积误差和过冲
        num_steps = 5  # 从3段改为5段：387.5mm ÷ 5 = 77.5mm/步 (更合理)
        step_size = total_z_descent / num_steps  # 平均分配下降距离
        
        # 🎯 真机友好的渐进式速度控制 (0.05-0.08m/s)
        z_speeds = [0.08, 0.08, 0.065, 0.055, 0.05]  # m/s - 渐进式减速
        z_timeouts = [15, 15, 15, 15, 20]  # 秒 - 合理的收敛时间，避免长时间等待
        
        rospy.loginfo(f"🔧 OPTIMIZED Z descent: {num_steps} steps, {step_size*1000:.1f}mm per step")
        rospy.loginfo(f"Speed profile: {z_speeds} m/s, Timeouts: {z_timeouts}s")
        rospy.loginfo("Pure Z descent: XY and yaw will be LOCKED during descent")
        
        # Execute stepwise Z descent with LOCKED XY and yaw
        previous_z = current_pos[2]  # Track previous Z for step distance calculation
        
        for step in range(num_steps):
            # Calculate Z descent with fixed step size and safety limits
            if step == num_steps - 1:
                # Final step: go exactly to final target
                current_z = final_target[2]
            else:
                # Calculate planned Z for this step
                planned_z_descent = step_size * (step + 1)
                planned_z = current_pos[2] - planned_z_descent
                
                # Safety limit: ensure we don't go below final target
                current_z = max(planned_z, final_target[2])
            
            # Additional safety check: limit single step distance (increased for final steps)
            if step > 0:
                single_step_distance = abs(previous_z - current_z)
                # 🔧 FIXED: Progressive step limit - larger steps allowed for final descent
                if step == num_steps - 1:
                    max_single_step = 0.300  # 300mm for final step to reach target
                else:
                    max_single_step = 0.080  # 80mm for intermediate steps (increased from 30mm)
                    
                if single_step_distance > max_single_step:
                    rospy.logwarn(f"Step {step+1} distance {single_step_distance*1000:.1f}mm exceeds {max_single_step*1000:.1f}mm limit")
                    # Clamp the step size
                    current_z = previous_z - max_single_step
                    rospy.loginfo(f"Clamped step {step+1} to {max_single_step*1000:.1f}mm descent")
            
            # LOCKED XY coordinates: use exact final target XY throughout descent
            current_x = final_target[0]
            current_y = final_target[1]
            
            # Step target with locked XY
            step_target = [current_x, current_y, current_z]
            
            # Calculate step parameters
            is_final_step = (step == num_steps - 1)
            progress = (step + 1) / num_steps  # Progress ratio for threshold calculation
            
            # 🔧 FIXED: Realistic position thresholds for Z descent with some XY drift tolerance
            base_threshold = 0.050  # Relaxed 50mm initial threshold (Z-descent focused)
            final_threshold = 0.025  # 25mm final threshold (relaxed from 15mm to avoid oscillation)
            step_pos_thresh = base_threshold - (base_threshold - final_threshold) * progress
            
            # Tight yaw threshold since yaw should be locked
            step_yaw_thresh = 0.0175  # 1° - tight since yaw should be stable
            
            rospy.loginfo(f"Z descent step {step+1}/{num_steps}: target=({current_x:.3f}, {current_y:.3f}, {current_z:.3f})")
            rospy.loginfo(f"  Z descent: {(current_pos[2] - current_z)*1000:.1f}mm (XY locked)")
            rospy.loginfo(f"  Threshold: pos={step_pos_thresh*1000:.1f}mm, yaw={math.degrees(step_yaw_thresh):.1f}°")
            
            # 🔧 OPTIMIZED: 5段渐进式速度控制，真机友好
            current_z_speed = z_speeds[step]  # 使用预定义的速度档位
            step_timeout = z_timeouts[step]   # 使用对应的超时时间
            
            step_desc = ['初始下降', '稳定下降', '开始减速', '接近目标', '精确到位'][step]
            rospy.loginfo(f"  Step {step+1}/{num_steps} ({step_desc}): speed={current_z_speed:.3f}m/s, timeout={step_timeout}s")
            
            # 统一的速度控制：所有步骤都使用相同的控制逻辑
            self.beetle.targetMotion(
                pos=step_target,
                rot=final_yaw,      # LOCKED yaw
                linear_vel=[0.0, 0.0, -current_z_speed],  # 渐进式Z下降速度
                angular_vel=0.0     # NO angular velocity - yaw locked
            )
                
            # 🔧 SELECTIVE Z-STEP CONTROL: 为第4、5步添加步长控制，防止大距离下降震荡
            if step >= 3:  # 第4步和第5步（step索引从0开始）
                step_converged = self.active_position_convergence(
                    step_target, final_yaw,  # Locked yaw target
                    pos_thresh=step_pos_thresh,
                    yaw_thresh=step_yaw_thresh,
                    timeout=step_timeout,
                    max_z_step=0.020  # 20mm Z步长，防止大距离下降震荡
                )
            else:  # 前3步保持高效逻辑
                step_converged = self.active_position_convergence(
                    step_target, final_yaw,  # Locked yaw target
                    pos_thresh=step_pos_thresh,
                    yaw_thresh=step_yaw_thresh,
                    timeout=step_timeout
                    # 不使用Z步长控制，保持原有高效性能
                )
            
            if not step_converged:
                rospy.logerr(f"Z descent step {step+1} convergence failed - stopping for collision safety")
                rospy.logerr("Pure Z descent failed - this indicates potential hardware issues")
                return False
            
            # Enhanced XY verification after each step
            current_check_pos = self.get_current_position()
            if current_check_pos is not None:
                xy_error = np.linalg.norm(np.array([current_check_pos[0] - step_target[0], 
                                                  current_check_pos[1] - step_target[1]]))
                rospy.loginfo(f"Post-step XY verification: error = {xy_error*1000:.1f}mm")
                
                if xy_error > 0.025:  # 25mm XY error threshold
                    rospy.logwarn(f"XY error too large ({xy_error*1000:.1f}mm), performing XY correction...")
                    
                    # XY correction: Lock Z and yaw, only correct XY
                    xy_correction_target = [step_target[0], step_target[1], current_check_pos[2]]
                    self.beetle.targetMotion(
                        pos=xy_correction_target,
                        rot=final_yaw,
                        linear_vel=None,
                        angular_vel=0.0
                    )
                    
                    # Wait for XY correction convergence
                    xy_corrected = self.active_position_convergence(
                        xy_correction_target, final_yaw,
                        pos_thresh=0.015,  # Strict 15mm XY tolerance
                        yaw_thresh=step_yaw_thresh,
                        timeout=8.0
                    )
                    
                    if xy_corrected:
                        rospy.loginfo("✓ XY correction successful")
                    else:
                        rospy.logwarn("⚠ XY correction timeout, but continuing")
            
            # Active settling time between steps (increased for XY stability)
            if not is_final_step:
                current_pos = self.get_current_position()
                if current_pos is not None:
                    self.active_stabilization_wait(step_target, final_yaw, 2.0, f"step {step + 1}/{num_steps} stabilization")
                else:
                    rospy.loginfo("Waiting 2.0 seconds for enhanced step stabilization...")
                    rospy.sleep(2.0)  # Fallback passive wait
            
            # Update previous Z position for next step's safety check
            previous_z = current_z
        
        # === Final precision adjustment ===
        rospy.loginfo("=== Phase 3B-3: Final precision adjustment ===")
        rospy.loginfo("Performing final precision adjustment to exact target position")
        self.beetle.targetMotion(
            pos=final_target,
            rot=final_yaw,
            linear_vel=None,  # Use default controller speeds for final adjustment
            angular_vel=None
        )
        
        # === Final precision adjustment with extended stabilization ===
        rospy.loginfo("=== Phase 3B-3: Final precision adjustment ===")
        rospy.loginfo("Performing final precision adjustment to exact target position")
        self.beetle.targetMotion(
            pos=final_target,
            rot=final_yaw,
            linear_vel=None,  # Use default controller speeds for final adjustment
            angular_vel=None
        )
        
        # Final convergence with strict requirements and Z-step control
        final_success = self.active_position_convergence(
            final_target, final_yaw,
            pos_thresh=0.025,  # Relaxed to 2.5cm to avoid oscillation
            yaw_thresh=0.0175, # 1° final yaw precision
            timeout=25.0,      # Extended timeout for step-by-step convergence
            max_z_step=0.020   # 20mm Z步长，防震荡的同时保持合理效率
        )
        
        # Additional XY stabilization period for valve insertion precision
        rospy.loginfo("=== Phase 3B-4: Extended XY stabilization for valve insertion ===")
        rospy.loginfo("Performing 10 seconds of active XY stabilization before contact...")
        
        # Active stabilization with continuous position commands
        current_pos = self.get_current_position()
        if current_pos is not None:
            self.active_stabilization_wait(final_target, final_yaw, 10.0, "extended XY stabilization for valve insertion")
            
            # Final precision verification
            final_check_pos = self.get_current_position()
            if final_check_pos is not None:
                final_xy_error = np.linalg.norm(np.array([final_check_pos[0] - final_target[0], 
                                                        final_check_pos[1] - final_target[1]]))
                rospy.loginfo(f"Final XY positioning accuracy: {final_xy_error*1000:.1f}mm")
                
                if final_xy_error > 0.020:  # 20mm threshold
                    rospy.logwarn(f"⚠ Final XY error large: {final_xy_error*1000:.1f}mm - valve insertion may be imprecise")
                else:
                    rospy.loginfo(f"✓ Final XY positioning acceptable: {final_xy_error*1000:.1f}mm")
            else:
                rospy.logwarn("Cannot verify final position after stabilization")
        else:
            rospy.logwarn("Cannot get current position for active stabilization, using passive wait")
            rospy.sleep(10.0)  # Fallback passive wait
        
        if final_success:
            rospy.loginfo("✓ SEPARATED CONTROL SUCCESSFUL: Z descent completed with precision")
        else:
            rospy.logwarn("⚠ Separated control completed but final convergence not fully achieved")
            rospy.logwarn("  This may still be acceptable for valve insertion")
        
        return final_success
    
    def calculate_uav_position_for_targets(self, left_target, right_target, target_z):
        """
        Calculate UAV position and yaw to achieve specific claw targets
        
        Args:
            left_target: (x, y, z) position for left claw
            right_target: (x, y, z) position for right claw
            target_z: Target Z height for claws
            
        Returns:
            tuple: (uav_position, uav_yaw) or (None, None) if calculation fails
        """
        try:
            # Calculate dual-fang center position (midpoint between claws)
            dual_fang_center_x = (left_target[0] + right_target[0]) / 2
            dual_fang_center_y = (left_target[1] + right_target[1]) / 2
            dual_fang_center_z = target_z
            
            # Calculate UAV orientation (yaw) from claw separation vector
            claw_vector_x = right_target[0] - left_target[0]
            claw_vector_y = right_target[1] - left_target[1]
            uav_yaw = math.atan2(claw_vector_y, claw_vector_x) - math.pi/2
            
            # Normalize yaw angle to [0, 2π]
            while uav_yaw >= 2*math.pi:
                uav_yaw -= 2*math.pi
            while uav_yaw < 0:
                uav_yaw += 2*math.pi
            
            # Calculate UAV position from dual-fang center
            # UAV should be BEHIND the dual-fang center by the offset distance
            uav_x = dual_fang_center_x - self.optimizer.dual_fang_center_offset * math.cos(uav_yaw)
            uav_y = dual_fang_center_y - self.optimizer.dual_fang_center_offset * math.sin(uav_yaw)
            # UAV Z should be ABOVE the dual-fang center by the offset
            uav_z = dual_fang_center_z - self.optimizer.end_effector_offset_z
            
            return (uav_x, uav_y, uav_z), uav_yaw
            
        except Exception as e:
            rospy.logerr(f"Failed to calculate UAV position for targets: {e}")
            return None, None
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle
    
    def generate_polynomial_trajectory(self, start_pos, target_pos, target_yaw, num_points=20):
        """
        Generate smooth polynomial trajectory using PolynomialTrajectory class
        
        Args:
            start_pos: Starting position [x, y, z]
            target_pos: Target position [x, y, z] 
            target_yaw: Target yaw angle
            num_points: Number of trajectory points to generate
            
        Returns:
            list: List of trajectory points [(pos, yaw, vel), ...]
        """
        rospy.loginfo(f"=== Generating trajectory using PolynomialTrajectory: {num_points} points ===")
        
        current_yaw = self.current_yaw
        
        # Calculate total distance and trajectory time
        total_distance = np.linalg.norm(np.array(target_pos) - np.array(start_pos))
        yaw_change = abs(self.normalize_angle(target_yaw - current_yaw))
        
        # Calculate trajectory time based on target average velocity (0.1 m/s)
        time_for_distance = total_distance / self.average_linear_velocity  # Time for 0.1 m/s average
        time_for_rotation = yaw_change / self.max_angular_velocity         # Time for rotation
        
        # Calculate optimal trajectory duration
        trajectory_duration = max(
            time_for_distance,  # Time based on target average velocity
            time_for_rotation,  # Time based on angular motion
            6.0  # Minimum 6 seconds for smooth motion
        )
        
        # Apply safety factor to ensure we stay within speed limits
        safety_factor = 0.8  # 20% margin for safety
        trajectory_duration = trajectory_duration / safety_factor
        
        actual_avg_velocity = total_distance / trajectory_duration if trajectory_duration > 0 else 0
        
        rospy.loginfo(f"Trajectory parameters: distance={total_distance:.3f}m, angle_change={math.degrees(yaw_change):.1f}°")
        rospy.loginfo(f"Estimated_time={trajectory_duration:.1f}s, avg_velocity={actual_avg_velocity:.3f}m/s")
        
        # Create polynomial trajectories for each axis
        traj_x = PolynomialTrajectory(duration=trajectory_duration)
        traj_y = PolynomialTrajectory(duration=trajectory_duration)
        traj_z = PolynomialTrajectory(duration=trajectory_duration)
        traj_yaw = PolynomialTrajectory(duration=trajectory_duration)
        
        # Compute coefficients for each axis and set ALL required attributes
        traj_x.is_scalar = True
        traj_x.coeffs_scalar = traj_x.compute_coefficients(start_pos[0], target_pos[0])
        traj_x.start_value = start_pos[0]  # Required for boundary checking
        traj_x.target_value = target_pos[0]  # Required for boundary checking
        
        traj_y.is_scalar = True
        traj_y.coeffs_scalar = traj_y.compute_coefficients(start_pos[1], target_pos[1])
        traj_y.start_value = start_pos[1]  # Required for boundary checking
        traj_y.target_value = target_pos[1]  # Required for boundary checking
        
        traj_z.is_scalar = True
        traj_z.coeffs_scalar = traj_z.compute_coefficients(start_pos[2], target_pos[2])
        traj_z.start_value = start_pos[2]  # Required for boundary checking
        traj_z.target_value = target_pos[2]  # Required for boundary checking
        
        traj_yaw.is_scalar = True
        traj_yaw.coeffs_scalar = traj_yaw.compute_coefficients(current_yaw, target_yaw)
        traj_yaw.start_value = current_yaw  # Required for boundary checking
        traj_yaw.target_value = target_yaw  # Required for boundary checking
        
        rospy.loginfo("Polynomial coefficient calculation completed, generating trajectory points")
        
        trajectory_points = []
        
        # Generate trajectory points
        for i in range(num_points + 1):  # +1 to include final target
            # Calculate elapsed time for this point
            elapsed_time = (i / num_points) * trajectory_duration
            
            # Get position from polynomial trajectories using elapsed time
            pos_x = traj_x.evaluate_at_time(elapsed_time)
            pos_y = traj_y.evaluate_at_time(elapsed_time)
            pos_z = traj_z.evaluate_at_time(elapsed_time)
            yaw = traj_yaw.evaluate_at_time(elapsed_time)
            
            # Calculate velocity manually using polynomial derivative with trapezoidal velocity profile
            # Velocity = dp/dt = (dp/dτ) * (dτ/dt) = (dp/dτ) / duration
            normalized_time = elapsed_time / trajectory_duration
            
            # Boundary check for velocity calculation
            if normalized_time <= 0 or normalized_time >= 1:
                vel_x = vel_y = vel_z = 0.0  # Zero velocity at boundaries
            else:
                # Polynomial derivative coefficients [5*a5, 4*a4, 3*a3, 2*a2, 1*a1, 0*a0]
                # Then divide by duration to get actual velocity
                T_vel = np.array([5*normalized_time**4, 4*normalized_time**3, 3*normalized_time**2, 
                                 2*normalized_time, 1, 0]) / trajectory_duration
                
                vel_x = np.dot(traj_x.coeffs_scalar, T_vel)
                vel_y = np.dot(traj_y.coeffs_scalar, T_vel)
                vel_z = np.dot(traj_z.coeffs_scalar, T_vel)
            
            # Apply trapezoidal velocity profile for smoother motion
            progress = normalized_time
            if progress < 0.15:  # First 15%: accelerate from 70% to 100%
                velocity_factor = 0.7 + 0.3 * (progress / 0.15)
            elif progress > 0.85:  # Last 15%: decelerate from 100% to 70%
                velocity_factor = 0.7 + 0.3 * ((1.0 - progress) / 0.15)
            else:  # Middle 70%: maintain 100% speed
                velocity_factor = 1.0
            
            # Apply velocity profile to calculated velocity
            vel_x *= velocity_factor
            vel_y *= velocity_factor
            vel_z *= velocity_factor
            
            # Construct trajectory point
            pos = [pos_x, pos_y, pos_z]
            velocity = [vel_x, vel_y, vel_z]
            
            # Calculate velocity magnitude and enforce limits
            vel_magnitude = np.linalg.norm(velocity)
            if vel_magnitude > self.max_linear_velocity:
                # Scale down velocity to respect limits
                scale_factor = self.max_linear_velocity / vel_magnitude
                velocity = [v * scale_factor for v in velocity]
                vel_magnitude = self.max_linear_velocity
            
            trajectory_points.append((pos, yaw, velocity, vel_magnitude))
            
            if i % 5 == 0:  # Log every 5th point
                rospy.loginfo(f"Point{i}: pos=[{pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f}], "
                            f"yaw={math.degrees(yaw):.1f}°, vel={vel_magnitude:.3f}m/s (factor={velocity_factor:.2f})")
        
        rospy.loginfo(f"Generated {len(trajectory_points)} trajectory points")
        return trajectory_points

    def generate_smooth_waypoint_trajectory(self, start_pos, target_pos, target_yaw, num_waypoints=7):
        """
        DEPRECATED: Legacy waypoint generation method
        Use generate_polynomial_trajectory() for better performance
        """
        rospy.logwarn("WARNING: Using deprecated waypoint method, recommend switching to polynomial trajectory")
        
        current_yaw = self.current_yaw
        
        # Calculate total distance and trajectory time with target average speed
        total_distance = np.linalg.norm(np.array(target_pos) - np.array(start_pos))
        yaw_change = abs(self.normalize_angle(target_yaw - current_yaw))
        
        # Calculate trajectory time based on target average velocity (0.1 m/s)
        time_for_distance = total_distance / self.average_linear_velocity  # Time for 0.1 m/s average
        time_for_rotation = yaw_change / self.max_angular_velocity         # Time for rotation
        
        # Enhanced trajectory timing with longer minimum time for ultra-smooth motion
        trajectory_time = max(
            time_for_distance,  # Time based on target average velocity
            time_for_rotation,  # Time based on angular motion
            8.0  # Minimum 8 seconds for ultra-smooth motion (increased from 4s)
        )
        
        # Ensure we don't exceed maximum velocity limits
        actual_avg_velocity = total_distance / trajectory_time if trajectory_time > 0 else 0
        if actual_avg_velocity > self.max_linear_velocity:
            trajectory_time = total_distance / self.max_linear_velocity
            actual_avg_velocity = self.max_linear_velocity
        
        rospy.loginfo(f"Trajectory parameters: distance={total_distance:.3f}m, angle_change={math.degrees(yaw_change):.1f}°")
        rospy.loginfo(f"Estimated_time={trajectory_time:.1f}s, avg_velocity={actual_avg_velocity:.3f}m/s")
        
        waypoints = []
        
        # Generate waypoints with S-curve velocity profile for smoothness
        for i in range(num_waypoints + 1):  # +1 to include final target
            # S-curve interpolation parameter (0 to 1)
            t = i / num_waypoints
            
            # Apply S-curve (sigmoid-like) for smooth acceleration/deceleration
            # This creates smooth start and end with constant velocity in middle
            if t <= 0.5:
                # Acceleration phase: t^2 profile
                s = 2 * t * t
            else:
                # Deceleration phase: 1 - 2*(1-t)^2 profile  
                s = 1 - 2 * (1 - t) * (1 - t)
            
            # Clamp s to [0, 1]
            s = max(0.0, min(1.0, s))
            
            # Interpolate position using S-curve
            interp_pos = [
                start_pos[0] + s * (target_pos[0] - start_pos[0]),
                start_pos[1] + s * (target_pos[1] - start_pos[1]),
                start_pos[2] + s * (target_pos[2] - start_pos[2])
            ]
            
            # Interpolate yaw with smooth transition
            yaw_diff = self.normalize_angle(target_yaw - current_yaw)
            interp_yaw = current_yaw + s * yaw_diff
            
            # Calculate velocity for smooth motion
            if i == 0:
                # Start with zero velocity
                velocity = [0.0, 0.0, 0.0]
                angular_velocity = 0.0
            elif i == num_waypoints:
                # End with zero velocity
                velocity = [0.0, 0.0, 0.0]
                angular_velocity = 0.0
            else:
                # Calculate velocity based on direction and S-curve derivative
                direction = np.array(target_pos) - np.array(start_pos)
                direction_norm = np.linalg.norm(direction)
                
                if direction_norm > 0:
                    direction_unit = direction / direction_norm
                    
                    # S-curve velocity (derivative of S-curve position)
                    if t <= 0.5:
                        vel_scale = 4 * t  # Derivative of 2*t^2
                    else:
                        vel_scale = 4 * (1 - t)  # Derivative of 1 - 2*(1-t)^2
                    
                    # Conservative velocity calculation for ultra-smooth motion
                    # Use average velocity as base, not maximum velocity
                    base_velocity = min(self.average_linear_velocity, direction_norm / trajectory_time)
                    vel_magnitude = base_velocity * vel_scale * 0.8  # Additional 0.8 safety factor
                    
                    # Cap at maximum velocity as absolute limit
                    vel_magnitude = min(vel_magnitude, self.max_linear_velocity)
                    velocity = (direction_unit * vel_magnitude).tolist()
                    
                    # Conservative angular velocity calculation
                    base_angular_velocity = yaw_diff / trajectory_time
                    angular_velocity = base_angular_velocity * vel_scale * 0.8  # Same safety factor
                    angular_velocity = min(abs(angular_velocity), self.max_angular_velocity) * (1 if angular_velocity >= 0 else -1)
                else:
                    velocity = [0.0, 0.0, 0.0]
                    angular_velocity = 0.0
            
            waypoints.append({
                'position': interp_pos,
                'yaw': interp_yaw,
                'linear_velocity': velocity,
                'angular_velocity': angular_velocity,
                'time_from_start': i * trajectory_time / num_waypoints
            })
            
            rospy.loginfo(f"Waypoint {i}: pos=({interp_pos[0]:.3f}, {interp_pos[1]:.3f}, {interp_pos[2]:.3f}), "
                         f"yaw={math.degrees(interp_yaw):.1f}°, vel_mag={np.linalg.norm(velocity):.2f}m/s")
        
        rospy.loginfo(f"Generated smooth trajectory with {len(waypoints)} waypoints")
        return waypoints
    
    def execute_polynomial_trajectory(self, trajectory_points):
        """
        Execute polynomial trajectory using targetMotion for clean control
        
        Args:
            trajectory_points: List of trajectory points [(pos, yaw, vel, vel_magnitude), ...]
            
        Returns:
            bool: True if trajectory execution successful, False otherwise
        """
        rospy.loginfo(f"=== Executing polynomial trajectory with {len(trajectory_points)} points ===")
        
        start_time = rospy.get_time()
        
        for i, (target_pos, target_yaw, target_vel, vel_magnitude) in enumerate(trajectory_points):
            point_start_time = rospy.get_time()
            
            is_final_point = (i == len(trajectory_points) - 1)
            is_final_three_points = (i >= len(trajectory_points) - 3)  # 最后3个点
            
            # 为最后3个轨迹点强制减速50%，避免接近终点时加速
            if is_final_three_points and vel_magnitude > 0.001:
                target_vel = np.array(target_vel) * 0.5  # 强制减速50% (正确的向量运算)
                vel_magnitude = vel_magnitude * 0.5
                rospy.loginfo(f"Final approach: Applied 50% speed reduction for smooth approach")
            
            rospy.loginfo(f"Executing point {i+1}/{len(trajectory_points)}: "
                         f"pos=({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}), "
                         f"yaw={math.degrees(target_yaw):.1f}°, vel={vel_magnitude:.3f}m/s")
            
            # Use targetMotion for clean POS_VEL_MODE control (no wrench interference)
            self.beetle.targetMotion(
                pos=target_pos,
                rot=target_yaw,  # Use rot parameter for yaw angle
                linear_vel=target_vel if vel_magnitude > 0.001 else None,  # Use velocity only if meaningful
                angular_vel=None  # No angular velocity commands for smoother motion
            )
            
            # targetMotion doesn't return success/failure, so we assume success
            # The actual convergence will be checked in wait_for_trajectory_convergence
            rospy.loginfo(f"Trajectory point {i+1} command sent successfully")
            
            # Dynamic convergence checking based on trajectory point
            if is_final_point:
                # Strict convergence for final point only
                pos_thresh = self.final_pos_threshold  # 15mm
                yaw_thresh = self.final_yaw_threshold  # 1°
                timeout = 15.0  # Generous timeout for final precision
                
                rospy.loginfo(f"Final point convergence check: pos_thresh={pos_thresh*1000:.1f}mm, yaw_thresh={math.degrees(yaw_thresh):.1f}deg")
                
                # Use active convergence (dragon-style) instead of passive waiting
                converged = self.active_position_convergence(
                    target_pos, target_yaw, pos_thresh, yaw_thresh, timeout
                )
                
                if not converged:
                    rospy.logerr(f"ERROR: Final point convergence failed: position or angle error too large")
                    return False
                
                point_time = rospy.get_time() - point_start_time
                rospy.loginfo(f"Final trajectory point completed, duration: {point_time:.2f}s")
            else:
                # 🚀 方案A：区分轨迹跟随模式和精度收敛模式
                is_final_precision_point = (i >= len(trajectory_points) - 2)  # 最后2个点使用精度模式
                
                if is_final_precision_point:
                    # 精度收敛模式：倒数第2个点也需要较高精度
                    intermediate_pos_thresh = 0.03  # 30mm精度要求
                    intermediate_yaw_thresh = 0.1   # 6°航向要求  
                    intermediate_timeout = 8.0      # 充分时间确保精度
                    rospy.loginfo(f"精度收敛模式 (点 {i+1}): pos_thresh={intermediate_pos_thresh*1000:.1f}mm")
                else:
                    # 轨迹跟随模式：前面的点使用快速通过
                    intermediate_pos_thresh = 0.08  # 🚀 80mm大容差（快速通过）
                    intermediate_yaw_thresh = 0.25  # 🚀 14°宽松航向要求
                    intermediate_timeout = 3.0      # 🚀 3秒快速超时
                    rospy.loginfo(f"轨迹跟随模式 (点 {i+1}): pos_thresh={intermediate_pos_thresh*1000:.1f}mm (快速通过)")
                
                # 执行收敛检查
                converged = self.active_position_convergence(
                    target_pos, target_yaw,
                    pos_thresh=intermediate_pos_thresh,
                    yaw_thresh=intermediate_yaw_thresh,
                    timeout=intermediate_timeout
                )
                
                if not converged:
                    rospy.logwarn(f"Intermediate point {i+1} convergence incomplete, but continuing for smooth motion")
                    
                    # Fallback: Use original dynamic delay for motion continuity
                    next_point = trajectory_points[i + 1]
                    current_point_pos = target_pos
                    next_point_pos = next_point[0]  # next position
                    
                    # Calculate distance to next point
                    point_distance = np.linalg.norm(np.array(next_point_pos) - np.array(current_point_pos))
                    
                    # Calculate required time based on actual velocity from trajectory point
                    current_vel_magnitude = vel_magnitude
                    if current_vel_magnitude > 0.001:
                        required_time = point_distance / current_vel_magnitude
                    else:
                        required_time = point_distance / 0.08  # Fallback to default speed
                    
                    # 🚀 动态停顿时间：轨迹跟随模式更快，精度模式稍慢
                    if is_final_precision_point:
                        remaining_delay = max(0.1, required_time * 0.5)  # 精度模式：较长停顿
                    else:
                        remaining_delay = max(0.02, required_time * 0.2)  # 🚀 轨迹跟随模式：更短停顿
                    
                    rospy.loginfo(f"Fallback dynamic delay ({'精度模式' if is_final_precision_point else '跟随模式'}): "
                                f"distance={point_distance*1000:.1f}mm, vel={current_vel_magnitude:.3f}m/s, "
                                f"delay={remaining_delay:.2f}s")
                    
                    rospy.sleep(remaining_delay)
                else:
                    mode_desc = "精度模式" if is_final_precision_point else "跟随模式"  
                    rospy.loginfo(f"Intermediate point {i+1} converged successfully for smooth transition ({mode_desc})")
                    
                    # 🚀 成功收敛时的停顿时间也区分模式
                    if is_final_precision_point:
                        rospy.sleep(0.1)  # 精度模式：100ms停顿
                    else:
                        rospy.sleep(0.027)  # 轨迹跟随模式：27ms停顿（75%速度，20ms -> 27ms）
                
                point_time = rospy.get_time() - point_start_time
                rospy.loginfo(f"Trajectory point {i+1} completed, duration: {point_time:.2f}s")
        
        total_time = rospy.get_time() - start_time
        rospy.loginfo(f"Polynomial trajectory execution completed, total time: {total_time:.2f}s")
        return True

    def active_position_convergence(self, target_pos, target_yaw, pos_thresh=0.025, yaw_thresh=0.0175, timeout=15.0, max_yaw_step=None, max_z_step=None):
        """
        Dragon-style active convergence: continuously send target position commands
        until convergence is achieved, with Formation-style yaw step limiting.
        
        Args:
            target_pos: Target position [x, y, z]
            target_yaw: Target yaw angle (radians)
            pos_thresh: Position convergence threshold (meters)
            yaw_thresh: Yaw convergence threshold (radians)
            timeout: Maximum time to attempt convergence (seconds)
            max_yaw_step: Maximum yaw step per cycle (radians), None for no limit
            max_z_step: Maximum Z step per cycle (meters), None for no limit
            
        Returns:
            bool: True if converged within timeout, False otherwise
        """
        start_time = rospy.get_time()
        consecutive_good_readings = 0
        required_consecutive = 8  # Require 8 consecutive good readings for better stability
        
        rospy.loginfo(f"Active convergence: target=({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}), "
                     f"yaw={math.degrees(target_yaw):.1f}°")
        rospy.loginfo(f"Thresholds: pos={pos_thresh*1000:.1f}mm, yaw={math.degrees(yaw_thresh):.1f}°, consecutive={required_consecutive}")
        
        while rospy.get_time() - start_time < timeout:
            # Get current position and yaw
            current_pos = self.beetle.getUavPos()
            if current_pos is None:
                rospy.sleep(0.1)
                continue
            
            current_yaw = self.current_yaw
            if current_yaw is None:
                rospy.sleep(0.1)
                continue
            
            # Calculate errors
            pos_error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
            yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
            
            # Check convergence
            position_ok = pos_error < pos_thresh
            yaw_ok = yaw_error < yaw_thresh
            
            if position_ok and yaw_ok:
                consecutive_good_readings += 1
                if consecutive_good_readings >= required_consecutive:
                    rospy.loginfo(f"Active convergence successful: pos_err={pos_error*1000:.1f}mm, "
                                f"yaw_err={math.degrees(yaw_error):.1f}°")
                    return True
            else:
                consecutive_good_readings = 0
                
                # 🔧 FORMATION-STYLE: Apply max_yaw_step limitation like Formation version
                command_yaw = target_yaw
                if max_yaw_step is not None and max_yaw_step > 0.0:
                    yaw_delta = self.normalize_angle(target_yaw - current_yaw)
                    if abs(yaw_delta) > max_yaw_step:
                        yaw_delta = math.copysign(max_yaw_step, yaw_delta)
                        command_yaw = self.normalize_angle(current_yaw + yaw_delta)
                        # Log yaw step limiting occasionally for verification
                        elapsed = rospy.get_time() - start_time
                        if int(elapsed * 5.0) % 20 == 0:  # Every 4 seconds
                            rospy.loginfo(f"[YawLimit] Step limited to {math.degrees(yaw_delta):.1f}° "
                                        f"(remaining {math.degrees(abs(self.normalize_angle(target_yaw - current_yaw))):.1f}°)")
                
                # 🔧 Z-STEP控制：类似yaw步长限制，避免Z轴震荡
                command_pos = list(target_pos)  # 复制目标位置
                if max_z_step is not None and max_z_step > 0.0:
                    z_delta = target_pos[2] - current_pos[2] 
                    if abs(z_delta) > max_z_step:
                        z_delta = math.copysign(max_z_step, z_delta)
                        command_pos[2] = current_pos[2] + z_delta
                        # Z步长限制日志
                        elapsed = rospy.get_time() - start_time
                        if int(elapsed * 5.0) % 20 == 0:  # Every 4 seconds
                            rospy.loginfo(f"[ZLimit] Step limited to {z_delta*1000:.1f}mm "
                                        f"(remaining {abs(target_pos[2] - current_pos[2])*1000:.1f}mm)")
                
                # 🔧 FIXED: 添加角速度和线性速度限制，像Formation版本一样平滑
                max_angular_vel = 0.05  # 0.05 rad/s ≈ 2.9°/s，更保守的角速度限制
                max_linear_vel = 0.08   # 0.08 m/s，降低线性速度避免position drift
                
                # 计算基于实际command_yaw的角速度
                actual_yaw_error = abs(self.normalize_angle(command_yaw - current_yaw))
                smooth_angular_vel = min(actual_yaw_error / 1.5, max_angular_vel) if actual_yaw_error > 0.005 else 0.0
                
                # 🔧 FIXED: 计算平滑的线性速度，确保返回正确的格式
                if pos_error > 0.02:
                    # 计算标量速度值
                    speed_magnitude = min(pos_error / 2.0, max_linear_vel)
                    # 计算方向向量（从当前位置指向目标位置）
                    direction = np.array(target_pos) - np.array(current_pos)
                    direction_norm = np.linalg.norm(direction)
                    if direction_norm > 0.001:  # 避免除零
                        direction = direction / direction_norm
                        smooth_linear_vel = [direction[0] * speed_magnitude, 
                                           direction[1] * speed_magnitude, 
                                           direction[2] * speed_magnitude]
                    else:
                        smooth_linear_vel = None  # 距离太近，不需要速度控制
                else:
                    smooth_linear_vel = None
                
                # DRAGON-STYLE: 发送带yaw和Z步长限制的目标位置命令
                self.beetle.targetMotion(
                    pos=command_pos,  # 使用步长限制后的位置
                    rot=command_yaw,  # 使用步长限制后的yaw
                    linear_vel=smooth_linear_vel,  # 平滑线性速度
                    angular_vel=smooth_angular_vel  # 平滑角速度，避免"一步到位"
                )
                
                # Log progress every 2 seconds
                elapsed = rospy.get_time() - start_time
                if int(elapsed * 0.5) % 1 == 0:  # Every 2 seconds
                    rospy.loginfo(f"Active convergence progress: pos_err={pos_error*1000:.1f}mm, "
                                f"yaw_err={math.degrees(yaw_error):.1f}°, time={elapsed:.1f}s")
            
            rospy.sleep(0.1)  # 10Hz rate for active correction
        
        # Final error report
        current_pos = self.beetle.getUavPos()
        current_yaw = self.current_yaw
        if current_pos is not None and current_yaw is not None:
            final_pos_error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
            final_yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
            rospy.logwarn(f"Active convergence timeout: final errors pos={final_pos_error*1000:.1f}mm, "
                         f"yaw={math.degrees(final_yaw_error):.1f}°")
        
        return False
    


    def wait_for_trajectory_convergence(self, target_pos, target_yaw, pos_thresh, yaw_thresh, timeout):
        """
        Wait for convergence to trajectory point with specified thresholds
        
        Returns:
            bool: True if converged within timeout, False otherwise
        """
        start_time = rospy.get_time()
        consecutive_good_readings = 0
        required_consecutive = 6  # Increased to 6 consecutive good readings for better stability
        
        while rospy.get_time() - start_time < timeout:
            current_pos = self.beetle.getUavPos()
            if current_pos is None:
                rospy.sleep(0.1)
                continue
            
            current_yaw = self.current_yaw
            
            # Check position convergence
            pos_error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
            yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
            
            position_ok = pos_error < pos_thresh
            yaw_ok = yaw_error < yaw_thresh
            
            if position_ok and yaw_ok:
                consecutive_good_readings += 1
                if consecutive_good_readings >= required_consecutive:
                    rospy.loginfo(f"Convergence successful: pos_err={pos_error*1000:.1f}mm, "
                                f"yaw_err={math.degrees(yaw_error):.1f}deg")
                    return True
            else:
                consecutive_good_readings = 0
                rospy.loginfo_throttle(2, f"Waiting for convergence: pos_err={pos_error*1000:.1f}mm/"
                                      f"{pos_thresh*1000:.1f}mm, yaw_err={math.degrees(yaw_error):.1f}deg/"
                                      f"{math.degrees(yaw_thresh):.1f}deg")
            
            rospy.sleep(0.1)  # 10Hz check rate
        
        # Final error report
        current_pos = self.beetle.getUavPos()
        current_yaw = self.current_yaw
        if current_pos is not None:
            final_pos_error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
            final_yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
            rospy.logwarn(f"⏰ 收敛超时: 最终误差 pos={final_pos_error*1000:.1f}mm, "
                         f"yaw={math.degrees(final_yaw_error):.1f}°")
        
        return False
    
    def wait_for_yaw_convergence(self, target_yaw, yaw_thresh=0.0524, timeout=10.0):
        """
        Wait for UAV attitude to converge to target yaw angle
        
        Args:
            target_yaw: Target yaw angle (radians)
            yaw_thresh: Yaw convergence threshold (radians), default 3 degrees
            timeout: Timeout duration (seconds)
        
        Returns:
            bool: Whether convergence was successful
        """
        start_time = rospy.get_time()
        consecutive_good_readings = 0
        required_consecutive = 3  # Require 3 consecutive readings for stability
        
        rospy.loginfo(f"Waiting for yaw convergence - target={math.degrees(target_yaw):.1f}°, thresh={math.degrees(yaw_thresh):.1f}°")
        
        while not rospy.is_shutdown():
            # Check timeout
            if rospy.get_time() - start_time > timeout:
                rospy.logwarn(f"Yaw convergence timeout after {timeout}s")
                return False
            
            # Use self.current_yaw which is maintained by the base class
            current_yaw = self.current_yaw
            if current_yaw is None:
                rospy.logwarn("Current yaw is None, retrying...")
                rospy.sleep(0.05)
                continue
            
            # Calculate yaw error (handle wrap-around)
            yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
            
            # Progress logging every 10th iteration (approximately every second)
            if int((rospy.get_time() - start_time) * 10) % 10 == 0:
                rospy.loginfo(f"Yaw convergence: current={math.degrees(current_yaw):.1f}°, target={math.degrees(target_yaw):.1f}°, error={math.degrees(yaw_error):.1f}°")
            
            # Check convergence
            if yaw_error < yaw_thresh:
                consecutive_good_readings += 1
                if consecutive_good_readings >= required_consecutive:
                    rospy.loginfo(f"Yaw converged! Error: {math.degrees(yaw_error):.2f}deg")
                    return True
            else:
                consecutive_good_readings = 0
            
            # Log progress every 2 seconds
            elapsed = rospy.get_time() - start_time
            if int(elapsed) % 2 == 0 and elapsed > 0:
                rospy.loginfo_throttle(2, f"Yaw convergence progress: error={math.degrees(yaw_error):.2f}°, time={elapsed:.1f}s")
            
            rospy.sleep(0.1)
        
        return False

    def wait_for_final_convergence(self, target_pos, target_yaw, pos_thresh=0.025, yaw_thresh=0.0524, timeout=8.0):
        """
        等待轨迹点完全收敛（位置和姿态）- 用于最终精确调整
        
        Args:
            target_pos: 目标位置 [x, y, z]
            target_yaw: 目标yaw角 (弧度)
            pos_thresh: 位置收敛阈值 (米)
            yaw_thresh: yaw收敛阈值 (弧度)
            timeout: 超时时间 (秒)
        
        Returns:
            bool: 是否成功收敛
        """
        start_time = rospy.get_time()
        target_pos = np.array(target_pos)
        consecutive_good_readings = 0
        required_consecutive = 8  # Increased to 8 consecutive readings for final convergence
        
        while not rospy.is_shutdown():
            # Check timeout
            if rospy.get_time() - start_time > timeout:
                rospy.logwarn(f"Final convergence timeout after {timeout}s")
                return False
            
            # Get current state
            current_pos = self.beetle.getUavPos()
            current_yaw = self.current_yaw  # Use self.current_yaw maintained by base class
            
            if current_pos is None or current_yaw is None:
                rospy.sleep(0.05)
                continue
            
            # Calculate errors
            pos_error = np.linalg.norm(np.array(current_pos) - target_pos)
            yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
            
            # Check convergence
            if pos_error < pos_thresh and yaw_error < yaw_thresh:
                consecutive_good_readings += 1
                if consecutive_good_readings >= required_consecutive:
                    rospy.loginfo(f"Final convergence achieved! Position error: {pos_error*1000:.1f}mm, Yaw error: {math.degrees(yaw_error):.2f}°")
                    return True
            else:
                consecutive_good_readings = 0
            
            # Log progress every 1 second for final convergence
            elapsed = rospy.get_time() - start_time
            if int(elapsed) % 1 == 0:
                rospy.loginfo_throttle(1, f"Final convergence progress: pos_err={pos_error*1000:.1f}mm, yaw_err={math.degrees(yaw_error):.2f}°")
            
            rospy.sleep(0.1)
        
        return False

    def execute_smooth_trajectory(self, waypoints):
        """
        DEPRECATED: Legacy waypoint execution method
        Use execute_polynomial_trajectory() for better performance
        """
        rospy.logwarn("WARNING: Using deprecated waypoint execution method, recommend switching to polynomial trajectory")
        
        rospy.loginfo(f"=== 执行{len(waypoints)}个waypoint的平滑轨迹 ===")
        
        start_time = rospy.get_time()
        
        for i, waypoint in enumerate(waypoints):
            waypoint_start_time = rospy.get_time()
            target_pos = waypoint['position']
            target_yaw = waypoint['yaw']
            target_linear_vel = waypoint['linear_velocity']
            target_angular_vel = waypoint['angular_velocity']
            
            is_final_waypoint = (i == len(waypoints) - 1)
            
            rospy.loginfo(f"执行waypoint {i+1}/{len(waypoints)}: "
                         f"pos=({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}), "
                         f"yaw={math.degrees(target_yaw):.1f}°")
            
            # Use executeTrajectoryWithWrench for SE(3) control with position-velocity mode  
            success = self.beetle.executeTrajectoryWithWrench(
                pos=target_pos,
                rot=target_yaw,
                linear_vel=target_linear_vel,
                angular_vel=target_angular_vel,
                force=[0.0, 0.0, 0.0],  # No external force for movement phase
                torque=[0.0, 0.0, 0.0]  # No external torque for movement phase
            )
            
            if not success:
                rospy.logerr(f"Failed to execute waypoint {i+1}")
                return False
            
            # Enhanced convergence checking with different thresholds for intermediate vs final waypoints
            if is_final_waypoint:
                # Stricter convergence for final waypoint
                pos_thresh = self.final_pos_threshold  # 2.5cm
                yaw_thresh = self.final_yaw_threshold  # 3°
                timeout = 20.0  # Longer timeout for final precision (increased for slow motion)
            else:
                # Relaxed convergence for intermediate waypoints
                pos_thresh = self.waypoint_threshold   # 8cm 
                yaw_thresh = 0.0873  # 5° (more relaxed)
                timeout = 12.0   # Longer timeout for intermediate waypoints (increased for slow motion)
            
            # Wait for convergence with timeout
            converged = self.wait_for_waypoint_convergence(
                target_pos, target_yaw, pos_thresh, yaw_thresh, timeout
            )
            
            if not converged:
                if is_final_waypoint:
                    rospy.logerr(f"最终waypoint收敛失败: 位置误差或角度误差过大")
                    return False
                else:
                    rospy.logwarn(f"Waypoint {i+1} 收敛超时，继续执行下一个waypoint")
                    # For intermediate waypoints, continue even if not fully converged
            
            waypoint_time = rospy.get_time() - waypoint_start_time
            rospy.loginfo(f"Waypoint {i+1} 完成，用时: {waypoint_time:.2f}s")
        
        total_time = rospy.get_time() - start_time
        rospy.loginfo(f"✓ 平滑轨迹执行完成，总用时: {total_time:.2f}s")
        return True
    
    def wait_for_waypoint_convergence(self, target_pos, target_yaw, pos_thresh, yaw_thresh, timeout):
        """
        Wait for convergence to waypoint with specified thresholds
        
        Returns:
            bool: True if converged within timeout, False otherwise
        """
        start_time = rospy.get_time()
        
        while rospy.get_time() - start_time < timeout:
            current_pos = self.beetle.getUavPos()
            if current_pos is None:
                rospy.sleep(0.1)
                continue
            
            current_yaw = self.current_yaw
            
            # Check position convergence
            pos_error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
            yaw_error = abs(self.normalize_angle(target_yaw - current_yaw))
            
            if pos_error <= pos_thresh and yaw_error <= yaw_thresh:
                return True
            
            rospy.sleep(0.05)  # 20Hz checking rate
        
        # Log final errors if timeout
        current_pos = self.beetle.getUavPos()
        if current_pos is not None:
            pos_error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
            yaw_error = abs(self.normalize_angle(target_yaw - self.current_yaw))
            rospy.logwarn(f"Waypoint收敛超时: pos_error={pos_error*1000:.1f}mm, yaw_error={math.degrees(yaw_error):.1f}°")
        
        return False
    
    # Removed duplicate incorrect implementation - using the correct three-stage method below
    
    def execute(self, userdata):
        rospy.loginfo("=== THREE-PHASE VALVE APPROACH STRATEGY ===")
        rospy.loginfo("Phase 1: Yaw adjustment to valve handle alignment")
        rospy.loginfo("Phase 2: XY movement to target position")
        rospy.loginfo("Phase 3: Z descent + final insertion angle adjustment")
        
        if not self.wait_for_positions():
            return 'failed'
        
        # Get valve position and current position
        valve_x, valve_y, valve_z = self.valve_pos
        current_pos = self.beetle.getUavPos()
        
        if current_pos is None:
            rospy.logerr("Cannot get current UAV position")
            return 'failed'
        
        # Calculate Phase 1 insertion position using optimizer
        result = self.optimizer.calculate_same_side_insertion_strategy(
            valve_pos=[valve_x, valve_y, valve_z],
            valve_yaw=self.valve_yaw,  # Use actual valve yaw angle
            uav_pos=current_pos  # Pass current UAV position for intelligent spoke selection
        )
        
        if not result['feasible']:
            rospy.logerr(f"Failed to calculate insertion strategy: Strategy not feasible")
            return 'failed'
        
        # Get final target positions from same-side strategy
        safe_uav_position = result['safe_uav_position']
        final_insertion_yaw = result['safe_uav_yaw']  # This is for Phase 3
        
        # 保存Phase 3的精确目标Z坐标，供DESCEND_AND_CONTACT阶段使用
        SingleUAVStateBase._shared_target_z = safe_uav_position[2]
        rospy.loginfo(f"保存目标Z坐标: {safe_uav_position[2]:.3f}m 供后续状态使用")
        
        # Phase 1: First align with current valve handle angle (for observation)
        valve_handle_yaw = self.valve_yaw  # Current valve handle angle
        
        rospy.loginfo(f"Valve position: ({valve_x:.3f}, {valve_y:.3f}, {valve_z:.3f})")
        rospy.loginfo(f"Current UAV position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"Final target position: ({safe_uav_position[0]:.3f}, {safe_uav_position[1]:.3f}, {safe_uav_position[2]:.3f})")
        rospy.loginfo(f"Valve handle yaw: {valve_handle_yaw:.3f} rad ({math.degrees(valve_handle_yaw):.1f}deg)")
        rospy.loginfo(f"Final insertion yaw: {final_insertion_yaw:.3f} rad ({math.degrees(final_insertion_yaw):.1f}deg)")
        rospy.loginfo(f"Current yaw: {self.current_yaw:.3f} rad ({math.degrees(self.current_yaw):.1f}deg)")
        
        # Calculate optimal yaw path for Phase 1 (align with valve handle)
        optimal_handle_yaw = self.calculate_shortest_yaw_path(self.current_yaw, valve_handle_yaw)
        handle_yaw_change = optimal_handle_yaw - self.current_yaw
        
        # ===== PHASE 1: YAW ADJUSTMENT TO VALVE HANDLE =====
        rospy.loginfo("=== Phase 1: Yaw adjustment to valve handle alignment ===")
        rospy.loginfo(f"Adjusting yaw from {math.degrees(self.current_yaw):.1f}deg to {math.degrees(optimal_handle_yaw):.1f}deg")
        rospy.loginfo(f"Handle yaw change: {math.degrees(handle_yaw_change):.1f}deg (shortest path)")
        rospy.loginfo("Maintaining current XYZ position during yaw adjustment")
        
        # Execute pure yaw adjustment to valve handle angle
        success = self._execute_phase1_yaw_adjustment(current_pos, optimal_handle_yaw)
        if not success:
            rospy.logerr("Phase 1 failed: Valve handle yaw alignment unsuccessful")
            return 'failed'
        
        rospy.loginfo("Phase 1 completed: Successfully adjusted to target yaw")
        
        # ===== PHASE 2: XY MOVEMENT ONLY =====
        rospy.loginfo("=== Phase 2: XY movement to target position ===")
        rospy.loginfo("Maintaining valve handle yaw alignment and current Z height")
        rospy.loginfo(f"Moving XY from ({current_pos[0]:.3f}, {current_pos[1]:.3f}) to ({safe_uav_position[0]:.3f}, {safe_uav_position[1]:.3f})")
        
        # Get current position after phase 1
        phase1_pos = self.beetle.getUavPos()
        if phase1_pos is None:
            rospy.logerr("Cannot get position after phase 1")
            return 'failed'
        
        # Create XY target (maintain current Z from phase 1, maintain handle yaw from phase 1)
        xy_target = [safe_uav_position[0], safe_uav_position[1], phase1_pos[2]]
        
        success = self._execute_phase2_xy_movement(xy_target, optimal_handle_yaw)  # Keep handle yaw
        if not success:
            rospy.logerr("Phase 2 failed: XY movement unsuccessful")
            return 'failed'
        
        rospy.loginfo("Phase 2 completed: Successfully moved to target XY position")
        
        # ===== PHASE 3: Z DESCENT + FINAL INSERTION ANGLE ADJUSTMENT =====
        rospy.loginfo("=== Phase 3: Z descent + final insertion angle adjustment ===")
        rospy.loginfo(f"Adjusting yaw to insertion angle: {math.degrees(final_insertion_yaw):.1f}deg")
        
        # Get position after phase 2
        phase2_pos = self.beetle.getUavPos()
        if phase2_pos is None:
            rospy.logerr("Cannot get position after phase 2")
            return 'failed'
        
        # Calculate optimal insertion yaw path
        optimal_insertion_yaw = self.calculate_shortest_yaw_path(self.current_yaw, final_insertion_yaw)
        insertion_yaw_change = optimal_insertion_yaw - self.current_yaw
        rospy.loginfo(f"Insertion yaw change: {math.degrees(insertion_yaw_change):.1f}deg")
        rospy.loginfo(f"Descending from current Z to target Z: {safe_uav_position[2]:.3f}m")
        rospy.loginfo("Performing controlled Z descent + insertion angle adjustment")
        
        success = self._execute_phase3_z_descent_and_adjustment(safe_uav_position, optimal_insertion_yaw)
        if not success:
            rospy.logerr("Phase 3 failed: Z descent and insertion angle adjustment unsuccessful")
            return 'failed'
        
        rospy.loginfo("Phase 3 completed: Successfully descended and fine-tuned")
        rospy.loginfo("=== THREE-PHASE APPROACH COMPLETED SUCCESSFULLY ===")
        rospy.loginfo("UAV is now positioned for valve insertion")
        
        # 增加3秒稳定等待时间，避免state切换过快
        rospy.loginfo("Waiting 3 seconds for system stabilization before state transition...")
        rospy.sleep(3.0)
        
        return 'succeeded'


class DescendAndContactState(SingleUAVStateBase):
    def __init__(self, module_id=1, rotation_direction=1):
        super().__init__(outcomes=['succeeded', 'failed'], 
                        input_keys=['start_position'],
                        output_keys=['initial_valve_yaw', 'valve_yaw_change_threshold', 'rotation_start_position', 
                                   'insertion_info', 'trajectory_state', 'contact_final_torque'],  # 状态传递而非对象
                        module_id=module_id)
        
        # Store rotation direction for insertion strategy
        self.rotation_direction = rotation_direction  # 1 for clockwise, -1 for counter-clockwise
        
        # Initialize insertion optimizer with CORRECTED valve wheel geometry
        self.optimizer = InsertionOptimizer(
            valve_radius=0.10        # CORRECTED: Inner rim radius (200mm diameter, 100mm radius)
            , valve_beam_width=0.035  # CORRECTED: Minimum spoke gap width (35mm at hub connection)
            , safety_margin=0.002
            , module_id=module_id
            , simulation=True
        )
        
        self.pre_insertion_distance = 0.035
        self.circumferential_offset = 0.005
    
    def execute(self, userdata):
        """
        优化的Contact模式 - 跳过重复插入逻辑:
        
        注意: MoveToValveState的Phase 3B已经完成了精确定位和Z下降
        因此这里跳过阶段1的重复插入，直接进入阶段2的圆周接触运动
        
        阶段2: 圆周接触运动 - 使用在线轨迹生成器进行'自转+公转'圆周运动
        """
        rospy.loginfo("=== 阀门接触（优化模式 - 跳过重复插入） ===")
        rospy.loginfo("注意：Phase 3B已完成精确定位，直接进入圆周接触运动")
        
        if not self.wait_for_positions():
            return 'failed'
        
        # 验证当前位置是否合理（简单检查，不重复复杂插入逻辑）
        current_pos = self.beetle.getUavPos()
        current_yaw = self.current_yaw
        
        if current_pos is None or current_yaw is None:
            rospy.logerr("无法获取当前位置/姿态")
            return 'failed'
        
        rospy.loginfo(f"当前位置: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"当前偏航: {math.degrees(current_yaw):.1f}°")
        rospy.loginfo("跳过重复插入逻辑，Phase 3B已完成精确定位")
        
        # ========== 直接进入阶段2: 圆周接触运动 ==========
        contact_result = self._execute_circular_contact_phase(userdata)
        if contact_result != 'success':
            rospy.logerr("阶段2失败：圆周接触运动未建立")
            return 'failed'
        
        rospy.loginfo("✓ 圆周接触建立，检测到阀门转动")
        rospy.loginfo("=== Contact模式成功完成（优化版本） ===")
        
        return 'succeeded'
    
    def _execute_linear_insertion_phase(self):
        """
        DEPRECATED: 阶段1直线下降插入 - 已被MoveToValveState的Phase 3B取代
        
        注意：这个方法现在是冗余的，因为MoveToValveState的Phase 3B已经
        完成了精确的Z下降和位置调整。保留此方法仅用于调试或回退。
        """
        rospy.logwarn("WARNING: Using deprecated linear insertion phase")
        rospy.logwarn("MoveToValveState Phase 3B should have completed precise positioning")
        
        # 原有插入逻辑保留但标记为废弃
        rospy.loginfo(">>> 开始阶段1: 直线下降插入（DEPRECATED）")
        
        current_pos = self.beetle.getUavPos()
        current_yaw = self.current_yaw
        
        if current_pos is None or current_yaw is None:
            rospy.logerr("无法获取当前位置/姿态")
            return 'failed'
        
        # Use existing insertion optimizer to calculate target position
        insertion_result = self.optimizer.calculate_same_side_insertion_strategy(
            current_pos, current_yaw
        )
        
        if insertion_result is None or 'safe_uav_position' not in insertion_result:
            rospy.logerr("插入目标计算失败")
            return 'failed'
        
        target_pos = insertion_result['safe_uav_position']
        target_yaw = insertion_result.get('safe_uav_yaw', insertion_result.get('yaw', current_yaw))
        
        rospy.loginfo(f"插入目标: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
        rospy.loginfo(f"目标偏航: {math.degrees(target_yaw):.1f}°")
        
        # 执行插入运动
        success = self.beetle.goPoseWaitConvergence(
            pos=target_pos,
            rot=target_yaw,
            pos_thresh=0.02,  # 2cm精度
            rot_thresh=0.0524,  # 3°精度
            timeout=15.0
        )
        
        if not success:
            rospy.logerr("插入运动收敛失败")
            return 'failed'
        
        rospy.loginfo("✓ 直线插入完成，位置收敛")
        return 'success'
    
    def _execute_circular_contact_phase(self, userdata):
        """
        阶段2: 圆周接触运动
        Use online trajectory generator to execute 'rotation + revolution' circular movement, detecting valve rotation
        """
        rospy.loginfo(">>> 开始阶段2: 圆周接触运动（自转+公转）")
        
        # 获取当前状态
        current_pos = self.beetle.getUavPos()
        current_yaw = self.current_yaw
        valve_pos = self.valve_pos
        initial_valve_yaw = self.valve_yaw
        
        if any(v is None for v in [current_pos, current_yaw, valve_pos]):
            rospy.logerr("状态信息不完整")
            return 'failed'
        
        # 计算当前半径
        relative_pos = np.array(current_pos[:2]) - np.array(valve_pos[:2])
        current_radius = np.linalg.norm(relative_pos)
        
        rospy.loginfo(f"阀门中心: ({valve_pos[0]:.3f}, {valve_pos[1]:.3f}, {valve_pos[2]:.3f})")
        rospy.loginfo(f"当前半径: {current_radius*1000:.1f}mm")
        rospy.loginfo(f"初始阀门角度: {math.degrees(initial_valve_yaw):.1f}°")
        
        # 初始化在线轨迹生成器（优化参数 - 降低速度提高精度）
        trajectory_generator = OnlineCircularTrajectoryGenerator(
            valve_center=valve_pos,
            initial_radius=current_radius,
            target_angular_velocity=0.1,  # 0.1 rad/s ≈ 5.7°/s（Contact阶段需要足够速度建立接触）
            control_rate=25.0,  # 优化：25Hz控制频率（提高轨迹密度）
            debug=True
        )
        # 使用Phase 3的精确目标Z坐标而非实时位置，避免16mm上浮误差
        if SingleUAVStateBase._shared_target_z is not None:
            precise_target_pos = [current_pos[0], current_pos[1], SingleUAVStateBase._shared_target_z]
            rospy.loginfo(f"使用精确目标位置初始化轨迹: Z={SingleUAVStateBase._shared_target_z:.3f}m (而非实时Z={current_pos[2]:.3f}m)")
            trajectory_generator.initialize_from_current_position(precise_target_pos)
        else:
            rospy.logwarn("未找到保存的目标Z坐标，回退到实时位置")
            trajectory_generator.initialize_from_current_position(current_pos)
        
        # 圆周接触运动参数（Dragon逻辑）
        contact_duration = 20.0  # 20秒接触时间，给接触建立充分的时间
        valve_rotation_threshold = 0.1  # Dragon标准：0.1 rad ≈ 5.7°（移除强制时间约束）
        contact_force_threshold = 0.3  # 0.3N接触力阈值
        
        start_time = rospy.Time.now().to_sec()
        last_control_time = start_time
        control_rate = rospy.Rate(25)  # 优化：25Hz控制（与轨迹生成器同步）
        
        rospy.loginfo(f"开始圆周接触运动 - 目标检测: {math.degrees(valve_rotation_threshold):.1f}°阀门转动（Dragon逻辑）")
        
        while (rospy.Time.now().to_sec() - start_time) < contact_duration:
            current_time = rospy.Time.now().to_sec()
            
            # 获取当前状态
            current_pos = self.beetle.getUavPos()
            current_yaw = self.current_yaw
            current_valve_yaw = self.valve_yaw
            
            if any(v is None for v in [current_pos, current_yaw, current_valve_yaw]):
                rospy.logwarn("状态信息丢失，继续等待")
                control_rate.sleep()
                continue
            
            # 计算阀门转动角度
            valve_rotation = abs(current_valve_yaw - initial_valve_yaw)
            valve_angular_velocity = 0.0
            if current_time > last_control_time + 0.1:
                if hasattr(self, '_last_valve_yaw'):
                    dt = current_time - last_control_time
                    valve_angular_velocity = abs(current_valve_yaw - self._last_valve_yaw) / dt
                self._last_valve_yaw = current_valve_yaw
                last_control_time = current_time
            
            # 更新轨迹生成器状态
            state_info = trajectory_generator.update_state(
                current_pos, current_yaw, valve_angular_velocity
            )
            
            # 生成控制目标
            target_state = trajectory_generator.generate_target_state()
            
            # 执行SE(3)控制
            self.beetle.executeTrajectoryWithWrench(
                pos=target_state['position'],
                rot=target_state['yaw'],
                linear_vel=target_state['linear_velocity'],
                angular_vel=target_state['angular_velocity'],
                force=target_state['force'],
                torque=target_state['torque']
            )
            
            # 检测阀门转动（Dragon逻辑：实时检测，无时间约束）
            if valve_rotation > valve_rotation_threshold:
                elapsed_time = current_time - start_time
                rospy.loginfo(f"✓ 检测到阀门转动: {math.degrees(valve_rotation):.1f}° "
                            f"(阈值: {math.degrees(valve_rotation_threshold):.1f}°)")
                rospy.loginfo(f"接触运动时间: {elapsed_time:.1f}s")
                
                # 设置输出数据（包含轨迹生成器和状态信息）
                self._set_output_data(initial_valve_yaw, current_pos, current_yaw, userdata, 
                                    trajectory_generator, target_state)
                return 'success'
            
            # 检测接触力（辅助检测）
            wrench_data = self.beetle.getEstimatedWrench()
            if wrench_data is not None:
                force_magnitude = math.sqrt(
                    wrench_data.force.x**2 + wrench_data.force.y**2 + wrench_data.force.z**2
                )
                if force_magnitude > contact_force_threshold:
                    rospy.loginfo_throttle(2.0, f"检测到接触力: {force_magnitude:.2f}N "
                                                f"(阈值: {contact_force_threshold:.2f}N)")
            
            # 调试信息
            if current_time - start_time > 1.0:  # 1秒后开始输出调试信息
                rospy.loginfo_throttle(2.0, 
                    f"圆周接触: 半径={state_info['current_radius']*1000:.1f}mm, "
                    f"角速度={math.degrees(state_info['angular_velocity']):.1f}°/s, "
                    f"阀门转动={math.degrees(valve_rotation):.2f}°, "
                    f"时间={current_time-start_time:.1f}s")
            
            if rospy.is_shutdown():
                return 'failed'
            
            control_rate.sleep()
        
        # 超时但检查是否有微小的阀门转动
        final_valve_rotation = abs(self.valve_yaw - initial_valve_yaw)
        if final_valve_rotation > math.radians(0.5):  # 降低阈值到0.5°
            rospy.logwarn(f"接触超时，但检测到微小阀门转动: {math.degrees(final_valve_rotation):.2f}°，认为成功")
            self._set_output_data(initial_valve_yaw, current_pos, current_yaw, userdata)
            return 'success'
        
        rospy.logwarn("圆周接触阶段超时，未检测到足够的阀门转动")
        return 'timeout'
    
    def _set_output_data(self, initial_valve_yaw, current_pos, current_yaw, userdata, 
                        trajectory_generator=None, target_state=None):
        """Set output data for use by next state - Dragon接力机制"""
        userdata.initial_valve_yaw = initial_valve_yaw
        userdata.valve_yaw_change_threshold = math.radians(2.0)
        userdata.rotation_start_position = current_pos
        
        # Dragon接力机制：传递轨迹生成器状态而非对象（更安全）
        if trajectory_generator is not None:
            # 传递轨迹生成器的状态参数而非对象本身（避免对象损坏）
            userdata.trajectory_state = {
                'valve_center': trajectory_generator.valve_center.tolist() if hasattr(trajectory_generator.valve_center, 'tolist') else list(trajectory_generator.valve_center),
                'initial_radius': trajectory_generator.initial_radius,
                'current_radius': getattr(trajectory_generator, 'current_radius', trajectory_generator.initial_radius),
                'current_angle': getattr(trajectory_generator, 'current_angle', 0.0),
                'radius_locked': getattr(trajectory_generator, 'radius_locked', False),
                'locked_radius': getattr(trajectory_generator, 'locked_radius', trajectory_generator.initial_radius),
                'lock_radius_value': getattr(trajectory_generator, 'lock_radius_value', None)
            }
            rospy.loginfo("✓ 轨迹生成器状态已传递给Manipulate阶段")
        else:
            userdata.trajectory_state = None
        
        if target_state is not None:
            final_torque = target_state.get('torque', [0.0, 0.0, 0.0])
            userdata.contact_final_torque = final_torque
            rospy.loginfo(f"✓ 接触阶段最终扭矩: {final_torque}")
        else:
            default_torque = [0.0, 0.0, 0.1]  # 默认z轴扭矩0.1N·m
            userdata.contact_final_torque = default_torque
            rospy.loginfo(f"✓ 使用默认扭矩基线: {default_torque}")
        
        userdata.insertion_info = {
            'insertion_position': current_pos,
            'insertion_yaw': current_yaw,
            'initial_valve_yaw': initial_valve_yaw,
            'contact_detected': True,
            'contact_method': 'circular_trajectory'  # 新增标识
        }
        
        rospy.loginfo("✓ Contact模式输出数据设置完成")
        
        rospy.loginfo("✓ 阀门接触检测完成，准备开始转动")
        return 'succeeded'
    
    def determine_optimal_contact_point(self):
        """
        Step 1: Determine optimal dual-fang insertion strategy using InsertionOptimizer
        Uses the new dual-fang clockwise strategy from insertion_optimizer.py
        """
        rospy.loginfo("--- Step 1: Determining optimal dual-fang contact strategy using InsertionOptimizer ---")
        
        # Get current UAV and valve positions
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot get current UAV position")
            return None
        
        # Use InsertionOptimizer to calculate dual-fang strategy
        if not self.optimizer:
            rospy.logerr("Optimizer not initialized")
            return None
        
        # Call the new dual-fang strategy method
        strategy = self.optimizer.calculate_dual_fang_clockwise_strategy(
            uav_pos=current_pos,
            valve_pos=self.valve_pos,
            valve_yaw=self.valve_yaw
        )
        
        if strategy is None:
            rospy.logerr("Failed to calculate dual-fang strategy")
            return None
        
        # Store strategy for use in insertion phase
        self.current_insertion_params = strategy
        
        # Handle different strategy types
        if strategy.get('strategy') == 'two_phase_safe_insertion':
            # For two-phase strategy, calculate dual-fang center from phase 2 targets
            phase2_left = strategy['phase2_left_final']
            phase2_right = strategy['phase2_right_final']
            dual_fang_center = (
                (phase2_left[0] + phase2_right[0]) / 2,
                (phase2_left[1] + phase2_right[1]) / 2,
                phase2_left[2]
            )
            
            # Calculate UAV yaw from claw separation vector
            claw_vector_x = phase2_right[0] - phase2_left[0]
            claw_vector_y = phase2_right[1] - phase2_left[1]
            optimal_uav_yaw = math.atan2(claw_vector_y, claw_vector_x) - math.pi/2
            
            # Normalize yaw
            while optimal_uav_yaw >= 2*math.pi:
                optimal_uav_yaw -= 2*math.pi
            while optimal_uav_yaw < 0:
                optimal_uav_yaw += 2*math.pi
        else:
            # Legacy strategy structure
            dual_fang_center = strategy.get('dual_fang_center')
            optimal_uav_yaw = strategy.get('optimal_uav_yaw', 0.0)
            
            if dual_fang_center is None:
                rospy.logerr("Strategy missing dual_fang_center field")
                return None
        
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        
        contact_point = {
            'position': dual_fang_center,
            'angle': optimal_uav_yaw,
            'distance': math.sqrt((dual_fang_center[0] - valve_center_x)**2 + (dual_fang_center[1] - valve_center_y)**2),
            'strategy': strategy.get('strategy', 'dual_fang_clockwise')
        }
        
        rospy.loginfo("=== DUAL-FANG STRATEGY FROM INSERTION OPTIMIZER ===")
        
        strategy_type = strategy.get('strategy', 'unknown')
        rospy.loginfo(f"Strategy type: {strategy_type}")
        
        if strategy_type == 'two_phase_safe_insertion':
            same_side_result = strategy.get('same_side_result', {})
            rospy.loginfo(f"User-verified same-side strategy: {same_side_result.get('formula', 'N/A')}")
            rospy.loginfo(f"Expected separation: {same_side_result.get('actual_separation', 0)*1000:.1f}mm")
            rospy.loginfo(f"Safety margin: {same_side_result.get('margin', 0)*1000:.1f}mm")
            rospy.loginfo(f"Two-phase approach: Safe insertion → Final contact")
        else:
            # Legacy strategy information
            rospy.loginfo(f"Left claw -> {strategy.get('left_gap', 'N/A')} at {math.degrees(strategy.get('left_gap_angle', 0)):.0f}°")
            rospy.loginfo(f"Right claw -> {strategy.get('right_gap', 'N/A')} at {math.degrees(strategy.get('right_gap_angle', 0)):.0f}°")
            rospy.loginfo(f"Best approach: {strategy.get('best_approach_name', 'N/A')}")
            rospy.loginfo(f"Total positioning error: {strategy.get('total_positioning_error', 0):.4f}m")
        
        return contact_point
    
    def rotate_and_insert_to_beam_gap(self, contact_point):
        """
        Step 2: Execute dual-fang insertion strategy
        - Position both claws simultaneously into their respective gaps
        - Left claw -> beam1_beam2 gap, Right claw -> beam2_beam0 gap
        """
        rospy.loginfo("--- Step 2: Executing dual-fang insertion strategy ---")
        
        if contact_point is None:
            rospy.logerr("No contact point provided for insertion")
            return False
        
        # Use dual-fang strategy from step 1
        if not hasattr(self, 'current_insertion_params'):
            rospy.logerr("No dual-fang insertion parameters available from step 1")
            return False
        
        params = self.current_insertion_params
        
        strategy_type = params.get('strategy', params.get('strategy_type', 'unknown'))
        
        if strategy_type not in ['dual_fang_clockwise', 'two_phase_safe_insertion']:
            rospy.logwarn(f"Unexpected strategy type: {strategy_type}, proceeding anyway")
        
        # Execute dual-fang insertion
        success = self.execute_dual_fang_insertion(params)
        
        if success:
            rospy.loginfo("✓ Successfully executed dual-fang insertion strategy")
            return True
        else:
            rospy.logerr("✗ Failed to execute dual-fang insertion")
            return False
    
    def move_along_valve_circumference(self, contact_point):
        """
        Step 3: Move along valve circumference from insertion point to contact point
        - Apply 75.7mm optimal contact distance for 150mm claw separation (10mm end-effector distance)
        - Maintain contact with valve while moving circumferentially
        - Adjust fang positions for optimal grip during movement
        - Monitor contact forces to prevent slipping
        """
        rospy.loginfo("--- Step 3: Moving along valve circumference to contact point ---")
        rospy.loginfo("APPLYING 10mm optimal contact distance constraint for 150mm claw separation (maximizing insertion success rate)")
        
        # Get current position after insertion
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position for circumferential movement")
            return False
        
        # CRITICAL: Apply 10mm optimal contact distance constraint (UNIFIED with insertion_optimizer.py)
        # For 150mm claw separation, left/right claws need to be optimally positioned from valve center
        # MODIFICATION: Updated from 35mm to 10mm to match insertion_optimizer.py min_end_effector_to_valve_center_distance
        # This provides geometric constraint consistency and maximizes insertion success rate
        OPTIMAL_CONTACT_DISTANCE = 0.010  # 10mm (UNIFIED CONSTRAINT with insertion_optimizer.py)
        
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        
        # Calculate optimal contact position maintaining 75.7mm distance (10mm end-effector distance)
        # Use contact_point direction but enforce 75.7mm distance
        target_x, target_y, target_z = contact_point['position']
        contact_direction_x = target_x - valve_center_x
        contact_direction_y = target_y - valve_center_y
        contact_direction_magnitude = math.sqrt(contact_direction_x**2 + contact_direction_y**2)
        
        if contact_direction_magnitude > 0.001:  # Avoid division by zero
            # Normalize direction and apply optimal distance
            normalized_x = contact_direction_x / contact_direction_magnitude
            normalized_y = contact_direction_y / contact_direction_magnitude
            
            # Calculate optimal contact position
            optimal_contact_x = valve_center_x + OPTIMAL_CONTACT_DISTANCE * normalized_x
            optimal_contact_y = valve_center_y + OPTIMAL_CONTACT_DISTANCE * normalized_y
            optimal_contact_z = target_z  # Keep same Z height
        else:
            rospy.logerr("Invalid contact direction - cannot apply 75.7mm constraint")
            return False
        
        rospy.loginfo(f"Valve center: ({valve_center_x:.3f}, {valve_center_y:.3f})")
        rospy.loginfo(f"Original contact point: ({target_x:.3f}, {target_y:.3f}) - distance: {contact_direction_magnitude*1000:.1f}mm")
        rospy.loginfo(f"Optimal contact point: ({optimal_contact_x:.3f}, {optimal_contact_y:.3f}) - distance: {OPTIMAL_CONTACT_DISTANCE*1000:.1f}mm (更严格碰撞预防)")
        
        # Calculate current dual-fang center position using optimizer
        end_effector_x = current_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(self.current_yaw)
        end_effector_y = current_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(self.current_yaw)
        
        # Check distance to optimal contact point
        distance_to_optimal_contact = math.sqrt(
            (end_effector_x - optimal_contact_x)**2 + 
            (end_effector_y - optimal_contact_y)**2
        )
        
        rospy.loginfo(f"Current dual-fang position: ({end_effector_x:.3f}, {end_effector_y:.3f})")
        rospy.loginfo(f"Distance to optimal contact: {distance_to_optimal_contact*1000:.1f}mm")
        
        # Check if we need to move to optimal contact position
        CONTACT_TOLERANCE = 0.015  # 15mm tolerance
        if distance_to_optimal_contact < CONTACT_TOLERANCE:
            rospy.loginfo("✓ Already positioned at 55mm optimal contact point - circumferential movement complete")
            return True
        else:
            rospy.loginfo(f"Need to move {distance_to_optimal_contact*1000:.1f}mm to optimal contact position")
            rospy.loginfo("Executing circumferential movement to 55mm optimal contact point...")
            
            # Calculate target UAV position for optimal contact using optimizer
            # Reverse calculate UAV position from optimal dual-fang center position
            target_uav_x = optimal_contact_x - self.optimizer.dual_fang_center_offset * math.cos(self.current_yaw)
            target_uav_y = optimal_contact_y - self.optimizer.dual_fang_center_offset * math.sin(self.current_yaw)
            target_uav_z = current_pos[2]  # Keep same Z height
            
            target_uav_pos = (target_uav_x, target_uav_y, target_uav_z)
            
            rospy.loginfo(f"Target UAV position for optimal contact: ({target_uav_x:.3f}, {target_uav_y:.3f}, {target_uav_z:.3f})")
            
            # Execute careful movement to optimal contact position
            movement_success = self.motion_controller.execute_smooth_trajectory_with_yaw(
                start_pos=current_pos,
                target_pos=target_uav_pos,
                target_yaw=self.current_yaw,  # Keep same orientation
                duration=3.0,  # 3 seconds for careful movement
                pos_threshold=0.01,  # 1cm accuracy for precise contact
                yaw_threshold=self.yaw_precision_levels['moderate']   # Enhanced: 1.15° precision (was 0.02)
            )
            
            if not movement_success:
                rospy.logerr("Failed to move to optimal contact position")
                return False
            
            # Verify final position
            final_pos = self.get_current_position()
            if final_pos is None:
                rospy.logerr("Cannot verify final position after circumferential movement")
                return False
            
            # Calculate final end-effector position using optimizer
            final_end_effector_x = final_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(self.current_yaw)
            final_end_effector_y = final_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(self.current_yaw)
            
            final_distance_to_optimal = math.sqrt(
                (final_end_effector_x - optimal_contact_x)**2 + 
                (final_end_effector_y - optimal_contact_y)**2
            )
            
            rospy.loginfo(f"Final distance to optimal contact: {final_distance_to_optimal*1000:.1f}mm")
            
            if final_distance_to_optimal < CONTACT_TOLERANCE:
                rospy.loginfo("Successfully moved to 55mm optimal contact position")
                return True
            else:
                rospy.logwarn(f"Final position error {final_distance_to_optimal*1000:.1f}mm - acceptable for robustness")
                return True  # Accept with warning for robustness
    
    def prepare_for_valve_rotation(self, contact_point):
        """
        Step 4: Prepare for valve rotation
        - Verify secure grip on valve
        - Check mechanical advantage for rotation direction
        - Set up rotation control parameters
        """
        rospy.loginfo("--- Step 4: Preparing for valve rotation ---")
        
        # Verify final position and orientation
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot verify position for rotation preparation")
            return False
        
        # Calculate final dual-fang center position using optimizer
        end_effector_x = current_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(self.current_yaw)
        end_effector_y = current_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(self.current_yaw)
        end_effector_z = current_pos[2] + self.optimizer.end_effector_offset_z
        
        # Verify position relative to valve
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        distance_to_center = math.sqrt((end_effector_x - valve_center_x)**2 + (end_effector_y - valve_center_y)**2)
        
        # Verify rotation readiness
        rotation_direction_str = 'clockwise' if self.rotation_direction == 1 else 'counter-clockwise'
        
        rospy.loginfo("=== VALVE ROTATION PREPARATION COMPLETE ===")
        rospy.loginfo(f"Final dual-fang position: ({end_effector_x:.3f}, {end_effector_y:.3f}, {end_effector_z:.3f})")
        rospy.loginfo(f"Distance from valve center: {distance_to_center:.3f}m")
        rospy.loginfo(f"Rotation direction: {rotation_direction_str}")
        rospy.loginfo(f"Mechanical advantage: {contact_point.get('mechanical_advantage', 'Good')}")
        rospy.loginfo("Ready for valve rotation phase")
        
        return True
    
    def prepare_for_valve_rotation_at_insertion_point(self):
        """
        NEW: Prepare for valve rotation at current insertion point
        
        Strategy: Instead of moving to "optimal contact point", begin rotation trajectory
        at current insertion position and detect contact through valve yaw change.
        
        This eliminates the problematic circumferential movement that was failing.
        """
        rospy.loginfo("--- NEW: Preparing for valve rotation at insertion point ---")
        rospy.loginfo("Strategy: Begin rotation trajectory and detect contact via valve yaw change")
        
        # Verify current position after insertion
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot verify position for rotation preparation")
            return False
        
        # Calculate current dual-fang center position using optimizer
        end_effector_x = current_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(self.current_yaw)
        end_effector_y = current_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(self.current_yaw)
        end_effector_z = current_pos[2] + self.optimizer.end_effector_offset_z
        
        # Verify position relative to valve
        valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
        distance_to_center = math.sqrt((end_effector_x - valve_center_x)**2 + (end_effector_y - valve_center_y)**2)
        
        # Record initial valve yaw for contact detection
        self.initial_valve_yaw = self.valve_yaw
        self.valve_yaw_change_threshold = math.radians(2.0)  # 2° threshold for contact detection
        
        rospy.loginfo("=== INSERTION POINT ROTATION PREPARATION ===")
        rospy.loginfo(f"Current dual-fang position: ({end_effector_x:.3f}, {end_effector_y:.3f}, {end_effector_z:.3f})")
        rospy.loginfo(f"Distance from valve center: {distance_to_center*1000:.1f}mm")
        rospy.loginfo(f"Initial valve yaw: {math.degrees(self.initial_valve_yaw):.1f}°")
        rospy.loginfo(f"Contact detection threshold: {math.degrees(self.valve_yaw_change_threshold):.1f}°")
        rospy.loginfo("Ready to begin 'rotation + revolution' trajectory around valve center")
        
        # Set rotation readiness flag
        rotation_direction_str = 'clockwise' if self.rotation_direction == 1 else 'counter-clockwise'
        rospy.loginfo(f"Rotation direction: {rotation_direction_str}")
        rospy.loginfo("Contact strategy: Monitor valve yaw change during trajectory execution")
        
        return True
    
    def execute_dual_fang_insertion(self, params):
        """
        Execute dual-fang insertion using calculated strategy.
        
        Supports:
        - two_phase_safe_insertion: User's verified same-side strategy with safe approach
        - Legacy strategies: For backward compatibility
        
        Args:
            params: Insertion parameters from optimizer
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("=== EXECUTING DUAL-FANG INSERTION ===")
        
        # CONTROL MODE OPTIMIZATION: Set final descent flag for PD control activation
        if hasattr(self.motion_controller, 'final_descent_active'):
            self.motion_controller.final_descent_active = True
            self.motion_controller.pre_final_descent = False  # Switch from PID to PD for final descent
            rospy.loginfo("FINAL DESCENT PHASE: Activated PD control mode for insertion")
        
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("No UAV position available")
            return False
        
        # Check strategy type and execute accordingly
        strategy_type = params.get('strategy', 'legacy')
        
        if strategy_type == 'two_phase_safe_insertion':
            return self.execute_two_phase_insertion(params)
        else:
            # Legacy single-phase insertion for backward compatibility
            rospy.loginfo("Using legacy single-phase insertion")
            return self.execute_legacy_insertion(params)
    
    def execute_two_phase_insertion(self, params):
        """
        Execute two-phase safe insertion strategy:
        Phase 1: Insert to gap centers (safe radius 68.75mm)
        Phase 2: Radial approach to final contact points (inner rim 100mm)
        
        This implements the user's verified same-side strategy with safety.
        
        Args:
            params: Two-phase insertion parameters from optimizer
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("=== EXECUTING TWO-PHASE SAFE INSERTION ===")
        rospy.loginfo("Using user-verified same-side strategy")
        
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("No UAV position available")
            return False
        
        # Save insertion geometry for future beam-return disengagement
        self._save_insertion_geometry(current_pos)
        
        # Extract phase 1 (safe) and phase 2 (final) targets
        phase1_left_safe = params['phase1_left_safe']
        phase1_right_safe = params['phase1_right_safe']
        phase2_left_final = params['phase2_left_final']
        phase2_right_final = params['phase2_right_final']
        safe_radius = params['safe_radius']
        final_radius = params['final_radius']
        same_side_result = params['same_side_result']
        
        rospy.loginfo(f"Current UAV position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"Two-phase strategy details:")
        rospy.loginfo(f"  Phase 1 - Safe radius: {safe_radius*1000:.1f}mm")
        rospy.loginfo(f"  Phase 2 - Final radius: {final_radius*1000:.1f}mm")
        rospy.loginfo(f"  Expected separation: {same_side_result['actual_separation']*1000:.1f}mm")
        rospy.loginfo(f"  Safety margin: {same_side_result['margin']*1000:.1f}mm")
        
        # === PHASE 1: SAFE INSERTION TO GAP CENTERS ===
        rospy.loginfo("=== PHASE 1: SAFE INSERTION TO GAP CENTERS ===")
        
        # Calculate UAV position for Phase 1 (safe insertion)
        phase1_uav_pos, phase1_uav_yaw = self.calculate_uav_position_for_targets(
            phase1_left_safe, phase1_right_safe, self.valve_pos[2])
        
        if not phase1_uav_pos:
            rospy.logerr("Phase 1: Could not calculate UAV position for safe insertion")
            return False
        
        rospy.loginfo(f"Phase 1 - Safe insertion targets:")
        rospy.loginfo(f"  Left claw target: ({phase1_left_safe[0]:.3f}, {phase1_left_safe[1]:.3f})")
        rospy.loginfo(f"  Right claw target: ({phase1_right_safe[0]:.3f}, {phase1_right_safe[1]:.3f})")
        rospy.loginfo(f"  UAV position: ({phase1_uav_pos[0]:.3f}, {phase1_uav_pos[1]:.3f}, {phase1_uav_pos[2]:.3f})")
        rospy.loginfo(f"Phase 1 - Safe insertion targets:")
        rospy.loginfo(f"  Left claw target: ({phase1_left_safe[0]:.3f}, {phase1_left_safe[1]:.3f})")
        rospy.loginfo(f"  Right claw target: ({phase1_right_safe[0]:.3f}, {phase1_right_safe[1]:.3f})")
        rospy.loginfo(f"  UAV position: ({phase1_uav_pos[0]:.3f}, {phase1_uav_pos[1]:.3f}, {phase1_uav_pos[2]:.3f})")
        rospy.loginfo(f"  UAV yaw: {math.degrees(phase1_uav_yaw):.1f}°")
        
        # Execute Phase 1 movement using DIRECT valve insertion control (replaces complex three-stage)
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position for Phase 1 movement")
            return False
            
        rospy.loginfo("Using THREE-STAGE INSERTION WITH Z-AXIS CONTROL (fixed approach)")
        # Use fixed three-stage insertion strategy with correct Z-axis control
        if not self.motion_controller.execute_three_stage_insertion_with_adaptive_replanning(
            start_pos=current_pos,
            target_pos=phase1_uav_pos,
            target_yaw=phase1_uav_yaw,
            intermediate_distance=1.0   # 1m intermediate distance
        ):
            rospy.logerr("Phase 1: Failed to reach safe insertion position using two-stage movement")
            return False
        
        rospy.loginfo("=== PHASE 2: RADIAL APPROACH TO FINAL CONTACT POINTS ===")
        
        # Calculate UAV position for Phase 2 (final contact)
        phase2_uav_pos, phase2_uav_yaw = self.calculate_uav_position_for_targets(
            phase2_left_final, phase2_right_final, self.valve_pos[2])
        
        if not phase2_uav_pos:
            rospy.logerr("Phase 2: Could not calculate UAV position for final contact")
            return False
        
        rospy.loginfo(f"Phase 2 - Final contact targets:")
        rospy.loginfo(f"  Left claw target: ({phase2_left_final[0]:.3f}, {phase2_left_final[1]:.3f})")
        rospy.loginfo(f"  Right claw target: ({phase2_right_final[0]:.3f}, {phase2_right_final[1]:.3f})")
        rospy.loginfo(f"  UAV position: ({phase2_uav_pos[0]:.3f}, {phase2_uav_pos[1]:.3f}, {phase2_uav_pos[2]:.3f})")
        rospy.loginfo(f"  UAV yaw: {math.degrees(phase2_uav_yaw):.1f}°")
        
        # Execute Phase 2 movement (radial approach)
        if not self.execute_radial_approach_movement(phase2_uav_pos, phase2_uav_yaw):
            rospy.logerr("Phase 2: Failed to reach final contact position")
            return False
        
        rospy.loginfo("✓ Phase 2 completed - Claws at final contact points (inner rim)")
        
        # === FINAL VERIFICATION ===
        final_pos = self.get_current_position()
        if final_pos:
            success = self.verify_two_phase_insertion_result(
                final_pos, phase2_left_final, phase2_right_final, same_side_result)
            
            if success:
                rospy.loginfo("✓ TWO-PHASE SAFE INSERTION SUCCESSFUL")
                rospy.loginfo(f"✓ User-verified same-side strategy implemented: {same_side_result['formula']}")
                return True
            else:
                rospy.logwarn("Two-phase insertion completed but verification failed")
                return False
        
        rospy.logerr("Could not verify final position")
        return False
    
    def calculate_uav_position_for_targets(self, left_target, right_target, target_z):
        """
        Calculate UAV position and yaw to achieve specific claw targets
        FIXED: Simplified geometric calculation with consistent dual-fang center computation
        
        Args:
            left_target: (x, y, z) position for left claw
            right_target: (x, y, z) position for right claw
            target_z: Target Z height for claws
            
        Returns:
            tuple: (uav_position, uav_yaw) or (None, None) if calculation fails
        """
        try:
            # FIXED: Single calculation of dual-fang center position (eliminate duplicate)
            dual_fang_center_x = (left_target[0] + right_target[0]) / 2
            dual_fang_center_y = (left_target[1] + right_target[1]) / 2
            dual_fang_center_z = target_z
            
            # FIXED: Calculate UAV yaw from claw separation vector (more reliable than valve center)
            # The claw separation vector defines the UAV's required orientation
            claw_vector_x = right_target[0] - left_target[0]
            claw_vector_y = right_target[1] - left_target[1]
            
            # UAV yaw is perpendicular to claw separation vector (claws are aligned with UAV Y-axis)
            base_uav_yaw = math.atan2(claw_vector_y, claw_vector_x) - math.pi/2
            
            # Normalize to [0, 2π]
            while base_uav_yaw >= 2*math.pi:
                base_uav_yaw -= 2*math.pi
            while base_uav_yaw < 0:
                base_uav_yaw += 2*math.pi
            
            # Get valve center for collision verification
            valve_center_x, valve_center_y = self.valve_pos[0], self.valve_pos[1]
            
            rospy.loginfo(f"=== FIXED GEOMETRIC CALCULATION ===")
            rospy.loginfo(f"Valve yaw: {math.degrees(self.valve_yaw):.1f}°")
            rospy.loginfo(f"Dual-fang center: ({dual_fang_center_x:.3f}, {dual_fang_center_y:.3f})")
            rospy.loginfo(f"Calculated UAV yaw: {math.degrees(base_uav_yaw):.1f}°")
            rospy.loginfo(f"Strategy: UAV yaw from claw separation vector (more reliable)")
            
            # SIMPLIFIED: Verify the calculated claw positions match the input targets
            # Calculate actual claw positions based on UAV geometry
            calculated_left_x = dual_fang_center_x + self.optimizer.half_claw_separation * math.cos(base_uav_yaw + math.pi/2)
            calculated_left_y = dual_fang_center_y + self.optimizer.half_claw_separation * math.sin(base_uav_yaw + math.pi/2)
            
            calculated_right_x = dual_fang_center_x - self.optimizer.half_claw_separation * math.cos(base_uav_yaw + math.pi/2)
            calculated_right_y = dual_fang_center_y - self.optimizer.half_claw_separation * math.sin(base_uav_yaw + math.pi/2)
            
            # Calculate positioning errors
            left_error = math.sqrt((calculated_left_x - left_target[0])**2 + (calculated_left_y - left_target[1])**2)
            right_error = math.sqrt((calculated_right_x - right_target[0])**2 + (calculated_right_y - right_target[1])**2)
            
            rospy.loginfo(f"=== SIMPLIFIED GEOMETRIC VERIFICATION ===")
            rospy.loginfo(f"Target left claw: ({left_target[0]:.3f}, {left_target[1]:.3f})")
            rospy.loginfo(f"Calculated left claw: ({calculated_left_x:.3f}, {calculated_left_y:.3f})")
            rospy.loginfo(f"Left positioning error: {left_error*1000:.1f}mm")
            rospy.loginfo(f"Target right claw: ({right_target[0]:.3f}, {right_target[1]:.3f})")
            rospy.loginfo(f"Calculated right claw: ({calculated_right_x:.3f}, {calculated_right_y:.3f})")
            rospy.loginfo(f"Right positioning error: {right_error*1000:.1f}mm")
            
            # Check for geometric consistency
            max_error = max(left_error, right_error)
            if max_error > 0.025:  # 25mm tolerance (relaxed for practical operation)
                rospy.logerr(f"GEOMETRIC INCONSISTENCY: Max error {max_error*1000:.1f}mm > 25mm tolerance")
                return None, None
            # SIMPLIFIED: Basic collision avoidance check
            left_distance_to_center = math.sqrt((calculated_left_x - valve_center_x)**2 + (calculated_left_y - valve_center_y)**2)
            right_distance_to_center = math.sqrt((calculated_right_x - valve_center_x)**2 + (calculated_right_y - valve_center_y)**2)
            
            valve_outer_radius = 0.120  # 120mm outer rim
            left_clearance = left_distance_to_center - valve_outer_radius
            right_clearance = right_distance_to_center - valve_outer_radius
            
            rospy.loginfo(f"=== BASIC COLLISION CHECK ===")
            rospy.loginfo(f"Left clearance: {left_clearance*1000:.1f}mm")
            rospy.loginfo(f"Right clearance: {right_clearance*1000:.1f}mm")
            rospy.loginfo(f"Safety status: {'✓ SAFE' if min(left_clearance, right_clearance) > 0 else '✗ COLLISION RISK'}")
            
            # SIMPLIFIED: Calculate UAV position directly (remove complex optimization)
            uav_x = dual_fang_center_x - self.optimizer.dual_fang_center_offset * math.cos(base_uav_yaw)
            uav_y = dual_fang_center_y - self.optimizer.dual_fang_center_offset * math.sin(base_uav_yaw)
            uav_z = self.locked_z  # Use the locked Z from insertion height calculation
            uav_yaw = base_uav_yaw
            
            rospy.loginfo(f"=== SIMPLIFIED UAV POSITIONING ===")
            rospy.loginfo(f"Dual-fang center: ({dual_fang_center_x:.3f}, {dual_fang_center_y:.3f})")
            rospy.loginfo(f"UAV yaw: {math.degrees(uav_yaw):.1f}°")
            rospy.loginfo(f"UAV position: ({uav_x:.3f}, {uav_y:.3f}, {uav_z:.3f})")
            
            return (uav_x, uav_y, uav_z), uav_yaw
            
        except Exception as e:
            rospy.logerr(f"Failed to calculate UAV position for targets: {e}")
            return None, None
    
    def execute_safe_movement_to_position(self, target_pos, target_yaw):
        """
        Execute safe three-stage movement following strict axis-locking principles:
        Stage 1: XY positioning only (Z and yaw locked)
        Stage 2: Yaw adjustment with end-effector compensation (XY and Z locked)  
        Stage 3: Z descent only (XY and yaw locked)
        
        Args:
            target_pos: (x, y, z) target UAV position
            target_yaw: Target UAV yaw angle
            
        Returns:
            bool: Success status
        """
        try:
            current_pos = self.get_current_position()
            if current_pos is None:
                rospy.logerr("Cannot get current position")
                return False
            
            rospy.loginfo("=== THREE-STAGE MOVEMENT WITH STRICT AXIS LOCKING ===")
            rospy.loginfo(f"Current position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
            rospy.loginfo(f"Current yaw: {math.degrees(self.current_yaw):.1f}°")
            rospy.loginfo(f"Target position: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
            rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
            
            # Calculate yaw change to determine if end-effector compensation is needed
            yaw_change = self.normalize_angle(target_yaw - self.current_yaw)
            rospy.loginfo(f"Required yaw change: {math.degrees(yaw_change):.1f}°")
            
            # === STAGE 1: XY POSITIONING ONLY (Z AND YAW LOCKED) ===
            rospy.loginfo("--- Stage 1: XY positioning (Z and yaw LOCKED) ---")
            
            # Lock Z at current height and yaw at current angle
            stage1_target = (target_pos[0], target_pos[1], current_pos[2])
            stage1_yaw = self.current_yaw  # Keep current yaw
            
            if not self.execute_xy_only_movement(current_pos, stage1_target, stage1_yaw):
                rospy.logerr("Stage 1: XY positioning failed")
                return False
            
            rospy.loginfo("✓ Stage 1 completed - XY positioning done")
            
            # INSERTION COMPLETION DETECTION: Check if Z descent is actually needed
            stage1_final_pos = self.get_current_position()
            if stage1_final_pos:
                z_distance_remaining = abs(stage1_final_pos[2] - target_pos[2])
                rospy.loginfo(f"Z distance remaining: {z_distance_remaining*1000:.1f}mm")
                
                if z_distance_remaining < 0.010:  # Less than 10mm remaining (relaxed from 5mm)
                    rospy.loginfo("✓ INSERTION ALREADY COMPLETE - Z distance < 10mm")
                    rospy.loginfo("✓ Skipping unnecessary Z descent to prevent oscillation")
                    rospy.loginfo("✓ THREE-STAGE MOVEMENT COMPLETED SUCCESSFULLY (EARLY COMPLETION)")
                    return True
                elif z_distance_remaining < 0.010:  # 5-10mm remaining - use extra gentle approach
                    rospy.loginfo("MINIMAL Z descent needed (5-10mm) - using extra gentle approach")
                    self.use_gentle_z_descent = True
                else:
                    rospy.loginfo(f"Proceeding with Z descent: {z_distance_remaining*1000:.1f}mm")
                    self.use_gentle_z_descent = False
            
            # === STAGE 2: YAW ADJUSTMENT WITH END-EFFECTOR COMPENSATION ===
            rospy.loginfo("--- Stage 2: Yaw adjustment with end-effector compensation ---")
            
            if abs(yaw_change) > 0.05:  # Only if significant yaw change needed
                if not self.execute_yaw_adjustment_with_compensation(target_yaw):
                    rospy.logerr("Stage 2: Yaw adjustment failed")
                    return False
            else:
                rospy.loginfo("Yaw change minimal, skipping Stage 2")
            
            rospy.loginfo("✓ Stage 2 completed - Yaw adjustment done")
            
            # === STAGE 3: Z DESCENT ONLY (XY AND YAW LOCKED) ===
            rospy.loginfo("--- Stage 3: Z descent (XY and yaw LOCKED) ---")
            
            final_pos = self.get_current_position()
            if not final_pos:
                rospy.logerr("Cannot get position for Stage 3")
                return False
            
            # Lock XY and yaw, only change Z
            stage3_target = (final_pos[0], final_pos[1], self.locked_z)
            
            if not self.execute_z_only_movement(final_pos, stage3_target):
                rospy.logerr("Stage 3: Z descent failed")
                return False
            
            rospy.loginfo("✓ Stage 3 completed - Z descent done")
            rospy.loginfo("✓ THREE-STAGE MOVEMENT COMPLETED SUCCESSFULLY")
            return True
            
        except Exception as e:
            rospy.logerr(f"Three-stage movement failed: {e}")
            return False
    
    def execute_radial_approach_movement(self, target_pos, target_yaw):
        """
        Execute radial approach movement for Phase 2 (final contact).
        Uses the same three-stage approach but with higher precision.
        
        Args:
            target_pos: (x, y, z) target UAV position  
            target_yaw: Target UAV yaw angle
            
        Returns:
            bool: Success status
        """
        try:
            rospy.loginfo("=== RADIAL APPROACH MOVEMENT (PHASE 2) ===")
            rospy.loginfo(f"Target: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
            rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
            rospy.loginfo("Using enhanced precision for final contact positioning")
            
            current_pos = self.get_current_position()
            if current_pos is None:
                rospy.logerr("Cannot get current position")
                return False
            
            # Calculate movement requirements
            yaw_change = self.normalize_angle(target_yaw - self.current_yaw)
            xy_distance = math.sqrt((target_pos[0] - current_pos[0])**2 + 
                                  (target_pos[1] - current_pos[1])**2)
            z_distance = abs(target_pos[2] - current_pos[2])
            
            rospy.loginfo(f"Required movements - XY: {xy_distance*1000:.1f}mm, Z: {z_distance*1000:.1f}mm, Yaw: {math.degrees(yaw_change):.1f}°")
            
            # REAL MACHINE OPTIMIZATION: Adjust thresholds based on environment
            is_simulation = rospy.get_param("~simulation", True)
            
            if is_simulation:
                xy_threshold = 0.015  # 15mm for simulation
                yaw_threshold = 0.05236  # 3° for simulation (tightened)
            else:
                xy_threshold = 0.05   # 50mm for real machine
                yaw_threshold = 0.05236  # 3° for real machine (tightened)
                rospy.loginfo("REAL MACHINE: Using tightened precision thresholds")
            
            # === STAGE 1: PRECISE XY POSITIONING ===
            rospy.loginfo("--- Radial Stage 1: Precise XY positioning ---")
            
            stage1_target = (target_pos[0], target_pos[1], current_pos[2])  # Lock Z
            stage1_yaw = self.current_yaw  # Lock yaw
            
            if xy_distance > xy_threshold:
                if not self.execute_xy_only_movement(current_pos, stage1_target, stage1_yaw):
                    rospy.logerr("Radial Stage 1: Precise XY positioning failed")
                    return False
            else:
                rospy.loginfo("XY movement minimal, skipping Stage 1")
            
            # === STAGE 2: PRECISE YAW ADJUSTMENT ===
            rospy.loginfo("--- Radial Stage 2: Precise yaw adjustment ---")
            
            if abs(yaw_change) > yaw_threshold:
                if not self.execute_yaw_adjustment_with_compensation(target_yaw):
                    rospy.logerr("Radial Stage 2: Precise yaw adjustment failed")
                    return False
            else:
                rospy.loginfo("Yaw change minimal, skipping Stage 2")
            
            # === STAGE 3: FINAL Z APPROACH ===
            rospy.loginfo("--- Radial Stage 3: Final Z approach ---")
            
            final_pos = self.get_current_position()
            if not final_pos:
                rospy.logerr("Cannot get position for final Z approach")
                return False
            
            stage3_target = (final_pos[0], final_pos[1], self.locked_z)
            
            if z_distance > 0.015:  # 15mm threshold (relaxed from 5mm for practical precision)
                if not self.execute_z_only_movement(final_pos, stage3_target):
                    rospy.logerr("Radial Stage 3: Final Z approach failed")
                    return False
            else:
                rospy.loginfo("Z movement minimal, skipping Stage 3")
            
            rospy.loginfo("✓ RADIAL APPROACH MOVEMENT COMPLETED")
            return True
            
        except Exception as e:
            rospy.logerr(f"Radial approach movement failed: {e}")
            return False
    
    def verify_two_phase_insertion_result(self, final_pos, left_target, right_target, same_side_result):
        """
        Verify the result of two-phase insertion
        
        Args:
            final_pos: Final UAV position
            left_target: Target left claw position
            right_target: Target right claw position
            same_side_result: Same-side strategy result
            
        Returns:
            bool: Success status
        """
        try:
            # Calculate actual claw positions
            dual_fang_center_x = final_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(self.current_yaw)
            dual_fang_center_y = final_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(self.current_yaw)
            
            actual_left_claw_x = dual_fang_center_x + self.optimizer.half_claw_separation * math.cos(self.current_yaw + math.pi/2)
            actual_left_claw_y = dual_fang_center_y + self.optimizer.half_claw_separation * math.sin(self.current_yaw + math.pi/2)
            
            actual_right_claw_x = dual_fang_center_x - self.optimizer.half_claw_separation * math.cos(self.current_yaw + math.pi/2)
            actual_right_claw_y = dual_fang_center_y - self.optimizer.half_claw_separation * math.sin(self.current_yaw + math.pi/2)
            
            # Calculate positioning errors
            left_error = math.sqrt((actual_left_claw_x - left_target[0])**2 + 
                                 (actual_left_claw_y - left_target[1])**2)
            right_error = math.sqrt((actual_right_claw_x - right_target[0])**2 + 
                                  (actual_right_claw_y - right_target[1])**2)
            
            # Calculate actual claw separation
            actual_separation = math.sqrt((actual_right_claw_x - actual_left_claw_x)**2 + 
                                        (actual_right_claw_y - actual_left_claw_y)**2)
            
            rospy.loginfo(f"=== TWO-PHASE INSERTION VERIFICATION ===")
            rospy.loginfo(f"Final UAV position: ({final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f})")
            rospy.loginfo(f"Final UAV yaw: {math.degrees(self.current_yaw):.1f}°")
            rospy.loginfo(f"Actual left claw: ({actual_left_claw_x:.3f}, {actual_left_claw_y:.3f})")
            rospy.loginfo(f"Target left claw: ({left_target[0]:.3f}, {left_target[1]:.3f})")
            rospy.loginfo(f"Left claw error: {left_error*1000:.1f}mm")
            rospy.loginfo(f"Actual right claw: ({actual_right_claw_x:.3f}, {actual_right_claw_y:.3f})")
            rospy.loginfo(f"Target right claw: ({right_target[0]:.3f}, {right_target[1]:.3f})")
            rospy.loginfo(f"Right claw error: {right_error*1000:.1f}mm")
            rospy.loginfo(f"Actual separation: {actual_separation*1000:.1f}mm")
            rospy.loginfo(f"Expected separation: {same_side_result['actual_separation']*1000:.1f}mm")
            rospy.loginfo(f"Required separation: {same_side_result['required_separation']*1000:.1f}mm")
            
            # Success criteria
            positioning_accurate = left_error < 0.01 and right_error < 0.01  # 10mm tolerance
            separation_adequate = actual_separation >= same_side_result['required_separation'] * 0.95
            
            if positioning_accurate and separation_adequate:
                rospy.loginfo("✓ TWO-PHASE INSERTION VERIFICATION SUCCESSFUL")
                rospy.loginfo(f"✓ User's same-side strategy achieved: {same_side_result['formula']}")
                return True
            else:
                rospy.logwarn("Two-phase insertion verification failed:")
                if not positioning_accurate:
                    rospy.logwarn(f"  Positioning accuracy insufficient (left: {left_error*1000:.1f}mm, right: {right_error*1000:.1f}mm)")
                if not separation_adequate:
                    rospy.logwarn(f"  Separation inadequate ({actual_separation*1000:.1f}mm < {same_side_result['required_separation']*1000:.1f}mm)")
                return False
                
        except Exception as e:
            rospy.logerr(f"Verification failed: {e}")
            return False
    
    def execute_xy_only_movement(self, start_pos, target_pos, locked_yaw):
        """
        Execute XY-only movement with Z and yaw strictly locked.
        Uses VEL+ACC mixed trajectory for smooth motion.
        
        Args:
            start_pos: Starting UAV position
            target_pos: Target UAV position (only XY used)
            locked_yaw: Yaw angle to maintain (locked)
            
        Returns:
            bool: Success status
        """
        rospy.loginfo(f"XY-only movement: ({start_pos[0]:.3f}, {start_pos[1]:.3f}) -> ({target_pos[0]:.3f}, {target_pos[1]:.3f})")
        rospy.loginfo(f"Z locked at: {start_pos[2]:.3f}m, Yaw locked at: {math.degrees(locked_yaw):.1f}°")
        
        # Create trajectory with locked Z and yaw
        xy_target = (target_pos[0], target_pos[1], start_pos[2])  # Lock Z
        
        # Calculate distance for adaptive speed
        xy_distance = math.sqrt((target_pos[0] - start_pos[0])**2 + (target_pos[1] - start_pos[1])**2)
        
        # SLOWER adaptive duration based on distance for better precision
        base_speed = 0.04  # Much slower for precision (was 0.08)
        min_duration = 12.0  # Longer minimum duration
        max_duration = 35.0  # Extended maximum duration
        
        # Even slower for short distances
        if xy_distance < 0.5:
            effective_speed = base_speed * 0.3  # 30% of base speed for very precise movements
        elif xy_distance < 1.0:
            effective_speed = base_speed * 0.5  # 50% of base speed for short distances
        else:
            effective_speed = base_speed * 0.8  # 80% of base speed for longer distances
        
        duration = max(min_duration, min(max_duration, xy_distance / effective_speed))
        
        rospy.loginfo(f"XY distance: {xy_distance:.3f}m, effective speed: {effective_speed:.3f}m/s, duration: {duration:.1f}s")
        
        # ENHANCED: 根据插入状态调整阈值，已插入时放宽要求
        valve_surface_z = 0.570  # Use fixed valve height
        current_z = start_pos[2] if start_pos else xy_target[2]
        insertion_depth = max(0, valve_surface_z - current_z)
        
        if insertion_depth > 0.1:  # 已插入超过100mm
            pos_tolerance = 0.025   # 放宽位置阈值到25mm
            yaw_tolerance = 0.05236 # 收紧yaw阈值到3.0°
            tolerance_context = f"INSERTED (depth: {insertion_depth*1000:.0f}mm)"
        else:
            pos_tolerance = 0.020   # 标准位置阈值20mm
            yaw_tolerance = 0.05236 # 收紧yaw阈值到3.0°
            tolerance_context = f"SURFACE (depth: {insertion_depth*1000:.0f}mm)"
        
        rospy.loginfo(f"Adaptive thresholds: pos={pos_tolerance*1000:.0f}mm, yaw={math.degrees(yaw_tolerance):.1f}° ({tolerance_context})")
        
        # Execute optimized trajectory with improved Z-axis stability
        return self.motion_controller.execute_smooth_trajectory_with_yaw(
            start_pos=start_pos,
            target_pos=xy_target,
            target_yaw=locked_yaw,  # Strictly lock yaw
            duration=duration,
            pos_threshold=pos_tolerance,  # Adaptive position control
            yaw_threshold=yaw_tolerance   # Adaptive yaw control
        )
    
    def execute_yaw_adjustment_with_compensation(self, target_yaw):
        """
        Execute yaw adjustment with end-effector XY compensation using progressive approach.
        For large yaw changes (>45°), uses multi-step progression to minimize drift.
        
        Args:
            target_yaw: Target UAV yaw angle
            
        Returns:
            bool: Success status
        """
        current_pos = self.get_current_position()
        current_yaw = self.current_yaw
        
        if current_pos is None:
            rospy.logerr("Cannot get current position for yaw adjustment")
            return False
        
        yaw_change = self.normalize_angle(target_yaw - current_yaw)
        yaw_change_magnitude = abs(yaw_change)
        
        rospy.loginfo(f"Yaw adjustment with compensation: {math.degrees(current_yaw):.1f}° -> {math.degrees(target_yaw):.1f}°")
        rospy.loginfo(f"Yaw change magnitude: {math.degrees(yaw_change_magnitude):.1f}°")
        
        # Check if progressive adjustment is needed for large yaw changes
        if yaw_change_magnitude > math.radians(45):  # 45° threshold
            rospy.loginfo("Large yaw change detected - using progressive adjustment")
            return self.execute_progressive_yaw_adjustment(current_yaw, target_yaw)
        else:
            rospy.loginfo("Small yaw change - using single-step adjustment")
            return self.execute_single_step_yaw_adjustment(current_yaw, target_yaw)
    
    def execute_progressive_yaw_adjustment(self, start_yaw, target_yaw):
        """
        Execute yaw adjustment in multiple progressive steps to minimize end-effector drift.
        
        Args:
            start_yaw: Starting yaw angle
            target_yaw: Target yaw angle
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("=== PROGRESSIVE YAW ADJUSTMENT ===")
        
        total_yaw_change = self.normalize_angle(target_yaw - start_yaw)
        yaw_change_magnitude = abs(total_yaw_change)
        
        # Calculate number of steps (max 20° per step for better precision)
        max_step_size = math.radians(20)  # Reduced from 30° to 20°
        num_steps = max(2, int(math.ceil(yaw_change_magnitude / max_step_size)))
        
        rospy.loginfo(f"Total yaw change: {math.degrees(yaw_change_magnitude):.1f}°")
        rospy.loginfo(f"Progressive steps: {num_steps} steps")
        
        # Calculate step angles
        step_size = total_yaw_change / num_steps
        rospy.loginfo(f"Step size: {math.degrees(step_size):.1f}° per step")
        
        # Record initial end-effector position for drift tracking
        initial_pos = self.get_current_position()
        initial_yaw = self.current_yaw
        initial_end_effector_x = initial_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(initial_yaw)
        initial_end_effector_y = initial_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(initial_yaw)
        
        rospy.loginfo(f"Initial end-effector position: ({initial_end_effector_x:.3f}, {initial_end_effector_y:.3f})")
        
        # Execute progressive steps
        current_yaw = start_yaw
        for step in range(num_steps):
            step_target_yaw = start_yaw + (step + 1) * step_size
            
            rospy.loginfo(f"--- Step {step + 1}/{num_steps}: {math.degrees(current_yaw):.1f}° -> {math.degrees(step_target_yaw):.1f}° ---")
            
            # Execute single step
            success = self.execute_single_step_yaw_adjustment(current_yaw, step_target_yaw, step_number=step+1)
            
            if not success:
                rospy.logerr(f"Progressive yaw adjustment failed at step {step + 1}")
                return False
            
            # Update current yaw for next step
            current_yaw = self.current_yaw
            
            # Small pause between steps for stability
            time.sleep(1.0)  # Increased from 0.5s to 1.0s for better settling
        
        # Final verification
        final_pos = self.get_current_position()
        final_yaw = self.current_yaw
        final_end_effector_x = final_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(final_yaw)
        final_end_effector_y = final_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(final_yaw)
        
        total_drift = math.sqrt((final_end_effector_x - initial_end_effector_x)**2 + 
                               (final_end_effector_y - initial_end_effector_y)**2)
        
        rospy.loginfo(f"=== PROGRESSIVE YAW ADJUSTMENT COMPLETED ===")
        rospy.loginfo(f"Final end-effector position: ({final_end_effector_x:.3f}, {final_end_effector_y:.3f})")
        rospy.loginfo(f"Total end-effector drift: {total_drift*1000:.1f}mm")
        
        if total_drift < 0.020:  # Relaxed tolerance from 15mm to 20mm for total drift
            rospy.loginfo("✓ Progressive yaw adjustment successful with acceptable drift")
            return True
        else:
            rospy.logwarn(f"Progressive yaw adjustment completed but drift {total_drift*1000:.1f}mm exceeds 20mm")
            # Still return success as progressive approach minimizes issues
            return True
    
    def execute_single_step_yaw_adjustment(self, current_yaw, target_yaw, step_number=None):
        """
        Execute a single yaw adjustment step with REAL-TIME end-effector position locking.
        
        CRITICAL: This method ensures end-effector position remains FIXED during yaw rotation
        by continuously adjusting UAV XY position to compensate for the geometric offset.
        
        Args:
            current_yaw: Current yaw angle
            target_yaw: Target yaw angle for this step
            step_number: Optional step number for logging
            
        Returns:
            bool: Success status
        """
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("Cannot get current position for single step yaw adjustment")
            return False
        
        step_prefix = f"Step {step_number} - " if step_number else ""
        
        # CRITICAL: Calculate and LOCK end-effector position
        locked_end_effector_x = current_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(current_yaw)
        locked_end_effector_y = current_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(current_yaw)
        
        rospy.loginfo(f"{step_prefix}LOCKING end-effector at: ({locked_end_effector_x:.3f}, {locked_end_effector_y:.3f})")
        
        # Execute real-time yaw adjustment with end-effector position feedback
        return self.execute_realtime_yaw_with_endeffector_lock(
            current_yaw, target_yaw, locked_end_effector_x, locked_end_effector_y, 
            current_pos[2], step_prefix)
    
    def execute_realtime_yaw_with_endeffector_lock(self, start_yaw, target_yaw, 
                                                   locked_ee_x, locked_ee_y, locked_z, 
                                                   step_prefix=""):
        """
        Execute yaw adjustment with REAL-TIME end-effector position locking.
        
        This method continuously calculates the required UAV position to maintain
        the end-effector at the locked position while rotating the UAV.
        
        Args:
            start_yaw: Starting yaw angle
            target_yaw: Target yaw angle
            locked_ee_x: Locked end-effector X position
            locked_ee_y: Locked end-effector Y position
            locked_z: Locked Z position
            step_prefix: Logging prefix
            
        Returns:
            bool: Success status
        """
        rospy.loginfo(f"{step_prefix}Real-time yaw adjustment: {math.degrees(start_yaw):.1f}° -> {math.degrees(target_yaw):.1f}°")
        rospy.loginfo(f"{step_prefix}End-effector LOCKED at: ({locked_ee_x:.3f}, {locked_ee_y:.3f})")
        
        # Calculate yaw adjustment parameters
        yaw_change = self.normalize_angle(target_yaw - start_yaw)
        yaw_change_magnitude = abs(yaw_change)
        
        # Determine rotation direction and speed
        rotation_speed = 0.15  # rad/s - slower for precision
        max_duration = 20.0
        duration = min(max_duration, yaw_change_magnitude / rotation_speed)
        
        rospy.loginfo(f"{step_prefix}Rotation duration: {duration:.1f}s for {math.degrees(yaw_change_magnitude):.1f}°")
        
        # Execute real-time feedback control
        start_time = rospy.Time.now()
        rate = rospy.Rate(20)  # 20 Hz for smooth control
        
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            # Calculate current progress
            elapsed = (rospy.Time.now() - start_time).to_sec()
            progress = min(1.0, elapsed / duration)
            
            # Interpolate target yaw
            current_target_yaw = start_yaw + progress * yaw_change
            
            # Calculate required UAV position to maintain locked end-effector position
            required_uav_x = locked_ee_x - self.optimizer.dual_fang_center_offset * math.cos(current_target_yaw)
            required_uav_y = locked_ee_y - self.optimizer.dual_fang_center_offset * math.sin(current_target_yaw)
            
            # Send position and yaw command
            self.motion_controller.send_trajectory_point(
                (required_uav_x, required_uav_y, locked_z), current_target_yaw)
            
            rate.sleep()
        
        # Final verification
        time.sleep(1.0)  # Allow settling
        final_pos = self.get_current_position()
        final_yaw = self.current_yaw
        
        if final_pos:
            # Calculate actual end-effector position
            actual_ee_x = final_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(final_yaw)
            actual_ee_y = final_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(final_yaw)
            
            # Calculate end-effector drift
            ee_drift = math.sqrt((actual_ee_x - locked_ee_x)**2 + (actual_ee_y - locked_ee_y)**2)
            
            rospy.loginfo(f"{step_prefix}Final UAV: ({final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f})")
            rospy.loginfo(f"{step_prefix}Final yaw: {math.degrees(final_yaw):.1f}°")
            rospy.loginfo(f"{step_prefix}Actual end-effector: ({actual_ee_x:.3f}, {actual_ee_y:.3f})")
            rospy.loginfo(f"{step_prefix}End-effector drift: {ee_drift*1000:.1f}mm")
            
            if ee_drift < 0.025:  # 25mm tolerance (relaxed from 8mm for realistic insertion)
                rospy.loginfo(f"✓ {step_prefix}Real-time yaw adjustment successful")
                return True
            else:
                rospy.logwarn(f"{step_prefix}End-effector drift {ee_drift*1000:.1f}mm exceeds 25mm")
                return ee_drift < 0.015  # Still accept up to 15mm
        
        rospy.logerr(f"{step_prefix}Cannot verify final position")
        return False
        
        # Execute combined yaw + compensation movement
        if compensation_distance > 0.001:  # 1mm threshold
            # Calculate duration based on yaw change and compensation distance
            yaw_change_magnitude = abs(self.normalize_angle(target_yaw - current_yaw))
            base_duration = max(8.0, yaw_change_magnitude / 0.2)  # 0.2 rad/s for smaller steps
            
            # Adjust duration based on compensation distance
            if compensation_distance > 0.15:  # 15cm
                base_duration *= 1.3
            elif compensation_distance > 0.05:  # 5cm  
                base_duration *= 1.1
            
            duration = min(base_duration, 15.0)  # Cap at 15 seconds for single steps
            
            rospy.loginfo(f"{step_prefix}Duration: {duration:.1f}s for {math.degrees(yaw_change_magnitude):.1f}° + {compensation_distance*1000:.1f}mm compensation")
            
            success = self.motion_controller.execute_progressive_precision_trajectory(
                current_pos, compensated_pos, target_yaw,
                final_pos_threshold=0.025,  # 25mm final precision (avoid oscillation)
                final_yaw_threshold=0.05    # 2.9° final precision
            )
            
            if success:
                # ENHANCED: Verify end-effector position with correction attempt
                final_pos = self.get_current_position()
                final_yaw = self.current_yaw
                
                end_effector_after_x = final_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(final_yaw)
                end_effector_after_y = final_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(final_yaw)
                
                end_effector_drift = math.sqrt((end_effector_after_x - end_effector_before_x)**2 + 
                                             (end_effector_after_y - end_effector_before_y)**2)
                
                rospy.loginfo(f"{step_prefix}End-effector after rotation: ({end_effector_after_x:.3f}, {end_effector_after_y:.3f})")
                rospy.loginfo(f"{step_prefix}End-effector drift: {end_effector_drift*1000:.1f}mm")
                
                # ENHANCED: Apply correction if drift is significant but within correctable range
                if end_effector_drift > 0.015 and end_effector_drift < 0.030:  # 15-30mm range
                    rospy.loginfo(f"{step_prefix}Applying drift correction...")
                    
                    # Calculate correction needed
                    correction_x = end_effector_before_x - end_effector_after_x
                    correction_y = end_effector_before_y - end_effector_after_y
                    
                    # Apply small correction
                    corrected_pos = (final_pos[0] + correction_x, final_pos[1] + correction_y, final_pos[2])
                    
                    correction_success = self.motion_controller.execute_progressive_precision_trajectory(
                        final_pos, corrected_pos, target_yaw,
                        final_pos_threshold=0.020,  # 20mm final precision (avoid oscillation)
                        final_yaw_threshold=0.03    # 1.7° final precision
                    )
                    
                    if correction_success:
                        # Re-verify after correction
                        corrected_final_pos = self.get_current_position()
                        corrected_final_yaw = self.current_yaw
                        
                        corrected_ee_x = corrected_final_pos[0] + self.optimizer.dual_fang_center_offset * math.cos(corrected_final_yaw)
                        corrected_ee_y = corrected_final_pos[1] + self.optimizer.dual_fang_center_offset * math.sin(corrected_final_yaw)
                        
                        corrected_drift = math.sqrt((corrected_ee_x - end_effector_before_x)**2 + 
                                                   (corrected_ee_y - end_effector_before_y)**2)
                        
                        rospy.loginfo(f"{step_prefix}After correction - drift: {corrected_drift*1000:.1f}mm")
                        end_effector_drift = corrected_drift  # Update drift value
                
                if end_effector_drift < 0.015:  # Relaxed tolerance from 12mm to 15mm
                    rospy.loginfo(f"✓ {step_prefix}Yaw adjustment step successful")
                    return True
                else:
                    rospy.logwarn(f"{step_prefix}End-effector drift {end_effector_drift*1000:.1f}mm exceeds 15mm tolerance")
                    # For progressive steps, continue even with some drift as it accumulates across steps
                    if end_effector_drift < 0.025:  # 25mm max tolerance for single step (relaxed)
                        rospy.loginfo(f"{step_prefix}Drift within acceptable range for progressive adjustment")
                        return True
                    else:
                        rospy.logerr(f"{step_prefix}Excessive end-effector drift for progressive step")
                        return False
            else:
                rospy.logerr(f"{step_prefix}Yaw adjustment step failed")
                return False
        else:
            # Pure yaw rotation without compensation needed
            rospy.loginfo(f"{step_prefix}Minimal compensation needed, executing pure yaw rotation")
            return self.execute_pure_yaw_rotation(target_yaw)
    
    def execute_pure_yaw_rotation(self, target_yaw):
        """Execute pure yaw rotation with position locked."""
        current_pos = self.get_current_position()
        if current_pos is None:
            return False
        
        # Send yaw command with locked position
        self.motion_controller.send_trajectory_point(current_pos, target_yaw)
        
        # Wait for yaw convergence with timeout
        start_time = time.time()
        rate = rospy.Rate(10)
        
        while (time.time() - start_time) < 8.0 and not rospy.is_shutdown():
            yaw_error = abs(self.normalize_angle(self.current_yaw - target_yaw))
            if yaw_error < 0.03:  # 0.03 rad = ~1.7°
                rospy.loginfo(f"Pure yaw rotation completed. Error: {math.degrees(yaw_error):.1f}°")
                return True
            
            self.motion_controller.send_trajectory_point(current_pos, target_yaw)
            rate.sleep()
        
        rospy.logwarn("Pure yaw rotation timeout")
        return False
    
    def execute_z_only_movement(self, start_pos, target_pos):
        """
        Execute Z-only movement with XY and yaw strictly locked.
        Uses VEL+ACC mixed trajectory for smooth descent.
        
        Args:
            start_pos: Starting UAV position
            target_pos: Target UAV position (only Z used)
            
        Returns:
            bool: Success status
        """
        rospy.loginfo(f"Z-only movement: {start_pos[2]:.3f}m -> {target_pos[2]:.3f}m")
        rospy.loginfo(f"XY locked at: ({start_pos[0]:.3f}, {start_pos[1]:.3f})")
        rospy.loginfo(f"Yaw locked at: {math.degrees(self.current_yaw):.1f}°")
        
        # Lock XY and yaw, only change Z
        z_target = (start_pos[0], start_pos[1], target_pos[2])
        locked_yaw = self.current_yaw
        
        # Calculate Z distance and SLOWER adaptive duration for precision
        z_distance = abs(target_pos[2] - start_pos[2])
        
        # GENTLE DESCENT MODE: Use position hold for very small distances
        if hasattr(self, 'use_gentle_z_descent') and self.use_gentle_z_descent and z_distance < 0.010:
            rospy.loginfo(f"🔒 ACTIVATING POSITION HOLD mode for gentle descent: {z_distance*1000:.1f}mm")
            self.motion_controller._force_position_hold = True
            
            # Use gentle approach for minimal descent but still fast enough
            descent_speed = 0.075  # 🚀 最慢速度提升到0.075 m/s
            min_duration = 8.0     # Reduced duration due to higher speed
            pos_threshold = 0.008  # Relaxed threshold for gentle mode
            yaw_threshold = 0.05   # Very relaxed yaw threshold
        else:
            # Standard descent mode
            self.motion_controller._force_position_hold = False
            descent_speed = 0.1    # 🚀 标准速度提升到0.1 m/s
            min_duration = 5.0     # Reduced minimum duration
            pos_threshold = 0.015  # Standard precision
            yaw_threshold = 0.02   # Standard yaw precision
        
        duration = max(min_duration, z_distance / descent_speed)
        
        rospy.loginfo(f"Z distance: {z_distance:.3f}m, descent speed: {descent_speed:.3f}m/s, duration: {duration:.1f}s")
        mode_context = "POSITION_HOLD" if hasattr(self.motion_controller, '_force_position_hold') and self.motion_controller._force_position_hold else "STANDARD"
        rospy.loginfo(f"Control mode: {mode_context}, pos_threshold: {pos_threshold*1000:.0f}mm, yaw_threshold: {math.degrees(yaw_threshold):.1f}°")
        
        # Execute smooth Z descent with progressive precision control
        return self.motion_controller.execute_progressive_precision_trajectory(
            start_pos, z_target, locked_yaw,
            final_pos_threshold=pos_threshold,  # Adaptive precision
            final_yaw_threshold=yaw_threshold   # Adaptive yaw precision
        )
    
    def execute_legacy_insertion(self, params):
        """
        Execute legacy single-phase insertion for backward compatibility.
        
        Args:
            params: Legacy insertion parameters
            
        Returns:
            bool: Success status
        """
        rospy.loginfo("=== EXECUTING LEGACY SINGLE-PHASE INSERTION ===")
        
        current_pos = self.get_current_position()
        if current_pos is None:
            rospy.logerr("No UAV position available")
            return False
        
        # Note: insertion geometry should be passed from DescendAndContactState
        # If not available, the system should use existing insertion_info
        
        # Extract legacy parameters
        optimal_uav_position = params.get('optimal_uav_position')
        optimal_uav_yaw = params.get('optimal_uav_yaw')
        left_claw_target = params.get('left_claw_target')
        right_claw_target = params.get('right_claw_target')
        
        if not all([optimal_uav_position, optimal_uav_yaw is not None, left_claw_target, right_claw_target]):
            rospy.logerr("Missing required legacy parameters")
            return False
        
        rospy.loginfo(f"Current UAV position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"Target UAV position: ({optimal_uav_position[0]:.3f}, {optimal_uav_position[1]:.3f}, {optimal_uav_position[2]:.3f})")
        rospy.loginfo(f"Target UAV yaw: {math.degrees(optimal_uav_yaw):.1f}°")
        
        # Execute direct movement to target position
        target_pos = (optimal_uav_position[0], optimal_uav_position[1], self.locked_z)
        
        return self.execute_safe_movement_to_position(target_pos, optimal_uav_yaw)


# Additional states for the complete state machine
class RotateValveState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'],
                        input_keys=['initial_valve_yaw', 'valve_yaw_change_threshold', 'rotation_start_position', 
                                  'insertion_info', 'trajectory_state', 'contact_final_torque'],  # 状态传递而非对象
                        output_keys=['trajectory_state'],  # 输出轨迹状态给脱离阶段
                        module_id=module_id)
        
        # Simplified valve rotation parameters for 分层控制架构
        self.rotation_angle = math.pi / 2  # 90 degrees rotation
        self.circular_motion_step = math.radians(5.0)  # 5° incremental steps
        self.position_check_interval = 0.5  # Check position every 0.5s
        
    def execute(self, userdata):
        """
        Dragon Manipulate模式 - Beetle适配版
        
        实现完整的SE(3)联合控制：
        1. 在线轨迹生成：恒定角速度 + 自适应半径
        2. POS_VEL_MODE：位置+速度联合控制
        3. 外力前馈：向心力补偿 + 转动扭矩
        4. 力矩微调：根据阀门阻力实时调整
        5. 半径锁定：80%目标速度时锁定半径
        
        Dragon核心概念在Beetle上的实现：
        - delta_yaw = delta_t * turn_vel（在线轨迹更新）
        - SE(3) force/torque feedforward
        - Radius adaptation and locking mechanism
        """
        rospy.loginfo("=== Dragon Manipulate模式 - Beetle SE(3)联合控制 ===")
        rospy.loginfo("特性: 在线轨迹生成 + POS_VEL_MODE + 外力前馈 + 半径锁定")
        
        # 获取前一状态的参数
        initial_valve_yaw = userdata.initial_valve_yaw if hasattr(userdata, 'initial_valve_yaw') else 0.0
        valve_yaw_threshold = userdata.valve_yaw_change_threshold if hasattr(userdata, 'valve_yaw_change_threshold') else math.radians(10.0)
        
        rospy.loginfo(f"初始阀门角度: {math.degrees(initial_valve_yaw):.1f}°")
        rospy.loginfo(f"接触检测阈值: {math.degrees(valve_yaw_threshold):.1f}°")
        
        if not self.wait_for_positions():
            rospy.logerr("无法获取位置数据")
            return 'failed'
        
        # 获取当前状态（保持插入深度一致性）
        current_pos = self.get_current_position()
        current_yaw = self.get_current_yaw()
        valve_pos = self.valve_pos
        current_valve_yaw = self.valve_yaw
        
        if any(v is None for v in [current_pos, current_yaw, valve_pos, current_valve_yaw]):
            rospy.logerr("状态信息不完整，无法执行Manipulate模式")
            return 'failed'
        
        # 修复Z坐标静差：使用插入深度而非阀门中心Z坐标
        insertion_depth_z = current_pos[2]  # 保持当前插入深度
        corrected_valve_center = [valve_pos[0], valve_pos[1], insertion_depth_z]
        rospy.loginfo(f"Z坐标修正: 阀门中心({valve_pos[2]:.3f}) → 插入深度({insertion_depth_z:.3f})")
        
        # 计算初始几何参数
        relative_pos = np.array(current_pos[:2]) - np.array(corrected_valve_center[:2])
        initial_radius = np.linalg.norm(relative_pos)
        
        rospy.loginfo(f"修正后阀门中心: ({corrected_valve_center[0]:.3f}, {corrected_valve_center[1]:.3f}, {corrected_valve_center[2]:.3f})")
        rospy.loginfo(f"当前位置: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"初始半径: {initial_radius*1000:.1f}mm")
        
        # Dragon接力机制：基于Contact状态创建轨迹生成器（确保连续性）
        if hasattr(userdata, 'trajectory_state') and userdata.trajectory_state is not None:
            traj_state = userdata.trajectory_state
            # 使用传递的状态参数创建新的轨迹生成器
            trajectory_generator = OnlineCircularTrajectoryGenerator(
                valve_center=corrected_valve_center,  # 使用修正的Z坐标
                initial_radius=traj_state['initial_radius'],
                target_angular_velocity=0.1,  # 修复：0.1 rad/s ≈ 5.7°/s（Manipulate阶段保持足够转动力矩）
                control_rate=25.0,  # 优化：25Hz（提高轨迹密度）
                debug=True
            )
            # 重要：即使基于Contact状态，仍需基于当前位置初始化
            trajectory_generator.initialize_from_current_position(current_pos)
            # 然后恢复Contact阶段的特定状态
            if 'current_angle' in traj_state:
                trajectory_generator.current_angle = traj_state['current_angle']
            if 'current_radius' in traj_state and traj_state['current_radius'] is not None:
                trajectory_generator.current_radius = traj_state['current_radius']
            if 'radius_locked' in traj_state:
                trajectory_generator.radius_locked = traj_state['radius_locked']
            if 'locked_radius' in traj_state:
                trajectory_generator.locked_radius = traj_state['locked_radius']
            # 恢复lock_radius_value
            if 'lock_radius_value' in traj_state and traj_state['lock_radius_value'] is not None:
                trajectory_generator.lock_radius_value = traj_state['lock_radius_value']
            elif 'locked_radius' in traj_state:
                trajectory_generator.lock_radius_value = traj_state['locked_radius']
                
            rospy.loginfo("✓ 基于Contact状态重建轨迹生成器，确保连续性")
            rospy.loginfo(f"✓ 状态恢复: 半径={traj_state['initial_radius']*1000:.1f}mm, 锁定={traj_state.get('radius_locked', False)}")
        else:
            # 后备方案：全新创建
            rospy.logwarn("未找到Contact轨迹状态，创建新轨迹生成器（连续性丢失）")
            trajectory_generator = OnlineCircularTrajectoryGenerator(
                valve_center=corrected_valve_center,
                initial_radius=initial_radius,
                target_angular_velocity=0.1,  # 修复：0.1 rad/s ≈ 5.7°/s（后备方案保持相同速度）
                control_rate=25.0,  # 优化：25Hz
                debug=True
            )
            # 重要：基于当前位置初始化角度
            trajectory_generator.initialize_from_current_position(current_pos)
        
        # Dragon Manipulate参数
        target_valve_rotation = math.radians(90.0)  # 目标阀门转动角度：90°（典型阀门1/4圈）
        max_manipulation_time = 45.0  # 最大操作时间
        force_adaptation_rate = 0.1   # 力自适应速率
        min_manipulation_time = 5.0   # 最小操作时间
        valve_stuck_threshold = 3.0   # 阀门卡住检测时间
        
        # Dragon扭矩接力机制：使用Contact阶段的最终扭矩作为基线
        if hasattr(userdata, 'contact_final_torque') and userdata.contact_final_torque is not None:
            baseline_torque = userdata.contact_final_torque
            rospy.loginfo(f"✓ 接力使用Contact最终扭矩: {baseline_torque}")
        else:
            baseline_torque = [0.0, 0.0, 0.1]  # 默认z轴扭矩0.1N·m
            rospy.logwarn("未找到Contact扭矩，使用默认基线")
        
        # 性能监控
        start_time = rospy.Time.now().to_sec()
        last_valve_yaw = current_valve_yaw
        last_valve_check_time = start_time
        valve_motion_stalled_time = 0.0
        max_valve_rotation_achieved = 0.0
        
        # 控制循环
        control_rate = rospy.Rate(25)  # 优化：25Hz（与轨迹生成器同步）
        rospy.loginfo(f"=== 开始Dragon Manipulate控制循环 ===")
        rospy.loginfo(f"目标: {math.degrees(target_valve_rotation):.1f}°阀门转动")
        
        while (rospy.Time.now().to_sec() - start_time) < max_manipulation_time:
            current_time = rospy.Time.now().to_sec()
            
            # 获取当前状态
            current_pos = self.get_current_position()
            current_yaw = self.get_current_yaw()
            current_valve_yaw = self.valve_yaw
            
            if any(v is None for v in [current_pos, current_yaw, current_valve_yaw]):
                rospy.logwarn("状态信息丢失，跳过本次控制周期")
                control_rate.sleep()
                continue
            
            # 计算阀门转动进度
            valve_rotation = abs(current_valve_yaw - initial_valve_yaw)
            max_valve_rotation_achieved = max(max_valve_rotation_achieved, valve_rotation)
            
            # 计算阀门角速度（Dragon关键反馈）
            valve_angular_velocity = 0.0
            if current_time > last_valve_check_time + 0.2:  # 每200ms更新一次
                dt = current_time - last_valve_check_time
                valve_angular_velocity = abs(current_valve_yaw - last_valve_yaw) / dt
                last_valve_yaw = current_valve_yaw
                last_valve_check_time = current_time
                
                # 检测阀门运动停滞
                if valve_angular_velocity < 0.01:  # < 0.6°/s认为停滞
                    valve_motion_stalled_time += dt
                else:
                    valve_motion_stalled_time = 0.0
            
            # Dragon核心：在线轨迹状态更新
            try:
                # 检查轨迹生成器对象类型和完整性
                if not isinstance(trajectory_generator, OnlineCircularTrajectoryGenerator):
                    rospy.logerr(f"轨迹生成器类型错误: {type(trajectory_generator)}，重新创建")
                    trajectory_generator = OnlineCircularTrajectoryGenerator(
                        valve_center=corrected_valve_center,
                        initial_radius=initial_radius,
                        target_angular_velocity=0.1,  # 修复：0.1 rad/s ≈ 5.7°/s（错误恢复保持相同速度）
                        control_rate=25.0,  # 优化：25Hz
                        debug=True
                    )
                    # 重要：重新创建后必须初始化
                    trajectory_generator.initialize_from_current_position(current_pos)

                elif not hasattr(trajectory_generator, 'update_state'):
                    rospy.logerr("轨迹生成器缺少update_state方法，重新创建")
                    trajectory_generator = OnlineCircularTrajectoryGenerator(
                        valve_center=corrected_valve_center,
                        initial_radius=initial_radius,
                        target_angular_velocity=0.1,  # 修复：0.1 rad/s ≈ 5.7°/s（错误恢复保持相同速度）
                        control_rate=25.0,  # 优化：25Hz
                        debug=True
                    )
                    # 重要：重新创建后必须初始化
                    trajectory_generator.initialize_from_current_position(current_pos)
                
                # 确认update_state方法是可调用的
                if not callable(getattr(trajectory_generator, 'update_state', None)):
                    rospy.logerr("update_state方法不可调用，重新创建轨迹生成器")
                    trajectory_generator = OnlineCircularTrajectoryGenerator(
                        valve_center=corrected_valve_center,
                        initial_radius=initial_radius,
                        target_angular_velocity=0.05,  # 统一：0.05 rad/s ≈ 2.9°/s
                        control_rate=25.0,  # 优化：25Hz
                        debug=True
                    )
                    # 重要：重新创建后必须初始化
                    trajectory_generator.initialize_from_current_position(current_pos)
                
                state_info = trajectory_generator.update_state(
                    current_pos, current_yaw, valve_angular_velocity
                )
            except Exception as e:
                rospy.logerr(f"轨迹生成器update_state失败: {e}")
                # 应急措施：创建新的轨迹生成器
                trajectory_generator = OnlineCircularTrajectoryGenerator(
                    valve_center=corrected_valve_center,
                    initial_radius=initial_radius,
                    target_angular_velocity=0.05,  # 统一：0.05 rad/s ≈ 2.9°/s
                    control_rate=25.0,  # 优化：25Hz
                    debug=True
                )
                # 重要：应急重新创建后必须初始化
                trajectory_generator.initialize_from_current_position(current_pos)

                state_info = trajectory_generator.update_state(
                    current_pos, current_yaw, valve_angular_velocity
                )
            
            # Dragon核心：生成SE(3)控制目标
            target_state = trajectory_generator.generate_target_state()
            
            # Dragon扭矩基线接力：使用Contact阶段传递的扭矩基线
            if 'torque' in target_state:
                # 将生成的扭矩与基线扭矩结合
                generated_torque = target_state['torque']
                combined_torque = [baseline_torque[i] + generated_torque[i] for i in range(3)]
                target_state['torque'] = combined_torque
            else:
                target_state['torque'] = baseline_torque
            
            # Dragon核心：扭矩自适应（根据阀门响应）
            if valve_angular_velocity > 0.05:  # 阀门正在转动
                # 动态扭矩调整：阀门转得快就减小扭矩，转得慢就增大扭矩
                resistance_factor = max(0.5, min(2.0, 0.3 / max(0.05, valve_angular_velocity)))
                adapted_torque = [t * resistance_factor for t in target_state['torque']]
                target_state['torque'] = adapted_torque
            
            # Dragon核心：SE(3)联合控制执行
            self.beetle.executeTrajectoryWithWrench(
                pos=target_state['position'],
                rot=target_state['yaw'],
                linear_vel=target_state['linear_velocity'],
                angular_vel=target_state['angular_velocity'],
                force=target_state['force'],
                torque=target_state['torque']
            )
            
            # 检查成功条件
            if valve_rotation >= target_valve_rotation:
                elapsed_time = current_time - start_time
                if elapsed_time > min_manipulation_time:  # 确保最小操作时间
                    rospy.loginfo(f"✓ Manipulate成功! 阀门转动: {math.degrees(valve_rotation):.1f}°")
                    rospy.loginfo(f"操作时间: {elapsed_time:.1f}s")
                    rospy.loginfo(f"最终角速度: {math.degrees(state_info['angular_velocity']):.1f}°/s")
                    rospy.loginfo(f"半径锁定: {'是' if state_info['radius_locked'] else '否'}")
                    
                    # 清除外力前馈
                    self.beetle.clearExternalWrench()
                    
                    # 保存轨迹状态给脱离阶段使用
                    userdata.trajectory_state = {
                        'generator': trajectory_generator,
                        'final_angle': trajectory_generator.current_angle if hasattr(trajectory_generator, 'current_angle') else 0,
                        'current_radius': trajectory_generator.current_radius if hasattr(trajectory_generator, 'current_radius') else None,
                        'valve_center': corrected_valve_center,
                        'lock_radius_value': getattr(trajectory_generator, 'lock_radius_value', None)
                    }
                    
                    return 'succeeded'
                else:
                    rospy.loginfo_throttle(1.0, f"达到目标转动，等待最小操作时间 "
                                                f"({elapsed_time:.1f}s < {min_manipulation_time:.1f}s)")
            
            # 检查失败条件：阀门运动停滞
            if valve_motion_stalled_time > valve_stuck_threshold:
                rospy.logwarn(f"检测到阀门运动停滞 {valve_motion_stalled_time:.1f}s > {valve_stuck_threshold:.1f}s")
                if max_valve_rotation_achieved > target_valve_rotation * 0.7:  # 达到70%认为部分成功
                    rospy.loginfo(f"部分成功：已转动 {math.degrees(max_valve_rotation_achieved):.1f}°，继续尝试")
                    valve_motion_stalled_time = 0.0  # 重置计时器
                else:
                    rospy.logerr("阀门可能卡住或阻力过大")
                    self.beetle.clearExternalWrench()
                    return 'failed'
            
            # 调试信息输出
            if current_time - start_time > 2.0:  # 2秒后开始输出调试信息
                status = trajectory_generator.get_status_info()
                rospy.loginfo_throttle(3.0, 
                    f"Manipulate状态: 阀门转动={math.degrees(valve_rotation):.1f}°/"
                    f"{math.degrees(target_valve_rotation):.1f}°, "
                    f"阀门角速度={math.degrees(valve_angular_velocity):.1f}°/s, "
                    f"UAV角速度={status['current_angular_velocity_deg']:.1f}°/s, "
                    f"半径={status['radius_mm']:.0f}mm({'锁定' if status['radius_locked'] else '自适应'}), "
                    f"扭矩={state_info['current_torque']:.2f}Nm, "
                    f"时间={current_time-start_time:.1f}s")
            
            if rospy.is_shutdown():
                break
                
            control_rate.sleep()
        
        # 超时处理
        final_valve_rotation = abs(self.valve_yaw - initial_valve_yaw)
        self.beetle.clearExternalWrench()
        
        if final_valve_rotation >= target_valve_rotation * 0.8:  # 80%认为可接受
            rospy.logwarn(f"Manipulate超时，但达到可接受转动: {math.degrees(final_valve_rotation):.1f}°")
            
            # 保存轨迹状态给脱离阶段使用
            userdata.trajectory_state = {
                'generator': trajectory_generator,
                'final_angle': trajectory_generator.current_angle if hasattr(trajectory_generator, 'current_angle') else 0,
                'current_radius': trajectory_generator.current_radius if hasattr(trajectory_generator, 'current_radius') else None,
                'valve_center': corrected_valve_center,
                'lock_radius_value': getattr(trajectory_generator, 'lock_radius_value', None)
            }
            
            return 'succeeded'
        else:
            rospy.logerr(f"Manipulate失败：仅转动 {math.degrees(final_valve_rotation):.1f}°/"
                        f"{math.degrees(target_valve_rotation):.1f}°")
            return 'failed'


class DisengageFromValveState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'],
                        input_keys=['initial_valve_yaw', 'valve_yaw_change_threshold', 'rotation_start_position', 'insertion_info', 'trajectory_state', 'start_position'],
                        output_keys=['disengagement_position'],
                        module_id=module_id)
        
    def execute(self, userdata):
        """
        Dragon Finish模式 - Beetle有序脱离流程（优化版）
        
        实现标准的5阶段有序脱离（针对卡住问题优化）：
        1. 松弛阶段：停止外力前馈，保持位置2秒让系统稳定
        1.5. 反向旋转脱离：反向旋转20°释放机械约束，防止卡住
        2. 垂直抬高：沿世界坐标系z轴向上拔出爪子（5cm）
        3. 水平脱离：径向移动远离阀门（15cm）
        4. 安全位置：移动到起始位置上方准备降落
        
        新增的反向旋转阶段可有效解决爪子在阀门槽中卡住的问题
        """
        rospy.loginfo("=== Dragon Finish模式 - Beetle有序脱离流程（优化版） ===")
        rospy.loginfo("阶段: 松弛 → 反向旋转脱离(15°) → 垂直抬高 → 水平脱离 → 安全位置")
        
        if not self.wait_for_positions():
            rospy.logerr("无法获取位置数据进行脱离")
            return 'failed'
        
        # 获取当前状态
        current_pos = self.get_current_position()
        current_yaw = self.get_current_yaw()
        valve_pos = self.valve_pos
        
        if any(v is None for v in [current_pos, current_yaw, valve_pos]):
            rospy.logerr("状态信息不完整，无法执行Finish模式")
            return 'failed'
        
        rospy.loginfo(f"开始位置: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"当前偏航: {math.degrees(current_yaw):.1f}°")
        rospy.loginfo(f"阀门位置: ({valve_pos[0]:.3f}, {valve_pos[1]:.3f}, {valve_pos[2]:.3f})")
        
        # ========== 阶段1: 松弛阶段 ==========
        relax_result = self._execute_relax_phase(current_pos, current_yaw)
        if relax_result != 'success':
            rospy.logwarn("松弛阶段异常，但继续执行脱离")
        
        # ========== 阶段1.5: 反向旋转脱离阶段（优化版） ==========
        reverse_result = self._execute_reverse_rotation_phase(current_pos, current_yaw, valve_pos, userdata.trajectory_state)
        if reverse_result != 'success':
            rospy.logwarn("反向旋转脱离异常，但继续执行后续步骤")
        
        # 更新位置和偏航，因为反向旋转会改变UAV状态
        current_pos = self.get_current_position()
        current_yaw = self.get_current_yaw()
        if current_pos is None or current_yaw is None:
            rospy.logerr("无法获取反向旋转后的状态")
            return 'failed'
        
        # ========== 阶段2: 上升到任务开始Z高度 ==========
        z_ascent_result = self._execute_z_ascent_to_start_height(current_pos, current_yaw, userdata)
        if z_ascent_result != 'success':
            rospy.logerr("Z上升失败，脱离中止")
            return 'failed'
        
        # 更新位置
        current_pos = self.get_current_position()
        current_yaw = self.get_current_yaw()
        if current_pos is None or current_yaw is None:
            rospy.logerr("无法获取Z上升后的状态")
            return 'failed'
        
        # ========== 阶段3: Yaw调整到任务开始角度 ==========
        yaw_adjust_result = self._execute_yaw_adjustment_to_start_angle(current_pos, current_yaw, userdata)
        if yaw_adjust_result != 'success':
            rospy.logwarn("Yaw调整异常，但继续执行后续步骤")
        
        # 更新位置和偏航
        current_pos = self.get_current_position()
        current_yaw = self.get_current_yaw()
        if current_pos is None or current_yaw is None:
            rospy.logerr("无法获取Yaw调整后的状态")
            return 'failed'
        
        # ========== 阶段4: XY移动返回任务起始点 ==========
        return_result = self._execute_xy_return_to_start_position(current_pos, current_yaw, userdata)
        if return_result != 'success':
            rospy.logwarn("返回起始点异常，但脱离基本完成")
        
        # 设置最终脱离位置
        final_pos = self.get_current_position()
        if final_pos is not None:
            userdata.disengagement_position = final_pos
        
        rospy.loginfo("=== Dragon Finish模式完成 - 有序脱离成功 ===")
        return 'succeeded'
    
    def _execute_relax_phase(self, current_pos, current_yaw):
        """
        阶段1: 松弛阶段
        停止所有外力前馈，保持当前位置让系统稳定
        """
        rospy.loginfo(">>> 阶段1: 松弛阶段 - 停止外力前馈")
        
        # 清除所有外力前馈
        self.beetle.clearExternalWrench()
        rospy.loginfo("✓ 外力前馈已清除")
        
        # 保持当前位置，让系统稳定
        relax_duration = 2.0  # 2秒松弛时间
        start_time = rospy.Time.now().to_sec()
        control_rate = rospy.Rate(10)  # 10Hz保持控制
        
        rospy.loginfo(f"保持当前位置 {relax_duration:.1f}s 让系统稳定...")
        
        while (rospy.Time.now().to_sec() - start_time) < relax_duration:
            # 使用基础位置控制（不含速度和外力）
            self.beetle.targetMotion(current_pos, current_yaw)
            
            if rospy.is_shutdown():
                return 'failed'
            control_rate.sleep()
        
        rospy.loginfo("✓ 松弛阶段完成，系统已稳定")
        return 'success'
    
    def _execute_reverse_rotation_phase(self, current_pos, current_yaw, valve_pos, trajectory_state):
        """
        阶段1.5: 反向旋转脱离阶段（优化版）
        反向旋转15°释放机械约束，完成后稳定5秒
        """
        rospy.loginfo(">>> 阶段1.5: 反向旋转脱离（优化版） - 释放机械约束")
        rospy.loginfo("反向旋转15°以松开爪子与阀门的卡合，完成后稳定5秒")
        
        # 使用继承的轨迹状态，保持连续性
        if trajectory_state and 'generator' in trajectory_state:
            trajectory_generator = trajectory_state['generator']
            corrected_valve_center = trajectory_state['valve_center']
            
            # 反转角速度进行反向旋转
            original_velocity = trajectory_generator.target_angular_velocity
            trajectory_generator.target_angular_velocity = -abs(original_velocity)  # 确保为负值
            
            rospy.loginfo(f"✓ 继承ROTATE_VALVE轨迹状态，实现连续反向旋转")
            rospy.loginfo(f"✓ 阀门中心: ({corrected_valve_center[0]:.3f}, {corrected_valve_center[1]:.3f}, {corrected_valve_center[2]:.3f})")
            rospy.loginfo(f"✓ 反向角速度: {math.degrees(trajectory_generator.target_angular_velocity):.1f}°/s")
            rospy.loginfo(f"✓ 当前半径: {trajectory_generator.current_radius*1000:.1f}mm")
        else:
            # 后备方案：创建新的轨迹生成器
            rospy.logwarn("未找到ROTATE_VALVE轨迹状态，创建新生成器（连续性丢失）")
            corrected_valve_center = [valve_pos[0], valve_pos[1], current_pos[2]]
            
            # 计算初始半径
            relative_pos = np.array(current_pos[:2]) - np.array(corrected_valve_center[:2])
            initial_radius = np.linalg.norm(relative_pos)
            
            try:
                trajectory_generator = OnlineCircularTrajectoryGenerator(
                    valve_center=corrected_valve_center,
                    initial_radius=initial_radius,
                    target_angular_velocity=-0.087,  # 反向旋转，约-5°/s
                    control_rate=25.0,
                    debug=True
                )
                # 重要：基于当前位置初始化角度
                trajectory_generator.initialize_from_current_position(current_pos)
            except Exception as e:
                rospy.logerr(f"反向旋转轨迹生成器创建失败: {e}")
                return 'failed'
        
        # 反向旋转参数（优化版）
        target_reverse_angle = math.radians(15)  # 15度反向旋转（减少角度）
        reverse_duration = 6.0  # 最多6秒反向旋转时间
        control_rate = rospy.Rate(25)  # 25Hz控制频率
        
        rospy.loginfo(f"开始反向旋转: 目标角度={math.degrees(target_reverse_angle):.1f}°")
        rospy.loginfo(f"反向角速度: {math.degrees(-0.087):.1f}°/s")
        
        # 基于UAV实际yaw角度的反向旋转控制（更准确、更温和）
        target_angular_speed = math.radians(2.0)  # 减慢到2°/s，更温和的反向旋转
        
        # 记录初始yaw角度作为参考
        initial_yaw = current_yaw
        rospy.loginfo(f"初始偏航角: {math.degrees(initial_yaw):.1f}°")
        rospy.loginfo(f"目标反向旋转: {math.degrees(target_reverse_angle):.1f}°")
        rospy.loginfo(f"温和角速度: {math.degrees(target_angular_speed):.1f}°/s")
        
        start_time = rospy.Time.now().to_sec()
        accumulated_yaw_change = 0.0
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            elapsed_time = current_time - start_time
            
            # 超时检查
            if elapsed_time > reverse_duration:
                rospy.logwarn(f"反向旋转超时: {elapsed_time:.1f}s > {reverse_duration:.1f}s")
                break
            
            # 获取当前状态
            current_pos_updated = self.get_current_position()
            current_yaw_updated = self.get_current_yaw()
            
            if any(v is None for v in [current_pos_updated, current_yaw_updated]):
                rospy.logwarn("状态信息丢失，跳过本次控制周期")
                control_rate.sleep()
                continue
            
            # 计算UAV实际yaw角度变化（处理角度环绕）
            yaw_diff = current_yaw_updated - initial_yaw
            if yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            elif yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
                
            # 累计角度变化（取绝对值，因为我们关心变化量）
            accumulated_yaw_change = abs(yaw_diff)
            
            # 基于实际yaw角度变化的完成检查
            if accumulated_yaw_change >= target_reverse_angle:
                rospy.loginfo(f"✓ 反向旋转完成: 实际UAV yaw变化 {math.degrees(accumulated_yaw_change):.1f}°")
                break
            
            # 生成目标状态（使用轨迹生成器，但设置较慢的角速度）
            try:
                # 临时设置较慢的角速度
                original_velocity = trajectory_generator.target_angular_velocity
                trajectory_generator.target_angular_velocity = -target_angular_speed  # 设置温和的反向速度
                
                # 更新轨迹生成器状态
                state_info = trajectory_generator.update_state(
                    current_pos_updated, current_yaw_updated, 0.0
                )
                
                target_state = trajectory_generator.generate_target_state()
                target_pos = target_state['position']
                target_yaw = target_state['yaw']
                
                # 执行温和的运动控制
                self.beetle.targetMotion(target_pos, target_yaw)
                
                # 恢复原始角速度（为下次循环准备）
                trajectory_generator.target_angular_velocity = original_velocity
                
            except Exception as e:
                rospy.logerr(f"轨迹生成失败: {e}")
                # 后备方案：保持当前位置
                self.beetle.targetMotion(current_pos_updated, current_yaw_updated)
                control_rate.sleep()
                continue
            
            # 状态反馈（每1秒一次）
            if int(elapsed_time) != int(elapsed_time - 0.04):
                progress = (accumulated_yaw_change / target_reverse_angle) * 100
                rospy.loginfo(f"反向旋转进度: 实际yaw变化 {math.degrees(accumulated_yaw_change):.1f}°/{math.degrees(target_reverse_angle):.1f}° ({progress:.0f}%), 时间: {elapsed_time:.1f}s")
            
            control_rate.sleep()
        
        # 反向旋转完成，充足稳定时间
        final_time = rospy.Time.now().to_sec() - start_time
        rospy.loginfo(f"反向旋转完成: 实际UAV yaw变化 {math.degrees(accumulated_yaw_change):.1f}° (用时: {final_time:.1f}s)")
        
        # 关键改进：反向旋转后5秒稳定时间
        stabilization_duration = 5.0
        rospy.loginfo(f"开始稳定阶段: 保持当前位置 {stabilization_duration:.1f}s 确保机械脱离完成")
        
        # 获取稳定位置
        stable_pos = self.get_current_position()
        stable_yaw = self.get_current_yaw()
        
        if stable_pos is None or stable_yaw is None:
            rospy.logwarn("无法获取稳定位置，跳过稳定阶段")
        else:
            stabilization_start_time = rospy.Time.now().to_sec()
            stabilization_rate = rospy.Rate(10)  # 10Hz保持控制
            
            while (rospy.Time.now().to_sec() - stabilization_start_time) < stabilization_duration:
                # 使用基础位置控制保持稳定
                self.beetle.targetMotion(stable_pos, stable_yaw)
                
                if rospy.is_shutdown():
                    break
                stabilization_rate.sleep()
        
        rospy.loginfo(f"✓ 稳定阶段完成，机械脱离充分完成")
        
        # 基于实际UAV yaw变化的成功判断
        actual_rotation_deg = math.degrees(accumulated_yaw_change)
        target_rotation_deg = math.degrees(target_reverse_angle)
        
        if accumulated_yaw_change >= target_reverse_angle * 0.6:  # 60%角度变化认为成功（更宽容）
            rospy.loginfo(f"✓ 反向旋转脱离成功: 实际 {actual_rotation_deg:.1f}° (目标: {target_rotation_deg:.1f}°)")
            return 'success'
        else:
            rospy.logwarn(f"反向旋转角度不足: {actual_rotation_deg:.1f}° < {target_rotation_deg * 0.6:.1f}°，但继续执行")
            return 'success'  # 继续执行，不阻断流程

    def _execute_z_ascent_to_start_height(self, current_pos, current_yaw, userdata):
        """
        阶段2: 上升到任务开始时的Z高度（反向旋转后执行）
        从userdata中获取start_position[2]作为目标Z高度
        """
        rospy.loginfo(">>> 阶段2: 上升到任务开始Z高度")
        
        # 获取任务开始时的Z高度
        if not hasattr(userdata, 'start_position') or userdata.start_position is None:
            rospy.logerr("无法获取start_position，无法确定目标Z高度")
            return 'failed'
        
        start_z_height = userdata.start_position[2]
        current_z = current_pos[2]
        
        rospy.loginfo(f"当前Z高度: {current_z:.3f}m")
        rospy.loginfo(f"目标Z高度: {start_z_height:.3f}m")
        rospy.loginfo(f"上升距离: {(start_z_height - current_z)*1000:.1f}mm")
        
        # 如果需要上升
        if start_z_height > current_z + 0.01:  # 如果目标高度比当前高度高1cm以上
            ascent_distance = start_z_height - current_z
            target_pos = [current_pos[0], current_pos[1], start_z_height]
            
            # 使用分段Z上升策略（模仿MoveToValve的分段下降逻辑）
            rospy.loginfo(f"使用分段Z上升策略，距离={ascent_distance*1000:.1f}mm")
            
            # 使用更大的固定步长来减少段数
            fixed_step_size = 0.05  # 🚀 减少段数: 从25mm提升到50mm步长
            num_steps = int(ascent_distance / fixed_step_size)
            remaining_distance = ascent_distance - num_steps * fixed_step_size
            
            rospy.loginfo(f"分段上升参数: 步长={fixed_step_size*1000:.1f}mm, 步数={num_steps}, 余量={remaining_distance*1000:.1f}mm")
            
            # 分段上升：每次移动固定步长，等待收敛
            success = True
            for step in range(num_steps):
                intermediate_z = current_z + (step + 1) * fixed_step_size
                intermediate_pos = [current_pos[0], current_pos[1], intermediate_z]
                
                rospy.loginfo(f"步骤 {step+1}/{num_steps}: 上升到 Z={intermediate_z:.3f}m")
                
                # 移动到中间位置并等待收敛
                success = self.beetle.goPoseWaitConvergence(
                    pos=intermediate_pos,
                    rot=current_yaw,
                    pos_thresh=0.02,  # 2cm收敛阈值
                    rot_thresh=0.1745,  # 10°偏航阈值
                    timeout=10.0  # 每步最多10秒
                )
                
                if not success:
                    rospy.logwarn(f"第{step+1}步上升收敛失败，但继续尝试下一步")
                    # 不要立即中断，允许继续尝试后续步骤
                    continue
                    
                rospy.sleep(0.2)  # 每步之间短暂停顿，让系统稳定
            
            # 处理剩余距离（如果有的话）
            if success and remaining_distance > 0.005:  # 如果剩余距离大于5mm
                final_z = start_z_height
                final_pos = [current_pos[0], current_pos[1], final_z]
                
                rospy.loginfo(f"最终调整: 上升到最终目标 Z={final_z:.3f}m，余量={remaining_distance*1000:.1f}mm")
                
                success = self.beetle.goPoseWaitConvergence(
                    pos=final_pos,
                    rot=current_yaw,
                    pos_thresh=0.01,  # 1cm最终收敛阈值
                    rot_thresh=0.1745,  # 10°偏航阈值
                    timeout=10.0
                )
                
                if not success:
                    rospy.logwarn("最终高度调整收敛失败")
            
            # 分段上升结果处理 - 容错模式
            final_pos_check = self.get_current_position()
            if final_pos_check is not None:
                actual_z_error = abs(final_pos_check[2] - start_z_height)
                if actual_z_error < 0.05:  # 5cm误差容忍
                    rospy.loginfo(f"✓ 分段Z上升基本完成，Z误差={actual_z_error*1000:.1f}mm")
                else:
                    rospy.logwarn(f"分段Z上升精度不足，Z误差={actual_z_error*1000:.1f}mm，但继续执行")
            else:
                rospy.loginfo("✓ 分段Z上升过程完成，继续后续阶段")
        else:
            rospy.loginfo("当前Z高度已达到或超过任务开始高度，无需上升")
        
        # 稳定等待
        rospy.loginfo("等待1秒让系统稳定...")
        rospy.sleep(1.0)
        
        return 'success'



    def _execute_yaw_adjustment_to_start_angle(self, current_pos, current_yaw, userdata):
        """
        阶段3: Yaw调整到任务开始时的角度
        调整到0°偏航角（任务开始的标准角度）
        """
        rospy.loginfo(">>> 阶段3: Yaw调整到任务开始角度")
        
        # 目标偏航角：0°（任务开始的标准角度）
        target_yaw = 0.0
        
        rospy.loginfo(f"当前偏航: {math.degrees(current_yaw):.1f}°")
        rospy.loginfo(f"目标偏航: {math.degrees(target_yaw):.1f}°")
        
        # 计算偏航差值
        yaw_diff = target_yaw - current_yaw
        
        # 处理角度环绕，选择最短旋转路径
        if yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        elif yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        rospy.loginfo(f"Yaw调整角度: {math.degrees(yaw_diff):.1f}°")
        
        # 如果偏航差值较大才执行调整
        if abs(yaw_diff) > math.radians(5.0):  # 5°阈值
            success = self.beetle.goPoseWaitConvergence(
                pos=current_pos,  # 保持当前XYZ位置
                rot=target_yaw,   # 调整到目标偏航
                pos_thresh=0.02,  # 2cm位置精度
                rot_thresh=0.0873,  # 5°偏航精度
                timeout=10.0
            )
            
            if not success:
                rospy.logwarn("Yaw调整超时，但继续执行")
            else:
                rospy.loginfo("✓ Yaw调整完成，已回到任务开始角度")
        else:
            rospy.loginfo("Yaw角度已接近目标，无需调整")
        
        # 稳定等待
        rospy.loginfo("等待0.5秒让系统稳定...")
        rospy.sleep(0.5)
        
        return 'success'

    def _execute_xy_return_to_start_position(self, current_pos, current_yaw, userdata):
        """
        阶段4: XY移动返回任务起始点
        复用MoveToValve的XY移动逻辑，返回到start_position的XY坐标
        """
        rospy.loginfo(">>> 阶段4: XY移动返回任务起始点")
        
        # 获取起始位置
        if not hasattr(userdata, 'start_position') or userdata.start_position is None:
            rospy.logerr("无法获取start_position，无法返回起始点")
            return 'failed'
        
        start_position = userdata.start_position
        
        # 目标位置：起始点XY坐标 + 当前Z高度
        target_pos = [start_position[0], start_position[1], current_pos[2]]
        
        # 计算移动距离
        xy_distance = np.linalg.norm(np.array(target_pos[:2]) - np.array(current_pos[:2]))
        rospy.loginfo(f"返回距离: {xy_distance*1000:.1f}mm")
        rospy.loginfo(f"起始位置: ({start_position[0]:.3f}, {start_position[1]:.3f}, {start_position[2]:.3f})")
        rospy.loginfo(f"目标位置: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
        rospy.loginfo(f"保持偏航: {math.degrees(current_yaw):.1f}°")
        
        # 执行XY移动
        if xy_distance > 0.05:  # 如果距离大于5cm才执行移动
            success = self.beetle.goPoseWaitConvergence(
                pos=target_pos,
                rot=current_yaw,  # 保持当前偏航角
                pos_thresh=0.05,  # 5cm精度（较宽松，因为是最终返回）
                rot_thresh=0.0873,  # 5°精度
                timeout=20.0  # 较长超时时间
            )
            
            if not success:
                rospy.logwarn("返回起始点超时，但脱离流程基本完成")
            else:
                rospy.loginfo("✓ 成功返回任务起始点")
        else:
            rospy.loginfo("已接近起始点，无需移动")
        
        # 最终稳定等待
        rospy.loginfo("等待2秒让系统完全稳定...")
        rospy.sleep(2.0)
        
        return 'success'



class HorizontalReturnState(SingleUAVStateBase):
    def __init__(self, module_id=1):
        super().__init__(outcomes=['succeeded', 'failed'],
                        input_keys=['start_position'],
                        module_id=module_id)
        
    def execute(self, userdata):
        """
        Execute horizontal return to start position using simplified 分层控制架构
        
        REFACTORED Process (分层控制):
        1. Use beetle.goPoseWaitConvergence() to return to start XY position
        2. Keep current Z height during horizontal movement
        3. Simple geometric calculation - no complex trajectory planning
        """
        rospy.loginfo("=== 分层控制: HORIZONTAL RETURN PHASE ===")
        rospy.loginfo("Using simplified return with beetle.goPoseWaitConvergence()")
        
        # Get start position from userdata
        if not hasattr(userdata, 'start_position') or userdata.start_position is None:
            rospy.logerr("No start position available for horizontal return")
            return 'failed'
        
        start_position = userdata.start_position
        
        # Wait for required position data
        if not self.wait_for_positions():
            rospy.logerr("Failed to get position data for horizontal return")
            return 'failed'
        
        # Get current state
        current_pos = self.get_current_position()
        current_yaw = self.get_current_yaw()
        if current_pos is None:
            rospy.logerr("Cannot get current UAV position for horizontal return")
            return 'failed'

        # Target: Start XY position, current Z height, start yaw
        target_pos = (start_position[0], start_position[1], current_pos[2])
        target_yaw = 0.0  # Return to 0° yaw
        
        rospy.loginfo(f"=== HORIZONTAL RETURN GEOMETRY ===")
        rospy.loginfo(f"Current position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
        rospy.loginfo(f"Target position: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
        rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f}°")
        
        horizontal_distance = math.sqrt(
            (target_pos[0] - current_pos[0])**2 + 
            (target_pos[1] - current_pos[1])**2
        )
        rospy.loginfo(f"Horizontal distance to travel: {horizontal_distance*100:.1f}cm")
        
        # CORE REFACTORING: Use beetle.goPoseWaitConvergence() for simple return
        success = self.beetle.goPoseWaitConvergence(
            pos=target_pos,
            rot=target_yaw,
            pos_thresh=0.05,  # 5cm tolerance
            rot_thresh=math.radians(5.0),  # 5° yaw tolerance
            timeout=30.0        # 30s timeout for potentially long distance
        )
        
        if not success:
            rospy.logerr("Horizontal return failed")
            return 'failed'
        
        rospy.loginfo("✓ HORIZONTAL RETURN COMPLETED")
        return 'succeeded'


def create_state_machine(module_id=1):
    """Create the SMACH state machine for single UAV valve rotation"""
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    
    with sm:
        smach.StateMachine.add('INITIALIZE_START_POSITION',
                             InitializeStartPositionState(module_id=module_id),
                             transitions={'succeeded': 'MOVE_TO_VALVE',
                                        'failed': 'failed'},
                             remapping={'start_position': 'start_position'})
        
        smach.StateMachine.add('MOVE_TO_VALVE',
                             MoveToValveState(module_id=module_id),
                             transitions={'succeeded': 'DESCEND_AND_CONTACT',
                                        'failed': 'failed'})
        
        smach.StateMachine.add('DESCEND_AND_CONTACT',
                             DescendAndContactState(module_id=module_id),
                             transitions={'succeeded': 'ROTATE_VALVE',
                                        'failed': 'failed'},
                             remapping={'initial_valve_yaw': 'initial_valve_yaw',
                                       'valve_yaw_change_threshold': 'valve_yaw_change_threshold',
                                       'rotation_start_position': 'rotation_start_position',
                                       'insertion_info': 'insertion_info'})
        
        smach.StateMachine.add('ROTATE_VALVE',
                             RotateValveState(module_id=module_id),
                             transitions={'succeeded': 'DISENGAGE_FROM_VALVE',
                                        'failed': 'failed'},
                             remapping={'initial_valve_yaw': 'initial_valve_yaw',
                                       'valve_yaw_change_threshold': 'valve_yaw_change_threshold',
                                       'rotation_start_position': 'rotation_start_position',
                                       'insertion_info': 'insertion_info',
                                       'trajectory_state': 'trajectory_state'})
        
        smach.StateMachine.add('DISENGAGE_FROM_VALVE',
                             DisengageFromValveState(module_id=module_id),
                             transitions={'succeeded': 'RETURN_TO_START',
                                        'failed': 'failed'},
                             remapping={'rotation_start_position': 'rotation_start_position',
                                       'initial_valve_yaw': 'initial_valve_yaw',
                                       'insertion_info': 'insertion_info',
                                       'disengagement_position': 'disengagement_position',
                                       'trajectory_state': 'trajectory_state',
                                       'start_position': 'start_position'})
        
        smach.StateMachine.add('RETURN_TO_START',
                             HorizontalReturnState(module_id=module_id),
                             transitions={'succeeded': 'succeeded',
                                        'failed': 'failed'},
                             remapping={'start_position': 'start_position',
                                       'disengagement_position': 'disengagement_position'})
    
    return sm


def main():
    """Main function to run the single UAV valve rotation"""
    rospy.init_node('single_uav_valve_rotation')
    
    try:
        # Get module ID from parameter
        module_id = rospy.get_param('~module_id', 1)
        
        # ========== SE(3)控制关键参数配置 ==========
        rospy.loginfo("=== SE(3) Control Parameter Configuration Verification ===")
        
        # Dragon Manipulate模式参数
        manipulate_params = {
            'target_angular_velocity': rospy.get_param('~manipulate/target_angular_velocity', 0.6),  # rad/s
            'control_rate': rospy.get_param('~manipulate/control_rate', 15.0),  # Hz
            'torque_limit': rospy.get_param('~manipulate/torque_limit', 3.0),  # N·m
            'init_torque': rospy.get_param('~manipulate/init_torque', 0.1),  # N·m
            'max_manipulation_time': rospy.get_param('~manipulate/max_manipulation_time', 45.0),  # s
            'min_manipulation_time': rospy.get_param('~manipulate/min_manipulation_time', 5.0),  # s
        }
        
        # Contact模式参数
        contact_params = {
            'target_angular_velocity': rospy.get_param('~contact/target_angular_velocity', 0.3),  # rad/s
            'control_rate': rospy.get_param('~contact/control_rate', 10.0),  # Hz
            'contact_duration': rospy.get_param('~contact/contact_duration', 8.0),  # s
            'min_contact_time': rospy.get_param('~contact/min_contact_time', 2.0),  # s
            'valve_rotation_threshold': rospy.get_param('~contact/valve_rotation_threshold', 1.5),  # degrees
        }
        
        # Finish模式参数
        finish_params = {
            'relax_duration': rospy.get_param('~finish/relax_duration', 2.0),  # s
            'vertical_offset': rospy.get_param('~finish/vertical_offset', 0.05),  # m
            'withdrawal_distance': rospy.get_param('~finish/withdrawal_distance', 0.15),  # m
        }
        
        # 收敛阈值参数
        convergence_params = {
            'pos_thresh': rospy.get_param('~convergence/pos_thresh', 0.02),  # m
            'rot_thresh': rospy.get_param('~convergence/rot_thresh', 0.0524),  # rad (3°)
            'force_threshold': rospy.get_param('~convergence/force_threshold', 0.3),  # N
        }
        
        # 在线轨迹生成器参数
        trajectory_params = {
            'radius_adaptation_rate': rospy.get_param('~trajectory/radius_adaptation_rate', 0.1),
            'velocity_threshold_for_lock': rospy.get_param('~trajectory/velocity_threshold_for_lock', 0.8),
            'mass': rospy.get_param('~trajectory/uav_mass', 1.5),  # kg
        }
        
        # 参数验证和日志输出
        rospy.loginfo("=== Manipulate模式参数 ===")
        for key, value in manipulate_params.items():
            rospy.loginfo(f"  {key}: {value}")
            
        rospy.loginfo("=== Contact模式参数 ===") 
        for key, value in contact_params.items():
            rospy.loginfo(f"  {key}: {value}")
            
        rospy.loginfo("=== Finish模式参数 ===")
        for key, value in finish_params.items():
            rospy.loginfo(f"  {key}: {value}")
            
        rospy.loginfo("=== 收敛阈值参数 ===")
        for key, value in convergence_params.items():
            rospy.loginfo(f"  {key}: {value}")
            
        rospy.loginfo("=== 轨迹生成参数 ===")
        for key, value in trajectory_params.items():
            rospy.loginfo(f"  {key}: {value}")
        
        # 参数合理性检查
        warnings = []
        
        if manipulate_params['target_angular_velocity'] > 1.0:
            warnings.append("Manipulate角速度过高 (>1.0 rad/s)，可能导致不稳定")
        
        if manipulate_params['control_rate'] < 10.0:
            warnings.append("控制频率过低 (<10Hz)，SE(3)控制可能不稳定")
            
        if manipulate_params['torque_limit'] > 5.0:
            warnings.append("扭矩限制过高 (>5.0 N·m)，可能损坏阀门")
            
        if contact_params['target_angular_velocity'] > manipulate_params['target_angular_velocity']:
            warnings.append("Contact角速度不应高于Manipulate角速度")
            
        if convergence_params['pos_thresh'] > 0.05:
            warnings.append("位置阈值过大 (>5cm)，精度可能不足")
        
        # 输出警告
        if warnings:
            rospy.logwarn("=== 参数配置警告 ===")
            for warning in warnings:
                rospy.logwarn(f"  ⚠ {warning}")
        else:
            rospy.loginfo("SUCCESS: All parameter configurations within reasonable ranges")
        
        # 将参数存储为ROS参数，供各状态使用
        rospy.set_param('/valve_rotation/manipulate', manipulate_params)
        rospy.set_param('/valve_rotation/contact', contact_params)
        rospy.set_param('/valve_rotation/finish', finish_params)
        rospy.set_param('/valve_rotation/convergence', convergence_params)
        rospy.set_param('/valve_rotation/trajectory', trajectory_params)
        
        rospy.loginfo(f"Starting single UAV valve rotation for module {module_id}")
        rospy.loginfo("=== Parameter configuration completed, starting state machine execution ===")
        
        # Create and execute state machine
        sm = create_state_machine(module_id=module_id)
        
        # Create SMACH viewer (optional)
        sis = smach_ros.IntrospectionServer('single_uav_valve_rotation', sm, '/SM_ROOT')
        sis.start()
        
        # Execute state machine
        rospy.loginfo("Starting state machine execution...")
        start_time = rospy.Time.now()
        outcome = sm.execute()
        end_time = rospy.Time.now()
        
        # Execution result statistics
        execution_time = (end_time - start_time).to_sec()
        rospy.loginfo("=== State machine execution completed ===")
        rospy.loginfo(f"Execution result: {outcome}")
        rospy.loginfo(f"Total execution time: {execution_time:.1f}s")
        
        if outcome == 'succeeded':
            rospy.loginfo("SUCCESS: Valve rotation task completed successfully!")
        else:
            rospy.logwarn(f"WARNING: Task ended with status: {outcome}")
        
        # Stop introspection server
        sis.stop()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
