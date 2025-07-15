#!/usr/bin/env python
"""
Simplified RotateValveState that uses UnifiedMotionController for alignment control
"""
import math
import time
import threading
import rospy
from unified_motion_controller import UnifiedMotionController
from trajectory import ValveRotationTrajectory


class SimplifiedRotateValveState:
    """Simplified valve rotation state that delegates alignment control to UnifiedMotionController"""
    
    def __init__(self, state_machine, rotation_angle=math.pi/2, rotation_duration=8.0):
        """
        Initialize SimplifiedRotateValveState
        
        Args:
            state_machine: Reference to the state machine
            rotation_angle: Angle to rotate in radians
            rotation_duration: Duration of rotation in seconds
        """
        self.sm = state_machine
        self.rotation_angle = rotation_angle
        self.rotation_duration = rotation_duration
        
        # Initialize enhanced motion controller
        self.motion_controller = UnifiedMotionController(state_machine)
        
        # Emergency detection variables
        self.emergency_stop = threading.Event()
        self.last_position = None
        self.last_yaw = 0.0
        self.stuck_start_time = None
        
        # Emergency detection parameters
        self.stuck_threshold = rospy.get_param("~stuck_threshold", 25.0)
        self.movement_threshold = rospy.get_param("~movement_threshold", 0.02)
        self.yaw_threshold = rospy.get_param("~yaw_threshold", 0.1)
        
        rospy.loginfo("=== SIMPLIFIED VALVE ROTATION STATE ===")
        rospy.loginfo("Using UnifiedMotionController for alignment control")
        rospy.loginfo(f"Rotation angle: {self.rotation_angle:.3f}rad ({self.rotation_angle*180/math.pi:.1f}°)")
        rospy.loginfo(f"Rotation duration: {self.rotation_duration:.1f}s")
    
    def check_rotation_emergency(self):
        """Check if UAV is stuck during rotation"""
        current_pos = self.sm.get_current_position()
        if current_pos is None:
            return False
        
        # Calculate movement since last check
        if self.last_position is not None:
            movement = math.sqrt(
                (current_pos[0] - self.last_position[0])**2 + 
                (current_pos[1] - self.last_position[1])**2 + 
                (current_pos[2] - self.last_position[2])**2
            )
            
            yaw_movement = abs(self.sm.normalize_angle(self.sm.current_yaw - self.last_yaw))
            
            if movement < self.movement_threshold and yaw_movement < self.yaw_threshold:
                if self.stuck_start_time is None:
                    self.stuck_start_time = time.time()
                    rospy.loginfo("UAV appears stuck during rotation - starting timer")
                else:
                    stuck_duration = time.time() - self.stuck_start_time
                    if stuck_duration > self.stuck_threshold:
                        rospy.logwarn(f"UAV stuck for {stuck_duration:.1f}s during rotation")
                        return True
            else:
                if self.stuck_start_time is not None:
                    rospy.loginfo("UAV movement detected - resetting stuck timer")
                self.stuck_start_time = None
        
        # Update last position
        self.last_position = current_pos
        self.last_yaw = self.sm.current_yaw
        
        return False
    
    def execute(self, userdata):
        """Execute valve rotation with enhanced alignment control"""
        rospy.loginfo("Starting enhanced valve rotation...")
        
        # Get start position for emergency return
        start_position = userdata.start_position
        if start_position is None:
            rospy.logerr("No start position available for emergency return")
            return 'failed'
        
        # Wait for position data
        if not self.sm.wait_for_positions():
            return 'failed'
        
        current_pos = self.sm.get_current_position()
        if current_pos is None:
            rospy.logerr("Failed to get current position")
            return 'failed'
        
        # Initialize rotation tracking
        self.last_position = current_pos
        self.last_yaw = self.sm.current_yaw
        self.stuck_start_time = None
        
        # Phase 1: Pre-rotation alignment
        rospy.loginfo("=== PHASE 1: PRE-ROTATION ALIGNMENT ===")
        alignment_success = self.motion_controller.execute_pre_rotation_alignment(
            self.sm.valve_pos, timeout=10.0)
        
        if not alignment_success:
            rospy.logwarn("Pre-rotation alignment incomplete - proceeding with caution")
        
        # Update position after alignment
        aligned_pos = self.sm.get_current_position()
        if aligned_pos is not None:
            current_pos = aligned_pos
        
        # Phase 2: Create and execute rotation trajectory
        rospy.loginfo("=== PHASE 2: ALIGNMENT-CONTROLLED ROTATION ===")
        
        # Create rotation trajectory
        rotation_traj = ValveRotationTrajectory(
            rotation_duration=self.rotation_duration,
            rotation_angle=self.rotation_angle,
            valve_center=self.sm.valve_pos,
            valve_radius=0.1225,
            valve_yaw=self.sm.valve_yaw
        )
        
        rotation_traj.set_start_position(current_pos, self.sm.current_yaw)
        
        # Execute rotation with enhanced motion controller
        try:
            # Start emergency monitoring thread
            emergency_thread = threading.Thread(target=self._emergency_monitor_thread)
            emergency_thread.daemon = True
            emergency_thread.start()
            
            # Execute alignment-controlled rotation
            rotation_stats = self.motion_controller.execute_alignment_controlled_rotation(
                rotation_traj, self.sm.valve_pos)
            
            # Stop emergency monitoring
            self.emergency_stop.set()
            
            # Check if emergency was triggered
            if self.emergency_stop.is_set():
                rospy.logwarn("Emergency detected during rotation")
                if self.sm.emergency_ascent_and_return(start_position):
                    return 'emergency'
                else:
                    return 'failed'
            
            # Log rotation statistics
            rospy.loginfo("=== ROTATION STATISTICS ===")
            rospy.loginfo(f"Execution time: {rotation_stats.get('execution_time', 0):.1f}s")
            rospy.loginfo(f"Alignment violations: {rotation_stats.get('alignment_violations', 0)}")
            rospy.loginfo(f"Corrections applied: {rotation_stats.get('corrections_applied', 0)}")
            rospy.loginfo(f"Max position error: {rotation_stats.get('max_position_error', 0):.3f}m")
            rospy.loginfo(f"Max yaw error: {rotation_stats.get('max_yaw_error', 0):.3f}rad")
            
            rospy.loginfo("Enhanced valve rotation completed successfully")
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f"Error during rotation execution: {e}")
            self.emergency_stop.set()
            return 'failed'
    
    def _emergency_monitor_thread(self):
        """Background thread for emergency monitoring"""
        rate = rospy.Rate(2)  # Check every 0.5 seconds
        
        while not self.emergency_stop.is_set() and not rospy.is_shutdown():
            if self.check_rotation_emergency():
                rospy.logwarn("Emergency condition detected - stopping rotation")
                self.emergency_stop.set()
                break
            
            rate.sleep()
