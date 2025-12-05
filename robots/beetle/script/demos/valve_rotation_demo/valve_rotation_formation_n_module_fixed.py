#!/usr/bin/env python3
"""
Formation Valve Rotation - N-Module Version (n >= 2)
Extends the dual-UAV version to support arbitrary number of UAVs

This script reuses the core logic from valve_rotation_formation_clean.py,
which already supports n modules through the FormationAdapter and 
NModuleTFCalculator classes.

Key Features:
- Supports n UAVs (n >= 2) configured via module_ids parameter
- Reuses FormationAdapter for dynamic n-module coordinate transformation
- Most calculations based on end-effector position (not individual UAV positions)
- Modular design: reuses n_modules_tf.py for coordinate transforms

Usage:
    roslaunch beetle valve_rotation_formation_n_module.launch module_ids:="1,2,3"
    roslaunch beetle valve_rotation_formation_n_module.launch module_ids:="2,3,4,5"

Parameters:
    module_ids (string): Comma-separated list of UAV IDs (e.g., "1,2,3")
                        The UAV with highest ID becomes the leader (carries end-effector)
    rotation_angle (float): Target valve rotation angle in radians (default: 1.571 = 90°)
    approach_height (float): Approach height for valve (default: 0.48m)
    real_machine (bool): Whether running on real hardware (default: false)
    simulation (bool): Whether running in simulation (default: true)

Example configurations:
    2 UAVs: module_ids:="2,3"  -> Leader: 3, Follower: 2
    3 UAVs: module_ids:="1,2,3" -> Leader: 3, Followers: 1,2
    4 UAVs: module_ids:="1,2,3,4" -> Leader: 4, Followers: 1,2,3
"""

import sys
import os
import rospy
import smach
import smach_ros

# Add current directory to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

# Import formation classes from clean version - these already support n modules
from valve_rotation_formation_clean_fixed import (
    FormationAdapter,
    FormationTakeoffState,
    FormationInitializeStartPositionState,
    FormationMoveToValveState,
    FormationRotateValveState,
    FormationDisengageFromValveState,
    FormationSingleUAVStateBase,
    WaitState  # Import WaitState for state transition delays
)


def validate_module_ids(module_ids_str):
    """
    Validate module_ids parameter
    
    Args:
        module_ids_str (str): Comma-separated module IDs
        
    Returns:
        tuple: (bool, list or str) - (is_valid, module_ids_list or error_message)
    """
    if not module_ids_str:
        return False, "module_ids parameter is empty"
    
    try:
        module_ids = [int(x.strip()) for x in module_ids_str.split(',')]
    except ValueError as e:
        return False, f"Invalid module_ids format: {e}"
    
    # Check n >= 2
    if len(module_ids) < 2:
        return False, f"At least 2 modules required, got {len(module_ids)}"
    
    # Check for duplicates
    if len(module_ids) != len(set(module_ids)):
        return False, f"Duplicate module IDs found: {module_ids}"
    
    # Check for valid IDs (positive integers)
    if any(mid <= 0 for mid in module_ids):
        return False, f"Module IDs must be positive integers: {module_ids}"
    
    return True, module_ids


def create_n_module_state_machine(module_ids_str, rotation_direction=1, rotation_angle=1.571):
    """
    Create SMACH state machine for n-module valve rotation task
    
    This function creates the same state machine structure as the dual-UAV version,
    but configured for n modules. The state classes (FormationAssembleState, etc.)
    already support n modules through FormationAdapter.
    
    Args:
        module_ids_str (str): Comma-separated module IDs
        rotation_direction (int): 1 for counter-clockwise, -1 for clockwise
        rotation_angle (float): Target valve rotation angle in radians (default: 90°)
        
    Returns:
        smach.StateMachine: Configured state machine
    """
    # Validate module IDs
    is_valid, result = validate_module_ids(module_ids_str)
    if not is_valid:
        rospy.logerr(f"Module ID validation failed: {result}")
        raise ValueError(f"Invalid module_ids: {result}")
    
    module_ids = result
    rospy.loginfo(f"Creating state machine for {len(module_ids)} modules: {module_ids}")
    rospy.loginfo(f"Leader (end-effector carrier): {max(module_ids)}")
    rospy.loginfo(f"Followers: {[mid for mid in module_ids if mid != max(module_ids)]}")
    
    # Set module_ids as ROS parameter for state classes to read
    rospy.set_param("~module_ids", module_ids_str)
    
    # Create top-level state machine
    sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed'])
    
    with sm:
        # State 1: Wait for formation ready and prepare for takeoff (ground assembly already done)
        smach.StateMachine.add(
            'TAKEOFF',
            FormationTakeoffState(),
            transitions={
                'succeeded': 'WAIT_AFTER_TAKEOFF',
                'failed': 'mission_failed'
            }
        )
        
        # Wait after takeoff
        smach.StateMachine.add(
            'WAIT_AFTER_TAKEOFF',
            WaitState(wait_time=3.0, state_name="INITIALIZE"),
            transitions={'succeeded': 'INITIALIZE'}
        )
        
        # State 2: Initialize start position
        smach.StateMachine.add(
            'INITIALIZE',
            FormationInitializeStartPositionState(),
            transitions={
                'succeeded': 'WAIT_AFTER_INITIALIZE',
                'failed': 'mission_failed'
            },
            remapping={
                'start_position': 'start_position',
                'valve_position': 'valve_position',
                'valve_yaw': 'valve_yaw'
            }
        )
        
        # Wait after initialization
        smach.StateMachine.add(
            'WAIT_AFTER_INITIALIZE',
            WaitState(wait_time=3.0, state_name="MOVE_TO_VALVE"),
            transitions={'succeeded': 'MOVE_TO_VALVE'}
        )
        
        # State 3: Move to valve (4-phase approach)
        smach.StateMachine.add(
            'MOVE_TO_VALVE',
            FormationMoveToValveState(),
            transitions={
                'succeeded': 'WAIT_AFTER_MOVE',
                'failed': 'mission_failed'
            },
            remapping={
                'valve_position': 'valve_position',
                'valve_yaw': 'valve_yaw',
                'insertion_contact_pose': 'insertion_contact_pose',
                'insertion_contact_yaw': 'insertion_contact_yaw'
            }
        )
        
        # Wait after moving to valve
        smach.StateMachine.add(
            'WAIT_AFTER_MOVE',
            WaitState(wait_time=3.0, state_name="ROTATE_VALVE"),
            transitions={'succeeded': 'ROTATE_VALVE'}
        )
        
        # State 4: Contact and rotate valve (unified state)
        smach.StateMachine.add(
            'ROTATE_VALVE',
            FormationRotateValveState(rotation_direction=rotation_direction, target_rotation=rotation_angle),
            transitions={
                'succeeded': 'WAIT_AFTER_ROTATE',
                'failed': 'mission_failed',
                'emergency': 'mission_failed'
            },
            remapping={
                'valve_position': 'valve_position',
                'valve_yaw': 'valve_yaw',
                'insertion_contact_pose': 'insertion_contact_pose',
                'insertion_contact_yaw': 'insertion_contact_yaw',
                'trajectory_state': 'trajectory_state',
                'contact_final_torque': 'contact_final_torque'
            }
        )
        
        # Wait after rotation
        smach.StateMachine.add(
            'WAIT_AFTER_ROTATE',
            WaitState(wait_time=3.0, state_name="DISENGAGE"),
            transitions={'succeeded': 'DISENGAGE'}
        )
        
        # State 5: Disengage from valve (return to safe position)
        smach.StateMachine.add(
            'DISENGAGE',
            FormationDisengageFromValveState(),
            transitions={
                'succeeded': 'mission_complete',
                'failed': 'mission_failed'
            },
            remapping={
                'valve_position': 'valve_position',
                'start_position': 'start_position',
                'trajectory_state': 'trajectory_state'
            }
        )
    
    return sm


def main():
    """
    Main entry point for n-module valve rotation task
    """
    # Initialize ROS node
    rospy.init_node('formation_valve_rotation_n_module', anonymous=False)
    
    rospy.loginfo("=" * 80)
    rospy.loginfo("Formation Valve Rotation - N-Module Version")
    rospy.loginfo("=" * 80)
    
    # Get module_ids parameter
    module_ids_str = rospy.get_param("~module_ids", "")
    
    if not module_ids_str:
        rospy.logerr("No module_ids parameter specified!")
        rospy.logerr("Usage: roslaunch beetle valve_rotation_formation_n_module.launch module_ids:=\"1,2,3\"")
        return 1
    
    # Validate and parse module IDs
    is_valid, result = validate_module_ids(module_ids_str)
    if not is_valid:
        rospy.logerr(f"Invalid module_ids parameter: {result}")
        rospy.logerr("Example: module_ids:=\"1,2,3\" for 3 UAVs")
        return 1
    
    module_ids = result
    n_modules = len(module_ids)
    
    # Log configuration
    rospy.loginfo(f"Configuration:")
    rospy.loginfo(f"  Module IDs: {module_ids}")
    rospy.loginfo(f"  Number of modules: {n_modules}")
    rospy.loginfo(f"  Leader (end-effector): UAV {max(module_ids)}")
    rospy.loginfo(f"  Followers: {sorted([mid for mid in module_ids if mid != max(module_ids)])}")
    
    # Get other parameters
    rotation_angle = rospy.get_param("~rotation_angle", 1.571)  # 90 degrees
    approach_height = rospy.get_param("~approach_height", 0.48)
    real_machine = rospy.get_param("~real_machine", False)
    simulation = rospy.get_param("~simulation", True)
    
    # Get rotation direction parameter
    direction_param = rospy.get_param("~valve_rotation_direction", "counterclockwise")
    direction_normalized = direction_param.strip().lower()
    if direction_normalized in ["clockwise", "cw"]:
        rotation_direction = -1
        direction_label = "Clockwise"
    elif direction_normalized in ["counterclockwise", "counter-clockwise", "ccw"]:
        rotation_direction = 1
        direction_label = "Counter-clockwise"
    else:
        rotation_direction = 1
        direction_label = f"Counter-clockwise (fallback for '{direction_param}')"
        rospy.logwarn(f"Unknown valve rotation direction '{direction_param}', defaulting to counter-clockwise")
    
    rospy.loginfo(f"  Rotation angle: {rotation_angle:.3f} rad ({rotation_angle * 180 / 3.14159:.1f}°)")
    rospy.loginfo(f"  Rotation direction: {direction_label}")
    rospy.loginfo(f"  Approach height: {approach_height:.3f} m")
    rospy.loginfo(f"  Real machine: {real_machine}")
    rospy.loginfo(f"  Simulation: {simulation}")
    rospy.loginfo("=" * 80)
    
    # Test coordinate transformation
    try:
        rospy.loginfo("Testing n-module coordinate transformation...")
        from n_modules_tf import NModuleTFCalculator
        
        tf_calculator = NModuleTFCalculator(module_ids_str)
        if not tf_calculator.validate_module_configuration():
            rospy.logerr("Module configuration validation failed!")
            return 1
        
        # Test transform
        test_assembly_pos = (2.5, 0.0, 1.5)
        test_assembly_yaw = 0.0
        test_ee_pos = tf_calculator.transform_assembly_to_end_effector(
            test_assembly_pos, test_assembly_yaw
        )
        
        rospy.loginfo(f"Coordinate transform test:")
        rospy.loginfo(f"  Test assembly CoG: {test_assembly_pos}")
        rospy.loginfo(f"  Calculated end-effector: {test_ee_pos}")
        rospy.loginfo(f"  Transform offset: {[test_ee_pos[i] - test_assembly_pos[i] for i in range(3)]}")
        rospy.loginfo("Coordinate transformation validated!")
        
    except Exception as e:
        rospy.logerr(f"Coordinate transformation test failed: {e}")
        return 1
    
    rospy.loginfo("=" * 80)
    
    # Create and execute state machine
    try:
        rospy.loginfo("Creating SMACH state machine...")
        sm = create_n_module_state_machine(module_ids_str, rotation_direction, rotation_angle)
        
        rospy.loginfo("State machine created successfully")
        rospy.loginfo("State sequence (with 3s waits) - Ground assembly workflow:")
        rospy.loginfo("  TAKEOFF -> [wait 3s] -> INITIALIZE -> [wait 3s] -> MOVE_TO_VALVE")
        rospy.loginfo("  -> [wait 3s] -> ROTATE_VALVE -> [wait 3s] -> DISENGAGE -> mission_complete")
        rospy.loginfo("=" * 80)
        
        # Create and start introspection server for visualization
        sis = smach_ros.IntrospectionServer('formation_valve_rotation_n_module', sm, '/SM_ROOT')
        sis.start()
        rospy.loginfo("SMACH introspection server started at /SM_ROOT")
        
        # Execute state machine
        rospy.loginfo("Starting valve rotation task execution...")
        rospy.loginfo("=" * 80)
        
        outcome = sm.execute()
        
        # Stop introspection server
        sis.stop()
        
        # Report result
        rospy.loginfo("=" * 80)
        if outcome == 'mission_complete':
            rospy.loginfo("MISSION COMPLETED SUCCESSFULLY!")
            rospy.loginfo(f"All {n_modules} UAVs coordinated to complete valve rotation")
            return 0
        else:
            rospy.logerr(f"MISSION FAILED with outcome: {outcome}")
            return 1
            
    except rospy.ROSInterruptException:
        rospy.logwarn("Task interrupted by user")
        return 1
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
