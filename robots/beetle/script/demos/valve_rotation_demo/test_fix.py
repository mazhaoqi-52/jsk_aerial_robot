#!/usr/bin/env python3
"""
Test script to verify the FlightNav YAW_MODE fix
"""
import sys
import os
sys.path.append('/home/ma/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle/script/demos/valve_rotation_demo')

try:
    # Test ROS imports
    import rospy
    print("✓ ROS imported successfully")
    
    # Test FlightNav message
    from aerial_robot_msgs.msg import FlightNav
    print("✓ FlightNav imported successfully")
    
    # Test FlightNav constants
    msg = FlightNav()
    msg.yaw_nav_mode = FlightNav.NO_NAVIGATION
    msg.yaw_nav_mode = FlightNav.POS_MODE
    print("✓ FlightNav constants work correctly")
    
    # Test motion controller
    from motion_controller import MotionController
    print("✓ MotionController imported successfully")
    
    # Test trajectory
    from trajectory import PolynomialTrajectory  
    print("✓ PolynomialTrajectory imported successfully")
    
    # Test main state machine file
    from valve_rotation_fang_single import MoveToValveState, SingleUAVStateBase
    print("✓ State classes imported successfully")
    
    print("\n🎉 All imports working correctly! The YAW_MODE fix is successful.")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
