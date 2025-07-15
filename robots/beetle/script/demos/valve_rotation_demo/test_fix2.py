#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../valve_rotation_demo'))

try:
    print("Testing imports...")
    import rospy
    print("✓ ROS imported successfully")
    
    from aerial_robot_msgs.msg import FlightNav
    print("✓ FlightNav imported successfully")
    
    # Test FlightNav constants
    msg = FlightNav()
    msg.yaw_nav_mode = FlightNav.NO_NAVIGATION
    print("✓ FlightNav constants work correctly")
    
    from motion_controller import MotionController
    print("✓ MotionController imported successfully")
    
    from trajectory import PolynomialTrajectory
    print("✓ PolynomialTrajectory imported successfully")
    
    # Test importing the main state classes
    from valve_rotation_fang_single import SingleUAVStateBase, DescendAndContactState
    print("✓ State classes imported successfully")
    
    print("\n🎉 All imports working correctly! The attribute error fix is successful.")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
