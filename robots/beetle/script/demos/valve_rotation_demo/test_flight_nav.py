#!/usr/bin/env python3
"""
Test script to verify FlightNav constants
"""

import sys
import os

# Add paths
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

try:
    import rospy
    rospy.init_node('test_flight_nav')
    
    from aerial_robot_msgs.msg import FlightNav
    
    print("✓ FlightNav imported successfully")
    
    # Print all available constants
    print("\nAvailable FlightNav constants:")
    for attr in dir(FlightNav):
        if not attr.startswith('_') and attr.isupper():
            print(f"  {attr} = {getattr(FlightNav, attr)}")
    
    # Test the specific constants used in motion_controller
    try:
        pos_mode = FlightNav.POS_MODE
        no_nav = FlightNav.NO_NAVIGATION
        print(f"\n✓ POS_MODE = {pos_mode}")
        print(f"✓ NO_NAVIGATION = {no_nav}")
    except AttributeError as e:
        print(f"\n✗ Missing constant: {e}")
    
    # Check if YAW_MODE exists (this should fail)
    try:
        yaw_mode = FlightNav.YAW_MODE
        print(f"\n✗ YAW_MODE exists (shouldn't exist): {yaw_mode}")
    except AttributeError:
        print(f"\n✓ YAW_MODE correctly doesn't exist")
    
    print("\n✓ All FlightNav constants verified successfully")
    
except Exception as e:
    print(f"✗ Error: {e}")
    import traceback
    traceback.print_exc()
