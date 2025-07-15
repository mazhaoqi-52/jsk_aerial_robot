#!/usr/bin/env python3
"""
Test script to verify all imports work correctly
"""

import sys
import os
import traceback

# Add paths
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)
sys.path.insert(0, os.path.join(current_dir, '..', '..'))
sys.path.insert(0, os.path.join(current_dir, '..'))

def test_imports():
    """Test all required imports"""
    try:
        import rospy
        print("✓ rospy imported")
        
        from aerial_robot_msgs.msg import FlightNav
        print("✓ FlightNav imported")
        
        from motion_controller import MotionController
        print("✓ MotionController imported")
        
        from trajectory import PolynomialTrajectory
        print("✓ PolynomialTrajectory imported")
        
        from insertion_optimizer import InsertionOptimizer
        print("✓ InsertionOptimizer imported")
        
        from constrained_optimizer import ConstrainedInsertionOptimizer
        print("✓ ConstrainedInsertionOptimizer imported")
        
        print("\n✓ All imports successful!")
        return True
        
    except Exception as e:
        print(f"✗ Import failed: {e}")
        traceback.print_exc()
        return False

def test_flight_nav_constants():
    """Test FlightNav constants"""
    try:
        from aerial_robot_msgs.msg import FlightNav
        
        # Test required constants
        constants = {
            'POS_MODE': FlightNav.POS_MODE,
            'NO_NAVIGATION': FlightNav.NO_NAVIGATION,
        }
        
        for name, value in constants.items():
            print(f"✓ FlightNav.{name} = {value}")
            
        # Verify YAW_MODE doesn't exist
        try:
            yaw_mode = FlightNav.YAW_MODE
            print(f"✗ YAW_MODE exists (shouldn't exist): {yaw_mode}")
            return False
        except AttributeError:
            print("✓ YAW_MODE correctly doesn't exist")
            
        return True
        
    except Exception as e:
        print(f"✗ FlightNav constants test failed: {e}")
        traceback.print_exc()
        return False

def test_motion_controller():
    """Test MotionController creation"""
    try:
        import rospy
        rospy.init_node('test_motion_controller')
        
        from motion_controller import MotionController
        
        # Create a mock state machine
        class MockStateMachine:
            def __init__(self):
                self.pub = None
                
            def get_current_position(self):
                return (0.0, 0.0, 1.0)
                
            @property
            def current_yaw(self):
                return 0.0
        
        mock_sm = MockStateMachine()
        controller = MotionController(mock_sm)
        
        print("✓ MotionController created successfully")
        return True
        
    except Exception as e:
        print(f"✗ MotionController test failed: {e}")
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Testing all imports and functionality...")
    
    success = True
    success &= test_imports()
    success &= test_flight_nav_constants()
    success &= test_motion_controller()
    
    if success:
        print("\n🎉 All tests passed!")
    else:
        print("\n❌ Some tests failed!")
        
    print("Ready to run valve_rotation_single_uav.launch")
