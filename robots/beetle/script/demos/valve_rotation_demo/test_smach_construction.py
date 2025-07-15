#!/usr/bin/env python
"""
Test SMACH state machine construction
"""
import rospy
import smach

# Add current directory to path
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_smach_states():
    """Test if SMACH states can be constructed without errors"""
    
    # Mock rospy functions for testing
    class MockROSPY:
        def get_param(self, name, default):
            return default
        def Publisher(self, *args, **kwargs):
            return None
        def Subscriber(self, *args, **kwargs):
            return None
        def loginfo(self, msg):
            print(f"[INFO] {msg}")
    
    # Replace rospy temporarily
    import valve_rotation_fang_single
    valve_rotation_fang_single.rospy = MockROSPY()
    
    try:
        # Test SingleUAVStateBase construction
        print("Testing SingleUAVStateBase construction...")
        from valve_rotation_fang_single import SingleUAVStateBase
        
        # Test without input_keys
        state1 = SingleUAVStateBase(outcomes=['succeeded', 'failed'], module_id=1)
        print("  ✓ SingleUAVStateBase without input_keys: OK")
        
        # Test with input_keys
        state2 = SingleUAVStateBase(outcomes=['succeeded', 'failed'], 
                                   input_keys=['start_position'], module_id=1)
        print("  ✓ SingleUAVStateBase with input_keys: OK")
        
        # Test ReturnToStartState
        print("Testing ReturnToStartState construction...")
        from valve_rotation_fang_single import ReturnToStartState
        return_state = ReturnToStartState(module_id=1)
        print("  ✓ ReturnToStartState: OK")
        
        # Test EmergencyState
        print("Testing EmergencyState construction...")
        from valve_rotation_fang_single import EmergencyState
        emergency_state = EmergencyState(module_id=1)
        print("  ✓ EmergencyState: OK")
        
        print("\n✅ All SMACH states constructed successfully!")
        return True
        
    except Exception as e:
        print(f"\n❌ SMACH state construction failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_smach_states()
    sys.exit(0 if success else 1)
