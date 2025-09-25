#!/usr/bin/env python3
"""
Test script for multi-UAV formation valve rotation
This script thoroughly tests the coordinate transformations and formation control
"""

import sys
import os
import rospy
import time
import math

# Add current directory to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from n_modules_tf import NModuleTFCalculator
from valve_rotation_formation_clean import FormationAdapter
from beetle_interface import BeetleInterface

class FormationValveRotationTester:
    """Comprehensive tester for formation valve rotation system"""
    
    def __init__(self, module_ids_str="2,3"):
        rospy.init_node('formation_valve_rotation_tester', log_level=rospy.INFO)
        
        self.module_ids_str = module_ids_str
        rospy.set_param("~module_ids", module_ids_str)
        
        rospy.loginfo("=== Formation Valve Rotation Tester ===")
        rospy.loginfo(f"Testing with module_ids: {module_ids_str}")
        
    def test_coordinate_transforms(self):
        """Test the coordinate transformation system"""
        rospy.loginfo("\n=== Testing Coordinate Transformations ===")
        
        try:
            # Initialize coordinate transformer
            tf_calc = NModuleTFCalculator(self.module_ids_str)
            
            # Validate configuration
            if not tf_calc.validate_module_configuration():
                rospy.logerr("Invalid module configuration!")
                return False
            
            # Run coordinate transformation tests
            tf_calc.test_coordinate_transforms()
            
            rospy.loginfo("✓ Coordinate transformation tests passed")
            return True
            
        except Exception as e:
            rospy.logerr(f"Coordinate transformation test failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def test_formation_adapter(self):
        """Test the FormationAdapter functionality"""
        rospy.loginfo("\n=== Testing FormationAdapter ===")
        
        try:
            # Initialize formation adapter
            adapter = FormationAdapter(self.module_ids_str)
            
            rospy.loginfo("FormationAdapter initialized successfully")
            rospy.loginfo(f"Leader ID: {adapter.get_leader_id()}")
            rospy.loginfo(f"Follower IDs: {adapter.get_follower_ids()}")
            
            # Wait for formation data (with timeout)
            rospy.loginfo("Waiting for formation position data...")
            if adapter.wait_for_formation_ready(timeout=15.0):
                rospy.loginfo("✓ Formation position data received")
                
                # Test end-effector position calculation
                ee_pos = adapter.get_end_effector_position()
                if ee_pos:
                    rospy.loginfo(f"End-effector position: {ee_pos}")
                    
                    # Test command transformation
                    test_ee_target = (3.0, 0.5, 1.5)  # Example target
                    assembly_target = adapter.transform_end_effector_to_assembly_command(
                        test_ee_target, 0.0
                    )
                    rospy.loginfo(f"Transform test - EE target: {test_ee_target} -> Assembly: {assembly_target}")
                    
                    rospy.loginfo("✓ FormationAdapter tests passed")
                    return True
                else:
                    rospy.logwarn("Could not calculate end-effector position")
            else:
                rospy.logerr("Formation position data not available")
                return False
            
        except Exception as e:
            rospy.logerr(f"FormationAdapter test failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def test_beetle_interface_assembly_mode(self):
        """Test BeetleInterface in assembly mode"""
        rospy.loginfo("\n=== Testing BeetleInterface Assembly Mode ===")
        
        try:
            # Get leader ID
            tf_calc = NModuleTFCalculator(self.module_ids_str)
            leader_id = tf_calc.get_leader_id()
            
            # Initialize BeetleInterface in assembly mode
            beetle = BeetleInterface(
                module_id=leader_id,
                assembly_mode=True,
                assembly_tf_calculator=tf_calc,
                debug_view=True
            )
            
            rospy.loginfo(f"BeetleInterface initialized for leader UAV{leader_id} in assembly mode")
            
            # Test position acquisition
            rospy.sleep(2.0)  # Allow time for data
            
            assembly_pos = beetle.getAssemblyPos()
            individual_pos = beetle.getIndividualUavPos()
            ee_pos = beetle.getEndEffectorPos()
            
            if assembly_pos is not None:
                rospy.loginfo(f"Assembly CoG position: {assembly_pos}")
            else:
                rospy.logwarn("Assembly position not available")
                
            if individual_pos is not None:
                rospy.loginfo(f"Individual UAV position: {individual_pos}")
            else:
                rospy.logwarn("Individual UAV position not available")
                
            if ee_pos is not None:
                rospy.loginfo(f"End-effector position: {ee_pos}")
                rospy.loginfo("✓ BeetleInterface assembly mode tests passed")
                return True
            else:
                rospy.logwarn("End-effector position calculation failed")
                return False
            
        except Exception as e:
            rospy.logerr(f"BeetleInterface assembly mode test failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def test_command_flow(self):
        """Test the complete command flow"""
        rospy.loginfo("\n=== Testing Command Flow ===")
        
        try:
            # Initialize components
            adapter = FormationAdapter(self.module_ids_str)
            
            if not adapter.wait_for_formation_ready(timeout=10.0):
                rospy.logerr("Formation not ready for command flow test")
                return False
            
            # Test sending a simple command
            current_pos = adapter.get_assembly_position()
            if current_pos is None:
                rospy.logerr("Cannot get current assembly position")
                return False
            
            # Send a small offset command
            test_target = (current_pos[0] + 0.1, current_pos[1], current_pos[2])
            rospy.loginfo(f"Sending test command to: {test_target}")
            
            success = adapter.send_assembly_command(test_target, 0.0)
            if success:
                rospy.loginfo("✓ Command sent successfully")
                return True
            else:
                rospy.logerr("Command sending failed")
                return False
            
        except Exception as e:
            rospy.logerr(f"Command flow test failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def run_comprehensive_test(self):
        """Run all tests in sequence"""
        rospy.loginfo("Starting comprehensive formation valve rotation test...")
        
        test_results = []
        
        # Test 1: Coordinate transformations
        test_results.append(("Coordinate Transforms", self.test_coordinate_transforms()))
        
        # Test 2: FormationAdapter
        test_results.append(("FormationAdapter", self.test_formation_adapter()))
        
        # Test 3: BeetleInterface assembly mode
        test_results.append(("BeetleInterface Assembly Mode", self.test_beetle_interface_assembly_mode()))
        
        # Test 4: Command flow
        test_results.append(("Command Flow", self.test_command_flow()))
        
        # Summary
        rospy.loginfo("\n=== Test Results Summary ===")
        all_passed = True
        for test_name, result in test_results:
            status = "✓ PASS" if result else "✗ FAIL"
            rospy.loginfo(f"{test_name:30s}: {status}")
            if not result:
                all_passed = False
        
        if all_passed:
            rospy.loginfo("\n🎉 All tests PASSED! Formation valve rotation system is ready.")
        else:
            rospy.logerr("\n❌ Some tests FAILED. Please check the issues above.")
        
        return all_passed


def main():
    """Main test function"""
    # Get module configuration from parameter or default
    module_ids = rospy.get_param("~module_ids", "2,3")
    
    # Create and run tester
    tester = FormationValveRotationTester(module_ids)
    
    try:
        success = tester.run_comprehensive_test()
        
        if success:
            rospy.loginfo("\n✓ Formation valve rotation system validation complete!")
            rospy.loginfo("You can now run: roslaunch beetle valve_rotation_formation.launch")
        else:
            rospy.logerr("\n✗ System validation failed. Please fix issues before proceeding.")
            
    except KeyboardInterrupt:
        rospy.loginfo("Test interrupted by user")
    except Exception as e:
        rospy.logerr(f"Test failed with exception: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()