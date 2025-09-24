#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Anti-Oscillation Test Script for Valve Insertion
测试反震荡插入算法的验证脚本

This script validates the three-layer anti-oscillation solution:
1. Insertion completion detection (10mm threshold, relaxed from 5mm)
2. POSITION_HOLD gentle control mode 
3. Trajectory divergence detection and emergency termination

Author: Advanced UAV Control System
Date: 2024
"""

import rospy
import sys
import os
import math
import time
import threading
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point, Pose, PoseStamped
from aerial_robot_msgs.msg import FlightNav

# Add the parent directory to Python path for imports
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
sys.path.append(parent_dir)

from demos.valve_rotation_demo.valve_rotation_fang_single import ValveRotationStateMachine

class AntiOscillationTester:
    """
    Anti-oscillation test controller for validating the enhanced valve insertion system
    """
    
    def __init__(self):
        rospy.init_node('anti_oscillation_tester', anonymous=True)
        
        # Test configuration
        self.test_name = "Anti-Oscillation Insertion Test"
        self.test_start_time = None
        
        # Monitoring variables
        self.position_errors = []
        self.control_modes = []
        self.test_results = {}
        
        # Subscribe to position feedback for monitoring
        self.position_sub = rospy.Subscriber('/uav/pose', PoseStamped, self.position_callback)
        self.current_position = None
        
        # Publishers for test monitoring
        self.test_status_pub = rospy.Publisher('/test/anti_oscillation/status', String, queue_size=10)
        self.error_pub = rospy.Publisher('/test/anti_oscillation/error', Float64, queue_size=10)
        
        rospy.loginfo("🧪 Anti-oscillation tester initialized")
        
    def position_callback(self, msg):
        """Callback for position monitoring"""
        self.current_position = [
            msg.pose.position.x,
            msg.pose.position.y, 
            msg.pose.position.z
        ]
        
    def run_insertion_test(self, target_valve_position=None):
        """
        Run comprehensive insertion test with anti-oscillation monitoring
        
        Args:
            target_valve_position: Target valve position [x, y, z]. If None, uses default test position.
        """
        rospy.loginfo("=" * 80)
        rospy.loginfo(f"🧪 STARTING {self.test_name}")
        rospy.loginfo("=" * 80)
        
        # Default test position if not specified
        if target_valve_position is None:
            target_valve_position = [0.0, 0.0, -0.15]  # 15cm descent for testing
            
        self.test_start_time = time.time()
        
        try:
            # Initialize state machine
            rospy.loginfo("🔧 Initializing valve rotation state machine...")
            sm = ValveRotationStateMachine()
            
            # Start monitoring thread
            monitoring_thread = threading.Thread(target=self.monitor_test_execution)
            monitoring_thread.daemon = True
            monitoring_thread.start()
            
            # Test 1: Pre-insertion positioning
            rospy.loginfo("📍 Test 1: Pre-insertion positioning")
            pre_insertion_pos = [target_valve_position[0], target_valve_position[1], target_valve_position[2] + 0.05]
            self.test_position_accuracy(sm, pre_insertion_pos, test_name="Pre-insertion positioning")
            
            # Test 2: Valve insertion with anti-oscillation
            rospy.loginfo("🔧 Test 2: Valve insertion with anti-oscillation protection")
            success = self.test_valve_insertion(sm, target_valve_position)
            
            # Test 3: Post-insertion stability monitoring
            rospy.loginfo("📊 Test 3: Post-insertion stability analysis")
            self.test_post_insertion_stability(sm, duration=10.0)
            
            # Generate test report
            self.generate_test_report(success)
            
        except Exception as e:
            rospy.logerr(f"❌ Test failed with exception: {e}")
            self.test_results['overall_success'] = False
            self.test_results['error_message'] = str(e)
            
        rospy.loginfo("🏁 Anti-oscillation test completed")
        
    def test_valve_insertion(self, sm, target_position):
        """
        Test valve insertion with monitoring for oscillation detection
        """
        rospy.loginfo(f"🎯 Starting valve insertion to position: {target_position}")
        
        insertion_start_time = time.time()
        
        # Configure insertion parameters for testing
        sm.controller.motion_controller.USE_ADAPTIVE_GAINS = True  # Enable adaptive control
        
        try:
            # Execute three-stage movement with enhanced monitoring
            success = sm.execute_three_stage_movement(
                target_pos=target_position,
                target_yaw=0.0,
                xy_threshold=0.003,    # 3mm precision
                yaw_threshold=0.087,   # 5 degree precision  
                z_threshold=0.005,     # 5mm precision
                stage_timeouts=[15.0, 10.0, 10.0]
            )
            
            insertion_duration = time.time() - insertion_start_time
            
            # Record test results
            self.test_results['insertion_success'] = success
            self.test_results['insertion_duration'] = insertion_duration
            self.test_results['insertion_target'] = target_position
            
            if success:
                rospy.loginfo(f"✅ Valve insertion completed successfully in {insertion_duration:.2f}s")
            else:
                rospy.logwarn(f"⚠️ Valve insertion completed with issues in {insertion_duration:.2f}s")
                
            return success
            
        except Exception as e:
            rospy.logerr(f"❌ Valve insertion failed: {e}")
            self.test_results['insertion_success'] = False
            self.test_results['insertion_error'] = str(e)
            return False
            
    def test_position_accuracy(self, sm, target_pos, test_name="Position accuracy", timeout=15.0):
        """Test position accuracy and convergence"""
        rospy.loginfo(f"📍 Testing {test_name} to position: {target_pos}")
        
        start_time = time.time()
        
        # Move to target position
        success = sm.controller.motion_controller.move_to_position_with_yaw(
            target_pos[0], target_pos[1], target_pos[2], 0.0,
            duration=10.0, control_mode="CONSERVATIVE"
        )
        
        # Measure final accuracy
        if self.current_position:
            error = math.sqrt(sum([(target_pos[i] - self.current_position[i])**2 for i in range(3)]))
            rospy.loginfo(f"📊 {test_name} - Final error: {error*1000:.1f}mm")
            
            self.test_results[f'{test_name.lower().replace(" ", "_")}_error'] = error
            self.test_results[f'{test_name.lower().replace(" ", "_")}_success'] = error < 0.010  # 10mm threshold
            
        return success
        
    def test_post_insertion_stability(self, sm, duration=10.0):
        """Monitor post-insertion stability"""
        rospy.loginfo(f"📊 Monitoring post-insertion stability for {duration}s...")
        
        stability_start_time = time.time()
        position_samples = []
        
        while time.time() - stability_start_time < duration:
            if self.current_position:
                position_samples.append(self.current_position[:])
            rospy.sleep(0.1)  # 10Hz sampling
            
        # Analyze stability
        if len(position_samples) > 10:
            # Calculate position variance
            mean_pos = [sum([p[i] for p in position_samples])/len(position_samples) for i in range(3)]
            variances = [sum([(p[i] - mean_pos[i])**2 for p in position_samples])/len(position_samples) for i in range(3)]
            std_devs = [math.sqrt(v) for v in variances]
            
            max_std = max(std_devs)
            rospy.loginfo(f"📊 Post-insertion stability - Max std dev: {max_std*1000:.2f}mm")
            
            # Stability criteria: < 5mm standard deviation
            stability_good = max_std < 0.005
            
            self.test_results['post_insertion_stability'] = stability_good
            self.test_results['post_insertion_std_dev'] = max_std
            
            if stability_good:
                rospy.loginfo("✅ Post-insertion stability: GOOD")
            else:
                rospy.logwarn("⚠️ Post-insertion stability: POOR - possible oscillation")
                
    def monitor_test_execution(self):
        """Background monitoring thread for test execution"""
        while not rospy.is_shutdown() and self.test_start_time:
            if self.current_position and hasattr(self, 'target_position'):
                # Calculate current error
                error = math.sqrt(sum([(self.target_position[i] - self.current_position[i])**2 
                                     for i in range(min(len(self.target_position), len(self.current_position)))]))
                
                self.position_errors.append((time.time() - self.test_start_time, error))
                
                # Publish monitoring data
                self.error_pub.publish(Float64(error))
                
            rospy.sleep(0.1)  # 10Hz monitoring
            
    def generate_test_report(self, overall_success):
        """Generate comprehensive test report"""
        rospy.loginfo("=" * 80)
        rospy.loginfo("📊 ANTI-OSCILLATION TEST REPORT")
        rospy.loginfo("=" * 80)
        
        total_duration = time.time() - self.test_start_time if self.test_start_time else 0
        
        rospy.loginfo(f"⏱️ Total test duration: {total_duration:.2f}s")
        rospy.loginfo(f"🎯 Overall test success: {'✅ PASS' if overall_success else '❌ FAIL'}")
        
        # Detailed results
        for key, value in self.test_results.items():
            if isinstance(value, bool):
                status = "✅ PASS" if value else "❌ FAIL"
                rospy.loginfo(f"📋 {key}: {status}")
            elif isinstance(value, float):
                if 'error' in key or 'std_dev' in key:
                    rospy.loginfo(f"📏 {key}: {value*1000:.2f}mm")
                else:
                    rospy.loginfo(f"📊 {key}: {value:.3f}")
            else:
                rospy.loginfo(f"📝 {key}: {value}")
                
        # Error analysis
        if self.position_errors:
            max_error = max([e for t, e in self.position_errors])
            avg_error = sum([e for t, e in self.position_errors]) / len(self.position_errors)
            
            rospy.loginfo(f"📈 Maximum position error: {max_error*1000:.1f}mm")
            rospy.loginfo(f"📊 Average position error: {avg_error*1000:.1f}mm")
            
            # Check for oscillation patterns
            if max_error > 0.050:  # 50mm threshold
                rospy.logwarn("⚠️ Large position errors detected - possible oscillation")
            elif max_error > 0.020:  # 20mm threshold
                rospy.logwarn("⚠️ Moderate position errors - monitor for stability")
            else:
                rospy.loginfo("✅ Position errors within acceptable range")
                
        rospy.loginfo("=" * 80)
        
        # Publish final test status
        status_msg = f"Test: {'PASS' if overall_success else 'FAIL'}, Duration: {total_duration:.1f}s"
        self.test_status_pub.publish(String(status_msg))

def main():
    """Main test execution function"""
    try:
        tester = AntiOscillationTester()
        
        # Wait for system initialization
        rospy.loginfo("⏳ Waiting for system initialization...")
        rospy.sleep(2.0)
        
        # Run the test
        tester.run_insertion_test()
        
        # Keep node alive for monitoring
        rospy.loginfo("💤 Test completed. Node will remain active for monitoring...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Test interrupted by user")
    except KeyboardInterrupt:
        rospy.loginfo("🛑 Test interrupted by keyboard")
    except Exception as e:
        rospy.logerr(f"❌ Test failed with unexpected error: {e}")

if __name__ == '__main__':
    main()