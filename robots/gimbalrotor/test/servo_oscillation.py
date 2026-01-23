#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Servo Oscillation Script
This script controls servos 0-5 to oscillate between positions:
2047 -> 1947 -> 2047 -> 2147 -> 2047 (8 second cycle)
"""

import rospy
from spinal.msg import ServoControlCmd

def main():
    rospy.init_node('servo_oscillation', anonymous=True)
    
    # Publisher for servo control
    pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=10)
    
    # Wait for publisher to be ready
    rospy.sleep(1.0)
    
    # Servo indices (0 to 5)
    servo_indices = [0, 1, 2, 3, 4, 5]
    
    # Position sequence: 2047 -> 1747 -> 2047 -> 2347 -> 2047
    # Each step takes 1 second, total cycle = 4 seconds
    positions = [2047, 1747, 2047, 2347]
    step_duration = 1.0  # seconds per step
    
    rate = rospy.Rate(10)  # 10 Hz for smooth publishing
    
    rospy.loginfo("Starting servo oscillation test...")
    rospy.loginfo("Servos 0-5 will oscillate: 2047 -> 1747 -> 2047 -> 2347 -> 2047")
    rospy.loginfo("Cycle period: 4 seconds")
    rospy.loginfo("Press Ctrl+C to stop")
    
    step_index = 0
    last_step_time = rospy.Time.now()
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        
        # Check if it's time to move to next position
        if (current_time - last_step_time).to_sec() >= step_duration:
            step_index = (step_index + 1) % len(positions)
            last_step_time = current_time
            rospy.loginfo("Moving to position: %d", positions[step_index])
        
        # Create and publish servo command
        msg = ServoControlCmd()
        msg.index = servo_indices
        msg.angles = [positions[step_index]] * len(servo_indices)
        
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Servo oscillation test stopped.")
